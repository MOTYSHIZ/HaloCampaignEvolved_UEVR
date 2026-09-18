#include "core/reload/ReloadEngine.hpp"
#include "core/config/CfgRead.hpp"
#include "core/PalettePose.hpp"   // placement_aim_fix

#include "Gesture.hpp"
#include "TwoHandAim.hpp"
#include "Holster.hpp"

#include "core/HiddenReload.hpp"   // g_wristhud_hide_cradle: the hidden reload hides the ammo cradle
#include "Config.hpp"
#include "core/FireInput.hpp"      // g_ft_fire_at: the reload family's fire timing
#include "Markers.hpp"            // the magwell insert marker rides the holster marker machinery
#include "core/MarkerFaces.hpp"
#include "core/Services.hpp"
#include "core/WeaponObject.hpp"   // the held weapon's object and the slide node
#include "core/host/GestureState.hpp"
#include "core/host/HolsterState.hpp"
#include "Math.hpp"
#include "MotionAimControl.hpp"
#include "Rig.hpp"                // fp_weapon_actor -- the mag hide walks its components
#include "UeObject.hpp"
#include "WeaponCalib.hpp"
#include <intrin.h>

#include <string>
#include <vector>
#include <algorithm>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <cwctype>

using uevr::API;

namespace halo {

std::atomic<bool> g_slide_zone_hot{false};   // read by TwoHand: the off hand is inside a live rack zone
std::atomic<float> g_reload_slide_t{-1.0f};
std::atomic<float> g_reload_slide_x{0.0f}, g_reload_slide_y{0.0f}, g_reload_slide_z{0.0f};
std::atomic<float> g_reload_slide_pitch{0.0f}, g_reload_slide_yaw{0.0f}, g_reload_slide_roll{0.0f};
std::atomic<bool>  g_reload_slide_rot_valid{false};
std::atomic<long long> g_reload_pose_hold_until{0};

bool reload_manual_available() { return service_active(SVC_MANUAL_RELOAD_AVAILABLE); }
bool reload_rack_available()   { return service_active(SVC_RACK_AVAILABLE); }
bool reload_engine_active()    { return service_active(SVC_MANUAL_RELOAD_AVAILABLE | SVC_RACK_AVAILABLE); }

void reload_pose_hold(int ms) {
    if (ms <= 0) { g_reload_pose_hold_until.store(0, std::memory_order_release); return; }
    const long long nowt = std::chrono::steady_clock::now().time_since_epoch().count();
    const long long span = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                               std::chrono::milliseconds(ms)).count();
    g_reload_pose_hold_until.store(nowt + span, std::memory_order_release);
}

namespace {

// ---- THE AUTHOR'S RELOAD MACHINE, through the bridge (core/host/GestureState.hpp): every read and
// write lands on Gesture.cpp's own state, under the names the engine code has always used.
using clock_t_ = std::chrono::steady_clock;   // Gesture.cpp's clock, the same type
ReloadState& s_reload = *host::g_gesture_state.reload;
long long (*const now_ticks)() = host::g_gesture_state.now_ticks;
long long (*const ms_to_ticks)(int ms) = host::g_gesture_state.ms_to_ticks;
void (*const set_state)(ReloadState next, const char* why) = host::g_gesture_state.set_state;
const char* (*const state_name)(ReloadState s) = host::g_gesture_state.state_name;

// ---- THE ENGINE'S OWN RELOAD STATE
// The weapon whose magazine the in-flight reload belongs to (weapon_key at the Idle exit).
std::string s_reload_weapon;
// A MAGAZINE THAT IS OUT STAYS OUT ACROSS A WEAPON SWAP (from the headset, 2026-09-04): drop gun A's mag,
// swap to B, fire, swap back -- A still needs the whole reload. What the machine knew about A
// when it left the hand, by weapon key, restored when A returns.
struct WpnMem { std::string key; bool mag_out; int chamber_left; bool empty; bool lock_pending; bool true_empty; };
WpnMem s_wpn_mem[8];
WpnMem* wpn_mem_find(const std::string& key) { for (auto& m : s_wpn_mem) if (!m.key.empty() && m.key == key) return &m; return nullptr; }
WpnMem* wpn_mem_slot(const std::string& key) { if (auto* m = wpn_mem_find(key)) return m; for (auto& m : s_wpn_mem) if (m.key.empty()) return &m; return &s_wpn_mem[0]; }
void wpn_mem_clear(const std::string& key) { if (auto* m = wpn_mem_find(key)) *m = WpnMem{}; }
bool s_restoring_mag_out = false;   // set_state must not drop a second magazine on a restore
static float            s_grab_y = 0.0f;         // left-hand height (VR y) at the belt grab
bool s_reload_on = true;   // one release on the off edge (at a boot with reloadvr 0 there is nothing to release)

// ================================================================ THE SNAPSHOT (zonesnapshot)
//
// ONE INSTANT, ONE SEQUENCE NUMBER, EVERY DECISION. Doctrine and the measured numbers are in
// ConfigFields.inl on zonesnapshot. Everything the rack zone, the magazine well and their two
// markers need is sampled here, once per engine tick, before the reload state machine runs; the
// tests then read this and nothing else, so no decision can mix two moments. Declared before the
// engine's own fragments because both of them consume it (the well marker in
// Engine_mag_anim_sound.inl, the rack in Engine_rack.inl).
struct ZoneSnap {
    unsigned seq = 0;            // which snapshot this is: a consumer logs it, never re-samples
    bool  poses_ok = false;      // the head and both hands resolved: the tests may use this snapshot
    Vec3  head{};                // HMD, room metres
    Vec3  aim{};  Quat aim_q{};  // the AIMING hand, aim pose (the frame the weapon is placed on)
    Quat  aim_fix{};             // the placement's own fix on that pose, resolved once here
    Vec3  r{}, u{}, f{};         // the aim-fixed frame: right, up, forward
    Vec3  off{};  Quat off_q{};  // the OTHER hand, grip pose (the hand that racks and fetches)
    Vec3  cam{};  bool cam_ok = false;     // the game camera at this instant
    Vec3  part{}; bool part_ok = false;    // the rack part's world centre, read IN this snapshot
    void* part_key = nullptr;              // which component that centre came from (the learn key)
    Vec3  well{}; bool well_ok = false;    // the magazine component's world location, read IN this snapshot
    Vec3  well_rot{}; bool well_rot_ok = false;
    bool  wpn_ok = false; Quat wpn_q{};    // the DRAWN weapon's rotation as the placement published it (mode 2)
    float cam_travel = 0.0f;     // the camera's travel since the previous snapshot, metres (mode 3)
};
ZoneSnap s_snap;
// Whether the snapshot rule is in force. Off = every consumer keeps its inherited newest-value reads.
bool zone_snapshot_on() { return g_cfg.zone_snapshot != 0; }
// Filled by the fragments below (each owns the objects it reads).
bool sl_part_world_now(Vec3* out, void** key);
bool mag_well_world_now(Vec3* loc, Vec3* rot, bool* rot_ok);
// The room<->world pair the gesture tests share, declared here because the well marker (the
// fragment above) draws through it and it is defined in the rack fragment below.
Vec3 reload_hand_world(const Vec3& hand_room, const Vec3& head_room);
Vec3 reload_world_room(const Vec3& world, const Vec3& head_room);

#include "core/reload/Engine_mag_anim_sound.inl"   // the weapon's own magazine, the well marker, the holds, the sound

#include "core/reload/Engine_rack.inl"   // the rack, the montage, slide fire, chamber and phantom, the parts

#include "core/reload/Engine_ammo_dump_magdrop.inl"   // the rounds probe and the dropped magazine

// The belt point for the weapon in hand: a reloadmagoffw entry if one matches, the fork's own
// reloadmagbelt otherwise (the author's reloadmagoff stays his, for his own mag marker fallback). Both the rendered mag and the grab zone read this, so what you see and
// what you reach for stay the same point.
Vec3 mag_belt_point() {
    Vec3 mo{g_cfg.reload_mag_belt[0], g_cfg.reload_mag_belt[1], g_cfg.reload_mag_belt[2]};
    const std::string key = weapon_key();
    if (key.empty() || g_cfg.reload_mag_off_w[0] == 0) return mo;
    std::string lk = key; for (auto& ch : lk) ch = (char)tolower((unsigned char)ch);
    const std::string tbl = g_cfg.reload_mag_off_w;
    size_t pos = 0;
    while (pos <= tbl.size()) {
        size_t comma = tbl.find(',', pos); if (comma == std::string::npos) comma = tbl.size();
        std::string ent = tbl.substr(pos, comma - pos);
        while (!ent.empty() && (unsigned char)ent.back()  <= ' ') ent.pop_back();
        while (!ent.empty() && (unsigned char)ent.front() <= ' ') ent.erase(ent.begin());
        const size_t colon = ent.find(':');
        if (colon != std::string::npos && colon > 0) {
            std::string name = ent.substr(0, colon);
            for (auto& ch : name) ch = (char)tolower((unsigned char)ch);
            if (lk.find(name) != std::string::npos) {
                float x = mo.x, y = mo.y, z = mo.z;
                if (sscanf_s(ent.c_str() + colon + 1, "%f/%f/%f", &x, &y, &z) == 3) mo = Vec3{x, y, z};
                return mo;
            }
        }
        if (comma >= tbl.size()) break;
        pos = comma + 1;
    }
    return mo;
}

} // namespace

#include "core/reload/Engine_hidden_display.inl"   // the hidden reload's display and gesture_render_tick()

#include "core/reload/Engine_reload_state.inl"   // the per-weapon reload state and reload_release_all()

// ================================================================ THE HOOKS

void reload_engine_tick_begin(float dt, bool active) {
    s_gest_dt = dt;
    // The fork's manual reload family (reloadvr / slidevr, experimental, default off). Every tick below
    // is idle-cheap on its own, but several resolve the weapon actor by reflection, so with both
    // masters off none of them runs and this function is the author's again. The two restore
    // windows (anim rate, state hold) stay outside: a window armed before a switch-off must close.
    reload_anim_rate_tick();
    reload_state_hold_tick();
    if (!active) return;
    // THE SNAPSHOT, FIRST (zonesnapshot). Before the reload state machine, before the rack, before
    // the holster's belt magazine: one instant, taken once, and every decision in this tick reads
    // it instead of sampling for itself. This is also the reason it lives at the top of the tick
    // rather than inside any one test -- a test that took its own snapshot would still disagree
    // with the next test's.
    zone_snapshot_take();
    g_ak_engine_on.store(true, std::memory_order_relaxed);
    mag_dump_probe();
    anim_dump_tick();
    anim_vars_tick();
    audio_dump_tick();
    ak_mute_tick();
    anim_var_set_tick();
    anim_seq_set_tick();
    wpn_ammo_dump_tick();
    ammo_seq_tick();
    mag_drop_tick();
    // The held weapon's Blam object index, EVERY TICK, unconditionally.
    //
    // It used to publish every 30 ticks unless the reload state tracker was running, and that
    // gate is now wrong: this branch retired the roundsguard switch and made the guard permanent,
    // so rounds_field() compares the published datum against the pointer's own datum on every
    // read (Engine_rack.inl, rounds_field). A datum that is up to 30 ticks (~1 s) stale after a
    // weapon swap makes that comparison fail on a perfectly good pointer, and the guard then
    // returns null for that whole second: the phantom round, the dry stop, the everyshot lock and
    // the empty judge all go blind on the gun the player just drew. reloadstate is a different
    // feature and has no business deciding it. One property read a tick is what the permanent
    // guard costs, and it is what the play build pays with roundsguard at its own default.
    {
        int32_t idx = -1;
        if (auto* wpn = fp_weapon_actor()) {
            auto** pc = wpn->get_property_data<API::UObject*>(L"BlamObjectSynchronization");
            if (pc != nullptr && !IsBadReadPtr(pc, sizeof(void*)) && *pc != nullptr && !IsBadReadPtr(*pc, sizeof(void*))) {
                auto* p = (*pc)->get_property_data<int32_t>(L"BlamObjectIndex");
                if (p != nullptr && !IsBadReadPtr(p, sizeof(int32_t))) idx = *p;
            }
        }
        g_wpn_obj_index.store(idx, std::memory_order_relaxed);
    }
}

void reload_engine_disabled() {
    if (s_reload_on) { s_reload_on = false; reload_release_all("manual reload switched off"); }
}

void reload_engine_update_begin() {
    s_reload_on = true;
    // The well marker and the slide exist only while the magazine is in hand; every other state
    // hides the one and resets the other.
    if (s_reload != ReloadState::MagHeld) {
        reload_well_marker_update(false, Vec3{});
        reload_slide_reset();
    }
}

void reload_engine_timed_out() {
    // The slide bookkeeping goes the way the weapon-swap cancel below clears it, so a timed-out
    // reload does not leave a lock pending on a gun the sim never emptied.
    s_sl_lock_pending = false; s_sl_reload_due = false; s_sl_pressed_early = false; s_sl_locked_back = false; s_sl_press_pending = false; s_sl_press_due_at = 0;
}

void reload_engine_swap_cancel() {
    // ---- A WEAPON CHANGE CANCELS THE RELOAD IN FLIGHT. The gesture is a state machine and
    // nothing told it the weapon changed: pull the mag on gun A, swap to gun B, and the fire
    // suppression follows the PLAYER, not the gun -- the player could not shoot the fresh weapon
    // until he performed a gesture that belonged to the previous one. The magazine you pulled was
    // gun A's; gun B arrives in whatever state the game has it. So the reload records which
    // weapon's mag came out, and the moment a DIFFERENT non-empty weapon is in hand the state
    // returns to Idle. Non-empty on both sides on purpose: a transient empty key (weapon lowered
    // for a frame, swap animation) must not cancel a legitimate reload of the same gun.
    // reloadstate 1-3 replaces both legacy blocks below (per-weapon records, Config.hpp).
    if (g_cfg.reload_state_id != 0) reload_state_track();
    if (g_cfg.reload_state_id == 0 && s_reload != ReloadState::Idle) {
        const std::string wk = weapon_key();
        if (!wk.empty() && !s_reload_weapon.empty() && wk != s_reload_weapon) {
            // Remember the gun that left with its mag out, then cancel for the gun in hand.
            WpnMem* m = wpn_mem_slot(s_reload_weapon);
            m->key = s_reload_weapon; m->mag_out = true; m->chamber_left = s_sl_chamber_left;
            m->empty = s_sl_empty_at_drop; m->lock_pending = s_sl_lock_pending; m->true_empty = s_true_empty;
            if (g_cfg.reload_vr_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD remembered %s: mag out, %d chambered", s_reload_weapon.c_str(), s_sl_chamber_left);
            s_sl_lock_pending = false; s_sl_reload_due = false; s_sl_pressed_early = false; s_sl_locked_back = false; s_sl_press_pending = false; s_sl_press_due_at = 0;
            set_state(ReloadState::Idle, "weapon changed, reload cancelled");
        }
    }
    // ...and a gun that comes back with its mag still out picks up where it left off.
    if (g_cfg.reload_state_id == 0) {
        static std::string s_last_key;
        const std::string wk = weapon_key();
        if (!wk.empty() && wk != s_last_key) {
            s_last_key = wk;
            if (s_reload == ReloadState::Idle) {
                if (WpnMem* m = wpn_mem_find(wk)) {
                    if (m->mag_out) {
                        s_reload_weapon = wk;
                        s_sl_chamber_left = m->chamber_left; s_sl_empty_at_drop = m->empty;
                        s_sl_lock_pending = m->lock_pending; s_true_empty = m->true_empty;
                        s_restoring_mag_out = true;
                        set_state(ReloadState::MagOut, "weapon returned with its mag out");
                        s_restoring_mag_out = false;
                    }
                    wpn_mem_clear(wk);
                }
            }
        }
    }
}

bool reload_engine_grip_held() {
    // THE GRIP: UEVR's per-hand ACTION, with the XInput mask kept only as a fallback. The mask
    // (reloadgrip, default 0x0100 = left shoulder) was the ONLY path here, and whether a physical
    // grip produces that bit depends on the controller and the runtime -- the plugin even ships a
    // remap that puts left X on the same bit. Field report from another machine: reload stalled in
    // MAG_OUT with the trigger suppressed and two-handing never latched, together, because both
    // gated on it. Holsters never had this problem; they have always read the action.
    const bool fetch_right    = g_cfg.aim_left_hand;   // the hand that is NOT aiming fetches
    return holster_grip_held(fetch_right);
}

bool reload_engine_fetch_pose(bool pose_ok, const Vec3& hand_l, const Vec3* head_p) {
    // No head pose, no belt or well to measure against: the fetch hand counts as absent.
    bool have_left = (head_p != nullptr) && pose_ok;
    // DEAD-POSE GATE. get_pose passes a sleeping/glitched controller as (0,0,0) -- the room origin,
    // which sits at the head on this rig. From there the well is within 30 cm and "lifted" reads
    // +60 cm, so ONE bad frame seats the magazine: logged 2026-08-16 12:32 as a seat 164 ms after
    // a grab that was 67 cm from the well. Same rule the palette publisher applies to the aim hand.
    if (have_left) {
        const Vec3& head = *head_p;
        const float rx = hand_l.x - head.x, ry = hand_l.y - head.y, rz = hand_l.z - head.z;
        const float reach2 = rx*rx + ry*ry + rz*rz;
        const bool zero = std::fabs(hand_l.x) < 1e-6f && std::fabs(hand_l.y) < 1e-6f && std::fabs(hand_l.z) < 1e-6f;
        if (zero || !std::isfinite(reach2) || reach2 > 1.5f * 1.5f) {
            have_left = false;
            if (g_cfg.reload_vr_log) {
                static uint32_t s_dead = 0;
                if ((s_dead++ % 30u) == 0u)
                    API::get()->log_info("[Halo-CampE-UEVR] RELOAD: ignoring dead/implausible left pose (zero=%d reach=%.2fm)", (int)zero, std::sqrt(reach2));
            }
        }
    }
    return have_left;
}

// A reload tap made while a reset's window held the tick, dropped at the next press test.
bool s_reset_drop_tap = false;

bool reload_engine_press_ignored() {
    ++s_rt_taps;   // reloadshotgunlog: every tap the engine saw, ignored or not
    // A tap made while a reset's window held the tick (a death, a ride, a cutscene) is dropped once:
    // it belongs to the gun that went away with the body. reloadresetholds 0 restores the old path.
    if (s_reset_drop_tap) {
        s_reset_drop_tap = false;
        if (g_cfg.reload_vr_log || g_cfg.reload_state_log)
            API::get()->log_info("[Halo-CampE-UEVR] RELOAD dropped a reload tap made during the reset's window");
        return true;
    }
    if (!weapon_in_list(g_cfg.reload_skip_weapons)) return false;
    // No magazine on this weapon (plasma rifle, plasma pistol, sentinel beam): nothing to
    // drop, nothing to seat, and the game's own reload does nothing to it either.
    if (g_cfg.reload_vr_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD ignored: %s has no magazine", weapon_key().c_str());
    return true;
}

void reload_engine_press_accepted() {
    s_reload_weapon = weapon_key();   // whose magazine this is (see the swap-cancel above)
    s_sl_empty_at_drop = weapon_empty_now();   // decides whether the seat wants a rack
    s_sl_locked_back = s_sl_empty_at_drop;
    s_sl_chamber_left  = s_sl_empty_at_drop ? 0 : 1;   // the one round already chambered
}

bool reload_engine_belt_grab_ok(bool belt) {
    // The grab. With the visible magazine on (and the holster tick alive to run its body
    // frame), the hand must take the MAG -- the point the mesh is drawn at, published by
    // Holster. Otherwise the legacy belt ring: below the head, near the body's vertical axis.
    // Y is up in this space -- the same convention quat_forward assumes for the VR frame.
    if (g_cfg.reload_mag != 0 && g_cfg.holster_enabled) return holster_mag_hand_in();
    return belt;
}

void reload_engine_grabbed(float hand_y) {
    s_grab_y = hand_y;   // where the mag was picked up: the lift gate measures from here
}

bool reload_engine_seat(bool have_left, const Vec3& hand_l, const Vec3* hand_r_p, const Vec3* head_p) {
    if (!have_left || hand_r_p == nullptr) return true;
    const Vec3& hand_r = *hand_r_p;
    const Vec3& head   = *head_p;
    // THE WELL. First choice: the weapon's OWN magazine component -- the one mag_hide_apply
    // just hid is the rendered magazine sitting in its well, so its live world location IS the
    // per-weapon insert point, on every weapon, with nothing to calibrate. The seat test then
    // runs in world space (the hand goes room->world through the same transform that places
    // every holster marker). Fallback when no component resolved: the old fixed point
    // reload_well_fwd along the aim direction from the aim hand. And the LIFT GATE either
    // way: the mag must have risen since it was grabbed. Without both, the log
    // (2026-08-16 12:07) shows the reload firing 214-224 ms after the belt grab, at the hip.
    // ONE SNAPSHOT (zonesnapshot, doctrine in ConfigFields.inl). The magazine component's world
    // position used to be read HERE, live, while the hand poses arrived from the caller's own
    // reads and reload_well_stabilize then took a THIRD read of the aim hand for itself: three
    // moments in one 7 cm gate. With the key on the component transform comes from the tick's
    // snapshot, beside the poses it is compared against, and the head is the snapshot's too.
    const bool use_snap = zone_snapshot_on();
    const Vec3 head_s = use_snap && s_snap.poses_ok ? s_snap.head : head;
    float join = 1e9f;
    Vec3  well_world{}; bool have_well_world = false;
    if (auto* mc = s_mag_hidden.get()) {
        Vec3 wl{};
        const bool got = use_snap ? s_snap.well_ok : call_ret_vec3(mc, L"K2_GetComponentLocation", &wl);
        if (use_snap && got) wl = s_snap.well;
        if (got) {
            well_world = wl;
            have_well_world = true;
            well_world = reload_well_stabilize(mc, well_world, head_s);
            const Vec3 hlw = reload_hand_world(hand_l, head_s);
            const float wdx = hlw.x - well_world.x, wdy = hlw.y - well_world.y,
                        wdz = hlw.z - well_world.z;
            join = std::sqrt(wdx * wdx + wdy * wdy + wdz * wdz) / 100.0f;  // cm -> m
        }
    }
    if (!have_well_world) {
        Vec3 well = hand_r;
        if (use_snap && s_snap.poses_ok) {
            well = Vec3{hand_r.x + s_snap.f.x * g_cfg.reload_well_fwd,
                        hand_r.y + s_snap.f.y * g_cfg.reload_well_fwd,
                        hand_r.z + s_snap.f.z * g_cfg.reload_well_fwd};
        } else {
            const auto ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                                  : API::VR::get_right_controller_index();
            Vec3 apos{}; Quat aq{};
            if (ridx >= 0 && get_pose(ridx, &apos, &aq, /*use_aim=*/true)) {
                const Vec3 f = quat_forward(placement_aim_fix(aq));
                well = Vec3{hand_r.x + f.x * g_cfg.reload_well_fwd,
                            hand_r.y + f.y * g_cfg.reload_well_fwd,
                            hand_r.z + f.z * g_cfg.reload_well_fwd};
            }
        }
        const float ddx = hand_l.x - well.x, ddy = hand_l.y - well.y, ddz = hand_l.z - well.z;
        join = std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
        well_world = use_snap ? reload_hand_world(well, head_s) : holster_room_to_world(well, head_s);
    }
    reload_well_marker_update(true, well_world);
    const bool lifted = (hand_l.y - s_grab_y) >= g_cfg.reload_lift;
    if (g_cfg.reload_vr_log) {
        static uint32_t s_rl = 0;
        if ((s_rl++ % 15u) == 0u)
            API::get()->log_info("[Halo-CampE-UEVR] RELOAD held: mag-to-well=%.0fcm lifted=%.0fcm (need <=%.0f, >=%.0f) | snap=%u mode=%d well=%s",
                                 join * 100.0f, (hand_l.y - s_grab_y) * 100.0f,
                                 g_cfg.reload_seat_dist * 100.0f, g_cfg.reload_lift * 100.0f,
                                 use_snap ? s_snap.seq : 0u, g_cfg.zone_snapshot,
                                 have_well_world ? "component" : "reloadwellfwd fallback");
    }
    // THE SLIDE, in place of the old four-tick debounce. Inside the capture radius (and
    // lifted) the magazine leaves the hand and travels into the well over reload_slide_ms;
    // the reload fires when it lands. Pulling the hand well clear mid-slide aborts it back to
    // the hand. The duration IS the debounce: a one-frame tracking glitch cannot complete it.
    {
        const long long nowt = now_ticks();
        // Target transform: the weapon's magazine component when we have it (its rotation
        // is the seated mag's, so the slide ends exactly on the real one); else the fallback
        // well point with no orientation, which slides position only.
        g_reload_slide_x.store(well_world.x, std::memory_order_relaxed);
        g_reload_slide_y.store(well_world.y, std::memory_order_relaxed);
        g_reload_slide_z.store(well_world.z, std::memory_order_relaxed);
        bool rot_ok = false;
        if (have_well_world) {
            if (auto* mc = s_mag_hidden.get()) {
                // The seated magazine's own rotation, from the SAME snapshot its position came
                // from: the insert axis below is built out of it, so a second read here would put
                // the axis in a different moment than the point it passes through.
                Vec3 wrot{};
                bool grot = use_snap ? s_snap.well_rot_ok : call_ret_vec3(mc, L"K2_GetComponentRotation", &wrot);
                if (use_snap && grot) wrot = s_snap.well_rot;
                if (grot) {
                    g_reload_slide_pitch.store(wrot.x, std::memory_order_relaxed);
                    g_reload_slide_yaw.store(wrot.y,   std::memory_order_relaxed);
                    g_reload_slide_roll.store(wrot.z,  std::memory_order_relaxed);
                    rot_ok = true;
                }
            }
        }
        g_reload_slide_rot_valid.store(rot_ok, std::memory_order_relaxed);

        // THE WELL'S MOUTH (insert mode). The seat point is the seated mag's own centre,
        // inside the grip, so a radius around it only fires when the mag is nearly home.
        // The lock instead happens anywhere on the axis from reload_insert below the seat up
        // to the seat, within reload_seat_dist of the axis SIDEWAYS; the abort is sideways too.
        bool  axis_ok = false; float lateral_m = join, d_axis_cm = 0.0f;
        if (g_cfg.reload_insert_mode == 1 && have_well_world && rot_ok) {
            const Quat qs = rotator_to_quat(g_reload_slide_pitch.load(std::memory_order_relaxed),
                                            g_reload_slide_yaw.load(std::memory_order_relaxed),
                                            g_reload_slide_roll.load(std::memory_order_relaxed));
            Vec3 up = quat_rotate(qs, Vec3{0.0f, 0.0f, 1.0f});
            up = Vec3{up.x * g_cfg.reload_insert_sign, up.y * g_cfg.reload_insert_sign, up.z * g_cfg.reload_insert_sign};
            const Vec3 hw = reload_hand_world(hand_l, head_s);
            const Vec3 r{hw.x - well_world.x, hw.y - well_world.y, hw.z - well_world.z};
            d_axis_cm = r.x * up.x + r.y * up.y + r.z * up.z;
            const Vec3 lat{r.x - up.x * d_axis_cm, r.y - up.y * d_axis_cm, r.z - up.z * d_axis_cm};
            lateral_m = std::sqrt(lat.x * lat.x + lat.y * lat.y + lat.z * lat.z) / 100.0f;
            axis_ok = true;
        }
        const float jd = g_cfg.reload_seat_dist + reload_gate_pad_m();
        const bool in_mouth = axis_ok
            ? (lateral_m <= jd && d_axis_cm >= -(reload_insert_for_weapon() + jd) * 100.0f && d_axis_cm <= jd * 100.0f)
            : (join <= jd);
        const bool pulled_clear = axis_ok ? (lateral_m > jd * 2.5f || d_axis_cm < -(reload_insert_for_weapon() + jd * 2.5f) * 100.0f)
                                         : (join > jd * 2.5f);

        if (s_slide_start == 0) {
            if (in_mouth && lifted) {
                s_slide_start = nowt;
                g_reload_slide_t.store(0.0f, std::memory_order_relaxed);
                ak_step_sound("seat");
                if (g_cfg.reload_vr_log)
                    API::get()->log_info("[Halo-CampE-UEVR] RELOAD slide begins at %.0fcm", join * 100.0f);
            }
        } else if (pulled_clear) {
            if (g_cfg.reload_vr_log)
                API::get()->log_info("[Halo-CampE-UEVR] RELOAD slide aborted, hand pulled clear (%.0fcm)", join * 100.0f);
            reload_slide_reset();
        } else {
            const float ms = (float)std::chrono::duration_cast<std::chrono::milliseconds>(
                clock_t_::duration(nowt - s_slide_start)).count();
            bool done = false;
            if (g_cfg.reload_insert_mode == 1 && have_well_world && rot_ok) {
                // HAND-DRIVEN. The mag snaps onto the well's axis (the short blend), then sits
                // on that axis at the HAND's height below the seat; it is home when the hand
                // has pushed it within reload_insert_done of the seat.
                const float snap = std::fmin(1.0f, ms / (float)std::fmax(1, g_cfg.reload_slide_ms));
                g_reload_slide_t.store(snap, std::memory_order_relaxed);
                const Quat qs = rotator_to_quat(g_reload_slide_pitch.load(std::memory_order_relaxed),
                                                g_reload_slide_yaw.load(std::memory_order_relaxed),
                                                g_reload_slide_roll.load(std::memory_order_relaxed));
                Vec3 up = quat_rotate(qs, Vec3{0.0f, 0.0f, 1.0f});
                up = Vec3{up.x * g_cfg.reload_insert_sign, up.y * g_cfg.reload_insert_sign, up.z * g_cfg.reload_insert_sign};
                const Vec3 hw = reload_hand_world(hand_l, head_s);
                const float d = (hw.x - well_world.x) * up.x + (hw.y - well_world.y) * up.y + (hw.z - well_world.z) * up.z;   // cm along the axis, negative = below the seat
                const float travel = reload_insert_for_weapon() * 100.0f;
                float along = d; if (along > 0.0f) along = 0.0f; if (along < -travel) along = -travel;
                g_reload_slide_x.store(well_world.x + up.x * along, std::memory_order_relaxed);
                g_reload_slide_y.store(well_world.y + up.y * along, std::memory_order_relaxed);
                g_reload_slide_z.store(well_world.z + up.z * along, std::memory_order_relaxed);
                done = (d >= -g_cfg.reload_insert_done * 100.0f);
                if (g_cfg.reload_vr_log) {
                    static uint32_t s_il = 0;
                    if ((s_il++ % 15u) == 0u) API::get()->log_info("[Halo-CampE-UEVR] RELOAD insert: %.1f cm below the seat (home within %.1f)", -d, g_cfg.reload_insert_done * 100.0f);
                }
            } else {
                const float t = std::fmin(1.0f, ms / (float)std::fmax(1, g_cfg.reload_slide_ms));
                g_reload_slide_t.store(t, std::memory_order_relaxed);
                done = (t >= 1.0f);
            }
            if (done) {
                reload_slide_reset();
                // Locked back until racked ONLY if the gun was empty when the mag left --
                // a chambered round needs no rack, that is how a pistol works.
                const bool rack_first = reload_rack_available() && g_cfg.slide_lock_reload && slide_weapon_ok() && slide_chamber_ok() && slide_rack_available()
                                        && (s_sl_empty_at_drop || weapon_in_list(g_cfg.slide_always_weapons));
                if (rack_first) {
                    s_sl_lock_pending = true; s_sl_lock_frame = 0; s_sl_rack_done = false;
                } else {
                    s_sl_locked_back = false;   // no rack will clear it: a stale flag kept the record alive and the rack zone live
                }
                if (rack_first && g_cfg.slide_chamber) {
                    // SLIDECHAMBER: the mag is in, the chamber is not. The rack ends the reload.
                    if (s_sl_press_pending || s_sl_press_due_at != 0) { s_sl_press_pending = false; s_sl_press_due_at = 0; s_sl_pressed_early = true; reload_press_now("seat (waited)"); }
                    s_sl_reload_due = true;
                    if (g_cfg.reload_vr_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD seated on an empty gun: waiting for the rack");
                } else if (s_sl_pressed_early) {
                    s_sl_pressed_early = false;
                    if (g_cfg.reload_vr_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD seated, the sim reloaded at the drop");
                } else {
                    s_sl_press_pending = false; s_sl_press_due_at = 0;
                    reload_press_now("seat");
                }
                if (g_cfg.anim_vars) anim_vars_begin();
                audio_dump_begin();
                ak_dump("seat");
                ak_step_sound("seated");
                if (g_cfg.anim_dump) {
                    s_anim_dump_until = nowt + ms_to_ticks(2500);
                    API::get()->log_info("[Halo-CampE-UEVR] ANIMDUMP begin (seat)");
                }
                set_state(ReloadState::Idle, "magazine seated, reload fired");
            }
        }
    }
    return true;
}

void reload_engine_state_set(ReloadState prev, ReloadState next) {
    // The weapon's own mag leaves with the state and returns with it -- Idle is the only state
    // where the gun should be showing a magazine.
    mag_hide_apply(next != ReloadState::Idle);
    // ...and on the way out it FALLS: the drop is spawned from the component just hidden.
    if (prev == ReloadState::Idle && next == ReloadState::MagOut && !s_restoring_mag_out) { if (!slide_parts_mag_drop()) mag_drop_spawn(); ak_dump("drop"); ak_step_sound("drop"); ak_step_sound("open");
        if (g_cfg.reload_press_at == 1 && slide_weapon_ok()) {
            const bool empty_now = weapon_empty_now();
            if (empty_now) { s_sl_pressed_early = true; reload_press_now("mag drop"); }
            else { s_sl_press_pending = true; if (g_cfg.reload_vr_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD press waits for the chambered shot or the seat"); }
        }
    }
}

int reload_engine_fire_suppressed() {
    const bool rack = reload_rack_available();
    // Slide weapons keep the trigger LIVE with the magazine out: whatever the game still has
    // loaded is the chambered round(s), and the game locks the slide back itself when they run
    // out. Everything else keeps the original suppression.
    if (g_cfg.reload_hold_fire && s_reload != ReloadState::Idle) {
        if (!(rack && slide_weapon_ok() && slide_chamber_ok())) return 1;
        if (s_sl_chamber_left <= 0) return 1;   // the one chambered round is spent (or never was)
    }
    // THE SLIDE: no shot with the chamber open. Dead while the slide is pulled more than a
    // third of its travel, and dead from a reload's seat until the rack completes.
    // Truly empty never fires, whatever slidevr says: the phantom re-asserts one round every tick,
    // so a trigger left live with slidevr off fired it forever; the dry stop and the hidden reload
    // promise a dead trigger too.
    if (s_true_empty) return 1;
    if (rack) {
        if (s_sl_lock_pending) return 1;
        if (s_sl_held && g_slide_pull.load(std::memory_order_relaxed) > g_cfg.slide_travel * 0.33f) return 1;
    }
    return 0;
}

// ---- SHOTGUNLOG (shotgun_log). Every reload value is sampled each tick and ONE line is printed on
// any tick where one of them changed, naming what changed. Read-only: it writes nothing to the game
// and touches no state but its own. Ported from the owner's play build, where it is what found the
// stale rounds pointer, the empty judge and the trigger verdict.
namespace {
struct RtSnap {
    std::string key;
    int state = -1, rounds = -1, reserve = -1, fps = -1, frame = -1;
    int lock = 0, lockedback = 0, due = 0, early = 0, presspend = 0, pressdue = 0, chamber = 0, emptydrop = 0, trueempty = 0, hidedisp = 0;
    int held = 0, racked = 0, pullmm = 0, hot = 0, pressact = 0, statehold = 0, akwin = 0;
    int everyshot = 0, pump = 0, reloadonly = 0, rack = 0, hidden = 0, coop = 0, taps = 0, presses = 0, mem = 0;
    int guard = 0, emptynow = -1, nofire = 0, reloadvr = 1;
};
RtSnap s_rt_prev;
bool   s_rt_have = false;
int    s_rt_inserts = 0, s_rt_shots = 0;
}  // namespace
void reload_trace_tick() {
    if (g_cfg.shotgun_log == 0 || (g_cfg.shotgun_log == 1 && !weapon_in_list("Shotgun"))) { s_rt_have = false; return; }
    RtSnap n;
    n.key = weapon_key();
    n.state = (int)s_reload;
    if (auto* r = rounds_field()) n.rounds = (int)*r;
    {
        const uintptr_t obj = g_wpn_obj_ptr.load(std::memory_order_relaxed);
        const int off = s_ph_reserve_seen;
        if (obj != 0 && off >= 0 && off < 0x7FE && !IsBadReadPtr((const void*)(obj + (uintptr_t)off), 2))
            n.reserve = (int)*reinterpret_cast<const uint16_t*>(obj + (uintptr_t)off);
    }
    if (auto* animbp = reload_weapon_anim_instance()) {
        if (auto* p = animbp->get_property_data<uint8_t>(L"FirstPersonState")) if (!IsBadReadPtr(p, 1)) n.fps = (int)*p;
        if (auto* p = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame")) if (!IsBadReadPtr(p, sizeof(int32_t))) n.frame = *p;
    }
    n.lock = s_sl_lock_pending; n.lockedback = s_sl_locked_back; n.due = s_sl_reload_due; n.early = s_sl_pressed_early;
    n.presspend = s_sl_press_pending; n.pressdue = (s_sl_press_due_at != 0); n.chamber = s_sl_chamber_left;
    n.emptydrop = s_sl_empty_at_drop; n.trueempty = s_true_empty;
    n.hidedisp = g_wristhud_hide_cradle.load(std::memory_order_relaxed);
    n.held = s_sl_held; n.racked = s_sl_racked;
    n.pullmm = ((int)(g_slide_pull.load(std::memory_order_relaxed) * 3048.0f) / 5) * 5;   // 5 mm steps
    n.hot = g_slide_zone_hot.load(std::memory_order_relaxed);
    n.pressact = reload_press_active();
    n.statehold = (s_sh_until != 0); n.akwin = (s_akm_until != 0);
    n.everyshot = g_sl_zone_every_shot.load(std::memory_order_relaxed); n.pump = g_sl_zone_pump.load(std::memory_order_relaxed);
    n.reloadonly = g_sl_zone_reload_only.load(std::memory_order_relaxed); n.rack = g_slide_rack_found;
    n.hidden = reload_hidden_mode(); n.coop = (g_cfg.coop_auto && net_is_coop());
    n.taps = s_rt_taps; n.presses = s_rt_presses;
    for (auto& m : s_wpn_mem) if (!m.key.empty()) ++n.mem;
    // The object guard refusing a stale pointer, the empty judge the drop would use now, the trigger
    // verdict and the manual reload switch.
    const int32_t sg_idx = g_wpn_obj_index.load(std::memory_order_relaxed);
    const int32_t sg_ptr = g_wpn_obj_ptr_datum.load(std::memory_order_relaxed);
    n.guard = (sg_idx != -1 && sg_ptr != sg_idx) ? 1 : 0;
    n.emptynow = (int)weapon_empty_now();
    n.nofire = reload_engine_fire_suppressed();
    n.reloadvr = g_cfg.reload_vr;
    if (s_rt_have && n.presses != s_rt_prev.presses) { s_rt_inserts = 0; s_rt_shots = 0; }
    if (s_rt_have && n.key == s_rt_prev.key && n.rounds >= 0 && s_rt_prev.rounds >= 0) {
        if (n.rounds == s_rt_prev.rounds + 1) ++s_rt_inserts;
        else if (n.rounds < s_rt_prev.rounds) ++s_rt_shots;
    }
    std::string ch;
    auto cmp = [&](const char* nm, int a, int b) { if (a != b) { if (!ch.empty()) ch += ','; ch += nm; } };
    if (!s_rt_have) ch = "first";
    else {
        if (n.key != s_rt_prev.key) ch = "weapon";
        cmp("state", n.state, s_rt_prev.state); cmp("rounds", n.rounds, s_rt_prev.rounds); cmp("reserve", n.reserve, s_rt_prev.reserve);
        cmp("fps", n.fps, s_rt_prev.fps); cmp("frame", n.frame, s_rt_prev.frame);
        cmp("lock", n.lock, s_rt_prev.lock); cmp("lockedback", n.lockedback, s_rt_prev.lockedback); cmp("due", n.due, s_rt_prev.due);
        cmp("early", n.early, s_rt_prev.early); cmp("presspend", n.presspend, s_rt_prev.presspend); cmp("pressdue", n.pressdue, s_rt_prev.pressdue);
        cmp("chamber", n.chamber, s_rt_prev.chamber); cmp("emptyatdrop", n.emptydrop, s_rt_prev.emptydrop); cmp("trueempty", n.trueempty, s_rt_prev.trueempty);
        cmp("hidedisp", n.hidedisp, s_rt_prev.hidedisp); cmp("held", n.held, s_rt_prev.held); cmp("racked", n.racked, s_rt_prev.racked);
        cmp("pull", n.pullmm, s_rt_prev.pullmm); cmp("hot", n.hot, s_rt_prev.hot); cmp("press", n.pressact, s_rt_prev.pressact);
        cmp("statehold", n.statehold, s_rt_prev.statehold); cmp("akwin", n.akwin, s_rt_prev.akwin);
        cmp("everyshot", n.everyshot, s_rt_prev.everyshot); cmp("pump", n.pump, s_rt_prev.pump); cmp("reloadonly", n.reloadonly, s_rt_prev.reloadonly);
        cmp("rack", n.rack, s_rt_prev.rack); cmp("hidden", n.hidden, s_rt_prev.hidden); cmp("coop", n.coop, s_rt_prev.coop);
        cmp("tap", n.taps, s_rt_prev.taps); cmp("presssent", n.presses, s_rt_prev.presses); cmp("wpnmem", n.mem, s_rt_prev.mem);
        cmp("guard", n.guard, s_rt_prev.guard); cmp("emptynow", n.emptynow, s_rt_prev.emptynow);
        cmp("nofire", n.nofire, s_rt_prev.nofire); cmp("reloadvr", n.reloadvr, s_rt_prev.reloadvr);
    }
    if (!ch.empty()) {
        const long long nowt = now_ticks();
        const long long since = (s_sl_press_at != 0)
            ? (long long)std::chrono::duration_cast<std::chrono::milliseconds>(clock_t_::duration(nowt - s_sl_press_at)).count() : -1;
        API::get()->log_info("[Halo-CampE-UEVR] SGLOG %s [%s] state=%s rounds=%d reserve=%d fps=%d frame=%d | lock=%d lockedback=%d due=%d early=%d presspend=%d pressdue=%d "
                             "chamber=%d emptyatdrop=%d trueempty=%d hidedisp=%d | held=%d racked=%d pull=%dmm hot=%d | press=%d since=%lldms pressms=%d taps=%d presses=%d "
                             "inserts=%d shots=%d | statehold=%d akwin=%d | everyshot=%d pump=%d reloadonly=%d rack=%d always=%d chamberok=%d | hidden=%d coop=%d wpnmem=%d "
                             "| guard=%d idx=0x%08X ptrdatum=0x%08X emptynow=%d nofire=%d reloadvr=%d",
                             n.key.c_str(), ch.c_str(), state_name(s_reload), n.rounds, n.reserve, n.fps, n.frame, n.lock, n.lockedback, n.due, n.early, n.presspend, n.pressdue,
                             n.chamber, n.emptydrop, n.trueempty, n.hidedisp, n.held, n.racked, n.pullmm, n.hot, n.pressact, since,
                             n.coop ? g_cfg.reload_press_ms_coop : g_cfg.reload_press_ms, n.taps, n.presses, s_rt_inserts, s_rt_shots, n.statehold, n.akwin,
                             n.everyshot, n.pump, n.reloadonly, n.rack, (int)weapon_in_list(g_cfg.slide_always_weapons), (int)slide_chamber_ok(), n.hidden, n.coop, n.mem,
                             n.guard, (unsigned)sg_idx, (unsigned)sg_ptr, n.emptynow, n.nofire, n.reloadvr);
    }
    s_rt_prev = n; s_rt_have = true;
}

void reload_engine_gesture_reset() {
    reload_state_on_reset();   // before anything below clears it: the state goes to its weapon
    s_sl_reload_due = false; s_sl_pressed_early = false; s_sl_locked_back = false; s_sl_press_pending = false; s_sl_press_due_at = 0;
    s_true_empty = false;
    sc_teardown("reset");
    sp_teardown("reset");
    // sp_teardown clears the rack flag for the rebuild mode, but the NATIVE part is not torn down
    // here and slide_part_tick only re-finds it on a weapon or component change: after a ride or a
    // cutscene the held gun read "no rack" (no lock-back at the seat, no phantom) until a swap.
    if (s_np_have && s_np_slide.get() != nullptr) g_slide_rack_found = true;
    // ...and the WINDOWS a press opens, which the state reset above does not touch: the synthesized
    // reload press on its way to the game, the FirstPersonState hold, the animation rate clamp, the
    // Wwise mute window and the first-person pose hold. Dying mid-reload left all five running on
    // the respawned gun. reloadresetholds 0 restores the old behaviour.
    if (g_cfg.reload_reset_holds) {
        g_reload_hold_until.store(0, std::memory_order_relaxed);
        s_sl_press_at = 0;
        s_sh_until = 0; s_sh_inst = TrackedObject{};
        if (s_anim_rate_until != 0) s_anim_rate_until = 1;   // the next tick hands rate and pause back
        if (s_akm_until != 0) ak_id_mute_end();
        reload_pose_hold(0);
        // A tap made while the window held the tick belongs to the body that died: the first frame
        // control comes back must not fire it.
        s_reset_drop_tap = true;
        if (g_cfg.reload_vr_log || g_cfg.reload_state_log)
            API::get()->log_info("[Halo-CampE-UEVR] RELOAD reset released the press, the state hold, the rate clamp, the sound mute and the pose hold");
    }
}

void reload_engine_ticks(bool poses_ok, const Vec3& hpos) {
    // The rack, the pump, the chamber and the rest of the reload's own ticks ride with it, not with
    // melee: switching melee off must not switch the slide off. They keep their pose gate.
    if (poses_ok) {
        slide_update(hpos);
        slide_montage_tick();
        anim_state_probe_tick();
        slide_fire_tick();
        slide_phantom_tick();
        slide_chamber_tick();
        slide_copy_tick();
        slide_part_tick();
        slide_node_write_tick();
        reload_trace_tick();
    }
    // Per-weapon reload state: re-hide a magazine that is out on whichever actor renders the weapon
    // now, and mirror the live state into its record. Before the melee-off return, so switching melee
    // off never stops the reload state from saving.
    mag_hide_enforce();
    reload_state_mirror();
}

void reload_engine_released() {
    reload_release_all("manual reload and rack switched off");
    s_reload_on = false;
    ak_engine_released();   // the Wwise detours stand down, an open mute window closes, an adopted emitter goes back
    // The state went Idle above with the engine already off, so its state hook did not run: the magazine
    // comes back, and the well marker, the slide into the well, the rack parts and the holds let go here.
    mag_hide_apply(false);
    reload_well_marker_update(false, Vec3{});
    reload_slide_reset();
    sc_teardown("released");
    sp_teardown("released");
    g_slide_zone_hot.store(false, std::memory_order_relaxed);
    g_reload_pose_hold_until.store(0, std::memory_order_release);
}

// ---- THE BELT MAGAZINE (Holster.cpp's hooks)

API::UObject* reload_engine_mag_mesh(int* out_rank) {
    // The weapon's own magazine component first (exact, rank 4); then the same asset found by NAME
    // from the weapon key, which is exact too and so also rank 4 (reloadmagasset, doctrine in
    // ConfigFields.inl). With reloadmagasset on there is no third answer: nothing else is a
    // magazine, so nothing else is drawn.
    auto* nm = native_mag_mesh_impl();
    if (nm == nullptr && g_cfg.reload_mag_asset != 0) nm = mag_asset_by_name_impl();
    if (nm != nullptr && out_rank != nullptr) *out_rank = 4;
    return nm;
}

// ---- THE FOUR GATES reloadmagasset PUTS ON THE AUTHOR'S SURVEY. Each is one hook call in
// Holster.cpp and each returns the author's own answer while the key is 0, so upstream's
// behaviour is intact and recoverable by one cfg line.

// The name survey itself. With the key on it never runs, which is what removes the "ammo" rank
// (its whole yield on the owner's level was eight ammo pickups and a crate), the "clip" rank, and
// the frag-grenade fallback at the end of the author's chain in one stroke. "clip" goes with it
// deliberately: no shipped asset in this game is named with it -- every magazine in the owner's
// SLIDEPART listings is SM_<Weapon>_Magazine*, the classic AR's is Megazine, and the only other
// ammunition assets are SM_AmmoPickup_*/SM_ammo_pickup_*/SM_ammo_crate -- so the rank could only
// ever match something that is not a magazine.
bool reload_engine_mag_survey_off() { return g_cfg.reload_mag_asset != 0; }

// The author's "re-survey once for a pick weaker than a magazine" condition. With the key on there
// is no survey to re-run, so it must not clear the candidate list and re-arm a ~290k walk.
bool reload_engine_mag_resurvey(int rank) { return g_cfg.reload_mag_asset != 0 ? false : rank < 3; }

// Whether this pick may be STORED under the weapon key as the final answer. Only the weapon's own
// magazine component or the name-matched asset (both rank 4) may. Anything weaker leaves the key
// unset, so the next tick picks again -- which is what unlatches the magnum: its component was
// simply not attached yet at the tick the first pick ran, and both resolvers memoise per weapon
// key, so re-picking walks nothing.
bool reload_engine_mag_pick_final(int rank) { return g_cfg.reload_mag_asset == 0 || rank >= 4; }

// The mesh the marker component is SPAWNED with. The author's line is "the survey found nothing:
// the frag stands in, visibly" -- that is the path that put a grenade on the belt, so with the key
// on there is no stand-in: our own resolver's answer, or no marker at all.
API::UObject* reload_engine_mag_spawn_mesh(API::UObject* survey, API::UObject* frag) {
    if (g_cfg.reload_mag_asset == 0) return survey != nullptr ? survey : frag;
    return reload_engine_mag_mesh(nullptr);
}

// The last word on the marker in a tick: a weapon whose magazine asset does not resolve draws NO
// magazine, rather than the previous weapon's magazine left on the component by the pick that
// was refused above.
void reload_engine_mag_drawn(API::UObject* m, bool wanted) {
    if (g_cfg.reload_mag_asset == 0 || m == nullptr || !wanted) return;
    if (reload_engine_mag_mesh(nullptr) != nullptr) return;
    holster_marker_show(m, false);
    marker_render_drop(m);
}

Vec3 reload_engine_mag_belt_point() { return mag_belt_point(); }

// THE BELT MAGAZINE'S ZONE, AND THE PROOF THAT IT IS ALREADY ONE FRAME. This was read the wrong
// way round once and must not be again, so the algebra is written out and the log checks it at
// runtime. The belt point is a BODY-FRAME offset (x right, y up, z back, from the body anchor).
// The fetch hand arriving here is body frame too: Holster builds it as R(-yaw) * (hand_room -
// anchor). The drawn magazine is placed at room = anchor + R(+yaw) * belt, and the body-frame
// image of that room point is R(-yaw) * (room - anchor) = R(-yaw) * R(+yaw) * belt = belt,
// identically, for every yaw and every anchor. So |hand_body - belt| IS the distance from the
// hand to the drawn mesh, it is rotation invariant, and it carries no anchor term. Turning the
// belt point into a room point for the compare would have subtracted a room position from a body
// position: not a distance at all, and one that grows with the anchor, which on an origin
// carrying floor height would stop the grab firing and deadlock the reload in MAG_OUT.
//
// The log prints both sides with their frames named, and re-derotates the drawn point so the
// identity above is a measured number in the owner's log rather than a claim in a comment.
void reload_engine_mag_zone_measured(float dist_m, const Vec3& hand_body, const Vec3& belt_body,
                                     const Vec3& anchor, float yaw_cos, float yaw_sin) {
    if (!g_cfg.reload_vr_log) return;
    static uint32_t s_n = 0;
    if ((s_n++ % 15u) != 0u) return;
    const Vec3 room{anchor.x + belt_body.x * yaw_cos + belt_body.z * yaw_sin,
                    anchor.y + belt_body.y,
                    anchor.z + (-belt_body.x * yaw_sin + belt_body.z * yaw_cos)};
    const float rx = room.x - anchor.x, ry = room.y - anchor.y, rz = room.z - anchor.z;
    const Vec3 back{rx * yaw_cos + rz * (-yaw_sin), ry, rx * yaw_sin + rz * yaw_cos};
    const float err = std::sqrt((back.x - belt_body.x) * (back.x - belt_body.x)
                              + (back.y - belt_body.y) * (back.y - belt_body.y)
                              + (back.z - belt_body.z) * (back.z - belt_body.z));
    API::get()->log_info("[Halo-CampE-UEVR] RELOAD magzone: hand-to-belt=%.1fcm (need <=%.1f) | hand[body]=(%.3f %.3f %.3f) "
                         "belt[body]=(%.3f %.3f %.3f) -- ONE frame | drawn[room]=(%.3f %.3f %.3f) derotated back to "
                         "[body]=(%.3f %.3f %.3f), identity error %.4fcm | anchor[room]=(%.3f %.3f %.3f)",
                         dist_m * 100.0f, g_cfg.reload_mag_radius * 100.0f,
                         hand_body.x, hand_body.y, hand_body.z, belt_body.x, belt_body.y, belt_body.z,
                         room.x, room.y, room.z, back.x, back.y, back.z, err * 100.0f,
                         anchor.x, anchor.y, anchor.z);
}

bool reload_engine_mag_cands_stale(int rank) {
    if (rank >= 4) return false;
    const auto& hs = host::g_holster_state;
    const size_t n = hs.mag_cand_count();
    if (n == 0) return false;
    for (size_t k = 0; k < n; ++k) if (hs.mag_cand_alive(k)) return false;
    return true;
}

bool reload_engine_mag_in_hand(API::UObject* m, const Vec3& gpos, const Vec3& hpos, float pitchr, float yawr, float rollr) {
    Vec3  place = holster_room_to_world(gpos, hpos);
    float pd = pitchr * RAD2DEG, yd = yawr * RAD2DEG, rd = rollr * RAD2DEG;
    // The in-hand tuning (reloadhandoff / reloadhandrot), in the hand's frame: UE local
    // axes are x forward, y right, z up, so (right, up, forward) maps to (z, x, y).
    {
        const Quat qh = rotator_to_quat(pd, yd, rd);
        const Vec3 lo{g_cfg.reload_hand_off[2] * 100.0f, g_cfg.reload_hand_off[0] * 100.0f, g_cfg.reload_hand_off[1] * 100.0f};
        const Vec3 wo = quat_rotate(qh, lo);
        place = Vec3{place.x + wo.x, place.y + wo.y, place.z + wo.z};
        const Quat qr = quat_mul(qh, rotator_to_quat(g_cfg.reload_hand_rot[0], g_cfg.reload_hand_rot[1], g_cfg.reload_hand_rot[2]));
        quat_to_rotator(qr.x, qr.y, qr.z, qr.w, &pd, &yd, &rd);
    }
    // THE SLIDE (Gesture publishes it): from the hand's pose to the well's, eased.
    // Rotation goes through a normalised quaternion blend so the mag turns the short
    // way into the seated orientation instead of spinning through a rotator wrap.
    const float st = g_reload_slide_t.load(std::memory_order_relaxed);
    if (st >= 0.0f) {
        const float e = st * st * (3.0f - 2.0f * st);   // smoothstep
        const Vec3 tgt{g_reload_slide_x.load(std::memory_order_relaxed),
                       g_reload_slide_y.load(std::memory_order_relaxed),
                       g_reload_slide_z.load(std::memory_order_relaxed)};
        place = Vec3{place.x + (tgt.x - place.x) * e,
                     place.y + (tgt.y - place.y) * e,
                     place.z + (tgt.z - place.z) * e};
        if (g_reload_slide_rot_valid.load(std::memory_order_relaxed)) {
            const Quat qa = rotator_to_quat(pd, yd, rd);
            Quat qb = rotator_to_quat(g_reload_slide_pitch.load(std::memory_order_relaxed),
                                      g_reload_slide_yaw.load(std::memory_order_relaxed),
                                      g_reload_slide_roll.load(std::memory_order_relaxed));
            float d = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w;
            if (d < 0.0f) { qb.x = -qb.x; qb.y = -qb.y; qb.z = -qb.z; qb.w = -qb.w; }
            Quat q{qa.x + (qb.x - qa.x) * e, qa.y + (qb.y - qa.y) * e,
                   qa.z + (qb.z - qa.z) * e, qa.w + (qb.w - qa.w) * e};
            const float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (n > 1e-6f) { q.x /= n; q.y /= n; q.z /= n; q.w /= n; }
            quat_to_rotator(q.x, q.y, q.z, q.w, &pd, &yd, &rd);
        }
    }
    holster_marker_place_rot(m, place, pd, yd, rd);
    marker_render_anchor_rot(m, holster_world_to_room(place, hpos), pd, yd, rd);
    return true;
}

bool reload_engine_reload_busy() { return reload_engine_active() && reload_gestures_busy(); }

} // namespace halo
