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

#include "core/reload/Engine_mag_anim_sound.inl"   // the weapon's own magazine, the well marker, the holds, the sound

#include "core/reload/Engine_rack.inl"   // the rack, the montage, slide fire, chamber and phantom, the parts

#include "core/reload/Engine_ammo_dump_magdrop.inl"   // the rounds probe and the dropped magazine

// The belt point for the weapon in hand: a reloadmagoffw entry if one matches, the global
// reloadmagoff otherwise. Both the rendered mag and the grab zone read this, so what you see and
// what you reach for stay the same point.
Vec3 mag_belt_point() {
    Vec3 mo{g_cfg.reload_mag_off[0], g_cfg.reload_mag_off[1], g_cfg.reload_mag_off[2]};
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
    // The held weapon's Blam object index, for the sim-side node probe (every ~30 ticks).
    {
        static int s_n = 0;
        // Every tick while the reload state tracker runs: the phantom's object guard compares it.
        if ((++s_n % 30) == 0 || g_cfg.reload_state_id != 0) {
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
            if (g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD remembered %s: mag out, %d chambered", s_reload_weapon.c_str(), s_sl_chamber_left);
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
            if (g_cfg.reload_log) {
                static uint32_t s_dead = 0;
                if ((s_dead++ % 30u) == 0u)
                    API::get()->log_info("[Halo-CampE-UEVR] RELOAD: ignoring dead/implausible left pose (zero=%d reach=%.2fm)", (int)zero, std::sqrt(reach2));
            }
        }
    }
    return have_left;
}

bool reload_engine_press_ignored() {
    if (!weapon_in_list(g_cfg.reload_skip_weapons)) return false;
    // No magazine on this weapon (plasma rifle, plasma pistol, sentinel beam): nothing to
    // drop, nothing to seat, and the game's own reload does nothing to it either.
    if (g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD ignored: %s has no magazine", weapon_key().c_str());
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
    float join = 1e9f;
    Vec3  well_world{}; bool have_well_world = false;
    if (auto* mc = s_mag_hidden.get()) {
        if (call_ret_vec3(mc, L"K2_GetComponentLocation", &well_world)) {
            have_well_world = true;
            well_world = reload_well_stabilize(mc, well_world, head);
            const Vec3 hlw = reload_hand_world(hand_l, head);
            const float wdx = hlw.x - well_world.x, wdy = hlw.y - well_world.y,
                        wdz = hlw.z - well_world.z;
            join = std::sqrt(wdx * wdx + wdy * wdy + wdz * wdz) / 100.0f;  // cm -> m
        }
    }
    if (!have_well_world) {
        Vec3 well = hand_r;
        const auto ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                              : API::VR::get_right_controller_index();
        Vec3 apos{}; Quat aq{};
        if (ridx >= 0 && get_pose(ridx, &apos, &aq, /*use_aim=*/true)) {
            const Vec3 f = quat_forward(placement_aim_fix(aq));
            well = Vec3{hand_r.x + f.x * g_cfg.reload_well_fwd,
                        hand_r.y + f.y * g_cfg.reload_well_fwd,
                        hand_r.z + f.z * g_cfg.reload_well_fwd};
        }
        const float ddx = hand_l.x - well.x, ddy = hand_l.y - well.y, ddz = hand_l.z - well.z;
        join = std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
        well_world = holster_room_to_world(well, head);
    }
    reload_well_marker_update(true, well_world);
    const bool lifted = (hand_l.y - s_grab_y) >= g_cfg.reload_lift;
    if (g_cfg.reload_log) {
        static uint32_t s_rl = 0;
        if ((s_rl++ % 15u) == 0u)
            API::get()->log_info("[Halo-CampE-UEVR] RELOAD held: mag-to-well=%.0fcm lifted=%.0fcm (need <=%.0f, >=%.0f)",
                                 join * 100.0f, (hand_l.y - s_grab_y) * 100.0f,
                                 g_cfg.reload_join_dist * 100.0f, g_cfg.reload_lift * 100.0f);
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
                Vec3 wrot{};
                if (call_ret_vec3(mc, L"K2_GetComponentRotation", &wrot)) {
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
        // to the seat, within reload_join_dist of the axis SIDEWAYS; the abort is sideways too.
        bool  axis_ok = false; float lateral_m = join, d_axis_cm = 0.0f;
        if (g_cfg.reload_insert_mode == 1 && have_well_world && rot_ok) {
            const Quat qs = rotator_to_quat(g_reload_slide_pitch.load(std::memory_order_relaxed),
                                            g_reload_slide_yaw.load(std::memory_order_relaxed),
                                            g_reload_slide_roll.load(std::memory_order_relaxed));
            Vec3 up = quat_rotate(qs, Vec3{0.0f, 0.0f, 1.0f});
            up = Vec3{up.x * g_cfg.reload_insert_sign, up.y * g_cfg.reload_insert_sign, up.z * g_cfg.reload_insert_sign};
            const Vec3 hw = reload_hand_world(hand_l, head);
            const Vec3 r{hw.x - well_world.x, hw.y - well_world.y, hw.z - well_world.z};
            d_axis_cm = r.x * up.x + r.y * up.y + r.z * up.z;
            const Vec3 lat{r.x - up.x * d_axis_cm, r.y - up.y * d_axis_cm, r.z - up.z * d_axis_cm};
            lateral_m = std::sqrt(lat.x * lat.x + lat.y * lat.y + lat.z * lat.z) / 100.0f;
            axis_ok = true;
        }
        const float jd = g_cfg.reload_join_dist + reload_gate_pad_m();
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
                if (g_cfg.reload_log)
                    API::get()->log_info("[Halo-CampE-UEVR] RELOAD slide begins at %.0fcm", join * 100.0f);
            }
        } else if (pulled_clear) {
            if (g_cfg.reload_log)
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
                const Vec3 hw = reload_hand_world(hand_l, head);
                const float d = (hw.x - well_world.x) * up.x + (hw.y - well_world.y) * up.y + (hw.z - well_world.z) * up.z;   // cm along the axis, negative = below the seat
                const float travel = reload_insert_for_weapon() * 100.0f;
                float along = d; if (along > 0.0f) along = 0.0f; if (along < -travel) along = -travel;
                g_reload_slide_x.store(well_world.x + up.x * along, std::memory_order_relaxed);
                g_reload_slide_y.store(well_world.y + up.y * along, std::memory_order_relaxed);
                g_reload_slide_z.store(well_world.z + up.z * along, std::memory_order_relaxed);
                done = (d >= -g_cfg.reload_insert_done * 100.0f);
                if (g_cfg.reload_log) {
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
                    if (g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD seated on an empty gun: waiting for the rack");
                } else if (s_sl_pressed_early) {
                    s_sl_pressed_early = false;
                    if (g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD seated, the sim reloaded at the drop");
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
            else { s_sl_press_pending = true; if (g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD press waits for the chambered shot or the seat"); }
        }
    }
}

int reload_engine_fire_suppressed() {
    const bool rack = reload_rack_available();
    // Slide weapons keep the trigger LIVE with the magazine out: whatever the game still has
    // loaded is the chambered round(s), and the game locks the slide back itself when they run
    // out. Everything else keeps the original suppression.
    if (g_cfg.reload_suppress_fire && s_reload != ReloadState::Idle) {
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
    // The weapon's own magazine component first (exact, rank 4); the survey is the fallback.
    auto* nm = native_mag_mesh_impl();
    if (nm != nullptr && out_rank != nullptr) *out_rank = 4;
    return nm;
}

Vec3 reload_engine_mag_belt_point() { return mag_belt_point(); }

// The survey HAS candidates and every one is a dead handle -- a level transition recycled them.
// The caller must re-survey on that, or the fallback chain bottoms out at the frag mesh and the
// belt mag renders as a grenade (field report, 2026-08-31). An empty list is "never surveyed".
// EXCEPT for a shell loader, a dead list is a level transition, and the once-per-key guard must not
// hold -- it is exactly how the belt mag ended up rendering as a frag grenade. rank 4 = the weapon's own
// component, no survey involved.
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
