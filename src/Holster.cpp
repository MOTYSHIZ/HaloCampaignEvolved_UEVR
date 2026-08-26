#include "Holster.hpp"

#include "Config.hpp"
#include "Math.hpp"
#include "MotionAimControl.hpp"
#include "Rig.hpp"
#include "BlamDrive.hpp"
#include "MotionAimControl.hpp"   // the throw aim hold
#include "UeObject.hpp"
#include "Markers.hpp"

#include <chrono>
#include <cmath>
#include <cwctype>

using uevr::API;

namespace halo {

std::atomic<long long> g_holster_swap_until{0};
std::atomic<long long> g_holster_throw_until{0};
std::atomic<long long> g_holster_gswitch_until{0};

namespace {

using clock_t_ = std::chrono::steady_clock;
inline long long now_ticks() { return clock_t_::now().time_since_epoch().count(); }
inline long long ms_to_ticks(int ms) {
    return std::chrono::duration_cast<clock_t_::duration>(std::chrono::milliseconds(ms)).count();
}

// Where A (the game's CURRENT weapon) and B (its backup) live; None = in the hand.
HolsterSlot s_slot_a = HolsterSlot::None;
HolsterSlot s_slot_b = HolsterSlot::RightShoulder;
bool  s_unarmed = false;                              // hand empty: hide + swallow fire
int   s_unhide_ticks = 0;                             // re-assert SetActorHiddenInGame(false) briefly after a draw
HolsterSlot s_in_zone = HolsterSlot::None;           // hand's current zone (for haptics / log)
bool  s_grip_prev = false;
bool  s_grenade_armed = false;
Vec3  s_prev_hand{}; bool s_have_prev = false;
Vec3  s_hand_vel{};                                  // head-relative, m/s, ~60 ms EMA
// ---- THE THROW IS JUDGED ON THE PEAK OF THE SWING, NOT ON THE FRAME YOU LET GO.
//
// Measured 2026-08-24, 10 throws and 1 deliberate put-back. The release-instant speed the gate
// used to read was a median of 0.72 of the swing's own peak, and the ratio got WORSE the harder
// the throw: peak 2.90 -> 0.99 of it survived, peak 4.85 -> 0.29. So the hardest throws were the
// closest to failing, which is why "throw it again, harder" made it worse and why it could not be
// reproduced on demand: it turns on the phase between your grip release and the peak, which is
// not something a person can aim at.
//
// Four of the ten cleared 1.20 m/s by less than 1.35x. The peak clears it by 1.7x at worst.
// The separation against a real put-back is not close: every throw peaked at 2.04 or above, the
// put-back peaked at 0.16. Thirteen times the gap, so no threshold in this range can confuse them.
//
// Stale frames were 0 of 32, 0 of 54, 0 of 16 ... zero in every sample. The EMA is fed properly
// and sampling aliasing is NOT the fault here. Measured, not assumed, and now ruled out.
float s_peak_fwd = 0.0f, s_peak_spd = 0.0f;          // windowed peak of the SAME EMA the gate reads
long long s_peak_at = 0;                             // when that peak was set (decay window)
long long s_arm_at = 0;                              // grip press that armed the grenade
int   s_stale_frames = 0, s_total_frames = 0;        // identical controller pose = a frame the
Vec3  s_prev_raw{};                                  // difference cannot see (sampling aliasing)
bool  s_have_raw = false;
long long s_last_swap = 0;                           // debounce: one swap per grip press, min gap
long long s_last_action = 0;                         // any holster action (melee veto window)
bool  s_near_zone = false;                           // hand within radius+margin of any zone
float s_nearest_dist = 1e9f;                         // ...and how far the nearest one actually is
// Grenade type: READ from the unit object (BlamDrive publishes unit+0x380/+0x382/+0x383), with a
// belief fallback only while that read is not live. The game auto-switches when a type runs out,
// which is exactly what a belief alone would lose.
int   s_gtype = 0;
int   s_gswitch_pending = 0;   // ticks left waiting for the game to flip after our press

// ---- GRENADE VISUALS: two pouch spheres + one on the hand while armed. TrackedObject because a
// level load recycles the components; a failed spawn retries every ~2 s rather than latching dead.
TrackedObject s_pouch_l, s_pouch_r, s_hand_g;
uint32_t s_mk_tick = 0;
// THE MARKER MESHES ARE THE GAME'S OWN GRENADE MODELS, resolved from the loaded-object list.
// The engine sphere proved unreliable on foot -- find_uobject cannot load, and this level had no
// BasicShapes in memory, so three meshless "spheres" rendered as nothing (2026-08-24, in-headset
// and in the spawn log, which said so at the time). Grenade meshes have the opposite property:
// they are loaded exactly where grenades exist, which is exactly where the visuals matter. The
// walk names ~290k objects, so it is PACED and stops for good once both meshes are found.
TrackedObject s_mesh_frag, s_mesh_plasma;
bool s_mesh_logged = false;

void resolve_grenade_meshes() {
    if (s_mesh_frag.get() != nullptr && s_mesh_plasma.get() != nullptr) return;
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;
    auto contains_ci = [](const std::wstring& hay, const wchar_t* needle) {
        std::wstring h; h.reserve(hay.size());
        for (wchar_t ch : hay) h.push_back((wchar_t)towlower(ch));
        return h.find(needle) != std::wstring::npos;
    };
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"StaticMesh") continue;
        const std::wstring full = o->get_full_name();
        if (!contains_ci(full, L"grenade")) continue;
        if (contains_ci(full, L"plasma")) {
            if (s_mesh_plasma.get() == nullptr) s_mesh_plasma.set_at(o, i);
        } else if (s_mesh_frag.get() == nullptr) {
            s_mesh_frag.set_at(o, i);
        }
        if (s_mesh_frag.get() != nullptr && s_mesh_plasma.get() != nullptr) break;
    }
    // One side found and not the other: the found one stands in for both rather than leaving a
    // pouch invisible. Logged so the substitution is visible, not silent.
    if (!s_mesh_logged && (s_mesh_frag.get() != nullptr || s_mesh_plasma.get() != nullptr)) {
        s_mesh_logged = true;
        auto* f = s_mesh_frag.get(); auto* pl = s_mesh_plasma.get();
        API::get()->log_info("[Halo-CampE-UEVR] HOLSTER marker meshes: frag=%ls plasma=%ls",
                             f != nullptr ? f->get_full_name().c_str() : L"(none, using plasma)",
                             pl != nullptr ? pl->get_full_name().c_str() : L"(none, using frag)");
    }
}
// World-frame (VR axes) hand velocity, head motion subtracted -- the head-frame EMA above serves
// the throw GATE, this one serves the throw DIRECTION, which must live in the same frame the aim
// extraction reads (atan2(x,-z)/asin(y), the derive_ctrl_angles convention).
Vec3 s_hand_vel_w{}; Vec3 s_prev_rel_w{};
Vec3 s_peak_velw{};                                  // world velocity AT the fwd peak, the throw's own moment

// Body yaw state for the torso leash. Radians, VR frame, same convention as the head yaw below.
float s_body_yaw = 0.0f;
bool  s_body_init = false;

void markers_hide_all() {
    holster_marker_show(s_pouch_l.get(), false);
    holster_marker_show(s_pouch_r.get(), false);
    holster_marker_show(s_hand_g.get(), false);
}

const char* slot_name(HolsterSlot s) {
    switch (s) {
        case HolsterSlot::RightShoulder: return "right shoulder";
        case HolsterSlot::LeftShoulder:  return "left shoulder";
        case HolsterSlot::RightHip:      return "right hip";
        case HolsterSlot::LeftChest:     return "left chest (frag)";
        case HolsterSlot::RightChest:    return "right chest (plasma)";
        default: return "none";
    }
}

void haptic(float dur, float amp) {
    if (!g_cfg.holster_haptic) return;
    // ABI order: (seconds_from_now, duration, frequency, amplitude, source) -- see TwoHand.cpp.
    API::VR::trigger_haptic_vibration(0.0f, dur, 0.0f, amp, API::VR::get_right_joystick_source());
}

void set_weapon_hidden(bool hidden) {
    auto* w = fp_weapon_actor();
    if (w == nullptr) return;
    alignas(16) uint8_t p[64] = {0};
    p[0] = hidden ? 1 : 0;             // SetActorHiddenInGame(bool bNewHidden)
    w->call_function(L"SetActorHiddenInGame", p);
}

bool right_grip_held() {
    static UEVR_ActionHandle grip = nullptr;
    if (grip == nullptr) grip = API::VR::get_action_handle("/actions/default/in/Grip");
    if (grip == nullptr) return false;
    return API::VR::is_action_active(grip, API::VR::get_right_joystick_source());
}

// Zone centre in the HEAD frame (x right, y up, z back), rotated by the head's yaw only.
Vec3 zone_offset(HolsterSlot s) {
    const float* o = nullptr;
    switch (s) {
        case HolsterSlot::RightShoulder: o = g_cfg.holster_rs; break;
        case HolsterSlot::LeftShoulder:  o = g_cfg.holster_ls; break;
        case HolsterSlot::RightHip:      o = g_cfg.holster_rh; break;
        case HolsterSlot::LeftChest:     o = g_cfg.holster_lc; break;
        case HolsterSlot::RightChest:    o = g_cfg.holster_rc; break;
        default: return Vec3{0.0f, 0.0f, 0.0f};
    }
    return Vec3{o[0], o[1], o[2]};
}

} // namespace

bool holster_swap_press_active()  { const auto u = g_holster_swap_until.load(std::memory_order_relaxed);  return u != 0 && now_ticks() < u; }
bool holster_throw_press_active() { const auto u = g_holster_throw_until.load(std::memory_order_relaxed); return u != 0 && now_ticks() < u; }
HolsterSlot holster_stowed_slot() { return (s_slot_a != HolsterSlot::None) ? s_slot_a : s_slot_b; }
bool holster_grenade_armed() { return s_grenade_armed; }
float holster_nearest_dist() { return s_nearest_dist; }
bool  holster_veto_by_proximity() { return s_near_zone; }
// A held grenade suppresses fire too: the weapon model is hidden while the hand visibly holds
// a grenade, and an invisible gun that still shoots is worse than either state alone.
bool holster_fire_suppressed() { return s_unarmed || s_grenade_armed; }
bool holster_gswitch_press_active() { const auto u = g_holster_gswitch_until.load(std::memory_order_relaxed); return u != 0 && now_ticks() < u; }
bool holster_melee_veto() {
    if (!g_cfg.holster_enabled) return false;
    if (s_near_zone || s_grenade_armed) return true;
    return (now_ticks() - s_last_action) < ms_to_ticks(g_cfg.holster_melee_veto_ms);
}

void holster_reset() {
    g_holster_swap_until.store(0, std::memory_order_relaxed);
    g_holster_throw_until.store(0, std::memory_order_relaxed);
    // An involuntary transition (vehicle, cutscene, menu, death) hands the weapon back: the game
    // itself may have swapped or re-armed us and the bookkeeping cannot be trusted past it.
    if (s_unarmed || s_grenade_armed) {
        set_weapon_hidden(false);
        if (g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER re-armed (reset)");
    }
    s_unarmed = false; s_unhide_ticks = 0;
    s_slot_a = HolsterSlot::None; s_slot_b = HolsterSlot::RightShoulder;
    if (s_grenade_armed && g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER grenade dropped (reset)");
    s_grenade_armed = false;
    s_grip_prev = false;
    s_in_zone = HolsterSlot::None;
    s_near_zone = false;
    s_have_prev = false;
    s_hand_vel = Vec3{0.0f, 0.0f, 0.0f};
    s_hand_vel_w = Vec3{0.0f, 0.0f, 0.0f};
    // The torso re-seeds from the head on the next tick: after a vehicle, a cutscene or a death
    // the old body yaw is a fact about a different situation.
    s_body_init = false;
    markers_hide_all();
}

void holster_update(float dt) {
    if (!g_cfg.enabled || !g_cfg.holster_enabled ||
        g_aim_calibrating.load(std::memory_order_relaxed) ||
        g_stick_mode_active.load(std::memory_order_relaxed)) {
        holster_reset();
        return;
    }
    if (!(dt > 0.0f) || dt > 0.25f) { s_have_prev = false; return; }

    Vec3 hpos{}; Quat hrot{};
    if (!get_pose(API::VR::get_hmd_index(), &hpos, &hrot, /*use_aim=*/false)) { s_have_prev = false; return; }
    Vec3 pos{}; Quat rot{};
    if (!get_pose(API::VR::get_right_controller_index(), &pos, &rot, /*use_aim=*/false)) { s_have_prev = false; return; }
    // A dead controller reports the origin; do not let it sit "in" a zone.
    if (std::fabs(pos.x) < 1e-6f && std::fabs(pos.y) < 1e-6f && std::fabs(pos.z) < 1e-6f) { s_have_prev = false; return; }

    // Head yaw from the head's forward vector (VR: forward = -z).
    const Vec3 fwd = quat_forward(hrot);
    const float head_yaw = std::atan2(-fwd.x, -fwd.z);

    // ---- THE TORSO LEASH (doctrine in Config.hpp). The zones used to rotate one-to-one with the
    // head, which put the pouches inside the body the moment the head turned. Now they hang on a
    // lagged BODY yaw: free look inside the dead zone, dragged beyond it, slow recentre toward the
    // head so a sustained turn brings the gear around while a glance moves nothing.
    auto wrap_rad = [](float a) {
        while (a >  3.14159265f) a -= 6.28318531f;
        while (a < -3.14159265f) a += 6.28318531f;
        return a;
    };
    if (!s_body_init) { s_body_yaw = head_yaw; s_body_init = true; }
    const float dead = g_cfg.holster_yaw_dead * 0.01745329f;
    float dyaw = wrap_rad(head_yaw - s_body_yaw);
    if (dyaw >  dead) { s_body_yaw = wrap_rad(head_yaw - dead); dyaw = dead; }
    if (dyaw < -dead) { s_body_yaw = wrap_rad(head_yaw + dead); dyaw = -dead; }
    if (g_cfg.holster_yaw_rate > 0.0f && dt > 0.0f) {
        const float step = g_cfg.holster_yaw_rate * 0.01745329f * dt;
        if (dyaw >  step)      s_body_yaw = wrap_rad(s_body_yaw + step);
        else if (dyaw < -step) s_body_yaw = wrap_rad(s_body_yaw - step);
        else                   s_body_yaw = head_yaw;
    }
    const float yaw = s_body_yaw;
    const float c = std::cos(yaw), s = std::sin(yaw);

    // ---- NECK PIVOT (doctrine in Config.hpp). The anchor is the neck, not the eyes: neck =
    // head + R_head * (0, -down, back), then lifted back to head height in the BODY frame so a
    // level head reproduces the old zone positions EXACTLY (the two offsets cancel by
    // construction) and only genuine head-about-neck rotation changes anything.
    Vec3 anchor = hpos;
    {
        const float nd = g_cfg.holster_neck_down, nb = g_cfg.holster_neck_back;
        if (nd != 0.0f || nb != 0.0f) {
            const Vec3 nw = quat_rotate(hrot, Vec3{0.0f, -nd, nb});
            anchor.x = hpos.x + nw.x + (-nb * s);
            anchor.y = hpos.y + nw.y + nd;
            anchor.z = hpos.z + nw.z + (-nb * c);
        }
    }

    // Hand in the body frame: translate from the ANCHOR, then rotate by -yaw so x=right, z=back.
    const float rx = pos.x - anchor.x, ry = pos.y - anchor.y, rz = pos.z - anchor.z;
    // Rotation about Y by -yaw: world (x, z) -> head frame.
    const float hx =  rx * c + rz * (-s);
    const float hz =  rx * s + rz * c;
    const Vec3 hand{hx, ry, hz};

    // Velocity (head frame), ~60 ms EMA, for the throw.
    if (s_have_prev) {
        const Vec3 raw{(hand.x - s_prev_hand.x) / dt, (hand.y - s_prev_hand.y) / dt, (hand.z - s_prev_hand.z) / dt};
        const float a = (dt / 0.06f > 1.0f) ? 1.0f : dt / 0.06f;
        s_hand_vel.x += (raw.x - s_hand_vel.x) * a; s_hand_vel.y += (raw.y - s_hand_vel.y) * a; s_hand_vel.z += (raw.z - s_hand_vel.z) * a;
        // World-frame twin, same subtraction, same alpha -- for the throw's DIRECTION. The
        // head-frame EMA cannot serve here: it accumulates across a rotating frame, so a value
        // built in it has no single frame to be converted back from.
        const Vec3 rel_w{pos.x - hpos.x, pos.y - hpos.y, pos.z - hpos.z};
        const Vec3 raww{(rel_w.x - s_prev_rel_w.x) / dt, (rel_w.y - s_prev_rel_w.y) / dt, (rel_w.z - s_prev_rel_w.z) / dt};
        s_hand_vel_w.x += (raww.x - s_hand_vel_w.x) * a; s_hand_vel_w.y += (raww.y - s_hand_vel_w.y) * a; s_hand_vel_w.z += (raww.z - s_hand_vel_w.z) * a;
        s_prev_rel_w = rel_w;

        // Peak-hold over a 250 ms window. The peaks landed 25 to 83 ms before release across all
        // ten measured throws, so 250 ms covers the worst of them three times over, while still
        // being short enough that the reach INTO the pouch cannot leak into the next throw.
        const float f_now = -s_hand_vel.z;
        const float s_now = std::sqrt(s_hand_vel.x * s_hand_vel.x + s_hand_vel.y * s_hand_vel.y + s_hand_vel.z * s_hand_vel.z);
        const long long tnow = now_ticks();
        if (f_now > s_peak_fwd || (tnow - s_peak_at) > ms_to_ticks(250)) {
            s_peak_fwd = f_now; s_peak_spd = s_now; s_peak_at = tnow;
            // The direction rides the peak: the throw is judged at the swing's best moment, so
            // the direction must be sampled at that same moment, not at the release instant.
            s_peak_velw = s_hand_vel_w;
        }
        else if (s_now > s_peak_spd) s_peak_spd = s_now;

        // A pose the tracker has not refreshed differences to exactly zero and drags the EMA down.
        // Count them rather than assume the rate: a probe cannot see faster than it samples.
        ++s_total_frames;
        if (s_have_raw && std::fabs(pos.x - s_prev_raw.x) < 1e-7f && std::fabs(pos.y - s_prev_raw.y) < 1e-7f
            && std::fabs(pos.z - s_prev_raw.z) < 1e-7f) ++s_stale_frames;
    }
    s_prev_raw = pos; s_have_raw = true;
    s_prev_hand = hand; s_have_prev = true;

    // Which zone, if any (nearest within radius).
    HolsterSlot zone = HolsterSlot::None; float best = 1e9f; float nearest = 1e9f;
    const HolsterSlot all[5] = {HolsterSlot::RightShoulder, HolsterSlot::LeftShoulder, HolsterSlot::RightHip, HolsterSlot::LeftChest, HolsterSlot::RightChest};
    for (HolsterSlot sl : all) {
        const Vec3 o = zone_offset(sl);
        const float d = std::sqrt((hand.x - o.x) * (hand.x - o.x) + (hand.y - o.y) * (hand.y - o.y) + (hand.z - o.z) * (hand.z - o.z));
        if (d < nearest) nearest = d;
        const bool pouch = (sl == HolsterSlot::LeftChest || sl == HolsterSlot::RightChest);
        const float rad = pouch ? g_cfg.holster_gradius : g_cfg.holster_radius;
        if (d < rad && d < best) { best = d; zone = sl; }
    }
    s_nearest_dist = nearest;
    s_near_zone = (nearest < g_cfg.holster_radius + g_cfg.holster_melee_margin);

    // ---- GRENADE VISUALS. Spawned lazily (paced), placed every tick, scale re-applied every
    // tick -- the wheel-disc lesson: anything a live cfg value controls must be re-applied on the
    // cadence the value can change. Placement: zone offsets are HEAD-frame; head->room is the
    // inverse of the rotation above (rx = hx*c + hz*s, rz = -hx*s + hz*c), then the palette's
    // room->world -- the same transform that puts the rendered gun on the hand on foot.
    if (g_cfg.holster_markers != 0) {
        if ((s_pouch_l.get() == nullptr || s_pouch_r.get() == nullptr || s_hand_g.get() == nullptr)
            && (++s_mk_tick % 120u) == 1u) {
            resolve_grenade_meshes();
            auto* mf = s_mesh_frag.get(); auto* mp = s_mesh_plasma.get();
            if (mf == nullptr) mf = mp;          // stand-ins, never a meshless component
            if (mp == nullptr) mp = mf;
            if (mf != nullptr) {
                if (auto* owner = API::get()->get_local_pawn(0)) {
                    const double ms = (double)g_cfg.holster_marker_scale * 12.5;
                    if (s_pouch_l.get() == nullptr) { if (auto* m = holster_marker_spawn_mesh(owner, mf, ms)) s_pouch_l.set(m); }
                    if (s_pouch_r.get() == nullptr) { if (auto* m = holster_marker_spawn_mesh(owner, mp, ms)) s_pouch_r.set(m); }
                    if (s_hand_g.get()  == nullptr) { if (auto* m = holster_marker_spawn_mesh(owner, mf, ms)) s_hand_g.set(m); }
                }
            }
        }
        const float capprch = 0.45f;   // pouches fade in as the hand approaches (mode 1)
        auto place_pouch = [&](TrackedObject& t, HolsterSlot sl) {
            auto* m = t.get();
            if (m == nullptr) return;
            const Vec3 o = zone_offset(sl);
            const float ddx = hand.x - o.x, ddy = hand.y - o.y, ddz = hand.z - o.z;
            const float dist = std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
            const bool show = (g_cfg.holster_markers == 2) || dist < capprch;
            holster_marker_show(m, show);
            if (!show) return;
            const Vec3 room{anchor.x + o.x * c + o.z * s, anchor.y + o.y, anchor.z + (-o.x * s + o.z * c)};
            holster_marker_place(m, holster_room_to_world(room, hpos));
            // 12.5x: holstermarkerscale keeps its meaning (0.08 = "natural size") across the move
            // from the 100 cm engine sphere to real assets authored at grenade size.
            holster_marker_scale(m, (double)g_cfg.holster_marker_scale * 12.5);
        };
        place_pouch(s_pouch_l, HolsterSlot::LeftChest);
        place_pouch(s_pouch_r, HolsterSlot::RightChest);
        // The grenade in the hand: the grab's receipt. Armed -> a sphere rides the controller;
        // released (thrown or put back) -> gone. Slightly larger than the pouch dots so the state
        // change reads at a glance.
        if (auto* m = s_hand_g.get()) {
            holster_marker_show(m, s_grenade_armed);
            if (s_grenade_armed) {
                holster_marker_place(m, holster_room_to_world(pos, hpos));
                holster_marker_scale(m, (double)g_cfg.holster_marker_scale * 12.5);
                // The hand shows the TYPE being held: frag mesh for frag, plasma for plasma.
                auto* want = (s_gtype != 0) ? s_mesh_plasma.get() : s_mesh_frag.get();
                if (want != nullptr) holster_marker_set_mesh(m, want);
            }
        }
    } else markers_hide_all();
    if (zone != s_in_zone) {
        // Entering a zone gets a tick of haptics -- the slot you can act on; leaving gets nothing.
        if (zone != HolsterSlot::None) {
            const bool full = (zone == s_slot_a) || (zone == s_slot_b) || zone == HolsterSlot::LeftChest || zone == HolsterSlot::RightChest;
            haptic(full ? 0.08f : 0.04f, full ? 0.5f : 0.25f);
            if (g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER enter %s (%s) hand=(%.2f %.2f %.2f)", slot_name(zone), full ? "full" : "empty", hand.x, hand.y, hand.z);
        }
        s_in_zone = zone;
    }

    const bool grip = right_grip_held();
    const bool pressed  = grip && !s_grip_prev;
    const bool released = !grip && s_grip_prev;
    s_grip_prev = grip;

    // A GRIP THAT CAUGHT NOTHING is the only evidence of a failed grab, and it was never
    // recorded. Log how far the hand actually was from each pouch: a near miss (d just over the
    // radius) and a wild miss (hand nowhere near) call for opposite fixes -- a bigger sphere
    // versus a moved one -- and the log line separates them without guessing.
    if (pressed && zone == HolsterSlot::None && g_cfg.holster_log) {
        const Vec3 olc = zone_offset(HolsterSlot::LeftChest), orc = zone_offset(HolsterSlot::RightChest);
        const float dlc = std::sqrt((hand.x-olc.x)*(hand.x-olc.x) + (hand.y-olc.y)*(hand.y-olc.y) + (hand.z-olc.z)*(hand.z-olc.z));
        const float drc = std::sqrt((hand.x-orc.x)*(hand.x-orc.x) + (hand.y-orc.y)*(hand.y-orc.y) + (hand.z-orc.z)*(hand.z-orc.z));
        API::get()->log_info("[Halo-CampE-UEVR] HOLSTER MISS: hand=(%.3f %.3f %.3f) frag d=%.3f plasma d=%.3f (gradius %.3f) nearest-any=%.3f",
                             hand.x, hand.y, hand.z, dlc, drc, g_cfg.holster_gradius, nearest);
    }

    if (pressed && zone != HolsterSlot::None) {
        s_last_action = now_ticks();
        if (zone == HolsterSlot::LeftChest || zone == HolsterSlot::RightChest) {
            const int want = (zone == HolsterSlot::LeftChest) ? 0 : 1;   // frag left, plasma right
            const bool live = g_unit_gvalid.load(std::memory_order_relaxed);
            const int have  = live ? g_unit_gtype.load(std::memory_order_relaxed) : s_gtype;
            const int count = live ? (want ? g_unit_gplasma.load(std::memory_order_relaxed) : g_unit_gfrag.load(std::memory_order_relaxed)) : 1;
            if (count <= 0) {
                // Nothing in that pouch: a weak buzz and no arm.
                haptic(0.05f, 0.3f);
                if (g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER %s pouch empty (frag %d plasma %d)", want ? "plasma" : "frag", g_unit_gfrag.load(), g_unit_gplasma.load());
            } else {
                if (want != have) {
                    if (g_cfg.holster_gswitch_mask != 0) {
                        g_holster_gswitch_until.store(now_ticks() + ms_to_ticks(g_cfg.holster_press_ms), std::memory_order_relaxed);
                        s_gtype = want; s_gswitch_pending = 60;
                        if (g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER grenade type -> %s (switch press; unit had %s)", want ? "plasma" : "frag", have ? "plasma" : "frag");
                    } else if (g_cfg.holster_log) {
                        API::get()->log_info("[Halo-CampE-UEVR] HOLSTER grenade type %s wanted but holstergswitchmask=0 -- not switching", want ? "plasma" : "frag");
                    }
                } else {
                    s_gtype = have;
                }
                s_grenade_armed = true;
                s_arm_at = now_ticks();
                // The WEAPON gives way to the grenade: holding both in one hand reads wrong the
                // moment the grenade is visible on the controller. Hidden here, re-asserted every
                // tick below (a swap mid-hold spawns a new visible actor), restored on release.
                set_weapon_hidden(true);
                // Clear the peak AT THE ARM, so only motion made while holding the grenade can
                // throw it. The reach in to the chest is itself a fast move; without this it
                // would sit in the window and could throw a grenade you only meant to pick up.
                s_peak_fwd = 0.0f; s_peak_spd = 0.0f; s_peak_at = now_ticks();
                s_stale_frames = 0; s_total_frames = 0;
                haptic(0.10f, 0.8f);
                if (g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER grenade armed (%s; frag %d plasma %d)", want ? "plasma" : "frag", g_unit_gfrag.load(), g_unit_gplasma.load());
            }
        } else {
            const long long now = now_ticks();
            if (now - s_last_swap > ms_to_ticks(400)) {
                s_last_swap = now;
                const bool a_here = (zone == s_slot_a), b_here = (zone == s_slot_b);
                const bool armed = !s_unarmed;
                if (armed && !a_here && !b_here) {
                    // STOW the held weapon (whichever is in hand) into this empty slot.
                    if (s_slot_a == HolsterSlot::None) s_slot_a = zone; else s_slot_b = zone;
                    s_unarmed = true;
                    set_weapon_hidden(true);
                    haptic(0.12f, 0.9f);
                    if (g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER STOW at %s -> unarmed (A@%s B@%s)", slot_name(zone), slot_name(s_slot_a), slot_name(s_slot_b));
                } else if (!armed && (a_here || b_here)) {
                    // DRAW. The game's backup needs a swap press first; afterwards it IS the
                    // current weapon, so relabel (A is always the game's current).
                    if (b_here) {
                        g_holster_swap_until.store(now + ms_to_ticks(g_cfg.holster_press_ms), std::memory_order_relaxed);
                        const HolsterSlot t = s_slot_a; s_slot_a = s_slot_b; s_slot_b = t;   // relabel
                    }
                    s_slot_a = HolsterSlot::None;
                    s_unarmed = false; s_unhide_ticks = 30;
                    set_weapon_hidden(false);
                    haptic(0.12f, 0.9f);
                    if (g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER DRAW from %s%s -> armed (A@hand B@%s)", slot_name(zone), b_here ? " (swap)" : "", slot_name(s_slot_b));
                } else if (armed && b_here) {
                    // EXCHANGE at the other gun's slot: held gun takes the slot, the other comes out.
                    g_holster_swap_until.store(now + ms_to_ticks(g_cfg.holster_press_ms), std::memory_order_relaxed);
                    s_slot_b = HolsterSlot::None; s_slot_a = zone;
                    { const HolsterSlot t = s_slot_a; s_slot_a = s_slot_b; s_slot_b = t; }     // relabel
                    s_unhide_ticks = 30;
                    haptic(0.12f, 0.9f);
                    if (g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER EXCHANGE at %s -> swap (A@hand B@%s)", slot_name(zone), slot_name(s_slot_b));
                } else if (g_cfg.holster_log) {
                    API::get()->log_info("[Halo-CampE-UEVR] HOLSTER grip at %s: nothing to do (%s, A@%s B@%s)", slot_name(zone), armed ? "armed" : "unarmed", slot_name(s_slot_a), slot_name(s_slot_b));
                }
            }
        }
    }

    // Keep the hidden state asserted: a swap or a pickup spawns a new weapon actor that arrives
    // visible, and SetActorHiddenInGame is per-actor. Cheap (one UFunction call per tick).
    if (s_unarmed || s_grenade_armed) set_weapon_hidden(true);
    else if (s_unhide_ticks > 0) { --s_unhide_ticks; set_weapon_hidden(false); }

    if (released && s_grenade_armed) {
        s_grenade_armed = false;
        s_last_action = now_ticks();
        if (!s_unarmed) { set_weapon_hidden(false); s_unhide_ticks = 30; }
        const float fwd_speed = -s_hand_vel.z;
        const float speed = std::sqrt(s_hand_vel.x * s_hand_vel.x + s_hand_vel.y * s_hand_vel.y + s_hand_vel.z * s_hand_vel.z);
        // PUT-BACK IS POSITIONAL, FULL STOP. The rule, stated from the headset, and it is the right one: "a grenade
        // put back is only, and solely, when I literally put it back in the pouch." Release inside
        // a pouch zone = put back; release anywhere else = a throw, however soft. This retires the
        // speed gate entirely (holsterthrowspeed no longer classifies anything): a speed threshold
        // was a proxy for intent, and it misread every gentle lob as a put-back. Where the hand IS
        // at release is not a proxy -- it is the intent. The zone test is the same 13 cm sphere a
        // grab uses, so putting back happens exactly where taking out does.
        const bool in_pouch = (zone == HolsterSlot::LeftChest || zone == HolsterSlot::RightChest);
        const bool threw = !in_pouch;
        if (!threw) haptic(0.06f, 0.4f);   // the pouch accepted it back
        if (threw) {
            g_holster_throw_until.store(now_ticks() + ms_to_ticks(g_cfg.holster_press_ms), std::memory_order_relaxed);
            haptic(0.10f, 1.0f);
            // AIM THE THROW ALONG THE SWING -- the melee fix, applied to the grenade. The game
            // lobs along the aim, and the aim during a throw is the flailing hand. The hold is fed
            // the world velocity SAMPLED AT THE PEAK (the same instant the gate read) through the
            // same extraction derive_ctrl_angles uses, so grenades fly where the arm sent them.
            if (g_cfg.holster_aim_hold_ms > 0) {
                const float vlen = std::sqrt(s_peak_velw.x * s_peak_velw.x + s_peak_velw.y * s_peak_velw.y
                                           + s_peak_velw.z * s_peak_velw.z);
                if (vlen > 0.2f) {
                    const float hy = wrap180(std::atan2(s_peak_velw.x, -s_peak_velw.z) * RAD2DEG
                                             + g_cfg.aim_turn * g_turn_offset.load(std::memory_order_relaxed));
                    const float hpp = std::asin(std::fmax(-1.0f, std::fmin(1.0f, s_peak_velw.y / vlen))) * RAD2DEG;
                    g_melee_aim_ctrl_yaw.store(hy, std::memory_order_relaxed);
                    g_melee_aim_ctrl_pitch.store(hpp, std::memory_order_relaxed);
                    g_melee_aim_hold_until.store(now_ticks() + ms_to_ticks(g_cfg.holster_aim_hold_ms),
                                                 std::memory_order_relaxed);
                    if (g_cfg.holster_log)
                        API::get()->log_info("[Halo-CampE-UEVR] HOLSTER THROW AIM: dir=(y%+.1f p%+.1f) |v|=%.2f held %d ms",
                                             hy, hpp, vlen, g_cfg.holster_aim_hold_ms);
                }
            }
        }
        if (g_cfg.holster_log) {
            // EVERY release logs the same four numbers, thrown or not, so the two populations are
            // directly comparable. peak-fwd is the same EMA at its best moment in the last 300 ms;
            // if the throws that failed have a healthy peak and a weak instant, the gate is
            // sampling too late. If BOTH are weak, the EMA is eating the throw. If stale frames
            // are a large share, the difference itself is under-sampled.
            const float hold_ms = (float)std::chrono::duration_cast<std::chrono::milliseconds>(
                clock_t_::duration(now_ticks() - s_arm_at)).count();
            const long long peak_age = now_ticks() - s_peak_at;
            const float peak_ms = (float)std::chrono::duration_cast<std::chrono::milliseconds>(
                clock_t_::duration(peak_age)).count();
            // Both numbers stay in the log so the next session can confirm the peak really is
            // the one that decided it, rather than taking this fit on trust.
            API::get()->log_info("[Halo-CampE-UEVR] HOLSTER %s: release fwd %.2f |v| %.2f | PEAK fwd %.2f |v| %.2f (%.0f ms ago) | gate %.2f | held %.0f ms | stale %d/%d",
                                 threw ? "THROW" : "put back (in pouch)",
                                 fwd_speed, speed, s_peak_fwd, s_peak_spd, peak_ms,
                                 g_cfg.holster_throw_speed, hold_ms, s_stale_frames, s_total_frames);
        }
    }
}

} // namespace halo
