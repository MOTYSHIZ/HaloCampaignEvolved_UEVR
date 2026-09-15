#include "core/fixes/MeleeInstruments.hpp"

#include "Config.hpp"
#include "Gesture.hpp"
#include "Holster.hpp"            // holster_melee_veto and the proximity evidence it logs
#include "Math.hpp"               // wrap180, RAD2DEG
#include "MotionAimControl.hpp"   // g_desired_*, derive_ctrl_angles, g_ref_*, g_turn_offset, the gesture aim hold
#include "core/host/GestureState.hpp"
#include "uevr/API.hpp"

#include <atomic>
#include <cmath>

using uevr::API;

namespace halo {

namespace {

// ---- MEASUREMENT (meleelog). Two different faults both fit "the swing melee does not connect
// but the stick click does", and they want opposite fixes:
//   A. the strike never fires at all, because the holster veto stands the detector down;
//   B. it fires, but the AIM has been dragged off the target by the swing itself, so Halo's
//      lunge tracks empty air. Nothing in this plugin freezes or compensates aim during a melee.
// The aim setpoint is captured when the swing starts moving and compared at the instant the
// strike fires. Which one is happening is a number, not an argument.
float s_swing_yaw0 = 0.0f, s_swing_pitch0 = 0.0f;
bool  s_swing_aim_ok = false;
// The CONTROLLER angles at the swing's start, which is what the aim law actually consumes.
// Freezing the input rather than overriding the output means the stick loop, the direct write and
// the sim driver all inherit the hold from one place instead of three that can drift apart.
float s_swing_ctrl_yaw = 0.0f, s_swing_ctrl_pitch = 0.0f;
bool  s_swing_ctrl_ok = false;
// ---- DOES THE HOLD ACTUALLY HOLD?
// The "AIM MOVED" figure on the FIRED line is sampled before the hold has been applied, so it
// measures the drift the hold is meant to cancel and can never show whether it was cancelled.
// Confirming the fix needs a reading from INSIDE the hold window: this fires one deferred log a
// short time after the strike, comparing the live setpoint against the swing's start. Near zero
// means the aim is being held where the punch was launched from. Anything else means it is not,
// and no amount of the fix looking right in the source changes that.
float     s_hold_ref_yaw = 0.0f, s_hold_ref_pitch = 0.0f;
long long s_hold_check_at = 0;
bool      s_hold_was_armed = false;
int   s_veto_count = 0;

}  // namespace

void melee_hold_check() {
    // Gesture.cpp's clock, through the bridge.
    const auto now_ticks = host::g_gesture_state.now_ticks;

    if (s_hold_check_at != 0 && now_ticks() >= s_hold_check_at) {
        s_hold_check_at = 0;
        if (g_cfg.melee_log) {
            float dy = g_desired_yaw.load(std::memory_order_relaxed) - s_hold_ref_yaw;
            while (dy > 180.0f) dy -= 360.0f;
            while (dy < -180.0f) dy += 360.0f;
            const float dp = g_desired_pitch.load(std::memory_order_relaxed) - s_hold_ref_pitch;
            API::get()->log_info("[Halo-CampE-UEVR] MELEE HOLD CHECK: armed=%s -- aim sits %+.1f yaw "
                                 "%+.1f pitch from where the swing began (0 = held on target)",
                                 s_hold_was_armed ? "yes" : "NO", dy, dp);
        }
    }
}

void melee_swing_moving(bool s_in_swing) {
        // The aim reference is taken on the FIRST tick of the swing -- where you were pointing
        // when you decided to hit something, which is the direction Halo's melee should lunge in.
        if (!s_in_swing) {
            s_swing_yaw0   = g_desired_yaw.load(std::memory_order_relaxed);
            s_swing_pitch0 = g_desired_pitch.load(std::memory_order_relaxed);
            s_swing_aim_ok = true;
            s_swing_ctrl_ok = derive_ctrl_angles(&s_swing_ctrl_yaw, &s_swing_ctrl_pitch);
        }
}

bool melee_vetoed(long long now, float speed, float reach) {
    // Gesture.cpp's own state and clock, through the bridge: the same objects under the same names.
    long long& s_cooldown_until = *host::g_gesture_state.cooldown_until;
    const float& s_ext = *host::g_gesture_state.ext;
    const auto ms_to_ticks = host::g_gesture_state.ms_to_ticks;

    // HOLSTER VETO: a reach over the shoulder is a strike to this detector. Stand down while the
    // hand is in or near a holster zone and briefly after any holster action.
    if (holster_melee_veto()) {
        ++s_veto_count;
        if (g_cfg.melee_log) {
            // WHICH veto, and by how much. Proximity covers holster_radius + margin around all
            // FIVE zones at once, which is a large volume of the space directly in front of the
            // chest -- exactly where a punch travels. If strikes are dying here, the number says
            // so and says by what margin.
            API::get()->log_info("[Halo-CampE-UEVR] MELEE VETOED (#%d) by %s: nearest zone %.3f m (veto radius %.3f) speed=%.2f ext=%.2f reach=%.2f",
                                 s_veto_count,
                                 holster_veto_by_proximity() ? "PROXIMITY" : "recent-action/grenade",
                                 holster_nearest_dist(),
                                 g_cfg.holster_radius + g_cfg.holster_melee_margin,   // holstermeleemargin
                                 speed, s_ext, reach);
        }
        s_cooldown_until = now + ms_to_ticks(150);
        return true;
    }

    return false;
}

bool melee_fired(long long now, float speed, float reach) {
    // Gesture.cpp's own state and clock, through the bridge.
    const Vec3& s_vel = *host::g_gesture_state.vel;
    const float& s_ext = *host::g_gesture_state.ext;
    const auto ms_to_ticks = host::g_gesture_state.ms_to_ticks;

    // PIN THE AIM BACK to where it was when the swing began, for as long as the game needs to
    // resolve the strike. The snap is instantaneous rather than servo-limited because aimdirect is
    // the live path (confirmed driving in the same session's log): the rotator is assigned, not
    // steered, so there is no turn rate to wait on. With the stick loop it would degrade to a fast
    // turn instead of a jump, which is worse but not wrong.
    // Choose what the hold points at. Mode 1: the swing itself. s_vel is the hand-minus-head
    // velocity in the VR WORLD frame -- the same frame derive_ctrl_angles extracts its angles
    // from -- and `speed` has already cleared melee_speed, so the direction is well-defined.
    // Locomotion is subtracted by construction (rel = hand - head), so this is the punch's own
    // travel, not the player walking.
    float hy = s_swing_ctrl_yaw, hp = s_swing_ctrl_pitch;
    bool  hold_src_ok = s_swing_ctrl_ok;
    if (g_cfg.melee_aim_mode == 1) {
        hy = wrap180(std::atan2(s_vel.x, -s_vel.z) * RAD2DEG
                     + g_cfg.aim_turn * g_turn_offset.load(std::memory_order_relaxed));
        hp = std::asin(std::fmax(-1.0f, std::fmin(1.0f, s_vel.y / speed))) * RAD2DEG;
        hold_src_ok = true;
    }
    s_hold_was_armed = (g_cfg.melee_aim_hold_ms > 0 && hold_src_ok);
    // Mode 1's hold is already stored, with these same values, by the author's strike path in Gesture.cpp
    // just before this slot runs (same condition, speed included). Only mode 0's swing-start hold is ours.
    const bool stored_by_author = g_cfg.melee_aim_mode == 1 && speed > 0.0001f;
    if (s_hold_was_armed && !stored_by_author) {
        g_melee_aim_ctrl_yaw.store(hy, std::memory_order_relaxed);
        g_melee_aim_ctrl_pitch.store(hp, std::memory_order_relaxed);
        g_melee_aim_hold_until.store(now + ms_to_ticks(g_cfg.melee_aim_hold_ms),
                                     std::memory_order_relaxed);
    }
    // Read back from inside the window rather than at its edges: 80 ms in, the hold is at full
    // weight and the ramp has not begun, so the number is the hold's own doing and nothing else's.
    // The reference is the INTENDED setpoint -- the same reference-plus-delta arithmetic the hold
    // consumers apply -- so "0 = held on target" stays true in both modes. Comparing against the
    // swing's start was only correct when the start WAS the target.
    s_hold_ref_yaw   = s_hold_was_armed
                     ? g_ref_aim_yaw.load(std::memory_order_relaxed)
                       + wrap180(hy - g_ref_ctrl_yaw.load(std::memory_order_relaxed))
                     : s_swing_yaw0;
    s_hold_ref_pitch = s_hold_was_armed
                     ? g_ref_aim_pitch.load(std::memory_order_relaxed)
                       + (hp - g_ref_ctrl_pitch.load(std::memory_order_relaxed))
                     : s_swing_pitch0;
    s_hold_check_at = now + ms_to_ticks(80);

    if (g_cfg.melee_log) {
        // HOW FAR THE AIM MOVED during the swing. The synthesised button is byte-identical to the
        // stick click, so if one connects and the other does not, the difference is not the press
        // -- it is the state of the game at the moment of the press, and aim is the obvious part
        // of that state which the swing itself changes. Degrees, since the swing began.
        float dyaw = 0.0f, dpitch = 0.0f;
        if (s_swing_aim_ok) {
            dyaw = g_desired_yaw.load(std::memory_order_relaxed) - s_swing_yaw0;
            while (dyaw > 180.0f) dyaw -= 360.0f;
            while (dyaw < -180.0f) dyaw += 360.0f;
            dpitch = g_desired_pitch.load(std::memory_order_relaxed) - s_swing_pitch0;
        }
        API::get()->log_info("[Halo-CampE-UEVR] MELEE FIRED: speed=%.2f ext=%.2f reach=%.2f "
                             "mask=0x%04X hold=%dms | AIM MOVED yaw %+.1f deg pitch %+.1f deg "
                             "since the swing began | mode=%d swing-dir=(y%+.1f p%+.1f)",
                             speed, s_ext, reach, (unsigned)g_cfg.melee_mask, g_cfg.melee_hold_ms,
                             dyaw, dpitch, g_cfg.melee_aim_mode,
                             g_cfg.melee_aim_mode == 1 ? hy : s_swing_ctrl_yaw,
                             g_cfg.melee_aim_mode == 1 ? hp : s_swing_ctrl_pitch);
    }
    return true;
}

} // namespace halo
