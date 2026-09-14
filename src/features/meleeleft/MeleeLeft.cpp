#include "features/meleeleft/MeleeLeft.hpp"

#include "Config.hpp"
#include "Gesture.hpp"            // g_melee_hold_until, ReloadState
#include "Holster.hpp"            // holster_offhand_busy
#include "Math.hpp"               // ema_alpha, wrap180, RAD2DEG, clampf
#include "MotionAimControl.hpp"   // get_pose, g_turn_offset, the gesture aim hold
#include "TwoHandAim.hpp"         // two_hand_latched
#include "core/FireInput.hpp"     // g_ft_fire_at: the off hand stands down while the stock kicks
#include "core/host/GestureState.hpp"
#include "core/host/HolsterState.hpp"
#include "uevr/API.hpp"

#include <atomic>
#include <cmath>
#include <cstring>

using uevr::API;

namespace halo {

// The OFF hand's own veto. holster_melee_veto() (Holster.cpp) tests the AIM hand's zone proximity --
// correct for the aim-hand detector it was built for, and exactly wrong for a left punch: the
// right hand holding a rifle at chest height parks inside the pouch space and stood every left
// punch down (measured 2026-08-31, five punches at speed 4.4-9.4 all killed by it). This one
// tests the OFF hand's pouch proximity plus the same recent-action window; the armed-grenade
// case is holster_offhand_busy(), which the caller already checks.
bool holster_offhand_melee_veto() {
    // Holster.cpp's own state and clock, through the bridge.
    const bool& s_gnear_zone = *host::g_holster_state.gnear_zone;
    const long long& s_last_action = *host::g_holster_state.last_action;
    const auto now_ticks = host::g_holster_state.now_ticks;
    const auto ms_to_ticks = host::g_holster_state.ms_to_ticks;

    if (!g_cfg.holster_enabled) return false;
    if (s_gnear_zone) return true;
    return (now_ticks() - s_last_action) < ms_to_ticks(g_cfg.holster_melee_veto_ms);
}

// ---- OFF-HAND MELEE (meleeleft). A punch does not care which hand throws it. Same three tests
// and thresholds as the aim hand, own state, SHARED cooldown so the two detectors cannot
// double-fire one press. What differs is what the off hand does all day -- fetch magazines, pull
// grenades, brace the weapon -- each a fast, extending reach, so each gets an explicit
// stand-down here rather than a threshold tweak.
void offhand_melee_update(float dt) {
    // Gesture.cpp's own state and clock, through the bridge: the same objects under the same names.
    long long& s_cooldown_until = *host::g_gesture_state.cooldown_until;
    const ReloadState& s_reload = *host::g_gesture_state.reload;
    const float REST_SPEED_MPS = host::g_gesture_state.rest_speed_mps;
    const auto now_ticks = host::g_gesture_state.now_ticks;
    const auto ms_to_ticks = host::g_gesture_state.ms_to_ticks;

    static Vec3      s2_prev_rel{};
    static float     s2_prev_reach = 0.0f;
    static bool      s2_have = false;
    static Vec3      s2_vel{};
    static float     s2_ext = 0.0f;
    static long long s2_last = 0;
    static bool      s2_in_swing = false;
    static float     s2_pk_spd = 0.0f, s2_pk_ext = 0.0f, s2_pk_reach = 0.0f;
    static Vec3      s2_rel0{};          // hand-rel-head where this swing began
    static float     s2_pk_disp = 0.0f;  // furthest it has travelled from there

    if (!g_cfg.melee_left) { s2_have = false; return; }

    // The gates above this call stop the whole tick (stick mode, calibration), so a gap in our
    // own run cadence means one of them was engaged -- reseed instead of differentiating across it.
    const long long nowt = now_ticks();
    if (s2_last != 0 && nowt - s2_last > ms_to_ticks(250)) s2_have = false;
    s2_last = nowt;

    const auto idx = g_cfg.aim_left_hand ? API::VR::get_right_controller_index()
                                         : API::VR::get_left_controller_index();
    Vec3 pos{}; Quat rot{};
    Vec3 hpos{}; Quat hrot{};
    if (!get_pose(idx, &pos, &rot, /*use_aim=*/false) ||
        !get_pose(API::VR::get_hmd_index(), &hpos, &hrot, /*use_aim=*/false)) {
        s2_have = false;
        return;
    }

    const Vec3  rel{pos.x - hpos.x, pos.y - hpos.y, pos.z - hpos.z};
    const float reach = std::sqrt(rel.x * rel.x + rel.y * rel.y + rel.z * rel.z);
    if (reach > g_cfg.melee_max_reach) {
        s2_have = false;
        s2_vel = Vec3{0.0f, 0.0f, 0.0f};
        s2_ext = 0.0f;
        return;
    }
    if (!s2_have) {
        s2_prev_rel = rel;
        s2_prev_reach = reach;
        s2_have = true;
        return;
    }

    const Vec3 raw{(rel.x - s2_prev_rel.x) / dt,
                   (rel.y - s2_prev_rel.y) / dt,
                   (rel.z - s2_prev_rel.z) / dt};
    const float ext_raw = (reach - s2_prev_reach) / dt;
    s2_prev_rel = rel;
    s2_prev_reach = reach;

    const float a = ema_alpha(g_cfg.melee_tau_ms, dt);
    s2_vel.x += (raw.x - s2_vel.x) * a;
    s2_vel.y += (raw.y - s2_vel.y) * a;
    s2_vel.z += (raw.z - s2_vel.z) * a;
    s2_ext   += (ext_raw - s2_ext) * a;

    const float speed = std::sqrt(s2_vel.x * s2_vel.x + s2_vel.y * s2_vel.y + s2_vel.z * s2_vel.z);
    if (speed > g_cfg.melee_max_speed) {
        s2_have = false;
        s2_vel = Vec3{0.0f, 0.0f, 0.0f};
        s2_ext = 0.0f;
        return;
    }

    // Swing segmentation, same shape as the main hand's: peaks over one continuous motion,
    // reported when the hand settles, so a swing that never fires still leaves its numbers.
    float disp = 0.0f;
    if (speed > REST_SPEED_MPS) {
        if (!s2_in_swing) s2_rel0 = rel;
        s2_in_swing = true;
        const float ddx = rel.x - s2_rel0.x, ddy = rel.y - s2_rel0.y, ddz = rel.z - s2_rel0.z;
        disp = std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
        if (speed  > s2_pk_spd)   s2_pk_spd   = speed;
        if (s2_ext > s2_pk_ext)   s2_pk_ext   = s2_ext;
        if (reach  > s2_pk_reach) s2_pk_reach = reach;
        if (disp   > s2_pk_disp)  s2_pk_disp  = disp;
    } else if (s2_in_swing) {
        s2_in_swing = false;
        if (g_cfg.melee_log) {
            const bool would = (s2_pk_spd   >= g_cfg.melee_speed) &&
                               (s2_pk_ext   >= g_cfg.melee_ext ||
                                (g_cfg.melee_disp > 0.0f && s2_pk_disp >= g_cfg.melee_disp)) &&
                               (s2_pk_reach >= g_cfg.melee_reach);
            API::get()->log_info(
                "[Halo-CampE-UEVR] MELEE swing (OFF HAND)  speed=%.2f  ext=%.2f  reach=%.2f  disp=%.2f   "
                "(need spd>=%.2f ext>=%.2f reach>=%.2f) -- %s",
                s2_pk_spd, s2_pk_ext, s2_pk_reach, s2_pk_disp,
                g_cfg.melee_speed, g_cfg.melee_ext, g_cfg.melee_reach,
                would ? "FIRED" : "no");
        }
        s2_pk_spd = 0.0f; s2_pk_ext = 0.0f; s2_pk_reach = 0.0f; s2_pk_disp = 0.0f;
    }

    if (nowt < s_cooldown_until)   return;
    if (speed < g_cfg.melee_speed) return;
    // Extension OR travel. A vertical chop arcs around the shoulder: the hand-to-head distance
    // barely grows (measured 2026-08-31: ext 1.43 and 1.80 against the 2.20 gate) while the hand
    // itself travels over a metre (disp 1.13, 0.77). Ambient jitter and the gunstock's kick both
    // stay under 0.1 m, so travel separates a chop from noise as cleanly as extension separates
    // a punch from it.
    const bool travelled = g_cfg.melee_disp > 0.0f && disp >= g_cfg.melee_disp;
    if (s2_ext < g_cfg.melee_ext && !travelled) return;
    if (reach < g_cfg.melee_reach) return;

    // GUNSTOCK KICK vs PUNCH. While the trigger is down (+ a tail) with the ForceTube on, the
    // stock's kick jolts the off hand into threshold-clearing VELOCITY (2.31 and 2.46 against the
    // 2.20 gate, measured) without the hand actually going anywhere. A real punch TRAVELS. So the
    // shot window demands displacement since the swing began -- not a harder swing, which is how
    // controllers meet door frames.
    if (g_cfg.force_tube && g_cfg.melee_shot_ms > 0 &&
        nowt - g_ft_fire_at.load(std::memory_order_relaxed) < ms_to_ticks(g_cfg.melee_shot_ms) &&
        disp < g_cfg.melee_shot_dist) return;

    // ---- THE OFF HAND'S DAY JOBS, each a hard stand-down. A swing that cleared every numeric
    // gate and dies here is invisible without a name, so each one says so in the log.
    const char* job = nullptr;
    if      (s_reload != ReloadState::Idle)                   job = "reload in progress";
    else if (two_hand_latched())    job = "two-hand brace";
    else if (holster_offhand_busy())                          job = "grenade in pouch/hand";
    else if (holster_offhand_melee_veto())                    job = "holster veto";
    if (job != nullptr) {
        if (g_cfg.melee_log) {
            API::get()->log_info("[Halo-CampE-UEVR] MELEE (OFF HAND) stood down by %s: "
                                 "speed=%.2f ext=%.2f reach=%.2f",
                                 job, speed, s2_ext, reach);
        }
        if (holster_offhand_melee_veto()) s_cooldown_until = nowt + ms_to_ticks(150);
        return;
    }

    g_melee_hold_until.store(nowt + ms_to_ticks(g_cfg.melee_hold_ms), std::memory_order_relaxed);
    s_cooldown_until = nowt + ms_to_ticks(g_cfg.melee_cooldown_ms);

    // Aim hold along the punch, mode 1 only -- mode 0 wants the swing-start aim, which this
    // detector does not track; the shipped mode is 1.
    if (g_cfg.melee_aim_mode == 1 && g_cfg.melee_aim_hold_ms > 0) {
        const float hy = wrap180(std::atan2(s2_vel.x, -s2_vel.z) * RAD2DEG
                                 + g_cfg.aim_turn * g_turn_offset.load(std::memory_order_relaxed));
        const float hp = std::asin(std::fmax(-1.0f, std::fmin(1.0f, s2_vel.y / speed))) * RAD2DEG;
        g_melee_aim_ctrl_yaw.store(hy, std::memory_order_relaxed);
        g_melee_aim_ctrl_pitch.store(hp, std::memory_order_relaxed);
        g_melee_aim_hold_until.store(nowt + ms_to_ticks(g_cfg.melee_aim_hold_ms),
                                     std::memory_order_relaxed);
    }
    if (g_cfg.melee_log) {
        API::get()->log_info("[Halo-CampE-UEVR] MELEE FIRED (OFF HAND): speed=%.2f ext=%.2f reach=%.2f",
                             speed, s2_ext, reach);
    }
}

bool meleeleft_parse_key(const char* key, const char* val, double v) {
    (void)val;
    if (_stricmp(key, "meleeleft")      == 0) { g_cfg.melee_left     = (v != 0.0); return true; }
    if (_stricmp(key, "meleeshotms")    == 0) { g_cfg.melee_shot_ms  = (int)clampf((float)v, 0.0f, 2000.0f); return true; }
    if (_stricmp(key, "meleeshotdist")  == 0) { g_cfg.melee_shot_dist = clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "meleedisp")      == 0) { g_cfg.melee_disp     = clampf((float)v, 0.0f, 2.0f); return true; }
    return false;
}

constinit const FeatureHooks kMeleeLeftHooks{
    .key                   = "meleeleft",
    .parse_key             = &meleeleft_parse_key,
    .gesture_melee_offhand = &offhand_melee_update,
};

} // namespace halo
