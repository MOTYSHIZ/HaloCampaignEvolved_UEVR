#include "Gesture.hpp"
#include "TwoHand.hpp"
#include "Holster.hpp"

#include "Config.hpp"
#include "Math.hpp"
#include "MotionAimControl.hpp"
#include "WeaponCalib.hpp"

#include <string>

#include <chrono>
#include <cmath>

using uevr::API;

namespace halo {

std::atomic<long long> g_melee_hold_until{0};
std::atomic<long long> g_reload_hold_until{0};
// g_pad_buttons lives in TwoHand.cpp; the gesture aim-hold atomics live in MotionAimControl.cpp
// beside the derivations they feed. This module consumes both through their headers.

namespace {

using clock_t_ = std::chrono::steady_clock;

inline long long now_ticks() { return clock_t_::now().time_since_epoch().count(); }
inline long long ms_to_ticks(int ms) {
    return std::chrono::duration_cast<clock_t_::duration>(std::chrono::milliseconds(ms)).count();
}

// Previous sample. Everything is stored HEAD-RELATIVE: s_prev_rel is (hand - head), which removes
// whole-body motion before any derivative is taken.
bool  s_have_prev  = false;
Vec3  s_prev_rel{0.0f, 0.0f, 0.0f};
float s_prev_reach = 0.0f;

// Smoothed relative velocity, and the smoothed rate the arm is extending at.
Vec3  s_vel{0.0f, 0.0f, 0.0f};
float s_ext = 0.0f;

// Refractory deadline, same clock as the hold deadline.
long long s_cooldown_until = 0;

// Peaks since the last time a swing settled, purely for melee_log. Reported when the hand comes
// back to rest so one line describes one swing, rather than a line per tick.
float s_peak_speed = 0.0f;
float s_peak_ext   = 0.0f;
float s_peak_reach = 0.0f;
bool  s_in_swing   = false;
// ---- MEASUREMENT (meleelog). Two different faults both fit "the swing melee does not connect
// but the stick click does", and they want opposite fixes:
//   A. the strike never fires at all, because the holster veto stands the detector down;
//   B. it fires, but the AIM has been dragged off the target by the swing itself, so Halo's
//      lunge tracks empty air. Nothing in this plugin freezes or compensates aim during a melee.
// The aim setpoint is captured when the swing starts moving and compared at the instant the
// strike fires. Which one is happening is a number, not an argument.
// The weapon whose magazine the in-flight reload belongs to (weapon_key at the Idle exit).
std::string s_reload_weapon;
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

// Below this relative speed the hand counts as at rest and a swing is considered over. Not
// configurable on purpose: it exists to segment the LOG, and a knob to tune the tuning instrument
// is not worth the config surface.
constexpr float REST_SPEED_MPS = 0.35f;

// ---- RELOAD STATE ------------------------------------------------------------------------------
ReloadState s_reload = ReloadState::Idle;
unsigned short s_prev_buttons = 0;

// How long the synthesised reload press is held. Same reasoning as melee_hold_ms: one poll can
// land between the game's own input samples and be missed entirely.
constexpr int RELOAD_HOLD_MS = 90;

const char* state_name(ReloadState s) {
    switch (s) {
        case ReloadState::MagOut:  return "MAG_OUT";
        case ReloadState::MagHeld: return "MAG_HELD";
        default:                   return "IDLE";
    }
}

void set_state(ReloadState next, const char* why) {
    if (s_reload == next) return;
    if (g_cfg.reload_log) {
        API::get()->log_info("[Halo-CampE-UEVR] RELOAD %s -> %s (%s)",
                             state_name(s_reload), state_name(next), why);
    }
    s_reload = next;
}

} // namespace

bool melee_press_active() {
    const long long until = g_melee_hold_until.load(std::memory_order_relaxed);
    return until != 0 && now_ticks() < until;
}

bool reload_press_active() {
    const long long until = g_reload_hold_until.load(std::memory_order_relaxed);
    return until != 0 && now_ticks() < until;
}

// ---- TAP / HOLD TRACKING -----------------------------------------------------------------------
//
// Lives in the XInput hook's cadence rather than the ~32 Hz tick, because a hold threshold judged
// at 31 ms granularity feels arbitrary to the hand. Only clocks and atomics here -- no reflection,
// no allocation, per the rule on that callback.
namespace {
std::atomic<long long>  s_reload_down_at{0};    // when the reload button went down, 0 = up
std::atomic<bool>       s_reload_passing{false};// hold threshold crossed: stop swallowing
std::atomic<bool>       s_reload_tap{false};    // a completed tap, waiting for the tick to consume
static float            s_grab_y = 0.0f;         // left-hand height (VR y) at the belt grab
static int              s_seat_ticks = 0;        // consecutive ticks the seat condition has held
}

void reload_note_buttons(unsigned short buttons) {
    if (!g_cfg.reload_vr || g_cfg.reload_mask == 0) {
        s_reload_down_at.store(0, std::memory_order_relaxed);
        s_reload_passing.store(false, std::memory_order_relaxed);
        return;
    }

    const bool down = (buttons & (unsigned short)g_cfg.reload_mask) != 0;
    const long long at = s_reload_down_at.load(std::memory_order_relaxed);

    if (down && at == 0) {
        s_reload_down_at.store(now_ticks(), std::memory_order_relaxed);
        s_reload_passing.store(false, std::memory_order_relaxed);
        return;
    }
    if (down) {
        // Still held. Once past the threshold this is an Interact / Enter Vehicle press, not a
        // reload, so stop withholding it and let the game have the rest of the hold.
        if (!s_reload_passing.load(std::memory_order_relaxed) &&
            now_ticks() - at >= ms_to_ticks(g_cfg.reload_hold_ms)) {
            s_reload_passing.store(true, std::memory_order_relaxed);
        }
        return;
    }

    // Released. A short press that was never promoted to a hold is a tap: the reload gesture.
    if (at != 0) {
        if (!s_reload_passing.load(std::memory_order_relaxed)) {
            s_reload_tap.store(true, std::memory_order_relaxed);
        }
        s_reload_down_at.store(0, std::memory_order_relaxed);
        s_reload_passing.store(false, std::memory_order_relaxed);
    }
}

bool reload_swallow_reload_button() {
    if (!g_cfg.reload_vr || g_cfg.reload_mask == 0) return false;
    // Withhold only while the press is still short enough to be a tap.
    return s_reload_down_at.load(std::memory_order_relaxed) != 0 &&
           !s_reload_passing.load(std::memory_order_relaxed);
}

bool reload_swallow_grip() {
    if (!g_cfg.reload_vr || g_cfg.reload_grip_mask == 0) return false;
    // EXCLUSIVE: the grip is ours outright, not just while a magazine is in play. It is the VR
    // interaction button -- magazine grabs now, weapon holding later -- and a button that lobs a
    // grenade when you reach for something is not one you can build physical interactions on.
    //
    // The cost is explicit and worth stating: with this on, THE GAME NEVER SEES THE GRIP, so
    // grenades have no binding until one is given to them elsewhere.
    if (g_cfg.grip_exclusive) return true;
    // Otherwise the narrower rule: only while a magazine is expected or held.
    return s_reload != ReloadState::Idle;
}

ReloadState reload_state() { return s_reload; }

bool reload_fire_suppressed() {
    return g_cfg.reload_suppress_fire && s_reload != ReloadState::Idle;
}

// The reload half of the tick. Separate from the melee detector because it is a state machine over
// BUTTONS and ZONES, not a derivative over velocity -- sharing a function would only tangle them.
static void reload_update(const Vec3& hand_r, const Vec3& head) {
    if (!g_cfg.enabled || !g_cfg.reload_vr) {
        if (s_reload != ReloadState::Idle) set_state(ReloadState::Idle, "disabled");
        return;
    }

    const unsigned short btn = g_pad_buttons.load(std::memory_order_relaxed);
    const unsigned short rising = (unsigned short)(btn & ~s_prev_buttons);
    s_prev_buttons = btn;

    // ---- A WEAPON CHANGE CANCELS THE RELOAD IN FLIGHT. The gesture is a state machine and
    // nothing told it the weapon changed: pull the mag on gun A, swap to gun B, and the fire
    // suppression follows the PLAYER, not the gun -- the player could not shoot the fresh weapon
    // until he performed a gesture that belonged to the previous one. The magazine you pulled was
    // gun A's; gun B arrives in whatever state the game has it. So the reload records which
    // weapon's mag came out, and the moment a DIFFERENT non-empty weapon is in hand the state
    // returns to Idle. Non-empty on both sides on purpose: a transient empty key (weapon lowered
    // for a frame, swap animation) must not cancel a legitimate reload of the same gun.
    if (s_reload != ReloadState::Idle) {
        const std::string wk = weapon_key();
        if (!wk.empty() && !s_reload_weapon.empty() && wk != s_reload_weapon) {
            set_state(ReloadState::Idle, "weapon changed, reload cancelled");
        }
    }

    // A completed TAP, not a rising edge. The rising edge cannot tell a reload from the start of
    // an Interact hold, and treating them alike is what cost a chapter of vehicles.
    const bool reload_pressed = s_reload_tap.exchange(false, std::memory_order_relaxed);
    const bool grip_held      = (g_cfg.reload_grip_mask != 0) &&
                                ((btn & (unsigned short)g_cfg.reload_grip_mask) != 0);

    // The LEFT hand is the one that fetches. If aim is left-handed the roles swap, so this asks
    // for "the hand that is not aiming" rather than hardcoding a side.
    const auto lidx = g_cfg.aim_left_hand ? API::VR::get_right_controller_index()
                                          : API::VR::get_left_controller_index();
    Vec3 hand_l{}; Quat lrot{};
    bool have_left = get_pose(lidx, &hand_l, &lrot, /*use_aim=*/false);
    // DEAD-POSE GATE. get_pose passes a sleeping/glitched controller as (0,0,0) -- the room origin,
    // which sits at the head on this rig. From there the well is within 30 cm and "lifted" reads
    // +60 cm, so ONE bad frame seats the magazine: logged 2026-08-16 12:32 as a seat 164 ms after
    // a grab that was 67 cm from the well. Same rule the palette publisher applies to the aim hand.
    if (have_left) {
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

    switch (s_reload) {
    case ReloadState::Idle:
        if (reload_pressed) {
            // Deliberately does NOT forward the press. The game is told to reload only when the
            // magazine goes in -- that deferral IS the feature.
            s_reload_weapon = weapon_key();   // whose magazine this is (see the swap-cancel above)
            set_state(ReloadState::MagOut, "reload pressed, mag dropped");
        }
        break;

    case ReloadState::MagOut: {
        if (reload_pressed && g_cfg.reload_cancel) {
            set_state(ReloadState::Idle, "cancelled, mag re-seated");
            break;
        }
        if (!have_left) break;
        // Belt zone: below the head, and near the body's vertical axis. Y is up in this space --
        // the same convention quat_forward assumes for the VR frame.
        const float drop = head.y - hand_l.y;
        const float dx = hand_l.x - head.x, dz = hand_l.z - head.z;
        const float horiz = std::sqrt(dx * dx + dz * dz);
        if (grip_held && drop >= g_cfg.reload_belt_drop && horiz <= g_cfg.reload_belt_radius) {
            s_grab_y = hand_l.y;   // where the mag was picked up: the lift gate measures from here
            set_state(ReloadState::MagHeld, "grabbed from belt");
        }
        break;
    }

    case ReloadState::MagHeld: {
        if (!grip_held) {
            set_state(ReloadState::MagOut, "grip released, dropped it");
            break;
        }
        if (!have_left) break;
        // THE WELL, not the hand: a point reload_well_fwd along the aim direction from the aim
        // hand -- the magazine well on any rifle, the trigger guard on a pistol -- so the mag has
        // to be brought to the gun rather than merely near the other hand. And the LIFT GATE: the
        // mag must have risen since it was grabbed. Without both, the log (2026-08-16 12:07) shows
        // the reload firing 214-224 ms after the belt grab, at the hip.
        Vec3 well = hand_r;
        {
            const auto ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                                  : API::VR::get_right_controller_index();
            Vec3 apos{}; Quat aq{};
            if (ridx >= 0 && get_pose(ridx, &apos, &aq, /*use_aim=*/true)) {
                const Vec3 f = quat_forward(apply_aim_fix(aq));
                well = Vec3{hand_r.x + f.x * g_cfg.reload_well_fwd,
                            hand_r.y + f.y * g_cfg.reload_well_fwd,
                            hand_r.z + f.z * g_cfg.reload_well_fwd};
            }
        }
        const float ddx = hand_l.x - well.x, ddy = hand_l.y - well.y, ddz = hand_l.z - well.z;
        const float join = std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
        const bool lifted = (hand_l.y - s_grab_y) >= g_cfg.reload_lift;
        if (g_cfg.reload_log) {
            static uint32_t s_rl = 0;
            if ((s_rl++ % 15u) == 0u)
                API::get()->log_info("[Halo-CampE-UEVR] RELOAD held: mag-to-well=%.0fcm lifted=%.0fcm (need <=%.0f, >=%.0f)",
                                     join * 100.0f, (hand_l.y - s_grab_y) * 100.0f,
                                     g_cfg.reload_join_dist * 100.0f, g_cfg.reload_lift * 100.0f);
        }
        // DEBOUNCE: the seat must hold for a few consecutive ticks. A physical insert is a held
        // contact, a tracking glitch is one frame.
        if (join <= g_cfg.reload_join_dist && lifted) {
            if (++s_seat_ticks >= 4) {
                s_seat_ticks = 0;
                g_reload_hold_until.store(now_ticks() + ms_to_ticks(RELOAD_HOLD_MS),
                                          std::memory_order_relaxed);
                set_state(ReloadState::Idle, "magazine seated, reload fired");
            }
        } else {
            s_seat_ticks = 0;
        }
        break;
    }
    }
}

void gesture_reset() {
    // The two-handed hold rides along. It is latched on a button and blended into aim, so a
    // transition the player did not choose (kill switch, stick mode, a tracking stall) must drop it
    // too -- otherwise the aim stays blended toward a support hand nothing is tracking any more.
    two_hand_reset();
    g_melee_hold_until.store(0, std::memory_order_relaxed);
    // Releasing the reload state is not optional: leaving it in MAG_OUT would keep the trigger
    // suppressed with no way for the player to notice why. Same rule as the arm hide.
    g_reload_hold_until.store(0, std::memory_order_relaxed);
    if (s_reload != ReloadState::Idle) set_state(ReloadState::Idle, "gesture reset");
    s_prev_buttons = 0;
    s_have_prev = false;
    s_vel = Vec3{0.0f, 0.0f, 0.0f};
    s_ext = 0.0f;
    s_in_swing = false;
    s_peak_speed = 0.0f;
    s_peak_ext = 0.0f;
    s_peak_reach = 0.0f;
}

void gesture_update(float dt) {
    // The deferred hold check runs BEFORE the gates, because the gates are exactly the states that
    // would swallow it (a melee into a vehicle, a death) and a check that only reports when nothing
    // went wrong is not a check.
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

    // ---- GATES. Every one of these is a state the player did not ask to melee in.
    //
    // stick mode covers vehicles, cutscenes and death; calibration means the player is holding
    // the controller still against a frozen reticle and any motion is measurement, not intent.
    if (!g_cfg.enabled || !g_cfg.melee_swing ||
        g_aim_calibrating.load(std::memory_order_relaxed) ||
        g_stick_mode_active.load(std::memory_order_relaxed)) {
        gesture_reset();
        return;
    }

    // A stalled or absurd dt turns a stationary hand into a teleport. Drop the history rather
    // than differentiate across the gap -- the next tick re-seeds cleanly.
    if (!(dt > 0.0f) || dt > 0.25f) {
        s_have_prev = false;
        return;
    }

    // Melee follows the AIM hand: whichever hand holds the gun is the one you would hit with.
    const auto ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                          : API::VR::get_right_controller_index();

    // Grip pose, not aim pose. get_aim_pose() is documented in Plugin.cpp as producing
    // teleport-scale travel readings, which is exactly the signal this must not confuse for a
    // strike.
    Vec3 pos{}; Quat rot{};
    if (!get_pose(ridx, &pos, &rot, /*use_aim=*/false)) {
        s_have_prev = false;
        return;
    }

    // HEAD-RELATIVE, and the head pose is REQUIRED -- no fail-open here. Without it there is no
    // extension measurement at all, and the previous version's fallback (assume the gate passes)
    // is precisely how it ended up firing on fast aiming.
    Vec3 hpos{}; Quat hrot{};
    if (!get_pose(API::VR::get_hmd_index(), &hpos, &hrot, /*use_aim=*/false)) {
        s_have_prev = false;
        return;
    }

    // Reload runs off the same two poses the melee detector just fetched, so it costs no extra
    // reads. It is driven from here rather than from its own tick entry for exactly that reason.
    reload_update(pos, hpos);

    // Subtracting the head removes walking, strafing and vehicle motion before any derivative is
    // taken: those move hand and head together, so they vanish from `rel` entirely.
    const Vec3  rel{pos.x - hpos.x, pos.y - hpos.y, pos.z - hpos.z};
    const float reach = std::sqrt(rel.x * rel.x + rel.y * rel.y + rel.z * rel.z);

    // ---- POSITION SANITY, checked before the derivative rather than after.
    // The velocity guard below catches the JUMP, but a session logged reach holding at 2.45-2.51 m
    // for several samples afterwards: the spike was caught and the position was still wrong. No
    // arm is that long, so treat an impossible reach as "tracking is lying" and refuse to build
    // any history on it.
    if (reach > g_cfg.melee_max_reach) {
        if (g_cfg.melee_log && s_have_prev) {
            API::get()->log_info("[Halo-CampE-UEVR] MELEE ignored impossible reach: %.2f m (cap %.2f)",
                                 reach, g_cfg.melee_max_reach);
        }
        s_have_prev = false;
        s_vel = Vec3{0.0f, 0.0f, 0.0f};
        s_ext = 0.0f;
        return;
    }

    if (!s_have_prev) {
        s_prev_rel = rel;
        s_prev_reach = reach;
        s_have_prev = true;
        return;
    }

    const Vec3 raw{(rel.x - s_prev_rel.x) / dt,
                   (rel.y - s_prev_rel.y) / dt,
                   (rel.z - s_prev_rel.z) / dt};
    const float ext_raw = (reach - s_prev_reach) / dt;   // radial: + is reaching away from the head
    s_prev_rel = rel;
    s_prev_reach = reach;

    const float a = ema_alpha(g_cfg.melee_tau_ms, dt);
    s_vel.x += (raw.x - s_vel.x) * a;
    s_vel.y += (raw.y - s_vel.y) * a;
    s_vel.z += (raw.z - s_vel.z) * a;
    s_ext   += (ext_raw - s_ext) * a;

    const float speed = std::sqrt(s_vel.x * s_vel.x + s_vel.y * s_vel.y + s_vel.z * s_vel.z);

    // ---- TRACKING DISCONTINUITY. A 26 m/s "swing" was logged in the first session; no arm does
    // that. Drop the history rather than merely refusing to fire, because the sample AFTER a
    // teleport is derived from the same bad position and would be garbage too.
    if (speed > g_cfg.melee_max_speed) {
        if (g_cfg.melee_log) {
            API::get()->log_info("[Halo-CampE-UEVR] MELEE ignored tracking spike: %.1f m/s (cap %.1f)",
                                 speed, g_cfg.melee_max_speed);
        }
        s_have_prev = false;
        s_vel = Vec3{0.0f, 0.0f, 0.0f};
        s_ext = 0.0f;
        return;
    }

    // ---- LEGACY FORWARD GATE. Off by default (melee_fwd = 0); see Config.hpp for why it is
    // unsound. Retained only so an existing config that sets it keeps working.
    float along = 1.0f;
    if (g_cfg.melee_fwd > 0.0f && speed > 1e-3f) {
        const Vec3 fwd = quat_forward(hrot);
        const float fl = std::sqrt(fwd.x * fwd.x + fwd.y * fwd.y + fwd.z * fwd.z);
        if (fl > 1e-4f) {
            const float d = (s_vel.x * fwd.x + s_vel.y * fwd.y + s_vel.z * fwd.z) / (speed * fl);
            along = std::fabs(d);
        }
    }

    // ---- SWING SEGMENTATION, for the log only. Peaks are tracked on EXTENSION rate, since that
    // is now the discriminator -- reach is sampled at that same instant so the log line describes
    // one coherent moment rather than three unrelated maxima.
    // Each peak tracked INDEPENDENTLY. Tying peak reach to the instant extension peaked produced
    // "ext=0.00 reach=0.00" lines whenever extension never went positive -- a hand at zero
    // distance from the head, which is impossible and made the log actively misleading.
    if (speed > REST_SPEED_MPS) {
        // The aim reference is taken on the FIRST tick of the swing -- where you were pointing
        // when you decided to hit something, which is the direction Halo's melee should lunge in.
        if (!s_in_swing) {
            s_swing_yaw0   = g_desired_yaw.load(std::memory_order_relaxed);
            s_swing_pitch0 = g_desired_pitch.load(std::memory_order_relaxed);
            s_swing_aim_ok = true;
            s_swing_ctrl_ok = derive_ctrl_angles(&s_swing_ctrl_yaw, &s_swing_ctrl_pitch);
        }
        s_in_swing = true;
        if (speed > s_peak_speed) s_peak_speed = speed;
        if (s_ext  > s_peak_ext)  s_peak_ext   = s_ext;
        if (reach  > s_peak_reach) s_peak_reach = reach;
    } else if (s_in_swing) {
        s_in_swing = false;
        if (g_cfg.melee_log) {
            const bool would = (s_peak_speed >= g_cfg.melee_speed) &&
                               (s_peak_ext   >= g_cfg.melee_ext)   &&
                               (s_peak_reach >= g_cfg.melee_reach);
            API::get()->log_info(
                "[Halo-CampE-UEVR] MELEE swing  speed=%.2f  ext=%.2f  reach=%.2f   "
                "(need spd>=%.2f ext>=%.2f reach>=%.2f) -- %s",
                s_peak_speed, s_peak_ext, s_peak_reach,
                g_cfg.melee_speed, g_cfg.melee_ext, g_cfg.melee_reach,
                would ? "FIRED" : "no");
        }
        s_peak_speed = 0.0f;
        s_peak_ext = 0.0f;
        s_peak_reach = 0.0f;
    }

    // ---- TRIGGER. Extension is the test; speed is a floor; reach proves the arm is actually out.
    const long long now = now_ticks();
    if (now < s_cooldown_until)    return;
    if (speed < g_cfg.melee_speed) return;
    if (s_ext < g_cfg.melee_ext)   return;
    if (reach < g_cfg.melee_reach) return;
    if (along < g_cfg.melee_fwd)   return;
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
        return;
    }

    g_melee_hold_until.store(now + ms_to_ticks(g_cfg.melee_hold_ms), std::memory_order_relaxed);
    s_cooldown_until = now + ms_to_ticks(g_cfg.melee_cooldown_ms);

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
    if (s_hold_was_armed) {
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
}

} // namespace halo
