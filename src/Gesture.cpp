#include "Gesture.hpp"

#include "Config.hpp"
#include "Math.hpp"
#include "MotionAimControl.hpp"

#include <chrono>
#include <cmath>

using uevr::API;

namespace halo {

std::atomic<long long> g_melee_hold_until{0};
std::atomic<long long> g_reload_hold_until{0};
std::atomic<unsigned short> g_pad_buttons{0};

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

// Below this relative speed the hand counts as at rest and a swing is considered over. Not
// configurable on purpose: it exists to segment the LOG, and a knob to tune the tuning instrument
// is not worth the config surface.
constexpr float REST_SPEED_MPS = 0.35f;

// ---- RELOAD STATE ------------------------------------------------------------------------------
ReloadState s_reload = ReloadState::Idle;
unsigned short s_prev_buttons = 0;

// When the CURRENT state was entered. Only meaningful outside Idle, and only the watchdog reads it.
long long s_reload_since = 0;

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
    // Restart the watchdog on EVERY transition, not only on leaving Idle: MAG_HELD -> MAG_OUT
    // (fumbling the magazine) is real progress and should buy the player the full window again.
    s_reload_since = now_ticks();
}

// Give up on a gesture that cannot be finished, and give the trigger back.
//
// Runs from reload_update() BEFORE anything that needs a pose, so a tracking dropout on the off
// hand -- one of the named ways to get stuck -- cannot also disable the escape hatch.
bool reload_watchdog_expired() {
    if (s_reload == ReloadState::Idle) return false;
    if (!(g_cfg.reload_timeout_s > 0.0f)) return false;
    const long long limit = ms_to_ticks((int)(g_cfg.reload_timeout_s * 1000.0f));
    return (now_ticks() - s_reload_since) >= limit;
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
//
// The poses are NULLABLE. Both are needed to make progress, but the watchdog below must run even
// when they are missing -- losing tracking mid-gesture is one of the ways a player gets stuck, so
// that is the last moment to stop servicing the state machine.
static void reload_update(const Vec3* hand_r, const Vec3* head) {
    if (!g_cfg.enabled || !g_cfg.reload_vr) {
        if (s_reload != ReloadState::Idle) set_state(ReloadState::Idle, "disabled");
        return;
    }

    // WATCHDOG FIRST. Logged unconditionally rather than under reload_log, because the symptom it
    // explains -- the fire trigger going dead -- is one a player will otherwise report as the mod
    // being broken, and a line they already have beats a setting they have to be told to enable.
    if (reload_watchdog_expired()) {
        API::get()->log_info(
            "[Halo-CampE-UEVR] RELOAD TIMED OUT after %.1fs in %s -- gesture never completed, "
            "returning to IDLE and releasing the fire trigger. (reloadtimeout=0 disables this.)",
            (double)g_cfg.reload_timeout_s, state_name(s_reload));
        set_state(ReloadState::Idle, "timed out");
        return;
    }

    const unsigned short btn = g_pad_buttons.load(std::memory_order_relaxed);
    const unsigned short rising = (unsigned short)(btn & ~s_prev_buttons);
    s_prev_buttons = btn;

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
    const bool have_left = get_pose(lidx, &hand_l, &lrot, /*use_aim=*/false);

    switch (s_reload) {
    case ReloadState::Idle:
        if (reload_pressed) {
            // Deliberately does NOT forward the press. The game is told to reload only when the
            // magazine goes in -- that deferral IS the feature.
            set_state(ReloadState::MagOut, "reload pressed, mag dropped");
        }
        break;

    case ReloadState::MagOut: {
        if (reload_pressed && g_cfg.reload_cancel) {
            set_state(ReloadState::Idle, "cancelled, mag re-seated");
            break;
        }
        if (!have_left || head == nullptr) break;
        // Belt zone: below the head, and near the body's vertical axis. Y is up in this space --
        // the same convention quat_forward assumes for the VR frame.
        const float drop = head->y - hand_l.y;
        const float dx = hand_l.x - head->x, dz = hand_l.z - head->z;
        const float horiz = std::sqrt(dx * dx + dz * dz);
        if (grip_held && drop >= g_cfg.reload_belt_drop && horiz <= g_cfg.reload_belt_radius) {
            set_state(ReloadState::MagHeld, "grabbed from belt");
        }
        break;
    }

    case ReloadState::MagHeld: {
        if (!grip_held) {
            set_state(ReloadState::MagOut, "grip released, dropped it");
            break;
        }
        if (!have_left || hand_r == nullptr) break;
        // Hand to hand, not hand to weapon: the gun is a separate actor whose grip point moves
        // per weapon, while both controllers are always known.
        const float ddx = hand_l.x - hand_r->x, ddy = hand_l.y - hand_r->y, ddz = hand_l.z - hand_r->z;
        const float join = std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
        if (join <= g_cfg.reload_join_dist) {
            g_reload_hold_until.store(now_ticks() + ms_to_ticks(RELOAD_HOLD_MS),
                                      std::memory_order_relaxed);
            set_state(ReloadState::Idle, "magazine seated, reload fired");
        }
        break;
    }
    }
}

// Melee-only. Split out so that turning melee off does not also tear down the reload machine --
// the two features share a tick for pose-reuse reasons, not because they are one feature.
static void melee_reset() {
    g_melee_hold_until.store(0, std::memory_order_relaxed);
    s_have_prev = false;
    s_vel = Vec3{0.0f, 0.0f, 0.0f};
    s_ext = 0.0f;
    s_in_swing = false;
    s_peak_speed = 0.0f;
    s_peak_ext = 0.0f;
    s_peak_reach = 0.0f;
}

void gesture_reset() {
    melee_reset();
    // Releasing the reload state is not optional: leaving it in MAG_OUT would keep the trigger
    // suppressed with no way for the player to notice why. Same rule as the arm hide.
    g_reload_hold_until.store(0, std::memory_order_relaxed);
    if (s_reload != ReloadState::Idle) set_state(ReloadState::Idle, "gesture reset");
    s_prev_buttons = 0;
}

void gesture_update(float dt) {
    // ---- GLOBAL STAND-DOWN. States the player did not ask to gesture in AT ALL, so both features
    // go down together and the reload machine is reset (which releases any fire suppression).
    //
    // stick mode covers vehicles, cutscenes and death; calibration means the player is holding
    // the controller still against a frozen reticle and any motion is measurement, not intent.
    //
    // melee_swing is deliberately NOT in this list any more. It used to be, which meant turning
    // melee off silently disabled VR reload and re-reset its state every tick -- two features
    // share this function only because they share two pose reads.
    if (!g_cfg.enabled ||
        g_aim_calibrating.load(std::memory_order_relaxed) ||
        g_stick_mode_active.load(std::memory_order_relaxed)) {
        gesture_reset();
        return;
    }

    // Melee follows the AIM hand: whichever hand holds the gun is the one you would hit with.
    const auto ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                          : API::VR::get_right_controller_index();

    // Grip pose, not aim pose. get_aim_pose() is documented in Plugin.cpp as producing
    // teleport-scale travel readings, which is exactly the signal this must not confuse for a
    // strike.
    //
    // BOTH poses are fetched before either feature runs, and a failure is no longer an early
    // return: reload_update() has to be called even when they are missing, because losing tracking
    // mid-gesture is one of the ways MAG_OUT used to latch and swallow the fire trigger for good.
    Vec3 pos{}; Quat rot{};
    Vec3 hpos{}; Quat hrot{};
    const bool have_hand = get_pose(ridx, &pos, &rot, /*use_aim=*/false);
    const bool have_head = get_pose(API::VR::get_hmd_index(), &hpos, &hrot, /*use_aim=*/false);
    const bool poses_ok  = have_hand && have_head;

    // Reload runs off the same two poses the melee detector uses, so it costs no extra reads. It is
    // driven from here rather than from its own tick entry for exactly that reason.
    reload_update(poses_ok ? &pos : nullptr, poses_ok ? &hpos : nullptr);

    // ---- MELEE ONLY from here down.
    if (!g_cfg.melee_swing) {
        melee_reset();
        return;
    }

    // A stalled or absurd dt turns a stationary hand into a teleport. Drop the history rather
    // than differentiate across the gap -- the next tick re-seeds cleanly.
    if (!(dt > 0.0f) || dt > 0.25f) {
        s_have_prev = false;
        return;
    }

    // HEAD-RELATIVE, and the head pose is REQUIRED -- no fail-open here. Without it there is no
    // extension measurement at all, and the previous version's fallback (assume the gate passes)
    // is precisely how it ended up firing on fast aiming.
    if (!poses_ok) {
        s_have_prev = false;
        return;
    }

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

    g_melee_hold_until.store(now + ms_to_ticks(g_cfg.melee_hold_ms), std::memory_order_relaxed);
    s_cooldown_until = now + ms_to_ticks(g_cfg.melee_cooldown_ms);

    if (g_cfg.melee_log) {
        API::get()->log_info("[Halo-CampE-UEVR] MELEE fired: speed=%.2f ext=%.2f reach=%.2f "
                             "mask=0x%04X hold=%dms",
                             speed, s_ext, reach, (unsigned)g_cfg.melee_mask, g_cfg.melee_hold_ms);
    }
}

} // namespace halo
