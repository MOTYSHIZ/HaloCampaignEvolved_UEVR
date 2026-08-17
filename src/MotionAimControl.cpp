// The aim control loop: making Halo's own aim follow your controller.
//
// WHY THIS IS A CONTROL PROBLEM AND NOT AN ASSIGNMENT
// Aim lives on Blam's ControlRotation, and Blam re-stamps it, so it cannot simply be written. It is
// steered instead by synthesizing right-stick deflection through the game's own input path -- which
// is exactly what keeps projectiles, target logic and vehicles behaving normally. That makes this a
// closed loop around a RATE actuator:
//
//   setpoint     where the controller points        (derive_ctrl_angles)
//   measurement  where the game's aim actually is   (read_control_rotation, a raw memory read)
//   actuator     the right stick, deg/s per unit    (gain-adapted, because it follows the player's
//                                                    in-game look sensitivity)
//   output       out_rx / out_ry, clamped to +-1    (aim_control_law)
//
// THE LAW ITSELF, in four parts:
//   * shape()            deadzone-compensated curve: exact zero at zero error, then straight to
//                        `floor` to clear the GAME's stick deadzone, then ramping to max_out.
//   * Schmitt deadband   leaving costs dead_deg, re-entering dead_deg * dead_hyst, so an error
//                        sitting on the threshold cannot chatter the stick.
//   * feedforward + damping   feedforward matches the target's angular rate outright (tracking
//                        without a standing error); damping opposes RATE error so it cannot cancel
//                        the feedforward.
//   * adaptive gain      measures the deg/s actually produced per unit of deflection and rescales,
//                        because a hand-calibrated constant silently depends on a game option.
//
// Reflection is never used against the PlayerController here: UEVR's reflection access-violates on
// this game's Blam objects, so aim is read by raw memory read at a validated offset instead.

// API.hpp, NOT Plugin.hpp -- Plugin.hpp defines the plugin entry points and may only be included
// by Plugin.cpp. See UeObject.hpp.
#include "uevr/API.hpp"
#include "MotionAimControl.hpp"
#include "Config.hpp"
#include "Math.hpp"
#include "UeObject.hpp"
#include "AimTrace.hpp"
#include "GameSettings.hpp"
#include "AimDirect.hpp"
#include "AimConverge.hpp"
#include "DevTools.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>

using namespace uevr;

namespace halo {

// ---- aim reference ---------------------------------------------------------------------------
std::atomic<float> g_ref_ctrl_yaw{0.0f}, g_ref_aim_yaw{0.0f};
std::atomic<float> g_ref_ctrl_pitch{0.0f}, g_ref_aim_pitch{0.0f};

// Armed by the game thread each tick when every gate (HMD, controllers, menu, references, offsets)
// passes; the hook-side law runs only while this is true. The controller index and player
// controller are published alongside so the hook never walks engine structures itself.
std::atomic<bool>    g_aim_law_armed{false};
std::atomic<int32_t> g_aim_law_ridx{-1};
std::atomic<void*>   g_aim_law_pc{nullptr};

// ---- turning ----------------------------------------------------------------------------------
// Turning state rather than aim state, but derive_ctrl_angles folds it into the setpoint, and a
// definition in Plugin.cpp could not be linked against: that file's body is an anonymous namespace.
std::atomic<float> g_turn_offset{0.0f};
std::atomic<float> g_desired_yaw{0.0f}, g_desired_pitch{0.0f};
std::atomic<bool>  g_aim_calibrating{false};
std::atomic<bool>  g_stick_mode_active{false};
std::atomic<bool>  g_menu_active{true};      // true until proven otherwise -- see MotionAimControl.hpp
std::atomic<bool>  g_frontend_active{true};  // ditto: "not yet established" must never read as live
std::atomic<void*> g_read_only_pc{nullptr};  // see MotionAimControl.hpp -- NOT the aim law's pointer
std::atomic<float> g_setpoint_rate_dps{0.0f};

// ---------------------------------------------------------------- ADAPTIVE GAIN
// The loop drives a RATE actuator whose deg/s per unit of stick is set by the GAME's controller
// sensitivity. `full_deg` was hand-calibrated against LookSensitivity30 (0.35 -> 4.3 deg/s,
// 0.90 -> ~137 deg/s), which makes the whole profile silently dependent on a game option the player
// can change, and which no other player will have set the same way.
//
// So measure it instead. Each tick we know the deflection we asked for and the yaw the game actually
// produced, giving deg/s per unit. `full_deg` is then scaled so the loop's effective gain stays put
// no matter what the sensitivity is.
//
// Deliberately conservative, because a hunting auto-tuner is worse than a mis-tuned constant:
//   * only samples when driving hard and actually turning, so noise and deadzone are excluded;
//   * adapts slowly (a few percent per sample) -- it must never move perceptibly mid-fight;
//   * clamped to +-4x the calibrated value, so a bad measurement cannot run away.
std::atomic<float> g_gain_scale{1.0f};       // multiplies full_deg
std::atomic<float> g_meas_rate{0.0f};        // deg/s per unit deflection, smoothed
std::atomic<bool>  g_gain_logged{false};

// Suppresses gain sampling briefly after any aim reference (re)capture. That is exactly when the
// aim value TELEPORTS, and across one tick a 90 degree jump is arithmetically indistinguishable
// from a 1600 deg/s turn -- which would drive the adapted gain straight to its clamp.
std::atomic<bool>     g_gain_hold{true};
std::atomic<uint32_t> g_gain_hold_until{0};

// REFERENCE_RATE_DPS is a constexpr in MotionAimControl.hpp -- Plugin.cpp reads it too.

// Fallback ratio: full-deflection rate divided by the deg/s-per-unit the adaptive gain typically
// measures. The adaptation samples wherever the loop happens to be driving, which on this title is
// mostly 0.6-0.9 -- where the measured curve gives ~115-228 per unit against a 364 full-deflection
// rate. ~2.0 is the middle of that. Used only when plant_full_dps is not set explicitly; it is a
// rough bridge from the old scalar world, not a measurement.
constexpr float PLANT_FULL_OVER_MEAN = 2.0f;

// Signed feedforward deflection for a wanted rate, via the measured curve.
static float ff_from_rate(float rate_dps, float plant_full_dps) {
    if (plant_full_dps <= 1.0f) return 0.0f;
    const float mag = std::fabs(rate_dps) / plant_full_dps;      // wanted rate as a fraction of max
    const float d   = plant_deflection_for_rate_frac(mag);       // deflection that produces it
    return (rate_dps < 0.0f) ? -d : d;
}

void update_gain_measurement(GainMeasState& st, double aim_yaw, float applied_now, float dt) {
    // A pinned gain wins outright: otherwise adaptation would drift it back and the pin would only
    // hold until the next admissible sample.
    if (g_cfg.meas_rate_fixed > 0.0f) return;
    if (!g_cfg.gain_adapt) return;

    if (!st.have_prev) { st.prev_aim = aim_yaw; st.have_prev = true; return; }
    if (!(dt > 0.0f) || dt > 0.2f) return;

    st.acc_dt     += dt;
    st.acc_out_dt += std::fabs(applied_now) * dt;   // integrate, so the mean below is time-weighted

    // Rejects aim DISCONTINUITIES, not fast turning: a level load, respawn or reference recapture
    // teleports the aim, and a 90 degree jump across one interval reads as ~1600 deg/s, which would
    // drive the gain straight to its clamp.
    constexpr float MAX_PLAUSIBLE_DPS = 400.0f;
    constexpr float EPS_DEG = 0.005f;
    // Time constants replacing the old per-sample 0.05 / 0.02 blends. Those were per CALL, so
    // moving this to the render path -- which runs several times more often -- would have silently
    // sped the adaptation up by the same factor, breaking the "slow: never moves mid-fight"
    // guarantee the gain scale depends on. These reproduce the old behaviour at tick rate and now
    // mean the same thing wherever this runs.
    constexpr float GAIN_RATE_TAU_MS  = 325.0f;
    constexpr float GAIN_SCALE_TAU_MS = 825.0f;

    const float d = wrap180((float)aim_yaw - (float)st.prev_aim);
    if (std::fabs(d) <= EPS_DEG) {
        // Aim has not moved. Do not let the accumulators grow without bound while the player
        // stands still, or the first movement afterwards would be divided by a huge interval.
        if (st.acc_dt > 0.5f) { st.acc_dt = 0.0f; st.acc_out_dt = 0.0f; }
        return;
    }

    if (st.acc_dt > 0.005f) {
        const float achieved = d / st.acc_dt;                     // deg/s over the interval
        const float applied  = st.acc_out_dt / st.acc_dt;         // MEAN deflection over that interval

        if (applied > 0.5f && std::fabs(achieved) > 15.0f && std::fabs(achieved) < MAX_PLAUSIBLE_DPS
            && !g_gain_hold.load()) {
            const float rate = std::fabs(achieved) / applied;     // deg/s per unit
            if (std::isfinite(rate) && rate > 10.0f && rate < MAX_PLAUSIBLE_DPS * 1.5f) {
                // Blend from the SEED, never straight from the raw sample. Taking the first
                // reading whole is what made the loop's opening gain a single interval's guess --
                // and feedforward divides by it, so a low first sample slams the stick. Falling
                // back to the seed here also covers the case where nothing has seeded yet.
                const float seed = (g_cfg.gain_seed > 0.0f) ? g_cfg.gain_seed : REFERENCE_RATE_DPS;
                const float cur  = (g_meas_rate.load() > 0.0f) ? g_meas_rate.load() : seed;
                const float a    = ema_alpha(GAIN_RATE_TAU_MS, st.acc_dt);
                const float smoothed = cur + a * (rate - cur);
                g_meas_rate = smoothed;

                // Faster game turn rate => reach full deflection over a WIDER error band, so the
                // loop does not overshoot. Hence scale full_deg with the measured rate.
                const float want = clampf(smoothed / REFERENCE_RATE_DPS, 0.25f, 4.0f);
                const float g    = g_gain_scale.load();
                g_gain_scale = g + ema_alpha(GAIN_SCALE_TAU_MS, st.acc_dt) * (want - g);
            }
        }
    }

    st.prev_aim = aim_yaw;
    st.acc_dt = 0.0f;
    st.acc_out_dt = 0.0f;
}

// Deadzone-compensated shaping. Exact zero at zero error (never creep); anything past the deadband
// jumps straight to `floor` so it actually crosses the game's deadzone, then rises to max_out.
// Same shape as Halo-MCC-VR's ToRawStick (floor 9000/32767 = 0.275 there).
float shape(float err_deg, float dt) {
    if (err_deg == 0.0f) return 0.0f;

    // ---- RATE-BASED SHAPING (aim_tau_s > 0) --------------------------------------------------
    // Ask for a closing RATE proportional to the error, then use the measured plant curve to find
    // the deflection that produces it: rate = err / tau, deflection = curve^-1(rate).
    //
    // WHY, over the linear map below:
    //   The linear map converts error straight to DEFLECTION, but deflection-to-rate is the plant's
    //   own curve, whose local gain runs ~52 to ~364 deg/s per unit. So the loop's actual closing
    //   gain -- deg/s per degree of error -- varies about 7x depending on how big the error happens
    //   to be, and the whole thing scales again with the game's sensitivity setting. Raising
    //   sensitivity from 30 to 90 therefore roughly tripled the loop gain without anything in the
    //   config changing, which is exactly when the aim started springing before it settled.
    //
    //   Going through rate removes both dependencies. tau is a time constant in SECONDS: the aim
    //   closes ~63% of the remaining error in that time, at any sensitivity, for any error size.
    //   Overshoot comes from demanding a closing rate the loop cannot stop in time, and tau is
    //   precisely the knob that bounds it.
    //
    // The curve also encodes the plant's dead region (~0.26), so `floor` is not applied here -- the
    // inverse already returns a deflection the game will act on, or zero.
    if (g_cfg.aim_tau_s > 0.0f) {
        const float plant_full = (g_cfg.plant_full_dps > 0.0f)
                               ? g_cfg.plant_full_dps
                               : (g_meas_rate.load() * PLANT_FULL_OVER_MEAN);
        if (plant_full > 1.0f) {
            // WHAT CLOSING RATE DO WE WANT FOR THIS MUCH ERROR?
            //
            // tau alone gives rate = err/tau: an EXPONENTIAL approach. Rate falls in proportion to
            // the remaining error, so the aim leaves fast and arrives asymptotically -- felt as
            // "moves over quickly, then slows before it gets there, then crawls". At 30 deg it asks
            // for 250 deg/s; by 10 deg only 83; by 3 deg just 25.
            //
            // aim_decel gives rate = sqrt(2*a*err): CONSTANT DECELERATION, the profile that brakes
            // at a fixed rate and therefore arrives in finite time instead of approaching forever.
            // At the same errors it asks 346 / 200 / 110 -- far more speed held far later, then a
            // hard stop. This is the continuous form of "different speed for small, mid and large
            // movements", with one parameter instead of three hand-placed thresholds.
            //
            // `a` cannot be raised without limit: the loop only learns its own rate a frame late,
            // so a profile that brakes harder than it can react to overshoots. That is a measurable
            // ceiling, not a matter of taste -- see the sweep-stop tuning.
            //
            // aim_deadbeat gives rate = err/dt: ask for the rate that closes ALL of the remaining
            // error in ONE tick. tau and decel both PACE the approach; deadbeat simply arrives.
            //
            // Why this is the right setpoint here, and not merely a faster tau:
            //   The plant was measured memoryless on 2026-08-06 (1 s vs 3 s ratio 2.974 against a
            //   predicted 3.000; 0.25 s ratio 1.030, the excess being ~0.7 of one 90 Hz tick of
            //   fixed injection overhead). A memoryless plant is exactly the condition under which
            //   curve^-1 means anything, so demanding a one-tick rate is legitimate rather than
            //   wishful.
            //
            //   It also fixes the deadzone stall at its root instead of papering over it. tau=0.12
            //   demands err/0.12; at 1.7 deg that is 14 deg/s, whose deflection sits at or under
            //   the 0.2652 hardware deadzone -- the documented "arrives quickly, then crawls the
            //   last bit". dt=1/90 demands err/0.0111, about 10.8x more, so the same 14 deg/s wall
            //   is not reached until roughly 0.16 deg of error. The `floor` below then almost never
            //   fires, which is the point: floor was a workaround for asking too little.
            //
            //   Saturation is fine and expected. One tick at the measured full rate is about 4 deg,
            //   so anything larger simply pins the stick and behaves as bang-bang -- correct for
            //   big errors, exact for small ones.
            //
            // The value IS the safety factor (0.85 = close 85% of the error per tick). Deadbeat has
            // no damping of its own, so a curve that overestimates gain would overshoot and chatter;
            // asking for slightly less than everything keeps margin and is still ~9x faster than
            // tau. 1.0 is true deadbeat.
            //
            // NOTE what this gives up: tau was doing TWO jobs -- pacing the approach AND low-pass
            // filtering hand tremor. Deadbeat removes both. If the aim reads jittery, the fix is to
            // smooth the TARGET, not to slow the approach back down.
            const float aerr = std::fabs(err_deg);
            float want_dps;
            if (g_cfg.aim_deadbeat > 0.0f) {
                // A hitch makes dt large, which asks for LESS -- safe. Only a zero/absurd dt needs
                // guarding, or the demand goes infinite.
                const float safe_dt = (dt > 0.002f && dt < 0.2f) ? dt : (1.0f / 90.0f);
                want_dps = (aerr / safe_dt) * g_cfg.aim_deadbeat;
            } else if (g_cfg.aim_decel > 0.0f) {
                want_dps = std::sqrt(2.0f * g_cfg.aim_decel * aerr);
            } else {
                want_dps = aerr / g_cfg.aim_tau_s;
            }
            float mag = plant_deflection_for_rate_frac(want_dps / plant_full);

            // FLOOR, and the comment that used to sit here was wrong.
            //
            // It claimed the curve encodes the dead region so `floor` was unnecessary. But the
            // curve's low end RETURNS deflections inside that dead region -- below about 1.7 deg of
            // error the demanded rate is under ~14 deg/s, whose deflection is 0.26-0.28, at or
            // under the hardware deadzone (XINPUT right-thumb = 0.2652). The game then ignores the
            // stick entirely and the aim STALLS just short of the target, creeping in on whatever
            // fraction squeaks past. Felt exactly as "arrives quickly, then crawls the last bit".
            //
            // Outside the deadband the loop has already decided it wants to move, so the smallest
            // thing it may ask for is a deflection the game will actually act on. This is what the
            // legacy map meant by "jumps straight to floor", and dropping it was a regression.
            if (mag > 0.0f && mag < g_cfg.floor) mag = g_cfg.floor;

            if (mag > g_cfg.max_out) mag = g_cfg.max_out;
            return err_deg < 0.0f ? -mag : mag;
        }
        // No plant estimate yet (gain not measured, nothing pinned): fall through to the linear
        // map rather than driving on a divide-by-nothing.
    }

    // ---- LEGACY LINEAR MAP -------------------------------------------------------------------
    // Error maps straight onto deflection between floor and max_out, saturating at `full` degrees.
    // Kept so the two can be compared directly, and as the fallback above.
    const float full = g_cfg.full_deg * (g_cfg.gain_adapt ? g_gain_scale.load() : 1.0f);
    float v = std::fabs(err_deg) / (full > 0.1f ? full : 0.1f);
    if (v > 1.0f) v = 1.0f;
    const float mag = g_cfg.floor + v * (g_cfg.max_out - g_cfg.floor);
    return err_deg < 0.0f ? -mag : mag;
}







// ---------------------------------------------------------------- game state
// Raw memory read, NOT reflection. UEVR's reflection access-violates on this game's Blam objects
// (UObjectHook crashes merely displaying their names), while raw reads at schema offsets are the
// proven-working instrument. So we never call a UFunction or property accessor on the controller.

// (get_pose is declared in MotionAimControl.hpp; the forward declaration it needed when this code
// lived in Plugin.cpp is no longer required.)

// Controller pose -> aim angles (game-space degrees), the same derivation the tick uses: UEVR's
// calibrated aim pose, the sightline through xdist (so hand TRANSLATION moves aim, not just
// rotation), and the snap-turn offset. Everything it touches is either a UEVR API read (internally
// locked) or an atomic, so it is callable from the XInput hook as well as the tick.
bool derive_ctrl_angles(float* out_yaw, float* out_pitch, int32_t ridx_override) {
    const int32_t ridx = (ridx_override >= 0) ? ridx_override : g_aim_law_ridx.load();
    if (ridx < 0) return false;

    Vec3 cpos{}; Quat cq{};
    if (!get_pose(ridx, &cpos, &cq, /*use_aim=*/true)) return false;

    Vec3 fwd = quat_forward(cq);

    // ---- ROLL-INVARIANT SOURCE (aimsrc=1) -- STILL UNPROVEN, DO NOT SHIP ON -------------------
    //
    // WHY ROLL MOVES AIM AT ALL. The direction above comes from the OpenXR AIM pose, which is
    // rigidly attached to the controller with its axis tilted well off the handle. You roll about
    // your WRIST, i.e. about the handle, so the aim vector sweeps a CONE about that axis: for a
    // tilt a and a roll t, forward moves by 2*asin(sin a * sin(t/2)). At a ~35 deg a 90 deg roll
    // displaces it by ~48 deg. Nothing is broken; the axis is simply the wrong one to roll about.
    //
    // It is then AMPLIFIED ASYMMETRICALLY by the yaw extraction: yaw is atan2(fwd.x, -fwd.z), so
    // as the cone carries the vector toward steeper pitch the horizontal projection shrinks and the
    // same displacement becomes a much larger yaw.
    //
    // THE PROPOSED FIX. Take the direction from the GRIP pose. If the grip's forward IS the handle
    // axis then rolling about it leaves the vector invariant, and the constant "handle points here,
    // I feel I am pointing there" difference is exactly what the Page Down calibration absorbs.
    //
    // ⚠️ WHAT 27,826 LOGGED SAMPLES ACTUALLY ESTABLISHED (2026-08-09), because the obvious reading
    // of them is wrong:
    //   * SOLID -- the sightline term is negligible: |mean| 0.18 deg, past 1 deg in 2.4% of
    //     samples, no trend against roll. Widening xdist cannot help; that hypothesis is dead.
    //   * SOLID -- grip forward is STEEPLY PITCHED: mean 41.5 deg, past 60 deg in HALF of samples,
    //     against the aim pose's 7.7 deg / 0.3%. So grip YAW is an atan2 over a small horizontal
    //     projection, whatever the vector does.
    //   * NOT ESTABLISHED -- anything measured against `roll` in that run. It came from a
    //     Tait-Bryan decomposition whose yaw/roll split degenerates as 1/cos(pitch), and half the
    //     samples were past 60 deg. The headline "grip yaw swings 1.08 deg/deg of roll" is the
    //     instrument failing near gimbal lock, not the vector moving. The aim pose's 0.19 deg/deg
    //     is suspect for the same reason.
    // The AIMROLL instrument now takes roll as the swing-twist about grip forward, which is
    // well-conditioned at any pitch. Re-measure before adopting or deleting this option.
    if (g_cfg.aim_src == 1) {
        Vec3 gpos{}; Quat gq{};
        if (get_pose(ridx, &gpos, &gq, /*use_aim=*/false)) {
            fwd = quat_forward(gq);
            // Position still comes from the aim pose: the sightline mixes this with cpos, and the
            // grip POSITION is a different point. Only the DIRECTION is being replaced.
        }
    }

    Vec3 origin{};
    const bool have_origin = aim_sightline_origin(&origin);
    if (have_origin) {
        Vec3 t{
            cpos.x + fwd.x * g_cfg.xdist_m - origin.x,
            cpos.y + fwd.y * g_cfg.xdist_m - origin.y,
            cpos.z + fwd.z * g_cfg.xdist_m - origin.z
        };
        const float len = std::sqrt(t.x * t.x + t.y * t.y + t.z * t.z);
        if (len > 1e-3f) { fwd = Vec3{t.x / len, t.y / len, t.z / len}; }
    }

    *out_yaw   = wrap180(std::atan2(fwd.x, -fwd.z) * RAD2DEG + g_cfg.aim_turn * g_turn_offset.load());
    *out_pitch = std::asin(clampf(fwd.y, -1.0f, 1.0f)) * RAD2DEG;

#if HALO_VR_DEV
    // AIMROLL -- measure the roll->aim coupling instead of arguing about it.
    //
    // Hold the controller pointing at one spot and roll the wrist left, then right. Read `roll`
    // against `aimYaw`/`gripYaw`:
    //   aimYaw moves a lot with roll, gripYaw does not  -> the cone; aimsrc=1 removes it
    //   BOTH move                                       -> the coupling is upstream of the source
    //                                                      choice and aimsrc=1 will not help
    //   |dYaw per degree of roll| differs left vs right -> the atan2 amplification, and `pitch`
    //                                                      will show which side is steeper
    if (g_cfg.aim_roll_log > 0) {
        static std::atomic<uint32_t> n{0};
        if ((n.fetch_add(1, std::memory_order_relaxed) % (uint32_t)g_cfg.aim_roll_log) == 0) {
            Vec3 gpos{}; Quat gq{};
            const bool have_grip = get_pose(ridx, &gpos, &gq, /*use_aim=*/false);
            const Vec3 afwd = quat_forward(cq);
            const Vec3 gfwd = have_grip ? quat_forward(gq) : Vec3{0.0f, 0.0f, 0.0f};

            // ROLL BY SWING-TWIST, NOT BY quat_to_rotator.
            //
            // The first version took roll from a Tait-Bryan decomposition. Yaw and roll DEGENERATE
            // as pitch steepens -- the split amplifies as 1/cos(pitch), so 2x at 60 deg and ~6x at
            // 80 -- and the measured grip pitch runs past 60 deg in HALF of all samples. The first
            // 27k-sample run produced "grip yaw swings 1.08 deg per degree of roll", which is the
            // decomposition coming apart near gimbal lock rather than the vector moving. Every
            // conclusion drawn against that axis was measuring the instrument.
            //
            // The twist of q about an axis a is well-conditioned at any pitch:
            //   proj  = (q.xyz . a) a ,  twist = normalize(quat(proj, q.w))
            // Taken about the GRIP FORWARD, which is the axis the wrist actually rolls about, so
            // this is the physical quantity the whole question is about.
            float gp = 0.0f, gy = 0.0f, gr = 0.0f;
            if (have_grip) {
                float discard_roll = 0.0f;   // quat_to_rotator dereferences all three unconditionally
                quat_to_rotator(-gq.z, gq.x, gq.y, -gq.w, &gp, &gy, &discard_roll);
                const float d = gq.x * gfwd.x + gq.y * gfwd.y + gq.z * gfwd.z;
                const float px = gfwd.x * d, py = gfwd.y * d, pz = gfwd.z * d;
                const float n  = std::sqrt(px * px + py * py + pz * pz);
                gr = 2.0f * std::atan2(n, std::fabs(gq.w)) * RAD2DEG;
                if (d < 0.0f) gr = -gr;
            }
            API::get()->log_info(
                "[Halo-CampE-UEVR] AIMROLL src=%d roll=%.1f | aim=(y%.2f,p%.2f) grip=(y%.2f,p%.2f) "
                "| out=(y%.2f,p%.2f) sightline_dyaw=%.2f haveGrip=%d",
                g_cfg.aim_src, gr,
                std::atan2(afwd.x, -afwd.z) * RAD2DEG,
                std::asin(clampf(afwd.y, -1.0f, 1.0f)) * RAD2DEG,
                have_grip ? std::atan2(gfwd.x, -gfwd.z) * RAD2DEG : 0.0f,
                have_grip ? std::asin(clampf(gfwd.y, -1.0f, 1.0f)) * RAD2DEG : 0.0f,
                *out_yaw, *out_pitch,
                wrap180(std::atan2(fwd.x, -fwd.z) * RAD2DEG
                        - std::atan2(afwd.x, -afwd.z) * RAD2DEG),
                (int)have_grip);
        }
    }
#endif
    return true;
}

// The aim setpoint, sampled now. See the header for why a consumer would want this instead of
// g_desired_yaw. The body is deliberately identical to the setpoint arithmetic in
// aim_control_law() below -- the hand's rotation since calibration, added to the aim captured at
// calibration -- and BlamAim's controller_desired_aim() forwards here so the sim driver and every
// consumer of the setpoint share one definition rather than three copies that can drift apart.
bool desired_aim_now(float* out_yaw, float* out_pitch) {
    const int32_t ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                             : API::VR::get_right_controller_index();
    float cy = 0.0f, cp = 0.0f;
    if (!derive_ctrl_angles(&cy, &cp, ridx)) return false;
    *out_yaw   = g_ref_aim_yaw.load()   + wrap180(cy - g_ref_ctrl_yaw.load());
    *out_pitch = g_ref_aim_pitch.load() + wrap180(cp - g_ref_ctrl_pitch.load());
    return true;
}

// ControlRotation raw read for the hook: the same guarded read as read_control_rotation, but
// against the PLAYER CONTROLLER PUBLISHED BY THE TICK rather than a fresh engine walk -- the hook
// must not touch engine containers from its thread.
bool read_control_rotation_hook(double* out_pitch, double* out_yaw) {
    // Prefer the law's pointer when it is armed; fall back to the read-only one so a caller that
    // merely wants to LOOK at ControlRotation still can while the motion stack is stood down. Both
    // are validated below, so the fallback widens availability without widening trust.
    auto* pc = reinterpret_cast<const uint8_t*>(g_aim_law_pc.load());
    if (pc == nullptr) pc = reinterpret_cast<const uint8_t*>(g_read_only_pc.load());
    if (pc == nullptr) return false;
    const double* rot = reinterpret_cast<const double*>(
        pc + g_control_rotation_offset.load(std::memory_order_relaxed));
    if (IsBadReadPtr(rot, sizeof(double) * 3)) return false;
    const double pv = rot[0], yv = rot[1];
    if (!std::isfinite(pv) || !std::isfinite(yv)) return false;
    if (pv < -400.0 || pv > 400.0 || yv < -400.0 || yv > 400.0) return false;
    *out_pitch = pv; *out_yaw = yv;
    return true;
}


#if HALO_VR_DEV
static unsigned g_direct_log_tick = 0;   // dev-only: paces the DIRECT diagnostic line
#endif

void aim_control_law(AimLawState& st, float ctrl_yaw, float ctrl_pitch,
                     double aim_yaw, double aim_pitch, float dt,
                     float* out_rx, float* out_ry) {
    // The hand's rotation SINCE CALIBRATION, kept as its own term because the direct-write path
    // needs to be able to mirror it independently of the reference it is added to.
    const float dctrl_yaw   = wrap180(ctrl_yaw   - g_ref_ctrl_yaw.load());
    const float dctrl_pitch = wrap180(ctrl_pitch - g_ref_ctrl_pitch.load());

    const float desired_yaw   = g_ref_aim_yaw.load()   + dctrl_yaw;
    const float desired_pitch = g_ref_aim_pitch.load() + dctrl_pitch;

    // Published for the reticule (see the header). Set before the deadband and shaping so it is the
    // raw setpoint, not something the actuator has already filtered.
    //
    // AND BEFORE THE CONVERGENCE CORRECTION, deliberately. There are two distinct quantities from
    // here on and conflating them is the trap:
    //   INTENT  (g_desired_*) -- where the player is pointing. What the reticule draws along and
    //                            what the movement frame rotates by; both want a direction that
    //                            does not twitch when the depth under the crosshair changes.
    //   COMMAND (cmd_*)       -- what is written into the game's aim, bent so the shot LANDS on the
    //                            point the intent ray hits. Depth-dependent by definition.
    // Every drive site takes the command; nothing else does. See AimConverge.hpp.
    g_desired_yaw   = desired_yaw;
    g_desired_pitch = desired_pitch;

    float cmd_yaw = desired_yaw, cmd_pitch = desired_pitch;
    aim_converge_apply(&cmd_yaw, &cmd_pitch);

    float err_yaw   = wrap180(cmd_yaw   - (float)aim_yaw);
    float err_pitch = wrap180(cmd_pitch - (float)aim_pitch);

    // Schmitt-gated deadband: leaving the band takes dead_deg, re-entering takes dead_deg *
    // dead_hyst, so an error hovering at the edge cannot chatter the stick between 0 and `floor`.
    {
        const float on  = g_cfg.dead_deg;
        const float off = g_cfg.dead_deg * g_cfg.dead_hyst;
        if (st.parked_yaw)   { if (std::fabs(err_yaw)   > on)  st.parked_yaw = false; }
        else                 { if (std::fabs(err_yaw)   < off) st.parked_yaw = true; }
        if (st.parked_pitch) { if (std::fabs(err_pitch) > on)  st.parked_pitch = false; }
        else                 { if (std::fabs(err_pitch) < off) st.parked_pitch = true; }
        if (st.parked_yaw)   err_yaw = 0.0f;
        if (st.parked_pitch) err_pitch = 0.0f;
    }

    // Velocity feedforward + rate damping. Feedforward supplies the deflection that matches the
    // target's angular rate outright (tracking without a standing error); damping opposes RATE
    // error so it cannot cancel the feedforward. Both scale by the MEASURED plant gain.
    float ff_yaw = 0.0f, ff_pitch = 0.0f, damp_yaw = 0.0f, damp_pitch = 0.0f;
    {
        const float mr = g_meas_rate.load();
        if (!st.have_prev) {
            st.prev_des_yaw = desired_yaw; st.prev_des_pitch = desired_pitch;
            st.prev_aim_y = aim_yaw;       st.prev_aim_p = aim_pitch;
            st.have_prev = true;
        } else if (dt > 0.0f && dt < 0.2f && mr > 10.0f) {
            constexpr float MAX_PLAUSIBLE_DPS = 400.0f;   // rejects teleports (loads, recaptures)
            constexpr float EPS_DEG = 0.005f;             // below this the angle has not changed
            // Time constant for the stale-rate decay below. 105 ms reproduces the old 0.9-per-call
            // factor at 90 fps, and now means the same thing at every frame rate.
            constexpr float RATE_DECAY_TAU_MS = 105.0f;

            st.acc_dt_des += dt;
            st.acc_dt_aim += dt;

            // Target (pose-derived) rate: update on change, over the accumulated span.
            const float d_des_yaw   = wrap180(desired_yaw   - st.prev_des_yaw);
            const float d_des_pitch = wrap180(desired_pitch - st.prev_des_pitch);
            if ((std::fabs(d_des_yaw) > EPS_DEG || std::fabs(d_des_pitch) > EPS_DEG) &&
                st.acc_dt_des > 0.0005f) {
                const float tr_yaw   = d_des_yaw   / st.acc_dt_des;
                const float tr_pitch = d_des_pitch / st.acc_dt_des;
                if (std::fabs(tr_yaw) < MAX_PLAUSIBLE_DPS && std::fabs(tr_pitch) < MAX_PLAUSIBLE_DPS) {
                    // dt is the ACCUMULATED span, not the frame dt: this filter only steps on a
                    // change boundary, so that span is how long it has actually been standing still.
                    const float a = ema_alpha(g_cfg.ff_smooth_ms, st.acc_dt_des);
                    st.ff_rate_yaw   += a * (tr_yaw   - st.ff_rate_yaw);
                    st.ff_rate_pitch += a * (tr_pitch - st.ff_rate_pitch);
                    // Published for the reticule's motion-gated smoothing (see the header).
                    g_setpoint_rate_dps = std::sqrt(st.ff_rate_yaw * st.ff_rate_yaw +
                                                    st.ff_rate_pitch * st.ff_rate_pitch);
                }
                st.prev_des_yaw = desired_yaw; st.prev_des_pitch = desired_pitch;
                st.acc_dt_des = 0.0f;
            }

            // Achieved (Blam aim) rate: same change-boundary measurement.
            const float d_aim_yaw   = wrap180((float)aim_yaw   - (float)st.prev_aim_y);
            const float d_aim_pitch = wrap180((float)aim_pitch - (float)st.prev_aim_p);
            if ((std::fabs(d_aim_yaw) > EPS_DEG || std::fabs(d_aim_pitch) > EPS_DEG) &&
                st.acc_dt_aim > 0.0005f) {
                const float ar_yaw   = d_aim_yaw   / st.acc_dt_aim;
                const float ar_pitch = d_aim_pitch / st.acc_dt_aim;
                if (std::fabs(ar_yaw) < MAX_PLAUSIBLE_DPS && std::fabs(ar_pitch) < MAX_PLAUSIBLE_DPS) {
                    // Filtered harder than the feedforward input: damping differentiates readout
                    // noise, feedforward must stay fast.
                    const float b = ema_alpha(g_cfg.d_smooth_ms, st.acc_dt_aim);
                    st.aim_rate_yaw   += b * (ar_yaw   - st.aim_rate_yaw);
                    st.aim_rate_pitch += b * (ar_pitch - st.aim_rate_pitch);
                }
                st.prev_aim_y = aim_yaw; st.prev_aim_p = aim_pitch;
                st.acc_dt_aim = 0.0f;
            }

            // Both rates decay toward zero if their source stops changing for a while, so a
            // stationary target cannot keep stale feedforward alive. Per unit TIME, not per call:
            // as a per-call factor this bled away three times faster at 90 fps than at 30, so how
            // long a stale rate survived depended on the frame rate.
            const float keep = 1.0f - ema_alpha(RATE_DECAY_TAU_MS, dt);
            if (st.acc_dt_des > 0.15f) { st.ff_rate_yaw *= keep; st.ff_rate_pitch *= keep; }
            if (st.acc_dt_aim > 0.15f) { st.aim_rate_yaw *= keep; st.aim_rate_pitch *= keep; }

            if (g_cfg.ff_gain > 0.0f) {
                float yaw_ff_scale = 1.0f;
                if (g_cfg.ff_pitch_comp) {
                    // cos(pitch) projection guard, floored so steep aim keeps yaw authority.
                    yaw_ff_scale = std::fabs(std::cos((float)aim_pitch * DEG2RAD));
                    if (yaw_ff_scale < 0.35f) yaw_ff_scale = 0.35f;
                }
                // INVERT THE MEASURED CURVE rather than divide by a single gain.
                //
                // rate / mr treats the plant as linear. It is not: the local gain runs ~52 to ~364
                // deg/s per unit across the stick, so one scalar over-drives at low deflection and
                // under-drives at high, and no value of it is right in more than one place. Asking
                // the measured curve "what deflection gives this rate?" is the question feedforward
                // was always trying to answer.
                //
                // plant_full is the full-deflection rate for the CURRENT sensitivity. The curve is
                // stored as fractions of that, so this one table serves any sensitivity -- provided
                // sensitivity is a pure multiplier on rate, which is measured at one setting and
                // still unproven (docs\PlantCurve.md).
                const float plant_full = (g_cfg.plant_full_dps > 0.0f)
                                       ? g_cfg.plant_full_dps
                                       : (mr * PLANT_FULL_OVER_MEAN);
                ff_yaw   = ff_from_rate(st.ff_rate_yaw,   plant_full) * g_cfg.ff_gain * yaw_ff_scale;
                ff_pitch = ff_from_rate(st.ff_rate_pitch, plant_full) * g_cfg.ff_gain;
            }
            if (g_cfg.d_gain > 0.0f) {
                // Damping stays on the LINEAR scalar deliberately. It opposes a rate ERROR, which is
                // a small correction around the operating point rather than a deflection that has to
                // produce a specific absolute rate -- so the local slope is the right model and the
                // curve inverse would be the wrong tool.
                damp_yaw   = -clampf((st.aim_rate_yaw   - st.ff_rate_yaw)   / mr, -1.0f, 1.0f) * g_cfg.d_gain;
                damp_pitch = -clampf((st.aim_rate_pitch - st.ff_rate_pitch) / mr, -1.0f, 1.0f) * g_cfg.d_gain;
            }
        }
    }

    // SIGN: +stick.x increases yaw on this game.
    //
    // g_invert_cancel_* CANCELS the game's own look inversion. Motion aim must never be inverted --
    // your hand points where it points, and there is no sense in which "up is down" when the gun IS
    // the input. Rather than change the game's setting (which would have to be written on every
    // mode transition, and could be left flipped by a crash), we simply negate the value we are
    // about to synthesise, so the game's inversion turns it back the right way round.
    //
    // These are 1.0 in stick mode, so a player who inverts deliberately for vehicles keeps exactly
    // the feel they configured -- their own stick passes through untouched.
    float raw_yaw   = shape(err_yaw,   dt) + ff_yaw   + damp_yaw;
    float raw_pitch = shape(err_pitch, dt) + ff_pitch + damp_pitch;

    // FLOOR THE SUM, not just shape()'s contribution.
    //
    // The game ignores any deflection under the hardware deadzone (XInput right thumb, 0.2652), so
    // there is no such thing as a small correction: the plant does nothing, or it does at least
    // ~14 deg/s. shape() already jumps to `floor` for that reason -- but DAMPING then subtracts
    // from it. Near a target the setpoint is still while the aim is moving, so damping opposes that
    // motion and drags the sum under the threshold.
    //
    // The result is not gentle braking, it is a STUTTER: output drops below the deadzone, the aim
    // stops dead, damping decays to nothing because there is no longer any rate to oppose, the sum
    // climbs back over `floor`, the aim jerks, and damping bites again. Measured at 0.243 mean
    // deflection with 100% of samples under the deadzone across the last 2 degrees -- which is
    // exactly the "slows down, then crawls the last bit" that prompted this.
    //
    // Outside the deadband the honest choice is between `floor` and zero, because the plant offers
    // nothing in between. Asking for floor, in the direction of the error, is the one that closes
    // it. The deadband still parks the loop below dead_deg, and floor moves only ~0.15 deg per
    // frame, so this cannot run away.
    if (!st.parked_yaw && err_yaw != 0.0f && std::fabs(raw_yaw) < g_cfg.floor) {
        raw_yaw = (err_yaw > 0.0f) ? g_cfg.floor : -g_cfg.floor;
    }
    if (g_cfg.drive_pitch && !st.parked_pitch && err_pitch != 0.0f
        && std::fabs(raw_pitch) < g_cfg.floor) {
        raw_pitch = (err_pitch > 0.0f) ? g_cfg.floor : -g_cfg.floor;
    }

    // DIRECT ASSIGNMENT, when it is enabled and the rotator has been located.
    //
    // Everything above -- shaping, floor, deadband, feedforward, damping -- exists to steer a rate
    // actuator toward `desired`. If the aim can simply BE `desired`, none of it is needed, so the
    // stick is left neutral and the value is written straight in.
    //
    // Deliberately placed here, after the setpoint is computed, rather than replacing the law
    // wholesale: `desired_yaw`/`desired_pitch` already carry the aim reference, the calibration
    // offsets, and the accumulated snap-turn, and duplicating that derivation for a second code
    // path is how the two would drift apart.
    //
    // Falls through to the stick output if the rotator is not resolved (still locating, or a level
    // load invalidated it), so a failed locate degrades to the loop instead of to no aim at all.
    // The sign knobs exist because the steered loop and a direct write are NOT interchangeable in
    // the way the maths says they should be. The loop converges aim -> desired, so assigning
    // `desired` ought to land in exactly the same place -- yet the first live test came back
    // "responsive, but inverted on both axes". Rather than guess at which frame is flipped and burn
    // a build per guess (launches are unreliable), the mapping is a live tunable: +1 reproduces the
    // steered setpoint exactly, -1 mirrors the hand's motion about the calibration reference.
    if (g_cfg.aim_direct && aim_direct_ready()) {
        float wy = g_ref_aim_yaw.load() + g_cfg.aim_direct_sign_x * dctrl_yaw;
        float wp = g_ref_aim_pitch.load() + g_cfg.aim_direct_sign_y * dctrl_pitch;
        // Corrected HERE TOO, and with the same call, because this is the LOCAL VIEW half of the
        // aim pair -- blamangles writes the simulation, this writes what you see. The two halves
        // disagreeing by even a degree is the documented "heavy jitter" failure, so a correction
        // applied to one and not the other would be worse than no correction at all.
        aim_converge_apply(&wy, &wp);

        const double want_yaw   = (double)wy;
        const double want_pitch = g_cfg.drive_pitch ? (double)wp : aim_pitch;
        if (aim_direct_set(want_pitch, want_yaw)) {
            *out_rx = 0.0f;
            *out_ry = 0.0f;
            // Traced from INSIDE the direct path. The old code returned above the sample call, so a
            // direct-mode trace recorded nothing at all and every measurement run came back empty.
            // `des` is what was actually written, so the trace measures the real setpoint.
            aim_trace_sample(dt, (float)want_yaw, (float)want_pitch, aim_yaw, aim_pitch,
                             0.0f, 0.0f, st.ff_rate_yaw);
#if HALO_VR_DEV
            if ((g_direct_log_tick++ % 600) == 0) {
                API::get()->log_info("[Halo-CampE-UEVR] DIRECT: hand d=(%.2f,%.2f) want=(%.2f,%.2f) "
                                     "aim=(%.2f,%.2f) steered_desired=(%.2f,%.2f) sign=(%.0f,%.0f)",
                                     dctrl_yaw, dctrl_pitch, want_yaw, want_pitch,
                                     aim_yaw, aim_pitch, desired_yaw, desired_pitch,
                                     g_cfg.aim_direct_sign_x, g_cfg.aim_direct_sign_y);
            }
#endif
            return;
        }
    }

    *out_rx = clampf(raw_yaw * g_cfg.yaw_sign * g_invert_cancel_x, -1.0f, 1.0f);
    *out_ry = g_cfg.drive_pitch
        ? clampf(raw_pitch * g_cfg.pitch_sign * g_invert_cancel_y, -1.0f, 1.0f)
        : 0.0f;

    // Sampled HERE, at the bottom of the law, because this is the only point where the setpoint,
    // the achieved aim and the emitted stick are all final and belong to the same instant.
    // Sampling anywhere else would correlate values from different iterations, which is exactly
    // the kind of error that makes a measured result worse than no measurement at all.
    // No-op in a release build (see AimTrace.hpp).
    aim_trace_sample(dt, desired_yaw, desired_pitch, aim_yaw, aim_pitch,
                     *out_rx, *out_ry, st.ff_rate_yaw);
}

std::atomic<size_t> g_control_rotation_offset{CONTROL_ROTATION_OFFSET_EXPECTED};

void resolve_control_rotation_offset(void* pc_raw) {
    static std::atomic<bool> s_done{false};
    if (pc_raw == nullptr || s_done.load(std::memory_order_relaxed)) return;

    auto* pc = reinterpret_cast<API::UObject*>(pc_raw);
    auto* p  = pc->get_property_data<double>(L"ControlRotation");
    if (p == nullptr) {
        s_done.store(true, std::memory_order_relaxed);
        API::get()->log_info("[Halo-CampE-UEVR] CTLROT: reflection could not find ControlRotation "
                             "-- keeping the compiled offset +0x%llX",
                             (unsigned long long)CONTROL_ROTATION_OFFSET_EXPECTED);
        return;
    }

    // VALIDATE BEFORE ADOPTING. A resolved-but-wrong offset is worse than the compiled one, because
    // it looks authoritative. An FRotator of doubles is 8-byte aligned, and a PlayerController is a
    // few KB -- anything outside that is not the field we asked for.
    const uintptr_t off = (uintptr_t)p - (uintptr_t)pc;
    if (off < 0x40 || off > 0x4000 || (off & 7) != 0) {
        s_done.store(true, std::memory_order_relaxed);
        API::get()->log_info("[Halo-CampE-UEVR] CTLROT: reflection returned an implausible offset "
                             "+0x%llX -- keeping the compiled +0x%llX",
                             (unsigned long long)off,
                             (unsigned long long)CONTROL_ROTATION_OFFSET_EXPECTED);
        return;
    }

    g_control_rotation_offset.store((size_t)off, std::memory_order_relaxed);
    s_done.store(true, std::memory_order_relaxed);

    if (off == CONTROL_ROTATION_OFFSET_EXPECTED) {
        API::get()->log_info("[Halo-CampE-UEVR] CTLROT: ControlRotation at +0x%llX (reflection; "
                             "matches the compiled expectation)", (unsigned long long)off);
    } else {
        API::get()->log_info("[Halo-CampE-UEVR] CTLROT: ControlRotation at +0x%llX -- the compiled "
                             "expectation +0x%llX is WRONG for this build. Reading there would have "
                             "failed the sanity gate and taken snap turn, aim and movement with it; "
                             "using the resolved value instead.",
                             (unsigned long long)off,
                             (unsigned long long)CONTROL_ROTATION_OFFSET_EXPECTED);
    }
}

bool read_control_rotation(double* out_pitch, double* out_yaw, void** out_pc) {
    auto* pc = API::get()->get_player_controller(0);
    if (pc == nullptr) return false;
    if (out_pc != nullptr) *out_pc = (void*)pc;

    // Resolved HERE because this is the game-thread reader: it already holds a live
    // PlayerController, it runs before any consumer needs the offset, and the hook-side reader is
    // deliberately never given a path that touches reflection.
    resolve_control_rotation_offset(pc);

    auto* base = reinterpret_cast<const uint8_t*>(pc);
    const double* rot = reinterpret_cast<const double*>(
        base + g_control_rotation_offset.load(std::memory_order_relaxed));

    if (IsBadReadPtr(rot, sizeof(double) * 3)) return false;

    const double p = rot[0];
    const double y = rot[1];
    // Sanity gate: a real FRotator here is finite and in a plausible range. If the offset ever
    // shifts (game patch), this fails closed rather than steering aim from garbage.
    if (!std::isfinite(p) || !std::isfinite(y)) return false;
    if (p < -400.0 || p > 400.0 || y < -400.0 || y > 400.0) return false;

    *out_pitch = p;
    *out_yaw   = y;
    return true;
}

// use_aim: UEVR's CALIBRATED POINTING pose rather than the raw device pose. This is the same
// source Halo-MCC-VR consumes (VR_GetAimPose) and it is what "use UEVR's setup" actually buys us
// here: per-runtime controller conventions applied by UEVR instead of hand-rolled.
//
// NOTE ON WHY WE DO NOT CONSUME UEVR'S *AIM METHOD*: aim-method-2 applies its result by writing
// APlayerController::ControlRotation (manual_update_control_rotation) or via ProcessViewRotation.
// Both target UE's aim pipeline. On this title ControlRotation is an OUTPUT MIRROR of the Blam
// camera -- a raw write reverts within one frame, bUseControllerRotationYaw/Pitch/Roll are all
// false, and nothing reads it. So aim-method-2 is inert here for architectural reasons, not
// configuration ones, and its output could only be recovered by racing Blam for the value it is
// about to overwrite. We read the same upstream source it does -- the controller pose -- directly.
// THE SIGHTLINE'S BODY REFERENCE. One definition, because there are two sightline sites (here and
// the closed loop in Plugin.cpp) and they must not be able to disagree about where the player is.
//
// aimorigin=1 means "the body", and the STANDING ORIGIN is the body only while something holds it
// to your head. A leash does exactly that -- at the default radius 0 it re-pins the origin to the
// HMD every tick, so modes 0 and 1 are numerically the same thing and the setting is inert.
//
// With hmdleash=0 nothing holds it. The origin stays where you last recentred while you stand up,
// lean or roll your chair away, and the sightline then swings by atan(drift / xdist) with no input
// from you: 1 m of drift at xdist=1000 is 5.7 deg, growing to the size of your room. That is not a
// tuning problem, it is a stale reference, so an unleashed head demotes mode 1 to mode 0 -- which
// is drift-free by construction, because the head term cancels out of (cpos + fwd*x - hmd).
//
// The cost of the demotion is mode 0's documented drawback: your head orbits ~10 cm about your neck
// as you look around, worth ~0.6 deg at xdist=1000. Small, constant, and it does not accumulate.
bool aim_sightline_origin(Vec3* out) {
    if (g_cfg.aim_origin == 1 && g_cfg.hmd_leash) {
        const auto so = API::VR::get_standing_origin();
        *out = Vec3{so.x, so.y, so.z};
        return true;
    }

    static bool said_demote = false;
    if (g_cfg.aim_origin == 1 && !said_demote) {
        said_demote = true;
        API::get()->log_info("[Halo-CampE-UEVR] SIGHTLINE: hmdleash=0, so aimorigin=1 (standing "
                             "origin) is using the HEAD instead -- an unleashed standing origin is "
                             "a stale body reference and would swing aim as you move.");
    }

    Vec3 hpos{}; Quat hq{};
    const auto hidx = API::VR::get_hmd_index();
    if (hidx >= 0 && get_pose(hidx, &hpos, &hq, /*use_aim=*/false)) { *out = hpos; return true; }
    return false;
}

bool get_pose(UEVR_TrackedDeviceIndex idx, Vec3* pos, Quat* rot, bool use_aim) {
    const auto pose = use_aim ? API::VR::get_aim_pose(idx) : API::VR::get_pose(idx);
    const auto& q = pose.rotation;
    const float m2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    // UEVR withholds real poses until it has observed controller INPUT, returning a non-unit
    // placeholder until then. Moving a controller is not enough -- press a button or trigger once.
    // The placeholder must never be treated as a real pose.
    if (m2 < 0.9f || m2 > 1.1f) return false;
    *pos = Vec3{pose.position.x, pose.position.y, pose.position.z};
    *rot = Quat{q.x, q.y, q.z, q.w};
    return true;
}


} // namespace halo
