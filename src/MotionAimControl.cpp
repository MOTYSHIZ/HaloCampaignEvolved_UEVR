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

// Deadzone-compensated shaping. Exact zero at zero error (never creep); anything past the deadband
// jumps straight to `floor` so it actually crosses the game's deadzone, then rises to max_out.
// Same shape as Halo-MCC-VR's ToRawStick (floor 9000/32767 = 0.275 there).
float shape(float err_deg) {
    if (err_deg == 0.0f) return 0.0f;
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
bool derive_ctrl_angles(float* out_yaw, float* out_pitch) {
    const int32_t ridx = g_aim_law_ridx.load();
    if (ridx < 0) return false;

    Vec3 cpos{}; Quat cq{};
    if (!get_pose(ridx, &cpos, &cq, /*use_aim=*/true)) return false;

    Vec3 fwd = quat_forward(cq);

    Vec3 origin{};
    bool have_origin = false;
    if (g_cfg.aim_origin == 1) {
        const auto so = API::VR::get_standing_origin();
        origin = Vec3{so.x, so.y, so.z};
        have_origin = true;
    } else {
        Vec3 hpos{}; Quat hq{};
        const auto hidx = API::VR::get_hmd_index();
        if (hidx >= 0 && get_pose(hidx, &hpos, &hq, /*use_aim=*/false)) { origin = hpos; have_origin = true; }
    }
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
    return true;
}

// ControlRotation raw read for the hook: the same guarded read as read_control_rotation, but
// against the PLAYER CONTROLLER PUBLISHED BY THE TICK rather than a fresh engine walk -- the hook
// must not touch engine containers from its thread.
bool read_control_rotation_hook(double* out_pitch, double* out_yaw) {
    auto* pc = reinterpret_cast<const uint8_t*>(g_aim_law_pc.load());
    if (pc == nullptr) return false;
    const double* rot = reinterpret_cast<const double*>(pc + CONTROL_ROTATION_OFFSET);
    if (IsBadReadPtr(rot, sizeof(double) * 3)) return false;
    const double pv = rot[0], yv = rot[1];
    if (!std::isfinite(pv) || !std::isfinite(yv)) return false;
    if (pv < -400.0 || pv > 400.0 || yv < -400.0 || yv > 400.0) return false;
    *out_pitch = pv; *out_yaw = yv;
    return true;
}


void aim_control_law(AimLawState& st, float ctrl_yaw, float ctrl_pitch,
                     double aim_yaw, double aim_pitch, float dt,
                     float* out_rx, float* out_ry) {
    const float desired_yaw   = g_ref_aim_yaw.load()   + wrap180(ctrl_yaw   - g_ref_ctrl_yaw.load());
    const float desired_pitch = g_ref_aim_pitch.load() + wrap180(ctrl_pitch - g_ref_ctrl_pitch.load());

    float err_yaw   = wrap180(desired_yaw   - (float)aim_yaw);
    float err_pitch = wrap180(desired_pitch - (float)aim_pitch);

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
                    const float a = g_cfg.ff_smooth;
                    st.ff_rate_yaw   += a * (tr_yaw   - st.ff_rate_yaw);
                    st.ff_rate_pitch += a * (tr_pitch - st.ff_rate_pitch);
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
                    const float b = g_cfg.d_smooth;
                    st.aim_rate_yaw   += b * (ar_yaw   - st.aim_rate_yaw);
                    st.aim_rate_pitch += b * (ar_pitch - st.aim_rate_pitch);
                }
                st.prev_aim_y = aim_yaw; st.prev_aim_p = aim_pitch;
                st.acc_dt_aim = 0.0f;
            }

            // Both rates decay toward zero if their source stops changing for a while, so a
            // stationary target cannot keep stale feedforward alive.
            if (st.acc_dt_des > 0.15f) { st.ff_rate_yaw *= 0.9f; st.ff_rate_pitch *= 0.9f; }
            if (st.acc_dt_aim > 0.15f) { st.aim_rate_yaw *= 0.9f; st.aim_rate_pitch *= 0.9f; }

            if (g_cfg.ff_gain > 0.0f) {
                float yaw_ff_scale = 1.0f;
                if (g_cfg.ff_pitch_comp) {
                    // cos(pitch) projection guard, floored so steep aim keeps yaw authority.
                    yaw_ff_scale = std::fabs(std::cos((float)aim_pitch * DEG2RAD));
                    if (yaw_ff_scale < 0.35f) yaw_ff_scale = 0.35f;
                }
                ff_yaw   = clampf(st.ff_rate_yaw   / mr, -1.0f, 1.0f) * g_cfg.ff_gain * yaw_ff_scale;
                ff_pitch = clampf(st.ff_rate_pitch / mr, -1.0f, 1.0f) * g_cfg.ff_gain;
            }
            if (g_cfg.d_gain > 0.0f) {
                damp_yaw   = -clampf((st.aim_rate_yaw   - st.ff_rate_yaw)   / mr, -1.0f, 1.0f) * g_cfg.d_gain;
                damp_pitch = -clampf((st.aim_rate_pitch - st.ff_rate_pitch) / mr, -1.0f, 1.0f) * g_cfg.d_gain;
            }
        }
    }

    // SIGN: +stick.x increases yaw on this game.
    *out_rx = clampf((shape(err_yaw) + ff_yaw + damp_yaw) * g_cfg.yaw_sign, -1.0f, 1.0f);
    *out_ry = g_cfg.drive_pitch
        ? clampf((shape(err_pitch) + ff_pitch + damp_pitch) * g_cfg.pitch_sign, -1.0f, 1.0f)
        : 0.0f;
}

bool read_control_rotation(double* out_pitch, double* out_yaw, void** out_pc) {
    auto* pc = API::get()->get_player_controller(0);
    if (pc == nullptr) return false;
    if (out_pc != nullptr) *out_pc = (void*)pc;

    auto* base = reinterpret_cast<const uint8_t*>(pc);
    const double* rot = reinterpret_cast<const double*>(base + CONTROL_ROTATION_OFFSET);

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
