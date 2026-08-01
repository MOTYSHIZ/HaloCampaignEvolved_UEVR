// The aim control loop -- the surface consumed by update() and the XInput hook in Plugin.cpp.
//
// See MotionAimControl.cpp for why aim is steered with a rate actuator rather than written, and for the four
// parts of the control law itself.

#pragma once

// API.hpp, NOT Plugin.hpp -- Plugin.hpp defines the plugin entry points and may only be included
// by Plugin.cpp. See UeObject.hpp.
#include "uevr/API.hpp"
#include "Math.hpp"

#include <atomic>
#include <cstdint>

namespace halo {

// ControlRotation offset on this build's PlayerController. MEASURED (double-precision FRotator,
// LWC) -- not derived, so it is validated before use and the driver fails closed if it looks wrong.
constexpr size_t CONTROL_ROTATION_OFFSET = 0x350;

// deg/s per unit deflection measured during the hand calibration that produced full_deg=6.0.
constexpr float REFERENCE_RATE_DPS = 152.0f;   // ~137 deg/s at 0.90 deflection

// ---- aim reference ---------------------------------------------------------------------------
// The empirical mapping between controller yaw in VR space and Blam yaw in game space. Anything
// that rotates the VR world underneath it (UEVR recentring, snap turn) invalidates it.
extern std::atomic<float> g_ref_ctrl_yaw, g_ref_aim_yaw;
extern std::atomic<float> g_ref_ctrl_pitch, g_ref_aim_pitch;

// Armed by the game thread each tick when every gate passes; the hook-side law runs only while this
// is true. The controller index and player controller are published alongside so the hook never
// walks engine structures itself.
extern std::atomic<bool>    g_aim_law_armed;
extern std::atomic<int32_t> g_aim_law_ridx;
extern std::atomic<void*>   g_aim_law_pc;

// The player's accumulated snap/smooth turn, in degrees. Turning state, but the aim setpoint folds
// it in -- see the note in MotionAimControl.cpp.
extern std::atomic<float> g_turn_offset;

// ---- adaptive gain ---------------------------------------------------------------------------
// The loop drives a rate actuator whose deg/s per unit of stick is set by the GAME's controller
// sensitivity, which no two players set alike. So it is measured live rather than assumed.
extern std::atomic<float>    g_gain_scale;   // multiplies full_deg
extern std::atomic<float>    g_meas_rate;    // deg/s per unit deflection, smoothed
extern std::atomic<bool>     g_gain_logged;
extern std::atomic<bool>     g_gain_hold;    // suppresses sampling right after a reference recapture
extern std::atomic<uint32_t> g_gain_hold_until;

// ---- the control law -------------------------------------------------------------------------
// The aim control law -- shaped P with a Schmitt-gated deadband, velocity feedforward, and rate
// damping -- shared between the tick path and the render-rate hook path. Each caller owns an
// AimLawState so the two paths never share filter history.
struct AimLawState {
    bool   parked_yaw = true, parked_pitch = true;
    float  prev_des_yaw = 0.0f, prev_des_pitch = 0.0f;
    double prev_aim_y = 0.0, prev_aim_p = 0.0;
    bool   have_prev = false;
    float  ff_rate_yaw = 0.0f, ff_rate_pitch = 0.0f;
    float  aim_rate_yaw = 0.0f, aim_rate_pitch = 0.0f;
    // Time accumulated since the tracked angles last CHANGED. At XInput-poll cadence (sub-ms)
    // both the pose (90 Hz) and the Blam aim (30 Hz) hold still across most calls; a per-call
    // derivative then reads a whole 30 Hz step over one millisecond as thousands of deg/s, trips
    // the teleport gate, and silently disables feedforward and damping -- an undamped limit
    // cycle. Rates are therefore measured across change boundaries, over the real elapsed time
    // between them.
    float  acc_dt_des = 0.0f, acc_dt_aim = 0.0f;
};

// Deadzone-compensated shaping. Exact zero at zero error (never creep); anything past the deadband
// jumps straight to `floor` so it actually crosses the game's deadzone, then rises to max_out.
float shape(float err_deg);

// Error -> stick deflection. Writes out_rx/out_ry, each clamped to +-1.
void aim_control_law(AimLawState& st, float ctrl_yaw, float ctrl_pitch,
                     double aim_yaw, double aim_pitch, float dt,
                     float* out_rx, float* out_ry);

// ---- reading the world -----------------------------------------------------------------------
// Controller pose -> aim angles in game-space degrees. Callable from the XInput hook as well as the
// tick: everything it touches is a UEVR API read or an atomic.
bool derive_ctrl_angles(float* out_yaw, float* out_pitch);

// Current aim, by RAW MEMORY READ at CONTROL_ROTATION_OFFSET -- never reflection, which
// access-violates on this game's Blam objects. The _hook variant uses the published PlayerController
// so it can run off the game thread.
bool read_control_rotation(double* out_pitch, double* out_yaw, void** out_pc);
bool read_control_rotation_hook(double* out_pitch, double* out_yaw);

// A tracked device pose, rejecting the identity placeholder UEVR returns before tracking is live.
bool get_pose(UEVR_TrackedDeviceIndex idx, Vec3* pos, Quat* rot, bool use_aim);

} // namespace halo
