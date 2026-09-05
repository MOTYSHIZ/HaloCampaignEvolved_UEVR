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

// Where ControlRotation sits on the PlayerController. RESOLVED AT RUNTIME, with the measured value
// below as both the fallback and the expectation.
//
// This is the widest-reaching build-dependent constant in the mod, and the only one that is a UE
// offset rather than a Blam one. read_control_rotation() gates update()'s early-out, ABOVE the
// turning block -- so if this is wrong the sanity gate fails closed and snap turn never even
// accumulates. Not "aim does not replicate": the mod does nothing at all. And being a UE offset, it
// can move on an engine-side change that never touches the Blam sim, which none of the Blam
// hardening would catch.
//
// It is also the easiest to resolve honestly, because it is a real UPROPERTY: reflection gives its
// address on a live PlayerController, and subtracting the object base gives the offset. That is done
// ONCE, on the game thread, and every read afterwards is the same raw memory read as before -- which
// is what the "never reflection" note further down is actually about (the hot path and the hook
// thread), not about one-time resolution.
extern std::atomic<size_t> g_control_rotation_offset;

// The value measured on the build this was developed against. Kept as the fallback when reflection
// cannot answer, and as the expectation that makes a moved offset visible in the log rather than
// merely survivable.
// ADDR-HYGIENE: resolved -- resolve_control_rotation_offset() asks UE reflection for the real
// offset and range-checks the answer before adopting it; this is the fallback when reflection
// declines, and which one is in force is logged once.
constexpr size_t CONTROL_ROTATION_OFFSET_EXPECTED = 0x350;

// One-shot, GAME THREAD ONLY -- it walks reflection, which the hook thread must never do. Safe to
// call every tick; it latches after the first answer. Takes the PlayerController the caller already
// has, so it never fetches one of its own.
void resolve_control_rotation_offset(void* pc);

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

// ---- the aim SETPOINT ------------------------------------------------------------------------
// Where the controller is asking the aim to be, in game yaw/pitch -- the same value the control law
// drives toward. Published so the reticule can be drawn from INTENT rather than from the achieved
// Blam aim: the loop's residual is high-frequency jitter that is distracting on screen while being
// negligible at the muzzle. Written by both callers of aim_control_law; the hook path runs at render
// rate, so a reader gets the freshest setpoint available.
//
// Only meaningful while g_aim_law_armed -- otherwise it is the last value from when the loop last
// ran, and a reticule drawn from it would lie confidently. Consumers must check.
extern std::atomic<float> g_desired_yaw, g_desired_pitch;

// TRUE while the Page Down aim calibration is held. Published here because the flag itself lives in
// Plugin.cpp's anonymous namespace, and every path that DRIVES the aim has to observe it or the
// reticle will not freeze. That is not optional bookkeeping: the whole gesture is "the aim actuator
// goes silent so you can point the controller at a stationary reticle", so any driver that ignores
// it keeps the aim chasing the hand and the calibration cannot be performed at all.
extern std::atomic<bool> g_aim_calibrating;

// TRUE while STICK MODE is engaged (vehicle seats, cutscenes, death). Published for the same reason
// and with the same urgency as the flag above: the control law is disarmed in stick mode, but the
// blamangles write is driven from a SIM-THREAD hook on the game's own orientation getter, so it is
// not on the law's code path and does not stand down with it. Left ungated, motion aim keeps
// steering the vehicle camera while the player's stick is supposed to own it.
extern std::atomic<bool> g_stick_mode_active;

// TRUE while the frontend or an in-game menu widget owns the input. Published for the third time
// for the same reason as the two flags above -- Plugin.cpp keeps this state in its anonymous
// namespace, which a sim-thread hook cannot see. DEFAULTS TO TRUE so that "we have not established
// that gameplay is running" never reads as "gameplay is running": the BlamDrive watchdog uses it to
// decide whether the sim ought to be calling the hook at all, and starting that count early
// condemns a healthy address before the game exists. See the watchdog in BlamDrive.cpp.
extern std::atomic<bool> g_menu_active;

// TRUE only while the FRONTEND owns the session -- the main menu, where there is no gameplay world
// and the simulation is not ticking. Deliberately NARROWER than g_menu_active above, and the two
// must not be confused:
//
//   g_menu_active     = frontend OR an in-game menu widget is open. The right question for "who
//                       owns the input", which is what it was added for.
//   g_frontend_active = frontend only. The right question for "is the simulation running at all".
//
// Measured 2026-08-15, and this is why both exist: with a menu widget open during real gameplay the
// sim keeps calling its orientation getter (hook observed running while inmenu=1 for three minutes
// straight). A watchdog that treats "a widget is open" as "the game is not running" therefore stops
// counting time in which the thing it watches was perfectly possible -- and a watchdog that cannot
// fire is the failure it was written to prevent, just pointed the other way.
extern std::atomic<bool> g_frontend_active;

// The gameplay PlayerController, published for READ-ONLY consumers on other threads.
//
// Deliberately separate from g_aim_law_pc, which is published only once the aim law is ARMED so an
// early-out leaves the law steering from nothing rather than from a stale reference. That is right
// for a driver and wrong for a reader: the BlamDrive layout guard only wants to READ
// ControlRotation, and tying that to the law's arming meant it could not validate at all whenever
// the motion stack stood down (no controller poses, calibration hold, idle). Measured 2026-08-15:
// the guard sat blocked for ten minutes under exactly that condition, holding the sim write off.
extern std::atomic<void*> g_read_only_pc;

// Magnitude of the smoothed SETPOINT angular velocity, deg/s -- how fast the player is actually
// swinging, as the control law already measures it for feedforward. Published so the reticule can
// stop smoothing during large movements: jitter only matters when the hand is near-still, and
// filtering during a fast swing buys nothing while costing visible lag.
extern std::atomic<float> g_setpoint_rate_dps;

// The player's accumulated snap/smooth turn, in degrees. Turning state, but the aim setpoint folds
// it in -- see the note in MotionAimControl.cpp.
extern std::atomic<float> g_turn_offset;

// ---- adaptive gain ---------------------------------------------------------------------------
// The loop drives a rate actuator whose deg/s per unit of stick is set by the GAME's controller
// sensitivity, which no two players set alike. So it is measured live rather than assumed.
// ---- plant-gain measurement ------------------------------------------------------------------
// Measures deg/s per unit of stick and adapts the loop gain to it. Runs on WHICHEVER path is
// driving, which is the point: it used to run only on the tick path and was explicitly skipped
// under aim_rate_render -- which defaults ON, so the gain stayed 0, and feedforward and damping
// (both gated on gain > 10) never executed at all in a default configuration.
//
// It integrates the applied deflection BETWEEN AIM CHANGES rather than pairing one stick reading
// with one aim reading. That is what makes it correct on the render path: Blam's aim updates near
// 30 Hz while the hook runs far faster, so most calls see no aim movement, and the deflection
// varies across the interval that produced any movement that does appear. Averaging over the
// interval is the honest pairing; sampling once was the reason this was disabled there.
//
// Each caller owns a state, so the two paths never share history.
struct GainMeasState {
    double prev_aim   = 0.0;
    bool   have_prev  = false;
    float  acc_dt     = 0.0f;   // time since the aim last CHANGED
    float  acc_out_dt = 0.0f;   // integral of |stick| over acc_dt, for the interval mean
};
void update_gain_measurement(GainMeasState& st, double aim_yaw, float applied_now, float dt);

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
// ridx_override lets a caller supply the controller index itself. Without it this reads
// g_aim_law_ridx, which is only published by the STICK path -- so under direct drive it is -1 and
// the derivation fails, taking any consumer down with it.
bool derive_ctrl_angles(float* out_yaw, float* out_pitch, int32_t ridx_override = -1);

// Where the controller is asking the aim to be, RIGHT NOW, computed from a fresh pose in the
// caller's own callback. Same quantity as g_desired_yaw but sampled rather than subscribed to --
// which matters to any consumer that must pair it with another live value at one instant.
//
// g_desired_yaw is *published* by aim_control_law(), so it always carries that law's cadence and
// call-order. On the default configuration (aimrate=1) the law runs in the XInput hook AFTER the
// movement-frame rotation in the same callback, so a reader there gets the PREVIOUS poll's
// setpoint paired against the CURRENT rendered view -- an error proportional to how fast the hand
// is turning. It also freezes outright on any early-out in that block (calibration held, law
// disarmed, ControlRotation unreadable).
//
// It takes the controller index straight from UEVR rather than g_aim_law_ridx, because that index
// is only published by the stick path and reads -1 under direct drive.
//
// This is the arithmetic the blamangles driver writes into the sim's angular control state, so a
// consumer that calls this is using the same value the sim is being given -- not a second value
// believed to be equal to it.
bool desired_aim_now(float* out_yaw, float* out_pitch);

// Current aim, by RAW MEMORY READ at CONTROL_ROTATION_OFFSET -- never reflection, which
// access-violates on this game's Blam objects. The _hook variant uses the published PlayerController
// so it can run off the game thread.
bool read_control_rotation(double* out_pitch, double* out_yaw, void** out_pc);
bool read_control_rotation_hook(double* out_pitch, double* out_yaw);

// A tracked device pose, rejecting the identity placeholder UEVR returns before tracking is live.
bool get_pose(UEVR_TrackedDeviceIndex idx, Vec3* pos, Quat* rot, bool use_aim);

// The aim sightline's body reference, honouring aimorigin AND the leash state. Both sightline
// sites call this; see the definition for why an unleashed head cannot use the standing origin.
bool aim_sightline_origin(Vec3* out);


// ---- MELEE AIM HOLD -- DECLARED, NOT POPULATED IN THIS TREE.
//
// Holster.cpp clears these when a holster action vetoes a melee. In blindcowboy24's PR they are
// written by his aim-along-the-swing melee work, which is a separate feature from the holsters
// and was not taken with this extraction. Left at zero: holster_melee_veto() still stands the
// swing detector down near a shoulder (that is local to Holster.cpp), and clearing an already-
// zero hold is a no-op. Populate these if the swing-aimed melee is ported later.
extern std::atomic<float>     g_melee_aim_ctrl_yaw;
extern std::atomic<float>     g_melee_aim_ctrl_pitch;
extern std::atomic<long long> g_melee_aim_hold_until;   // 0 = no hold in effect
} // namespace halo
