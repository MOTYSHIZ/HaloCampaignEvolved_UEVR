// DriverProbe -- THE ARM DRIVER COMPARISON INSTRUMENT (driverprobe, core/config/ConfigFields.inl).
//
// WHAT IT IS FOR. Two drivers place the weapon: the author's palette arm driver (armdriver 2) and the
// weapon placement feature (armdriver 3). Each had its own evidence, printed from inside itself, so the
// evidence only existed while that driver ran and nothing measured both the same way. This measures the
// SAME quantities, from the SAME sources, whichever driver is running, so two sessions are comparable.
//
// WHAT IT MEASURES.
//   A. Hand to gun, one to one. The controller poses (aim hand and off hand, UEVR's raw get_pose, never a
//      driver's latched copy), the DRAWN weapon (the PrimaryWeapon socket of the posed first-person arms
//      mesh -- the point the weapon actor is attached at under every driver, Rig.cpp -- with the weapon
//      actor's own transform logged beside it so a disagreement between the two is itself visible), and the
//      drawn weapon expressed in the hand's own frame, so the number does not depend on which way the
//      player faces. Plus the target each driver computed, so drawn-vs-intended separates from
//      intended-vs-hand.
//   B. Performance. The author's per-tick PerfScope buckets (read through the plugin state bridge; they
//      fill only while his perflog key is on), the time spent inside feature slots per lane, the pose hook
//      on the sim thread (either driver's detour), the stereo callbacks' span, and the frame time.
//
// THE MEASUREMENT RULES IT OBEYS.
//   * ONE SNAPSHOT. An R row takes every input inside one stereo view callback: the finished view, the
//     head pose, both hands and the drawn socket are simultaneous there and nowhere else. A T row is taken
//     at the end of the engine tick; the one thing it cannot read at that instant is the rendered view, so
//     its derived columns go through the standing origin model with the room to world rotation of the last
//     R row, and the row carries that rotation's age (rw_age_ms) so the mix is visible, never hidden.
//   * VALUES, NOT DRIFTS. Every raw pose is in the row. The derived columns are conveniences that can be
//     recomputed from them.
//   * THE PROBE'S OWN RATE IS STATED. T rows: one per engine tick. R rows: one per rendered frame (view
//     index 0). The measured rates are printed in the summary line each second. A change faster than the
//     row rate cannot be seen by this instrument.
//
// OFF = NOTHING. With driverprobe=0 every entry point below returns after one atomic pointer load (the
// game thread entry also compares the key), no thread exists, no file is open and no memory is held: the
// whole state is one heap object created on the key's on edge.

#pragma once

#include "Math.hpp"
#include "uevr/API.h"

struct _XINPUT_STATE;

namespace halo {

// Lanes a feature slot's time is charged to (driver_probe_slot_done).
enum ProbeLane : int {
    PROBE_LANE_TICK_IN  = 0,   // game thread, inside the author's whole-tick PerfScope
    PROBE_LANE_TICK_OUT = 1,   // game thread, outside it (engine tick start / end, post engine tick)
    PROBE_LANE_RENDER   = 2,   // the stereo view callbacks
    PROBE_LANE_SIM      = 3,   // the sim thread's hooks
    PROBE_LANE_RESTAMP  = 4,   // the stereo view callbacks' render_refresh slot: the per-frame restamp
    PROBE_LANE_COUNT    = 5,
};

// GAME THREAD, the end of on_pre_engine_tick: the key's on and off edge, and the author's per-tick buckets.
void driver_probe_engine_tick_end();
// GAME THREAD, on_post_engine_tick, after the features' slots: the T row.
void driver_probe_post_engine_tick();

// THE STEREO VIEW CALLBACKS. begin/end bracket the span between the first and the last hook point of a
// callback; post_view notes the finished view; render_sample takes the R row (view index 0).
void driver_probe_stereo_begin();
void driver_probe_stereo_end();
void driver_probe_post_view(int index, float world_to_meters, const UEVR_Vector3f* position,
                            const UEVR_Rotatorf* rotation, bool is_double);
void driver_probe_render_sample(int index);

// ANY THREAD. The probe's clock: 0 while the probe is off, else a QPC count to hand back to a _done call.
long long driver_probe_clock();
// The pose hook on the sim thread finished its post-pass. who = the arm driver mode whose detour it was.
void driver_probe_pose_hook_done(long long t0, int who);
// A feature slot returned. pose_provider = the feature that provides the palette pose (the weapon placer).
void driver_probe_slot_done(long long t0, bool pose_provider, int lane);

// THE TARGET A DRIVER COMPUTED, for drawn-vs-intended.
//   mesh:   the intended socket position in the arms mesh's palette space (palette units), sim thread.
//   parent: the intended weapon point as a world-axes offset from the rig's attach parent (cm) and the
//           mesh rotation that goes with it, game thread.
void driver_probe_note_intent_mesh(float x_u, float y_u, float z_u);
void driver_probe_note_intent_parent(bool valid, const Vec3& off_world_cm, const Quat& q_mesh);

// The XInput hook's thread: the raw pad, for the moving / sprinting tags.
void driver_probe_note_pad(const _XINPUT_STATE* state);

// Plugin teardown: stop the writer, flush and close. Safe when nothing is running.
void driver_probe_shutdown();

} // namespace halo
