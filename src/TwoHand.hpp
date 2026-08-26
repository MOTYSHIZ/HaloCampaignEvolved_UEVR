// THE TWO-HANDED HOLD.
//
// Grab the barrel with the support hand and the weapon starts aiming down the line between your
// two hands instead of down the aim hand alone. That is the whole feature: a rifle held at two
// points is steadier and points where a rifle actually points, and it costs nothing to leave off
// because a hold that is never engaged blends to zero influence.
//
// WHERE IT ACTS. One place: the forward vector inside derive_ctrl_angles(), after the source pose
// is chosen and before the sightline. Everything downstream -- the control law, the direct drive,
// the reticule, BlamAim's shot comparison -- reads its aim from that one derivation, so blending
// there means they all agree by construction. Blending further down would have produced a reticule
// that disagreed with the muzzle, which is the specific failure this codebase already has scar
// tissue about.
//
// WHAT IT DOES NOT TOUCH. Calibration. The grip/pitch/yaw trim and the per-weapon offsets sit
// underneath as they always did; two-handing rotates the result, it does not re-derive it.

#pragma once

#include "Math.hpp"

#include <atomic>

namespace halo {

// Pad buttons as last seen by the XInput hook (Plugin.cpp publishes every poll). The two-hand
// grip latch reads its mask from here; the hook must never be read from the game thread directly.
extern std::atomic<unsigned short> g_pad_buttons;

// Game thread, once per tick, with the tick's dt in seconds. Samples both controllers, decides
// whether the hold is engaged, and ramps the blend. Publishes everything the aim path needs, so
// the aim path itself never samples a second controller.
void two_hand_update(float dt);

// Blend `fwd` toward the hand-to-hand line, in place. Returns true if it changed anything.
//
// Callable from the XInput hook: it reads only atomics. That is deliberate -- the hook runs at
// render rate, far faster than tracking updates, so re-deriving the hand line there would be
// per-call work for a value that cannot have changed.
bool two_hand_blend(Vec3* fwd);

// The ONE rigid rotation the hold implies, VR space. False when the hold is not influencing aim.
//
// ONE rotation, applied to every pose the weapon is built from -- not a per-pose correction. The
// aim pose and the grip pose point along DIFFERENT axes (the grip's forward is the handle, not the
// barrel; the rig measures the difference as tilt_pitch/tilt_yaw), so rotating each so its own
// forward lands on the hand line applies a large wrong rotation to the grip and tears the two
// apart. The hold rotates the whole hand-and-weapon assembly, so it is one delta for all of it.
//
// Derived from whichever pose the AIM derivation points with, per aimsrc -- that is the one whose
// forward really is the barrel direction, and therefore the only one the hand line is comparable to.
//
// Shortest arc, so ROLL SURVIVES UNTOUCHED: the hand line is two positions and carries no roll of
// its own, and a weapon taking roll from it would twist about its own barrel whenever the support
// hand rolled.
bool two_hand_delta(Quat* out);

// Drop the hold immediately. Same rule the reload state machine follows: a person wearing a
// headset must never be left with the aim blended toward a hand the mod has stopped tracking.
void two_hand_reset();

// ---- published state -------------------------------------------------------------------------
// For diagnostics and for anything that wants to show the player what the hold is doing.
extern std::atomic<bool>  g_th_latched;    // the hold is engaged
extern std::atomic<bool>  g_th_in_zone;    // support hand is inside the grab zone right now
extern std::atomic<float> g_th_blend;      // 0..1 ramp, what the aim path actually applies

} // namespace halo
