// SOCKET-CANCELLING WEAPON DRIVE -- put the gun on the controller WITHOUT moving the arm mesh.
//
// THE PROBLEM THIS EXISTS FOR. The legacy rig driver delivers controller tracking by placing
// BPC_FP_SkeletalMesh_C, and per Rig.hpp:15 that component moves ARMS + GUN together. So the
// shoulders are, structurally, children of the aim controller: rotate the controller and the whole
// upper body swings with it. Once palettearm is posing the arms as a body hung off the HEAD, that
// is the single thing standing between "arms parented to my hand" and "arms attached to me".
//
// THE COMPOSITION, and why it is exactly this and not something simpler:
//
//     W = M o S o R        weapon world = mesh world, then the socket, then the weapon root's
//                          own relative transform
//
// Today we drive M := C (the calibrated controller transform) and never write R, so R is identity
// and the gun rides along at W = C o S. To unpin the shoulders we must stop writing M -- and then
// the gun must be put back by writing R instead:
//
//     want  W = C o S      (bit-for-bit what legacy produces -- that IS the no-recalibration test)
//     have  W = M o S o R
//     =>    R = S^-1 o M^-1 o C o S
//
// C PASSES THROUGH UNTOUCHED. That is the load-bearing property, not an implementation detail:
// grip_deg/grip_yaw/grip_roll, off_x/y/z and the per-weapon wpnoff deltas (WeaponOffset.cpp writes
// those straight into g_cfg, so they are already inside C) mean the same thing in both paths. A
// recalibration performed with this mode on is therefore valid with it off, and vice versa. The
// socket compensation lives ONLY in the runtime transform and is never folded into stored config
// -- if you ever find yourself tempted to bake S into off_x, that is the bug.
//
// Sanity check that falls out of the algebra: ask for the mesh where it already is (C == M) and
// R = S^-1 o S = identity. Nothing moves, as it must.
//
// !!! S IS LIVE, AND MUST BE. Plugin.cpp measures S as the weapon expressed in the mesh's frame and
// notes it "feeds back nowhere: both terms move together" -- true only while the weapon root is
// unwritten. An early version therefore LATCHED S once. Two measurements killed that:
//
//   * the first available reading is not the socket (the weapon actor's transform has not moved onto
//     the socket yet), so it latched (0,0,0) and silently deleted the whole socket term; and
//   * S is not constant anyway -- it rides the animation, excursing ~11 cm, exactly as Rig.hpp's
//     "socket_offset(ANIMATED POSE)" always said. Legacy lets the gun ride that, so reproducing
//     legacy requires the live value.
//
// The feedback the latch was avoiding is instead removed exactly, because we know what we wrote:
//
//     S_measured = S_true o R_prev     =>     S_true = S_measured o R_prev^-1
//
// A correction, not an integrator -- it divides out the very transform that produced the reading, so
// it cannot compound. R_prev is identity on a fresh weapon actor, which is why the actor pointer
// resets it. WeaponDrive.cpp carries the full measurement history.

#pragma once

#include "Math.hpp"

namespace halo {

// True when this drive owns the weapon transform, so the legacy mesh writes must stand down.
// Read from BOTH the game thread and the render thread; false whenever the mode is off, the socket
// latch is not yet held, or the self-check has tripped the fail-closed latch.
bool weapon_drive_owns();

// Cheap "is the mode switched on at all" -- distinct from weapon_drive_owns(), which additionally
// requires that we are actually in a position to drive. Used to switch on the socket measurement
// this module consumes.
bool weapon_drive_enabled();

// GAME THREAD, once per tick. Resolves the weapon targets and takes the socket latch. Kept separate
// from weapon_drive_apply() because resolving the weapon root walks reflection, which has no business
// running on the render callback -- the same reason g_rig_component is published rather than resolved
// there.
// `player_unarmed` is Plugin.cpp's g_on_foot_unarmed. Passed in rather than read as a global so
// this module keeps no opinion about the pawn -- and because a resolved weapon pointer is NOT
// evidence the player is holding anything (see WeaponDrive.cpp: it returns a stale pooled actor).
void weapon_drive_tick(bool player_unarmed);

// Apply the retarget for this frame. `c_rot`/`c_loc` are the calibrated WORLD transform the mesh
// would have been given -- i.e. C. Call from the render-rate re-apply, the same place the legacy
// mesh write happens, so the gun gets the same fresh-parent treatment the arms used to.
void weapon_drive_apply(const Quat& c_rot, const Vec3& c_loc);

// Hand the gun back: zero the weapon root's relative transform so legacy composes on a clean
// child. MUST be called on every exit path -- disengage, trip, mode off, level change -- or our
// last R stays written on the weapon and legacy silently composes on top of it.
void weapon_drive_release();

// Drop the socket state and clear the fail-closed trip. Call on level transition / rig loss.
void weapon_drive_reset(const char* why);

// One line for the periodic status block: latch state, self-check error in cm, trip count.
const char* weapon_drive_status();

} // namespace halo
