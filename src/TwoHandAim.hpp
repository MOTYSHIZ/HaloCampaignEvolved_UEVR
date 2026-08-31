// The two-handed hold, as an AIM feature.
//
// Put your support hand on the barrel and squeeze its grip: aim eases onto the line between your
// two hands, roll still taken from the aim hand. Release the grip and it eases back.
//
// WHY THIS IS NOT IN src\palettearm\. It started there, because that is where it was ported to.
// But the hold needs nothing from the palette route -- only two controller poses, a button, and
// somewhere to send the result. Leaving it there would have meant two-handed aiming worked only
// under armdriver=2, which has never been observed running on this build. It is an aim feature
// that the palette route happens to also render, so it lives with the aim and runs in EVERY
// armdriver mode, including 0.
//
// ── THE ONE RULE ────────────────────────────────────────────────────────────────────────────
//
// There are THREE places a controller orientation becomes aim in this plugin, and they are not
// derived from one another:
//
//   1. derive_ctrl_angles()   MotionAimControl.cpp -- the sim write and the render-rate law
//   2. the tick-side copy     Plugin.cpp, just above the reference capture -- the aim control law
//   3. the rendered weapon    Plugin.cpp, q_ctrl in the rig block
//
// Feed some and not others and the gun points where the shots do not go. That is not a
// hypothetical: it is the state this project shipped into its own palette route, where the arms
// followed the two-hand line and the bullets did not.
//
// So this module publishes ONE rotation per tick and every site applies that same rotation.
// Nothing else in the plugin may call TwoHandHold::effective_basis(). If you find yourself
// wanting a second blend, you want this one.
//
// WHY A ROTATION AND NOT A BLENDED DIRECTION. Sites 1/2 hold a forward vector from the AIM pose;
// site 3 holds an orientation from the GRIP pose. Those differ by a lot -- the 27,826-sample run
// recorded in MotionAimControl.cpp puts grip forward at a mean 41.5 degrees of pitch against the
// aim pose's 7.7. If each site blended its own forward it would compute its own agreement dot
// against the hand line and therefore its own smoothstep weight: one could sit at full authority
// while another was still entirely one-handed. A single shortest-arc swing cannot do that.
//
// ── WHAT IT DELIBERATELY DOES NOT TOUCH ─────────────────────────────────────────────────────
//
// The aim REFERENCE. The control law is desired = ref_aim + (ctrl - ref_ctrl), so a reference
// captured through the blend would cancel it exactly and the feature would do nothing at all.
// Worse, the Page Down calibration PERSISTS that pair to disk, so one hold live at the release
// edge bakes a two-hand offset into every future one-handed session. Capture sites take the raw
// orientation; see two_hand_bend_forward()'s note and the capture guard in Plugin.cpp.

#pragma once

#include "Math.hpp"

#include <cstdint>

namespace halo {

// Game thread, once per tick, in EVERY armdriver mode. Reads poses and the grip button, advances
// the hold, and publishes the swing.
//
// `gameplay_active` is the caller's judgement, because the flags it needs live in Plugin.cpp:
// pass FALSE in menus, in stick mode / vehicles, and while either calibration gesture is held.
// False drops the latch immediately and eases the blend out; it does not merely pause it.
void two_hand_update(float delta_seconds, bool gameplay_active, uint32_t tick);

// Apply the published swing. Any thread -- these read a seqlock-protected snapshot and take no
// poses, no locks and no API calls, because derive_ctrl_angles() is reached from the sim hook at
// roughly 2600 calls/sec.
//
// Both return false and leave the argument ALONE when the hold contributes nothing, which is the
// overwhelmingly common case: one atomic load and a branch.
//
// The quaternion form is for a controller ORIENTATION in raw VR space, applied before any frame
// conversion. The forward form is for a direction already extracted in that same space.
bool two_hand_bend_orientation(Quat* q);
bool two_hand_bend_forward(Vec3* fwd);

// True while the hold is engaged. For the reticule policy and the reload gesture's magazine
// suppression -- you cannot pull a magazine with both hands on the gun.
bool two_hand_latched();

// How far into the hold we are, 0..1 -- the SAME ramp the orientation bend uses, so a consumer
// that eases something else in stays in lockstep with the swing instead of inventing its own
// timing. Needed by the palette support hand, which has to travel onto the gun as the hold takes.
float two_hand_blend_weight();

// Drop the latch, the blend and the remembered hand line. Level load, respawn, teardown.
void two_hand_reset(const char* why);

// One line for the support diagnostics.
const char* two_hand_status();

// Consume the two-hand config keys. ONE link on Config.cpp's chain, never a new `else if` rung --
// that chain is at MSVC's 128-deep block limit and overflowing it is a fatal C1061 that points
// nowhere near the change that caused it.
bool two_hand_parse_key(const char* key, double value);

} // namespace halo
