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
// ---- THE REACH, for the grab guide -------------------------------------------------------------
//
// Where the support hand is and where it would have to be to latch, published because the zone
// geometry is computed here and must not be computed anywhere else. A guide that derived its own
// zone would start pointing at a spot that no longer latches the first time twohandalong/radius
// were retuned, and an affordance that lies is worse than none.
//
// VR SPACE, IN METRES, AND AS OFFSETS FROM THE AIM GRIP -- never as absolute positions.
//
// That is not a convenience, it is the bug-proofing. The caller (Plugin.cpp's rig block) holds the
// aim hand HEAD-RELATIVE, while the poses here are raw tracking space; subtracting one from the
// other silently adds the head position and puts the guide somewhere plausible but wrong. Both
// endpoints are differenced HERE, against the same pose from the same read, so there is no frame
// for the caller to get wrong -- it only has to rotate them, which is vr_to_rig()'s job.
struct TwoHandReach {
    bool  valid      = false;  // the poses were usable this tick; false means ignore every field
    bool  in_zone    = false;  // a grip press RIGHT NOW would latch -- the guide's show condition
    bool  latched    = false;  // already held, so the guide has done its job
    float along_m    = 0.0f;   // hand's distance DOWN THE BARREL from the rig origin
    float lateral_m  = 0.0f;   // hand's perpendicular distance off the barrel
    float clamped_along_m = 0.0f;  // that distance pulled into the zone: where it would grab
};

// Game thread only, valid for the tick in which two_hand_update() last ran.
const TwoHandReach& two_hand_reach();

// ---- THE ZONE MEASUREMENT, taken in the GUN's frame ---------------------------------------------
//
// Published by Plugin.cpp's rig block, which is the only place that has both the blessed
// VR-to-game transform (vr_to_rig) and the gun's world orientation (g_rigw_*). two_hand_update()
// runs LATER in the same tick, so this is never stale.
//
// x = distance down the barrel from the rig origin, y/z = off it. CENTIMETRES, game space.
// Everything the grab guide draws and everything the zone tests comes from this one measurement,
// so the beam cannot promise a grab the latch would refuse.
// ORIGIN IS THE AIM GRIP, axis is the GUN. Both halves matter and they were confused once:
//
// the AXIS moved to the gun deliberately -- the grab cylinder has to lie along the barrel the
// player can see, not along the controller's ray, which is what was asked for.
//
// the ORIGIN stayed on the GRIP, because the zone bounds (0.08..0.80 m) are authored as "this far
// FORWARD OF YOUR HAND". Moving the origin onto the rig component -- which sits further down the
// weapon -- shifted every reading by that distance, and the field log showed along going NEGATIVE
// (-0.088 to -0.217 m) while lateral stayed inside the radius. Axis right, origin wrong, and a
// negative along can never satisfy along > 0.08, so the hold simply stopped latching.
//
// `rig_off` carries that grip-to-rig difference separately, for the GUIDE -- which draws relative
// to the rig component and therefore does need it.
struct TwoHandZoneMeas {
    bool valid = false;
    Vec3 hand_gun{};           // support hand from the AIM GRIP, in the gun's frame, game cm
    Vec3 rig_off{};            // aim grip -> rig component, same frame. Drawing adds this; the
                               // zone test must not.
};
void two_hand_set_zone_measurement(const TwoHandZoneMeas& m);
const TwoHandZoneMeas& two_hand_zone_measurement();

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
