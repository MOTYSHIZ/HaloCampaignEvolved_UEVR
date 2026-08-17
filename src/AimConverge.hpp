// AIM CONVERGENCE -- make the SHOT land where the player is pointing, not merely parallel to it.
//
// THE PROBLEM, which only exists in 6DoF.
// Shots leave from where Blam says they do -- the game camera -- while the player sights along a ray
// from their own EYE. Those are the same point only while the head is pinned to the camera. Let the
// head off the leash (hmdleash=0) and they separate by up to the size of the room.
//
// Aim is commanded as a DIRECTION. A direction alone cannot express "hit that thing": a shot fired
// from C parallel to a sightline from E is displaced from that sightline by (E - C) at every range.
// The old sightline construction papered over this by assuming a fixed target range (xdist), so aim
// was exact at ~10 m and wrong everywhere else, by an amount proportional to how far the eye had
// drifted from the camera. That is the "my shots land where the reticule is, not where I'm pointing"
// report.
//
// THE FIX. A direction plus a RANGE does express "hit that thing". The reticule already traces the
// player's sightline to find what it hits, so the range is measured rather than assumed:
//
//      T   = E + u * d          the point the player is actually pointing at
//      aim = normalize(T - C)   the direction that puts a shot from C through it
//          = normalize(delta + u * d),    delta = E - C
//
// C cancels. Only the eye-to-shot-origin OFFSET and the RANGE are needed, so nothing here depends on
// world positions agreeing between two coordinate spaces -- the one class of bug this file would
// otherwise be built on.
//
// `delta` is measured, not modelled: it is exactly what UEVR adds to the view between the pre- and
// post-stereo callbacks, sampled at the source. Averaged across both eyes, so the correction is
// anchored at the cyclopean eye and half an IPD does not read as a permanent aim bias.
//
// WHEN THE HEAD IS LEASHED (the default) delta is ~0 and every function here declines to act, so the
// shipped configuration is bit-for-bit what it was. This is checked explicitly rather than left to
// arithmetic that "should" come out as a no-op.

#pragma once

#include "Math.hpp"

#include <atomic>

namespace halo {

// ---- published by the stereo callbacks (render thread), read on the game thread ---------------
// EYE MINUS SHOT ORIGIN, in game-space cm: the post-callback view position minus the pre-callback
// one, averaged over both eyes. Zero until both eyes have been seen once.
extern std::atomic<float> g_eye_delta_x, g_eye_delta_y, g_eye_delta_z;
extern std::atomic<bool>  g_have_eye_delta;

// Called from on_pre_/on_post_calculate_stereo_view_offset. Render thread: no allocation, no
// reflection, no logging.
void aim_converge_note_pre(int view_index, float x, float y, float z);
void aim_converge_note_post(int view_index, float x, float y, float z);

// ---- convergence range ------------------------------------------------------------------------
// Feed the range the sightline trace measured, in cm, once per tick. Smoothed, because the world has
// depth DISCONTINUITIES and the correction is proportional to 1/range: sweeping off a near wall onto
// a distant one would otherwise snap the aim by the whole difference in one frame.
void aim_converge_feed(float hit_cm, bool hit, float dt);

// Drop the smoothed range. Call on anything that invalidates the geometry under the reticule --
// level load, respawn, re-anchor -- so the first correction after it is measured, not inherited.
void aim_converge_reset();

// The smoothed range in cm, 0 when there is none yet.
float aim_converge_range();

// ---- the correction -----------------------------------------------------------------------------
// TRUE when the correction is live: enabled, a range is known, and the eye has actually diverged
// from the shot origin by enough to matter. The reticule asks this to decide which ray to draw on,
// so that the marker and the shot always describe the same geometry.
bool aim_converge_engaged();

// The measured eye-minus-shot-origin offset. False when it is not known yet.
bool aim_converge_delta(Vec3* out);

// Bend a setpoint so a shot leaving the shot origin passes through the point the player's own
// sightline hits. Angles in degrees, UE convention. Leaves them UNTOUCHED and returns false whenever
// the correction is not live or comes out implausible -- every caller then keeps today's behaviour
// exactly, which is what makes this safe to apply in all three drive paths.
bool aim_converge_apply(float* yaw, float* pitch);

} // namespace halo
