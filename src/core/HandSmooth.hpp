#pragma once

// HAND SMOOTHING (core service SVC_HAND_SMOOTH, switched on by the handsmooth feature).
//
// A 1 Euro filter (Casiez, Roussel & Vogel, CHI 2012; the same family VRExpansionPlugin's
// UGripMotionControllerComponent runs as bSmoothWithEuroLowPassFunction) on the tracked controllers,
// applied at the ONE place every reader gets a hand from: halo::get_pose(), through the
// features_pose_latched slot, and the pose latch's snapshot. Aim, the arms, two-hand, gestures,
// holsters and reload are all downstream of it, so they cannot disagree about where the hand is.
//
// WHY IT STEPS ON A CHANGED SAMPLE AND NOT ON A CALL. get_pose() is read from the Blam sim thread
// (~2600 calls/s), the XInput hook, the game tick (~32 Hz) and the render thread. A stateful filter
// stepped per call would be fed mostly duplicates of one sample at tiny dt: the speed estimate swings
// between zero and huge and the output depends on thread timing. So there is ONE filter per hand,
// under a lock, and it advances only when UEVR hands back a different raw pose, with dt measured
// since the last step. Every reader between two samples gets the identical filtered pose.
//
// THE FRAME. The filter runs on UEVR's raw tracking-space pose (metres), before the standing origin,
// the turn offset and the world scale are applied downstream -- VRE filters the controller's RELATIVE
// transform for the same reason. Snap turns, recentres and the head leash never pass through it;
// only real hand motion does.
//
// GRIP AND AIM SHARE ONE FILTER. The grip pose is filtered, and the aim pose is rebuilt from the
// filtered grip plus this read's raw grip->aim offset, so the two stay rigidly related. The aim pose's
// translation is known to carry teleport-scale readings, so filtering it on its own would poison the
// speed estimate.
//
// Differences from VRE's FBPEuroLowPassFilterTrans, on purpose: position and rotation have separate
// parameters (VRE shares one slope between cm/s and quaternion-component rates), and the cutoff is
// driven by the speed MAGNITUDE (VRE's is per axis, so a diagonal move is smoothed differently from
// the same move along one axis).
//
// The HMD is never smoothed. Keys: handsmoothhands, handsmoothposmin, handsmoothposbeta,
// handsmoothrotmin, handsmoothrotbeta, handsmoothdcut, handsmoothlog (parsed here, owned by the
// handsmooth row in core/registry/Features.cpp).

#include "uevr/API.hpp"

#include <cstdint>

namespace halo {

// Replace a raw pose pair read for device idx with the smoothed pair. aim may be null (a grip-only
// read). Returns false and leaves both untouched when the service is off, idx is not a smoothed
// controller, or the pose is UEVR's pre-tracking placeholder. Any thread.
bool hand_smooth_apply(int32_t idx, uevr::API::VR::Pose* grip, uevr::API::VR::Pose* aim);

// get_pose's read (the handsmooth feature's pose_latched slot): a fresh raw read of idx, smoothed.
// False = not smoothed, the caller reads live as before. Any thread.
bool hand_smooth_lookup(int32_t idx, bool use_aim, uevr::API::VR::Pose* out);

// Core keys (core/CoreKeys.cpp).
bool hand_smooth_parse_key(const char* key, const char* val, double v);

} // namespace halo
