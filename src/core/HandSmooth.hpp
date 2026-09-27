#pragma once

// HAND SMOOTHING (core service SVC_HAND_SMOOTH, switched on by the handsmooth feature).
//
// A 1 Euro filter (Casiez, Roussel & Vogel, CHI 2012; the same family VRExpansionPlugin's
// UGripMotionControllerComponent runs as bSmoothWithEuroLowPassFunction) on the tracked controllers.
// It is a LAYER in the pose read, not a change to any consumer:
//
//     UEVR raw pose  ->  hand smoothing  ->  pose latch (armdriver 3)  ->  halo::get_pose()
//
// so aim, the arms, two-hand, holsters and reload all read the same steadied hands. The layer below
// it stays reachable: halo::get_pose_raw() (MotionAimControl.hpp) reads UEVR directly, with the same
// placeholder rejection, for a consumer that wants the unfiltered hand. Melee does by default
// (handsmoothmelee=0): its swing detector is tuned on raw motion.
//
// ONE OWNER, ONE STEP PER RENDERED FRAME. get_pose() is read from the Blam sim thread (~2600/s), the
// XInput hook, the game tick (~32 Hz) and the render thread. A filter advanced by whichever reader
// arrives would smooth by an amount that depends on who is reading and when. So hand_smooth_frame()
// runs once per rendered frame on the render thread (the stereo pre-callback, before anything in that
// frame reads a hand), reads each smoothed controller's grip+aim pair once, steps the filter, and
// PUBLISHES the result. Readers only copy the published pair: no UEVR call, no filter work. A
// publication older than 100 ms (no stereo frames: a load, a flat menu) is not served, and the read
// falls through to the live pose.
//
// THE FRAME. The filter runs on UEVR's raw tracking-space pose (metres), before the standing origin,
// the turn offset and the world scale are applied downstream -- VRE filters the controller's RELATIVE
// transform for the same reason. Snap turns, recentres and the head leash never pass through it.
//
// GRIP AND AIM SHARE ONE FILTER. The grip pose is filtered, and the aim pose is rebuilt from the
// filtered grip plus the raw grip->aim offset of the SAME read, so the two stay rigidly related. The
// pair is read grip, aim, grip: if the grip changed across that (UEVR updated mid-read) the aim is
// re-read, so the offset never mixes two frames.
//
// Differences from VRE's FBPEuroLowPassFilterTrans, on purpose: position and rotation have separate
// parameters (VRE shares one slope between cm/s and quaternion-component rates), and the cutoff is
// driven by the speed MAGNITUDE (VRE's is per axis, so a diagonal move is smoothed differently from
// the same move along one axis).
//
// The HMD is never smoothed. Keys: handsmoothhands, handsmoothposmin, handsmoothposbeta,
// handsmoothrotmin, handsmoothrotbeta, handsmoothdcut, handsmoothmelee, handsmoothlog (parsed here,
// owned by the handsmooth row in core/registry/Features.cpp). handsmoothlog is dev-build only.

#include "uevr/API.hpp"

#include <cstdint>

namespace halo {

// Render thread, once per rendered frame (features_stereo_pre_eye; the second eye of a frame is
// ignored). Steps the filter and publishes, or withdraws the publication while the service is off.
void hand_smooth_frame();

// The published smoothed pair for device idx. Either pointer may be null. False = not smoothed
// right now (service off, not a smoothed hand, the HMD, or no fresh publication): the caller reads
// live. Any thread; no UEVR call.
bool hand_smooth_get(int32_t idx, uevr::API::VR::Pose* grip, uevr::API::VR::Pose* aim);

// get_pose's read (the handsmooth feature's pose_latched slot).
bool hand_smooth_lookup(int32_t idx, bool use_aim, uevr::API::VR::Pose* out);

// Core keys (core/CoreKeys.cpp).
bool hand_smooth_parse_key(const char* key, const char* val, double v);

} // namespace halo
