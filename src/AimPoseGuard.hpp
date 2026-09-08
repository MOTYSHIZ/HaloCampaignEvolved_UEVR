#pragma once
//
// IS THE CONTROLLER POSE EMPTY? One predicate, computed once, read by two callers.
//
// ---------------------------------------------------------------------------------------------
// THE FAULT IT DETECTS. When an OpenXR session leaves the FOCUSED state -- a runtime overlay or
// dashboard opens, a level load stalls frame submission, the window leaves the foreground --
// xrSyncActions returns XR_SESSION_NOT_FOCUSED and the runtime stops updating action poses.
// UEVR's get_pose() still returns SUCCESS; the pose it hands back is simply empty. One 16-minute
// session logged 13,397 consecutive sync failures across six focus drops, two of which never
// recovered before the session ended.
//
// Without a guard, the direct write assigns aim to wherever the empty pose points: measured as a
// 76 degree slam (aim -83.3 -> -7.4 in one frame, 2026-09-02 20:41:00), then held for the 3m38s
// the session stayed unfocused.
//
// ---------------------------------------------------------------------------------------------
// TEST THE POSE, NOT AN ANGLE DERIVED FROM IT. This is the correction that matters, and the first
// version of this file got it wrong in a way that made the whole guard dead code.
//
// That version took `ctrl_yaw` / `ctrl_pitch` -- the values handed to the aim law -- and tested
// them for zero. But derive_ctrl_angles() folds the accumulated snap-turn into the yaw it returns:
//
//     *out_yaw = wrap180(atan2(fwd.x, -fwd.z) * RAD2DEG + g_cfg.aim_turn * g_turn_offset.load());
//
// so an EMPTY pose still yields ctrl_yaw == the current turn offset, not zero. In the field session
// that exposed this the offset was -20.4 deg, and the guard sat silent through nine separate
// dropouts: `CONTROLLER POSITION MISSING` fired nine times, `AIM FROZEN` zero times.
//
// The lesson is not "add the turn offset back". It is that a guard must key on the RAW fact it
// claims to detect. Position and the quaternion's vector part are what the runtime actually fills
// in; every angle downstream has had a frame convention, a calibration reference or an accumulated
// offset applied to it, and any of those can be non-zero while the pose is empty. Cf. the standing
// rule in this repo about measuring in a frame that does not move with the thing being tested.
//
// ---------------------------------------------------------------------------------------------
// WHY BOTH HALVES ARE REQUIRED, and this is the subtlety that survives from the first version:
//
//   position zero + rotation LIVE  -> a REAL, DIFFERENT fault (PSVR2 over ALVR, 2026-08-18). The
//                                     runtime publishes orientation but never translation. Aim only
//                                     needs direction, so it works perfectly and MUST keep working.
//                                     Only the weapon rig is affected. Freezing here would take
//                                     motion aim away from someone who still has it.
//
//   position zero + rotation ZERO  -> there is no pose at all. This is the one to freeze on.
//
// Testing position alone cannot tell those apart, and they need opposite responses.
//
// The rotation half tests the quaternion's VECTOR part (x, y, z) for exact zero, which catches
// both empty encodings seen in the field: a true identity (w=1, x=y=z=0) and the unnormalised
// (w=0.5, x=y=z=0) that an untracked device reports. It deliberately does not look at w, so it
// does not have to care which one a given runtime chooses.
//
// NO RUN-LENGTH FILTER, unlike the rig's `position_dead` (which waits 120 ticks). The failure is
// asymmetric: freezing for one spurious frame is invisible, because "frozen" means the aim simply
// is not written and it keeps the value it already had. Driving from one dead frame is a
// full-scale slam. So this answers for the CURRENT frame and the caller acts immediately.
//
// EXACT equality on floats is deliberate, not an oversight. The signature being detected is a
// zero-filled struct, so the bits are exactly zero. A real pose does not land on 0.000000 in
// position and in all three quaternion vector components at once. Do not "improve" this into an
// epsilon compare: a tolerance would start catching a hand held near the origin at a near-identity
// orientation, which is a pose a player can actually hold.

namespace halo {

// pos_is_zero -- controller TRANSLATION is exactly (0,0,0) this frame.
// qx / qy / qz -- the VECTOR part of the controller's orientation quaternion, straight from the
//                 runtime. Pass the raw values; do not normalise, convert frames, or apply any
//                 calibration or turn offset first -- doing so is exactly the bug this file
//                 documents above.
inline bool aim_pose_is_empty(bool pos_is_zero, float qx, float qy, float qz) {
    return pos_is_zero && qx == 0.0f && qy == 0.0f && qz == 0.0f;
}

} // namespace halo
