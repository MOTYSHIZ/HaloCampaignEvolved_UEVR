#pragma once

// HOOK POINTS IN MotionAimControl.cpp. Definitions: src/features/FeatureList.cpp. Any thread the aim
// derivation runs on (the XInput hook, the game tick, the sim thread's Blam writer).

#include "Math.hpp"
#include "uevr/API.hpp"

namespace halo {

// shotpoint_aim_active, first term: true = a feature owns the aim (the author's shot-point aim stands down).
bool features_aim_owned_by_feature();
// derive_ctrl_angles, at both aim source poses: the rotation the forward is taken from, noted per thread and returned:
// the placement owner's aim fix on q_src while a feature owns the aim, else his_fixed (apply_aim_fix on the same pose).
Quat features_aim_source(const Quat& q_src, const Quat& his_fixed);
// derive_ctrl_angles, in place of the shot-point direction and the two-handed bend: true = a feature set
// the forward (the palette weapon's barrel and its two-handed hold), and the author's block is skipped.
bool features_aim_forward(Vec3* fwd);
// aim_control_law's direct write: right before it, the angles about to be written; skipped = the write is
// withheld (the branch still runs); written = first statement of the branch.
void features_aim_direct_writing(float wy, float wp);
bool features_aim_direct_write_skipped();
void features_aim_direct_written(float yaw, float pitch);
// get_pose, the read: true = the pose comes from a feature's latched snapshot.
bool features_pose_latched(UEVR_TrackedDeviceIndex idx, bool use_aim, uevr::API::VR::Pose* out);

} // namespace halo
