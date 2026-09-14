#pragma once

// HMD POSE PLAUSIBILITY GATE, for the HMD translation leash block (game thread).

#include "Math.hpp"

namespace halo {

// One dropped tracking frame (measured: hp jumped 5.6 m for ONE 18 ms tick, then back) went straight
// into the leash, which yanked the standing origin 4.6 m to clamp it; the pose recovered and
// roomscale sprinted the biped a metre to walk out the phantom offset. A head cannot move faster
// than ~5 m/s, so a pose that claims it is a dropout: false (logged 1 in 16), and the reference is
// NOT advanced, so the next good pose is judged against the last good one. After 0.5 s without a
// good pose anything is accepted (a real recenter or teleport must be able to win).
bool hmd_pose_plausible(const Vec3& hp);

} // namespace halo
