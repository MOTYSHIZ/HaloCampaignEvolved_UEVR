#pragma once

// THE PALETTE WEAPON'S PUBLISHED POSE, a core interface. A feature that places the weapon on the Blam palette
// provides it (FeatureHooks::palette_pose); consumers (aimbore, the aim derivation's hook) read it here and never
// the provider's own state. With no provider in the build every call returns false.

#include "Math.hpp"

namespace halo {

struct PalettePoseProvider {
    bool (*owns_aim)();                                          // the provider owns placement and aim right now
    bool (*trim_rotations)(float grip_q[4], float weapon_q[4]);  // the drawn weapon's rotations on the aim-fixed hand
    bool (*two_hand_blend)(Vec3* fwd);                           // the two-handed hold's blend of a forward
    bool (*barrel_axis)(Vec3* out);                              // the measured barrel axis in the trimmed pose frame
};

bool palette_pose_owns_aim();
bool palette_pose_trim_rotations(float grip_q[4], float weapon_q[4]);
bool palette_pose_two_hand_blend(Vec3* fwd);
bool palette_pose_barrel_axis(Vec3* out);

} // namespace halo
