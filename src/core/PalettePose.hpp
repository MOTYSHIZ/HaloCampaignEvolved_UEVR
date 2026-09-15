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
    bool (*stamp_available)();                                   // the hand intent is stamped per snapshot (the pose latch runs)
    bool (*stamped_intent)(bool two_back, float* yaw, float* pitch);   // the intent one or two snapshots back
    void (*mark)(int point, float yaw, float e0, float e1, float e2);  // the provider's STOMPLOG ring
    float (*roll_trim_deg)();                                    // the roll trim between grip rotation and weapon trim
    bool (*weapon_quat)(Quat* out);                              // the drawn weapon's world rotation as last published (raw)
};

bool palette_pose_owns_aim();
bool palette_pose_trim_rotations(float grip_q[4], float weapon_q[4]);
bool palette_pose_two_hand_blend(Vec3* fwd);
bool palette_pose_barrel_axis(Vec3* out);
bool palette_pose_stamp_available();
bool palette_pose_stamped_intent(bool two_back, float* yaw, float* pitch);
void palette_pose_mark(int point, float yaw, float e0, float e1, float e2);
float palette_pose_roll_trim_deg();   // 0 with no provider
bool palette_pose_weapon_quat(Quat* out);

} // namespace halo
