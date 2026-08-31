#include "ArmSolve.hpp"

#include <algorithm>

namespace halo::palettearm {
namespace {

// Turn `rotation` down to `scale` of its angle. Used to rotate a finger joint PART of the way onto
// the open line: the open pose is derived, so the in-between poses have to be too.
Quat scaled_rotation(const Quat& rotation, float scale) {
    const Quat q = normalized(rotation);
    scale = std::clamp(scale, 0.0f, 1.0f);
    // Sign-normalise to the short way round. Without this a q with negative w scales toward the
    // 360-degree rotation instead of toward identity, and a finger snaps the wrong way.
    const float w = q.w < 0.0f ? -q.w : q.w;
    const Vec3  v = q.w < 0.0f ? Vec3{-q.x, -q.y, -q.z} : Vec3{q.x, q.y, q.z};

    const float sin_half = length(v);
    if (!std::isfinite(sin_half) || sin_half < 1.0e-6f) return {};   // identity: nothing to scale
    const float half   = std::atan2(sin_half, std::clamp(w, -1.0f, 1.0f));
    const Vec3  axis   = v / sin_half;
    const float scaled = half * scale;
    const float s      = std::sin(scaled);
    return {axis.x * s, axis.y * s, axis.z * s, std::cos(scaled)};
}

Quat quaternion_between(const Vec3& source, const Vec3& destination) {
    const Vec3 from = normalized(source);
    const Vec3 to   = normalized(destination);
    if (length_squared(from) < 0.8f || length_squared(to) < 0.8f) return {};

    const float alignment = std::clamp(dot(from, to), -1.0f, 1.0f);
    if (alignment > 0.9999f) return {};
    if (alignment < -0.9999f) {
        Vec3 axis = cross(from, {1.0f, 0.0f, 0.0f});
        if (length_squared(axis) < 1.0e-6f) axis = cross(from, {0.0f, 1.0f, 0.0f});
        axis = normalized(axis);
        return {axis.x, axis.y, axis.z, 0.0f};
    }
    const Vec3 axis = cross(from, to);
    return normalized(Quat{axis.x, axis.y, axis.z, 1.0f + alignment});
}

// Rotate finger chain nodes from `joint` outward about the joint's current position.
bool rotate_finger_chain_about_joint(BlamMatrix4x3* palette, const FingerChain& chain,
                                     std::size_t joint, const Quat& rotation) {
    if (palette == nullptr || joint >= chain.size()) return false;
    const Mat3 basis = rotation_basis(rotation);
    if (!valid_basis(basis)) return false;

    const Vec3 pivot = palette[chain[joint]].position;
    for (std::size_t i = joint; i < chain.size(); ++i) {
        BlamMatrix4x3& m = palette[chain[i]];
        m.forward  = normalized(transform_vector(basis, m.forward));
        m.left     = normalized(transform_vector(basis, m.left));
        m.up       = normalized(transform_vector(basis, m.up));
        m.position = pivot + transform_vector(basis, m.position - pivot);
    }
    return true;
}

bool apply_finger_openness(BlamMatrix4x3* palette, std::uint8_t wrist,
                           const FingerChain& chain, float curl) {
    if (palette == nullptr) return false;

    const float open_amount = 1.0f - std::clamp(curl, 0.0f, 1.0f);
    if (open_amount <= 0.0f) return true;         // already the authored closed grip

    // The open direction: away from the wrist, through the digit's first knuckle. Derived from the
    // rig's own proportions rather than authored, so it holds for either hand and any weapon.
    const Vec3 wrist_position = palette[wrist].position;
    const Vec3 extension = normalized(palette[chain.front()].position - wrist_position);
    if (length_squared(extension) < 0.8f) return false;

    for (std::size_t joint = 0; joint + 1 < chain.size(); ++joint) {
        const Vec3 current =
            normalized(palette[chain[joint + 1]].position - palette[chain[joint]].position);
        if (length_squared(current) < 0.8f) return false;

        const Quat rotation = scaled_rotation(quaternion_between(current, extension), open_amount);
        // A near-identity rotation is a joint already on the open line -- not a failure.
        if (std::fabs(rotation.w) > 0.9999f) continue;
        if (!rotate_finger_chain_about_joint(palette, chain, joint, rotation)) return false;
    }
    return true;
}

} // namespace

void apply_rigid_delta(BlamMatrix4x3* palette, const std::uint8_t* nodes, std::size_t count,
                       const Mat3& rotation, const Vec3& pivot) {
    if (palette == nullptr || nodes == nullptr) return;
    for (std::size_t i = 0; i < count; ++i) {
        BlamMatrix4x3& m = palette[nodes[i]];
        m.forward  = normalized(transform_vector(rotation, m.forward));
        m.left     = normalized(transform_vector(rotation, m.left));
        m.up       = normalized(transform_vector(rotation, m.up));
        m.position = pivot + transform_vector(rotation, m.position - pivot);
    }
}

void apply_rigid_offset(BlamMatrix4x3* palette, const std::uint8_t* nodes, std::size_t count,
                        const Vec3& offset) {
    if (palette == nullptr || nodes == nullptr) return;
    for (std::size_t i = 0; i < count; ++i) {
        palette[nodes[i]].position = palette[nodes[i]].position + offset;
    }
}

void apply_rigid_transform(BlamMatrix4x3* palette, const std::uint8_t* nodes, std::size_t count,
                           const Mat3& basis, const Vec3& translation) {
    if (palette == nullptr || nodes == nullptr) return;
    for (std::size_t i = 0; i < count; ++i) {
        BlamMatrix4x3& m = palette[nodes[i]];
        m.forward  = normalized(transform_vector(basis, m.forward));
        m.left     = normalized(transform_vector(basis, m.left));
        m.up       = normalized(transform_vector(basis, m.up));
        // Absolute, not relative to a pivot: the caller has already folded the source pose into
        // `translation`, so this is a plain affine placement.
        m.position = translation + transform_vector(basis, m.position);
    }
}

Mat3 torso_basis_from_root(const Mat3& root_basis) {
    // Up is WORLD up (+Z in Blam), not the root's own up axis. That is the whole point: a torso
    // does not roll with the camera, and taking vertical from the root would let head roll tilt
    // the shoulders -- which is the "arms swing across your face" failure this frame exists to
    // avoid. Only the root's YAW survives, by flattening its forward against world up.
    constexpr Vec3 up{0.0f, 0.0f, 1.0f};
    Vec3 forward{root_basis.forward.x, root_basis.forward.y, 0.0f};
    if (length_squared(forward) < 1.0e-6f) {
        // Looking straight up or down: forward carries no yaw at all, so take it from the up axis,
        // which is horizontal in exactly that case. The sign keeps the turn continuous through the
        // pole instead of flipping the player around.
        const float sign = root_basis.forward.z <= 0.0f ? 1.0f : -1.0f;
        forward = {root_basis.up.x * sign, root_basis.up.y * sign, 0.0f};
    }
    forward = normalized(forward);
    if (length_squared(forward) < 0.8f) return {};
    return Mat3{forward, cross(up, forward), up};
}

bool anchor_shoulder_to_torso(BlamMatrix4x3* palette, const ArmNodes& arm, const Mat3& torso_basis,
                              const Vec3& head_position, bool left_side, const ArmTuning& tuning) {
    if (palette == nullptr || !valid_basis(torso_basis) || !finite(head_position)) return false;

    // Blam local axes: +X forward, +Y left, +Z up. Back is -X, down is -Z, and the lateral term
    // flips sign by side.
    const Vec3 local_offset{
        -tuning.shoulder_back_m / kMetresPerBlamUnit,
        (left_side ? tuning.shoulder_lateral_m : -tuning.shoulder_lateral_m) / kMetresPerBlamUnit,
        -tuning.shoulder_down_m / kMetresPerBlamUnit};

    const Vec3 anchor = head_position + transform_vector(torso_basis, local_offset);
    const Vec3 shift  = anchor - palette[arm.shoulder].position;
    if (!finite(shift)) return false;

    apply_rigid_offset(palette, arm.shoulder_subtree, arm.shoulder_count, shift);
    return true;
}

Vec3 wrist_from_grip(const Vec3& grip_position, const Mat3& grip_basis, const ArmTuning& tuning) {
    const Vec3 local{-tuning.grip_to_wrist_back_m / kMetresPerBlamUnit,
                     0.0f,
                     -tuning.grip_to_wrist_down_m / kMetresPerBlamUnit};
    return grip_position + transform_vector(grip_basis, local);
}

bool place_wrist_subtree(BlamMatrix4x3* palette, const ArmNodes& arm,
                         const Vec3& wrist_position, const Mat3& wrist_basis) {
    if (palette == nullptr || !finite(wrist_position) || !valid_basis(wrist_basis)) return false;

    const Vec3 source_position = palette[arm.wrist].position;
    const Mat3 source_basis    = orthonormal_basis(palette[arm.wrist]);
    if (!valid_basis(source_basis)) return false;

    const Mat3 delta = multiply(wrist_basis, transpose(source_basis));
    if (!valid_basis(delta)) return false;

    for (std::size_t i = 0; i < arm.wrist_count; ++i) {
        BlamMatrix4x3& m = palette[arm.wrist_subtree[i]];
        m.forward  = normalized(transform_vector(delta, m.forward));
        m.left     = normalized(transform_vector(delta, m.left));
        m.up       = normalized(transform_vector(delta, m.up));
        m.position = wrist_position + transform_vector(delta, m.position - source_position);
    }
    // The wrist must land where it was asked to, to well under a millimetre. If it did not, the
    // delta was not a rigid transform and the hand is somewhere unintended.
    return length_squared(palette[arm.wrist].position - wrist_position) < 1.0e-6f;
}

// How much perpendicular the current elbow must have, as a fraction of the upper-arm length, before
// its direction is trusted as the pole. Below this the direction is noise -- see the pop analysis in
// solve_two_bone_arm. 0.25 = the elbow must sit at least a quarter of an upper-arm off the
// shoulder-to-wrist line; below that we fade to the caller stable fallback (the torso up axis).
constexpr float kPoleWellConditioned = 0.25f;

// Small enough to be invisible, non-zero so nothing downstream divides by it or treats the node as
// degenerate. pancreations use exactly this value.
constexpr float kHiddenScale = 0.0001f;

bool solve_two_bone_arm(BlamMatrix4x3* palette, const ArmNodes& arm,
                        const Vec3& requested_wrist_position, const Mat3& desired_wrist_basis,
                        const Vec3& fallback_pole, const ArmTuning& tuning) {
    if (palette == nullptr) return false;

    Vec3 shoulder_position = palette[arm.shoulder].position;
    Vec3 elbow_position    = palette[arm.elbow].position;
    const Vec3 wrist_position = palette[arm.wrist].position;

    const float upper_length = length(elbow_position - shoulder_position);
    const float lower_length = length(wrist_position - elbow_position);
    if (!std::isfinite(upper_length) || !std::isfinite(lower_length) ||
        upper_length < 1.0e-4f || lower_length < 1.0e-4f ||
        !finite(requested_wrist_position) || !valid_basis(desired_wrist_basis)) {
        return false;
    }

    Vec3  target_delta    = requested_wrist_position - shoulder_position;
    float target_distance = length(target_delta);
    if (!std::isfinite(target_distance) || target_distance < 1.0e-4f) return false;

    const Vec3  target_direction = target_delta / target_distance;
    const float minimum_reach    = std::fabs(upper_length - lower_length) + 1.0e-4f;
    const float maximum_reach    = upper_length + lower_length - 1.0e-4f;

    // Clavicle assist: rather than stopping the hand at the reach sphere, slide the whole arm root
    // toward an out-of-reach target the way a real shoulder rolls into an overreach. Whatever is
    // still out of reach after that gets clamped below, and the caller's exact wrist placement
    // absorbs the shortfall.
    const float overshoot = target_distance - maximum_reach;
    if (overshoot > 0.0f) {
        const float assist = std::min(overshoot, tuning.clavicle_assist_m / kMetresPerBlamUnit);
        const Vec3  offset = target_direction * assist;
        apply_rigid_offset(palette, arm.shoulder_subtree, arm.shoulder_count, offset);
        shoulder_position = shoulder_position + offset;
        elbow_position    = elbow_position + offset;
        target_distance  -= assist;
    }

    target_distance = std::clamp(target_distance, minimum_reach, maximum_reach);
    const Vec3 wrist_target = shoulder_position + target_direction * target_distance;

    // The pole decides which way the elbow bends. Prefer where the elbow already is (so the stock
    // animation bend is preserved), projected off the shoulder-to-wrist line.
    //
    // THE POLE GOES ILL-CONDITIONED BEFORE IT GOES ZERO. As the current arm straightens along the
    // target direction the perpendicular component vanishes, and long before it reaches zero its
    // DIRECTION is already pure numerical noise. The elbow is then placed at an arbitrary azimuth
    // at radius sqrt(height_squared) -- which is large whenever the TARGET still wants a bend. That
    // is a single-frame elbow pop of tens of centimetres with a motionless hand.
    //
    // Measured 2026-08-30 at hook rate with the controller static (ctrl 0.00/0.00 cm): elbow max
    // 31.7 / 32.0 / 41.5 cm in one frame while the wrist target moved under 1.3 cm and mean elbow
    // motion was 0.4-0.7 cm. Reported in-headset as constant jitter in the bones between the hand
    // and the shoulder.
    //
    // The old guard was `length_squared(pole) < 1e-6`, which only catches EXACTLY zero -- bone
    // lengths here are ~0.2 units, so a squared perpendicular of 1e-5 is already noise and sailed
    // straight through. Condition on the perpendicular RELATIVE to the bone, and BLEND to the
    // fallback across the ill-conditioned band: a hard switch at any threshold is itself a pop.
    Vec3 pole = elbow_position - shoulder_position;
    pole = pole - target_direction * dot(pole, target_direction);

    Vec3 fallback = fallback_pole - target_direction * dot(fallback_pole, target_direction);
    if (length_squared(fallback) < 1.0e-6f) fallback = cross(target_direction, {0.0f, 0.0f, 1.0f});
    if (length_squared(fallback) < 1.0e-6f) fallback = cross(target_direction, {0.0f, 1.0f, 0.0f});

    const float perp      = length(pole);
    const float condition = (upper_length > 1.0e-4f) ? (perp / upper_length) : 0.0f;
    const Vec3  fb        = normalized(fallback);
    Vec3        own       = (perp > 1.0e-6f) ? (pole / perp) : fb;

    // Ill-conditioned: fade the authored contribution out entirely (see the pop analysis above).
    if (condition < kPoleWellConditioned) {
        const float t = (kPoleWellConditioned > 0.0f) ? (condition / kPoleWellConditioned) : 0.0f;
        own = fb * (1.0f - t) + own * t;
    }

    // Then anchor the pole mostly to the BODY so its azimuth does not ride the camera. See
    // ArmTuning::pole_body_fraction.
    const float bodyw = (tuning.pole_body_fraction < 0.0f) ? 0.0f
                      : ((tuning.pole_body_fraction > 1.0f) ? 1.0f : tuning.pole_body_fraction);
    pole = fb * bodyw + own * (1.0f - bodyw);
    pole = normalized(pole);
    if (length_squared(pole) < 0.8f) return false;

    // Law of cosines: distance along the target line to the elbow's projection, and its height
    // off that line.
    const float along = (target_distance * target_distance + upper_length * upper_length -
                         lower_length * lower_length) / (2.0f * target_distance);
    const float height_squared = std::max(upper_length * upper_length - along * along, 0.0f);
    const Vec3  elbow_target =
        shoulder_position + target_direction * along + pole * std::sqrt(height_squared);

    const Mat3 shoulder_rotation =
        rotation_between(elbow_position - shoulder_position, elbow_target - shoulder_position);
    if (!valid_basis(shoulder_rotation)) return false;
    apply_rigid_delta(palette, arm.shoulder_subtree, arm.shoulder_count,
                      shoulder_rotation, shoulder_position);

    const Vec3 moved_elbow = palette[arm.elbow].position;
    const Vec3 moved_wrist = palette[arm.wrist].position;
    const Mat3 elbow_rotation =
        rotation_between(moved_wrist - moved_elbow, wrist_target - moved_elbow);
    if (!valid_basis(elbow_rotation)) return false;
    apply_rigid_delta(palette, arm.elbow_subtree, arm.elbow_count, elbow_rotation, moved_elbow);

    const Vec3 final_wrist_position = palette[arm.wrist].position;
    const Mat3 current_wrist_basis  = orthonormal_basis(palette[arm.wrist]);
    if (!valid_basis(current_wrist_basis)) return false;
    const Mat3 wrist_rotation = multiply(desired_wrist_basis, transpose(current_wrist_basis));
    if (!valid_basis(wrist_rotation)) return false;
    apply_rigid_delta(palette, arm.wrist_subtree, arm.wrist_count,
                      wrist_rotation, final_wrist_position);
    return true;
}

bool collapse_to_hands(BlamMatrix4x3* palette, std::size_t node_count,
                       const ArmNodes& left, const ArmNodes& right,
                       const std::uint8_t* keep_extra, std::size_t keep_extra_count) {
    if (palette == nullptr || node_count == 0 || node_count > kMaxPaletteNodes) return false;
    if (left.wrist_subtree == nullptr || left.wrist_count == 0) return false;
    if (right.wrist_subtree == nullptr || right.wrist_count == 0) return false;
    if (keep_extra == nullptr && keep_extra_count != 0) return false;

    bool keep[kMaxPaletteNodes] = {false};
    auto mark = [&](const std::uint8_t* list, std::size_t n) {
        for (std::size_t i = 0; i < n; ++i) {
            if (list[i] < node_count) keep[list[i]] = true;
        }
    };
    mark(left.wrist_subtree,  left.wrist_count);
    mark(right.wrist_subtree, right.wrist_count);
    mark(keep_extra,          keep_extra_count);

    // Node 0 is the root everything is expressed in -- never written, always kept.
    for (std::size_t i = 1; i < node_count; ++i) {
        if (!keep[i]) palette[i].scale = kHiddenScale;
    }
    return true;
}

bool solve_arm_for_tracked_wrist(BlamMatrix4x3* palette, const ArmNodes& arm,
                                 const Vec3& wrist_position, const Mat3& wrist_basis,
                                 const Vec3& fallback_pole, const ArmTuning& tuning) {
    if (solve_two_bone_arm(palette, arm, wrist_position, wrist_basis, fallback_pole, tuning)) {
        // The IK clamps at the reach sphere, so the wrist can end up short of where it was asked
        // for. Snap the hand the rest of the way: a hand that tracks with a slightly wrong forearm
        // beats a hand that lags behind the controller.
        return place_wrist_subtree(palette, arm, wrist_position, wrist_basis);
    }
    // No solve. Place the hand alone rather than leaving it in the stock pose -- a tracked hand
    // with a wrong arm is still a tracked hand.
    return place_wrist_subtree(palette, arm, wrist_position, wrist_basis);
}

bool apply_hand_openness(BlamMatrix4x3* palette, const ArmNodes& arm, const HandCurl& curl) {
    if (palette == nullptr) return false;
    return apply_finger_openness(palette, arm.wrist, arm.index,  curl.index) &&
           apply_finger_openness(palette, arm.wrist, arm.middle, curl.grip)  &&
           apply_finger_openness(palette, arm.wrist, arm.ring,   curl.grip)  &&
           apply_finger_openness(palette, arm.wrist, arm.pinky,  curl.grip)  &&
           apply_finger_openness(palette, arm.wrist, arm.thumb,  curl.thumb);
}

} // namespace halo::palettearm
