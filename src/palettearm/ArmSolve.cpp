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

// Recorded 2026-09-17 with pahandrec from the LEFT hand: open = the grenade throw's release, fist =
// the Magnum's off-hand punch. (x, y, z, w), parent-relative.
constexpr float kHandPoseOpen[5][4][4] = {   // index, middle, ring, pinky, thumb; joint 0 is relative to the WRIST
    {{0.075015f, 0.016572f, -0.153967f, 0.985085f}, {0.000000f, -0.045626f, 0.000000f, 0.998959f}, {0.000000f, 0.000000f, 0.000000f, 1.000000f}, {0.000000f, 0.000000f, 0.000000f, 1.000000f}},
    {{-0.022462f, 0.034487f, -0.041689f, 0.998283f}, {-0.004730f, 0.056643f, -0.083164f, 0.994914f}, {0.000000f, 0.000000f, 0.000092f, 1.000000f}, {0.000000f, 0.000000f, 0.000000f, 1.000000f}},
    {{-0.129555f, -0.041293f, -0.006470f, 0.990691f}, {0.002106f, 0.008393f, 0.244637f, 0.969576f}, {-0.000000f, 0.000000f, -0.055178f, 0.998477f}, {0.000000f, 0.000000f, 0.000000f, 1.000000f}},
    {{-0.171560f, 0.024386f, 0.343507f, 0.923025f}, {-0.000000f, 0.064578f, 0.000000f, 0.997913f}, {0.000000f, -0.000000f, 0.000397f, 1.000000f}, {0.000000f, 0.000000f, 0.000000f, 1.000000f}},
    {{0.508638f, -0.554387f, 0.021364f, 0.658397f}, {-0.015809f, -0.033632f, 0.063236f, 0.997306f}, {-0.012269f, 0.031587f, 0.284404f, 0.958105f}, {0.000000f, 0.000000f, 0.000000f, 1.000000f}},
};
constexpr float kHandPoseFist[5][4][4] = {   // index, middle, ring, pinky, thumb; joint 0 is relative to the WRIST
    {{-0.001221f, -0.028718f, 0.624083f, 0.780829f}, {-0.031007f, -0.033479f, 0.678559f, 0.733127f}, {0.000000f, 0.000000f, 0.663460f, 0.748212f}, {0.000000f, 0.000000f, 0.000000f, 1.000000f}},
    {{0.038302f, -0.034029f, 0.714581f, 0.697674f}, {0.038240f, 0.042086f, 0.671417f, 0.738895f}, {0.000000f, -0.000000f, 0.482743f, 0.875762f}, {0.000000f, 0.000000f, 0.000000f, 1.000000f}},
    {{-0.092383f, 0.058048f, 0.693067f, 0.712569f}, {0.006897f, 0.005249f, 0.795702f, 0.605627f}, {0.000000f, -0.000000f, 0.477532f, 0.878614f}, {0.000000f, 0.000000f, 0.000000f, 1.000000f}},
    {{-0.066658f, 0.102658f, 0.696254f, 0.707283f}, {0.044253f, 0.047031f, 0.683818f, 0.726789f}, {0.000000f, -0.000000f, 0.638589f, 0.769548f}, {0.000000f, 0.000000f, 0.000000f, 1.000000f}},
    {{0.390557f, -0.579899f, 0.080174f, 0.710461f}, {-0.022431f, -0.034395f, 0.162879f, 0.985791f}, {-0.021149f, 0.028138f, 0.519217f, 0.853917f}, {0.000000f, 0.000000f, 0.000000f, 1.000000f}},
};

// The RELAXED hand, recorded the same way: a frame of the weapon-draw animation where the left hand
// hangs free -- every finger gently curved (about 20 / 35 / 5 degrees down the joints), no splay, the
// thumb lying alongside. The grenade-release pose above is a hand at full stretch, and a rest pose
// blended from it toward the fist kept that stretch's splay: "more tense than I was expecting".
constexpr float kHandPoseRest[5][4][4] = {   // index, middle, ring, pinky, thumb; joint 0 is relative to the WRIST
    {{0.053774f, -0.011139f, 0.123295f, 0.990849f}, {-0.012482f, -0.043886f, 0.273359f, 0.960829f}, {0.000000f, -0.000000f, 0.056704f, 0.998391f}, {0.000000f, 0.000000f, 0.000000f, 1.000000f}},
    {{0.002167f, -0.037356f, 0.167307f, 0.985195f}, {0.015076f, 0.054812f, 0.264876f, 0.962605f}, {-0.000000f, -0.000000f, 0.000122f, 1.000000f}, {0.000000f, 0.000000f, 0.000000f, 1.000000f}},
    {{-0.122442f, 0.045046f, 0.139990f, 0.981520f}, {0.003266f, 0.008026f, 0.378679f, 0.925487f}, {-0.000000f, -0.000000f, -0.058871f, 0.998266f}, {0.000000f, 0.000000f, 0.000000f, 1.000000f}},
    {{-0.183623f, 0.000775f, 0.254334f, 0.949524f}, {0.023652f, 0.060061f, 0.365522f, 0.928562f}, {0.000000f, 0.000000f, -0.021089f, 0.999778f}, {0.000000f, 0.000000f, 0.000000f, 1.000000f}},
    {{0.413511f, -0.588510f, -0.112556f, 0.685563f}, {-0.020540f, -0.034212f, 0.134255f, 0.990143f}, {0.008271f, 0.037203f, 0.004822f, 0.999262f}, {0.000000f, 0.000000f, 0.000000f, 1.000000f}},
};

Quat slerp_short(const Quat& a, const Quat& b_in, float t) {
    Quat b = b_in;
    float d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (d < 0.0f) { b = Quat{-b.x, -b.y, -b.z, -b.w}; d = -d; }
    if (d > 0.9995f) {
        return normalized(Quat{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
                               a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t});
    }
    const float th = std::acos(std::clamp(d, -1.0f, 1.0f));
    const float s  = std::sin(th);
    const float wa = std::sin((1.0f - t) * th) / s, wb = std::sin(t * th) / s;
    return normalized(Quat{a.x * wa + b.x * wb, a.y * wa + b.y * wb,
                           a.z * wa + b.z * wb, a.w * wa + b.w * wb});
}

} // namespace

bool apply_hand_shape(BlamMatrix4x3* palette, const ArmNodes& arm, float curl, float authored) {
    if (palette == nullptr) return false;
    curl     = std::clamp(curl, -1.0f, 1.0f);
    authored = std::clamp(authored, 0.0f, 1.0f);
    if (authored >= 0.999f) return true;               // the game's own fingers, untouched

    const Mat3 wrist_basis = orthonormal_basis(palette[arm.wrist]);
    if (!valid_basis(wrist_basis)) return false;
    const Vec3 wrist_pos = palette[arm.wrist].position;

    const FingerChain* chains[5] = {&arm.index, &arm.middle, &arm.ring, &arm.pinky, &arm.thumb};
    for (std::size_t f = 0; f < 5; ++f) {
        const FingerChain& ch = *chains[f];
        // MEASURE the authored chain first -- every joint's rotation and offset in its parent's
        // frame -- because rebuilding joint j overwrites the frame joint j+1 was measured in.
        Mat3 stock_rel[4];
        Vec3 stock_off[4];
        Mat3 pb = wrist_basis;
        Vec3 pp = wrist_pos;
        for (std::size_t j = 0; j < ch.size(); ++j) {
            const Mat3 nb = orthonormal_basis(palette[ch[j]]);
            if (!valid_basis(nb)) return false;
            const Mat3 pinv = transpose(pb);
            stock_rel[j] = multiply(pinv, nb);
            stock_off[j] = transform_vector(pinv, palette[ch[j]].position - pp);
            pb = nb;
            pp = palette[ch[j]].position;
        }
        pb = wrist_basis;
        pp = wrist_pos;
        for (std::size_t j = 0; j < ch.size(); ++j) {
            const Quat qo{kHandPoseOpen[f][j][0], kHandPoseOpen[f][j][1],
                          kHandPoseOpen[f][j][2], kHandPoseOpen[f][j][3]};
            const Quat qf{kHandPoseFist[f][j][0], kHandPoseFist[f][j][1],
                          kHandPoseFist[f][j][2], kHandPoseFist[f][j][3]};
            const Quat qr{kHandPoseRest[f][j][0], kHandPoseRest[f][j][1],
                          kHandPoseRest[f][j][2], kHandPoseRest[f][j][3]};
            // Three key poses on one axis, the relaxed hand in the middle: a grip press travels
            // rest -> fist and never passes through the stretched hand on the way.
            Quat q = (curl >= 0.0f) ? slerp_short(qr, qf, curl) : slerp_short(qr, qo, -curl);
            if (authored > 0.001f) q = slerp_short(q, rotation_from_basis(stock_rel[j]), authored);
            const Mat3 rel = rotation_basis(q);
            if (!valid_basis(rel)) return false;
            const Mat3 nb = multiply(pb, rel);
            const Vec3 np = pp + transform_vector(pb, stock_off[j]);
            BlamMatrix4x3& m = palette[ch[j]];
            m.forward = nb.forward; m.left = nb.left; m.up = nb.up;
            m.position = np;
            pb = nb;
            pp = np;
        }
    }
    return true;
}

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

    float upper_length = length(elbow_position - shoulder_position);
    float lower_length = length(wrist_position - elbow_position);
    if (!std::isfinite(upper_length) || !std::isfinite(lower_length) ||
        upper_length < 1.0e-4f || lower_length < 1.0e-4f ||
        !finite(requested_wrist_position) || !valid_basis(desired_wrist_basis)) {
        return false;
    }

    Vec3  target_delta    = requested_wrist_position - shoulder_position;
    float target_distance = length(target_delta);
    if (!std::isfinite(target_distance) || target_distance < 1.0e-4f) return false;

    const Vec3 target_direction = target_delta / target_distance;

    // NOT DONE, DELIBERATELY -- "stretch, don't clamp". pancreations MCC VR (game.cpp:4638-4649)
    // scale BOTH bones by k = min(targetDist / reach, 1.8) and solve stretched, because a chain that
    // stops short makes "the hand visibly detach from the forearm" and skinning stretches the mesh
    // with the bones.
    //
    // Attempted 2026-08-31 and reverted: scaling only these two LENGTHS does nothing, because the
    // solve below applies a pure ROTATION (rotation_between + apply_rigid_delta) which cannot change
    // a bone's length. The elbow would still land at its unscaled position while `along` and
    // `height_squared` were computed from stretched numbers -- a worse pose than clamping, not a
    // better one. A real stretch has to move the bone POSITIONS along the chain.
    //
    // Left undone on purpose: with pa_target_frame=1 the target no longer flies out of reach (hand
    // excursion 102 cm -> 0.002 cm), so over-reach went from constant to rare, and a half-built
    // stretch is worse than an honest clamp.
    float minimum_reach = std::fabs(upper_length - lower_length) + 1.0e-4f;
    float maximum_reach = upper_length + lower_length - 1.0e-4f;
    float solve_upper = upper_length, solve_lower = lower_length, stretch_k = 1.0f;

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

    // STRETCH, DONE PROPERLY (2026-09-16): pancreations' rule, with the POSITION move the 2026-08-31
    // attempt lacked. Whatever the clavicle assist could not absorb scales BOTH solve lengths by
    // k <= stretch_max, and after the shoulder rotation below the elbow subtree is TRANSLATED onto
    // the stretched elbow, so the forearm's end lands on the hand instead of short of it.
    if (tuning.stretch_max > 1.0f && target_distance > maximum_reach) {
        stretch_k     = std::min(target_distance / (upper_length + lower_length), tuning.stretch_max);
        solve_upper   = upper_length * stretch_k;
        solve_lower   = lower_length * stretch_k;
        minimum_reach = std::fabs(solve_upper - solve_lower) + 1.0e-4f;
        maximum_reach = solve_upper + solve_lower - 1.0e-4f;
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
    const float along = (target_distance * target_distance + solve_upper * solve_upper -
                         solve_lower * solve_lower) / (2.0f * target_distance);
    const float height_squared = std::max(solve_upper * solve_upper - along * along, 0.0f);
    const Vec3  elbow_target =
        shoulder_position + target_direction * along + pole * std::sqrt(height_squared);

    const Mat3 shoulder_rotation =
        rotation_between(elbow_position - shoulder_position, elbow_target - shoulder_position);
    if (!valid_basis(shoulder_rotation)) return false;
    apply_rigid_delta(palette, arm.shoulder_subtree, arm.shoulder_count,
                      shoulder_rotation, shoulder_position);
    if (stretch_k > 1.0f) {
        // The rotation above put the elbow at its AUTHORED length along the solved direction;
        // slide everything from the elbow down onto the stretched elbow.
        const Vec3 stretch_shift = elbow_target - palette[arm.elbow].position;
        apply_rigid_offset(palette, arm.elbow_subtree, arm.elbow_count, stretch_shift);
    }

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

// ---- FOREARM TWIST ------------------------------------------------------------------------------

namespace {

float wrap_degrees(float d) {
    if (!std::isfinite(d)) return 0.0f;
    d = std::fmod(d + 180.0f, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return d - 180.0f;
}

// The signed angle, about `axis`, from `u` to `v` once both are flattened against it. False when
// either has too little left after flattening for the angle to mean anything.
bool signed_angle_about(const Vec3& u, const Vec3& v, const Vec3& axis, float& out_deg) {
    const Vec3 uf = u - axis * dot(u, axis);
    const Vec3 vf = v - axis * dot(v, axis);
    if (length_squared(uf) < 0.04f * length_squared(u) || length_squared(vf) < 0.04f * length_squared(v))
        return false;
    const Vec3 un = normalized(uf), vn = normalized(vf);
    if (length_squared(un) < 0.8f || length_squared(vn) < 0.8f) return false;
    out_deg = std::atan2(dot(cross(un, vn), axis), dot(un, vn)) * 57.2957795131f;
    return std::isfinite(out_deg);
}

// How much of the hand's roll a bone at fraction `t` of the way from the elbow to the wrist takes.
// MEASURED off this rig's own animations (5149 recorded frames, both arms): the bone a third of
// the way down carries 0.308 of the hand's twist and the one two thirds down carries 0.718-0.720,
// with a worst-case fit error of 2.2 degrees on the right arm and 5.0 on the left. Piecewise-linear
// through those points, so a derived node map with differently placed twist bones still gets a
// sensible share.
float twist_share(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    constexpr float kT[4] = {0.0f, 0.3333f, 0.6667f, 1.0f};
    constexpr float kS[4] = {0.0f, 0.308f,  0.719f,  1.0f};
    for (int i = 0; i < 3; ++i) {
        if (t <= kT[i + 1]) return kS[i] + (kS[i + 1] - kS[i]) * (t - kT[i]) / (kT[i + 1] - kT[i]);
    }
    return 1.0f;
}

// Roll the forearm follows 1:1 out to `kTwistFull` degrees either side of the thumb-up neutral,
// then hands back by the time the hand is thumb-DOWN -- the one roll no forearm reaches. Periodic
// and continuous, so the 180-degree seam of the twist angle lands where this is already zero.
constexpr float kTwistFull = 135.0f;
float twist_follow(float from_neutral_deg) {
    const float a = std::fabs(from_neutral_deg);
    if (a <= kTwistFull) return from_neutral_deg;
    const float back = kTwistFull * (180.0f - a) / (180.0f - kTwistFull);
    return from_neutral_deg < 0.0f ? -back : back;
}

bool in_list(const std::uint8_t* list, std::size_t count, std::uint8_t node) {
    for (std::size_t i = 0; i < count; ++i) if (list[i] == node) return true;
    return false;
}

} // namespace

ForearmStock capture_forearm_stock(const BlamMatrix4x3* palette, const ArmNodes& arm) {
    ForearmStock s{};
    if (palette == nullptr) return s;
    s.elbow_basis = orthonormal_basis(palette[arm.elbow]);
    s.wrist_basis = orthonormal_basis(palette[arm.wrist]);
    const Vec3 along = normalized(palette[arm.wrist].position - palette[arm.elbow].position);
    if (!valid_basis(s.elbow_basis) || !valid_basis(s.wrist_basis) || length_squared(along) < 0.8f)
        return s;
    s.axis_local = transform_vector(transpose(s.elbow_basis), along);
    s.valid = true;
    return s;
}

bool distribute_forearm_twist(BlamMatrix4x3* palette, const ArmNodes& arm, const ForearmStock& stock,
                              const Vec3& thumb_up_hint, float gain, float plate_gain,
                              ForearmTwistResult* result) {
    if (result != nullptr) *result = ForearmTwistResult{};
    if (palette == nullptr || !stock.valid || !std::isfinite(gain) || !std::isfinite(plate_gain)) return false;
    gain       = std::clamp(gain, 0.0f, 2.0f);
    plate_gain = std::clamp(plate_gain, 0.0f, 3.0f);

    const Mat3 elbow_now = orthonormal_basis(palette[arm.elbow]);
    const Mat3 wrist_now = orthonormal_basis(palette[arm.wrist]);
    if (!valid_basis(elbow_now) || !valid_basis(wrist_now)) return false;

    // The forearm's own long axis, carried by the elbow -- NOT elbow-to-wrist as drawn. The twist
    // bones sit on the first by construction; the second bends away from it whenever the reach
    // clamp or the stretch lets the placed wrist leave the solved one.
    const Vec3 axis = normalized(transform_vector(elbow_now, stock.axis_local));
    if (length_squared(axis) < 0.8f) return false;

    // Where the hand WOULD be had it kept its authored relation to this forearm, and the rotation
    // from there to where it is. Its twist about the forearm is the roll the solve added.
    const Mat3 wrist_ref = multiply(elbow_now, multiply(transpose(stock.elbow_basis), stock.wrist_basis));
    const Mat3 added     = multiply(wrist_now, transpose(wrist_ref));
    if (!valid_basis(wrist_ref) || !valid_basis(added)) return false;
    Quat q = rotation_from_basis(added);
    if (q.w < 0.0f) q = Quat{-q.x, -q.y, -q.z, -q.w};
    const float hand_deg =
        2.0f * std::atan2(q.x * axis.x + q.y * axis.y + q.z * axis.z, q.w) * 57.2957795131f;

    // ...measured from the AUTHORED roll, but followed from the THUMB-UP NEUTRAL. The authored
    // support hand is a palm-up hold about 100 degrees of supination from neutral (measured on the
    // Assault Rifle), so a free hand turned palm-DOWN is ~190 degrees from it -- past the seam of
    // any twist angle, where a forearm that simply followed would snap a third of a turn. Putting
    // the seam at thumb-down instead moves it to a pose no arm makes, and twist_follow() is already
    // zero there. Subtracting the authored pose's own term keeps "authored hand = no added twist"
    // exact wherever that pose sits.
    float neutral_deg = 0.0f;
    {
        const Vec3 radial_now = palette[arm.index[0]].position - palette[arm.pinky[0]].position;
        const Vec3 radial_ref = transform_vector(transpose(added), radial_now);
        float c = 0.0f;
        if (length_squared(radial_now) > 1.0e-10f && signed_angle_about(radial_ref, thumb_up_hint, axis, c))
            neutral_deg = c;
    }
    const float follow_deg = twist_follow(wrap_degrees(hand_deg - neutral_deg)) -
                             twist_follow(wrap_degrees(-neutral_deg));

    if (result != nullptr) {
        result->hand_deg    = wrap_degrees(hand_deg);
        result->neutral_deg = neutral_deg;
        result->follow_deg  = follow_deg;
    }
    if (gain <= 0.0f || std::fabs(follow_deg) < 0.01f) return true;

    const Vec3  elbow_pos = palette[arm.elbow].position;
    const float fore_len  = length(palette[arm.wrist].position - elbow_pos);
    if (!std::isfinite(fore_len) || fore_len < 1.0e-4f) return false;

    int turned = 0, plates = 0;
    for (std::size_t i = 0; i < arm.elbow_count; ++i) {
        const std::uint8_t node = arm.elbow_subtree[i];
        if (node == arm.elbow || in_list(arm.wrist_subtree, arm.wrist_count, node)) continue;
        // A twist bone lies ON the forearm's axis, between the joints. Everything else hanging off
        // the elbow is ARMOUR (the gauntlet plates, 10-13 cm out on this rig). The game carries those
        // rigidly with the elbow and never rolls them -- which reads fine under its own animations,
        // where the forearm seldom rolls far, and wrong under a tracked hand: the sleeve turns
        // inside a plate that does not ("make the forearm armor piece follow the rotation of the
        // forearm"). So a plate takes the roll of the forearm AT ITS OWN STATION along the bone --
        // the gauntlet sits at t = 0.33, beside the near twist bone, and turns with it -- orbiting
        // the axis as it goes, times plate_gain.
        const Vec3  rel    = palette[node].position - elbow_pos;
        const float t      = dot(rel, axis) / fore_len;
        const float radial = length(rel - axis * dot(rel, axis)) * kMetresPerBlamUnit;
        const bool  bone   = (t > 0.08f && t < 0.95f) && (radial < 0.015f);
        const bool  plate  = !bone && (t > 0.0f && t < 1.0f) && (radial >= 0.015f) && plate_gain > 0.0f;
        if (!bone && !plate) continue;

        const float half = gain * (plate ? plate_gain : 1.0f) * twist_share(t) * follow_deg *
                           0.00872664626f;                                        // half angle, rad
        const float s    = std::sin(half);
        const Mat3  roll = rotation_basis(Quat{axis.x * s, axis.y * s, axis.z * s, std::cos(half)});
        if (!valid_basis(roll)) continue;
        apply_rigid_delta(palette, &node, 1, roll, elbow_pos);
        if (plate) ++plates; else ++turned;
    }
    if (result != nullptr) { result->nodes = turned; result->plates = plates; }
    return true;
}

// ---- RECOIL PASS-THROUGH ------------------------------------------------------------------------

void RecoilPass::reset() { *this = RecoilPass{}; }

Vec3 RecoilPass::update(const Vec3& marker_pos, const Mat3& marker_basis, float gain, float max_m,
                        bool learn) {
    last_back_m = 0.0f;
    if (!finite(marker_pos) || !valid_basis(marker_basis)) { if (learn) stable = 0; return {}; }

    if (learn) {
        // "Holding still" is judged frame to frame, so a slow idle sway still counts as rest and the
        // reference follows it; a burst never does -- the Assault Rifle's kick cycle moves the
        // marker 0.6-2 cm on four frames of every six.
        bool still = false;
        if (have_prev) {
            const float step_m = length(marker_pos - prev_pos) * kMetresPerBlamUnit;
            float align = dot(prev_basis.forward, marker_basis.forward);
            align = std::min(align, dot(prev_basis.left, marker_basis.left));
            align = std::min(align, dot(prev_basis.up, marker_basis.up));
            still = step_m < 0.0006f && align > 0.999986f;          // 0.6 mm, 0.3 degrees
        }
        prev_pos = marker_pos; prev_basis = marker_basis; have_prev = true;
        stable = still ? std::min(stable + 1, 1000000) : 0;
        if (stable >= 20) {
            if (!have_ref) { ref_pos = marker_pos; ref_basis = marker_basis; have_ref = true; ++latches; }
            else {
                ref_pos   = ref_pos + (marker_pos - ref_pos) * 0.2f;
                ref_basis = blend_basis(ref_basis, marker_basis, 0.2f);
                if (!valid_basis(ref_basis)) ref_basis = marker_basis;
            }
        }
    }
    if (!have_ref || !(gain > 0.0f) || !(max_m > 0.0f) || !std::isfinite(gain) || !std::isfinite(max_m))
        return {};

    // BACK ALONG THE BARREL, and nothing else. The authored marker carries the gun's long axis as
    // its LEFT axis (measured on the Assault Rifle and the Magnum: left = +X, the view's forward),
    // so "back" is its negative; a marker authored some other way falls back to the view's own
    // backward, which is where every first-person gun points to within a few degrees.
    Vec3 back_axis = ref_basis.left * -1.0f;
    if (back_axis.x > -0.7f) back_axis = Vec3{-1.0f, 0.0f, 0.0f};

    const Vec3  moved  = marker_pos - ref_pos;
    const float back   = dot(moved, back_axis);
    if (!(back > 0.0f)) return {};

    // A KICK IS A TRANSLATION. Every other animation that moves the marker turns it as well --
    // measured: the rifle's burst stays within 0.64 degrees of rest, while the draw, reload, melee,
    // grenade and swap animations turn it 24 to 177 degrees -- and travels several times further.
    // Both fades are smooth so an animation passing through the band eases the gun rather than
    // popping it.
    float align = dot(ref_basis.forward, marker_basis.forward);
    align = std::min(align, dot(ref_basis.left, marker_basis.left));
    align = std::min(align, dot(ref_basis.up, marker_basis.up));
    const float turned_deg = std::acos(std::clamp(align, -1.0f, 1.0f)) * 57.2957795131f;
    const float weight = (1.0f - smoothstep(8.0f, 20.0f, turned_deg)) *
                         (1.0f - smoothstep(max_m, 2.0f * max_m, length(moved) * kMetresPerBlamUnit));
    const float out_m = std::min(back * kMetresPerBlamUnit, max_m) * weight * std::min(gain, 2.0f);
    if (!(out_m > 0.0f) || !std::isfinite(out_m)) return {};
    last_back_m = out_m;
    return back_axis * (out_m / kMetresPerBlamUnit);
}

} // namespace halo::palettearm
