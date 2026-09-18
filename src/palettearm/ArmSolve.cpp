#include "ArmSolve.hpp"

#include <algorithm>

namespace halo::palettearm {
namespace {

bool in_list(const std::uint8_t* list, std::size_t count, std::uint8_t node) {
    for (std::size_t i = 0; i < count; ++i) if (list[i] == node) return true;
    return false;
}

// STRETCH A BONE, NOT A JOINT. Slides every node that hangs BETWEEN two joints out along the bone
// in proportion to its station: a node a third of the way down a bone stretched by k moves a third
// of the extra length. `nodes` is the upper joint's subtree, `below` the lower joint's (left alone
// here -- the caller moves it as one piece), `joint` the upper joint itself.
//
// Without this a stretched bone keeps every helper node at its authored distance and opens the
// whole extension as ONE gap in front of the lower joint, which is exactly the "the hand stretches
// from the wrist" look one joint further up. This rig has helper bones at 1/3 and 2/3 of both the
// upper arm and the forearm (measured), so the skin is asked to stretch evenly along each.
void spread_along_bone(BlamMatrix4x3* palette, const std::uint8_t* nodes, std::size_t count,
                       const std::uint8_t* below, std::size_t below_count, std::uint8_t joint,
                       const Vec3& origin, const Vec3& dir, float bone_length, float k) {
    if (!(k > 1.0f) || !(bone_length > 1.0e-5f)) return;
    for (std::size_t i = 0; i < count; ++i) {
        const std::uint8_t node = nodes[i];
        if (node == joint || in_list(below, below_count, node)) continue;
        const float t = std::clamp(dot(palette[node].position - origin, dir) / bone_length, 0.0f, 1.0f);
        palette[node].position = palette[node].position + dir * (t * bone_length * (k - 1.0f));
    }
}

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

Mat3 slerp_basis(const Mat3& a, const Mat3& b, float weight) {
    weight = std::clamp(weight, 0.0f, 1.0f);
    if (!valid_basis(a) || !valid_basis(b)) return weight < 0.5f ? a : b;
    const Mat3 out = rotation_basis(slerp_short(rotation_from_basis(a), rotation_from_basis(b), weight));
    return valid_basis(out) ? out : (weight < 0.5f ? a : b);
}

bool apply_hand_shape(BlamMatrix4x3* palette, const ArmNodes& arm, float curl, float authored) {
    // The whole-hand shape (pahandrest, the pre-gesture path) keeps its clamp; the pose tunables are
    // unbounded, which is where the exploring happens.
    curl = std::isfinite(curl) ? std::clamp(curl, -1.0f, 1.0f) : 0.0f;
    HandPose p{};
    for (auto& f : p.finger) f.curl = curl;
    return apply_hand_pose(palette, arm, p, authored);
}

const char* hand_pose_name(HandPoseId id) {
    switch (id) {
        case HandPoseId::Rest:      return "rest";
        case HandPoseId::RestIndex: return "index";
        case HandPoseId::Fist:      return "fist";
        case HandPoseId::ThumbsUp:  return "thumbsup";
        case HandPoseId::Point:     return "point";
        case HandPoseId::PointDown: return "pointdown";
        case HandPoseId::Ok:        return "ok";
        default:                    return "?";
    }
}

HandPoseId pose_for_inputs(bool grip, bool trigger, bool thumb) {
    if (grip) {
        if (trigger) return thumb ? HandPoseId::Fist : HandPoseId::ThumbsUp;
        return thumb ? HandPoseId::PointDown : HandPoseId::Point;
    }
    if (trigger) return thumb ? HandPoseId::Ok : HandPoseId::RestIndex;
    return HandPoseId::Rest;
}

void ease_pose_blend(HandPoseBlend& blend, HandPoseId target, float dt, float tau) {
    if (!(dt > 0.0f) || dt > 0.1f) dt = 0.1f;
    const float k = (tau > 0.0f) ? 1.0f - std::exp(-dt / tau) : 1.0f;
    const int t = static_cast<int>(target);
    float sum = 0.0f;
    for (int i = 0; i < kHandPoseCount; ++i) {
        const float want = (i == t) ? 1.0f : 0.0f;
        blend.w[i] += (want - blend.w[i]) * k;
        if (!std::isfinite(blend.w[i]) || blend.w[i] < 0.0f) blend.w[i] = want;
        sum += blend.w[i];
    }
    if (sum > 1.0e-6f) { for (float& w : blend.w) w /= sum; }
    else               { for (int i = 0; i < kHandPoseCount; ++i) blend.w[i] = (i == t) ? 1.0f : 0.0f; }
}

HandPose blend_hand_poses(const HandPose* table, const HandPoseBlend& blend) {
    HandPose out{};
    if (table == nullptr) return out;
    const auto mix = [](float& dst, float src, float w) { if (std::isfinite(src)) dst += src * w; };
    for (int p = 0; p < kHandPoseCount; ++p) {
        const float w = blend.w[p];
        if (!(w > 1.0e-6f)) continue;
        const HandPose& s = table[p];
        for (int f = 0; f < kHandFingers; ++f) {
            mix(out.finger[f].curl, s.finger[f].curl, w);
            for (int j = 0; j < kHandSegments; ++j) {
                mix(out.finger[f].seg[j], s.finger[f].seg[j], w);
                for (int a = 0; a < 3; ++a) mix(out.finger[f].rot[j][a], s.finger[f].rot[j][a], w);
            }
        }
        mix(out.thumb_over, s.thumb_over, w);
        mix(out.thumb_ext,  s.thumb_ext,  w);
        mix(out.thumb_out,  s.thumb_out,  w);
    }
    return out;
}

namespace {
// A rotation of `deg` degrees about local X, then Y, then Z (composed in the joint's own frame).
Quat quat_from_xyz_deg(const float deg[3]) {
    const float k = 0.00872664626f;   // pi / 360: half-angle per degree
    const Quat qx{std::sin(deg[0] * k), 0.0f, 0.0f, std::cos(deg[0] * k)};
    const Quat qy{0.0f, std::sin(deg[1] * k), 0.0f, std::cos(deg[1] * k)};
    const Quat qz{0.0f, 0.0f, std::sin(deg[2] * k), std::cos(deg[2] * k)};
    return qx * qy * qz;
}
float finite_or(float v, float fallback) { return std::isfinite(v) ? v : fallback; }
} // namespace

bool apply_hand_pose(BlamMatrix4x3* palette, const ArmNodes& arm, const HandPose& pose, float authored) {
    if (palette == nullptr) return false;
    authored = std::isfinite(authored) ? std::clamp(authored, 0.0f, 1.0f) : 0.0f;   // a blend weight, not a tunable
    if (authored >= 0.999f) return true;               // the game's own fingers, untouched
    const float over = finite_or(pose.thumb_over, 0.0f);
    const float ext  = finite_or(pose.thumb_ext,  0.0f);
    const float out  = finite_or(pose.thumb_out,  0.0f);

    const Mat3 wrist_basis = orthonormal_basis(palette[arm.wrist]);
    if (!valid_basis(wrist_basis)) return false;
    const Vec3 wrist_pos = palette[arm.wrist].position;

    const FingerChain* chains[5] = {&arm.index, &arm.middle, &arm.ring, &arm.pinky, &arm.thumb};
    for (std::size_t f = 0; f < 5; ++f) {
        const FingerChain& ch = *chains[f];
        const FingerPose&  fp = pose.finger[f];
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
            // rest -> fist and never passes through the stretched hand on the way. The segment's
            // own offset rides on top; past +-1 extrapolates beyond the recorded poses.
            float curl = finite_or(fp.curl, 0.0f);
            if (j < static_cast<std::size_t>(kHandSegments)) curl += finite_or(fp.seg[j], 0.0f);
            Quat q = (curl >= 0.0f) ? slerp_short(qr, qf, curl) : slerp_short(qr, qo, -curl);
            if (f == 4) {
                // The thumb's extras. Past the fist to wrap over the fingers, scaled by the curl so
                // it never leads it...
                if (curl > 0.0f && over != 0.0f) q = slerp_short(qo, q, 1.0f + over * curl);
                // ...its BASE turned back out toward the open hand, so a curled thumb lies outside
                // the index rather than through it...
                if (j == 0 && curl > 0.0f && out != 0.0f) q = slerp_short(q, qo, out * curl);
                // ...or, OPEN, its outer joints carried past the open hand (the recorded open hand
                // leaves the tip bent); the base stays put. Negative = short of the open hand.
                if (j >= 1 && curl < 0.0f && ext != 0.0f) q = slerp_short(qr, qo, -curl * (1.0f + ext));
            }
            // The segment's own rotation trim, degrees about its local X / Y / Z.
            if (j < static_cast<std::size_t>(kHandSegments)) {
                const float deg[3] = {finite_or(fp.rot[j][0], 0.0f), finite_or(fp.rot[j][1], 0.0f),
                                      finite_or(fp.rot[j][2], 0.0f)};
                if (deg[0] != 0.0f || deg[1] != 0.0f || deg[2] != 0.0f) q = normalized(q * quat_from_xyz_deg(deg));
            }
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
    //
    // SHARED WITH THE WRIST (stretch_share). The bones take `share` of the extension the target
    // asks for, up to the cap; whatever is left still opens at the wrist when the hand is snapped
    // onto the controller. 1 = the arm takes all of it until the cap, 0 = the old clamp.
    if (tuning.stretch_max > 1.0f && tuning.stretch_share > 0.0f && target_distance > maximum_reach) {
        const float needed = target_distance / (upper_length + lower_length);
        stretch_k     = std::min(1.0f + (needed - 1.0f) * std::min(tuning.stretch_share, 1.0f),
                                 tuning.stretch_max);
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
        // slide everything from the elbow down onto the stretched elbow -- and the upper arm's own
        // helper nodes out along the bone with it, each by its station.
        const Vec3 upper_dir = normalized(palette[arm.elbow].position - shoulder_position);
        if (length_squared(upper_dir) > 0.8f) {
            spread_along_bone(palette, arm.shoulder_subtree, arm.shoulder_count,
                              arm.elbow_subtree, arm.elbow_count, arm.shoulder,
                              shoulder_position, upper_dir, upper_length, stretch_k);
        }
        const Vec3 stretch_shift = elbow_target - palette[arm.elbow].position;
        apply_rigid_offset(palette, arm.elbow_subtree, arm.elbow_count, stretch_shift);
    }

    const Vec3 moved_elbow = palette[arm.elbow].position;
    const Vec3 moved_wrist = palette[arm.wrist].position;
    const Mat3 elbow_rotation =
        rotation_between(moved_wrist - moved_elbow, wrist_target - moved_elbow);
    if (!valid_basis(elbow_rotation)) return false;
    apply_rigid_delta(palette, arm.elbow_subtree, arm.elbow_count, elbow_rotation, moved_elbow);
    if (stretch_k > 1.0f) {
        // THE FOREARM'S HALF, which the 2026-09-16 stretch left out: the solve above used a
        // stretched forearm LENGTH, but a rotation cannot lengthen anything, so the wrist still
        // sat at its authored distance and the whole forearm extension opened at the wrist. Carry
        // the hand out to the stretched length and spread the forearm's nodes behind it.
        const Vec3 fore_dir = normalized(palette[arm.wrist].position - moved_elbow);
        if (length_squared(fore_dir) > 0.8f) {
            spread_along_bone(palette, arm.elbow_subtree, arm.elbow_count,
                              arm.wrist_subtree, arm.wrist_count, arm.elbow,
                              moved_elbow, fore_dir, lower_length, stretch_k);
            apply_rigid_offset(palette, arm.wrist_subtree, arm.wrist_count,
                               fore_dir * (lower_length * (stretch_k - 1.0f)));
        }
    }

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
                              const Vec3& thumb_up_hint, float gain, float plate_gain, float bone_gain,
                              ForearmTwistResult* result) {
    if (result != nullptr) *result = ForearmTwistResult{};
    if (palette == nullptr || !stock.valid || !std::isfinite(gain) || !std::isfinite(plate_gain) ||
        !std::isfinite(bone_gain)) return false;
    gain       = std::clamp(gain, 0.0f, 2.0f);
    plate_gain = std::clamp(plate_gain, 0.0f, 3.0f);
    bone_gain  = std::clamp(bone_gain, 0.0f, 3.0f);

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
    // THE FOREARM BONE ITSELF. Everything above turns nodes that hang OFF the elbow; whatever is
    // skinned to the elbow node proper -- on this rig, by elimination, the big forearm armour: the
    // twist bones visibly roll the sleeve, the armour nodes turned out to carry nothing visible, and
    // the plate the player watches did not move -- still rides the elbow rigidly. It takes the near
    // twist bone's share (so the roll stays monotonic elbow -> wrist at bone_gain <= 1), about its
    // own origin, which is on the axis: the joint does not move, the twist bones and the hand are
    // model-space nodes of their own and are not carried along.
    if (bone_gain > 0.0f) {
        const float half = gain * bone_gain * twist_share(1.0f / 3.0f) * follow_deg * 0.00872664626f;
        const float s    = std::sin(half);
        const Mat3  roll = rotation_basis(Quat{axis.x * s, axis.y * s, axis.z * s, std::cos(half)});
        if (valid_basis(roll)) {
            apply_rigid_delta(palette, &arm.elbow, 1, roll, elbow_pos);
            if (result != nullptr) result->bone_deg = gain * bone_gain * twist_share(1.0f / 3.0f) * follow_deg;
        }
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
            // 3 mm and 1 degree a frame: an idle SWAY (a slow figure-of-eight of a centimetre or
            // two) moves the marker up to ~2 mm a frame and is rest; a kick or a draw moves it
            // 6-20 mm a frame and is not. At the old 0.6 mm / 0.3 deg a swaying weapon never rested
            // and its first reload after pickup had nothing to measure from (4-7 s in the log).
            still = step_m < 0.003f && align > 0.99985f;
        }
        prev_pos = marker_pos; prev_basis = marker_basis; have_prev = true;
        stable = still ? std::min(stable + 1, 1000000) : 0;
        // A pose FAR from the known rest has to hold for a second and a half before it is believed,
        // not a third of one: an animation that pauses (a shell-by-shell reload holds the gun tilted
        // between shells) must not teach its pause as the rest pose, or everything measured from
        // rest -- the kick below, and the action watch -- would read zero in the middle of it.
        // A REMEMBERED rest pose (adopt) is held loosely: the first learn takes it wherever the
        // gun now rests, in the short time, so a memory a sway's width off does not cost 1.5 s.
        int need = 20;
        if (have_ref && !remembered) {
            float near_align = dot(ref_basis.forward, marker_basis.forward);
            near_align = std::min(near_align, dot(ref_basis.left, marker_basis.left));
            near_align = std::min(near_align, dot(ref_basis.up, marker_basis.up));
            const bool near_rest = length(marker_pos - ref_pos) * kMetresPerBlamUnit < 0.015f &&
                                   near_align > 0.99619f;                      // 1.5 cm, 5 degrees
            if (!near_rest) need = 90;
        }
        if (stable >= need) {
            if (!have_ref || remembered) { ref_pos = marker_pos; ref_basis = marker_basis; have_ref = true; ++latches; }
            else {
                ref_pos   = ref_pos + (marker_pos - ref_pos) * 0.2f;
                ref_basis = blend_basis(ref_basis, marker_basis, 0.2f);
                if (!valid_basis(ref_basis)) ref_basis = marker_basis;
            }
            remembered = false;
        }
    }
    // HOW FAR THE GUN IS FROM REST, for anyone who asks (the action watch does) -- whatever the
    // recoil gain is, so switching the kick off does not blind it.
    last_moved_m = 0.0f; last_turned_deg = 0.0f;
    if (!have_ref) return {};
    {
        float a = dot(ref_basis.forward, marker_basis.forward);
        a = std::min(a, dot(ref_basis.left, marker_basis.left));
        a = std::min(a, dot(ref_basis.up, marker_basis.up));
        last_moved_m    = length(marker_pos - ref_pos) * kMetresPerBlamUnit;
        last_turned_deg = std::acos(std::clamp(a, -1.0f, 1.0f)) * 57.2957795131f;
    }
    if (!(gain > 0.0f) || !(max_m > 0.0f) || !std::isfinite(gain) || !std::isfinite(max_m))
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
    const float weight = (1.0f - smoothstep(8.0f, 20.0f, last_turned_deg)) *
                         (1.0f - smoothstep(max_m, 2.0f * max_m, last_moved_m));
    const float out_m = std::min(back * kMetresPerBlamUnit, max_m) * weight * std::min(gain, 2.0f);
    if (!(out_m > 0.0f) || !std::isfinite(out_m)) return {};
    last_back_m = out_m;
    return back_axis * (out_m / kMetresPerBlamUnit);
}

// ---- REST RELATIONS, THE ACTION WATCH, THE MELEE GATE ------------------------------------------

bool RecoilPass::at_rest() const {
    return have_ref && stable >= 20 && last_moved_m < 0.005f && last_turned_deg < 2.0f;
}

void RecoilPass::adopt(const Vec3& pos, const Mat3& basis) {
    reset();
    if (!finite(pos) || !valid_basis(basis)) return;
    ref_pos = pos; ref_basis = basis; have_ref = true; remembered = true;
}

void RestRelation::reset() { *this = RestRelation{}; }

void RestRelation::adopt(const Vec3& p, const Mat3& b) {
    reset();
    if (!finite(p) || !valid_basis(b)) return;
    pos = p; basis = b; have = true; remembered = true;
}

void RestRelation::learn(bool gun_at_rest, const Vec3& p, const Mat3& b) {
    dev_m = 0.0f; dev_deg = 0.0f;
    if (!finite(p) || !valid_basis(b)) { stable = 0; have_prev = false; return; }

    bool still = false;
    if (have_prev) {
        float a = dot(prev_basis.forward, b.forward);
        a = std::min(a, dot(prev_basis.left, b.left));
        a = std::min(a, dot(prev_basis.up, b.up));
        still = length(p - prev_pos) * kMetresPerBlamUnit < 0.003f && a > 0.99985f;   // 3 mm, 1 degree (as RecoilPass)
    }
    prev_pos = p; prev_basis = b; have_prev = true;
    stable = still ? std::min(stable + 1, 1000000) : 0;

    if (have) {
        float a = dot(basis.forward, b.forward);
        a = std::min(a, dot(basis.left, b.left));
        a = std::min(a, dot(basis.up, b.up));
        dev_m   = length(p - pos) * kMetresPerBlamUnit;
        dev_deg = std::acos(std::clamp(a, -1.0f, 1.0f)) * 57.2957795131f;
    }
    // Only ever learned while the GUN is at rest, and -- like the gun's own rest pose -- a relation
    // far from the known one has to hold for 1.5 s before it is believed. A REMEMBERED one (adopt)
    // is re-anchored by the first learn in the short time, whatever the distance.
    if (!gun_at_rest) return;
    const bool near_rest = !have || remembered || (dev_m < 0.015f && dev_deg < 5.0f);
    if (stable < (near_rest ? 20 : 90)) return;
    if (!have || remembered) { pos = p; basis = b; have = true; }
    else {
        pos   = pos + (p - pos) * 0.2f;
        basis = blend_basis(basis, b, 0.2f);
        if (!valid_basis(basis)) basis = b;
    }
    remembered = false;
}

void ActionWatch::reset() {
    const float keep = weight;                 // the hand must not pop because the weapon changed
    *this = ActionWatch{};
    weight = keep;
}

float ActionWatch::update(const RecoilPass& gun, const Vec3& hand_pos, const Mat3& hand_basis,
                          float gate, float dt, float melee_age_s, float reload_age_s, float grenade_age_s,
                          float grenade_trim_s) {
    if (!(dt > 0.0f) || dt > 0.1f) dt = 0.1f;
    gate = std::isfinite(gate) ? std::clamp(gate, 0.25f, 4.0f) : 1.0f;
    const float prev_dev = hand.have ? hand.dev_m : 0.0f;
    hand.learn(gun.at_rest(), hand_pos, hand_basis);

    // WHAT COUNTS AS AN ACTION. Two bodies of evidence: the recording (Magnum + Assault Rifle, 5149
    // frames) and a headset session's PALETTE ANIM lines (54 hand-overs across the whole arsenal).
    //
    //                               gun from rest          off hand RELATIVE TO THE GUN
    //   melee / reload / grenade    13-87 cm, 19-178 deg   14-108 cm,  44-176 deg
    //   a weapon's PUT-AWAY         25-46 cm, 27-55 deg    0.6-10.4 cm, 1-5 deg
    //   a big kick (one weapon)     16-17 cm,  5-6 deg     0.9-1.4 cm,  1-3 deg
    //   the rifle's burst, idle     <= 4.6 cm, <= 5.8 deg  <= 0.7 cm,   <= 4 deg
    //
    // So the OFF HAND is the action signal -- above all its TURN, which separates an action from a
    // put-away by 44 against 5 degrees. The gun leaving rest on its own means only "this weapon
    // is going away" (or kicked hard), and is the EquipGate's business, not this one's. The bands
    // are smoothsteps so a borderline case tugs rather than throws; `gate` scales them.
    float target = 0.0f;
    if (gun.have_ref && hand.have) {
        target = std::max(smoothstep(0.12f * gate, 0.20f * gate, hand.dev_m),
                          smoothstep(12.0f * gate, 30.0f * gate, hand.dev_deg));
    }

    // A NEW PRESS STARTS A NEW ACTION. A hand-over cut for its return holds until the authored
    // hand is home -- and if it never quite gets there (a relation a few centimetres off after a
    // re-anchor), the NEXT reload's press found the cut still latched and was refused outright
    // ("action w=0.01 (cut, waiting for home)" at a reload press, headset 2026-09-17). The press
    // is the player asking again: drop the old action, cut and all, and take this one fresh.
    {
        float youngest = -1.0f;
        const auto take = [&](float a) { if (a >= 0.0f && (youngest < 0.0f || a < youngest)) youngest = a; };
        take(melee_age_s); take(reload_age_s); take(grenade_age_s);
        if (youngest >= 0.0f && youngest < 0.05f && (home_cut || (engaged && since_onset_s > 0.3f))) {
            home_cut = false; engaged = false;
        }
    }

    // WHICH ACTION, from the press that asked for it (a throw, a reload and a melee are always
    // asked for -- by a gesture that presses the mask, or a thumb).
    if (target > 0.0f && !engaged) {
        engaged = true; since_onset_s = 0.0f; peak_m = 0.0f; descending = 0;
        kind = Kind::Other;
        if      (grenade_age_s >= 0.0f && grenade_age_s < 0.6f) kind = Kind::Grenade;
        else if (reload_age_s  >= 0.0f && reload_age_s  < 0.6f) kind = Kind::Reload;
        else if (melee_age_s   >= 0.0f && melee_age_s   < 0.6f) kind = Kind::Melee;
    }
    if (engaged) {
        since_onset_s += dt;
        peak_m = std::max(peak_m, hand.dev_m);
        descending = (hand.dev_m < prev_dev - 0.008f) ? descending + 1 : 0;     // 0.8 cm a frame
    }

    // THE RETURN TO THE GRIP. Every action ends with the authored off hand coming home onto the
    // forestock; played on a free hand that walks the player's hand onto the gun and drops it back
    // at the controller from there ("a notify or something that tells the left hand to go back to
    // the support grip, and that might confuse some players"). Measured on the recording:
    //   * a melee is one peak: out, held, and a single 0.2-0.3 s return (the punch: 107 cm held,
    //     then 98 / 76 / 52 / 10 cm on consecutive 4-frame steps) -- so the FIRST sustained approach
    //     after the peak is the return, and the hand lets go there;
    //   * a reload has a return in the MIDDLE that looks the same at its onset -- the hand bringing
    //     the magazine in while the gun untilts (105 -> 68 cm with the gun still 89 deg over), then
    //     out again -- so for a reload, and for anything not asked for (an auto-reload on an empty
    //     magazine has no press), the approach only counts once the gun is nearly home itself;
    //   * a throw's final return is too quick to catch by its shape (42 -> 16 cm in four frames after
    //     a 0.4 s hang), so it keeps its time trim: cut `grenade_trim_s` before the authored end
    //     (1.35 s: release ~0.15, arm out to ~0.75, home by ~1.0).
    // A cut holds until the authored hand is home, so the hand is not taken again on the way in.
    if (engaged && !home_cut) {
        const bool past_peak  = hand.dev_m < 0.85f * peak_m && hand.dev_m > 0.20f * gate;
        const bool approaching = descending >= 3 && past_peak;
        const bool gun_home    = gun.last_moved_m < 0.25f && gun.last_turned_deg < 40.0f;
        switch (kind) {
            case Kind::Grenade: if (grenade_trim_s > 0.0f && since_onset_s > kThrowSeconds - grenade_trim_s) home_cut = true; break;
            case Kind::Melee:   if (approaching) home_cut = true; break;
            case Kind::Reload:
            case Kind::Other:   if (approaching && gun_home) home_cut = true; break;
        }
    }
    if (home_cut) {
        if (target <= 0.0f) home_cut = false;                   // the authored hand is home again
        target = 0.0f;
    }
    last_target = target;

    // In quickly (a melee lands within a quarter second), out at the two-hand hold's own pace.
    const float tau = target > weight ? 0.06f : 0.12f;
    weight += (target - weight) * (1.0f - std::exp(-dt / tau));
    if (!std::isfinite(weight)) weight = 0.0f;
    weight = std::clamp(weight, 0.0f, 1.0f);
    if (weight < 0.001f && target <= 0.0f) weight = 0.0f;
    if (weight <= 0.0f && target <= 0.0f) { engaged = false; kind = Kind::Other; }
    return weight;
}

void EquipGate::reset() { *this = EquipGate{}; }

float EquipGate::update(float swap_age_s, bool weapon_changed, const RecoilPass& gun, float dt,
                        float other_age_s) {
    if (!(dt > 0.0f) || dt > 0.1f) dt = 0.1f;
    // THE PUT-AWAY: the gun leaving rest within a second of a swap being asked for. THE DRAW: from
    // the weapon model changing until the new weapon has come to rest (3 s at most). Neither can
    // be told from a melee or a big kick by the pose alone -- the swap press and the model change
    // are what make them equip.
    const bool away = gun.have_ref && (gun.last_moved_m > 0.06f || gun.last_turned_deg > 12.0f);
    if (weapon_changed) { active = true; draw = true; since_s = 0.0f; }
    else if (!active && swap_age_s >= 0.0f && swap_age_s < 1.0f && away &&
             !(other_age_s >= 0.0f && other_age_s < swap_age_s)) {      // ...unless something else was asked for since
        active = true; draw = false; since_s = 0.0f;
    }
    if (active) {
        since_s += dt;
        if (draw) { if ((gun.have_ref && gun.at_rest()) || since_s > 3.0f) active = false; }
        else      { if (!away || since_s > 2.0f) active = false; }
        // ANOTHER ACTION ASKED FOR since this began ends it: the game takes no reload, melee or
        // throw during a swap, so the press means the swap is over -- and a reload straight off
        // the draw (the common one: swap to the empty gun, reload it) would otherwise sit behind
        // this gate until the gun rested, which a reload never lets it do.
        if (other_age_s >= 0.0f && other_age_s < since_s) active = false;
    }
    const float target = active ? 1.0f : 0.0f;
    const float tau = target > weight ? 0.06f : 0.12f;
    weight += (target - weight) * (1.0f - std::exp(-dt / tau));
    if (!std::isfinite(weight)) weight = 0.0f;
    weight = std::clamp(weight, 0.0f, 1.0f);
    if (weight < 0.001f && !active) weight = 0.0f;
    return weight;
}

bool capture_hand_rest(const BlamMatrix4x3* palette, const ArmNodes& arm, RestNode* out, std::size_t out_count) {
    if (palette == nullptr || out == nullptr || out_count < arm.wrist_count) return false;
    const Mat3 wb = orthonormal_basis(palette[arm.wrist]);
    if (!valid_basis(wb)) return false;
    const Mat3 winv = transpose(wb);
    const Vec3 wp   = palette[arm.wrist].position;
    for (std::size_t i = 0; i < arm.wrist_count; ++i) {
        const BlamMatrix4x3& n = palette[arm.wrist_subtree[i]];
        const Mat3 nb = orthonormal_basis(n);
        if (!valid_basis(nb)) return false;
        out[i].pos   = transform_vector(winv, n.position - wp);
        out[i].basis = multiply(winv, nb);
    }
    return true;
}

bool blend_hand_to_rest(BlamMatrix4x3* palette, const ArmNodes& arm, const RestNode* rest, std::size_t rest_count,
                        float weight) {
    if (palette == nullptr || rest == nullptr || rest_count < arm.wrist_count) return false;
    weight = std::clamp(weight, 0.0f, 1.0f);
    if (weight <= 0.0f) return true;
    const Mat3 wb = orthonormal_basis(palette[arm.wrist]);
    if (!valid_basis(wb)) return false;
    const Vec3 wp = palette[arm.wrist].position;
    for (std::size_t i = 0; i < arm.wrist_count; ++i) {
        const std::uint8_t node = arm.wrist_subtree[i];
        if (node == arm.wrist) continue;
        BlamMatrix4x3& n = palette[node];
        const Mat3 live = orthonormal_basis(n);
        const Mat3 want = multiply(wb, rest[i].basis);
        if (!valid_basis(live) || !valid_basis(want)) continue;
        const Mat3 b = slerp_basis(live, want, weight);
        const Vec3 p = wp + transform_vector(wb, rest[i].pos);
        n.forward = b.forward; n.left = b.left; n.up = b.up;
        n.position = n.position + (p - n.position) * weight;
    }
    return true;
}

void MeleeGate::reset() { *this = MeleeGate{}; }

float MeleeGate::update(float press_age_s, const RecoilPass& gun, float hand_dev_m, float hand_dev_deg,
                        float dt, float other_age_s) {
    if (!(dt > 0.0f) || dt > 0.1f) dt = 0.1f;
    const bool pressed = press_age_s >= 0.0f && press_age_s < 0.4f;
    // "Still going" is read off the same two signals the action watch uses, at a much lower bar:
    // this is not deciding WHETHER something is an action, only whether the one a melee press
    // started has come back to rest. Without a rest pose to measure from (a melee straight after a
    // swap) it is a plain timer.
    const bool busy = gun.have_ref
        ? (gun.last_moved_m > 0.02f || gun.last_turned_deg > 3.0f || hand_dev_m > 0.02f || hand_dev_deg > 5.0f)
        : (since_s < 1.2f);
    if (!active) {
        // ...and a melee press only opens it while nothing else has been asked for since, or
        // the press that closed it below would reopen it on the next frame.
        if (pressed && !(other_age_s >= 0.0f && other_age_s < press_age_s)) { active = true; since_s = 0.0f; }
    } else {
        since_s += dt;
        if ((!pressed && !busy && since_s > 0.15f) || since_s > 4.0f) active = false;
        // A reload or a throw asked for after the swing ends it: "busy" cannot tell the melee's
        // own return from the reload that follows it, and would hold the free hand off that
        // reload for the whole 4 s cap.
        if (other_age_s >= 0.0f && other_age_s < since_s) active = false;
    }
    const float target = active ? 1.0f : 0.0f;
    // In AHEAD of the animation -- the press leads it by a few frames, which is the whole point of
    // keying this off the press rather than off the pose -- and out at the hand-over's own pace.
    const float tau = target > weight ? 0.03f : 0.12f;
    weight += (target - weight) * (1.0f - std::exp(-dt / tau));
    if (!std::isfinite(weight)) weight = 0.0f;
    weight = std::clamp(weight, 0.0f, 1.0f);
    if (weight < 0.001f && !active) weight = 0.0f;
    return weight;
}

void SprintWatch::reset() { *this = SprintWatch{}; }

float SprintWatch::update(float button_age_s, float move_age_s, const RecoilPass& gun, float dt,
                          float other_age_s) {
    if (!(dt > 0.0f) || dt > 0.1f) dt = 0.1f;
    // The sprint animation carries the gun well away from rest for as long as the sprint lasts.
    // That alone is also what a put-away or a melee looks like, so it counts only with the player
    // pushing the stick AND having asked for a sprint in the last three seconds (a hold, or a
    // toggle pressed before setting off). It ends when either the stick or the pose lets go --
    // the game itself ends a sprint on firing, aiming or stopping -- and never outlives the rest
    // pose it is measured against. And it is NOT a sprint within two seconds of a reload, melee
    // or throw being asked for: a reload takes the gun just as far from rest, with the stick
    // pushed and a sprint asked for moments before it read as one and held the free hand off.
    const bool away   = gun.have_ref && (gun.last_moved_m > 0.06f || gun.last_turned_deg > 12.0f);
    const bool moving = move_age_s >= 0.0f && move_age_s < 0.2f;
    const bool asked  = button_age_s >= 0.0f && button_age_s < 3.0f;
    const bool other  = other_age_s >= 0.0f && other_age_s < 2.0f;
    if (!active) {
        if (away && moving && asked && !other) { active = true; quiet_s = 0.0f; }
    } else {
        if (away && moving) quiet_s = 0.0f; else quiet_s += dt;
        if (quiet_s > 0.15f || !gun.have_ref || other) active = false;
    }
    const float target = active ? 1.0f : 0.0f;
    const float tau = target > weight ? 0.08f : 0.15f;
    weight += (target - weight) * (1.0f - std::exp(-dt / tau));
    if (!std::isfinite(weight)) weight = 0.0f;
    weight = std::clamp(weight, 0.0f, 1.0f);
    if (weight < 0.001f && !active) weight = 0.0f;
    return weight;
}

} // namespace halo::palettearm
