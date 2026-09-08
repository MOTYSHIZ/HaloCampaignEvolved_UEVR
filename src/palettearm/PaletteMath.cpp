#include "PaletteMath.hpp"

namespace halo::palettearm {

Vec3 normalized(const Vec3& v) {
    const float squared = length_squared(v);
    // Zero, not NaN, for the degenerate case -- see the header. 1e-8 on the SQUARED length is a
    // 1e-4 vector; below that the direction is noise whatever the arithmetic says.
    if (!std::isfinite(squared) || squared < 1.0e-8f) return {};
    return v / std::sqrt(squared);
}

Quat operator*(const Quat& a, const Quat& b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

Quat conjugate(const Quat& q) { return {-q.x, -q.y, -q.z, q.w}; }

Quat normalized(const Quat& q) {
    const float squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    // Identity, not zero: a zero quaternion is not a rotation, and every caller here wants
    // "leave it alone" as the failure mode.
    if (!std::isfinite(squared) || squared < 1.0e-8f) return {};
    const float inv = 1.0f / std::sqrt(squared);
    return {q.x * inv, q.y * inv, q.z * inv, q.w * inv};
}

Vec3 rotate(const Quat& rotation, const Vec3& v) {
    const Quat vector{v.x, v.y, v.z, 0.0f};
    const Quat result = rotation * vector * conjugate(rotation);
    return {result.x, result.y, result.z};
}

Mat3 multiply(const Mat3& a, const Mat3& b) {
    return {transform_vector(a, b.forward),
            transform_vector(a, b.left),
            transform_vector(a, b.up)};
}

Mat3 transpose(const Mat3& m) {
    return {{m.forward.x, m.left.x, m.up.x},
            {m.forward.y, m.left.y, m.up.y},
            {m.forward.z, m.left.z, m.up.z}};
}

Mat3 rotation_basis(const Quat& q) {
    return {rotate(q, {1.0f, 0.0f, 0.0f}),
            rotate(q, {0.0f, 1.0f, 0.0f}),
            rotate(q, {0.0f, 0.0f, 1.0f})};
}

Quat rotation_from_basis(const Mat3& m) {
    // Refuse anything that is not a rotation rather than returning a quaternion built from garbage:
    // this feeds a value that gets WRITTEN TO DISK and composed onto for the rest of the session.
    if (!valid_basis(m)) return {};

    // Shepperd's method: pick the branch whose divisor is largest, so the square root is never
    // taken near zero. Our Mat3 stores the basis as COLUMNS (transform_vector sums
    // forward*x + left*y + up*z), so m[row][col] reads as:
    //     m00 m01 m02       forward.x  left.x  up.x
    //     m10 m11 m12   =   forward.y  left.y  up.y
    //     m20 m21 m22       forward.z  left.z  up.z
    const float m00 = m.forward.x, m01 = m.left.x, m02 = m.up.x;
    const float m10 = m.forward.y, m11 = m.left.y, m12 = m.up.y;
    const float m20 = m.forward.z, m21 = m.left.z, m22 = m.up.z;

    const float trace = m00 + m11 + m22;
    Quat q{};
    if (trace > 0.0f) {
        const float s = std::sqrt(trace + 1.0f) * 2.0f;
        q = {(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25f * s};
    } else if (m00 > m11 && m00 > m22) {
        const float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        q = {0.25f * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s};
    } else if (m11 > m22) {
        const float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        q = {(m01 + m10) / s, 0.25f * s, (m12 + m21) / s, (m02 - m20) / s};
    } else {
        const float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        q = {(m02 + m20) / s, (m12 + m21) / s, 0.25f * s, (m10 - m01) / s};
    }
    // ALWAYS normalise. valid_basis() tolerates a drifted basis on purpose (see the header), so the
    // quaternion it produces is only near-unit -- and a non-unit quaternion in rotate() is a
    // scale-and-skew, not a rotation. blindcowboy24 measured that exact drift reaching |q| = 1.028
    // over four captures, which logs as "rotated 0 deg" while the weapon visibly moves.
    if (!finite(q)) return {};
    return normalized(q);
}

Mat3 rotation_between(const Vec3& source, const Vec3& destination) {
    const Vec3 from = normalized(source);
    const Vec3 to   = normalized(destination);
    // normalized() returns zero for anything it could not use, so this one test covers
    // "too short", "NaN" and "infinite" together.
    if (length_squared(from) < 0.8f || length_squared(to) < 0.8f) return {};

    float alignment = dot(from, to);
    alignment = alignment < -1.0f ? -1.0f : (alignment > 1.0f ? 1.0f : alignment);
    if (alignment > 0.9999f) return {};          // already there

    Quat rotation{};
    if (alignment < -0.9999f) {
        // Antiparallel: the shortest arc is undefined, so pick ANY perpendicular axis and turn a
        // half circle about it. Trying X first and falling back to Y covers the case where `from`
        // is itself the X axis.
        Vec3 axis = cross(from, {1.0f, 0.0f, 0.0f});
        if (length_squared(axis) < 1.0e-6f) axis = cross(from, {0.0f, 1.0f, 0.0f});
        axis = normalized(axis);
        rotation = {axis.x, axis.y, axis.z, 0.0f};
    } else {
        const Vec3 axis = cross(from, to);
        rotation = normalized(Quat{axis.x, axis.y, axis.z, 1.0f + alignment});
    }
    return rotation_basis(rotation);
}

Mat3 barrel_lock_correction(const Vec3& barrel, const Vec3& aim, float full_deg, float release_deg) {
    const Vec3 b = normalized(barrel);
    const Vec3 a = normalized(aim);
    // normalized() returns zero for anything unusable, so this covers short, NaN and infinite.
    if (length_squared(b) < 0.8f || length_squared(a) < 0.8f) return {};

    float alignment = dot(b, a);
    alignment = alignment < -1.0f ? -1.0f : (alignment > 1.0f ? 1.0f : alignment);
    const float degrees = std::acos(alignment) * 57.2957795131f;

    // A NEGATIVE OR ZERO full_deg DISABLES THE LOCK, rather than meaning "correct nothing ever
    // except an exact match". A caller that ends up with a zeroed tunable gets the stock behaviour,
    // which is the safe reading of an unset number.
    if (!(full_deg > 0.0f) || !std::isfinite(full_deg)) return {};

    float strength;
    if (!(release_deg > full_deg) || !std::isfinite(release_deg)) {
        strength = (degrees <= full_deg) ? 1.0f : 0.0f;   // blindcowboy24's hard cap
    } else {
        strength = 1.0f - smoothstep(full_deg, release_deg, degrees);
    }
    if (!(strength > 0.0f)) return {};
    if (strength >= 1.0f)   return rotation_between(b, a);

    // PARTIAL. Aim at a point along the chord instead of at the target: same axis, a fraction of
    // the arc. Normalised-lerp rather than a slerp for the same reason blend_basis() uses one --
    // the inputs here are within release_deg of each other by construction, where the two agree to
    // well under the angle a player can see, and the endpoints are exact either way.
    const Vec3 partial = normalized(b + (a - b) * strength);
    if (length_squared(partial) < 0.8f) return {};
    return rotation_between(b, partial);
}

Mat3 orthonormal_basis(const BlamMatrix4x3& node) {
    return {normalized(node.forward), normalized(node.left), normalized(node.up)};
}

bool valid_basis(const Mat3& m) {
    if (!finite(m.forward) || !finite(m.left) || !finite(m.up)) return false;

    const float f = length_squared(m.forward);
    const float l = length_squared(m.left);
    const float u = length_squared(m.up);
    if (f < 0.8f || f > 1.2f || l < 0.8f || l > 1.2f || u < 0.8f || u > 1.2f) return false;

    return std::fabs(dot(m.forward, m.left)) < 0.2f &&
           std::fabs(dot(m.forward, m.up))   < 0.2f &&
           std::fabs(dot(m.left,    m.up))   < 0.2f;
}

bool reasonable_palette_node(const BlamMatrix4x3& node) {
    if (!std::isfinite(node.scale) || !finite(node.forward) || !finite(node.left) ||
        !finite(node.up) || !finite(node.position)) {
        return false;
    }
    // A first-person rig lives within a couple of Blam units of its own root; 100 units squared is
    // ~30 m, far outside anything legitimate and far inside the range a wild pointer produces.
    constexpr float kMaxBasisComponent    = 4.0f;
    constexpr float kMaxPositionSquared   = 10000.0f;
    const auto bounded = [](const Vec3& v) {
        return std::fabs(v.x) <= kMaxBasisComponent &&
               std::fabs(v.y) <= kMaxBasisComponent &&
               std::fabs(v.z) <= kMaxBasisComponent;
    };
    return std::fabs(node.scale) <= 16.0f &&
           bounded(node.forward) && bounded(node.left) && bounded(node.up) &&
           length_squared(node.position) <= kMaxPositionSquared;
}

Mat3 blend_basis(const Mat3& a, const Mat3& b, float weight) {
    const Vec3 forward = normalized(a.forward + (b.forward - a.forward) * weight);
    Vec3 left = a.left + (b.left - a.left) * weight;
    left = left - forward * dot(left, forward);        // re-orthogonalise against the new forward
    if (!finite(left) || length_squared(left) < 1.0e-6f) {
        // Collinear inputs: no basis to build. Snap to whichever end we are nearer rather than
        // returning something that is not a rotation.
        return weight < 0.5f ? a : b;
    }
    left = normalized(left);
    return Mat3{forward, left, normalized(cross(forward, left))};
}

} // namespace halo::palettearm
