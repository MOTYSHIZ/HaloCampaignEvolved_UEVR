// Blam-frame vector algebra for the palette arm driver.
//
// SELF-CONTAINED ON PURPOSE. Nothing here includes a project header, a UEVR header or Windows.
// See README.md: the pure layer is what Scripts\Verify-PaletteArm.ps1 can compile out of tree and
// unit-test without a game, and that property survives only if it is defended.
//
// This does NOT reuse src\Math.hpp, and the duplication is deliberate. Math.hpp is Unreal's frame:
// centimetres, Z-up, left-handed, and its Vec3 means a world-space UE vector. Everything in this
// folder is Halo's frame -- Blam units (1 unit = 3.048 m), forward/left/up basis triples stored the
// way the engine's own node matrices store them. Sharing a Vec3 between the two frames would make
// a units or handedness mistake invisible at the call site, and that class of mistake has already
// cost this project once (see the memory note on distance units).
//
// Ported from elliotttate's HaloCampaignEvolved-UEVR (main.cpp @ 62ee34f) with permission.

#pragma once

#include <cmath>
#include <cstdint>

namespace halo::palettearm {

// 1 Blam world unit in metres. Every constant in this folder is authored in metres and divided by
// this at use, because metres are the unit a person can reason about in a headset.
constexpr float kMetresPerBlamUnit = 3.048f;

struct Vec3 {
    float x{};
    float y{};
    float z{};
};

struct Quat {
    float x{};
    float y{};
    float z{};
    float w{1.0f};
};

// A rotation as three basis vectors, in Blam's own naming. NOT a generic 3x3: the field order is
// the order the engine stores them, so a BlamMatrix4x3 can be read into one without shuffling.
struct Mat3 {
    Vec3 forward{1.0f, 0.0f, 0.0f};
    Vec3 left{0.0f, 1.0f, 0.0f};
    Vec3 up{0.0f, 0.0f, 1.0f};
};

// One node of the first-person palette, exactly as the engine lays it out. The static_assert is
// the contract: this struct is reinterpreted over memory the game wrote, so a field added or
// reordered here corrupts every node rather than failing to compile.
struct BlamMatrix4x3 {
    float scale{1.0f};
    Vec3  forward{};
    Vec3  left{};
    Vec3  up{};
    Vec3  position{};
};
static_assert(sizeof(BlamMatrix4x3) == 0x34, "palette node layout must match the engine's");

// ---- Vec3 -------------------------------------------------------------------------------------

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(const Vec3& v, float s)       { return {v.x * s, v.y * s, v.z * s}; }
inline Vec3 operator/(const Vec3& v, float s)       { return {v.x / s, v.y / s, v.z / s}; }

inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

inline float length_squared(const Vec3& v) { return dot(v, v); }
inline float length(const Vec3& v)         { return std::sqrt(length_squared(v)); }

inline bool finite(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
inline bool finite(const Quat& q) {
    return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

// Returns the ZERO vector for a degenerate input rather than a NaN one. Callers rely on that:
// several of them test length_squared(normalized(v)) < 0.8f as "this direction was unusable",
// which only works because failure is zero and not NaN.
Vec3 normalized(const Vec3& v);

// ---- Quat -------------------------------------------------------------------------------------

Quat operator*(const Quat& a, const Quat& b);
Quat conjugate(const Quat& q);
Quat normalized(const Quat& q);
Vec3 rotate(const Quat& rotation, const Vec3& v);

// ---- Mat3 -------------------------------------------------------------------------------------

inline Vec3 transform_vector(const Mat3& m, const Vec3& v) {
    return m.forward * v.x + m.left * v.y + m.up * v.z;
}

Mat3 multiply(const Mat3& a, const Mat3& b);
Mat3 transpose(const Mat3& m);
Mat3 rotation_basis(const Quat& q);

// The inverse of rotation_basis(): the quaternion whose rotation IS this basis.
//
// Exists so a solved rotation can be PERSISTED. Everything in this folder composes bases, but a
// config file has to hold four numbers rather than nine, and re-deriving a basis from three Euler
// angles is both lossy near vertical and wrong to compose (see the per-weapon fix in
// PaletteArm.cpp, which round-trips through this on every capture).
//
// Returns IDENTITY for anything that is not a rotation, matching rotation_between()'s contract --
// a caller that writes the result to disk must never be handed a NaN.
Quat rotation_from_basis(const Mat3& m);

// The shortest-arc rotation taking `source` onto `destination`.
//
// Returns IDENTITY for the degenerate cases -- inputs too short to normalise, or already aligned
// within 0.9999. Identity is the honest answer there (nothing to do), and it is also what makes
// the IK safe: an un-normalisable bone must not produce a rotation at all.
Mat3 rotation_between(const Vec3& source, const Vec3& destination);

// ---- THE BARREL LOCK -----------------------------------------------------------------------
//
// The correction that puts a rendered `barrel` direction back onto the `aim` ray, faded out once
// the two disagree by more than a player would accept as "a trim".
//
// WHY A SHORTEST ARC AND NOTHING ELSE. It is the unique rotation taking one direction onto another
// with no twist about either, so left-multiplying it onto a weapon basis moves ONLY where the gun
// points -- the roll the player's wrist put on the gun survives untouched. Any other rotation with
// the same effect on the barrel would also spin the model about it.
//
// THE FADE, and it is the one place this differs from blindcowboy24's PR #1. His is a hard cap: full
// correction below 25 degrees, none above. That is a 25-degree SNAP of the weapon model at the
// boundary, which is exactly where a fast flick lives. `full_deg` is where the correction is still
// applied whole and `release_deg` is where it has faded to nothing, so the model eases back to
// following the hand instead of jumping there. Passing release_deg <= full_deg restores his hard
// cap exactly, which is what makes the two comparable in a headset.
//
// Returns IDENTITY -- never a NaN, never a partial basis -- for every degenerate input, so a caller
// can left-multiply the result unconditionally.
Mat3 barrel_lock_correction(const Vec3& barrel, const Vec3& aim, float full_deg, float release_deg);

// The rotation part of a palette node, with each axis normalised. Node matrices carry a separate
// scale field, so the stored axes are not unit length and must not be used raw as a basis.
Mat3 orthonormal_basis(const BlamMatrix4x3& node);

// Is this actually a rotation? Checks finiteness, near-unit axis lengths (0.8..1.2 squared) and
// near-orthogonality (|dot| < 0.2 pairwise).
//
// The tolerances are loose ON PURPOSE. This gates writes into the game's render palette, and the
// question it must answer is "is this garbage" -- not "is this precisely orthonormal". A tight
// tolerance here would reject slightly-drifted-but-fine bases every frame and silently disable the
// arms, which is the failure that looks like the feature was never implemented.
bool valid_basis(const Mat3& m);

// Is this node plausibly something the engine wrote, rather than a stale or wild pointer?
// Bounded scale, bounded basis components, bounded position magnitude.
bool reasonable_palette_node(const BlamMatrix4x3& node);

// Normalised-lerp between two orthonormal bases, re-orthonormalised. Only meaningful for NEARBY
// orientations; the two-hand blend keeps its inputs within a few degrees, which is what makes a
// lerp acceptable there instead of a slerp.
Mat3 blend_basis(const Mat3& a, const Mat3& b, float weight);

// Smoothstep, clamped. 0 at or below `edge0`, 1 at or above `edge1`.
inline float smoothstep(float edge0, float edge1, float x) {
    if (!(edge1 > edge0)) return x >= edge1 ? 1.0f : 0.0f;
    float t = (x - edge0) / (edge1 - edge0);
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    return t * t * (3.0f - 2.0f * t);
}

} // namespace halo::palettearm
