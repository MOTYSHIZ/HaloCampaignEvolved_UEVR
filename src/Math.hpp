// Pure maths for the aim loop and the first-person rig.
//
// Everything here is FREE OF PLUGIN STATE by design: no globals, no config, no UEVR API. That is
// what makes it safe to read, reuse and unit-test in isolation, and it is the rule to keep if you
// add to this file -- anything that needs g_cfg or the API belongs in a feature module, not here.
//
// Small, per-frame helpers are `inline` in this header so they stay as cheap as they were when this
// was one translation unit. The heavier calibration-only routines live in Math.cpp.

#pragma once

#include <cmath>

namespace halo {

constexpr float RAD2DEG = 57.2957795131f;
constexpr float DEG2RAD = 0.01745329251f;

// ---- PLANT CURVE -----------------------------------------------------------------------------
// Stick deflection -> turn rate, MEASURED on this title rather than assumed. See
// docs\PlantCurve.md for the measurement and its conditions.
//
// WHY A TABLE AND NOT A NUMBER
//   The loop drives a rate actuator, and feedforward's whole job is "I want R deg/s, what deflection
//   produces that?". That was computed as R / g_meas_rate -- a single deg/s-per-unit scalar. But the
//   real response is strongly progressive: the local gain runs from ~52 at the deadzone edge to ~364
//   at full stick, a 7x spread. A scalar can only be right at one deflection, so feedforward
//   over-drove below it and under-drove above it, everywhere, always. That is why pinning the scalar
//   to any value never helped -- the number was not the problem, the model was.
//
// The table is normalised on rate so it can be SCALED to a different sensitivity: entry rates are
// fractions of the full-deflection rate. Whether sensitivity is a pure multiplier (and therefore
// whether one shape serves every setting) is measured-at-one-point and UNPROVEN -- see the doc.
struct PlantPoint { float deflection; float rate_frac; };

// Measured at LookSensitivity90 / deadzone 0 / acceleration min. rate_frac = deg/s / 363.6.
inline constexpr PlantPoint kPlantCurve[] = {
    {0.260f, 0.0000f},   // dead below here even with the game's look deadzone at 0
    {0.280f, 0.0396f},   // 14.4 deg/s
    {0.350f, 0.0556f},   // 20.2
    {0.450f, 0.0910f},   // 33.1
    {0.600f, 0.1901f},   // 69.1
    {0.750f, 0.3548f},   // 129.0
    {0.900f, 0.5649f},   // 205.4
    {1.000f, 1.0000f},   // 363.6
};
inline constexpr int kPlantCurveN = (int)(sizeof(kPlantCurve) / sizeof(kPlantCurve[0]));

// INVERSE of the curve: given a wanted rate as a fraction of full-deflection rate, return the
// deflection that produces it. Piecewise-linear between measured points, which is honest about
// being an interpolation of samples rather than pretending to a closed form the data does not
// support. Returns 0 for a rate the plant cannot produce at all (inside the dead region), and
// clamps at 1 for anything beyond full deflection -- the loop is slew-limited there and no
// deflection exists that would go faster.
inline float plant_deflection_for_rate_frac(float rate_frac) {
    if (!(rate_frac > 0.0f)) return 0.0f;
    if (rate_frac >= kPlantCurve[kPlantCurveN - 1].rate_frac) return 1.0f;
    for (int i = 1; i < kPlantCurveN; ++i) {
        if (rate_frac <= kPlantCurve[i].rate_frac) {
            const float r0 = kPlantCurve[i - 1].rate_frac, r1 = kPlantCurve[i].rate_frac;
            const float d0 = kPlantCurve[i - 1].deflection, d1 = kPlantCurve[i].deflection;
            const float span = r1 - r0;
            const float t = (span > 1e-6f) ? ((rate_frac - r0) / span) : 0.0f;
            return d0 + t * (d1 - d0);
        }
    }
    return 1.0f;
}

// Frame-rate-independent blend factor for an exponential moving average.
//
// Every filter here used to apply a fixed fraction PER CALL, which is not a filter setting at all
// -- it is a filter setting multiplied by the frame rate. The same number smoothed roughly three
// times harder at 30 fps than at 90, so tuning done in a quiet corridor stopped holding in a
// firefight, and the aim law and the reticule (which run on different loops) disagreed with each
// other while reading identical config.
//
// tau_ms states the intent directly: the filter closes ~63% of the remaining gap in that many
// milliseconds, at any frame rate. Larger = smoother and laggier; 0 = no filtering at all.
//
// Pass the time since THIS filter last advanced, which is not always the frame dt -- the aim law's
// rate filters only step on change boundaries, so they pass their accumulated span instead.
inline float ema_alpha(float tau_ms, float dt) {
    if (tau_ms <= 0.0f) return 1.0f;   // no smoothing: take the new value whole
    if (!(dt > 0.0f))   return 0.0f;   // no time passed (first call, or a stall): do not move
    return 1.0f - std::exp(-dt / (tau_ms * 0.001f));
}

// Pitch is clamped just shy of +-90: at exactly +-90 a quaternion->rotator conversion loses yaw and
// roll to gimbal lock, and the rig mirrors.
constexpr float SINGULARITY = 0.4999995f;

struct Vec3 { float x, y, z; };
struct Quat { float x, y, z, w; };
struct Mat3 { float m[3][3]; };

inline float wrap180(float d) {
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// VR-space forward: (0,0,-1) rotated by q. Stays in VR space deliberately -- the parallax
// construction mixes it with VR-space POSITIONS, and mixing frames here is a silent bug.
inline Vec3 quat_forward(const Quat& q) {
    return Vec3{
        -2.0f * (q.w * q.y + q.x * q.z),
         2.0f * (q.w * q.x - q.y * q.z),
        -(1.0f - 2.0f * (q.x * q.x + q.y * q.y))
    };
}

// ---- quaternion algebra, needed to express the rig in its PARENT's frame.
//
// A relative transform composes on top of its parent, and the rig's parent is rotated by the game's
// aim. So neither of the two simpler shapes is right:
//   * ABSOLUTE controller orientation  -> counted twice (parent aim + our absolute), and the mesh
//     stops matching the aim ray, so shots leave the barrel wrong.
//   * AIM ERROR                        -> ~0 once the loop converges, so the rig sits neutral and
//     contributes no roll and no translation at all.
// The correct value is the controller expressed IN THE PARENT FRAME:
//     q_rel = inverse(q_parent) * q_controller
// which keeps the gun matching the hand in WORLD space no matter what the parent does -- full roll,
// no double count, and the mesh stays aligned with the ray the aim loop is steering towards.
inline Quat quat_mul(const Quat& a, const Quat& b) {
    return Quat{
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    };
}

// Unit quaternion => conjugate is the inverse.
inline Quat quat_conj(const Quat& q) { return Quat{-q.x, -q.y, -q.z, q.w}; }

inline Vec3 quat_rotate(const Quat& q, const Vec3& v) {
    const Vec3 u{q.x, q.y, q.z};
    const Vec3 uv{u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x};
    const Vec3 uuv{u.y * uv.z - u.z * uv.y, u.z * uv.x - u.x * uv.z, u.x * uv.y - u.y * uv.x};
    return Vec3{
        v.x + 2.0f * (q.w * uv.x + uuv.x),
        v.y + 2.0f * (q.w * uv.y + uuv.y),
        v.z + 2.0f * (q.w * uv.z + uuv.z)
    };
}

// UE rotator -> FQuat, matching UE's ZYX (yaw, pitch, roll) composition order.
//
// !!! EXACTLY UE's FRotator::Quaternion(). This MUST be the precise inverse of quat_to_rotator() or
// calibration cannot work: the solve produces a quaternion, stores it as a rotator, and rebuilds it
// next frame -- any mismatch reappears as a jump at that moment.
// (Beware: transposing yaw and roll in X and Z looks half-right because Y and W are symmetric under
// that swap; the symptom is the weapon snapping by an orientation-dependent amount.)
inline Quat rotator_to_quat(float pitch, float yaw, float roll) {
    const float hp = pitch * 0.5f * DEG2RAD, hy = yaw * 0.5f * DEG2RAD, hr = roll * 0.5f * DEG2RAD;
    const float sp = std::sin(hp), cp = std::cos(hp);
    const float sy = std::sin(hy), cy = std::cos(hy);
    const float sr = std::sin(hr), cr = std::cos(hr);
    return Quat{
        cr * sp * sy - sr * cp * cy,
        -cr * sp * cy - sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy
    };
}

// ---- defined in Math.cpp (calibration-path only, not per-frame) ----

Mat3 quat_to_mat3(const Quat& q);
Mat3 mat3_sub(const Mat3& a, const Mat3& b);

// Solve A x = b in the least-squares sense, Tikhonov-regularised.
bool solve_lstsq(const Mat3& A, const Vec3& b, float lambda, Vec3* out);

// Quaternion -> UE rotator. The exact inverse of rotator_to_quat().
void quat_to_rotator(float X, float Y, float Z, float W, float* pitch, float* yaw, float* roll);

} // namespace halo
