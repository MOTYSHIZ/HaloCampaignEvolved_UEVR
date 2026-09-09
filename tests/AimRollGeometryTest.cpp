// Out-of-tree check of the wrist-roll cancellation geometry (aimrollfix). No game, no headset, no
// engine -- it includes the REAL src\Math.hpp and exercises the functions the plugin actually runs.
//
// WHY IT LIVES IN tests\ AND NOT src\test\: both build scripts do
//     Get-ChildItem $srcDir -Filter *.cpp -Recurse
// so ANY .cpp under src\ is compiled into halo_vr.dll. A test file with a main() placed there links
// into the DLL and breaks the contributor build and CI, not just this harness. Keep it out here.
//
// WHY IT COMPILES THE REAL HEADER rather than copying the maths: a copied helper passes forever
// while the shipping one rots. If Math.hpp changes shape, this stops compiling, which is the point.
//
// Run it with scripts\Verify-AimRollGeometry.ps1 after touching quat_up, rotate_about_axis or
// wrist_twist_upright.

#include "../src/Math.hpp"

#include <cstdio>
#include <cstdlib>

using halo::Vec3;
using halo::Quat;
using halo::DEG2RAD;
using halo::RAD2DEG;

// ---- scaffolding ------------------------------------------------------------------------------

static int g_fail = 0;
static void check(bool ok, const char* what, double got, double tol) {
    if (!ok) { std::printf("  FAIL  %-52s (got %.8f, tol %.8f)\n", what, got, tol); ++g_fail; }
}

static Quat axis_angle(const Vec3& a, float rad) {
    const float h = rad * 0.5f, s = std::sin(h);
    return Quat{a.x * s, a.y * s, a.z * s, std::cos(h)};
}
static float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static Vec3 norm(const Vec3& v) {
    const float n = std::sqrt(dot(v, v));
    return Vec3{v.x / n, v.y / n, v.z / n};
}

// ANGLE VIA THE CHORD, NOT acos. acos is ill-conditioned near 1 -- its error grows as 1/sin(theta),
// so in float it cannot resolve below about 0.02 deg and reports plain rounding as a real angle.
// An earlier draft of this test "failed" at 0.079 deg for exactly that reason and the geometry was
// fine. The chord is linear in the error, so it separates float noise (1e-5 deg) from a real bias.
static float ang_deg(const Vec3& a, const Vec3& b) {
    const float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    const float c = std::sqrt(dx * dx + dy * dy + dz * dz);
    return 2.0f * std::asin(halo::clampf(c * 0.5f, -1.0f, 1.0f)) * RAD2DEG;
}

static float frand(float lo, float hi) {
    return lo + (hi - lo) * ((float)std::rand() / (float)RAND_MAX);
}

// A controller orientation: yaw about world up, then pitch, then a GENUINE wrist roll -- a rotation
// about the pose's own forward, which is what the hand physically does.
static Quat pose(float yaw_deg, float pitch_deg, float roll_deg) {
    const Quat qy = axis_angle(Vec3{0, 1, 0}, yaw_deg * DEG2RAD);
    const Quat qp = halo::quat_mul(qy, axis_angle(Vec3{1, 0, 0}, pitch_deg * DEG2RAD));
    return halo::quat_mul(axis_angle(norm(halo::quat_forward(qp)), roll_deg * DEG2RAD), qp);
}

// The corrected aim direction, built exactly as derive_ctrl_angles() builds it -- including the
// direction blend, which is the part with a trap in it. See test 8.
static Vec3 corrected_aim(const Quat& q_grip, const Quat& tilt, float strength, float vert_deg) {
    const Vec3 A = halo::quat_forward(halo::quat_mul(q_grip, tilt));
    const Vec3 axis = norm(halo::quat_forward(q_grip));
    float tw = 0.0f, fade = 0.0f;
    if (!halo::wrist_twist_upright(q_grip, axis, vert_deg, &tw, &fade)) return A;
    const Vec3  flat = halo::rotate_about_axis(A, axis, -tw);
    const float s    = halo::clampf(strength * fade, 0.0f, 1.0f);
    Vec3 out{A.x + (flat.x - A.x) * s, A.y + (flat.y - A.y) * s, A.z + (flat.z - A.z) * s};
    const float ol = std::sqrt(dot(out, out));
    return (ol > 1e-4f) ? Vec3{out.x / ol, out.y / ol, out.z / ol} : flat;
}

// The form this code had FIRST, kept only so test 8 can show what it did wrong. Scaling the ANGLE
// looks equivalent and is not: see the comment on test 8.
static Vec3 corrected_aim_by_angle(const Quat& q_grip, const Quat& tilt, float strength,
                                   float vert_deg) {
    const Vec3 A = halo::quat_forward(halo::quat_mul(q_grip, tilt));
    const Vec3 axis = norm(halo::quat_forward(q_grip));
    float tw = 0.0f, fade = 0.0f;
    if (!halo::wrist_twist_upright(q_grip, axis, vert_deg, &tw, &fade)) return A;
    return halo::rotate_about_axis(A, axis, -tw * strength * fade);
}

// ---- the checks -------------------------------------------------------------------------------

int main() {
    std::srand(20260908);
    const float VERT = 15.0f;   // the shipped aim_roll_vert_deg default

    // The fixed aim-vs-handle tilt. ~35 deg is the figure BLAM_AIM_FINDINGS.md works from.
    const Quat TILT = axis_angle(Vec3{1, 0, 0}, 35.0f * DEG2RAD);

    std::printf("1. quat_up is perpendicular to quat_forward (400 random poses)\n");
    for (int i = 0; i < 400; ++i) {
        const Quat q = pose(frand(-180, 180), frand(-80, 80), frand(-180, 180));
        const float d = dot(halo::quat_forward(q), halo::quat_up(q));
        check(std::fabs(d) < 2e-3f, "forward . up == 0", d, 2e-3);
    }

    std::printf("2. the measured twist tracks a wrist roll one-for-one (200 poses)\n");
    for (int i = 0; i < 200; ++i) {
        const float yaw = frand(-120, 120), pit = frand(-55, 55);
        const float r0 = frand(-60, 60), dr = frand(-70, 70);
        const Vec3 ax = norm(halo::quat_forward(pose(yaw, pit, 0)));
        float t0 = 0, t1 = 0, f0 = 0, f1 = 0;
        halo::wrist_twist_upright(pose(yaw, pit, r0),      ax, VERT, &t0, &f0);
        halo::wrist_twist_upright(pose(yaw, pit, r0 + dr), ax, VERT, &t1, &f1);
        check(std::fabs(halo::wrap180((t1 - t0) * RAD2DEG - dr)) < 0.05f,
              "d(twist) == d(roll)", halo::wrap180((t1 - t0) * RAD2DEG - dr), 0.05);
    }

    std::printf("3. THE POINT: the corrected aim direction does not move when the wrist rolls\n");
    float worst_fixed = 0.0f, worst_raw = 0.0f;
    for (int i = 0; i < 300; ++i) {
        const float yaw = frand(-120, 120), pit = frand(-50, 50);
        const Vec3 ref_fix = corrected_aim(pose(yaw, pit, 0.0f), TILT, 1.0f, VERT);
        const Vec3 ref_raw = halo::quat_forward(halo::quat_mul(pose(yaw, pit, 0.0f), TILT));
        for (float r = -90.0f; r <= 90.0f; r += 15.0f) {
            const Quat q = pose(yaw, pit, r);
            const float e_fix = ang_deg(ref_fix, corrected_aim(q, TILT, 1.0f, VERT));
            const float e_raw = ang_deg(ref_raw, halo::quat_forward(halo::quat_mul(q, TILT)));
            if (e_fix > worst_fixed) worst_fixed = e_fix;
            if (e_raw  > worst_raw)  worst_raw  = e_raw;
        }
    }
    // 0.01 deg is far above float rounding through a dozen quaternion products and far below
    // anything a hand could hold to.
    check(worst_fixed < 0.01f, "worst corrected drift across +-90 deg of roll", worst_fixed, 0.01);
    std::printf("     corrected %.6f deg  |  uncorrected %.2f deg  |  %.0fx better\n",
                worst_fixed, worst_raw, worst_raw / (worst_fixed > 0.0f ? worst_fixed : 1e-9f));

    std::printf("4. control: the test is sensitive -- uncorrected really does swing\n");
    check(worst_raw > 20.0f, "uncorrected drift is large", worst_raw, 20.0);

    std::printf("5. null control: rotating a direction about ITSELF is the identity\n");
    for (int i = 0; i < 100; ++i) {
        const Quat q = pose(frand(-120, 120), frand(-50, 50), frand(-90, 90));
        const Vec3 axis = norm(halo::quat_forward(q));
        float tw = 0, fade = 0;
        halo::wrist_twist_upright(q, axis, VERT, &tw, &fade);
        const Vec3 out = halo::rotate_about_axis(axis, axis, -tw * fade);
        check(ang_deg(axis, out) < 1e-3f, "rot(v, v, t) == v", ang_deg(axis, out), 1e-3);
    }

    std::printf("6. strength scales the correction\n");
    {
        const Quat q = pose(30.0f, -12.0f, 55.0f);
        const Vec3 a0 = corrected_aim(q, TILT, 0.0f, VERT);
        const float half = ang_deg(a0, corrected_aim(q, TILT, 0.5f, VERT));
        const float full = ang_deg(a0, corrected_aim(q, TILT, 1.0f, VERT));
        std::printf("     0 -> 0.5 = %.3f deg, 0 -> 1.0 = %.3f deg\n", half, full);
        check(full > 1.0f, "full correction is non-trivial here", full, 1.0);
        // Not exactly half: equal rotations about the axis do not cut equal arcs on the cone.
        check(std::fabs(half / full - 0.5f) < 0.05f, "half strength is about half the arc",
              half / full, 0.05);
    }

    std::printf("7. the pole fades out instead of exploding\n");
    for (float elev = 0.0f; elev <= 90.0f; elev += 10.0f) {
        const Quat q = pose(0.0f, -elev, 40.0f);
        const Vec3 axis = norm(halo::quat_forward(q));
        float tw = 0, fade = 0;
        const bool ok = halo::wrist_twist_upright(q, axis, VERT, &tw, &fade);
        std::printf("     elevation %5.1f -> ok=%d fade=%.3f twist=%7.2f deg\n",
                    elev, (int)ok, fade, tw * RAD2DEG);
        check(fade >= 0.0f && fade <= 1.0f, "fade stays inside [0,1]", fade, 1.0);
        if (elev <= 70.0f) check(fade > 0.999f, "full strength below 75 deg elevation", fade, 1.0);
    }

    // REGRESSION. Found by the live instrument on 2026-09-08, not by this harness, which had been
    // testing only +-90 deg of roll at full strength -- the one corner where the bug is invisible.
    //
    // Rotating by an angle is 2*pi-periodic, so at multiplier 1 the twist crossing +-180 lands on
    // the same vector and nothing happens. SCALE that angle and it no longer does: the applied
    // rotation jumps by 2*180*(1-s) at the wrap. And the multiplier is below 1 almost always,
    // because the pole `fade` is a factor in it. Live data: |twist| passes 150 deg in 1% of
    // samples, so this is ordinary play, not a corner.
    std::printf("8. regression: no seam where the twist wraps through +-180\n");
    {
        // Handle elevation 80 deg, so fade < 1 with the shipped band and the multiplier is scaled.
        const float ELEV = 80.0f, STRENGTH = 1.0f, STEP = 0.5f;
        float worst_blend = 0.0f, worst_angle = 0.0f;
        Vec3 prev_b = corrected_aim(pose(0.0f, -ELEV, -180.0f), TILT, STRENGTH, VERT);
        Vec3 prev_a = corrected_aim_by_angle(pose(0.0f, -ELEV, -180.0f), TILT, STRENGTH, VERT);
        for (float r = -180.0f + STEP; r <= 180.0f; r += STEP) {
            const Quat q = pose(0.0f, -ELEV, r);
            const Vec3 b = corrected_aim(q, TILT, STRENGTH, VERT);
            const Vec3 a = corrected_aim_by_angle(q, TILT, STRENGTH, VERT);
            const float db = ang_deg(prev_b, b), da = ang_deg(prev_a, a);
            if (db > worst_blend) worst_blend = db;
            if (da > worst_angle) worst_angle = da;
            prev_b = b; prev_a = a;
        }
        std::printf("     worst step over a %.1f deg roll increment:  blend %.3f deg  |  "
                    "angle-scaled %.1f deg\n", STEP, worst_blend, worst_angle);
        // A half-degree of roll can never move the aim more than a couple of degrees.
        check(worst_blend < 2.0f, "direction blend is continuous through the wrap",
              worst_blend, 2.0);
        // The control: the old form must still show the seam, or this test proves nothing.
        check(worst_angle > 30.0f, "control -- angle scaling really does jump", worst_angle, 30.0);
    }

    std::printf("\n%s  (%d failure%s)\n", g_fail ? "FAILED" : "ALL CHECKS PASSED",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
