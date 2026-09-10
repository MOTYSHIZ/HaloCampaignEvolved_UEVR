// Out-of-tree check of the roll-geometry helpers in src\Math.hpp -- quat_up, rotate_about_axis and
// wrist_twist_upright -- and an EXECUTABLE RECORD of why the roll-cancellation built on them was
// withdrawn. No game, no headset, no engine: it includes the real header and runs the real code.
//
// WHY IT LIVES IN tests\ AND NOT src\test\: both build scripts do
//     Get-ChildItem $srcDir -Filter *.cpp -Recurse
// so ANY .cpp under src\ is compiled into halo_vr.dll. A test file with a main() placed there links
// into the DLL and breaks the contributor build and CI, not just this harness. Keep it out here.
//
// WHY IT COMPILES THE REAL HEADER rather than copying the maths: a copied helper passes forever
// while the shipping one rots. If Math.hpp changes shape, this stops compiling, which is the point.
//
// THE RECORD, in two tests that must both pass:
//   * A roll about the HANDLE sweeps the aim on a 60 deg cone, and rotating the aim back about the
//     handle by the twist against upright cancels it to float precision. The maths was right.
//   * A roll about the BORE -- which is what a wrist holding a pistol grip actually does, measured
//     in a headset on 2026-09-10 -- leaves the raw aim exactly where it was, and that same
//     cancellation then MOVES it, by about 1.7x the roll. The physics was wrong, and the harness
//     that only ran the first test could not know. Neither can this one; it just refuses to let
//     the first fact be read without the second.
//
// Run it with scripts\Verify-AimRollGeometry.ps1 after touching any of the three helpers.

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
    if (!ok) { std::printf("  FAIL  %-56s (got %.8f, tol %.8f)\n", what, got, tol); ++g_fail; }
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

// A GRIP orientation: yaw about world up, then pitch, then a roll about its own forward.
static Quat pose(float yaw_deg, float pitch_deg, float roll_deg) {
    const Quat qy = axis_angle(Vec3{0, 1, 0}, yaw_deg * DEG2RAD);
    const Quat qp = halo::quat_mul(qy, axis_angle(Vec3{1, 0, 0}, pitch_deg * DEG2RAD));
    return halo::quat_mul(axis_angle(norm(halo::quat_forward(qp)), roll_deg * DEG2RAD), qp);
}

// The aim pose rides the grip pose rigidly. On Touch controllers the two forwards are 60.0 deg
// apart (measured, sd 0.00), the aim above the handle within the controller's symmetry plane.
static const float SEP_DEG = 60.0f;
static Quat aim_of(const Quat& grip) {
    return halo::quat_mul(grip, axis_angle(Vec3{1, 0, 0}, SEP_DEG * DEG2RAD));
}

// The withdrawn construction, verbatim in spirit: twist of the grip about the HANDLE against
// upright, aim rotated back about the handle by it.
static Vec3 handle_flattened_aim(const Quat& grip, float vert_deg) {
    const Vec3 A = halo::quat_forward(aim_of(grip));
    const Vec3 G = norm(halo::quat_forward(grip));
    float tw = 0.0f, fade = 0.0f;
    if (!halo::wrist_twist_upright(grip, G, vert_deg, &tw, &fade)) return A;
    return halo::rotate_about_axis(A, G, -tw * fade);
}

// ---- the checks -------------------------------------------------------------------------------

int main() {
    std::srand(20260908);
    const float VERT = 15.0f;

    std::printf("1. quat_up is perpendicular to quat_forward (400 random poses)\n");
    for (int i = 0; i < 400; ++i) {
        const Quat q = pose(frand(-180, 180), frand(-80, 80), frand(-180, 180));
        const float d = dot(halo::quat_forward(q), halo::quat_up(q));
        check(std::fabs(d) < 2e-3f, "forward . up == 0", d, 2e-3);
    }

    std::printf("2. wrist_twist_upright tracks a roll about its axis one-for-one (200 poses)\n");
    for (int i = 0; i < 200; ++i) {
        const float yaw = frand(-120, 120), pit = frand(-55, 55);
        const float r0 = frand(-60, 60), dr = frand(-70, 70);
        const Vec3 ax = norm(halo::quat_forward(pose(yaw, pit, 0)));
        float t0 = 0, t1 = 0, f0 = 0, f1 = 0;
        halo::wrist_twist_upright(pose(yaw, pit, r0),      ax, VERT, &t0, &f0);
        halo::wrist_twist_upright(pose(yaw, pit, r0 + dr), ax, VERT, &t1, &f1);
        const float err = halo::wrap180((t1 - t0) * RAD2DEG - dr);
        check(std::fabs(err) < 0.05f, "d(twist) == d(roll)", err, 0.05);
    }

    std::printf("3. rotate_about_axis: rotating a direction about ITSELF is the identity\n");
    for (int i = 0; i < 100; ++i) {
        const Quat q = pose(frand(-120, 120), frand(-50, 50), frand(-90, 90));
        const Vec3 axis = norm(halo::quat_forward(q));
        const Vec3 out = halo::rotate_about_axis(axis, axis, frand(-3.0f, 3.0f));
        check(ang_deg(axis, out) < 1e-3f, "rot(v, v, t) == v", ang_deg(axis, out), 1e-3);
    }

    std::printf("4. the pole fade winds down instead of exploding\n");
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

    // ---- THE RECORD ---------------------------------------------------------------------------
    // One body, aim level and pointing forward, handle 60 deg below it: the natural hold.
    const Quat HOLD = pose(0.0f, -SEP_DEG, 0.0f);
    const Vec3 A0   = halo::quat_forward(aim_of(HOLD));
    const Vec3 F0   = handle_flattened_aim(HOLD, VERT);
    check(std::fabs(std::asin(A0.y) * RAD2DEG) < 0.05f, "the hold aims level", std::asin(A0.y) * RAD2DEG, 0.05);

    std::printf("5. RECORD A: a roll about the HANDLE sweeps the aim on a 60 deg cone, and the\n"
                "   handle-frame cancellation removes it exactly\n");
    {
        float worst_raw = 0.0f, worst_flat = 0.0f;
        const Vec3 G = norm(halo::quat_forward(HOLD));
        for (float r = -90.0f; r <= 90.0f; r += 15.0f) {
            const Quat q = halo::quat_mul(axis_angle(G, r * DEG2RAD), HOLD);
            const float e_raw  = ang_deg(A0, halo::quat_forward(aim_of(q)));
            const float e_flat = ang_deg(F0, handle_flattened_aim(q, VERT));
            if (e_raw  > worst_raw)  worst_raw  = e_raw;
            if (e_flat > worst_flat) worst_flat = e_flat;
        }
        const float expect = 2.0f * std::asin(std::sin(SEP_DEG * DEG2RAD) * std::sin(45.0f * DEG2RAD)) * RAD2DEG;
        std::printf("     raw aim swept %.2f deg (cone predicts %.2f)  |  cancelled: %.6f deg\n",
                    worst_raw, expect, worst_flat);
        check(worst_raw > 0.9f * expect, "raw aim sweeps the predicted cone", worst_raw, expect);
        check(worst_flat < 0.01f, "handle-frame cancellation is exact for a HANDLE roll", worst_flat, 0.01);
    }

    std::printf("6. RECORD B: a roll about the BORE -- what the wrist actually does -- leaves the raw\n"
                "   aim alone, and that same cancellation then MOVES it\n");
    {
        float worst_raw = 0.0f, worst_flat = 0.0f, ratio_at_10 = 0.0f;
        for (float r = -20.0f; r <= 20.0f; r += 5.0f) {
            const Quat q = halo::quat_mul(axis_angle(norm(A0), r * DEG2RAD), HOLD);
            const float e_raw  = ang_deg(A0, halo::quat_forward(aim_of(q)));
            const float e_flat = ang_deg(F0, handle_flattened_aim(q, VERT));
            if (e_raw  > worst_raw)  worst_raw  = e_raw;
            if (e_flat > worst_flat) worst_flat = e_flat;
            if (std::fabs(r - 10.0f) < 0.01f) ratio_at_10 = e_flat / 10.0f;
        }
        std::printf("     raw aim moved %.6f deg  |  'cancelled' aim moved up to %.2f deg  |  %.2fx the roll at 10 deg\n",
                    worst_raw, worst_flat, ratio_at_10);
        check(worst_raw < 0.01f, "raw aim is invariant under a BORE roll", worst_raw, 0.01);
        check(ratio_at_10 > 1.4f, "handle-frame cancellation amplifies a bore roll (>1.4x)", ratio_at_10, 1.4);
    }

    std::printf("\n%s  (%d failure%s)\n", g_fail ? "FAILED" : "ALL CHECKS PASSED",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
