// Aim convergence. The doctrine, the derivation and the reason C cancels are in AimConverge.hpp.

#include "AimConverge.hpp"
#include "Config.hpp"

#include <cmath>

namespace halo {

std::atomic<float> g_eye_delta_x{0.0f}, g_eye_delta_y{0.0f}, g_eye_delta_z{0.0f};
std::atomic<bool>  g_have_eye_delta{false};

namespace {

// Per-eye state. Touched only from the render thread, inside one callback pair, so plain floats are
// correct here -- the atomics are the publication boundary, not the working storage.
struct EyeSample {
    float pre[3]  = {0.0f, 0.0f, 0.0f};
    float d[3]    = {0.0f, 0.0f, 0.0f};
    bool  have_pre  = false;
    bool  have_delta = false;
};
EyeSample g_eye[2];

std::atomic<float> g_range_cm{0.0f};

} // namespace

void aim_converge_note_pre(int view_index, float x, float y, float z) {
    const int i = (view_index & 1);
    g_eye[i].pre[0] = x; g_eye[i].pre[1] = y; g_eye[i].pre[2] = z;
    g_eye[i].have_pre = true;
}

void aim_converge_note_post(int view_index, float x, float y, float z) {
    const int i = (view_index & 1);
    if (!g_eye[i].have_pre) return;
    g_eye[i].d[0] = x - g_eye[i].pre[0];
    g_eye[i].d[1] = y - g_eye[i].pre[1];
    g_eye[i].d[2] = z - g_eye[i].pre[2];
    g_eye[i].have_delta = true;

    // CYCLOPEAN EYE. Each eye sits half an IPD off the head's centre line, and that offset is real
    // -- it IS part of this eye's view position -- but it is not part of where the PLAYER is. Taking
    // one eye would bake ~3 cm of lateral offset into the correction permanently, which at a 5 m
    // target is a third of a degree of standing aim bias for nothing. Averaging cancels it exactly.
    // Until the second eye has been seen, use the one we have rather than publishing nothing.
    const bool both = g_eye[0].have_delta && g_eye[1].have_delta;
    const float k = both ? 0.5f : 1.0f;
    const int   a = both ? 0 : i;
    const int   b = both ? 1 : i;

    g_eye_delta_x = (g_eye[a].d[0] + g_eye[b].d[0]) * k;
    g_eye_delta_y = (g_eye[a].d[1] + g_eye[b].d[1]) * k;
    g_eye_delta_z = (g_eye[a].d[2] + g_eye[b].d[2]) * k;
    g_have_eye_delta = true;
}

void aim_converge_reset() {
    g_range_cm = 0.0f;
}

float aim_converge_range() {
    return g_range_cm.load();
}

void aim_converge_feed(float hit_cm, bool hit, float dt) {
    // A MISS HOLDS THE LAST RANGE rather than jumping to the trace length. Sky has no range, so any
    // number chosen for it is invented -- and because the correction goes as 1/range, "invented as
    // very far" means the aim swings by the whole correction every time the reticule crosses a
    // skyline. Holding is both calmer and more honest: nothing was measured, so nothing changes.
    if (!hit) return;
    // Floored, not just validated. The correction goes as 1/range, so a range approaching zero --
    // the muzzle pressed into a wall, or the player's head physically inside geometry -- sends it to
    // infinity, and the sanity rail then DECLINES, which makes the aim flick between corrected and
    // uncorrected as you close on a surface. A floor keeps it continuous, and at contact range there
    // is nothing to aim at anyway.
    if (!std::isfinite(hit_cm) || hit_cm < 30.0f) return;

    const float cur = g_range_cm.load();
    if (!(cur > 0.0f) || !(dt > 0.0f)) { g_range_cm = hit_cm; return; }

    const float a = ema_alpha(g_cfg.aim_converge_tau_ms, dt);
    g_range_cm = cur + (hit_cm - cur) * a;
}

bool aim_converge_delta(Vec3* out) {
    if (!g_have_eye_delta.load()) return false;
    *out = Vec3{g_eye_delta_x.load(), g_eye_delta_y.load(), g_eye_delta_z.load()};
    return true;
}

bool aim_converge_engaged() {
    if (!g_cfg.aim_converge) return false;
    // THE RANGE COMES FROM THE RETICULE'S TRACE and from nowhere else, so switching that off must
    // switch this off too. Checked here rather than trusted: without it, turning the trace off
    // would leave the last measured range LATCHED and quietly bend aim toward a wall that was
    // measured minutes ago -- a stale input is worse than a missing one.
    if (!g_cfg.aim_reticule || !g_cfg.aim_reticule_trace) return false;
    if (!(g_range_cm.load() > 1.0f)) return false;

    Vec3 d{};
    if (!aim_converge_delta(&d)) return false;

    // BELOW THE THRESHOLD, DO NOTHING AT ALL. Not "apply a tiny correction" -- nothing, so the
    // leashed default keeps the exact numbers it shipped with and this feature cannot be blamed for
    // a drift someone sees in it. The threshold is in cm of eye-to-camera divergence.
    const float m = d.x * d.x + d.y * d.y + d.z * d.z;
    const float t = g_cfg.aim_converge_min_cm;
    return m > (t * t);
}

bool aim_converge_apply(float* yaw, float* pitch) {
    if (!aim_converge_engaged()) return false;

    Vec3 d{};
    if (!aim_converge_delta(&d)) return false;
    const float range = g_range_cm.load();

    const float cp = std::cos(*pitch * DEG2RAD);
    const Vec3  u{cp * std::cos(*yaw * DEG2RAD),
                  cp * std::sin(*yaw * DEG2RAD),
                  std::sin(*pitch * DEG2RAD)};

    // aim = normalize(delta + u * range). See the header: the shot origin cancels out of this.
    const Vec3 t{d.x + u.x * range, d.y + u.y * range, d.z + u.z * range};
    const float len = std::sqrt(t.x * t.x + t.y * t.y + t.z * t.z);
    if (!(len > 1.0f) || !std::isfinite(len)) return false;

    const float ny = std::atan2(t.y, t.x) * RAD2DEG;
    const float np = std::asin(clampf(t.z / len, -1.0f, 1.0f)) * RAD2DEG;
    if (!std::isfinite(ny) || !std::isfinite(np)) return false;

    // SANITY RAIL. A correction this large means the range or the delta is wrong, not that the
    // player is pointing somewhere extreme -- and a wrong large correction in VR is a weapon that
    // fires sideways. Decline instead, which degrades to the uncorrected setpoint.
    const float lim = g_cfg.aim_converge_max_deg;
    if (std::fabs(wrap180(ny - *yaw)) > lim || std::fabs(np - *pitch) > lim) return false;

    *yaw = ny; *pitch = np;
    return true;
}

} // namespace halo
