#include "features/aimbore/AimBore.hpp"

#include "Config.hpp"
#include "Math.hpp"   // clampf

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include "Math.hpp"
#include "core/PalettePose.hpp"
#include "core/Services.hpp"

#include <cmath>

namespace halo {

namespace {
bool aim_bore_parse_key(const char* key, const char* val, double v) {
    (void)val; (void)v;
    if (_stricmp(key, "aimbore") == 0) { g_cfg.aim_bore = (int)clampf((float)v, 0.0f, 3.0f); return true; }
    if (_stricmp(key, "aimboreaxis") == 0) { sscanf_s(val, "%f,%f", &g_cfg.aim_bore_axis[0], &g_cfg.aim_bore_axis[1]); return true; }
    return false;
}

bool aim_bore_enabled() { return g_cfg.aim_bore != 0; }

// The aim derivation's palette branch (features_aim_forward), before the palette's two-handed blend: true =
// the forward is the drawn barrel. Only reached while the palette weapon owns the aim.
bool aim_bore_forward(const Quat& q_src, Vec3* fwd_out) {
    Vec3& fwd = *fwd_out;
    // ---- AIMBORE (aimbore=1): the aim IS the drawn barrel. The palette renders the weapon as
    // f(aim-fixed hand) * G * W in UE convention, f(q) = (-q.z, q.x, q.y, -q.w). f is a
    // homomorphism, so the same pose in this XR frame is hand * f^-1(G) * f^-1(W), with
    // f^-1(u) = (u.y, u.z, -u.x, -u.w) (checked numerically to 1e-15 over 2000 random poses; the AR
    // trim alone gives +2.000 deg pitch, the measured barrel-over-aim). The gun is not touched.
    // The two-handed hold stays ONE rotation for the whole assembly, as on the weapon: the shortest
    // arc the blend puts on the hand forward is applied to the bore forward.
    bool bore_done = false;
    if (g_cfg.aim_bore != 0) {
        float gq4[4], wq4[4];
        if (palette_pose_trim_rotations(gq4, wq4)) {
            auto qn = [](Quat q) {
                const float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
                return (n > 1.0e-6f && std::isfinite(n)) ? Quat{q.x / n, q.y / n, q.z / n, q.w / n}
                                                         : Quat{0.0f, 0.0f, 0.0f, 1.0f};
            };
            const Quat gx = qn(Quat{gq4[1], gq4[2], -gq4[0], -gq4[3]});
            const Quat wx = qn(Quat{wq4[1], wq4[2], -wq4[0], -wq4[3]});
            // The roll trim sits between the grip rotation and the weapon trim, as in the pullback.
            const float rh = palette_pose_roll_trim_deg() * 0.5f * DEG2RAD;
            const Quat rx = qn(Quat{0.0f, 0.0f, -std::sin(rh), -std::cos(rh)});
            // Barrel axis in the trimmed pose frame, UE convention, then mapped to this frame
            // (x = ue.y, y = ue.z, z = -ue.x). Mode 1 is the pose forward (+X).
            Vec3 b_ue{1.0f, 0.0f, 0.0f};
            Vec3 live_b{};
            const bool live_axis = (g_cfg.aim_bore == 3) && palette_pose_barrel_axis(&live_b);
            if (live_axis) {
                b_ue = live_b;
            } else if (g_cfg.aim_bore >= 2) {
                const float bp = g_cfg.aim_bore_axis[0] * DEG2RAD, byw = g_cfg.aim_bore_axis[1] * DEG2RAD;
                b_ue = Vec3{std::cos(bp) * std::cos(byw), std::cos(bp) * std::sin(byw), std::sin(bp)};
            }
            const Vec3 b_xr{b_ue.y, b_ue.z, -b_ue.x};
            Vec3 bore = quat_rotate(quat_mul(quat_mul(quat_mul(q_src, gx), rx), wx), b_xr);
            {
                const float bl = std::sqrt(bore.x * bore.x + bore.y * bore.y + bore.z * bore.z);
                if (bl > 1.0e-4f) bore = Vec3{bore.x / bl, bore.y / bl, bore.z / bl};
            }
            Vec3 blended = fwd;
            if (palette_pose_two_hand_blend(&blended)) {
                const float d = fwd.x * blended.x + fwd.y * blended.y + fwd.z * blended.z;
                if (d > -0.99f) {
                    const Quat arc = qn(Quat{fwd.y * blended.z - fwd.z * blended.y,
                                             fwd.z * blended.x - fwd.x * blended.z,
                                             fwd.x * blended.y - fwd.y * blended.x, 1.0f + d});
                    bore = quat_rotate(arc, bore);
                }
            }
            if (std::isfinite(bore.x) && std::isfinite(bore.y) && std::isfinite(bore.z)) {
                fwd = bore;
                bore_done = true;
            }
        }
    }
    return bore_done;
}
}  // namespace

constinit const FeatureHooks kAimBoreHooks{
    .key      = "aimbore",
    .parse_key = &aim_bore_parse_key,
    .aim_bore_forward = &aim_bore_forward,
    .enabled  = &aim_bore_enabled,
    .services = 0,
};

} // namespace halo
