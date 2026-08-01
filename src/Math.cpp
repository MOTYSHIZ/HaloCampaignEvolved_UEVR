// Calibration-path maths. Not called per frame, so these live in a translation unit rather than
// being inlined in the header. See Math.hpp for the per-frame helpers and the no-plugin-state rule.

#include "Math.hpp"

namespace halo {

Mat3 quat_to_mat3(const Quat& q) {
    const float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    const float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    const float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    Mat3 r{};
    r.m[0][0] = 1.0f - 2.0f * (yy + zz); r.m[0][1] = 2.0f * (xy - wz);        r.m[0][2] = 2.0f * (xz + wy);
    r.m[1][0] = 2.0f * (xy + wz);        r.m[1][1] = 1.0f - 2.0f * (xx + zz); r.m[1][2] = 2.0f * (yz - wx);
    r.m[2][0] = 2.0f * (xz - wy);        r.m[2][1] = 2.0f * (yz + wx);        r.m[2][2] = 1.0f - 2.0f * (xx + yy);
    return r;
}

Mat3 mat3_sub(const Mat3& a, const Mat3& b) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.m[i][j] = a.m[i][j] - b.m[i][j];
    return r;
}

// Solve A x = b in the least-squares sense, Tikhonov-regularised.
//
// REGULARISATION IS NOT OPTIONAL HERE. A = R1 - R2 is SINGULAR along the axis the two samples
// share: a rotation difference tells you nothing about displacement parallel to its own axis. Two
// wrist rolls about the same axis therefore leave one component of the pivot undetermined, and a
// plain inverse would explode on it. lambda bounds that component instead of amplifying noise into
// it -- the price is that the undetermined direction stays near its prior (zero) rather than
// becoming garbage. Rolling AND pitching between samples is what actually pins all three axes.
bool solve_lstsq(const Mat3& A, const Vec3& b, float lambda, Vec3* out) {
    float N[3][3] = {};
    float r[3] = {};
    const float bv[3] = {b.x, b.y, b.z};

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += A.m[k][i] * A.m[k][j];
            N[i][j] = s + ((i == j) ? lambda : 0.0f);
        }
        float s = 0.0f;
        for (int k = 0; k < 3; ++k) s += A.m[k][i] * bv[k];
        r[i] = s;
    }

    const float c00 = N[1][1] * N[2][2] - N[1][2] * N[2][1];
    const float c01 = N[1][2] * N[2][0] - N[1][0] * N[2][2];
    const float c02 = N[1][0] * N[2][1] - N[1][1] * N[2][0];
    const float det = N[0][0] * c00 + N[0][1] * c01 + N[0][2] * c02;
    if (!std::isfinite(det) || std::fabs(det) < 1e-9f) return false;

    const float inv[3][3] = {
        { c00 / det, (N[0][2] * N[2][1] - N[0][1] * N[2][2]) / det, (N[0][1] * N[1][2] - N[0][2] * N[1][1]) / det },
        { c01 / det, (N[0][0] * N[2][2] - N[0][2] * N[2][0]) / det, (N[0][2] * N[1][0] - N[0][0] * N[1][2]) / det },
        { c02 / det, (N[0][1] * N[2][0] - N[0][0] * N[2][1]) / det, (N[0][0] * N[1][1] - N[0][1] * N[1][0]) / det }
    };

    out->x = inv[0][0] * r[0] + inv[0][1] * r[1] + inv[0][2] * r[2];
    out->y = inv[1][0] * r[0] + inv[1][1] * r[1] + inv[1][2] * r[2];
    out->z = inv[2][0] * r[0] + inv[2][1] * r[1] + inv[2][2] * r[2];
    return std::isfinite(out->x) && std::isfinite(out->y) && std::isfinite(out->z);
}

// Quaternion -> UE rotator. The exact inverse of rotator_to_quat() in the header; see the warning
// there about why the two must stay in lockstep.
void quat_to_rotator(float X, float Y, float Z, float W, float* pitch, float* yaw, float* roll) {
    const float sing = Z * X - W * Y;
    const float yawY = 2.0f * (W * Z + X * Y);
    const float yawX = 1.0f - 2.0f * (Y * Y + Z * Z);

    if (sing < -SINGULARITY) {
        *pitch = -90.0f;
        *yaw   = std::atan2(yawY, yawX) * RAD2DEG;
        *roll  = -*yaw - (2.0f * std::atan2(X, W) * RAD2DEG);
    } else if (sing > SINGULARITY) {
        *pitch = 90.0f;
        *yaw   = std::atan2(yawY, yawX) * RAD2DEG;
        *roll  = *yaw - (2.0f * std::atan2(X, W) * RAD2DEG);
    } else {
        *pitch = std::asin(clampf(2.0f * sing, -1.0f, 1.0f)) * RAD2DEG;
        *yaw   = std::atan2(yawY, yawX) * RAD2DEG;
        *roll  = std::atan2(-2.0f * (W * X + Y * Z), 1.0f - 2.0f * (X * X + Y * Y)) * RAD2DEG;
    }
}

} // namespace halo
