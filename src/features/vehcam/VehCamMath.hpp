#pragma once

// The rotation maths the vehicle cameras stand on, pure and header-only so it can be tested out of
// tree: UE's rotator <-> axes conversions, and the per-ride capture of a vehicle's own frame.

#include <cmath>

namespace halo::vehcammath {

// UE's FRotationMatrix for a rotator (degrees): the world directions of the rotated local X (forward),
// Y (right) and Z (up).
inline void rot_axes(double pitch, double yaw, double roll, double X[3], double Y[3], double Z[3]) {
    const double D2R = 0.01745329252;
    const double cp = std::cos(pitch * D2R), sp = std::sin(pitch * D2R);
    const double cy = std::cos(yaw * D2R),   sy = std::sin(yaw * D2R);
    const double cr = std::cos(roll * D2R),  sr = std::sin(roll * D2R);
    X[0] = cp * cy;                   X[1] = cp * sy;                   X[2] = sp;
    Y[0] = sr * sp * cy - cr * sy;    Y[1] = sr * sp * sy + cr * cy;    Y[2] = -sr * cp;
    Z[0] = -(cr * sp * cy + sr * sy); Z[1] = cy * sr - cr * sp * sy;    Z[2] = cr * cp;
}

// UE's FMatrix::Rotator() for an orthonormal frame given as its X / Y / Z axes (degrees out).
inline void rotator_from_axes(const double X[3], const double Y[3], const double Z[3],
                              double* pitch, double* yaw, double* roll) {
    const double R2D = 57.29577951;
    *pitch = std::atan2(X[2], std::sqrt(X[0] * X[0] + X[1] * X[1])) * R2D;
    *yaw   = std::atan2(X[1], X[0]) * R2D;
    double SX[3], SY[3], SZ[3];
    rot_axes(*pitch, *yaw, 0.0, SX, SY, SZ);
    *roll = std::atan2(Z[0] * SY[0] + Z[1] * SY[1] + Z[2] * SY[2],
                       Y[0] * SY[0] + Y[1] * SY[1] + Y[2] * SY[2]) * R2D;
}

// THE VEHICLE'S OWN FRAME, captured once per ride as C = R_mesh^T . V (row-major; row = mesh axis,
// column = vehicle axis forward/right/up): the vehicle's axes expressed along the chassis MESH's axes,
// so R_mesh . C rebuilds them every frame and they ride the vehicle rigidly. MX/MY/MZ are the mesh's
// world axes (rot_axes of its rotation), yaw_deg its yaw. A skeletal mesh's rotation carries its baked
// modelling axes; capture and apply go through the same basis, so that part cancels.
//
// SNAPPED TO THE MESH'S PRINCIPAL AXES. Captured as "level, at the mesh's heading" as-is, a vehicle
// mounted on a slope would bake the slope in, and every camera that tilts with it would carry a horizon
// tipped by the slope for the whole ride. Baked modelling offsets are whole quarter-turns, so the true
// up is the mesh axis nearest world up and the true forward the one nearest the heading. If the mount
// attitude is over 30 degrees off every quarter-turn the snap could pick the wrong axis, so the level
// capture is kept instead (*snapped = false) -- exact on flat ground, which is the old behaviour.
inline void capture_vehicle_frame(const double MX[3], const double MY[3], const double MZ[3], double yaw_deg,
                                  double C[9], bool* snapped) {
    const double D2R = 0.01745329252;
    const double LX[3] = { std::cos(yaw_deg * D2R), std::sin(yaw_deg * D2R), 0.0 };
    const double* const M[3] = { MX, MY, MZ };
    double f[3], u[3];   // the level forward and world up, along the mesh axes
    for (int r = 0; r < 3; ++r) {
        f[r] = M[r][0] * LX[0] + M[r][1] * LX[1] + M[r][2] * LX[2];
        u[r] = M[r][2];
    }
    int ku = 0;
    for (int k = 1; k < 3; ++k) if (std::fabs(u[k]) > std::fabs(u[ku])) ku = k;
    int kf = (ku == 0) ? 1 : 0;
    for (int k = 0; k < 3; ++k) if (k != ku && std::fabs(f[k]) > std::fabs(f[kf])) kf = k;
    const double lim = std::cos(30.0 * D2R);
    *snapped = std::fabs(u[ku]) > lim && std::fabs(f[kf]) > lim;
    double F[3] = { 0.0, 0.0, 0.0 }, U[3] = { 0.0, 0.0, 0.0 }, R[3];
    if (*snapped) {
        F[kf] = f[kf] > 0.0 ? 1.0 : -1.0;
        U[ku] = u[ku] > 0.0 ? 1.0 : -1.0;
    } else {
        for (int k = 0; k < 3; ++k) { F[k] = f[k]; U[k] = u[k]; }
    }
    // Right = Up x Forward (UE: X forward, Y right, Z up, and Z = X x Y component-wise).
    R[0] = U[1] * F[2] - U[2] * F[1];
    R[1] = U[2] * F[0] - U[0] * F[2];
    R[2] = U[0] * F[1] - U[1] * F[0];
    for (int r = 0; r < 3; ++r) { C[r * 3 + 0] = F[r]; C[r * 3 + 1] = R[r]; C[r * 3 + 2] = U[r]; }
}

// The vehicle's forward / right / up in the world this frame, from the mesh's axes and the captured C.
inline void vehicle_axes(const double C[9], const double MX[3], const double MY[3], const double MZ[3],
                         double F[3], double R[3], double U[3]) {
    for (int k = 0; k < 3; ++k) {
        F[k] = C[0] * MX[k] + C[3] * MY[k] + C[6] * MZ[k];
        R[k] = C[1] * MX[k] + C[4] * MY[k] + C[7] * MZ[k];
        U[k] = C[2] * MX[k] + C[5] * MY[k] + C[8] * MZ[k];
    }
}

} // namespace halo::vehcammath
