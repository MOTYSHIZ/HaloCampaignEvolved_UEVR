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

// A FRAME THAT TRACKS ONLY SOME OF THE VEHICLE'S ROTATIONS -- what a camera's rotationTracking /
// locationTracking lists mean. VF/VR/VU is the vehicle's own frame this instant.
//
//   The TILT: the frame's up leans with the vehicle only by the tracked parts of its pitch and roll,
//   taken about the vehicle's OWN axes (its nose up/down, its bank) -- so "pitch" means the vehicle's
//   pitch whatever way the frame itself faces.
//   The HEADING: the vehicle's when yaw is tracked; otherwise `free_heading_deg` (a world yaw), laid
//   onto that tilted deck -- the frame stands on the vehicle's floor but faces its own way. Look
//   sideways in a pitching Banshee with only pitch and roll tracked and you see the horizon ROLL, as
//   you would standing on its deck.
//   Then turned by `turn_deg` about its own up (the right-stick turn).
//
// With all three tracked and no turn it IS the vehicle's frame; with none it is level at the free
// heading. Degenerate only when the deck stands on its edge relative to the free heading (the tilt
// then keeps the vehicle's forward rather than inventing one).
inline void tracked_frame(const double VF[3], const double VR[3], const double VU[3],
                          bool track_yaw, bool track_pitch, bool track_roll,
                          double free_heading_deg, double turn_deg,
                          double F[3], double R[3], double U[3]) {
    const double D2R = 0.01745329252;
    double vp = 0.0, vy = 0.0, vr = 0.0;
    rotator_from_axes(VF, VR, VU, &vp, &vy, &vr);
    double TX[3], TY[3], TZ[3];
    rot_axes(track_pitch ? vp : 0.0, vy, track_roll ? vr : 0.0, TX, TY, TZ);
    double F0[3] = { TX[0], TX[1], TX[2] };
    if (!track_yaw) {
        // Laid onto the deck VERTICALLY: raise or lower the level heading until it lies in the deck.
        // Its compass bearing (seen from above) is then EXACTLY the free heading however the deck
        // tilts -- the view never yaws, it only tilts. Projecting along the deck's own normal instead
        // (the first cut) nudged the bearing whenever the deck banked, a small unwanted yaw that a unit
        // test caught. Degenerate only when the deck stands on edge (up nearly horizontal).
        const double h[3] = { std::cos(free_heading_deg * D2R), std::sin(free_heading_deg * D2R), 0.0 };
        if (std::fabs(TZ[2]) > 0.1) {
            const double t = -(h[0] * TZ[0] + h[1] * TZ[1]) / TZ[2];
            const double p[3] = { h[0], h[1], t };
            const double pl = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
            for (int k = 0; k < 3; ++k) F0[k] = p[k] / pl;
        }
    }
    // R0 = U x F0 (UE: X forward, Y right, Z up, and Z = X x Y component-wise).
    const double R0[3] = { TZ[1] * F0[2] - TZ[2] * F0[1],
                           TZ[2] * F0[0] - TZ[0] * F0[2],
                           TZ[0] * F0[1] - TZ[1] * F0[0] };
    const double t = turn_deg * D2R, ct = std::cos(t), st = std::sin(t);
    for (int k = 0; k < 3; ++k) {
        F[k] = ct * F0[k] + st * R0[k];
        R[k] = ct * R0[k] - st * F0[k];
        U[k] = TZ[k];
    }
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

// THE HEAD'S YAW IN THE ROOM, from its tracking quaternion (the runtime's axes: -Z forward, +Y up, +X
// right), in degrees, positive to the RIGHT (UE's sense). UEVR turns the whole room by the view's
// rotation, so this is also the head's yaw about the VIEW's own up, tilted view or not. Taken from the
// forward vector's level part; looking nearly straight up or down that part vanishes, so then from the
// right vector's (turned back by the 90 degrees between them), which the pitch of your head leaves alone.
inline double head_yaw_deg(double qx, double qy, double qz, double qw) {
    const double R2D = 57.29577951;
    // forward = q . (0,0,-1); in the view's frame its forward part is -z and its right part is x (the swizzle
    // the controller ray and the head anchor use: UE X = -z, Y = x, Z = y).
    const double fx = -2.0 * (qw * qy + qx * qz);
    const double fz = -(1.0 - 2.0 * (qx * qx + qy * qy));
    if (fx * fx + fz * fz >= 0.04) return std::atan2(fx, -fz) * R2D;
    const double rx = 1.0 - 2.0 * (qy * qy + qz * qz);   // right = q . (1,0,0)
    const double rz = 2.0 * (qx * qz - qw * qy);
    double y = std::atan2(rx, -rz) * R2D - 90.0;
    while (y > 180.0) y -= 360.0;
    while (y <= -180.0) y += 360.0;
    return y;
}

// THE TURN THAT PUTS WHERE YOU ARE LOOKING ON THE VEHICLE'S FORWARD (vehcamrecenter). F0/R0 are the view's
// forward and right before any turn (tracked_frame with no turn), VF the vehicle's forward, head_yaw your
// head's yaw in the room (head_yaw_deg). The stick turn rotates the view about its own up, and UEVR adds
// your head's yaw about that same axis, so after this turn your gaze's heading IS the vehicle's -- in a
// camera that tilts with the vehicle too, since both angles are measured in the tilted frame. With the
// vehicle's forward nearly along the view's up (it cannot be seen as a heading) only your head is undone.
// Degrees, in (-180, 180].
inline double recenter_turn_deg(const double VF[3], const double F0[3], const double R0[3], double head_yaw) {
    const double R2D = 57.29577951;
    const double f = VF[0] * F0[0] + VF[1] * F0[1] + VF[2] * F0[2];
    const double r = VF[0] * R0[0] + VF[1] * R0[1] + VF[2] * R0[2];
    const double veh = (f * f + r * r > 0.01) ? std::atan2(r, f) * R2D : 0.0;
    double t = veh - head_yaw;
    while (t > 180.0) t -= 360.0;
    while (t <= -180.0) t += 360.0;
    return t;
}

} // namespace halo::vehcammath
