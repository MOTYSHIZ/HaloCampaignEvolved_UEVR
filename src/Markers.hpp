// World-space marker machinery -- see the header comment in Markers.cpp.
#pragma once

#include <atomic>

#include "Math.hpp"
#include "uevr/API.hpp"

namespace halo {

// The rendered camera position (UE world cm), published by the stereo callback each frame.
// room_to_world anchors on it; consumers place markers with the functions below.
extern std::atomic<float> g_cam_x, g_cam_y, g_cam_z;

uevr::API::UObject* holster_marker_spawn_mesh(uevr::API::UObject* owner, uevr::API::UObject* mesh, double scale);
void holster_marker_set_mesh(uevr::API::UObject* comp, uevr::API::UObject* mesh);
void holster_marker_place(uevr::API::UObject* comp, const Vec3& world);
void holster_marker_show(uevr::API::UObject* comp, bool show);
void holster_marker_scale(uevr::API::UObject* comp, double s);
Vec3 holster_room_to_world(const Vec3& room, const Vec3& hmd_room);

} // namespace halo
