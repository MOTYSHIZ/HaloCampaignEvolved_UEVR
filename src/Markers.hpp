// World-space marker machinery -- see the header comment in Markers.cpp.
#pragma once

#include <atomic>

#include "Math.hpp"
#include "uevr/API.hpp"

namespace halo {

// The rendered camera position (UE world cm), published by the stereo callback each frame.
// room_to_world anchors on it; consumers place markers with the functions below.
extern std::atomic<float> g_cam_x, g_cam_y, g_cam_z;

// The rendered base yaw (degrees), published by Plugin.cpp's view callback each frame.
extern std::atomic<float> g_view_base_yaw;

uevr::API::UObject* holster_marker_spawn_mesh(uevr::API::UObject* owner, uevr::API::UObject* mesh, double scale);
// Tint a marker from an "r,g,b" cfg string (0..1): a dynamic material instance of the mesh's own
// material with its colour parameter set. Empty string = untouched. Harmless on a material
// without the parameter.
void marker_tint(uevr::API::UObject* comp, const char* rgb);
void holster_marker_set_mesh(uevr::API::UObject* comp, uevr::API::UObject* mesh);
void holster_marker_place(uevr::API::UObject* comp, const Vec3& world);
void holster_marker_place_rot(uevr::API::UObject* comp, const Vec3& world, float pitch, float yaw, float roll);
void holster_marker_show(uevr::API::UObject* comp, bool show);
void holster_marker_scale(uevr::API::UObject* comp, double s);
Vec3 holster_room_to_world(const Vec3& room, const Vec3& hmd_room);
Vec3 holster_world_to_room(const Vec3& world, const Vec3& hmd_room);
// The same transforms with the CAMERA PASSED IN, for callers that need the room<->world mapping
// coherent with a specific time base (the gesture tests use the game tick's camera; the g_cam_*
// pair above is the rendered frame's). Math identical to the two above.
Vec3 holster_room_to_world_at(const Vec3& room, const Vec3& hmd_room, const Vec3& cam);
Vec3 holster_world_to_room_at(const Vec3& world, const Vec3& hmd_room, const Vec3& cam);

// ---- ONE SOLVE FOR EVERYTHING RENDERED (from the headset, 2026-09-11: "make sure everything uses the
// exact same solve"). Any marker a game-tick system places registers its ROOM pose here; the
// stereo callback re-places every registered marker through the camera the frame is actually
// drawn from, in one pass, right where the wrist HUD places itself. Room poses are
// camera-invariant, so the re-anchor is exact, and every rendered helper is smooth or juddery
// TOGETHER -- never a mix. The tick keeps every decision (spawn, show, mesh, scale, state);
// this layer only re-anchors position and rotation. World-simulated things (the dropped mag
// falling) stay world-placed on purpose and must not register.
void marker_render_anchor(uevr::API::UObject* comp, const Vec3& room);
void marker_render_anchor_rot(uevr::API::UObject* comp, const Vec3& room, float pitch, float yaw, float roll);
void marker_render_drop(uevr::API::UObject* comp);   // hidden or retired: stop re-placing it
void markers_render_place();                          // the render pass (stereo callback, once per frame)

// The richer faces the vehicle wheel uses: name-list spawn with per-axis scale (a squashed
// sphere reads as a disc), placement with orientation, and the hull-frame transforms that make
// a point bolted to the vehicle mean the same thing to the ring and to the grab zone.
uevr::API::UObject* marker_spawn_list(uevr::API::UObject* owner, const wchar_t* const* meshes,
                                      int nmesh, double sx, double sy, double sz);
void marker_place_rot(uevr::API::UObject* comp, const Vec3& world, float pitch, float yaw);
void marker_scale3(uevr::API::UObject* comp, double sx, double sy, double sz);
bool markers_hull_local_to_world(uevr::API::UObject* hull, const Vec3& local, Vec3* out, float* hull_yaw);
bool markers_world_to_hull_local(uevr::API::UObject* hull, const Vec3& world, Vec3* out);

} // namespace halo
