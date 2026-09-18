#pragma once

// MARKER FACES AND THE RENDER-ANCHOR REGISTRY, on top of the author's marker machinery (Markers.hpp).
// Shared by the holsters' fork additions, the manual reload and rack, the palette weapon's two-hand
// hold, the vehicle wheel and the wrist HUD.

#include <string_view>

#include "Math.hpp"
#include "uevr/API.hpp"

namespace halo {

// Tint a marker from an "r,g,b" cfg string (0..1): a dynamic material instance of the mesh's own
// material with its colour parameter set. Empty string = untouched. Harmless on a material
// without the parameter.
void marker_tint(uevr::API::UObject* comp, const char* rgb);

Vec3 holster_world_to_room(const Vec3& world, const Vec3& hmd_room);
// The same transforms with the CAMERA PASSED IN, for callers that need the room<->world mapping
// coherent with a specific time base (the gesture tests use the game tick's camera; the g_cam_*
// pair is the rendered frame's). Math identical to holster_room_to_world / holster_world_to_room.
Vec3 holster_room_to_world_at(const Vec3& room, const Vec3& hmd_room, const Vec3& cam);
Vec3 holster_world_to_room_at(const Vec3& world, const Vec3& hmd_room, const Vec3& cam);

// THE ROOM ANCHOR (reloadroomanchor): 0 = the HMD, the author's transform; 1 = the standing origin, what the
// head's rendered offset is measured from. True = anchor 1, and *out holds the point.
bool room_to_world_anchored(const Vec3& room, const Vec3& hmd_room, Vec3* out);

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
// The rendered camera position (g_cam_*), from the stereo pre-callback's view position, for the render pass and
// every room->world placed against the frame (SVC_MARKER_ANCHOR).
void marker_camera_publish();

// ---- THE MARKER MESH SWEEP'S BACKOFF, a CORE guard because it guards the AUTHOR'S sweep and
// serves whichever consumer asked for a marker. His spawn gate re-runs resolve_grenade_meshes()
// every 120 ticks until every wanted marker exists, and that walk is the whole ~290k object array
// (65-100 ms, measured by the sweep itself). In a level with no grenade mesh at all, and with
// holstermarkers=2 asking for the pouches unconditionally, nothing ever satisfies the gate, so the
// walk repeats for the whole level with no ceiling. Five empty sweeps in a row and the period goes
// to 1200 ticks; one that finds a mesh resets it.
//
// It lived inside holsterpollthrow, whose own master key switched the ceiling off, which meant the
// only configuration that needs the ceiling -- his markers on, our throw feature off -- was the one
// configuration that did not get it.
unsigned marker_sweep_period();              // ticks between the author's mesh sweeps
void     marker_sweep_result(const void* mf);   // null = the sweep found no mesh

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
