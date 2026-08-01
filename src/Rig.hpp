// The first-person weapon rig: making the visible gun follow your hand.
//
// The gun is a SEPARATE ACTOR socket-attached (socket "PrimaryWeapon") to the pawn's first-person
// skeletal mesh component. Two consequences drive everything here:
//
//   * Writing the WORLD transform is futile -- the attachment recomputes it from the socket every
//     frame, so the write is visibly undone as the idle animation plays.
//   * Writing the RELATIVE transform works, and persists. It is applied ON TOP of the socket, so it
//     rides the animation instead of fighting it. That is the seam this whole module stands on:
//     the anim node re-poses the SKELETON, while a scene component's relative transform sits a
//     level above the pose and is never touched by it.
//
// Two independent targets, and offsets on them stack:
//   * the weapon actor's own root component -> moves the GUN
//   * BPC_FP_SkeletalMesh_C on the pawn      -> moves ARMS + GUN (the motion-controller anchor)
//
// The rig is absent for ~5 s after a level load, resets on map load, and the gun's offset resets on
// every weapon swap (each weapon is a new actor). Re-resolve rather than caching a pointer.

#pragma once

// API.hpp, NOT Plugin.hpp -- Plugin.hpp defines the plugin entry points and may only be included
// by Plugin.cpp. See UeObject.hpp.
#include "uevr/API.hpp"
#include "Math.hpp"
#include "UeObject.hpp"

#include <atomic>
#include <cstdint>
#include <string>

namespace halo {

// ---- rig state -------------------------------------------------------------------------------
extern std::atomic<uint32_t> g_rig_resolve_tick;
extern std::atomic<bool>  g_rig_neutral_valid;
extern std::atomic<float> g_rig_neutral_x, g_rig_neutral_y, g_rig_neutral_z;
extern std::atomic<int>   g_rig_loc_works;      // -1 unknown, 0 no-op detected, 1 confirmed
extern std::atomic<float> g_ctrl_travel_max;
extern std::atomic<float> g_dbg_rig_x, g_dbg_rig_y, g_dbg_rig_z, g_dbg_rig_roll;
extern std::atomic<bool>  g_rig_wrote_once;
extern std::atomic<float> g_rig_survive_drift;
extern std::atomic<float> g_dbg_pos_x, g_dbg_pos_y, g_dbg_pos_z;
extern std::atomic<void*> g_rig_component;
extern uevr::API::UObject* g_rig_parent;

// ---- UObjectHook attachment ------------------------------------------------------------------
extern bool g_attached;
extern std::atomic<float> g_rigw_x, g_rigw_y, g_rigw_z, g_rigw_w;
extern std::atomic<bool>  g_rigw_valid;
extern std::atomic<float> g_rigw_parent_yaw;

// ---- debug markers ---------------------------------------------------------------------------
// A "borrowed" marker is an existing world actor repurposed as a visible dot, because UE strips
// DrawDebugSphere from shipping builds.
struct BorrowedMarker {
    uevr::API::UObject* actor = nullptr;
    uevr::API::UObject* root  = nullptr;
    Vec3 home{0.0f, 0.0f, 0.0f};
    Vec3 center_off{0.0f, 0.0f, 0.0f};   // actor origin -> bounds centre, after scaling
    bool active = false;
};

extern BorrowedMarker g_pivot_marker;
extern BorrowedMarker g_aim_marker;

// ---- rig resolution and writing --------------------------------------------------------------
uevr::API::UObject* resolve_rig();
uevr::API::UObject* follow_object(uevr::API::UObject* obj, const wchar_t* prop);

// Is the cached first-person-weapon route still live -- i.e. is the game rendering an FP weapon
// right now? The same TrackedObject + attachment walk as the resolve fast path; a live handle is
// left untouched (only resolve_rig re-establishes a dead one). This is stick mode's core detector:
// the route dies in vehicle seats, cutscenes, death and the post-load window, and -- unlike
// g_rig_component, a raw pointer that goes stale-non-null -- a dead route is DETECTED, not
// silently followed.
bool fp_weapon_route_alive();

// Diagnostic twin for the transition logs: is the rig COMPONENT itself still tracked-live? A
// weapon swap kills the route but keeps the rig (it is the pawn's component); what a vehicle seat
// does to it is exactly recon question R1, which this answers with one log line.
bool rig_component_alive();

// Write the rig's RELATIVE transform. Never the world transform -- see the note at the top.
bool rig_set_rotation(uevr::API::UObject* rig, double pitch, double yaw, double roll);
bool rig_set_location(uevr::API::UObject* rig, double x, double y, double z);
bool rig_set_scale(uevr::API::UObject* rig, double s);

bool call_ret_vec3(uevr::API::UObject* obj, const wchar_t* fn, Vec3* out);

// ---- UObjectHook attachment ------------------------------------------------------------------
void attach_apply(uevr::API::UObject* rig, const Quat& rot_off, const Vec3& loc_off_cm);
void attach_release(uevr::API::UObject* rig, const char* why);

// ---- pivot -----------------------------------------------------------------------------------
bool derive_pivot(uevr::API::UObject* rig, const wchar_t* socket, Vec3* out);
void log_pivot_candidates(uevr::API::UObject* rig);

// ---- markers ---------------------------------------------------------------------------------
void draw_debug_sphere(uevr::API::UObject* world_ctx, const Vec3& c, float radius,
                       float r, float g, float b,
                       float duration = 0.0f, int32_t segments = 12, float thickness = 2.0f);
bool resolve_marker(BorrowedMarker& m, float scale, const Vec3& ref, const char* label,
                    uevr::API::UObject* exclude);
bool marker_alive(BorrowedMarker& m);
void park_marker(BorrowedMarker& m, const Vec3& p);
void release_marker(BorrowedMarker& m, const char* label);

} // namespace halo