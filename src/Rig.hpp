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

// WEAPON-TO-RIG SEPARATION, the one measurement that tells the two post-death hypotheses apart.
//
// The weapon is a separate ACTOR socketed onto the rig mesh, so where it ends up is
//     weapon_world = rig_component_world + socket_offset(animated pose)
// Our placement only controls the first term. Logging the DIFFERENCE therefore splits the cases:
// if it holds steady across a respawn the rig is being placed correctly and the socket moved
// under us; if it jumps, the placement itself is wrong. Every other number in the rig log varies
// with hand position, which is why none of them could answer this.
extern std::atomic<float> g_dbg_wpn_dx, g_dbg_wpn_dy, g_dbg_wpn_dz;
extern std::atomic<bool>  g_dbg_wpn_ok;
// Bumped whenever the component RelativeLocation is composed against a different parent than last
// tick. A new parent silently reinterprets every offset we write.
extern std::atomic<int>   g_dbg_parent_changes;

// The socket expressed in the MESH's own frame: position and rotation. Both are fixed properties
// of the skeleton, so unlike the world-space separation they should not vary as you aim -- which
// makes them the right things to compare across a respawn. The world separation says THAT the
// geometry changed; these say WHICH part of it did.
extern std::atomic<float> g_dbg_sock_x, g_dbg_sock_y, g_dbg_sock_z;
extern std::atomic<float> g_dbg_sock_p, g_dbg_sock_yw, g_dbg_sock_r;
extern std::atomic<bool>  g_dbg_sock_ok;

// WHERE THE GUN ACTUALLY ENDED UP, in controller-local cm -- ground truth for "is the weapon where
// the calibration says it is". Built from two LIVE engine reads (the weapon actor and the rig's
// parent) plus the controller pose; it consumes none of the values we wrote, so it cannot come out
// right by construction the way a check against our own output would.
// Compare with off_x/off_y/off_z: equal means correct, and the difference IS the placement error.
extern std::atomic<float> g_dbg_Lact_x, g_dbg_Lact_y, g_dbg_Lact_z;
extern std::atomic<bool>  g_dbg_Lact_ok;
extern std::atomic<float> g_dbg_pos_x, g_dbg_pos_y, g_dbg_pos_z;
extern std::atomic<void*> g_rig_component;
extern uevr::API::UObject* g_rig_parent;

// ---- UObjectHook attachment ------------------------------------------------------------------
extern bool g_attached;
extern std::atomic<float> g_rigw_x, g_rigw_y, g_rigw_z, g_rigw_w;
extern std::atomic<bool>  g_rigw_valid;
extern std::atomic<float> g_rigw_parent_yaw;

// The rig's WORLD-space offset from its parent, published by the tick so the render side can
// re-derive RelativeLocation against the LIVE parent -- exactly as it already does for rotation.
//
// Without this, only rotation is corrected per frame. The location stays as the tick computed it,
// conj(parent_at_tick) * off_world, so between ticks the weapon's world position becomes
// R_parent_now * conj(R_parent_tick) * off_world: the offset vector swung by however far the parent
// turned. That is a pure lever -- harmless when the parent crawls, violent when it does not. With
// direct aim assignment the parent tracks the hand exactly, RIGRENDER measured 82 deg of parent
// motion between ticks, and through a ~57 cm controller offset that is most of a metre of swing.
extern std::atomic<float> g_rigw_off_x, g_rigw_off_y, g_rigw_off_z;
extern std::atomic<bool>  g_rigw_off_valid;

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
//
// It does NOT mean "the player is not on foot": standing unarmed kills the route too. Pair it with
// fp_presentation_state() below before concluding anything about who should own the camera.
bool fp_weapon_route_alive();

// Is the game presenting the player in FIRST PERSON right now? 1 = yes, 0 = no, -1 = cannot tell.
//
// Reads BlamPawn's own CurrentBlamCameraPerspective, and LEARNS which value means first person by
// sampling it while a first-person weapon is live (pass route_alive so it can). -1 means the read
// failed and the caller must assume nothing -- see the doctrine block in Rig.cpp.
int fp_presentation_state(bool route_alive);

// The last raw perspective byte read, for the transition logs. -1 until one is read.
extern std::atomic<int> g_dbg_persp;

// Diagnostic twin for the transition logs: is the rig COMPONENT itself still tracked-live? A
// weapon swap kills the route but keeps the rig (it is the pawn's component); what a vehicle seat
// does to it is exactly recon question R1, which this answers with one log line.
bool rig_component_alive();

// The tracked rig component itself, or nullptr when dead -- for cheap per-tick probes (the
// stick-mode dismount watcher) that need the component without triggering a resolve.
uevr::API::UObject* rig_tracked_component();

// The FP weapon ACTOR the rig was last reached through, or nullptr. For the reticule trace: the gun
// is a separate actor, so ignoring the pawn does not cover it.
uevr::API::UObject* fp_weapon_actor();

// The weapon's own root component -- the thing to pin when attaching the GUN rather than the arms.
// Null whenever no weapon is in hand, which the caller must treat as "release, do not fall back".
uevr::API::UObject* fp_weapon_root();

// ---- first-person shield shell ---------------------------------------------------------------
// BPC_FP_TranslucentSkeletalMesh_C: the translucent energy skin that lights up when shields flare
// or break and when the overshield is active. A SIBLING of the arms rig, running its own instance
// of the same first-person anim blueprint -- so it is posed identically to the arms but transformed
// independently, and our relative-transform write to the arms never moves it. Driving it with the
// IDENTICAL transform is what keeps the shield on the hands.
//
// resolve_shield_shell() walks the PAWN's component list (O(components)), never the object array.
// Pass the arms rig's attach parent for the sibling fallback; nullptr is fine.
uevr::API::UObject* resolve_shield_shell(uevr::API::UObject* rig_parent);

// The tracked shell, or nullptr if never acquired / its array slot was recycled. O(1), but it
// class-name-checks the slot, which builds a string -- fine per tick, NOT per render frame.
uevr::API::UObject* shield_shell();

// Render-thread mirror of the above: a bare validated pointer, published by the tick and consumed
// by the stereo callback, exactly as g_rig_component is. Exists because the shell has to be
// re-applied at RENDER rate as well -- writing the arms per frame and the shell only per tick
// (~32 Hz) leaves the shield trailing the hands while you turn and snapping back when you stop.
extern std::atomic<void*> g_shell_component;

// Adopt a freshly resolved shell (no-op if unchanged). One O(n) slot lookup per acquisition.
void note_resolved_shell(uevr::API::UObject* shell);

// Drop the handle -- level transition, or the feature being switched off.
void forget_shield_shell();

// Write the rig's RELATIVE transform. Never the world transform -- see the note at the top.
bool rig_set_rotation(uevr::API::UObject* rig, double pitch, double yaw, double roll);
bool rig_set_location(uevr::API::UObject* rig, double x, double y, double z);
bool rig_set_scale(uevr::API::UObject* rig, double s);

// Show/hide a first-person component AND ITS CHILDREN (the rig's six armour static meshes ride on
// the propagate flag; the shield shell is a sibling and needs its own call). Used to hide the arms
// while the player is unarmed -- see the hide-arms block in Plugin.cpp for when and why.
bool rig_set_visible(uevr::API::UObject* comp, bool visible);

// KEEP A HIDDEN SKELETAL MESH ANIMATING.
//
// EVisibilityBasedAnimTickOption::AlwaysTickPoseAndRefreshBones = 0. UE defaults to
// OnlyTickPoseWhenRendered, so anything hidden with rig_set_visible() stops evaluating its pose --
// and the weapon hangs off this mesh's PrimaryWeapon socket, so a frozen pose is a frozen socket:
// no recoil, and the gun sits wherever the animation happened to stop.
//
// THE CANONICAL COPY. Arms.cpp used to carry its own; a second one is how the two hide paths came
// to disagree. Note that collapsing them fixed the DUPLICATION but not the behaviour -- both
// copies called a UFUNCTION this build does not have. See Rig.cpp: it is a PROPERTY WRITE now,
// and it RETURNS FALSE (and logs, once) when the property cannot be resolved. Do not turn it
// back into a call_function, and do not let it return true unconditionally: an unobservable
// no-op is what hid this for the whole life of the feature.
bool rig_set_always_tick_pose(uevr::API::UObject* comp);

// World-space equivalents. See the note in Rig.cpp: the relative ROTATION write does not take on
// this game's first-person mesh, so rigmode 3 drives the world transform instead of composing a
// relative one against a parent whose result the engine then discards.
bool rig_set_world_rotation(uevr::API::UObject* rig, double pitch, double yaw, double roll);
bool rig_set_world_location(uevr::API::UObject* rig, double x, double y, double z);

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