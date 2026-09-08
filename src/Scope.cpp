// The weapon scope. Design and doctrine in Scope.hpp; every engine call here is a pattern the
// reticule already proves on this title (component creation, MID + SlateUI binding, absolute
// world placement with the same K2_* marshalling).

#include "Scope.hpp"
#include "ScopeOffset.hpp"
#include "Config.hpp"
#include "DevTools.hpp"
#include "Rig.hpp"   // call_ret_vec3, for the placement readback
#include "Reticule.hpp"   // make_color_rt, for the dev probe's texture
#include "ScopeBlit.hpp"  // digital zoom: registers its own render callback
// The gun-mounted compositor quad. Driven from HERE and nowhere else -- same reason ScopeBlit
// registers its own callback: the feature lands without touching Plugin.cpp, which other sessions
// are editing. Every call it makes into the compositor lane is behind one adapter in ScopeLayer.cpp.
#include "ScopeLayer.hpp"
#include <d3d12.h>   // ID3D12Resource::GetDesc, for the scene-RT probe only

#include <cmath>
#include <cstring>

using namespace uevr;

namespace halo {

// Defined in Reticule.cpp with external linkage (shared engine helpers).
API::UObject* make_color_rt(float r, float g, float b, float a, int size);
// FORCE-LOADS an asset by path, unlike find_uobject which only sees what is already in memory.
API::UObject* load_asset_by_path(const char* path);

std::atomic<bool> g_scope_active{false};
std::atomic<uint32_t> g_scope_lt_edges{0};
std::atomic<uint8_t>  g_scope_lt_max{0};
std::atomic<uint32_t> g_scope_captures{0};

namespace {

TrackedObject s_pane;       // StaticMeshComponent showing the target (Engine Plane)
TrackedObject s_pane_mid;   // its MID, built from EmissiveMeshMaterial (see the pane
                            // creation below; an older comment here said
                            // Widget3DPassThrough_Opaque, which was stale and sent one
                            // investigation down the wrong path)
TrackedObject s_capture;    // SceneCaptureComponent2D on the shot line
TrackedObject s_rt;         // UTextureRenderTarget2D both of them share

// A note on detecting a WEAPON SWITCH here, because three detectors were built before one worked
// and two of the three were discarded on reasoning rather than measurement:
//   * The RIG does not change across a swap. Measured: one "attached to rig" line in a whole
//     session of switching. Genuinely dead.
//   * Watching the PANE for being HIDDEN does not work. The game never hides the arms mid-swap; it
//     only ever forces visibility ON (which is what used to flash the closed pane onto the screen).
//     With the scope open that writes true over true, so there is nothing to see.
//   * A "canary" component held at bVisible=false, to catch that propagation where the pane could
//     not -- built, deployed, and NEVER TRIPPED once. Removed.
//   * fp_weapon_actor() CHANGES, every single time. This was dismissed early on the strength of a
//     code comment (resolve_rig caches the weapon actor, so it was called "sticky"), which only
//     holds if the old actor SURVIVES the swap. It does not: the game destroys the first-person
//     viewmodel actor, the cached handle goes dead, and the sweep finds the new one.
// The lesson is the ordinary one -- a comment describes intent, and only the log describes
// behaviour. See docs\BLAM_AIM_FINDINGS.md, 2026-08-16.

bool  s_failed = false;         // latch: no per-tick retry storm after a hard failure
int   s_rt_size_applied = 0;    // scoperes the live RT was built at
// The ETextureRenderTargetFormat the live RT was built at, derived from scopesrc. Tracked for the
// same reason as the size: the target is allocated ONCE and scopesrc is live, so without this a
// source change leaves the old format in place and the new source silently renders into a target
// it does not match. That cost a whole session -- every scopesrc edit needed a relaunch, and the
// XR layer's format refusal ("DXGI format 10 is not in the swapchain's family") persisted after a
// live edit that looked like it should have fixed it. -1 = nothing built yet.
int   s_rt_format_applied = -1;
// scope_shape the live pane was built at. -1 = nothing built, which is also the REQUEST TO
// REBUILD (the shape gate's own `s_shape_built != -1` check then skips the destroy and goes
// straight to one clean ensure_components).
//
// File scope, not a static inside scope_apply, for the same reason as the weapon handle below:
// the teardown-recovery path that has to ask for a rebuild runs EARLIER in the function than the
// declaration did, so a function-local could not be reached from it.
int   s_shape_built = -1;
float s_fov_applied = 0.0f;     // FOVAngle last written (write on change only)
bool  s_pane_shown = false;
// What the scope was opened holding -- the weapon-switch close compares against it.
//
// File scope, not a static inside the detector, because the detector only runs while the pane is
// SHOWN: a scope closed by the trigger or by staleness never reached the reset, and the stale value
// then cancelled the next open on its first tick.
//
// The CLASS is the identity, not the actor pointer. The actor is recreated for reasons that are not
// weapon switches at all -- reloads, animation states, leaving a cutscene -- and each of those hands
// back a NEW instance of the SAME gun, which an instance comparison reads as a switch and closes the
// scope for no reason. A real switch always changes the class (BP_FP_Magnum_WeaponActor_C ->
// BP_FP_AR_WeaponActor_C). Comparing class pointers is also cheaper: no name string is built.
API::UObject* s_seen_weapon     = nullptr;   // kept for the log line only
API::UClass*  s_seen_weapon_cls = nullptr;   // the actual comparison
uint32_t      s_scope_open_tick = 0;
// Which rig the components are currently attached to. A weapon swap or respawn hands us a new
// rig; re-attaching on change is what keeps the pane parented across those transitions.
API::UObject* s_attached_rig = nullptr;
// The socket we last attached AT, as part of the attachment's identity. Without this the
// change-detect compares parent pointers only -- and rig-at-a-socket has the SAME parent pointer as
// rig-at-the-origin, so flipping scopeparent live would be a no-op that looks like a dead setting.
const wchar_t* s_attached_socket = nullptr;

// PANE RE-ANCHORING. The pane is placed in world space ONCE and then left strictly alone, so it
// is a rigid child of the rig and moves exactly as the controller does. Re-writing its world
// transform every tick (what the first attachment attempt did) makes the engine recompute a new
// relative offset each time: the pane then re-derives its position from the aim ray at ~32 Hz,
// which both lets the relative placement DRIFT and feeds the aim signal's jitter into something
// a 2x lens magnifies -- the reported "I can drift its relative location, and it moves much more
// jittery than my controller". So a write happens only when something actually changed.
bool  s_pane_anchored = false;
// Defined further down with the calibration gesture. Forward-declared because the ATTACH decision
// (well above it) must know when a capture is in progress: a capture reads RelativeLocation, so it
// has to happen in the canonical rig frame or it records numbers in the socket's frame instead.
extern bool s_calib_held;
float s_anchored_dist = 0.0f, s_anchored_right = 0.0f, s_anchored_up = 0.0f;
float s_anchored_size = 0.0f;
float s_anchored_rp = 0.0f, s_anchored_ry = 0.0f, s_anchored_rr = 0.0f;

// Has a placement knob moved since the pane was anchored? Live tuning stays possible without
// giving up rigidity: a knob edit re-anchors once, then it is rigid again at the new offset.
int   s_anchored_mount = -1;
// Set once if the relative mount is measured to have put the pane somewhere unintended -- see
// the readback in scope_apply. Session-scoped; a config edit to scopemount clears it.
bool  s_relative_rejected = false;
// Last good roll-lock angle. The lock is ABSOLUTE now (the image is locked to the lens itself,
// not to a remembered pose), so no reference pose is needed -- this only carries the previous
// value across the one degenerate case, where the lens's up axis lies along the aim ray.
float s_roll_lock_last = 0.0f;
bool pane_placement_changed() {
    return s_anchored_dist  != g_cfg.scope_dist  || s_anchored_right != g_cfg.scope_right ||
           s_anchored_up    != g_cfg.scope_up    || s_anchored_size  != g_cfg.scope_size  ||
           s_anchored_rp    != g_cfg.scope_rot_p || s_anchored_ry    != g_cfg.scope_rot_y ||
           s_anchored_rr    != g_cfg.scope_rot_r || s_anchored_mount != g_cfg.scope_mount;
}

bool read_component_rotation(API::UObject* comp, Vec3* out) {
    return call_ret_vec3(comp, L"K2_GetComponentRotation", out);
}

// No-op kept as a single call site rather than deleted at four places: the roll lock is ABSOLUTE
// now (image locked to the lens itself), so there is no reference pose to record. Left here so a
// future lock that DOES need a reference has an obvious home.
void note_pane_roll_reference(API::UObject*) {}

void note_pane_anchored() {
    s_pane_anchored  = true;
    s_anchored_dist  = g_cfg.scope_dist;   s_anchored_right = g_cfg.scope_right;
    s_anchored_up    = g_cfg.scope_up;     s_anchored_size  = g_cfg.scope_size;
    s_anchored_rp    = g_cfg.scope_rot_p;  s_anchored_ry    = g_cfg.scope_rot_y;
    s_anchored_rr    = g_cfg.scope_rot_r;  s_anchored_mount = g_cfg.scope_mount;
}

// The ray for the CURRENT tick, written by scope_notice_ray immediately before it applies
// placement (same tick -- the old consume-on-the-next-tick shape read as smoothing lag in the
// headset). s_last_notice_tick lets scope_frame_end park the pane once no rays arrive.
Vec3 s_ray_origin{0.0f, 0.0f, 0.0f};
Vec3 s_ray_target{0.0f, 0.0f, 0.0f};
// The REAL traced hit, published separately from the ray because the ray's target is an arbitrary
// 500 cm point (see scope_notice_focus in Scope.hpp). Used ONLY to decide what depth the capture
// converges at -- never for direction, which must stay unsmoothed.
Vec3 s_focus{0.0f, 0.0f, 0.0f};
bool s_focus_valid = false;
uint32_t s_focus_tick = 0;      // when s_focus was last refreshed; see scope_notice_focus
// How long a focus point stays trustworthy. Short, because the FALLBACK IS SAFE AND TRACKS: it is
// the ray's own 500 cm point, which is right in DIRECTION and merely converges nearer than the real
// hit. A frozen focus is strictly worse than a tracking approximation -- it is wrong in direction,
// which is the one thing the pane must never be.
constexpr uint32_t kFocusGraceTicks = 6;
uint32_t s_last_notice_tick = 0;

// ATTACH a component to the rig so the ENGINE composes its world transform every render frame.
//
// This is the whole answer to "the scope does not feel like a child of my controller": our
// per-tick world writes run at ~32 Hz, while the rig they were chasing is re-applied at RENDER
// rate in the stereo callback. Same failure the shield shell hit (Rig.hpp: "leaves the shield
// trailing the hands while you turn and snapping back when you stop"), same fix.
//
// KeepWorld (rule 1) on all three axes: the caller keeps writing world transforms exactly as
// before, the engine converts each write into a relative offset, and BETWEEN writes the child
// rides the parent. So tick rate still decides how often the aim ray is re-derived, but the
// hand-following itself becomes render rate and lag-free.
//
// SOCKET, and this is the part that makes recoil work at all. Attaching to a skeletal mesh
// COMPONENT parents to its origin, which does not animate -- the bones do. `socket` names a socket
// on that mesh, and the child then rides the animated bone. That distinction is the entire reason
// the first attempt at weapon-following did nothing useful: it parented to a component root and
// the pane duly ignored every animation the mesh played. Pass nullptr for the component itself.
bool attach_to(API::UObject* comp, API::UObject* parent, const wchar_t* socket = nullptr) {
    if (comp == nullptr || parent == nullptr) return false;
    alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
    *reinterpret_cast<void**>(q) = parent;   // Parent
    // SocketName, 8 bytes at +8. Zeroed = None = the component itself. make_fname, NOT API::FName:
    // the latter resolves to None on this build and a None socket fails QUIETLY by attaching to the
    // origin -- which is indistinguishable from the bug we are here to fix.
    if (socket != nullptr && socket[0] != 0) {
        API::FName sn = make_fname(socket);
        memcpy(q + 8, &sn, sizeof(int32_t) * 2);
    }
    q[16] = 1;   // LocationRule = KeepWorld
    q[17] = 1;   // RotationRule = KeepWorld
    q[18] = 1;   // ScaleRule    = KeepWorld
    q[19] = 0;   // bWeldSimulatedBodies
    comp->call_function(L"K2_AttachToComponent", q);

    // VERIFY, DO NOT ASSUME. This used to `return true` for "we made the call", which is not the
    // same claim at all -- K2_AttachToComponent can no-op and says nothing when it does. That cost
    // a live session: the log cheerfully read "attached to WEAPON (capture=1 pane=1)" while the
    // pane was in fact parented to NOTHING and stayed put in the world, so the player could walk
    // away from it. Read AttachParent back and report what actually happened.
    if (auto* ap = comp->get_property_data<API::UObject*>(L"AttachParent")) {
        // Parent only. AttachSocketName is NOT re-read: a socket the mesh does not own is accepted
        // by the engine and silently behaves as None, so a readback would confirm the NAME we asked
        // for while the attachment sat at the origin. The socket is validated where it can be --
        // by the caller, against the socket the weapon itself is known to ride.
        return *ap == parent;
    }
    // AttachParent unreadable -- report failure rather than success. A caller that falls back on
    // false is safe; one that trusts an unverified true is what produced the floating pane.
    return false;
}

// RELATIVE setters. With the component attached, these ARE the pane's offset from the rig --
// independent of each other by construction, which is the point: the world-space placement they
// replace derived position from the aim ray and then rotated, so a rotation trim moved the pane.
void set_relative_location(API::UObject* comp, double x, double y, double z) {
    alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
    auto* d = reinterpret_cast<double*>(q);
    d[0] = x; d[1] = y; d[2] = z;
    *reinterpret_cast<int32_t*>(q + 40) = 2;   // ETeleportType::ResetPhysics
    comp->call_function(L"K2_SetRelativeLocation", q);
}

void set_relative_rotation(API::UObject* comp, double pitch, double yaw, double roll) {
    alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
    auto* d = reinterpret_cast<double*>(q);
    d[0] = pitch; d[1] = yaw; d[2] = roll;
    *reinterpret_cast<int32_t*>(q + 32) = 2;
    comp->call_function(L"K2_SetRelativeRotation", q);
}

void set_relative_scale(API::UObject* comp, double x, double y, double z) {
    alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
    auto* d = reinterpret_cast<double*>(q);
    d[0] = x; d[1] = y; d[2] = z;
    comp->call_function(L"SetRelativeScale3D", q);
}

void set_visibility(API::UObject* comp, bool on) {
    if (comp == nullptr) return;
    alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
    q[0] = on ? 1 : 0;
    comp->call_function(L"SetVisibility", q);
}

// Same marshalling as reticule_mesh_move -- doubles for LWC, ETeleportType at the tail.
void set_world_location(API::UObject* comp, const Vec3& p) {
    alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
    auto* d = reinterpret_cast<double*>(q);
    d[0] = p.x; d[1] = p.y; d[2] = p.z;
    *reinterpret_cast<int32_t*>(q + 40) = 2;
    comp->call_function(L"K2_SetWorldLocation", q);
}

void set_world_rotation(API::UObject* comp, double pitch, double yaw, double roll) {
    alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
    auto* d = reinterpret_cast<double*>(q);
    d[0] = pitch; d[1] = yaw; d[2] = roll;
    *reinterpret_cast<int32_t*>(q + 32) = 2;
    comp->call_function(L"K2_SetWorldRotation", q);
}

// How flat the round lens is, as a fraction of its diameter. The Engine Cylinder is a solid the
// same height as it is wide, so at uniform scale it reads as a drum rather than a lens; uevrlib
// squashes the same asset by a comparable ratio for its ocular lens.
constexpr double kLensThickness = 0.001;

void set_world_scale3(API::UObject* comp, double x, double y, double z) {
    alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
    auto* d = reinterpret_cast<double*>(q);
    d[0] = x; d[1] = y; d[2] = z;
    comp->call_function(L"SetWorldScale3D", q);
}

void set_world_scale(API::UObject* comp, double s) { set_world_scale3(comp, s, s, s); }

// PIN THE CAPTURE'S EXPOSURE.
//
// A post-processed capture source (FinalColorHDR / FinalToneCurveHDR) is the only way to get the
// effects that LIVE in post -- bloom, and with it the shield shimmer and tracer glow the scope was
// missing. But post also brings the capture's own AUTO-EXPOSURE, which on a scene capture has no
// sane view history to converge against: measured in-headset, the pane showed a plausible image
// and then faded to solid black and stayed there. Pinning min = max exposure turns auto-exposure
// into a fixed one, which is what a scope wants anyway (a real optic does not re-expose).
//
// The offsets inside FPostProcessSettings are resolved by REFLECTION rather than hardcoded: this
// struct's layout differs between engine versions and even between builds with different
// features compiled in, and a guessed offset here would write into an unrelated float.
void pin_capture_exposure(API::UObject* cap, float ev) {
    auto* cls = cap->get_class();
    if (cls == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] scope: pin -- no class"); return; }
    auto* pps_prop = cls->find_property(L"PostProcessSettings");
    if (pps_prop == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] scope: pin -- PostProcessSettings property NOT "
                             "FOUND on the capture component");
        return;
    }
    auto* pps = reinterpret_cast<uint8_t*>(cap) + pps_prop->get_offset();

    auto* pps_struct = API::get()->find_uobject<API::UStruct>(
        L"ScriptStruct /Script/Engine.PostProcessSettings");
    if (pps_struct == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] scope: pin -- FPostProcessSettings SCRIPTSTRUCT "
                             "not found; no field offsets are resolvable this way");
        return;
    }
#if HALO_VR_DEV
    // One-shot census: does find_property work on this struct AT ALL? Distinguishes "this build
    // renamed/stripped one field" from "struct property lookup does not work here", which are
    // very different problems and were indistinguishable from the old silent early-outs.
    {
        static bool censused = false;
        if (!censused) {
            censused = true;
            static const wchar_t* kProbe[] = {
                L"AutoExposureMinBrightness", L"AutoExposureMaxBrightness", L"AutoExposureBias",
                L"AntiAliasingMethod", L"BloomIntensity", L"MotionBlurAmount"
            };
            for (const wchar_t* n : kProbe) {
                auto* p = pps_struct->find_property(n);
                API::get()->log_info("[Halo-CampE-UEVR] SCOPEDEV pps probe: %-28s %s",
                                     narrow(n).c_str(),
                                     p != nullptr ? "found" : "NOT FOUND");
            }
        }
    }
#endif

    // Each value has a paired bOverride_ bit that must be set or the engine ignores the value.
    struct Field { const wchar_t* value; const wchar_t* over; float v; };
    const Field fields[] = {
        { L"AutoExposureMinBrightness", L"bOverride_AutoExposureMinBrightness", ev },
        { L"AutoExposureMaxBrightness", L"bOverride_AutoExposureMaxBrightness", ev },
        // Was hardcoded 0.0f. Now the tunable, so a PINNED capture and an AUTO one are brightened
        // by the same key -- otherwise turning the pin on would silently throw the bias away.
        { L"AutoExposureBias",          L"bOverride_AutoExposureBias",
          g_cfg.scope_autoexposure_bias },
    };
    // COUNT what actually resolved. This used to log "PINNED" unconditionally, which is a
    // success message for a function that may have written nothing -- and that is exactly how
    // "black even with the exposure pinned" got recorded as a measurement when the pin may never
    // have applied. A writer that cannot report its own failure is not instrumentation.
    int applied = 0;
    for (const auto& f : fields) {
        auto* vp = pps_struct->find_property(f.value);
        auto* op = pps_struct->find_property(f.over);
        if (vp == nullptr || op == nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] scope: exposure field NOT FOUND (%s / %s)",
                                 narrow(f.value).c_str(), narrow(f.over).c_str());
            continue;
        }
        *reinterpret_cast<float*>(pps + vp->get_offset()) = f.v;
        // The bOverride_ flags are bitfields; the FBoolProperty knows its own mask.
        if (auto* bp = static_cast<API::FBoolProperty*>(op)) {
            auto* byte = pps + bp->get_offset();
            *byte = (uint8_t)(*byte | bp->get_field_mask());
        }
        ++applied;
    }
    API::get()->log_info("[Halo-CampE-UEVR] scope: capture exposure pin -- %d of %d fields applied "
                         "at %.2f%s", applied, (int)(sizeof(fields) / sizeof(fields[0])), ev,
                         applied == 0 ? "  <-- NOTHING WAS WRITTEN, treat any result as untested"
                                      : "");
}

// EXPOSURE COMPENSATION ON ITS OWN, FOR THE AUTO-EXPOSED CASE.
//
// pin_capture_exposure() also writes AutoExposureBias, but it only runs when scopeexposure > 0.
// With the pin off -- which is the configuration that lets the capture adapt per scene, and the
// one worth shipping -- nothing wrote any exposure field at all, so the image had no pre-tonemap
// brightness control. This is that control, and it is deliberately a separate function rather than
// a flag on the pin: the pin REPLACES adaptation, this one SHIFTS it, and conflating the two is
// how you end up unable to have both.
//
// Same discipline as the pin: resolve the field, refuse loudly if it is not there, and report what
// was actually written rather than announcing success.
void apply_autoexposure_bias(API::UObject* cap, float ev) {
    auto* cls = cap->get_class();
    auto* pps_prop = (cls != nullptr) ? cls->find_property(L"PostProcessSettings") : nullptr;
    auto* pps_struct = API::get()->find_uobject<API::UStruct>(
        L"ScriptStruct /Script/Engine.PostProcessSettings");
    if (pps_prop == nullptr || pps_struct == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] scope: autoexposure bias -- PostProcessSettings "
                             "unresolved (prop=%p struct=%p), NOTHING WRITTEN",
                             (void*)pps_prop, (void*)pps_struct);
        return;
    }
    auto* pps = reinterpret_cast<uint8_t*>(cap) + pps_prop->get_offset();
    auto* vp = pps_struct->find_property(L"AutoExposureBias");
    auto* op = pps_struct->find_property(L"bOverride_AutoExposureBias");
    if (vp == nullptr || op == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] scope: autoexposure bias -- field NOT FOUND on this "
                             "build (value=%p override=%p), NOTHING WRITTEN", (void*)vp, (void*)op);
        return;
    }
    *reinterpret_cast<float*>(pps + vp->get_offset()) = ev;
    if (auto* bp = static_cast<API::FBoolProperty*>(op)) {
        auto* byte = pps + bp->get_offset();
        *byte = (uint8_t)(*byte | bp->get_field_mask());
    }
    // READ BACK. The whole reason this key exists is that the previous brightness lever was applied
    // somewhere the value could not be checked; do not repeat that.
    const float back = *reinterpret_cast<float*>(pps + vp->get_offset());
    API::get()->log_info("[Halo-CampE-UEVR] scope: autoexposure bias -> %+.2f EV (readback %+.2f)"
                         " -- exposure compensation AFTER adaptation, so the capture still adapts "
                         "per scene and just lands brighter. +1 EV = 2x.", ev, back);
}

// Defined further down, next to the blendable copy that also uses them.
std::string scope_path_of(API::UObject* o);
int blendable_count(API::UObject* obj, const wchar_t* pp_field);


// WHICH OBJECT'S PostProcessSettings DESCRIBE WHAT THE PLAYER ACTUALLY SEES?
//
// ONE picker for both copies, because they must never disagree: a grade taken from one object and
// blendables from another is a mixture no view ever renders, and the log would name two different
// sources for what reads as a single "we copied the camera".
//
// MEASURED 2026-09-07, and this is why the function exists: the grade copy took the FIRST
// CameraComponent in the object array and landed on
//     /Script/Engine.Default__CameraActor.CameraComponent
// -- a CLASS DEFAULT OBJECT's subobject, i.e. an engine template that has never rendered anything.
// It copied a neutral grade (WhiteTemp 6500 both sides, matching because both were defaults) and
// the readback happily confirmed it. The test looked clean and measured nothing.
//
// TWO separate defects, both fixed here:
//   * The CDO guard checked `o == class_default_object()`, which is FALSE for a component INSIDE a
//     a CDO -- the component is not itself the default object, only owned by one. Rejecting any
//     path containing "Default__" catches the subobject case the identity test cannot.
//   * "First in the object array" is nondeterministic. A level holds cutscene cameras, spectator
//     cameras and templates, most carrying no blendables at all. Scoring by blendable count is what
//     found the real pawn camera with 8, so both copies now score the same way.
API::UObject* pick_pp_source(const wchar_t** out_field, std::string* out_name) {
    API::UObject* src = nullptr;
    const wchar_t* field = nullptr;
    int best_cnt = 0;               // 0-blendable sources tell us nothing; require at least one
    if (auto* arr = API::get()->get_uobject_array()) {
        const int32_t n = arr->get_object_count();
        API::UObject* vol = nullptr;
        for (int32_t i = 0; i < n; ++i) {
            auto* o = reinterpret_cast<API::UObject*>(arr->get_object(i));
            if (o == nullptr) continue;
            const std::wstring cn = class_name_of(o);
            const bool is_cam = cn.find(L"CameraComponent") != std::wstring::npos;
            const bool is_vol = cn.find(L"PostProcessVolume") != std::wstring::npos;
            if (!is_cam && !is_vol) continue;
            auto* oc = o->get_class();
            if (oc != nullptr && o == oc->get_class_default_object()) continue;
            // The template check the identity test above cannot make. A real level object lives
            // under /Game/Levels/...; a template under /Script/Engine.Default__...
            if (scope_path_of(o).find("Default__") != std::string::npos) continue;
            const int cnt = blendable_count(o, is_cam ? L"PostProcessSettings" : L"Settings");
            if (cnt > best_cnt) {
                best_cnt  = cnt;
                src       = o;
                field     = is_cam ? L"PostProcessSettings" : L"Settings";
            }
            if (is_vol && vol == nullptr) vol = o;
        }
        if (src == nullptr && vol != nullptr) { src = vol; field = L"Settings"; }
    }
    if (out_field != nullptr) *out_field = field;
    if (out_name  != nullptr) *out_name  = (src != nullptr) ? scope_path_of(src)
                                                            : std::string("<none>");
    return src;
}

// HOW MANY BYTES IS THIS FPostProcessSettings FIELD?
//
// RESOLVED, never assumed. The colour-grading fields are FVector4, and under LWC that is four
// DOUBLES (32 bytes) on some builds and four floats (16) on others -- copying 16 bytes of a 32-byte
// field gives you half a value and no error. A StructProperty knows its own struct, and the struct
// knows its own size, so ask rather than guess. 0 = unknown type, and the caller must then REFUSE
// to copy rather than pick a plausible number.
int pps_field_size(API::FProperty* p) {
    if (p == nullptr) return 0;
    auto* fc = p->get_class();
    const auto* tn_name = (fc != nullptr) ? fc->get_fname() : nullptr;
    const std::wstring tn = (tn_name != nullptr) ? tn_name->to_string() : std::wstring();
    if (tn == L"StructProperty") {
        auto* st = static_cast<API::FStructProperty*>(p)->get_struct();
        return (st != nullptr) ? (int)st->get_properties_size() : 0;
    }
    if (tn == L"FloatProperty")  return 4;
    if (tn == L"DoubleProperty") return 8;
    if (tn == L"IntProperty")    return 4;
    if (tn == L"ObjectProperty" || tn == L"SoftObjectProperty" ||
        tn == L"WeakObjectProperty") return 8;
    if (tn == L"ByteProperty" || tn == L"EnumProperty") return 1;
    return 0;
}

// COPY THE GAME'S COLOUR GRADE ONTO THE CAPTURE.
//
// MEASURED IN A HEADSET 2026-09-07: the same corridor renders purple-lit in the main view and warm
// tan through the scope -- not a tint, a different colour family. That is raw albedo with the
// game's stylistic grade missing, and it survived every other explanation: it is not exposure, not
// bloom, not double-tonemapping (the compositor quad has ONE tonemapper and still shows it), and
// not Lumen (scopelumen=1 applied with readback GI=1 refl=1 and changed nothing).
//
// WHY THE CAPTURE MISSES IT. PostProcess VOLUMES are gathered identically -- UWorld::
// AddPostProcessingSettings runs for both views with no view-type filter. But a scene capture never
// goes through APlayerCameraManager, so anything on the CAMERA COMPONENT'S OWN PostProcessSettings
// is invisible to it. If a title puts its look on the camera rather than a volume, the capture
// renders ungraded, and nothing downstream can put it back.
//
// WHY THIS IS A SEPARATE FUNCTION FROM scopeppcopy. That one originally memcpy'd the WHOLE
// FPostProcessSettings, which would have carried the grade -- and was replaced with a
// blendables-only copy because the struct contains a TArray and byte-copying it duplicates the
// array POINTER into two owners, a genuine heap-corruption hazard. The intent was right and only
// the mechanism was wrong: this copies the grading fields INDIVIDUALLY, every one of them POD or a
// single object pointer, and never touches the TArray.
bool copy_pp_grading_onto_capture(API::UObject* cap) {
    auto* pps_struct = API::get()->find_uobject<API::UStruct>(
        L"ScriptStruct /Script/Engine.PostProcessSettings");
    if (pps_struct == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] scope: grade copy -- FPostProcessSettings not "
                             "reflected, NOTHING WRITTEN");
        return false;
    }

    // SAME SOURCE AS THE BLENDABLE COPY, and it has to stay that way: a grade taken from one object
    // and blendables from another is a mixture neither view ever renders. Camera first, volume as
    // the fallback, CDOs skipped.
    const wchar_t* src_field = nullptr;
    std::string src_name;
    API::UObject* src = pick_pp_source(&src_field, &src_name);
    if (src == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] scope: grade copy -- no usable CameraComponent or "
                             "PostProcessVolume (templates and 0-blendable sources are rejected)");
        return false;
    }

    auto* src_cls = src->get_class();
    auto* src_pp  = (src_cls != nullptr) ? src_cls->find_property(src_field) : nullptr;
    auto* cap_cls = cap->get_class();
    auto* cap_pp  = (cap_cls != nullptr) ? cap_cls->find_property(L"PostProcessSettings") : nullptr;
    if (src_pp == nullptr || cap_pp == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] scope: grade copy ABORTED -- src '%s'.%s = %p, "
                             "capture.PostProcessSettings = %p",
                             src_name.c_str(), narrow(src_field).c_str(),
                             (void*)src_pp, (void*)cap_pp);
        return false;
    }
    auto* s_base = reinterpret_cast<uint8_t*>(src) + src_pp->get_offset();
    auto* d_base = reinterpret_cast<uint8_t*>(cap) + cap_pp->get_offset();

    // Everything that decides the LOOK. All POD or one object pointer -- no TArray, by design.
    static const wchar_t* kGrade[] = {
        L"ColorGradingIntensity", L"ColorGradingLUT",
        L"WhiteTemp", L"WhiteTint",
        // ColorGain is DELIBERATELY ABSENT: scopegain owns it. Copying it from the source writes
        // the camera's default (1,1,1,1) straight over the player's gain, and since both appliers
        // run on-change only, the gain never comes back. Measured 2026-09-07 -- ColorGain was the
        // ONLY one of the 36 fields that differed, and the difference was our own write.
        L"ColorSaturation", L"ColorContrast", L"ColorGamma", L"ColorOffset",
        L"ColorSaturationShadows", L"ColorContrastShadows", L"ColorGammaShadows",
        L"ColorGainShadows", L"ColorOffsetShadows",
        L"ColorSaturationMidtones", L"ColorContrastMidtones", L"ColorGammaMidtones",
        L"ColorGainMidtones", L"ColorOffsetMidtones",
        L"ColorSaturationHighlights", L"ColorContrastHighlights", L"ColorGammaHighlights",
        L"ColorGainHighlights", L"ColorOffsetHighlights",
        L"ColorCorrectionShadowsMax", L"ColorCorrectionHighlightsMin",
        L"ColorCorrectionHighlightsMax",
        L"BlueCorrection", L"ExpandGamut", L"ToneCurveAmount", L"SceneColorTint",
        L"FilmSlope", L"FilmToe", L"FilmShoulder", L"FilmBlackClip", L"FilmWhiteClip",
    };

    // COUNT WHAT ACTUALLY DIFFERS, not just what we copied.
    //
    // "36 fields copied" says the loop ran. It does NOT say the source had a grade -- if the camera
    // sits at engine defaults, all 36 copies write the value that was already there and the pane
    // cannot change no matter how correct the mechanism is. That is the difference between "the
    // copy is broken" and "there is nothing to copy", and the WhiteTemp readback stopped
    // distinguishing them the moment the source became a real camera whose WhiteTemp is also the
    // 6500 default. This counter is the discriminator.
    int copied = 0, missing = 0, unsized = 0, differed = 0;
    char diff_names[240]; diff_names[0] = '\0';
    for (const wchar_t* name : kGrade) {
        auto* vp = pps_struct->find_property(name);
        if (vp == nullptr) { ++missing; continue; }
        const int sz = pps_field_size(vp);
        if (sz <= 0 || sz > 64) { ++unsized; continue; }   // unknown type: refuse, do not guess
        if (std::memcmp(d_base + vp->get_offset(), s_base + vp->get_offset(), (size_t)sz) != 0) {
            ++differed;
            // Name the first few, because "3 differed" and "which 3" are different questions and
            // the second one is the one that says where the look actually lives.
            const std::string n = narrow(name);
            if (std::strlen(diff_names) + n.size() + 2 < sizeof(diff_names)) {
                if (diff_names[0] != '\0') std::strcat(diff_names, ", ");
                std::strcat(diff_names, n.c_str());
            }
        }
        std::memcpy(d_base + vp->get_offset(), s_base + vp->get_offset(), (size_t)sz);

        // The paired override bit, or the engine ignores the value we just wrote.
        std::wstring over = L"bOverride_"; over += name;
        if (auto* op = pps_struct->find_property(over.c_str())) {
            if (auto* bp = static_cast<API::FBoolProperty*>(op)) {
                auto* byte = d_base + bp->get_offset();
                *byte = (uint8_t)(*byte | bp->get_field_mask());
            }
        }
        ++copied;
    }

    // READ BACK something a human can sanity-check against the source, rather than reporting a
    // count and calling it verified.
    float st = -12345.0f, dt = -12345.0f;
    if (auto* p = pps_struct->find_property(L"WhiteTemp")) {
        st = *reinterpret_cast<float*>(s_base + p->get_offset());
        dt = *reinterpret_cast<float*>(d_base + p->get_offset());
    }
    API::get()->log_info("[Halo-CampE-UEVR] scope: grade copied from %s -- %d fields, %d not on "
                         "this build, %d refused. **%d ACTUALLY DIFFERED** (%s). WhiteTemp "
                         "src=%.1f capture=%.1f. %s",
                         src_name.c_str(), copied, missing, unsized, differed,
                         diff_names[0] != '\0' ? diff_names : "none",
                         st, dt,
                         copied == 0
                             ? "<-- NOTHING COPIED, treat any result as untested."
                             : (differed == 0
                                ? "<-- THE SOURCE CARRIES NO GRADE. Every field already matched, so "
                                  "this copy cannot change the image and the look is NOT in "
                                  "FPostProcessSettings. Stop tuning grade keys and look at "
                                  "lighting or a scene view extension."
                                : "<-- a real grade was inherited; if the pane still looks wrong "
                                  "the remaining difference is elsewhere."));
    return copied > 0;
}

// A FLAT BRIGHTNESS MULTIPLY ON THE CAPTURE, VIA ColorGain.
//
// This is the compositor quad's answer to scopebright. scopebright tints the in-world pane's
// MATERIAL, so it does nothing at all once the layer presents and the mesh is hidden -- and the
// quad has no material to tint. OpenXR's own per-layer gain
// (XR_KHR_composition_layer_color_scale_bias) is not usable here either: extensions must be enabled
// when the XrInstance is CREATED, and UEVR creates it; we only hook xrEndFrame on a session that
// already exists.
//
// So the gain has to live in the capture, and ColorGain is the right field for it: a straight
// multiply in the grading chain, independent of exposure. That matters -- scopeautoexposurebias
// moves the exposure and therefore changes what the tonemapper's shoulder does to the highlights,
// while this scales the graded colour. Two different tools, and mixing them up is how a brightness
// hunt turns into a colour-cast hunt.
//
// ColorGain is an FVector4 and under LWC that is four DOUBLES on some builds and four floats on
// others, so the width is RESOLVED from the reflected struct and a size we do not recognise is
// REFUSED rather than guessed at. XYZ take the gain, W stays 1: W is the master term and doubling
// it as well would square the effect for anyone reading the number as "how much brighter".
void apply_capture_gain(API::UObject* cap, float gain) {
    auto* cls = cap->get_class();
    auto* pps_prop = (cls != nullptr) ? cls->find_property(L"PostProcessSettings") : nullptr;
    auto* pps_struct = API::get()->find_uobject<API::UStruct>(
        L"ScriptStruct /Script/Engine.PostProcessSettings");
    if (pps_prop == nullptr || pps_struct == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] scope: gain -- PostProcessSettings unresolved, "
                             "NOTHING WRITTEN");
        return;
    }
    auto* pps = reinterpret_cast<uint8_t*>(cap) + pps_prop->get_offset();
    auto* vp  = pps_struct->find_property(L"ColorGain");
    auto* op  = pps_struct->find_property(L"bOverride_ColorGain");
    if (vp == nullptr || op == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] scope: gain -- ColorGain not on this build "
                             "(value=%p override=%p), NOTHING WRITTEN", (void*)vp, (void*)op);
        return;
    }
    const int sz = pps_field_size(vp);
    auto* base = pps + vp->get_offset();
    if (sz == 32) {                       // FVector4 of doubles (LWC)
        auto* d = reinterpret_cast<double*>(base);
        d[0] = d[1] = d[2] = (double)gain; d[3] = 1.0;
    } else if (sz == 16) {                // FVector4 of floats
        auto* f = reinterpret_cast<float*>(base);
        f[0] = f[1] = f[2] = gain; f[3] = 1.0f;
    } else {
        API::get()->log_info("[Halo-CampE-UEVR] scope: gain -- ColorGain is %d bytes, which is "
                             "neither 4 floats nor 4 doubles. REFUSING to write rather than guess "
                             "a layout.", sz);
        return;
    }
    if (auto* bp = static_cast<API::FBoolProperty*>(op)) {
        auto* byte = pps + bp->get_offset();
        *byte = (uint8_t)(*byte | bp->get_field_mask());
    }
    const double back = (sz == 32) ? *reinterpret_cast<double*>(base)
                                   : (double)*reinterpret_cast<float*>(base);
    API::get()->log_info("[Halo-CampE-UEVR] scope: gain -> %.3f (readback %.3f, %d-byte ColorGain) "
                         "-- a flat multiply in the capture's grading chain. This is the quad's "
                         "equivalent of scopebright, which only ever tinted the in-world mesh.",
                         gain, back, sz);
}

// RE-ENABLE LUMEN ON THE CAPTURE. THE ENGINE TURNS IT OFF FOR EVERY SCENE CAPTURE.
//
// SceneCaptureRendering.cpp:880-885, verbatim:
//     // By default, Lumen is disabled in scene captures, but can be re-enabled with the post
//     // process settings in the component.
//     View->FinalPostProcessSettings.DynamicGlobalIlluminationMethod = ...::None;
//     View->FinalPostProcessSettings.ReflectionMethod                = ...::None;
//     View->FinalPostProcessSettings.LumenSurfaceCacheResolution     = 0.5f;
//
// So the main view runs Lumen GI and Lumen reflections and OUR CAPTURE RUNS NEITHER -- same scene,
// same camera, a different lighting model. Reported from a headset as "it is like the lighting is
// different for the scene through the scope", which is exactly what losing indirect bounce and
// specular reflections looks like: flatter, warmer, and darker wherever the light was indirect.
//
// This is BASE-PASS lighting, upstream of every post-process knob in this file -- which is why
// exposure, bloom, tint and tone-curve tuning could never reach it, and why a whole evening of
// those produced improvement without ever producing a match.
//
// The forcing happens AFTER volumes blend in but BEFORE the component's own override is applied,
// so the component override is the one thing that survives -- the engine's comment says it is meant
// to be re-enabled this way. Nothing here is a trick.
//
// EDynamicGlobalIlluminationMethod: 0 None, 1 Lumen, 2 ScreenSpace, 3 Plugin  (EngineTypes.h:438)
// EReflectionMethod:                0 None, 1 Lumen, 2 ScreenSpace            (EngineTypes.h:459)
// Both are byte-sized on this build, so one uint8 write covers TEnumAsByte and enum-class alike.
void apply_capture_lumen(API::UObject* cap, int mode) {
    auto* cls = cap->get_class();
    auto* pps_prop = (cls != nullptr) ? cls->find_property(L"PostProcessSettings") : nullptr;
    auto* pps_struct = API::get()->find_uobject<API::UStruct>(
        L"ScriptStruct /Script/Engine.PostProcessSettings");
    if (pps_prop == nullptr || pps_struct == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] scope: lumen -- PostProcessSettings unresolved "
                             "(prop=%p struct=%p), NOTHING WRITTEN",
                             (void*)pps_prop, (void*)pps_struct);
        return;
    }
    auto* pps = reinterpret_cast<uint8_t*>(cap) + pps_prop->get_offset();

    struct Field { const wchar_t* value; const wchar_t* over; };
    const Field fields[] = {
        { L"DynamicGlobalIlluminationMethod", L"bOverride_DynamicGlobalIlluminationMethod" },
        { L"ReflectionMethod",                L"bOverride_ReflectionMethod" },
    };
    int applied = 0;
    for (const auto& f : fields) {
        auto* vp = pps_struct->find_property(f.value);
        auto* op = pps_struct->find_property(f.over);
        if (vp == nullptr || op == nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] scope: lumen field NOT FOUND (%s / %s)",
                                 narrow(f.value).c_str(), narrow(f.over).c_str());
            continue;
        }
        *(pps + vp->get_offset()) = (uint8_t)mode;
        if (auto* bp = static_cast<API::FBoolProperty*>(op)) {
            auto* byte = pps + bp->get_offset();
            *byte = (uint8_t)(*byte | bp->get_field_mask());
        }
        ++applied;
    }

    // The surface cache is halved for captures too (:885). Restoring it only matters once Lumen is
    // actually on, so it rides along rather than getting a key of its own.
    if (mode > 0) {
        auto* vp = pps_struct->find_property(L"LumenSurfaceCacheResolution");
        auto* op = pps_struct->find_property(L"bOverride_LumenSurfaceCacheResolution");
        if (vp != nullptr && op != nullptr) {
            *reinterpret_cast<float*>(pps + vp->get_offset()) = 1.0f;
            if (auto* bp = static_cast<API::FBoolProperty*>(op)) {
                auto* byte = pps + bp->get_offset();
                *byte = (uint8_t)(*byte | bp->get_field_mask());
            }
        }
    }

    // READ BACK both, because "wrote a byte" and "the renderer used it" are different claims and
    // this whole feature exists because one of them was assumed for weeks.
    int gi_back = -1, refl_back = -1;
    if (auto* p = pps_struct->find_property(L"DynamicGlobalIlluminationMethod"))
        gi_back = (int)*(pps + p->get_offset());
    if (auto* p = pps_struct->find_property(L"ReflectionMethod"))
        refl_back = (int)*(pps + p->get_offset());
    static const char* kName[] = { "None", "Lumen", "ScreenSpace", "Plugin" };
    API::get()->log_info("[Halo-CampE-UEVR] scope: lumen -> mode %d (%s) -- %d of 2 fields applied, "
                         "readback GI=%d refl=%d. The engine forces BOTH to None for every scene "
                         "capture (SceneCaptureRendering.cpp:881); this overrides that, which is "
                         "the documented way to re-enable it.",
                         mode, (mode >= 0 && mode <= 3) ? kName[mode] : "?", applied,
                         gi_back, refl_back);
}

// ENABLE DEPTH OF FIELD ON THE CAPTURE. TRIED AS THE SHIELD-COLOUR FIX; IT IS NOT ONE.
//
// MEASURED 2026-08-16: writes cleanly (2 of 2 fields) with scopesrc=8 + scopepersist=0, and the
// shields' colour still does not appear. Kept because the write is correct and DOF may be wanted
// for other reasons -- but do not re-run it expecting shields.
//
// The theory behind it was WRONG and is written down so nobody rebuilds it: the shield census
// reports the material's TranslucencyPass as 1, which was read as TPT_TranslucencyAfterDOFModulate
// -- a member of ETranslucencyPass, the RENDERER'S enum. The property is EMaterialTranslucencyPass
// (Material.h:140): MTP_BeforeDOF=0, MTP_AfterDOF=1, MTP_AfterMotionBlur=2. The value means plain
// After DOF; the separate modulate buffer the argument depended on never entered into it.
//
// IsEnabled (DiaphragmDOF.cpp:1374) wants Fstop > 0 AND FocalDistance > 0, both ordinary
// FPostProcessSettings fields. Same reflection discipline as the exposure pin: resolve offsets
// from the struct, never hardcode, set the paired bOverride_, and COUNT what applied so a null
// result cannot be confused with a write that went nowhere.
void enable_capture_dof(API::UObject* cap, float fstop, float focus) {
    auto* cls = cap->get_class();
    if (cls == nullptr) return;
    auto* pps_prop = cls->find_property(L"PostProcessSettings");
    if (pps_prop == nullptr) return;
    auto* pps = reinterpret_cast<uint8_t*>(cap) + pps_prop->get_offset();
    auto* pps_struct = API::get()->find_uobject<API::UStruct>(
        L"ScriptStruct /Script/Engine.PostProcessSettings");
    if (pps_struct == nullptr) return;

    struct F { const wchar_t* value; const wchar_t* over; float v; };
    const F fields[] = {
        { L"DepthOfFieldFstop",         L"bOverride_DepthOfFieldFstop",         fstop },
        { L"DepthOfFieldFocalDistance", L"bOverride_DepthOfFieldFocalDistance", focus },
    };
    int applied = 0;
    for (const auto& f : fields) {
        auto* vp = pps_struct->find_property(f.value);
        auto* op = pps_struct->find_property(f.over);
        if (vp == nullptr || op == nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] scope: DOF field NOT FOUND (%s / %s)",
                                 narrow(f.value).c_str(), narrow(f.over).c_str());
            continue;
        }
        *reinterpret_cast<float*>(pps + vp->get_offset()) = f.v;
        if (auto* bp = static_cast<API::FBoolProperty*>(op)) {
            auto* byte = pps + bp->get_offset();
            *byte = (uint8_t)(*byte | bp->get_field_mask());
        }
        ++applied;
    }
    API::get()->log_info("[Halo-CampE-UEVR] scope: capture DOF -- %d of %d fields applied "
                         "(fstop %.1f, focus %.0f cm)%s%s", applied,
                         (int)(sizeof(fields) / sizeof(fields[0])), fstop, focus,
                         applied == 0 ? "  <-- NOTHING WRITTEN, treat any result as untested" : "",
                         (g_cfg.scope_capture_src == 0)
                             ? "  <-- scopesrc=0 SKIPS THE POST CHAIN; DOF cannot run, use scopesrc=8"
                             : "");
}

#if HALO_VR_DEV
// Read a D3D12 resource description behind SEH. Separated into its own function with NO C++
// objects in scope because __try cannot coexist with unwinding, and guarded at all because the
// pointer comes back from an engine hook we have never called on this title -- a bad pointer here
// would take the game down in the player's headset, which is not an acceptable way to learn that
// an API is unavailable.
static bool safe_get_d3d12_desc(void* res, unsigned* w, unsigned* h, int* fmt, int* dim) {
    if (res == nullptr) return false;
    __try {
        D3D12_RESOURCE_DESC d = ((ID3D12Resource*)res)->GetDesc();
        *w = (unsigned)d.Width; *h = (unsigned)d.Height;
        *fmt = (int)d.Format;   *dim = (int)d.Dimension;
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

#if HALO_VR_DEV
// READ THE CAPTURE'S OWN PIXELS -- settle "is the glow in the RT" before any display-side work.
//
// A composition layer cannot add a halo the capture never rendered. So before committing to
// compositor work on the strength of "the pane must be destroying it", look at what the render
// target actually contains. UKismetRenderingLibrary::ReadRenderTargetRawPixel is BlueprintCallable
// and returns an FLinearColor; with bNormalize FALSE the HDR values come back un-clamped, which is
// the whole point -- an 8-bit read would saturate the bright core and hide the very falloff being
// looked for.
//
// Reads a horizontal line through the middle of the target and logs relative luminance. A bloom
// halo is a GRADUAL falloff either side of a bright core; an unbloomed emissive is a bright core
// with an abrupt edge. The corner sample is a sanity check on the marshalling: if that does not
// come back near-black, the parameter layout is wrong and the whole row is meaningless.
void dev_rt_scan(uint32_t tick) {
    // RE-ARMS on a 0 -> 1 transition. It used to latch on a plain static and never fire again,
    // which meant every re-aim needed a game restart -- unusable for a test whose whole method is
    // "point at a different thing and look again".
    static bool armed = false;
    static bool done  = false;
    if (!g_cfg.scope_rt_scan) { armed = true; return; }
    if (armed) { armed = false; done = false; }
    if (done) return;
    auto* rt = s_rt.get_checked(L"TextureRenderTarget2D");
    if (rt == nullptr) return;
    auto* pc = API::get()->get_player_controller(0);
    if (pc == nullptr) return;
    static API::UObject* krl = nullptr;
    if (krl == nullptr) {
        if (auto* c = API::get()->find_uobject<API::UClass>(
                L"Class /Script/Engine.KismetRenderingLibrary")) {
            krl = c->get_class_default_object();
        }
        if (krl == nullptr) return;
    }
    done = true;

    const int dim = (g_cfg.scope_rt_size > 0) ? g_cfg.scope_rt_size : 512;
    auto read_px = [&](int x, int y, float* out) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<void**>(p)      = pc;      // WorldContextObject
        *reinterpret_cast<void**>(p + 8)  = rt;      // TextureRenderTarget
        *reinterpret_cast<int32_t*>(p + 16) = x;
        *reinterpret_cast<int32_t*>(p + 20) = y;
        p[24] = 0;                                   // bNormalize = false -> raw HDR
        krl->call_function(L"ReadRenderTargetRawPixel", p);
        const auto* c = reinterpret_cast<const float*>(p + 28);
        out[0] = c[0]; out[1] = c[1]; out[2] = c[2]; out[3] = c[3];
    };

    float px[4] = {0,0,0,0};
    read_px(2, 2, px);
    API::get()->log_info("[Halo-CampE-UEVR] RTSCAN corner(2,2) = %.3f %.3f %.3f a=%.3f "
                         "(sanity: should be the scene's darkest corner, NOT garbage)",
                         px[0], px[1], px[2], px[3]);

    // RADIAL FALLOFF FROM THE BRIGHTEST POINT -- the measurement that actually answers the
    // question. Two horizontal-row scans were run before this and BOTH were uninterpretable: 24
    // samples across 1024 px is one every 43 px, which aliases scene detail into noise, and a
    // bloom halo is smooth at that scale. The row could not tell a halo from a bright object, no
    // matter how the shot was framed.
    //
    // A halo IS a falloff around a core, so measure exactly that: find the brightest pixel, then
    // sample outward from it at increasing radii (averaged over four directions so one dark
    // neighbour cannot skew a radius). Then report the radius at which brightness reaches half and
    // a tenth of the peak.
    //   half-brightness many pixels out, tenth further still -> a long soft tail = BLOOM.
    //   half and tenth within a few pixels                   -> a hard edge = the object itself,
    //                                                           no bloom in the capture.
    // This is robust to framing and to zoom, which the row scan was not.
    //
    // ~290 ReadRenderTargetRawPixel calls in one tick. That is a real hitch, which is why this is
    // one-shot, dev-only, and re-armed deliberately rather than run on a timer.
    int bx = 0, by = 0; float bl = -1.0f;
    for (int gy = 0; gy < 16; ++gy) {
        for (int gx = 0; gx < 16; ++gx) {
            const int x = (dim - 1) * gx / 15, yy = (dim - 1) * gy / 15;
            read_px(x, yy, px);
            const float lum = 0.2126f * px[0] + 0.7152f * px[1] + 0.0722f * px[2];
            if (lum > bl) { bl = lum; bx = x; by = yy; }
        }
    }

    static const int kRadii[] = { 0, 1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64, 96 };
    char prof[512]; int pn = 0;
    float half_r = -1.0f, tenth_r = -1.0f;
    int flat_radii = 0; bool still_flat = true;
    for (int i = 0; i < (int)(sizeof(kRadii) / sizeof(kRadii[0])); ++i) {
        const int r = kRadii[i];
        float acc = 0.0f; int cnt = 0;
        const int offs[4][2] = { { r, 0 }, { -r, 0 }, { 0, r }, { 0, -r } };
        for (int d = 0; d < 4; ++d) {
            const int x = bx + offs[d][0], yy = by + offs[d][1];
            if (x < 0 || yy < 0 || x >= dim || yy >= dim) continue;
            read_px(x, yy, px);
            acc += 0.2126f * px[0] + 0.7152f * px[1] + 0.0722f * px[2];
            ++cnt;
            if (r == 0) break;   // centre is one sample, not four
        }
        const float lum = (cnt > 0) ? acc / (float)cnt : 0.0f;
        if (still_flat) {
            if (lum >= bl * 0.999f) ++flat_radii; else still_flat = false;
        }
        if (half_r  < 0.0f && lum <= 0.50f * bl) half_r  = (float)r;
        if (tenth_r < 0.0f && lum <= 0.10f * bl) tenth_r = (float)r;
        if (pn < (int)sizeof(prof) - 16)
            pn += snprintf(prof + pn, sizeof(prof) - (size_t)pn, "%d:%.0f ", r, lum);
    }
    API::get()->log_info("[Halo-CampE-UEVR] RTSCAN peak %.0f at (%d,%d); radial falloff %s",
                         bl, bx, by, prof);

    // CLIPPING, checked before the falloff is interpreted. If the innermost radii all read the
    // SAME value, the capture is saturating and the halo has been flattened away inside the render
    // target -- in which case the falloff below describes a clipped plateau, not the effect, and
    // no display-side change can recover it. First seen by eye as 11 samples pinned at exactly
    // 537.99; measured here so nobody has to notice repeated numbers in a log line again.
    if (flat_radii >= 3) {
        API::get()->log_info("[Halo-CampE-UEVR] RTSCAN CLIPPING: the innermost %d radii all read "
                             "%.0f -- the capture is SATURATING, so the halo is flattened inside "
                             "the RT and the falloff below is a plateau, not the effect. Raise "
                             "scopeexposure until this line stops appearing, then re-run.",
                             flat_radii, bl);
    }
    API::get()->log_info("[Halo-CampE-UEVR] RTSCAN half-brightness at r=%.0f px, tenth at r=%.0f px "
                         "(-1 = never reached within 96 px). A LONG soft tail = bloom IS in the "
                         "capture, so the pane is the problem. Half AND tenth within ~4 px = a hard "
                         "edge, no bloom was captured, and no display change fixes it.",
                         half_r, tenth_r);
}
#endif

// THE DIGITAL-ZOOM FEASIBILITY PROBE.
//
// The whole scope-FX problem exists because we RE-RENDER the scene into a capture, and something
// about that path loses the effects. The alternative is to stop re-rendering: sample the frame the
// engine already produced -- which has every effect correct by construction, and costs no second
// scene pass -- and magnify a crop of it onto the pane, with the crop factor driven by Blam's own
// per-weapon GetZoomMagnification.
//
// That plan needs one thing to be true, and this measures it rather than trusting a header: that
// UEVR's StereoHook answers on THIS title and hands back a real native texture. It also reports
// the surface's SIZE, which is the hard ceiling on magnification -- a crop blown up past the
// source resolution is mush, and a sniper's 8x is a small patch of it.
void dev_scene_rt_probe(uint32_t tick) {
    if (!g_cfg.scene_rt_probe) return;
    static bool done = false;
    if (done) return;
    if ((tick % 32) != 5) return;   // let the stereo hook settle before the first call
    done = true;

    auto* scene = uevr::API::StereoHook::get_scene_render_target();
    auto* ui    = uevr::API::StereoHook::get_ui_render_target();
    void* scene_native = (scene != nullptr) ? scene->get_native_resource() : nullptr;
    void* ui_native    = (ui    != nullptr) ? ui->get_native_resource()    : nullptr;

    API::get()->log_info("[Halo-CampE-UEVR] SCENERT: sceneTex=%p native=%p | uiTex=%p native=%p",
                         (void*)scene, scene_native, (void*)ui, ui_native);

    unsigned w = 0, h = 0; int fmt = 0, dim = 0;
    if (safe_get_d3d12_desc(scene_native, &w, &h, &fmt, &dim)) {
        API::get()->log_info("[Halo-CampE-UEVR] SCENERT: scene resource %ux%u fmt=%d dim=%d -- "
                             "DIGITAL ZOOM IS VIABLE; at %ux%u a crop for Nx magnification is "
                             "%u px wide", w, h, fmt, dim, w, h, (w ? w / 8u : 0u));
    } else {
        API::get()->log_info("[Halo-CampE-UEVR] SCENERT: could not read the scene resource desc "
                             "(null or not a live D3D12 resource) -- digital zoom would need "
                             "another source");
    }
    if (safe_get_d3d12_desc(ui_native, &w, &h, &fmt, &dim)) {
        API::get()->log_info("[Halo-CampE-UEVR] SCENERT: ui resource %ux%u fmt=%d", w, h, fmt);
    }
}
#endif

// FORCE BLOOM ONTO THE CAPTURE.
//
// The whole FX complaint reduces to "right hue, no glow", and glow is bloom, and bloom is post.
// A capture only runs post on a final-colour source, but even then it uses ITS OWN
// FPostProcessSettings -- default-constructed by the component, not the player camera's. So a
// post-processed capture can still render every effect minus its bloom. This writes bloom in
// directly, with the same reflection discipline as the exposure pin: resolve offsets from the
// struct, set the paired bOverride_ bit, and COUNT what applied so a null result cannot be
// confused with a write that went nowhere.
//
// The threshold is dropped to -1 (bloom everything) on purpose: an additive shield or a plasma
// bolt is not necessarily above a default brightness threshold, and "bloom is on but nothing
// clears the bar" would look identical to "bloom is off".
void force_capture_bloom(API::UObject* cap, float intensity) {
    auto* cls = cap->get_class();
    if (cls == nullptr) return;
    auto* pps_prop = cls->find_property(L"PostProcessSettings");
    if (pps_prop == nullptr) return;
    auto* pps = reinterpret_cast<uint8_t*>(cap) + pps_prop->get_offset();
    auto* pps_struct = API::get()->find_uobject<API::UStruct>(
        L"ScriptStruct /Script/Engine.PostProcessSettings");
    if (pps_struct == nullptr) return;

    struct F { const wchar_t* value; const wchar_t* over; float v; };
    const F fields[] = {
        { L"BloomIntensity", L"bOverride_BloomIntensity", intensity },
        { L"BloomThreshold", L"bOverride_BloomThreshold", -1.0f },
    };
    int applied = 0;
    for (const auto& f : fields) {
        auto* vp = pps_struct->find_property(f.value);
        auto* op = pps_struct->find_property(f.over);
        if (vp == nullptr || op == nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] scope: bloom field NOT FOUND (%s / %s)",
                                 narrow(f.value).c_str(), narrow(f.over).c_str());
            continue;
        }
        *reinterpret_cast<float*>(pps + vp->get_offset()) = f.v;
        if (auto* bp = static_cast<API::FBoolProperty*>(op)) {
            auto* byte = pps + bp->get_offset();
            *byte = (uint8_t)(*byte | bp->get_field_mask());
        }
        ++applied;
    }
    API::get()->log_info("[Halo-CampE-UEVR] scope: capture bloom -- %d of %d fields applied "
                         "(intensity %.2f, threshold -1)%s%s", applied,
                         (int)(sizeof(fields) / sizeof(fields[0])), intensity,
                         applied == 0 ? "  <-- NOTHING WRITTEN, treat any result as untested" : "",
                         (g_cfg.scope_capture_src == 0)
                             ? "  <-- scopesrc=0 RUNS NO POST AT ALL; bloom cannot appear, use scopesrc=8"
                             : "");
}

// Local path builder -- Plugin.cpp has its own outer_path_of, but this file cannot see it and a
// second copy of four lines beats an extern into another translation unit.
std::string scope_path_of(API::UObject* o) {
    std::string path;
    for (API::UObject* q = o; q != nullptr; q = q->get_outer()) {
        const auto* fn = q->get_fname();
        path = (fn != nullptr ? narrow(fn->to_string()) : std::string("?"))
             + (path.empty() ? "" : "." + path);
    }
    return path;
}

// NOT dev-gated: the shipping blendable copy calls this to score candidate sources, and a
// HALO_VR_DEV-only definition compiles fine in the dev build while breaking the RELEASE
// build -- a failure the normal build script never shows. Verified with -Release.
int blendable_count(API::UObject* obj, const wchar_t* pp_field) {
    if (obj == nullptr) return -1;
    auto* cls = obj->get_class();
    if (cls == nullptr) return -1;
    auto* pp = cls->find_property(pp_field);
    if (pp == nullptr) return -1;
    auto* pps_struct = API::get()->find_uobject<API::UStruct>(
        L"ScriptStruct /Script/Engine.PostProcessSettings");
    if (pps_struct == nullptr) return -1;
    auto* wb = pps_struct->find_property(L"WeightedBlendables");
    if (wb == nullptr) return -1;
    auto* base = reinterpret_cast<uint8_t*>(obj) + pp->get_offset() + wb->get_offset();
    const int32_t num = *reinterpret_cast<int32_t*>(base + 8);
    return (num >= 0 && num < 4096) ? (int)num : -1;   // a wild count means a wrong offset
}

#if HALO_VR_DEV
// HOW MANY POST-PROCESS BLENDABLES DOES THE CAPTURE HAVE, VERSUS THE GAME'S OWN SOURCES?
//
// This probe is what found the actual structural difference: MEASURED 2026-08-25, the player's
// camera carries EIGHT weighted blendables and our capture carries ZERO. A post-process material
// (the *PP assets) attaches to a camera or volume as a blendable, never to a mesh -- so an entire
// class of effect was absent from the capture because it was never asked for.
//
// Counting answers it. FPostProcessSettings::WeightedBlendables is a struct whose sole member is a
// TArray, and a TArray's Num sits 8 bytes in (ptr, Num, Max).

void dev_blendable_probe(API::UObject* cap) {
    if (!g_cfg.shield_census) return;
    static bool done = false;
    if (done || cap == nullptr) return;
    done = true;

    const int cap_n = blendable_count(cap, L"PostProcessSettings");
    int best_n = -1; std::string best_name = "<none found>";
    if (auto* arr = API::get()->get_uobject_array()) {
        const int32_t n = arr->get_object_count();
        for (int32_t i = 0; i < n; ++i) {
            auto* o = reinterpret_cast<API::UObject*>(arr->get_object(i));
            if (o == nullptr) continue;
            const std::wstring cn = class_name_of(o);
            const bool vol = cn.find(L"PostProcessVolume") != std::wstring::npos;
            const bool cam = cn.find(L"CameraComponent") != std::wstring::npos;
            if (!vol && !cam) continue;
            auto* oc = o->get_class();
            if (oc != nullptr && o == oc->get_class_default_object()) continue;
            const int cnt = blendable_count(o, vol ? L"Settings" : L"PostProcessSettings");
            if (cnt > best_n) { best_n = cnt; best_name = scope_path_of(o); }
        }
    }

    API::get()->log_info("[Halo-CampE-UEVR] BLENDABLES: our capture has %d, the game's richest "
                         "source (%s) has %d. If the game's count is >0 and ours is 0, its "
                         "post-process EFFECT MATERIALS (the *PP assets, e.g. "
                         "MI_OverShieldActivatePP) are attached where our capture never looks -- a "
                         "structural absence, not an exposure or bloom problem.",
                         cap_n, best_name.c_str(), best_n);
}
#endif

// COPY THE GAME'S POST-PROCESS BLENDABLES ONTO THE CAPTURE.
//
// MEASURED 2026-08-25: the player's camera carries EIGHT weighted blendables; our capture carries
// ZERO. A post-process material (the *PP assets -- MI_OverShieldActivatePP, MI_OvershieldBreakPP
// and friends) is not attached to a mesh: it hangs off a camera or a volume as a blendable, and
// the renderer applies it during that view's post chain. Our SceneCaptureComponent2D is
// constructed with an empty list, so an entire CLASS of effect was never asked for -- a structural
// absence, not a rendering failure, and nothing about exposure, bloom or capture source could ever
// have recovered it.
//
// THIS REPLACES A RAW memcpy OF THE WHOLE STRUCT, WHICH WAS UNSAFE.
// FPostProcessSettings contains a TArray. Copying the struct byte-for-byte duplicates the array's
// POINTER, so two objects end up owning one heap allocation -- a double free waiting for whichever
// destructs or reallocates first. It was default-off and the crash family here predates it by
// weeks, so it is not the cause of anything observed; it was still a heap-corruption hazard
// written during a session that was chasing heap-corruption-shaped crashes.
//
// The safe mechanism is the engine's own: USceneCaptureComponent2D::AddOrUpdateBlendable is
// BlueprintCallable (SceneCaptureComponent2D.h:236) and forwards to FPostProcessSettings::
// AddBlendable, so UE owns the allocation and the array grows properly.
//
// Every offset and stride below is RESOLVED from the reflected struct -- never a sizeof and never
// a hardcoded stride. FWeightedBlendable's layout is not something to assume: a wrong stride walks
// off the end of the array and hands garbage pointers to a UFUNCTION.
bool copy_pp_settings_onto_capture(API::UObject* cap) {
    // ENTRY TRACE. Without it, "the function was never called" and "it returned early"
    // produce the same evidence: no log line at all. That ambiguity cost a test cycle.
    API::get()->log_info("[Halo-CampE-UEVR] scope: blendable copy STARTING (cap=%p)",
                         (void*)cap);
    auto* pps_struct = API::get()->find_uobject<API::UStruct>(
        L"ScriptStruct /Script/Engine.PostProcessSettings");
    auto* wb_struct = API::get()->find_uobject<API::UStruct>(
        L"ScriptStruct /Script/Engine.WeightedBlendable");
    if (pps_struct == nullptr || wb_struct == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] scope: blendable structs not reflected on this "
                             "build -- cannot copy");
        return false;
    }
    auto* wb_prop = pps_struct->find_property(L"WeightedBlendables");
    auto* w_prop  = wb_struct->find_property(L"Weight");
    auto* o_prop  = wb_struct->find_property(L"Object");
    const int stride = wb_struct->get_properties_size();
    if (wb_prop == nullptr || w_prop == nullptr || o_prop == nullptr ||
        stride <= 0 || stride > 256) {
        API::get()->log_info("[Halo-CampE-UEVR] scope: blendable layout unresolved "
                             "(wb=%p w=%p o=%p stride=%d) -- refusing to walk it",
                             (void*)wb_prop, (void*)w_prop, (void*)o_prop, stride);
        return false;
    }

    // THE SOURCE IS THE CAMERA, not a PostProcessVolume. The previous version preferred a volume
    // and dutifully copied 1840 bytes from one -- while the eight blendables sat on the pawn's
    // camera, which it never looked at. Volume stays as the fallback.
    const wchar_t* src_field = nullptr;
    std::string src_name;
    API::UObject* src = pick_pp_source(&src_field, &src_name);
    if (src == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] scope: no usable CameraComponent or "
                             "PostProcessVolume to take blendables from (templates and "
                             "0-blendable sources are rejected)");
        return false;
    }

    auto* src_cls = src->get_class();
    auto* src_pp = (src_cls != nullptr) ? src_cls->find_property(src_field) : nullptr;
    auto* cap_cls = cap->get_class();
    auto* cap_pp = (cap_cls != nullptr) ? cap_cls->find_property(L"PostProcessSettings") : nullptr;
    if (src_pp == nullptr || cap_pp == nullptr) {
        // THE ONLY PATH IN THIS FUNCTION THAT USED TO RETURN IN SILENCE, and therefore the one it
        // actually took. Every other failure announced itself; this one left no trace, so an
        // enabled scopeppcopy that did nothing was indistinguishable from a call that never
        // happened. Name both pointers and the field that was looked up.
        API::get()->log_info("[Halo-CampE-UEVR] scope: blendable copy ABORTED -- src '%s'.%s = %p, "
                             "capture.PostProcessSettings = %p (a null here means the property "
                             "name is wrong for this source class)",
                             src_name.c_str(), narrow(src_field).c_str(), (void*)src_pp,
                             (void*)cap_pp);
        return false;
    }

    // The TArray inside FWeightedBlendables: {data ptr, Num, Max}.
    auto* arr_base = reinterpret_cast<uint8_t*>(src) + src_pp->get_offset() + wb_prop->get_offset();
    auto* elems = *reinterpret_cast<uint8_t**>(arr_base);
    const int32_t num = *reinterpret_cast<int32_t*>(arr_base + 8);
    if (elems == nullptr || num <= 0 || num > 256) {
        API::get()->log_info("[Halo-CampE-UEVR] scope: source %s has no usable blendables "
                             "(ptr=%p num=%d)", src_name.c_str(), (void*)elems, num);
        return false;
    }

    int added = 0;
    for (int32_t i = 0; i < num; ++i) {
        auto* e = elems + (size_t)i * (size_t)stride;
        const float weight = *reinterpret_cast<float*>(e + w_prop->get_offset());
        auto* obj = *reinterpret_cast<API::UObject**>(e + o_prop->get_offset());
        if (obj == nullptr) continue;
        // TScriptInterface is {UObject* ObjectPointer; void* InterfacePointer;}. AddBlendable uses
        // GetObject(), i.e. the object pointer, so the interface half is left null deliberately.
        alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<void**>(q)      = obj;
        *reinterpret_cast<void**>(q + 8)  = nullptr;
        *reinterpret_cast<float*>(q + 16) = weight;
        cap->call_function(L"AddOrUpdateBlendable", q);
        ++added;
    }

    // READ BACK. "Called a UFUNCTION" is not "the array grew" -- this whole session has been a
    // lesson in the difference, so the count is re-read rather than assumed.
    auto* cap_arr = reinterpret_cast<uint8_t*>(cap) + cap_pp->get_offset() + wb_prop->get_offset();
    const int32_t after = *reinterpret_cast<int32_t*>(cap_arr + 8);
    API::get()->log_info("[Halo-CampE-UEVR] scope: blendables from %s -- source had %d, attempted "
                         "%d, capture now reports %d%s", src_name.c_str(), num, added, after,
                         (after <= 0) ? "  <-- NOTHING LANDED, treat any result as untested" : "");
    return after > 0;
}

// FORCE AN ANTI-ALIASING METHOD ON THE CAPTURE.
//
// The open question this exists to answer: post-processed capture sources (FinalColorLDR/HDR,
// FinalToneCurveHDR) render SOLID BLACK on this title, while scene-colour sources work but have
// post-processing force-disabled by the engine -- which is why shield/FX shading never arrives.
// This game ships a temporal upscaler (DLSS/Streamline; the day-one work needed Upscaler=Off to
// fix a Present failure), and SceneCaptureRendering.cpp notes screen percentage is unsupported in
// scene captures, so a post chain expecting an upscaler pass that never ran is a plausible cause
// of the black. Forcing a NON-temporal AA on the capture alone tests that without touching the
// player's main view -- the console cannot be used here (it is inert on this title).
//
// EAntiAliasingMethod: 0 None, 1 FXAA, 2 TAA, 3 MSAA, 4 TSR. Same reflection discipline as the
// exposure pin: resolve offsets from the struct, never hardcode, and set the paired bOverride_.
void set_capture_aa(API::UObject* cap, int method) {
    auto* cls = cap->get_class();
    if (cls == nullptr) return;
    auto* pps_prop = cls->find_property(L"PostProcessSettings");
    if (pps_prop == nullptr) return;
    auto* pps = reinterpret_cast<uint8_t*>(cap) + pps_prop->get_offset();

    auto* pps_struct = API::get()->find_uobject<API::UStruct>(
        L"ScriptStruct /Script/Engine.PostProcessSettings");
    if (pps_struct == nullptr) return;

    auto* vp = pps_struct->find_property(L"AntiAliasingMethod");
    auto* op = pps_struct->find_property(L"bOverride_AntiAliasingMethod");
    if (vp == nullptr || op == nullptr) {
        // NOT a quirk of this build. `FPostProcessSettings` HAS NO AntiAliasingMethod IN UE 5.5 --
        // grep Engine/Classes/Engine/Scene.h and there is no such field. This whole function was
        // written against a property that does not exist in this engine version, so scopeaa has
        // never done anything and never can through this route.
        //
        // The view's AA is decided in FSceneView::SetupAntiAliasingMethod (SceneView.cpp:1067)
        // from the default method plus SHOW FLAGS, and for a scene-colour capture it lands on
        // AAM_None regardless, because EngineShowFlags.PostProcessing is force-disabled. Anything
        // that really needs to move AA has to go through the capture's ShowFlagSettings array.
        API::get()->log_info("[Halo-CampE-UEVR] scope: scopeaa DOES NOTHING -- FPostProcessSettings "
                             "has no AntiAliasingMethod in UE 5.5. Not a build quirk; the key is "
                             "inert by construction. Scene-colour captures are AAM_None anyway.");
        return;
    }
    *reinterpret_cast<uint8_t*>(pps + vp->get_offset()) = (uint8_t)method;
    if (auto* bp = static_cast<API::FBoolProperty*>(op)) {
        auto* byte = pps + bp->get_offset();
        *byte = (uint8_t)(*byte | bp->get_field_mask());
    }
    API::get()->log_info("[Halo-CampE-UEVR] scope: capture AA forced to %d "
                         "(0 None 1 FXAA 2 TAA 3 MSAA 4 TSR)", method);
}

void set_tint(API::UObject* mid, float bright) {
    if (mid == nullptr) return;
    // "Color" is EmissiveMeshMaterial's brightness parameter (the uevrlib scope recipe);
    // TintColorAndOpacity is written too so a Widget3DPassThrough fallback material still tints.
    static const wchar_t* kNames[] = { L"Color", L"TintColorAndOpacity" };
    for (const wchar_t* name : kNames) {
        alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
        API::FName param = make_fname(name);
        memcpy(q, &param, sizeof(int32_t) * 2);
        auto* c = reinterpret_cast<float*>(q + 8);
        c[0] = bright; c[1] = bright; c[2] = bright; c[3] = 1.0f;
        mid->call_function(L"SetVectorParameterValue", q);
    }
}

// The scope's render target: RGBA16f, because the capture stores LINEAR HDR scene colour
// (SCS_SceneColorHDR). An LDR capture depends on the capture's own eye adaptation, which on this
// title sits orders of magnitude dark; linear HDR shown through an emissive material re-enters
// the MAIN view's tonemapper instead and inherits the player's exposure by construction -- the
// uevrlib scope recipe, field-proven across UE5 titles. Magenta clear = "no capture landed yet".
// Which ETextureRenderTargetFormat to allocate for a given capture source. An LDR source
// (FinalColorLDR) writes 8-bit non-linear colour; handing it a float target is a plausible reason
// for the solid black every post-processed source produces on this title, so the format follows
// the source instead of being fixed. 2 = RTF_RGBA8, 6 = RTF_RGBA16f.
int rt_format_for_source(int src) {
    return (src == 2) ? 2 : 6;
}

API::UObject* make_scope_rt(int size) {
    auto* pc = API::get()->get_player_controller(0);
    if (pc == nullptr) return nullptr;
    auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.KismetRenderingLibrary");
    auto* krl = (cls != nullptr) ? cls->get_class_default_object() : nullptr;
    if (krl == nullptr) return nullptr;

    API::UObject* rt = nullptr;
    {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<void**>(p) = pc;
        *reinterpret_cast<int32_t*>(p + 8)  = size;
        *reinterpret_cast<int32_t*>(p + 12) = size;
        p[16] = (uint8_t)rt_format_for_source(g_cfg.scope_capture_src);
        auto* cc = reinterpret_cast<float*>(p + 20);
        cc[0] = 1.0f; cc[1] = 0.0f; cc[2] = 1.0f; cc[3] = 1.0f;
        krl->call_function(L"CreateRenderTarget2D", p);
        rt = *reinterpret_cast<API::UObject**>(p + 40);
    }
    if (rt == nullptr) return nullptr;
    {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<void**>(p)     = pc;
        *reinterpret_cast<void**>(p + 8) = rt;
        auto* cc = reinterpret_cast<float*>(p + 16);
        cc[0] = 1.0f; cc[1] = 0.0f; cc[2] = 1.0f; cc[3] = 1.0f;
        krl->call_function(L"ClearRenderTarget2D", p);
    }
    API::get()->log_info("[Halo-CampE-UEVR] scope: HDR render target created @%p (%dx%d RGBA16f)",
                         (void*)rt, size, size);
    return rt;
}

// Create whichever of the three pieces is missing. Piecewise on purpose: a death/respawn kills
// the pawn-outered components while the PC-outered render target survives, and a level change
// kills all three -- each simply gets remade against the new owners.
// WHY ensure_components LAST REFUSED. Both of its early returns were silent, and they are the
// two that strand the scope in its worst state: reporting itself ACTIVE with no pane, no
// capture and not one line of explanation. s_failed in particular is a LATCH -- once set the
// scope is dead for the session and every later tick returns false without a word.
const char* s_ec_bail = nullptr;

// PER-TICK. Hoisted out of ensure_components() on 2026-09-06, and that move IS the fix for
// "the pane is in the right place but does not follow the weapon".
//
// ensure_components() SOUNDS like a per-tick ensure and is not: all three of its call sites sit
// inside rebuild conditions (a shape change, an RT size change, a capture-source format change).
// It therefore runs once when the pane is built and then essentially never. With the attach living
// inside it, the space-switch handshake could not work by construction -- phase one ran at build
// time, when the pane is BY DEFINITION not yet anchored, and phase two needed a later tick that
// never came. The log said so exactly once and then went quiet: one `attached to rig`, one
// `holding the pane on the RIG ... anchored=0 placement_changed=1`, and nothing further.
//
// This is the same shape of mistake as the mode-3 repair that was hosted on reticule_widget_move():
// a function whose NAME implies a cadence it does not have. Cheap to call every tick -- the
// change-detect below means the steady state is two pointer compares.
// ---- IS THE SOCKET BONE STILL ENOUGH TO CONVERT AGAINST? ---------------------------------------
//
// The space-switch handshake attaches with KeepWorld, so the engine computes rel = socket^-1 * world
// USING THE SOCKET AS IT IS AT THAT INSTANT. The pane then rides socket(t) * rel forever after, which
// equals the intended world transform only while the bone is near the pose it held during the
// conversion. Convert during a weapon DRAW -- which is exactly when this fires, right after a switch
// -- and the bone's displacement at that moment is baked in permanently.
//
// MEASURED 2026-09-07, from five conversions of the IDENTICAL rig-frame calibration 27.813/13.183/
// 2.051 in one session: scoperight came out -9.131, -10.716, -10.273 and -8.332 on four of them, and
// -30.798 on the fifth, whose yaw was also ~18 degrees off the other four. A 20 cm error from the
// same input, with nothing in the log calling it a failure. That fifth one is the reported "it did
// not place the quad properly", and cycling weapons cleared it because the next handshake happened
// to catch a quieter moment.
//
// So gate the conversion on the bone being SLOW, which separates a draw or reload (many cm per tick)
// from idle sway (well under one). Not on being motionless: the bone never is, and a gate nothing can
// satisfy is worse than no gate.
//
// BOUNDED, like every other wait in this file: if the bone never settles -- the player is running,
// or an animation loops -- convert anyway once the deadline passes and say so. Waiting costs nothing
// visible (the pane is correctly placed in the rig frame throughout, it merely does not ride recoil
// yet), but waiting FOREVER would silently cost the weapon-following the handshake exists to give.
constexpr float    kSocketStillCm   = 0.75f;   // per tick; idle sway is well under this
constexpr uint32_t kSocketStillTicks = 3;      // consecutive quiet ticks before we trust it
constexpr uint32_t kSocketWaitTicks  = 48;     // ~1.5 s, then convert regardless
// ...AND NEAR WHERE THE BONE ACTUALLY LIVES. "Slow" is not "at rest", and the difference is the
// second half of this bug: an EQUIP animation is fast throughout, but a RELOAD has slow phases, and
// a firing cycle pauses between shots. Any of those can be under the per-tick threshold while the
// bone sits many centimetres from its rest pose -- so the gate passed, the conversion baked in that
// displacement, and because the deadline never fired it was never even marked as suspect.
//
// The rest pose is not something we can look up, so it is LEARNED: a slow exponential average of the
// socket's position. A weapon spends most of its time idle, so the average is dominated by rest, and
// requiring the current position to be near it rejects the slow parts of an animation that a
// velocity test alone waves through.
// The current rest estimate, published by socket_ready_to_convert for the audit below. A plain
// value, written and read on the game thread only.
Vec3 g_socket_rest_est{};
constexpr float kSocketRestCm  = 3.0f;    // how close to the learned rest pose counts as "at rest"
constexpr float kSocketRestEma = 0.02f;   // ~2% per tick: several seconds of memory

//
// `placing` IS A PARAMETER RATHER THAN A CALLER-SIDE SHORT-CIRCUIT, and that distinction was the
// whole bug in the first version of this gate. It was called as `placing || ready(...)`, so while a
// placement was in progress the function was NOT CALLED and its wait state froze. The weapon-switch
// sequence interrupts a wait exactly this way -- the per-weapon trim lands partway through and flips
// placement_changed, which is visible in the logs as `anchored=1 placement_changed=1` -- so the wait
// resumed with an ancient s_first_wait, `tick - s_first_wait >= kSocketWaitTicks` was ALREADY true,
// and the deadline fired on the first tick. The gate converted on a moving bone while appearing to
// work. Sampling every tick also keeps the velocity estimate continuous; a gap makes the first delta
// after it enormous and meaningless.
bool socket_ready_to_convert(API::UObject* rig, const wchar_t* socket, uint32_t tick,
                             bool placing, bool* out_timed_out) {
    static Vec3     s_prev{};
    static bool     s_prev_valid  = false;
    static uint32_t s_moved_tick  = 0;   // last tick the bone moved more than the threshold
    static uint32_t s_first_wait  = 0;   // when this wait began, for the deadline
    static bool     s_waiting     = false;
    *out_timed_out = false;

    Vec3 sp{};
    if (rig == nullptr || !call_socket_location(rig, socket, &sp)) {
        // Cannot measure -> do not withhold. An unmeasurable socket must degrade to the old
        // behaviour, not to a pane that never follows the weapon.
        s_prev_valid = false;
        s_waiting    = false;
        return true;
    }
    float moved = 1.0e9f;
    if (s_prev_valid) {
        const float dx = sp.x - s_prev.x, dy = sp.y - s_prev.y, dz = sp.z - s_prev.z;
        moved = std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    s_prev = sp;
    s_prev_valid = true;
    if (moved > kSocketStillCm) s_moved_tick = tick;

    // The learned rest pose. Seeded on the first sample so it is never far off at startup.
    static Vec3 s_rest{};
    static bool s_rest_valid = false;
    if (!s_rest_valid) { s_rest = sp; s_rest_valid = true; }
    else {
        s_rest.x += (sp.x - s_rest.x) * kSocketRestEma;
        s_rest.y += (sp.y - s_rest.y) * kSocketRestEma;
        s_rest.z += (sp.z - s_rest.z) * kSocketRestEma;
    }
    const float rdx = sp.x - s_rest.x, rdy = sp.y - s_rest.y, rdz = sp.z - s_rest.z;
    const float from_rest = std::sqrt(rdx * rdx + rdy * rdy + rdz * rdz);
    g_socket_rest_est = s_rest;   // published for the post-conversion audit in the caller

    // Sampling above happens even while placing, so the velocity stays continuous. The WAIT,
    // however, is restarted: the socket attach is not being requested yet, so any window opened
    // before now is meaningless and must not be allowed to expire in the background.
    if (placing) { s_waiting = false; return false; }

    if (!s_waiting) { s_waiting = true; s_first_wait = tick; }
    if ((tick - s_moved_tick) >= kSocketStillTicks && from_rest <= kSocketRestCm) {
        s_waiting = false;
        return true;
    }
    if ((tick - s_first_wait) >= kSocketWaitTicks) {
        if (g_cfg.scope_socket_wait == 0) {
            // Legacy: convert anyway. The caller marks it dirty and redoes it once the bone rests.
            s_waiting = false;
            *out_timed_out = true;
            return true;
        }
        // DEFAULT: keep waiting. Staying on the rig is correct placement without recoil-riding,
        // which is a far better resting state than a permanently wrong socket offset. Said once per
        // wait episode so a bone that never settles is visible rather than silent.
        static uint32_t s_said_wait = 0;
        if (s_said_wait != s_first_wait) {
            s_said_wait = s_first_wait;
            API::get()->log_info(
                "[Halo-CampE-UEVR] scope: the PrimaryWeapon bone has not reached its rest pose in "
                "%u ticks, so the space-switch conversion is being HELD. The pane is correctly "
                "placed on the rig and simply is not riding recoil yet; it will convert by itself "
                "the moment the weapon settles. Set scopesocketwait=0 to convert anyway.",
                kSocketWaitTicks);
        }
        return false;
    }
    return false;
}

// `tick` is the GAME TICK, used only by the socket-settle gate below; pass 0 from any caller
// that has no tick, which disables the gate. That is safe for ensure_components(), the only
// such caller: it runs inside a rebuild condition, when the pane is by definition not yet
// anchored, so `placing` is true and the gate is never consulted on that path anyway.
void update_pane_attachment(API::UObject* rig, bool ready, uint32_t tick) {
        API::UObject* want_parent = rig;
        const wchar_t* want_socket = nullptr;
        if (g_cfg.scope_parent == 1) {
            // THE SOCKET THE WEAPON ITSELF RIDES -- not the weapon actor. Rig.cpp establishes the
            // hierarchy: the first-person arms mesh carries socket `PrimaryWeapon`, and
            // `BP_FP_*_WeaponActor_C` is attached there ("the gun rides along"). Parenting the pane to
            // the same socket makes it ride EXACTLY what the gun rides, by construction, so recoil can
            // no longer drive the barrel through the pane.
            //
            // This is strictly better than hanging off the weapon actor, which is what the first
            // attempt did and which the player could walk away from:
            //   * NO WEAPON-ACTOR LIFECYCLE AT ALL. The socket lives on the rig we already track and
            //     already handle losing. A weapon swap, a holster, a death or a vehicle changes which
            //     actor sits at the socket and changes nothing about our attachment -- so the whole
            //     class of "parented to a pooled actor that got recycled" risk simply does not arise.
            //   * It is the animated frame. A component root does not move when the mesh animates; a
            //     socket is a bone transform, so it carries recoil, sway and reload motion.
            //
            // Evidence that this socket is genuinely animated, rather than an assumption: hiding the
            // arms froze this exact socket and the weapon visibly LOST ITS RECOIL (the armhide bug in
            // Rig.cpp). A socket whose freezing removes recoil is a socket that supplies recoil.
            //
            // ---- BUT ONLY ONCE THE PANE IS PLACED, AND THAT ORDERING IS THE WHOLE SPACE SWITCH ------
            //
            // scope_dist/right/up and the rot trims are RELATIVE TO THE ATTACH PARENT -- the capture
            // reads them straight off RelativeLocation/RelativeRotation. They were all authored, and
            // the shipped canonical fit was measured, with the pane parented to the RIG. Re-parenting to
            // a socket silently reinterprets every one of those numbers in a frame with a different
            // origin AND different axes, which is exactly what "this isn't the default scope calibration
            // position we had before" was (reported 2026-09-06).
            //
            // So the numbers are never converted, and never re-authored. Instead the placement happens
            // in the frame it was written for, and the ENGINE performs the space switch:
            //
            //   tick N   : not yet anchored -> want_socket stays null -> attached to the RIG ORIGIN.
            //              The placement block writes the calibrated relative loc/rot in the rig frame,
            //              which is what those numbers mean, and sets s_pane_anchored.
            //   tick N+1 : anchored -> attach at the socket with KeepWorld. The engine recomputes the
            //              relative offset that PRESERVES the pane's world transform, so the pane does
            //              not move and is now expressed in the socket's frame.
            //   after    : nothing rewrites the placement (the write is anchor-gated, not per-tick), so
            //              the attachment carries it and the socket's animation -- recoil -- rides on
            //              top as a delta.
            //
            // This is the same mechanism the calibration capture already relies on ("the engine has been
            // recomputing the relative transform against the rig on every frozen write, so the answer is
            // simply there to be read"). It needs no socket ROTATION function -- only GetSocketLocation
            // is on this build's probed list, and calling an absent UFUNCTION fails silently here -- and
            // no hand-rolled quaternion maths, which is a category of error this project has paid for
            // more than once.
            //
            // A placement edit drops us back to the rig for one tick and the handshake repeats, so live
            // tuning stays authored in the canonical frame. A capture does the same, so what it records
            // is always rig-frame and stays comparable with the shipped fit and with every calibration
            // players already have.
            // RE-RUN THE HANDSHAKE ON A WEAPON SWITCH.
        //
        // The socket itself does not change -- PrimaryWeapon lives on the arms mesh, not the gun --
        // but what hangs off it does, and so does the per-weapon trim that feeds the rig-frame
        // placement. A conversion computed for the previous weapon is then carried forward by an
        // attachment nobody re-evaluated, which is exactly the class of silent staleness this whole
        // lane keeps producing. Dropping the anchor forces one rig-frame placement and one fresh
        // KeepWorld conversion, costing two ticks per swap.
        //
        // Cheap: one class-name read on the held weapon, compared against the last one.
        {
            static std::wstring s_last_wpn;
            std::wstring wpn;
            if (auto* wa = fp_weapon_actor()) wpn = class_name_of(wa);
            if (wpn != s_last_wpn) {
                s_last_wpn = wpn;
                s_pane_anchored = false;   // re-place in the rig frame, then re-convert
            }
        }

        const bool placing = !s_pane_anchored || pane_placement_changed() || s_calib_held;
            // FOURTH REASON THE SOCKET CAN BE WITHHELD: the bone is mid-animation. Added after the
            // arming window taught this file the cost of introducing a state that the code
            // enumerating the existing ones does not know about -- so the log below names it too.
            bool bone_late = false;
            // CALLED UNCONDITIONALLY (except the no-tick caller), never short-circuited -- see the
            // note on the function. `placing` is passed in so it can keep sampling while restarting
            // its wait.
            const bool bone_settled =
                (tick != 0) &&
                socket_ready_to_convert(rig, L"PrimaryWeapon", tick, placing, &bone_late);
            const bool bone_ok = (tick == 0) || placing || bone_settled;

            // SELF-HEAL A CONVERSION TAKEN ON A MOVING BONE.
            //
            // The deadline exists so the pane always ends up weapon-following even if the bone never
            // settles -- but a conversion forced that way bakes in whatever displacement the bone had,
            // permanently, which is the original defect. So remember that it was forced and REDO it
            // once the bone is genuinely quiet. The handshake is cheap and idempotent (drop the
            // anchor, one rig-frame placement, one fresh KeepWorld conversion), and it cannot
            // oscillate: the redo happens only when the bone is settled, so the replacement
            // conversion is clean and clears the flag.
            //
            // This is what makes raising the scope DURING a weapon-draw animation recoverable
            // without the player cycling weapons to re-roll the dice.
            // ---- AUDIT THE CONVERSION AFTER THE FACT ------------------------------------------
            //
            // The gate now only decides the FIRST conversion, so a wrongly-detected "rest" would be
            // baked in permanently. The rest estimate is an EMA, and an EMA seeded during an
            // animation is wrong until it has had time to converge -- which is exactly the window a
            // scope raised mid-draw lands in.
            //
            // So remember where "rest" was believed to be when we converted, and check back once the
            // estimate has had time to settle. If rest has since moved a long way, the conversion was
            // taken against a bad belief and is redone.
            //
            // THE REDO GOES THROUGH THE ANCHOR, NEVER THROUGH A DETACH. Clearing s_pane_anchored
            // forces a fresh rig-frame placement from the AUTHORED calibration and then a new
            // conversion, so the input is the calibration rather than the previous output -- which is
            // what makes it idempotent. A bare detach re-converts against the pane's own drifted
            // transform, and that is the fuel-rod accumulation.
            //
            // Bounded to a couple of corrections so a weapon whose bone genuinely never settles
            // cannot sit in a redo loop.
            static Vec3     s_rest_at_convert{};
            static bool     s_have_convert_rest = false;
            static uint32_t s_convert_tick = 0;
            static int      s_audits_left = 2;
            if (g_cfg.scope_socket_audit != 0 &&
                s_have_convert_rest && s_audits_left > 0 && !placing && bone_settled &&
                s_attached_socket != nullptr && (tick - s_convert_tick) >= 96) {
                const float mx = g_socket_rest_est.x - s_rest_at_convert.x;
                const float my = g_socket_rest_est.y - s_rest_at_convert.y;
                const float mz = g_socket_rest_est.z - s_rest_at_convert.z;
                const float moved_rest = std::sqrt(mx * mx + my * my + mz * mz);
                s_have_convert_rest = false;          // one audit per conversion
                if (moved_rest > kSocketRestCm) {
                    --s_audits_left;
                    s_pane_anchored = false;          // re-place from the calibration, then reconvert
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] scope: the weapon's rest pose has moved %.1f cm since the "
                        "space-switch conversion was taken, so that conversion was made against a "
                        "bad estimate and is being REDONE from the calibration. %d audit(s) left.",
                        moved_rest, s_audits_left);
                }
            }

            static bool s_convert_dirty = false;
            if (s_convert_dirty && !placing && bone_settled && !bone_late &&
                s_attached_socket != nullptr) {
                s_convert_dirty = false;
                s_pane_anchored = false;   // forces one rig-frame placement, then a clean convert
                API::get()->log_info(
                    "[Halo-CampE-UEVR] scope: the PrimaryWeapon bone has gone still, so the "
                    "space-switch conversion that was forced on a moving bone is being REDONE. "
                    "The pane should settle into its calibrated position now.");
            }
            if (bone_late) {
                s_convert_dirty = true;
                API::get()->log_info(
                    "[Halo-CampE-UEVR] scope: the PrimaryWeapon bone never went still within %u "
                    "ticks, so the space-switch conversion is being taken anyway. Expect the pane's "
                    "socket-frame placement to carry however far the bone was from rest -- if it "
                    "looks misplaced, close and re-open the scope while standing still.",
                    kSocketWaitTicks);
            }
            // ONCE SOCKETED, THE GATE IS SPENT. It decides WHEN TO FIRST CONVERT and must never
            // re-litigate a conversion already made.
            //
            // REGRESSION FIXED HERE, and it was mine. want_socket is recomputed every tick, so a
            // gate that says "not yet" while the pane is ALREADY on the socket does not merely wait
            // -- it asks for a DIFFERENT attachment, and the attach block below duly detaches back
            // to the rig. Firing moves the bone fast enough to trip that every shot. A socket-only
            // change does not set parent_changed, so s_pane_anchored is left alone, no fresh
            // rig-frame placement happens, and the pane carries its drifted world transform into the
            // next KeepWorld conversion -- which bakes the drift in. Fire again and it compounds.
            //
            // Reported exactly that way on the fuel rod cannon: "every time I fire, the quad rotates
            // lower and stays there. I can accumulate changes with continued fire." Accumulation is
            // the signature of a conversion being RE-TAKEN against its own previous output, and it
            // is strictly worse than the mis-timed first conversion the gate exists to prevent.
            //
            // `placing` still drops to the rig, which is correct: that is the handshake deliberately
            // re-placing in the authored frame.
            const bool already_socketed = (s_attached_socket != nullptr);
            if (!placing && (already_socketed || bone_ok)) want_socket = L"PrimaryWeapon";
            if (!placing && !already_socketed && bone_ok) {
                s_rest_at_convert   = g_socket_rest_est;   // what we believed rest was, for the audit
                s_have_convert_rest = true;
                s_convert_tick      = tick;
            }

            // WHY THE SOCKET IS BEING WITHHELD, said out loud. Without this the failure is a NON-EVENT:
            // the pane sits in exactly the right place (the rig-frame placement is correct on its own)
            // and simply never rides the weapon, with nothing in the log to distinguish "the handshake
            // is waiting" from "the handshake is stuck". That is what happened on 2026-09-06 -- a scope
            // session logged one `attached to rig` and then nothing at all, and no amount of reading the
            // source narrowed WHICH of the three terms was holding it.
            //
            // On CHANGE only, so the steady state costs three bool tests and no log traffic.
            {
                static int s_said = -1;
                // The BONE term is folded into the state key, not just into the message. A key that
                // encodes only the three original reasons cannot change when the fourth one does, so
                // the new branch below would be unreachable in exactly the case it was written for --
                // the same "enumerates the old set" mistake that the weapon-watch regression was.
                const int now_state = placing ? (1 + (s_pane_anchored ? 0 : 1) * 1
                                                   + (pane_placement_changed() ? 2 : 0)
                                                   + (s_calib_held ? 4 : 0))
                                              : (bone_ok ? 0 : 8);
                if (now_state != s_said) {
                    s_said = now_state;
                    if (placing) {
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] scope: holding the pane on the RIG for the space-switch "
                            "handshake -- anchored=%d placement_changed=%d calibrating=%d. The socket "
                            "attach waits until all three are settled; if this line does not clear, the "
                            "pane will be correctly placed but will NOT follow the weapon.",
                            (int)s_pane_anchored, (int)pane_placement_changed(), (int)s_calib_held);
                    } else if (!bone_ok) {
                        // Distinct from the line above on purpose: "placing" and "the bone is still
                        // moving" are different waits with different cures, and one message covering
                        // both would send the reader to the wrong one.
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] scope: placement is settled but the PrimaryWeapon "
                            "bone is still moving, so the KeepWorld conversion is being held off. "
                            "This is deliberate -- converting mid-draw bakes the bone's displacement "
                            "into the pane's offset permanently (measured: a 20 cm error from the "
                            "same calibration).");
                    } else {
                        API::get()->log_info("[Halo-CampE-UEVR] scope: handshake settled -- attaching at "
                                             "socket PrimaryWeapon on this pass.");
                    }
                }
            }
        } else if (g_cfg.scope_parent == 2) {
            // The weapon actor's own root. Kept as a dial rather than deleted so the two can be A/B'd
            // in-headset, but it is NOT the default: it depends on a pooled actor's lifetime and it
            // follows only animation authored on the weapon itself.
            if (auto* wroot = fp_weapon_root()) want_parent = wroot;
            // else: no weapon this frame -> stay on the rig, and re-attach when one comes back.
        }

        if (ready && (s_attached_rig != want_parent || s_attached_socket != want_socket)) {
            // RE-ATTACH ONLY. The "a new weapon closes the scope" rule deliberately does NOT live here,
            // for two independent reasons: this function runs only while the scope is ACTIVE, so
            // closing from here would fire on the first tick after the player opened the scope on a new
            // weapon and cancel the very press that opened it -- and more fundamentally the rig does
            // not change on a weapon swap at all, so this condition never fires for that reason
            // anyway. The working detector is at the bottom of scope_apply.
            bool a1 = attach_to(s_capture.ptr, want_parent, want_socket);
            bool a2 = attach_to(s_pane.ptr, want_parent, want_socket);

            // FALL BACK TO THE RIG IF THE WEAPON WOULD NOT TAKE IT.
            //
            // A refused attach leaves the component parented to nothing, which does not look like a
            // failure -- it looks like a pane hanging in the world that you can walk away from. The rig
            // is the known-good parent, so a weapon that will not accept the attach costs recoil
            // tracking rather than the whole scope.
            if ((want_parent != rig || want_socket != nullptr) && !(a1 && a2)) {
                API::get()->log_info("[Halo-CampE-UEVR] scope: weapon-following attach REFUSED "
                                     "(capture=%d pane=%d) -- falling back to the rig origin. The pane "
                                     "will follow your hands but not the weapon's recoil.",
                                     (int)a1, (int)a2);
                want_parent = rig;
                want_socket = nullptr;
                a1 = attach_to(s_capture.ptr, want_parent);
                a2 = attach_to(s_pane.ptr, want_parent);
            }
            const bool ok = (a1 && a2);
            const bool parent_changed = (s_attached_rig != want_parent);
            s_attached_rig    = ok ? want_parent : nullptr;
            s_attached_socket = ok ? want_socket : nullptr;

            // RE-PLACE ONLY IF THE PARENT COMPONENT ITSELF CHANGED -- never for a socket-only move.
            //
            // The socket handshake above deliberately re-attaches the SAME component at a socket, using
            // KeepWorld precisely so the placement survives. Clearing the anchor there would order a
            // fresh rig-frame write on the next tick, which drops the socket again, which re-anchors,
            // which re-attaches... a permanent oscillation between the two frames, at tick rate, on a
            // pane the player is looking through.
            if (parent_changed) s_pane_anchored = false;

            // ---- THE CONVERTED NUMBERS, PRINTED IN A FORM THAT CAN BE CANONIZED ------------------
            //
            // The handshake above is a migration step, not a permanent feature: once these socket-frame
            // values become the compiled defaults, the placement writes them directly and the two-phase
            // dance can go. Nothing else in the tree can tell us what they are, though -- the engine
            // computed them inside K2_AttachToComponent -- so read them straight back off the component.
            //
            // Logged on EVERY conversion rather than once, deliberately. The socket is a BONE, so its
            // transform moves with idle sway and breathing as well as recoil, and a single sample would
            // bake whatever pose the arms happened to be in into the shipped default. Several samples
            // that agree are the evidence that the value is a rest-pose constant; several that disagree
            // say to capture it somewhere quieter. Do not canonize one reading.
            if (ok && want_socket != nullptr) {
                auto* rl = s_pane.ptr->get_property_data<double>(L"RelativeLocation");
                auto* rr = s_pane.ptr->get_property_data<double>(L"RelativeRotation");
                if (rl != nullptr && rr != nullptr) {
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] scope SPACE SWITCH -> socket frame: "
                        "scopedist=%.3f scoperight=%.3f scopeup=%.3f "
                        "scoperotp=%.3f scoperoty=%.3f scoperotr=%.3f "
                        "(was rig frame %.3f/%.3f/%.3f %.3f/%.3f/%.3f). Sample this a few times before "
                        "canonizing -- the socket is a bone and moves with idle sway.",
                        rl[0], rl[1], rl[2], rr[0], rr[1], rr[2],
                        g_cfg.scope_dist, g_cfg.scope_right, g_cfg.scope_up,
                        g_cfg.scope_rot_p, g_cfg.scope_rot_y, g_cfg.scope_rot_r);
                }
            }
            API::get()->log_info("[Halo-CampE-UEVR] scope: attached to %s %p (capture=%d pane=%d) "
                                 "-- transform now composed at render rate",
                                 (want_parent != rig) ? "WEAPON ACTOR"
                                                     : (want_socket ? "rig socket PrimaryWeapon" : "rig"),
                                 (void*)want_parent, (int)a1, (int)a2);
        }
}

bool ensure_components(API::UObject* rig) {
    if (s_failed) {
        s_ec_bail = "s_failed LATCH is set (a creation failed earlier; it never retries)";
        return false;
    }
    auto* owner = (rig != nullptr) ? rig->get_outer() : nullptr;
    if (owner == nullptr) {
        s_ec_bail = (rig == nullptr) ? "rig is null" : "rig->get_outer() is null";
        return false;
    }
    s_ec_bail = nullptr;

    if (s_rt.empty()) {
        auto* rt = make_scope_rt(g_cfg.scope_rt_size);
        if (rt == nullptr) {
            s_failed = true;
            API::get()->log_info("[Halo-CampE-UEVR] scope: render target creation FAILED");
            return false;
        }
        s_rt.set(rt);
        s_rt_size_applied = g_cfg.scope_rt_size;
        // Record the format this target was actually allocated with, from the SAME expression
        // make_scope_rt() used, so the two can never disagree about what is on the GPU.
        s_rt_format_applied = rt_format_for_source(g_cfg.scope_capture_src);
        // A new RT invalidates both bindings.
        s_fov_applied = 0.0f;
        if (auto* cap = s_capture.get_checked(L"SceneCaptureComponent2D")) {
            if (auto* p = cap->get_property_data<API::UObject*>(L"TextureTarget")) *p = rt;
        }
        if (auto* mid = s_pane_mid.get_checked(L"MaterialInstanceDynamic")) {
            // BOTH parameter names, exactly as the creation path binds them. This rebind used to
            // set only "SlateUI" -- but the lens is EmissiveMeshMaterial, which samples
            // "LinearColor", so every RT rebuild (a scoperes change, or a capture-source change
            // that crosses the 8-bit/float boundary) left the pane pointed at the DESTROYED
            // target and rendering black. The creation path and the rebind path must bind the
            // same set or a rebuild silently breaks the pane.
            static const wchar_t* kTexParams[] = { L"LinearColor", L"SlateUI" };
            for (const wchar_t* name : kTexParams) {
                alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
                API::FName param = make_fname(name);
                memcpy(q, &param, sizeof(int32_t) * 2);
                *reinterpret_cast<void**>(q + 8) = rt;
                mid->call_function(L"SetTextureParameterValue", q);
            }
        }
    }

    if (s_capture.empty()) {
        auto* cls = API::get()->find_uobject<API::UClass>(
            L"Class /Script/Engine.SceneCaptureComponent2D");
        auto* comp = (cls != nullptr) ? API::get()->add_component_by_class(owner, cls, false)
                                      : nullptr;
        if (comp == nullptr) {
            s_failed = true;
            API::get()->log_info("[Halo-CampE-UEVR] scope: SceneCaptureComponent2D creation FAILED "
                                 "(class=%p)", (void*)cls);
            return false;
        }
        // Cadence per scopecapmode (0 = manual divisor, the perf valve; 1 = every frame). The
        // bCaptureOnMovement default would otherwise re-render the scene on every camera move.
        comp->set_bool_property(L"bCaptureEveryFrame", g_cfg.scope_cap_mode == 1);
        comp->set_bool_property(L"bCaptureOnMovement", false);
        // Keeps exposure/TAA state alive between manual captures so a low-rate scope holds a
        // steady image instead of re-adapting on every capture. Config-driven rather than a
        // hardcoded true: that persistent state is also a temporal history, and dropping it is one
        // of the levers on the black final-colour capture (scopepersist).
        comp->set_bool_property(L"bAlwaysPersistRenderingState", g_cfg.scope_persist != 0);
        if (auto* p = comp->get_property_data<uint8_t>(L"CaptureSource"))
            *p = (uint8_t)g_cfg.scope_capture_src;
        // Post-processed sources (8 = FinalColorHDR, 9 = FinalToneCurveHDR) carry the effects
        // that live in post -- and the auto-exposure that fades this capture to black.
        if ((g_cfg.scope_capture_src == 8 || g_cfg.scope_capture_src == 9) &&
            g_cfg.scope_exposure > 0.0f) {
            pin_capture_exposure(comp, g_cfg.scope_exposure);
        }
        if (auto* p = comp->get_property_data<API::UObject*>(L"TextureTarget"))
            *p = s_rt.ptr;
        // NOT SetAbsolute. The reticule marks itself absolute so the owner cannot drag it, but
        // the scope wants the opposite: it is ATTACHED to the rig below so the engine composes
        // its transform every RENDER frame off a parent that already tracks the hand at render
        // rate. Absolute would defeat that and leave it chasing at ~32 Hz -- which is exactly
        // the "doesn't feel like a child of my controller" lag the first headset pass reported.
        s_capture.set(comp);
        s_fov_applied = 0.0f;
        // READ BACK what was written, and prove the capture function exists in this build --
        // call_function on a missing name does nothing and returns cleanly, so without this a
        // dead capture is indistinguishable from a working one (the codebase's readback rule).
        {
            API::UObject* tt = nullptr;
            int src_rb = -1, cef = -1;
            if (auto* p = comp->get_property_data<API::UObject*>(L"TextureTarget")) tt = *p;
            if (auto* p = comp->get_property_data<uint8_t>(L"CaptureSource")) src_rb = *p;
            if (auto* p = comp->get_property_data<uint8_t>(L"bCaptureEveryFrame")) cef = *p;
            auto* fn = API::get()->find_uobject<API::UObject>(
                L"Function /Script/Engine.SceneCaptureComponent2D.CaptureScene");
            API::get()->log_info("[Halo-CampE-UEVR] scope: capture readback target=%p (rt=%p) "
                                 "src=%d cefByte=%d CaptureScene fn=%s",
                                 (void*)tt, (void*)s_rt.ptr, src_rb, cef,
                                 fn != nullptr ? "found" : "MISSING");
        }
        API::get()->log_info("[Halo-CampE-UEVR] scope: capture component created @%p (src=%d)",
                             (void*)comp, g_cfg.scope_capture_src);
    }

    if (s_pane.empty()) {
        auto* smc_cls = API::get()->find_uobject<API::UClass>(
            L"Class /Script/Engine.StaticMeshComponent");
        auto* comp = (smc_cls != nullptr) ? API::get()->add_component_by_class(owner, smc_cls, false)
                                          : nullptr;
        if (comp == nullptr) {
            s_failed = true;
            API::get()->log_info("[Halo-CampE-UEVR] scope: pane component creation FAILED");
            return false;
        }
        // Round lens or square pane. The Cylinder squashed on its axis is uevrlib's ocular-lens
        // trick and gives a real circular edge with no alpha work; the Plane is the flat square.
        //
        // MUST GO THROUGH load_asset_by_path, not find_uobject. find_uobject only sees what is
        // ALREADY LOADED: the Plane happens to be (the reticule uses it), the Cylinder is not, so
        // a find-only lookup returned null and built a pane with NO MESH -- which draws nothing
        // and is indistinguishable from the feature being broken. uevrlib force-loads this exact
        // asset for the same reason. Falls back to the Plane rather than shipping an empty pane.
        // find_uobject FIRST (it wants the "StaticMesh <path>" form and is what has always found
        // the Plane), then load_asset_by_path as the fallback for an asset that is not resident
        // yet -- and note that one takes a BARE object path: passing the class prefix to it makes
        // it reject the string at its first character, which is how a "fix" here managed to lose
        // the Plane that had been working. Same shape as find_or_load_material just below it.
        auto find_or_load_mesh = [](const wchar_t* prefixed, const char* bare) -> API::UObject* {
            if (auto* m = API::get()->find_uobject<API::UObject>(prefixed)) return m;
            return load_asset_by_path(bare);
        };
        const char* want = (g_cfg.scope_shape == 1) ? "/Engine/BasicShapes/Cylinder.Cylinder"
                                                    : "/Engine/BasicShapes/Plane.Plane";
        auto* mesh = find_or_load_mesh(
            g_cfg.scope_shape == 1 ? L"StaticMesh /Engine/BasicShapes/Cylinder.Cylinder"
                                   : L"StaticMesh /Engine/BasicShapes/Plane.Plane", want);
        if (mesh == nullptr && g_cfg.scope_shape == 1) {
            API::get()->log_info("[Halo-CampE-UEVR] scope: %s unavailable -- falling back to the "
                                 "square Plane", want);
            want = "/Engine/BasicShapes/Plane.Plane";
            mesh = find_or_load_mesh(L"StaticMesh /Engine/BasicShapes/Plane.Plane", want);
        }
        if (mesh != nullptr) {
            alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
            *reinterpret_cast<void**>(q) = mesh;
            comp->call_function(L"SetStaticMesh", q);
        } else {
            // A pane with no mesh renders NOTHING while every other log line reports success --
            // the exact failure that shipped as "I don't see the pane at all". Treat it as a hard
            // failure instead of a warning, so it latches, says so once, and cannot masquerade.
            s_failed = true;
            API::get()->log_info("[Halo-CampE-UEVR] scope: pane mesh %s COULD NOT BE LOADED -- "
                                 "the scope cannot draw. Feature disabled until the next rebuild "
                                 "or shape change.", want);
            return false;
        }

        // EmissiveMeshMaterial, the uevrlib scope lens: its "LinearColor" texture parameter takes
        // the HDR target and its emissive output re-enters the main view's tonemapper, which is
        // what makes the pane match the player's exposure. BlendMode is forced opaque on the
        // asset first (uevrlib does the same) so the pane cannot render additive/translucent.
        auto* base = API::get()->find_uobject<API::UObject>(
            L"Material /Engine/EngineMaterials/EmissiveMeshMaterial.EmissiveMeshMaterial");
        API::UObject* mid = nullptr;
        if (base != nullptr) {
            // BLEND MODE, restored after being removed on a wrong theory. I argued this was a
            // compile-time property whose runtime write could not matter; the headset says
            // otherwise -- without it the pane renders as a washed-out ghost you can see the
            // world through, with it the image is solid. MEASURED, both shapes, same session.
            // It is a write to the SHARED engine material (uevrlib does the same): acceptable
            // only because EmissiveMeshMaterial is an engine debug/visualisation material this
            // game does not use, and worth revisiting if anything else ever renders oddly.
            // TwoSided stays untouched -- that one really was unnecessary.
            if (auto* bm = base->get_property_data<uint8_t>(L"BlendMode")) *bm = 0;  // BLEND_Opaque
            alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
            *reinterpret_cast<int32_t*>(q) = 0;      // ElementIndex
            *reinterpret_cast<void**>(q + 8) = base; // SourceMaterial
            comp->call_function(L"CreateDynamicMaterialInstance", q);
            mid = *reinterpret_cast<API::UObject**>(q + 24);
        }
        if (mid != nullptr) {
            static const wchar_t* kTexParams[] = { L"LinearColor", L"SlateUI" };
            for (const wchar_t* name : kTexParams) {
                alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
                API::FName param = make_fname(name);
                memcpy(q, &param, sizeof(int32_t) * 2);
                *reinterpret_cast<void**>(q + 8) = s_rt.ptr;
                mid->call_function(L"SetTextureParameterValue", q);
            }
            set_tint(mid, g_cfg.scope_bright);
            s_pane_mid.set(mid);
        }
        API::get()->log_info("[Halo-CampE-UEVR] scope: pane created @%p (mesh=%s mid=%p)",
                             (void*)comp, mesh != nullptr ? "ok" : "MISSING", (void*)mid);

        { alignas(16) uint8_t q[RIG_PARAM_BUF] = {0}; comp->call_function(L"SetCollisionEnabled", q); }
        // No SetAbsolute -- see the capture component above; the pane is attached to the rig.
        // NEVER let the pane appear in a scene capture: it hangs between the capture camera and
        // the world, so without this it photographs its own back and the scope shows black. The
        // camera also sits beyond it (scope_cam_dist), but that is a comfort margin -- this flag
        // is the structural guarantee, and it survives any mis-tuning of the distances.
        comp->set_bool_property(L"bHiddenInSceneCapture", true);
        { alignas(16) uint8_t q[RIG_PARAM_BUF] = {0}; comp->call_function(L"SetCastShadow", q); }
        { alignas(16) uint8_t q[RIG_PARAM_BUF] = {0}; comp->call_function(L"SetHiddenInGame", q); }
        set_visibility(comp, false);   // hidden until the toggle says otherwise
        s_pane_shown = false;
        s_pane.set(comp);
    }

    const bool ready = !s_pane.empty() && !s_capture.empty() && !s_rt.empty();

    // (Re)attach whenever the rig changes or a component was just rebuilt. Cheap and idempotent:
    // one call each, only on change.
    // ---- WHAT THE PANE HANGS OFF.
    //
    // The weapon is a SEPARATE ACTOR socketed onto the arms mesh, so attaching to the rig means the
    // pane follows the arms but NOT the weapon's own animation -- and sniper recoil then drives the
    // gun straight through the pane, which is the reported artefact.
    //
    // scopeparent=1 hangs it off the weapon's root component instead, so recoil, sway and any other
    // weapon-local animation carry the pane with them and the two can no longer intersect.
    //
    // LIFECYCLE, and this is the whole risk of the change:
    //   * fp_weapon_root() RE-RESOLVES every call (it walks from the live weapon actor). No handle
    //     is held across frames, which is the rule this codebase already states for pooled actors --
    //     they are recycled, and a stored pointer silently becomes someone else's object.
    //   * A weapon swap returns a DIFFERENT root, so the change-detect below re-attaches. Death, a
    //     holster, a vehicle or any state with no first-person weapon returns null, and we FALL BACK
    //     TO THE RIG rather than leaving the pane parented to something that is going away.
    //   * The pane is our component on the PAWN; the weapon is only its attach parent. Losing the
    //     weapon orphans the attachment, it does not destroy our component -- and the fallback
    //     re-parents it on the very next tick.
    // The capture moves with the pane deliberately: they must stay rigid relative to each other or
    // the rendered image slides across the pane as the weapon animates.
    update_pane_attachment(rig, ready, 0);
    return ready;
}

#if HALO_VR_DEV
// ---------------------------------------------------------------------------------------------
// TEST OBJECT -- the instrument that answers "does a TRANSLUCENT primitive reach the capture?"
//
// Everything the scope is missing (enemy shields, cover shields, tracers, the ocean) was assumed
// to share one cause, but nothing in the pane was ever OURS, so every explanation stayed a guess.
// This puts a cube whose blend mode we choose directly in front of the capture camera -- parented
// TO THE CAPTURE, at its local +X, which is exactly where a scene capture looks -- so it is in
// frame by construction and needs no aiming. It is also in the MAIN view (same place, in front of
// the weapon), which is the built-in control: "visible in the world, absent from the pane" is the
// whole measurement, in one screenshot.
//
// The A/B is scopetest 1-vs-2 (a natively translucent engine material vs the opaque member of the
// same family) and 3-vs-4 (one shared material forced translucent vs opaque). Two independent
// pairs on purpose: a single pair could be defeated by something specific to one material.
//
// NOT shipped -- dev-compiled only. It draws geometry into the player's view, which is exactly
// the class of thing the compile-time gate exists to keep out of a headset.
TrackedObject s_testobj;
TrackedObject s_testobj_mid;
int  s_testobj_built = -1;
// PLACEMENT MEMO -- file scope so the rebuild path can clear it, which is the whole point.
// These were function-level statics, and they SURVIVED the destroy-and-remake that a mode change
// performs. The new cube then failed the "has anything changed?" test and was never placed at all:
// it sat at the capture's own origin at the engine Cube's full 100 cm. That is what "the cube is
// displaced" looked like in the headset, and it silently made the 3-vs-4 A/B unrunnable -- the
// control half of the experiment, so the half whose absence is easiest to miss.
float s_testobj_dist = -1.0f, s_testobj_size = -1.0f;

// IS THIS POINTER A LIVE UObject?
//
// This probe has now been defeated THREE times by the same failure: a texture path that does not
// resolve on this build, a bind that silently does nothing, and a black cube that then reads as
// "the blend mode under test is invisible" -- the exact answer the probe exists to measure,
// arriving for entirely the wrong reason. find_uobject and load_asset_by_path both hand back
// something non-null on failure here, and the giveaway was only visible by eye: the returned
// pointer sat in the MODULE address range and moved with the module base between sessions, which
// is not where UObjects live.
//
// So the pointer is checked against the object array before anything is built on it. O(n) over
// ~296k objects, but only at cube creation, and only in a dev build.
bool is_live_uobject(API::UObject* p) {
    if (p == nullptr) return false;
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return false;
    const int32_t n = arr->get_object_count();
    for (int32_t i = 0; i < n; ++i) {
        if (arr->get_object(i) == reinterpret_cast<void*>(p)) return true;
    }
    return false;
}

// A texture that actually EXISTS on this build. Tries each candidate through both lookups and
// validates the result, rather than trusting the first non-null answer.
API::UObject* resolve_probe_texture() {
    // MODE 1 (default): a solid white render target we build and clear ourselves. The pane proves
    // this exact object TYPE samples correctly through this exact material and parameter name, so
    // a black cube can no longer be blamed on the texture -- which is what the old default,
    // WhiteSquareTexture, quietly did for three rounds despite resolving to a live UObject.
    if (g_cfg.scope_test_tex == 1) {
        static TrackedObject s_white;
        if (s_white.get() == nullptr) {
            if (auto* rt = make_color_rt(1.0f, 1.0f, 1.0f, 1.0f, 64)) s_white.set(rt);
        }
        if (auto* rt = s_white.get()) {
            API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST: probe texture = solid white RT @%p "
                                 "(built + cleared at runtime)", (void*)rt);
            return rt;
        }
        API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST: make_color_rt FAILED -- falling through "
                             "to the engine texture candidates");
    }
    // MODE 2: the scope's own target. Certain to sample, but it feeds the capture back into itself
    // -- judge the MAIN VIEW only.
    if (g_cfg.scope_test_tex == 2) {
        if (auto* rt = s_rt.get()) {
            API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST: probe texture = the scope's OWN RT "
                                 "@%p -- FEEDBACK LOOP, read the main view, not the pane",
                                 (void*)rt);
            return rt;
        }
    }
    struct Cand { const wchar_t* prefixed; const char* bare; };
    static const Cand kCands[] = {
        { L"Texture2D /Engine/EngineResources/WhiteSquareTexture.WhiteSquareTexture",
          "/Engine/EngineResources/WhiteSquareTexture.WhiteSquareTexture" },
        { L"Texture2D /Engine/EngineResources/DefaultTexture.DefaultTexture",
          "/Engine/EngineResources/DefaultTexture.DefaultTexture" },
        { L"Texture2D /Engine/EngineResources/Black.Black",
          "/Engine/EngineResources/Black.Black" },
    };
    for (const auto& c : kCands) {
        auto* t = API::get()->find_uobject<API::UObject>(c.prefixed);
        if (!is_live_uobject(t)) t = load_asset_by_path(c.bare);
        if (is_live_uobject(t)) {
            API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST: probe texture RESOLVED %s @%p",
                                 c.bare, (void*)t);
            return t;
        }
        API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST: probe texture %s did not resolve to a "
                             "live UObject (got %p)", c.bare, (void*)t);
    }
    return nullptr;
}

void dev_test_object(API::UObject* cap) {
    // Rebuild whenever the mode changes: the material family is chosen at MID creation (the
    // parent's BlendMode is read then, which is the whole point of modes 3/4), so a live switch
    // has to destroy and remake rather than re-parameterise.
    s_testobj.get_checked(L"StaticMeshComponent");
    if (s_testobj_built != g_cfg.scope_test_obj || (s_testobj.empty() && s_testobj_built > 0)) {
        if (auto* old = s_testobj.get_checked(L"StaticMeshComponent")) {
            alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
            *reinterpret_cast<void**>(q) = old;
            old->call_function(L"K2_DestroyComponent", q);
        }
        s_testobj.reset();
        s_testobj_mid.reset();
        s_testobj_built = g_cfg.scope_test_obj;
        if (g_cfg.scope_test_obj == 0) {
            API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST: off (object destroyed)");
            return;
        }
    }
    if (g_cfg.scope_test_obj == 0 || cap == nullptr) return;

    if (s_testobj.empty()) {
        auto* owner = cap->get_outer();
        auto* smc_cls = API::get()->find_uobject<API::UClass>(
            L"Class /Script/Engine.StaticMeshComponent");
        auto* comp = (owner != nullptr && smc_cls != nullptr)
                         ? API::get()->add_component_by_class(owner, smc_cls, false) : nullptr;
        if (comp == nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST: component creation FAILED");
            g_cfg.scope_test_obj = 0;
            return;
        }
        // A CUBE, not the Plane: a single-sided quad that happens to face away renders nothing,
        // and "nothing" is the exact answer this test is trying to measure. A cube cannot be
        // mistaken for the failure it is looking for.
        auto* mesh = API::get()->find_uobject<API::UObject>(L"StaticMesh /Engine/BasicShapes/Cube.Cube");
        if (mesh == nullptr) mesh = load_asset_by_path("/Engine/BasicShapes/Cube.Cube");
        if (mesh == nullptr) {
            mesh = API::get()->find_uobject<API::UObject>(L"StaticMesh /Engine/BasicShapes/Plane.Plane");
            if (mesh == nullptr) mesh = load_asset_by_path("/Engine/BasicShapes/Plane.Plane");
        }
        if (mesh != nullptr) {
            alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
            *reinterpret_cast<void**>(q) = mesh;
            comp->call_function(L"SetStaticMesh", q);
        } else {
            API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST: no mesh could be loaded -- aborting");
            g_cfg.scope_test_obj = 0;
            return;
        }

        // The material, and with it the blend mode under test.
        const bool widget_family = (g_cfg.scope_test_obj == 1 || g_cfg.scope_test_obj == 2);
        const bool plain_lit     = (g_cfg.scope_test_obj == 5);
        const bool real_shield   = (g_cfg.scope_test_obj == 6);
        API::UObject* base = nullptr;
        const char* what = "?";
        if (real_shield) {
            // MODE 6 -- THE ACTUAL MATERIAL IN QUESTION, on a cube we control.
            //
            // Every other mode tests a STAND-IN for the shield material and then argues by
            // analogy, which is how three runs went wrong: the stand-in needs parameters bound
            // before it can be seen, and the binding was the thing that failed. This mode skips
            // the analogy. MI_JackalShield is the exact material the census found on all 54 live
            // shield primitives, it is already loaded, and it arrives with its own textures and
            // parameters already set up by the game -- there is nothing of ours to get wrong.
            //
            // A cube wearing it is the shield, minus the shield's geometry, ownership, actor and
            // attachment. Visible in the pane => the material renders in captures perfectly well
            // and the shields' absence is about something else entirely (ownership, a per-actor
            // visibility flag, a render-pass gate we have not found). Invisible in the pane but
            // visible in the main view => the material itself is what captures will not draw, and
            // that is the answer.
            base = API::get()->find_uobject<API::UObject>(
                L"MaterialInstanceConstant /Game/FX/Library/Sandbox/Characters/Cov/Jackal/"
                L"MI_JackalShield.MI_JackalShield");
            if (!is_live_uobject(base))
                base = load_asset_by_path("/Game/FX/Library/Sandbox/Characters/Cov/Jackal/"
                                          "MI_JackalShield.MI_JackalShield");
            if (!is_live_uobject(base)) {
                // The parent, if the instance is not loaded right now (no Jackals in the level).
                base = API::get()->find_uobject<API::UObject>(
                    L"Material /Game/FX/Materials/Library/Characters/Cov/Jackal/"
                    L"M_JackalShield.M_JackalShield");
                if (!is_live_uobject(base))
                    base = load_asset_by_path("/Game/FX/Materials/Library/Characters/Cov/Jackal/"
                                              "M_JackalShield.M_JackalShield");
            }
            what = "MI_JackalShield -- THE REAL SHIELD MATERIAL, unlit/additive, game-authored";
        } else if (plain_lit) {
            // MODE 5 -- THE POSITIVE CONTROL, AND THE ONE THAT SHOULD HAVE EXISTED FIRST.
            //
            // Every other mode needs a parameter bound correctly before it can be seen at all, and
            // every single failure of this probe so far has been that binding rather than anything
            // about scene captures. WorldGridMaterial takes NO parameters: it is lit, opaque, and
            // visibly checkered out of the box, so it cannot come out black for a plumbing reason.
            //
            // If mode 5 shows in the pane, captures render our geometry AND shade it, and a null
            // result from any other mode means something real. If mode 5 is also invisible or
            // black, nothing measured through this probe has ever meant anything and that is the
            // bug to chase.
            base = API::get()->find_uobject<API::UObject>(
                L"Material /Engine/EngineMaterials/WorldGridMaterial.WorldGridMaterial");
            if (!is_live_uobject(base))
                base = load_asset_by_path("/Engine/EngineMaterials/WorldGridMaterial.WorldGridMaterial");
            if (!is_live_uobject(base)) {
                base = API::get()->find_uobject<API::UObject>(
                    L"Material /Engine/EngineMaterials/DefaultMaterial.DefaultMaterial");
                if (!is_live_uobject(base))
                    base = load_asset_by_path("/Engine/EngineMaterials/DefaultMaterial.DefaultMaterial");
            }
            what = "WorldGridMaterial -- LIT, OPAQUE, NO PARAMETERS (positive control)";
        } else if (widget_family) {
            const wchar_t* path = (g_cfg.scope_test_obj == 1)
                ? L"MaterialInstanceConstant /Engine/EngineMaterials/Widget3DPassThrough_Translucent."
                  L"Widget3DPassThrough_Translucent"
                : L"MaterialInstanceConstant /Engine/EngineMaterials/Widget3DPassThrough_Opaque."
                  L"Widget3DPassThrough_Opaque";
            base = API::get()->find_uobject<API::UObject>(path);
            if (base == nullptr) {
                base = load_asset_by_path(g_cfg.scope_test_obj == 1
                    ? "/Engine/EngineMaterials/Widget3DPassThrough_Translucent.Widget3DPassThrough_Translucent"
                    : "/Engine/EngineMaterials/Widget3DPassThrough_Opaque.Widget3DPassThrough_Opaque");
            }
            what = (g_cfg.scope_test_obj == 1) ? "Widget3DPassThrough_Translucent (authored BLEND_Translucent)"
                                               : "Widget3DPassThrough_Opaque (authored BLEND_Opaque)";
        } else {
            base = API::get()->find_uobject<API::UObject>(
                L"Material /Engine/EngineMaterials/EmissiveMeshMaterial.EmissiveMeshMaterial");
            // The pane's own recipe: the parent's BlendMode is written BEFORE the MID is built,
            // because a UMaterialInstance caches the base properties it inherits at construction.
            // 3 = BLEND_Additive (this material's AUTHORED mode, so the cooked shader matches),
            // 0 = BLEND_Opaque.
            if (base != nullptr) {
                if (auto* bm = base->get_property_data<uint8_t>(L"BlendMode"))
                    *bm = (g_cfg.scope_test_obj == 3) ? 3 : 0;
            }
            what = (g_cfg.scope_test_obj == 3) ? "EmissiveMeshMaterial at BLEND_Additive (authored)"
                                               : "EmissiveMeshMaterial forced BLEND_Opaque";
        }
        // VALIDATE THE MATERIAL ITSELF, not merely "non-null". Both lookups return junk rather
        // than null on this build -- that is how three runs of this probe measured a cube whose
        // material was never bound.
        if (base != nullptr && !is_live_uobject(base)) {
            API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST: base material @%p is NOT a live "
                                 "UObject -- the lookup returned junk. THIS RUN PROVES NOTHING.",
                                 (void*)base);
            base = nullptr;
        }
        API::UObject* mid = nullptr;
        if (base != nullptr) {
            alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
            *reinterpret_cast<int32_t*>(q) = 0;
            *reinterpret_cast<void**>(q + 8) = base;
            comp->call_function(L"CreateDynamicMaterialInstance", q);
            mid = *reinterpret_cast<API::UObject**>(q + 24);
            // Mode 5 is deliberately parameter-free: binding nothing is the entire point, so the
            // texture plumbing below is skipped for it.
            if (mid != nullptr && (plain_lit || real_shield)) {
                API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST: mode %d binds NO parameters of "
                                     "ours -- if this cube is black, the fault is not a bind",
                                     g_cfg.scope_test_obj);
                s_testobj_mid.set(mid);
            } else if (mid != nullptr) {
                // A WHITE SOURCE TEXTURE, or the cube renders black in every mode and an additive
                // black cube is invisible -- which is the answer this test is looking for, arriving
                // for the wrong reason. EmissiveMeshMaterial takes its emissive from "LinearColor"
                // and the Widget3DPassThrough family takes colour AND ALPHA from "SlateUI": with no
                // texture bound the widget MIDs come out fully transparent, which is exactly why
                // the first run of this probe showed nothing for modes 1 and 2 either.
                // MEASURED 2026-08-16: modes 3 and 4 came out black in the MAIN VIEW as well as in
                // the pane, so the instrument was broken and both A/B runs measured nothing.
                //
                // The cause is NOT the one first written here. That note blamed the texture for
                // "not resolving", on the grounds that the returned pointer sat in the module
                // address range rather than the UObject heap. That inference is WRONG: mode 5's
                // WorldGridMaterial resolves to 0x00007FF4-something too, and is_live_uobject
                // FINDS IT IN THE OBJECT ARRAY. Addresses in that range are ordinary objects on
                // this build, and the range says nothing about validity. Left here because it is
                // an easy and convincing mistake to make twice.
                //
                // So why EmissiveMeshMaterial cubes render black in both views is still OPEN --
                // most likely the emissive parameter name, not the texture object. Use mode 6 (the
                // real shield material, no parameters of ours) to sidestep the question entirely.
                // RESOLVED AND VALIDATED, not merely non-null -- see resolve_probe_texture. The
                // scope's own RT is NOT used as a fallback any more: it is a real object, but its
                // scene-colour alpha is 0 and pointing the cube at the very target the pane
                // displays is a feedback loop, so both earlier "fallbacks" guaranteed a black cube.
                API::UObject* tex = resolve_probe_texture();
                if (tex == nullptr) {
                    API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST: NO valid probe texture on "
                                         "this build -- %s has no emissive source and WILL be "
                                         "black in every view. THIS RUN PROVES NOTHING; use "
                                         "scopetest=5, which needs no texture.", what);
                }
                if (tex != nullptr) {
                    // BIND, THEN READ BACK. SetTextureParameterValue on a name the material does
                    // not have is a silent no-op, and that is indistinguishable from a bind that
                    // worked -- which is how a black cube got read as a result twice. The pane
                    // binds through this same pair of names and displays its render target, so
                    // "the parameter name must be wrong" was never a safe assumption either.
                    // K2_GetTextureParameterValue answers it directly instead of by inference.
                    static const wchar_t* kTexParams[] = { L"LinearColor", L"SlateUI" };
                    int bound = 0;
                    for (const wchar_t* name : kTexParams) {
                        {
                            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                            API::FName param = make_fname(name);
                            memcpy(p, &param, sizeof(int32_t) * 2);
                            *reinterpret_cast<void**>(p + 8) = tex;
                            mid->call_function(L"SetTextureParameterValue", p);
                        }
                        API::UObject* got = nullptr;
                        {
                            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                            API::FName param = make_fname(name);
                            memcpy(p, &param, sizeof(int32_t) * 2);
                            mid->call_function(L"K2_GetTextureParameterValue", p);
                            got = *reinterpret_cast<API::UObject**>(p + 8);
                        }
                        const bool ok = (got == tex);
                        if (ok) ++bound;
                        API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST: texparam '%s' -> %s "
                                             "(set %p, read back %p)", narrow(name).c_str(),
                                             ok ? "BOUND" : "not on this material",
                                             (void*)tex, (void*)got);
                    }
                    if (bound == 0) {
                        API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST: NO texture parameter "
                                             "took on this material -- it has no emissive source "
                                             "and WILL be black in every view. THIS RUN PROVES "
                                             "NOTHING about the capture.");
                    }
                }
                API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST: cube texture=%p (scope RT=%p)",
                                     (void*)tex, (void*)s_rt.ptr);
                {   // Widget3DPassThrough multiplies by this and defaults it to BLACK/zero alpha.
                    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                    API::FName param = make_fname(L"OpacityFromTexture");
                    memcpy(p, &param, sizeof(int32_t) * 2);
                    *reinterpret_cast<float*>(p + 8) = 1.0f;
                    mid->call_function(L"SetScalarParameterValue", p);
                }
                set_tint(mid, 4.0f);   // bright, so the cube cannot be lost in the scene
                // Same readback for the COLOUR parameter. EmissiveMeshMaterial's emissive is the
                // texture MULTIPLIED by this, so a colour that silently failed to bind leaves the
                // product at the material's default -- another way to get a black cube with every
                // individual step apparently succeeding.
                {
                    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                    API::FName param = make_fname(L"Color");
                    memcpy(p, &param, sizeof(int32_t) * 2);
                    mid->call_function(L"K2_GetVectorParameterValue", p);
                    const auto* c = reinterpret_cast<const float*>(p + 8);
                    API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST: vecparam 'Color' reads "
                                         "(%.2f,%.2f,%.2f,%.2f) after setting 4.0 -- %s",
                                         c[0], c[1], c[2], c[3],
                                         (c[0] > 0.01f) ? "BOUND"
                                                        : "NOT BOUND (emissive multiplies to zero)");
                }
                s_testobj_mid.set(mid);
            }
        }
        // Report the blend mode actually in force, so the log alone says what was tested. For a
        // MaterialInstanceConstant the byte lives on the instance; for a UMaterial, on the material.
        {
            int bm_read = -1;
            if (base != nullptr) {
                if (auto* bm = base->get_property_data<uint8_t>(L"BlendMode")) bm_read = (int)*bm;
            }
            API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST: base BlendMode byte reads %d "
                                 "(0=Opaque 1=Masked 2=Translucent 3=Additive 4=Modulate)", bm_read);
        }
        // RESTORE the shared material immediately. EmissiveMeshMaterial is also the scope pane's
        // parent, and leaving it translucent would silently turn the pane into a ghost -- which
        // would look exactly like a second bug and confuse the very measurement being taken.
        if (!widget_family) {
            if (auto* em = API::get()->find_uobject<API::UObject>(
                    L"Material /Engine/EngineMaterials/EmissiveMeshMaterial.EmissiveMeshMaterial")) {
                if (auto* bm = em->get_property_data<uint8_t>(L"BlendMode")) *bm = 0;
            }
        }

        { alignas(16) uint8_t q[RIG_PARAM_BUF] = {0}; comp->call_function(L"SetCollisionEnabled", q); }
        { alignas(16) uint8_t q[RIG_PARAM_BUF] = {0}; comp->call_function(L"SetCastShadow", q); }
        // The whole point: this one is NOT hidden from scene captures (the pane is).
        comp->set_bool_property(L"bHiddenInSceneCapture", false);
        attach_to(comp, cap);
        s_testobj.set(comp);
        s_testobj_dist = s_testobj_size = -1.0f;   // a fresh component has no placement yet
        API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST %d: cube @%p mesh=%s base=%p mid=%p -- %s",
                             g_cfg.scope_test_obj, (void*)comp, mesh != nullptr ? "ok" : "MISSING",
                             (void*)base, (void*)mid, what);
        if (base == nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST: material NOT FOUND -- the cube is on "
                                 "the DEFAULT material, so this run proves nothing about blending");
        }
    }

    // Placement, on change only (the codebase rule for engine calls).
    if (auto* comp = s_testobj.get_checked(L"StaticMeshComponent")) {
        if (s_testobj_dist != g_cfg.scope_test_dist || s_testobj_size != g_cfg.scope_test_size) {
            s_testobj_dist = g_cfg.scope_test_dist;
            s_testobj_size = g_cfg.scope_test_size;
            set_relative_location(comp, (double)g_cfg.scope_test_dist, 0.0, 0.0);
            set_relative_rotation(comp, 0.0, 0.0, 0.0);
            set_relative_scale(comp, g_cfg.scope_test_size, g_cfg.scope_test_size,
                               g_cfg.scope_test_size);
            set_visibility(comp, true);
            // DOES IT EVEN FIT IN THE FRAME? The engine Cube is 100 cm, and the capture's FOV is
            // scopebase/scopezoom -- 7 deg at the 10x the headset pass runs at. A 50 cm cube at
            // 300 cm subtends ~9.5 deg, so it OVERFLOWS the capture entirely and the pane shows
            // nothing but cube. "The pane is black" and "the cube is black" are then the same
            // picture, and the measurement cannot distinguish them. Say so rather than letting a
            // full-frame cube be read as a result.
            const float ang = 2.0f * (float)std::atan((100.0f * g_cfg.scope_test_size * 0.5f)
                                           / (g_cfg.scope_test_dist > 1.0f ? g_cfg.scope_test_dist
                                                                           : 1.0f)) / DEG2RAD;
            const float cap_fov = g_cfg.scope_base_fov / (g_cfg.scope_zoom > 0.01f ? g_cfg.scope_zoom
                                                                                  : 1.0f);
            API::get()->log_info("[Halo-CampE-UEVR] SCOPETEST: placed %.0f cm ahead of the capture, "
                                 "scale %.2f -- subtends %.1f deg in a %.1f deg capture%s",
                                 g_cfg.scope_test_dist, g_cfg.scope_test_size, ang, cap_fov,
                                 (ang >= cap_fov * 0.8f)
                                     ? "  <-- FILLS THE FRAME, the A/B cannot be read; shrink "
                                       "scopetestsize or raise scopetestdist"
                                     : "");
        }
    }
}
#endif

#if HALO_VR_DEV
// Blam zoom-state probe: logs the player's native zoom level/magnification ON CHANGE. This is the
// instrument that answers "does LT reach Blam zoom" the day a zoomable weapon is in hand -- and
// with scopeeat=0 it also verifies native zoom engaging underneath us. Full-array walk, so it is
// dev-compiled only and throttled to ~2 s, and it matches by class pointer, never by name string.
void dev_blam_zoom_probe(uint32_t tick) {
    if ((tick % 64) != 17) return;   // ~2 s, offset off the config-reload tick
    static API::UClass* unit_cls = nullptr;
    static bool tried = false;
    if (!tried) {
        tried = true;
        unit_cls = API::get()->find_uobject<API::UClass>(
            L"Class /Script/BlamSynchronization.BlamUnitComponent");
        API::get()->log_info("[Halo-CampE-UEVR] SCOPEDEV: BlamUnitComponent class %s",
                             unit_cls != nullptr ? "found" : "NOT FOUND");
    }
    if (unit_cls == nullptr) return;
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;

    static int32_t s_last_abs = -12345;
    static float   s_last_mag = -1.0f;
    static bool    listed = false;   // one-shot candidate census (set after the first full sweep)

    const int32_t n = arr->get_object_count();
    for (int32_t i = 0; i < n; ++i) {
        auto* o = reinterpret_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || !o->is_a(unit_cls)) continue;
        // Skip archetypes -- calling Blam-backed getters on a template with no live datum is
        // exactly the class of dereference that must fail closed.
        const std::wstring full = o->get_full_name();
        if (full.find(L"Default__") != std::wstring::npos ||
            full.find(L"GEN_VARIABLE") != std::wstring::npos) continue;

        int32_t datum = -1;
        {
            alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
            o->call_function(L"GetControllingPlayerDatumIndex", q);
            datum = *reinterpret_cast<int32_t*>(q);
        }
        // The census line, so a wrong filter is diagnosable from the log alone.
        if (!listed) {
            API::get()->log_info("[Halo-CampE-UEVR] SCOPEDEV candidate: datum=%d (0x%08X) %s",
                                 datum, (unsigned)datum, narrow(full).c_str());
        }
        // NONE is exactly -1. `datum < 0` is the wrong test: Blam datum handles carry salt in
        // the high bits, so a perfectly valid player handle can be negative as an int32.
        if (datum == -1) continue;

        int32_t abs = 0;
        float mag = 0.0f, frac = 0.0f;
        { alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
          o->call_function(L"GetZoomLevelAbsolute", q);
          abs = *reinterpret_cast<int32_t*>(q); }
        { alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
          o->call_function(L"GetZoomMagnification", q);
          mag = *reinterpret_cast<float*>(q); }
        { alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
          o->call_function(L"GetZoomLevelFraction", q);
          frac = *reinterpret_cast<float*>(q); }

        if (abs != s_last_abs || std::fabs(mag - s_last_mag) > 0.01f) {
            s_last_abs = abs;
            s_last_mag = mag;
            API::get()->log_info("[Halo-CampE-UEVR] SCOPEDEV blam zoom CHANGED: abs=%d mag=%.2f "
                                 "frac=%.2f (unit %s)", abs, mag, frac,
                                 narrow(full).c_str());
        }
        break;   // the first player-controlled unit is ours
    }
    listed = true;
}
#endif

// DO NOT TRUST s_pane_shown ALONE -- WE ARE NOT THE ONLY WRITER OF THE PANE'S VISIBILITY.
//
// This used to early-return whenever our own flag said the pane was already hidden. That is only
// sound if nothing else can show it, and something else can: the pane is ATTACHED to the weapon
// rig, and both USceneComponent::SetVisibility and ::SetHiddenInGame propagate to children
// (SceneComponent.h:851 / :882). The game showing a weapon as it is drawn therefore flips OUR pane
// visible underneath us. Our flag still reads "hidden", the early return fires, and the pane sits
// there displaying the last render target it captured -- VISIBLE AND FROZEN, which is exactly the
// weapon-switch report.
//
// It also explains why closing the scope on a rig change did nothing: the scope was already
// closed. g_scope_active was never the variable at fault; the component's own bVisible was.
//
// So: read the component's ACTUAL bVisible and hide whenever it disagrees with what we want. The
// offset and bitfield mask are resolved once per class and cached, because this runs every tick
// the scope is closed -- find_property walks the class chain and get_checked builds a string, and
// neither belongs in a per-tick path in VR.
void set_hidden_in_game(API::UObject* comp, bool hidden) {
    if (comp == nullptr) return;
    alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
    q[0] = hidden ? 1 : 0;   // q[1] = bPropagateToChildren, deliberately false: the pane has none
    comp->call_function(L"SetHiddenInGame", q);
}

// Read the pane's two visibility gates as the ENGINE sees them, not as we believe them to be.
// Offsets and bitfield masks are resolved once per class and cached: this runs every tick the
// scope is closed, and neither find_property (walks the class chain) nor get_checked (builds a
// wstring) belongs in a per-tick path in VR. -1 in either field means "could not resolve".
struct PaneVis { int visible = -1; int hidden_in_game = -1; };

PaneVis read_pane_vis(API::UObject* pane) {
    static uintptr_t s_cls   = 0;
    static int32_t   s_v_off = -1, s_h_off = -1;
    static uint8_t   s_v_msk = 0,  s_h_msk = 0;

    PaneVis out;
    if (pane == nullptr) return out;
    auto* cls = pane->get_class();
    if (cls != nullptr && reinterpret_cast<uintptr_t>(cls) != s_cls) {
        s_cls = reinterpret_cast<uintptr_t>(cls);
        s_v_off = s_h_off = -1;
        if (auto* p = cls->find_property(L"bVisible")) {
            auto* bp = static_cast<API::FBoolProperty*>(p);
            s_v_off = (int32_t)bp->get_offset();  s_v_msk = bp->get_field_mask();
        }
        if (auto* p = cls->find_property(L"bHiddenInGame")) {
            auto* bp = static_cast<API::FBoolProperty*>(p);
            s_h_off = (int32_t)bp->get_offset();  s_h_msk = bp->get_field_mask();
        }
        if (s_v_off < 0) {
            API::get()->log_info("[Halo-CampE-UEVR] scope: bVisible NOT FOUND on the pane -- "
                                 "falling back to the shown flag; an externally shown pane will "
                                 "not be caught");
        }
    }
    const auto* base = reinterpret_cast<const uint8_t*>(pane);
    if (s_v_off >= 0) {
        const uint8_t b = *(base + s_v_off);
        out.visible = (s_v_msk == 0) ? (b != 0) : ((b & s_v_msk) != 0);
    }
    if (s_h_off >= 0) {
        const uint8_t b = *(base + s_h_off);
        out.hidden_in_game = (s_h_msk == 0) ? (b != 0) : ((b & s_h_msk) != 0);
    }
    return out;
}

// DO NOT TRUST s_pane_shown ALONE -- WE ARE NOT THE ONLY WRITER OF THE PANE'S VISIBILITY.
// Confirmed live 2026-08-15: "pane was visible WITHOUT us showing it" fired on a weapon switch.
// The pane is attached to the arms rig, and SetVisibility/SetHiddenInGame both propagate to
// children (SceneComponent.h:851/:882), so the game re-showing the arms at the end of a weapon
// swap flips OUR pane visible underneath us.
//
// TWO GATES, because correcting it afterwards is a visible FLASH. Reading bVisible and fixing it
// on the next tick is reactive -- at ~32 Hz that is up to ~31 ms of pane on screen, which the
// headset pass reported as distracting. So while the scope is closed the pane is BOTH
// bVisible=false AND bHiddenInGame=true. A propagated SetVisibility(true) then still leaves it
// hidden, and nothing flashes. The read below stays as the backstop for the case where the game
// propagates hidden-in-game as well -- and will say so in the log if it does.
void hide_pane_if_shown() {
    auto* pane = s_pane.get();   // O(1) slot check; no string, unlike get_checked
    if (pane == nullptr) { s_pane_shown = false; return; }

    const PaneVis vis = read_pane_vis(pane);
    const bool externally_shown = (vis.visible == 1) || (vis.hidden_in_game == 0);
    if (!s_pane_shown && !externally_shown) return;          // genuinely hidden: nothing to do
    if (!s_pane_shown && vis.visible < 0 && vis.hidden_in_game < 0) return;   // unresolvable

    if (!s_pane_shown) {
        // Rate-limited: if something re-shows the pane every frame we must not log every frame.
        // WHICH gate was breached is the diagnostic -- hidden_in_game==0 means the game propagates
        // hidden-in-game too, and the pre-emptive gate above cannot hold on its own.
        static uint32_t s_extern_hits = 0;
        if ((s_extern_hits++ % 64) == 0) {
            API::get()->log_info("[Halo-CampE-UEVR] scope: pane shown WITHOUT us (#%u) -- "
                                 "bVisible=%d bHiddenInGame=%d; forcing it hidden",
                                 s_extern_hits, vis.visible, vis.hidden_in_game);
        }
    }
    // Re-validate the class before the write; get() alone would let a recycled slot through.
    if (auto* checked = s_pane.get_checked(L"StaticMeshComponent")) {
        set_visibility(checked, false);
        set_hidden_in_game(checked, true);
    }
    s_pane_shown = false;
}

// SEED THE WEAPON WATCH AT OPEN TIME. Hoisted out of the pane's show block because the compositor
// quad is a second way for the scope to start DISPLAYING, and the watch has to start from the
// weapon actually in hand either way.
//
// It used to seed itself lazily inside the detector, which only runs while the scope is displaying
// -- so a scope closed by the trigger or by going stale left the LAST weapon pointer behind.
// Switching weapons while closed and re-opening then compared new-against-old on the very first
// tick and closed the scope the player had just opened: "it turns on, then turns itself off really
// quickly". Seed on the open edge, from one place, whichever surface is doing the displaying.
void seed_weapon_watch(uint32_t tick) {
    s_seen_weapon     = fp_weapon_actor();
    s_seen_weapon_cls = (s_seen_weapon != nullptr) ? s_seen_weapon->get_class() : nullptr;
    s_scope_open_tick = tick;
}

} // namespace

bool scope_handle_lt(uint8_t lt_raw, bool in_menu, bool stick_mode) {
    static bool s_down = false;
    if (lt_raw > g_scope_lt_max.load(std::memory_order_relaxed))
        g_scope_lt_max.store(lt_raw, std::memory_order_relaxed);   // diagnostic; race harmless
    // scopedevray also lifts the stick-mode refusal: under SimVR the FP route never establishes
    // (no valid controller pose), so the plugin sits in stick mode on foot and the toggle would
    // be unreachable by the very automation the dev ray exists for. Release builds synthesize no
    // ray, so even a hand-set key there just toggles a state the frame end immediately drops.
    if (!g_cfg.scope_enabled || in_menu || (stick_mode && !g_cfg.scope_dev_ray)) {
        s_down = false;
        return false;   // menus and seats keep the game's own trigger semantics
    }
    const uint8_t on_t  = (uint8_t)(g_cfg.scope_thresh * 255.0f);
    const uint8_t off_t = (uint8_t)(on_t / 2);   // hysteresis: no re-fire on an analog wobble
    if (!s_down && lt_raw >= on_t) {
        s_down = true;
        g_scope_active = !g_scope_active.load();
        g_scope_lt_edges.fetch_add(1, std::memory_order_relaxed);
    } else if (s_down && lt_raw <= off_t) {
        s_down = false;
    }
    return g_cfg.scope_eat_lt;
}

// Scope toggle from a BOUND BUTTON (bindscope) instead of the left trigger.
//
// Its own edge state rather than a reuse of scope_handle_lt's: both can be live at once (the
// trigger keeps working when a button is bound, so a player who binds one and dislikes it is not
// stranded without a scope), and sharing one `s_down` would let a trigger release cancel a button
// press. A digital button needs no hysteresis -- that exists for analog wobble on the trigger.
//
// Same refusals as the trigger path, and for the same reasons: menus and vehicle seats keep the
// game's own semantics.
void scope_handle_button(bool down, bool in_menu, bool stick_mode) {
    static bool s_down = false;
    if (!g_cfg.scope_enabled || in_menu || (stick_mode && !g_cfg.scope_dev_ray)) {
        s_down = false;
        return;
    }
    if (down && !s_down) {
        s_down = true;
        g_scope_active = !g_scope_active.load();
        g_scope_lt_edges.fetch_add(1, std::memory_order_relaxed);
    } else if (!down) {
        s_down = false;
    }
}

// ---- PLACEMENT CALIBRATION (hold Delete) ------------------------------------------------------
//
// The gesture the End pose-match uses, applied to the pane: FREEZE the thing being fitted, move
// the hand until the relationship looks right, release to capture. Here the pane holds its world
// transform while the key is down -- so as the weapon hand moves, the pane appears to stay put and
// you are really choosing where it sits ON THE GUN. On release the component's own relative
// transform IS the answer (the engine has been recomputing it against the rig on every write), so
// the capture is a readback rather than a derivation, and it lands directly in the mount=1 keys.
namespace {
bool s_calib_held = false;
Vec3 s_calib_pos{0.0f, 0.0f, 0.0f};
Vec3 s_calib_rot{0.0f, 0.0f, 0.0f};   // pitch, yaw, roll
bool s_calib_frozen = false;
} // namespace

static void scope_apply(API::UObject* rig, uint32_t tick);

void scope_notice_focus(const Vec3& world_hit, bool valid, uint32_t tick) {
    s_focus       = world_hit;
    s_focus_valid = valid;
    s_focus_tick  = tick;
}

void scope_notice_ray(const Vec3& origin, const Vec3& target, API::UObject* rig, uint32_t tick) {
    s_ray_origin = origin;
    s_ray_target = target;
    s_last_notice_tick = tick;
    if (!g_cfg.scope_enabled) return;

    // DO NOT ADD A WEAPON-SWITCH DETECTOR HERE. Two obvious ones were tried and both are DEAD on
    // this title, measured 2026-08-15:
    //   * comparing `rig` -- the rig is the PAWN's component, identical across a swap. A whole
    //     session of switching weapons produced exactly ONE "attached to rig" line.
    //   * comparing fp_weapon_actor() -- deliberately sticky. resolve_rig's fast path KEEPS the
    //     cached weapon actor precisely because the rig is unchanged (Rig.cpp:204), so it does not
    //     report a new gun either.
    // The signal that does move is the arms being hidden mid-swap, which propagates onto our
    // attached pane. That detector lives at the bottom of scope_apply, where the pane is in hand.
    // scopeforce holds the pane on with no input at all -- the config file is the one automation
    // channel that reaches this code under SimVR, where neither XInput injection nor the sim's
    // own VR trigger can arrive at the LT hook.
    // ---- CALIBRATION KEEPS THE PANE ALIVE.
    //
    // This early-out is why scope calibration could not be completed at all. The ENTIRE calibration
    // gesture lives inside scope_apply() below, so the instant the scope closed -- letting go of LT,
    // or two-handed aiming stealing it -- scope_apply stopped being called and the gesture's RELEASE
    // EDGE never fired. The pane vanishing mid-calibration and the capture never happening were one
    // bug, not two: you cannot finish a hold-and-release gesture whose handler stops running.
    //
    // So while a calibration is in play the pane is held on regardless of the trigger:
    //   * armed for a per-weapon trim (the Script UI button), or
    //   * the calibration key is down right now, or
    //   * a hold is already in flight.
    // All three are transient and self-clearing, so ordinary behaviour returns the moment the
    // gesture ends or the arm is cancelled -- no sticky state to get wedged in.
    const bool calib_key_down = game_window_focused() && (g_cfg.scope_calib_key != 0) &&
                                ((GetAsyncKeyState(g_cfg.scope_calib_key) & 0x8000) != 0);
    const bool calib_wants_pane = s_calib_held || calib_key_down ||
                                  scope_offset_armed() || scope_base_armed();

    if (!g_scope_active.load() && !g_cfg.scope_force && !calib_wants_pane) {
        hide_pane_if_shown();
        return;
    }
    // Apply IMMEDIATELY, on the same tick the ray was produced: the consume-on-the-next-tick
    // shape this replaced put a whole ~32 Hz tick between hand and pane, which the first
    // headset pass reported as visible smoothing lag.
    scope_apply(rig, tick);
}

void scope_frame_end(uint32_t tick) {
    // The compositor quad's housekeeping: config edges, the atlas-cell request, retirement when no
    // ray arrived. Above the early-outs deliberately, exactly like xrlayer_tick() -- retirement has
    // to happen on precisely the ticks the scope is NOT feeding rays. One int test while off.
    scopelayer_tick(tick);
#if HALO_VR_DEV
    dev_blam_zoom_probe(tick);
    dev_scene_rt_probe(tick);
#if HALO_VR_DEV
    dev_rt_scan(tick);
#endif
    scope_blit_tick();
#endif
    // Park-when-stale only; placement lives in scope_notice_ray now. Within a tick the dev ray
    // notices BEFORE this call and the real reticule path notices AFTER it, so an age over one
    // tick means no ray source is alive -- menus, seats, death, or the feature switched off.
    // GRACE, not one tick. The ray source is the on-foot reticule block, which is itself gated
    // (rig resolved, origin readable, trace, ...) and so does not fire on literally every tick --
    // with a 1-tick window the pane was hiding and re-showing constantly (55 show events in one
    // session's log). A few ticks of slack removes the flicker while still parking the pane
    // promptly on the transitions that matter, which last far longer than this.
    // MEASURED 2026-08-27: 4 IS TOO SMALL, AND THIS IS THE ROOT OF "it turns on then turns itself
    // off". ScopeLayer's identical grace was instrumented to report the real gap between notices;
    // it reports **5**. So this test fires on an ordinary frame with a perfectly healthy ray
    // source, closes the scope, and the log then blames "the aim-ray source is dying" -- which
    // sent at least one investigation after the rig and the reticule for something that was
    // always this constant.
    //
    // The "~125 ms at ~32 Hz" note was an assumption about the tick domain rather than a
    // measurement: this runs from scope_frame_end(), which ticks PER FRAME, so at 90 Hz four ticks
    // is ~44 ms and the window was under half what the comment claimed. Raised to match
    // ScopeLayer's, which survived a live session at this cadence.
    //
    // It cannot be unbounded -- this is what parks the pane on menus, seats, death and weapon
    // switches -- but every one of those lasts far longer than a second, exactly as the comment
    // above already argued.
    constexpr uint32_t kRayGraceTicks = 48;
    const bool stale = (tick - s_last_notice_tick) > kRayGraceTicks;
    if (!g_cfg.scope_enabled || stale) {
        // SAY SO WHEN THIS CLOSES AN OPEN SCOPE. This path used to clear the toggle in silence,
        // which meant "I press the trigger and the scope does not come up" had no entry in the log
        // at all -- the scope opened, this closed it a few ticks later, and the only visible
        // evidence was a repeated "scope ON" with nothing after it. That is the single most
        // confusing failure this feature can produce, because everything upstream looks healthy:
        // the toggle fires, edges count up, and the pane genuinely was shown.
        //
        // Rate-limited: if the ray source is dead the condition is true every tick, and a line per
        // tick would bury the log it is meant to make readable.
        if (g_scope_active.load()) {
            static uint32_t s_closed = 0;
            if ((s_closed++ % 32) == 0) {
                API::get()->log_info("[Halo-CampE-UEVR] scope: CLOSED BY %s (#%u) -- ray last seen "
                                     "%u ticks ago, grace is %u. If you pressed the trigger and the "
                                     "pane vanished immediately, the aim-ray source is dying, NOT "
                                     "the scope: check whatever owns the rig/reticule this session.",
                                     !g_cfg.scope_enabled ? "scope=0" : "RAY STALENESS", s_closed,
                                     tick - s_last_notice_tick, kRayGraceTicks);
            }
            g_scope_active = false;
        }
        hide_pane_if_shown();
    }
}

// The active path: validate, (re)create, place, capture, show. Only called with a same-tick ray.
// THE STATE THAT HAD NO VOICE: scope ACTIVE, scope_apply reached, then a silent return -- the
// player sees a scope that will not open and the log says nothing at all. Edge-triggered on the
// reason (and rate-limited) so a steady failure costs one line, not a stream. Prints the two
// LATCHES that can pin this state, because neither is visible from outside: s_failed kills the
// scope for the session, and s_pane_shown can claim the pane is already up when it is not.
static void scope_report_bail(uint32_t tick) {
    static const char* s_prev = nullptr;
    static uint32_t    s_last = 0;
    const char* why = (s_ec_bail != nullptr) ? s_ec_bail : "unknown";
    if (why == s_prev && (tick - s_last) < 320) return;   // ~10 s at 32 Hz
    s_prev = why;
    s_last = tick;
    API::get()->log_info("[Halo-CampE-UEVR] scope: ACTIVE but NOT PRESENTING -- ensure_components "
                         "refused: %s | s_failed=%d s_pane_shown=%d pane=%s capture=%s rt=%s. "
                         "The scope believes it is on; nothing is being built or drawn.",
                         why, (int)s_failed, (int)s_pane_shown,
                         s_pane.empty()    ? "empty" : "held",
                         s_capture.empty() ? "empty" : "held",
                         s_rt.empty()      ? "empty" : "held");
}

static void scope_apply(API::UObject* rig, uint32_t tick) {
    // Validate every handle through the object array before use; a dead one simply gets remade
    // (pawn swap kills pane+capture, a level change kills all three). get_checked resets dead
    // handles as a side effect, which is what lets ensure_components rebuild piecewise.
    s_pane.get_checked(L"StaticMeshComponent");
    s_capture.get_checked(L"SceneCaptureComponent2D");
    s_rt.get_checked(L"TextureRenderTarget2D");
    if (s_pane.empty() || s_capture.empty() || s_rt.empty()) {
        if (s_pane.empty()) { s_pane_mid.reset(); s_pane_shown = false; s_pane_anchored = false; }
        s_attached_rig = nullptr; s_attached_socket = nullptr;   // a rebuilt component is not attached to anything
        s_failed = false;           // allow one clean re-create against the new owners
        // ...AND ACTUALLY ASK FOR THAT RE-CREATE. THIS LINE IS THE WHOLE FIX.
        //
        // Clearing s_failed only REMOVES A BLOCK -- it does not call ensure_components(), and all
        // three of its call sites are gated on a CONFIG change (scope_shape, scoperes, the
        // capture-source format). A level teardown changes none of them, so after a mission ->
        // menu -> mission the freed pane/capture/RT were detected here, silently forgiven, and
        // then never rebuilt: the scope was dead for the rest of the PROCESS. That is why a
        // checkpoint reload did not fix it either.
        //
        // The symptom this produces is deceptive, which is why it survived several sessions: the
        // toggle works and `edges` counts up (input is fine), the ray is fine, no force-close
        // fires -- and scope_report_bail() stays SILENT, because ensure_components() is never
        // reached to bail out of. Measured 2026-09-06 from the user's own repro: captures frozen
        // at 3329 across 3.7 min of gameplay while edges climbed 25 -> 27.
        //
        // Re-entering through the shape gate reuses the one rebuild path that is already tested,
        // rather than adding a fourth ensure_components() call site with its own conditions.
        // CLAUDE.md's "ensure_components() is NOT per-tick" note is the general form of this bug.
        s_shape_built = -1;
    }

    // ---- MARK THE PANE AS A FIRST-PERSON PRIMITIVE.
    //
    // The FP arms and weapon draw over the pane. The reason is UE 5.5's FIRST PERSON RENDERING, not
    // the classic foreground depth-priority group: docs\Community-HaloCE-VR-Research.md records this
    // camera carrying bUseFirstPersonParameters with FirstPersonFOV 78 and FirstPersonScale 0.15,
    // so first-person primitives are drawn through a separate projection whose DEPTH IS COMPRESSED
    // to 0.15. That compression is what keeps them in front of everything; it is not an ordering
    // group we can join by raising a priority.
    //
    // So the pane opts into the same path: FirstPersonPrimitiveType = FirstPerson (1) on
    // UPrimitiveComponent (EFirstPersonPrimitiveType: 0 None, 1 FirstPerson, 2 WorldSpaceRepresentation).
    //
    // DepthPriorityGroup is written too, and deliberately so -- it is one byte, it is harmless if
    // the first-person path is the one doing the work, and if FirstPersonPrimitiveType turns out
    // NOT to exist on this build it is the only ordering lever left. The log below says which of
    // the two actually resolved rather than leaving us to guess from the picture.
    //
    // APPLIED ON CHANGE, not per tick: these are reflected property writes.
    //
    // ⚠ WATCH THE PANE'S APPARENT SIZE AND DISTANCE. Joining the first-person projection means the
    // pane is drawn at FirstPersonFOV with a 0.15 depth scale rather than the main projection, and
    // it is placed in WORLD space ~63 cm along the aim ray. If it suddenly looks the wrong size or
    // sits at the wrong depth, that is this change, and scopefpdepth=0 reverts it live.
    {
        static int s_fp_depth_applied = -1;
        const int want = g_cfg.scope_fp_depth ? 1 : 0;
        if (s_fp_depth_applied != want) {
            // DO NOT LATCH UNTIL THE WRITE ACTUALLY LANDS. The first pass after a config load can
            // arrive before the pane handle is live, and latching on the attempt meant one null
            // pane permanently disabled the feature -- which is exactly what the first live run
            // reported: both properties -1 on the pane while the ARMS carried
            // FirstPersonPrimitiveType=1, i.e. the property plainly exists on this build and we
            // simply asked a component that was not there yet. Retry until one resolves.

            // Write one byte property by name, reporting the readback. -1 = the property does not
            // exist on this build, which is the answer that matters.
            auto write_byte = [](API::UObject* o, const wchar_t* name, uint8_t v) -> int {
                if (o == nullptr) return -1;
                auto* c = o->get_class();
                if (c == nullptr) return -1;
                auto* p = c->find_property(name);
                if (p == nullptr) return -1;
                auto* b = reinterpret_cast<uint8_t*>(o) + p->get_offset();
                *b = v;
                return (int)*b;
            };
            auto read_byte = [](API::UObject* o, const wchar_t* name) -> int {
                if (o == nullptr) return -1;
                auto* c = o->get_class();
                if (c == nullptr) return -1;
                auto* p = c->find_property(name);
                if (p == nullptr) return -1;
                return (int)*(reinterpret_cast<uint8_t*>(o) + p->get_offset());
            };

            auto* pane_c = s_pane.get_checked(L"StaticMeshComponent");
            const int fp_rb  = write_byte(pane_c, L"FirstPersonPrimitiveType", (uint8_t)want);
            const int dpg_rb = write_byte(pane_c, L"DepthPriorityGroup",       (uint8_t)want);

            // What the ARMS actually use. This comparison IS the diagnosis: whichever property the
            // arms carry a non-default value for is the one doing the work on this build.
            auto* arms = rig_tracked_component();
            const int arms_fp  = read_byte(arms, L"FirstPersonPrimitiveType");
            const int arms_dpg = read_byte(arms, L"DepthPriorityGroup");

            API::get()->log_info(
                "[Halo-CampE-UEVR] scope: FP-ordering -> want=%d | pane FirstPersonPrimitiveType=%d "
                "DepthPriorityGroup=%d | arms FirstPersonPrimitiveType=%d DepthPriorityGroup=%d "
                "(-1 = property absent on this build)",
                want, fp_rb, dpg_rb, arms_fp, arms_dpg);
            if (fp_rb >= 0 || dpg_rb >= 0) {
                s_fp_depth_applied = want;          // landed: stop retrying
            } else {
                API::get()->log_info("[Halo-CampE-UEVR] scope: neither ordering property resolved on the "
                                     "pane THIS pass (pane=%p). Not latching -- will retry next tick. "
                                     "If the arms report FirstPersonPrimitiveType>=0 above while the pane "
                                     "keeps reporting -1, the handle is the problem, not the property.",
                                     (void*)pane_c);
            }
        }
    }

    // scopeshape is live too, but the MESH is chosen at creation -- so a change rebuilds the pane
    // (and with it the MID). Rare enough that a rebuild is the simplest correct answer.
    if (s_shape_built != g_cfg.scope_shape) {
        if (s_shape_built != -1 && !s_pane.empty()) {
            API::get()->log_info("[Halo-CampE-UEVR] scope: shape -> %s, rebuilding the pane",
                                 g_cfg.scope_shape == 1 ? "round" : "square");
            // DESTROY the old one, do not merely drop the handle. Resetting the TrackedObject
            // only forgets our pointer -- the component stays alive on the actor and keeps
            // rendering, so toggling the shape left BOTH panes on screen at once (measured) and
            // leaked another every time.
            if (auto* old_pane = s_pane.get_checked(L"StaticMeshComponent")) {
                alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
                *reinterpret_cast<void**>(q) = old_pane;
                old_pane->call_function(L"K2_DestroyComponent", q);
            }
            s_pane.reset(); s_pane_mid.reset();
            s_pane_shown = false; s_pane_anchored = false; s_attached_rig = nullptr; s_attached_socket = nullptr;
        }
        s_shape_built = g_cfg.scope_shape;
        if (!ensure_components(rig)) { scope_report_bail(tick); return; }
    }

    // scoperes AND scopesrc are both live, and BOTH change what has to be allocated. Size was
    // always handled here; FORMAT was not, and that was a real gap rather than an oversight of
    // degree: rt_format_for_source() returns RTF_RGBA8 only for scopesrc=2 and RTF_RGBA16f for
    // everything else, so switching sources across that boundary needs a NEW target. Without this
    // the old target survived, the new source rendered into a format it did not match, and the
    // only way to apply a source change was to restart the game -- which is exactly how a live
    // scopesrc edit could look inert while being obeyed, and how the compositor's "DXGI format 10
    // is not in the swapchain's family" refusal outlived the edit that should have cleared it.
    const int want_fmt = rt_format_for_source(g_cfg.scope_capture_src);
    const bool size_changed = (s_rt_size_applied   != g_cfg.scope_rt_size);
    const bool fmt_changed  = (s_rt_format_applied != want_fmt);
    if (size_changed || fmt_changed) {
        const int old_fmt = s_rt_format_applied;
        s_rt.reset();
        if (!ensure_components(rig)) { scope_report_bail(tick); return; }
        // Name WHICH input forced it. "rebuilt" alone cannot distinguish a size edit from a source
        // edit, and those have different consequences downstream (the compositor cares only about
        // the format, the pane only about the size).
        API::get()->log_info("[Halo-CampE-UEVR] scope: render target rebuilt at %dx%d, format %d "
                             "(was %d) -- triggered by %s%s%s. Format follows scopesrc: 2 = RGBA8 "
                             "(8-bit, the only one the compositor layer accepts), anything else = "
                             "RGBA16f.",
                             g_cfg.scope_rt_size, g_cfg.scope_rt_size, want_fmt, old_fmt,
                             size_changed ? "scoperes" : "",
                             (size_changed && fmt_changed) ? " + " : "",
                             fmt_changed ? "scopesrc" : "");
    }

    auto* cap  = s_capture.ptr;
    auto* pane = s_pane.ptr;

#if HALO_VR_DEV
    // The translucency probe, parented to the capture so it needs no aiming. Inline no-op in a
    // shipping build -- it draws geometry into the player's view, which is precisely what the
    // compile-time dev gate exists to keep out of a headset.
    dev_test_object(cap);
#endif

    // The ray. Guard a degenerate target (trace start == end) before normalising.
    Vec3 dir{s_ray_target.x - s_ray_origin.x,
             s_ray_target.y - s_ray_origin.y,
             s_ray_target.z - s_ray_origin.z};
    const float len = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
    if (len < 1e-3f) return;
    dir.x /= len; dir.y /= len; dir.z /= len;

    // CAPTURE: on the shot line, looking along it -- so what the reticule promises is what the
    // pane shows. It sits scope_cam_dist out rather than at the origin, which puts it BEYOND the
    // pane: from the origin the pane fills its view and the scope photographs the back of its
    // own display surface (reported as "the scope shows black"). Staying on the ray keeps the
    // direction exact; only the origin moves, so distant aim is unaffected.
    // NOTE: ray_yaw/ray_pitch describe the SHOT LINE's direction. They are only the camera's
    // direction too when the camera sits ON that line -- see the look-at below, which is what makes
    // the image agree with the aim from any origin.
    const float ray_yaw   = std::atan2(dir.y, dir.x) / DEG2RAD;
    const float ray_pitch = std::atan2(dir.z, std::sqrt(dir.x * dir.x + dir.y * dir.y)) / DEG2RAD;
    // Written every tick in track mode 0 (accurate: the image keeps looking down the live shot
    // line) and once in mode 1 (rigid: purest controller motion, image can drift off the shot
    // line). Between writes the camera rides the rig either way -- it is attached.
    static bool s_cam_anchored = false;
    static float s_cam_anchored_dist = -1.0f, s_cam_anchored_roll = 0.0f;
    if (s_attached_rig == nullptr) s_cam_anchored = false;
    const bool cam_write = (g_cfg.scope_cam_track == 0) || !s_cam_anchored ||
                           s_cam_anchored_dist != g_cfg.scope_cam_dist ||
                           s_cam_anchored_roll != g_cfg.scope_cam_roll;
    if (cam_write) {
        // WHERE THE CAMERA SITS ALONG THE RAY -- and this is the origin question, not the
        // direction one.
        //
        // s_ray_origin IS THE HEAD. Measured 2026-09-07 from the SIGHTLINE line, whose `origin`
        // equals `hmd` on every tick. So placing the capture there makes the scope a HEAD-mounted
        // camera pointed down the shot line, not a weapon-mounted optic -- and the two differ by
        // however far your head is from the gun. hmdleash was hiding it by pinning origin to the
        // standing reference (its own log reports "absorbed drift"); with the leash off, a 24 cm
        // body step moves the capture 24 cm sideways and the pane shows the ground BESIDE what you
        // are aiming at. Reported exactly that way in a headset.
        //
        // Mode 1 puts the camera on the PANE instead -- the surface that is already weapon-mounted
        // and already carries the calibration -- and keeps the ray's DIRECTION, so the image still
        // looks down the true shot line. That is what a real optic does: the tube moves with the
        // rifle, not with your skull.
        Vec3 cam_origin = s_ray_origin;
        if (g_cfg.scope_cam_origin != 0 && pane != nullptr) {
            Vec3 pw{};
            if (call_ret_vec3(pane, L"K2_GetComponentLocation", &pw)) cam_origin = pw;
        }

        // LOOK AT THE TARGET, DO NOT LOOK ALONG THE RAY. This is the whole correctness argument
        // and it was got wrong once already.
        //
        // `dir` is the SHOT LINE's direction, from the ray's origin. Pointing the camera along it
        // is only right when the camera is ON that line. Move the camera off it -- which is exactly
        // what scopecamorigin=1 does, and what walking away from your body does with hmdleash off --
        // and the camera becomes PARALLEL to the shot line but displaced from it, so it never
        // converges on the target. The miss equals the offset between the two origins, at every
        // depth, and it looks like "the pane shows the ground beside what I am aiming at".
        //
        // Aiming the camera AT s_ray_target instead puts the target dead centre from ANY origin,
        // which is what the in-pane reticule promises. Note this is a no-op when cam_origin is the
        // ray origin (the two directions are then identical), so mode 0 is bit-for-bit unchanged
        // and only the displaced case is corrected.
        // CONVERGE ON THE REAL HIT, not on the ray's arbitrary 500 cm point. Those are the same
        // DIRECTION but different DEPTHS, and depth is the whole of this problem: a camera aimed at
        // 5 m does not look at a target 3 m away once it is displaced sideways, and the miss grows
        // with (distance error x lateral offset). The headset symptom was the world-space reticule
        // sitting correctly ON the target while the target slid out of the pane during a strafe --
        // the reticule was at the true hit and the camera was aimed past it.
        // STALENESS, NOT JUST VALIDITY. s_focus_valid says a focus was published ONCE; it says
        // nothing about whether that was this tick or a minute ago, and it is never set false. The
        // publish site sits behind gates (see xrlayer_note_publish_gate: the arm rig's parent must
        // resolve, and the rig-held-at-origin path skips it entirely), so missing it is a normal
        // event rather than an exotic one -- and missing it used to freeze the capture's convergence
        // point permanently while the world reticule carried on tracking. That presents as the pane
        // looking somewhere the shot does not go, which is indistinguishable from a frame or
        // calibration error and was chased as one.
        const bool focus_fresh = s_focus_valid && (tick - s_focus_tick) <= kFocusGraceTicks;
        {
            static bool s_said_stale = false;
            if (focus_fresh == s_said_stale) {
                s_said_stale = !focus_fresh;
                API::get()->log_info(
                    "[Halo-CampE-UEVR] scope: convergence point %s (age %u ticks). %s",
                    focus_fresh ? "FRESH again" : "went STALE -- falling back to the ray's own "
                                                  "500 cm point",
                    (unsigned)(tick - s_focus_tick),
                    focus_fresh ? "The capture is converging on the traced hit again."
                                : "The pane still looks down the shot LINE, but converges nearer "
                                  "than the real hit. If this line stays up, the reticule publish "
                                  "site is not being reached -- check the layer's gate value.");
            }
        }
        const Vec3 focus = focus_fresh ? s_focus : s_ray_target;
        Vec3 look{focus.x - cam_origin.x,
                  focus.y - cam_origin.y,
                  focus.z - cam_origin.z};
        const float look_len = std::sqrt(look.x * look.x + look.y * look.y + look.z * look.z);
        if (look_len > 1.0f) { look.x /= look_len; look.y /= look_len; look.z /= look_len; }
        else                 { look = dir; }   // degenerate: target on top of the camera

        const Vec3 cam_pos{cam_origin.x + look.x * g_cfg.scope_cam_dist,
                           cam_origin.y + look.y * g_cfg.scope_cam_dist,
                           cam_origin.z + look.z * g_cfg.scope_cam_dist};
        set_world_location(cap, cam_pos);
        // Roll counter-rotates the CONTENT (see scope_cam_roll in Config.hpp): the displayed
        // texture read 90 deg sideways while the pane transform was verified correct, so the fix
        // is applied to what the capture writes, never to the pane.
        //
        // PER-SHAPE UV COMPENSATION. The correction is a property of the MESH's UVs, not of the
        // capture: the Cylinder's cap unwraps 90 deg round from the Plane's, so the roll that
        // lands upright on the square arrives sideways on the round lens (measured in-headset,
        // both shapes, same scopecamroll). Carrying that here keeps scopecamroll meaning "the
        // trim for MY setup" instead of silently meaning something different per shape.
        const float uv_roll = (g_cfg.scope_shape == 1) ? 90.0f : 0.0f;

        // ROLL LOCK: let the image ride the lens instead of the world. A world-pinned roll means
        // canting the weapon spins the picture inside the tube, which no real optic does -- and
        // it makes calibration roll-sensitive, because a cant held during the gesture is baked
        // into the captured pane while the image stays level. Tracking the pane's roll DELTA from
        // where it was placed leaves scope_cam_roll meaning what it always did.
        // ROLL LOCK, done in VECTORS rather than Euler angles.
        //
        // The previous two attempts read the pane's Euler ROLL COMPONENT and treated it as the
        // weapon's physical roll. It is not: this camera's rotation is built as yaw/pitch from
        // the aim ray plus a roll term, and an Euler decomposition redistributes between those
        // three as pitch changes -- degenerating entirely near vertical. That produced exactly
        // the two reported symptoms: the image FLIPPING past a pitch threshold, and roll that
        // tracked unevenly through a sweep while happening to be right at 90 degrees.
        //
        // The frame-correct question is "where is the lens's up, measured about the aim axis",
        // which is a vector projection and has no gimbal in it. Take the pane's own up, remove
        // the component along the aim direction, and roll the camera to match what remains --
        // the construction pancreations uses for the same purpose ("preserve the rifle's roll
        // while keeping up perpendicular to the actual bullet direction").
        float roll_lock = 0.0f;
        if (g_cfg.scope_cam_lock == 1) {
            Vec3 pane_up{};
            if (call_ret_vec3(pane, L"GetUpVector", &pane_up)) {
                // Perpendicularise the lens up against the aim direction.
                const float along = pane_up.x * dir.x + pane_up.y * dir.y + pane_up.z * dir.z;
                Vec3 u{pane_up.x - dir.x * along,
                       pane_up.y - dir.y * along,
                       pane_up.z - dir.z * along};
                const float ulen = std::sqrt(u.x * u.x + u.y * u.y + u.z * u.z);
                // Degenerate only when the lens's up axis lies along the aim ray; hold the last
                // good value rather than snapping through the singularity.
                if (ulen > 1e-3f) {
                    u.x /= ulen; u.y /= ulen; u.z /= ulen;

                    // The up vector the camera would have at roll = 0, i.e. the rotator built
                    // from this yaw/pitch alone. Rolling from that to `u` about the aim axis is
                    // the angle we want, and it is signed by construction.
                    const float cp_n = std::cos(ray_pitch * DEG2RAD);
                    const float sp_n = std::sin(ray_pitch * DEG2RAD);
                    const float cy_n = std::cos(ray_yaw * DEG2RAD);
                    const float sy_n = std::sin(ray_yaw * DEG2RAD);
                    const Vec3 n{-sp_n * cy_n, -sp_n * sy_n, cp_n};   // natural up (roll = 0)

                    const float dot_nu = n.x * u.x + n.y * u.y + n.z * u.z;
                    const Vec3 cross_nu{n.y * u.z - n.z * u.y,
                                        n.z * u.x - n.x * u.z,
                                        n.x * u.y - n.y * u.x};
                    const float sin_nu = cross_nu.x * dir.x + cross_nu.y * dir.y + cross_nu.z * dir.z;
                    s_roll_lock_last = std::atan2(sin_nu, dot_nu) / DEG2RAD;
                }
                roll_lock = s_roll_lock_last;
            }
        }
        // ROTATION FROM THE LOOK VECTOR, not from the shot line's direction. Placing the camera at
        // the target-facing direction and then rotating it along the RAY's direction would undo the
        // whole point: the two differ by exactly the angle the parallax was made of. They are
        // identical whenever the camera is on the ray, so mode 0 is unchanged.
        const float cam_yaw   = std::atan2(look.y, look.x) / DEG2RAD;
        const float cam_pitch = std::atan2(look.z,
                                    std::sqrt(look.x * look.x + look.y * look.y)) / DEG2RAD;
        set_world_rotation(cap, cam_pitch, cam_yaw,
                           g_cfg.scope_cam_roll + uv_roll + roll_lock);
#if HALO_VR_DEV
        // ---- DID THE AIM ACTUALLY TAKE? Read the capture back and compare. -----------------
        //
        // This partitions the two ways "the pane is not looking at the traced hit" can happen, which
        // are indistinguishable from a screenshot and were chased as one:
        //
        //   err_deg LARGE  -> the WRITE did not survive. The capture is attached to an animated
        //                     socket, so the engine recomposes world = parent * relative every
        //                     frame; a world write that loses that race is overwritten by the bone.
        //   err_deg ~0     -> the write took and the capture is looking exactly where we asked --
        //                     so the FOCUS POINT is what is wrong, and focus_age says whether it is
        //                     stale. This is the case that exonerates the whole reticule/frame lane.
        //
        // Rate-limited to ~1 s: it costs one reflection read, and the question is a steady-state one.
        {
            static uint32_t s_last_chk = 0;
            if (tick - s_last_chk >= 32) {
                s_last_chk = tick;
                Vec3 cfwd{};
                if (call_ret_vec3(cap, L"GetForwardVector", &cfwd)) {
                    Vec3 want = look;
                    const float wl = std::sqrt(want.x*want.x + want.y*want.y + want.z*want.z);
                    const float cl = std::sqrt(cfwd.x*cfwd.x + cfwd.y*cfwd.y + cfwd.z*cfwd.z);
                    if (wl > 1e-4f && cl > 1e-4f) {
                        want.x /= wl; want.y /= wl; want.z /= wl;
                        cfwd.x /= cl; cfwd.y /= cl; cfwd.z /= cl;
                        float d = want.x*cfwd.x + want.y*cfwd.y + want.z*cfwd.z;
                        if (d >  1.0f) d =  1.0f;
                        if (d < -1.0f) d = -1.0f;
                        const float err_deg = std::acos(d) / DEG2RAD;
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] scope CAPTURE AIM: err=%.2fdeg (asked "
                            "(%.3f,%.3f,%.3f), reads (%.3f,%.3f,%.3f)) focus_age=%u fresh=%d "
                            "camorigin=%d. err near 0 means the write took and any remaining "
                            "mismatch is the FOCUS POINT, not the capture.",
                            err_deg, want.x, want.y, want.z, cfwd.x, cfwd.y, cfwd.z,
                            (unsigned)(tick - s_focus_tick), (int)focus_fresh,
                            g_cfg.scope_cam_origin);
                    }
                }
            }
        }
#endif
        s_cam_anchored = true;
        s_cam_anchored_dist = g_cfg.scope_cam_dist;
        s_cam_anchored_roll = g_cfg.scope_cam_roll;
    }

    const float fov = g_cfg.scope_base_fov / (g_cfg.scope_zoom > 1.0f ? g_cfg.scope_zoom : 1.0f);
    if (std::fabs(fov - s_fov_applied) > 1e-3f) {
        if (auto* p = cap->get_property_data<float>(L"FOVAngle")) {
            *p = fov;
            s_fov_applied = fov;
        }
    }

    // CAPTURE SOURCE, live. This used to be written ONLY at component creation, so changing
    // scopesrc did nothing to an existing capture: a session that had gone black on a
    // post-processed source stayed black no matter what the config said, and the key was
    // documented as live while not being live. Re-applied on change, with the exposure pin
    // following it, because the pin only matters for the post-processed sources.
    static int s_src_applied = -1;
    static float s_exposure_applied = -1.0f;
    static int s_aa_applied = -2;   // -1 is a meaningful value here, so seed outside the range
    if (s_src_applied != g_cfg.scope_capture_src ||
        s_exposure_applied != g_cfg.scope_exposure ||
        s_aa_applied != g_cfg.scope_aa) {
        s_aa_applied = g_cfg.scope_aa;
        if (g_cfg.scope_aa >= 0) set_capture_aa(cap, g_cfg.scope_aa);
        const int prev_src = s_src_applied;
        s_src_applied = g_cfg.scope_capture_src;
        s_exposure_applied = g_cfg.scope_exposure;
        if (auto* p = cap->get_property_data<uint8_t>(L"CaptureSource"))
            *p = (uint8_t)g_cfg.scope_capture_src;
        // The target's FORMAT depends on the source (LDR sources want 8-bit), so a live source
        // change that crosses that boundary has to rebuild the target too -- otherwise the new
        // source writes into a target allocated for the old one.
        // Compare against the format the LIVE TARGET was actually built at, not against prev_src.
        // The size/format guard earlier in this function now rebuilds on a format change too, and
        // it runs first -- so testing prev_src here would rebuild a second time in the same tick,
        // for a target that is already correct. s_rt_format_applied is the one fact both guards
        // agree on, which is the point of tracking it rather than inferring it from inputs.
        if (prev_src >= 0 &&
            s_rt_format_applied != rt_format_for_source(g_cfg.scope_capture_src)) {
            s_rt.reset();
            if (!ensure_components(rig)) { scope_report_bail(tick); return; }
            API::get()->log_info("[Halo-CampE-UEVR] scope: render target rebuilt for source %d "
                                 "(format %s)", g_cfg.scope_capture_src,
                                 rt_format_for_source(g_cfg.scope_capture_src) == 2 ? "RGBA8"
                                                                                    : "RGBA16f");
        }
        if ((g_cfg.scope_capture_src == 8 || g_cfg.scope_capture_src == 9) &&
            g_cfg.scope_exposure > 0.0f) {
            pin_capture_exposure(cap, g_cfg.scope_exposure);
        }
        API::get()->log_info("[Halo-CampE-UEVR] scope: capture source -> %d (exposure pin %s)",
                             g_cfg.scope_capture_src,
                             ((g_cfg.scope_capture_src == 8 || g_cfg.scope_capture_src == 9) &&
                              g_cfg.scope_exposure > 0.0f) ? "on" : "off");
    }

    // AUTOEXPOSURE BIAS -- on change, never per tick, like every other capture lever here.
    //
    // Deliberately AFTER the source block: pin_capture_exposure() runs in there and also writes
    // this field, so re-applying afterwards keeps the two in agreement regardless of which path
    // last touched it. Seeded at 0.0f so a config that never sets the key writes nothing at all
    // and the capture keeps whatever it would otherwise inherit -- the default has to be inert.
    static float s_bias_applied = 0.0f;
    if (s_bias_applied != g_cfg.scope_autoexposure_bias) {
        s_bias_applied = g_cfg.scope_autoexposure_bias;
        apply_autoexposure_bias(cap, g_cfg.scope_autoexposure_bias);
    }

    // LUMEN. On change, seeded at -1 so a config that never sets the key writes nothing and the
    // capture keeps the engine's forced-off state -- the default must not change anyone's frame
    // time, since re-enabling it makes the capture run a second Lumen scene.
    static int s_lumen_applied = -1;
    if (s_lumen_applied != g_cfg.scope_lumen) {
        s_lumen_applied = g_cfg.scope_lumen;
        if (g_cfg.scope_lumen >= 0) apply_capture_lumen(cap, g_cfg.scope_lumen);
    }

    // COLOUR GRADE. On change, and re-runnable: toggling the key off and on re-copies, which is how
    // you pick up a grade that changed with the level. It does NOT un-copy on 0 -- the override bits
    // stay set once written, so reverting means a fresh session, and the log says so.
    // GAIN. On change, seeded at 0 so a config that never sets it writes nothing -- 0 means "leave
    // ColorGain alone", which is not the same as 1.0 (neutral), because writing 1.0 would set the
    // override bit and stop the capture inheriting a gain from anywhere else.
    static float s_gain_applied = 0.0f;
    if (s_gain_applied != g_cfg.scope_gain) {
        s_gain_applied = g_cfg.scope_gain;
        if (g_cfg.scope_gain > 0.0f) apply_capture_gain(cap, g_cfg.scope_gain);
    }

    static int s_grade_applied = -1;
    const int grade_want = g_cfg.scope_pp_grade ? 1 : 0;
    if (s_grade_applied != grade_want) {
        s_grade_applied = grade_want;
        if (grade_want != 0) {
            copy_pp_grading_onto_capture(cap);
        } else {
            API::get()->log_info("[Halo-CampE-UEVR] scope: scopeppgrade off -- the fields already "
                                 "copied STAY copied (their bOverride_ bits are set); a fresh "
                                 "session is what actually reverts it.");
        }
    }

    // THE BLACK-FINAL-COLOUR LEVERS (see Config.hpp for why each one is a candidate).
    //
    // Applied on change, with a READBACK, because the whole point is that a null result has to
    // mean "not the cause" rather than "the write went nowhere" -- the mistake that put three
    // false conclusions in the findings doc when the console channel turned out to be inert.
    // MAIN-VIEW-FAMILY LEVERS. All three are uint32:1 bitfields, so they are written through the
    // FBoolProperty mask rather than a raw byte -- writing the byte would clobber the neighbouring
    // flags packed beside them. Each logs a readback, because "set a bool and nothing changed" is
    // the exact shape of a write that never landed.
    {
        struct MV { const wchar_t* name; int want; };
        const MV flags[] = {
            { L"bMainViewFamily",     g_cfg.scope_main_family },
            { L"bMainViewResolution", g_cfg.scope_main_res },
            { L"bMainViewCamera",     g_cfg.scope_main_cam },
        };
        static int s_applied[3] = { -2, -2, -2 };
        for (int i = 0; i < 3; ++i) {
            if (flags[i].want < 0 || s_applied[i] == flags[i].want) continue;
            s_applied[i] = flags[i].want;
            auto* c = cap->get_class();
            auto* prop = (c != nullptr) ? c->find_property(flags[i].name) : nullptr;
            if (prop == nullptr) {
                API::get()->log_info("[Halo-CampE-UEVR] scope: %s NOT FOUND on this build",
                                     narrow(flags[i].name).c_str());
                continue;
            }
            auto* bp = static_cast<API::FBoolProperty*>(prop);
            auto* byte = reinterpret_cast<uint8_t*>(cap) + bp->get_offset();
            const uint8_t mask = bp->get_field_mask();
            if (flags[i].want != 0) *byte = (uint8_t)(*byte | mask);
            else                    *byte = (uint8_t)(*byte & ~mask);
            API::get()->log_info("[Halo-CampE-UEVR] scope: %s -> %d (mask=0x%02X, readback %d)",
                                 narrow(flags[i].name).c_str(), flags[i].want, (unsigned)mask,
                                 (*byte & mask) != 0);
        }
    }

    // POST-PROCESS LEVERS, applied on change and in a DELIBERATE ORDER.
    //
    // scopeppcopy overwrites the ENTIRE FPostProcessSettings struct, so it must run FIRST -- any
    // bloom or exposure written before it would be silently wiped by the copy, and the result
    // would read as "the lever did nothing". Everything we want to survive is re-applied after.
    static int   s_ppcopy_applied = -1;
    static float s_bloom_applied  = -1.0f;
    if (s_ppcopy_applied != (int)g_cfg.scope_pp_copy || s_bloom_applied != g_cfg.scope_bloom) {
        s_ppcopy_applied = (int)g_cfg.scope_pp_copy;
        s_bloom_applied  = g_cfg.scope_bloom;
        if (g_cfg.scope_pp_copy) copy_pp_settings_onto_capture(cap);
        if (g_cfg.scope_bloom > 0.0f) force_capture_bloom(cap, g_cfg.scope_bloom);
        // Re-assert the exposure pin: the copy above may have replaced it, and a post-processed
        // capture without a pinned exposure was the original cause of the fade-to-black.
        if ((g_cfg.scope_capture_src == 8 || g_cfg.scope_capture_src == 9) &&
            g_cfg.scope_exposure > 0.0f) {
            pin_capture_exposure(cap, g_cfg.scope_exposure);
        }
    }

#if HALO_VR_DEV
    dev_blendable_probe(cap);
#endif
    static float s_dof_applied = -1.0f, s_dof_focus_applied = -1.0f;
    if (s_dof_applied != g_cfg.scope_dof || s_dof_focus_applied != g_cfg.scope_dof_focus) {
        s_dof_applied = g_cfg.scope_dof;
        s_dof_focus_applied = g_cfg.scope_dof_focus;
        if (g_cfg.scope_dof > 0.0f) enable_capture_dof(cap, g_cfg.scope_dof, g_cfg.scope_dof_focus);
    }
    static int   s_persist_applied = -1;
    static float s_ppw_applied     = -2.0f;
    if (s_persist_applied != g_cfg.scope_persist) {
        s_persist_applied = g_cfg.scope_persist;
        cap->set_bool_property(L"bAlwaysPersistRenderingState", g_cfg.scope_persist != 0);
        int rb = -1;
        if (auto* c = cap->get_class()) {
            if (auto* p = c->find_property(L"bAlwaysPersistRenderingState"))
                rb = *(reinterpret_cast<uint8_t*>(cap) + p->get_offset()) != 0;
        }
        API::get()->log_info("[Halo-CampE-UEVR] scope: bAlwaysPersistRenderingState -> %d "
                             "(readback %d)", g_cfg.scope_persist, rb);
    }
    if (s_ppw_applied != g_cfg.scope_pp_weight) {
        s_ppw_applied = g_cfg.scope_pp_weight;
        if (g_cfg.scope_pp_weight >= 0.0f) {
            if (auto* p = cap->get_property_data<float>(L"PostProcessBlendWeight"))
                *p = g_cfg.scope_pp_weight;
        }
        float rb = -1.0f;
        if (auto* p = cap->get_property_data<float>(L"PostProcessBlendWeight")) rb = *p;
        API::get()->log_info("[Halo-CampE-UEVR] scope: PostProcessBlendWeight -> %.2f "
                             "(readback %.2f)%s", g_cfg.scope_pp_weight, rb,
                             g_cfg.scope_pp_weight < 0.0f ? "  [left at engine default]" : "");
    }
    // bCameraCutThisFrame is the exception to the on-change rule, and deliberately so: the
    // renderer clears it after every capture (SceneCaptureRendering.cpp:1410), so writing it once
    // would test nothing at all. It is a single bitfield write on a component we own, not an
    // engine call, so a per-tick write here costs nothing measurable.
    // Written through the field mask rather than set_bool_property because this one is a
    // `uint32 : 1` bitfield sharing its byte with neighbouring flags -- the same reason the
    // exposure pin above resolves its bOverride_ masks by hand.
    if (g_cfg.scope_cam_cut != 0) {
        static int s_cut_state = -1;   // -1 unknown, 0 property missing, 1 writing
        if (s_cut_state != 0) {
            auto* c = cap->get_class();
            auto* prop = (c != nullptr) ? c->find_property(L"bCameraCutThisFrame") : nullptr;
            if (prop == nullptr) {
                s_cut_state = 0;
                API::get()->log_info("[Halo-CampE-UEVR] scope: bCameraCutThisFrame NOT FOUND -- "
                                     "scopecamcut cannot be tested on this build");
            } else {
                auto* bp = static_cast<API::FBoolProperty*>(prop);
                auto* byte = reinterpret_cast<uint8_t*>(cap) + bp->get_offset();
                *byte = (uint8_t)(*byte | bp->get_field_mask());
                if (s_cut_state != 1) {
                    s_cut_state = 1;
                    API::get()->log_info("[Halo-CampE-UEVR] scope: bCameraCutThisFrame forced every "
                                         "tick (mask=0x%02X, readback %d) -- temporal history reset "
                                         "each capture", (unsigned)bp->get_field_mask(),
                                         (*byte & bp->get_field_mask()) != 0);
                }
            }
        }
    }

    // Capture cadence. Mode 1 hands the cadence to the renderer (bCaptureEveryFrame while the
    // pane is up); mode 0 issues manual captures every scopediv ticks. The bool is only written
    // when the mode changes -- the on-change rule for engine calls.
    static int s_cap_mode_applied = -1;
    if (s_cap_mode_applied != g_cfg.scope_cap_mode) {
        s_cap_mode_applied = g_cfg.scope_cap_mode;
        cap->set_bool_property(L"bCaptureEveryFrame", g_cfg.scope_cap_mode == 1);
        API::get()->log_info("[Halo-CampE-UEVR] scope: capture mode -> %s",
                             g_cfg.scope_cap_mode == 1 ? "every-frame" : "manual/divisor");
    }
    const int div = (g_cfg.scope_div < 1) ? 1 : g_cfg.scope_div;
    if (g_cfg.scope_cap_mode == 0 && (tick % (uint32_t)div) == 0) {
        alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
        cap->call_function(L"CaptureScene", q);
        g_scope_captures.fetch_add(1, std::memory_order_relaxed);
    }

    // ---- THE GUN-MOUNTED COMPOSITOR QUAD (scopelayer, default off) ----------------------------
    //
    // Fed the SAME ray and the SAME render target the capture above just wrote, on the same tick,
    // so the quad and the image inside it cannot disagree about the shot line. Costs one int test
    // while the feature is off, which is the shipped state.
    //
    // s_rt_size_applied, not scoperes: a live scoperes edit rebuilds the target, and the two
    // disagree for a tick. The layer validates its whole resolved D3D12 chain against this number,
    // so a guess here is a wrong ValueAgreement rather than a cosmetic slip.
    if (g_cfg.scope_layer != 0) {
        ScopeLayerFeed feed;
        feed.ray_origin    = s_ray_origin;
        feed.ray_target    = s_ray_target;
        feed.render_target = s_rt.ptr;
        feed.rt_dim        = s_rt_size_applied;
        if (g_cfg.scope_layer_roll != 0) {
            // The lens's own up carries the weapon's cant. Same source and same call the roll lock
            // above reads; ScopeLayer perpendicularises it against the aim axis.
            feed.lens_up_valid = call_ret_vec3(pane, L"GetUpVector", &feed.lens_up);
        }
        // THE PANE'S OWN WORLD PLACEMENT, for scopelayerfollowpane. Read from the PLACED component
        // rather than recomputed from scopedist/right/up, so every calibration the pane has already
        // absorbed -- per-weapon wpnscope trims, the socket handshake, a live recalibration -- is
        // inherited by the quad without a second copy of the maths to keep in step.
        // scope_size is the pane's diameter in cm (the mesh's own scale is derived from it), so it
        // is the width the quad needs to subtend the same angle at the same distance.
        {
            Vec3 pw{};
            if (call_ret_vec3(pane, L"K2_GetComponentLocation", &pw)) {
                feed.pane_world    = pw;
                feed.pane_width_cm = g_cfg.scope_size;
                feed.pane_valid    = (g_cfg.scope_size > 0.0f);
                // The pane's own facing axis, so the quad can inherit ORIENTATION as well as
                // position. Which way it points is resolved against the aim downrange, not assumed.
                // All three axes. The facing one is picked downstream by alignment with the
                // aim, because the pane's forward is NOT its normal (measured: -0.17 against aim).
                const bool ax_f = call_ret_vec3(pane, L"GetForwardVector", &feed.pane_fwd);
                const bool ax_r = call_ret_vec3(pane, L"GetRightVector",   &feed.pane_right);
                const bool ax_u = call_ret_vec3(pane, L"GetUpVector",      &feed.pane_up_axis);
                feed.pane_fwd_valid = (ax_f || ax_r || ax_u);
                // The rig's rotation on THIS tick, measured alongside the pane axes so the two
                // describe one instant. The submit thread differences it against the render-rate
                // rig to cancel the interval the pane has already moved through.
                if (rig != nullptr) {
                    // AXES, not a rotator. K2_GetComponentRotation would hand back Euler angles and
                    // put a decomposition on this path -- the exact thing that made the pane jitter
                    // on roll. Three vectors carry the same information with no ordering and no
                    // singularity.
                    const bool rp = call_ret_vec3(rig, L"K2_GetComponentLocation", &feed.rig_pos);
                    const bool rf = call_ret_vec3(rig, L"GetForwardVector", &feed.rig_fwd);
                    const bool rr = call_ret_vec3(rig, L"GetRightVector",   &feed.rig_right);
                    const bool ru = call_ret_vec3(rig, L"GetUpVector",      &feed.rig_up);
                    feed.rig_rot_valid = (rp && rf && rr && ru);
                }
            } else {
                static bool said = false;
                if (!said) {
                    said = true;
                    API::get()->log_info("[Halo-CampE-UEVR] scope: K2_GetComponentLocation is not "
                                         "callable on this build -- scopelayerfollowpane cannot "
                                         "read the pane's placement and the quad falls back to the "
                                         "scopelayer* offsets. Calibration will NOT be shared.");
                }
            }
        }
        // THE RETICULE'S TRUE POSITION IN THE PANE. Project the traced impact point into the
        // CAPTURE CAMERA'S OWN frustum -- its real world basis read back from the component, not
        // the aim ray we asked it to look down. Those are the same thing only while the camera is
        // re-anchored to the ray every tick; under recoil, sway, or a camera that rides the weapon
        // they diverge, and that divergence is exactly the error a centred reticule cannot show.
        //
        // The render target is SQUARE, so the vertical half-angle equals the horizontal one and a
        // single tan() serves both axes. s_fov_applied is the FOV actually WRITTEN to the capture,
        // never the config value -- the two differ for a tick after a zoom change, and using the
        // config one would swing the reticule on exactly the frames the zoom is moving.
        {
            // HOISTED OUT OF THE PROJECTION'S CONDITION. These axes are needed by the roll
            // reconciliation for BOTH reticule modes, and mode 2 computes its own offset inside
            // XrLayer without ever consulting scoperetproject -- so gating the reads on that key
            // would have left mode 2's correction silently unavailable.
            Vec3 cpos{}, cf{}, cr{}, cu{};
            const bool cam_axes =
                call_ret_vec3(cap, L"K2_GetComponentLocation", &cpos) &&
                call_ret_vec3(cap, L"GetForwardVector", &cf) &&
                call_ret_vec3(cap, L"GetRightVector",   &cr) &&
                call_ret_vec3(cap, L"GetUpVector",      &cu);
            if (cam_axes) {
                feed.cam_fwd        = cf;
                feed.cam_right      = cr;
                feed.cam_up         = cu;
                feed.cam_axes_valid = true;
            }
            if (g_cfg.scope_ret_project != 0 && s_fov_applied > 0.1f && cam_axes) {
                // PROJECT THE POINT THE CAMERA IS ACTUALLY AIMED AT. Since the capture now
                // LOOKS AT the traced hit, that hit is dead centre by construction and this should
                // compute (0,0) every tick -- which makes it a free CONSISTENCY CHECK rather than a
                // correction: a non-zero result means the camera and the reticule have gone back to
                // disagreeing about the target, the exact fault that made the image slide off
                // during a strafe.
                //
                // Projecting s_ray_target here instead would be actively wrong now: that is the
                // arbitrary 500 cm ray point, and from the pane's origin it is NOT centre, so the
                // reticule would be pushed off by the parallax the look-at just removed.
                const Vec3 aimpt = (s_focus_valid && (tick - s_focus_tick) <= kFocusGraceTicks)
                                       ? s_focus : s_ray_target;
                const Vec3 d{aimpt.x - cpos.x,
                             aimpt.y - cpos.y,
                             aimpt.z - cpos.z};
                const float z = d.x * cf.x + d.y * cf.y + d.z * cf.z;
                // Behind the camera, or so close the divide explodes: leave it centred rather than
                // fling the reticule somewhere. A scope pointed at something 1 cm away is not a
                // case worth a special answer.
                if (z > 1.0f) {
                    const float t = std::tan(s_fov_applied * 0.5f * 3.14159265f / 180.0f);
                    if (t > 1e-4f) {
                        const float x = d.x * cr.x + d.y * cr.y + d.z * cr.z;
                        const float y = d.x * cu.x + d.y * cu.y + d.z * cu.z;
                        float u = (x / z) / t;
                        float v = (y / z) / t;
                        // SIGN CONVENTION IS NOT PROVABLE FROM HERE. UE's camera right/up and the
                        // compositor quad's local +X/+Y are both "right and up", but nothing
                        // guarantees they agree once the pose has crossed the module's UE->XR
                        // mapping. Rather than guess and ship a mirrored reticule that looks
                        // plausible until you aim off-axis, the flips are config -- one edit in a
                        // headset instead of a rebuild.
                        if (g_cfg.scope_ret_flip_x != 0) u = -u;
                        if (g_cfg.scope_ret_flip_y != 0) v = -v;
                        // Clamp to the pane. Outside -1..1 the impact point is off the edge of what
                        // the scope can see, and pinning it to the rim is honest: it says "that way"
                        // without claiming a position the image does not contain.
                        feed.ret_u     = (u < -1.0f) ? -1.0f : (u > 1.0f ? 1.0f : u);
                        feed.ret_v     = (v < -1.0f) ? -1.0f : (v > 1.0f ? 1.0f : v);
                        feed.ret_valid = true;
#if HALO_VR_DEV
                        // ACTUALLY REPORT THE CONSISTENCY CHECK THIS BLOCK CLAIMS TO BE.
                        //
                        // The comment above says the capture looks AT the aim point, so this should
                        // compute (0,0) every tick and a non-zero result means the camera and the
                        // reticule have gone back to disagreeing about the target. That is exactly
                        // the right diagnostic -- and it was never printed, so the check has never
                        // once been read. With the capture readback showing err=0.00deg and the
                        // focus fresh, this is the only remaining place a lateral offset can enter.
                        //
                        // z is the distance along the capture's forward to the aim point; a value
                        // far from the traced range would mean we are projecting the wrong point.
                        {
                            static uint32_t s_last_uv = 0;
                            if (tick - s_last_uv >= 32) {
                                s_last_uv = tick;
                                API::get()->log_info(
                                    "[Halo-CampE-UEVR] scope RET PROJECT: u=%.4f v=%.4f "
                                    "(pre-flip %.4f/%.4f, flips %d/%d) z=%.1fcm fov=%.2f "
                                    "campos=(%.1f,%.1f,%.1f) aim=(%.1f,%.1f,%.1f). Both u and v "
                                    "should be ~0: the capture looks AT this point. A non-zero u is "
                                    "a LATERAL offset of the reticule on the pane, in half-widths.",
                                    feed.ret_u, feed.ret_v, (x / z) / t, (y / z) / t,
                                    g_cfg.scope_ret_flip_x, g_cfg.scope_ret_flip_y,
                                    z, s_fov_applied, cpos.x, cpos.y, cpos.z,
                                    aimpt.x, aimpt.y, aimpt.z);
                            }
                        }
#endif
                    }
                }
            }
        }
        scopelayer_notice(feed, tick);
    }

    // PANE: ANCHORED, NOT DRIVEN.
    //
    // The placement below runs only when the pane has no anchor yet (fresh component, new rig)
    // or a placement knob moved. Every other tick the pane is not touched at all: it is a rigid
    // child of the rig, so the engine composes its world transform each render frame and it
    // moves EXACTLY as the controller does. Re-writing it per tick is what made it re-derive
    // from the aim ray at ~32 Hz -- drifting relative to the hand and inheriting aim jitter that
    // the lens then magnified.
    //
    // Placement is still computed in WORLD space against the aim ray, so "just over the aim
    // ray" stays the meaning of scopedist/scoperight/scopeup; the attachment (KeepWorld)
    // converts that one write into the relative offset it then holds.
    // THE ATTACHMENT, EVERY TICK, AND AHEAD OF THE CALIBRATION GESTURE.
    //
    // Deliberately not in ensure_components(), which despite its name runs only on a rebuild (see
    // update_pane_attachment). And deliberately ABOVE the gesture below, which RETURNS early to keep
    // ownership of the pane while held -- so a call placed after it is simply not reached on the
    // calibration path.
    //
    // That placement was the bug reported 2026-09-06: "calibrated the sniper, then found the scope
    // pane in a very different position than how I calibrated it". The capture reads the pane's
    // RelativeLocation/RelativeRotation, and scope_offset_capture() then subtracts the RIG-frame
    // global fit to get a delta. With the attachment update unreachable during the gesture, the pane
    // stayed parented at socket PrimaryWeapon throughout, so the capture recorded SOCKET-frame
    // numbers and the subtraction mixed two frames. The stored delta was then re-applied as a
    // rig-frame offset and converted to the socket a second time -- hence a pane nowhere near where
    // it was placed.
    //
    // Calling it here makes s_calib_held actually do its job: placing stays true for the whole
    // gesture, the pane is held on the RIG ORIGIN, and the capture is guaranteed to be in the same
    // frame as the base it is differenced against and as the shipped canonical fit. The re-attach
    // uses KeepWorld, so dropping to the rig at the start of a calibration does not move the pane.
    //
    // Ordering rule worth keeping: this call must precede every early return in scope_apply that can
    // happen while the pane is alive. It is the only thing that keeps the attachment frame and the
    // calibration frame in agreement.
    update_pane_attachment(rig, true, tick);

    // ---- CALIBRATION GESTURE, before the ordinary placement so it owns the pane while held.
    {
        // Foreground-gated like every keyboard gesture (see game_window_focused in Config.hpp):
        // DELETE is an ordinary editing key, and GetAsyncKeyState registers it from ANY app --
        // a background game was capturing scope placements off desktop keystrokes.
        const bool held = game_window_focused() && (g_cfg.scope_calib_key != 0) &&
                          ((GetAsyncKeyState(g_cfg.scope_calib_key) & 0x8000) != 0);
        if (held && !s_calib_held) {
            // Rising edge: remember exactly where the pane is now and hold it there.
            s_calib_frozen = call_ret_vec3(pane, L"K2_GetComponentLocation", &s_calib_pos) &&
                             read_component_rotation(pane, &s_calib_rot);
            API::get()->log_info("[Halo-CampE-UEVR] scope calib: HOLDING -- the pane is frozen in "
                                 "place%s. Move your weapon hand until it sits where you want it, "
                                 "then release.",
                                 s_calib_frozen ? "" : " (FAILED to read its transform)");
        } else if (!held && s_calib_held && s_calib_frozen) {
            // Falling edge: the engine has been recomputing the relative transform against the
            // rig on every frozen write, so the answer is simply there to be read.
            auto* rel_loc = pane->get_property_data<double>(L"RelativeLocation");
            auto* rel_rot = pane->get_property_data<double>(L"RelativeRotation");
            if (rel_loc != nullptr && rel_rot != nullptr) {
                g_cfg.scope_dist  = (float)rel_loc[0];
                g_cfg.scope_right = (float)rel_loc[1];
                g_cfg.scope_up    = (float)rel_loc[2];
                g_cfg.scope_rot_p = (float)rel_rot[0];
                g_cfg.scope_rot_y = (float)rel_rot[1];
                g_cfg.scope_rot_r = (float)rel_rot[2];
                // ---- PER-WEAPON TRIM CLAIMS THE CAPTURE, IF ARMED.
                //
                // The gesture above is identical either way -- freeze the pane, move it, release.
                // Only the DESTINATION differs: armed, the result is stored as this weapon's delta
                // against the global fit and the global fit is deliberately left alone, because
                // overwriting it would move every OTHER weapon to suit this one.
                //
                // The absolute values assigned just above are exactly what the delta is measured
                // from, so this must run after them and before write_calib_file().
                // The base arm is consumed by ANY completed capture, including one the per-weapon
                // table claimed. Leaving it set would hold the pane open indefinitely and re-arm a
                // calibration the player has already finished.
                scope_base_arm_clear();
                if (scope_offset_capture()) {
                    note_pane_anchored();
                    note_pane_roll_reference(pane);
                    // claimed: the global scope fit is deliberately left untouched.
                } else {
                g_cfg.scope_mount = 1;            // the capture IS a rig-relative offset
                g_cfg.scope_calib_valid = true;
                s_relative_rejected = false;
                note_pane_anchored();             // keep it exactly here; do not re-place
                // The pose the player just chose IS the roll reference -- so any cant they were
                // holding during the gesture costs them nothing afterwards.
                note_pane_roll_reference(pane);
                write_calib_file();
                API::get()->log_info("[Halo-CampE-UEVR] scope calib: CAPTURED -- fwd=%.1f "
                                     "right=%.1f up=%.1f cm, rot=%.1f/%.1f/%.1f. Saved; delete "
                                     "the scope block in the calibration file to undo.",
                                     g_cfg.scope_dist, g_cfg.scope_right, g_cfg.scope_up,
                                     g_cfg.scope_rot_p, g_cfg.scope_rot_y, g_cfg.scope_rot_r);
                }
            } else {
                API::get()->log_info("[Halo-CampE-UEVR] scope calib: could not read the pane's "
                                     "relative transform -- nothing captured, nothing changed.");
            }
            s_calib_frozen = false;
        }
        s_calib_held = held;

        if (held && s_calib_frozen) {
            // Hold the world pose against the rig's motion. This is the one place the pane is
            // deliberately written every tick: the hand is moving underneath it on purpose.
            set_world_location(pane, s_calib_pos);
            set_world_rotation(pane, s_calib_rot.x, s_calib_rot.y, s_calib_rot.z);
            // Both gates, same as the show at the bottom of this function -- hide_pane_if_shown
            // sets bHiddenInGame too, so clearing only bVisible would leave the pane invisible.
            if (!s_pane_shown) {
                set_hidden_in_game(pane, false);
                set_visibility(pane, true);
                s_pane_shown = true;
            }
            return;   // no ordinary placement while the gesture owns the pane
        }
    }

    if (s_anchored_mount != g_cfg.scope_mount) s_relative_rejected = false;   // an edit re-tries
    if ((!s_pane_anchored || pane_placement_changed()) && g_cfg.scope_mount == 1
        && !s_relative_rejected) {
        // RELATIVE MOUNT (default). The knobs ARE the offset from the rig: X forward, Y right,
        // Z up in rig space, with the facing trims as a plain relative rotation. Nothing here
        // consults the aim ray, so rotation and translation cannot interact -- the reported
        // "applying a rotation changes how the translation variables are evaluated".
        set_relative_location(pane, g_cfg.scope_dist, g_cfg.scope_right, g_cfg.scope_up);
        set_relative_rotation(pane, g_cfg.scope_rot_p, g_cfg.scope_rot_y, g_cfg.scope_rot_r);
        const double s = g_cfg.scope_size / 100.0;   // Engine Plane is 100 cm across
        // The round lens is a Cylinder squashed on its own axis; the flat Plane keeps uniform
        // scale. Both end up scope_size across.
        if (g_cfg.scope_shape == 1) set_relative_scale(pane, s, s, s * kLensThickness);
        else                        set_relative_scale(pane, s, s, s);

        // ---- WHY THE PANE'S VIEW CAN COME OUT MIRRORED -----------------------------------------
        //
        // DEV-ONLY. Reported 2026-09-06: after the socket re-parent, the image INSIDE the pane was
        // flipped and motion in it inverted -- on the BASE fit, on every weapon, so not a
        // calibration artefact. Three candidates were ruled out by reading: the capture cannot drift
        // (scopecamtrack=0 re-aims it in world space every tick), the camera is not behind the pane
        // (90 cm vs a 63.57 cm pane), and the stored rotations are ~17 deg off canonical, not 180.
        //
        // The remaining candidate is SCALE SIGN. The scale above is RELATIVE, so it composes with the
        // parent's. A skeletal socket on a mirrored limb can carry a NEGATIVE scale component, and a
        // quad with an odd number of negative axes renders inside-out: the texture reads mirrored and
        // apparent motion within it reverses -- exactly the report. Attaching to the rig ORIGIN would
        // not show it; attaching at PrimaryWeapon would.
        //
        // So log what we WROTE against what the component ENDED UP WITH, plus which parent it is on.
        // A relative scale of +s that reads back negative in world is the whole answer; if both are
        // positive, scale is innocent and the next suspect is the quad's facing.
#if HALO_VR_DEV
        {
            auto* rs = pane->get_property_data<double>(L"RelativeScale3D");
            Vec3 wrot{};
            const bool got_rot = read_component_rotation(pane, &wrot);
            API::get()->log_info(
                "[Halo-CampE-UEVR] scope PANE FRAME: parent=%s | wrote rel scale %.4f | "
                "RelativeScale3D=(%.4f,%.4f,%.4f) | world rot=(%.1f,%.1f,%.1f)%s | "
                "rel rot=(%.1f,%.1f,%.1f)",
                (s_attached_socket != nullptr) ? "socket PrimaryWeapon"
                                               : (s_attached_rig ? "rig origin" : "NONE"),
                s,
                rs ? rs[0] : 0.0, rs ? rs[1] : 0.0, rs ? rs[2] : 0.0,
                got_rot ? wrot.x : 0.0f, got_rot ? wrot.y : 0.0f, got_rot ? wrot.z : 0.0f,
                got_rot ? "" : " (rot READ FAILED)",
                g_cfg.scope_rot_p, g_cfg.scope_rot_y, g_cfg.scope_rot_r);
        }
#endif
        note_pane_anchored();
        note_pane_roll_reference(pane);
        // READ BACK WHERE IT LANDED. A relative offset assumes the rig's local axes are the
        // usual X-forward/Y-right/Z-up and that its scale is 1; if either is untrue the pane is
        // somewhere unintended and the only symptom is "I cannot see it". This says exactly
        // where it went, and how far that is from the aim origin the player is looking along.
        Vec3 got{};
        if (call_ret_vec3(pane, L"K2_GetComponentLocation", &got)) {
            const float dxr = got.x - s_ray_origin.x;
            const float dyr = got.y - s_ray_origin.y;
            const float dzr = got.z - s_ray_origin.z;
            const float range = std::sqrt(dxr * dxr + dyr * dyr + dzr * dzr);
            // Component of the offset ALONG the aim ray: it should be about scope_dist. A tiny
            // or negative value means the local axes are not what this assumed.
            const float along = dxr * dir.x + dyr * dir.y + dzr * dir.z;
            API::get()->log_info("[Halo-CampE-UEVR] scope: pane RELATIVE mount -- asked fwd=%.0f "
                                 "right=%.0f up=%.0f cm, landed %.0f cm from the aim origin "
                                 "(%.0f cm of that along the aim ray)%s",
                                 g_cfg.scope_dist, g_cfg.scope_right, g_cfg.scope_up,
                                 range, along,
                                 (range > g_cfg.scope_dist * 3.0f || along < g_cfg.scope_dist * 0.25f)
                                     ? "  <-- NOT WHERE IT WAS ASKED FOR: the rig's local axes or "
                                       "scale are not what a relative offset assumed. Falling "
                                       "back to the aim-ray placement for this session."
                                     : "");
            // SELF-HEAL. An invisible pane is indistinguishable from a broken feature, and the
            // relative mount rests on an assumption about the rig (standard local axes, unit
            // scale) that this is the first evidence for either way. If the readback says the
            // pane is not where it was asked to be, drop to the aim-ray placement -- which is
            // measured in world space and cannot be wrong about the rig -- rather than leaving
            // the player with nothing. Loud, once, and scopemount=0 makes it permanent.
            if (range > g_cfg.scope_dist * 3.0f || along < g_cfg.scope_dist * 0.25f) {
                s_relative_rejected = true;
                s_pane_anchored = false;   // re-place through the world path on the next tick
            }
        }
        API::get()->log_info("[Halo-CampE-UEVR] scope: pane mounted RELATIVE to the rig "
                             "(rot=%.0f/%.0f/%.0f, size=%.0f, %s)",
                             g_cfg.scope_rot_p, g_cfg.scope_rot_y, g_cfg.scope_rot_r,
                             g_cfg.scope_size, g_cfg.scope_shape == 1 ? "round" : "square");
    } else if (!s_pane_anchored || pane_placement_changed()) {
        Vec3 right{dir.y, -dir.x, 0.0f};
        const float rlen = std::sqrt(right.x * right.x + right.y * right.y);
        if (rlen > 1e-3f) { right.x /= rlen; right.y /= rlen; }
        else              { right = Vec3{0.0f, 1.0f, 0.0f}; }   // looking straight up/down
        const Vec3 up{right.y * dir.z - right.z * dir.y,
                      right.z * dir.x - right.x * dir.z,
                      right.x * dir.y - right.y * dir.x};

        const Vec3 pos{
            s_ray_origin.x + dir.x * g_cfg.scope_dist + right.x * g_cfg.scope_right + up.x * g_cfg.scope_up,
            s_ray_origin.y + dir.y * g_cfg.scope_dist + right.y * g_cfg.scope_right + up.y * g_cfg.scope_up,
            s_ray_origin.z + dir.z * g_cfg.scope_dist + right.z * g_cfg.scope_right + up.z * g_cfg.scope_up};

        const float dx = s_ray_origin.x - pos.x;
        const float dy = s_ray_origin.y - pos.y;
        const float dz = s_ray_origin.z - pos.z;
        const float horiz = std::sqrt(dx * dx + dy * dy);
        const float face_yaw   = std::atan2(dy, dx) / DEG2RAD;
        const float face_pitch = std::atan2(dz, horiz) / DEG2RAD;

        set_world_location(pane, pos);
        set_world_rotation(pane,
                           face_pitch + g_cfg.scope_rot_p,
                           face_yaw + g_cfg.scope_rot_y,
                           g_cfg.scope_rot_r);
        // Engine Plane is 100 cm across. The ROUND lens must also be squashed on its own axis --
        // this path was applying a UNIFORM scale, so the Cylinder stayed a full-height drum
        // instead of a disc ("the cylinder is quite thick"). The relative-mount path already
        // flattened it; both now agree.
        const double s = g_cfg.scope_size / 100.0;
        if (g_cfg.scope_shape == 1) set_world_scale3(pane, s, s, s * kLensThickness);
        else                        set_world_scale(pane, s);
        note_pane_anchored();
        note_pane_roll_reference(pane);
        API::get()->log_info("[Halo-CampE-UEVR] scope: pane ANCHORED to the rig "
                             "(dist=%.0f right=%.0f up=%.0f size=%.0f rot=%.0f/%.0f/%.0f) "
                             "-- rigid from here until a knob moves",
                             g_cfg.scope_dist, g_cfg.scope_right, g_cfg.scope_up,
                             g_cfg.scope_size, g_cfg.scope_rot_p, g_cfg.scope_rot_y,
                             g_cfg.scope_rot_r);
    }

    // Live-tunable brightness, same as the reticule tint path.
    if (auto* mid = s_pane_mid.get_checked(L"MaterialInstanceDynamic")) {
        set_tint(mid, g_cfg.scope_bright);
    }

    // ---- WHICH SURFACE IS DISPLAYING THE SCOPE? ------------------------------------------------
    //
    // scopelayerhidepane hands the display to the gun-mounted compositor quad instead of drawing
    // both -- but ONLY while the layer is PROVEN to be presenting this slot's art, never on "we
    // asked it to". That is the reticule lane's expensive lesson applied here: gate on live, not on
    // setup-succeeded, or a refused layer leaves the player looking at nothing at all.
    //
    // The scope is still OPEN in this state; only its display moved. So the open bookkeeping and
    // the weapon-switch watch below must keep running exactly as they do for the pane.
    static bool s_layer_owned = false;
    const bool layer_owns_display = (g_cfg.scope_layer_hide_pane != 0) && scopelayer_presenting();

    // ---- THE ARMING WINDOW: DO NOT FLASH THE IN-WORLD PANE WHILE THE QUAD IS COMING UP ---------
    //
    // layer_owns_display is gated on PROVEN presentation, which is correct and which necessarily
    // takes a few ticks after the scope opens: a capture has to reach the atlas and the slot has to
    // pass the readiness gate. During those ticks the rule below reads "the layer is not presenting,
    // so show the in-world pane" -- so the player watches the world-space pane appear and then be
    // replaced by the quad. That is the reported "I see the ws pane for a split second before the xr
    // layer one", and it is structural rather than a timing accident: nothing about hiding it FASTER
    // fixes it, because at that moment there is genuinely nothing else to show yet.
    //
    // So when the layer is CONFIGURED to own the display, show neither surface for a bounded window
    // instead of showing the one we are about to take away. An empty scope for a few ticks reads as
    // an optic powering up; the wrong image at the wrong depth does not.
    //
    // BOUNDED IS THE ENTIRE SAFETY ARGUMENT, and it is the same one the presenting gate makes: if
    // the quad never arrives -- source refused, no atlas cell, scopelayer off -- the window expires
    // and the in-world pane appears exactly as it does today. "Show nothing" must never be reachable
    // as a resting state, only as a brief one with a deadline.
    //
    // ~32 Hz tick, so 16 ticks is about half a second: comfortably longer than a healthy bring-up,
    // short enough that a failed one is a blink rather than a broken scope.
    constexpr uint32_t kArmTicks = 16;
    static uint32_t s_arm_tick = 0;
    static uint32_t s_arm_seen = 0;
    // A GAP IN TICKS MEANS THE SCOPE WAS CLOSED, which is the edge we want to arm on. Deriving it
    // from the tick gap keeps this self-contained instead of adding a hook into every close path --
    // and a missed close would only mean the window does not re-arm, which degrades to today's
    // behaviour rather than to a stuck blank pane.
    const bool reopened = (s_arm_seen == 0) || (tick - s_arm_seen > 2);
    s_arm_seen = tick;

    // ALSO RE-ARM WHEN THE QUAD STOPS PRESENTING, not only on a tick gap.
    //
    // The tick-gap test detects the scope having been CLOSED, which misses a fast close/reopen: if
    // scope_apply ran again within two ticks, `reopened` stays false and the window runs down from a
    // stale s_arm_tick. MEASURED 2026-09-07: a reopen armed from an old stamp expired at ~342 ms
    // instead of the intended ~500, and the log caught the consequence exactly --
    //   44.086 display -> in-world pane   (reopened; quad not ready)
    //   44.428 scope ON                   (window expired -> in-world pane SHOWN)
    //   44.445 display -> COMPOSITOR QUAD (quad ready, 17 ms later)
    // -- a single frame of the in-world pane, which is precisely the flash the window exists to
    // prevent. It was not removed, only moved from the start of the window to the end.
    //
    // Losing presentation is the event that actually matters and cannot be missed by a fast reopen,
    // so arm on that edge too. It needs no new bookkeeping: layer_owns_display is already computed
    // above, and while hidepane is on it IS the presenting state.
    static bool s_was_owning = false;
    const bool lost_presenting = s_was_owning && !layer_owns_display;
    s_was_owning = layer_owns_display;

    static bool s_arm_expired_said = false;
    if (lost_presenting && !reopened) {
        // Re-arm only; the weapon watch is seeded on the OPEN edge and must not be re-stamped here,
        // or every presentation hiccup would silently extend its settle grace.
        s_arm_tick = tick;
        s_arm_expired_said = false;
    }
    if (reopened) {
        s_arm_tick = tick;
        s_arm_expired_said = false;
        // SEED THE WEAPON WATCH ON THE OPEN EDGE, WHATEVER IS (OR IS NOT) DISPLAYING.
        //
        // REGRESSION FIXED HERE, caused by the arming window above. seed_weapon_watch() stamps
        // s_scope_open_tick, and the weapon-switch watch at the bottom of this function closes the
        // scope when the weapon CLASS differs and we are past kSettleTicks from that stamp. Its two
        // existing call sites are "the in-world pane was just shown" and "the compositor quad just
        // took over" -- which covered every case until the arming window introduced a THIRD state,
        // displaying NEITHER. Through that window nothing seeded, so the grace was measured from the
        // PREVIOUS scope session and had long expired: switch weapon, raise the scope, and the watch
        // closed it the moment the new weapon resolved. Exactly the symptom seed_weapon_watch's own
        // comment records having fixed once already -- "it turns on, then turns itself off really
        // quickly" -- reintroduced through a state that comment could not have anticipated.
        //
        // Seeding on the OPEN EDGE instead of on a display transition makes the invariant stronger
        // than it was: it no longer depends on which surface wins, so a future fourth display state
        // cannot break it the way the third one did. The other two sites are left alone; re-seeding
        // is idempotent and cheap.
        seed_weapon_watch(tick);
    }
    if (layer_owns_display) s_arm_tick = 0;      // the handoff completed; the window is spent
    const bool arming = (g_cfg.scope_layer_hide_pane != 0) && (g_cfg.scope_layer != 0) &&
                        !layer_owns_display && s_arm_tick != 0 &&
                        (tick - s_arm_tick) < kArmTicks;
    {
        if (!arming && s_arm_tick != 0 && (tick - s_arm_tick) >= kArmTicks && !s_arm_expired_said &&
            (g_cfg.scope_layer_hide_pane != 0) && (g_cfg.scope_layer != 0)) {
            s_arm_expired_said = true;
            API::get()->log_info("[Halo-CampE-UEVR] scope: the compositor quad did not present "
                                 "within %u ticks of the scope opening, so the in-world pane is "
                                 "being shown instead. This is the arming window's fallback doing "
                                 "its job -- the quad is not coming up, and the reason is on the "
                                 "scopelayer line above.", kArmTicks);
        }
    }
    if (layer_owns_display != s_layer_owned) {
        s_layer_owned = layer_owns_display;
        API::get()->log_info("[Halo-CampE-UEVR] scope: display -> %s",
                             layer_owns_display ? "COMPOSITOR QUAD (in-world pane hidden)"
                                                : "in-world pane");
        if (layer_owns_display) seed_weapon_watch(tick);
    }
    // Idempotent, and cheap once the pane is down: hide_pane_if_shown() returns after two cached
    // byte reads when it is already hidden, so this is not a per-tick engine call.
    // Also during arming: the note above hide_pane_if_shown() warns that WE ARE NOT THE ONLY WRITER
    // of the pane's visibility, so the game can put it back up during the window. Suppressing it
    // means keeping it down, not merely declining to raise it ourselves.
    if (layer_owns_display || arming) hide_pane_if_shown();

    if (!s_pane_shown && !layer_owns_display && !arming) {
        // Clear BOTH gates -- hide_pane_if_shown sets bHiddenInGame as well as bVisible, so
        // showing has to undo both or the pane stays invisible after the first close.
        set_hidden_in_game(pane, false);
        set_visibility(pane, true);
        s_pane_shown = true;
        seed_weapon_watch(tick);
        API::get()->log_info("[Halo-CampE-UEVR] scope ON: %.1fx (fov %.1f) rt=%d div=%d dist=%.0fcm",
                             g_cfg.scope_zoom, fov, g_cfg.scope_rt_size, div, g_cfg.scope_dist);
#if HALO_VR_DEV
        // A TEST CUBE IN THE FRAME INVALIDATES EVERY JUDGEMENT ABOUT THE CAPTURE. At 10x the
        // capture FOV is 7 deg and the default cube overflows it, so "the pane is black" and "a
        // black cube fills the pane" are the same picture -- and a cube left on from an earlier
        // experiment is invisible in the config file until someone goes looking. Say so on every
        // open, so no result can be recorded without the reader knowing the instrument was there.
        if (g_cfg.scope_test_obj != 0) {
            API::get()->log_info("[Halo-CampE-UEVR] scope ON: *** scopetest=%d IS ACTIVE -- a probe "
                                 "cube is in the capture's view; do NOT read the pane's appearance "
                                 "as a property of the capture ***", g_cfg.scope_test_obj);
        }
#endif
    } else {
        // HAS THE FP WEAPON ACTOR CHANGED? -- the weapon-switch close.
        //
        // MEASURED 2026-08-16, five consecutive switches, distinct pointers every time:
        //   FP weapon actor changed 0000021AD0CD2110 -> 0000021AD4173C40 -- scope closed
        // The game DESTROYS the first-person viewmodel actor on a swap, so the cached handle in
        // resolve_rig goes dead and the sweep finds the new one. This was dismissed early on the
        // strength of a code comment calling that cache "sticky" -- true only if the old actor
        // survives, which it does not. Two other detectors were built and shipped before this one
        // was simply tried; see the note at the top of this file.
        // Seeded at open (above), never lazily here -- a lazy seed is what let a stale value survive
        // a close and cancel the next open. A null seed still falls back to seeding on the first
        // resolvable reading, for the case where the weapon is briefly unresolvable at open time.
        //
        // SETTLING WINDOW. Opening the scope moments after a switch can catch resolve_rig
        // mid-handover: g_fp_weapon may still hold the dying old actor at open, and the new one
        // arrives a tick or two later. Seeding from that transient and then closing on the settle
        // is the residual "it turns on then turns itself off" -- intermittent, and worst right
        // after a switch, which is exactly when a player reaches for the scope. Inside the window a
        // change RE-SEEDS instead of closing; a real mid-scope switch is always far later than this.
        constexpr uint32_t kSettleTicks = 8;   // ~250 ms at ~32 Hz
        auto* w  = fp_weapon_actor();
        auto* wc = (w != nullptr) ? w->get_class() : nullptr;
        if (wc != nullptr) {
            if (s_seen_weapon_cls == nullptr) {
                s_seen_weapon = w;
                s_seen_weapon_cls = wc;
            } else if (wc != s_seen_weapon_cls) {
                if (tick - s_scope_open_tick < kSettleTicks) {
                    API::get()->log_info("[Halo-CampE-UEVR] scope: weapon resolved to a different "
                                         "class %u tick(s) after opening -- treating as the resolve "
                                         "settling, re-seeding rather than closing",
                                         tick - s_scope_open_tick);
                    s_seen_weapon = w;
                    s_seen_weapon_cls = wc;
                } else {
                    s_seen_weapon = w;
                    s_seen_weapon_cls = wc;
                    g_scope_active = false;
                    hide_pane_if_shown();
                    API::get()->log_info("[Halo-CampE-UEVR] scope: FP weapon CLASS changed to %s "
                                         "-- weapon switch, scope closed",
                                         narrow(class_name_of(w)).c_str());
                }
            }
        }
    }
}

} // namespace halo
