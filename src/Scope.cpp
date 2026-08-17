// The weapon scope. Design and doctrine in Scope.hpp; every engine call here is a pattern the
// reticule already proves on this title (component creation, MID + SlateUI binding, absolute
// world placement with the same K2_* marshalling).

#include "Scope.hpp"
#include "Config.hpp"
#include "DevTools.hpp"
#include "Rig.hpp"   // call_ret_vec3, for the placement readback

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
TrackedObject s_pane_mid;   // its MID, built from Widget3DPassThrough_Opaque
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

// PANE RE-ANCHORING. The pane is placed in world space ONCE and then left strictly alone, so it
// is a rigid child of the rig and moves exactly as the controller does. Re-writing its world
// transform every tick (what the first attachment attempt did) makes the engine recompute a new
// relative offset each time: the pane then re-derives its position from the aim ray at ~32 Hz,
// which both lets the relative placement DRIFT and feeds the aim signal's jitter into something
// a 2x lens magnifies -- the reported "I can drift its relative location, and it moves much more
// jittery than my controller". So a write happens only when something actually changed.
bool  s_pane_anchored = false;
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
bool attach_to(API::UObject* comp, API::UObject* parent) {
    if (comp == nullptr || parent == nullptr) return false;
    alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
    *reinterpret_cast<void**>(q) = parent;   // Parent
    // SocketName stays None (8 bytes at +8, already zeroed) -- attaching to the component itself.
    q[16] = 1;   // LocationRule = KeepWorld
    q[17] = 1;   // RotationRule = KeepWorld
    q[18] = 1;   // ScaleRule    = KeepWorld
    q[19] = 0;   // bWeldSimulatedBodies
    comp->call_function(L"K2_AttachToComponent", q);
    return true;
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
        { L"AutoExposureBias",          L"bOverride_AutoExposureBias",          0.0f },
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
bool ensure_components(API::UObject* rig) {
    if (s_failed) return false;
    auto* owner = (rig != nullptr) ? rig->get_outer() : nullptr;
    if (owner == nullptr) return false;

    if (s_rt.empty()) {
        auto* rt = make_scope_rt(g_cfg.scope_rt_size);
        if (rt == nullptr) {
            s_failed = true;
            API::get()->log_info("[Halo-CampE-UEVR] scope: render target creation FAILED");
            return false;
        }
        s_rt.set(rt);
        s_rt_size_applied = g_cfg.scope_rt_size;
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
    if (ready && s_attached_rig != rig) {
        // RE-ATTACH ONLY. The "a new weapon closes the scope" rule deliberately does NOT live here,
        // for two independent reasons: this function runs only while the scope is ACTIVE, so
        // closing from here would fire on the first tick after the player opened the scope on a new
        // weapon and cancel the very press that opened it -- and more fundamentally the rig does
        // not change on a weapon swap at all, so this condition never fires for that reason
        // anyway. The working detector is at the bottom of scope_apply.
        const bool a1 = attach_to(s_capture.ptr, rig);
        const bool a2 = attach_to(s_pane.ptr, rig);
        s_attached_rig = (a1 && a2) ? rig : nullptr;
        s_pane_anchored = false;   // a new parent needs one fresh placement to anchor against
        API::get()->log_info("[Halo-CampE-UEVR] scope: attached to rig %p (capture=%d pane=%d) "
                             "-- transform now composed at render rate", (void*)rig, (int)a1, (int)a2);
    }
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
    if (!g_scope_active.load() && !g_cfg.scope_force) {
        hide_pane_if_shown();
        return;
    }
    // Apply IMMEDIATELY, on the same tick the ray was produced: the consume-on-the-next-tick
    // shape this replaced put a whole ~32 Hz tick between hand and pane, which the first
    // headset pass reported as visible smoothing lag.
    scope_apply(rig, tick);
}

void scope_frame_end(uint32_t tick) {
#if HALO_VR_DEV
    dev_blam_zoom_probe(tick);
#endif
    // Park-when-stale only; placement lives in scope_notice_ray now. Within a tick the dev ray
    // notices BEFORE this call and the real reticule path notices AFTER it, so an age over one
    // tick means no ray source is alive -- menus, seats, death, or the feature switched off.
    // GRACE, not one tick. The ray source is the on-foot reticule block, which is itself gated
    // (rig resolved, origin readable, trace, ...) and so does not fire on literally every tick --
    // with a 1-tick window the pane was hiding and re-showing constantly (55 show events in one
    // session's log). A few ticks of slack removes the flicker while still parking the pane
    // promptly on the transitions that matter, which last far longer than this.
    constexpr uint32_t kRayGraceTicks = 4;   // ~125 ms at ~32 Hz
    const bool stale = (tick - s_last_notice_tick) > kRayGraceTicks;
    if (!g_cfg.scope_enabled || stale) {
        // Losing the route drops the toggle too -- coming back up should not surprise the
        // player with a pane they closed a level ago.
        if (g_scope_active.load()) g_scope_active = false;
        hide_pane_if_shown();
    }
}

// The active path: validate, (re)create, place, capture, show. Only called with a same-tick ray.
static void scope_apply(API::UObject* rig, uint32_t tick) {
    // Validate every handle through the object array before use; a dead one simply gets remade
    // (pawn swap kills pane+capture, a level change kills all three). get_checked resets dead
    // handles as a side effect, which is what lets ensure_components rebuild piecewise.
    s_pane.get_checked(L"StaticMeshComponent");
    s_capture.get_checked(L"SceneCaptureComponent2D");
    s_rt.get_checked(L"TextureRenderTarget2D");
    if (s_pane.empty() || s_capture.empty() || s_rt.empty()) {
        if (s_pane.empty()) { s_pane_mid.reset(); s_pane_shown = false; s_pane_anchored = false; }
        s_attached_rig = nullptr;   // a rebuilt component is not attached to anything
        s_failed = false;           // allow one clean re-create against the new owners
    }
    if (!ensure_components(rig)) return;

    // scopeshape is live too, but the MESH is chosen at creation -- so a change rebuilds the pane
    // (and with it the MID). Rare enough that a rebuild is the simplest correct answer.
    static int s_shape_built = -1;
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
            s_pane_shown = false; s_pane_anchored = false; s_attached_rig = nullptr;
        }
        s_shape_built = g_cfg.scope_shape;
        if (!ensure_components(rig)) return;
    }

    // scoperes is live: a size change rebuilds the RT and the bindings follow inside ensure.
    if (s_rt_size_applied != g_cfg.scope_rt_size) {
        s_rt.reset();
        if (!ensure_components(rig)) return;
        API::get()->log_info("[Halo-CampE-UEVR] scope: render target rebuilt at %dx%d",
                             g_cfg.scope_rt_size, g_cfg.scope_rt_size);
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
        const Vec3 cam_pos{s_ray_origin.x + dir.x * g_cfg.scope_cam_dist,
                           s_ray_origin.y + dir.y * g_cfg.scope_cam_dist,
                           s_ray_origin.z + dir.z * g_cfg.scope_cam_dist};
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
        set_world_rotation(cap, ray_pitch, ray_yaw,
                           g_cfg.scope_cam_roll + uv_roll + roll_lock);
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
        if (prev_src >= 0 &&
            rt_format_for_source(prev_src) != rt_format_for_source(g_cfg.scope_capture_src)) {
            s_rt.reset();
            if (!ensure_components(rig)) return;
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

    // THE BLACK-FINAL-COLOUR LEVERS (see Config.hpp for why each one is a candidate).
    //
    // Applied on change, with a READBACK, because the whole point is that a null result has to
    // mean "not the cause" rather than "the write went nowhere" -- the mistake that put three
    // false conclusions in the findings doc when the console channel turned out to be inert.
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

    if (!s_pane_shown) {
        // Clear BOTH gates -- hide_pane_if_shown sets bHiddenInGame as well as bVisible, so
        // showing has to undo both or the pane stays invisible after the first close.
        set_hidden_in_game(pane, false);
        set_visibility(pane, true);
        s_pane_shown = true;
        // SEED THE WEAPON WATCH AT OPEN TIME. It used to seed itself lazily in the detector below
        // and clear itself there too -- but that code only runs while the pane is SHOWN, so a scope
        // closed by the trigger or by going stale left the LAST weapon pointer behind. Switching
        // weapons while closed and then re-opening therefore compared new-against-old on the very
        // first tick and closed the scope the player had just opened: "it turns on, then turns
        // itself off really quickly". Seeding here means the watch always starts from the weapon
        // actually in hand.
        s_seen_weapon     = fp_weapon_actor();
        s_seen_weapon_cls = (s_seen_weapon != nullptr) ? s_seen_weapon->get_class() : nullptr;
        s_scope_open_tick = tick;
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
