// XrLayerAbi -- THE CONTRACT BETWEEN halo_vr.dll AND XrApiLayer_HALOVR_reticule.dll.
//
// ============================================================================================
// WHAT THIS IS AND WHY IT IS SO SMALL
// ============================================================================================
// An OpenXR *implicit API layer* is loaded by the OpenXR loader into the SAME PROCESS as the game
// and therefore the same process as this plugin. Two DLLs, one address space. So there is no IPC
// here, no shared memory, no named pipe -- the plugin finds the layer with GetModuleHandleW +
// GetProcAddress and calls it directly. This header is the whole interface.
//
// The layer exists for one reason, and it is worth stating precisely because the temptation is to
// grow it: XrLayerAttach.hpp explains that UEVR statically links its OpenXR loader, so the plugin
// cannot reach the entry points UEVR actually calls without UEVRBackend.pdb -- which only exists in
// a UEVR checkout. A layer is handed those entry points by the loader through the normal chain, for
// free, on any machine. The layer's job is to PASS THEM ACROSS and to own the one call the plugin
// cannot otherwise intercept, xrEndFrame. Everything else -- the swapchain, the atlas blit, the
// pose maths, the capture, the fail-open contract -- stays in XrLayer.cpp where it already works.
//
// ============================================================================================
// VERSIONING, AND WHY IT IS EXPLICIT RATHER THAN IMPLIED
// ============================================================================================
// These two DLLs ship in the same zip, but they will not always be the same age on a player's disk:
// the layer is registered ONCE (a registry value pointing at a path) while halo_vr.dll is replaced
// by every update. A player who updates the mod but whose registry still points at an older
// unpacked copy has exactly the skew this guards against.
//
// The project has already been burned by a contract that could not detect its own skew. From
// CLAUDE.md, about LuaVR: it "does no version negotiation -- it grabs the backend's
// g_plugin_initialize_param and dereferences callback slots blind, so a newer LuaVR against an
// older backend is a garbage-pointer call with no dump", and its version constant "is static and is
// NOT bumped when the struct grows, so it cannot detect skew". Both halves of that failure are
// designed out here:
//
//   * halovr_layer_get_api() TAKES the caller's ABI version and REFUSES (returns null) on anything
//     it does not implement. A mismatch is a clean null and a log line, never a call.
//   * HaloVrLayerApi::struct_size is the FIRST field and is filled with sizeof() by the producer.
//     A consumer must check it before touching any field beyond the first two. That is what lets
//     the struct grow without the version bump becoming a hard break.
//
// RULE FOR ANYONE EDITING THIS FILE: append fields at the END, never reorder or resize existing
// ones, and bump HALOVR_LAYER_ABI_VERSION for any change that is not a pure append.

#ifndef HALOVR_XRLAYERABI_H
#define HALOVR_XRLAYERABI_H

#include <stdint.h>

// The OpenXR types below (XrSession, XrFrameEndInfo, ...) must be THE SAME LAYOUTS on both sides.
// Both sides include the one vendored copy in src/thirdparty/openxr (1.0.22, taken from UEVR's own
// dependency tree so it matches the loader UEVR calls). Do not let either side drift onto a
// different SDK: a struct-layout disagreement here is not a compile error, it is a wrong pointer.
#ifndef XR_USE_PLATFORM_WIN32
#define XR_USE_PLATFORM_WIN32
#endif
#ifndef XR_USE_GRAPHICS_API_D3D12
#define XR_USE_GRAPHICS_API_D3D12
#endif
#include "thirdparty/openxr/openxr.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---- identity -------------------------------------------------------------------------------
//
// The layer NAME is what appears in the JSON manifest and in XR_ENABLE_API_LAYERS. The MODULE name
// is what the plugin passes to GetModuleHandleW. Keep all three in step: the manifest, this header,
// and the build script's output filename.
#define HALOVR_LAYER_NAME        "XR_APILAYER_HALOVR_reticule"
#define HALOVR_LAYER_MODULE_W    L"XrApiLayer_HALOVR_reticule.dll"
#define HALOVR_LAYER_GET_API_SYM "halovr_layer_get_api"

// Bump on any change that is not a pure append to HaloVrLayerApi.
#define HALOVR_LAYER_ABI_VERSION 1u

// How many composition layers of our own the layer will accept in one frame. Nine is what
// XrLayer.hpp's XRLAYER_SLOTS asks for (one reticule + eight navpoints); the margin is so a future
// slot count does not silently truncate at the boundary. The layer refuses a batch larger than this
// rather than writing past its array -- see HaloVrLayerEndFrameFn.
#define HALOVR_LAYER_MAX_EXTRA_LAYERS 16u

// ---- the per-frame hand-off -------------------------------------------------------------------
//
// CALLED ON THE APPLICATION'S SUBMIT THREAD, from inside the layer's xrEndFrame, with the frame the
// application (UEVR) is about to present. This is the same thread and the same moment as the inline
// hook in XrLayer.cpp used, so the code that fills it in needs no rethinking -- only the way it is
// reached has changed.
//
// CONTRACT, and every clause of it is load-bearing:
//
//   * `frame_end_info` is READ-ONLY and belongs to the application. Do not modify it. The layer
//     copies its layer array and appends yours; you never see the copy.
//   * Write at most `out_capacity` pointers into `out_layers` and return how many you wrote.
//     Returning more than `out_capacity` is treated as a fault: the layer discards the batch and
//     presents the application's frame untouched. It does not clamp, because a caller that
//     miscounted has probably also written past the end of something else.
//   * THE POINTERS YOU WRITE MUST REMAIN VALID UNTIL YOU RETURN FROM THE NEXT CALL. The layer
//     forwards them to the runtime inside this same xrEndFrame and does not retain them, so
//     function-static or module-static storage is correct and a stack local is not.
//   * Return 0 for "nothing to add". That is the normal state whenever the feature is off, and it
//     must be cheap: this runs on every presented frame.
//   * DO NOT BLOCK, and do not call back into the layer. You are inside the loader's dispatch of
//     xrEndFrame; a reentrant OpenXR call from here is undefined.
//
// FAIL-OPEN IS THE LAYER'S JOB, NOT YOURS. If the runtime rejects the frame with your layers in it,
// the layer retries with the application's original list, so the worst case is a missing overlay
// rather than a dropped frame. XrLayer.hpp already treats that as the invariant; it survives.
typedef uint32_t (XRAPI_PTR *HaloVrLayerEndFrameFn)(
    XrSession                             session,
    const XrFrameEndInfo*                 frame_end_info,
    const XrCompositionLayerBaseHeader**  out_layers,
    uint32_t                              out_capacity,
    void*                                 user);

// ---- the API the layer exports ------------------------------------------------------------------
//
// Obtained by GetProcAddress(HALOVR_LAYER_GET_API_SYM) and one call. Everything is null-safe to
// call at any time and from any thread unless a field says otherwise; before an OpenXR instance
// exists the getters simply answer null/zero.
typedef struct HaloVrLayerApi {
    // MUST BE FIRST TWO FIELDS, MUST NEVER MOVE. A consumer reads these before anything else and
    // refuses on disagreement -- that is the only way a struct that grows stays safe.
    uint32_t struct_size;      // sizeof(HaloVrLayerApi) as the LAYER compiled it
    uint32_t abi_version;      // HALOVR_LAYER_ABI_VERSION as the LAYER compiled it

    // Human-readable build stamp, e.g. "XR_APILAYER_HALOVR_reticule 1 (Aug 23 2026 21:04:11)".
    // Logged once by the plugin so a support report says WHICH layer build is on disk. A registered
    // implicit layer is a file a player installed months ago; without this there is no way to ask.
    const char* build_stamp;

    // The OpenXR instance and session the layer is sitting in front of, or XR_NULL_HANDLE before
    // they exist. THESE ARE THE AUTHORITATIVE HANDLES for the chain the plugin must call into --
    // they come from the loader dispatch UEVR itself is using, which is the exact thing the PDB
    // attachment had to go digging for.
    XrInstance (XRAPI_PTR *get_instance)(void);
    XrSession  (XRAPI_PTR *get_session)(void);

    // Resolve an OpenXR entry point on the SAME CHAIN, BELOW THIS LAYER. This is the whole reason
    // the layer is worth building: xrCreateSwapchain reached this way belongs to the same loader
    // instance as the session above, so handing it that session is correct by construction.
    //
    // XrLayerAttach.hpp records what happens otherwise -- entry points taken from
    // openxr_loader.dll while the session belongs to UEVR's statically-linked loader is "an
    // unrelated pointer being dereferenced as a dispatch table".
    //
    // Returns null before the instance exists, or for a name the runtime does not implement.
    PFN_xrVoidFunction (XRAPI_PTR *get_proc)(const char* name);

    // Register (or clear, with fn == NULL) the per-frame callback above. Returns 1 on success.
    //
    // Clearing BLOCKS until any in-flight call has returned, so it is safe to unload state the
    // callback reads immediately afterwards. Setting is atomic. Only one callback at a time -- a
    // second registration replaces the first.
    int (XRAPI_PTR *set_end_frame_callback)(HaloVrLayerEndFrameFn fn, void* user);

    // ---- the watchdog counters ----------------------------------------------------------------
    //
    // "INSTALLED IS NOT RUNNING" is written all over this project, and a layer can fail in exactly
    // that shape: the registry value is present, the DLL loads, get_api succeeds, the callback is
    // registered -- and the application never reaches this layer's xrEndFrame because it is on a
    // different runtime, or the session ended, or another layer swallowed the call.
    //
    // frames_seen() counts calls into the layer's xrEndFrame. layers_appended() counts frames where
    // ours actually went to the runtime. A caller that wants to claim the compositor overlay is
    // live must gate on these MOVING, never on registration having returned success.
    uint64_t (XRAPI_PTR *frames_seen)(void);
    uint64_t (XRAPI_PTR *layers_appended)(void);

    // One line for the log: gate decision, instance/session state, counters. Never null.
    const char* (XRAPI_PTR *status)(void);

    // APPEND NEW FIELDS BELOW THIS LINE ONLY.
} HaloVrLayerApi;

// The single export. Returns null -- deliberately, loudly, and without touching anything -- when
// `abi_version` is not the one this layer implements, or when the layer gated itself off because
// the host process is not the game.
//
// A null here is not an error to work around. It means the compositor path is unavailable in this
// process and the caller must fall back to what it did before.
typedef const HaloVrLayerApi* (XRAPI_PTR *PFN_halovr_layer_get_api)(uint32_t abi_version);

#ifdef __cplusplus
}   // extern "C"
#endif

#endif  // HALOVR_XRLAYERABI_H
