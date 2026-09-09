// XrApiLayer_HALOVR_reticule -- the SHIPPING attachment for the compositor reticule.
//
// ============================================================================================
// WHY A SEPARATE DLL EXISTS AT ALL
// ============================================================================================
// src/XrLayer.cpp draws the aim reticule as an OpenXR quad composition layer. A composition layer
// is submitted to the runtime AFTER the whole post chain, so scene exposure, eye adaptation and the
// tonemapper never touch it -- it is immune by construction rather than compensating with a
// constant multiplier that no single value wins. That part works and is verified in a headset.
//
// The problem was never the drawing. It was getting called.
//
// UEVR is the OpenXR *application*; the plugin is a guest inside it, and UEVR's plugin API offers no
// way to add a layer to the list UEVR passes to xrEndFrame. So the plugin has to intercept that
// call -- and UEVR STATICALLY LINKS the OpenXR loader into UEVRBackend.dll, so there is no exported
// xrEndFrame to hook. `dumpbin /imports UEVRBackend.dll` shows no openxr_loader.dll import at all.
// The working dev attachment resolves the address by symbol out of UEVRBackend.pdb (see
// src/XrLayerAttach.hpp), which is exact and correct and CANNOT SHIP: only a UEVR checkout has that
// PDB. On a player install the resolve fails, the module latches off, and the feature -- which is
// enabled by default -- does nothing at all, silently.
//
// AN API LAYER NEEDS NO HOOK AND NO SYMBOLS. The loader hands a layer xrEndFrame through the normal
// dispatch chain, and it does so EVEN THOUGH UEVR statically links the loader, because the loader
// still walks the API-layer registry at instance creation. That is not a hopeful reading of the
// specification: on the development machine, Virtual Desktop's `openxr-oculus-compatibility.json`
// is registered as an implicit layer and its DLL is observed loaded inside the Halo process
// alongside UEVR. A layer gets in where a hook cannot.
//
// This file is that layer. It is deliberately thin. Everything below the attachment point --
// swapchain creation, the atlas blit, the pose maths, the owned-texture capture, the fail-open
// contract -- lives in XrLayer.cpp and is attachment-agnostic by design. The layer owns exactly two
// things the plugin cannot get on its own:
//
//   1. xrEndFrame, which it calls the plugin back from so the plugin can append its quads.
//   2. A down-chain xrGetInstanceProcAddr, so the plugin's xrCreateSwapchain and friends come from
//      THE SAME loader instance as the session UEVR handed it. XrLayerAttach.hpp is emphatic about
//      why that matters: entry points from one loader driving another loader's session is "an
//      unrelated pointer being dereferenced as a dispatch table", and it fails as memory corruption
//      rather than as an error code.
//
// ============================================================================================
// THIS DLL LOADS INTO EVERY OPENXR APPLICATION ON THE MACHINE. READ THIS BEFORE EDITING.
// ============================================================================================
// An IMPLICIT layer is registered once, per user, and from then on the OpenXR loader loads it into
// every OpenXR application that user runs -- their other VR games, SteamVR Home, the headset's own
// utilities. That is the price of not needing per-process environment variables, and it is the
// entire reason this file is written the way it is.
//
// So the FIRST thing this layer does is decide whether it is in the game, and if it is not, it
// becomes a pass-through with no interception of any kind. Not "does less work" -- does NOTHING:
// xrGetInstanceProcAddr forwards straight to the next layer, no function is wrapped, no callback
// runs, nothing is allocated. One `if` at instance creation and the layer is invisible for the rest
// of that process's life.
//
// WHY IT PASSES THROUGH RATHER THAN DECLINING TO NEGOTIATE. Returning an error from
// xrNegotiateLoaderApiLayerInterface is the tidier-looking way to opt out, and the loader is
// supposed to skip a layer that does it. "Supposed to" is doing a lot of work in that sentence, and
// the blast radius of being wrong is every OpenXR application the user owns failing to start
// because of a mod for a different game. A pass-through cannot break an application even if the
// loader's error handling is not what we believe. We take the loaded-but-inert cost -- a few tens of
// kilobytes of address space -- and keep the failure mode impossible instead of unlikely.
//
// The corollary: NOTHING IN THIS FILE MAY DO WORK BEFORE THE GATE. No file I/O in DllMain, no
// registry reads, no threads, no allocations on a static initialiser. If you add something and are
// unsure whether it runs in a non-Halo process, it does.

#include <windows.h>

#include "../../src/XrLayerAbi.h"
#include "thirdparty/openxr/loader_interfaces.h"

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cwchar>

namespace {

// ============================================================================================
// State
// ============================================================================================
//
// All of it is process-global because the loader gives us no per-layer context to hang it on, and
// all of it is atomic because the callback runs on the application's submit thread while the
// plugin registers from the game thread.

HMODULE g_self = nullptr;

// The gate decision, computed once. -1 = not yet decided, 0 = inert, 1 = active.
std::atomic<int> g_enabled{-1};
char             g_gate_reason[256] = "not evaluated";

// The chain below us. Written once at instance creation, read on the submit thread.
std::atomic<PFN_xrGetInstanceProcAddr> g_next_gipa{nullptr};
std::atomic<uint64_t>                  g_instance{0};
std::atomic<uint64_t>                  g_session{0};
std::atomic<PFN_xrEndFrame>            g_next_end_frame{nullptr};
std::atomic<PFN_xrDestroyInstance>     g_next_destroy_instance{nullptr};
std::atomic<PFN_xrCreateSession>       g_next_create_session{nullptr};
std::atomic<PFN_xrDestroySession>      g_next_destroy_session{nullptr};

// The plugin's per-frame callback, plus the in-flight count that makes clearing it safe. See
// set_end_frame_callback() for the ordering argument -- it is the interesting part.
std::atomic<HaloVrLayerEndFrameFn> g_cb{nullptr};
std::atomic<void*>                 g_cb_user{nullptr};
std::atomic<uint32_t>              g_cb_inflight{0};

// Watchdog counters. "Installed is not running" -- these are how the plugin proves the difference.
std::atomic<uint64_t> g_frames_seen{0};
std::atomic<uint64_t> g_layers_appended{0};
// MONO PROJECTION (see XrLayerAbi.h, set_projection_mono). Set by the plugin on the game thread,
// read here on the app's render/submit thread; relaxed is enough because a frame late is invisible
// and there is nothing to order against. g_mono_patched counts projection layers actually rewritten
// -- the number a caller must see MOVING before claiming the mono path is live.
std::atomic<int>      g_projection_mono{0};
std::atomic<uint64_t> g_mono_patched{0};
std::atomic<uint64_t> g_batch_refused{0};   // callback returned more than it was offered
std::atomic<uint64_t> g_runtime_rejects{0}; // runtime refused the frame WITH our layers in it
std::atomic<uint64_t> g_passthrough_rejects{0}; // refused for a reason that is NOT ours; returned as-is

char g_build_stamp[160] = HALOVR_LAYER_NAME " (unstamped)";
char g_status_line[512] = HALOVR_LAYER_NAME ": not initialised";

// ============================================================================================
// Logging
// ============================================================================================
//
// OutputDebugStringA always (free, captured by DebugView and by any attached debugger), plus a
// best-effort file beside this DLL so a player's support report has something in it. BOTH ARE
// BEHIND THE GATE except for the one line that records the gate decision itself, and neither is
// ever called per frame -- a layer that writes a line every frame would be a stutter source in a
// title where a hitch is nausea.

void logf(const char* fmt, ...) {
    char msg[512];
    va_list ap;
    va_start(ap, fmt);
    _vsnprintf_s(msg, sizeof(msg), _TRUNCATE, fmt, ap);
    va_end(ap);

    char line[600];
    _snprintf_s(line, sizeof(line), _TRUNCATE, "[HALOVR-XRLAYER] %s\n", msg);
    OutputDebugStringA(line);

    // Beside the DLL, which for a shipped install is inside the UEVR profile folder -- the same
    // place the player already knows to look. Opened and closed per line on purpose: this is called
    // a handful of times per session, and a held handle would keep the profile folder locked.
    wchar_t path[MAX_PATH];
    if (g_self != nullptr && GetModuleFileNameW(g_self, path, MAX_PATH) != 0) {
        wchar_t* slash = wcsrchr(path, L'\\');
        if (slash != nullptr) {
            slash[1] = L'\0';
            wcsncat_s(path, MAX_PATH, L"halo_vr_layer.log", _TRUNCATE);
            FILE* f = nullptr;
            if (_wfopen_s(&f, path, L"a") == 0 && f != nullptr) {
                fputs(line, f);
                fclose(f);
            }
        }
    }
}

// ============================================================================================
// The gate
// ============================================================================================

bool env_is_set(const wchar_t* name) {
    wchar_t buf[64];
    const DWORD n = GetEnvironmentVariableW(name, buf, 64);
    if (n == 0 || n >= 64) return n >= 64;          // present but long counts as set
    return !(buf[0] == L'0' && buf[1] == L'\0');    // "0" means explicitly not set
}

// The executables this layer is FOR. Everything else gets a pass-through.
//
// Two names because the retail build and the internal/branch build differ: CLAUDE.md records the
// retail exe as HaloCampaignEvolved.exe under Meteorite\Binaries\Win64\, and internal branches
// shipping as Meteorite-Win64-Shipping.exe. Matching on the BASENAME rather than the full path is
// deliberate -- a player's Steam library can be on any drive, and a path match would fail for the
// people most likely to have installed it somewhere unusual.
//
// This is a permissive gate by nature: another program could be called HaloCampaignEvolved.exe. The
// consequence of a false positive is bounded and benign -- the layer would wrap xrEndFrame and add
// nothing, because no callback is ever registered without halo_vr.dll being in the process too.
const wchar_t* const kGameExeNames[] = {
    L"HaloCampaignEvolved.exe",
    L"Meteorite-Win64-Shipping.exe",
};

bool evaluate_gate() {
    // The manifest's disable_environment. The loader honours this for an implicit layer and will
    // not load us at all -- but the same variable must also work when the layer is activated the
    // developer way, through XR_ENABLE_API_LAYERS, where the loader does not check it. One kill
    // switch that works on both routes is worth four lines.
    if (env_is_set(L"HALOVR_LAYER_DISABLE")) {
        strcpy_s(g_gate_reason, "disabled by HALOVR_LAYER_DISABLE");
        return false;
    }

    // The developer/test escape hatch. The self-test host is not called HaloCampaignEvolved.exe and
    // must still be able to exercise the whole chain; so must anyone debugging the layer in a
    // sample application. It is opt-in per process, so it cannot leak into a player's machine.
    if (env_is_set(L"HALOVR_LAYER_FORCE")) {
        strcpy_s(g_gate_reason, "forced on by HALOVR_LAYER_FORCE");
        return true;
    }

    wchar_t path[MAX_PATH];
    if (GetModuleFileNameW(nullptr, path, MAX_PATH) == 0) {
        strcpy_s(g_gate_reason, "inert: could not read the host executable name");
        return false;
    }
    const wchar_t* slash = wcsrchr(path, L'\\');
    const wchar_t* base  = (slash != nullptr) ? slash + 1 : path;

    for (const wchar_t* name : kGameExeNames) {
        if (_wcsicmp(base, name) == 0) {
            _snprintf_s(g_gate_reason, sizeof(g_gate_reason), _TRUNCATE,
                        "active: host is %ls", base);
            return true;
        }
    }
    _snprintf_s(g_gate_reason, sizeof(g_gate_reason), _TRUNCATE,
                "inert: host is %ls, not this game", base);
    return false;
}

bool gate_ok() {
    int e = g_enabled.load(std::memory_order_acquire);
    if (e >= 0) return e == 1;
    // Benign race: two threads may evaluate concurrently and reach the same answer. Cheaper and far
    // less dangerous than any lock taken this early in a foreign process.
    e = evaluate_gate() ? 1 : 0;
    g_enabled.store(e, std::memory_order_release);
    return e == 1;
}

// ============================================================================================
// Wrapped entry points
// ============================================================================================

XRAPI_ATTR XrResult XRAPI_CALL layer_xrEndFrame(XrSession session, const XrFrameEndInfo* info) {
    PFN_xrEndFrame next = g_next_end_frame.load(std::memory_order_acquire);
    if (next == nullptr) return XR_ERROR_FUNCTION_UNSUPPORTED;   // cannot happen; not a crash if it does

    g_frames_seen.fetch_add(1, std::memory_order_relaxed);

    // INCREMENT BEFORE LOADING THE CALLBACK, not after. The clearing side stores null and then spins
    // until this counter reaches zero; if we loaded first and incremented second, a clear could slip
    // into that gap, observe zero in flight, return to the plugin, and let the plugin free state we
    // are about to call into. Incrementing first makes the clear wait for us. The cost is one
    // relaxed add on a frame that is about to talk to a compositor.
    g_cb_inflight.fetch_add(1, std::memory_order_acquire);
    const HaloVrLayerEndFrameFn cb   = g_cb.load(std::memory_order_acquire);
    void* const                 user = g_cb_user.load(std::memory_order_acquire);

    XrResult result = XR_SUCCESS;
    bool     handled = false;

    // MONO is a reason to rebuild the layer list on its own, with or without a plugin callback and
    // with or without extra quads -- so it is read here, ahead of the callback, and folded into the
    // same "do we patch this frame" decision the quads use. One rebuild path, not two.
    const int mono = g_projection_mono.load(std::memory_order_relaxed);

    if (info != nullptr && (cb != nullptr || mono != 0)) {
        const XrCompositionLayerBaseHeader* extra[HALOVR_LAYER_MAX_EXTRA_LAYERS];
        const uint32_t n = (cb != nullptr) ? cb(session, info, extra, HALOVR_LAYER_MAX_EXTRA_LAYERS, user) : 0u;

        if (n > HALOVR_LAYER_MAX_EXTRA_LAYERS) {
            // A caller that returned more than it was offered has miscounted, and the array it wrote
            // into is ours. Discard the whole batch rather than trusting any of it: presenting the
            // application's own frame is always a safe answer, and clamping would hide the bug.
            g_batch_refused.fetch_add(1, std::memory_order_relaxed);
        } else if (n > 0 || mono != 0) {
            // One combined array. Sized for the runtime's realistic ceiling plus our own; anything
            // beyond it falls through to the untouched frame rather than truncating the
            // APPLICATION's layers, which would be a visible regression for the player.
            constexpr uint32_t kMaxTotal = 64;
            // Subtraction, not addition: `layerCount + n` is unsigned and would WRAP on a bogus
            // count, turning an overflow check into a green light for a buffer overrun.
            if (n <= kMaxTotal && info->layerCount <= kMaxTotal - n && info->layers != nullptr) {
                const XrCompositionLayerBaseHeader* combined[kMaxTotal];
                uint32_t w = 0;

                // ---- MONO PROJECTION: clone-and-patch, never edit the app's memory --------------
                //
                // The application's layer structs are const and it may reuse them next frame, so
                // the rewrite happens on a copy that lives on THIS stack frame -- valid for exactly
                // as long as next() needs it (xrEndFrame is synchronous; the runtime has consumed
                // the structs by the time it returns). The copy is shallow on purpose: `next`
                // chains and the subImage swapchain handle are borrowed, not owned.
                //
                // The patch itself is one field per extra view: every view after the first is
                // given view[0]'s subImage (swapchain + array index + imageRect), so the runtime
                // composites the LEFT eye's pixels for both eyes. Pose and fov are left as the app
                // set them -- each eye is still reprojected from its own position, which is what
                // keeps head movement correct; only the CONTENT is shared.
                //
                // Bounded scratch: at most kMonoMaxLayers projection layers with kMonoMaxViews
                // views each. Anything beyond that is passed through UNPATCHED rather than refused
                // -- a cutscene with a stray extra layer must never lose the frame, only the fix.
                constexpr uint32_t kMonoMaxLayers = 4;
                constexpr uint32_t kMonoMaxViews  = 4;
                XrCompositionLayerProjection     mono_layer[kMonoMaxLayers];
                XrCompositionLayerProjectionView mono_view[kMonoMaxLayers][kMonoMaxViews];
                uint32_t mono_used = 0;

                for (uint32_t i = 0; i < info->layerCount; ++i) {
                    const XrCompositionLayerBaseHeader* base = info->layers[i];

                    // MODES 3 AND 4 REMOVE A CLASS OF APP LAYER instead of rewriting one. They
                    // exist because mode 1 rewrote every projection frame on 2026-09-08 and the
                    // doubled cutscene did not change, which means the doubling is not a
                    // left/right mismatch inside the projection. The profile has UEVR presenting
                    // the game's Slate UI as its own quad (UI_OverlayType=0, UI_Size=2.0,
                    // UI_Distance=2.43) and the cutscene movie is drawn by Slate -- so the
                    // likeliest picture is the movie shown TWICE, once in the projection images
                    // and once on that quad, at different depths. Dropping one class at a time,
                    // live, says which copy is which. Mode 4 is also the candidate FIX: the
                    // movie alone, on a flat mono screen, with nothing doubled behind it.
                    // Our OWN appended quads are added after this loop and are unaffected.
                    if (base != nullptr && mono == 3 && base->type != XR_TYPE_COMPOSITION_LAYER_PROJECTION) {
                        static bool s_said3 = false;
                        if (!s_said3) { s_said3 = true; logf("mono mode 3: dropping app layer type=%d (first of them)", (int)base->type); }
                        g_mono_patched.fetch_add(1, std::memory_order_relaxed);
                        continue;
                    }
                    if (base != nullptr && mono == 4 && base->type == XR_TYPE_COMPOSITION_LAYER_PROJECTION) {
                        static bool s_said4 = false;
                        if (!s_said4) { s_said4 = true; logf("mono mode 4: dropping the projection layer (first of them); quads carry the frame"); }
                        g_mono_patched.fetch_add(1, std::memory_order_relaxed);
                        continue;
                    }

                    if ((mono == 1 || mono == 2) && base != nullptr
                        && base->type == XR_TYPE_COMPOSITION_LAYER_PROJECTION
                        && mono_used < kMonoMaxLayers) {
                        const auto* src = reinterpret_cast<const XrCompositionLayerProjection*>(base);
                        if (src->views != nullptr && src->viewCount >= 2 && src->viewCount <= kMonoMaxViews) {
                            // mode 1 = view[0] (left) to every eye; mode 2 = view[1] (right) to
                            // every eye. Two modes so a player can flip LIVE mid-cutscene: if the
                            // picture shifts, the movie is in the projection images; if it does
                            // not, it is somewhere the projection rewrite cannot reach.
                            const uint32_t sv = (mono == 2) ? 1u : 0u;
                            XrCompositionLayerProjection& dst = mono_layer[mono_used];
                            dst = *src;                                        // shallow clone
                            for (uint32_t v = 0; v < src->viewCount; ++v) {
                                mono_view[mono_used][v] = src->views[v];       // clone each view
                                if (v != sv) mono_view[mono_used][v].subImage = src->views[sv].subImage;
                            }
                            dst.views = mono_view[mono_used];
                            combined[w++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&dst);
                            ++mono_used;
                            g_mono_patched.fetch_add(1, std::memory_order_relaxed);
                            // ONCE: the positive proof that a projection layer was actually
                            // rewritten, with the shape we saw. Without this line a run where
                            // the switch applied but no layer ever matched (wrong type, viewCount
                            // outside 2..4, null views) is indistinguishable from one that
                            // worked -- which is exactly the run of 2026-09-08 14:38.
                            static bool s_said_first = false;
                            if (!s_said_first) {
                                s_said_first = true;
                                logf("mono: first projection layer patched -- views=%u, "
                                     "imageArrayIndex[0]=%u, rect[0]=%dx%d@%d,%d; every view "
                                     "now shows view[0]'s sub-image (poses/FOVs untouched)",
                                     src->viewCount,
                                     src->views[0].subImage.imageArrayIndex,
                                     src->views[0].subImage.imageRect.extent.width,
                                     src->views[0].subImage.imageRect.extent.height,
                                     src->views[0].subImage.imageRect.offset.x,
                                     src->views[0].subImage.imageRect.offset.y);
                                // INVENTORY OF EVERYTHING THE APP SUBMITTED THIS FRAME. The
                                // 2026-09-08 20:27 run rewrote every projection frame and the
                                // doubled cutscene did not change -- so the doubling is not a
                                // left/right mismatch inside the projection. Either it rides a
                                // DIFFERENT layer (UEVR submits Slate UI as its own quad, and the
                                // movie is drawn by Slate) or it is two copies inside one eye
                                // image. This list is what separates those: a second layer here,
                                // or not.
                                logf("mono: layer inventory this frame -- %u layer(s):", info->layerCount);
                                for (uint32_t k = 0; k < info->layerCount && k < 16; ++k) {
                                    const XrCompositionLayerBaseHeader* h = info->layers[k];
                                    if (h == nullptr) { logf("  [%u] <null>", k); continue; }
                                    if (h->type == XR_TYPE_COMPOSITION_LAYER_PROJECTION) {
                                        const auto* pl = reinterpret_cast<const XrCompositionLayerProjection*>(h);
                                        logf("  [%u] PROJECTION views=%u flags=0x%llx", k, pl->viewCount,
                                             (unsigned long long)pl->layerFlags);
                                    } else if (h->type == XR_TYPE_COMPOSITION_LAYER_QUAD) {
                                        const auto* q = reinterpret_cast<const XrCompositionLayerQuad*>(h);
                                        logf("  [%u] QUAD eyeVisibility=%d size=%.3fx%.3f rect=%dx%d@%d,%d "
                                             "swapchain=%p flags=0x%llx", k, (int)q->eyeVisibility,
                                             q->size.width, q->size.height,
                                             q->subImage.imageRect.extent.width, q->subImage.imageRect.extent.height,
                                             q->subImage.imageRect.offset.x, q->subImage.imageRect.offset.y,
                                             (void*)q->subImage.swapchain, (unsigned long long)q->layerFlags);
                                    } else {
                                        logf("  [%u] type=%d (not projection/quad)", k, (int)h->type);
                                    }
                                }
                            }
                            continue;
                        }
                    }
                    combined[w++] = base;
                }
                // OURS LAST, therefore topmost. The reticule must draw over the scene projection,
                // never under it.
                for (uint32_t i = 0; i < n; ++i) combined[w++] = extra[i];

                XrFrameEndInfo patched = *info;
                patched.layerCount = w;
                patched.layers     = combined;

                result  = next(session, &patched);
                handled = true;

                if (XR_SUCCEEDED(result)) {
                    // Only a frame that actually carried our quads counts as "appended"; a
                    // mono-only rebuild is tallied by g_mono_patched instead.
                    if (n > 0) g_layers_appended.fetch_add(1, std::memory_order_relaxed);
                } else if (result == XR_ERROR_LAYER_INVALID
                        || result == XR_ERROR_LAYER_LIMIT_EXCEEDED
                        || result == XR_ERROR_SWAPCHAIN_RECT_INVALID) {
                    // FAIL OPEN, BUT ONLY FOR FAILURES THAT ARE OURS. Retry with the application's
                    // original list rather than dropping the player's frame: a missing overlay is
                    // cosmetic, a dropped frame in VR is nausea.
                    g_runtime_rejects.fetch_add(1, std::memory_order_relaxed);
                    handled = false;
                } else {
                    // NOT OURS -- return it to the application untouched. The plugin-side hook had
                    // exactly this bug and it was measured on 2026-09-04: retrying a frame the
                    // runtime rejected for a STALE DISPLAY TIME (-30 TIME_INVALID) called xrEndFrame
                    // a second time on a finished frame (-37 CALL_ORDER_INVALID) and discarded the
                    // next one. Resubmitting cannot fix a time, a call order, or a lost session.
                    g_passthrough_rejects.fetch_add(1, std::memory_order_relaxed);
                    handled = true;
                }
            }
        }
    }

    g_cb_inflight.fetch_sub(1, std::memory_order_release);

    if (handled) return result;
    return next(session, info);
}

XRAPI_ATTR XrResult XRAPI_CALL layer_xrCreateSession(XrInstance instance,
                                                     const XrSessionCreateInfo* create_info,
                                                     XrSession* session) {
    PFN_xrCreateSession next = g_next_create_session.load(std::memory_order_acquire);
    if (next == nullptr) return XR_ERROR_FUNCTION_UNSUPPORTED;

    const XrResult r = next(instance, create_info, session);
    if (XR_SUCCEEDED(r) && session != nullptr) {
        g_session.store((uint64_t)*session, std::memory_order_release);
        logf("session created (%p)", (void*)*session);
    }
    return r;
}

XRAPI_ATTR XrResult XRAPI_CALL layer_xrDestroySession(XrSession session) {
    PFN_xrDestroySession next = g_next_destroy_session.load(std::memory_order_acquire);
    if (next == nullptr) return XR_ERROR_FUNCTION_UNSUPPORTED;

    // FORGET IT BEFORE DESTROYING IT, not after. get_session() is read by the plugin from another
    // thread; if we cleared afterwards there would be a window in which the plugin could hand a
    // destroyed session to xrCreateSwapchain.
    if (g_session.load(std::memory_order_acquire) == (uint64_t)session) {
        g_session.store(0, std::memory_order_release);
    }
    logf("session destroyed (%p)", (void*)session);
    return next(session);
}

XRAPI_ATTR XrResult XRAPI_CALL layer_xrDestroyInstance(XrInstance instance) {
    PFN_xrDestroyInstance next = g_next_destroy_instance.load(std::memory_order_acquire);
    if (next == nullptr) return XR_ERROR_FUNCTION_UNSUPPORTED;

    // The plugin may still be holding our API pointer. Drop the callback FIRST and wait for any
    // in-flight call, then forget the chain -- after this returns, every function pointer we hold
    // belongs to a destroyed instance and calling one is undefined.
    g_cb.store(nullptr, std::memory_order_release);
    while (g_cb_inflight.load(std::memory_order_acquire) != 0) { Sleep(0); }

    if (g_instance.load(std::memory_order_acquire) == (uint64_t)instance) {
        g_instance.store(0, std::memory_order_release);
        g_session.store(0, std::memory_order_release);
        g_next_end_frame.store(nullptr, std::memory_order_release);
        g_next_create_session.store(nullptr, std::memory_order_release);
        g_next_destroy_session.store(nullptr, std::memory_order_release);
        g_next_gipa.store(nullptr, std::memory_order_release);
    }
    logf("instance destroyed (%p) -- frames_seen=%llu layers_appended=%llu mono_patched=%llu",
         (void*)instance,
         (unsigned long long)g_frames_seen.load(),
         (unsigned long long)g_layers_appended.load(),
         (unsigned long long)g_mono_patched.load(std::memory_order_relaxed));
    return next(instance);
}

// ============================================================================================
// The loader-facing dispatch
// ============================================================================================

XRAPI_ATTR XrResult XRAPI_CALL layer_xrGetInstanceProcAddr(XrInstance instance, const char* name,
                                                           PFN_xrVoidFunction* function) {
    PFN_xrGetInstanceProcAddr next = g_next_gipa.load(std::memory_order_acquire);
    if (next == nullptr) return XR_ERROR_FUNCTION_UNSUPPORTED;

    // THE INERT PATH. In any process that is not the game this is the only line that ever runs, and
    // it makes the layer indistinguishable from not being installed.
    if (!gate_ok()) return next(instance, name, function);

    if (name == nullptr || function == nullptr) return next(instance, name, function);

    // Wrap only what we need. Everything else -- hundreds of functions, including every one the
    // plugin later resolves through get_proc -- goes straight down the chain untouched.
    if (strcmp(name, "xrEndFrame") == 0) {
        *function = (PFN_xrVoidFunction)layer_xrEndFrame;
        return XR_SUCCESS;
    }
    if (strcmp(name, "xrCreateSession") == 0) {
        *function = (PFN_xrVoidFunction)layer_xrCreateSession;
        return XR_SUCCESS;
    }
    if (strcmp(name, "xrDestroySession") == 0) {
        *function = (PFN_xrVoidFunction)layer_xrDestroySession;
        return XR_SUCCESS;
    }
    if (strcmp(name, "xrDestroyInstance") == 0) {
        *function = (PFN_xrVoidFunction)layer_xrDestroyInstance;
        return XR_SUCCESS;
    }
    if (strcmp(name, "xrGetInstanceProcAddr") == 0) {
        *function = (PFN_xrVoidFunction)layer_xrGetInstanceProcAddr;
        return XR_SUCCESS;
    }
    return next(instance, name, function);
}

XRAPI_ATTR XrResult XRAPI_CALL layer_xrCreateApiLayerInstance(const XrInstanceCreateInfo* info,
                                                              const XrApiLayerCreateInfo* layer_info,
                                                              XrInstance* instance) {
    // Validate what the loader handed us before dereferencing any of it. A layer that trusts this
    // struct blindly is a layer that crashes every OpenXR application on the machine the day a
    // loader ships a different struct version.
    if (info == nullptr || instance == nullptr ||
        layer_info == nullptr ||
        layer_info->structType != XR_LOADER_INTERFACE_STRUCT_API_LAYER_CREATE_INFO ||
        layer_info->structVersion != XR_API_LAYER_CREATE_INFO_STRUCT_VERSION ||
        layer_info->structSize != sizeof(XrApiLayerCreateInfo) ||
        layer_info->nextInfo == nullptr ||
        layer_info->nextInfo->structType != XR_LOADER_INTERFACE_STRUCT_API_LAYER_NEXT_INFO ||
        layer_info->nextInfo->structVersion != XR_API_LAYER_NEXT_INFO_STRUCT_VERSION ||
        layer_info->nextInfo->structSize != sizeof(XrApiLayerNextInfo) ||
        layer_info->nextInfo->nextGetInstanceProcAddr == nullptr ||
        layer_info->nextInfo->nextCreateApiLayerInstance == nullptr) {
        return XR_ERROR_INITIALIZATION_FAILED;
    }

    // Advance the chain by one link and pass it on. This is the contract every layer implements:
    // the copy we forward names the NEXT layer's info, never our own.
    XrApiLayerCreateInfo forwarded = *layer_info;
    forwarded.nextInfo = layer_info->nextInfo->next;

    const XrResult r = layer_info->nextInfo->nextCreateApiLayerInstance(info, &forwarded, instance);
    if (XR_FAILED(r)) return r;

    PFN_xrGetInstanceProcAddr next = layer_info->nextInfo->nextGetInstanceProcAddr;
    g_next_gipa.store(next, std::memory_order_release);

    // THE GATE IS EVALUATED HERE, once per instance, and everything after it is skipped when we are
    // not in the game. Note what is NOT above this line: no logging, no file I/O, no resolution of
    // anything. In a foreign process this function does the chain hop and returns.
    if (!gate_ok()) {
        g_instance.store((uint64_t)*instance, std::memory_order_release);
        return r;
    }

    // We are in the game. Resolve the functions we wrap, from the chain BELOW us.
    auto resolve = [&](const char* n) -> PFN_xrVoidFunction {
        PFN_xrVoidFunction f = nullptr;
        if (XR_FAILED(next(*instance, n, &f))) return nullptr;
        return f;
    };
    g_next_end_frame.store       ((PFN_xrEndFrame)       resolve("xrEndFrame"),       std::memory_order_release);
    g_next_create_session.store  ((PFN_xrCreateSession)  resolve("xrCreateSession"),  std::memory_order_release);
    g_next_destroy_session.store ((PFN_xrDestroySession) resolve("xrDestroySession"), std::memory_order_release);
    g_next_destroy_instance.store((PFN_xrDestroyInstance)resolve("xrDestroyInstance"),std::memory_order_release);

    const uint64_t prev = g_instance.exchange((uint64_t)*instance, std::memory_order_acq_rel);

    _snprintf_s(g_build_stamp, sizeof(g_build_stamp), _TRUNCATE,
                HALOVR_LAYER_NAME " abi=%u (built " __DATE__ " " __TIME__ ")",
                (unsigned)HALOVR_LAYER_ABI_VERSION);

    logf("%s", g_build_stamp);
    logf("gate: %s", g_gate_reason);
    logf("instance %p created; app='%s'; xrEndFrame(next)=%p",
         (void*)*instance,
         info->applicationInfo.applicationName,
         (void*)g_next_end_frame.load());

    if (prev != 0) {
        // ONE INSTANCE IS ASSUMED, and this line is here so the assumption is visible if it ever
        // stops holding. UEVR creates exactly one XrInstance. If a host created a second, the
        // handles the plugin reads through get_instance()/get_session() would silently start
        // describing the newer one -- which is a wrong answer, not a crash, and therefore the kind
        // that goes unnoticed. If this line ever appears in a log, the state above needs to become
        // a small per-instance table.
        logf("WARNING: a SECOND XrInstance was created (previous %p). The bridge tracks only the "
             "newest; the plugin's handles now describe that one.", (void*)prev);
    }
    return r;
}

// ============================================================================================
// The API handed to halo_vr.dll
// ============================================================================================

XRAPI_ATTR XrInstance XRAPI_CALL api_get_instance(void) {
    return (XrInstance)g_instance.load(std::memory_order_acquire);
}

XRAPI_ATTR XrSession XRAPI_CALL api_get_session(void) {
    return (XrSession)g_session.load(std::memory_order_acquire);
}

XRAPI_ATTR PFN_xrVoidFunction XRAPI_CALL api_get_proc(const char* name) {
    PFN_xrGetInstanceProcAddr next = g_next_gipa.load(std::memory_order_acquire);
    const uint64_t inst = g_instance.load(std::memory_order_acquire);
    if (next == nullptr || inst == 0 || name == nullptr) return nullptr;

    PFN_xrVoidFunction f = nullptr;
    if (XR_FAILED(next((XrInstance)inst, name, &f))) return nullptr;
    return f;
}

XRAPI_ATTR int XRAPI_CALL api_set_end_frame_callback(HaloVrLayerEndFrameFn fn, void* user) {
    if (fn == nullptr) {
        g_cb.store(nullptr, std::memory_order_release);
        // BLOCK UNTIL QUIESCENT. The caller is entitled to tear down whatever the callback reads
        // the instant this returns, which is only true if no submit-thread call is still inside it.
        while (g_cb_inflight.load(std::memory_order_acquire) != 0) { Sleep(0); }
        g_cb_user.store(nullptr, std::memory_order_release);
        logf("end-frame callback cleared");
        return 1;
    }
    if (!gate_ok()) return 0;
    // User pointer FIRST, then the function: the submit thread reads the function to decide whether
    // to call at all, so publishing it last means it never sees a function with a stale user value.
    g_cb_user.store(user, std::memory_order_release);
    g_cb.store(fn, std::memory_order_release);
    logf("end-frame callback registered (%p)", (void*)fn);
    return 1;
}

XRAPI_ATTR uint64_t XRAPI_CALL api_frames_seen(void) {
    return g_frames_seen.load(std::memory_order_relaxed);
}

XRAPI_ATTR uint64_t XRAPI_CALL api_layers_appended(void) {
    return g_layers_appended.load(std::memory_order_relaxed);
}

XRAPI_ATTR const char* XRAPI_CALL api_status(void) {
    _snprintf_s(g_status_line, sizeof(g_status_line), _TRUNCATE,
                "%s | gate=%s | instance=%p session=%p next_end_frame=%p | frames=%llu "
                "appended=%llu refused=%llu rejects=%llu | cb=%s | mono=%d patched=%llu",
                g_build_stamp, g_gate_reason,
                (void*)g_instance.load(), (void*)g_session.load(),
                (void*)g_next_end_frame.load(),
                (unsigned long long)g_frames_seen.load(),
                (unsigned long long)g_layers_appended.load(),
                (unsigned long long)g_batch_refused.load(),
                (unsigned long long)g_runtime_rejects.load(),
                g_cb.load() != nullptr ? "set" : "none",
                g_projection_mono.load(std::memory_order_relaxed),
                (unsigned long long)g_mono_patched.load(std::memory_order_relaxed));
    return g_status_line;
}

// Names for the log, indexed by mode. Kept in ONE place so the flip line and the plugin's line
// cannot describe the same number two different ways.
const char* mono_mode_name(int m) {
    switch (m) {
        case 1:  return "ON mode 1: left eye's image to every view";
        case 2:  return "ON mode 2: right eye's image to every view";
        case 3:  return "ON mode 3: app QUAD layers dropped, projection kept";
        case 4:  return "ON mode 4: PROJECTION dropped, app quad layers kept";
        default: return "OFF";
    }
}

XRAPI_ATTR int XRAPI_CALL api_set_projection_mono(int on) {
    // Behind the gate like everything else: an inert layer must not start rewriting a stranger's
    // projection layers because a plugin asked. g_enabled is the same decision xrEndFrame honours.
    if (g_enabled.load(std::memory_order_acquire) != 1) return 0;
    // 0 = off, 1/2 = left/right eye to every view, 3 = drop app quads, 4 = drop the projection.
    // Anything outside clamps. See mono_mode_name().
    const int want = (on <= 0) ? 0 : ((on >= 4) ? 4 : on);
    const int prev = g_projection_mono.exchange(want, std::memory_order_relaxed);
    // Say so in the LAYER's log, on change only. The plugin logs what it asked for; this is the
    // record of what the layer actually accepted, plus how many frames it had rewritten up to the
    // flip -- so an OFF line reads as "N frames went mono", not just "switched".
    if (prev != want) {
        logf("mono projection %s (layers rewritten/dropped so far=%llu)", mono_mode_name(want),
             (unsigned long long)g_mono_patched.load(std::memory_order_relaxed));
    }
    return 1;
}

const HaloVrLayerApi g_api = {
    (uint32_t)sizeof(HaloVrLayerApi),
    HALOVR_LAYER_ABI_VERSION,
    g_build_stamp,
    api_get_instance,
    api_get_session,
    api_get_proc,
    api_set_end_frame_callback,
    api_frames_seen,
    api_layers_appended,
    api_status,
    api_set_projection_mono,   // appended after ABI 1; callers size-check before use
};

}   // namespace

// ============================================================================================
// Exports
// ============================================================================================

extern "C" {

// The plugin's entry point into the layer. Deliberately refuses rather than adapts: see the
// versioning section of XrLayerAbi.h for why a contract that cannot detect its own skew is the
// failure this project has already paid for once.
__declspec(dllexport) XRAPI_ATTR const HaloVrLayerApi* XRAPI_CALL
halovr_layer_get_api(uint32_t abi_version) {
    if (abi_version != HALOVR_LAYER_ABI_VERSION) return nullptr;
    if (!gate_ok()) return nullptr;
    return &g_api;
}

// The loader's entry point into the layer, and the only symbol the OpenXR loader looks for.
//
// THIS RUNS IN EVERY OPENXR APPLICATION ON THE MACHINE once the layer is registered implicitly. It
// must succeed, cheaply, and without doing anything -- the gate is evaluated at instance creation,
// not here, because refusing to negotiate is the one thing that could stop an unrelated application
// from starting.
__declspec(dllexport) XRAPI_ATTR XrResult XRAPI_CALL
xrNegotiateLoaderApiLayerInterface(const XrNegotiateLoaderInfo* loader_info,
                                   const char* api_layer_name,
                                   XrNegotiateApiLayerRequest* api_layer_request) {
    if (loader_info == nullptr || api_layer_request == nullptr) return XR_ERROR_INITIALIZATION_FAILED;

    if (loader_info->structType    != XR_LOADER_INTERFACE_STRUCT_LOADER_INFO ||
        loader_info->structVersion != XR_LOADER_INFO_STRUCT_VERSION ||
        loader_info->structSize    != sizeof(XrNegotiateLoaderInfo)) {
        return XR_ERROR_INITIALIZATION_FAILED;
    }
    if (api_layer_request->structType    != XR_LOADER_INTERFACE_STRUCT_API_LAYER_REQUEST ||
        api_layer_request->structVersion != XR_API_LAYER_INFO_STRUCT_VERSION ||
        api_layer_request->structSize    != sizeof(XrNegotiateApiLayerRequest)) {
        return XR_ERROR_INITIALIZATION_FAILED;
    }
    if (XR_CURRENT_LOADER_API_LAYER_VERSION < loader_info->minInterfaceVersion ||
        XR_CURRENT_LOADER_API_LAYER_VERSION > loader_info->maxInterfaceVersion) {
        return XR_ERROR_INITIALIZATION_FAILED;
    }
    if (XR_CURRENT_API_VERSION < loader_info->minApiVersion ||
        XR_CURRENT_API_VERSION > loader_info->maxApiVersion) {
        return XR_ERROR_INITIALIZATION_FAILED;
    }
    // A loader may ask a multi-layer library which layer it wants. We publish exactly one, so any
    // other name is not ours.
    if (api_layer_name != nullptr && api_layer_name[0] != '\0' &&
        strcmp(api_layer_name, HALOVR_LAYER_NAME) != 0) {
        return XR_ERROR_INITIALIZATION_FAILED;
    }

    api_layer_request->layerInterfaceVersion  = XR_CURRENT_LOADER_API_LAYER_VERSION;
    api_layer_request->layerApiVersion        = XR_CURRENT_API_VERSION;
    api_layer_request->getInstanceProcAddr    = layer_xrGetInstanceProcAddr;
    api_layer_request->createApiLayerInstance = layer_xrCreateApiLayerInstance;
    return XR_SUCCESS;
}

}   // extern "C"

BOOL APIENTRY DllMain(HMODULE module, DWORD reason, LPVOID) {
    // NOTHING HAPPENS HERE. This runs under the loader lock, in every OpenXR application on the
    // machine, and anything beyond recording a handle -- a file open, a registry read, a thread, an
    // allocation -- would be work done in processes that have nothing to do with this mod. The gate
    // is evaluated lazily on the first call that could matter.
    if (reason == DLL_PROCESS_ATTACH) {
        g_self = module;
        DisableThreadLibraryCalls(module);
    }
    return TRUE;
}
