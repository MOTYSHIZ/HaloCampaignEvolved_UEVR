// XrLayer -- compositor-side reticule. Doctrine, staging and the fail-open contract are in
// XrLayer.hpp; read that first. This file is the machinery.
//
// THREADING.
//   GAME THREAD    xrlayer_notice_reticule(), xrlayer_tick(), xrlayer_set_source(),
//                  xrlayer_capture_source(). Reads g_cfg, writes the pose snapshot, runs the
//                  watchdog, owns the D3D12 OBJECT LIFETIMES, and -- since 2026-08-23 -- owns the
//                  one copy that reads an engine-owned resource. Never calls OpenXR.
//   SUBMIT THREAD  produce_layers(). Reads the snapshot, owns every OpenXR call and the copy into
//                  the swapchain image. Never calls UE reflection, never allocates on the steady
//                  path, and NEVER touches a resource this plugin did not create.
//
// ATTACHMENT -- HOW produce_layers() GETS CALLED. Two rungs, chosen in resolve_openxr(), reported
// as tier= in the state line. The per-frame work is identical on both; only the plumbing differs.
//
//   tier=apilayer  THE SHIPPING ROUTE. Our own OpenXR API layer (built in apilayer/, registered by
//                  scripts/Register-XrApiLayer.ps1) is loaded into the process by the loader and
//                  calls bridge_end_frame(); the layer does the copy, the call-through and the
//                  fail-open retry. No PDB, no signature, no recorded address, and NOTHING HERE TO
//                  ROT when UEVR or the game updates. XrLayerBridge.hpp is our side of it.
//
//   tier=pdb       DEV FALLBACK. An inline hook on xrEndFrame, whose address comes from symbols in
//                  UEVRBackend.pdb. Exact, correct, and CANNOT REACH PLAYERS -- a player install
//                  has no PDB, so this rung resolves nothing and the feature latches off. That is
//                  the whole reason the layer above exists; see XrLayerAttach.hpp.
//
// The two share the pose snapshot (seqlock), the config mirror (per-field atomics), and one atomic
// texture pointer that is only ever g_owned or nullptr. Each has its own command list, allocators
// and fence; the ID3D12CommandQueue is shared, which is legal (free-threaded) and is what orders
// capture-then-present on the GPU.
//
// The "game thread never calls D3D12" rule that used to be written here was not a safety property,
// it was a habit -- and obeying it is what forced the engine-resource read onto the submit thread,
// where nothing could validate it. Three crashes later, the copy is on the thread that can.

#include "XrLayer.hpp"
#include "ScopeMask.hpp"

#include "Config.hpp"
#include "Scope.hpp"   // g_scope_active -- the one-reticule-at-a-time gate
#include "DevTools.hpp"
#include "addrcascade/AddressCascade.hpp"
#include "XrLayerAttach.hpp"

#include "uevr/API.hpp"

#include <Windows.h>
#include <d3d12.h>

#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_D3D12
#include "thirdparty/openxr/openxr.h"
#include "thirdparty/openxr/openxr_platform.h"

// The SHIPPING attachment. Included after the OpenXR headers on purpose: XrLayerAbi.h pulls in the
// same vendored copy, and both sides of the ABI must see one set of struct layouts.
#include "XrLayerBridge.hpp"

#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

using uevr::API;

namespace halo {
namespace {

constexpr const char* TAG = "[Halo-CampE-UEVR] XRLAYER:";

void logf(const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    _vsnprintf_s(buf, sizeof(buf), _TRUNCATE, fmt, ap);
    va_end(ap);
    API::get()->log_info("%s %s", TAG, buf);
}

// ============================================================================================
// OpenXR entry points
// ============================================================================================
//
// ADDR-HYGIENE: resolved -- every one of these is looked up BY NAME, never recorded as an address.
// The winning tier is a symbol lookup in UEVRBackend.pdb (see XrLayerAttach.hpp), which is
// authoritative and tracks UEVR pin bumps on its own because a PDB always matches its own DLL.
// There is no byte signature and no offset in this file, so a UEVR update has nothing here to rot.
//
// What DID rot was an assumption, not an address: the first version resolved these from
// openxr_loader.dll, which is loaded in the process but which UEVR never calls. Every check
// passed and nothing ran. XrLayerAttach.hpp has the full account.

struct XrFns {
    PFN_xrEndFrame                 end_frame              = nullptr;
    PFN_xrEnumerateSwapchainFormats enumerate_formats     = nullptr;
    PFN_xrCreateSwapchain          create_swapchain       = nullptr;
    PFN_xrDestroySwapchain         destroy_swapchain      = nullptr;
    PFN_xrEnumerateSwapchainImages enumerate_images       = nullptr;
    PFN_xrAcquireSwapchainImage    acquire_image          = nullptr;
    PFN_xrWaitSwapchainImage       wait_image             = nullptr;
    PFN_xrReleaseSwapchainImage    release_image          = nullptr;
    // OPTIONAL -- the layer-budget query. Not part of the all-or-nothing set below: failing to
    // resolve these costs us the measured budget and nothing else, and the fallback is the spec's
    // own guaranteed minimum rather than a guess.
    PFN_xrGetSystem                get_system             = nullptr;
    PFN_xrGetSystemProperties      get_system_props       = nullptr;
    bool ok = false;
};
XrFns g_xr;

// WHICH RUNG WON. Recorded rather than inferred: the two tiers reach the compositor by completely
// different routes -- one appends inside our own inline hook, the other hands quads to a layer that
// appends them for us -- and a log that does not say which is in play cannot be read after the fact.
XrAttachTier g_tier = XrAttachTier::None;

// Set by resolve_openxr() when it returns false for a TIMING reason rather than a failure. The
// difference decides whether the caller waits or latches Failed, and getting it wrong turns "the
// session has not started yet" into "this feature is broken for the rest of the run".
bool g_resolve_waiting = false;

// True once the API layer has accepted our end-frame callback. The hook rung has g_hook_id; this is
// its counterpart, and remove_hook() must clear whichever one is set.
bool g_bridge_cb = false;

const char* tier_name(XrAttachTier t) {
    switch (t) {
        case XrAttachTier::ApiLayer:     return "apilayer";
        case XrAttachTier::BackendPdb:   return "pdb";
        case XrAttachTier::LoaderExport: return "loader";
        default:                         return "none";
    }
}

// EVERY entry point comes from the SAME loader instance, and it must be the one UEVR uses.
//
// This is not only about the hook. The XrSession the plugin API hands us belongs to the loader
// statically linked inside UEVRBackend.dll; passing that handle to openxr_loader.dll's
// xrCreateSwapchain would hand a handle to a different loader's dispatch table. Mixing the two is
// not a near-miss, and the first version of this file did exactly that for all eight functions --
// it only ever looked like a hook problem because the hook never fired to expose the rest.
bool resolve_openxr() {
    if (g_xr.ok) return true;
    g_resolve_waiting = false;

    // ---- THE SHIPPING RUNG, ASKED FIRST: our own OpenXR API layer -----------------------------
    //
    // Asked before the PDB for the reason XrLayerAttach.hpp spells out at length: the PDB tier is
    // exact, correct, and CANNOT REACH PLAYERS, because UEVRBackend.pdb exists only in a UEVR
    // checkout. A player install resolves nothing, the feature latches off, and it does so while
    // shipping enabled -- which is the worst shape a failure can take. The layer needs no PDB, no
    // signature and no recorded address; the loader hands it the chain for free.
    //
    // get_proc() resolves BELOW our layer on the SAME chain the session came from, so the handle
    // and the calls belong to one loader instance BY CONSTRUCTION. That is the exact correctness
    // argument this module lost the first time round, when the session came from UEVR and the
    // entry points came from openxr_loader.dll.
    if (const HaloVrLayerApi* api = xrbridge_api()) {
        // THE INSTANCE MAY NOT EXIST YET, AND THAT IS NOT A FAILURE. api->get_proc() answers null
        // for every name until the layer has seen xrCreateInstance, so resolving now would fail the
        // whole tier for a timing reason and latch us onto the PDB for the rest of the session --
        // on a player machine, onto nothing at all. Wait instead; the caller retries every tick.
        if (api->get_instance() == XR_NULL_HANDLE) {
            static uint32_t waited  = 0;
            static bool     gave_up = false;
            if (!gave_up && ++waited >= 900) {          // ~10 s at the game-thread tick rate
                gave_up = true;
                logf("the API layer is loaded but no OpenXR instance has appeared in ~10 s. That is "
                     "not what a working layer looks like; falling through to the PDB tier so a dev "
                     "checkout still has a route. Layer status: %s", xrbridge_status());
            }
            if (!gave_up) {
                static bool said = false;
                if (!said) {
                    said = true;
                    logf("API layer present -- waiting for the OpenXR instance before resolving "
                         "entry points through it. %s", xrbridge_status());
                }
                g_resolve_waiting = true;
                return false;
            }
        } else {
            auto lget = [&](const char* name) -> void* {
                void* f = (void*)api->get_proc(name);
                if (f == nullptr) logf("the API layer could not resolve %s", name);
                return f;
            };
            XrFns f{};
            f.enumerate_formats = (PFN_xrEnumerateSwapchainFormats)lget("xrEnumerateSwapchainFormats");
            f.create_swapchain  = (PFN_xrCreateSwapchain)          lget("xrCreateSwapchain");
            f.destroy_swapchain = (PFN_xrDestroySwapchain)         lget("xrDestroySwapchain");
            f.enumerate_images  = (PFN_xrEnumerateSwapchainImages) lget("xrEnumerateSwapchainImages");
            f.acquire_image     = (PFN_xrAcquireSwapchainImage)    lget("xrAcquireSwapchainImage");
            f.wait_image        = (PFN_xrWaitSwapchainImage)       lget("xrWaitSwapchainImage");
            f.release_image     = (PFN_xrReleaseSwapchainImage)    lget("xrReleaseSwapchainImage");
            // Optional here for the same reason as below: losing these costs the measured layer
            // budget and nothing else.
            f.get_system        = (PFN_xrGetSystem)                lget("xrGetSystem");
            f.get_system_props  = (PFN_xrGetSystemProperties)      lget("xrGetSystemProperties");

            // NOTE WHAT IS DELIBERATELY NOT REQUIRED: xrEndFrame. On this rung the LAYER owns that
            // call and we never make it, so demanding it would fail the shipping route over a
            // function it does not use. The hook rung below still requires it, because there it IS
            // the hook. end_frame stays null here, and hooked_end_frame is never installed.
            if (f.enumerate_formats && f.create_swapchain && f.destroy_swapchain &&
                f.enumerate_images && f.acquire_image && f.wait_image && f.release_image) {
                g_xr    = f;
                g_xr.ok = true;
                g_tier  = XrAttachTier::ApiLayer;
                logf("entry points resolved through the API LAYER -- no PDB, no signature, no "
                     "recorded address. %s", xrbridge_status());
                return true;
            }
            logf("the API layer is present and has an instance, but did not resolve the swapchain "
                 "entry points. Falling through to the PDB tier.");
        }
    }

    if (!xrattach_ready()) { g_resolve_waiting = true; return false; }   // PDB still loading

    XrAttachTier tier = XrAttachTier::None;
    XrAttachTier first_tier = XrAttachTier::None;
    bool mixed = false;

    auto get = [&](const char* name) -> void* {
        void* p = xrattach_resolve(name, &tier);
        if (p == nullptr) { logf("could not resolve %s", name); return nullptr; }
        if (first_tier == XrAttachTier::None) first_tier = tier;
        else if (tier != first_tier)          mixed = true;
        return p;
    };

    g_xr.end_frame          = (PFN_xrEndFrame)                 get("xrEndFrame");
    g_xr.enumerate_formats  = (PFN_xrEnumerateSwapchainFormats)get("xrEnumerateSwapchainFormats");
    g_xr.create_swapchain   = (PFN_xrCreateSwapchain)          get("xrCreateSwapchain");
    g_xr.destroy_swapchain  = (PFN_xrDestroySwapchain)         get("xrDestroySwapchain");
    g_xr.enumerate_images   = (PFN_xrEnumerateSwapchainImages) get("xrEnumerateSwapchainImages");
    g_xr.acquire_image      = (PFN_xrAcquireSwapchainImage)    get("xrAcquireSwapchainImage");
    g_xr.wait_image         = (PFN_xrWaitSwapchainImage)       get("xrWaitSwapchainImage");
    g_xr.release_image      = (PFN_xrReleaseSwapchainImage)    get("xrReleaseSwapchainImage");

    // Optional, and resolved through the SAME path for the same reason as the rest: an XrInstance
    // that belongs to UEVR's statically-linked loader must not be handed to another loader's
    // dispatch table. Nothing here is added to `all` -- see the struct.
    g_xr.get_system         = (PFN_xrGetSystem)          get("xrGetSystem");
    g_xr.get_system_props   = (PFN_xrGetSystemProperties)get("xrGetSystemProperties");

    const bool all = g_xr.end_frame && g_xr.enumerate_formats && g_xr.create_swapchain &&
                     g_xr.destroy_swapchain && g_xr.enumerate_images && g_xr.acquire_image &&
                     g_xr.wait_image && g_xr.release_image;
    if (!all) return false;

    // A SPLIT ANSWER IS WORSE THAN NO ANSWER. If some functions came from the PDB and others from
    // the loader export, we would be driving one loader's session with another loader's calls --
    // which fails as memory corruption, not as an error code. Refuse.
    if (mixed) {
        logf("REFUSING: entry points resolved from MIXED sources. A session belongs to exactly one "
             "loader, and calling across two corrupts rather than errors.");
        return false;
    }

    if (first_tier == XrAttachTier::LoaderExport) {
        logf("REFUSING: entry points came from openxr_loader.dll, but UEVR statically links its own "
             "loader (no openxr_loader.dll import in UEVRBackend.dll). Those addresses are real and "
             "wrong. Needs a dev build with UEVRBackend.pdb beside the DLL. Status: %s",
             xrattach_status());
        return false;
    }

    logf("entry points resolved from UEVRBackend.pdb -- xrEndFrame @ %p. THIS IS THE DEV RUNG: a "
         "player install has no PDB, so a release that reaches here has no compositor overlay. "
         "Register the API layer to get the shipping route.", (void*)g_xr.end_frame);
    g_xr.ok = true;
    g_tier  = XrAttachTier::BackendPdb;
    return true;
}

// ============================================================================================
// The pose snapshot (game thread -> submit thread)
// ============================================================================================
//
// A seqlock rather than a mutex: the submit thread must never block on the game thread, and a
// stale frame is harmless where a stall is not. Odd sequence = write in progress.

// THE FINISHED STAGE-SPACE POSE, not the ingredients for one.
//
// This used to carry the raw UE positions and let the submit thread build the pose against the
// CURRENT head rotation. That is what the drift was: the offset was expressed in camera-local
// coordinates at snapshot time, then re-anchored to a fresher head orientation at display time, so
// the quad swung WITH the head instead of staying put in the world. Between a 32 Hz publish and a
// 90 Hz display that is up to ~30 ms of head rotation -- small, constant, and exactly the "does not
// stay centred, drifts a bit" the first headset pass reported.
//
// Computing the stage pose here, from the head pose sampled at the SAME instant as the aim data,
// makes it world-locked by construction: the compositor then holds it against the real display-time
// pose, which is the entire reason to use a composition layer. It also takes the per-frame
// get_pose() off the submit thread.
struct Snapshot {
    XrPosef  pose{};          // stage (or view) space, ready to submit
    float    size_m = 0.05f;
    // SECOND EXTENT, 0 = square (which every slot but the guide is). XrCompositionLayerQuad::size
    // is an XrExtent2Df; the squareness was our own single scalar, never the runtime's. A beam is
    // long and thin, so it needs the other half. The drop key `ang` deliberately keeps using
    // size_m alone -- a beam's apparent size is its LENGTH, and ranking a thin quad by its
    // thickness would make the guide the first thing dropped every time.
    float    size_h_m = 0.0f;
    // THE DROP KEY: apparent (angular) size, size_m / distance_m. One number that expresses
    // "furthest/smallest first" exactly -- a marker twice as far with the same quad size has half
    // the angular size, and so does a marker at the same range drawn half as big. When the runtime
    // has fewer free layers than we have quads, the smallest angular size goes first.
    float    ang    = 0.0f;
    uint32_t tick   = 0;
    int      prio   = 0;      // 0 = never dropped
    bool     valid  = false;
};

// ONE seqlock over ALL slots, not one per slot. The submit thread needs a mutually consistent set
// -- nine independently-torn poses would let two markers be a frame apart, which is exactly the
// shimmer the render-rate split exists to remove. The struct is ~450 bytes; copying it under the
// seqlock is a memcpy, not a lock.
struct Frame {
    Snapshot slot[XRLAYER_SLOTS];
};

std::atomic<uint32_t> g_seq{0};
Frame                 g_frame{};         // guarded by g_seq
std::atomic<uint32_t> g_game_tick{0};

// The TICK-RATE half: which world point, and how big. Written by xrlayer_notice_reticule /
// xrlayer_notice_quad (game thread), read by xrlayer_note_eye (render thread). Individually atomic
// rather than seqlocked -- a torn combination is one frame of a slightly stale target, which is
// invisible beside the drift that splitting these apart removes.
//
// g_tgt_cm IS A WORLD SIZE IN UE CENTIMETRES, not a multiplier. The reticule used to publish its
// 1/distance compensation factor and let compute_pose() multiply it by aim_widget_draw x
// aim_widget_scale; that hard-coded ONE caller's config into this module and there is now more than
// one caller. Each caller computes its own world size and passes it.
std::atomic<float>    g_tgt_x[XRLAYER_SLOTS]{}, g_tgt_y[XRLAYER_SLOTS]{}, g_tgt_z[XRLAYER_SLOTS]{};
std::atomic<float>    g_tgt_cm[XRLAYER_SLOTS]{};
// Second extent, 0 = square. Only the guide sets it; see xrlayer_notice_quad.
std::atomic<float>    g_tgt_cm_h[XRLAYER_SLOTS]{};
std::atomic<float>    g_tgt_hold[XRLAYER_SLOTS]{};   // 0 = at the point; >0 = along the ray at this
std::atomic<int>      g_tgt_prio[XRLAYER_SLOTS]{};
std::atomic<uint32_t> g_tgt_tick[XRLAYER_SLOTS]{};
std::atomic<bool>     g_tgt_live[XRLAYER_SLOTS]{};
// TARGET IS AN OFFSET FROM THE EYE, NOT A WORLD POINT -- for quads glued to something that moves
// WITH the player, which is the case a world point handles worst.
//
// compute_pose does d_world = target - eye. The target is chosen on the 32 Hz game tick; the eye
// belongs to the frame being drawn. For a WORLD-FIXED thing (a navpoint) that is exactly right and
// is why the subtraction lives at render rate. For a HAND-ATTACHED thing it is exactly wrong: while
// you locomote, both your hand and your eye translate together, so the fresh eye is differenced
// against a stale hand and the quad sits one tick of travel BEHIND the controller. Reported as
// "the xr layer visual lags behind when I locomote", and at a walking pace a tick is several cm.
//
// With this set, the stored vector is (target - eye) captured at publish time and the eye is added
// back at render rate, so the two sides of the subtraction come from the same instant and the
// translation cancels. What remains is sub-frame, not sub-tick.
std::atomic<bool>     g_tgt_headrel[XRLAYER_SLOTS]{};
// The MONO camera the head-relative offsets were measured against, published for readers on the
// SUBMIT thread that need to turn an offset back into a world point. note_eye writes it every
// frame. Anything reading g_tgt_* for WORLD-SPACE maths must add this back when the slot's
// headrel flag is set -- the scope-pane reticule (mode 2) is one such reader, and it silently
// became wrong the moment head-relative targets landed, because it was written when g_tgt_* could
// only ever hold a world position.
std::atomic<float>    g_mono_view_x{0.0f}, g_mono_view_y{0.0f}, g_mono_view_z{0.0f};
std::atomic<bool>     g_mono_view_have{false};

// The RENDER-RATE half: each eye's last rendered position, so their midpoint gives the head.
std::atomic<float>    g_eye_x[2]{}, g_eye_y[2]{}, g_eye_z[2]{};
std::atomic<bool>     g_eye_have[2]{};

void publish(const Frame& f) {
    const uint32_t seq = g_seq.load(std::memory_order_relaxed);
    g_seq.store(seq + 1, std::memory_order_release);       // -> odd
    std::atomic_thread_fence(std::memory_order_release);
    g_frame = f;
    std::atomic_thread_fence(std::memory_order_release);
    g_seq.store(seq + 2, std::memory_order_release);       // -> even
}

bool read_snapshot(Frame* out) {
    for (int attempt = 0; attempt < 4; ++attempt) {
        const uint32_t a = g_seq.load(std::memory_order_acquire);
        if ((a & 1u) != 0u) continue;                       // writer mid-update
        std::atomic_thread_fence(std::memory_order_acquire);
        *out = g_frame;
        std::atomic_thread_fence(std::memory_order_acquire);
        if (g_seq.load(std::memory_order_acquire) == a) return true;
    }
    return false;                                           // contended; skip this frame
}

// ============================================================================================
// Live state
// ============================================================================================

enum class State {
    Off,        // feature disabled, or never started
    Pending,    // enabled, waiting for a live session in the hook
    Armed,      // swapchain up, appending layers
    Failed      // latched off after a failure; only a config toggle retries
};

std::atomic<State> g_state{State::Off};

int          g_hook_id      = -1;
PFN_xrEndFrame g_end_frame_orig = nullptr;

XrSession    g_session     = XR_NULL_HANDLE;
XrSwapchain  g_swapchain   = XR_NULL_HANDLE;
int32_t      g_sc_w = 0, g_sc_h = 0;

// ============================================================================================
// ONE SWAPCHAIN, A TEXTURE ATLAS, NINE QUADS
// ============================================================================================
//
// The obvious way to draw nine quads is nine swapchains. It is also the wrong way, and the reason
// is a number already in this file: xrWaitSwapchainImage is called with a 20 ms timeout on the
// SUBMIT THREAD. Nine swapchains is nine acquire/wait/release rounds per frame and a 180 ms worst
// case on the thread that hands frames to the compositor. In VR that is not a blemish, it is
// nausea -- the repo's own framing, and the reason the diagnostic sweeps are compile-gated.
//
// XrSwapchainSubImage::imageRect exists precisely so N quads can reference disjoint rectangles of
// ONE image, so that is what this does: one acquire, one wait, one release, one command list, one
// fence, and the existing three-deep allocator ring scales unchanged. Nine XrCompositionLayerQuads
// all point at the same swapchain with different imageRects.
//
// The cost of the atlas is one extra rule everywhere a copy happens: the destination is a
// RECTANGLE now, so CopyResource becomes CopyTextureRegion with a destination offset. The
// format-family rule is untouched (a copy still has to stay inside one typeless family), and the
// "refuse rather than copy" gate in xrlayer_set_slot_source() changes from "source dims must equal
// the swapchain" to "source dims must equal ITS CELL".
struct Cell {
    int32_t x = 0, y = 0, dim = 0;   // dim 0 = this slot has no cell
};
Cell g_cell[XRLAYER_SLOTS]{};

// The generated ring's cell edge when no atlas is built (nav off) -- kept as its own name so the
// reticule-only layout reads the same as it did before slots existed.
int32_t g_ret_dim = 0;
int32_t g_nav_dim = 0;

// The pane cell (slot 9). REQUESTED by xrlayer_pane_configure() before bring-up; GRANTED as
// g_pane_dim by build_atlas_layout(), which may refuse it to keep the reticule and markers. The two
// are separate on purpose: "what was asked for" and "what the atlas actually has" answer different
// questions, and collapsing them is how a refused pane reads as a pane that exists.
int32_t g_pane_req = 0;
int32_t g_pane_dim = 0;

// PER-SLOT ORIENTATION OVERRIDE. Absent (the default, and every slot's state until someone calls
// xrlayer_set_quad_orientation) means head-oriented, which is what all nine slots did before the
// pane existed and what every billboarded slot still wants.
//
// Individually atomic rather than seqlocked, matching g_tgt_* above and for the same reason: these
// are published by the game thread and read while composing the render-rate snapshot, and a torn
// read is one frame of a slightly stale weapon angle -- invisible next to the tick-to-render lag
// that the split already accepts.
//
// STORED AS UE WORLD DIRECTIONS, converted to an XR orientation at the point of use through
// ue_offset_to_xr() -- the same proven mapping every quad's POSITION goes through. See the header:
// carrying a UE quaternion across to XrPosef::orientation would cross a determinant -1 basis change
// and is not a componentwise copy.
//
// BE PRECISE ABOUT WHY, because the loose version of this claim is wrong and would mislead whoever
// reads it next. It is NOT that a rotation cannot cross a det -1 map: conjugation carries one
// correctly, since det(M R M^-1) = det(R) = +1, and that form also reverses the rotation SENSE the
// way a handedness flip requires. What is broken is the shortcut everyone actually reaches for --
// map the quaternion's (x, y, z) with the vector map and keep w -- which yields a MIRRORED rotation,
// exact on the mirror axis and quietly wrong off it.
//
// So the choice here is not "rotations are impossible", it is that directions need no such care at
// all: they go through the one mapping already proven in a headset, and the basis is assembled on
// this side. (Correction owed to the scope session, 2026-08-26, who caught the overstatement.)
// The last COMPOSED view rotator, kept for xrlayer_view_basis(). Written by xrlayer_note_eye on the
// render thread; g_view_have stays false until a stereo frame has actually been composed, which is
// the difference between "the view is level" and "there is no view yet" -- a distinction a consumer
// reading raw angles cannot make, and one that reads as a plausible identity basis at the frontend.
std::atomic<float> g_view_yaw{0.0f}, g_view_pitch{0.0f}, g_view_roll{0.0f};
std::atomic<bool>  g_view_have{false};

// Which upstream gate stopped the reticule publish this tick (see xrlayer_note_publish_gate).
// Relaxed: it is read only by the once-a-tick dark reporter, and a torn read there is one
// stale reason on a line that is already rate-limited.
std::atomic<int> g_publish_gate{0};

// The raw scope ray (UE world cm), published by the same site that feeds scope_notice_ray.
std::atomic<float> g_scope_ray_ox{0.0f}, g_scope_ray_oy{0.0f}, g_scope_ray_oz{0.0f};
std::atomic<float> g_scope_ray_tx{0.0f}, g_scope_ray_ty{0.0f}, g_scope_ray_tz{0.0f};
std::atomic<bool>  g_scope_ray_have{false};

std::atomic<bool>  g_slot_orient_on[XRLAYER_SLOTS]{};
std::atomic<float> g_slot_fwd_x[XRLAYER_SLOTS]{}, g_slot_fwd_y[XRLAYER_SLOTS]{}, g_slot_fwd_z[XRLAYER_SLOTS]{};
std::atomic<float> g_slot_up_x[XRLAYER_SLOTS]{},  g_slot_up_y[XRLAYER_SLOTS]{},  g_slot_up_z[XRLAYER_SLOTS]{};

// HOW MANY COMPOSITION LAYERS THE RUNTIME WILL ACCEPT, from xrGetSystemProperties. 0 = not queried
// (fall back to XR_MIN_COMPOSITION_LAYERS_SUPPORTED, never to "unlimited").
uint32_t g_max_layers = 0;
// The format the RUNTIME chose, and the two things that have to follow from it. Neither may be
// assumed: a B8G8R8A8 bitmap copied into an R8G8B8A8 image is not a colour bug, it is an invalid
// copy (the two are different DXGI families and CopyTextureRegion requires a shared typeless
// parent), and a channel swap would show up as the wrong hue rather than as an error.
int64_t      g_format  = 0;
bool         g_is_bgra = true;
// Space handles, fetched once at bring-up. They are session-lifetime and asking per frame was one
// more plugin-API call on the submit thread for a value that never changes.
XrSpace      g_stage_space = XR_NULL_HANDLE;
XrSpace      g_view_space  = XR_NULL_HANDLE;

ID3D12Device*       g_device = nullptr;
ID3D12CommandQueue* g_queue  = nullptr;

std::vector<ID3D12Resource*> g_images;      // swapchain images (owned by the runtime)
std::vector<bool>            g_image_dirty; // needs a copy before it is presented again

ID3D12Resource*            g_upload   = nullptr;   // staging buffer holding the generated bitmap
ID3D12GraphicsCommandList* g_list     = nullptr;
ID3D12Fence*               g_fence    = nullptr;
UINT64                     g_fence_v  = 0;
HANDLE                     g_fence_ev = nullptr;

// A RING OF COMMAND ALLOCATORS, not one.
//
// Stage 1 could afford a single allocator because it fence-WAITED after every copy: the generated
// bitmap never changes, so each swapchain image is written once and the steady path does no copy at
// all. Stage 2's source changes every frame, so the copy happens every frame -- and a blocking wait
// on the submit thread every frame is not a blemish here, it is nausea.
//
// Resetting an allocator whose commands are still executing is undefined behaviour, so the wait
// cannot simply be deleted. Three allocators, each remembering the fence value of the work it last
// recorded, turn it into a wait that only fires if the GPU has fallen more than three frames behind
// -- which on the steady path is never.
constexpr int              FRAME_RING = 3;
ID3D12CommandAllocator*    g_alloc[FRAME_RING] = {};
UINT64                     g_alloc_fence[FRAME_RING] = {};
int                        g_ring = 0;

// ============================================================================================
// STAGE 2's SOURCE -- A TEXTURE WE OWN, NOT THE ENGINE'S
// ============================================================================================
//
// THREE crashes on 2026-08-23, all the same faulting instruction: an EXCEPTION_ACCESS_VIOLATION
// reading 0x3c inside nvwgf2umx, reached from `g_list->ResourceBarrier(1, &to_src)` in blit_into --
// the barrier on the ENGINE-OWNED widget render target. The third dump makes the identification
// exact rather than inferred: the barrier's StateBefore/StateAfter pair (0xC0 = ENGINE_SRC_COLOR,
// 0x800 = COPY_SOURCE) and the resource pointer are both still on the stack, and the driver's
// per-resource allocation register is 0. The object was alive; its GPU backing was not.
//
// TWO PREVIOUS DEFENCES FAILED, and they failed for the same underlying reason.
//
//   1. A TICK COUNTER. LoadMap blocks the game thread, so both sides of "is the game thread
//      current?" froze together and it read healthy through the whole teardown.
//   2. A WALL-CLOCK HEARTBEAT (250 ms) plus per-tick re-validation. Better -- it demonstrably
//      fires, five times in the crashing session's own log -- but it is a CHECK-THEN-USE, and the
//      gap between the check in hooked_end_frame and the barrier in blit_into is not small: on the
//      way there, blit_into waits up to 1000 ms on the frame-ring fence. The crashing frame checked
//      the heartbeat while it was still (just) fresh, blocked ~1 s while the level tore down, and
//      then barriered a resource that had died in the meantime. The arithmetic matches the log to
//      within milliseconds.
//
// There is no third heartbeat that works, because the quantity we need is not "is the game thread
// alive" -- it is "does this resource still have GPU backing", and NOTHING the submit thread can
// ask answers that. The vtable is valid, the refcount is 2, the device matches, GetDesc succeeds,
// and AddRef does not keep the backing alive (UE hands these out of a pooled/placed allocator).
//
// SO WE STOP BORROWING. g_owned is an ID3D12Resource WE create, at the swapchain's exact size and
// format family, held for the life of the layer and kept permanently in COPY_SOURCE. The submit
// thread copies from THAT and issues no barrier on it at all -- the crashing call is not merely
// guarded, it is deleted.
//
// The copy IN still reads the engine's resource, and that read is still the dangerous one. What
// changes is WHO does it and WHEN: it moves to the GAME THREAD, into xrlayer_capture_source(),
// called by XrSource immediately after a full re-validation with nothing blocking in between. That
// does not make the read provably safe -- only UE's own render-command ordering could, and a UEVR
// plugin cannot enqueue into it -- but it removes the state every one of the three crashes was in.
// A game thread parked inside LoadMap issues no captures at all, so the window the submit thread
// used to keep barriering through simply has no code running in it.
// ONE ATLAS-SIZED RESOURCE, NOT ONE PER SLOT, and that is worth stating because the alternative
// was the obvious design. Nine owned textures would be nine D3D12 lifetimes to create, publish,
// unpublish and release -- and xrlayer_shutdown() leaking ONE swapchain into UEVR's session
// teardown is already on record as having hung the game on exit. g_owned mirrors the swapchain
// image exactly (same extent, same typeless family), every slot captures into its own cell of it,
// and the submit thread copies the whole thing in one CopyResource. Adding a slot adds no resource.
ID3D12Resource* g_owned = nullptr;   // GAME THREAD creates/releases; submit thread reads via the
                                     // atomic below. Always in D3D12_RESOURCE_STATE_COPY_SOURCE.

// The engine resources XrSource offered and xrlayer_set_slot_source() accepted, one per slot. GAME
// THREAD ONLY -- deliberately NOT published to the submit thread, which is the point of this whole
// rework: the submit thread must never hold a pointer to an engine-owned resource.
ID3D12Resource* g_capture_src[XRLAYER_SLOTS] = {};

// PER-SLOT freshness, in the same currency as g_src_beat below: when this slot's cell was last
// captured (GetTickCount64 ms). A slot with a stale beat is simply not appended -- its cell still
// holds its last art, but showing a marker whose art stopped updating is how a waypoint ends up
// frozen where the player can walk around it, which the navpoint lane already has a rule against.
std::atomic<uint64_t> g_slot_beat[XRLAYER_SLOTS]{};

// PER-SLOT CELL/HOST COHERENCE (game tick, not ms). A marker slot's WidgetComponent is a stable
// pool object, so its component pointer does not change when the sparse navpoint map reorders --
// instead the game thread re-HOSTS the same component with a different navpoint's widget
// (navw_host_class -> SetWidget). The component's render target then redraws with the new art on a
// LATER frame, but the atlas cell still holds the OLD art until the next capture picks the redraw
// up, while the quad's POSE has already moved to the new navpoint. That one-frame skew is the
// "wrong marker flashed over the objective" report: old widget art at the new marker's position.
//
//   g_slot_host_tick  the game tick the slot was last re-hosted to a NEW widget class.
//   g_slot_cap_tick   the game tick the slot's cell was last actually captured (capture_submit).
//
// A slot's cell is COHERENT only once it has been captured on a tick STRICTLY AFTER its last
// re-host -- that guarantees a full frame passed for the render target to redraw the new widget.
// Until then the marker is simply not put on the layer; Plugin.cpp keeps the in-scene marker
// visible for those one or two frames (on_layer follows xrlayer_slot_ready), which shows coherent
// engine-rendered art+pose and is NOT the generated-ring fall-back. Reticule (slot 0) is never
// re-hosted, so its host tick stays 0 and this never gates it. Written by the game thread (the same
// thread that captures), read by the submit thread, exactly like g_slot_beat.
std::atomic<uint32_t> g_slot_host_tick[XRLAYER_SLOTS]{};
std::atomic<uint32_t> g_slot_cap_tick[XRLAYER_SLOTS]{};

// Has this slot's cell been captured on a tick after its most recent re-host? True for a slot that
// was never re-hosted once it has any capture. False in the one/two-frame window after a re-host,
// which is exactly when the cell art and the quad pose disagree.
bool slot_cell_coherent(int slot) {
    const uint32_t host = g_slot_host_tick[slot].load(std::memory_order_relaxed);
    const uint32_t cap  = g_slot_cap_tick[slot].load(std::memory_order_relaxed);
    if (cap == 0) return host == 0;   // no capture yet: coherent only if nothing was ever hosted
    // Wrap-safe strictly-after over a monotonic game tick (fetch_add each frame).
    return (int32_t)(cap - host) > 0;
}

// The game thread's OWN copy machinery. Separate list, allocators and fence from the submit
// thread's: D3D12 command lists and allocators must be externally synchronised per object, and two
// threads incrementing one fence counter would break its monotonicity. The QUEUE is shared on
// purpose -- submitting both copies to the same queue is what orders capture-then-present without
// any cross-queue fence of our own.
constexpr int              GT_RING = 2;
ID3D12CommandAllocator*    g_gt_alloc[GT_RING] = {};
UINT64                     g_gt_alloc_fence[GT_RING] = {};
int                        g_gt_ring  = 0;
ID3D12GraphicsCommandList* g_gt_list  = nullptr;
ID3D12Fence*               g_gt_fence = nullptr;
UINT64                     g_gt_fence_v = 0;
uint32_t                   g_gt_captures = 0;   // successful submissions, for the state line
uint32_t                   g_gt_skips    = 0;   // ring busy -- never a block, never an error
// The open batch: which ring slot the currently-recording list belongs to (-1 = none), how many
// copies went into it, and which layer slots they were for. See xrlayer_capture_begin().
int                        g_gt_open_slot = -1;
int                        g_gt_recorded  = 0;
uint32_t                   g_gt_rec_mask  = 0;

// STAGE-2 SEAM AS THE SUBMIT THREAD SEES IT. Atomic because the game thread sets it while the
// submit thread reads it every frame. nullptr = present the generated bitmap. It only ever holds
// g_owned or nullptr; an engine pointer must never reach it.
std::atomic<ID3D12Resource*> g_source_override{nullptr};

// WHEN THE GAME THREAD LAST CAPTURED INTO g_owned (GetTickCount64 ms).
//
// THIS IS NOW COSMETIC, NOT A SAFETY MECHANISM, and saying so is the point. It used to be the thing
// standing between the submit thread and a dead engine resource, and it was not up to the job (see
// the g_owned comment above). Since the submit thread now only ever touches a resource we own, a
// stale beat cannot cause a fault -- it just means g_owned still holds the last widget frame the
// game thread managed to capture.
//
// It is kept because a FROZEN crosshair hanging in space through a loading screen looks worse than
// the generated ring, and falling back costs nothing. It must never again be described, or relied
// on, as the thing that makes the copy safe.
std::atomic<uint64_t> g_src_beat{0};

// HOW LONG WE HOLD THE LAST CAPTURED WIDGET FRAME BEFORE GIVING UP AND DRAWING THE RING.
//
// This was a flat 250 ms constant and that number was chosen against ONE question ("is this a
// loading screen?"). It answers a second question it was never sized for, and answers it wrongly:
// any in-gameplay game-thread stall longer than a quarter second ALSO stops the captures, so the
// crosshair is replaced by a placeholder ring for the duration and then pops back. Reported from a
// headset on 2026-08-23 as "the generated ring flashes on screen for a split second" during the
// hitch when you pick a weapon off the floor -- and the log had already stopped saying so, because
// the line below is capped at five prints per session.
//
// The flash is not a second bug next to the hitch. It is this gate, correctly detecting a stall
// that is long enough to matter, and then choosing the worse of the two available pictures. Showing
// the PREVIOUS weapon's crosshair for a beat is far less jarring than a placeholder appearing.
//
// So the window becomes a tunable measured against the OTHER question: it must be comfortably
// longer than the worst in-gameplay hitch (measured 600-750 ms on this title) and comfortably
// shorter than a level load (seconds). 1500 ms sits between those with room on both sides.
//
// SAFETY IS UNAFFECTED, and this is the part to check before changing it. What we keep presenting
// is g_owned -- OUR resource, created in bring-up, held for the life of the layer, never freed
// while it is published (release_d3d unpublishes first). Holding it longer touches NO engine
// resource for longer; if anything it reduces engine contact, because the game thread stops
// capturing the moment the source is dropped. This does not widen any check-then-use gap.
constexpr uint32_t SOURCE_HOLD_MS_DEFAULT = 1500;
std::atomic<uint32_t> g_m_hold_ms{SOURCE_HOLD_MS_DEFAULT};

// HOW MANY TIMES THE COMPOSITOR HAS ACTUALLY FALLEN BACK TO THE RING after presenting real art.
//
// Counted rather than logged, because the log line for this is capped at five prints and therefore
// goes quiet exactly when a player starts noticing the symptom. A COUNTER in the state line makes
// the flash reproducible from the outside: pick a weapon up, watch the number move by one.
std::atomic<uint32_t> g_ring_falls{0};
std::atomic<uint32_t> g_hold_worst_ms{0};   // longest gap we rode out WITHOUT falling back

// HOW MANY QUADS THE BUDGET HAS ACTUALLY THROWN AWAY, cumulative.
//
// Counted rather than logged for the same reason as g_ring_falls, and it exists because of a
// lesson this feature already provided: `skips=0` in the state line means the capture's "previous
// still in flight" rung has NEVER executed, so nothing about it is known to work. The drop path is
// the same shape of hazard -- it only runs when a runtime is short of layers, which on a healthy
// machine is never. xrlayerbudget forces it, and this counter is how you see that it ran.
std::atomic<uint32_t> g_layer_drops{0};

// WHAT THE COMPOSITOR IS PRESENTING RIGHT NOW, written by the submit thread.
//
// g_source_override can no longer answer this. It says "we have art to show"; since the hold
// window was introduced it stays set through a fall-back, so reading it for the state line would
// report `src=owned` while the player is looking at the ring. That is precisely the kind of
// state line that makes a symptom unreportable.
std::atomic<bool> g_showing_ring{true};

// Set by the game thread when a colour/alpha tunable moves; consumed by the submit thread, which
// owns the staging buffer and the queue.
std::atomic<bool> g_regen{false};

// Liveness. Set by the hook, consumed and cleared by the game-thread watchdog.
std::atomic<uint32_t> g_layers_submitted{0};
// COHERENCE REJECTIONS, and the window that produces a visibly wrong marker.
//
// slot_cell_coherent() is the ONLY thing between a re-let slot and one or two frames of the
// PREVIOUS navpoint's art drawn at the NEW navpoint's pose -- the "flicker of a wrong navpoint"
// report. It had NO telemetry. The census `staleart` field counts the PLUGIN-side guard, which
// fires on a different condition (a re-host that did not complete), so a reader seeing staleart=0
// would reasonably conclude this window was covered when nothing had ever measured it.
//
// cohskip counts rejections. cohdrawn counts the opposite and is the one that matters: slots
// appended within 2 ticks of their own re-host, i.e. frames where wrong art COULD have reached the
// compositor. cohskip climbing with cohdrawn at 0 is the guard working; cohdrawn climbing is the
// bug, measured rather than inferred.
// WHY WAS THIS SLOT NOT SUBMITTED? Every gate below drops a slot with a bare `continue` and no
// record, so "the quad is never submitted" has been an unanswerable question -- exactly the gap the
// grab-guide hunt ran into from the other side. One byte per slot, written on the submit thread,
// read by the game thread's state line.
//   0 not dropped   1 !valid (compute_pose)   2 pose stale (>8 ticks)   3 no atlas cell
//   4 capture stale (freshness)   5 art incoherent after re-host   6 shed by the layer budget
std::atomic<uint8_t>  g_slot_drop[XRLAYER_SLOTS]{};
// LAST VALUE IS NOT ENOUGH, and the grab-guide hunt is why.
//
// The state line samples every 64 ticks. A slot that publishes BRIEFLY -- the guide appears for
// a fraction of a second while a hand crosses a threshold -- is almost never caught by that
// sample, so the line read 10:nopose forever and I reported 'never admitted' when the truth was
// 'admitted twice, between samples'. The teammate's edge-triggered length log had the mirror-
// image flaw: it could only emit values under its own threshold. Two instruments, one failure --
// SAMPLING AN EVENT INSTEAD OF COUNTING IT.
//
// cand counts ticks where the slot HAD a pose and was therefore a real candidate; drop counts
// ticks where it was then rejected. cand=0 means it never published at all and the slot is
// simply idle -- which is why idle slots are now omitted from the line instead of reading as
// 'nopose' and crying wolf on every unused slot.
std::atomic<uint32_t> g_slot_cand[XRLAYER_SLOTS]{};
std::atomic<uint32_t> g_slot_dropn[XRLAYER_SLOTS]{};
// PER-GATE COUNTS, because "which gate" is the question and `last` cannot answer it.
//
// g_slot_drop holds only the MOST RECENT reason, sampled once every 64 ticks by the state
// line. That is the same sample-an-event-instead-of-counting-it flaw already fixed once in
// this file: slot 9 showed cand=5960 drop=949 last=nopose, and `nopose` was simply whatever
// the last tick happened to be -- it said nothing about the 949. Index = the drop code.
std::atomic<uint32_t> g_slot_gate[XRLAYER_SLOTS][7]{};
const char* drop_name(uint8_t d) {
    switch (d) {
        case 0: return "ok";     case 1: return "nopose";  case 2: return "posestale";
        case 3: return "nocell"; case 4: return "capstale"; case 5: return "incoherent";
        case 6: return "budget"; default: return "?";
    }
}

std::atomic<uint32_t> g_coh_skips{0};
std::atomic<uint32_t> g_coh_drawn{0};
std::atomic<bool>     g_live{false};

// Config mirror, so the submit thread never reads g_cfg (which the config poll rewrites under it).
//
// SEPARATE ATOMICS, NOT std::atomic<struct>. A 24-byte atomic is not lock-free on any target we
// build for: MSVC implements it with a spinlock, which would put a lock on the submit thread on
// every frame -- the exact thing this mirror exists to avoid. Each field is independently atomic
// and a torn combination is harmless (one frame drawn with last frame's size).
struct Mirror {
    bool  enabled   = false;
    int   space     = 0;
    float size_m    = 0.06f;
    float cm_per_m  = 0.0f;    // 0 = derive from VR_WorldScale
    float alpha     = 1.0f;
    float cr = 0.35f, cg = 0.95f, cb = 1.0f;
    bool  verbose   = false;
    // FORCED LAYER BUDGET. 0 = use what the runtime reported. Anything else overrides it, and its
    // whole reason for existing is that a fallback nobody can force is a fallback nobody has
    // tested: `skips=0` in the state line means the capture's "previous still in flight" rung has
    // never once executed in this feature's life. Setting xrlayerbudget to 2 or 3 makes the DROP
    // ORDERING run on a healthy machine, which is the rung that would otherwise rot unobserved.
    // Budget 0 is the early-out, not the interesting case.
    int   budget    = 0;
    int   scope_ret      = 0;      // draw a reticule quad over the scope pane
    float scope_ret_size = 0.12f;  // as a fraction of the pane quad
    float scope_ret_depth_cm = 1.0f; // in FRONT of the pane's surface, cm (see Config.hpp)
    int   scope_ret_roll     = 1;    // mode 2's projection frame; see Config.hpp
    float cap_fov_deg    = 4.375f; // the capture's own FOV: scope_base_fov / scope_zoom
    float cap_dist_cm    = 90.0f;  // how far along the ray the capture sits (scope_cam_dist)
};

std::atomic<bool>  g_m_enabled{false};
std::atomic<int>   g_m_budget{0};
// Scope-pane reticule, mirrored for the submit thread for the same reason as the budget: the
// hooked xrEndFrame must never read g_cfg. Relaxed -- a torn read is one frame of a slightly
// different size, and the feature is off by default.
std::atomic<int>   g_m_scope_ret{0};
std::atomic<float> g_m_scope_ret_size{0.12f};
std::atomic<float> g_m_scope_ret_depth{1.0f};   // cm; converted with g_cm_per_m at use
std::atomic<int>   g_m_scope_ret_roll{1};
// WHERE the scope reticule sits in the pane, normalised -1..1. Written on the game thread by
// xrlayer_set_scope_reticle_offset, read on the submit thread when the quad is built.
std::atomic<float> g_scope_ret_u{0.0f};
std::atomic<float> g_scope_ret_v{0.0f};
std::atomic<bool>  g_scope_ret_off_have{false};
// The CAPTURE CAMERA'S world right/up, for mode 2's projection. UE world vectors, written on the
// game thread. Relaxed: a torn read is one frame of a slightly stale basis on a reticule offset.
std::atomic<float> g_scope_cam_rx{0.0f}, g_scope_cam_ry{1.0f}, g_scope_cam_rz{0.0f};
std::atomic<float> g_scope_cam_ux{0.0f}, g_scope_cam_uy{0.0f}, g_scope_cam_uz{1.0f};
std::atomic<bool>  g_scope_cam_axes{false};

// ---- RIG-TRACKED ORIENTATION ------------------------------------------------------------------
// g_rig_* is written on the RENDER thread every frame; g_slot_rig_* on the GAME thread with the
// orientation it belongs to. The submit thread rotates the stored vectors by the difference, which
// is the same correction Plugin.cpp's apply_render makes for the arm meshes.
// BASIS VECTORS, NOT EULER ANGLES -- and that distinction is the whole of a reported bug.
//
// The first version of this published the rig as (pitch, yaw, roll) and rebuilt quaternions from
// them. Scope.cpp already records why that fails, from two earlier attempts at a different problem:
// "an Euler decomposition redistributes between those three as pitch changes -- degenerating
// entirely near vertical", producing "roll that tracked unevenly through a sweep". Reported here as
// the pane going jittery when the controller is ROLLED, which is the same failure.
//
// Three orthonormal vectors have no decomposition and no singularity. Plugin.cpp already holds the
// rig as a quaternion, and Scope.cpp can read the rig's axes directly, so nothing has to be
// decomposed anywhere along this path.
std::atomic<float> g_rig_fx{1.0f}, g_rig_fy{0.0f}, g_rig_fz{0.0f};
std::atomic<float> g_rig_rx{0.0f}, g_rig_ry{1.0f}, g_rig_rz{0.0f};
std::atomic<float> g_rig_ux{0.0f}, g_rig_uy{0.0f}, g_rig_uz{1.0f};
std::atomic<float> g_rig_px{0.0f}, g_rig_py{0.0f}, g_rig_pz{0.0f};
std::atomic<bool>  g_rig_have{false};
// Per-slot: the offset from the rig, IN THE RIG'S OWN BASIS, plus the flag saying to use it.
// Separate from the orientation tracking above because a slot may need one without the other --
// the control arm publishes no orientation at all and still needs its position corrected.
std::atomic<float> g_slot_rp_x[XRLAYER_SLOTS]{}, g_slot_rp_y[XRLAYER_SLOTS]{}, g_slot_rp_z[XRLAYER_SLOTS]{};
std::atomic<bool>  g_slot_rig_pos[XRLAYER_SLOTS]{};
std::atomic<float> g_slot_rig_fx[XRLAYER_SLOTS]{}, g_slot_rig_fy[XRLAYER_SLOTS]{}, g_slot_rig_fz[XRLAYER_SLOTS]{};
std::atomic<float> g_slot_rig_rx[XRLAYER_SLOTS]{}, g_slot_rig_ry[XRLAYER_SLOTS]{}, g_slot_rig_rz[XRLAYER_SLOTS]{};
std::atomic<float> g_slot_rig_ux[XRLAYER_SLOTS]{}, g_slot_rig_uy[XRLAYER_SLOTS]{}, g_slot_rig_uz[XRLAYER_SLOTS]{};
std::atomic<bool>  g_slot_rig_track[XRLAYER_SLOTS]{};
std::atomic<float> g_m_cap_fov{4.375f};
std::atomic<float> g_m_cap_dist{90.0f};
std::atomic<int>   g_m_space{0};
std::atomic<float> g_m_size{0.06f};
std::atomic<float> g_m_cmpm{0.0f};
std::atomic<float> g_m_alpha{1.0f};
std::atomic<float> g_m_cr{0.35f}, g_m_cg{0.95f}, g_m_cb{1.0f};
std::atomic<bool>  g_m_verbose{false};

Mirror mirror_load() {
    Mirror m;
    m.enabled  = g_m_enabled.load(std::memory_order_relaxed);
    m.space    = g_m_space.load(std::memory_order_relaxed);
    m.size_m   = g_m_size.load(std::memory_order_relaxed);
    m.cm_per_m = g_m_cmpm.load(std::memory_order_relaxed);
    m.alpha    = g_m_alpha.load(std::memory_order_relaxed);
    m.cr       = g_m_cr.load(std::memory_order_relaxed);
    m.cg       = g_m_cg.load(std::memory_order_relaxed);
    m.cb       = g_m_cb.load(std::memory_order_relaxed);
    m.verbose  = g_m_verbose.load(std::memory_order_relaxed);
    m.budget   = g_m_budget.load(std::memory_order_relaxed);
    return m;
}

void mirror_store(const Mirror& m) {
    g_m_enabled.store(m.enabled, std::memory_order_relaxed);
    g_m_space.store(m.space, std::memory_order_relaxed);
    g_m_size.store(m.size_m, std::memory_order_relaxed);
    g_m_cmpm.store(m.cm_per_m, std::memory_order_relaxed);
    g_m_alpha.store(m.alpha, std::memory_order_relaxed);
    g_m_cr.store(m.cr, std::memory_order_relaxed);
    g_m_cg.store(m.cg, std::memory_order_relaxed);
    g_m_cb.store(m.cb, std::memory_order_relaxed);
    g_m_verbose.store(m.verbose, std::memory_order_relaxed);
    g_m_budget.store(m.budget, std::memory_order_relaxed);
    g_m_scope_ret.store(m.scope_ret, std::memory_order_relaxed);
    g_m_scope_ret_size.store(m.scope_ret_size, std::memory_order_relaxed);
    g_m_scope_ret_depth.store(m.scope_ret_depth_cm, std::memory_order_relaxed);
    g_m_scope_ret_roll.store(m.scope_ret_roll, std::memory_order_relaxed);
    g_m_cap_fov.store(m.cap_fov_deg, std::memory_order_relaxed);
    g_m_cap_dist.store(m.cap_dist_cm, std::memory_order_relaxed);
}

// ============================================================================================
// The generated reticule bitmap
// ============================================================================================
//
// STAGE 1 ART. A soft ring plus a centre dot, the same shape family as the mesh reticule, drawn
// once on the CPU at whatever colour the config asks for and uploaded once per swapchain image.
//
// This is where the "override the colour data" half of the design lives, and at stage 1 it is
// free: we are generating the pixels, so tint, gain and alpha are simply what we write. There is
// no material, no MID, no pak and no shader. (Stage 2 replaces this with a copy from the widget's
// render target, and the colour override becomes a real pixel shader -- that is stage 2's cost,
// not stage 1's.)
//
// Colour is written PREMULTIPLIED and the layer is submitted without the unpremultiplied flag.
// Getting that pair wrong is the classic dark-halo-around-the-crosshair bug, and it is worth
// noting that the in-scene path has exactly this problem today: Config.hpp records that UMG writes
// its target premultiplied while the pass-through material samples it straight, which darkens
// precisely the semi-transparent pixels a thin antialiased crosshair is mostly made of.

// The generated ring's own size. 128 is plenty for a soft ring and costs 64 KB.
constexpr int RING_DIM = 128;

// Draw the ring into a SUB-RECTANGLE of a larger buffer.
//
// `stride` is the destination's row pitch in BYTES and `out` already points at the cell's top-left
// pixel, so the same generator serves a whole-image bitmap (stride = dim*4) and cell 0 of an atlas
// (stride = atlas_w*4) with no second code path. Everything else is unchanged.
void generate_bitmap(uint8_t* out, int dim, size_t stride, float r, float g, float b, float a,
                     bool bgra) {
    const float c  = (float)dim * 0.5f - 0.5f;
    const float R  = (float)dim * 0.40f;   // ring radius, px
    const float T  = (float)dim * 0.055f;  // ring half-thickness, px
    const float D  = (float)dim * 0.045f;  // centre dot radius, px
    const float AA = 1.25f;                // edge softness, px

    for (int y = 0; y < dim; ++y) {
        for (int x = 0; x < dim; ++x) {
            const float dx = (float)x - c, dy = (float)y - c;
            const float d  = std::sqrt(dx * dx + dy * dy);

            // Ring: 1 inside the band, feathering to 0 across AA on both edges.
            float ring = 1.0f - (std::fabs(d - R) - T) / AA;
            if (ring > 1.0f) ring = 1.0f;
            if (ring < 0.0f) ring = 0.0f;

            float dot = 1.0f - (d - D) / AA;
            if (dot > 1.0f) dot = 1.0f;
            if (dot < 0.0f) dot = 0.0f;

            float cov = ring > dot ? ring : dot;
            cov *= a;

            uint8_t* p = out + (size_t)y * stride + (size_t)x * 4;
            // Premultiplied, in whichever channel order the runtime's chosen format wants.
            const uint8_t R = (uint8_t)(r * cov * 255.0f + 0.5f);
            const uint8_t G = (uint8_t)(g * cov * 255.0f + 0.5f);
            const uint8_t B = (uint8_t)(b * cov * 255.0f + 0.5f);
            p[0] = bgra ? B : R;
            p[1] = G;
            p[2] = bgra ? R : B;
            p[3] = (uint8_t)(cov * 255.0f + 0.5f);
        }
    }
}

// THE GRAB GUIDE'S ART: a feathered capsule, generated once.
//
// Same conventions as generate_bitmap above -- premultiplied, BGRA-aware, alpha carries the shape.
// The quad it lands on is NON-SQUARE (long along the barrel, thin across it), so the capsule is
// authored square and stretched by the extent. Stretching is exactly right for this shape: a
// capsule is a rectangle with semicircular caps, and scaling one axis keeps the caps round-ish
// while the body simply gets longer. A circle would work too -- it is this with body_half = 0.
//
// FEATHERED IN ALPHA, which is the whole point. A hard-edged bar in a compositor layer reads as a
// sticker pasted over the scene, because it has none of the softness everything behind it has. The
// falloff is what makes it look like light rather than geometry, and it costs nothing: it is a
// distance field either way.
//
// LOW RES IS FINE and 32 px is plenty. The thing is a couple of centimetres on screen, it has no
// detail to lose, and the alpha ramp is what the eye reads -- not the resolution. Bigger would only
// cost atlas.
// The beam's art. KEPT although the label is the default: the beam earns its place again the
// moment there is more than one thing to grab (grenades, magazines), because then "you can grip"
// is not enough and WHICH ONE has to be answered. Fully dim-parameterised, so it renders correctly
// in the label-sized 128px cell as well as the 32px one it was written for.
void generate_capsule(uint8_t* out, int dim, size_t stride, float r, float g, float b, float a,
                      bool bgra) {
    const float c         = (float)dim * 0.5f - 0.5f;
    const float radius    = (float)dim * 0.34f;   // cap radius / half-thickness, px
    const float body_half = (float)dim * 0.16f;   // half-length of the straight section, px
    const float AA        = (float)dim * 0.16f;   // feather width, px -- generous on purpose

    for (int y = 0; y < dim; ++y) {
        for (int x = 0; x < dim; ++x) {
            // Distance to a horizontal SEGMENT, which is what makes this a capsule rather than an
            // ellipse: clamp along the body, then measure radially. The caps fall out of the clamp.
            const float dx = (float)x - c, dy = (float)y - c;
            float ax = dx;
            if (ax >  body_half) ax -= body_half;
            else if (ax < -body_half) ax += body_half;
            else ax = 0.0f;
            const float d = std::sqrt(ax * ax + dy * dy);

            // 1 inside, feathering to 0 across AA. Smoothstep rather than linear so the edge has
            // no visible band where the ramp starts.
            float t = 1.0f - (d - (radius - AA)) / AA;
            if (t > 1.0f) t = 1.0f;
            if (t < 0.0f) t = 0.0f;
            float cov = t * t * (3.0f - 2.0f * t);
            cov *= a;

            uint8_t* p = out + (size_t)y * stride + (size_t)x * 4;
            const uint8_t R = (uint8_t)(r * cov * 255.0f + 0.5f);
            const uint8_t G = (uint8_t)(g * cov * 255.0f + 0.5f);
            const uint8_t B = (uint8_t)(b * cov * 255.0f + 0.5f);
            p[0] = bgra ? B : R;
            p[1] = G;
            p[2] = bgra ? R : B;
            p[3] = (uint8_t)(cov * 255.0f + 0.5f);
        }
    }
}

// THE GRAB-GUIDE LABEL: the word "Grip", rendered with a real font, anchored to the off hand.
//
// It replaces the beam (generate_capsule, removed). The beam had to state a DIRECTION, and a
// direction is a claim about which object you are about to grab -- a claim it got visibly wrong.
// A label anchored to the hand makes no such claim: it says "you can grip here, now", which is the
// affordance that was actually wanted, and it cannot point at the wrong thing because it does not
// point. Note what this does and does not fix: the ZONE test still decides WHEN the label appears,
// so a wrong zone now reads as "Grip showing when nothing is grabbable" rather than as bad aim.
//
// WHY GDI. The atlas is CPU-side pixels (see fill_upload), so text has to be rasterised on the CPU.
// GDI is the smallest way to get a real, hinted, antialiased typeface -- the alternative was baking
// a glyph atlas by hand, which is more code and a worse-looking result. It costs one gdi32.lib
// link and runs ONCE at bring-up (and on a colour change), never per frame.
//
// ANTIALIASED_QUALITY, NOT CLEARTYPE, and this is not a style preference. ClearType is subpixel
// antialiasing: it deliberately puts DIFFERENT coverage in R, G and B to exploit the physical
// stripe order of a desktop LCD. We read the luminance back as an ALPHA MASK, so per-channel
// coverage would come back as coloured fringing on the glyph edges -- and on an HMD's optics,
// aimed at a subpixel layout that is not there, it would be wrong even if we did not.
void generate_label(uint8_t* out, int dim, size_t stride, float r, float g, float b, float a,
                    bool bgra) {
    // GDI gives us no alpha: DrawText writes RGB and leaves the fourth byte at zero. So render
    // WHITE ON BLACK and read the result back as coverage. That is why the bitmap is cleared to
    // black and the text colour is pure white -- the luminance IS the glyph's alpha.
    // Array, not a pointer, so the length comes from sizeof and this needs no <cwchar> for wcslen.
    static const wchar_t kText[]  = L"Grip";
    constexpr int        kTextLen = (int)(sizeof(kText) / sizeof(kText[0])) - 1;

    HDC dc = CreateCompatibleDC(nullptr);
    if (dc == nullptr) return;

    BITMAPINFO bi{};
    bi.bmiHeader.biSize        = sizeof(BITMAPINFOHEADER);
    bi.bmiHeader.biWidth       = dim;
    bi.bmiHeader.biHeight      = -dim;          // negative = TOP-DOWN, matching our atlas rows
    bi.bmiHeader.biPlanes      = 1;
    bi.bmiHeader.biBitCount    = 32;
    bi.bmiHeader.biCompression = BI_RGB;

    void*   bits = nullptr;
    HBITMAP bmp  = CreateDIBSection(dc, &bi, DIB_RGB_COLORS, &bits, nullptr, 0);
    if (bmp == nullptr || bits == nullptr) { DeleteDC(dc); return; }
    ZeroMemory(bits, (size_t)dim * (size_t)dim * 4);

    HGDIOBJ old_bmp = SelectObject(dc, bmp);

    // MEASURE, THEN FIT. "Grip" at a guessed point size either clips or swims in whitespace, and
    // every wrong guess costs a full rebuild to see. So pick a height, ask GDI how wide that
    // actually is in the chosen face, and scale once to fit the cell with a small margin. One
    // correction is enough -- text width is very nearly linear in font height.
    auto make_font = [](int h) {
        return CreateFontW(-h, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
                           OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY,
                           DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
    };

    const int  margin_px = (int)((float)dim * 0.10f);
    const int  avail     = dim - 2 * margin_px;
    int        height    = (int)((float)dim * 0.46f);
    HFONT      font      = make_font(height);
    // A NULL font is not fatal: GDI then draws in the DC's default face, which is ugly but legible.
    // Failing closed here would trade a cosmetic problem for a missing affordance.
    HGDIOBJ    old_font  = (font != nullptr) ? SelectObject(dc, font) : nullptr;

    SIZE ext{};
    if (GetTextExtentPoint32W(dc, kText, kTextLen, &ext) && ext.cx > avail && ext.cx > 0) {
        const int shrunk = (int)((float)height * ((float)avail / (float)ext.cx));
        if (HFONT f2 = make_font(shrunk > 6 ? shrunk : 6)) {
            SelectObject(dc, f2);
            if (font != nullptr) DeleteObject(font);
            font = f2;
        }
    }

    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, RGB(255, 255, 255));
    RECT rc{0, 0, dim, dim};
    DrawTextW(dc, kText, -1, &rc, DT_CENTER | DT_VCENTER | DT_SINGLELINE | DT_NOCLIP);

    // GDI flushes lazily; the batch must be drained before the DIB's bits are read on the CPU.
    GdiFlush();

    const uint8_t* srcpx = (const uint8_t*)bits;
    for (int y = 0; y < dim; ++y) {
        for (int x = 0; x < dim; ++x) {
            const uint8_t* sp = srcpx + ((size_t)y * (size_t)dim + (size_t)x) * 4;
            // DIB is BGRX. White-on-black means all three channels carry the same coverage; take
            // the max so a substituted face that happens to render with any tint still reads.
            const uint8_t mx  = sp[0] > sp[1] ? (sp[0] > sp[2] ? sp[0] : sp[2])
                                              : (sp[1] > sp[2] ? sp[1] : sp[2]);
            const float   cov = ((float)mx / 255.0f) * a;

            uint8_t* p = out + (size_t)y * stride + (size_t)x * 4;
            const uint8_t R = (uint8_t)(r * cov * 255.0f + 0.5f);
            const uint8_t G = (uint8_t)(g * cov * 255.0f + 0.5f);
            const uint8_t B = (uint8_t)(b * cov * 255.0f + 0.5f);
            p[0] = bgra ? B : R;
            p[1] = G;
            p[2] = bgra ? R : B;
            p[3] = (uint8_t)(cov * 255.0f + 0.5f);
        }
    }

    if (old_font != nullptr) SelectObject(dc, old_font);
    if (font     != nullptr) DeleteObject(font);
    SelectObject(dc, old_bmp);
    DeleteObject(bmp);
    DeleteDC(dc);
}


// ============================================================================================
// D3D12 plumbing
// ============================================================================================

void release_d3d() {
    // UNPUBLISH BEFORE RELEASING. The submit thread reads g_source_override every frame and this
    // runs on the game thread, so anything released while that pointer is still visible is a
    // use-after-free of exactly the kind this rework exists to remove.
    g_source_override.store(nullptr, std::memory_order_release);
    for (int i = 0; i < XRLAYER_SLOTS; ++i) {
        g_capture_src[i] = nullptr;
        g_slot_beat[i].store(0, std::memory_order_relaxed);
        // Clear the coherence clocks with the beats they travel with -- a rebuilt atlas has no
        // captures, so a stale host/cap pair must not read coherent against the fresh cells.
        g_slot_host_tick[i].store(0, std::memory_order_relaxed);
        g_slot_cap_tick[i].store(0, std::memory_order_relaxed);
    }
    g_src_beat.store(0, std::memory_order_relaxed);

    if (g_fence_ev != nullptr) { CloseHandle(g_fence_ev); g_fence_ev = nullptr; }
    if (g_fence  != nullptr) { g_fence->Release();  g_fence  = nullptr; }
    if (g_list   != nullptr) { g_list->Release();   g_list   = nullptr; }
    for (int i = 0; i < FRAME_RING; ++i) {
        if (g_alloc[i] != nullptr) { g_alloc[i]->Release(); g_alloc[i] = nullptr; }
        g_alloc_fence[i] = 0;
    }
    g_ring = 0;
    if (g_upload != nullptr) { g_upload->Release(); g_upload = nullptr; }
    g_fence_v = 0;

    if (g_gt_list  != nullptr) { g_gt_list->Release();  g_gt_list  = nullptr; }
    if (g_gt_fence != nullptr) { g_gt_fence->Release(); g_gt_fence = nullptr; }
    for (int i = 0; i < GT_RING; ++i) {
        if (g_gt_alloc[i] != nullptr) { g_gt_alloc[i]->Release(); g_gt_alloc[i] = nullptr; }
        g_gt_alloc_fence[i] = 0;
    }
    g_gt_ring = 0;
    g_gt_fence_v = 0;
    g_gt_captures = 0;
    g_gt_skips = 0;
    g_gt_open_slot = -1;
    g_gt_recorded  = 0;
    g_gt_rec_mask  = 0;
    if (g_owned != nullptr) { g_owned->Release(); g_owned = nullptr; }
}

// Re-generate the bitmap into the staging buffer at D3D12's 256-byte row-pitch alignment.
//
// Safe to call on the submit thread mid-session: every blit fence-waits before it returns, so no
// copy is ever in flight when this runs. That is the only reason a plain Map/write/Unmap is
// sufficient here rather than a second buffer.
// THE STAGING BUFFER HOLDS A WHOLE ATLAS, not just the ring.
//
// Cell 0 gets the generated ring; every other cell is left at zero, i.e. fully transparent and
// PREMULTIPLIED-correct. That matters for more than tidiness: it is what makes the atlas safe to
// present before any slot has captured anything -- a marker cell that has never been written shows
// nothing rather than whatever the allocator handed us, and a slot with no art is simply never
// appended anyway.
bool fill_upload(float r, float g, float b, float a) {
    if (g_upload == nullptr || g_sc_w <= 0 || g_sc_h <= 0) return false;

    const UINT row_pitch = (g_sc_w * 4 + D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1) &
                           ~(UINT)(D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1);

    std::vector<uint8_t> tex((size_t)g_sc_w * g_sc_h * 4, 0);
    const Cell& c0 = g_cell[XRLAYER_SLOT_RETICULE];
    if (c0.dim > 0) {
        generate_bitmap(tex.data() + ((size_t)c0.y * g_sc_w + c0.x) * 4, c0.dim,
                        (size_t)g_sc_w * 4, r, g, b, a, g_is_bgra);
    }
    // THE GUIDE'S CELL, filled from the same staging pass. It is a generated shape like the
    // fallback ring, not captured art, so it belongs here rather than anywhere near XrSource --
    // no widget, no render target, no capture, nothing to re-resolve. Filled ONCE with the rest of
    // the staging buffer and then simply present forever.
    const Cell& cg = g_cell[XRLAYER_SLOT_GUIDE];
    if (cg.dim > 0) {
        // ONE CELL, TWO POSSIBLE PICTURES. grabguidemode picks which; changing it live sets
        // g_regen (see the config poll), which is what re-runs this pass and swaps the art. Without
        // that the key would change the QUAD's shape and leave the old picture in the cell -- a
        // tunable that half-works, which is harder to diagnose than one that does nothing at all.
        uint8_t* const dstpx = tex.data() + ((size_t)cg.y * g_sc_w + cg.x) * 4;
        if (g_cfg.grab_guide_mode == 1) {
            generate_capsule(dstpx, cg.dim, (size_t)g_sc_w * 4,
                             g_cfg.grab_guide_r, g_cfg.grab_guide_g, g_cfg.grab_guide_b, 1.0f,
                             g_is_bgra);
        } else {
            generate_label(dstpx, cg.dim, (size_t)g_sc_w * 4,
                           g_cfg.grab_guide_r, g_cfg.grab_guide_g, g_cfg.grab_guide_b, 1.0f,
                           g_is_bgra);
        }
    }

    void* mapped = nullptr;
    D3D12_RANGE none{0, 0};
    if (FAILED(g_upload->Map(0, &none, &mapped)) || mapped == nullptr) {
        logf("upload buffer Map FAILED");
        return false;
    }
    for (int y = 0; y < g_sc_h; ++y) {
        memcpy((uint8_t*)mapped + (size_t)y * row_pitch,
               tex.data() + (size_t)y * g_sc_w * 4,
               (size_t)g_sc_w * 4);
    }
    g_upload->Unmap(0, nullptr);
    return true;
}

// Build the staging buffer, the command list and the fence, and fill the staging buffer once.
bool create_d3d_resources(float r, float g, float b, float a) {
    if (g_device == nullptr) return false;

    const UINT row_pitch = (g_sc_w * 4 + D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1) &
                           ~(UINT)(D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1);
    const UINT64 total = (UINT64)row_pitch * g_sc_h;

    D3D12_HEAP_PROPERTIES hp{};
    hp.Type = D3D12_HEAP_TYPE_UPLOAD;

    D3D12_RESOURCE_DESC rd{};
    rd.Dimension        = D3D12_RESOURCE_DIMENSION_BUFFER;
    rd.Width            = total;
    rd.Height           = 1;
    rd.DepthOrArraySize = 1;
    rd.MipLevels        = 1;
    rd.Format           = DXGI_FORMAT_UNKNOWN;
    rd.SampleDesc.Count = 1;
    rd.Layout           = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

    if (FAILED(g_device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd,
                                                 D3D12_RESOURCE_STATE_GENERIC_READ, nullptr,
                                                 IID_PPV_ARGS(&g_upload)))) {
        logf("CreateCommittedResource(upload) FAILED");
        return false;
    }

    if (!fill_upload(r, g, b, a)) return false;

    for (int i = 0; i < FRAME_RING; ++i) {
        if (FAILED(g_device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT,
                                                    IID_PPV_ARGS(&g_alloc[i])))) {
            logf("CreateCommandAllocator[%d] FAILED", i);
            return false;
        }
        g_alloc_fence[i] = 0;
    }
    g_ring = 0;
    if (FAILED(g_device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, g_alloc[0], nullptr,
                                           IID_PPV_ARGS(&g_list)))) {
        logf("CreateCommandList FAILED");
        return false;
    }
    g_list->Close();

    if (FAILED(g_device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&g_fence)))) {
        logf("CreateFence FAILED");
        return false;
    }
    g_fence_ev = CreateEventW(nullptr, FALSE, FALSE, nullptr);
    if (g_fence_ev == nullptr) {
        logf("CreateEvent FAILED");
        return false;
    }

    // ---- the texture WE own, and the game thread's copy machinery -----------------------------
    //
    // Same edge and same typeless family as the swapchain, so BOTH copies are legal by
    // construction: engine RT -> g_owned (checked in xrlayer_set_source) and g_owned -> swapchain
    // image. UNORM rather than the runtime's possibly-_SRGB member -- the two share a typeless
    // parent, so the copy is valid either way, and matching what the widget's target actually is
    // keeps the colour decision in exactly one place (collect_formats).
    //
    // CREATED IN COPY_SOURCE AND LEFT THERE FOREVER. The submit thread therefore issues no barrier
    // on it at all; the capture path transitions it to COPY_DEST and straight back within one
    // command list. A resource whose state the submit thread never changes is a resource the submit
    // thread cannot be wrong about.
    {
        D3D12_HEAP_PROPERTIES hp2{};
        hp2.Type = D3D12_HEAP_TYPE_DEFAULT;

        D3D12_RESOURCE_DESC td{};
        td.Dimension        = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        td.Width            = (UINT64)g_sc_w;
        td.Height           = (UINT)g_sc_h;
        td.DepthOrArraySize = 1;
        td.MipLevels        = 1;
        td.Format           = g_is_bgra ? DXGI_FORMAT_B8G8R8A8_UNORM : DXGI_FORMAT_R8G8B8A8_UNORM;
        td.SampleDesc.Count = 1;
        td.Layout           = D3D12_TEXTURE_LAYOUT_UNKNOWN;
        // UAV ONLY WHEN THE MASK IS ON. scopemask writes this texture's alpha with a compute
        // shader, which needs ALLOW_UNORDERED_ACCESS -- and that is a CREATION-TIME decision, so it
        // is taken here rather than lazily. With the mask off the atlas is created byte-identically
        // to before: an unconfigured build carries none of it, and the flag cannot affect anyone
        // who has not asked for the feature. This texture has crash history around its lifetime,
        // so its creation is deliberately the one place this decision is made.
        const bool want_uav = scopemask_wanted();
        td.Flags            = want_uav ? D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS
                                       : D3D12_RESOURCE_FLAG_NONE;

        if (FAILED(g_device->CreateCommittedResource(&hp2, D3D12_HEAP_FLAG_NONE, &td,
                                                     D3D12_RESOURCE_STATE_COPY_SOURCE, nullptr,
                                                     IID_PPV_ARGS(&g_owned)))) {
            logf("CreateCommittedResource(owned atlas %dx%d) FAILED -- stage 2 unavailable, the "
                 "generated ring keeps drawing", g_sc_w, g_sc_h);
            return false;
        }
        // Tell the mask what this atlas can actually DO. Without it the apply path cannot tell a
        // UAV-capable atlas from one built before scopemask was set, and creating a UAV on the
        // latter removes the device -- which is exactly what a live scopemask=1 did on 2026-09-07.
        scopemask_note_atlas(want_uav);
    }

    for (int i = 0; i < GT_RING; ++i) {
        if (FAILED(g_device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT,
                                                    IID_PPV_ARGS(&g_gt_alloc[i])))) {
            logf("CreateCommandAllocator(capture %d) FAILED", i);
            return false;
        }
        g_gt_alloc_fence[i] = 0;
    }
    g_gt_ring = 0;
    if (FAILED(g_device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, g_gt_alloc[0],
                                           nullptr, IID_PPV_ARGS(&g_gt_list)))) {
        logf("CreateCommandList(capture) FAILED");
        return false;
    }
    g_gt_list->Close();
    if (FAILED(g_device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&g_gt_fence)))) {
        logf("CreateFence(capture) FAILED");
        return false;
    }
    g_gt_fence_v = 0;
    return true;
}

// Copy the staging buffer (or the stage-2 source) into one swapchain image and wait for it.
//
// Submitted on UEVR'S OWN QUEUE, which is what makes this safe: the OpenXR runtime synchronises
// against the queue it was handed in the graphics binding, so work we submit there is ordered
// ahead of the runtime's read without any cross-queue fence of our own. Submitting on a private
// queue would compile, run, and tear.
// THE STATE THE ENGINE LEAVES ITS COLOUR TARGETS IN.
//
// A copy needs the source in COPY_SOURCE, and a barrier that names the wrong StateBefore is not an
// error you get told about -- it is a debug-layer complaint on a machine that has the debug layer,
// and a decompression the driver skips on one that does not. So this is an ASSUMPTION and it is
// named as one: it is the same pair UEVR itself assumes for engine-owned colour textures
// (ENGINE_SRC_COLOR in D3D12Component.cpp), which is the closest thing to a measurement available
// without instrumenting the game's own RHI. If the widget's crosshair ever presents as garbage
// rather than as nothing, this is the first thing to doubt.
constexpr D3D12_RESOURCE_STATES ENGINE_SRC_COLOR =
    (D3D12_RESOURCE_STATES)(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE |
                            D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);

// Fill one swapchain image.
//
//   src         the atlas the GAME THREAD captured real widget art into, or nullptr before any
//               capture has happened.
//   ring_cell0  additionally lay the GENERATED ring back over cell 0. Only meaningful with a
//               non-null src: it is the reticule's stale-capture fall-back, and it is per-CELL now
//               rather than per-image, so a stale reticule no longer takes the markers with it.
//
// src == nullptr is the pre-capture state: the whole generated atlas (ring in cell 0, transparent
// everywhere else) copied straight out of the staging buffer.
//
// THE POINTER IS PASSED IN, NOT READ FROM g_owned. It must be the value the caller loaded from
// g_source_override, because that atomic IS the publication protocol: release_d3d() (game thread)
// stores nullptr through it BEFORE releasing the resource, so a submit thread that only ever
// follows the published pointer cannot be handed a released one. Reading the raw g_owned global
// here instead would quietly reintroduce exactly that race.
// GUIDE_CELL: lay the generated grab-guide capsule back over a CAPTURED atlas, same idea as
// ring_cell0 and for the same reason -- it is the fix for "the guide is submitted and invisible".
//
// The guide's pixels are GENERATED once into the staging buffer by fill_upload(). That is fine
// while nothing is being captured, because then the whole generated atlas is copied. The moment
// real widget art arrives, the `src != nullptr` path below does CopyResource(dst, src) -- the WHOLE
// captured atlas over the WHOLE image, every frame -- and nothing ever captures into the guide's
// cell at (g_ret_dim, 0). So the capsule is wiped on the first captured frame and never restored:
// slot 10 keeps being posed, admitted and submitted (measured: cand=370, 277 admitted) while
// sampling a rectangle that has been blanked to transparent.
//
// Every upstream instrument said "healthy", because every upstream stage WAS healthy. The one
// question none of them asked is whether the pixels the quad points at survived the frame.
bool blit_into(ID3D12Resource* dst, ID3D12Resource* src, bool ring_cell0, bool guide_cell) {
    if (dst == nullptr || g_list == nullptr || g_queue == nullptr) return false;

    // Pick this frame's allocator and wait ONLY if the GPU has not finished what that allocator
    // last recorded. Resetting one whose commands are in flight is undefined behaviour, so the
    // wait cannot be deleted -- it can only be made almost never taken.
    const int slot = g_ring;
    g_ring = (g_ring + 1) % FRAME_RING;
    ID3D12CommandAllocator* alloc = g_alloc[slot];
    if (alloc == nullptr) return false;
    //
    // A TIMEOUT IS A FAILURE, NOT A GO-AHEAD. This used to discard WaitForSingleObject's return and
    // fall through, which reset an allocator whose commands were still in flight (undefined
    // behaviour) AND -- back when the source was the engine's own resource -- issued a barrier
    // against a pointer whose freshness had been checked a full second earlier. That check-then-use
    // gap is the measured cause of the third crash on 2026-08-23. Skipping the frame costs one
    // stale overlay image; proceeding cost the process.
    if (g_alloc_fence[slot] != 0 && g_fence->GetCompletedValue() < g_alloc_fence[slot]) {
        if (FAILED(g_fence->SetEventOnCompletion(g_alloc_fence[slot], g_fence_ev))) return false;
        if (WaitForSingleObject(g_fence_ev, 1000) != WAIT_OBJECT_0) {
            static uint32_t said = 0;
            if (said < 5) {
                ++said;
                logf("frame-ring fence did not signal within 1 s -- skipping this overlay frame "
                     "rather than resetting an allocator whose commands are still in flight.");
            }
            return false;
        }
    }

    if (FAILED(alloc->Reset())) return false;
    if (FAILED(g_list->Reset(alloc, nullptr))) return false;

    D3D12_RESOURCE_BARRIER to_dst{};
    to_dst.Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    to_dst.Transition.pResource   = dst;
    to_dst.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    to_dst.Transition.StateBefore = D3D12_RESOURCE_STATE_COMMON;
    to_dst.Transition.StateAfter  = D3D12_RESOURCE_STATE_COPY_DEST;
    g_list->ResourceBarrier(1, &to_dst);

    if (src != nullptr) {
        // STAGE 2 path. `src` is ALWAYS g_owned -- a resource we created, that lives for the life
        // of the layer, and that sits permanently in COPY_SOURCE. So there is no barrier here at
        // all, and that absence is the fix rather than an omission:
        //
        //   The three 2026-08-23 crashes were ALL this call site, transitioning the ENGINE's widget
        //   render target from ENGINE_SRC_COLOR to COPY_SOURCE. No amount of checking made that
        //   safe, because the question ("does this resource still have GPU backing?") is one the
        //   submit thread cannot ask. The copy IN now happens on the game thread, in
        //   xrlayer_capture_source(); see the g_owned comment at the top of this file.
        //
        // Do not "restore" a source barrier here. If you find yourself needing one, the source is
        // no longer ours, and that is the bug.
        g_list->CopyResource(dst, src);
    }

    // The staging-buffer path: either the WHOLE generated atlas (nothing captured yet), or the
    // GENERATED cells laid back over a captured atlas -- cell 0 for the reticule's stale fall-back,
    // and the guide's cell whenever the guide is being drawn.
    if (src == nullptr || ring_cell0 || guide_cell) {
        const UINT row_pitch = (g_sc_w * 4 + D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1) &
                               ~(UINT)(D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1);

        D3D12_TEXTURE_COPY_LOCATION s{};
        s.pResource                          = g_upload;
        s.Type                               = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
        s.PlacedFootprint.Offset             = 0;
        // The UNORM member of the destination's own family. Copying across families is invalid,
        // and UNORM -> UNORM_SRGB within a family is allowed (shared typeless parent).
        s.PlacedFootprint.Footprint.Format   = g_is_bgra ? DXGI_FORMAT_B8G8R8A8_UNORM
                                                         : DXGI_FORMAT_R8G8B8A8_UNORM;
        s.PlacedFootprint.Footprint.Width    = (UINT)g_sc_w;
        s.PlacedFootprint.Footprint.Height   = (UINT)g_sc_h;
        s.PlacedFootprint.Footprint.Depth    = 1;
        s.PlacedFootprint.Footprint.RowPitch = row_pitch;

        D3D12_TEXTURE_COPY_LOCATION dl{};
        dl.pResource        = dst;
        dl.Type             = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
        dl.SubresourceIndex = 0;

        if (src == nullptr) {
            g_list->CopyTextureRegion(&dl, 0, 0, 0, &s, nullptr);
        } else {
            // ONE CELL AT A TIME, EACH BEHIND ITS OWN GATE. This branch used to copy cell 0
            // unconditionally, which was safe only because ring_cell0 was the single way in.
            // Now that guide_cell can also open it, an ungated cell-0 copy would lay the STALE
            // GENERATED RING over perfectly good captured reticule art every frame the guide is
            // up -- turning a fix for one slot into a regression on the slot that already worked.
            //
            // The source box is in the full-atlas footprint's own coordinates, which for these
            // cells is their rectangle unchanged.
            auto lay_back = [&](const Cell& c) {
                if (c.dim <= 0) return;
                D3D12_BOX box{};
                box.left   = (UINT)c.x;
                box.top    = (UINT)c.y;
                box.front  = 0;
                box.right  = (UINT)(c.x + c.dim);
                box.bottom = (UINT)(c.y + c.dim);
                box.back   = 1;
                g_list->CopyTextureRegion(&dl, (UINT)c.x, (UINT)c.y, 0, &s, &box);
            };

            if (ring_cell0)  lay_back(g_cell[XRLAYER_SLOT_RETICULE]);
            // 32x32 -- 4 KB, and only on frames the guide is actually drawn. The per-frame cost of
            // NOT doing this was an invisible feature, which is the more expensive of the two.
            if (guide_cell)  lay_back(g_cell[XRLAYER_SLOT_GUIDE]);
        }
    }

    D3D12_RESOURCE_BARRIER back = to_dst;
    back.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
    back.Transition.StateAfter  = D3D12_RESOURCE_STATE_COMMON;
    g_list->ResourceBarrier(1, &back);

    if (FAILED(g_list->Close())) return false;

    ID3D12CommandList* lists[] = {g_list};
    g_queue->ExecuteCommandLists(1, lists);

    ++g_fence_v;
    if (FAILED(g_queue->Signal(g_fence, g_fence_v))) return false;
    g_alloc_fence[slot] = g_fence_v;

    // WAIT ONLY FOR THE UPLOAD PATH. There the CPU rewrites g_upload in place on a colour change,
    // so the copy has to be finished before the next fill_upload() maps it -- and that path runs
    // once per image, not once per frame.
    //
    // The stage-2 path deliberately does NOT wait. It is GPU-to-GPU on the queue the OpenXR runtime
    // was handed in its graphics binding, so the runtime's read is already ordered behind our copy
    // by the queue itself; there is nothing a CPU wait would add except a stall on the submit
    // thread of every single frame.
    if (src == nullptr) {
        if (g_fence->GetCompletedValue() < g_fence_v) {
            if (FAILED(g_fence->SetEventOnCompletion(g_fence_v, g_fence_ev))) return false;
            WaitForSingleObject(g_fence_ev, 1000);
        }
    }
    return true;
}

// ============================================================================================
// Swapchain lifecycle
// ============================================================================================

void destroy_swapchain() {
    if (g_swapchain != XR_NULL_HANDLE && g_xr.destroy_swapchain != nullptr) {
        g_xr.destroy_swapchain(g_swapchain);
    }
    g_swapchain = XR_NULL_HANDLE;
    g_images.clear();
    g_image_dirty.clear();
    g_sc_w = g_sc_h = 0;
}

// Collect the BGRA/RGBA8 formats the runtime offers, in preference order.
//
// Never assume one: a format the runtime does not advertise is a hard xrCreateSwapchain failure,
// and the sRGB-vs-linear choice is the difference between the right colour and a washed-out one --
// which would look exactly like the brightness bug this whole module exists to fix.
//
// Returns candidates rather than a single pick, because the FORMAT is not the only thing a runtime
// can refuse (see create_swapchain).
size_t collect_formats(int64_t* out, size_t cap) {
    uint32_t count = 0;
    if (XR_FAILED(g_xr.enumerate_formats(g_session, 0, &count, nullptr)) || count == 0) return 0;

    std::vector<int64_t> formats(count);
    if (XR_FAILED(g_xr.enumerate_formats(g_session, count, &count, formats.data()))) return 0;

    // sRGB first: the runtime then does the linear->sRGB conversion on read, and our bitmap is
    // authored in sRGB-ish 0..1, which is what makes the colour come out as asked.
    const int64_t want_ring[] = {
        (int64_t)DXGI_FORMAT_B8G8R8A8_UNORM_SRGB,
        (int64_t)DXGI_FORMAT_R8G8B8A8_UNORM_SRGB,
        (int64_t)DXGI_FORMAT_B8G8R8A8_UNORM,
        (int64_t)DXGI_FORMAT_R8G8B8A8_UNORM,
    };
    // STAGE 2 FLIPS THE PREFERENCE, and this is an UNMEASURED choice stated as one.
    //
    // The pixels then come from UMG's own render target rather than from us. A UWidgetComponent
    // allocates PF_B8G8R8A8, and Slate writes values already encoded for display; presenting them
    // through an _SRGB view asks the runtime to decode them a second time, which reads as a
    // washed-out crosshair rather than as an error. Both members share a typeless parent so either
    // is a LEGAL CopyResource destination -- only the colour differs, which is why this can be
    // settled by one look at a composited capture and does not deserve a config key yet.
    const int64_t want_src[] = {
        (int64_t)DXGI_FORMAT_B8G8R8A8_UNORM,
        (int64_t)DXGI_FORMAT_R8G8B8A8_UNORM,
        (int64_t)DXGI_FORMAT_B8G8R8A8_UNORM_SRGB,
        (int64_t)DXGI_FORMAT_R8G8B8A8_UNORM_SRGB,
    };
    const int64_t* want = g_cfg.xr_layer_src ? want_src : want_ring;
    size_t n = 0;
    for (size_t wi = 0; wi < 4; ++wi) {
        for (int64_t f : formats) {
            if (f == want[wi] && n < cap) { out[n++] = f; break; }
        }
    }
    if (n == 0) logf("no BGRA/RGBA8 swapchain format among the %u the runtime offers", count);
    return n;
}

// Create the swapchain, trying every plausible (format, usage) pair rather than one shot.
//
// The first version tried exactly one combination and latched the whole module off when the runtime
// answered XR_ERROR_RUNTIME_FAILURE (-2) -- a generic code that says "no" and nothing else. One
// attempt plus a generic error is not a diagnosis: it cannot distinguish an unacceptable format
// from unacceptable usage flags, and it turns a one-line fix into another headset session.
//
// USAGE FLAGS ARE THE LIKELIER CULPRIT of the two. Runtimes generally allocate swapchain images as
// render targets, and some validate that COLOR_ATTACHMENT is present even for an overlay that is
// only ever a copy destination. Ours asked for TRANSFER_DST|SAMPLED and nothing else.
bool create_swapchain() {
    int64_t formats[4]{};
    const size_t nf = collect_formats(formats, 4);
    if (nf == 0) return false;

    struct Usage { XrSwapchainUsageFlags flags; const char* desc; };
    const Usage usages[] = {
        {XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT |
         XR_SWAPCHAIN_USAGE_SAMPLED_BIT, "COLOR|DST|SAMPLED"},
        {XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT,
         "COLOR|DST"},
        {XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT, "DST|SAMPLED"},
        {XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT, "COLOR"},
    };

    XrResult last = XR_SUCCESS;
    for (size_t fi = 0; fi < nf; ++fi) {
        for (const Usage& u : usages) {
            XrSwapchainCreateInfo ci{XR_TYPE_SWAPCHAIN_CREATE_INFO};
            ci.usageFlags  = u.flags;
            ci.format      = formats[fi];
            ci.sampleCount = 1;
            ci.width       = (uint32_t)g_sc_w;
            ci.height      = (uint32_t)g_sc_h;
            ci.faceCount   = 1;
            ci.arraySize   = 1;
            ci.mipCount    = 1;

            const XrResult r = g_xr.create_swapchain(g_session, &ci, &g_swapchain);
            if (XR_SUCCEEDED(r)) {
                g_format  = formats[fi];
                g_is_bgra = (g_format == (int64_t)DXGI_FORMAT_B8G8R8A8_UNORM_SRGB ||
                             g_format == (int64_t)DXGI_FORMAT_B8G8R8A8_UNORM);
                logf("swapchain accepted: format 0x%llx, usage %s",
                     (unsigned long long)g_format, u.desc);
                goto created;
            }
            last = r;
            g_swapchain = XR_NULL_HANDLE;
            logf("  xrCreateSwapchain refused format 0x%llx usage %s -> %d",
                 (unsigned long long)formats[fi], u.desc, (int)r);
        }
    }
    logf("xrCreateSwapchain refused every format/usage pair (last %d) -- layer off", (int)last);
    return false;

created:
    // g_sc_w/g_sc_h were set by the atlas layout BEFORE this function ran -- they are the request,
    // not the answer. (xrCreateSwapchain cannot silently hand back a different extent; it either
    // accepts the one it was given or refuses.)
    uint32_t n = 0;
    if (XR_FAILED(g_xr.enumerate_images(g_swapchain, 0, &n, nullptr)) || n == 0) {
        logf("xrEnumerateSwapchainImages returned nothing -- layer off");
        destroy_swapchain();
        return false;
    }
    std::vector<XrSwapchainImageD3D12KHR> imgs(n, {XR_TYPE_SWAPCHAIN_IMAGE_D3D12_KHR});
    if (XR_FAILED(g_xr.enumerate_images(g_swapchain, n, &n,
                                        (XrSwapchainImageBaseHeader*)imgs.data()))) {
        logf("xrEnumerateSwapchainImages(fill) FAILED -- layer off");
        destroy_swapchain();
        return false;
    }

    g_images.resize(n);
    g_image_dirty.assign(n, true);
    for (uint32_t i = 0; i < n; ++i) g_images[i] = imgs[i].texture;

    logf("swapchain up: %dx%d, format 0x%llx, %u images, %s", g_sc_w, g_sc_h,
         (unsigned long long)g_format, n, g_is_bgra ? "BGRA" : "RGBA");
    return true;
}

// ============================================================================================
// Pose
// ============================================================================================
//
// UE cm -> VR metres.
//
// RECON-NEEDED. UEVR's VR_WorldScale is the factor relating the two, but the DIRECTION of the
// relationship is not documented anywhere we can cite, and guessing it is the "check value scale /
// normalization" trap: a wrong factor does not look broken, it looks like a reticule sitting at
// the wrong depth, which reads as a tuning problem and sends the investigation somewhere else.
//
// So: the assumption is stated here, it is logged out loud on the first frame, and xrlayercmpm
// overrides it outright. One in-headset run with the override swept over a decade settles it, and
// the answer belongs back in this comment as a measurement rather than an inference.
// Resolved on the GAME THREAD by xrlayer_tick() and cached here. The submit thread only ever reads
// this atomic.
//
// It used to be resolved inline in build_pose(), which put a UEVR mod-value lookup -- a string
// search plus an atof -- on the submit thread once per frame, for a value that changes when the
// player edits a file. That is the same mistake as applying a live tunable with an engine call
// every tick, which this repo already has a rule against; it is written down here because the
// version that did it looked perfectly reasonable.
std::atomic<float> g_cm_per_m{100.0f};

// GAME THREAD ONLY.
float resolve_cm_per_metre() {
    const Mirror m = mirror_load();
    if (m.cm_per_m > 0.0f) return m.cm_per_m;

    float ws = 1.0f;
    // Read through the raw C API rather than API::VR::get_mod_value<float>(): the header's
    // float path goes through std::stof on a buffer that is empty when the key is absent.
    char buf[64]{};
    if (auto* p = API::get()->param(); p != nullptr && p->vr != nullptr && p->vr->get_mod_value != nullptr) {
        p->vr->get_mod_value("VR_WorldScale", buf, sizeof(buf));
        if (buf[0] != 0) {
            const float v = (float)atof(buf);
            if (v > 0.01f && v < 100.0f) ws = v;
        }
    }
    return 100.0f * ws;
}

// UE world offset -> OpenXR offset, expressed in the HMD's own frame.
//
// The UE camera and the HMD are the same physical object, so an offset expressed in CAMERA-LOCAL
// coordinates is frame-invariant: it can be handed straight to the HMD's runtime-space rotation
// without ever needing the UE-world-to-stage transform, the standing origin or the recentre state.
// That is the whole reason this function exists in this shape -- the alternative is reconstructing
// UEVR's world mapping, which is a much larger surface to be wrong about.
//
//   UE camera-local:  X forward, Y right,    Z up
//   OpenXR view-local: X right,  Y up,       Z backward
XrVector3f ue_offset_to_xr(const Vec3& d_world, float view_yaw, float view_pitch, float view_roll,
                           float cm_per_m) {
    // ROLL IS NOT OPTIONAL, and passing 0.0f here was a real bug (reported in headset 2026-08-23:
    // "if I roll my head I can get the layer reticule to differ very greatly").
    //
    // The head orientation this offset is later rotated BY -- head_q, straight from get_pose() --
    // carries the player's true roll. Reconstructing the camera basis without it makes the pair
    // disagree about roll, and the composition head_q * inverse(q_cam) is then a spurious rotation
    // ABOUT THE VIEW AXIS. That swings the offset vector around the view centre, so the error is
    // proportional to how far off-axis the reticule sits and to sin(roll) -- near zero when you are
    // looking straight at it, and enormous when you are not. Which is exactly how it presented, and
    // is why it read as "mostly fine, occasionally way off" rather than as a constant error.
    const Quat q_cam = rotator_to_quat(view_pitch, view_yaw, view_roll);
    const Vec3 d_cam = quat_rotate(quat_conj(q_cam), d_world);

    const float k = 1.0f / cm_per_m;
    XrVector3f v;
    v.x =  d_cam.y * k;
    v.y =  d_cam.z * k;
    v.z = -d_cam.x * k;
    return v;
}

// Build an orientation from a forward/up pair that is ALREADY IN XR AXES (i.e. both have been put
// through ue_offset_to_xr). Returns false on a degenerate pair -- either vector ~zero, or the two
// parallel so the cross product has no direction -- and the caller then leaves the slot
// head-oriented. Refusing beats emitting a basis that is merely nearly-orthonormal: a quad with a
// subtly non-unit basis shears rather than rotating, which reads as a distorted image, not as a
// wrong angle, and is much harder to recognise as an orientation bug.
//
// OpenXR looks down -Z with +Y up, so the basis Z axis is the NEGATED forward.
bool xr_look_rotation(const XrVector3f& fwd, const XrVector3f& up, XrQuaternionf* out) {
    auto norm = [](XrVector3f& v) {
        const float l = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        if (!(l > 1e-6f)) return false;
        v.x /= l; v.y /= l; v.z /= l;
        return true;
    };
    auto cross = [](const XrVector3f& a, const XrVector3f& b) {
        return XrVector3f{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    };

    XrVector3f z{-fwd.x, -fwd.y, -fwd.z};
    if (!norm(z)) return false;
    XrVector3f x = cross(up, z);
    if (!norm(x)) return false;                 // up parallel to forward -- no roll reference
    const XrVector3f y = cross(z, x);           // unit by construction: z and x are unit+orthogonal

    // Rotation matrix columns are (x, y, z); Shepperd's method, branching on the largest diagonal
    // term so the divisor is never near zero.
    const float m00 = x.x, m01 = y.x, m02 = z.x;
    const float m10 = x.y, m11 = y.y, m12 = z.y;
    const float m20 = x.z, m21 = y.z, m22 = z.z;
    const float tr = m00 + m11 + m22;
    XrQuaternionf q;
    if (tr > 0.0f) {
        const float s = 0.5f / std::sqrt(tr + 1.0f);
        q.w = 0.25f / s;  q.x = (m21 - m12) * s;  q.y = (m02 - m20) * s;  q.z = (m10 - m01) * s;
    } else if (m00 > m11 && m00 > m22) {
        const float s = 2.0f * std::sqrt(1.0f + m00 - m11 - m22);
        q.w = (m21 - m12) / s;  q.x = 0.25f * s;  q.y = (m01 + m10) / s;  q.z = (m02 + m20) / s;
    } else if (m11 > m22) {
        const float s = 2.0f * std::sqrt(1.0f + m11 - m00 - m22);
        q.w = (m02 - m20) / s;  q.x = (m01 + m10) / s;  q.y = 0.25f * s;  q.z = (m12 + m21) / s;
    } else {
        const float s = 2.0f * std::sqrt(1.0f + m22 - m00 - m11);
        q.w = (m10 - m01) / s;  q.x = (m02 + m20) / s;  q.y = (m12 + m21) / s;  q.z = 0.25f * s;
    }
    *out = q;
    return true;
}

XrQuaternionf xr_mul(const XrQuaternionf& a, const XrQuaternionf& b) {
    XrQuaternionf r;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return r;
}

XrVector3f xr_rotate(const XrQuaternionf& q, const XrVector3f& v) {
    // v + 2w(q x v) + 2(q x (q x v))
    const float tx = 2.0f * (q.y * v.z - q.z * v.y);
    const float ty = 2.0f * (q.z * v.x - q.x * v.z);
    const float tz = 2.0f * (q.x * v.y - q.y * v.x);
    XrVector3f r;
    r.x = v.x + q.w * tx + (q.y * tz - q.z * ty);
    r.y = v.y + q.w * ty + (q.z * tx - q.x * tz);
    r.z = v.z + q.w * tz + (q.x * ty - q.y * tx);
    return r;
}

// Build the quad pose and pick the space.
//
// SPACE LADDER (xrlayerspace) -- a diagnostic, in the spirit of aim_widget_bg: one run in the
// headset separates possibilities that no amount of further reasoning can. Whether UEVR's
// get_pose() reports in the same space get_stage_space() names is an assumption, not a fact we
// have measured, and every mode below fails in a DIFFERENT, recognisable way:
//
//   0  stage space, raw HMD pose      -- correct if get_pose() is stage-relative.
//                                        Wrong => reticule is offset by the recentre, and the
//                                        error CHANGES when the player recentres.
//   1  stage space, recentre-corrected -- applies inverse(rotation_offset) + standing_origin, the
//                                        same correction UEVR's own stage-space quad applies
//                                        (OverlayComponent.cpp, generate_framework_ui_quad).
//                                        Correct if get_pose() is raw-runtime-relative.
//   2  view space                      -- head-locked. WRONG for a reticule by construction, and
//                                        that is the point: if mode 2 draws a ring in front of
//                                        your face then the hook, swapchain, format, blend and
//                                        blit are all proven and only the pose math is in doubt.
//                                        Start here when nothing appears at all.
// SPACE MODE 0 vs 1 IS STILL A LADDER, but 2 has done its job: the ring drew, so hook, swapchain,
// format, blend and blit are all proven and only the frame is in question.
//
// GAME THREAD ONLY. get_pose(), get_standing_origin() and get_rotation_offset() are all read here,
// at the same instant as the aim data, so the head pose the maths uses is the head pose the aim
// data belongs to.
// `orient` is the optional per-slot orientation override (nullptr = head-oriented, the default for
// every billboarded slot). It is taken as a parameter rather than read from g_slot_orient_* inside,
// so that this function stays what it has always been -- pure geometry that knows nothing about slot
// indices -- and so the atomic loads all happen together at the one call site, under the same
// snapshot as the target position.
bool compute_pose(const Vec3& world_pos, const Vec3& cam_pos,
                  float cam_yaw, float cam_pitch, float cam_roll,
                  float world_cm, float hold_cm,
                  XrPosef* out_pose, float* out_size_m, float* out_ang,
                  const XrQuaternionf* orient = nullptr) {
    const Mirror m = mirror_load();

    auto* p = API::get()->param();
    if (p == nullptr || p->openxr == nullptr || p->vr == nullptr) return false;

    const float cm_per_m = g_cm_per_m.load(std::memory_order_relaxed);
    if (cm_per_m <= 0.0f) return false;

    Vec3 d_world{world_pos.x - cam_pos.x,
                 world_pos.y - cam_pos.y,
                 world_pos.z - cam_pos.z};

    // HOLD ALONG THE RAY, at render rate. Shortening here rather than at the caller's tick is the
    // whole point: the direction AND the distance are then both derived from the eye position of
    // the frame being drawn, so walking towards an objective neither swings the marker (the tick's
    // stale direction) nor grows it (a world-fixed quad at a fixed world size). This is the fourth
    // place in this codebase that has had to move a computation into the callback that consumes it.
    if (hold_cm > 0.0f) {
        const float len = std::sqrt(d_world.x * d_world.x + d_world.y * d_world.y +
                                    d_world.z * d_world.z);
        if (len > 1.0f) {
            const float k = hold_cm / len;
            d_world.x *= k; d_world.y *= k; d_world.z *= k;
        } else {
            return false;   // the target is on top of the eye; there is no direction to hold
        }
    }

    const XrVector3f d_view = ue_offset_to_xr(d_world, cam_yaw, cam_pitch, cam_roll, cm_per_m);

    // SIZE IS DERIVED FROM THE IN-SCENE WIDGET, not tuned against it.
    //
    // The first pass used an absolute 0.06 m and came out a fraction of the game's crosshair,
    // because 0.06 m is ~7.9 UE cm at this world scale while the widget quad is
    // aim_widget_draw x aim_widget_scale = 256 x 0.24 = 61.4 UE cm. Roughly 8x, which is what the
    // headset showed.
    //
    // A constant that has to be dialled in until two things look the same is a constant that goes
    // wrong the moment either side is touched -- change aim_widget_scale and the layer silently
    // stops matching. So compute the same world size the widget uses and convert once. xrlayersize
    // is now a MULTIPLIER on that (1.0 = exactly the widget's size), not a length.
    //
    // THE WORLD SIZE ARRIVES FROM THE CALLER. This used to read aim_widget_draw x aim_widget_scale
    // straight out of the config -- correct for the reticule and silently wrong for anything else,
    // because a navpoint marker's size comes from nav_world_scale, its per-kind multiplier and the
    // distance it is drawn at. Hard-coding one caller's config in a shared function is how a
    // second caller ends up looking right in the first headset pass and wrong the moment either
    // key is retuned. Slot 0 folds xrlayersize in on its way here; see xrlayer_notice_reticule.
    *out_size_m = world_cm / cm_per_m;
    if (*out_size_m < 0.002f) *out_size_m = 0.002f;

    // The drop key. Angular size, not distance and not quad size -- see Snapshot::ang.
    const float dist_m = std::sqrt(d_view.x * d_view.x + d_view.y * d_view.y + d_view.z * d_view.z);
    if (out_ang != nullptr) *out_ang = (dist_m > 0.01f) ? (*out_size_m / dist_m) : 1.0e3f;

    if (m.space == 2) {
        // Head-locked diagnostic: park it straight ahead at the measured distance.
        out_pose->orientation = XrQuaternionf{0.0f, 0.0f, 0.0f, 1.0f};
        out_pose->position    = XrVector3f{0.0f, 0.0f, -(dist_m > 0.05f ? dist_m : 0.05f)};
        return true;
    }

    // Where is the head, in the space we are about to submit against?
    UEVR_Vector3f hp{};
    UEVR_Quaternionf hq{};
    const auto hmd = p->vr->get_hmd_index();
    p->vr->get_pose(hmd, &hp, &hq);

    XrQuaternionf head_q{hq.x, hq.y, hq.z, hq.w};
    XrVector3f    head_p{hp.x, hp.y, hp.z};

    if (m.space == 1) {
        // Same correction UEVR applies to its own stage-space quads.
        UEVR_Vector3f so{};
        UEVR_Quaternionf ro{};
        p->vr->get_standing_origin(&so);
        p->vr->get_rotation_offset(&ro);

        // inverse(rotation_offset)
        const XrQuaternionf inv{-ro.x, -ro.y, -ro.z, ro.w};
        head_q = xr_mul(inv, head_q);
        head_p = xr_rotate(inv, head_p);
        head_p.x += so.x; head_p.y += so.y; head_p.z += so.z;
    }

    const XrVector3f world_off = xr_rotate(head_q, d_view);
    out_pose->position = XrVector3f{head_p.x + world_off.x,
                                    head_p.y + world_off.y,
                                    head_p.z + world_off.z};

    // ---- FACE THE VIEWER, properly ----
    //
    // This used to be `orientation = head_q` with a comment excusing it as "close enough at the
    // small angles a reticule ever sits at". BOTH HALVES WERE WRONG. Using the head orientation
    // makes the quad parallel to the VIEW PLANE, not perpendicular to the eye->quad RAY; the two
    // coincide only when the reticule is dead centre. And the small-angle premise does not hold on
    // this title at all: aim here is controller-driven and independent of head yaw/pitch, so the
    // reticule routinely sits well off-axis. Reported from a headset on 2026-08-23 as a visible
    // perspective difference against the in-scene (world-space) reticule at low angles.
    //
    // A MINIMAL-ARC SWING, NOT A LOOK-AT. A look-at picks its own up-vector and would throw away the
    // player's roll; the crosshair art is NOT radially symmetric once the widget's own render target
    // is the source, so roll is load-bearing. Swinging head_q by the shortest rotation that takes
    // the quad's facing axis onto the eye->quad ray corrects the facing and changes nothing else --
    // and it reduces to identity on-axis, so it cannot regress the centred case that already looks
    // right.
    //
    // THE FACING AXIS IS +Z, pointing BACK at the viewer. Derived from this code's own working
    // behaviour rather than from the spec prose: `orientation = head_q` looks correct when centred,
    // and OpenXR view space is -Z-forward, so head_q's +Z points back at the head. The no-pose
    // fallback above agrees (identity orientation, position at -Z). UEVR's own quad builder in
    // OverlayComponent.cpp reads the same way. If this sign is nonetheless wrong the reticule faces
    // away and vanishes -- check a composited capture, do not re-reason about it.
    {
        const XrVector3f to_head{head_p.x - out_pose->position.x,
                                 head_p.y - out_pose->position.y,
                                 head_p.z - out_pose->position.z};
        const float len2 = to_head.x * to_head.x + to_head.y * to_head.y + to_head.z * to_head.z;

        // Below a few mm the ray direction is numerical noise; the un-swung head orientation is the
        // only sane answer and the quad is inside the viewer's eye anyway.
        if (len2 > 1.0e-4f) {
            const float inv = 1.0f / std::sqrt(len2);
            const XrVector3f b{to_head.x * inv, to_head.y * inv, to_head.z * inv};
            const XrVector3f a = xr_rotate(head_q, XrVector3f{0.0f, 0.0f, 1.0f});

            const float dot = a.x * b.x + a.y * b.y + a.z * b.z;
            // dot ~ -1 is a 180-degree swing with no defined axis. It cannot happen for a reticule
            // in front of the viewer, and guessing an axis there would spin the art; leave the
            // orientation alone rather than inventing one.
            if (dot > -0.999999f) {
                XrQuaternionf swing;
                swing.x = a.y * b.z - a.z * b.y;
                swing.y = a.z * b.x - a.x * b.z;
                swing.z = a.x * b.y - a.y * b.x;
                swing.w = 1.0f + dot;
                const float n2 = swing.x * swing.x + swing.y * swing.y +
                                 swing.z * swing.z + swing.w * swing.w;
                if (n2 > 1.0e-12f) {
                    const float ninv = 1.0f / std::sqrt(n2);
                    swing.x *= ninv; swing.y *= ninv; swing.z *= ninv; swing.w *= ninv;
                    // Swing is in the layer's space, so it composes on the LEFT of head_q.
                    head_q = xr_mul(swing, head_q);
                }
            }
        }
    }
    // THE OVERRIDE IS HEAD-LOCAL AND COMPOSES ON THE RIGHT OF head_q -- exactly mirroring how the
    // POSITION is built a few lines above (head position + head_q applied to a head-local offset).
    // The caller's directions were converted into head-local XR axes by the same ue_offset_to_xr()
    // that carries those offsets, so both halves of the pose cross the UE/XR boundary through one
    // proven mapping and cannot disagree about the frame.
    //
    // The identity case is the self-check quoted in the header: feed the VIEW's own forward and up
    // and the head-local basis is identity, so this reduces to head_q -- byte for byte the behaviour
    // every slot had before the pane existed.
    out_pose->orientation = (orient != nullptr) ? xr_mul(head_q, *orient) : head_q;
    return true;
}

// ============================================================================================
// The hook
// ============================================================================================

// ============================================================================================
// Atlas layout, and how many layers the runtime will take
// ============================================================================================

// Decide every cell rectangle, and with it the swapchain extent. GAME THREAD, once, at bring-up.
//
// The layout is FIXED at bring-up on purpose, exactly like the swapchain it sizes: the eight
// marker cells exist whether or not xrlayernavmax lets eight markers be submitted, so turning that
// key up mid-session cannot produce the "REFUSING source, wrong size" dead end that gating the
// swapchain on xrlayersrc already produced once.
//
// THE MARKER CELLS EXIST EVEN WHEN xrlayernav IS OFF, and that is deliberate (fixed 2026-08-25).
// The atlas used to shrink to reticule-only when nav was off and grow back when it was on, which
// meant toggling xrlayernav at runtime resized the swapchain -- a create-once resource -- so the
// code tore the whole layer DOWN and re-armed. That re-arm did not cleanly re-seat the RETICULE's
// captured source into the new atlas, so the reticule fell to the generated ring and never
// recovered (src=ring, permanent, reported in-headset). Reserving the marker cells unconditionally
// costs one always-allocated atlas (~1 MB) and makes xrlayernav a PURE per-frame decision: which
// cells get submitted, never the atlas shape. Nothing a live key does reshapes this now.
void build_atlas_layout() {
    for (auto& c : g_cell) c = Cell{};

    // Cell 0 -- the reticule, at the widget's own draw size so its render target can be presented
    // 1:1. Same read as before slots existed.
    g_ret_dim = RING_DIM;
    {
        const int d = g_cfg.aim_widget_draw;
        if (d >= 16 && d <= 4096) {
            g_ret_dim = d;
        } else {
            logf("aimwidgetdraw is %d, which cannot be a texture edge -- sizing cell 0 at %d. The "
                 "widget render target can never be presented at this setting.", d, RING_DIM);
        }
    }
    g_cell[XRLAYER_SLOT_RETICULE] = Cell{0, 0, g_ret_dim};
    g_sc_w = g_ret_dim;
    g_sc_h = g_ret_dim;
    g_nav_dim = 0;

    // The marker cells are laid out UNCONDITIONALLY -- not gated on xrlayernav -- so toggling that
    // key never resizes the atlas. See the header comment. When nav is off the cells simply are not
    // fed or submitted; they sit unused in the atlas.

    // The markers' cell edge is the navpoint widget's own draw size -- the same number
    // navw_host_class passes to SetDrawSize, read the same way, so the two cannot drift.
    int nd = (g_cfg.nav_world_draw > 0.0f) ? (int)g_cfg.nav_world_draw : 128;
    if (nd < 16 || nd > 1024) {
        logf("navworlddraw is %d, which cannot be a texture cell edge -- using 128.", nd);
        nd = 128;
    }

    constexpr int COLS = 4;
    const int rows = (XRLAYER_NAV_COUNT + COLS - 1) / COLS;        // 8 markers -> 2 rows

    // A SANE CAP, and falling back rather than refusing. A 4096 reticule plus eight 1024 markers
    // would ask for a 8192x6144 swapchain -- 200 MB of runtime allocation for an overlay, which a
    // runtime is entitled to refuse in a way that reads as "the feature does not work". Better to
    // keep the reticule, which is the proven half, and say why the markers did not fit.
    constexpr int CAP = 2048;

    // THE FALLBACK ORDER IS AN EXPLICIT PRIORITY, not an accident of arithmetic: reticule, then
    // markers, then pane. The pane is the newest and least proven of the three, so it is the first
    // thing dropped when the atlas will not fit -- a scope that says why it is off beats a reticule
    // that silently stopped existing to make room for one.
    int pane = g_pane_req;
    if (pane > 0 && (pane < 16 || pane > CAP)) {
        logf("pane cell %dpx is not a usable texture edge -- pane OFF.", pane);
        pane = 0;
    }

    auto atlas_w = [&](int p) { int m = (g_ret_dim > COLS * nd) ? g_ret_dim : COLS * nd; return (p > m) ? p : m; };
    auto atlas_h = [&](int p) { return g_ret_dim + rows * nd + p; };

    if (pane > 0 && (atlas_w(pane) > CAP || atlas_h(pane) > CAP)) {
        logf("atlas with a %dpx pane would be %dx%d, over the %d cap -- PANE DROPPED, reticule and "
             "markers kept. Lower the pane size to fit it in.",
             pane, atlas_w(pane), atlas_h(pane), CAP);
        pane = 0;
    }

    const int w = atlas_w(pane);
    const int h = atlas_h(pane);
    if (w > CAP || h > CAP) {
        logf("atlas would be %dx%d, over the %d cap (aimwidgetdraw=%d navworlddraw=%d) -- falling "
             "back to RETICULE ONLY. Lower navworlddraw or aimwidgetdraw to fit the markers in.",
             w, h, CAP, g_ret_dim, nd);
        return;
    }

    g_nav_dim = nd;
    g_sc_w = w;
    g_sc_h = h;
    for (int i = 0; i < XRLAYER_NAV_COUNT; ++i) {
        g_cell[XRLAYER_SLOT_NAV_BASE + i] = Cell{(int32_t)((i % COLS) * nd),
                                                 (int32_t)(g_ret_dim + (i / COLS) * nd),
                                                 (int32_t)nd};
    }

    g_pane_dim = pane;
    if (pane > 0) {
        g_cell[XRLAYER_SLOT_PANE] = Cell{0, (int32_t)(g_ret_dim + rows * nd), (int32_t)pane};
    }

    // THE GUIDE CELL GOES IN SPARE SPACE, and that is the entire design of it.
    //
    // atlas_w is max(g_ret_dim, COLS*nd, pane) -- so whenever the marker row is wider than the
    // reticule (512 vs 256 on the shipped numbers) there is already an unused rectangle to the
    // reticule's right. Putting the guide there means the atlas does NOT grow, the swapchain extent
    // does NOT change, and cells 0..9 keep byte-identical rectangles.
    //
    // That matters more than saving memory. This function's own header records that resizing the
    // atlas at runtime tore the layer down and left the reticule permanently on its generated ring
    // -- so a new cell that changed the extent would risk exactly the failure the fixed layout
    // exists to prevent. Fitting into slack risks nothing.
    //
    // IF IT DOES NOT FIT, the guide simply has no cell (dim 0) and never presents. Degrading to
    // "no guide" is correct; growing the atlas to make room is not.
    constexpr int GUIDE_DIM = 128;  // TEXT needs resolution a capsule did not: 32px could not spell "Grip"
    if (g_ret_dim + GUIDE_DIM <= w && GUIDE_DIM <= g_ret_dim) {
        g_cell[XRLAYER_SLOT_GUIDE] = Cell{(int32_t)g_ret_dim, 0, (int32_t)GUIDE_DIM};
    } else {
        logf("no slack in the atlas for the grab-guide cell (%dpx needed beside a %dpx reticule in "
             "a %dpx-wide atlas) -- the guide will not present on the layer.",
             GUIDE_DIM, g_ret_dim, w);
    }

    // The guide is stated POSITIVELY here, not left to be inferred from the absence of the "no
    // slack" warning above. Inferring presence from a missing line is the weaker evidence, and it
    // reads identically to "that code never ran" -- which is exactly the ambiguity that cost a
    // round of guessing when the guide did not appear.
    const Cell& gc = g_cell[XRLAYER_SLOT_GUIDE];
    logf("atlas: %dx%d -- cell 0 reticule %dpx at (0,0), %d navpoint cells %dpx from y=%d, "
         "pane %dpx at y=%d%s, guide %dpx at (%d,%d)%s",
         g_sc_w, g_sc_h, g_ret_dim, XRLAYER_NAV_COUNT, nd, g_ret_dim,
         pane, g_ret_dim + rows * nd, pane > 0 ? "" : " (none)",
         gc.dim, gc.x, gc.y, gc.dim > 0 ? "" : " (NONE -- no atlas slack, guide will not present)");
}

// HOW MANY COMPOSITION LAYERS WILL THIS RUNTIME ACCEPT? Ask it. GAME THREAD, once.
//
// There was no runtime query here at all before this: MAX_LAYERS = 32 in hooked_end_frame is a
// local array bound and nothing else, and reading it as a budget would be inventing a number. The
// spec guarantees at least XR_MIN_COMPOSITION_LAYERS_SUPPORTED (16), so that -- and never
// "unlimited" -- is the fallback when the query is unavailable.
//
// ADDR-HYGIENE: resolved -- xrGetSystem/xrGetSystemProperties come through the same
// xrattach_resolve path as every other entry point in this file (symbol lookup in UEVRBackend.pdb).
// No address is recorded anywhere, so there is nothing here for a UEVR update to rot.
void query_max_layers() {
    g_max_layers = 0;
    if (g_xr.get_system == nullptr || g_xr.get_system_props == nullptr) {
        logf("layer-budget query unavailable (xrGetSystem/xrGetSystemProperties did not resolve) "
             "-- assuming the spec minimum of %d.", XR_MIN_COMPOSITION_LAYERS_SUPPORTED);
        return;
    }
    auto* p = API::get()->param();
    if (p == nullptr || p->openxr == nullptr) return;
    const XrInstance inst = (XrInstance)p->openxr->get_xr_instance();
    if (inst == XR_NULL_HANDLE) {
        logf("UEVR reports no XrInstance -- assuming the spec minimum of %d composition layers.",
             XR_MIN_COMPOSITION_LAYERS_SUPPORTED);
        return;
    }

    XrSystemGetInfo gi{XR_TYPE_SYSTEM_GET_INFO};
    gi.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    XrSystemId sys = XR_NULL_SYSTEM_ID;
    if (XR_FAILED(g_xr.get_system(inst, &gi, &sys)) || sys == XR_NULL_SYSTEM_ID) {
        logf("xrGetSystem failed -- assuming the spec minimum of %d composition layers.",
             XR_MIN_COMPOSITION_LAYERS_SUPPORTED);
        return;
    }
    XrSystemProperties sp{XR_TYPE_SYSTEM_PROPERTIES};
    if (XR_FAILED(g_xr.get_system_props(inst, sys, &sp))) {
        logf("xrGetSystemProperties failed -- assuming the spec minimum of %d composition layers.",
             XR_MIN_COMPOSITION_LAYERS_SUPPORTED);
        return;
    }
    g_max_layers = sp.graphicsProperties.maxLayerCount;
}

// The budget for OUR layers on this frame: what the runtime accepts, minus what UEVR already put in
// the frame. UEVR's own usage is READ FROM THE FRAME every time rather than assumed -- it is the
// half of this arithmetic most likely to change under us.
uint32_t our_layer_budget(uint32_t uevr_layers, int forced) {
    if (forced > 0) return (uint32_t)forced;
    const uint32_t maxl = (g_max_layers > 0) ? g_max_layers
                                             : (uint32_t)XR_MIN_COMPOSITION_LAYERS_SUPPORTED;
    return (uevr_layers >= maxl) ? 0u : (maxl - uevr_layers);
}

// One-time bring-up. GAME THREAD.
//
// This used to run inside the xrEndFrame hook, on the theory that the hook is where a live session
// is guaranteed. That was both unnecessary and harmful: the plugin API hands us the session
// directly, and creating a swapchain from inside xrEndFrame is a REENTRANT call into the loader
// while it is mid-frame and holding its own state. The runtime answered XR_ERROR_RUNTIME_FAILURE
// (-2) -- a generic refusal that named nothing, cost a headset session to see, and was entirely
// self-inflicted. Ordinary applications create swapchains from ordinary threads; so do we.
bool bring_up(const Mirror& m) {
    auto* p = API::get()->param();
    if (p == nullptr || p->openxr == nullptr || p->renderer == nullptr) return false;

    const XrSession session = (XrSession)p->openxr->get_xr_session();
    if (session == XR_NULL_HANDLE) return false;      // not up yet; try again next tick

    if (p->renderer->device == nullptr || p->renderer->command_queue == nullptr) {
        logf("UEVR reports no D3D12 device/queue -- layer off");
        return false;
    }

    g_session = session;

    // CROSS-CHECK THE HANDLE ON THE LAYER RUNG. produce_layers() gates on session == g_session, so
    // a disagreement here would not surface as an error -- it would be the overlay silently never
    // drawing, on every frame, with every other diagnostic reading healthy. The two handles come
    // from different places (UEVR's plugin API, and the layer's own xrCreateSession) and are
    // expected to be the same value; nothing downstream can tell you when they are not.
    if (g_tier == XrAttachTier::ApiLayer) {
        const HaloVrLayerApi* api = xrbridge_api();
        const XrSession ls = (api != nullptr && api->get_session != nullptr)
                                 ? api->get_session() : XR_NULL_HANDLE;
        if (ls != XR_NULL_HANDLE && ls != session) {
            logf("SESSION MISMATCH: UEVR reports %p, the API layer reports %p. Using UEVR's -- if "
                 "the overlay never appears, this is why.", (void*)session, (void*)ls);
        }
    }

    g_device  = (ID3D12Device*)p->renderer->device;
    g_queue   = (ID3D12CommandQueue*)p->renderer->command_queue;

    g_stage_space = (XrSpace)p->openxr->get_stage_space();
    g_view_space  = (XrSpace)p->openxr->get_view_space();
    if (g_stage_space == XR_NULL_HANDLE || g_view_space == XR_NULL_HANDLE) {
        logf("UEVR reports no stage/view space -- layer off");
        return false;
    }

    // SIZE THE SWAPCHAIN FOR WHAT IT WILL PRESENT, before creating it.
    //
    // CopyResource requires IDENTICAL dimensions, so a 128x128 swapchain simply cannot present a
    // 256x256 widget render target -- and the failure would be an invalid copy, not a small
    // picture. The size is therefore decided by the SOURCE MODE, not by a constant, and the actual
    // resource is still checked against it in xrlayer_set_source() before anything is copied.
    //
    // Read from aim_widget_draw rather than hardcoded: it is the same number reticule_widget_ensure
    // passes to SetDrawSize, so the two cannot drift apart.
    // SIZED FROM aim_widget_draw UNCONDITIONALLY -- NOT gated on xrlayersrc, and that gate was the
    // bug. The swapchain is created once, at bring-up, and never resized; xrlayersrc is a LIVE key.
    // So a session that brought the layer up with xrlayersrc=0 got a RING_DIM swapchain, and turning
    // xrlayersrc on afterwards produced a permanent "REFUSING source: it is 256x256 and the
    // swapchain is 128x128" that no tunable could clear. Worse, it looked intermittent: whether the
    // feature worked depended on which keys happened to be on at the instant the session came up.
    //
    // The generated ring is fully dim-parameterised (see generate_bitmap), so sizing it at
    // aim_widget_draw costs nothing but a slightly larger staging buffer and removes the ordering
    // dependency outright. A source is still checked against the real number in xrlayer_set_source().
    build_atlas_layout();
    query_max_layers();

    // SWAPCHAIN FIRST, then the bitmap. The runtime picks the format, and the format decides both
    // the channel order the bitmap must be written in and the footprint format the copy must use.
    // Generating the bitmap first would mean guessing that, and a guess here shows up as a wrong
    // hue or an invalid copy rather than as an error at the point of the mistake.
    if (!create_swapchain() || !create_d3d_resources(m.cr, m.cg, m.cb, m.alpha)) {
        release_d3d();
        destroy_swapchain();
        return false;
    }

    // SAY THE BUDGET BESIDE THE ARMED LINE. An overlay that silently stops being appended because
    // the runtime is one layer short is indistinguishable from an overlay that is broken, and the
    // number is not knowable from the outside afterwards.
    char maxl[32];
    if (g_max_layers > 0) _snprintf_s(maxl, sizeof(maxl), _TRUNCATE, "%u", g_max_layers);
    else                  strcpy_s(maxl, "unknown");
    logf("ARMED -- cm/m %.1f, space mode %d, size %.3f m, atlas %dx%d, runtime maxLayerCount=%s "
         "(fallback %d)",
         g_cm_per_m.load(std::memory_order_relaxed), m.space, m.size_m, g_sc_w, g_sc_h,
         maxl, XR_MIN_COMPOSITION_LAYERS_SUPPORTED);
    return true;
}

// SUBMIT-THREAD ONLY, and one per slot. Static rather than a local because the layer structs must
// outlive this function's call into the runtime -- XrFrameEndInfo holds POINTERS to them.
XrCompositionLayerQuad g_quads[XRLAYER_SLOTS]{};
// The scope-pane reticule's quad. Its own storage rather than an eleventh g_quads entry, because
// g_quads is indexed by DRAW ORDER within the slot budget and this quad is outside that accounting
// -- it is derived from another slot's pose rather than owning a slot of its own.
XrCompositionLayerQuad g_scope_ret_quad{};

// PRODUCE OUR LAYERS FOR THIS FRAME. SUBMIT THREAD. Returns how many pointers it wrote into
// `out`, at most `out_capacity`; 0 means "nothing to add" and is the fail-open answer at every
// gate below.
//
// THIS IS THE WHOLE OF THE PER-FRAME WORK, AND BOTH ATTACHMENT RUNGS SHARE IT. What differs is
// only who does the appending afterwards:
//
//   layer rung -- bridge_end_frame() hands these straight back to the API layer, which does the
//                 copy, the call-through and the fail-open retry itself (see XrLayerAbi.h).
//   hook rung  -- hooked_end_frame() copies UEVR's layers, appends ours, calls through, and
//                 retries clean if the runtime refuses.
//
// Splitting it this way is not tidiness. Two copies of these gates would drift, and the ones that
// would drift silently are exactly the ones deciding whether a wrong marker is shown.
//
// THE POINTERS WRITTEN HERE MUST OUTLIVE THE RETURN -- both callers forward them to the runtime
// after this returns. That is why g_quads and g_scope_ret_quad are static and not locals.
uint32_t produce_layers(XrSession session, const XrFrameEndInfo* info,
                        const XrCompositionLayerBaseHeader** out, uint32_t out_capacity) {
    if (info == nullptr || out == nullptr) return 0;

    const Mirror m = mirror_load();
    if (!m.enabled) return 0;

    const State st = g_state.load(std::memory_order_relaxed);
    if (st == State::Failed || st == State::Off) return 0;

    // Bring-up is NOT done here -- see bring_up() on the game thread. Until it succeeds we are only
    // a pass-through.
    if (st != State::Armed) return 0;
    if (session != g_session) return 0;

    Frame fr{};
    if (!read_snapshot(&fr)) return 0;

    const XrSpace space = (m.space == 2) ? g_view_space : g_stage_space;
    if (space == XR_NULL_HANDLE) return 0;

    // ============================================================================================
    // WHICH SLOTS ARE DRAWABLE, and how many of them fit
    // ============================================================================================
    //
    // Decided BEFORE the swapchain is touched, deliberately: a frame that turns out to have no
    // budget must not acquire an image it is only going to release again, and an early return
    // between acquire and release is how a swapchain ring buffer leaks entries.

    // ONE load of the seam, used for every decision below AND for the copy itself. Reading the
    // atomic twice would let the game thread drop the atlas between "should I copy?" and "copy from
    // what?", which is a null dereference on exactly the rare frame that is hardest to reproduce --
    // and reading the raw g_owned global instead would skip the publication protocol entirely.
    ID3D12Resource* const atlas = g_source_override.load(std::memory_order_acquire);
    const bool from_atlas = (atlas != nullptr);

    const uint64_t now       = GetTickCount64();
    const uint64_t hold      = (uint64_t)g_m_hold_ms.load(std::memory_order_relaxed);
    const uint32_t game_tick = g_game_tick.load(std::memory_order_relaxed);

    struct Draw { int slot; float ang; int prio; };
    Draw draw[XRLAYER_SLOTS];
    int  n_draw = 0;
    bool ret_stale = false;

    for (int s = 0; s < XRLAYER_SLOTS; ++s) {
        const Snapshot& sn = fr.slot[s];
        g_slot_drop[s].store(0, std::memory_order_relaxed);
        if (!sn.valid) { g_slot_drop[s].store(1, std::memory_order_relaxed); continue; }
        g_slot_cand[s].fetch_add(1, std::memory_order_relaxed);   // had a pose: a real candidate
        // No fresh pose for a while means the drive path is not running (menu, cutscene, loading).
        // Draw nothing rather than leaving a stale quad hanging in space.
        if (game_tick - sn.tick > 8) { g_slot_drop[s].store(2, std::memory_order_relaxed);
                                       g_slot_gate[s][2].fetch_add(1, std::memory_order_relaxed);
                                       g_slot_dropn[s].fetch_add(1, std::memory_order_relaxed); continue; }
        if (g_cell[s].dim <= 0) { g_slot_drop[s].store(3, std::memory_order_relaxed);
                                  g_slot_gate[s][3].fetch_add(1, std::memory_order_relaxed);
                                  g_slot_dropn[s].fetch_add(1, std::memory_order_relaxed); continue; }

        // ---- THE FRESHNESS GATE, NOW PER SLOT AND STILL COSMETIC ----
        //
        // Every cell holds OUR pixels, so a stale beat can no longer hurt anything -- it only means
        // the game thread has stopped capturing that slot. What follows from that differs by slot,
        // and conflating the two is what made a single global gate wrong here:
        //
        //   slot 0 has a fallback picture (the generated ring), so it keeps drawing and swaps art.
        //   a MARKER has none. A navpoint whose art stopped updating is a waypoint the player can
        //   walk around -- the lane's own "fail by hiding, never by freezing" rule -- so it is
        //   simply not appended.
        const uint64_t beat  = g_slot_beat[s].load(std::memory_order_relaxed);
        const bool     fresh = (beat != 0 && now >= beat && (now - beat) <= hold);

        if (s == XRLAYER_SLOT_RETICULE) {
            ret_stale = !fresh;
        } else if (s == XRLAYER_SLOT_GUIDE) {
            // EXEMPT, for the same reason slot 0 is: its art is GENERATED, not captured.
            //
            // This gate asks "is the game thread still capturing this slot", and the guide has
            // nothing to capture -- generate_label writes its cell once at bring-up and it is
            // correct forever. g_slot_beat therefore stays 0, `fresh` is permanently false, and
            // without this branch the guide was built, posed, and then silently dropped from the
            // layer list every single frame. That was the "I do not see it" report: everything
            // upstream worked and the quad was never appended.
            //
            // Nothing is lost by exempting it. Liveness is already enforced above by the POSE --
            // sn.valid and the 8-tick staleness check -- which is the right question for a quad
            // whose pixels cannot go stale. xrlayer_retire_quad() is what turns it off.
        } else if (!fresh) {
            g_slot_drop[s].store(4, std::memory_order_relaxed);
            g_slot_gate[s][4].fetch_add(1, std::memory_order_relaxed);
            g_slot_dropn[s].fetch_add(1, std::memory_order_relaxed);
            continue;
        } else if (!slot_cell_coherent(s)) {
            g_slot_drop[s].store(5, std::memory_order_relaxed);
            g_slot_gate[s][5].fetch_add(1, std::memory_order_relaxed);
            g_slot_dropn[s].fetch_add(1, std::memory_order_relaxed);
            g_coh_skips.fetch_add(1, std::memory_order_relaxed);
            // Fresh art, but captured BEFORE this slot's most recent re-host -- the cell still holds
            // the previous navpoint's widget while the pose has already moved to the new one. Do not
            // append it: the coherent capture arrives next tick, and Plugin.cpp draws the in-scene
            // marker (correct art AND pose) in the meantime. This is the wrong-marker-flash fix; it
            // costs a marker at most one or two frames on the layer when its kind changes.
            continue;
        }
        // ABOUT TO APPEND. If this slot was re-hosted within the last couple of ticks the cell may
        // still hold the outgoing navpoint's pixels while coherence has just said otherwise.
        // Counting it turns "I sometimes see a wrong marker" into a number.
        if (s != XRLAYER_SLOT_RETICULE) {
            const uint32_t host = g_slot_host_tick[s].load(std::memory_order_relaxed);
            if (host != 0 && (int32_t)(game_tick - host) >= 0 && (game_tick - host) <= 2) {
                g_coh_drawn.fetch_add(1, std::memory_order_relaxed);
            }
        }
        draw[n_draw].slot = s;
        draw[n_draw].ang  = sn.ang;
        draw[n_draw].prio = sn.prio;
        ++n_draw;
    }
    if (n_draw == 0) return 0;

    // ---- the ring fall-back counter, on the EDGE only ----
    //
    // GATED ON from_atlas, because "we have never captured anything" and "we had the game's art and
    // lost it" are different events and only the second is a fall-back. Counting the first would
    // put ringfalls=1 in the state line of every session where the source simply never resolved --
    // a number that means "the crosshair flickered" reporting something else entirely.
    if (from_atlas) {
        static bool was_stale = false;
        if (ret_stale && !was_stale) {
            g_ring_falls.fetch_add(1, std::memory_order_relaxed);
            static uint32_t stale_says = 0;
            if (stale_says < 5) {
                ++stale_says;
                logf("no reticule capture for >%llu ms (the game thread is busy or gone -- a level "
                     "change looks exactly like this). Presenting the generated ring rather than a "
                     "frozen crosshair until it comes back. THIS LINE IS CAPPED AT 5 PRINTS -- "
                     "after that, read ringfalls= in the state line instead.",
                     (unsigned long long)hold);
            }
            for (size_t i = 0; i < g_image_dirty.size(); ++i) g_image_dirty[i] = true;
        } else if (!ret_stale && was_stale) {
            for (size_t i = 0; i < g_image_dirty.size(); ++i) g_image_dirty[i] = true;
        }
        if (!ret_stale) {
            // The worst gap we RODE OUT, i.e. one the hold window actually covered. Recorded only
            // on this branch on purpose: on the fall-back branch the age keeps climbing for the
            // whole length of a loading screen, so measuring it there would report tens of seconds
            // and tell you nothing about whether the window is big enough for a HITCH.
            const uint64_t beat = g_slot_beat[XRLAYER_SLOT_RETICULE].load(std::memory_order_relaxed);
            const uint64_t age  = (beat != 0 && now >= beat) ? (now - beat) : 0;
            if (age > (uint64_t)g_hold_worst_ms.load(std::memory_order_relaxed)) {
                g_hold_worst_ms.store((uint32_t)age, std::memory_order_relaxed);
            }
        }
        was_stale = ret_stale;
    }
    g_showing_ring.store(ret_stale || !from_atlas, std::memory_order_relaxed);

    // ---- THE LAYER BUDGET. Queried, never assumed; UEVR's own usage read from the frame ----
    //
    // MAX_LAYERS is the local array bound and nothing more -- it is NOT a budget and reading it as
    // one is exactly the invention this block exists to avoid. The real ceiling came from
    // xrGetSystemProperties at bring-up (see query_max_layers), and what UEVR has already put in
    // this frame is read from info->layerCount every time rather than guessed.
    // `out_capacity` is the caller's room for OUR layers and nothing more -- the hook rung passes
    // what is left of its 32-entry array after UEVR's own layers, the API-layer rung passes
    // HALOVR_LAYER_MAX_EXTRA_LAYERS. It is NOT a budget, and reading it as one is the invention
    // this block exists to avoid. The real ceiling came from xrGetSystemProperties at bring-up
    // (see query_max_layers), and what UEVR has already put in this frame is read from
    // info->layerCount every time rather than guessed.
    //
    // Tested HERE rather than at the top of the function on purpose: the freshness gates and the
    // ring fall-back counter above have already run, and they must keep running on a frame we end
    // up adding nothing to, or ringfalls and g_showing_ring stop tracking reality.
    if (out_capacity == 0) return 0;

    uint32_t budget = our_layer_budget(info->layerCount, m.budget);
    if (budget > out_capacity) budget = out_capacity;
    if (budget == 0) return 0;   // nothing acquired, so nothing to release

    // DROP ORDER: priority first (0 = the reticule, never dropped), then LARGEST APPARENT SIZE
    // first, so what goes is the furthest/smallest -- one comparison expressing both. Insertion
    // sort over at most nine elements on the submit thread: no allocation, no branch misprediction
    // worth measuring, and trivially auditable, which matters more here than asymptotics.
    for (int i = 1; i < n_draw; ++i) {
        const Draw key = draw[i];
        int j = i - 1;
        while (j >= 0 && (draw[j].prio > key.prio ||
                          (draw[j].prio == key.prio && draw[j].ang < key.ang))) {
            draw[j + 1] = draw[j];
            --j;
        }
        draw[j + 1] = key;
    }
    uint32_t n_use = (uint32_t)n_draw;
    if (n_use > budget) {
        g_layer_drops.fetch_add((uint32_t)n_draw - budget, std::memory_order_relaxed);
        // Mark the ones the sort is about to shed, so "my quad vanished" names the budget rather
        // than looking identical to never having been a candidate.
        for (uint32_t di = budget; di < (uint32_t)n_draw; ++di) {
            g_slot_drop[draw[di].slot].store(6, std::memory_order_relaxed);
            g_slot_gate[draw[di].slot][6].fetch_add(1, std::memory_order_relaxed);
            g_slot_dropn[draw[di].slot].fetch_add(1, std::memory_order_relaxed);
        }
        n_use = budget;
    }

    // ---- acquire / wait / blit / release ----
    uint32_t idx = 0;
    XrSwapchainImageAcquireInfo ai{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
    if (XR_FAILED(g_xr.acquire_image(g_swapchain, &ai, &idx))) return 0;

    // TIMEOUT IS A SUCCESS CODE. XR_TIMEOUT_EXPIRED is non-negative, so XR_FAILED() does not catch
    // it -- testing with XR_FAILED here would sail past a wait that never completed and write into
    // an image the runtime still owns. Only XR_SUCCESS means the image is ours.
    //
    // 20 ms rather than 0: a zero timeout makes a momentarily-busy ring buffer look like a hard
    // failure, and rather than stall we would silently drop the overlay on exactly the frames the
    // compositor is busiest. 20 ms is generously above any healthy runtime's turnaround and still
    // well under a frame's worth of damage if it ever does hit.
    XrSwapchainImageWaitInfo wi{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
    wi.timeout = 20LL * 1000LL * 1000LL;   // ns
    if (g_xr.wait_image(g_swapchain, &wi) != XR_SUCCESS) {
        // The image was acquired but never became ours to write. Release it so the ring buffer
        // does not leak an entry, draw nothing this frame, and forward the frame untouched.
        XrSwapchainImageReleaseInfo ri{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
        g_xr.release_image(g_swapchain, &ri);
        static uint32_t waits = 0;
        if (waits < 5) { ++waits; logf("xrWaitSwapchainImage did not complete -- overlay skipped this frame"); }
        return 0;
    }

    // A colour/alpha edge rewrites the staging buffer IN PLACE, so the GPU must be done reading it.
    // This is the one place a fence wait is correct rather than lazy: it fires only when a live
    // tunable moved, not per frame -- and it is what makes the cell-0 ring overlay below safe to
    // issue from the same buffer without a per-frame stall.
    if (g_regen.exchange(false, std::memory_order_relaxed)) {
        if (g_fence != nullptr && g_fence_ev != nullptr && g_fence_v != 0 &&
            g_fence->GetCompletedValue() < g_fence_v) {
            if (SUCCEEDED(g_fence->SetEventOnCompletion(g_fence_v, g_fence_ev))) {
                WaitForSingleObject(g_fence_ev, 1000);
            }
        }
        if (fill_upload(m.cr, m.cg, m.cb, m.alpha)) {
            for (size_t i = 0; i < g_image_dirty.size(); ++i) g_image_dirty[i] = true;
        }
    }

    // The generated atlas never changes, so while nothing has been captured each image is written
    // once and then reused -- acquire/wait/release and nothing else. Once real art is arriving the
    // atlas changes every frame and every image is copied every frame, which is why blit_into does
    // NOT fence-wait on that path.
    const bool ring_cell0 = from_atlas && ret_stale;
    // THE GUIDE'S CELL HAS TO BE RE-LAID EVERY CAPTURED FRAME, not once per image. CopyResource
    // above rewrites the whole image from the captured atlas each frame, so a dirty-flag-style
    // "write it once" is exactly what fails here -- the capsule survives until the first captured
    // frame and is gone from then on. g_tgt_live is the game thread saying it wants the guide
    // drawn; it is cleared by xrlayer_retire_quad, so the copy stops when the guide does.
    const bool guide_cell = from_atlas &&
                            g_tgt_live[XRLAYER_SLOT_GUIDE].load(std::memory_order_relaxed) &&
                            g_cell[XRLAYER_SLOT_GUIDE].dim > 0;
    const bool need_copy  = from_atlas || (idx < g_image_dirty.size() && g_image_dirty[idx]);
    if (need_copy && idx < g_images.size()) {
        if (blit_into(g_images[idx], atlas, ring_cell0, guide_cell) && idx < g_image_dirty.size()) {
            g_image_dirty[idx] = false;
        }
    }

    XrSwapchainImageReleaseInfo ri{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
    g_xr.release_image(g_swapchain, &ri);

    // ---- build our layers ----
    //
    // One quad per drawable slot, all referencing the SAME swapchain through different imageRects.
    // That is what XrSwapchainSubImage::imageRect is for and it is the whole reason there is one
    // swapchain rather than nine: nine acquire/wait pairs at a 20 ms timeout each is a 180 ms worst
    // case on this thread.
    //
    // NOTE on eyeVisibility: BOTH, never a LEFT/RIGHT pair. CUTSCENE_FINDINGS.md records, measured
    // on this stack, that SteamVR DROPS eye-visibility quad pairs from normal presentation -- they
    // only appear when the dashboard flattens app layers. A per-eye disparity pair would therefore
    // work on some runtimes and silently render nothing on the one most of our users are on.
    // OURS ONLY. Copying UEVR's own layers alongside them belongs to the caller, because only the
    // caller knows whether it or the API layer is going to do the appending.
    uint32_t n_ours = 0;

    // APPENDED IN REVERSE OF THE DROP ORDER, so the thing least willing to be dropped ends up LAST
    // -- and last is topmost. The reticule therefore draws over the markers, which is the right way
    // round: it is the only one of the nine that is aimed with.
    for (int k = (int)n_use - 1; k >= 0; --k) {
        const int s = draw[k].slot;
        const Cell& c = g_cell[s];
        XrCompositionLayerQuad& q = g_quads[k];
        q = XrCompositionLayerQuad{XR_TYPE_COMPOSITION_LAYER_QUAD};
        // ALPHA BLENDING IS PER-SLOT, because the slots are not the same KIND of image.
        //
        // Slots 0..8 are WIDGETS on transparent backgrounds -- the reticule and the navpoint
        // markers. They need source-alpha blending or each one composites as a big opaque square.
        //
        // Slot 9 is the scope pane: a FULL-FRAME SCENE CAPTURE. Its alpha comes from the renderer,
        // where it is not meaningful for an opaque scene and reads ~0 across the image. Blended by
        // that alpha the entire pane disappears EXCEPT where something wrote alpha -- which, since
        // xrlayerhidews=3 keeps the world-space reticule in the capture, is the reticule's own
        // pixels. Reported from a headset as "the render is happening within the lines of the
        // reticle", with its colour tracking aim: that is the scene showing through an alpha mask,
        // and it was misread as a brightness problem for an hour (it is not -- no amount of gain
        // moves an alpha of zero).
        // Slot 9 blends ONLY when scopemask actually wrote its alpha this frame. Without the mask
        // the capture's alpha is ~0 and blending by it makes the whole pane disappear except where
        // the captured reticule wrote alpha -- the exact bug this per-slot flag was added to fix.
        // So the flag follows the mask having RUN, never merely the mask being configured.
        q.layerFlags    = (s == XRLAYER_SLOT_PANE && !scopemask_applied())
                        ? 0
                        : XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
        q.space         = space;
        q.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
        q.subImage.swapchain        = g_swapchain;
        q.subImage.imageRect.offset = {c.x, c.y};
        q.subImage.imageRect.extent = {c.dim, c.dim};
        q.subImage.imageArrayIndex  = 0;
        q.pose = fr.slot[s].pose;
        // Non-square when the caller asked for it (the grab guide); square otherwise, which is
        // every other slot and is what size_h_m == 0 means.
        q.size = {fr.slot[s].size_m,
                  fr.slot[s].size_h_m > 0.0f ? fr.slot[s].size_h_m : fr.slot[s].size_m};
        out[n_ours++] = (const XrCompositionLayerBaseHeader*)&q;
    }

    // ---- SCOPE PANE RETICULE -------------------------------------------------------------------
    //
    // One extra quad, drawn LAST so it composites over the pane. Everything it needs already exists:
    // the reticule's atlas cell (captured every frame anyway) and the pane's finished pose. No new
    // capture, no GPU copy, no blend code -- the compositor blends layers by itself.
    //
    // Guarded on every precondition rather than assumed, because this runs on the submit thread:
    // feature on, pane actually drawn this frame, reticule cell exists, and room in the array.
    if (g_m_scope_ret.load(std::memory_order_relaxed) != 0 && n_ours < out_capacity) {
        bool pane_drawn = false;
        for (uint32_t k = 0; k < n_use; ++k) {
            if (draw[k].slot == XRLAYER_SLOT_PANE) { pane_drawn = true; break; }
        }
        const Cell& rc = g_cell[XRLAYER_SLOT_RETICULE];
        // WHY THE SCOPE RETICULE IS NOT THERE -- edge-triggered on the reason, so a steady state
        // costs nothing and every transition is recorded (including the transition INTO working).
        //
        // This block had NO instrument at all, which is why "I still see no reticle in the scope"
        // survived several rounds of guessing: it silently does nothing when the pane is not drawn,
        // and the pane not being drawn has its own four possible causes one layer down. Absence of
        // a complaint was being read as "this part is fine".
        {
            enum : int { R_OK = 0, R_NO_PANE = 1, R_NO_RET_CELL = 2 };
            const int why = !pane_drawn ? R_NO_PANE : (rc.dim <= 0 ? R_NO_RET_CELL : R_OK);
            static int s_why_prev = -1;
            if (why != s_why_prev) {
                s_why_prev = why;
                static const char* kWhy[] = {
                    "DRAWING over the pane",
                    "NOT drawing: the pane quad was not submitted this frame (slot 9 lost its "
                    "admission -- read the slot 9 gate breakdown in the state line)",
                    "NOT drawing: the reticule has no atlas cell",
                };
                logf("scope reticule: %s", kWhy[why]);
            }
        }
        if (pane_drawn && rc.dim > 0) {
            const Snapshot& pane = fr.slot[XRLAYER_SLOT_PANE];
            float f = g_m_scope_ret_size.load(std::memory_order_relaxed);
            if (!(f > 0.0f) || f > 1.0f) f = 0.12f;

            g_scope_ret_quad = XrCompositionLayerQuad{XR_TYPE_COMPOSITION_LAYER_QUAD};
            g_scope_ret_quad.layerFlags    = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
            g_scope_ret_quad.space         = space;
            g_scope_ret_quad.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
            g_scope_ret_quad.subImage.swapchain        = g_swapchain;
            g_scope_ret_quad.subImage.imageRect.offset = {rc.x, rc.y};
            g_scope_ret_quad.subImage.imageRect.extent = {rc.dim, rc.dim};
            g_scope_ret_quad.subImage.imageArrayIndex  = 0;

            // Same plane and facing as the pane, nudged toward the viewer along the quad's own +Z
            // so it cannot z-fight with the pane it sits on -- and SLID ACROSS THE PANE'S FACE by
            // the projected impact offset, in the pane's OWN local axes, so it stays on the surface
            // at any pane orientation instead of being displaced through the world.
            //
            // The offset is normalised -1..1 against the pane's HALF extent, so u=1 is exactly the
            // right edge. Absent or invalid, it is 0,0 -- dead centre, the old behaviour.
            g_scope_ret_quad.pose = pane.pose;
            float ru = 0.0f, rv = 0.0f;
            if (g_scope_ret_off_have.load(std::memory_order_relaxed)) {
                ru = g_scope_ret_u.load(std::memory_order_relaxed);
                rv = g_scope_ret_v.load(std::memory_order_relaxed);
            }
            const float rhalf = pane.size_m * 0.5f;
            // THE STANDOFF, via g_cm_per_m rather than a bare /100. That atomic already carries the
            // UEVR world scale, which is what the pane's own size_m was built with -- so converting
            // the same way keeps the reticule's standoff proportional to the pane instead of
            // drifting relative to it whenever world scale changes. The old hardcoded 0.01 m was
            // correct only at world scale 1, and silently wrong at the 1.312 this profile ships.
            float cmpm = g_cm_per_m.load(std::memory_order_relaxed);
            if (!(cmpm > 0.0f)) cmpm = 100.0f;
            const float depth_m = g_m_scope_ret_depth.load(std::memory_order_relaxed) / cmpm;
            const XrVector3f n = xr_rotate(pane.pose.orientation,
                                           XrVector3f{ru * rhalf, rv * rhalf, depth_m});
            g_scope_ret_quad.pose.position.x += n.x;
            g_scope_ret_quad.pose.position.y += n.y;
            g_scope_ret_quad.pose.position.z += n.z;
            g_scope_ret_quad.size = {pane.size_m * f, pane.size_m * f};

            // ---- MODE 2: PUT IT WHERE THE SHOT ACTUALLY GOES, not at the optical centre --------
            //
            // Mode 1 draws at the pane's centre, which is EXACT for the ray the capture is mounted
            // on -- cam_pos = origin + dir * scope_cam_dist, looking down dir, so a point on that
            // ray is on the optical axis by construction. What makes centre a lie is that the two
            // consumers are not given the same ray: the scope gets the RAW aim angles while the
            // reticule is drawn at a SMOOTHED target. The gap between them is real information
            // about where the round lands, and it is exactly what the in-scene reticule used to
            // show by drifting off-centre inside the pane.
            //
            // THE FRAME DISCIPLINE MATTERS AND IS THE WHOLE REASON THIS IS SAFE: the angles are
            // computed entirely in UE WORLD space, where the ray and the target both live, and only
            // the resulting dimensionless FRACTIONS cross into XR space, where they are applied
            // along the pane quad's own right/up. No vector, no rotation and no handedness crosses
            // the boundary -- which is the mistake this module has already paid for once.
            if (g_m_scope_ret.load(std::memory_order_relaxed) == 2
                && g_scope_ray_have.load(std::memory_order_acquire)) {
                const Vec3 ro{g_scope_ray_ox.load(std::memory_order_relaxed),
                              g_scope_ray_oy.load(std::memory_order_relaxed),
                              g_scope_ray_oz.load(std::memory_order_relaxed)};
                const Vec3 rt{g_scope_ray_tx.load(std::memory_order_relaxed),
                              g_scope_ray_ty.load(std::memory_order_relaxed),
                              g_scope_ray_tz.load(std::memory_order_relaxed)};
                // WORLD POINT, RECONSTRUCTED. g_tgt_* no longer always holds one: with
                // xrlayerhrel on, the reticule's entry is an OFFSET from the mono camera (that is
                // what stops it trailing a tick behind during locomotion). Every line below is UE
                // world-space maths against the scope ray, so an offset used here would be wrong by
                // the whole distance from the world origin to your head -- and it would look like
                // "mode 2 puts the reticule in the wrong place", not like a unit mismatch.
                Vec3 tgt{g_tgt_x[XRLAYER_SLOT_RETICULE].load(std::memory_order_relaxed),
                         g_tgt_y[XRLAYER_SLOT_RETICULE].load(std::memory_order_relaxed),
                         g_tgt_z[XRLAYER_SLOT_RETICULE].load(std::memory_order_relaxed)};
                bool tgt_ok = true;
                if (g_tgt_headrel[XRLAYER_SLOT_RETICULE].load(std::memory_order_relaxed)) {
                    if (g_mono_view_have.load(std::memory_order_acquire)) {
                        tgt.x += g_mono_view_x.load(std::memory_order_relaxed);
                        tgt.y += g_mono_view_y.load(std::memory_order_relaxed);
                        tgt.z += g_mono_view_z.load(std::memory_order_relaxed);
                    } else {
                        // No camera to undo it with, so we do NOT have a world point. Leave the
                        // quad at the pane centre rather than place it from a fabricated one: mode
                        // 1 is a known approximation, a bad world point is a lie about where the
                        // round lands -- and this reticule's whole purpose is that promise.
                        tgt_ok = false;
                    }
                }

                Vec3 d{rt.x - ro.x, rt.y - ro.y, rt.z - ro.z};
                const float dl = std::sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
                const float fov = g_m_cap_fov.load(std::memory_order_relaxed);
                if (tgt_ok && dl > 1e-3f && fov > 0.01f) {
                    d.x /= dl; d.y /= dl; d.z /= dl;                    // capture optical axis

                    // Capture position: along the ray at scope_cam_dist, per Scope.cpp.
                    const float cd = g_m_cap_dist.load(std::memory_order_relaxed);
                    const Vec3 cam{ro.x + d.x*cd, ro.y + d.y*cd, ro.z + d.z*cd};
                    Vec3 v{tgt.x - cam.x, tgt.y - cam.y, tgt.z - cam.z};
                    const float fwd = v.x*d.x + v.y*d.y + v.z*d.z;      // depth along the axis

                    if (fwd > 1.0f) {
                        // Camera right/up in UE world space. UE is Z-up, so world up is the
                        // reference; if the axis is near-vertical there is no stable right vector
                        // and we leave the quad centred rather than emit a spun basis.
                        // PROJECT IN THE FRAME THE IMAGE IS ACTUALLY RENDERED IN.
                        //
                        // The world-levelled basis below (right = aim x worldUp) is a THIRD frame
                        // and it is the bug: image coordinates are defined by the CAPTURE CAMERA'S
                        // axes, and the blit maps image axes onto the quad's local axes one to one,
                        // so projecting against world-up is wrong by the camera's entire roll --
                        // scope_cam_roll + the per-shape uv_roll + THE LIVE ROLL LOCK. The roll lock
                        // is why this was never fixable with a sign: it moves continuously to hold
                        // the image upright as the weapon cants, so the slide direction was wrong by
                        // an angle that changed as the gun rolled.
                        //
                        // Mode 1's chain was already right for exactly this reason -- it projects on
                        // the camera's own right/up -- which is why only the SLIDING mode misbehaved.
                        //
                        // scoperetroll: 0 keeps the old world-levelled basis for A/B, 1 uses the
                        // camera's, 2 uses the camera's with the vertical negated (the UE->XR up
                        // sign is not provable from here -- the same admission scoperetflipx/y makes).
                        const int   roll_mode = g_m_scope_ret_roll.load(std::memory_order_relaxed);
                        const bool  use_cam   = (roll_mode != 0) &&
                                                g_scope_cam_axes.load(std::memory_order_acquire);
                        const Vec3 wup{0.0f, 0.0f, 1.0f};
                        Vec3 rgt{d.y*wup.z - d.z*wup.y, d.z*wup.x - d.x*wup.z, d.x*wup.y - d.y*wup.x};
                        float rl = std::sqrt(rgt.x*rgt.x + rgt.y*rgt.y + rgt.z*rgt.z);
                        Vec3 cam_r{}, cam_u{};
                        if (use_cam) {
                            cam_r = Vec3{g_scope_cam_rx.load(std::memory_order_relaxed),
                                         g_scope_cam_ry.load(std::memory_order_relaxed),
                                         g_scope_cam_rz.load(std::memory_order_relaxed)};
                            cam_u = Vec3{g_scope_cam_ux.load(std::memory_order_relaxed),
                                         g_scope_cam_uy.load(std::memory_order_relaxed),
                                         g_scope_cam_uz.load(std::memory_order_relaxed)};
                            const float cl = std::sqrt(cam_r.x*cam_r.x + cam_r.y*cam_r.y +
                                                       cam_r.z*cam_r.z);
                            if (cl > 1e-3f) { rgt = cam_r; rl = cl; }
                        }
                        if (rl > 1e-3f) {
                            rgt.x /= rl; rgt.y /= rl; rgt.z /= rl;
                            Vec3 up{rgt.y*d.z - rgt.z*d.y, rgt.z*d.x - rgt.x*d.z,
                                    rgt.x*d.y - rgt.y*d.x};
                            if (use_cam) {
                                up = cam_u;
                                if (roll_mode == 2) { up.x = -up.x; up.y = -up.y; up.z = -up.z; }
                            }

                            // Image fractions in [-1,1]: tan(theta) / tan(fov/2).
                            const float half = std::tan(fov * 0.5f * 3.14159265f / 180.0f);
                            float u = ((v.x*rgt.x + v.y*rgt.y + v.z*rgt.z) / fwd) / half;
                            float w = ((v.x*up.x  + v.y*up.y  + v.z*up.z ) / fwd) / half;
                            // CLAMPED so a divergence larger than the field of view parks the
                            // reticule at the pane edge instead of flying off into the scene --
                            // "at the edge" is honest, "somewhere over there" is not.
                            if (u < -1.0f) u = -1.0f; else if (u > 1.0f) u = 1.0f;
                            if (w < -1.0f) w = -1.0f; else if (w > 1.0f) w = 1.0f;

                            // Apply along the PANE quad's own right/up, in XR space. Half-extent,
                            // because size is the full edge.
                            const float hx = pane.size_m * 0.5f;
                            const XrVector3f qr = xr_rotate(pane.pose.orientation,
                                                            XrVector3f{u * hx, 0.0f, 0.0f});
                            const XrVector3f qu = xr_rotate(pane.pose.orientation,
                                                            XrVector3f{0.0f, w * hx, 0.0f});
                            g_scope_ret_quad.pose.position.x += qr.x + qu.x;
                            g_scope_ret_quad.pose.position.y += qr.y + qu.y;
                            g_scope_ret_quad.pose.position.z += qr.z + qu.z;
                        }
                    }
                }
            }

            out[n_ours++] = (const XrCompositionLayerBaseHeader*)&g_scope_ret_quad;
        }
    }

    return n_ours;
}

// ---- RUNG 1: the API layer. THE SHIPPING ROUTE -----------------------------------------------
//
// Registered with the layer through set_end_frame_callback(). The layer calls this from inside its
// own xrEndFrame -- the same thread and the same moment the inline hook used, so nothing above
// needed rethinking; only the way it is reached has changed.
//
// Every clause of the contract in XrLayerAbi.h is honoured by construction: `info` is never
// touched, never more than `cap` pointers are written, the pointers are static, and nothing here
// calls back into the layer. The copy, the call-through and the fail-open retry are its job.
XRAPI_ATTR uint32_t XRAPI_CALL bridge_end_frame(XrSession session, const XrFrameEndInfo* info,
                                                const XrCompositionLayerBaseHeader** out,
                                                uint32_t cap, void* /*user*/) {
    return produce_layers(session, info, out, cap);
}

// ---- RUNG 2: our own inline hook on xrEndFrame. DEV ONLY, needs UEVRBackend.pdb ---------------
XRAPI_ATTR XrResult XRAPI_CALL hooked_end_frame(XrSession session, const XrFrameEndInfo* info) {
    // Fail-open on every path below: anything unexpected forwards the call untouched.
    if (g_end_frame_orig == nullptr) return XR_ERROR_RUNTIME_FAILURE;
    if (info == nullptr) return g_end_frame_orig(session, info);

    // The local array bound, and nothing more: room for UEVR's layers plus ours.
    constexpr uint32_t MAX_LAYERS = 32;
    const uint32_t room = (info->layerCount >= MAX_LAYERS) ? 0u : (MAX_LAYERS - info->layerCount);

    const XrCompositionLayerBaseHeader* ours[MAX_LAYERS];
    const uint32_t n = produce_layers(session, info, ours, room);
    if (n == 0) return g_end_frame_orig(session, info);

    const XrCompositionLayerBaseHeader* layers[MAX_LAYERS];
    for (uint32_t i = 0; i < info->layerCount; ++i) layers[i] = info->layers[i];
    // OURS LAST, therefore topmost -- the same ordering the API layer applies on its own rung.
    for (uint32_t i = 0; i < n; ++i) layers[info->layerCount + i] = ours[i];

    XrFrameEndInfo patched = *info;
    patched.layerCount = info->layerCount + n;
    patched.layers     = layers;

    const XrResult r = g_end_frame_orig(session, &patched);
    if (XR_SUCCEEDED(r)) {
        g_layers_submitted.fetch_add(1, std::memory_order_relaxed);
        return r;
    }

    // The runtime rejected the frame WITH our layer in it. Retry without it rather than dropping
    // the player's whole frame: a missing overlay is a cosmetic fault, a dropped frame is nausea.
    //
    // IS THIS FAILURE ACTUALLY OURS? Only retry the ones that are.
    //
    // The retry exists so a runtime that dislikes OUR composition layer costs the player an
    // overlay rather than a frame. That premise is sound and stays -- but it was applied to EVERY
    // XrResult, and most failures have nothing to do with us.
    //
    // MEASURED 2026-09-04. The runtime returned -30 XR_ERROR_TIME_INVALID, and UEVR logged the
    // arithmetic on the same tick: submitted 1053954453232700 vs frame_state 1053954469895900 --
    // a ~16.7 ms STALE DISPLAY TIME. Resubmitting cannot fix a stale time; it resubmits the same
    // one. What it did instead was call xrEndFrame a SECOND time on a frame the runtime had
    // already finished with, which is -37 XR_ERROR_CALL_ORDER_INVALID, and the next xrBeginFrame
    // then came back XR_FRAME_DISCARDED. One rejected frame became three failures inside 4 ms.
    //
    // So: retry only on codes attributable to the layer WE added, and pass everything else back
    // untouched. A stale display time, a call-order violation and a lost session are the
    // application's frame and the application's problem; our job is to not make them worse.
    //
    // Codes verified against src/thirdparty/openxr/openxr.h rather than remembered.
    const bool blame_our_layer = (r == XR_ERROR_LAYER_INVALID)
                              || (r == XR_ERROR_LAYER_LIMIT_EXCEEDED)
                              || (r == XR_ERROR_SWAPCHAIN_RECT_INVALID);
    if (!blame_our_layer) {
        static uint32_t passed = 0;
        if (passed < 5) {
            ++passed;
            logf("xrEndFrame failed (%d) for a reason that is NOT our composition layer -- passing "
                 "it back untouched. Resubmitting here used to turn this into a second xrEndFrame "
                 "on a finished frame (CALL_ORDER_INVALID) and a discarded next frame.", (int)r);
        }
        return r;
    }

    static uint32_t complained = 0;
    if (complained < 5) { ++complained; logf("xrEndFrame rejected with our layer (%d) -- retrying clean", (int)r); }
    return g_end_frame_orig(session, info);
}

bool install_hook() {
    if (g_hook_id >= 0) return true;
    if (!resolve_openxr()) return false;
    if (g_tier != XrAttachTier::BackendPdb) {
        // Belt and braces: nothing should reach the hook on another rung, and if something does,
        // register_inline_hook would happily install at whatever g_xr.end_frame holds. On the layer
        // rung that field is deliberately null.
        logf("refusing to install the inline hook on tier %s", tier_name(g_tier));
        return false;
    }

    auto* p = API::get()->param();
    if (p == nullptr || p->functions == nullptr || p->functions->register_inline_hook == nullptr) {
        logf("UEVR register_inline_hook unavailable -- layer off");
        return false;
    }

    void* orig = nullptr;
    const int id = p->functions->register_inline_hook((void*)g_xr.end_frame, (void*)&hooked_end_frame, &orig);
    if (id < 0 || orig == nullptr) {
        logf("register_inline_hook(xrEndFrame) FAILED (id %d)", id);
        return false;
    }
    g_hook_id        = id;
    g_end_frame_orig = (PFN_xrEndFrame)orig;

    // Say WHICH address won and how it was reached, once. An address nobody can name in the log is
    // an address nobody can check after a patch.
    static addrcascade::TierReporter reporter;
    if (reporter.changed((uintptr_t)g_xr.end_frame, "openxr_loader.dll!xrEndFrame (GetProcAddress)")) {
        logf("hook installed at %p -- INSTALLED IS NOT RUNNING; the watchdog decides",
             (void*)g_xr.end_frame);
    }
    return true;
}

// Register our producer with the API layer. No hook, no PDB, no address -- this is the whole of
// the shipping attachment.
bool install_bridge_callback() {
    if (g_bridge_cb) return true;
    const HaloVrLayerApi* api = xrbridge_api();
    if (api == nullptr || api->set_end_frame_callback == nullptr) return false;
    if (api->set_end_frame_callback(&bridge_end_frame, nullptr) != 1) {
        logf("the API layer REFUSED the end-frame callback -- %s", xrbridge_status());
        return false;
    }
    g_bridge_cb = true;
    logf("end-frame callback registered with the API layer. REGISTERED IS NOT RUNNING: the layer "
         "counts its own frames, and the watchdog below reads those rather than trusting this line.");
    return true;
}

// WAITING IS NOT FAILING, and conflating them is what would make the layer rung unusable: the
// OpenXR instance appears some way into startup, so a caller that latched Failed on the first
// "not yet" would turn every session into a permanent refusal. Only a hard no latches.
enum class Attach { Waiting, Ok, Failed };

Attach attach_submit_path() {
    if (g_hook_id >= 0 || g_bridge_cb) return Attach::Ok;
    if (!resolve_openxr()) return g_resolve_waiting ? Attach::Waiting : Attach::Failed;
    if (g_tier == XrAttachTier::ApiLayer) {
        return install_bridge_callback() ? Attach::Ok : Attach::Failed;
    }
    return install_hook() ? Attach::Ok : Attach::Failed;
}

void remove_hook() {
    if (g_bridge_cb) {
        // Clearing BLOCKS until any in-flight call has returned (XrLayerAbi.h spells out the
        // handshake), which is exactly what makes destroying the swapchain on the caller's next
        // line safe. The inline hook below offers no such guarantee -- this rung is the stronger
        // of the two on teardown, not merely the more portable one.
        const HaloVrLayerApi* api = xrbridge_api();
        if (api != nullptr && api->set_end_frame_callback != nullptr) {
            api->set_end_frame_callback(nullptr, nullptr);
        }
        g_bridge_cb = false;
    }
    if (g_hook_id >= 0) {
        auto* p = API::get()->param();
        if (p != nullptr && p->functions != nullptr && p->functions->unregister_inline_hook != nullptr) {
            p->functions->unregister_inline_hook(g_hook_id);
        }
        g_hook_id = -1;
    }
    g_end_frame_orig = nullptr;
}

}   // namespace

// ============================================================================================
// Public surface
// ============================================================================================

void xrlayer_notice_quad(int slot, const Vec3& world_pos, float world_cm, float hold_cm,
                         int priority, float world_cm_h) {
    if (!g_cfg.xr_layer) return;
    if (slot < 0 || slot >= XRLAYER_SLOTS) return;
    if (!(world_cm > 0.0f)) return;      // a zero-size quad is not a quad

    // ROLL, FOR THE MARKER SLOTS. xrlayerroll=0 keeps a quad world-upright instead of rolling with
    // the head, and a navpoint wants that at least as much as the reticule does -- a waypoint that
    // tilts when you tilt reads as the world tilting.
    //
    // SLOT-RANGE GATED, DELIBERATELY, AND THIS IS THE PART THAT WOULD BREAK IF GENERALISED:
    //   slot 0  is handled in xrlayer_notice_reticule (which calls this function), so doing it here
    //           as well would be redundant, not wrong.
    //   slots 1..8 are the markers -- this block.
    //   slot 9  is THE SCOPE PANE, whose orientation is owned by its caller: the pane is gun-mounted
    //           from the aim ray, and stamping a world-upright basis over it every tick would
    //           silently undo the entire point of the per-slot override. Never touch it here.
    if (slot >= XRLAYER_SLOT_NAV_BASE && slot < XRLAYER_SLOT_NAV_BASE + XRLAYER_NAV_COUNT) {
        if (g_cfg.xr_layer_roll == 0) {
            Vec3 fwd{}, up_unused{};
            if (xrlayer_view_basis(&fwd, &up_unused)) {
                xrlayer_set_quad_orientation(slot, fwd, Vec3{0.0f, 0.0f, 1.0f});
            }
        } else {
            xrlayer_clear_quad_orientation(slot);
        }
    }

    // TICK RATE: record the target only. No pose maths here -- see xrlayer_note_eye.
    g_tgt_x[slot].store(world_pos.x, std::memory_order_relaxed);
    g_tgt_y[slot].store(world_pos.y, std::memory_order_relaxed);
    g_tgt_z[slot].store(world_pos.z, std::memory_order_relaxed);
    g_tgt_cm[slot].store(world_cm, std::memory_order_relaxed);
    g_tgt_cm_h[slot].store(world_cm_h > 0.0f ? world_cm_h : 0.0f, std::memory_order_relaxed);
    g_tgt_hold[slot].store(hold_cm > 0.0f ? hold_cm : 0.0f, std::memory_order_relaxed);
    g_tgt_prio[slot].store(priority, std::memory_order_relaxed);
    g_tgt_live[slot].store(true, std::memory_order_relaxed);
    g_tgt_tick[slot].store(g_game_tick.load(std::memory_order_relaxed), std::memory_order_release);
}

void xrlayer_retire_quad(int slot) {
    if (slot < 0 || slot >= XRLAYER_SLOTS) return;
    // The CAPTURE is deliberately left alone. A navpoint that blinks out for a tick and comes back
    // -- which happens whenever the game re-hosts a marker's widget -- should not have to re-resolve
    // its render target through the whole chain to be drawn again.
    g_tgt_live[slot].store(false, std::memory_order_release);

    // THE ORIENTATION OVERRIDE, HOWEVER, IS CLEARED -- unlike the capture. The two are treated
    // differently on purpose: art is expensive to rebuild and harmless if briefly stale, whereas a
    // stale WEAPON ANGLE inherited by whatever occupies this slot next would point a quad somewhere
    // nothing asked for. Retirement is the one moment we know the slot's meaning may change.
    g_slot_orient_on[slot].store(false, std::memory_order_release);
}

void xrlayer_invalidate_capture(int slot) {
    if (slot < 0 || slot >= XRLAYER_SLOTS) return;
    // 0 is the module's existing "has never captured" value -- see the skip path in the capture,
    // which refreshes every slot that already has art but deliberately leaves a 0 alone so it
    // cannot pretend to be fresh. Returning the slot to 0 reuses that rule rather than adding a
    // second notion of staleness.
    g_slot_beat[slot].store(0, std::memory_order_release);
}

void xrlayer_set_quad_orientation(int slot, const Vec3& fwd_world, const Vec3& up_world) {
    if (slot < 0 || slot >= XRLAYER_SLOTS) return;
    g_slot_fwd_x[slot].store(fwd_world.x, std::memory_order_relaxed);
    g_slot_fwd_y[slot].store(fwd_world.y, std::memory_order_relaxed);
    g_slot_fwd_z[slot].store(fwd_world.z, std::memory_order_relaxed);
    g_slot_up_x[slot].store(up_world.x, std::memory_order_relaxed);
    g_slot_up_y[slot].store(up_world.y, std::memory_order_relaxed);
    g_slot_up_z[slot].store(up_world.z, std::memory_order_relaxed);
    // RELEASE LAST, so the render thread can never observe the flag set over half-written vectors.
    // The vectors themselves are relaxed: they are only meaningful once this flag is visible.
    g_slot_orient_on[slot].store(true, std::memory_order_release);
}

// Declare that this slot's next target is an OFFSET FROM THE EYE rather than a world point.
// See the g_tgt_headrel comment for why. Set it every publish, next to notice_quad, so the flag
// and the vector it describes can never disagree -- a stale flag reinterprets a world position as
// an offset, which puts the quad a whole world-origin away and looks like the layer vanishing.
void xrlayer_set_quad_head_relative(int slot, bool on) {
    if (slot < 0 || slot >= XRLAYER_SLOTS) return;
    g_tgt_headrel[slot].store(on, std::memory_order_relaxed);
}

// Clamped HERE as well as at the caller. The caller clamps because an off-pane impact point should
// pin to the rim rather than fly off; this clamps because a NaN or a wild value arriving on the
// submit thread would move the quad somewhere no log would explain, and a compositor pose is not
// the place to find out a divide went wrong.
void xrlayer_note_rig(const Vec3& pos, const Vec3& fwd, const Vec3& right, const Vec3& up) {
    g_rig_px.store(pos.x, std::memory_order_relaxed);
    g_rig_py.store(pos.y, std::memory_order_relaxed);
    g_rig_pz.store(pos.z, std::memory_order_relaxed);
    g_rig_fx.store(fwd.x, std::memory_order_relaxed);
    g_rig_fy.store(fwd.y, std::memory_order_relaxed);
    g_rig_fz.store(fwd.z, std::memory_order_relaxed);
    g_rig_rx.store(right.x, std::memory_order_relaxed);
    g_rig_ry.store(right.y, std::memory_order_relaxed);
    g_rig_rz.store(right.z, std::memory_order_relaxed);
    g_rig_ux.store(up.x, std::memory_order_relaxed);
    g_rig_uy.store(up.y, std::memory_order_relaxed);
    g_rig_uz.store(up.z, std::memory_order_relaxed);
    g_rig_have.store(true, std::memory_order_release);
}

// The offset is decomposed into the rig's basis HERE, on the game thread, so the submit thread does
// three multiply-adds against a fresh basis and nothing else. Storing a WORLD offset instead would
// leave the rotation uncancelled, which is the entire bug this exists for.
void xrlayer_set_quad_rig_relative(int slot, bool on, const Vec3& rig_pos, const Vec3& rig_fwd,
                                   const Vec3& rig_right, const Vec3& rig_up) {
    if (slot < 0 || slot >= XRLAYER_SLOTS) return;
    if (!on) { g_slot_rig_pos[slot].store(false, std::memory_order_relaxed); return; }
    const Vec3 w{g_tgt_x[slot].load(std::memory_order_relaxed) - rig_pos.x,
                 g_tgt_y[slot].load(std::memory_order_relaxed) - rig_pos.y,
                 g_tgt_z[slot].load(std::memory_order_relaxed) - rig_pos.z};
    g_slot_rp_x[slot].store(w.x * rig_fwd.x   + w.y * rig_fwd.y   + w.z * rig_fwd.z,
                            std::memory_order_relaxed);
    g_slot_rp_y[slot].store(w.x * rig_right.x + w.y * rig_right.y + w.z * rig_right.z,
                            std::memory_order_relaxed);
    g_slot_rp_z[slot].store(w.x * rig_up.x    + w.y * rig_up.y    + w.z * rig_up.z,
                            std::memory_order_relaxed);
    g_slot_rig_pos[slot].store(true, std::memory_order_release);
}

void xrlayer_set_quad_orientation_tracked(int slot, const Vec3& fwd_world, const Vec3& up_world,
                                          const Vec3& rig_fwd, const Vec3& rig_right,
                                          const Vec3& rig_up) {
    if (slot < 0 || slot >= XRLAYER_SLOTS) return;
    xrlayer_set_quad_orientation(slot, fwd_world, up_world);
    g_slot_rig_fx[slot].store(rig_fwd.x, std::memory_order_relaxed);
    g_slot_rig_fy[slot].store(rig_fwd.y, std::memory_order_relaxed);
    g_slot_rig_fz[slot].store(rig_fwd.z, std::memory_order_relaxed);
    g_slot_rig_rx[slot].store(rig_right.x, std::memory_order_relaxed);
    g_slot_rig_ry[slot].store(rig_right.y, std::memory_order_relaxed);
    g_slot_rig_rz[slot].store(rig_right.z, std::memory_order_relaxed);
    g_slot_rig_ux[slot].store(rig_up.x, std::memory_order_relaxed);
    g_slot_rig_uy[slot].store(rig_up.y, std::memory_order_relaxed);
    g_slot_rig_uz[slot].store(rig_up.z, std::memory_order_relaxed);
    // Released LAST, so the submit thread never sees the flag before the angles it needs.
    g_slot_rig_track[slot].store(true, std::memory_order_release);
}

void xrlayer_note_scope_cam_axes(const Vec3& right, const Vec3& up, bool valid) {
    if (valid) {
        g_scope_cam_rx.store(right.x, std::memory_order_relaxed);
        g_scope_cam_ry.store(right.y, std::memory_order_relaxed);
        g_scope_cam_rz.store(right.z, std::memory_order_relaxed);
        g_scope_cam_ux.store(up.x, std::memory_order_relaxed);
        g_scope_cam_uy.store(up.y, std::memory_order_relaxed);
        g_scope_cam_uz.store(up.z, std::memory_order_relaxed);
    }
    g_scope_cam_axes.store(valid, std::memory_order_release);
}

void xrlayer_set_scope_reticle_offset(float u, float v, bool valid) {
    if (!(u > -1.0f)) u = (u != u) ? 0.0f : -1.0f;   // NaN -> centre, else clamp low
    if (!(v > -1.0f)) v = (v != v) ? 0.0f : -1.0f;
    if (u > 1.0f) u = 1.0f;
    if (v > 1.0f) v = 1.0f;
    g_scope_ret_u.store(u, std::memory_order_relaxed);
    g_scope_ret_v.store(v, std::memory_order_relaxed);
    g_scope_ret_off_have.store(valid, std::memory_order_relaxed);
}

void xrlayer_clear_quad_orientation(int slot) {
    if (slot < 0 || slot >= XRLAYER_SLOTS) return;
    // A stale tracking flag would keep applying a delta against a rig rotation from whenever the
    // orientation was last published -- which grows without bound once nobody is updating it.
    g_slot_rig_track[slot].store(false, std::memory_order_relaxed);
    g_slot_orient_on[slot].store(false, std::memory_order_release);
}

void xrlayer_note_publish_gate(int gate) {
    g_publish_gate.store(gate, std::memory_order_relaxed);
}

void xrlayer_note_scope_ray(const Vec3& origin, const Vec3& target) {
    g_scope_ray_ox.store(origin.x, std::memory_order_relaxed);
    g_scope_ray_oy.store(origin.y, std::memory_order_relaxed);
    g_scope_ray_oz.store(origin.z, std::memory_order_relaxed);
    g_scope_ray_tx.store(target.x, std::memory_order_relaxed);
    g_scope_ray_ty.store(target.y, std::memory_order_relaxed);
    g_scope_ray_tz.store(target.z, std::memory_order_relaxed);
    g_scope_ray_have.store(true, std::memory_order_release);
}

// THE MONO VIEW POSITION, so a caller outside this file can publish a HEAD-RELATIVE target.
//
// Plugin.cpp already does this via its own static layer_anchor(), but that helper and the
// g_view_pos_* it reads are in Plugin.cpp's ANONYMOUS NAMESPACE, so nothing else can reach them --
// internal linkage, and no amount of extern spelling gets around it. Rather than move Plugin.cpp's
// globals (a file several sessions edit) this exposes the value the layer ALREADY receives every
// frame through xrlayer_note_eye, which is the same number by construction.
//
// MONO, never per-eye: g_eye_pos alternates left/right every frame, so capturing an offset against
// it and re-anchoring per eye throws one eye a full IPD sideways and flattens the stereo. That trap
// is documented at xrlayer_note_eye and this getter deliberately cannot expose the per-eye value.
bool xrlayer_mono_view_pos(Vec3* out_world) {
    if (out_world == nullptr) return false;
    if (!g_mono_view_have.load(std::memory_order_acquire)) return false;
    *out_world = Vec3{g_mono_view_x.load(std::memory_order_relaxed),
                      g_mono_view_y.load(std::memory_order_relaxed),
                      g_mono_view_z.load(std::memory_order_relaxed)};
    return true;
}

bool xrlayer_view_basis(Vec3* fwd_world, Vec3* up_world) {
    if (fwd_world == nullptr || up_world == nullptr) return false;
    if (!g_view_have.load(std::memory_order_acquire)) return false;

    // UE rotator -> UE world basis, through the same rotator_to_quat() that ue_offset_to_xr() uses
    // to build the camera basis it rotates every quad offset by. Same function, same convention, so
    // a caller feeding this straight back into xrlayer_set_quad_orientation must land on identity --
    // which is exactly what makes it usable as a self-check rather than merely a convenience.
    const Quat q = rotator_to_quat(g_view_pitch.load(std::memory_order_relaxed),
                                   g_view_yaw.load(std::memory_order_relaxed),
                                   g_view_roll.load(std::memory_order_relaxed));
    *fwd_world = quat_rotate(q, Vec3{1.0f, 0.0f, 0.0f});   // UE forward is +X
    *up_world  = quat_rotate(q, Vec3{0.0f, 0.0f, 1.0f});   // UE up      is +Z
    return true;
}

bool xrlayer_pane_configure(int cell_px) {
    if (cell_px < 0) return false;
    g_pane_req = cell_px;

    // BEFORE BRING-UP this is all there is to do -- build_atlas_layout() will read g_pane_req and
    // decide, and its decision (including a refusal) is logged there.
    //
    // AFTER bring-up the atlas is already a live swapchain, and resizing it would mean destroying
    // and recreating the swapchain, the allocator ring and the fence underneath a submit thread that
    // is reading them. That is precisely the borrowed-resource lifetime hazard this module has
    // already crashed on three times, and a scope pane is not worth re-opening it. So a late call
    // records the request for the NEXT bring-up and says plainly that it did not take effect now,
    // rather than half-applying or pretending to succeed.
    if (g_pane_dim == cell_px) return cell_px == 0 || g_cell[XRLAYER_SLOT_PANE].dim == cell_px;
    // THE TEST IS "IS THE ATLAS ALREADY BUILT", NOT "IS THE LAYER SUBMITTING", and the difference
    // is a real 92 ms window that silently cost the pane its cell.
    //
    // MEASURED 2026-09-06:
    //     14:20:41.894  atlas built -- pane 0px (none)
    //     14:20:41.901  ARMED
    //     14:20:41.986  pane cell requested 1024px -> reported "accepted for this bring-up"
    //
    // xrlayer_live() only becomes true once frames are actually being submitted, which is LATER
    // than the atlas being built. A request landing in between took this early-return path and was
    // told it had been accepted, while the atlas it needed to be part of had already been decided
    // without it. The pane then reported "no atlas cell" forever and the two logs did not appear to
    // be about the same thing. A false "accepted" is worse than a refusal: it sends the reader to
    // look somewhere else entirely.
    if (g_sc_w > 0 || g_state.load(std::memory_order_relaxed) != State::Off) {
        logf("pane cell %dpx requested while the layer is already up -- recorded for the next "
             "bring-up; the atlas is NOT resized under a running submit thread.", cell_px);
        return false;
    }
    return true;
}

// A marker slot's hosted widget CLASS just changed (game thread, from navw_host_class). Its atlas
// cell still holds the previous widget's art until the render target redraws and the next capture
// copies it in, so mark the slot incoherent until a capture lands on a strictly later tick. See the
// g_slot_host_tick / slot_cell_coherent comment above -- this is the write half of that gate.
void xrlayer_notice_rehost(int slot) {
    if (slot < 0 || slot >= XRLAYER_SLOTS) return;
    g_slot_host_tick[slot].store(g_game_tick.load(std::memory_order_relaxed),
                                 std::memory_order_relaxed);
}

// Watchdog state for the scope-hide of the compositor reticule. File scope rather than function
// statics because BOTH branches touch it: the hiding branch arms it, the non-hiding branch clears it.
static uint32_t s_hide_since = 0;
static bool     s_warned     = false;

void xrlayer_notice_reticule(const Vec3& world_pos, float apparent_scale) {
    if (!g_cfg.xr_layer) return;

    // ---- ONE RETICULE AT A TIME. Hide this one while the scope pane is up.
    //
    // The compositor reticule draws over everything by construction, and the scope pane carries its
    // own reticule at the magnified range. Both are CORRECT -- they simply sit at different
    // distances, and having two crosshairs a few degrees apart reads as noise rather than as aim.
    //
    // RETIRED EXPLICITLY, not merely left un-noticed. A quad that stops being noticed only fades on
    // the retirement grace (dozens of ticks), which would leave the second reticule hanging around
    // for a second or so every time the scope comes up -- exactly the artefact this removes. Note
    // the comment at report_dark_reason: nothing in the tree had ever retired slot 0 before, so this
    // is the first caller to do it.
    if (g_cfg.xr_layer_hide_scope && g_scope_active.load(std::memory_order_relaxed)) {
        // STUCK-HIDDEN WATCHDOG.
        //
        // Recovery works by construction: retire only clears g_tgt_live, and the next call that gets
        // past this gate re-arms the slot through notice_quad. So the ONLY way the crosshair stays
        // gone is g_scope_active being stuck TRUE with no scope on screen.
        //
        // That should not happen -- Scope.cpp forces it false on a stale aim ray, on a weapon swap
        // and at teardown -- but "should not" is how a reticule goes missing for a whole session with
        // nothing in the log. Deliberately NOT cross-checked against s_pane_shown: that variable
        // carries two comments saying we are not its only writer, so a second source of truth here
        // would desync and cause the exact bug it was added to prevent.
        //
        // So this does not override anything. It SAYS SO, once, if the hide has been continuous for
        // implausibly long -- turning a silent missing crosshair into a log line naming the cause
        // and the escape hatch.
        {
            const uint32_t now = g_game_tick.load(std::memory_order_relaxed);
            if (s_hide_since == 0) { s_hide_since = now; s_warned = false; }
            // ~32 Hz tick, so 2000 ticks is about a minute of unbroken scoping.
            else if (!s_warned && (now - s_hide_since) > 2000u) {
                s_warned = true;
                API::get()->log_info(
                    "[Halo-CampE-UEVR] XRLAYER: the compositor reticule has been hidden for the scope "
                    "for ~%u ticks unbroken. That is legitimate if you have genuinely been scoped that "
                    "long. If the scope is NOT up, g_scope_active is stuck true and the crosshair will "
                    "not return -- set xrlayerhidescope=0 to get it back and say so.",
                    now - s_hide_since);
            }
        }
        xrlayer_retire_quad(XRLAYER_SLOT_RETICULE);
        return;
    }
    // Not hiding this pass -- reset the watchdog so the next scope-in starts a fresh window, and
    // so a warning can fire again on a genuinely new stuck episode rather than only once per run.
    s_hide_since = 0;
    s_warned     = false;

    const float s = (apparent_scale > 0.0f) ? apparent_scale : 1.0f;

    // THE RETICULE'S WORLD SIZE IS COMPUTED HERE, not inside compute_pose, and xrlayersize is
    // folded in on the way. compute_pose is shared with eight navpoint markers now, and a shared
    // function that reads one caller's config keys is a coupling that only shows up when the other
    // caller is retuned. Same numbers as before -- aim_widget_draw x aim_widget_scale x the
    // 1/distance compensation, times the xrlayersize multiplier -- just computed on this side of
    // the seam.
    const float widget_cm = g_cfg.aim_widget_draw * g_cfg.aim_widget_scale * s * g_cfg.xr_layer_size;
    xrlayer_notice_quad(XRLAYER_SLOT_RETICULE, world_pos, widget_cm, /*hold_cm=*/0.0f,
                        /*priority=*/0);

    // ROLL. Every quad is head-ORIENTED by default, so tilting your head tilts the crosshair. With
    // xrlayerroll=0 the reticule instead keeps world up as its up reference: it still faces you,
    // but it no longer rolls. This is the per-slot orientation override built for the scope pane,
    // pointed at slot 0 -- no new frame maths crosses the UE/XR boundary, because the vectors go
    // through the same ue_offset_to_xr the position already uses.
    if (g_cfg.xr_layer_roll == 0) {
        Vec3 fwd{}, up_unused{};
        if (xrlayer_view_basis(&fwd, &up_unused)) {
            xrlayer_set_quad_orientation(XRLAYER_SLOT_RETICULE, fwd, Vec3{0.0f, 0.0f, 1.0f});
        }
        // No view composed yet -> leave whatever is set; the next tick with a view fixes it.
    } else {
        xrlayer_clear_quad_orientation(XRLAYER_SLOT_RETICULE);
    }
}

bool xrlayer_slot_ready(int slot) {
    if (slot < 0 || slot >= XRLAYER_SLOTS) return false;
    if (!g_live.load(std::memory_order_relaxed)) return false;
    if (g_source_override.load(std::memory_order_relaxed) == nullptr) return false;
    const uint64_t beat = g_slot_beat[slot].load(std::memory_order_relaxed);
    if (beat == 0) return false;
    // Cell/host coherence, the same gate the submit-thread append loop applies: a slot whose cell
    // was captured before its last re-host is NOT ready. Plugin.cpp reads this for `on_layer`, so a
    // false here keeps the in-scene marker visible for the one incoherent frame instead of hiding it
    // behind a compositor cell that still holds the previous navpoint's art.
    if (!slot_cell_coherent(slot)) return false;
    const uint64_t now = GetTickCount64();
    return now >= beat && (now - beat) <= (uint64_t)g_m_hold_ms.load(std::memory_order_relaxed);
}

int xrlayer_cell_dim(int slot) {
    if (slot < 0 || slot >= XRLAYER_SLOTS) return 0;
    return (int)g_cell[slot].dim;
}

void xrlayer_note_eye(int eye_index, const Vec3& eye_pos, const Vec3& mono_view_pos,
                      float view_yaw, float view_pitch, float view_roll) {
    if (!g_cfg.xr_layer) return;
    if (g_state.load(std::memory_order_relaxed) != State::Armed) return;
    if (eye_index < 0 || eye_index > 1) return;

    // Remember this eye and use the MIDPOINT of the two as the head position.
    //
    // Not one eye's position: the callback fires once per eye, so computing from whichever eye ran
    // would alternate the quad between two points half an IPD apart every frame -- a shimmer, and a
    // far more obvious artefact than the constant offset it would be replacing. The midpoint IS the
    // head, which is what get_pose() reports, so the correspondence becomes exact rather than
    // merely close. Publishing on both callbacks is then harmless: the answer is identical either
    // time.
    g_eye_x[eye_index].store(eye_pos.x, std::memory_order_relaxed);
    g_eye_y[eye_index].store(eye_pos.y, std::memory_order_relaxed);
    g_eye_z[eye_index].store(eye_pos.z, std::memory_order_relaxed);
    g_eye_have[eye_index].store(true, std::memory_order_release);

    // KEEP THE COMPOSED VIEW ROTATOR, for xrlayer_view_basis(). Retained HERE rather than
    // reconstructed by a consumer, because this is the one place all THREE components are known
    // together and known current. Plugin.cpp publishes yaw and pitch as globals but deliberately
    // does NOT publish roll -- so anything rebuilding the basis from those would be rebuilding it
    // roll-less, which is precisely the bug this module already found and fixed in a headset
    // (see the ROLL MUST BE PASSED note on xrlayer_note_eye's declaration). Storing it here means a
    // consumer cannot reproduce that bug by accident.
    g_view_yaw.store(view_yaw, std::memory_order_relaxed);
    g_view_pitch.store(view_pitch, std::memory_order_relaxed);
    g_view_roll.store(view_roll, std::memory_order_relaxed);
    g_view_have.store(true, std::memory_order_release);

    Vec3 head{};
    if (g_eye_have[0].load(std::memory_order_acquire) && g_eye_have[1].load(std::memory_order_acquire)) {
        head.x = 0.5f * (g_eye_x[0].load(std::memory_order_relaxed) + g_eye_x[1].load(std::memory_order_relaxed));
        head.y = 0.5f * (g_eye_y[0].load(std::memory_order_relaxed) + g_eye_y[1].load(std::memory_order_relaxed));
        head.z = 0.5f * (g_eye_z[0].load(std::memory_order_relaxed) + g_eye_z[1].load(std::memory_order_relaxed));
    } else {
        // Only one eye has ever reported (2D-screen mode, or the very first frame). Use it rather
        // than drawing nothing: the error is half an IPD and CONSTANT, which is not the drift this
        // function exists to remove.
        head = eye_pos;
    }

    // EVERY LIVE SLOT, from THIS eye. The reticule's reason for being here applies to the markers
    // with more force, not less: the navpoint lane already re-places its in-scene markers in this
    // exact callback because "at a 4 m draw distance the eye-to-objective direction changes
    // materially between 32 Hz ticks", and a compositor quad computed on the tick would carry the
    // same error with none of the in-scene lane's ability to hide it.
    // BEFORE THE LOOP, not inside it. The loop skips slots that are not live, so a session with
    // nothing published would never have set this -- and a submit-thread reader would then undo a
    // head-relative offset with a stale or zero camera, which is worse than not undoing it at all.
    g_mono_view_x.store(mono_view_pos.x, std::memory_order_relaxed);
    g_mono_view_y.store(mono_view_pos.y, std::memory_order_relaxed);
    g_mono_view_z.store(mono_view_pos.z, std::memory_order_relaxed);
    g_mono_view_have.store(true, std::memory_order_release);

    Frame f{};
    for (int s = 0; s < XRLAYER_SLOTS; ++s) {
        if (!g_tgt_live[s].load(std::memory_order_relaxed)) continue;
        Snapshot& sn = f.slot[s];
        sn.tick  = g_tgt_tick[s].load(std::memory_order_acquire);
        sn.prio  = g_tgt_prio[s].load(std::memory_order_relaxed);
        // Per-slot orientation. The caller's UE world directions go through ue_offset_to_xr() --
        // the same mapping the position uses -- and only then become a quaternion. cm_per_m is
        // irrelevant to a direction (xr_look_rotation normalises), so the value passed is immaterial
        // as long as it is non-zero; 1.0f keeps the intent obvious.
        //
        // A degenerate pair leaves orient_p null, i.e. head-oriented. That is the same fail-soft the
        // rest of this module uses: a scope at the wrong angle is a bug you can see and report, a
        // scope built from a sheared basis is one you describe as "blurry".
        XrQuaternionf        orient{};
        const XrQuaternionf* orient_p = nullptr;
        if (g_slot_orient_on[s].load(std::memory_order_acquire)) {
            const Vec3 fwd_w{g_slot_fwd_x[s].load(std::memory_order_relaxed),
                             g_slot_fwd_y[s].load(std::memory_order_relaxed),
                             g_slot_fwd_z[s].load(std::memory_order_relaxed)};
            Vec3 up_w {g_slot_up_x[s].load(std::memory_order_relaxed),
                       g_slot_up_y[s].load(std::memory_order_relaxed),
                       g_slot_up_z[s].load(std::memory_order_relaxed)};

            // RIG-TRACKED: rotate the published vectors by how far the rig has turned SINCE the
            // tick that measured them. Without this a weapon-mounted quad is placed from a ~32 Hz
            // sample of a transform the engine recomposes every frame, so it tracks correctly while
            // still and trails while you rotate -- which is precisely how it was reported.
            //
            // delta = render_rig * inverse(tick_rig), applied in UE world space BEFORE the UE->XR
            // conversion below, so the correction and the conversion stay in the order the rest of
            // this file assumes.
            Vec3 fwd_c = fwd_w;
            if (g_slot_rig_track[s].load(std::memory_order_acquire) &&
                g_rig_have.load(std::memory_order_acquire)) {
                // A PURE CHANGE OF BASIS, with no decomposition anywhere: express the vector
                // in the TICK's rig basis (three dot products), then rebuild it in the RENDER
                // basis. Equivalent to rotating by render x inverse(tick), but built only from
                // orthonormal axes, so there is no Euler ordering to get wrong and no singularity
                // near vertical -- which is what made this jitter on roll the first time.
                const Vec3 tf{g_slot_rig_fx[s].load(std::memory_order_relaxed),
                              g_slot_rig_fy[s].load(std::memory_order_relaxed),
                              g_slot_rig_fz[s].load(std::memory_order_relaxed)};
                const Vec3 tr{g_slot_rig_rx[s].load(std::memory_order_relaxed),
                              g_slot_rig_ry[s].load(std::memory_order_relaxed),
                              g_slot_rig_rz[s].load(std::memory_order_relaxed)};
                const Vec3 tu{g_slot_rig_ux[s].load(std::memory_order_relaxed),
                              g_slot_rig_uy[s].load(std::memory_order_relaxed),
                              g_slot_rig_uz[s].load(std::memory_order_relaxed)};
                const Vec3 nf{g_rig_fx.load(std::memory_order_relaxed),
                              g_rig_fy.load(std::memory_order_relaxed),
                              g_rig_fz.load(std::memory_order_relaxed)};
                const Vec3 nr{g_rig_rx.load(std::memory_order_relaxed),
                              g_rig_ry.load(std::memory_order_relaxed),
                              g_rig_rz.load(std::memory_order_relaxed)};
                const Vec3 nu{g_rig_ux.load(std::memory_order_relaxed),
                              g_rig_uy.load(std::memory_order_relaxed),
                              g_rig_uz.load(std::memory_order_relaxed)};
                auto rebase = [&](const Vec3& v) {
                    const float a = v.x * tf.x + v.y * tf.y + v.z * tf.z;
                    const float b = v.x * tr.x + v.y * tr.y + v.z * tr.z;
                    const float c = v.x * tu.x + v.y * tu.y + v.z * tu.z;
                    return Vec3{nf.x * a + nr.x * b + nu.x * c,
                                nf.y * a + nr.y * b + nu.y * c,
                                nf.z * a + nr.z * b + nu.z * c};
                };
                fwd_c = rebase(fwd_c);
                up_w  = rebase(up_w);
            }
            const XrVector3f fwd_l = ue_offset_to_xr(fwd_c, view_yaw, view_pitch, view_roll, 1.0f);
            const XrVector3f up_l  = ue_offset_to_xr(up_w,  view_yaw, view_pitch, view_roll, 1.0f);
            if (xr_look_rotation(fwd_l, up_l, &orient)) orient_p = &orient;
        }

        // HEAD-RELATIVE TARGETS ARE RE-ANCHORED HERE, at render rate, which is the whole point:
        // compute_pose immediately subtracts `head` again, so d_world comes out as the stored
        // offset exactly and the tick-old eye that offset was measured against never enters the
        // arithmetic. A world target (a navpoint) skips this and keeps the render-rate subtraction
        // it wants.
        Vec3 tgt{g_tgt_x[s].load(std::memory_order_relaxed),
                 g_tgt_y[s].load(std::memory_order_relaxed),
                 g_tgt_z[s].load(std::memory_order_relaxed)};
        // RE-ANCHOR AGAINST THE MONO CAMERA, NEVER AGAINST `head`.
        //
        // `head` is the RENDERED EYE: this callback runs once per eye, and the two differ by the
        // IPD. Using it here was a stereo bug, shipped and caught in a headset. The offset is
        // captured once on the game tick against whichever eye happened to be written last, then
        // added back per eye -- so one eye landed a full IPD (~6.4 cm) to the side. Worse, it made
        // d_world identical in both eyes, which DESTROYS PARALLAX: the quad loses its real depth
        // and stops being at the distance it is supposed to mark.
        //
        // The mono pre-hook camera is the same value for both eyes, so the offset survives the
        // round trip unchanged and compute_pose's per-eye subtraction still produces parallax.
        if (g_tgt_headrel[s].load(std::memory_order_relaxed)) {
            tgt.x += mono_view_pos.x; tgt.y += mono_view_pos.y; tgt.z += mono_view_pos.z;
        }

        // RIG-RELATIVE: rebuild the world point from the rig AS IT IS THIS FRAME.
        //
        // The offset is held in the rig's own basis, so rotating the weapon carries the quad with it
        // instead of leaving it on a ~32 Hz sample of an arc. MEASURED 2026-09-07: with no
        // orientation published at all (scopelayer=3) the quad still hopped while the controller
        // rotated, which isolated the fault to POSITION -- and head-relative could never fix it,
        // because it cancels only what the target shares with the EYE, and a weapon swinging about
        // a stationary head shares nothing.
        //
        // Mutually exclusive with head-relative by construction: the rig frame already carries the
        // body's translation, so applying both would try to remove the same motion twice.
        if (g_slot_rig_pos[s].load(std::memory_order_acquire) &&
            g_rig_have.load(std::memory_order_acquire)) {
            const float a = g_slot_rp_x[s].load(std::memory_order_relaxed);
            const float b = g_slot_rp_y[s].load(std::memory_order_relaxed);
            const float c = g_slot_rp_z[s].load(std::memory_order_relaxed);
            tgt = Vec3{g_rig_px.load(std::memory_order_relaxed)
                         + g_rig_fx.load(std::memory_order_relaxed) * a
                         + g_rig_rx.load(std::memory_order_relaxed) * b
                         + g_rig_ux.load(std::memory_order_relaxed) * c,
                       g_rig_py.load(std::memory_order_relaxed)
                         + g_rig_fy.load(std::memory_order_relaxed) * a
                         + g_rig_ry.load(std::memory_order_relaxed) * b
                         + g_rig_uy.load(std::memory_order_relaxed) * c,
                       g_rig_pz.load(std::memory_order_relaxed)
                         + g_rig_fz.load(std::memory_order_relaxed) * a
                         + g_rig_rz.load(std::memory_order_relaxed) * b
                         + g_rig_uz.load(std::memory_order_relaxed) * c};
        }

        sn.valid = compute_pose(tgt,
                                head, view_yaw, view_pitch, view_roll,
                                g_tgt_cm[s].load(std::memory_order_relaxed),
                                g_tgt_hold[s].load(std::memory_order_relaxed),
                                &sn.pose, &sn.size_m, &sn.ang, orient_p);
        // Second extent, through the SAME cm_per_m compute_pose used for the first -- read back off
        // size_m rather than re-deriving, so the two axes cannot end up on different scales if the
        // world-scale handling in compute_pose ever changes.
        const float cm_w = g_tgt_cm[s].load(std::memory_order_relaxed);
        const float cm_h = g_tgt_cm_h[s].load(std::memory_order_relaxed);
        sn.size_h_m = (cm_h > 0.0f && cm_w > 0.0f) ? (sn.size_m * (cm_h / cm_w)) : 0.0f;
    }
    publish(f);
}

// The publish-gate breadcrumb, rendered. Kept next to the reporter because the two only make sense
// together, and split apart the codes drift from their text.
//
// GATE 6 IS THE RESET VALUE, and it is the whole reason this is trustworthy. The first version of
// this breadcrumb was only ever WRITTEN inside the aim_reticule block -- so when the fault stopped
// that block from running at all, the last value ("0: reached the publish") simply stayed there and
// the diagnostic reported that everything was fine while the reticule was demonstrably dark. A
// breadcrumb that is not reset every tick does not degrade to "unknown", it degrades to a LIE.
void describe_gate(char* out, size_t n) {
    static const char* const kGate[] = {
        "0 = reached the publish (healthy)",
        "1 = the feature is OFF (aimreticule=0)",
        "2 = NO RIG -- g_rig_component null, or K2_GetComponentLocation on it failed",
        "3 = NO ORIGIN -- g_rig_parent null, or K2_GetComponentLocation on it failed (the arm rig's "
            "attach parent)",
        "4 = NO TRACE TARGET this tick",
        "5 = CONTROLLER POSITION DEAD or no rig neutral -- Plugin.cpp's position_dead gate held the "
            "rig at origin and took the branch that skips the whole reticule/trace/publish group. "
            "The controller is giving ROTATION but not POSITION (see CONTROLLER POSITION MISSING); "
            "the reticule needs no position at all, so this gate is stopping a lane that did not "
            "depend on what failed",
        "6 = the publish site was NOT REACHED this tick (reset value -- something above the "
            "aim_reticule block returned early)",
    };
    const int g = g_publish_gate.load(std::memory_order_relaxed);
    _snprintf_s(out, n, _TRUNCATE, " %s",
                (g >= 0 && g < (int)(sizeof(kGate) / sizeof(kGate[0]))) ? kGate[g] : "unknown");
}

// WHY IS THE LAYER DARK? -- the question this module could not answer about itself.
//
// Reported 2026-08-28: "it shows for a few seconds, turns off and won't come back". The state line
// said `live=0 slots=0x000` and stopped there, which names the SYMPTOM and not one thing about the
// cause -- and worse, `slots` is derived from `live` (xrlayer_slot_ready early-outs on it), so the
// two zeros look like independent corroboration and are one fact printed twice. Diagnosing it took
// an evening of log archaeology to reach "the game thread stopped publishing a target", which this
// module knew all along and never said.
//
// So it says it now. Game thread, once per tick, at most two lines a minute: which gate is holding
// slot 0 down, in the order the submit thread applies them. NOT behind HALO_VR_DEV -- a player
// reporting "the reticule vanished" should be able to send a log that already contains the answer,
// and the cost is one branch per tick over six atomics.
//
// Deliberately reports SLOT 0 ONLY. The markers come and go by design (a navpoint that is not on
// screen is not a fault), so reporting them would be noise that trains the reader to ignore the
// line. The reticule going dark is always worth knowing about.
void report_dark_reason(uint32_t tick) {
    static uint32_t s_reason       = 0xFFFFFFFFu;
    static uint64_t s_when         = 0;
    // When g_live first went false, or 0 while it is true. Function scope so the recovery path can
    // clear it -- left inside the branch it would latch at the first dip and every later blip would
    // instantly qualify as "held for 2 s".
    static uint64_t s_live_lost_at = 0;

    if (!g_cfg.xr_layer) return;
    const State st = g_state.load(std::memory_order_relaxed);
    if (st != State::Armed) return;          // bring-up has its own reporting

    const int  s     = XRLAYER_SLOT_RETICULE;
    const bool live  = g_live.load(std::memory_order_relaxed);
    const uint32_t gt = g_game_tick.load(std::memory_order_relaxed);

    uint32_t    reason = 0;
    const char* text   = nullptr;
    char        extra[400] = {0};   // must hold the longest gate string plus the age suffix

    if (!g_tgt_live[s].load(std::memory_order_relaxed)) {
        reason = 1;
        text   = "NO TARGET PUBLISHED -- xrlayer_notice_reticule() is not being called. NOT a "
                 "compositor fault; the gate is upstream. Reported gate";
        describe_gate(extra, sizeof(extra));
    } else {
        const uint32_t age = gt - g_tgt_tick[s].load(std::memory_order_acquire);
        if (age > 8) {
            // REASON 2 IS THE ONE THIS FAULT ACTUALLY REPORTS UNDER, so it must carry the gate.
            //
            // Reason 1 requires !g_tgt_live[0], and NOTHING EVER RETIRES SLOT 0 -- every
            // xrlayer_retire_quad call in the tree is a marker slot or the scope pane. So
            // g_tgt_live[0] latches true at first publish and stays true forever, which made the
            // gate breadcrumb unreachable for the reticule in the first version of this reporter:
            // the interesting half of the diagnostic was wired to a branch that cannot be taken.
            // A stopped publish presents here, as a target that is simply never refreshed.
            reason = 2;
            text   = "TARGET STALE -- it was published once and has stopped being refreshed, so the "
                     "drive path has stopped running. Gate";
            char g[200] = {0};
            describe_gate(g, sizeof(g));
            _snprintf_s(extra, sizeof(extra), _TRUNCATE, "%s (age %u ticks, limit 8)", g, age);
        } else if (g_cell[s].dim <= 0) {
            reason = 3;
            text   = "NO ATLAS CELL for slot 0 -- the layout fell back at bring-up";
        } else {
            const uint64_t beat = g_slot_beat[s].load(std::memory_order_relaxed);
            const uint64_t now  = GetTickCount64();
            const uint64_t hold = (uint64_t)g_m_hold_ms.load(std::memory_order_relaxed);
            if (beat == 0 || now < beat || (now - beat) > hold) {
                reason = 4;
                text   = "CAPTURE STALE -- the cell holds art but it stopped being refreshed";
            } else if (!live) {
                // REASON 5 MUST PERSIST BEFORE IT IS BELIEVED. g_live is recomputed once per
                // watchdog window from `submitted`, which is read with exchange(0) -- so for the
                // moment between that reset and the next appended frame, live reads false while
                // nothing is wrong. Measured 2026-08-28: this fired twice in a healthy sim run and
                // recovered in 40 ms both times.
                //
                // A false alarm here is worse than silence: this line exists so that a player's log
                // already contains the answer, and a reason that cries wolf every window boundary
                // is one the reader learns to skip. So require it to hold for longer than a window.
                const uint64_t now5 = GetTickCount64();
                if (s_live_lost_at == 0) s_live_lost_at = now5;
                if (now5 - s_live_lost_at < 2000) return;      // not yet worth saying
                reason = 5;
                text   = "GATES PASS BUT NOTHING WAS SUBMITTED for >2 s -- the xrEndFrame hook is "
                         "not appending. Check the watchdog line and whether the HMD reads active";
            } else {
                reason = 0;              // drawing normally
            }
        }
    }

    if (reason == 0) {
        s_live_lost_at = 0;              // healthy: restart the reason-5 persistence timer
        if (s_reason != 0) {             // announce the recovery too, once
            s_reason = 0; s_when = GetTickCount64();
            logf("reticule layer RECOVERED at tick %u -- drawing again.", tick);
        }
        return;
    }

    const uint64_t now = GetTickCount64();
    if (s_reason == reason && (now - s_when) < 30000) return;
    s_reason = reason;
    s_when   = now;
    logf("reticule layer DARK (reason %u): %s%s.", reason, text, extra);
}

void xrlayer_tick() {
    const uint32_t tick = g_game_tick.fetch_add(1, std::memory_order_relaxed) + 1;
    report_dark_reason(tick);
    // RESET THE BREADCRUMB EVERY TICK, immediately after it has been read. Without this it only
    // ever holds the last value anyone bothered to write -- and since every write lives inside the
    // aim_reticule block, a fault that stops that block from running would leave "0 = reached the
    // publish" standing and the diagnostic would report health while the reticule was dark. Reset
    // to "not reached", so silence from the publish site reads as silence.
    g_publish_gate.store(6, std::memory_order_relaxed);

    // ---- mirror the config, so the submit thread never touches g_cfg ----
    Mirror m{};
    m.enabled  = g_cfg.xr_layer;
    m.space    = g_cfg.xr_layer_space;
    m.size_m   = g_cfg.xr_layer_size;
    m.cm_per_m = g_cfg.xr_layer_cm_per_m;
    m.alpha    = g_cfg.xr_layer_alpha;
    m.cr       = g_cfg.xr_layer_cr;
    m.cg       = g_cfg.xr_layer_cg;
    m.cb       = g_cfg.xr_layer_cb;
    m.verbose  = g_cfg.xr_layer_log;
    m.budget   = g_cfg.xr_layer_budget;
    m.scope_ret      = g_cfg.xr_layer_scope_reticle;
    m.scope_ret_size = g_cfg.xr_layer_scope_reticle_size;
    m.scope_ret_depth_cm = g_cfg.xr_layer_scope_reticle_depth;
    m.scope_ret_roll     = g_cfg.scope_ret_roll;
    m.cap_fov_deg    = (g_cfg.scope_zoom > 1.0f) ? (g_cfg.scope_base_fov / g_cfg.scope_zoom)
                                                 : g_cfg.scope_base_fov;
    m.cap_dist_cm    = g_cfg.scope_cam_dist;

    const Mirror prev = mirror_load();
    mirror_store(m);

    // xrlayernav NO LONGER rebuilds anything. The atlas reserves the marker cells unconditionally
    // (build_atlas_layout), so this key only decides per frame which cells are submitted -- it does
    // not change the swapchain, so it needs no teardown. The old teardown-and-re-arm here is what
    // stranded the reticule on the generated ring after a runtime toggle; removing it is the fix.

    // Its own atomic rather than a Mirror field: the submit thread reads it on the freshness gate,
    // which runs before the Mirror is even loaded, and a torn value here is one frame of a slightly
    // different hold window -- harmless. Live-tunable like everything else in the mirror.
    g_m_hold_ms.store((uint32_t)g_cfg.xr_layer_hold_ms, std::memory_order_relaxed);

    // Resolve the UE-cm-per-VR-metre factor HERE, on the game thread, and cache it for the submit
    // thread. Only when it could have changed: the override key moving, or the first tick after the
    // feature is enabled. VR_WorldScale is a UEVR mod value the player edits, not a per-frame
    // quantity.
    if (m.enabled && (!prev.enabled || prev.cm_per_m != m.cm_per_m)) {
        const float cmpm = resolve_cm_per_metre();
        g_cm_per_m.store(cmpm, std::memory_order_relaxed);
        logf("UE cm per VR metre = %.1f (%s). UNMEASURED -- if the ring sits at the wrong DEPTH, "
             "sweep xrlayercmpm over a decade rather than tuning xrlayersize, which only changes "
             "how big it is.",
             cmpm, m.cm_per_m > 0.0f ? "xrlayercmpm override" : "derived from VR_WorldScale");
    }

    // ---- enable / disable edges ----
    if (!m.enabled) {
        if (prev.enabled) {
            logf("disabled -- tearing down");
            xrlayer_shutdown();
        }
        return;
    }

    if (!prev.enabled) {
        // WHICH BUILD IS THIS? The first thing a field report has to establish and the hardest to
        // get afterwards -- addrcascade's module_identity comment says the same thing about
        // addresses, and it is just as true of the DLL as a whole.
        //
        // Earned the hard way on 2026-08-23: a fix was compiled but never deployed (the game had
        // the DLL locked, so the build ran -NoDeploy), the operator saw "BUILD OK" and injected,
        // and the old binary reproduced the old symptom exactly. Two headset cycles went into
        // re-testing an unchanged build, and nothing in the log could have told anyone. This line
        // makes "is my fix actually in?" answerable in one grep, without comparing file sizes.
        logf("enabled -- plugin built " __DATE__ " " __TIME__);

        // Re-arming after a previous failure is deliberate: the operator flipped the key, so the
        // latch is theirs to clear.
        g_state.store(State::Off, std::memory_order_relaxed);
    }

    // Colour or alpha changed under a live swapchain: ask the submit thread to re-generate the
    // bitmap and re-upload it. The request is a flag, not the work -- the staging buffer and the
    // queue both belong to that thread, and this is the config poll on the game thread.
    //
    // Setting only the dirty flags would NOT be enough: those re-copy the staging buffer, which
    // still holds the old colour. That is a live tunable that appears to do nothing, which is the
    // aimwidgettint failure over again.
    if (prev.cr != m.cr || prev.cg != m.cg || prev.cb != m.cb || prev.alpha != m.alpha) {
        g_regen.store(true, std::memory_order_relaxed);
    }

    // SAME ARGUMENT, FOR THE GUIDE'S ART MODE. grabguidemode selects between the label and the beam
    // and they are two different pictures in one cell, so the cell has to be re-generated when it
    // moves -- otherwise the quad changes shape while still showing the other mode's art. Tracked
    // with its own static rather than added to the Mirror struct: the mirror is the submit thread's
    // snapshot of what it needs per frame, and this is a once-in-a-blue-moon edge.
    {
        static int s_guide_mode_seen = -1;
        const int  now = g_cfg.grab_guide_mode;
        if (s_guide_mode_seen != now) {
            s_guide_mode_seen = now;
            g_regen.store(true, std::memory_order_relaxed);
        }
    }

    if (g_state.load(std::memory_order_relaxed) == State::Off) {
        if (!API::VR::is_openxr()) {
            static bool said = false;
            if (!said) {
                said = true;
                logf("runtime is not OpenXR -- layer unavailable. This is expected under the "
                     "OpenVR/SimVR harness and means this feature CANNOT be verified headlessly.");
            }
            return;
        }
        // THE PDB IS FOR THE FALLBACK RUNG ONLY. When our API layer is in the process the entry
        // points come from it, so loading ~142 MB of symbols would be pure cost -- and on a player
        // install there is no UEVRBackend.pdb to load at all, which is the entire reason the layer
        // exists. Probing the layer first is one GetModuleHandleW.
        if (!xrbridge_available()) {
            // Kick the PDB load onto its own thread and come back next tick. Loading ~142 MB of
            // symbols on the game thread would not be a stutter, it would be a fault -- the same
            // reason MemScan runs on a worker. Costs one bool test per tick until it lands.
            xrattach_begin_async();
            if (!xrattach_ready()) {
                static bool said = false;
                if (!said) { said = true; logf("resolving UEVR's OpenXR entry points (loading symbols off-thread)..."); }
                return;
            }
        }
        const Attach a = attach_submit_path();
        if (a == Attach::Waiting) return;      // not yet is not never -- retried next tick
        if (a == Attach::Failed) {
            g_state.store(State::Failed, std::memory_order_relaxed);
            return;
        }
        g_state.store(State::Pending, std::memory_order_relaxed);
        logf("submit path attached via %s; waiting for the session to bring up the swapchain "
             "(space mode %d)", tier_name(g_tier), m.space);
    }

    // Bring-up on the GAME THREAD, retried each tick until the session exists. Only a hard refusal
    // from the runtime latches Failed -- "no session yet" is patience, not failure, and the two must
    // not share a state or a menu-time start looks like a broken feature.
    if (g_state.load(std::memory_order_relaxed) == State::Pending) {
        auto* p = API::get()->param();
        const bool have_session = p != nullptr && p->openxr != nullptr &&
                                  p->openxr->get_xr_session() != nullptr;
        if (have_session) {
            if (bring_up(m)) g_state.store(State::Armed, std::memory_order_relaxed);
            else             g_state.store(State::Failed, std::memory_order_relaxed);
        }
    }

    // ---- liveness: INSTALLED IS NOT RUNNING ----
    //
    // event_possible is not optional bookkeeping. A watchdog that counts from installation rather
    // than from the first moment the call could occur condemns a healthy address -- addrcascade's
    // README records that failing exactly here, alarming 11 seconds before gameplay even started.
    // The call is only possible once the state is Armed and the runtime is presenting.
    // WHERE `submitted` COMES FROM DEPENDS ON THE RUNG, and on the layer rung its number is the
    // better one. We do the appending on the hook rung, so our own counter is authoritative there.
    // On the layer rung the LAYER does the appending, the call-through AND the fail-open retry --
    // so our producer returning a count only says we OFFERED quads. layers_appended() is the only
    // thing that knows the runtime accepted a frame with them in it, and counting our offers
    // instead would report the overlay live while every frame was being rejected.
    uint32_t submitted;
    if (g_tier == XrAttachTier::ApiLayer) {
        const HaloVrLayerApi* api = xrbridge_api();
        const uint64_t n = (api != nullptr && api->layers_appended != nullptr)
                               ? api->layers_appended() : 0;
        static uint64_t prev = 0;
        submitted = (n >= prev) ? (uint32_t)(n - prev) : 0;
        prev = n;
    } else {
        submitted = g_layers_submitted.exchange(0, std::memory_order_relaxed);
    }
    const State now = g_state.load(std::memory_order_relaxed);

    // PENDING COUNTS, NOT JUST ARMED. This gate was Armed-only, and that made the watchdog
    // unable to fire in the one case it exists for: bring-up happens INSIDE the hook, so a hook
    // that is never called never reaches Armed, so "possible" was never true, so the alarm never
    // sounded -- while the state line reported state=1 forever and looked merely patient.
    //
    // That is the mirror image of the mistake addrcascade's README warns about. It warns against
    // counting time in which the event was impossible (which condemns a healthy address); this was
    // the opposite, refusing to count time in which the event was entirely possible, which
    // exonerates a dead one. Both fail silently. The event is possible from the moment the hook is
    // installed and the runtime is presenting -- which is exactly Pending or Armed.
    const bool possible = (now == State::Pending || now == State::Armed) && API::VR::is_hmd_active();

    // This caller ticks at the game-thread rate (~32 Hz); 96 ticks is ~3 s of frames that should
    // have happened and did not.
    static addrcascade::HookWatchdog watchdog{96};
    const bool alarm = watchdog.tick(possible, submitted > 0);
    if (alarm && g_tier == XrAttachTier::ApiLayer) {
        // THE LAYER RUNG SPLITS THE DIAGNOSIS FOR FREE, and the split is the useful half: it can
        // say whether the layer is being called AT ALL, which the hook rung could only infer.
        const HaloVrLayerApi* api = xrbridge_api();
        const unsigned long long seen =
            (api != nullptr && api->frames_seen != nullptr) ? api->frames_seen() : 0ull;
        if (seen == 0) {
            logf("WATCHDOG: the API layer accepted our callback but its xrEndFrame has NOT run once "
                 "in ~3 s of live frames (frames_seen=0). The layer is loaded but is not on the "
                 "chain this application submits through -- a second OpenXR runtime, or a layer "
                 "ordered after the one UEVR calls. Layer marked not-live; the in-scene reticule is "
                 "unaffected. Layer status: %s", xrbridge_status());
        } else {
            logf("WATCHDOG: the API layer IS running (frames_seen=%llu) but nothing of ours has "
                 "reached the runtime in ~3 s. That is OUR end, not the attachment: either "
                 "produce_layers is gating everything out (read the dark-reason line above) or the "
                 "runtime is rejecting the frame with our quads in it. Layer status: %s",
                 seen, xrbridge_status());
        }
    } else if (alarm) {
        logf("WATCHDOG: the xrEndFrame hook installed but has NOT been called once in ~3 s of live "
             "frames. MEASURED CAUSE (2026-08-23): UEVR STATICALLY LINKS the OpenXR loader into "
             "UEVRBackend.dll -- `dumpbin /imports` shows no openxr_loader.dll import at all -- so "
             "the loader export we hook is not on any path UEVR calls, even though openxr_loader.dll "
             "is loaded in the process by something else. Hooking that export can never work here. "
             "The fix is an OpenXR API LAYER, which the statically-linked loader still loads -- it "
             "is built in apilayer/ and registered with Register-XrApiLayer.ps1. Layer marked "
             "not-live; the in-scene reticule is unaffected.");
    }
    g_live.store(possible && submitted > 0, std::memory_order_relaxed);

    if (m.verbose && (tick % 64) == 0) {
        // `src` says WHICH TEXTURE THE COMPOSITOR IS PRESENTING, not whether a key is on: `owned`
        // means the game thread has captured the widget's art into our own resource at least once,
        // `ring` means the generated bitmap. captures/skips make the game-thread copy path visible
        // -- a capture count that stops advancing while the layer stays live is the signature of a
        // source that has quietly stopped re-validating.
        //
        // ringfalls/held are THE RETICULE-FLASH INSTRUMENT. `ringfalls` counts how many times the
        // compositor has actually replaced the game's art with the generated ring; `held` is the
        // longest capture gap the hold window absorbed WITHOUT doing so. If a player reports the
        // ring flashing, ringfalls is the number that moves, and held creeping up towards
        // xrlayerhold says the window needs to be bigger on their machine.
        // slots= is a BITMASK of which slots have fresh art right now (bit 0 = the reticule), so a
        // "the markers are not showing" report can be answered without another headset session:
        // slots=0x001 means only the reticule resolved, slots=0x0FF means seven markers did and the
        // reticule did not. drops= says the layer budget threw quads away -- and if it is 0 forever
        // while xrlayerbudget is 0, the drop ordering has never run and is therefore unproven.
        uint32_t mask = 0;
        for (int s = 0; s < XRLAYER_SLOTS; ++s) if (xrlayer_slot_ready(s)) mask |= (1u << s);

        logf("state=%d tier=%s live=%d submitted=%u cm/m=%.1f src=%s captures=%u skips=%u "
             "ringfalls=%u held=%ums/%ums atlas=%dx%d slots=0x%03X drops=%u budget=%s "
             "cohskip=%u cohdrawn=%u drops[%s]",
             (int)g_state.load(std::memory_order_relaxed), tier_name(g_tier),
             (int)g_live.load(), submitted,
             g_cm_per_m.load(std::memory_order_relaxed),
             g_showing_ring.load(std::memory_order_relaxed)
                 ? "ring"
                 : ((g_source_override.load(std::memory_order_relaxed) != nullptr) ? "owned" : "ring"),
             g_gt_captures, g_gt_skips,
             g_ring_falls.load(std::memory_order_relaxed),
             g_hold_worst_ms.load(std::memory_order_relaxed),
             g_m_hold_ms.load(std::memory_order_relaxed),
             g_sc_w, g_sc_h, mask,
             g_layer_drops.load(std::memory_order_relaxed),
             (m.budget > 0) ? "FORCED" : "queried",
             g_coh_skips.load(std::memory_order_relaxed),
             g_coh_drawn.load(std::memory_order_relaxed),
             [] {
                 // "0:ok 1:nopose ..." for every slot whose last verdict was not ok. Silent when
                 // everything is being submitted, which is the common case.
                 //
                 // SIZED FOR THE WORST CASE, and it was not. Each entry grew from "N:nopose" (~9
                 // bytes) to "N:cand=34567 drop=3024 last=nopose" (~36) when the counters landed,
                 // and 256 bytes then held only the first eight slots -- the line ended mid-entry
                 // at "8:]" and the GUIDE's counters, the one thing being investigated, were the
                 // bytes that fell off the end. A truncated instrument does not read as broken; it
                 // reads as "slot 10 is absent", which is a FINDING, and I nearly reported it as
                 // one. 11 slots * 40 bytes + slack.
                 static char buf[1400];
                 buf[0] = 0;   // NUL terminator, written as 0: a char escape here has
                 for (int s2 = 0; s2 < XRLAYER_SLOTS; ++s2) {
                     const uint32_t cn = g_slot_cand[s2].load(std::memory_order_relaxed);
                     // THE GUIDE IS NEVER OMITTED. Idle slots are skipped so eight unused navpoint
                     // cells do not cry wolf every line -- but for the guide, cand=0 IS the answer
                     // ("it never reached the admission loop") and suppressing it makes the absent
                     // case indistinguishable from the healthy one.
                     if (cn == 0 && s2 != XRLAYER_SLOT_GUIDE) continue;
                     const uint32_t dn = g_slot_dropn[s2].load(std::memory_order_relaxed);
                     const uint8_t  d  = g_slot_drop[s2].load(std::memory_order_relaxed);
                     char one[128];
                     _snprintf_s(one, sizeof(one), _TRUNCATE,
                                 "%s%d:cand=%u drop=%u(stale=%u nocell=%u notfresh=%u incoh=%u budget=%u) last=%s",
                                 buf[0] ? " " : "", s2, cn, dn,
                                 g_slot_gate[s2][2].load(std::memory_order_relaxed),
                                 g_slot_gate[s2][3].load(std::memory_order_relaxed),
                                 g_slot_gate[s2][4].load(std::memory_order_relaxed),
                                 g_slot_gate[s2][5].load(std::memory_order_relaxed),
                                 g_slot_gate[s2][6].load(std::memory_order_relaxed),
                                 drop_name(d));
                     strncat_s(buf, sizeof(buf), one, _TRUNCATE);
                 }
                 return buf[0] ? buf : "none";
             }());
    }
}

bool xrlayer_live() {
    return g_live.load(std::memory_order_relaxed);
}

bool xrlayer_attached() {
    return g_state.load(std::memory_order_relaxed) == State::Armed;
}

// GAME THREAD. The LAST gate before a pointer the caller inferred starts being copied from every
// frame, so it re-asks the resource itself the questions the caller answered by inference.
//
// XrSource has already validated dimensions and device; this checks the thing only THIS side knows
// -- whether the copy it is about to be used for is a legal one. A CopyResource between mismatched
// sizes or incompatible format families is not a bad picture, it is an invalid call, and refusing
// it here costs the player nothing: the generated ring keeps drawing.
bool xrlayer_set_source(void* d3d12_resource) {
    return xrlayer_set_slot_source(XRLAYER_SLOT_RETICULE, d3d12_resource);
}

// A REFUSAL MUST STAY VISIBLE FOR AS LONG AS IT LASTS.
//
// Every refusal below used to be throttled as "log the first N ever". For a TRANSIENT refusal that
// is right. For a PERMANENT one it is the worst possible behaviour: N lines early in the session,
// then silence forever, so a slot that is being refused on every retry looks -- hours later, to
// whoever is actually debugging it -- exactly like a slot nobody ever offered. That cost the scope
// session an evening (2026-08-26): they traced their whole feed chain and reached "xrlayer refuses
// silently", because from the caller's side it does.
//
// So: log when the REASON CHANGES, and re-log a standing reason every 30 s. A permanent refusal now
// costs two lines a minute and is impossible to miss; a flapping one names each new reason as it
// arrives. `reason` mixes a per-site constant with the salient value (format, dimensions), so
// "refused for a different size now" is a different reason and says so.
bool refusal_should_log(int slot, uint32_t reason) {
    static uint32_t s_reason[XRLAYER_SLOTS] = {};
    static uint64_t s_when[XRLAYER_SLOTS]   = {};
    if (slot < 0 || slot >= XRLAYER_SLOTS) return true;
    const uint64_t now = GetTickCount64();
    if (s_reason[slot] == reason && (now - s_when[slot]) < 30000) return false;
    s_reason[slot] = reason;
    s_when[slot]   = now;
    return true;
}

bool xrlayer_set_slot_source(int slot, void* d3d12_resource) {
    auto* res = (ID3D12Resource*)d3d12_resource;
    if (slot < 0 || slot >= XRLAYER_SLOTS) return false;

    if (res != nullptr) {
        if (g_swapchain == XR_NULL_HANDLE || g_sc_w == 0) {
            logf("source offered before the swapchain exists -- ignored, will be re-offered.");
            return false;
        }
        // THE SIZE QUESTION IS NOW PER CELL, not per swapchain. One image holds every slot's art,
        // so "does this match the swapchain" would reject a perfectly good 128px marker against a
        // 512px atlas. What has to match is the rectangle this slot was given.
        const Cell& cell = g_cell[slot];
        if (cell.dim <= 0) {
            if (refusal_should_log(slot, 0x1000u)) {
                logf("REFUSING source for slot %d: it has no cell in the atlas (pane cell is %dpx, "
                     "requested %dpx). The layout is fixed at bring-up -- with xrlayernav=0 only "
                     "slot 0 exists, the layout falls back to reticule-only when the cells would "
                     "not fit the size cap, and xrlayer_pane_configure() called AFTER bring-up "
                     "records the request for next time rather than resizing a live swapchain.",
                     slot, (int)g_pane_dim, (int)g_pane_req);
            }
            return false;
        }
        const D3D12_RESOURCE_DESC d = res->GetDesc();
        if ((int)d.Width != cell.dim || (int)d.Height != cell.dim) {
            // NOT a one-shot `static bool`. This refusal used to be logged once and then be silent
            // forever while the caller latched "handed over" and stopped offering -- so the feature
            // stayed dead for the session with one stale line to explain it. Say it again, rarely,
            // for as long as it is still true.
            //
            // AND "rarely" HAS TO MEAN PERIODICALLY, NOT "SIX TIMES EVER". That intent was written
            // here and `said < 6` did not deliver it: six lines early in a session, then the same
            // silence the comment was warning against, just further from the start. Fixed properly
            // in refusal_should_log -- reason-change plus a 30 s heartbeat.
            if (refusal_should_log(slot, 0x2000u ^ ((uint32_t)d.Width << 16) ^ (uint32_t)d.Height)) {
                logf("REFUSING source %p for slot %d: it is %llux%u and its atlas cell is %dx%d. "
                     "The copy needs them identical. The atlas is laid out ONCE, at bring-up, from "
                     "aimwidgetdraw (cell 0) and navworlddraw (the marker cells) -- so changing "
                     "either key mid-session needs xrlayer toggled off and on. Still offering; the "
                     "generated ring keeps drawing meanwhile.",
                     (void*)res, slot, (unsigned long long)d.Width, (unsigned)d.Height,
                     cell.dim, cell.dim);
            }
            return false;
        }
        // Copy compatibility is by TYPELESS FAMILY, not by exact equality: BGRA8_UNORM and
        // BGRA8_UNORM_SRGB share a parent and copy fine, while BGRA8 -> RGBA8 does not and would be
        // an invalid call rather than a channel swap.
        const bool src_bgra = (d.Format == DXGI_FORMAT_B8G8R8A8_UNORM ||
                               d.Format == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB ||
                               d.Format == DXGI_FORMAT_B8G8R8A8_TYPELESS);
        const bool src_rgba = (d.Format == DXGI_FORMAT_R8G8B8A8_UNORM ||
                               d.Format == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB ||
                               d.Format == DXGI_FORMAT_R8G8B8A8_TYPELESS);
        if ((g_is_bgra && !src_bgra) || (!g_is_bgra && !src_rgba)) {
            if (refusal_should_log(slot, 0x3000u ^ (uint32_t)d.Format)) {
                logf("REFUSING source %p for slot %d: DXGI format %u is not in the swapchain's "
                     "family (swapchain is %s, 0x%llx). A cross-family CopyResource is invalid, not "
                     "a colour bug -- this needs a shader blit, not a different constant. If this is "
                     "a render target you CREATE, the fix is to create it in the swapchain's family "
                     "(an 8-bit BGRA/RGBA target), not to convert here.",
                     (void*)res, slot, (unsigned)d.Format, g_is_bgra ? "BGRA" : "RGBA",
                     (unsigned long long)g_format);
            }
            return false;
        }
        if (g_owned == nullptr) {
            logf("source offered but the owned capture texture does not exist -- ignored.");
            return false;
        }
        if (g_capture_src[slot] == res) return true;   // already accepted; do not re-log
        logf("source ACCEPTED for slot %d: %p, %llux%u, DXGI %u -> atlas cell (%d,%d) %dpx -> "
             "swapchain %dx%d 0x%llx",
             slot, (void*)res, (unsigned long long)d.Width, (unsigned)d.Height, (unsigned)d.Format,
             cell.x, cell.y, cell.dim, g_sc_w, g_sc_h, (unsigned long long)g_format);

        // ACCEPTED IS NOT PUBLISHED. The pointer is recorded for the GAME THREAD's capture path and
        // goes no further; g_source_override starts holding g_owned only once a capture has
        // actually been submitted, so the submit thread never sees an engine resource even for one
        // frame.
        g_capture_src[slot] = res;
    } else {
        // DROPPING THE SOURCE IS NOT THE SAME AS REVERTING TO THE RING, and conflating the two is
        // what put a placeholder on the player's screen every time a weapon changed.
        //
        // XrSource drops the source for reasons that are almost always TRANSIENT: the widget
        // component re-hosted its crosshair onto a new render target (a weapon pickup does exactly
        // that), the component was rebuilt, the tracked handle failed one revalidation. It then
        // re-resolves within a few ticks and offers a new pointer. The old code unpublished
        // IMMEDIATELY, so every one of those produced a visible pop to the generated ring and back.
        //
        // So: stop CAPTURING at once (g_capture_src = nullptr -- we must not read an engine
        // resource we no longer trust, and that is the safety-relevant half), but keep PRESENTING
        // what we already captured. g_owned is ours and its contents persist. The freshness gate in
        // hooked_end_frame then decides how long that stands: within xrlayerhold nothing visibly
        // happens, and only a drop that fails to re-resolve for longer than that -- a real level
        // change -- reaches the ring.
        //
        // The slot's beat is deliberately LEFT ALONE rather than zeroed: zeroing it forces the
        // gate's `beat == 0` arm, which is the instant fall-back this change exists to remove. It
        // ages out by itself, which is exactly the behaviour wanted.
        g_capture_src[slot] = nullptr;
    }
    return true;
}

// ---- the batched capture --------------------------------------------------------------------
//
// GAME THREAD, all three. Everything here runs between two UE ticks, which is the only property
// that makes reading the engine's render targets defensible at all -- see the g_owned commentary.
bool xrlayer_capture_begin() {
    if (g_gt_open_slot >= 0) {
        // The caller opened a list and never submitted it. Refusing is the honest answer: silently
        // re-opening would leak the previous recording and, worse, would make the failure look like
        // "capture just stopped working" a hundred ticks later.
        static uint32_t said = 0;
        if (said < 3) { ++said; logf("capture_begin called with a list still open -- caller must submit."); }
        return false;
    }
    if (g_state.load(std::memory_order_relaxed) != State::Armed) return false;
    if (g_owned == nullptr || g_gt_list == nullptr || g_gt_fence == nullptr || g_queue == nullptr) {
        return false;
    }

    // NEVER BLOCK THE GAME THREAD. If the previous capture on this allocator is still executing,
    // skip: the atlas keeps the last capture's pixels and the overlay is one widget frame stale,
    // which is invisible. A fence wait here would be a game-thread stall every frame, which in VR
    // is nausea -- and a blocking wait on a shared queue is precisely the shape of the bug that
    // produced the 2026-08-23 crashes.
    const int slot = g_gt_ring;
    if (g_gt_alloc_fence[slot] != 0 && g_gt_fence->GetCompletedValue() < g_gt_alloc_fence[slot]) {
        ++g_gt_skips;
        // The game thread IS alive, so nothing should age out of the hold window on account of one
        // busy frame. Refresh every slot that already has art; a slot that has never captured stays
        // at 0 so it cannot pretend to be fresh.
        const uint64_t now = GetTickCount64();
        for (int i = 0; i < XRLAYER_SLOTS; ++i) {
            if (g_slot_beat[i].load(std::memory_order_relaxed) != 0) {
                g_slot_beat[i].store(now, std::memory_order_relaxed);
            }
        }
        g_src_beat.store(now, std::memory_order_relaxed);
        return false;
    }
    g_gt_ring = (g_gt_ring + 1) % GT_RING;

    ID3D12CommandAllocator* alloc = g_gt_alloc[slot];
    if (alloc == nullptr || FAILED(alloc->Reset())) return false;
    if (FAILED(g_gt_list->Reset(alloc, nullptr))) return false;

    // The atlas is transitioned ONCE for the whole batch rather than per slot: it is one resource
    // and nine open/close pairs on it would be nine no-op decompressions to no purpose.
    D3D12_RESOURCE_BARRIER to_dst{};
    to_dst.Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    to_dst.Transition.pResource   = g_owned;
    to_dst.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    to_dst.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_SOURCE;
    to_dst.Transition.StateAfter  = D3D12_RESOURCE_STATE_COPY_DEST;
    g_gt_list->ResourceBarrier(1, &to_dst);

    g_gt_open_slot = slot;
    g_gt_recorded  = 0;
    g_gt_rec_mask  = 0;
    return true;
}

bool xrlayer_capture_record(int slot) {
    if (g_gt_open_slot < 0) return false;
    if (slot < 0 || slot >= XRLAYER_SLOTS) return false;
    ID3D12Resource* src = g_capture_src[slot];
    const Cell& c = g_cell[slot];
    if (src == nullptr || c.dim <= 0) return false;

    // Borrow the engine's target into COPY_SOURCE and hand it straight back inside one command
    // list, so it is never left in a state the engine does not expect on the next frame. This is
    // the same ENGINE_SRC_COLOR assumption blit_into used to make -- it has moved thread, it has
    // not become a measurement. If the art ever presents as garbage rather than as nothing, this
    // pair is still the first thing to doubt.
    //
    // THIS IS THE DANGEROUS INSTRUCTION. All three 2026-08-23 access violations faulted inside the
    // driver reached from a ResourceBarrier on an engine-owned resource whose GPU backing had gone.
    // It is recorded here, IMMEDIATELY after the caller re-validated this slot's chain end to end,
    // and that adjacency is the safety argument -- do not move validation away from it.
    D3D12_RESOURCE_BARRIER to_src{};
    to_src.Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    to_src.Transition.pResource   = src;
    to_src.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    to_src.Transition.StateBefore = ENGINE_SRC_COLOR;
    to_src.Transition.StateAfter  = D3D12_RESOURCE_STATE_COPY_SOURCE;
    g_gt_list->ResourceBarrier(1, &to_src);

    // Into THIS SLOT'S RECTANGLE of the atlas. CopyResource cannot express a destination offset,
    // which is the one mechanical change the atlas forces on every copy in this file.
    D3D12_TEXTURE_COPY_LOCATION dl{};
    dl.pResource        = g_owned;
    dl.Type             = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    dl.SubresourceIndex = 0;
    D3D12_TEXTURE_COPY_LOCATION sl{};
    sl.pResource        = src;
    sl.Type             = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    sl.SubresourceIndex = 0;
    g_gt_list->CopyTextureRegion(&dl, (UINT)c.x, (UINT)c.y, 0, &sl, nullptr);

    D3D12_RESOURCE_BARRIER src_back = to_src;
    src_back.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_SOURCE;
    src_back.Transition.StateAfter  = ENGINE_SRC_COLOR;
    g_gt_list->ResourceBarrier(1, &src_back);

    // THE SCOPE PANE'S FEATHERED OVAL, immediately after its scene copy and while the atlas is
    // still COPY_DEST (capture_begin put it there; capture_submit takes it out). The call makes and
    // unmakes its own UNORDERED_ACCESS transition, so this sequence's state tracking is unchanged.
    //
    // Pane only: slots 0-8 are widgets whose alpha is authored on the CPU and already correct.
    if (slot == XRLAYER_SLOT_PANE) {
        scopemask_apply(g_device, g_gt_list, g_owned, c.x, c.y, c.dim);
    }

    ++g_gt_recorded;
    g_gt_rec_mask |= (1u << slot);
    return true;
}

void xrlayer_capture_submit() {
    const int slot = g_gt_open_slot;
    if (slot < 0) return;
    g_gt_open_slot = -1;

    if (g_gt_list == nullptr) return;

    D3D12_RESOURCE_BARRIER dst_back{};
    dst_back.Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    dst_back.Transition.pResource   = g_owned;
    dst_back.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    dst_back.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
    dst_back.Transition.StateAfter  = D3D12_RESOURCE_STATE_COPY_SOURCE;
    g_gt_list->ResourceBarrier(1, &dst_back);

    // CLOSE EVEN WHEN THERE IS NOTHING TO SUBMIT. A list left open makes the next begin()'s
    // Reset() fail, and the visible symptom of that is "capture silently stopped for the rest of
    // the session" -- which is exactly the class of failure this feature keeps producing.
    if (FAILED(g_gt_list->Close())) return;
    if (g_gt_recorded == 0) return;      // recorded barriers are discarded unexecuted; nothing ran

    // SAME QUEUE AS THE PRESENT COPY, deliberately. ID3D12CommandQueue is free-threaded and the
    // two threads use different lists and allocators, so this is legal -- and submitting to one
    // queue is what makes "capture, then present" ordered on the GPU without a cross-queue fence.
    ID3D12CommandList* lists[] = {g_gt_list};
    g_queue->ExecuteCommandLists(1, lists);

    ++g_gt_fence_v;
    if (FAILED(g_queue->Signal(g_gt_fence, g_gt_fence_v))) return;
    g_gt_alloc_fence[slot] = g_gt_fence_v;

    ++g_gt_captures;
    const uint64_t now = GetTickCount64();
    const uint32_t gtick = g_game_tick.load(std::memory_order_relaxed);
    g_src_beat.store(now, std::memory_order_relaxed);
    for (int i = 0; i < XRLAYER_SLOTS; ++i) {
        if ((g_gt_rec_mask & (1u << i)) != 0u) {
            g_slot_beat[i].store(now, std::memory_order_relaxed);
            // The game tick THIS cell's art was captured on -- the coherence half of the freshness
            // gate. Only a real capture advances it; a skipped/held capture (below) refreshes the
            // ms beat but must NOT advance this, or a slot re-hosted then skipped would read coherent
            // without its new art ever being copied.
            g_slot_cap_tick[i].store(gtick, std::memory_order_relaxed);
        }
    }
    g_gt_rec_mask = 0;

    // Publish on the FIRST successful capture only. Before that the submit thread presents the
    // generated atlas, which is the correct picture for "we have not captured anything yet".
    if (g_source_override.load(std::memory_order_relaxed) != g_owned) {
        g_source_override.store(g_owned, std::memory_order_release);
        for (size_t i = 0; i < g_image_dirty.size(); ++i) g_image_dirty[i] = true;
        logf("presenting the OWNED atlas %p (%dx%d). The submit thread no longer touches an engine "
             "resource on any path.", (void*)g_owned, g_sc_w, g_sc_h);
    }
}

bool xrlayer_capture_source() {
    // Slot 0 on its own, for the reticule's existing call path.
    if (!xrlayer_capture_begin()) return false;
    const bool ok = xrlayer_capture_record(XRLAYER_SLOT_RETICULE);
    xrlayer_capture_submit();
    return ok;
}

void xrlayer_shutdown() {
    // Order matters: stop the hook appending before the swapchain it points at goes away.
    remove_hook();
    destroy_swapchain();
    release_d3d();
    g_session = XR_NULL_HANDLE;
    g_device  = nullptr;
    g_queue   = nullptr;
    g_source_override.store(nullptr, std::memory_order_release);
    g_live.store(false, std::memory_order_relaxed);
    g_state.store(State::Off, std::memory_order_relaxed);

    // The atlas layout goes with the swapchain it sized. Leaving stale cells behind would let a
    // slot accept a source against a rectangle of an image that no longer exists.
    for (auto& c : g_cell) c = Cell{};
    g_sc_w = g_sc_h = 0;
    g_ret_dim = g_nav_dim = 0;
    g_max_layers = 0;
    for (int i = 0; i < XRLAYER_SLOTS; ++i) g_tgt_live[i].store(false, std::memory_order_relaxed);

    // NO PER-SLOT RESOURCES TO RELEASE, and that is by design rather than by luck: every slot lives
    // in a cell of the ONE g_owned atlas, which release_d3d() above already freed. Adding a tenth
    // slot adds no D3D12 lifetime here. The hang this teardown exists to prevent came from exactly
    // one leaked swapchain, so "adding a slot cannot leak" was worth buying.
}

}   // namespace halo
