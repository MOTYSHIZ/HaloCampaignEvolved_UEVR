// LayerSelfTest -- drive XrApiLayer_HALOVR_reticule.dll through a FAKE OPENXR LOADER.
//
// ============================================================================================
// WHAT THIS PROVES, AND WHY IT IS WORTH A SEPARATE PROGRAM
// ============================================================================================
// The layer's whole job happens between two parties we do not control: the OpenXR loader above it
// and the runtime below it. The obvious way to test it is to run the game -- which needs the game
// installed, a headset or a simulator, an injection, and a human. That is a slow, scarce and
// serialised resource on this project, and it is the WRONG instrument anyway: when a quad does not
// appear in a headset, "the negotiation struct is wrong" and "the pose maths is wrong" look
// identical.
//
// So this harness IS the loader. It builds XrNegotiateLoaderInfo by hand, calls the layer's
// negotiation export, walks the layer through xrCreateApiLayerInstance with a stub chain underneath
// it, and then inspects EXACTLY WHAT THE RUNTIME WOULD HAVE RECEIVED. Every claim below is checked
// against the stub's recorded arguments rather than against the layer's own reporting -- a layer
// that says it appended a quad and a runtime that received one are different facts, and this
// project has already lost days to code that reported success and ran zero times.
//
// It needs no game, no headset, no OpenXR runtime, and no UEVR. It runs in about a second.
//
// WHAT IT DOES NOT PROVE, stated here so nobody reads a green run as more than it is:
//   * That the REAL loader accepts this layer. The negotiation ABI is implemented here from the
//     same header the loader is built from, but this harness cannot catch a disagreement between
//     that header and the loader binary actually on a player's machine.
//   * That UEVR's statically-linked loader walks the implicit-layer registry. That is evidence
//     (Virtual Desktop's implicit layer is observed loaded in the Halo process) rather than proof,
//     and only a live run settles it.
//   * Anything about D3D12, swapchains, poses or pixels. None of that is in the layer.
//
// Modes, because the process-name gate is decided ONCE per process and cannot be tested both ways
// in one run:
//   active  -- HALOVR_LAYER_FORCE is set by the caller; the layer must intercept and function.
//   gated   -- no force variable and the host is not the game; the layer must be INVISIBLE.

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include "../../src/XrLayerAbi.h"
#include "thirdparty/openxr/loader_interfaces.h"

#include <cstdio>
#include <cstring>

// ============================================================================================
// Test scaffolding
// ============================================================================================

static int g_failures = 0;
static int g_checks   = 0;

static void check(bool cond, const char* what) {
    ++g_checks;
    if (cond) {
        printf("  ok    %s\n", what);
    } else {
        printf("  FAIL  %s\n", what);
        ++g_failures;
    }
}

// ============================================================================================
// The stub chain -- everything BELOW the layer
// ============================================================================================
//
// This stands in for the runtime. It records what it was handed so the test can assert on the
// runtime's view rather than on the layer's claims.

namespace stub {

const XrInstance kInstance = (XrInstance)(uintptr_t)0xA11CE000;
const XrSession  kSession  = (XrSession)(uintptr_t)0x5E5510;

int      end_frame_calls = 0;
uint32_t last_layer_count = 0;
const XrCompositionLayerBaseHeader* last_layers[64];
XrResult next_end_frame_result = XR_SUCCESS;   // flip to force the fail-open retry
int      create_session_calls = 0;
int      destroy_session_calls = 0;

void reset() {
    end_frame_calls = 0;
    last_layer_count = 0;
    memset(last_layers, 0, sizeof(last_layers));
    next_end_frame_result = XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL end_frame(XrSession, const XrFrameEndInfo* info) {
    ++end_frame_calls;
    last_layer_count = info->layerCount;
    for (uint32_t i = 0; i < info->layerCount && i < 64; ++i) last_layers[i] = info->layers[i];
    // The first call may be made to fail on purpose; the fail-open RETRY must then succeed, which is
    // what makes "a missing overlay, not a dropped frame" a tested property rather than a comment.
    const XrResult r = next_end_frame_result;
    next_end_frame_result = XR_SUCCESS;
    return r;
}

XRAPI_ATTR XrResult XRAPI_CALL create_session(XrInstance, const XrSessionCreateInfo*, XrSession* s) {
    ++create_session_calls;
    *s = kSession;
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL destroy_session(XrSession) {
    ++destroy_session_calls;
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL destroy_instance(XrInstance) { return XR_SUCCESS; }

// A function the layer does NOT wrap, used to prove pass-through reaches the bottom of the chain.
XRAPI_ATTR XrResult XRAPI_CALL request_exit_session(XrSession) { return XR_SUCCESS; }

XRAPI_ATTR XrResult XRAPI_CALL gipa(XrInstance, const char* name, PFN_xrVoidFunction* fn) {
    if (name == nullptr || fn == nullptr) return XR_ERROR_VALIDATION_FAILURE;
    if (strcmp(name, "xrEndFrame") == 0)          { *fn = (PFN_xrVoidFunction)end_frame;           return XR_SUCCESS; }
    if (strcmp(name, "xrCreateSession") == 0)     { *fn = (PFN_xrVoidFunction)create_session;      return XR_SUCCESS; }
    if (strcmp(name, "xrDestroySession") == 0)    { *fn = (PFN_xrVoidFunction)destroy_session;     return XR_SUCCESS; }
    if (strcmp(name, "xrDestroyInstance") == 0)   { *fn = (PFN_xrVoidFunction)destroy_instance;    return XR_SUCCESS; }
    if (strcmp(name, "xrRequestExitSession") == 0){ *fn = (PFN_xrVoidFunction)request_exit_session; return XR_SUCCESS; }
    *fn = nullptr;
    return XR_ERROR_FUNCTION_UNSUPPORTED;
}

XRAPI_ATTR XrResult XRAPI_CALL create_api_layer_instance(const XrInstanceCreateInfo*,
                                                         const XrApiLayerCreateInfo* layer_info,
                                                         XrInstance* instance) {
    // The bottom of the chain: the loader's terminator. It must have been handed a create-info
    // whose nextInfo has ALREADY been advanced past the layer -- if the layer forgot to advance it,
    // a real loader would loop forever here.
    if (layer_info != nullptr && layer_info->nextInfo != nullptr) {
        printf("  FAIL  the layer did not advance nextInfo before forwarding\n");
        ++g_failures;
    }
    *instance = kInstance;
    return XR_SUCCESS;
}

}   // namespace stub

// ============================================================================================
// The plugin side -- a stand-in for what XrLayer.cpp will do
// ============================================================================================

namespace plugin {

// Static storage, because the ABI says the pointers must outlive the call. A stack local here would
// be the exact bug the contract warns about, and it would pass on a good day.
XrCompositionLayerQuad g_quads[3];
uint32_t g_quads_to_emit = 0;
uint32_t g_overclaim     = 0;    // return this many MORE than were actually written
int      g_callback_hits = 0;
void*    g_seen_user     = nullptr;

XRAPI_ATTR uint32_t XRAPI_CALL on_end_frame(XrSession, const XrFrameEndInfo*,
                                            const XrCompositionLayerBaseHeader** out,
                                            uint32_t capacity, void* user) {
    ++g_callback_hits;
    g_seen_user = user;
    uint32_t n = 0;
    for (uint32_t i = 0; i < g_quads_to_emit && i < capacity && i < 3; ++i) {
        g_quads[i] = XrCompositionLayerQuad{XR_TYPE_COMPOSITION_LAYER_QUAD};
        g_quads[i].size = {0.1f, 0.1f};
        out[n++] = (const XrCompositionLayerBaseHeader*)&g_quads[i];
    }
    return n + g_overclaim;
}

}   // namespace plugin

// ============================================================================================

static XrFrameEndInfo make_frame(const XrCompositionLayerBaseHeader** layers, uint32_t count) {
    XrFrameEndInfo info{XR_TYPE_FRAME_END_INFO};
    info.displayTime               = 1;
    info.environmentBlendMode      = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    info.layerCount                = count;
    info.layers                    = layers;
    return info;
}

int main(int argc, char** argv) {
    const bool gated_mode = (argc > 1 && strcmp(argv[1], "gated") == 0);

    printf("XrApiLayer_HALOVR_reticule self-test  [mode: %s]\n", gated_mode ? "gated" : "active");
    printf("----------------------------------------------------------------\n");

    // ---- load the layer exactly as the loader would ------------------------------------------
    HMODULE lib = LoadLibraryW(HALOVR_LAYER_MODULE_W);
    if (lib == nullptr) {
        printf("  FAIL  could not load %ls (error %lu)\n", HALOVR_LAYER_MODULE_W, GetLastError());
        return 1;
    }
    auto negotiate = (PFN_xrNegotiateLoaderApiLayerInterface)
        GetProcAddress(lib, "xrNegotiateLoaderApiLayerInterface");
    auto get_api = (PFN_halovr_layer_get_api)GetProcAddress(lib, HALOVR_LAYER_GET_API_SYM);
    check(negotiate != nullptr, "exports xrNegotiateLoaderApiLayerInterface");
    check(get_api   != nullptr, "exports " HALOVR_LAYER_GET_API_SYM);
    if (negotiate == nullptr || get_api == nullptr) return 1;

    // ---- negotiation -------------------------------------------------------------------------
    XrNegotiateLoaderInfo loader_info{};
    loader_info.structType         = XR_LOADER_INTERFACE_STRUCT_LOADER_INFO;
    loader_info.structVersion      = XR_LOADER_INFO_STRUCT_VERSION;
    loader_info.structSize         = sizeof(XrNegotiateLoaderInfo);
    loader_info.minInterfaceVersion = 1;
    loader_info.maxInterfaceVersion = XR_CURRENT_LOADER_API_LAYER_VERSION;
    loader_info.minApiVersion       = XR_MAKE_VERSION(1, 0, 0);
    loader_info.maxApiVersion       = XR_MAKE_VERSION(1, 0, 0xFFF);

    XrNegotiateApiLayerRequest req{};
    req.structType    = XR_LOADER_INTERFACE_STRUCT_API_LAYER_REQUEST;
    req.structVersion = XR_API_LAYER_INFO_STRUCT_VERSION;
    req.structSize    = sizeof(XrNegotiateApiLayerRequest);

    check(negotiate(&loader_info, HALOVR_LAYER_NAME, &req) == XR_SUCCESS, "negotiation succeeds");
    check(req.getInstanceProcAddr    != nullptr, "negotiation returns a getInstanceProcAddr");
    check(req.createApiLayerInstance != nullptr, "negotiation returns a createApiLayerInstance");
    check(req.layerInterfaceVersion  == XR_CURRENT_LOADER_API_LAYER_VERSION,
          "negotiation reports the loader interface version it was asked for");

    // Negotiation must REFUSE a malformed or out-of-range request rather than filling it in. A layer
    // that answers a struct it does not understand is how a loader ends up calling a garbage
    // pointer -- the exact failure mode CLAUDE.md records for LuaVR.
    {
        XrNegotiateLoaderInfo bad = loader_info;
        bad.structSize = sizeof(XrNegotiateLoaderInfo) + 8;
        XrNegotiateApiLayerRequest r2 = req;
        check(negotiate(&bad, HALOVR_LAYER_NAME, &r2) != XR_SUCCESS,
              "negotiation refuses a loader info of the wrong size");

        XrNegotiateLoaderInfo future = loader_info;
        future.minInterfaceVersion = XR_CURRENT_LOADER_API_LAYER_VERSION + 1;
        future.maxInterfaceVersion = XR_CURRENT_LOADER_API_LAYER_VERSION + 2;
        check(negotiate(&future, HALOVR_LAYER_NAME, &r2) != XR_SUCCESS,
              "negotiation refuses an interface version it does not implement");

        check(negotiate(&loader_info, "XR_APILAYER_SOMEONE_else", &r2) != XR_SUCCESS,
              "negotiation refuses a request for a different layer name");
    }

    // ---- instance creation, with our stub as the chain below -----------------------------------
    XrApiLayerNextInfo next_info{};
    next_info.structType    = XR_LOADER_INTERFACE_STRUCT_API_LAYER_NEXT_INFO;
    next_info.structVersion = XR_API_LAYER_NEXT_INFO_STRUCT_VERSION;
    next_info.structSize    = sizeof(XrApiLayerNextInfo);
    strcpy_s(next_info.layerName, HALOVR_LAYER_NAME);
    next_info.nextGetInstanceProcAddr    = stub::gipa;
    next_info.nextCreateApiLayerInstance = stub::create_api_layer_instance;
    next_info.next = nullptr;

    XrApiLayerCreateInfo create{};
    create.structType    = XR_LOADER_INTERFACE_STRUCT_API_LAYER_CREATE_INFO;
    create.structVersion = XR_API_LAYER_CREATE_INFO_STRUCT_VERSION;
    create.structSize    = sizeof(XrApiLayerCreateInfo);
    create.loaderInstance = nullptr;
    create.nextInfo       = &next_info;

    XrInstanceCreateInfo ici{XR_TYPE_INSTANCE_CREATE_INFO};
    strcpy_s(ici.applicationInfo.applicationName, "LayerSelfTest");
    ici.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;

    XrInstance instance = XR_NULL_HANDLE;
    check(req.createApiLayerInstance(&ici, &create, &instance) == XR_SUCCESS,
          "createApiLayerInstance succeeds");
    check(instance == stub::kInstance, "the instance came from the chain below, unchanged");

    // A malformed create-info must be refused, not dereferenced.
    {
        XrApiLayerCreateInfo bad = create;
        bad.nextInfo = nullptr;
        XrInstance dummy = XR_NULL_HANDLE;
        check(req.createApiLayerInstance(&ici, &bad, &dummy) != XR_SUCCESS,
              "createApiLayerInstance refuses a create-info with no nextInfo");
    }

    // ---- dispatch ------------------------------------------------------------------------------
    PFN_xrVoidFunction fn = nullptr;
    check(req.getInstanceProcAddr(instance, "xrEndFrame", &fn) == XR_SUCCESS && fn != nullptr,
          "xrEndFrame resolves through the layer");
    auto layer_end_frame = (PFN_xrEndFrame)fn;

    PFN_xrVoidFunction passthrough = nullptr;
    req.getInstanceProcAddr(instance, "xrRequestExitSession", &passthrough);
    check(passthrough == (PFN_xrVoidFunction)stub::request_exit_session,
          "an unwrapped function passes straight through to the chain below");

    if (gated_mode) {
        // THE WHOLE POINT OF THIS MODE. In a process that is not the game the layer must be
        // indistinguishable from not being installed: no wrapping, no API, nothing.
        check(layer_end_frame == (PFN_xrEndFrame)stub::end_frame,
              "GATED: xrEndFrame is NOT wrapped - the chain below is returned verbatim");
        check(get_api(HALOVR_LAYER_ABI_VERSION) == nullptr,
              "GATED: halovr_layer_get_api refuses, so the plugin cannot arm a layer that is inert");

        printf("----------------------------------------------------------------\n");
        printf("%d checks, %d failures\n", g_checks, g_failures);
        return g_failures == 0 ? 0 : 1;
    }

    check(layer_end_frame != (PFN_xrEndFrame)stub::end_frame,
          "ACTIVE: xrEndFrame IS wrapped by the layer");

    // ---- the bridge API ------------------------------------------------------------------------
    check(get_api(HALOVR_LAYER_ABI_VERSION + 1000) == nullptr,
          "get_api REFUSES an ABI version it does not implement (no silent adaptation)");

    const HaloVrLayerApi* api = get_api(HALOVR_LAYER_ABI_VERSION);
    check(api != nullptr, "get_api returns the API for the matching ABI version");
    if (api == nullptr) return 1;

    check(api->struct_size == sizeof(HaloVrLayerApi), "struct_size matches the header both sides use");
    check(api->abi_version == HALOVR_LAYER_ABI_VERSION, "abi_version matches");
    check(api->get_instance() == stub::kInstance, "get_instance reports the live instance");
    check(api->get_proc("xrEndFrame") == (PFN_xrVoidFunction)stub::end_frame,
          "get_proc resolves BELOW the layer - the plugin never re-enters our own xrEndFrame");
    check(api->get_proc("xrNoSuchFunction") == nullptr, "get_proc returns null for an unknown name");

    // ---- session tracking ----------------------------------------------------------------------
    PFN_xrVoidFunction cs = nullptr;
    req.getInstanceProcAddr(instance, "xrCreateSession", &cs);
    XrSession session = XR_NULL_HANDLE;
    XrSessionCreateInfo sci{XR_TYPE_SESSION_CREATE_INFO};
    ((PFN_xrCreateSession)cs)(instance, &sci, &session);
    check(api->get_session() == stub::kSession, "the layer captures the session at xrCreateSession");

    // ---- the frame path ------------------------------------------------------------------------
    XrCompositionLayerProjection app_layer{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
    const XrCompositionLayerBaseHeader* app_layers[1] = {
        (const XrCompositionLayerBaseHeader*)&app_layer
    };

    // (a) no callback registered -> the application's frame goes through untouched.
    stub::reset();
    {
        XrFrameEndInfo f = make_frame(app_layers, 1);
        check(layer_end_frame(session, &f) == XR_SUCCESS, "unarmed: xrEndFrame succeeds");
        check(stub::last_layer_count == 1, "unarmed: the runtime sees the application's frame unchanged");
    }
    const uint64_t frames_after_a = api->frames_seen();
    check(frames_after_a >= 1, "frames_seen counts calls that reached the layer");
    check(api->layers_appended() == 0, "layers_appended stays 0 while nothing is registered");

    // (b) armed with two quads -> the runtime must receive three layers, OURS LAST.
    int user_marker = 42;
    check(api->set_end_frame_callback(plugin::on_end_frame, &user_marker) == 1,
          "the callback registers");
    plugin::g_quads_to_emit = 2;
    plugin::g_overclaim     = 0;
    stub::reset();
    {
        XrFrameEndInfo f = make_frame(app_layers, 1);
        check(layer_end_frame(session, &f) == XR_SUCCESS, "armed: xrEndFrame succeeds");
        check(plugin::g_callback_hits == 1, "armed: the plugin callback ran");
        check(plugin::g_seen_user == &user_marker, "armed: the user pointer arrived intact");
        check(stub::last_layer_count == 3, "armed: THE RUNTIME RECEIVED 3 LAYERS (1 app + 2 ours)");
        check(stub::last_layers[0] == app_layers[0],
              "armed: the application's own layer is still first");
        check(stub::last_layers[1] == (const XrCompositionLayerBaseHeader*)&plugin::g_quads[0] &&
              stub::last_layers[2] == (const XrCompositionLayerBaseHeader*)&plugin::g_quads[1],
              "armed: ours are appended LAST, therefore topmost");
        check(api->layers_appended() == 1, "armed: layers_appended moved");
    }

    // (c) FAIL OPEN. The runtime rejects the frame that contains our layers; the layer must retry
    //     with the application's original list rather than dropping the player's frame.
    stub::reset();
    stub::next_end_frame_result = XR_ERROR_LAYER_INVALID;
    {
        XrFrameEndInfo f = make_frame(app_layers, 1);
        const XrResult r = layer_end_frame(session, &f);
        check(r == XR_SUCCESS, "fail-open: the frame still succeeds after the runtime rejected ours");
        check(stub::end_frame_calls == 2, "fail-open: the layer retried");
        check(stub::last_layer_count == 1, "fail-open: the retry carried the application's frame ONLY");
    }

    // (d) a callback that overclaims must have its whole batch discarded, not clamped.
    plugin::g_quads_to_emit = 2;
    plugin::g_overclaim     = HALOVR_LAYER_MAX_EXTRA_LAYERS;    // returns far more than it wrote
    stub::reset();
    {
        XrFrameEndInfo f = make_frame(app_layers, 1);
        check(layer_end_frame(session, &f) == XR_SUCCESS, "overclaim: the frame still succeeds");
        check(stub::last_layer_count == 1,
              "overclaim: the batch is DISCARDED - the runtime sees the untouched frame");
    }
    plugin::g_overclaim = 0;

    // (e) clearing the callback must actually stop the calls.
    const int hits_before = plugin::g_callback_hits;
    check(api->set_end_frame_callback(nullptr, nullptr) == 1, "the callback clears");
    stub::reset();
    {
        XrFrameEndInfo f = make_frame(app_layers, 1);
        layer_end_frame(session, &f);
        check(plugin::g_callback_hits == hits_before, "cleared: the plugin callback no longer runs");
        check(stub::last_layer_count == 1, "cleared: the runtime sees the untouched frame again");
    }

    // ---- teardown -------------------------------------------------------------------------------
    PFN_xrVoidFunction ds = nullptr;
    req.getInstanceProcAddr(instance, "xrDestroySession", &ds);
    ((PFN_xrDestroySession)ds)(session);
    check(api->get_session() == XR_NULL_HANDLE, "the session is forgotten at xrDestroySession");

    PFN_xrVoidFunction di = nullptr;
    req.getInstanceProcAddr(instance, "xrDestroyInstance", &di);
    ((PFN_xrDestroyInstance)di)(instance);
    check(api->get_instance() == XR_NULL_HANDLE, "the instance is forgotten at xrDestroyInstance");
    check(api->get_proc("xrEndFrame") == nullptr,
          "after teardown get_proc refuses - no handing out pointers into a destroyed instance");

    printf("\nstatus: %s\n", api->status());
    printf("----------------------------------------------------------------\n");
    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
