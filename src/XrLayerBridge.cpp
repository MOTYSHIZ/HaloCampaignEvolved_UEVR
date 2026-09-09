// XrLayerBridge -- see XrLayerBridge.hpp for what this is and why it is so small.

#include "XrLayerBridge.hpp"

#include "uevr/API.hpp"

#include <windows.h>

#include <atomic>
#include <cstdarg>
#include <cstdio>

using namespace uevr;

namespace halo {
namespace {

constexpr const char* TAG = "[XRBRIDGE]";

void logf(const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    _vsnprintf_s(buf, sizeof(buf), _TRUNCATE, fmt, ap);
    va_end(ap);
    API::get()->log_info("%s %s", TAG, buf);
}

// -1 not probed, 0 absent/refused, 1 available.
std::atomic<int> g_state{-1};
const HaloVrLayerApi* g_api = nullptr;
char g_status[640] = "not probed";

// GetModuleHandleW, NOT LoadLibraryW, and that distinction is the whole safety argument.
//
// If the OpenXR loader put the layer in this process, the module is already mapped and this finds
// it. If it did not, the layer is NOT SUPPOSED TO BE HERE and loading it ourselves would create
// exactly the thing the layer is designed to avoid: a DLL wrapping nothing, outside the loader's
// chain, with no instance to attach to. A LoadLibrary here would also succeed on any file that
// happened to sit on the search path under that name.
//
// So: present means the loader loaded it. Absent means the player has not registered it, and the
// correct response is to carry on without it.
void probe() {
    HMODULE m = GetModuleHandleW(HALOVR_LAYER_MODULE_W);
    if (m == nullptr) {
        _snprintf_s(g_status, sizeof(g_status), _TRUNCATE,
                    "layer NOT loaded in this process (%ls). The OpenXR loader only loads it when it "
                    "is registered, or when XR_API_LAYER_PATH/XR_ENABLE_API_LAYERS name it. The "
                    "compositor reticule is unavailable; the in-scene one is unaffected.",
                    HALOVR_LAYER_MODULE_W);
        logf("%s", g_status);
        g_state.store(0, std::memory_order_release);
        return;
    }

    auto get_api = (PFN_halovr_layer_get_api)GetProcAddress(m, HALOVR_LAYER_GET_API_SYM);
    if (get_api == nullptr) {
        // A module by that name without our export is not our layer, or is one so old it predates
        // the bridge. Either way the only safe move is to leave it alone.
        _snprintf_s(g_status, sizeof(g_status), _TRUNCATE,
                    "a module named %ls is loaded but does not export %s -- not our layer, or a "
                    "build that predates this bridge. Ignoring it.",
                    HALOVR_LAYER_MODULE_W, HALOVR_LAYER_GET_API_SYM);
        logf("%s", g_status);
        g_state.store(0, std::memory_order_release);
        return;
    }

    const HaloVrLayerApi* api = get_api(HALOVR_LAYER_ABI_VERSION);
    if (api == nullptr) {
        // THE VERSION-SKEW CASE, AND IT IS EXPECTED TO HAPPEN. The layer is registered ONCE, by
        // path, while halo_vr.dll is replaced by every update -- so a player whose registry still
        // points at an older unpacked copy lands exactly here. It is a clean null and a log line
        // rather than a call into a struct whose fields have moved, which is the failure CLAUDE.md
        // records for LuaVR: "a garbage-pointer call with no dump".
        //
        // It also covers the gated case: the layer refuses when the host process is not the game.
        _snprintf_s(g_status, sizeof(g_status), _TRUNCATE,
                    "the layer is loaded but REFUSED ABI version %u. Either it is an older build "
                    "than this plugin (re-run Register-XrApiLayer.ps1 against the current install), "
                    "or it gated itself off for this process. Carrying on without it.",
                    (unsigned)HALOVR_LAYER_ABI_VERSION);
        logf("%s", g_status);
        g_state.store(0, std::memory_order_release);
        return;
    }

    // The layer agreed the version. Now check the SIZE, which is the other half of the contract:
    // a version match with a smaller struct means the layer was built against a header that had
    // fewer fields, and reading past its end would be reading whatever follows it in the layer's
    // data segment. Refuse rather than trim -- a same-version disagreement is a build mistake, not
    // a compatibility case to absorb.
    if (api->struct_size < sizeof(HaloVrLayerApi)) {
        _snprintf_s(g_status, sizeof(g_status), _TRUNCATE,
                    "REFUSING the layer: it reports ABI %u but a struct of %u bytes where this "
                    "plugin expects %u. Same version, different header -- one of the two was built "
                    "against a stale XrLayerAbi.h.",
                    (unsigned)api->abi_version, (unsigned)api->struct_size,
                    (unsigned)sizeof(HaloVrLayerApi));
        logf("%s", g_status);
        g_state.store(0, std::memory_order_release);
        return;
    }

    g_api = api;
    _snprintf_s(g_status, sizeof(g_status), _TRUNCATE,
                "layer AVAILABLE -- %s", api->build_stamp != nullptr ? api->build_stamp : "(no stamp)");
    logf("%s", g_status);
    // Say it once, plainly, because the next person to read a log will want the distinction spelled
    // out rather than inferred.
    logf("attachment tier = ApiLayer. This is the SHIPPING route: no PDB, no signature, no hook. "
         "Present does not mean presenting -- watch frames_seen/layers_appended.");
    g_state.store(1, std::memory_order_release);
}

}   // namespace

bool xrbridge_available() {
    int s = g_state.load(std::memory_order_acquire);
    if (s < 0) {
        probe();
        s = g_state.load(std::memory_order_acquire);
    }
    return s == 1;
}

const HaloVrLayerApi* xrbridge_api() {
    return xrbridge_available() ? g_api : nullptr;
}

bool xrbridge_set_projection_mono(int mode) {
    const HaloVrLayerApi* api = xrbridge_api();
    if (api == nullptr) return false;
    // THE SIZE CHECK IS THE WHOLE POINT OF THIS WRAPPER. set_projection_mono was appended after ABI
    // 1 shipped, so an older layer negotiates the same version with a struct that ENDS before this
    // slot; reading it there is reading past the layer's data. A same-version, smaller struct is
    // "the call does not exist", not an error.
    constexpr size_t kNeed = offsetof(HaloVrLayerApi, set_projection_mono)
                           + sizeof(((HaloVrLayerApi*)nullptr)->set_projection_mono);
    if (api->struct_size < kNeed || api->set_projection_mono == nullptr) return false;
    return api->set_projection_mono(mode) == 1;
}

const char* xrbridge_status() {
    if (g_state.load(std::memory_order_acquire) < 0) return "not probed";
    if (g_api != nullptr && g_api->status != nullptr) {
        // Prefer the LAYER's own line when we have one: it carries the live counters, and a counter
        // read from the thing doing the work beats one relayed by the thing asking for it.
        return g_api->status();
    }
    return g_status;
}

}   // namespace halo
