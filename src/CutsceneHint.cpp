// SteamVR overlay cutscene hint. Doctrine in CutsceneHint.hpp; config in Config.hpp (cut_hint).

#include "uevr/API.hpp"
#include "CutsceneHint.hpp"
#include "Config.hpp"

#include <Windows.h>
#include <cstdio>
#include <cstring>
#include <string>

// Interface declarations only -- never the inline VR_Init helpers, which would need the import
// library. Everything is resolved from openvr_api.dll at runtime instead.
#include "thirdparty/openvr.h"

using uevr::API;

namespace halo {
namespace {

// The DLL's C exports, resolved by hand so nothing links against openvr_api.
typedef uint32_t (__cdecl* Fn_InitInternal2)(vr::EVRInitError*, vr::EVRApplicationType, const char*);
typedef void*    (__cdecl* Fn_GetGenericInterface)(const char*, vr::EVRInitError*);

enum class HintState { UNTRIED, READY, FAILED };
HintState g_state = HintState::UNTRIED;
vr::IVROverlay* g_overlay = nullptr;
vr::VROverlayHandle_t g_handle = vr::k_ulOverlayHandleInvalid;
bool g_shown = false;

// One line on the way down, never spam: FAILED latches for the session.
void fail(const char* what) {
    g_state = HintState::FAILED;
    API::get()->log_info("[Halo-CampE-UEVR] cutscene hint: %s -- hint disabled this session", what);
}

// openvr_api.dll: already in the process (an OpenVR session), next to the game exe (the
// headless-injection layout), or wherever %LOCALAPPDATA%\openvr\openvrpaths.vrpath says the
// SteamVR runtime lives (the canonical discovery every OpenVR app uses).
HMODULE load_openvr_api() {
    if (HMODULE m = GetModuleHandleW(L"openvr_api.dll")) return m;

    wchar_t exe[MAX_PATH]{};
    if (GetModuleFileNameW(nullptr, exe, MAX_PATH) > 0) {
        std::wstring dir(exe);
        const size_t cut = dir.find_last_of(L'\\');
        if (cut != std::wstring::npos) {
            dir.resize(cut + 1);
            if (HMODULE m = LoadLibraryW((dir + L"openvr_api.dll").c_str())) return m;
        }
    }

    wchar_t local[MAX_PATH]{};
    if (GetEnvironmentVariableW(L"LOCALAPPDATA", local, MAX_PATH) == 0) return nullptr;
    std::wstring vrpath = std::wstring(local) + L"\\openvr\\openvrpaths.vrpath";

    FILE* f = nullptr;
    if (_wfopen_s(&f, vrpath.c_str(), L"rb") != 0 || f == nullptr) return nullptr;
    char buf[4096]{};
    const size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    buf[n] = 0;

    // Minimal parse: first string of the "runtime" array. The file is tiny, machine-written
    // JSON; a full parser earns nothing here.
    const char* rt = strstr(buf, "\"runtime\"");
    if (rt == nullptr) return nullptr;
    const char* open = strchr(rt, '[');
    if (open == nullptr) return nullptr;
    const char* q1 = strchr(open, '"');
    if (q1 == nullptr) return nullptr;
    const char* q2 = strchr(q1 + 1, '"');
    if (q2 == nullptr) return nullptr;

    std::string path(q1 + 1, q2);
    // JSON escapes backslashes.
    std::string un;
    for (size_t i = 0; i < path.size(); ++i) {
        if (path[i] == '\\' && i + 1 < path.size() && path[i + 1] == '\\') { un += '\\'; ++i; }
        else un += path[i];
    }
    un += "\\bin\\win64\\openvr_api.dll";
    return LoadLibraryA(un.c_str());
}

bool init_once() {
    if (g_state == HintState::READY)  return true;
    if (g_state == HintState::FAILED) return false;

    HMODULE mod = load_openvr_api();
    if (mod == nullptr) { fail("openvr_api.dll not found (no SteamVR?)"); return false; }

    auto init = (Fn_InitInternal2)GetProcAddress(mod, "VR_InitInternal2");
    auto getiface = (Fn_GetGenericInterface)GetProcAddress(mod, "VR_GetGenericInterface");
    if (init == nullptr || getiface == nullptr) { fail("openvr_api exports missing"); return false; }

    // A SECOND client connection, as an overlay app -- deliberately independent of the game's
    // own OpenXR session, which is the whole point.
    vr::EVRInitError err = vr::VRInitError_None;
    init(&err, vr::VRApplication_Overlay, nullptr);
    if (err != vr::VRInitError_None) { fail("VR_Init(Overlay) refused (SteamVR not running?)"); return false; }

    g_overlay = (vr::IVROverlay*)getiface(vr::IVROverlay_Version, &err);
    if (g_overlay == nullptr || err != vr::VRInitError_None) { fail("IVROverlay unavailable"); return false; }

    if (g_overlay->CreateOverlay("halo_vr.cutscene_hint", "Halo VR cutscene hint", &g_handle)
            != vr::VROverlayError_None || g_handle == vr::k_ulOverlayHandleInvalid) {
        fail("CreateOverlay failed");
        return false;
    }

    // The image ships next to halo_vr.cfg, so the profile directory is already known.
    char png[MAX_PATH]{};
    strncpy_s(png, sizeof(png), g_cfg_path, _TRUNCATE);
    if (char* slash = strrchr(png, '\\')) *(slash + 1) = 0;
    strncat_s(png, sizeof(png), "cutscene_hint.png", _TRUNCATE);
    if (g_overlay->SetOverlayFromFile(g_handle, png) != vr::VROverlayError_None) {
        fail("cutscene_hint.png missing from the profile folder");
        return false;
    }

    g_state = HintState::READY;
    API::get()->log_info("[Halo-CampE-UEVR] cutscene hint overlay ready (%s)", png);
    return true;
}

}   // namespace

void cutscene_hint_show() {
    if (!g_cfg.cut_hint || g_shown) return;
    if (!init_once()) return;

    // HEAD-LOCKED, a little low and ahead: the one anchoring the player cannot lose, which
    // matters because this shows exactly when the rest of the view may be black. Position is
    // re-applied on every show so the cfg values tune live.
    vr::HmdMatrix34_t m{};
    m.m[0][0] = 1.0f; m.m[1][1] = 1.0f; m.m[2][2] = 1.0f;
    m.m[0][3] = 0.0f;
    m.m[1][3] = -g_cfg.cut_hint_drop;
    m.m[2][3] = -g_cfg.cut_hint_dist;   // OpenVR: -Z is forward
    g_overlay->SetOverlayTransformTrackedDeviceRelative(g_handle, vr::k_unTrackedDeviceIndex_Hmd, &m);
    g_overlay->SetOverlayWidthInMeters(g_handle, g_cfg.cut_hint_w);
    g_overlay->ShowOverlay(g_handle);
    g_shown = true;
}

void cutscene_hint_hide() {
    if (!g_shown) return;
    g_shown = false;
    if (g_state == HintState::READY && g_overlay != nullptr) {
        g_overlay->HideOverlay(g_handle);
    }
}

}   // namespace halo
