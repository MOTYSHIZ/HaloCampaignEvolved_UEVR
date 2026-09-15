#include "HeightCal.hpp"
#include "core/config/CfgRead.hpp"

#include "BlamDrive.hpp"
#include "core/UnitState.hpp"
#include "Config.hpp"
#include "core/Services.hpp"
#include "Math.hpp"                    // clampf
#include "Rig.hpp"                     // g_rig_component: the weapon the floor trace ignores
#include "core/EyeTrace.hpp"
#include "core/XrDisplayTime.hpp"
#include "core/host/PluginState.hpp"
#include "XrLayerBridge.hpp"
#include "thirdparty/openvr.h"
#include "uevr/API.hpp"

#include <Windows.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <vector>

using namespace uevr;

namespace halo {

namespace {

enum Src : int { SRC_AUTO = 0, SRC_OPENXR = 1, SRC_OPENVR = 2, SRC_UEVR = 3 };
enum Mode : int { MODE_ABSOLUTE = 0, MODE_SEATED = 1, MODE_EYES = 2 };

const char* src_name(int s) {
    switch (s) {
    case SRC_OPENXR: return "OpenXR-stage";
    case SRC_OPENVR: return "OpenVR-standing";
    case SRC_UEVR:   return "UEVR-pose";
    default:         return "auto";
    }
}

const char* mode_name(int m) {
    switch (m) {
    case MODE_ABSOLUTE: return "absolute";
    case MODE_SEATED:   return "seated";
    case MODE_EYES:     return "eyes";
    default:            return "none";
    }
}

void hlog(const char* fmt, ...) {
    if (g_cfg.height_log <= 0) return;
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    API::get()->log_info("[Halo-CampE-UEVR] %s", buf);
}

// ---- SOURCE 1: OpenXR STAGE ------------------------------------------------------------------
//
// The plugin cannot call OpenXR directly: UEVR links its loader statically (XrLayerAttach.hpp), so
// the entry points are taken from the HALOVR API layer, which sits on the same chain as UEVR's
// session. We create our own STAGE (floor origin) and VIEW spaces on that session.
//
// WHAT IS MEASURED is the floor offset of UEVR's pose space: UEVR's pose-space origin located in
// STAGE. That is time-invariant, so a one-frame-old display time costs nothing. The VIEW locate is
// kept as a cross-check: head-in-STAGE minus (UEVR hmd.y + offset) should sit near zero.
struct XrProbe {
    int       state   = 0;   // 0 not ready (retry), 1 spaces created, -1 refused (sticky until heightsrc changes)
    XrSession session = XR_NULL_HANDLE;
    XrSpace   stage   = XR_NULL_HANDLE;
    XrSpace   view    = XR_NULL_HANDLE;
    PFN_xrCreateReferenceSpace create = nullptr;
    PFN_xrLocateSpace          locate = nullptr;
    int       no_time = 0;
    bool      ever_ok = false;
    char      why[240] = "not tried";
} g_xr;

void xr_refuse(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_xr.why, sizeof(g_xr.why), fmt, ap);
    va_end(ap);
    g_xr.state = -1;
    hlog("HEIGHT: OpenXR source unavailable -- %s", g_xr.why);
}

bool xr_probe(float hmd_y, float* out_off, float* out_resid) {
    if (g_xr.state < 0) return false;
    if (!API::VR::is_openxr()) { xr_refuse("the runtime is not OpenXR"); return false; }
    const HaloVrLayerApi* api = xrbridge_api();
    if (api == nullptr) {
        xr_refuse("the HALOVR API layer is not loaded (it is the plugin's only route to OpenXR entry points)");
        return false;
    }
    const XrSession sess = api->get_session();
    if (api->get_instance() == XR_NULL_HANDLE || sess == XR_NULL_HANDLE) {
        std::snprintf(g_xr.why, sizeof(g_xr.why), "waiting for the OpenXR session");
        return false;
    }
    if (g_xr.state == 1 && sess != g_xr.session) {
        // A new session: the old spaces died with the old session. Never destroy them against it.
        g_xr.stage = XR_NULL_HANDLE;
        g_xr.view  = XR_NULL_HANDLE;
        g_xr.state = 0;
        hlog("HEIGHT: OpenXR session changed -- recreating the probe spaces");
    }
    if (g_xr.state == 0) {
        g_xr.create = (PFN_xrCreateReferenceSpace)api->get_proc("xrCreateReferenceSpace");
        g_xr.locate = (PFN_xrLocateSpace)api->get_proc("xrLocateSpace");
        if (g_xr.create == nullptr || g_xr.locate == nullptr) {
            xr_refuse("the API layer could not resolve xrCreateReferenceSpace / xrLocateSpace");
            return false;
        }
        XrReferenceSpaceCreateInfo ci{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
        ci.poseInReferenceSpace.orientation.w = 1.0f;
        ci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
        XrResult r = g_xr.create(sess, &ci, &g_xr.stage);
        if (XR_FAILED(r)) {
            xr_refuse("xrCreateReferenceSpace(STAGE) returned %d -- no floor-level stage on this runtime", (int)r);
            return false;
        }
        ci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
        r = g_xr.create(sess, &ci, &g_xr.view);
        if (XR_FAILED(r)) {
            xr_refuse("xrCreateReferenceSpace(VIEW) returned %d", (int)r);
            return false;
        }
        g_xr.session = sess;
        g_xr.state   = 1;
        hlog("HEIGHT: OpenXR STAGE + VIEW probe spaces created on session %p through the API layer",
             (void*)sess);
    }

    const int64_t t = xr_display_time();
    if (t == 0) {
        // Probed at 2 Hz: 20 misses is ~10 s with no frame ever reaching the submit path.
        if (++g_xr.no_time >= 20) {
            xr_refuse("no display time from the submit path in ~10 s (the API layer end-frame "
                      "callback is not registered)");
        } else {
            std::snprintf(g_xr.why, sizeof(g_xr.why), "waiting for a display time");
        }
        return false;
    }

    XrSpaceLocation head{XR_TYPE_SPACE_LOCATION};
    XrResult r = g_xr.locate(g_xr.view, g_xr.stage, (XrTime)t, &head);
    if (XR_FAILED(r) || (head.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) == 0) {
        std::snprintf(g_xr.why, sizeof(g_xr.why), "xrLocateSpace(VIEW in STAGE) = %d flags=0x%llx",
                      (int)r, (unsigned long long)head.locationFlags);
        return false;   // not sticky: tracking loss is transient
    }

    float off = head.pose.position.y - hmd_y;
    const char* how = "head";
    auto* p = API::get()->param();
    if (p != nullptr && p->openxr != nullptr && p->openxr->get_stage_space != nullptr) {
        const XrSpace us = (XrSpace)p->openxr->get_stage_space();
        if (us != XR_NULL_HANDLE) {
            XrSpaceLocation org{XR_TYPE_SPACE_LOCATION};
            const XrResult r2 = g_xr.locate(us, g_xr.stage, (XrTime)t, &org);
            if (XR_SUCCEEDED(r2) && (org.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0) {
                off = org.pose.position.y;
                how = "origin";
            }
        }
    }
    if (!g_xr.ever_ok) {
        g_xr.ever_ok = true;
        hlog("HEIGHT: OpenXR floor probe OK -- head %.3f m above STAGE floor, UEVR hmd.y %.3f, UEVR "
             "pose-space origin %.3f m above floor (from %s)", head.pose.position.y, hmd_y, off, how);
    }
    *out_off   = off;
    *out_resid = head.pose.position.y - (hmd_y + off);
    return true;
}

// ---- SOURCE 2: OpenVR standing universe ------------------------------------------------------
//
// UEVR hands out its own IVRSystem (PluginLoader.cpp: runtime->hmd). The vendored openvr.h is the
// same interface version UEVR builds against (IVRSystem_022), so the vtable matches.
struct VrProbe {
    int  state   = 0;
    bool ever_ok = false;
    char why[240] = "not tried";
} g_vr;

bool vr_probe(float hmd_y, float* out_off, float* out_resid) {
    if (g_vr.state < 0) return false;
    if (!API::VR::is_openvr()) {
        std::snprintf(g_vr.why, sizeof(g_vr.why), "the runtime is not OpenVR");
        g_vr.state = -1;
        hlog("HEIGHT: OpenVR source unavailable -- %s", g_vr.why);
        return false;
    }
    auto* p = API::get()->param();
    if (p == nullptr || p->openvr == nullptr || p->openvr->get_vr_system == nullptr) {
        std::snprintf(g_vr.why, sizeof(g_vr.why), "UEVR exposes no IVRSystem");
        g_vr.state = -1;
        hlog("HEIGHT: OpenVR source unavailable -- %s", g_vr.why);
        return false;
    }
    auto* sys = reinterpret_cast<vr::IVRSystem*>(p->openvr->get_vr_system());
    if (sys == nullptr) { std::snprintf(g_vr.why, sizeof(g_vr.why), "waiting for IVRSystem"); return false; }
    vr::TrackedDevicePose_t pose{};
    sys->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0f, &pose, 1);
    if (!pose.bPoseIsValid) { std::snprintf(g_vr.why, sizeof(g_vr.why), "HMD pose not valid"); return false; }
    const float y = pose.mDeviceToAbsoluteTracking.m[1][3];
    if (!g_vr.ever_ok) {
        g_vr.ever_ok = true;
        hlog("HEIGHT: OpenVR floor probe OK -- head %.3f m above the standing floor, UEVR hmd.y %.3f",
             y, hmd_y);
    }
    g_vr.state = 1;
    *out_off   = y - hmd_y;
    *out_resid = 0.0f;
    return true;
}

// ---- source selection and the floor offset ---------------------------------------------------
int   g_last_cfg_src = -1;
int   g_active_src   = SRC_UEVR;
int   g_logged_src   = -1;
float g_off_hist[9]{};
int   g_off_n = 0, g_off_i = 0;
bool  g_floor_known = false;
float g_floor_off   = 0.0f;
float g_resid       = 0.0f;
float g_probe_timer = 0.0f;
float g_since_ok    = 1.0e9f;

float median9() {
    float tmp[9];
    for (int i = 0; i < g_off_n; ++i) tmp[i] = g_off_hist[i];
    std::sort(tmp, tmp + g_off_n);
    return (g_off_n & 1) ? tmp[g_off_n / 2] : 0.5f * (tmp[g_off_n / 2 - 1] + tmp[g_off_n / 2]);
}

// A floor source that has neither delivered nor refused yet.
bool source_pending() {
    const int want = g_cfg.height_src;
    const bool xr_wait = API::VR::is_openxr() && g_xr.state >= 0 && !g_xr.ever_ok;
    const bool vr_wait = API::VR::is_openvr() && g_vr.state >= 0 && !g_vr.ever_ok;
    if (want == SRC_OPENXR) return g_xr.state >= 0 && !g_xr.ever_ok;
    if (want == SRC_OPENVR) return g_vr.state >= 0 && !g_vr.ever_ok;
    if (want == SRC_UEVR)   return false;
    return xr_wait || vr_wait;
}

void update_source(float hmd_y, float dt) {
    if (g_cfg.height_src != g_last_cfg_src) {
        if (g_last_cfg_src != -1) hlog("HEIGHT: heightsrc %d -> %d, re-probing", g_last_cfg_src, g_cfg.height_src);
        g_last_cfg_src = g_cfg.height_src;
        if (g_xr.state < 0) { g_xr.state = (g_xr.stage != XR_NULL_HANDLE && g_xr.view != XR_NULL_HANDLE) ? 1 : 0; }
        g_xr.no_time = 0;
        if (g_vr.state < 0) g_vr.state = 0;
        g_off_n = 0; g_off_i = 0;
        g_floor_known = false;
        g_logged_src = -1;
        g_probe_timer = 0.0f;
    }

    g_since_ok += dt;
    g_probe_timer -= dt;
    if (g_probe_timer <= 0.0f) {
        g_probe_timer = 0.5f;
        const int want = g_cfg.height_src;
        float off = 0.0f, resid = 0.0f;
        int got = SRC_UEVR;
        if (want == SRC_OPENXR || want == SRC_AUTO) {
            if (xr_probe(hmd_y, &off, &resid)) got = SRC_OPENXR;
        }
        if (got == SRC_UEVR && (want == SRC_OPENVR || (want == SRC_AUTO && !(g_xr.state >= 0 && API::VR::is_openxr())))) {
            if (vr_probe(hmd_y, &off, &resid)) got = SRC_OPENVR;
        }
        if (got != SRC_UEVR) {
            g_off_hist[g_off_i] = off;
            g_off_i = (g_off_i + 1) % 9;
            if (g_off_n < 9) ++g_off_n;
            g_floor_off   = median9();
            g_floor_known = true;
            g_resid       = resid;
            g_since_ok    = 0.0f;
            g_active_src  = got;
        } else if (g_since_ok > 5.0f) {
            g_active_src = SRC_UEVR;
            // UEVR's OpenVR runtime tracks in TrackingUniverseStanding, so its pose alone still
            // knows the floor there. On OpenXR (LOCAL pose space) it does not.
            g_floor_known = API::VR::is_openvr();
            if (g_floor_known) g_floor_off = 0.0f;
        }
    }

    if (g_active_src != g_logged_src && !source_pending()) {
        g_logged_src = g_active_src;
        if (g_active_src == SRC_UEVR) {
            hlog("HEIGHT: source = UEVR-pose (requested %s), floor %s. OpenXR: %s | OpenVR: %s",
                 src_name(g_cfg.height_src), g_floor_known ? "known (standing universe)" : "UNKNOWN",
                 g_xr.why, g_vr.why);
        } else {
            hlog("HEIGHT: source = %s (requested %s), UEVR pose-space floor offset %.3f m, residual %.3f m",
                 src_name(g_active_src), src_name(g_cfg.height_src), g_floor_off, g_resid);
        }
    }
}

// ---- world scale: UE cm per VR metre, re-read with the config poll ----------------------------
float g_S = 100.0f;
float g_scale_timer = 0.0f;

float resolve_S() {
    float ws = 1.0f;
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

// ---- E_game: the camera's height above the character's feet, three ways ----------------------
//
// ONE FILTER FOR ALL THREE, so the log compares measurements and not filters. A change of at most
// heightestep cm per tick is taken AT ONCE (walking, slopes, bob, the crouch camera: no lag). A
// larger jump is HELD until the new value has stayed within 2 cm for heightholdms -- a stair edge
// passing under the trace, the start of a jump or a fall, a trace that hit something passing by.
// Implausible raws (under 20 or over 400 cm, or no hit) hold as well.
struct EFilt {
    bool  have = false;
    float E = 0.0f;
    float cand = 0.0f;
    float cand_t = 0.0f;
    bool  raw_ok = false;
    float raw = 0.0f;
    bool  took = false;   // this tick's raw was accepted within the step (stable ground)
};

void efilt(EFilt& f, bool ok, float raw, float dt) {
    f.took = false;
    f.raw_ok = ok && std::isfinite(raw) && raw >= 20.0f && raw <= 400.0f;
    f.raw = ok ? raw : 0.0f;
    if (!f.raw_ok) return;
    if (!f.have) { f.E = raw; f.have = true; f.cand = raw; f.cand_t = 0.0f; f.took = true; return; }
    if (std::fabs(raw - f.E) <= g_cfg.height_e_step) {
        f.E = raw; f.cand = raw; f.cand_t = 0.0f; f.took = true;
        return;
    }
    if (std::fabs(raw - f.cand) <= 2.0f) f.cand_t += dt;
    else { f.cand = raw; f.cand_t = 0.0f; }
    if (f.cand_t * 1000.0f >= g_cfg.height_hold_ms) { f.E = raw; f.cand_t = 0.0f; }
}

EFilt g_etr, g_eblam, g_epawn;
float g_floor_z = 0.0f;                    // UE cm, from the trace
float g_blam_off = 0.0f;  bool g_blam_off_have = false;   // unit z (cm) minus floor z, learned
float g_pawn_off = 0.0f;  bool g_pawn_off_have = false;   // pawn root z minus floor z, learned
float g_unit_zcm = 0.0f;
float g_root_z = 0.0f;
float g_half = 0.0f;      bool g_half_ok = false;
int   g_e_used = 0, g_e_used_logged = -1;

// UE pawn capsule. Everything resolved by reflection; any gap leaves the capsule unused.
API::UObject* g_cap_class_seen = nullptr;
int32_t       g_cap_ret = -1;
bool          g_cap_ret_double = false;
bool          g_cap_logged = false;

std::wstring fname_of(API::FField* f) {
    if (f == nullptr) return L"";
    auto* n = f->get_fname();
    return (n != nullptr) ? n->to_string() : L"";
}

bool pawn_capsule_half(API::UObject* pawn, float* out) {
    auto** pp = pawn->get_property_data<API::UObject*>(L"CapsuleComponent");
    API::UObject* cap = (pp != nullptr) ? *pp : nullptr;
    if (cap == nullptr) {
        if (!g_cap_logged) { g_cap_logged = true; hlog("HEIGHT: pawn has no CapsuleComponent property (or it is null) -- pawn feet use the learned offset"); }
        return false;
    }
    auto* cls = cap->get_class();
    if (cls == nullptr) return false;
    if ((API::UObject*)cls != g_cap_class_seen) {
        g_cap_class_seen = (API::UObject*)cls;
        g_cap_ret = -1;
        std::wstring rclass;
        if (auto* fn = cls->find_function(L"GetScaledCapsuleHalfHeight")) {
            for (auto* f = fn->get_child_properties(); f != nullptr; f = f->get_next()) {
                if (fname_of(f) != L"ReturnValue") continue;
                auto* fc = f->get_class();
                auto* nm = (fc != nullptr) ? fc->get_fname() : nullptr;
                rclass = (nm != nullptr) ? nm->to_string() : L"";
                if (rclass == L"FloatProperty" || rclass == L"DoubleProperty") {
                    g_cap_ret = reinterpret_cast<API::FProperty*>(f)->get_offset();
                    g_cap_ret_double = (rclass == L"DoubleProperty");
                }
            }
        }
        hlog("HEIGHT: pawn capsule %p GetScaledCapsuleHalfHeight ReturnValue offset=%d (%ls)",
             (void*)cap, g_cap_ret, rclass.c_str());
    }
    if (g_cap_ret < 0 || g_cap_ret > 40) return false;
    alignas(16) uint8_t buf[64] = {0};
    cap->call_function(L"GetScaledCapsuleHalfHeight", buf);
    const double v = g_cap_ret_double ? *reinterpret_cast<double*>(buf + g_cap_ret)
                                      : (double)*reinterpret_cast<float*>(buf + g_cap_ret);
    if (!std::isfinite(v) || v <= 1.0 || v > 500.0) return false;
    *out = (float)v;
    return true;
}

bool actor_location(API::UObject* obj, Vec3* out) {
    alignas(16) uint8_t buf[64] = {0};
    obj->call_function(L"K2_GetActorLocation", buf);
    const auto* d = reinterpret_cast<const double*>(buf);
    if (!std::isfinite(d[0]) || !std::isfinite(d[1]) || !std::isfinite(d[2])) return false;
    if (d[0] == 0.0 && d[1] == 0.0 && d[2] == 0.0) return false;
    *out = Vec3{(float)d[0], (float)d[1], (float)d[2]};
    return true;
}

void measure_e(const Vec3& cam, bool active, API::UObject* const* ignore, int n_ignore, float dt) {
    if (!active) return;   // hold all three

    // 1: downward trace from the camera.
    Vec3 imp{};
    const Vec3 down{cam.x, cam.y, cam.z - g_cfg.height_trace_max};
    const bool hit = kismet_line_trace(cam, down, ignore, n_ignore, g_cfg.height_trace_channel, &imp);
    if (hit) g_floor_z = imp.z;
    efilt(g_etr, hit, cam.z - imp.z, dt);

    // 2: the Blam biped's world position (+0x20, Blam units) and an offset to the feet.
    const bool blam_ok = g_unit_pvalid.load(std::memory_order_relaxed);
    g_unit_zcm = g_unit_pz.load(std::memory_order_relaxed) * g_cfg.height_biped_scale;
    if (blam_ok && g_etr.took) {
        const float o = g_unit_zcm - g_floor_z;
        if (!g_blam_off_have) { g_blam_off = o; g_blam_off_have = true; }
        else g_blam_off += (o - g_blam_off) * clampf(dt / 1.0f, 0.0f, 1.0f);
    }
    const bool blam_have_off = (g_cfg.height_biped_feet != 0.0f) || g_blam_off_have;
    const float blam_off = (g_cfg.height_biped_feet != 0.0f) ? g_cfg.height_biped_feet : g_blam_off;
    efilt(g_eblam, blam_ok && blam_have_off, cam.z - (g_unit_zcm - blam_off), dt);

    // 3: the UE pawn root, minus the capsule half-height (or a learned / configured offset).
    bool pawn_ok = false;
    float pawn_off = 0.0f;
    if (auto* pawn = reinterpret_cast<API::UObject*>(API::get()->get_local_pawn(0))) {
        Vec3 root{};
        if (actor_location(pawn, &root)) {
            g_root_z = root.z;
            g_half_ok = pawn_capsule_half(pawn, &g_half);
            if (g_etr.took) {
                const float o = g_root_z - g_floor_z;
                if (!g_pawn_off_have) { g_pawn_off = o; g_pawn_off_have = true; }
                else g_pawn_off += (o - g_pawn_off) * clampf(dt / 1.0f, 0.0f, 1.0f);
            }
            if (g_cfg.height_pawn_feet != 0.0f) { pawn_off = g_cfg.height_pawn_feet; pawn_ok = true; }
            else if (g_half_ok)                 { pawn_off = g_half;                 pawn_ok = true; }
            else if (g_pawn_off_have)           { pawn_off = g_pawn_off;             pawn_ok = true; }
        }
    }
    efilt(g_epawn, pawn_ok, cam.z - (g_root_z - pawn_off), dt);
}

const char* e_name(int i) {
    switch (i) { case 1: return "trace"; case 2: return "blam"; case 3: return "pawn"; default: return "none"; }
}

EFilt* e_filter(int i) {
    switch (i) { case 1: return &g_etr; case 2: return &g_eblam; case 3: return &g_epawn; default: return nullptr; }
}

// Returns the selected E, falling back in order when the selected one has never measured.
bool pick_e(float* out) {
    int want = (g_cfg.height_eye >= 1 && g_cfg.height_eye <= 3) ? g_cfg.height_eye : 1;
    int used = 0;
    if (e_filter(want)->have) used = want;
    else for (int i = 1; i <= 3; ++i) if (e_filter(i)->have) { used = i; break; }
    if (used != g_e_used_logged) {
        g_e_used_logged = used;
        if (used == 0) hlog("HEIGHT: no E_game measurement yet (heighteye=%d)", want);
        else if (used != want) hlog("HEIGHT: E_game from %s -- requested %s has not measured", e_name(used), e_name(want));
        else hlog("HEIGHT: E_game from %s", e_name(used));
    }
    g_e_used = used;
    if (used == 0) return false;
    *out = e_filter(used)->E;
    return true;
}

// ---- eyes mode (the standing head mapped to the character's eyes) -----------------------------
struct Sample { float t; float y; };

std::vector<float>  g_win;
float               g_win_t = 0.0f;
std::deque<Sample>  g_ring;
float               g_clock = 0.0f;
bool                g_have_H = false;
float               g_H = 0.0f;
float               g_low_persist = 0.0f;
float               g_cont_timer = 0.0f;

float percentile(std::vector<float> v, float q) {
    if (v.empty()) return 0.0f;
    std::sort(v.begin(), v.end());
    size_t i = (size_t)std::floor(q * (float)(v.size() - 1) + 0.5f);
    if (i >= v.size()) i = v.size() - 1;
    return v[i];
}

void accept_H(float cand, const char* how, size_t n, float secs) {
    const bool had = g_have_H;
    const float prev = g_H;
    g_H = cand;
    g_have_H = true;
    hlog("HEIGHT EYES CAL (%s): standing head at UEVR y %.3f%s, %zu samples over %.1f s%s", how, cand,
         g_floor_known ? " (floor known)" : " (floor unknown)", n, secs, had ? "" : " -- first calibration");
    if (had) hlog("HEIGHT EYES CAL: changed by %+.3f m", cand - prev);
}

// ---- state shared by the modes ----------------------------------------------------------------
Vec3  g_prev{};
bool  g_have_prev = false;
float g_vy = 0.0f, g_vl = 0.0f;
bool  g_key_prev = false;
std::atomic<bool> g_menu_req{false};

// One-shot capture for a calibration: seated offset (0.5 s) or eyes H (1 s).
float              g_cap_t = -1.0f;
int                g_cap_mode = -1;
const char*        g_cap_why = "";
std::vector<float> g_cap_s;

int   g_cfg_mode_prev = -1;
int   g_eff_mode = -1;
bool  g_auto_seated = false;
bool  g_autoseat_done = false;
int   g_autoseat_cfg_prev = -1;
std::vector<float> g_seat_s;
float g_seat_t = 0.0f;

bool  g_have_O = false;
float g_O = 0.0f, g_T = 0.0f, g_head0 = 0.0f;

bool  g_have_out = false;
bool  g_slew = false;
float g_out = 0.0f;
float g_log_t = 0.0f;
std::string g_status;
std::chrono::steady_clock::time_point g_last_call{};

bool uevr_menu_open() {
    auto* p = API::get()->param();
    return p != nullptr && p->functions != nullptr && p->functions->is_drawing_ui != nullptr &&
           p->functions->is_drawing_ui();
}

void start_capture(int mode, const char* why) {
    g_cap_t = 0.0f;
    g_cap_mode = mode;
    g_cap_why = why;
    g_cap_s.clear();
    hlog("HEIGHT: calibration capture started (%s, %s) -- hold still for %.1f s", mode_name(mode), why,
         mode == MODE_SEATED ? 0.5f : 1.0f);
}

}  // namespace

bool height_request_calibrate() {
    g_menu_req.store(true, std::memory_order_relaxed);
    return true;
}

std::string height_status_line() {
    return (g_cfg.height_cal != 0) ? g_status : std::string();
}

bool height_tick(const Vec3& hmd, float so_y, bool active, bool key_focus, float dt,
                 API::UObject* const* ignore, int n_ignore, float* out_y) {
    const auto now = std::chrono::steady_clock::now();
    if (g_last_call.time_since_epoch().count() != 0 &&
        std::chrono::duration<float>(now - g_last_call).count() > 1.0f) {
        g_have_out = false;   // re-enabled after a gap: slew from where the origin is now
        g_have_prev = false;
    }
    g_last_call = now;
    if (!(dt > 0.0f) || dt > 0.5f) dt = 0.0f;
    g_clock += dt;

    update_source(hmd.y, dt);

    g_scale_timer -= dt;
    if (g_scale_timer <= 0.0f) {
        g_scale_timer = 2.0f;
        const float s = resolve_S();
        if (std::fabs(s - g_S) > 0.01f) hlog("HEIGHT: world scale %.1f UE cm per VR metre (100 x VR_WorldScale)", s);
        g_S = s;
    }
    const float S = g_S;
    const float K = (g_cfg.height_scale == 1) ? 100.0f : S;

    // STILL: a head settling at one height, not one moving through it. Filtered speeds, ~150 ms.
    if (g_have_prev && dt > 0.0f) {
        const float vy = std::fabs(hmd.y - g_prev.y) / dt;
        const float lx = hmd.x - g_prev.x, lz = hmd.z - g_prev.z;
        const float vl = std::sqrt(lx * lx + lz * lz) / dt;
        const float a = clampf(dt / 0.15f, 0.0f, 1.0f);
        g_vy += (vy - g_vy) * a;
        g_vl += (vl - g_vl) * a;
    }
    g_prev = hmd;
    g_have_prev = true;
    const bool still = (g_vy < 0.12f) && (g_vl < 0.6f);
    const bool pending = source_pending();
    const float head_abs = hmd.y + g_floor_off;

    Vec3 cam{};
    const bool have_cam = eye_body_world(&cam);
    if (have_cam) measure_e(cam, active, ignore, n_ignore, dt);
    float E = 0.0f;
    const bool have_E = pick_e(&E);

    // ---- MODE -----------------------------------------------------------------------------------
    const int cfg_mode = (g_cfg.height_mode >= 0 && g_cfg.height_mode <= 2) ? g_cfg.height_mode : 0;
    if (cfg_mode != g_cfg_mode_prev || g_cfg.height_auto_seat != g_autoseat_cfg_prev) {
        if (g_cfg_mode_prev != -1) hlog("HEIGHT: heightmode %s -> %s", mode_name(g_cfg_mode_prev), mode_name(cfg_mode));
        if (cfg_mode == MODE_SEATED && g_cfg_mode_prev != MODE_SEATED) g_have_O = false;   // switched to seated: calibrate now
        g_cfg_mode_prev = cfg_mode;
        g_autoseat_cfg_prev = g_cfg.height_auto_seat;
        g_auto_seated = false;
        g_autoseat_done = false;
        g_seat_s.clear();
        g_seat_t = 0.0f;
    }

    // AUTO-SEAT: ONE SHOT, at the first on-foot moment with a known floor. Accumulates heightseatdwell
    // seconds of STILL head samples; seated when their 90th percentile is under heightseatbelow.
    // Never re-evaluated mid-session, so a crouch later cannot trigger it.
    if (cfg_mode == MODE_ABSOLUTE && g_cfg.height_auto_seat != 0 && !g_autoseat_done && g_floor_known && !pending) {
        if (active && still) { g_seat_s.push_back(head_abs); g_seat_t += dt; }
        if (g_seat_t >= g_cfg.height_seat_dwell && g_seat_s.size() >= 10) {
            const float p90 = percentile(g_seat_s, 0.9f);
            g_autoseat_done = true;
            if (p90 < g_cfg.height_seat_below) {
                g_auto_seated = true;
                g_have_O = false;
                hlog("HEIGHT: AUTO-SEAT -> seated: resting head P90 %.3f m < heightseatbelow %.2f m over %.1f s still",
                     p90, g_cfg.height_seat_below, g_seat_t);
            } else {
                hlog("HEIGHT: AUTO-SEAT -> standing: resting head P90 %.3f m >= heightseatbelow %.2f m over %.1f s still",
                     p90, g_cfg.height_seat_below, g_seat_t);
            }
            g_seat_s.clear();
        }
    }

    int eff = cfg_mode;
    if (eff == MODE_ABSOLUTE && g_auto_seated) eff = MODE_SEATED;
    const char* why = (eff == cfg_mode) ? "heightmode" : "auto-seat";
    if (eff != MODE_EYES && !g_floor_known) {
        if (pending) { eff = -1; why = "floor source still probing"; }
        else { eff = MODE_EYES; why = "floor UNKNOWN (UEVR pose only) -- fallback"; }
    }
    if (eff != g_eff_mode) {
        hlog("HEIGHT MODE: %s -> %s (%s)", mode_name(g_eff_mode), mode_name(eff), why);
        g_eff_mode = eff;
        g_slew = true;
        g_cap_t = -1.0f;
    }

    // ---- CALIBRATION TRIGGERS: the key (never while UEVR's own menu is open) and the menu button.
    bool req = g_menu_req.exchange(false, std::memory_order_relaxed);
    const bool key_down = key_focus && g_cfg.height_key != 0 && (GetAsyncKeyState(g_cfg.height_key) & 0x8000) != 0;
    if (key_down && !g_key_prev) {
        if (uevr_menu_open()) hlog("HEIGHT: key ignored -- UEVR's menu is open");
        else req = true;
    }
    g_key_prev = key_down;
    if (req) {
        if (eff == MODE_SEATED)    start_capture(MODE_SEATED, "recalibrate");
        else if (eff == MODE_EYES) start_capture(MODE_EYES, "recalibrate");
        else hlog("HEIGHT: recalibrate in %s mode -- nothing to calibrate (the floor defines the height)", mode_name(eff));
    }
    if (eff == MODE_SEATED && !g_have_O && g_cap_t < 0.0f) start_capture(MODE_SEATED, "entering seated");

    if (g_cap_t >= 0.0f) {
        g_cap_s.push_back(g_cap_mode == MODE_SEATED ? head_abs : hmd.y);
        g_cap_t += dt;
        const float need = (g_cap_mode == MODE_SEATED) ? 0.5f : 1.0f;
        if (g_cap_t >= need && g_cap_s.size() >= 5) {
            const float med = percentile(g_cap_s, 0.5f);
            if (g_cap_mode == MODE_SEATED) {
                const bool cfg_target = g_cfg.height_seat_target > 0.0f;
                if (cfg_target || have_E) {
                    g_head0 = med;
                    g_T = cfg_target ? g_cfg.height_seat_target : E;
                    g_O = g_T - K * g_head0;
                    g_have_O = true;
                    g_slew = true;
                    g_cap_t = -1.0f;
                    hlog("HEIGHT SEATED CAL (%s): head %.3f m above floor, target %.1f cm (%s), offset %+.1f cm",
                         g_cap_why, g_head0, g_T, cfg_target ? "heightseattarget" : "character eye height E_game", g_O);
                }
                // else: keep capturing until E_game measures (target defaults to it)
            } else {
                accept_H(med, g_cap_why, g_cap_s.size(), g_cap_t);
                g_cap_t = -1.0f;
                g_slew = true;
                g_win.clear(); g_win_t = 0.0f; g_ring.clear(); g_low_persist = 0.0f;
            }
            if (g_cap_s.size() > 4096) g_cap_s.clear();
        }
    }

    // ---- EYES: window / continuous learner for H (unchanged from the first build) -----------------
    if (eff == MODE_EYES) {
        const bool sampling = active && still && g_cap_t < 0.0f;
        const int smode = g_cfg.height_sample;
        if (smode != 2 && !g_have_H) {
            if (sampling) { g_win.push_back(hmd.y); g_win_t += dt; }
            if (g_win.size() > 4096) { g_win.erase(g_win.begin(), g_win.begin() + 2048); g_win_t *= 0.5f; }
            if (g_win_t >= g_cfg.height_window_s && g_win.size() >= 10) {
                accept_H(percentile(g_win, 0.9f), "window", g_win.size(), g_win_t);
                g_win.clear();
                g_win_t = 0.0f;
                g_slew = true;
            }
        }
        if (smode == 1 && g_have_H) {
            if (sampling) g_ring.push_back(Sample{g_clock, hmd.y});
            while (!g_ring.empty() && g_clock - g_ring.front().t > g_cfg.height_window_s) g_ring.pop_front();
            g_cont_timer += dt;
            if (g_cont_timer >= 1.0f) {
                const float step = g_cont_timer;
                g_cont_timer = 0.0f;
                if (g_ring.size() >= 10 && (g_ring.back().t - g_ring.front().t) >= 0.5f * g_cfg.height_window_s) {
                    std::vector<float> ys;
                    ys.reserve(g_ring.size());
                    for (const auto& s : g_ring) ys.push_back(s.y);
                    const float cand = percentile(std::move(ys), 0.9f);
                    if (cand > g_H + 0.01f) {
                        g_H += (std::min)(cand - g_H, 0.05f * step);
                        g_low_persist = 0.0f;
                    } else if (cand < g_H - 0.01f && cand >= g_H - g_cfg.height_band) {
                        g_low_persist += step;
                        if (g_low_persist >= 30.0f) g_H -= (std::min)(g_H - cand, 0.01f * step);
                    } else {
                        g_low_persist = 0.0f;
                    }
                }
            }
        }
    }
    // ---- TARGET -----------------------------------------------------------------------------------
    bool have_target = false;
    float tgt = 0.0f, V_target = -1.0f;
    if (active) {
        if ((eff == MODE_ABSOLUTE || (eff == MODE_SEATED && g_have_O)) && have_E) {
            V_target = K * head_abs + ((eff == MODE_SEATED) ? g_O : 0.0f);
            tgt = hmd.y + (E - V_target) / S;
            have_target = true;
        } else if (eff == MODE_EYES && g_have_H) {
            tgt = g_H - g_cfg.height_trim;
            have_target = true;
        }
    }

    bool own = false;
    if (have_target) {
        if (!g_have_out) { g_out = so_y; g_have_out = true; g_slew = true; }
        const float step = g_cfg.height_slew * dt;
        const float d = tgt - g_out;
        if (g_slew && g_cfg.height_slew > 0.0f && std::fabs(d) > step && step > 0.0f) {
            g_out += (d > 0.0f) ? step : -step;
        } else {
            g_out = tgt;
            g_slew = false;
        }
        own = true;
    } else if (g_have_out && eff != -1) {
        own = true;   // hold the last origin Y (menu, vehicle, cutscene, E_game not yet measured)
    }
    if (own) *out_y = g_out;

    // ---- STATUS + LOG, once a second --------------------------------------------------------------
    g_log_t += dt;
    if (g_log_t >= 1.0f) {
        g_log_t = 0.0f;
        const float so_now = own ? g_out : so_y;
        const float V_pred = have_E ? E + S * (hmd.y - so_now) : -1.0f;
        Vec3 c{};
        const bool have_c = eye_head_offset(&c);
        const float V_meas = (have_E && have_c) ? E + c.z : -1.0f;
        char st[96];
        if (have_E && eff >= 0) std::snprintf(st, sizeof(st), "height=%s %.2f m", mode_name(eff), (V_meas >= 0.0f ? V_meas : V_pred) * 0.01f);
        else std::snprintf(st, sizeof(st), "height=%s --", mode_name(eff));
        g_status = st;

        if (g_cfg.height_log >= 2) {
            const float want_log = (eff == MODE_ABSOLUTE || eff == MODE_SEATED) ? V_target : -1.0f;
            hlog("HEIGHT mode=%s cfg=%s src=%s floor_off=%.3f resid=%.3f S=%.1f K=%.1f hmd_y=%.3f head_abs=%.3f | "
                 "E trace=%.1f(raw %.1f ok %d floor_z %.1f) blam=%.1f(raw %.1f ok %d unit_zcm %.1f off %.1f%s) "
                 "pawn=%.1f(raw %.1f ok %d root_z %.1f half %.1f%s off %.1f) use=%s E=%.1f | O=%.1f T=%.1f "
                 "V_target=%.1f so_y=%.4f V_pred=%.1f V_meas=%.1f err=%.1f cm | active=%d still=%d own=%d H=%.3f(%d)",
                 mode_name(eff), mode_name(cfg_mode), src_name(g_active_src), g_floor_off, g_resid, S, K, hmd.y, head_abs,
                 g_etr.E, g_etr.raw, (int)g_etr.raw_ok, g_floor_z,
                 g_eblam.E, g_eblam.raw, (int)g_eblam.raw_ok, g_unit_zcm, g_blam_off,
                 g_cfg.height_biped_feet != 0.0f ? " cfg" : " learned",
                 g_epawn.E, g_epawn.raw, (int)g_epawn.raw_ok, g_root_z, g_half, g_half_ok ? " capsule" : " nocapsule", g_pawn_off,
                 e_name(g_e_used), E, g_have_O ? g_O : 0.0f, g_T, want_log, so_now, V_pred, V_meas,
                 (want_log >= 0.0f && V_meas >= 0.0f) ? V_meas - want_log : 0.0f,
                 (int)active, (int)still, (int)own, g_H, (int)g_have_H);
        }
    }
    return own;
}

namespace {

// heightmode takes a word (absolute, seated, eyes) or its number.
static int parse_height_mode(const char* val, double v) {
    while (*val == ' ' || *val == '\t') ++val;
    if (_strnicmp(val, "absolute", 8) == 0) return 0;
    if (_strnicmp(val, "seated", 6) == 0)   return 1;
    if (_strnicmp(val, "eyes", 4) == 0)     return 2;
    return (int)clampf((float)v, 0.0f, 2.0f);
}

}  // namespace

bool heightcal_parse_key(const char* key, const char* val, double v) {
    if (_stricmp(key, "heightmode")       == 0) { g_cfg.height_mode     = parse_height_mode(val, v); return true; }
    if (_stricmp(key, "heightscale")      == 0) { g_cfg.height_scale    = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "heighteye")        == 0) { g_cfg.height_eye      = (int)clampf((float)v, 1.0f, 3.0f); return true; }
    if (_stricmp(key, "heighttracechannel") == 0) { g_cfg.height_trace_channel = (int)clampf((float)v, 0.0f, 32.0f); return true; }
    if (_stricmp(key, "heighttracemax")   == 0) { g_cfg.height_trace_max = clampf((float)v, 50.0f, 2000.0f); return true; }
    if (_stricmp(key, "heightholdms")     == 0) { g_cfg.height_hold_ms  = clampf((float)v, 0.0f, 5000.0f); return true; }
    if (_stricmp(key, "heightestep")      == 0) { g_cfg.height_e_step   = clampf((float)v, 0.1f, 50.0f); return true; }
    if (_stricmp(key, "heightbipedscale") == 0) { g_cfg.height_biped_scale = clampf((float)v, 1.0f, 1000.0f); return true; }
    if (_stricmp(key, "heightbipedfeet")  == 0) { g_cfg.height_biped_feet  = clampf((float)v, -500.0f, 500.0f); return true; }
    if (_stricmp(key, "heightpawnfeet")   == 0) { g_cfg.height_pawn_feet   = clampf((float)v, -500.0f, 500.0f); return true; }
    if (_stricmp(key, "heightseattarget") == 0) { g_cfg.height_seat_target = clampf((float)v, 0.0f, 400.0f); return true; }
    if (_stricmp(key, "heightautoseat")   == 0) { g_cfg.height_auto_seat   = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "heightseatbelow")  == 0) { g_cfg.height_seat_below  = clampf((float)v, 0.0f, 250.0f) * 0.01f; return true; }
    if (_stricmp(key, "heightseatdwell")  == 0) { g_cfg.height_seat_dwell  = clampf((float)v, 0.5f, 60.0f); return true; }
    if (_stricmp(key, "heightband")       == 0) { g_cfg.height_band     = clampf((float)v, 1.0f, 50.0f) * 0.01f; return true; }
    if (_stricmp(key, "heightcal")        == 0) { g_cfg.height_cal      = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "heightkey")        == 0) {
        const int k = (int)strtol(val, nullptr, 0);
        g_cfg.height_key = (k == 0x2D) ? 0 : k;   // Insert opens UEVR's menu: never a height key
        return true;
    }
    if (_stricmp(key, "heightlog")        == 0) { g_cfg.height_log      = (int)clampf((float)v, 0.0f, 100000.0f); return true; }
    if (_stricmp(key, "heightmin")        == 0) { g_cfg.height_min_abs  = clampf((float)v, 0.0f, 250.0f) * 0.01f; return true; }
    if (_stricmp(key, "heightsample")     == 0) { g_cfg.height_sample   = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "heightslew")       == 0) { g_cfg.height_slew     = clampf((float)v, 0.0f, 500.0f) * 0.01f; return true; }
    if (_stricmp(key, "heightsrc")        == 0) { g_cfg.height_src      = (int)clampf((float)v, 0.0f, 3.0f); return true; }
    if (_stricmp(key, "heighttrim")       == 0) { g_cfg.height_trim     = clampf((float)v, -50.0f, 50.0f) * 0.01f; return true; }
    if (_stricmp(key, "heightwindow")     == 0) { g_cfg.height_window_s = clampf((float)v, 1.0f, 60.0f); return true; }
    return false;
}

namespace {

bool heightcal_leash_block_wanted() {
    return g_cfg.height_cal != 0;
}

// Runs where the author's vertical leash is, exactly where the height tick always ran.
bool heightcal_leash_vertical(const Vec3& hp, const UEVR_Vector3f& so, float& ny, bool& moved) {
    // Plugin.cpp's own state, through the bridge: the same objects under the same names.
    const auto& g_in_menu       = *host::g_plugin_state.in_menu;
    const auto& g_cut2d_engaged = *host::g_plugin_state.cut2d_engaged;
    const auto& g_last_dt       = *host::g_plugin_state.last_dt;

            // AUTO HEIGHT owns the origin's Y. While it is on, the vertical leash never acts: it
            // would drag Y back onto the head and break the floor-to-floor mapping.
            float hc_y = 0.0f;
            bool hc_own = false;
            if (g_cfg.height_cal != 0) {
                API::UObject* hc_ignore[2] = {};
                int hc_n = 0;
                if (auto* pawn = API::get()->get_local_pawn(0)) hc_ignore[hc_n++] = pawn;
                if (auto* rigc = reinterpret_cast<API::UObject*>(g_rig_component.load())) {
                    if (auto* wep = rigc->get_outer()) hc_ignore[hc_n++] = wep;
                }
                const bool hc_active = !g_in_menu.load() && !g_cut2d_engaged.load()
                                    && !halo::g_unit_mounted.load(std::memory_order_relaxed);
                hc_own = halo::height_tick(hp, so.y, hc_active, game_window_focused(), g_last_dt.load(),
                                           hc_ignore, hc_n, &hc_y);
            }
            if (hc_own) {
                if (std::fabs(hc_y - ny) > 0.0005f) { ny = hc_y; moved = true; }
                return true;
            }
            return g_cfg.height_cal != 0;
}

bool heightcal_menu_command(const std::string& line) {
            if (line == "calib:height")    { height_request_calibrate(); return true; }
            return false;
}

std::string heightcal_menu_status_line() {
    return height_status_line();
}

}  // namespace

namespace {
bool height_cal_enabled() { CFG_HOOK_READ; return g_cfg.height_cal != 0; }
}  // namespace

constinit const FeatureHooks kHeightCalHooks{
    .key                = "heightcal",
    .parse_key          = &heightcal_parse_key,
    .leash_block_wanted = &heightcal_leash_block_wanted,
    .leash_vertical     = &heightcal_leash_vertical,
    .menu_command       = &heightcal_menu_command,
    .menu_status_line   = &heightcal_menu_status_line,
    .enabled                    = &height_cal_enabled,
    .services                   = SVC_UNIT_STATE | SVC_EYE_TRACE | SVC_LEASH_GATE,
};

}  // namespace halo
