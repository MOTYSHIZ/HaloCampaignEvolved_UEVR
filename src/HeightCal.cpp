#include "HeightCal.hpp"

#include "Config.hpp"
#include "XrLayerBridge.hpp"
#include "thirdparty/openvr.h"
#include "uevr/API.hpp"

#include <Windows.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <deque>
#include <vector>

using namespace uevr;

namespace halo {

std::atomic<int64_t> g_xr_last_display_time{0};

namespace {

enum Src : int { SRC_AUTO = 0, SRC_OPENXR = 1, SRC_OPENVR = 2, SRC_UEVR = 3 };

const char* src_name(int s) {
    switch (s) {
    case SRC_OPENXR: return "OpenXR-stage";
    case SRC_OPENVR: return "OpenVR-standing";
    case SRC_UEVR:   return "UEVR-pose";
    default:         return "auto";
    }
}

void hlog(const char* fmt, ...) {
    if (g_cfg.height_log <= 0) return;
    char buf[768];
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
// kept as a cross-check: head-in-STAGE minus (UEVR hmd.y + offset) should sit near zero, and a
// large residual says the offset is not what this code believes it is.
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

    const int64_t t = g_xr_last_display_time.load(std::memory_order_relaxed);
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

// A floor source that has neither delivered nor refused yet. Calibration waits for it rather than
// finishing without the seated-start check it would have provided.
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
            // The floor source stopped answering. A calibrated H stays valid (it is in UEVR pose
            // space); only the floor-relative checks lose their reference.
            g_active_src = SRC_UEVR;
            // The OpenVR standing universe is floor-relative by construction (UEVR's OpenVR runtime
            // tracks in TrackingUniverseStanding), so the UEVR pose alone still knows the floor there.
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

// ---- sampling --------------------------------------------------------------------------------
struct Sample { float t; float y; };

std::vector<float>  g_win;
float               g_win_t = 0.0f;
std::deque<Sample>  g_ring;
float               g_clock = 0.0f;
bool                g_have_H = false;
float               g_H = 0.0f;
Vec3                g_prev{};
bool                g_have_prev = false;
float               g_vy = 0.0f, g_vl = 0.0f;
float               g_low_persist = 0.0f;
float               g_cont_timer = 0.0f;
bool                g_key_prev = false;
float               g_key_t = -1.0f;
std::vector<float>  g_key_s;
bool                g_have_out = false;
float               g_out = 0.0f;
int                 g_last_mode = -1;
bool                g_said_wait = false;
std::chrono::steady_clock::time_point g_last_call{};

float percentile(std::vector<float> v, float q) {
    if (v.empty()) return 0.0f;
    std::sort(v.begin(), v.end());
    size_t i = (size_t)std::floor(q * (float)(v.size() - 1) + 0.5f);
    if (i >= v.size()) i = v.size() - 1;
    return v[i];
}

void accept(float cand, const char* how, size_t n, float secs) {
    const float prev = g_H;
    const bool had = g_have_H;
    g_H = cand;
    g_have_H = true;
    if (g_floor_known) {
        hlog("HEIGHT CAL (%s): source=%s standing eye height %.3f m above floor (UEVR y %.3f, floor "
             "offset %.3f), %zu samples over %.1f s%s", how, src_name(g_active_src), cand + g_floor_off,
             cand, g_floor_off, n, secs, had ? "" : " -- first calibration");
    } else {
        hlog("HEIGHT CAL (%s): source=%s standing head at UEVR y %.3f (floor unknown), %zu samples "
             "over %.1f s%s", how, src_name(g_active_src), cand, n, secs, had ? "" : " -- first calibration");
    }
    if (had) hlog("HEIGHT CAL: changed by %+.3f m", cand - prev);
}

}  // namespace

bool height_tick(const Vec3& hmd, float so_y, bool gameplay, bool key_focus, float dt, float* out_y) {
    const auto now = std::chrono::steady_clock::now();
    if (g_last_call.time_since_epoch().count() != 0 &&
        std::chrono::duration<float>(now - g_last_call).count() > 1.0f) {
        g_have_out = false;   // re-enabled after a gap: slew from where the origin is now
        g_have_prev = false;
    }
    g_last_call = now;
    if (!(dt > 0.0f) || dt > 0.5f) dt = 0.0f;
    g_clock += dt;

    if (g_cfg.height_sample != g_last_mode) {
        if (g_last_mode != -1) hlog("HEIGHT: heightsample %d -> %d, window restarted", g_last_mode, g_cfg.height_sample);
        g_last_mode = g_cfg.height_sample;
        g_win.clear(); g_win_t = 0.0f; g_ring.clear(); g_low_persist = 0.0f; g_cont_timer = 0.0f;
    }

    update_source(hmd.y, dt);

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

    // RECALIBRATE KEY (any mode): one second of head samples, median. An explicit press is the
    // player's decision, so it is not refused by the seated-start check.
    const bool key_down = key_focus && g_cfg.height_key != 0 &&
                          (GetAsyncKeyState(g_cfg.height_key) & 0x8000) != 0;
    if (key_down && !g_key_prev) {
        g_key_t = 0.0f;
        g_key_s.clear();
        hlog("HEIGHT: key pressed -- stand straight and still for 1 s");
    }
    g_key_prev = key_down;
    if (g_key_t >= 0.0f) {
        g_key_s.push_back(hmd.y);
        g_key_t += dt;
        if (g_key_t >= 1.0f && g_key_s.size() >= 5) {
            accept(percentile(g_key_s, 0.5f), "key", g_key_s.size(), g_key_t);
            g_key_t = -1.0f;
            g_win.clear(); g_win_t = 0.0f; g_ring.clear(); g_low_persist = 0.0f;
        }
    }

    const int  mode = g_cfg.height_sample;
    const bool sampling = gameplay && still && g_key_t < 0.0f;

    // WINDOW (modes 0 and 1 until the first calibration): heightwindow seconds of STILL gameplay
    // samples, then the 90th percentile. P90 rather than the maximum so a moment on tiptoe or a
    // tracking spike does not set it; rather than the median so glancing down does not lower it.
    if (mode != 2 && !g_have_H) {
        if (sampling) { g_win.push_back(hmd.y); g_win_t += dt; }
        if (g_win.size() > 4096) {
            g_win.erase(g_win.begin(), g_win.begin() + 2048);
            g_win_t *= 0.5f;
        }
        if (g_win_t >= g_cfg.height_window_s && g_win.size() >= 10) {
            if (pending) {
                if (!g_said_wait) { g_said_wait = true; hlog("HEIGHT: window full, waiting for the floor source to answer or refuse"); }
            } else {
                const float cand = percentile(g_win, 0.9f);
                if (g_floor_known && cand + g_floor_off < g_cfg.height_min_abs) {
                    hlog("HEIGHT: NOT calibrating -- head only %.3f m above the floor (< heightmin %.0f cm): "
                         "seated or kneeling. Window restarted; stand up, or press the height key.",
                         cand + g_floor_off, g_cfg.height_min_abs * 100.0f);
                } else {
                    accept(cand, "window", g_win.size(), g_win_t);
                }
                g_win.clear();
                g_win_t = 0.0f;
            }
        }
    }

    // CONTINUOUS (mode 1 after the first calibration): track the standing height without chasing
    // crouches. Once a second, the P90 of the last window of still samples:
    //   above H by >1 cm       -> rise toward it at 5 cm/s (standing up straighter, headset reseated);
    //   below H by <= heightband -> only after 30 s continuously there, then 1 cm/s (posture drift);
    //   below H by more          -> a crouch or a seat, ignored.
    if (mode == 1 && g_have_H) {
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
                    const bool seated = g_floor_known && cand + g_floor_off < g_cfg.height_min_abs;
                    if (!seated) {
                        g_H += (std::min)(cand - g_H, 0.05f * step);
                        if (g_cfg.height_log > 1) hlog("HEIGHT: continuous rise -> %.3f (window P90 %.3f)", g_H, cand);
                    }
                    g_low_persist = 0.0f;
                } else if (cand < g_H - 0.01f && cand >= g_H - g_cfg.height_band) {
                    g_low_persist += step;
                    if (g_low_persist >= 30.0f) {
                        g_H -= (std::min)(g_H - cand, 0.01f * step);
                        if (g_cfg.height_log > 1) hlog("HEIGHT: continuous settle -> %.3f (window P90 %.3f)", g_H, cand);
                    }
                } else {
                    g_low_persist = 0.0f;
                }
            }
        }
    }

    if (g_cfg.height_log > 1) {
        static uint32_t s_n = 0;
        if ((s_n++ % (uint32_t)g_cfg.height_log) == 0u) {
            hlog("HEIGHT src=%s floor=%d off=%.3f resid=%.3f hmd_y=%.3f head_abs=%.3f H=%.3f(%d) H_abs=%.3f "
                 "trim=%.3f so_y=%.3f out=%.3f crouch=%.3f still=%d vy=%.2f vl=%.2f win=%.1f/%.1fs n=%zu "
                 "mode=%d pending=%d gameplay=%d",
                 src_name(g_active_src), (int)g_floor_known, g_floor_off, g_resid, hmd.y,
                 g_floor_known ? hmd.y + g_floor_off : -1.0f, g_H, (int)g_have_H,
                 (g_floor_known && g_have_H) ? g_H + g_floor_off : -1.0f, g_cfg.height_trim, so_y,
                 g_have_out ? g_out : so_y, g_have_H ? g_H - hmd.y : 0.0f, (int)still, g_vy, g_vl,
                 g_win_t, g_cfg.height_window_s, g_win.size(), mode, (int)pending, (int)gameplay);
        }
    }

    if (!g_have_H) return false;

    // The standing origin's Y. heighttrim raises the view: a lower origin Y is a larger hmd - origin.
    const float target = g_H - g_cfg.height_trim;
    if (!g_have_out) { g_out = so_y; g_have_out = true; }
    const float step = g_cfg.height_slew * dt;
    const float d = target - g_out;
    if (g_cfg.height_slew > 0.0f && std::fabs(d) > step) g_out += (d > 0.0f) ? step : -step;
    else g_out = target;
    *out_y = g_out;
    return true;
}

}  // namespace halo
