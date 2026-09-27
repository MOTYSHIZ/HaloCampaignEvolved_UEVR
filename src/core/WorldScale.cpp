#include "core/WorldScale.hpp"

#include "Config.hpp"
#include "core/Clock.hpp"
#include "uevr/API.hpp"

#include <atomic>
#include <cmath>
#include <cstdlib>

namespace halo {

namespace {

std::atomic<float>     s_scale{1.0f};
std::atomic<bool>      s_known{false};
std::atomic<long long> s_next_ms{0};

void refresh() {
    // The raw C API, not API::VR::get_mod_value<float>(): its float path runs std::stof on a buffer
    // that is empty when the key is absent (XrLayer.cpp found that first).
    auto* p = uevr::API::get()->param();
    if (p == nullptr || p->vr == nullptr || p->vr->get_mod_value == nullptr) return;
    char buf[64]{};
    p->vr->get_mod_value("VR_WorldScale", buf, sizeof(buf));
    if (buf[0] == 0) return;
    const float v = (float)std::atof(buf);
    // Under 0.1 is the mono collapse's 0.01 floor, or garbage: keep the last good value.
    if (v >= 0.1f && v < 100.0f) {
        s_scale.store(v, std::memory_order_relaxed);
        s_known.store(true, std::memory_order_relaxed);
    }
}

}  // namespace

float uevr_world_scale() {
    const long long now = clock::now_ms();
    if (now >= s_next_ms.load(std::memory_order_relaxed)) {
        // Every 2 s once known; every 250 ms until then, so a slow first answer from UEVR is not
        // stuck on the 1.0 fallback for a whole poll.
        const bool known = s_known.load(std::memory_order_relaxed);
        s_next_ms.store(now + (known ? 2000 : 250), std::memory_order_relaxed);
        refresh();
    }
    return s_scale.load(std::memory_order_relaxed);
}

float uevr_world_scale_cached() {
    return s_scale.load(std::memory_order_relaxed);
}

bool uevr_world_scale_known() {
    (void)uevr_world_scale();   // asking first still triggers the first read
    return s_known.load(std::memory_order_relaxed);
}

void world_scale_resolve_config() {
    // Called every config poll, so this is also what keeps the _cached form fresh for the other
    // threads, whether or not any scale-dependent feature is on.
    if (!uevr_world_scale_known()) return;   // keep the compiled default (or the explicit value)
    const float cm = uevr_cm_per_metre();
    static float s_logged = -1.0f;
    if (g_cfg.rig_scale_explicit && g_cfg.world_scale_follow == 0) {
        // An explicit rigscale is a deliberate override: leave it. Say so once per value, because a
        // pinned rig at a different world scale is a mismatch someone should be able to find.
        if (std::fabs(g_cfg.rig_scale - s_logged) > 0.05f) {
            uevr::API::get()->log_info("[Halo-CampE-UEVR] WORLDSCALE: rigscale=%.1f is set explicitly, so the arms do "
                                       "not follow UEVR's world scale (%.1f cm per metre)", g_cfg.rig_scale, cm);
            s_logged = g_cfg.rig_scale;
        }
        return;
    }
    // Logged against the last RESOLVED value, not g_cfg's -- every reload resets g_cfg to the
    // compiled default first, so comparing against that would log every poll.
    if (std::fabs(cm - s_logged) > 0.05f) {
        uevr::API::get()->log_info("[Halo-CampE-UEVR] WORLDSCALE: rig_scale %.1f (100 x UEVR VR_WorldScale %.3f)",
                                   cm, cm * 0.01f);
        s_logged = cm;
    }
    g_cfg.rig_scale = cm;
}

}  // namespace halo
