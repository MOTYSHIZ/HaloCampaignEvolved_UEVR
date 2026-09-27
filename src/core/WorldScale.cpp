#include "core/WorldScale.hpp"

#include "core/Clock.hpp"
#include "uevr/API.hpp"

#include <atomic>
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

bool uevr_world_scale_known() {
    (void)uevr_world_scale();   // asking first still triggers the first read
    return s_known.load(std::memory_order_relaxed);
}

}  // namespace halo
