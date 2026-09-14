#include "core/ReticuleDepth.hpp"

#include <atomic>
#include <chrono>

namespace halo {

namespace {
// RETSTAMP: the latest on-foot trace depth (cm) and when it was taken, for the render publish.
std::atomic<float>     g_ret_last_d{0.0f};
std::atomic<long long> g_ret_last_d_ms{0};
} // namespace

void reticule_depth_note(float d) {
    g_ret_last_d.store(d, std::memory_order_relaxed);
    g_ret_last_d_ms.store((long long)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count(), std::memory_order_relaxed);
}
float reticule_depth() { return g_ret_last_d.load(std::memory_order_relaxed); }
long long reticule_depth_ms() { return g_ret_last_d_ms.load(std::memory_order_relaxed); }

} // namespace halo
