#include "core/XrDisplayTime.hpp"

#include <atomic>

namespace halo {

namespace {
std::atomic<int64_t> s_display_time{0};
}

void xr_display_time_note(int64_t display_time) {
    if (display_time != 0) s_display_time.store(display_time, std::memory_order_relaxed);
}

int64_t xr_display_time() {
    return s_display_time.load(std::memory_order_relaxed);
}

} // namespace halo
