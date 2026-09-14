#include "core/HiddenReload.hpp"

namespace halo {

std::atomic<bool> g_wristhud_hide_cradle{false};

void hidden_reload_reset() { g_wristhud_hide_cradle.store(false, std::memory_order_relaxed); }

} // namespace halo
