#pragma once

// THE HIDDEN RELOAD'S DISPLAY FLAG, shared by the manual reload (which sets it) and the wrist HUD
// (whose weapon cradle panel shows the refill).

#include <atomic>

namespace halo {

// The hidden reload (co-op, and solo): the weapon cradle would show the host's refill before the
// magazine is in, so the cradle's number is held at 0 while this is set.
extern std::atomic<bool> g_wristhud_hide_cradle;

// SVC_HIDDEN_RELOAD went inactive: nothing holds the cradle any more.
void hidden_reload_reset();

} // namespace halo
