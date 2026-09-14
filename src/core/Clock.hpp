#pragma once

// A WALL-CLOCK MILLISECOND COUNT (steady_clock), for cadences that must not depend on how often a
// hook happens to be called: the sim-thread radar scan and the grenade tracking both measure time
// this way, because a call count at an unmeasured rate aliases.

#include <chrono>

namespace halo::clock {

inline long long now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

} // namespace halo::clock
