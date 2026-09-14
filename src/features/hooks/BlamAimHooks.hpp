#pragma once

// HOOK POINTS IN BlamAim.cpp (dev builds: BlamAim.cpp compiles only with HALO_VR_DEV).
// Definitions: src/features/FeatureList.cpp.

#include "core/host/BlamAimState.hpp"   // HALO_BLAMAIM_STATE_BRIDGE

#include <cstdint>

namespace halo {

// hooked_create_projectile (the sim thread), right before the original constructor is called.
void features_blam_create_before(uintptr_t params);

// hooked_create_projectile (the sim thread), wrapping the original constructor's return value: runs
// right after it, returns it unchanged.
uintptr_t features_blam_create_after(uintptr_t params, uintptr_t cret);

} // namespace halo
