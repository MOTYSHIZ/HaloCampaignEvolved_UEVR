#pragma once

// HOOK POINTS IN Gesture.cpp. Definitions: src/features/FeatureList.cpp.

#include "core/host/GestureState.hpp"   // HALO_GESTURE_STATE_BRIDGE

namespace halo {

// gesture_update (game thread), in the melee half: after the melee_swing gate and the dt sanity
// check, before the aim hand's pose check. The off-hand melee detector.
void features_gesture_melee_offhand(float dt);

} // namespace halo
