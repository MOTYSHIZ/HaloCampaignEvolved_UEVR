#pragma once

// HOOK POINTS IN Gesture.cpp. Definitions: src/features/FeatureList.cpp.

#include "core/host/GestureState.hpp"   // HALO_GESTURE_STATE_BRIDGE

namespace halo {

// gesture_update (game thread), in the melee half: after the melee_swing gate and the dt sanity
// check, before the aim hand's pose check. The off-hand melee detector.
void features_gesture_melee_offhand(float dt);

// gesture_update (game thread), near its top, after the reload family's ticks and before the global
// stand-down: the deferred melee hold check (core/fixes/MeleeInstruments).
void features_melee_hold_check();

// gesture_update (game thread), in the aim hand's swing segmentation, the statement right before
// `s_in_swing = true;`, with the value s_in_swing has then: the swing-start aim capture (core).
void features_melee_swing_moving(bool in_swing);

// gesture_update (game thread), right after the aim hand's trigger gates (the legacy forward gate
// last). True = the holster veto stood the strike down and armed the short cooldown; the tick
// returns (core).
bool features_melee_vetoed(long long now, float speed, float reach);

// gesture_update (game thread), right after a fired strike arms the cooldown: the aim pin, the hold
// check schedule and the FIRED log line (core). True = the line is logged there, so the tick returns
// before the author's own line.
bool features_melee_fired(long long now, float speed, float reach);

} // namespace halo
