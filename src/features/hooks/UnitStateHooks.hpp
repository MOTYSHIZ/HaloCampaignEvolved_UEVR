#pragma once

// HOOK POINTS IN core/UnitState.cpp. Definitions: src/features/FeatureList.cpp.

#include <cstdint>

namespace halo {

// publish_unit_state (the sim thread, inside the orientation getter hook), right after the throw dump
// probe, before the instant-release experiments. obj is the player's resolved unit object.
void features_sim_unit_state_radar(uintptr_t obj);

// publish_unit_state (the sim thread, inside the orientation getter hook), at its end, right after
// the unit facing publish. obj is the player's resolved unit object.
void features_sim_unit_state_end(uintptr_t obj);

} // namespace halo
