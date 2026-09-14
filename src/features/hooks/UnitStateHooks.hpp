#pragma once

// HOOK POINTS IN core/UnitState.cpp. Definitions: src/features/FeatureList.cpp.

#include <cstdint>

namespace halo {

// publish_unit_state (the sim thread, inside the orientation getter hook), right after the UNITSTATE
// evidence line, before the radar scan. obj is the player's resolved unit object.
void features_sim_unit_state_grenades(uintptr_t obj);

// publish_unit_state (the sim thread), right after the grenade slot, before the instant-release
// experiments. obj is the player's resolved unit object.
void features_sim_unit_state_radar(uintptr_t obj);

// publish_unit_state (the sim thread), right after the radar scan, before the seat publish.
void features_sim_unit_state_after_radar(uintptr_t obj);

// publish_unit_state (the sim thread), at its end, right after the unit facing publish. obj is the
// player's resolved unit object.
void features_sim_unit_state_end(uintptr_t obj);

} // namespace halo
