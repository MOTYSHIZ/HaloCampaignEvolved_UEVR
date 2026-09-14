#pragma once

// HOOK POINTS IN Holster.cpp. Definitions: src/features/FeatureList.cpp.

#include "Math.hpp"                       // Vec3
#include "core/host/HolsterState.hpp"   // HALO_HOLSTER_STATE_BRIDGE

namespace halo {

enum class HolsterSlot : int;   // Holster.hpp

// holster_reset (game thread), right after the body yaw re-init and before markers_hide_all().
void features_holster_reset();

// holster_update (game thread), in the grenade marker spawn gate: the sweep period in ticks (120
// unless a feature backs it off).
unsigned features_holster_mesh_sweep_period();

// holster_update (game thread), right after a grenade mesh sweep picked its stand-ins: mf is the frag
// mesh the markers will use (null = the sweep found none).
void features_holster_mesh_swept(const void* mf);

// holster_update (game thread), right after the weapon hide reconciliation and before the tick's own
// release edge, with the pouch each hand is in and the hand and head poses.
void features_holster_before_release(HolsterSlot zone_g, HolsterSlot zone_p,
                                     const Vec3& pos, const Vec3& gpos, const Vec3& hpos);

} // namespace halo
