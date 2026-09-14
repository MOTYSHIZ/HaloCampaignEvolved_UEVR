#pragma once

// HOOK POINTS IN Holster.cpp. Definitions: src/features/FeatureList.cpp.

#include "Math.hpp"                       // Vec3
#include "uevr/API.hpp"
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

// holster_update (game thread), in the pouch loop, after the off hand's grab test: the off hand's
// distance to this pouch, whether or not it may grab.
void features_holster_pouch_offhand(bool ghand_ok, const Vec3& ghand, const Vec3& pouch);
// holster_update (game thread), right after the aim hand's proximity is published.
void features_holster_pouches_measured();
// holster_update (game thread), a pouch or hand marker just spawned, before it is kept.
void features_holster_marker_spawned(uevr::API::UObject* marker);
// holster_update (game thread), the tick's release edge: true = the release is too slow to throw.
bool features_holster_throw_too_slow(float peak_speed);
// holster_update (game thread), the release log's put-back text.
const char* features_holster_putback_text(const char* his_text, bool in_pouch);

} // namespace halo
