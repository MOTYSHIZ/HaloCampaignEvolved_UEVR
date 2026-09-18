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

// ---- THE RELOAD ENGINE'S BELT MAGAZINE (core/reload), game thread.
// mag_mesh_for_weapon, first: the weapon's own magazine component (writes rank 4), or null.
uevr::API::UObject* features_holster_mag_mesh(int* out_rank);
// holster_update, the belt magazine block: the belt point for the weapon in hand (the author's offset in).
Vec3 features_holster_mag_belt_point(const Vec3& his_offset);
// holster_update, the magazine mesh pick, after the resurvey guard: true = re-arm it (every candidate is dead).
bool features_holster_mag_cands_stale(int rank);
// mag_mesh_for_weapon, after the component path: true = the author's name survey (its "ammo" and "clip"
// ranks, and the frag-grenade fallback at the end of its chain) stands down, so the pick is ours or nothing.
bool features_holster_mag_survey_off();
// mag_mesh_for_weapon's caller, in place of the author's "rank < 3": whether to re-run the name survey.
bool features_holster_mag_resurvey(int rank);
// mag_mesh_for_weapon's caller: true = this pick is final and may be stored under the weapon key.
bool features_holster_mag_pick_final(int rank);
// holster_update, the magazine marker's spawn: the mesh to spawn it with (the author's survey pick and his
// frag stand-in in), or null for no marker at all.
uevr::API::UObject* features_holster_mag_spawn_mesh(uevr::API::UObject* survey, uevr::API::UObject* frag);
// holster_update, the last statement of the magazine marker block: a weapon with no magazine asset draws none.
void features_holster_mag_drawn(uevr::API::UObject* m, bool wanted);
// holster_update, the magazine in hand, in place of the placement: true = placed (in-hand tuning, the slide
// into the well, the render anchor).
bool features_holster_mag_in_hand(uevr::API::UObject* m, const Vec3& gpos, const Vec3& hpos, float pitchr, float yawr, float rollr);

} // namespace halo
