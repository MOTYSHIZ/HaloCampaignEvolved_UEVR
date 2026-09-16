// HolsterPollThrow -- the grenade leaves the hand the instant the grip opens.
//
// The author's holster throws on the game tick's release edge. This moves the release to the XInput
// hook's poll rate: the tick publishes a standing verdict (armed grenade, carrier hand outside every
// pouch, the aim-hold direction from the swing's peak) and the hook fires the synthetic throw press on
// the carrier grip's falling edge, in the very report that shows the grip open. The tick path stays in
// full: it is the fallback with this off, when the grip mask is wrong for a profile, and it is still
// the sole owner of the put-back. It also reads the live grenade type and pouch counts from the unit
// object, and carries the throw experiments (holsterpollthrowdump, holsterpollthrowhand, holsterpollthrowinstant; the create_projectile
// parts are dev builds only).
//
// FEATURE holsterpollthrow (Experimental). Hook slots: parse_key (holsterpollthrow, holsterpollthrowgripmaskl,
// holsterpollthrowgripmaskr, holsterpollthrowdump, holsterpollthrowhand, holsterpollthrowinstant, holsterpollthrowbackdate, holsterpollthrowspeed), xinput_note_buttons,
// game_tick_after_blam_aim, holster_reset, holster_mesh_sweep_period, holster_mesh_swept,
// holster_before_release, sim_unit_state_grenades, sim_unit_state_after_radar, blam_create_before,
// blam_create_after. Table: kHolsterPollThrowHooks.

#pragma once

#include <atomic>
#include <cstdint>

#include "DevTools.hpp"
#include "features/FeatureHooks.hpp"
#include "uevr/API.hpp"

namespace halo {

extern const FeatureHooks kHolsterPollThrowHooks;

// Called from the XInput hook with the RAW pad buttons, before remapping and before the throw
// press mask is composed. Fires the synthetic throw on the carrier grip's falling edge at poll
// rate -- the tick publishes the verdict, the hook only pulls the trigger. Clocks and atomics
// only; safe on the hook's thread.
void holster_note_buttons(unsigned short buttons);
// The carrier hand's last published position in Blam units (holsterpollthrowhand experiment). Returns false
// until a grenade has been armed once. Safe on any thread.
bool holster_hand_blam(float* x, float* y, float* z);
// The swing's peak direction in Blam units, normalized (holsterpollthrowinstant). Safe on any thread.
bool holster_throw_blam_dir(float* x, float* y, float* z);
// The resolved grenade meshes (wrist-radar blip art). Game thread; null until resolved.
uevr::API::UObject* holster_mesh_frag();
uevr::API::UObject* holster_mesh_plasma();

// GRENTRACK (dev, holsterpollthrowdump): the projectile object the spawn hook just created, and when. The
// hook (sim thread) writes them; throw_dump_probe samples the object's position for ~1.2 s so the
// log shows whether the grenade FLIES from spawn or sits held until an animation event.
extern std::atomic<uintptr_t> g_grentrack_obj;
extern std::atomic<long long> g_grentrack_at_ms;

// GRENINSTANT (dev, holsterpollthrowinstant): the grenade released at spawn, and the velocity to keep
// re-asserting on it for 400 ms so the animation keyframe's own late release is overwritten.
extern std::atomic<uintptr_t> g_greninst_obj;
extern std::atomic<long long> g_greninst_at_ms;
extern std::atomic<float>     g_greninst_vx, g_greninst_vy, g_greninst_vz;

#if HALO_VR_DEV

// The create_projectile hook alone, driven by `holsterpollthrowdump` -- spawn timestamps for the grenade
// windup capture WITHOUT the aim-write ownership change that made blamaim unplayable. See the
// doctrine at its definition.
void blam_spawnlog_tick();

#else

inline void blam_spawnlog_tick() {}

#endif

// The holsterpollthrow keys.
bool holsterpollthrow_parse_key(const char* key, const char* val, double v);

} // namespace halo
