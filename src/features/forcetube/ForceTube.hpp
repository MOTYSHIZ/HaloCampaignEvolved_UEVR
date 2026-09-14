// ForceTubeVR haptic gunstock: a recoil kick on every shot the player fires.
//
// The vendor ships a plain C API (ForceTubeVR_API_x64.dll, the same DLL their UE plugin merely
// LoadLibrary's): InitRifle / KickChannel / RumbleChannel / GetBatteryLevel. This module loads it
// from the plugins directory beside halo_vr.dll, connects once, and kicks per shot.
//
// PER-SHOT DETECTION is the projectile spawn (create_projectile, the same prologue-gated hook
// address the dev tooling uses): it fires once per round at the weapon's true cadence, automatics
// included, and grenade throws ride the same path for free. Player shots are separated from NPC
// shots by TWO tests, because the muzzle-origin test alone is not enough: its radius is in Blam
// units (1 u ~ 3.05 m), so the original 1.5 accepted every NPC muzzle within four and a half
// metres and kicked all through a firefight. Now the origin must be inside force_tube_radius AND
// the player's own trigger must have been down within force_tube_fire_ms.
//
// THREADING: the hook runs on the sim thread and only increments an atomic. The vendor DLL is
// called from the GAME tick exclusively -- their own UE plugin calls it from there, so that is
// the proven-safe thread.

//
// The per-shot kick is gated on the player's own fire input (core/FireInput.hpp).
//
// FEATURE forcetube (Experimental). Hook slots: parse_key (the forcetube* keys) and game_tick_late
// (the tick below). Table: kForceTubeHooks.

#pragma once

#include "features/FeatureHooks.hpp"

namespace halo {

extern const FeatureHooks kForceTubeHooks;

// Game tick: load + init once (cfg force_tube), install the spawn hook, drain pending shots into
// kicks. Safe to call every tick; does nothing while the feature is off.
void forcetube_tick();

// The ForceTube keys (forcetube*).
bool forcetube_parse_key(const char* key, const char* val, double v);

} // namespace halo
