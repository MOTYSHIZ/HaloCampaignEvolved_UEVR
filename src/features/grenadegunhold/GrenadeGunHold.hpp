#pragma once

// THE WEAPON HELD STILL THROUGH A GRENADE THROW (grenadegunhold, Experimental).
//
// Under the base mod's arm driver (armdriver 2) the game's own throw animation plays on the first
// person rig and swings the gun off the hand. Measured 2026-09-24, three pouch throws: the authored
// action starts within 50 ms of the throw press and runs 1.54 to 1.60 s, moving the gun 15 to 16 cm
// and 24 deg from rest while the off hand swings about a metre.
//
// This raises his own per-action hold and his off-hand gate, exactly as reloadgunhold's mechanism 1
// does for a reload, for grenadegunholdms from the moment the throw press goes to the game. The gun
// keeps following the hand; only the animation is held at rest. RAISE ONLY: it can never pull one
// of his gates down.
//
// The window is read off the throw press itself (g_holster_throw_until, an atomic the holster
// writes when it throws): the press starts holsterpressms before that deadline. One read per pose
// build, on the sim thread, and the capture banks reuse the live slot's sample.
//
// FEATURE grenadegunhold. Hook slots: parse_key, pa_anim_gates. Table: kGrenadeGunHoldHooks.

#include "features/FeatureHooks.hpp"

namespace halo {

extern const FeatureHooks kGrenadeGunHoldHooks;

bool grenadegunhold_parse_key(const char* key, const char* val, double v);

} // namespace halo
