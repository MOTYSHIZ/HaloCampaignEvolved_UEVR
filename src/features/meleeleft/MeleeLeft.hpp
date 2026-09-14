// MeleeLeft -- melee with the off hand.
//
// A punch does not care which hand throws it. The off hand runs the aim hand's three tests with the
// same thresholds and its own state, and SHARES the aim hand's cooldown so the two detectors cannot
// double-fire one press. The off hand's everyday reaches (a magazine fetch, a grenade pull, a brace)
// each get an explicit, logged stand-down, and a gunstock kick needs real travel to count.
//
// FEATURE meleeleft (Experimental). Hook slots: parse_key (meleeleft, meleeshotms, meleeshotdist,
// meleedisp) and gesture_melee_offhand (the detector, in the melee half of the gesture tick, before
// the aim hand's detector). Table: kMeleeLeftHooks.

#pragma once

#include "features/FeatureHooks.hpp"

namespace halo {

extern const FeatureHooks kMeleeLeftHooks;

// Game thread: the off-hand detector, once per gesture tick.
void offhand_melee_update(float dt);

// The OFF-hand detector's holster veto: the off hand's own pouch proximity + the recent-action
// window. The aim-hand version (holster_melee_veto) reads the AIM hand's proximity and must not gate
// left punches.
bool holster_offhand_melee_veto();

// The meleeleft keys.
bool meleeleft_parse_key(const char* key, const char* val, double v);

} // namespace halo
