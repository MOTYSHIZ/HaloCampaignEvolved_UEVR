#pragma once

// GRENADES FROM THE POUCHES ONLY (grenadeswallow, Experimental).
//
// The left hand's A-face button stops throwing grenades: UEVR's action state says whether that
// button is down, and the pad code it produced (grenadecode) is stripped from the raw pad so the
// holster gesture is the only path to a throw. Not in menus, where that button navigates.
//
// FEATURE grenadeswallow. Hook slots: parse_key (grenadeswallow, grenadecode) and xinput_raw_pad (the
// strip). Table: kGrenadeSwallowHooks.

#include "features/FeatureHooks.hpp"

namespace halo {

extern const FeatureHooks kGrenadeSwallowHooks;

// The grenadeswallow keys.
bool grenadeswallow_parse_key(const char* key, const char* val, double v);

} // namespace halo
