#pragma once

// THE ARMS FOLLOW UEVR's WORLD SCALE (worldscalefollow, Experimental).
//
// The base mod turns real metres into game centimetres with rig_scale, fixed at 131.2 because its
// profile ships VR_WorldScale=1.312. A player who changes UEVR's world scale sees the world drawn at
// the new size while the arms and the weapon carry keep 131.2. Measured 2026-09-24 at VR_WorldScale
// 1.132 under armdriver 2: the drawn gun sat 1.160 times the hand's distance from the eye
// (131.2 / 113.2 = 1.159), 2.7 cm off the hand at a normal hold and 11.5 cm at full reach. On a run
// where the two matched (131.2 both) the same fit read 0.970.
//
// This keeps rig_scale at 100 x VR_WorldScale. UEVR's value is read on every config poll, so a change
// in UEVR's menu lands within one poll, and the value is put back right after every config reload,
// which resets rig_scale to its cfg value. A world scale under 0.1 is not taken: the mono collapse
// drops VR_WorldScale to 0.01 for the scene and restores it. His paworldscale, when set, still wins
// for the palette arm driver, exactly as he wrote it.
//
// FEATURE worldscalefollow. Hook slots: parse_key. The poll is called from features_config_loaded.
// Table: kWorldScaleFollowHooks.

#include "features/FeatureHooks.hpp"

namespace halo {

extern const FeatureHooks kWorldScaleFollowHooks;

bool worldscalefollow_parse_key(const char* key, const char* val, double v);

// Game thread, right after the config poll's load_config.
void worldscalefollow_poll();

} // namespace halo
