#pragma once

// THE ARMS FOLLOW UEVR's WORLD SCALE (worldscalefollow, Experimental) -- NOW THE DEFAULT.
//
// The base mod turns real metres into game centimetres with rig_scale, which was fixed at 131.2
// because its profile ships VR_WorldScale=1.312. A player who changed UEVR's world scale saw the world
// drawn at the new size while the arms and the weapon carry kept 131.2. Measured 2026-09-24 at
// VR_WorldScale 1.132 under armdriver 2: the drawn gun sat 1.160 times the hand's distance from the
// eye (131.2 / 113.2 = 1.159), 2.7 cm off the hand at a normal hold and 11.5 cm at full reach. On a
// run where the two matched (131.2 both) the same fit read 0.970.
//
// This feature was the fix, behind a switch that was off by default. Since 2026-09-27 following is
// the DEFAULT for everyone: core/WorldScale's world_scale_resolve_config() sets rig_scale to
// 100 x VR_WorldScale as the last write of every config reload, unless rigscale= is set explicitly.
// What this switch still does is narrower: ON, the world scale wins even over an explicit rigscale.
// Resolving moved into the reload itself because this file used to set rig_scale AFTER it, leaving a
// window each poll in which a sim-thread reader could see the compiled default.
//
// His paworldscale, when set, still wins for the palette arm driver, exactly as he wrote it.
//
// FEATURE worldscalefollow. Hook slots: parse_key. Table: kWorldScaleFollowHooks.

#include "features/FeatureHooks.hpp"

namespace halo {

extern const FeatureHooks kWorldScaleFollowHooks;

bool worldscalefollow_parse_key(const char* key, const char* val, double v);

} // namespace halo
