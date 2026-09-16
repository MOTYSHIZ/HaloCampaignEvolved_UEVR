#pragma once

// ROOMSCALE (roomscale, Experimental): walk around the play space and the game's own movement
// moves the Spartan. The head offset from the standing origin is walked out through the game's
// movement -- the left stick, or the unit object's own throttle vectors (roomscalethrottle 3) --
// and the origin is credited only with the travel roomscale demonstrably caused. Also the camera
// bob measurement (bobcancel / boblog), published through core/CameraBob.hpp.
//
// FEATURE roomscale. Hook slots: parse_key (roomscale*, roomscalethrottleoff*, roomscalethrottleysign,
// bob*), game_tick_before_leash (camera bob and the throttle-frame probe log), leash_block_wanted,
// leash_lateral (the walk, and the leash at roomscale's radius), xinput_before_brake (the stick
// injection) and sim_unit_state_end (throttle mode 3). Table: kRoomscaleHooks.

#include "features/FeatureHooks.hpp"

namespace halo {

extern const FeatureHooks kRoomscaleHooks;

// The roomscale key family.
bool roomscale_parse_key(const char* key, const char* val, double v);

} // namespace halo
