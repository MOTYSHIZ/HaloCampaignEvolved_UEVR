#pragma once

// PHYSICAL SCOPE LENS (scopelens, Experimental). Implementation in ScopeLens.cpp.
//
// FEATURE scopelens. Hook slots: parse_key (scopelens and the physical scope key family),
// game_tick_after_offsets (scope_update), rig_lost (scope_reset), scope_trigger_stood_down and
// scope_pane_stands_down (the pane's toggles stand down while the lens owns the scope).
// Table: kScopeLensHooks.

#include "features/FeatureHooks.hpp"

namespace halo {

extern const FeatureHooks kScopeLensHooks;

// Game thread, once per tick.
void scope_update(float dt);
// Drop everything (level transition, rig loss). Components belong to the pawn and die with it.
void scope_reset();

// The physical scope key family: scopelens, the scopewpn= entries and the capture knobs.
bool parse_physscope_key(const char* key, const char* val, double v);

} // namespace halo
