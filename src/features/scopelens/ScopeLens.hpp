#pragma once

// PHYSICAL SCOPE LENS (scopelens, fork feature, Experimental). Implementation in ScopeLens.cpp.

namespace halo {

// =====================================================================================// Game thread, once per tick.
void scope_update(float dt);
// Drop everything (level transition, rig loss). Components belong to the pawn and die with it.
void scope_reset();

// The physical scope key family (scopewpn= entries and the capture knobs), called from
// Config.cpp's parse dispatch.
bool parse_physscope_key(const char* key, const char* val, double v);

} // namespace halo
