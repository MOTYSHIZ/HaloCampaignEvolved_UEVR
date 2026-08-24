// The palette arm driver, as the rest of the plugin sees it.
//
// ⚠️ UNVERIFIED AGAINST A RUNNING GAME. Nothing in this module has been watched working: the offset
// chain in PaletteHook.cpp was measured on someone else's copy of the simulation DLL, and the
// frame conversion below has never been checked against a real palette. It is default-OFF
// (armdriver=0) and it exists to be A/B'd against the UE route in a headset, not to be shipped.
//
// EXCLUSIVITY. Never call these directly from a tick. ArmDriver.hpp arbitrates; this module only
// runs when arm_driver_owns(ArmDriverMode::Palette) is true, because the UE route drives the same
// visible arms and two drivers must never both have them.

#pragma once

namespace halo {

// Game thread, once per tick. Installs or removes the hook to match the mode, advances the
// two-handed hold, and republishes tuning from config. The actual palette work happens later, on
// the game's own thread, inside the detour.
void palettearm_update(float delta_seconds);

// Remove the hook and drop all state. Safe to call when nothing is installed.
void palettearm_release();

// Consume the palettearm config keys. Called from Config.cpp's parse chain; returns true when the
// key belonged to this module.
//
// Hoisted out of the main chain for the same reason parse_scope_key() was: MSVC counts every
// `else if` toward its 128-deep block limit and the chain is already near it (fatal C1061).
bool palettearm_parse_key(const char* key, double value);

// One line of status for the support diagnostics: resolution, install state, call count.
const char* palettearm_status();

} // namespace halo
