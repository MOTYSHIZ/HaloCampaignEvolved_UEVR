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

// Second status line: the last drive's geometry -- root, wrist target, achieved wrist, and
// how far the solver missed by. For diagnosing a pose that is WRONG rather than absent.
const char* palettearm_status_geom();
const char* palettearm_status_jitter();

// Has this route given up for the session?
//
// True after either terminal failure: the builder could not be resolved in a LOADED simulation
// module (deterministic -- the same scan fails identically every tick), or the hook installed and
// the watchdog then proved it is never called. ArmDriver.cpp reads this and falls back to the UE
// route, because the alternative is far worse than it sounds: with the palette route selected and
// broken, the UE route is also standing down, so the weapon is not controller-attached and the
// arms are stock. That state is strictly worse than never having enabled the palette route, and
// nothing recovers from it on its own.
//
// Latched in a session-local static, deliberately NOT written back to g_cfg: load_config() resets
// the whole struct every ~2 s, so a config field cannot carry a runtime decision -- the file value
// would come straight back and this would become a retry-and-log loop. BlamDrive.cpp records the
// same trap.
bool palettearm_unavailable();

// True when the palette owns the weapon, so the legacy MESH drive must stand down -- otherwise the
// container displaces the gun and the palette displaces it again, and the two compose.
//
// MODE-LEVEL, NOT PER-FRAME, and deliberately so. Keying this off "did we actually write the branch
// this frame" would hand the mesh back on any frame the reach guard rejected, and an ownership flag
// that flickers is what made the wpndrive engage edge stomp the arms fifteen times in one session.
// Ownership is a decision about configuration; success is a decision about a frame.
bool palettearm_weapon_owns();

// Can the SUPPORT-HAND calibration gesture do anything?
//
//   0 = NOT CONFIGURED -- this driver is not posing the support hand at all (a different armdriver,
//       the support arm switched off in paarms, or the route gave up). The feature does not exist
//       in that configuration, and the shipped arm driver is exactly that configuration.
//   1 = configured, but nothing has been posed yet: no weapon, no tracking, or still at the
//       frontend. Nothing to freeze YET.
//   2 = ready.
//
// TWO STATES AND NOT ONE, because the menu must treat them differently: at 0 it hides the entry
// entirely (a player on the shipped configuration should not be shown a control that cannot work),
// while at 1 it shows the entry and explains what is missing. Collapsing them would either
// advertise the feature to everyone or hide it from the people who have it.
//
// Published to the menu through the status file the bridge writes, so the panel never arms a mode
// nothing will ever look at.
int palettearm_support_hand_ready();

// Clear the give-up latch and try again. Called by the arbiter when the armdriver key CHANGES, so
// toggling the key away and back re-arms the route without restarting the game.
void palettearm_retry();

} // namespace halo
