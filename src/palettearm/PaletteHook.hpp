// Finding the native first-person weapon builder, and reaching the palette it writes.
//
// THE IMPURE EDGE. This is the only file in the folder that knows about Windows, UEVR or the game
// process, and it is not covered by Scripts\Verify-PaletteArm.ps1. Everything it produces is
// handed to the pure layer as plain memory.
//
// ⚠️ NOT YET VERIFIED AGAINST A RUNNING GAME. Every offset here was measured by elliotttate against
// his copy of HaloSimulation_tag_release.dll and has never been checked against the build we ship
// for. Until someone has watched it install AND run in a headset, `palettearm` stays default-off
// and the log lines below are the only evidence anyone has. Read the "HARDCODING AN ADDRESS OR
// STRUCT OFFSET" section of the project CLAUDE.md before trusting a single number in the .cpp.
//
// Ported from elliotttate's HaloCampaignEvolved-UEVR (main.cpp @ 62ee34f) with permission.

#pragma once

#include "PaletteMath.hpp"

#include <cstdint>

namespace halo::palettearm {

// What the detour found for one call. `palette` points at LIVE engine memory -- it is valid only
// for the duration of the callback, and writing to it is the whole point.
struct PaletteAccess {
    BlamMatrix4x3* palette{};
    std::uint32_t  node_count{};
    std::int32_t   local_player{};
    std::int32_t   weapon_slot{};
};

// Called from inside the detour, after the game's own builder has run. Return false to leave the
// stock pose alone. Runs on the GAME'S thread, inside a function on the frame path: no allocation,
// no reflection, no logging beyond a rate-limited line.
using PaletteDriveFn = bool (*)(const PaletteAccess& access);

// Resolve the builder and install the detour. Idempotent -- a second call while installed is a
// no-op and says so.
//
// Resolution is a cascade, not a constant (the project's addrcascade doctrine):
//   1. signature scan of HaloSimulation_tag_release.dll, which must match EXACTLY ONCE
//   2. the recorded RVA, accepted only if the signature bytes are actually there
//   3. nothing -- log loudly and stay off
//
// Returns false if the hook did not install. A false return is a working state, not an error: the
// arms simply stay stock.
bool palettehook_install(PaletteDriveFn drive);

// Remove the detour, if installed.
void palettehook_uninstall();

bool palettehook_installed();

// Has the detour actually been CALLED? "Installed" is not "running" -- register_inline_hook
// succeeds on any readable address, so this is the only thing that distinguishes a correct
// address from a plausible one.
std::uint64_t palettehook_call_count();

// Per-tick watchdog. `gameplay_active` says whether a first-person weapon could be being built
// right now; without it the alarm counts time in which the call was impossible and condemns a
// healthy hook (see addrcascade's README -- this exact mistake fired 11 seconds before gameplay on
// a real title). Returns true the first time it decides the hook is dead.
bool palettehook_watchdog_tick(bool gameplay_active);

// One-line human-readable account of how the address was resolved, for the log and for support.
const char* palettehook_resolution();

} // namespace halo::palettearm
