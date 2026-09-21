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

    // Identity of the skeleton in `palette`, carried so the capture mirror can prove it is looking
    // at a record for the SAME weapon before writing through it.
    std::int32_t   model_tag{};

    // Which buffer this is. The live slot is what the builder just wrote; the capture banks are
    // what the RENDERER may read instead -- see palettehook_install().
    //
    // A drive that only writes the live slot is invisible on screen. That is not a hypothesis: it
    // was measured on 2026-08-25, with the whole node set displaced half a metre for 2,931
    // consecutive successful writes and no change on screen at all.
    bool           is_capture_bank{};
    std::uint8_t   bank_index{};

    // WHICH BANK THE GAME CALLS CURRENT, from the capture context's first byte -- the one the gate
    // in drive_capture_banks() already range-checks to < 2. The renderer blends the two banks for
    // sub-tick smoothness, so supplying two DIFFERENT endpoints is only meaningful if we know which
    // end is "now"; see the pabankmirror modes in PaletteArm.cpp.
    //
    // NOT YET PROVEN to be a bank index. It is range-checked like one and nothing else in the
    // context looks like one, but the only evidence that it ALTERNATES is the dev census this
    // change adds. Until that census has been read in a live session, treat `live_bank_valid` as
    // "the byte was in range", not as "the meaning is established" -- which is why every mode that
    // depends on it has a sign-flipped twin, and why an invalid one falls back to writing both
    // banks identically rather than guessing.
    bool           live_bank_valid{};
    std::uint8_t   live_bank{};
};

// Called from inside the detour, after the game's own builder has run. Return false to leave the
// stock pose alone. Runs on the GAME'S thread, inside a function on the frame path: no allocation,
// no reflection, no logging beyond a rate-limited line.
using PaletteDriveFn = bool (*)(const PaletteAccess& access);

// The outcome of trying to install. Three states, not a bool, because "not yet" and "never" want
// completely different responses from the caller and a bool cannot tell them apart.
enum class HookInstall {
    // Installed. The detour is live -- though see palettehook_call_count(): still not "running".
    Installed,
    // HaloSimulation_tag_release.dll is not loaded yet. Normal at the main menu and during a load.
    // KEEP CALLING; nothing is wrong.
    WaitingForModule,
    // The module is loaded and the builder could not be found in it, or the hook refused to
    // install. This is DETERMINISTIC -- the same scan over the same image will fail identically
    // every tick, so retrying is pure waste and the caller should give up and fall back.
    Failed,
};

// Resolve the builder and install the detour. Idempotent -- a second call while installed returns
// Installed and does nothing.
//
// Resolution is a cascade, not a constant (the project's addrcascade doctrine):
//   1. signature scan of HaloSimulation_tag_release.dll, which must match EXACTLY ONCE
//   2. the recorded RVA, accepted only if the signature bytes are actually there
//   3. nothing -- log loudly and return Failed
//
// Not installing is a working state, not an error: the arms stay stock and the caller falls back.
HookInstall palettehook_install(PaletteDriveFn drive);

// Remove the detour, if installed.
void palettehook_uninstall();

bool palettehook_installed();

// Has the detour actually been CALLED? "Installed" is not "running" -- register_inline_hook
// succeeds on any readable address, so this is the only thing that distinguishes a correct
// address from a plausible one.
std::uint64_t palettehook_call_count();

// Split of that count by the builder's `capture_render_palette` flag. If one of these is zero the
// game only ever calls it one way and the flag is a red herring; if both are large, WHICH call we
// write on is the whole question.
std::uint64_t palettehook_capture_calls();
std::uint64_t palettehook_nocapture_calls();

// How many capture-bank records have been written. If the hook is running and the drive succeeds
// but this stays ZERO, the mirror is finding no record whose tag and node count match -- a
// different failure from the drive itself failing, and the one that leaves a correct pose
// invisible on screen.
std::uint64_t palettehook_bank_writes();
// Why the capture mirror declined, per reason -- see the counters in PaletteHook.cpp.
void palettehook_capture_census(std::uint64_t* no_tls, std::uint64_t* no_ctx,
                                std::uint64_t* gate, std::uint64_t* mismatch);

// THE BANK CENSUS. How often the capture context named bank 0, how often bank 1, and how often the
// name CHANGED between consecutive calls. This answers the one question the two-endpoint modes rest
// on: if `flips` stays at zero while `bank0`/`bank1` climb, that byte is not a current-bank index
// and pabankmirror 2-5 are writing endpoints into arbitrary buffers. Dev build only.
void palettehook_bank_census(std::uint64_t* bank0, std::uint64_t* bank1, std::uint64_t* flips);

// Per-tick watchdog. `gameplay_active` says whether a first-person weapon could be being built
// right now; without it the alarm counts time in which the call was impossible and condemns a
// healthy hook (see addrcascade's README -- this exact mistake fired 11 seconds before gameplay on
// a real title). Returns true the first time it decides the hook is dead.
bool palettehook_watchdog_tick(bool gameplay_active);

// One-line human-readable account of how the address was resolved, for the log and for support.
const char* palettehook_resolution();

} // namespace halo::palettearm
