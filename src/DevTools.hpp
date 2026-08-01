// Compile-time gate for development-only tooling, in the spirit of Unreal's WITH_EDITOR.
//
// WHY THIS EXISTS
//   The diagnostic sweeps in this plugin (material hunt, texture-parameter hunt, material dump,
//   render-target pixel probe) each walk the ENTIRE UObject array -- roughly 296,000 objects on
//   this game. They are invaluable while investigating and ruinous if they ever run in a player's
//   session: a full array scan per tick has already collapsed framerate here once, and a later
//   microstutter fix had to remove two more of them.
//
//   Config flags alone are not a strong enough guarantee. A default can be edited, a shipped
//   profile can be stale, a key can be typo'd into the wrong state, and the code is still THERE,
//   one bad value away from running inside a headset. This makes the guarantee structural: in a
//   release build the code is not compiled in at all, so no configuration can reach it.
//
// USAGE
//   #if HALO_VR_DEV
//       ... diagnostic-only code ...
//   #endif
//
//   Prefer wrapping the FUNCTION BODY and leaving a no-op stub (see HALO_VR_DEV_STUB below) so
//   call sites stay readable and the compiler removes the empty call.
//
// HOW IT IS SET
//   Release / contributor / CI build (scripts\build.ps1)  -> undefined, defaults to 0 here.
//   Internal dev loop (Scripts\Build-AimDriver.ps1)       -> /DHALO_VR_DEV=1
//
//   So the DEFAULT IS SAFE: an unflagged build is a shipping build. Someone has to opt in.
//
// WHAT BELONGS BEHIND IT
//   Anything that exists to answer a question rather than to play the game: full-array sweeps,
//   bulk reflection dumps, pixel readbacks, verbose per-object logging. Things a player benefits
//   from -- the kill switch, live tuning, the per-site perf timer -- stay in the shipping build.

#pragma once

#ifndef HALO_VR_DEV
#define HALO_VR_DEV 0
#endif

// Marks a function whose body is dev-only. Keeps one definition in both configurations so call
// sites need no #if of their own.
#if HALO_VR_DEV
#define HALO_VR_DEV_ONLY(code) code
#else
#define HALO_VR_DEV_ONLY(code) ((void)0)
#endif
