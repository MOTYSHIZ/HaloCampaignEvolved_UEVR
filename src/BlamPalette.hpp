// ============================================================================================
// THE FIRST-PERSON BONE PALETTE -- finding it, and eventually owning it.
//
// WHY THIS EXISTS. Everything painful about the VR arms comes from being on the wrong side of the
// Blam/UE boundary. This game's first-person rig -- arms, hands, fingers, and the weapon socketed
// among them -- is posed by the Blam simulation every frame. UE renders the result. So the mod's
// current approach (hide the game's arms, spawn replacements, drive them per tick) is an attempt to
// rebuild in UE something that already exists, correctly and animated, one layer down:
//
//   * the spawned mesh has no anim instance, so it renders the BIND pose -- an open hand
//   * SetLeaderPoseComponent is the only way to pose it, and it defeats HideBoneByName
//   * SetBoneTransformByName does not exist in this build, so per-bone posing is unavailable
//   * the weapon is placed by a separate system, so it and the hands disagree about where the
//     grip is
//
// None of those problems exist if the palette is transformed in place instead. The stock pose is an
// AUTHORED CLOSED GRIP: the fingers already hold the weapon. Moving a branch of the palette moves
// the hand and everything below it while leaving the authored pose intact, so grip, trigger and
// reload animation all survive for free. There is nothing to author and nothing to hide.
//
// SIM THREAD ONLY. The palette is reached through the same gs:[0x58] thread-local block BlamDrive
// resolves the control record from, and that returns zero on every other thread -- see the note in
// BlamDrive.hpp, which cost 1.19 billion zero samples to learn. Anything here that touches memory
// must be called from inside the sim's own call stack.
//
// FOUND, NOT ASSUMED. The scan below searches for the palette by its SHAPE rather than reading a
// hardcoded offset. Two reasons. It is self-validating, so a hit is evidence rather than a hope.
// And an offset is only true for one build of the game: a shape survives a patch, which is the
// property that separates this codebase's reflection-based work from an offset table that dies on
// the next update.
// ============================================================================================

#pragma once

#include <atomic>
#include <cstdint>

namespace halo {

// One-shot discovery pass. SIM THREAD ONLY.
//
// Re-arms whenever `palettescan` returns to 0, so a scan can be repeated with a different weapon in
// hand without restarting the game -- which matters, because a weapon's own nodes are only present
// in the palette while it is held.
void blam_palette_scan();

// Where the scan settled, or 0. Published so later work can use it without re-deriving, and so a
// diagnostic can say whether discovery actually succeeded rather than assuming it did.
uintptr_t blam_palette_address();
int32_t   blam_palette_node_count();
// Sim-thread palette hook timing since the last call (microseconds); resets on read.
void      blam_palette_hook_perf(double* out_mean_us, double* out_max_us, uint32_t* out_n);

// THE PROOF. SIM THREAD ONLY.
//
// The scan finds palettes by shape, and a live session holds several: 76-node first-person rigs for
// more than one weapon slot, plus other skeletons entirely. Shape cannot say WHICH one is the rig
// on the player's screen, and writing to the wrong one is indistinguishable from writing to the
// right one at the wrong moment -- both render nothing.
//
// So displace one node of one candidate by an amount nobody could miss, and look. A rig that jumps
// is the rig being rendered, and it simultaneously proves the write lands somewhere the game reads
// AFTER it has posed the skeleton. Both facts are needed before any of this is worth building on,
// and neither is available by reading memory.
//
// `palettepoke` selects the candidate by the index the scan logged, 0 = off.
void blam_palette_poke();

// WHO WRITES THE PALETTE. SIM THREAD ONLY.
//
// The poke proved we can write these bytes and that the game puts them back within a few calls, so
// the address is right and the moment is wrong. Writing a pose the game is about to recompute is
// pointless; the transform has to run immediately after it builds the rig, which means knowing
// which function builds it.
//
// Rather than scan for a signature, catch it in the act: a hardware write breakpoint on one node,
// and record the RIP of whatever trips it. That is the same instrument AimDirect uses to find the
// ControlRotation writer, including the filter that ignores this plugin's own stores -- without it
// the first writer caught is reliably our own poke.
//
// The handler does the minimum and nothing that can fault: match a RIP, bump a counter, return.
// Module names and RVAs are resolved later, on the hook, where a mistake is survivable.
void blam_palette_watch();

// Axis-probe state, for the game-thread sampler in Plugin.cpp: which phase the probe is in
// (-1 = off; 0/2/4 = baselines; 1/3/5 = node 8 displaced along bone X/Y/Z), and by how much.
// The probe is the empirical answer to "which world direction does each bone axis move the
// rendered weapon" -- the one term of the pullback that was assumed rather than measured.
int   blam_palette_probe_phase();
float blam_palette_probe_amount();

// Sample the head and aim-hand poses once on the GAME thread and publish them for the hook.
//
// The hook runs on the sim thread; calling the VR runtime from there is a risk with no upside when
// the poses only change at tracking rate anyway. Same pattern as the turn quaternion Hands.cpp
// reads instead of rebuilding.
void blam_palette_publish_poses();

// Install/remove the pose hook and keep it in step with config. GAME THREAD, once per frame --
// same contract as blam_drive_tick(), and for the same reason: hook installation is not something
// to do from inside the hot path it is installing onto.
//
// Hooks the first-person BUILD routine, dll+0x46EC10, void(int32 local_player, int32 weapon_slot,
// bool). Not the two low-level matrix writers a hardware write-watch found first: those are what it
// drives, and writing after either corrupted state the other still needed. Its arguments also name
// which palette was just built, which is what removes the candidate guessing.
//
// CAN CRASH THE GAME. It rewrites the entry of a function running at frame rate, so a wrong target
// or a wrong signature takes the game down rather than doing nothing -- both of which happened
// before the arity was corrected from four arguments to three. Config-gated and defaulted off so a
// bad outcome is always recoverable by editing a file.
void blam_palette_hook_tick();

// DIAGNOSTIC ONLY. True while a freeze-and-align key is down (Page Up / Home / Tilde).
//
// Exists so the TRACE line can sample every tick for the duration of a hold instead of every 45.
// A hold lasts 1-2 seconds, which at the 45-tick cadence is TWO OR THREE samples -- too coarse to
// characterise motion the player can plainly see, and a measurement taken at that rate was read as
// "the gun is holding still" when it was not. Changes no behaviour; only who may look.
bool blam_palette_freeze_active();

// TRUE while the game is actively building the local player's first-person palette (a weapon is
// rendered in first person right now). The stick-mode detector consumes this: with the rig driver
// off (the palette-weapon configuration), the rig never resolves, and without this signal the
// detector reads "no weapon" and parks the aim stack in stick mode on foot.
bool blam_palette_fp_live();

} // namespace halo
