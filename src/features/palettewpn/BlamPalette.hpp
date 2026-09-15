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
#include <cstddef>
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
// WPNERR (Config.hpp wpn_err_log): once per rendered frame, from the stereo callback -- measures
// the controller's fresh pose against what the palette chain is drawing from.
void blam_palette_wpnerr_frame();
// Per-frame pose republish (Config.hpp pal_pub_frame): the stereo callback re-runs the publish
// math with fresh poses so the sim's next build draws from a pose newer than the last tick.
void blam_palette_republish_frame();
// PALRENDER (Config.hpp pal_render): replay the weapon branch + arms onto the render banks from
// their stock snapshots with a fresh camera, once per rendered frame (the stereo callback).
void blam_palette_render_refresh();
// STOMPLOG points 10/11: what is ACTUALLY in each render bank when the frame starts -- node 8's
// position read back and compared against the hook's last write, so a frame holding the game's
// stock pose (or anyone else's write) shows as centimetres of distance instead of a guess.
void blam_palette_stamp_bank();
void blam_palette_readback_probe();
// SOCKROT: the render side reads the DRAWN weapon socket's rotation and hands it here, because
// the published hand it must be compared against lives in an anonymous namespace inside
// BlamPalette.cpp and cannot be declared at namespace scope without making every existing use
// ambiguous (C2872, which is exactly what a header extern for it produced).
void blam_palette_sockrot_probe(float sx, float sy, float sz, float sw);
// PALETTESYNC: called from the aim-law callback with the controller aim + grip pose and the
// ControlRotation read in that same callback, so placement can use the exact pair the aim used.
void blam_palette_sync_capture(float aqx, float aqy, float aqz, float aqw,
                               float apx, float apy, float apz,
                               float gqx, float gqy, float gqz, float gqw,
                               float gpx, float gpy, float gpz,
                               float ctl_pitch, float ctl_yaw);
// SAMEINST: the socket AND the rig component's world rotation read back to back in the stereo
// callback, plus ControlRotation from the same moment, so the post-blend residual can be measured
// with no cross-thread timing in it and correlated against the AIM versus the HAND.
void blam_palette_sameinst_probe(float sx, float sy, float sz, float sw,
                                 float cx, float cy, float cz, float cw,
                                 float aim_pitch, float aim_yaw, bool aim_ok);

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

// Hold the current first-person pose (all 76 nodes, raw) for `ms`, masking whatever animation
// the game plays underneath -- the gesture reload calls this at the seat so the arms do not act
// out a second reload. The weapon still rides the hand. 0 clears. Game thread.
void blam_palette_hold_pose(int ms);

// THREAD + PHASE INSTRUMENT (slidelog/slidewatch): the game thread's id, and 1 while the engine
// tick is in flight (set after on_pre_engine_tick's work, cleared in on_post_engine_tick).
// Published by Plugin.cpp; the sim hook reads them to say which thread it runs on and whether
// it fires inside the engine tick -- the two facts that place a slide write between the sim's
// rebuild of the node and the mesh sync's read of it.
extern std::atomic<uint32_t> g_game_tid;
extern std::atomic<int>      g_engine_phase;

// AIMBORE: the rotations the drawn weapon carries on top of the aim-fixed hand, UE convention
// (x, y, z, w) and in the order applied, the global grip rotation then the held weapon's trim, as
// last published. False when the palette weapon is off (the aim then uses the plain hand forward).
bool palette_trim_rotations(float grip_q[4], float weapon_q[4]);

// ---- OWNERSHIP (armdriver mode 3). GAME THREAD.
// Install or remove the builder pose hook so it matches palette_weapon_mode(). Called every tick by
// the arm driver arbiter while mode 3 owns, and from blam_palette_hook_tick. Refuses while the
// palettearm route's hook holds the same function. Returns the installed hook id, or -1.
int  blam_palette_pose_hook_sync();
// Remove every hook this stack owns on the builder path (pose hook, consumption slerp, bank blend,
// FP anim kill) and drop the published pose, holds and freezes. Writes the removed ids to `out`
// (may be null). The slide capture hook is NOT touched: it belongs to slidevr, not to placement.
void blam_palette_release(const char* why, char* out, size_t cap);
// palettewpn switched off (game thread): the dev discovery instruments too -- watchpoints and their
// exception handler, the palettefinal and fpanimkill hooks with the pose hook, the palsniff thread,
// and the termlog rows.
void blam_palette_instruments_release();
// True once the pose hook has refused this session (prologue mismatch or a failed install); the
// arbiter then falls back to UeRig. blam_palette_retry() re-arms it on an armdriver key change.
bool blam_palette_unavailable();
void blam_palette_retry();

} // namespace halo
