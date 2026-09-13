// ============================================================================================
// THE AIM WRITE -- the shipping half of the Blam work. This file is NOT dev-gated.
//
// BlamAim.cpp next door is a dev-only investigation: object-table sweeps, globals scans, spawn
// redirects, half a dozen refuted hypotheses kept for their evidence. This file is what that
// investigation FOUND, and nothing else -- one hook, one resolve, one write.
//
// WHAT IT DOES
//   Drives the sim's ANGULAR CONTROL STATE, the (yaw,pitch) pair the Blam simulation keeps per
//   player at TLS block +0xB8, stride 0x198, yaw at +0x94 wrapped into [0,2pi), pitch at +0x98.
//
//   That location is the one worth having, and the reason is its position in the chain:
//     * AFTER input conditioning. Acceleration, deadzone and magnetism are all applied upstream,
//       so a write here is 1:1 with the hand -- no curve to fight, no filter to undo. Driving the
//       stick instead means fighting every one of those, which is what made hand-aim feel either
//       jittery when still or sluggish across a large sweep.
//     * BEFORE the aim vector is derived, so every downstream mirror recomputes from it.
//     * PERSISTENT. Verified live: writing yaw=1.50 pitch=0.30 snapped the aim to
//       (0.0676,-0.9529,0.2955) -- cos/sin of those angles to four decimals -- and it HELD after
//       the writes stopped. Every other candidate (+0x1D4/+0x1F8/+0x204/+0x21C/+0x228, the
//       orientation basis, the scalar angle copies) is recomputed each tick and cannot be written.
//     * ON THE WIRE. In co-op the host follows it, which is how it was proven to be the outbound
//       replication source rather than another local mirror.
//
// WHY A HOOK
//   gs:[0x58] resolves the sim's thread-local block ONLY on the sim thread. Read from the UEVR
//   frame path it is zero -- 1.19 billion samples of zero, across every thread, before that was
//   understood. So the write has to happen inside the sim's own call stack, and the orientation
//   getter is the vehicle: it runs there at ~2600 calls/sec.
//
//   Specifically NOT the aim setter at dll+0x1842C0. That hook installs and reports success and is
//   never invoked during normal aiming -- zero calls across a full session while the aim moved
//   freely. "Installed" is not "running", and this is the file where that distinction was paid for.
// ============================================================================================

#pragma once

#include "DevTools.hpp"

#include <atomic>
#include <cstdint>

#include <cstdint>   // uintptr_t / uint32_t in the sim_tls_index signature below

namespace halo {

// Install/remove the sim-thread hook and keep it in step with config. Call once per frame from the
// game thread. Cheap: a flag compare once the hook is up.
void blam_drive_tick();
// FRAMEAUDIT: the control record's angles in UE-convention degrees (false if unresolved).
bool blam_ctl_read_ue_deg(float* yaw_deg, float* pitch_deg);

// The sim's TLS block, published when the control record resolves (gs:[0x58] only means anything
// ON the sim thread, but the block it yields is ordinary heap memory readable from anywhere).
// Consumed by the TLS-graph walker in MemScan.cpp -- the navpoint hunt's anchor search walks the
// structures reachable from this root, the same family the control table itself lives in.
extern std::atomic<uintptr_t> g_sim_tls_block;

// The resolved Blam control record, read-only. Consumed by the AIMDIG probe (MemScan.cpp) to ask
// whether this record is reachable from the PlayerController by pointer derefs -- the same shape
// that retired AimDirect's watchpoint hunt. A hit would mean resolution no longer needs the sim
// thread, i.e. neither the getter hook nor tier 2's TEB walk would be load-bearing for FINDING it.
uintptr_t blam_control_record();

// Resolve ANY object datum through the sim's object table. SIM THREAD ONLY (walks gs:[0x58]).
uintptr_t resolve_object_by_datum(uint32_t datum);

// GRENTRACK (dev, throwdump): the projectile object the spawn hook just created, and when. The
// hook (sim thread) writes them; throw_dump_probe samples the object's position for ~1.2 s so the
// log shows whether the grenade FLIES from spawn or sits held until an animation event.
extern std::atomic<uintptr_t> g_grentrack_obj;
extern std::atomic<long long> g_grentrack_at_ms;

// GRENINSTANT (dev, greninstant): the grenade released at spawn, and the velocity to keep
// re-asserting on it for 400 ms so the animation keyframe's own late release is overwritten.
extern std::atomic<uintptr_t> g_greninst_obj;
extern std::atomic<long long> g_greninst_at_ms;
extern std::atomic<float>     g_greninst_vx, g_greninst_vy, g_greninst_vz;

// ---- WRIST RADAR blips (blipdump survey, 2026-08-28): unit+0x177 is the TEAM byte -- 0x0E
// human (player + marines, armed or corpse), 0x0D covenant (the one carrier photographed held
// PLASMA grenades). Published by the sim (slow cached table scan + per-publish position reads):
// relative Blam-unit offsets from the player, team, and a moving flag. Consumed by WristHud.
constexpr int MAX_BLIPS = 12;
extern std::atomic<int>   g_blip_count;
extern std::atomic<float> g_blip_dx[MAX_BLIPS], g_blip_dy[MAX_BLIPS];
extern std::atomic<int>   g_blip_team[MAX_BLIPS];     // 0 = human, 1 = covenant
extern std::atomic<bool>  g_blip_moving[MAX_BLIPS];
// Identity (low dword of the object pointer): publish order compacts as contacts drop in and
// out of range, so an INDEX is not a contact -- the renderer's per-blip smoothing must key on
// this or it smears one dot's motion onto another's.
extern std::atomic<uint32_t> g_blip_id[MAX_BLIPS];
// The RAW +0x177 byte, published alongside the two-way classification. The original survey saw
// three humans and one Covenant, which is far too thin to call the byte "team" -- different
// enemies paint different colours in the field, so at least one more value exists. Logged so the
// real value set can be read off instead of assumed.
extern std::atomic<uint32_t> g_blip_raw[MAX_BLIPS];
// SPECIES ID: the object's leading dword, a tag/definition id. THIS is the real species key --
// surveyed 2026-08-29, it groups instances exactly (six of one type, two of another) and it
// separates a MARINE from an ELITE, which +0x177 cannot (both read 0x0E there). Blip colour is
// keyed on this. Assumed stable across runs; if colours ever shuffle between sessions, re-survey
// -- a per-run pointer or handle would look just like this in a single capture.
extern std::atomic<uint32_t> g_blip_type[MAX_BLIPS];
// The movement test's own numbers per contact: measured speed (blam units/sec), the window it
// was measured over (ms), and the consecutive-window run. Published because contacts that were
// visibly walking metres reported mv=0, and the test has to be read rather than reasoned about.
// ABSOLUTE Blam position per contact, for the species-naming match: a contact is identified by
// finding the Unreal actor standing at the same place and reading its CLASS NAME, which -- unlike
// the tag id -- is the same on every level.
extern std::atomic<float> g_blip_wx[MAX_BLIPS], g_blip_wy[MAX_BLIPS], g_blip_wz[MAX_BLIPS];
extern std::atomic<float> g_blip_speed[MAX_BLIPS];
extern std::atomic<int>   g_blip_dtm[MAX_BLIPS];
extern std::atomic<int>   g_blip_run[MAX_BLIPS];

// A loaded module's `_tls_index`, read from its own PE TLS directory (IMAGE_TLS_DIRECTORY's
// AddressOfIndex, which the loader has already relocated). EXACT on any build of any variant of the
// binary: nothing to record, nothing to re-derive after a patch, no signature to maintain.
//
// Shared rather than copied. BlamAim.cpp hooks the same function on the same module and needs the
// same number, and its own note is explicit about not wanting a third copy of a definition -- and a
// stale RVA there would be worse than useless: that file is dev-only, so it would fail in a way no
// player could ever reproduce, sending an investigation after a phantom.
//
// out_rva reports where the index actually landed, so two logs from different builds are directly
// comparable -- which is how a build difference gets noticed at all.
bool sim_tls_index(uintptr_t module_base, uint32_t* out_index, uintptr_t* out_rva);

// The write, TIER 1. Called from the sim-thread hook: it resolves through gs:[0x58], which only
// answers on that thread. Stands down during aim calibration and in stick mode; see the body.
void drive_control_angles();

// The write, TIER 2 -- GAME THREAD, and only when tier 1 cannot happen at all.
//
// Same gates, same arithmetic, different RESOLVER: it finds the sim's TLS block by walking the
// process's threads and reading each TEB, so it needs no hook and no code address. That matters
// because the getter RVA is the last build-fragile constant here, and on a build where it is stale
// the hook installs, never fires, and nothing is ever written -- silently.
//
// Call ONLY from blam_drive_tick(), and only after the watchdog has established the hook is dead.
// It opens a toolhelp snapshot while the record is unresolved, which is fine at frame rate and
// ruinous at the 2600 Hz the hook runs at.
void blam_drive_offthread_write();

// HOOK OWNERSHIP (dev builds only -- in a release build this file always owns the address).
//
// BlamAim.cpp installs a richer diagnostic hook on the SAME function, dll+0x5A6AD0, and calls
// drive_control_angles() from it. Both must never be installed at once.
//
// Ownership is therefore decided by ONE value that both sides can read BEFORE either installs:
// `blamaim`. Non-zero means the diagnostics own the address. There is deliberately no runtime
// claim/release handshake -- the first attempt used one and it raced: the claim only took effect
// on the next blam_drive_tick(), while blam_aim_tick() installed on the SAME frame, so both hooks
// went onto the function and the subsequent uninstall tore out the survivor's trampoline. The
// getter then read "installed but NEVER CALLED" and the aim silently stopped replicating.
//
// Self-healing by construction: if BlamAim fails to install it sets blamaim=0, which flips
// ownership back here on the next frame.
//
// blam_drive_tick() must run BEFORE blam_aim_tick() so that on a 0->non-zero transition this file
// removes its hook in the same frame the diagnostics install theirs.

// Unit state read from the player's unit object on the sim thread (see publish_unit_state):
// grenade type (0 frag / 1 plasma), pouch counts, whether the read is live, and whether the unit
// has a parent object (vehicle seat / turret). Consumed by the holsters.
extern std::atomic<int>  g_unit_gtype, g_unit_gfrag, g_unit_gplasma;
extern std::atomic<bool> g_unit_gvalid;
extern std::atomic<bool> g_unit_mounted;
// The unit's WORLD POSITION (+0x20, Blam world units) and FACING (+0x50, unit vector, Blam
// frame), published beside the grenade state for the seat camera and the in-vehicle view.
// Measured: over 972 samples the large-motion delta ratios against the camera are +308/-306/+332,
// i.e. the 304.8 cm world unit with Blam's Y negation, constant residual = the eye height.
extern std::atomic<float> g_unit_px, g_unit_py, g_unit_pz;
extern std::atomic<bool>  g_unit_pvalid;
extern std::atomic<float> g_unit_fx, g_unit_fy;
// The MOUNTED VEHICLE's facing (+0x1D4 pair -- swept 3226 deg as a unit vector in the spin test
// while every +0x50 field stayed constant) and its own position (+0x20, same layout as the
// biped's). Resolved once per mount from the biped's parent datum and cached; retried ~1 s while
// mounted-unresolved, because a mount-edge failure used to latch a backwards camera all ride.
extern std::atomic<float> g_veh_fx, g_veh_fy;
extern std::atomic<bool>  g_veh_fvalid;
extern std::atomic<float> g_vehpx, g_vehpy, g_vehpz;

// SEAT PUBLISH EVIDENCE (vehlog). seq advances on every successful rider read from any path (sim
// publish or direct read), so the camera can tell a live rider from a frozen one. calls = sim
// publishes, norec = stick-mode calls that had no control record to publish from, reresolve =
// stick-mode record re-resolves, direct = vehseatdirect reads.
extern std::atomic<uint32_t> g_seat_pub_seq, g_seat_pub_calls, g_seat_norec, g_seat_reresolve,
                             g_seat_direct_reads;
// The rider object and its vehicle object as the last sim publish saw them (vehseatdirect).
extern std::atomic<uintptr_t> g_seat_obj, g_seat_vobj;

// vehseatdirect: refresh the seat atomics from the cached object pointers. Any thread; acts only
// while stick mode holds the sim publish's normal path off. No-op when the key is 0.
void seat_direct_refresh();

} // namespace halo
