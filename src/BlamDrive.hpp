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

} // namespace halo
