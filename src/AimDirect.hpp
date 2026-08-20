// Set Blam's aim DIRECTLY, instead of steering it through the right stick.
//
// WHAT THIS REPLACES
//   The whole aim loop exists because we believed Blam only accepted additive input, so aim had to
//   be driven by synthesising stick deflection and closing a loop around a rate actuator. That is
//   false: there is a writable rotator upstream of ControlRotation, and assigning to it sets the
//   game's aim exactly and immediately (docs\BLAM_AIM_DIRECT_WRITE.md).
//
//   Everything the loop works around belongs to the stick path and not to the game: the ~0.265
//   hardware deadzone, the 7x-nonlinear response curve, the ~14 deg/s smallest possible correction,
//   a frame of dead time, and the overshoot and settling that follow from all three. A direct write
//   has none of them -- no error, no lag, no tuning.
//
// WHY IT IS BEHIND A FLAG, DEFAULT OFF
//   Direct assignment writes the value the game itself replicates, which is the reason aim-steering
//   was chosen over redirecting projectiles locally -- but THAT IT REPLICATES IS UNVERIFIED. Until
//   multiplayer is tested, the stick loop stays the shipping path and this is opt-in.
//
// HOW THE TARGET IS FOUND
//   The rotator is heap-allocated, so its address differs every run and cannot be hard-coded. It is
//   located by watching the copy chain and reading the source pointer out of the trap context:
//
//     stage 1  watch PlayerController+0x358  ->  writer's rbx  = L1 (a mirror)
//     stage 2  watch L1+0x08                 ->  writer's rbx + 0x20 = the real rotator
//
//   Two brief hardware watchpoints at session start, then never again. Self-locating: nothing is
//   pinned to an offset that a patch could move, and if the layout changes it simply re-derives.
//   The alternative -- calling exe+0x1218AA0, which returns the struct -- is cheaper but needs its
//   calling convention established first, and guessing wrong there crashes rather than misreads.

#pragma once

#include <cstdint>

namespace halo {

// Drives the locate state machine. Call from the tick. Cheap once resolved (an atomic load), and a
// no-op entirely when aim_direct is off.
void aim_direct_tick();

// True once the rotator has been located and is safe to write.
bool aim_direct_ready();

// Read-only views for the AIMDIG chain dig (MemScan.cpp) and its logs: the resolved rotator, the
// quaternion source, and the RIP of the game instruction the watch caught writing the chain --
// module-relative, that RIP is a build-stable code RVA. All zero until located.
uintptr_t aim_direct_target();
uintptr_t aim_direct_quat_src();
uintptr_t aim_direct_writer_rip();
uintptr_t aim_direct_known_pc();

// Assign the aim. Angles in degrees, the same convention as ControlRotation. Roll is left alone.
// Returns false if the target is not resolved, so callers can fall back to the stick loop.
bool aim_direct_set(double pitch_deg, double yaw_deg);

// Forget the resolved address. Called when the PlayerController changes -- a level load or respawn
// reallocates the chain, and writing through a stale pointer would be a write into freed memory.
void aim_direct_invalidate();

} // namespace halo
