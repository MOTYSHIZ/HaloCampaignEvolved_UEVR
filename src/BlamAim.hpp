// ============================================================================================
// ⚠️ READ THIS FIRST -- THE ANSWER IS NEXT DOOR, IN BlamDrive.cpp (2026-08-08).
//
// The working aim path is the sim's ANGULAR CONTROL STATE (tls_block+0xB8, stride 0x198, yaw +0x94
// / pitch +0x98), driven by `blamangles`. It is 1:1 with the hand, it persists, and it reaches the
// host in co-op. That lives in BlamDrive.cpp/.hpp, which is NOT dev-gated and ships.
//
// THIS file is the investigation that found it, and everything else it ruled out along the way. It
// is dev-only and stays that way. The banner below is the state of the OBJECT-TABLE hunt -- still
// accurate, still a dead lane -- and must not be read as a verdict on the aim work as a whole.
//
// ---------------------------------------------------------------------------------------------
// ⚠️ SETTLED 2026-08-05 -- THE OBJECT-TABLE HUNT IS A DEAD LANE. Keep it for the evidence.
//
// The premise below (that the Blam sim owns a private, OFFSET aim basis we must locate and write)
// is REFUTED. Measured live while firing: the Blam shot direction equals the UE ControlRotation
// forward vector with Y negated, to 0.000 degrees, across a 52.8 degree sweep -- with the plugin
// off, on, and under direct drive. There is no divergent Blam aim to find, so locating one buys
// nothing.
//
// This does NOT mean writing ControlRotation steers the shot -- it does not. ControlRotation is an
// output mirror of the Blam camera and nothing reads it (see docs\Wargames\25-blam-aim-hunt.md).
// The 0.000 degrees is two mirrors of one upstream aim agreeing. The lever is upstream of both,
// via the input layer -- docs\GameInput-Findings-Handoff.md.
//
// So: do NOT resume the object-table hunt. The TLS chain never resolved (zero across 1.19e9
// samples) and it would not have helped. The remaining problem is MOVING the aim, not aiming the
// shot: if the aim moves, the shot follows it exactly.
//
// Also recorded here as a negative result: writing the finished direction at params+0x28 DOES land
// (the field visibly changes before the constructor reads it) but does NOT steer the projectile --
// ~360 rounds at a 90 degree offset went where they always went. That path is `blamaim >= 3`,
// dev-gated and default off. It is not a feature. Full write-up: docs\BLAM_AIM_FINDINGS.md.
// ============================================================================================
//
// Resolve the Blam sim's OBJECT ORIENTATION -- the field the shot direction is actually taken from.
//
// WHY THIS EXISTS
//   Value scanning for the aim failed repeatedly, and the reason is now understood: the aim appears
//   in many derived copies, and the copies are indistinguishable from the source by value. Writing
//   the best candidate we found changed nothing -- the write persisted untouched while the shot did
//   not move, which is the signature of a copy nothing reads.
//
//   Static analysis found the source instead. The projectile spawn path converges: 45 of the 55
//   sites that create a projectile share one parameter constructor, and the direction handed to it
//   comes from a getter that reads the SHOOTER'S OBJECT:
//
//     HaloSimulation_tag_release.dll + 0x5A6AD0
//       mov  r9d, [dll+0xD72730]      ; TLS slot index
//       mov  rax, gs:[0x58]           ; TEB->ThreadLocalStoragePointer
//       mov  rax, [rax + r9*8]        ; TLS slot -> block
//       mov  r9,  [rax + 0x20]        ; block+0x20 -> sim context
//       mov  rdx, [r9 + 0x50]         ; context+0x50 -> object table
//       mov  r9,  [rdx + idx*24 + 0x10]   ; -> the object
//       vmovsd [r9+0x50] -> out A     ; 3 floats: orientation vector A
//       vmovsd [r9+0x5C] -> out B     ; 3 floats: orientation vector B
//
//   So the aim basis is a PERSISTENT FIELD on a Blam object, reachable by pointer chain rather than
//   by search. That is why no scan could isolate it: we never had the object pointer.
//
// WHAT THIS DOES -- READ ONLY
//   Walks the object table, reads +0x50 from each live object, and reports the ones whose vector
//   best matches the direction the game is currently aiming. That validates the finding and yields
//   the object pointer in one pass. It writes nothing.
//
//   Self-validating on purpose: if +0x50 is NOT the aim basis, no object will correlate and the
//   report says so, rather than handing back a plausible-looking address to be trusted on faith.
//   Today already produced one address that looked perfect and turned out to be inert.

#pragma once

#include "DevTools.hpp"

namespace halo {

#if HALO_VR_DEV

// Driven by the `blamaim` config key: set it to a non-zero value to run one pass. Cheap enough to
// run on demand, far too chatty to leave on.
void blam_aim_tick();

#else

inline void blam_aim_tick() {}

#endif

} // namespace halo
