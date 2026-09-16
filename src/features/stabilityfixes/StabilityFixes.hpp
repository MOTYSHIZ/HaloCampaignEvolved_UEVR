// StabilityFixes -- robustness fixes to the base mod's own code paths.
//
// FEATURE stabilityfixes (Experimental, off by default). One master key for fixes that belong to no
// feature. The code lives in core/fixes; this row only switches the services on:
//   SVC_STABILITY
//     nav marker lane fault quarantine (SEH guard, pool drop, 10 s sleep)
//     tick stage markers named in the tick fault report
//     HaloUIManagerSubsystem miss throttle (one sweep per 128 misses)
//     flat reticle sweep only until the hidden widget is found; rescan when a hidden widget dies
//     compositor reticule tick, latch, re-assert and source tick above the early-outs
//     stick mode exit after a death: full re-anchor instead of folding the respawn camera
//     turn instrument (stabilityturnlog)
//     teardown order: the OpenXR layer first, and the API layer projection restore
//     reticle mode 3/4 late re-assert after a widget move, the widget probe log gate (stabilitywidgetlog)
//     holster marker tint (markertint, stabilityholstermarkercolor) and minimum throw speed (stabilitygrenminthrow)
//     the two-handed hold drops on a gesture reset (stick mode, calibration, the kill switch)
//     the menu command file: an attributes query before the open, on the tick path
//   SVC_RIG_GUARD      fault recovery rig drop, stale rig and parent guard
//   SVC_LEASH_GATE     HMD pose plausibility gate in the leash block
//   SVC_RETICULE_FIXES asset load failure memo (no retry for 5 minutes)
//   SVC_MELEE_INSTRUMENTS  aim-hand melee holster veto, second aim pin, hold check, FIRED line
//
// RELIANCE: roomscale and heightcal also declare SVC_LEASH_GATE, and roomscale SVC_RIG_GUARD, because
// they do not work correctly without them (see the report). Table: kStabilityFixesHooks.

#pragma once

#include "features/FeatureHooks.hpp"

namespace halo {

extern const FeatureHooks kStabilityFixesHooks;

} // namespace halo
