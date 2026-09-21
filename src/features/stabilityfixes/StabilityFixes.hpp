// StabilityFixes -- what is left of the fork's "stabilityfixes" bundle once the FIXES moved out.
//
// FEATURE stabilityfixes (Experimental, off by default). It arrived as one master key over three
// different kinds of thing, and only one of them belonged behind a key:
//
//   FIXES to our own code        -> now UNCONDITIONAL. See kAlwaysOnServices in core/Services.hpp.
//   DIAGNOSTICS                  -> still here, still opt-in (that is what a diagnostic should be).
//   TUNING for the fork features -> still here, because it only means anything with them on.
//
// WHY THE FIXES MOVED (2026-09-20). A fix to OUR code is not a feature. Behind an experimental key
// it is off for every player who never opens that menu, so the bug ships and the fix rides along
// disabled. The bundle contained, among others: a blocking disk load re-run on every marker rebuild
// (measured by the fork's author at 124 loads in 13 minutes, a freeze every six seconds), a
// find_ui_manager() miss that walks the whole ~296k object array and caches nothing, and a rig
// pointer used after a tick fault. Those are ours to carry, not to offer.
//
// WHAT IS STILL SWITCHED BY THIS ROW:
//   SVC_STABILITY
//     turn instrument (stabilityturnlog) and the widget probe log gate (stabilitywidgetlog)
//     holster marker tint (stabilityholstermarkercolor) and minimum throw speed (stabilitygrenminthrow)
//     the holster button steal's extra/dead masks
//   SVC_MELEE_INSTRUMENTS  aim-hand melee holster veto, second aim pin, hold check, FIRED line
//
// ONE ENTRY WAS DROPPED AS REDUNDANT: the late re-assert for xrlayerhidews 3/4. We already run
// reticule_mode3_reassert() from four hosts, one of them at the same point that hook fired, plus a
// reflection-free variant above the fault pause. See features_reticule_widget_moved().
//
// NO RELIANCE DECLARATIONS ANY MORE. roomscale and heightcal used to declare SVC_LEASH_GATE (and
// roomscale SVC_RIG_GUARD) because they misbehave without them. Those guards are unconditional now,
// so the features no longer have to ask for them. Table: kStabilityFixesHooks.

#pragma once

#include "features/FeatureHooks.hpp"

namespace halo {

extern const FeatureHooks kStabilityFixesHooks;

} // namespace halo
