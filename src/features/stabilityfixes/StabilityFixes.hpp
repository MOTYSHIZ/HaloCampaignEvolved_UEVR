// StabilityFixes -- what is left of the fork's "stabilityfixes" bundle once the FIXES moved out.
//
// FEATURE stabilityfixes (Stable, ON by default since 2026-09-27 -- see THE 2026-09-27 AUDIT below).
// It arrived as one master key over three different kinds of thing, and only one of them belonged
// behind a key:
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
// WHAT IS STILL SWITCHED BY THIS ROW (re-read 2026-09-27; the first two were missing from this list):
//   SVC_STABILITY
//     THE CONFIG RELOAD BRACKET (core/config/CfgRead.cpp, cfg_reload_begin): while load_config
//     resets and re-parses g_cfg, readers on the hook threads (XInput, render, sim) take a copy of
//     the old config instead of a half-parsed one. A fix, not a tuning -- it stayed behind this key
//     on 2026-09-20 when the others moved. core/WorldScale.hpp works around its absence for rig_scale.
//     the widget readback gate (features_widget_log, stabilitywidgetlog): off, the author's
//     TINT COMPARE readback runs -- a K2_GetVectorParameterValue call and a log line on every 32nd
//     bind_widget_slate_ui call, ~every 6 s while the reticule widget lives. On, only with the key.
//     turn instrument (stabilityturnlog)
//     holster marker tint (stabilityholstermarkercolor) -- the grenade pouch markers, which exist
//     only with holstergren or holstermarkers=2. The minimum throw speed (stabilitygrenminthrow)
//     moved to the grenade gesture: it is on whenever holstergren is (core/fixes/HostFixes.cpp)
//     the holster button steal's extra/dead masks (stealextra, holstergswitchmask: both 0 by default)
//     render time for the base mod's gun and hands (stabilityrendertime, a DEV key, off; measured
//     worse than off, kept for the timing work) -- core/fixes/RenderTime.hpp
//   SVC_MELEE_INSTRUMENTS  the veto is REDUNDANT now: Gesture.cpp has called holster_melee_veto()
//     itself, one line earlier, since the 2026-09-18 review, so melee_vetoed never fires (and its
//     MELEE VETOED line with it). The swing-start aim hold only acts at meleeaimmode=0 (default 1,
//     whose hold the author's strike path stores). The rest -- hold check, FIRED line -- is meleelog.
//
// THE 2026-09-27 AUDIT, which moved this row to Stable and on by default. At default settings it
// changes two things for a player -- the reload bracket and the silenced readback, both fixes --
// and nothing else: every other entry above is either off behind its own key or has nothing to
// act on. Nothing in it is an experiment any more; the one that was (stabilityrendertime) is a dev
// key and stays off. It was already on in the author's own play settings when the audit ran.
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
