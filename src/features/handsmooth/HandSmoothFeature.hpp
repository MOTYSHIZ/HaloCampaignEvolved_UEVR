#pragma once

// HAND SMOOTHING (handsmooth, Stable since 2026-09-27: on by default, handsmooth=0 turns it off).
//
// Steadies the tracked controllers with a 1 Euro filter before anything reads them. The filter is
// core machinery (core/HandSmooth, service SVC_HAND_SMOOTH) because the pose latch reads through it
// too; this feature owns the master key, turns the service on, and serves get_pose() through its
// pose_latched slot. It sits after palettewpn in the feature list so a live pose-latch snapshot --
// which was itself taken through the filter -- still answers first.
//
// FEATURE handsmooth. Hook slots: parse_key, pose_latched. Services: SVC_HAND_SMOOTH.
// Table: kHandSmoothHooks.

#include "features/FeatureHooks.hpp"

namespace halo {

extern const FeatureHooks kHandSmoothHooks;

bool handsmooth_parse_key(const char* key, const char* val, double v);

} // namespace halo
