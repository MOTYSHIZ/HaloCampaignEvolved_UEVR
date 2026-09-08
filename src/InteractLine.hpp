#pragma once
//
// THE GRAB GUIDE: a thin beam from the support hand to the point it would grab.
//
// WHAT IT IS FOR. Player IK is not enabled in release, so there is no left arm on screen and no
// visual answer to "is my hand actually on the barrel yet?". The two-handed hold latches on a zone
// the player cannot see, which makes a working feature feel broken -- you squeeze, nothing happens,
// and nothing tells you whether you were 2 cm out or holding the wrong button.
//
// IT ONLY APPEARS WHEN THE GRAB WOULD SUCCEED. The show condition is TwoHandReach::in_zone, which
// is the hold's own acquisition gate -- true exactly when a grip press right now would latch. So
// the beam is a promise, not a hint: if you can see it, squeezing works. That is deliberately
// stricter than "near the barrel", and it is why the beam is short (at most the zone radius, ~9 cm)
// rather than a pointer stretching across the room.
//
// It disappears once the hold latches: the affordance has been taken, and a beam threaded through
// a hand that is already on the gun is clutter.
//
// GEOMETRY COMES FROM THE HOLD, NOT FROM HERE. TwoHandAim publishes both endpoints because it owns
// the zone; this module only draws. See TwoHandReach.
//
// SPACE. The caller hands both endpoints already converted to GAME space, relative to the weapon
// rig component, because the VR-metres to game-space transform is Plugin.cpp's vr_to_rig() and
// that lambda is the single blessed copy. Do not re-derive it here -- its own comment records why
// a second copy would drift.
//
// COST. One StaticMeshComponent, created once and moved on the ticks it is visible. No allocation,
// no scans, no per-frame reflection beyond the three transform calls the reticule already makes.
// Hidden by SetVisibility rather than destroyed, so toggling costs nothing.
//
// LATER. Grenades and magazines want exactly this, and the interface is already the right shape:
// two points plus a reason to show. When manual reloading lands, feed it the magazine well instead
// of the barrel and nothing here changes.

#include "Math.hpp"
#include "uevr/API.hpp"

namespace halo {

// Draw (or hide) the guide for this tick.
//
// `rig`         the weapon rig component -- the world anchor both endpoints are relative to.
//               Pass nullptr and the guide hides itself.
// `hand_rel`    support hand, GAME-space centimetres relative to the rig component.
// `target_rel`  the point it would grab, same frame.
// `show`        false hides it. The caller owns the policy (in_zone, not latched, feature on).
// `alpha`       0..1 opacity, so the caller can fade it in rather than pop it.
void interact_line_update(uevr::API::UObject* rig, const Vec3& hand_rel, const Vec3& target_rel,
                          bool show, float alpha);

// Drop the component handle -- the pawn it is outered to was destroyed. Next update re-creates it.
void interact_line_release();

// This family's config keys, owned here rather than on Config.cpp's else-if chain. That chain has
// already hit MSVC's C1061 nesting limit twice (see parse_scope_key and two_hand_parse_key), and
// adding seven more rungs to it tipped it over again -- so new families get a flat link.
// Returns true if `key` was one of ours.
bool interact_line_parse_key(const char* key, double v, const char* val);

} // namespace halo
