// PER-WEAPON SCOPE TRIMS -- zoom and pane placement, as DELTAS on the global scope fit.
//
// The same shape as WeaponOffset.hpp, and for the same reason: one scope calibration cannot suit
// a pistol, a battle rifle and a sniper. What differs per weapon is small (a sniper wants more
// magnification and the pane further out; a pistol wants it closer), so a REPLACEMENT table would
// mean re-fitting every weapon from scratch. Deltas mean the global fit still does the work and an
// entry is only needed where a weapon actually disagrees.
//
// A weapon with no entry lands EXACTLY on the global fit, so switching this on changes nothing
// until something is captured. That is what makes it safe to ship enabled.
//
// WHAT IT TRIMS. The in-scene pane path in Scope.cpp:
//     scope_zoom                          magnification (pane lens = scope_base_fov / zoom),
//                                         trimmed by a PLAIN MULTIPLIER: 1.5 = 1.5x. 0 = unset.
//     scope_dist / scope_right / scope_up placement along/around the aim ray, cm
//     scope_rot_p / scope_rot_y / scope_rot_r  pane facing, degrees
//
// NOT the compositor-layer path (ScopeLayer.cpp, scope_layer_*). That is a second placement
// system with its own offsets; when it ships, it wants its own delta set rather than having this
// one overloaded onto it -- the two do not share units or a frame.
//
// WHY IT IS ITS OWN FILE. Scope.cpp is 2600 lines and under active development. Everything here
// is additive: it reads the base out of g_cfg, writes the trimmed values back, and Scope.cpp
// consumes them exactly as it always has. The only edit over there is routing a capture.
#pragma once

namespace halo {

// Apply the held weapon's trim on top of the global scope fit. Call once per tick, BEFORE anything
// that reads g_cfg.scope_* for placement. Cheap: one class-name read on the held weapon, no sweep.
void scope_offset_update();

// Store the CURRENT g_cfg.scope_* as a delta against the base, for the weapon in hand, and persist.
// Returns true if it claimed the capture (including the "no weapon in hand" refusal, which must
// still swallow it so the caller does not fall through to writing the global fit).
bool scope_offset_capture();

// Arm the next scope-calibration release to land in the held weapon's trim instead of the global
// fit. Cleared by the capture. Set from the Script UI bridge (calib:wpnscope) or the config key.
void scope_offset_arm(bool on);
bool scope_offset_armed();

// Arm a BASE (global-fit) calibration. Holds the scope pane open for the gesture -- which the
// calibration key alone no longer does -- and routes nothing: the capture already writes the global
// fit when the per-weapon arm is clear. Mutually exclusive with scope_offset_arm().
void scope_base_arm(bool on);
bool scope_base_armed();
void scope_base_arm_clear();
// Drop the equipped weapon's wpnscope trim (back to the global fit). false = nothing to clear:
// no weapon in hand, or that weapon never had one. Rewrites halo_vr_weapons.cfg.
bool scope_offset_clear_current();

} // namespace halo
