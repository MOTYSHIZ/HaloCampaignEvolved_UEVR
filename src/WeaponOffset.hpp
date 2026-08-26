// Per-weapon grip and offset adjustment.
//
// WHY. The calibration in halo_vr_calib.cfg is ONE set of numbers -- grip, gripyaw, griproll,
// offx/offy/offz -- and it is applied to whatever you happen to be holding. A pistol, an assault
// rifle and a rocket launcher do not share a grip geometry, so a calibration that suits one is
// visibly wrong on another.
//
// DELTAS, NOT REPLACEMENTS. Each entry adjusts the calibrated base rather than overriding it, so
// the pose-match calibration keeps doing its job for every weapon and an entry is only needed
// where a specific weapon disagrees. It also means a weapon with no entry behaves exactly as it
// does today -- adding this feature changes nothing until you tune something.
//
// The PIVOT is deliberately not included: Rig.cpp already derives it per weapon from the
// PrimaryWeapon socket, so it is correct without help and overriding it would make things worse.

#pragma once

// API.hpp, NOT Plugin.hpp -- see UeObject.hpp.
#include "uevr/API.hpp"

namespace halo {

// Game thread, once per tick, BEFORE the rig consumes g_cfg's grip and offset fields.
//
// Captures the calibrated base whenever the config is re-read, then applies the delta for whatever
// weapon is currently held. Capturing on reload rather than once is what stops the delta
// compounding on itself every tick.
void weapon_offset_update();

// Called at the END-key (global) calibration release, BEFORE write_calib_file(), and only on the
// path where the per-weapon capture did not claim the result.
//
// The solve leaves an ABSOLUTE fit in g_cfg, measured while the current weapon's delta was applied
// -- so the fit contains the delta. Without this, write_calib_file() persists wpn_base_*, which is
// still the OLD base, and the global calibration silently discards its own result whenever the
// weapon in hand happens to have an entry.
void weapon_offset_adopt_solve();

} // namespace halo
