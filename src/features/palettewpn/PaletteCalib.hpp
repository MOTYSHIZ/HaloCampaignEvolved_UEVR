#pragma once

// THE PALETTE WEAPON'S CALIBRATION (palettewpn): its own keys (palgripfix, palaimfix, palaimoffyaw, palaimoffpitch,
// palaimcalibver, palwpnfix, palwpncalibkey) and its own file, halo_vr_palette_calib.cfg. The author's calibration
// keys and files are neither read nor written here.

#include "Config.hpp"   // WeaponFix
#include "Math.hpp"     // Quat

#include <string>

namespace halo {

// palettewpn's parse slot, for the keys above. True = taken.
bool palette_calib_parse_key(const char* key, const char* val, double v);

// palaimfix right-multiplied onto an aim pose (the pose unchanged when palaimfix is invalid). Any thread.
Quat pal_apply_aim_fix(const Quat& q_src);

// The palwpnfix entry whose match is a substring of the weapon key, or nullptr. Game thread (points into g_cfg).
const WeaponFix* pal_wpnfix_find(const std::string& key);

// A Home capture for the weapon key: replaces the entry that matches the key (else adds one), marks it captured and
// rewrites halo_vr_palette_calib.cfg. Game thread.
void pal_wpnfix_set(const std::string& key, const float q[4], const float t[3]);

// Rewrite halo_vr_palette_calib.cfg in full with the captured values only. Called only by the palette weapon's
// captures and its menu reset, never while the palette weapon is off. Game thread.
void pal_calib_write_file();

} // namespace halo
