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

// palettewpn's aim_reference_offset slot: while the palette weapon owns the aim, the reference restore uses
// palaimoffyaw/palaimoffpitch, and adds the frame yaw only for palaimcalibver >= 2.
bool palette_calib_aim_reference_offset(float* yaw, float* pitch, bool* valid, float* frame_yaw, float write_frame_yaw);
// palettewpn's aim_calibrated slot: while the palette weapon owns the aim, Page Down lands in palaimoffyaw/palaimoffpitch
// (stamped palaimcalibver=2) and halo_vr_palette_calib.cfg.
bool palette_calib_aim_calibrated(float off_yaw, float off_pitch, float frame_yaw);

// palettewpn's menu_command slot: calibreset:palette deletes halo_vr_palette_calib.cfg (the shipped values return
// with the reload it causes); calibreset:palwpn drops the held weapon's captured palwpnfix. Neither creates the file.
bool palette_calib_menu_command(const std::string& line);

} // namespace halo
