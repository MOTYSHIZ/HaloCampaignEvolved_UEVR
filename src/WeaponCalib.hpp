// Per-weapon calibration capture.
//
// Reuses the author's pose-match gesture wholesale: hold a key, the weapon freezes, you line your
// controller up with it, release and the solve runs. The only difference is where the answer goes.
// A separate key is what selects that -- END still writes the global calibration in
// halo_vr_calib.cfg, and the per-weapon key writes a DELTA for whatever you are holding.
//
// WHY A DELTA AND NOT A SECOND ABSOLUTE. The global calibration is what makes every unlisted
// weapon sit correctly; replacing it per weapon would mean calibrating all of them before any felt
// right. Storing the difference means the base keeps working and you only capture where a specific
// weapon annoys you.
//
// WHY A THIRD FILE. Config.cpp states the rule: calibration results live in their own file because
// the plugin rewrites it on every capture, and rewriting halo_vr.cfg would destroy the comments
// documenting every other knob. Hand-written wpnoff lines therefore stay in halo_vr.cfg and keep
// their comments; captured ones go to halo_vr_weapons.cfg, which is machine-owned and safe to
// rewrite.

#pragma once

// API.hpp, NOT Plugin.hpp -- see UeObject.hpp.
#include "uevr/API.hpp"
#include "Config.hpp"

#include <windows.h>
#include <string>

namespace halo {

// Full path to halo_vr_weapons.cfg. Set once at startup alongside the other config paths.
extern char g_wpn_calib_path[MAX_PATH];

// TRUE while the per-weapon key is held, so the caller can drive the existing freeze/solve path
// exactly as the global key does.
bool wpn_calib_held();

// Poll the per-weapon key. Called from the same block that polls calib_key, so both gestures are
// edge-detected on one thread rather than two.
void wpn_calib_poll();

// Called at the point the solve has finished and g_cfg holds the freshly fitted values. Returns
// TRUE if this capture was claimed as a per-weapon one -- in which case the caller must NOT run
// the global write_calib_file(), because the global calibration is meant to be left alone.
bool wpn_calib_capture();

// Load halo_vr_weapons.cfg into the wpnoff table. Called after the main config parse, so captured
// entries and hand-written ones end up in the same list.
void wpn_calib_write_file();

void wpn_calib_load();

// The weapon in hand, as the tables key it ("AssaultRifle", "Magnum"); empty if none.
std::string weapon_key();

// PALETTE per-weapon rigid delta (Config::WeaponFix). find: the entry whose match is a substring
// of the key, or nullptr. set: replace-or-append and rewrite halo_vr_weapons.cfg.
const WeaponFix* wpnfix_find(const std::string& key);
void wpnfix_set(const std::string& key, const float q[4], const float t[3]);

// Consume the "per-weapon key was just released" latch. The palette freeze path takes it so the
// old rig-path capture cannot later claim an unrelated END solve with it.
bool wpn_calib_take_pending();

} // namespace halo
