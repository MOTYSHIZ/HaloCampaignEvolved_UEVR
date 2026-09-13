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

// ---- TWO DESTINATIONS, ONE KEY (2026-08-30) ----------------------------------------------------
//
// The capture above writes `wpnoff` -- a delta on the RIG path's fitted grip and mount. The PALETTE
// weapon carry (src\palettearm\PaletteArm.cpp) consumes none of those: it puts the weapon's own
// AUTHORED marker node onto the controller, so there is no grip fit in its chain to be a delta
// against. What it needs is a rigid transform in the controller's frame -- WeaponFix in Config.hpp,
// resolved by weapon_fix_for() and written by wpnfix_set() below.
//
// WHICH ONE A PRESS WRITES IS DECIDED BY WHICH DRIVER OWNS THE WEAPON, not by a second hotkey.
// palettearm_weapon_owns() is the same flag that makes the legacy mesh drive stand down, so the two
// destinations are mutually exclusive by construction rather than by the player remembering which
// key belongs to which driver -- and a press under the palette driver can no longer leave a
// meaningless wpnoff line behind that would surface later if they switched back.
//
// BOTH LIVE IN THE SAME MACHINE-OWNED halo_vr_weapons.cfg, which is created on the FIRST capture
// and never shipped (package.ps1 forbids it from the zip, exactly as it does halo_vr_calib*.cfg).
// That is what makes a captured trim survive an update -- and what makes DELETING the file a clean
// revert: it is parsed last, so removing it leaves the shipped per-weapon baseline in halo_vr.cfg
// standing on its own.

#pragma once

#include <string>

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

// The COMBINED calibration hold (END, menu mode, or the per-weapon key), published so other modules
// can stand down while a gesture owns the weapon. Plugin.cpp's own flag is in an anonymous namespace
// and cannot be reached from another translation unit.
void calib_hold_publish(bool on);
bool calib_hold_active();

// Poll the per-weapon key. Called from the same block that polls calib_key, so both gestures are
// edge-detected on one thread rather than two.
// The held weapon's short key ("FP_Magnum"), implemented in WeaponCalib.cpp. Declared here for
// Holster.cpp, which renders the magazine for the weapon actually in hand.
std::string weapon_key();
void wpn_calib_poll();

// Called at the point the solve has finished and g_cfg holds the freshly fitted values. Returns
// TRUE if this capture was claimed as a per-weapon one -- in which case the caller must NOT run
// the global write_calib_file(), because the global calibration is meant to be left alone.
bool wpn_calib_capture();

// Load halo_vr_weapons.cfg into the wpnoff table. Called after the main config parse, so captured
// entries and hand-written ones end up in the same list.
void wpn_calib_write_file();

void wpn_calib_load();

// The weapon in hand, as both tables key it: the weapon actor's class name with the BP_ prefix and
// the _WeaponActor_C suffix stripped; empty if none.
//
// Costs one reflected call, so it is for the CAPTURE EDGE only -- a per-tick consumer wants
// weapon_offset_current_class(), which is already published.
std::string weapon_key();

// PALETTE per-weapon rigid delta (Config::WeaponFix): the entry whose match is a substring of the
// key, or nullptr. LAST match wins, the same rule weapon_fix_for() applies, so a captured entry
// outranks the shipped baseline it sits behind in the table. Consumed by BlamPalette.cpp.
const WeaponFix* wpnfix_find(const std::string& key);

// Store a captured palette rigid delta for `key` and rewrite halo_vr_weapons.cfg.
//
// Replace-or-append AMONG THE CAPTURED ENTRIES ONLY. The in-memory table also holds the shipped
// baseline parsed from halo_vr.cfg, and overwriting one of those would copy it into the player's
// file -- where it would then outlive the shipped value it was a copy of, and silently defeat any
// future update to the baseline. See the `captured` flag on WeaponFix.
void wpnfix_set(const std::string& key, const float q[4], const float t[3]);

// Store a captured SUPPORT-HAND GRIP OFFSET (gun frame, game cm) for `key`, and rewrite the file.
// Same captured-entries-only rule as wpnfix_set -- see its note.
void wpngrip_set(const std::string& key, float off_y, float off_z, float at_x);

// Drop `key`'s captured grip offset. false = there was none. Rewrites halo_vr_weapons.cfg.
bool wpngrip_clear(const std::string& key);

// Consume the "the per-weapon key was just released" latch.
//
// The palette freeze path takes it, so the rig-path capture cannot ALSO claim the same press.
// Without this a single INSERT under the palette driver would write both a wpnfix and a wpnoff.
bool wpn_calib_take_pending();

// Claim the release that is about to run as a PER-WEAPON capture.
//
// Set from the calibration edge in Plugin.cpp when the armed mode was 4, for the same reason
// wpn_calib_poll() latches it on the Insert key's falling edge: by the time wpn_calib_capture()
// runs there is no held key or armed mode left to read. Setting it is what makes the solve land in
// this weapon's delta instead of the global fit.
void wpn_calib_set_pending();

// Drop the equipped weapon's per-weapon POSE delta -> back to the global fit. false = nothing to
// clear. Rewrites halo_vr_weapons.cfg.
bool wpnoff_clear_current();

} // namespace halo
