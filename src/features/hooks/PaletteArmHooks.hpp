#pragma once

// HOOK POINTS IN palettearm/PaletteHook.cpp AND IN ANY OTHER DETOUR OF THE FIRST-PERSON POSE BUILDER.
// Definitions: src/features/FeatureList.cpp.
//
// The builder runs on the sim thread, which no game-thread timer can see. These two calls bracket a
// detour's own post-pass (never the game's builder) so its cost is a number, measured the same way
// whichever arm driver installed the detour. While the driver probe is off (core/dev/DriverProbe)
// begin returns 0 and end returns at once.

#include "palettearm/PaletteMath.hpp"   // Mat3, Vec3: the weapon target and the wrist a slot may shape
#include "core/fixes/RenderTime.hpp"      // PaDriveDone

namespace halo {

// hooked_build (the sim thread), right after the drive pointer is taken and before the palette is derived.
long long features_pose_hook_begin();

// hooked_build (the sim thread), the last statement. arm_driver_mode = the mode whose detour this is.
void features_pose_hook_end(long long t0, int arm_driver_mode);

// ---- THE POSE BUILD'S OWN SLOTS (the SIM THREAD). Everything below runs inside drive_palette, so
// it runs only while armdriver 2 owns the arms; no other mode reaches any of it.
//
// WHERE THEY SIT, and why the order is the whole design. A feature that shapes the pose must come
// in BEFORE the author's own holds, never after: rigcarryrot went in after his sprint hold and
// divided by the held pose, so his hold was re-applied inverted and the gun swung more than 30
// degrees (removed, e6c236c). Both weapon slots below are upstream of his per-action hold block
// and of the carry that follows it, and the gate slot may only RAISE a weight -- so his hold reads
// our raise as one more gate of his own, and nothing downstream ever sees its own output again.

// drive_palette, immediately after the author folds his melee / equip / sprint gates into the
// join / off / stock / hold weights, and before the first line that reads them. A slot may RAISE a
// weight and nothing else: raising is the only edit that composes (max), so two slots, and his own
// gates, never undo each other. With no slot enabled the four come back untouched.
void features_pa_anim_gates(bool is_capture_bank, float& join_w, float& off_w, float& stock_w_all,
                            float& hold_w);

// drive_palette, route (d), right after the rig weapon target is read out of its seqlock and
// before anything consumes it. A slot may replace the target the base mod's weapon carry will
// drive the gun onto. Upstream of his rest-pose hold and of the carry, so a replacement is an
// ordinary target as far as the rest of the route is concerned. `have_rt` false = there is no
// target this build and a slot must leave both alone.
void features_pa_rig_target(bool is_capture_bank, bool have_rt, palettearm::Mat3& basis,
                            palettearm::Vec3& position);

// drive_palette, the per-arm placement pass, bracketing the author's "the aim hand rides the gun"
// block. The NOTE call hands a slot the wrist target and rotation the player's controller alone
// asked for, before he carries them onto the gun; the KEEP call, the statement after that block,
// lets a slot bring the finished wrist back toward what it was handed. A slot that wants nothing
// leaves both untouched, which is his code exactly.
void features_pa_aim_wrist_note(bool is_aim, bool is_capture_bank,
                                const palettearm::Vec3& wrist_target,
                                const palettearm::Mat3& desired_wrist);
void features_pa_aim_wrist_keep(bool is_aim, bool is_capture_bank, palettearm::Vec3& wrist_target,
                                palettearm::Mat3& desired_wrist);

// drive_palette, the end of a successful LIVE solve: the frame the build maps controller poses
// through, and the pose each group was placed with. Read only.
void features_pa_drive_done(const PaDriveDone& done);
// drive_palette, a render bank has just copied the live solve (the bank mirror). Read only.
void features_pa_bank_mirrored(palettearm::BlamMatrix4x3* bank, std::uint32_t node_count,
                               std::int32_t model_tag, std::int32_t weapon_slot, std::uint8_t bank_index);

} // namespace halo
