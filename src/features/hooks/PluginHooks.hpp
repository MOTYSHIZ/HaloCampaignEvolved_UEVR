#pragma once

// HOOK POINTS IN Plugin.cpp. Each is called from exactly one place, named here with its thread.
// Definitions: src/features/FeatureList.cpp.

#include "Math.hpp"                    // Vec3
#include "core/fixes/TickStage.hpp"
#include "core/host/PluginState.hpp"   // HALO_PLUGIN_STATE_BRIDGE
#include "uevr/API.h"                  // UEVR_Vector3f

struct _XINPUT_STATE;

namespace halo {

// on_xinput_get_state (the XInput hook's thread), right after the raw button note below: the RAW pad,
// before any of the plugin's own swallowing, rebinding and synthetic presses. The player's fire input
// is noted first (core/FireInput), then the features' slots run and may modify the pad.
void features_xinput_raw_pad(_XINPUT_STATE* state);

// on_xinput_get_state (the XInput hook's thread), right after reload_note_buttons() and before
// features_xinput_raw_pad: the RAW pad buttons, before any remapping and before the throw press mask
// is composed in the same poll.
void features_xinput_note_buttons(unsigned short buttons);

// update() (game thread), right after blam_aim_tick() and before aim_watch_tick().
void features_game_tick_after_blam_aim();

// on_xinput_get_state (the XInput hook's thread), right after the calibration menu's trigger eat and
// before the control remapping.
void features_xinput_after_calib_trigger(_XINPUT_STATE* state);

// on_xinput_get_state (the XInput hook's thread), right before the vehicle hard brake's pad-side
// delivery, after the movement rotation and the d-pad shift.
void features_xinput_before_brake(_XINPUT_STATE* state);

// update() (game thread), right after scope_frame_end(tick).
void features_game_tick_late();

// update() (game thread), where the physical scope's tick has always run: after the second
// weapon_offset_update() and palettewpn's per-tick block. dt is g_last_dt, whose only writer is
// on_pre_engine_tick, before update() runs on the same thread.
void features_game_tick_after_offsets(float dt);

// update() (game thread), in the stale rig guard, right after it drops the rig and its parent.
void features_rig_lost();

// update() (game thread), right after the stale rig guard, before the HMD translation leash block.
void features_game_tick_before_leash();

// THE HMD TRANSLATION LEASH BLOCK in update() (game thread).
//   features_leash_block_wanted: in the block's gate, after `g_cfg.hmd_leash ||`.
//   features_hmd_pose_plausible: in the pose condition, after get_pose succeeded (core fix).
//   features_leash_lateral: the statement right before the author's lateral leash; true = skip his.
//   features_leash_vertical: the statement right before the author's vertical leash; true = skip his.
bool features_leash_block_wanted();
bool features_hmd_pose_plausible(const Vec3& hp);
bool features_leash_lateral(const Vec3& hp, float& nx, float& ny, float& nz, bool& moved);
bool features_leash_vertical(const Vec3& hp, const UEVR_Vector3f& so, float& ny, bool& moved);

// update() (game thread), right after the HMD translation leash block, before the calibrate key.
void features_game_tick_after_leash();

// on_pre_calculate_stereo_view_offset (render thread), inside `position != nullptr`, right after
// g_have_view_pos is set and before aim_converge_note_pre: the body eye.
void features_stereo_pre_eye(int index, UEVR_Vector3f* position, bool is_double);

// on_pre_calculate_stereo_view_offset (render thread), in the once-per-frame render pass (index 0),
// first, before the reload display tick and the marker re-anchor.
void features_render_frame();

// on_post_calculate_stereo_view_offset (render thread), right after the STOMPLOG sample and before
// aim_converge_note_post: the head offset, and a feature's clamp of the rendered eye.
void features_stereo_post_eye(int index, UEVR_Vector3f* position, bool is_double);

// update() (game thread), inside palettewpn's per-tick block, after the rendered-hand pose publish and
// before the parent frame measurement: the vehicle body work, the wheel and the heading.
void features_game_tick_vehicle();

// on_pre_calculate_stereo_view_offset (render thread), inside `position != nullptr`, right after
// aim_converge_note_pre, last in that block: the seat camera.
void features_stereo_pre_eye_seat(int index, UEVR_Vector3f* position, UEVR_Rotatorf* rotation, bool is_double);

// on_pre_calculate_stereo_view_offset (render thread), after the render-rate rig re-apply, the
// statement right before `if (rotation == nullptr || !g_cfg.view_lock) return;`. True = return now
// (the in-vehicle view set the rotation).
bool features_stereo_view_override(UEVR_Rotatorf* rotation, bool is_double);

// on_post_calculate_stereo_view_offset (render thread), inside `position != nullptr`, right after
// aim_converge_note_post and before the eye position publish: the rendered eye.
void features_stereo_post_eye_rendered(int index, float ex, float ey, float ez);

} // namespace halo
