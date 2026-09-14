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

// update() (game thread), right after the periodic load_config(): features whose master key went off
// in this reload release what they hold, and a changed feature state is logged.
void features_config_loaded();

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

// ---- STABILITY FIXES (core/fixes/HostFixes; each a no-op while its service is inactive)
// update() (game thread), its first statement.
void features_tick_begin();
// The engine tick's exception filter, where the fault is counted.
void features_tick_faulted();
// update() (game thread), in the rig resolve, right after the attach parent is taken.
void features_rig_parent_resolved();
// update() (game thread), right before features_game_tick_before_leash.
void features_stale_rig_guard();
// A tick stage marker, at the author's stage boundaries (game thread).
void features_tick_stage(const char* stage);
// report_tick_fault: the " stage 'x'" part of the fault line ("" while inactive).
const char* features_tick_fault_stage();
// update() (game thread), the nav lane call: true = the lane ran under the fault quarantine.
bool features_nav_world_guarded(bool engaged, uint32_t tick);
// find_ui_manager (game thread), right after the cached hit: true = skip this miss's sweep.
bool features_ui_manager_miss_throttled();
// reticle_rescan (game thread), hud_follow's term of the sweep need (true while inactive).
bool features_reticle_rescan_follow(bool hud_hide, int reticle_count);
// hud_reticle_follow (game thread): the hide pass, a dead hosted crosshair, and the pass end
// (true = forget the crosshairs so the sweep finds the rebuilt one).
void features_reticle_hide_begin();
void features_reticle_hide_dead();
bool features_reticle_hide_end();
// update() (game thread), right after the nav lane block, before the enabled early-out.
void features_xrlayer_early(uint32_t tick);
// update() (game thread), the stick-mode detector, right after the force overrides decide `want`.
void features_stick_mode_want(bool want);
// update() (game thread), the stick-mode exit: true = re-anchored after a death (skips the fold).
bool features_stick_exit_after_death();
// update() (game thread), the turning block: a flick a gate swallowed, and a snap that landed.
void features_turn_gate_note(bool fp_control_now);
void features_turn_snap_note(float step);
// Teardown (the thread UEVR tears down on): right after the teardown line, and after the game
// settings restore.
void features_teardown_early();
void features_teardown_restore();
// update() (game thread), the weapon rig block's resolve gate beside rig_enabled: true = the FP weapon-actor
// route and the rig component are resolved for a feature or the reload engine. The palette weapon needs it
// (the stick-mode detector's rigcomp, the mesh constants its pullback stands on, the pivot latch: booted
// with rig=0 the route never resolved and the palette weapon silently never applied). The reload engine
// needs it under every arm driver mode, including rig=0 with the author's palettearm route: its per-weapon
// state saves on a weapon swap, seen through this resolve. The resolve only reads; the rig WRITES stay in
// the rig_enabled block.
bool features_rig_resolve_wanted();
// the on-foot reticule's compositor publish and the stereo post-callback: > 0 = a feature publishes the reticle
// at render rate in that mode (the tick publish stands down).
int features_reticule_render_publish_mode();
// update(), right after the shipping Blam aim write.
void features_game_tick_after_blam_drive();
// update(), after the per-weapon delta stage marker, before the vehicle hook.
void features_game_tick_before_vehicle();
// update(), right after the vehicle hook.
void features_game_tick_after_vehicle(uint32_t tick);
// the engine tick, right after gesture_update.
void features_game_tick_after_gestures(float dt);
// the engine tick callback, after the start stage marker.
void features_engine_tick_start();
// the engine tick callback, after the hitch report.
void features_engine_tick_end();
// the post-engine tick callback.
void features_post_engine_tick();
// the stick-mode detector: true = a first-person weapon is positively rendered.
bool features_fp_weapon_live();
// the PlayerController change, right after the rig parent is dropped.
void features_rig_parent_dropped();
// update(), the rig driver gate beside rig_enabled: true = the driver stands down.
bool features_rig_driver_stood_down();
// the stereo post-callback, after the stamp publish.
void features_stereo_post_eye_late(int index);
// the XInput hook's aim law, before derive_ctrl_angles.
void features_aim_law_sampling();
// the XInput hook's aim law, after the aim read.
void features_aim_law_sampled(double ay, double ap);
// the stereo pre-callback, after the view lock's rotation publish: the rendered camera publish (core), the
// palette instruments, the per-frame render pass (eye 0) and the meters.
void features_stereo_pre_eye_rendered(int index);
// on_initialize, beside the scope blit registration: render callbacks the fork's dev tools register.
void features_render_callbacks_register();

} // namespace halo
