#pragma once

// ONE INTERFACE FOR EVERY FORK FEATURE.
//
// Each feature under src/features/<feature>/ defines exactly one FeatureHooks table and fills only
// the slots it uses. src/features/FeatureList.cpp lists every table in one explicit order and
// defines the dispatchers the author's files call at their hook points (declared per host file in
// src/features/hooks/). A dispatcher runs the non-null slots in list order, so the list order is the
// relative order of features at every hook point more than one feature uses.
//
// A slot runs on the thread and at the point its dispatcher is called from. The dispatcher
// declarations name both, and each slot below names its dispatcher.
//
// Tables hold function addresses only and are constant-initialised (constinit at the definition),
// so there is no static-initialisation order between features, the list and the dispatchers.

#include "Math.hpp"       // Vec3
#include "uevr/API.h"     // UEVR_Vector3f

#include <cstdint>
#include <string>

struct _XINPUT_STATE;

namespace halo {

struct HeadClamp;   // core/EyeTrace.hpp
enum class HolsterSlot : int;   // Holster.hpp

struct FeatureHooks {
    // The feature's master key in the feature registry (Features.cpp).
    const char* key;

    // features_parse_key: a cfg key none of the author's parsers took. True = taken.
    bool (*parse_key)(const char* key, const char* val, double v);

    // features_game_tick_late: the late per-tick work.
    void (*game_tick_late)();

    // features_game_tick_after_offsets: per-tick work that follows the per-weapon offsets, with the
    // tick's dt.
    void (*game_tick_after_offsets)(float dt);

    // features_rig_lost: the stale rig guard just dropped the rig component and its parent.
    void (*rig_lost)();

    // features_scope_trigger_stood_down: true = the feature owns the scope, so the pane's left
    // trigger toggle stands down (the slot clears the edge state it is handed and the pane flag).
    bool (*scope_trigger_stood_down)(bool& s_down);

    // features_scope_pane_stands_down: true = the feature owns the scope, so the pane's bound
    // button toggle stands down.
    bool (*scope_pane_stands_down)();

    // features_game_tick_after_leash: per-tick work right after the HMD translation leash block.
    void (*game_tick_after_leash)();

    // features_stereo_post_eye: a clamp on the rendered eye, handed to the head-offset measurement
    // (core/EyeTrace.hpp). The first non-null clamp in list order is used.
    const HeadClamp* head_clamp;

    // features_xinput_raw_pad: the raw pad at the top of the XInput hook, after the fire input note.
    // The slot may modify the pad.
    void (*xinput_raw_pad)(_XINPUT_STATE* state);

    // features_game_tick_before_leash: per-tick work right before the HMD translation leash block.
    void (*game_tick_before_leash)();

    // features_leash_block_wanted: true = the feature needs the leash block to run even with the
    // author's leash (hmdleash) off.
    bool (*leash_block_wanted)();

    // features_leash_lateral: runs where the author's lateral leash is, with the plausible HMD pose
    // and the origin being built. True = the feature applied the lateral leash itself.
    bool (*leash_lateral)(const Vec3& hp, float& nx, float& ny, float& nz, bool& moved);

    // features_leash_vertical: runs where the author's vertical leash is. True = the feature owns the
    // origin's Y this tick (whether or not it moved it), so the vertical leash must not run.
    bool (*leash_vertical)(const Vec3& hp, const UEVR_Vector3f& so, float& ny, bool& moved);

    // features_xinput_before_brake: the pad in the XInput hook, right before the vehicle hard brake.
    void (*xinput_before_brake)(_XINPUT_STATE* state);

    // features_sim_unit_state_end: the end of the sim-thread unit state publish, with the unit.
    void (*sim_unit_state_end)(uintptr_t obj);

    // features_menu_command: one line of the settings menu's command file. True = handled.
    bool (*menu_command)(const std::string& line);

    // features_menu_status_line: the feature's line for the menu status file; empty = none.
    std::string (*menu_status_line)();

    // features_render_frame: once per rendered frame (stereo pre, eye 0), in the render pass.
    void (*render_frame)();

    // features_xinput_after_calib_trigger: the pad in the XInput hook, right after the calibration
    // menu's trigger eat.
    void (*xinput_after_calib_trigger)(_XINPUT_STATE* state);

    // features_sim_unit_state_radar: the sim-thread unit state publish, right after the throw dump
    // probe, with the unit.
    void (*sim_unit_state_radar)(uintptr_t obj);

    // features_widget_tint_mul: the gain multiplier the tint call in progress on this thread asked
    // for. The first non-null slot answers.
    float (*widget_tint_mul)();

    // features_game_tick_vehicle: the per-tick vehicle work, in palettewpn's per-tick block.
    void (*game_tick_vehicle)();

    // features_stereo_pre_eye_seat: the stereo pre callback per eye, after the aim convergence note,
    // with the engine's camera position and rotation. The slot may move the camera.
    void (*stereo_pre_eye_seat)(int index, UEVR_Vector3f* position, UEVR_Rotatorf* rotation, bool is_double);

    // features_stereo_view_override: the stereo pre callback per eye, before the view lock gate. True =
    // the slot set the view rotation itself and the callback returns. Handed a null rotation too.
    bool (*stereo_view_override)(UEVR_Rotatorf* rotation, bool is_double);

    // features_stereo_post_eye_rendered: the stereo post callback per eye, with the rendered eye, after
    // the aim convergence note.
    void (*stereo_post_eye_rendered)(int index, float ex, float ey, float ez);

    // features_gesture_melee_offhand: the melee half of the gesture tick, before the aim hand's
    // detector, with the tick's dt.
    void (*gesture_melee_offhand)(float dt);

    // features_xinput_note_buttons: the raw pad buttons in the XInput hook, before remapping.
    void (*xinput_note_buttons)(unsigned short buttons);

    // features_game_tick_after_blam_aim: per-tick work right after the dev aim hook's tick.
    void (*game_tick_after_blam_aim)();

    // features_holster_reset: the holster reset (level transition, menu, stand-down).
    void (*holster_reset)();

    // features_holster_mesh_sweep_period: the grenade mesh sweep period in ticks. The first non-null
    // slot answers.
    unsigned (*holster_mesh_sweep_period)();

    // features_holster_mesh_swept: a grenade mesh sweep finished; mf = the frag mesh (null = none).
    void (*holster_mesh_swept)(const void* mf);

    // features_holster_before_release: the holster tick right before its release edge, with the pouch
    // each hand is in and the hand and head poses.
    void (*holster_before_release)(HolsterSlot zone_g, HolsterSlot zone_p,
                                   const Vec3& pos, const Vec3& gpos, const Vec3& hpos);

    // features_sim_unit_state_grenades: the sim-thread unit publish, right after the unit evidence.
    void (*sim_unit_state_grenades)(uintptr_t obj);

    // features_sim_unit_state_after_radar: the sim-thread unit publish, right after the radar scan.
    void (*sim_unit_state_after_radar)(uintptr_t obj);

    // features_blam_create_before: the dev create_projectile hook, before the original call.
    void (*blam_create_before)(uintptr_t params);

    // features_blam_create_after: the dev create_projectile hook, after the original call, with its
    // return value.
    void (*blam_create_after)(uintptr_t params, uintptr_t cret);

    // features_holster_pouch_offhand: the holster tick's pouch loop, the off hand against one pouch.
    void (*holster_pouch_offhand)(bool ghand_ok, const Vec3& ghand, const Vec3& pouch);

    // features_holster_pouches_measured: the holster tick, right after the pouch loop.
    void (*holster_pouches_measured)();

    // features_gesture_reset: gesture_reset, after the reload engine's reset and before the melee reset.
    void (*gesture_reset)();

    // features_rig_resolve_wanted: the rig resolve gate in update(). True = resolve for this feature.
    bool (*rig_resolve_wanted)();

    // features_sim_record_written: drive_angles_impl (SIM THREAD, ~2600 calls/s), right after the aim
    // convergence bends the angles about to be written to the Blam control record.
    void (*sim_record_written)(float yaw, float pitch);

    // ---- RUNTIME STATE (every table fills these).
    // Whether the feature is enabled right now: its master key(s), read from g_cfg. Any thread.
    bool (*enabled)();
    // The core services the feature consumes (core/Services.hpp). A service runs while any enabled
    // feature declares it.
    uint32_t services;
    // Game thread, once, right after a config reload turned the feature off: release everything it
    // holds (overrides, markers, latches) so the author's code runs as if it had never been on.
    void (*released)();
};

} // namespace halo
