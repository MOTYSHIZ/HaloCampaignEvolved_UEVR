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
};

} // namespace halo
