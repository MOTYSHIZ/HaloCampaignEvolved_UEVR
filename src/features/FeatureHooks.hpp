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
};

} // namespace halo
