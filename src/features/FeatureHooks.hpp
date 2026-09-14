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

struct FeatureHooks {
    // The feature's master key in the feature registry (Features.cpp).
    const char* key;

    // features_parse_key: a cfg key none of the author's parsers took. True = taken.
    bool (*parse_key)(const char* key, const char* val, double v);

    // features_game_tick_late: the late per-tick work.
    void (*game_tick_late)();
};

} // namespace halo
