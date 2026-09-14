#include "features/FeatureHooks.hpp"
#include "features/hooks/ConfigHooks.hpp"
#include "features/hooks/PluginHooks.hpp"

#include "core/FireInput.hpp"

namespace halo {

// Every feature's hooks table, each defined in its own folder.
extern const FeatureHooks kForceTubeHooks;

namespace {

// THE ORDER. At every hook point the features run in this order. It is chosen so that each hook
// point keeps the relative order its features had when they were textual fragments of the author's
// files; a hook point whose original order no list order can satisfy gets separate slots instead.
const FeatureHooks* const kFeatureList[] = {
    &kForceTubeHooks,
};

} // namespace

bool features_parse_key(const char* key, const char* val, double v) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->parse_key != nullptr && f->parse_key(key, val, v)) return true;
    return false;
}

void features_xinput_raw_pad(_XINPUT_STATE* state) {
    fire_input_note_pad(state);
}

void features_game_tick_late() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->game_tick_late != nullptr) f->game_tick_late();
}

} // namespace halo
