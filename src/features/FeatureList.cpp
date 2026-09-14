#include "features/FeatureHooks.hpp"
#include "features/hooks/ConfigHooks.hpp"
#include "features/hooks/PluginHooks.hpp"
#include "features/hooks/ScopeHooks.hpp"

#include "core/EyeTrace.hpp"
#include "core/FireInput.hpp"

namespace halo {

// Every feature's hooks table, each defined in its own folder.
extern const FeatureHooks kForceTubeHooks;
extern const FeatureHooks kScopeLensHooks;
extern const FeatureHooks kHeadBlockHooks;
extern const FeatureHooks kGrenadeSwallowHooks;

namespace {

// THE ORDER. At every hook point the features run in this order. It is chosen so that each hook
// point keeps the relative order its features had when they were textual fragments of the author's
// files; a hook point whose original order no list order can satisfy gets separate slots instead.
const FeatureHooks* const kFeatureList[] = {
    &kForceTubeHooks,
    &kScopeLensHooks,
    &kHeadBlockHooks,
    &kGrenadeSwallowHooks,
};

} // namespace

bool features_parse_key(const char* key, const char* val, double v) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->parse_key != nullptr && f->parse_key(key, val, v)) return true;
    return false;
}

void features_xinput_raw_pad(_XINPUT_STATE* state) {
    fire_input_note_pad(state);
    for (const FeatureHooks* f : kFeatureList)
        if (f->xinput_raw_pad != nullptr) f->xinput_raw_pad(state);
}

void features_game_tick_late() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->game_tick_late != nullptr) f->game_tick_late();
}

void features_game_tick_after_offsets(float dt) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->game_tick_after_offsets != nullptr) f->game_tick_after_offsets(dt);
}

void features_rig_lost() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->rig_lost != nullptr) f->rig_lost();
}

bool features_scope_trigger_stood_down(bool& s_down) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->scope_trigger_stood_down != nullptr && f->scope_trigger_stood_down(s_down)) return true;
    return false;
}

bool features_scope_pane_stands_down() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->scope_pane_stands_down != nullptr && f->scope_pane_stands_down()) return true;
    return false;
}

void features_game_tick_after_leash() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->game_tick_after_leash != nullptr) f->game_tick_after_leash();
}

void features_stereo_pre_eye(int index, UEVR_Vector3f* position, bool is_double) {
    eye_note_pre_view(index, position, is_double);
}

void features_stereo_post_eye(int index, UEVR_Vector3f* position, bool is_double) {
    const HeadClamp* clamp = nullptr;
    for (const FeatureHooks* f : kFeatureList)
        if (f->head_clamp != nullptr) { clamp = f->head_clamp; break; }
    eye_note_post_view(index, position, is_double, clamp);
}

} // namespace halo
