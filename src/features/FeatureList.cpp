#include "features/FeatureHooks.hpp"
#include "features/hooks/BlamDriveHooks.hpp"
#include "features/hooks/ConfigHooks.hpp"
#include "features/hooks/GestureHooks.hpp"
#include "features/hooks/MarkersHooks.hpp"
#include "features/hooks/PluginHooks.hpp"
#include "features/hooks/ReticuleHooks.hpp"
#include "features/hooks/ScopeHooks.hpp"
#include "features/hooks/UnitStateHooks.hpp"

#include "Config.hpp"
#include "core/EyeTrace.hpp"
#include "core/FireInput.hpp"
#include "core/MarkerFaces.hpp"
#include "core/UnitState.hpp"
#include "core/fixes/HmdPoseGate.hpp"
#include "core/fixes/ReticuleFixes.hpp"

#include <string>

namespace halo {

// Every feature's hooks table, each defined in its own folder.
extern const FeatureHooks kForceTubeHooks;
extern const FeatureHooks kScopeLensHooks;
extern const FeatureHooks kHeadBlockHooks;
extern const FeatureHooks kGrenadeSwallowHooks;
extern const FeatureHooks kRoomscaleHooks;
extern const FeatureHooks kHeightCalHooks;
extern const FeatureHooks kWristHudHooks;
extern const FeatureHooks kVehCamHooks;
extern const FeatureHooks kMeleeLeftHooks;

namespace {

// THE ORDER. At every hook point the features run in this order. It is chosen so that each hook
// point keeps the relative order its features had when they were textual fragments of the author's
// files; a hook point whose original order no list order can satisfy gets separate slots instead.
//   game_tick_late: forcetube, then wristhud (their calls were in that order in update()).
const FeatureHooks* const kFeatureList[] = {
    &kForceTubeHooks,
    &kScopeLensHooks,
    &kHeadBlockHooks,
    &kGrenadeSwallowHooks,
    &kRoomscaleHooks,
    &kHeightCalHooks,
    &kWristHudHooks,
    &kVehCamHooks,
    &kMeleeLeftHooks,
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

void features_xinput_after_calib_trigger(_XINPUT_STATE* state) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->xinput_after_calib_trigger != nullptr) f->xinput_after_calib_trigger(state);
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

void features_render_frame() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->render_frame != nullptr) f->render_frame();
}

void features_stereo_post_eye(int index, UEVR_Vector3f* position, bool is_double) {
    const HeadClamp* clamp = nullptr;
    for (const FeatureHooks* f : kFeatureList)
        if (f->head_clamp != nullptr) { clamp = f->head_clamp; break; }
    eye_note_post_view(index, position, is_double, clamp);
}

void features_game_tick_before_leash() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->game_tick_before_leash != nullptr) f->game_tick_before_leash();
}

bool features_leash_block_wanted() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->leash_block_wanted != nullptr && f->leash_block_wanted()) return true;
    return false;
}

bool features_hmd_pose_plausible(const Vec3& hp) {
    return hmd_pose_plausible(hp);
}

// Every slot runs (their bookkeeping runs whenever the block does). With the block entered only for
// a feature (hmdleash off), the author's leash must not run either.
bool features_leash_lateral(const Vec3& hp, float& nx, float& ny, float& nz, bool& moved) {
    bool owned = false;
    for (const FeatureHooks* f : kFeatureList)
        if (f->leash_lateral != nullptr && f->leash_lateral(hp, nx, ny, nz, moved)) owned = true;
    return owned || !g_cfg.hmd_leash;
}

bool features_leash_vertical(const Vec3& hp, const UEVR_Vector3f& so, float& ny, bool& moved) {
    bool owned = false;
    for (const FeatureHooks* f : kFeatureList)
        if (f->leash_vertical != nullptr && f->leash_vertical(hp, so, ny, moved)) owned = true;
    return owned || !g_cfg.hmd_leash;
}

void features_xinput_before_brake(_XINPUT_STATE* state) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->xinput_before_brake != nullptr) f->xinput_before_brake(state);
}

void features_sim_stick_mode_hold(bool off_thread) {
    unit_state_stick_mode_publish(off_thread);
}

void features_sim_record_ready(uintptr_t rec, bool off_thread) {
    unit_state_record_ready(rec, off_thread);
}

void features_sim_unit_state_radar(uintptr_t obj) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->sim_unit_state_radar != nullptr) f->sim_unit_state_radar(obj);
}

void features_sim_unit_state_end(uintptr_t obj) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->sim_unit_state_end != nullptr) f->sim_unit_state_end(obj);
}

bool features_menu_command(const std::string& line) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->menu_command != nullptr && f->menu_command(line)) return true;
    return false;
}

std::string features_menu_status_line() {
    for (const FeatureHooks* f : kFeatureList) {
        if (f->menu_status_line == nullptr) continue;
        std::string s = f->menu_status_line();
        if (!s.empty()) return s;
    }
    return std::string();
}

bool features_asset_load_recently_failed(const char* path) {
    return load_asset_recently_failed(path);
}

const char* features_asset_load_suffix(uevr::API::UObject* obj) {
    return load_asset_log_suffix(obj);
}

void features_asset_load_done(const char* path, uevr::API::UObject* obj) {
    load_asset_note_result(path, obj);
}

bool features_widget_log() {
    return widget_log_enabled();
}

float features_widget_tint_mul() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->widget_tint_mul != nullptr) return f->widget_tint_mul();
    return 1.0f;
}

bool features_widget_alpha_hide_applies(uevr::API::UObject* comp) {
    return widget_alpha_hide_applies(comp);
}

void features_reticule_widget_moved() {
    reticule_widget_moved();
}

void features_game_tick_vehicle() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->game_tick_vehicle != nullptr) f->game_tick_vehicle();
}

void features_stereo_pre_eye_seat(int index, UEVR_Vector3f* position, UEVR_Rotatorf* rotation, bool is_double) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->stereo_pre_eye_seat != nullptr) f->stereo_pre_eye_seat(index, position, rotation, is_double);
}

bool features_stereo_view_override(UEVR_Rotatorf* rotation, bool is_double) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->stereo_view_override != nullptr && f->stereo_view_override(rotation, is_double)) return true;
    return false;
}

void features_stereo_post_eye_rendered(int index, float ex, float ey, float ez) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->stereo_post_eye_rendered != nullptr) f->stereo_post_eye_rendered(index, ex, ey, ez);
}

void features_gesture_melee_offhand(float dt) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->gesture_melee_offhand != nullptr) f->gesture_melee_offhand(dt);
}

bool features_room_to_world(const Vec3& room, const Vec3& hmd_room, Vec3* out) {
    return room_to_world_anchored(room, hmd_room, out);
}

} // namespace halo
