#include "features/FeatureHooks.hpp"
#include "features/FeatureList.hpp"
#include "features/hooks/BlamAimHooks.hpp"
#include "features/hooks/BlamDriveHooks.hpp"
#include "features/hooks/ConfigHooks.hpp"
#include "features/hooks/GestureHooks.hpp"
#include "features/hooks/HolsterHooks.hpp"
#include "features/hooks/MarkersHooks.hpp"
#include "features/hooks/PluginHooks.hpp"
#include "features/hooks/ReticuleHooks.hpp"
#include "features/hooks/ArmDriverHooks.hpp"
#include "features/hooks/ArmsHooks.hpp"
#include "features/hooks/MotionAimHooks.hpp"
#include "features/hooks/ScopeHooks.hpp"
#include "features/hooks/TwoHandHooks.hpp"
#include "features/hooks/UnitStateHooks.hpp"

#include "Config.hpp"
#include "core/Services.hpp"
#include "uevr/API.hpp"
#include "core/CameraBob.hpp"
#include "core/CoreKeys.hpp"
#include "core/config/KeyAlias.hpp"
#include "core/EyeTrace.hpp"
#include "core/HiddenReload.hpp"
#include "core/FireInput.hpp"
#include "core/MarkerFaces.hpp"
#include "core/dev/CutsceneDump.hpp"
#include "core/PalettePose.hpp"
#include "core/UnitState.hpp"
#include "core/WeaponObject.hpp"
#include "core/reload/ReloadEngine.hpp"
#include "core/fixes/HmdPoseGate.hpp"
#include "core/fixes/MeleeInstruments.hpp"
#include "core/fixes/ReticuleFixes.hpp"
#include "core/fixes/HostFixes.hpp"
#include "MotionAimControl.hpp"   // apply_aim_fix

#include <span>
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
extern const FeatureHooks kHolsterPollThrowHooks;
extern const FeatureHooks kReloadVrHooks;
extern const FeatureHooks kSlideVrHooks;
extern const FeatureHooks kPaletteWpnHooks;
extern const FeatureHooks kAimBoreHooks;
extern const FeatureHooks kAimReticuleStampHooks;
extern const FeatureHooks kStabilityFixesHooks;

namespace {

// THE ORDER. At every hook point the features run in this order. It is chosen so that each hook
// point keeps the relative order its features had when they were textual fragments of the author's
// files; a hook point whose original order no list order can satisfy gets separate slots instead.
//   game_tick_late: forcetube, then wristhud (their calls were in that order in update()).
const FeatureHooks* const kFeatureListStorage[] = {
    &kForceTubeHooks,
    &kScopeLensHooks,
    &kHeadBlockHooks,
    &kGrenadeSwallowHooks,
    &kRoomscaleHooks,
    &kHeightCalHooks,
    &kWristHudHooks,
    &kVehCamHooks,
    &kMeleeLeftHooks,
    &kHolsterPollThrowHooks,
    &kReloadVrHooks,
    &kSlideVrHooks,
    &kPaletteWpnHooks,
    &kAimBoreHooks,
    &kAimReticuleStampHooks,
    &kStabilityFixesHooks,
    nullptr,   // end marker: keeps the array non-empty in a build with every feature folder removed
};
// The tables, without the end marker. A span, so a build with no feature at all still compiles and every
// dispatcher simply finds nothing to run.
const std::span<const FeatureHooks* const> kFeatureList{kFeatureListStorage, std::size(kFeatureListStorage) - 1};

} // namespace

bool features_parse_key(const char* key, const char* val, double v) {
    // A fork key renamed since it shipped arrives under its old name; translate once, here, so
    // every parser below sees only current names (core/config/KeyAlias.hpp).
    key = key_current_name(key);
    if (core_parse_key(key, val, v)) return true;
    for (const FeatureHooks* f : kFeatureList)
        if (f->parse_key != nullptr && f->parse_key(key, val, v)) return true;
    return false;
}

void features_xinput_raw_pad(_XINPUT_STATE* state) {
    if (service_active(SVC_FIRE_INPUT)) fire_input_note_pad(state);
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
    if (service_active(SVC_EYE_TRACE)) eye_note_pre_view(index, position, is_double);
}

void features_render_frame() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->render_frame != nullptr) f->render_frame();
    if (reload_engine_active()) gesture_render_tick();
}

void features_stereo_post_eye(int index, UEVR_Vector3f* position, bool is_double) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->stereo_post_eye_sample != nullptr && f->enabled != nullptr && f->enabled()) f->stereo_post_eye_sample(index);
    const HeadClamp* clamp = nullptr;
    for (const FeatureHooks* f : kFeatureList)
        if (f->head_clamp != nullptr && f->enabled != nullptr && f->enabled()) { clamp = f->head_clamp; break; }
    if (service_active(SVC_EYE_TRACE)) eye_note_post_view(index, position, is_double, clamp);
}

void features_game_tick_before_leash() {
    if (service_active(SVC_CAMERA_BOB)) camera_bob_tick();
    for (const FeatureHooks* f : kFeatureList)
        if (f->game_tick_before_leash != nullptr) f->game_tick_before_leash();
}

bool features_leash_block_wanted() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->leash_block_wanted != nullptr && f->leash_block_wanted()) return true;
    return false;
}

bool features_hmd_pose_plausible(const Vec3& hp) {
    return !service_active(SVC_LEASH_GATE) || hmd_pose_plausible(hp);
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
    if (service_active(SVC_SEAT)) unit_state_stick_mode_publish(off_thread);
}

void features_sim_record_ready(uintptr_t rec, bool off_thread) {
    if (service_active(SVC_UNIT_STATE)) unit_state_record_ready(rec, off_thread);
}

void features_sim_record_written(float yaw, float pitch) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->sim_record_written != nullptr && f->enabled != nullptr && f->enabled()) f->sim_record_written(yaw, pitch);
}

void features_sim_orientation_returned() {
    if (service_active(SVC_WEAPON_OBJECT)) weapon_object_offhook_tick();
}

void features_sim_unit_state_grenades(uintptr_t obj) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->sim_unit_state_grenades != nullptr) f->sim_unit_state_grenades(obj);
}

void features_sim_unit_state_after_radar(uintptr_t obj) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->sim_unit_state_after_radar != nullptr) f->sim_unit_state_after_radar(obj);
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
    return service_active(SVC_RETICULE_FIXES) && load_asset_recently_failed(path);
}

const char* features_asset_load_suffix(uevr::API::UObject* obj) {
    return service_active(SVC_RETICULE_FIXES) ? load_asset_log_suffix(obj) : "";
}

void features_asset_load_done(const char* path, uevr::API::UObject* obj) {
    if (service_active(SVC_RETICULE_FIXES)) load_asset_note_result(path, obj);
}

bool features_widget_log() {
    // Inactive: the author's probe log, every 32nd call, as he shipped it.
    return !service_active(SVC_STABILITY) || widget_log_enabled();
}

float features_widget_tint_mul() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->widget_tint_mul != nullptr) return f->widget_tint_mul();
    return 1.0f;
}

bool features_widget_alpha_hide_applies(uevr::API::UObject* comp) {
    return !service_active(SVC_WIDGET_HOSTS) || widget_alpha_hide_applies(comp);
}

void features_reticule_widget_moved() {
    if (service_active(SVC_STABILITY)) reticule_widget_moved();
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

void features_melee_hold_check() {
    if (service_active(SVC_MELEE_INSTRUMENTS)) melee_hold_check();
}

void features_melee_swing_moving(bool in_swing) {
    if (service_active(SVC_MELEE_INSTRUMENTS)) melee_swing_moving(in_swing);
}

bool features_melee_vetoed(long long now, float speed, float reach) {
    return service_active(SVC_MELEE_INSTRUMENTS) && melee_vetoed(now, speed, reach);
}

bool features_melee_fired(long long now, float speed, float reach) {
    return service_active(SVC_MELEE_INSTRUMENTS) && melee_fired(now, speed, reach);
}

void features_xinput_note_buttons(unsigned short buttons) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->xinput_note_buttons != nullptr) f->xinput_note_buttons(buttons);
}

void features_game_tick_after_blam_aim() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->game_tick_after_blam_aim != nullptr) f->game_tick_after_blam_aim();
}

void features_holster_reset() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->holster_reset != nullptr) f->holster_reset();
}

unsigned features_holster_mesh_sweep_period() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->holster_mesh_sweep_period != nullptr) return f->holster_mesh_sweep_period();
    return 120u;
}

void features_holster_mesh_swept(const void* mf) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->holster_mesh_swept != nullptr) f->holster_mesh_swept(mf);
}

void features_holster_before_release(HolsterSlot zone_g, HolsterSlot zone_p,
                                     const Vec3& pos, const Vec3& gpos, const Vec3& hpos) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->holster_before_release != nullptr) f->holster_before_release(zone_g, zone_p, pos, gpos, hpos);
}

void features_blam_create_before(uintptr_t params) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->blam_create_before != nullptr) f->blam_create_before(params);
}

uintptr_t features_blam_create_after(uintptr_t params, uintptr_t cret) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->blam_create_after != nullptr) f->blam_create_after(params, cret);
    return cret;
}

void features_tick_begin() { stability_tick_begin(); }
void features_tick_faulted() { stability_tick_faulted(); }
void features_rig_parent_resolved() { stability_rig_parent_resolved(); }
void features_stale_rig_guard() { stability_stale_rig_guard(); }
void features_tick_stage(const char* stage) { stability_tick_stage(stage); }
const char* features_tick_fault_stage() { return stability_fault_stage_suffix(); }
bool features_nav_world_guarded(bool engaged, uint32_t tick) { return stability_nav_world_guarded(engaged, tick); }
bool features_ui_manager_miss_throttled() { return stability_ui_manager_miss_throttled(); }
bool features_reticle_rescan_follow(bool hud_hide, int reticle_count) { return stability_reticle_rescan_follow(hud_hide, reticle_count); }
void features_reticle_hide_begin() { stability_reticle_hide_begin(); }
void features_reticle_hide_dead() { stability_reticle_hide_dead(); }
bool features_reticle_hide_end() { return stability_reticle_hide_end(); }
bool features_xrlayer_latch_released() { return stability_xrlayer_latch_released(); }
bool features_xrsource_wanted() { return stability_xrsource_wanted(); }
bool features_move_probe_allowed() { return stability_move_probe_allowed(); }
unsigned short features_steal_extra_mask() { return stability_steal_extra_mask(); }
// THE SEAT STAND-DOWN ON THE HOLSTER STEAL. The steal takes the throw and grenade-switch buttons
// off the pad so the holster gestures own them; mounted in a Warthog or a turret those are the
// game's own controls and taking them reads as "the controllers stopped working".
//
// Whether the biped has a parent object is the fork's unit-state service to answer, so the answer
// is given here rather than by his code. It is gated on the SERVICE, not on the atomic alone: with
// no consumer enabled publish_unit_state is never called and the flag keeps whatever it last held,
// so a cfg reload that switches the last consumer off WHILE MOUNTED would otherwise latch a stale
// "mounted" for the rest of the session and stand the steal down for good. Inactive service = not
// mounted = his line behaves exactly as it does with every feature of ours off.
bool features_holster_steal_mounted() {
    return service_active(SVC_UNIT_STATE) && g_unit_mounted.load(std::memory_order_relaxed);
}
void features_stick_mode_want(bool want) { stability_stick_mode_want(want); }
bool features_stick_exit_after_death() { return stability_stick_exit_after_death(); }
void features_turn_gate_note(bool fp_control_now) { stability_turn_gate_note(fp_control_now); }
void features_turn_snap_note(float step) { stability_turn_snap_note(step); }
void features_teardown_early() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->teardown != nullptr) f->teardown();
    stability_teardown_early();
}
void features_teardown_restore() { stability_teardown_restore(); }

void features_holster_pouch_offhand(bool ghand_ok, const Vec3& ghand, const Vec3& pouch) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->holster_pouch_offhand != nullptr) f->holster_pouch_offhand(ghand_ok, ghand, pouch);
}

void features_holster_pouches_measured() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->holster_pouches_measured != nullptr) f->holster_pouches_measured();
}

// ---- THE RELOAD ENGINE (core/reload). The state machine's hooks run only inside the author's enabled
// reload (reloadvr on), so they read the manual availability; the rest read either availability.
void features_gesture_tick_begin(float dt) { reload_engine_tick_begin(dt, reload_engine_active()); }
void features_reload_state_set(ReloadState prev, ReloadState next) { if (reload_engine_active()) reload_engine_state_set(prev, next); }
int  features_reload_fire_suppressed() { return reload_engine_active() ? reload_engine_fire_suppressed() : -1; }
void features_reload_disabled() { if (reload_engine_active()) reload_engine_disabled(); }
void features_reload_update_begin() { if (reload_engine_active()) reload_engine_update_begin(); }
void features_reload_timed_out() { if (reload_engine_active()) reload_engine_timed_out(); }
void features_reload_buttons_read() { if (reload_engine_active()) reload_engine_swap_cancel(); }
bool features_reload_grip_held() { return reload_manual_available() && reload_engine_grip_held(); }
bool features_reload_fetch_pose(bool pose_ok, const Vec3& hand_l, const Vec3* head) {
    return reload_manual_available() ? reload_engine_fetch_pose(pose_ok, hand_l, head) : pose_ok;
}
bool features_reload_press_ignored() { return reload_manual_available() && reload_engine_press_ignored(); }
void features_reload_press_accepted() { if (reload_manual_available()) reload_engine_press_accepted(); }
bool features_reload_belt_grab_ok(bool belt) { return reload_manual_available() ? reload_engine_belt_grab_ok(belt) : belt; }
void features_reload_grabbed(float hand_y) { if (reload_manual_available()) reload_engine_grabbed(hand_y); }
bool features_reload_seat(bool have_left, const Vec3& hand_l, const Vec3* hand_r, const Vec3* head) {
    return reload_manual_available() && reload_engine_seat(have_left, hand_l, hand_r, head);
}
void features_gesture_reset() {
    if (reload_engine_active()) reload_engine_gesture_reset();
    if (service_active(SVC_STABILITY)) stability_gesture_reset_two_hand();
    for (const FeatureHooks* f : kFeatureList)
        if (f->gesture_reset != nullptr && f->enabled != nullptr && f->enabled()) f->gesture_reset();
}
void features_reload_ticks(bool poses_ok, const Vec3& hpos) { if (reload_engine_active()) reload_engine_ticks(poses_ok, hpos); }
bool features_two_hand_support_blocked() {
    return reload_engine_active() && g_slide_zone_hot.load(std::memory_order_relaxed);
}
uevr::API::UObject* features_holster_mag_mesh(int* out_rank) { return reload_manual_available() ? reload_engine_mag_mesh(out_rank) : nullptr; }
Vec3 features_holster_mag_belt_point(const Vec3& his_offset) { return reload_manual_available() ? reload_engine_mag_belt_point() : his_offset; }
bool features_holster_mag_cands_stale(int rank) { return reload_manual_available() && reload_engine_mag_cands_stale(rank); }
Vec3 features_holster_mag_zone_point(const Vec3& belt, const Vec3& anchor, float yaw_cos, float yaw_sin) {
    return reload_manual_available() ? reload_engine_mag_zone_point(belt, anchor, yaw_cos, yaw_sin) : belt;
}
bool features_holster_mag_survey_off() { return reload_manual_available() && reload_engine_mag_survey_off(); }
bool features_holster_mag_resurvey(int rank) {
    return reload_manual_available() ? reload_engine_mag_resurvey(rank) : (rank < 3);
}
bool features_holster_mag_pick_final(int rank) {
    return !reload_manual_available() || reload_engine_mag_pick_final(rank);
}
uevr::API::UObject* features_holster_mag_spawn_mesh(uevr::API::UObject* survey, uevr::API::UObject* frag) {
    return reload_manual_available() ? reload_engine_mag_spawn_mesh(survey, frag)
                                     : (survey != nullptr ? survey : frag);
}
void features_holster_mag_drawn(uevr::API::UObject* m, bool wanted) {
    if (reload_manual_available()) reload_engine_mag_drawn(m, wanted);
}
bool features_holster_mag_in_hand(uevr::API::UObject* m, const Vec3& gpos, const Vec3& hpos, float pitchr, float yawr, float rollr) {
    return reload_manual_available() && reload_engine_mag_in_hand(m, gpos, hpos, pitchr, yawr, rollr);
}
bool features_rig_resolve_wanted() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->rig_resolve_wanted != nullptr && f->rig_resolve_wanted()) return true;
    return reload_engine_active();
}

// ---- THE AUTHOR'S ARM DRIVER ARBITER AND ARM HIDE
const char* features_arm_driver_name(int mode) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->arm_driver_name != nullptr) if (const char* n = f->arm_driver_name(mode)) return n;
    return nullptr;
}
int features_arm_driver_mode_wanted() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->arm_driver_mode_wanted != nullptr) if (const int m = f->arm_driver_mode_wanted(); m > 0) return m;
    return 0;
}
bool features_arm_driver_mode_unavailable(int wanted) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->arm_driver_mode_unavailable != nullptr && f->arm_driver_mode_unavailable(wanted)) return true;
    return false;
}
bool features_arm_driver_key_changed() {
    bool any = false;
    for (const FeatureHooks* f : kFeatureList)
        if (f->arm_driver_key_changed != nullptr && f->arm_driver_key_changed()) any = true;
    return any;
}
void features_arm_driver_steady(int active) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->arm_driver_steady != nullptr) f->arm_driver_steady(active);
}
void features_arm_driver_active(int mode, bool switched) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->arm_driver_active != nullptr) f->arm_driver_active(mode, switched);
}
void features_arm_driver_release_all(const char* why) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->arm_driver_release_all != nullptr) f->arm_driver_release_all(why);
}
bool features_arm_hide_component(uevr::API::UObject* comp, bool hide, int mode) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->arm_hide_component != nullptr
            && f->arm_hide_component(comp, hide, mode, f->enabled != nullptr && f->enabled())) return true;
    return false;
}
bool features_arm_hide_held_off() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->arm_hide_held_off != nullptr && f->enabled != nullptr && f->enabled() && f->arm_hide_held_off()) return true;
    return false;
}
bool features_arm_hide_needs_rig() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->arm_hide_needs_rig != nullptr && f->enabled != nullptr && f->enabled() && f->arm_hide_needs_rig()) return true;
    return false;
}

// ---- THE AIM DERIVATION AND THE PALETTE POSE INTERFACE
namespace {
const PalettePoseProvider* palette_pose_provider() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->palette_pose != nullptr) return f->palette_pose;
    return nullptr;
}
// The aim-fixed source rotation derive_ctrl_angles last took its forward from, on this thread.
thread_local Quat t_aim_source{0.0f, 0.0f, 0.0f, 1.0f};
} // namespace

bool palette_pose_owns_aim() {
    const auto* p = palette_pose_provider();
    return p != nullptr && p->owns_aim != nullptr && p->owns_aim();
}
bool palette_pose_trim_rotations(float grip_q[4], float weapon_q[4]) {
    const auto* p = palette_pose_provider();
    return p != nullptr && p->trim_rotations != nullptr && p->trim_rotations(grip_q, weapon_q);
}
bool palette_pose_two_hand_blend(Vec3* fwd) {
    const auto* p = palette_pose_provider();
    return p != nullptr && p->two_hand_blend != nullptr && p->two_hand_blend(fwd);
}
bool palette_pose_barrel_axis(Vec3* out) {
    const auto* p = palette_pose_provider();
    return p != nullptr && p->barrel_axis != nullptr && p->barrel_axis(out);
}

bool palette_pose_stamp_available() {
    const auto* p = palette_pose_provider();
    return p != nullptr && p->stamp_available != nullptr && p->stamp_available();
}
bool palette_pose_stamped_intent(bool two_back, float* yaw, float* pitch) {
    const auto* p = palette_pose_provider();
    return p != nullptr && p->stamped_intent != nullptr && p->stamped_intent(two_back, yaw, pitch);
}
void palette_pose_mark(int point, float yaw, float e0, float e1, float e2) {
    const auto* p = palette_pose_provider();
    if (p != nullptr && p->mark != nullptr) p->mark(point, yaw, e0, e1, e2);
}
void features_game_tick_after_rig_driver(double aim_yaw, double aim_pitch, uint32_t tick) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->game_tick_after_rig_driver != nullptr && f->enabled != nullptr && f->enabled()) f->game_tick_after_rig_driver(aim_yaw, aim_pitch, tick);
}
void features_stereo_post_eye_publish(int index) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->stereo_post_eye_publish != nullptr && f->enabled != nullptr && f->enabled()) f->stereo_post_eye_publish(index);
}
bool palette_pose_weapon_quat(Quat* out) {
    const auto* p = palette_pose_provider();
    return p != nullptr && p->weapon_quat != nullptr && p->weapon_quat(out);
}
bool palette_pose_aim_fix(const Quat& q_src, Quat* out) {
    const auto* p = palette_pose_provider();
    return p != nullptr && p->owns_aim != nullptr && p->aim_fix != nullptr && p->owns_aim() && p->aim_fix(q_src, out);
}
Quat placement_aim_fix(const Quat& q_src) {
    Quat q{};
    return palette_pose_aim_fix(q_src, &q) ? q : apply_aim_fix(q_src);
}
float palette_pose_roll_trim_deg() {
    const auto* p = palette_pose_provider();
    return (p != nullptr && p->roll_trim_deg != nullptr) ? p->roll_trim_deg() : 0.0f;
}
int features_reticule_render_publish_mode() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->reticule_render_publish_mode != nullptr && f->enabled != nullptr && f->enabled())
            if (const int m = f->reticule_render_publish_mode(); m > 0) return m;
    return 0;
}

bool features_aim_owned_by_feature() { return palette_pose_owns_aim(); }
Quat features_aim_fix_or(const Quat& q_src, const Quat& his_fixed) {
    Quat q{};
    return palette_pose_aim_fix(q_src, &q) ? q : his_fixed;
}
Quat features_aim_source(const Quat& q_src, const Quat& his_fixed) {
    t_aim_source = features_aim_fix_or(q_src, his_fixed);
    return t_aim_source;
}
bool features_aim_reference_offset(float* yaw, float* pitch, bool* valid, float* frame_yaw, float write_frame_yaw) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->aim_reference_offset != nullptr && f->enabled != nullptr && f->enabled()
            && f->aim_reference_offset(yaw, pitch, valid, frame_yaw, write_frame_yaw)) return true;
    return false;
}
bool features_aim_calibrated(float off_yaw, float off_pitch, float frame_yaw) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->aim_calibrated != nullptr && f->enabled != nullptr && f->enabled()
            && f->aim_calibrated(off_yaw, off_pitch, frame_yaw)) return true;
    return false;
}
bool features_aim_forward(Vec3* fwd) {
    if (!palette_pose_owns_aim()) return false;
    // ---- THE TWO-HANDED HOLD, here and deliberately: after the source pose is chosen, before
    // the sightline. Every consumer of aim -- the control law, the direct drive, the reticle,
    // the rendered weapon pose -- takes its direction from this one derivation, so blending at
    // this point keeps them agreeing by construction. The palette applies the SAME rotation to
    // the weapon pose (two_hand_delta); blending only one of the pair was field-observed as the
    // gun turning two-handed while the shots kept following the single hand.
    bool bore_done = false;
    for (const FeatureHooks* f : kFeatureList)
        if (f->aim_bore_forward != nullptr && f->enabled != nullptr && f->enabled() && f->aim_bore_forward(t_aim_source, fwd)) { bore_done = true; break; }
    if (!bore_done) palette_pose_two_hand_blend(fwd);
    return true;
}
void features_aim_direct_writing(float wy, float wp) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->aim_direct_writing != nullptr && f->enabled != nullptr && f->enabled()) f->aim_direct_writing(wy, wp);
}
bool features_aim_direct_write_skipped() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->aim_direct_write_skipped != nullptr && f->enabled != nullptr && f->enabled() && f->aim_direct_write_skipped()) return true;
    return false;
}
void features_aim_direct_written(float yaw, float pitch) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->aim_direct_written != nullptr && f->enabled != nullptr && f->enabled()) f->aim_direct_written(yaw, pitch);
}
bool features_pose_latched(UEVR_TrackedDeviceIndex idx, bool use_aim, uevr::API::VR::Pose* out) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->pose_latched != nullptr && f->enabled != nullptr && f->enabled() && f->pose_latched(idx, use_aim, out)) return true;
    return false;
}

// ---- THE AUTHOR'S PLUGIN CALLBACKS
void features_game_tick_after_blam_drive() {
    blam_capture_hook_tick();   // core: the object capture pre-hook follows rack availability every tick
    for (const FeatureHooks* f : kFeatureList)
        if (f->game_tick_after_blam_drive != nullptr && f->enabled != nullptr && f->enabled()) f->game_tick_after_blam_drive();
}
void features_game_tick_before_vehicle() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->game_tick_before_vehicle != nullptr && f->enabled != nullptr && f->enabled()) f->game_tick_before_vehicle();
}
void features_game_tick_after_vehicle(uint32_t tick) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->game_tick_after_vehicle != nullptr && f->enabled != nullptr && f->enabled()) f->game_tick_after_vehicle(tick);
}
void features_game_tick_after_gestures(float dt) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->game_tick_after_gestures != nullptr && f->enabled != nullptr && f->enabled()) f->game_tick_after_gestures(dt);
}
void features_engine_tick_start() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->engine_tick_start != nullptr && f->enabled != nullptr && f->enabled()) f->engine_tick_start();
}
void features_engine_tick_end() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->engine_tick_end != nullptr && f->enabled != nullptr && f->enabled()) f->engine_tick_end();
}
void features_post_engine_tick() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->post_engine_tick != nullptr && f->enabled != nullptr && f->enabled()) f->post_engine_tick();
}
bool features_fp_weapon_live() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->fp_weapon_live != nullptr && f->enabled != nullptr && f->enabled() && f->fp_weapon_live()) return true;
    return false;
}
void features_rig_parent_dropped() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->rig_parent_dropped != nullptr) f->rig_parent_dropped();
}
bool features_rig_driver_stood_down() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->rig_driver_stood_down != nullptr && f->rig_driver_stood_down()) return true;
    return false;
}
void features_stereo_post_eye_late(int index) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->stereo_post_eye_late != nullptr && f->enabled != nullptr && f->enabled()) f->stereo_post_eye_late(index);
}
void features_aim_law_sampling() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->aim_law_sampling != nullptr && f->enabled != nullptr && f->enabled()) f->aim_law_sampling();
}
void features_aim_law_sampled(double ay, double ap) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->aim_law_sampled != nullptr && f->enabled != nullptr && f->enabled()) f->aim_law_sampled(ay, ap);
}
void features_stereo_pre_eye_rendered(int index) {
    // The rendered camera position, for the marker layer's room->world (Markers.cpp).
    if (service_active(SVC_MARKER_ANCHOR)) marker_camera_publish();
    for (const FeatureHooks* f : kFeatureList)
        if (f->stereo_pre_eye_instruments != nullptr && f->enabled != nullptr && f->enabled()) f->stereo_pre_eye_instruments(index);
    // WRIST HUD PLACEMENT, here rather than on the tick: the camera above is the one this
    // frame is drawn from, so the forearm panels land against it instead of against a camera
    // several milliseconds stale. Once per frame, not per eye.
    if (index == 0) {
        features_render_frame();
        markers_render_place();
        for (const FeatureHooks* f : kFeatureList)
            if (f->render_refresh != nullptr && f->enabled != nullptr && f->enabled()) f->render_refresh();
    }
    for (const FeatureHooks* f : kFeatureList)
        if (f->stereo_pre_eye_meters != nullptr && f->enabled != nullptr && f->enabled()) f->stereo_pre_eye_meters(index);
}
void features_render_callbacks_register() {
    // Dev-only eye dump (a no-op stub in player builds): a render callback, so registered here.
    cutscene_dump_register();
}

bool features_menu_command_file_absent(const char* path) {
    return service_active(SVC_STABILITY) && stability_menu_command_file_absent(path);
}

void features_holster_marker_spawned(uevr::API::UObject* marker) { stability_holster_marker_tint(marker); }
bool features_holster_throw_too_slow(float peak_speed) { return stability_throw_too_slow(peak_speed); }
const char* features_holster_putback_text(const char* his_text, bool in_pouch) { return stability_putback_text(his_text, in_pouch); }

bool features_room_to_world(const Vec3& room, const Vec3& hmd_room, Vec3* out) {
    return room_to_world_anchored(room, hmd_room, out);   // SVC_MARKER_ANCHOR gate inside
}


// ================================================================================================
// RUNTIME STATE: services, the resolve log, and the release on a master key's off edge.
// ================================================================================================

namespace {

bool feature_on(const FeatureHooks* f) { return f->enabled != nullptr && f->enabled(); }

constexpr struct { uint32_t bit; const char* name; } kServiceNames[] = {
    { SVC_UNIT_STATE,        "unitstate" },
    { SVC_SEAT,              "seat" },
    { SVC_FIRE_INPUT,        "fireinput" },
    { SVC_EYE_TRACE,         "eyetrace" },
    { SVC_LEASH_GATE,        "leashgate" },
    { SVC_MARKER_ANCHOR,     "markeranchor" },
    { SVC_MELEE_INSTRUMENTS, "meleeinstruments" },
    { SVC_CAMERA_BOB,        "camerabob" },
    { SVC_HIDDEN_RELOAD,     "hiddenreload" },
    { SVC_RETICULE_FIXES,    "reticulefixes" },
    { SVC_WIDGET_HOSTS,      "widgethosts" },
    { SVC_STABILITY,         "stability" },
    { SVC_RIG_GUARD,         "rigguard" },
    { SVC_WEAPON_OBJECT,     "weaponobject" },
    { SVC_RACK_AVAILABLE,    "rackavailable" },
    { SVC_MANUAL_RELOAD_AVAILABLE, "manualreloadavailable" },
    { SVC_POSE_INTENTS,      "poseintents" },
};

uint32_t s_logged_mask = 0xFFFFFFFFu;   // the feature on/off mask the last log line showed
uint32_t s_service_mask = 0;             // the services active at that log line

uint32_t active_services() {
    uint32_t m = 0;
    for (const auto& sn : kServiceNames)
        if (service_active(sn.bit)) m |= sn.bit;
    return m;
}

uint32_t enabled_mask() {
    uint32_t m = 0;
    int i = 0;
    for (const FeatureHooks* f : kFeatureList) {
        if (feature_on(f)) m |= (1u << i);
        ++i;
    }
    return m;
}

void log_state(const char* why) {
    std::string feats;
    for (const FeatureHooks* f : kFeatureList) {
        if (!feats.empty()) feats += ' ';
        feats += f->key;
        feats += feature_on(f) ? "=on" : "=off";
    }
    uevr::API::get()->log_info("[Halo-CampE-UEVR] FEATURESTATE (%s): %s", why, feats.c_str());
    for (const auto& sn : kServiceNames) {
        const std::string who = service_enablers(sn.bit);
        uevr::API::get()->log_info("[Halo-CampE-UEVR] SERVICE %-16s %s%s%s%s", sn.name,
                                   who.empty() ? "inactive" : "active",
                                   who.empty() ? "" : " (enabled by ", who.c_str(), who.empty() ? "" : ")");
    }
}

}  // namespace

bool service_active(uint32_t service) {
    for (const FeatureHooks* f : kFeatureList)
        if ((f->services & service) != 0 && feature_on(f)) return true;
    return false;
}

const char* service_name(uint32_t service) {
    for (const auto& sn : kServiceNames)
        if (sn.bit == service) return sn.name;
    return "?";
}

std::string service_enablers(uint32_t service) {
    std::string out;
    for (const FeatureHooks* f : kFeatureList) {
        if ((f->services & service) == 0 || !feature_on(f)) continue;
        if (!out.empty()) out += ',';
        out += f->key;
    }
    return out;
}

void features_log_runtime() {
    s_logged_mask = enabled_mask();
    s_service_mask = active_services();
    log_state("startup");
}

void features_config_loaded() {
    const uint32_t now = enabled_mask();
    if (now == s_logged_mask) return;
    const uint32_t went_off = (s_logged_mask == 0xFFFFFFFFu) ? 0u : (s_logged_mask & ~now);
    int i = 0;
    for (const FeatureHooks* f : kFeatureList) {
        if ((went_off & (1u << i)) != 0 && f->released != nullptr) {
            uevr::API::get()->log_info("[Halo-CampE-UEVR] FEATURESTATE: %s switched off -- releasing", f->key);
            f->released();
        }
        ++i;
    }
    // Core state a service held while it was active, released on the service's own off edge.
    const uint32_t svc_now = active_services();
    const uint32_t svc_off = s_service_mask & ~svc_now;
    // The reload engine first: its release reads the weapon object the resets below clear.
    const uint32_t kEngine = SVC_MANUAL_RELOAD_AVAILABLE | SVC_RACK_AVAILABLE;
    if ((s_service_mask & kEngine) != 0 && (svc_now & kEngine) == 0) {
        reload_engine_released();
        // The held weapon's object index is the engine's to publish (nothing else writes it). Cleared with
        // the engine even while another consumer keeps the weapon object service on, or the probe would go
        // on resolving the weapon that was held when reload switched off.
        if ((svc_now & SVC_WEAPON_OBJECT) != 0) weapon_object_reset();
    }
    if ((svc_off & SVC_CAMERA_BOB) != 0) camera_bob_reset();
    if ((svc_off & SVC_HIDDEN_RELOAD) != 0) hidden_reload_reset();
    if ((svc_off & SVC_RACK_AVAILABLE) != 0) weapon_object_rack_reset();
    if ((svc_off & SVC_WEAPON_OBJECT) != 0) weapon_object_reset();
    s_service_mask = svc_now;
    s_logged_mask = now;
    log_state("config reload");
}

} // namespace halo
