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
#include "features/hooks/ScopeHooks.hpp"
#include "features/hooks/UnitStateHooks.hpp"

#include "Config.hpp"
#include "core/Services.hpp"
#include "uevr/API.hpp"
#include "core/CameraBob.hpp"
#include "core/CoreKeys.hpp"
#include "core/EyeTrace.hpp"
#include "core/HiddenReload.hpp"
#include "core/FireInput.hpp"
#include "core/MarkerFaces.hpp"
#include "core/UnitState.hpp"
#include "core/WeaponObject.hpp"
#include "core/fixes/HmdPoseGate.hpp"
#include "core/fixes/MeleeInstruments.hpp"
#include "core/fixes/ReticuleFixes.hpp"
#include "core/fixes/HostFixes.hpp"

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
    &kHolsterPollThrowHooks,
    &kReloadVrHooks,
    &kSlideVrHooks,
    &kPaletteWpnHooks,
    &kAimBoreHooks,
    &kAimReticuleStampHooks,
    &kStabilityFixesHooks,
};

} // namespace

bool features_parse_key(const char* key, const char* val, double v) {
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
}

void features_stereo_post_eye(int index, UEVR_Vector3f* position, bool is_double) {
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
void features_xrlayer_early(uint32_t tick) { stability_xrlayer_early(tick); }
void features_stick_mode_want(bool want) { stability_stick_mode_want(want); }
bool features_stick_exit_after_death() { return stability_stick_exit_after_death(); }
void features_turn_gate_note(bool fp_control_now) { stability_turn_gate_note(fp_control_now); }
void features_turn_snap_note(float step) { stability_turn_snap_note(step); }
void features_teardown_early() { stability_teardown_early(); }
void features_teardown_restore() { stability_teardown_restore(); }

void features_holster_pouch_offhand(bool ghand_ok, const Vec3& ghand, const Vec3& pouch) {
    for (const FeatureHooks* f : kFeatureList)
        if (f->holster_pouch_offhand != nullptr) f->holster_pouch_offhand(ghand_ok, ghand, pouch);
}

void features_holster_pouches_measured() {
    for (const FeatureHooks* f : kFeatureList)
        if (f->holster_pouches_measured != nullptr) f->holster_pouches_measured();
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
    if ((svc_off & SVC_CAMERA_BOB) != 0) camera_bob_reset();
    if ((svc_off & SVC_HIDDEN_RELOAD) != 0) hidden_reload_reset();
    if ((svc_off & SVC_RACK_AVAILABLE) != 0) weapon_object_rack_reset();
    if ((svc_off & SVC_WEAPON_OBJECT) != 0) weapon_object_reset();
    s_service_mask = svc_now;
    s_logged_mask = now;
    log_state("config reload");
}

} // namespace halo
