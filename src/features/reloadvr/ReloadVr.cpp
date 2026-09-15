#include "features/reloadvr/ReloadVr.hpp"
#include "core/config/CfgRead.hpp"

#include "Arms.hpp"                       // arms_release_hide: the author's release path
#include "Config.hpp"
#include "core/Services.hpp"
#include "core/host/ArmsState.hpp"        // the author's hide-applied flag
#include "core/reload/ReloadEngine.hpp"   // reload_engine_reload_busy
#include "uevr/API.hpp"

namespace halo {

namespace {
bool reload_vr_enabled() { CFG_HOOK_READ; return g_cfg.reload_vr; }

// reloadhidearms=3: armhide is derived on (features_apply), so the author's arm hide pass runs, and
// this holds his sweep off except while a reload is in progress; between reloads a hide he applied is
// released through his own release path. Dispatched only while reloadvr is enabled, and only from
// inside his arms_hide_update while armhide is on: with reloadvr off none of it runs.
bool reload_vr_arm_hide_held_off() {
    static bool s_engaged = false;
    if (g_cfg.reload_hide_arms_active != 3) { s_engaged = false; return false; }
    if (reload_engine_reload_busy()) {
        if (!s_engaged) {
            s_engaged = true;
            uevr::API::get()->log_info("[Halo-CampE-UEVR] ARMHIDE reloadhidearms=3 engages: a reload is in progress");
        }
        return false;
    }
    if (s_engaged) {
        s_engaged = false;
        uevr::API::get()->log_info("[Halo-CampE-UEVR] ARMHIDE reloadhidearms=3 releases: the reload finished");
    }
    if (*host::g_arms_state.any_hidden) arms_release_hide();
    return true;
}
}  // namespace

constinit const FeatureHooks kReloadVrHooks{
    .key      = "reloadvr",
    .arm_hide_held_off = &reload_vr_arm_hide_held_off,
    .enabled  = &reload_vr_enabled,
    .services = SVC_FIRE_INPUT | SVC_MARKER_ANCHOR | SVC_HIDDEN_RELOAD | SVC_WEAPON_OBJECT | SVC_MANUAL_RELOAD_AVAILABLE,
};

} // namespace halo
