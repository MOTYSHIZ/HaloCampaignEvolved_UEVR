#include "features/reloadvr/ReloadVr.hpp"
#include "core/config/CfgRead.hpp"

#include "Config.hpp"
#include "core/Services.hpp"

namespace halo {

namespace {
bool reload_vr_enabled() { CFG_HOOK_READ; return g_cfg.reload_vr; }
}  // namespace

constinit const FeatureHooks kReloadVrHooks{
    .key      = "reloadvr",
    .enabled  = &reload_vr_enabled,
    .services = SVC_FIRE_INPUT | SVC_MARKER_ANCHOR | SVC_HIDDEN_RELOAD | SVC_WEAPON_OBJECT | SVC_MANUAL_RELOAD_AVAILABLE,
};

} // namespace halo
