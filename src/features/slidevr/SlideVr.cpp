#include "features/slidevr/SlideVr.hpp"

#include "Config.hpp"
#include "core/Services.hpp"

namespace halo {

namespace {
bool slide_vr_enabled() { return g_cfg.slide_vr; }
}  // namespace

constinit const FeatureHooks kSlideVrHooks{
    .key      = "slidevr",
    .enabled  = &slide_vr_enabled,
    .services = SVC_FIRE_INPUT | SVC_MARKER_ANCHOR | SVC_HIDDEN_RELOAD | SVC_WEAPON_OBJECT | SVC_RACK_AVAILABLE,
};

} // namespace halo
