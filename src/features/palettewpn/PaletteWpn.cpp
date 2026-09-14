#include "features/palettewpn/PaletteWpn.hpp"

#include "Config.hpp"
#include "core/Services.hpp"

namespace halo {

namespace {
bool palette_wpn_enabled() { return g_cfg.palette_weapon || g_cfg.arm_driver == 3; }
}  // namespace

constinit const FeatureHooks kPaletteWpnHooks{
    .key      = "palettewpn",
    .enabled  = &palette_wpn_enabled,
    .services = SVC_MARKER_ANCHOR | SVC_CAMERA_BOB,
};

} // namespace halo
