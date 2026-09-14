#include "features/aimreticulestamp/AimReticuleStamp.hpp"

#include "Config.hpp"
#include "core/Services.hpp"

namespace halo {

namespace {
bool aim_reticule_stamp_enabled() { return g_cfg.aim_reticule_stamp != 0; }
}  // namespace

constinit const FeatureHooks kAimReticuleStampHooks{
    .key      = "aimreticulestamp",
    .enabled  = &aim_reticule_stamp_enabled,
    .services = 0,
};

} // namespace halo
