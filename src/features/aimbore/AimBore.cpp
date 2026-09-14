#include "features/aimbore/AimBore.hpp"

#include "Config.hpp"
#include "core/Services.hpp"

namespace halo {

namespace {
bool aim_bore_enabled() { return g_cfg.aim_bore != 0; }
}  // namespace

constinit const FeatureHooks kAimBoreHooks{
    .key      = "aimbore",
    .enabled  = &aim_bore_enabled,
    .services = SVC_HOST_FIXES,
};

} // namespace halo
