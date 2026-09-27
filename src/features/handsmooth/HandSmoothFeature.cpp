#include "HandSmoothFeature.hpp"
#include "core/config/CfgRead.hpp"

#include "Config.hpp"
#include "core/HandSmooth.hpp"
#include "core/Services.hpp"

#include <cstring>

namespace halo {

bool handsmooth_parse_key(const char* key, const char* val, double v) {
    (void)val;
    if (_stricmp(key, "handsmooth") == 0) { g_cfg.hand_smooth = (v != 0.0); return true; }
    return false;
}

namespace {
bool hand_smooth_enabled() { CFG_HOOK_READ; return g_cfg.hand_smooth; }
bool hand_smooth_pose(UEVR_TrackedDeviceIndex idx, bool use_aim, uevr::API::VR::Pose* out) {
    return hand_smooth_lookup((int32_t)idx, use_aim, out);
}
}  // namespace

constinit const FeatureHooks kHandSmoothHooks{
    .key          = "handsmooth",
    .parse_key    = &handsmooth_parse_key,
    .pose_latched = &hand_smooth_pose,
    .enabled      = &hand_smooth_enabled,
    .services     = SVC_HAND_SMOOTH,
};

} // namespace halo
