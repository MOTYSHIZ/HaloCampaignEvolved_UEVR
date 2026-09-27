#include "WorldScaleFollow.hpp"
#include "core/config/CfgRead.hpp"

#include "Config.hpp"
#include "Math.hpp"                    // clampf
#include "core/WorldScale.hpp"         // the shared, collapse-proof VR_WorldScale reader
#include "uevr/API.hpp"

#include <cmath>

using namespace uevr;

namespace halo {

bool worldscalefollow_parse_key(const char* key, const char* val, double v) {
    (void)val;
    if (_stricmp(key, "worldscalefollow") == 0) { g_cfg.world_scale_follow = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    return false;
}

namespace {
float s_logged    = -1.0f;   // the rig_scale last reported, so a reload that puts it back is silent
}  // namespace

void worldscalefollow_poll() {
    // Off: the reload that switched it off already put the cfg value back.
    if (g_cfg.world_scale_follow == 0) { s_logged = -1.0f; return; }
    // The shared reader (core/WorldScale). It carries the guard this file used to keep for itself --
    // under 0.1 is the mono collapse, keep the last good value -- so the arms now read the SAME value
    // as roomscale, auto height and the compositor layer, instead of each keeping its own.
    if (!uevr_world_scale_known()) return;   // never write rig_scale from the 1.0 fallback
    const float ws = uevr_world_scale();
    const float want = 100.0f * ws;
    if (std::fabs(want - s_logged) > 0.05f) {
        API::get()->log_info("[Halo-CampE-UEVR] WORLDSCALE: rig_scale %.1f -> %.1f (100 x UEVR VR_WorldScale %.3f)",
                             g_cfg.rig_scale, want, ws);
        s_logged = want;
    }
    g_cfg.rig_scale = want;
}

namespace {
bool world_scale_follow_enabled() { CFG_HOOK_READ; return g_cfg.world_scale_follow != 0; }
}  // namespace

constinit const FeatureHooks kWorldScaleFollowHooks{
    .key       = "worldscalefollow",
    .parse_key = &worldscalefollow_parse_key,
    .enabled   = &world_scale_follow_enabled,
    .services  = 0,
};

} // namespace halo
