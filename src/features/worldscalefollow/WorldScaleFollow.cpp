#include "WorldScaleFollow.hpp"
#include "core/config/CfgRead.hpp"

#include "Config.hpp"
#include "Math.hpp"                    // clampf

namespace halo {

// The key only. What it means now lives in core/WorldScale's world_scale_resolve_config(), which reads
// g_cfg.world_scale_follow inside the reload: ON, UEVR's world scale wins even over an explicit
// rigscale. Following is otherwise the default, so this used to be the fix and is now the override.
bool worldscalefollow_parse_key(const char* key, const char* val, double v) {
    (void)val;
    if (_stricmp(key, "worldscalefollow") == 0) { g_cfg.world_scale_follow = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    return false;
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
