#include "WorldScaleFollow.hpp"
#include "core/config/CfgRead.hpp"

#include "Config.hpp"
#include "Math.hpp"                    // clampf
#include "uevr/API.hpp"

#include <cmath>
#include <cstdlib>

using namespace uevr;

namespace halo {

bool worldscalefollow_parse_key(const char* key, const char* val, double v) {
    (void)val;
    if (_stricmp(key, "worldscalefollow") == 0) { g_cfg.world_scale_follow = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    return false;
}

namespace {
float s_last_good = 0.0f;    // the last VR_WorldScale taken (0 = none yet)
float s_logged    = -1.0f;   // the rig_scale last reported, so a reload that puts it back is silent
}  // namespace

void worldscalefollow_poll() {
    // Off: the reload that switched it off already put the cfg value back.
    if (g_cfg.world_scale_follow == 0) { s_logged = -1.0f; return; }
    char buf[64]{};
    if (auto* p = API::get()->param(); p != nullptr && p->vr != nullptr && p->vr->get_mod_value != nullptr)
        p->vr->get_mod_value("VR_WorldScale", buf, sizeof(buf));
    const float ws = buf[0] != 0 ? (float)atof(buf) : 0.0f;
    if (ws >= 0.1f && ws < 100.0f) s_last_good = ws;
    if (s_last_good <= 0.0f) return;
    const float want = 100.0f * s_last_good;
    if (std::fabs(want - s_logged) > 0.05f) {
        API::get()->log_info("[Halo-CampE-UEVR] WORLDSCALE: rig_scale %.1f -> %.1f (100 x UEVR VR_WorldScale %.3f)",
                             g_cfg.rig_scale, want, s_last_good);
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
