#include "features/aimreticulestamp/AimReticuleStamp.hpp"

#include "Config.hpp"
#include "Math.hpp"   // clampf

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include "core/PalettePose.hpp"
#include "core/Services.hpp"

namespace halo {

namespace {
bool aim_reticule_stamp_parse_key(const char* key, const char* val, double v) {
    (void)val; (void)v;
    if (_stricmp(key, "aimreticulestamp") == 0) { g_cfg.aim_reticule_stamp = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    return false;
}

bool aim_reticule_stamp_enabled() { return g_cfg.aim_reticule_stamp != 0; }

// The stamped placement (modes 1/2) owns the compositor reticle's publish at render, and only while the
// palette weapon owns the aim and its latch stamps the intent. Without it the render publish never fires
// and the compositor reticle goes dark with no message, so the tick publish takes over instead.
int reticule_render_publish_mode() {
    if (!palette_pose_owns_aim() || !palette_pose_stamp_available()) return 0;
    return g_cfg.aim_reticule_stamp;
}
}  // namespace

constinit const FeatureHooks kAimReticuleStampHooks{
    .key      = "aimreticulestamp",
    .parse_key = &aim_reticule_stamp_parse_key,
    .reticule_render_publish_mode = &reticule_render_publish_mode,
    .enabled  = &aim_reticule_stamp_enabled,
    .services = SVC_POSE_INTENTS,
};

} // namespace halo
