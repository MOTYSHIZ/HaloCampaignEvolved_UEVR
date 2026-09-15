#include "features/slidevr/SlideVr.hpp"
#include "core/config/CfgRead.hpp"

#include "Config.hpp"
#include "Math.hpp"   // clampf

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include "core/Services.hpp"

namespace halo {

namespace {
bool slide_vr_parse_key(const char* key, const char* val, double v) {
    (void)val; (void)v;
    if (_stricmp(key, "slidevr")        == 0) { g_cfg.slide_vr = (v != 0.0); return true; }
    return false;
}

bool slide_vr_enabled() { CFG_HOOK_READ; return g_cfg.slide_vr; }
}  // namespace

constinit const FeatureHooks kSlideVrHooks{
    .key      = "slidevr",
    .parse_key = &slide_vr_parse_key,
    .enabled  = &slide_vr_enabled,
    .services = SVC_FIRE_INPUT | SVC_MARKER_ANCHOR | SVC_HIDDEN_RELOAD | SVC_WEAPON_OBJECT | SVC_RACK_AVAILABLE,
};

} // namespace halo
