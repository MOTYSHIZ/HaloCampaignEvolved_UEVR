#include "features/stabilityfixes/StabilityFixes.hpp"

#include "Config.hpp"
#include "core/Services.hpp"

#include <cstring>

namespace halo {

namespace {

bool stability_fixes_enabled() { return g_cfg.stability_fixes; }

bool stabilityfixes_parse_key(const char* key, const char* val, double v) {
    (void)val;
    if (_stricmp(key, "stabilityfixes") == 0) { g_cfg.stability_fixes = (v != 0.0); return true; }
    return false;
}

}  // namespace

constinit const FeatureHooks kStabilityFixesHooks{
    .key       = "stabilityfixes",
    .parse_key = &stabilityfixes_parse_key,
    .enabled   = &stability_fixes_enabled,
    .services  = SVC_STABILITY | SVC_RIG_GUARD | SVC_LEASH_GATE | SVC_RETICULE_FIXES | SVC_MELEE_INSTRUMENTS,
};

} // namespace halo
