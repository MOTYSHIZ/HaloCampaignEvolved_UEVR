#include "features/stabilityfixes/StabilityFixes.hpp"
#include "core/config/CfgRead.hpp"

#include "Config.hpp"
#include "Math.hpp"   // clampf
#include "core/Services.hpp"

#include <cstring>

namespace halo {

namespace {

bool stability_fixes_enabled() { CFG_HOOK_READ; return g_cfg.stability_fixes; }

bool stabilityfixes_parse_key(const char* key, const char* val, double v) {
    (void)val;
    if (_stricmp(key, "stabilityfixes") == 0) { g_cfg.stability_fixes = (v != 0.0); return true; }
    // ---- GESTURE RELOAD tuning, the Wwise reload-sound adoption (ak*) and the animation probes.
    // Flat links in this family for the reason the note above gives.
    if (_stricmp(key, "turnlog")        == 0) { g_cfg.turn_log       = (v != 0.0); return true; }
    if (_stricmp(key, "widgetlog")      == 0) { g_cfg.widget_log     = (v != 0.0); return true; }
    if (_stricmp(key, "holstermarkercolor") == 0) { strncpy_s(g_cfg.holster_marker_color, val, _TRUNCATE); return true; }
    // The minimum throw speed.
    if (_stricmp(key, "grenminthrow")   == 0) { g_cfg.gren_min_throw = clampf((float)v, 0.0f, 6.0f); return true; }
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
