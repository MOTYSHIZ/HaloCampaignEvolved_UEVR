#include "core/CoreKeys.hpp"

#include "Config.hpp"
#include "Math.hpp"   // clampf

#include <cstdlib>
#include <cstring>

namespace halo {

bool core_parse_key(const char* key, const char* val, double v) {
    // SVC_CAMERA_BOB (core/CameraBob).
    if (_stricmp(key, "bobcancel") == 0) { g_cfg.bob_cancel = (v != 0.0); return true; }
    if (_stricmp(key, "bobtau")    == 0) { g_cfg.bob_tau    = clampf((float)v, 0.02f, 5.0f); return true; }
    if (_stricmp(key, "boblog")    == 0) { g_cfg.bob_log    = (v != 0.0); return true; }
    // SVC_UNIT_STATE / SVC_SEAT (core/UnitState): the vehicle facing read, the stick-mode seat publish,
    // and the unit publish's evidence lines.
    if (_stricmp(key, "vehfacing")      == 0) { g_cfg.veh_facing = (int)v; return true; }
    if (_stricmp(key, "vehfacingoff")   == 0) { g_cfg.veh_facing_off = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "vehlog")         == 0) { g_cfg.veh_log = (int)v; return true; }
    if (_stricmp(key, "vehseatpub")     == 0) { g_cfg.veh_seat_pub = (int)v; return true; }
    return false;
}

} // namespace halo
