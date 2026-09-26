#include "core/CoreKeys.hpp"

#include "Config.hpp"
#include "Math.hpp"   // clampf
#include "core/reload/ReloadKeys.hpp"

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
    // SVC_WEAPON_OBJECT (core/WeaponObject), core/MarkerFaces, and two stabilityfixes gates on the author's
    // code (core/fixes/HostFixes): moveprobe (his movement PROBE) and stealextra (extra holster steal bits).
    if (_stricmp(key, "magrender")      == 0) { g_cfg.mag_render = (int)v; return true; }
    if (_stricmp(key, "moveprobe")      == 0) { g_cfg.move_probe     = (v != 0.0); return true; }
    if (_stricmp(key, "reloadroomanchor")     == 0) { g_cfg.room_anchor = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "wpnnodedump")    == 0) { g_cfg.wpn_node_dump = (v != 0.0); return true; }
    if (_stricmp(key, "wpnnodecopyscan") == 0) { g_cfg.wpn_node_copy_scan = (v != 0.0); return true; }
    if (_stricmp(key, "wpnnodepoke")    == 0) { g_cfg.wpn_node_poke = (int)clampf((float)v, -1.0f, 63.0f); return true; }
    if (_stricmp(key, "wpnnodepokeamt") == 0) { g_cfg.wpn_node_poke_amt = clampf((float)v, -2.0f, 2.0f); return true; }
    if (_stricmp(key, "slidenode")      == 0) { g_cfg.slide_node = (int)clampf((float)v, -1.0f, 63.0f); return true; }
    if (_stricmp(key, "markertint")         == 0) { g_cfg.marker_tint_on = (v != 0.0); return true; }
    if (_stricmp(key, "stealextra")     == 0) { g_cfg.steal_extra_mask = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "slidehook")      == 0) { g_cfg.slide_hook = (v != 0.0); return true; }
    // Core readers of what were the author's log switches, now the fork's own (core/UnitState's
    // holster evidence and core/dev's cutscene frame grab).
    if (_stricmp(key, "holsterpollthrowlog") == 0) { g_cfg.holster_throw_log = (v != 0.0); return true; }
    if (_stricmp(key, "cutscenegrab")    == 0) { g_cfg.cutscene_grab = (int)v; return true; }
    if (_stricmp(key, "driverprobe")     == 0) { g_cfg.driver_probe = (int)v; return true; }
    // The palette weapon's diagnostic log switch, also read by the wrist HUD's one-shot widget log.
    if (_stricmp(key, "palettewpnlog")  == 0) { g_cfg.palette_weapon_log   = (v != 0.0); return true; }
    // The reload engine's keys (core/reload).
    if (reload_engine_parse_key(key, val, v)) return true;
    return false;
}

} // namespace halo
