// roomscale (fork feature, Experimental): its key family (roomscale*, blamunitthrottleoff*, blamthrottleysign, bob*).
// Textual fragment, included by Config.cpp at namespace halo scope, beside the other hoisted key families. Moved verbatim; not compiled on its own.
// ---- ROOMSCALE + BOB CANCEL. Hoisted, early-return, same C1061 reasoning.
static bool parse_roomscale_key(const char* key, const char* val, double v) {
    if (_stricmp(key, "roomscale")      == 0) { g_cfg.roomscale       = (v != 0.0); return true; }
    if (_stricmp(key, "roomscalegain")  == 0) { g_cfg.roomscale_gain  = clampf((float)v, 0.1f, 50.0f); return true; }
    if (_stricmp(key, "roomscaledead")  == 0) { g_cfg.roomscale_dead  = clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "roomscaleleash") == 0) { g_cfg.roomscale_leash = clampf((float)v, 0.1f, 5.0f); return true; }
    if (_stricmp(key, "roomscalestick") == 0) { g_cfg.roomscale_stick = clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "roomscalelog")   == 0) { g_cfg.roomscale_log   = (v != 0.0); return true; }
    if (_stricmp(key, "roomscalemin")   == 0) { g_cfg.roomscale_min   = clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "roomscalespeed") == 0) { g_cfg.roomscale_speed = clampf((float)v, 0.1f, 20.0f); return true; }
    if (_stricmp(key, "roomscalelat")   == 0) { g_cfg.roomscale_lat   = clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "roomscaledz")    == 0) { g_cfg.roomscale_dz    = clampf((float)v, 0.0f, 0.9f); return true; }
    if (_stricmp(key, "roomscalepulse") == 0) { g_cfg.roomscale_pulse = (int)clampf((float)v, 1.0f, 30.0f); return true; }
    if (_stricmp(key, "roomscaleff")    == 0) { g_cfg.roomscale_ff    = clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "roomscalemaxspeed")  == 0) { g_cfg.roomscale_max_speed = (float)v; return true; }
    if (_stricmp(key, "roomscalestanddown") == 0) { g_cfg.roomscale_standdown = clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "roomscalethrottle")  == 0) { g_cfg.roomscale_throttle = (int)v; return true; }
    if (_stricmp(key, "roomscalethrspeed")  == 0) { g_cfg.roomscale_thr_speed = clampf((float)v, 0.5f, 30.0f); return true; }
    if (_stricmp(key, "roomscalethrprobe")  == 0) { g_cfg.roomscale_thr_probe = (int)v; return true; }
    if (_stricmp(key, "blamunitthrottleoff")  == 0) { g_cfg.blam_unit_throttle_off  = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "blamunitthrottleoff2") == 0) { g_cfg.blam_unit_throttle_off2 = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "blamthrottleysign")    == 0) { g_cfg.blam_throttle_ysign = (v < 0.0) ? -1 : 1; return true; }
    if (_stricmp(key, "bobcancel") == 0) { g_cfg.bob_cancel = (v != 0.0); return true; }
    if (_stricmp(key, "bobtau")    == 0) { g_cfg.bob_tau    = clampf((float)v, 0.02f, 5.0f); return true; }
    if (_stricmp(key, "boblog")    == 0) { g_cfg.bob_log    = (v != 0.0); return true; }
    return false;
}
