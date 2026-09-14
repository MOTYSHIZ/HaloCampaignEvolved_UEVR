// palettewpn (fork feature, Experimental): palette keys, first run (aimdirectwrite..fppin).
// Textual fragment, included by Config.cpp inside parse_fork_port_key(), after the reticule stamp keys. Moved verbatim; not compiled on its own.
    if (_stricmp(key, "aimdirectwrite") == 0) { g_cfg.aim_direct_write = (v != 0.0) ? 1 : 0; return true; }
    if (_stricmp(key, "camleadall") == 0) { g_cfg.cam_lead_all = (v != 0.0) ? 1 : 0; return true; }
    if (_stricmp(key, "compgain") == 0) { g_cfg.comp_gain = clampf((float)v, -2.0f, 2.0f); return true; }
    if (_stricmp(key, "complatch") == 0) { g_cfg.comp_latch = (v != 0.0) ? 1 : 0; return true; }
    if (_stricmp(key, "complatchms") == 0) { g_cfg.comp_latch_ms = clampf((float)v, 0.0f, 40.0f); return true; }
    if (_stricmp(key, "fpanimkill")     == 0) { g_cfg.fp_anim_kill = (int)v; return true; }
    if (_stricmp(key, "fpmeshlog")      == 0) { g_cfg.fpmesh_log = (int)v; return true; }
    if (_stricmp(key, "fppin")          == 0) { g_cfg.fp_pin = (int)v; return true; }
