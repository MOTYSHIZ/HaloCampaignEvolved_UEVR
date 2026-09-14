// heightcal (fork feature, Experimental): the auto height keys, second run (heightband..heightwindow).
// Textual fragment, included by Config.cpp inside parse_fork_port_key(), after the head block keys. Moved verbatim; not compiled on its own.
    if (_stricmp(key, "heightband")       == 0) { g_cfg.height_band     = clampf((float)v, 1.0f, 50.0f) * 0.01f; return true; }
    if (_stricmp(key, "heightcal")        == 0) { g_cfg.height_cal      = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "heightkey")        == 0) {
        const int k = (int)strtol(val, nullptr, 0);
        g_cfg.height_key = (k == 0x2D) ? 0 : k;   // Insert opens UEVR's menu: never a height key
        return true;
    }
    if (_stricmp(key, "heightlog")        == 0) { g_cfg.height_log      = (int)clampf((float)v, 0.0f, 100000.0f); return true; }
    if (_stricmp(key, "heightmin")        == 0) { g_cfg.height_min_abs  = clampf((float)v, 0.0f, 250.0f) * 0.01f; return true; }
    if (_stricmp(key, "heightsample")     == 0) { g_cfg.height_sample   = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "heightslew")       == 0) { g_cfg.height_slew     = clampf((float)v, 0.0f, 500.0f) * 0.01f; return true; }
    if (_stricmp(key, "heightsrc")        == 0) { g_cfg.height_src      = (int)clampf((float)v, 0.0f, 3.0f); return true; }
    if (_stricmp(key, "heighttrim")       == 0) { g_cfg.height_trim     = clampf((float)v, -50.0f, 50.0f) * 0.01f; return true; }
    if (_stricmp(key, "heightwindow")     == 0) { g_cfg.height_window_s = clampf((float)v, 1.0f, 60.0f); return true; }
