// heightcal (fork feature, Experimental): the auto height keys, first run (heightmode..heightseatdwell).
// Textual fragment, included by Config.cpp inside parse_fork_port_key(), at its top. Moved verbatim; not compiled on its own.
    if (_stricmp(key, "heightmode")       == 0) { g_cfg.height_mode     = parse_height_mode(val, v); return true; }
    if (_stricmp(key, "heightscale")      == 0) { g_cfg.height_scale    = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "heighteye")        == 0) { g_cfg.height_eye      = (int)clampf((float)v, 1.0f, 3.0f); return true; }
    if (_stricmp(key, "heighttracechannel") == 0) { g_cfg.height_trace_channel = (int)clampf((float)v, 0.0f, 32.0f); return true; }
    if (_stricmp(key, "heighttracemax")   == 0) { g_cfg.height_trace_max = clampf((float)v, 50.0f, 2000.0f); return true; }
    if (_stricmp(key, "heightholdms")     == 0) { g_cfg.height_hold_ms  = clampf((float)v, 0.0f, 5000.0f); return true; }
    if (_stricmp(key, "heightestep")      == 0) { g_cfg.height_e_step   = clampf((float)v, 0.1f, 50.0f); return true; }
    if (_stricmp(key, "heightbipedscale") == 0) { g_cfg.height_biped_scale = clampf((float)v, 1.0f, 1000.0f); return true; }
    if (_stricmp(key, "heightbipedfeet")  == 0) { g_cfg.height_biped_feet  = clampf((float)v, -500.0f, 500.0f); return true; }
    if (_stricmp(key, "heightpawnfeet")   == 0) { g_cfg.height_pawn_feet   = clampf((float)v, -500.0f, 500.0f); return true; }
    if (_stricmp(key, "heightseattarget") == 0) { g_cfg.height_seat_target = clampf((float)v, 0.0f, 400.0f); return true; }
    if (_stricmp(key, "heightautoseat")   == 0) { g_cfg.height_auto_seat   = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "heightseatbelow")  == 0) { g_cfg.height_seat_below  = clampf((float)v, 0.0f, 250.0f) * 0.01f; return true; }
    if (_stricmp(key, "heightseatdwell")  == 0) { g_cfg.height_seat_dwell  = clampf((float)v, 0.5f, 60.0f); return true; }
