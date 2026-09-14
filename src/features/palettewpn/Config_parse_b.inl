// palettewpn (fork feature, Experimental): palette keys, second run (palbuildgate..poselatch).
// Textual fragment, included by Config.cpp inside parse_fork_port_key(), after magrender. Moved verbatim; not compiled on its own.
    if (_stricmp(key, "palbuildgate")   == 0) { g_cfg.pal_build_gate = (int)v; return true; }
    if (_stricmp(key, "palettecamlead") == 0) { g_cfg.palette_cam_lead = clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "palettecamsmooth") == 0) { g_cfg.palette_cam_smooth_ms = clampf((float)v, 0.0f, 2000.0f); return true; }
    if (_stricmp(key, "palettefinal")   == 0) { g_cfg.palette_final = (int)v; return true; }
    if (_stricmp(key, "palettelatch") == 0) { g_cfg.palette_latch = (v != 0.0) ? 1 : 0; return true; }
    if (_stricmp(key, "palettelatchms") == 0) { g_cfg.palette_latch_ms = clampf((float)v, 0.0f, 40.0f); return true; }
    if (_stricmp(key, "palettelocal") == 0) { g_cfg.palette_local = (v != 0.0) ? 1 : 0; return true; }
    if (_stricmp(key, "palettesync") == 0) { g_cfg.palette_sync = (v != 0.0) ? 1 : 0; return true; }
    if (_stricmp(key, "palpubframe")    == 0) { g_cfg.pal_pub_frame = (int)v; return true; }
    if (_stricmp(key, "palrender")      == 0) { g_cfg.pal_render = (int)v; return true; }
    if (_stricmp(key, "palsniff")       == 0) { g_cfg.pal_sniff = (int)v; return true; }
    if (_stricmp(key, "palstep") == 0) { g_cfg.palette_step = clampf((float)v, -2.0f, 2.0f); return true; }
    if (_stricmp(key, "palstepctx") == 0) { g_cfg.palette_step_ctx = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "palstepsrc") == 0) { g_cfg.palette_step_src = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "posefilter")     == 0) { g_cfg.pose_filter = (int)v; return true; }
    if (_stricmp(key, "posefilterbeta") == 0) { g_cfg.pose_filter_beta = clampf((float)v, 0.0f, 500.0f); return true; }
    if (_stricmp(key, "posefilterdcut") == 0) { g_cfg.pose_filter_dcut = clampf((float)v, 0.05f, 30.0f); return true; }
    if (_stricmp(key, "posefiltermin")  == 0) { g_cfg.pose_filter_min = clampf((float)v, 0.05f, 60.0f); return true; }
    if (_stricmp(key, "posefilterrbeta")== 0) { g_cfg.pose_filter_rbeta = clampf((float)v, 0.0f, 500.0f); return true; }
    if (_stricmp(key, "posefreeze")     == 0) { g_cfg.pose_freeze = (int)v; return true; }
    if (_stricmp(key, "poselatch") == 0) { g_cfg.pose_latch = (int)clampf((float)v, 0.0f, 3.0f); return true; }
