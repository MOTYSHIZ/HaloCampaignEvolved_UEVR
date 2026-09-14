// wristhud (fork feature, Experimental): the wrist HUD panel keys (wristhud*).
// Textual fragment, included by Config.cpp inside parse_blam_key(), after the ForceTube keys. Moved verbatim; not compiled on its own.
    if (_stricmp(key, "wristhud")        == 0) { g_cfg.wrist_hud = (v != 0.0); return true; }
    if (_stricmp(key, "wristhudclasses") == 0) { strncpy_s(g_cfg.wrist_hud_classes, val, _TRUNCATE); return true; }
    if (_stricmp(key, "wristhudoff")     == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.wrist_hud_off[0], &g_cfg.wrist_hud_off[1], &g_cfg.wrist_hud_off[2]); return true; }
    if (_stricmp(key, "wristhudrot")     == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.wrist_hud_rot[0], &g_cfg.wrist_hud_rot[1], &g_cfg.wrist_hud_rot[2]); return true; }
    if (_stricmp(key, "wristhudscale")   == 0) { g_cfg.wrist_hud_scale = clampf((float)v, 0.005f, 1.0f); return true; }
    if (_stricmp(key, "wristhudgap")     == 0) { g_cfg.wrist_hud_gap = clampf((float)v, 0.0f, 0.5f); return true; }
    if (_stricmp(key, "wristhuddraw")    == 0) { g_cfg.wrist_hud_draw = clampf((float)v, 64.0f, 2048.0f); return true; }
    if (_stricmp(key, "wristhudtrigger") == 0) { g_cfg.wrist_hud_trigger = (v != 0.0); return true; }
    if (_stricmp(key, "wristhudclassesr") == 0) { strncpy_s(g_cfg.wrist_hud_classes_r, val, _TRUNCATE); return true; }
    if (_stricmp(key, "wristhudoffr")    == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.wrist_hud_off_r[0], &g_cfg.wrist_hud_off_r[1], &g_cfg.wrist_hud_off_r[2]); return true; }
    if (_stricmp(key, "wristhudrotr")    == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.wrist_hud_rot_r[0], &g_cfg.wrist_hud_rot_r[1], &g_cfg.wrist_hud_rot_r[2]); return true; }
    if (_stricmp(key, "wristhudgapr")    == 0) { g_cfg.wrist_hud_gap_r = clampf((float)v, 0.0f, 0.5f); return true; }
