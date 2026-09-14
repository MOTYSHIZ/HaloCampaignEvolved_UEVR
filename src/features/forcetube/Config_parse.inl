// forcetube (fork feature, Experimental): the ForceTube keys (forcetube*).
// Textual fragment, included by Config.cpp inside parse_blam_key(), after the grenade hand-spawn keys. Moved verbatim; not compiled on its own.
    if (_stricmp(key, "forcetube")        == 0) { g_cfg.force_tube = (v != 0.0); return true; }
    if (_stricmp(key, "forcetubekick")    == 0) { g_cfg.force_tube_kick = (int)clampf((float)v, 0.0f, 255.0f); return true; }
    if (_stricmp(key, "forcetuberadius")  == 0) { g_cfg.force_tube_radius = clampf((float)v, 0.05f, 5.0f); return true; }
    if (_stricmp(key, "forcetubefirems")  == 0) { g_cfg.force_tube_fire_ms = (int)clampf((float)v, 0.0f, 2000.0f); return true; }
    if (_stricmp(key, "forcetubechannel") == 0) { g_cfg.force_tube_channel = (int)clampf((float)v, 0.0f, 7.0f); return true; }
