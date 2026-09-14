// wristhud (fork feature, Experimental): the radar and tracker keys (blip*, tracker*, wristradar*, wristhudblend/gain).
// Textual fragment, included by Config.cpp inside parse_blam_key(), after throwdump. Moved verbatim; not compiled on its own.
    if (_stricmp(key, "blipdump")    == 0) { g_cfg.blip_dump      = (int)v; return true; }
    if (_stricmp(key, "blipcolor") == 0) {
        // <hex class>,<r>,<g>,<b>. Repeatable; a repeat of the same class overwrites it, so a
        // live edit retunes a colour instead of exhausting the table.
        unsigned cls = 0; float c[3] = {1.0f, 1.0f, 1.0f};
        if (sscanf_s(val, "%x,%f,%f,%f", &cls, &c[0], &c[1], &c[2]) >= 2) {
            int slot = -1;
            for (int i = 0; i < g_cfg.blip_color_n; ++i)
                if (g_cfg.blip_color[i].cls == cls) { slot = i; break; }
            if (slot < 0 && g_cfg.blip_color_n < kMaxBlipColor) slot = g_cfg.blip_color_n++;
            if (slot >= 0) {
                // Bump the generation ONLY on a real change. The file is re-read every ~2 s, so
                // bumping per parse threw away the colour cache and rebuilt every render target
                // continuously -- a standing leak dressed up as a feature.
                BlipColor& e = g_cfg.blip_color[slot];
                if (e.cls != (uint32_t)cls || e.r != c[0] || e.g != c[1] || e.b != c[2]) {
                    ++g_cfg.blip_color_gen;
                    e.cls = (uint32_t)cls;
                    e.r = c[0]; e.g = c[1]; e.b = c[2];
                }
            }
        }
        return true;
    }
    if (_stricmp(key, "blipcolorother") == 0) {
        float c[3] = {g_cfg.blip_color_other[0], g_cfg.blip_color_other[1], g_cfg.blip_color_other[2]};
        sscanf_s(val, "%f,%f,%f", &c[0], &c[1], &c[2]);
        if (c[0] != g_cfg.blip_color_other[0] || c[1] != g_cfg.blip_color_other[1] ||
            c[2] != g_cfg.blip_color_other[2]) {
            ++g_cfg.blip_color_gen;
            g_cfg.blip_color_other[0] = c[0];
            g_cfg.blip_color_other[1] = c[1];
            g_cfg.blip_color_other[2] = c[2];
        }
        return true;
    }
    if (_stricmp(key, "blipname") == 0) {
        char m[64] = {0}; float c[3] = {1.0f, 1.0f, 1.0f};
        if (sscanf_s(val, "%63[^,],%f,%f,%f", m, (unsigned)sizeof(m), &c[0], &c[1], &c[2]) >= 2) {
            int slot = -1;
            for (int i = 0; i < g_cfg.blip_name_n; ++i)
                if (_stricmp(g_cfg.blip_name[i].match, m) == 0) { slot = i; break; }
            if (slot < 0 && g_cfg.blip_name_n < kMaxBlipName) slot = g_cfg.blip_name_n++;
            if (slot >= 0) {
                BlipName& e = g_cfg.blip_name[slot];
                if (_stricmp(e.match, m) != 0 || e.r != c[0] || e.g != c[1] || e.b != c[2]) {
                    ++g_cfg.blip_color_gen;
                    strncpy_s(e.match, m, _TRUNCATE);
                    e.r = c[0]; e.g = c[1]; e.b = c[2];
                }
            }
        }
        return true;
    }
    if (_stricmp(key, "blipbytes")   == 0) { g_cfg.blip_bytes     = (int)v; return true; }
    if (_stricmp(key, "trackerdump") == 0) { g_cfg.tracker_dump   = (int)v; return true; }
    if (_stricmp(key, "trackermid")  == 0) { g_cfg.tracker_mid    = (int)v; return true; }
    if (_stricmp(key, "wristradar")  == 0) { g_cfg.wrist_radar    = (v != 0.0); return true; }
    if (_stricmp(key, "wristradarblip") == 0) { g_cfg.wrist_radar_blip = clampf((float)v, 0.002f, 0.1f); return true; }
    if (_stricmp(key, "wristradargain") == 0) { g_cfg.wrist_radar_gain = clampf((float)v, 1.0f, 100000.0f); return true; }
    if (_stricmp(key, "wristradaraimsign") == 0) { g_cfg.wrist_radar_aimsign = (v < 0.0) ? -1.0f : 1.0f; return true; }
    if (_stricmp(key, "wristradarrot")  == 0) { g_cfg.wrist_radar_rot  = (float)v; return true; }
    if (_stricmp(key, "wristradarflip") == 0) { g_cfg.wrist_radar_flip = (v != 0.0) ? 1 : 0; return true; }
    if (_stricmp(key, "wristradarlog")  == 0) { g_cfg.wrist_radar_log  = (v != 0.0); return true; }
    if (_stricmp(key, "wristradartest") == 0) { g_cfg.wrist_radar_test = (int)v; return true; }
    if (_stricmp(key, "wristradarcenter") == 0) { sscanf_s(val, "%f,%f", &g_cfg.wrist_radar_center[0], &g_cfg.wrist_radar_center[1]); return true; }
    if (_stricmp(key, "wristradartilt") == 0) { g_cfg.wrist_radar_tilt = clampf((float)v, -90.0f, 90.0f); return true; }
    if (_stricmp(key, "wristhudblend") == 0) { g_cfg.wrist_hud_blend = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "wristhudblendr") == 0) { g_cfg.wrist_hud_blend_r = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "wristhudgainr") == 0) { g_cfg.wrist_hud_gain_r = clampf((float)v, 0.05f, 100.0f); return true; }
