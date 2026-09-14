// headblock (fork feature, Experimental): the head block keys (headblock*).
// Textual fragment, included by Config.cpp inside parse_fork_port_key(), after the palette keys. Moved verbatim; not compiled on its own.
    if (_stricmp(key, "headblock")        == 0) { g_cfg.head_block         = (int)clampf((float)v, 0.0f, 3.0f); return true; }
    if (_stricmp(key, "headblockchannel") == 0) { g_cfg.head_block_channel = (int)clampf((float)v, 0.0f, 32.0f); return true; }
    if (_stricmp(key, "headblocklean")    == 0) { g_cfg.head_block_lean    = clampf((float)v, 0.0f, 200.0f); return true; }
    if (_stricmp(key, "headblocklog")     == 0) { g_cfg.head_block_log     = (int)clampf((float)v, 0.0f, 100000.0f); return true; }
    if (_stricmp(key, "headblockradius")  == 0) { g_cfg.head_block_radius  = clampf((float)v, 0.0f, 50.0f); return true; }
    if (_stricmp(key, "headblockrelease") == 0) { g_cfg.head_block_release = clampf((float)v, 1.0f, 2000.0f); return true; }
