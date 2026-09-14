// reloadvr (fork feature, Experimental): the per-weapon reload state keys (reloadstate*).
// Textual fragment, included by Config.cpp at namespace halo scope, beside the other hoisted key families. Moved verbatim; not compiled on its own.
// Per-weapon reload state (Gesture.cpp). Hoisted for the same C1061 reason as its siblings.
static bool parse_reloadstate_key(const char* key, double v) {
    if (_stricmp(key, "reloadstate")       == 0) { g_cfg.reload_state_id      = (int)clampf((float)v, 0.0f, 3.0f); return true; }
    if (_stricmp(key, "reloadstatesave")   == 0) { g_cfg.reload_state_save    = (int)clampf((float)v, 1.0f, 2.0f); return true; }
    if (_stricmp(key, "reloadstatehide")   == 0) { g_cfg.reload_state_hide    = (int)clampf((float)v, 0.0f, 3.0f); return true; }
    if (_stricmp(key, "reloadstatewaitms") == 0) { g_cfg.reload_state_wait_ms = (int)clampf((float)v, 0.0f, 10000.0f); return true; }
    if (_stricmp(key, "reloadstatedrop")   == 0) { g_cfg.reload_state_drop    = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "reloadstatedeath")  == 0) { g_cfg.reload_state_death   = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "reloadstatelevel")  == 0) { g_cfg.reload_state_level   = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "reloadstatelog")    == 0) { g_cfg.reload_state_log     = (v != 0.0); return true; }
    return false;
}
