// meleeleft (fork feature, Experimental): the off hand's own holster veto.
// Textual fragment, included by Holster.cpp at namespace halo scope, after holster_melee_veto(). Moved verbatim; not compiled on its own.
// The OFF hand's own veto. holster_melee_veto() above tests the AIM hand's zone proximity --
// correct for the aim-hand detector it was built for, and exactly wrong for a left punch: the
// right hand holding a rifle at chest height parks inside the pouch space and stood every left
// punch down (measured 2026-08-31, five punches at speed 4.4-9.4 all killed by it). This one
// tests the OFF hand's pouch proximity plus the same recent-action window; the armed-grenade
// case is holster_offhand_busy(), which the caller already checks.
bool holster_offhand_melee_veto() {
    if (!g_cfg.holster_enabled) return false;
    if (s_gnear_zone) return true;
    return (now_ticks() - s_last_action) < ms_to_ticks(g_cfg.holster_melee_veto_ms);
}
