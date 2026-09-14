// reloadvr (fork feature, Experimental): a weapon change cancels the reload in flight.
// Textual fragment, included by Gesture.cpp inside reload_update(), after the button edge. Moved verbatim; not compiled on its own.
    // ---- A WEAPON CHANGE CANCELS THE RELOAD IN FLIGHT. The gesture is a state machine and
    // nothing told it the weapon changed: pull the mag on gun A, swap to gun B, and the fire
    // suppression follows the PLAYER, not the gun -- the player could not shoot the fresh weapon
    // until he performed a gesture that belonged to the previous one. The magazine you pulled was
    // gun A's; gun B arrives in whatever state the game has it. So the reload records which
    // weapon's mag came out, and the moment a DIFFERENT non-empty weapon is in hand the state
    // returns to Idle. Non-empty on both sides on purpose: a transient empty key (weapon lowered
    // for a frame, swap animation) must not cancel a legitimate reload of the same gun.
    // reloadstate 1-3 replaces both legacy blocks below (per-weapon records, Config.hpp).
    if (g_cfg.reload_state_id != 0) reload_state_track();
    if (g_cfg.reload_state_id == 0 && s_reload != ReloadState::Idle) {
        const std::string wk = weapon_key();
        if (!wk.empty() && !s_reload_weapon.empty() && wk != s_reload_weapon) {
            // Remember the gun that left with its mag out, then cancel for the gun in hand.
            WpnMem* m = wpn_mem_slot(s_reload_weapon);
            m->key = s_reload_weapon; m->mag_out = true; m->chamber_left = s_sl_chamber_left;
            m->empty = s_sl_empty_at_drop; m->lock_pending = s_sl_lock_pending; m->true_empty = s_true_empty;
            if (g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD remembered %s: mag out, %d chambered", s_reload_weapon.c_str(), s_sl_chamber_left);
            s_sl_lock_pending = false; s_sl_reload_due = false; s_sl_pressed_early = false; s_sl_locked_back = false; s_sl_press_pending = false; s_sl_press_due_at = 0;
            set_state(ReloadState::Idle, "weapon changed, reload cancelled");
        }
    }
    // ...and a gun that comes back with its mag still out picks up where it left off.
    if (g_cfg.reload_state_id == 0) {
        static std::string s_last_key;
        const std::string wk = weapon_key();
        if (!wk.empty() && wk != s_last_key) {
            s_last_key = wk;
            if (s_reload == ReloadState::Idle) {
                if (WpnMem* m = wpn_mem_find(wk)) {
                    if (m->mag_out) {
                        s_reload_weapon = wk;
                        s_sl_chamber_left = m->chamber_left; s_sl_empty_at_drop = m->empty;
                        s_sl_lock_pending = m->lock_pending; s_true_empty = m->true_empty;
                        s_restoring_mag_out = true;
                        set_state(ReloadState::MagOut, "weapon returned with its mag out");
                        s_restoring_mag_out = false;
                    }
                    wpn_mem_clear(wk);
                }
            }
        }
    }
