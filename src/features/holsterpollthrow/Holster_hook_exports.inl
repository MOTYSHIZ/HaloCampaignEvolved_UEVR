// holsterpollthrow (fork feature, Experimental): the XInput-cadence throw note and the hand/direction exports (the grenade mesh getters here also feed the wrist radar).
// Textual fragment, included by Holster.cpp at namespace halo scope, after holster_mag_hand_in(). Moved verbatim; not compiled on its own.
// XINPUT HOOK CADENCE -- clocks and atomics only, per the rule on that callback (no poses, no
// reflection, no logging, no haptics; all of that is the tick's, before or after). Fires the
// synthetic throw press on the carrier grip's falling edge. Called with the RAW buttons before
// any remapping, and BEFORE the press mask is composed into the same poll -- so the throw the
// player just released goes out in the very report that shows the grip open.
void holster_note_buttons(unsigned short buttons) {
    static unsigned short s_prev = 0;
    const unsigned short prev = s_prev;
    s_prev = buttons;
    const unsigned short mask = g_pollthrow_mask.load(std::memory_order_relaxed);
    if (mask == 0) return;
    if ((prev & mask) == 0 || (buttons & mask) != 0) return;   // fire on held -> released only
    g_pollthrow_mask.store(0, std::memory_order_relaxed);      // one shot; the tick re-arms
    g_holster_throw_until.store(now_ticks() + ms_to_ticks(g_cfg.holster_press_ms),
                                std::memory_order_relaxed);
    if (g_pollthrow_hold.load(std::memory_order_relaxed) && g_cfg.holster_aim_hold_ms > 0) {
        g_melee_aim_ctrl_yaw.store(g_pollthrow_hy.load(std::memory_order_relaxed), std::memory_order_relaxed);
        g_melee_aim_ctrl_pitch.store(g_pollthrow_hp.load(std::memory_order_relaxed), std::memory_order_relaxed);
        g_melee_aim_hold_until.store(now_ticks() + ms_to_ticks(g_cfg.holster_aim_hold_ms),
                                     std::memory_order_relaxed);
    }
    g_pollthrow_fired.store(true, std::memory_order_relaxed);
}

// The carrier hand's last published Blam-unit position (grenhand). Safe on any thread.
bool holster_hand_blam(float* x, float* y, float* z) {
    if (!g_hand_blam_valid.load(std::memory_order_relaxed)) return false;
    *x = g_hand_blam_x.load(std::memory_order_relaxed);
    *y = g_hand_blam_y.load(std::memory_order_relaxed);
    *z = g_hand_blam_z.load(std::memory_order_relaxed);
    return true;
}
// The resolved grenade meshes, for the wrist radar's blips (frag = human, plasma = covenant --
// the factions' own ordnance as their marker art). Game thread; may be null until resolved.
uevr::API::UObject* holster_mesh_frag()   { return s_mesh_frag.get(); }
uevr::API::UObject* holster_mesh_plasma() { return s_mesh_plasma.get(); }

// The swing's peak direction in Blam units, normalized (greninstant). Safe on any thread.
bool holster_throw_blam_dir(float* x, float* y, float* z) {
    if (!g_throw_blam_valid.load(std::memory_order_relaxed)) return false;
    *x = g_throw_blam_x.load(std::memory_order_relaxed);
    *y = g_throw_blam_y.load(std::memory_order_relaxed);
    *z = g_throw_blam_z.load(std::memory_order_relaxed);
    return true;
}
