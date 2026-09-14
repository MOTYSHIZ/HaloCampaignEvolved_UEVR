// reloadvr (fork feature, Experimental): the reload's own ticks (rack, montage, chamber, phantom, parts, mag hide, state mirror).
// Textual fragment, included by Gesture.cpp inside gesture_update(), after reload_update(). Moved verbatim; not compiled on its own.
    // The rack, the pump, the chamber and the rest of the reload's own ticks ride with it, not with
    // melee: switching melee off must not switch the slide off. They keep their pose gate.
    if (poses_ok && fork_reload) {
        slide_update(hpos);
        slide_montage_tick();
        anim_state_probe_tick();
        slide_fire_tick();
        slide_phantom_tick();
        slide_chamber_tick();
        slide_copy_tick();
        slide_part_tick();
        slide_node_write_tick();
    }
    // Per-weapon reload state: re-hide a magazine that is out on whichever actor renders the weapon
    // now, and mirror the live state into its record. Before the melee-off return, so switching melee
    // off never stops the reload state from saving.
    if (fork_reload) {
        mag_hide_enforce();
        reload_state_mirror();
    }
