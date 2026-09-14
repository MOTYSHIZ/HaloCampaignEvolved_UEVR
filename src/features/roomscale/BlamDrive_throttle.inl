// roomscale (fork feature, Experimental): throttle mode 3, the unit object's own throttle vectors, written on the sim thread.
// Textual fragment, included by BlamDrive.cpp at the end of publish_unit_state(). Moved verbatim; not compiled on its own.
    // ROOMSCALE THROTTLE, MODE 3: write the UNIT's own throttle vectors (found by BLAMUNIT-DUMP:
    // +0x250 fwd/+0x254 left, second copy at +0x25C/+0x260) from this sim-side path, so the
    // value sits there whenever the biped reads it. THE ONE THAT MOVES THE BIPED: eye speed =
    // 6.65 m/s x throttle, linear from 0.02 up, no floor -- modes that wrote the control record
    // survived but moved nothing (consumed before the write landed). Yields to the player's own
    // stick and to stick mode, same gates as the pad path.
    const bool thr_probe = (g_cfg.roomscale_thr_probe != 0)
                        && !g_stick_mode_active.load(std::memory_order_relaxed)
                        && g_pad_user_mag.load(std::memory_order_relaxed) < g_cfg.roomscale_stick;
    if ((g_cfg.roomscale_throttle == 3 && g_rs_thr_active.load(std::memory_order_relaxed)
         && !g_stick_mode_active.load(std::memory_order_relaxed)
         && g_pad_user_mag.load(std::memory_order_relaxed) < g_cfg.roomscale_stick) || thr_probe) {
        const uintptr_t o1 = (uintptr_t)g_cfg.blam_unit_throttle_off;
        const uintptr_t o2 = (uintptr_t)g_cfg.blam_unit_throttle_off2;
        if (o1 != 0 && !IsBadWritePtr((void*)(obj + o1), 8)) {
            // Candidate BODY-FACING vectors, published for the probe log (game thread): the flat
            // direction pairs the unit dump showed. Whichever angle tracks move_world exactly is
            // the frame the throttle is consumed in.
            if (!IsBadReadPtr((const void*)(obj + 0x1D4), 8)) {
                g_dbg_face[0].store(*(const float*)(obj + 0x1D4), std::memory_order_relaxed);
                g_dbg_face[1].store(*(const float*)(obj + 0x1D8), std::memory_order_relaxed);
                g_dbg_face[4].store(*(const float*)(obj + 0x1D4), std::memory_order_relaxed);
                g_dbg_face[5].store(*(const float*)(obj + 0x1D8), std::memory_order_relaxed);
            }
            if (!IsBadReadPtr((const void*)(obj + 0x1E0), 8)) {
                g_dbg_face[2].store(*(const float*)(obj + 0x1E0), std::memory_order_relaxed);
                g_dbg_face[3].store(*(const float*)(obj + 0x1E4), std::memory_order_relaxed);
            }
            // AIM-FRAME command, kept as FINAL after two measured attempts to do better both
            // lost: a body-forward (+0x50) basis was off by the torso twist and flipped 180 in
            // the biped's turn state; the game's own +0x1D4 basis fed back -- the biped rotates
            // that vector while moving, the re-projection chased it, and the probe walked in
            // curves. g_rs_thr_fwd/right are (fwd, right) in the AIM frame.
            float f, l;
            if (thr_probe) { f = (float)g_cfg.roomscale_thr_probe / 100.0f; l = 0.0f; }
            else {
                f = g_rs_thr_fwd.load(std::memory_order_relaxed);
                const float r = g_rs_thr_right.load(std::memory_order_relaxed);
                l = (g_cfg.blam_throttle_ysign < 0) ? -r : r;
            }
            *(float*)(obj + o1) = f; *(float*)(obj + o1 + 4) = l;
            if (o2 != 0 && !IsBadWritePtr((void*)(obj + o2), 8)) { *(float*)(obj + o2) = f; *(float*)(obj + o2 + 4) = l; }
            g_rs_thr_written.fetch_add(1, std::memory_order_relaxed);
        }
    }
