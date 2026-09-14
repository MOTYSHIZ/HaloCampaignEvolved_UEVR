// holsterpollthrow (fork feature, Experimental): the poll-path reconciliation and publish, and the carrier hand and swing direction in Blam units.
// Textual fragment, included by Holster.cpp inside holster_update(), before the tick release edge. Moved verbatim; not compiled on its own.
    // ---- POLL-PATH RECONCILIATION. The hook threw between ticks: press and aim hold are already
    // out the door; this is everything else the release branch does. Runs BEFORE the tick's own
    // release edge below -- s_grenade_armed drops here, so the same grenade cannot throw twice.
    if (g_pollthrow_fired.exchange(false, std::memory_order_relaxed)) {
        if (s_grenade_armed) {
            const bool coff = s_carry_off;
            s_grenade_armed = false;
            s_last_action = now_ticks();
            if (!coff && !s_unarmed) { set_weapon_hidden(false); s_unhide_ticks = 30; }
            haptic_on(coff ? off_is_right() : aim_is_right(), 0.10f, 1.0f);
            if (g_cfg.holster_log)
                API::get()->log_info("[Halo-CampE-UEVR] HOLSTER THROW (poll-rate release, %s hand)",
                                     coff ? "off" : "aim");
        }
    }

    // ---- POLL-PATH PUBLISH: the standing verdict the hook acts on. Armed grenade, carrier hand
    // OUTSIDE every pouch (in a pouch, a release is a put-back and stays the tick's call), and the
    // aim-hold direction from the current peak -- all at most one tick old at fire time.
    {
        unsigned short pmask = 0;
        if (g_cfg.holster_poll_throw && s_grenade_armed) {
            const bool coff = s_carry_off;
            const HolsterSlot czone = coff ? zone_g : zone_p;
            if (czone == HolsterSlot::None) {
                pmask = (unsigned short)((coff ? off_is_right() : aim_is_right())
                                         ? g_cfg.grip_mask_r : g_cfg.grip_mask_l);
                const Vec3& cpeak = coff ? s_gpeak_velw : s_peak_velw;
                const float vlen = std::sqrt(cpeak.x * cpeak.x + cpeak.y * cpeak.y + cpeak.z * cpeak.z);
                if (vlen > 0.2f) {
                    g_pollthrow_hy.store(wrap180(std::atan2(cpeak.x, -cpeak.z) * RAD2DEG
                                                 + g_cfg.aim_turn * g_turn_offset.load(std::memory_order_relaxed)),
                                         std::memory_order_relaxed);
                    g_pollthrow_hp.store(std::asin(std::fmax(-1.0f, std::fmin(1.0f, cpeak.y / vlen))) * RAD2DEG,
                                         std::memory_order_relaxed);
                    g_pollthrow_hold.store(true, std::memory_order_relaxed);
                } else {
                    g_pollthrow_hold.store(false, std::memory_order_relaxed);
                }
            }
        }
        g_pollthrow_mask.store(pmask, std::memory_order_relaxed);
    }

    // Carrier hand in Blam units, for the grenhand spawn-origin experiment. UE world / 304.8 with
    // Y negated -- the same fit that placed the vehicle camera (BlamDrive, unit+0x20 vs camera).
    if (s_grenade_armed) {
        const Vec3 cpos = s_carry_off ? gpos : pos;
        const Vec3 hw = holster_room_to_world(cpos, hpos);
        g_hand_blam_x.store(hw.x / 304.8f, std::memory_order_relaxed);
        g_hand_blam_y.store(-hw.y / 304.8f, std::memory_order_relaxed);
        g_hand_blam_z.store(hw.z / 304.8f, std::memory_order_relaxed);
        g_hand_blam_valid.store(true, std::memory_order_relaxed);
        // The peak swing direction, room frame -> Blam frame, by differencing room_to_world at
        // two points (the translation cancels, leaving exactly the rotation + swizzle + scale
        // that frame applies -- no second frame-math implementation to drift out of sync).
        const Vec3& cpk = s_carry_off ? s_gpeak_velw : s_peak_velw;
        const float pklen = std::sqrt(cpk.x * cpk.x + cpk.y * cpk.y + cpk.z * cpk.z);
        if (pklen > 0.2f) {
            const Vec3 pw = holster_room_to_world(Vec3{cpos.x + cpk.x, cpos.y + cpk.y, cpos.z + cpk.z}, hpos);
            float bx = (pw.x - hw.x), by = -(pw.y - hw.y), bz = (pw.z - hw.z);
            const float bl = std::sqrt(bx * bx + by * by + bz * bz);
            if (bl > 1e-4f) {
                g_throw_blam_x.store(bx / bl, std::memory_order_relaxed);
                g_throw_blam_y.store(by / bl, std::memory_order_relaxed);
                g_throw_blam_z.store(bz / bl, std::memory_order_relaxed);
                g_throw_blam_valid.store(true, std::memory_order_relaxed);
            }
        }
    }
