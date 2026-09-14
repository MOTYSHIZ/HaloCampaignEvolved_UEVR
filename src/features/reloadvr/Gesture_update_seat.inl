// reloadvr (fork feature, Experimental): the MAG_HELD body: the well, the lift, the insert and the seat.
// Textual fragment, included by Gesture.cpp inside reload_update(), in case MagHeld after the grip check. Moved verbatim; not compiled on its own.
        if (!have_left || hand_r_p == nullptr) break;
        const Vec3& hand_r = *hand_r_p;
        const Vec3& head   = *head_p;
        // THE WELL. First choice: the weapon's OWN magazine component -- the one mag_hide_apply
        // just hid is the rendered magazine sitting in its well, so its live world location IS the
        // per-weapon insert point, on every weapon, with nothing to calibrate. The seat test then
        // runs in world space (the hand goes room->world through the same transform that places
        // every holster marker). Fallback when no component resolved: the old fixed point
        // reload_well_fwd along the aim direction from the aim hand. And the LIFT GATE either
        // way: the mag must have risen since it was grabbed. Without both, the log
        // (2026-08-16 12:07) shows the reload firing 214-224 ms after the belt grab, at the hip.
        float join = 1e9f;
        Vec3  well_world{}; bool have_well_world = false;
        if (auto* mc = s_mag_hidden.get()) {
            if (call_ret_vec3(mc, L"K2_GetComponentLocation", &well_world)) {
                have_well_world = true;
                well_world = reload_well_stabilize(mc, well_world, head);
                const Vec3 hlw = reload_hand_world(hand_l, head);
                const float wdx = hlw.x - well_world.x, wdy = hlw.y - well_world.y,
                            wdz = hlw.z - well_world.z;
                join = std::sqrt(wdx * wdx + wdy * wdy + wdz * wdz) / 100.0f;  // cm -> m
            }
        }
        if (!have_well_world) {
            Vec3 well = hand_r;
            const auto ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                                  : API::VR::get_right_controller_index();
            Vec3 apos{}; Quat aq{};
            if (ridx >= 0 && get_pose(ridx, &apos, &aq, /*use_aim=*/true)) {
                const Vec3 f = quat_forward(apply_aim_fix(aq));
                well = Vec3{hand_r.x + f.x * g_cfg.reload_well_fwd,
                            hand_r.y + f.y * g_cfg.reload_well_fwd,
                            hand_r.z + f.z * g_cfg.reload_well_fwd};
            }
            const float ddx = hand_l.x - well.x, ddy = hand_l.y - well.y, ddz = hand_l.z - well.z;
            join = std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
            well_world = holster_room_to_world(well, head);
        }
        reload_well_marker_update(true, well_world);
        const bool lifted = (hand_l.y - s_grab_y) >= g_cfg.reload_lift;
        if (g_cfg.reload_log) {
            static uint32_t s_rl = 0;
            if ((s_rl++ % 15u) == 0u)
                API::get()->log_info("[Halo-CampE-UEVR] RELOAD held: mag-to-well=%.0fcm lifted=%.0fcm (need <=%.0f, >=%.0f)",
                                     join * 100.0f, (hand_l.y - s_grab_y) * 100.0f,
                                     g_cfg.reload_join_dist * 100.0f, g_cfg.reload_lift * 100.0f);
        }
        // THE SLIDE, in place of the old four-tick debounce. Inside the capture radius (and
        // lifted) the magazine leaves the hand and travels into the well over reload_slide_ms;
        // the reload fires when it lands. Pulling the hand well clear mid-slide aborts it back to
        // the hand. The duration IS the debounce: a one-frame tracking glitch cannot complete it.
        {
            const long long nowt = now_ticks();
            // Target transform: the weapon's magazine component when we have it (its rotation
            // is the seated mag's, so the slide ends exactly on the real one); else the fallback
            // well point with no orientation, which slides position only.
            g_reload_slide_x.store(well_world.x, std::memory_order_relaxed);
            g_reload_slide_y.store(well_world.y, std::memory_order_relaxed);
            g_reload_slide_z.store(well_world.z, std::memory_order_relaxed);
            bool rot_ok = false;
            if (have_well_world) {
                if (auto* mc = s_mag_hidden.get()) {
                    Vec3 wrot{};
                    if (call_ret_vec3(mc, L"K2_GetComponentRotation", &wrot)) {
                        g_reload_slide_pitch.store(wrot.x, std::memory_order_relaxed);
                        g_reload_slide_yaw.store(wrot.y,   std::memory_order_relaxed);
                        g_reload_slide_roll.store(wrot.z,  std::memory_order_relaxed);
                        rot_ok = true;
                    }
                }
            }
            g_reload_slide_rot_valid.store(rot_ok, std::memory_order_relaxed);

            // THE WELL'S MOUTH (insert mode). The seat point is the seated mag's own centre,
            // inside the grip, so a radius around it only fires when the mag is nearly home.
            // The lock instead happens anywhere on the axis from reload_insert below the seat up
            // to the seat, within reload_join_dist of the axis SIDEWAYS; the abort is sideways too.
            bool  axis_ok = false; float lateral_m = join, d_axis_cm = 0.0f;
            if (g_cfg.reload_insert_mode == 1 && have_well_world && rot_ok) {
                const Quat qs = rotator_to_quat(g_reload_slide_pitch.load(std::memory_order_relaxed),
                                                g_reload_slide_yaw.load(std::memory_order_relaxed),
                                                g_reload_slide_roll.load(std::memory_order_relaxed));
                Vec3 up = quat_rotate(qs, Vec3{0.0f, 0.0f, 1.0f});
                up = Vec3{up.x * g_cfg.reload_insert_sign, up.y * g_cfg.reload_insert_sign, up.z * g_cfg.reload_insert_sign};
                const Vec3 hw = reload_hand_world(hand_l, head);
                const Vec3 r{hw.x - well_world.x, hw.y - well_world.y, hw.z - well_world.z};
                d_axis_cm = r.x * up.x + r.y * up.y + r.z * up.z;
                const Vec3 lat{r.x - up.x * d_axis_cm, r.y - up.y * d_axis_cm, r.z - up.z * d_axis_cm};
                lateral_m = std::sqrt(lat.x * lat.x + lat.y * lat.y + lat.z * lat.z) / 100.0f;
                axis_ok = true;
            }
            const float jd = g_cfg.reload_join_dist + reload_gate_pad_m();
            const bool in_mouth = axis_ok
                ? (lateral_m <= jd && d_axis_cm >= -(reload_insert_for_weapon() + jd) * 100.0f && d_axis_cm <= jd * 100.0f)
                : (join <= jd);
            const bool pulled_clear = axis_ok ? (lateral_m > jd * 2.5f || d_axis_cm < -(reload_insert_for_weapon() + jd * 2.5f) * 100.0f)
                                             : (join > jd * 2.5f);

            if (s_slide_start == 0) {
                if (in_mouth && lifted) {
                    s_slide_start = nowt;
                    g_reload_slide_t.store(0.0f, std::memory_order_relaxed);
                    ak_step_sound("seat");
                    if (g_cfg.reload_log)
                        API::get()->log_info("[Halo-CampE-UEVR] RELOAD slide begins at %.0fcm", join * 100.0f);
                }
            } else if (pulled_clear) {
                if (g_cfg.reload_log)
                    API::get()->log_info("[Halo-CampE-UEVR] RELOAD slide aborted, hand pulled clear (%.0fcm)", join * 100.0f);
                reload_slide_reset();
            } else {
                const float ms = (float)std::chrono::duration_cast<std::chrono::milliseconds>(
                    clock_t_::duration(nowt - s_slide_start)).count();
                bool done = false;
                if (g_cfg.reload_insert_mode == 1 && have_well_world && rot_ok) {
                    // HAND-DRIVEN. The mag snaps onto the well's axis (the short blend), then sits
                    // on that axis at the HAND's height below the seat; it is home when the hand
                    // has pushed it within reload_insert_done of the seat.
                    const float snap = std::fmin(1.0f, ms / (float)std::fmax(1, g_cfg.reload_slide_ms));
                    g_reload_slide_t.store(snap, std::memory_order_relaxed);
                    const Quat qs = rotator_to_quat(g_reload_slide_pitch.load(std::memory_order_relaxed),
                                                    g_reload_slide_yaw.load(std::memory_order_relaxed),
                                                    g_reload_slide_roll.load(std::memory_order_relaxed));
                    Vec3 up = quat_rotate(qs, Vec3{0.0f, 0.0f, 1.0f});
                    up = Vec3{up.x * g_cfg.reload_insert_sign, up.y * g_cfg.reload_insert_sign, up.z * g_cfg.reload_insert_sign};
                    const Vec3 hw = reload_hand_world(hand_l, head);
                    const float d = (hw.x - well_world.x) * up.x + (hw.y - well_world.y) * up.y + (hw.z - well_world.z) * up.z;   // cm along the axis, negative = below the seat
                    const float travel = reload_insert_for_weapon() * 100.0f;
                    float along = d; if (along > 0.0f) along = 0.0f; if (along < -travel) along = -travel;
                    g_reload_slide_x.store(well_world.x + up.x * along, std::memory_order_relaxed);
                    g_reload_slide_y.store(well_world.y + up.y * along, std::memory_order_relaxed);
                    g_reload_slide_z.store(well_world.z + up.z * along, std::memory_order_relaxed);
                    done = (d >= -g_cfg.reload_insert_done * 100.0f);
                    if (g_cfg.reload_log) {
                        static uint32_t s_il = 0;
                        if ((s_il++ % 15u) == 0u) API::get()->log_info("[Halo-CampE-UEVR] RELOAD insert: %.1f cm below the seat (home within %.1f)", -d, g_cfg.reload_insert_done * 100.0f);
                    }
                } else {
                    const float t = std::fmin(1.0f, ms / (float)std::fmax(1, g_cfg.reload_slide_ms));
                    g_reload_slide_t.store(t, std::memory_order_relaxed);
                    done = (t >= 1.0f);
                }
                if (done) {
                    reload_slide_reset();
                    // Locked back until racked ONLY if the gun was empty when the mag left --
                    // a chambered round needs no rack, that is how a pistol works.
                    const bool rack_first = g_cfg.slide_vr && g_cfg.slide_lock_reload && slide_weapon_ok() && slide_chamber_ok() && slide_rack_available()
                                            && (s_sl_empty_at_drop || weapon_in_list(g_cfg.slide_always_weapons));
                    if (rack_first) {
                        s_sl_lock_pending = true; s_sl_lock_frame = 0; s_sl_rack_done = false;
                    } else {
                        s_sl_locked_back = false;   // no rack will clear it: a stale flag kept the record alive and the rack zone live
                    }
                    if (rack_first && g_cfg.slide_chamber) {
                        // SLIDECHAMBER: the mag is in, the chamber is not. The rack ends the reload.
                        if (s_sl_press_pending || s_sl_press_due_at != 0) { s_sl_press_pending = false; s_sl_press_due_at = 0; s_sl_pressed_early = true; reload_press_now("seat (waited)"); }
                        s_sl_reload_due = true;
                        if (g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD seated on an empty gun: waiting for the rack");
                    } else if (s_sl_pressed_early) {
                        s_sl_pressed_early = false;
                        if (g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD seated, the sim reloaded at the drop");
                    } else {
                        s_sl_press_pending = false; s_sl_press_due_at = 0;
                        reload_press_now("seat");
                    }
                    if (g_cfg.anim_vars) anim_vars_begin();
                    audio_dump_begin();
                    ak_dump("seat");
                    ak_step_sound("seated");
                    if (g_cfg.anim_dump) {
                        s_anim_dump_until = nowt + ms_to_ticks(2500);
                        API::get()->log_info("[Halo-CampE-UEVR] ANIMDUMP begin (seat)");
                    }
                    set_state(ReloadState::Idle, "magazine seated, reload fired");
                }
            }
        }
