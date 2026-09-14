// vehcam (fork feature, Experimental): the in-vehicle view, anchored forward to the vehicle (vehview).
// Textual fragment, included by Plugin.cpp inside the stereo pre-callback, before the view lock. Moved verbatim; not compiled on its own.
        // ---- IN-VEHICLE VIEW: ANCHOR FORWARD TO THE VEHICLE (vehview).
        //
        // The lock pins the rendered yaw to a room-anchored value, right on foot and exactly wrong
        // seated: the hog turns under a view held at a fixed world yaw. Anchor the base to the
        // vehicle's facing instead and let the headset add free-look. Runs BEFORE the view-lock and
        // stick-mode gates so the seat wins while mounted; unmounted it is a no-op.
        //
        // The branch runs for the WHOLE mount. At the mount edge the eased yaw primes from the game
        // camera and HOLDS until a heading is available. With vehfacing the vehicle object's own
        // orientation (+0x1D4) is the heading; without it the travel direction is used, hemisphere
        // chosen by continuity and healed by a slow pull toward the game camera at driving speed.
        // Eased at render rate (~100 ms), since the heading updates far slower than the frame.
        {
            static bool  s_veh_primed = false;
            static float s_veh_cur = 0.0f;
            static float s_veh_hemi = 0.0f;
            static std::chrono::steady_clock::time_point s_veh_last{};
            // The seated view is the SEAT CAMERA's orientation; without vehcam the stock chase camera is
            // showing, and re-basing and flattening that view is not what vehview is for.
            const bool seated = g_cfg.veh_view != 0 && g_cfg.veh_cam != 0
                             && halo::g_unit_mounted.load(std::memory_order_relaxed);
            if (!seated) {
                s_veh_primed = false;   // next mount re-primes from the fresh game camera
            } else {
                const float game_yaw = is_double
                    ? (float)reinterpret_cast<UEVR_Rotatord*>(rotation)->yaw
                    : rotation->yaw;
                const auto now = std::chrono::steady_clock::now();
                float rdt = s_veh_primed ? std::chrono::duration<float>(now - s_veh_last).count() : 0.0f;
                s_veh_last = now;
                if (rdt < 0.0f || rdt > 0.25f) rdt = 0.0f;
                if (!s_veh_primed) { s_veh_cur = game_yaw; s_veh_hemi = 0.0f; s_veh_primed = true; }

                auto wrap180 = [](float a) {
                    while (a > 180.0f) a -= 360.0f;
                    while (a < -180.0f) a += 360.0f;
                    return a;
                };
                float target = s_veh_cur;   // heading not armed: hold where the mount primed us
                bool  valid  = halo::g_veh_heading_valid.load(std::memory_order_relaxed);
                float travel = halo::g_veh_heading.load(std::memory_order_relaxed);
                if (g_cfg.veh_facing != 0 && halo::g_veh_fvalid.load(std::memory_order_relaxed)) {
                    const float fx = halo::g_veh_fx.load(std::memory_order_relaxed);
                    const float fy = halo::g_veh_fy.load(std::memory_order_relaxed);
                    if (fx * fx + fy * fy > 0.25f) {
                        // Blam frame -> UE: y is negated, as everywhere else in this project.
                        travel = wrap180((float)(std::atan2(-(double)fy, (double)fx) * 57.295779513)
                                         + g_cfg.veh_facing_bias);
                        valid = true;
                        s_veh_hemi = 0.0f;   // no hemisphere guessing needed against a real facing
                    }
                }
                if (valid) {
                    const float cand = wrap180(travel + s_veh_hemi);
                    const float alt  = wrap180(cand + 180.0f);
                    if (std::fabs(wrap180(alt - s_veh_cur)) + 30.0f < std::fabs(wrap180(cand - s_veh_cur))) {
                        s_veh_hemi = (s_veh_hemi == 0.0f) ? 180.0f : 0.0f;
                        target = alt;
                    } else {
                        target = cand;
                    }
                }
                const float k = (rdt / 0.10f > 1.0f) ? 1.0f : rdt / 0.10f;   // ~100 ms to settle
                s_veh_cur = wrap180(s_veh_cur + wrap180(target - s_veh_cur) * k);
                if (valid && halo::g_veh_speed.load(std::memory_order_relaxed) >= 1.0f) {
                    const float kg = (rdt / 2.0f > 1.0f) ? 1.0f : rdt / 2.0f;   // ~2 s hood healer
                    s_veh_cur = wrap180(s_veh_cur + wrap180(game_yaw - s_veh_cur) * kg);
                }

                if (is_double) {
                    auto* r = reinterpret_cast<UEVR_Rotatord*>(rotation);
                    r->yaw = (double)s_veh_cur;
                    if (g_cfg.veh_view_flat) { r->pitch = 0.0; r->roll = 0.0; }
                } else {
                    rotation->yaw = s_veh_cur;
                    if (g_cfg.veh_view_flat) { rotation->pitch = 0.0f; rotation->roll = 0.0f; }
                }
                // One line a second while seated: armed?, raw travel, chosen hemisphere, the eased
                // output, and the untouched game yaw.
                if (g_cfg.veh_log) {
                    static uint32_t n = 0;
                    if ((n++ % 90u) == 0u)
                        API::get()->log_info("[Halo-CampE-UEVR] VEHVIEW: valid=%d travel=%.1f hemi=%.0f eased=%.1f gameyaw=%.1f pos=(%.2f %.2f)",
                                             (int)valid, travel, s_veh_hemi, s_veh_cur, game_yaw,
                                             halo::g_unit_px.load(std::memory_order_relaxed),
                                             halo::g_unit_py.load(std::memory_order_relaxed));
                }
                g_dbg_view_in = s_veh_cur; g_dbg_view_out = s_veh_cur;
                halo::g_view_base_yaw.store(s_veh_cur, std::memory_order_relaxed);
                g_lock_primed = false;   // re-prime the on-foot lock when you dismount
                return;
            }
        }
