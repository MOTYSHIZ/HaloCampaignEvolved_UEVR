// roomscale (fork feature, Experimental): the walk out of the head offset through the game's own movement.
// Textual fragment, included by Plugin.cpp inside update(), in the HMD translation leash block, before the leash itself. Moved verbatim; not compiled on its own.
            // ---- ROOMSCALE: walk the player out of the head offset before the leash absorbs it.
            //
            // Measured with roomscalelog: full stick moves the eye at 3.0-4.7 m/s and the loop
            // (command -> pawn -> eye read) is ~0.1 s late, so a 35 cm offset commanded at gain 4
            // overshot by 32 cm and flipped to full stick the other way -- bang-bang with a lag.
            // And after a stick run, the pawn's own deceleration was credited to roomscale, so
            // the origin slid 0.74 m in 0.5 s and the offset ballooned to 0.8 m. Hence:
            //   1) credit only motion roomscale could have caused: capped by the commanded speed
            //      -- never the player's momentum;
            //   2) command from the offset MINUS the travel already in flight (last roomscale_lat
            //      seconds of commands x speed), so the controller stops chasing what is on its way;
            //   3) hysteresis on the deadband (start above roomscale_dead, stop below 0.8x).
            // Speed and latency are config so they can be set from the per-tick log, not assumed.
            // THROTTLE MODE (roomscale_throttle=3): the command is written into the unit object's
            // own throttle vectors on the sim thread (BlamDrive), so there is no deadzone, no
            // floor and no pulsing -- throttle t is speed t x roomscale_thr_speed, down to zero.
            const bool rs_thr = (g_cfg.roomscale_throttle == 3) && (g_cfg.blam_unit_throttle_off != 0);
            auto rs_speed_of = [&](float stick) -> float {
                if (rs_thr) return (stick <= 0.0f) ? 0.0f : g_cfg.roomscale_thr_speed * stick;
                const float dzc = g_cfg.roomscale_dz;
                if (stick <= dzc) return 0.0f;
                return g_cfg.roomscale_speed * (stick - dzc) / (1.0f - dzc);
            };
            auto rs_stick_for = [&](float v) -> float {
                if (v <= 0.0f) return 0.0f;
                if (rs_thr) return v / g_cfg.roomscale_thr_speed;
                const float dzc = g_cfg.roomscale_dz;
                return dzc + (1.0f - dzc) * (v / g_cfg.roomscale_speed);
            };
            // Never while MOUNTED, whatever stick mode says: with stickmode=0 a seat is not stick
            // mode, and roomscale would drive the rider's unit from the headset.
            const bool rs_ok = g_cfg.roomscale && !g_stick_mode.load() && !g_in_menu.load()
                            && !halo::g_unit_mounted.load(std::memory_order_relaxed)
                            && g_rig_parent != nullptr;
            static Vec3 s_rs_prev_eye{}; static bool s_rs_have_prev = false;
            static std::chrono::steady_clock::time_point s_rs_prev_t{}; static bool s_rs_have_t = false;
            static float s_rs_cmd_dir_x = 0.0f, s_rs_cmd_dir_z = 0.0f, s_rs_cmd_mag = 0.0f;   // room frame, last tick
            static bool  s_rs_engaged = false;
            static float s_rs_vavg = 0.0f;         // EMA of commanded eye speed, ~150 ms
            static int   s_rs_pulse_left = 0;      // ticks left in the current on-block
            // In-flight ring: (room dx, room dz) metres commanded per tick, oldest dropped after roomscale_lat.
            constexpr int kRsRing = 64;
            static float s_rs_ring_x[kRsRing]{}, s_rs_ring_z[kRsRing]{}, s_rs_ring_t[kRsRing]{}; static int s_rs_ring_n = 0;
            const auto now_t = std::chrono::steady_clock::now();
            const float dt = s_rs_have_t ? std::chrono::duration<float>(now_t - s_rs_prev_t).count() : 0.0f;
            s_rs_prev_t = now_t; s_rs_have_t = true;
            const float now_s = std::chrono::duration<float>(now_t.time_since_epoch()).count();

            Vec3 eye{};
            const bool have_eye = rs_ok && call_ret_vec3(g_rig_parent, L"K2_GetComponentLocation", &eye);
            static uint32_t s_rs_inj_prev = 0;
            const uint32_t inj_now = rs_thr ? halo::g_rs_thr_written.load(std::memory_order_relaxed)
                                            : g_rs_injected.load(std::memory_order_relaxed);
            const bool rs_drove = (inj_now != s_rs_inj_prev);
            s_rs_inj_prev = inj_now;
            float credit = 0.0f, eye_room_x = 0.0f, eye_room_z = 0.0f, expected = 0.0f;
            float log_ewx = 0.0f, log_ewy = 0.0f;
            if (have_eye && s_rs_have_prev) { log_ewx = (eye.x - s_rs_prev_eye.x) * 0.01f; log_ewy = (eye.y - s_rs_prev_eye.y) * 0.01f; }
            // ---- INVOLUNTARY-MOTION STAND-DOWN (roomscalemaxspeed / roomscalestanddown).
            //
            // "My body flies away from me -- usually jumping, hitting a rock." Those are the
            // moments the game moves the biped for reasons roomscale neither caused nor can act
            // on, and while that is happening our command and the game's own momentum COMPOUND:
            // air control and knockback both accept stick input, so a standing command becomes a
            // launch. Roomscale can only ever move the eye at its own speed; anything travelling
            // well above that is not ours. Stand down and let the physics finish. Strictly
            // SUBTRACTIVE: it can only make roomscale do less, never more.
            static float s_rs_standdown = 0.0f;
            if (dt > 0.0f) {
                if (have_eye && s_rs_have_prev && g_cfg.roomscale_max_speed > 0.0f) {
                    const float espd = std::sqrt(log_ewx * log_ewx + log_ewy * log_ewy) / dt;
                    if (espd > g_cfg.roomscale_max_speed) {
                        if (s_rs_standdown <= 0.0f && g_cfg.roomscale_log)
                            API::get()->log_info("[Halo-CampE-UEVR] ROOMSCALE: stand-down, eye %.1f m/s > %.1f (thrown, not walking)",
                                                 espd, g_cfg.roomscale_max_speed);
                        s_rs_standdown = g_cfg.roomscale_standdown;
                    }
                }
                if (s_rs_standdown > 0.0f) s_rs_standdown -= dt;
            }
            const bool rs_thrown = (s_rs_standdown > 0.0f);
            if (have_eye && s_rs_have_prev && rs_drove && s_rs_cmd_mag > 0.0f && dt > 0.0f) {
                const float ex = (eye.x - s_rs_prev_eye.x) * 0.01f, ey = (eye.y - s_rs_prev_eye.y) * 0.01f;   // UE cm -> m
                // World -> room: undo the view-lock yaw, then UE (x fwd, y right) -> VR (x right, z back).
                const float vy = halo::g_view_base_yaw.load() * DEG2RAD;
                const float rx =  ex * std::cos(vy) + ey * std::sin(vy);
                const float ry = -ex * std::sin(vy) + ey * std::cos(vy);
                eye_room_x = ry; eye_room_z = -rx;
                // CREDIT = THIS TICK'S EYE DELTA, SIGNED AND MAGNITUDE-CAPPED. Two wrong versions
                // preceded this: a zero-floored projection counted only jitter's positive half and
                // credited 5-10x the real travel (the origin slid toward the head while the body
                // stood still); a 100 ms low-passed velocity was numerically right but LATE, so
                // the origin trailed the body and kept sliding after it stopped. Signed and
                // per-tick, jitter cancels in the sum and the origin tracks with zero lag.
                // The cap has a FLOOR: as the head stops the command dies faster than the biped
                // decelerates, so "1.25 x recent commanded speed" collapsed under the body's real
                // catch-up and the last ~1 cm went uncredited -- the view drifted after every
                // stop. 0.7 m/s covers any roomscale catch-up (0.2-0.5 m/s) and still rejects
                // teleports and stick-run momentum, which are metres per second.
                expected = (std::max)(s_rs_vavg, 0.7f) * dt;
                // FULL 2D DELTA, magnitude-capped -- not its projection onto the command. The
                // biped's real travel is ~6 deg off the command (the aim-frame wart), and a
                // projection discarded the perpendicular part of every catch-up: the body went
                // there, the origin did not follow, the world slid by it -- 5-30% of each lean.
                const float em2 = std::sqrt(eye_room_x * eye_room_x + eye_room_z * eye_room_z);
                const float cap = 1.25f * expected;
                const float kk = (em2 > cap && em2 > 1.0e-9f) ? (cap / em2) : 1.0f;
                const float cx = eye_room_x * kk, cz = eye_room_z * kk;
                credit = cx * s_rs_cmd_dir_x + cz * s_rs_cmd_dir_z;      // logged: along-command part
                if (em2 > 1.0e-6f) { nx += cx; nz += cz; moved = true; }
            }
            if (have_eye) { s_rs_prev_eye = eye; s_rs_have_prev = true; } else { s_rs_have_prev = false; }

            // Remaining lateral offset after the slide, in room space.
            const float rdx = hp.x - nx, rdz = hp.z - nz;
            const float rlat = std::sqrt(rdx * rdx + rdz * rdz);
            // Drop ring entries older than roomscale_lat and sum what is still in flight.
            float fl_x = 0.0f, fl_z = 0.0f;
            {
                int w = 0;
                for (int i = 0; i < s_rs_ring_n; ++i) {
                    if (now_s - s_rs_ring_t[i] <= g_cfg.roomscale_lat) {
                        s_rs_ring_x[w] = s_rs_ring_x[i]; s_rs_ring_z[w] = s_rs_ring_z[i]; s_rs_ring_t[w] = s_rs_ring_t[i]; ++w;
                        fl_x += s_rs_ring_x[i]; fl_z += s_rs_ring_z[i];
                    }
                }
                s_rs_ring_n = w;
            }
            const float pdx = rdx - fl_x, pdz = rdz - fl_z;      // predicted offset once in-flight travel lands
            // HEAD VELOCITY (room frame, ~50 ms low-pass) for feed-forward: command the biped at
            // the speed the head is actually moving, so the body leaves and arrives WITH the head
            // instead of a quarter-second behind it. The offset term then only mops up.
            static float s_hp_px = 0.0f, s_hp_pz = 0.0f; static bool s_hpv_have = false;
            static float s_vh_x = 0.0f, s_vh_z = 0.0f;
            if (dt > 0.0f && s_hpv_have) {
                const float ivx = (hp.x - s_hp_px) / dt, ivz = (hp.z - s_hp_pz) / dt;
                const float av = clampf(dt / 0.05f, 0.0f, 1.0f);
                if (std::fabs(ivx) < 6.0f && std::fabs(ivz) < 6.0f) { s_vh_x += (ivx - s_vh_x) * av; s_vh_z += (ivz - s_vh_z) * av; }
            }
            s_hp_px = hp.x; s_hp_pz = hp.z; s_hpv_have = true;
            // Hysteresis on the measured offset.
            if (!s_rs_engaged && rlat > g_cfg.roomscale_dead) s_rs_engaged = true;
            if (s_rs_engaged && rlat < 0.8f * g_cfg.roomscale_dead) { s_rs_engaged = false; s_rs_pulse_left = 0; }
            bool rs_cmd = false;
            float cmd_lx = 0.0f, cmd_ly = 0.0f;
            // Desired room-space velocity = gain x predicted offset + ff x head velocity.
            const float cvx = pdx * g_cfg.roomscale_gain + s_vh_x * g_cfg.roomscale_ff;
            const float cvz = pdz * g_cfg.roomscale_gain + s_vh_z * g_cfg.roomscale_ff;
            const float cvm = std::sqrt(cvx * cvx + cvz * cvz);
            if (rs_thrown) { s_rs_engaged = false; s_rs_cmd_mag = 0.0f; s_rs_pulse_left = 0; }
            if (rs_ok && !rs_thrown && s_rs_engaged && cvm > 1.0e-4f) {
                double cp_ = 0.0, cy_ = 0.0;
                if (read_control_rotation(&cp_, &cy_, nullptr)) {
                    // Command direction in ROOM space (unit) and magnitude from the desired velocity.
                    const float ux_r = cvx / cvm, uz_r = cvz / cvm;
                    float ux_r_f = ux_r, uz_r_f = uz_r;
                    const float v_want = cvm;
                    float mag = rs_stick_for(v_want);
                    if (mag > 1.0f) mag = 1.0f;
                    // DUTY-CYCLE BELOW THE GAME'S MINIMUM SPEED. The stick has no slow walk:
                    // nothing below ~0.33, and the floor already moves the eye at ~0.5 m/s, so
                    // creeping physically came out as 4-8 cm hops once a second -- "it snaps in
                    // increments". Below the floor, pulse the floor command a fraction of ticks
                    // equal to wanted/floor speed (accumulator, so the fraction is exact); block
                    // pulses, because a single 8 ms pulse does not move the character over its
                    // inertia while a 25 ms one does.
                    bool pulse_on = true;
                    if (!rs_thr && mag < g_cfg.roomscale_min) {
                        const float v_floor = rs_speed_of(g_cfg.roomscale_min);
                        const float duty = (v_floor > 1.0e-4f) ? clampf(v_want / v_floor, 0.0f, 1.0f) : 1.0f;
                        const int kBlock = (g_cfg.roomscale_pulse < 1) ? 1 : g_cfg.roomscale_pulse;
                        static float s_duty_acc = 0.0f;
                        if (s_rs_pulse_left > 0) { --s_rs_pulse_left; pulse_on = true; }
                        else {
                            s_duty_acc += duty * kBlock;
                            if (s_duty_acc >= (float)kBlock) { s_duty_acc -= (float)kBlock; s_rs_pulse_left = kBlock - 1; pulse_on = true; }
                            else pulse_on = false;
                        }
                        mag = g_cfg.roomscale_min;
                    }
                    // Low-pass the direction/magnitude (~40 ms) so one noisy tick cannot flip it.
                    {
                        static float s_fx = 0.0f, s_fz = 0.0f; static bool s_have_f = false;
                        const float af = (dt > 0.0f) ? clampf(dt / 0.04f, 0.0f, 1.0f) : 1.0f;
                        const float tx = ux_r * mag, tz = uz_r * mag;
                        if (!s_have_f || s_rs_cmd_mag <= 0.0f) { s_fx = tx; s_fz = tz; s_have_f = true; }
                        else { s_fx += (tx - s_fx) * af; s_fz += (tz - s_fz) * af; }
                        const float fm = std::sqrt(s_fx * s_fx + s_fz * s_fz);
                        if (fm > 1.0e-4f) { mag = (fm > 1.0f) ? 1.0f : fm; ux_r_f = s_fx / fm; uz_r_f = s_fz / fm; }
                        else { ux_r_f = ux_r; uz_r_f = uz_r; }
                    }
                    // Room (x right, z back) -> UE room (x fwd = -z, y right = x) -> world by view yaw -> aim frame.
                    const float vy2 = halo::g_view_base_yaw.load() * DEG2RAD;
                    const float ux = -uz_r_f, uy = ux_r_f;
                    const float wx = ux * std::cos(vy2) - uy * std::sin(vy2);
                    const float wy = ux * std::sin(vy2) + uy * std::cos(vy2);
                    const float ay2 = (float)cy_ * DEG2RAD;
                    const float sf = wx * std::cos(ay2) + wy * std::sin(ay2);
                    const float sr = -wx * std::sin(ay2) + wy * std::cos(ay2);
                    if (!pulse_on) mag = 0.0f;
                    cmd_lx = sr * mag; cmd_ly = sf * mag;
                    if (rs_thr) {
                        // AIM-FRAME command (fwd, right) -- final; a body-forward basis was off by
                        // the torso twist and flipped 180 in the biped's turn state, and the
                        // game's own +0x1D4 basis fed back (the biped rotates it while moving, the
                        // re-projection chased it, and the probe walked in curves). The aim is the
                        // one external, stable reference and matches the consumer everywhere
                        // except near the ~80 deg torso-twist clamp, the accepted residual.
                        halo::g_rs_thr_fwd.store(cmd_ly, std::memory_order_relaxed);
                        halo::g_rs_thr_right.store(cmd_lx, std::memory_order_relaxed);
                    } else {
                        g_rs_lx.store(cmd_lx, std::memory_order_relaxed);
                        g_rs_ly.store(cmd_ly, std::memory_order_relaxed);
                    }
                    s_rs_cmd_dir_x = ux_r_f; s_rs_cmd_dir_z = uz_r_f; s_rs_cmd_mag = mag;
                    // Book the travel this tick's command will produce (lands after roomscale_lat).
                    if (dt > 0.0f && s_rs_ring_n < kRsRing) {
                        const float tr = rs_speed_of(mag) * dt;
                        s_rs_ring_x[s_rs_ring_n] = ux_r_f * tr; s_rs_ring_z[s_rs_ring_n] = uz_r_f * tr; s_rs_ring_t[s_rs_ring_n] = now_s; ++s_rs_ring_n;
                    }
                    rs_cmd = true;
                }
            }
            if (!rs_cmd) {
                g_rs_lx.store(0.0f, std::memory_order_relaxed); g_rs_ly.store(0.0f, std::memory_order_relaxed);
                halo::g_rs_thr_fwd.store(0.0f, std::memory_order_relaxed); halo::g_rs_thr_right.store(0.0f, std::memory_order_relaxed);
                s_rs_cmd_mag = 0.0f;
            }
            // Only ONE delivery path is armed: the unit write in throttle mode, else the stick.
            // The throttle write also yields to the player's own stick (checked on the sim thread
            // from g_pad_user_mag), so a deliberate push is still locomotion.
            g_rs_active.store(rs_cmd && !rs_thr, std::memory_order_relaxed);
            halo::g_rs_thr_active.store(rs_cmd && rs_thr, std::memory_order_relaxed);
            {
                const float ae = (dt > 0.0f) ? clampf(dt / 0.15f, 0.0f, 1.0f) : 1.0f;
                const float v_now = rs_speed_of(s_rs_cmd_mag);
                s_rs_vavg += (v_now - s_rs_vavg) * ae;
                if (!rs_cmd && s_rs_vavg < 1.0e-3f) s_rs_vavg = 0.0f;
            }
            if (g_cfg.roomscale_log && rs_ok) {
                // Every tick while engaged or a command was standing, else 1 in 64.
                static uint32_t s_rs_log = 0; static int s_rs_tail = 0;
                if (rs_cmd || rs_drove) s_rs_tail = 30; else if (s_rs_tail > 0) --s_rs_tail;
                if (s_rs_tail > 0 || (s_rs_log++ % 64u) == 0u) {
                    API::get()->log_info("[Halo-CampE-UEVR] ROOMSCALE dt=%.1fms off=(%.3f %.3f)|%.3f| pred=(%.3f %.3f) infl=(%.3f %.3f) "
                                         "cmd=(lx %.2f ly %.2f)mag%.2f eng=%d drove=%d eyeD_room=(%.3f %.3f) exp=%.3f credit=%.3f "
                                         "origin=(%.3f %.3f %.3f) userstick=%.2f inj=%u hp=(%.3f %.3f) vh=(%.3f %.3f)",
                                         dt * 1000.0f, rdx, rdz, rlat, pdx, pdz, fl_x, fl_z, cmd_lx, cmd_ly, s_rs_cmd_mag,
                                         (int)s_rs_engaged, (int)rs_drove, eye_room_x, eye_room_z, expected, credit,
                                         nx, ny, nz, g_rs_user_stick.load(), inj_now, hp.x, hp.z, s_vh_x, s_vh_z);
                }
            }
