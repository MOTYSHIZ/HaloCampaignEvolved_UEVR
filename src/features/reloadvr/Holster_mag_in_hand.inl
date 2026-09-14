// reloadvr (fork feature, Experimental): the magazine in the hand: in-hand tuning and the slide into the well.
// Textual fragment, included by Holster.cpp inside holster_update(), in the magazine marker placement. Moved verbatim; not compiled on its own.
                Vec3  place = holster_room_to_world(gpos, hpos);
                float pd = pitchr * RAD2DEG, yd = yawr * RAD2DEG, rd = rollr * RAD2DEG;
                // The in-hand tuning (reloadhandoff / reloadhandrot), in the hand's frame: UE local
                // axes are x forward, y right, z up, so (right, up, forward) maps to (z, x, y).
                {
                    const Quat qh = rotator_to_quat(pd, yd, rd);
                    const Vec3 lo{g_cfg.reload_hand_off[2] * 100.0f, g_cfg.reload_hand_off[0] * 100.0f, g_cfg.reload_hand_off[1] * 100.0f};
                    const Vec3 wo = quat_rotate(qh, lo);
                    place = Vec3{place.x + wo.x, place.y + wo.y, place.z + wo.z};
                    const Quat qr = quat_mul(qh, rotator_to_quat(g_cfg.reload_hand_rot[0], g_cfg.reload_hand_rot[1], g_cfg.reload_hand_rot[2]));
                    quat_to_rotator(qr.x, qr.y, qr.z, qr.w, &pd, &yd, &rd);
                }
                // THE SLIDE (Gesture publishes it): from the hand's pose to the well's, eased.
                // Rotation goes through a normalised quaternion blend so the mag turns the short
                // way into the seated orientation instead of spinning through a rotator wrap.
                const float st = g_reload_slide_t.load(std::memory_order_relaxed);
                if (st >= 0.0f) {
                    const float e = st * st * (3.0f - 2.0f * st);   // smoothstep
                    const Vec3 tgt{g_reload_slide_x.load(std::memory_order_relaxed),
                                   g_reload_slide_y.load(std::memory_order_relaxed),
                                   g_reload_slide_z.load(std::memory_order_relaxed)};
                    place = Vec3{place.x + (tgt.x - place.x) * e,
                                 place.y + (tgt.y - place.y) * e,
                                 place.z + (tgt.z - place.z) * e};
                    if (g_reload_slide_rot_valid.load(std::memory_order_relaxed)) {
                        const Quat qa = rotator_to_quat(pd, yd, rd);
                        Quat qb = rotator_to_quat(g_reload_slide_pitch.load(std::memory_order_relaxed),
                                                  g_reload_slide_yaw.load(std::memory_order_relaxed),
                                                  g_reload_slide_roll.load(std::memory_order_relaxed));
                        float d = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w;
                        if (d < 0.0f) { qb.x = -qb.x; qb.y = -qb.y; qb.z = -qb.z; qb.w = -qb.w; }
                        Quat q{qa.x + (qb.x - qa.x) * e, qa.y + (qb.y - qa.y) * e,
                               qa.z + (qb.z - qa.z) * e, qa.w + (qb.w - qa.w) * e};
                        const float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
                        if (n > 1e-6f) { q.x /= n; q.y /= n; q.z /= n; q.w /= n; }
                        quat_to_rotator(q.x, q.y, q.z, q.w, &pd, &yd, &rd);
                    }
                }
