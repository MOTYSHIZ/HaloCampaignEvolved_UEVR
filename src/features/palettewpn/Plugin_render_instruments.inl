// palettewpn (fork feature, Experimental): the render-thread instruments: readback, SOCKROT, SAMEINST, RAYGUN, FPPIN, FPMESH.
// Textual fragment, included by Plugin.cpp inside the stereo pre-callback, after the marker camera publish. Moved verbatim; not compiled on its own.
        // The RENDERED base yaw, published for the palette's frame math (see the declaration).
        halo::g_view_base_yaw.store(g_dbg_view_out.load(), std::memory_order_relaxed);

        // READBACK: the last point before the draw. Does the palette still hold our bytes?
        if (index == 0) halo::blam_palette_readback_probe();
        // ---- SOCKROT (2026-09-12). THE DRAWN WEAPON'S ROTATION, never once measured.
        //
        // Everything upstream is proven clean. READBACK says the palette holds our exact bytes at
        // draw time (0.000 cm, 0.02 deg of float noise). RELSTOCK says the stock node relations
        // are dead constants (per-frame change 0.0000). The publisher is transparent (gain 1.01 in
        // every band). The camera is not the carrier -- palettelocal removed it entirely and the
        // judder survived. So the palette contains precisely what we intend.
        //
        // And yet point 12, the only drawn-result recorder in this project, logs the socket's
        // POSITION only. The complaint is rotational, and a socket sitting on the wrist barely
        // moves in position while its ROTATION swings the muzzle 60 cm away. So the one quantity
        // that corresponds to the symptom has never been recorded. rig_socket_world_rot has
        // existed the whole time and was never called from here.
        if (index == 0 && g_cfg.palette_weapon_log && palette_weapon_mode()) {
            auto* srcomp = rig_tracked_component();
            Vec3 srot{};
            if (srcomp != nullptr && rig_socket_world_rot(srcomp, L"PrimaryWeapon", &srot)) {
                const Quat sq = rotator_to_quat(srot.x, srot.y, srot.z);
                halo::blam_palette_sockrot_probe(sq.x, sq.y, sq.z, sq.w);
                // SAMEINST: component rotation and aim read right beside the socket, same callback,
                // same thread, so nothing published by another thread enters the comparison.
                Vec3 crot2{};
                if (call_ret_vec3(srcomp, L"K2_GetComponentRotation", &crot2)) {
                    const Quat cq2 = rotator_to_quat(crot2.x, crot2.y, crot2.z);
                    double ap = 0.0, ay = 0.0;
                    const bool aok = read_control_rotation_hook(&ap, &ay);
                    halo::blam_palette_sameinst_probe(sq.x, sq.y, sq.z, sq.w, cq2.x, cq2.y, cq2.z, cq2.w,
                                                      (float)ap, (float)ay, aok);
                }
            }
        }
        stomp_sample(index == 0 ? 1 : 2);
        if (index == 0) halo::blam_palette_stamp_bank();
        // Point 12: the RENDERED gun, the chain's final output. PrimaryWeapon socket of the
        // posed skeleton, world cm, once per frame. Judder that survives every upstream zero
        // must appear here as a back-and-forth world path -- and if this path is smooth while
        // the headset still shows judder, the defect is beyond the skeleton (view/reprojection).
        if (index == 0 && g_cfg.stomp_log != 0) {
            auto* s12 = rig_tracked_component();
            Vec3 s12p{};
            if (s12 != nullptr && rig_socket_world(s12, L"PrimaryWeapon", &s12p))
                halo::stomp_mark(12, s12p.x, s12p.y, s12p.z,
                                 halo::g_view_base_yaw.load(std::memory_order_relaxed));
        }
        // ---- RAYGUN (2026-09-12). From the headset: "check that the aim ray moves / rotates the exact same
        // way as the weapon positioning." Every render frame, same callback, same instant:
        //   point 20  yaw = aim ray yaw (ControlRotation), e0 = aim ray pitch,
        //             e1 = hand intent yaw (desired_aim_now, what both writers are given), e2 = intent pitch
        //   point 21  yaw = drawn barrel yaw (socket -Y), e0 = barrel pitch,
        //             e1 = angle barrel to aim ray (deg), e2 = angle barrel to intent (deg)
        // An overshoot-and-snap-back shows as the barrel (or the aim) running past the intent for
        // one frame and returning, which the per-frame series makes a number instead of a feeling.
        if (index == 0 && g_cfg.stomp_log != 0) {
            double rcp = 0.0, rcy = 0.0;
            float riy = 0.0f, rip = 0.0f;
            const bool r_aim = read_control_rotation_hook(&rcp, &rcy);
            const bool r_int = halo::desired_aim_now(&riy, &rip);
            if (r_aim) halo::stomp_mark(20, (float)rcy, (float)rcp, r_int ? riy : -999.0f, r_int ? rip : -999.0f);
            {   // Point 27: at render, the generation this thread was served and the Blam record.
                float bry = -999.0f, brp = -999.0f;
                halo::blam_ctl_read_ue_deg(&bry, &brp);
                halo::stomp_mark(27, bry, brp, (float)halo::pose_latch_last_gen(),
                                 (float)halo::g_tick_id.load(std::memory_order_relaxed));
            }
            auto* rgc = rig_tracked_component();
            Vec3 rgrot{};
            if (rgc != nullptr && rig_socket_world_rot(rgc, L"PrimaryWeapon", &rgrot)) {
                const Quat rq = rotator_to_quat(rgrot.x, rgrot.y, rgrot.z);
                const Vec3 ry = quat_rotate(rq, Vec3{0, 1, 0});
                const Vec3 bd{-ry.x, -ry.y, -ry.z};
                const float by = std::atan2(bd.y, bd.x) * RAD2DEG;
                const float bp = std::asin(clampf(bd.z, -1.0f, 1.0f)) * RAD2DEG;
                auto dir_of = [](float pdeg, float ydeg) {
                    const float pr = pdeg * DEG2RAD, yr = ydeg * DEG2RAD;
                    return Vec3{std::cos(pr) * std::cos(yr), std::cos(pr) * std::sin(yr), std::sin(pr)};
                };
                auto ang = [](const Vec3& a, const Vec3& b) {
                    return std::acos(clampf(a.x * b.x + a.y * b.y + a.z * b.z, -1.0f, 1.0f)) * RAD2DEG;
                };
                const float to_aim = r_aim ? ang(bd, dir_of((float)rcp, (float)rcy)) : -999.0f;
                const float to_int = r_int ? ang(bd, dir_of(rip, riy)) : -999.0f;
                halo::stomp_mark(21, by, bp, to_aim, to_int);
            }
        }
        // ---- FPPIN (Config.hpp fp_pin): the FP mesh is re-anchored on the VIEW frame before
        // the palette refresh reads it back. Pitch is ControlRotation's (the lock owns yaw
        // only); yaw is the one this frame renders. Fails closed on any missing ingredient.
        if (index == 0 && g_cfg.fp_pin != 0 && g_cfg.pal_render == 1 && palette_weapon_mode()
            && !g_stick_mode.load() && halo::g_mesh_const_valid.load(std::memory_order_acquire)) {
            auto* pinc = rig_tracked_component();
            auto* pinp = g_rig_parent;
            double pcp = 0.0, pcy = 0.0;
            Vec3 ppos{};
            if (pinc != nullptr && pinp != nullptr && !IsBadReadPtr(pinp, sizeof(void*))
                && read_control_rotation_hook(&pcp, &pcy)
                && call_ret_vec3(pinp, L"K2_GetComponentLocation", &ppos)) {
                const float vy = halo::g_view_base_yaw.load(std::memory_order_relaxed);
                const Quat pr = rotator_to_quat((float)pcp, vy, 0.0f);
                const Quat pM{halo::g_meshM_x.load(std::memory_order_relaxed), halo::g_meshM_y.load(std::memory_order_relaxed),
                              halo::g_meshM_z.load(std::memory_order_relaxed), halo::g_meshM_w.load(std::memory_order_relaxed)};
                const Quat mrot = quat_mul(pr, pM);
                float mp = 0.0f, my = 0.0f, mr = 0.0f;
                quat_to_rotator(mrot.x, mrot.y, mrot.z, mrot.w, &mp, &my, &mr);
                const Vec3 v0{halo::g_meshV0_x.load(std::memory_order_relaxed), halo::g_meshV0_y.load(std::memory_order_relaxed), halo::g_meshV0_z.load(std::memory_order_relaxed)};
                const Vec3 off = quat_rotate(pr, v0);
                if (std::isfinite(mp) && std::isfinite(my) && std::isfinite(off.x))
                    halo::holster_marker_place_rot(pinc, Vec3{ppos.x + off.x, ppos.y + off.y, ppos.z + off.z}, mp, my, mr);
            }
        }
        // WRIST HUD PLACEMENT, here rather than on the tick: the camera above is the one this
        // frame is drawn from, so the forearm panels land against it instead of against a camera
        // several milliseconds stale. Once per frame, not per eye.
        if (index == 0) { halo::wristhud_place(); halo::gesture_render_tick(); halo::markers_render_place(); halo::blam_palette_republish_frame(); halo::blam_palette_render_refresh(); halo::blam_palette_wpnerr_frame(); }
        // ---- FPMESH METER (Config.hpp fpmesh_log): does the FP mesh's own transform step at
        // sim rate under the 90 Hz view? Read-only; the embedded camera is recovered through
        // the measured M constant, exactly the relation the mesh-constant block validates.
        if (index == 0 && g_cfg.fpmesh_log != 0) {
            static float s_fm_prev = 0.0f, s_fm_prev_ctl = 0.0f; static bool s_fm_have = false;
            static int s_fm_fr = 0, s_fm_moved = 0; static float s_fm_sum = 0.0f, s_fm_max = 0.0f, s_fm_csum = 0.0f;
            static ULONGLONG s_fm_said = 0;
            auto* fmc = rig_tracked_component();
            Vec3 fcrot{};
            double fcp = 0.0, fcy = 0.0;
            if (fmc != nullptr && call_ret_vec3(fmc, L"K2_GetComponentRotation", &fcrot) &&
                read_control_rotation_hook(&fcp, &fcy)) {
                const Quat fM{halo::g_meshM_x.load(std::memory_order_relaxed), halo::g_meshM_y.load(std::memory_order_relaxed),
                                    halo::g_meshM_z.load(std::memory_order_relaxed), halo::g_meshM_w.load(std::memory_order_relaxed)};
                const Quat fcam = quat_mul(rotator_to_quat(fcrot.x, fcrot.y, fcrot.z), quat_conj(fM));
                float fep = 0.0f, fey = 0.0f, fer = 0.0f;
                quat_to_rotator(fcam.x, fcam.y, fcam.z, fcam.w, &fep, &fey, &fer);
                if (s_fm_have) {
                    const float dmesh = std::fabs(wrap180(fey - s_fm_prev));
                    const float dctl  = std::fabs(wrap180((float)fcy - s_fm_prev_ctl));
                    ++s_fm_fr;
                    if (dmesh > 0.01f) ++s_fm_moved;
                    s_fm_sum += dmesh; if (dmesh > s_fm_max) s_fm_max = dmesh;
                    s_fm_csum += dctl;
                }
                s_fm_prev = fey; s_fm_prev_ctl = (float)fcy; s_fm_have = true;
                const ULONGLONG fnow = GetTickCount64();
                if (s_fm_said == 0) s_fm_said = fnow;
                if (fnow - s_fm_said >= 1000 && s_fm_fr > 0) {
                    API::get()->log_info("[Halo-CampE-UEVR] FPMESH frames=%d moved=%d step mean %.3f max %.3f deg | ctl-per-frame mean %.3f | mesh-vs-ctl gap %.2f | mesh-vs-view gap %.2f",
                                         s_fm_fr, s_fm_moved, s_fm_sum / s_fm_fr, s_fm_max, s_fm_csum / s_fm_fr,
                                         wrap180((float)fcy - fey), wrap180(g_dbg_view_out.load() - fey));
                    s_fm_fr = s_fm_moved = 0; s_fm_sum = s_fm_max = s_fm_csum = 0.0f; s_fm_said = fnow;
                }
            }
        }
