// vehcam (fork feature, Experimental): the engine camera capture, the chase-cam anchor probe and the first-person seat camera.
// Textual fragment, included by Plugin.cpp inside the stereo pre-callback, after the aim-convergence note. Moved verbatim; not compiled on its own.
            // The ENGINE's camera and view yaw, captured before anything below modifies them. The
            // seat anchor is built from these; see the vehcam section.
            double rawcx = 0.0, rawcy = 0.0, rawcz = 0.0, rawgyaw = 0.0, rawgpitch = 0.0;
            if (is_double) {
                auto* p = reinterpret_cast<UEVR_Vector3d*>(position);
                rawcx = p->x; rawcy = p->y; rawcz = p->z;
                if (rotation != nullptr) {
                    rawgyaw = reinterpret_cast<UEVR_Rotatord*>(rotation)->yaw;
                    rawgpitch = reinterpret_cast<UEVR_Rotatord*>(rotation)->pitch;
                }
            } else {
                rawcx = position->x; rawcy = position->y; rawcz = position->z;
                if (rotation != nullptr) { rawgyaw = rotation->yaw; rawgpitch = rotation->pitch; }
            }
            // ---- CHASE-CAM ANCHOR PROBE (vehanchor = sample every N calls, 0 = off). READ-ONLY.
            // Records what the engine hands us before the seat camera touches it, with the rider
            // and vehicle Blam positions and the vehicle facing beside it, so the boom frame can be
            // fitted from the log rather than assumed.
            if (g_cfg.veh_anchor > 0 && halo::g_unit_mounted.load(std::memory_order_relaxed)) {
                static uint32_t an = 0;
                if ((an++ % (uint32_t)g_cfg.veh_anchor) == 0u) {
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] VEHANCHOR: cam=(%.2f %.2f %.2f) gyaw=%.2f gpitch=%.2f "
                        "vpos=(%.4f %.4f %.4f) bpos=(%.4f %.4f %.4f) face=(%.4f %.4f) fvalid=%d",
                        rawcx, rawcy, rawcz, rawgyaw, rawgpitch,
                        halo::g_vehpx.load(std::memory_order_relaxed),
                        halo::g_vehpy.load(std::memory_order_relaxed),
                        halo::g_vehpz.load(std::memory_order_relaxed),
                        halo::g_unit_px.load(std::memory_order_relaxed),
                        halo::g_unit_py.load(std::memory_order_relaxed),
                        halo::g_unit_pz.load(std::memory_order_relaxed),
                        halo::g_veh_fx.load(std::memory_order_relaxed),
                        halo::g_veh_fy.load(std::memory_order_relaxed),
                        (int)halo::g_veh_fvalid.load(std::memory_order_relaxed));
                }
            }
            // ---- FIRST PERSON IN A VEHICLE (vehcam).
            //
            // Halo's vehicle camera is a third-person chase cam, unusable in VR. There is no
            // first-person mode to switch on, so the rendered position is moved to the seat.
            //
            // Do not SYNTHESISE a camera from the Blam position (a 300-500 Hz physics value) while
            // the mesh is drawn from a frame-rate snapshot of that same physics: two curves at two
            // rates, and the breathing gap between them is judder that parallax puts on the hog.
            //
            // vehcamanchor=2 (default): the camera rides the hog's DRAWN hull component. Fallback
            // vehcamanchor=1: seat = cam - R(gyaw) . boom, the boom rigid in the AIM frame (measured
            // +-10 cm lateral scatter there against +-513 world and +-133 vehicle facing), only the
            // boom LENGTH filtered, never the position. vehcamanchor=0: the old synthesised camera.
            //
            // Runs only while mounted (or always with vehcam=2), so the on-foot view is untouched.
            if (index == 0) halo::seat_direct_refresh();   // vehseatdirect: render-rate rider read
            const bool vc_gate = g_cfg.veh_cam != 0 && halo::g_unit_pvalid.load(std::memory_order_relaxed)
                && (g_cfg.veh_cam == 2 || halo::g_unit_mounted.load(std::memory_order_relaxed));
            // VEHSEAT (vehlog): once a second while seated or in stick mode, everything the "camera
            // stays behind the hog" question needs in one line -- the rider as published, the hull
            // as drawn, whether the publish is alive (calls/s, stale ms), what gated it (mounted,
            // pvalid, record), what the learn/snap did, and how many frames each path rendered.
            if (index == 0) {
                const bool stick_now = halo::g_stick_mode_active.load(std::memory_order_relaxed);
                if (!vc_gate && g_cfg.veh_cam != 0 && stick_now) ++g_vcd.frames[VCP_NONE];
                static auto s_vs_t = std::chrono::steady_clock::now();
                static uint32_t s_c0 = 0, s_n0 = 0, s_r0 = 0, s_d0 = 0;
                const auto vnow = std::chrono::steady_clock::now();
                const float vel = std::chrono::duration<float>(vnow - s_vs_t).count();
                if (vel >= 1.0f) {
                    const uint32_t c = halo::g_seat_pub_calls.load(std::memory_order_relaxed);
                    const uint32_t n = halo::g_seat_norec.load(std::memory_order_relaxed);
                    const uint32_t r = halo::g_seat_reresolve.load(std::memory_order_relaxed);
                    const uint32_t d = halo::g_seat_direct_reads.load(std::memory_order_relaxed);
                    const bool mounted_now = halo::g_unit_mounted.load(std::memory_order_relaxed);
                    const uint32_t fsum = g_vcd.frames[0] + g_vcd.frames[1] + g_vcd.frames[2]
                                        + g_vcd.frames[3] + g_vcd.frames[4];
                    if (g_cfg.veh_log && (mounted_now || stick_now || fsum != 0)) {
                        const double bx = halo::g_unit_px.load(std::memory_order_relaxed);
                        const double by = halo::g_unit_py.load(std::memory_order_relaxed);
                        const double bz = halo::g_unit_pz.load(std::memory_order_relaxed);
                        const double S = g_cfg.veh_cam_scale;
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] VEHSEAT: mounted=%d pvalid=%d stick=%d rec=%d | rider=(%.3f %.3f %.3f)wu ue=(%.0f %.0f %.0f) "
                            "hull=%s(%.0f %.0f %.0f) hullspeed=%.0fcm/s speed=%.2fwu/s | pub/s=%.0f norec/s=%.0f reresolve/s=%.1f direct/s=%.0f stale=%.0fms | "
                            "learn=%d learnframes=%u snaps=%u refused=%u off=(%.0f %.0f %.0f) | frames rigid=%u hold=%u chase=%u synth=%u gateoff=%u | "
                            "seatpub=%d direct=%d guard=%d hullcheck=%d anchor=%d src=%d | veh=(%.0f %.0f %.0f)ue fvalid=%d hulldead=%d | "
                            "post: frames=%u bad=%u maxdev=%.0fcm seat=(%.0f %.0f %.0f)",
                            (int)mounted_now, (int)halo::g_unit_pvalid.load(std::memory_order_relaxed), (int)stick_now,
                            (int)(halo::blam_control_record() != 0),
                            bx, by, bz, bx * S, -by * S, bz * S,
                            g_vcd.have_hull ? "" : "UNSEEN", g_vcd.hull[0], g_vcd.hull[1], g_vcd.hull[2],
                            g_vcd.hspeed, (double)halo::g_veh_speed.load(std::memory_order_relaxed),
                            (double)(c - s_c0) / vel, (double)(n - s_n0) / vel, (double)(r - s_r0) / vel,
                            (double)(d - s_d0) / vel, (double)g_vcd.stale_ms,
                            (int)g_vcd.learning, g_vcd.learn_frames, g_vcd.snaps, g_vcd.snaps_refused,
                            g_vcd.off[0], g_vcd.off[1], g_vcd.off[2],
                            g_vcd.frames[VCP_RIGID], g_vcd.frames[VCP_HOLD], g_vcd.frames[VCP_CHASE],
                            g_vcd.frames[VCP_SYNTH], g_vcd.frames[VCP_NONE],
                            g_cfg.veh_seat_pub, g_cfg.veh_seat_direct, g_cfg.veh_cam_guard,
                            g_cfg.veh_cam_hull_check, g_cfg.veh_cam_anchor, g_cfg.veh_cam_src,
                            (double)halo::g_vehpx.load(std::memory_order_relaxed) * S,
                            -(double)halo::g_vehpy.load(std::memory_order_relaxed) * S,
                            (double)halo::g_vehpz.load(std::memory_order_relaxed) * S,
                            (int)halo::g_veh_fvalid.load(std::memory_order_relaxed), (int)g_vcd.hull_dead,
                            g_vcd.post_frames, g_vcd.post_bad, g_vcd.post_max,
                            g_vcd.fc[0], g_vcd.fc[1], g_vcd.fc[2]);
                    }
                    s_vs_t = vnow; s_c0 = c; s_n0 = n; s_r0 = r; s_d0 = d;
                    g_vcd.post_frames = 0; g_vcd.post_bad = 0; g_vcd.post_max = 0.0;
                    for (auto& f : g_vcd.frames) f = 0;
                    g_vcd.learn_frames = 0;
                    g_vcd.have_hull = false;
                }
            }
            if (vc_gate) {
                const float S = g_cfg.veh_cam_scale;
                const auto cnow = std::chrono::steady_clock::now();
                // Which body the boom is measured against. The rider is the default: it lands on
                // 95-100% of frames against 91-97% for the vehicle, and it is already AT the seat.
                const bool use_veh = (g_cfg.veh_cam_src != 0)
                                  && halo::g_veh_fvalid.load(std::memory_order_relaxed);
                const double bpx = use_veh ? (double)halo::g_vehpx.load(std::memory_order_relaxed)
                                           : (double)halo::g_unit_px.load(std::memory_order_relaxed);
                const double bpy = use_veh ? (double)halo::g_vehpy.load(std::memory_order_relaxed)
                                           : (double)halo::g_unit_py.load(std::memory_order_relaxed);
                const double bpz = use_veh ? (double)halo::g_vehpz.load(std::memory_order_relaxed)
                                           : (double)halo::g_unit_pz.load(std::memory_order_relaxed);
                // LATCHED ONCE PER FRAME. This callback runs per EYE. Reading the source fresh on
                // each eye let a publish land BETWEEN the two calls, rendering left and right from
                // positions up to a frame of travel apart (~47 cm of bogus disparity, on the hog and
                // nowhere else). Compute on eye 0, reuse on the other.
                static double fcx = 0.0, fcy = 0.0, fcz = 0.0;
                static bool   fvalid = false;
                static std::chrono::steady_clock::time_point tframe{};
                static bool   tframe_ok = false;
                const float since = tframe_ok
                    ? std::chrono::duration<float>(cnow - tframe).count() : 1.0f;
                if (index == 0 || !fvalid || since > 0.050f) {
                    float fdt = tframe_ok ? since : 0.0f;
                    tframe = cnow; tframe_ok = true;
                    if (!(fdt > 0.0f) || fdt > 0.10f) fdt = 0.011f;   // first frame, or a hitch
                    // Fallback path (vehcamanchor=0): the synthesised camera, kept for A/B only.
                    double lcx = bpx * S, lcy = -bpy * S, lcz = bpz * S;
                    // ---- RIGID VEHICLE CAMERA (vehcamanchor=2).
                    //
                    // Bolt the camera to the hog's own DRAWN component: one source, zero filters on
                    // position, so neither the chase-cam spring nor the two-curve judder exists.
                    //
                    // The seat offset is LEARNED, not configured: rider minus hull in the hull's
                    // FULL rotation frame (a yaw-only frame read slope pitch as the offset moving),
                    // learned ONLY WHILE PARKED (speed < 0.5 wu/s) and frozen while moving. At speed
                    // the sim rider leads the drawn hull by a persistent bias an EMA integrates, so
                    // parked is the one state the offset is learnable in, and a frozen offset is the
                    // definition of rigid.
                    bool rigid_done = false;
                    bool vc_hold = false;   // rigid, but vehcamguard is holding the offset
                    if (g_cfg.veh_cam_anchor == 2) {
                        static TrackedObject s_hog;
                        static uintptr_t s_hog_raw = 0;
                        static uint32_t s_hog_gen = 0;   // bumps on every hull change: re-prime the seat
                        const uintptr_t hp = halo::g_hog_body_ptr.load(std::memory_order_relaxed);
                        if (hp != s_hog_raw) {
                            s_hog_raw = hp;
                            ++s_hog_gen;
                            s_hog.set_at(reinterpret_cast<API::UObject*>(hp),
                                         halo::g_hog_body_idx.load(std::memory_order_relaxed));
                        }
                        auto* hog = (hp != 0) ? s_hog.get_checked(L"SkeletalMeshComponent") : nullptr;
                        Vec3 hloc{}, hrot{};
                        if (hog != nullptr && call_ret_vec3(hog, L"K2_GetComponentLocation", &hloc)
                            && call_ret_vec3(hog, L"K2_GetComponentRotation", &hrot)) {
                            // UE rotator convention (FRotationMatrix): rows are the world-space
                            // X/Y/Z axes; the rotator reads (pitch, yaw, roll).
                            const double D2R = 0.01745329252;
                            const double cp2 = std::cos((double)hrot.x * D2R), sp2 = std::sin((double)hrot.x * D2R);
                            const double cy2 = std::cos((double)hrot.y * D2R), sy2 = std::sin((double)hrot.y * D2R);
                            const double cr2 = std::cos((double)hrot.z * D2R), sr2 = std::sin((double)hrot.z * D2R);
                            const double ax[3] = { cp2 * cy2, cp2 * sy2, sp2 };
                            const double ay[3] = { sr2 * sp2 * cy2 - cr2 * sy2, sr2 * sp2 * sy2 + cr2 * cy2, -sr2 * cp2 };
                            const double az[3] = { -(cr2 * sp2 * cy2 + sr2 * sy2), cy2 * sr2 - cr2 * sp2 * sy2, cr2 * cp2 };
                            const double rx = bpx * S - (double)hloc.x;
                            const double ry = -bpy * S - (double)hloc.y;
                            const double rz = bpz * S - (double)hloc.z;
                            const double lof = ax[0] * rx + ax[1] * ry + ax[2] * rz;
                            const double lol = ay[0] * rx + ay[1] * ry + ay[2] * rz;
                            const double lou = az[0] * rx + az[1] * ry + az[2] * rz;
                            static double sof = 0.0, sol = 0.0, sou = 0.0;
                            static bool   sprimed = false;
                            static std::chrono::steady_clock::time_point tprime{};
                            static uint32_t s_gen_seen = 0;
                            if (s_gen_seen != s_hog_gen) { s_gen_seen = s_hog_gen; sprimed = false; }
                            const double oj = (lof - sof) * (lof - sof) + (lol - sol) * (lol - sol)
                                            + (lou - sou) * (lou - sou);
                            // The speed the learn gate sees, and what it decided, for the log below.
                            const float lspeed = halo::g_veh_speed.load(std::memory_order_relaxed);
                            bool learning = false;
                            static uint32_t s_learn_frames = 0, s_snaps = 0, s_refused = 0;

                            // THE DRAWN HULL'S OWN SPEED. g_veh_speed is differentiated from the
                            // RIDER position, so a frozen rider reads 0 and calls a moving hog
                            // parked. The hull transform is live by construction: it is what is drawn.
                            static double s_hpx = 0.0, s_hpy = 0.0, s_hpz = 0.0, s_hspeed = 0.0;
                            static bool   s_hp_ok = false;
                            if (s_hp_ok && fdt > 0.0f) {
                                const double dx = (double)hloc.x - s_hpx, dy = (double)hloc.y - s_hpy,
                                             dz = (double)hloc.z - s_hpz;
                                const double inst = std::sqrt(dx * dx + dy * dy + dz * dz) / (double)fdt;
                                const double kh = (fdt / 0.1 > 1.0) ? 1.0 : (double)fdt / 0.1;
                                s_hspeed += (inst - s_hspeed) * kh;
                            }
                            s_hpx = hloc.x; s_hpy = hloc.y; s_hpz = hloc.z; s_hp_ok = true;
                            // RIDER FRESHNESS: the publish sequence advances ~2600/s while the sim
                            // publish is live; silence means the rider values are a frozen snapshot.
                            static uint32_t s_seq_prev = 0;
                            static std::chrono::steady_clock::time_point s_seq_t = cnow;
                            const uint32_t seq = halo::g_seat_pub_seq.load(std::memory_order_relaxed);
                            if (seq != s_seq_prev) { s_seq_prev = seq; s_seq_t = cnow; }
                            const float stale_ms = std::chrono::duration<float, std::milli>(cnow - s_seq_t).count();
                            // vehcamguard (approach C): never learn or snap on a moving hull or a
                            // stale rider -- hold the offset and ride the hull.
                            const bool rider_stale = stale_ms > (float)g_cfg.veh_cam_stale_ms;
                            const bool hull_moving = s_hspeed > (double)g_cfg.veh_cam_guard_speed;
                            const bool guard_hold = (g_cfg.veh_cam_guard != 0) && (rider_stale || hull_moving);
                            // Never PRIME from a stale rider: that would bolt the camera to the frozen
                            // point. Fall through to the chase anchor until the rider is live.
                            // vehcamhullcheck (approach D): the hull component must move WITH the
                            // Blam vehicle. Driving speed on the Blam side with a motionless
                            // component means this transform is not what is drawn, and a camera
                            // bolted to it stays where it is while the hog drives off. Latched per
                            // hull; a new hull (remount) re-arms it.
                            static uint32_t s_dead_gen = 0xFFFFFFFFu;
                            static bool     s_hull_dead = false;
                            static float    s_still_s = 0.0f;
                            if (s_dead_gen != s_hog_gen) { s_dead_gen = s_hog_gen; s_hull_dead = false; s_still_s = 0.0f; }
                            if (g_cfg.veh_cam_hull_check != 0 && !s_hull_dead) {
                                const bool blam_driving = lspeed > 1.0f && !rider_stale;
                                if (blam_driving && s_hspeed < 30.0) s_still_s += fdt; else s_still_s = 0.0f;
                                if (s_still_s > g_cfg.veh_cam_hull_dead_s) {
                                    s_hull_dead = true;
                                    API::get()->log_info("[Halo-CampE-UEVR] VEHCAMHULL: hull component still (%.0f cm/s) while the Blam vehicle drives (%.2f wu/s) for %.2f s -- "
                                                         "rigid camera disabled for this hull, riding the chase-cam anchor. hull=(%.0f %.0f %.0f)",
                                                         s_hspeed, (double)lspeed, (double)s_still_s,
                                                         (double)hloc.x, (double)hloc.y, (double)hloc.z);
                                }
                            }
                            g_vcd.hull_dead = s_hull_dead;
                            const bool unprimable = (!sprimed && (g_cfg.veh_cam_guard != 0) && rider_stale)
                                                 || (s_hull_dead && g_cfg.veh_cam_hull_check != 0);
                            if (!unprimable) {
                                if (!sprimed || oj > 500.0 * 500.0) {
                                    if (sprimed && guard_hold) {
                                        ++s_refused;
                                    } else {
                                        sof = lof; sol = lol; sou = lou; sprimed = true;
                                        tprime = cnow;
                                        ++s_snaps;
                                    }
                                } else if (lspeed < 0.5f && !guard_hold) {
                                    // LEARN ONLY WHILE PARKED; frozen outright while moving.
                                    learning = true;
                                    ++s_learn_frames;
                                    const float age2 = std::chrono::duration<float>(cnow - tprime).count();
                                    const double otau = (age2 < 2.0f) ? 0.3 : 1.5;
                                    const double ko = (fdt / otau > 1.0) ? 1.0 : (double)fdt / otau;
                                    sof += (lof - sof) * ko; sol += (lol - sol) * ko; sou += (lou - sou) * ko;
                                }
                                lcx = (double)hloc.x + ax[0] * sof + ay[0] * sol + az[0] * sou;
                                lcy = (double)hloc.y + ax[1] * sof + ay[1] * sol + az[1] * sou;
                                lcz = (double)hloc.z + ax[2] * sof + ay[2] * sol + az[2] * sou;
                                rigid_done = true;
                                vc_hold = guard_hold;
                            }
                            g_vcd.have_hull = true;
                            g_vcd.hull[0] = hloc.x; g_vcd.hull[1] = hloc.y; g_vcd.hull[2] = hloc.z;
                            g_vcd.off[0] = sof; g_vcd.off[1] = sol; g_vcd.off[2] = sou;
                            g_vcd.hspeed = s_hspeed; g_vcd.stale_ms = stale_ms;
                            g_vcd.learning = learning;
                            if (learning) ++g_vcd.learn_frames;
                            g_vcd.snaps = s_snaps; g_vcd.snaps_refused = s_refused;
                            if (g_cfg.veh_log && rigid_done) {
                                // HOGLEARN on every gate edge, so a learn that runs while moving is
                                // named with the speed it saw rather than inferred from off= drift.
                                static int s_learn_prev = -1;
                                if ((int)learning != s_learn_prev) {
                                    s_learn_prev = (int)learning;
                                    API::get()->log_info("[Halo-CampE-UEVR] HOGLEARN: %s at speed=%.2f wu/s (gate < 0.50) off=(%.0f %.0f %.0f)",
                                                         learning ? "LEARNING (parked)" : "FROZEN",
                                                         (double)lspeed, sof, sol, sou);
                                }
                                static uint32_t hn = 0;
                                if ((hn++ % 90u) == 0u) {
                                    API::get()->log_info("[Halo-CampE-UEVR] HOGCAM: hog=(%.1f %.1f %.1f) "
                                                         "rot=(%.1f %.1f %.1f) off=(%.0f %.0f %.0f) drift=(%.1f %.1f %.1f) "
                                                         "speed=%.2f learn=%d learnframes=%u snaps=%u",
                                                         (double)hloc.x, (double)hloc.y, (double)hloc.z,
                                                         (double)hrot.x, (double)hrot.y, (double)hrot.z,
                                                         sof, sol, sou, lof - sof, lol - sol, lou - sou,
                                                         (double)lspeed, (int)learning, s_learn_frames, s_snaps);
                                    s_learn_frames = 0;
                                }
                            }
                        }
                        // Unresolved or invalid: fall through to the chase-cam anchor below, so a
                        // vehicle with no hull degrades to the working camera instead of nothing.
                    }
                    if (!rigid_done && g_cfg.veh_cam_anchor != 0) {
                        const double wx = rawcx - bpx * S;            // rider -> cam, world cm
                        const double wy = rawcy + bpy * S;
                        const double wz = rawcz - bpz * S;
                        const double ar = rawgyaw * 0.01745329252;
                        const double ca = std::cos(ar), sa = std::sin(ar);
                        const double bf =  wx * ca + wy * sa;         // into the aim frame
                        const double bl = -wx * sa + wy * ca;
                        static double ef = 0.0, el = 0.0, eu = 0.0;
                        static bool   eb = false;
                        const float tau = (g_cfg.veh_cam_boom_tau > 0.01f) ? g_cfg.veh_cam_boom_tau : 0.5f;
                        // A big jump is a mount, a seat swap or a level load, never a spring: snap.
                        const double bj = (bf - ef) * (bf - ef) + (bl - el) * (bl - el)
                                        + (wz - eu) * (wz - eu);
                        // TWO POLES, NOT ONE (vehboomorder=2). Two cascaded one-poles at tau/2 have
                        // the same total group delay as one at tau but roll off at 40 dB/decade
                        // instead of 20, so the frame-scale noise is rejected harder for the same
                        // lag. Cascaded one-poles on purpose: no complex poles, no overshoot.
                        static double m1f = 0.0, m1l = 0.0, m1u = 0.0;   // first stage
                        if (!eb || bj > 600.0 * 600.0) {
                            ef = bf; el = bl; eu = wz;
                            m1f = bf; m1l = bl; m1u = wz;
                            eb = true;
                        } else if (g_cfg.veh_boom_order >= 2) {
                            const double t2 = (double)tau * 0.5;
                            const double k2 = (fdt / t2 > 1.0) ? 1.0 : (double)(fdt / t2);
                            m1f += (bf - m1f) * k2; m1l += (bl - m1l) * k2; m1u += (wz - m1u) * k2;
                            ef += (m1f - ef) * k2; el += (m1l - el) * k2; eu += (m1u - eu) * k2;
                        } else {
                            const double kb = (fdt / tau > 1.0f) ? 1.0 : (double)(fdt / tau);
                            ef += (bf - ef) * kb; el += (bl - el) * kb; eu += (wz - eu) * kb;
                            m1f = ef; m1l = el; m1u = eu;   // stay primed for a live switch
                        }
                        lcx = rawcx - (ef * ca - el * sa);            // rotate back, then subtract
                        lcy = rawcy - (ef * sa + el * ca);
                        lcz = rawcz - eu;
                    }
                    // Seat offset, in the VEHICLE's frame (x forward, y left, z up) so it stays put
                    // as the hog turns. Applied on every path so vehcamoff means one thing.
                    double ox = g_cfg.veh_cam_off[0], oy = g_cfg.veh_cam_off[1];
                    {
                        const double fxv =  (double)halo::g_veh_fx.load(std::memory_order_relaxed);
                        const double fyv = -(double)halo::g_veh_fy.load(std::memory_order_relaxed);
                        const double n = std::sqrt(fxv * fxv + fyv * fyv);
                        if (halo::g_veh_fvalid.load(std::memory_order_relaxed) && n > 0.5) {
                            const double cf = fxv / n, sf = fyv / n;
                            ox = g_cfg.veh_cam_off[0] * cf - g_cfg.veh_cam_off[1] * sf;
                            oy = g_cfg.veh_cam_off[0] * sf + g_cfg.veh_cam_off[1] * cf;
                        }
                    }
                    fcx = lcx + ox; fcy = lcy + oy; fcz = lcz + g_cfg.veh_cam_off[2];
                    fvalid = true;
                    // WHICH PATH RENDERED THIS FRAME, counted for the VEHSEAT line and named on
                    // every change so a fallback cannot hide inside a once-a-second summary.
                    {
                        const int path = rigid_done ? (vc_hold ? VCP_HOLD : VCP_RIGID)
                                       : (g_cfg.veh_cam_anchor != 0 ? VCP_CHASE : VCP_SYNTH);
                        ++g_vcd.frames[path];
                        if (g_cfg.veh_log && path != g_vcd.last_path) {
                            static const char* const kPath[] = {"none", "rigid", "rigid-hold", "chase", "synth"};
                            API::get()->log_info("[Halo-CampE-UEVR] VEHCAMPATH: %s -> %s (stale=%.0fms hullspeed=%.0fcm/s speed=%.2fwu/s)",
                                                 g_vcd.last_path >= 0 ? kPath[g_vcd.last_path] : "start", kPath[path],
                                                 (double)g_vcd.stale_ms, g_vcd.hspeed,
                                                 (double)halo::g_veh_speed.load(std::memory_order_relaxed));
                        }
                        g_vcd.last_path = path;
                    }
                }
                if (is_double) {
                    auto* p = reinterpret_cast<UEVR_Vector3d*>(position);
                    p->x = fcx; p->y = fcy; p->z = fcz;
                } else {
                    position->x = (float)fcx; position->y = (float)fcy; position->z = (float)fcz;
                }
                // The seated reticule and the compositor read the view position; the wheel zone and
                // markers read g_cam. Both must be the seat, not the chase cam. The lock path below
                // is skipped while seated, so the mirror is done here.
                g_view_pos_x = (float)fcx; g_view_pos_y = (float)fcy; g_view_pos_z = (float)fcz;
                halo::g_cam_x.store((float)fcx, std::memory_order_relaxed);
                halo::g_cam_y.store((float)fcy, std::memory_order_relaxed);
                halo::g_cam_z.store((float)fcz, std::memory_order_relaxed);
                if (index == 0) { g_vcd.fc[0] = fcx; g_vcd.fc[1] = fcy; g_vcd.fc[2] = fcz; g_vcd.wrote = true; }
            } else if (index == 0) {
                g_vcd.wrote = false;
            }
