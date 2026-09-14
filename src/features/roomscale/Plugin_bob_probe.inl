// roomscale (fork feature, Experimental): the camera-bob measurement and the throttle-frame probe log.
// Textual fragment, included by Plugin.cpp inside update(), before the HMD translation leash. Moved verbatim; not compiled on its own.
    // ---- CAMERA BOB: measure the camera component against the pawn root in the aim-yaw frame,
    // low-pass the slow part (eye height, crouch), publish the fast remainder as the bob.
    {
        const bool want_bob = g_cfg.bob_cancel || g_cfg.bob_log;
        Vec3 bob{0.0f, 0.0f, 0.0f};
        if (want_bob && g_rig_parent != nullptr && !g_stick_mode.load()) {
            auto* pawn_b = reinterpret_cast<API::UObject*>(API::get()->get_local_pawn(0));
            Vec3 root{}, camb{}; double cpb = 0.0, cyb = 0.0;
            if (pawn_b != nullptr && call_ret_vec3(pawn_b, L"K2_GetActorLocation", &root)
                && call_ret_vec3(g_rig_parent, L"K2_GetComponentLocation", &camb)
                && read_control_rotation(&cpb, &cyb, nullptr)) {
                static Vec3 s_lp{}; static bool s_have = false;
                static std::chrono::steady_clock::time_point s_t{};
                const auto nowb = std::chrono::steady_clock::now();
                const float dtb = s_have ? std::chrono::duration<float>(nowb - s_t).count() : 0.0f;
                s_t = nowb;
                const float ayb = (float)cyb * DEG2RAD, cb = std::cos(ayb), snb = std::sin(ayb);
                const Vec3 dw{camb.x - root.x, camb.y - root.y, camb.z - root.z};
                const Vec3 dl{ dw.x * cb + dw.y * snb, -dw.x * snb + dw.y * cb, dw.z};   // aim-yaw frame
                const float taub = (g_cfg.bob_tau > 0.02f) ? g_cfg.bob_tau : 0.4f;
                if (!s_have || dtb <= 0.0f || dtb > 0.5f) { s_lp = dl; s_have = true; }
                else { const float ab = clampf(dtb / taub, 0.0f, 1.0f); s_lp.x += (dl.x - s_lp.x) * ab; s_lp.y += (dl.y - s_lp.y) * ab; s_lp.z += (dl.z - s_lp.z) * ab; }
                const Vec3 bl{dl.x - s_lp.x, dl.y - s_lp.y, dl.z - s_lp.z};
                bob = Vec3{bl.x * cb - bl.y * snb, bl.x * snb + bl.y * cb, bl.z};        // back to world
                if (g_cfg.bob_log) {
                    static uint32_t nlog = 0;
                    if ((nlog++ % 2u) == 0u) {
                        API::get()->log_info("[Halo-CampE-UEVR] BOB dt=%.1fms d_local=(%.2f %.2f %.2f) lp=(%.2f %.2f %.2f) bob=(%.2f %.2f %.2f)cm aim=%.1f",
                                             dtb * 1000.0f, dl.x, dl.y, dl.z, s_lp.x, s_lp.y, s_lp.z, bl.x, bl.y, bl.z, (float)cyb);
                    }
                }
            }
        }
        if (!g_cfg.bob_cancel) bob = Vec3{0.0f, 0.0f, 0.0f};
        halo::g_bob_x.store(bob.x, std::memory_order_relaxed);
        halo::g_bob_y.store(bob.y, std::memory_order_relaxed);
        halo::g_bob_z.store(bob.z, std::memory_order_relaxed);
    }

    // ---- THROTTLE-FRAME PROBE LOG (Config::roomscale_thr_probe). BlamDrive is writing a constant
    // forward throttle into the biped; here we log the world direction the eye actually moves
    // beside the aim yaw and the view yaw, so the frame is fit from a KNOWN command, not a lean.
    if (g_cfg.roomscale_thr_probe != 0 && g_rig_parent != nullptr && !g_stick_mode.load()) {
        Vec3 eyep{}; double cpp2 = 0.0, cyp = 0.0;
        if (call_ret_vec3(g_rig_parent, L"K2_GetComponentLocation", &eyep)
            && read_control_rotation(&cpp2, &cyp, nullptr)) {
            static Vec3 s_pe{}; static bool s_h = false;
            if (s_h) {
                const float dxp = eyep.x - s_pe.x, dyp = eyep.y - s_pe.y;
                const float dmp = std::sqrt(dxp * dxp + dyp * dyp);
                if (dmp > 0.2f) {   // cm; only while actually moving
                    const float wdir = std::atan2(dyp, dxp) * RAD2DEG;   // world direction of travel
                    const float f1 = std::atan2(halo::g_dbg_face[1].load(), halo::g_dbg_face[0].load()) * RAD2DEG;
                    const float f2 = std::atan2(halo::g_dbg_face[3].load(), halo::g_dbg_face[2].load()) * RAD2DEG;
                    const float fb = std::atan2(halo::g_dbg_face[5].load(), halo::g_dbg_face[4].load()) * RAD2DEG;
                    API::get()->log_info("[Halo-CampE-UEVR] ROOMSCALE-PROBE move_world=%.1fdeg |%.2fcm| aim=%.1f view=%.1f  (world-aim=%.1f world-view=%.1f) f1D4=%.1f f1E0=%.1f body=%.1f  [target 0: err=%.1f]",
                                         wdir, dmp, (float)cyp, halo::g_view_base_yaw.load(),
                                         wrap180(wdir - (float)cyp), wrap180(wdir - halo::g_view_base_yaw.load()),
                                         f1, f2, fb, wrap180(wdir));
                }
            }
            s_pe = eyep; s_h = true;
        }
    }
