// palettewpn (fork feature, Experimental): the FPMESH meter.
// Textual fragment, included by Plugin.cpp inside the stereo pre-callback, after the per-frame render pass. Moved verbatim; not compiled on its own.
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
