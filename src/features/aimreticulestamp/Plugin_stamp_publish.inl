// aimreticulestamp (fork feature, Experimental): the render-rate publish of the stamped hand intent (modes 1 and 2).
// Textual fragment, included by Plugin.cpp inside the stereo post-callback, before the RETPROBE instrument. Moved verbatim; not compiled on its own.
        // RETSTAMP render publish (modes 1/2). Same frame as the eye note that follows: this frame's
        // view position, the stamped intent, the latest trace depth. No smoothing.
        if (index == 0 && features_reticule_render_publish_mode() != 0 && g_cfg.aim_reticule && g_cfg.xr_layer
            && !g_stick_mode.load() && g_have_view_pos.load()) {
            const long long rs_now = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            const long long rs_dms = reticule_depth_ms();
            const bool two_back = (g_cfg.aim_reticule_stamp == 1);
            float ay = 0.0f, ap = 0.0f;
            const bool rs_ok = halo::palette_pose_stamped_intent(two_back, &ay, &ap);
            if (rs_ok && rs_dms != 0 && rs_now - rs_dms < 200) {
                const float rd = reticule_depth();
                const float cpr = std::cos(ap * DEG2RAD);
                const Vec3 rf{cpr * std::cos(ay * DEG2RAD), cpr * std::sin(ay * DEG2RAD), std::sin(ap * DEG2RAD)};
                const Vec3 rtarget{g_view_pos_x.load() + rf.x * rd,
                                   g_view_pos_y.load() + rf.y * rd,
                                   g_view_pos_z.load() + rf.z * rd};
                halo::xrlayer_note_publish_gate(0);
                halo::xrlayer_notice_reticule(layer_anchor(halo::XRLAYER_SLOT_RETICULE, rtarget),
                                              g_ret_scale_mul.load());
                if (g_cfg.stomp_log != 0) halo::palette_pose_mark(46, ay, ap, rd, (float)g_cfg.aim_reticule_stamp);
            }
        }
