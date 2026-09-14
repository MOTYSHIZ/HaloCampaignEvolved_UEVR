// palettewpn (fork feature, Experimental): the RETPROBE eye and view samples (STOMPLOG points 44 and 45).
// Textual fragment, included by Plugin.cpp inside the stereo post-callback, before the eye note. Moved verbatim; not compiled on its own.
        // RETPROBE 44: eye position this frame and the view position x published beside it.
        //          45: view position y/z and the finished view yaw, same frame.
        if (index == 0 && g_cfg.stomp_log != 0 && g_have_eye_pos.load()) {
            halo::stomp_mark(44, g_eye_pos_x.load(), g_eye_pos_y.load(), g_eye_pos_z.load(), g_view_pos_x.load());
            halo::stomp_mark(45, g_view_pos_y.load(), g_view_pos_z.load(), g_render_view_yaw.load(), 0.0f);
        }
