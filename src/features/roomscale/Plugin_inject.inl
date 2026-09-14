// roomscale (fork feature, Experimental): the left-stick injection in the XInput hook.
// Textual fragment, included by Plugin.cpp inside the XInput hook, before the vehicle hard brake. Moved verbatim; not compiled on its own.
        // ---- ROOMSCALE INJECTION (Config::roomscale). The game tick published a left-stick
        // vector in the movement frame; deliver it whenever the player's own stick is idle.
        // Never in stick mode, menus or d-pad shift -- the same gates as the movement rotation
        // above -- and never over a pushed stick: a deliberate push is locomotion, the head
        // offset waits.
        {
            const float ulx = (float)state->Gamepad.sThumbLX / 32767.0f;
            const float uly = (float)state->Gamepad.sThumbLY / 32767.0f;
            const float um = std::sqrt(ulx * ulx + uly * uly);
            g_rs_user_stick.store(um, std::memory_order_relaxed);
            halo::g_pad_user_mag.store(um, std::memory_order_relaxed);
        }
        if (g_cfg.roomscale && g_rs_active.load(std::memory_order_relaxed)
            && !g_dpad_shift_active.load() && !g_in_menu.load() && !g_stick_mode.load()) {
            const float ulx = (float)state->Gamepad.sThumbLX / 32767.0f;
            const float uly = (float)state->Gamepad.sThumbLY / 32767.0f;
            if (std::sqrt(ulx * ulx + uly * uly) < g_cfg.roomscale_stick) {
                const float rlx = g_rs_lx.load(std::memory_order_relaxed);
                const float rly = g_rs_ly.load(std::memory_order_relaxed);
                state->Gamepad.sThumbLX = to_raw(clampf(rlx, -1.0f, 1.0f));
                state->Gamepad.sThumbLY = to_raw(clampf(rly, -1.0f, 1.0f));
                state->dwPacketNumber++;
                g_rs_injected.fetch_add(1);
            }
        }
