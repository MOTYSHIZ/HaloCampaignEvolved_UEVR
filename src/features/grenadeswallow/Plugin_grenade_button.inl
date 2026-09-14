// grenadeswallow (fork feature, Experimental): the left A-face grenade code stripped from the pad, by UEVR's action state.
// Textual fragment, included by Plugin.cpp inside the XInput hook, after the ForceTube fire note. Moved verbatim; not compiled on its own.
        // ---- THE GRENADE BUTTON, BY ACTION. UEVR says whether the left hand's A-face button is
        // down; the pad code it produced is stripped so the gesture is the only path to a throw.
        // Not in menus, where that button navigates.
        if (g_cfg.grenade_swallow != 0 && !g_in_menu.load(std::memory_order_relaxed)) {
            static decltype(API::VR::get_action_handle("")) s_a = nullptr;
            static bool s_tried = false;
            if (s_a == nullptr && !s_tried) { s_tried = true; s_a = API::VR::get_action_handle("/actions/default/in/AButtonLeft");
                API::get()->log_info("[Halo-CampE-UEVR] GREN: AButtonLeft action %s", s_a ? "resolved" : "NOT found"); }
            static ULONGLONG s_gren_at = 0; static bool s_gren_down = false;
            { const ULONGLONG t = GetTickCount64(); if (s_a != nullptr && t - s_gren_at >= 4) { s_gren_at = t; s_gren_down = API::VR::is_action_active(s_a, API::VR::get_left_joystick_source()); } }
            if (s_a != nullptr && s_gren_down) {
                static bool s_said = false;
                if (!s_said && g_cfg.map_btn_log) { s_said = true; API::get()->log_info("[Halo-CampE-UEVR] GREN: left A-face down, raw pad 0x%04X, stripping 0x%04X", (unsigned)state->Gamepad.wButtons, (unsigned)g_cfg.grenade_code); }
                state->Gamepad.wButtons &= (WORD)~(WORD)g_cfg.grenade_code;
            }
        }
