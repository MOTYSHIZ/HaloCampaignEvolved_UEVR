// wristhud (fork feature, Experimental): the left-trigger glance gate in the XInput hook.
// Textual fragment, included by Plugin.cpp inside the XInput hook, after the calibration trigger eat. Moved verbatim; not compiled on its own.
        // ---- WRIST HUD GLANCE GATE. After the calibration eat above so calibration keeps
        // precedence (its zeroed trigger reads as "not held" here). While the trigger serves the
        // HUD it is swallowed from the game -- a glance must not also fire LT's native action.
        // NOT in stick mode: the panels are hidden in vehicles and cutscenes anyway, so eating the
        // trigger there is pure loss -- and LT is the vehicle handbrake / quick turn, so the
        // glance gate was silently disabling a driving control for no benefit.
        if (g_cfg.enabled && g_cfg.wrist_hud && g_cfg.wrist_hud_trigger &&
            !halo::g_stick_mode_active.load(std::memory_order_relaxed)) {
            g_wristhud_lt.store(state->Gamepad.bLeftTrigger >= 64, std::memory_order_relaxed);
            state->Gamepad.bLeftTrigger = 0;
        } else {
            g_wristhud_lt.store(false, std::memory_order_relaxed);
        }
