// palettewpn (fork feature, Experimental): the on-foot reticule run from the resolved rig when the rig driver is off.
// Textual fragment, included by Plugin.cpp inside update(), after the rig driver block. Moved verbatim; not compiled on its own.
    // ON-FOOT RETICULE WITHOUT THE RIG DRIVER.
    //
    // The block above is the rig DRIVER and only runs with rig=1, but the on-foot reticule (the aim
    // point, the hosted crosshair widget, the compositor publish) lives inside it. With the palette
    // weapon placing the gun and rig=0, none of it ran: no aim point, no widget, and the compositor
    // layer reported NO TARGET PUBLISHED. The rig component and its parent are still resolved for
    // the palette (see the resolve gate), so the reticule runs here from those, with no rig writes.
    // rig=1 never reaches this; its call inside the driver is unchanged. g_stick_mode is written
    // once per tick above both, so this and the seated publish below cannot both run.
    if (!g_cfg.rig_enabled && palette_weapon_mode() && !g_stick_mode.load()
        && (g_cfg.aim_reticule || g_aim_marker.active)) {
        auto* rig = reinterpret_cast<API::UObject*>(g_rig_component.load());
        Vec3 comp_world{};
        if (rig != nullptr && call_ret_vec3(rig, L"K2_GetComponentLocation", &comp_world)) {
            g_tick_stage = "onfoot_reticule";
            onfoot_reticule_tick(rig, comp_world, aim_yaw, aim_pitch, tick);
            g_tick_stage = "after onfoot_reticule";
        }
    }
