// headblock (fork feature, Experimental): the per-tick head block trace, with its stand-downs.
// Textual fragment, included by Plugin.cpp inside update(), after the HMD translation leash. Moved verbatim; not compiled on its own.
    // ---- HEAD BLOCK (HeadBlock.hpp): trace the body-eye -> head offset the view callbacks
    // publish and hand back how far the head may extend. Stands down in menus, vehicles and the
    // 2D cutscene screen, where the engine camera is not the body's eye.
    {
        const bool hb_active = (g_cfg.head_block != 0) && !g_in_menu.load() && !g_cut2d_engaged.load()
                            && !halo::g_unit_mounted.load(std::memory_order_relaxed) && g_cfg.veh_cam != 2;
        API::UObject* hb_ignore[2] = {};
        int hb_n = 0;
        if (hb_active && (g_cfg.head_block == 1 || g_cfg.head_block == 2)) {
            if (auto* pawn = API::get()->get_local_pawn(0)) hb_ignore[hb_n++] = pawn;
            if (auto* rigc = reinterpret_cast<API::UObject*>(g_rig_component.load())) {
                if (auto* wep = rigc->get_outer()) hb_ignore[hb_n++] = wep;
            }
        }
        halo::headblock_tick(hb_active, hb_ignore, hb_n, g_last_dt.load());
    }
