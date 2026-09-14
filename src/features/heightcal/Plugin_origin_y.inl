// heightcal (fork feature, Experimental): the auto height tick that owns the standing origin's Y.
// Textual fragment, included by Plugin.cpp inside update(), in the HMD translation leash block, before the vertical leash. Moved verbatim; not compiled on its own.
            // AUTO HEIGHT owns the origin's Y (HeightCal.hpp). While it is on, the vertical leash
            // never acts: it would drag Y back onto the head and break the floor-to-floor mapping.
            float hc_y = 0.0f;
            bool hc_own = false;
            if (g_cfg.height_cal != 0) {
                API::UObject* hc_ignore[2] = {};
                int hc_n = 0;
                if (auto* pawn = API::get()->get_local_pawn(0)) hc_ignore[hc_n++] = pawn;
                if (auto* rigc = reinterpret_cast<API::UObject*>(g_rig_component.load())) {
                    if (auto* wep = rigc->get_outer()) hc_ignore[hc_n++] = wep;
                }
                const bool hc_active = !g_in_menu.load() && !g_cut2d_engaged.load()
                                    && !halo::g_unit_mounted.load(std::memory_order_relaxed);
                hc_own = halo::height_tick(hp, so.y, hc_active, game_window_focused(), g_last_dt.load(),
                                           hc_ignore, hc_n, &hc_y);
            }
