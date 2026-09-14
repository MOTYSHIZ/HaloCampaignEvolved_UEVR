// palettewpn (fork feature, Experimental): the tick-start instruments (game thread id, tick id, STOMPLOG points 0 and 24) and pose latch site 1.
// Textual fragment, included by Plugin.cpp inside the engine tick callback, at its start. Moved verbatim; not compiled on its own.
        // The game thread's id and the tick phase, for the sim hook's phase instrument (see
        // BlamPalette.hpp): the palette hook tells 'inside the engine tick' from 'from the sim
        // thread' by these two.
        g_game_tid.store((uint32_t)GetCurrentThreadId(), std::memory_order_relaxed);
        halo::g_tick_id.fetch_add(1, std::memory_order_relaxed);
        stomp_sample(0);
        {   // Point 24: the Blam control record at tick start, before this tick's writes.
            float ry = 0.0f, rp = 0.0f;
            if (g_cfg.stomp_log != 0 && halo::blam_ctl_read_ue_deg(&ry, &rp))
                halo::stomp_mark(24, ry, rp, (float)halo::g_tick_id.load(std::memory_order_relaxed),
                                 (float)halo::g_latch_gen.load(std::memory_order_relaxed));
        }
        // POSELATCH sites 1/2 (palette weapon mode): the frame's one hand sample, taken before
        // anything this frame reads it and before update() publishes the palette poses.
        halo::pose_latch_refresh(1);
