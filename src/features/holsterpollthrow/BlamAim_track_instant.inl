// holsterpollthrow (fork feature, Experimental): the grenade-track arm and the instant release at spawn (greninstant 1).
// Textual fragment, included by BlamAim.cpp inside hooked_create_projectile(), after the original call. Moved verbatim; not compiled on its own.
    // ---- GRENTRACK arm: try the return value as an object datum, on this thread (the resolve
    // walks the sim TLS, which only this thread owns). A failed resolve is logged as itself --
    // it means the return value is not a datum, and the tracker needs a different handle.
    if ((g_cfg.throw_dump != 0 || g_cfg.gren_instant != 0) && holster_throw_press_active() && cret != 0) {
        const long long tnow_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const uintptr_t gobj = resolve_object_by_datum((uint32_t)cret);
        if (g_cfg.throw_dump != 0) {
            g_grentrack_obj.store(gobj, std::memory_order_relaxed);
            g_grentrack_at_ms.store(tnow_ms, std::memory_order_relaxed);
            API::get()->log_info("[Halo-CampE-UEVR] GRENTRACK: create ret=0x%llX -> datum 0x%08X obj=0x%llX",
                                 (unsigned long long)cret, (uint32_t)cret, (unsigned long long)gobj);
        }
        // ---- GRENINSTANT (doctrine in Config.hpp): perform the release RIGHT NOW, exactly as
        // GRENSNAP watched the keyframe do it at ~250 ms -- detach from the throw-hand bone,
        // set the released state and flag, write velocity along the player's own swing.
        if (g_cfg.gren_instant == 1 && gobj != 0 && !IsBadReadPtr((void*)gobj, 0x80)) {
            float dx = 0.0f, dy = 1.0f, dz = 0.0f;
            const bool have_dir = holster_throw_blam_dir(&dx, &dy, &dz);
            uint8_t* p8 = (uint8_t*)gobj;
            *(uint32_t*)(p8 + 0x0C) = 0xFFFFFFFFu;
            *(uint32_t*)(p8 + 0x14) = 0xFFFFFFFFu;
            *(uint32_t*)(p8 + 0x18) = 0xFFFF00FFu;
            *(uint32_t*)(p8 + 0x08) = 0x00000004u;
            *(uint32_t*)(p8 + 0x04) |= 0x80u;
            float* vel = (float*)(p8 + 0x68);
            vel[0] = dx * g_cfg.gren_speed;
            vel[1] = dy * g_cfg.gren_speed;
            vel[2] = dz * g_cfg.gren_speed;
            g_greninst_vx.store(vel[0], std::memory_order_relaxed);
            g_greninst_vy.store(vel[1], std::memory_order_relaxed);
            g_greninst_vz.store(vel[2], std::memory_order_relaxed);
            g_greninst_obj.store(gobj, std::memory_order_relaxed);
            g_greninst_at_ms.store(tnow_ms, std::memory_order_relaxed);
            API::get()->log_info("[Halo-CampE-UEVR] GRENINSTANT: released at spawn, vel=(%.2f,%.2f,%.2f) swing_dir=%d",
                                 vel[0], vel[1], vel[2], (int)have_dir);
        }
    }
