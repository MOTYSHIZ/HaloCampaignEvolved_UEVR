// holsterpollthrow (fork feature, Experimental): the instant-release experiments (greninstant 2: backdated stamp; 3: hold the release).
// Textual fragment, included by core/UnitState.cpp inside publish_unit_state(), after the radar scan. Moved verbatim; not compiled on its own.
    // ---- GRENINSTANT mode 2: backdate the throw-start stamp (doctrine in Config.hpp). The
    // stamp at unit+0x38C is written by the game within ~1 ms of the press; the first probe call
    // that sees it change during the press window rewrites it N ticks into the past, once per
    // throw. If the release is timed against it, the game's own release fires immediately -- and
    // if nothing changes, the stamp was bookkeeping, which is an answer too.
    if (g_cfg.gren_instant == 2 && !IsBadReadPtr((const void*)(obj + 0x38C), 4)) {
        static uint32_t s_stamp_prev = 0;
        static bool s_backdated = false;
        const uint32_t st = *(const uint32_t*)(obj + 0x38C);
        if (!holster_throw_press_active()) {
            s_backdated = false;
            s_stamp_prev = st;
        } else if (!s_backdated && st != s_stamp_prev) {
            s_backdated = true;
            const uint32_t bd = st - (uint32_t)g_cfg.gren_backdate;
            *(uint32_t*)(obj + 0x38C) = bd;
            s_stamp_prev = bd;
            API::get()->log_info("[Halo-CampE-UEVR] GRENBACKDATE: stamp 0x%08X -> 0x%08X (-%d ticks)",
                                 st, bd, g_cfg.gren_backdate);
        }
    }

    // ---- GRENINSTANT v3. Version 2 (fight the re-attacher on the grenade alone, continuously)
    // WEDGED THE UNIT: the keyframe handler found its grenade already released, bailed before
    // clearing the unit's own throw bookkeeping, and the unit refused every later throw -- state
    // that survives checkpoints. The missing piece is the UNIT's side: unit+0x10 is the "object
    // in hand" slot (weapon datum at rest, the grenade's datum during a throw -- watched all
    // day). v3 empties that slot the moment the grenade is freed, so the re-attacher and the
    // keyframe handler both lose their reference, and restores the remembered weapon datum once
    // the animation window is over. The gun may flicker during the window; that is the cost of
    // the experiment, not the final shape.
    {
        static uint32_t  s_idle_hand = 0xFFFFFFFFu;   // unit+0x10 as it reads between throws
        static uintptr_t s_prev_io = 0;
        static bool      s_released = false;
        const uintptr_t io = g_greninst_obj.load(std::memory_order_relaxed);
        if (io != s_prev_io) { s_prev_io = io; s_released = false; }
        if (io == 0) {
            if (!IsBadReadPtr((const void*)(obj + 0x10), 4))
                s_idle_hand = *(const uint32_t*)(obj + 0x10);
        } else {
            const long long since = now_ms() - g_greninst_at_ms.load(std::memory_order_relaxed);
            if (since > 400) {
                if (s_idle_hand != 0xFFFFFFFFu && !IsBadReadPtr((void*)(obj + 0x10), 4))
                    *(uint32_t*)(obj + 0x10) = s_idle_hand;
                g_greninst_obj.store(0, std::memory_order_relaxed);
            } else if (!IsBadReadPtr((void*)io, 0x80)) {
                if (!s_released) {
                    s_released = true;
                    *(uint32_t*)(io + 0x0C) = 0xFFFFFFFFu;
                    *(uint32_t*)(io + 0x14) = 0xFFFFFFFFu;
                    *(uint32_t*)(io + 0x18) = 0xFFFF00FFu;
                    *(uint32_t*)(io + 0x08) = 0x00000004u;
                    *(uint32_t*)(io + 0x04) |= 0x80u;
                }
                if (since <= 330) {
                    // Both hands of the fight, every call: the unit's slot stays empty, the
                    // grenade stays detached, the velocity stays ours. ~25x the tick rate.
                    if (!IsBadReadPtr((void*)(obj + 0x10), 4))
                        *(uint32_t*)(obj + 0x10) = 0xFFFFFFFFu;
                    *(uint32_t*)(io + 0x0C) = 0xFFFFFFFFu;
                    float* vel = (float*)(io + 0x68);
                    vel[0] = g_greninst_vx.load(std::memory_order_relaxed);
                    vel[1] = g_greninst_vy.load(std::memory_order_relaxed);
                    vel[2] = g_greninst_vz.load(std::memory_order_relaxed);
                }
            } else {
                g_greninst_obj.store(0, std::memory_order_relaxed);
            }
        }
    }
