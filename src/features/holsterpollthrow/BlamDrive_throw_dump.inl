// holsterpollthrow (fork feature, Experimental): the throw windup dump (throwdump, dev), sim thread.
// Textual fragment, included by BlamDrive.cpp at namespace halo scope, before the radar scan. Moved verbatim; not compiled on its own.
void throw_dump_probe(uintptr_t obj) {
    constexpr uintptr_t SPAN = 0x600;
    constexpr int       NDW  = (int)(SPAN / 4);
    static uint32_t  s_prev[NDW];
    static uint8_t   s_noisy[NDW];
    static uintptr_t s_obj = 0;
    static int  s_armed_as = 0;
    static bool s_have_prev = false;
    static int  s_learn = 0;
    static int  s_window = 0;
    static int  s_t = 0;
    static int  s_lines = 0;
    static bool s_press_prev = false;
    static int  s_frag0 = -1, s_plas0 = -1;

    // A new object or a bumped throwdump value restarts the learn from scratch.
    if (obj != s_obj || g_cfg.throw_dump != s_armed_as) {
        s_obj = obj; s_armed_as = g_cfg.throw_dump;
        s_have_prev = false; s_learn = 0; s_window = 0; s_press_prev = false;
        memset(s_noisy, 0, sizeof(s_noisy));
    }
    if (IsBadReadPtr((const void*)obj, SPAN)) return;
    const uint32_t* cur = (const uint32_t*)obj;
    const uint8_t*  u8  = (const uint8_t*)obj;

    // ---- GRENTRACK: the projectile the spawn hook just created, sampled ~every 30 ms for 1.2 s.
    // Whether the position MOVES from the first sample is the whole question: flies-immediately
    // means the delay is presentation, sits-then-launches means an animation event holds it.
    // +0x20 as the position is the UNIT layout's offset, unverified for projectiles -- if the
    // samples read as garbage, that is the finding, not a malfunction.
    {
        const uintptr_t gobj = g_grentrack_obj.load(std::memory_order_relaxed);
        if (gobj != 0) {
            const long long since = now_ms() - g_grentrack_at_ms.load(std::memory_order_relaxed);
            static long long s_last = 0;
            if (since > 1200) {
                g_grentrack_obj.store(0, std::memory_order_relaxed);
                API::get()->log_info("[Halo-CampE-UEVR] GRENTRACK end (+%lld ms)", since);
            } else if (now_ms() - s_last >= 30) {
                s_last = now_ms();
                if (!IsBadReadPtr((const void*)(gobj + 0x20), 12)) {
                    const float* q = (const float*)(gobj + 0x20);
                    API::get()->log_info("[Halo-CampE-UEVR] GRENTRACK +%lld ms pos=(%.4f,%.4f,%.4f)",
                                         since, q[0], q[1], q[2]);
                    // GRENSNAP: the object's head at three moments -- created (held), mid-hold,
                    // and in flight. The dwords dead in the first two that match the measured
                    // ~8 blam-units/s in the third are the VELOCITY; whatever flips between the
                    // second and third is the RELEASE mechanism. One throw answers both.
                    static uintptr_t s_snap_obj = 0;
                    static uint8_t   s_snapped = 0;
                    if (gobj != s_snap_obj) { s_snap_obj = gobj; s_snapped = 0; }
                    int want_snap = -1;
                    if      (since < 60   && !(s_snapped & 1)) { want_snap = 0; s_snapped |= 1; }
                    else if (since >= 100 && since < 220 && !(s_snapped & 2)) { want_snap = 1; s_snapped |= 2; }
                    else if (since >= 300 && !(s_snapped & 4)) { want_snap = 2; s_snapped |= 4; }
                    if (want_snap >= 0 && !IsBadReadPtr((const void*)gobj, 0x100)) {
                        const uint32_t* d = (const uint32_t*)gobj;
                        char buf[352];
                        for (int half = 0; half < 2; ++half) {
                            int n = 0;
                            for (int i = half * 32; i < half * 32 + 32 && n < (int)sizeof(buf) - 12; ++i)
                                n += snprintf(buf + n, sizeof(buf) - n, "%08X ", d[i]);
                            API::get()->log_info("[Halo-CampE-UEVR] GRENSNAP phase=%d +%lld ms +0x%02X | %s",
                                                 want_snap, since, half * 0x80, buf);
                        }
                    }
                } else {
                    g_grentrack_obj.store(0, std::memory_order_relaxed);
                    API::get()->log_info("[Halo-CampE-UEVR] GRENTRACK object unreadable (+%lld ms) -- dropped", since);
                }
            }
        }
    }

    const bool press = holster_throw_press_active();
    const bool edge  = press && !s_press_prev;
    s_press_prev = press;

    if (!s_have_prev) { memcpy(s_prev, cur, SPAN); s_have_prev = true; return; }

    // THE RELEASE WATCH runs EVERY call, window or not. The first cut only watched inside the
    // window, and the window turned out to cover 50 ms: the learn counters measured this hook at
    // ~8000 calls/sec, 25x the assumed rate -- state the probe's own rate before trusting any rate
    // it reports. The count decrement is the game letting go; ms-since-press is the windup.
    static long long s_press_at = 0;
    static int s_frag_w = -1, s_plas_w = -1;
    if (edge) s_press_at = now_ms();
    if (s_frag_w >= 0 && ((int)u8[0x382] != s_frag_w || (int)u8[0x383] != s_plas_w)) {
        const long long since = s_press_at != 0 ? now_ms() - s_press_at : -1;
        API::get()->log_info("[Halo-CampE-UEVR] THROWDUMP RELEASE %+lld ms after press "
                             "(frag %d->%d plasma %d->%d)%s",
                             since, s_frag_w, (int)u8[0x382], s_plas_w, (int)u8[0x383],
                             s_window > 0 ? "" : " [outside window]");
    }
    s_frag_w = (int)u8[0x382]; s_plas_w = (int)u8[0x383];

    if (s_window <= 0) {
        for (int i = 0; i < NDW; ++i)
            if (cur[i] != s_prev[i]) s_noisy[i] = 1;
        ++s_learn;
        if (edge) {
            int masked = 0;
            for (int i = 0; i < NDW; ++i) masked += s_noisy[i];
            // 12000 calls at the MEASURED ~8 kHz is ~1.5 s -- long enough for any windup.
            s_window = 12000; s_t = 0; s_lines = 0;
            s_frag0 = (int)u8[0x382]; s_plas0 = (int)u8[0x383];
            API::get()->log_info("[Halo-CampE-UEVR] THROWDUMP armed: %d idle calls learned, "
                                 "%d/%d dwords masked, frag=%d plasma=%d",
                                 s_learn, masked, NDW, s_frag0, s_plas0);
        }
        memcpy(s_prev, cur, SPAN);
        return;
    }

    ++s_t; --s_window;
    // ---- UNITSNAP: the whole unit head at four moments DURING the windup. The animation clock
    // ticks in idle too, so the noise mask hides it by design; a clock cannot hide from a linear
    // fit across timed snapshots -- any field advancing by equal steps between these four is a
    // clock candidate, and writing one forward is the clean instant-throw (the game's own release
    // fires early, with its physics registration and fuse intact -- the field-poked release
    // produced a grenade frozen outside the simulation, which closed that route).
    {
        static uint8_t s_usnapped = 0;
        if (s_t == 1) s_usnapped = 0;
        const long long pms = now_ms() - s_press_at;
        int phase = -1;
        if      (pms >= 40  && !(s_usnapped & 1)) { phase = 0; s_usnapped |= 1; }
        else if (pms >= 90  && !(s_usnapped & 2)) { phase = 1; s_usnapped |= 2; }
        else if (pms >= 140 && !(s_usnapped & 4)) { phase = 2; s_usnapped |= 4; }
        else if (pms >= 190 && !(s_usnapped & 8)) { phase = 3; s_usnapped |= 8; }
        if (phase >= 0) {
            // 0x2000 when readable: the 0x1000 sweep found only mirrors of the global tick
            // counter, so the animation block (index + frame, the actual windup clock) lives
            // deeper in the unit if it lives in the unit at all.
            const uintptr_t uspan = !IsBadReadPtr((const void*)obj, 0x2000) ? 0x2000
                                  : !IsBadReadPtr((const void*)obj, 0x1000) ? 0x1000 : SPAN;
            const uint32_t* d = (const uint32_t*)obj;
            char buf[352];
            for (uintptr_t off = 0; off < uspan; off += 0x80) {
                int n = 0;
                for (int i = (int)(off / 4); i < (int)(off / 4) + 32 && n < (int)sizeof(buf) - 12; ++i)
                    n += snprintf(buf + n, sizeof(buf) - n, "%08X ", d[i]);
                API::get()->log_info("[Halo-CampE-UEVR] UNITSNAP phase=%d +%lld ms +0x%03X | %s",
                                     phase, pms, (unsigned)off, buf);
            }
        }
    }
    if (s_lines < 120) {
        char buf[352]; int n = 0; int shown = 0, more = 0;
        for (int i = 0; i < NDW; ++i) {
            if (cur[i] == s_prev[i] || s_noisy[i] != 0) continue;
            if (shown < 8 && n < (int)sizeof(buf) - 40) {
                n += snprintf(buf + n, sizeof(buf) - n, "+0x%03X %08X->%08X  ", i * 4, s_prev[i], cur[i]);
                ++shown;
            } else ++more;
        }
        if (shown > 0) {
            ++s_lines;
            API::get()->log_info("[Halo-CampE-UEVR] THROWDUMP t=%d (%+lld ms) | %s(+%d more)",
                                 s_t, now_ms() - s_press_at, buf, more);
        }
    }
    if (s_window == 0)
        API::get()->log_info("[Halo-CampE-UEVR] THROWDUMP window end (t=%d, %d lines logged)",
                             s_t, s_lines);
    memcpy(s_prev, cur, SPAN);
}
