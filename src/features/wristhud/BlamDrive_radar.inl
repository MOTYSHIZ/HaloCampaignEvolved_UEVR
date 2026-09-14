// wristhud (fork feature, Experimental): the blip survey (blipdump) and the wrist radar scan, sim thread.
// Textual fragment, included by BlamDrive.cpp at namespace halo scope, before the seat publish. Moved verbatim; not compiled on its own.
// ---- BLIPDUMP (wrist-radar survey, doctrine in Config.hpp). SIM THREAD, one shot per value
// change. Walks the object table the same way resolve_object_by_datum does, collects everything
// with a plausible position within ~36 m of the player, and dumps each header -- the diff between
// a marines-only capture and a covenant-only capture names the team byte.
void blip_dump_probe(uintptr_t player_obj) {
    static int s_armed_as = 0;
    if (g_cfg.blip_dump == s_armed_as) return;
    if (IsBadReadPtr((const void*)(player_obj + 0x20), 12)) return;
    s_armed_as = g_cfg.blip_dump;
    const float* pp = (const float*)(player_obj + 0x20);

    const uintptr_t tls_array = (uintptr_t)__readgsqword(0x58);
    uintptr_t block = 0, ctx = 0, table = 0;
    if (tls_array == 0 ||
        !read_ptr(tls_array + (uintptr_t)g_tls_index * 8, &block) || block == 0 ||
        !read_ptr(block + 0x20, &ctx) || ctx == 0 ||
        !read_ptr(ctx + 0x50, &table) || table == 0) {
        API::get()->log_info("[Halo-CampE-UEVR] BLIPDUMP: table chain unresolved");
        return;
    }

    // 0x400 per object: the 0x100 sweep produced only spawn-order artifacts (the E1-vs-E2 salt
    // at +0xCC ages the object, it does not side it), so the team field lives deeper. The player
    // dumps too, labeled SELF -- its team must equal the marines', which prunes candidates hard.
    auto dump_obj = [&](uintptr_t o, int idx, float dist_m) {
        const uintptr_t span = !IsBadReadPtr((const void*)o, 0x400) ? 0x400 : 0x100;
        const uint32_t* d = (const uint32_t*)o;
        char buf[352];
        for (uintptr_t off = 0; off < span; off += 0x80) {
            int n = 0;
            for (int i = (int)(off / 4); i < (int)(off / 4) + 32 && n < (int)sizeof(buf) - 12; ++i)
                n += snprintf(buf + n, sizeof(buf) - n, "%08X ", d[i]);
            API::get()->log_info("[Halo-CampE-UEVR] BLIPDUMP cap=%d idx=%d dist=%.1fm +0x%03X | %s",
                                 g_cfg.blip_dump, idx, dist_m, (unsigned)off, buf);
        }
    };
    if (!IsBadReadPtr((const void*)player_obj, 0x100)) dump_obj(player_obj, -1, 0.0f);   // SELF
    // 24 hits, not 12: the table's low indices are load-time scenery, and a 12-object budget
    // filled with it before ever reaching the late-spawned actors the survey exists to catch.
    int hits = 0;
    for (int idx = 0; idx < 4096 && hits < 24; ++idx) {
        uintptr_t o = 0;
        if (!read_ptr(table + (uintptr_t)idx * 24 + 0x10, &o) || o == 0) continue;
        if (o == player_obj || IsBadReadPtr((const void*)(o + 0x20), 12)) continue;
        const float* q = (const float*)(o + 0x20);
        const float dx = q[0] - pp[0], dy = q[1] - pp[1], dz = q[2] - pp[2];
        const float d2 = dx * dx + dy * dy + dz * dz;
        if (!(d2 == d2) || d2 > 12.0f * 12.0f) continue;   // NaN or beyond ~36 m
        if (IsBadReadPtr((const void*)o, 0x100)) continue;
        ++hits;
        dump_obj(o, idx, std::sqrt(d2) * 3.048f);
    }
    API::get()->log_info("[Halo-CampE-UEVR] BLIPDUMP cap=%d done: %d neighbours within 36 m",
                         g_cfg.blip_dump, hits);
}

// ---- WRIST RADAR SCAN (doctrine at the exports in BlamDrive.hpp). Two cadences, per the
// vehfacing lesson (a table walk per publish shook the whole picture): the TABLE walk that finds
// unit-like objects runs every ~2500 calls (~0.3 Hz), caching pointers; the cheap position reads
// off the cache run every 32nd call. Everything sim-thread-local except the published atomics.
void blip_scan(uintptr_t player_obj) {
    constexpr int MAX_TRACK = 24;
    static uintptr_t s_track[MAX_TRACK];
    static int       s_team[MAX_TRACK];
    static uint8_t   s_raw[MAX_TRACK];
    static uint32_t  s_type[MAX_TRACK];
    static float     s_px[MAX_TRACK], s_py[MAX_TRACK];   // last-sampled position (moving flag)
    static bool      s_moving[MAX_TRACK];
    // SPEED over WALL-CLOCK time, not displacement per N calls: the call cadence was never
    // measured, and the log caught the consequence red-handed -- contacts sprinting between
    // samples, every one flagged mv=0, no dots at all (and the old strobing dots were the same
    // broken window occasionally crossing threshold by luck). Two consecutive >=0.10 blam-unit/s
    // (~0.3 m/s) samples at >=130 ms spacing = a mover; single spikes stay filtered.
    static uint8_t   s_mvrun[MAX_TRACK];
    static long long s_mvat[MAX_TRACK];
    static int       s_ntrack = 0;
    static uint32_t  s_call = 0;
    ++s_call;

    if (IsBadReadPtr((const void*)(player_obj + 0x20), 12)) return;
    const float* pp = (const float*)(player_obj + 0x20);

    // REBUILD ON A WALL CLOCK, never on a call count. This was every 2500 calls, assumed to be
    // ~1 s from a sim rate that was never re-measured; it is actually ~125 ms, so the rebuild
    // wiped the movement window (130 ms) before it could ever close -- speeds measured fine at
    // 0.7-1.2 u/s while the run counter was reset to 0 forever and nothing ever counted as
    // moving. Probe-sampling aliasing, and the second time in this project.
    static long long s_scan_at = 0;
    const long long scan_now = now_ms();
    if (scan_now - s_scan_at >= 1000) {
        s_scan_at = scan_now;
        const uintptr_t tls_array = (uintptr_t)__readgsqword(0x58);
        uintptr_t block = 0, ctx = 0, table = 0;
        if (tls_array != 0 &&
            read_ptr(tls_array + (uintptr_t)g_tls_index * 8, &block) && block != 0 &&
            read_ptr(block + 0x20, &ctx) && ctx != 0 &&
            read_ptr(ctx + 0x50, &table) && table != 0) {
            // Snapshot the old set so a contact that survives the rebuild keeps its movement
            // history. Rebuilding into fresh state is what made the window unclosable.
            uintptr_t old_track[MAX_TRACK];
            float     old_px[MAX_TRACK], old_py[MAX_TRACK];
            bool      old_moving[MAX_TRACK];
            uint8_t   old_run[MAX_TRACK];
            long long old_at[MAX_TRACK];
            const int old_n = s_ntrack;
            for (int k = 0; k < old_n; ++k) {
                old_track[k] = s_track[k]; old_px[k] = s_px[k]; old_py[k] = s_py[k];
                old_moving[k] = s_moving[k]; old_run[k] = s_mvrun[k]; old_at[k] = s_mvat[k];
            }
            s_ntrack = 0;
            for (int idx = 0; idx < 4096 && s_ntrack < MAX_TRACK; ++idx) {
                uintptr_t o = 0;
                if (!read_ptr(table + (uintptr_t)idx * 24 + 0x10, &o) || o == 0) continue;
                if (o == player_obj || IsBadReadPtr((const void*)o, 0x180)) continue;
                // Unit-like: unattached, and the TEAM byte reads human or covenant.
                if (*(const uint32_t*)(o + 0x0C) != 0xFFFFFFFFu) continue;
                const uint8_t team = *(const uint8_t*)(o + 0x177);
                if (team != 0x0E && team != 0x0D) continue;
                const float* q = (const float*)(o + 0x20);
                const float ddx = q[0] - pp[0], ddy = q[1] - pp[1];
                if (!(ddx == ddx) || ddx * ddx + ddy * ddy > 20.0f * 20.0f) continue;
                s_track[s_ntrack] = o;
                s_team[s_ntrack] = (team == 0x0D) ? 1 : 0;
                s_raw[s_ntrack] = team;
                s_type[s_ntrack] = *(const uint32_t*)o;
                int prev = -1;
                for (int k = 0; k < old_n; ++k) if (old_track[k] == o) { prev = k; break; }
                if (prev >= 0) {
                    s_px[s_ntrack] = old_px[prev]; s_py[s_ntrack] = old_py[prev];
                    s_moving[s_ntrack] = old_moving[prev];
                    s_mvrun[s_ntrack] = old_run[prev];
                    s_mvat[s_ntrack] = old_at[prev];
                } else {
                    s_px[s_ntrack] = q[0]; s_py[s_ntrack] = q[1];
                    s_moving[s_ntrack] = false;
                    s_mvrun[s_ntrack] = 0;
                    s_mvat[s_ntrack] = scan_now;
                }
                ++s_ntrack;
            }
        }
    }

    // Positions publish every 8th call (the render-side jitter fix: ~4x the old rate); the
    // MOVING test stays on the original 32-call baseline, because its 2 cm threshold was sized
    // for the ~0.13 s window and a faster window can no longer see a slow walker over the noise.
    // FACTION-FIELD SURVEY (doctrine at Config::blip_bytes). Fires once per value change, on the
    // cached contact set, so both captures cover the same object shape.
    {
        static int s_bytes_armed = 0;
        if (g_cfg.blip_bytes != s_bytes_armed) {
            s_bytes_armed = g_cfg.blip_bytes;
            if (s_bytes_armed != 0) {
                for (int i = 0; i < s_ntrack; ++i) {
                    const uintptr_t o = s_track[i];
                    if (o == 0 || IsBadReadPtr((const void*)o, 0x200)) continue;
                    // TWO windows. The HEADER first: a Blam object's leading dwords carry its
                    // tag / definition index, which is a real SPECIES identity rather than the
                    // coarse body class at +0x177 (a marine and an Elite share that). If a
                    // header dword tracks species one-for-one, per-species colour keys on it.
                    char hdr[3 * 32 + 1];
                    int hw = 0;
                    for (int b = 0; b < 32; ++b)
                        hw += sprintf_s(hdr + hw, sizeof(hdr) - (size_t)hw, "%02X ",
                                        *(const uint8_t*)(o + b));
                    char buf[3 * 96 + 1];
                    int w = 0;
                    for (int b = 0; b < 96; ++b)
                        w += sprintf_s(buf + w, sizeof(buf) - (size_t)w, "%02X ",
                                       *(const uint8_t*)(o + 0x140 + b));
                    const float* q = (const float*)(o + 0x20);
                    const float ddx = q[0] - pp[0], ddy = q[1] - pp[1];
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] BLIPBYTES %d id=%08X cls=%02X d=%.1f +0x00: %s| +0x140: %s",
                        s_bytes_armed, (uint32_t)(o & 0xFFFFFFFFu), s_raw[i],
                        std::sqrt(ddx * ddx + ddy * ddy), hdr, buf);
                }
                API::get()->log_info("[Halo-CampE-UEVR] BLIPBYTES %d done: %d contacts",
                                     s_bytes_armed, s_ntrack);
            }
        }
    }

    if ((s_call % 8u) != 0u) return;
    int n = 0;
    for (int i = 0; i < s_ntrack && n < MAX_BLIPS; ++i) {
        const uintptr_t o = s_track[i];
        if (o == 0 || IsBadReadPtr((const void*)o, 0x180)) continue;
        if (*(const uint8_t*)(o + 0x177) != (s_team[i] == 1 ? 0x0D : 0x0E)) continue;  // slot recycled
        const float* q = (const float*)(o + 0x20);
        const float dx = q[0] - pp[0], dy = q[1] - pp[1];
        if (!(dx == dx) || dx * dx + dy * dy > 8.2f * 8.2f) continue;   // 25 m radar range
        // MOVING, the real tracker's rule: position changed since the last publish sample
        // (~0.13 s at this cadence; 2 cm noise floor).
        bool moving = s_moving[i];
        float dbg_speed = -1.0f;
        long long dbg_dtm = 0;
        {
            const long long nowm = now_ms();
            const long long dtm = nowm - s_mvat[i];
            dbg_dtm = dtm;
            if (dtm >= 130) {
                const float mx = q[0] - s_px[i], my = q[1] - s_py[i];
                const float speed = std::sqrt(mx * mx + my * my) * 1000.0f / (float)dtm;
                s_px[i] = q[0]; s_py[i] = q[1];
                s_mvat[i] = nowm;
                s_mvrun[i] = (speed > 0.10f) ? (uint8_t)((s_mvrun[i] < 250) ? s_mvrun[i] + 1 : 250) : 0;
                moving = s_mvrun[i] >= 2;   // sustained -- one twitch is a settle, not a contact
                s_moving[i] = moving;
                dbg_speed = speed;
            }
        }
        g_blip_dx[n].store(dx, std::memory_order_relaxed);
        g_blip_dy[n].store(dy, std::memory_order_relaxed);
        g_blip_team[n].store(s_team[i], std::memory_order_relaxed);
        g_blip_moving[n].store(moving, std::memory_order_relaxed);
        g_blip_id[n].store((uint32_t)(o & 0xFFFFFFFFu), std::memory_order_relaxed);
        g_blip_raw[n].store((uint32_t)s_raw[i], std::memory_order_relaxed);
        g_blip_type[n].store(s_type[i], std::memory_order_relaxed);
        g_blip_wx[n].store(q[0], std::memory_order_relaxed);
        g_blip_wy[n].store(q[1], std::memory_order_relaxed);
        g_blip_wz[n].store(q[2], std::memory_order_relaxed);
        if (dbg_speed >= 0.0f) g_blip_speed[n].store(dbg_speed, std::memory_order_relaxed);
        g_blip_dtm[n].store((int)dbg_dtm, std::memory_order_relaxed);
        g_blip_run[n].store((int)s_mvrun[i], std::memory_order_relaxed);
        ++n;
    }
    g_blip_count.store(n, std::memory_order_relaxed);
}
