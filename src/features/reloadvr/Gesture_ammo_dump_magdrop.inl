// reloadvr (fork feature, Experimental): the rounds-loaded probe (wpnammodump) and the dropped magazine (magdrop).
// Textual fragment, included by Gesture.cpp in its anonymous namespace, before the reload state machine. Moved verbatim; not compiled on its own.
// ---- WPNAMMODUMP (dev): which field of the weapon object is ROUNDS LOADED? Every tick the
// first 0x400 bytes of the resolved weapon object are compared to the previous tick as 16-bit
// words; a word that dropped by exactly one is logged with its offset. Firing a few rounds
// names the counter (and possibly a mirror or two); a game reload shows it jump back up.
// The rack's live-round eject is one write to that field.
void wpn_ammo_dump_tick() {
    if (!g_cfg.wpn_ammo_dump) return;
    // SMALL INTEGERS, AT THE SHOT. The first pass flagged 16246 -> 16245: the high halves of
    // floats ticking, not a round count. A magazine counter is a small number that drops by one
    // within a few ticks of the trigger, so: values under 2000, and only while the player's
    // fire stamp is fresh. Span widened past the node block (+0x344 on the pistol).
    constexpr int SPAN = 0x800;
    static uint16_t  s_prev[SPAN / 2];
    static uintptr_t s_obj = 0;
    static bool      s_have = false;
    static int       s_lines = 0;
    const uintptr_t obj = g_wpn_obj_ptr.load(std::memory_order_relaxed);
    if (obj == 0 || IsBadReadPtr((const void*)obj, SPAN)) { s_have = false; return; }
    if (obj != s_obj) { s_obj = obj; s_have = false; }
    const uint16_t* cur = reinterpret_cast<const uint16_t*>(obj);
    const long long since_fire = now_ticks() - g_ft_fire_at.load(std::memory_order_relaxed);
    const bool fresh = since_fire >= 0 && since_fire < ms_to_ticks(150);
    if (s_have) {
        for (int i = 0; i < SPAN / 2 && s_lines < 120; ++i) {
            if (fresh && s_prev[i] < 2000 && cur[i] + 1 == s_prev[i]) {
                ++s_lines;
                API::get()->log_info("[Halo-CampE-UEVR] WPNAMMO: +0x%03X u16 %u -> %u (at the shot)", (unsigned)(i * 2), s_prev[i], cur[i]);
            }
            // A RELOAD refills: the loaded counter jumps UP by several; the total does not move.
            if (s_prev[i] < 2000 && cur[i] < 2000 && cur[i] >= s_prev[i] + 2 && cur[i] - s_prev[i] <= 200) {
                ++s_lines;
                API::get()->log_info("[Halo-CampE-UEVR] WPNAMMO: +0x%03X u16 %u -> %u (jump up)", (unsigned)(i * 2), s_prev[i], cur[i]);
            }
        }
    }
    // NAMED CANDIDATES, ANY CHANGE, ANY TIME. +0x2BE went 12 -> 11 at a shot (the CE pistol
    // holds 12) and the +1 of a topped-up reload never clears the jump filter above; the two
    // counters named before it (+0x236, +0x29E) count DOWN every tick, not per shot.
    if (s_have) {
        static const int kWatch[] = {0x2BC, 0x2BE, 0x2C0, 0x2C8, 0x2CA, 0x2CC};
        static int s_wl = 0;
        for (int off : kWatch) {
            const int i = off / 2;
            if (cur[i] != s_prev[i] && s_wl < 300) {
                ++s_wl;
                API::get()->log_info("[Halo-CampE-UEVR] WPNAMMO watch +0x%03X: %u -> %u", (unsigned)off, s_prev[i], cur[i]);
            }
        }
    }
    memcpy(s_prev, cur, sizeof(s_prev));
    s_have = true;
}

// ---- THE DROPPED MAGAZINE (magdrop). On the reload press the gun's own magazine is hidden;
// a copy of its mesh is spawned in its place, detached from the weapon, given collision and
// physics, and falls. Removed after magdrop_ms. Pure presentation: the game's reload is
// untouched. Sequence matters: place, detach (keep world), collision profile, collision on,
// simulate -- a component that simulates while attached fights its parent every frame.
TrackedObject s_magdrop, s_magdrop_proxy;
long long     s_magdrop_at = 0;
API::UObject* sp_drop_proxy_for(API::UObject* m);   // defined with the parts code below
void mag_drop_spawn() {
    if (!g_cfg.mag_drop) return;
    auto* mc = s_mag_hidden.get();
    if (mc == nullptr) return;
    auto** pm = mc->get_property_data<API::UObject*>(L"StaticMesh");
    if (pm == nullptr || IsBadReadPtr(pm, sizeof(void*)) || *pm == nullptr) return;
    Vec3 loc{}, rot{}, scl{1.0f, 1.0f, 1.0f};
    if (!call_ret_vec3(mc, L"K2_GetComponentLocation", &loc)) return;
    call_ret_vec3(mc, L"K2_GetComponentRotation", &rot);
    call_ret_vec3(mc, L"K2_GetComponentScale", &scl);
    if (s_mag_hid_mode == 3) {   // hidden by scale: the drop takes the magazine's own size
        scl.x = (float)(scl.x / 0.001 * s_mag_hid_scale[0]);
        scl.y = (float)(scl.y / 0.001 * s_mag_hid_scale[1]);
        scl.z = (float)(scl.z / 0.001 * s_mag_hid_scale[2]);
    }
    auto* owner = API::get()->get_local_pawn(0);
    if (owner == nullptr) return;
    // A previous drop still falling is destroyed first; one at a time keeps this cheap.
    if (auto* old = s_magdrop.get()) { alignas(16) uint8_t p[64] = {0}; ue_destroy_component(old); }
    s_magdrop = TrackedObject{};
    auto* m = holster_marker_spawn_mesh(owner, *pm, (double)(scl.x > 0.01f ? scl.x : 1.0f));
    if (m == nullptr) return;
    holster_marker_place_rot(m, loc, rot.x, rot.y, rot.z);
    holster_marker_show(m, true);
    { alignas(16) uint8_t p[64] = {0}; p[0] = 1; p[1] = 1; p[2] = 1; p[3] = 0;   // KeepWorld x3, no modify
      m->call_function(L"K2_DetachFromComponent", p); }
    { alignas(16) uint8_t p[64] = {0}; API::FName nm = make_fname(L"PhysicsActor");
      memcpy(p, &nm, sizeof(int32_t) * 2); p[8] = 1;   // bUpdateOverlaps
      m->call_function(L"SetCollisionProfileName", p); }
    { alignas(16) uint8_t p[64] = {0}; p[0] = 3;   // ECollisionEnabled::QueryAndPhysics
      m->call_function(L"SetCollisionEnabled", p); }
    if (g_cfg.slide_part_dropmode == 1) {
        { alignas(16) uint8_t p[64] = {0}; m->call_function(L"SetAbsolute", p); }   // follow the parent again
        if (auto* box = sp_drop_proxy_for(m)) s_magdrop_proxy.set(box);
    } else {
        alignas(16) uint8_t p[64] = {0}; p[0] = 1;
        m->call_function(L"SetSimulatePhysics", p);
    }
    s_magdrop.set(m);
    s_magdrop_at = now_ticks();
    if (g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD mag dropped at (%.0f %.0f %.0f), scale %.2f", loc.x, loc.y, loc.z, scl.x);
}
void mag_drop_tick() {
    if (s_magdrop_at == 0) return;
    if (now_ticks() - s_magdrop_at < ms_to_ticks(g_cfg.mag_drop_ms)) return;
    if (auto* m = s_magdrop.get()) ue_destroy_component(m);
    if (auto* b = s_magdrop_proxy.get()) ue_destroy_component(b);
    s_magdrop = TrackedObject{}; s_magdrop_proxy = TrackedObject{};
    s_magdrop_at = 0;
}
