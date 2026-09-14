// holsterpollthrow (fork feature, Experimental): the grenade spawn origin moved to the carrier hand (grenhand).
// Textual fragment, included by BlamAim.cpp inside hooked_create_projectile(), before the original call. Moved verbatim; not compiled on its own.
    // ---- GRENHAND (doctrine in Config.hpp): rewrite the spawn ORIGIN to the carrier hand,
    // BEFORE the constructor consumes the params. Gated on the synthetic throw press (the spawn
    // measured ~42 ms into the 120 ms press window), so gunfire and NPC spawns are never touched.
    if (g_cfg.gren_hand_spawn != 0 && holster_throw_press_active()
        && params != 0 && !IsBadReadPtr((void*)(params + P_VEC1), 12)) {
        float hx = 0.0f, hy2 = 0.0f, hz = 0.0f;
        if (holster_hand_blam(&hx, &hy2, &hz)) {
            float* o = (float*)(params + P_VEC1);
            API::get()->log_info("[Halo-CampE-UEVR] GRENHAND origin (%.4f,%.4f,%.4f) -> (%.4f,%.4f,%.4f)",
                                 o[0], o[1], o[2], hx, hy2, hz);
            o[0] = hx; o[1] = hy2; o[2] = hz;
        }
    }
