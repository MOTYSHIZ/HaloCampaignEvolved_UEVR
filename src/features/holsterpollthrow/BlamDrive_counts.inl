// holsterpollthrow (fork feature, Experimental): the live grenade type and pouch counts read from the unit object.
// Textual fragment, included by core/UnitState.cpp inside publish_unit_state(), after the unit-state evidence line. Moved verbatim; not compiled on its own.
    // Grenade type and counts from raw unit offsets: only for the fork's grenade-gesture variant
    // (holsterpollthrow, experimental). Off, g_unit_gvalid stays false and the author's pouches keep
    // their fail-closed "counts unknown" behaviour exactly as he shipped it.
    if (g_cfg.holster_poll_throw && !IsBadReadPtr((const void*)(obj + 0x380), 4)) {
        const uint8_t* u8 = (const uint8_t*)obj;
        g_unit_gtype.store((int)u8[0x380], std::memory_order_relaxed);
        g_unit_gfrag.store((int)u8[0x382], std::memory_order_relaxed);
        g_unit_gplasma.store((int)u8[0x383], std::memory_order_relaxed);
        g_unit_gvalid.store(true, std::memory_order_relaxed);
    } else {
        g_unit_gvalid.store(false, std::memory_order_relaxed);
    }
