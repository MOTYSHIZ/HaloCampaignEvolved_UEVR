// vehcam (fork feature, Experimental): the seat publish while stick mode holds the aim write off (vehseatpub).
// Textual fragment, included by BlamDrive.cpp inside drive_angles_impl(), in the stick-mode hold. Moved verbatim; not compiled on its own.
        // The aim write holds off; the seat publish must not. The 0.2.0 tree published unit state
        // from the hook independently of this gate. With it behind the gate, mounting froze the
        // rider position (speed 0, heading never armed, the parked-only seat learn never stopping)
        // and the vehicle facing never resolved. Sim thread only (the resolve walks gs:[0x58]).
        //
        // THE RECORD MUST BE RE-FOUND HERE TOO (vehseatpub=1). blam_drive_tick() drops g_ctl_rec
        // every RERESOLVE_TICKS calls (~14 s at 46 ticks/s), and the only re-resolve lives below
        // this hold. Publishing only "if the cache is set" therefore worked until the first drop in
        // a ride and then froze the rider, the mounted flag and the speed for the rest of it -- the
        // camera learned onto the frozen point and the hog drove away from the view. The 0.2.0
        // extras publish re-resolved on its own; this does the same. Failures are rate-limited so
        // a stick-mode state with no record (a cutscene) does not scan 2600 times a second.
        if (!off_thread) {
            uintptr_t srec = g_ctl_rec.load(std::memory_order_relaxed);
            bool have = (srec != 0 && !IsBadReadPtr((const void*)(srec - OFF_CTL_YAW), 8));
            if (!have && g_cfg.veh_seat_pub != 0) {
                static uint32_t s_fail_wait = 0;
                if (s_fail_wait == 0) {
                    const char* why = "?";
                    int index = -1;
                    srec = resolve_control_record(&why, &index);
                    if (srec != 0 && !IsBadWritePtr((void*)srec, 8)) {
                        g_ctl_rec.store(srec, std::memory_order_relaxed);
                        g_seat_reresolve.fetch_add(1, std::memory_order_relaxed);
                        have = true;
                    } else {
                        s_fail_wait = 256;
                    }
                } else {
                    --s_fail_wait;
                }
            }
            if (have) publish_seated_unit_state(srec - OFF_CTL_YAW);
            else g_seat_norec.fetch_add(1, std::memory_order_relaxed);
        }
