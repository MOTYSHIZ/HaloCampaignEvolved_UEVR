// vehcam (fork feature, Experimental): the seat half of the unit publish, its stick-mode variant and vehseatdirect.
// Textual fragment, included by BlamDrive.cpp at file scope, before publish_unit_state(). Moved verbatim; not compiled on its own.
// THE SEAT HALF of the unit publish: the rider's world position (+0x20) and, through the parent
// datum (+0x0C), the vehicle's facing (+0x1D4) and position. Split out of publish_unit_state so it
// also runs while stick mode holds the aim write off -- a seat IS stick mode, and the seat camera,
// the parked-only seat learn (speed), the heading and the seated view all read these. SIM THREAD.
static void publish_seat_state(uintptr_t obj) {
    // WORLD POSITION at +0x20 (3 floats, Blam world units). Measured by logging the rendered
    // camera beside every unit float and fitting: over 972 samples the large-motion delta ratios
    // are +308/-306/+332 against cam x/y/z, i.e. the 304.8 cm world unit with Blam's Y negation,
    // and the constant residual is the eye height (~78 cm). This is what puts the VR camera in a
    // vehicle seat.
    g_seat_pub_calls.fetch_add(1, std::memory_order_relaxed);
    if (!IsBadReadPtr((const void*)(obj + 0x20), 12)) {
        const float* q = (const float*)(obj + 0x20);
        g_unit_px.store(q[0], std::memory_order_relaxed);
        g_unit_py.store(q[1], std::memory_order_relaxed);
        g_unit_pz.store(q[2], std::memory_order_relaxed);
        g_unit_pvalid.store(true, std::memory_order_relaxed);
        g_seat_obj.store(obj, std::memory_order_relaxed);
        g_seat_pub_seq.fetch_add(1, std::memory_order_relaxed);
    } else g_unit_pvalid.store(false, std::memory_order_relaxed);

    // THE VEHICLE'S FACING (vehfacing). Measured by spinning the hog: the pairs at
    // +0x1D4/+0x1E0/+0x1EC are unit vectors that swept 3226 deg with it, so they carry the real
    // orientation -- unlike +0x50, which is parent-local and reads a constant (1,0) from a seat.
    //
    // The pointer is resolved ONCE PER MOUNT and cached. The first version walked the object
    // table on every publish (~325/sec on the sim thread) and shook the whole picture, on foot
    // included -- that is the bug this cache exists to avoid.
    if (g_cfg.veh_facing != 0 && !IsBadReadPtr((const void*)(obj + 0x0C), 4)) {
        static uintptr_t s_vobj = 0;
        static uint32_t  s_vdat = 0xFFFFFFFFu;
        static uint32_t  s_vretry = 0;
        const uint32_t pdat = *(const uint32_t*)(obj + 0x0C);
        if (pdat != s_vdat) {          // mount, dismount or vehicle swap
            s_vdat = pdat;
            s_vretry = 0;
            s_vobj = (pdat != 0xFFFFFFFFu) ? resolve_object_by_datum(pdat) : 0;
            if (s_vobj != 0 && IsBadReadPtr((const void*)(s_vobj + 0x1F4), 4)) s_vobj = 0;
        } else if (s_vobj == 0 && pdat != 0xFFFFFFFFu && (++s_vretry % 325u) == 0u) {
            // RETRY WHILE MOUNTED-AND-UNRESOLVED (~1 s cadence at this hook's rate). The resolve
            // used to run once, at the mount edge -- mid entry animation, a transient window --
            // and a failure there latched for the WHOLE ride: facing stayed invalid, the camera
            // fell back to the travel-heading hemisphere guess, and the rare "camera starts
            // backwards, dismount and remount fixes it" is exactly that fallback seeded wrong
            // and self-latched. A resolve that can fail transiently must retry. Logged both ways
            // so a backwards ride names its own cause.
            s_vobj = resolve_object_by_datum(pdat);
            if (s_vobj != 0 && IsBadReadPtr((const void*)(s_vobj + 0x1F4), 4)) s_vobj = 0;
            if (s_vretry == 325u || s_vobj != 0)
                API::get()->log_info("[Halo-CampE-UEVR] VEHFACING: %s (retry %u)",
                                     s_vobj != 0 ? "resolved on retry -- facing live"
                                                 : "vehicle object unresolved -- camera is on the travel-heading fallback",
                                     s_vretry / 325u);
        }
        g_seat_vobj.store(s_vobj, std::memory_order_relaxed);
        g_seat_vdat.store(pdat, std::memory_order_relaxed);
        if (s_vobj != 0) {
            const float* fv = (const float*)(s_vobj + (uintptr_t)g_cfg.veh_facing_off);
            g_veh_fx.store(fv[0], std::memory_order_relaxed);
            g_veh_fy.store(fv[1], std::memory_order_relaxed);
            g_veh_fvalid.store(true, std::memory_order_relaxed);
            // The VEHICLE'S OWN POSITION, same +0x20 layout as the biped's. The seat camera
            // wants THIS when vehcamsrc=1: measured, driving with the camera on the biped
            // position left the world smooth and the hog juddering -- the camera was moving on a
            // different curve from the thing it is supposed to be bolted to.
            const float* vp = (const float*)(s_vobj + 0x20);
            g_vehpx.store(vp[0], std::memory_order_relaxed);
            g_vehpy.store(vp[1], std::memory_order_relaxed);
            g_vehpz.store(vp[2], std::memory_order_relaxed);
        } else g_veh_fvalid.store(false, std::memory_order_relaxed);
    } else g_veh_fvalid.store(false, std::memory_order_relaxed);
}

// Stick-mode publish (vehicle seats, cutscenes, death): the mounted flag and the seat half only.
// Everything else publish_unit_state does (grenade state and writes, throw probes, radar scan,
// roomscale throttle) stays behind the stick-mode hold exactly as before.
static void publish_seated_unit_state(uintptr_t rec_base) {
    uint32_t datum = 0;
    const uintptr_t obj = resolve_unit_object(rec_base, &datum);
    if (obj == 0 || datum == 0 || (datum & 0xFFFFu) == 0) {
        g_unit_mounted.store(false, std::memory_order_relaxed);
        g_unit_pvalid.store(false, std::memory_order_relaxed);
        g_veh_fvalid.store(false, std::memory_order_relaxed);
        return;
    }
    g_unit_mounted.store(!IsBadReadPtr((const void*)(obj + 0x0C), 4)
                         && *(const uint32_t*)(obj + 0x0C) != 0xFFFFFFFFu,
                         std::memory_order_relaxed);
    publish_seat_state(obj);
}

// vehseatdirect (approach B): the seat without the sim hook. The rider and vehicle objects are
// pool entries that stay put for the life of the ride, so once the sim publish has named them the
// fields are plain memory reads from any thread -- no gs:[0x58] walk, no control record, no hook
// cadence. Only while stick mode holds the normal publish off; on foot the sim publish owns these.
// The vehicle half is trusted only while the rider's parent datum still names the cached vehicle.
void seat_direct_refresh() {
    if (g_cfg.veh_seat_direct == 0) return;
    if (!g_stick_mode_active.load(std::memory_order_relaxed)) return;
    const uintptr_t obj = g_seat_obj.load(std::memory_order_relaxed);
    if (obj == 0 || IsBadReadPtr((const void*)obj, 0x2C)) return;
    const uint32_t pdat = *(const uint32_t*)(obj + 0x0C);
    const float* q = (const float*)(obj + 0x20);
    if (!std::isfinite(q[0]) || !std::isfinite(q[1]) || !std::isfinite(q[2])) return;
    g_unit_mounted.store(pdat != 0xFFFFFFFFu, std::memory_order_relaxed);
    g_unit_px.store(q[0], std::memory_order_relaxed);
    g_unit_py.store(q[1], std::memory_order_relaxed);
    g_unit_pz.store(q[2], std::memory_order_relaxed);
    g_unit_pvalid.store(true, std::memory_order_relaxed);
    g_seat_pub_seq.fetch_add(1, std::memory_order_relaxed);
    g_seat_direct_reads.fetch_add(1, std::memory_order_relaxed);
    const uintptr_t vobj = g_seat_vobj.load(std::memory_order_relaxed);
    if (g_cfg.veh_facing != 0 && vobj != 0 && pdat != 0xFFFFFFFFu
        && pdat == g_seat_vdat.load(std::memory_order_relaxed)
        && !IsBadReadPtr((const void*)vobj, 0x1F4)) {
        const float* fv = (const float*)(vobj + (uintptr_t)g_cfg.veh_facing_off);
        const float* vp = (const float*)(vobj + 0x20);
        if (std::isfinite(fv[0]) && std::isfinite(fv[1])) {
            g_veh_fx.store(fv[0], std::memory_order_relaxed);
            g_veh_fy.store(fv[1], std::memory_order_relaxed);
            g_veh_fvalid.store(true, std::memory_order_relaxed);
        }
        if (std::isfinite(vp[0]) && std::isfinite(vp[1]) && std::isfinite(vp[2])) {
            g_vehpx.store(vp[0], std::memory_order_relaxed);
            g_vehpy.store(vp[1], std::memory_order_relaxed);
            g_vehpz.store(vp[2], std::memory_order_relaxed);
        }
    }
}
