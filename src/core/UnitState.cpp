#include "core/UnitState.hpp"

#include "BlamDrive.hpp"          // the author's grenade atomics
#include "Config.hpp"
#include "MotionAimControl.hpp"   // g_stick_mode_active
#include "core/Services.hpp"
#include "core/host/BlamDriveState.hpp"
#include "features/hooks/UnitStateHooks.hpp"
#include "uevr/API.hpp"

#include <Windows.h>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <intrin.h>

using namespace uevr;

namespace halo {

std::atomic<float> g_unit_px{0.0f}, g_unit_py{0.0f}, g_unit_pz{0.0f};
std::atomic<bool>  g_unit_pvalid{false};
std::atomic<float> g_unit_fx{1.0f}, g_unit_fy{0.0f};
std::atomic<float> g_veh_fx{1.0f}, g_veh_fy{0.0f};
std::atomic<bool>  g_veh_fvalid{false};
std::atomic<float> g_vehpx{0.0f}, g_vehpy{0.0f}, g_vehpz{0.0f};
std::atomic<bool> g_unit_mounted{false};
std::atomic<uint32_t> g_seat_pub_seq{0}, g_seat_pub_calls{0}, g_seat_norec{0}, g_seat_reresolve{0},
                      g_seat_direct_reads{0};
std::atomic<uintptr_t> g_seat_obj{0}, g_seat_vobj{0};
std::atomic<uint32_t>  g_seat_vdat{0xFFFFFFFFu};

// The unit and object resolves below read BlamDrive.cpp's guarded pointer read and the sim module's
// TLS index through the host bridge, bound under their own names.
// UNIT OBJECT DUMP (Config::blam_unit_dump). The record's +0x80 is our unit datum; the game's own
// walk (seen at dll+0x279BEA: tls+0x20 -> [..] -> +0x50 -> entry idx*24 -> +0x10) yields the object.
// Every N calls, log the delivered left stick beside every non-zero float in [-1.5,1.5] of the
// object's first blam_unit_dump_len bytes -- the biped's throttle is whatever tracks the stick.
uintptr_t resolve_unit_object(uintptr_t rec_base, uint32_t* out_idx) {
    const auto read_ptr = host::g_blamdrive_state.read_ptr;
    const uint32_t& g_tls_index = *host::g_blamdrive_state.tls_index;
    const uintptr_t tls_array = (uintptr_t)__readgsqword(0x58);
    uintptr_t block = 0, ctx = 0, ctx2 = 0, table = 0, obj = 0;
    if (tls_array == 0) return 0;
    if (!read_ptr(tls_array + (uintptr_t)g_tls_index * 8, &block) || block == 0) return 0;
    if (IsBadReadPtr((const void*)(rec_base + 0x80), 4)) return 0;
    const uint32_t datum = *(const uint32_t*)(rec_base + 0x80);
    if (datum == 0xFFFFFFFFu) return 0;
    const uint32_t idx = datum & 0xFFFFu;
    if (out_idx) *out_idx = datum;
    if (!read_ptr(block + 0x20, &ctx) || ctx == 0) return 0;
    (void)ctx2;
    if (!read_ptr(ctx + 0x50, &table) || table == 0) return 0;
    if (!read_ptr(table + (uintptr_t)idx * 24 + 0x10, &obj) || obj == 0) return 0;
    return obj;
}

// Resolve ANY object datum through the same table walk. SIM THREAD ONLY (gs:[0x58]).
// Found 2026-08-20: the biped's +0x0C holds its parent object's datum while mounted (the
// Warthog's unit) and 0xFFFFFFFF on foot -- this is the door into vehicle state.
uintptr_t resolve_object_by_datum(uint32_t datum) {
    const auto read_ptr = host::g_blamdrive_state.read_ptr;
    const uint32_t& g_tls_index = *host::g_blamdrive_state.tls_index;
    const uintptr_t tls_array = (uintptr_t)__readgsqword(0x58);
    uintptr_t block = 0, ctx = 0, table = 0, obj = 0;
    if (tls_array == 0 || datum == 0xFFFFFFFFu) return 0;
    if (!read_ptr(tls_array + (uintptr_t)g_tls_index * 8, &block) || block == 0) return 0;
    if (!read_ptr(block + 0x20, &ctx) || ctx == 0) return 0;
    if (!read_ptr(ctx + 0x50, &table) || table == 0) return 0;
    if (!read_ptr(table + (uintptr_t)(datum & 0xFFFFu) * 24 + 0x10, &obj) || obj == 0) return 0;
    return obj;
}


// ---- UNIT STATE FOR THE HOLSTERS, published per sim call. SIM THREAD ONLY (the resolve walks
// gs:[0x58]). Mounted: the biped's +0x0C parent datum != 0xFFFFFFFF (a vehicle seat or turret) --
// gates the holster button steal, which must never eat the buttons a seat needs. Grenades: type
// at +0x380, frag count +0x382, plasma count +0x383 -- the game auto-switches type when one runs
// out, which is exactly what a plugin-side belief would lose; reading the object is the truth.
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
    if (service_active(SVC_SEAT) && g_cfg.veh_facing != 0 && !IsBadReadPtr((const void*)(obj + 0x0C), 4)) {
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


void publish_unit_state(uintptr_t rec_base) {
    uint32_t datum = 0;
    const uintptr_t obj = resolve_unit_object(rec_base, &datum);
    {
        static uint32_t s_last_datum = 0xDEADBEEFu;
        if ((g_cfg.veh_log || g_cfg.holster_log) && datum != s_last_datum && !IsBadReadPtr((const void*)(rec_base + 0x70), 0x20)) {
            s_last_datum = datum;
            const uint32_t* rw = (const uint32_t*)(rec_base + 0x70);
            // The datum slot read zero all session while play was live, so either the offset or
            // the RECORD is wrong for this resolve path. Probe the same slot in the neighbouring
            // records: the one holding a plausible datum identifies the live slot directly.
            uint32_t nb[4] = {0, 0, 0, 0};
            for (int k = 0; k < 4; ++k) {
                const uintptr_t r2 = rec_base + (uintptr_t)k * 0x198;
                if (!IsBadReadPtr((const void*)(r2 + 0x80), 4)) nb[k] = *(const uint32_t*)(r2 + 0x80);
            }
            API::get()->log_info("[Halo-CampE-UEVR] UNITREC: rec 0x%llX rec+0x70..0x8C=[%08X %08X %08X %08X %08X %08X %08X %08X] -> datum 0x%08X | +0x80 of records 0..3 = [%08X %08X %08X %08X]",
                                 (unsigned long long)rec_base,
                                 rw[0], rw[1], rw[2], rw[3], rw[4], rw[5], rw[6], rw[7], datum,
                                 nb[0], nb[1], nb[2], nb[3]);
        }
    }
    // Datum 0x00000000 is NOT a unit (measured: it resolves table slot 0, a garbage object whose
    // +0x0C read as "mounted" -- which disabled the button steal -- and whose counts read zero,
    // refusing every pouch). Only a plausible datum publishes; anything else stands the state
    // down so the consumers run on their safe defaults (not mounted, counts unknown).
    if (obj == 0 || datum == 0 || (datum & 0xFFFFu) == 0) {
        g_unit_gvalid.store(false, std::memory_order_relaxed);
        g_unit_mounted.store(false, std::memory_order_relaxed);
        g_unit_pvalid.store(false, std::memory_order_relaxed);
        g_veh_fvalid.store(false, std::memory_order_relaxed);
        return;
    }
    g_unit_mounted.store(!IsBadReadPtr((const void*)(obj + 0x0C), 4)
                         && *(const uint32_t*)(obj + 0x0C) != 0xFFFFFFFFu,
                         std::memory_order_relaxed);
    // EVIDENCE, once per resolved object: the pouches read "frag 0 plasma 0" on a unit that
    // demonstrably had grenades, so either this is the wrong object or the offsets do not hold
    // on this path. Print the datum, the object, and the raw bytes -- the next session decides.
    {
        static uintptr_t s_said_obj = 0;
        if ((g_cfg.veh_log || g_cfg.holster_log) && obj != s_said_obj && !IsBadReadPtr((const void*)(obj + 0x380), 8)) {
            s_said_obj = obj;
            const uint8_t* u8 = (const uint8_t*)obj;
            const uint32_t* rw = (const uint32_t*)(rec_base + 0x70);
            API::get()->log_info("[Halo-CampE-UEVR] UNITSTATE: rec 0x%llX rec+0x70..0x8C=[%08X %08X %08X %08X %08X %08X %08X %08X] datum 0x%08X obj 0x%llX bytes@0x380=[%02X %02X %02X %02X %02X %02X %02X %02X]",
                                 (unsigned long long)rec_base,
                                 rw[0], rw[1], rw[2], rw[3], rw[4], rw[5], rw[6], rw[7],
                                 datum, (unsigned long long)obj,
                                 u8[0x380], u8[0x381], u8[0x382], u8[0x383],
                                 u8[0x384], u8[0x385], u8[0x386], u8[0x387]);
        }
    }
    features_sim_unit_state_grenades(obj);
    features_sim_unit_state_radar(obj);
    features_sim_unit_state_after_radar(obj);

    publish_seat_state(obj);

    // FACING at +0x50 (unit vector, Blam frame -- seen as (0.028, 1.000, ~0) in the seated
    // dump). UE direction is (fx, -fy, fz), so UE yaw = atan2(-fy, fx): the same Y negation the
    // aim reconstruction uses.
    if (!IsBadReadPtr((const void*)(obj + 0x50), 8)) {
        const float* fv = (const float*)(obj + 0x50);
        g_unit_fx.store(fv[0], std::memory_order_relaxed);
        g_unit_fy.store(fv[1], std::memory_order_relaxed);
    }

    features_sim_unit_state_end(obj);
}

void unit_state_stick_mode_publish(bool off_thread) {
    std::atomic<uintptr_t>& g_ctl_rec = *host::g_blamdrive_state.ctl_rec;
    const uintptr_t OFF_CTL_YAW = host::g_blamdrive_state.off_ctl_yaw;
    const auto resolve_control_record = host::g_blamdrive_state.resolve_control_record;
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
}

void unit_state_record_ready(uintptr_t rec, bool off_thread) {
    const uintptr_t OFF_CTL_YAW = host::g_blamdrive_state.off_ctl_yaw;
    // Holster inputs (mounted, grenade type/counts), published beside the write. Sim thread only:
    // the unit resolve walks gs:[0x58], which reads zero from any other thread -- the off-thread
    // TEB path skips it and the atomics simply hold.
    //
    // REBASED: rec points AT the yaw field (resolve returns record + OFF_CTL_YAW; fp[0]/fp[1]
    // below write through it directly), but the unit resolve walks offsets from the RECORD START.
    // Passing rec unrebased shifted every read by 0x94 -- the datum slot read from the middle of
    // the record and came back zero for a whole session, which is what stranded the grenade
    // pouches on the belief fallback and let the type belief invert against the game.
    if (!off_thread) publish_unit_state(rec - OFF_CTL_YAW);
}

} // namespace halo
