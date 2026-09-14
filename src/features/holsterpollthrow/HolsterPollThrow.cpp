#include "features/holsterpollthrow/HolsterPollThrow.hpp"

#include "BlamDrive.hpp"          // the author's grenade atomics
#include "Config.hpp"
#include "Holster.hpp"            // HolsterSlot, g_holster_throw_until, holster_throw_press_active
#include "Markers.hpp"            // holster_room_to_world
#include "Math.hpp"               // Vec3, wrap180, RAD2DEG, clampf
#include "MotionAimControl.hpp"   // g_turn_offset, the gesture aim hold
#include "UeObject.hpp"           // TrackedObject
#include "core/UnitState.hpp"     // resolve_object_by_datum
#include "core/host/BlamAimState.hpp"
#include "core/host/HolsterState.hpp"
#include "uevr/API.hpp"

#include <Windows.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using uevr::API;

namespace halo {

namespace {

// ---- POLL-RATE THROW RELEASE (doctrine in Config.hpp). The tick's standing verdict, consumed by
// the XInput hook. Mask 0 = disarmed (no grenade, hand in a pouch = put-back territory, feature
// off). The hold angles are the tick's peak-direction math, republished every tick so the hook
// only ever copies numbers.
std::atomic<unsigned short> g_pollthrow_mask{0};
std::atomic<float> g_pollthrow_hy{0.0f}, g_pollthrow_hp{0.0f};
std::atomic<bool>  g_pollthrow_hold{false};
std::atomic<bool>  g_pollthrow_fired{false};

// The carrier hand's position in BLAM units, for the grenade-at-hand spawn-origin experiment
// (grenhand, doctrine in Config.hpp). Published per tick while a grenade is armed; the last value
// deliberately survives the release, because the spawn lands ~42 ms after it.
std::atomic<float> g_hand_blam_x{0.0f}, g_hand_blam_y{0.0f}, g_hand_blam_z{0.0f};
std::atomic<bool>  g_hand_blam_valid{false};
// The swing's peak direction in BLAM units (normalized), for the instant-release velocity.
std::atomic<float> g_throw_blam_x{0.0f}, g_throw_blam_y{1.0f}, g_throw_blam_z{0.0f};
std::atomic<bool>  g_throw_blam_valid{false};

uint32_t s_mk_fails = 0;   // empty surveys in a row: the sweep backs off (120 ticks -> 1200)

}  // namespace

// ================================================================================================
// THE UNIT OBJECT SIDE (sim thread): the grenade track handoff, the throw windup dump, the live counts
// and the instant-release experiments.
// ================================================================================================

// ---- THROW WINDUP DUMP (throwdump, doctrine in Config.hpp). SIM THREAD. Statics only, no
// allocation; the cost while idle is one memcmp-sized pass over 0x600 bytes per sim call, and the
// log lines are capped per window. The mask is learned, not assumed: anything that churns while
// nothing is being thrown is by definition not the throw.
inline long long now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// GRENTRACK handoff (see BlamDrive.hpp): written by the dev spawn hook, sampled below.
std::atomic<uintptr_t> g_grentrack_obj{0};
std::atomic<long long> g_grentrack_at_ms{0};
std::atomic<uintptr_t> g_greninst_obj{0};
std::atomic<long long> g_greninst_at_ms{0};
std::atomic<float>     g_greninst_vx{0.0f}, g_greninst_vy{0.0f}, g_greninst_vz{0.0f};

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

// ================================================================================================
// THE HOLSTER SIDE: the XInput-cadence throw and the exports.
// ================================================================================================

// XINPUT HOOK CADENCE -- clocks and atomics only, per the rule on that callback (no poses, no
// reflection, no logging, no haptics; all of that is the tick's, before or after). Fires the
// synthetic throw press on the carrier grip's falling edge. Called with the RAW buttons before
// any remapping, and BEFORE the press mask is composed into the same poll -- so the throw the
// player just released goes out in the very report that shows the grip open.
void holster_note_buttons(unsigned short buttons) {
    // Holster.cpp's clock, through the bridge.
    const auto now_ticks = host::g_holster_state.now_ticks;
    const auto ms_to_ticks = host::g_holster_state.ms_to_ticks;
    static unsigned short s_prev = 0;
    const unsigned short prev = s_prev;
    s_prev = buttons;
    const unsigned short mask = g_pollthrow_mask.load(std::memory_order_relaxed);
    if (mask == 0) return;
    if ((prev & mask) == 0 || (buttons & mask) != 0) return;   // fire on held -> released only
    g_pollthrow_mask.store(0, std::memory_order_relaxed);      // one shot; the tick re-arms
    g_holster_throw_until.store(now_ticks() + ms_to_ticks(g_cfg.holster_press_ms),
                                std::memory_order_relaxed);
    if (g_pollthrow_hold.load(std::memory_order_relaxed) && g_cfg.holster_aim_hold_ms > 0) {
        g_melee_aim_ctrl_yaw.store(g_pollthrow_hy.load(std::memory_order_relaxed), std::memory_order_relaxed);
        g_melee_aim_ctrl_pitch.store(g_pollthrow_hp.load(std::memory_order_relaxed), std::memory_order_relaxed);
        g_melee_aim_hold_until.store(now_ticks() + ms_to_ticks(g_cfg.holster_aim_hold_ms),
                                     std::memory_order_relaxed);
    }
    g_pollthrow_fired.store(true, std::memory_order_relaxed);
}

// The carrier hand's last published Blam-unit position (grenhand). Safe on any thread.
bool holster_hand_blam(float* x, float* y, float* z) {
    if (!g_hand_blam_valid.load(std::memory_order_relaxed)) return false;
    *x = g_hand_blam_x.load(std::memory_order_relaxed);
    *y = g_hand_blam_y.load(std::memory_order_relaxed);
    *z = g_hand_blam_z.load(std::memory_order_relaxed);
    return true;
}
// The resolved grenade meshes, for the wrist radar's blips (frag = human, plasma = covenant --
// the factions' own ordnance as their marker art). Game thread; may be null until resolved.
uevr::API::UObject* holster_mesh_frag()   { TrackedObject& s_mesh_frag = *host::g_holster_state.mesh_frag; return s_mesh_frag.get(); }
uevr::API::UObject* holster_mesh_plasma() { TrackedObject& s_mesh_plasma = *host::g_holster_state.mesh_plasma; return s_mesh_plasma.get(); }

// The swing's peak direction in Blam units, normalized (greninstant). Safe on any thread.
bool holster_throw_blam_dir(float* x, float* y, float* z) {
    if (!g_throw_blam_valid.load(std::memory_order_relaxed)) return false;
    *x = g_throw_blam_x.load(std::memory_order_relaxed);
    *y = g_throw_blam_y.load(std::memory_order_relaxed);
    *z = g_throw_blam_z.load(std::memory_order_relaxed);
    return true;
}

namespace {

void holsterpollthrow_holster_reset() {
    // Disarm the poll-rate throw: a stale mask would let a menu or a seat's grip release lob a
    // grenade the instant play resumes.
    g_pollthrow_mask.store(0, std::memory_order_relaxed);
    g_pollthrow_fired.store(false, std::memory_order_relaxed);
}

unsigned holsterpollthrow_mesh_sweep_period() {
    return (s_mk_fails < 5 || !g_cfg.holster_poll_throw) ? 120u : 1200u;
}

void holsterpollthrow_mesh_swept(const void* mf) {
        if (mf == nullptr) ++s_mk_fails; else s_mk_fails = 0;   // a level with no grenade mesh: one sweep per ~40 s, not per 4 s
}

void holsterpollthrow_before_release(HolsterSlot zone_g, HolsterSlot zone_p,
                                     const Vec3& pos, const Vec3& gpos, const Vec3& hpos) {
    // Holster.cpp's own state and helpers, through the bridge: the same objects under the same names.
    bool& s_grenade_armed = *host::g_holster_state.grenade_armed;
    const bool& s_carry_off = *host::g_holster_state.carry_off;
    const bool& s_unarmed = *host::g_holster_state.unarmed;
    int& s_unhide_ticks = *host::g_holster_state.unhide_ticks;
    long long& s_last_action = *host::g_holster_state.last_action;
    const Vec3& s_peak_velw = *host::g_holster_state.peak_velw;
    const Vec3& s_gpeak_velw = *host::g_holster_state.gpeak_velw;
    const auto now_ticks = host::g_holster_state.now_ticks;
    const auto aim_is_right = host::g_holster_state.aim_is_right;
    const auto off_is_right = host::g_holster_state.off_is_right;
    const auto haptic_on = host::g_holster_state.haptic_on;
    const auto set_weapon_hidden = host::g_holster_state.set_weapon_hidden;

    // ---- POLL-PATH RECONCILIATION. The hook threw between ticks: press and aim hold are already
    // out the door; this is everything else the release branch does. Runs BEFORE the tick's own
    // release edge below -- s_grenade_armed drops here, so the same grenade cannot throw twice.
    if (g_pollthrow_fired.exchange(false, std::memory_order_relaxed)) {
        if (s_grenade_armed) {
            const bool coff = s_carry_off;
            s_grenade_armed = false;
            s_last_action = now_ticks();
            if (!coff && !s_unarmed) { set_weapon_hidden(false); s_unhide_ticks = 30; }
            haptic_on(coff ? off_is_right() : aim_is_right(), 0.10f, 1.0f);
            if (g_cfg.holster_log)
                API::get()->log_info("[Halo-CampE-UEVR] HOLSTER THROW (poll-rate release, %s hand)",
                                     coff ? "off" : "aim");
        }
    }

    // ---- POLL-PATH PUBLISH: the standing verdict the hook acts on. Armed grenade, carrier hand
    // OUTSIDE every pouch (in a pouch, a release is a put-back and stays the tick's call), and the
    // aim-hold direction from the current peak -- all at most one tick old at fire time.
    {
        unsigned short pmask = 0;
        if (g_cfg.holster_poll_throw && s_grenade_armed) {
            const bool coff = s_carry_off;
            const HolsterSlot czone = coff ? zone_g : zone_p;
            if (czone == HolsterSlot::None) {
                pmask = (unsigned short)((coff ? off_is_right() : aim_is_right())
                                         ? g_cfg.grip_mask_r : g_cfg.grip_mask_l);
                const Vec3& cpeak = coff ? s_gpeak_velw : s_peak_velw;
                const float vlen = std::sqrt(cpeak.x * cpeak.x + cpeak.y * cpeak.y + cpeak.z * cpeak.z);
                if (vlen > 0.2f) {
                    g_pollthrow_hy.store(wrap180(std::atan2(cpeak.x, -cpeak.z) * RAD2DEG
                                                 + g_cfg.aim_turn * g_turn_offset.load(std::memory_order_relaxed)),
                                         std::memory_order_relaxed);
                    g_pollthrow_hp.store(std::asin(std::fmax(-1.0f, std::fmin(1.0f, cpeak.y / vlen))) * RAD2DEG,
                                         std::memory_order_relaxed);
                    g_pollthrow_hold.store(true, std::memory_order_relaxed);
                } else {
                    g_pollthrow_hold.store(false, std::memory_order_relaxed);
                }
            }
        }
        g_pollthrow_mask.store(pmask, std::memory_order_relaxed);
    }

    // Carrier hand in Blam units, for the grenhand spawn-origin experiment. UE world / 304.8 with
    // Y negated -- the same fit that placed the vehicle camera (BlamDrive, unit+0x20 vs camera).
    if (s_grenade_armed) {
        const Vec3 cpos = s_carry_off ? gpos : pos;
        const Vec3 hw = holster_room_to_world(cpos, hpos);
        g_hand_blam_x.store(hw.x / 304.8f, std::memory_order_relaxed);
        g_hand_blam_y.store(-hw.y / 304.8f, std::memory_order_relaxed);
        g_hand_blam_z.store(hw.z / 304.8f, std::memory_order_relaxed);
        g_hand_blam_valid.store(true, std::memory_order_relaxed);
        // The peak swing direction, room frame -> Blam frame, by differencing room_to_world at
        // two points (the translation cancels, leaving exactly the rotation + swizzle + scale
        // that frame applies -- no second frame-math implementation to drift out of sync).
        const Vec3& cpk = s_carry_off ? s_gpeak_velw : s_peak_velw;
        const float pklen = std::sqrt(cpk.x * cpk.x + cpk.y * cpk.y + cpk.z * cpk.z);
        if (pklen > 0.2f) {
            const Vec3 pw = holster_room_to_world(Vec3{cpos.x + cpk.x, cpos.y + cpk.y, cpos.z + cpk.z}, hpos);
            float bx = (pw.x - hw.x), by = -(pw.y - hw.y), bz = (pw.z - hw.z);
            const float bl = std::sqrt(bx * bx + by * by + bz * bz);
            if (bl > 1e-4f) {
                g_throw_blam_x.store(bx / bl, std::memory_order_relaxed);
                g_throw_blam_y.store(by / bl, std::memory_order_relaxed);
                g_throw_blam_z.store(bz / bl, std::memory_order_relaxed);
                g_throw_blam_valid.store(true, std::memory_order_relaxed);
            }
        }
    }
}

void holsterpollthrow_unit_state_grenades(uintptr_t obj) {
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

    if (g_cfg.throw_dump != 0) throw_dump_probe(obj);
}

void holsterpollthrow_unit_state_after_radar(uintptr_t obj) {
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
}

void holsterpollthrow_game_tick_after_blam_aim() {
    blam_spawnlog_tick();   // spawn hook alone (throwdump) -- no aim ownership change
}

#if HALO_VR_DEV

void holsterpollthrow_blam_create_before(uintptr_t params) {
    // BlamAim.cpp's spawn origin offset, through the bridge.
    const uintptr_t P_VEC1 = host::g_blamaim_state.p_vec1;

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
}

void holsterpollthrow_blam_create_after(uintptr_t params, uintptr_t cret) {
    (void)params;
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
}

#endif  // HALO_VR_DEV

}  // namespace

#if HALO_VR_DEV

// THE SPAWN HOOK ALONE, for the grenade-windup capture. blamaim=1 proved unusable for this: it
// takes ownership of the aim-write function, which (a) replaces months of shipping aim with the
// investigation-era law -- "aim completely off" in the headset -- and (b) stands down BlamDrive's
// hook, killing publish_unit_state and with it the THROWDUMP probe. One session produced spawn
// rows with frozen aim and no press marks: worthless twice over. This installs ONLY the
// create_projectile hook, driven by the same `throwdump` key as the probe, so press timeline and
// spawn timestamps come from one session with normal aim.
void blam_spawnlog_tick() {
    // BlamAim.cpp's own hook state, through the bridge: the same objects under the same names.
    int& g_create_hook_id = *host::g_blamaim_state.create_hook_id;
    const int& g_hook_id = *host::g_blamaim_state.hook_id;
    auto& g_orig_create = *host::g_blamaim_state.orig_create;
    uintptr_t& g_sim_base = *host::g_blamaim_state.sim_base;
    const uintptr_t RVA_CREATE_PROJECTILE = host::g_blamaim_state.rva_create_projectile;
    const auto& CREATE_PROJECTILE_PROLOGUE = *host::g_blamaim_state.create_projectile_prologue;
    auto& hooked_create_projectile = *host::g_blamaim_state.hooked_create_projectile;

    if (g_cfg.blam_aim != 0) return;   // blamaim owns both hooks; stand down to it entirely
    const bool want = g_cfg.throw_dump != 0;
    if (!want) {
        if (g_create_hook_id >= 0 && g_hook_id < 0) {   // ours, not blamaim's
            API::get()->param()->functions->unregister_inline_hook(g_create_hook_id);
            g_create_hook_id = -1;
            g_orig_create = nullptr;
            API::get()->log_info("[Halo-CampE-UEVR] BLAMSPAWN: removed (throwdump off)");
        }
        return;
    }
    if (g_create_hook_id >= 0) return;
    HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
    if (sim == nullptr) return;
    g_sim_base = (uintptr_t)sim;
    void* ctarget = (void*)(g_sim_base + RVA_CREATE_PROJECTILE);
    static bool s_refused = false;   // one refusal line, not one per tick
    if (IsBadReadPtr(ctarget, sizeof(CREATE_PROJECTILE_PROLOGUE)) ||
        memcmp(ctarget, CREATE_PROJECTILE_PROLOGUE, sizeof(CREATE_PROJECTILE_PROLOGUE)) != 0) {
        if (!s_refused) {
            s_refused = true;
            API::get()->log_info("[Halo-CampE-UEVR] BLAMSPAWN: prologue mismatch at dll+0x%llX -- "
                                 "the game moved; spawn hook stays off",
                                 (unsigned long long)RVA_CREATE_PROJECTILE);
        }
        return;
    }
    const int cid = API::get()->param()->functions->register_inline_hook(
        ctarget, (void*)&hooked_create_projectile, (void**)&g_orig_create);
    if (cid < 0 || g_orig_create == nullptr) {
        if (!s_refused) {
            s_refused = true;
            API::get()->log_info("[Halo-CampE-UEVR] BLAMSPAWN: hook FAILED (id=%d)", cid);
        }
        return;
    }
    g_create_hook_id = cid;
    API::get()->log_info("[Halo-CampE-UEVR] BLAMSPAWN: installed standalone on 0x%llX (dll+0x%llX) id=%d",
                         (unsigned long long)ctarget, (unsigned long long)RVA_CREATE_PROJECTILE, cid);
}

#endif  // HALO_VR_DEV

bool holsterpollthrow_parse_key(const char* key, const char* val, double v) {
    if (_stricmp(key, "holsterpollthrow") == 0) { g_cfg.holster_poll_throw = (v != 0.0); return true; }
    if (_stricmp(key, "gripmaskl")      == 0) { g_cfg.grip_mask_l = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "gripmaskr")      == 0) { g_cfg.grip_mask_r = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "throwdump")   == 0) { g_cfg.throw_dump     = (int)v; return true; }
    if (_stricmp(key, "grenhand")    == 0) { g_cfg.gren_hand_spawn = (int)v; return true; }
    if (_stricmp(key, "greninstant") == 0) { g_cfg.gren_instant = (int)v; return true; }
    if (_stricmp(key, "grenbackdate") == 0) { g_cfg.gren_backdate = (int)clampf((float)v, 1.0f, 60.0f); return true; }
    if (_stricmp(key, "grenspeed")   == 0) { g_cfg.gren_speed = clampf((float)v, 1.0f, 30.0f); return true; }
    return false;
}

constinit const FeatureHooks kHolsterPollThrowHooks{
    .key                        = "holsterpollthrow",
    .parse_key                  = &holsterpollthrow_parse_key,
    .xinput_note_buttons        = &holster_note_buttons,
    .game_tick_after_blam_aim   = &holsterpollthrow_game_tick_after_blam_aim,
    .holster_reset              = &holsterpollthrow_holster_reset,
    .holster_mesh_sweep_period  = &holsterpollthrow_mesh_sweep_period,
    .holster_mesh_swept         = &holsterpollthrow_mesh_swept,
    .holster_before_release     = &holsterpollthrow_before_release,
    .sim_unit_state_grenades    = &holsterpollthrow_unit_state_grenades,
    .sim_unit_state_after_radar = &holsterpollthrow_unit_state_after_radar,
#if HALO_VR_DEV
    .blam_create_before         = &holsterpollthrow_blam_create_before,
    .blam_create_after          = &holsterpollthrow_blam_create_after,
#endif
};

} // namespace halo
