#include "BlamDrive.hpp"

#include "Config.hpp"
#include "Math.hpp"
#include "MotionAimControl.hpp"
#include "uevr/API.hpp"

#include <Windows.h>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <intrin.h>

using namespace uevr;

namespace halo {
namespace {

// The orientation getter, dll+0x5A6AD0. Hooked purely to get onto the sim thread -- its return
// value is passed through untouched. See BlamDrive.hpp for why this function and not the setter.
constexpr uintptr_t RVA_GET_ORIENTATION = 0x5A6AD0;

// _tls_index for the sim module, read from the same instruction stream the getter uses.
constexpr uintptr_t RVA_TLS_INDEX = 0xD72730;

// The player control table: tls_block + 0xB8, record = table + index * 0x198, yaw at +0x94 and
// pitch at +0x98. Aim reconstructs as (cos p * cos y, -cos p * sin y, sin p) -- the Blam Y
// negation, which is why the yaw sign knob below exists.
constexpr uintptr_t OFF_CTL_TABLE  = 0xB8;
constexpr uintptr_t CTL_REC_STRIDE = 0x198;
constexpr uintptr_t OFF_CTL_YAW    = 0x94;

// The wrap the game itself applies after its own store: yaw is kept in [0,2pi). Named locally
// because it is a property of THIS record's encoding, not general maths. DEG2RAD comes from
// Math.hpp -- a second copy here would be one more constant to keep in step for no gain.
constexpr float CTL_YAW_WRAP = 6.28318531f;

struct Vec3f { float x, y, z; };

// Returning uintptr_t rather than void deliberately: the real function's return value is not
// modelled, and declaring void would let the compiler clobber rax on the way back out.
using GetOrientFn = uintptr_t (*)(uintptr_t handle, Vec3f* outA, Vec3f* outB);

GetOrientFn g_original = nullptr;
int         g_hook_id  = -1;
uintptr_t   g_sim_base = 0;
uint32_t    g_tls_index = 0;

// Latched once resolved. Re-resolved only when it goes bad (level load frees the table), so the
// steady state is a null check on a hot path and nothing more.
std::atomic<uintptr_t> g_ctl_rec{0};

#if HALO_VR_DEV
// Index within the control table that resolve_control_record() settled on, and the table base --
// kept only so the report below can say WHICH record we own, which is the question in co-op.
int       g_ctl_index = -1;
uintptr_t g_ctl_table = 0;

// The last values WE wrote, so the next pass can tell "the record still holds our value" from
// "something else has written it since".
float g_last_written_y = 0.0f, g_last_written_p = 0.0f;
bool  g_have_written   = false;

// BLAMCTL REPORT -- the three quantities that actually settle a reticle-vs-shot disagreement,
// sampled at ONE instant on the sim thread:
//
//   prev     what the record held on entry, i.e. after whatever last touched it
//   ours     what we wrote on the previous pass
//   wrote    what we are writing now
//   ctlrot   ControlRotation right now, which is what the reticle is drawn from
//
// Read it as two independent checks:
//
//   drift = prev - ours.  Non-zero means the record is NOT exclusively ours -- either the game
//     rewrites it between our passes, or we resolved onto another player's record. Zero means we
//     own it and our value survives.
//
//   dYaw/dPitch = (record expressed back in UE degrees) - ControlRotation.  This is THE number:
//     the reticle follows ControlRotation and the shot follows the record, so a constant non-zero
//     here IS the offset, measured rather than estimated. Zero here with a visible offset on
//     screen means the divergence is downstream of both and the reticle geometry is the suspect.
//
// Rate-limited hard: this runs ~2600 times a second on the sim thread.
void blam_ctl_report(uintptr_t rec, float prev_y, float prev_p, float wrote_y, float wrote_p,
                     float want_yaw_deg, float want_pitch_deg) {
    const int every = g_cfg.blam_ctl_log;
    if (every <= 0) return;
    static std::atomic<uint32_t> n{0};
    if ((n.fetch_add(1, std::memory_order_relaxed) % (uint32_t)every) != 0) return;

    constexpr float RAD2DEG_L = 57.2957795f;
    // The record back in UE convention: undo the wrap and the Blam yaw negation, so it is directly
    // comparable with ControlRotation instead of needing mental arithmetic at read time.
    float rec_yaw_ue = -(prev_y * RAD2DEG_L);
    while (rec_yaw_ue >  180.0f) rec_yaw_ue -= 360.0f;
    while (rec_yaw_ue < -180.0f) rec_yaw_ue += 360.0f;
    const float rec_pitch_ue = prev_p * RAD2DEG_L;

    double cr_pitch = 0.0, cr_yaw = 0.0;
    const bool have_cr = read_control_rotation_hook(&cr_pitch, &cr_yaw);

    float d_yaw = have_cr ? (rec_yaw_ue - (float)cr_yaw) : 0.0f;
    while (d_yaw >  180.0f) d_yaw -= 360.0f;
    while (d_yaw < -180.0f) d_yaw += 360.0f;
    const float d_pitch = have_cr ? (rec_pitch_ue - (float)cr_pitch) : 0.0f;

    const float drift_y = g_have_written ? (prev_y - g_last_written_y) : 0.0f;
    const float drift_p = g_have_written ? (prev_p - g_last_written_p) : 0.0f;

    API::get()->log_info(
        "[Halo-CampE-UEVR] BLAMCTL idx=%d rec=0x%llX | want=(y%.2f,p%.2f) wrote=(y%.4f,p%.4f)rad "
        "| recNow=(y%.2f,p%.2f)deg-UE ctlrot=(y%.2f,p%.2f) D=(y%.2f,p%.2f) "
        "| drift=(%.4f,%.4f)rad own=%d",
        g_ctl_index, (unsigned long long)rec,
        want_yaw_deg, want_pitch_deg, wrote_y, wrote_p,
        rec_yaw_ue, rec_pitch_ue,
        have_cr ? (float)cr_yaw : 0.0f, have_cr ? (float)cr_pitch : 0.0f,
        d_yaw, d_pitch,
        drift_y, drift_p, (int)have_cr);
}

// Every populated slot in the control table, logged ONCE per resolve. In single player only two
// records are ever populated; in co-op the table holds every player, and "which index is me" is
// the question that decides whether the lowest-index heuristic is safe. Resolve-time only, so it
// costs nothing on the hot path.
void blam_ctl_dump_table(uintptr_t table, int chosen) {
    API::get()->log_info("[Halo-CampE-UEVR] BLAMCTL TABLE at 0x%llX (chose idx %d):",
                         (unsigned long long)table, chosen);
    for (int i = 0; i < 16; ++i) {
        const uintptr_t rec = table + (uintptr_t)i * CTL_REC_STRIDE + OFF_CTL_YAW;
        if (IsBadReadPtr((const void*)rec, 8)) continue;
        const float y = ((const float*)rec)[0], p = ((const float*)rec)[1];
        if (!std::isfinite(y) || !std::isfinite(p)) continue;
        if (y == 0.0f && p == 0.0f) continue;
        API::get()->log_info("[Halo-CampE-UEVR]   idx %2d: yaw=%.4f pitch=%.4f rad  (%.1f, %.1f deg)%s",
                             i, y, p, y * 57.2957795f, p * 57.2957795f, (i == chosen) ? "  <== ours" : "");
    }
}
#endif  // HALO_VR_DEV

bool read_ptr(uintptr_t p, uintptr_t* out) {
    if (p == 0 || IsBadReadPtr((const void*)p, sizeof(uintptr_t))) return false;
    *out = *(const uintptr_t*)p;
    return true;
}

// SIM THREAD ONLY. gs:[0x58] is the TEB's TLS pointer, so off the sim thread this resolves to zero
// or garbage.
//
// The record index is found STRUCTURALLY -- the lowest-index entry holding a plausible
// (yaw in [0,2pi), pitch in [-pi/2,pi/2]) pair. Live, only records 0 and 4 are populated and the
// rest are zero, so this lands on the player without depending on any other plugin state.
//
// An earlier version matched the record against the plugin's own aim sample instead. That failed
// SILENTLY: the sample defaults to (1,0,0) and is not guaranteed fresh on this path, so every
// candidate was rejected, nothing was written, and NOTHING WAS LOGGED. Hence the reason string and
// the unconditional failure log at the call site -- a resolver that can fail must say so.
uintptr_t resolve_control_record(const char** why) {
    *why = "ok";
    const uintptr_t tls_array = (uintptr_t)__readgsqword(0x58);
    uintptr_t block = 0, table = 0;
    if (tls_array == 0) { *why = "no TLS array (wrong thread?)"; return 0; }
    if (!read_ptr(tls_array + (uintptr_t)g_tls_index * 8, &block) || block == 0) {
        *why = "TLS block null"; return 0;
    }
    if (!read_ptr(block + OFF_CTL_TABLE, &table) || table == 0) {
        *why = "control table null"; return 0;
    }

    for (int i = 0; i < 16; ++i) {
        const uintptr_t rec = table + (uintptr_t)i * CTL_REC_STRIDE + OFF_CTL_YAW;
        if (IsBadReadPtr((const void*)rec, 8)) continue;
        const float yaw = ((const float*)rec)[0], pitch = ((const float*)rec)[1];
        if (!std::isfinite(yaw) || !std::isfinite(pitch)) continue;
        if (yaw == 0.0f && pitch == 0.0f) continue;         // unpopulated slot
        if (yaw < -0.01f || yaw > 6.2932f) continue;        // not the [0,2pi) wrap
        if (pitch < -1.5808f || pitch > 1.5808f) continue;  // not a pitch
#if HALO_VR_DEV
        g_ctl_index = i;
        g_ctl_table = table;
        g_have_written = false;      // a new record: no prior write of ours to compare against
        blam_ctl_dump_table(table, i);
#endif
        return rec;
    }
    *why = "no populated record in the table";
    return 0;
}

// Minimal pass-through hook: call the original, then take the one action this file exists for.
// Deliberately nothing else on this path -- it runs ~2600 times a second on the sim thread, and a
// chatty hook on a hot path is how a diagnostic becomes a stutter.
uintptr_t hooked_get_orientation(uintptr_t handle, Vec3f* outA, Vec3f* outB) {
    const uintptr_t ret = g_original ? g_original(handle, outA, outB) : 0;
    drive_control_angles();
    return ret;
}

}  // namespace

void drive_control_angles() {
    if (g_cfg.blam_angles == 0) return;

    // HOLD OFF DURING AIM CALIBRATION (Page Down). That gesture works by silencing every aim
    // driver so the reticle stands still while the player points the controller at it. This write
    // is one of those drivers, so it observes the same hold -- otherwise the aim keeps tracking the
    // hand, the reticle never freezes, and there is nothing to calibrate against.
    if (g_aim_calibrating.load(std::memory_order_relaxed)) return;

    // HOLD OFF IN STICK MODE (vehicle seats, cutscenes, death). Halo binds the vehicle chase camera
    // to the aim, so a motion-driven aim swings the whole camera -- which is the entire reason
    // stick mode exists. The control law is disarmed at the stick-mode gate in Plugin.cpp, but this
    // write is driven from the sim's orientation getter and is NOT on that code path, so without
    // this it kept steering the seat camera from the hand while the player's stick did nothing.
    if (g_stick_mode_active.load(std::memory_order_relaxed)) return;

    uintptr_t rec = g_ctl_rec.load(std::memory_order_relaxed);
    if (rec == 0 || IsBadWritePtr((void*)rec, 8)) {
        const char* why = "?";
        rec = resolve_control_record(&why);
        if (rec != 0) {
            g_ctl_rec.store(rec, std::memory_order_relaxed);
            API::get()->log_info("[Halo-CampE-UEVR] BLAMCTL: control record resolved at 0x%llX "
                                 "(yaw +0x94, pitch +0x98)", (unsigned long long)rec);
        } else {
            // NEVER fail silently here -- see the note on resolve_control_record().
            static std::atomic<uint32_t> flog{0};
            const uint32_t fn = flog.fetch_add(1, std::memory_order_relaxed);
            if (fn < 3 || (fn % 8000) == 0) {
                API::get()->log_info("[Halo-CampE-UEVR] BLAMCTL: resolve FAILED (%s) -- blamangles "
                                     "is on but nothing is being written", why);
            }
            return;
        }
    }
    if (IsBadWritePtr((void*)rec, 8)) return;

    float* fp = (float*)rec;
#if HALO_VR_DEV
    // Sampled BEFORE our write: this is what the record holds after whatever last touched it, which
    // is the only way to tell our value surviving from something else owning the slot.
    const float prev_y = fp[0], prev_p = fp[1];
#endif
    if (g_cfg.blam_angles == 3) {
        // Absolute probe: pin a distinctive angle. If this is the authoritative input the view
        // snaps there and stays, regardless of the stick. PINS THE VIEW -- not a play setting.
        fp[0] = 1.50f; fp[1] = 0.30f;
        return;
    }

    float yaw = 0.0f, pitch = 0.0f;
    if (!desired_aim_now(&yaw, &pitch)) return;

    // YAW SIGN. desired_aim_now() returns UE-convention degrees, but this record stores BLAM yaw,
    // which is its negation -- the game derives the aim as (cos p * cos y, -cos p * sin y, sin p).
    //
    // Confirmed live in co-op, and the failure was diagnostic in itself: with no flip the LOCAL
    // view was correct (direct drive owns that independently) while the HOST saw yaw mirrored --
    // and pitch was right on both sides, which is exactly what a Y-only negation predicts, since
    // z = sin(pitch) is identical in both conventions. That asymmetry is also what proved this
    // record is the outbound replication source.
    const float asign = (g_cfg.blam_angles_ysign >= 0) ? 1.0f : -1.0f;
    float ry = asign * (yaw + g_cfg.blam_yaw_off) * DEG2RAD;
    const float rp = (pitch + g_cfg.blam_pitch_off) * DEG2RAD;
    // The game stores yaw wrapped into [0,2pi) and re-wraps after its own store, so match that or
    // the value reads as out of range.
    ry -= std::floor(ry / CTL_YAW_WRAP) * CTL_YAW_WRAP;
    fp[0] = ry; fp[1] = rp;

#if HALO_VR_DEV
    blam_ctl_report(rec, prev_y, prev_p, ry, rp, yaw, pitch);
    g_last_written_y = ry; g_last_written_p = rp; g_have_written = true;
#endif
}

void blam_drive_tick() {
    const bool want = (g_cfg.blam_angles != 0);

    // THE CANONICAL PAIRING IS blamangles=1 WITH aimdirect=1 -- see Config.hpp on either key. Split
    // them and the closed loop is still steering the aim toward a setpoint that this file is also
    // assigning exactly; the two fight, and it reads as heavy jitter rather than as a setting being
    // wrong. That is a miserable thing to diagnose from feel, so say it. Edge-triggered, because
    // config is re-read every ~2 s and a per-poll line would be its own problem.
    {
        static int prev = -1;
        const int state = (want ? 2 : 0) | (g_cfg.aim_direct ? 1 : 0);
        if (state != prev) {
            prev = state;
            if (state == 2) {
                API::get()->log_info("[Halo-CampE-UEVR] BLAMDRIVE: blamangles=1 with aimdirect=0 -- "
                                     "expect jitter. These two are one setting in two halves; set "
                                     "aimdirect=1 unless you are deliberately A/Bing them.");
            } else if (state == 1) {
                API::get()->log_info("[Halo-CampE-UEVR] BLAMDRIVE: aimdirect=1 with blamangles=0 -- "
                                     "local aim only. Other players will not see where you point.");
            }
        }
    }

#if HALO_VR_DEV
    // Ownership of dll+0x5A6AD0, read from config rather than from a runtime claim -- see the
    // block in BlamDrive.hpp for the race that made the handshake version unusable. Both this and
    // blam_aim_tick() evaluate the same value before either touches the address, so there is no
    // window in which both are installed.
    const bool yield = (g_cfg.blam_aim != 0);
#else
    constexpr bool yield = false;
#endif

    if (!want || yield) {
        if (g_hook_id >= 0) {
            API::get()->param()->functions->unregister_inline_hook(g_hook_id);
            g_hook_id  = -1;
            g_original = nullptr;
            g_ctl_rec.store(0, std::memory_order_relaxed);
            API::get()->log_info("[Halo-CampE-UEVR] BLAMDRIVE: hook removed (%s)",
                                 want ? "dev diagnostic hook owns the address" : "blamangles=0");
        }
        return;
    }

    if (g_hook_id >= 0) return;

    HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
    if (sim == nullptr) {
        // Not an error while the game is still loading -- the sim DLL arrives late. Rate-limited so
        // a genuinely missing module is still visible without filling the log during startup.
        static std::atomic<uint32_t> mlog{0};
        const uint32_t mn = mlog.fetch_add(1, std::memory_order_relaxed);
        if (mn == 0 || (mn % 600) == 0) {
            API::get()->log_info("[Halo-CampE-UEVR] BLAMDRIVE: HaloSimulation_tag_release.dll not "
                                 "loaded yet -- aim write idle");
        }
        return;
    }

    g_sim_base  = (uintptr_t)sim;
    g_tls_index = *(const uint32_t*)(g_sim_base + RVA_TLS_INDEX);

    void* target = (void*)(g_sim_base + RVA_GET_ORIENTATION);
    const int id = API::get()->param()->functions->register_inline_hook(
        target, (void*)&hooked_get_orientation, (void**)&g_original);
    if (id < 0 || g_original == nullptr) {
        // Fail LOUD and stay off. A half-installed hook that silently does nothing is the failure
        // mode this whole lane kept hitting; better to say the aim write is unavailable.
        API::get()->log_info("[Halo-CampE-UEVR] BLAMDRIVE: register_inline_hook FAILED (id=%d) on "
                             "0x%llX -- motion aim will not reach the sim",
                             id, (unsigned long long)target);
        g_cfg.blam_angles = 0;
        return;
    }

    g_hook_id = id;
    API::get()->log_info("[Halo-CampE-UEVR] BLAMDRIVE: installed on 0x%llX (dll+0x%llX) id=%d "
                         "tlsIndex=%u -- aim write live",
                         (unsigned long long)target, (unsigned long long)RVA_GET_ORIENTATION,
                         id, g_tls_index);
}

}  // namespace halo
