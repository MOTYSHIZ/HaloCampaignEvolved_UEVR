#include "features/palettewpn/PoseLatch.hpp"

#include "Config.hpp"
#include "core/Services.hpp"   // SVC_POSE_INTENTS: a consumer wants the stamped intents
#include "Math.hpp"
#include "MotionAimControl.hpp"   // desired_aim_now
#include "features/palettewpn/PaletteArmDriver.hpp"   // palette_weapon_mode
#include "uevr/API.hpp"

#include <Windows.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>

using uevr::API;

namespace halo {

// ---- POSELATCH (2026-09-12). ONE HAND SAMPLE PER FRAME FOR EVERY READER.
//
// Every consumer of the controller reads it through this function, at its own moment, on its own
// thread: the Blam aim writer (sim thread, ~2600 calls/s), the UE aim writer (XInput hook), the
// weapon placement (game tick), the frame republish (render thread). UEVR returns the freshest
// prediction at each call, so the hand the sim aimed from, the hand UE aimed from and the hand the
// gun was placed at are different samples a fraction of a frame apart. The fitted model of the
// judder is exactly that: two aim reads about one frame apart with a random slip, and it only
// involves the hand that drives the aim, which is why spawned objects and the left hand are smooth.
//
// With the latch on, a snapshot of every device is taken ONCE at a chosen point and every reader
// in that frame gets the identical numbers. A stale snapshot (no refresh for 100 ms, e.g. a menu
// that stops the tick) falls through to the live read so nothing can freeze.
//   0 = off, live reads (every build before this)
//   1 = controllers latched at the engine tick start
//   2 = controllers AND the HMD latched at the engine tick start
//   3 = controllers latched at the aim law's own sample (XInput hook), the instant the aim is taken
// ---- FRAMEAUDIT (2026-09-12). Every snapshot carries a generation; every reader records the
// generation it was served on its own thread, so each aim write and each placement can be stamped
// with the exact hand sample it used, and the log shows at which hop the two-frame aim delay and
// the one-frame bone delay appear.
void stomp_mark(int point, float yaw, float e0, float e1, float e2);   // Plugin.cpp, STOMPLOG ring
extern std::atomic<unsigned> g_tick_id;                                 // Plugin.cpp
std::atomic<uint32_t> g_latch_gen{0};
std::atomic<uint32_t> g_intent_prev_gen{0};
std::atomic<uint32_t> g_intent_cur_gen{0};
static thread_local uint32_t t_last_gen = 0;
uint32_t pose_latch_last_gen() { return t_last_gen; }

namespace {
struct PoseSnapDev {
    int32_t idx = -1;
    API::VR::Pose grip{}, aim{};
};
SRWLOCK g_snap_lock = SRWLOCK_INIT;
PoseSnapDev g_snap[3];
long long g_snap_ms = 0;
uint32_t g_snap_gen = 0;
std::atomic<uint32_t> g_snap_refreshes{0}, g_snap_served{0}, g_snap_live{0};
std::atomic<int32_t> g_snap_max_age{0};

long long snap_now_ms() {
    return (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

std::atomic<float> g_intent_prev_y{0.0f}, g_intent_prev_p{0.0f}, g_intent_cur_y{0.0f}, g_intent_cur_p{0.0f};
std::atomic<bool>  g_intent_prev_ok{false}, g_intent_cur_ok{false};
// RETSTAMP: the intent from TWO snapshots back. At render the game shows exactly this aim
// (frame audit: ControlRotation at render = intent(gen-2) on 99.5% of frames).
std::atomic<float> g_intent_prev2_y{0.0f}, g_intent_prev2_p{0.0f};
std::atomic<bool>  g_intent_prev2_ok{false};

void pose_latch_refresh(int site) {
    // Only while the palette weapon (armdriver mode 3) owns the aim: a latched hand sample is part of
    // that stack's frame audit, and the author's aim path reads the live pose.
    const int mode = palette_weapon_mode() ? g_cfg.pose_latch : 0;
    if (mode == 0) return;
    if (site == 1 && mode != 1 && mode != 2) return;
    if (site == 3 && mode != 3) return;
    // DRAWAIM (palettecam 14/15, 2026-09-12 fit): the frame draws bones written one frame earlier
    // but composes them with its own aim, and that aim is exactly the intent of the snapshot BEFORE
    // the build's (aim(t) = intent(t-2), median error 0.0000 deg). So the intent of the outgoing
    // snapshot is stored here, before the swap, and the placement divides by that stored number.
    if (g_cfg.palette_cam == 14 || g_cfg.stomp_log != 0 || service_active(SVC_POSE_INTENTS)) {
        // Shift the outgoing previous intent down one slot before computing the new one.
        g_intent_prev2_y.store(g_intent_prev_y.load(std::memory_order_relaxed), std::memory_order_relaxed);
        g_intent_prev2_p.store(g_intent_prev_p.load(std::memory_order_relaxed), std::memory_order_relaxed);
        g_intent_prev2_ok.store(g_intent_prev_ok.load(std::memory_order_relaxed), std::memory_order_relaxed);
        float py = 0.0f, pp = 0.0f;
        const bool ok = desired_aim_now(&py, &pp) && std::isfinite(py) && std::isfinite(pp);
        if (ok) { g_intent_prev_y.store(py, std::memory_order_relaxed); g_intent_prev_p.store(pp, std::memory_order_relaxed); }
        g_intent_prev_ok.store(ok, std::memory_order_relaxed);
        g_intent_prev_gen.store(t_last_gen, std::memory_order_relaxed);
    } else {
        g_intent_prev_ok.store(false, std::memory_order_relaxed);   // never serve a stale value after a mode switch
        g_intent_prev2_ok.store(false, std::memory_order_relaxed);
    }
    const int32_t ids[3] = {API::VR::get_hmd_index(), API::VR::get_left_controller_index(),
                            API::VR::get_right_controller_index()};
    PoseSnapDev fresh[3];
    for (int i = 0; i < 3; ++i) {
        fresh[i].idx = ids[i];
        if (ids[i] < 0) continue;
        fresh[i].grip = API::VR::get_pose(ids[i]);
        fresh[i].aim  = API::VR::get_aim_pose(ids[i]);
    }
    const long long now = snap_now_ms();
    AcquireSRWLockExclusive(&g_snap_lock);
    for (int i = 0; i < 3; ++i) g_snap[i] = fresh[i];
    g_snap_ms = now;
    const uint32_t new_gen = g_latch_gen.fetch_add(1, std::memory_order_relaxed) + 1;
    g_snap_gen = new_gen;
    ReleaseSRWLockExclusive(&g_snap_lock);
    g_snap_refreshes.fetch_add(1, std::memory_order_relaxed);
    if (g_cfg.palette_cam == 15 || g_cfg.stomp_log != 0) {
        float cy = 0.0f, cp = 0.0f;
        const bool ok = desired_aim_now(&cy, &cp) && std::isfinite(cy) && std::isfinite(cp);
        if (ok) { g_intent_cur_y.store(cy, std::memory_order_relaxed); g_intent_cur_p.store(cp, std::memory_order_relaxed); }
        g_intent_cur_ok.store(ok, std::memory_order_relaxed);
        g_intent_cur_gen.store(new_gen, std::memory_order_relaxed);
        // Point 26: the generation table, this snapshot's intent, so every aim value can be
        // matched back to the sample that produced it.
        if (ok) stomp_mark(26, cy, cp, (float)new_gen, (float)g_tick_id.load(std::memory_order_relaxed));
    } else {
        g_intent_cur_ok.store(false, std::memory_order_relaxed);
    }

    static long long s_last_log = 0;
    if (s_last_log == 0) s_last_log = now;
    if (g_cfg.palette_weapon_log != 0 && now - s_last_log >= 10000) {
        const double secs = (double)(now - s_last_log) / 1000.0;
        API::get()->log_info("[Halo-CampE-UEVR] POSELATCH mode %d: %.1f refreshes/s, %.0f reads/s served "
                             "from the snapshot, %.0f reads/s live (stale or unlatched device), oldest "
                             "snapshot served %d ms",
                             mode, g_snap_refreshes.exchange(0) / secs, g_snap_served.exchange(0) / secs,
                             g_snap_live.exchange(0) / secs, (int)g_snap_max_age.exchange(0));
        s_last_log = now;
    }
}

// The latched pose for idx, or false when the live read must be used.
bool pose_latch_lookup(UEVR_TrackedDeviceIndex idx, bool use_aim, API::VR::Pose* out) {
    const int mode = palette_weapon_mode() ? g_cfg.pose_latch : 0;
    if (mode == 0 || idx < 0) return false;
    bool hit = false;
    int32_t age = 0;
    AcquireSRWLockShared(&g_snap_lock);
    if (g_snap_ms != 0) {
        age = (int32_t)(snap_now_ms() - g_snap_ms);
        if (age >= 0 && age < 100) {
            for (int i = 0; i < 3; ++i) {
                if (g_snap[i].idx != idx) continue;
                if (i == 0 && mode != 2) break;          // HMD latched only in mode 2
                *out = use_aim ? g_snap[i].aim : g_snap[i].grip;
                t_last_gen = g_snap_gen;
                hit = true;
                break;
            }
        }
    }
    ReleaseSRWLockShared(&g_snap_lock);
    if (hit) {
        g_snap_served.fetch_add(1, std::memory_order_relaxed);
        int32_t m = g_snap_max_age.load(std::memory_order_relaxed);
        while (age > m && !g_snap_max_age.compare_exchange_weak(m, age, std::memory_order_relaxed)) {}
    } else {
        g_snap_live.fetch_add(1, std::memory_order_relaxed);
        t_last_gen = 0;            // FRAMEAUDIT: 0 = this read did not come from a snapshot
    }
    return hit;
}

// ---- WRITER AGREEMENT. The Blam writer notes the angle it wrote; the UE writer compares its own
// against it when both belong to the same frame. With one hand sample per frame the difference is
// zero by construction, so this line is the number that proves the latch did what it claims.
namespace {
std::atomic<float> g_wa_blam_y{0.0f}, g_wa_blam_p{0.0f};
std::atomic<long long> g_wa_blam_ms{0};
}
void aim_writer_note_blam(float yaw_deg, float pitch_deg) {
    if (g_cfg.palette_weapon_log == 0) return;
    g_wa_blam_y.store(yaw_deg, std::memory_order_relaxed);
    g_wa_blam_p.store(pitch_deg, std::memory_order_relaxed);
    g_wa_blam_ms.store(snap_now_ms(), std::memory_order_relaxed);
}
void aim_writer_compare_direct(float yaw_deg, float pitch_deg) {
    if (g_cfg.palette_weapon_log == 0) return;     // diagnostic only; no work in play
    static double s_sum = 0.0, s_max = 0.0;
    static uint32_t s_n = 0, s_skip = 0;
    static long long s_last = 0;
    const long long now = snap_now_ms();
    const long long bms = g_wa_blam_ms.load(std::memory_order_relaxed);
    if (bms != 0 && now - bms <= 8) {
        const double d = std::fabs((double)wrap180(yaw_deg - g_wa_blam_y.load(std::memory_order_relaxed)))
                       + std::fabs((double)(pitch_deg - g_wa_blam_p.load(std::memory_order_relaxed)));
        s_sum += d; if (d > s_max) s_max = d; ++s_n;
    } else {
        ++s_skip;
    }
    if (s_last == 0) s_last = now;
    if (now - s_last >= 10000) {
        API::get()->log_info("[Halo-CampE-UEVR] AIMWRITERS UE-direct vs Blam angle written within 8 ms: "
                             "n=%u mean |dyaw|+|dpitch| %.3f deg, max %.3f deg, %u unpaired  "
                             "[poselatch=%d aimdirectwrite=%d; 0.000 = both writers used one hand sample]",
                             s_n, s_n ? s_sum / s_n : 0.0, s_max, s_skip, g_cfg.pose_latch, g_cfg.aim_direct_write);
        s_sum = s_max = 0.0; s_n = s_skip = 0; s_last = now;
    }
}

// AIMDIRECTWRITE (Config.hpp aim_direct_write). 0 = keep the direct branch (stick held at
// zero, so the loop cannot become a third writer) but skip the UE write, leaving the Blam
// record as the ONLY aim writer. The honest single-writer test: blamangles=0 instead left
// nothing moving the aim at all (hand vs ControlRotation corr +0.21/-0.34/-0.09).
bool palette_wpn_aim_direct_write_skipped() { return g_cfg.aim_direct_write == 0; }

void palette_wpn_aim_direct_written(float yaw, float pitch) {
    // Point 22: the UE direct write, stamped with the snapshot this thread's hand came from.
    if (g_cfg.stomp_log != 0) stomp_mark(22, yaw, pitch, (float)pose_latch_last_gen(),
               (float)g_tick_id.load(std::memory_order_relaxed));
}

} // namespace halo
