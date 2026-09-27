#include "features/roomscale/ThrottleGuard.hpp"
#include "features/roomscale/ThrottleGuardVerdict.hpp"   // the pure half, tested out of tree

#include "Config.hpp"
#include "DevTools.hpp"                    // HALO_VR_DEV: the fault-injection sites
#include "MotionAimControl.hpp"            // g_stick_mode_active
#include "addrcascade/AddressCascade.hpp"
#include "core/UnitState.hpp"              // g_unit_mounted
#include "core/config/CfgRead.hpp"
#include "uevr/API.hpp"

#include <Windows.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>

using uevr::API;

namespace halo {
namespace {

using namespace rs_thr_verdict;

// ---- WHAT IS COMPARED, AND WHY THAT IS THE RIGHT REFERENCE.
//
// Mode 0 and mode 3 deliver the SAME command two ways. Roomscale's aim-frame (right, forward) goes
// into the left stick as (lx, ly), or into the unit as (forward, left) = (ly, ysign < 0 ? -lx : lx);
// Roomscale.cpp's own conversion says so, and blam_throttle_ysign was measured with it. So when the
// GAME fills the throttle from the stick it received, that throttle must point the way the stick
// points under the same conversion -- the stick after our movement rotation, the d-pad shift and
// roomscale's own injection, which is what the XInput slot sees last. That is a prediction of the
// VALUE (addrcascade::ValueAgreement), not merely of motion (CoVariation), which is the test the
// doctrine asks for: the likely failure of a patch is the NEIGHBOURING field of the same struct, and
// one float over, (left, next) points nowhere near (forward, left). The verdict itself -- tolerance,
// duration, breadth -- is in ThrottleGuardVerdict.hpp.
//
// Sampled only while nothing of ours writes the field: UNVERIFIED, roomscale is on the stick and
// writes no throttle at all -- and a pushed stick is what makes roomscale yield anyway. Once VERIFIED
// or WRONG, sampling stops: the offsets do not move within a session.
//
// NOT THE AIM GUARD'S DEADLINE. BLAMLAYOUT (BlamDrive.cpp) arms its write UNVERIFIED after ~90 s,
// because its fallback costs the player their aim. Roomscale's fallback -- the left stick -- keeps
// roomscale working, so here unverified stays closed and simply keeps measuring.

// Fault-injection bits (blamfault), DEV builds only -- see Config::blam_fault.
constexpr int FAULT_RS_THR_SHIFTED = 0x1000;   // read each copy one float late: a member inserted before it
constexpr int FAULT_RS_THR_STUCK   = 0x2000;   // freeze the read at FORWARD: a dead field, and the worst case
                                               // for a player who mostly walks forward (the breadth rule)

// The stick must be held steady this long: the sim fills the throttle once per tick (~33 ms) from the
// input it last polled, so a direction change needs a few ticks to arrive. Sampling across one would
// count the arrival as disagreement.
constexpr long long kSteadyMs  = 150;
constexpr float     kSteadyDeg = 8.0f;        // a steady stick changes less than this between polls
constexpr float     kSteadyMag = 0.1f;
constexpr long long kFreshMs   = 250;         // an older reference is from before a pause or a menu
// The publish runs on every orientation getter call (~2600/s): one sample per sim tick is plenty.
constexpr long long kSampleEveryMs = 30;

enum : int { kUnverified = 0, kVerified = 1, kWrong = 2 };
std::atomic<int> g_state{kUnverified};
// What the verdict was reached FOR. A cfg reload that changes any of them re-opens it: writing is only
// ever allowed with exactly the offsets and handedness that were measured.
std::atomic<int>  g_key_o1{0}, g_key_o2{0}, g_key_ysign{0};
std::atomic<bool> g_have_key{false};
std::atomic<bool> g_said_wanted{false};
std::atomic<bool> g_said_off{false};

// The reference, published by the XInput slot.
std::atomic<float>     g_ref_lx{0.0f}, g_ref_ly{0.0f};
std::atomic<bool>      g_ref_move{false};
std::atomic<long long> g_ref_ms{0};           // when it was last published
std::atomic<long long> g_ref_steady_ms{0};    // since when it has been steady

long long now_ms() {
    return (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// SIM THREAD ONLY: plain state, like the aim record's layout guard.
CopyCheck g_c1{"first"}, g_c2{"second"};

void reopen(const char* why, int o1, int o2, int ys) {
    g_c1.reset_all();
    g_c2.reset_all();
    g_key_o1.store(o1);
    g_key_o2.store(o2);
    g_key_ysign.store(ys);
    g_have_key.store(true);
    g_state.store(kUnverified);
    g_said_wanted.store(false);
    if (why != nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] RSTHROTTLE: %s -- re-verifying the unit throttle at +0x%X / +0x%X "
                             "before roomscale writes it; the left stick carries roomscale meanwhile.",
                             why, (unsigned)o1, (unsigned)o2);
    }
}

// Acts on one copy's window: a proven copy stops sampling, a bad window is re-measured once and then
// condemns the offsets for the session, an inconclusive one starts again. Every outcome but a proof
// says so in the log, with the window's own numbers.
void settle(CopyCheck& c, Window w, int off) {
    WindowReport r;
    switch (settle_window(c, w, r)) {
    case Action::Remeasure:
        API::get()->log_info(
            "[Halo-CampE-UEVR] RSTHROTTLE: the %s throttle copy at +0x%X disagreed with the stick this "
            "window (%u of %u samples within %.0f deg, longest disagreement run %u) -- strike %d of %d, "
            "re-measuring before condemning it.",
            c.name, (unsigned)off, r.agree, r.n, kTolDeg, r.worst_run, r.bad_windows, kBadWindows);
        return;
    case Action::Condemned:
        g_state.store(kWrong);
        API::get()->log_info(
            "[Halo-CampE-UEVR] RSTHROTTLE: WRONG OFFSETS -- the %s throttle copy at +0x%X did not follow the "
            "stick the game received (%u of %u samples within %.0f deg, longest disagreement run %u of %u "
            "allowed) in %d windows. Those offsets are not this build's throttle, so roomscale will NOT write "
            "there this session and keeps moving you through the left stick. Almost certainly a patched or "
            "different-store binary; roomscalethrguard=0 overrides this check.",
            c.name, (unsigned)off, r.agree, r.n, kTolDeg, r.worst_run, kStrikes, r.bad_windows);
        return;
    case Action::Restart:
        API::get()->log_info(
            "[Halo-CampE-UEVR] RSTHROTTLE: still unverified -- the %s throttle copy had %u samples, %.0f of "
            "%.0f deg of stick swing and %d of %d directions: not enough to decide, starting another window. "
            "Roomscale stays on the left stick meanwhile.",
            c.name, r.n, r.motion, kRefMotionDeg, r.sectors, kSectorsNeeded);
        return;
    case Action::Proven:
    case Action::None:
    default:
        return;
    }
}

} // namespace

void rs_thr_guard_publish_stick(float lx, float ly, bool movement) {
    const long long now = now_ms();
    const float plx = g_ref_lx.load(std::memory_order_relaxed), ply = g_ref_ly.load(std::memory_order_relaxed);
    const float m0 = std::sqrt(plx * plx + ply * ply), m1 = std::sqrt(lx * lx + ly * ly);
    bool steady = movement && g_ref_move.load(std::memory_order_relaxed) && std::fabs(m1 - m0) < kSteadyMag;
    if (steady && m0 > 1.0e-3f && m1 > 1.0e-3f) {
        steady = wrapped_err(std::atan2((double)ly, (double)lx) * kRad2Deg,
                             std::atan2((double)ply, (double)plx) * kRad2Deg) < kSteadyDeg;
    }
    if (!steady) g_ref_steady_ms.store(now, std::memory_order_relaxed);
    g_ref_lx.store(lx, std::memory_order_relaxed);
    g_ref_ly.store(ly, std::memory_order_relaxed);
    g_ref_move.store(movement, std::memory_order_relaxed);
    g_ref_ms.store(now, std::memory_order_relaxed);
}

void rs_thr_guard_sample(uintptr_t obj) {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    const int o1 = g_cfg.blam_unit_throttle_off, o2 = g_cfg.blam_unit_throttle_off2, ys = g_cfg.blam_throttle_ysign;
    if (!g_have_key.load()) {
        reopen(nullptr, o1, o2, ys);
    } else if (o1 != g_key_o1.load() || o2 != g_key_o2.load() || ys != g_key_ysign.load()) {
        reopen("the throttle offsets or handedness changed", o1, o2, ys);
    }
    if (g_state.load(std::memory_order_relaxed) != kUnverified) return;   // decided for this session
    if (!g_cfg.roomscale || g_cfg.roomscale_throttle != 3 || o1 == 0 || g_cfg.roomscale_thr_guard == 0) return;
    if (g_cfg.roomscale_thr_probe != 0) return;   // the probe is driving the field
    if (g_stick_mode_active.load(std::memory_order_relaxed) || g_unit_mounted.load(std::memory_order_relaxed)) return;

    const long long now = now_ms();
    static long long s_last = 0;
    if (now - s_last < kSampleEveryMs) return;
    if (!g_ref_move.load(std::memory_order_relaxed)) return;
    if (now - g_ref_ms.load(std::memory_order_relaxed) > kFreshMs) return;
    if (now - g_ref_steady_ms.load(std::memory_order_relaxed) < kSteadyMs) return;
    const float lx = g_ref_lx.load(std::memory_order_relaxed), ly = g_ref_ly.load(std::memory_order_relaxed);
    if (!stick_usable(lx, ly)) return;

    uintptr_t a1 = obj + (uintptr_t)o1;
    uintptr_t a2 = (o2 != 0) ? obj + (uintptr_t)o2 : 0;
#if HALO_VR_DEV
    if (addrcascade::fault(FAULT_RS_THR_SHIFTED)) { a1 += 4; if (a2 != 0) a2 += 4; }
#endif
    // An unreadable unit is not evidence about the offsets: skip, do not count.
    if (obj == 0 || IsBadReadPtr((const void*)a1, 8) || (a2 != 0 && IsBadReadPtr((const void*)a2, 8))) return;
    s_last = now;

    float f1 = ((const float*)a1)[0], l1 = ((const float*)a1)[1];
    float f2 = 0.0f, l2 = 0.0f;
    if (a2 != 0) { f2 = ((const float*)a2)[0]; l2 = ((const float*)a2)[1]; }
#if HALO_VR_DEV
    if (addrcascade::fault(FAULT_RS_THR_STUCK)) { f1 = 1.0f; l1 = 0.0f; f2 = 1.0f; l2 = 0.0f; }
#endif

    const double exp_deg = expected_deg(lx, ly, ys);
    if (!g_c1.proven) settle(g_c1, feed(g_c1, f1, l1, exp_deg), o1);
    if (g_state.load(std::memory_order_relaxed) == kWrong) return;
    if (o2 != 0 && !g_c2.proven) settle(g_c2, feed(g_c2, f2, l2, exp_deg), o2);
    if (g_state.load(std::memory_order_relaxed) == kWrong) return;

    if (g_c1.proven && (o2 == 0 || g_c2.proven)) {
        g_state.store(kVerified);
        g_said_wanted.store(false);
        API::get()->log_info(
            "[Halo-CampE-UEVR] RSTHROTTLE: CONFIRMED -- the game's own throttle at +0x%X%s followed the stick it "
            "received: first copy %u of %u samples within %.0f deg (worst %.1f, longest disagreement run %u of "
            "%u allowed, %.0f deg of stick swing, %d directions)%s. Roomscale now writes the unit's throttle "
            "(mode 3).",
            (unsigned)o1, (o2 != 0) ? " and its copy" : "", g_c1.agree, g_c1.n, kTolDeg, g_c1.va.worst_error(),
            g_c1.va.worst_run(), kStrikes, g_c1.va.reference_motion(), g_c1.sectors(),
            (o2 != 0) ? "; the copy agreed as well" : "");
    }
}

bool rs_thr_guard_allows_write() {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    if (g_cfg.roomscale_thr_guard == 0) {
        if (!g_said_off.exchange(true)) {
            API::get()->log_info("[Halo-CampE-UEVR] RSTHROTTLE: roomscalethrguard=0 -- roomscale writes the unit "
                                 "throttle at +0x%X / +0x%X UNVERIFIED, as it did before 2026-09-27.",
                                 (unsigned)g_cfg.blam_unit_throttle_off, (unsigned)g_cfg.blam_unit_throttle_off2);
        }
        return true;
    }
    g_said_off.store(false, std::memory_order_relaxed);
    return g_state.load(std::memory_order_relaxed) == kVerified && g_have_key.load(std::memory_order_relaxed)
        && g_key_o1.load(std::memory_order_relaxed) == g_cfg.blam_unit_throttle_off
        && g_key_o2.load(std::memory_order_relaxed) == g_cfg.blam_unit_throttle_off2
        && g_key_ysign.load(std::memory_order_relaxed) == g_cfg.blam_throttle_ysign;
}

void rs_thr_guard_note_wanted() {
    if (g_said_wanted.exchange(true)) return;
    if (g_state.load(std::memory_order_relaxed) == kWrong) {
        API::get()->log_info("[Halo-CampE-UEVR] RSTHROTTLE: roomscale is moving you through the left stick -- the "
                             "unit throttle offsets were found WRONG for this binary this session (see the WRONG "
                             "OFFSETS line).");
        return;
    }
    API::get()->log_info("[Halo-CampE-UEVR] RSTHROTTLE: roomscale is moving you through the left stick for now -- the "
                         "unit throttle it would write is UNVERIFIED on this binary. Walk with the stick for a few "
                         "seconds, in more than one direction, and the check either confirms it (RSTHROTTLE: "
                         "CONFIRMED) or refuses it (WRONG OFFSETS).");
}

} // namespace halo
