#include "features/reloadvr/ReloadHold.hpp"

#include "Config.hpp"
#include "core/config/CfgRead.hpp"
#include "core/reload/ReloadEngine.hpp"   // reload_hold_window: the reload's own window
#include "palettearm/ArmSolve.hpp"        // slerp_basis
#include "uevr/API.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace halo {

namespace pa = ::halo::palettearm;

namespace {

// THE BUILD'S ONE SAMPLE. Taken by the gates slot, read by every slot after it in the same pose
// build. Sim thread only: the whole file runs inside the base mod's palette drive, which is one
// thread, and nothing here is touched from anywhere else.
struct Snapshot {
    int   mode = 0;        // g_cfg.reload_hold, as of this build
    bool  busy = false;    // the reload engine's window, as of this build
    float w    = 0.0f;     // the ramp weight, derived from the age of that window's own edge
};
Snapshot s_snap{};

// THE LATCH: the base mod's weapon target as it stood when the hold engaged.
bool     s_latched = false;
pa::Mat3 s_latch_basis{};
pa::Vec3 s_latch_pos{};

// The aim wrist the controller asked for, noted before his carry and given back after it.
bool     s_note_have = false;
pa::Vec3 s_note_pos{};
pa::Mat3 s_note_basis{};

// The log's accumulators for one reload.
bool      s_engaged      = false;
int       s_engaged_mode = 0;
long long s_engaged_at   = 0;
float     s_drift_cm     = 0.0f;
float     s_drift_deg    = 0.0f;

float ms_since(long long ticks) {
    const auto d = std::chrono::steady_clock::duration(
        std::chrono::steady_clock::now().time_since_epoch().count() - ticks);
    return std::chrono::duration_cast<std::chrono::duration<float, std::milli>>(d).count();
}

// THE RAMP, DERIVED AND NOT INTEGRATED. The window publishes a level and the instant it last
// changed; the weight is a function of the age of that edge, so this thread runs no clock of its
// own and cannot drift against the game thread's. Smoothstep, so the gun neither leaves nor
// returns with a step in its velocity -- the release is the edge that would otherwise read as a
// snap.
float ramp_weight(const ReloadHoldWindow& win, float ramp_ms) {
    float t = 1.0f;
    if (ramp_ms > 1.0f && win.edge_ticks != 0) {
        const float age = ms_since(win.edge_ticks);
        t = (age <= 0.0f) ? 0.0f : (age >= ramp_ms ? 1.0f : age / ramp_ms);
    }
    t = t * t * (3.0f - 2.0f * t);
    return win.busy ? t : (1.0f - t);
}

float forward_angle_deg(const pa::Mat3& a, const pa::Mat3& b) {
    const float c = std::clamp(pa::dot(a.forward, b.forward), -1.0f, 1.0f);
    return std::acos(c) * 57.2957795f;
}

void take_snapshot() {
    CFG_HOOK_READ;
    Snapshot s{};
    s.mode = g_cfg.reload_hold;
    // NOT FOR ANY OTHER ARM DRIVER, and the guarantee is structural rather than a test. Every call
    // site is inside the base mod's palette drive; that drive is installed by palettearm_update
    // only while armdriver 2 owns the arms, and palettearm_release() uninstalls the hook and clears
    // the drive pointer on the way out, so under armdriver 3 not one of these slots is reached and
    // the fork's own reloadposefreeze keeps that mode to itself. Testing it here would mean reading
    // the arbiter's mode off the sim thread -- the read the fork's own driver deliberately replaced
    // with an atomic mirror -- so this asks nothing at all.
    if (s.mode > 0) {
        const ReloadHoldWindow win = reload_hold_window();   // ONE read of level and edge together
        s.busy = win.busy;
        s.w    = std::clamp(ramp_weight(win, g_cfg.reload_hold_ramp_ms), 0.0f, 1.0f);
    }

    const bool live = s.mode > 0 && (s.busy || s.w > 0.001f);
    if (live && !s_engaged) {
        s_engaged      = true;
        s_engaged_mode = s.mode;
        s_engaged_at   = std::chrono::steady_clock::now().time_since_epoch().count();
        s_drift_cm = s_drift_deg = 0.0f;
        if (g_cfg.reload_hold_log)
            uevr::API::get()->log_info(
                "[Halo-CampE-UEVR] RELOADHOLD engages (reloadgunhold=%d: %s), ramp %.0f ms",
                s.mode,
                s.mode == 1 ? "his own per-action hold, raised"
                            : (s.mode == 2 ? "the weapon target latched"
                                           : "the weapon target latched, the aim hand left tracking"),
                g_cfg.reload_hold_ramp_ms);
    } else if (!live && s_engaged) {
        if (g_cfg.reload_hold_log)
            uevr::API::get()->log_info(
                "[Halo-CampE-UEVR] RELOADHOLD releases after %.0f ms (reloadgunhold=%d); the base mod's "
                "weapon target travelled up to %.1f cm and %.1f deg while the gun was held",
                ms_since(s_engaged_at), s_engaged_mode, s_drift_cm, s_drift_deg);
        s_engaged   = false;
        s_latched   = false;
        s_note_have = false;
    }
    s_snap = s;
}

}  // namespace

// ---- MECHANISM 1, and the build's snapshot for the other two.
//
// RAISE ONLY. His gates fold to the maximum of three modes; raising one more is one more gate of
// his own as far as everything downstream is concerned, and it can never pull one of his back
// down. hold + off together is exactly his mode 3 ("no animation"): the gun and the aim wrist eased
// onto their rest pose, the aim fingers with them, the kick faded, and the support hand kept OFF
// the animation and on its own controller, which is the hand that has to fetch the magazine.
//
// THE POINT IS UPSTREAM OF EVERYTHING THAT READS THESE WEIGHTS: his per-action hold block, and the
// carry after it, both consume what this leaves. Nothing here ever sees his hold's output, which is
// the failure that cost two days when rigcarryrot went in on the other side of it.
void reload_hold_pa_anim_gates(bool is_capture_bank, float& join_w, float& off_w, float& stock_w_all,
                               float& hold_w) {
    (void)join_w;
    (void)stock_w_all;
    // The live slot leads and the capture banks reuse its sample, so every bank draws one pose.
    if (!is_capture_bank) take_snapshot();
    const Snapshot s = s_snap;
    if (s.mode != 1 || s.w <= 0.001f) return;
    hold_w = (std::max)(hold_w, s.w);
    off_w  = (std::max)(off_w,  s.w);
}

// ---- MECHANISMS 2 AND 3, and the measurement every mechanism reports.
//
// The target is latched on the first build of the hold and fed back in, weighted, until the ramp
// has run out at the other end -- so the gun eases onto where it was rather than jumping there, and
// eases back onto the hand rather than snapping. Under mechanism 1 the latch is taken and measured
// but never applied: his hold is doing the holding, and the number is still worth having.
//
// UPSTREAM OF HIS REST-POSE HOLD AND OF THE CARRY. What this hands back is an ordinary weapon
// target; his hold, his kick and his carry then run on it exactly as they run on his own.
void reload_hold_pa_rig_target(bool is_capture_bank, bool have_rt, pa::Mat3& basis, pa::Vec3& position) {
    const Snapshot s = s_snap;
    if (s.mode <= 0 || !have_rt) return;
    if (!pa::valid_basis(basis) || !pa::finite(position)) return;
    if (!s.busy && s.w <= 0.001f) { if (!is_capture_bank) s_latched = false; return; }
    if (!s_latched) {
        if (is_capture_bank) return;      // the live slot takes the latch; the banks then share it
        s_latch_basis = basis;
        s_latch_pos   = position;
        s_latched     = true;
    }
    if (!is_capture_bank) {
        const float cm = pa::length(position - s_latch_pos) * pa::kMetresPerBlamUnit * 100.0f;
        if (std::isfinite(cm) && cm > s_drift_cm) s_drift_cm = cm;
        const float deg = forward_angle_deg(basis, s_latch_basis);
        if (std::isfinite(deg) && deg > s_drift_deg) s_drift_deg = deg;
    }
    if (s.mode == 1) return;              // measurement only: his hold holds the gun
    const pa::Vec3 p = position + (s_latch_pos - position) * s.w;
    const pa::Mat3 b = pa::slerp_basis(basis, s_latch_basis, s.w);
    if (pa::finite(p) && pa::valid_basis(b)) { position = p; basis = b; }
}

// ---- MECHANISM 3 ONLY: the gun pinned, the drawn aim hand still the player's.
//
// Under the base mod's carry the aim hand is placed ON the gun, so pinning the gun pins the hand
// with it. This notes the wrist the controller alone asked for and gives it back after his block,
// weighted by the same ramp: at full hold the gun stands still and the hand keeps tracking, and the
// two visibly separate. It exists to be compared against mechanism 2, which does not separate them.
void reload_hold_pa_aim_wrist_note(bool is_aim, bool is_capture_bank, const pa::Vec3& wrist_target,
                                   const pa::Mat3& desired_wrist) {
    (void)is_capture_bank;
    if (!is_aim) return;
    const Snapshot s = s_snap;
    if (s.mode != 3 || s.w <= 0.001f) { s_note_have = false; return; }
    s_note_pos   = wrist_target;
    s_note_basis = desired_wrist;
    s_note_have  = pa::finite(wrist_target) && pa::valid_basis(desired_wrist);
}

void reload_hold_pa_aim_wrist_keep(bool is_aim, bool is_capture_bank, pa::Vec3& wrist_target,
                                   pa::Mat3& desired_wrist) {
    (void)is_capture_bank;
    if (!is_aim || !s_note_have) return;
    s_note_have = false;                  // one note, one use: the pair brackets one block
    const Snapshot s = s_snap;
    if (s.mode != 3 || s.w <= 0.001f) return;
    const pa::Vec3 p = wrist_target + (s_note_pos - wrist_target) * s.w;
    const pa::Mat3 b = pa::slerp_basis(desired_wrist, s_note_basis, s.w);
    if (pa::finite(p) && pa::valid_basis(b)) { wrist_target = p; desired_wrist = b; }
}

} // namespace halo
