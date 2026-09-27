#pragma once

// THE THROTTLE GUARD'S VERDICT -- the pure half of features/roomscale/ThrottleGuard: given the stick the
// game received and what sits at a throttle offset, does that offset hold the throttle?
//
// No game, engine or project includes -- only addrcascade, itself self-contained -- so
// Scripts\Verify-ThrottleGuardStandalone.ps1 can compile it OUT OF TREE (/W4 /WX) and drive it with
// synthetic fields: healthy, one float late, frozen at forward, dead, mirrored. Run that after any
// change here. ThrottleGuard.cpp owns everything that touches the game: the stick it publishes, the
// memory it reads, when it samples, and the log lines.

#include "addrcascade/AddressCascade.hpp"

#include <cmath>
#include <cstdint>

namespace halo::rs_thr_verdict {

// The stick must be well past the game's own deadzone (~0.30, roomscale_dz) for its direction to be
// the game's.
constexpr float kRefMagMin = 0.6f;
// The throttle's magnitude must be sane: nonzero past the deadzone, not a position or a count. Only
// the DIRECTION is compared -- the game's deadzone and response curve set the magnitude.
constexpr float kMagMin = 0.1f, kMagMax = 2.0f;

// TOLERANCE. The healthy error comes from the game's AXIAL deadzone (the biped tag's controller input:
// 0.125 per axis): at the 0.6 floor it can zero a small component, asin(0.125 / 0.6) = 12 deg at most.
// 30 is 2.5x that. A wrong field is not "30 deg off" -- it has nothing to do with the stick.
// NOT MEASURED on this game yet: the CONFIRMED line reports the worst error and the longest
// disagreement run seen, so this and kStrikes can be tightened on evidence.
constexpr double kTolDeg = 30.0;
// DURATION decides disagreement, as in the aim record's layout guard (BlamDrive.cpp): a wrong field
// disagrees on every sample, a healthy excursion is a transient. 15 samples is ~0.5 s of CONTINUOUS
// disagreement with the stick held steady past the deadzone. Far below the aim guard's 600, because a
// false positive is cheap here (roomscale stays on the stick, which works) while a false negative
// writes into another field every tick.
constexpr uint32_t kStrikes      = 15;
constexpr uint32_t kMinSamples   = 30;        // ~1 s of steady stick at one sample per sim tick
constexpr double   kRefMotionDeg = 90.0;      // the stick must have swung this far in the window
constexpr uint32_t kMaxSamples   = 1800;      // ~60 s of steady stick, then a fresh window
// A MATCH ALSO NEEDS BREADTH. ValueAgreement credits a match on the first agreeing sample once it has
// enough samples and motion -- right for the aim record, whose wrong field disagrees by 80-90 deg on
// every sample. Here a field that holds a constant FORWARD vector would agree with a player who mostly
// walks forward. So a match also needs kAgreeRatio of ALL samples within tolerance, and agreement in at
// least kSectorsNeeded of four stick directions (forward, left, back, right: 90 deg each) with
// kSectorAgree samples in each. A constant agrees in one direction at most.
constexpr double   kAgreeRatio    = 0.9;
constexpr uint32_t kSectorAgree   = 8;
constexpr int      kSectorsNeeded = 2;
// A window that runs out of samples while mostly DISAGREEING is evidence against; one that runs out
// while mostly agreeing just never swung far or wide enough, which is evidence of nothing.
constexpr double kMostlyWrong = 0.6;
// Windows that must end in disagreement before the offsets are condemned for the session: one bad
// window is re-measured first, as the aim guard does.
constexpr int kBadWindows = 2;

constexpr double kRad2Deg = 57.29577951308232;

inline double wrapped_err(double a, double b) {
    const double d = std::fmod(std::fabs(a - b), 360.0);
    return (d > 180.0) ? 360.0 - d : d;
}

// Forward (-45..45), left, back, right: the quarter of the circle the STICK points into.
inline int sector_of(double deg) {
    const double a = std::fmod(deg + 45.0 + 720.0, 360.0);
    return ((int)(a / 90.0)) & 3;
}

inline bool stick_usable(float lx, float ly) { return std::sqrt(lx * lx + ly * ly) >= kRefMagMin; }

// The direction, in degrees, of the throttle the game makes of stick (lx, ly) -- in the convention
// roomscale writes with: (forward, left) = (ly, ysign < 0 ? -lx : lx).
inline double expected_deg(float lx, float ly, int ysign) {
    const float ef = ly, el = (ysign < 0) ? -lx : lx;
    return std::atan2((double)el, (double)ef) * kRad2Deg;
}

inline addrcascade::ValueAgreement::Config agreement_cfg() {
    addrcascade::ValueAgreement::Config c;
    c.tolerance                 = kTolDeg;
    c.reference_motion_required = kRefMotionDeg;
    c.min_samples               = kMinSamples;
    c.max_samples               = kMaxSamples;
    c.strikes_to_fail           = kStrikes;
    c.wrap                      = 360.0;   // or a 179 -> -179 step reads as 358 deg of motion
    return c;
}

// One copy of the throttle. The unit carries two and roomscale writes both, so both must be proven.
struct CopyCheck {
    const char* name;
    addrcascade::ValueAgreement va{agreement_cfg()};
    uint32_t n = 0, agree = 0;
    uint32_t sector[4] = {0, 0, 0, 0};
    int  bad_windows = 0;
    bool proven = false;

    explicit CopyCheck(const char* nm) : name(nm) {}
    void reset_window() { va.reset(); n = 0; agree = 0; for (auto& s : sector) s = 0; }
    void reset_all()    { reset_window(); bad_windows = 0; proven = false; }
    int  sectors() const {
        int k = 0;
        for (uint32_t s : sector) if (s >= kSectorAgree) ++k;
        return k;
    }
};

enum class Window { Pending, Proven, Bad, Inconclusive };

// One sample: the throttle (f, l) read at the copy's offset, against the direction the stick predicts.
inline Window feed(CopyCheck& c, float f, float l, double exp_deg) {
    const float mag = std::sqrt(f * f + l * l);
    const bool sane = std::isfinite(f) && std::isfinite(l) && mag >= kMagMin && mag <= kMagMax;
    // An insane read counts as disagreement, not as a skipped sample: a field that reads zero or
    // garbage while the stick is pushed is exactly the evidence this exists to collect.
    const double cand_deg = sane ? std::atan2((double)l, (double)f) * kRad2Deg : exp_deg + 180.0;
    ++c.n;
    if (wrapped_err(cand_deg, exp_deg) <= kTolDeg) { ++c.agree; ++c.sector[sector_of(exp_deg)]; }
    using V = addrcascade::ValueAgreement::Verdict;
    const V v = c.va.sample(cand_deg, exp_deg);
    if (v == V::Mismatch) return Window::Bad;
    if (v == V::Match && (double)c.agree >= kAgreeRatio * (double)c.n && c.sectors() >= kSectorsNeeded)
        return Window::Proven;
    if (v == V::Inconclusive || c.n >= kMaxSamples)
        return ((double)c.agree < kMostlyWrong * (double)c.n) ? Window::Bad : Window::Inconclusive;
    return Window::Pending;
}

// What a closed window means for the copy, and the numbers it closed with (for the log line, which
// has to be written before the window is reset).
enum class Action { None, Proven, Remeasure, Condemned, Restart };
struct WindowReport {
    uint32_t n = 0, agree = 0, worst_run = 0;
    double   motion = 0.0, worst_err = 0.0;
    int      sectors = 0, bad_windows = 0;
};

inline Action settle_window(CopyCheck& c, Window w, WindowReport& rep) {
    rep.n = c.n; rep.agree = c.agree; rep.worst_run = c.va.worst_run();
    rep.motion = c.va.reference_motion(); rep.worst_err = c.va.worst_error();
    rep.sectors = c.sectors(); rep.bad_windows = c.bad_windows;
    switch (w) {
    case Window::Proven:
        c.proven = true;
        return Action::Proven;
    case Window::Bad:
        rep.bad_windows = ++c.bad_windows;
        if (c.bad_windows < kBadWindows) { c.reset_window(); return Action::Remeasure; }
        return Action::Condemned;
    case Window::Inconclusive:
        c.reset_window();
        return Action::Restart;
    case Window::Pending:
    default:
        return Action::None;
    }
}

} // namespace halo::rs_thr_verdict
