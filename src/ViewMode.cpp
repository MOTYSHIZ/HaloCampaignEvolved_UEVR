// View topology detector. The doctrine and the per-method table are in ViewMode.hpp.
//
// DEPENDENCY-FREE ON PURPOSE: no API.hpp, no Config.hpp, no logging. It is exercised out of tree
// by a synthetic callback sequence (the render thread cannot be driven from a test, and the
// SimVR rig cannot render AFR or Mono), so anything it pulled in would have to be stubbed there.

#include "ViewMode.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>

namespace halo {

namespace {

// Published state.
std::atomic<int>      g_mode{(int)ViewMode::Unknown};
std::atomic<int>      g_declared{-1};
std::atomic<unsigned> g_samples{0};

// Render-thread working state. Touched only inside viewmode_note_post, which runs on the one
// thread that dispatches the stereo callbacks, so plain fields are correct here -- the atomics
// above are the publication boundary, not the working storage.
uint32_t s_seq = 0;
uint32_t s_last_seq[2] = {0, 0};
bool     s_seen[2]     = {false, false};
float    s_last_pos[2][3] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};

// The alternation test. Same-index deltas only: under Native Stereo consecutive CALLBACKS are the
// two eyes and differ by an IPD by design, but consecutive samples of the SAME index are one frame
// apart and differ by head motion alone. Under AFR the one index that reports carries both eyes,
// so its same-index delta is +IPD, -IPD, +IPD ... plus head motion. An 8-deep ring of "jumped AND
// reversed direction" verdicts separates the two without knowing the IPD or the world scale.
float    s_prev_d[3]   = {0.0f, 0.0f, 0.0f};
bool     s_have_prev_d = false;
uint8_t  s_hist        = 0;   // one bit per same-index sample, newest in bit 0
int      s_hist_n      = 0;   // how many samples the ring holds, saturates at 8

// A jump this large between two frames of one view is not a head: 2 cm in 11 ms is 1.8 m/s. The
// eye-to-eye jump is one IPD in game units -- 6.4 cm at world scale 1.0, and the scale on this
// title is above 1 -- so there is a factor of three between the two even at a sprint.
constexpr float kJumpGameCm = 2.0f;
// Callbacks the other slot may go unheard before it is treated as gone. Under stereo the slots
// take turns, so the other slot is always exactly one callback old.
constexpr uint32_t kStaleCalls = 3;
// Verdicts in the ring that make it Alternating. 6 of 8 tolerates a dropped frame either way.
constexpr int kAltVotes = 6;

int popcount8(uint8_t v) {
    int n = 0;
    while (v != 0) { n += (v & 1u); v = (uint8_t)(v >> 1); }
    return n;
}

} // namespace

void viewmode_note_post(int view_index, float x, float y, float z) {
    const int i = view_index & 1;
    ++s_seq;

    if (s_seen[i]) {
        const float d[3] = {x - s_last_pos[i][0], y - s_last_pos[i][1], z - s_last_pos[i][2]};
        const float len  = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        bool flagged = false;
        if (len > kJumpGameCm && s_have_prev_d) {
            const float dot = d[0] * s_prev_d[0] + d[1] * s_prev_d[1] + d[2] * s_prev_d[2];
            flagged = dot < 0.0f;
        }
        s_hist = (uint8_t)((s_hist << 1) | (flagged ? 1u : 0u));
        if (s_hist_n < 8) ++s_hist_n;
        s_prev_d[0] = d[0]; s_prev_d[1] = d[1]; s_prev_d[2] = d[2];
        s_have_prev_d = true;
    }
    s_last_pos[i][0] = x; s_last_pos[i][1] = y; s_last_pos[i][2] = z;
    s_last_seq[i] = s_seq;
    s_seen[i]     = true;

    const int  o           = 1 - i;
    const bool other_fresh = s_seen[o] && (s_seq - s_last_seq[o]) <= kStaleCalls;

    ViewMode m;
    if (other_fresh) {
        m = ViewMode::Stereo;
    } else if (s_hist_n >= 8) {
        m = (popcount8(s_hist) >= kAltVotes) ? ViewMode::Alternating : ViewMode::Mono;
    } else {
        m = ViewMode::Unknown;
    }
    g_mode.store((int)m, std::memory_order_relaxed);
    g_samples.store(s_seq, std::memory_order_relaxed);
}

ViewMode viewmode_current() {
    return (ViewMode)g_mode.load(std::memory_order_relaxed);
}

void viewmode_set_declared(int method) {
    g_declared.store(method, std::memory_order_relaxed);
}

int viewmode_declared() {
    return g_declared.load(std::memory_order_relaxed);
}

bool viewmode_is_mono() {
    return viewmode_current() == ViewMode::Mono && viewmode_declared() == 3;
}

unsigned viewmode_samples() {
    return g_samples.load(std::memory_order_relaxed);
}

const char* viewmode_name(ViewMode m) {
    switch (m) {
    case ViewMode::Stereo:      return "two views per frame (stereo)";
    case ViewMode::Alternating: return "one view per frame, alternating eyes (AFR)";
    case ViewMode::Mono:        return "one view per frame, the centre eye (mono)";
    default:                    return "not yet classified";
    }
}

const char* viewmode_method_name(int declared_method) {
    switch (declared_method) {
    case 0:  return "Native Stereo";
    case 1:  return "Synchronized Sequential";
    case 2:  return "Alternating/AFR";
    case 3:  return "Mono on the monofix backend (AFW on PureDark's)";
    case -1: return "unreadable";
    default: return "unknown value";
    }
}

} // namespace halo
