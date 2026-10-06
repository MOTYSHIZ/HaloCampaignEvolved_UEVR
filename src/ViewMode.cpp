// View topology detector. The doctrine and the per-method table are in ViewMode.hpp.
//
// DEPENDENCY-FREE ON PURPOSE: no API.hpp, no Config.hpp, no logging. It is exercised out of tree
// by a synthetic callback sequence (the stereo callback cannot be driven from a test, and the
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
std::atomic<bool>     g_shared_projection{false};
std::atomic<bool>     g_backend_warps{false};
std::atomic<bool>     g_extreme_compat{false};
std::atomic<unsigned> g_samples{0};
// Diagnostics for the VIEWMODE line: the vote ring as of the last sample, and the largest
// second difference seen since the poll last read it (in game cm). Relaxed stores; the callback
// never waits on them.
std::atomic<unsigned> g_diag_ring{0};
std::atomic<float>    g_diag_swing_max{0.0f};

// The callback's working state. Touched only inside viewmode_note_post, which runs on the one
// thread that dispatches the stereo callbacks, so plain fields are correct here -- the atomics
// above are the publication boundary, not the working storage.
uint32_t s_seq = 0;
uint32_t s_last_seq[2] = {0, 0};
bool     s_seen[2]     = {false, false};
float    s_last_pos[2][3] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
bool     s_two_view    = false;   // what the previous sample concluded about the other slot

// THE ALTERNATION TEST, on the SECOND difference of the same-index position.
//
// Same-index only: under Native Stereo consecutive CALLBACKS are the two eyes and differ by an
// IPD by design, but consecutive samples of the SAME index are one frame apart and differ by
// head motion alone. Under AFR the one index that reports carries both eyes.
//
// Second difference, not first: the per-frame delta under AFR is  d_n = v_n +/- J  (v = the view's
// motion that frame, J = one IPD in game units along the head's right axis), so a sign test on
// consecutive deltas is  |v|^2 - |J|^2  and stops working the moment the camera moves faster than
// one IPD per frame -- 8.4 cm at this title's world scale, i.e. every vehicle and nearly a sprint
// at 72 Hz. The difference of two consecutive deltas,  a_n = d_n - d_(n-1) = (v_n - v_(n-1)) -/+ 2J,
// cancels the common motion: under AFR it is a swing of two IPDs that reverses every frame at ANY
// speed; under Mono or Stereo it is the view's acceleration, millimetres per frame^2 even in a
// Warthog. An 8-deep ring of "swung AND reversed" verdicts separates the two without knowing the
// IPD or the world scale.
float    s_prev_d[3]   = {0.0f, 0.0f, 0.0f};   // previous same-index delta
bool     s_have_prev_d = false;
float    s_prev_a[3]   = {0.0f, 0.0f, 0.0f};   // previous second difference
bool     s_have_prev_a = false;
uint8_t  s_hist        = 0;   // one bit per same-index sample, newest in bit 0
int      s_hist_n      = 0;   // how many samples the ring holds, saturates at 8

// A second difference this large is not a head accelerating: 4 cm per frame^2 at 90 Hz is 320 m/s^2.
// The AFR swing is two IPDs in game units -- 12.8 cm at world scale 1.0, 16.8 cm at this title's
// 1.312, and still 6.4 cm at a world scale of 0.5.
constexpr float kSwingGameCm = 4.0f;
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

void reset_ring() {
    s_hist = 0;
    s_hist_n = 0;
    s_have_prev_d = false;
    s_have_prev_a = false;
}

} // namespace

void viewmode_note_post(int view_index, float x, float y, float z) {
    const int i = view_index & 1;
    ++s_seq;

    const int  o           = 1 - i;
    const bool other_fresh = s_seen[o] && (s_seq - s_last_seq[o]) <= kStaleCalls;

    // TWO VIEWS -> ONE VIEW: start the vote from nothing. The ring still holds the stereo era's
    // (never-swinging) samples, and letting them count would read as Mono for the six or so
    // frames it takes to outvote them -- during which a consumer would take ONE eye as the head,
    // which under AFR is the half-IPD hop this module exists to remove. Unknown instead, which the
    // consumers treat as "average with the previous sample": harmless if it turns out to be Mono,
    // right if it turns out to be AFR.
    if (s_two_view && !other_fresh) reset_ring();
    s_two_view = other_fresh;

    if (s_seen[i]) {
        const float d[3] = {x - s_last_pos[i][0], y - s_last_pos[i][1], z - s_last_pos[i][2]};
        if (s_have_prev_d) {
            const float a[3] = {d[0] - s_prev_d[0], d[1] - s_prev_d[1], d[2] - s_prev_d[2]};
            const float len  = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
            bool flagged = false;
            if (len > kSwingGameCm && s_have_prev_a) {
                const float dot = a[0] * s_prev_a[0] + a[1] * s_prev_a[1] + a[2] * s_prev_a[2];
                flagged = dot < 0.0f;
            }
            s_hist = (uint8_t)((s_hist << 1) | (flagged ? 1u : 0u));
            if (s_hist_n < 8) ++s_hist_n;
            s_prev_a[0] = a[0]; s_prev_a[1] = a[1]; s_prev_a[2] = a[2];
            s_have_prev_a = true;
            if (len > g_diag_swing_max.load(std::memory_order_relaxed)) {
                g_diag_swing_max.store(len, std::memory_order_relaxed);
            }
        }
        s_prev_d[0] = d[0]; s_prev_d[1] = d[1]; s_prev_d[2] = d[2];
        s_have_prev_d = true;
    }
    s_last_pos[i][0] = x; s_last_pos[i][1] = y; s_last_pos[i][2] = z;
    s_last_seq[i] = s_seq;
    s_seen[i]     = true;

    // WHAT UEVR SAYS IT IS DOING, for the two decisions below that the positions cannot make alone.
    // Extreme Compatibility Mode forces the alternating plumbing under EVERY method (is_using_afr()
    // in UEVR's VR.hpp), and our own Mono yields to it (is_mono_rendering()).
    const int  declared = g_declared.load(std::memory_order_relaxed);
    const bool warps    = g_backend_warps.load(std::memory_order_relaxed);
    const bool extreme  = g_extreme_compat.load(std::memory_order_relaxed);
    const bool declared_alternating =
        extreme || declared == 1 || declared == 2 || (declared == 3 && warps);

    ViewMode m;
    if (other_fresh) {
        m = ViewMode::Stereo;
    } else if (g_shared_projection.load(std::memory_order_relaxed) && declared == 3 && !warps && !extreme) {
        // ONE PROJECTION FOR BOTH EYES, ON THE MONO METHOD OF A BACKEND THAT HAS ONE, SETTLES IT.
        // Needed because on this title the mono view's position DOES swing while scoped and
        // walking (2026-09-15 log: the ring voted Alternating for a second at a time, flattening
        // flickered off and on, and the scope pane visibly switched between stereo and mono).
        //
        // THE SHARED PROJECTION ALONE PROVES NOTHING, and this branch used to rest on it alone
        // ("an alternating pair always carries mirrored frustums"). False: UEVR's
        // HorizontalProjectionOverride=Symmetric hands both eyes one frustum on ANY method and any
        // backend (OpenXR.cpp get_mat), and a symmetric-lens headset does it unasked. PureDark's AFW
        // renders the eyes BY TURNS (true_index = g_frame_count % 2, the eye offset applied each
        // frame -- FFakeStereoRenderingHook.cpp, read at commit c40eaab) and reported identical
        // projections in a player's log, so this branch called it Mono, every consumer took ONE EYE
        // as the head, and the reticule was placed from the left eye one frame and the right eye the
        // next: the doubled reticule of 2026-09-30 and 2026-10-05, which survived v0.6.1's removal of
        // the flattening. So the shortcut now also needs the declared method to be 3, the backend
        // not to be AFW, and Extreme Compatibility off (it makes method 3 alternate on our own
        // backend too); everything else is decided by what the positions actually do, below.
        m = ViewMode::Mono;
    } else if (s_hist_n >= 8) {
        // A SINGLE VIEW IS NEVER MONO WHILE UEVR DECLARES AN ALTERNATING METHOD. The ring cannot see
        // the alternation when the two eyes nearly coincide -- a swing under kSwingGameCm, i.e. a
        // world scale below ~0.3, an inactive HMD, or the 0.01 our own cutscene collapse writes --
        // and used to answer Mono there. Consumers then took ONE eye as the head for the ~7 frames
        // the ring needs after the eyes separate again: a half-IPD hop of the reticule and markers
        // after every such cutscene on AFR/AFW (review, 2026-10-05). Averaging two consecutive
        // samples is exact when the eyes coincide, so Alternating costs nothing there.
        const bool alternating = popcount8(s_hist) >= kAltVotes;
        m = (alternating || declared_alternating) ? ViewMode::Alternating : ViewMode::Mono;
    } else {
        m = ViewMode::Unknown;
    }
    g_mode.store((int)m, std::memory_order_relaxed);
    g_diag_ring.store(s_hist, std::memory_order_relaxed);
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

void viewmode_set_shared_projection(bool shared) {
    g_shared_projection.store(shared, std::memory_order_relaxed);
}

bool viewmode_shared_projection() {
    return g_shared_projection.load(std::memory_order_relaxed);
}

void viewmode_set_backend_warps(bool warps) {
    g_backend_warps.store(warps, std::memory_order_relaxed);
}

bool viewmode_backend_warps() {
    return g_backend_warps.load(std::memory_order_relaxed);
}

void viewmode_set_extreme_compat(bool on) {
    g_extreme_compat.store(on, std::memory_order_relaxed);
}

bool viewmode_extreme_compat() {
    return g_extreme_compat.load(std::memory_order_relaxed);
}

bool viewmode_is_mono() {
    return viewmode_current() == ViewMode::Mono && viewmode_declared() == 3 &&
           viewmode_shared_projection() && !viewmode_backend_warps() && !viewmode_extreme_compat();
}

unsigned viewmode_samples() {
    return g_samples.load(std::memory_order_relaxed);
}

void viewmode_diag(unsigned* ring, float* swing_max_cm) {
    if (ring != nullptr)         *ring = g_diag_ring.load(std::memory_order_relaxed);
    if (swing_max_cm != nullptr) *swing_max_cm = g_diag_swing_max.exchange(0.0f, std::memory_order_relaxed);
}

const char* viewmode_name(ViewMode m) {
    switch (m) {
    case ViewMode::Stereo:      return "two views per frame (stereo)";
    case ViewMode::Alternating: return "one view per frame, alternating eyes (AFR)";
    case ViewMode::Mono:        return "one view per frame, the centre eye (mono)";
    default:                    return "one view per frame, verdict pending";
    }
}

const char* viewmode_method_name(int declared_method) {
    switch (declared_method) {
    case 0:  return "Native Stereo";
    case 1:  return "Synchronized Sequential";
    case 2:  return "Alternating/AFR";
    case 3:  return viewmode_backend_warps()
                        ? "Alternate Frame Warping on this AFW backend: the eyes render by turns, the other is warped"
                        : "Mono on the monofix backend";
    case -1: return "unreadable";
    default: return "unknown value";
    }
}

} // namespace halo
