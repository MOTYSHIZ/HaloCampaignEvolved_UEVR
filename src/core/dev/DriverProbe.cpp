// DriverProbe -- see the header for what it measures and the rules it obeys.

#include "core/dev/DriverProbe.hpp"

#include "ArmDriver.hpp"              // arm_driver_owns
#include "Config.hpp"
#include "Markers.hpp"                // g_cam_x/y/z: the game camera the stereo pre callback published
#include "MotionAimControl.hpp"       // g_menu_active, g_stick_mode_active
#include "Rig.hpp"                    // rig_tracked_component, fp_weapon_actor, g_rig_parent, call_ret_vec3
#include "UeObject.hpp"               // RIG_PARAM_BUF, make_fname
#include "WeaponCalib.hpp"            // calib_hold_active
#include "WeaponOffset.hpp"           // weapon_offset_current_class
#include "core/PalettePose.hpp"       // palette_pose_weapon_quat: the placement provider's published rotation
#include "core/XrDisplayTime.hpp"
#include "core/config/CfgRead.hpp"
#include "core/host/PluginState.hpp"
#include "uevr/API.hpp"

#include <Windows.h>
#include <Xinput.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <new>
#include <string>
#include <vector>

using uevr::API;

namespace halo {
namespace {

// ---- THE BOUNDS. A forgotten switch stops by itself.
constexpr uint32_t kRingCap   = 2048;              // rows per ring; the writer drains every 100 ms
constexpr uint32_t kRowCap    = 200000;            // rows per session
constexpr double   kTimeCapMs = 15.0 * 60000.0;    // ms per session
constexpr double   kIntentMaxAgeMs = 250.0;        // a driver's target older than this is not that driver's any more
constexpr float    kCmPerPaletteUnit = 304.8f;     // the palette's unit, the same map the placement writes with
constexpr int      kHookBins = 48;                 // pose hook histogram, quarter-octave bins of microseconds
constexpr double   kTransitionMs = 2000.0;         // a row this soon after a transition state or an identity change is flagged

enum OkBit : uint32_t {
    OK_HMD = 1u << 0, OK_AIM = 1u << 1, OK_OFF = 1u << 2, OK_SOCKP = 1u << 3, OK_SOCKQ = 1u << 4,
    OK_COMP = 1u << 5, OK_ACT = 1u << 6, OK_PAR = 1u << 7, OK_VIEW = 1u << 8, OK_INTENT = 1u << 9,
    OK_RW = 1u << 10, OK_G2H = 1u << 11, OK_G2O = 1u << 12,
};

double qpc_ms_per_count() {
    static const double f = [] {
        LARGE_INTEGER q{};
        return QueryPerformanceFrequency(&q) && q.QuadPart != 0 ? 1000.0 / (double)q.QuadPart : 0.0;
    }();
    return f;
}
long long qpc_now() { LARGE_INTEGER t{}; QueryPerformanceCounter(&t); return t.QuadPart; }

long long wall_unix_ms() {
    FILETIME ft{};
    GetSystemTimeAsFileTime(&ft);
    const unsigned long long t = ((unsigned long long)ft.dwHighDateTime << 32) | ft.dwLowDateTime;
    return (long long)(t / 10000ull) - 11644473600000ll;
}

// Room (x right, y up, -z forward, metres) to UE axes (x forward, y right, z up). The map is improper, so a
// rotation's axis maps as a pseudo vector: the same (-z, x, y, -w) the rest of the plugin uses.
Vec3 v_ue(const Vec3& v) { return Vec3{-v.z, v.x, v.y}; }
Quat q_ue(const Quat& q) { return Quat{-q.z, q.x, q.y, -q.w}; }

Quat q_norm(const Quat& q) {
    const float m = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!(m > 1.0e-6f)) return Quat{0.0f, 0.0f, 0.0f, 1.0f};
    return Quat{q.x / m, q.y / m, q.z / m, q.w / m};
}
float q_angle_deg(const Quat& q) {
    const float w = std::fabs(q.w) > 1.0f ? 1.0f : std::fabs(q.w);
    return 2.0f * std::acos(w) * 57.29578f;
}
float v_len(const Vec3& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }
Vec3 v_sub(const Vec3& a, const Vec3& b) { return Vec3{a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 v_add(const Vec3& a, const Vec3& b) { return Vec3{a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 v_mul(const Vec3& a, float s) { return Vec3{a.x * s, a.y * s, a.z * s}; }
bool v_finite(const Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

// ---- ONE ROW. Plain data: it crosses a lock-free ring to the writer thread.
struct Row {
    char      kind;            // 'T' engine tick, 'R' rendered frame
    uint32_t  seq, tick, tid;
    double    t_ms;
    long long wall_ms;
    int       drv_cfg, drv_active;
    char      wpn[48];
    uint32_t  rig_epoch, wpn_epoch;
    int       reacq;
    double    since_reacq_ms;
    int       moving;
    float     body_speed;      // cm/s
    unsigned  pad_buttons;
    int       pad_lx, pad_ly;
    int       calib;
    // THE CONTEXT A TRANSITION CANNOT HIDE BEHIND. A mission restart once read as twenty seconds of sprint lag.
    int       in_menu, cutscene, stick_mode, wpn_drawn, pad_sprint;
    uint32_t  pawn_epoch;
    int       transition;          // 1 = exclude by rule: a transition state now, or within kTransitionMs of one
    double    since_transition_ms;
    double    row_dt_ms;           // since the previous row of this kind
    float     sock_step_deg, sock_step_cm, handw_step_deg, handw_step_cm, aim_step_deg, aim_step_cm;
    uint32_t  ok;
    Vec3 hmd_p, aim_p, off_p, so;
    Quat hmd_q, aim_q, off_q, rotoff;
    Vec3 view_p, cam_p;  Quat view_q;
    float wtm_cb, scale;  int scale_src;  int eye0_left;
    float model_err_cm;        // R rows: the standing origin model's eye against the rendered eye
    Vec3 sock_p, comp_p, act_p, par_p;
    Quat sock_q, comp_q, act_q, par_q;
    int   int_src;  double int_age_ms;  Vec3 int_p;  Quat int_q;
    double rw_age_ms;
    Vec3 hand_w;  Quat hand_wq;
    Vec3 g2h;  Quat g2h_q;  float g2h_dist, g2h_ang;
    Vec3 g2o;  float g2o_dist, g2o_ang;
    float act_sock_cm, act_sock_deg;
    Vec3 d2i;  float d2i_dist, d2i_ang;
    Vec3 i2h;  float i2h_dist;
    // perf
    float  tick_dt_ms;  int perf_on;
    double his_tick_ms, his_palarm_ms;
    double feat_tick_in_ms, feat_tick_out_ms, wpn_tick_in_ms, wpn_tick_out_ms;
    double feat_sim_us, wpn_sim_us;
    int    hook_who;  uint32_t hook_n;  double hook_sum_us, hook_max_us;
    double frame_ms, stereo_cb_ms, feat_render_ms, wpn_render_ms, wpn_restamp_ms;
    uint32_t wpn_restamp_n;
    long long xr_display_ns;
    double probe_us;
};

struct Ring {
    Row*                  rows = nullptr;
    std::atomic<uint32_t> head{0}, tail{0};
    std::atomic<uint32_t> dropped{0};
    bool push(const Row& r) {
        const uint32_t h = head.load(std::memory_order_relaxed);
        if (h - tail.load(std::memory_order_acquire) >= kRingCap) { dropped.fetch_add(1, std::memory_order_relaxed); return false; }
        rows[h % kRingCap] = r;
        head.store(h + 1, std::memory_order_release);
        return true;
    }
};

// Per row kind, touched only by that kind's producer thread.
struct KindState {
    uint32_t seq = 0;
    void*    last_rig = nullptr;
    void*    last_wpn = nullptr;
    uint32_t rig_epoch = 0, wpn_epoch = 0;
    double   reacq_t_ms = -1.0;
    bool     have_prev = false;
    Vec3     prev_pos{};
    double   prev_t_ms = 0.0;
    void*    last_pawn = nullptr;
    uint32_t pawn_epoch = 0;
    double   last_bad_t_ms = -1.0e9;    // the last row a transition state or an identity change was seen in
    double   prev_row_t_ms = -1.0;
    bool     have_sock = false, have_handw = false, have_aim = false;
    Vec3     prev_sock_p{}, prev_handw_p{}, prev_aim_p{};
    Quat     prev_sock_q{0, 0, 0, 1}, prev_handw_q{0, 0, 0, 1}, prev_aim_q{0, 0, 0, 1};
};

// A value one thread publishes and another reads whole: sequence counter, odd while being written.
template <class T> struct SeqBox {
    std::atomic<uint32_t> seq{0};
    T                     v{};
    void put(const T& x) {
        const uint32_t s = seq.load(std::memory_order_relaxed);
        seq.store(s + 1, std::memory_order_release);
        v = x;
        seq.store(s + 2, std::memory_order_release);
    }
    bool get(T* out) const {
        for (int i = 0; i < 8; ++i) {
            const uint32_t a = seq.load(std::memory_order_acquire);
            if (a == 0) return false;
            if (a & 1u) continue;
            *out = v;
            if (seq.load(std::memory_order_acquire) == a) return true;
        }
        return false;
    }
};

struct RoomToWorld { Quat q; float scale; int scale_src; long long t_qpc; };
struct IntentMesh   { float x, y, z; long long t_qpc; };
struct IntentParent { Vec3 off; Quat q; long long t_qpc; };

struct Probe {
    Ring ring_t, ring_r;
    KindState ks_t, ks_r;
    FILE*     file = nullptr;
    char      path[MAX_PATH] = {0};
    HANDLE    thread = nullptr;
    std::atomic<bool> stop{false};
    long long t0_qpc = 0;
    std::atomic<uint32_t> rows_written{0};
    std::atomic<uint32_t> faults{0};
    std::atomic<uint32_t> tick{0};
    std::atomic<int>      drv_active{0};

    // game thread: the author's buckets as the tick just closed them
    double pend_tick_ms = 0.0, pend_palarm_ms = 0.0;
    bool   pend_perf_on = false;

    // stereo callback thread
    long long cb_t0 = 0, cb_acc = 0;
    long long frame_prev_qpc = 0;  double frame_ms = 0.0;
    bool   have_view = false;
    Vec3   view_p{};  Quat view_q{0, 0, 0, 1};  float wtm_cb = 0.0f;
    long long view_t_qpc = 0;
    float  scale = 0.0f;  int scale_src = 0;  int eye0_left = 1;
    float  model_err_cm = -1.0f;

    // across threads
    SeqBox<RoomToWorld>  rw;
    SeqBox<IntentMesh>   intent_mesh;
    SeqBox<IntentParent> intent_parent;
    std::atomic<unsigned> pad_buttons{0};
    std::atomic<int>      pad_lx{0}, pad_ly{0}, pad_sprint{0};
    std::atomic<uint32_t>  lane_wpn_n[PROBE_LANE_COUNT]{};
    std::atomic<long long> lane_all[PROBE_LANE_COUNT]{};
    std::atomic<long long> lane_wpn[PROBE_LANE_COUNT]{};
    std::atomic<uint32_t>  hook_n{0}, hook_max_us{0};
    std::atomic<uint64_t>  hook_sum_us{0};
    std::atomic<int>       hook_who{0};
    std::atomic<uint32_t>  hook_hist[kHookBins]{};
};

std::atomic<Probe*> s_probe{nullptr};
// Game thread only. A stopped session's state is freed a few seconds later, never at the stop itself: a
// hook on another thread may be inside a call that loaded the pointer a microsecond before it was cleared.
Probe*    s_retired = nullptr;
long long s_retired_qpc = 0;
bool      s_capped = false;       // this on-period hit a cap; stays stopped until the key returns to 0
bool      s_said_no_dir = false;

// ------------------------------------------------------------------------------------------ the CSV
// One emitter writes the header and the rows, so a column can never be named in one and missing in the other.
struct Emit {
    std::string* out;
    bool header;
    bool first = true;
    void sep() { if (!first) out->push_back(','); first = false; }
    void str(const char* name, const char* v) { sep(); out->append(header ? name : v); }
    void i(const char* name, long long v, bool valid = true) {
        sep();
        if (header) { out->append(name); return; }
        if (!valid) return;
        char b[32]; _snprintf_s(b, sizeof(b), _TRUNCATE, "%lld", v); out->append(b);
    }
    void f(const char* name, double v, int prec, bool valid = true) {
        sep();
        if (header) { out->append(name); return; }
        if (!valid || !std::isfinite(v)) return;
        char b[40]; _snprintf_s(b, sizeof(b), _TRUNCATE, "%.*f", prec, v); out->append(b);
    }
    void v3(const char* base, const char* unit, const Vec3& v, int prec, bool valid) {
        static const char* ax[3] = {"x", "y", "z"};
        const float a[3] = {v.x, v.y, v.z};
        for (int k = 0; k < 3; ++k) {
            char n[64]; _snprintf_s(n, sizeof(n), _TRUNCATE, "%s_%s_%s", base, ax[k], unit);
            f(n, a[k], prec, valid);
        }
    }
    void q4(const char* base, const Quat& q, bool valid) {
        static const char* ax[4] = {"qx", "qy", "qz", "qw"};
        const float a[4] = {q.x, q.y, q.z, q.w};
        for (int k = 0; k < 4; ++k) {
            char n[64]; _snprintf_s(n, sizeof(n), _TRUNCATE, "%s_%s", base, ax[k]);
            f(n, a[k], 6, valid);
        }
    }
};

void emit_row(Emit& e, const Row& r) {
    const bool T = r.kind == 'T', R = r.kind == 'R';
    const char kind[2] = {r.kind, 0};
    e.str("kind", kind);
    e.i("seq", r.seq);
    e.i("tick", r.tick);
    e.f("t_ms", r.t_ms, 3);
    e.i("wall_unix_ms", r.wall_ms);
    e.i("tid", r.tid);
    e.i("drv_cfg", r.drv_cfg);
    e.i("drv_active", r.drv_active);
    e.str("wpn", r.wpn[0] ? r.wpn : "-");
    e.i("rig_epoch", r.rig_epoch);
    e.i("wpn_epoch", r.wpn_epoch);
    e.i("reacq", r.reacq);
    e.f("since_reacq_ms", r.since_reacq_ms, 1, r.since_reacq_ms >= 0.0);
    e.i("moving", r.moving);
    e.f("body_speed_cms", r.body_speed, 1, r.body_speed >= 0.0f);
    e.i("pad_buttons", r.pad_buttons);
    e.i("pad_lx", r.pad_lx);
    e.i("pad_ly", r.pad_ly);
    e.i("pad_sprint_btn", r.pad_sprint);
    e.i("calib_hold", r.calib);
    e.i("in_menu", r.in_menu);
    e.i("cutscene", r.cutscene);
    e.i("stick_mode", r.stick_mode);
    e.i("wpn_drawn", r.wpn_drawn);
    e.i("pawn_epoch", r.pawn_epoch);
    e.i("transition", r.transition);
    e.f("since_transition_ms", r.since_transition_ms, 1, r.since_transition_ms < 1.0e8);
    e.f("row_dt_ms", r.row_dt_ms, 3, r.row_dt_ms >= 0.0);
    e.i("ok_bits", r.ok);
    // raw, room space (metres, UEVR tracking axes: x right, y up, -z forward)
    e.v3("hmd", "m", r.hmd_p, 5, (r.ok & OK_HMD) != 0);  e.q4("hmd", r.hmd_q, (r.ok & OK_HMD) != 0);
    e.v3("aim", "m", r.aim_p, 5, (r.ok & OK_AIM) != 0);  e.q4("aim", r.aim_q, (r.ok & OK_AIM) != 0);
    e.v3("off", "m", r.off_p, 5, (r.ok & OK_OFF) != 0);  e.q4("off", r.off_q, (r.ok & OK_OFF) != 0);
    e.v3("origin", "m", r.so, 5, true);
    e.q4("rotoff", r.rotoff, true);
    // raw, world (UE cm, UE quaternions)
    e.v3("view", "cm", r.view_p, 2, (r.ok & OK_VIEW) != 0);  e.q4("view", r.view_q, (r.ok & OK_VIEW) != 0);
    e.v3("cam", "cm", r.cam_p, 2, R);
    e.f("wtm_cb", r.wtm_cb, 3, R);
    e.f("scale_cm_per_m", r.scale, 3, r.scale > 0.0f);
    e.i("scale_src", r.scale_src, r.scale > 0.0f);
    e.i("eye0_left", r.eye0_left, R);
    e.f("model_err_cm", r.model_err_cm, 2, R && r.model_err_cm >= 0.0f);
    e.v3("sock", "cm", r.sock_p, 2, (r.ok & OK_SOCKP) != 0);  e.q4("sock", r.sock_q, (r.ok & OK_SOCKQ) != 0);
    e.v3("comp", "cm", r.comp_p, 2, (r.ok & OK_COMP) != 0);   e.q4("comp", r.comp_q, (r.ok & OK_COMP) != 0);
    e.v3("act", "cm", r.act_p, 2, (r.ok & OK_ACT) != 0);      e.q4("act", r.act_q, (r.ok & OK_ACT) != 0);
    e.v3("par", "cm", r.par_p, 2, (r.ok & OK_PAR) != 0);      e.q4("par", r.par_q, (r.ok & OK_PAR) != 0);
    e.i("int_src", r.int_src);
    e.f("int_age_ms", r.int_age_ms, 2, (r.ok & OK_INTENT) != 0);
    e.v3("int", "cm", r.int_p, 2, (r.ok & OK_INTENT) != 0);   e.q4("int", r.int_q, (r.ok & OK_INTENT) != 0);
    // derived (recomputable from the raw columns)
    const bool g = (r.ok & OK_G2H) != 0, o = (r.ok & OK_G2O) != 0, in = g && (r.ok & OK_INTENT) != 0;
    e.f("rw_age_ms", r.rw_age_ms, 2, T && (r.ok & OK_RW) != 0);
    e.v3("hand_w", "cm", r.hand_w, 2, g);  e.q4("hand_w", r.hand_wq, g);
    e.v3("g2h", "cm", r.g2h, 3, g);
    e.f("g2h_dist_cm", r.g2h_dist, 3, g);
    e.q4("g2h", r.g2h_q, g);
    e.f("g2h_ang_deg", r.g2h_ang, 3, g);
    e.v3("g2o", "cm", r.g2o, 3, o);
    e.f("g2o_dist_cm", r.g2o_dist, 3, o);
    e.f("g2o_ang_deg", r.g2o_ang, 3, o);
    // per-row steps, beside the values they come from: the hitch locator. Blank on the first valid row.
    e.f("sock_step_deg", r.sock_step_deg, 4, r.sock_step_deg >= 0.0f);
    e.f("sock_step_cm", r.sock_step_cm, 3, r.sock_step_cm >= 0.0f);
    e.f("handw_step_deg", r.handw_step_deg, 4, r.handw_step_deg >= 0.0f);
    e.f("handw_step_cm", r.handw_step_cm, 3, r.handw_step_cm >= 0.0f);
    e.f("aim_room_step_deg", r.aim_step_deg, 4, r.aim_step_deg >= 0.0f);
    e.f("aim_room_step_cm", r.aim_step_cm, 3, r.aim_step_cm >= 0.0f);
    e.f("act_sock_cm", r.act_sock_cm, 3, (r.ok & (OK_ACT | OK_SOCKP)) == (OK_ACT | OK_SOCKP));
    e.f("act_sock_deg", r.act_sock_deg, 3, (r.ok & (OK_ACT | OK_SOCKQ)) == (OK_ACT | OK_SOCKQ));
    e.v3("d2i", "cm", r.d2i, 3, in);
    e.f("d2i_dist_cm", r.d2i_dist, 3, in);
    e.f("d2i_ang_deg", r.d2i_ang, 3, in);
    e.v3("i2h", "cm", r.i2h, 3, in);
    e.f("i2h_dist_cm", r.i2h_dist, 3, in);
    // perf
    e.f("tick_dt_ms", r.tick_dt_ms, 3, T);
    e.i("perflog_on", r.perf_on, T);
    e.f("his_tick_all_ms", r.his_tick_ms, 4, T && r.perf_on);
    e.f("his_palettearm_ms", r.his_palarm_ms, 4, T && r.perf_on);
    e.f("feat_tick_in_ms", r.feat_tick_in_ms, 4, T);
    e.f("feat_tick_out_ms", r.feat_tick_out_ms, 4, T);
    e.f("wpn_tick_in_ms", r.wpn_tick_in_ms, 4, T);
    e.f("wpn_tick_out_ms", r.wpn_tick_out_ms, 4, T);
    e.f("feat_sim_us", r.feat_sim_us, 1, T);
    e.f("wpn_sim_us", r.wpn_sim_us, 1, T);
    e.i("hook_who", r.hook_who, T);
    e.i("hook_n", r.hook_n, T);
    e.f("hook_sum_us", r.hook_sum_us, 1, T);
    e.f("hook_max_us", r.hook_max_us, 1, T);
    e.f("frame_ms", r.frame_ms, 3, R && r.frame_ms > 0.0);
    e.f("stereo_cb_ms", r.stereo_cb_ms, 4, R);
    e.f("feat_render_ms", r.feat_render_ms, 4, R);
    e.f("wpn_render_ms", r.wpn_render_ms, 4, R);
    e.f("wpn_restamp_ms", r.wpn_restamp_ms, 4, R);
    e.i("wpn_restamp_n", r.wpn_restamp_n, R);
    e.i("xr_display_ns", r.xr_display_ns, R && r.xr_display_ns != 0);
    e.f("probe_us", r.probe_us, 1);
}

// ------------------------------------------------------------------------------------------ the summary
struct Stat { double mean = 0, p95 = 0, worst = 0; size_t n = 0; };
Stat stat_of(std::vector<double>& v) {
    Stat s; s.n = v.size();
    if (v.empty()) return s;
    double sum = 0; for (double x : v) sum += x;
    s.mean = sum / (double)v.size();
    std::sort(v.begin(), v.end());
    s.worst = v.back();
    s.p95 = v[(size_t)((v.size() - 1) * 0.95)];
    return s;
}

struct Window {
    std::vector<double> g2h_dist, g2h_x, g2h_y, g2h_z, d2i, act_sock, frame, cb, wpn_render,
                        his_tick, his_palarm, wpn_tick, feat_tick, hook_tick_us, tick_dt, probe_t, probe_r;
    std::vector<Quat>   g2h_q;
    uint32_t n_t = 0, n_r = 0;
    int drv = 0;  char wpn[48] = {0};
    void clear() {
        for (auto* v : {&g2h_dist, &g2h_x, &g2h_y, &g2h_z, &d2i, &act_sock, &frame, &cb, &wpn_render, &his_tick,
                        &his_palarm, &wpn_tick, &feat_tick, &hook_tick_us, &tick_dt, &probe_t, &probe_r}) v->clear();
        g2h_q.clear(); n_t = n_r = 0;
    }
};

double sd_of(const std::vector<double>& v) {
    if (v.size() < 2) return 0.0;
    double m = 0; for (double x : v) m += x; m /= (double)v.size();
    double s = 0; for (double x : v) s += (x - m) * (x - m);
    return std::sqrt(s / (double)(v.size() - 1));
}

void window_note(Window& w, const Row& r) {
    w.drv = r.drv_active;
    strncpy_s(w.wpn, sizeof(w.wpn), r.wpn, _TRUNCATE);
    if (r.kind == 'R') {
        ++w.n_r;
        if (r.ok & OK_G2H) {
            w.g2h_dist.push_back(r.g2h_dist); w.g2h_x.push_back(r.g2h.x); w.g2h_y.push_back(r.g2h.y);
            w.g2h_z.push_back(r.g2h.z); w.g2h_q.push_back(r.g2h_q);
            if (r.ok & OK_INTENT) w.d2i.push_back(r.d2i_dist);
        }
        if ((r.ok & (OK_ACT | OK_SOCKP)) == (OK_ACT | OK_SOCKP)) w.act_sock.push_back(r.act_sock_cm);
        if (r.frame_ms > 0.0) w.frame.push_back(r.frame_ms);
        w.cb.push_back(r.stereo_cb_ms);
        w.wpn_render.push_back(r.wpn_render_ms);
        w.probe_r.push_back(r.probe_us);
    } else {
        ++w.n_t;
        if (r.perf_on) { w.his_tick.push_back(r.his_tick_ms); w.his_palarm.push_back(r.his_palarm_ms); }
        w.wpn_tick.push_back(r.wpn_tick_in_ms + r.wpn_tick_out_ms);
        w.feat_tick.push_back(r.feat_tick_in_ms + r.feat_tick_out_ms);
        w.hook_tick_us.push_back(r.hook_sum_us);
        w.tick_dt.push_back(r.tick_dt_ms);
        w.probe_t.push_back(r.probe_us);
    }
}

// The rotation spread: every sample's angle to the window's mean rotation.
double rot_sd_deg(const std::vector<Quat>& qs) {
    if (qs.size() < 2) return 0.0;
    double sx = 0, sy = 0, sz = 0, sw = 0;
    for (const Quat& q : qs) {
        const float d = q.x * qs[0].x + q.y * qs[0].y + q.z * qs[0].z + q.w * qs[0].w;
        const float s = d < 0.0f ? -1.0f : 1.0f;
        sx += s * q.x; sy += s * q.y; sz += s * q.z; sw += s * q.w;
    }
    const Quat m = q_norm(Quat{(float)sx, (float)sy, (float)sz, (float)sw});
    double acc = 0;
    for (const Quat& q : qs) { const double a = q_angle_deg(quat_mul(quat_conj(m), q)); acc += a * a; }
    return std::sqrt(acc / (double)qs.size());
}

void window_report(Probe* p, Window& w, double span_ms) {
    if (w.n_t == 0 && w.n_r == 0) return;
    const double sec = span_ms > 1.0 ? span_ms / 1000.0 : 1.0;
    const double wob = std::sqrt(sd_of(w.g2h_x) * sd_of(w.g2h_x) + sd_of(w.g2h_y) * sd_of(w.g2h_y) + sd_of(w.g2h_z) * sd_of(w.g2h_z));
    const double rsd = rot_sd_deg(w.g2h_q);
    int over72 = 0, over90 = 0, over120 = 0;
    for (double f : w.frame) { if (f > 1000.0 / 72.0 * 1.10) ++over72; if (f > 1000.0 / 90.0 * 1.10) ++over90; if (f > 1000.0 / 120.0 * 1.10) ++over120; }
    // the pose hook's per-call p95, from the histogram the sim thread filled
    uint32_t hist[kHookBins]; uint64_t hn = 0;
    for (int b = 0; b < kHookBins; ++b) { hist[b] = p->hook_hist[b].exchange(0, std::memory_order_relaxed); hn += hist[b]; }
    double hook_p95 = 0.0;
    if (hn > 0) {
        uint64_t acc = 0; const uint64_t want = (uint64_t)((double)hn * 0.95);
        for (int b = 0; b < kHookBins; ++b) { acc += hist[b]; if (acc > want || b == kHookBins - 1) { hook_p95 = std::pow(2.0, (double)(b + 1) / 4.0); break; } }
    }
    Stat g = stat_of(w.g2h_dist), d = stat_of(w.d2i), as = stat_of(w.act_sock), fr = stat_of(w.frame), cb = stat_of(w.cb),
         wr = stat_of(w.wpn_render), ht = stat_of(w.his_tick), hp = stat_of(w.his_palarm), wt = stat_of(w.wpn_tick),
         ft = stat_of(w.feat_tick), hk = stat_of(w.hook_tick_us), td = stat_of(w.tick_dt), pt = stat_of(w.probe_t), pr = stat_of(w.probe_r);
    API::get()->log_info(
        "[Halo-CampE-UEVR] DRVPROBE drv=%d wpn=%s | rows R %u (%.1f Hz) T %u (%.1f Hz) | gun in hand frame (R rows): "
        "dist mean %.2f p95 %.2f worst %.2f cm, wobble sd %.2f cm, rot sd %.2f deg, n=%zu | drawn-intended mean %.2f worst %.2f cm n=%zu | "
        "actor-socket mean %.2f worst %.2f cm | ms mean/p95/worst: frame %.2f/%.2f/%.2f (over budget 72Hz %d 90Hz %d 120Hz %d of %zu) "
        "tick_dt %.2f/%.2f/%.2f his_tick %.3f/%.3f/%.3f his_palettearm %.3f/%.3f/%.3f (n=%zu, 0 = perflog off) "
        "wpn_slots_tick %.3f/%.3f/%.3f all_slots_tick %.3f/%.3f/%.3f stereo_cb %.3f/%.3f/%.3f wpn_slots_render %.3f/%.3f/%.3f | "
        "pose hook us per tick %.0f/%.0f/%.0f, per call p95 <%.0f us n=%llu | probe self us T %.0f/%.0f/%.0f R %.0f/%.0f/%.0f | "
        "written %u dropped T %u R %u faults %u",
        w.drv, w.wpn[0] ? w.wpn : "-", w.n_r, w.n_r / sec, w.n_t, w.n_t / sec,
        g.mean, g.p95, g.worst, wob, rsd, g.n, d.mean, d.worst, d.n, as.mean, as.worst,
        fr.mean, fr.p95, fr.worst, over72, over90, over120, fr.n,
        td.mean, td.p95, td.worst, ht.mean, ht.p95, ht.worst, hp.mean, hp.p95, hp.worst, ht.n,
        wt.mean, wt.p95, wt.worst, ft.mean, ft.p95, ft.worst, cb.mean, cb.p95, cb.worst, wr.mean, wr.p95, wr.worst,
        hk.mean, hk.p95, hk.worst, hook_p95, (unsigned long long)hn,
        pt.mean, pt.p95, pt.worst, pr.mean, pr.p95, pr.worst,
        p->rows_written.load(std::memory_order_relaxed), p->ring_t.dropped.load(std::memory_order_relaxed),
        p->ring_r.dropped.load(std::memory_order_relaxed), p->faults.load(std::memory_order_relaxed));
    w.clear();
}

// ------------------------------------------------------------------------------------------ the writer thread
void drain(Probe* p, Ring& ring, std::string& text, Window& w) {
    uint32_t t = ring.tail.load(std::memory_order_relaxed);
    const uint32_t h = ring.head.load(std::memory_order_acquire);
    while (t != h) {
        const Row& r = ring.rows[t % kRingCap];
        Emit e{&text, false};
        emit_row(e, r);
        text.append("\r\n");
        window_note(w, r);
        ++t;
        p->rows_written.fetch_add(1, std::memory_order_relaxed);
    }
    ring.tail.store(t, std::memory_order_release);
}

DWORD WINAPI writer_main(LPVOID arg) {
    Probe* p = static_cast<Probe*>(arg);
    std::string text;
    text.reserve(1u << 18);
    Window w;
    long long last_report = qpc_now();
    for (;;) {
        const bool stopping = p->stop.load(std::memory_order_acquire);
        if (!stopping) Sleep(100);
        text.clear();
        drain(p, p->ring_t, text, w);
        drain(p, p->ring_r, text, w);
        if (!text.empty() && p->file != nullptr) { fwrite(text.data(), 1, text.size(), p->file); fflush(p->file); }
        const long long now = qpc_now();
        const double span = (double)(now - last_report) * qpc_ms_per_count();
        if (span >= 1000.0 || stopping) { window_report(p, w, span); last_report = now; }
        if (stopping) break;
    }
    return 0;
}

// ------------------------------------------------------------------------------------------ start and stop (game thread)
void write_header(Probe* p) {
    std::string h;
    h += "# halo_vr driver probe. One file per session. Lines starting with # are comments; the next line names every column.\r\n";
    h += "# SAMPLING RATE OF THIS PROBE: kind=T is one row per engine tick (taken in on_post_engine_tick, after the frame was\r\n";
    h += "#   submitted); kind=R is one row per rendered frame (taken in the stereo view offset post callback, view index 0).\r\n";
    h += "#   Measured rates are in the DRVPROBE line of the plugin log each second, and follow from t_ms here. Nothing faster\r\n";
    h += "#   than a row interval can be seen. tid is the OS thread the row was taken on.\r\n";
    h += "# ONE SNAPSHOT: every input of an R row is read inside that one callback. A T row cannot read the rendered view, so its\r\n";
    h += "#   derived columns use the standing origin model (hand_w = par + Rrw * ue(aim - origin) * scale) with Rrw and scale\r\n";
    h += "#   from the last R row; rw_age_ms is how old that was. model_err_cm (R rows) is that model's eye against the rendered eye.\r\n";
    h += "# UNITS are in the column names. Room poses: metres, UEVR tracking axes (x right, y up, -z forward), raw API::VR::get_pose.\r\n";
    h += "#   World: UE cm, UE quaternions (x forward, y right, z up). Blank = not available in that row (see ok_bits).\r\n";
    h += "# DRAWN WEAPON: sock_* is the PrimaryWeapon socket of the posed first-person arms mesh (comp_*), the point the weapon actor\r\n";
    h += "#   (act_*) is attached at under every arm driver; par_* is the mesh's attach parent (the camera component).\r\n";
    h += "# g2h = the drawn socket in the AIM hand's own frame (x along the controller's forward, y its right, z its up); g2o the same\r\n";
    h += "#   in the OFF hand's frame. One to one tracking = g2h constant. d2i = drawn minus the driver's target, i2h = target minus\r\n";
    h += "#   hand, both in the aim hand's frame. int_src 2 = the palette arm driver's rig target, 3 = the placement's palette target.\r\n";
    h += "# TRANSITIONS: transition=1 marks a row to exclude by rule: in a menu, a cutscene, stick mode (vehicle, death), a weapon\r\n";
    h += "#   calibration hold, no first-person weapon drawn, no socket read, no aim hand (untracked or exactly zero while\r\n";
    h += "#   the XR session is unfocused), an identity change of the pawn, the arms mesh or the\r\n";
    h += "#   weapon actor (pawn_epoch, rig_epoch, wpn_epoch count them; reacq=1 on the row that saw it), or within 2000 ms of any.\r\n";
    h += "# STEPS: *_step_* is the change since the previous row of the same kind (row_dt_ms apart), beside the values themselves.\r\n";
    h += "# ok_bits: 1 hmd 2 aim 4 off 8 sock_pos 16 sock_rot 32 comp 64 actor 128 parent 256 view 512 intent 1024 rw 2048 g2h 4096 g2o\r\n";
    h += "# PERF: his_* are the author's PerfScope buckets for that tick (blank unless perflog=1). *_slots / feat_* = time inside\r\n";
    h += "#   feature hook slots; wpn_* = the slots of the feature that provides the palette pose. tick_in is inside his_tick_all,\r\n";
    h += "#   tick_out is not. hook_* = the first-person pose builder detour on the sim thread since the previous T row.\r\n";
    h += "#   wpn_restamp_* = the placer's render_refresh slot (its per-frame restamp), part of wpn_render_ms.\r\n";
    h += "#   frame_ms = time between rendered frames. xr_display_ns = the last OpenXR display time (its smallest step = one refresh).\r\n";
    h += "# CAPS: 200000 rows or 15 minutes per session.\r\n";
    Emit e{&h, true};
    Row dummy{};
    emit_row(e, dummy);
    h += "\r\n";
    fwrite(h.data(), 1, h.size(), p->file);
    fflush(p->file);
}

int active_driver_mode() {
    for (int m = 1; m <= 3; ++m)
        if (arm_driver_owns((ArmDriverMode)m)) return m;
    return 0;
}

void probe_start() {
    if (g_data_dir[0] == 0) {
        if (!s_said_no_dir) { s_said_no_dir = true; API::get()->log_info("[Halo-CampE-UEVR] DRVPROBE: no data folder yet, not started"); }
        return;
    }
    Probe* p = new (std::nothrow) Probe();
    if (p == nullptr) return;
    p->ring_t.rows = new (std::nothrow) Row[kRingCap];
    p->ring_r.rows = new (std::nothrow) Row[kRingCap];
    SYSTEMTIME st{};
    GetLocalTime(&st);
    const int drv = active_driver_mode();
    sprintf_s(p->path, sizeof(p->path), "%s\\drvprobe-%04u%02u%02u-%02u%02u%02u-drv%d.csv", g_data_dir,
              st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, drv);
    if (p->ring_t.rows == nullptr || p->ring_r.rows == nullptr || fopen_s(&p->file, p->path, "wb") != 0 || p->file == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] DRVPROBE: could not open %s, not started", p->path);
        delete[] p->ring_t.rows; delete[] p->ring_r.rows; delete p;
        s_capped = true;   // do not retry every tick; the key going to 0 re-arms
        return;
    }
    write_header(p);
    p->t0_qpc = qpc_now();
    p->drv_active.store(drv, std::memory_order_relaxed);
    p->thread = CreateThread(nullptr, 0, &writer_main, p, 0, nullptr);
    s_probe.store(p, std::memory_order_release);
    API::get()->log_info("[Halo-CampE-UEVR] DRVPROBE: started, armdriver cfg %d active %d, perflog %d (the author's buckets need "
                         "perflog=1), writing %s. T rows per engine tick, R rows per rendered frame; caps %u rows or %.0f min.",
                         g_cfg.arm_driver, drv, (int)g_cfg.perf_log, p->path, kRowCap, kTimeCapMs / 60000.0);
}

void probe_stop(const char* why) {
    Probe* p = s_probe.exchange(nullptr, std::memory_order_acquire);
    if (p == nullptr) return;
    p->stop.store(true, std::memory_order_release);
    if (p->thread != nullptr) { WaitForSingleObject(p->thread, 3000); CloseHandle(p->thread); p->thread = nullptr; }
    if (p->file != nullptr) { fclose(p->file); p->file = nullptr; }
    API::get()->log_info("[Halo-CampE-UEVR] DRVPROBE: stopped (%s), %u rows in %s", why,
                         p->rows_written.load(std::memory_order_relaxed), p->path);
    // Freed later: see s_retired. A second stop inside the grace frees the older one now, which is as old as it needs to be.
    if (s_retired != nullptr) { delete[] s_retired->ring_t.rows; delete[] s_retired->ring_r.rows; delete s_retired; }
    s_retired = p;
    s_retired_qpc = qpc_now();
}

// ------------------------------------------------------------------------------------------ the sample
bool raw_pose(UEVR_TrackedDeviceIndex idx, Vec3* pos, Quat* rot) {
    if (idx < 0) return false;
    const auto pose = API::VR::get_pose(idx);
    const auto& q = pose.rotation;
    const float m2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (m2 < 0.9f || m2 > 1.1f) return false;   // UEVR's placeholder before tracking is live
    *pos = Vec3{pose.position.x, pose.position.y, pose.position.z};
    *rot = Quat{q.x, q.y, q.z, q.w};
    return v_finite(*pos);
}

bool socket_rot(API::UObject* comp, const wchar_t* socket, Vec3* out_pyr) {
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    API::FName name = make_fname(socket);
    memcpy(params, &name, sizeof(int32_t) * 2);
    comp->call_function(L"GetSocketRotation", params);
    auto* d = reinterpret_cast<double*>(params + 8);
    if (!std::isfinite(d[0]) || !std::isfinite(d[1]) || !std::isfinite(d[2])) return false;
    *out_pyr = Vec3{(float)d[0], (float)d[1], (float)d[2]};
    return true;
}

struct WorldRead {
    void* rig; void* wpn; void* pawn;
    bool  drawn;
    uint32_t ok;
    Vec3 sock_p, comp_p, act_p, par_p;
    Vec3 sock_r, comp_r, act_r, par_r;   // rotators (pitch, yaw, roll)
};

// Every reflected read of one row, in one place and under SEH: these objects belong to a level that can be
// torn down under us, and a fault in an instrument must cost a row, never the game. Plain data only in here.
bool read_world_guarded(WorldRead* w) {
    __try {
        auto* rig = rig_tracked_component();
        auto* act = fp_weapon_actor();
        auto* par = g_rig_parent;
        w->rig = rig; w->wpn = act;
        w->pawn = API::get()->get_local_pawn(0);
        w->drawn = fp_weapon_route_alive();
        if (rig != nullptr) {
            if (call_socket_location(rig, L"PrimaryWeapon", &w->sock_p) && v_finite(w->sock_p)) w->ok |= OK_SOCKP;
            if (socket_rot(rig, L"PrimaryWeapon", &w->sock_r)) w->ok |= OK_SOCKQ;
            if (call_ret_vec3(rig, L"K2_GetComponentLocation", &w->comp_p) &&
                call_ret_vec3(rig, L"K2_GetComponentRotation", &w->comp_r)) w->ok |= OK_COMP;
        }
        if (act != nullptr) {
            if (call_ret_vec3(act, L"K2_GetActorLocation", &w->act_p) &&
                call_ret_vec3(act, L"K2_GetActorRotation", &w->act_r)) w->ok |= OK_ACT;
        }
        if (par != nullptr) {
            if (call_ret_vec3(par, L"K2_GetComponentLocation", &w->par_p) &&
                call_ret_vec3(par, L"K2_GetComponentRotation", &w->par_r)) w->ok |= OK_PAR;
        }
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        w->ok = 0;
        return false;
    }
}

Quat rot_q(const Vec3& pyr) { return rotator_to_quat(pyr.x, pyr.y, pyr.z); }

void copy_weapon_class(char* out, size_t cap) {
    const char* c = weapon_offset_current_class();
    size_t i = 0;
    if (c != nullptr)
        for (; i + 1 < cap && c[i] != 0; ++i) out[i] = (c[i] > 32 && c[i] < 127 && c[i] != ',') ? c[i] : '_';
    out[i] = 0;
}

void take_sample(Probe* p, char kind) {
    CFG_HOOK_READ;   // the R row may be taken off the game thread: see core/config/CfgRead.hpp
    const long long q0 = qpc_now();
    KindState& ks = (kind == 'T') ? p->ks_t : p->ks_r;
    Row r{};
    r.kind = kind;
    r.seq = ks.seq++;
    r.tick = p->tick.load(std::memory_order_relaxed);
    r.tid = (uint32_t)GetCurrentThreadId();
    r.t_ms = (double)(q0 - p->t0_qpc) * qpc_ms_per_count();
    r.wall_ms = wall_unix_ms();
    r.drv_active = p->drv_active.load(std::memory_order_relaxed);
    r.since_reacq_ms = -1.0;
    r.body_speed = -1.0f;
    r.model_err_cm = -1.0f;
    r.sock_step_deg = r.sock_step_cm = r.handw_step_deg = r.handw_step_cm = r.aim_step_deg = r.aim_step_cm = -1.0f;
    r.hand_wq = r.g2h_q = r.int_q = Quat{0, 0, 0, 1};

    // ---- the poses: UEVR's own, raw. Never a driver's latched or filtered copy.
    const auto li = API::VR::get_left_controller_index(), ri = API::VR::get_right_controller_index();
    const bool aim_left = g_cfg.aim_left_hand;
    if (raw_pose(API::VR::get_hmd_index(), &r.hmd_p, &r.hmd_q)) r.ok |= OK_HMD;
    if (raw_pose(aim_left ? li : ri, &r.aim_p, &r.aim_q)) r.ok |= OK_AIM;
    if (raw_pose(aim_left ? ri : li, &r.off_p, &r.off_q)) r.ok |= OK_OFF;
    { const auto so = API::VR::get_standing_origin(); r.so = Vec3{so.x, so.y, so.z}; }
    { const auto ro = API::VR::get_rotation_offset(); r.rotoff = Quat{ro.x, ro.y, ro.z, ro.w}; }

    // ---- the world, same instant
    WorldRead w{};
    if (!read_world_guarded(&w)) p->faults.fetch_add(1, std::memory_order_relaxed);
    r.ok |= w.ok;
    r.sock_p = w.sock_p; r.comp_p = w.comp_p; r.act_p = w.act_p; r.par_p = w.par_p;
    r.sock_q = rot_q(w.sock_r); r.comp_q = rot_q(w.comp_r); r.act_q = rot_q(w.act_r); r.par_q = rot_q(w.par_r);

    // ---- tags
    r.drv_cfg = g_cfg.arm_driver;
    copy_weapon_class(r.wpn, sizeof(r.wpn));
    if (w.rig != ks.last_rig) { ks.last_rig = w.rig; ++ks.rig_epoch; r.reacq = 1; }
    if (w.wpn != ks.last_wpn) { ks.last_wpn = w.wpn; ++ks.wpn_epoch; r.reacq = 1; }
    if (r.reacq && w.rig != nullptr && w.wpn != nullptr) ks.reacq_t_ms = r.t_ms;
    r.rig_epoch = ks.rig_epoch; r.wpn_epoch = ks.wpn_epoch;
    if (ks.reacq_t_ms >= 0.0) r.since_reacq_ms = r.t_ms - ks.reacq_t_ms;
    r.pad_buttons = p->pad_buttons.load(std::memory_order_relaxed);
    r.pad_lx = p->pad_lx.load(std::memory_order_relaxed);
    r.pad_ly = p->pad_ly.load(std::memory_order_relaxed);
    r.pad_sprint = p->pad_sprint.load(std::memory_order_relaxed);
    r.calib = calib_hold_active() ? 1 : 0;
    r.in_menu = g_menu_active.load(std::memory_order_relaxed) ? 1 : 0;
    r.cutscene = host::g_plugin_state.cut2d_engaged->load(std::memory_order_relaxed) ? 1 : 0;
    r.stick_mode = g_stick_mode_active.load(std::memory_order_relaxed) ? 1 : 0;
    r.wpn_drawn = w.drawn ? 1 : 0;
    if (w.pawn != ks.last_pawn) { ks.last_pawn = w.pawn; ++ks.pawn_epoch; r.reacq = 1; }
    r.pawn_epoch = ks.pawn_epoch;
    {
        // NO HAND, NO MEASUREMENT. When the XR session loses focus (a headset dashboard, the headset
        // lifted) the runtime keeps answering pose queries with EXACTLY zero translation. The drivers
        // correctly stop carrying the gun then, and these rows measured a hand that was not there:
        // 18 s of them turned a steady run's still p90 from ~10 cm into 68 cm (2026-09-21).
        const bool no_hand = (r.ok & OK_AIM) == 0
                          || (r.aim_p.x == 0.0f && r.aim_p.y == 0.0f && r.aim_p.z == 0.0f);
        const bool bad = r.reacq || r.in_menu || r.cutscene || r.stick_mode || !r.wpn_drawn || r.calib || no_hand
                      || (r.ok & (OK_SOCKP | OK_SOCKQ)) != (OK_SOCKP | OK_SOCKQ);
        if (bad) ks.last_bad_t_ms = r.t_ms;
        r.since_transition_ms = r.t_ms - ks.last_bad_t_ms;
        r.transition = (bad || r.since_transition_ms < kTransitionMs) ? 1 : 0;
    }
    r.row_dt_ms = ks.prev_row_t_ms >= 0.0 ? r.t_ms - ks.prev_row_t_ms : -1.0;
    ks.prev_row_t_ms = r.t_ms;

    // ---- the view (R) or the standing origin model (T)
    bool have_hand_w = false;
    if (kind == 'R') {
        r.cam_p = Vec3{g_cam_x.load(std::memory_order_relaxed), g_cam_y.load(std::memory_order_relaxed), g_cam_z.load(std::memory_order_relaxed)};
        r.wtm_cb = p->wtm_cb;
        r.eye0_left = p->eye0_left;
        if (p->scale > 0.0f)      { r.scale = p->scale;  r.scale_src = 2; }
        else if (p->wtm_cb > 1.0f){ r.scale = p->wtm_cb; r.scale_src = 1; }
        else                      { r.scale = 100.0f;    r.scale_src = 0; }
        if (p->have_view) {
            r.ok |= OK_VIEW;
            r.view_p = p->view_p; r.view_q = p->view_q;
            if ((r.ok & (OK_HMD | OK_AIM)) == (OK_HMD | OK_AIM)) {
                // The view frame IS the head frame: the hand relative to the head, in the head's axes, carried
                // into the world by the finished view. No standing origin, no yaw bookkeeping, no driver state.
                const auto eo = API::VR::get_eye_offset(r.eye0_left ? API::VR::Eye::LEFT : API::VR::Eye::RIGHT);
                const Quat hinv = quat_conj(r.hmd_q);
                const Vec3 hl = quat_rotate(hinv, v_sub(r.aim_p, r.hmd_p));
                const Vec3 ph = v_sub(v_ue(hl), v_ue(Vec3{eo.x, eo.y, eo.z}));
                const Quat qh = q_ue(quat_mul(hinv, r.aim_q));
                r.hand_w  = v_add(r.view_p, quat_rotate(r.view_q, v_mul(ph, r.scale)));
                r.hand_wq = q_norm(quat_mul(r.view_q, qh));
                have_hand_w = true;
                // Publish the room to world rotation for the T rows, and check the model they will use.
                RoomToWorld rw{};
                rw.q = q_norm(quat_mul(r.view_q, quat_conj(q_ue(r.hmd_q))));
                rw.scale = r.scale; rw.scale_src = r.scale_src; rw.t_qpc = q0;
                p->rw.put(rw);
                const Vec3 eye_room = v_add(v_sub(r.hmd_p, r.so), quat_rotate(r.hmd_q, Vec3{eo.x, eo.y, eo.z}));
                const Vec3 model = v_add(r.cam_p, quat_rotate(rw.q, v_mul(v_ue(eye_room), r.scale)));
                r.model_err_cm = v_len(v_sub(model, r.view_p));
            }
        }
        if (ks.have_prev && r.t_ms > ks.prev_t_ms) r.body_speed = v_len(v_sub(r.cam_p, ks.prev_pos)) / (float)((r.t_ms - ks.prev_t_ms) / 1000.0);
        ks.prev_pos = r.cam_p; ks.prev_t_ms = r.t_ms; ks.have_prev = true;
    } else {
        RoomToWorld rw{};
        if (p->rw.get(&rw)) {
            r.ok |= OK_RW;
            r.scale = rw.scale; r.scale_src = rw.scale_src;
            r.rw_age_ms = (double)(q0 - rw.t_qpc) * qpc_ms_per_count();
            if ((r.ok & (OK_AIM | OK_PAR)) == (OK_AIM | OK_PAR)) {
                r.hand_w  = v_add(r.par_p, quat_rotate(rw.q, v_mul(v_ue(v_sub(r.aim_p, r.so)), rw.scale)));
                r.hand_wq = q_norm(quat_mul(rw.q, q_ue(r.aim_q)));
                have_hand_w = true;
            }
        }
        if (r.ok & OK_PAR) {
            if (ks.have_prev && r.t_ms > ks.prev_t_ms) r.body_speed = v_len(v_sub(r.par_p, ks.prev_pos)) / (float)((r.t_ms - ks.prev_t_ms) / 1000.0);
            ks.prev_pos = r.par_p; ks.prev_t_ms = r.t_ms; ks.have_prev = true;
        } else {
            ks.have_prev = false;
        }
    }
    r.moving = (r.body_speed > 10.0f) ? 1 : 0;

    // ---- the target the running driver computed
    {
        IntentMesh im{}; IntentParent ip{};
        const bool hm = p->intent_mesh.get(&im), hp = p->intent_parent.get(&ip);
        const double am = hm ? (double)(q0 - im.t_qpc) * qpc_ms_per_count() : 1.0e9;
        const double ap = hp ? (double)(q0 - ip.t_qpc) * qpc_ms_per_count() : 1.0e9;
        if (am <= kIntentMaxAgeMs && am <= ap && (r.ok & (OK_PAR | OK_COMP)) == (OK_PAR | OK_COMP)) {
            // Palette space to world exactly as the socket itself composes: the attach parent's origin, the mesh's rotation.
            const Vec3 local{im.x * kCmPerPaletteUnit, -im.y * kCmPerPaletteUnit, im.z * kCmPerPaletteUnit};
            r.int_p = v_add(r.par_p, quat_rotate(r.comp_q, local));
            Quat wq{};
            r.int_q = palette_pose_weapon_quat(&wq) ? wq : Quat{0, 0, 0, 1};
            r.int_src = 3; r.int_age_ms = am; r.ok |= OK_INTENT;
        } else if (ap <= kIntentMaxAgeMs && (r.ok & OK_PAR) != 0) {
            r.int_p = v_add(r.par_p, ip.off);
            r.int_q = ip.q;
            r.int_src = 2; r.int_age_ms = ap; r.ok |= OK_INTENT;
        }
    }

    // ---- the drawn weapon in the hand's own frame
    if (have_hand_w && (r.ok & (OK_SOCKP | OK_SOCKQ)) == (OK_SOCKP | OK_SOCKQ)) {
        const Quat hi = quat_conj(r.hand_wq);
        r.g2h = quat_rotate(hi, v_sub(r.sock_p, r.hand_w));
        r.g2h_dist = v_len(r.g2h);
        r.g2h_q = q_norm(quat_mul(hi, r.sock_q));
        r.g2h_ang = q_angle_deg(r.g2h_q);
        r.ok |= OK_G2H;
        if (r.ok & OK_INTENT) {
            r.d2i = quat_rotate(hi, v_sub(r.sock_p, r.int_p));
            r.d2i_dist = v_len(r.d2i);
            r.d2i_ang = q_angle_deg(quat_mul(quat_conj(r.int_q), r.sock_q));
            r.i2h = quat_rotate(hi, v_sub(r.int_p, r.hand_w));
            r.i2h_dist = v_len(r.i2h);
        }
        if (r.ok & OK_OFF) {
            // The off hand through the same room to world map as the aim hand: world = hand_w + Rrw * ue(off - aim) * scale.
            const Quat rwq = q_norm(quat_mul(r.hand_wq, quat_conj(q_ue(r.aim_q))));
            const Vec3 off_w = v_add(r.hand_w, quat_rotate(rwq, v_mul(v_ue(v_sub(r.off_p, r.aim_p)), r.scale)));
            const Quat off_wq = q_norm(quat_mul(rwq, q_ue(r.off_q)));
            const Quat oi = quat_conj(off_wq);
            r.g2o = quat_rotate(oi, v_sub(r.sock_p, off_w));
            r.g2o_dist = v_len(r.g2o);
            r.g2o_ang = q_angle_deg(quat_mul(oi, r.sock_q));
            r.ok |= OK_G2O;
        }
    }
    if ((r.ok & (OK_ACT | OK_SOCKP)) == (OK_ACT | OK_SOCKP)) r.act_sock_cm = v_len(v_sub(r.act_p, r.sock_p));
    if ((r.ok & (OK_ACT | OK_SOCKQ)) == (OK_ACT | OK_SOCKQ)) r.act_sock_deg = q_angle_deg(quat_mul(quat_conj(r.sock_q), r.act_q));

    // ---- the per-row steps of the drawn gun and of the hand, from the values above
    if ((r.ok & (OK_SOCKP | OK_SOCKQ)) == (OK_SOCKP | OK_SOCKQ)) {
        if (ks.have_sock) {
            r.sock_step_deg = q_angle_deg(quat_mul(quat_conj(ks.prev_sock_q), r.sock_q));
            r.sock_step_cm = v_len(v_sub(r.sock_p, ks.prev_sock_p));
        }
        ks.prev_sock_q = r.sock_q; ks.prev_sock_p = r.sock_p; ks.have_sock = true;
    } else ks.have_sock = false;
    if (have_hand_w) {
        if (ks.have_handw) {
            r.handw_step_deg = q_angle_deg(quat_mul(quat_conj(ks.prev_handw_q), r.hand_wq));
            r.handw_step_cm = v_len(v_sub(r.hand_w, ks.prev_handw_p));
        }
        ks.prev_handw_q = r.hand_wq; ks.prev_handw_p = r.hand_w; ks.have_handw = true;
    } else ks.have_handw = false;
    if (r.ok & OK_AIM) {
        if (ks.have_aim) {
            r.aim_step_deg = q_angle_deg(quat_mul(quat_conj(ks.prev_aim_q), r.aim_q));
            r.aim_step_cm = v_len(v_sub(r.aim_p, ks.prev_aim_p)) * 100.0f;
        }
        ks.prev_aim_q = r.aim_q; ks.prev_aim_p = r.aim_p; ks.have_aim = true;
    } else ks.have_aim = false;

    // ---- perf
    const double k = qpc_ms_per_count();
    if (kind == 'T') {
        r.tick_dt_ms = host::g_plugin_state.last_dt->load(std::memory_order_relaxed) * 1000.0f;
        r.perf_on = p->pend_perf_on ? 1 : 0;
        r.his_tick_ms = p->pend_tick_ms; r.his_palarm_ms = p->pend_palarm_ms;
        r.feat_tick_in_ms  = (double)p->lane_all[PROBE_LANE_TICK_IN].exchange(0, std::memory_order_relaxed) * k;
        r.feat_tick_out_ms = (double)p->lane_all[PROBE_LANE_TICK_OUT].exchange(0, std::memory_order_relaxed) * k;
        r.wpn_tick_in_ms   = (double)p->lane_wpn[PROBE_LANE_TICK_IN].exchange(0, std::memory_order_relaxed) * k;
        r.wpn_tick_out_ms  = (double)p->lane_wpn[PROBE_LANE_TICK_OUT].exchange(0, std::memory_order_relaxed) * k;
        r.feat_sim_us      = (double)p->lane_all[PROBE_LANE_SIM].exchange(0, std::memory_order_relaxed) * k * 1000.0;
        r.wpn_sim_us       = (double)p->lane_wpn[PROBE_LANE_SIM].exchange(0, std::memory_order_relaxed) * k * 1000.0;
        r.hook_who = p->hook_who.load(std::memory_order_relaxed);
        r.hook_n = p->hook_n.exchange(0, std::memory_order_relaxed);
        r.hook_sum_us = (double)p->hook_sum_us.exchange(0, std::memory_order_relaxed);
        r.hook_max_us = (double)p->hook_max_us.exchange(0, std::memory_order_relaxed);
    } else {
        r.frame_ms = p->frame_ms;
        r.stereo_cb_ms = (double)p->cb_acc * k;  p->cb_acc = 0;
        r.wpn_restamp_ms = (double)p->lane_wpn[PROBE_LANE_RESTAMP].exchange(0, std::memory_order_relaxed) * k;
        r.wpn_restamp_n  = p->lane_wpn_n[PROBE_LANE_RESTAMP].exchange(0, std::memory_order_relaxed);
        r.feat_render_ms = (double)(p->lane_all[PROBE_LANE_RENDER].exchange(0, std::memory_order_relaxed)
                                  + p->lane_all[PROBE_LANE_RESTAMP].exchange(0, std::memory_order_relaxed)) * k;
        r.wpn_render_ms  = (double)p->lane_wpn[PROBE_LANE_RENDER].exchange(0, std::memory_order_relaxed) * k + r.wpn_restamp_ms;
        r.xr_display_ns = xr_display_time();
    }
    r.probe_us = (double)(qpc_now() - q0) * k * 1000.0;
    ((kind == 'T') ? p->ring_t : p->ring_r).push(r);
}

} // namespace

// ================================================================================================ entry points
void driver_probe_engine_tick_end() {
    Probe* p = s_probe.load(std::memory_order_acquire);
    const bool want = g_cfg.driver_probe != 0;
    if (p == nullptr && !want) {
        s_capped = false;
        if (s_retired != nullptr && (double)(qpc_now() - s_retired_qpc) * qpc_ms_per_count() > 5000.0) {
            delete[] s_retired->ring_t.rows; delete[] s_retired->ring_r.rows; delete s_retired;
            s_retired = nullptr;
        }
        return;
    }
    if (p == nullptr) {
        if (!s_capped) probe_start();
        return;
    }
    if (!want) { probe_stop("driverprobe=0"); return; }
    const double age = (double)(qpc_now() - p->t0_qpc) * qpc_ms_per_count();
    if (age > kTimeCapMs || p->rows_written.load(std::memory_order_relaxed) >= kRowCap || p->faults.load(std::memory_order_relaxed) >= 8) {
        s_capped = true;
        probe_stop(p->faults.load(std::memory_order_relaxed) >= 8 ? "8 faulted reads" : (age > kTimeCapMs ? "time cap" : "row cap"));
        return;
    }
    p->tick.fetch_add(1, std::memory_order_relaxed);
    p->drv_active.store(active_driver_mode(), std::memory_order_relaxed);
    // The author's buckets, exactly as his own hitch report reads them: this tick's finished totals.
    const auto& ps = host::g_plugin_state;
    p->pend_perf_on = g_cfg.perf_log;
    p->pend_tick_ms   = (ps.perf_now != nullptr && ps.perf_tick   >= 0 && ps.perf_tick   < ps.perf_count) ? ps.perf_now[ps.perf_tick]   : 0.0;
    p->pend_palarm_ms = (ps.perf_now != nullptr && ps.perf_palarm >= 0 && ps.perf_palarm < ps.perf_count) ? ps.perf_now[ps.perf_palarm] : 0.0;
}

void driver_probe_post_engine_tick() {
    Probe* p = s_probe.load(std::memory_order_acquire);
    if (p == nullptr) return;
    take_sample(p, 'T');
}

void driver_probe_stereo_begin() {
    Probe* p = s_probe.load(std::memory_order_acquire);
    if (p == nullptr) return;
    p->cb_t0 = qpc_now();
}

void driver_probe_stereo_end() {
    Probe* p = s_probe.load(std::memory_order_acquire);
    if (p == nullptr || p->cb_t0 == 0) return;
    p->cb_acc += qpc_now() - p->cb_t0;
    p->cb_t0 = 0;
}

void driver_probe_post_view(int index, float world_to_meters, const UEVR_Vector3f* position,
                            const UEVR_Rotatorf* rotation, bool is_double) {
    Probe* p = s_probe.load(std::memory_order_acquire);
    if (p == nullptr || position == nullptr || rotation == nullptr) return;
    Vec3 pos{}; Vec3 pyr{};
    if (is_double) {
        const auto* dp = reinterpret_cast<const UEVR_Vector3d*>(position);
        const auto* dr = reinterpret_cast<const UEVR_Rotatord*>(rotation);
        pos = Vec3{(float)dp->x, (float)dp->y, (float)dp->z};
        pyr = Vec3{(float)dr->pitch, (float)dr->yaw, (float)dr->roll};
    } else {
        pos = Vec3{position->x, position->y, position->z};
        pyr = Vec3{rotation->pitch, rotation->yaw, rotation->roll};
    }
    if (!v_finite(pos) || !v_finite(pyr)) return;
    const long long now = qpc_now();
    if (index == 0) {
        if (p->frame_prev_qpc != 0) p->frame_ms = (double)(now - p->frame_prev_qpc) * qpc_ms_per_count();
        p->frame_prev_qpc = now;
        p->view_p = pos; p->view_q = rot_q(pyr); p->wtm_cb = world_to_meters;
        p->view_t_qpc = now; p->have_view = true;
    } else if (index == 1 && p->have_view && (double)(now - p->view_t_qpc) * qpc_ms_per_count() < 3.0) {
        // Both eyes of ONE frame: their separation against the runtime's own eye offsets is the rendered scale, and
        // its direction says which eye view index 0 is. Measured, because it is what the derived columns rest on.
        const auto l = API::VR::get_eye_offset(API::VR::Eye::LEFT), rr = API::VR::get_eye_offset(API::VR::Eye::RIGHT);
        const float ipd = v_len(Vec3{rr.x - l.x, rr.y - l.y, rr.z - l.z});
        const Vec3 sep = v_sub(pos, p->view_p);
        const float s = v_len(sep);
        if (ipd > 0.03f && ipd < 0.09f && s > 0.5f) {
            const float sc = s / ipd;
            p->scale = (p->scale > 0.0f) ? p->scale + 0.1f * (sc - p->scale) : sc;
            const Vec3 right = quat_rotate(p->view_q, Vec3{0.0f, 1.0f, 0.0f});
            p->eye0_left = (sep.x * right.x + sep.y * right.y + sep.z * right.z) >= 0.0f ? 1 : 0;
        }
    }
}

void driver_probe_render_sample(int index) {
    Probe* p = s_probe.load(std::memory_order_acquire);
    if (p == nullptr || index != 0) return;
    take_sample(p, 'R');
}

long long driver_probe_clock() {
    return s_probe.load(std::memory_order_acquire) != nullptr ? qpc_now() : 0;
}

void driver_probe_pose_hook_done(long long t0, int who) {
    if (t0 == 0) return;
    Probe* p = s_probe.load(std::memory_order_acquire);
    if (p == nullptr) return;
    const double us = (double)(qpc_now() - t0) * qpc_ms_per_count() * 1000.0;
    const uint32_t u = (uint32_t)(us < 0.0 ? 0.0 : (us > 4.0e9 ? 4.0e9 : us));
    p->hook_who.store(who, std::memory_order_relaxed);
    p->hook_n.fetch_add(1, std::memory_order_relaxed);
    p->hook_sum_us.fetch_add(u, std::memory_order_relaxed);
    uint32_t prev = p->hook_max_us.load(std::memory_order_relaxed);
    while (u > prev && !p->hook_max_us.compare_exchange_weak(prev, u, std::memory_order_relaxed)) {}
    int bin = (int)(4.0 * std::log2((double)u + 1.0));
    if (bin < 0) bin = 0;
    if (bin >= kHookBins) bin = kHookBins - 1;
    p->hook_hist[bin].fetch_add(1, std::memory_order_relaxed);
}

void driver_probe_slot_done(long long t0, bool pose_provider, int lane) {
    if (t0 == 0 || lane < 0 || lane >= PROBE_LANE_COUNT) return;
    Probe* p = s_probe.load(std::memory_order_acquire);
    if (p == nullptr) return;
    const long long d = qpc_now() - t0;
    p->lane_all[lane].fetch_add(d, std::memory_order_relaxed);
    if (pose_provider) {
        p->lane_wpn[lane].fetch_add(d, std::memory_order_relaxed);
        p->lane_wpn_n[lane].fetch_add(1, std::memory_order_relaxed);
    }
}

void driver_probe_note_intent_mesh(float x_u, float y_u, float z_u) {
    Probe* p = s_probe.load(std::memory_order_acquire);
    if (p == nullptr) return;
    if (!std::isfinite(x_u) || !std::isfinite(y_u) || !std::isfinite(z_u)) return;
    p->intent_mesh.put(IntentMesh{x_u, y_u, z_u, qpc_now()});
}

void driver_probe_note_intent_parent(bool valid, const Vec3& off_world_cm, const Quat& q_mesh) {
    Probe* p = s_probe.load(std::memory_order_acquire);
    if (p == nullptr || !valid || !v_finite(off_world_cm)) return;
    p->intent_parent.put(IntentParent{off_world_cm, q_mesh, qpc_now()});
}

void driver_probe_note_pad(const _XINPUT_STATE* state) {
    Probe* p = s_probe.load(std::memory_order_acquire);
    if (p == nullptr || state == nullptr) return;
    CFG_HOOK_READ;   // the XInput hook's thread
    p->pad_sprint.store((g_cfg.pa_sprint_mask != 0 && (state->Gamepad.wButtons & (WORD)g_cfg.pa_sprint_mask) != 0) ? 1 : 0,
                        std::memory_order_relaxed);
    p->pad_buttons.store(state->Gamepad.wButtons, std::memory_order_relaxed);
    p->pad_lx.store(state->Gamepad.sThumbLX, std::memory_order_relaxed);
    p->pad_ly.store(state->Gamepad.sThumbLY, std::memory_order_relaxed);
}

void driver_probe_shutdown() {
    probe_stop("teardown");
}

} // namespace halo
