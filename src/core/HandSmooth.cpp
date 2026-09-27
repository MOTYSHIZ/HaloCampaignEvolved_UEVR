#include "core/HandSmooth.hpp"
#include "core/config/CfgRead.hpp"

#include "Config.hpp"
#include "Math.hpp"
#include "core/Services.hpp"
#include "uevr/API.hpp"

#include <Windows.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>

using uevr::API;

namespace halo {
namespace {

// A changed sample less than this after the last step waits for the next read, so a runtime that
// re-predicts on every call cannot feed the speed estimate a delta over a near-zero dt. A read that
// raced an older sample in behind a newer step lands here too (its dt is negative).
constexpr float kMinStepS = 0.001f;
// No step for this long (a loading hitch, a paused frontend, tracking lost): restart at the raw pose
// rather than easing across the gap from where the hand used to be.
constexpr float kStaleS = 0.1f;

struct HandState {
    bool      have = false;
    long long t_us = 0;
    Vec3 raw_pos{};  Quat raw_rot{0.0f, 0.0f, 0.0f, 1.0f};   // the raw sample last stepped on
    Vec3 pos{};      Quat rot{0.0f, 0.0f, 0.0f, 1.0f};       // the filtered pose
    Vec3 dpos{};     Vec3 drot{};                            // filtered velocity: m/s, and rad/s as axis*rate
};

SRWLOCK   g_lock = SRWLOCK_INIT;
HandState g_hand[2];   // 0 = left controller, 1 = right controller

// HANDSMOOTHLOG counters, reset at each report.
std::atomic<uint32_t>  g_steps{0}, g_serves{0};
std::atomic<float>     g_max_pos_mm{0.0f}, g_max_rot_deg{0.0f};
std::atomic<long long> g_log_us{0};

long long now_us() {
    return (long long)std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// The 1 Euro filter's smoothing factor for cutoff fc (Hz) over dt (s): alpha = 1 / (1 + tau/dt),
// tau = 1 / (2 pi fc). Same form as VRE's FBasicLowPassFilter::CalculateAlphaTau.
float alpha_for(float fc, float dt) {
    if (!(fc > 0.0f)) return 0.0f;
    const float tau = 1.0f / (6.28318530718f * fc);
    return 1.0f / (1.0f + tau / dt);
}

Vec3  v_lerp(const Vec3& a, const Vec3& b, float t) { return Vec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t}; }
float v_len(const Vec3& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }
float q_dot(const Quat& a, const Quat& b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }

Quat q_norm(const Quat& q) {
    const float m = std::sqrt(q_dot(q, q));
    if (!(m > 1e-8f)) return Quat{0.0f, 0.0f, 0.0f, 1.0f};
    return Quat{q.x / m, q.y / m, q.z / m, q.w / m};
}

// a toward b by t along the shorter arc. Nlerp when the two are nearly equal, where slerp's divide
// by sin(theta) loses precision; the per-step angles here are almost always in that regime.
Quat q_slerp(const Quat& a, Quat b, float t) {
    float d = q_dot(a, b);
    if (d < 0.0f) { b = Quat{-b.x, -b.y, -b.z, -b.w}; d = -d; }
    if (d > 0.9995f)
        return q_norm(Quat{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t});
    const float th = std::acos(d), s = std::sin(th);
    const float wa = std::sin((1.0f - t) * th) / s, wb = std::sin(t * th) / s;
    return q_norm(Quat{a.x * wa + b.x * wb, a.y * wa + b.y * wb, a.z * wa + b.z * wb, a.w * wa + b.w * wb});
}

// The rotation vector (axis * angle, radians) of a unit quaternion, shortest arc.
Vec3 q_log(Quat q) {
    if (q.w < 0.0f) q = Quat{-q.x, -q.y, -q.z, -q.w};
    const float s = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    if (s < 1e-7f) return Vec3{2.0f * q.x, 2.0f * q.y, 2.0f * q.z};
    const float k = 2.0f * std::atan2(s, q.w) / s;
    return Vec3{q.x * k, q.y * k, q.z * k};
}

// Which filter slot a device index uses, or -1 when this device is not smoothed. The HMD is never
// a slot. hands: 0 both, 1 the aim hand, 2 the support hand (aim hand = aimlefthand's choice).
int hand_slot(int32_t idx, const Config& cfg) {
    const int32_t li = API::VR::get_left_controller_index();
    const int32_t ri = API::VR::get_right_controller_index();
    const int slot = (li >= 0 && idx == li) ? 0 : (ri >= 0 && idx == ri) ? 1 : -1;
    if (slot < 0) return -1;
    const bool is_aim = (slot == 0) == cfg.aim_left_hand;
    if (cfg.hand_smooth_hands == 1 && !is_aim) return -1;
    if (cfg.hand_smooth_hands == 2 &&  is_aim) return -1;
    return slot;
}

// Advance h with one raw sample, if it is a new one. Caller holds g_lock exclusive.
void step(HandState& h, const Vec3& rp, const Quat& rq, long long t, const Config& cfg) {
    const float dt = (float)(t - h.t_us) * 1e-6f;
    if (!h.have || dt > kStaleS) {
        h.have = true; h.t_us = t;
        h.raw_pos = rp; h.raw_rot = rq; h.pos = rp; h.rot = rq;
        h.dpos = Vec3{}; h.drot = Vec3{};
        return;
    }
    if (std::memcmp(&rp, &h.raw_pos, sizeof(Vec3)) == 0 && std::memcmp(&rq, &h.raw_rot, sizeof(Quat)) == 0)
        return;   // UEVR has not produced a new sample since the last step
    if (dt < kMinStepS) return;

    const float ad = alpha_for(clampf(cfg.hand_smooth_dcut, 0.1f, 100.0f), dt);

    if (cfg.hand_smooth_pos_min > 0.0f) {
        const float inv = 1.0f / dt;
        const Vec3 v{(rp.x - h.raw_pos.x) * inv, (rp.y - h.raw_pos.y) * inv, (rp.z - h.raw_pos.z) * inv};
        h.dpos = v_lerp(h.dpos, v, ad);
        const float fc = cfg.hand_smooth_pos_min + cfg.hand_smooth_pos_beta * v_len(h.dpos);
        h.pos = v_lerp(h.pos, rp, alpha_for(fc, dt));
    } else {
        h.pos = rp; h.dpos = Vec3{};
    }

    if (cfg.hand_smooth_rot_min > 0.0f) {
        const Vec3 rv = q_log(quat_mul(rq, quat_conj(h.raw_rot)));   // this step's rotation, world frame
        const float inv = 1.0f / dt;
        h.drot = v_lerp(h.drot, Vec3{rv.x * inv, rv.y * inv, rv.z * inv}, ad);
        const float fc = cfg.hand_smooth_rot_min + cfg.hand_smooth_rot_beta * v_len(h.drot);
        h.rot = q_slerp(h.rot, rq, alpha_for(fc, dt));
    } else {
        h.rot = rq; h.drot = Vec3{};
    }

    h.raw_pos = rp; h.raw_rot = rq; h.t_us = t;
    g_steps.fetch_add(1, std::memory_order_relaxed);
}

void atomic_max(std::atomic<float>& a, float v) {
    float cur = a.load(std::memory_order_relaxed);
    while (v > cur && !a.compare_exchange_weak(cur, v, std::memory_order_relaxed)) {}
}

void maybe_log(long long t, const Config& cfg) {
    if (!cfg.hand_smooth_log) return;
    long long last = g_log_us.load(std::memory_order_relaxed);
    if (last == 0) { g_log_us.compare_exchange_strong(last, t, std::memory_order_relaxed); return; }
    if (t - last < 10000000 || !g_log_us.compare_exchange_strong(last, t, std::memory_order_relaxed)) return;
    const double secs = (double)(t - last) * 1e-6;
    API::get()->log_info("[Halo-CampE-UEVR] HANDSMOOTH hands=%d: %.1f steps/s, %.0f smoothed reads/s, largest "
                         "correction %.1f mm / %.2f deg  [posmin %.2f posbeta %.0f rotmin %.2f rotbeta %.2f dcut %.1f]",
                         cfg.hand_smooth_hands, g_steps.exchange(0) / secs, g_serves.exchange(0) / secs,
                         g_max_pos_mm.exchange(0.0f), g_max_rot_deg.exchange(0.0f),
                         cfg.hand_smooth_pos_min, cfg.hand_smooth_pos_beta, cfg.hand_smooth_rot_min,
                         cfg.hand_smooth_rot_beta, cfg.hand_smooth_dcut);
}

}  // namespace

bool hand_smooth_apply(int32_t idx, API::VR::Pose* grip, API::VR::Pose* aim) {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    if (idx < 0 || grip == nullptr || !service_active(SVC_HAND_SMOOTH)) return false;
    const int slot = hand_slot(idx, g_cfg);
    if (slot < 0) return false;

    const Vec3 rp{grip->position.x, grip->position.y, grip->position.z};
    const Quat rq{grip->rotation.x, grip->rotation.y, grip->rotation.z, grip->rotation.w};
    const float m2 = q_dot(rq, rq);
    if (!(m2 > 0.9f && m2 < 1.1f) || !std::isfinite(rp.x) || !std::isfinite(rp.y) || !std::isfinite(rp.z)) {
        // UEVR's pre-tracking placeholder (see get_pose): hand it on untouched for get_pose to refuse,
        // and start the filter over once a real pose arrives.
        AcquireSRWLockExclusive(&g_lock);
        g_hand[slot].have = false;
        ReleaseSRWLockExclusive(&g_lock);
        return false;
    }

    const long long t = now_us();
    AcquireSRWLockExclusive(&g_lock);
    step(g_hand[slot], rp, rq, t, g_cfg);
    const Vec3 fp = g_hand[slot].pos;
    const Quat fq = g_hand[slot].rot;
    ReleaseSRWLockExclusive(&g_lock);

    if (aim != nullptr) {
        // aim = grip * offset, so the smoothed aim is smoothed grip * (raw grip^-1 * raw aim). The
        // position takes the grip's correction as a plain shift -- see the header on why the aim
        // translation is never used as a signal.
        const Quat ra{aim->rotation.x, aim->rotation.y, aim->rotation.z, aim->rotation.w};
        const Quat fa = q_norm(quat_mul(fq, quat_mul(quat_conj(rq), ra)));
        aim->rotation.x = fa.x; aim->rotation.y = fa.y; aim->rotation.z = fa.z; aim->rotation.w = fa.w;
        aim->position.x += fp.x - rp.x; aim->position.y += fp.y - rp.y; aim->position.z += fp.z - rp.z;
    }
    grip->position.x = fp.x; grip->position.y = fp.y; grip->position.z = fp.z;
    grip->rotation.x = fq.x; grip->rotation.y = fq.y; grip->rotation.z = fq.z; grip->rotation.w = fq.w;

    if (g_cfg.hand_smooth_log) {
        g_serves.fetch_add(1, std::memory_order_relaxed);
        atomic_max(g_max_pos_mm, 1000.0f * v_len(Vec3{fp.x - rp.x, fp.y - rp.y, fp.z - rp.z}));
        atomic_max(g_max_rot_deg, RAD2DEG * v_len(q_log(quat_mul(fq, quat_conj(rq)))));
        maybe_log(t, g_cfg);
    }
    return true;
}

bool hand_smooth_lookup(int32_t idx, bool use_aim, API::VR::Pose* out) {
    if (out == nullptr) return false;
    API::VR::Pose grip = API::VR::get_pose(idx);
    if (!use_aim) {
        if (!hand_smooth_apply(idx, &grip, nullptr)) return false;
        *out = grip;
        return true;
    }
    API::VR::Pose aim = API::VR::get_aim_pose(idx);
    if (!hand_smooth_apply(idx, &grip, &aim)) return false;
    *out = aim;
    return true;
}

bool hand_smooth_parse_key(const char* key, const char* val, double v) {
    (void)val;
    if (_stricmp(key, "handsmoothhands")   == 0) { g_cfg.hand_smooth_hands    = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "handsmoothposmin")  == 0) { g_cfg.hand_smooth_pos_min  = clampf((float)v, 0.0f, 30.0f); return true; }
    if (_stricmp(key, "handsmoothposbeta") == 0) { g_cfg.hand_smooth_pos_beta = clampf((float)v, 0.0f, 100000.0f); return true; }
    if (_stricmp(key, "handsmoothrotmin")  == 0) { g_cfg.hand_smooth_rot_min  = clampf((float)v, 0.0f, 30.0f); return true; }
    if (_stricmp(key, "handsmoothrotbeta") == 0) { g_cfg.hand_smooth_rot_beta = clampf((float)v, 0.0f, 1000.0f); return true; }
    if (_stricmp(key, "handsmoothdcut")    == 0) { g_cfg.hand_smooth_dcut     = clampf((float)v, 0.1f, 100.0f); return true; }
    if (_stricmp(key, "handsmoothlog")     == 0) { g_cfg.hand_smooth_log      = (v != 0.0); return true; }
    return false;
}

} // namespace halo
