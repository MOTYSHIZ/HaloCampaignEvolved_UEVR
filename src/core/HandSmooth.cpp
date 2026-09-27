#include "core/HandSmooth.hpp"
#include "core/config/CfgRead.hpp"

#include "Config.hpp"
#include "DevTools.hpp"
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

// Two stereo pre-callbacks closer than this are the two eyes of ONE frame (they are microseconds
// apart; the shortest real frame here is ~7 ms at 144 Hz). Only the first steps the filter.
constexpr long long kSameFrameUs = 2000;
// No step for this long (a loading hitch, tracking lost): restart at the raw pose rather than
// easing across the gap from where the hand used to be. Also the age past which a publication is
// no longer served.
constexpr long long kStaleUs = 100000;

// FILTER STATE: render thread only (hand_smooth_frame), so it needs no lock.
struct HandState {
    bool      have = false;
    long long t_us = 0;
    Vec3 raw_pos{};  Quat raw_rot{0.0f, 0.0f, 0.0f, 1.0f};   // the raw sample last stepped on
    Vec3 pos{};      Quat rot{0.0f, 0.0f, 0.0f, 1.0f};       // the filtered pose
    Vec3 dpos{};     Vec3 drot{};                            // filtered velocity: m/s, and rad/s as axis*rate
};
HandState g_hand[2];   // 0 = left controller, 1 = right controller

// THE PUBLICATION: written by the owner, copied by every reader.
struct Published {
    bool          valid = false;
    int32_t       idx = -1;
    long long     t_us = 0;
    API::VR::Pose grip{}, aim{};
};
SRWLOCK   g_pub_lock = SRWLOCK_INIT;
Published g_pub[2];

#if HALO_VR_DEV
// HANDSMOOTHLOG (dev builds only). Counted only while the log is on; reset on its off->on edge.
std::atomic<bool>     g_log_on{false};
std::atomic<uint32_t> g_serves{0};
uint32_t  s_frames = 0, s_steps = 0;          // render thread only
float     s_max_pos_mm = 0.0f, s_max_rot_deg = 0.0f;
long long s_log_t = 0;
#endif

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

// The rotation vector (axis * angle, radians) of a unit quaternion, shortest arc.
Vec3 q_log(Quat q) {
    if (q.w < 0.0f) q = Quat{-q.x, -q.y, -q.z, -q.w};
    const float s = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    if (s < 1e-7f) return Vec3{2.0f * q.x, 2.0f * q.y, 2.0f * q.z};
    const float k = 2.0f * std::atan2(s, q.w) / s;
    return Vec3{q.x * k, q.y * k, q.z * k};
}

Vec3 to_vec(const UEVR_Vector3f& v) { return Vec3{v.x, v.y, v.z}; }
Quat to_quat(const UEVR_Quaternionf& q) { return Quat{q.x, q.y, q.z, q.w}; }

bool pose_valid(const API::VR::Pose& p) {
    const Quat q = to_quat(p.rotation);
    const float m2 = quat_dot(q, q);   // UEVR's pre-tracking placeholder is not unit length (see get_pose)
    return m2 > 0.9f && m2 < 1.1f && std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
           std::isfinite(p.position.z);
}

// One grip+aim pair from ONE UEVR sample: grip, aim, grip again; if the grip moved across the aim
// read, UEVR updated mid-pair, so take the later grip and read the aim again.
void read_pair(int32_t idx, API::VR::Pose* grip, API::VR::Pose* aim) {
    *grip = API::VR::get_pose(idx);
    *aim  = API::VR::get_aim_pose(idx);
    const API::VR::Pose again = API::VR::get_pose(idx);
    if (std::memcmp(&again, grip, sizeof(again)) != 0) {
        *grip = again;
        *aim  = API::VR::get_aim_pose(idx);
    }
}

// Advance h with one raw sample, if it is a new one.
bool step(HandState& h, const Vec3& rp, const Quat& rq, long long t, const Config& cfg) {
    const float dt = (float)(t - h.t_us) * 1e-6f;
    if (!h.have || t - h.t_us > kStaleUs) {
        h.have = true; h.t_us = t;
        h.raw_pos = rp; h.raw_rot = rq; h.pos = rp; h.rot = rq;
        h.dpos = Vec3{}; h.drot = Vec3{};
        return false;
    }
    // UEVR has not produced a new sample since the last step (a frame rate above its pose rate):
    // wait, so the next step spans the real interval instead of differentiating a zero and a double.
    if (std::memcmp(&rp, &h.raw_pos, sizeof(Vec3)) == 0 && std::memcmp(&rq, &h.raw_rot, sizeof(Quat)) == 0)
        return false;
    if (!(dt > 0.0f)) return false;

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
        h.rot = quat_slerp_short(h.rot, rq, alpha_for(fc, dt));
    } else {
        h.rot = rq; h.drot = Vec3{};
    }

    h.raw_pos = rp; h.raw_rot = rq; h.t_us = t;
    return true;
}

void withdraw(int slot) {
    AcquireSRWLockExclusive(&g_pub_lock);
    g_pub[slot].valid = false;
    ReleaseSRWLockExclusive(&g_pub_lock);
}

#if HALO_VR_DEV
void log_frame(long long t, const Config& cfg) {
    const bool on = cfg.hand_smooth_log;
    if (on && !g_log_on.load(std::memory_order_relaxed)) {   // off->on: start a clean window
        s_frames = s_steps = 0; s_max_pos_mm = s_max_rot_deg = 0.0f; s_log_t = t;
        g_serves.store(0, std::memory_order_relaxed);
    }
    g_log_on.store(on, std::memory_order_relaxed);
    if (!on) return;
    ++s_frames;
    if (t - s_log_t < 10000000) return;
    const double secs = (double)(t - s_log_t) * 1e-6;
    API::get()->log_info("[Halo-CampE-UEVR] HANDSMOOTH hands=%d: %.1f frames/s, %.1f filter steps/s, %.0f smoothed "
                         "reads/s, largest correction %.1f mm / %.2f deg  [posmin %.2f posbeta %.0f rotmin %.2f "
                         "rotbeta %.2f dcut %.1f melee %s]",
                         cfg.hand_smooth_hands, s_frames / secs, s_steps / secs,
                         g_serves.exchange(0, std::memory_order_relaxed) / secs, s_max_pos_mm, s_max_rot_deg,
                         cfg.hand_smooth_pos_min, cfg.hand_smooth_pos_beta, cfg.hand_smooth_rot_min,
                         cfg.hand_smooth_rot_beta, cfg.hand_smooth_dcut, cfg.hand_smooth_melee ? "smoothed" : "raw");
    s_frames = s_steps = 0; s_max_pos_mm = s_max_rot_deg = 0.0f; s_log_t = t;
}
#endif

}  // namespace

void hand_smooth_frame() {
    CFG_HOOK_READ;   // render thread: see core/config/CfgRead.hpp
    static long long s_last_frame = 0;
    const long long t = now_us();
    if (s_last_frame != 0 && t - s_last_frame < kSameFrameUs) return;   // the other eye of this frame
    s_last_frame = t;

    if (!service_active(SVC_HAND_SMOOTH)) {
        for (int s = 0; s < 2; ++s) { if (g_hand[s].have) { g_hand[s].have = false; withdraw(s); } }
        return;
    }
#if HALO_VR_DEV
    log_frame(t, g_cfg);
#endif

    const int32_t ids[2] = {API::VR::get_left_controller_index(), API::VR::get_right_controller_index()};
    for (int s = 0; s < 2; ++s) {
        const int32_t idx = ids[s];
        const bool is_aim = (s == 0) == g_cfg.aim_left_hand;
        const bool wanted = idx >= 0 && !(g_cfg.hand_smooth_hands == 1 && !is_aim)
                                     && !(g_cfg.hand_smooth_hands == 2 &&  is_aim);
        if (!wanted) { g_hand[s].have = false; withdraw(s); continue; }

        API::VR::Pose grip{}, aim{};
        read_pair(idx, &grip, &aim);
        if (!pose_valid(grip)) { g_hand[s].have = false; withdraw(s); continue; }

        const Vec3 rp = to_vec(grip.position);
        const Quat rq = to_quat(grip.rotation);
        const bool stepped = step(g_hand[s], rp, rq, t, g_cfg);
        const Vec3 fp = g_hand[s].pos;
        const Quat fq = g_hand[s].rot;

        // aim = grip * offset, so the smoothed aim is smoothed grip * (raw grip^-1 * raw aim). The
        // aim's translation takes the grip's correction as a plain shift -- it is never a signal.
        Published p{};
        p.valid = true; p.idx = idx; p.t_us = t;
        p.grip = grip;
        p.grip.position.x = fp.x; p.grip.position.y = fp.y; p.grip.position.z = fp.z;
        p.grip.rotation.x = fq.x; p.grip.rotation.y = fq.y; p.grip.rotation.z = fq.z; p.grip.rotation.w = fq.w;
        const Quat fa = quat_normalize(quat_mul(fq, quat_mul(quat_conj(rq), to_quat(aim.rotation))));
        p.aim = aim;
        p.aim.rotation.x = fa.x; p.aim.rotation.y = fa.y; p.aim.rotation.z = fa.z; p.aim.rotation.w = fa.w;
        p.aim.position.x += fp.x - rp.x; p.aim.position.y += fp.y - rp.y; p.aim.position.z += fp.z - rp.z;

        AcquireSRWLockExclusive(&g_pub_lock);
        g_pub[s] = p;
        ReleaseSRWLockExclusive(&g_pub_lock);

#if HALO_VR_DEV
        if (g_cfg.hand_smooth_log) {
            if (stepped) ++s_steps;
            const float mm  = 1000.0f * v_len(Vec3{fp.x - rp.x, fp.y - rp.y, fp.z - rp.z});
            const float deg = RAD2DEG * v_len(q_log(quat_mul(fq, quat_conj(rq))));
            if (mm > s_max_pos_mm) s_max_pos_mm = mm;
            if (deg > s_max_rot_deg) s_max_rot_deg = deg;
        }
#else
        (void)stepped;
#endif
    }
}

bool hand_smooth_get(int32_t idx, API::VR::Pose* grip, API::VR::Pose* aim) {
    if (idx < 0) return false;
    bool hit = false;
    const long long t = now_us();
    AcquireSRWLockShared(&g_pub_lock);
    for (const Published& p : g_pub) {
        if (!p.valid || p.idx != idx || t - p.t_us > kStaleUs) continue;
        if (grip != nullptr) *grip = p.grip;
        if (aim != nullptr)  *aim  = p.aim;
        hit = true;
        break;
    }
    ReleaseSRWLockShared(&g_pub_lock);
#if HALO_VR_DEV
    if (hit && g_log_on.load(std::memory_order_relaxed)) g_serves.fetch_add(1, std::memory_order_relaxed);
#endif
    return hit;
}

bool hand_smooth_lookup(int32_t idx, bool use_aim, API::VR::Pose* out) {
    if (out == nullptr) return false;
    return use_aim ? hand_smooth_get(idx, nullptr, out) : hand_smooth_get(idx, out, nullptr);
}

bool hand_smooth_parse_key(const char* key, const char* val, double v) {
    (void)val;
    if (_stricmp(key, "handsmoothhands")   == 0) { g_cfg.hand_smooth_hands    = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "handsmoothposmin")  == 0) { g_cfg.hand_smooth_pos_min  = clampf((float)v, 0.0f, 30.0f); return true; }
    if (_stricmp(key, "handsmoothposbeta") == 0) { g_cfg.hand_smooth_pos_beta = clampf((float)v, 0.0f, 100000.0f); return true; }
    if (_stricmp(key, "handsmoothrotmin")  == 0) { g_cfg.hand_smooth_rot_min  = clampf((float)v, 0.0f, 30.0f); return true; }
    if (_stricmp(key, "handsmoothrotbeta") == 0) { g_cfg.hand_smooth_rot_beta = clampf((float)v, 0.0f, 1000.0f); return true; }
    if (_stricmp(key, "handsmoothdcut")    == 0) { g_cfg.hand_smooth_dcut     = clampf((float)v, 0.1f, 100.0f); return true; }
    if (_stricmp(key, "handsmoothmelee")   == 0) { g_cfg.hand_smooth_melee    = (v != 0.0); return true; }
    // Parsed in every build so a dev cfg line is never an unknown key; it only acts in dev builds.
    if (_stricmp(key, "handsmoothlog")     == 0) { g_cfg.hand_smooth_log      = (v != 0.0); return true; }
    return false;
}

} // namespace halo
