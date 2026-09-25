#include "core/fixes/RenderTime.hpp"

#include "Config.hpp"
#include "core/config/CfgRead.hpp"
#include "core/Services.hpp"
#include "Math.hpp"
#include "MotionAimControl.hpp"   // get_pose
#include "palettearm/NodeMap.hpp"  // kMaxPaletteNodes
#include "TwoHandAim.hpp"         // two_hand_latched
#include "uevr/API.hpp"

#include <Windows.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>

namespace halo {

namespace pa = ::halo::palettearm;

namespace {

constexpr std::uint32_t kMaxNodes = pa::kMaxPaletteNodes;

// The base mod's own frame conversions (palettearm/PaletteArm.cpp), restated exactly.
pa::Vec3 xr_to_blam(const pa::Vec3& v) { return {-v.z, -v.x, v.y}; }
pa::Mat3 xr_rotation_to_blam_basis(const pa::Quat& q) {
    const pa::Vec3 f = pa::rotate(q, {0.0f, 0.0f, -1.0f});
    const pa::Vec3 l = pa::rotate(q, {-1.0f, 0.0f, 0.0f});
    const pa::Vec3 u = pa::rotate(q, {0.0f, 1.0f, 0.0f});
    return {xr_to_blam(f), xr_to_blam(l), xr_to_blam(u)};
}

// THE SNAPSHOT. Written on the sim thread, read by the render pass; the seq is odd while written.
struct Snap {
    std::atomic<std::uint32_t> seq{0};
    // the build
    bool have_build = false;
    long long build_ticks = 0;
    std::int32_t model_tag = 0;
    std::uint32_t node_count = 0;
    std::uint8_t aim_idx[kMaxNodes]{}; std::size_t aim_n = 0;   // gun + aim wrist subtree, deduplicated
    std::uint8_t sup_idx[kMaxNodes]{}; std::size_t sup_n = 0;
    pa::Vec3 root{}; pa::Mat3 stage{}; pa::Quat comp{}; float wscale = 1.0f;
    pa::Vec3 aim_hmd{}, aim_pos{}; pa::Quat aim_rot{};
    pa::Vec3 sup_hmd{}, sup_pos{}; pa::Quat sup_rot{}; bool sup_valid = false;
    // the render banks, as the build left them
    pa::BlamMatrix4x3* bank[2] = {nullptr, nullptr};
    std::uint32_t bank_seq[2] = {0, 0};   // which build filled it
    pa::BlamMatrix4x3 stock[2][kMaxNodes]{};
    std::uint32_t build_id = 0;
};
Snap g_s;

long long now_ticks() { return std::chrono::steady_clock::now().time_since_epoch().count(); }
double ticks_to_ms(long long t) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::duration(t)).count();
}

bool active_mode(int* mode) {
    CFG_HOOK_READ;
    if (!service_active(SVC_STABILITY)) return false;
    *mode = g_cfg.stab_render_time;
    return *mode != 0;
}

struct Rigid { pa::Mat3 b{}; pa::Vec3 p{}; };
Rigid map_pose(const pa::Vec3& root, const pa::Mat3& stage, const pa::Quat& comp, float wscale,
               const pa::Vec3& hmd, const pa::Vec3& pos, const pa::Quat& rot) {
    Rigid r;
    r.b = pa::multiply(stage, xr_rotation_to_blam_basis(pa::normalized(comp * rot)));
    r.p = root + pa::transform_vector(stage, xr_to_blam(pa::rotate(comp, pos - hmd)) * (wscale / pa::kMetresPerBlamUnit));
    return r;
}

float basis_angle_deg(const pa::Mat3& a, const pa::Mat3& b) {
    // angle of a * b^T from its trace
    const pa::Mat3 d = pa::multiply(a, pa::transpose(b));
    const float tr = d.forward.x + d.left.y + d.up.z;
    const float c = std::clamp((tr - 1.0f) * 0.5f, -1.0f, 1.0f);
    return std::acos(c) * 57.2957795f;
}

void apply(pa::BlamMatrix4x3& n, const pa::Mat3& d, const pa::Vec3& from, const pa::Vec3& to) {
    pa::Mat3 nb{n.forward, n.left, n.up};
    nb = pa::multiply(d, nb);
    const pa::Vec3 np = to + pa::transform_vector(d, n.position - from);
    if (!pa::finite(nb.forward) || !pa::finite(nb.left) || !pa::finite(nb.up) || !pa::finite(np)) return;
    n.forward = nb.forward; n.left = nb.left; n.up = nb.up; n.position = np;
}

// The 1 Hz summary.
struct Stats { int frames = 0, applied = 0, sk_stale = 0, sk_torn = 0, sk_pose = 0, sk_two = 0; double aim_deg = 0, aim_cm = 0, age_ms = 0;
               float aim_deg_max = 0, age_max = 0; long long said = 0; };
Stats g_st;
void summary(int mode) {
    CFG_HOOK_READ;
    const long long t = now_ticks();
    if (g_st.said == 0) { g_st.said = t; return; }
    if (ticks_to_ms(t - g_st.said) < 1000.0) return;
    if (g_cfg.stab_render_time_log && g_st.frames > 0)
        uevr::API::get()->log_info("[Halo-CampE-UEVR] RENDERTIME mode %d: %d frames, %d applied (skipped stale %d torn %d pose %d) | "
                                   "aim hand moved since its placing pose: mean %.2f deg %.2f cm, max %.2f deg | build to render mean %.1f ms max %.1f | two-handed %d",
                                   mode, g_st.frames, g_st.applied, g_st.sk_stale, g_st.sk_torn, g_st.sk_pose,
                                   g_st.aim_deg / g_st.frames, g_st.aim_cm / g_st.frames, g_st.aim_deg_max,
                                   g_st.age_ms / g_st.frames, g_st.age_max, g_st.sk_two);
    g_st = Stats{}; g_st.said = t;
}

void add_unique(std::uint8_t* out, std::size_t* n, const std::uint8_t* in, std::size_t count, std::uint32_t node_count) {
    for (std::size_t k = 0; k < count && *n < kMaxNodes; ++k) {
        const std::uint8_t i = in[k];
        if (i == 0 || i >= node_count) continue;
        bool have = false;
        for (std::size_t j = 0; j < *n; ++j) if (out[j] == i) { have = true; break; }
        if (!have) out[(*n)++] = i;
    }
}

}  // namespace

void render_time_note_build(const PaDriveDone& d) {
    int mode = 0;
    if (!active_mode(&mode)) { g_s.have_build = false; return; }
    if (d.weapon_slot != 0 || d.palette == nullptr || d.node_count == 0 || d.node_count > kMaxNodes) return;
    g_s.seq.fetch_add(1, std::memory_order_acq_rel);   // odd: writing
    g_s.have_build = true;
    g_s.build_ticks = now_ticks();
    g_s.model_tag = d.model_tag;
    g_s.node_count = d.node_count;
    g_s.aim_n = 0; g_s.sup_n = 0;
    add_unique(g_s.aim_idx, &g_s.aim_n, d.wpn_nodes, d.wpn_count, d.node_count);
    add_unique(g_s.aim_idx, &g_s.aim_n, d.aim_nodes, d.aim_count, d.node_count);
    add_unique(g_s.sup_idx, &g_s.sup_n, d.sup_nodes, d.sup_count, d.node_count);
    g_s.root = d.root_position; g_s.stage = d.stage_basis; g_s.comp = d.composition; g_s.wscale = d.wscale;
    g_s.aim_hmd = d.aim_hmd; g_s.aim_pos = d.aim_pos; g_s.aim_rot = d.aim_rot;
    g_s.sup_hmd = d.sup_hmd; g_s.sup_pos = d.sup_pos; g_s.sup_rot = d.sup_rot; g_s.sup_valid = d.sup_valid;
    ++g_s.build_id;
    g_s.seq.fetch_add(1, std::memory_order_acq_rel);   // even: stable
}

void render_time_note_bank(pa::BlamMatrix4x3* bank, std::uint32_t node_count, std::int32_t model_tag,
                           std::int32_t weapon_slot, std::uint8_t bank_index) {
    int mode = 0;
    if (!active_mode(&mode)) return;
    if (weapon_slot != 0 || bank == nullptr || bank_index > 1 || !g_s.have_build) return;
    if (node_count != g_s.node_count || model_tag != g_s.model_tag) return;
    g_s.seq.fetch_add(1, std::memory_order_acq_rel);
    g_s.bank[bank_index] = bank;
    std::memcpy(g_s.stock[bank_index], bank, sizeof(pa::BlamMatrix4x3) * node_count);
    g_s.bank_seq[bank_index] = g_s.build_id;
    g_s.seq.fetch_add(1, std::memory_order_acq_rel);
}

void render_time_refresh() {
    int mode = 0;
    if (!active_mode(&mode)) return;
    summary(mode);
    const std::uint32_t s0 = g_s.seq.load(std::memory_order_acquire);
    if (s0 & 1u) { ++g_st.sk_torn; return; }
    if (!g_s.have_build) return;
    ++g_st.frames;
    const double age = ticks_to_ms(now_ticks() - g_s.build_ticks);
    if (age > 60.0) { ++g_st.sk_stale; return; }

    // The poses NOW, sampled the way the build samples them.
    CFG_HOOK_READ;
    const auto hidx = uevr::API::VR::get_hmd_index();
    const auto aidx = g_cfg.aim_left_hand ? uevr::API::VR::get_left_controller_index() : uevr::API::VR::get_right_controller_index();
    const auto sidx = g_cfg.aim_left_hand ? uevr::API::VR::get_right_controller_index() : uevr::API::VR::get_left_controller_index();
    Vec3 hp{}, ap{}, sp{}; Quat hq{}, aq{}, sq{};
    if (hidx < 0 || aidx < 0 || !get_pose(hidx, &hp, &hq, false) || !get_pose(aidx, &ap, &aq, false)) { ++g_st.sk_pose; return; }
    const bool have_sup = sidx >= 0 && get_pose(sidx, &sp, &sq, false);
    const pa::Vec3 h_now{hp.x, hp.y, hp.z};
    if ((ap.x == 0.0f && ap.y == 0.0f && ap.z == 0.0f) || !std::isfinite(ap.x)) { ++g_st.sk_pose; return; }

    // Copy what this pass needs out of the snapshot, then prove the writer stayed out.
    const pa::Vec3 root = g_s.root; const pa::Mat3 stage = g_s.stage; const pa::Quat comp = g_s.comp; const float ws = g_s.wscale;
    const Rigid a_b = map_pose(root, stage, comp, ws, g_s.aim_hmd, g_s.aim_pos, g_s.aim_rot);
    const Rigid a_n = map_pose(root, stage, comp, ws, h_now, pa::Vec3{ap.x, ap.y, ap.z}, pa::Quat{aq.x, aq.y, aq.z, aq.w});
    const pa::Mat3 da = pa::multiply(a_n.b, pa::transpose(a_b.b));
    const float adeg = basis_angle_deg(a_n.b, a_b.b);
    const float acm = pa::length(a_n.p - a_b.p) * pa::kMetresPerBlamUnit * 100.0f;
    g_st.aim_deg += adeg; g_st.aim_cm += acm; g_st.age_ms += age;
    if (adeg > g_st.aim_deg_max) g_st.aim_deg_max = adeg;
    if ((float)age > g_st.age_max) g_st.age_max = (float)age;
    if (mode == 1) return;   // measure only

    // THE TWO-HANDED HOLD swings the gun from BOTH hands (TwoHandAim), so the aim controller alone
    // does not say where it went: stand down rather than guess.
    if (two_hand_latched()) { ++g_st.sk_two; return; }
    pa::Mat3 ds = da; pa::Vec3 s_from = a_b.p, s_to = a_n.p;
    const bool do_sup = mode != 4 && g_s.sup_valid && have_sup;
    if (do_sup) {
        const Rigid s_b = map_pose(root, stage, comp, ws, g_s.sup_hmd, g_s.sup_pos, g_s.sup_rot);
        const Rigid s_n = map_pose(root, stage, comp, ws, h_now, pa::Vec3{sp.x, sp.y, sp.z}, pa::Quat{sq.x, sq.y, sq.z, sq.w});
        ds = pa::multiply(s_n.b, pa::transpose(s_b.b)); s_from = s_b.p; s_to = s_n.p;
    }
    const pa::Vec3 a_to = (mode == 3) ? a_b.p : a_n.p;
    if (mode == 3) s_to = s_from;

    static pa::BlamMatrix4x3 scratch[kMaxNodes];
    const std::uint32_t cnt = g_s.node_count;
    bool any = false;
    for (int b = 0; b < 2; ++b) {
        pa::BlamMatrix4x3* bank = g_s.bank[b];
        if (bank == nullptr || g_s.bank_seq[b] != g_s.build_id) continue;
        if (IsBadWritePtr(bank, sizeof(pa::BlamMatrix4x3) * cnt)) continue;
        std::memcpy(scratch, g_s.stock[b], sizeof(pa::BlamMatrix4x3) * cnt);
        for (std::size_t k = 0; k < g_s.aim_n; ++k) apply(scratch[g_s.aim_idx[k]], da, a_b.p, a_to);
        if (do_sup) for (std::size_t k = 0; k < g_s.sup_n; ++k) apply(scratch[g_s.sup_idx[k]], ds, s_from, s_to);
        if (g_s.seq.load(std::memory_order_acquire) != s0) { ++g_st.sk_torn; return; }   // a build got in
        std::memcpy(bank, scratch, sizeof(pa::BlamMatrix4x3) * cnt);
        any = true;
    }
    if (any) ++g_st.applied;
}

} // namespace halo
