#include "Config.hpp"
#include "Markers.hpp"            // the grab-zone marker rides the holster marker machinery
#include "UeObject.hpp"           // TrackedObject for that marker

#include <chrono>
// The pad-button mirror lives HERE (published by the XInput hook in Plugin.cpp): the two-hand
// latch is its consumer. Gesture features added later share this same mirror.
#include "PaletteTwoHand.hpp"
#include "Holster.hpp"            // g_pad_buttons -- the raw pad, before our own remapping
#include "MotionAimControl.hpp"   // get_pose
#include "uevr/API.hpp"

#include <cmath>

using namespace uevr;

namespace halo {

std::atomic<unsigned short> g_pad_buttons{0};

std::atomic<bool>  g_th_latched{false};
std::atomic<bool>  g_th_in_zone{false};
std::atomic<float> g_th_blend{0.0f};

namespace {

// ---- THE GRAB-ZONE MARKER: a small dot on the barrel where two-handing latches, shown while
// the off hand is approaching and not yet latched (a tester video, 2026-09-01: testers could
// not find the grip). Same paced-respawn lifecycle as every holster marker.
TrackedObject s_zone_marker;
long long s_zone_try_at = 0;
inline long long th_now_ticks() { return std::chrono::steady_clock::now().time_since_epoch().count(); }
void twohand_marker_update(bool show, const Vec3& room) {
    if (!g_cfg.two_hand_marker) show = false;
    auto* m = s_zone_marker.get();
    Vec3 hpos{}; Quat hq{};
    if (show && !get_pose(API::VR::get_hmd_index(), &hpos, &hq, /*use_aim=*/false)) show = false;
    if (!show) { if (m != nullptr) { holster_marker_show(m, false); marker_render_drop(m); } return; }
    if (m == nullptr) {
        const long long nowt = th_now_ticks();
        if (nowt - s_zone_try_at < std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                       std::chrono::milliseconds(2000)).count()) return;
        s_zone_try_at = nowt;
        auto* owner = API::get()->get_local_pawn(0);
        if (owner == nullptr) return;
        static const wchar_t* kDot[] = { L"StaticMesh /Engine/BasicShapes/Sphere.Sphere" };
        const double s = (double)g_cfg.two_hand_marker_scale;
        m = marker_spawn_list(owner, kDot, 1, s, s, s);
        if (m == nullptr) return;
        marker_tint(m, g_cfg.two_hand_marker_color);
        s_zone_marker.set(m);
    }
    holster_marker_place(m, holster_room_to_world(room, hpos));
    holster_marker_show(m, true);
    // Re-anchored at render from the same room point, like every other hand marker (Markers.hpp):
    // tick-only placement used the camera from the last render, so the dot trailed the view.
    marker_render_anchor(m, room);
}

// The hand-to-hand direction, VR space, published by the tick for the aim path to read. Kept as
// the LAST VALID one rather than the current one: support-hand tracking drops out (occlusion, out
// of view, a hand behind the gun), and an aim that lurches to a garbage line for those frames is
// far worse than one that holds the last good heading for them.
std::atomic<float> s_line_x{0.0f}, s_line_y{0.0f}, s_line_z{0.0f};
std::atomic<bool>  s_line_valid{false};

inline float dot3(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline bool norm3(Vec3* v) {
    const float l = std::sqrt(dot3(*v, *v));
    if (!(l > 1.0e-6f) || !std::isfinite(l)) return false;
    v->x /= l; v->y /= l; v->z /= l;
    return true;
}

// The aim hand's forward, taken from the SAME pose the aim derivation uses.
//
// aimsrc=1 switches the aim to the grip pose so wrist roll stops sweeping the aim around a cone.
// The grab zone is measured along the aim ray, so it has to follow that choice -- measuring the
// zone against the aim pose while the player aims with the grip pose would put the zone a fixed
// angle off the barrel, and the cylinder would sit beside the gun rather than along it.
bool aim_hand_forward(int32_t idx, Vec3* out_fwd, Vec3* out_grip_pos) {
    Vec3 gpos{}; Quat gq{};
    if (!get_pose(idx, &gpos, &gq, /*use_aim=*/false)) return false;
    *out_grip_pos = gpos;

    // Through the rigid aim fix, same as derive_ctrl_angles: the grab zone is a cylinder along the
    // line the reticle actually sits on. Raw here and 8 deg down there is 8 cm off-axis at 60 cm
    // reach -- most of a 9 cm radius -- so the zone would miss a hand that IS on the barrel.
    if (g_cfg.aim_src == 1) {
        *out_fwd = quat_forward(apply_aim_fix(gq));
        return true;
    }
    Vec3 apos{}; Quat aq{};
    if (!get_pose(idx, &apos, &aq, /*use_aim=*/true)) return false;
    *out_fwd = quat_forward(apply_aim_fix(aq));
    return true;
}

void haptic(bool engaged) {
    if (!g_cfg.two_hand_haptic) return;
    // ABI ORDER, NOT WRAPPER NAMES. API.h declares
    //     (seconds_from_now, duration, frequency, amplitude, source)
    // while the C++ wrapper in API.hpp names the same positions
    //     (seconds_from_now, amplitude, frequency, duration, source)
    // and forwards them positionally. The wrapper's names are simply wrong; position 2 is
    // DURATION and position 4 is AMPLITUDE. Verified against both headers -- write to the ABI.
    const auto src = g_cfg.aim_left_hand ? API::VR::get_right_joystick_source()
                                         : API::VR::get_left_joystick_source();
    API::VR::trigger_haptic_vibration(0.0f,
                                      engaged ? 0.12f : 0.06f,   // duration
                                      0.0f,                      // frequency (0 = runtime default)
                                      engaged ? 0.70f : 0.35f,   // amplitude
                                      src);
}

} // namespace

void palette_two_hand_reset() {
    if (g_th_latched.exchange(false, std::memory_order_acq_rel)) {
        // No haptic on a forced release. The buzz means "you did something"; firing it when the
        // mod dropped the hold for its own reasons teaches the player a lie about their hands.
    }
    g_th_in_zone.store(false, std::memory_order_relaxed);
    g_th_blend.store(0.0f, std::memory_order_relaxed);
    s_line_valid.store(false, std::memory_order_relaxed);
    twohand_marker_update(false, Vec3{});
}

void palette_two_hand_update(float dt) {
    if (!g_cfg.enabled || !g_cfg.two_hand) {
        if (g_th_latched.load(std::memory_order_relaxed) ||
            g_th_blend.load(std::memory_order_relaxed) != 0.0f) {
            palette_two_hand_reset();
        }
        return;
    }

    // WHICH HAND FETCHES. The aim hand holds the weapon, so the other one is the one that comes up
    // to the barrel. Asked this way round, left-handed play works with no second code path -- the
    // same rule the reload gesture and the hand meshes already follow.
    const auto aim_idx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                             : API::VR::get_right_controller_index();
    const auto sup_idx = g_cfg.aim_left_hand ? API::VR::get_right_controller_index()
                                             : API::VR::get_left_controller_index();

    bool in_zone = false;
    bool have_line = false;
    Vec3 line{};
    bool have_zone_center = false;
    bool approaching = false;
    Vec3 zone_center{};

    if (aim_idx >= 0 && sup_idx >= 0) {
        Vec3 aim_fwd{}, aim_pos{};
        Vec3 sup_pos{}; Quat sup_rot{};
        if (aim_hand_forward(aim_idx, &aim_fwd, &aim_pos) &&
            get_pose(sup_idx, &sup_pos, &sup_rot, /*use_aim=*/false)) {

            // THE GRAB ZONE: a thin cylinder along the aim ray, measured from the aim hand's grip
            // -- the weapon's rear hand. Along the ray it spans a forestock's worth of barrel;
            // across it, a hand's width. A sphere around a point would either be too small to find
            // by feel or large enough to catch a hand resting at your side, and the barrel is a
            // line, not a point.
            const Vec3 hand_line{sup_pos.x - aim_pos.x,
                                 sup_pos.y - aim_pos.y,
                                 sup_pos.z - aim_pos.z};
            const float along = dot3(hand_line, aim_fwd);
            const Vec3  perp{hand_line.x - aim_fwd.x * along,
                             hand_line.y - aim_fwd.y * along,
                             hand_line.z - aim_fwd.z * along};
            const float lateral = std::sqrt(dot3(perp, perp));

            in_zone = std::isfinite(along) && std::isfinite(lateral) &&
                      along   > g_cfg.two_hand_min_m &&
                      along   < g_cfg.two_hand_max_m &&
                      lateral < g_cfg.two_hand_radius_m;
            // The rack zone wins: a hand on the slide, pump or handle is racking, not supporting
            // (Config.hpp slide_zone_priority). A hold already latched keeps going.
            if (g_cfg.slide_zone_priority && in_zone && !g_th_latched.load(std::memory_order_relaxed) && g_slide_zone_hot.load(std::memory_order_relaxed)) in_zone = false;

            // The marker's anchor (zone middle, on the ray) and whether the off hand is close
            // enough that showing it helps -- a generous envelope around the zone, so the dot
            // appears as the hand comes up and never sits on screen all fight.
            const float mid = 0.5f * (g_cfg.two_hand_min_m + g_cfg.two_hand_max_m);
            zone_center = Vec3{aim_pos.x + aim_fwd.x * mid,
                               aim_pos.y + aim_fwd.y * mid,
                               aim_pos.z + aim_fwd.z * mid};
            have_zone_center = true;
            approaching = std::isfinite(along) && std::isfinite(lateral) &&
                          along   > g_cfg.two_hand_min_m - 0.15f &&
                          along   < g_cfg.two_hand_max_m + 0.15f &&
                          lateral < g_cfg.two_hand_radius_m * 4.0f;

            line = hand_line;
            have_line = norm3(&line);
        }
    }
    g_th_in_zone.store(in_zone, std::memory_order_relaxed);

    // THE LATCH. Engaging needs the grip button AND the hand in the zone; holding needs only the
    // button. The zone gates ACQUISITION, never RETENTION -- once you have the barrel, drifting a
    // few centimetres off the ray must not throw the gun, and the alternative (re-testing the zone
    // every frame) makes the hold flicker exactly when the weapon is moving most.
    const unsigned short btn = g_pad_buttons.load(std::memory_order_relaxed);
    // THE GRIP: UEVR's per-hand ACTION, with the XInput mask kept only as a fallback. The mask
    // (reloadgrip, default 0x0100 = left shoulder) was the ONLY path here, and whether a physical
    // grip produces that bit depends on the controller and the runtime -- the plugin even ships a
    // remap that puts left X on the same bit. Field report from another machine: reload stalled in
    // MAG_OUT with the trigger suppressed and two-handing never latched, together, because both
    // gated on it. Holsters never had this problem; they have always read the action.
    const bool grip_held = holster_grip_held(g_cfg.aim_left_hand) ||
                           ((g_cfg.reload_grip_mask != 0) &&
                            ((btn & (unsigned short)g_cfg.reload_grip_mask) != 0));

    // The grenade pouches share this grip when they live on the off hand: a squeeze that is
    // pulling a grenade (or holding one) must not also brace the weapon. Gates ACQUISITION only
    // -- an established hold survives a hand passing near a pouch, same rule as the zone.
    const bool gren_busy = holster_offhand_busy();
    const bool was = g_th_latched.load(std::memory_order_relaxed);
    const bool now = grip_held && (was || (in_zone && !gren_busy));
    if (now != was) {
        g_th_latched.store(now, std::memory_order_release);
        haptic(now);
        if (g_cfg.two_hand_log) {
            API::get()->log_info("[Halo-CampE-UEVR] TWOHAND %s", now ? "grabbed" : "released");
        }
    }

    // The grab-zone dot: visible while the off hand approaches an unlatched zone with nothing
    // else claiming the grip; gone the moment the barrel is held or the hand withdraws.
    twohand_marker_update(have_zone_center && approaching && !now && !gren_busy, zone_center);

    if (have_line) {
        s_line_x.store(line.x, std::memory_order_relaxed);
        s_line_y.store(line.y, std::memory_order_relaxed);
        s_line_z.store(line.z, std::memory_order_release);
        s_line_valid.store(true, std::memory_order_release);
    }

    // RAMP, never a step. Latching is a button edge, and letting the weapon's heading jump on that
    // edge is a hard snap in the middle of aiming. The ramp is short enough to feel immediate and
    // long enough that the transition is a sweep rather than a cut.
    const float target = now ? 1.0f : 0.0f;
    const float secs = (g_cfg.two_hand_blend_ms > 1.0f) ? (g_cfg.two_hand_blend_ms * 0.001f) : 0.001f;
    float blend = g_th_blend.load(std::memory_order_relaxed);
    const float step = (dt > 0.0f ? dt : 0.0f) / secs;
    if (blend < target)      blend = (blend + step > target) ? target : blend + step;
    else if (blend > target) blend = (blend - step < target) ? target : blend - step;
    g_th_blend.store(clampf(blend, 0.0f, 1.0f), std::memory_order_release);
}

bool palette_two_hand_blend(Vec3* fwd) {
    if (fwd == nullptr) return false;
    const float blend = g_th_blend.load(std::memory_order_acquire);
    if (!(blend > 0.0f)) return false;
    if (!s_line_valid.load(std::memory_order_acquire)) return false;

    Vec3 line{s_line_x.load(std::memory_order_relaxed),
              s_line_y.load(std::memory_order_relaxed),
              s_line_z.load(std::memory_order_acquire)};
    if (!norm3(&line)) return false;

    // THE AGREEMENT BAND.
    //
    // The hand line is only a sane heading while it broadly agrees with where the weapon already
    // points. It stops agreeing in the ordinary course of play: you lower the support hand, you
    // reach across yourself, the hand passes behind the gun. Handing full authority to the line at
    // those moments swings the weapon to wherever your hands happen to lie.
    //
    // A hard cutoff is worse than no gate at all, because the hold is LATCHED -- a latched hand
    // crossing a threshold flips the weapon between two headings that can be most of a right angle
    // apart, in one frame. So the influence fades in across a band instead, and is smoothstepped
    // so it also leaves and arrives with zero slope.
    const float agree = dot3(line, *fwd);
    const float lo = g_cfg.two_hand_agree_min;
    const float hi = (g_cfg.two_hand_agree_full > lo + 1.0e-3f) ? g_cfg.two_hand_agree_full
                                                                : lo + 1.0e-3f;
    if (!(agree > lo)) return false;

    float t = clampf((agree - lo) / (hi - lo), 0.0f, 1.0f);
    t = t * t * (3.0f - 2.0f * t);
    const float w = blend * t;
    if (!(w > 0.0f)) return false;

    Vec3 out{fwd->x + (line.x - fwd->x) * w,
             fwd->y + (line.y - fwd->y) * w,
             fwd->z + (line.z - fwd->z) * w};
    if (!norm3(&out)) return false;
    *fwd = out;
    return true;
}

// Shortest-arc rotation carrying unit vector `a` onto unit vector `b`.
//
// Shortest arc specifically: it introduces no twist about the resulting axis, which is what lets
// the caller keep the original roll. A rotation built from Euler differences would smuggle one in.
namespace {
Quat quat_between(const Vec3& a, const Vec3& b) {
    const float d = dot3(a, b);
    if (d > 0.999999f) return Quat{0.0f, 0.0f, 0.0f, 1.0f};
    if (d < -0.999999f) {
        // Exactly opposed: the arc is a half turn about ANY perpendicular axis, so pick one that
        // is definitely not parallel to `a` rather than trusting a cross product that is zero here.
        Vec3 axis{-a.y, a.x, 0.0f};
        if (dot3(axis, axis) < 1.0e-6f) axis = Vec3{0.0f, -a.z, a.y};
        if (!norm3(&axis)) return Quat{0.0f, 0.0f, 0.0f, 1.0f};
        return Quat{axis.x, axis.y, axis.z, 0.0f};
    }
    const Vec3 c{a.y * b.z - a.z * b.y,
                 a.z * b.x - a.x * b.z,
                 a.x * b.y - a.y * b.x};
    const float s = std::sqrt((1.0f + d) * 2.0f);
    if (!(s > 1.0e-6f)) return Quat{0.0f, 0.0f, 0.0f, 1.0f};
    const float inv = 1.0f / s;
    return Quat{c.x * inv, c.y * inv, c.z * inv, s * 0.5f};
}
} // namespace

bool palette_two_hand_delta(Quat* out) {
    if (out == nullptr) return false;
    *out = Quat{0.0f, 0.0f, 0.0f, 1.0f};
    if (!(g_th_blend.load(std::memory_order_acquire) > 0.0f)) return false;

    const auto aim_idx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                             : API::VR::get_right_controller_index();
    if (aim_idx < 0) return false;

    // The SAME forward the zone is measured against and the aim is derived from. Taking it from
    // anywhere else would compare the hand line to an axis it has no relationship with.
    Vec3 fwd{}, grip_pos{};
    if (!aim_hand_forward(aim_idx, &fwd, &grip_pos)) return false;

    Vec3 blended = fwd;
    if (!palette_two_hand_blend(&blended)) return false;
    *out = quat_between(fwd, blended);
    return true;
}

} // namespace halo
