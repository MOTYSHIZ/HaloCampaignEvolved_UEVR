#include "Vehicle.hpp"

#include "Arms.hpp"
#include "BlamDrive.hpp"
#include "core/UnitState.hpp"
#include "Config.hpp"
#include "Markers.hpp"
#include "core/MarkerFaces.hpp"
#include "Math.hpp"
#include "MotionAimControl.hpp"
#include "Rig.hpp"
#include "UeObject.hpp"

#include <cmath>

using uevr::API;

namespace halo {

std::atomic<bool>  g_veh_active{false};
std::atomic<float> g_veh_steer{0.0f};
std::atomic<float> g_veh_thr{0.0f};
std::atomic<bool>  g_veh_thr_on{false};
std::atomic<float> g_veh_heading{0.0f};
std::atomic<bool>  g_veh_heading_valid{false};
std::atomic<float> g_veh_speed{0.0f};
std::atomic<float> g_unit_vx{0.0f}, g_unit_vy{0.0f};
std::atomic<uint32_t> g_veh_writes{0};

namespace {

bool  s_held = false;          // wheel currently gripped
float s_steer = 0.0f;          // smoothed output
bool  s_hand_r = false, s_hand_l = false;   // hand currently on the wheel (hysteresis)
// Which hands held the wheel last frame (for reference-preserving hand changes), and the steer at
// the moment of full release (for the hand-over-hand grace re-grab).
bool  s_cfg_r = false, s_cfg_l = false;
// Per-hand rotation accumulators: each hand carries its OWN total rotation since it grabbed,
// and the wheel takes their mean. UNWRAPPED, so a lock past 180 deg is reachable.
float s_acc_r = 0.0f, s_acc_l = 0.0f;
float s_prev_r = 0.0f, s_prev_l = 0.0f;
float s_wheel_delta = 0.0f;
float s_release_steer = 0.0f;
float s_release_age = 1e9f;
bool  s_logged_mount = false;

bool grip_held(bool right) {
    static UEVR_ActionHandle grip = nullptr;
    if (grip == nullptr) grip = API::VR::get_action_handle("/actions/default/in/Grip");
    if (grip == nullptr) return false;
    return API::VR::is_action_active(grip, right ? API::VR::get_right_joystick_source()
                                                 : API::VR::get_left_joystick_source());
}

void haptic(bool right, float dur, float amp) {
    if (!g_cfg.holster_haptic) return;
    API::VR::trigger_haptic_vibration(0.0f, dur, 0.0f, amp,
                                      right ? API::VR::get_right_joystick_source()
                                            : API::VR::get_left_joystick_source());
}

float wrap_pi(float a) {
    while (a >  3.14159265f) a -= 6.28318531f;
    while (a < -3.14159265f) a += 6.28318531f;
    return a;
}

} // namespace

void vehicle_reset() {
    g_veh_heading_valid.store(false, std::memory_order_relaxed);
    g_veh_speed.store(0.0f, std::memory_order_relaxed);
    if (s_held) { g_veh_active.store(false, std::memory_order_relaxed); }
    s_held = false; s_steer = 0.0f;
    s_hand_r = s_hand_l = false;
    g_veh_active.store(false, std::memory_order_relaxed);
    g_veh_thr_on.store(false, std::memory_order_relaxed);
}

// THE VEHICLE'S HEADING, FROM ITS OWN MOTION.
//
// Nothing in the object read as a usable facing from the seat (measured 2026-08-20), so the
// heading comes from where the vehicle actually goes. Position is already published every sim
// tick for the seat camera, so this costs the sim thread NOTHING -- the first attempt walked the
// object table from that thread on every publish and shook the whole picture, on foot included.
//
// The VELOCITY VECTOR is smoothed, not the angle: averaging angles across the +-180 seam swings
// the result half a turn, and a heading recomputed in 0.1 s chunks and written every frame is
// exactly the 10 Hz stepping that made v1 unusable. Vector EMA is continuous and seam-free.
void update_heading(float dt) {
    static float px = 0.0f, py = 0.0f; static bool have = false;
    static float vx = 0.0f, vy = 0.0f;
    if (!g_unit_pvalid.load(std::memory_order_relaxed) || !(dt > 0.0f) || dt > 0.25f) { have = false; return; }
    const float x = g_unit_px.load(std::memory_order_relaxed);
    const float y = g_unit_py.load(std::memory_order_relaxed);
    if (!have) { px = x; py = y; have = true; return; }
    const float rawx = (x - px) / dt, rawy = (y - py) / dt;   // world units/sec
    px = x; py = y;
    // 0.12 s, down from 0.25: the two smoothing stages (this EMA + the render-side ease) stack,
    // and at 0.25+0.20 the view trailed a turning hog by close to half a second. The position
    // source steps at the ~30 Hz sim rate, and two poles at ~0.1 s each still bury those steps.
    const float a = (dt / 0.12f > 1.0f) ? 1.0f : dt / 0.12f;
    vx += (rawx - vx) * a;
    vy += (rawy - vy) * a;
    const float speed = std::sqrt(vx * vx + vy * vy);
    g_veh_speed.store(speed, std::memory_order_relaxed);
    // Published for the seat camera: the position source steps at the sim tick while the vehicle
    // is RENDERED interpolated every frame, so the camera has to be projected forward between
    // ticks to sit still relative to the hog. Smoothing it instead (the first attempt) just put
    // the camera on a different curve from the hog, which read as judder on the nearest thing to
    // your face.
    g_unit_vx.store(vx, std::memory_order_relaxed);
    g_unit_vy.store(vy, std::memory_order_relaxed);
    // Below driving speed the EMA vector is mostly noise (measured whipping the heading +-150 deg
    // inside a second while creeping) -- so the heading only ARMS at 1.0 wu/s and only keeps
    // UPDATING above 0.5 (asymmetric so it cannot chatter at the boundary). Below that the last
    // heading holds, which is also the parked behaviour.
    const bool was_valid = g_veh_heading_valid.load(std::memory_order_relaxed);
    if (speed < (was_valid ? 0.50f : 1.00f)) return;
    // UE direction from the Blam frame is (x, -y). This is the RAW travel direction: no reverse
    // flip here. The old stick-back flip misfired constantly because this title's throttle is
    // camera-relative, not vehicle-relative (proven in the log -- heading exactly 180 off a
    // well-fitted forward travel for 2+ s). The render side resolves the hemisphere by
    // continuity with the view instead.
    float h = std::atan2(-vy, vx) * 57.2957795f;
    while (h > 180.0f) h -= 360.0f;
    while (h < -180.0f) h += 360.0f;
    g_veh_heading.store(h, std::memory_order_relaxed);
    g_veh_heading_valid.store(true, std::memory_order_relaxed);
}

// ---------------------------------------------------------------- WHEEL MARKERS
//
// The driver steers blind: the third-person body is hidden (scale trick) and the FP arms do not
// exist in vehicles, so there is NO hand representation in the seat -- and the grab zone is
// invisible. Two markers fix it: a ring where the zone is, a dot per steering hand. The marker
// machinery itself lives in Markers.cpp; this is just the wheel's use of it.
namespace {
TrackedObject s_mark_ring, s_mark_dot, s_mark_dot2;
bool s_mark_failed = false;

void wheel_markers_update(bool active) {
    const bool want = active && g_cfg.veh_wheel_marker != 0;
    auto* ring = s_mark_ring.get();
    auto* dot  = s_mark_dot.get();
    if (!want) {
        if (ring != nullptr) holster_marker_show(ring, false);
        if (dot  != nullptr) holster_marker_show(dot, false);
        holster_marker_show(s_mark_dot2.get(), false);
        return;
    }
    if ((ring == nullptr || dot == nullptr) && !s_mark_failed) {
        const uintptr_t hp = g_hog_body_ptr.load(std::memory_order_relaxed);
        auto* hull = reinterpret_cast<API::UObject*>(hp);
        auto* owner = (hull != nullptr) ? hull->get_outer() : nullptr;
        if (owner == nullptr) return;               // hull not resolved yet; retry next tick
        // ONLY LOADED ASSETS ARE FINDABLE: find_uobject does not load on demand. The Sphere and
        // Cube are proven loaded in vehicles; the chain ends in meshes that cannot miss, and a
        // squashed sphere reads as a disc anyway.
        static const wchar_t* kRing[] = {
            L"StaticMesh /Engine/BasicShapes/Torus.Torus",
            L"StaticMesh /Engine/BasicShapes/Cylinder.Cylinder",
            L"StaticMesh /Engine/BasicShapes/Sphere.Sphere",
            L"StaticMesh /Engine/BasicShapes/Cube.Cube",
        };
        static const wchar_t* kDot[] = { L"StaticMesh /Engine/BasicShapes/Sphere.Sphere" };
        // Ring: sized to the zone diameter (BasicShapes are ~100 cm), squashed thin so a cylinder
        // fallback reads as a disc rather than a drum. Dot: ~4 cm.
        const double rs = (double)(g_cfg.veh_wheel_radius * 2.0f);
        if (ring == nullptr) { auto* c = marker_spawn_list(owner, kRing, 4, rs, rs, 0.04); if (c) s_mark_ring.set(c); }
        if (dot == nullptr)  { auto* c = marker_spawn_list(owner, kDot, 1, 0.04, 0.04, 0.04); if (c) s_mark_dot.set(c); }
        if (s_mark_dot2.get() == nullptr) { auto* c = marker_spawn_list(owner, kDot, 1, 0.04, 0.04, 0.04); if (c) s_mark_dot2.set(c); }
        ring = s_mark_ring.get(); dot = s_mark_dot.get();
        if (ring == nullptr && dot == nullptr) { s_mark_failed = true; return; }
        API::get()->log_info("[Halo-CampE-UEVR] WHEEL: markers spawned on %s",
                             narrow(class_name_of(owner)).c_str());
    }
    Vec3 hmd{}; Quat hq{};
    if (!get_pose(API::VR::get_hmd_index(), &hmd, &hq, false)) return;
    auto* hull_now = reinterpret_cast<API::UObject*>(g_hog_body_ptr.load(std::memory_order_relaxed));
    if (hull_now == nullptr) return;
    if (ring != nullptr) {
        holster_marker_show(ring, true);
        // THE ZONE CENTRE IS A CONSTANT IN THE HOG'S FRAME -- no room frame, no head term. The
        // old path ran it through room_to_world, which subtracts the CURRENT head position and
        // made a bolted-down point head-relative, so looking left slid it right. Converted to
        // world through the hull transform, and oriented by the HULL's yaw rather than the view's.
        const Vec3 lr{g_cfg.veh_wheel_pos[0] * 100.0f, g_cfg.veh_wheel_pos[1] * 100.0f,
                      g_cfg.veh_wheel_pos[2] * 100.0f};
        const double rs = (double)(g_cfg.veh_wheel_radius * 2.0f);
        // Scale applied EVERY TICK, not once at spawn: anything a live cfg value controls has to
        // be re-applied on the same cadence the value can change, or the dial answers to nothing.
        marker_scale3(ring, rs, rs, 0.04);
        Vec3 rw{}; float hyaw = 0.0f;
        if (markers_hull_local_to_world(hull_now, lr, &rw, &hyaw))
            marker_place_rot(ring, rw, -(90.0f - g_cfg.veh_wheel_tilt), hyaw);
    }
    // A dot per steering hand -- with vehwheelhand=2 both hands drive, so both get one.
    auto place_dot = [&](API::UObject* d, bool right) {
        if (d == nullptr) return;
        Vec3 hpos{}; Quat hrot2{};
        const auto idx = right ? API::VR::get_right_controller_index()
                               : API::VR::get_left_controller_index();
        if (idx >= 0 && get_pose(idx, &hpos, &hrot2, false)) {
            holster_marker_show(d, true);
            holster_marker_place(d, holster_room_to_world(hpos, hmd));
        } else holster_marker_show(d, false);
    };
    const int hand_mode = g_cfg.veh_wheel_hand;
    place_dot(dot, hand_mode == 1);
    if (hand_mode == 2) place_dot(s_mark_dot2.get(), true);
    else holster_marker_show(s_mark_dot2.get(), false);
}
} // namespace

void vehicle_update(float dt) {
    // The HEADING feeds the view anchor and the seat camera, not just the wheel, so this runs
    // whenever ANY vehicle feature is on. Gating the whole function on vehiclewheel meant that
    // switching the steering off silently killed the heading the camera depends on.
    const bool want_wheel = (g_cfg.vehicle_wheel != 0);
    const bool want_any = want_wheel || g_cfg.veh_view != 0 || g_cfg.veh_cam != 0;
    if (!g_cfg.enabled || !want_any) { vehicle_reset(); return; }
    seat_direct_refresh();   // vehseatdirect: live rider read that does not wait on the sim hook
    // Only while actually seated in something: the mounted flag is the biped's parent datum
    // (+0x0C != 0xFFFFFFFF), published by BlamDrive -- see the measurement note there.
    if (!g_unit_mounted.load(std::memory_order_relaxed)) {
        if (s_logged_mount) { s_logged_mount = false; API::get()->log_info("[Halo-CampE-UEVR] WHEEL: dismounted"); }
        wheel_markers_update(false);
        vehicle_reset();
        return;
    }
    update_heading(dt);
    if (!s_logged_mount) {
        s_logged_mount = true;
        API::get()->log_info("[Halo-CampE-UEVR] WHEEL: mounted -- grip inside the wheel zone to steer");
    }
    if (!want_wheel) { wheel_markers_update(false); g_veh_active.store(false, std::memory_order_relaxed); return; }
    if (!(dt > 0.0f) || dt > 0.25f) return;

    // ---- EVERYTHING IN THE HOG'S FRAME.
    //
    // Two earlier frames were tried and both failed from the seat: head-relative (lean an inch
    // and the invisible wheel moves with your face, away from the hand reaching for it) and a
    // room-yaw latch at mount (unless you mount looking exactly down the bonnet, the zone's
    // forward runs diagonal to the vehicle and every axis bleeds into the others -- the 45 cm of
    // "right" that setup needed was compensation for a rotated frame, not an offset).
    //
    // The hull's transform is the frame that cannot be wrong: the wheel is bolted to the hog, so
    // its position is a CONSTANT in hull space. Hands go room -> world -> hull-local, the same
    // path the markers take, so ring and grab zone are the same arithmetic by construction.
    // vehwheelpos means (forward, right, up) in the VEHICLE, from the hull origin. Mounting
    // crooked, leaning, or looking around cannot move it.
    auto* hull_g = reinterpret_cast<API::UObject*>(g_hog_body_ptr.load(std::memory_order_relaxed));
    if (hull_g == nullptr) { wheel_markers_update(false); return; }   // hull not resolved yet
    Vec3 hmd{}; Quat hmdq{};
    if (!get_pose(API::VR::get_hmd_index(), &hmd, &hmdq, false)) return;
    wheel_markers_update(true);

    auto hand_hull_local = [&](bool right, Vec3* out) {
        Vec3 pos{}; Quat rot{};
        const auto idx = right ? API::VR::get_right_controller_index()
                               : API::VR::get_left_controller_index();
        if (idx < 0 || !get_pose(idx, &pos, &rot, false)) return false;
        if (std::fabs(pos.x) < 1e-6f && std::fabs(pos.y) < 1e-6f && std::fabs(pos.z) < 1e-6f) return false;
        return markers_world_to_hull_local(hull_g, holster_room_to_world(pos, hmd), out);
    };
    Vec3 hr{}, hl{};
    const bool have_r = hand_hull_local(true,  &hr);
    const bool have_l = hand_hull_local(false, &hl);

    // BOTH HANDS, ALWAYS, WHILE MOUNTED. The grab-gated version was a catch-22: the position log
    // only fired on a successful grab, and a grab needs the zone, and the zone is what is being
    // located. Logged unconditionally, locating the wheel costs one touch: rest a hand on the
    // drawn wheel, read the line, and those three numbers ARE vehwheelpos.
    if (g_cfg.veh_log) {
        static uint32_t wp = 0;
        if ((wp++ % 60u) == 0u)
            API::get()->log_info("[Halo-CampE-UEVR] WHEEL-POS L=(%.2f %.2f %.2f) R=(%.2f %.2f %.2f) m "
                                 "hull frame (fwd,right,up) -- rest a hand ON the drawn wheel, that is vehwheelpos",
                                 have_l ? hl.x / 100.0f : 0.0f, have_l ? hl.y / 100.0f : 0.0f,
                                 have_l ? hl.z / 100.0f : 0.0f,
                                 have_r ? hr.x / 100.0f : 0.0f, have_r ? hr.y / 100.0f : 0.0f,
                                 have_r ? hr.z / 100.0f : 0.0f);
    }

    // vehwheelpos is METRES in the hull frame (x fwd, y right, z up); the hull works in cm.
    const Vec3 wc{g_cfg.veh_wheel_pos[0] * 100.0f, g_cfg.veh_wheel_pos[1] * 100.0f,
                  g_cfg.veh_wheel_pos[2] * 100.0f};
    const bool need_grip = (g_cfg.veh_wheel_grip != 0);
    const bool gr = need_grip ? grip_held(true)  : true;
    const bool gl = need_grip ? grip_held(false) : true;
    // THE CATCH REACHES BEYOND THE RIM. With the sphere exactly the disc's radius, a hand ON
    // the rim sits at the boundary and flickers in and out -- which is why the log read "one
    // hand" on nearly every two-handed grab. You grab a rim from outside it, so the catch is
    // 1.3x the visual radius: the disc shows the wheel, the zone is the wheel plus a hand.
    const float r_in = g_cfg.veh_wheel_radius * 130.0f;   // metres -> cm, x1.3 for the rim
    const float r_out = r_in * 1.6f;
    auto within = [&](const Vec3& h, float r) {
        const float dx = h.x - wc.x, dy = h.y - wc.y, dz = h.z - wc.z;
        return (dx * dx + dy * dy + dz * dz) <= (r * r);
    };
    // vehwheelhand: 0 = left only, 1 = right only, 2 = BOTH -- two hands on the wheel, per-hand
    // accumulators below. One-handed driving was the complaint, not the design goal.
    const bool allow_r = (g_cfg.veh_wheel_hand != 0);
    const bool allow_l = (g_cfg.veh_wheel_hand != 1);
    // GRIP IS A LATCH, NOT A PROXIMITY TEST.
    //
    // Proximity decides when you TAKE hold; after that only releasing the grip lets go. The old
    // version re-tested distance every frame, and the zone is bolted to the hull -- so a kick
    // swings the wheel out from under hands that never moved, and the grip silently dies
    // mid-corner. The seat is ~2.4 m from the hull origin, so hull pitch swings the zone through
    // a long lever -- field-observed as "when the hog kicks and I shift, I am out of the zone".
    // Physically it is also just wrong: your hands do not let go of a wheel because the truck
    // bounced.
    //
    // Held, the hand may go anywhere: the angle is still measured about the wheel centre and each
    // hand accumulates its own rotation, so a hand that has drifted off the rim keeps steering
    // exactly as it did. Only with vehwheelgrip=0 (no grip required) does proximity still govern
    // staying, because then there is no button whose release could mean "let go".
    const bool r_stay = need_grip ? true : within(hr, r_out);
    const bool l_stay = need_grip ? true : within(hl, r_out);
    const bool r_on = allow_r && have_r && gr && (s_hand_r ? r_stay : within(hr, r_in));
    const bool l_on = allow_l && have_l && gl && (s_hand_l ? l_stay : within(hl, r_in));
    s_hand_r = r_on; s_hand_l = l_on;

    // THE WHEEL PLANE: tilted vehwheeltilt degrees back from vertical about the lateral axis,
    // like a car's. The angle lives in (u,v); the component along the column normal is ignored,
    // so pushing the hand THROUGH the wheel does not steer.
    const float tilt = g_cfg.veh_wheel_tilt * 3.14159265f / 180.0f;
    const float ct = std::cos(tilt), st = std::sin(tilt);
    // SIGN CONVENTION (fixed in the headset): positive tilt = the TOP of the wheel leans TOWARD
    // the driver, which is how the hog's wheel is drawn. The marker uses the same sign, so the
    // ring you see and the plane the hand is measured in cannot disagree.
    // In the hull frame the wheel faces FORWARD (+x); its face spans right(+y) and up(+z).
    auto plane_uv = [&](const Vec3& h, float* pu, float* pv) {
        const float dfw = h.x - wc.x, dr = h.y - wc.y, du = h.z - wc.z;
        *pu = dr;
        *pv = du * ct + dfw * st;
    };
    // ---- EACH HAND CARRIES ITS OWN ROTATION; THE WHEEL TAKES THE MEAN.
    //
    // Averaging the two hands' POSITION angles about the hub has a degeneracy exactly where real
    // driving lives: hands at 9 and 3 are ~180 deg apart, and the circular mean of two antipodal
    // angles is ambiguous -- a millimetre of tracking noise swings it 90 deg. (The version before
    // that used the line between the hands, which fails the opposite way: hands close together
    // give a short baseline and wild angles.)
    //
    // Accumulating per hand has no degenerate configuration at all. Each held hand tracks its own
    // total rotation about the hub since it grabbed, and the wheel angle is the mean over the held
    // hands. Both together -> full rate. One moving while the other holds -> half rate, and the
    // held hand genuinely resists. That is "it rotates at the rate both my hands do", literally.
    float ar = 0.0f, al = 0.0f;
    if (r_on) { float u, v; plane_uv(hr, &u, &v); ar = std::atan2(u, v); }
    if (l_on) { float u, v; plane_uv(hl, &u, &v); al = std::atan2(u, v); }
    const bool have_angle = (r_on || l_on);

    if (!have_angle) {
        if (s_held) {
            s_held = false;
            s_release_steer = s_steer;
            s_release_age = 0.0f;
            s_cfg_r = s_cfg_l = false;
            g_veh_active.store(false, std::memory_order_relaxed);
            g_veh_thr_on.store(false, std::memory_order_relaxed);
            if (g_cfg.veh_log) API::get()->log_info("[Halo-CampE-UEVR] WHEEL: released");
        }
        s_release_age += dt;
        return;
    }

    // THE WHEEL'S ANGLE SURVIVES ANY CHANGE OF HANDS. An earlier latch re-referenced AND zeroed
    // the steer on every new grab, so landing a second hand mid-corner recentred the wheel and
    // handed the maths to whichever configuration now held -- "it uses the hand I gripped last".
    // And dropping from two hands to one never re-referenced at all, so the angle convention
    // switched under a stale reference and the steer jumped.
    //
    // Now: any change in WHICH hands hold (one<->two, left<->right) re-references such that the
    // current steer is preserved exactly. Only a grab from fully-released centres the wheel, and
    // even that keeps a 0.7 s grace in which the released steer is restored, so hand-over-hand
    // through a long corner holds the turn.
    const float lock = (g_cfg.veh_wheel_lock > 5.0f ? g_cfg.veh_wheel_lock : 90.0f) * 3.14159265f / 180.0f;
    const float signf = (g_cfg.veh_steer_sign < 0) ? -1.0f : 1.0f;
    const bool was_r = s_cfg_r, was_l = s_cfg_l;
    const bool config_changed = (r_on != was_r) || (l_on != was_l);
    if (!s_held) {
        s_held = true;
        // A re-grab inside the grace keeps the turn; otherwise the wheel centres here.
        const float carry = (s_release_age < 0.7f) ? s_release_steer : 0.0f;
        s_steer = carry;
        s_wheel_delta = (carry * signf) * lock;
        haptic(r_on, 0.06f, 0.5f);
        if (g_cfg.veh_log) API::get()->log_info("[Halo-CampE-UEVR] WHEEL: grabbed (%s)%s",
                                                (r_on && l_on) ? "two hands" : "one hand",
                                                carry != 0.0f ? " -- turn carried" : "");
    } else if (config_changed) {
        haptic(r_on, 0.03f, 0.3f);
        if (g_cfg.veh_log) API::get()->log_info("[Halo-CampE-UEVR] WHEEL: hands changed (%s) -- steer preserved",
                                                (r_on && l_on) ? "two" : (r_on ? "right" : "left"));
    }
    // A hand that has just taken hold starts from the wheel's CURRENT rotation, so adding or
    // dropping a hand mid-corner cannot move the wheel by even a degree.
    if (r_on && !was_r) { s_acc_r = s_wheel_delta; s_prev_r = ar; }
    if (l_on && !was_l) { s_acc_l = s_wheel_delta; s_prev_l = al; }
    s_cfg_r = r_on; s_cfg_l = l_on;
    if (r_on) { s_acc_r += wrap_pi(ar - s_prev_r); s_prev_r = ar; }
    if (l_on) { s_acc_l += wrap_pi(al - s_prev_l); s_prev_l = al; }
    s_wheel_delta = (r_on && l_on) ? (s_acc_r + s_acc_l) * 0.5f : (r_on ? s_acc_r : s_acc_l);

    float steer = s_wheel_delta / lock;
    if (steer >  1.0f) steer =  1.0f;
    if (steer < -1.0f) steer = -1.0f;
    steer *= (g_cfg.veh_steer_sign < 0) ? -1.0f : 1.0f;

    const float a = (dt / 0.05f > 1.0f) ? 1.0f : dt / 0.05f;
    s_steer += (steer - s_steer) * a;

    g_veh_steer.store(s_steer, std::memory_order_relaxed);
    g_veh_active.store(true, std::memory_order_relaxed);
    g_veh_thr_on.store(false, std::memory_order_relaxed);   // throttle stays on the stick

    if (g_cfg.veh_log) {
        static uint32_t n = 0;
        static uint32_t prev_writes = 0;
        if ((n++ % 30u) == 0u) {
            const uint32_t w = g_veh_writes.load(std::memory_order_relaxed);
            // writes: how many steering values actually reached the sim since the last line. Zero
            // with a live steer value means the WRITE is the problem, not the gesture.
            API::get()->log_info("[Halo-CampE-UEVR] WHEEL-POS hand_hull=(%.2f %.2f %.2f)m",
                                 (r_on ? hr.x : hl.x) / 100.0f, (r_on ? hr.y : hl.y) / 100.0f,
                                 (r_on ? hr.z : hl.z) / 100.0f);
            API::get()->log_info("[Halo-CampE-UEVR] WHEEL: angle=%.0fdeg steer=%+.2f (%s) writes=%u(+%u) facing=(%.3f %.3f)",
                                 s_wheel_delta * 57.2958f, s_steer,
                                 (r_on && l_on) ? "2h" : "1h", w, w - prev_writes,
                                 g_unit_fx.load(std::memory_order_relaxed),
                                 g_unit_fy.load(std::memory_order_relaxed));
            prev_writes = w;
        }
    }
}

} // namespace halo
