#include "features/vehcam/VehCam.hpp"
#include "core/config/CfgRead.hpp"

#include "BlamDrive.hpp"          // blam_control_record(): the VEHSEAT line's record flag
#include "Config.hpp"
#include "core/Services.hpp"
#include "core/ViewState.hpp"
#include "Markers.hpp"
#include "Math.hpp"
#include "MotionAimControl.hpp"   // get_pose, g_stick_mode_active
#include "HitTrace.hpp"           // hit_trace(): camera-collision spring arm (game-thread)
#include "Reticule.hpp"           // g_ret_scale_mul: the per-frame vehicle reticule stamp's apparent size
#include "XrLayer.hpp"            // xrlayer_notice_reticule: the per-frame vehicle reticule stamp
#include "features/vehcam/VehCamMath.hpp"      // rotator <-> axes, the per-ride vehicle frame (tested out of tree)
#include "features/vehcam/VehCamPresets.hpp"   // the camera file's types (CamType, Origin, Rides)
#include "features/vehcam/VehCamSelect.hpp"    // the selected camera: veh_active_cam()
#include "Rig.hpp"
#include "UeObject.hpp"
#include "core/MarkerFaces.hpp"
#include "core/UnitState.hpp"
#include "core/fixes/TickStage.hpp"
#include "core/host/ArmsState.hpp"
#include "core/host/PluginState.hpp"
#include "uevr/API.hpp"

#include <Windows.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cwchar>
#include <cwctype>
#include <string>

using uevr::API;

namespace halo {

using vehcammath::rot_axes;
using vehcammath::rotator_from_axes;
using vehcammath::capture_vehicle_frame;
using vehcammath::vehicle_axes;
using vehcammath::tracked_frame;

// Defined with the third-person camera below; bc24's seat-camera gates above it consult them.
bool veh_fp_selected();
int  veh_cam_mode(int cfg_veh_cam);

// ================================================================================================
// THE WHEEL AND THE HEADING (game thread).
// ================================================================================================

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
    // switching the steering off silently killed the heading the camera depends on. vehview is not
    // one of them: it is a setting of the seat camera (the view override reads it only with vehcam
    // on), so on its own it runs nothing here.
    const bool want_wheel = (g_cfg.vehicle_wheel != 0);
    const bool want_any = want_wheel || veh_cam_mode(g_cfg.veh_cam) != 0;
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

// ================================================================================================
// THE DRIVER BODY HIDE AND THE HULL RESOLVE (game thread), read by the seat camera and the wheel.
// ================================================================================================

namespace {

// SetRelativeScale3D(FVector NewScale3D). LWC: three DOUBLES, not floats -- Hands.cpp pays for
// that distinction already and getting it wrong writes garbage into the first two components.
void call_set_scale(API::UObject* comp, double sc) {
    if (comp == nullptr) return;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    auto* d = reinterpret_cast<double*>(p);
    d[0] = d[1] = d[2] = sc;
    comp->call_function(L"SetRelativeScale3D", p);
}

// ---------------------------------------------------------------- DRIVER BODY HIDE
//
// WHAT IT IS, measured 2026-08-21 by a ranked sweep around the rendered eye:
//
//   BP_SpartansBipedActor_C_<id>.Body     SkeletalMeshComponent, SK_Spartans_AnimDynamics
//
// That actor is the player. With the camera at the seat your head is inside it, so it clips
// constantly. MATCHED BY OUTER CHAIN, NOT BY CLASS: the class is plain "SkeletalMeshComponent",
// shared with every marine, weapon and NPC in the level. The full name carries the owning actor,
// so "SpartansBipedActor" + ".Body" is the thing that actually identifies it; the instance id
// changes per load, so it is deliberately not part of the match.
bool is_driver_body(const std::wstring& full) {
    return full.find(L"SpartansBipedActor") != std::wstring::npos
        && full.size() >= 5 && full.compare(full.size() - 5, 5, L".Body") == 0;
}

// Pull "BP_SpartansBipedActor_C_<id>" out of a full name. The instance id changes every load,
// so the ACTOR TOKEN has to be read from a component we already matched.
std::wstring driver_actor_token(const std::wstring& full) {
    const size_t k = full.find(L"SpartansBipedActor");
    if (k == std::wstring::npos) return L"";
    const size_t start = full.rfind(L'.', k);
    const size_t end = full.find(L'.', k);
    if (end == std::wstring::npos) return L"";
    const size_t from = (start == std::wstring::npos) ? 0 : start + 1;
    if (end <= from) return L"";
    return full.substr(from, end - from);
}

// EVERY mesh component on the driver actor, not just .Body. These are modular characters (the
// marines nearby are SIX components each), so the Spartan is one too and .Body is one piece.
constexpr int kMaxDriverParts = 24;
TrackedObject s_driver_parts[kMaxDriverParts];
int           s_driver_part_count = 0;
bool          s_driver_hidden = false;
int           s_driver_tries = 0; ULONGLONG s_driver_try_at = 0;

// Resolve the driver actor, then collect its parts. Enumerates every Spartan body with distance
// from BOTH references and SELECTS ON d_blam: the Blam unit position is the player's biped by
// definition, while the eye is only meaningful after the vehicle camera has run -- and at the
// mount edge it has not. (Selecting on the eye once picked a Spartan 847 cm away.)
int resolve_driver_parts() {
    s_driver_part_count = 0;
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return 0;
    const double ex = (double)g_cam_x.load(std::memory_order_relaxed);
    const double ey = (double)g_cam_y.load(std::memory_order_relaxed);
    const double ez = (double)g_cam_z.load(std::memory_order_relaxed);
    const double S = 304.8;
    double bx =  (double)g_unit_px.load(std::memory_order_relaxed) * S;
    double by = -(double)g_unit_py.load(std::memory_order_relaxed) * S;
    double bz =  (double)g_unit_pz.load(std::memory_order_relaxed) * S;
    // No Blam rider position (it reads dead in some seats): the local pawn is the same body, and its
    // UE position is always there -- otherwise "nearest to the Blam position" is nearest to the world
    // origin, which picks an arbitrary Spartan wherever there is more than one.
    if (!g_unit_pvalid.load(std::memory_order_relaxed)) {
        Vec3 pl{};
        if (auto* pawn = API::get()->get_local_pawn(0); pawn != nullptr && call_ret_vec3(pawn, L"K2_GetActorLocation", &pl)) {
            bx = pl.x; by = pl.y; bz = pl.z;
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] VEHBODY: eye=(%.0f %.0f %.0f) blam=(%.0f %.0f %.0f)",
                         ex, ey, ez, bx, by, bz);
    const int32_t n = arr->get_object_count();
    std::wstring token; double bestd = 1e18;
    int spartans = 0;
    for (int32_t i = 0; i < n; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"SkeletalMeshComponent") continue;
        const std::wstring full = o->get_full_name();
        if (!is_driver_body(full)) continue;
        Vec3 w{};
        if (!call_ret_vec3(o, L"K2_GetComponentLocation", &w)) continue;
        const double de = std::sqrt(((double)w.x - ex) * ((double)w.x - ex)
                                  + ((double)w.y - ey) * ((double)w.y - ey)
                                  + ((double)w.z - ez) * ((double)w.z - ez));
        const double db = std::sqrt(((double)w.x - bx) * ((double)w.x - bx)
                                  + ((double)w.y - by) * ((double)w.y - by)
                                  + ((double)w.z - bz) * ((double)w.z - bz));
        ++spartans;
        API::get()->log_info("[Halo-CampE-UEVR] VEHBODY   spartan d_eye=%8.1f d_blam=%8.1f  %ls",
                             de, db, full.c_str());
        if (db < bestd) { bestd = db; token = driver_actor_token(full); }
    }
    API::get()->log_info("[Halo-CampE-UEVR] VEHBODY: %d spartan bodies in the level", spartans);
    if (token.empty()) return 0;
    for (int32_t i = 0; i < n && s_driver_part_count < kMaxDriverParts; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        const std::wstring cn = class_name_of(o);
        if (cn.find(L"Mesh") == std::wstring::npos) continue;
        if (cn.find(L"Component") == std::wstring::npos) continue;
        const std::wstring full = o->get_full_name();
        if (full.find(token) == std::wstring::npos) continue;
        s_driver_parts[s_driver_part_count++].set_at(o, i);
    }
    API::get()->log_info("[Halo-CampE-UEVR] VEHBODY: chose %ls at d_blam=%.1f -- %d mesh parts",
                         token.c_str(), bestd, s_driver_part_count);
    return s_driver_part_count;
}

} // namespace

// Reconciled EVERY TICK while mounted, not once on the transition: a component whose whole job
// is to drive this mesh from the Blam simulation is exactly the kind of thing that reasserts
// state underneath us, and re-applying every tick beats guessing.
//
// SCALE TO NOTHING, as well as hiding. The readback settled that SetHiddenInGame TAKES on this
// component (hid=1, held, nothing reverting it) and the Spartan still drew -- the mesh is drawn
// by something that does not consult UE visibility. A transform is not a visibility flag: the
// draw demonstrably honours scale, so 0.001 is what actually removes the body (vehhidebody=2).
void driver_hide_update() {
    // Arms.cpp's own visibility helpers, through the bridge.
    const auto call_set_hidden = host::g_arms_state.call_set_hidden;
    const auto call_set_visibility = host::g_arms_state.call_set_visibility;
    // The body is hidden because the SEAT CAMERA sits inside it. With vehcam off the view is the
    // stock chase camera, where hiding it just deletes the Spartan from the shot.
    //
    // OUR CAMERAS decide per camera (the camera file's "hideBody"; by default hidden exactly when the
    // camera sits at your seat): the Onboard and Cockpit views are inside the body, a chase view wants
    // to see it. Stepping between cameras shows and hides it live. Scale-to-nothing when vehhidebody
    // does not say otherwise -- mode 1 alone was measured insufficient (the body drew regardless).
    const VehActiveCam hide_cam = veh_active_cam();
    const bool tp_hide = g_veh_tp_active.load(std::memory_order_relaxed) && hide_cam.valid && hide_cam.hide_body
                      && halo::g_stick_mode_active.load(std::memory_order_relaxed);
    const bool seat_hide = g_cfg.veh_hide_body != 0 && veh_cam_mode(g_cfg.veh_cam) != 0
                        && (g_unit_mounted.load(std::memory_order_relaxed) || veh_fp_selected());
    const bool want = g_cfg.enabled && (tp_hide || seat_hide);
    const int  hide_mode = (g_cfg.veh_hide_body != 0) ? g_cfg.veh_hide_body : 2;

    // Still in the vehicle? Then the body we found is still the body: stepping between cameras shows
    // and hides it from the cached parts instead of walking the object array (twice) on every step.
    const bool still_seated = halo::g_stick_mode_active.load(std::memory_order_relaxed);
    if (!want) {
        if (s_driver_hidden) {
            for (int i = 0; i < s_driver_part_count; ++i) {
                if (auto* c = s_driver_parts[i].get()) {
                    call_set_hidden(c, false);
                    call_set_visibility(c, true);
                    call_set_scale(c, 1.0);   // unconditional: restore whatever mode did
                }
            }
            API::get()->log_info("[Halo-CampE-UEVR] VEHBODY: driver restored (%d parts)",
                                 s_driver_part_count);
            s_driver_hidden = false;
        }
        if (!still_seated) s_driver_part_count = 0;   // the next ride resolves afresh
        s_driver_tries = 0;
        return;
    }

    if (!s_driver_hidden && s_driver_part_count > 0) {
        bool any = false;
        for (int i = 0; i < s_driver_part_count && !any; ++i) any = s_driver_parts[i].get() != nullptr;
        if (any) {
            s_driver_hidden = true;
            API::get()->log_info("[Halo-CampE-UEVR] VEHBODY: driver hidden again (%d cached parts)",
                                 s_driver_part_count);
        } else {
            s_driver_part_count = 0;                  // gone (a respawn, a level change): resolve again
        }
    }

    if (!s_driver_hidden) {
        // Two full object-array walks per try: every 2 s, ten tries per mount, then it gives up
        // until the next mount (it retried every tick before, perf audit 2026-09-06).
        if (s_driver_tries >= 10) return;
        const ULONGLONG t = GetTickCount64();
        if (t - s_driver_try_at < 2000) return;
        s_driver_try_at = t; ++s_driver_tries;
        if (resolve_driver_parts() == 0) return;      // not resolvable yet; retry in 2 s
        s_driver_tries = 0;
        s_driver_hidden = true;
        API::get()->log_info("[Halo-CampE-UEVR] VEHBODY: driver hidden (%d parts)",
                             s_driver_part_count);
    }

    for (int i = 0; i < s_driver_part_count; ++i) {
        auto* c = s_driver_parts[i].get();
        if (c == nullptr) continue;
        rig_set_always_tick_pose(c);
        call_set_hidden(c, true);
        call_set_visibility(c, false);
        if (hide_mode == 2) call_set_scale(c, 0.001);
    }
    // Mode 2 -> 1 mid-ride: put the scale back once, or the parts stay shrunk under mode 1.
    {
        static int s_last_mode = 0;
        if (s_last_mode == 2 && hide_mode != 2) {
            for (int i = 0; i < s_driver_part_count; ++i)
                if (auto* c = s_driver_parts[i].get()) call_set_scale(c, 1.0);
        }
        s_last_mode = hide_mode;
    }

    // One readback a second, so the log answers "did it take" without inference.
    if (g_cfg.veh_log) {
        static uint32_t n = 0;
        if ((n++ % 90u) == 0u) {
            for (int i = 0; i < s_driver_part_count; ++i) {
                auto* c = s_driver_parts[i].get();
                if (c == nullptr) continue;
                int vis = -1, hid = -1;
                if (auto* v = c->get_property_data<bool>(L"bVisible")) vis = *v ? 1 : 0;
                if (auto* h = c->get_property_data<bool>(L"bHiddenInGame")) hid = *h ? 1 : 0;
                API::get()->log_info("[Halo-CampE-UEVR] VEHBODY   readback vis=%d hid=%d  %ls",
                                     vis, hid, c->get_full_name().c_str());
            }
        }
    }
}

// ---------------------------------------------------------------- HOG BODY RESOLVE
//
// The rigid vehicle camera needs the component the hog is drawn from, resolved on the GAME
// thread (this walk names 290k objects; the render callback must never pay that) and published
// as a pointer+slot pair the render side re-validates through TrackedObject each frame.
//
// The match is the Spartan pattern transplanted: a SkeletalMeshComponent on a
// "...VehicleActor_C_<id>" instance in the PersistentLevel, nearest the rider's Blam position.
std::atomic<uintptr_t> g_hog_body_ptr{0};
std::atomic<int32_t>   g_hog_body_idx{-1};

// ---- ROUTE A P0: owned THIRD-PERSON camera chassis. Resolved off the PLAYER PAWN's world
// position (the pawn sits in the vehicle; the Blam seat resolution reads dead), published game-side;
// the render callbacks read its transform FRESH each frame. The yaw is published by the eye
// callback (which already reads the transform) for the view override to consume.
std::atomic<uintptr_t> g_tp_chassis_ptr{0};
std::atomic<int32_t>   g_tp_chassis_idx{-1};
std::atomic<float>     g_tp_chassis_yaw{0.0f};   // the VIEW yaw: the view override + room_to_world read it
std::atomic<bool>      g_tp_chassis_yaw_valid{false};
// ...and the VIEW's pitch and roll, zero unless the selected camera tilts with the vehicle
// (viewFollows pitch/roll). The eye computes all three; the view override and the tick's controller
// ray read them, so the rendered view, the head's placement and the hand's ray share one rotation.
std::atomic<float>     g_tp_view_pitch{0.0f}, g_tp_view_roll{0.0f};
// The anchor's world OFFSET from its origin point, unscaled by the spring arm, as the eye built it last
// frame. The collision trace follows THIS, not the view yaw: the two diverge whenever the offset rides
// the vehicle, and a yaw alone cannot describe an offset that pitches and banks with it.
// An offset, not a position -- the tick adds its own fresh chassis location (two clocks).
std::atomic<float>     g_tp_boom_ox{0.0f}, g_tp_boom_oy{0.0f}, g_tp_boom_oz{0.0f};
// Where that offset is measured FROM, as a world offset from the chassis: zero for origin "vehicle",
// the seat for origin "seat". The collision trace starts there.
std::atomic<float>     g_tp_origin_ox{0.0f}, g_tp_origin_oy{0.0f}, g_tp_origin_oz{0.0f};
// THE SEAT, in the chassis MESH's own frame: the seated pawn's offset from the chassis, measured on the
// tick (pawn and chassis read in the same game state) and rebuilt by the eye against the mesh's live
// rotation, so it rides the vehicle rigidly at render rate. Updated every tick while in a vehicle: the
// enter animation carries the pawn into the seat over a second or so.
std::atomic<float>     g_tp_seat_x{0.0f}, g_tp_seat_y{0.0f}, g_tp_seat_z{0.0f};
std::atomic<bool>      g_tp_seat_valid{false};
// The VIEW BASE the eye handed UEVR last frame, as an offset from the chassis: the anchor, the spring
// arm and the head-offset subtraction all included. The ray aim rebuilds the controller's world ray on
// the tick from it plus the tick's own chassis read -- the same two-clocks split as the boom offset.
std::atomic<float>     g_tp_eye_ox{0.0f}, g_tp_eye_oy{0.0f}, g_tp_eye_oz{0.0f};
std::atomic<bool>      g_tp_eye_valid{false};
// The GAME'S OWN chase camera, as the engine handed it to the eye callback before our override, as an
// offset from the chassis read at the same instant. The ray aim takes its origin from it
// (vehaimorigin=1): the vehicle's guns converge on what that camera's line of sight hits.
std::atomic<float>     g_tp_ncam_ox{0.0f}, g_tp_ncam_oy{0.0f}, g_tp_ncam_oz{0.0f};
std::atomic<bool>      g_tp_ncam_valid{false};
// Bumped on every rising edge of our camera (a mount, or stepping back from a first-person entry). The
// eye re-arms its per-ride captures -- the frozen view yaw, the vehicle frame, the head anchor -- when it
// changes. Stepping between two of OUR cameras does not bump it: the view carries on where it is.
std::atomic<uint32_t>  g_tp_mount_gen{0};
// Camera-collision spring arm: the game tick traces from the chassis to the desired boom endpoint
// and publishes a [floor..1] scale; the render eye multiplies the boom offset by it. 1 = unobstructed.
std::atomic<float>     g_tp_collision_frac{1.0f};

// RUNTIME third-person state: true while OUR camera draws -- a chase entry from halo_vr_vehcams.json is
// selected for the vehicle you are in (VehCamSelect.cpp sets it on every selection; a first-person
// entry clears it and hands the view to the seat camera). All the TP gates read this, not g_cfg.veh_tp.
std::atomic<bool>      g_veh_tp_active{false};

// Right-stick TURN (deg) added to the VIEW yaw when motion aim frees the stick (vehstick=1). A turn,
// not an orbit: it pivots on your head and never moves the anchor (an offset that rides the VIEW does
// swing round with it -- that camera is an orbit by choice). Accumulated on the game tick, read by the
// eye; zeroed on every new ride, held otherwise (vehorbitreturn=0).
std::atomic<float>     g_veh_turn_yaw{0.0f};

// Left X / left Y in a vehicle: next / previous camera. Called from the input hook (any thread); the
// game tick applies it (VehCamSelect.cpp).
void veh_cam_next_prev(int dir) { veh_cam_step(dir); }

// vehaim: the Blam aim write (BlamDrive.cpp) consults this to lift its stick-mode hold-off in a
// vehicle. True only when the owned TP camera is ACTIVE AND motion aim is on for this vehicle (its
// "motionAim" in the camera file, else vehaim) AND the chassis is resolved (= we are actually in a
// vehicle, not a cutscene/death, which also raise stick mode). POD reads + atomics, so it is safe on
// the sim orientation getter's thread.
bool veh_tp_motion_aim_active() {
    if (!g_veh_tp_active.load(std::memory_order_relaxed)) return false;
    if (g_tp_chassis_ptr.load(std::memory_order_relaxed) == 0) return false;
    const VehActiveCam ac = veh_active_cam();
    return ac.motion_aim >= 0 ? ac.motion_aim != 0 : g_cfg.veh_aim;
}

// True while our head-anchored camera is actually drawing (the eye wrote this frame --
// g_tp_chassis_yaw_valid -- not merely "a chassis is resolved"). With hmdleash=0 Plugin.cpp stands the
// leash block down on it, so nothing slides the standing origin and you lean freely off the anchor.
// With hmdleash=1 the leash keeps running and holds your head to the anchor exactly as it holds it to
// your body on foot. Any thread.
bool veh_tp_anchor_active() {
    return g_veh_tp_active.load(std::memory_order_relaxed)
        && halo::g_stick_mode_active.load(std::memory_order_relaxed)
        && g_tp_chassis_yaw_valid.load(std::memory_order_relaxed);
}

// bc24's first-person SEAT camera (vehcam): its own key, or ON while a first-person entry is selected
// in the camera file -- listing one is asking for it. Callers pass their own cfg value (render callbacks
// read a snapshot, see CfgRead.hpp). Any thread.
bool veh_fp_selected() {
    const VehActiveCam ac = veh_active_cam();
    return ac.valid && ac.type == static_cast<uint8_t>(vehcampresets::CamType::FirstPerson);
}
int veh_cam_mode(int cfg_veh_cam) {
    if (cfg_veh_cam != 0) return cfg_veh_cam;
    return veh_fp_selected() ? 1 : 0;
}

// VEHICLE RAY AIM (vehaimray), published by the game tick, consumed by the sim-thread aim write.
// ANGLES, not positions: the target is a world-fixed point and the angle toward it changes only as
// fast as the hand and the vehicle move, so a tick-rate publish is the same cadence the infantry aim
// law already runs at. UE convention (yaw = atan2(y, x), pitch up positive); BlamDrive flips the yaw.
std::atomic<float> g_veh_aim_yaw{0.0f}, g_veh_aim_pitch{0.0f};
std::atomic<bool>  g_veh_aim_valid{false};
// The last measured range along the pointing ray, HELD on a miss -- sky has no range, and inventing
// one swings the aim every time the ray crosses a skyline (AimConverge.hpp's rule, same reason).
std::atomic<float> g_veh_aim_range{0.0f};
// The point the vehicle is aimed THROUGH (the pointing ray at that range), world cm, from this tick.
// Consumed on the same tick by the world-space reticules; the compositor copy is re-stamped per frame.
std::atomic<float> g_veh_aim_tx{0.0f}, g_veh_aim_ty{0.0f}, g_veh_aim_tz{0.0f};

bool veh_aim_ray_angles(float* yaw, float* pitch) {
    if (!veh_tp_motion_aim_active() || !g_cfg.veh_aim_ray) return false;
    if (!g_veh_aim_valid.load(std::memory_order_relaxed)) return false;
    *yaw   = g_veh_aim_yaw.load(std::memory_order_relaxed);
    *pitch = g_veh_aim_pitch.load(std::memory_order_relaxed);
    return true;
}

// UE cm per REAL metre, as UEVR is rendering it right now: 100 x VR_WorldScale (the direction is
// measured, not assumed -- HeightCal fitted 112.7 cm/m against 100 x 1.126 = 112.6). Read LIVE rather
// than taken from rig_scale: rig_scale is a fixed number that matches the shipped profile's 1.312,
// and players set their own world scale -- a mismatch puts the controller ray and the head anchor off
// by the ratio. Polled on the game tick every ~2 s while our camera is up (a mod-value read is a string
// search, not a render-thread job); the render side reads the atomic. 0 = not read yet.
std::atomic<float> g_veh_cm_per_m{0.0f};

namespace {
float veh_cm_per_m() {
    const float v = g_veh_cm_per_m.load(std::memory_order_relaxed);
    if (v > 1.0f) return v;
    return (g_cfg.rig_scale > 1.0f) ? g_cfg.rig_scale : 100.0f;   // before the first read
}

// GAME THREAD. Raw C accessor, as XrLayer/ScopeLayer/HeightCal read it: the header's float path runs
// std::stof on an empty buffer when the key is absent. An absent or implausible value keeps the last
// good one -- the mono-collapse view parks VR_WorldScale at its 0.01 floor, which is not a preference.
void veh_poll_world_scale() {
    char buf[64]{};
    auto* p = API::get()->param();
    if (p == nullptr || p->vr == nullptr || p->vr->get_mod_value == nullptr) return;
    p->vr->get_mod_value("VR_WorldScale", buf, sizeof(buf));
    if (buf[0] == 0) return;
    const float ws = (float)std::atof(buf);
    if (!(ws > 0.05f && ws < 100.0f)) return;
    const float cmpm = 100.0f * ws;
    const float prev = g_veh_cm_per_m.exchange(cmpm, std::memory_order_relaxed);
    if (std::fabs(prev - cmpm) > 0.05f)
        API::get()->log_info("[Halo-CampE-UEVR] VEHTP: world scale %.3f -> %.1f UE cm per real metre "
                             "(vehicle controller ray + head anchor)", ws, cmpm);
}

// The range the reticule and the aim use: the held measurement, or the far end of the ray before this
// ride has measured anything. ONE rule for the tick and the per-frame stamp, so they cannot disagree.
float veh_aim_range_eff() {
    const float r = g_veh_aim_range.load(std::memory_order_relaxed);
    if (r > 1.0f) return r;
    return (g_cfg.veh_aim_far > 1.0f) ? g_cfg.veh_aim_far : 10000.0f;
}

// The controller's world ray EXACTLY as UEVR draws it in our third-person camera:
//     world = view_base + R(view) . swizzle(room - standing_origin) . (100 x VR_WorldScale)
// -- the transform the head gets too (the anchor capture in the eye uses the same one). R(view) is the
// FULL view rotation: a camera that tilts with the vehicle tilts the whole tracking space, hands included,
// and UEVR composes it as UE's own rotator (yaw, then pitch, then roll -- FFakeStereoRenderingHook.cpp's
// yawPitchRoll(-yaw, pitch, -roll) is that rotator expressed in the VR axes).
// room_to_world() is the wrong tool here, twice: it scales by 100 cm/m where UEVR renders at
// 100 x VR_WorldScale (131.2 on the shipped profile, whatever the player chose on theirs), and without
// room_anchor it hangs the offset off the HMD at g_cam, which in this camera is the VIEW BASE rather
// than the head. Either one shifts the whole ray sideways -- centimetres to tens of centimetres -- so
// the aim point and the crosshair sat off the line the hand actually points along. Direction is
// scale-free. Any thread: pure maths, the cached world scale, and UEVR's standing origin (which the
// eye callback already reads).
void veh_room_ray(const Vec3& cpos, const Vec3& fwd, const double base[3],
                  float view_pitch, float view_yaw, float view_roll, Vec3* o, Vec3* d) {
    const auto so = API::VR::get_standing_origin();
    const double rs = (double)veh_cm_per_m();
    double X[3], Y[3], Z[3];
    rot_axes(view_pitch, view_yaw, view_roll, X, Y, Z);
    // room -> UE: X = -z, Y = x, Z = y. Same rotation as the eye's anchor subtraction.
    const double lx = -((double)cpos.z - (double)so.z) * rs;
    const double ly =  ((double)cpos.x - (double)so.x) * rs;
    const double lz =  ((double)cpos.y - (double)so.y) * rs;
    *o = Vec3{ (float)(base[0] + lx * X[0] + ly * Y[0] + lz * Z[0]),
               (float)(base[1] + lx * X[1] + ly * Y[1] + lz * Z[1]),
               (float)(base[2] + lx * X[2] + ly * Y[2] + lz * Z[2]) };
    const double fx = -(double)fwd.z, fy = (double)fwd.x, fz = (double)fwd.y;
    double dx = fx * X[0] + fy * Y[0] + fz * Z[0];
    double dy = fx * X[1] + fy * Y[1] + fz * Z[1];
    double dz = fx * X[2] + fy * Y[2] + fz * Z[2];
    const double dl = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dl > 1e-9) { dx /= dl; dy /= dl; dz /= dl; }
    *d = Vec3{ (float)dx, (float)dy, (float)dz };
}

} // namespace

// Who publishes the ONE compositor reticule while you aim a vehicle with the controller: the eye
// callback, every frame, on the live pointing ray (see the stamp in vehcam_stereo_pre_eye_seat). The
// tick path in Plugin.cpp stands its compositor publish down on this -- the layer's snapshot takes one
// writer at a time -- and keeps placing the world-space widget/mesh reticules. Any thread.
bool veh_tp_reticle_stamp_owns() {
    return g_cfg.xr_layer && g_cfg.aim_reticule
        && veh_tp_motion_aim_active() && g_cfg.veh_aim_ray
        && halo::g_stick_mode_active.load(std::memory_order_relaxed)
        && g_tp_chassis_yaw_valid.load(std::memory_order_relaxed)
        && g_veh_aim_valid.load(std::memory_order_relaxed);
}

// THIRD-PERSON VEHICLE RETICULE, tick placement (the world-space widget/mesh reticules, and the
// compositor one whenever the per-frame stamp does not own it).
//
// Aiming with the controller (vehaimray), it is the INTENT point: where the pointing ray reaches, the
// same point the vehicle is aimed through -- the infantry rule (intent ray -> impact point -> the shot
// converged on it). Until 2026-09-24 it was instead re-traced along the vehicle's CURRENT aim from the
// seated unit, which trailed the hand by the aim write and landed at a different depth than the
// pointing ray, so it sat off the line you point along.
//
// Otherwise (stick-driven vehicle aim) it follows the aim that is actually there: traced from the
// seated unit along ControlRotation. The old origin -- the rendered view position -- is right while the
// eye sits in the vehicle and wrong behind our boom camera: that ray starts ~10 m behind and above the
// vehicle, so the marker floated on a line PARALLEL to the shots rather than on what they hit.
// GAME THREAD (hit_trace is reflection). False = our third-person camera is not up; keep the old path.
bool veh_tp_reticle_target(float yaw, float pitch, Vec3* out) {
    if (out == nullptr || !g_veh_tp_active.load(std::memory_order_relaxed)) return false;
    const uintptr_t cp = g_tp_chassis_ptr.load(std::memory_order_relaxed);
    if (cp == 0) return false;
    if (veh_tp_motion_aim_active() && g_cfg.veh_aim_ray && g_veh_aim_valid.load(std::memory_order_relaxed)) {
        *out = Vec3{g_veh_aim_tx.load(std::memory_order_relaxed),
                    g_veh_aim_ty.load(std::memory_order_relaxed),
                    g_veh_aim_tz.load(std::memory_order_relaxed)};
        return true;
    }
    auto* pawn = API::get()->get_local_pawn(0);
    Vec3 c{};
    if (pawn == nullptr || !call_ret_vec3(pawn, L"K2_GetActorLocation", &c)) return false;
    c.z += g_cfg.veh_aim_pivot_z;
    const float cpch = std::cos(pitch * DEG2RAD);
    const Vec3 u{cpch * std::cos(yaw * DEG2RAD), cpch * std::sin(yaw * DEG2RAD), std::sin(pitch * DEG2RAD)};
    const float far_cm = (g_cfg.veh_aim_far > 1.0f) ? g_cfg.veh_aim_far : 10000.0f;   // not `far`: a <Windows.h> macro
    const Vec3 e{c.x + u.x * far_cm, c.y + u.y * far_cm, c.z + u.z * far_cm};
    static TrackedObject s_tpc_ret;
    static uintptr_t s_tpc_ret_raw = 0;
    if (cp != s_tpc_ret_raw) {
        s_tpc_ret_raw = cp;
        s_tpc_ret.set_at(reinterpret_cast<API::UObject*>(cp), g_tp_chassis_idx.load(std::memory_order_relaxed));
    }
    API::UObject* ignore[2] = {}; int ni = 0;
    ignore[ni++] = pawn;
    if (auto* ch = s_tpc_ret.get_checked(L"SkeletalMeshComponent"))
        if (auto* owner = ch->get_outer()) ignore[ni++] = owner;
    Vec3 hit{};
    *out = hit_trace(c, e, ignore, ni, &hit) ? hit : e;   // sky: along the aim at the trace length
    return true;
}

namespace {
// The resolved chassis's full object name ("...BP_BansheeVehicleActor_C_<id>.hull"): the camera file
// picks a vehicle's cameras by matching it. GAME THREAD.
std::wstring s_tp_chassis_name;

// GAME THREAD. Nearest VehicleActor SkeletalMeshComponent to the player pawn = the chassis of the
// vehicle the player is in. No name gate (chassis naming varies: Banshee ".hull", Wraith
// "SK_WraithMortar"); no rider-Blam seed (dead). Walks the object array, so callers throttle it.
void resolve_tp_chassis() {
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;
    auto* pawn = API::get()->get_local_pawn(0);
    Vec3 ploc{};
    if (pawn == nullptr || !call_ret_vec3(pawn, L"K2_GetActorLocation", &ploc)) {
        g_tp_chassis_ptr.store(0, std::memory_order_relaxed);
        g_tp_chassis_idx.store(-1, std::memory_order_relaxed);
        return;
    }
    const double px = (double)ploc.x, py = (double)ploc.y, pz = (double)ploc.z;
    const int32_t n = arr->get_object_count();
    API::UObject* best = nullptr; int32_t besti = -1; double bestd = 1e18;
    for (int32_t i = 0; i < n; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"SkeletalMeshComponent") continue;
        const std::wstring full = o->get_full_name();
        if (full.find(L"PersistentLevel") == std::wstring::npos) continue;
        if (full.find(L"VehicleActor")   == std::wstring::npos) continue;
        Vec3 w{}; if (!call_ret_vec3(o, L"K2_GetComponentLocation", &w)) continue;
        const double d = std::sqrt(((double)w.x - px) * ((double)w.x - px)
                                 + ((double)w.y - py) * ((double)w.y - py)
                                 + ((double)w.z - pz) * ((double)w.z - pz));
        if (d < bestd) { bestd = d; best = o; besti = i; }
    }
    if (best != nullptr && bestd < 800.0) {   // within 8 m of the pawn = the vehicle it is in
        const uintptr_t prev = g_tp_chassis_ptr.load(std::memory_order_relaxed);
        g_tp_chassis_ptr.store((uintptr_t)best, std::memory_order_relaxed);
        g_tp_chassis_idx.store(besti, std::memory_order_relaxed);
        s_tp_chassis_name = best->get_full_name();
        if ((uintptr_t)best != prev)
            API::get()->log_info("[Halo-CampE-UEVR] VEHTP chassis: %ls at %.0fcm from pawn(%.0f %.0f %.0f)",
                                 best->get_full_name().c_str(), bestd, px, py, pz);
    } else {
        g_tp_chassis_ptr.store(0, std::memory_order_relaxed);
        g_tp_chassis_idx.store(-1, std::memory_order_relaxed);
        static uint32_t s_warn = 0;
        if ((s_warn++ % 90u) == 0u)
            API::get()->log_info("[Halo-CampE-UEVR] VEHTP: no VehicleActor mesh within 8 m of pawn "
                                 "(best %.0f, pawn %.0f %.0f %.0f)", bestd < 1e17 ? bestd : -1.0, px, py, pz);
    }
}
} // namespace

namespace {

void resolve_hog_body() {
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;
    const double S = 304.8;
    const double bx =  (double)g_unit_px.load(std::memory_order_relaxed) * S;
    const double by = -(double)g_unit_py.load(std::memory_order_relaxed) * S;
    const double bz =  (double)g_unit_pz.load(std::memory_order_relaxed) * S;
    const int32_t n = arr->get_object_count();
    API::UObject* best = nullptr; int32_t besti = -1; double bestd = 1e18;
    for (int32_t i = 0; i < n; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"SkeletalMeshComponent") continue;
        const std::wstring full = o->get_full_name();
        if (full.find(L"PersistentLevel") == std::wstring::npos) continue;
        if (full.find(L"VehicleActor") == std::wstring::npos) continue;
        // The Warthog's drawn mesh is ".hull" -- lowercase, measured by HOGDUMP, and the reason
        // a ".Body"-only match resolved nothing while the camera silently fell back. The NAME
        // gate cannot be dropped for "nearest skeletal", because the nearest skeletal mesh from
        // the driver's seat is the CHAINGUN (199 cm, its own actor) -- bolting the camera to the
        // turret would aim your head wherever the gunner points. Named hull/body first; anything
        // else only as a logged last resort for vehicles this convention misses.
        auto ends_with_ci = [&full](const wchar_t* suf) {
            const size_t m = wcslen(suf);
            if (full.size() < m) return false;
            for (size_t k = 0; k < m; ++k)
                if (towlower(full[full.size() - m + k]) != towlower(suf[k])) return false;
            return true;
        };
        const bool named = ends_with_ci(L".hull") || ends_with_ci(L".body");
        Vec3 w{};
        if (!call_ret_vec3(o, L"K2_GetComponentLocation", &w)) continue;
        const double d = std::sqrt(((double)w.x - bx) * ((double)w.x - bx)
                                 + ((double)w.y - by) * ((double)w.y - by)
                                 + ((double)w.z - bz) * ((double)w.z - bz));
        // A named hull always beats an unnamed candidate; distance only breaks ties in a class.
        const double score = named ? d : d + 100000.0;
        if (score < bestd) { bestd = score; best = o; besti = i; }
    }
    const bool fell_back = (bestd >= 100000.0 && bestd < 1e17);
    if (fell_back) bestd -= 100000.0;
    if (best != nullptr && bestd < 1000.0) {   // within 10 m, or it is not the thing you sit in
        g_hog_body_ptr.store((uintptr_t)best, std::memory_order_relaxed);
        g_hog_body_idx.store(besti, std::memory_order_relaxed);
        API::get()->log_info("[Halo-CampE-UEVR] HOGBODY: %ls at %.1fcm%s", best->get_full_name().c_str(),
                             bestd, fell_back ? "  (UNNAMED fallback -- check this is the hull)" : "");
    } else {
        g_hog_body_ptr.store(0, std::memory_order_relaxed);
        g_hog_body_idx.store(-1, std::memory_order_relaxed);
        API::get()->log_info("[Halo-CampE-UEVR] HOGBODY: no VehicleActor hull within 10 m (best %.0f) -- "
                             "camera stays on the chase-cam anchor", bestd < 1e17 ? bestd : -1.0);
    }
}

} // namespace

// ================================================================================================
// VEHPROBE (dev only) -- does a vehicle expose a WEAPON/TURRET aim distinct from the Blam control
// record (the one value that also drives the chase camera)? This is the Route A/B decision probe
// for the aim/camera-decouple lane; doctrine + how to read the output live in
// docs\VEHICLE_AIM_DECOUPLE_PROBE.md. Read-only. Game thread, seated only, self-contained: it reads
// the rider position straight off the seat object so it does not depend on veh_cam/veh_seat_direct
// publishing anything. `vehprobe_tick()` is an empty stub in a release build (DevTools pattern), so
// the call site in vehcam_game_tick_vehicle needs no #if.
#if HALO_VR_DEV
namespace {

std::atomic<uintptr_t> g_vehprobe_turret{0};   // nearest non-hull VehicleActor mesh (the gun)
bool                   g_vehprobe_scanned = false;

// The Blam control-record aim, radians -> degrees. This is the SHARED value: in a vehicle it drives
// both the weapon and the chase camera. rec points AT the yaw field (rec+0x94), pitch at rec+0x98.
bool vehprobe_record_deg(float* yaw_deg, float* pitch_deg) {
    const uintptr_t rec = blam_control_record();
    if (rec == 0 || IsBadReadPtr((const void*)rec, 8)) return false;
    const float* fp = (const float*)rec;
    if (!std::isfinite(fp[0]) || !std::isfinite(fp[1])) return false;
    const float R2D = 180.0f / 3.14159265358979f;
    *yaw_deg = fp[0] * R2D; *pitch_deg = fp[1] * R2D;
    return true;
}

// One-shot: log the name + offset of every UPROPERTY on obj's leaf class. A turret-rotation field
// (RelativeRotation, or a bespoke aim field) that we could write for Route B shows itself by name.
void vehprobe_dump_props(API::UObject* obj, const char* label) {
    if (obj == nullptr) return;
    API::get()->log_info("[Halo-CampE-UEVR] VEHPROBE %s = %ls; UPROPERTIES (leaf..base):",
                         label, obj->get_full_name().c_str());
    // Walk the WHOLE class chain (get_super_struct), not just the leaf: a seat/vehicle reference on
    // a C++ base pawn class would be invisible in a leaf-only dump. Capped so a deep chain can't
    // flood the log.
    int total = 0;
    for (API::UStruct* st = obj->get_class(); st != nullptr && total < 500; st = st->get_super_struct()) {
        for (auto* f = st->get_child_properties(); f != nullptr && total < 500; f = f->get_next()) {
            auto* nm = f->get_fname();
            const int32_t off = reinterpret_cast<API::FProperty*>(f)->get_offset();
            API::get()->log_info("[Halo-CampE-UEVR]     %ls  @ +0x%X",
                                 nm ? nm->to_string().c_str() : L"?", (unsigned)off);
            ++total;
        }
    }
}

// One walk of the UObject array: log every VehicleActor SkeletalMeshComponent near the search
// CENTRE with its WORLD rotation, and latch the nearest one NOT named .hull/.body as the turret
// candidate. Centre is the rendered VIEW position (reliable in stick mode); the seat resolution
// (g_seat_obj / g_unit_mounted) can read 0, so this does not depend on it. If no centre is
// available it logs all VehicleActor meshes (capped). The rotations tell whether the gun points
// independently of the hull; the record aim (each beat) tells whether that aim is the shared value.
void vehprobe_scan(bool have_ctr, double cx, double cy, double cz) {
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;
    auto ends_with_ci = [](const std::wstring& s, const wchar_t* suf) {
        const size_t m = wcslen(suf);
        if (s.size() < m) return false;
        for (size_t k = 0; k < m; ++k)
            if (towlower(s[s.size() - m + k]) != towlower(suf[k])) return false;
        return true;
    };
    const int32_t n = arr->get_object_count();
    API::UObject* nearest_gun = nullptr; double ngd = 1e18; int logged = 0;
    API::get()->log_info("[Halo-CampE-UEVR] VEHPROBE scan (haveCtr=%d ctr=%.0f,%.0f,%.0f): VehicleActor skeletal meshes --",
                         (int)have_ctr, cx, cy, cz);
    for (int32_t i = 0; i < n; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"SkeletalMeshComponent") continue;
        const std::wstring full = o->get_full_name();
        if (full.find(L"PersistentLevel") == std::wstring::npos) continue;
        if (full.find(L"VehicleActor")   == std::wstring::npos) continue;
        Vec3 w{}; const bool haveLoc = call_ret_vec3(o, L"K2_GetComponentLocation", &w);
        double d = -1.0;
        if (have_ctr && haveLoc) {
            d = std::sqrt((w.x - cx) * (w.x - cx) + (w.y - cy) * (w.y - cy) + (w.z - cz) * (w.z - cz));
            if (d > 2500.0) continue;   // 25 m of the camera -- the chase cam sits well back
        }
        Vec3 r{}; const bool haveR = call_ret_vec3(o, L"K2_GetComponentRotation", &r);
        const bool is_hull = ends_with_ci(full, L".hull") || ends_with_ci(full, L".body");
        if (logged < 20) {
            API::get()->log_info("[Halo-CampE-UEVR]     d=%.0fcm rot(x=%.1f y=%.1f z=%.1f ok%d) %s %ls",
                                 d, r.x, r.y, r.z, (int)haveR, is_hull ? "[HULL]" : "", full.c_str());
            ++logged;
        }
        if (!is_hull) {
            if (have_ctr) { if (d >= 0.0 && d < ngd) { ngd = d; nearest_gun = o; } }
            else if (nearest_gun == nullptr) { nearest_gun = o; }
        }
    }
    if (nearest_gun != nullptr) {
        g_vehprobe_turret.store((uintptr_t)nearest_gun, std::memory_order_relaxed);
        vehprobe_dump_props(nearest_gun, "turret-candidate");
    } else {
        API::get()->log_info("[Halo-CampE-UEVR] VEHPROBE: no non-hull VehicleActor mesh found (logged=%d) -- "
                             "whole-vehicle aim, or the scan centre missed it.", logged);
    }
}

void vehprobe_tick() {
    if (!g_cfg.veh_probe) return;
    // Gate on STICK MODE, not g_unit_mounted: stick mode is the proven vehicle signal (the log
    // shows it reliably), while UnitState's seat/datum resolve can read 0 and leave g_unit_mounted
    // stuck false -- which silently killed the first cut of this probe. Stick mode also covers
    // cutscenes/death, where the scan just finds no VehicleActor mesh (harmless).
    const bool stick = g_stick_mode_active.load(std::memory_order_relaxed);
    static bool was_stick = false;
    if (!stick) {   // reset on exit so the next vehicle re-scans
        if (was_stick) { g_vehprobe_scanned = false; g_vehprobe_turret.store(0, std::memory_order_relaxed); }
        was_stick = false;
        return;
    }
    was_stick = true;

    // Search centre: the rendered view position, via the plugin-state bridge (reliable).
    auto& vpx = *host::g_plugin_state.view_pos_x;
    auto& vpy = *host::g_plugin_state.view_pos_y;
    auto& vpz = *host::g_plugin_state.view_pos_z;
    const double cx = (double)vpx.load(std::memory_order_relaxed);
    const double cy = (double)vpy.load(std::memory_order_relaxed);
    const double cz = (double)vpz.load(std::memory_order_relaxed);
    const bool have_ctr = (cx != 0.0 || cy != 0.0 || cz != 0.0);

    if (!g_vehprobe_scanned) {
        vehprobe_scan(have_ctr, cx, cy, cz);
        // Is the possessed pawn the VEHICLE (boom off it directly) or the biped (find its vehicle
        // ref)? get_local_pawn is the cheap live handle -- no object-array walk. A
        // "...VehicleActor_C_..." full name lets P0 match the chassis mesh by NAME PREFIX (robust,
        // unlike proximity which picked a parked Banshee over the Wraith). Dump its UPROPERTIES too,
        // in case the pawn is the biped and holds a vehicle reference.
        if (auto* pawn = API::get()->get_local_pawn(0)) {
            API::get()->log_info("[Halo-CampE-UEVR] VEHPROBE pawn = %ls (class %ls)",
                                 pawn->get_full_name().c_str(), class_name_of(pawn).c_str());
            vehprobe_dump_props(pawn, "pawn");
        } else {
            API::get()->log_info("[Halo-CampE-UEVR] VEHPROBE: get_local_pawn(0) returned null while seated");
        }
        g_vehprobe_scanned = true;
    }

    // Heartbeat + comparison, ~every 60 ticks (~2 s). The heartbeat runs unconditionally in stick
    // mode so a silent no-op can never happen again: it prints exactly what is readable, and lets
    // you watch whether the turret's world rotation tracks the record aim (coupled) or diverges.
    static uint32_t c = 0;
    if ((c++ % 60u) != 0u) return;
    const uintptr_t seat = g_seat_obj.load(std::memory_order_relaxed);
    const bool seat_ok = (seat != 0 && !IsBadReadPtr((const void*)seat, 0x2C));
    uint32_t pdat = 0xFFFFFFFFu;
    if (seat_ok && !IsBadReadPtr((const void*)(seat + 0x0C), 4)) pdat = *(const uint32_t*)(seat + 0x0C);
    float ry = 0, rp = 0; const bool haveRec = vehprobe_record_deg(&ry, &rp);
    float dy = 0, dp = 0; const bool haveDes = desired_aim_now(&dy, &dp);
    Vec3 tr{}; bool haveTur = false;
    const uintptr_t tur = g_vehprobe_turret.load(std::memory_order_relaxed);
    if (tur != 0 && !IsBadReadPtr((const void*)tur, sizeof(void*)))
        haveTur = call_ret_vec3((API::UObject*)tur, L"K2_GetComponentRotation", &tr);
    API::get()->log_info(
        "[Halo-CampE-UEVR] VEHPROBE beat: stick=1 g_unit_mounted=%d seatObj=0x%llX seatOk=%d "
        "pdat=0x%08X haveCtr=%d | rec(y=%.1f p=%.1f ok%d) des(y=%.1f p=%.1f ok%d) "
        "turretWorld(x=%.1f y=%.1f z=%.1f ok%d)",
        (int)g_unit_mounted.load(std::memory_order_relaxed),
        (unsigned long long)seat, (int)seat_ok, (unsigned)pdat, (int)have_ctr,
        ry, rp, (int)haveRec, dy, dp, (int)haveDes, tr.x, tr.y, tr.z, (int)haveTur);
}

} // namespace
#else
namespace { inline void vehprobe_tick() {} }
#endif // HALO_VR_DEV

// Game-thread tick for the vehicle body work: the driver hide reconciles every tick, and the
// hog hull resolves at the mount edge and KEEPS RETRYING while it fails. The one-shot version
// cost a whole session of false verdicts: it fired at the exact instant the mounted flag flips
// -- mid entry animation -- found the nearest hull 235 m away, gave up for the rest of the
// mount, and the camera silently rode the fallback while three builds of the rigid path went
// untested. A resolve that can fail transiently must retry; every ~2 s while mounted-and-
// unresolved is invisible in cost. Cleared on dismount.
void vehicle_body_update() {
    driver_hide_update();
    // The hull resolve feeds the seat camera, the seated view and the wheel only (experimental, all off
    // by default): with none of them on, a mount must not sweep for the hog.
    // Exactly two consumers read the hull: the rigid seat camera (vehcam with anchor 2) and the wheel.
    if (!(g_cfg.enabled && ((veh_cam_mode(g_cfg.veh_cam) != 0 && g_cfg.veh_cam_anchor == 2) || g_cfg.vehicle_wheel != 0))) {
        g_hog_body_ptr.store(0, std::memory_order_relaxed);
        g_hog_body_idx.store(-1, std::memory_order_relaxed);
        return;
    }
    static bool s_was_mounted = false;
    static uint32_t s_hog_tick = 0;
    const bool m = g_unit_mounted.load(std::memory_order_relaxed);
    if (m && !s_was_mounted) { resolve_hog_body(); s_hog_tick = 0; }
    else if (m && g_hog_body_ptr.load(std::memory_order_relaxed) == 0) {
        if ((++s_hog_tick % 90u) == 0u) resolve_hog_body();
    }
    if (!m && s_was_mounted) { g_hog_body_ptr.store(0, std::memory_order_relaxed);
                               g_hog_body_idx.store(-1, std::memory_order_relaxed); }
    s_was_mounted = m;
}

// vehseatdirect (approach B): the seat without the sim hook. The rider and vehicle objects are
// pool entries that stay put for the life of the ride, so once the sim publish has named them the
// fields are plain memory reads from any thread -- no gs:[0x58] walk, no control record, no hook
// cadence. Only while stick mode holds the normal publish off; on foot the sim publish owns these.
// The vehicle half is trusted only while the rider's parent datum still names the cached vehicle.
void seat_direct_refresh() {
    if (g_cfg.veh_seat_direct == 0) return;
    if (!g_stick_mode_active.load(std::memory_order_relaxed)) return;
    const uintptr_t obj = g_seat_obj.load(std::memory_order_relaxed);
    if (obj == 0 || IsBadReadPtr((const void*)obj, 0x2C)) return;
    const uint32_t pdat = *(const uint32_t*)(obj + 0x0C);
    const float* q = (const float*)(obj + 0x20);
    if (!std::isfinite(q[0]) || !std::isfinite(q[1]) || !std::isfinite(q[2])) return;
    g_unit_mounted.store(pdat != 0xFFFFFFFFu, std::memory_order_relaxed);
    g_unit_px.store(q[0], std::memory_order_relaxed);
    g_unit_py.store(q[1], std::memory_order_relaxed);
    g_unit_pz.store(q[2], std::memory_order_relaxed);
    g_unit_pvalid.store(true, std::memory_order_relaxed);
    g_seat_pub_seq.fetch_add(1, std::memory_order_relaxed);
    g_seat_direct_reads.fetch_add(1, std::memory_order_relaxed);
    const uintptr_t vobj = g_seat_vobj.load(std::memory_order_relaxed);
    if (g_cfg.veh_facing != 0 && vobj != 0 && pdat != 0xFFFFFFFFu
        && pdat == g_seat_vdat.load(std::memory_order_relaxed)
        && !IsBadReadPtr((const void*)vobj, 0x1F4)) {
        const float* fv = (const float*)(vobj + (uintptr_t)g_cfg.veh_facing_off);
        const float* vp = (const float*)(vobj + 0x20);
        if (std::isfinite(fv[0]) && std::isfinite(fv[1])) {
            g_veh_fx.store(fv[0], std::memory_order_relaxed);
            g_veh_fy.store(fv[1], std::memory_order_relaxed);
            g_veh_fvalid.store(true, std::memory_order_relaxed);
        }
        if (std::isfinite(vp[0]) && std::isfinite(vp[1]) && std::isfinite(vp[2])) {
            g_vehpx.store(vp[0], std::memory_order_relaxed);
            g_vehpy.store(vp[1], std::memory_order_relaxed);
            g_vehpz.store(vp[2], std::memory_order_relaxed);
        }
    }
}

// ---- VEHICLE WHEEL + SEAT CAMERA keys.
static bool parse_veh_key(const char* key, const char* val, double v) {
    if (_stricmp(key, "vehiclewheel")   == 0) { g_cfg.vehicle_wheel = (int)v; return true; }
    if (_stricmp(key, "vehwheelpos")    == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.veh_wheel_pos[0], &g_cfg.veh_wheel_pos[1], &g_cfg.veh_wheel_pos[2]); return true; }
    if (_stricmp(key, "vehwheelrad")    == 0) { g_cfg.veh_wheel_radius = clampf((float)v, 0.05f, 1.0f); return true; }
    if (_stricmp(key, "vehwheellock")   == 0) { g_cfg.veh_wheel_lock = clampf((float)v, 10.0f, 360.0f); return true; }
    if (_stricmp(key, "vehwheelsteersign")   == 0) { g_cfg.veh_steer_sign = (v < 0.0) ? -1 : 1; return true; }
    if (_stricmp(key, "vehwheelgrip")   == 0) { g_cfg.veh_wheel_grip = (int)v; return true; }
    if (_stricmp(key, "vehwheelhand")   == 0) { g_cfg.veh_wheel_hand = (int)v; return true; }
    if (_stricmp(key, "vehwheeltilt")   == 0) { g_cfg.veh_wheel_tilt = clampf((float)v, -80.0f, 80.0f); return true; }
    if (_stricmp(key, "vehwheelmarker") == 0) { g_cfg.veh_wheel_marker = (int)v; return true; }
    if (_stricmp(key, "vehcam")         == 0) { g_cfg.veh_cam = (int)v; return true; }
    if (_stricmp(key, "vehcamoff")      == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.veh_cam_off[0], &g_cfg.veh_cam_off[1], &g_cfg.veh_cam_off[2]); return true; }
    if (_stricmp(key, "vehcamscale")    == 0) { g_cfg.veh_cam_scale = (float)v; return true; }
    if (_stricmp(key, "vehcamsrc")      == 0) { g_cfg.veh_cam_src = (int)v; return true; }
    if (_stricmp(key, "vehanchor")      == 0) { g_cfg.veh_anchor = (int)v; return true; }
    if (_stricmp(key, "vehprobe")       == 0) { g_cfg.veh_probe = (v != 0.0); return true; }
    if (_stricmp(key, "vehtp")          == 0) { g_cfg.veh_tp = (v != 0.0); return true; }
    if (_stricmp(key, "vehaim")         == 0) { g_cfg.veh_aim = (v != 0.0); return true; }
    if (_stricmp(key, "vehstick")       == 0) { g_cfg.veh_stick_mode = (int)v; return true; }
    if (_stricmp(key, "vehorbitrate")   == 0) { g_cfg.veh_orbit_rate = (float)v; return true; }
    if (_stricmp(key, "vehorbitreturn") == 0) { g_cfg.veh_orbit_return = (float)v; return true; }
    if (_stricmp(key, "vehaimorigin")   == 0) { g_cfg.veh_aim_origin = (int)v; return true; }
    if (_stricmp(key, "vehcamreadout")  == 0) { g_cfg.veh_cam_readout = (v != 0.0); return true; }
    if (_stricmp(key, "vehctrlclick")   == 0) { g_cfg.veh_ctrl_click = (int)v; return true; }
    if (_stricmp(key, "vehaimray")      == 0) { g_cfg.veh_aim_ray = (v != 0.0); return true; }
    if (_stricmp(key, "vehaimfar")      == 0) { g_cfg.veh_aim_far = (float)v; return true; }
    if (_stricmp(key, "vehaimpivotz")   == 0) { g_cfg.veh_aim_pivot_z = (float)v; return true; }
    if (_stricmp(key, "vehcamanchor")   == 0) { g_cfg.veh_cam_anchor = (int)v; return true; }
    if (_stricmp(key, "vehhidebody")    == 0) { g_cfg.veh_hide_body = (int)v; return true; }
    if (_stricmp(key, "vehcamboomtau")  == 0) { g_cfg.veh_cam_boom_tau = clampf((float)v, 0.02f, 3.0f); return true; }
    if (_stricmp(key, "vehboomorder")   == 0) { g_cfg.veh_boom_order = (int)v; return true; }
    if (_stricmp(key, "vehfacingbias")  == 0) { g_cfg.veh_facing_bias = (float)v; return true; }
    if (_stricmp(key, "vehview")        == 0) { g_cfg.veh_view = (int)v; return true; }
    if (_stricmp(key, "vehviewflat")    == 0) { g_cfg.veh_view_flat = (int)v; return true; }
    if (_stricmp(key, "vehseatdirect")  == 0) { g_cfg.veh_seat_direct = (int)v; return true; }
    if (_stricmp(key, "vehcamguard")    == 0) { g_cfg.veh_cam_guard = (int)v; return true; }
    if (_stricmp(key, "vehcamguardspeed") == 0) { g_cfg.veh_cam_guard_speed = clampf((float)v, 1.0f, 5000.0f); return true; }
    if (_stricmp(key, "vehcamstalems")  == 0) { g_cfg.veh_cam_stale_ms = (int)clampf((float)v, 20.0f, 5000.0f); return true; }
    if (_stricmp(key, "vehcamhullcheck") == 0) { g_cfg.veh_cam_hull_check = (int)v; return true; }
    if (_stricmp(key, "vehcamhulldead") == 0) { g_cfg.veh_cam_hull_dead_s = clampf((float)v, 0.1f, 10.0f); return true; }
    return false;
}

namespace {

// SEAT CAMERA EVIDENCE (vehlog). Written by the vehcam block on the frame-computing eye, read by
// the once-a-second VEHSEAT line in the same callback. Render thread only.
enum VehCamPath { VCP_NONE = 0, VCP_RIGID, VCP_HOLD, VCP_CHASE, VCP_SYNTH };
struct VehCamDbg {
    uint32_t frames[5] = {0, 0, 0, 0, 0};   // per path since the last VEHSEAT line; [0] = gate off
    int      last_path = -1;
    bool     have_hull = false;
    double   hull[3] = {0.0, 0.0, 0.0};
    double   off[3] = {0.0, 0.0, 0.0};
    double   hspeed = 0.0;                   // drawn hull speed, cm/s
    float    stale_ms = 0.0f;
    bool     learning = false;
    uint32_t learn_frames = 0, snaps = 0, snaps_refused = 0;
    bool     hull_dead = false;              // vehcamhullcheck ruled the hull component not the drawn vehicle
    // WRITE SURVIVAL: the seat position the pre callback wrote on eye 0, and what the post callback
    // (after UEVR's HMD transform) actually rendered. A gap larger than any head offset means
    // something replaced the camera after we wrote it.
    double   fc[3] = {0.0, 0.0, 0.0};
    bool     wrote = false;
    double   post_max = 0.0;
    uint32_t post_frames = 0, post_bad = 0;
};
VehCamDbg g_vcd;

void vehcam_game_tick_vehicle() {
    // Plugin.cpp's own state, through the bridge: the same objects under the same names.
    const auto& g_in_menu = *host::g_plugin_state.in_menu;
    const auto& g_last_dt = *host::g_plugin_state.last_dt;

    // The seat camera's always mode keeps the rendered eye off the body even unmounted.
    g_view_seat_always.store(g_cfg.veh_cam == 2, std::memory_order_relaxed);

    // Vehicle work: the driver-body hide + hog hull resolve, then the wheel gesture and heading
    // publisher. Menus drop the hold like the holsters do.
    g_tick_stage = "vehicle_body";
    vehicle_body_update();
    if (g_in_menu.load()) vehicle_reset(); else vehicle_update(g_last_dt.load());
    // ROUTE A P0: resolve the vehicle's chassis off the pawn, on the stick-mode ENTER edge (reliable,
    // unlike the dead mount flag) and retried every ~3 s while unresolved; cleared on exit so the next
    // vehicle re-resolves. Only walks the object array when it must -- never per tick. The render
    // callbacks read the resolved mesh's transform fresh.
    //
    // Resolved whenever the camera system runs in a vehicle (vehtp), WHICHEVER camera is selected: the
    // chassis is how the vehicle is identified, and a first-person entry still needs it to step back.
    // Then the camera file picks the vehicle's cameras and applies left X / left Y (VehCamSelect.cpp),
    // which is what sets g_veh_tp_active.
    {
        static bool s_sys_was = false;
        static bool s_tp_was = false;
        static uint32_t s_tp_tick = 0;
        const bool stick = halo::g_stick_mode_active.load(std::memory_order_relaxed);
        const bool sys = stick && g_cfg.veh_tp;
        // The live world scale: read at the ride's start (before the eye's first head capture uses it)
        // and re-read every ~2 s during it, so a world-scale change lands mid-ride. Never on foot.
        static uint32_t s_ws_tick = 0;
        if (sys && (!s_sys_was || (++s_ws_tick % 64u) == 0u)) veh_poll_world_scale();
        if (sys && !s_sys_was) {
            resolve_tp_chassis(); s_tp_tick = 0;
            g_veh_turn_yaw.store(0.0f, std::memory_order_relaxed);    // each ride starts facing its heading
        }
        else if (sys && g_tp_chassis_ptr.load(std::memory_order_relaxed) == 0) {
            if ((++s_tp_tick % 90u) == 0u) resolve_tp_chassis();
        }
        if (!sys && s_sys_was) {
            g_tp_chassis_ptr.store(0, std::memory_order_relaxed);
            g_tp_chassis_idx.store(-1, std::memory_order_relaxed);
            g_tp_seat_valid.store(false, std::memory_order_relaxed);
        }
        s_sys_was = sys;

        const uintptr_t cp = g_tp_chassis_ptr.load(std::memory_order_relaxed);
        vehcam_select_tick(sys, cp, s_tp_chassis_name);

        // THE SEAT, in the chassis mesh's frame (origin "seat"): the pawn's offset from the chassis,
        // pawn and chassis read in the same game state, rotated into the mesh's axes. Two reads and a
        // rotation per tick while in a vehicle; the eye rebuilds it against the mesh's live rotation.
        if (sys && cp != 0) {
            static TrackedObject s_tpc_seat;
            static uintptr_t s_tpc_seat_raw = 0;
            if (cp != s_tpc_seat_raw) {
                s_tpc_seat_raw = cp;
                s_tpc_seat.set_at(reinterpret_cast<API::UObject*>(cp), g_tp_chassis_idx.load(std::memory_order_relaxed));
            }
            auto* ch = s_tpc_seat.get_checked(L"SkeletalMeshComponent");
            auto* pawn = API::get()->get_local_pawn(0);
            Vec3 cl{}, cr{}, pl{};
            if (ch != nullptr && pawn != nullptr
                && call_ret_vec3(ch, L"K2_GetComponentLocation", &cl)
                && call_ret_vec3(ch, L"K2_GetComponentRotation", &cr)
                && call_ret_vec3(pawn, L"K2_GetActorLocation", &pl)) {
                double MX[3], MY[3], MZ[3];
                rot_axes(cr.x, cr.y, cr.z, MX, MY, MZ);
                const double w[3] = { (double)pl.x - cl.x, (double)pl.y - cl.y, (double)pl.z - cl.z };
                g_tp_seat_x.store((float)(MX[0] * w[0] + MX[1] * w[1] + MX[2] * w[2]), std::memory_order_relaxed);
                g_tp_seat_y.store((float)(MY[0] * w[0] + MY[1] * w[1] + MY[2] * w[2]), std::memory_order_relaxed);
                g_tp_seat_z.store((float)(MZ[0] * w[0] + MZ[1] * w[1] + MZ[2] * w[2]), std::memory_order_relaxed);
                g_tp_seat_valid.store(true, std::memory_order_relaxed);
            }
        }

        const bool tp_on = sys && cp != 0 && g_veh_tp_active.load(std::memory_order_relaxed);
        if (tp_on && !s_tp_was) g_tp_mount_gen.fetch_add(1, std::memory_order_relaxed);   // the eye re-arms
        if (!tp_on && s_tp_was) {
            g_tp_chassis_yaw_valid.store(false, std::memory_order_relaxed);
            g_tp_eye_valid.store(false, std::memory_order_relaxed);
            g_tp_ncam_valid.store(false, std::memory_order_relaxed);
            g_tp_view_pitch.store(0.0f, std::memory_order_relaxed);
            g_tp_view_roll.store(0.0f, std::memory_order_relaxed);
        }
        s_tp_was = tp_on;
    }

    // RIGHT-STICK TURN (vehstick=1): with motion aim freeing the stick (vehaim), the right stick X turns
    // your VIEW -- a smooth turn, the in-vehicle counterpart of turning on foot. The eye adds
    // g_veh_turn_yaw to the view yaw only: for a camera whose view does not follow the vehicle's yaw
    // that view is independent of the vehicle, and the turn pivots on your head without moving the
    // anchor (only an offset that rides the VIEW swings round with it, by choice). It HOLDS
    // where you leave it (vehorbitreturn=0, default). The first cut orbited the boom and eased back at
    // 60 deg/s, which in-headset read as "the stick lerps my head yaw back to the front of the
    // vehicle". Zeroed on every new ride. The game's own stick-look is masked by the aim write.
    {
        float turn = g_veh_turn_yaw.load(std::memory_order_relaxed);
        const float dt = g_last_dt.load();
        const bool can_turn = veh_tp_motion_aim_active() && g_cfg.veh_stick_mode == 1;
        const float sx = can_turn ? host::g_plugin_state.raw_stick_x->load() : 0.0f;
        const float dz = 0.15f;
        if (std::fabs(sx) > dz) {
            const float s = (sx - (sx > 0.0f ? dz : -dz)) / (1.0f - dz);   // rescale past the deadzone
            turn += s * g_cfg.veh_orbit_rate * dt;
            while (turn >  180.0f) turn -= 360.0f;
            while (turn < -180.0f) turn += 360.0f;
        } else if (g_cfg.veh_orbit_return > 0.0f) {
            const float step = g_cfg.veh_orbit_return * dt;
            if (turn > step) turn -= step; else if (turn < -step) turn += step; else turn = 0.0f;
        }
        g_veh_turn_yaw.store(turn, std::memory_order_relaxed);
    }

    // VEHICLE RAY AIM (vehaimray) -- aim the vehicle at WHERE THE CONTROLLER POINTS.
    //
    // The infantry setpoint (desired_aim_now) is a RELATIVE mapping -- the hand's rotation since the
    // on-foot calibration, added to the aim captured then. That is right while the view is pinned to
    // the gun and wrong here both ways a player can feel: the calibration frame was measured in a view
    // that is not this one (logged 2026-09-23: des yaw 70-90 deg off the camera forward, pitch
    // -85..-95), and the camera sits ~10 m off the vehicle, so even a correct direction carries that
    // much parallax. It read as "rotate the controller in 3DoF to steer".
    //
    // So in a vehicle: (1) the controller's ABSOLUTE world ray, built exactly the way UEVR draws the
    // controller in this camera (veh_room_ray: view base + R(view yaw) . (hand - standing origin) .
    // the LIVE world scale), with the view base recomposed on THIS tick from the eye's published offset and a
    // fresh chassis read; (2) trace it for the point being pointed at, holding the last range on a
    // miss; (3) aim from the seated unit through that point. aim.md's
    // aim = normalize((eye - origin) + range * sightline). GAME THREAD: hit_trace is reflection.
    // Publishes ANGLES for the sim-thread write (BlamDrive.cpp drive_angles_impl), and the aim point
    // for the reticules.
    //
    // THE ORIGIN (vehaimorigin) is where the shot's LINE actually runs, and for a vehicle that is the
    // game's own chase camera, not the seat. The guns converge on what that camera's line of sight
    // hits, so the aim must put THAT line through the crosshair. Aiming from the seat left it 1.5-3 m
    // off (fitted from 112 camera/aim pairs, 2026-09-24) and the Banshee's shots "regularly landed
    // above the crosshair". The camera orbits a pivot along the aim (logged 11-14 m above the Banshee
    // while it aimed down), so it moves when the aim does -- but aiming FROM wherever it sits is a
    // fixed-point iteration whose fixed point is exactly the line through the pivot and T, and it
    // contracts the error by D/(range + D) per tick, monotonically, from ANY start: no pivot or boom
    // length to model or measure per vehicle. Recomposed from the eye's published offset against this
    // tick's chassis read (two clocks). vehaimorigin=0 keeps the seated unit, for A/B.
    //
    // (1) used room_to_world() until 2026-09-24: 100 cm/m instead of UEVR's 100 x VR_WorldScale, anchored
    // at the view base instead of the head. Both shift the ray sideways off the line the hand points along.
    {
        static TrackedObject s_tpc_aim;
        static uintptr_t s_tpc_aim_raw = 0;
        bool ok = false;
        const bool want = veh_tp_motion_aim_active() && g_cfg.veh_aim_ray;
        if (want) {
            const int32_t ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                                     : API::VR::get_right_controller_index();
            Vec3 cpos{}; Quat cq{};
            auto* pawn = API::get()->get_local_pawn(0);
            Vec3 c{};
            if (ridx >= 0 && pawn != nullptr && g_tp_eye_valid.load(std::memory_order_relaxed)
                && get_pose(ridx, &cpos, &cq, /*use_aim=*/true)
                && call_ret_vec3(pawn, L"K2_GetActorLocation", &c)) {
                const uintptr_t cp = g_tp_chassis_ptr.load(std::memory_order_relaxed);
                if (cp != s_tpc_aim_raw) {
                    s_tpc_aim_raw = cp;
                    s_tpc_aim.set_at(reinterpret_cast<API::UObject*>(cp), g_tp_chassis_idx.load(std::memory_order_relaxed));
                }
                auto* ch = (cp != 0) ? s_tpc_aim.get_checked(L"SkeletalMeshComponent") : nullptr;
                // The view base on this tick: the eye's offset from the chassis, hung off the chassis
                // as read NOW -- the pawn, the trace and this read all describe the same game state.
                // No chassis read: the eye's last base as published (off by one frame of travel).
                double base[3] = { (double)g_cam_x.load(std::memory_order_relaxed),
                                   (double)g_cam_y.load(std::memory_order_relaxed),
                                   (double)g_cam_z.load(std::memory_order_relaxed) };
                Vec3 cl{};
                const bool have_cl = ch != nullptr && call_ret_vec3(ch, L"K2_GetComponentLocation", &cl);
                if (have_cl) {
                    base[0] = (double)cl.x + (double)g_tp_eye_ox.load(std::memory_order_relaxed);
                    base[1] = (double)cl.y + (double)g_tp_eye_oy.load(std::memory_order_relaxed);
                    base[2] = (double)cl.z + (double)g_tp_eye_oz.load(std::memory_order_relaxed);
                }
                Vec3 o{}, d{};
                veh_room_ray(cpos, quat_forward(cq), base,
                             g_tp_view_pitch.load(std::memory_order_relaxed),
                             g_tp_chassis_yaw.load(std::memory_order_relaxed),
                             g_tp_view_roll.load(std::memory_order_relaxed), &o, &d);
                if (d.x * d.x + d.y * d.y + d.z * d.z > 0.5f) {
                    // The aim origin: the game's chase camera (vehaimorigin=1), else the seated unit.
                    // Without a fresh chassis read or a published camera, the seat is the safe fallback.
                    bool from_cam = false;
                    if (g_cfg.veh_aim_origin == 1 && have_cl && g_tp_ncam_valid.load(std::memory_order_relaxed)) {
                        c = Vec3{cl.x + g_tp_ncam_ox.load(std::memory_order_relaxed),
                                 cl.y + g_tp_ncam_oy.load(std::memory_order_relaxed),
                                 cl.z + g_tp_ncam_oz.load(std::memory_order_relaxed)};
                        from_cam = true;
                    } else {
                        c.z += g_cfg.veh_aim_pivot_z;
                    }
                    (void)from_cam;   // read only by the dev VEHAIM line below
                    // Ignore our own vehicle and the pawn, so pointing THROUGH the hull reaches the
                    // world beyond it instead of aiming the vehicle at itself.
                    API::UObject* ignore[2] = {}; int ni = 0;
                    ignore[ni++] = pawn;
                    if (ch != nullptr)
                        if (auto* owner = ch->get_outer()) ignore[ni++] = owner;
                    // NOT `far`: <Windows.h> defines `far` (and `near`) as empty macros, which turns
                    // `const float far = ...` into `const float = ...` and wrecks the whole TU.
                    const float far_cm = g_cfg.veh_aim_far;
                    const Vec3 e{o.x + d.x * far_cm, o.y + d.y * far_cm, o.z + d.z * far_cm};
                    Vec3 hit{};
                    float range = veh_aim_range_eff();   // held, or the far end before any hit this ride
                    const bool hitok = far_cm > 1.0f && hit_trace(o, e, ignore, ni, &hit);
                    if (hitok) {
                        const float hx = hit.x - o.x, hy = hit.y - o.y, hz = hit.z - o.z;
                        range = std::sqrt(hx * hx + hy * hy + hz * hz);
                        g_veh_aim_range.store(range, std::memory_order_relaxed);
                    }
                    const Vec3 t{o.x + d.x * range, o.y + d.y * range, o.z + d.z * range};
                    g_veh_aim_tx.store(t.x, std::memory_order_relaxed);
                    g_veh_aim_ty.store(t.y, std::memory_order_relaxed);
                    g_veh_aim_tz.store(t.z, std::memory_order_relaxed);
                    const float ax = t.x - c.x, ay = t.y - c.y, az = t.z - c.z;
                    const float al = std::sqrt(ax * ax + ay * ay + az * az);
                    if (al > 100.0f) {
                        g_veh_aim_yaw.store(std::atan2(ay, ax) * RAD2DEG, std::memory_order_relaxed);
                        g_veh_aim_pitch.store(std::asin(clampf(az / al, -1.0f, 1.0f)) * RAD2DEG, std::memory_order_relaxed);
                        ok = true;
                    } else {
                        // Pointing at the unit itself: the direction is undefined there, so keep the
                        // last good solution rather than falling back to the rotation mapping.
                        ok = g_veh_aim_valid.load(std::memory_order_relaxed);
                    }
#if HALO_VR_DEV
                    // VEHAIM: ray (world yaw/pitch, relative to the camera's yaw), hit/range, the
                    // target, the pivot, the aim sent -- against the old rotation mapping (des).
                    if (g_cfg.veh_probe) {
                        static uint32_t s_aimlog = 0;
                        if ((s_aimlog++ % 30u) == 0u) {
                            float dy = 0.0f, dp = 0.0f;
                            const bool hd = desired_aim_now(&dy, &dp);
                            const float ry = std::atan2(d.y, d.x) * RAD2DEG;
                            const float rp = std::asin(clampf(d.z, -1.0f, 1.0f)) * RAD2DEG;
                            API::get()->log_info(
                                "[Halo-CampE-UEVR] VEHAIM ray(y=%.1f p=%.1f rel-cam=%.1f) %s range=%.0f "
                                "o=(%.0f %.0f %.0f) t=(%.0f %.0f %.0f) pivot=(%.0f %.0f %.0f) org=%s "
                                "aim(y=%.1f p=%.1f) des(y=%.1f p=%.1f ok%d) ok=%d",
                                ry, rp, wrap180(ry - halo::g_view_base_yaw.load(std::memory_order_relaxed)),
                                hitok ? "HIT" : "miss", range, o.x, o.y, o.z, t.x, t.y, t.z, c.x, c.y, c.z,
                                from_cam ? "cam" : "seat",
                                g_veh_aim_yaw.load(), g_veh_aim_pitch.load(), dy, dp, (int)hd, (int)ok);
                        }
                    }
#endif
                }
            }
        } else {
            g_veh_aim_range.store(0.0f, std::memory_order_relaxed);   // next ride measures afresh
        }
        g_veh_aim_valid.store(ok, std::memory_order_relaxed);
    }

    // ROUTE A P0: CAMERA COLLISION (spring arm). Trace game-side from the camera's origin point (the
    // vehicle's origin, or the seat) to the desired endpoint and publish a scale the render eye applies to
    // the offset -- hit_trace() is a reflection LineTraceSingle and must run on the tick, not render-side
    // (two clocks: publish a fraction game-side, consume it at render rate). Ignore the vehicle actor
    // (the chassis mesh's outer) and the pawn, or the trace collapses onto the hull. Per camera
    // ("collide", "collideMargin" in the camera file).
    const VehActiveCam col_cam = veh_active_cam();
    if (g_veh_tp_active.load(std::memory_order_relaxed) && col_cam.collide
        && halo::g_stick_mode_active.load(std::memory_order_relaxed)) {
        static TrackedObject s_tpc_col;
        static uintptr_t s_tpc_col_raw = 0;
        const uintptr_t cp = g_tp_chassis_ptr.load(std::memory_order_relaxed);
        if (cp != s_tpc_col_raw) {
            s_tpc_col_raw = cp;
            s_tpc_col.set_at(reinterpret_cast<API::UObject*>(cp), g_tp_chassis_idx.load(std::memory_order_relaxed));
        }
        auto* ch = (cp != 0) ? s_tpc_col.get_checked(L"SkeletalMeshComponent") : nullptr;
        Vec3 cloc{};
        float frac = 1.0f;
        if (ch != nullptr && call_ret_vec3(ch, L"K2_GetComponentLocation", &cloc)) {
            // Trace toward where the camera ACTUALLY is: the eye's own offsets from last frame, unscaled
            // -- the origin point (vehicle or seat) and the offset from it, which may ride the vehicle's
            // full attitude or the view's yaw, so a boom rebuilt here from any one yaw would trace
            // somewhere else. Offsets, so this tick's fresh chassis location is what they hang off
            // (two clocks).
            const Vec3 start{ cloc.x + g_tp_origin_ox.load(std::memory_order_relaxed),
                              cloc.y + g_tp_origin_oy.load(std::memory_order_relaxed),
                              cloc.z + g_tp_origin_oz.load(std::memory_order_relaxed) };
            const double ox = (double)g_tp_boom_ox.load(std::memory_order_relaxed);
            const double oy = (double)g_tp_boom_oy.load(std::memory_order_relaxed);
            const double oz = (double)g_tp_boom_oz.load(std::memory_order_relaxed);
            const double boom_len = std::sqrt(ox * ox + oy * oy + oz * oz);
            const Vec3 desired{ (float)((double)start.x + ox),
                                (float)((double)start.y + oy),
                                (float)((double)start.z + oz) };
            API::UObject* ignore[2] = {}; int ni = 0;
            if (auto* pawn = API::get()->get_local_pawn(0)) ignore[ni++] = pawn;
            if (auto* owner = ch->get_outer()) ignore[ni++] = owner;
            Vec3 hit{};
            if (boom_len > 1.0 && hit_trace(start, desired, ignore, ni, &hit)) {
                const double dx = (double)hit.x - (double)start.x;
                const double dy = (double)hit.y - (double)start.y;
                const double dz = (double)hit.z - (double)start.z;
                const double hd = std::sqrt(dx * dx + dy * dy + dz * dz);
                double f = (hd - (double)col_cam.collide_margin) / boom_len;
                if (f < 0.10) f = 0.10;   // never collapse into the hull / first person
                if (f > 1.00) f = 1.00;
                frac = (float)f;
            }
        }
        g_tp_collision_frac.store(frac, std::memory_order_relaxed);
    } else {
        g_tp_collision_frac.store(1.0f, std::memory_order_relaxed);
    }

    vehprobe_tick();   // dev-only; empty stub in a release build
}

void vehcam_stereo_pre_eye_seat(int index, UEVR_Vector3f* position, UEVR_Rotatorf* rotation, bool is_double) {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    // Plugin.cpp's view position mirror, through the bridge.
    auto& g_view_pos_x = *host::g_plugin_state.view_pos_x;
    auto& g_view_pos_y = *host::g_plugin_state.view_pos_y;
    auto& g_view_pos_z = *host::g_plugin_state.view_pos_z;

    // ---- ROUTE A P0: OWNED THIRD-PERSON CAMERA. Self-contained; bypasses bc24's first-person seat
    // machinery below, and independent of veh_cam. Boom BEHIND the chassis mesh (g_tp_chassis,
    // resolved game-side off the player pawn), read at render rate so it tracks the moving vehicle;
    // the view yaw is published here for the view override. Gated on stick mode (the mount flag is
    // dead). Head free-look composes on top via UEVR.
    if (g_veh_tp_active.load(std::memory_order_relaxed) && halo::g_stick_mode_active.load(std::memory_order_relaxed)) {
        static TrackedObject s_tpc;
        static uintptr_t s_tpc_raw = 0;
        const uintptr_t cp = halo::g_tp_chassis_ptr.load(std::memory_order_relaxed);
        if (cp != s_tpc_raw) {
            s_tpc_raw = cp;
            s_tpc.set_at(reinterpret_cast<API::UObject*>(cp), halo::g_tp_chassis_idx.load(std::memory_order_relaxed));
        }
        // The engine's camera as handed to us -- captured so the diagnostic can show whether our
        // boom actually moves it (game vs eye), and so a failed resolve leaves it untouched.
        double ogx = 0, ogy = 0, ogz = 0;
        if (is_double) { auto* p = reinterpret_cast<UEVR_Vector3d*>(position); ogx = p->x; ogy = p->y; ogz = p->z; }
        else { ogx = position->x; ogy = position->y; ogz = position->z; }
        auto* ch = (cp != 0) ? s_tpc.get_checked(L"SkeletalMeshComponent") : nullptr;
        Vec3 cloc{}, crot{};
        const bool locOk = (ch != nullptr) && call_ret_vec3(ch, L"K2_GetComponentLocation", &cloc);
        const bool rotOk = locOk && call_ret_vec3(ch, L"K2_GetComponentRotation", &crot);
        double ecx = ogx, ecy = ogy, ecz = ogz;
        if (rotOk) {
            const double D2R = 0.01745329252, R2D = 57.29577951;
            // THE SELECTED CAMERA (halo_vr_vehcams.json via VehCamSelect.cpp): a plain copy, so one
            // frame never mixes two cameras' settings.
            const VehActiveCam ac = halo::veh_active_cam();
            // RE-ARM PER RIDE. The vehicle's frame, the frozen view heading and the head anchor are
            // captured when our camera comes up: a new mount, stepping back from a first-person entry,
            // or a different chassis. (The frozen yaw used to latch once per SESSION, so every later
            // vehicle inherited the first one's heading.) Stepping between two of OUR cameras re-arms
            // nothing, so the view carries on where it is. An hmdleash flip re-captures only the head
            // offset, so editing the leash does not also snap your view to the hull.
            static uint32_t  s_arm_gen = 0xFFFFFFFFu;
            static uintptr_t s_arm_cp = 0;
            static float     s_frozen_yaw = 0.0f;
            static float     s_loc_heading = 0.0f;   // the vehicle's heading at mount: an untracked-yaw offset keeps it
            static double    s_c0[3] = {0.0, 0.0, 0.0};   // head offset from the standing origin at capture, UE cm
            static bool      s_arm_leash = false;
            // Per ride: the VEHICLE's frame along the chassis mesh's axes (capture_vehicle_frame). The
            // FRAME is captured, never an offset -- so every camera's offset stays live mid-ride.
            static double    s_C[9] = {1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0};
            // hmdleash IS RESPECTED here (the user's call, 2026-09-24). With it on, Plugin.cpp's leash
            // keeps running under this camera and slides the standing origin onto your head exactly as
            // on foot, so the anchor plays the part of your body: the head offset it backs out is zero
            // by definition, and the leash radius (0/0 = none) is all the lean you get. With it off,
            // the leash block stands down and you lean freely off the anchor -- the captured c0 below.
            const bool leash = g_cfg.hmd_leash;
            // The mesh's FULL world axes -- the rows of UE's FRotationMatrix for crot (pitch x, yaw y,
            // roll z). This is the basis that tumbled the boom when the boom was applied IN it directly
            // (measured: boom.up=+1000 landed at world Z -785..-105, the camera diving under the hull):
            // it carries the mesh's baked modelling axes. The vehicle frame goes through this same
            // basis at capture and at apply, so the baked part cancels and only real motion is left.
            double MX[3], MY[3], MZ[3];
            rot_axes(crot.x, crot.y, crot.z, MX, MY, MZ);
            const uint32_t gen = halo::g_tp_mount_gen.load(std::memory_order_relaxed);
            const bool rearm = (gen != s_arm_gen || cp != s_arm_cp);
            // A world-scale change mid-ride re-captures too: c0 is in UE cm at the scale it was read at.
            static float s_arm_cmpm = 0.0f;
            const float cmpm = halo::veh_cm_per_m();
            const bool recapture_head = rearm || leash != s_arm_leash || std::fabs(cmpm - s_arm_cmpm) > 0.05f;
            s_arm_cmpm = cmpm;
            if (rearm) {
                s_arm_gen = gen; s_arm_cp = cp;
                bool snapped = false;
                capture_vehicle_frame(MX, MY, MZ, crot.y, s_C, &snapped);
                // The frozen view heading starts as the vehicle's own heading -- and so does the fixed
                // heading an offset keeps when its location does not track yaw.
                const double f0 = s_C[0] * MX[0] + s_C[3] * MY[0] + s_C[6] * MZ[0];
                const double f1 = s_C[0] * MX[1] + s_C[3] * MY[1] + s_C[6] * MZ[1];
                s_frozen_yaw = (float)(std::atan2(f1, f0) * R2D);
                s_loc_heading = s_frozen_yaw;
#if HALO_VR_DEV
                // Once per ride; dev only -- the render thread does not log in a player build.
                API::get()->log_info("[Halo-CampE-UEVR] VEHTP: vehicle frame %s (mesh rot p=%.1f y=%.1f r=%.1f)",
                                     snapped ? "snapped to the mesh's axes"
                                             : "taken level at mount (the mesh sat over 30 deg off its axes)",
                                     (double)crot.x, (double)crot.y, (double)crot.z);
#else
                (void)snapped;
#endif
            }
            if (recapture_head) {
                s_arm_leash = leash;
                s_c0[0] = s_c0[1] = s_c0[2] = 0.0;
                if (!leash) {
                    // HEAD-CENTRIC, NOT PLAY-SPACE-CENTRIC. UEVR draws the head at
                    //   view_base + R(view_yaw) . conv(hmd - standing_origin) . scale
                    // so rotating the view base swings an off-centre head around the ROOM origin, and the
                    // head's place relative to the vehicle would depend on where the player happened to
                    // stand when they got in. Capture that offset once and back it out below
                    // (view_base = anchor - R(view_yaw) . c0): the HEAD then sits on the anchor and is the
                    // rotation pivot, and the live head position cancels out of the sum, so leaning
                    // still moves you off the anchor (6DoF). With hmdleash off Plugin.cpp stands the leash
                    // block down, so the standing origin holds still and c0 stays valid. With it ON there
                    // is nothing to capture: the leash keeps the head on the standing origin, so c0 = 0.
                    // Re-captured on a live leash flip, so turning the leash off mid-ride leaves your
                    // head exactly where it was instead of jumping by the offset it had absorbed.
                    Vec3 hp{}; Quat hq{};
                    const auto hi = API::VR::get_hmd_index();
                    if (hi >= 0 && get_pose(hi, &hp, &hq, /*use_aim=*/false)) {
                        const auto so = API::VR::get_standing_origin();
                        const double rs = (double)halo::veh_cm_per_m(); // UE cm per real metre, LIVE world scale
                        s_c0[0] = -((double)hp.z - (double)so.z) * rs; // room -> UE: X = -z, Y = x, Z = y
                        s_c0[1] =  ((double)hp.x - (double)so.x) * rs;
                        s_c0[2] =  ((double)hp.y - (double)so.y) * rs;
                    }
                }
            }
            // THE VEHICLE'S FRAME NOW: R_mesh . C -- its forward, right and up as world directions.
            double VF[3], VR[3], VU[3];
            vehicle_axes(s_C, MX, MY, MZ, VF, VR, VU);
            const double heading = std::atan2(VF[1], VF[0]) * R2D;
            if (ac.rot_yaw) s_frozen_yaw = (float)heading;   // keep the freeze current -> seamless when you step to one that holds
            const float turn = g_veh_turn_yaw.load(std::memory_order_relaxed);   // right-stick turn (vehstick=1)
            // THE VIEW ROTATION, from the camera's rotationTracking (vehcammath::tracked_frame). Nothing
            // tracked: the heading captured this ride, level, and the vehicle turns within your view. Yaw:
            // the vehicle's heading. Pitch / roll: your view tilts with the vehicle's deck -- about the
            // vehicle's OWN axes, so with yaw untracked a pitching Banshee seen side-on rolls your
            // horizon, as it would standing on its deck. The right-stick turn turns the view about its
            // own up, and nothing else. UEVR keeps pitch and roll only with its decoupled pitch off,
            // which VehCamSelect.cpp arranges while such a camera is up.
            double WF[3], WR[3], WU[3];
            tracked_frame(VF, VR, VU, ac.rot_yaw, ac.rot_pitch, ac.rot_roll, (double)s_frozen_yaw, (double)turn,
                          WF, WR, WU);
            double vp = 0.0, vy = 0.0, vrl = 0.0;
            rotator_from_axes(WF, WR, WU, &vp, &vy, &vrl);
            if (!ac.rot_pitch && !ac.rot_roll) { vp = 0.0; vrl = 0.0; }   // exactly level, not level to rounding
            const float view_yaw = (float)vy;
            // Spring-arm pull-in from the game-side collision trace (1 = unobstructed).
            const double cfrac = ac.collide ? (double)halo::g_tp_collision_frac.load(std::memory_order_relaxed) : 1.0;
            // WHERE THE OFFSET IS MEASURED FROM: the vehicle's origin, or your seat -- the pawn's offset
            // the tick measured in the mesh's frame, rebuilt against its rotation now, so it rides the
            // vehicle rigidly at render rate. Until the tick has measured it, the vehicle's origin.
            double org[3] = { 0.0, 0.0, 0.0 };
            if (ac.origin == static_cast<uint8_t>(vehcampresets::Origin::Seat)
                && halo::g_tp_seat_valid.load(std::memory_order_relaxed)) {
                const double sx = halo::g_tp_seat_x.load(std::memory_order_relaxed);
                const double sy = halo::g_tp_seat_y.load(std::memory_order_relaxed);
                const double sz = halo::g_tp_seat_z.load(std::memory_order_relaxed);
                for (int k = 0; k < 3; ++k) org[k] = sx * MX[k] + sy * MY[k] + sz * MZ[k];
            }
            // THE OFFSET, in the frame its locationTracking describes -- the same builder, so "pitch" means
            // the same thing for where the camera sits as for where it looks. All three: rigid to the
            // vehicle, holding its place through yaw, pitch and bank. Yaw only: a level offset that turns
            // with it. None: a fixed world direction from the vehicle (its heading when you got in).
            // "view": the VIEW's own frame, so the camera orbits the vehicle as you turn -- the only case
            // the stick turn moves it; otherwise the stick turns your view ABOUT your head. A live edit to
            // the file lands immediately either way: the frame is captured, never the offset.
            const double bf = (double)ac.offset[0], bl = (double)ac.offset[1], bu = (double)ac.offset[2];
            double LF[3], LR[3], LU[3];
            if (ac.loc_view) {
                for (int k = 0; k < 3; ++k) { LF[k] = WF[k]; LR[k] = WR[k]; LU[k] = WU[k]; }
            } else {
                tracked_frame(VF, VR, VU, ac.loc_yaw, ac.loc_pitch, ac.loc_roll, (double)s_loc_heading, 0.0,
                              LF, LR, LU);
            }
            double off[3];
            for (int k = 0; k < 3; ++k) off[k] = bf * LF[k] + bl * LR[k] + bu * LU[k];
            ecx = (double)cloc.x + org[0] + off[0] * cfrac;
            ecy = (double)cloc.y + org[1] + off[1] * cfrac;
            ecz = (double)cloc.z + org[2] + off[2] * cfrac;
            {
                // view_base = anchor - R(view) . c0 -- puts the HEAD (not the room origin) on the anchor
                // just computed. R(view) is the FULL view rotation, because that is what UEVR turns the
                // head's room offset by; it is the yaw-only one unless the camera tilts.
                double X[3], Y[3], Z[3];
                rot_axes(vp, vy, vrl, X, Y, Z);
                ecx -= s_c0[0] * X[0] + s_c0[1] * Y[0] + s_c0[2] * Z[0];
                ecy -= s_c0[0] * X[1] + s_c0[1] * Y[1] + s_c0[2] * Z[1];
                ecz -= s_c0[0] * X[2] + s_c0[1] * Y[2] + s_c0[2] * Z[2];
            }
            if (is_double) { auto* p = reinterpret_cast<UEVR_Vector3d*>(position); p->x = ecx; p->y = ecy; p->z = ecz; }
            else { position->x = (float)ecx; position->y = (float)ecy; position->z = (float)ecz; }
            g_view_pos_x = (float)ecx; g_view_pos_y = (float)ecy; g_view_pos_z = (float)ecz;
            halo::g_cam_x.store((float)ecx, std::memory_order_relaxed);
            halo::g_cam_y.store((float)ecy, std::memory_order_relaxed);
            halo::g_cam_z.store((float)ecz, std::memory_order_relaxed);
            halo::g_tp_chassis_yaw.store(view_yaw, std::memory_order_relaxed);   // the VIEW rotation -> view override
            halo::g_tp_view_pitch.store((float)vp, std::memory_order_relaxed);   //   and the tick's controller ray
            halo::g_tp_view_roll.store((float)vrl, std::memory_order_relaxed);
            halo::g_tp_origin_ox.store((float)org[0], std::memory_order_relaxed); // the offset's ORIGIN (vehicle or
            halo::g_tp_origin_oy.store((float)org[1], std::memory_order_relaxed); //   seat) -> the collision trace
            halo::g_tp_origin_oz.store((float)org[2], std::memory_order_relaxed);
            halo::g_tp_boom_ox.store((float)off[0], std::memory_order_relaxed);  // the anchor OFFSET, unscaled ->
            halo::g_tp_boom_oy.store((float)off[1], std::memory_order_relaxed);  //   the collision trace (it adds
            halo::g_tp_boom_oz.store((float)off[2], std::memory_order_relaxed);  //   its own fresh chassis location)
            halo::g_tp_eye_ox.store((float)(ecx - (double)cloc.x), std::memory_order_relaxed);   // the VIEW BASE as an
            halo::g_tp_eye_oy.store((float)(ecy - (double)cloc.y), std::memory_order_relaxed);   //   offset -> the tick's
            halo::g_tp_eye_oz.store((float)(ecz - (double)cloc.z), std::memory_order_relaxed);   //   controller ray
            halo::g_tp_eye_valid.store(true, std::memory_order_relaxed);
            halo::g_tp_ncam_ox.store((float)(ogx - (double)cloc.x), std::memory_order_relaxed);   // the GAME's camera
            halo::g_tp_ncam_oy.store((float)(ogy - (double)cloc.y), std::memory_order_relaxed);   //   as an offset -> the
            halo::g_tp_ncam_oz.store((float)(ogz - (double)cloc.z), std::memory_order_relaxed);   //   tick's aim origin
            halo::g_tp_ncam_valid.store(true, std::memory_order_relaxed);
            halo::g_tp_chassis_yaw_valid.store(true, std::memory_order_relaxed);
            if (index == 0) { g_vcd.fc[0] = ecx; g_vcd.fc[1] = ecy; g_vcd.fc[2] = ecz; g_vcd.wrote = true; }

            // PER-FRAME RETICULE STAMP (vehaimray). The ONE compositor reticule on the LIVE pointing
            // ray, at the range the tick measured -- the on-foot stamp's rule (AimReticuleStamp: intent
            // at render rate, trace depth from the tick), with the ray starting at the hand. Publishing
            // the tick's world point instead left it up to a tick behind the hand, and head-relative
            // re-anchoring carried a GROUND point along with the vehicle between ticks. Built against
            // this frame's view base and view rotation, so hand, camera and marker share one instant: a
            // RANGE crosses the clock boundary, never a position. Eye 0 only (one mono publish; the
            // layer re-anchors it for both eyes against this same view base).
            if (index == 0 && halo::veh_tp_reticle_stamp_owns()) {
                const int32_t ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                                         : API::VR::get_right_controller_index();
                Vec3 cpos{}; Quat cq{};
                if (ridx >= 0 && get_pose(ridx, &cpos, &cq, /*use_aim=*/true)) {
                    const double base[3] = { ecx, ecy, ecz };
                    Vec3 o{}, d{};
                    halo::veh_room_ray(cpos, quat_forward(cq), base, (float)vp, view_yaw, (float)vrl, &o, &d);
                    const float range = halo::veh_aim_range_eff();
                    // The same surface pull-back the tick placement uses, so the three reticules agree.
                    const float r = range - std::fmin(g_cfg.aim_reticule_surface_off, range * 0.5f);
                    const Vec3 t{o.x + d.x * r, o.y + d.y * r, o.z + d.z * r};
                    const auto layer_anchor = host::g_plugin_state.layer_anchor;
                    halo::xrlayer_note_publish_gate(0);
                    halo::xrlayer_notice_reticule(layer_anchor(halo::XRLAYER_SLOT_RETICULE, t), g_ret_scale_mul.load());
                }
            }
        } else if (index == 0) {
            g_vcd.wrote = false;
            halo::g_tp_chassis_yaw_valid.store(false, std::memory_order_relaxed);
            halo::g_tp_eye_valid.store(false, std::memory_order_relaxed);
            halo::g_tp_ncam_valid.store(false, std::memory_order_relaxed);
        }
        // vehprobe diagnostic: is the boom computed, and does it differ from the engine camera?
        // If game vs eye differ here but you see no change in-headset, the write is being ignored
        // downstream; if locOk/rotOk are 0, the render-side chassis read is the fault.
        // Compiled out of player builds: a config flag is not a sufficient guard (CLAUDE.md).
#if HALO_VR_DEV
        if (index == 0 && g_cfg.veh_probe) {
            static uint32_t s_dbg = 0;
            if ((s_dbg++ % 90u) == 0u) {
                // gcam = the GAME camera's own rotation as handed to us (the override runs after this
                // callback), beside the aim we are writing: the camera-origin aim assumes the camera
                // looks along that aim, and this pair is what checks it -- at the same instant as game=.
                double gp = 0.0, gy = 0.0;
                if (rotation != nullptr) {
                    if (is_double) { auto* r = reinterpret_cast<UEVR_Rotatord*>(rotation); gp = r->pitch; gy = r->yaw; }
                    else { gp = rotation->pitch; gy = rotation->yaw; }
                }
                const VehActiveCam lac = halo::veh_active_cam();
                API::get()->log_info("[Halo-CampE-UEVR] VEHTP eye: cp=0x%llX ch=%d locOk=%d rotOk=%d "
                                     "game=(%.0f %.0f %.0f) chassis=(%.0f %.0f %.0f) rot(p=%.1f y=%.1f r=%.1f) "
                                     "eye=(%.0f %.0f %.0f) cam=%d/%d offset=(%.0f %.0f %.0f) origin=%d loc=%d%d%d%s rot=%d%d%d "
                                     "view(p=%.1f y=%.1f r=%.1f) gcam(p=%.1f y=%.1f) aimw(p=%.1f y=%.1f)",
                                     (unsigned long long)cp, (int)(ch != nullptr), (int)locOk, (int)rotOk,
                                     ogx, ogy, ogz, (double)cloc.x, (double)cloc.y, (double)cloc.z,
                                     (double)crot.x, (double)crot.y, (double)crot.z,
                                     ecx, ecy, ecz, lac.index + 1, lac.count,
                                     (double)lac.offset[0], (double)lac.offset[1], (double)lac.offset[2],
                                     (int)lac.origin, (int)lac.loc_yaw, (int)lac.loc_pitch, (int)lac.loc_roll,
                                     lac.loc_view ? "(view)" : "", (int)lac.rot_yaw, (int)lac.rot_pitch, (int)lac.rot_roll,
                                     (double)halo::g_tp_view_pitch.load(std::memory_order_relaxed),
                                     (double)halo::g_tp_chassis_yaw.load(std::memory_order_relaxed),
                                     (double)halo::g_tp_view_roll.load(std::memory_order_relaxed),
                                     gp, gy, (double)halo::g_veh_aim_pitch.load(std::memory_order_relaxed),
                                     (double)halo::g_veh_aim_yaw.load(std::memory_order_relaxed));
            }
        }
#endif
        return;   // third-person owns the eye; skip the first-person path
    }

    // Off: the camera is the engine's. The same bookkeeping the ungated path does on eye 0. (On while a
    // first-person entry is selected in the camera file, even with vehcam=0: see veh_cam_mode.)
    const int vcm = veh_cam_mode(g_cfg.veh_cam);
    if (vcm == 0) { if (index == 0) g_vcd.wrote = false; return; }

            // The ENGINE's camera and view yaw, captured before anything below modifies them. The
            // seat anchor is built from these; see the vehcam section.
            double rawcx = 0.0, rawcy = 0.0, rawcz = 0.0, rawgyaw = 0.0, rawgpitch = 0.0;
            if (is_double) {
                auto* p = reinterpret_cast<UEVR_Vector3d*>(position);
                rawcx = p->x; rawcy = p->y; rawcz = p->z;
                if (rotation != nullptr) {
                    rawgyaw = reinterpret_cast<UEVR_Rotatord*>(rotation)->yaw;
                    rawgpitch = reinterpret_cast<UEVR_Rotatord*>(rotation)->pitch;
                }
            } else {
                rawcx = position->x; rawcy = position->y; rawcz = position->z;
                if (rotation != nullptr) { rawgyaw = rotation->yaw; rawgpitch = rotation->pitch; }
            }
            // ---- CHASE-CAM ANCHOR PROBE (vehanchor = sample every N calls, 0 = off). READ-ONLY.
            // Records what the engine hands us before the seat camera touches it, with the rider
            // and vehicle Blam positions and the vehicle facing beside it, so the boom frame can be
            // fitted from the log rather than assumed.
            if (g_cfg.veh_anchor > 0 && halo::g_unit_mounted.load(std::memory_order_relaxed)) {
                static uint32_t an = 0;
                if ((an++ % (uint32_t)g_cfg.veh_anchor) == 0u) {
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] VEHANCHOR: cam=(%.2f %.2f %.2f) gyaw=%.2f gpitch=%.2f "
                        "vpos=(%.4f %.4f %.4f) bpos=(%.4f %.4f %.4f) face=(%.4f %.4f) fvalid=%d",
                        rawcx, rawcy, rawcz, rawgyaw, rawgpitch,
                        halo::g_vehpx.load(std::memory_order_relaxed),
                        halo::g_vehpy.load(std::memory_order_relaxed),
                        halo::g_vehpz.load(std::memory_order_relaxed),
                        halo::g_unit_px.load(std::memory_order_relaxed),
                        halo::g_unit_py.load(std::memory_order_relaxed),
                        halo::g_unit_pz.load(std::memory_order_relaxed),
                        halo::g_veh_fx.load(std::memory_order_relaxed),
                        halo::g_veh_fy.load(std::memory_order_relaxed),
                        (int)halo::g_veh_fvalid.load(std::memory_order_relaxed));
                }
            }
            // ---- FIRST PERSON IN A VEHICLE (vehcam).
            //
            // Halo's vehicle camera is a third-person chase cam, unusable in VR. There is no
            // first-person mode to switch on, so the rendered position is moved to the seat.
            //
            // Do not SYNTHESISE a camera from the Blam position (a 300-500 Hz physics value) while
            // the mesh is drawn from a frame-rate snapshot of that same physics: two curves at two
            // rates, and the breathing gap between them is judder that parallax puts on the hog.
            //
            // vehcamanchor=2 (default): the camera rides the hog's DRAWN hull component. Fallback
            // vehcamanchor=1: seat = cam - R(gyaw) . boom, the boom rigid in the AIM frame (measured
            // +-10 cm lateral scatter there against +-513 world and +-133 vehicle facing), only the
            // boom LENGTH filtered, never the position. vehcamanchor=0: the old synthesised camera.
            //
            // Runs only while mounted (or always with vehcam=2), so the on-foot view is untouched.
            if (index == 0) halo::seat_direct_refresh();   // vehseatdirect: render-rate rider read
            // A first-person entry selected in the camera file counts as mounted: the vehicle it was
            // selected for is identified, and the Blam mount flag has read dead on this build.
            const bool vc_gate = vcm != 0 && halo::g_unit_pvalid.load(std::memory_order_relaxed)
                && (vcm == 2 || halo::g_unit_mounted.load(std::memory_order_relaxed) || veh_fp_selected());
            // VEHSEAT (vehlog): once a second while seated or in stick mode, everything the "camera
            // stays behind the hog" question needs in one line -- the rider as published, the hull
            // as drawn, whether the publish is alive (calls/s, stale ms), what gated it (mounted,
            // pvalid, record), what the learn/snap did, and how many frames each path rendered.
            if (index == 0) {
                const bool stick_now = halo::g_stick_mode_active.load(std::memory_order_relaxed);
                if (!vc_gate && vcm != 0 && stick_now) ++g_vcd.frames[VCP_NONE];
                static auto s_vs_t = std::chrono::steady_clock::now();
                static uint32_t s_c0 = 0, s_n0 = 0, s_r0 = 0, s_d0 = 0;
                const auto vnow = std::chrono::steady_clock::now();
                const float vel = std::chrono::duration<float>(vnow - s_vs_t).count();
                if (vel >= 1.0f) {
                    const uint32_t c = halo::g_seat_pub_calls.load(std::memory_order_relaxed);
                    const uint32_t n = halo::g_seat_norec.load(std::memory_order_relaxed);
                    const uint32_t r = halo::g_seat_reresolve.load(std::memory_order_relaxed);
                    const uint32_t d = halo::g_seat_direct_reads.load(std::memory_order_relaxed);
                    const bool mounted_now = halo::g_unit_mounted.load(std::memory_order_relaxed);
                    const uint32_t fsum = g_vcd.frames[0] + g_vcd.frames[1] + g_vcd.frames[2]
                                        + g_vcd.frames[3] + g_vcd.frames[4];
                    if (g_cfg.veh_log && (mounted_now || stick_now || fsum != 0)) {
                        const double bx = halo::g_unit_px.load(std::memory_order_relaxed);
                        const double by = halo::g_unit_py.load(std::memory_order_relaxed);
                        const double bz = halo::g_unit_pz.load(std::memory_order_relaxed);
                        const double S = g_cfg.veh_cam_scale;
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] VEHSEAT: mounted=%d pvalid=%d stick=%d rec=%d | rider=(%.3f %.3f %.3f)wu ue=(%.0f %.0f %.0f) "
                            "hull=%s(%.0f %.0f %.0f) hullspeed=%.0fcm/s speed=%.2fwu/s | pub/s=%.0f norec/s=%.0f reresolve/s=%.1f direct/s=%.0f stale=%.0fms | "
                            "learn=%d learnframes=%u snaps=%u refused=%u off=(%.0f %.0f %.0f) | frames rigid=%u hold=%u chase=%u synth=%u gateoff=%u | "
                            "seatpub=%d direct=%d guard=%d hullcheck=%d anchor=%d src=%d | veh=(%.0f %.0f %.0f)ue fvalid=%d hulldead=%d | "
                            "post: frames=%u bad=%u maxdev=%.0fcm seat=(%.0f %.0f %.0f)",
                            (int)mounted_now, (int)halo::g_unit_pvalid.load(std::memory_order_relaxed), (int)stick_now,
                            (int)(halo::blam_control_record() != 0),
                            bx, by, bz, bx * S, -by * S, bz * S,
                            g_vcd.have_hull ? "" : "UNSEEN", g_vcd.hull[0], g_vcd.hull[1], g_vcd.hull[2],
                            g_vcd.hspeed, (double)halo::g_veh_speed.load(std::memory_order_relaxed),
                            (double)(c - s_c0) / vel, (double)(n - s_n0) / vel, (double)(r - s_r0) / vel,
                            (double)(d - s_d0) / vel, (double)g_vcd.stale_ms,
                            (int)g_vcd.learning, g_vcd.learn_frames, g_vcd.snaps, g_vcd.snaps_refused,
                            g_vcd.off[0], g_vcd.off[1], g_vcd.off[2],
                            g_vcd.frames[VCP_RIGID], g_vcd.frames[VCP_HOLD], g_vcd.frames[VCP_CHASE],
                            g_vcd.frames[VCP_SYNTH], g_vcd.frames[VCP_NONE],
                            g_cfg.veh_seat_pub, g_cfg.veh_seat_direct, g_cfg.veh_cam_guard,
                            g_cfg.veh_cam_hull_check, g_cfg.veh_cam_anchor, g_cfg.veh_cam_src,
                            (double)halo::g_vehpx.load(std::memory_order_relaxed) * S,
                            -(double)halo::g_vehpy.load(std::memory_order_relaxed) * S,
                            (double)halo::g_vehpz.load(std::memory_order_relaxed) * S,
                            (int)halo::g_veh_fvalid.load(std::memory_order_relaxed), (int)g_vcd.hull_dead,
                            g_vcd.post_frames, g_vcd.post_bad, g_vcd.post_max,
                            g_vcd.fc[0], g_vcd.fc[1], g_vcd.fc[2]);
                    }
                    s_vs_t = vnow; s_c0 = c; s_n0 = n; s_r0 = r; s_d0 = d;
                    g_vcd.post_frames = 0; g_vcd.post_bad = 0; g_vcd.post_max = 0.0;
                    for (auto& f : g_vcd.frames) f = 0;
                    g_vcd.learn_frames = 0;
                    g_vcd.have_hull = false;
                }
            }
            if (vc_gate) {
                const float S = g_cfg.veh_cam_scale;
                const auto cnow = std::chrono::steady_clock::now();
                // Which body the boom is measured against. The rider is the default: it lands on
                // 95-100% of frames against 91-97% for the vehicle, and it is already AT the seat.
                const bool use_veh = (g_cfg.veh_cam_src != 0)
                                  && halo::g_veh_fvalid.load(std::memory_order_relaxed);
                const double bpx = use_veh ? (double)halo::g_vehpx.load(std::memory_order_relaxed)
                                           : (double)halo::g_unit_px.load(std::memory_order_relaxed);
                const double bpy = use_veh ? (double)halo::g_vehpy.load(std::memory_order_relaxed)
                                           : (double)halo::g_unit_py.load(std::memory_order_relaxed);
                const double bpz = use_veh ? (double)halo::g_vehpz.load(std::memory_order_relaxed)
                                           : (double)halo::g_unit_pz.load(std::memory_order_relaxed);
                // LATCHED ONCE PER FRAME. This callback runs per EYE. Reading the source fresh on
                // each eye let a publish land BETWEEN the two calls, rendering left and right from
                // positions up to a frame of travel apart (~47 cm of bogus disparity, on the hog and
                // nowhere else). Compute on eye 0, reuse on the other.
                static double fcx = 0.0, fcy = 0.0, fcz = 0.0;
                static bool   fvalid = false;
                static std::chrono::steady_clock::time_point tframe{};
                static bool   tframe_ok = false;
                const float since = tframe_ok
                    ? std::chrono::duration<float>(cnow - tframe).count() : 1.0f;
                if (index == 0 || !fvalid || since > 0.050f) {
                    float fdt = tframe_ok ? since : 0.0f;
                    tframe = cnow; tframe_ok = true;
                    if (!(fdt > 0.0f) || fdt > 0.10f) fdt = 0.011f;   // first frame, or a hitch
                    // Fallback path (vehcamanchor=0): the synthesised camera, kept for A/B only.
                    double lcx = bpx * S, lcy = -bpy * S, lcz = bpz * S;
                    // ---- RIGID VEHICLE CAMERA (vehcamanchor=2).
                    //
                    // Bolt the camera to the hog's own DRAWN component: one source, zero filters on
                    // position, so neither the chase-cam spring nor the two-curve judder exists.
                    //
                    // The seat offset is LEARNED, not configured: rider minus hull in the hull's
                    // FULL rotation frame (a yaw-only frame read slope pitch as the offset moving),
                    // learned ONLY WHILE PARKED (speed < 0.5 wu/s) and frozen while moving. At speed
                    // the sim rider leads the drawn hull by a persistent bias an EMA integrates, so
                    // parked is the one state the offset is learnable in, and a frozen offset is the
                    // definition of rigid.
                    bool rigid_done = false;
                    bool vc_hold = false;   // rigid, but vehcamguard is holding the offset
                    if (g_cfg.veh_cam_anchor == 2) {
                        static TrackedObject s_hog;
                        static uintptr_t s_hog_raw = 0;
                        static uint32_t s_hog_gen = 0;   // bumps on every hull change: re-prime the seat
                        const uintptr_t hp = halo::g_hog_body_ptr.load(std::memory_order_relaxed);
                        if (hp != s_hog_raw) {
                            s_hog_raw = hp;
                            ++s_hog_gen;
                            s_hog.set_at(reinterpret_cast<API::UObject*>(hp),
                                         halo::g_hog_body_idx.load(std::memory_order_relaxed));
                        }
                        auto* hog = (hp != 0) ? s_hog.get_checked(L"SkeletalMeshComponent") : nullptr;
                        Vec3 hloc{}, hrot{};
                        if (hog != nullptr && call_ret_vec3(hog, L"K2_GetComponentLocation", &hloc)
                            && call_ret_vec3(hog, L"K2_GetComponentRotation", &hrot)) {
                            // UE rotator convention (FRotationMatrix): rows are the world-space
                            // X/Y/Z axes; the rotator reads (pitch, yaw, roll).
                            const double D2R = 0.01745329252;
                            const double cp2 = std::cos((double)hrot.x * D2R), sp2 = std::sin((double)hrot.x * D2R);
                            const double cy2 = std::cos((double)hrot.y * D2R), sy2 = std::sin((double)hrot.y * D2R);
                            const double cr2 = std::cos((double)hrot.z * D2R), sr2 = std::sin((double)hrot.z * D2R);
                            const double ax[3] = { cp2 * cy2, cp2 * sy2, sp2 };
                            const double ay[3] = { sr2 * sp2 * cy2 - cr2 * sy2, sr2 * sp2 * sy2 + cr2 * cy2, -sr2 * cp2 };
                            const double az[3] = { -(cr2 * sp2 * cy2 + sr2 * sy2), cy2 * sr2 - cr2 * sp2 * sy2, cr2 * cp2 };
                            const double rx = bpx * S - (double)hloc.x;
                            const double ry = -bpy * S - (double)hloc.y;
                            const double rz = bpz * S - (double)hloc.z;
                            const double lof = ax[0] * rx + ax[1] * ry + ax[2] * rz;
                            const double lol = ay[0] * rx + ay[1] * ry + ay[2] * rz;
                            const double lou = az[0] * rx + az[1] * ry + az[2] * rz;
                            static double sof = 0.0, sol = 0.0, sou = 0.0;
                            static bool   sprimed = false;
                            static std::chrono::steady_clock::time_point tprime{};
                            static uint32_t s_gen_seen = 0;
                            if (s_gen_seen != s_hog_gen) { s_gen_seen = s_hog_gen; sprimed = false; }
                            const double oj = (lof - sof) * (lof - sof) + (lol - sol) * (lol - sol)
                                            + (lou - sou) * (lou - sou);
                            // The speed the learn gate sees, and what it decided, for the log below.
                            const float lspeed = halo::g_veh_speed.load(std::memory_order_relaxed);
                            bool learning = false;
                            static uint32_t s_learn_frames = 0, s_snaps = 0, s_refused = 0;

                            // THE DRAWN HULL'S OWN SPEED. g_veh_speed is differentiated from the
                            // RIDER position, so a frozen rider reads 0 and calls a moving hog
                            // parked. The hull transform is live by construction: it is what is drawn.
                            static double s_hpx = 0.0, s_hpy = 0.0, s_hpz = 0.0, s_hspeed = 0.0;
                            static bool   s_hp_ok = false;
                            if (s_hp_ok && fdt > 0.0f) {
                                const double dx = (double)hloc.x - s_hpx, dy = (double)hloc.y - s_hpy,
                                             dz = (double)hloc.z - s_hpz;
                                const double inst = std::sqrt(dx * dx + dy * dy + dz * dz) / (double)fdt;
                                const double kh = (fdt / 0.1 > 1.0) ? 1.0 : (double)fdt / 0.1;
                                s_hspeed += (inst - s_hspeed) * kh;
                            }
                            s_hpx = hloc.x; s_hpy = hloc.y; s_hpz = hloc.z; s_hp_ok = true;
                            // RIDER FRESHNESS: the publish sequence advances ~2600/s while the sim
                            // publish is live; silence means the rider values are a frozen snapshot.
                            static uint32_t s_seq_prev = 0;
                            static std::chrono::steady_clock::time_point s_seq_t = cnow;
                            const uint32_t seq = halo::g_seat_pub_seq.load(std::memory_order_relaxed);
                            if (seq != s_seq_prev) { s_seq_prev = seq; s_seq_t = cnow; }
                            const float stale_ms = std::chrono::duration<float, std::milli>(cnow - s_seq_t).count();
                            // vehcamguard (approach C): never learn or snap on a moving hull or a
                            // stale rider -- hold the offset and ride the hull.
                            const bool rider_stale = stale_ms > (float)g_cfg.veh_cam_stale_ms;
                            const bool hull_moving = s_hspeed > (double)g_cfg.veh_cam_guard_speed;
                            const bool guard_hold = (g_cfg.veh_cam_guard != 0) && (rider_stale || hull_moving);
                            // Never PRIME from a stale rider: that would bolt the camera to the frozen
                            // point. Fall through to the chase anchor until the rider is live.
                            // vehcamhullcheck (approach D): the hull component must move WITH the
                            // Blam vehicle. Driving speed on the Blam side with a motionless
                            // component means this transform is not what is drawn, and a camera
                            // bolted to it stays where it is while the hog drives off. Latched per
                            // hull; a new hull (remount) re-arms it.
                            static uint32_t s_dead_gen = 0xFFFFFFFFu;
                            static bool     s_hull_dead = false;
                            static float    s_still_s = 0.0f;
                            if (s_dead_gen != s_hog_gen) { s_dead_gen = s_hog_gen; s_hull_dead = false; s_still_s = 0.0f; }
                            if (g_cfg.veh_cam_hull_check != 0 && !s_hull_dead) {
                                const bool blam_driving = lspeed > 1.0f && !rider_stale;
                                if (blam_driving && s_hspeed < 30.0) s_still_s += fdt; else s_still_s = 0.0f;
                                if (s_still_s > g_cfg.veh_cam_hull_dead_s) {
                                    s_hull_dead = true;
                                    API::get()->log_info("[Halo-CampE-UEVR] VEHCAMHULL: hull component still (%.0f cm/s) while the Blam vehicle drives (%.2f wu/s) for %.2f s -- "
                                                         "rigid camera disabled for this hull, riding the chase-cam anchor. hull=(%.0f %.0f %.0f)",
                                                         s_hspeed, (double)lspeed, (double)s_still_s,
                                                         (double)hloc.x, (double)hloc.y, (double)hloc.z);
                                }
                            }
                            g_vcd.hull_dead = s_hull_dead;
                            const bool unprimable = (!sprimed && (g_cfg.veh_cam_guard != 0) && rider_stale)
                                                 || (s_hull_dead && g_cfg.veh_cam_hull_check != 0);
                            if (!unprimable) {
                                if (!sprimed || oj > 500.0 * 500.0) {
                                    if (sprimed && guard_hold) {
                                        ++s_refused;
                                    } else {
                                        sof = lof; sol = lol; sou = lou; sprimed = true;
                                        tprime = cnow;
                                        ++s_snaps;
                                    }
                                } else if (lspeed < 0.5f && !guard_hold) {
                                    // LEARN ONLY WHILE PARKED; frozen outright while moving.
                                    learning = true;
                                    ++s_learn_frames;
                                    const float age2 = std::chrono::duration<float>(cnow - tprime).count();
                                    const double otau = (age2 < 2.0f) ? 0.3 : 1.5;
                                    const double ko = (fdt / otau > 1.0) ? 1.0 : (double)fdt / otau;
                                    sof += (lof - sof) * ko; sol += (lol - sol) * ko; sou += (lou - sou) * ko;
                                }
                                lcx = (double)hloc.x + ax[0] * sof + ay[0] * sol + az[0] * sou;
                                lcy = (double)hloc.y + ax[1] * sof + ay[1] * sol + az[1] * sou;
                                lcz = (double)hloc.z + ax[2] * sof + ay[2] * sol + az[2] * sou;
                                rigid_done = true;
                                vc_hold = guard_hold;
                            }
                            g_vcd.have_hull = true;
                            g_vcd.hull[0] = hloc.x; g_vcd.hull[1] = hloc.y; g_vcd.hull[2] = hloc.z;
                            g_vcd.off[0] = sof; g_vcd.off[1] = sol; g_vcd.off[2] = sou;
                            g_vcd.hspeed = s_hspeed; g_vcd.stale_ms = stale_ms;
                            g_vcd.learning = learning;
                            if (learning) ++g_vcd.learn_frames;
                            g_vcd.snaps = s_snaps; g_vcd.snaps_refused = s_refused;
                            if (g_cfg.veh_log && rigid_done) {
                                // HOGLEARN on every gate edge, so a learn that runs while moving is
                                // named with the speed it saw rather than inferred from off= drift.
                                static int s_learn_prev = -1;
                                if ((int)learning != s_learn_prev) {
                                    s_learn_prev = (int)learning;
                                    API::get()->log_info("[Halo-CampE-UEVR] HOGLEARN: %s at speed=%.2f wu/s (gate < 0.50) off=(%.0f %.0f %.0f)",
                                                         learning ? "LEARNING (parked)" : "FROZEN",
                                                         (double)lspeed, sof, sol, sou);
                                }
                                static uint32_t hn = 0;
                                if ((hn++ % 90u) == 0u) {
                                    API::get()->log_info("[Halo-CampE-UEVR] HOGCAM: hog=(%.1f %.1f %.1f) "
                                                         "rot=(%.1f %.1f %.1f) off=(%.0f %.0f %.0f) drift=(%.1f %.1f %.1f) "
                                                         "speed=%.2f learn=%d learnframes=%u snaps=%u",
                                                         (double)hloc.x, (double)hloc.y, (double)hloc.z,
                                                         (double)hrot.x, (double)hrot.y, (double)hrot.z,
                                                         sof, sol, sou, lof - sof, lol - sol, lou - sou,
                                                         (double)lspeed, (int)learning, s_learn_frames, s_snaps);
                                    s_learn_frames = 0;
                                }
                            }
                        }
                        // Unresolved or invalid: fall through to the chase-cam anchor below, so a
                        // vehicle with no hull degrades to the working camera instead of nothing.
                    }
                    if (!rigid_done && g_cfg.veh_cam_anchor != 0) {
                        const double wx = rawcx - bpx * S;            // rider -> cam, world cm
                        const double wy = rawcy + bpy * S;
                        const double wz = rawcz - bpz * S;
                        const double ar = rawgyaw * 0.01745329252;
                        const double ca = std::cos(ar), sa = std::sin(ar);
                        const double bf =  wx * ca + wy * sa;         // into the aim frame
                        const double bl = -wx * sa + wy * ca;
                        static double ef = 0.0, el = 0.0, eu = 0.0;
                        static bool   eb = false;
                        const float tau = (g_cfg.veh_cam_boom_tau > 0.01f) ? g_cfg.veh_cam_boom_tau : 0.5f;
                        // A big jump is a mount, a seat swap or a level load, never a spring: snap.
                        const double bj = (bf - ef) * (bf - ef) + (bl - el) * (bl - el)
                                        + (wz - eu) * (wz - eu);
                        // TWO POLES, NOT ONE (vehboomorder=2). Two cascaded one-poles at tau/2 have
                        // the same total group delay as one at tau but roll off at 40 dB/decade
                        // instead of 20, so the frame-scale noise is rejected harder for the same
                        // lag. Cascaded one-poles on purpose: no complex poles, no overshoot.
                        static double m1f = 0.0, m1l = 0.0, m1u = 0.0;   // first stage
                        if (!eb || bj > 600.0 * 600.0) {
                            ef = bf; el = bl; eu = wz;
                            m1f = bf; m1l = bl; m1u = wz;
                            eb = true;
                        } else if (g_cfg.veh_boom_order >= 2) {
                            const double t2 = (double)tau * 0.5;
                            const double k2 = (fdt / t2 > 1.0) ? 1.0 : (double)(fdt / t2);
                            m1f += (bf - m1f) * k2; m1l += (bl - m1l) * k2; m1u += (wz - m1u) * k2;
                            ef += (m1f - ef) * k2; el += (m1l - el) * k2; eu += (m1u - eu) * k2;
                        } else {
                            const double kb = (fdt / tau > 1.0f) ? 1.0 : (double)(fdt / tau);
                            ef += (bf - ef) * kb; el += (bl - el) * kb; eu += (wz - eu) * kb;
                            m1f = ef; m1l = el; m1u = eu;   // stay primed for a live switch
                        }
                        lcx = rawcx - (ef * ca - el * sa);            // rotate back, then subtract
                        lcy = rawcy - (ef * sa + el * ca);
                        lcz = rawcz - eu;
                    }
                    // Seat offset, in the VEHICLE's frame (x forward, y left, z up) so it stays put
                    // as the hog turns. Applied on every path so vehcamoff means one thing.
                    double ox = g_cfg.veh_cam_off[0], oy = g_cfg.veh_cam_off[1];
                    {
                        const double fxv =  (double)halo::g_veh_fx.load(std::memory_order_relaxed);
                        const double fyv = -(double)halo::g_veh_fy.load(std::memory_order_relaxed);
                        const double n = std::sqrt(fxv * fxv + fyv * fyv);
                        if (halo::g_veh_fvalid.load(std::memory_order_relaxed) && n > 0.5) {
                            const double cf = fxv / n, sf = fyv / n;
                            ox = g_cfg.veh_cam_off[0] * cf - g_cfg.veh_cam_off[1] * sf;
                            oy = g_cfg.veh_cam_off[0] * sf + g_cfg.veh_cam_off[1] * cf;
                        }
                    }
                    fcx = lcx + ox; fcy = lcy + oy; fcz = lcz + g_cfg.veh_cam_off[2];
                    fvalid = true;
                    // WHICH PATH RENDERED THIS FRAME, counted for the VEHSEAT line and named on
                    // every change so a fallback cannot hide inside a once-a-second summary.
                    {
                        const int path = rigid_done ? (vc_hold ? VCP_HOLD : VCP_RIGID)
                                       : (g_cfg.veh_cam_anchor != 0 ? VCP_CHASE : VCP_SYNTH);
                        ++g_vcd.frames[path];
                        if (g_cfg.veh_log && path != g_vcd.last_path) {
                            static const char* const kPath[] = {"none", "rigid", "rigid-hold", "chase", "synth"};
                            API::get()->log_info("[Halo-CampE-UEVR] VEHCAMPATH: %s -> %s (stale=%.0fms hullspeed=%.0fcm/s speed=%.2fwu/s)",
                                                 g_vcd.last_path >= 0 ? kPath[g_vcd.last_path] : "start", kPath[path],
                                                 (double)g_vcd.stale_ms, g_vcd.hspeed,
                                                 (double)halo::g_veh_speed.load(std::memory_order_relaxed));
                        }
                        g_vcd.last_path = path;
                    }
                }
                if (is_double) {
                    auto* p = reinterpret_cast<UEVR_Vector3d*>(position);
                    p->x = fcx; p->y = fcy; p->z = fcz;
                } else {
                    position->x = (float)fcx; position->y = (float)fcy; position->z = (float)fcz;
                }
                // The seated reticule and the compositor read the view position; the wheel zone and
                // markers read g_cam. Both must be the seat, not the chase cam. The lock path below
                // is skipped while seated, so the mirror is done here.
                g_view_pos_x = (float)fcx; g_view_pos_y = (float)fcy; g_view_pos_z = (float)fcz;
                halo::g_cam_x.store((float)fcx, std::memory_order_relaxed);
                halo::g_cam_y.store((float)fcy, std::memory_order_relaxed);
                halo::g_cam_z.store((float)fcz, std::memory_order_relaxed);
                if (index == 0) { g_vcd.fc[0] = fcx; g_vcd.fc[1] = fcy; g_vcd.fc[2] = fcz; g_vcd.wrote = true; }
            } else if (index == 0) {
                g_vcd.wrote = false;
            }
}

bool vehcam_stereo_view_override(UEVR_Rotatorf* rotation, bool is_double) {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    if (rotation == nullptr) return false;
    // Plugin.cpp's view debug values and lock prime, through the bridge.
    auto& g_dbg_view_in  = *host::g_plugin_state.dbg_view_in;
    auto& g_dbg_view_out = *host::g_plugin_state.dbg_view_out;
    auto& g_lock_primed  = *host::g_plugin_state.lock_primed;

        // ---- ROUTE A P0: THIRD-PERSON VIEW ROTATION, published by the eye callback this frame from
        // the selected camera's viewFollows: yaw always, pitch and roll zero unless the camera tilts
        // with the vehicle (a level view, as the first-person path flattens it). Head free-look
        // composes on top in UEVR. Written WHOLE -- never left to the game camera, which pitches with
        // the aim. Runs before the FP seated block, so third-person wins when on.
        if (g_veh_tp_active.load(std::memory_order_relaxed) && halo::g_stick_mode_active.load(std::memory_order_relaxed)
            && halo::g_tp_chassis_yaw_valid.load(std::memory_order_relaxed)) {
            const float cyaw = halo::g_tp_chassis_yaw.load(std::memory_order_relaxed);
            const float cpitch = halo::g_tp_view_pitch.load(std::memory_order_relaxed);
            const float croll = halo::g_tp_view_roll.load(std::memory_order_relaxed);
            if (is_double) {
                auto* r = reinterpret_cast<UEVR_Rotatord*>(rotation);
                r->yaw = (double)cyaw; r->pitch = (double)cpitch; r->roll = (double)croll;
            } else {
                rotation->yaw = cyaw; rotation->pitch = cpitch; rotation->roll = croll;
            }
            g_dbg_view_in = cyaw; g_dbg_view_out = cyaw;
            halo::g_view_base_yaw.store(cyaw, std::memory_order_relaxed);
            halo::g_view_base_pitch.store(cpitch, std::memory_order_relaxed);   // the whole base rotation:
            halo::g_view_base_roll.store(croll, std::memory_order_relaxed);     //   room-anchored quads need it
            g_lock_primed = false;   // re-prime the on-foot lock when you dismount
            return true;
        }

        // ---- IN-VEHICLE VIEW: ANCHOR FORWARD TO THE VEHICLE (vehview).
        //
        // The lock pins the rendered yaw to a room-anchored value, right on foot and exactly wrong
        // seated: the hog turns under a view held at a fixed world yaw. Anchor the base to the
        // vehicle's facing instead and let the headset add free-look. Runs BEFORE the view-lock and
        // stick-mode gates so the seat wins while mounted; unmounted it is a no-op.
        //
        // The branch runs for the WHOLE mount. At the mount edge the eased yaw primes from the game
        // camera and HOLDS until a heading is available. With vehfacing the vehicle object's own
        // orientation (+0x1D4) is the heading; without it the travel direction is used, hemisphere
        // chosen by continuity and healed by a slow pull toward the game camera at driving speed.
        // Eased at render rate (~100 ms), since the heading updates far slower than the frame.
        {
            static bool  s_veh_primed = false;
            static float s_veh_cur = 0.0f;
            static float s_veh_hemi = 0.0f;
            static std::chrono::steady_clock::time_point s_veh_last{};
            // The seated view is the SEAT CAMERA's orientation; without vehcam the stock chase camera is
            // showing, and re-basing and flattening that view is not what vehview is for.
            const bool seated = g_cfg.veh_view != 0 && veh_cam_mode(g_cfg.veh_cam) != 0
                             && (halo::g_unit_mounted.load(std::memory_order_relaxed) || veh_fp_selected());
            if (!seated) {
                s_veh_primed = false;   // next mount re-primes from the fresh game camera
            } else {
                const float game_yaw = is_double
                    ? (float)reinterpret_cast<UEVR_Rotatord*>(rotation)->yaw
                    : rotation->yaw;
                const auto now = std::chrono::steady_clock::now();
                float rdt = s_veh_primed ? std::chrono::duration<float>(now - s_veh_last).count() : 0.0f;
                s_veh_last = now;
                if (rdt < 0.0f || rdt > 0.25f) rdt = 0.0f;
                if (!s_veh_primed) { s_veh_cur = game_yaw; s_veh_hemi = 0.0f; s_veh_primed = true; }

                auto wrap180 = [](float a) {
                    while (a > 180.0f) a -= 360.0f;
                    while (a < -180.0f) a += 360.0f;
                    return a;
                };
                float target = s_veh_cur;   // heading not armed: hold where the mount primed us
                bool  valid  = halo::g_veh_heading_valid.load(std::memory_order_relaxed);
                float travel = halo::g_veh_heading.load(std::memory_order_relaxed);
                if (g_cfg.veh_facing != 0 && halo::g_veh_fvalid.load(std::memory_order_relaxed)) {
                    const float fx = halo::g_veh_fx.load(std::memory_order_relaxed);
                    const float fy = halo::g_veh_fy.load(std::memory_order_relaxed);
                    if (fx * fx + fy * fy > 0.25f) {
                        // Blam frame -> UE: y is negated, as everywhere else in this project.
                        travel = wrap180((float)(std::atan2(-(double)fy, (double)fx) * 57.295779513)
                                         + g_cfg.veh_facing_bias);
                        valid = true;
                        s_veh_hemi = 0.0f;   // no hemisphere guessing needed against a real facing
                    }
                }
                if (valid) {
                    const float cand = wrap180(travel + s_veh_hemi);
                    const float alt  = wrap180(cand + 180.0f);
                    if (std::fabs(wrap180(alt - s_veh_cur)) + 30.0f < std::fabs(wrap180(cand - s_veh_cur))) {
                        s_veh_hemi = (s_veh_hemi == 0.0f) ? 180.0f : 0.0f;
                        target = alt;
                    } else {
                        target = cand;
                    }
                }
                const float k = (rdt / 0.10f > 1.0f) ? 1.0f : rdt / 0.10f;   // ~100 ms to settle
                s_veh_cur = wrap180(s_veh_cur + wrap180(target - s_veh_cur) * k);
                if (valid && halo::g_veh_speed.load(std::memory_order_relaxed) >= 1.0f) {
                    const float kg = (rdt / 2.0f > 1.0f) ? 1.0f : rdt / 2.0f;   // ~2 s hood healer
                    s_veh_cur = wrap180(s_veh_cur + wrap180(game_yaw - s_veh_cur) * kg);
                }

                if (is_double) {
                    auto* r = reinterpret_cast<UEVR_Rotatord*>(rotation);
                    r->yaw = (double)s_veh_cur;
                    if (g_cfg.veh_view_flat) { r->pitch = 0.0; r->roll = 0.0; }
                } else {
                    rotation->yaw = s_veh_cur;
                    if (g_cfg.veh_view_flat) { rotation->pitch = 0.0f; rotation->roll = 0.0f; }
                }
                // One line a second while seated: armed?, raw travel, chosen hemisphere, the eased
                // output, and the untouched game yaw.
                if (g_cfg.veh_log) {
                    static uint32_t n = 0;
                    if ((n++ % 90u) == 0u)
                        API::get()->log_info("[Halo-CampE-UEVR] VEHVIEW: valid=%d travel=%.1f hemi=%.0f eased=%.1f gameyaw=%.1f pos=(%.2f %.2f)",
                                             (int)valid, travel, s_veh_hemi, s_veh_cur, game_yaw,
                                             halo::g_unit_px.load(std::memory_order_relaxed),
                                             halo::g_unit_py.load(std::memory_order_relaxed));
                }
                g_dbg_view_in = s_veh_cur; g_dbg_view_out = s_veh_cur;
                halo::g_view_base_yaw.store(s_veh_cur, std::memory_order_relaxed);
                g_lock_primed = false;   // re-prime the on-foot lock when you dismount
                return true;
            }
        }
    return false;
}

void vehcam_stereo_post_eye_rendered(int index, float ex, float ey, float ez) {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
            // WRITE SURVIVAL (vehlog): the rendered eye against the seat we wrote this frame. The
            // gap is the HMD offset from the standing origin -- under 2 m in any real play space.
            // More than 3 m means the camera was replaced between our write and the render.
            if (index == 0 && g_vcd.wrote && g_cfg.veh_log) {
                const double dx = (double)ex - g_vcd.fc[0], dy = (double)ey - g_vcd.fc[1],
                             dz = (double)ez - g_vcd.fc[2];
                const double dev = std::sqrt(dx * dx + dy * dy + dz * dz);
                ++g_vcd.post_frames;
                if (dev > g_vcd.post_max) g_vcd.post_max = dev;
                if (dev > 300.0) {
                    ++g_vcd.post_bad;
                    static uint32_t s_said = 0;
                    if (s_said++ < 5)
                        API::get()->log_info("[Halo-CampE-UEVR] VEHPOST: rendered eye (%.0f %.0f %.0f) is %.0f cm from the seat we wrote (%.0f %.0f %.0f) -- the camera was replaced after the write",
                                             (double)ex, (double)ey, (double)ez, dev, g_vcd.fc[0], g_vcd.fc[1], g_vcd.fc[2]);
                }
            }
}

}  // namespace

namespace {
bool veh_cam_enabled() { CFG_HOOK_READ; return g_cfg.veh_cam != 0 || g_cfg.vehicle_wheel != 0 || g_cfg.veh_tp; }
}  // namespace

void vehcam_released() {
    // The tick keeps running and restores the driver body and drops the wheel on its own off path;
    // this clears what the render side and the shared view flag still hold.
    g_view_seat_always.store(false, std::memory_order_relaxed);
    vehicle_reset();
    g_vcd.wrote = false;
}

constinit const FeatureHooks kVehCamHooks{
    .key                      = "vehcam",
    .parse_key                = &parse_veh_key,
    .game_tick_vehicle        = &vehcam_game_tick_vehicle,
    .stereo_pre_eye_seat      = &vehcam_stereo_pre_eye_seat,
    .stereo_view_override     = &vehcam_stereo_view_override,
    .stereo_post_eye_rendered = &vehcam_stereo_post_eye_rendered,
    .enabled                    = &veh_cam_enabled,
    .services                   = SVC_UNIT_STATE | SVC_SEAT | SVC_MARKER_ANCHOR,
    .released                   = &vehcam_released,
};

} // namespace halo
