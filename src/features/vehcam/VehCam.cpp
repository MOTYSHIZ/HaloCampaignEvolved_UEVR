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
#include "core/WorldScale.hpp"         // UE cm per real metre at the player's scale, any thread
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
#include <vector>

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

// ---------------------------------------------------------------- THE RIDE SCAN
//
// ONE WALK of the object array per ride, SPREAD OVER TICKS, finding both things a ride needs from it:
// the vehicle's chassis mesh (the cameras' frame, and how the camera file knows the vehicle) and the
// driver's body parts (hidden while a camera sits inside them).
//
// WHY. Boarding froze the game for about a third of a second. The chassis search and the body hider
// each walked all ~290k objects building every object's CLASS NAME as a string, and the hider walked
// twice: ~90-106 ms a walk, measured in the 2026-09-25 log (VEHBODY .176 -> .264 -> .370, STICK MODE
// ENTER .929 -> VEHTP chassis .076). Now one walk serves both, each class is named once (a
// class-pointer cache), each owning actor is looked at once, and the walk spends at most ~1.5 ms of any
// tick, carrying on from where it stopped. The answer arrives a few ticks later and no frame pays for it.
//
// What is kept is a pointer plus its array slot, checked against the array again when used, so an
// object destroyed mid-walk is dropped rather than followed. GAME THREAD.

// A pointer -> byte cache, open-addressed: the walk asks it ~290k times, and a node per entry is what
// makes std::unordered_map slow at that. At most half full; past that it answers without caching.
struct PtrKindCache {
    static constexpr uint32_t N = 1u << 15;
    const void* key[N];
    uint8_t     val[N];
    uint32_t    used = 0;
    void clear() { std::memset(key, 0, sizeof(key)); used = 0; }
    static uint32_t slot(const void* p) {
        uint64_t x = (uint64_t)(uintptr_t)p;
        x ^= x >> 33; x *= 0xff51afd7ed558ccdull; x ^= x >> 33;
        return (uint32_t)x & (N - 1);
    }
    bool find(const void* p, uint8_t* v) const {
        for (uint32_t h = slot(p);; h = (h + 1) & (N - 1)) {   // never full, so an empty slot ends it
            if (key[h] == nullptr) return false;
            if (key[h] == p) { *v = val[h]; return true; }
        }
    }
    void put(const void* p, uint8_t v) {
        if (used * 2 >= N) return;
        uint32_t h = slot(p);
        while (key[h] != nullptr) h = (h + 1) & (N - 1);
        key[h] = p; val[h] = v; ++used;
    }
};

constexpr uint8_t kScanSkm        = 1;    // class is exactly SkeletalMeshComponent (a chassis, a Body)
constexpr uint8_t kScanMesh       = 2;    // class is a *Mesh*Component* (a body part)
constexpr uint8_t kScanVehicle    = 4;    // actor class named *VehicleActor*
constexpr uint8_t kScanSpartan    = 8;    // actor class named *SpartansBipedActor* -- the player's biped
constexpr uint8_t kScanPersistent = 16;   // (an owner) sits in the persistent level
constexpr uint8_t kScanUnit       = 32;   // class is (or derives from) BlamUnitComponent -- a unit's seats

struct ScanHit { API::UObject* o; int32_t i; API::UObject* owner; };

// The Blam unit's UE component, whose GetSeatStates says who sits where (the seat resolve, below).
API::UClass* unit_class() {
    static API::UClass* s_cls = nullptr;
    static bool s_tried = false;
    if (!s_tried) {
        s_tried = true;
        s_cls = API::get()->find_uobject<API::UClass>(L"Class /Script/BlamSynchronization.BlamUnitComponent");
    }
    return s_cls;
}

PtrKindCache s_scan_classes, s_scan_owners;   // per walk: classes and actors can unload between rides
bool     s_scan_active = false;
int32_t  s_scan_next = 0;
uint32_t s_scan_serial = 0;     // bumped when a walk STARTS
uint32_t s_scan_done = 0;       // the serial of the last walk that FINISHED -- the s_res_* below
uint32_t s_scan_gen = 0;        // the stick-mode window: bumped as stick mode engages
uint32_t s_scan_walk_gen = 0, s_scan_result_gen = 0;
int      s_scan_ticks = 0;
double   s_scan_ms = 0.0, s_scan_max_ms = 0.0;
std::vector<ScanHit> s_walk_chassis, s_walk_bodies, s_walk_parts, s_walk_units;   // being filled
std::vector<ScanHit> s_res_chassis, s_res_bodies, s_res_parts, s_res_units;      // the last finished walk
// Meshes one level further down: owned by a COMPONENT of a Spartan (his BlamMeshSynchronization makes the
// visible armour), recorded with the actor as their owner. Only the head hide reads these.
std::vector<ScanHit> s_walk_subparts, s_res_subparts;

uint8_t scan_class_kind(API::UClass* cls) {
    uint8_t k = 0;
    if (s_scan_classes.find(cls, &k)) return k;
    auto* f = cls->get_fname();
    const std::wstring n = (f != nullptr) ? f->to_string() : std::wstring{};
    if (n == L"SkeletalMeshComponent") k |= kScanSkm;
    if (n.find(L"Mesh") != std::wstring::npos && n.find(L"Component") != std::wstring::npos) k |= kScanMesh;
    if (n.find(L"VehicleActor") != std::wstring::npos) k |= kScanVehicle;
    if (n.find(L"SpartansBipedActor") != std::wstring::npos) k |= kScanSpartan;
    // By class pointer up the super chain, not by name: once per class, so the walk never pays for it.
    if (auto* uc = unit_class()) {
        for (API::UStruct* s = cls; s != nullptr; s = s->get_super_struct())
            if (s == uc) { k |= kScanUnit; break; }
    }
    s_scan_classes.put(cls, k);
    return k;
}

uint8_t scan_owner_kind(API::UObject* owner) {
    uint8_t k = 0;
    if (s_scan_owners.find(owner, &k)) return k;
    if (auto* oc = owner->get_class()) k = scan_class_kind(oc) & (kScanVehicle | kScanSpartan);
    if ((k & kScanVehicle) != 0) {
        // In the persistent level, as the old full-name test required: a template's owner is a
        // package, and a streamed level's actors belong to that level.
        auto* lvl = owner->get_outer();
        auto* lf = (lvl != nullptr) ? lvl->get_fname() : nullptr;
        if (lf != nullptr && lf->to_string() == L"PersistentLevel") k |= kScanPersistent;
    }
    s_scan_owners.put(owner, k);
    return k;
}

// Start a walk, or join the one running; returns the serial the caller waits for (ride_scan_done()
// reaching it). force_new: a retry wants a FRESH look; otherwise a walk finished in this stick-mode
// window is answer enough, and the caller has it at once.
uint32_t ride_scan_request(bool force_new) {
    if (s_scan_active) return s_scan_serial;
    if (!force_new && s_scan_done != 0 && s_scan_result_gen == s_scan_gen) return s_scan_done;
    s_scan_active = true;
    s_scan_next = 0;
    ++s_scan_serial;
    s_scan_walk_gen = s_scan_gen;
    s_scan_ticks = 0; s_scan_ms = 0.0; s_scan_max_ms = 0.0;
    s_walk_chassis.clear(); s_walk_bodies.clear(); s_walk_parts.clear(); s_walk_units.clear();
    s_walk_subparts.clear();
    s_scan_classes.clear(); s_scan_owners.clear();
    return s_scan_serial;
}
uint32_t ride_scan_done() { return s_scan_done; }
bool     ride_scan_fresh() { return s_scan_done != 0 && s_scan_result_gen == s_scan_gen; }
void     ride_scan_cancel() { s_scan_active = false; }
void     ride_scan_new_window() { ++s_scan_gen; }

// Once per tick: carry the walk on for at most ~1.5 ms.
void ride_scan_step() {
    if (!s_scan_active) return;
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) { s_scan_active = false; return; }
    static LARGE_INTEGER s_freq{};
    if (s_freq.QuadPart == 0) QueryPerformanceFrequency(&s_freq);
    LARGE_INTEGER t0{}, t1{};
    QueryPerformanceCounter(&t0);
    const LONGLONG budget = s_freq.QuadPart * 15 / 10000;
    const int32_t n = arr->get_object_count();
    int32_t i = s_scan_next;
    for (; i < n; ++i) {
        if (((i - s_scan_next) & 1023) == 1023) {
            QueryPerformanceCounter(&t1);
            if (t1.QuadPart - t0.QuadPart > budget) break;
        }
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        auto* c = o->get_class();
        if (c == nullptr) continue;
        const uint8_t k = scan_class_kind(c);
        if ((k & (kScanMesh | kScanUnit)) == 0) continue;
        auto* owner = o->get_outer();
        if (owner == nullptr) continue;
        const uint8_t ok = scan_owner_kind(owner);
        if ((k & kScanUnit) != 0) {
            // A VEHICLE's Blam unit (a biped's is skipped): asked for its seats by the seat resolve.
            if ((ok & kScanVehicle) != 0 && (ok & kScanPersistent) != 0) s_walk_units.push_back({o, i, owner});
            continue;
        }
        if ((k & kScanSkm) != 0 && (ok & kScanVehicle) != 0 && (ok & kScanPersistent) != 0)
            s_walk_chassis.push_back({o, i, owner});
        if ((ok & kScanSpartan) != 0) {
            s_walk_parts.push_back({o, i, owner});
            if ((k & kScanSkm) != 0) {
                auto* f = o->get_fname();
                if (f != nullptr && f->to_string() == L"Body") s_walk_bodies.push_back({o, i, owner});
            }
        } else if (auto* oo = owner->get_outer(); oo != nullptr && (scan_owner_kind(oo) & kScanSpartan) != 0) {
            s_walk_subparts.push_back({o, i, oo});   // made by one of his components: the actor is its owner
        }
    }
    QueryPerformanceCounter(&t1);
    const double ms = (double)(t1.QuadPart - t0.QuadPart) * 1000.0 / (double)s_freq.QuadPart;
    s_scan_ms += ms;
    if (ms > s_scan_max_ms) s_scan_max_ms = ms;
    ++s_scan_ticks;
    s_scan_next = i;
    if (i < n) return;
    s_scan_active = false;
    s_res_chassis.swap(s_walk_chassis);
    s_res_bodies.swap(s_walk_bodies);
    s_res_parts.swap(s_walk_parts);
    s_res_units.swap(s_walk_units);
    s_res_subparts.swap(s_walk_subparts);
    s_scan_done = s_scan_serial;
    s_scan_result_gen = s_scan_walk_gen;
    API::get()->log_info("[Halo-CampE-UEVR] VEHSCAN: %d objects in %d tick(s), %.1f ms of work (%.1f ms at most in one "
                         "tick): %d vehicle meshes, %d Spartan bodies, %d vehicle units",
                         n, s_scan_ticks, s_scan_ms, s_scan_max_ms, (int)s_res_chassis.size(), (int)s_res_bodies.size(),
                         (int)s_res_units.size());
}

// Is this hit still the object it was? Its slot must still hold it.
bool scan_hit_live(const ScanHit& h) {
    auto* arr = API::get()->get_uobject_array();
    return arr != nullptr && h.i >= 0 && h.i < arr->get_object_count() && arr->get_object(h.i) == h.o;
}

// ---------------------------------------------------------------- DRIVER BODY HIDE
//
// WHAT IT IS, measured 2026-08-21 by a ranked sweep around the rendered eye:
//
//   BP_SpartansBipedActor_C_<id>.Body     SkeletalMeshComponent, SK_Spartans_AnimDynamics
//
// That actor is the player. With the camera at the seat your head is inside it, so it clips
// constantly. MATCHED BY OWNER, NOT BY CLASS: the class is plain "SkeletalMeshComponent", shared
// with every marine, weapon and NPC in the level; the owning actor's class (SpartansBipedActor) and
// the component's name (Body) are what identify it. The ride scan collects them.

// EVERY mesh component on the driver actor, not just .Body. These are modular characters (the
// marines nearby are SIX components each), so the Spartan is one too and .Body is one piece.
constexpr int kMaxDriverParts = 24;
TrackedObject s_driver_parts[kMaxDriverParts];
int           s_driver_part_count = 0;
bool          s_driver_hidden = false;
int           s_hide_applied = -1;   // the part count the hide was last applied to; -1 = apply at once
int           s_driver_tries = 0; ULONGLONG s_driver_try_at = 0;
uint32_t      s_driver_wait = 0;   // the ride scan serial the hider waits for; 0 = not waiting

// The driver's parts from the last finished ride scan. SELECTS ON d_blam: the Blam unit position is
// the player's biped by definition, while the eye is only meaningful after the vehicle camera has
// run -- and at the mount edge it has not. (Selecting on the eye once picked a Spartan 847 cm away.)
int ride_scan_take_driver_parts() {
    s_driver_part_count = 0;
    const double ex = (double)g_cam_x.load(std::memory_order_relaxed);
    const double ey = (double)g_cam_y.load(std::memory_order_relaxed);
    const double ez = (double)g_cam_z.load(std::memory_order_relaxed);
    const double S = 304.8;
    double bx =  (double)g_unit_px.load(std::memory_order_relaxed) * S;
    double by = -(double)g_unit_py.load(std::memory_order_relaxed) * S;
    double bz =  (double)g_unit_pz.load(std::memory_order_relaxed) * S;
    // No Blam rider position (not published yet -- the seat publish runs only while a vehicle-camera
    // feature enables it -- or a build it cannot read): the local pawn is the same body, and its
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
    API::UObject* owner = nullptr; double bestd = 1e18;
    int spartans = 0;
    for (const ScanHit& h : s_res_bodies) {
        if (!scan_hit_live(h)) continue;
        Vec3 w{};
        if (!call_ret_vec3(h.o, L"K2_GetComponentLocation", &w)) continue;
        const double de = std::sqrt(((double)w.x - ex) * ((double)w.x - ex)
                                  + ((double)w.y - ey) * ((double)w.y - ey)
                                  + ((double)w.z - ez) * ((double)w.z - ez));
        const double db = std::sqrt(((double)w.x - bx) * ((double)w.x - bx)
                                  + ((double)w.y - by) * ((double)w.y - by)
                                  + ((double)w.z - bz) * ((double)w.z - bz));
        ++spartans;
        API::get()->log_info("[Halo-CampE-UEVR] VEHBODY   spartan d_eye=%8.1f d_blam=%8.1f  %ls",
                             de, db, h.o->get_full_name().c_str());
        if (db < bestd) { bestd = db; owner = h.owner; }
    }
    API::get()->log_info("[Halo-CampE-UEVR] VEHBODY: %d spartan bodies in the level", spartans);
    if (owner == nullptr) return 0;
    for (const ScanHit& h : s_res_parts) {
        if (s_driver_part_count >= kMaxDriverParts) break;
        if (h.owner != owner || !scan_hit_live(h)) continue;
        s_driver_parts[s_driver_part_count++].set_at(h.o, h.i);
    }
    API::get()->log_info("[Halo-CampE-UEVR] VEHBODY: chose %ls at d_blam=%.1f -- %d mesh parts",
                         owner->get_full_name().c_str(), bestd, s_driver_part_count);
    return s_driver_part_count;
}

// The driver's Body from the last finished ride scan, on the same rule (nearest the Blam rider position,
// else the pawn) and without the census log -- for the playerhead origin. nullptr = none.
const ScanHit* ride_scan_pick_driver_body() {
    const double S = 304.8;
    double bx =  (double)g_unit_px.load(std::memory_order_relaxed) * S;
    double by = -(double)g_unit_py.load(std::memory_order_relaxed) * S;
    double bz =  (double)g_unit_pz.load(std::memory_order_relaxed) * S;
    if (!g_unit_pvalid.load(std::memory_order_relaxed)) {
        Vec3 pl{};
        if (auto* pawn = API::get()->get_local_pawn(0); pawn != nullptr && call_ret_vec3(pawn, L"K2_GetActorLocation", &pl)) {
            bx = pl.x; by = pl.y; bz = pl.z;
        }
    }
    const ScanHit* best = nullptr; double bestd = 1e18;
    for (const ScanHit& h : s_res_bodies) {
        if (!scan_hit_live(h)) continue;
        Vec3 w{};
        if (!call_ret_vec3(h.o, L"K2_GetComponentLocation", &w)) continue;
        const double d = std::sqrt(((double)w.x - bx) * ((double)w.x - bx) + ((double)w.y - by) * ((double)w.y - by)
                                 + ((double)w.z - bz) * ((double)w.z - bz));
        if (d < bestd) { bestd = d; best = &h; }
    }
    return best;
}

// The Chief's HEAD BONE, by name: his skeleton's bone names are not written down anywhere here, so they
// are listed once and the best match taken -- exactly "b_head", then "head", then a name ending in "head",
// then one containing it (not an "end"/"nub"/"tip" helper). GetNumBones / GetBoneName are the calls
// Arms.cpp already relies on (int32 in at 0, FName out at 4). Empty = none, and the playerhead origin
// then falls back to the seat. GAME THREAD.
std::wstring find_head_bone(API::UObject* body, int32_t* nbones) {
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    body->call_function(L"GetNumBones", p);
    const int32_t nb = *reinterpret_cast<int32_t*>(p);
    if (nbones != nullptr) *nbones = nb;
    if (nb <= 0 || nb > 2048) return L"";
    std::wstring best;
    int best_rank = 99;
    for (int32_t b = 0; b < nb; ++b) {
        std::memset(p, 0, 16);
        *reinterpret_cast<int32_t*>(p) = b;
        body->call_function(L"GetBoneName", p);
        const std::wstring n = reinterpret_cast<API::FName*>(p + 4)->to_string();
        std::wstring l = n;
        for (auto& ch : l) ch = (wchar_t)towlower(ch);
        const size_t k = l.find(L"head");
        if (k == std::wstring::npos) continue;
        int rank = 3;
        if (l == L"b_head") rank = 0;
        else if (l == L"head") rank = 1;
        else if (k + 4 == l.size()) rank = 2;
        else if (l.find(L"end") != std::wstring::npos || l.find(L"nub") != std::wstring::npos
                 || l.find(L"tip") != std::wstring::npos) continue;
        if (rank < best_rank) { best_rank = rank; best = n; }
    }
    return best;
}

// ---------------------------------------------------------------- THE SEAT, AS THE GAME REPORTS IT
//
// WHICH VEHICLE YOU ARE IN, AND IN WHICH SEAT -- asked of the game, not guessed. Every Blam unit's UE
// component (BlamUnitComponent) answers GetSeatStates(): one FBlamUnitSeatState per seat, naming its
// occupant (SeatedUnitActor) and its role (bIsDriver, bIsGunner, bOccupied). Every offset comes from the
// game's own reflection, by field NAME, so this resolves on any build that keeps those names and fails
// closed -- the nearest mesh decides, as before -- on one that does not. Nothing is measured on one build.
//
// WHY. The nearest vehicle mesh is not always your vehicle's. In the Wraith's driver seat the
// anti-infantry turret's mesh is nearer than the Wraith's own, so the driver was handed the turret's
// cameras (the user, 2026-09-26). And a passenger rides the same actor as its driver, which no distance
// tells apart. The game's own Blueprint asks exactly this (BP_Audio_VehiclePlayerRoleProvider.GetDriverSeatState).
//
// The layout as this build's reflection reports it (the first live bind, 2026-09-26): an 80-byte seat --
// SeatWorldPosition +0, EntryRadius +24, the flags bIsInvisible / bIsLocked / bIsDriver / bIsGunner /
// bSeatAllowsWeapons / bIsBoardingSeat / bNotForPlayer as bits of +28, SeatedUnitDatumIndex +32, and
// SeatedUnitActor +40 as a SOFT pointer. There is no bOccupied (that name belongs to an event's
// parameter), so a seat is occupied when it names an occupant. The first cut accepted only plain and weak
// pointers, refused the soft one, and so left every seat unknown for a session. GAME THREAD.

std::wstring ffield_name(API::FField* f) {
    auto* n = (f != nullptr) ? f->get_fname() : nullptr;
    return (n != nullptr) ? n->to_string() : std::wstring{};
}
std::wstring ffield_type(API::FField* f) {
    auto* c = (f != nullptr) ? f->get_class() : nullptr;
    auto* n = (c != nullptr) ? c->get_fname() : nullptr;
    return (n != nullptr) ? n->to_string() : std::wstring{};
}

struct SeatRefl {
    int state = -1;                            // -1 not tried, 0 unusable (fail closed), 1 ready
    API::UFunction* fn = nullptr;
    int32_t psize = 0;                         // the function's parameter block, the engine's own size
    int32_t arr_off = -1;                      // the TArray<FBlamUnitSeatState> in it (return or out)
    int32_t elem = 0;                          // sizeof(FBlamUnitSeatState)
    // SeatedUnitActor: 1 = object pointer, 2 = weak pointer, 3 = SOFT pointer (TSoftObjectPtr -- what this
    // build has: measured 2026-09-26, +40 of an 80-byte seat). A soft pointer is a weak pointer followed by
    // the object's path (FSoftObjectPath), so its first 8 bytes resolve like a weak pointer; the path's
    // SubPathString ("PersistentLevel.<actor>") is the fallback witness, and it is heap memory to free.
    int32_t actor_off = -1; int actor_kind = 0;
    int32_t soft_str_off = -1;                 // the SubPathString within a seat; -1 = layout not proven
    bool    soft_str_utf8 = false;             // FUtf8String rather than FString
    int32_t occ_off = -1, drv_off = -1, gun_off = -1;       // bOccupied / bIsDriver / bIsGunner: the byte...
    uint8_t occ_mask = 0, drv_mask = 0, gun_mask = 0;       // ...and its bit
};
SeatRefl s_sr;

bool seat_refl_ready() {
    if (s_sr.state >= 0) return s_sr.state == 1;
    s_sr.state = 0;
    auto* uc = unit_class();
    s_sr.fn = (uc != nullptr) ? uc->find_function(L"GetSeatStates") : nullptr;
    if (s_sr.fn == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] VEHSEAT: BlamUnitComponent.GetSeatStates not found (class %s) -- "
                             "vehicles are told by the nearest mesh", uc != nullptr ? "found" : "NOT FOUND");
        return false;
    }
    s_sr.psize = s_sr.fn->get_properties_size();
    API::UScriptStruct* st = nullptr;
    for (auto* f = s_sr.fn->get_child_properties(); f != nullptr; f = f->get_next()) {
        if (ffield_type(f) != L"ArrayProperty") continue;
        auto* inner = reinterpret_cast<API::FArrayProperty*>(f)->get_inner();
        if (inner == nullptr || ffield_type(inner) != L"StructProperty") continue;
        st = reinterpret_cast<API::FStructProperty*>(inner)->get_struct();
        s_sr.arr_off = reinterpret_cast<API::FProperty*>(f)->get_offset();
        break;
    }
    if (st != nullptr) {
        s_sr.elem = st->get_struct_size();
        for (auto* f = st->get_child_properties(); f != nullptr; f = f->get_next()) {
            const std::wstring n = ffield_name(f), ty = ffield_type(f);
            auto* p = reinterpret_cast<API::FProperty*>(f);
            if (n == L"SeatedUnitActor") {
                s_sr.actor_off = p->get_offset();
                s_sr.actor_kind = (ty == L"ObjectProperty" || ty == L"ObjectPtrProperty") ? 1
                                : (ty == L"WeakObjectProperty") ? 2
                                : (ty == L"SoftObjectProperty") ? 3 : 0;
            } else if (ty == L"BoolProperty" && (n == L"bOccupied" || n == L"bIsDriver" || n == L"bIsGunner")) {
                auto* b = reinterpret_cast<API::FBoolProperty*>(f);
                const int32_t off = p->get_offset() + static_cast<int32_t>(b->get_byte_offset());
                const uint8_t mask = static_cast<uint8_t>(b->get_byte_mask());
                if (n == L"bOccupied")      { s_sr.occ_off = off; s_sr.occ_mask = mask; }
                else if (n == L"bIsDriver") { s_sr.drv_off = off; s_sr.drv_mask = mask; }
                else                        { s_sr.gun_off = off; s_sr.gun_mask = mask; }
            }
#if HALO_VR_DEV
            API::get()->log_info("[Halo-CampE-UEVR] VEHSEAT   field %-24ls %-20ls +%d", n.c_str(), ty.c_str(), p->get_offset());
#endif
        }
    }
    // A SOFT pointer's path, by reflection too: FSoftObjectPath is a reflected struct, so its size and the
    // SubPathString's offset are the engine's, not ours. The layout counts as proven only when the weak
    // half (8 bytes) plus that struct exactly fill the room the seat leaves for it -- then the string is
    // freed after each read; otherwise it is never touched (a few bytes leak per read rather than a free
    // of the wrong pointer).
    if (s_sr.actor_kind == 3 && s_sr.actor_off >= 0) {
        auto* sop = API::get()->find_uobject<API::UScriptStruct>(L"ScriptStruct /Script/CoreUObject.SoftObjectPath");
        if (sop != nullptr) {
            const int32_t sop_size = sop->get_struct_size();
            for (auto* f = sop->get_child_properties(); f != nullptr; f = f->get_next()) {
                if (ffield_name(f) != L"SubPathString") continue;
                const std::wstring ty = ffield_type(f);
                if (ty != L"StrProperty" && ty != L"Utf8StrProperty") break;
                const int32_t off = s_sr.actor_off + 8 + reinterpret_cast<API::FProperty*>(f)->get_offset();
                // The soft pointer is the seat's last field in this build; wherever it sits, its path
                // must end inside the seat, and the next field (if any) cannot start inside it.
                if (sop_size > 0 && s_sr.actor_off + 8 + sop_size <= s_sr.elem && off + 16 <= s_sr.elem) {
                    s_sr.soft_str_off = off;
                    s_sr.soft_str_utf8 = (ty == L"Utf8StrProperty");
                }
                break;
            }
        }
    }
    const bool ok = st != nullptr && s_sr.psize > 0 && s_sr.arr_off >= 0 && s_sr.arr_off + 16 <= s_sr.psize
                 && s_sr.elem > 0 && s_sr.actor_kind != 0 && s_sr.actor_off >= 0 && s_sr.actor_off + 8 <= s_sr.elem;
    API::get()->log_info("[Halo-CampE-UEVR] VEHSEAT: GetSeatStates %s -- params %d, array +%d, seat %d bytes, "
                         "occupant +%d (%s%s), occupied +%d, driver +%d, gunner +%d",
                         ok ? "ready" : "UNUSABLE, vehicles are told by the nearest mesh",
                         s_sr.psize, s_sr.arr_off, s_sr.elem, s_sr.actor_off,
                         s_sr.actor_kind == 1 ? "pointer" : (s_sr.actor_kind == 2 ? "weak"
                                                           : (s_sr.actor_kind == 3 ? "soft" : "?")),
                         s_sr.actor_kind != 3 ? "" : (s_sr.soft_str_off >= 0 ? ", path proven" : ", path NOT proven: never freed"),
                         s_sr.occ_off, s_sr.drv_off, s_sr.gun_off);
    s_sr.state = ok ? 1 : 0;
    return ok;
}

struct SeatRow {
    API::UObject* occupant;   // resolved from the pointer (or a soft/weak pointer's weak half); only compared
    std::wstring  path;       // a soft pointer's SubPathString ("PersistentLevel.<actor>"); the fallback witness
    bool occupied, driver, gunner;
};

// One unit's seats, into rows (at most max). Returns the seat count, or -1 when the call gave nothing
// usable. The returned array belongs to us once the native thunk has built it into our frame, so it is
// freed here through the engine's own allocator, and with it the only heap memory a seat holds: a soft
// pointer's path string, freed only when its layout is proven (seat_refl_ready).
int read_seats(API::UObject* unit, SeatRow* rows, int max) {
    static std::vector<uint8_t> buf;
    buf.assign(static_cast<size_t>(s_sr.psize), 0);
    unit->process_event(s_sr.fn, buf.data());
    struct FRawArray { uint8_t* data; int32_t num; int32_t max; };
    struct FRawString { void* data; int32_t num; int32_t max; };
    auto* arr = reinterpret_cast<FRawArray*>(buf.data() + s_sr.arr_off);
    int n = -1;
    const bool readable = arr->data != nullptr && arr->num > 0 && arr->num <= 64 && arr->max >= arr->num
                       && !IsBadReadPtr(arr->data, static_cast<size_t>(arr->num) * static_cast<size_t>(s_sr.elem));
    if (arr->data == nullptr && arr->num == 0) {
        n = 0;
    } else if (readable) {
        n = arr->num;
        auto* objs = API::get()->get_uobject_array();
        for (int i = 0; i < n && i < max; ++i) {
            const uint8_t* e = arr->data + static_cast<size_t>(i) * static_cast<size_t>(s_sr.elem);
            SeatRow& r = rows[i];
            r.occupant = nullptr;
            r.path.clear();
            if (s_sr.actor_kind == 1) {
                r.occupant = *reinterpret_cast<API::UObject* const*>(e + s_sr.actor_off);
            } else {
                // TWeakObjectPtr { int32 ObjectIndex; int32 SerialNumber } -- a soft pointer starts with one.
                // Resolved through the object array, only ever compared, never followed.
                const int32_t idx = *reinterpret_cast<const int32_t*>(e + s_sr.actor_off);
                if (objs != nullptr && idx > 0 && idx < objs->get_object_count())
                    r.occupant = reinterpret_cast<API::UObject*>(objs->get_object(idx));
            }
            if (s_sr.actor_kind == 3 && s_sr.soft_str_off >= 0) {
                const auto* s = reinterpret_cast<const FRawString*>(e + s_sr.soft_str_off);
                if (s->data != nullptr && s->num > 1 && s->num < 1024) {
                    if (s_sr.soft_str_utf8) {
                        if (!IsBadReadPtr(s->data, static_cast<size_t>(s->num))) {
                            const char* c = static_cast<const char*>(s->data);
                            r.path.assign(c, c + (s->num - 1));
                        }
                    } else if (!IsBadReadPtr(s->data, static_cast<size_t>(s->num) * sizeof(wchar_t))) {
                        r.path.assign(static_cast<const wchar_t*>(s->data), static_cast<size_t>(s->num - 1));
                    }
                }
            }
            auto bit = [&](int32_t off, uint8_t mask) { return off >= 0 && off < s_sr.elem && (e[off] & mask) != 0; };
            r.occupied = (s_sr.occ_off >= 0) ? bit(s_sr.occ_off, s_sr.occ_mask)
                                             : (r.occupant != nullptr || !r.path.empty());
            r.driver = bit(s_sr.drv_off, s_sr.drv_mask);
            r.gunner = bit(s_sr.gun_off, s_sr.gun_mask);
        }
    }
    auto* m = API::FMalloc::get();
    if (m != nullptr && readable && s_sr.actor_kind == 3 && s_sr.soft_str_off >= 0) {
        for (int i = 0; i < arr->num; ++i) {   // every seat's path, not only the ones read
            auto* s = reinterpret_cast<FRawString*>(arr->data + static_cast<size_t>(i) * static_cast<size_t>(s_sr.elem)
                                                    + s_sr.soft_str_off);
            if (s->data != nullptr) m->free(s->data);
            s->data = nullptr; s->num = 0; s->max = 0;
        }
    }
    if (arr->data != nullptr && m != nullptr) m->free(arr->data);
    arr->data = nullptr; arr->num = 0; arr->max = 0;
    return n;
}

// Does a soft pointer's path name this actor? "PersistentLevel.BP_SpartansBipedActor_C_<id>" ends with the
// actor's own name, after a dot.
bool path_names(const std::wstring& path, API::UObject* actor) {
    if (path.empty() || actor == nullptr) return false;
    auto* f = actor->get_fname();
    const std::wstring n = (f != nullptr) ? f->to_string() : std::wstring{};
    if (n.empty() || path.size() < n.size()) return false;
    if (path.compare(path.size() - n.size(), n.size(), n) != 0) return false;
    return path.size() == n.size() || path[path.size() - n.size() - 1] == L'.';
}

// The last answer. The vehicle actor holding your seat, its unit, the seat, and its flags as the bits an
// entry's "seat" is tested against (vcp::seat_bits).
struct SeatFix {
    bool valid = false;
    API::UObject* actor = nullptr;
    API::UObject* unit = nullptr;
    int32_t unit_idx = -1;
    int seat = -1, nseats = 0;
    uint8_t bits = 0;
    bool by_path = false;   // matched on the soft pointer's path rather than its weak half
};
SeatFix s_seat;
// Once the game's seats have named your seat this session, they are the authority: a ride they do not
// back gets no vehicle camera (the Blam mount flag alone is not trusted to start one). Until then -- and
// on a build where the seat query cannot bind -- the nearest mesh stands in, as it always did.
bool s_seats_proven = false;
// The OTHER vehicles near you that have seats of their own -- never your chassis, when yours has no mesh.
API::UObject* s_seat_others[8];
int s_seat_other_n = 0;

// Ask the vehicles within 15 m, nearest first, which one lists YOU in a seat. me = the actors that are
// you (the pawn, and the Spartan biped the body hider picks). Nearest wins if more than one does.
bool seat_resolve(API::UObject* const* me, int nme, const double p[3], SeatFix* out) {
#if HALO_VR_DEV
    const SeatFix prev = s_seat;   // for the seat table's log-on-change; taken before `out` (maybe s_seat) is cleared
#endif
    *out = SeatFix{};
    s_seat_other_n = 0;
    if (!seat_refl_ready()) return false;
    constexpr int kMax = 12;
    const ScanHit* cand[kMax]; double cd[kMax]; int nc = 0;
    for (const ScanHit& h : s_res_units) {
        if (h.owner == nullptr || !scan_hit_live(h)) continue;
        Vec3 w{};
        if (!call_ret_vec3(h.owner, L"K2_GetActorLocation", &w)) continue;
        const double d = std::sqrt(((double)w.x - p[0]) * ((double)w.x - p[0]) + ((double)w.y - p[1]) * ((double)w.y - p[1])
                                 + ((double)w.z - p[2]) * ((double)w.z - p[2]));
        if (d > 1500.0) continue;
        if (nc == kMax && d >= cd[kMax - 1]) continue;       // full, and farther than the farthest kept
        int at = (nc < kMax) ? nc++ : kMax - 1;               // append, or replace the farthest
        while (at > 0 && cd[at - 1] > d) { cand[at] = cand[at - 1]; cd[at] = cd[at - 1]; --at; }
        cand[at] = &h; cd[at] = d;
    }
    SeatRow rows[16];
    int found = 0;
    for (int k = 0; k < nc; ++k) {
        const int n = read_seats(cand[k]->o, rows, 16);
        if (n <= 0) continue;
        int mine = -1;
        bool via_path = false;
        for (int i = 0; i < n && i < 16 && mine < 0; ++i)
            for (int m = 0; m < nme; ++m) {
                if (me[m] == nullptr) continue;
                if (rows[i].occupant == me[m]) { mine = i; break; }
                if (path_names(rows[i].path, me[m])) { mine = i; via_path = true; break; }
            }
        if (mine < 0) {
            if (s_seat_other_n < 8) s_seat_others[s_seat_other_n++] = cand[k]->owner;
            continue;
        }
        if (++found > 1) {
            API::get()->log_info("[Halo-CampE-UEVR] VEHSEAT: %ls also lists you (seat %d) -- the nearer vehicle is kept",
                                 cand[k]->owner->get_full_name().c_str(), mine);
            continue;
        }
        out->valid = true;
        out->actor = cand[k]->owner; out->unit = cand[k]->o; out->unit_idx = cand[k]->i;
        out->seat = mine; out->nseats = n;
        out->bits = vehcampresets::seat_bits(rows[mine].driver, rows[mine].gunner);
        out->by_path = via_path;
        s_seats_proven = true;
#if HALO_VR_DEV
        // The seat table, when the answer is new: the ~2 s re-check asks again all ride long, and a table
        // repeated every time it agrees says nothing (it filled the log at four lines every 2 s).
        if (!prev.valid || prev.actor != out->actor || prev.seat != out->seat || prev.bits != out->bits)
            for (int i = 0; i < n && i < 16; ++i)
                API::get()->log_info("[Halo-CampE-UEVR] VEHSEAT   seat %d: %s%s%s occupant=%ls path=%ls%s", i,
                                     rows[i].driver ? "driver " : "", rows[i].gunner ? "gunner " : "",
                                     rows[i].occupied ? "occupied" : "empty",
                                     rows[i].occupant != nullptr ? rows[i].occupant->get_full_name().c_str() : L"none",
                                     rows[i].path.empty() ? L"-" : rows[i].path.c_str(),
                                     i == mine ? "  <-- YOU" : "");
#endif
    }
    return out->valid;
}

// ---------------------------------------------------------------- THE PLAYER'S HEAD, HIDDEN ("hideHead")
//
// True first person from a camera at the Chief's head -- a turret's head camera -- where his own helmet
// otherwise fills the view. The camera file's "hideHead", per seat or per camera; off by default, since the
// Chief in his seat is part of the picture everywhere else (the user, 2026-09-26).
//
// UE's own USkinnedMeshComponent.HideBoneByName on his head bone: that bone and every bone below it draw at
// zero scale. Only the SKINNING changes -- his bones' positions do not, so the playerhead origin still reads
// the head -- and PBO_None leaves his physics alone. Called on EVERY mesh part of his biped: the part that
// owns the pose takes it, and parts that follow its pose (a modular character's armour pieces) take it from
// there -- the engine ignores the call on those, as on a part without that bone. The parameter blocks come
// from the functions' own reflection, by name, so a build that reshapes them is refused, not written into.
// UNDONE on the way out -- a camera without it, the seat left, a cutscene: the Chief appears in cutscenes.
//
// THE HELMET IS NOT IN THE SKELETON. The first in-headset test (2026-09-27): the bone hide landed on the
// Body ("readback says hidden on 1 of 1") and the head still drew. The Spartan is modular -- the pak lists
// SK_Spartans_<armour> plus STATIC meshes per piece (SM_Spartans_Helmet_M_<armour>, ..._Chest_..., ...),
// hung from its skeleton -- and a hidden bone scales only the skinning, never what hangs from its socket.
// The whole-body hide had removed the helmet with the rest (the user) -- by shrinking his
// BlamMeshSynchronization component, which the armour hangs under (the second test: his actor owns only
// Collider, Body and that component). So the helmet pieces are found in his attachment tree (rider_tree) by
// the mesh they draw, the socket they hang from or their name, and SHRUNK (head_item_hide says why not
// hidden too); each keeps its own scale for the restore. The
// bone hide stays, on every skinned mesh of his, for whatever of the head the skeleton draws under it.
constexpr int kMaxHeadItems = 8;
struct HeadHide {
    int state = -1;                              // -1 not tried; 0 unusable (fail closed); 1 ready
    API::UClass* skinned = nullptr;              // USkinnedMeshComponent: the parts the call is for
    int32_t hide_name = -1, hide_op = -1, unhide_name = -1, is_name = -1, is_ret = -1;
    TrackedObject parts[kMaxDriverParts];
    bool part_was_hidden[kMaxDriverParts] = {};  // the head bone was hidden before we came: the game's, left so
    int n = 0;
    TrackedObject items[kMaxHeadItems];          // the helmet pieces, shrunk
    double item_scale[kMaxHeadItems][3] = {};    // ...and the relative scale each had
    int ni = 0;
    uintptr_t body = 0;                          // the Body the parts were collected for
    std::wstring bone;
    bool on = false;
    uint32_t reassert = 0;
};
HeadHide s_hh;

std::wstring lower_w(std::wstring s) {
    for (auto& ch : s) ch = (wchar_t)towlower(ch);
    return s;
}

// A helmet piece: by the static mesh it draws (SM_Spartans_Helmet_M_<armour>), the socket or bone it hangs
// from (the head bone, or one named for the helmet), or its own name. `why` says which. GAME THREAD.
bool is_head_item(API::UObject* c, const std::wstring& head_bone, std::wstring* why) {
    if (auto* pm = c->get_property_data<API::UObject*>(L"StaticMesh"); pm != nullptr && !IsBadReadPtr(pm, sizeof(void*))) {
        if (auto* mesh = *pm) {
            auto* fn = mesh->get_fname();
            const std::wstring m = (fn != nullptr) ? fn->to_string() : std::wstring{};
            const std::wstring lm = lower_w(m);
            if (lm.find(L"helmet") != std::wstring::npos || lm.find(L"visor") != std::wstring::npos) {
                *why = L"mesh " + m;
                return true;
            }
        }
    }
    if (auto* ps = c->get_property_data<API::FName>(L"AttachSocketName"); ps != nullptr && !IsBadReadPtr(ps, sizeof(API::FName))) {
        const std::wstring s = ps->to_string();
        const std::wstring ls = lower_w(s);
        if ((!head_bone.empty() && ls == lower_w(head_bone)) || ls.find(L"helmet") != std::wstring::npos
            || ls.find(L"visor") != std::wstring::npos) {
            *why = L"socket " + s;
            return true;
        }
    }
    auto* f = c->get_fname();
    const std::wstring n = (f != nullptr) ? f->to_string() : std::wstring{};
    const std::wstring ln = lower_w(n);
    if (ln.find(L"helmet") != std::wstring::npos || ln.find(L"visor") != std::wstring::npos) {
        *why = L"name " + n;
        return true;
    }
    return false;
}

// HIS WHOLE MESH TREE. The biped actor itself owns three parts -- Collider, Body and a
// BlamMeshSynchronizationComponent (measured 2026-09-27, the head-hide scan) -- and the visible armour, the
// helmet among it, is made BY that synchronization component, owned through it rather than the actor. So
// the ride scan's owner test never sees a helmet. They all hang in the actor's attachment tree, though, so
// the tree is walked from those parts and the actor's root, down every AttachChildren, breadth first.
// Things hung on him that are not his (his weapon's own actor) come along too; only a mesh named for the
// helmet or hung from the head is ever hidden, so they are left as they are. GAME THREAD; once per apply.
struct RawObjArray { API::UObject** data; int32_t num; int32_t max; };
int rider_tree(API::UObject* actor, API::UObject** out, int max) {
    int n = 0;
    auto push = [&](API::UObject* c) {
        if (c == nullptr || n >= max || !uobject_slot_valid(c)) return;
        for (int i = 0; i < n; ++i) if (out[i] == c) return;
        out[n++] = c;
    };
    for (const ScanHit& h : s_res_parts)
        if (h.owner == actor && scan_hit_live(h)) push(h.o);
    for (const ScanHit& h : s_res_subparts)   // made by his components, whether attached in the tree or not
        if (h.owner == actor && scan_hit_live(h)) push(h.o);
    if (auto* rp = actor->get_property_data<API::UObject*>(L"RootComponent"); rp != nullptr && !IsBadReadPtr(rp, sizeof(void*)))
        push(*rp);
    for (int i = 0; i < n; ++i) {   // n grows as children are found
        auto* kids = out[i]->get_property_data<RawObjArray>(L"AttachChildren");
        if (kids == nullptr || IsBadReadPtr(kids, sizeof(RawObjArray))) continue;
        if (kids->num <= 0 || kids->num > 256 || kids->data == nullptr
            || IsBadReadPtr(kids->data, sizeof(void*) * static_cast<size_t>(kids->num))) continue;
        for (int k = 0; k < kids->num; ++k) push(kids->data[k]);
    }
    return n;
}

// SHRUNK, NOT HIDDEN -- and that is a fix, not a style (the user, 2026-09-27: "the helmet ended up present
// on infantry when I jumped out"). The first cut also cleared the pieces' visibility flags and, on the way
// out, set them to "visible, not hidden" -- which is not their state on foot: in first person the game keeps
// its third-person helmet out of your view itself, and the restore undid that. Scale is the part that does
// the work on this character (the body hider's history: SetHiddenInGame took and the Spartan still drew),
// and the game has no reason to touch a helmet's scale, so the restore owns it cleanly: the scale it had,
// and only while it is still ours.
constexpr double kHeadItemScale = 0.001;

void head_item_hide(API::UObject* c) {
    call_set_scale(c, kHeadItemScale);
}

void head_item_restore(API::UObject* c, const double sc[3]) {
    if (auto* ps = c->get_property_data<double>(L"RelativeScale3D"); ps != nullptr && !IsBadReadPtr(ps, 24)) {
        const bool ours = std::fabs(ps[0] - kHeadItemScale) < 1e-4 && std::fabs(ps[1] - kHeadItemScale) < 1e-4
                       && std::fabs(ps[2] - kHeadItemScale) < 1e-4;
        if (!ours) return;                        // something else set it since: not ours to put back
    }
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    auto* d = reinterpret_cast<double*>(p);
    d[0] = sc[0]; d[1] = sc[1]; d[2] = sc[2];
    c->call_function(L"SetRelativeScale3D", p);
}

int32_t fn_param(API::UFunction* fn, const wchar_t* name) {
    for (auto* f = fn->get_child_properties(); f != nullptr; f = f->get_next())
        if (ffield_name(f) == name) return reinterpret_cast<API::FProperty*>(f)->get_offset();
    return -1;
}

bool head_hide_ready() {
    if (s_hh.state >= 0) return s_hh.state == 1;
    s_hh.state = 0;
    s_hh.skinned = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.SkinnedMeshComponent");
    auto* hide = s_hh.skinned != nullptr ? s_hh.skinned->find_function(L"HideBoneByName") : nullptr;
    auto* unhide = s_hh.skinned != nullptr ? s_hh.skinned->find_function(L"UnHideBoneByName") : nullptr;
    auto* is = s_hh.skinned != nullptr ? s_hh.skinned->find_function(L"IsBoneHiddenByName") : nullptr;
    if (hide != nullptr && unhide != nullptr
        && hide->get_properties_size() <= (int32_t)RIG_PARAM_BUF && unhide->get_properties_size() <= (int32_t)RIG_PARAM_BUF) {
        s_hh.hide_name = fn_param(hide, L"BoneName");
        s_hh.hide_op = fn_param(hide, L"PhysBodyOption");
        s_hh.unhide_name = fn_param(unhide, L"BoneName");
        if (is != nullptr && is->get_properties_size() <= (int32_t)RIG_PARAM_BUF) {   // the readback: optional
            s_hh.is_name = fn_param(is, L"BoneName");
            s_hh.is_ret = fn_param(is, L"ReturnValue");
        }
        if (s_hh.hide_name >= 0 && s_hh.hide_op >= 0 && s_hh.unhide_name >= 0) s_hh.state = 1;
    }
    API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: hiding the player's head bone %s (SkinnedMeshComponent %s, "
                         "HideBoneByName %s, UnHideBoneByName %s)",
                         s_hh.state == 1 ? "is available" : "is NOT available on this build -- hideHead hides the helmet pieces only",
                         s_hh.skinned != nullptr ? "found" : "NOT found", hide != nullptr ? "found" : "NOT found",
                         unhide != nullptr ? "found" : "NOT found");
    return s_hh.state == 1;
}

void head_hide_call(API::UObject* part, bool hide) {
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    const API::FName name = make_fname(s_hh.bone.c_str());   // make_fname: API::FName resolves to None here
    if (hide) {
        std::memcpy(p + s_hh.hide_name, &name, sizeof(int32_t) * 2);
        p[s_hh.hide_op] = 0;                                  // PBO_None: his physics stays as it is
        part->call_function(L"HideBoneByName", p);
    } else {
        std::memcpy(p + s_hh.unhide_name, &name, sizeof(int32_t) * 2);
        part->call_function(L"UnHideBoneByName", p);
    }
}

// -1 = no readback on this build; else whether the part says the bone is hidden.
int head_hidden_readback(API::UObject* part) {
    if (s_hh.is_name < 0 || s_hh.is_ret < 0) return -1;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    const API::FName name = make_fname(s_hh.bone.c_str());
    std::memcpy(p + s_hh.is_name, &name, sizeof(int32_t) * 2);
    part->call_function(L"IsBoneHiddenByName", p);
    return p[s_hh.is_ret] != 0 ? 1 : 0;
}

void head_hide_restore(const char* why) {
    if (!s_hh.on) return;
    int n = 0, ni = 0;
    for (int i = 0; i < s_hh.n; ++i)   // only a bone WE hid: one the game had hidden stays the game's
        if (auto* c = s_hh.parts[i].get(); c != nullptr && !s_hh.part_was_hidden[i]) { head_hide_call(c, false); ++n; }
    for (int i = 0; i < s_hh.ni; ++i)
        if (auto* c = s_hh.items[i].get()) { head_item_restore(c, s_hh.item_scale[i]); ++ni; }
    API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: the player's head shown again (%s; %d helmet piece(s), "
                         "the head bone on %d part(s))", why, ni, n);
    s_hh.on = false;
    s_hh.ni = 0;
}

// Game thread, every tick. want = the selected camera asks for it; body = the Chief's Body (nullptr = not
// resolved yet) at object-array slot body_idx; bone = his head bone (empty = none found). Applied on a
// change and re-asserted once a second -- a hidden bone state is component state, and nothing is known to
// reset it, but a ride is long.
void head_hide_update(bool want, API::UObject* body, int32_t body_idx, const std::wstring& bone) {
    if (!want || body == nullptr || !g_cfg.enabled) {
        head_hide_restore(!want ? "this camera does not hide it" : "the player's body is not resolved");
        return;
    }
    if (s_hh.on && ((uintptr_t)body != s_hh.body || bone != s_hh.bone)) head_hide_restore("a new body");
    // No head bone found, or no bone hide on this build: the helmet pieces still go.
    const bool bones = !bone.empty() && head_hide_ready();
    if (!s_hh.on) {
        s_hh.body = (uintptr_t)body;
        s_hh.bone = bone;
        s_hh.n = 0;
        s_hh.ni = 0;
        auto* owner = body->get_outer();
        std::wstring item_names;
        constexpr int kTree = 128;
        API::UObject* tree[kTree];
        const int nt = (owner != nullptr) ? rider_tree(owner, tree, kTree) : 0;
        for (int t = 0; t < nt; ++t) {
            API::UObject* c = tree[t];
            const int32_t idx = uobject_slot_index(c);
            if (idx < 0) continue;
            std::wstring why;
            // Only a MESH is ever hidden here -- never a camera, a light or a scene node hung from the head.
            const bool mesh = class_name_of(c).find(L"Mesh") != std::wstring::npos;
            if (c != body && mesh && s_hh.ni < kMaxHeadItems && is_head_item(c, bone, &why)) {
                // Its own relative scale, for the restore (1 is only the usual answer).
                double sc[3] = {1.0, 1.0, 1.0};
                if (auto* ps = c->get_property_data<double>(L"RelativeScale3D"); ps != nullptr && !IsBadReadPtr(ps, 24)
                    && std::isfinite(ps[0]) && std::isfinite(ps[1]) && std::isfinite(ps[2]) && ps[0] > 0.01 && ps[1] > 0.01 && ps[2] > 0.01) {
                    sc[0] = ps[0]; sc[1] = ps[1]; sc[2] = ps[2];
                }
                s_hh.items[s_hh.ni].set_at(c, idx);
                for (int k = 0; k < 3; ++k) s_hh.item_scale[s_hh.ni][k] = sc[k];
                ++s_hh.ni;
                item_names += (item_names.empty() ? L"" : L", ") + why;
                continue;
            }
            if (bones && s_hh.n < kMaxDriverParts && c->is_a(s_hh.skinned)) s_hh.parts[s_hh.n++].set_at(c, idx);
        }
        if (bones && s_hh.n == 0) { s_hh.parts[0].set_at(body, body_idx); s_hh.n = 1; }   // not in the walk: the Body itself
        for (int i = 0; i < s_hh.ni; ++i)
            if (auto* c = s_hh.items[i].get()) head_item_hide(c);
        int hidden = 0, readable = 0;
        for (int i = 0; i < s_hh.n; ++i) {
            auto* c = s_hh.parts[i].get();
            s_hh.part_was_hidden[i] = false;
            if (c == nullptr) continue;
            s_hh.part_was_hidden[i] = head_hidden_readback(c) == 1;   // before we touch it
            head_hide_call(c, true);
            const int r = head_hidden_readback(c);
            if (r >= 0) { ++readable; hidden += r; }
        }
        s_hh.on = true;
        s_hh.reassert = 0;
        // The readback says the bone call LANDED, not that the head stopped drawing: only the headset says that.
        API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: the player's head hidden -- %d helmet piece(s) shrunk "
                             "(%ls) of %d component(s) on him; bone \"%ls\" on %d part(s), the readback says hidden on %d of %d",
                             s_hh.ni, item_names.empty() ? L"NONE FOUND" : item_names.c_str(), nt, s_hh.bone.c_str(), s_hh.n,
                             hidden, readable);
#if HALO_VR_DEV
        // His whole tree, so a helmet this test missed is named in the log: class, mesh, socket.
        for (int t = 0; t < nt; ++t) {
            API::UObject* c = tree[t];
            if (!uobject_slot_valid(c)) continue;
            std::wstring why, mesh, sock;
            const bool item = c != body && class_name_of(c).find(L"Mesh") != std::wstring::npos && is_head_item(c, bone, &why);
            for (const wchar_t* prop : { L"StaticMesh", L"SkinnedAsset", L"SkeletalMesh" }) {
                if (!mesh.empty()) break;
                if (auto* pm = c->get_property_data<API::UObject*>(prop); pm != nullptr && !IsBadReadPtr(pm, sizeof(void*)) && *pm != nullptr)
                    if (auto* fn = (*pm)->get_fname()) mesh = fn->to_string();
            }
            if (auto* ps = c->get_property_data<API::FName>(L"AttachSocketName"); ps != nullptr && !IsBadReadPtr(ps, sizeof(API::FName)))
                sock = ps->to_string();
            API::get()->log_info("[Halo-CampE-UEVR] VEHCAM   head-hide tree: %ls mesh=%ls socket=%ls%s", c->get_full_name().c_str(),
                                 mesh.empty() ? L"-" : mesh.c_str(), sock.empty() ? L"-" : sock.c_str(),
                                 item ? "  <-- HELMET" : "");
        }
#endif
        return;
    }
    // Re-asserted once a second, like the body hider: anything that re-applies the pieces' visibility or
    // scale (the body hider's own restore, among others) is overruled within a second.
    if ((++s_hh.reassert % 32u) == 0u) {
        for (int i = 0; i < s_hh.n; ++i)
            if (auto* c = s_hh.parts[i].get()) head_hide_call(c, true);
        for (int i = 0; i < s_hh.ni; ++i)
            if (auto* c = s_hh.items[i].get()) head_item_hide(c);
    }
}

} // namespace

// Applied on every change while mounted, and re-asserted once a second in case something drives it
// back (it was every tick: the readback since showed the hide holding, so that cost bought nothing).
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
    // OUR CAMERAS hide it only while vehcamhidebody asks (off by default: you see the Chief in the seat),
    // and then per camera (the camera file's "hideBody"; left out = hidden exactly when the camera sits at
    // your seat). Stepping between cameras, or flipping the key, shows and hides it live. 2 shrinks it as
    // well -- mode 1 alone was measured insufficient on the seat camera (the body drew regardless).
    const VehActiveCam hide_cam = veh_active_cam();
    const bool tp_hide = g_cfg.veh_cam_hide_body != 0 && g_veh_tp_active.load(std::memory_order_relaxed)
                      && hide_cam.valid && hide_cam.hide_body
                      && halo::g_stick_mode_active.load(std::memory_order_relaxed);
    // A FIRST-PERSON entry puts the view INSIDE the body, so it honours its own "hideBody" (left out =
    // hidden) whatever vehcamhidebody says -- showing the Chief is for the cameras that look at him.
    // Hidden and shrunk unless vehcamhidebody names a mode: hiding alone was measured not to take.
    const bool fp_hide = veh_fp_selected() && hide_cam.valid && hide_cam.hide_body
                      && halo::g_stick_mode_active.load(std::memory_order_relaxed);
    const bool seat_hide = g_cfg.veh_hide_body != 0 && veh_cam_mode(g_cfg.veh_cam) != 0
                        && (g_unit_mounted.load(std::memory_order_relaxed) || veh_fp_selected());
    const bool want = g_cfg.enabled && (tp_hide || fp_hide || seat_hide);
    // The mode is the switch that asked: the vehicle cameras' own, a first-person entry's, or bc24's
    // seat camera's.
    const int  hide_mode = tp_hide ? g_cfg.veh_cam_hide_body
                         : fp_hide ? (g_cfg.veh_cam_hide_body != 0 ? g_cfg.veh_cam_hide_body : 2)
                                   : g_cfg.veh_hide_body;

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
        s_hide_applied = -1;   // the next hide applies at once
        if (!still_seated) s_driver_part_count = 0;   // the next ride resolves afresh
        s_driver_tries = 0;
        s_driver_wait = 0;
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
        // The parts come from the RIDE SCAN: one walk shared with the chassis search and spread over
        // ticks, instead of two full walks of its own on the one tick that needed them (~200 ms, most
        // of the boarding freeze). Every 2 s, ten tries per mount, then it gives up until the next
        // mount. The first try takes a walk already finished in this stick-mode window -- the chassis
        // search's -- so it usually costs nothing; a retry looks again.
        if (s_driver_wait == 0) {
            if (s_driver_tries >= 10) return;
            const ULONGLONG t = GetTickCount64();
            if (t - s_driver_try_at < 2000) return;
            s_driver_try_at = t;
            s_driver_wait = ride_scan_request(/*force_new=*/s_driver_tries > 0);
            ++s_driver_tries;
        }
        if (ride_scan_done() < s_driver_wait) return;     // still walking
        s_driver_wait = 0;
        if (ride_scan_take_driver_parts() == 0) return;    // none found: the next try, in 2 s
        s_driver_tries = 0;
        s_driver_hidden = true;
        s_hide_applied = -1;                               // new parts: hide them this tick
        API::get()->log_info("[Halo-CampE-UEVR] VEHBODY: driver hidden (%d parts)",
                             s_driver_part_count);
    }

    // APPLIED ON A CHANGE, THEN RE-ASSERTED ONCE A SECOND -- not every tick. The readback showed the hide
    // HOLD with nothing reverting it, so the per-tick re-apply (3-4 reflection calls a part, every tick)
    // bought nothing; the 1 Hz re-assert stays in case something ever puts it back. A change here = the
    // parts were just found, the mode changed, or the hide just came back on.
    {
        static int      s_last_mode = 0;
        static uint32_t s_reassert = 0;
        const bool changed = hide_mode != s_last_mode || s_driver_part_count != s_hide_applied;
        if (changed || (++s_reassert % 32u) == 0u) {
            for (int i = 0; i < s_driver_part_count; ++i) {
                auto* c = s_driver_parts[i].get();
                if (c == nullptr) continue;
                rig_set_always_tick_pose(c);
                call_set_hidden(c, true);
                call_set_visibility(c, false);
                if (hide_mode == 2) call_set_scale(c, 0.001);
            }
            s_hide_applied = s_driver_part_count;
        }
        // Mode 2 -> 1 mid-ride: put the scale back once, or the parts stay shrunk under mode 1.
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
// position (the pawn sits in the vehicle), found when a ride starts (veh_ride_update), published game-side;
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
// THE PLAYER'S HEAD (origin "playerhead"): the Chief's head bone, measured on the tick in the CHIEF's own
// frame -- his actor's -- which turns with a turret whose mesh does not. The eye rebuilds it every frame
// against his actor's live transform (read through his Body component, published here), and uses that
// frame for the camera's tracking too.
std::atomic<uintptr_t> g_tp_rider_ptr{0};
std::atomic<int32_t>   g_tp_rider_idx{-1};
std::atomic<float>     g_tp_head_x{0.0f}, g_tp_head_y{0.0f}, g_tp_head_z{0.0f};
std::atomic<bool>      g_tp_head_valid{false};
// A SOCKET ORIGIN (a camera file "origin" naming a bone or socket, e.g. the Scorpion cannon's MainTurret_M):
// the mesh component it was found on and the name as an FName's 8 bytes, resolved on the tick
// (socket_resolve). The eye reads the socket's live transform every frame and re-captures its frame when
// the generation moves (a new socket, a new vehicle).
std::atomic<uintptr_t> g_tp_sock_ptr{0};
std::atomic<int32_t>   g_tp_sock_idx{-1};
std::atomic<uint64_t>  g_tp_sock_fname{0};
std::atomic<uint32_t>  g_tp_sock_gen{0};
// WHICH PATH OUR CAMERA TOOK THIS FRAME (bc24's VEHCAMPATH, release-safe): the eye cannot log, so it sets
// these bits and the game tick names each change, the moment a fallback starts or ends.
constexpr uint8_t kTpPathEngine = 1;      // the vehicle's transform could not be read: the game's own camera
constexpr uint8_t kTpPathHeadToSeat = 2;  // a playerhead camera with no head measured: the seat stands in
constexpr uint8_t kTpPathToOrigin = 4;    // a seat/head camera with no seat measured: the vehicle's origin
constexpr uint8_t kTpPathSocketToVehicle = 8;   // a socket camera whose socket is not found: the vehicle's origin
std::atomic<uint8_t>   g_tp_path{0};
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
// THE RECENTER ON A CAMERA CHANGE (vehcamrecenter). The eye works out the turn that points what aims the
// vehicle (the aim hand, else your head) where the vehicle is aiming, publishes it here and uses it at
// once; the tick adopts it into g_veh_turn_yaw, the turn it owns, and acknowledges. Until then the eye
// keeps using the published value.
std::atomic<float>     g_veh_turn_reset_val{0.0f};
std::atomic<uint32_t>  g_veh_turn_reset_seq{0}, g_veh_turn_reset_ack{0};

// Left Y in a vehicle: the next camera; left X: the current camera's next tethering mode. Called from the
// input hook (any thread); the game tick applies them (VehCamSelect.cpp).
void veh_cam_next_prev(int dir) { veh_cam_step(dir); }
bool veh_uevr_override_active() { return host::g_plugin_state.cut2d_engaged->load(std::memory_order_relaxed); }
void veh_cam_mode_next() { veh_cam_mode_step(+1); }

// vehaim: the Blam aim write (BlamDrive.cpp) consults this to lift its stick-mode hold-off in a
// vehicle. True only when the owned TP camera is ACTIVE AND motion aim is on for this vehicle (its
// "motionAim" in the camera file, else vehaim) AND the chassis is resolved (= we are actually in a
// vehicle, not a cutscene/death, which also raise stick mode). POD reads + atomics, so it is safe on
// the sim orientation getter's thread.
bool veh_tp_motion_aim_active() {
    // AND OUR EYE DREW LAST FRAME. The hand's ray is built from the view the eye publishes, and without
    // our camera the game's own chase camera follows the aim -- hand aim would swing it round, which is
    // the whole reason stick mode exists (BlamDrive.cpp). So the aim holds until our camera is up.
    return veh_tp_motion_aim_selected() && g_tp_chassis_yaw_valid.load(std::memory_order_relaxed);
}

// What the selected camera ASKS for, whether or not the eye has drawn yet -- the recenter decides with it
// on the very first frame of a camera, before the eye has published anything.
bool veh_tp_motion_aim_selected() {
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

// THE VEHICLE AIM MARKER (the camera file's "aimMarker", XRLAYER_SLOT_VEHAIM): the generated ring where
// the VEHICLE aims, beside the crosshair that shows where you point -- with a camera that turns with the
// vehicle, the view settles only once the two meet, and this is how you see where that is. The eye
// publishes the ray it is drawn along (its origin as an offset from the chassis, and its direction: the
// vehicle's heading at the game's aim pitch); the tick traces that ray for its depth -- the reticule's
// rule: on what it hits, the last range held on a miss, the far end before any hit -- and publishes the
// range; the eye rebuilds the point each frame on the live chassis and direction. A range crosses the
// clock boundary, never a point.
std::atomic<float> g_vmk_ox{0.0f}, g_vmk_oy{0.0f}, g_vmk_oz{0.0f};
std::atomic<float> g_vmk_fx{1.0f}, g_vmk_fy{0.0f}, g_vmk_fz{0.0f};
std::atomic<bool>  g_vmk_ray_valid{false};
std::atomic<float> g_vmk_range{0.0f};

bool veh_aim_ray_angles(float* yaw, float* pitch) {
    if (!veh_tp_motion_aim_active() || !g_cfg.veh_aim_ray) return false;
    if (!g_veh_aim_valid.load(std::memory_order_relaxed)) return false;
    *yaw   = g_veh_aim_yaw.load(std::memory_order_relaxed);
    *pitch = g_veh_aim_pitch.load(std::memory_order_relaxed);
    return true;
}

// UE cm per REAL metre, as UEVR is rendering it right now: 100 x VR_WorldScale (the direction is
// measured, not assumed -- HeightCal fitted 112.7 cm/m against 100 x 1.126 = 112.6). The player sets
// their own world scale, so a fixed number puts the controller ray and the head anchor off by the
// ratio. Read through the one shared reader (core/WorldScale.hpp): refreshed on every config poll,
// kept through the cutscene mono collapse, and the cached form is a single atomic load on any thread.
namespace {
float veh_cm_per_m() { return halo::uevr_cm_per_metre_cached(); }

// UEVR'S ROTATION OFFSET -- what its "Recenter View" writes. UEVR draws the view AND the hands as
// rotation_offset . (pose - standing_origin), in VR space before the axis swap (FFakeStereoRenderingHook.cpp
// calculate_stereo_view_offset; UObjectHook.cpp for the hands). A raw get_pose() leaves it out, so every
// room -> world step here puts it back: without it, after a Recenter View the pointing ray, the head capture
// and the recenter all turned by its angle away from the hand you see. The on-foot rig applies the same
// value (Plugin.cpp, q_ro). GAME THREAD: the tick and the eye.
Quat veh_rotation_offset() {
    const auto ro = API::VR::get_rotation_offset();
    return quat_normalize(Quat{ro.x, ro.y, ro.z, ro.w});
}

// The range the reticule and the aim use: the held measurement, or the far end of the ray before this
// ride has measured anything. ONE rule for the tick and the per-frame stamp, so they cannot disagree.
float veh_aim_range_eff() {
    const float r = g_veh_aim_range.load(std::memory_order_relaxed);
    if (r > 1.0f) return r;
    return (g_cfg.veh_aim_far > 1.0f) ? g_cfg.veh_aim_far : 10000.0f;
}

// The controller's world ray EXACTLY as UEVR draws it in our third-person camera:
//     world = view_base + R(view) . swizzle(ro . (room - standing_origin)) . (100 x VR_WorldScale)
// where ro is UEVR's rotation offset (veh_rotation_offset), applied in VR space before the swizzle.
// -- the transform the head gets too (the anchor capture in the eye uses the same one). R(view) is the
// FULL view rotation: a camera that tilts with the vehicle tilts the whole tracking space, hands included,
// and UEVR composes it as UE's own rotator (yaw, then pitch, then roll -- FFakeStereoRenderingHook.cpp's
// yawPitchRoll(-yaw, pitch, -roll) is that rotator expressed in the VR axes).
// room_to_world() is the wrong tool here: without room_anchor it hangs the offset off the HMD at g_cam,
// which in this camera is the VIEW BASE rather than the head. That shifts the whole ray sideways --
// centimetres to tens of centimetres -- so the aim point and the crosshair sat off the line the hand
// actually points along. Direction is scale-free. Any thread: pure maths, the cached world scale, and
// UEVR's standing origin (which the eye callback already reads).
void veh_room_ray(const Vec3& cpos, const Vec3& fwd, const double base[3],
                  float view_pitch, float view_yaw, float view_roll, Vec3* o, Vec3* d) {
    const auto so = API::VR::get_standing_origin();
    const double rs = (double)veh_cm_per_m();
    double X[3], Y[3], Z[3];
    rot_axes(view_pitch, view_yaw, view_roll, X, Y, Z);
    const Quat ro = veh_rotation_offset();
    const Vec3 rel = quat_rotate(ro, Vec3{cpos.x - so.x, cpos.y - so.y, cpos.z - so.z});
    const Vec3 rf  = quat_rotate(ro, fwd);
    // room -> UE: X = -z, Y = x, Z = y. Same rotation as the eye's anchor subtraction.
    const double lx = -(double)rel.z * rs;
    const double ly =  (double)rel.x * rs;
    const double lz =  (double)rel.y * rs;
    *o = Vec3{ (float)(base[0] + lx * X[0] + ly * Y[0] + lz * Z[0]),
               (float)(base[1] + lx * X[1] + ly * Y[1] + lz * Z[1]),
               (float)(base[2] + lx * X[2] + ly * Y[2] + lz * Z[2]) };
    const double fx = -(double)rf.z, fy = (double)rf.x, fz = (double)rf.y;
    double dx = fx * X[0] + fy * Y[0] + fz * Z[0];
    double dy = fx * X[1] + fy * Y[1] + fz * Z[1];
    double dz = fx * X[2] + fy * Y[2] + fz * Z[2];
    const double dl = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dl > 1e-9) { dx /= dl; dy /= dl; dz /= dl; }
    *d = Vec3{ (float)dx, (float)dy, (float)dz };
}

// THE CONTROLLER THAT AIMS A VEHICLE: the aim hand (right unless aimhand says left). ONE place for the
// ray aim, its per-frame reticule and the recenter, so the three can never read different hands -- and
// the one place a head-aimed or other-hand vehicle lane would change. -1 = no such controller.
int32_t veh_aim_hand_index() {
    return g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                               : API::VR::get_right_controller_index();
}

} // namespace

// Who publishes the ONE compositor reticule while you aim a vehicle with the controller: the eye
// callback, every frame, on the live pointing ray (see the stamp in vehcam_stereo_pre_eye_seat). The
// tick path in Plugin.cpp stands its compositor publish down on this -- the layer's snapshot takes one
// writer at a time -- and keeps placing the world-space widget/mesh reticules. Any thread.
// NOT IN A MENU. Stick mode holds through a pause, and a stamp taken every frame never goes stale, so
// without this the crosshair floated over the pause menu. The tick path it hands back to is menu-gated
// itself, so the slot simply ages out.
bool veh_tp_reticle_stamp_owns() {
    return g_cfg.xr_layer && g_cfg.aim_reticule
        && veh_tp_motion_aim_active() && g_cfg.veh_aim_ray
        && halo::g_stick_mode_active.load(std::memory_order_relaxed)
        && !host::g_plugin_state.in_menu->load(std::memory_order_relaxed)
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
// The resolved chassis's full object name ("...BP_BansheeVehicleActor_C_<id>.hull"), for the log.
std::wstring s_tp_chassis_name;
// What the camera file is matched against: the vehicle actor your SEAT belongs to when the game has said
// (seat_resolve), else the chassis mesh's name. GAME THREAD.
std::wstring s_tp_match_name;

// Who "you" are to the seat resolve: the pawn, and the Spartan biped the body hider would pick.
int seat_me(API::UObject** me, double p[3]) {
    auto* pawn = API::get()->get_local_pawn(0);
    Vec3 ploc{};
    if (pawn == nullptr || !call_ret_vec3(pawn, L"K2_GetActorLocation", &ploc)) return 0;
    p[0] = (double)ploc.x; p[1] = (double)ploc.y; p[2] = (double)ploc.z;
    me[0] = pawn;
    me[1] = nullptr;
    if (const ScanHit* b = ride_scan_pick_driver_body()) me[1] = b->owner;
    return 2;
}

// GAME THREAD. THE VEHICLE YOU ARE IN, and the mesh the cameras use as its frame, from the last finished
// RIDE SCAN (the walk itself is spread over ticks there). The game names the vehicle actor holding your
// seat (seat_resolve) -- that actor's own mesh is the chassis, and that actor is what the camera file
// matches. When the game does not say, or that actor has no mesh of its own, the nearest VehicleActor
// SkeletalMeshComponent to the pawn stands in, skipping the meshes of OTHER vehicles near you that have
// seats (from the Wraith's driver seat the anti-infantry turret's mesh is the nearer one).
//
// NAMES FIRST, distance second (bc24's hull rule): a mesh named in the entry's "chassis" -- or, with none
// given, one named hull or body -- beats any unnamed mesh, however near; bc24 measured the Warthog's
// chaingun (its own actor) nearer the driver than the hull. The next two candidates are logged beside the
// choice. `known` = a seat already resolved by the caller. True = resolved.
bool pick_tp_chassis(const SeatFix* known = nullptr) {
    API::UObject* me[2] = {};
    double pp[3] = {};
    if (seat_me(me, pp) == 0) return false;
    const double px = pp[0], py = pp[1], pz = pp[2];
    if (known != nullptr) s_seat = *known;
    else seat_resolve(me, 2, pp, &s_seat);
    // TWO WITNESSES. The Blam mount flag started this ride; once the game's own seats have named yours
    // this session, they must agree. A ride they do not back -- a misread flag, or a scene that parks you
    // beside a vehicle -- gets no camera rather than the nearest one. (The caller retries while the game
    // catches up: a seat can be named a moment after the flag.)
    if (s_seats_proven && !s_seat.valid) {
        static ULONGLONG s_said = 0;
        const ULONGLONG t = GetTickCount64();
        if (t - s_said > 5000) {
            s_said = t;
            API::get()->log_info("[Halo-CampE-UEVR] VEHSEAT: the mount flag says seated but no vehicle within 15 m lists "
                                 "you -- no vehicle camera until one does");
        }
        return false;
    }

    auto other_seated = [](API::UObject* owner) {
        for (int i = 0; i < s_seat_other_n; ++i) if (s_seat_others[i] == owner) return true;
        return false;
    };
    std::vector<std::wstring> hint;
    if (s_seat.valid)
        for (const std::string& s : vehcam_chassis_hint(s_seat.actor->get_full_name(), s_seat.bits))
            hint.emplace_back(s.begin(), s.end());
    auto preferred = [&hint](API::UObject* o) {
        auto* f = o->get_fname();
        std::wstring n = (f != nullptr) ? f->to_string() : std::wstring{};
        for (auto& ch : n) ch = (wchar_t)towlower(ch);
        if (!hint.empty()) {
            for (const std::wstring& s : hint) if (n.find(s) != std::wstring::npos) return true;
            return false;
        }
        return n == L"hull" || n == L"body";
    };
    // pass 0: your seat's own vehicle (up to 30 m: it IS yours); 1: the nearest within 8 m, skipping other
    // seated vehicles; 2: the nearest within 8 m. Within a pass a preferred name wins, then distance.
    static const wchar_t* const kWhy[] = { L"your seat's vehicle (the game)", L"nearest, skipping other seated vehicles",
                                           L"nearest" };
    constexpr double kUnnamed = 100000.0;
    const ScanHit* cand[3] = {nullptr, nullptr, nullptr};
    double nd[3] = {1e18, 1e18, 1e18};   // score: the distance, plus kUnnamed for a mesh without a preferred name
    int used = -1;
    for (int pass = s_seat.valid ? 0 : 1; pass <= 2 && used < 0; ++pass) {
        cand[0] = cand[1] = cand[2] = nullptr;
        nd[0] = nd[1] = nd[2] = 1e18;
        for (const ScanHit& h : s_res_chassis) {
            if (!scan_hit_live(h)) continue;
            if (pass == 0 && h.owner != s_seat.actor) continue;
            if (pass == 1 && other_seated(h.owner)) continue;
            Vec3 w{}; if (!call_ret_vec3(h.o, L"K2_GetComponentLocation", &w)) continue;
            const double dist = std::sqrt(((double)w.x - px) * ((double)w.x - px)
                                        + ((double)w.y - py) * ((double)w.y - py)
                                        + ((double)w.z - pz) * ((double)w.z - pz));
            if (dist >= (pass == 0 ? 3000.0 : 800.0)) continue;   // within 8 m of the pawn = the vehicle it is in
            const double d = dist + (preferred(h.o) ? 0.0 : kUnnamed);
            for (int k = 0; k < 3; ++k) {
                if (d < nd[k]) {
                    for (int m = 2; m > k; --m) { nd[m] = nd[m - 1]; cand[m] = cand[m - 1]; }
                    nd[k] = d; cand[k] = &h;
                    break;
                }
            }
        }
        if (cand[0] != nullptr) used = pass;
    }
    if (used < 0) {
        API::get()->log_info("[Halo-CampE-UEVR] VEHTP: no VehicleActor mesh within 8 m of pawn "
                             "(pawn %.0f %.0f %.0f, %d candidates)", px, py, pz, (int)s_res_chassis.size());
        return false;
    }
    const bool named = nd[0] < kUnnamed;
    for (double& d : nd) if (d >= kUnnamed && d < 1e17) d -= kUnnamed;   // back to distances, for the log
    g_tp_chassis_ptr.store((uintptr_t)cand[0]->o, std::memory_order_relaxed);
    g_tp_chassis_idx.store(cand[0]->i, std::memory_order_relaxed);
    s_tp_chassis_name = cand[0]->o->get_full_name();
    s_tp_match_name = s_seat.valid ? s_seat.actor->get_full_name() : s_tp_chassis_name;
    std::wstring alt;
    for (int k = 1; k < 3; ++k) {
        if (cand[k] == nullptr) break;
        wchar_t d[32]; swprintf_s(d, L" %.0fcm", nd[k]);
        alt += L"; ";
        alt += cand[k]->o->get_full_name();
        alt += d;
    }
    if (s_seat.valid)
        API::get()->log_info("[Halo-CampE-UEVR] VEHSEAT: the game seats you in %ls, seat %d of %d (%s)%s",
                             s_tp_match_name.c_str(), s_seat.seat + 1, s_seat.nseats,
                             vehcampresets::seat_text(s_seat.bits).c_str(),
                             s_seat.by_path ? " -- known by the seat's path, its pointer was unresolved" : "");
    else if (seat_refl_ready())
        API::get()->log_info("[Halo-CampE-UEVR] VEHSEAT: no vehicle within 15 m lists you in a seat (%d nearby with "
                             "seats) -- the nearest mesh decides", s_seat_other_n);
    API::get()->log_info("[Halo-CampE-UEVR] VEHTP chassis: %ls at %.0fcm from pawn(%.0f %.0f %.0f), %ls, %s -- next: %ls",
                         s_tp_chassis_name.c_str(), nd[0], px, py, pz, kWhy[used],
                         named ? (hint.empty() ? "named hull/body" : "named in the entry's chassis") : "no preferred name",
                         alt.empty() ? L"none" : alt.c_str() + 2);
    return true;
}

// ---- THE RIDE: a POSITIVE "you are in a vehicle seat", not merely stick mode.
//
// Stick mode is also every cutscene, every death and the window after a level load, and the chassis
// search used to run in all of them -- every ~3 s, a full walk of the object array each time. The
// game's own MOUNT FLAG (the biped's parent datum, core/UnitState, published on the sim thread while
// our cameras are on) says "in a seat" and nothing else: it read mounted on 302 of 318 samples of a
// ride, the rest at its edges (2026-09-25 log). So a ride is stick mode AND mounted AND no cutscene --
// a cutscene is never a ride, mounted or not.
//
// The flag only means something once the seat publish has run at all this session (it has published a
// rider position); on a build where the unit record never resolves, this falls back to stick mode
// minus cutscenes rather than to nothing. Leaving waits ~0.5 s of unmounted readings, so a flicker
// mid-ride never drops the camera; stick mode ending or a cutscene starting ends it at once.
// GAME THREAD (the cutscene flags are the tick's own).
bool s_in_ride = false;

bool veh_ride_update(bool stick) {
    static bool     s_proven = false;
    static uint32_t s_unmounted = 0;
    static bool     s_said = false;
    if (!s_proven && halo::g_seat_pub_seq.load(std::memory_order_relaxed) != 0) s_proven = true;
    const auto& ps = host::g_plugin_state;
    const bool cine = *ps.cine_answering && *ps.cine_active;
    if (!stick || cine) {
        s_unmounted = 0; s_said = false;
        s_in_ride = false;
        return false;
    }
    if (!s_proven || halo::g_unit_mounted.load(std::memory_order_relaxed)) {
        s_unmounted = 0;
        s_in_ride = true;
        return true;
    }
    ++s_unmounted;
    if (s_in_ride && s_unmounted < 16) return true;          // a flicker mid-ride
    s_in_ride = false;
    if (s_unmounted == 64 && !s_said) {                      // ~2 s: say once why nothing is searched
        s_said = true;
        API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: stick mode without the mount flag -- not a vehicle seat "
                             "(a death, a load or a scripted scene), so no vehicle search");
    }
    return false;
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

// Log every VehicleActor SkeletalMeshComponent near the search CENTRE with its WORLD rotation, and latch
// the nearest one NOT named .hull/.body as the turret candidate. The meshes come from the RIDE SCAN's
// finished walk (it used to walk the whole object array itself -- another ~100 ms stall per boarding).
// Centre is the rendered VIEW position. If no centre is available it logs all VehicleActor meshes
// (capped). The rotations tell whether the gun points independently of the hull; the record aim (each
// beat) tells whether that aim is the shared value.
void vehprobe_scan(bool have_ctr, double cx, double cy, double cz) {
    auto ends_with_ci = [](const std::wstring& s, const wchar_t* suf) {
        const size_t m = wcslen(suf);
        if (s.size() < m) return false;
        for (size_t k = 0; k < m; ++k)
            if (towlower(s[s.size() - m + k]) != towlower(suf[k])) return false;
        return true;
    };
    API::UObject* nearest_gun = nullptr; double ngd = 1e18; int logged = 0;
    API::get()->log_info("[Halo-CampE-UEVR] VEHPROBE scan (haveCtr=%d ctr=%.0f,%.0f,%.0f): VehicleActor skeletal meshes --",
                         (int)have_ctr, cx, cy, cz);
    for (const ScanHit& h : s_res_chassis) {
        if (!scan_hit_live(h)) continue;
        auto* o = h.o;
        const std::wstring full = o->get_full_name();
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
    // Gate on THE RIDE (veh_ride_update: stick mode + the mount flag, never a cutscene). The first cut
    // gated on stick mode alone, which also ran it through every cutscene, death and post-load window.
    // (The "g_unit_mounted reads dead" that pushed it there was a probe run without the seat publish
    // switched on -- see docs\VEHICLE-BC24-COMPARISON-2026-09-26.md.)
    const bool ride = s_in_ride;
    static bool was_ride = false;
    if (!ride) {   // reset on exit so the next vehicle re-scans
        if (was_ride) { g_vehprobe_scanned = false; g_vehprobe_turret.store(0, std::memory_order_relaxed); }
        was_ride = false;
        return;
    }
    was_ride = true;

    // Search centre: the rendered view position, via the plugin-state bridge (reliable).
    auto& vpx = *host::g_plugin_state.view_pos_x;
    auto& vpy = *host::g_plugin_state.view_pos_y;
    auto& vpz = *host::g_plugin_state.view_pos_z;
    const double cx = (double)vpx.load(std::memory_order_relaxed);
    const double cy = (double)vpy.load(std::memory_order_relaxed);
    const double cz = (double)vpz.load(std::memory_order_relaxed);
    const bool have_ctr = (cx != 0.0 || cy != 0.0 || cz != 0.0);

    if (!g_vehprobe_scanned && ride_scan_fresh()) {   // the ride scan's walk for this window, once it is in
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
    // Clamped, and a non-finite value ignored: atof takes "inf" and "nan", and the menu's drag box does not
    // clamp a value typed with Ctrl+Click. A rate of 1e13 hung the game thread in the view-turn wrap.
    if (_stricmp(key, "vehorbitrate")   == 0) { if (std::isfinite(v)) g_cfg.veh_orbit_rate = clampf((float)v, 0.0f, 720.0f); return true; }
    if (_stricmp(key, "vehorbitreturn") == 0) { if (std::isfinite(v)) g_cfg.veh_orbit_return = clampf((float)v, 0.0f, 360.0f); return true; }
    if (_stricmp(key, "vehaimorigin")   == 0) { g_cfg.veh_aim_origin = (int)v; return true; }
    if (_stricmp(key, "vehcamreadout")  == 0) { g_cfg.veh_cam_readout = (v != 0.0); return true; }
    if (_stricmp(key, "vehctrlclick")   == 0) { g_cfg.veh_ctrl_click = (int)v; return true; }
    if (_stricmp(key, "vehseatgrip")    == 0) { g_cfg.veh_seat_grip = (int)v; return true; }
    if (_stricmp(key, "vehseatmask")    == 0) { g_cfg.veh_seat_mask = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "vehcamrecenter") == 0) { g_cfg.veh_cam_recenter = (v != 0.0); return true; }
    if (_stricmp(key, "vehcamrecenterpos") == 0) { g_cfg.veh_cam_recenter_pos = (v != 0.0); return true; }
    if (_stricmp(key, "vehcamhidebody") == 0) { g_cfg.veh_cam_hide_body = (int)v; return true; }
    if (_stricmp(key, "vehmarker")      == 0) { g_cfg.veh_marker = (v != 0.0); return true; }
    if (_stricmp(key, "vehmarkerradius") == 0) { g_cfg.veh_marker_radius = (float)v; return true; }
    if (_stricmp(key, "vehmarkerthick") == 0) { g_cfg.veh_marker_thick = (float)v; return true; }
    if (_stricmp(key, "vehmarkerdot")   == 0) { g_cfg.veh_marker_dot = (float)v; return true; }
    if (_stricmp(key, "vehmarkersize")  == 0) { g_cfg.veh_marker_size = (float)v; return true; }
    if (_stricmp(key, "vehmarkercr")    == 0) { g_cfg.veh_marker_cr = clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "vehmarkercg")    == 0) { g_cfg.veh_marker_cg = clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "vehmarkercb")    == 0) { g_cfg.veh_marker_cb = clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "vehmarkeralpha") == 0) { g_cfg.veh_marker_alpha = clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "vehaimray")      == 0) { g_cfg.veh_aim_ray = (v != 0.0); return true; }
    // An infinite far end made the traced point infinite and the aim pitch NaN, written into Blam's aim record.
    if (_stricmp(key, "vehaimfar")      == 0) { if (std::isfinite(v)) g_cfg.veh_aim_far = clampf((float)v, 100.0f, 1000000.0f); return true; }
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

// ---------------------------------------------------------------- A SOCKET ORIGIN
//
// A camera file "origin" that is not vehicle / seat / playerhead names a BONE OR SOCKET on the vehicle, so a
// camera can ride a part that turns on its own: the Scorpion's cannon turns separately from its hull, its seat
// and the Chief (the user, 2026-09-27). Searched on the vehicle's own mesh first, then on the vehicle meshes
// near your seat (a turret is often an actor of its own -- BP_ScorpionCannonVehicleActor), nearest first;
// "Part/Name" keeps only meshes whose path (and so their actor's name) contains Part. An exact bone or
// socket name first (DoesSocketExist answers both), then the first bone whose name CONTAINS it.
//
// The names came from the pak, without a session (2026-09-27, retoc to-legacy of the SK_* assets): nearly
// every vehicle skeleton carries Root_M / Pedestal / AimYaw / AimPitch -- the Scorpion's HULL included -- so
// "AimYaw" alone finds the hull's; the cannon's own parts (MainTurret_M, Barrel_M, aiming_pivot) are unique
// to it, or name the part: "ScorpionCannon/AimYaw".
namespace {
struct SockFnRefl { int state = -1; int32_t name_off = -1, ret_off = -1; };
SockFnRefl s_dse;   // USceneComponent.DoesSocketExist(FName InSocketName) -> bool

bool does_socket_exist(API::UObject* comp, const API::FName& name) {
    if (s_dse.state < 0) {
        s_dse.state = 0;
        auto* sc = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.SceneComponent");
        if (auto* fn = (sc != nullptr) ? sc->find_function(L"DoesSocketExist") : nullptr) {
            s_dse.name_off = fn_param(fn, L"InSocketName");
            s_dse.ret_off = fn_param(fn, L"ReturnValue");
            if (s_dse.name_off >= 0 && s_dse.ret_off >= 0 && fn->get_properties_size() <= (int32_t)RIG_PARAM_BUF)
                s_dse.state = 1;
        }
        API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: DoesSocketExist %s", s_dse.state == 1
                             ? "resolved -- socket origins search by exact name" : "NOT resolved -- socket origins search bone names only");
    }
    if (s_dse.state != 1) return false;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    std::memcpy(p + s_dse.name_off, &name, sizeof(int32_t) * 2);
    comp->call_function(L"DoesSocketExist", p);
    return p[s_dse.ret_off] != 0;
}

std::wstring widen_ascii(const std::string& s) { return std::wstring(s.begin(), s.end()); }

// GetSocketLocation / GetSocketRotation(FName) with the name as its raw 8 bytes -- made once on the tick, so
// the render thread never builds an FName. The FVector / FRotator return lands at offset 8 (Rig.cpp's
// call_socket_location documents the layout).
bool socket_vec(API::UObject* comp, const wchar_t* fn, uint64_t fname, Vec3* out) {
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    std::memcpy(p, &fname, sizeof(fname));
    comp->call_function(fn, p);
    const auto* d = reinterpret_cast<const double*>(p + 8);
    if (!std::isfinite(d[0]) || !std::isfinite(d[1]) || !std::isfinite(d[2])) return false;
    *out = Vec3{(float)d[0], (float)d[1], (float)d[2]};
    return true;
}

std::wstring short_path(API::UObject* o) {   // "BP_ScorpionCannonVehicleActor_C_12.SkeletalMeshComponent0"
    const std::wstring f = o->get_full_name();
    const size_t pl = f.find(L"PersistentLevel.");
    return pl != std::wstring::npos ? f.substr(pl + 16) : f;
}

// On success: the component, its object-array slot and the exact bone/socket name found. `tried` lists the
// meshes searched, for the log.
bool socket_resolve(const char* want, uintptr_t chassis, int32_t chassis_idx, API::UObject** comp, int32_t* idx,
                    std::wstring* found, std::wstring* tried) {
    std::string part, name = want;
    if (const char* sl = std::strchr(want, '/')) { part.assign(want, static_cast<size_t>(sl - want)); name = sl + 1; }
    auto trim = [](std::string& s) {
        while (!s.empty() && s.front() == ' ') s.erase(s.begin());
        while (!s.empty() && s.back() == ' ') s.pop_back();
    };
    trim(part); trim(name);
    if (name.empty()) return false;
    const std::wstring wname = widen_ascii(name), lname = lower_w(wname), lpart = lower_w(widen_ascii(part));

    // Where you sit: the distance the nearby parts are ordered by.
    Vec3 me{};
    bool have_me = false;
    if (auto* pawn = API::get()->get_local_pawn(0)) have_me = call_ret_vec3(pawn, L"K2_GetActorLocation", &me);

    struct Cand { API::UObject* o; int32_t i; double d; };
    constexpr int kMax = 16;
    Cand cand[kMax];
    int nc = 0;
    auto* ch = reinterpret_cast<API::UObject*>(chassis);
    if (ch != nullptr && uobject_slot_valid(ch)) cand[nc++] = {ch, chassis_idx, -1.0};
    for (const ScanHit& h : s_res_chassis) {
        if (h.o == ch || !scan_hit_live(h)) continue;
        Vec3 w{};
        if (!call_ret_vec3(h.o, L"K2_GetComponentLocation", &w)) continue;
        const double d = have_me ? std::sqrt(((double)w.x - me.x) * ((double)w.x - me.x) + ((double)w.y - me.y) * ((double)w.y - me.y)
                                             + ((double)w.z - me.z) * ((double)w.z - me.z)) : 0.0;
        if (d > 1200.0) continue;                                      // not part of what you are sitting in
        if (nc == kMax && d >= cand[kMax - 1].d) continue;
        int at = (nc < kMax) ? nc++ : kMax - 1;
        while (at > 1 && cand[at - 1].d > d) { cand[at] = cand[at - 1]; --at; }   // [0] stays the chassis
        cand[at] = {h.o, h.i, d};
    }
    auto part_ok = [&](API::UObject* o) { return lpart.empty() || lower_w(o->get_full_name()).find(lpart) != std::wstring::npos; };
    for (int k = 0; k < nc; ++k)
        if (part_ok(cand[k].o)) *tried += (tried->empty() ? L"" : L", ") + short_path(cand[k].o);

    const API::FName fname = make_fname(wname.c_str());
    for (int k = 0; k < nc; ++k) {                                     // the exact name, bone or socket
        if (!part_ok(cand[k].o) || !does_socket_exist(cand[k].o, fname)) continue;
        *comp = cand[k].o; *idx = cand[k].i; *found = wname;
        return true;
    }
    for (int k = 0; k < nc; ++k) {                                     // else a bone whose name contains it
        if (!part_ok(cand[k].o)) continue;
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        cand[k].o->call_function(L"GetNumBones", p);
        const int32_t nb = *reinterpret_cast<int32_t*>(p);
        if (nb <= 0 || nb > 512) continue;
        for (int32_t b = 0; b < nb; ++b) {
            std::memset(p, 0, 16);
            *reinterpret_cast<int32_t*>(p) = b;
            cand[k].o->call_function(L"GetBoneName", p);
            const std::wstring bn = reinterpret_cast<API::FName*>(p + 4)->to_string();
            if (lower_w(bn).find(lname) == std::wstring::npos) continue;
            *comp = cand[k].o; *idx = cand[k].i; *found = bn;
            return true;
        }
    }
    return false;
}
} // namespace

void vehcam_game_tick_vehicle() {
    // Plugin.cpp's own state, through the bridge: the same objects under the same names.
    const auto& g_in_menu = *host::g_plugin_state.in_menu;
    const auto& g_last_dt = *host::g_plugin_state.last_dt;

    // The seat camera's always mode keeps the rendered eye off the body even unmounted.
    g_view_seat_always.store(g_cfg.veh_cam == 2, std::memory_order_relaxed);

    // THE RIDE SCAN'S WINDOW. A new stick-mode window makes any earlier walk's answer stale; leaving
    // stick mode abandons a walk nobody needs any more. Then carry a running walk on (~1.5 ms at most).
    // First in the tick, so the body hide and the chassis search below both see this tick's state.
    const bool stick_now = halo::g_stick_mode_active.load(std::memory_order_relaxed);
    {
        static bool s_stick_was = false;
        if (stick_now && !s_stick_was) ride_scan_new_window();
        if (!stick_now && s_stick_was) ride_scan_cancel();
        s_stick_was = stick_now;
        ride_scan_step();
    }

    // Vehicle work: the driver-body hide + hog hull resolve, then the wheel gesture and heading
    // publisher. Menus drop the hold like the holsters do.
    g_tick_stage = "vehicle_body";
    vehicle_body_update();
    if (g_in_menu.load()) vehicle_reset(); else vehicle_update(g_last_dt.load());
    // ROUTE A P0: find the vehicle's chassis when a RIDE starts (stick mode + the mount flag, never a
    // cutscene -- veh_ride_update), from the ride scan's walk, and retry every ~3 s while unresolved,
    // ten times a ride at most; cleared on exit so the next vehicle re-resolves. The render callbacks
    // read the resolved mesh's transform fresh.
    //
    // Resolved whenever the camera system runs in a vehicle (vehtp), WHICHEVER camera is selected: the
    // chassis is how the vehicle is identified, and a first-person entry still needs it to step back.
    // Then the camera file picks the vehicle's cameras and applies left Y (camera) / left X (tethering
    // mode) (VehCamSelect.cpp), which is what sets g_veh_tp_active.
    {
        static bool s_sys_was = false;
        static bool s_tp_was = false;
        static uint32_t s_ch_wait = 0, s_ch_tries = 0, s_ch_retry = 0, s_ch_seat_tries = 0;
        static uint32_t s_far_ticks = 0, s_far_repicks = 0;   // the hull check, below
        const bool ride = veh_ride_update(stick_now);
        // The kill switch (Ctrl+HOME) ends the ride for the cameras too: the selection clears, and with it
        // our eye, the view override, the aim write, the input lanes and the decoupled-pitch hold.
        const bool sys = ride && g_cfg.veh_tp && g_cfg.enabled;
        bool resolved_now = false;
        if (sys && !s_sys_was) {
            g_tp_chassis_ptr.store(0, std::memory_order_relaxed);
            g_tp_chassis_idx.store(-1, std::memory_order_relaxed);
            s_ch_tries = 1; s_ch_retry = 0; s_ch_seat_tries = 0;
            s_far_ticks = 0; s_far_repicks = 0;
            s_ch_wait = ride_scan_request(/*force_new=*/false);
            g_veh_turn_yaw.store(0.0f, std::memory_order_relaxed);    // each ride starts facing its heading
        }
        if (sys && g_tp_chassis_ptr.load(std::memory_order_relaxed) == 0) {
            if (s_ch_wait != 0) {
                if (ride_scan_done() >= s_ch_wait) {
                    s_ch_wait = 0;
                    if (pick_tp_chassis()) resolved_now = true;
                    else s_ch_retry = s_seats_proven ? 8 : 90;          // ~0.25 s to re-ask the seats; ~3 s to re-walk
                }
            } else if (s_ch_retry > 0 && --s_ch_retry == 0) {
                if (s_seats_proven && s_ch_seat_tries < 12) {
                    // The game has not named your seat yet: ask it again from this walk -- no new walk --
                    // for about three seconds, then fall through to a fresh one.
                    ++s_ch_seat_tries;
                    if (pick_tp_chassis()) resolved_now = true;
                    else s_ch_retry = 8;
                } else if (s_ch_tries < 10) {
                    ++s_ch_tries;
                    s_ch_wait = ride_scan_request(/*force_new=*/true);
                } else {
                    API::get()->log_info("[Halo-CampE-UEVR] VEHTP: no vehicle mesh found in 10 tries this ride -- "
                                         "the game's own camera stays");
                }
            }
        }
        if (!sys && s_sys_was) {
            g_tp_chassis_ptr.store(0, std::memory_order_relaxed);
            g_tp_chassis_idx.store(-1, std::memory_order_relaxed);
            g_tp_seat_valid.store(false, std::memory_order_relaxed);
            s_ch_wait = 0; s_ch_retry = 0;
            s_seat = SeatFix{};
            s_tp_match_name.clear();
        }
        s_sys_was = sys;

        // THE SEAT, asked again every ~2 s while riding: the game may name it a moment after you get in, and
        // a seat swap changes it. A newly named or different seat re-picks the vehicle -- and with it the
        // camera entry and the frame. An answer that goes missing keeps the one you have.
        if (sys && g_tp_chassis_ptr.load(std::memory_order_relaxed) != 0) {
            static uint32_t s_seat_tick = 0;
            if ((++s_seat_tick % 64u) == 0u && seat_refl_ready()) {
                API::UObject* me[2] = {};
                double pp[3] = {};
                SeatFix now;
                if (seat_me(me, pp) != 0 && seat_resolve(me, 2, pp, &now)
                    && (!s_seat.valid || now.actor != s_seat.actor || now.seat != s_seat.seat || now.bits != s_seat.bits)) {
                    API::get()->log_info("[Halo-CampE-UEVR] VEHSEAT: your seat is %s now -- choosing the vehicle again",
                                         s_seat.valid ? "different" : "named");
                    pick_tp_chassis(&now);
                }
            }
        }

        const uintptr_t cp = g_tp_chassis_ptr.load(std::memory_order_relaxed);
        vehcam_select_tick(sys, cp, s_tp_match_name, s_seat.valid ? s_seat.bits : 0);
        // The tick the chassis resolved, the camera was selected just now -- AFTER this tick's body hide
        // ran. Run it again so a seat camera's first frames are not drawn from inside your own body.
        if (resolved_now) driver_hide_update();

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
                // THE HULL CHECK (bc24's anchor validation): you sit IN the vehicle, so you are never far from
                // its mesh. More than 8 m for half a second means this mesh is not the one carrying you -- a
                // wrong pick, or a vehicle swapped under you -- so the vehicle is chosen again, at most 5
                // times a ride, and said.
                const double apart = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);   // not `far`: a Windows.h macro
                if (apart > 800.0) {
                    if (++s_far_ticks == 16 && s_far_repicks < 5) {
                        ++s_far_repicks;
                        API::get()->log_info("[Halo-CampE-UEVR] VEHTP: you are %.0f cm from the vehicle's mesh -- it is not "
                                             "the one carrying you; choosing the vehicle again (%u of 5)", apart, s_far_repicks);
                        pick_tp_chassis();
                    }
                } else {
                    s_far_ticks = 0;
                }
            }
        }

        // THE PLAYER'S HEAD (origin "playerhead"), only while such a camera is up. The Chief's Body comes
        // from the ride scan's finished walk (the body hider's rule: nearest the rider); his head bone's
        // name is found once by listing the skeleton's bones. Each tick: the head's world position, turned
        // into his ACTOR's frame, so the eye can rebuild it against that frame live -- a turret turns the
        // Chief even where it does not turn its own mesh. The same Body and bone serve "hideHead"
        // (head_hide_update, below), whatever the camera's origin.
        {
            static TrackedObject s_rider;
            static uint32_t      s_rider_wait = 0;
            static std::wstring  s_head_bone;
            static bool          s_head_tried = false;
            const VehActiveCam hc = veh_active_cam();
            const bool want_head = sys && cp != 0 && hc.valid
                                && hc.origin == static_cast<uint8_t>(vehcampresets::Origin::Head);
            // Only while OUR camera draws: a first-person entry hides the whole body its own way.
            const bool want_hide = sys && cp != 0 && hc.valid && hc.hide_head
                                && g_veh_tp_active.load(std::memory_order_relaxed);
            API::UObject* hide_body = nullptr;
            if (!sys) {
                s_rider.reset(); s_rider_wait = 0;
                g_tp_rider_ptr.store(0, std::memory_order_relaxed);
                g_tp_rider_idx.store(-1, std::memory_order_relaxed);
                g_tp_head_valid.store(false, std::memory_order_relaxed);
            } else if (want_head || want_hide) {
                auto* body = s_rider.get_checked(L"SkeletalMeshComponent");
                if (body == nullptr) {
                    if (s_rider_wait == 0) s_rider_wait = ride_scan_request(/*force_new=*/false);
                    if (ride_scan_done() >= s_rider_wait) {
                        s_rider_wait = 0;
                        if (const ScanHit* h = ride_scan_pick_driver_body()) {
                            s_rider.set_at(h->o, h->i);
                            body = h->o;
                            g_tp_rider_ptr.store((uintptr_t)h->o, std::memory_order_relaxed);
                            g_tp_rider_idx.store(h->i, std::memory_order_relaxed);
                            if (s_head_bone.empty()) s_head_tried = false;   // a new body: look again
                        }
                    }
                }
                bool ok = false;
                if (body != nullptr) {
                    hide_body = body;
                    if (!s_head_tried) {
                        s_head_tried = true;
                        int32_t nb = 0;
                        s_head_bone = find_head_bone(body, &nb);
                        if (s_head_bone.empty())
                            API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: no head bone among the Chief's %d bones -- "
                                                 "playerhead cameras use the seat instead", nb);
                        else
                            API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: the player's head is bone \"%ls\" (of %d)",
                                                 s_head_bone.c_str(), nb);
                    }
                    auto* actor = body->get_outer();
                    Vec3 hw{}, bl{}, al{}, ar{};
                    if (want_head && !s_head_bone.empty() && actor != nullptr && call_socket_location(body, s_head_bone.c_str(), &hw)
                        && call_ret_vec3(body, L"K2_GetComponentLocation", &bl)
                        && call_ret_vec3(actor, L"K2_GetActorLocation", &al)
                        && call_ret_vec3(actor, L"K2_GetActorRotation", &ar)) {
                        double AX[3], AY[3], AZ[3];
                        rot_axes(ar.x, ar.y, ar.z, AX, AY, AZ);
                        const double d[3] = { (double)hw.x - al.x, (double)hw.y - al.y, (double)hw.z - al.z };
                        const double hx = AX[0] * d[0] + AX[1] * d[1] + AX[2] * d[2];
                        const double hy = AY[0] * d[0] + AY[1] * d[1] + AY[2] * d[2];
                        const double hz = AZ[0] * d[0] + AZ[1] * d[1] + AZ[2] * d[2];
                        // A bone name the mesh does not have returns the component's OWN origin (Rig.cpp's
                        // trap), so a "head" sitting on the body's origin is a failed lookup; and a head 4 m
                        // from the actor is not a head either.
                        const double sx = (double)hw.x - bl.x, sy = (double)hw.y - bl.y, sz = (double)hw.z - bl.z;
                        if (sx * sx + sy * sy + sz * sz > 1.0 && hx * hx + hy * hy + hz * hz < 400.0 * 400.0) {
                            g_tp_head_x.store((float)hx, std::memory_order_relaxed);
                            g_tp_head_y.store((float)hy, std::memory_order_relaxed);
                            g_tp_head_z.store((float)hz, std::memory_order_relaxed);
                            ok = true;
                        }
                    }
#if HALO_VR_DEV
                    // UNMEASURED: whether the Chief's ACTOR turns with a turret whose own mesh does not. The
                    // camera follows his actor's frame on the belief that it does. Every ~2 s while a
                    // playerhead camera is up: his actor's rotation beside his Body's and the chassis', the
                    // head offset, and whether his actor is the local pawn (the "seat" origin reads the pawn).
                    if (g_cfg.veh_probe && want_head) {
                        static uint32_t s_hdlog = 0;
                        if ((s_hdlog++ % 64u) == 0u) {
                            static TrackedObject s_hd_ch;
                            static uintptr_t s_hd_ch_raw = 0;
                            if (cp != s_hd_ch_raw) {
                                s_hd_ch_raw = cp;
                                s_hd_ch.set_at(reinterpret_cast<API::UObject*>(cp), g_tp_chassis_idx.load(std::memory_order_relaxed));
                            }
                            auto* chm = s_hd_ch.get_checked(L"SkeletalMeshComponent");
                            Vec3 br{}, cr2{};
                            const bool b_ok = call_ret_vec3(body, L"K2_GetComponentRotation", &br);
                            const bool c_ok = chm != nullptr && call_ret_vec3(chm, L"K2_GetComponentRotation", &cr2);
                            API::get()->log_info(
                                "[Halo-CampE-UEVR] VEHHEAD: bone=\"%ls\" ok=%d actor(p=%.1f y=%.1f r=%.1f) body(y=%.1f)%s "
                                "chassis(y=%.1f)%s head=(%.0f %.0f %.0f) pawn=%d",
                                s_head_bone.c_str(), (int)ok, (double)ar.x, (double)ar.y, (double)ar.z,
                                (double)br.y, b_ok ? "" : "?", (double)cr2.y, c_ok ? "" : "?",
                                (double)g_tp_head_x.load(std::memory_order_relaxed),
                                (double)g_tp_head_y.load(std::memory_order_relaxed),
                                (double)g_tp_head_z.load(std::memory_order_relaxed),
                                (int)(actor != nullptr && actor == API::get()->get_local_pawn(0)));
                        }
                    }
#endif
                }
                g_tp_head_valid.store(ok, std::memory_order_relaxed);
            } else {
                g_tp_head_valid.store(false, std::memory_order_relaxed);
            }
            head_hide_update(want_hide, hide_body, s_rider.index, s_head_bone);
        }

        // A SOCKET ORIGIN (socket_resolve, above): found when a camera asks for one -- a new name or a new
        // vehicle -- and retried every ~2 s, five times, while not found (a turret actor may stream in a
        // moment after you sit). The eye reads the socket's transform live; until it is found the vehicle's
        // origin stands in, and the log says what was searched.
        {
            static std::string   s_sock_want;
            static uintptr_t     s_sock_cp = 0;
            static TrackedObject s_sock_comp;
            static uint32_t      s_sock_tries = 0;
            static ULONGLONG     s_sock_at = 0;
            const VehActiveCam sc = veh_active_cam();
            const bool want_sock = sys && cp != 0 && sc.valid && sc.socket[0] != 0
                                && sc.origin == static_cast<uint8_t>(vehcampresets::Origin::Socket);
            if (!want_sock) {
                if (!s_sock_want.empty()) {
                    s_sock_want.clear(); s_sock_cp = 0; s_sock_comp.reset();
                    g_tp_sock_ptr.store(0, std::memory_order_relaxed);
                    g_tp_sock_idx.store(-1, std::memory_order_relaxed);
                }
            } else {
                if (s_sock_want != sc.socket || s_sock_cp != cp) {
                    s_sock_want = sc.socket; s_sock_cp = cp;
                    s_sock_comp.reset(); s_sock_tries = 0; s_sock_at = 0;
                    g_tp_sock_ptr.store(0, std::memory_order_relaxed);
                    g_tp_sock_idx.store(-1, std::memory_order_relaxed);
                }
                if (s_sock_comp.ptr != nullptr && s_sock_comp.get() == nullptr) {   // gone with its vehicle
                    g_tp_sock_ptr.store(0, std::memory_order_relaxed);
                    g_tp_sock_idx.store(-1, std::memory_order_relaxed);
                }
                const ULONGLONG now = GetTickCount64();
                if (s_sock_comp.get() == nullptr && s_sock_tries < 5 && (s_sock_tries == 0 || now - s_sock_at >= 2000)) {
                    s_sock_at = now;
                    ++s_sock_tries;
                    API::UObject* comp = nullptr;
                    int32_t idx = -1;
                    std::wstring found, tried;
                    if (socket_resolve(sc.socket, cp, g_tp_chassis_idx.load(std::memory_order_relaxed), &comp, &idx, &found, &tried)
                        && idx >= 0) {
                        s_sock_comp.set_at(comp, idx);
                        const API::FName fn = make_fname(found.c_str());
                        uint64_t raw = 0;
                        std::memcpy(&raw, &fn, sizeof(int32_t) * 2);
                        g_tp_sock_fname.store(raw, std::memory_order_relaxed);
                        g_tp_sock_idx.store(idx, std::memory_order_relaxed);
                        g_tp_sock_gen.fetch_add(1, std::memory_order_relaxed);
                        g_tp_sock_ptr.store(reinterpret_cast<uintptr_t>(comp), std::memory_order_release);
                        API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: origin \"%s\" is \"%ls\" on %ls", sc.socket, found.c_str(),
                                             short_path(comp).c_str());
                    } else {
                        API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: origin \"%s\" not found (try %u of 5) on: %ls -- the "
                                             "vehicle's origin stands in", sc.socket, s_sock_tries,
                                             tried.empty() ? L"no meshes" : tried.c_str());
                    }
                }
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
            // The eye no longer runs to retire the vehicle-facing marker, so do it here.
            g_vmk_ray_valid.store(false, std::memory_order_relaxed);
            xrlayer_retire_quad(XRLAYER_SLOT_VEHAIM);
        }
        s_tp_was = tp_on;

        // YOUR HEAD BACK ON THE CAMERA'S POINT on a camera change (vehcamrecenterpos), the leash-ON half: the
        // leash lets the head sit anywhere inside its radius of the anchor, so the change puts the standing
        // origin right under the head -- no offset left over from the last camera. (With the leash off the
        // eye re-captures the head's offset instead; with a zero radius this changes nothing.)
        {
            static uint32_t s_place_tick = 0;
            const uint32_t pg = veh_active_cam().place_gen;
            if (pg != s_place_tick) {
                s_place_tick = pg;
                if (tp_on && g_cfg.hmd_leash) {
                    Vec3 hp{}; Quat hq{};
                    const auto hi = API::VR::get_hmd_index();
                    if (hi >= 0 && get_pose(hi, &hp, &hq, /*use_aim=*/false)) {
                        const UEVR_Vector3f n{hp.x, hp.y, hp.z};
                        API::VR::set_standing_origin(n);
                    }
                }
            }
        }

        // VEHCAMPATH (release-safe, bc24's pattern): name each fallback the eye takes, when it starts and
        // when it ends -- after 8 ticks (~0.25 s) of the new path, so a one-frame blip does not flood the log.
        {
            static uint8_t  s_path_said = 0, s_path_pending = 0;
            static uint32_t s_path_ticks = 0;
            if (!tp_on) {
                s_path_said = s_path_pending = 0; s_path_ticks = 0;
                g_tp_path.store(0, std::memory_order_relaxed);
            } else {
                const uint8_t p = g_tp_path.load(std::memory_order_relaxed);
                if (p != s_path_pending) { s_path_pending = p; s_path_ticks = 0; }
                if (p != s_path_said && ++s_path_ticks >= 8) {
                    s_path_said = p;
                    if (p == 0)
                        API::get()->log_info("[Halo-CampE-UEVR] VEHCAMPATH: back on the camera's own path");
                    else
                        API::get()->log_info("[Halo-CampE-UEVR] VEHCAMPATH:%s%s%s%s",
                                             (p & kTpPathEngine) ? " the vehicle's transform cannot be read -- the game's own camera shows;" : "",
                                             (p & kTpPathHeadToSeat) ? " no head measured -- this playerhead camera sits at your seat;" : "",
                                             (p & kTpPathToOrigin) ? " no seat measured -- this camera sits at the vehicle's origin;" : "",
                                             (p & kTpPathSocketToVehicle) ? " its socket is not found -- this camera sits at the vehicle's origin;" : "");
                }
            }
        }
    }

    // RIGHT-STICK TURN (vehstick=1): with motion aim freeing the stick (vehaim), the right stick X turns
    // your VIEW -- a smooth turn, the in-vehicle counterpart of turning on foot. The eye adds
    // g_veh_turn_yaw to the view yaw only: for a camera whose view does not follow the vehicle's yaw
    // that view is independent of the vehicle, and the turn pivots on your head without moving the
    // anchor (only an offset that rides the VIEW swings round with it, by choice). It HOLDS
    // where you leave it (vehorbitreturn=0, default). The first cut orbited the boom and eased back at
    // 60 deg/s, which in-headset read as "the stick lerps my head yaw back to the front of the
    // vehicle". Zeroed on every new ride. The game's own stick-look is masked by the aim write.
    //
    // THE ONLY WRITER of g_veh_turn_yaw. A camera change's recenter (vehcamrecenter) is worked out by
    // the eye, which has the view's frame; it is ADOPTED here, then acknowledged -- in that order, so
    // an eye that sees the acknowledgement also sees the turn that includes it.
    {
        static uint32_t s_reset_ack = 0;
        const uint32_t rseq = g_veh_turn_reset_seq.load(std::memory_order_acquire);
        const bool adopt = (rseq != s_reset_ack);
        float turn = adopt ? g_veh_turn_reset_val.load(std::memory_order_relaxed)
                           : g_veh_turn_yaw.load(std::memory_order_relaxed);
        const float dt = g_last_dt.load();
        const bool can_turn = veh_tp_motion_aim_active() && g_cfg.veh_stick_mode == 1;
        const float sx = can_turn ? host::g_plugin_state.raw_stick_x->load() : 0.0f;
        const float dz = 0.15f;
        if (std::fabs(sx) > dz) {
            const float s = (sx - (sx > 0.0f ? dz : -dz)) / (1.0f - dz);   // rescale past the deadzone
            turn += s * g_cfg.veh_orbit_rate * dt;
            // remainder(), not a subtract loop: past ~2^33 a float minus 360 is the same float, and the
            // loop never ends. A non-finite turn starts over at the vehicle's front.
            turn = std::isfinite(turn) ? std::remainder(turn, 360.0f) : 0.0f;
        } else if (g_cfg.veh_orbit_return > 0.0f) {
            const float step = g_cfg.veh_orbit_return * dt;
            if (turn > step) turn -= step; else if (turn < -step) turn += step; else turn = 0.0f;
        }
        g_veh_turn_yaw.store(turn, std::memory_order_release);
        if (adopt) {
            s_reset_ack = rseq;
            g_veh_turn_reset_ack.store(rseq, std::memory_order_release);
        }
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
            const int32_t ridx = veh_aim_hand_index();
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
                    const float aim_y = std::atan2(ay, ax) * RAD2DEG;
                    const float aim_p = std::asin(clampf(az / al, -1.0f, 1.0f)) * RAD2DEG;
                    // The sim thread writes these straight into Blam's aim record: never a NaN or an inf
                    // (clampf passes NaN through, and asin of it is NaN).
                    if (al > 100.0f && std::isfinite(aim_y) && std::isfinite(aim_p)) {
                        g_veh_aim_yaw.store(aim_y, std::memory_order_relaxed);
                        g_veh_aim_pitch.store(aim_p, std::memory_order_relaxed);
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

    // THE VEHICLE-FACING MARKER'S DEPTH (aimMarker). The ray the eye published -- from the aim's origin
    // along the vehicle's forward -- traced here for what it meets, on the reticule's rule: the hit's
    // range, held on a miss (sky has no range), the far end before any hit this ride. GAME THREAD:
    // hit_trace is reflection. The eye places and draws the ring every frame from this range.
    {
        const VehActiveCam mk = veh_active_cam();
        const bool want = g_veh_tp_active.load(std::memory_order_relaxed)
                       && halo::g_stick_mode_active.load(std::memory_order_relaxed)
                       && mk.valid && mk.aim_marker && g_vmk_ray_valid.load(std::memory_order_relaxed);
        if (want) {
            static TrackedObject s_tpc_mk;
            static uintptr_t s_tpc_mk_raw = 0;
            const uintptr_t cp = g_tp_chassis_ptr.load(std::memory_order_relaxed);
            if (cp != s_tpc_mk_raw) {
                s_tpc_mk_raw = cp;
                s_tpc_mk.set_at(reinterpret_cast<API::UObject*>(cp), g_tp_chassis_idx.load(std::memory_order_relaxed));
            }
            auto* ch = (cp != 0) ? s_tpc_mk.get_checked(L"SkeletalMeshComponent") : nullptr;
            Vec3 cl{};
            if (ch != nullptr && call_ret_vec3(ch, L"K2_GetComponentLocation", &cl)) {
                const Vec3 o{ cl.x + g_vmk_ox.load(std::memory_order_relaxed),
                              cl.y + g_vmk_oy.load(std::memory_order_relaxed),
                              cl.z + g_vmk_oz.load(std::memory_order_relaxed) };
                const Vec3 d{ g_vmk_fx.load(std::memory_order_relaxed), g_vmk_fy.load(std::memory_order_relaxed),
                              g_vmk_fz.load(std::memory_order_relaxed) };
                const float far_cm = g_cfg.veh_aim_far;
                API::UObject* ignore[2] = {}; int ni = 0;
                if (auto* pawn = API::get()->get_local_pawn(0)) ignore[ni++] = pawn;
                if (auto* owner = ch->get_outer()) ignore[ni++] = owner;
                Vec3 hit{};
                if (far_cm > 1.0f && hit_trace(o, Vec3{o.x + d.x * far_cm, o.y + d.y * far_cm, o.z + d.z * far_cm},
                                               ignore, ni, &hit)) {
                    const float hx = hit.x - o.x, hy = hit.y - o.y, hz = hit.z - o.z;
                    g_vmk_range.store(std::sqrt(hx * hx + hy * hy + hz * hz), std::memory_order_relaxed);
                }
            }
        } else if (!g_veh_tp_active.load(std::memory_order_relaxed)) {
            g_vmk_range.store(0.0f, std::memory_order_relaxed);   // the next ride measures afresh
        }
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
    // resolved game-side from the vehicle your seat belongs to), read at render rate so it tracks the
    // moving vehicle; the view yaw is published here for the view override. g_veh_tp_active is set only
    // during a RIDE (stick mode + the Blam mount flag + no cutscene, veh_ride_update). Head free-look
    // composes on top via UEVR.
    if (g_veh_tp_active.load(std::memory_order_relaxed) && halo::g_stick_mode_active.load(std::memory_order_relaxed)) {
        // ONE COMPUTATION PER FRAME (bc24's latch). Eye 0 builds the view base; eye 1 reuses it. The tick
        // values it reads (the turn, the collision fraction, the seat and head offsets) can change between
        // the two eye callbacks, and two bases built from different ticks are a disparity error -- bc24
        // measured ~47 cm of it on his camera before he latched. Both eyes take the same base: UEVR adds
        // the eye separation after this callback. Only when eye 1 follows within a few milliseconds: when
        // the eyes render on different frames (AFR), a latched base would be a frame stale, so each
        // computes its own.
        static bool          s_latch_fresh = false;
        static double        s_latch[3] = {0.0, 0.0, 0.0};
        static LARGE_INTEGER s_latch_t{}, s_qpf{};
        if (s_qpf.QuadPart == 0) QueryPerformanceFrequency(&s_qpf);
        if (index != 0 && s_latch_fresh) {
            s_latch_fresh = false;
            LARGE_INTEGER now{};
            QueryPerformanceCounter(&now);
            if ((now.QuadPart - s_latch_t.QuadPart) * 1000 < s_qpf.QuadPart * 3) {   // within 3 ms
                if (is_double) {
                    auto* p = reinterpret_cast<UEVR_Vector3d*>(position);
                    p->x = s_latch[0]; p->y = s_latch[1]; p->z = s_latch[2];
                } else {
                    position->x = (float)s_latch[0]; position->y = (float)s_latch[1]; position->z = (float)s_latch[2];
                }
                g_view_pos_x = (float)s_latch[0]; g_view_pos_y = (float)s_latch[1]; g_view_pos_z = (float)s_latch[2];
                return;
            }
        }
        s_latch_fresh = false;
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
            // A world-scale change mid-ride re-captures too: c0 is in UE cm at the scale it was read at. So does
            // every CAMERA CHANGE (vehcamrecenterpos: a camera, a tethering mode, a seat, the controls): with the
            // leash off you may have leaned or walked anywhere in the room, and a new camera should start with
            // your head on its point, not wherever the last one left it (the user, 2026-09-27). The leash-on
            // half is the tick's: it snaps the standing origin onto the head (vehcam_game_tick_vehicle).
            static float s_arm_cmpm = 0.0f;
            static uint32_t s_place_seen = 0;
            const float cmpm = halo::veh_cm_per_m();
            const bool place = ac.place_gen != s_place_seen;
            s_place_seen = ac.place_gen;
            // A RECENTRE OF YOUR PLAY SPACE -- the headset's own (holding the Quest's Meta button) or UEVR's.
            // UEVR answers OpenXR's reference-space change by resetting the standing origin (OpenXR.cpp:
            // REFERENCE_SPACE_CHANGE_PENDING -> wants_reset_origin), and with the leash off the captured
            // offset then describes a room that is gone: you sit off the camera's point by however far you
            // had been from the old origin (the user, 2026-09-27: "slightly offset" after a system reset).
            // With the leash off nothing of ours moves the origin under this camera, so a move of it IS that
            // recentre: re-capture, and your head goes back on the camera's point. UEVR's own Recenter View
            // moves no origin: it writes only its rotation offset, which the capture applies as UEVR does
            // (veh_rotation_offset) -- so a turn of THAT is a recentre too.
            static UEVR_Vector3f s_so_seen{};
            static Quat s_ro_seen{0.0f, 0.0f, 0.0f, 1.0f};
            static bool s_so_have = false;
            bool origin_moved = false;
            if (!leash) {
                const auto so = API::VR::get_standing_origin();
                const Quat ro = veh_rotation_offset();
                if (s_so_have) {
                    const float dx = so.x - s_so_seen.x, dy = so.y - s_so_seen.y, dz = so.z - s_so_seen.z;
                    origin_moved = dx * dx + dy * dy + dz * dz > 0.01f * 0.01f    // 1 cm: a reset, not float noise
                                || std::fabs(quat_dot(ro, s_ro_seen)) < 0.99999f;  // ~0.5 deg of offset turn
                }
                s_so_seen = so;
                s_ro_seen = ro;
                s_so_have = true;
            } else {
                s_so_have = false;   // the leash moves it every tick: not a signal while it runs
            }
            const bool recapture_head = rearm || place || origin_moved || leash != s_arm_leash
                                     || std::fabs(cmpm - s_arm_cmpm) > 0.05f;
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
                        // UEVR's rotation offset first, in VR space: the head as UEVR draws it.
                        const Vec3 rel = quat_rotate(veh_rotation_offset(), Vec3{hp.x - so.x, hp.y - so.y, hp.z - so.z});
                        s_c0[0] = -(double)rel.z * rs; // room -> UE: X = -z, Y = x, Z = y
                        s_c0[1] =  (double)rel.x * rs;
                        s_c0[2] =  (double)rel.y * rs;
                    }
                }
            }
            // THE VEHICLE'S FRAME NOW: R_mesh . C -- its forward, right and up as world directions.
            double VF[3], VR[3], VU[3];
            vehicle_axes(s_C, MX, MY, MZ, VF, VR, VU);
            // A PLAYERHEAD CAMERA RIDES THE CHIEF. His actor's frame, read now like the chassis, replaces the
            // vehicle's for everything this camera tracks -- so a turret that turns him but not its own mesh
            // turns this view -- and its origin is his head, rebuilt from the offset the tick measured in
            // that same frame (two clocks: an offset crosses the boundary, never a position). Both eyes see
            // one Chief through the frame latch above. Until the head is known, the seat stands in.
            bool   head_ok = false;
            double head_w[3] = { 0.0, 0.0, 0.0 };
            if (ac.origin == static_cast<uint8_t>(vehcampresets::Origin::Head)
                && halo::g_tp_head_valid.load(std::memory_order_relaxed)) {
                static TrackedObject s_rb;
                static uintptr_t s_rb_raw = 0;
                const uintptr_t rp = halo::g_tp_rider_ptr.load(std::memory_order_relaxed);
                if (rp != s_rb_raw) {
                    s_rb_raw = rp;
                    s_rb.set_at(reinterpret_cast<API::UObject*>(rp), halo::g_tp_rider_idx.load(std::memory_order_relaxed));
                }
                auto* rb = (rp != 0) ? s_rb.get_checked(L"SkeletalMeshComponent") : nullptr;
                auto* ra = (rb != nullptr) ? rb->get_outer() : nullptr;
                Vec3 al{}, ar{};
                if (ra != nullptr && call_ret_vec3(ra, L"K2_GetActorLocation", &al)
                    && call_ret_vec3(ra, L"K2_GetActorRotation", &ar)) {
                    double HX[3], HY[3], HZ[3];
                    rot_axes(ar.x, ar.y, ar.z, HX, HY, HZ);
                    const double hx = halo::g_tp_head_x.load(std::memory_order_relaxed);
                    const double hy = halo::g_tp_head_y.load(std::memory_order_relaxed);
                    const double hz = halo::g_tp_head_z.load(std::memory_order_relaxed);
                    const double a[3] = { (double)al.x, (double)al.y, (double)al.z };
                    for (int k = 0; k < 3; ++k) {
                        head_w[k] = a[k] + hx * HX[k] + hy * HY[k] + hz * HZ[k];
                        VF[k] = HX[k]; VR[k] = HY[k]; VU[k] = HZ[k];
                    }
                    head_ok = true;
                }
            }
            // A SOCKET CAMERA RIDES ITS BONE -- a turret that turns on its own, like the Scorpion's cannon. Its
            // live transform, read now like the chassis, gives the origin point and the frame this camera
            // tracks. A bone's local axes follow no convention, so the frame is snapped to the socket's
            // principal axes once (capture_vehicle_frame): up the one nearest world up, forward the one nearest
            // where the GAME's camera looks -- down the gun's line, which is where the barrel points once it
            // has caught up with the aim -- re-taken on every camera change, so a reset re-squares it too.
            bool   sock_ok = false;
            double sock_w[3] = { 0.0, 0.0, 0.0 };
            if (ac.origin == static_cast<uint8_t>(vehcampresets::Origin::Socket)) {
                static TrackedObject s_sk;
                static uintptr_t s_sk_raw = 0;
                static double    s_Cs[9] = {1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0};
                static uint32_t  s_cs_gen = 0xFFFFFFFFu;
                const uintptr_t sp = halo::g_tp_sock_ptr.load(std::memory_order_acquire);
                if (sp != s_sk_raw) {
                    s_sk_raw = sp;
                    s_sk.set_at(reinterpret_cast<API::UObject*>(sp), halo::g_tp_sock_idx.load(std::memory_order_relaxed));
                }
                auto* skc = (sp != 0) ? s_sk.get() : nullptr;
                const uint64_t sfn = halo::g_tp_sock_fname.load(std::memory_order_relaxed);
                Vec3 sl{}, sr{};
                if (skc != nullptr && socket_vec(skc, L"GetSocketLocation", sfn, &sl) && socket_vec(skc, L"GetSocketRotation", sfn, &sr)) {
                    double SX[3], SY[3], SZ[3];
                    rot_axes(sr.x, sr.y, sr.z, SX, SY, SZ);
                    const uint32_t sg = halo::g_tp_sock_gen.load(std::memory_order_relaxed);
                    if (sg != s_cs_gen || rearm || place) {
                        s_cs_gen = sg;
                        double ref = std::atan2(VF[1], VF[0]) * R2D;   // the vehicle's heading, else
                        if (rotation != nullptr) {
                            const double gy = is_double ? reinterpret_cast<UEVR_Rotatord*>(rotation)->yaw : (double)rotation->yaw;
                            if (std::isfinite(gy)) ref = gy;           // the game camera: down the gun's line
                        }
                        bool snapped = false;
                        capture_vehicle_frame(SX, SY, SZ, ref, s_Cs, &snapped);
                    }
                    double F[3], R[3], U[3];
                    vehicle_axes(s_Cs, SX, SY, SZ, F, R, U);
                    for (int k = 0; k < 3; ++k) { VF[k] = F[k]; VR[k] = R[k]; VU[k] = U[k]; }
                    sock_w[0] = sl.x; sock_w[1] = sl.y; sock_w[2] = sl.z;
                    sock_ok = true;
                }
            }
            const double heading = std::atan2(VF[1], VF[0]) * R2D;
            if (ac.rot_yaw) s_frozen_yaw = (float)heading;   // keep the freeze current -> seamless when you step to one that holds
            // RECENTER ON A CAMERA CHANGE (vehcamrecenter): getting in, left Y or left X, turns the view so
            // WHAT AIMS THE VEHICLE points where the VEHICLE IS AIMING. The source: the aim hand while your hand
            // aims this vehicle (veh_aim_hand_index), else your head. The target:
            //  - a camera that TURNS WITH the vehicle (rotationTracking yaw): the frame it turns with (VF: the
            //    chassis, or the Chief for playerhead). Nowhere else does it rest. The hand is read in a view
            //    that turns with the vehicle, so any gap between them is a standing offset the vehicle chases
            //    forever, the view turning along with it. Recentring the HEAD left the hand off by however it
            //    happened to be held, and the vehicle kept turning that way (the user, 2026-09-26);
            //  - a camera that holds its heading: where the vehicle is already aimed (the ray aim's direction),
            //    so stepping cameras swings nothing -- a turret stays where it points. Until the ray aim has a
            //    solution (stick controls, or the frame the left stick click hands the aim to your hand), the
            //    GAME's own camera stands for it: it follows the aim within 1-2 deg (measured 2026-09-26);
            //  - on getting in (the ride's first frame): the vehicle's forward.
            // Worked out here, where the view's own frame is, on the new camera's first frame (eye 0), with
            // this camera's frame before any turn; the tick adopts it into the turn it owns. Yaw only, about
            // your head, through the right stick's own turn.
            static uint32_t s_recenter_seen = 0;
            if (index == 0 && ac.recenter_gen != s_recenter_seen) {
                s_recenter_seen = ac.recenter_gen;
                double F0[3], R0[3], U0[3];
                tracked_frame(VF, VR, VU, ac.rot_yaw, ac.rot_pitch, ac.rot_roll, (double)s_frozen_yaw, 0.0, F0, R0, U0);
                const bool hand_aims = halo::veh_tp_motion_aim_selected();   // this frame may be the eye's first
                double tgt[3] = { VF[0], VF[1], VF[2] };
                if (!ac.rot_yaw && !rearm) {
                    double ay = 0.0, ap = 0.0;
                    bool have_aim = false;
                    if (hand_aims && halo::g_veh_aim_valid.load(std::memory_order_relaxed)) {
                        ay = (double)halo::g_veh_aim_yaw.load(std::memory_order_relaxed) * D2R;
                        ap = (double)halo::g_veh_aim_pitch.load(std::memory_order_relaxed) * D2R;
                        have_aim = true;
                    } else if (rotation != nullptr) {
                        // The game camera, as handed to this callback before the view override.
                        const double gy = is_double ? reinterpret_cast<UEVR_Rotatord*>(rotation)->yaw : (double)rotation->yaw;
                        const double gp = is_double ? reinterpret_cast<UEVR_Rotatord*>(rotation)->pitch : (double)rotation->pitch;
                        // Looking nearly straight up or down, its heading is not a direction.
                        if (std::isfinite(gy) && std::isfinite(gp) && std::cos(gp * D2R) > 0.2) {
                            ay = gy * D2R; ap = gp * D2R; have_aim = true;
                        }
                    }
                    if (have_aim) {
                        tgt[0] = std::cos(ap) * std::cos(ay);
                        tgt[1] = std::cos(ap) * std::sin(ay);
                        tgt[2] = std::sin(ap);
                    }
                }
                Vec3 sp{}; Quat sq{};
                const int32_t hand = hand_aims ? veh_aim_hand_index() : -1;
                bool have = hand >= 0 && get_pose(hand, &sp, &sq, /*use_aim=*/true);
                if (!have) {
                    const auto hi = API::VR::get_hmd_index();
                    have = hi >= 0 && get_pose(hi, &sp, &sq, /*use_aim=*/false);
                }
                if (have) {
                    // Where it points as UEVR DRAWS it: its rotation offset composed in first (VR space).
                    const Quat sd = quat_mul(veh_rotation_offset(), sq);
                    const double src = vehcammath::pose_yaw_deg(sd.x, sd.y, sd.z, sd.w);
                    halo::g_veh_turn_reset_val.store((float)vehcammath::recenter_turn_deg(tgt, F0, R0, src),
                                                     std::memory_order_relaxed);
                    halo::g_veh_turn_reset_seq.fetch_add(1, std::memory_order_release);
                }
            }
            // The right-stick turn (vehstick=1) -- or a recenter the tick has not adopted yet.
            const uint32_t rseq = halo::g_veh_turn_reset_seq.load(std::memory_order_acquire);
            const uint32_t rack = halo::g_veh_turn_reset_ack.load(std::memory_order_acquire);
            const float turn = (rseq != rack) ? halo::g_veh_turn_reset_val.load(std::memory_order_relaxed)
                                              : g_veh_turn_yaw.load(std::memory_order_relaxed);
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
            if (sock_ok) {
                // THE SOCKET (above), as an offset from the chassis like every origin here.
                org[0] = sock_w[0] - (double)cloc.x;
                org[1] = sock_w[1] - (double)cloc.y;
                org[2] = sock_w[2] - (double)cloc.z;
            } else if (head_ok) {
                // THE PLAYER'S HEAD (above), as an offset from the chassis like every origin here.
                org[0] = head_w[0] - (double)cloc.x;
                org[1] = head_w[1] - (double)cloc.y;
                org[2] = head_w[2] - (double)cloc.z;
            } else if ((ac.origin == static_cast<uint8_t>(vehcampresets::Origin::Seat)
                        || ac.origin == static_cast<uint8_t>(vehcampresets::Origin::Head))
                       && halo::g_tp_seat_valid.load(std::memory_order_relaxed)) {
                const double sx = halo::g_tp_seat_x.load(std::memory_order_relaxed);
                const double sy = halo::g_tp_seat_y.load(std::memory_order_relaxed);
                const double sz = halo::g_tp_seat_z.load(std::memory_order_relaxed);
                for (int k = 0; k < 3; ++k) org[k] = sx * MX[k] + sy * MY[k] + sz * MZ[k];
            }
            if (index == 0) {
                const bool wants_head = ac.origin == static_cast<uint8_t>(vehcampresets::Origin::Head);
                const bool wants_seat = wants_head || ac.origin == static_cast<uint8_t>(vehcampresets::Origin::Seat);
                uint8_t path = 0;
                if (wants_head && !head_ok) path |= halo::kTpPathHeadToSeat;
                if (wants_seat && !head_ok && !halo::g_tp_seat_valid.load(std::memory_order_relaxed))
                    path |= halo::kTpPathToOrigin;
                if (ac.origin == static_cast<uint8_t>(vehcampresets::Origin::Socket) && !sock_ok)
                    path |= halo::kTpPathSocketToVehicle;
                halo::g_tp_path.store(path, std::memory_order_relaxed);
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
                // THE CAMERA'S LEASH ("leashMin" / "leashMax"): how far your HEAD may move from the camera's
                // point, on the offset's own axes (LF / LR / LU), in cm -- for seats tuned to tight
                // tolerances, where a 6DoF lean would put your eyes through the canopy or into the gun. Past a
                // limit the view stops following your head that way (the world moves with you), so you
                // cannot clip into what surrounds the seat. The head's displacement from the point is exactly
                // what UEVR adds to this base: R(view) . (swizzle(ro . (hmd - so)) x scale - c0), c0 being
                // where it stood at the last camera change (0 with the head leash on, which keeps the head
                // on the standing origin). Clamped HERE, before the base is written or published, so the
                // hands, the aim ray and the reticle stamp all stand on the same base as the view.
                if (ac.leashed) {
                    Vec3 lp{}; Quat lq{};
                    const auto li = API::VR::get_hmd_index();
                    if (li >= 0 && get_pose(li, &lp, &lq, /*use_aim=*/false)) {
                        const auto so = API::VR::get_standing_origin();
                        const double rs = (double)halo::veh_cm_per_m();
                        const Vec3 rel = quat_rotate(veh_rotation_offset(), Vec3{lp.x - so.x, lp.y - so.y, lp.z - so.z});
                        // room -> UE (X = -z, Y = x, Z = y), less where the head was captured.
                        const double u[3] = { -(double)rel.z * rs - s_c0[0], (double)rel.x * rs - s_c0[1],
                                              (double)rel.y * rs - s_c0[2] };
                        double hd[3], sh[3];
                        for (int k = 0; k < 3; ++k) hd[k] = u[0] * X[k] + u[1] * Y[k] + u[2] * Z[k];
                        vehcammath::leash_shift(hd, LF, LR, LU, ac.leash_min, ac.leash_max, sh);
                        ecx -= sh[0]; ecy -= sh[1]; ecz -= sh[2];
                    }
                }
            }
            if (is_double) { auto* p = reinterpret_cast<UEVR_Vector3d*>(position); p->x = ecx; p->y = ecy; p->z = ecz; }
            else { position->x = (float)ecx; position->y = (float)ecy; position->z = (float)ecz; }
            g_view_pos_x = (float)ecx; g_view_pos_y = (float)ecy; g_view_pos_z = (float)ecz;
            if (index == 0) {   // the latch eye 1 takes (above)
                s_latch[0] = ecx; s_latch[1] = ecy; s_latch[2] = ecz;
                QueryPerformanceCounter(&s_latch_t);
                s_latch_fresh = true;
            }
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
                const int32_t ridx = veh_aim_hand_index();
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

            // THE VEHICLE AIM MARKER (vehmarker + the camera file's "aimMarker"): a ring where the VEHICLE IS
            // AIMING. Its heading is the vehicle's own -- the frame a turning camera follows (VF: the chassis,
            // or the Chief for playerhead) -- at the pitch the game aims at, from the game's own chase camera,
            // the line the vehicle's guns converge on (vehaimorigin's finding). Once the vehicle has turned to
            // where your hand points it sits around the crosshair; while it is still turning, the gap is how
            // far behind it is. It is also exactly where a camera that turns with the vehicle comes to rest,
            // which is what the recenter lines your hand up with. The game camera's own line of sight was
            // tried first and showed no lag at all: it follows the aim we write to within a degree or two
            // (gcam against aimw in the vehprobe line), while the chassis trailed it by 10-25 deg through
            // turns (gcam against the mesh yaw; 2,242 samples, 2026-09-26). Placed and sized on the vehicle
            // reticule's rules (pulled back off the surface, apparent size held) times vehmarkersize, at the
            // range the tick traced along this same ray. Eye 0 only; the layer re-anchors it for both eyes.
            if (index == 0) {
                static bool s_mk_was = false;
                double gp = 0.0;
                const bool have_rot = rotation != nullptr;
                if (have_rot) gp = is_double ? reinterpret_cast<UEVR_Rotatord*>(rotation)->pitch : rotation->pitch;
                // Not over a menu: posed every frame, the ring never goes stale on its own (the crosshair's
                // rule, veh_tp_reticle_stamp_owns); a menu retires it below and the next frame out re-poses it.
                const bool in_menu = host::g_plugin_state.in_menu->load(std::memory_order_relaxed);
                if (g_cfg.xr_layer && g_cfg.veh_marker && ac.aim_marker && have_rot && !in_menu) {
                    const double o[3] = { ogx, ogy, ogz };
                    const double cpp = std::cos(gp * D2R), hr = heading * D2R;
                    const double MF[3] = { cpp * std::cos(hr), cpp * std::sin(hr), std::sin(gp * D2R) };
                    halo::g_vmk_ox.store((float)(o[0] - (double)cloc.x), std::memory_order_relaxed);
                    halo::g_vmk_oy.store((float)(o[1] - (double)cloc.y), std::memory_order_relaxed);
                    halo::g_vmk_oz.store((float)(o[2] - (double)cloc.z), std::memory_order_relaxed);
                    halo::g_vmk_fx.store((float)MF[0], std::memory_order_relaxed);
                    halo::g_vmk_fy.store((float)MF[1], std::memory_order_relaxed);
                    halo::g_vmk_fz.store((float)MF[2], std::memory_order_relaxed);
                    halo::g_vmk_ray_valid.store(true, std::memory_order_relaxed);
                    float range = halo::g_vmk_range.load(std::memory_order_relaxed);
                    if (!(range > 1.0f)) range = (g_cfg.veh_aim_far > 1.0f) ? g_cfg.veh_aim_far : 10000.0f;
                    const float r = range - std::fmin(g_cfg.aim_reticule_surface_off, range * 0.5f);
                    const Vec3 t{ (float)(o[0] + MF[0] * r), (float)(o[1] + MF[1] * r), (float)(o[2] + MF[2] * r) };
                    // Apparent size: the vehicle reticule's (distance from the eye / aimreticuledist, times
                    // aimreticulescaleveh), through the reticule's own world-size formula, times its own size.
                    const double dx = (double)t.x - ecx, dy = (double)t.y - ecy, dz = (double)t.z - ecz;
                    const float d = (float)std::sqrt(dx * dx + dy * dy + dz * dz);
                    const float ref = (g_cfg.aim_reticule_dist > 1.0f) ? g_cfg.aim_reticule_dist : 500.0f;
                    const float cm = g_cfg.aim_widget_draw * g_cfg.aim_widget_scale * (d / ref)
                                   * g_cfg.aim_reticule_scale_veh * g_cfg.xr_layer_size
                                   * (g_cfg.veh_marker_size > 0.05f ? g_cfg.veh_marker_size : 0.05f);
                    const auto mk_anchor = host::g_plugin_state.layer_anchor;
                    halo::xrlayer_notice_quad(halo::XRLAYER_SLOT_VEHAIM, mk_anchor(halo::XRLAYER_SLOT_VEHAIM, t), cm,
                                              /*hold_cm=*/0.0f, /*priority=*/1);
                    // Upright like the reticule when xrlayerroll=0; otherwise it faces you as it is.
                    if (g_cfg.xr_layer_roll == 0) {
                        Vec3 fwd{}, up{};
                        if (halo::xrlayer_view_basis(&fwd, &up))
                            halo::xrlayer_set_quad_orientation(halo::XRLAYER_SLOT_VEHAIM, fwd, Vec3{0.0f, 0.0f, 1.0f});
                    } else {
                        halo::xrlayer_clear_quad_orientation(halo::XRLAYER_SLOT_VEHAIM);
                    }
                    s_mk_was = true;
                } else if (s_mk_was) {
                    s_mk_was = false;
                    halo::g_vmk_ray_valid.store(false, std::memory_order_relaxed);
                    halo::xrlayer_retire_quad(halo::XRLAYER_SLOT_VEHAIM);
                }
            }
        } else if (index == 0) {
            g_vcd.wrote = false;
            halo::g_tp_chassis_yaw_valid.store(false, std::memory_order_relaxed);
            halo::g_tp_eye_valid.store(false, std::memory_order_relaxed);
            halo::g_tp_ncam_valid.store(false, std::memory_order_relaxed);
            halo::g_tp_path.store(halo::kTpPathEngine, std::memory_order_relaxed);   // fails closed: the game's camera
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
            // fitted from the log rather than assumed. Render-thread logging: compiled into dev builds only
            // (a config flag is not a sufficient guard, CLAUDE.md).
#if HALO_VR_DEV
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
#endif
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
            // selected for is identified (during a ride, which already required the mount flag). The mount
            // flag alone needs stick mode beside it -- a seat IS stick mode -- so a misread flag on some
            // other build can never take over the on-foot view.
            const bool vc_gate = vcm != 0 && halo::g_unit_pvalid.load(std::memory_order_relaxed)
                && (vcm == 2 || veh_fp_selected()
                    || (halo::g_unit_mounted.load(std::memory_order_relaxed)
                        && halo::g_stick_mode_active.load(std::memory_order_relaxed)));
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
#if HALO_VR_DEV   // render-thread logging: dev builds only
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
#else
                    (void)mounted_now; (void)fsum;
#endif
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
#if HALO_VR_DEV   // render-thread logging: dev builds only
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
#endif
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
#if HALO_VR_DEV   // render-thread logging: dev builds only
                        if (g_cfg.veh_log && path != g_vcd.last_path) {
                            static const char* const kPath[] = {"none", "rigid", "rigid-hold", "chase", "synth"};
                            API::get()->log_info("[Halo-CampE-UEVR] VEHCAMPATH: %s -> %s (stale=%.0fms hullspeed=%.0fcm/s speed=%.2fwu/s)",
                                                 g_vcd.last_path >= 0 ? kPath[g_vcd.last_path] : "start", kPath[path],
                                                 (double)g_vcd.stale_ms, g_vcd.hspeed,
                                                 (double)halo::g_veh_speed.load(std::memory_order_relaxed));
                        }
#endif
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
                // output, and the untouched game yaw. Render-thread logging: dev builds only.
#if HALO_VR_DEV
                if (g_cfg.veh_log) {
                    static uint32_t n = 0;
                    if ((n++ % 90u) == 0u)
                        API::get()->log_info("[Halo-CampE-UEVR] VEHVIEW: valid=%d travel=%.1f hemi=%.0f eased=%.1f gameyaw=%.1f pos=(%.2f %.2f)",
                                             (int)valid, travel, s_veh_hemi, s_veh_cur, game_yaw,
                                             halo::g_unit_px.load(std::memory_order_relaxed),
                                             halo::g_unit_py.load(std::memory_order_relaxed));
                }
#endif
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
            // WRITE SURVIVAL (vehlog): the rendered eye against the seat -- or OUR camera's view base --
            // written this frame. The gap is the head's offset from it -- under 2 m in any real play space.
            // More than 3 m means the camera was replaced between our write and the render. A render-thread
            // diagnostic: compiled into dev builds only.
#if !HALO_VR_DEV
            (void)index; (void)ex; (void)ey; (void)ez;
#else
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
#endif
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
