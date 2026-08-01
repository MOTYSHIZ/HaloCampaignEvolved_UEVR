// The VR gameplay plugin for Halo: Campaign Evolved (built on UEVR).
//
// Ships as halo_vr.dll, reading halo_vr.cfg from the profile root. Motion-controller aim is the
// centrepiece, but the scope is the whole VR feel of the game, not just aim.
//
// WHAT IT DOES
//   * MOTION-CONTROLLER AIM -- the game's OWN aim tracks your right controller, so the gun, the
//     reticle and the SHOTS all agree. Still the centrepiece; the control law itself lives in
//     MotionAimControl.cpp, driven from update() here.
//   * VIEW LOCK -- aiming steers the game's camera, so without this the controller would drag your
//     head. The rendered view is held still while aim moves underneath it.
//   * TURNING -- snap/smooth turn, needed precisely because the view is locked.
//   * FIRST-PERSON WEAPON RIG -- the weapon mesh follows your hand, driven through the rig
//     component's RELATIVE transform.
//   * WORLD-SPACE RETICULE -- a ring at the true aim point, hosting the game's own crosshair.
//   * MOVEMENT RE-ORIENTATION -- stick-forward follows where you LOOK, independent of the gun.
//   * MENU-AWARE INPUT -- gameplay remaps stand down in menus; B acts as Back.
//   * CALIBRATION + LIVE CONFIG -- pose-match and aim-ray calibration, persisted; every tunable
//     re-read from disk about every 2 s, including the kill switch.
//
// HOW THE AIM WORKS
//   * Reads the Blam camera (APlayerController::ControlRotation) as RAW MEMORY at a validated
//     offset. UEVR's reflection access-violates on this title's Blam-backed objects, so raw reads
//     behind sanity gates are the instrument, and the driver fails closed if the value looks wrong.
//   * Steers it by synthesizing right-stick XInput, so the game's own aim pipeline does all the
//     work. on_xinput_get_state receives uint32_t* retval, so the plugin also reports a CONNECTED
//     pad itself and does not depend on an Oculus/Virtual Desktop/ViGEm bus being installed.
//
// CONTROL-LAW FACTS THIS ENCODES
//   * SIGN: +stick.x INCREASES yaw on this title. Inverted, the loop drives AWAY from the target
//     and parks at the antipode -- which is easy to misread as an excessive-gain limit cycle.
//   * DEADZONE: output below ~0.24 does NOTHING (0.05..0.20 -> 0 deg/s, 0.35 -> 4.3,
//     0.90 -> ~137). A plain proportional law emits sub-deadzone values near convergence and stalls
//     with a large standing error. Hence the floor.
//   * PARALLAX: Halo spawns first-person shots at the CAMERA. Aiming the ray parallel to the hand
//     ray leaves a permanent head-to-hand miss. Steer the head-origin ray THROUGH the point the hand
//     ray reaches at crosshair distance instead.
//
// LAYOUT
// This file holds the plugin class, the callbacks, and the features not yet split out. Shared code
// lives in namespace halo, pulled in unqualified below so call sites read the same either way:
//   Math.hpp/.cpp             quaternions, rotators, the calibration solve. No plugin state, no API.
//   Config.hpp/.cpp           the Config struct, g_cfg, both .cfg files (tunables + kill switch).
//   UeObject.hpp/.cpp         TrackedObject (recycle-safe handles), FName resolution, class names.
//   Rig.hpp/.cpp              the first-person weapon rig: gun/arms follow the hand.
//   Reticule.hpp/.cpp         both reticules (our mesh ring, and one hosting the game's crosshair).
//   MotionAimControl.hpp/.cpp the aim control loop: reading Blam's aim, and the law that steers it.
//
// What is left HERE is update() -- the per-tick loop that drives all of the above in order -- plus
// the stereo view lock, turning, movement re-orientation, HUD discovery/follow and menu detection,
// which are read by the render and XInput callbacks that live here too.
//
// BUILD
// scripts\build.ps1 compiles every .cpp under src\ against the UEVR C++ plugin SDK and produces the
// DLL for the profile's plugins\ directory; -Deploy installs it. Adding a file needs no script edit.
// See COMPILING.md.
//
// !! Only THIS file may include uevr/Plugin.hpp -- it DEFINES the plugin entry points, so a second
// translation unit including it fails to link with duplicate DllMain/uevr_plugin_initialize.
// Modules include uevr/API.hpp instead.
//
// CREDITS
//   * praydog's UEVR -- the injection framework and plugin API this is built on; its
//     controller-aim path is the model for the sightline construction and the view lock.
//   * Pande's OblivionVR (Profile/scripts/VRHud.lua) -- the deferred widget-component
//     construction sequence that makes the world-space reticule render.
//   * pancreations' Halo-MCC-VR (MIT) -- prior art for controller aim on native Blam titles: the
//     stick-shaping law, the fake-pad presence trick and the hide-native-reticle doctrine.
//   * jbusfield's uevrlib -- its reticule module is the model for the world-space reticule,
//     including the BoundsScale fix that stops small widget components being frustum-culled.

#include <Windows.h>
#include <Xinput.h>
#include <cstdint>
#include <cmath>
#include <string>
// API.hpp uses std::string_view without including it, so the TU must provide it first.
#include <string_view>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "uevr/Plugin.hpp"

// Pure maths (quaternions, rotators, the calibration solve). Free of plugin state by design --
// see the note at the top of Math.hpp before adding to it.
#include "Math.hpp"

// Live config + calibration persistence. Defines g_cfg, which nearly everything below reads.
#include "Config.hpp"

// UE object/name helpers: TrackedObject (recycle-safe handles), FName resolution, class names.
#include "UeObject.hpp"

// First-person weapon rig: the gun/arms follow the hand via RELATIVE component transforms.
#include "Rig.hpp"

// Aim reticule: our own mesh reticule, plus one hosting the game's own reticle widget.
#include "Reticule.hpp"

// The aim control loop: Halo's own aim is steered to follow the controller via synthesized stick.
#include "MotionAimControl.hpp"

// Shipped version, logged at startup so a bug report identifies the build it came from. There is no
// other build marker in the DLL, so this is the only thing tying a log.txt to a release.
// BUMP THIS WITH THE RELEASE TAG -- CI publishes on `v*`, and the two are not linked automatically.
#define HALO_VR_VERSION "0.1.2"

using namespace uevr;

namespace {

// The maths lives in namespace halo; pull it in unqualified so call sites read the same as they
// did when this was one file.
using namespace halo;


// CONTROL_ROTATION_OFFSET is a constexpr in MotionAimControl.hpp.

// The PlayerController the reference was captured against. The frontend and a mission use
// DIFFERENT controller objects (BP_FrontendPlayerController_C vs BP_MeteoritePlayerController_C),
// and a map load makes another one. A reference latched against the wrong controller carries a
// bogus offset into gameplay -- aim wrong by exactly the frontend-to-mission yaw difference,
// which looks like a calibration problem rather than a bug. So: re-capture whenever the
// controller changes.
std::atomic<void*>  g_ref_pc{nullptr};
std::atomic<bool>  g_have_ref{false};

// UEVR's world rotation offset at the moment the reference was captured.
//
// WHY THIS EXISTS -- the difference between us and stock UEVR aim. UEVR's own aim methods do not
// need an anchor at all: they WRITE ControlRotation absolutely from the controller pose, and UEVR
// owns the VR-to-game correspondence because it renders the camera. We cannot write aim on this
// title (Blam re-stamps it), so we can only nudge it with a rate actuator -- which means we must
// hold an empirical mapping between "controller yaw in VR space" and "Blam yaw in game space".
// That mapping is the reference, and anything that rotates the VR world underneath it invalidates
// it: UEVR recentering, and snap turn.
//
// Snap turn is the sharp case. It rotates the rendered view but NOT Blam's yaw, so the controller
// yaw jumps while game aim does not. Without this check the loop would read that as a genuine
// error and drive aim by the snap amount -- turning the player twice. Re-anchoring on the offset
// change makes a snap turn a no-op for aim, which is the correct behaviour.
std::atomic<float> g_ref_rot_x{0.0f}, g_ref_rot_y{0.0f}, g_ref_rot_z{0.0f}, g_ref_rot_w{1.0f};

// The offset WE last wrote for the view lock. Needed because with view_lock on we rewrite the
// rotation offset every tick, so "the offset changed" can no longer mean "something external
// changed it" -- without this the re-anchor check would trigger on our own writes every frame and
// the reference would never hold. Compare against this, not against the captured reference.
std::atomic<float> g_self_rot_x{0.0f}, g_self_rot_y{0.0f}, g_self_rot_z{0.0f}, g_self_rot_w{1.0f};
std::atomic<bool>  g_self_rot_valid{false};

// Written by the tick, read by the XInput callback. Two scalars, nothing else shared.
std::atomic<float> g_out_rx{0.0f}, g_out_ry{0.0f};

// g_aim_law_armed / _ridx / _pc are defined in MotionAimControl.cpp (see MotionAimControl.hpp).
std::atomic<bool>  g_driving{false};

std::atomic<uint32_t> g_ticks{0}, g_xhits{0};
std::atomic<float> g_dbg_err_yaw{0.0f}, g_dbg_err_pitch{0.0f};
std::atomic<float> g_dbg_ctrl_yaw{0.0f}, g_dbg_aim_yaw{0.0f};
// Yaw we are cancelling in VR space this tick. Logged so the view lock can be verified WITHOUT a
// headset: it must track the game's yaw change with the opposite sign and equal magnitude.
std::atomic<float> g_dbg_lock_deg{0.0f};

// Yaw the game has gained since the reference. Written by the tick, read by the stereo-view
// callback (render thread) which subtracts it from the rendered rotation. One scalar, nothing else
// shared -- same discipline as the XInput callback, and for the same reason.
std::atomic<float> g_view_lock_deg{0.0f};

// The rendered view yaw as UEVR hands it to us, and what we leave it as. Recorded from the stereo
// callback and logged from the tick, because nothing else reports the final render rotation --
// the game camera and the raw HMD device pose both move differently from the rendered view.
// These two numbers are what make the view lock verifiable rather than assumed.
std::atomic<float> g_dbg_view_in{0.0f}, g_dbg_view_out{0.0f};

// The rendered view yaw we hold. Written by the render callback (priming) and the tick (turning);
// read by the render callback. The player's accumulated turn that pairs with it is g_turn_offset,
// which lives in MotionAimControl.cpp because the aim setpoint folds it in.
std::atomic<float> g_locked_view_yaw{0.0f};
std::atomic<bool>  g_lock_primed{false};

// JUDDER METRIC -- measures FRAME-TO-FRAME CHANGE in the yaw we actually render:
// |viewOut(n) - viewOut(n-1)| at render rate, excluding deliberate turning. That is the wobble a
// player feels. NOT |viewIn - locked| -- that is the size of the correction, which is huge by
// design (Blam's camera can be far from the pinned view) and says nothing about wobble.
std::atomic<float> g_judder_max{0.0f};
std::atomic<float> g_prev_view_out{0.0f};
std::atomic<bool>  g_have_prev_out{false};

// The player's RAW right-stick X, sampled in on_xinput_get_state BEFORE we overwrite it with the
// aim output. With the view locked, waving the controller no longer turns you, so the player needs
// a turn control -- and their own right-stick deflection is the natural one. We are already
// intercepting that stick, so we can consume it for turning and still write our aim value out.
std::atomic<float> g_raw_stick_x{0.0f};

// Degrees to rotate the LEFT stick by so movement follows the player's facing rather than the
// game's camera. Computed on the game thread, consumed in the XInput hook.
std::atomic<float> g_move_rot_deg{0.0f};
// Counts frames where the stick was actually rewritten. Distinguishes "the correction is wrong"
// from "the correction never ran" -- a null result looks identical either way from the headset.
std::atomic<uint32_t> g_move_applied{0};

// The FINAL rendered view yaw, in game space, straight from UEVR's post-stereo callback: the game
// camera with UEVR's rotation offset and the HMD's own rotation already composed in. This is the
// authoritative "where is the player looking" value -- reconstructing it from parts was guesswork.
// Smoothed target angular velocity, in game degrees/sec. Feeds both the feedforward term and the
// rate-error damping.
std::atomic<float> g_ff_rate_yaw{0.0f}, g_ff_rate_pitch{0.0f};
// Filtered measurement of how fast the aim is ACTUALLY moving; only the damping term uses it.
std::atomic<float> g_aim_rate_yaw{0.0f}, g_aim_rate_pitch{0.0f};

std::atomic<float> g_render_view_yaw{0.0f};
// Pitch of the same finished view. Only the yaw was published before, because movement is the only
// consumer that needs a heading -- the reticle needs both axes or it tracks left/right and ignores
// the far more visible up/down error.
std::atomic<float> g_render_view_pitch{0.0f};
std::atomic<bool>  g_have_render_yaw{false};
std::atomic<float> g_dbg_hmd_yaw{0.0f}, g_dbg_view_a{0.0f}, g_dbg_view_b{0.0f};

// ---------------------------------------------------------------- MOVE PROBE
// Closes the loop on movement instead of reasoning about it: every movement-frame symptom
// ("depends on head direction", "walks in a circle") is consistent with several different sign
// errors. So: record what the player asked for, what we handed the game, and WHERE THE CAMERA
// ACTUALLY WENT. The residual (actual movement yaw - facing yaw) is the only number that matters,
// and it cannot be argued with.
std::atomic<float> g_raw_lx{0.0f}, g_raw_ly{0.0f};   // player's stick, before our rotation
std::atomic<float> g_out_lx{0.0f}, g_out_ly{0.0f};   // what the game received

// Damped copy of the aim yaw, used ONLY by the movement frame. The aim loop itself keeps using the
// raw value -- aiming must stay crisp; only locomotion wants the settled version.
std::atomic<float> g_move_aim_smooth{0.0f};
std::atomic<bool>  g_move_aim_primed{false};

// True while right-stick-up is holding the d-pad shift, so the movement rotation stands down.
std::atomic<bool> g_dpad_shift_active{false};

// True when the stick is driving a MENU rather than the player. Set from the PlayerController class
// (the frontend uses its own) and from "no FP rig", which covers loads and transitions.
std::atomic<bool> g_in_menu{true};

// True while STICK MODE has the motion stack stood down -- vehicle seats, cutscenes, death, the
// post-load window: the sticks reach the game untouched and the game camera owns the rendered
// view, exactly like gamepad play. Computed by the detector in update() (game thread), consumed
// by both hooks. Config.hpp's stick-mode block carries the full doctrine.
std::atomic<bool> g_stick_mode{false};

// The pawn the FP rig was last resolved under -- "on foot", as far as motion aim is concerned.
// Tracked, not raw: pawns here are pooled shells. Game thread only.
TrackedObject g_stick_pawn_base;

// Pose-match calibration button state. The geometry lives further down, with the quaternion
// helpers it depends on.
std::atomic<bool> g_calib_held{false};
std::atomic<bool> g_calib_start{false};
std::atomic<bool> g_calib_finish{false};
// Raw pad-button state, published by the XInput hook and consumed on the game thread. The hook
// deliberately does NOT edge-detect: the keyboard source is invisible from there, and detecting
// edges on two threads against one state would race.
std::atomic<bool> g_pad_calib_down{false};

// ---------------------------------------------------------------- AIM CALIBRATION
// The mirror of the mesh calibration, and deliberately a SEPARATE gesture: one adjusts where the
// weapon sits in your hand, the other adjusts where the game aims for a given hand direction.
// Bundled into one gesture, adjusting the mesh silently re-calibrates aim.
//
// Hold: the aim actuator goes silent, so the reticle stops chasing your hand and stays where it is.
// The weapon keeps tracking normally, so you can see what you are pointing.
// Release: the hand-to-aim reference is re-captured, binding "controller pointing here" to
// "game aiming there" -- which is the entire content of that reference.
std::atomic<bool> g_aimcal_held{false};
std::atomic<bool> g_kill_held{false};   // Ctrl+kill_key edge state (see the kill switch below)
std::atomic<bool> g_aimcal_start{false};
std::atomic<bool> g_aimcal_finish{false};
// Set on release so the NEXT reference capture measures a new offset rather than restoring the
// saved one -- the same capture path serves both, and this is what distinguishes them.
std::atomic<bool> g_aimcal_capture{false};
std::atomic<bool>  g_snap_latched{false};
std::atomic<float> g_last_dt{0.033f};   // engine tick delta, for smooth turn


// ---------------------------------------------------------------- PERIODIC-WORK TIMING
// update() gates several expensive operations behind tick counters, and a periodic microstutter
// was reported with three of them at similar periods. Nothing here had ever been frame-timed, so
// the suspects could only be ranked by reading the code -- which is how you fix the wrong one.
//
// Each site is timed ONLY on the ticks it actually does its work (the timer sits inside the gate,
// not around it), so `n` is a real count of runs and `mean` is the true cost of a run rather than
// an average over the ticks that early-returned.
//
// Stats are RESET every report window. A latching max would show the worst hitch since injection
// forever, which cannot show whether a fix worked.
enum PerfSite { PERF_CFG = 0, PERF_RETICLE, PERF_RIG, PERF_COUNT };
const char* const kPerfName[PERF_COUNT] = { "load_config   ", "reticle_rescan", "resolve_rig   " };

struct PerfStat {
    double   max_ms = 0.0;
    double   sum_ms = 0.0;
    uint32_t n      = 0;
};
PerfStat g_perf[PERF_COUNT];

// QPC ticks -> milliseconds. The frequency is fixed for the life of the process, so it is read once.
double perf_tick_ms() {
    static const double f = [] {
        LARGE_INTEGER q{};
        return QueryPerformanceFrequency(&q) && q.QuadPart != 0 ? 1000.0 / (double)q.QuadPart : 0.0;
    }();
    return f;
}

// Times from construction to end of scope and folds the result into one site's stat.
// Captures the enable flag at construction so a live config edit mid-scope cannot unbalance it.
struct PerfScope {
    PerfSite      site;
    LARGE_INTEGER t0{};
    bool          on;

    explicit PerfScope(PerfSite s) : site(s), on(g_cfg.perf_log) {
        if (on) QueryPerformanceCounter(&t0);
    }
    ~PerfScope() {
        if (!on) return;
        LARGE_INTEGER t1{};
        QueryPerformanceCounter(&t1);
        const double ms = (double)(t1.QuadPart - t0.QuadPart) * perf_tick_ms();
        PerfStat& p = g_perf[site];
        if (ms > p.max_ms) p.max_ms = ms;
        p.sum_ms += ms;
        ++p.n;
    }
    PerfScope(const PerfScope&) = delete;
    PerfScope& operator=(const PerfScope&) = delete;
};

// Dump and reset. Also prints the MEASURED tick rate: every period in this plugin is expressed in
// ticks, and the conversion to seconds was assumed (~30 Hz) and never verified -- if it is wrong,
// every documented interval is wrong with it.
void perf_report(uint32_t tick) {
    if (!g_cfg.perf_log) return;
    static uint32_t last = 0;
    if (tick - last < 600) return;
    last = tick;

    const float dt = g_last_dt.load();
    API::get()->log_info("[Halo-CampE-UEVR] PERF window=600 ticks dt=%.1fms (%.1f Hz tick)",
                         dt * 1000.0f, dt > 0.0f ? 1.0f / dt : 0.0f);
    for (int i = 0; i < PERF_COUNT; ++i) {
        PerfStat& p = g_perf[i];
        if (p.n == 0) {
            API::get()->log_info("[Halo-CampE-UEVR]   %s  (did not run)", kPerfName[i]);
        } else {
            API::get()->log_info("[Halo-CampE-UEVR]   %s  n=%u  max=%.3fms  mean=%.3fms  total=%.2fms",
                                 kPerfName[i], p.n, p.max_ms, p.sum_ms / (double)p.n, p.sum_ms);
        }
        p = PerfStat{};
    }
}


// Adaptive gain and shape() are in MotionAimControl.cpp -- they are part of the control law.

// ---------------------------------------------------------------- POSE-MATCH CALIBRATION
// Hold the calibrate button, physically move your controller onto the weapon you can see, release.
//
// WHY IT HAS TO FREEZE. The naive version cannot work: the weapon follows your hand, and your hand
// also steers the aim, which rotates the weapon's parent -- so moving to "match" the gun pushes the
// gun away and you chase it forever. While the button is held BOTH are frozen: the rig keeps its
// last written transform and the aim actuator is silenced, so the weapon is genuinely static in the
// world and can be overlaid.
//
// THE SOLVE. In mode 2 the weapon's world orientation is exactly q_ctrl * q_grip, so with
//   q_gun     = the frozen weapon's world orientation
//   q_release = the controller's orientation when you let go (now overlaying it)
// the trim that makes the weapon sit where your hand is, is one multiplication:
//   q_grip_new = inverse(q_release) * q_gun
// and because q_release * q_grip_new == q_gun by construction, the position solve reduces to
//   local_off_new = inverse(q_gun) * (off_frozen - pose_part)
// ---- 3x3 linear algebra, needed only by the two-sample pivot solve.




// Previous calibration sample, kept so the NEXT one can solve the pivot.
bool g_calib_have_prev = false;
Vec3 g_calib_prev_pose{0.0f, 0.0f, 0.0f};
Vec3 g_calib_prev_off{0.0f, 0.0f, 0.0f};
Quat g_calib_prev_gun{0.0f, 0.0f, 0.0f, 1.0f};

bool g_calib_valid = false;
Quat g_calib_gun_world{0.0f, 0.0f, 0.0f, 1.0f};   // weapon world orientation, frozen
Vec3 g_calib_off_world{0.0f, 0.0f, 0.0f};         // weapon world offset, frozen
Quat g_last_gun_world{0.0f, 0.0f, 0.0f, 1.0f};    // updated every driven tick
Vec3 g_last_off_world{0.0f, 0.0f, 0.0f};

// UE's FQuat::Rotator(), singularity handling included. Deriving pitch by taking asin() of a
// forward vector instead -- the obvious shortcut, and what the aim path can get away with --
// silently discards YAW and ROLL by construction, which the rig needs. It also reintroduces the
// +-90 degree wrap that a quaternion avoids.


// ---------------------------------------------------------------- HUD DISCOVERY
// One-shot dump of a class's property names, so the HUD's per-element visibility flags can be
// FOUND rather than guessed. External reflection explorers reject every object on this title
// ("Object no longer valid" -- the Blam reflection fault) while the plugin's own
// get_property_data works fine, so enumeration has to happen from in here.
//
// Target: MeteoriteHudVisibility, a component on BP_MeteoriteHUD_C. Its class exposes only
// Initialize / OnGameInfoChanged / OnUnitChanged as functions, so the per-element switches are
// almost certainly PROPERTIES rather than setters.
//
// TYPED property dump: printing one raw byte per property is meaningless for reference fields
// (an ObjectProperty's first byte is just the low byte of a pointer). FFieldClass::get_name()
// gives the property TYPE, so object references can be followed instead of guessed at.
void dump_class_properties(API::UObject* obj, const char* label, bool follow_objects) {
    if (obj == nullptr) return;
    auto* cls = obj->get_class();
    if (cls == nullptr) return;

    API::get()->log_info("[Halo-CampE-UEVR] ---- properties of %s (%s) ----",
                         label, narrow(class_name_of(obj)).c_str());

    int n = 0;
    for (auto* f = cls->get_child_properties(); f != nullptr; f = f->get_next()) {
        auto* fname = f->get_fname();
        if (fname == nullptr) continue;
        const std::wstring wpn = fname->to_string();
        const std::string  pn  = narrow(wpn);

        std::string type = "?";
        auto* fc = f->get_class();
        if (fc != nullptr) type = narrow(fc->get_name());

        void* data = obj->get_property_data<void>(wpn);
        if (data == nullptr || IsBadReadPtr(data, 1)) {
            API::get()->log_info("[Halo-CampE-UEVR]    %-40s %-18s <unreadable>", pn.c_str(), type.c_str());
        } else if (type == "ObjectProperty" || type == "WeakObjectProperty" ||
                   type == "SoftObjectProperty" || type == "ClassProperty") {
            // The payload is a UObject* (weak/soft carry it first as well). Follow it and print
            // what it actually points at -- that is how the live widget gets found.
            auto* target = *reinterpret_cast<API::UObject**>(data);
            if (target != nullptr && !IsBadReadPtr(target, sizeof(void*))) {
                const std::string tc = narrow(class_name_of(target));
                const std::string tn = (target->get_fname() != nullptr)
                                     ? narrow(target->get_fname()->to_string()) : "?";
                API::get()->log_info("[Halo-CampE-UEVR]    %-40s %-18s -> %s '%s' @%p",
                                     pn.c_str(), type.c_str(), tc.c_str(), tn.c_str(), (void*)target);
            } else {
                API::get()->log_info("[Halo-CampE-UEVR]    %-40s %-18s -> null", pn.c_str(), type.c_str());
            }
        } else if (type == "BoolProperty") {
            API::get()->log_info("[Halo-CampE-UEVR]    %-40s %-18s = %s", pn.c_str(), type.c_str(),
                                 (*reinterpret_cast<uint8_t*>(data) != 0) ? "true" : "false");
        } else if (type == "FloatProperty") {
            API::get()->log_info("[Halo-CampE-UEVR]    %-40s %-18s = %.3f", pn.c_str(), type.c_str(),
                                 *reinterpret_cast<float*>(data));
        } else if (type == "IntProperty" || type == "ByteProperty" || type == "EnumProperty") {
            API::get()->log_info("[Halo-CampE-UEVR]    %-40s %-18s = %d", pn.c_str(), type.c_str(),
                                 (int)*reinterpret_cast<uint8_t*>(data));
        } else {
            API::get()->log_info("[Halo-CampE-UEVR]    %-40s %-18s", pn.c_str(), type.c_str());
        }
        if (++n > 200) { API::get()->log_info("[Halo-CampE-UEVR]    ... truncated at 200"); break; }
    }
    API::get()->log_info("[Halo-CampE-UEVR] ---- end (%d properties) ----", n);
}

// Walk a widget's WidgetTree looking for a named child, logging everything seen. UMG stores the
// live hierarchy under WidgetTree->RootWidget, and a Panel's children live in an array we cannot
// model blindly -- so instead we scan the object array for widgets OUTERED to this widget, which is
// how UMG names constructed children.
API::UObject* find_widget_child(API::UObject* owner, const wchar_t* want_class, int depth = 0) {
    if (owner == nullptr) return nullptr;
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return nullptr;

    const int32_t n = arr->get_object_count();
    for (int32_t i = 0; i < n; ++i) {
        auto* o = arr->get_object(i);
        if (o == nullptr) continue;
        if (o->get_outer() != owner) continue;

        const std::wstring cn = class_name_of(o);
        const std::string nm = (o->get_fname() != nullptr) ? narrow(o->get_fname()->to_string()) : "?";
        API::get()->log_info("[Halo-CampE-UEVR]      %*schild: %-38s '%s' @%p",
                             depth * 2, "", narrow(cn).c_str(), nm.c_str(), (void*)o);

        if (cn.find(want_class) != std::wstring::npos) return o;

        // One level down: WidgetTree sits between the widget and its children.
        if (depth < 2 && (cn == L"WidgetTree" || cn.find(L"Panel") != std::wstring::npos ||
                          cn.find(L"Overlay") != std::wstring::npos || cn.find(L"Canvas") != std::wstring::npos)) {
            if (auto* hit = find_widget_child(o, want_class, depth + 1)) return hit;
        }
    }
    return nullptr;
}

// Find the live HUD visibility component and dump it once.
void dump_hud_visibility_once(uint32_t tick) {
    static bool done = false;
    if (done) return;

    // Throttled, and never during a load: a full object-array scan per tick during level
    // transitions (exactly when the array is being rebuilt) is a reliable crash.
    static uint32_t last = 0;
    if (tick - last < 120) return;
    last = tick;

    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;

    // Target the live HUD ACTOR, not the visibility component: the component holds a single
    // property, while the actor is what constructs and therefore REFERENCES the widget -- the
    // thing we cannot otherwise reach.
    const int32_t n = arr->get_object_count();
    for (int32_t i = 0; i < n; ++i) {
        auto* obj = arr->get_object(i);
        if (obj == nullptr) continue;
        if (class_name_of(obj).find(L"BP_MeteoriteHUD_C") == std::wstring::npos) continue;
        auto* cls = obj->get_class();
        if (cls != nullptr && obj == cls->get_class_default_object()) continue;

        done = true;
        dump_class_properties(obj, "BP_MeteoriteHUD_C (live actor)", true);

        // Anything outered to the HUD actor -- constructed widgets included.
        API::get()->log_info("[Halo-CampE-UEVR]   -- objects outered to the HUD actor --");
        find_widget_child(obj, L"WBP_FirstPersonReticle");

        // And the visibility component, dumped with types.
        for (int32_t j = 0; j < n; ++j) {
            auto* c = arr->get_object(j);
            if (c == nullptr || c->get_outer() != obj) continue;
            if (class_name_of(c) == L"MeteoriteHudVisibility") {
                dump_class_properties(c, "MeteoriteHudVisibility", true);
                break;
            }
        }
        return;
    }
}


// Resolved reticle widgets. The STORAGE lives here because reticle_rescan() below populates it, but
// the type and the extern declarations are in Reticule.hpp: the widget reticule consumes these from
// the other translation unit.
// g_reticles / g_reticle_count / g_reticle_scan_tick are DEFINED in Reticule.cpp and declared in
// Reticule.hpp. They cannot be defined here: everything below sits in an anonymous namespace, so a
// definition here would be internal to this translation unit and the reticule module could not see
// it. reticle_rescan() below still populates them through the extern declarations.





// Read pixels straight out of a render target and log them. This answers "does the widget's render
// target contain colour?" NUMERICALLY, with no display material in the loop -- every on-screen
// attempt so far has been hostage to whether the chosen material family renders at all.
// ReadRenderTargetPixel(WorldContextObject, TextureRenderTarget2D*, int32 X, int32 Y) -> FColor.
void probe_render_target_pixels(API::UObject* rt) {
    auto* pc = API::get()->get_player_controller(0);
    if (rt == nullptr || pc == nullptr) return;

    auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.KismetRenderingLibrary");
    auto* krl = (cls != nullptr) ? cls->get_class_default_object() : nullptr;
    if (krl == nullptr) return;

    // Coarse scan: log the first opaque-ish pixels found, plus corner samples. With the background
    // clear this finds the crosshair's own pixels and reports their true colours.
    int logged = 0;
    for (int32_t y = 64; y <= 192 && logged < 24; y += 6) {
        for (int32_t x = 64; x <= 192 && logged < 24; x += 6) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            *reinterpret_cast<void**>(p)      = pc;
            *reinterpret_cast<void**>(p + 8)  = rt;
            *reinterpret_cast<int32_t*>(p + 16) = x;
            *reinterpret_cast<int32_t*>(p + 20) = y;
            krl->call_function(L"ReadRenderTargetPixel", p);
            // FColor is B,G,R,A bytes; the return slot follows the parameters.
            const uint8_t* col = p + 24;
            if (col[3] > 8) {
                API::get()->log_info("[Halo-CampE-UEVR] RTPIXEL (%d,%d) = R%u G%u B%u A%u",
                                     x, y, col[2], col[1], col[0], col[3]);
                ++logged;
            }
        }
    }
    if (logged == 0) API::get()->log_info("[Halo-CampE-UEVR] RTPIXEL scan: no pixels with alpha > 8");
}

// Asset loading helpers (load_asset_by_path, find_or_load_material, import_texture_file,
// make_color_rt) moved to Reticule.cpp -- the reticule is their only caller.



// Sweep MaterialInstanceConstants for texture-parameter OVERRIDES. An override entry proves the
// parent material samples a texture under that name, which is exactly what binding our own texture
// (or the widget's render target) needs. Chunked: full-array sweeps in one tick stall the game.
int32_t g_tex_hunt_idx  = 0;
bool    g_tex_hunt_done = false;
int     g_tex_hunt_hits = 0;


void texture_param_hunt(uint32_t tick) {
    if (!g_cfg.tex_hunt || g_tex_hunt_done) return;
    static uint32_t last = 0;
    if (tick - last < 2) return;
    last = tick;

    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;
    const int32_t n = arr->get_object_count();

    constexpr size_t OFF_TEXTURE = 440;
    constexpr size_t TEX_STRIDE  = 40;

    int processed = 0;
    while (g_tex_hunt_idx < n && processed < 3000) {
        auto* o = arr->get_object(g_tex_hunt_idx++);
        ++processed;
        if (o == nullptr) continue;
        if (class_name_of(o) != L"MaterialInstanceConstant") continue;

        auto* base = reinterpret_cast<const uint8_t*>(o);
        if (IsBadReadPtr(base + OFF_TEXTURE, 16)) continue;
        auto* data = *reinterpret_cast<uint8_t* const*>(base + OFF_TEXTURE);
        const int32_t num = *reinterpret_cast<const int32_t*>(base + OFF_TEXTURE + 8);
        if (data == nullptr || num <= 0 || num > 64) continue;   // 64: environment materials routinely carry 10-20
        if (IsBadReadPtr(data, TEX_STRIDE * (size_t)num)) continue;

        // Names must decode as printable ASCII; garbage means the entry is not what we think and
        // the whole object is skipped rather than logged as a false lead.
        std::string names;
        bool ok = true;
        for (int32_t i = 0; i < num && ok; ++i) {
            API::FName nm{};
            memcpy(&nm, data + TEX_STRIDE * (size_t)i, sizeof(int32_t) * 2);
            const std::string dec = narrow(nm.to_string());
            if (!fname_is_sane(dec)) { ok = false; break; }
            if (!names.empty()) names += ", ";
            names += dec;
        }
        if (!ok || names.empty()) continue;

        // Colour capability: any of the usual vector names answering non-zero on the instance
        // (K2_GetVectorParameterValue traverses to the parent).
        bool has_colour = false;
        static const wchar_t* kColourNames[] = { L"Color", L"Colour", L"Tint", L"TintColor", L"BaseColor" };
        for (const wchar_t* cn : kColourNames) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            API::FName param = make_fname(cn);
            memcpy(p, &param, sizeof(int32_t) * 2);
            o->call_function(L"K2_GetVectorParameterValue", p);
            const auto* c = reinterpret_cast<const float*>(p + 8);
            if (c[0] != 0.0f || c[1] != 0.0f || c[2] != 0.0f) { has_colour = true; break; }
        }

        if (g_tex_hunt_hits < 120) {
            std::string path;
            for (API::UObject* q = o; q != nullptr; q = q->get_outer()) {
                const auto* fn = q->get_fname();
                path = (fn != nullptr ? narrow(fn->to_string()) : std::string("?"))
                     + (path.empty() ? "" : "." + path);
            }
            API::get()->log_info("[Halo-CampE-UEVR] TEXHUNT %s%s :: tex params [%s]",
                                 has_colour ? "[+colour] " : "", path.c_str(), names.c_str());
        }
        ++g_tex_hunt_hits;
    }

    if (g_tex_hunt_idx >= n) {
        g_tex_hunt_done = true;
        API::get()->log_info("[Halo-CampE-UEVR] TEXHUNT complete: %d MIC(s) with texture overrides", g_tex_hunt_hits);
    }
}

// Log every parameter override on a material instance: names via FName::to_string, plus values.
// Walks the arrays raw because reflection on these objects is unreliable here, and the layouts are
// stable engine structs:
//   TArray            = { T* Data; int32 Num; int32 Max }
//   FMaterialParameterInfo = { FName Name(8); uint8 Association; pad[3]; int32 Index } = 16 bytes
//   scalar entry  = Info(16) + float(4) + pad(4) + FGuid(16) = 40
//   vector entry  = Info(16) + FLinearColor(16)  + FGuid(16) = 48
//   texture entry = Info(16) + UTexture*(8)      + FGuid(16) = 40
// A base UMaterial keeps parameters in its expression graph, not these arrays, so parents are
// followed and dumped too (their MIC/MID layers carry the overridden names, which are the usable
// ones for SetXParameterValue).
void dump_material_params(API::UObject* mat, const char* label, int depth = 0) {
    if (mat == nullptr || depth > 3) return;

    constexpr size_t OFF_SCALAR = 392, OFF_VECTOR = 408, OFF_TEXTURE = 440, OFF_PARENT = 272;
    auto* base = reinterpret_cast<const uint8_t*>(mat);
    const std::wstring cls = class_name_of(mat);
    API::get()->log_info("[Halo-CampE-UEVR] MATDUMP %s%s (%s @%p)", depth ? "  parent: " : "", label,
                         narrow(cls).c_str(), (void*)mat);

    const bool is_instance = cls.find(L"MaterialInstance") != std::wstring::npos;
    if (is_instance) {
        struct ArraySpec { size_t off; const char* kind; size_t stride; };
        const ArraySpec specs[] = {
            { OFF_SCALAR,  "scalar",  40 },
            { OFF_VECTOR,  "vector",  48 },
            { OFF_TEXTURE, "texture", 40 },
        };
        for (const auto& spec : specs) {
            if (IsBadReadPtr(base + spec.off, 16)) continue;
            auto* data = *reinterpret_cast<uint8_t* const*>(base + spec.off);
            const int32_t num = *reinterpret_cast<const int32_t*>(base + spec.off + 8);
            if (data == nullptr || num <= 0 || num > 256) continue;
            if (IsBadReadPtr(data, spec.stride * (size_t)num)) continue;

            for (int32_t i = 0; i < num; ++i) {
                const uint8_t* e = data + spec.stride * (size_t)i;
                API::FName name{};
                memcpy(&name, e, sizeof(int32_t) * 2);
                const std::string pname = narrow(name.to_string());

                if (spec.kind[0] == 's') {
                    const float v = *reinterpret_cast<const float*>(e + 16);
                    API::get()->log_info("[Halo-CampE-UEVR]   scalar  '%s' = %.3f", pname.c_str(), v);
                } else if (spec.kind[0] == 'v') {
                    const float* c = reinterpret_cast<const float*>(e + 16);
                    API::get()->log_info("[Halo-CampE-UEVR]   vector  '%s' = (%.3f, %.3f, %.3f, %.3f)",
                                         pname.c_str(), c[0], c[1], c[2], c[3]);
                } else {
                    auto* tex = *reinterpret_cast<API::UObject* const*>(e + 16);
                    std::string tn = "null";
                    if (tex != nullptr && !IsBadReadPtr(tex, 16)) {
                        const auto* fn = tex->get_fname();
                        if (fn != nullptr) tn = narrow(fn->to_string());
                    }
                    API::get()->log_info("[Halo-CampE-UEVR]   texture '%s' = %s (%p)",
                                         pname.c_str(), tn.c_str(), (void*)tex);
                }
            }
        }
        if (!IsBadReadPtr(base + OFF_PARENT, 8)) {
            auto* parent = *reinterpret_cast<API::UObject* const*>(base + OFF_PARENT);
            if (parent != nullptr) dump_material_params(parent, label, depth + 1);
        }
    } else {
        API::get()->log_info("[Halo-CampE-UEVR]   (base UMaterial: parameters live in its expression graph, not walkable here)");
    }
}

// One-shot driver for the matdump config key.
void run_mat_dump() {
    static std::string done;
    if (g_cfg.mat_dump[0] == 0 || done == g_cfg.mat_dump) return;
    done = g_cfg.mat_dump;

    std::string list{g_cfg.mat_dump};
    size_t pos = 0;
    while (pos < list.size()) {
        size_t comma = list.find(',', pos);
        if (comma == std::string::npos) comma = list.size();
        std::string path = list.substr(pos, comma - pos);
        pos = comma + 1;
        while (!path.empty() && path.front() == ' ') path.erase(path.begin());
        while (!path.empty() && path.back() == ' ') path.pop_back();
        if (path.empty()) continue;

        const std::wstring w(path.begin(), path.end());
        API::UObject* m = API::get()->find_uobject<API::UObject>((L"MaterialInstanceConstant " + w).c_str());
        if (m == nullptr) m = API::get()->find_uobject<API::UObject>((L"Material " + w).c_str());
        if (m == nullptr) m = API::get()->find_uobject<API::UObject>((L"MaterialInstanceDynamic " + w).c_str());
        if (m == nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] MATDUMP not found: %s", path.c_str());
            continue;
        }
        dump_material_params(m, path.c_str());
    }
}

// ---------------------------------------------------------------- MATERIAL HUNT
//
// Sweeps the object array for Materials / MaterialInstanceConstants that expose a colour parameter,
// so the mesh reticule can be tinted a chosen colour. Probing the PARENT asset is what discriminates:
// a MID hands back any override you write under any name, so it cannot tell a real parameter from a
// typo, whereas an asset returns a non-zero default only for parameters it genuinely has.
//
// CHUNKED ON PURPOSE. A full sweep is ~296k objects; doing it in one tick collapses the
// framerate. This walks a slice per tick and then stops for good.
int32_t g_mat_hunt_idx  = 0;
bool    g_mat_hunt_done = false;
int     g_mat_hunt_hits = 0;
int     g_mat_hunt_seen = 0;      // materials actually PROBED -- without this, "0 hits" is ambiguous
int32_t g_mat_hunt_last_n = 0;    // array size at completion, to detect later streaming
int     g_mat_hunt_passes = 0;

void material_hunt(uint32_t tick) {
    if (!g_cfg.mat_hunt || g_mat_hunt_done) return;

    static uint32_t last = 0;
    if (tick - last < 4) return;
    last = tick;

    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;
    const int32_t n = arr->get_object_count();

    // Between passes, wait until the array has grown by a real margin before sweeping again;
    // otherwise this would spin through identical passes for no reason.
    if (g_mat_hunt_idx == 0 && g_mat_hunt_passes > 0 && n < g_mat_hunt_last_n + 5000) return;

    static const wchar_t* kNames[] = {
        L"Color", L"Colour", L"BaseColor", L"Tint", L"TintColor", L"TintColour",
        L"EmissiveColor", L"Emissive", L"EmissiveTint", L"BaseColorTint", L"ColorTint"
    };

    int processed = 0;
    while (g_mat_hunt_idx < n && processed < 2000) {
        auto* o = arr->get_object(g_mat_hunt_idx++);
        ++processed;
        if (o == nullptr) continue;

        const std::wstring cn = class_name_of(o);
        if (cn != L"Material" && cn != L"MaterialInstanceConstant") continue;
        ++g_mat_hunt_seen;

        for (const wchar_t* name : kNames) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            API::FName param = make_fname(name);
            memcpy(p, &param, sizeof(int32_t) * 2);
            o->call_function(L"K2_GetVectorParameterValue", p);
            const auto* c = reinterpret_cast<const float*>(p + 8);
            // RGB must be non-zero, NOT merely "any component". Verified live: querying a parameter
            // that does not exist returns (0,0,0,1) on a MaterialInstanceConstant -- so an
            // any-component test would flag every material in the game as a hit. (A UMaterial
            // returns (0,0,0,0) for a miss; this test covers both.)
            // Known limitation: a real parameter whose default is pure black is indistinguishable
            // from a miss through this API, so such a material would be skipped.
            if (c[0] == 0.0f && c[1] == 0.0f && c[2] == 0.0f) continue;

            if (g_mat_hunt_hits < 40) {
                std::string path;
                for (API::UObject* q = o; q != nullptr; q = q->get_outer()) {
                    const auto* fn = q->get_fname();
                    path = (fn != nullptr ? narrow(fn->to_string()) : std::string("?"))
                         + (path.empty() ? "" : "." + path);
                }
                API::get()->log_info("[Halo-CampE-UEVR] MATHUNT %s :: '%s' = (%.2f,%.2f,%.2f,%.2f)",
                                     path.c_str(), narrow(name).c_str(), c[0], c[1], c[2], c[3]);
            }
            ++g_mat_hunt_hits;
            break;   // one hit per material is enough to identify it
        }
    }

    if (g_mat_hunt_idx >= n) {
        ++g_mat_hunt_passes;
        g_mat_hunt_last_n = n;
        API::get()->log_info("[Halo-CampE-UEVR] MATHUNT pass %d complete: %d hit(s) from %d material(s) "
                             "probed, %d objects scanned",
                             g_mat_hunt_passes, g_mat_hunt_hits, g_mat_hunt_seen, n);

        // A first pass finishes while the level is still streaming -- the object array grows
        // several-fold after load, so a single pass latches done having never seen most of it.
        // Re-sweep whenever the array has grown meaningfully, so the result describes the loaded
        // game rather than whatever existed at load time.
        if (g_mat_hunt_passes >= 4) {
            g_mat_hunt_done = true;
            API::get()->log_info("[Halo-CampE-UEVR] MATHUNT finished after %d passes", g_mat_hunt_passes);
        } else {
            g_mat_hunt_idx = 0;
            g_mat_hunt_seen = 0;
            g_mat_hunt_hits = 0;
        }
    }
}


// The world-space mesh reticule and the widget reticule now live in Reticule.cpp.
// Driven from update() below via reticule_mesh_ensure/_move and reticule_widget_ensure/_move.

// ---------------------------------------------------------------- HUD RETICLE FOLLOW
//
// Move the game's own first-person reticle to where the shot actually goes.
//
// WHY THE GAME'S RETICLE AND NOT OUR OWN
// It already carries the hit marker, the per-weapon art and the reload/scope states. A world-space
// reticule of our own would have to re-implement all of that. (Borrowing a level StaticMeshActor
// instead removes set dressing from the map and is unshippable.)
//
// WHY IT IS OFF-CENTRE IN THE FIRST PLACE
// The aim lives on ControlRotation (the Blam camera, which is what the bullets follow) while the
// player looks along UEVR's composed view. Those two are deliberately decoupled by the view lock,
// so the game drawing its crosshair at the centre of ITS camera puts it nowhere near the aim.
//
// WHY IT IS SAFE TO DRIVE FROM HERE
// Only the UMG render transform is written, which is a presentation-layer value: no gameplay state,
// no Blam object, and nothing the game reads back. Worst case on a bad value is a mispositioned
// 2D sprite, which is why this is preferred over any attempt to move the aim itself.
// (ReticleTarget / g_reticles declared above the mesh+widget reticule section, which consumes them.)

// ---- MENU WIDGET DETECTION
// Candidates are collected in the SAME object-array pass as the reticle: a second full sweep of
// tens of thousands of objects on a timer is the pattern that crashes during loads.
//
// EXISTENCE IS NOT OPENNESS. A pause-menu widget is typically constructed once and kept alive, so
// merely finding one says nothing about whether it is on screen. IsInViewport() is UMG's own
// answer to that question -- widgets are added to the viewport when shown and removed when closed.
TrackedObject g_menu_candidates[8];
int  g_menu_candidate_count = 0;

// Whether menu_poll's PRIMARY path (the UI-manager subsystem) is answering. When it is, the
// candidate list below is dead weight -- menu_poll returns before ever reading it -- and the
// object-array sweep that fills it has one less reason to run. Starts true so that the very first
// scan is not skipped before menu_poll has had a chance to decide.
std::atomic<bool> g_ui_manager_ok{true};
std::atomic<bool> g_menu_widget_open{false};

// ---- THE AUTHORITATIVE MENU SIGNAL
//
// HaloUIManagerSubsystem::IsUIActiveState() -- the game's own "is the UI up" state: pause menu
// closed -> false, pause menu open -> true (verified).
//
// Preferred over widget hunting: the widget route needs a full object-array scan to find
// candidates and then a per-candidate visibility call, and it cannot see the pause menu at all
// through IsInViewport(). This is one call on a GameInstanceSubsystem -- not a Blam object, so
// UEVR's reflection is safe here, unlike on the PlayerController.
TrackedObject g_ui_manager;

API::UObject* find_ui_manager() {
    if (auto* cached = g_ui_manager.get_checked(L"HaloUIManagerSubsystem")) {
        return cached;
    }

    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return nullptr;

    const int32_t n = arr->get_object_count();
    for (int32_t i = 0; i < n; ++i) {
        auto* o = arr->get_object(i);
        if (o == nullptr) continue;
        if (class_name_of(o).find(L"HaloUIManagerSubsystem") == std::wstring::npos) continue;
        auto* cls = o->get_class();
        if (cls != nullptr && o == cls->get_class_default_object()) continue;
        g_ui_manager.set_at(o, i);
        API::get()->log_info("[Halo-CampE-UEVR] HaloUIManagerSubsystem acquired @%p", (void*)o);
        return o;
    }
    return nullptr;
}

bool is_menuish_class(const std::wstring& cn) {
    return cn.find(L"PauseMenu")  != std::wstring::npos
        || cn.find(L"PauseScreen")!= std::wstring::npos
        || cn.find(L"WBP_Pause")  != std::wstring::npos
        || cn.find(L"MainMenu")   != std::wstring::npos
        || cn.find(L"OptionsMenu")!= std::wstring::npos
        || cn.find(L"WBP_Menu")   != std::wstring::npos;
}

bool call_ret_bool(API::UObject* obj, const wchar_t* fn) {
    if (obj == nullptr) return false;
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    obj->call_function(fn, params);
    return params[0] != 0;
}

// ESlateVisibility, returned as a single byte:
//   0 Visible  1 Collapsed  2 Hidden  3 HitTestInvisible  4 SelfHitTestInvisible
// Sentinel 0xFF distinguishes "the call did not happen" from a real Visible(0).
constexpr uint8_t VIS_UNKNOWN = 0xFF;
uint8_t widget_visibility(API::UObject* w) {
    if (w == nullptr) return VIS_UNKNOWN;
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    params[0] = VIS_UNKNOWN;
    w->call_function(L"GetVisibility", params);
    return params[0];
}

bool visibility_means_shown(uint8_t v) {
    return v != VIS_UNKNOWN && v != 1 /*Collapsed*/ && v != 2 /*Hidden*/;
}

// Deliberately re-resolved on a timer rather than cached once. Actors on this title are pooled and
// recycled and holding a reference across frames is a known crash here, so the pointer is treated
// as valid only for the ~2 s window it was found in, and its class is re-checked before every use.
void reticle_rescan(uint32_t tick) {
    // Throttled UNCONDITIONALLY -- including when the count is zero. A class name that matches
    // nothing would otherwise turn this into a full object-array sweep with a class-name lookup
    // per object EVERY TICK, collapsing the framerate and starving the aim loop. "Found nothing"
    // is exactly when a scan must back off, not when it should run hardest.
    if (tick - g_reticle_scan_tick < 120) return;
    g_reticle_scan_tick = tick;

    // DOES ANYONE ACTUALLY READ THIS? Measured at 100-125 ms per sweep on the game thread -- ten
    // frames at 90 Hz -- and it ran five times per 20 s forever. That is the periodic microstutter.
    //
    // The throttle above bounds how OFTEN it runs; it never asked whether it should run at all.
    // Everything the sweep produces has exactly two consumers, and in the shipping profile both go
    // quiet seconds after a level loads:
    //   g_reticles         -> hud_reticle_follow (returns immediately when hudfollow=0, the default)
    //                      -> the widget reticule's ONE-SHOT pick, done once the component binds
    //   g_menu_candidates  -> menu_poll's FALLBACK only, dead while the UI-manager subsystem answers
    // So after the widget bound, this was rebuilding two arrays that nothing would ever look at.
    //
    // Each condition is re-tested every 120 ticks rather than latched, so turning hudfollow on in
    // the config, or losing the widget binding, brings the scan straight back. That is why this is
    // a demand check and not a "scanned once, done" flag.
    const bool needed = g_cfg.menu_dump                              // discovery: the sweep IS the product
                     || g_cfg.hud_follow                             // moves/hides the flat reticle
                     || reticle_widget_needs_pick()                  // still choosing a widget to host
                     || (g_cfg.menu_detect && !g_ui_manager_ok.load());   // candidates are the fallback
    if (!needed) return;

    PerfScope _perf(PERF_RETICLE);   // inside the gate: times the sweep, not the 119 early-outs
    g_reticle_count = 0;

    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;

    g_menu_candidate_count = 0;

    const int32_t n = arr->get_object_count();
    for (int32_t i = 0; i < n; ++i) {
        auto* o = arr->get_object(i);
        if (o == nullptr) continue;
        const std::wstring cn = class_name_of(o);

        const bool is_reticle = cn.find(wanted_widget_class()) != std::wstring::npos;
        const bool is_menu    = g_cfg.menu_detect && is_menuish_class(cn);
        if (!is_reticle && !is_menu && !g_cfg.menu_dump) continue;

        auto* cls = o->get_class();
        if (cls != nullptr && o == cls->get_class_default_object()) continue;   // never the CDO

        if (is_reticle && g_reticle_count < 8) {
            g_reticles[g_reticle_count].obj.set_at(o, i);
            g_reticles[g_reticle_count].found_tick = tick;
            ++g_reticle_count;
        }
        if (is_menu && g_menu_candidate_count < 8) {
            g_menu_candidates[g_menu_candidate_count++].set_at(o, i);
        }

        // Discovery. The match list above is a guess at this game's naming, and a guess that fails
        // silently would leave menu detection permanently off with no clue why. With menudump=1,
        // open a pause menu and the log names every widget that is actually in the viewport.
        // The scan itself already runs only every 120 ticks, so listing everything it finds is
        // the right granularity -- a per-object throttle would report one widget per scan and
        // hide the rest.
        if (g_cfg.menu_dump && !is_reticle) {
            const bool widgetish = cn.find(L"WBP_") != std::wstring::npos
                                || cn.find(L"UserWidget") != std::wstring::npos;
            if (widgetish && call_ret_bool(o, L"IsInViewport")) {
                API::get()->log_info("[Halo-CampE-UEVR] MENUDUMP in-viewport widget: %s",
                                     narrow(cn).c_str());
            }
        }
    }

    // Log only on a CHANGE in count. Per-scan logging even at 0.5 Hz buries the log over a
    // session, and the count is the only part that carries information.
    //
    // The OUTER CHAIN is logged because it is the one thing that distinguishes the two candidates,
    // and they behave completely differently:
    //   ...WBP_HUD_Main_C:WidgetTree.FirstPersonReticle  = the TEMPLATE on the class. Writing to it
    //       may only be inherited by the next HUD that gets constructed -- a one-off offset, not
    //       something that can track aim.
    //   ...<some live widget>:WidgetTree.FirstPersonReticle = a constructed instance, which is what
    //       actually renders and what we need.
    static int last_reported = -1;
    if (g_reticle_count == 0 && last_reported != 0) {
        API::get()->log_info("[Halo-CampE-UEVR] widget scan: NO match for class '%s' -- nothing to host",
                             narrow(wanted_widget_class()).c_str());
    }
    if (g_reticle_count != last_reported) {
        last_reported = g_reticle_count;
        API::get()->log_info("[Halo-CampE-UEVR] HUD reticle: %d widget(s) resolved", g_reticle_count);
        for (int i = 0; i < g_reticle_count; ++i) {
            auto* o = g_reticles[i].obj.get();
            if (o == nullptr) continue;
            std::string path;
            for (API::UObject* p = o; p != nullptr; p = p->get_outer()) {
                const auto* fn = p->get_fname();
                path = (fn != nullptr ? narrow(fn->to_string()) : std::string("?")) +
                       (path.empty() ? "" : "." + path);
            }
            API::get()->log_info("[Halo-CampE-UEVR]   [%d] %s", i, path.c_str());
        }
    }
}

// Ask the candidates whether any is actually on screen. Polled faster than the object-array scan
// (which is expensive) but far slower than every frame: a menu opening 10 ticks late is
// imperceptible, while an IsInViewport call per candidate per frame is not free.
void menu_poll(uint32_t tick) {
    if (!g_cfg.menu_detect) { g_menu_widget_open = false; return; }

    static uint32_t last = 0;
    if (tick - last < 10) return;
    last = tick;

    // PRIMARY: ask the game. Cheap enough to poll and correct by construction.
    if (auto* uim = find_ui_manager()) {
        g_ui_manager_ok = true;
        const bool ui_active = call_ret_bool(uim, L"IsUIActiveState");
        const bool prev = g_menu_widget_open.exchange(ui_active);
        if (prev != ui_active) {
            API::get()->log_info("[Halo-CampE-UEVR] IsUIActiveState -> %d", (int)ui_active);
        }
        return;
    }

    g_ui_manager_ok = false;

    // FALLBACK ONLY. Kept because the subsystem lookup could fail on a future build, but it cannot
    // see the pause menu (nested widgets never report IsInViewport) -- so this is a degraded mode,
    // not an equal alternative.

    // VISIBILITY, NOT IsInViewport.
    //
    // IsInViewport() is only true for a widget added DIRECTLY to the viewport. Halo's pause menu
    // is WBP_PauseMenu_C nested inside the persistent WBP_MeteoriteUILayout_C.WidgetTree, so it
    // is never itself "in viewport" and that test always answers false for it -- only the layout
    // container, the HUD and the fade overlay pass it.
    bool open = false;
    for (int i = 0; i < g_menu_candidate_count; ++i) {
        auto* o = g_menu_candidates[i].get();
        if (o == nullptr) continue;
        if (!is_menuish_class(class_name_of(o))) { g_menu_candidates[i].reset(); continue; }

        const uint8_t vis = widget_visibility(o);

        // Logged on CHANGE, per candidate, because which enum value this game uses for "closed" is
        // an assumption (Collapsed vs Hidden vs destroying the widget outright). One run with this
        // makes it a measurement instead.
        static uint8_t last_vis[8] = {VIS_UNKNOWN, VIS_UNKNOWN, VIS_UNKNOWN, VIS_UNKNOWN,
                                      VIS_UNKNOWN, VIS_UNKNOWN, VIS_UNKNOWN, VIS_UNKNOWN};
        if (vis != last_vis[i]) {
            last_vis[i] = vis;
            API::get()->log_info("[Halo-CampE-UEVR] menu candidate[%d] %s visibility=%u (%s)",
                                 i, narrow(class_name_of(o)).c_str(), (unsigned)vis,
                                 visibility_means_shown(vis) ? "SHOWN" : "hidden/unknown");
        }

        if (visibility_means_shown(vis)) { open = true; }
    }

    const bool was = g_menu_widget_open.exchange(open);
    if (was != open) {
        API::get()->log_info("[Halo-CampE-UEVR] menu %s (widget in viewport)", open ? "OPENED" : "CLOSED");
    }
}

// UMG screen convention: +X right, +Y DOWN. So an aim ABOVE the view is a NEGATIVE Y.
void hud_reticle_follow(float aim_pitch, float aim_yaw, uint32_t tick) {
    if (!g_cfg.hud_follow) return;
    if (!g_have_render_yaw.load()) return;      // no composed view yet -- pre-injection or menu
    if (g_in_menu.load()) return;               // frontend has no first-person reticle to move
    if (g_stick_mode.load()) return;            // stick mode: the game's own reticle is correct as drawn

    if (g_reticle_count == 0) return;   // scan is driven from update(), not from here

    // HIDE THE FLAT CROSSHAIR. Re-applied every tick with no "already done" latch, because the game
    // re-asserts HUD visibility on weapon switch, respawn and scope changes -- a one-shot hide gets
    // silently undone and reads as "the hide never worked". SetVisibility early-outs when the value
    // already matches, so repeating it is close to free.
    if (g_cfg.hud_hide) {
        for (int i = 0; i < g_reticle_count; ++i) {
            auto* o = g_reticles[i].obj.get_checked(L"WBP_FirstPersonReticle");
            if (o == nullptr) continue;
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            p[0] = 1;   // ESlateVisibility::Collapsed
            o->call_function(L"SetVisibility", p);
        }
        return;   // nothing to follow once it is hidden
    }

    // ---- EXACT OFFSET BY PROJECTION (hudproject).
    //
    // The k*tan(angle) form below needs hud_k -- a fudge constant derived from the HUD quad's
    // geometry with an assumed canvas height, and a built-in source of drift. The engine can
    // answer exactly instead: project the aim point to screen space and subtract the viewport
    // centre. That delta IS the render translation, with no constant to calibrate.
    if (g_cfg.hud_project && g_have_ret_origin.load()) {
        static API::UObject* gps = nullptr;
        static API::UObject* wll = nullptr;
        static bool tried = false;
        if (!tried) {
            tried = true;
            if (auto* c = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.GameplayStatics"))
                gps = c->get_class_default_object();
            if (auto* c = API::get()->find_uobject<API::UClass>(L"Class /Script/UMG.WidgetLayoutLibrary"))
                wll = c->get_class_default_object();
            API::get()->log_info("[Halo-CampE-UEVR] hudproject: GameplayStatics=%p WidgetLayoutLibrary=%p",
                                 (void*)gps, (void*)wll);
        }

        auto* pc = API::get()->get_player_controller(0);
        if (gps != nullptr && wll != nullptr && pc != nullptr) {
            // Aim point in world: origin + forward * distance, same ray the reticule uses.
            const float cp = std::cos(aim_pitch * DEG2RAD);
            const Vec3 o = g_ret_origin;
            const float d = g_cfg.aim_reticule_dist;
            const Vec3 w{o.x + cp * std::cos(aim_yaw * DEG2RAD) * d,
                         o.y + cp * std::sin(aim_yaw * DEG2RAD) * d,
                         o.z + std::sin(aim_pitch * DEG2RAD) * d};

            double sx = 0.0, sy = 0.0;
            bool ok = false;
            {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                *reinterpret_cast<void**>(p) = pc;                 // Player
                auto* wv = reinterpret_cast<double*>(p + 8);       // WorldPosition
                wv[0] = w.x; wv[1] = w.y; wv[2] = w.z;
                // ScreenPosition (out, FVector2D) at 32; bPlayerViewportRelative at 48; ret at 49
                gps->call_function(L"ProjectWorldToScreen", p);
                const auto* sp = reinterpret_cast<const double*>(p + 32);
                sx = sp[0]; sy = sp[1];
                ok = p[49] != 0;
            }

            double vw = 0.0, vh = 0.0;
            {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                *reinterpret_cast<void**>(p) = pc;                 // WorldContextObject
                wll->call_function(L"GetViewportSize", p);
                const auto* vs = reinterpret_cast<const double*>(p + 8);
                vw = vs[0]; vh = vs[1];
            }

            if (ok && vw > 1.0 && vh > 1.0 && std::isfinite(sx) && std::isfinite(sy)) {
                float px = (float)(sx - vw * 0.5);
                float py = (float)(sy - vh * 0.5);
                px = clampf(px, -g_cfg.hud_max, g_cfg.hud_max);
                py = clampf(py, -g_cfg.hud_max, g_cfg.hud_max);

                for (int i = 0; i < g_reticle_count; ++i) {
                    auto* r = g_reticles[i].obj.get_checked(L"WBP_FirstPersonReticle");
                    if (r == nullptr) continue;
                    alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
                    auto* dd = reinterpret_cast<double*>(q);
                    dd[0] = px; dd[1] = py;
                    r->call_function(L"SetRenderTranslation", q);
                }

                static uint32_t last_log = 0;
                if (tick - last_log >= 600) {
                    last_log = tick;
                    API::get()->log_info("[Halo-CampE-UEVR] hudproject: screen=(%.0f,%.0f) viewport=(%.0f,%.0f) -> px=(%.0f,%.0f)",
                                         sx, sy, vw, vh, px, py);
                }
                return;
            }

            static uint32_t last_warn = 0;
            if (tick - last_warn >= 600) {
                last_warn = tick;
                API::get()->log_info("[Halo-CampE-UEVR] hudproject unusable (ok=%d viewport=%.0fx%.0f) - falling back to hudk",
                                     (int)ok, vw, vh);
            }
        }
    }

    const float dyaw   = wrap180(aim_yaw   - g_render_view_yaw.load()   - g_cfg.hud_trim_yaw);
    const float dpitch = wrap180(aim_pitch - g_render_view_pitch.load() - g_cfg.hud_trim_pitch);

    // Behind the player, tan() flips sign and would place the reticle on the WRONG SIDE while
    // looking perfectly plausible. Bail rather than draw a confident lie.
    if (std::fabs(dyaw) > 89.0f || std::fabs(dpitch) > 89.0f) return;

    float px = g_cfg.hud_k * std::tan(dyaw   * DEG2RAD);
    float py = -g_cfg.hud_k * std::tan(dpitch * DEG2RAD);
    px = clampf(px, -g_cfg.hud_max, g_cfg.hud_max);
    py = clampf(py, -g_cfg.hud_max, g_cfg.hud_max);
    if (!std::isfinite(px) || !std::isfinite(py)) return;

    for (int i = 0; i < g_reticle_count; ++i) {
        auto* o = g_reticles[i].obj.get_checked(L"WBP_FirstPersonReticle");
        if (o == nullptr) {
            g_reticle_count = 0;          // force a rescan next tick
            continue;
        }

        alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
        if (g_cfg.hud_float) {
            auto* f = reinterpret_cast<float*>(params);
            f[0] = px; f[1] = py;
        } else {
            auto* d = reinterpret_cast<double*>(params);
            d[0] = (double)px; d[1] = (double)py;
        }
        o->call_function(L"SetRenderTranslation", params);
    }

    static uint32_t last_log = 0;
    if (tick - last_log >= 600) {           // ~20 s
        last_log = tick;
        API::get()->log_info("[Halo-CampE-UEVR] HUD reticle follow: dyaw=%.1f dpitch=%.1f -> px=(%.0f,%.0f) k=%.0f",
                             dyaw, dpitch, px, py, g_cfg.hud_k);
    }
}

// The aim control loop (control-rotation reads, the control law, pose reading) is in MotionAimControl.cpp.
// ---------------------------------------------------------------- the loop
void update() {
    g_aim_law_armed = false;
    const uint32_t tick = g_ticks.fetch_add(1);

    // Re-read the config about every 2 s at ~32 Hz. Cheap, and it is what makes `enabled=0` an
    // actual kill switch rather than a comment. Checked BEFORE the enabled test so the driver can
    // also be turned back ON from the file without a restart.
    if (tick - g_cfg_check_tick >= 64) {
        g_cfg_check_tick = tick;
        PerfScope _perf(PERF_CFG);
        load_config();
    }

    // Above every early-out below, so the numbers still arrive when the driver is disabled or
    // parked in a menu -- "it stutters at the frontend too" is a diagnosis, not a gap.
    perf_report(tick);

    // ---- Calibrate button/key, edge-detected on the GAME THREAD.
    // Both sources are polled here rather than in the XInput hook, because a keyboard key is not
    // visible from there at all and splitting the edge detection across two threads would race.
    // GetAsyncKeyState reads global key state, so it registers with the headset on and the game
    // focused -- which is the only way this is usable mid-session.
    {
        const bool key_down = (g_cfg.calib_key != 0) &&
                              ((GetAsyncKeyState(g_cfg.calib_key) & 0x8000) != 0);
        const bool held = key_down || g_pad_calib_down.load();
        const bool was  = g_calib_held.exchange(held);
        if (held && !was) g_calib_start  = true;
        if (!held && was) g_calib_finish = true;

        const bool aim_down = (g_cfg.aim_calib_key != 0) &&
                              ((GetAsyncKeyState(g_cfg.aim_calib_key) & 0x8000) != 0);
        const bool aim_was  = g_aimcal_held.exchange(aim_down);
        if (aim_down && !aim_was) {
            g_aimcal_start = true;
            API::get()->log_info("[Halo-CampE-UEVR] AIM CALIBRATE: aim frozen -- point your controller at the "
                                 "reticle, then release");
        }
        if (!aim_down && aim_was) g_aimcal_finish = true;

        // ---- KILL SWITCH, reachable with the headset ON.
        // Requires a modifier so it cannot be brushed mid-fight, and fires on the PRESS edge so a
        // held key toggles once rather than chattering. This is the in-headset counterpart to
        // `enabled` in halo_vr.cfg -- editing that file means finding a desktop, which is exactly
        // what you cannot do when aim is misbehaving and the headset is on.
        //
        // NOT Ctrl+Esc: Windows owns that (Start menu). It would steal focus, and on OpenXR losing
        // focus freezes controller poses -- the kill switch would cause the failure it exists to
        // rescue you from.
        if (g_cfg.kill_key != 0) {
            const bool ctrl = (GetAsyncKeyState(VK_CONTROL) & 0x8000) != 0;
            const bool kill_down = ctrl && ((GetAsyncKeyState(g_cfg.kill_key) & 0x8000) != 0);
            const bool kill_was  = g_kill_held.exchange(kill_down);
            if (kill_down && !kill_was) {
                g_cfg.enabled = !g_cfg.enabled;
                API::get()->log_info(g_cfg.enabled
                    ? "[Halo-CampE-UEVR] KILL SWITCH: driver RE-ENABLED (aim follows your controller again)"
                    : "[Halo-CampE-UEVR] KILL SWITCH: driver DISABLED -- stick neutral, game plays stock. "
                      "Press again to re-enable.");
            }
        }
    }

    // Release: rebind "controller pointing here" to "game aiming there". Clearing the reference is
    // the whole operation -- the existing capture path re-reads both on the next tick.
    if (g_aimcal_finish.exchange(false)) {
        g_have_ref = false;
        g_aimcal_capture = true;   // next capture MEASURES the offset instead of restoring it
    }

    if (!g_cfg.enabled) { g_out_rx = 0.0f; g_out_ry = 0.0f; g_driving = false; return; }

    // ---- MENU STATE. Computed HERE, above every early-out below, and deliberately so: the
    // ControlRotation read further down is a raw memory read of the Blam gameplay camera, and at
    // the FRONTEND there is no such camera -- update() bails before reaching it. Menu state
    // assigned after that read would never update in menus, leaving g_in_menu stuck at its last
    // gameplay value and every menu binding dead. Everything this needs is available now: a
    // controller pointer and the rig pointer. Nothing below is required.
    {
        auto* pc0 = API::get()->get_player_controller(0);
        const std::wstring pcn = (pc0 != nullptr)
            ? class_name_of(reinterpret_cast<API::UObject*>(pc0)) : std::wstring();
        const bool frontend = pcn.find(L"Frontend") != std::wstring::npos;
        const bool no_rig   = (g_rig_component.load() == nullptr);

        // Widget scan/poll also sit above the gates: a pause menu that happened to invalidate the
        // aim pose would otherwise switch detection off at the exact moment it is needed.
        // menu_poll is one subsystem call, so it runs unconditionally. Only the reticle scan
        // (which sweeps the whole object array) waits for a rig.
        menu_poll(tick);
        if (!no_rig) reticle_rescan(tick);
        material_hunt(tick);
        run_mat_dump();
        texture_param_hunt(tick);

        const bool now_menu = frontend || no_rig || g_menu_widget_open.load();
        const bool was_menu = g_in_menu.exchange(now_menu);
        if (was_menu != now_menu) {
            API::get()->log_info("[Halo-CampE-UEVR] IN_MENU %d -> %d (frontend=%d norig=%d widget=%d) pc=%s",
                                 (int)was_menu, (int)now_menu, (int)frontend, (int)no_rig,
                                 (int)g_menu_widget_open.load(), narrow(pcn).c_str());
        }

        // ---- STICK MODE detector (doctrine in Config.hpp). Evaluated here, above the early-out
        // gates, for the same reason menu state is: it must keep updating in the very states it
        // exists to detect.
        //
        // Core signal: gameplay is running but the game is not rendering a first-person weapon --
        // the weapon-actor route is how that is observable from outside (vehicle seats, cutscenes,
        // death, the post-load window). The pawn baseline and rig-component liveness ride along in
        // the log only: WHICH of the three flips per vehicle seat is recon R1
        // (docs\VEHICLE_CAMERA_FINDINGS.md), and one logged vehicle entry settles it.
        {
            const bool gameplay    = (pc0 != nullptr) && !frontend;
            const bool route_alive = fp_weapon_route_alive();

            auto* pawn = API::get()->get_local_pawn(0);
            auto* base = g_stick_pawn_base.get_checked(L"Pawn");
            if (route_alive && pawn != nullptr && base != pawn) {
                g_stick_pawn_base.set(pawn);   // (re)baseline while the FP weapon is live
                base = pawn;
            }
            const bool pawn_match = (pawn != nullptr) && (pawn == base);

            const bool signal = g_cfg.stick_mode && gameplay && !route_alive;

            // Debounce, in ~32 Hz ticks. Enter is slow on purpose: a weapon swap kills the route
            // for up to the resolve cadence (~2 s), and flapping the camera mode mid-fight is
            // worse than a late vehicle transition. Exit is quick -- the route reviving IS a
            // successful resolve, which is already debounce enough.
            static uint32_t on_streak = 0, off_streak = 0;
            const uint32_t need_on  = (uint32_t)(clampf(g_cfg.stick_on_s,  0.1f, 30.0f) * 32.0f);
            const uint32_t need_off = (uint32_t)(clampf(g_cfg.stick_off_s, 0.03f, 30.0f) * 32.0f);

            bool want = g_stick_mode.load();
            if (signal) { off_streak = 0; if (++on_streak  >= need_on)  want = true;  }
            else        { on_streak  = 0; if (++off_streak >= need_off) want = false; }

            // Force bypasses the detector outright -- the A/B lever, and the manual fallback if
            // some vehicle seat keeps its FP weapon alive and the detector misses.
            if      (!g_cfg.stick_mode || g_cfg.stick_force == 2) want = false;
            else if (g_cfg.stick_force == 1)                      want = true;

            const bool was = g_stick_mode.exchange(want);
            if (was != want) {
                if (want) {
                    // ENTER: neutralise the actuator now; drop the aim reference (restored from
                    // the saved calibration on exit); invalidate the render-rate rig target so
                    // the stereo hook stops re-applying a stale one.
                    g_out_rx = 0.0f; g_out_ry = 0.0f; g_driving = false;
                    g_have_ref = false;
                    g_rigw_valid = false;
                } else {
                    // EXIT: re-anchor against wherever the game camera is now. The view lock was
                    // held unprimed throughout, so the next stereo frame adopts the current camera
                    // yaw as the new base; the aim reference re-captures (restoring the saved
                    // hand-to-aim offset) on the next armed tick; the rig neutral re-captures with
                    // it.
                    g_have_ref = false;
                    g_rig_neutral_valid = false;
                }
                // g_turn_offset is deliberately PRESERVED across both transitions. The rig and
                // aim mappings fold `rig_turn * g_turn_offset` / `aim_turn * g_turn_offset` into
                // their room->game frames, so zeroing it here rotated the hand-to-weapon mapping
                // by the entire accumulated snap turn at the moment of exit -- observed in the
                // field as the weapon sitting ~45 deg wrong after a Pelican drop-off that
                // followed one 45 deg snap. The view stays seamless anyway, because the re-prime
                // compensates: it captures base = current - turn, so base + turn lands exactly on
                // the current view (see on_pre_calculate_stereo_view_offset).
                API::get()->log_info("[Halo-CampE-UEVR] STICK MODE %s (route=%d rigcomp=%d pawnmatch=%d "
                                     "gameplay=%d force=%d) -- %s",
                                     want ? "ENTER" : "EXIT",
                                     (int)route_alive, (int)rig_component_alive(), (int)pawn_match,
                                     (int)gameplay, g_cfg.stick_force,
                                     want ? "sticks pass through, the game camera owns the view"
                                          : "motion aim re-anchoring");
            }
        }
    }

    // ---- WHY THE DRIVER STOPPED. Logged on every CHANGE of state, never per frame.
    //
    // When this driver bails out it does so silently, and the symptom is indistinguishable from a
    // bug in the maths: tracking appears to collapse to 3DoF (positions stop updating), aim stops
    // being steered, and snap turn dies -- all at once, because all three live downstream of these
    // gates. This log is what says which gate closed.
    // Deliberately NOT queried before the tick-count gate below: calling
    // is_using_contriollers() during injection and level load, before the VR runtime is
    // necessarily up, crashes the game. Cheap to gate, expensive to debug.
    // Applied once, after injection has settled. Deliberately not at plugin load: UEVR's mod values
    // are not necessarily ready before the VR runtime is up.
    if (tick == 300 && g_cfg.vr_inactivity > 0.0f) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.1f", g_cfg.vr_inactivity);
        // The C function directly: the C++ template wrapper does not compile for string values (its
        // fallthrough calls std::to_string on the argument, which only exists for arithmetic types).
        API::get()->param()->vr->set_mod_value("VR_MotionControlsInactivityTimer", buf);
        API::get()->log_info("[Halo-CampE-UEVR] VR_MotionControlsInactivityTimer -> %s "
                             "(profile default is the 30s MINIMUM; 100 is the slider max)", buf);
    }

    if (tick > 120) {
        const bool hmd  = API::VR::is_hmd_active();
        const bool ctrl = API::VR::is_using_contriollers();   // sic: typo is in UEVR's header
        static int last_state = -1;
        const int state = (hmd ? 1 : 0) | (ctrl ? 2 : 0);
        if (state != last_state) {
            last_state = state;
            API::get()->log_info("[Halo-CampE-UEVR] VR STATE CHANGE: hmd_active=%d using_controllers=%d%s",
                                 (int)hmd, (int)ctrl,
                                 ctrl ? "" : "  <-- controllers dropped: poses freeze (looks like 3DoF), "
                                             "stick input stops (snap turn dies), aim stops driving");
        }
    }

    // `requirehmd=0` bypasses this gate for headless testing: under a null SteamVR driver
    // is_hmd_active() reports FALSE even with everything running, so the whole driver silently
    // no-ops and every log goes quiet -- which looks exactly like a broken build.
    if (g_cfg.require_hmd && !API::VR::is_hmd_active()) {
        static uint32_t last = 0;
        if (tick - last > 300) { last = tick; API::get()->log_info("[Halo-CampE-UEVR] IDLE: hmd not active"); }
        g_out_rx = 0.0f; g_out_ry = 0.0f; g_driving = false; return;
    }

    const auto ridx = API::VR::get_right_controller_index();
    if (ridx < 0) {
        static uint32_t last = 0;
        if (tick - last > 300) { last = tick; API::get()->log_info("[Halo-CampE-UEVR] IDLE: no right controller index"); }
        g_out_rx = 0.0f; g_out_ry = 0.0f; g_driving = false; return;
    }

    // AIM pose: the runtime's POINTING pose. Correct for aim direction.
    Vec3 cpos{}; Quat cq{};
    if (!get_pose(ridx, &cpos, &cq, /*use_aim=*/true)) {
        static uint32_t last = 0;
        if (tick - last > 300) { last = tick; API::get()->log_info("[Halo-CampE-UEVR] IDLE: aim pose invalid"); }
        g_out_rx = 0.0f; g_out_ry = 0.0f; g_driving = false; return;
    }

    // GRIP pose: where the HAND actually is. Used for rig TRANSLATION only.
    //
    // The aim pose is NOT the hand: it is a reprojected ray origin, so taking POSITION from
    // get_aim_pose() produces teleport-scale "travel" readings and pins the translation offset at
    // its clamp, while rotation stays fine. Position must come from the grip pose; only the
    // pointing direction comes from the aim pose.
    Vec3 gpos{}; Quat gq{};
    const bool have_grip = get_pose(ridx, &gpos, &gq, /*use_aim=*/false);
    // Fall back to the aim pose only if the grip pose is unavailable, so translation degrades to
    // an approximate position rather than to nothing.
    const Vec3 rigpos = have_grip ? gpos : cpos;

    double aim_pitch = 0.0, aim_yaw = 0.0;
    void* pc = nullptr;
    if (!read_control_rotation(&aim_pitch, &aim_yaw, &pc)) {
        // A different PlayerController class (multiplayer, new game modes) can put ControlRotation
        // somewhere other than the validated offset; the sanity gate then rejects it and the
        // driver fails closed -- exactly as designed, but invisibly without this log.
        static uint32_t last = 0;
        if (tick - last > 300) {
            last = tick;
            API::get()->log_info("[Halo-CampE-UEVR] IDLE: ControlRotation read failed (offset 0x%zX rejected) "
                                 "-- different PlayerController class?", CONTROL_ROTATION_OFFSET);
        }
        g_out_rx = 0.0f; g_out_ry = 0.0f; g_driving = false; return;
    }

    // Controller identity changed (frontend -> mission, or a map load): the old reference describes
    // a different world, so drop it and re-capture against this one.
    // (Menu detection lives at the top of update(), above the early-out gates -- see the note there.)

    if (g_ref_pc.load() != pc) {
        g_ref_pc = pc;
        g_have_ref = false;
        g_rig_neutral_valid = false;   // rig neutral is captured with the aim reference; drop both

        // Level transition: drop the UObjectHook attachment and the rig pointer BEFORE the old
        // actors are torn down. See attach_release() -- this is what keeps us out of the
        // render-path use-after-free.
        auto* old_rig = reinterpret_cast<API::UObject*>(g_rig_component.load());
        attach_release(old_rig, "PlayerController changed");
        g_rig_component = nullptr;
        g_rig_parent = nullptr;
        g_rig_resolve_tick = 0;   // re-resolve promptly on the new level
    }

    // UEVR's world rotation changed (recenter, or a snap turn): the VR-to-game anchor no longer
    // describes reality. Re-capture instead of chasing a phantom error. Compared as a quaternion
    // dot product so it is sign- and wrap-agnostic; ~0.9999 is about 1.6 degrees of change, well
    // below any real snap increment and well above tracking noise.
    {
        const auto ro = API::VR::get_rotation_offset();

        // Compare against OUR last write when view lock is driving the offset, otherwise against
        // the captured reference. Getting this wrong makes the driver re-anchor on its own
        // compensation every frame, which silently destroys the reference it depends on.
        const bool cmp_self = g_cfg.view_lock && g_self_rot_valid.load();
        const float rx = cmp_self ? g_self_rot_x.load() : g_ref_rot_x.load();
        const float ry = cmp_self ? g_self_rot_y.load() : g_ref_rot_y.load();
        const float rz = cmp_self ? g_self_rot_z.load() : g_ref_rot_z.load();
        const float rw = cmp_self ? g_self_rot_w.load() : g_ref_rot_w.load();

        const float dot = ro.x * rx + ro.y * ry + ro.z * rz + ro.w * rw;
        if (std::fabs(dot) < 0.9999f) {
            // Externally changed (UEVR recenter, snap turn): adopt it as the new base and re-anchor.
            g_ref_rot_x = ro.x; g_ref_rot_y = ro.y; g_ref_rot_z = ro.z; g_ref_rot_w = ro.w;
            g_self_rot_valid = false;
            if (g_have_ref.load()) {
                g_have_ref = false;
                API::get()->log_info("[Halo-CampE-UEVR] VR world rotation changed externally (recenter/snap turn) - re-anchoring");
            }
        }
    }

    Vec3 fwd = quat_forward(cq);

    // SIGHTLINE -- this is where controller TRANSLATION enters the aim solution, not just rotation.
    // Aim is the ray from an origin THROUGH the point the gun points at, so sliding the gun
    // sideways with unchanged rotation still moves aim. Straight from UEVR's controller-aim path.
    {
        Vec3 origin{};
        bool have_origin = false;

        if (g_cfg.aim_origin == 1) {
            const auto so = API::VR::get_standing_origin();   // body reference; head motion does NOT alter aim
            origin = Vec3{so.x, so.y, so.z};
            have_origin = true;
        } else {
            Vec3 hpos{}; Quat hq{};
            const auto hidx = API::VR::get_hmd_index();
            if (hidx >= 0 && get_pose(hidx, &hpos, &hq, /*use_aim=*/false)) { origin = hpos; have_origin = true; }
        }

        if (have_origin) {
            Vec3 t{
                cpos.x + fwd.x * g_cfg.xdist_m - origin.x,
                cpos.y + fwd.y * g_cfg.xdist_m - origin.y,
                cpos.z + fwd.z * g_cfg.xdist_m - origin.z
            };
            const float len = std::sqrt(t.x * t.x + t.y * t.y + t.z * t.z);
            if (len > 1e-3f) { fwd = Vec3{t.x / len, t.y / len, t.z / len}; }
        }
    }

    // ---- SNAP TURN, applied to the AIM mapping as well as the rig.
    //
    // The aim reference maps a ROOM-space controller yaw onto a GAME-space aim yaw, and a snap
    // turn rotates the rendered world without touching ControlRotation -- so after a turn the
    // same physical pointing direction means a different game direction, and the reference is
    // stale by exactly the turn. Both the rig frame AND this mapping need the correction;
    // correcting only one leaves the other breaking the same way.
    const float ctrl_yaw   = wrap180(std::atan2(fwd.x, -fwd.z) * RAD2DEG
                                     + g_cfg.aim_turn * g_turn_offset.load());
    const float ctrl_pitch = std::asin(clampf(fwd.y, -1.0f, 1.0f)) * RAD2DEG;

    // Reference capture: record the hand-to-aim offset once so enabling never snaps the view.
    // Not while stick mode holds the stack down -- a reference captured against a vehicle camera
    // is garbage, and the exit transition re-captures the moment the stack re-arms.
    if (!g_stick_mode.load() && !g_have_ref.load()) {
        g_ref_ctrl_yaw   = ctrl_yaw;
        g_ref_ctrl_pitch = ctrl_pitch;

        const bool measuring = g_aimcal_capture.exchange(false);

        if (g_cfg.aim_off_valid && !measuring) {
            // RESTORE a saved calibration rather than re-capturing whatever the game happens to
            // be aiming at: re-capturing binds the controller to the aim of that instant, which
            // after a level load or respawn is arbitrary -- silently discarding the calibration.
            // The loop then drives the aim onto the saved mapping.
            g_ref_aim_yaw   = wrap180(ctrl_yaw + g_cfg.aim_off_yaw);
            g_ref_aim_pitch = ctrl_pitch + g_cfg.aim_off_pitch;
            g_have_ref = true;
            g_gain_hold = true; g_gain_hold_until = tick + 90;   // let the aim settle before measuring gain
            API::get()->log_info("[Halo-CampE-UEVR] reference RESTORED from saved calibration: offset yaw=%.1f pitch=%.1f",
                                 g_cfg.aim_off_yaw, g_cfg.aim_off_pitch);
        } else {
            g_ref_aim_yaw   = (float)aim_yaw;
            g_ref_aim_pitch = (float)aim_pitch;
            g_have_ref = true;
            g_gain_hold = true; g_gain_hold_until = tick + 90;   // let the aim settle before measuring gain

            if (measuring) {
                // Page Down release: the offset between where the hand points and where the game
                // aims IS the calibration. Store it, not the absolute pair.
                g_cfg.aim_off_yaw   = wrap180((float)aim_yaw - ctrl_yaw);
                g_cfg.aim_off_pitch = (float)aim_pitch - ctrl_pitch;
                g_cfg.aim_off_valid = true;
                write_calib_file();
                API::get()->log_info("[Halo-CampE-UEVR] AIM CALIBRATED: offset yaw=%.1f pitch=%.1f -> saved",
                                     g_cfg.aim_off_yaw, g_cfg.aim_off_pitch);
            } else {
                API::get()->log_info("[Halo-CampE-UEVR] reference captured: ctrlYaw=%.1f aimYaw=%.1f",
                                     ctrl_yaw, (float)aim_yaw);
            }
        }
    }

    // Publish everything the render-rate law needs, then run whichever path is selected.
    // Arming is re-asserted every tick and cleared at entry, so any early-out above leaves the
    // hook-side law disarmed rather than steering from stale references.
    g_aim_law_ridx = ridx;
    g_aim_law_pc   = pc;

    {
        const float dbg_des_yaw   = g_ref_aim_yaw.load()   + wrap180(ctrl_yaw   - g_ref_ctrl_yaw.load());
        const float dbg_des_pitch = g_ref_aim_pitch.load() + wrap180(ctrl_pitch - g_ref_ctrl_pitch.load());
        g_dbg_err_yaw   = wrap180(dbg_des_yaw   - (float)aim_yaw);
        g_dbg_err_pitch = wrap180(dbg_des_pitch - (float)aim_pitch);
        g_dbg_ctrl_yaw = ctrl_yaw; g_dbg_aim_yaw = (float)aim_yaw;
    }

    if (!g_cfg.aim_rate_render && !g_stick_mode.load()) {
        static AimLawState tick_law;
        float rx = 0.0f, ry = 0.0f;
        aim_control_law(tick_law, ctrl_yaw, ctrl_pitch, aim_yaw, aim_pitch, g_last_dt.load(), &rx, &ry);
        g_out_rx = rx; g_out_ry = ry;
    }
    // STICK MODE: the law stays disarmed (cleared at entry), so both hook paths pass the player's
    // right stick straight through -- the whole point of the mode.
    if (!g_stick_mode.load()) {
        g_aim_law_armed = true;
        g_driving = true;
    }

    // ---- MEASURE THE ACTUAL TURN RATE and adapt the gain (see shape()).
    {
        if (g_gain_hold.load() && tick >= g_gain_hold_until.load()) g_gain_hold = false;
        static double prev_aim = 0.0;
        static float  prev_out = 0.0f;
        static bool   have_prev = false;
        const float dt = g_last_dt.load();

        if (have_prev && dt > 0.001f && dt < 0.2f) {
            const float achieved = wrap180((float)aim_yaw - (float)prev_aim) / dt;   // deg/s
            const float applied  = std::fabs(prev_out);

            // Only sample where the reading is meaningful: well past the deadzone, genuinely
            // moving, and not saturated against something (a wall of clamped error).
            // MAX_PLAUSIBLE_DPS rejects aim DISCONTINUITIES, not fast turning. A level load,
            // respawn or reference re-capture teleports the aim, and a 90 degree jump across one
            // tick reads as ~1600 deg/s -- which would drive the gain straight to its 4x clamp.
            // Nothing the stick can do exceeds this.
            constexpr float MAX_PLAUSIBLE_DPS = 400.0f;

            // The applied deflection is sampled once per tick; on the render-rate path it
            // changes many times within a tick, so this pairing would misestimate the plant gain
            // and the mis-adapted gain limit-cycles the loop.
            if (!g_cfg.aim_rate_render && applied > 0.5f && std::fabs(achieved) > 15.0f
                && std::fabs(achieved) < MAX_PLAUSIBLE_DPS && !g_gain_hold.load()) {
                const float rate = std::fabs(achieved) / applied;      // deg/s per unit
                if (std::isfinite(rate) && rate > 10.0f && rate < MAX_PLAUSIBLE_DPS * 1.5f) {
                    const float cur = g_meas_rate.load();
                    const float smoothed = (cur <= 0.0f) ? rate : (cur + 0.05f * (rate - cur));
                    g_meas_rate = smoothed;

                    // Faster game turn rate => reach full deflection over a WIDER error band, so
                    // the loop does not overshoot. Hence scale full_deg with the measured rate.
                    const float want = clampf(smoothed / REFERENCE_RATE_DPS, 0.25f, 4.0f);
                    const float g    = g_gain_scale.load();
                    g_gain_scale = g + 0.02f * (want - g);   // slow: never moves mid-fight

                    if (!g_gain_logged.load() && std::fabs(want - 1.0f) > 0.15f) {
                        g_gain_logged = true;
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] gain adapt: measured %.0f deg/s per unit (reference %.0f) "
                            "-> full_deg x%.2f. Game controller sensitivity differs from the "
                            "calibrated LookSensitivity30.", smoothed, REFERENCE_RATE_DPS, want);
                    }
                }
            }
        }

        prev_aim = aim_yaw;
        prev_out = g_out_rx.load();
        have_prev = true;
    }

    // ------------------------------------------------------------------ VIEW LOCK
    // Cancel, in VR space, the yaw the game has gained since the reference. Without this the
    // headset view is dragged along by every aim correction, because on this title
    // ControlRotation IS the Blam camera. UEVR does the identical thing in its controller-aim
    // path -- point the game's view along the controller, then set_rotation_offset() to the
    // inverse so the player's view does not move.
    //
    // YAW ONLY, deliberately: UEVR flattens its offset for the same reason. Compensating pitch
    // would fight the HMD, and pitch is already decoupled on this build.
    // Publish the yaw the GAME has gained since the reference. The actual cancellation happens in
    // on_pre_calculate_stereo_view_offset, which owns the rotation that is really rendered.
    //
    // set_rotation_offset() CANNOT do this here. On a normal UEVR title UEVR owns the view
    // rotation and the offset applies within it; on this title Blam owns the camera and UEVR
    // overlays HMD tracking on top, so the offset never cancels Blam's yaw.
    g_view_lock_deg = g_cfg.view_lock ? wrap180((float)aim_yaw - g_ref_aim_yaw.load()) : 0.0f;
    g_dbg_lock_deg = g_view_lock_deg.load();

    // ------------------------------------------------------------------ MOVEMENT FRAME
    // The angle between where the player is FACING and where the game's camera points. Blam moves
    // relative to the latter; the player expects the former. Published here and consumed in the
    // XInput hook, which is the only place the stick can be rewritten before the game reads it.
    // Taken from UEVR's FINAL rendered rotation, not rebuilt from parts: rebuilding it as
    // (pinned yaw + our turn + HMD yaw) means a VR->UE conversion and three terms that each have
    // to be signed correctly, with no way to tell a wrong sign from a wrong magnitude.
    // on_post_calculate_stereo_view_offset is dispatched AFTER UEVR composes HMD rotation into
    // the view (FFakeStereoRenderingHook.cpp:14294 vs the composition at ~14188), so it hands
    // over the finished view rotation ALREADY IN GAME SPACE -- the same space as aim_yaw. Both
    // sides of the subtraction come from the game, and the whole reconstruction disappears.
    // Every TERM is still published, not just the result: a single combined angle cannot
    // distinguish "wrong sign", "wrong magnitude" and "a term that is silently always zero" --
    // they look identical from the headset. With the parts logged, one deliberate test (turn
    // only the head, then only the controller) identifies which is which.
    {
        float hmd_yaw = 0.0f;
        Vec3 hpos{}; Quat hq{};
        const auto hidx = API::VR::get_hmd_index();
        if (hidx >= 0 && get_pose(hidx, &hpos, &hq, /*use_aim=*/false)) {
            float hp, hy, hr;
            const float ux = -hq.z, uy = hq.x, uz = hq.y, uw = -hq.w;   // VR -> UE
            quat_to_rotator(ux, uy, uz, uw, &hp, &hy, &hr);
            hmd_yaw = hy;
        }
        g_dbg_hmd_yaw = hmd_yaw;

        // Candidate A: UEVR's post-stereo rotation. Correct IF UEVR writes the HMD-composed value
        // back into view_rotation -- not confirmed in the source, hence the switch.
        const float view_a = g_render_view_yaw.load();
        // Candidate B: rebuilt from parts. Definitely contains the head term.
        const float view_b = g_locked_view_yaw.load() + g_turn_offset.load() + hmd_yaw;

        const float view_yaw = (g_cfg.move_src == 0 && g_have_render_yaw.load()) ? view_a : view_b;
        g_move_rot_deg = wrap180(view_yaw - (float)aim_yaw);
        g_dbg_view_a = view_a;
        g_dbg_view_b = view_b;

        // Low-pass the aim term used by the movement frame. Wrap-safe: filter the DELTA to the
        // current value, never the raw angles, or a pass through +-180 makes the filter sweep the
        // long way round and the player's movement snaps through a half circle.
        {
            const float a = clampf(g_cfg.move_smooth, 0.0f, 1.0f);
            if (!g_move_aim_primed.load() || a <= 0.0f) {
                g_move_aim_smooth = (float)aim_yaw;
                g_move_aim_primed = true;
            } else {
                const float cur = g_move_aim_smooth.load();
                g_move_aim_smooth = wrap180(cur + a * wrap180((float)aim_yaw - cur));
            }
        }

        // ---- MOVE PROBE: derive Blam's ACTUAL movement frame from observed motion.
        //
        // Given the stick vector we handed the game and the direction the camera actually travelled,
        //     blam_frame = actual_move_yaw - stick_angle_we_supplied
        // and whichever known angle that matches IS the frame Blam moves in:
        //   ~= aim   -> the assumption behind all of this is right, and any error is a sign
        //   ~= view  -> Blam already moves relative to the RENDERED view, so the correction is not
        //               needed at all and applying one is what breaks it
        //   ~= other -> neither, and the whole approach needs rebasing on whatever it is
        //
        // Sampled from the CameraComponent (the rig's attach parent), which is the camera itself.
        static Vec3  probe_pos{};
        static bool  probe_have = false;
        static uint32_t probe_tick = 0;

        const float lx = g_out_lx.load(), ly = g_out_ly.load();
        const float mag = std::sqrt(lx * lx + ly * ly);

        if (g_rig_parent != nullptr && mag > 0.5f) {
            Vec3 now{};
            if (call_ret_vec3(g_rig_parent, L"K2_GetComponentLocation", &now)) {
                if (probe_have && (tick - probe_tick) >= 12) {
                    const float dx = now.x - probe_pos.x, dy = now.y - probe_pos.y;
                    const float dist = std::sqrt(dx * dx + dy * dy);
                    if (dist > 15.0f) {   // ignore jitter and being wedged against geometry
                        const float actual   = std::atan2(dy, dx) * RAD2DEG;
                        const float stick_in = std::atan2(g_raw_lx.load(), g_raw_ly.load()) * RAD2DEG;
                        const float stick_out = std::atan2(lx, ly) * RAD2DEG;
                        const float frame    = wrap180(actual - stick_out);
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] PROBE stickIn=%.0f stickOut=%.0f actualMove=%.0f | BLAM_FRAME=%.0f "
                            "vs aim=%.0f view=%.0f | err(actual-view)=%.0f dist=%.0fcm",
                            stick_in, stick_out, actual, frame,
                            (float)aim_yaw, view_yaw, wrap180(actual - view_yaw), dist);
                    }
                    probe_pos = now; probe_tick = tick;
                } else if (!probe_have) {
                    probe_pos = now; probe_tick = tick; probe_have = true;
                }
            }
        } else {
            probe_have = false;   // stick released: restart the baseline
        }
    }

    // One-shot HUD discovery. Cheap: self-disables after the first successful dump.
    if (g_cfg.hud_dump && g_rig_component.load() != nullptr) dump_hud_visibility_once(tick);

    // Presentation only, and gated on gameplay inside -- so it runs after the aim has been read for
    // this tick but takes no part in producing it.
    // (reticle_rescan / menu_poll run at the top of update(), above the early-out gates.)
    // While the widget-RT source is active, probe the target's pixels periodically: a one-shot
    // probe at bind time raced the widget's first draw and read an empty target.
    if (g_cfg.aim_rt_from_widget) {
        static uint32_t last_probe = 0;
        if (tick - last_probe >= 600) {
            last_probe = tick;
            if (auto* wc = g_ret_widget_comp.get_checked(L"WidgetComponent")) {
                alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
                wc->call_function(L"GetRenderTarget", q);
                probe_render_target_pixels(*reinterpret_cast<API::UObject**>(q));
            }
        }
    }

    hud_reticle_follow((float)aim_pitch, (float)aim_yaw, tick);

    // ------------------------------------------------------------------ WEAPON RIG
    // Drives the FP rig from the same controller pose the aim loop uses, so the gun visually
    // follows the hand while aim follows the gun.
    if (g_cfg.rig_enabled) {
        // Re-resolve on a timer ALWAYS, not only when we hold nothing. A pointer to a recycled
        // component never becomes null -- it keeps accepting writes -- so "resolve once, cache
        // until null" can pin the driver to residue with no symptom other than nothing moving.
        if ((tick - g_rig_resolve_tick.load()) >= 60) {
            g_rig_resolve_tick = tick;
            API::UObject* found = nullptr;
            { PerfScope _perf(PERF_RIG); found = resolve_rig(); }
            auto* prev  = reinterpret_cast<API::UObject*>(g_rig_component.load());
            if (found != nullptr && found != prev) {
                g_rig_component = found;
                g_rig_neutral_valid = false;   // new rig => the old neutral is meaningless
                // Log the IDENTITY, not just "acquired": being able to compare the full object
                // name (e.g. ...BP_MeteoritePawn_C_<n>.BPC_FP_SkeletalMesh_C_<n>) is the
                // difference between knowing we bound to the live rig and assuming it.
                const std::string nm = (found->get_fname() != nullptr)
                                     ? narrow(found->get_fname()->to_string()) : "?";
                API::get()->log_info("[Halo-CampE-UEVR] rig acquired: %s %s",
                                     narrow(class_name_of(found)).c_str(), nm.c_str());

                g_rig_parent = follow_object(found, L"AttachParent");
                API::get()->log_info("[Halo-CampE-UEVR]   attach parent: %s",
                                     g_rig_parent != nullptr
                                        ? narrow(class_name_of(g_rig_parent)).c_str()
                                        : "<none - falling back to ControlRotation yaw>");

                // Read the grip point off the weapon socket. Re-read per rig because a different
                // weapon means a different grip.
                if (g_cfg.piv_auto && !g_pivot_from_calib) {
                    log_pivot_candidates(found);

                    wchar_t wsock[64] = {0};
                    MultiByteToWideChar(CP_UTF8, 0, g_cfg.piv_socket, -1, wsock, 63);

                    Vec3 piv{};
                    if (derive_pivot(found, wsock, &piv)) {
                        g_cfg.piv_x = piv.x + g_cfg.piv_adj_x;
                        g_cfg.piv_y = piv.y + g_cfg.piv_adj_y;
                        g_cfg.piv_z = piv.z + g_cfg.piv_adj_z;
                        API::get()->log_info("[Halo-CampE-UEVR] pivot = socket '%s' (%.1f,%.1f,%.1f) + adj (%.1f,%.1f,%.1f) = (%.1f,%.1f,%.1f) cm",
                                             g_cfg.piv_socket, piv.x, piv.y, piv.z,
                                             g_cfg.piv_adj_x, g_cfg.piv_adj_y, g_cfg.piv_adj_z,
                                             g_cfg.piv_x, g_cfg.piv_y, g_cfg.piv_z);
                    } else {
                        API::get()->log_info("[Halo-CampE-UEVR] pivot read FAILED for socket '%s' -- keeping piv=(%.1f,%.1f,%.1f); "
                                             "set pivauto=0 and pivx/pivy/pivz to override",
                                             g_cfg.piv_socket, g_cfg.piv_x, g_cfg.piv_y, g_cfg.piv_z);
                    }
                }
            }
        }

        // Manual recenter: any CHANGE to `recenter` re-captures the neutral at the current hand
        // position. Edge-triggered on the value, not level-triggered on a flag -- see the config.
        {
            static float last_recenter = 0.0f;
            static bool  seeded = false;
            if (!seeded) { last_recenter = g_cfg.recenter; seeded = true; }
            else if (g_cfg.recenter != last_recenter) {
                last_recenter = g_cfg.recenter;
                g_rig_neutral_valid = false;
                API::get()->log_info("[Halo-CampE-UEVR] recenter -> neutral will re-capture at the current hand pose");
            }
        }

        // Freeze on press: snapshot what the weapon looks like RIGHT NOW, then stop driving it.
        if (g_calib_start.exchange(false)) {
            g_calib_gun_world = g_last_gun_world;
            g_calib_off_world = g_last_off_world;
            g_calib_valid = true;
            API::get()->log_info("[Halo-CampE-UEVR] CALIBRATE: weapon frozen -- move your controller onto it, then release");
        }

        auto* rig = reinterpret_cast<API::UObject*>(g_rig_component.load());
        auto for_each_rig = [&](auto&& fn) { if (rig != nullptr) fn(rig); };

        // NOTE: the rig KEEPS BEING DRIVEN while calibrating -- it is pinned to a fixed WORLD
        // transform rather than left at a fixed RELATIVE one. See the calibration branch below.
        //
        // NOT while stick mode is engaged: whatever g_rig_component points at then is either a
        // stale recycled shell (vehicle seat) or a rig the game is not rendering -- and with
        // stickforce=1 on foot, freezing the arms is the honest picture of the stack being down.
        // RESOLUTION (above) keeps running either way; the route reviving is how stick mode ends.
        if (rig != nullptr && !g_stick_mode.load()) {
            // ROTATION -- must be RELATIVE TO THE PARENT, which is the aim.
            //
            // A scene component's relative transform COMPOSES on top of its parent, and the parent
            // here is oriented by the game's aim. Writing the controller's absolute orientation as
            // the relative value therefore produces
            //     gun_world = aim + controller
            // i.e. the controller rotation is counted twice, and because aim itself is converging
            // the composite drifts continuously -- gyro-like incremental rotation instead of the
            // gun matching the hand.
            //
            // In aim-error mode the relative value is (controller - aim), which the loop already
            // computes every tick. So:
            //   * aim caught up (err ~ 0) -> gun sits neutral on the rig, matching the hand
            //   * aim lagging            -> gun LEADS by the error, so it still points where the
            //                               hand points while the game's aim catches up behind it
            // That makes the gun feel absolutely tracked even though the aim actuator is a rate.
            //
            // Grip correction stays a LOCAL pitch twist on top; adding degrees to a single Euler
            // axis of a full 3-axis orientation fights gimbal, but here we are building the
            // rotator directly from two scalars so it is safe and readable.
            // The rig must represent the HAND, the aim logic must use the POINTING ray. Those are
            // two different orientations, and conflating them costs both placement and roll:
            //   * the AIM pose already carries the runtime's grip->aim tilt (and may normalise roll
            //     away entirely, so it cannot supply roll);
            //   * the GRIP pose is the actual controller orientation -- what a held object should
            //     match, and the only source with real roll in it.
            //
            // So instead of a hardcoded grip constant, MEASURE the tilt: it is simply
            // (grip - aim), available every tick because both poses are read. That self-calibrates
            // per controller type (Index/Touch/Vive differ) rather than baking in one headset's
            // constant.
            // ---- ROOM SPACE -> GAME SPACE, taken from UEVR rather than re-derived.
            //
            // Controller poses arrive in VR ROOM space, but the rig composes them against the
            // game's aim frame. Those differ by UEVR's rotation offset, which MUST be applied.
            // (Omitting it is masked by calibration, which absorbs the mismatch at whatever yaw
            // the player faced -- the weapon is then correct at that yaw, drifts as the body
            // turns, and converges again at the calibration yaw.)
            //
            // This is the same value UObjectHook uses for motion-controller attachments
            // (UObjectHook.cpp:1962), so it stays right through recentres and snap turns that a
            // hand-rolled equivalent would miss.
            Quat q_ro{0.0f, 0.0f, 0.0f, 1.0f};
            if (g_cfg.rig_view_yaw != 0.0f) {
                const auto ro = API::VR::get_rotation_offset();
                q_ro = Quat{ro.x, ro.y, ro.z, ro.w};
                if (g_cfg.rig_view_yaw < 0.0f) q_ro = quat_conj(q_ro);
            }

            float a_pitch, a_yaw, a_roll;
            {
                const float ux = -cq.z, uy = cq.x, uz = cq.y, uw = -cq.w;   // aim pose, VR -> UE
                quat_to_rotator(ux, uy, uz, uw, &a_pitch, &a_yaw, &a_roll);
            }
            float g_pitch = a_pitch, g_yaw = a_yaw, g_roll = a_roll;
            if (have_grip) {
                // rotation_offset is applied in VR SPACE, before the axis conversion -- the same
                // order as UObjectHook.cpp:2056 (`right_hand_rotation = rotation_offset * ...`).
                // Applying it after conversion would rotate about the wrong axis.
                const Quat gqo = quat_mul(q_ro, gq);
                const float ux = -gqo.z, uy = gqo.x, uz = gqo.y, uw = -gqo.w;   // VR -> UE
                quat_to_rotator(ux, uy, uz, uw, &g_pitch, &g_yaw, &g_roll);
            }

            // Measured grip->aim tilt, plus the manual `grip` trim on top for taste.
            const float tilt_pitch = wrap180(g_pitch - a_pitch);
            const float tilt_yaw   = wrap180(g_yaw   - a_yaw);

            // The rig's parent is carried by the game's aim, so that is the frame to divide out.
            // Built from ControlRotation, which we already read every tick and trust; roll is 0
            // because Blam's camera has none to contribute.
            // ---- THE PARENT FRAME, MEASURED RATHER THAN MODELLED.
            //
            // We write a RELATIVE transform, and the parent then re-applies its own rotation to it.
            // So the value to divide out is the parent's ACTUAL rotation -- read from the parent
            // component, not inferred from ControlRotation.
            //
            // DO NOT FLATTEN THIS. UEVR flattens (UObjectHook.cpp:1968) because it sets a WORLD
            // transform, where the parent never gets a second say. We do not: flattening leaves the
            // parent's real PITCH multiplying our offset, so the weapon's origin swings off the
            // pivot the moment the aim pitches. The flatten cannot be copied without its
            // world-vs-relative context.
            Quat q_parent = rotator_to_quat(0.0f, (float)aim_yaw, 0.0f);
            if (g_rig_parent != nullptr) {
                Vec3 prot{};
                if (call_ret_vec3(g_rig_parent, L"K2_GetComponentRotation", &prot)) {
                    // ---- STALENESS INSTRUMENTATION.
                    //
                    // The rig is a CHILD of this component, and we write it a RELATIVE rotation that
                    // is only correct for the parent orientation sampled right here. The parent then
                    // keeps turning for the rest of the tick interval while that relative value
                    // stays fixed, so the rig's WORLD orientation drifts by however far the parent
                    // moved, then snaps back on the next write. That sawtooth is the arm shake.
                    //
                    // Its amplitude is exactly |d(parent yaw)| per tick, which is what this logs.
                    // Screenshots cannot measure it -- a capture takes far longer than the ~31 ms
                    // interval, so the artifact is far above the sampling rate.
                    {
                        static float prev_pyaw = 0.0f;
                        static bool  have_prev = false;
                        static float worst = 0.0f;
                        static uint32_t last_rep = 0;
                        if (have_prev) {
                            const float d = std::fabs(wrap180(prot.y - prev_pyaw));
                            if (d > worst) worst = d;
                        }
                        prev_pyaw = prot.y; have_prev = true;
                        if (tick - last_rep >= 150) {
                            last_rep = tick;
                            API::get()->log_info(
                                "[Halo-CampE-UEVR] PARENT-YAW step: worst %.2f deg/tick over last window "
                                "(that is the arm swing amplitude between writes)", worst);
                            worst = 0.0f;
                        }
                    }

                    q_parent = rotator_to_quat(prot.x, prot.y, prot.z);   // pitch, yaw, roll
                }
            }

            // ---- ROOM SPACE -> GAME SPACE.
            //
            // Controller poses arrive in VR ROOM space, but the rig composes them against
            // q_parent, which is ControlRotation in GAME space. Those two frames differ by the
            // yaw the rendered view is pinned to -- the same value the stereo hook writes -- and
            // that difference must be applied. (Omitted, it is absorbed by calibration into the
            // grip trim and mount offset at whatever yaw the player faced: correct on release,
            // drifting as the body turns, converging again at the calibration yaw. A scale error
            // or a constant offset would not converge like that.)
            // The controller's orientation in UE space -- already carries rotation_offset, because
            // that was applied to the quaternion in VR space above. The hold trim is expressed in
            // the controller's own frame. Both are needed by the translation path too, so they
            // live out here rather than inside the mode-2 branch.
            // ---- SNAP / SMOOTH TURN must rotate the rig frame too.
            //
            // Our turn is applied in the stereo hook (it is added to the pinned view yaw), NOT via
            // UEVR's set_rotation_offset -- so `get_rotation_offset()` above knows nothing about
            // it, and without this term the rig frame does not turn with the world: every snap
            // leaves the weapon a step out, and turning back the other way cancels it exactly
            // (the signature of an accumulating, purely rotational term).
            //
            // The VIEW LOCK does not need the same treatment: it is pinned to a value that only
            // changes on re-anchor, so it is a constant that calibration absorbs. The turn offset
            // is not constant, which is why only snap turning exposes the fault.
            const Quat q_turn = rotator_to_quat(0.0f, g_cfg.rig_turn * g_turn_offset.load(), 0.0f);

            const Quat q_ctrl = quat_mul(q_turn, rotator_to_quat(g_pitch, g_yaw, g_roll));
            const Quat q_grip = rotator_to_quat(g_cfg.grip_deg, g_cfg.grip_yaw, g_cfg.grip_roll);


            // ---- CALIBRATION HOLD. Pin the weapon to the WORLD transform it had at freeze, by
            // re-deriving the relative transform against the CURRENT parent every tick:
            //     q_rel = inverse(q_parent_now) * q_gun_frozen
            //
            // Merely not driving the rig would freeze the RELATIVE transform instead -- the weapon
            // would still ride the parent, so the parent would have to be frozen too, which means
            // silencing the aim. That couples aim to this calibration: the release then
            // re-references it, editing the aim relationship as a side effect of adjusting the
            // mesh. Two unrelated calibrations must not share one gesture. Holding world-space
            // leaves the aim loop completely alone -- aim keeps tracking your hand while the
            // weapon stays put.
            const bool calibrating = g_calib_held.load() && g_calib_valid;

            float rig_pitch, rig_yaw, c_roll;
            if (calibrating) {
                const Quat q_rel = quat_mul(quat_conj(q_parent), g_calib_gun_world);
                quat_to_rotator(q_rel.x, q_rel.y, q_rel.z, q_rel.w, &rig_pitch, &rig_yaw, &c_roll);
            } else if (g_cfg.rig_mode == 2) {
                // q_rel = inverse(parent) * controller * trim. The trim multiplies on the RIGHT so
                // it is applied in the CONTROLLER's local frame -- a constant wrist angle at any
                // orientation. Adding degrees to a Euler axis instead fights gimbal near vertical.
                const Quat q_rel  = quat_mul(quat_conj(q_parent), quat_mul(q_ctrl, q_grip));
                quat_to_rotator(q_rel.x, q_rel.y, q_rel.z, q_rel.w, &rig_pitch, &rig_yaw, &c_roll);
            } else if (g_cfg.rig_mode == 1) {
                rig_pitch = clampf(g_pitch, -89.0f, 89.0f) + g_cfg.grip_deg;
                rig_yaw   = wrap180(g_yaw);
                c_roll    = g_roll;
            } else {
                rig_pitch = clampf(g_dbg_err_pitch.load() + tilt_pitch, -89.0f, 89.0f) + g_cfg.grip_deg;
                rig_yaw   = wrap180(g_dbg_err_yaw.load() + tilt_yaw);
                c_roll    = g_roll;   // roll from the HAND -- the aim ray has none to give
            }
            // ---- PIVOT MARKER. Shrink the rig and zero its rotation so the remaining blob marks
            // the pivot exactly. Scale is written only on the TRANSITION, not every tick: it never
            // varies, and restoring it on the way out is what stops the rig being left shrunk.
            {
                static bool viz_applied = false;
                if (g_cfg.piv_viz != viz_applied) {
                    viz_applied = g_cfg.piv_viz;
                    const double s = g_cfg.piv_viz ? (double)g_cfg.piv_viz_scale : 1.0;
                    for_each_rig([&](API::UObject* r) { rig_set_scale(r, s); });
                    API::get()->log_info("[Halo-CampE-UEVR] pivot marker %s (rig scale %.2f)",
                                         g_cfg.piv_viz ? "ON" : "off", s);
                }
                if (g_cfg.piv_viz) { rig_pitch = 0.0f; rig_yaw = 0.0f; c_roll = 0.0f; }
            }

            // ---- MODE 1: hand the arms to UEVR and stop driving the transform ourselves.
            // The two mechanisms MUST NOT both run: UObjectHook writes the component's world
            // transform per-eye while we would be writing its relative transform per-tick, and the
            // result is a fight whose winner depends on frame timing.
            if (g_cfg.attach_mode == 1) {
                if (!g_attached) {
                    attach_apply(rig, q_grip, Vec3{g_cfg.off_x, g_cfg.off_y, g_cfg.off_z});
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] UObjectHook ATTACH: hand=RIGHT permanent=%d rot=(%.3f,%.3f,%.3f,%.3f) loc=(%.1f,%.1f,%.1f)cm",
                        (int)g_cfg.attach_permanent, q_grip.x, q_grip.y, q_grip.z, q_grip.w,
                        g_cfg.off_x, g_cfg.off_y, g_cfg.off_z);
                } else if (tick % 64 == 0) {
                    // Re-apply periodically so calibration edits take effect live, and so a
                    // re-acquired component gets its offsets back.
                    attach_apply(rig, q_grip, Vec3{g_cfg.off_x, g_cfg.off_y, g_cfg.off_z});
                }
                g_dbg_rig_roll = c_roll;
            } else {

            if (g_attached) attach_release(rig, "attachmode switched back to 0");

            g_dbg_rig_roll = c_roll;
            for_each_rig([&](API::UObject* r) {
                rig_set_rotation(r, (double)rig_pitch, (double)rig_yaw, (double)c_roll);
            });

            // The weapon's ACTUAL world rotation: parent composed with what we just wrote. Derived
            // rather than assumed so it stays correct in every rig mode and while calibrating, and
            // it is what both the pivot arm and a freeze snapshot must use.
            const Quat q_gun = quat_mul(q_parent, rotator_to_quat(rig_pitch, rig_yaw, c_roll));
            g_last_gun_world = q_gun;

            // Publish for the render-rate re-apply (see rig_render).
            g_rigw_x = q_gun.x; g_rigw_y = q_gun.y; g_rigw_z = q_gun.z; g_rigw_w = q_gun.w;
            g_rigw_valid = true;
            {
                // Yaw of the parent this target was built against, so the render side can report
                // how stale it had become by the time it wrote. Derived from q_parent so it is
                // correct in every rig mode, not just the one that reads the component directly.
                float pp = 0.0f, py = 0.0f, pr = 0.0f;
                quat_to_rotator(q_parent.x, q_parent.y, q_parent.z, q_parent.w, &pp, &py, &pr);
                g_rigw_parent_yaw = py;
            }

            // TRANSLATION. Written and then READ BACK, so a silent no-op is distinguishable from
            // a real write: K2_SetRelativeTransform is a known no-op on this stack, so
            // K2_SetRelativeLocation is never assumed to apply.
            // Static-offset probe. Bypasses ALL pose/neutral/clamp math so the only question left
            // is whether writing RelativeLocation moves this mesh on screen.
            if (g_cfg.rig_test_cm != 0.0f) {
                for_each_rig([&](API::UObject* r) {
                    rig_set_location(r, 0.0, 0.0, (double)g_cfg.rig_test_cm);
                });
                g_dbg_rig_x = 0.0f; g_dbg_rig_y = 0.0f; g_dbg_rig_z = g_cfg.rig_test_cm;
                g_rig_wrote_once = true;
            }
            else if (g_cfg.rig_loc) {
                // Neutral is captured at the SAME moment as the aim reference, not independently:
                // capturing on "first valid pose" alone can land on a transient position, leaving
                // the offset pinned at the clamp from a neutral that was never where the hand was.
                // Tying it to g_have_ref guarantees a moment with a valid pose AND valid
                // ControlRotation, and it re-captures on every re-anchor (level load, snap turn).
                // BODY-ANCHORED: measure the hand relative to the HEAD, not to a fixed point in the
                // room. Without this, walking anywhere drags the gun to the clamp and holds it
                // there. Subtracting the head leaves only hand-relative-to-body motion, which is
                // the only part a held object should follow.
                Vec3 hand = rigpos;
                if (g_cfg.rig_body_anchor) {
                    Vec3 hpos{}; Quat hq{};
                    const auto hidx = API::VR::get_hmd_index();
                    if (hidx >= 0 && get_pose(hidx, &hpos, &hq, /*use_aim=*/false)) {
                        hand = Vec3{rigpos.x - hpos.x, rigpos.y - hpos.y, rigpos.z - hpos.z};
                    }
                }

                if (!g_rig_neutral_valid.load() && g_have_ref.load()) {
                    g_rig_neutral_x = hand.x; g_rig_neutral_y = hand.y; g_rig_neutral_z = hand.z;
                    g_rig_neutral_valid = true;
                }

                if (!g_rig_neutral_valid.load()) {
                    // No trustworthy neutral yet: hold the rig at origin rather than fling it to
                    // the clamp. A wrong offset is far more visible than no offset.
                    for_each_rig([&](API::UObject* r) { rig_set_location(r, 0.0, 0.0, 0.0); });
                } else {
                // ---- Does our write SURVIVE THE FRAME?
                // Reading back immediately after writing only proves the write LANDED. If the
                // attachment or anim system recomputes this component's location later in the
                // frame, our value is gone before anything is drawn and the immediate readback
                // still reports success. So sample the property at the START of this tick,
                // i.e. one full frame after we last wrote it. If it does not still hold roughly
                // what we wrote, something is resetting it and RelativeLocation is the wrong lever
                // for translation -- "the property holds it but nothing moves".
                {
                    auto* prev = rig->get_property_data<double>(L"RelativeLocation");
                    if (prev != nullptr && g_rig_wrote_once.load()) {
                        const float dxr = (float)prev[0] - g_dbg_rig_x.load();
                        const float dyr = (float)prev[1] - g_dbg_rig_y.load();
                        const float dzr = (float)prev[2] - g_dbg_rig_z.load();
                        const float drift = std::sqrt(dxr * dxr + dyr * dyr + dzr * dzr);
                        if (drift > g_rig_survive_drift.load()) g_rig_survive_drift = drift;
                    }
                }

                // Raw travel from neutral, BEFORE scale/clamp -- the honest measure of whether the
                // runtime is giving us position at all.
                {
                    const float tx = hand.x - g_rig_neutral_x.load();
                    const float ty = hand.y - g_rig_neutral_y.load();
                    const float tz = hand.z - g_rig_neutral_z.load();
                    const float t = std::sqrt(tx * tx + ty * ty + tz * tz);
                    if (t > g_ctrl_travel_max.load()) g_ctrl_travel_max = t;
                }

                // Order matters and follows UObjectHook.cpp:2047:
                //   1. hand relative to the head, in VR space
                //   2. rotated by rotation_offset, still in VR space
                //   3. scaled by world_to_meters * world_scale
                //   4. converted VR (Y up, -Z fwd) -> UE (Z up, +X fwd)
                // Rotating after the axis conversion instead applies the offset about the wrong
                // axis entirely.
                // !!! NO NEUTRAL BY DEFAULT -- UEVR subtracts none (UObjectHook.cpp:2047 uses the
                // raw head-relative offset), and subtracting one is actively wrong here.
                //
                // `hand` ROTATES about the head as the body turns; `neutral` is a FIXED room-space
                // vector. (rotating - constant) is not a rotation: it is a rotation whose CENTRE is
                // displaced by `neutral` -- the centre of rotation sits off the player, and the
                // pivot does not sit on the grip, because the constant shifts the arc the weapon
                // travels.
                //
                // The constant a neutral would stand in for is the mount offset, which calibration
                // already solves properly.
                Vec3 d_vr = hand;
                if (g_cfg.rig_neutral) {
                    d_vr = Vec3{hand.x - g_rig_neutral_x.load(),
                                hand.y - g_rig_neutral_y.load(),
                                hand.z - g_rig_neutral_z.load()};
                }
                d_vr = quat_rotate(q_ro, d_vr);

                const float dx = d_vr.x * g_cfg.rig_scale;
                const float dy = d_vr.y * g_cfg.rig_scale;
                const float dz = d_vr.z * g_cfg.rig_scale;
                // Same turn correction as the orientation -- if only one of them gets it they
                // disagree and the weapon slides sideways with every snap.
                Vec3 pose_off = quat_rotate(q_turn, Vec3{-dz, dx, dy});

                // Optional travel clamp, OFF by default (rigclamp<=0).
                //
                // PER-AXIS CLAMPING DISTORTS DIRECTION, not just magnitude: as soon as one axis
                // saturates the offset vector ROTATES, so a diagonal hand movement no longer maps
                // to a diagonal weapon movement -- it reads as motion "slower or not properly
                // mapped in certain directions" rather than as a hard limit. If a bound is ever
                // wanted, clamp the LENGTH of the vector and leave its direction alone.
                //
                // Applied before the solve when enabled, so the solve sees the value that will
                // actually be used.
                if (g_cfg.rig_clamp > 0.0f) {
                    pose_off.x = clampf(pose_off.x, -g_cfg.rig_clamp, g_cfg.rig_clamp);
                    pose_off.y = clampf(pose_off.y, -g_cfg.rig_clamp, g_cfg.rig_clamp);
                    pose_off.z = clampf(pose_off.z, -g_cfg.rig_clamp, g_cfg.rig_clamp);
                }

                // The pivot arm for THIS frame's weapon orientation.
                const Vec3 G{g_cfg.piv_x, g_cfg.piv_y, g_cfg.piv_z};

                // ---- CALIBRATION RELEASE: solve the placement that puts the weapon where the
                // hand is. The model being solved is
                //     off = pose + C - R*G
                // so with R pinned to the frozen orientation by the rotation trim above,
                //     C = off_frozen - pose + R_frozen*G
                if (g_calib_finish.exchange(false) && g_calib_valid) {
                    const Quat q_grip_new = quat_mul(quat_conj(q_ctrl), g_calib_gun_world);
                    quat_to_rotator(q_grip_new.x, q_grip_new.y, q_grip_new.z, q_grip_new.w,
                                    &g_cfg.grip_deg, &g_cfg.grip_yaw, &g_cfg.grip_roll);

                    // ONE SAMPLE IS SUFFICIENT. A held weapon is a RIGID ATTACHMENT, which has no
                    // free pivot parameter: physics fixes the rotation centre at the controller
                    // origin. Solving for a floating pivot invents a degree of freedom that does
                    // not exist, and any value found really absorbs a translation-SCALE error.
                    //
                    //   off = pose + R_ctrl*L - R_gun*G
                    // with G normally 0, so:
                    //   L = inverse(R_ctrl) * (off_frozen - pose + R_gun_frozen*G)
                    const Vec3 G_now{g_cfg.piv_x, g_cfg.piv_y, g_cfg.piv_z};
                    const Vec3 arm_frozen = quat_rotate(g_calib_gun_world, G_now);
                    const Vec3 resid{g_calib_off_world.x - pose_off.x + arm_frozen.x,
                                     g_calib_off_world.y - pose_off.y + arm_frozen.y,
                                     g_calib_off_world.z - pose_off.z + arm_frozen.z};
                    const Vec3 L = quat_rotate(quat_conj(q_ctrl), resid);
                    g_cfg.off_x = clampf(L.x, -100.0f, 100.0f);
                    g_cfg.off_y = clampf(L.y, -100.0f, 100.0f);
                    g_cfg.off_z = clampf(L.z, -100.0f, 100.0f);

                    // Deliberately NOT touching g_have_ref. The aim loop ran normally throughout,
                    // so its hand-to-aim mapping is still valid -- re-referencing here would
                    // silently re-calibrate aim as a side effect of a mesh adjustment.
                    write_calib_file();
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] CALIBRATED: grip=%.1f gripyaw=%.1f griproll=%.1f  mount=(%.1f,%.1f,%.1f)cm controller-local",
                        g_cfg.grip_deg, g_cfg.grip_yaw, g_cfg.grip_roll,
                        g_cfg.off_x, g_cfg.off_y, g_cfg.off_z);
                }

                // RIGID ATTACHMENT:  off = pose + R_ctrl*L - R_gun*G
                //
                // The mount offset L rotates WITH THE CONTROLLER -- that is what makes this a rigid
                // attachment rather than a weapon sliding around on a world-aligned offset, and it
                // puts the rotation centre at the controller origin where a held object's is.
                // G stays 0 unless someone deliberately overrides the pivot.
                const Vec3 mount = quat_rotate(q_ctrl, Vec3{g_cfg.off_x, g_cfg.off_y, g_cfg.off_z});
                const Vec3 arm   = quat_rotate(q_gun, G);
                Vec3 off{pose_off.x + mount.x - arm.x,
                         pose_off.y + mount.y - arm.y,
                         pose_off.z + mount.z - arm.z};

                // Marker mode: drop BOTH the mount offset and the pivot arm so the component origin
                // lands on the pivot itself -- the point the weapon rotates about.
                if (g_cfg.piv_viz) off = pose_off;

                // While calibrating, hold the exact world offset captured at freeze.
                if (calibrating) off = g_calib_off_world;

                g_last_off_world = off;

                // ---- PIVOT MARKER, drawn rather than acted out.
                //
                // The component's world position is (parent_origin + off), because the relative
                // value we write is conj(q_parent)*off and the parent then rotates it back. So
                //     parent_origin = comp_world - off
                //     pivot_world   = parent_origin + pose = comp_world + (pose - off)
                // which needs exactly one live read: the component's own world location.
                // NOTE: the reticule must be in this condition too -- gated behind the pivot
                // markers alone, turning the pivot cube off silently disables it, which reads as
                // "the reticule is too small to see" rather than "never ran".
                if (g_cfg.piv_draw > 0.0f || g_cfg.piv_cube || g_cfg.aim_reticule) {
                    Vec3 comp_world{};
                    if (call_ret_vec3(rig, L"K2_GetComponentLocation", &comp_world)) {
                        static bool logged_once = false;
                        if (!logged_once) {
                            logged_once = true;
                            API::get()->log_info("[Halo-CampE-UEVR] component getter WORKS: comp_world=(%.1f, %.1f, %.1f)",
                                                 comp_world.x, comp_world.y, comp_world.z);
                        }
                        const Vec3 pivot_world{comp_world.x + pose_off.x - off.x,
                                               comp_world.y + pose_off.y - off.y,
                                               comp_world.z + pose_off.z - off.z};
                        if (g_cfg.piv_draw > 0.0f) {
                            // Kept, but stripped in shipping -- see draw_debug_sphere().
                            draw_debug_sphere(rig, pivot_world, g_cfg.piv_draw, 0.0f, 1.0f, 0.0f);
                            draw_debug_sphere(rig, comp_world, g_cfg.piv_draw * 0.5f, 1.0f, 0.0f, 0.0f);
                        }

                        // The cube: acquire on first use, park it on the pivot, release on the way
                        // out so the level is not left with a shrunken prop stuck to the player.
                        if (g_cfg.piv_cube) {
                            if (!g_pivot_marker.active) {
                                g_pivot_marker.active = resolve_marker(g_pivot_marker, g_cfg.piv_cube_scale,
                                                                       comp_world, "pivot cube",
                                                                       g_aim_marker.actor);
                                if (!g_pivot_marker.active) g_cfg.piv_cube = false;   // do not retry every tick
                            }
                            if (g_pivot_marker.active) park_marker(g_pivot_marker, pivot_world);
                        } else if (g_pivot_marker.active) {
                            release_marker(g_pivot_marker, "pivot cube");
                        }

                        // ---- VR RETICULE, on the AIM RAY.
                        //
                        // Halo's own HUD reticle sits at the centre of the GAME camera, but the
                        // view lock pins our rendered yaw somewhere else, so it never lines up.
                        //
                        // Approach taken from uevrlib's reticule module: do not fight the game's
                        // HUD projection, draw a separate world-space reticule along the true ray.
                        // The aim ray is ControlRotation, which is the exact quantity the aim loop
                        // steers and the exact thing shots follow -- so aligning the weapon to this
                        // marker is unambiguous even on weapons with no usable sights.
                        if (g_cfg.aim_reticule) {
                            Vec3 origin{};
                            const bool have_origin = (g_rig_parent != nullptr)
                                && call_ret_vec3(g_rig_parent, L"K2_GetComponentLocation", &origin);

                            if (have_origin) {
                                // Only borrow a prop if the cube path is actually wanted -- otherwise
                                // the map quietly loses a piece of set dressing for nothing.
                                if (g_cfg.aim_reticule_cube && !g_aim_marker.active) {
                                    g_aim_marker.active = resolve_marker(g_aim_marker, g_cfg.aim_reticule_cm,
                                                                          comp_world, "aim reticule",
                                                                          g_pivot_marker.actor);
                                    if (!g_aim_marker.active) g_cfg.aim_reticule_cube = false;
                                } else if (!g_cfg.aim_reticule_cube && g_aim_marker.active) {
                                    release_marker(g_aim_marker, "aim reticule");
                                }
                                // NOT gated on g_aim_marker.active (the BORROWED LEVEL PROP): with
                                // aimreticulecube=0 the marker is never acquired, so gating the
                                // ray, the Lua publish or the world-space draw on it silently
                                // disables them all -- the same trap as gating on piv_cube above.
                                // Only park_marker belongs under that condition.
                                {
                                    // UE forward from the aim rotator.
                                    const float cp = std::cos((float)aim_pitch * DEG2RAD);
                                    const Vec3 fwd{cp * std::cos((float)aim_yaw * DEG2RAD),
                                                   cp * std::sin((float)aim_yaw * DEG2RAD),
                                                   std::sin((float)aim_pitch * DEG2RAD)};
                                    const float d = g_cfg.aim_reticule_dist;
                                    const Vec3 target{origin.x + fwd.x * d,
                                                      origin.y + fwd.y * d,
                                                      origin.z + fwd.z * d};

                                    // ---- PUBLISH THE RAY TO LUA (the real reticule).
                                    //
                                    // Borrowing a level prop is only a diagnostic: it steals
                                    // set dressing, inherits whatever mesh happens to be nearby,
                                    // and cannot ship. uevrlib's reticule module builds its OWN
                                    // sphere mesh, so the Lua side owns the visual and this side
                                    // just supplies geometry.
                                    //
                                    // plugin -> Lua is the direction that actually exists in the
                                    // API (dispatch_lua_event); there is no Lua -> plugin channel,
                                    // which is why the split is this way round and not the reverse.
                                    if (g_cfg.aim_reticule_lua) {
                                        char json[256];
                                        snprintf(json, sizeof(json),
                                                 "{\"ox\":%.1f,\"oy\":%.1f,\"oz\":%.1f,"
                                                 "\"tx\":%.1f,\"ty\":%.1f,\"tz\":%.1f,\"dist\":%.1f}",
                                                 origin.x, origin.y, origin.z,
                                                 target.x, target.y, target.z, d);
                                        API::get()->dispatch_lua_event("halo_aim_ray", json);


                                        // Count it, and say so periodically. This site sits five
                                        // conditions deep (rig_loc -> the piv/reticule group ->
                                        // a successful component read -> aim_reticule -> a valid
                                        // origin), so without the counter "no reticule appeared"
                                        // could equally mean the plugin never published or the
                                        // script never drew. This makes the two distinguishable.
                                        static uint32_t dispatches = 0;
                                        static uint32_t last_rep = 0;
                                        ++dispatches;
                                        if (tick - last_rep >= 600) {
                                            last_rep = tick;
                                            API::get()->log_info(
                                                "[Halo-CampE-UEVR] aim ray dispatched to Lua: %u so far, latest %s",
                                                dispatches, json);
                                        }
                                    }

                                    // THE RETICULE. World space, so it has no HUD bounds, and its
                                    // own geometry, so nothing is taken from the level -- the two
                                    // constraints the borrowed-prop marker cannot meet.
                                    // The game's own crosshair, in world space -- brings the
                                    // per-weapon art and the hit marker with it.
                                    if (g_cfg.aim_widget) {
                                        reticule_widget_ensure(rig);
                                        reticule_widget_move(target, origin);
                                    }

                                    // Plain sphere. Kept as the fallback that is known to render.
                                    // NOT gated on aim_mesh: the hide path lives inside
                                    // reticule_mesh_move, so gating the call would leave the quad
                                    // frozen and visible at its last position whenever the
                                    // feature is switched off.
                                    g_ret_origin = origin;
                                    g_have_ret_origin = true;
                                    if (g_cfg.aim_mesh) reticule_mesh_ensure(rig);

                                    // Park on the view axis for measurement runs (see aim_park_view).
                                    Vec3 mesh_target = target;
                                    if (g_cfg.aim_park_view && g_have_render_yaw.load()) {
                                        const float vy = g_render_view_yaw.load();
                                        const float vp = g_render_view_pitch.load();
                                        const float cv = std::cos(vp * DEG2RAD);
                                        mesh_target = Vec3{origin.x + cv * std::cos(vy * DEG2RAD) * d,
                                                           origin.y + cv * std::sin(vy * DEG2RAD) * d,
                                                           origin.z + std::sin(vp * DEG2RAD) * d};
                                    }
                                    reticule_mesh_move(mesh_target);

                                    if (g_cfg.aim_draw) {
                                        draw_debug_sphere(rig, target, g_cfg.aim_draw_r,
                                                          g_cfg.aim_draw_cr, g_cfg.aim_draw_cg,
                                                          g_cfg.aim_draw_cb,
                                                          g_cfg.aim_draw_dur, g_cfg.aim_draw_seg,
                                                          g_cfg.aim_draw_th);

                                        static uint32_t last_draw_rep = 0;
                                        if (tick - last_draw_rep >= 600) {
                                            last_draw_rep = tick;
                                            API::get()->log_info(
                                                "[Halo-CampE-UEVR] reticule drawn at (%.0f,%.0f,%.0f) r=%.1f dur=%.3f",
                                                target.x, target.y, target.z,
                                                g_cfg.aim_draw_r, g_cfg.aim_draw_dur);
                                        }
                                    }

                                    if (g_cfg.aim_reticule_cube) park_marker(g_aim_marker, target);

                                    // Read back where it actually landed. On this title "the call
                                    // succeeded" does not mean "the object moved", so the only
                                    // trustworthy check is the object's own reported location.
                                    if (tick % 600 == 0 && marker_alive(g_aim_marker)) {
                                        Vec3 actual{};
                                        if (call_ret_vec3(g_aim_marker.actor, L"K2_GetActorLocation", &actual)) {
                                            const float ex = actual.x + g_aim_marker.center_off.x - target.x;
                                            const float ey = actual.y + g_aim_marker.center_off.y - target.y;
                                            const float ez = actual.z + g_aim_marker.center_off.z - target.z;
                                            API::get()->log_info(
                                                "[Halo-CampE-UEVR]   reticule target=(%.0f,%.0f,%.0f) err=%.1f cm  aim=(%.1f,%.1f)",
                                                target.x, target.y, target.z,
                                                std::sqrt(ex * ex + ey * ey + ez * ez),
                                                (float)aim_pitch, (float)aim_yaw);
                                        }
                                    }
                                }
                            }
                        } else if (g_aim_marker.active) {
                            release_marker(g_aim_marker, "aim reticule");
                        }
                    } else {
                        static bool warned = false;
                        if (!warned) {
                            warned = true;
                            API::get()->log_info("[Halo-CampE-UEVR] K2_GetComponentLocation FAILED -- cannot place the "
                                                 "pivot marker in world space (this also explains the socket reads)");
                        }
                    }
                }

                // Same frame correction as the rotation: the offset is a WORLD-space displacement,
                // but RelativeLocation is interpreted in the parent's frame, which the aim rotates.
                // Without this, "move my hand right" means right-of-the-parent rather than right in
                // the world, so the gun swings differently depending on which way you are facing --
                // half of what makes the controls feel wrong.
                if (g_cfg.rig_mode == 2) off = quat_rotate(quat_conj(q_parent), off);

                // rigclamp was already applied to the pose part above. What remains here is only a
                // sanity rail so a bad solve cannot fling the weapon out of the world -- it must
                // stay well clear of any legitimate mount offset or it becomes the same bug again.
                constexpr float SANITY_CM = 200.0f;
                const float ex = clampf(off.x, -SANITY_CM, SANITY_CM);
                const float ey = clampf(off.y, -SANITY_CM, SANITY_CM);
                const float ez = clampf(off.z, -SANITY_CM, SANITY_CM);

                g_dbg_rig_x = ex; g_dbg_rig_y = ey; g_dbg_rig_z = ez;
                g_dbg_pos_x = rigpos.x; g_dbg_pos_y = rigpos.y; g_dbg_pos_z = rigpos.z;
                for_each_rig([&](API::UObject* r) {
                    rig_set_location(r, (double)ex, (double)ey, (double)ez);
                });
                g_rig_wrote_once = true;

                // Did it stick? Compare what we asked for against the component's own field.
                auto* rl = rig->get_property_data<double>(L"RelativeLocation");
                if (rl != nullptr && (std::fabs(ex) + std::fabs(ey) + std::fabs(ez)) > 1.0f) {
                    const float applied = (float)(std::fabs(rl[0]) + std::fabs(rl[1]) + std::fabs(rl[2]));
                    const int verdict = (applied > 0.5f) ? 1 : 0;
                    if (g_rig_loc_works.load() != verdict) {
                        g_rig_loc_works = verdict;
                        API::get()->log_info("[Halo-CampE-UEVR] rig LOCATION %s (asked %.1f,%.1f,%.1f  read %.1f,%.1f,%.1f)",
                            verdict ? "APPLIES" : "IS A NO-OP", ex, ey, ez,
                            (float)rl[0], (float)rl[1], (float)rl[2]);
                    }
                }
                }   // end: neutral valid
            }
            }   // end: attachmode 0 -- the rig-driver path
        }
    }

    // ------------------------------------------------------------------ TURNING
    // Consumes the player's raw right-stick X (sampled in the XInput hook before the aim value
    // replaced it). Adjusts the locked view yaw, so the world turns while aim stays on the gun.
    // Stands down in stick mode: the right stick IS the game's look input there, and consuming it
    // for snap turn would rotate the player twice.
    if (g_cfg.turn_mode != 0 && !g_stick_mode.load()) {
        const float sx = g_raw_stick_x.load();
        const bool past_dz = std::fabs(sx) > g_cfg.turn_dz;

        if (g_cfg.turn_mode == 1) {
            // SNAP: one step per flick. Requires returning inside the deadzone before re-firing,
            // otherwise holding the stick spins continuously -- which is the nausea this avoids.
            if (past_dz && !g_snap_latched.load()) {
                // WRAP. Without this the offset accumulates without bound (25 snaps at 30 deg is
                // already +750). Every consumer that wraps hides this; every consumer that does
                // not inherits a wildly out-of-range angle, and the rendered yaw is written raw.
                g_turn_offset = wrap180(g_turn_offset.load() + ((sx > 0.0f) ? g_cfg.snap_deg : -g_cfg.snap_deg));
                g_snap_latched = true;
            } else if (!past_dz) {
                g_snap_latched = false;
            }
        } else {
            // SMOOTH: scale past the deadzone so there is no jump at the threshold.
            if (past_dz) {
                const float t = (std::fabs(sx) - g_cfg.turn_dz) / (1.0f - g_cfg.turn_dz);
                const float dir = (sx > 0.0f) ? 1.0f : -1.0f;
                // Wrapped for the same reason as the snap path: smooth turn accumulates far faster.
                g_turn_offset = wrap180(g_turn_offset.load() + dir * t * g_cfg.smooth_dps * g_last_dt.load());
            }
        }
    }

    if ((g_ticks.load() % 600) == 0) {
        API::get()->log_info("[Halo-CampE-UEVR]   move: rot=%.1f applied=%u src=%d | viewA(uevr)=%.1f viewB(rebuilt)=%.1f "
                             "hmd=%.1f pinned=%.1f turn=%.1f aim=%.1f",
                             g_move_rot_deg.load(), g_move_applied.load(), g_cfg.move_src,
                             g_dbg_view_a.load(), g_dbg_view_b.load(), g_dbg_hmd_yaw.load(),
                             g_locked_view_yaw.load(), g_turn_offset.load(), (float)aim_yaw);
        API::get()->log_info("[Halo-CampE-UEVR] t%u ctrl=%.1f aim=%.1f err=%.1f stick=%.2f | viewIn=%.1f viewOut=%.1f turn=%.1f JUDDER=%.2f%s",
            g_ticks.load(), ctrl_yaw, (float)aim_yaw, g_dbg_err_yaw.load(), g_out_rx.load(),
            g_dbg_view_in.load(), g_dbg_view_out.load(), g_turn_offset.load(), g_judder_max.load(),
            g_stick_mode.load() ? "  [STICK MODE]" : "");
        API::get()->log_info("[Halo-CampE-UEVR]   rig: travel=%.3fm rigOff=(%.1f,%.1f,%.1f)cm roll=%.1f | pos=(%.3f,%.3f,%.3f) SURVIVE_DRIFT=%.2fcm",
            g_ctrl_travel_max.load(), g_dbg_rig_x.load(), g_dbg_rig_y.load(), g_dbg_rig_z.load(),
            g_dbg_rig_roll.load(), g_dbg_pos_x.load(), g_dbg_pos_y.load(), g_dbg_pos_z.load(),
            g_rig_survive_drift.load());
        g_ctrl_travel_max = 0.0f;      // per-window, so a single early jump stops dominating
        g_rig_survive_drift = 0.0f;
        g_judder_max = 0.0f;   // report peak-since-last-log, not all-time
    }
}

SHORT to_raw(float v) {
    if (std::fabs(v) < 1e-3f) return 0;
    float raw = std::fabs(v) * 32767.0f;
    if (raw > 32767.0f) raw = 32767.0f;
    return (SHORT)(v < 0.0f ? -raw : raw);
}

} // namespace

class HaloAimDriverPlugin : public uevr::Plugin {
public:
    void on_initialize() override {
        // Config lives beside the UEVR profile so it is where a user would look for it.
        char appdata[MAX_PATH] = {0};
        DWORD n = GetEnvironmentVariableA("APPDATA", appdata, MAX_PATH);
        if (n > 0 && n < MAX_PATH) {
            sprintf_s(g_cfg_path, MAX_PATH,
                      "%s\\UnrealVRMod\\HaloCampaignEvolved\\halo_vr.cfg", appdata);
            sprintf_s(g_calib_path, MAX_PATH,
                      "%s\\UnrealVRMod\\HaloCampaignEvolved\\halo_vr_calib.cfg", appdata);
        } else {
            strcpy_s(g_cfg_path, MAX_PATH, "halo_vr.cfg");
            strcpy_s(g_calib_path, MAX_PATH, "halo_vr_calib.cfg");
        }
        load_config();   // writes a commented default file if none exists

        // Version first, on its own line: this is what a bug report needs to be actionable, and it
        // must survive even if the settings line below changes shape.
        API::get()->log_info("[Halo-CampE-UEVR] Halo: Campaign Evolved VR  v%s  (halo_vr.dll)",
                             HALO_VR_VERSION);
        API::get()->log_info("[Halo-CampE-UEVR] closed-loop controller aim. "
                             "enabled=%d floor=%.2f full=%.1fdeg max=%.2f xdist=%.0fm fake_pad=%d",
                             (int)g_cfg.enabled, g_cfg.floor, g_cfg.full_deg, g_cfg.max_out,
                             g_cfg.xdist_m, (int)g_cfg.fake_pad);
        API::get()->log_info("[Halo-CampE-UEVR] KILL SWITCH: set enabled=0 in %s (re-read every ~2s)", g_cfg_path);
    }

    void on_pre_engine_tick(API::UGameEngine*, float delta) override {
        if (delta > 0.0f && delta < 1.0f) g_last_dt = delta;
        update();
    }

    // VIEW LOCK -- the enforcement point. This callback owns the rotation that is actually used
    // to render the stereo view, so replacing the yaw here keeps the headset view still no matter
    // what Blam does to its camera. set_rotation_offset() cannot do this: on this title Blam
    // owns the camera and UEVR only overlays HMD tracking, so the offset never cancels Blam yaw.
    //
    // RENDER THREAD. Reads one pre-computed scalar and adjusts one field. No reflection, no
    // allocation, no logging -- same rule as the XInput callback.
    //
    // is_double: the header states the pointer must be interpreted per this flag, and this game
    // is double-precision (LWC). Treating a double rotator as floats would corrupt the view.
    void on_pre_calculate_stereo_view_offset(UEVR_StereoRenderingDeviceHandle, int index, float,
                                             UEVR_Vector3f*, UEVR_Rotatorf* rotation,
                                             bool is_double) override {
        // ---- RENDER-RATE RIG RE-APPLY.
        //
        // The tick publishes the desired WORLD rotation; here we recompute the RELATIVE rotation
        // against the parent as it is right now and write that. The tick's own write is left alone
        // -- this simply supersedes it with a fresher one before the frame is drawn.
        //
        // Parent yaw can move well over 10 deg between ~32 Hz ticks during a turn, so a relative
        // rotation written once per tick is wrong by that much for most of the interval. This is
        // the same callback UEVR's own UObjectHook applies attachments from.
        //
        // Skipped entirely when attach_mode owns the transform, because two writers fighting over
        // one component produces a result that depends on frame timing. Deliberately NOT gated on
        // the eye index: writing on both eyes is harmless (the value is identical) and costs one
        // extra UFunction call per frame.
        {
            static bool logged = false;
            if (!logged) { logged = true;
                API::get()->log_info("[Halo-CampE-UEVR] stereo view index observed = %d", index); }
        }
        if (g_cfg.rig_render && g_cfg.attach_mode == 0
            && g_rigw_valid.load() && !g_in_menu.load() && !g_stick_mode.load()) {
            auto* rig = reinterpret_cast<API::UObject*>(g_rig_component.load());
            auto* par = g_rig_parent;
            if (rig != nullptr && par != nullptr) {
                Vec3 prot{};
                if (call_ret_vec3(par, L"K2_GetComponentRotation", &prot)) {
                    const Quat q_par = rotator_to_quat(prot.x, prot.y, prot.z);
                    const Quat q_w{g_rigw_x.load(), g_rigw_y.load(),
                                   g_rigw_z.load(), g_rigw_w.load()};
                    const Quat q_rel = quat_mul(quat_conj(q_par), q_w);
                    float rp = 0.0f, ry = 0.0f, rr = 0.0f;
                    quat_to_rotator(q_rel.x, q_rel.y, q_rel.z, q_rel.w, &rp, &ry, &rr);
                    if (std::isfinite(rp) && std::isfinite(ry) && std::isfinite(rr)) {
                        rig_set_rotation(rig, (double)rp, (double)ry, (double)rr);

                        // NOT a residual -- this is the error being CORRECTED. It measures how far
                        // the parent moved since the tick that produced this target, i.e. exactly
                        // how wrong the arms WOULD be if the tick's write were the last word. A
                        // large number here means the fix is doing a lot of work, not that error
                        // remains: the relative rotation is recomputed against the live parent, so
                        // the world orientation is correct by construction at the moment of writing.
                        static float worst = 0.0f;
                        static uint32_t n = 0;
                        const float d = std::fabs(wrap180(prot.y - g_rigw_parent_yaw.load()));
                        if (d > worst) worst = d;
                        if (((++n) % 600) == 0) {
                            API::get()->log_info("[Halo-CampE-UEVR] RIGRENDER: corrected up to %.2f deg of parent motion "
                                                 "since tick, over last window", worst);
                            worst = 0.0f;
                        }
                    }
                }
            }
        }

        if (rotation == nullptr || !g_cfg.view_lock) return;

        // STICK MODE: the game camera must reach the eyes unmodified -- the chase camera turning
        // the rendered view IS the gamepad experience the mode exists to restore. Holding the lock
        // UNPRIMED the whole time is also the exit re-anchor: the first frame after stick mode
        // ends re-primes against the then-current camera yaw (turn-compensated -- see the prime
        // below), so leaving a vehicle never restores a stale heading from before it.
        if (g_stick_mode.load()) { g_lock_primed = false; return; }

        // JUDDER FIX -- ASSIGN the locked yaw, never subtract a tick-latched delta.
        //
        // A delta computed in on_pre_engine_tick (~32 Hz) but consumed here at render rate
        // (90+ Hz) is stale between ticks: Blam has already moved the camera, so the
        // uncompensated rotation shows for a frame or two and then snaps back when the tick
        // catches up -- the view follows the controller for a split second, then re-centres.
        //
        // Assigning is exact and has ZERO latency: whatever value Blam put in this frame, we
        // replace it with the locked one, computed from data available in this same call.
        // HMD head-look is unaffected -- the rotation handed to us is the game's camera only
        // (viewIn tracks Blam's yaw one-for-one), and UEVR composes HMD tracking after.
        const float locked = g_locked_view_yaw.load() + g_turn_offset.load();

        if (is_double) {
            auto* r = reinterpret_cast<UEVR_Rotatord*>(rotation);
            g_dbg_view_in = (float)r->yaw;
            if (!g_lock_primed.load()) {
                // PRIME WITH COMPENSATION: capture base = current - turn, so what the lock
                // renders (base + turn) equals the current view exactly. A plain capture is only
                // correct while turn == 0 (the original first-frame case); after a stick-mode
                // exit the PRESERVED turn offset would otherwise be counted twice.
                g_locked_view_yaw = (float)r->yaw - g_turn_offset.load();
                g_lock_primed = true;
            } else {
                r->yaw = (double)locked;
            }
            g_dbg_view_out = (float)r->yaw;
        } else {
            g_dbg_view_in = rotation->yaw;
            if (!g_lock_primed.load()) {
                // Same compensated prime as the double branch above.
                g_locked_view_yaw = rotation->yaw - g_turn_offset.load();
                g_lock_primed = true;
            } else {
                rotation->yaw = locked;
            }
            g_dbg_view_out = rotation->yaw;
        }

        // Frame-to-frame movement of the RENDERED yaw, minus any turning we asked for. This is the
        // judder the player actually feels. With a direct assignment it should be 0.00.
        const float out_now = g_dbg_view_out.load();
        if (g_have_prev_out.load()) {
            const float moved = std::fabs(wrap180(out_now - g_prev_view_out.load()));
            if (moved > g_judder_max.load()) g_judder_max = moved;
        }
        g_prev_view_out = out_now;
        g_have_prev_out = true;
    }

    // Minimal by design: reads two pre-computed scalars and writes the pad struct. No reflection,
    // no allocation, no logging -- this fires on the order of 200,000 times per second.
    // Read-only. Runs after UEVR has composed HMD tracking, so `rotation` here is the finished
    // view in GAME space -- what the player is actually looking along. Published for the movement
    // frame; nothing is written back.
    void on_post_calculate_stereo_view_offset(UEVR_StereoRenderingDeviceHandle, int, float,
                                              UEVR_Vector3f*, UEVR_Rotatorf* rotation,
                                              bool is_double) override {
        if (rotation == nullptr) return;
        if (is_double) {
            auto* r = reinterpret_cast<UEVR_Rotatord*>(rotation);
            g_render_view_yaw   = (float)r->yaw;
            g_render_view_pitch = (float)r->pitch;
        } else {
            g_render_view_yaw   = rotation->yaw;
            g_render_view_pitch = rotation->pitch;
        }
        g_have_render_yaw = true;
    }

    void on_xinput_get_state(uint32_t* retval, uint32_t user_index, XINPUT_STATE* state) override {
        g_xhits.fetch_add(1);
        if (state == nullptr || user_index != 0) return;

        // Presence: report a connected pad ourselves so the mod does not depend on an Oculus /
        // Virtual Desktop / ViGEm bus being installed. This is what Halo-MCC-VR means by "the mod
        // owns virtual slot 0 when no physical pad is present".
        if (g_cfg.fake_pad && retval != nullptr && *retval != ERROR_SUCCESS) {
            ZeroMemory(state, sizeof(XINPUT_STATE));
            *retval = ERROR_SUCCESS;
        }

        // Sample the player's own stick BEFORE we overwrite it -- this is the turn input.
        g_raw_stick_x = (float)state->Gamepad.sThumbRX / 32767.0f;

        // ---- BUTTON MASK LOGGER. Reports the RAW mask before any remapping, on change only.
        // This is how the controller->XInput mapping gets established instead of guessed: press
        // one button, read one line. A guessed mapping can silently unbind a combat action.
        if (g_cfg.map_btn_log) {
            static WORD prev_btn = 0;
            const WORD now_btn = state->Gamepad.wButtons;
            if (now_btn != prev_btn) {
                prev_btn = now_btn;
                API::get()->log_info("[Halo-CampE-UEVR] BTN mask=0x%04X%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
                    (unsigned)now_btn,
                    (now_btn & 0x1000) ? " A" : "",        (now_btn & 0x2000) ? " B" : "",
                    (now_btn & 0x4000) ? " X" : "",        (now_btn & 0x8000) ? " Y" : "",
                    (now_btn & 0x0100) ? " LB" : "",       (now_btn & 0x0200) ? " RB" : "",
                    (now_btn & 0x0040) ? " LTHUMB" : "",   (now_btn & 0x0080) ? " RTHUMB" : "",
                    (now_btn & 0x0010) ? " START" : "",    (now_btn & 0x0020) ? " BACK" : "",
                    (now_btn & 0x0001) ? " DUP" : "",      (now_btn & 0x0002) ? " DDOWN" : "",
                    (now_btn & 0x0004) ? " DLEFT" : "",    (now_btn & 0x0008) ? " DRIGHT" : "");
            }
        }

        // ---- CONTROL REMAPPING. Must run BEFORE the aim output overwrites the right stick, and
        // before any early-out below, or the remaps would stop working whenever the driver idles.
        {
            const float ry = (float)state->Gamepad.sThumbRY / 32767.0f;

            // D-PAD SHIFT: right stick up turns the LEFT stick into a d-pad.
            // Not in menus: this ZEROES the left stick to turn it into a d-pad, which in a menu
            // would silently kill the primary navigation axis.
            // Not in stick mode either: the right stick is the game's own look/orbit input there,
            // so "stick up" is LOOK UP -- shifting on it would kill throttle/steering mid-look.
            if (g_cfg.map_dpad_shift && ry > g_cfg.map_rstick_dz
                && !g_stick_mode.load()
                && !(g_in_menu.load() && g_cfg.menu_suppress)) {
                const float lx = (float)state->Gamepad.sThumbLX / 32767.0f;
                const float ly = (float)state->Gamepad.sThumbLY / 32767.0f;
                const float dz = g_cfg.map_dpad_dz;

                // Dominant axis only. Emitting both on a diagonal produces two simultaneous d-pad
                // presses, which menus read as a double input.
                if (std::fabs(ly) >= std::fabs(lx)) {
                    if (ly >  dz) state->Gamepad.wButtons |= XINPUT_GAMEPAD_DPAD_UP;
                    if (ly < -dz) state->Gamepad.wButtons |= XINPUT_GAMEPAD_DPAD_DOWN;
                } else {
                    if (lx >  dz) state->Gamepad.wButtons |= XINPUT_GAMEPAD_DPAD_RIGHT;
                    if (lx < -dz) state->Gamepad.wButtons |= XINPUT_GAMEPAD_DPAD_LEFT;
                }

                // Suppress movement while shifted, exactly as UEVR's thumbrest mode does --
                // otherwise you walk in the direction you are trying to select.
                state->Gamepad.sThumbLX = 0;
                state->Gamepad.sThumbLY = 0;
                g_dpad_shift_active = true;
            } else {
                g_dpad_shift_active = false;
            }

            // ORDER MATTERS, and the two steps below collide if it is wrong.
            //
            // The rebind must consume only the PHYSICAL press, so it runs first. The intended
            // configuration moves crouch off left-controller X and onto right-stick-down while
            // giving left-X to equipment -- which means the same mask (0x2000, measured) is both
            // the rebind's SOURCE and the stick's INJECTED value. Injecting before rebinding would
            // feed our own synthetic crouch straight into the rebind and turn it into equipment,
            // so stick-down would fire equipment and crouch would exist nowhere.
            const bool in_menu = g_in_menu.load();

            if (in_menu && g_cfg.menu_suppress) {
                // MENU BINDINGS. XInput B is both crouch in gameplay and "back" in every menu, so
                // moving crouch to the right stick left menu-back reachable only by pushing the
                // stick down. In a menu reload is meaningless, so right-controller B becomes back.
                //
                // The gameplay remaps are skipped entirely here rather than layered on top: with
                // them active, navigating with the right stick would fire "back", and left-X would
                // be spent on equipment instead of working as the second back button it natively is.
                if (g_cfg.map_menu_back != 0
                    && (state->Gamepad.wButtons & (WORD)g_cfg.map_menu_back) != 0) {
                    state->Gamepad.wButtons &= (WORD)~g_cfg.map_menu_back;
                    state->Gamepad.wButtons |= XINPUT_GAMEPAD_B;
                }
            } else {
                if (g_cfg.map_from != 0 && (state->Gamepad.wButtons & (WORD)g_cfg.map_from) != 0) {
                    state->Gamepad.wButtons &= (WORD)~g_cfg.map_from;
                    if (g_cfg.map_to != 0) state->Gamepad.wButtons |= (WORD)g_cfg.map_to;
                }

                // Injected AFTER the rebind, so this mask reaches the game untouched.
                // Not in stick mode: looking down with the right stick must not press crouch.
                if (g_cfg.map_rstick_down != 0 && ry < -g_cfg.map_rstick_dz
                    && !g_stick_mode.load()) {
                    state->Gamepad.wButtons |= (WORD)g_cfg.map_rstick_down;
                }
            }
        }

        // ---- MOVEMENT DIRECTION. Rotate the LEFT stick into the player's facing frame.
        // Done here because this is the last point before the game reads it, and it must NOT be
        // gated on g_driving: movement has to stay corrected even when the aim loop is idle.
        {
            const float rlx = (float)state->Gamepad.sThumbLX / 32767.0f;
            const float rly = (float)state->Gamepad.sThumbLY / 32767.0f;
            g_raw_lx = rlx; g_raw_ly = rly;
            // Default the "out" values to the raw ones so the probe stays valid with move_rot=0.
            g_out_lx = rlx; g_out_ly = rly;
        }

        // Skip the movement rotation when the stick is a MENU input rather than a direction --
        // otherwise menu input depends on which way the player happens to be facing. Two cases:
        //   * d-pad shifted -- the left stick is selecting, not walking;
        //   * a menu is up -- the correction rotates by (view - aim), so which way you happen to be
        //     facing decides whether "up" reads as up. Menus have no world frame, so the raw stick
        //     is always the right answer there.
        // Also skipped in stick mode: the game camera owns the view there, so its own
        // camera-relative movement is already correct -- rotating the stick on top of it is what
        // bends vehicle steering.
        if (g_cfg.move_rot != 0.0f && !g_dpad_shift_active.load() && !g_in_menu.load()
            && !g_stick_mode.load()) {
            const float lx = (float)state->Gamepad.sThumbLX / 32767.0f;
            const float ly = (float)state->Gamepad.sThumbLY / 32767.0f;
            if (lx != 0.0f || ly != 0.0f) {
                // ---- COMPUTE THE ANGLE HERE, not on the engine tick.
                //
                // g_move_rot_deg is produced at ~32 Hz and consumed here at controller poll rate.
                // The VIEW term moves as fast as your head does, so a tick-latched angle is stale by
                // up to ~30 ms and movement lags the facing it is supposed to follow -- drifting
                // while you turn and settling only once you stop. Same class of failure as the
                // view-lock judder, same fix: use values available in the consuming callback.
                //
                // g_render_view_yaw is refreshed in the post-stereo callback at RENDER rate; aim
                // moves far more slowly, so taking it at tick rate is fine.
                float delta = g_move_rot_deg.load();
                if (g_cfg.move_live && g_have_render_yaw.load()) {
                    // SMOOTHED aim, live head.
                    //
                    // aim is produced by a RATE actuator that is always converging on the controller,
                    // so waving the controller makes it slew and overshoot -- and since delta
                    // subtracts aim, that transient lands directly in the movement direction as
                    // jitter. Movement has no reason to care how the aim loop is settling.
                    //
                    // Only the AIM term is damped. The head term stays instantaneous, because that
                    // is the one the player is actively steering with and any lag there is felt
                    // immediately (that is what move_live addresses).
                    delta = wrap180(g_render_view_yaw.load() - g_move_aim_smooth.load());
                }
                const float th = delta * g_cfg.move_rot * DEG2RAD;
                const float c = std::cos(th), s = std::sin(th);
                // Standard 2-D rotation; stick +Y is forward, +X is right.
                const float nx = lx * c - ly * s;
                const float ny = lx * s + ly * c;
                state->Gamepad.sThumbLX = to_raw(clampf(nx, -1.0f, 1.0f));
                state->Gamepad.sThumbLY = to_raw(clampf(ny, -1.0f, 1.0f));
                g_out_lx = clampf(nx, -1.0f, 1.0f);
                g_out_ly = clampf(ny, -1.0f, 1.0f);
                // XInput signals "state changed" through the packet number, so a consumer that
                // trusts it will discard an edit that leaves it untouched -- every rewrite of the
                // state must bump it.
                state->dwPacketNumber++;
                g_move_applied.fetch_add(1);
            }
        }

        // Publish pad-button state only; the game thread owns the edge detection.
        g_pad_calib_down = (g_cfg.calib_btn != 0) &&
                           ((state->Gamepad.wButtons & (WORD)g_cfg.calib_btn) != 0);

        // Aim is NOT silenced during MESH calibration -- the weapon is pinned in world space there,
        // so the aim loop can run untouched and the two stay independent.
        //
        // It IS silenced during AIM calibration, which is the point of that gesture: the reticle
        // must hold still while you point the controller at it, or you would be chasing it.
        if (g_aimcal_held.load()) return;

        if (g_cfg.aim_rate_render) {
            // RENDER-RATE LAW. Recomputed from a FRESH controller pose and a fresh ControlRotation
            // read at every XInput poll, instead of applying a deflection the ~32 Hz tick computed
            // up to a frame-third ago. That stale-step staircase produces fast-motion jitter (and
            // side-to-side arm sway, from rig writes chasing aim steps that land late). The tick
            // still owns arming, references, and calibration.
            if (!g_aim_law_armed.load()) return;

            static AimLawState hook_law;
            static std::chrono::steady_clock::time_point last_call{};
            const auto now = std::chrono::steady_clock::now();
            float dt = 0.0f;
            if (last_call.time_since_epoch().count() != 0) {
                dt = std::chrono::duration<float>(now - last_call).count();
            }
            last_call = now;

            float cy = 0.0f, cp = 0.0f;
            double ay = 0.0, ap = 0.0;
            if (!derive_ctrl_angles(&cy, &cp)) return;
            if (!read_control_rotation_hook(&ap, &ay)) return;

            float law_rx = 0.0f, law_ry = 0.0f;
            aim_control_law(hook_law, cy, cp, ay, ap, dt, &law_rx, &law_ry);

            // Published so the probes and the gain adaptation keep reading the live output.
            g_out_rx = law_rx; g_out_ry = law_ry;

            if (law_rx == 0.0f && law_ry == 0.0f) return;
            state->Gamepad.sThumbRX = to_raw(law_rx);
            state->Gamepad.sThumbRY = to_raw(law_ry);
            state->dwPacketNumber++;
            return;
        }

        if (!g_driving.load()) return;

        const float rx = g_out_rx.load();
        const float ry = g_out_ry.load();
        if (rx == 0.0f && ry == 0.0f) return;

        state->Gamepad.sThumbRX = to_raw(rx);
        state->Gamepad.sThumbRY = to_raw(ry);
        state->dwPacketNumber++;
    }
};

static auto g_plugin = std::make_unique<HaloAimDriverPlugin>();
