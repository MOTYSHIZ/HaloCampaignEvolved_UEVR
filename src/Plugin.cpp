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
#include <unordered_map>
#include <cstring>
#include <cstdlib>

#include <chrono>
#include "uevr/Plugin.hpp"

// Pure maths (quaternions, rotators, the calibration solve). Free of plugin state by design --
// see the note at the top of Math.hpp before adding to it.
#include "Math.hpp"

// Live config + calibration persistence. Defines g_cfg, which nearly everything below reads.
#include "Config.hpp"
#include "DevTools.hpp"
// The address-verification harness (signature scan that refuses ambiguity, PE identity). Used
// here by the tick-fault reporter, so a fault can name its function and its build.
#include "addrcascade/AddressCascade.hpp"

// UE object/name helpers: TrackedObject (recycle-safe handles), FName resolution, class names.
#include "UeObject.hpp"

// First-person weapon rig: the gun/arms follow the hand via RELATIVE component transforms.
#include "Rig.hpp"

// Aim reticule: our own mesh reticule, plus one hosting the game's own reticle widget.
#include "Reticule.hpp"
// A third, EXPERIMENTAL reticule drawn by the OpenXR compositor instead of the scene, so tonemapping
// and exposure cannot dim it. Default off, draws alongside the two above, never instead of them.
#include "XrLayer.hpp"
#include "XrSource.hpp"
#include "CutsceneHint.hpp"

// The weapon scope: LT-toggled magnified pane on the aim ray (native zoom stays suppressed).
#include "Scope.hpp"
// For scopelayer_configure_cell_early() only -- the pane's atlas cell must be requested from
// update() BEFORE xrlayer_tick() builds the atlas. Everything else in the scope lane is reached
// through Scope.hpp.
#include "ScopeLayer.hpp"
#include "ScopeOffset.hpp"

// The aim control loop: Halo's own aim is steered to follow the controller via synthesized stick.
#include "MotionAimControl.hpp"
#include "AimTrace.hpp"
#include "MemScan.hpp"

// Motion gestures: swing to melee. Detection on the tick, injection in the XInput hook.
#include "Gesture.hpp"
#include "Holster.hpp"
#include "Markers.hpp"

// Two independent arms: recon for now (skeleton dump + bone-function probe).
#include "ArmDriver.hpp"
#include "TwoHandAim.hpp"
#include "InteractLine.hpp"
#include "Arms.hpp"

// Per-weapon grip/offset deltas on top of the calibration.
#include "WeaponOffset.hpp"
#include "WeaponDrive.hpp"

// Per-weapon calibration capture on its own key.
#include "WeaponCalib.hpp"

// Our own controller-attached hands, and the reload magazine.
#include "Hands.hpp"
// The alternative arm driver. ArmDriver.hpp decides which of the two runs; only one ever does.
#include "palettearm/PaletteArm.hpp"
#include "BlamAim.hpp"
#include "BlamDrive.hpp"
#include "HitTrace.hpp"
#include "AimWatch.hpp"
#include "AimDirect.hpp"
#include "AimConverge.hpp"
#include "GameSettings.hpp"

// View-consumer fixes: the audio listener drive and the dev exec harness. The navpoint half
// lives in this file (it shares the reticle widget scan below).
#include "ViewFix.hpp"

// Shipped version, logged at startup so a bug report identifies the build it came from. There is no
// other build marker in the DLL, so this is the only thing tying a log.txt to a release.
// BUMP THIS WITH THE RELEASE TAG -- CI publishes on `v*`, and the two are not linked automatically.
#define HALO_VR_VERSION "0.4.1"

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

// Mirror the aim-vs-body yaw difference into the externally-linked g_view_lock_delta so palettearm
// can anchor the shoulders to the BODY rather than to the aim-driven camera. Defined here because
// the two raw yaws have internal linkage; see MotionAimControl.hpp for why the delta is the right
// thing to export rather than the pair.
void publish_view_lock_delta() {
    float d = g_dbg_view_out.load() - g_dbg_view_in.load();
    while (d > 180.0f)  d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    ::halo::g_view_lock_delta.store(d);
}
std::atomic<bool>  g_lock_primed{false};
// False until the very first prime. Distinguishes "adopt the camera as the base" (session start)
// from a RE-prime after stick mode, which must fold the difference into the turn offset instead
// -- see the prime site for why those are different operations.
std::atomic<bool>  g_lock_ever{false};

// REMOVED: a re-prime suppressor that tried to stop the stick-mode exit fold from running across a
// level load, first keyed on the world object and then on the pawn. Recorded because the reasoning
// looked sound both times and was wrong both times, measurably:
//
//   * WORLD: Restart Mission reloads the same level, so the outermost object never changed. One log
//     line in an entire session while the bug reproduced every time.
//   * PAWN: fourteen stick-mode transitions, including vehicles, with zero pawn changes -- the
//     stick-mode detector's own pawnmatch=1 says the same thing. Riding a vehicle does not
//     re-possess on this title.
//
// So it never fired, and the arms-after-restart bug it was chasing turned out to be the PIVOT being
// re-derived from an animated skeleton (see the solve, which now pins it). If a cross-level fold
// ever needs suppressing, find a signal that demonstrably fires FIRST -- both of these were written,
// reasoned about at length, shipped, and inert.

// ---------------------------------------------------------------- THE CALIBRATION FRAME
// The yaw a persisted calibration must be measured against, or 0 when the feature is off.
//
// `g_locked_view_yaw` is the constant relating ROOM yaw to the yaw we render, and it is primed ONCE
// per session from the game camera on the first stereo frame after injection. So it encodes WHERE
// YOU INJECTED: the menu's camera (measured 0.0, reproducible) or, if you attach mid-mission,
// whatever direction the player happens to be facing (arbitrary). Storing the calibration MINUS
// this and applying it PLUS the live value makes the saved numbers independent of that choice --
// which is the whole fix. See the long note on Config::calib_relative for the derivation.
//
// Read through one function so the four call sites cannot drift apart: two WRITE sites (the
// calibration solves) must subtract exactly what the two USE sites add, or the calibration is
// silently biased by twice the term.
// !!! THE FRAME IS A ROTATION, NOT A NUMBER YOU CAN ADD TO A ROTATOR'S YAW FIELD.
// The first version of this added the frame to `gripyaw`. That is only equivalent to rotating the
// frame when pitch and roll are both zero -- and the grip trim carries a ~-65 degree PITCH, so it
// mixed the axes instead: rolling the controller pitched the arms and pitching rolled them.
// It must be applied as a world-yaw rotation on the LEFT of the controller orientation, which is
// exactly where q_turn already sits, so it is folded in there instead (both are pure yaws, so
// adding those two scalars IS valid).
//
// TWO accessors, because use and write must disagree exactly once -- while a v1 file is being
// upgraded:
//   USE   applies the frame only for a file that is actually stored relative (calibver >= 2).
//         A v1 file already has a frame baked in; adding another would double-count it.
//   WRITE applies it whenever the feature is on, so the value the solve stores is frame-relative
//         and can be stamped v2.
// The round trip is what makes the upgrade seamless: you calibrate against a gun rendered with
// frame 0 (v1), the solve stores (aligned - locked), and the next use adds locked back to render
// the identical pose -- while a DIFFERENT injection point now gets its own locked. No transient
// breakage, no forced re-calibration ordering.
// Gated PER GESTURE (see Config::aim_calib_ver): the mesh trim and the aim offset are rebased by
// different gestures, so one can be relative while the other is still absolute. Applying the frame
// to whichever has NOT been rebased is precisely the "arms right, reticle wrong" split.
static float calib_frame_yaw_use() {        // gripyaw   -- mesh calibration
    return (g_cfg.calib_relative && g_cfg.calib_ver >= 2) ? g_locked_view_yaw.load() : 0.0f;
}
static float aim_frame_yaw_use() {          // aimoffyaw -- aim calibration
    return (g_cfg.calib_relative && g_cfg.aim_calib_ver >= 2) ? g_locked_view_yaw.load() : 0.0f;
}
static float calib_frame_yaw_write() {
    return g_cfg.calib_relative ? g_locked_view_yaw.load() : 0.0f;
}

// ---------------------------------------------------------------- THE PRIME/CAPTURE RACE
// The rig and the aim consume the frame DIFFERENTLY, and only one of them tolerates being early:
//   * the rig re-reads it every tick, so it self-corrects the moment the lock primes;
//   * the aim BAKES it into g_ref_aim_yaw once, at reference capture, and never revisits it.
// Those two run on different threads -- reference capture on the game thread, the prime in the
// stereo callback -- and nothing ordered them. Lose the race and the aim mapping is anchored with
// frame 0 while the stored offset was rebased against a non-zero one, so aim is wrong by exactly
// `locked` while the arms look perfect. Field-observed: `pinned=143.2` alongside
// `reference RESTORED ... (frame yaw 0.0)`.
//
// Dropping the reference makes the next armed tick re-capture against the now-known frame. A
// re-capture is an ordinary event here (level load, respawn, recentre all do it) and the restore
// path is offset-based, so it lands on the same mapping -- just with the right frame this time.
// Called from the render thread, so it touches nothing but atomics.
static void invalidate_ref_for_frame() {
    if (!g_cfg.calib_relative) return;   // frame unused -> nothing to re-anchor, change nothing
    g_have_ref = false;
}

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

// The rendered view's world POSITION, captured in the stereo callback. The vehicle reticule needs
// a ray origin, and its on-foot source (the rig's camera parent, reached through the first-person
// weapon) does not exist while seated.
std::atomic<float> g_view_pos_x{0.0f}, g_view_pos_y{0.0f}, g_view_pos_z{0.0f};

// The RENDERED EYE position (post-stereo, HMD transform applied) -- distinct from g_view_pos_*
// above, which is the GAME CAMERA the pre-hook hands us. World-space markers must use this one:
// see the note at the publish site in on_post_calculate_stereo_view_offset.
std::atomic<float> g_eye_pos_x{0.0f}, g_eye_pos_y{0.0f}, g_eye_pos_z{0.0f};
std::atomic<bool>  g_have_eye_pos{false};
std::atomic<bool>  g_have_view_pos{false};
// Tick faults attributed to the reticule's trace lane. Set from the SEH filter (which runs
// before unwinding, while g_tick_lane still names the lane), read by the trace call site to
// switch the lane off rather than fault into it forever. See report_tick_fault.
std::atomic<uint32_t> g_trace_faults{0};
// Tick at which each lane may be attempted again after it faulted. Index = PerfSite. Written
// by the SEH filter (before unwinding, while g_tick_lane still names the lane), read by the
// lanes themselves. A COOLDOWN, not a kill: the fault that produced this is transient -- it
// lasts about five seconds around a level transition -- so parking a lane for the session
// trades a five-second outage for a permanently degraded feature, which is the wrong trade.
// The current tick, published so the SEH filter can timestamp a cooldown. The filter cannot
// take a parameter and must not call anything that could itself fault.
std::atomic<uint32_t> g_tick_now{0};

// ---- THE REFLECTION SETTLE GATE ---------------------------------------------------------
//
// SYMBOLIZED 2026-09-06, which is what makes this a fix rather than a guess:
//     0xC0000005 reading 0x10, at UEVRBackend.dll +0x61DF58
//     -> UEVRBackend!sdk::UObjectBase::update_offsets
//
// UEVR re-derives UObject layout offsets lazily, underneath whatever reflection call we happen
// to make. During a mission -> menu -> mission transition the object array is being torn down
// and rebuilt, so that derivation walks an object that is not there and dereferences null.
// TWO UNRELATED LANES faulted at the IDENTICAL instruction (reticule_trace and socket_sample),
// which is what proved it was one shared call rather than anything about the line trace.
//
// WHY THE GATE IS AT THE TICK AND NOT AT THE CALL SITES: there are ~545 raw reflection calls
// across 17 files (215 call_function alone) and no wrapper layer. A per-site check would be
// 500 edits and would be forgotten by the next feature -- the same reason the dev-tooling split
// is compile-time rather than a config flag. Gating update() is ONE place, and it covers every
// lane that exists plus every lane anyone adds later, because they all live inside it.
//
// It is a TIMING guard, not a correctness proof. If the array is still rebuilding when the
// window expires a fault is still possible -- that is what the per-lane cooldown above is for.
std::atomic<uint32_t> g_reflect_ok_at{0};

// PLAYER-ATTACHED LAYER QUADS: hand the slot an offset from the eye instead of a world point, and
// set the slot's flag to match. See xrlayer_set_quad_head_relative / g_tgt_headrel.
//
// WHY IT IS ONE FUNCTION AND NOT TWO CALLS: the flag says how to interpret the vector, so writing
// them apart lets them disagree. A stale flag reinterprets a world position as an offset, which
// throws the quad a whole world-origin away -- it reads as "the layer vanished", not as a bad
// number, and nothing in the log would say why.
//
// Falls back to the world point when the eye is not known yet (no view composed this session), so
// the worst case is the old behaviour rather than a quad at the origin.
static Vec3 layer_anchor(int slot, const Vec3& world) {
    // g_view_pos (MONO pre-hook camera), never g_eye_pos (PER-EYE). g_eye_pos alternates between
    // the left and right eye every frame, so capturing against it and re-anchoring per eye threw
    // one eye a full IPD sideways and flattened the stereo depth. See xrlayer_note_eye.
    if (g_cfg.xr_layer_head_rel != 0 && g_have_view_pos.load(std::memory_order_relaxed)) {
        halo::xrlayer_set_quad_head_relative(slot, true);
        return Vec3{world.x - g_view_pos_x.load(std::memory_order_relaxed),
                    world.y - g_view_pos_y.load(std::memory_order_relaxed),
                    world.z - g_view_pos_z.load(std::memory_order_relaxed)};
    }
    halo::xrlayer_set_quad_head_relative(slot, false);
    return world;
}

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

// Ticks below this value resolve the rig on a FAST cadence (every 8 ticks instead of 60). Set by
// the route-death burst and the dismount watcher so stick-mode transitions are event-driven
// instead of waiting out the 2 s resolve poll -- the cost is a handful of extra sweeps per
// boarding/swap/dismount EVENT, never a standing rate increase. Game thread only.
uint32_t g_rig_fast_until = 0;

// This tick's "the player has first-person foot control" verdict, published by the detector.
// Snap/smooth turn gates on it so turning stands down the INSTANT that control is lost at
// boarding, without waiting out the stick debounce -- entering a vehicle used to bank several
// accidental snap turns in that window. A weapon swap loses turning for ~0.3 s, imperceptible.
//
// NOT simply the weapon route. Standing unarmed kills the route while leaving the player in full
// first-person control, and gating turning on the route alone was half of why the campaign's
// opening felt broken. The detector below is where the two are told apart. Game thread only.
bool g_fp_control_now = true;

// This tick's "on foot, in first person, holding nothing" verdict -- the stick-mode exception's own
// state, published because the rig block hides the arms on exactly the same condition. Deliberately
// the SAME signal rather than a second "is there a weapon" test: two tests would drift, and the one
// state where they disagreed would be a pair of T-posed arms in someone's face.
bool g_on_foot_unarmed = false;

// The game's own cutscene state (BlamCinematicSubsystem::IsCinematicInProgress), owned by the
// cutscene block and READ by the stick-mode detector earlier in the same tick. A cutscene can leave
// the player standing weaponless in first person, which is indistinguishable from the campaign's
// unarmed opening by any signal the detector has -- so the on-foot exception consults the
// authoritative answer instead of trying to infer one. Game thread only.
bool g_cine_active    = false;
bool g_cine_answering = false;

// True while the grip brake is held AND the delivery is pad-side (brake_mode 2/3). Written by the
// game thread, consumed in the XInput hook.
std::atomic<bool> g_brake_pad{false};

// True while the CUTSCENE 2D SCREEN owns UEVR's VR_2DScreenMode (we set it, we restore it).
// Distinct from the raw camera signal: engage latches on a cinematic-camera sighting during
// stick mode and holds until the weapon returns, because the camera signal only marks a
// cutscene's start while the weapon route spans its whole length. Game thread only; atomic for
// consistency with its neighbours.
std::atomic<bool> g_cut2d_engaged{false};

// Pose-match calibration button state. The geometry lives further down, with the quaternion
// helpers it depends on.
std::atomic<bool> g_calib_held{false};

// Menu-armed calibration: trigger states sampled (and eaten) in on_xinput_get_state while
// g_menu_calib_mode is armed. RT = save & exit, LT = save & re-arm on release.
std::atomic<bool> g_menu_calib_lt{false};
std::atomic<bool> g_menu_calib_rt{false};
std::atomic<bool> g_calib_start{false};
std::atomic<bool> g_calib_finish{false};
// Raw pad-button state, published by the XInput hook and consumed on the game thread. The hook
// deliberately does NOT edge-detect: the keyboard source is invisible from there, and detecting
// edges on two threads against one state would race.

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
// Controller angles sampled AT the Page Down release edge, so the aim reference binds to where the
// hand was when the key came up rather than where it has drifted to a tick later. See the release
// handler for why that tick matters.
std::atomic<bool>  g_aimcal_have_snap{false};
std::atomic<float> g_aimcal_snap_yaw{0.0f}, g_aimcal_snap_pitch{0.0f};

// (No equivalent for the End pose-match bind, deliberately. One existed; it was removed after the
// instrument showed the release edge and the solve are the same update() call, 0.00 cm apart. The
// Page Down capture above is different because ITS consumer is genuinely deferred to a later tick.)
std::atomic<bool> g_kill_held{false};   // Ctrl+kill_key edge state (see the kill switch below)
std::atomic<bool> g_mode_held{false};   // Ctrl+mode_key edge state (aim-mode toggle)

// HOTKEY OVERRIDES. -1 = no override, 0/1 = the value the hotkey forced.
//
// update() calls load_config() unconditionally every ~2 s, so a hotkey that only flips g_cfg in
// memory is undone by the next poll. That is not hypothetical: it is what happens to the kill
// switch today -- Ctrl+HOME disables the driver and `enabled=1` in the file turns it straight back
// on, which is the one feature that must never quietly fail. Each override is re-applied after
// every reload, and released the moment the FILE's own value changes, so hand-editing the config
// still wins exactly as the kill switch's comment promises.
std::atomic<int> g_kill_override{-1};
std::atomic<int> g_mode_override{-1};
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
//
// PERF_RETICLE COUNTS SLICES, NOT SWEEPS (changed 2026-08-23). The widget sweep is spread across
// consecutive ticks at `retsweepms` per tick, so `n` is now the number of slices in the window and
// `max` is the worst single slice -- which is the number that matters for a dropped frame. It is
// NOT comparable with the pre-2026-08-23 figures (83.9 / 29.8 / 55.3 ms), which were whole sweeps.
// For the whole-sweep cost, read the `widget sweep: N objects in M slice(s), X ms total` line
// instead, and read it together with the `objects=` field on the PERF header: the object array is
// a high-water mark that grows all session, so a sweep timing without its array size is meaningless.
//
// PERF_TRACE is the reticule's line trace: a reflected UKismetSystemLibrary::LineTraceSingle,
// EVERY TICK, which is the only per-tick engine call this plugin makes. It was added without being
// measured, on the reasoning that one trace is cheap -- exactly the reasoning that produced the
// periodic microstutter these timers exist to catch. Unlike the other four it is not gated behind a
// tick counter, so `n` here should track the tick count and `mean` is the per-frame cost.
// PERF_BRIDGE is the settings-menu file bridge (menu_bridge_tick), nested inside PERF_CFG's
// window so the two can be told apart: the bridge was shipped in v0.3.0 doing 9 synchronous
// opens/~130 KB of re-reads per poll on this thread -- the same I/O class whose measured
// 143.9/108.7 ms contention stalls got load_config its stat gate (Config.cpp).
// PERF_NAVHOST times only the EXPENSIVE path of navw_host_class (widget Create + image-child
// collection): the re-host storm ranked #1 in-plugin freeze candidate for the v0.3.1 field
// reports (docs\Perf\V031-EPISODIC-WORKLOADS-2026-08-19.md), and it had never been frame-timed.
//
// The six after PERF_NAVHOST come from PR #7, timed separately so a performance question can be
// answered with a measurement instead of an argument about which change did it. socket_sample is
// the one to watch: four reflected calls EVERY tick (component and actor location, component and
// actor rotation) to feed rigsocket/rigsockrot. Reflected calls are the expensive operation in a
// UEVR plugin -- reticule_trace is in this list for exactly that reason -- and unlike the sweeps
// it is not behind a tick counter. PERF_TICK brackets the whole tick, so it is the denominator
// the others should be read against.
enum PerfSite { PERF_CFG = 0, PERF_RETICLE, PERF_RIG, PERF_SHELL, PERF_TRACE, PERF_BRIDGE,
                PERF_NAVWORLD, PERF_NAVHOST, PERF_NAVSLOT,
                PERF_GEST, PERF_ARMS, PERF_HANDS, PERF_WPNOFF, PERF_SOCK,
                PERF_XRLAYER, PERF_XRSRC,
                PERF_PALARM, PERF_2HAND, PERF_HOLSTER, PERF_TICK, PERF_COUNT };
const char* const kPerfName[PERF_COUNT] = { "load_config   ", "reticle_rescan", "resolve_rig   ",
                                            "resolve_shell ", "reticule_trace", "menu_bridge   ",
                                            "navworld_tick ", "navw_rehost   ", "navw_newslot  ",
                                            "gesture_update",
                                            "arms_update   ", "hands_update  ", "weapon_offset ",
                                            "socket_sample ", "xrlayer_tick  ", "xrsource_tick ",
                                            "palettearm    ", "two_hand      ", "holster_update",
                                            "TICK (all)    " };

// SITES WHOSE TIME IS ALREADY INSIDE ANOTHER SITE. Counted normally in the window report (where
// each line stands alone) but excluded from the attributed total in the hitch line below, which
// would otherwise double-count them and report a negative remainder.
//   * PERF_BRIDGE  is inside PERF_CFG (menu_bridge_tick runs within load_config's window).
//   * PERF_NAVHOST is inside PERF_NAVWORLD (navw_host_class's expensive path runs within
//     nav_world_tick). nav_world_tick used to be UNSCOPED entirely -- which is exactly how a
//     ~570 ms blocking asset load at marker-slot creation hid for a session as "unattributed"
//     (log 2026-08-23 22:51). PERF_NAVWORLD now brackets the whole lane so it can never hide
//     again; navw_rehost stays broken out for the re-host-storm question it was added for.
//   * PERF_NAVSLOT is inside PERF_NAVWORLD too, and PERF_NAVHOST is in turn inside IT. Added
//     2026-08-25 because bracketing the whole lane was not enough: the hitch moved from
//     "unattributed" to "navworld_tick=549.2, navw_rehost=0.5" and stopped there, naming a
//     500 ms region with no owner inside a 2000-line function. navw_newslot is the CREATION of a
//     marker quad (navw_ensure_slot past its already-exists fast path) -- the one thing in this
//     lane that happens once per slot and never again, which is exactly the shape the field
//     reported. The fast path stays OUTSIDE the scope so `n` keeps meaning "slots created".
bool perf_site_is_nested(int s) {
    return s == PERF_BRIDGE || s == PERF_NAVHOST || s == PERF_NAVSLOT;
}

struct PerfStat {
    double   max_ms = 0.0;
    double   sum_ms = 0.0;
    uint32_t n      = 0;
};
PerfStat g_perf[PERF_COUNT];
std::atomic<float> g_dt_worst{0.0f};

// ONE TICK'S worth, cleared at the top of update(). The window report above answers "what does this
// cost on average"; it cannot answer "what was the 600 ms tick DOING", because a max is a single
// number with no breakdown attached. That is the question a stutter report actually asks, and until
// now the only honest answer was "one of these thirteen things, or none of them".
double g_perf_now[PERF_COUNT] = {};

// A tick slower than this gets one line naming where its time went. ms.
//
// Not a config key on purpose: the else-if chain that parses the general keys is already at MSVC's
// nesting ceiling (C1061 -- see parse_scope_key), and perflog=1 is already this project's documented
// stutter switch. 100 ms is four dropped frames at 40 Hz: unmistakably a hitch, and far above the
// ordinary per-tick cost, so a healthy session prints nothing at all.
constexpr double PERF_HITCH_MS = 100.0;

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
// ---- WHICH LANE WAS THE TICK IN WHEN IT DIED? ------------------------------------------------
//
// on_pre_engine_tick has been throwing every tick and never recovering (5,720 in one session,
// 13,879 in another). UEVR catches it and prints "one of the plugins has an error", which names
// neither the plugin nor the site, and the tick body is dead from that point so no lane gets to log
// its own name. Every diagnosis so far has been inferred from WHICH LANE STOPPED LOGGING FIRST,
// which is ordering, not evidence -- and it has now been wrong repeatedly.
//
// So: PerfScope already brackets every lane, and it costs one relaxed store to record which one we
// are inside. The next tick reads it and reports the lane the PREVIOUS tick never came back from.
//
// WHY THE RAII GUARD IS THE RIGHT SHAPE, and it depends on a build flag: we compile with /EHsc, so
// an SEH fault (an access violation, which is what this is) does NOT unwind C++ destructors. A
// normal return -- including every early return in the tick -- runs ~TickDoneGuard and marks the
// tick finished; a FAULT does not. That is exactly the distinction we need, and it means no early
// return can masquerade as a crash.
std::atomic<int>  g_tick_lane{-1};       // INNERMOST PerfScope currently running; -1 = none

// WHERE INSIDE nav_world_tick WE ARE. A lane name is not enough here: nav_world_tick is ~1,600
// lines with 26 separate engine calls, and the fault we are chasing is at ONE fixed instruction in
// the game (always +0x36FD8A6), so naming the lane narrows it to "somewhere in a sixth of the
// file". Two rounds of reasoning about which call it was have now been wrong, so this measures it.
//
// A plain literal pointer, stored relaxed: the strings are static, the store is a few instructions
// a handful of times per tick, and it is read only from the fault filter. Cleared on the way out
// so a fault OUTSIDE the lane cannot inherit the last marker -- the same mistake PerfScope was
// making, which is what sent the last two rounds down the wrong path.
std::atomic<const char*> g_navw_mark{nullptr};
#define NAVW_MARK(s) g_navw_mark.store((s), std::memory_order_relaxed)
// Sized on PerfSite, so they must live BELOW the enum -- they were declared ~230 lines above it
// and did not compile. Same contract as described at g_trace_faults.
std::atomic<uint32_t> g_lane_retry_at[PERF_COUNT]{};
std::atomic<uint32_t> g_lane_faults[PERF_COUNT]{};

// HONOUR THE COOLDOWN. The state above was recorded for EVERY lane but only the reticule trace
// actually checked it -- so after the trace backed off correctly, socket_sample walked straight
// into the same faulting UEVR call 7 more times and went on killing the tick. A net that only
// one lane consults is not a net. This is the one-line check every reflection-heavy lane uses.
inline bool lane_cooling(PerfSite site, uint32_t tick) {
    return tick < g_lane_retry_at[site].load(std::memory_order_relaxed);
}
std::atomic<bool> g_tick_finished{true};

// DID THE LAST TICK ABORT? Set by the __except FILTER, which runs before any unwinding and so
// cannot be undone by it. g_tick_finished cannot answer this: its only writer is ~TickDoneGuard,
// a destructor, and that destructor also runs while the frame unwinds -- so an aborted tick marks
// itself "finished" on the way out and the detector reads a clean run. That is why TICK FAULTED
// reported 0 against 8 observed faults on 2026-09-08 (and 0 against 8,477 before it): not a quiet
// system, a blinded one. An instrument whose signal is erased by the very event it measures will
// always read "nothing happened", which is the most convincing wrong answer there is.
std::atomic<bool> g_tick_aborted{false};

// ---- NAME THE FUNCTION A TICK FAULT LANDED IN, WITHOUT RECORDING ITS ADDRESS ------------------
//
// Every report from the 2026-09-08 release playthrough landed at the same RVA, +0x36FD8A6. Resolved
// OFFLINE from the exe's own .pdata (Scripts\Resolve-ExeRva.py, no session needed) that is
// FName::ToString(FString&)+0x16 -- the `mov ecx,[rcx]` that reads the FName it was handed. UEVR's
// own resolver, which shares no code with that script, logged the same function at the same RVA
// ("FName::get_to_string (inlined alternative): result=...d890" against "Game Module Addr:
// ...240000", log 2026-09-08). Two resolvers agreeing is the standard the Direct Drive lane set.
//
// So the fault was never "a field read at +0x18 off a poisoned base" INSIDE the game. The faulting
// read is of the FName pointer ITSELF, and 0x40400018 is &((UObject*)0x40400000)->NamePrivate:
// some caller asked for the name of an object whose pointer was 0x40400000 -- the float 3.0f, i.e.
// a recycled block, not an object. Which caller is what the return chain below now records.
//
// THAT RVA IS A MEASUREMENT OF ONE BUILD and is deliberately NOT written into this file. The
// function is found by SHAPE at startup, the scan refuses an ambiguous match, and a fault is named
// only when the OS's own unwind metadata says its function begins exactly where the scan landed.
// On a build where the scan fails the report carries the bare RVA -- the previous behaviour, which
// eight real faults have already exercised, so the fallback is not untested code.
//
// The signature is the prologue through the first FNameEntry header decode (`shr r9d,6` is the
// 10-bit length field); the one rel32 (the FNamePool resolve call) is wildcarded. Verified unique
// in the exe on disk at 0x18, 0x30, 0x48 and the full 0x81-byte body (2026-09-08).
// ADDR-HYGIENE: resolved -- scan_signature at on_initialize; there is no fallback constant at all.
constexpr unsigned char FNAME_TOSTRING_SIG[] = {
    0x48,0x89,0x5C,0x24,0x10,  0x48,0x89,0x7C,0x24,0x18,  0x41,0x56,  0x48,0x83,0xEC,0x20,
    0x4C,0x8B,0xF1,  0x48,0x8B,0xDA,  0x8B,0x09,  0xE8,0x00,0x00,0x00,0x00,  0x45,0x8B,0x46,0x04,
    0x48,0x8B,0xF8,  0x44,0x8B,0x53,0x0C,  0x44,0x0F,0xB7,0x08,  0x41,0xC1,0xE9,0x06 };
constexpr char FNAME_TOSTRING_MASK[] =
    "xxxxx" "xxxxx" "xx" "xxxx" "xxx" "xxx" "xx" "x????" "xxxx" "xxx" "xxxx" "xxxx" "xxxx";
static_assert(sizeof(FNAME_TOSTRING_SIG) == sizeof(FNAME_TOSTRING_MASK) - 1,
              "signature and mask must pair up");

std::atomic<uintptr_t> g_fname_tostring{0};   // absolute address in this process, 0 = unresolved

void fault_names_init() {
    void* exe = (void*)GetModuleHandleW(nullptr);
    if (exe == nullptr) return;
    LARGE_INTEGER t0{}, t1{};
    QueryPerformanceCounter(&t0);
    const addrcascade::Signature  sig{FNAME_TOSTRING_SIG, FNAME_TOSTRING_MASK, sizeof(FNAME_TOSTRING_SIG)};
    const addrcascade::ScanResult hit = addrcascade::scan_signature(exe, sig);
    QueryPerformanceCounter(&t1);
    const double ms = (double)(t1.QuadPart - t0.QuadPart) * perf_tick_ms();
    if (hit.unique()) {
        g_fname_tostring.store(hit.address, std::memory_order_relaxed);
        API::get()->log_info(
            "[Halo-CampE-UEVR] FAULTNAMES: FName::ToString resolved by signature at +0x%llX "
            "(unique match, %.1f ms scan). A tick fault landing inside it will say so by name.",
            (unsigned long long)(hit.address - (uintptr_t)exe), ms);
    } else {
        API::get()->log_info(
            "[Halo-CampE-UEVR] FAULTNAMES: FName::ToString NOT resolved -- %s (%zu match(es), "
            "%.1f ms). Tick faults will carry the bare RVA only, exactly as before.",
            hit.matches > 1 ? "ambiguous signature, refusing to guess" : "no match on this build",
            hit.matches, ms);
    }
}

// Basename of a module path. Written without a backslash literal on purpose: this file has been
// mangled twice by tooling that collapses escapes, and 92 is unambiguous. The inline version this
// replaces advanced one character past the separator, which is why every report so far has read
// "in aloCampaignEvolved.exe".
const char* fault_module_leaf(const char* path) {
    const char* leaf = path;
    for (const char* q = path; *q != '\0'; ++q) {
        if (*q == '/' || *q == (char)92) leaf = q + 1;
    }
    return leaf;
}

// THE RETURN-ADDRESS CHAIN ABOVE A FAULT, as module+RVA so each frame resolves against that
// module's PDB afterwards. Walked with the same unwind metadata the OS uses (RtlLookupFunctionEntry
// + RtlVirtualUnwind on a COPY of the faulting context) -- no recorded address anywhere.
//
// This is what finally names the CALLER of a fault that lands in a shared engine utility.
// FName::ToString has 2,521 call sites in the exe, and "which one" is not something a step marker
// in our own lane can answer when the call is made from inside UEVR's SDK on our behalf.
//
// Filter-safe by construction: fixed-size stack buffers, no allocation, no dbghelp, and its own
// __try so a torn stack ends the walk instead of nesting a second exception inside the first.
// Only PODs live in here, which is what lets it contain __try at all.
void fault_return_chain(const CONTEXT* in, char* out, size_t cap) {
    out[0] = '\0';
    if (in == nullptr || cap < 8) return;
    CONTEXT ctx = *in;
    size_t used = 0;
    __try {
        for (int frame = 0; frame < 8; ++frame) {
            DWORD64 img = 0;
            RUNTIME_FUNCTION* rf = RtlLookupFunctionEntry(ctx.Rip, &img, nullptr);
            if (rf == nullptr) {
                // A leaf function: its return address is at the top of the stack.
                if (IsBadReadPtr((const void*)(uintptr_t)ctx.Rsp, sizeof(DWORD64))) break;
                ctx.Rip = *(const DWORD64*)(uintptr_t)ctx.Rsp;
                ctx.Rsp += sizeof(DWORD64);
            } else {
                PVOID   handler = nullptr;
                DWORD64 est     = 0;
                RtlVirtualUnwind(UNW_FLAG_NHANDLER, img, ctx.Rip, rf, &ctx, &handler, &est, nullptr);
            }
            if (ctx.Rip == 0) break;
            HMODULE   hm = nullptr;
            char      name[MAX_PATH] = "?";
            uintptr_t rva = (uintptr_t)ctx.Rip;
            if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS
                                 | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                                   (LPCSTR)(uintptr_t)ctx.Rip, &hm) && hm != nullptr) {
                GetModuleFileNameA(hm, name, MAX_PATH);
                rva = (uintptr_t)ctx.Rip - (uintptr_t)hm;
            }
            const int n = _snprintf_s(out + used, cap - used, _TRUNCATE, "%s%s+0x%llX",
                                      frame ? " < " : "", fault_module_leaf(name),
                                      (unsigned long long)rva);
            if (n < 0) break;
            used += (size_t)n;
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        _snprintf_s(out + used, cap - used, _TRUNCATE, "%s(walk faulted)", used ? " < " : "");
    }
}
std::atomic<bool> g_tick_ever{false};    // suppress the report for the very first tick

struct PerfScope {
    PerfSite      site;
    int           prev_lane;   // the lane this scope displaced; restored on the way out
    LARGE_INTEGER t0{};
    bool          on;

    // RESTORE THE LANE ON EXIT, OR THE FAULT ATTRIBUTION IS A LIE THAT READS LIKE EVIDENCE.
    //
    // This used to only STORE on entry, so g_tick_lane meant "the last scope ever ENTERED", not
    // "the scope we are IN". Once a scope exited its name stayed in the field for everything that
    // ran afterwards -- and only 19 sites in this whole file open a scope, so most of update()
    // reported whichever lane happened to finish last. navworld_tick casts the longest shadow:
    // nothing opens another scope for ~900 lines after it, so a fault anywhere in that stretch
    // was labelled 'navworld_tick'.
    //
    // That is exactly how it misled us on 2026-09-07. A fault labelled navworld_tick survived a
    // correct fix made inside navworld, because the fault was never in navworld. Any older
    // conclusion resting on this field deserves re-reading -- the lanes an earlier session listed
    // as having faulted (reticule_trace, socket_sample, resolve_shell, reticle_rescan) were read
    // the same way and may be the same artifact.
    //
    // Restored, the field means THE INNERMOST SCOPE STILL RUNNING, and a fault outside every scope
    // prints '(none)' -- which is the honest answer and is itself the signal: "not in any
    // instrumented lane" rather than the name of an innocent one.
    explicit PerfScope(PerfSite s) : site(s), on(g_cfg.perf_log) {
        // UNCONDITIONAL, outside the perf_log gate: the breadcrumb has to work in a build where
        // nobody thought to switch perf logging on, which is every build a player is running.
        prev_lane = g_tick_lane.exchange((int)s, std::memory_order_relaxed);
        if (on) QueryPerformanceCounter(&t0);
    }
    ~PerfScope() {
        // UNCONDITIONAL, AND BEFORE THE perf_log EARLY-OUT: the lane field is fault attribution,
        // not timing, so it must stay correct with perflog off -- the shipped state, and the state
        // every one of these fault reports has come from.
        g_tick_lane.store(prev_lane, std::memory_order_relaxed);
        if (!on) return;
        LARGE_INTEGER t1{};
        QueryPerformanceCounter(&t1);
        const double ms = (double)(t1.QuadPart - t0.QuadPart) * perf_tick_ms();
        PerfStat& p = g_perf[site];
        if (ms > p.max_ms) p.max_ms = ms;
        p.sum_ms += ms;
        ++p.n;
        g_perf_now[site] += ms;   // this tick only; cleared at the top of update()
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
    // Compared against TICK (all) this says whether the stall is even inside this plugin:
    // a 60ms frame with a 0.3ms tick is not ours, whatever else is true.
    const float wdt = g_dt_worst.exchange(0.0f);
    // THE OBJECT-ARRAY SIZE IS PART OF THE MEASUREMENT, not trivia. Every full-array sweep in
    // this plugin costs O(this number), and FUObjectArray's count is a HIGH-WATER MARK -- UE
    // never shrinks it, so it climbs for the whole session as levels stream in. Two sweep timings
    // taken at different array sizes are not comparable, and reading them as a regression is
    // exactly the mistake this line exists to prevent: reticle_rescan "went from" 29.8 ms to
    // 55.3 ms between 2026-08-12 and 2026-08-23 with its optimisation completely intact.
    auto* obj_arr = API::get()->get_uobject_array();
    const int32_t obj_n = (obj_arr != nullptr) ? obj_arr->get_object_count() : -1;
    API::get()->log_info("[Halo-CampE-UEVR] PERF window=600 ticks dt=%.1fms (%.1f Hz tick)  "
                         "WORST FRAME=%.1fms (%.1f Hz)  objects=%d",
                         dt * 1000.0f, dt > 0.0f ? 1.0f / dt : 0.0f,
                         wdt * 1000.0f, wdt > 0.0f ? 1.0f / wdt : 0.0f, obj_n);
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

    // The two-handed hold's own state, next to the perf table because that is the block a support
    // report already contains. Unconditional and one snprintf'd string -- the whole point is that
    // "it does nothing" stops being a hypothesis you need a headset to test.
    if (g_cfg.two_hand) {
        API::get()->log_info("[Halo-CampE-UEVR] %s", halo::two_hand_status());
    }
    // Same reasoning: the palette arm driver has six silent exits between "node map resolved"
    // and "arms moving", and none of them was observable from outside until now.
    if (g_cfg.arm_driver == 2) {
        API::get()->log_info("[Halo-CampE-UEVR] %s", halo::palettearm_status());
        API::get()->log_info("[Halo-CampE-UEVR] %s", halo::palettearm_status_geom());
    }
    // Carries the self-check error, which is the number that says whether a player switching this
    // on would have to recalibrate. Printed whenever the mode is on, including after it has tripped
    // -- a mode that quietly fell back is exactly the thing worth seeing.
    if (halo::weapon_drive_enabled()) {
        API::get()->log_info("[Halo-CampE-UEVR] %s", halo::weapon_drive_status());
    }
}

// ONE LINE PER HITCHING TICK, saying where that tick's time went. Called right after the PERF_TICK
// scope closes, so g_perf_now[PERF_TICK] is this tick's finished total.
//
// WHY THE WINDOW REPORT IS NOT ENOUGH. A window says `TICK (all) max=622ms` and, on the same lines,
// `resolve_rig max=0.002ms` and `reticle_rescan (did not run)` -- so the 622 ms was not the two
// sweeps everyone reaches for first, and nothing in the report says what it WAS. That is measured,
// not hypothetical: it is what the 2026-08-23 session's log looks like at 19:40:15 and 19:47:17.
//
// So this prints the same tick's per-site numbers TOGETHER WITH the remainder that no site claims.
// A large `unattributed` is the honest answer "this hitch is inside update() and none of the
// instrumented sites did it", which is a real result and points at where to instrument next. It is
// also the answer that settles "is this hitch even ours": compare it against the frame delta.
void perf_hitch_report() {
    if (!g_cfg.perf_log) return;
    const double total = g_perf_now[PERF_TICK];
    if (total < PERF_HITCH_MS) return;

    char buf[512];
    int  n = 0;
    double attributed = 0.0;
    for (int i = 0; i < PERF_COUNT; ++i) {
        if (i == PERF_TICK) continue;
        if (g_perf_now[i] < 0.05) continue;           // noise; keeps the line readable
        if (!perf_site_is_nested(i)) attributed += g_perf_now[i];
        // kPerfName is space-padded for the column report; trim it for an inline list.
        char name[24] = {0};
        for (int c = 0; c < 20 && kPerfName[i][c] != '\0' && kPerfName[i][c] != ' '; ++c) name[c] = kPerfName[i][c];
        const int wrote = _snprintf_s(buf + n, sizeof(buf) - (size_t)n, _TRUNCATE,
                                      " %s=%.1f", name, g_perf_now[i]);
        if (wrote <= 0) break;
        n += wrote;
    }
    API::get()->log_info("[Halo-CampE-UEVR] HITCH: one tick took %.1f ms |%s | unattributed=%.1f ms "
                         "(inside update(), not in any instrumented site)",
                         total, (n > 0) ? buf : " no instrumented site ran ", total - attributed);
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
// Head displacement from the standing origin at the instant of the freeze, VR room space. The hold
// rides any change from this so the frozen weapon stays put relative to the PLAYER rather than to
// the camera, which is what keeps head movement during the hold out of the fitted mount. Always
// zero while the head is leashed. See the hold-ride block in the rig update.
Vec3 g_calib_delta_room{0.0f, 0.0f, 0.0f};
bool g_calib_have_delta = false;
Vec3 g_calib_sock_local{0.0f, 0.0f, 0.0f};        // socket in the mesh frame, frozen
Quat g_last_gun_world{0.0f, 0.0f, 0.0f, 1.0f};    // updated every driven tick
#if HALO_VR_DEV
// CALIBJUMP: consecutive driven frames across a calibration release. Each quantity is differenced
// against ITSELF on the previous frame -- see the sampling site for why the first attempt (rate
// limited, and differencing two different coordinate spaces) measured nothing.
int  g_calibjump_arm = 0;
bool g_calibjump_have_prev = false;
Vec3 g_calibjump_gun{}, g_calibjump_par{};
float g_calibjump_rot_p = 0.0f, g_calibjump_rot_y = 0.0f;
#endif
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

// Resolved NAVPOINT container widgets (navclass instances -- the screen-space waypoint/objective
// layer), collected by the same sweep. Unlike the reticles these have no cross-TU consumer, so
// plain statics in this anonymous namespace are the whole storage. hud_navpoint_follow() is the
// only reader; see it for why a stale shift must be RELEASED rather than left in place.
// SHUTDOWN LATCH. Set at the first sign of teardown (window message, or a null engine on the
// tick) and never cleared -- once the game is going down there is nothing to resume. Every
// callback that touches game objects or does work checks it FIRST: after teardown the objects
// we would write to are being destroyed, and the hooks we would drive are being removed.
std::atomic<bool> g_shutting_down{false};

TrackedObject g_navpoints[4];
int           g_nav_count = 0;
bool          g_nav_shift_applied = false;   // a non-zero RenderTranslation is currently in place

// RENDER-RATE APPLICATION (navrender). The first navfix iteration wrote the shift from the
// ~32 Hz tick and read as JITTER: the aim-vs-view delta moves at render rate, so a tick-rate
// correction lags it for most of every frame -- the same class of bug as the rig re-apply and
// movelive before it ("compute in the callback that consumes the value", third occurrence).
// The tick now only CALIBRATES (probes the game's own projection for pixels-per-tan) and the
// stereo callback applies, using the exact same-frame delta it already owns.
std::atomic<bool>  g_nav_apply{false};    // tick-evaluated gates; render applies only while true
std::atomic<float> g_nav_keff{1000.0f};   // measured px-per-tan, EMA of the tick's probe





// Read pixels straight out of a render target and log them. This answers "does the widget's render
// target contain colour?" NUMERICALLY, with no display material in the loop -- every on-screen
// attempt so far has been hostage to whether the chosen material family renders at all.
// ReadRenderTargetPixel(WorldContextObject, TextureRenderTarget2D*, int32 X, int32 Y) -> FColor.
void probe_render_target_pixels(API::UObject* rt) {
#if HALO_VR_DEV
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
#else
    // Dev-only diagnostic: omitted from release builds (see DevTools.hpp).
#endif
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
#if HALO_VR_DEV
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
#else
    // Dev-only diagnostic: omitted from release builds (see DevTools.hpp).
#endif
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
#if HALO_VR_DEV
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
#else
    // Dev-only diagnostic: omitted from release builds (see DevTools.hpp).
#endif
}

// One-shot driver for the matdump config key.
void run_mat_dump() {
#if HALO_VR_DEV
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
#else
    // Dev-only diagnostic: omitted from release builds (see DevTools.hpp).
#endif
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
#if HALO_VR_DEV
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
#else
    // Dev-only diagnostic: omitted from release builds (see DevTools.hpp).
#endif
}


// ---------------------------------------------------------------- SHIELD FX CENSUS  [dev only]
//
// WHAT QUESTION THIS ANSWERS
// The scope's capture renders enemy shields as unshaded outlines: the geometry arrives, the
// shading does not. The leading explanation is that their look is produced in POST, which the
// engine force-disables for every scene-colour capture (SceneCaptureRendering.cpp:853). That is a
// THEORY until the shield primitives themselves are read. Three reads decide it:
//
//   bRenderCustomDepth / CustomDepthStencilValue -- a mesh rendering to custom depth is being
//       drawn by a post-process pass keyed on depth/stencil. Set on shields => the post-process
//       explanation is CONFIRMED, and it also explains the unlit projectiles in one stroke.
//   bHiddenInSceneCapture / bVisibleInSceneCaptureOnly -- checked FIRST because if the game simply
//       hides shields from scene captures, every other explanation is irrelevant and the fix is
//       one property write. Never looked at until now.
//   material domain / blend mode / shading model -- says whether a capture-only stand-in could
//       ever look right, and whether the material is one a scene capture can shade at all.
//
// Same discipline as material_hunt above: chunked (a full sweep is ~296k objects and doing it in
// one tick collapses framerate -- this has already cost a live session once), throttled, and it
// stops for good. Dev builds only.
int32_t g_shield_census_idx  = 0;
bool    g_shield_census_done = false;
int     g_shield_census_hits = 0;
int     g_shield_census_pass = 0;
int32_t g_shield_census_last_n = 0;

#if HALO_VR_DEV
// Read a bool UPROPERTY by name, honouring the bitfield mask. get_property_data<uint8_t> alone
// returns the byte SHARED by up to eight bitfields, so without the mask every packed flag reads
// as whatever its neighbours happen to be -- a false positive generator.
static int read_bool_prop(API::UObject* obj, const wchar_t* name) {
    if (obj == nullptr) return -1;
    auto* cls = obj->get_class();
    if (cls == nullptr) return -1;
    auto* prop = cls->find_property(name);
    if (prop == nullptr) return -1;
    auto* byte = reinterpret_cast<uint8_t*>(obj) + prop->get_offset();
    auto* bp = static_cast<API::FBoolProperty*>(prop);
    const uint8_t mask = bp->get_field_mask();
    // A native (non-bitfield) bool carries a full 0xFF mask, so the masked test covers both cases.
    // The mask==0 arm is defensive only.
    return (mask == 0) ? (*byte != 0) : ((*byte & mask) != 0);
}

static int read_byte_prop(API::UObject* obj, const wchar_t* name) {
    if (obj == nullptr) return -1;
    auto* cls = obj->get_class();
    if (cls == nullptr) return -1;
    auto* prop = cls->find_property(name);
    if (prop == nullptr) return -1;
    return *(reinterpret_cast<uint8_t*>(obj) + prop->get_offset());
}

static std::string outer_path_of(API::UObject* o) {
    std::string path;
    for (API::UObject* q = o; q != nullptr; q = q->get_outer()) {
        const auto* fn = q->get_fname();
        path = (fn != nullptr ? narrow(fn->to_string()) : std::string("?"))
             + (path.empty() ? "" : "." + path);
    }
    return path;
}
#endif

void shield_fx_census(uint32_t tick) {
#if HALO_VR_DEV
    if (!g_cfg.shield_census || g_shield_census_done) return;

    static uint32_t last = 0;
    if (tick - last < 4) return;
    last = tick;

    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;
    const int32_t n = arr->get_object_count();
    if (g_shield_census_idx == 0 && g_shield_census_pass > 0 && n < g_shield_census_last_n + 5000)
        return;

    int processed = 0;
    while (g_shield_census_idx < n && processed < 2000) {
        auto* o = arr->get_object(g_shield_census_idx++);
        ++processed;
        if (o == nullptr) continue;

        const std::wstring cn = class_name_of(o);
        // MESHES *AND* PARTICLE SYSTEMS. The original filter listed three mesh component classes
        // and therefore never once looked at a NiagaraComponent -- which matters because the pak
        // inventory shows the overshield pickup is built from NS_Overshield_PickupHolo (a Niagara
        // SYSTEM) plus flare/glow materials, not from a mesh at all. The findings doc retired the
        // "Niagara is excluded from captures" theory FOR SHIELDS, which are skeletal meshes, and
        // said in the same breath that it might still hold elsewhere. This is elsewhere, and we
        // had no instrument pointed at it.
        //
        // UNiagaraComponent derives from UPrimitiveComponent, so bHiddenInSceneCapture,
        // bVisibleInSceneCaptureOnly and bRenderInMainPass all apply to it -- meaning the single
        // most valuable check (is the game hiding its particle FX from scene captures?) is the
        // same property read we already do for meshes.
        const bool is_mesh = (cn == L"SkeletalMeshComponent" || cn == L"StaticMeshComponent" ||
                              cn == L"InstancedStaticMeshComponent");
        const bool is_fx   = (cn.find(L"Niagara") != std::wstring::npos ||
                              cn.find(L"Particle") != std::wstring::npos);
        if (!is_mesh && !is_fx) continue;

        // LIVE INSTANCES ONLY. A class default object and a Blueprint archetype both carry the
        // same properties and would report confidently about geometry that is not in the level --
        // the exact false positive the earlier shield probe hit. Everything actually placed or
        // spawned lives under PersistentLevel.
        const std::string path = outer_path_of(o);
        if (path.find("PersistentLevel") == std::string::npos) continue;

        // MATCH ON THE WHOLE PATH, not just the component's own name. Jackal shields happen to
        // name the component "Shield" (verified), but that was luck: a cover shield whose mesh
        // component is called something else would report ZERO live shields and read as "none in
        // range" rather than as a filter that missed. The owning ACTOR carries the word in every
        // case seen so far (BP_JackalShieldWeaponActor_C, BP_CovPortableShield...), so testing the
        // path catches both. Broader on purpose -- extra rows are cheap here and each one prints
        // the material it wears, whereas a silent miss costs a whole test session.
        std::string hay = path;
        for (auto& c : hay) if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
        if (hay.find("shield") == std::string::npos) continue;

        if (is_fx && g_shield_census_hits < 24) {
            // A PARTICLE SYSTEM ROW. No GetMaterial walk -- a Niagara component's look comes from
            // its emitters, not from element 0 -- but the scene-capture visibility flags are the
            // whole reason to be here, so they are read exactly as for a mesh.
            const int hsc  = read_bool_prop(o, L"bHiddenInSceneCapture");
            const int vsco = read_bool_prop(o, L"bVisibleInSceneCaptureOnly");
            const int mainp= read_bool_prop(o, L"bRenderInMainPass");
            const int vis  = read_bool_prop(o, L"bVisible");
            const int act  = read_bool_prop(o, L"bIsActive");
            // The Niagara SYSTEM asset this component is playing, so the row names the effect
            // rather than just the component.
            std::string asset = "<none>";
            if (auto* c = o->get_class()) {
                if (auto* ap = c->find_property(L"Asset")) {
                    auto* sys = *reinterpret_cast<API::UObject**>(
                        reinterpret_cast<uint8_t*>(o) + ap->get_offset());
                    if (sys != nullptr) asset = outer_path_of(sys);
                }
            }
            API::get()->log_info(
                "[Halo-CampE-UEVR] SHIELDFX-FX %s :: class=%s hiddenInCapture=%d captureOnly=%d "
                "mainpass=%d visible=%d active=%d asset='%s'",
                path.c_str(), narrow(cn).c_str(), hsc, vsco, mainp, vis, act, asset.c_str());
            ++g_shield_census_hits;
            continue;
        }
        if (g_shield_census_hits < 24) {
            const int cd   = read_bool_prop(o, L"bRenderCustomDepth");
            const int hsc  = read_bool_prop(o, L"bHiddenInSceneCapture");
            const int vsco = read_bool_prop(o, L"bVisibleInSceneCaptureOnly");
            const int mainp= read_bool_prop(o, L"bRenderInMainPass");
            int stencil = -1;
            if (auto* cls = o->get_class()) {
                if (auto* p = cls->find_property(L"CustomDepthStencilValue"))
                    stencil = *reinterpret_cast<int32_t*>(reinterpret_cast<uint8_t*>(o) + p->get_offset());
            }

            // Material element 0. GetMaterial is BlueprintPure on UPrimitiveComponent
            // (PrimitiveComponent.h:1431), so it is reachable by reflection: int32 ElementIndex at
            // offset 0, the returned pointer at 8.
            API::UObject* mat = nullptr;
            {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                *reinterpret_cast<int32_t*>(p) = 0;
                o->call_function(L"GetMaterial", p);
                mat = *reinterpret_cast<API::UObject**>(p + 8);
            }
            // Walk MaterialInstance -> Parent to the base UMaterial, which is where the domain and
            // blend mode actually live. Bounded: a cyclic or absurd chain must not hang a tick.
            std::string mat_name = (mat != nullptr) ? outer_path_of(mat) : std::string("<none>");
            API::UObject* base = mat;
            for (int hop = 0; hop < 4 && base != nullptr && class_name_of(base) != L"Material"; ++hop) {
                auto* cls = base->get_class();
                auto* pp = (cls != nullptr) ? cls->find_property(L"Parent") : nullptr;
                base = (pp != nullptr)
                     ? *reinterpret_cast<API::UObject**>(reinterpret_cast<uint8_t*>(base) + pp->get_offset())
                     : nullptr;
            }
            const int domain = read_byte_prop(base, L"MaterialDomain");
            const int blend  = read_byte_prop(base, L"BlendMode");
            const int shade  = read_byte_prop(base, L"ShadingModel");

            // FULL PROPERTY DUMP, ONCE PER DISTINCT BASE MATERIAL.
            //
            // The shield's colour is lost in the capture while its DISTORTION survives -- measured
            // with this exact material on a cube (scopetest=6), correct in the world, colourless in
            // the pane. That splits the material's two contributions, and the only way to reason
            // about which is which is to read what the material actually declares. Dumped once
            // rather than per primitive: all 54 shields share one base material, and 54 copies of
            // the same 20 fields would bury it.
            //
            // The reference to diff against is EmissiveMeshMaterial, whose equivalent dump is in
            // docs\BLAM_AIM_FINDINGS.md -- an unlit material that behaves normally.
            static API::UObject* s_dumped_base = nullptr;
            if (base != nullptr && base != s_dumped_base) {
                s_dumped_base = base;
                API::get()->log_info(
                    "[Halo-CampE-UEVR] SHIELDMAT %s :: usesDistortion=%d refractMode=%d "
                    "refractMethod=%d translucencyPass=%d sepTranslucency=%d translucencyLighting=%d "
                    "twoSided=%d thinSurface=%d disablePreExposure=%d forceNoEmissive=%d "
                    "allowNegEmissive=%d disableDepthTest=%d frontLayer=%d screenSpaceRefl=%d "
                    "usedWithSkel=%d usedWithNiagaraMesh=%d",
                    outer_path_of(base).c_str(),
                    read_bool_prop(base, L"bUsesDistortion"),
                    read_byte_prop(base, L"RefractionMode"),
                    read_byte_prop(base, L"RefractionMethod"),
                    read_byte_prop(base, L"TranslucencyPass"),
                    read_bool_prop(base, L"bEnableSeparateTranslucency"),
                    read_byte_prop(base, L"TranslucencyLightingMode"),
                    read_bool_prop(base, L"TwoSided"),
                    read_bool_prop(base, L"bIsThinSurface"),
                    read_bool_prop(base, L"bDisablePreExposureScale"),
                    read_bool_prop(base, L"bForceNoEmissive"),
                    read_bool_prop(base, L"bAllowNegativeEmissiveColor"),
                    read_bool_prop(base, L"bDisableDepthTest"),
                    read_bool_prop(base, L"bAllowFrontLayerTranslucency"),
                    read_bool_prop(base, L"bScreenSpaceReflections"),
                    read_bool_prop(base, L"bUsedWithSkeletalMesh"),
                    read_bool_prop(base, L"bUsedWithNiagaraMeshParticles"));
            }

            API::get()->log_info(
                "[Halo-CampE-UEVR] SHIELDFX %s :: customdepth=%d stencil=%d hiddenInCapture=%d "
                "captureOnly=%d mainpass=%d mat='%s' base='%s' domain=%d blend=%d shading=%d",
                path.c_str(), cd, stencil, hsc, vsco, mainp, mat_name.c_str(),
                base != nullptr ? outer_path_of(base).c_str() : "<unresolved>",
                domain, blend, shade);
        }
        ++g_shield_census_hits;
    }

    if (g_shield_census_idx >= n) {
        ++g_shield_census_pass;
        g_shield_census_last_n = n;
        API::get()->log_info("[Halo-CampE-UEVR] SHIELDFX pass %d complete: %d live shield "
                             "primitive(s) from %d objects%s",
                             g_shield_census_pass, g_shield_census_hits, n,
                             g_shield_census_hits == 0
                                 ? "  <-- none in range; stand near a shielded enemy and retry"
                                 : "");
        // Shields belong to enemies that stream in, so a first pass can legitimately find nothing.
        // Re-sweep on meaningful array growth, same reasoning as material_hunt.
        if (g_shield_census_pass >= 4) {
            g_shield_census_done = true;
            API::get()->log_info("[Halo-CampE-UEVR] SHIELDFX finished after %d passes",
                                 g_shield_census_pass);
        } else {
            g_shield_census_idx = 0;
            g_shield_census_hits = 0;
        }
    }
#else
    (void)tick;   // Dev-only diagnostic: omitted from release builds (see DevTools.hpp).
#endif
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

// ---- THE AUTHORITATIVE CUTSCENE SIGNAL
//
// BlamCinematicSubsystem::IsCinematicInProgress() -- the game's own cutscene state, verified
// live on this title: true for the entire span of a pre-rendered movie, false outside it.
// Like the UI manager above, it is a GameInstanceSubsystem: it exists from boot and UEVR's
// reflection is safe on it. (Its LevelSequenceActor field is NOT usable -- the actor it points
// at is transient and dies while the movie is still playing.) The camera-class heuristic in
// the cutscene block stays as the fallback for a build where this class is missing.
TrackedObject g_cine_subsystem;

API::UObject* find_cine_subsystem() {
    if (auto* cached = g_cine_subsystem.get_checked(L"BlamCinematicSubsystem")) {
        return cached;
    }

    // A miss means a full object-array sweep to look again, so retry only every 128th call --
    // on a build without the class that is one sweep per ~30 s at the 4 Hz poll, not one per
    // poll forever.
    static uint32_t miss_calls = 0;
    if ((miss_calls++ & 127) != 0) return nullptr;

    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return nullptr;

    const int32_t n = arr->get_object_count();
    for (int32_t i = 0; i < n; ++i) {
        auto* o = arr->get_object(i);
        if (o == nullptr) continue;
        if (class_name_of(o).find(L"BlamCinematicSubsystem") == std::wstring::npos) continue;
        auto* cls = o->get_class();
        if (cls != nullptr && o == cls->get_class_default_object()) continue;
        g_cine_subsystem.set_at(o, i);
        API::get()->log_info("[Halo-CampE-UEVR] BlamCinematicSubsystem acquired @%p", (void*)o);
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
// Last tick the aim path actually reached reticule_widget_ensure(). needs_pick mirrors ensure's
// OWN early-out, but ensure also has a CALLER precondition -- the rig/pawn must have resolved --
// and without this stamp the sweep below cannot see it: measured 2026-08-19, a session whose rig
// never came up (SimVR, controllers idle) ran the ~55 ms sweep every ~4 s for five minutes
// straight, feeding a pick that could never bind. Stamped at both ensure call sites.
uint32_t g_ret_ensure_seen_tick = 0;

// ================================================================================================
// THE WIDGET SWEEP -- WHY IT IS SLICED, AND WHY MAKING IT "FASTER" WAS NEVER GOING TO BE ENOUGH
// ================================================================================================
//
// This sweep has been optimised twice and hitched three times. The 2026-08-12 hoist-and-memoise
// took it from 83.9 ms to 29.8 ms and is fully intact; it was measured back at 49-55 ms on
// 2026-08-23. Read from one live session of ~19 minutes (89 perf windows, perflog=1):
//
//   40 sweeps, mean 49.9 ms, worst 55.4 ms, 2.0 SECONDS of game-thread stall in total,
//   delivered as 40 dropped frames clustered into 12 bursts of 3-5.
//
// TWO THINGS THAT LOG SETTLES, both of which had been guessed at wrongly before:
//
// 1. IT IS NOT PERIODIC BACKGROUND COST. It is EVENT-DRIVEN, and the event is resolve_rig
//    losing its cached weapon actor. In every one of the 12 windows where this sweep ran,
//    resolve_rig's own max jumps from 0.002 ms (its O(1) fast path) to 22-25 ms (its fallback
//    sweep) in the same or the preceding window; in all 77 other windows this sweep did not run
//    at all. The chain is: fast path fails -> fallback sweep re-finds a weapon actor that is not
//    the cached one -> reticle_arm_stray_check() -> a ~400-tick (~10 s) window in which the
//    120-tick throttle fires 3-5 sweeps. One weapon-actor churn therefore costs ~150-250 ms of
//    THIS function plus ~25-60 ms of resolve_rig. The other gates were all quiet: the widget was
//    bound (needs_pick false), NAVFIX resolved 2-3 containers, and IsUIActiveState answered, so
//    the UI-manager fallback was dead.
//
// 2. IT WAS NOT THE OBJECT COUNT. That was the obvious suspect -- FUObjectArray's count is
//    NumElements on the chunked array, a HIGH-WATER MARK that UE never shrinks, so an O(N) sweep
//    does get permanently slower as levels stream in. It is a real effect and it is not this one.
//    The instrumentation added with this change settles it: `objects=298997`, against the ~296k
//    the 2026-08-12 measurement was taken at. The array did not grow, and the cost was flat at
//    48.6-55.4 ms across the 19-minute session. Keep reading `objects=` next to any sweep timing
//    anyway -- two timings at different array sizes are still not comparable, and the only reason
//    this could be ruled out is that the number is now recorded.
//
//    WHAT IT ACTUALLY WAS: the 2026-08-12 fix memoised the class NAME. That removed the FName
//    conversion, but every object still paid a lookup into a node-based map keyed on UClass*,
//    then a SECOND dependent chase into that name's heap buffer, then ~7 substring scans over it.
//    With 7,053 distinct classes loaded (also now logged) that two-miss chain is far outside
//    cache, so the per-object cost scales with how much content is loaded rather than with the
//    work being done. The navpoint search added since made it eight scans instead of seven --
//    real, but single-digit milliseconds, not the other twenty.
//
//    So this memoises the ANSWER, not the name: one byte in the map node, no string touched on a
//    hit, no scanning at all for the ~292k objects whose class was already judged. MEASURED after
//    the change, same session, same level, same array: 298,997 objects walked in 21.59 ms total
//    (was 49.9 ms mean), finding the same 2 reticle widgets and 3 navpoint containers as before.
//
// The remaining ~21 ms is the walk itself and is not reducible by cleverness: for every object we
// must touch the object's own header to read its class pointer, and those headers are scattered
// across chunks -- one cache miss per object, three hundred thousand of them, before any of our
// own work happens. Memoisation can only remove work done AFTER that miss. (resolve_rig's sweep,
// which does almost nothing per object, costs 22-25 ms on the same array: that is the floor.)
//
// So the walk is SLICED across ticks: each tick spends at most `retsweepms` (default 2 ms) on it
// and the pass continues where it left off. MEASURED: a pass now completes in 10 slices, and the
// perf window reports `reticle_rescan n=40 max=3.777ms mean=2.219ms` where it previously reported
// `n=5 max=55.442ms`. The 3.8 ms outlier is the budget-check granularity (it is sampled every
// 4096 objects, so a slow chunk can overrun); tightening that trades against a pass taking more
// ticks to finish, and 3.8 ms of a 25 ms frame did not need it. retsweepms=0 restores the old
// single-tick behaviour for A/B, the same way rigfast=0 A/Bs resolve_rig's fast path.
//
// STILL OPEN, and visible in the same log: resolve_rig's fallback sweep is 22-25 ms and fires in
// the same bursts, so it is a dropped frame in its own right. It cannot be sliced the same way --
// it has to return a rig THIS tick -- so it needs its own answer. And the burst is armed by
// `g_fp_weapon.get() != obj`, which is true whenever the tracked handle merely failed to
// revalidate, not only when the weapon genuinely changed; tightening that would cut how often
// the burst starts. Left alone deliberately: a missed HUD rebuild puts two crosshairs on the
// player's screen, and that trade needs a headset to judge, not an argument.
//
// Results are published ATOMICALLY at the end of a pass, into shadow arrays, so no consumer ever
// sees a half-built list. Every published handle has its class RE-CHECKED at publish time (~20
// objects), which also closes a staleness window that slicing would otherwise open: the verdict
// cache is keyed on a UClass POINTER, and a class freed mid-pass whose address is reused would
// otherwise answer with the old verdict.
//
// What this does NOT do is stop sweeping. The three things the sweep looks for are things we do
// not have yet (a reticle widget to bind, a navpoint layer, menu candidates while the UI-manager
// subsystem is unavailable), so there is no cached route to re-derive them from the way
// resolve_rig re-derives the rig through a cached weapon actor. Getting rid of the walk entirely
// needs a HUD-anchored lookup (HUD actor -> WidgetTree -> named child) whose reflection path has
// not been verified live on this title -- a design change, not an optimisation.

// One flag byte per CLASS, so the substring searches happen once per distinct class instead of
// once per object. The 2026-08-12 fix memoised the class NAME and then searched that name again
// for every object; this memoises the ANSWER. Same strings, same order, same results -- the
// searches are a pure function of the class name and of values that cannot change during a pass.
constexpr uint8_t VERDICT_RETICLE  = 0x01;
constexpr uint8_t VERDICT_MENU     = 0x02;
constexpr uint8_t VERDICT_NAV      = 0x04;
constexpr uint8_t VERDICT_WIDGETISH= 0x08;   // menudump discovery only

// A sweep in progress. Everything the loop needs is captured HERE at pass start rather than read
// from g_cfg inside the loop: a pass now spans several ticks and the config is live-reloaded, so
// reading it per object could change the meaning of a sweep halfway through it.
//
// The found objects are held as TrackedObject, NOT as raw pointers, and that is not a style
// choice. A pass now spans several ticks, and the cardinal rule on this title is that a raw
// API::UObject* must never be carried across a frame -- objects here are pooled and a slot is
// reused in place. TrackedObject carries the array index with the pointer, so at publish time
// get() can prove the slot still holds the same object and a corpse reads as null instead of
// being handed to reticle_collapse_strays(), which would remove it from its parent.
struct RescanPass {
    bool     active      = false;
    int32_t  cursor      = 0;      // next object index to examine
    int32_t  total       = 0;      // object count captured at pass start
    uint32_t start_tick  = 0;
    uint32_t slices      = 0;
    double   spent_ms    = 0.0;

    TrackedObject ret[8];   int ret_n  = 0;
    TrackedObject nav[4];   int nav_n  = 0;
    TrackedObject menu[8];  int menu_n = 0;

    std::wstring wanted;
    wchar_t      nav_w[64] = {0};
    bool         menu_detect = false;
    bool         menu_dump   = false;
    const char*  nav_eff     = "";

    std::unordered_map<const void*, uint8_t> verdict;
};
RescanPass g_pass;

// Distinct classes seen by the last COMPLETED pass, so the next one can reserve() instead of
// rehashing its way up from one bucket. Kept outside the pass because the pass is reset wholesale.
size_t g_last_class_count = 512;

// Verdict for one object's class, computed once per class per pass.
// Takes the object (not just the class) because class_name_of() walks object -> class -> fname.
uint8_t class_verdict(const void* cls, API::UObject* o) {
    auto it = g_pass.verdict.find(cls);
    if (it != g_pass.verdict.end()) return it->second;

    const std::wstring cn = class_name_of(o);
    uint8_t f = 0;
    if (cn.find(g_pass.wanted) != std::wstring::npos) f |= VERDICT_RETICLE;
    if (g_pass.menu_detect && is_menuish_class(cn))   f |= VERDICT_MENU;
    if (g_pass.nav_w[0] != 0 && cn.find(g_pass.nav_w) != std::wstring::npos) f |= VERDICT_NAV;
    if (g_pass.menu_dump && (cn.find(L"WBP_") != std::wstring::npos ||
                             cn.find(L"UserWidget") != std::wstring::npos)) f |= VERDICT_WIDGETISH;

    g_pass.verdict.emplace(cls, f);
    return f;
}

// Publish a completed pass into the arrays the rest of the plugin reads.
//
// RE-CHECKING THE CLASS HERE IS NOT BELT-AND-BRACES. reticle_collapse_strays() is DESTRUCTIVE --
// it removes widgets from their parent -- and it decides "stray" purely from this list. A handle
// that reached the list through a recycled UClass address, or an object-array slot reused in
// place between the slice that found it and the tick that consumes it, would be acted on. Twenty
// class-name reads is nothing next to the walk that produced them.
void rescan_publish(uint32_t tick) {
    g_reticle_count = 0;
    for (int i = 0; i < g_pass.ret_n; ++i) {
        auto* p = g_pass.ret[i].get();                      // dead or recycled slot -> null
        if (p == nullptr) continue;
        if (class_name_of(p).find(g_pass.wanted) == std::wstring::npos) continue;
        g_reticles[g_reticle_count].obj = g_pass.ret[i];
        // Stamped with the value pick_live_reticle() compares against, so the freshly published
        // list reads as current. g_reticle_scan_tick holds the PASS-START tick (set by the
        // throttle at the top of reticle_rescan), and is not advanced again until the next pass.
        g_reticles[g_reticle_count].found_tick = g_reticle_scan_tick;
        ++g_reticle_count;
    }

    g_nav_count = 0;
    for (int i = 0; i < g_pass.nav_n && g_pass.nav_w[0] != 0; ++i) {
        auto* p = g_pass.nav[i].get();
        if (p == nullptr) continue;
        if (class_name_of(p).find(g_pass.nav_w) == std::wstring::npos) continue;
        g_navpoints[g_nav_count++] = g_pass.nav[i];
    }

    g_menu_candidate_count = 0;
    for (int i = 0; i < g_pass.menu_n; ++i) {
        auto* p = g_pass.menu[i].get();
        if (p == nullptr) continue;
        if (!is_menuish_class(class_name_of(p))) continue;
        g_menu_candidates[g_menu_candidate_count++] = g_pass.menu[i];
    }

    // ---- PROOF THAT THE SLICED PATH ACTUALLY RAN, and the evidence for the growth story above.
    //
    // Not "the symbol is in the binary" and not "it compiled": this line only appears if a pass
    // reached completion, and it carries the two numbers that decide whether the sweep is still
    // a problem -- how big the array has become, and how much of one frame a slice cost. Logged
    // on the FIRST completed pass and thereafter only when the array size moves by more than
    // 10%, so it is a growth curve rather than log spam.
    static int32_t last_logged_total = 0;
    const int32_t  delta = g_pass.total - last_logged_total;
    if (last_logged_total == 0 ||
        (delta > 0 ? delta : -delta) * 10 > last_logged_total) {
        last_logged_total = g_pass.total;
        API::get()->log_info("[Halo-CampE-UEVR] widget sweep: %d objects in %u slice(s), "
                             "%.2f ms total (%.2f ms/slice), %d classes; "
                             "%d reticle / %d nav / %d menu",
                             g_pass.total, g_pass.slices, g_pass.spent_ms,
                             g_pass.slices != 0 ? g_pass.spent_ms / (double)g_pass.slices : 0.0,
                             (int)g_pass.verdict.size(),
                             g_reticle_count, g_nav_count, g_menu_candidate_count);
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
                             narrow(g_pass.wanted).c_str());
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

    // Navpoint resolution, logged on CHANGE for the same reason as the reticles above. "0
    // resolved" while navfix=1 means the navclass guess is wrong -- menudump the live widget
    // names and correct the config, not the code.
    if (g_cfg.nav_fix || g_cfg.nav_world) {
        static int last_nav = -1;
        if (g_nav_count != last_nav) {
            last_nav = g_nav_count;
            API::get()->log_info("[Halo-CampE-UEVR] NAVFIX: %d navpoint container(s) resolved "
                                 "(navclass='%s')", g_nav_count, g_pass.nav_eff);
        }
    }

    // Hide any of the game's flat crosshairs that are not the one we host. Placed HERE, at the end
    // of the pass, so it reuses the list that was just built and costs nothing of its own -- and
    // so it sees the complete list rather than deciding "stray" from a partial scan. That last
    // point is exactly why the slices write to shadow arrays: a partial list published mid-pass
    // would make this collapse widgets it has not finished looking at.
    reticle_collapse_strays();

    (void)tick;
    g_last_class_count = g_pass.verdict.size();
    g_pass.active = false;
    g_pass.verdict.clear();
}

// One tick's worth of the walk. Returns with the pass either advanced or finished.
void rescan_slice(uint32_t tick) {
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) { g_pass.active = false; return; }

    // The array only grows within a session, but never trust that: a shorter array than the one
    // the pass started on means indices past the end, and get_object() would be reading a chunk
    // that no longer exists.
    const int32_t now_n = arr->get_object_count();
    if (now_n < g_pass.total) g_pass.total = now_n;

    // 0 disables slicing and walks the whole array in this tick -- the pre-2026-08-23 behaviour,
    // kept so the change can be A/B'd in a headset without a rebuild.
    const double budget = (double)g_cfg.ret_sweep_ms;

    LARGE_INTEGER t0{};
    QueryPerformanceCounter(&t0);
    ++g_pass.slices;

    int32_t i = g_pass.cursor;
    for (; i < g_pass.total; ++i) {
        // BUDGET CHECK AT THE TOP OF THE BODY, NOT THE BOTTOM.
        //
        // Almost every object fails the verdict test below and hits `continue`, so a check placed
        // after that work is reached a few dozen times per pass instead of a few dozen times per
        // SLICE -- the budget would be silently ignored and the whole array would be walked in one
        // tick, which is precisely the stall this exists to remove. It would still compile, still
        // log, and still look like it was working.
        //
        // Sampled every 4096 objects. Two QPC calls per 4096 objects is far below the noise floor
        // of the walk itself, and a power-of-two boundary keeps the test a mask. `i != cursor`
        // stops it firing on the first iteration of a slice that starts on a boundary, which would
        // make no progress at all. Breaking BEFORE processing i leaves the cursor exactly right.
        if (budget > 0.0 && (i & 0xFFF) == 0 && i != g_pass.cursor) {
            LARGE_INTEGER tn{};
            QueryPerformanceCounter(&tn);
            if ((double)(tn.QuadPart - t0.QuadPart) * perf_tick_ms() >= budget) break;
        }

        auto* o = arr->get_object(i);
        if (o == nullptr) continue;
        auto* ocls = o->get_class();
        if (ocls == nullptr) continue;

        const uint8_t f = class_verdict(ocls, o);
        if ((f & (VERDICT_RETICLE | VERDICT_MENU | VERDICT_NAV)) == 0 && !g_pass.menu_dump) continue;

        if (o == ocls->get_class_default_object()) continue;   // never the CDO

        if ((f & VERDICT_RETICLE) && g_pass.ret_n  < 8) g_pass.ret [g_pass.ret_n++ ].set_at(o, i);
        if ((f & VERDICT_NAV)     && g_pass.nav_n  < 4) g_pass.nav [g_pass.nav_n++ ].set_at(o, i);
        if ((f & VERDICT_MENU)    && g_pass.menu_n < 8) g_pass.menu[g_pass.menu_n++].set_at(o, i);

        // Discovery. The match list above is a guess at this game's naming, and a guess that fails
        // silently would leave menu detection permanently off with no clue why. With menudump=1,
        // open a pause menu and the log names every widget that is actually in the viewport.
        if (g_pass.menu_dump && !(f & VERDICT_RETICLE) && (f & VERDICT_WIDGETISH) &&
            call_ret_bool(o, L"IsInViewport")) {
            API::get()->log_info("[Halo-CampE-UEVR] MENUDUMP in-viewport widget: %s",
                                 narrow(class_name_of(o)).c_str());
        }
    }

    LARGE_INTEGER t1{};
    QueryPerformanceCounter(&t1);
    g_pass.spent_ms += (double)(t1.QuadPart - t0.QuadPart) * perf_tick_ms();

    g_pass.cursor = i;
    if (i >= g_pass.total) rescan_publish(tick);
}

void reticle_rescan(uint32_t tick) {
    // A pass already in flight owns the next slice; the gating below is per PASS, not per tick.
    // Checked first so the throttle stamp and the demand gate are not re-evaluated mid-pass.
    //
    // ABANDON A PASS THAT HAS BEEN SUSPENDED. This function is not called at all while the
    // frontend is up (see the call site), so a pass started just before the player opened the menu
    // sits half-finished until they come back -- possibly across a level transition, possibly
    // minutes later. Publishing it then would hand pick_live_reticle() a list stitched together
    // from two different worlds and stamp it as the current sweep, which is exactly the staleness
    // its found_tick check exists to reject. Five throttle periods is far longer than any healthy
    // pass (a pass is tens of ticks) and far shorter than a menu visit.
    if (g_pass.active && tick - g_pass.start_tick > 600) {
        g_pass = RescanPass{};   // active=false; nothing published, the previous list stands
    }
    if (g_pass.active) {
        PerfScope _perf(PERF_RETICLE);
        rescan_slice(tick);
        return;
    }

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
    //   g_reticles         -> hud_reticle_follow (returns immediately when hudfollow=0)
    //     âš ï¸ hud_follow's CODE default is true, not 0. Both shipped and live profiles set
    //     hudfollow=0, so in practice this term is false -- but a config without that line
    //     re-enables the sweep permanently, which is the microstutter this gate exists to stop.
    //     The safe default is false; left as-is only because changing it is a behaviour change.
    //                      -> the widget reticule's ONE-SHOT pick, done once the component binds
    //   g_menu_candidates  -> menu_poll's FALLBACK only, dead while the UI-manager subsystem answers
    // So after the widget bound, this was rebuilding two arrays that nothing would ever look at.
    //
    // Each condition is re-tested every 120 ticks rather than latched, so turning hudfollow on in
    // the config, or losing the widget binding, brings the scan straight back. That is why this is
    // a demand check and not a "scanned once, done" flag.
    // needs_pick is AND-ed with "the consumer ran recently": ensure() is only called once a
    // rig/pawn has resolved, so while the rig is down a sweep feeds a pick nothing can consume.
    // The 240-tick window also covers cold boot (stamp still 0, tick small), priming candidates
    // before the first bind; after rig-up, ensure stamps every tick and the next 120-tick
    // boundary sweeps fresh -- worst case the bind waits one throttle period, same as today.
    const bool needed = g_cfg.menu_dump                              // discovery: the sweep IS the product
                     || g_cfg.hud_follow                             // moves/hides the flat reticle
                     || (reticle_widget_needs_pick()
                         && tick - g_ret_ensure_seen_tick < 240)     // still choosing, and bindable
                     || reticle_stray_check_due(tick)                // HUD rebuilt a second crosshair
                     || ((g_cfg.nav_fix || g_cfg.nav_world) && g_nav_count == 0)   // navpoint layer not yet resolved
                     || (g_cfg.menu_detect && !g_ui_manager_ok.load());   // candidates are the fallback
    if (!needed) return;

    PerfScope _perf(PERF_RETICLE);   // inside the gate: times the sweep, not the 119 early-outs

    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;

    // ---- BEGIN A PASS. Nothing published yet: g_reticles / g_navpoints / g_menu_candidates keep
    // the PREVIOUS pass's contents until this one completes, so a consumer that runs mid-pass sees
    // a complete (if slightly older) list rather than a half-built one. The old code could get
    // away with clearing them up front only because the whole walk happened inside one tick.
    g_pass = RescanPass{};
    g_pass.active      = true;
    g_pass.cursor      = 0;
    g_pass.total       = arr->get_object_count();
    g_pass.start_tick  = tick;
    g_pass.menu_detect = g_cfg.menu_detect;
    g_pass.menu_dump   = g_cfg.menu_dump;
    g_pass.wanted      = wanted_widget_class();

    // The navpoint class comes from config (navclass -- a pak-inventory guess until confirmed
    // live), so it is widened once per pass rather than per object. The default lives HERE, not in
    // the struct initializer: a char-array string default on the global g_cfg does not survive
    // MSVC's constant-initialization (see the note on nav_class in Config.hpp).
    g_pass.nav_eff = (g_cfg.nav_class[0] != '\0') ? g_cfg.nav_class : "WBP_Navpoints";
    if (g_cfg.nav_fix || g_cfg.nav_world) {
        for (size_t k = 0; k < 63 && g_pass.nav_eff[k] != '\0'; ++k) {
            g_pass.nav_w[k] = (wchar_t)(unsigned char)g_pass.nav_eff[k];
        }
    }

    // Sized from the last pass's class count, so the table does not rehash its way up from one
    // bucket every time. Distinct classes are in the low thousands and barely move between passes.
    g_pass.verdict.reserve(g_last_class_count + (g_last_class_count >> 2) + 64);

    rescan_slice(tick);
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

// SCREEN-SPACE NAVPOINT CORRECTION (navfix) -- docs\CAMERA_CONSUMERS_FINDINGS.md issue C.
//
// The game projects waypoint/objective markers against ITS OWN camera -- which the aim driver
// points at the controller -- while the player looks along the view lock's pinned yaw, so the
// whole projected layer drifts with the right hand (README known issue). The exact fix is
// upstream (feed the projection the rendered rotation at its source); this is the FIRST-ORDER
// correction: shift the navpoint container by where the RENDERED view's forward lands in the
// game's own projection. Exact for markers at the view centre, degrades with off-centre angle
// -- a uniform translation cannot re-project each marker individually -- and off-screen
// edge-clamped markers will still misbehave.
//
// Same arithmetic as hud_reticle_follow with the roles swapped: the reticle moves TO the aim
// direction (+tan(aim-view)); the game pre-shifted the marker layer BY -tan(aim-view), so the
// correction is +tan(aim-view) again. If one path ever reads correct and the other mirrored,
// suspect this reasoning first and A/B the sign by observation.
//
// DISENGAGE IS HALF THE FEATURE. In stick mode (and whenever the delta is unavailable) the
// game's own projection is already correct, so the shift must drop to ZERO, once -- a stale
// translation here is this exact bug re-introduced in reverse.
void hud_navpoint_follow(bool engaged, double aim_pitch, double aim_yaw, uint32_t tick) {
    if (!engaged || !g_cfg.nav_fix) {
        g_nav_apply = false;   // stop the render-rate writer BEFORE zeroing, or it re-shifts
        if (g_nav_shift_applied) {
            g_nav_shift_applied = false;
            for (int i = 0; i < g_nav_count; ++i) {
                auto* o = g_navpoints[i].get();
                if (o == nullptr) continue;
                // A zeroed buffer reads as (0,0) in both the float and the double layout, so the
                // release does not depend on hud_float being set correctly.
                alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
                o->call_function(L"SetRenderTranslation", q);
            }
            API::get()->log_info("[Halo-CampE-UEVR] NAVFIX: correction released");
        }
        return;
    }

    if (g_nav_count == 0) { g_nav_apply = false; return; }   // scan is driven from update()

    const float view_yaw   = g_render_view_yaw.load();
    const float view_pitch = g_render_view_pitch.load();
    const float dyaw   = wrap180((float)aim_yaw   - view_yaw);
    const float dpitch = g_cfg.nav_pitch ? wrap180((float)aim_pitch - view_pitch) : 0.0f;

    // Behind-the-view: tan() flips sign and a confident wrong shift is worse than none. Hold the
    // last value rather than snapping to zero -- markers are already meaningless at this angle.
    if (std::fabs(dyaw) > 89.0f || std::fabs(dpitch) > 89.0f) return;

    float px = 0.0f, py = 0.0f;
    bool  projected = false;

    // Engine projection first (same lane as hudproject above): project a point along the
    // RENDERED view direction through the game's own ProjectWorldToScreen -- which uses the aim
    // camera -- and read the correction as that point's distance from the viewport centre.
    // Exact at the view centre by construction, and no constant to calibrate.
    if (g_cfg.nav_project && g_have_view_pos.load()) {
        static API::UObject* gps = nullptr;
        static API::UObject* wll = nullptr;
        static bool tried = false;
        if (!tried) {
            tried = true;
            if (auto* c = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.GameplayStatics"))
                gps = c->get_class_default_object();
            if (auto* c = API::get()->find_uobject<API::UClass>(L"Class /Script/UMG.WidgetLayoutLibrary"))
                wll = c->get_class_default_object();
        }
        auto* pc = API::get()->get_player_controller(0);
        if (gps != nullptr && wll != nullptr && pc != nullptr) {
            const float vy = view_yaw * DEG2RAD, vp = view_pitch * DEG2RAD;
            const float cp = std::cos(vp);
            const Vec3 o{g_view_pos_x.load(), g_view_pos_y.load(), g_view_pos_z.load()};
            // 1000 cm along the view: the projection origin IS the camera whose position we
            // published, so the probe distance barely matters -- direction is the payload.
            const Vec3 w{o.x + cp * std::cos(vy) * 1000.0f,
                         o.y + cp * std::sin(vy) * 1000.0f,
                         o.z + std::sin(vp) * 1000.0f};

            double sx = 0.0, sy = 0.0;
            bool ok = false;
            {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                *reinterpret_cast<void**>(p) = pc;                 // Player
                auto* wv = reinterpret_cast<double*>(p + 8);       // WorldPosition
                wv[0] = w.x; wv[1] = w.y; wv[2] = w.z;
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
                px = (float)(vw * 0.5 - sx);
                py = (float)(vh * 0.5 - sy);
                if (!g_cfg.nav_pitch) py = 0.0f;
                projected = true;
            }
        }
    }

    // RENDER-RATE MODE: the tick's job ends at CALIBRATION. The probe above measured the game
    // projection's actual pixels-per-tan at this instant; fold it into the shared constant and
    // let the stereo callback do the writing with a same-frame delta. Sampled only when the
    // delta is big enough to carry signal -- near zero the division is noise.
    if (g_cfg.nav_render) {
        if (projected) {
            const float t = std::tan(dyaw * DEG2RAD);
            if (std::fabs(dyaw) >= 3.0f && std::fabs(t) > 0.01f) {
                const float k = px / t;
                if (std::isfinite(k) && k > 100.0f && k < 20000.0f) {
                    const float prev = g_nav_keff.load();
                    g_nav_keff = prev + 0.2f * (k - prev);
                }
            }
        } else {
            g_nav_keff = g_cfg.nav_k;   // navproject=0: the manual knob drives the render path
        }
        g_nav_apply = true;
        g_nav_shift_applied = true;   // the render path is writing; release must still zero
        static uint32_t last_klog = 0;
        if (tick - last_klog >= 600) {
            last_klog = tick;
            API::get()->log_info("[Halo-CampE-UEVR] NAVFIX: render-rate mode, keff=%.0f px/tan "
                                 "(dyaw=%.1f at calib)", g_nav_keff.load(), dyaw);
        }
        return;
    }

    if (!projected) {
        px = g_cfg.nav_k * std::tan(dyaw * DEG2RAD);
        py = -g_cfg.nav_k * std::tan(dpitch * DEG2RAD);
    }
    px = clampf(px, -g_cfg.nav_max, g_cfg.nav_max);
    py = clampf(py, -g_cfg.nav_max, g_cfg.nav_max);
    if (!std::isfinite(px) || !std::isfinite(py)) return;

    for (int i = 0; i < g_nav_count; ++i) {
        auto* o = g_navpoints[i].get();
        if (o == nullptr) {
            // Recycled slot: force a rescan and skip the tick -- writing through residue is how a
            // "working" feature pins itself to a dead object.
            g_nav_count = 0;
            return;
        }
        alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
        if (g_cfg.hud_float) {
            auto* f = reinterpret_cast<float*>(q);
            f[0] = px; f[1] = py;
        } else {
            auto* d = reinterpret_cast<double*>(q);
            d[0] = (double)px; d[1] = (double)py;
        }
        o->call_function(L"SetRenderTranslation", q);
    }
    g_nav_shift_applied = true;

    static uint32_t last_log = 0;
    if (tick - last_log >= 600) {           // ~20 s
        last_log = tick;
        API::get()->log_info("[Halo-CampE-UEVR] NAVFIX: dyaw=%.1f dpitch=%.1f -> shift=(%.0f,%.0f)%s",
                             dyaw, dpitch, px, py, projected ? " [projected]" : " [navk]");
    }
}

// WORLD-SPACE NAVPOINTS (navworld) -- the lane that has no angular ceiling.
//
// WHY (docs\CAMERA_CONSUMERS_FINDINGS.md, 2026-08-13): every screen-space correction is capped
// by the flat HUD quad, which spans only Â±20.7Â°Ã—Â±12.0Â° of view -- a marker for an enemy 30Â°
// off-centre CANNOT sit on that enemy no matter how exact the canvas math is. So markers move
// into the WORLD: one mesh marker per live navpoint, placed along the true world direction of
// its target, head-relative correct by construction.
//
// WHERE THE DIRECTION COMES FROM. The game's HaloUINavpointsManager projects each marker
// against ITS camera (the aim camera) and stamps the result into the container's canvas slots.
// That projection is a pinhole we can measure and invert:
//   * TWO probe points along known view-relative directions, pushed through the game's own
//     ProjectWorldToScreen, yield the projection constant (px per tan) AND the aim camera's
//     true yaw/pitch -- no trust in ControlRotation's pitch (which decouples from the camera).
//   * Each visible child's slot position then inverts to a world direction:
//       yaw = cam_yaw + atan(cx/keff), pitch = cam_pitch - atan(cy/keff)   (canvas y is DOWN)
// Slot positions are read one tick stale; the direction of a marker moves at target-screen
// speed (slow), unlike the aim-vs-view delta, so tick-rate reading is fine here.
//
// LIMITS, stated: markers the game has already edge-clamped (its off-screen arrows) carry no
// recoverable direction -- those are SKIPPED here and the flat layer's own edge arrow still
// serves. Distance is unknown per marker, so all markers sit at navworlddist with
// distance-normalised scale -- direction is what matters and is exact.
//
// DISENGAGE: stick mode / cutscenes / kill switch hide the whole pool one-shot; the game's
// flat layer is never touched by this lane (navfix's shift is forced released while navworld
// owns the job, so the flat markers stay game-native).
TrackedObject g_navw_pool[8];
bool          g_navw_mid_ok[8] = {};        // dynamic material bound on this slot
bool          g_navw_compensated = false;   // VREditor exposure-compensated MIC bound (pak present)

// LAST TRUE DIRECTION per marker, keyed by the marker's widget instance (element+0x08). While a
// marker is EDGE-CLAMPED its screen position tracks the aim frustum and the inversion yields a
// hand-following direction -- the lane's structural ceiling. But whenever the aim sweeps near
// enough that the marker is comfortably inside the canvas, the inversion is TRUE: cache that,
// and hold the marker at its last true direction while clamped. In normal play the aim crosses
// most of the view constantly, so directions refresh opportunistically and markers read as
// world-pinned without any Blam-side target data. Keys recycle with the game's widget pool; a
// recycled key serves a stale direction only until its first unclamped refresh.
struct NavwCachedDir { void* wkey = nullptr; float yaw = 0, pitch = 0; bool valid = false; };
NavwCachedDir g_navw_dirs[12];

// PUBLISHED FOR RENDER-RATE RE-PLACEMENT (lane 2). The tick resolves each objective's world
// position and how far along the ray to draw; the STEREO CALLBACK re-derives the marker's
// transform from the CURRENT eye every frame.
//
// This matters because the marker is drawn CLOSE (4 m): at that range the direction from eye to
// objective swings quickly as the player walks, so a transform computed at the ~32 Hz tick and
// left alone for the intervening 90 Hz frames visibly stutters -- field-reported as "a bit
// jittery when I walk around". Same lesson as the rig re-apply and movelive before it: compute
// in the callback that CONSUMES the value. Fourth occurrence.
struct NavwPlaced { float ox, oy, oz, dist; };
NavwPlaced       g_navw_placed[8] = {};
std::atomic<int> g_navw_placed_n{0};
// WHICH SLOTS the tick placed this frame, one bit per slot. Stable slots (BUG 1) are
// NON-CONTIGUOUS -- a navpoint can hold slot 0 and 5 with 1-4 empty -- so the render-rate
// re-place can no longer walk [0,n); it tests this mask per slot. Published after g_navw_placed
// is written, consumed on the stereo callback.
std::atomic<uint32_t> g_navw_placed_mask{0};

// The widget CLASS each pool slot is currently wearing. Navpoints are not all objectives --
// the same map carries co-op partners, tracked enemies, item highlights, each with its own
// authored art (and its own colour semantics). Every entry names its class, so a slot re-hosts
// whenever the navpoint occupying it changes type; without this every marker would wear the
// objective icon regardless of what it actually marks.
void* g_navw_slot_class[8] = {};

// What KIND of navpoint each slot is showing, classified from its widget class name. Drives
// per-kind sizing: an objective wants to be seen from anywhere, a floor weapon should not shout.
enum NavwKind : uint8_t { NAVW_OTHER = 0, NAVW_OBJECTIVE, NAVW_ALLY, NAVW_ENEMY, NAVW_ITEM };
NavwKind g_navw_slot_kind[8] = {};

// The size multiplier each slot RESOLVED to, decided once at host time where the class name is
// in hand (override first, then kind default). Placement reads this rather than re-deriving,
// so what the log printed and what the marker is drawn at cannot disagree.
float g_navw_slot_size[8] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

// STABLE COMPOSITOR-SLOT ASSIGNMENT (BUG 1: the flicker). Each navpoint keeps ONE slot across
// ticks, keyed to its IDENTITY -- NOT to how many EARLIER entries resolved this tick. Dense-packed
// resolve order made every later marker jump slots whenever an enemy navpoint entered or left the
// HMD view, so one slot thrashed between an objective and a floor-item icon every ~0.5 s.
// g_navw_slot_prio (1 = objective, 2 = everything else) lets a short slot budget shed the
// lowest-priority navpoint deterministically rather than by iteration order.
//
// ⚠️ THE IDENTITY WAS THE SPARSE TMap INDEX `s`, AND THAT WAS WRONG (corrected 2026-08-25).
// The old comment here asserted "a sparse-array index is stable for a given element across
// insert/remove". Refuted by the field log of 2026-08-25 13:54-14:08: across that session the
// summary line reported a steady `2 entr(ies) resolved` out of `map num=5`, yet the SAME two
// markers wandered over compositor slots 0,1,2,3,4 and re-typed between WBP_NavpointObjective_C
// and WBP_NavpointWidgetItemHighlight_C thirty times. Two independent reasons the index cannot
// carry identity here:
//   * The map is REBUILT, not merely appended to -- the same log shows num/max stepping
//     1/4 -> 4/4 -> 5/24, which is a reallocation-and-rehash, after which indices mean nothing.
//   * Only 2 of the 5 entries ever resolve a position (the other kinds carry none -- see the
//     lane-2 header), and WHICH ones resolve flickers. An entry dropping out retires its slot and
//     the next one to appear is handed a different free slot, so the assignment churns even while
//     the map itself is perfectly still.
// Each churn costs a re-host, and a re-host is where the WRONG ART reaches the eye.
//
// The identity is now the navpoint's LIVE WIDGET INSTANCE pointer (element+0x08, the same field
// the visibility gate already reads and validates by reflection). One widget per navpoint, created
// with it and destroyed with it, unmoved by any rehash of the map that indexes it. Entries whose
// widget cannot be validated fall back to the old index in a DISJOINT numeric space (bit 0 set --
// no 8-byte-aligned object pointer can collide with it), so an unreadable widget degrades to the
// previous behaviour for that one entry instead of colliding with a real identity. 0 = free.
uint64_t g_navw_slot_ident[8] = {};
int      g_navw_slot_prio[8]  = {99, 99, 99, 99, 99, 99, 99, 99};

// ---- NAVWORLD CENSUS: proof that each gate RAN, not merely that it was compiled ---------------
// Every one of these is a POSITIVE counter -- "N examined", not "something was suppressed" -- and
// they are printed by the always-compiled 600-tick summary line at the end of lane 2, so the proof
// survives into a release build and needs no cfg key to switch on. That last part is the whole
// point: the visibility gate shipped with its only telemetry behind `navworldlog`, which nothing
// sets, so a 7-minute session produced zero lines and left "did it ever run?" unanswerable. A
// silent log is not evidence of a quiet gate. See [[prove-the-tick-not-the-init]].
struct NavwCensus {
    uint32_t examined;    // entries whose position resolved (the gate's input population)
    uint32_t vis_sup;     // suppressed by navw_entry_shown (game had collapsed/hidden the widget)
    uint32_t kind_sup;    // suppressed by the navworldkindmask stopgap
    uint32_t noclass;     // dropped: no readable widget class -- would have worn the OBJECTIVE icon
    uint32_t beyond;      // resolved past map->num, i.e. in uninitialised sparse capacity
    uint32_t staleart;    // placement skipped: slot still wearing the previous navpoint's art
    uint32_t rehost;      // navw_host_class expensive-path completions
    uint32_t identfb;     // identities that fell back to the sparse index (widget unreadable)
    uint32_t vis_seen;    // BITMASK of ESlateVisibility values observed (bit n = value n seen)
    uint32_t deadslot;    // skipped: the map's own allocation bitmap says this slot is FREE. These
                          // are the phantoms -- a freed entry keeps a plausible stale position and
                          // was previously indistinguishable from a live one.
    uint32_t stale;       // rejected: a pointer read from a raw element offset was READABLE but
                          // is not a live UObject -- its InternalIndex slot does not hold it.
                          // This is the pointer class_name_of would otherwise have
                          // dereferenced; the 2026-09-08 tick faults read exactly that shape.
    uint32_t nullpos;     // rejected: no widget AND a position within 1 m of the world origin,
                          // i.e. zeroed/never-written memory that still resolves a position
    uint32_t slotreuse;   // a slot freed THIS tick had to be re-let the same tick (starvation
                          // fallback). Non-zero means the one-tick deferral could not hold and
                          // the outgoing marker's art can still lag onto an incoming one.
};
NavwCensus g_navw_census = {};

NavwKind navw_classify(const std::wstring& cn) {
    if (cn.find(L"Objective")    != std::wstring::npos
     || cn.find(L"Scripted")     != std::wstring::npos) return NAVW_OBJECTIVE;
    // CO-OP PARTNERS / allies. The pak carries WBP_NavpointWidgetPlayer and the
    // MI_UI_Navmarker_Ally_* materials; without this they classify as "other" and draw at FULL
    // base size -- bigger than the objectives, which is backwards.
    // RECON IS THE ALLY MARK ON THIS TITLE -- inferred by ELIMINATION, exactly as Destination=enemy
    // was, and flagged as an inference rather than a fact.
    //
    // A full census of every navpoint class this game has ever produced across seven archived
    // sessions returns FOUR: Objective (1044 rows), Destination (734), WidgetItemHighlight (543)
    // and Recon (247). Three are accounted for. The player reports a missing marker over Sgt
    // Johnson -- an ally -- and Recon is the only class left, at 22 distinct positions that move
    // like an NPC rather than sitting still like an objective. None of Player/Ally/Partner/
    // Teammate/Squad has EVER appeared, so there is no separate ally class to find.
    //
    // HOW TO FALSIFY IT: if Recon turns out to mark something else (a scanned point of interest, a
    // recon objective), it will show up somewhere no ally is standing. The cost of being wrong is a
    // marker drawn at ally SIZE instead of its right one -- visible, harmless, and easy to re-file.
    if (cn.find(L"Recon")        != std::wstring::npos) return NAVW_ALLY;
    if (cn.find(L"Player")       != std::wstring::npos
     || cn.find(L"Ally")         != std::wstring::npos
     || cn.find(L"Partner")      != std::wstring::npos
     || cn.find(L"Teammate")     != std::wstring::npos
     || cn.find(L"Squad")        != std::wstring::npos) return NAVW_ALLY;
    // DESTINATION IS THE ENEMY MARK ON THIS TITLE -- established by elimination, not by the
    // name. Across a full session the live map produced exactly three classes: Objective (the
    // objective), WidgetItemHighlight (floor weapons) and Destination, which appears in
    // MULTIPLES simultaneously and is what the field kept reporting as "the enemy markers are
    // still large". Filing it under objective on the strength of the word "Destination" is
    // exactly the naming guess that made navsizeenemy govern an empty set for three rounds.
    if (cn.find(L"Destination")  != std::wstring::npos
     || cn.find(L"Hostile")      != std::wstring::npos
     || cn.find(L"TrackedTarget")!= std::wstring::npos
     || cn.find(L"GhostTarget")  != std::wstring::npos
     || cn.find(L"Enemy")        != std::wstring::npos) return NAVW_ENEMY;
    if (cn.find(L"Item")         != std::wstring::npos
     || cn.find(L"Highlight")    != std::wstring::npos
     || cn.find(L"Weapon")       != std::wstring::npos) return NAVW_ITEM;
    return NAVW_OTHER;
}

const char* navw_kind_name(NavwKind k) {
    switch (k) {
    case NAVW_OBJECTIVE: return "objective";
    case NAVW_ALLY:      return "ally";
    case NAVW_ENEMY:     return "enemy";
    case NAVW_ITEM:      return "item";
    default:             return "other";
    }
}

// PER-CLASS SIZE OVERRIDE, by name -- "WBP_NavpointDestination_C:0.3,WBP_NavpointRecon_C:0.4".
//
// Kind classification is a GUESS at this game's taxonomy from class names, and the guesses have
// been wrong: WBP_NavpointDestination_C was filed as an objective (x0.8) when the field reads it
// as an enemy mark. Rather than keep re-guessing which bucket a class belongs in, this addresses
// any class by name. Checked before the kind default; returns <0 when the class is not listed.
float navw_class_override(const std::wstring& cn) {
    const char* s = g_cfg.nav_size_class;
    if (s == nullptr || s[0] == '\0') return -1.0f;
    const std::string narrow_cn = narrow(cn);
    while (*s != '\0') {
        while (*s == ' ' || *s == ',') ++s;
        const char* name = s;
        while (*s != '\0' && *s != ':' && *s != ',') ++s;
        if (*s != ':') { while (*s != '\0' && *s != ',') ++s; continue; }
        const size_t nlen = (size_t)(s - name);
        ++s;                                    // past ':'
        const float mult = (float)atof(s);
        while (*s != '\0' && *s != ',') ++s;    // past the value
        if (nlen == 0 || !(mult > 0.0f)) continue;
        // Substring match, so either the bare name or the _C form works.
        if (narrow_cn.find(std::string(name, nlen)) != std::string::npos) return mult;
    }
    return -1.0f;
}

// Per-kind size multiplier over nav_world_scale.
float navw_kind_size(NavwKind k) {
    switch (k) {
    case NAVW_OBJECTIVE: return g_cfg.nav_size_obj;
    case NAVW_ALLY:      return g_cfg.nav_size_ally;
    case NAVW_ENEMY:     return g_cfg.nav_size_enemy;
    case NAVW_ITEM:      return g_cfg.nav_size_item;
    default:             return g_cfg.nav_size_other;
    }
}

// Collect a widget's Image children (in tree order). The FIRST image is not always the icon --
// a navpoint layout can carry an on-screen icon AND an off-screen arrow, and picking blind is
// how the objective marker came back gold when its class changed. navworldimg selects which.
int navw_collect_images(API::UObject* owner, API::UObject** out, int cap) {
    if (owner == nullptr || cap <= 0) return 0;
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return 0;

    // ONE pass over the object array, not one per container. The recursive version re-walked the
    // whole array for the root AND AGAIN for every WidgetTree/Panel/Overlay/Box child it met
    // (depth<=2), so one re-host cost 3+ full walks -- the multi-hundred-ms game-thread stall
    // class behind the v0.3.1 freeze reports (docs\Perf\V031-EPISODIC-WORKLOADS-2026-08-19.md).
    // Here every object is tested once by hopping <=3 outers toward `owner`; a navpoint widget's
    // subtree is a handful of nodes, so the DFS below runs over a tiny local list.
    struct Node { API::UObject* obj; API::UObject* outer; };
    Node nodes[64];
    int n_nodes = 0;

    const int32_t n = arr->get_object_count();
    for (int32_t i = 0; i < n && n_nodes < 64; ++i) {
        auto* o = arr->get_object(i);
        if (o == nullptr) continue;
        API::UObject* p = o->get_outer();
        for (int hop = 0; hop < 3 && p != nullptr; ++hop) {
            if (p == owner) { nodes[n_nodes++] = { o, o->get_outer() }; break; }
            p = p->get_outer();
        }
    }

    // The SAME selection the recursive version made -- array order within one parent, containers
    // recursed inline, Images collected at up to depth 2 -- replayed over the local list. The
    // ORDER is load-bearing: navworldimg indexes into it, so a reordering would silently re-pick
    // every player's icon art.
    struct Dfs {
        const Node* nodes;
        int         n_nodes;
        static bool is_container(const std::wstring& cn) {
            return cn == L"WidgetTree" || cn.find(L"Panel") != std::wstring::npos
                || cn.find(L"Overlay") != std::wstring::npos
                || cn.find(L"Box") != std::wstring::npos;
        }
        int walk(API::UObject* parent, API::UObject** out, int cap, int depth) const {
            if (cap <= 0) return 0;
            int got = 0;
            for (int i = 0; i < n_nodes && got < cap; ++i) {
                if (nodes[i].outer != parent) continue;
                auto* o = nodes[i].obj;
                const std::wstring cn = class_name_of(o);
                if (cn.find(L"Image") != std::wstring::npos) { out[got++] = o; continue; }
                if (depth < 2 && is_container(cn))
                    got += walk(o, out + got, cap - got, depth + 1);
            }
            return got;
        }
    };
    const Dfs dfs{ nodes, n_nodes };
    return dfs.walk(owner, out, cap, 0);
}

// The array-free lane: walk the widget's OWN subtree through reflection --
// WidgetTree -> RootWidget -> Slots[] -> Content, depth-first. A few dozen property reads
// instead of a ~300k-object array pass, so a host event stops costing a visible frame.
//
// NOT interchangeable with the array lane by construction: UMG outers every tree widget FLAT
// to the WidgetTree object, so the array lane's "tree order" is really CREATION order, while
// this lane yields SLOT-HIERARCHY order. navworldimg indexes into the result, so if the two
// orders ever disagree for a class, trusting this lane would silently re-pick that marker's
// art on every player's machine. Hence the hybrid below: this lane is only used for a class
// after one live host has PROVEN both lanes return the identical sequence.
int navw_collect_images_tree(API::UObject* owner, API::UObject** out, int cap,
                             const char** why = nullptr) {
    const char* why_local = "ok";
    if (why == nullptr) why = &why_local;
    *why = "ok";
    if (owner == nullptr || cap <= 0) { *why = "no owner"; return 0; }
    auto* tree_pp = owner->get_property_data<API::UObject*>(L"WidgetTree");
    if (tree_pp == nullptr) { *why = "no WidgetTree prop"; return 0; }
    if (*tree_pp == nullptr || IsBadReadPtr(*tree_pp, 0x30)) { *why = "WidgetTree null"; return 0; }
    auto* root_pp = (*tree_pp)->get_property_data<API::UObject*>(L"RootWidget");
    if (root_pp == nullptr) { *why = "no RootWidget prop"; return 0; }
    if (*root_pp == nullptr || IsBadReadPtr(*root_pp, 0x30)) { *why = "RootWidget null"; return 0; }
    struct Walk {
        static int rec(API::UObject* w, API::UObject** out, int cap, int depth) {
            // Depth bounds runaway recursion only -- it must clear a real HUD widget's nesting.
            // This game's objective navpoint puts its icons SEVEN containers down (SizeBox ->
            // Overlay -> Overlay -> ScaleBox -> NamedSlot -> Overlay -> Border -> Image); a cap
            // of 6 returned zero images and read as a lane mismatch. 12 clears that with margin.
            if (w == nullptr || cap <= 0 || depth > 12) return 0;
            const std::wstring cn = class_name_of(w);
            if (cn.find(L"Image") != std::wstring::npos) { out[0] = w; return 1; }
            // Any widget with a Slots array is a panel; everything else (leaf widgets, nested
            // user widgets -- which the array lane also does not enter) ends the branch.
            struct FRawArr { void* data; int32_t num; int32_t max; };
            auto* slots = w->get_property_data<FRawArr>(L"Slots");
            if (slots == nullptr || slots->data == nullptr || slots->num <= 0 || slots->num > 64) return 0;
            int got = 0;
            auto** elems = reinterpret_cast<API::UObject**>(slots->data);
            for (int i = 0; i < slots->num && got < cap; ++i) {
                auto* slot = elems[i];
                if (slot == nullptr || IsBadReadPtr(slot, 0x30)) continue;
                auto* content_pp = slot->get_property_data<API::UObject*>(L"Content");
                if (content_pp == nullptr || *content_pp == nullptr || IsBadReadPtr(*content_pp, 0x30)) continue;
                got += rec(*content_pp, out + got, cap - got, depth + 1);
            }
            return got;
        }
    };
    const int n = Walk::rec(*root_pp, out, cap, 0);
    if (n == 0) *why = "walk found no images";
    return n;
}

// VERIFY-THEN-TRUST dispatch between the two collect lanes, per widget class.
//
// First host of a class runs BOTH lanes, serves the array result (ground truth for order), and
// certifies the tree lane only if the sequences match element-for-element. Every later host of
// a certified class takes the tree lane -- so the ~10 ms array pass is paid at most ONCE per
// class per session, and a marker kind whose orders disagree stays on the array lane forever
// rather than silently re-picking its art. A certified lane that later returns nothing (a
// future engine's UMG moving the Slots layout, say) demotes itself back to the array walk out
// loud instead of hosting a blank quad. navwtree=0 forces the array lane everywhere -- the
// live A/B, and the drill that proves the fallback still works (fallbacks that never run rot).
int navw_collect_images_hybrid(API::UClass* cls, API::UObject* w, API::UObject** out, int cap,
                               const char** lane) {
    *lane = "array";
    if (!g_cfg.navw_tree || cls == nullptr) return navw_collect_images(w, out, cap);

    // 16 slots: this game ships ~11 navpoint widget classes (incl. image-less bases, which the
    // pre-certify sweep also feeds through here); at 8 the cache filled with bases and the two
    // classes that actually host every mission fell out, re-verifying on every host.
    struct Cache { wchar_t cls[96]; int8_t tree_ok; };
    static Cache s_cache[16] = {};
    static int   s_cache_n = 0;

    wchar_t cn[96] = {};
    if (const auto* fn = cls->get_fname()) {
        const std::wstring s = fn->to_string();
        wcsncpy_s(cn, s.c_str(), 95);
    }
    if (cn[0] == L'\0') return navw_collect_images(w, out, cap);

    for (int i = 0; i < s_cache_n; ++i) {
        if (wcscmp(s_cache[i].cls, cn) != 0) continue;
        if (!s_cache[i].tree_ok) return navw_collect_images(w, out, cap);
        const int n = navw_collect_images_tree(w, out, cap);
        if (n > 0) { *lane = "tree"; return n; }
        s_cache[i].tree_ok = 0;
        API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: tree lane went EMPTY for %s - demoted "
                             "back to the array walk", narrow(cn).c_str());
        *lane = "array(demoted)";
        return navw_collect_images(w, out, cap);
    }

    // First sight of this class: certify. Order must match EXACTLY, not just the counts --
    // same count with swapped elements is precisely the silent re-pick this exists to prevent.
    const int n_arr = navw_collect_images(w, out, cap);
    API::UObject* t[8] = {};
    const char* why = "ok";
    const int n_tree = navw_collect_images_tree(w, t, (cap < 8) ? cap : 8, &why);
    bool same = (n_tree == n_arr) && (n_arr > 0);
    for (int i = 0; same && i < n_arr; ++i) same = (t[i] == out[i]);
    if (s_cache_n < 16) {
        wcscpy_s(s_cache[s_cache_n].cls, cn);
        s_cache[s_cache_n].tree_ok = same ? 1 : 0;
        ++s_cache_n;
    }
    // "no images" is its own verdict, not a mismatch: the image-less navpoint BASE classes land
    // here, and calling them MISMATCH would read as a tree-lane defect in field logs.
    const char* verdict = same ? "VERIFIED" : ((n_arr == 0 && n_tree == 0) ? "no images" : "MISMATCH");
    API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: tree lane %s for %s (arr=%d tree=%d why=%s)%s",
                         verdict, narrow(cn).c_str(), n_arr, n_tree, why,
                         same ? " - later hosts of this class skip the array walk"
                              : " - this class stays on the array walk");
    *lane = same ? "array(certifying)" : "array(mismatch)";
    return n_arr;
}

// PRE-CERTIFY the tree lane at mission entry, so the ~10 ms per-class certification (the array
// walk the verify needs) lands inside the load fade instead of on the first marker of its kind
// mid-combat. One shot per session: collect every loaded navpoint widget class in one sweep,
// create a throwaway instance of each, and run it through the same verify-then-trust dispatch a
// real host uses -- the cache it fills IS the host path's cache. Classes a later mission streams
// in are not covered (they certify organically, ~10 ms once). Runs on the first tick that has a
// player controller, which is still inside the load/black window (the controller flips to the
// mission class before the scene is visible -- the same fact Enter-Mission's polling relies on).
// ONE UNIT OF WORK PER TICK -- the sweep on one tick, then ONE class certified per tick after it.
// The first version did all of them in a single tick and measured 217 ms: fine if it lands under
// the load fade, a hard hitch if the fade has already gone. Paced, the worst frame is one class
// (~15-20 ms) and the whole pass still finishes inside the first second of a mission, long before
// a marker can appear. This is the shipping path, so it obeys the never-stall rule the same way
// the rest of update() does.
void navw_precertify_tick() {
    static int          s_state = 0;          // 0 = need the sweep, 1 = certifying, 2 = done
    static API::UClass* s_found[16] = {};
    static int          s_n_found = 0, s_next = 0, s_certified = 0;
    static uint32_t     s_last_tick = ~0u;
    static double       s_total_ms = 0.0;

    if (s_state == 2) return;
    if (!g_cfg.nav_world || !g_cfg.navw_tree || !g_cfg.nav_world_icon) { s_state = 2; return; }
    auto* pc0 = API::get()->get_player_controller(0);
    if (pc0 == nullptr) return;                       // not in a mission yet -- try next tick

    const uint32_t now_tick = g_ticks.load(std::memory_order_relaxed);
    if (now_tick == s_last_tick) return;              // at most one unit of work per tick
    s_last_tick = now_tick;

    LARGE_INTEGER f, t0, t1;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&t0);

    if (s_state == 1) {
        // ---- certify exactly ONE class, then yield the frame.
        auto* wbl_cls = API::get()->find_uobject<API::UClass>(L"Class /Script/UMG.WidgetBlueprintLibrary");
        auto* wbl = (wbl_cls != nullptr) ? wbl_cls->get_class_default_object() : nullptr;
        if (wbl != nullptr && s_next < s_n_found) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            *reinterpret_cast<void**>(p)      = pc0;
            *reinterpret_cast<void**>(p + 8)  = s_found[s_next];
            *reinterpret_cast<void**>(p + 16) = pc0;
            wbl->call_function(L"Create", p);
            if (auto* w = *reinterpret_cast<API::UObject**>(p + 24)) {
                API::UObject* imgs[8] = {};
                const char* lane = "";
                navw_collect_images_hybrid(s_found[s_next], w, imgs, 8, &lane);
                ++s_certified;
            }
        }
        ++s_next;
        QueryPerformanceCounter(&t1);
        s_total_ms += (double)(t1.QuadPart - t0.QuadPart) * 1000.0 / (double)f.QuadPart;
        if (s_next >= s_n_found || wbl == nullptr) {
            s_state = 2;
            API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: pre-certified %d/%d navpoint class(es) "
                                 "at mission entry in %.1f ms total, one per tick - certified kinds "
                                 "host via the tree lane from their first marker",
                                 s_certified, s_n_found, s_total_ms);
        }
        return;
    }

    // One sweep for loaded navpoint widget classes -- WidgetBlueprintGeneratedClass objects whose
    // NAME carries the navpoint taxonomy. Pointers are used within this same tick only. 16 slots:
    // this game ships ~11 such classes including the image-less bases, and a cap of 8 crowded out
    // the two kinds that actually host every mission (measured 2026-08-19).
    // ---- s_state == 0: the class-collection sweep, alone on this tick.
    // The throwaway instances the certify step creates are never viewport-added or hosted;
    // unreferenced, they go with the next GC pass, same as a rejected host candidate.
    if (auto* arr = API::get()->get_uobject_array()) {
        const int32_t n = arr->get_object_count();
        for (int32_t i = 0; i < n && s_n_found < 16; ++i) {
            auto* o = arr->get_object(i);
            if (o == nullptr) continue;
            if (class_name_of(o).find(L"WidgetBlueprintGeneratedClass") == std::wstring::npos) continue;
            const auto* fn = o->get_fname();
            if (fn == nullptr) continue;
            const std::wstring nm = fn->to_string();
            if (nm.find(L"Navpoint") == std::wstring::npos
                && nm.find(L"TrackedTarget") == std::wstring::npos) continue;
            if (nm.size() < 2 || nm.compare(nm.size() - 2, 2, L"_C") != 0) continue;
            s_found[s_n_found++] = reinterpret_cast<API::UClass*>(o);
        }
    }
    QueryPerformanceCounter(&t1);
    s_total_ms += (double)(t1.QuadPart - t0.QuadPart) * 1000.0 / (double)f.QuadPart;
    s_state = (s_n_found > 0) ? 1 : 2;
    if (s_state == 2) {
        API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: pre-certify found no navpoint widget "
                             "classes loaded - marker kinds will certify as they first appear");
    }
}
bool          g_navw_shown = false;         // anything visible last tick (drives one-shot hide)
std::atomic<float> g_navw_keff{1000.0f};    // measured px-per-tan (independent of navfix's)

// Outermost-package test: live widgets are outered under /Engine/Transient; the blueprint
// ARCHETYPE (which also matches the class-name scan) is outered under /Game/... and writes to
// it do nothing. The scan cannot tell them apart; this can.
bool navw_is_live_widget(API::UObject* o) {
    API::UObject* top = o;
    while (top != nullptr && top->get_outer() != nullptr) top = top->get_outer();
    if (top == nullptr) return false;
    const auto* fn = top->get_fname();
    if (fn == nullptr) return false;
    const std::wstring n = fn->to_string();
    return n.find(L"Transient") != std::wstring::npos;
}

// HOST THE NAVPOINT ART on a marker quad, re-hosting when the slot changes navpoint TYPE.
//
// `want_class` is the navpoint's OWN widget class, taken from its map entry -- so a co-op
// partner marker wears the partner art, a tracked enemy wears the enemy art, and each keeps the
// colour that carries its meaning. nullptr falls back to navworldclass, then to the objective
// widget. Returns true when a widget is on the quad.
//
// HOST THE ICON, NOT THE WHOLE WIDGET. A navpoint widget is a HUD LAYOUT (icon plus a
// distance/label block, authored for a canvas); hosted whole on a square quad it renders a
// window onto that layout -- the "same text, cropped" seen in the field three times. Sizing from
// its desired size cannot fix that either: a widget that never enters a viewport never lays out
// and reports desired=(0,0) (measured). So the IMAGE child is hosted instead, with the whole
// widget as the fallback and navworldicon=0 to force it.
bool navw_host_class(API::UObject* comp, int slot, API::UClass* want_class) {
    if (comp == nullptr) return false;

    API::UClass* nav_wcls = want_class;
    if (nav_wcls == nullptr && g_cfg.nav_world_class[0] != '\0') {
        const std::string a{g_cfg.nav_world_class};
        const std::wstring w(a.begin(), a.end());
        nav_wcls = API::get()->find_uobject<API::UClass>(w.c_str());
        if (nav_wcls == nullptr) {
            // Bare name: sweep for a widget class whose name matches. NEGATIVE RESULTS ARE
            // MEMOISED: an unresolvable name (typo, or a patch renamed the class) used to re-pay
            // this full-array sweep on EVERY re-host, forever -- the same unthrottled
            // sweep-on-miss hazard flagged for find_ui_manager. A miss is cached against the
            // configured string and re-armed every 600 ticks (~20 s), so a level that loads the
            // class later still gets found; a live edit of navworldclass re-arms immediately.
            // A HIT is deliberately NOT cached: the pointer would dangle across GC/level
            // changes, and a successful sweep only happens on a re-host, which is rare.
            static char     s_miss_for[sizeof(g_cfg.nav_world_class)] = {0};
            static uint32_t s_miss_tick = 0;
            const uint32_t  now_tick = g_ticks.load(std::memory_order_relaxed);
            const bool cached_miss = (strcmp(s_miss_for, g_cfg.nav_world_class) == 0)
                                     && (now_tick - s_miss_tick < 600);
            if (!cached_miss) {
                if (auto* arr = API::get()->get_uobject_array()) {
                    const int32_t n = arr->get_object_count();
                    for (int32_t k = 0; k < n; ++k) {
                        auto* o = arr->get_object(k);
                        if (o == nullptr) continue;
                        if (class_name_of(o).find(L"WidgetBlueprintGeneratedClass") == std::wstring::npos)
                            continue;
                        const auto* fn = o->get_fname();
                        if (fn != nullptr && fn->to_string().find(w) != std::wstring::npos) {
                            nav_wcls = reinterpret_cast<API::UClass*>(o);
                            break;
                        }
                    }
                }
                if (nav_wcls == nullptr) {
                    strcpy_s(s_miss_for, g_cfg.nav_world_class);
                    s_miss_tick = now_tick;
                }
            }
        }
    }
    if (nav_wcls == nullptr) {
        nav_wcls = API::get()->find_uobject<API::UClass>(
            L"WidgetBlueprintGeneratedClass /Game/UI/Hud/Navpoints/ScriptedNavpoints/"
            L"WBP_NavpointObjective.WBP_NavpointObjective_C");
    }
    if (nav_wcls == nullptr) return false;
    if (slot >= 0 && slot < 8 && g_navw_slot_class[slot] == (void*)nav_wcls) return true;  // already wearing it

    // AT MOST ONE EXPENSIVE RE-HOST PER TICK. A composition change (objective update, slots
    // re-typed) can re-class several slots in the same tick, and paying widget-Create + subtree
    // collection for all of them at once is what turned those updates into a single long stall.
    // Returning false leaves this quad empty for one tick; the placement loop asks again next
    // tick (the already-wearing fast path above keeps settled slots free), so markers pop in
    // over consecutive ~30 ms ticks instead of freezing the frame. Checked BEFORE the perf scope
    // so a deferred call does not record a near-zero sample and dilute the re-host mean.
    {
        static uint32_t s_rehost_tick = ~0u;
        const uint32_t now_tick = g_ticks.load(std::memory_order_relaxed);
        if (s_rehost_tick == now_tick) return false;
        s_rehost_tick = now_tick;
    }

    // Everything below is the expensive path (widget Create + image-child collection); the
    // fast path above runs every tick and must stay untimed or `n` stops meaning "re-hosts".
    PerfScope _perf(PERF_NAVHOST);

    auto* wbl_cls = API::get()->find_uobject<API::UClass>(L"Class /Script/UMG.WidgetBlueprintLibrary");
    auto* wbl = (wbl_cls != nullptr) ? wbl_cls->get_class_default_object() : nullptr;
    auto* pc0 = API::get()->get_player_controller(0);
    if (wbl == nullptr || pc0 == nullptr) return false;

    API::UObject* w = nullptr;
    {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<void**>(p) = pc0;          // WorldContextObject
        *reinterpret_cast<void**>(p + 8) = nav_wcls; // WidgetType
        *reinterpret_cast<void**>(p + 16) = pc0;     // OwningPlayer
        wbl->call_function(L"Create", p);
        w = *reinterpret_cast<API::UObject**>(p + 24);
    }
    if (w == nullptr) return false;

    API::UObject* host = w;
    const char* which = "whole widget";
    const char* lane = "off";
    int n_imgs = 0;
    if (g_cfg.nav_world_icon) {
        API::UObject* imgs[8] = {};
        n_imgs = navw_collect_images_hybrid(nav_wcls, w, imgs, 8, &lane);
        if (n_imgs > 0) {
            const int pick = (g_cfg.nav_world_img >= 0 && g_cfg.nav_world_img < n_imgs)
                           ? g_cfg.nav_world_img : 0;
            host = imgs[pick];
            which = "image child";
        }
    }
    { alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
      *reinterpret_cast<void**>(q) = host;
      comp->call_function(L"SetWidget", q); }

    const double px = (g_cfg.nav_world_draw > 0.0f) ? (double)g_cfg.nav_world_draw : 128.0;
    { alignas(16) uint8_t sz[RIG_PARAM_BUF] = {0};
      auto* sd = reinterpret_cast<double*>(sz);
      sd[0] = px; sd[1] = px;
      comp->call_function(L"SetDrawSize", sz); }

    // NAME THE NAVPOINT CLASS, not just the image class. The previous log said only
    // "hosting HaloUIImage", which is true of every marker and therefore identifies nothing --
    // when the objective marker came back gold there was no way to tell WHICH art it had picked
    // up. The class name is also what the kind classification (and per-kind sizing) reads.
    ++g_navw_census.rehost;   // a re-host actually completed -- the census counts the expensive path
    const std::wstring wcn = class_name_of(w);
    const NavwKind kind = navw_classify(wcn);
    const float ov = navw_class_override(wcn);
    const float mult = (ov > 0.0f) ? ov : navw_kind_size(kind);
    if (slot >= 0 && slot < 8) {
        g_navw_slot_class[slot] = (void*)nav_wcls;
        g_navw_slot_kind[slot]  = kind;
        g_navw_slot_size[slot]  = mult;
        // ---- DESTROY THE STALE PIXELS. Do not try to out-time them. --------------------------
        //
        // We just swapped this slot's hosted widget, and the component's render target still holds
        // the PREVIOUS navpoint's art until the widget redraws. Everything downstream then has to
        // answer "has it redrawn yet?", and the only tool it had was tick arithmetic:
        // slot_cell_coherent() asks whether the capture happened after the re-host. THAT IS NOT THE
        // SAME QUESTION. A capture can land after the re-host and still copy the old pixels,
        // because the redraw is the engine's to schedule, not ours -- which is why cohdrawn
        // measured 12-16 wrong-art appends per 10k entries examined no matter what identity scheme
        // was in use (2026-09-03: position-keyed 12.0, index-keyed 16.0, both with the guard on).
        //
        // So do not race the redraw: CLEAR THE RENDER TARGET to fully transparent. The wrong art
        // then does not exist to be shown. Any capture taken in the gap copies transparency, the
        // compositor quad blends to nothing, and the marker is INVISIBLE for a frame or two rather
        // than WRONG -- which is the trade this lane already says it wants ("fail by hiding, never
        // by freezing"). No margin, no tick threshold, no tuning constant.
        //
        // RequestRedraw() immediately afterwards so the new art arrives at the earliest frame the
        // engine will give it, rather than whenever the widget's own redraw timer next fires.
        //
        // Costs one clear per ACTUAL re-host -- 14 to 60 in a session, not per frame -- and both
        // calls are already proven reachable on this game (Reticule.cpp uses them on its own RT).
        {
            API::UObject* rt = nullptr;
            { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
              comp->call_function(L"GetRenderTarget", p);
              rt = *reinterpret_cast<API::UObject**>(p); }

            auto* pc  = API::get()->get_player_controller(0);
            auto* rcls = API::get()->find_uobject<API::UClass>(
                L"Class /Script/Engine.KismetRenderingLibrary");
            auto* krl = (rcls != nullptr) ? rcls->get_class_default_object() : nullptr;

            if (rt != nullptr && pc != nullptr && krl != nullptr) {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                *reinterpret_cast<void**>(p)     = pc;
                *reinterpret_cast<void**>(p + 8) = rt;
                auto* cc = reinterpret_cast<float*>(p + 16);
                cc[0] = 0.0f; cc[1] = 0.0f; cc[2] = 0.0f; cc[3] = 0.0f;   // fully transparent
                krl->call_function(L"ClearRenderTarget2D", p);
            } else {
                // SAY SO. If the clear cannot run, the coherence heuristic below is all that stands
                // between a re-host and a wrong marker, and that is a materially weaker position --
                // it must not be discovered by someone puzzling over a flicker months from now.
                static uint32_t s_noclear = 0;
                if (s_noclear < 5) {
                    ++s_noclear;
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] NAVWORLD: could not clear slot %d's render target on "
                        "re-host (rt=%p pc=%p krl=%p) -- stale art is now only covered by the "
                        "tick-based coherence check.",
                        slot, (void*)rt, (void*)pc, (void*)krl);
                }
            }
            { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; comp->call_function(L"RequestRedraw", p); }
        }

        // Belt and braces, kept because it is free: the layer still declines to present a cell
        // captured before this re-host. With the clear above it is no longer load-bearing -- the
        // worst it can now hide is a transparent cell -- but cohskip/cohdrawn remain the instrument
        // that says whether any of this is working.
        halo::xrlayer_notice_rehost(halo::XRLAYER_SLOT_NAV_BASE + slot);
    }
    API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: slot %d = %s [kind=%s size x%.2f%s] hosting "
                         "%s (%s, %d image(s), navworldimg=%d, lane=%s) draw=%.0fpx",
                         slot, narrow(wcn).c_str(), navw_kind_name(kind), mult,
                         (ov > 0.0f) ? " BY navsizeclass" : "",
                         narrow(class_name_of(host)).c_str(), which, n_imgs,
                         g_cfg.nav_world_img, lane, px);
    return true;
}

// One pool slot: a small sphere, absolute transform, no collision, no shadow -- the
// reticule_mesh_ensure recipe with the parts this lane needs.
//
// OWNER: the FP WEAPON actor (the rig component's outer), exactly like the reticule mesh --
// NOT the pawn. v3 parented these to the pawn and the field result was "3 markers placed,
// none visible": this title hides the first-person pawn's components from the player's own
// view, so a pawn-owned marker exists, positions, and never renders for the person wearing
// the headset. The weapon actor is the one actor PROVEN to render owner-visible components
// here. It dies in vehicles, which is fine -- navworld hides in stick mode anyway, and the
// TrackedObject pool re-creates on the next foot segment.
//
// `want_class` is the navpoint class this slot is ABOUT to show, handed in by the caller. Passing
// it matters (fixed 2026-08-25): this function used to host nullptr, which navw_host_class resolves
// to the DEFAULT objective widget -- so every freshly created slot was dressed as an objective and
// then re-hosted with its real art on the very next tick. In the 10:25-10:27 log that is visible as
// each of slots 0/2/3/4/5 hosting TWICE, ~45 ms apart, objective then item-highlight, every single
// time a slot was created. It cost a wasted UMG widget Create, a wasted image-child collection (a
// full object-array walk for any class the tree lane has not certified, ~10 ms), and a spurious
// xrlayer_notice_rehost that pulled the marker off the compositor for a frame. None of that was
// visible to the player after the visibility gate landed, which is precisely why it needed the log
// to find. The three callers that genuinely have no class to offer still pass nullptr and get the
// old fallback.
API::UObject* navw_ensure_slot(int i, API::UClass* want_class = nullptr) {
    // FAST PATH OUTSIDE THE PERF SCOPE, deliberately: this runs for every live slot every tick, and
    // timing it would both add QPC pairs to the steady state and drown the creation cost -- the one
    // number this site exists to report -- in a mean of near-zero samples. Same rule as PERF_NAVHOST.
    if (auto* c = g_navw_pool[i].get_checked(L"WidgetComponent")) return c;

    // AT MOST ONE SLOT CREATED PER TICK, for the same reason navw_host_class re-hosts at most one:
    // a composition change can bring several navpoints into view on the same tick, and creating
    // eight widget quads in one frame is a hitch even when each one is cheap. It is also WASTED
    // work -- navw_host_class's own one-per-tick throttle means only the first of them could get
    // its art anyway, and the rest would register widget-less (degenerate) quads to be re-hosted
    // next tick regardless. Returning nullptr here is the path the caller already handles: the
    // slot keeps its stable identity and is retried on the next tick, so markers pop in over
    // consecutive ticks instead of arriving together in one long frame. Checked BEFORE the perf
    // scope so a deferred call cannot dilute navw_newslot's mean with a near-zero sample.
    {
        static uint32_t s_create_tick = ~0u;
        const uint32_t now_tick = g_ticks.load(std::memory_order_relaxed);
        if (s_create_tick == now_tick) return nullptr;
        s_create_tick = now_tick;
    }

    PerfScope _perf(PERF_NAVSLOT);
    g_navw_mid_ok[i] = false;

    auto* rig = reinterpret_cast<API::UObject*>(g_rig_component.load());
    auto* owner = (rig != nullptr) ? rig->get_outer() : nullptr;
    if (owner == nullptr) return nullptr;   // no FP weapon this tick: try again next tick

    // THE SHARED WIDGET-QUAD RECIPE (Reticule.cpp). Markers are the same kind of object as the
    // reticule -- a world-space WidgetComponent built by the OblivionVR deferred sequence, the
    // only world-space visual proven to render on this title. That sequence, its measured
    // BlendMode offset and the exposure-compensated material chain live in ONE place; this call
    // site supplies only what is navpoint-specific (which widget, how big).
    bool compensated = false;
    auto* comp = widget_quad_begin(owner, /*blend=*/2, &compensated);
    if (comp == nullptr) return nullptr;
    g_navw_compensated = compensated;
    g_navw_mid_ok[i] = true;

    // THE HOSTED WIDGET -- not optional. A WidgetComponent with no widget builds a DEGENERATE
    // quad (CurrentDrawSize 0,0) and renders nothing, which is how the background-fill-only
    // version of this marker stayed invisible. navw_host_class() supplies the art: the caller's
    // class when it has one (so the slot is born wearing the RIGHT art -- see the note above),
    // nullptr taking the configured/default class as before. The placement loop still re-hosts
    // per navpoint type as slots are genuinely reused; what it no longer does is re-host a slot
    // one tick after creating it.
    const bool hosted = navw_host_class(comp, i, want_class);

    widget_quad_finish(owner, comp, /*bounds_scale=*/10.0f);

    g_navw_pool[i].set(comp);
    API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: marker slot %d created as WIDGET quad on %s "
                         "(mic=%d hosted=%d)", i, narrow(class_name_of(owner)).c_str(),
                         (int)g_navw_mid_ok[i], (int)hosted);
    return comp;
}

// Is the compositor lane in charge of the markers right now?
//
// xrlayer_live() and not merely "the key is on": the layer must be PROVEN reaching the compositor.
// The attachment resolves UEVR's xrEndFrame out of UEVRBackend.pdb, which no player has, so on a
// player install this is false and the in-scene lane below runs exactly as it always did. That is
// not a temporary state to be tidied away later -- the in-scene lane is the SHIPPING path and must
// not be degraded to make this one look better.
bool navw_layer_owns() {
    // ATTACHED, NOT LIVE -- see xrlayer_attached() in XrLayer.hpp for the measurement.
    //
    // With live() this predicate was a self-latch: g_live counts quads submitted across ALL slots,
    // so it stayed true while any marker drew, then went false when the last one stopped -- at
    // which point this returned false, the lane retired its quads, and nothing it controlled could
    // ever set live again. Ownership must not be decided by a signal that its own output feeds.
    //
    // The in-scene HIDE still keys on live() (see reticule_widget_set_scene_hidden's caller), and
    // that asymmetry is deliberate: hiding a fallback is only safe while the layer is actually
    // drawing, whereas owning a lane is a question about configuration and attachment.
    return g_cfg.xr_layer && g_cfg.xr_layer_nav && halo::xrlayer_attached();
}

// HIDE ONE MARKER FROM THE SCENE WITHOUT STOPPING IT RENDERING.
//
// SetVisibility(false) freezes the compositor layer -- measured in a headset twice for the
// reticule (reticule_widget_set_scene_hidden documents it): the widget stops redrawing its render
// target once the engine stops rendering the component, so the layer holds whatever frame it last
// drew. TickWhenOffscreen is NOT the gate and believing it was cost a build.
//
// So while the compositor owns a marker, the component stays fully visible and RENDERED and its
// tint alpha goes to zero. "Hidden" for the player becomes "not appended to the frame", which is
// XrLayer's business, not the component's. The alpha is written by the same per-tick tint write
// that already re-asserts itself when the component rebuilds its material behind our back -- a
// one-shot write here would silently revert exactly as the reticule's tint did.
void navw_set_alpha_hidden(API::UObject* c, bool hidden) {
    if (c == nullptr) return;
    const float gain = g_navw_compensated ? 1.0f : g_cfg.aim_widget_gain * g_cfg.aim_widget_tint;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    auto* col = reinterpret_cast<float*>(p);
    col[0] = gain * g_cfg.nav_world_cr;
    col[1] = gain * g_cfg.nav_world_cg;
    col[2] = gain * g_cfg.nav_world_cb;
    col[3] = hidden ? 0.0f : 1.0f;
    c->call_function(L"SetTintColorAndOpacity", p);
}

void navw_hide_all() {

    for (int i = 0; i < 8; ++i) {
        // Retire the compositor quad and stop resolving the component's texture FIRST, so nothing
        // is left being submitted against a component we are about to stop driving.
        //
        // VISIBILITY, not alpha, is right here even under the compositor lane -- and the
        // distinction is worth stating because it looks like a contradiction of
        // navw_set_alpha_hidden above. This is the WHOLE LANE disengaging (stick mode, a cutscene,
        // the kill switch): no quad is being submitted, so there is no layer left to freeze, and
        // alpha-hiding would keep eight widget quads rendering for the length of a cutscene to
        // present art nobody is looking at. The alpha path exists for the other case -- a marker
        // that is momentarily unused while the lane is still running.
        halo::xrlayer_retire_quad(halo::XRLAYER_SLOT_NAV_BASE + i);
        halo::xrsource_set_slot_component(halo::XRLAYER_SLOT_NAV_BASE + i, nullptr, 0);

        // MARKED, because this whole function was a BLIND SPOT and the fault reports said so.
        // Every report from the 2026-09-08 playthrough read `step '-'` -- the marker was null,
        // meaning the fault beat the first marked call in the tick. The marks all live from the
        // projection onward, so "before the first mark" is exactly this gate-disengage path plus
        // the entry, and neither had one. A blank breadcrumb is not "no information", it is a gap
        // in the trail, and reading it as "not in a marked call" is only useful once the calls
        // that COULD fault are all marked.
        NAVW_MARK("hide_all:get_checked");
        auto* c = g_navw_pool[i].get_checked(L"WidgetComponent");
        if (c == nullptr) continue;
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        p[0] = 0;
        // The prime suspect. get_checked proves the array slot still holds this pointer and that
        // the class still matches -- neither of which proves the component is not MID-TEARDOWN,
        // which is the state that would have SetVisibility dereference freed internals.
        NAVW_MARK("hide_all:SetVisibility");
        c->call_function(L"SetVisibility", p);
    }
    // Free every stable slot: the whole lane is down, so the next engage re-assigns from scratch.
    for (int i = 0; i < 8; ++i) { g_navw_slot_ident[i] = 0; g_navw_slot_prio[i] = 99; }
    g_navw_placed_mask.store(0);
}

// BUG 2 (phantom markers): draw only navpoints the GAME's own HUD is showing.
//
// The flat HUD gates each navpoint on its widget's Slate visibility; the compositor lane, reading
// straight from the manager map, drew every entry that had a valid world position -- including
// ones the game had collapsed, which surfaced as objective/item markers floating over walls while
// the live enemy marks rendered correctly. Read the entry's live widget (element+0x08) and honour
// its reflected Visibility.
//
// Returns true = SHOW, and FAILS OPEN: an unreadable widget, a class that is not a widget, or a
// missing Visibility property all return true. So the gate can only ever SUPPRESS a marker it can
// positively prove the game hid -- it can never blank a marker on a layout it did not understand,
// which is the fail-closed direction that would cost the player a real waypoint.
//
// ESlateVisibility: 0 Visible, 1 Collapsed, 2 Hidden, 3 HitTestInvisible, 4 SelfHitTestInvisible.
// Collapsed and Hidden are the game saying "do not show this navpoint".
//
// ⚠️ MEASURED 2026-08-25, AND IT IS NOT DOING THE JOB IT WAS ADDED FOR. This gate DOES run -- the
// always-compiled summary line in lane 2 printed `N entr(ies) resolved, N shown` on all 51 of its
// samples across the 13:54-14:08 session, and resolved == shown on EVERY ONE. So over seven
// minutes of live play, with the gate enabled by default, it suppressed exactly zero entries.
// It is not dead; it is unanimous, which is a different and more misleading failure.
//
// That leaves the hypothesis the previous session wrote down but could not test: the game may hide
// these navpoints by DE-PARENTING them (removing the widget from its panel) rather than by
// collapsing them, in which case `Visibility` on the widget itself stays 0 forever and this gate
// can never fire. `g_navw_census.vis_seen` now accumulates a bitmask of every ESlateVisibility
// value observed and prints it in that same summary line -- if it only ever reads `0x1` (Visible
// and nothing else) the flag is confirmed wrong and the state to key on is elsewhere (the widget's
// parent/slot, or the manager's own per-entry enable). Do NOT widen this gate on a guess; the
// bitmask settles it in one session. `navworldkindmask` remains the labelled STOPGAP, not a fix.
//
// ADDR-HYGIENE: structural -- NAVW_ELEM_WIDGET_OFF is a field offset INTO the game's
// NavpointInstances map element, not a code address. The anatomy dumps that settled +0x10 as the
// element's WidgetBlueprintGeneratedClass and +0x50/+0x78 as its screen/world position place the
// live UUserWidget instance at +0x08. GUARDED at use: the pointer is IsBadReadPtr-checked and only
// trusted when class_name_of resolves to a "Widget" class, and the visibility itself is read by
// REFLECTION (a named UPROPERTY), never by a further raw offset -- a wrong element layout yields
// "no widget" and the gate opens (draws), never a bad read written through.
//
// SECOND CONSUMER (2026-08-25): this same pointer is now the STABLE SLOT IDENTITY. That does not
// weaken the guard, and its failure mode is the mildest of the three: a wrong offset yields "no
// widget", identity falls back to the sparse index (census `identfb` counts it, so the fallback
// cannot rot unobserved), and the lane degrades to the keying it had before rather than
// mis-identifying anything. Nothing is ever WRITTEN through this offset.
constexpr int32_t NAVW_ELEM_WIDGET_OFF = 0x08;

// The entry's LIVE WIDGET, or nullptr when this element does not positively yield one. Split out
// of navw_entry_shown so the same validated pointer serves two jobs -- the visibility gate below
// and the STABLE SLOT IDENTITY -- off ONE read and ONE reflection check per entry, instead of the
// gate reading it and the slot keying guessing at an index.
// A pointer read from a raw element offset that IsBadReadPtr accepted and the object array
// rejected. Counted in the census and said ONCE in full -- the first one is the interesting one,
// and a storm of them is the census's job -- so the guard is observable in a support log rather
// than a silent skip. An unobserved fallback is the thing this project keeps being bitten by.
void navw_note_stale_ptr(const char* where, const void* p) {
    ++g_navw_census.stale;
    static bool s_said = false;
    if (!s_said) {
        s_said = true;
        API::get()->log_info(
            "[Halo-CampE-UEVR] NAVWORLD: REJECTED a readable-but-dead object pointer at %s (%p): "
            "its InternalIndex slot does not hold it. This is the pointer class_name_of would have "
            "handed to UEVR; the 2026-09-08 tick faults (FName::ToString reading 0x40400018) are "
            "that dereference.", where, p);
    }
}

API::UObject* navw_entry_widget(const uint8_t* elem) {
    if (IsBadReadPtr(elem + NAVW_ELEM_WIDGET_OFF, 8)) return nullptr;
    auto* w = *reinterpret_cast<API::UObject* const*>(elem + NAVW_ELEM_WIDGET_OFF);
    if (w == nullptr || IsBadReadPtr(w, 0x30)) return nullptr;
    // READABLE IS NOT ALIVE. The guard the ADDR-HYGIENE note above describes stopped at
    // IsBadReadPtr, and its second step -- class_name_of -- is itself a dereference: it reads
    // this pointer's ClassPrivate and hands THAT to UEVR's FName::ToString. A freed object's
    // block is readable and belongs to whatever was allocated next. The census shows most
    // entries yield no widget here (identfb 79-96%) but never recorded whether those pointers
    // were null or non-object memory; a non-object one reached class_name_of with nothing but
    // a readability check and faults the moment [w+0x10] holds a float. `stale` now counts
    // exactly those. One indexed compare settles it.
    if (!uobject_slot_valid(w)) { navw_note_stale_ptr("elem+0x08", w); return nullptr; }
    NAVW_MARK("entry_widget:class_name_of");
    if (class_name_of(w).find(L"Widget") == std::wstring::npos) return nullptr;
    return w;
}

// LATCHED FOR THE SESSION, and only ever set to true. Once ANY entry has yielded a
// reflection-validated widget at NAVW_ELEM_WIDGET_OFF, the element layout is PROVEN for this
// build -- so from then on a null is information ("this entry has no live widget") rather than
// ignorance ("we cannot read this layout"). Visgate mode 2 below is the only consumer, and this
// latch is what makes it safe: on a build where the offset is wrong nothing ever resolves, the
// latch stays false, and mode 2 degrades silently to the fail-open behaviour of mode 1 instead of
// hiding every marker in the game.
std::atomic<bool> g_navw_widget_off_proven{false};

// Takes the widget navw_entry_widget already validated. nullptr = we could not identify a widget.
bool navw_entry_shown_w(API::UObject* w, int* out_vis, NavwKind kind) {
    if (out_vis != nullptr) *out_vis = -1;
    if (g_cfg.nav_world_visgate == 0) return true;
    if (w == nullptr) {
        // ---- MODE 2: TREAT "NO LIVE WIDGET" AS "NOT SHOWN" ----
        //
        // MEASURED, from the user's own always-compiled census (2026-09-01 session):
        //   visgate=11661/0  -- 11,661 entries examined, ZERO ever suppressed
        //   vis=0x10         -- every entry that DID yield a widget read Visibility=4
        //                       (SelfHitTestInvisible). Never 0, never Collapsed(1), never
        //                       Hidden(2). The flag mode 1 keys on is one the game never sets on
        //                       these widgets, exactly as navw_entry_shown's banner predicted.
        //   identfb=9187     -- 79% of ACCEPTED entries had no readable widget at all.
        //
        // Those 79% are the phantoms. They still resolve a world position (from the element's
        // +0x20/+0x28 chain) and still carry a class at +0x10 -- overwhelmingly
        // WBP_NavpointWidgetItemHighlight_C -- so they are drawn as item/weapon pickup markers
        // standing at stale positions with nothing behind them. That is the reported symptom
        // verbatim: "mostly weapon/item pickup markers just showing up randomly, not pointing at
        // anything in particular."
        //
        // TWO READINGS OF A NULL WIDGET, AND THIS IS CORRECT UNDER BOTH, which is the reason it is
        // worth doing before either has been proved:
        //   (a) SPARSE-ARRAY TOMBSTONE. The sweep walks raw indices 0..scan_hi and never consults
        //       the map's allocation bitmap, so freed elements are read; their stale position and
        //       stale class survive the free while the widget pointer does not. Then the entry is
        //       dead and must not be drawn.
        //   (b) LAZY WIDGET. The game instantiates a widget only for markers it is actually
        //       showing. Then a null means "not currently shown" and must not be drawn either.
        // Under (a) the position is also garbage; under (b) it may be fine. Neither wants a marker.
        //
        // ⚠️ REFUTED IN A HEADSET 2026-09-02, THE SESSION IT WAS ADDED. DO NOT ENABLE THIS.
        //
        // It fired (visgate=6843/3615, so 53% suppressed -- the gate works). It suppressed THE
        // WRONG THINGS. Reported: "I see the screen space obj icon and no xr layer icon for the
        // objective. I do see xr layer icons for floor items." The per-entry rows say why:
        //
        //   s=2 id=..00000005 vis=-1 en=-1 op=-1.00 shown=0 pos=(-28477,-6919,809)   <- OBJECTIVE
        //   s=4 id=..B9B0DCF0 vis=4  en=1  op=1.00  shown=1 pos=(-26392,-7745,5)     <- floor item
        //
        // The OBJECTIVE is the entry with no widget -- persistently, with a stable and entirely
        // plausible elevated position -- while the floor items have real widgets whose POINTERS
        // CHURN between samples at a fixed position. So "no live widget" does not mean "phantom".
        // Both readings this mode was built on (tombstone, lazy widget) are wrong for the objective,
        // and the id column proves the nulls are not garbage: they are the sparse-index fallback
        // (1|(s<<1)) landing exactly where it should.
        //
        // Left in the code rather than deleted because the measurement is worth keeping and the
        // mode is one live key away from being re-tried by someone who has not read this. If you
        // are tempted: the thing that actually distinguishes these populations has not been found
        // yet, and it is not widget presence.
        //
        // DEFAULT IS 1, AND SHOULD STAY 1.
        if (!g_navw_widget_off_proven.load(std::memory_order_relaxed)) return true;   // layout unproven

        // ---- MODE 3: NO WIDGET MEANS THE GAME IS NOT DISPLAYING IT -- EXCEPT THE OBJECTIVE ----
        //
        // CAPTURED LIVE 2026-09-04 with a phantom on the player's screen, which is what finally
        // settled this after five wrong fixes:
        //
        //   real:    cls=..ItemHighlight vis=4  en=1  op=1.00  pos=(-26731,-15308,629)
        //   real:    cls=..ItemHighlight vis=4  en=1  op=1.00  pos=(-26841,-15141,635)
        //   PHANTOM: cls=..ItemHighlight vis=-1 en=-1 op=-1.00 pos=(1,1,225)      <- on screen
        //   PHANTOM: cls=..ItemHighlight vis=-1 en=-1 op=-1.00 pos=(952,0,0)
        //   PHANTOM: cls=..ItemHighlight vis=-1 en=-1 op=-1.00 pos=(3137,0,53396)
        //
        // WIDGET PRESENCE IS THE DISCRIMINATOR, cleanly, on every row. The game builds a widget for
        // a navpoint it is DISPLAYING as a world marker; an item that has lost its widget is one it
        // has stopped displaying, and drawing it is the phantom.
        //
        // These are NOT freed slots -- deadslot was frozen at 10265 across the same samples while
        // visgate climbed, so the allocation bitmap says they are live. Two different populations:
        // the bitmap catches freed entries, this catches live-but-not-displayed ones.
        //
        // THE OBJECTIVE IS EXEMPT because it never has a widget: the game presents the objective in
        // SCREEN SPACE, so our world marker for it is our own addition rather than a mirror of
        // something the game draws. Mode 2 missed that and suppressed the objective, which is why
        // it was refuted the same night it shipped.
        //
        // This also supersedes the nullpos guard, which required a position within 1 m of the world
        // origin on all three axes -- (1,1,225) has z=225 and sailed straight through it. Widget
        // presence is the general form of that test; nullpos was a special case that happened to
        // catch the phantoms whose stale coordinates were small on every axis.
        //
        // Deliberately NOT extended to ally/enemy kinds: nobody has measured whether the game
        // builds widgets for those, and guessing is what cost the previous five attempts.
        // MEASURED 2026-09-04, and it narrows the rule rather than widening a guess. Counting
        // per-entry rows over one session:
        //   enemy      259 rows, vis=-1 on EVERY one, 86 distinct plausible positions
        //   objective   41 rows, vis=-1, 5 distinct plausible positions
        //   other       36 rows, vis=-1 -- and they are WBP_NavpointRecon_C, a REAL class we simply
        //                do not classify, at 22 distinct plausible positions
        //   item        38 rows WITH a widget, 26 WITHOUT -- and the widget-less ones carry the
        //                junk coordinates ((-1305,0,-0), (-122431,0,13213)) that started this
        //
        // So WIDGET PRESENCE DISCRIMINATES PHANTOMS FOR ITEMS AND ONLY ITEMS. Every other kind is
        // widget-less BY DESIGN, because the game presents those in screen space and our world
        // marker is our own addition rather than a mirror of something it draws. Exempting only the
        // objective (the first version of this rule) therefore deleted every enemy marker -- the
        // symptom reported here -- and the code comment beside it said outright that ally/enemy had
        // not been measured. Now they have been.
        //
        // `other` STAYS SUPPRESSED, deliberately. It is the unclassified bucket and the fallback art
        // for it is the OBJECTIVE icon, which is exactly the "extra objective icon" artifact from
        // earlier in this hunt. Recon markers should be given a real kind in navw_classify rather
        // than let in through the catch-all; that is a separate change with a visible result.
        // AN ALLOW-LIST OF KINDS, deliberately, and it is the safer half of a real trade-off.
        //
        // Inverting this to "suppress only items" was tried and reverted the same session. The
        // argument for inverting was that an allow-list silently hides a class nobody has met yet.
        // The argument against, which wins: an entry we cannot classify draws with the OBJECTIVE
        // FALLBACK ART, so letting unknowns through resurrects the "extra objective icon" artifact
        // this hunt already chased once. A census of seven sessions found exactly four classes and
        // all four are now classified, so the allow-list is complete rather than hopeful.
        //
        // The silent-hiding risk is answered by TELEMETRY instead of by policy -- see the
        // unclassified-suppression log at the call site. A fifth class announces itself by name the
        // first time it is seen, which is what the inversion was really protecting against.
        if (g_cfg.nav_world_visgate >= 3) {
            return (kind == NAVW_OBJECTIVE) || (kind == NAVW_ENEMY) || (kind == NAVW_ALLY);
        }
        if (g_cfg.nav_world_visgate >= 2) return false;
        return true;                                                           // no widget: open
    }
    auto* vis = w->get_property_data<uint8_t>(L"Visibility");
    if (vis == nullptr) return true;                                           // no such property: open
    if (out_vis != nullptr) *out_vis = (int)*vis;
    if (*vis < 32) g_navw_census.vis_seen |= (1u << *vis);
    return !(*vis == 1 || *vis == 2);
}

void nav_world_tick(bool engaged, uint32_t tick) {
    if (!g_cfg.nav_world || !engaged) {
        if (g_navw_shown) { g_navw_shown = false; navw_hide_all();
            API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: markers hidden (gates)"); }
        return;
    }
    if (!g_have_view_pos.load() || !g_have_render_yaw.load()) return;

    // ---- CALIBRATION: two probes through the game's own projection. A yields the camera's
    // rotation, B (offset a known 10Â° in view yaw) yields the constant; one fixed-point pass
    // resolves their mutual dependency, and the EMA absorbs the rest.
    static API::UObject* gps = nullptr;
    static API::UObject* wll = nullptr;
    static bool tried = false;
    if (!tried) {
        tried = true;
        NAVW_MARK("entry:resolve_CDOs");
        if (auto* c = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.GameplayStatics"))
            gps = c->get_class_default_object();
        if (auto* c = API::get()->find_uobject<API::UClass>(L"Class /Script/UMG.WidgetLayoutLibrary"))
            wll = c->get_class_default_object();
    }
    // The only engine call on the entry path that runs EVERY tick, and until now unmarked -- so a
    // fault here was indistinguishable from a fault in the gate-disengage path above.
    NAVW_MARK("entry:get_player_controller");
    auto* pc = API::get()->get_player_controller(0);

    // Every remaining exit says WHY, rate-limited -- the v1 of this function went dark instead,
    // and locating its dead gate took a live session.
    static uint32_t s_blocked_log = 0;
    auto blocked = [&](const char* why) {
        if (tick - s_blocked_log >= 600) {
            s_blocked_log = tick;
            API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: blocked(%s)", why);
        }
    };

    if (gps == nullptr || wll == nullptr || pc == nullptr) { blocked("no projection surface"); return; }

    const float vy = g_render_view_yaw.load(), vp = g_render_view_pitch.load();
    const Vec3  o{g_view_pos_x.load(), g_view_pos_y.load(), g_view_pos_z.load()};
    auto project = [&](float yaw_deg, float pitch_deg, double* sx, double* sy) -> bool {
        const float cy2 = std::cos(pitch_deg * DEG2RAD);
        const Vec3 w{o.x + cy2 * std::cos(yaw_deg * DEG2RAD) * 1000.0f,
                     o.y + cy2 * std::sin(yaw_deg * DEG2RAD) * 1000.0f,
                     o.z + std::sin(pitch_deg * DEG2RAD) * 1000.0f};
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<void**>(p) = pc;
        auto* wv = reinterpret_cast<double*>(p + 8);
        wv[0] = w.x; wv[1] = w.y; wv[2] = w.z;
        NAVW_MARK("ProjectWorldToScreen@3762");
        gps->call_function(L"ProjectWorldToScreen", p);
        const auto* sp = reinterpret_cast<const double*>(p + 32);
        *sx = sp[0]; *sy = sp[1];
        return p[49] != 0 && std::isfinite(sp[0]) && std::isfinite(sp[1]);
    };

    double vw = 0, vh = 0;
    float keff = g_navw_keff.load(), cam_yaw = 0.0f, cam_pitch = 0.0f;
    // ⚠️ LANE 2 MUST NOT ENTER THIS BLOCK. It reads the objective's world position directly and
    // needs no projection, no calibration and no conditioning -- but the block below RETURNS
    // early on a rejected probe, and every one of those returns leaves the markers exactly where
    // they were. That is a marker frozen in world space that the player can walk around, which
    // is precisely what the field reported ("sometimes it just disconnects"). The guard exists
    // for the screen-inversion lanes; running it ahead of a lane that does not need it is the
    // same mistake as the first time this ordering was wrong, and it has now cost two rounds.
    if (g_cfg.nav_world_src != 2) {
        // Probes + conditioning + calibration -- screen-inversion lanes only. Lane 1 reads each
        // LIVE SCREEN POSITION out of the instance map (f32 pair at element+0x50 -- the anatomy
        // dumps settled the layout after the "world position" read was refuted twice) and
        // inverts it through this same measured projection, so it needs keff/cam like lane 0.
        //
        // CONDITIONING RULE (added after live keff collapse, 2026-08-14): the probes only carry
        // signal while the view sits reasonably inside the aim camera's frustum. When the hand
        // is far off the view -- exactly when a naive sample is garbage -- the right move is to
        // KEEP the previous markers and calibration untouched, not to hide anything and not to
        // learn from a degenerate sample. keff=300 with cam_pitch=-66 in the field log was this
        // failure: ill-conditioned samples poisoning the EMA and flinging every marker.
        double axs = 0, ays = 0, bxs = 0, bys = 0;
        if (!project(vy, vp, &axs, &ays)) { blocked("probe A rejected -- keeping last markers"); return; }
        if (!project(vy + 10.0f, vp, &bxs, &bys)) { blocked("probe B rejected -- keeping last markers"); return; }
        {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            *reinterpret_cast<void**>(p) = pc;
            NAVW_MARK("GetViewportSize@3795");
            wll->call_function(L"GetViewportSize", p);
            const auto* vs = reinterpret_cast<const double*>(p + 8);
            vw = vs[0]; vh = vs[1];
        }
        if (vw <= 1.0 || vh <= 1.0) { blocked("no viewport"); return; }
        if (std::fabs(axs - vw * 0.5) > vw * 0.40 || std::fabs(ays - vh * 0.5) > vh * 0.40) {
            blocked("aim too far off view -- keeping last markers");
            return;
        }

        for (int it = 0; it < 2; ++it) {   // fixed-point: d needs k, k needs d
            const float d  = std::atan((float)(axs - vw * 0.5) / keff);
            const float t2 = std::tan(d + 10.0f * DEG2RAD) - std::tan(d);
            if (std::fabs(t2) < 1e-4f) break;
            const float k = (float)(bxs - axs) / t2;
            // Plausibility BAND, not just finiteness: px-per-tan is (vw/2)/tan(hfov/2), so any
            // real value sits between ~0.2*vw (hfov 136) and ~0.9*vw (hfov 58). Outside =
            // degenerate sample; keep the previous constant.
            if (std::isfinite(k) && k > 0.2f * (float)vw && k < 0.9f * (float)vw) keff = k;
        }
        {
            const float prev = g_navw_keff.load();
            g_navw_keff = prev + 0.2f * (keff - prev);
            keff = g_navw_keff.load();
        }
        // The aim camera's true rotation, from probe A (which lies along the RENDERED view).
        cam_yaw   = vy - RAD2DEG * std::atan((float)(axs - vw * 0.5) / keff);
        cam_pitch = vp + RAD2DEG * std::atan((float)(ays - vh * 0.5) / keff);
    }

    // ---- LANE 1 (navworldsrc=1, default): TRUE WORLD POSITIONS from the manager's
    // NavpointInstances map. Decoded live 2026-08-14 by raw-dumping an element while comparing
    // against the camera: each instance carries the marker's CURRENT SCREEN POSITION at +0x50
    // (f32 pair, per-frame) and the TARGET WORLD POSITION at +0x78 (FVector, doubles, UE cm --
    // verified static across view motion and ~273 m from the camera at island-plausible
    // coordinates). Placing at the true position beats every screen-inversion scheme: exact at
    // any distance, correct even BEHIND the player (the flat layer's edge-clamp limitation
    // simply ceases to exist), and needs no calibration at all.
    //
    // The property is a TMap (MapProperty -- no reflected element type reachable from here), so
    // iteration is raw: {data, num, max} header, elements at data + slot*navworldstride, and a
    // SPARSE array -- removed markers leave holes. Every slot is therefore VALIDATED (finite,
    // inside world bounds, not all-zero) and invalid slots are skipped; a wrong stride yields
    // skipped slots and a log count, never garbage placement. Stride is a dev-tunable for the
    // day a game patch moves it.
    // ---- LANE 2 (navworldsrc=2, DEFAULT): THE OBJECTIVE'S TRUE WORLD POSITION.
    //
    // SETTLED 2026-08-16 by the manager-rooted graph walk (navscan=5) plus live verification:
    //
    //   manager                                  (UObject, via the widget tree -- reflection)
    //     +0xB8  NavpointInstances {data,num,max}          (the TMap, named property)
    //       element[s] at data + s*stride              (stride 0x78; screen pair at +0x50)
    //         +0x20 -> entry ; position FVector3f at entry+0x28
    //         +0x28 -> entry ; position FVector3f at entry+0x38   (same address, 0x10 apart)
    //
    // PROOF it is the OBJECTIVE and not the player: read (-18648.5, 7074.3, 898.2) while the
    // player stood at (-18560.5, 7192.1) -- ~1.5 m apart, matching the field report that the
    // objective cannot be approached closer than ~2 m -- and it stayed CONSTANT as the camera
    // moved. (The player-position hits in the same walk came from manager+0x60
    // CachedPlayerController, a chain this lane deliberately does not use.)
    //
    // Entries are NOT uniform: element[1] in the same live map had no position at either
    // offset (a different navpoint kind). So every candidate is validated and skipped when
    // implausible -- a marker is only ever placed on data that reads like a world position.
    if (g_cfg.nav_world_src == 2) {
        // ONE evaluation for the whole lane, not one per marker: xrlayer_live() reads an atomic the
        // watchdog owns, and a value that changed halfway down the loop would leave half the
        // markers on each path with no way to tell from the log which.
        const bool layer_owns = navw_layer_owns();

        // MARKED FROM HERE DOWN. The shipped lane (navworldsrc=2) never enters the projection block
        // above, so the first mark a shipping build could reach was SetVisibility in the PLACE pass
        // -- everything between the entry and that point, including this whole sweep, read as
        // `step '-'`. The 2026-09-08 handoff took '-' to mean "before line ~3794"; on the shipped
        // path it meant "anywhere in the next ~600 lines".
        NAVW_MARK("lane2:manager_lookup");
        API::UObject* mgr = nullptr;
        for (int i = 0; i < g_nav_count; ++i) {
            auto* w = g_navpoints[i].get();
            if (w == nullptr || !navw_is_live_widget(w)) continue;
            if (auto* mp = w->get_property_data<void*>(L"NavpointsManager")) {
                if (*mp != nullptr) { mgr = reinterpret_cast<API::UObject*>(*mp); break; }
            }
        }
        // FAIL BY HIDING, NEVER BY FREEZING. A marker left at its last world position when the
        // data behind it goes away reads as a waypoint you can walk around -- worse than no
        // waypoint, because it is confidently wrong. Every exit below hides first.
        if (mgr == nullptr) {
            blocked("no live NavpointsManager");
            if (g_navw_shown) { g_navw_shown = false; navw_hide_all(); g_navw_placed_n = 0; }
            return;
        }
        struct FMapRaw { void* data; int32_t num; int32_t max; };
        NAVW_MARK("lane2:NavpointInstances");
        auto* map = mgr->get_property_data<FMapRaw>(L"NavpointInstances");
        if (map == nullptr || map->data == nullptr || map->num <= 0) {
            if (g_navw_shown) { g_navw_shown = false; navw_hide_all(); g_navw_placed_n = 0; }
            return;
        }
        // ---- WHICH ENTRIES ARE ACTUALLY LIVE: the TSparseArray allocation bitmap --------------
        //
        // THE BUG THIS EXISTS FOR, reported 2026-09-04 and finally described precisely enough to
        // act on: a marker appears in a wrong place when an item unassigns (the game culls against
        // the AIM-DRIVEN camera, not the HMD) and then STAYS -- "can be indefinite if I never cause
        // a nav marker reassignment". INDEFINITE IS THE TELL. A transient read clears itself; an
        // entry that keeps resolving a position every tick, forever, is a FREED SLOT whose contents
        // survive the free. The sweep below walks raw indices and accepts the first `num` that
        // resolve, so a hole is indistinguishable from a live entry: it has a plausible stale
        // position, a plausible class, and passes every finite/range check we own.
        //
        // ADDR-HYGIENE: structural -- offsets into UE's own TSparseArray/TBitArray, whose layout is
        // fixed by the engine rather than measured from this build, and NOTHING IS EVER WRITTEN
        // through them. Validated before use and fails OPEN: a wrong layout gives alloc_ok=false
        // and the sweep behaves exactly as it did before.
        //
        // THE VALIDATION IS ALSO THE ANSWER TO A QUESTION THIS FILE HAS CARRIED UNRESOLVED. The
        // sweep's own comment says `num` "could be the sparse array's slot count (holes below it)
        // or the map's live pair count" and bounds the accepted count to survive both readings. If
        // NumBits == num then num is the SLOT COUNT, the popcount is the live count, and the gap
        // between them is exactly the phantom population.
        struct FBitArrayRaw {
            uint32_t  inline_bits[4];   // TInlineAllocator<4>: bits 0..127 live here
            uint32_t* secondary;        // only used above 128 bits
            int32_t   num_bits;
            int32_t   max_bits;
        };
        const auto* alloc = reinterpret_cast<const FBitArrayRaw*>(
                                reinterpret_cast<const uint8_t*>(map) + sizeof(FMapRaw));
        bool    alloc_ok   = false;
        int32_t alloc_live = 0;
        if (!IsBadReadPtr(alloc, sizeof(FBitArrayRaw))) {
            // NumBits tracks Data.Num() one-for-one in TSparseArray::Add, so this is a tight
            // structural check: a wrong offset almost never lands on a value that equals num.
            alloc_ok = (alloc->num_bits == map->num)
                    && (alloc->max_bits >= alloc->num_bits)
                    && (alloc->num_bits > 0) && (alloc->num_bits <= 4096)
                    && (alloc->num_bits <= 128 ? alloc->secondary == nullptr
                                               : !IsBadReadPtr(alloc->secondary, 8));
            if (alloc_ok) {
                for (int32_t i2 = 0; i2 < alloc->num_bits; ++i2) {
                    const uint32_t w = (i2 < 128) ? alloc->inline_bits[i2 >> 5]
                                                  : alloc->secondary[i2 >> 5];
                    if ((w >> (i2 & 31)) & 1u) ++alloc_live;
                }
                // At least one live entry, never more than there are slots. All-zeros or all-ones
                // is what an unrelated field looks like.
                alloc_ok = (alloc_live > 0) && (alloc_live <= alloc->num_bits);
            }
        }
        auto entry_live = [&](int32_t idx) -> bool {
            if (!alloc_ok || idx < 0 || idx >= alloc->num_bits) return true;   // fail OPEN
            const uint32_t w = (idx < 128) ? alloc->inline_bits[idx >> 5]
                                           : alloc->secondary[idx >> 5];
            return ((w >> (idx & 31)) & 1u) != 0;
        };
        {
            // Say it once, and again whenever the answer changes. "Resolved" here is a claim about
            // someone else's memory layout; it has to be checkable from a support log.
            static int     s_alloc_state = -1;
            static int32_t s_alloc_shape = -1;
            const int      state = alloc_ok ? 1 : 0;
            const int32_t  shape = alloc_ok ? (map->num * 1000 + alloc_live) : -1;
            if (state != s_alloc_state || shape != s_alloc_shape) {
                s_alloc_state = state; s_alloc_shape = shape;
                if (alloc_ok) {
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] NAVWORLD: allocation bitmap RESOLVED -- num=%d is the "
                        "SLOT count, %d live (%d hole(s)). Holes are freed entries whose stale "
                        "position still resolves; they are now skipped.",
                        map->num, alloc_live, map->num - alloc_live);
                } else {
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] NAVWORLD: allocation bitmap did NOT validate "
                        "(numbits=%d vs num=%d) -- falling open, every resolvable slot accepted "
                        "exactly as before. Freed entries can still be drawn.",
                        IsBadReadPtr(alloc, sizeof(FBitArrayRaw)) ? -1 : alloc->num_bits, map->num);
                }
            }
        }
        const int32_t stride = (g_cfg.nav_world_stride > 0x40 && g_cfg.nav_world_stride < 0x400)
                             ? g_cfg.nav_world_stride : 0x78;
        // THE RENDERED EYE, not the game camera. Every ray below (direction, trace, facing) is
        // cast from where the player's eye actually is, so a marker drawn short still lines up
        // with the objective behind it. Using the camera made the alignment range-dependent.
        const Vec3 vo = g_have_eye_pos.load()
            ? Vec3{g_eye_pos_x.load(), g_eye_pos_y.load(), g_eye_pos_z.load()}
            : Vec3{g_view_pos_x.load(), g_view_pos_y.load(), g_view_pos_z.load()};

        // Read an FVector3f through one candidate chain. Returns false unless every step is
        // readable AND the result reads like a world position.
        auto try_chain = [&](const uint8_t* elem, int32_t ptr_off, int32_t pos_off, Vec3* out) -> bool {
            if (IsBadReadPtr(elem + ptr_off, 8)) return false;
            const uint64_t p = *reinterpret_cast<const uint64_t* const>(elem + ptr_off);
            if (p < 0x10000 || p > 0x7FFFFFFFFFFFull) return false;
            const uint8_t* e = reinterpret_cast<const uint8_t*>((uintptr_t)p);
            if (IsBadReadPtr(e + pos_off, 12)) return false;
            const float* f = reinterpret_cast<const float*>(e + pos_off);
            if (!std::isfinite(f[0]) || !std::isfinite(f[1]) || !std::isfinite(f[2])) return false;
            if (std::fabs(f[0]) > 1.0e6f || std::fabs(f[1]) > 1.0e6f || std::fabs(f[2]) > 1.0e5f) return false;
            // NEAR-zero, not exactly zero. An unbound entry holds tiny fractions (~1e-8), which
            // an == 0 test passes and %.0f prints as "-0" -- that is how a phantom marker ended
            // up parked at the world origin 191 m away, field-visible as a stray icon.
            if (std::fabs(f[0]) < 1.0f && std::fabs(f[1]) < 1.0f) return false;
            *out = Vec3{f[0], f[1], f[2]};
            return true;
        };

        // ============================ BUG 1: STABLE SLOT KEYING ==============================
        // The old loop assigned compositor slots by DENSE-PACKED resolve order (a running `used`
        // counter), so a navpoint's slot depended on how many EARLIER entries resolved this tick.
        // When an enemy navpoint entered or left the HMD view the earlier set shifted and every
        // later marker was bumped to a different slot -- the ~0.5 s flicker, logged as one slot
        // thrashing between an objective and a floor-item. Three passes now: RESOLVE every entry
        // tagged with its stable identity (the sparse TMap index s), ASSIGN each identity a
        // persistent slot, PLACE per slot. A given navpoint keeps one slot for its whole lifetime.

        // ---- PASS A: RESOLVE. Collect entries that read like a live navpoint the game is showing.
        struct NavwCand { uint64_t ident; int prio; Vec3 wp; API::UClass* ecls; int slot; };
        NavwCand cand[16];
        int n_cand = 0, seen_entries = 0;

        // POPULATION CAP: `max` IS CAPACITY, NOT POPULATION. FScriptArray's ArrayMax is the
        // allocated capacity; everything past the map's own count is memory it has never written.
        // The field log of 2026-08-25 read `map num=5 max=24` for most of a session, so this loop
        // was dereferencing chains out of nineteen uninitialised slots on every tick and would
        // place a marker on any of them whose leftover bytes happened to survive try_chain's
        // plausibility checks -- a phantom generator that needs no game bug at all to fire.
        //
        // ⚠️ THE CAP IS ON THE COUNT ACCEPTED, NOT ON THE INDEX SCANNED, and that distinction is
        // the whole safety argument. `num` could be the sparse array's slot count (holes below it)
        // or the map's live pair count; under the second reading a live entry can legitimately sit
        // at an index ABOVE num, and bounding the LOOP at num would silently delete a real waypoint
        // on a layout we had merely guessed wrong. Bounding the accepted COUNT is correct under
        // BOTH readings -- neither can produce more than `num` live entries -- so it can never drop
        // a real navpoint, while still refusing to harvest a whole capacity's worth of garbage.
        //
        // census.beyond counts entries that resolved AFTER the cap was reached: resolvable entries
        // in excess of what the map says exists, i.e. a direct measurement of how much garbage the
        // old unbounded scan was feeding in. Zero means the bound never mattered.
        const int32_t scan_hi = (map->max < 16) ? map->max : 16;
        const int32_t pop_cap = (map->num > 0 && map->num < scan_hi) ? map->num : scan_hi;

#if HALO_VR_DEV
        // Per-entry census cadence, decided ONCE for the whole sweep and armed here rather than
        // inside the loop. Latching it on a particular index (`s == 0`) would never re-arm on a map
        // whose slot 0 does not resolve -- which is the normal case -- and the "rate-limited"
        // diagnostic would then print every entry every tick. A per-tick log spew is a frame hitch,
        // and a frame hitch in VR is nausea.
        static uint32_t s_entry_log = 0;
        const bool census_slow = (tick - s_entry_log >= 600);
        if (census_slow) s_entry_log = tick;
        const bool census_fast = (g_cfg.nav_world_log != 0) && ((tick % 64) == 0);
#endif

        for (int32_t s = 0; s < scan_hi && n_cand < 16; ++s) {
            const uint8_t* elem = reinterpret_cast<const uint8_t*>(map->data) + (size_t)s * stride;
            if (IsBadReadPtr(elem, (size_t)stride)) continue;
            // A FREED SLOT IS SKIPPED BEFORE IT COSTS ANYTHING -- deliberately above the pop_cap
            // accounting. A hole that consumed budget would push a genuinely live entry at a higher
            // index out of the accepted set, which is the other half of this bug: not only is a
            // phantom drawn, a real navpoint can be crowded out by one. Fails open (entry_live
            // returns true) whenever the bitmap did not validate.
            if (!entry_live(s)) { ++g_navw_census.deadslot; continue; }
            Vec3 wp{};
            if (!try_chain(elem, 0x20, 0x28, &wp) && !try_chain(elem, 0x28, 0x38, &wp)) continue;
            if (seen_entries >= pop_cap) { ++g_navw_census.beyond; continue; }
            ++seen_entries;
            ++g_navw_census.examined;

            // THIS navpoint's own art class. Element+0x10 is its widget class (anatomy dump: +0x08
            // is the live widget instance, +0x10 its WidgetBlueprintGeneratedClass), so an
            // objective, a co-op partner and a tracked enemy each keep their authored icon and
            // colour instead of every marker wearing the objective's. Read here so priority and
            // the per-kind art are decided without hosting anything.
            API::UClass* ecls = nullptr;
            if (!IsBadReadPtr(elem + 0x10, 8)) {
                auto* c2 = *reinterpret_cast<API::UObject* const*>(elem + 0x10);
                if (c2 != nullptr && !IsBadReadPtr(c2, 0x30)) {
                    // Same rule as navw_entry_widget: prove it is a live UObject before its class
                    // pointer is read and named. A UClass is a UObject, so the slot test applies.
                    if (!uobject_slot_valid(c2)) {
                        navw_note_stale_ptr("elem+0x10", c2);
                    } else {
                        NAVW_MARK("elem+0x10:class_name_of");
                        if (class_name_of(c2).find(L"WidgetBlueprintGeneratedClass") != std::wstring::npos) {
                            ecls = reinterpret_cast<API::UClass*>(c2);
                        }
                    }
                }
            }
            // THE NAME OF THE CLASS ITSELF -- not the name of the class's class.
            //
            // class_name_of(obj) answers obj->get_class()->get_fname(). That is exactly right for a
            // widget INSTANCE, which is how the HOSTING path uses it (navw_host_class passes the
            // live widget and gets "WBP_NavpointObjective_C"). It is wrong for a UClass: ecls IS a
            // class, so its class is the metaclass, and the answer is the constant string
            // "WidgetBlueprintGeneratedClass" for every navpoint in the game.
            //
            // MEASURED 2026-09-02, from the user's per-entry census: EVERY row read
            // `cls=WidgetBlueprintGeneratedClass kind=other`. navw_classify has therefore never once
            // seen a real class name on this path, and two things silently depended on it:
            //   * navworldkindmask could not distinguish an item from an objective, so the
            //     documented stopgap was inert whatever it was set to.
            //   * prio below is (kind == NAVW_OBJECTIVE) ? 1 : 2, so the OBJECTIVE never got
            //     priority 1 and could be shed by a short slot budget like any floor item -- one of
            //     the two reasons the objective marker goes missing while item markers do not.
            NAVW_MARK("ecls:to_string");
            const std::wstring ecn = (ecls != nullptr && ecls->get_fname() != nullptr)
                                   ? ecls->get_fname()->to_string() : std::wstring();
            const NavwKind kind = ecn.empty() ? NAVW_OTHER : navw_classify(ecn);

            // ---- STABLE IDENTITY (see g_navw_slot_ident). The live widget instance, validated by
            // reflection, is what this navpoint IS; the sparse index is only where the map filed it
            // this instant. One read, shared with the gate below.
            API::UObject* ewidget = navw_entry_widget(elem);
            uint64_t ident;
            if (ewidget != nullptr) {
                ident = (uint64_t)(uintptr_t)ewidget;
            } else {
                // ---- WIDGET-LESS ENTRIES ARE KEYED ON WHAT THEY ARE, NOT WHERE THEY ARE FILED ----
                //
                // This used to be 1|(s<<1) -- the sparse index. The banner three lines up says why
                // that is wrong ("the sparse index is only where the map filed it this instant")
                // and it was written as an acceptable degradation. It is not, because the
                // population that lands here is not a rare unreadable straggler: THE OBJECTIVE
                // LIVES HERE PERMANENTLY. Measured 2026-09-02, identfb=3978 of 8746 examined, and
                // the objective is the entry with no widget on every sample.
                //
                // The map RESIZES underneath us -- the census caught it going from num=4 max=4 to
                // num=5 max=24 in one session -- and a reallocation moves entries between indices.
                // The objective's identity therefore changed, its slot was released as "gone", and
                // the assignment pass handed it whichever slot was free, which is normally one
                // already wearing WBP_NavpointWidgetItemHighlight_C. Until navw_ensure_slot
                // re-hosts it a tick or two later the player sees AN ITEM MARKER SITTING ON THE
                // OBJECTIVE -- reported verbatim as "an item marker is flickering over the
                // objective marker from time to time".
                //
                // So key on the entry's own content instead: its class plus its world position,
                // quantised to a metre so ordinary jitter cannot re-key it. Both are properties of
                // the navpoint itself and survive any amount of map reshuffling.
                //
                // WHY THE CLASS IS IN THE MIX: position alone would collide between an objective
                // and an item pickup that happen to sit on the same spot, which is exactly the
                // pairing that produces this bug's signature.
                //
                // A MOVING navpoint would re-key as it crosses metre boundaries -- but a navpoint
                // the game is drawing has a widget and never reaches this branch, so the entries
                // keyed this way are the static ones.
                // ⚠️ POSITION IS DELIBERATELY *NOT* IN THIS HASH. IT WAS, AND IT WAS A REGRESSION.
                //
                // The first version mixed the metre-quantised world position, reasoning that "a
                // navpoint the game is drawing has a widget and never reaches this branch, so the
                // entries keyed this way are the static ones". THE PREMISE IS FALSE. Measured
                // 2026-09-03 in a busy area: identfb=62853 of visgate=65179 -- 96% of entries have
                // no widget, INCLUDING WBP_NavpointDestination_C, which tracks a moving target.
                //
                // A moving navpoint crosses a metre boundary constantly, so its identity changed
                // constantly, so it was released and re-let a slot constantly, so that slot was
                // re-hosted to a different class constantly. The log shows slot 0 alone cycling
                // objective -> item -> enemy -> item -> enemy inside one sample window, with
                // rehost=60. That churn IS the wrong-art flicker, and the new cohdrawn counter
                // measured its cost: 78 frames appended within 2 ticks of a re-host against only
                // 20 coherence rejections.
                //
                // INDEX + CLASS instead. The index is what the original code used and what its own
                // banner calls "only where the map filed it this instant" -- true, but it changes
                // only on a REALLOCATION, which happened once in a session, whereas position
                // changed every few frames for every moving marker. Mixing the class in keeps the
                // one property the plain index lacked: an index re-used by a DIFFERENT kind of
                // navpoint reads as a new identity rather than inheriting the old slot's art.
                //
                // So this trades a rare, bounded churn for none of the continuous kind. It does not
                // make identity perfect -- a realloc still re-keys everything, and cohdrawn is the
                // number that says whether that residue matters.
                uint64_t h = 1469598103934665603ull;                       // FNV-1a offset basis
                auto mix = [&h](uint64_t v) {
                    for (int b = 0; b < 8; ++b) { h ^= (v & 0xFF); h *= 1099511628211ull; v >>= 8; }
                };
                mix((uint64_t)(uint32_t)s);
                mix((uint64_t)(uintptr_t)ecls);
                // Disjoint fallback space preserved: bit 0 set can never equal an 8-byte-aligned
                // UObject pointer, so these can never collide with a widget-keyed identity.
                ident = 1ull | (h << 1);
            }
            if (ewidget == nullptr) ++g_navw_census.identfb;
            // PROVE THE OFFSET, ONCE, FROM A POSITIVE RESULT -- see g_navw_widget_off_proven.
            // Set here rather than inferred anywhere else: this is the only place a widget is
            // actually resolved, so a latch that is never set means the layout never resolved,
            // which is exactly the state visgate mode 2 must not act on.
            else if (!g_navw_widget_off_proven.load(std::memory_order_relaxed)) {
                g_navw_widget_off_proven.store(true, std::memory_order_relaxed);
                API::get()->log_info(
                    "[Halo-CampE-UEVR] NAVWORLD: element widget offset PROVEN (first live widget "
                    "resolved by reflection). navworldvisgate=2 is now able to suppress entries "
                    "that have no widget; until this line appears it cannot, by design.");
            }

            // ---- AN ENTRY AT THE WORLD ORIGIN WITH NO WIDGET IS NOT A NAVPOINT ----
            //
            // MEASURED 2026-09-03. The surviving rogue marker was caught in the per-entry census:
            //
            //   s=2 cls=WBP_NavpointWidgetItemHighlight_C kind=item vis=-1 shown=1 pos=(44,0,-0)
            //
            // while the two real navpoints in the same sweep sat at (-18262,-10159,754) and
            // (-20853,-11003,626). A position of (44,0,-0) is not a place in this level; it is
            // memory that has been zeroed or never written, whose position chain still resolves and
            // still passes every finite/range check. That is the signature the earlier tombstone
            // reading predicted, finally visible now that the census prints real class names.
            //
            // BOTH CONDITIONS ARE REQUIRED, and that is what makes this safe rather than a heuristic:
            //   * NEAR THE ORIGIN -- within a metre. No navpoint in a shipped level is there.
            //   * NO LIVE WIDGET -- so the game is not currently drawing it either.
            // An entry the game HAS built a widget for is kept no matter where it claims to be; if a
            // level ever does put a real navpoint at the origin, it will have a widget and survive.
            //
            // Rejecting instead of drawing is the right way round here for once: the failure of
            // drawing it is a marker standing in the middle of the map pointing at nothing, and the
            // failure of hiding it is one missing marker that has no widget and is therefore not on
            // the game's own HUD either.
            if (ewidget == nullptr
                && std::fabs(wp.x) < 100.0f && std::fabs(wp.y) < 100.0f && std::fabs(wp.z) < 100.0f) {
                ++g_navw_census.nullpos;
                continue;
            }

            // ---- BUG 2: draw only navpoints the game's own HUD is showing. See navw_entry_shown.
            int vis_dbg = -1;
            NAVW_MARK("entry_shown:reflect");
            const bool shown = navw_entry_shown_w(ewidget, &vis_dbg, kind);

            // A FIFTH NAVPOINT CLASS WOULD OTHERWISE VANISH IN SILENCE. The gate above is an
            // allow-list, so anything navw_classify does not recognise is suppressed -- which is
            // correct for junk and wrong for a real marker type we have simply never met. Name it
            // the first time each distinct class is hidden this way, so the next Recon is a log
            // line rather than a bug report about a missing marker.
            //
            // Recon itself was found the hard way: the player reported no ally marker over Sgt
            // Johnson, and it took a class census across seven archived logs to notice that
            // WBP_NavpointRecon_C existed at all.
            if (!shown && kind == NAVW_OTHER && !ecn.empty()) {
                static std::wstring s_said[8];
                static int          s_n = 0;
                bool already = false;
                for (int k = 0; k < s_n; ++k) if (s_said[k] == ecn) { already = true; break; }
                if (!already && s_n < 8) {
                    s_said[s_n++] = ecn;
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] NAVWORLD: suppressing an UNCLASSIFIED navpoint class '%s' "
                        "-- navw_classify does not recognise it, so it is not on the visgate "
                        "allow-list. If this marks something real, give it a kind there.",
                        narrow(ecn).c_str());
                }
            }
            // STOPGAP kind filter (navworldkindmask, default 0 = allow all): a fallback for the day
            // the visibility flag cannot be resolved, NOT the primary gate.
            const bool kind_ok = (g_cfg.nav_world_kindmask == 0)
                               || ((g_cfg.nav_world_kindmask & (1 << (int)kind)) != 0);
#if HALO_VR_DEV
            // PER-ENTRY CENSUS. Deliberately NOT gated on navworldlog any more, only rate-limited:
            // the previous version of this line was the ONLY telemetry the visibility gate had, and
            // because navworldlog defaults to 0 and no shipped or user cfg sets it, a full session
            // of live play produced zero lines and nobody could answer "did the gate run?". A
            // diagnostic that needs a key nobody sets is a diagnostic that does not exist. The fast
            // (tick%64) cadence stays behind the key; the ~20 s cadence is always on in a dev build.
            //
            // It prints the IDENTITY as well as the index, which is what makes the slot collision
            // visible AS a collision: two entries whose `s` swaps while `id` stays put (or the
            // reverse) is the churn the stable keying is supposed to absorb.
            {
                if (census_fast || census_slow) {
                    // Extra flags for the visibility-flag hunt: RenderOpacity and bIsEnabled
                    // alongside the Slate Visibility the gate reads, so one live session can say
                    // WHICH one tracks the game's own show/hide -- see navw_entry_shown's banner.
                    int en_dbg = -1; float op_dbg = -1.0f;
                    if (ewidget != nullptr) {
                        if (auto* e = ewidget->get_property_data<uint8_t>(L"bIsEnabled")) en_dbg = (int)(*e & 1);
                        if (auto* o = ewidget->get_property_data<float>(L"RenderOpacity")) op_dbg = *o;
                    }
                    API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: entry s=%d id=%016llX cls=%s "
                                         "kind=%s vis=%d en=%d op=%.2f shown=%d kindok=%d "
                                         "pos=(%.0f,%.0f,%.0f)",
                                         s, (unsigned long long)ident,
                                         ecn.empty() ? "<none>" : narrow(ecn).c_str(),
                                         navw_kind_name(kind), vis_dbg, en_dbg, op_dbg, (int)shown,
                                         (int)kind_ok, wp.x, wp.y, wp.z);
                }
            }
#endif
            if (!shown)   { ++g_navw_census.vis_sup;  continue; }
            if (!kind_ok) { ++g_navw_census.kind_sup; continue; }

            // ---- NO CLASS, NO MARKER. THIS IS THE PHANTOM OBJECTIVE. --------------------------
            // An entry whose widget class at +0x10 does not read was still accepted here, and both
            // navw_ensure_slot and navw_host_class resolve a nullptr `want_class` through
            // navworldclass to a hardcoded fallback of WBP_NavpointObjective -- so a navpoint we
            // could not identify was drawn, at its real world position, WEARING THE OBJECTIVE'S
            // ICON. That is exactly the user's report: an objective marker where no objective is.
            //
            // Fail closed on the ART, which is the thing that lies. Drawing nothing costs at most
            // one marker for a navpoint we could not classify (and the game's own flat marker comes
            // back the moment this lane places nothing); drawing the objective icon costs the player
            // a trip across the level. The count is published so "how often?" is a measurement.
            if (ecls == nullptr) { ++g_navw_census.noclass; continue; }

            const int prio = (kind == NAVW_OBJECTIVE) ? 1 : 2;
            cand[n_cand++] = NavwCand{ ident, prio, wp, ecls, -1 };
        }

        // ---- PASS B: choose which candidates get a slot (top 8 by priority, then identity for
        // determinism) and map each to a STABLE slot -- keeping any identity that already holds
        // one, giving free slots to the rest, retiring slots whose identity is not among the
        // chosen. A given navpoint keeps its slot for its lifetime; a short slot budget (>8 live
        // navpoints) sheds the lowest-priority ones, never an objective.
        int order[16]; for (int i = 0; i < n_cand; ++i) order[i] = i;
        for (int i = 1; i < n_cand; ++i) {          // insertion sort by (prio asc, ident asc)
            const int key = order[i]; int j = i - 1;
            auto worse = [&](int a, int b) {
                if (cand[a].prio != cand[b].prio) return cand[a].prio > cand[b].prio;
                return cand[a].ident > cand[b].ident;
            };
            while (j >= 0 && worse(order[j], key)) { order[j + 1] = order[j]; --j; }
            order[j + 1] = key;
        }
        const int n_take = (n_cand < 8) ? n_cand : 8;
        bool chosen_ident_live[8] = {};
        for (int t = 0; t < n_take; ++t) {          // keep chosen candidates already holding a slot
            NavwCand& cc = cand[order[t]];
            for (int sl = 0; sl < 8; ++sl) {
                if (g_navw_slot_ident[sl] == cc.ident) {
                    cc.slot = sl; chosen_ident_live[sl] = true; g_navw_slot_prio[sl] = cc.prio; break;
                }
            }
        }
        // ---- A SLOT FREED THIS TICK MUST NOT BE RE-LET THIS TICK ----
        //
        // Reported from a headset 2026-09-02, and it is the residual "rogue marker" after the
        // classification fix: "whenever an item nav point supposedly gets hidden (I point my aim
        // away enough that it no longer gets considered)... the one that gets hidden gets moved
        // somewhere else instead before it disappears later when I move."
        //
        // That is exactly what the three passes below used to do. Freeing only cleared the IDENT --
        // it did not retire the quad or hide the widget, because that happens in PASS C for slots
        // with no candidate. So a slot freed here was immediately eligible again, and the
        // assignment loop scans from slot 0 and takes the LOWEST free index -- which is very often
        // the one just freed, even with untouched slots sitting spare. The slot was then re-let to
        // a DIFFERENT navpoint and PASS C placed it at that navpoint's position, still wearing the
        // outgoing one's art until navw_ensure_slot re-hosts it a tick or two later.
        //
        // The player sees the marker that should have vanished JUMP somewhere else and linger.
        // slot_cell_coherent() already suppressed the compositor quad for the art mismatch, which
        // is why this reads as "occasional" rather than constant -- it was hiding the symptom for
        // a frame or two without addressing the churn underneath.
        //
        // Deferring by one tick sends the slot through PASS C's retire-and-hide path first, which
        // is where a marker is supposed to end its life.
        bool freed_now[8] = {};
        for (int sl = 0; sl < 8; ++sl) {            // retire slots not among the chosen (gone/dropped)
            if (g_navw_slot_ident[sl] != 0 && !chosen_ident_live[sl]) {
                g_navw_slot_ident[sl] = 0; g_navw_slot_prio[sl] = 99;
                freed_now[sl] = true;
            }
        }
        for (int t = 0; t < n_take; ++t) {          // give a free slot to each still-unassigned chosen
            NavwCand& cc = cand[order[t]];
            if (cc.slot != -1) continue;
            // Preferred pass: a slot that was already idle before this tick.
            for (int sl = 0; sl < 8; ++sl) {
                if (g_navw_slot_ident[sl] == 0 && !freed_now[sl]) {
                    g_navw_slot_ident[sl] = cc.ident; g_navw_slot_prio[sl] = cc.prio; cc.slot = sl; break;
                }
            }
            if (cc.slot != -1) continue;
            // FALLBACK, so a churn storm can never STARVE a real navpoint of a slot: every free
            // slot was freed this instant, so take one anyway and accept the one-tick art lag.
            // Counted, because a fallback that never runs is a fallback nobody can trust, and one
            // that runs constantly means the deferral above is not buying anything.
            for (int sl = 0; sl < 8; ++sl) {
                if (g_navw_slot_ident[sl] == 0) {
                    g_navw_slot_ident[sl] = cc.ident; g_navw_slot_prio[sl] = cc.prio; cc.slot = sl;
                    ++g_navw_census.slotreuse;
                    break;
                }
            }
        }
        int slot_cand[8]; for (int sl = 0; sl < 8; ++sl) slot_cand[sl] = -1;
        for (int t = 0; t < n_take; ++t) if (cand[order[t]].slot >= 0) slot_cand[cand[order[t]].slot] = order[t];

        // ---- PASS C: PLACE each slot. Occupied slots run the (unchanged) per-marker placement;
        // empty slots are retired and hidden. The slot index is now the STABLE one, not resolve order.
        uint32_t placed_mask = 0; int placed_n = 0, submitted = 0;
        for (int sl = 0; sl < 8; ++sl) {
            const int ci = slot_cand[sl];
            if (ci < 0) {
                // Unused slot this tick: retire the quad first ("hidden" is "not appended this
                // frame"), then hide -- alpha under the layer (keep the cell warm), visibility
                // otherwise. Same rule as the old tail sweep.
                halo::xrlayer_retire_quad(halo::XRLAYER_SLOT_NAV_BASE + sl);
                auto* c = g_navw_pool[sl].get_checked(L"WidgetComponent");
                if (c == nullptr) continue;
                if (layer_owns) {
                    navw_set_alpha_hidden(c, true);
                } else {
                    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 0;
                    NAVW_MARK("SetVisibility@4381");
                    c->call_function(L"SetVisibility", p);
                }
                continue;
            }
            const Vec3 wp = cand[ci].wp;

            // Hand the candidate's OWN class in, so a slot created this tick is born wearing the
            // art it is about to need instead of the default objective icon it would then have to
            // re-host away from on the next tick (see navw_ensure_slot).
            auto* comp = navw_ensure_slot(sl, cand[ci].ecls);
            if (comp == nullptr) continue;   // no FP weapon this tick: slot keeps its ident, retry next

            // The return is load-bearing: false means the re-host was DEFERRED this tick (the
            // one-re-host-per-tick throttle) or failed, so this slot is still wearing the PREVIOUS
            // navpoint's widget while everything below moves its quad to the new navpoint. Feeding
            // that into on_layer keeps the compositor off the slot until the art matches -- the
            // other half of the wrong-marker-flash fix.
            const bool art_hosted = navw_host_class(comp, sl, cand[ci].ecls);

            // ---- WRONG ART IS WORSE THAN NO ART. THE SECOND PHANTOM PATH. --------------------
            // `art_hosted == false` was fed only into `on_layer`, which keeps the COMPOSITOR off
            // the slot -- but the in-scene marker below was still moved to the new navpoint's
            // position and explicitly made visible, wearing the PREVIOUS navpoint's icon. With the
            // one-re-host-per-tick throttle, a composition change that re-types several slots at
            // once leaves each of them showing the wrong icon for as many ticks as it takes to
            // work through the queue -- and the icon most often left behind is the objective's,
            // because that is the fallback every unresolved class lands on. The 2026-08-25 field
            // log shows six re-hosts across four slots inside six seconds, which is exactly that
            // queue draining.
            //
            // So: if this slot is not yet wearing the class this navpoint needs, do not draw it at
            // all this tick. Retire and hide, same as an unused slot, and let it appear next tick
            // with the right art -- one dropped frame of a marker against an icon that names the
            // wrong thing. The slot KEEPS its identity, so normally this is a deferral of a tick or
            // two, not a loss.
            //
            // THE HONEST FAILURE MODE: if hosting a particular class fails PERMANENTLY (Create
            // returns null, the class will not resolve), this defers forever and that navpoint is
            // never drawn -- traded against the old behaviour, which drew it forever wearing
            // someone else's icon. That trade is deliberate, but it must not be invisible: watch
            // `staleart` in the census line. A few per composition change is the throttle working;
            // a count that climbs steadily with no re-hosts landing is this stall, and the
            // diagnosis is then navw_host_class, not this guard.
            if (!art_hosted && g_navw_slot_class[sl] != (void*)cand[ci].ecls) {
                ++g_navw_census.staleart;
                halo::xrlayer_retire_quad(halo::XRLAYER_SLOT_NAV_BASE + sl);
                if (layer_owns) {
                    navw_set_alpha_hidden(comp, true);
                } else {
                    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 0;
                    NAVW_MARK("SetVisibility@4431");
                    comp->call_function(L"SetVisibility", p);
                }
                continue;
            }

            // ---- PLACEMENT: exact DIRECTION, managed DISTANCE.
            // The true position is known, but drawing there buries the marker behind terrain
            // and shrinks it to nothing at range. Draw along the true direction instead, at
            // min(true, navworldmax), pulled in front of anything the trace finds nearer --
            // the reticule's surface doctrine, applied to waypoints. Angular size is held
            // constant so the shortened range is invisible; the direction, which is the
            // information, stays exact.
            const float tdx = wp.x - vo.x, tdy = wp.y - vo.y, tdz = wp.z - vo.z;
            const float true_dist = std::sqrt(tdx * tdx + tdy * tdy + tdz * tdz);
            if (!(true_dist > 1.0f)) continue;   // degenerate: objective on top of the camera
            const Vec3 dir{tdx / true_dist, tdy / true_dist, tdz / true_dist};

            // ---- IS THE COMPOSITOR DRAWING THIS ONE? --------------------------------------------
            //
            // Per marker, not per lane, and it depends on the slot having ART -- not merely on the
            // key. That bootstraps safely: the in-scene marker draws normally until the layer has
            // actually captured this slot's widget, and it comes straight back if the capture is
            // ever lost. There is no state in which turning this on leaves the player with no
            // waypoint, which is the same contract the reticule's own hide is gated on.
            const int  lay_slot = halo::XRLAYER_SLOT_NAV_BASE + sl;
            // AND art_hosted: a deferred/failed re-host means the slot's art is still the previous
            // navpoint's, so keep the compositor off it (draw in-scene) until the swap lands.
            const bool on_layer = layer_owns && art_hosted && halo::xrlayer_slot_ready(lay_slot);

            // PULL BACK toward the player, the reticule's surface-offset idea applied to the
            // objective itself: draw the marker navworldback cm SHORT of the thing it marks, so
            // it floats in front of its target rather than inside it. Applied before the clamp,
            // so it only bites when the objective is nearer than navworldmax.
            float draw_dist = true_dist - g_cfg.nav_world_back;
            // ON THE LAYER THE FAR CLAMP IS ITS OWN KEY, not navworldmax.
            //
            // navworldmax exists to stop an in-scene marker shrinking to nothing and disappearing
            // behind terrain at range. Neither pressure applies to a composition layer, so reusing
            // the same number would silently tie a comfort choice to a rendering workaround. What
            // the quad's distance actually decides here is VERGENCE, and the near clamp below is a
            // fixed 1 m for the same reason.
            const float far_clamp = on_layer ? g_cfg.xr_layer_nav_dist : g_cfg.nav_world_max;
            if (draw_dist > far_clamp) draw_dist = far_clamp;
            if (on_layer && draw_dist < 100.0f) draw_dist = 100.0f;
            bool  occluded  = false;
            // Say ONCE whether tracing is even available. "Never pulled in front" reads the same
            // whether the trace is missing geometry or was never resolved at all, and that
            // ambiguity is what made the field's "sometimes invisible" hard to act on.
            {
                static int s_trace_state = -1;
                const int now_state = g_cfg.nav_world_trace ? (hit_trace_ready() ? 1 : 0) : 2;
                if (now_state != s_trace_state) {
                    s_trace_state = now_state;
                    API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: occlusion trace %s",
                                         now_state == 1 ? "READY" :
                                         now_state == 0 ? "UNAVAILABLE (markers can be occluded)"
                                                        : "disabled by navworldtrace=0");
                }
            }
            // ---- THE OCCLUSION TRACE, SKIPPED WHEN THE COMPOSITOR OWNS THE MARKER ---------------
            //
            // This trace exists for exactly one reason: to pull the marker in front of intervening
            // geometry so it is not buried in a wall. A COMPOSITION LAYER IS NEVER OCCLUDED -- it is
            // submitted after the whole post chain and composited over the finished eye images -- so
            // on that path the trace has nothing left to do. Verified by reading the code rather
            // than assumed: its only outputs are a shortened draw_dist and `occluded`, and
            // `occluded` is read by nothing but the HALO_VR_DEV log line below. Sizing survives
            // removal because sc is PROPORTIONAL to draw_dist, so angular size is invariant to how
            // far along the ray the marker is drawn -- the code says so where sc is computed.
            //
            // SKIPPED CONDITIONALLY, NEVER DELETED. The in-scene lane is the shipping path (the
            // compositor attachment needs a PDB no player has) and a marker buried in a wall is
            // exactly what it would go back to. `on_layer` is per marker and depends on the slot
            // actually having art, so a slot that loses its capture gets its trace back on the very
            // next tick.
            //
            // AND NOTE THE ASYMMETRY IS DELIBERATE: the RETICULE's trace stays on both paths. That
            // one is not an occlusion workaround -- it puts the reticle on the surface the shot will
            // hit, which is real information about where the bullet goes. Do not "tidy" the two into
            // one rule.
            if (!on_layer && g_cfg.nav_world_trace && hit_trace_ready()) {
                const Vec3 tstart{vo.x, vo.y, vo.z};
                const Vec3 tend{vo.x + dir.x * draw_dist, vo.y + dir.y * draw_dist,
                                vo.z + dir.z * draw_dist};
                Vec3 hit{};
                API::UObject* ignore[2] = {};
                int n_ignore = 0;
                if (auto* pawn = API::get()->get_local_pawn(0)) ignore[n_ignore++] = pawn;
                // LIVENESS BEFORE DEREFERENCE. Defensive, and correct on its own terms --
                // g_rig_component is a RAW pointer that a level teardown frees, and the sweep that
                // notices and drops it runs near the END of update(), thousands of lines below
                // here, so on the teardown tick this site could read a freed component and hand
                // its garbage outer to hit_trace's ignore list.
                //
                // BUT IT IS NOT THE FIX FOR THE 2026-09-07 STUTTER, and the record should say so.
                // That fault was labelled `last lane entered 'navworld_tick'` and it SURVIVED this
                // guard unchanged: same instruction, same address. The label was an artifact --
                // PerfScope only set the lane on entry and never restored it, so navworld_tick's
                // name stayed in the field for the ~900 unscoped lines that follow it. See the
                // PerfScope comment; the restore landed in the same change as this note.
                //
                // Kept because a missing liveness check on a pointer we KNOW a teardown frees is
                // worth closing regardless of which bug is open, and it costs an indexed array
                // compare that never dereferences. Skipping the ignore entry costs nothing worth
                // having: the trace may clip the player's own weapon for the one tick before the
                // sweep drops the handle.
                static int32_t s_navw_rigcomp_idx = -1;
                if (auto* rigc = reinterpret_cast<API::UObject*>(g_rig_component.load())) {
                    if (uobject_live(rigc, &s_navw_rigcomp_idx)) {
                        if (auto* wep = rigc->get_outer()) ignore[n_ignore++] = wep;
                    }
                }
                NAVW_MARK("hit_trace@4543");
                if (hit_trace(tstart, tend, ignore, n_ignore, &hit)) {
                    const float hx = hit.x - vo.x, hy = hit.y - vo.y, hz = hit.z - vo.z;
                    const float hd = std::sqrt(hx * hx + hy * hy + hz * hz);
                    if (hd > 1.0f && hd < draw_dist) {
                        // Stand off by the LARGER of the fixed offset and 20% of the hit range:
                        // a flat 60 cm leaves the quad's corners inside the surface up close.
                        const float standoff = (g_cfg.nav_world_surf > hd * 0.2f)
                                             ? g_cfg.nav_world_surf : hd * 0.2f;
                        draw_dist = (hd > standoff) ? (hd - standoff) : (hd * 0.5f);
                        occluded = true;
                    }
                }
            }
            if (draw_dist < 50.0f) draw_dist = 50.0f;   // never inside the player's face

            const Vec3 place{vo.x + dir.x * draw_dist, vo.y + dir.y * draw_dist,
                             vo.z + dir.z * draw_dist};
            {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                auto* d = reinterpret_cast<double*>(p);
                d[0] = place.x; d[1] = place.y; d[2] = place.z;
                NAVW_MARK("K2_SetWorldLocation@4564");
                comp->call_function(L"K2_SetWorldLocation", p);
            }
            {
                // Face the viewer -- a widget quad is edge-on invisible otherwise.
                const float fh = std::sqrt(dir.x * dir.x + dir.y * dir.y);
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                auto* d = reinterpret_cast<double*>(p);
                d[0] = (double)(std::atan2(-dir.z, fh) * RAD2DEG);   // normal back at the eye
                d[1] = (double)(std::atan2(-dir.y, -dir.x) * RAD2DEG);
                d[2] = 0.0;
                NAVW_MARK("K2_SetWorldRotation@4574");
                comp->call_function(L"K2_SetWorldRotation", p);
            }
            // Scale from the DRAWN distance (constant apparent size however far the marker
            // was pulled in) times the PER-KIND multiplier: an objective should read from
            // across the level, a floor weapon should not compete with it.
            const float mult = g_navw_slot_size[sl];
            const double sc = (double)(g_cfg.nav_world_scale * mult * draw_dist / 1000.0f);
            {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                auto* d = reinterpret_cast<double*>(p);
                d[0] = sc; d[1] = sc; d[2] = sc;
                NAVW_MARK("SetWorldScale3D@4585");
                comp->call_function(L"SetWorldScale3D", p);
            }
            // Publish for the render-rate re-place (see g_navw_placed), keyed by STABLE slot.
            g_navw_placed[sl] = NavwPlaced{wp.x, wp.y, wp.z, draw_dist};
            placed_mask |= (1u << sl);
            ++placed_n;
            NAVW_MARK("SetVisibility@4591");
            { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1; comp->call_function(L"SetVisibility", p); }

            // ---- THE COMPOSITOR LANE ------------------------------------------------------------
            //
            // The component keeps being placed, sized, faced and VISIBLE above whatever happens
            // here -- that is not redundancy, it is the mechanism. The compositor quad PRESENTS
            // this component's render target, so the component has to keep rendering or the layer
            // freezes on its last frame. What changes when the layer owns it is only the tint
            // alpha, which takes it out of the scene while leaving it rendered.
            if (layer_owns) {
                // Hand this component's identity and draw size to the resolver. It walks the same
                // measured UTexture/FTexture chain the reticule's uses -- see the cross-check in
                // XrSource.cpp for why one measurement is allowed to serve nine components.
                const int npx = (g_cfg.nav_world_draw > 0.0f) ? (int)g_cfg.nav_world_draw : 128;
                halo::xrsource_set_slot_component(lay_slot, (void*)comp, npx);

                // WORLD SIZE, in the same currency the reticule publishes: draw size x world
                // scale. A UWidgetComponent's quad is DrawSize units across at scale 1, so this is
                // the marker's real extent in UE centimetres and the layer needs no knowledge of
                // either key to convert it.
                const float world_cm = (float)npx * (float)sc;

                // PRIORITY: objectives outrank everything else, so a short layer budget sheds a
                // floor weapon before it sheds the thing the mission is about. Within a priority
                // the smallest apparent quad goes first, which XrLayer works out for itself.
                const int prio = (g_navw_slot_kind[sl] == NAVW_OBJECTIVE) ? 1 : 2;

                // Compositor budget: submit at most xr_layer_nav_max quads. Counted over quads
                // actually submitted this tick (not the slot index, which stable keying makes
                // non-contiguous); pass B has already dropped the lowest-priority navpoints, and
                // the layer sheds further within a priority by apparent size.
                if (submitted < g_cfg.xr_layer_nav_max) {
                    halo::xrlayer_notice_quad(lay_slot, wp, world_cm, draw_dist, prio);
                    ++submitted;
                } else {
                    halo::xrlayer_retire_quad(lay_slot);
                }
            } else if (g_cfg.xr_layer && g_cfg.xr_layer_nav) {
                // The key is on but the layer is not live (no PDB, not OpenXR, hook not running).
                // Keep resolving so the slot is warm the moment it does come up, but submit
                // nothing -- and leave the in-scene marker fully visible, which the tint below
                // does by passing on_layer=false.
                const int npx = (g_cfg.nav_world_draw > 0.0f) ? (int)g_cfg.nav_world_draw : 128;
                halo::xrsource_set_slot_component(lay_slot, (void*)comp, npx);
                halo::xrlayer_retire_quad(lay_slot);
            }

            // TINT, AND THE IN-SCENE HIDE. One write, re-asserted every tick, because the component
            // reverts its tint whenever it rebuilds its material behind our back. alpha 0 while the
            // compositor is drawing this marker; full colour otherwise.
            navw_set_alpha_hidden(comp, on_layer);
#if HALO_VR_DEV
            if (g_cfg.nav_world_log && (tick % 64) == 0) {
                API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD[%d]: ident=%016llX objective=(%.0f,%.0f,%.0f) "
                                     "true=%.0fcm drawn=%.0fcm%s%s", sl,
                                     (unsigned long long)cand[ci].ident, wp.x, wp.y, wp.z,
                                     true_dist, draw_dist, occluded ? " [pulled in front]" : "",
                                     on_layer ? " [COMPOSITOR: no trace, in-scene alpha 0]" : "");
            }
#endif
        }
        // Unused slots were already retired + hidden in the ci<0 branch above, so there is no
        // separate tail sweep to run: with stable keying the placed slots are non-contiguous.
        g_navw_placed_mask.store(placed_mask);   // published AFTER g_navw_placed is written
        g_navw_placed_n = placed_n;              // arm/disarm the render-rate re-place to match
        g_navw_shown = (placed_n > 0) || g_navw_shown;

        // ---- HIDE THE FLAT NAVPOINTS while the world markers are doing their job. Two sets of
        // waypoints for the same objectives is confusing, and the flat ones are the pair that is
        // wrong in VR (projected against the aim camera).
        //
        // SELF-HEALING, deliberately: the hide is tied to placed_n > 0, so if this lane ever stops
        // placing -- no manager, empty map, stick mode, kill switch -- the game's own markers
        // come straight back rather than leaving the player with no waypoints at all. Re-asserted
        // every tick because the HUD re-shows its children on weapon swap, respawn and scope.
        if (g_cfg.nav_hide_flat) {
            static bool s_flat_hidden = false;

            // ---- DEBOUNCE THE RESTORE. Hiding is instant; coming back is not. ----
            //
            // want_hidden was (placed_n > 0) with no hysteresis, so ANY single tick that placed no
            // world marker flashed the game's whole screen-space navpoint layer back on and off
            // again. Reported from a headset 2026-09-01 as "a few instances where a screen space
            // objective waypoint was still visible", and the census lines from that same session
            // show placed_n swinging 4,2,2,1,3,1 between samples -- so a zero tick is ordinary,
            // not exceptional.
            //
            // AND IT IS ABOUT TO GET MUCH WORSE, which is why this lands with the visgate work
            // rather than after it: placed_n is currently INFLATED by the phantom entries
            // navworldvisgate=2 suppresses (~79% of accepted entries had no live widget). Remove
            // the phantoms and placed_n reaches 0 far more often, so the un-debounced layer would
            // flicker harder the moment the other bug is fixed. Fixing one without the other
            // trades a visible fault for a different visible fault.
            //
            // Deliberately asymmetric, and it can only ever DELAY a restore, never prevent one:
            // the moment a real marker appears the flat layer hides on that same tick. The player
            // is never left without waypoints for longer than this window, which is the property
            // the original comment cared about.
            constexpr uint32_t FLAT_RESTORE_TICKS = 120;   // ~2-4 s at the game-thread rate
            static uint32_t s_no_marker_run = 0;
            if (placed_n > 0)                  s_no_marker_run = 0;
            else if (s_no_marker_run < 100000) ++s_no_marker_run;
            const bool want_hidden = (placed_n > 0) || (s_no_marker_run < FLAT_RESTORE_TICKS);
            // ---- EVERY DISTINCT CONTAINER, NOT THE FIRST ONE FOUND ----
            //
            // This loop used to `break` on the first non-null NavpointsContainer and hide only
            // that. Reported 2026-09-02: "I still see screen space icons in addition to the xr
            // layer ones" -- while the log said `flat navpoint layer hidden` and never contradicted
            // itself, because the one container we DID hide stayed hidden. A second container is
            // invisible to a check that stopped looking after the first.
            //
            // g_navpoints holds several navpoint widgets and nothing guarantees they share one
            // parent panel; the objective's screen-space icon evidently does not live in the same
            // one as the item highlights. Collecting the distinct set costs a handful of pointer
            // compares over at most g_nav_count entries, once a tick.
            // ⚠️ RESOLVED EVERY TICK, ON PURPOSE. DO NOT CACHE THESE POINTERS.
            //
            // A cache was added here on 2026-09-02 as a PERFORMANCE fix -- navw_is_live_widget()
            // costs an outer-chain walk plus an FName->wstring allocation per navpoint, and hiding
            // every container instead of the first turned ~1 allocation per tick into up to 4. The
            // cache held the resolved UObject* across up to 300 ticks, invalidated only by
            // navw_hide_all().
            //
            // IT FROZE THE GAME, TWICE, WITHIN TEN MINUTES OF SHIPPING. UEVR logged
            // "Exception occurred in on_pre_engine_tick callback" every ~25 ms -- a dangling
            // UObject* being called through on every tick. The HUD rebuilds its children on weapon
            // swap, respawn and scope (the original code says so, three lines up, which is WHY it
            // re-resolved every tick), and none of those disengage the lane, so navw_hide_all()
            // never ran and the dead pointer was never dropped. Making the cache accumulate-only --
            // added to fix a 1/2/1 count flap -- guaranteed a dead entry could never leave it.
            //
            // THE TRADE WAS INDEFENSIBLE AND THE NUMBERS SAY SO: at most 3 extra heap allocations
            // per tick, against a dangling call into the engine every tick. g_nav_count is bounded
            // at 4, so the honest cost of correctness here is trivial and always was.
            //
            // Resolving fresh also fixes the count flap for free: a tick where a navpoint is
            // transiently unresolvable simply self-corrects on the next one, which is what the
            // original code did before anyone tried to make it faster.
            API::UObject* containers[8] = {};
            int n_containers = 0;
            for (int i = 0; i < g_nav_count && n_containers < 8; ++i) {
                auto* w = g_navpoints[i].get();
                if (w == nullptr || !navw_is_live_widget(w)) continue;
                auto* pp = w->get_property_data<void*>(L"NavpointsContainer");
                if (pp == nullptr || *pp == nullptr) continue;
                auto* c = reinterpret_cast<API::UObject*>(*pp);
                bool dup = false;
                for (int k = 0; k < n_containers; ++k) if (containers[k] == c) { dup = true; break; }
                if (!dup) containers[n_containers++] = c;
            }

            if (n_containers > 0) {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                const uint8_t want_vis = want_hidden ? 1 : 4;   // 1 = Collapsed, 4 = SelfHitTestInvisible
                p[0] = want_vis;
                NAVW_MARK("SetVisibility@4746");
                for (int k = 0; k < n_containers; ++k) containers[k]->call_function(L"SetVisibility", p);

                // ---- DID THE HIDE LAND, AND ON WHAT? ------------------------------------------
                //
                // The count alone said "we called SetVisibility on N objects". It did NOT say which
                // objects, nor whether the call took. The player has reported screen-space icons
                // showing through this hide across several sessions, and every one of those reports
                // was compatible with: the call landing on the wrong container, the call being
                // reverted by the HUD, or a container we never find at all. Nothing here could tell
                // those apart, and I spent four fixes on the compositor instead.
                //
                // So name each container and READ ITS VISIBILITY BACK. A readback that disagrees
                // with what we asked is the engine or the HUD overriding us, which is a completely
                // different bug from not finding the container -- and the two are indistinguishable
                // from the outside.
                static int      s_last_n = -1;
                static bool     s_last_ok = true;
                bool all_took = true;
                char names[512]; names[0] = '\0';
                for (int k = 0; k < n_containers; ++k) {
                    int got = -1;
                    if (auto* vp = containers[k]->get_property_data<uint8_t>(L"Visibility")) got = (int)*vp;
                    if (got != (int)want_vis) all_took = false;
                    char one[128];
                    _snprintf_s(one, sizeof(one), _TRUNCATE, "%s%s@%p vis=%d%s",
                                (k > 0) ? ", " : "",
                                narrow(class_name_of(containers[k])).c_str(),
                                (void*)containers[k], got,
                                (got != (int)want_vis) ? " REVERTED" : "");
                    strncat_s(names, sizeof(names), one, _TRUNCATE);
                }
                if (want_hidden != s_flat_hidden || n_containers != s_last_n || all_took != s_last_ok) {
                    s_flat_hidden = want_hidden;
                    s_last_n      = n_containers;
                    s_last_ok     = all_took;
                    API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: flat navpoint layer %s "
                                         "(%d container(s), asked vis=%d, %s) -- %s",
                                         want_hidden ? "hidden (world markers active)"
                                                     : "restored (no world markers)",
                                         n_containers, (int)want_vis,
                                         all_took ? "all took" : "AT LEAST ONE DID NOT TAKE",
                                         names);
                }
            } else {
                // THE SILENT PATH THAT WAS NOT THERE BEFORE. Finding no container at all means
                // navhideflat is on and doing NOTHING, which from the player's seat is
                // indistinguishable from the feature being broken -- and there was no line in the
                // log to tell the two apart. Rate-limited; it is a per-tick condition.
                static uint32_t s_nc_log = 0;
                if (tick - s_nc_log >= 600) {
                    s_nc_log = tick;
                    API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: navhideflat is ON but NO "
                                         "NavpointsContainer resolved from %d navpoint widget(s) -- "
                                         "the game's screen-space icons are being left visible.",
                                         g_nav_count);
                }
            }
        }
        static uint32_t last_tlog = 0;
        if (tick - last_tlog >= 600) {
            last_tlog = tick;
            // ---- THE CENSUS LINE. Every field is a POSITIVE, CUMULATIVE count, printed whether or
            // not anything was suppressed, and ALWAYS COMPILED so it is present in a release build.
            //
            // This exists because the previous version of this line reported only `resolved` and
            // `shown`, which happen to be equal whenever the visibility gate is unanimous -- and it
            // was unanimous for the whole of the 2026-08-25 session, so the log looked healthy while
            // the gate was doing nothing at all. `visgate=E/S` says outright how many entries the
            // gate EXAMINED and how many it SUPPRESSED; S staying 0 while E climbs is the gate
            // proving its own uselessness instead of hiding it. `vis` is the bitmask of
            // ESlateVisibility values ever seen -- 0x1 alone means the game never sets this flag on
            // these widgets and the gate is watching the wrong state (see navw_entry_shown).
            API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: %d marker(s) at OBJECTIVE world "
                                 "positions (map num=%d max=%d, %d entr(ies) resolved, %d shown) | "
                                 "census visgate=%u/%u vis=0x%X kindmask=%u noclass=%u beyond=%u "
                                 "staleart=%u rehost=%u identfb=%u slotreuse=%u nullpos=%u deadslot=%u "
                                 "stale=%u",
                                 placed_n, map->num, map->max, seen_entries, n_cand,
                                 g_navw_census.examined, g_navw_census.vis_sup,
                                 g_navw_census.vis_seen, g_navw_census.kind_sup,
                                 g_navw_census.noclass, g_navw_census.beyond,
                                 g_navw_census.staleart, g_navw_census.rehost,
                                 g_navw_census.identfb, g_navw_census.slotreuse,
                                 g_navw_census.nullpos, g_navw_census.deadslot,
                                 g_navw_census.stale);
        }
        return;
    }

    if (g_cfg.nav_world_src == 1) {
        API::UObject* mgr = nullptr;
        for (int i = 0; i < g_nav_count; ++i) {
            auto* w = g_navpoints[i].get();
            if (w == nullptr || !navw_is_live_widget(w)) continue;
            if (auto* mp = w->get_property_data<void*>(L"NavpointsManager")) {
                if (*mp != nullptr) { mgr = reinterpret_cast<API::UObject*>(*mp); break; }
            }
        }
        if (mgr == nullptr) { blocked("no live NavpointsManager"); return; }
        struct FMapRaw { void* data; int32_t num; int32_t max; };
        auto* map = mgr->get_property_data<FMapRaw>(L"NavpointInstances");
        if (map == nullptr || map->data == nullptr || map->num <= 0) {
            // No live markers is a normal state, not a fault -- hide and go quiet.
            if (g_navw_shown) { g_navw_shown = false; navw_hide_all(); }
            return;
        }
        const int32_t stride = (g_cfg.nav_world_stride > 0x40 && g_cfg.nav_world_stride < 0x400)
                             ? g_cfg.nav_world_stride : 0x78;
        const Vec3 vo{g_view_pos_x.load(), g_view_pos_y.load(), g_view_pos_z.load()};

#if HALO_VR_DEV
        // INSTANCE ANATOMY (navworldlog >= 3, one-shot per value change): walk every 8-byte
        // slot of each live instance and NAME anything that is a UObject pointer. Exists
        // because +0x78 was field-refuted as "the position" (the warthog objective was active
        // while it read near-zero): the instance almost certainly references its TARGET OBJECT,
        // and following that pointer to the actor's live location is the offset-guess-free
        // design. Run this with an ACTIVE flat-HUD waypoint and the dump names the slot.
        {
            static int s_anat_done = 0;
            if (g_cfg.nav_world_log >= 3 && s_anat_done != g_cfg.nav_world_log) {
                s_anat_done = g_cfg.nav_world_log;
                for (int32_t s = 0; s < map->max && s < 4; ++s) {
                    const uint8_t* base = reinterpret_cast<const uint8_t*>(map->data)
                                        + (size_t)s * stride;
                    if (IsBadReadPtr(base, (size_t)stride)) continue;
                    API::get()->log_info("[Halo-CampE-UEVR] NAVANAT slot %d @%p:", s, (const void*)base);
                    for (int32_t off = 0; off + 8 <= stride; off += 8) {
                        const uint64_t qv = *reinterpret_cast<const uint64_t*>(base + off);
                        if (qv == 0) continue;
                        // Plausible heap/user pointer? Try it as a UObject and name it.
                        if (qv > 0x10000 && qv < 0x7FFFFFFFFFFF
                            && !IsBadReadPtr(reinterpret_cast<const void*>(qv), 0x30)) {
                            auto* cand = reinterpret_cast<API::UObject*>(qv);
                            const std::wstring cn2 = class_name_of(cand);
                            if (!cn2.empty()) {
                                API::get()->log_info("[Halo-CampE-UEVR] NAVANAT   +0x%02X ptr -> %s",
                                                     off, narrow(cn2).c_str());
                                continue;
                            }
                        }
                        const double dv = *reinterpret_cast<const double*>(base + off);
                        const float f0 = *reinterpret_cast<const float*>(base + off);
                        const float f1 = *reinterpret_cast<const float*>(base + off + 4);
                        if (std::isfinite(dv) && std::fabs(dv) > 1e-3 && std::fabs(dv) < 1e8) {
                            API::get()->log_info("[Halo-CampE-UEVR] NAVANAT   +0x%02X dbl %.2f "
                                                 "| f32 (%.2f, %.2f)", off, dv, f0, f1);
                        } else if (std::isfinite(f0) && std::isfinite(f1)
                                   && (std::fabs(f0) > 1e-3 || std::fabs(f1) > 1e-3)
                                   && std::fabs(f0) < 1e8 && std::fabs(f1) < 1e8) {
                            API::get()->log_info("[Halo-CampE-UEVR] NAVANAT   +0x%02X f32 (%.2f, %.2f)",
                                                 off, f0, f1);
                        }
                    }
                }
            }
        }
#endif
        int used = 0;
        int found = 0;
        for (int32_t s = 0; s < map->max && s < 16 && used < 8 && found < map->num; ++s) {
            const uint8_t* base = reinterpret_cast<const uint8_t*>(map->data) + (size_t)s * stride;
            if (IsBadReadPtr(base + 0x50, 8)) continue;
            // THE MARKER'S LIVE SCREEN POSITION (f32 pair at +0x50, viewport px) -- the one
            // per-marker quantity this map reliably carries (anatomy-dump-settled; the map is
            // WIDGET bookkeeping, world targets are not in it). Inverted through the measured
            // projection below into a world direction. Off-viewport values = stale/edge slots.
            const float* sp = reinterpret_cast<const float*>(base + 0x50);
            const float msx = sp[0], msy = sp[1];
            if (!std::isfinite(msx) || !std::isfinite(msy)) continue;
            if (msx < (float)vw * -0.2f || msx > (float)vw * 1.2f
                || msy < (float)vh * -0.2f || msy > (float)vh * 1.2f) continue;
            if (std::fabs(msx) < 1.0f && std::fabs(msy) < 1.0f) continue;   // zeroed slot
            ++found;

            float m_yaw   = cam_yaw + RAD2DEG * std::atan((msx - (float)vw * 0.5f) / keff);
            float m_pitch = cam_pitch - RAD2DEG * std::atan((msy - (float)vh * 0.5f) / keff);

            // CLAMP-AWARE DIRECTION CACHE (see g_navw_dirs). Comfortably-central screen
            // positions are trusted and cached per marker; near-edge ones are presumed clamped
            // and the cached direction is served instead when one exists.
            {
                void* wkey = nullptr;
                if (!IsBadReadPtr(base + 0x08, 8)) wkey = *reinterpret_cast<void* const*>(base + 0x08);
                const bool central = std::fabs(msx - (float)vw * 0.5f) < (float)vw * 0.33f
                                  && std::fabs(msy - (float)vh * 0.5f) < (float)vh * 0.33f;
                if (wkey != nullptr) {
                    NavwCachedDir* entry = nullptr;
                    NavwCachedDir* spare = nullptr;
                    for (auto& e : g_navw_dirs) {
                        if (e.wkey == wkey) { entry = &e; break; }
                        if (spare == nullptr && !e.valid) spare = &e;
                    }
                    if (entry == nullptr && central) {
                        entry = (spare != nullptr) ? spare : &g_navw_dirs[0];
                        entry->wkey = wkey;
                        entry->valid = false;
                    }
                    if (entry != nullptr) {
                        if (central) {
                            entry->yaw = m_yaw; entry->pitch = m_pitch; entry->valid = true;
                        } else if (entry->valid) {
                            m_yaw = entry->yaw; m_pitch = entry->pitch;
                        }
                    }
                }
            }
            const float dist = g_cfg.nav_world_dist;
            const float mcp = std::cos(m_pitch * DEG2RAD);
            const Vec3 wpos{vo.x + mcp * std::cos(m_yaw * DEG2RAD) * dist,
                            vo.y + mcp * std::sin(m_yaw * DEG2RAD) * dist,
                            vo.z + std::sin(m_pitch * DEG2RAD) * dist};

            auto* comp = navw_ensure_slot(used);
            if (comp == nullptr) break;
            {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                auto* d = reinterpret_cast<double*>(p);
                d[0] = wpos.x; d[1] = wpos.y; d[2] = wpos.z;
                NAVW_MARK("K2_SetWorldLocation@4962");
                comp->call_function(L"K2_SetWorldLocation", p);
            }
            {
                // FACE THE VIEWER -- a widget quad is edge-on invisible without this.
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                auto* d = reinterpret_cast<double*>(p);
                d[0] = (double)(-m_pitch);            // normal back toward the viewer
                d[1] = (double)wrap180(m_yaw + 180.0f);
                d[2] = 0.0;
                NAVW_MARK("K2_SetWorldRotation@4971");
                comp->call_function(L"K2_SetWorldRotation", p);
            }
            {
                const double sc = (double)(g_cfg.nav_world_scale * clampf(dist, 300.0f, 20000.0f) / 1000.0f);
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                auto* d = reinterpret_cast<double*>(p);
                d[0] = sc; d[1] = sc; d[2] = sc;
                NAVW_MARK("SetWorldScale3D@4978");
                comp->call_function(L"SetWorldScale3D", p);
            }
            NAVW_MARK("SetVisibility@4980");
            { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1; comp->call_function(L"SetVisibility", p); }
            {
                // EXPOSURE GAIN, re-asserted per placement -- the reticule-proven fix for the
                // stock Widget3D material tonemapping to black in bright scenes ("black marks in
                // the sky", twice). The component reverts the tint on internal rebuilds, so this
                // is a standing assert, not a one-shot. Unity gain when the pak's compensated
                // material bound instead.
                // gain x tint, SAME AS THE RETICULE'S EFFECTIVE VALUE (~5120 in the field log).
                // v1 applied the gain alone (5.0) -- three orders of magnitude too dim under
                // this title's pre-exposure, indistinguishable from black. The brightness lives
                // in aim_widget_tint; the two multiply.
                const float gain = g_navw_compensated
                                 ? 1.0f : g_cfg.aim_widget_gain * g_cfg.aim_widget_tint;
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                auto* c = reinterpret_cast<float*>(p);
                c[0] = gain * g_cfg.nav_world_cr;
                c[1] = gain * g_cfg.nav_world_cg;
                c[2] = gain * g_cfg.nav_world_cb;
                c[3] = 1.0f;
                NAVW_MARK("SetTintColorAndOpacity@4999");
                comp->call_function(L"SetTintColorAndOpacity", p);
            }
#if HALO_VR_DEV
            if (g_cfg.nav_world_log && (tick % 64) == 0) {
                API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD[%d]: screen=(%.0f,%.0f) -> "
                                     "yaw=%.1f pitch=%.1f", used, msx, msy, m_yaw, m_pitch);
            }
#endif
            ++used;
        }
        for (int i = used; i < 8; ++i) {
            auto* c = g_navw_pool[i].get_checked(L"WidgetComponent");
            if (c == nullptr) continue;
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            p[0] = 0;
            NAVW_MARK("SetVisibility@5014");
            c->call_function(L"SetVisibility", p);
        }
        g_navw_shown = (used > 0) || g_navw_shown;
        static uint32_t last_wlog = 0;
        if (tick - last_wlog >= 600) {
            last_wlog = tick;
            API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: %d marker(s) placed by screen-pos "
                                 "inversion (map num=%d max=%d stride=0x%X keff=%.0f cam=%.1f/%.1f)",
                                 used, map->num, map->max, stride, keff, cam_yaw, cam_pitch);
        }
        return;
    }

    // ---- LANE 0 (navworldsrc=0, fallback): the widget-tree screen-inversion walk.
    // g_navpoints holds both the live widget and the blueprint
    // archetype (same class name); only the Transient-outered one renders. The container is a
    // HaloUIOverlay (measured live 2026-08-13 via the MCP object search) -- an OVERLAY, not a
    // canvas: its children carry OverlaySlots with no coordinates, and the manager positions
    // markers through each child's RenderTransform instead. Both container families are
    // accepted; the per-child position code below handles both slot models.
    API::UObject* panel = nullptr;
    int n_live = 0, n_prop = 0;
    for (int i = 0; i < g_nav_count; ++i) {
        auto* w = g_navpoints[i].get();
        if (w == nullptr || !navw_is_live_widget(w)) continue;
        ++n_live;
        auto* pp = w->get_property_data<void*>(L"NavpointsContainer");
        if (pp == nullptr || *pp == nullptr) continue;
        ++n_prop;
        auto* cand = reinterpret_cast<API::UObject*>(*pp);
        const std::wstring ccn = class_name_of(cand);
        if (ccn.find(L"Panel") == std::wstring::npos && ccn.find(L"Overlay") == std::wstring::npos) continue;
        panel = cand;
        break;
    }
    if (panel == nullptr) {
        static char why[96];
        sprintf_s(why, "no live container (live=%d prop-ok=%d of %d)", n_live, n_prop, g_nav_count);
        blocked(why);
        return;
    }

    // The DPI scale, once per sweep: overlay-child RenderTransform translations are in slate
    // units; the engine maps those to viewport px by this factor.
    float vscale = 1.0f;
    {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<void**>(p) = pc;
        NAVW_MARK("GetViewportScale@5062");
        wll->call_function(L"GetViewportScale", p);
        const float vs2 = *reinterpret_cast<const float*>(p + 8);
        if (std::isfinite(vs2) && vs2 > 0.05f && vs2 < 20.0f) vscale = vs2;
    }

    // LEAF COLLECTION, two levels deep. Measured live: NavpointsContainer held only two
    // OverlaySlots -- nested PIN CONTAINERS (the manager's PinContainers property), with the
    // marker widgets one level down. A child that is itself a container contributes its
    // children; anything else is a leaf.
    API::UObject* leaves[24] = {};
    double leaf_base_x[24] = {}, leaf_base_y[24] = {};   // translation inherited from the pin container
    int n_leaves = 0;
    auto children_of = [&](API::UObject* cont, API::UObject** out, int cap) -> int {
        int32_t cn = 0;
        {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            NAVW_MARK("GetChildrenCount@5078");
            cont->call_function(L"GetChildrenCount", p);
            cn = *reinterpret_cast<int32_t*>(p);
        }
        int got = 0;
        for (int32_t ci = 0; ci < cn && got < cap; ++ci) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            *reinterpret_cast<int32_t*>(p) = ci;
            NAVW_MARK("GetChildAt@5085");
            cont->call_function(L"GetChildAt", p);
            auto* c = *reinterpret_cast<API::UObject**>(p + 8);
            if (c != nullptr) out[got++] = c;
        }
        return got;
    };
#if HALO_VR_DEV
    // DEEP PROBE (navworldlog >= 2, one-shot per value change): where does the game actually
    // put a marker's position? Round 1 said not the leaf's RenderTransform; round 2 said not
    // the pin container's either. This dumps every plausible widget-side channel (transforms,
    // paddings, slot paddings) plus the manager's NavpointInstances -- whose elements may carry
    // the TARGET WORLD POSITION outright, which would obsolete screen inversion entirely.
    static int s_deep_done = 0;
    const bool deep = (g_cfg.nav_world_log >= 2 && s_deep_done != g_cfg.nav_world_log);
    if (deep) s_deep_done = g_cfg.nav_world_log;
    auto dump_widget_channels = [&](API::UObject* w, const char* tag) {
        if (w == nullptr) return;
        double tx = 0, ty = 0;
        if (auto* rt = w->get_property_data<double>(L"RenderTransform")) { tx = rt[0]; ty = rt[1]; }
        float pad[4] = {0, 0, 0, 0};
        if (auto* pm = w->get_property_data<float>(L"Padding")) { memcpy(pad, pm, 16); }
        float spad[4] = {0, 0, 0, 0};
        const char* slot_cls = "?";
        static std::string slot_cls_s;
        if (auto* sp = w->get_property_data<void*>(L"Slot")) {
            if (*sp != nullptr) {
                auto* so = reinterpret_cast<API::UObject*>(*sp);
                slot_cls_s = narrow(class_name_of(so));
                slot_cls = slot_cls_s.c_str();
                if (auto* pm2 = so->get_property_data<float>(L"Padding")) { memcpy(spad, pm2, 16); }
            }
        }
        API::get()->log_info("[Halo-CampE-UEVR] NAVDEEP %s: %s rt=(%.1f,%.1f) pad=(%.1f,%.1f,%.1f,%.1f) "
                             "slot=%s spad=(%.1f,%.1f,%.1f,%.1f)",
                             tag, narrow(class_name_of(w)).c_str(), tx, ty,
                             pad[0], pad[1], pad[2], pad[3], slot_cls,
                             spad[0], spad[1], spad[2], spad[3]);
    };
    auto dump_fields_of = [&](API::UObject* o, const char* tag) {
        if (o == nullptr) return;
        auto* cls2 = o->get_class();
        API::get()->log_info("[Halo-CampE-UEVR] NAVDEEP %s: instance of %s", tag,
                             narrow(class_name_of(o)).c_str());
        int lvl = 0;
        for (auto* s2 = reinterpret_cast<API::UStruct*>(cls2); s2 != nullptr && lvl < 3;
             s2 = s2->get_super_struct(), ++lvl) {
            int n2 = 0;
            for (auto* f2 = s2->get_child_properties(); f2 != nullptr && n2 < 48; f2 = f2->get_next(), ++n2) {
                auto* nm = f2->get_fname();
                if (nm == nullptr) continue;
                API::get()->log_info("[Halo-CampE-UEVR] NAVDEEP %s:   +0x%04X %s", tag,
                                     reinterpret_cast<API::FProperty*>(f2)->get_offset(),
                                     narrow(nm->to_string()).c_str());
            }
        }
    };
    if (deep) {
        // The manager and its instance array, reached through the live widget.
        for (int i = 0; i < g_nav_count; ++i) {
            auto* w = g_navpoints[i].get();
            if (w == nullptr || !navw_is_live_widget(w)) continue;
            if (auto* mp = w->get_property_data<void*>(L"NavpointsManager")) {
                if (*mp != nullptr) {
                    auto* mgr = reinterpret_cast<API::UObject*>(*mp);
                    struct FRawArr { void* data; int32_t num; int32_t max; };
                    if (auto* arr2 = mgr->get_property_data<FRawArr>(L"NavpointInstances")) {
                        API::get()->log_info("[Halo-CampE-UEVR] NAVDEEP: NavpointInstances num=%d",
                                             arr2->num);
                        auto** elems = reinterpret_cast<API::UObject**>(arr2->data);
                        for (int e = 0; e < arr2->num && e < 3; ++e) {
                            if (elems != nullptr && !IsBadReadPtr(elems, sizeof(void*) * (e + 1))
                                && elems[e] != nullptr) {
                                dump_fields_of(elems[e], "inst");
                            }
                        }
                    }
                }
            }
            break;
        }
    }
#endif
    {
        API::UObject* first[12] = {};
        const int n_first = children_of(panel, first, 12);
#if HALO_VR_DEV
        if (deep) {
            dump_widget_channels(panel, "panel");
            for (int i = 0; i < n_first; ++i) {
                dump_widget_channels(first[i], "pin");
                API::UObject* second[12] = {};
                const int ns2 = children_of(first[i], second, 12);
                for (int k = 0; k < ns2 && k < 4; ++k) dump_widget_channels(second[k], "leaf");
            }
        }
#endif
        for (int i = 0; i < n_first && n_leaves < 24; ++i) {
            const std::wstring ccn2 = class_name_of(first[i]);
            const bool is_container = ccn2.find(L"Overlay") != std::wstring::npos
                                   || ccn2.find(L"Panel") != std::wstring::npos
                                   || ccn2.find(L"Box") != std::wstring::npos;
            if (is_container) {
                // THE PIN CONTAINER CARRIES THE POSITION (measured live 2026-08-14: leaves read
                // RenderTransform (0,0) and every marker collapsed onto the aim axis). The
                // manager translates each pin container; the marker widget sits untranslated
                // inside it. Inherit it here and sum with whatever the leaf itself carries.
                double bx = 0, by = 0;
                if (auto* rt = first[i]->get_property_data<double>(L"RenderTransform")) {
                    if (std::isfinite(rt[0]) && std::isfinite(rt[1])) { bx = rt[0]; by = rt[1]; }
                }
                API::UObject* second[12] = {};
                const int n_second = children_of(first[i], second, 12);
                for (int k = 0; k < n_second && n_leaves < 24; ++k) {
                    leaf_base_x[n_leaves] = bx;
                    leaf_base_y[n_leaves] = by;
                    leaves[n_leaves++] = second[k];
                }
            } else {
                leaves[n_leaves++] = first[i];
            }
        }
    }
    const int n_children = n_leaves;

    // Filter accounting, so "no markers" is always explainable from the log. The v1 of this
    // function had silent early-outs and cost a live investigation to even locate the gate --
    // the blocked(reason) doctrine exists for exactly this.
    int n_vis = 0, n_slot = 0, n_edge = 0;
    double sample_cx = 0, sample_cy = 0;
    int used = 0;
    for (int ci = 0; ci < n_children && used < 8; ++ci) {
        API::UObject* child = leaves[ci];
        if (child == nullptr) continue;
        {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            NAVW_MARK("GetVisibility@5220");
            child->call_function(L"GetVisibility", p);
            const uint8_t vis = p[0];
            // ESlateVisibility: 0=Visible, 3/4=HitTestInvisible variants; 1=Collapsed 2=Hidden.
            if (vis == 1 || vis == 2) continue;
        }
        ++n_vis;

        // POSITION SOURCE, by slot family (measured live: this container is an Overlay).
        //   * CanvasPanelSlot: position + anchors, converted to centre-relative viewport px.
        //   * OverlaySlot (no coordinates): the manager positions the child through its
        //     RenderTransform; the translation is in SLATE units of the child's local space,
        //     which the engine maps to viewport px by the DPI scale -- so multiply by
        //     GetViewportScale rather than guessing a design resolution.
        double cx = 0, cy2 = 0;
        bool have_pos = false;
        auto* slot_pp = child->get_property_data<void*>(L"Slot");
        auto* slot = (slot_pp != nullptr && *slot_pp != nullptr)
                   ? reinterpret_cast<API::UObject*>(*slot_pp) : nullptr;
        if (slot != nullptr && class_name_of(slot).find(L"CanvasPanelSlot") != std::wstring::npos) {
            {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                NAVW_MARK("GetPosition@5241");
                slot->call_function(L"GetPosition", p);
                const auto* d = reinterpret_cast<const double*>(p);
                cx = d[0]; cy2 = d[1];
            }
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            NAVW_MARK("GetAnchors@5246");
            slot->call_function(L"GetAnchors", p);
            const auto* a = reinterpret_cast<const double*>(p);   // FAnchors: Min(x,y), Max(x,y)
            const double anch_x = (a[0] + a[2]) * 0.5;
            const double anch_y = (a[1] + a[3]) * 0.5;
            if (std::isfinite(anch_x) && std::isfinite(anch_y)
                && anch_x >= -0.01 && anch_x <= 1.01 && anch_y >= -0.01 && anch_y <= 1.01) {
                cx = anch_x * vw + cx - vw * 0.5;
                cy2 = anch_y * vh + cy2 - vh * 0.5;
            } else {
                cx -= vw * 0.5;
                cy2 -= vh * 0.5;
            }
            have_pos = true;
        } else {
            // Overlay child: the PIN CONTAINER's inherited translation plus the child's own
            // RenderTransform (usually zero), slate units -> viewport px via the DPI scale.
            double ox = leaf_base_x[ci], oy = leaf_base_y[ci];
            auto* rt = child->get_property_data<double>(L"RenderTransform");
            if (rt != nullptr && std::isfinite(rt[0]) && std::isfinite(rt[1])) {
                ox += rt[0];
                oy += rt[1];
            }
            cx = ox * (double)vscale;
            cy2 = oy * (double)vscale;
            have_pos = true;
        }
        if (!have_pos) continue;
        ++n_slot;
        // Edge-clamped (the game's own off-screen arrows): direction unrecoverable, skip.
        sample_cx = cx; sample_cy = cy2;
        if (std::fabs(cx) > vw * 0.45 || std::fabs(cy2) > vh * 0.45) { ++n_edge; continue; }

        const float m_yaw   = cam_yaw + RAD2DEG * std::atan((float)cx / keff);
        const float m_pitch = cam_pitch - RAD2DEG * std::atan((float)cy2 / keff);

        auto* comp = navw_ensure_slot(used);
        if (comp == nullptr) break;
        const float dist = g_cfg.nav_world_dist;
        const float cp = std::cos(m_pitch * DEG2RAD);
        const Vec3 wpos{o.x + cp * std::cos(m_yaw * DEG2RAD) * dist,
                        o.y + cp * std::sin(m_yaw * DEG2RAD) * dist,
                        o.z + std::sin(m_pitch * DEG2RAD) * dist};
        {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            auto* d = reinterpret_cast<double*>(p);
            d[0] = wpos.x; d[1] = wpos.y; d[2] = wpos.z;
            NAVW_MARK("K2_SetWorldLocation@5292");
            comp->call_function(L"K2_SetWorldLocation", p);
        }
        {
            const double s = (double)(g_cfg.nav_world_scale * dist / 1000.0f);
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            auto* d = reinterpret_cast<double*>(p);
            d[0] = s; d[1] = s; d[2] = s;
            NAVW_MARK("SetWorldScale3D@5299");
            comp->call_function(L"SetWorldScale3D", p);
        }
        NAVW_MARK("SetVisibility@5301");
        { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1; comp->call_function(L"SetVisibility", p); }
        ++used;

#if HALO_VR_DEV
        if (g_cfg.nav_world_log && (tick % 64) == 0) {
            API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD[%d]: slot=(%.0f,%.0f) -> yaw=%.1f pitch=%.1f "
                                 "(cam=%.1f/%.1f keff=%.0f)", used - 1, cx, cy2, m_yaw, m_pitch,
                                 cam_yaw, cam_pitch, keff);
        }
#endif
    }
    for (int i = used; i < 8; ++i) {
        auto* c = g_navw_pool[i].get_checked(L"WidgetComponent");
        if (c == nullptr) continue;
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        p[0] = 0;
        NAVW_MARK("SetVisibility@5317");
        c->call_function(L"SetVisibility", p);
    }
    g_navw_shown = (used > 0) || g_navw_shown;

    static uint32_t last_log = 0;
    if (tick - last_log >= 600) {
        last_log = tick;
        if (used > 0) {
            API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: %d marker(s) world-placed "
                                 "(keff=%.0f cam=%.1f/%.1f)", used, keff, cam_yaw, cam_pitch);
        } else {
            // The blocked(reason) line: which filter ate everything, with one sample position so
            // a wrong coordinate model is visible in a single log excerpt.
            API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: 0 placed (children=%d visible=%d "
                                 "canvas-slotted=%d edge-skipped=%d lastpos=(%.0f,%.0f) "
                                 "viewport=%.0fx%.0f keff=%.0f)",
                                 n_children, n_vis, n_slot, n_edge, sample_cx, sample_cy, vw, vh, keff);
        }
    }
}

#if HALO_VR_DEV
// NAVSCAN -- the waypoint-position hunt harness (docs\CAMERA_CONSUMERS_FINDINGS.md, the
// target-derived design). The screen-space marker source is aim-poisoned beyond repair
// (field-settled 2026-08-15), so the marker's TRUE world position must come from the Blam
// side, where the game demonstrably keeps it. The hunt manufactures a known value: the player
// STANDS AT the objective, so the waypoint's stored position ~= the view position -- which
// this harness reads itself, converts per unit hypothesis, and feeds to the MemScan worker.
// The player never touches a coordinate.
//
//   navscan=1  scan for (x,y) as FLOAT UE CENTIMETRES
//   navscan=2  scan for (x,y) as FLOAT BLAM WORLD UNITS (cm / 304.8)
//   navscan=3  scan for (x,y) as FLOAT METRES          (cm / 100)
//   navscan=0  between every step -- resets the scanner's edge for the next one
//
// X,Y only, deliberately: the marker hovers some unknown height above the ground the player
// stands on, so Z would reject the very record being hunted.
void nav_scan_tick() {
    static int s_last = 0;
    if (g_cfg.nav_scan == s_last) return;
    s_last = g_cfg.nav_scan;
    if (g_cfg.nav_scan == 0) {
        g_cfg.mem_scan = false;   // re-arm the scanner's edge for the next hypothesis
        return;
    }
    if (!g_have_view_pos.load()) {
        API::get()->log_info("[Halo-CampE-UEVR] NAVSCAN: no view position yet -- enter gameplay first");
        return;
    }
    const float cx = g_view_pos_x.load(), cy = g_view_pos_y.load(), cz = g_view_pos_z.load();

    // navscan=4: the TLS-GRAPH WALK -- the anchor search. Heap scans proved the objective's
    // wu-pair lives in a RELOCATING event buffer (round 2 forensics); this instead walks the
    // pointer graph from the sim's TLS root, where a hit's CHAIN is a stable structural path.
    if (g_cfg.nav_scan == 4) {
        nav_tls_scan(cx / 304.8f, cy / 304.8f);
        return;
    }

    // navscan=5: THE UE-SIDE WALK -- manager-rooted, and the one that should have been first.
    // Graphics and UI are UE's domain on this hybrid (the research doc says so outright), so
    // the navpoints MANAGER -- a UObject reached through the widget tree navworld already
    // resolves -- is a STABLE root, unlike anything in the Blam heap. A hit's offset chain from
    // it is directly hardcodable as per-tick resolution.
    if (g_cfg.nav_scan == 5) {
        API::UObject* mgr = nullptr;
        for (int i = 0; i < g_nav_count; ++i) {
            auto* w = g_navpoints[i].get();
            if (w == nullptr || !navw_is_live_widget(w)) continue;
            if (auto* mp = w->get_property_data<void*>(L"NavpointsManager")) {
                if (*mp != nullptr) { mgr = reinterpret_cast<API::UObject*>(*mp); break; }
            }
        }
        if (mgr == nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] NAVSCAN 5: no live NavpointsManager "
                                 "(navworld=1 and an active waypoint are required)");
            return;
        }
        API::get()->log_info("[Halo-CampE-UEVR] NAVSCAN 5: manager @%p, standing at "
                             "cm=(%.1f, %.1f) -> wu=(%.3f, %.3f)",
                             (void*)mgr, cx, cy, cx / 304.8f, cy / 304.8f);
        nav_graph_scan((uintptr_t)mgr, cx / 304.8f, cy / 304.8f);
        return;
    }

    float sx = cx, sy = cy;
    const char* hyp = "UE centimetres (float)";
    switch ((g_cfg.nav_scan - 1) % 3) {
    case 1: sx = cx / 304.8f; sy = cy / 304.8f; hyp = "Blam world units (float, cm/304.8)"; break;
    case 2: sx = cx * 0.01f;  sy = cy * 0.01f;  hyp = "metres (float, cm/100)"; break;
    default: break;
    }
    snprintf(g_cfg.mem_scan_vals, sizeof(g_cfg.mem_scan_vals), "%.3f,%.3f", sx, sy);
    g_cfg.mem_scan = true;    // mem_scan_tick (called right after this in the poll) sees the edge
    API::get()->log_info("[Halo-CampE-UEVR] NAVSCAN %d: standing at cm=(%.1f, %.1f, %.1f) -> "
                         "scanning as %s: %s",
                         g_cfg.nav_scan, cx, cy, cz, hyp, g_cfg.mem_scan_vals);
}
#else
inline void nav_scan_tick() {}
#endif

// The aim control loop (control-rotation reads, the control law, pose reading) is in MotionAimControl.cpp.
// ---------------------------------------------------------------- the loop

// ---------------------------------------------------------------- reticule ray angles
//
// Which angles the reticule is drawn from, plus the two things that make the smooth option safe.
// One helper serves the on-foot and vehicle sites so they cannot drift apart.
//
// SOURCE (aim_reticule_src). 0 = the game's own aim: truthful, and the only on-screen readout of
// loop error, so it stays the default and tuning runs must use it. 1 = the controller setpoint:
// smooth, because it is intent rather than achieved angle.
//
// DIVERGENCE GUARD. Source 1 lies when the loop cannot keep up, and lies most confidently exactly
// when something is broken. So the gap is measured continuously and, once it has been too large for
// longer than a brief grace period, the reticule SNAPS back to the true aim and stays there until
// the loop reconverges. The grace period is what stops a normal fast turn -- where lag is expected,
// harmless and transient -- from flickering the source every time the player whips around.
//
// SMOOTHING. A first-order filter on the emitted angles. The jitter being hidden is high-frequency;
// the truth worth keeping is the low-frequency mean. Filtering therefore removes the distraction
// while a systematic offset still shows through, which is why it is applied to BOTH sources rather
// than only to the smooth one.
//
// Timed with steady_clock rather than a tick counter: this may be called more than once per tick,
// and wall-clock deltas stay correct in that case instead of double-counting.
static void reticule_ray_angles(double aim_yaw, double aim_pitch, float* out_yaw, float* out_pitch) {
    static bool  snapped = false;          // latched: guard currently forcing the true aim
    static float over_s  = 0.0f;           // seconds the divergence has been over threshold
    static float sm_yaw = 0.0f, sm_pitch = 0.0f;
    static bool  have_sm = false;
    static auto  last_t = std::chrono::steady_clock::now();

    const auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last_t).count();
    last_t = now;
    if (!(dt > 0.0f) || dt > 0.5f) dt = 0.0f;   // first call, or a hitch/level load: do not integrate

    float yaw   = (float)aim_yaw;
    float pitch = (float)aim_pitch;
    bool  used_setpoint = false;

    const bool want_setpoint = (g_cfg.aim_reticule_src == 1) && halo::g_aim_law_armed.load();
    if (want_setpoint) {
        const float des_yaw   = halo::g_desired_yaw.load();
        const float des_pitch = halo::g_desired_pitch.load();

        const float dy = wrap180(des_yaw   - (float)aim_yaw);
        const float dp = wrap180(des_pitch - (float)aim_pitch);
        const float diverge = std::sqrt(dy * dy + dp * dp);

        const float on_deg  = g_cfg.aim_reticule_div_deg;
        // Recover at half the trip threshold: a single threshold would sit the guard on the edge and
        // flap between sources, which is more distracting than the jitter it exists to hide.
        const float off_deg = on_deg * 0.5f;

        if (!snapped) {
            over_s = (diverge > on_deg) ? (over_s + dt) : 0.0f;
            if (over_s >= g_cfg.aim_reticule_div_ms * 0.001f) {
                snapped = true;
                API::get()->log_info("[HALO-AIM] reticule: divergence %.1f deg held past %.0f ms "
                                     "-- snapping to true aim", diverge, g_cfg.aim_reticule_div_ms);
            }
        } else if (diverge < off_deg) {
            snapped = false;
            over_s  = 0.0f;
            API::get()->log_info("[HALO-AIM] reticule: reconverged (%.1f deg) -- setpoint again", diverge);
        }

        if (!snapped) { yaw = des_yaw; pitch = des_pitch; used_setpoint = true; }
    } else {
        snapped = false;
        over_s  = 0.0f;
    }

    // Filter last, so it applies to whichever source won and a snap is eased rather than instant.
    //
    // MOTION-GATED. Smoothing unconditionally was wrong: it hid jitter while the hand was still,
    // but also lagged every deliberate swing, which is the part that actually reads as sluggish.
    // So the filter disengages as the setpoint speeds up, using the rate the control law already
    // measures for feedforward -- full smoothing when near-still, none at all during a fast move,
    // linear blend between so there is no perceptible switch.
    // PER-SOURCE strength. The two sources carry different noise and want different filters: the
    // game aim carries the control loop's residual, while the controller setpoint carries hand
    // tremor and tracking noise -- finer and faster, so it needs only a light touch to settle.
    // Keyed off the source actually used, not the one configured, so a guard snap is filtered as
    // the game aim it is now showing.
    float tau_ms = used_setpoint ? g_cfg.aim_reticule_smooth_ctrl_ms
                                 : g_cfg.aim_reticule_smooth_ms;
    {
        // MOTION-GATE RATE, measured HERE by differentiating this function's own input.
        //
        // It used to read g_setpoint_rate_dps, which only the fallback loop's feedforward path
        // writes -- and that path is further gated on a measured plant gain, so under the
        // shipped direct-drive aim it sits at 0 for a whole session. The gate then read "hand
        // is still" and every deliberate swing carried the FULL filter: precisely the lag the
        // gate was built to remove. Same trap the MOVERESID diagnostic documents at its own
        // call site, same fix: differentiate the value actually in hand.
        //
        // Details that matter:
        //   * The PRE-filter angles are differentiated. The filtered output under-reads rate
        //     during exactly the swings that must open the gate.
        //   * The instantaneous rate RISES the gate immediately (a swing's first sample already
        //     disengages the filter -- zero added lag at onset) and DECAYS over ~120 ms, so a
        //     single tremor spike at rest cannot flicker the filter off and re-admit the
        //     shimmer it exists to hide.
        //   * A source switch or divergence-guard snap is a discontinuity, not motion: the
        //     sample is skipped, so at rest a snap stays EASED by the filter as designed
        //     instead of becoming an instant jump.
        static float rate_gate_dps = 0.0f;
        static float prev_in_yaw = 0.0f, prev_in_pitch = 0.0f;
        static bool  have_prev_in = false;
        static bool  prev_used_setpoint = false;

        if (prev_used_setpoint != used_setpoint) have_prev_in = false;
        prev_used_setpoint = used_setpoint;

        if (dt > 0.0f) {
            if (have_prev_in) {
                const float dy = wrap180(yaw - prev_in_yaw);
                const float dp = pitch - prev_in_pitch;
                const float inst = std::sqrt(dy * dy + dp * dp) / dt;
                const float decay = std::exp(-dt / 0.120f);
                rate_gate_dps = (inst > rate_gate_dps) ? inst : rate_gate_dps * decay;
            }
            prev_in_yaw = yaw;
            prev_in_pitch = pitch;
            have_prev_in = true;
        }

        const float slow = g_cfg.aim_reticule_smooth_slow_dps;
        const float fast = g_cfg.aim_reticule_smooth_fast_dps;
        if (fast > slow) {
            const float t = clampf((rate_gate_dps - slow) / (fast - slow), 0.0f, 1.0f);
            tau_ms *= (1.0f - t);   // -> 0 ms (raw) as the swing gets faster
        }
    }
    // dt is the frame dt here: unlike the aim law's rate filters, this one advances every call.
    // NOTE the sense of a==0 changed with the switch to time constants, and getting it backwards
    // would be invisible in code review and obvious in a headset. It no longer means "smoothing is
    // off, pass the value through" -- tau_ms=0 gives a==1 and does that. It now means NO TIME HAS
    // PASSED (first call, or a stall the dt guard rejected), and the right answer there is to hold.
    const float a = ema_alpha(tau_ms, dt);
    if (!have_sm) {
        sm_yaw = yaw; sm_pitch = pitch; have_sm = true;   // seed; never filter the first sample
    } else if (a > 0.0f) {
        // Through wrap180 so the filter never takes the long way round at the +-180 seam.
        sm_yaw   = wrap180(sm_yaw   + a * wrap180(yaw   - sm_yaw));
        sm_pitch = sm_pitch + a * (pitch - sm_pitch);
    }

    *out_yaw   = sm_yaw;
    *out_pitch = sm_pitch;
}

void update() {
    g_aim_law_armed = false;
    const uint32_t tick = g_ticks.fetch_add(1);
    g_tick_now.store(tick, std::memory_order_relaxed);

    // REFLECTION PAUSE AFTER A FAULT -- keyed on a MEASURED fault, not on a guess about when.
    //
    // The previous version of this gate fired on a PlayerController change, on the theory that the
    // object array rebuilding during a level transition was the trigger. MEASURED: the faults land
    // ~40 s AFTER that window closes, so it guarded an empty room and cost a 1.5 s pause after
    // every transition for nothing. Removed 2026-09-06.
    //
    // What the evidence DOES show: once the first fault happens, every lane that touches reflection
    // faults in turn -- reticule_trace, then socket_sample, then navworld_tick. Per-lane cooldowns
    // therefore just move the fault along instead of stopping it. So one fault now pauses ALL
    // reflection briefly, which is the same cooldown scoped to the thing actually going wrong.
    // ABOVE THE PAUSE, AND ABOVE EVERY LANE THAT FAULTS. Reflection-free by construction, so it
    // is safe here: a cached class-level offset, an object-array liveness check, one masked
    // byte. The three reflection-based re-assert hosts all live BELOW this point -- two inside
    // update()'s body and one behind the aim pick -- so a paused tick skips all three, and a
    // tick that faults at an early lane (reticule_trace ~9300, socket_sample ~8600) never
    // reaches them either. This is the only host that survives both.
    //
    // It repairs within ONE frame rather than the same frame: a rebuild later in this tick
    // still wins until the next one. That is the accepted cost of being early enough to run
    // at all, and one frame is not what the player was reporting.
    halo::reticule_mode3_reassert_raw();

    if (tick < g_reflect_ok_at.load(std::memory_order_relaxed)) return;

    // Warm the optional-material cache OFF the gameplay path. Self-gating and one-shot: it fires at
    // the frontend (engine up, no mission), so the whole blocking discovery of the absent VREditor
    // material -- three full object-array find_uobject probe walks AND the synchronous pak scan,
    // ~500 ms of game thread between them mid-mission -- is paid before any widget quad is created
    // rather than at level start. Free every tick after it has run, and throttled to ~1 s per
    // attempt before then (its own gate probe is a full array walk while it is failing).
    // See reticule_prime_material_cache().
    reticule_prime_material_cache();

    // Per-tick perf accumulators. Cleared HERE rather than in on_pre_engine_tick because PERF_TICK's
    // own scope opens before this call and closes after it -- clearing outside would wipe the total
    // this tick is about to record. Unconditional and free: thirteen stores of a double.
    for (int i = 0; i < PERF_COUNT; ++i) g_perf_now[i] = 0.0;

    // Re-read the config about every 2 s at ~32 Hz. Cheap, and it is what makes `enabled=0` an
    // actual kill switch rather than a comment. Checked BEFORE the enabled test so the driver can
    // also be turned back ON from the file without a restart.
    if (tick - g_cfg_check_tick >= 64) {
        g_cfg_check_tick = tick;
        PerfScope _perf(PERF_CFG);
        // Settings-menu bridge FIRST, so a command applied this tick is parsed by the very same
        // load_config below -- the menu's changes land within one poll, like any file edit.
        int menu_applied = 0;
        {
            PerfScope _bridge(PERF_BRIDGE);
            menu_applied = menu_bridge_tick();
        }
        load_config();
        if (menu_applied > 0) {
            API::get()->log_info("[Halo-CampE-UEVR] settings menu: applied %d change(s) to halo_vr_user.cfg",
                                 menu_applied);
        }

        // Re-apply hotkey overrides on top of what was just parsed (see g_kill_override). Each is
        // released when the file's own value CHANGES, so an edit still beats a stale hotkey.
        {
            static int s_file_enabled = -1;
            const int file_enabled = g_cfg.enabled ? 1 : 0;
            if (s_file_enabled != file_enabled) { s_file_enabled = file_enabled; g_kill_override = -1; }
            const int ov = g_kill_override.load();
            if (ov >= 0) g_cfg.enabled = (ov != 0);
        }
        {
            // Aim mode only. The rig is deliberately NOT forced here -- see the toggle itself: rig
            // mode 3 is the fixed rig and belongs on both aim paths, so tying it to the aim mode
            // reverted the arms to the broken write every time you switched back to stick drive.
            static int s_file_direct = -1;
            const int file_direct = g_cfg.aim_direct ? 1 : 0;
            if (s_file_direct != file_direct) { s_file_direct = file_direct; g_mode_override = -1; }
            const int ov = g_mode_override.load();
            if (ov >= 0) g_cfg.aim_direct = (ov != 0);
        }
        // SEED the plant gain, ON CHANGE ONLY.
        //
        // On-change rather than every reload for two reasons: re-seeding on every ~2 s poll would
        // continuously wipe out whatever adaptation had learned, and applying an unchanged tunable
        // every tick is the habit this codebase already avoids. The first pass through here always
        // fires (the remembered value starts impossible), which is what puts a sane gain in place
        // before the first measurement instead of leaving feedforward dead until one arrives.
        //
        // Writing a NEW value deliberately RESETS the learned gain, so the startup transient can be
        // reproduced on demand rather than only once per launch.
        {
            static float s_seeded = -1.0f;
            const float seed = (g_cfg.gain_seed > 0.0f) ? g_cfg.gain_seed : REFERENCE_RATE_DPS;
            if (seed != s_seeded) {
                s_seeded = seed;
                g_meas_rate = seed;
                g_gain_logged = false;   // let the mismatch line report the re-converged value
            }
        }

        // The PIN comes after the seed so it always wins: measrate is "hold the plant still",
        // which is only meaningful if nothing else writes the gain afterwards.
        if (g_cfg.meas_rate_fixed > 0.0f) g_meas_rate = g_cfg.meas_rate_fixed;

        // Immediately after the reload, on the TICK thread: this is where the trace file actually
        // gets written, deliberately far away from the input path that fills the buffer.
        aim_trace_tick();
        nav_scan_tick();   // BEFORE mem_scan_tick: it composes the values and arms the edge
        mem_scan_tick();
        mem_diff_tick();
        aim_chain_scan_tick();   // AIMDIG: aimdig=1 with aimdirect Ready -> one chain-dig report
        // The SHIPPING aim write. Separate call from blam_aim_tick() below on purpose: that one is
        // the dev investigation and does not exist in a release build, while this one is the
        // feature. Ordered first so it owns the address unless the diagnostics explicitly claim it.
        blam_drive_tick();
        blam_aim_tick();
        aim_watch_tick();
        aim_direct_tick();
        game_settings_tick(g_stick_mode.load());
        // The culling fix (shipping code): re-applies the cull distance on value/level change
        // and a slow insurance timer. Poll cadence is exactly right for all three triggers.
        cull_fix_tick();
        // Dev builds only (inline no-ops otherwise): the on-change console-command harness, the
        // audio-object survey, and the navpoint/CHUD recon dump. Config-poll rate on purpose --
        // all are edge-triggered on their keys changing, never per-tick work.
        dev_exec_tick();
        audio_dump_tick();
        nav_dump_tick();
    }

    // PER-WEAPON DELTAS: every tick, and specifically here.
    //
    // After the reload block, because load_config() restores the calibrated values and would wipe
    // an adjustment applied before it. Before the rig reads grip/off further down this same
    // function, or it would use the unadjusted numbers.
    //
    // EVERY tick rather than inside the reload block: the config is only re-read about every two
    // seconds, and a weapon swap between reloads would otherwise keep the previous weapon's
    // adjustment for up to that long. weapon_offset_update() re-captures its base only when the
    // config tick changes, so calling it per tick cannot compound.
    { PerfScope _perf(PERF_WPNOFF); weapon_offset_update(); }
    // Immediately after the weapon trims and for the same reason: both read the weapon in hand and
    // write g_cfg, and both must land before anything downstream consumes those values. The scope
    // pane re-anchors itself when scope_dist/right/up change (Scope.cpp), so a weapon swap picks
    // the new trim up with no extra plumbing.
    { PerfScope _perf(PERF_WPNOFF); scope_offset_update(); }

    // Above every early-out below, so the numbers still arrive when the driver is disabled or
    // parked in a menu -- "it stutters at the frontend too" is a diagnosis, not a gap.
    perf_report(tick);
#if HALO_VR_DEV
    // FAST GEOM LINE. perf_report is throttled to 600 ticks (~15 s), far too coarse to catch a
    // player holding a pose for a few seconds -- a sweep briefed at 3 s per pose lands between
    // samples entirely. Dev builds only, and only while perf logging is already on.
    if (g_cfg.arm_driver == 2 && g_cfg.perf_log) {
        static uint32_t geom_last = 0;
        if (tick - geom_last >= 60) {
            geom_last = tick;
            API::get()->log_info("[Halo-CampE-UEVR] %s", halo::palettearm_status_geom());
            API::get()->log_info("[Halo-CampE-UEVR] %s", halo::palettearm_status_jitter());
        }
    }
#endif

    // THE COMPOSITOR RETICULE's game-thread half, for the same reason as the scope below: its
    // config mirror, its bring-up and above all its liveness watchdog must keep running on exactly
    // the ticks the aim stack early-outs on, or "the hook stopped firing" and "the aim path stopped
    // asking" become indistinguishable. Costs one bool test while the feature is off (default).
    // This is also what defines the watchdog's tick unit -- ~32 Hz, stated at the call site because
    // getting that wrong fails silently in the direction that looks fine.
    // BEFORE xrlayer_tick(), and the order is the whole point: xrlayer_tick() is what brings the
    // layer up and builds the atlas, and the atlas is never resized afterwards. The scope's own
    // tick runs from scope_frame_end() at the BOTTOM of update() -- too late, and at the main menu
    // update() returns before reaching it at all, so the pane never got a cell for the session.
    // Config-only, so it needs nothing that gameplay provides. See scopelayer_configure_cell_early.
    scopelayer_configure_cell_early();
    { PerfScope _perf(PERF_XRLAYER); xrlayer_tick(); }

    // Hide the in-scene crosshair ONLY while the compositor layer is proven live.
    //
    // xrlayer_live() is not "the feature is switched on" -- it is "our layer reached the compositor
    // within the watchdog window". That distinction is the point: gating on the config key alone
    // would hide the player's only crosshair the moment the layer failed for any reason, which is
    // precisely the outcome the whole module is built to avoid. On-change inside, so this is one
    // bool test per tick.
    //
    // ...BUT NOT ON A BARE xrlayer_live(), AND THAT IS THE FIX FOR A DOUBLE CROSSHAIR REPORTED
    // 2026-09-06 ("I still saw double after scoping out").
    //
    // xrlayer_live() is `possible && submitted > 0` -- it means "a quad of ours reached the runtime
    // in this window", NOT "our layer is attached". Those come apart the moment anything DELIBERATELY
    // stops submitting a quad, and xrlayerhidescope does exactly that: it retires the reticule quad
    // for the duration of the scope. So the chain ran:
    //
    //   hide the compositor reticule for the scope -> submitted drops to 0 -> xrlayer_live() false
    //     -> g_ws_scene_hidden false -> the WORLD reticule is restored to the main pass
    //     -> the scope ends, slot 0 re-arms, the compositor reticule returns -> TWO crosshairs,
    //        until the next window makes the layer live again and re-hides the world one.
    //
    // Hiding one reticule was un-hiding the other. The live log shows the flap directly: `restored`
    // at 18:26:08.756 followed by `HIDDEN` 17 ms later, and `restored` at 18:26:12.247 followed by
    // `HIDDEN` 518 ms later -- and a ~0.5 s double is long enough to see and to report.
    //
    // THE FIX IS A ONE-WAY LATCH, not a smarter gate -- see the block below. An earlier attempt
    // held the state through a deliberate hide and added a grace period on a drop; that removed the
    // observed flapping but kept the underlying idea that the world reticule may return to the main
    // view, which is the thing that has no justification in the first place.
    //
    {
        // LATCH ON FIRST LIVENESS, THEN STAY HIDDEN. The world reticule has no reason to EVER draw
        // in the main view -- the compositor quad is the main-view crosshair -- so anything that can
        // put it back there is a bug surface, not a feature. Gating it on the INSTANTANEOUS
        // xrlayer_live() did exactly that: that flag is `possible && submitted > 0`, i.e. "a quad
        // reached the runtime this window", so a quiet window, or our own deliberate scope-hide
        // retiring the reticule quad, read as a dead layer and handed the world reticule back.
        //
        // The safety property the original gate protected is real but narrower than it was written:
        // what must never happen is a player left with NO crosshair because our layer never worked.
        // That is answered by "has the layer EVER been live", not by "is it live this instant".
        //
        //   never live  -> the layer is not working here; show the world reticule, as before.
        //   ever live   -> the compositor is the crosshair; stay capture-only, permanently.
        //
        // One-way, so it cannot flap: no quiet frame, no scope-hide, and no watchdog blip can put
        // the reticule back once the layer has proven itself.
        static bool s_layer_ever_live = false;
        if (xrlayer_live()) s_layer_ever_live = true;

        reticule_widget_set_scene_hidden(g_cfg.xr_layer_hide_ws != 0 && s_layer_ever_live);
    }

    // MODE 3'S REPAIR, ON AN UNCONDITIONAL HOST. It used to live inside reticule_widget_move(),
    // which sounds per-tick and is not: that call sits several branches deep behind a SUCCESSFUL AIM
    // PICK (`if (g_cfg.aim_widget)` inside the on-foot and vehicle target blocks). Every tick whose
    // pick fails therefore skipped the repair entirely.
    //
    // That is precisely the window Config.hpp's xr_layer_hide_ws note describes and could not
    // explain -- "the flags are re-applied after the widget is re-hosted, so there is a window after
    // each re-host where the widget is back in the main pass", one session flapping 17 times. A
    // re-host hands us a NEW widget whose bVisibleInSceneCaptureOnly is false while g_ws_scene_hidden
    // is still true, and reticule_widget_set_scene_hidden is change-only on that latch, so it writes
    // nothing. Only the bit-compare repair closes the window -- and it was hosted on the one call
    // most likely to be skipped at exactly those moments, because a re-host and a failed pick have
    // the same causes (a scope transition, a weapon swap, a pawn rebuild).
    //
    // Reported from a live session 2026-09-06: exited the scope and the world reticule stayed
    // visible in the main view, which is mode 3's stated job to prevent.
    //
    // Cheap enough to be unconditional: it returns immediately unless hide_ws == 3, and then costs
    // one property lookup and a masked byte compare, writing ONLY when the bit disagrees.
    reticule_mode3_reassert();

    // STAGE 2's game-thread half: resolve the widget reticule's render target down to an
    // ID3D12Resource so the layer can present the game's OWN animated crosshair. Here rather than
    // inside xrlayer_tick() because it is useful on its own -- the walk is pure UE/D3D12 and can be
    // measured on the headless OpenVR/SimVR lane, where the compositor layer itself cannot exist.
    // Costs one bool test while both of its keys are off (default).
    // TIMED. It re-validates the whole UE -> FRHITexture -> ID3D12Resource chain EVERY tick once a
    // source is live (deliberately -- see XrSource.cpp), and it calls xrlayer_capture_source() from
    // inside, which submits a command list on the game's own D3D12 queue. Both are cheap in theory
    // and neither was measured, which is exactly the combination this project keeps getting caught
    // by. Now it is one line in the perf window.
    { PerfScope _perf(PERF_XRSRC); xrsource_tick(tick); }

    // THE SCOPE, also above every early-out: it must HIDE the pane on ticks where the aim stack
    // is parked (menus, seats, invalid pose -- the paths that return early below), and the
    // first placement at the end of the tick would never run on exactly those ticks. It consumes
    // the ray scope_notice_ray() deposited LAST tick, so pane placement trails the reticule by
    // one ~32 Hz tick -- invisible next to the smoothing already on the reticule itself.
#if HALO_VR_DEV
    // SCOPEDEV state line, ~2 s: every input the scope's gates consume, so a dead pane is
    // explainable from the log alone (the aim-servo `blocked(reason)` idea, applied here).
    if ((tick % 64) == 33) {
        API::get()->log_info("[Halo-CampE-UEVR] SCOPEDEV state: active=%d edges=%u ltmax=%u "
                             "captures=%u havevp=%d inmenu=%d stick=%d devray=%d scope=%d",
                             (int)g_scope_active.load(), g_scope_lt_edges.load(),
                             (unsigned)g_scope_lt_max.load(), g_scope_captures.load(),
                             (int)g_have_view_pos.load(),
                             (int)g_in_menu.load(), (int)g_stick_mode.load(),
                             (int)g_cfg.scope_dev_ray, (int)g_cfg.scope_enabled);
    }
    // SCOPE DEV RAY (scopedevray=1, dev cfg): synthesize the ray from the RENDERED VIEW instead of
    // the controller. Exists because the SimVR null driver never validates the controller aim
    // pose, so the on-foot reticule path -- the scope's real ray source -- stays parked and the
    // scope cannot be exercised by headless automation at all. View pos/yaw/pitch are published
    // by the stereo callback, which does run under the sim. Verification only; a release build
    // does not contain this path.
    if (g_cfg.scope_dev_ray && g_have_view_pos.load() && !g_in_menu.load()) {
        const Vec3 devo{g_view_pos_x.load(), g_view_pos_y.load(), g_view_pos_z.load()};
        const float vy = g_render_view_yaw.load() * DEG2RAD;
        const float vp = g_render_view_pitch.load() * DEG2RAD;
        const float cp = std::cos(vp);
        const Vec3 devt{devo.x + cp * std::cos(vy) * 500.0f,
                        devo.y + cp * std::sin(vy) * 500.0f,
                        devo.z + std::sin(vp) * 500.0f};
        auto* dev_pawn = API::get()->get_local_pawn(0);
        auto* dev_root = follow_object(reinterpret_cast<API::UObject*>(dev_pawn), L"RootComponent");
        if (dev_root != nullptr) scope_notice_ray(devo, devt, dev_root, tick);

        // TRACE FROM THE VIEW TOO, so the "what did the ray hit" readout is exercisable under
        // SimVR. The real trace lives in the on-foot reticule block, which never runs here: the
        // null driver never validates a controller pose, so the plugin sits in stick mode and
        // that whole path is skipped. Without this the identification work cannot be tested
        // headlessly at all. Throttled -- this is a full line trace, not a free read.
        if (hit_trace_ready() && (tick % 8) == 0) {
            API::UObject* ig[2] = {nullptr, nullptr};
            int nig = 0;
            if (dev_pawn != nullptr) ig[nig++] = reinterpret_cast<API::UObject*>(dev_pawn);
            if (auto* wa = fp_weapon_actor()) ig[nig++] = wa;
            // A SEPARATE, LONG end point. devt is only 500 cm out -- fine for defining a
            // direction, useless as a trace: looking level across open ground there is nothing
            // within five metres, so the trace correctly returned nothing and the readout looked
            // broken when it was simply out of range.
            const float reach = 20000.0f;   // 200 m
            const Vec3 devfar{devo.x + cp * std::cos(vy) * reach,
                              devo.y + cp * std::sin(vy) * reach,
                              devo.z + std::sin(vp) * reach};
            Vec3 devhit{};
            const bool got_dev = hit_trace(devo, devfar, ig, nig, &devhit);
            static uint32_t s_dev_trace_rep = 0;
            if ((s_dev_trace_rep++ % 40) == 0) {
                API::get()->log_info("[Halo-CampE-UEVR] HITDEV: trace %s (reach %.0fm)",
                                     got_dev ? "HIT" : "no hit", reach / 100.0f);
            }
            if (got_dev) hit_trace_dev_report();
        }
    }
#endif
    scope_frame_end(tick);

#if HALO_VR_DEV
    // One-shot: name the project's collision channels so the reticule trace channel can be chosen
    // deliberately. Runs once, late enough that the engine's config is loaded.
    {
        static bool dumped = false;
        if (!dumped && tick > 200) { dumped = true; hit_trace_dump_channels(); }
        // Same one-shot window: prove the shield getters answer against a live enemy.
        static bool probed = false;
        if (!probed && tick > 240) { probed = true; hit_trace_dev_shield_probe(); }
    }
#endif

    // A distance key arrived as a legacy METRES value and was scaled to cm. Said once, loudly: a
    // silent rescale is the same class of surprise as the unit mismatch it exists to prevent.
    {
        static bool said_units = false;
        if (!said_units && g_cfg.legacy_units_key != nullptr) {
            said_units = true;
            API::get()->log_info("[Halo-CampE-UEVR] UNITS: %s=%.2f looked like the old METRES value, "
                                 "so it was read as %.0f cm. Distance settings are CENTIMETRES now "
                                 "(matching Unreal) -- write %s=%.0f to silence this. Check the other "
                                 "distance keys in the same file while you are there.",
                                 g_cfg.legacy_units_key, g_cfg.legacy_units_val,
                                 g_cfg.legacy_units_val * 100.0f,
                                 g_cfg.legacy_units_key, g_cfg.legacy_units_val * 100.0f);
        }
    }

    // An aim offset with no aimcalibver stamp has an INFERRED schema, and inferring wrong is a
    // constant yaw error that is invisible until you compare where you point with where shots go --
    // and that then gets written back to disk the next time any calibration saves. Said once.
    {
        static bool said = false;
        if (!said && g_calib_stamp_ambiguous) {
            said = true;
            API::get()->log_info("[Halo-CampE-UEVR] CALIB: aimoffyaw/aimoffpitch loaded with no "
                                 "aimcalibver stamp -- reading them as ABSOLUTE (schema 1), so the "
                                 "view-lock frame is NOT applied. If aim sits off by a constant, run "
                                 "the Page Down calibration once: it re-measures against the current "
                                 "frame and stamps the result 2.");
        }
    }

    // ---- UEVR'S OWN ROOMSCALE RE-CENTRE, WHICH DOES THE SAME JOB AS THE LEASH.
    //
    // VR_RoomscaleMovement re-centres the standing origin to the HMD every frame, X and Z only
    // (FFakeStereoRenderingHook.cpp: `current_standing_origin.x = hmd_pos.x; ... // dont touch the
    // Y axis`). That IS a lateral leash at radius zero, and it defaults on -- so it was cancelling
    // lateral head movement before this plugin's leash existed, which is why the effect was noticed
    // as "already leashed somehow", why it was lateral-only, and why hmdleashlat had so little to
    // do once we added it.
    //
    // It also means hmdleash=0 WAS A LIE: switching ours off left UEVR's running, so the option we
    // want to offer people who find a tight leash nauseating did nothing. The setting has to own
    // both or it is not a setting.
    //
    // Note what UEVR's version is actually FOR: the line above the re-centre moves the pawn by the
    // head offset, so you walk in-game. That half does not work on this title -- the Blam sim owns
    // the player's position and overwrites the UE actor -- so all that survives is the re-centre.
    // We lose nothing by taking it over, and we gain a radius, which UEVR's has no notion of.
    //
    // Written on CHANGE only. It is a UEVR mod value, not a config poll, and rewriting it every
    // tick would be a per-tick engine call for a value that changes when the player edits a file.
    {
        static int last_want = -1;
        static uint32_t last_assert = 0;
        const int want = g_cfg.hmd_leash ? 1 : 0;

        // RE-ASSERTED, not written once. This used to fire only when `want` CHANGED -- i.e. only
        // when the player edited hmdleash -- and `last_want` is a function-local static, so it
        // survives level loads. Anything on UEVR's side that restores its mod values from
        // config.txt (which ships VR_RoomscaleMovement=true) would therefore leave BOTH leashes
        // running: ours, plus UEVR's zero-radius lateral re-centre. Lateral head movement
        // cancelled twice reads as lost positional tracking, and the 2026-08-24 log shows this
        // line firing exactly once at session start across five subsequent level loads.
        //
        // Every 600 ticks is ~10 s. The original comment warned against a per-tick engine call and
        // that still holds; this is six writes a minute, which is nothing, and it makes the
        // override true rather than merely once-true.
        const bool due = (want != last_want) || (tick - last_assert) >= 600;
        if (due) {
            last_want = want;
            last_assert = tick;
            API::VR::set_mod_value("VR_RoomscaleMovement", false);
            static int logged_want = -2;
            if (logged_want != want) {           // the 10 s re-assert must not spam the log
                logged_want = want;
                API::get()->log_info("[Halo-CampE-UEVR] LEASH: VR_RoomscaleMovement forced OFF -- it is a "
                                     "zero-radius lateral leash of UEVR's own, and hmdleash owns this "
                                     "behaviour now (hmdleash=%d, lat=%.2f vert=%.2f); re-asserted "
                                     "every 600 ticks so a level load cannot restore UEVR's",
                                     want, g_cfg.hmd_leash_lat * 100.0f, g_cfg.hmd_leash_vert * 100.0f);
            }
        }
    }

    // ---- HMD TRANSLATION LEASH (doctrine in Config.hpp).
    //
    // Slide the standing origin to absorb any head displacement past the radius. Inside it, nothing
    // happens and roomscale is untouched; outside, the origin follows you, so the divergence between
    // your eye and the game camera is bounded by the radius rather than by your room.
    //
    // On the GAME THREAD at ~32 Hz rather than per frame: the slide only runs while the player is
    // actively pushing the boundary, and its rate is their own walking speed, so a 32 Hz correction
    // is smooth. Doing it in the stereo callback would mean an engine call per eye per frame for a
    // value that changes at human speed.
    //
    // Above the early-outs, because the divergence accrues whether or not the aim stack is armed.
    if (g_cfg.hmd_leash) {
        Vec3 hp{}; Quat hq{};
        const auto hi = API::VR::get_hmd_index();
        if (hi >= 0 && get_pose(hi, &hp, &hq, /*use_aim=*/false)) {
            const auto so = API::VR::get_standing_origin();
            // VR room space: Y is up (the VR->UE conversion elsewhere maps VR y to UE z), so the
            // lateral pair is X/Z and vertical is Y.
            const float dx = hp.x - so.x, dy = hp.y - so.y, dz = hp.z - so.z;
            const float lat = std::sqrt(dx * dx + dz * dz);

            float nx = so.x, ny = so.y, nz = so.z;
            bool moved = false;

            if (lat > g_cfg.hmd_leash_lat && lat > 1e-4f) {
                // Absorb only the EXCESS, so the player keeps the full radius of free movement
                // rather than being dragged to the centre.
                const float k = (lat - g_cfg.hmd_leash_lat) / lat;
                nx += dx * k; nz += dz * k; moved = true;
            }
            const float adz = (dy < 0.0f) ? -dy : dy;
            if (adz > g_cfg.hmd_leash_vert) {
                ny += dy - ((dy > 0.0f) ? g_cfg.hmd_leash_vert : -g_cfg.hmd_leash_vert);
                moved = true;
            }

            if (moved) {
                const UEVR_Vector3f n{nx, ny, nz};
                API::VR::set_standing_origin(n);
#if HALO_VR_DEV
                static uint32_t last = 0;
                if (tick - last >= 60) {
                    last = tick;
                    API::get()->log_info("[Halo-CampE-UEVR] HMDLEASH: absorbed drift lat=%.1fcm vert=%.1fcm "
                                         "(limits %.0f/%.0f cm) origin -> (%.3f,%.3f,%.3f)",
                                         lat * 100.0f, dy * 100.0f, g_cfg.hmd_leash_lat * 100.0f, g_cfg.hmd_leash_vert * 100.0f, nx, ny, nz);
                }
#endif
            }
        }
    }

    // ---- Calibrate key, edge-detected on the GAME THREAD.
    // GetAsyncKeyState reads global key state, so it registers with the headset on and the game
    // focused -- which is the only way this is usable mid-session.
    //
    // FOREGROUND-GATED (2026-08-15). "Global" cuts the other way too: the keys registered while
    // the game sat in the BACKGROUND, and End (end of line), Page Down (scrolling), Ctrl+Home
    // (top of document) and Ctrl+PgUp (previous tab) are keys a desktop session presses
    // constantly -- the log showed four garbage aim calibrations in ten seconds of desktop
    // work, which is the calibbtn lesson below re-imported through the keyboard. Every gesture
    // in this block now requires game_window_focused(); headset play is unaffected because a
    // played game IS the foreground window.
    //
    // KEYBOARD ONLY, AND DELIBERATELY SO.
    //
    // There was a gamepad binding here (calibbtn, default BACK/View). It has been removed, not
    // merely defaulted off. On Touch controllers the MENU button commonly lands on BACK through
    // UEVR, so opening the pause menu performed a full pose-match calibration and wrote the result
    // to disk. That is not a mis-set default; it is a gesture that mutates persistent state bound
    // to a control players press for an unrelated reason, and it produced a whole day of
    // calibrations that "changed on their own" and A/B tests whose baseline moved under them.
    //
    // A keyboard key is the right shape for this: distinct, deliberate, impossible to hit while
    // playing, and not reachable by any controller remap. The eventual home for the gesture is a
    // UEVR Lua UI, which is deliberate by construction -- and that is another reason not to keep a
    // hidden binding around in the meantime.
    {
        // ---- MENU-ARMED CALIBRATION -- the "UEVR Lua UI" home the removal note above always
        // anticipated. The settings menu arms a mode (bridge command); an armed mode behaves
        // exactly like the calibration key being HELD, and the triggers (sampled + eaten in
        // on_xinput_get_state) supply the release edges:
        //   RIGHT trigger: drop the mode -> the hold falls -> the existing release edge solves
        //                  and saves, once, and the gesture is over.
        //   LEFT trigger:  interrupt the hold only while pressed -> its press edge solves and
        //                  saves, its release re-enters the hold -- consecutive calibrations
        //                  with no menu round-trip.
        {
            static int s_prev_mode = 0;
            int mode = g_menu_calib_mode.load(std::memory_order_relaxed);
            if (mode != 0 && g_menu_calib_rt.load(std::memory_order_relaxed)) {
                g_menu_calib_mode.store(0, std::memory_order_relaxed);
                mode = 0;
            }
            if (mode != s_prev_mode) {
                if (mode != 0) {
                    API::get()->log_info("[Halo-CampE-UEVR] MENU CALIBRATION ARMED (%s): close the UEVR menu "
                                         "(controllers do not reach the game while it is open), align, then "
                                         "RIGHT trigger = save & finish, LEFT trigger = save & re-arm.",
                                         mode == 1 ? "weapon pose" :
                                         mode == 2 ? "aim ray" : "support hand");
                } else {
                    API::get()->log_info("[Halo-CampE-UEVR] MENU CALIBRATION finished/disarmed.");
                }
                s_prev_mode = mode;
            }
        }
        const int  menu_mode = g_menu_calib_mode.load(std::memory_order_relaxed);
        const bool menu_lt   = g_menu_calib_lt.load(std::memory_order_relaxed);
        // The gate for every keyboard gesture below. The menu-armed path is deliberately NOT
        // gated: arming is an explicit act, and its triggers only arrive through the game's own
        // input in the first place.
        const bool key_focus = game_window_focused();


        // The per-weapon capture key drives the IDENTICAL gesture -- freeze, align, release,
        // solve -- and differs only in where the answer is stored. Polled here so both edge
        // detections sit on one thread, which is the reason the comment above gives for not
        // doing this in the XInput hook.
        //
        // Behind key_focus for the same reason the keyboard gestures below are: GetAsyncKeyState
        // is global, so without it a capture latches from a keypress in another window.
        if (key_focus) wpn_calib_poll();
        const bool held = (key_focus && (g_cfg.calib_key != 0) &&
                           ((GetAsyncKeyState(g_cfg.calib_key) & 0x8000) != 0)) ||
                          (menu_mode == 1 && !menu_lt) ||
                          wpn_calib_held();
        const bool was  = g_calib_held.exchange(held);
        // Mirror it for other translation units -- see calib_hold_active() in WeaponCalib.hpp.
        calib_hold_publish(held);
        if (held && !was) g_calib_start  = true;
        if (!held && was) g_calib_finish = true;

        // ---- THE SUPPORT-HAND GESTURE'S HOLD. Published rather than edge-detected here, because
        // its consumer lives in another translation unit (src\palettearm\PaletteArm.cpp, which owns
        // both the freeze and the solve) while the left-trigger "save & re-arm" half is only
        // visible from this one -- g_menu_calib_lt is TU-local, sampled inside the XInput hook.
        //
        // NO KEYBOARD SOURCE, on purpose: the player asked for a menu entry rather than a fourth
        // hotkey, and the nav cluster is already fully spoken for (End, Page Down, Insert, Delete,
        // Ctrl+Home, Ctrl+Page Up). Nothing here reads a key, so nothing here needs key_focus.
        //
        // The consumer re-checks `mode == 3` itself -- see the note on the declaration. This value
        // alone is not the hold; the mode is the half that a right trigger or Cancel can clear.
        halo::g_hand_calib_held.store(menu_mode == 3 && !menu_lt, std::memory_order_relaxed);

        const bool aim_down = (key_focus && (g_cfg.aim_calib_key != 0) &&
                               ((GetAsyncKeyState(g_cfg.aim_calib_key) & 0x8000) != 0)) ||
                              (menu_mode == 2 && !menu_lt);
        const bool aim_was  = g_aimcal_held.exchange(aim_down);
        // Publish it: blamangles drives the sim's angular control state from a sim-thread hook in
        // BlamAim.cpp, which cannot see this file's anonymous namespace. Without this the actuator
        // goes silent as designed but the control-record write keeps going, so the reticle never
        // freezes and the calibration is impossible to perform.
        halo::g_aim_calibrating.store(aim_down, std::memory_order_relaxed);
        if (aim_down && !aim_was) {
            g_aimcal_start = true;

            // CALIBRATE FROM YOUR NEUTRAL POSITION -- i.e. with your head at the standing origin.
            //
            // This is a measurement procedure, not a nag. Convergence bends the command by
            // roughly (eye-to-camera offset / range to target), and calibrating inside that bend
            // means measuring the hand-to-aim offset through a lens that is itself moving:
            // MEASURED over four calibrations 45 s apart while walked away from the pawn, the bend
            // at capture was +1.8, -17.7, -19.0 degrees, and then did not engage at all.
            //
            // The capture removes the bend, and that works -- those four produced offsets agreeing
            // to ~2 degrees. What it cannot remove is the bend CHANGING between the capture and the
            // next drive tick, which is the residual snap. At the standing origin the bend is
            // identically zero, so there is nothing to remove and nothing to change: the gesture
            // becomes exact, and the offset it stores is the clean one that stays valid at every
            // displacement afterwards.
            //
            // Warn, do not refuse. A player who cannot conveniently recentre is better served by a
            // slightly noisy calibration than by a gesture that will not run.
            {
                constexpr float NEUTRAL_WARN_CM = 10.0f;
                Vec3 dv{};
                const float dcm = halo::aim_converge_delta(&dv)
                    ? std::sqrt(dv.x * dv.x + dv.y * dv.y + dv.z * dv.z) : 0.0f;
                if (dcm > NEUTRAL_WARN_CM) {
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] AIM CALIBRATE: WARNING -- your head is %.0f cm from the standing "
                        "origin. Aim will settle for a moment after you release, and the fit is "
                        "noisier than it needs to be. RESET YOUR PLAY AREA (or set hmdleash=1) and "
                        "calibrate from your neutral position for an exact result.", dcm);
                }
            }

            API::get()->log_info("[Halo-CampE-UEVR] AIM CALIBRATE: aim frozen -- point your controller at the "
                                 "reticle, then release");
        }
        if (!aim_down && aim_was) {
            g_aimcal_finish = true;
            // SNAPSHOT THE POSE AT THE RELEASE EDGE.
            //
            // The capture path re-reads the controller on the NEXT tick, so the reference is bound
            // to wherever the hand was ~30 ms after the key came up -- and the player has already
            // started moving by then, because releasing a key IS a hand movement. That lands as a
            // calibration error in exactly the gesture whose whole purpose is precision.
            //
            // This runs in the key-polling path, which is far faster than the tick, so the sample
            // is taken essentially at the instant of release. derive_ctrl_angles is explicitly safe
            // here: it only touches UEVR API reads and atomics.
            const auto cal_ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                                      : API::VR::get_right_controller_index();
            float scy = 0.0f, scp = 0.0f;
            // RAW: allow_two_hand=false. This snapshot becomes a PERSISTED calibration offset,
            // and a hold live at the release edge would bake the blend into every future
            // one-handed session. See MotionAimControl.hpp.
            if (halo::derive_ctrl_angles(&scy, &scp, cal_ridx, /*allow_two_hand=*/false)) {
                g_aimcal_snap_yaw   = scy;
                g_aimcal_snap_pitch = scp;
                g_aimcal_have_snap  = true;
            }
        }

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
            const bool kill_down = key_focus && ctrl && ((GetAsyncKeyState(g_cfg.kill_key) & 0x8000) != 0);
            const bool kill_was  = g_kill_held.exchange(kill_down);
            if (kill_down && !kill_was) {
                g_cfg.enabled = !g_cfg.enabled;
                g_kill_override = g_cfg.enabled ? 1 : 0;   // or the next config poll undoes it
                API::get()->log_info(g_cfg.enabled
                    ? "[Halo-CampE-UEVR] KILL SWITCH: driver RE-ENABLED (aim follows your controller again)"
                    : "[Halo-CampE-UEVR] KILL SWITCH: driver DISABLED -- stick neutral, game plays stock. "
                      "Press again to re-enable.");
            }
        }

        // ---- AIM MODE TOGGLE: Ctrl+mode_key swaps aim actuation <-> direct drive. Same nav cluster
        // as the kill and calibration keys, so all four are findable by feel in a headset.
        //
        // IT NO LONGER TOUCHES THE RIG. It used to force rigmode 3 with direct and 2 with actuation,
        // which was written before we knew what the rig modes actually were: mode 3 is not the
        // direct-drive rig, it is the FIXED rig -- the relative-rotation write mode 2 uses is
        // measurably discarded by this game's first-person mesh, so mode 2's arms barely rotate at
        // all. Coupling them meant switching back to stick drive silently reverted the arms to the
        // broken path. The two settings are independent; the toggle changes one thing.
        if (g_cfg.mode_key != 0) {
            const bool ctrl = (GetAsyncKeyState(VK_CONTROL) & 0x8000) != 0;
            const bool down = key_focus && ctrl && ((GetAsyncKeyState(g_cfg.mode_key) & 0x8000) != 0);
            const bool was  = g_mode_held.exchange(down);
            if (down && !was) {
                const bool to_direct = !g_cfg.aim_direct;
                g_cfg.aim_direct = to_direct;
                g_mode_override  = to_direct ? 1 : 0;
                API::get()->log_info(to_direct
                    ? "[Halo-CampE-UEVR] AIM MODE: DIRECT DRIVE (aim assigned - responsive, but the sim "
                      "still fires along its own aim; rig unchanged at rigmode %d)"
                    : "[Halo-CampE-UEVR] AIM MODE: ACTUATION (aim steered via stick - projectiles correct; "
                      "rig unchanged at rigmode %d)", g_cfg.rig_mode);
            }
        }
    }

    // Release: rebind "controller pointing here" to "game aiming there". Clearing the reference is
    // the whole operation -- the existing capture path re-reads both on the next tick.
    if (g_aimcal_finish.exchange(false)) {
        g_have_ref = false;
        g_aimcal_capture = true;   // next capture MEASURES the offset instead of restoring it
    }

    // ---- VIEW-CONSUMER FIXES (audio listener, navpoint layer). Sited ABOVE every early-out --
    // including the kill switch -- because both must be able to RELEASE when their gates drop:
    // update() returning early with an override still applied is the stale-correction bug in a
    // new coat. Gated on g_cfg.enabled inside `fixes_ok`, so enabled=0 means "everything back to
    // vanilla", corrections included.
    //
    // The ControlRotation read is repeated here rather than hoisted from below because the read
    // below sits UNDER the frontend bail (at the frontend there is no Blam gameplay camera). The
    // same frontend test guards this one; the read itself fails closed on garbage regardless.
    {
        const bool want_any = g_cfg.audio_fix || g_cfg.audio_comp || g_cfg.nav_fix
                           || g_nav_shift_applied;   // releases still pending count as wanted
        double vf_pitch = 0.0, vf_yaw = 0.0;
        bool   have_aim = false;
        if (want_any) {
            auto* pc0 = API::get()->get_player_controller(0);
            const std::wstring pcn = (pc0 != nullptr)
                ? class_name_of(reinterpret_cast<API::UObject*>(pc0)) : std::wstring();
            const bool frontend = pcn.find(L"Frontend") != std::wstring::npos;
            if (pc0 != nullptr && !frontend) {
                void* vf_pc = nullptr;
                have_aim = read_control_rotation(&vf_pitch, &vf_yaw, &vf_pc);
            }
        }
        const bool fixes_ok = g_cfg.enabled && g_cfg.view_lock && !g_stick_mode.load()
                           && have_aim && g_have_render_yaw.load();
        audio_fix_tick(g_cfg.audio_fix && fixes_ok && g_have_view_pos.load(),
                       g_view_pos_x.load(), g_view_pos_y.load(), g_view_pos_z.load(),
                       g_render_view_yaw.load(), g_render_view_pitch.load());
        audio_comp_tick(g_cfg.audio_comp && fixes_ok && g_have_view_pos.load(), tick,
                        g_view_pos_x.load(), g_view_pos_y.load(), g_view_pos_z.load(),
                        g_render_view_yaw.load(), g_render_view_pitch.load());
        // navworld supersedes the flat-layer shift: while it owns the job, hud_navpoint_follow
        // sees "disengaged" and releases, so the flat markers stay game-native under the
        // world-space ones.
        hud_navpoint_follow(fixes_ok && !g_cfg.nav_world, vf_pitch, vf_yaw, tick);
        // HONOUR THE BACK-OFF THE FAULT FILTER ALREADY WROTE FOR THIS LANE.
        //
        // report_tick_fault has always recorded a fault against its lane and written an escalating
        // g_lane_retry_at for it. Nothing here ever read that, so navworld kept walking back into
        // the same faulting call -- and because a fault propagates out of on_pre_engine_tick, it
        // takes EVERY LANE BELOW IT with it: the rig, arms, hands, palette arm, two-hand and
        // gestures all run after this line and simply do not execute on a faulting tick. Aim
        // survives only because it rides the XInput hook's separate dispatch, which is why the
        // player's report is "the arms stopped tracking but I could still shoot".
        //
        // MEASURED 2026-09-08, release playthrough: 42 aborted ticks in 32 minutes, in bursts, all
        // 8 reports (the cap) identical -- 0xC0000005 reading 0x40400018 at +0x36FD8A6, lane
        // navworld_tick. The same shape as the 9,625-fault reticule_trace episode this filter was
        // built for; the mechanism was already there and simply unread on this lane.
        //
        // A COOLDOWN, NOT A KILL. The escalation (96*n ticks, capped at 1024) parks a genuinely
        // broken lane without ever needing a "disable forever" rule, and lets a lane that faulted
        // once on a transition come back on its own. Markers are cosmetic; the hands are not.
        if (!lane_cooling(PERF_NAVWORLD, tick)) {
            PerfScope _perf(PERF_NAVWORLD);
            nav_world_tick(fixes_ok, tick);
        }
        // CLEAR THE STEP MARKER ON THE WAY OUT. Without this a fault anywhere later in the tick
        // would report navworld's last engine call and read as damning evidence about a lane it
        // had already left -- which is precisely the trap PerfScope's missing restore set for the
        // two previous rounds of this investigation.
        NAVW_MARK(nullptr);
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
        // Gated on gameplay, NOT on the rig. The scan finds the HUD's reticle widgets, and it used
        // to wait for a first-person weapon -- so spawning straight into a vehicle (no weapon, ever)
        // meant the widgets were never found and the seated reticule had nothing to host. Same
        // stale proxy as the menu-state bug above, same fix.
        if (!frontend) reticle_rescan(tick);
        if (!frontend) navw_precertify_tick();
        material_hunt(tick);
        shield_fx_census(tick);
        run_mat_dump();
        texture_param_hunt(tick);

        // NOT `no_rig`. It was here as a cheap stand-in for "a load or transition, so nothing the
        // player presses matters" -- but it also means "no first-person weapon", which is exactly
        // a VEHICLE SEAT, and that made every menu binding fire during a ride.
        //
        // It only bites when the rig pointer is genuinely null while seated. BOARDING from on foot
        // leaves it stale-non-null (nothing clears it short of a PlayerController change), which is
        // why this survived testing; LOADING A CHECKPOINT THAT SPAWNS YOU IN A VEHICLE does not --
        // the PC change clears the pointer, and with no first-person weapon to walk back from it
        // never re-resolves. The whole ride then ran with UI bindings: right-controller B rewritten
        // to "back" and every gameplay remap stood down. Field-reported 2026-08-02; the player had
        // to reach for a keyboard to get out of the vehicle.
        //
        // The two remaining terms are the real signals and cover what no_rig was standing in for:
        // `frontend` is the PlayerController class, and `g_menu_widget_open` is the game's own
        // UI-manager state (verified: pause menu closed -> false, open -> true).
        const bool now_menu = frontend || g_menu_widget_open.load();
        const bool was_menu = g_in_menu.exchange(now_menu);
        // Published every tick rather than on the edge, for the same reason as stick mode below:
        // the BlamDrive watchdog reads it from outside this file's anonymous namespace, and a
        // publication that only fires on transitions is one missed tick from misjudging the state.
        halo::g_menu_active.store(now_menu, std::memory_order_relaxed);
        // The narrower one, published alongside: `frontend` WITHOUT the widget term. See the header
        // -- the sim keeps running (and keeps calling the hook) while a menu widget is open, so the
        // two questions have different answers and the wrong one silences a watchdog.
        halo::g_frontend_active.store(frontend, std::memory_order_relaxed);
        // Read-only PlayerController for off-thread READERS (the BlamDrive layout guard). Published
        // here, above every early-out, precisely because it must survive the motion stack standing
        // down -- unlike g_aim_law_pc further below, which is deliberately armed-only. Cleared at
        // the frontend so nobody reads a menu controller as if it were gameplay.
        halo::g_read_only_pc.store(frontend ? nullptr : pc0, std::memory_order_relaxed);
        if (was_menu != now_menu) {
            API::get()->log_info("[Halo-CampE-UEVR] IN_MENU %d -> %d (frontend=%d widget=%d norig=%d) pc=%s",
                                 (int)was_menu, (int)now_menu, (int)frontend,
                                 (int)g_menu_widget_open.load(), (int)no_rig, narrow(pcn).c_str());
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

            // ROUTE-DEATH BURST: the enter debounce only exists to tell a SEAT from a weapon
            // SWAP, and a swap's new actor is findable within a couple hundred ms -- but only if
            // someone looks. Resolving immediately (and fast for ~1.2 s) collapses that ambiguity
            // window, which is what lets stick_on_s default to 1 s instead of 3.
            static bool prev_route = true;
            if (prev_route && !route_alive) {
                g_rig_fast_until   = tick + 40;
                g_rig_resolve_tick = 0;      // resolve on this very tick's rig block
            }
            prev_route = route_alive;

            // ---- ON FOOT WITH NO WEAPON -- the one case "no weapon" must NOT mean "give the
            // camera back". The campaign opens unarmed, and without this exception those first
            // minutes ran as flat gamepad with motion aim and turning dead.
            //
            // Both terms must AGREE before the exception applies, and each fails closed on its own:
            //   * the game says it is presenting FIRST PERSON  (fp_presentation_state; -1 unknown)
            //   * the arms rig is live on the local pawn       (rig_tracked_component)
            // The rig term is what keeps the POST-LOAD window behaving as before: it reads 0 there
            // (nothing resolved yet), so stick mode still engages during a load exactly as designed.
            //
            // A CUTSCENE IS EXCLUDED OUTRIGHT. A scene can leave the player standing weaponless in
            // first person, which no signal here can tell from the unarmed opening -- but the game
            // answers that question directly, so ask it rather than infer. Only honoured while the
            // subsystem is actually answering; on a build without it the term is inert and the
            // camera heuristic downstream carries cutscenes exactly as before.
            //
            // It cannot fire in a vehicle unless the perspective byte reads first-person while
            // seated -- the one thing here that has not been measured on a real ride, and the
            // reason `stickonfoot` exists as an off switch.
            const int  persp   = fp_presentation_state(route_alive);
            const bool in_cine = g_cine_answering && g_cine_active;
            const bool on_foot = g_cfg.stick_onfoot && !route_alive && persp == 1 && !in_cine
                                 && rig_tracked_component() != nullptr;

            // Either route is "the player is driving in first person", which is what turning wants.
            g_fp_control_now   = route_alive || on_foot;
            g_on_foot_unarmed  = on_foot;   // the rig block hides the arms on this

            // A successful suppression produces NO stick-mode transition at all -- which is the
            // point, and also means the log would be silent about the very state this exists for.
            // So log the edge itself, and the raw byte with it: the unarmed opening and a vehicle
            // ride are then directly comparable in one file.
            {
                static int prev_on_foot = -1;
                if ((int)on_foot != prev_on_foot) {
                    prev_on_foot = (int)on_foot;
                    API::get()->log_info("[Halo-CampE-UEVR] ON-FOOT (no weapon) %s -- persp=%d "
                                         "rigcomp=%d. %s",
                                         on_foot ? "ENGAGED" : "cleared",
                                         g_dbg_persp.load(), (int)rig_component_alive(),
                                         on_foot ? "motion aim and turning stay with the player"
                                                 : "stick mode decides normally");
                }
            }

            // DISMOUNT WATCHER: the rig component survives a ride (R1-measured), and the new
            // weapon actor re-attaches to it the moment the game gives the weapon back. Its
            // AttachChildren count growing is therefore the "input is yours again" edge, for the
            // price of one guarded memory read per tick -- and it triggers a single immediate
            // resolve instead of waiting out the poll, so exit lands in ~0.3 s.
            static int32_t prev_children = -1;
            if (g_stick_mode.load()) {
                int32_t n = -1;
                if (auto* rigc = rig_tracked_component()) {
                    struct FRawArray { void* data; int32_t num; int32_t max; };
                    auto* arr = rigc->get_property_data<FRawArray>(L"AttachChildren");
                    if (arr != nullptr && !IsBadReadPtr(arr, sizeof(FRawArray))) n = arr->num;
                }
                if (n >= 0 && prev_children >= 0 && n > prev_children) {
                    g_rig_fast_until   = tick + 16;
                    g_rig_resolve_tick = 0;
                }
                prev_children = n;
            } else {
                prev_children = -1;
            }

            auto* pawn = API::get()->get_local_pawn(0);
            auto* base = g_stick_pawn_base.get_checked(L"Pawn");
            if (route_alive && pawn != nullptr && base != pawn) {
                g_stick_pawn_base.set(pawn);   // (re)baseline while the FP weapon is live
                base = pawn;
            }
            const bool pawn_match = (pawn != nullptr) && (pawn == base);

            const bool signal = g_cfg.stick_mode && gameplay && !route_alive && !on_foot;

            // Debounce, in ~32 Hz ticks. Enter is slow on purpose: a weapon swap kills the route
            // for up to the resolve cadence (~2 s), and flapping the camera mode mid-fight is
            // worse than a late vehicle transition. Exit is quick -- the route reviving IS a
            // successful resolve, which is already debounce enough.
            //
            // This is now the FALLBACK path, not the usual one: when the perspective byte answers
            // it short-circuits enter entirely (below), and the debounce only carries builds or
            // states where it does not.
            static uint32_t on_streak = 0, off_streak = 0;
            const uint32_t need_on  = (uint32_t)(clampf(g_cfg.stick_on_s,  0.1f, 30.0f) * 32.0f);
            const uint32_t need_off = (uint32_t)(clampf(g_cfg.stick_off_s, 0.03f, 30.0f) * 32.0f);

            bool want = g_stick_mode.load();
            if (signal) { off_streak = 0; if (++on_streak  >= need_on)  want = true;  }
            else        { on_streak  = 0; if (++off_streak >= need_off) want = false; }

            // PERSPECTIVE FLIP COLLAPSES THE ENTER DEBOUNCE TO ZERO -- the use the FA session that
            // found this field recommended, now that the same read is here for the on-foot case.
            //
            // The debounce exists ONLY to tell a vehicle seat from a weapon swap, and it costs
            // 0.75 s of hand-driven vehicle camera on every boarding. Weapon-absence is ambiguous;
            // a perspective flip is not -- a swap does not change how the game frames the player.
            // So when the byte positively says "no longer first person", stop waiting.
            //
            // Enter only. Exit is already near-instant via the dismount watcher, and letting this
            // force an EXIT would hand the camera back mid-ride the moment the byte twitched.
            if (signal && persp == 0) want = true;

            // Force bypasses the detector outright -- the A/B lever, and the manual fallback if
            // some vehicle seat keeps its FP weapon alive and the detector misses.
            if      (!g_cfg.stick_mode || g_cfg.stick_force == 2) want = false;
            else if (g_cfg.stick_force == 1)                      want = true;

            const bool was = g_stick_mode.exchange(want);
            // Published every tick, not just on the edge: the blamangles write reads it from a
            // sim-thread hook that has no other view of this file's anonymous namespace, and a
            // publication that only happens on transitions is one missed tick away from silently
            // leaving the write armed for a whole ride.
            halo::g_stick_mode_active.store(want, std::memory_order_relaxed);
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
                // g_turn_offset is not touched here. On exit, the stereo hook's re-prime folds
                // the ride's net camera rotation into it (see on_pre_calculate_stereo_view_offset)
                // -- the snap-turn path, which every room->game anchor already consumes. The
                // reference restore below is offset-based (desired aim == ctrl + saved offset), so
                // it is correct whichever side of the re-prime it lands on.
                // persp/onfoot are here for one reason: ONE vehicle ride settles whether the
                // perspective byte actually distinguishes a seat from standing unarmed. An ENTER
                // line reading `persp=<the first-person value> onfoot=0` while genuinely seated
                // means the byte does NOT change on boarding and `stickonfoot` must go to 0.
                API::get()->log_info("[Halo-CampE-UEVR] STICK MODE %s (route=%d rigcomp=%d pawnmatch=%d "
                                     "gameplay=%d force=%d persp=%d onfoot=%d) -- %s",
                                     want ? "ENTER" : "EXIT",
                                     (int)route_alive, (int)rig_component_alive(), (int)pawn_match,
                                     (int)gameplay, g_cfg.stick_force,
                                     g_dbg_persp.load(), (int)on_foot,
                                     want ? "sticks pass through, the game camera owns the view"
                                          : "motion aim re-anchoring");
            }
        }

        // ---- VEHICLE HARD BRAKE: either grip -> the brake key, while stick mode is engaged.
        // Sits ABOVE the early-out gates on purpose: every path that stops this function must
        // release the key first, or a pose/HMD loss mid-brake would leave Ctrl logically stuck.
        // (A hard crash mid-brake can still strand the OS key state -- one real Ctrl press clears
        // it -- but no code path here can.)
        {
            static bool brake_down = false;
            bool want_brake = false;
            // tick > 120: same guard as the VR-state logger -- VR API calls during injection,
            // before the runtime is up, can crash the game.
            if (g_cfg.brake_enabled && tick > 120 && g_stick_mode.load() && !g_in_menu.load()) {
                static UEVR_ActionHandle grip_action = nullptr;
                if (grip_action == nullptr) {
                    grip_action = API::VR::get_action_handle("/actions/default/in/Grip");
                }
                if (grip_action != nullptr) {
                    want_brake = API::VR::is_action_active_any_joystick(grip_action);
                }
            }
            // Pad-side delivery is the default: field testing showed synthesized keyboard never
            // reaches this game's driving input (see Config.hpp). The hook does the actual write.
            g_brake_pad = want_brake && (g_cfg.brake_mode == 2 || g_cfg.brake_mode == 3);

            if (want_brake != brake_down) {
                brake_down = want_brake;
                if (g_cfg.brake_mode == 1) {
                    // Keyboard lane, kept for reference: a SCANCODE event, the form raw-input
                    // readers accept. Proven NOT to reach this game's driving input.
                    INPUT in{};
                    in.type = INPUT_KEYBOARD;
                    in.ki.wScan   = (WORD)MapVirtualKeyW((UINT)g_cfg.brake_key, MAPVK_VK_TO_VSC);
                    in.ki.dwFlags = KEYEVENTF_SCANCODE | (want_brake ? 0 : KEYEVENTF_KEYUP);
                    SendInput(1, &in, sizeof(INPUT));
                }
                API::get()->log_info("[Halo-CampE-UEVR] BRAKE %s (grip, mode=%d)",
                                     want_brake ? "DOWN" : "UP", g_cfg.brake_mode);
            }
        }

        // ---- CUTSCENE FLAT VIEW (doctrine in Config.hpp; evidence: docs\CUTSCENE_FINDINGS.md
        // in the private tree). This game's cutscenes are pre-rendered movies drawn by the
        // engine's native fullscreen movie player, outside the UObject world -- which is why
        // they double in stereo and why no reflected media object exists to re-host. While one
        // plays, the configured flatten mode is applied and restored when gameplay returns.
        // All transitions are applied ON CHANGE, never per tick.
        {
            static uint32_t cine_seen_tick = 0;
            static bool     cine_prev      = false;
            static std::string cine_name;      // last sighted cinematic camera class, for the log

            // The game's own answer, polled below. Held across ticks: the engage/release logic
            // after the poll gate runs per tick. FILE-SCOPE rather than function-local because the
            // stick-mode detector earlier in this same tick consults it (see g_cine_active); it
            // reads the PREVIOUS tick's value, which at a 4 Hz poll against a multi-second movie is
            // not a distinction that exists.
            bool& s_cin_active = g_cine_active;   // IsCinematicInProgress(), when it answers
            bool& s_cin_ok     = g_cine_answering;// whether the subsystem is answering at all

            static uint32_t cut_poll = 0;
            if (tick - cut_poll >= 8) {        // ~4 Hz: the camera signal holds for seconds
                cut_poll = tick;

                // PRIMARY: ask the game. BlamCinematicSubsystem::IsCinematicInProgress() is
                // authoritative for the movie's whole span (live-verified). One UFunction call
                // per poll, same cost class as menu_poll's IsUIActiveState.
                if (auto* cs = find_cine_subsystem()) {
                    s_cin_ok = true;
                    const bool now_active = call_ret_bool(cs, L"IsCinematicInProgress");
                    if (now_active != s_cin_active) {
                        s_cin_active = now_active;
                        API::get()->log_info("[Halo-CampE-UEVR] IsCinematicInProgress -> %d",
                                             (int)now_active);
                    }
                } else {
                    s_cin_ok = false;
                    s_cin_active = false;
                }

                bool cinecam = false;
                if (pc0 != nullptr && !frontend) {
                    auto* pco = reinterpret_cast<API::UObject*>(pc0);
                    if (auto* pcm_p = pco->get_property_data<API::UObject*>(L"PlayerCameraManager")) {
                        API::UObject* pcm = *pcm_p;
                        if (pcm != nullptr && !IsBadReadPtr(pcm, sizeof(void*))) {
                            // FTViewTarget's first member is the target actor, so the struct's
                            // property data IS the pointer. Same read the community cutscene
                            // plugin uses, so the layout is field-proven on this game.
                            if (auto* vt_p = pcm->get_property_data<API::UObject*>(L"ViewTarget")) {
                                API::UObject* vt = *vt_p;
                                if (vt != nullptr && !IsBadReadPtr(vt, sizeof(void*))) {
                                    // Native class, present from module load -- ONE lookup ever.
                                    // find_uobject sweeps the whole object array, so it must not
                                    // be retried on a poll cadence.
                                    static API::UClass* cine_cls = nullptr;
                                    static bool cine_cls_tried = false;
                                    if (!cine_cls_tried) {
                                        cine_cls_tried = true;
                                        cine_cls = API::get()->find_uobject<API::UClass>(
                                            L"Class /Script/CinematicCamera.CineCameraActor");
                                        if (cine_cls == nullptr) {
                                            API::get()->log_info(
                                                "[Halo-CampE-UEVR] CineCameraActor class not found -- "
                                                "cutscene 2D screen disarmed");
                                        }
                                    }
                                    if (cine_cls != nullptr && vt->is_a(cine_cls)) {
                                        cinecam = true;
                                        cine_name = narrow(class_name_of(vt));
                                    }
#if HALO_VR_DEV
                                    // Recon for a full-span cutscene signal: what the camera
                                    // actually IS when the CineCameraActor drops away mid-scene.
                                    static std::wstring vtn_prev;
                                    const std::wstring vtn = class_name_of(vt);
                                    if (g_stick_mode.load() && vtn != vtn_prev) {
                                        vtn_prev = vtn;
                                        API::get()->log_info("[Halo-CampE-UEVR] DEV viewtarget class -> %s",
                                                             narrow(vtn).c_str());
                                    }
#endif
                                }
                            }
                        }
                    }
                }
                if (cinecam) cine_seen_tick = tick;
                if (cinecam != cine_prev) {
                    cine_prev = cinecam;
                    API::get()->log_info("[Halo-CampE-UEVR] cutscene camera signal -> %d%s%s",
                                         (int)cinecam, cinecam ? " " : "",
                                         cinecam ? cine_name.c_str() : "");
                }
            }

            // A sighting inside the last ~3 s. Bridges the gap between the camera cut and the
            // stick-mode enter debounce, whichever lands first.
            const bool cine_recent = (cine_seen_tick != 0) && (tick - cine_seen_tick < 96);

            // Which lever ENGAGE actually pulled (1 = 2D screen, 2 = mono collapse), and what to
            // restore. Latched at engage time -- the config may change mid-scene, and the
            // release must undo what was done, not what the config now says.
            static int  engaged_mode = 0;
            static char saved_scale[32] = {};

            // ---- ONE PREDICATE DRIVES BOTH EDGES.
            //
            // The first version asked a DIFFERENT question to engage than to release: engage
            // OR'd the camera heuristic in, release consulted only the subsystem. At the end of
            // a scene the subsystem says "over" while the cinematic camera was still seen moments
            // ago and the weapon has not come back yet -- so it released, immediately re-engaged,
            // released again, toggling VR_2DScreenMode several times a second. Every toggle
            // reallocates the entire view target, which in a headset reads as violent stereo
            // thrashing: field-reported 2026-08-05 leaving the Silent Cartographer opener, and
            // visible in the logs as 126 transitions in a single session.
            //
            // Deriving both edges from the SAME value makes that impossible by construction. The
            // subsystem, when it answers, is the sole authority; the camera/stick heuristic is
            // consulted only on a build where the subsystem is missing.
            const bool cine_signal = s_cin_ok ? s_cin_active
                                              : (g_stick_mode.load() && cine_recent);

            // Comfort backstop, independent of the logic above: a VR-VISIBLE ACTUATOR MUST NEVER
            // BE ALLOWED TO OSCILLATE, whatever the upstream signal does. Engage is rate-limited
            // after any transition (release never is -- being stuck flat is far better than
            // flashing the view), and if transitions still pile up the whole feature latches OFF
            // for the session rather than keep strobing someone's eyes.
            static uint32_t cut_engage_block_until = 0;   // tick
            static uint32_t flap_window_start      = 0;   // tick
            static int      flap_count             = 0;
            static bool     cut_flap_latched       = false;

            bool want = g_cut2d_engaged.load();
            const char* why = nullptr;
            if (!want) {
                // ENGAGE while a cutscene is running. Menus excluded: the frontend already
                // resolves to a proper screen on its own.
                if (g_cfg.cutscene_2d != 0 && !g_in_menu.load() && cine_signal
                    && !cut_flap_latched && tick >= cut_engage_block_until) {
                    if (g_cfg.cutscene_2d == 1) {
                        // If the flat screen is already on, the player runs it deliberately --
                        // nothing to own, and nothing to wrongly restore later.
                        char cur[16]{};
                        API::get()->param()->vr->get_mod_value("VR_2DScreenMode", cur, sizeof(cur));
                        if (strcmp(cur, "true") != 0) { engaged_mode = 1; want = true; }
                    } else {
                        // MONO COLLAPSE. The saved value is the restore target, so an unreadable
                        // or already-collapsed scale means do nothing rather than engage blind.
                        char cur[32]{};
                        API::get()->param()->vr->get_mod_value("VR_WorldScale", cur, sizeof(cur));
                        if (atof(cur) > 0.02) {
                            strncpy_s(saved_scale, sizeof(saved_scale), cur, _TRUNCATE);
                            engaged_mode = 2;
                            want = true;
                        }
                    }
                }
            } else {
                // RELEASE the moment the same predicate goes false -- authoritative when the
                // subsystem answers, weapon-return/camera-recency otherwise. The brake grip
                // stays as the manual escape.
                if      (g_cfg.cutscene_2d == 0) why = "config off";
                else if (cut_flap_latched)       why = "flap guard";
                else if (!cine_signal)           why = s_cin_ok ? "cinematic over" : "weapon returned";
                else if (g_brake_pad.load())     why = "brake grip (escape)";
                if (why != nullptr) want = false;
            }

            const bool was = g_cut2d_engaged.exchange(want);
            if (was != want) {
                // Hold off the next ENGAGE briefly (~1 s at 32 Hz), and watch for flapping: more
                // than 6 transitions inside ~4 s is not a cutscene, it is an oscillation, and it
                // gets shut down for the session.
                cut_engage_block_until = tick + 32;
                if (tick - flap_window_start > 128) { flap_window_start = tick; flap_count = 0; }
                if (++flap_count > 6 && !cut_flap_latched) {
                    cut_flap_latched = true;
                    API::get()->log_info("[Halo-CampE-UEVR] CUTSCENE FLAT LATCHED OFF -- %d "
                                         "transitions in under 4 s. Please report this log; the "
                                         "flat view stays off for this session (cutscene2d=0 to "
                                         "disable permanently).", flap_count);
                }
            }
            if (was != want) {
                if (want) {
                    if (engaged_mode == 1) {
                        API::get()->param()->vr->set_mod_value("VR_2DScreenMode", "true");
                        API::get()->log_info("[Halo-CampE-UEVR] CUTSCENE FLAT ENGAGE (%s) -- "
                                             "VR_2DScreenMode -> true", cine_name.c_str());
                        // The hint rides the 2D screen only: in mono collapse the world stays
                        // visible, so "if nothing shows" would be nonsense there.
                        cutscene_hint_show();
                    } else {
                        API::get()->param()->vr->set_mod_value("VR_WorldScale", "0.01");
                        API::get()->log_info("[Halo-CampE-UEVR] CUTSCENE FLAT ENGAGE (%s) -- "
                                             "VR_WorldScale %s -> 0.01 (mono collapse)",
                                             cine_name.c_str(), saved_scale);
                    }
                } else {
                    if (engaged_mode == 1) {
                        API::get()->param()->vr->set_mod_value("VR_2DScreenMode", "false");
                        API::get()->log_info("[Halo-CampE-UEVR] CUTSCENE FLAT RELEASE (%s) -- "
                                             "VR_2DScreenMode -> false", why != nullptr ? why : "?");
                    } else if (engaged_mode == 2 && saved_scale[0] != '\0') {
                        API::get()->param()->vr->set_mod_value("VR_WorldScale", saved_scale);
                        API::get()->log_info("[Halo-CampE-UEVR] CUTSCENE FLAT RELEASE (%s) -- "
                                             "VR_WorldScale -> %s", why != nullptr ? why : "?",
                                             saved_scale);
                    }
                    engaged_mode = 0;
                    cutscene_hint_hide();
                }
            }

            // A cfg flip mid-scene must also clear the hint; hide() is a cheap no-op otherwise.
            if (!g_cfg.cut_hint) cutscene_hint_hide();

            // Startup sanity, once mod values are ready (same timing as the inactivity-timer
            // write above). A stale VR_2DScreenMode=true is unambiguous while this mod owns
            // cutscene flattening, so it is corrected; a floor-level world scale could be a
            // deliberate setting, so it is only called out.
            if (tick == 300 && g_cfg.cutscene_2d != 0 && !g_cut2d_engaged.load()) {
                char cur[32]{};
                API::get()->param()->vr->get_mod_value("VR_2DScreenMode", cur, sizeof(cur));
                if (strcmp(cur, "true") == 0) {
                    API::get()->param()->vr->set_mod_value("VR_2DScreenMode", "false");
                    API::get()->log_info("[Halo-CampE-UEVR] stale VR_2DScreenMode=true at startup "
                                         "-> false (cutscene2d owns this switch; set cutscene2d=0 "
                                         "to run the flat screen permanently)");
                }
                API::get()->param()->vr->get_mod_value("VR_WorldScale", cur, sizeof(cur));
                const double ws = atof(cur);
                if (ws > 0.0 && ws <= 0.011) {
                    API::get()->log_info("[Halo-CampE-UEVR] VR_WorldScale reads %s at startup -- "
                                         "if that is stale from a cutscene, restore your intended "
                                         "world scale in the UEVR menu", cur);
                }
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

    // A dev path that RETURNS from update() once lived here (`devreticle`), placing the reticule
    // from the pawn transform so it could be tested with no VR runtime. It answered its question
    // and was removed 2026-08-09. Do not reintroduce the shape: returning here skips not just the
    // aim loop but every button remap below it, so a stale devreticle=1 in a live config took a
    // player's motion controls away mid-session with no error and no log line. Anything that
    // bypasses the whole driver must not be reachable from a config key.

    // THE aim source. Everything downstream -- the control law, the rig, the reticule -- flows from
    // this one index, which is why handedness is a single decision here rather than a sweep through
    // the file. The maths is hand-agnostic: it turns a pose into angles.
    const auto ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                          : API::VR::get_right_controller_index();
    if (ridx < 0) {
        static uint32_t last = 0;
        if (tick - last > 300) {
            last = tick;
            API::get()->log_info("[Halo-CampE-UEVR] IDLE: no %s controller index",
                                 g_cfg.aim_left_hand ? "left" : "right");
        }
        g_out_rx = 0.0f; g_out_ry = 0.0f; g_driving = false; return;
    }

    // AIM pose: the runtime's POINTING pose. Correct for aim direction.
    Vec3 cpos{}; Quat cq{};
    bool have_pose = get_pose(ridx, &cpos, &cq, /*use_aim=*/true);
#if HALO_VR_DEV
    // GRIP-POSE FALLBACK -- DEV BUILDS ONLY, and only when the aim pose is genuinely unavailable.
    //
    // SimVR's null driver supplies a grip pose but not the runtime's separate AIM pose, so this
    // guard failed every time and the loop never armed: no shape(), no RIGTRACK, no t600, and
    // settings_are_loaded() stayed false so vrsens/vrdeadzone/vraccel never applied either. That
    // made the entire aim lane untestable without a headset on someone's face -- which is why the
    // deadbeat setpoint sat unvalidated.
    //
    // Falling back to the grip pose changes WHERE the ray starts, not how the control loop behaves,
    // so convergence and steady-state residual -- the things being measured -- stay meaningful.
    // Never compiled into a shipping build: on real hardware the aim pose exists, and silently
    // switching a player's aim basis would be a genuine behaviour change.
    if (!have_pose) {
        have_pose = get_pose(ridx, &cpos, &cq, /*use_aim=*/false);
        static uint32_t fb_last = 0;
        if (have_pose && tick - fb_last > 300) {
            fb_last = tick;
            API::get()->log_info("[Halo-CampE-UEVR] DEV: aim pose unavailable, using GRIP pose "
                                 "(SimVR harness path -- not a shipping behaviour)");
        }
    }
#endif
    if (!have_pose) {
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

    // IS THERE A POSITION AT ALL? Rotation and position can arrive independently: a runtime may
    // report a fully valid orientation while never populating translation, and UEVR passes that
    // through unchanged. Aim only needs the ROTATION -- it is a direction -- so a position-less
    // controller can still drive aim perfectly. What it cannot do is place the rig.
    //
    // Field evidence (2026-08-18, PSVR2 over ALVR): controller position read exactly (0,0,0) for an
    // entire session while rotation swung the full range and the aim loop tracked it to err<0.3 deg.
    // Because `hand` is measured from the standing origin, a zero position resolves to a phantom
    // hand ~1.5 m away, so the weapon was placed off in space and every rig number in the log was
    // nonsense. From the player's seat that is indistinguishable from "motion controls don't work",
    // even though the aim half was working the whole time.
    //
    // EXACT zero on all three axes is the signature. Real tracking does not land on the origin
    // bit-exactly, and requiring a RUN of them means a single odd frame cannot trip this.
    static uint32_t s_zero_pos_run = 0;
    static bool     s_pos_dead_logged = false;
    const bool pos_is_zero = (rigpos.x == 0.0f && rigpos.y == 0.0f && rigpos.z == 0.0f);
    if (pos_is_zero) { if (s_zero_pos_run < 100000u) ++s_zero_pos_run; }
    else             { s_zero_pos_run = 0; s_pos_dead_logged = false; }

    // IS THERE A POSE AT ALL? Evaluated HERE, against the raw quaternion the runtime handed us,
    // and published for the aim law -- which must not re-derive it from ctrl_yaw/ctrl_pitch,
    // because those carry the accumulated snap-turn and are not zero for an empty pose. That
    // mistake shipped once and left the guard silent through nine field dropouts.
    // AimPoseGuard.hpp carries the reasoning; the rig keeps using the run-length gate below.
    // DEBOUNCED, for the same reason the zero-POSITION run above is: one odd frame must not trip a
    // guard whose whole purpose is to ride out an outage lasting seconds.
    //
    // Undebounced, this flapped. Measured 2026-09-07 from a pre-release build with the sniper in
    // hand: ELEVEN freeze/release cycles inside two seconds, of 1, 1, 2, 2, 3, 4 ticks each,
    // alongside the genuine long ones (805, 1998, 2508 ticks). The long ones are what the guard is
    // for -- focus loss, a level-load stall -- and they are unaffected by a two-tick delay. The
    // short ones are a runtime handing us one empty pose and then a good one, and freezing for a
    // single tick on that is worse than simply using the last good pose: the player feels it as
    // periodic stutter, which is exactly how it was reported.
    //
    // The freeze REPORTER's own comment a few hundred lines below assumed "an outage lasts seconds
    // rather than a single tick" and used that to justify a non-atomic edge detector. That
    // assumption was false; this is what makes it true.
    //
    // Release is INSTANT and arm is DELAYED, deliberately asymmetric: coming back late costs a
    // frame of aim, going in late costs nothing, and the failure we are guarding against is long.
    {
        static uint32_t s_empty_run = 0;
        const bool raw_empty = halo::aim_pose_is_empty(pos_is_zero, cq.x, cq.y, cq.z);
        if (raw_empty) { if (s_empty_run < 1000u) ++s_empty_run; }
        else           { s_empty_run = 0; }
        // 3 ticks: long enough that a one- or two-frame runtime hiccup never reaches it, short
        // enough to be imperceptible against an outage measured in hundreds of ticks.
        halo::g_ctrl_pose_empty.store(s_empty_run >= 3u, std::memory_order_relaxed);
    }

    // ~2 s at tick rate. Long enough that a transient cannot reach it, short enough to be reported
    // before the player has finished wondering why the gun is not in their hand.
    const bool position_dead = (s_zero_pos_run > 120u);
    if (position_dead && !s_pos_dead_logged) {
        s_pos_dead_logged = true;
        // WORDING CORRECTED 2026-09-03. This used to assert "the runtime is giving rotation but no
        // translation", which sent a reader hunting a translation-only fault. The shipped logs show
        // the common cause is the whole pose going empty at once -- position AND rotation -- when
        // the OpenXR session drops out of FOCUSED and xrSyncActions stops updating actions. One
        // 16-minute session logged 13,397 consecutive `XR_SESSION_NOT_FOCUSED` sync failures.
        // Both causes are named here because they need different answers from the player.
        API::get()->log_info(
            "[Halo-CampE-UEVR] CONTROLLER POSITION MISSING: no translation for the aim hand "
            "(exact 0,0,0 for %u ticks). TWO CAUSES, and the log above tells them apart. (1) The "
            "XR session lost FOCUS -- a runtime overlay or dashboard opened, the game stalled on a "
            "level load and stopped submitting frames, or its window left the foreground. Then the "
            "whole pose is empty, rotation included, aim is FROZEN where you left it rather than "
            "driven from an empty pose, and everything resumes by itself when focus comes back "
            "(look for 'SESSION_STATE_CHANGED 5'). (2) The runtime is publishing rotation but no "
            "position: aim keeps working, only the weapon rig is held at its neutral instead of "
            "being flung ~1.5 m away. If your hands track in other VR apps, it is (2).",
            s_zero_pos_run);
    }

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
                                 "-- different PlayerController class?",
                                 (size_t)g_control_rotation_offset.load(std::memory_order_relaxed));
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
        // The smoothed convergence range describes geometry in the level we just left. It would
        // wash out on its own within a few hundred ms, but "a few hundred ms of aim bent toward a
        // wall that no longer exists" is exactly the kind of first-second-after-a-load weirdness
        // that gets reported as a calibration bug. Measure it fresh instead.
        halo::aim_converge_reset();

        // The two-handed hold, for the same reason and then one more. Its latch survives anything
        // that does not explicitly clear it, and its remembered hand-to-hand line describes the
        // level we just left -- so a hold live across a transition would keep bending aim and the
        // rendered weapon toward geometry that no longer exists, with no way for the player to
        // work out why. The menu gate usually releases it on the loading screen, but "usually" is
        // not a guarantee and this is the event that actually means it.
        halo::two_hand_reset("level transition");
        // The latched socket belongs to an actor that no longer exists, and the cached weapon root
        // is a dangling pointer into a pooled actor. Both must go before anything reads them.
        halo::weapon_drive_reset("level transition");

        // Level transition: drop the UObjectHook attachment and the rig pointer BEFORE the old
        // actors are torn down. See attach_release() -- this is what keeps us out of the
        // render-path use-after-free.
        auto* old_rig = reinterpret_cast<API::UObject*>(g_rig_component.load());
        attach_release(old_rig, "PlayerController changed");
        g_rig_component = nullptr;
        g_rig_parent = nullptr;
        // The shell belongs to the pawn being torn down. Dropping it here, alongside the rig, is
        // what keeps the re-acquire honest instead of writing into a recycled slot on the new level.
        forget_shield_shell();
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

    // ---- THE TWO-HANDED HOLD, on the tick-side copy of the aim derivation.
    //
    // This is a SECOND entry point -- it does not call derive_ctrl_angles(), it repeats it (the
    // sightline comment below says as much). Bending only one of the two would put the control law
    // and the sim write on different lines, which is the exact failure this feature must avoid.
    //
    // NOT on a tick that captures the reference. The law is desired = ref_aim + (ctrl - ref_ctrl):
    // capture the reference from a bent ctrl and the blend cancels exactly, so the hold would do
    // nothing at all. Skipping the bend on capture ticks keeps the captured pair self-consistent
    // and costs one tick of one-handed aim, which is invisible. The condition is the same one the
    // capture block below tests, written here so the two cannot drift apart.
    const bool capturing_reference = !g_stick_mode.load() && !g_have_ref.load();
    if (!capturing_reference) halo::two_hand_bend_forward(&fwd);

    // SIGHTLINE -- this is where controller TRANSLATION enters the aim solution, not just rotation.
    // Aim is the ray from an origin THROUGH the point the gun points at, so sliding the gun
    // sideways with unchanged rotation still moves aim. Straight from UEVR's controller-aim path.
    {
        // Shared with derive_ctrl_angles' copy of this sightline -- one definition of "where the
        // player is", so the two cannot disagree. It also handles the unleashed case, where the
        // standing origin stops being a body reference at all.
        Vec3 origin{};
        const bool have_origin = aim_sightline_origin(&origin);

        if (have_origin) {
            Vec3 t{
                cpos.x + fwd.x * g_cfg.xdist_m - origin.x,
                cpos.y + fwd.y * g_cfg.xdist_m - origin.y,
                cpos.z + fwd.z * g_cfg.xdist_m - origin.z
            };
            const float len = std::sqrt(t.x * t.x + t.y * t.y + t.z * t.z);
            if (len > 1e-3f) { fwd = Vec3{t.x / len, t.y / len, t.z / len}; }
        }

#if HALO_VR_DEV
        // SIGHTLINE -- is aimorigin actually doing anything, and how far have you drifted?
        //
        // The two modes differ ONLY in how the ray reacts to the player TRANSLATING:
        //   mode 1 anchors to the standing origin, a fixed room point set by the play-area reset.
        //     Walk away from it and the ray swings by roughly atan(drift / xdist) with no rotation
        //     on your part -- 30 cm at xdist 10 m is 1.7 deg, and it stays until you re-centre.
        //   mode 0 anchors to the HMD, which travels with you, so the drift term cancels.
        //
        // `drift` is the distance from the standing origin to your head. Under mode 1 that number
        // IS the error source, and `swing` converts it to the angle it costs. Under mode 0 both
        // should stay flat while you walk. If they do not differ between modes, the two origins are
        // not in the space this assumes and the whole sightline needs re-deriving, not tuning.
        {
            static uint32_t last = 0;
            if (tick - last >= 60) {
                last = tick;
                Vec3 hp{}; Quat hq{};
                const auto hi = API::VR::get_hmd_index();
                const bool have_h = (hi >= 0) && get_pose(hi, &hp, &hq, /*use_aim=*/false);
                const auto so = API::VR::get_standing_origin();
                const float dx = have_h ? (hp.x - so.x) : 0.0f;
                const float dy = have_h ? (hp.y - so.y) : 0.0f;
                const float dz = have_h ? (hp.z - so.z) : 0.0f;
                const float drift = std::sqrt(dx * dx + dy * dy + dz * dz);
                const float swing = (g_cfg.xdist_m > 0.01f)
                    ? std::atan2(drift, g_cfg.xdist_m) * RAD2DEG : 0.0f;
                API::get()->log_info(
                    "[Halo-CampE-UEVR] SIGHTLINE mode=%d origin=(%.3f,%.3f,%.3f) hmd=(%.3f,%.3f,%.3f) "
                    "stand=(%.3f,%.3f,%.3f) drift=%.1fcm swing=%.2fdeg xdist=%.0fcm haveH=%d",
                    g_cfg.aim_origin, origin.x, origin.y, origin.z,
                    have_h ? hp.x : 0.0f, have_h ? hp.y : 0.0f, have_h ? hp.z : 0.0f,
                    so.x, so.y, so.z, drift * 100.0f, swing, g_cfg.xdist_m * 100.0f, (int)have_h);
            }
        }
#endif
    }

    // ---- SNAP TURN, applied to the AIM mapping as well as the rig.
    //
    // The aim reference maps a ROOM-space controller yaw onto a GAME-space aim yaw, and a snap
    // turn rotates the rendered world without touching ControlRotation -- so after a turn the
    // same physical pointing direction means a different game direction, and the reference is
    // stale by exactly the turn. Both the rig frame AND this mapping need the correction;
    // correcting only one leaves the other breaking the same way.
    float ctrl_yaw   = wrap180(std::atan2(fwd.x, -fwd.z) * RAD2DEG
                               + g_cfg.aim_turn * g_turn_offset.load());
    float ctrl_pitch = std::asin(clampf(fwd.y, -1.0f, 1.0f)) * RAD2DEG;

    // TARGET SMOOTHING -- filter WHERE WE ARE ASKED TO POINT, not how fast we get there.
    //
    // `aim_tau_s` used to do two unrelated jobs: pace the approach AND low-pass hand tremor.
    // aim_deadbeat removes the pacing, and with it the incidental filtering, so tremor would ride
    // straight through into the aim. This is the intended replacement, and the reason the deadbeat
    // comment says "smooth the TARGET rather than lowering the gain back toward tau" -- lowering the
    // gain restores the lag deadbeat exists to remove, whereas this leaves convergence alone.
    //
    // Applied BEFORE the reference capture below so the reference is taken against the same
    // (smoothed) signal the loop will chase; capturing against the raw angle would bake in a
    // one-sample offset.
    //
    // 0 = off, and it ships off: SimVR has no tremor, so this cannot be validated headlessly. It is
    // a knob for the first in-headset session, not a tuned default.
    if (g_cfg.aim_target_smooth_ms > 0.0f) {
        static float sm_cy = 0.0f, sm_cp = 0.0f;
        static bool  have_sm_ctrl = false;
        const float a = ema_alpha(g_cfg.aim_target_smooth_ms, g_last_dt.load());
        if (!have_sm_ctrl) {
            sm_cy = ctrl_yaw; sm_cp = ctrl_pitch; have_sm_ctrl = true;   // never filter sample one
        } else if (a > 0.0f) {
            // Through wrap180 so the filter never takes the long way round at the +-180 seam.
            sm_cy = wrap180(sm_cy + a * wrap180(ctrl_yaw - sm_cy));
            sm_cp = sm_cp + a * (ctrl_pitch - sm_cp);
        }
        ctrl_yaw = sm_cy; ctrl_pitch = sm_cp;
    }

    // Reference capture: record the hand-to-aim offset once so enabling never snaps the view.
    // Not while stick mode holds the stack down -- a reference captured against a vehicle camera
    // is garbage, and the exit transition re-captures the moment the stack re-arms.
    if (!g_stick_mode.load() && !g_have_ref.load()) {
        // Prefer the pose sampled AT the Page Down release edge over the one read here a tick
        // later. Only for this capture -- consumed once, so an ordinary reference recapture (level
        // load, respawn, stick-mode exit) still reads the controller live as it always did.
        if (g_aimcal_have_snap.exchange(false)) {
            ctrl_yaw   = g_aimcal_snap_yaw.load();
            ctrl_pitch = g_aimcal_snap_pitch.load();
        }
        g_ref_ctrl_yaw   = ctrl_yaw;
        g_ref_ctrl_pitch = ctrl_pitch;

        const bool measuring = g_aimcal_capture.exchange(false);

        if (g_cfg.aim_off_valid && !measuring) {
            // RESTORE a saved calibration rather than re-capturing whatever the game happens to
            // be aiming at: re-capturing binds the controller to the aim of that instant, which
            // after a level load or respawn is arbitrary -- silently discarding the calibration.
            // The loop then drives the aim onto the saved mapping.
            // USE SITE 1 of 2: add back the frame the offset was measured against (0 when off).
            // Safe as a scalar add here, unlike the grip trim: ctrl_yaw and aim_off_yaw are both
            // plain yaws in a yaw-only computation, so there is no pitch/roll to mix.
            g_ref_aim_yaw   = wrap180(ctrl_yaw + g_cfg.aim_off_yaw + aim_frame_yaw_use());
            g_ref_aim_pitch = ctrl_pitch + g_cfg.aim_off_pitch;
            g_have_ref = true;
            g_gain_hold = true; g_gain_hold_until = tick + 90;   // let the aim settle before measuring gain
            API::get()->log_info("[Halo-CampE-UEVR] reference RESTORED from saved calibration: offset yaw=%.1f pitch=%.1f (frame yaw %.1f)",
                                 g_cfg.aim_off_yaw, g_cfg.aim_off_pitch, aim_frame_yaw_use());
        } else {
            // CAPTURE IN INTENT SPACE, NOT COMMAND SPACE.
            //
            // aim_yaw is the game's ACHIEVED aim, and under 6DoF convergence that is the intent
            // already BENT toward the traced range. The drive re-applies that bend to whatever
            // reference we store here, so capturing the bent value hands it over twice: at the
            // capture instant ctrl == ref_ctrl, so desired == ref_aim, and the command comes out
            // converge(aim_yaw) rather than aim_yaw. The aim therefore JUMPS by exactly the bend
            // on the first tick after release -- a snap the instant the calibration lands, scaling
            // with how far the head has walked (a metre of divergence at 5 m is ~11 degrees).
            //
            // Removing the bend before storing fixes both halves at once: the release is seamless
            // again, and the saved offset is bend-free, so restoring it later at a different
            // divergence or range gets this session's geometry applied rather than that one's.
            //
            // converge() is small and smooth, so subtracting the bend measured AT aim_yaw inverts
            // it to first order; what is left is second order in how much the bend varies across
            // that angle, which is nothing. Identically a no-op while the head is leashed, where
            // converge declines to act at all.
            float ref_yaw = (float)aim_yaw, ref_pitch = (float)aim_pitch;
            {
                float by = ref_yaw, bp = ref_pitch;
                if (halo::aim_converge_apply(&by, &bp)) {
                    ref_yaw   = wrap180(ref_yaw - wrap180(by - ref_yaw));
                    ref_pitch = ref_pitch - (bp - ref_pitch);
                    API::get()->log_info("[Halo-CampE-UEVR] AIM CALIBRATE: removed the convergence "
                                         "bend (%.2f,%.2f deg) before capture -- the drive re-applies "
                                         "it live, and storing it would double it",
                                         wrap180(by - (float)aim_yaw), bp - (float)aim_pitch);
                }
            }

            g_ref_aim_yaw   = ref_yaw;
            g_ref_aim_pitch = ref_pitch;
            g_have_ref = true;
            g_gain_hold = true; g_gain_hold_until = tick + 90;   // let the aim settle before measuring gain

            if (measuring) {
                // Page Down release: the offset between where the hand points and where the game
                // aims IS the calibration. Store it, not the absolute pair.
                // WRITE SITE 1 of 2: store MINUS the frame, so the saved offset does not encode
                // where this session was injected. Must mirror use site 1 exactly.
                g_cfg.aim_off_yaw   = wrap180(ref_yaw - ctrl_yaw - calib_frame_yaw_write());
                // Stamp the AIM version only -- this gesture rebased the aim offset and nothing else.
                if (g_cfg.calib_relative) g_cfg.aim_calib_ver = 2;
                g_cfg.aim_off_pitch = ref_pitch - ctrl_pitch;
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
#if HALO_VR_DEV
        // AIMSTAT -- dev-only error statistics. The 600-tick report gives ~4 spot samples per
        // 80 s window, which is far too few to compare controller settings: the first deadbeat A/B
        // had a spread of 0.0-4.9 deg across 4 points. Accumulate EVERY tick and report the mean,
        // so one line summarises ~40 ticks instead of sampling one of them. Both the accumulate and
        // the report happen here on the game thread, so plain statics are safe.
        {
            static double  acc_sum = 0.0;
            static uint32_t acc_n  = 0;
            static float   acc_max = 0.0f;
            const float ae = std::fabs(g_dbg_err_yaw.load());
            acc_sum += ae; ++acc_n;
            if (ae > acc_max) acc_max = ae;
            if (g_cfg.aim_stat > 0 && acc_n >= (uint32_t)g_cfg.aim_stat) {
                API::get()->log_info("[Halo-CampE-UEVR] AIMSTAT mean=%.3f max=%.3f n=%u",
                                     acc_sum / (double)acc_n, acc_max, acc_n);
                acc_sum = 0.0; acc_n = 0; acc_max = 0.0f;
            }
        }
#endif
        g_dbg_err_pitch = wrap180(dbg_des_pitch - (float)aim_pitch);
        g_dbg_ctrl_yaw = ctrl_yaw; g_dbg_aim_yaw = (float)aim_yaw;
    }

    // AIM FREEZE, reported on the EDGE. Not dev-gated: this is the line that explains a genuine
    // player-visible outage ("my controls stopped after the loading screen"), and a player cannot
    // rebuild the mod to find out why. It fires at most twice per outage, so it costs nothing.
    //
    // Reads the SAME published flag the law acts on, so this line cannot claim something the law
    // did not do. The edge detector lives here rather than in the law because the law runs on two
    // threads and a static inside it would race; the tick path is enough, since an outage lasts
    // seconds rather than a single tick.
    {
        static bool s_frozen_prev = false;
        static uint32_t s_frozen_since = 0;
        const bool frozen = g_cfg.aim_freeze_lost
                            && halo::g_ctrl_pose_empty.load(std::memory_order_relaxed);
        if (frozen && !s_frozen_prev) {
            s_frozen_since = tick;
            API::get()->log_info(
                "[Halo-CampE-UEVR] AIM FROZEN: the controller pose is empty (position AND rotation "
                "exactly zero), which means the XR runtime has stopped updating it -- almost always "
                "a lost session focus (overlay, level-load stall, or the window leaving the "
                "foreground). Holding your aim where it is instead of driving it from an empty "
                "pose. It will resume by itself when tracking returns.");
        } else if (!frozen && s_frozen_prev) {
            API::get()->log_info("[Halo-CampE-UEVR] AIM FROZEN: released after %u ticks, aim resumed.",
                                 tick - s_frozen_since);
        }
        s_frozen_prev = frozen;
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
    //
    // The measurement itself now lives in update_gain_measurement() so it can run on EITHER path.
    // It used to be inline here and explicitly skipped whenever aim_rate_render was set -- which is
    // the default -- so the measured gain stayed 0 for every normal session, and feedforward and
    // damping, both gated on gain > 10, never ran at all.
    {
        if (g_gain_hold.load() && tick >= g_gain_hold_until.load()) g_gain_hold = false;

        // TICK PATH ONLY. When the law runs at render rate the hook measures instead, pairing each
        // aim change with the deflection actually applied across it.
        if (!g_cfg.aim_rate_render) {
            static GainMeasState tick_gain;
            update_gain_measurement(tick_gain, aim_yaw, g_out_rx.load(), g_last_dt.load());
        }

        // Logged once, from whichever path measured it, so a sensitivity mismatch is still visible.
        if (!g_gain_logged.load() && g_meas_rate.load() > 10.0f) {
            const float want = clampf(g_meas_rate.load() / REFERENCE_RATE_DPS, 0.25f, 4.0f);
            if (std::fabs(want - 1.0f) > 0.15f) {
                g_gain_logged = true;
                API::get()->log_info(
                    "[Halo-CampE-UEVR] gain adapt: measured %.0f deg/s per unit (reference %.0f) "
                    "-> full_deg x%.2f. Game controller sensitivity differs from the "
                    "calibrated LookSensitivity30.", g_meas_rate.load(), REFERENCE_RATE_DPS, want);
            }
        }
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
        if ((tick - g_rig_resolve_tick.load()) >= ((tick < g_rig_fast_until) ? 8u : 60u)) {
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
            }

            // ---- THE GRIP PIVOT -- derived ONCE, then latched, and only ONCE THERE IS A WEAPON.
            //
            // This used to re-read per rig resolve, justified as "a different weapon means a
            // different grip". True of the geometry, wrong as a policy: the mount offset is
            // fitted AGAINST this value, so re-reading it silently invalidates the fit. Worse,
            // the read samples a LIVE ANIMATED skeleton, so consecutive derives of the SAME
            // socket on the SAME weapon disagree by tens of centimetres depending on what the
            // arms were mid-pose -- measured at 43.7 cm across one mission restart.
            //
            // It sits OUTSIDE the rig-identity branch above, gated on the weapon route instead,
            // because the rig can now be acquired from the pawn before any weapon exists. Left
            // inside, the one derive this session gets would sample the UNARMED pose -- and the
            // unarmed pose is a T-pose, the furthest thing from a weapon idle there is -- and then
            // latch it forever; and the identity branch would never re-enter when the gun finally
            // arrived, because the rig is the same component either way.
            //
            // A calibration pins it properly (see the solve, which sets pivot_from_calib). This
            // latch is what protects a session that has not calibrated yet: one derive, then
            // constant, so the arms cannot move because the rig happened to re-resolve.
            {
                auto* rig_now = reinterpret_cast<API::UObject*>(g_rig_component.load());
                static bool s_pivot_latched = false;
                if (g_cfg.piv_auto && !g_pivot_from_calib && !s_pivot_latched
                    && rig_now != nullptr && fp_weapon_route_alive()) {
                    s_pivot_latched = true;
                    log_pivot_candidates(rig_now);

                    // CAUSE ESTABLISHED 2026-08-12: MSVC's constant-initialization of the global
                    // g_cfg drops char-array string defaults (kept numeric ones) -- measured by
                    // printing the global (empty) vs a stack instance (the literal). This
                    // fallback is therefore THE default now, not a workaround: Config.hpp
                    // deliberately ships the field empty and points here. Since an empty name
                    // silently disables the pivot term entirely -- and a zero pivot is what makes
                    // the weapon swing on a lever when you only rotated -- fall back rather than
                    // fail closed, and say so.
                    const char* sock = g_cfg.piv_socket;
                    if (sock[0] == '\0') {
                        sock = "PrimaryWeapon";   // the built-in default (see Config.hpp note)
                    }

                    wchar_t wsock[64] = {0};
                    MultiByteToWideChar(CP_UTF8, 0, sock, -1, wsock, 63);

                    Vec3 piv{};
                    if (derive_pivot(rig_now, wsock, &piv)) {
                        g_cfg.piv_x = piv.x + g_cfg.piv_adj_x;
                        g_cfg.piv_y = piv.y + g_cfg.piv_adj_y;
                        g_cfg.piv_z = piv.z + g_cfg.piv_adj_z;
                        // Report `sock`, not g_cfg.piv_socket: when the fallback above fires those
                        // differ, and printing the empty one makes a WORKING lookup read as a
                        // failed one. That log line cost a misdiagnosis already.
                        API::get()->log_info("[Halo-CampE-UEVR] pivot = socket '%s' (%.1f,%.1f,%.1f) + adj (%.1f,%.1f,%.1f) = (%.1f,%.1f,%.1f) cm",
                                             sock, piv.x, piv.y, piv.z,
                                             g_cfg.piv_adj_x, g_cfg.piv_adj_y, g_cfg.piv_adj_z,
                                             g_cfg.piv_x, g_cfg.piv_y, g_cfg.piv_z);
                    } else {
                        API::get()->log_info("[Halo-CampE-UEVR] pivot read FAILED for socket '%s' -- keeping piv=(%.1f,%.1f,%.1f); "
                                             "set pivauto=0 and pivx/pivy/pivz to override",
                                             sock, g_cfg.piv_x, g_cfg.piv_y, g_cfg.piv_z);
                    }
                }
            }

            // SHIELD SHELL. RE-DERIVED FROM THE LIVE PAWN EVERY TIME -- never "only while we hold
            // nothing". That gate was a real bug (mission restart, 2026-08-02): this title recycles
            // object-array slots, and a destroyed shell's slot gets reused by a NEW object OF THE
            // SAME CLASS, so the class-name check in shield_shell() passes on a corpse. The handle
            // looks healthy, never re-resolves, and we drive a dead pawn's shell while the live one
            // gets nothing -- the arms track and the shield does not. Measured windows of up to 48 s
            // where the held shell's generation did not match the rig's.
            //
            // The rig above never had this bug because it re-derives THROUGH a live weapon actor
            // every time rather than adopting by class match; this is the same discipline. Nor can
            // the level-transition reset cover it: a mission restart reuses the PlayerController, so
            // that signal never fires (no "PlayerController changed" line appears across a restart).
            //
            // Cost is O(components) on the rig's existing timer, not per tick, and note_resolved_
            // shell() only pays its O(n) slot lookup when the pointer actually CHANGES. Timed under
            // its own perf site so that claim is measurable with perflog=1 rather than argued.
            if (g_cfg.shell_drive) {
                auto* prev = shield_shell();
                API::UObject* sh = nullptr;
                { PerfScope _perf(PERF_SHELL); sh = resolve_shield_shell(g_rig_parent); }
                if (sh == nullptr) {
                    // No pawn, or the component is gone: drop the handle rather than keep writing
                    // into whatever the slot now holds. Re-acquisition is one timer tick away.
                    if (prev != nullptr) {
                        forget_shield_shell();
                        API::get()->log_info("[Halo-CampE-UEVR] shield shell lost -- will re-acquire");
                    }
                } else if (sh != prev) {
                    note_resolved_shell(sh);
                    const std::string snm = (sh->get_fname() != nullptr)
                                          ? narrow(sh->get_fname()->to_string()) : "?";
                    API::get()->log_info("[Halo-CampE-UEVR] shield shell acquired: %s %s%s",
                                         narrow(class_name_of(sh)).c_str(), snm.c_str(),
                                         prev != nullptr ? "  (REBOUND from a previous instance)" : "");

                    // THE ASSUMPTION THIS FEATURE STANDS ON, stated as a check rather than a
                    // hope: an identical RELATIVE transform only lands in the same place if
                    // both components hang off the SAME parent. Same parent -> the shell tracks
                    // the arms exactly. Different parent -> it will move, but to the wrong
                    // place, and that is far more confusing to debug than not moving at all.
                    auto* sh_par = follow_object(sh, L"AttachParent");
                    if (sh_par == g_rig_parent) {
                        API::get()->log_info("[Halo-CampE-UEVR]   shell parent MATCHES the rig's -- relative transforms are directly comparable");
                    } else {
                        API::get()->log_info("[Halo-CampE-UEVR]   WARNING: shell parent %s DIFFERS from the rig's %s -- "
                                             "identical relative transforms will NOT co-locate; set shell=0 and re-measure",
                                             sh_par != nullptr ? narrow(class_name_of(sh_par)).c_str() : "<none>",
                                             g_rig_parent != nullptr ? narrow(class_name_of(g_rig_parent)).c_str() : "<none>");
                    }
                }
            } else {
                forget_shield_shell();
            }
        }

        // ---- HIDE THE ARMS WHILE UNARMED (doctrine in Config.hpp).
        //
        // The empty first-person arms are T-POSED -- the game never expects them to be looked at,
        // because on a flat screen "no weapon" means no viewmodel at all. Now that the rig follows
        // the hand while unarmed, that T-pose swings around with it.
        //
        // WHAT GETS HIDDEN: the rig with PROPAGATION (its six armour static meshes are children and
        // would otherwise stay behind as floating shoulder pads) plus the shield shell, which is a
        // SIBLING and so is not covered by propagation. The full-body shadow proxy is deliberately
        // left alone -- it belongs at the player's feet and is not part of the first-person arms.
        //
        // APPLIED ON CHANGE, then re-asserted on a slow cadence while hidden. Not per tick: this is
        // a UFunction call that walks children, and the tunables rule here is explicit that engine
        // calls do not belong on every tick. Not once-only either -- the game re-poses and re-shows
        // first-person components on pickups and level events, and a one-shot hide that gets
        // silently undone reads exactly like the hide never working.
        //
        // !!! WE ONLY EVER UNDO OUR OWN HIDE. NEVER ASSERT "VISIBLE" ON OUR OWN INITIATIVE.
        //
        // The first version computed a desired visibility every tick and wrote it, so "not unarmed"
        // meant an active SetVisibility(TRUE). That is not a no-op: a vehicle seat, a cutscene and
        // a checkpoint that loads you straight into a Warthog are all "not unarmed", and the game
        // has ALREADY torn down first-person presentation in every one of them. Writing `true`
        // there overrode the game's own hide and left the arms floating in the middle of a
        // third-person chase camera -- field-reported 2026-08-16 from a checkpoint load in a
        // vehicle, with `FP arms shown` in the log as the write that did it.
        //
        // Visibility belongs to the GAME. The only claim this feature has is the hide it performed
        // itself, so the state tracked below is `we_hid` -- did WE hide these arms -- and not "what
        // should visibility be". If we never hid them we never write, and whatever the game decided
        // stands untouched. That is also why there is no "restore on startup" path: there is
        // nothing to restore until we have taken something away.
        //
        // FAIL-SAFE DIRECTION IS UNCHANGED: every path that stops wanting the hide -- config off,
        // kill switch, weapon returned -- runs the restore, because `we_hid` is true and
        // `want_hidden` is false. Nothing can leave the arms hidden behind us.
        {
            static bool     we_hid   = false;   // did WE hide them; the ONLY thing we may undo
            static void*    last_rig = nullptr;
            static uint32_t last_assert_tick = 0;

            // ANTI-FLICKER GRACE, ON THE HIDE DIRECTION ONLY.
            //
            // A vehicle dismount leaves you weaponless for a couple of frames before the game hands
            // the gun back, and that was enough to fire a full hide/show cycle: measured at 24 ms,
            // about two frames at 90 Hz, on every one of five dismounts in a session. Requiring the
            // unarmed state to HOLD before acting costs nothing anywhere it matters -- a genuinely
            // unarmed stretch lasts minutes, and the arms already take a beat to disappear at the
            // start of a scene -- while a transient cannot reach the hide at all.
            //
            // Deliberately asymmetric: SHOWING stays instant. Late arms are a glitch, but late
            // hiding is just a T-pose lingering a moment longer, and there is never a reason to
            // make a weapon-in-hand wait for a timer.
            constexpr uint32_t HIDE_GRACE_TICKS = 5;   // ~150 ms at this loop's ~32 Hz
            static bool     unarmed_prev  = false;
            static uint32_t unarmed_since = 0;
            if (g_on_foot_unarmed && !unarmed_prev) unarmed_since = tick;
            unarmed_prev = g_on_foot_unarmed;
            const bool unarmed_held = g_on_foot_unarmed
                                   && (tick - unarmed_since) >= HIDE_GRACE_TICKS;

            auto* rig_v = reinterpret_cast<API::UObject*>(g_rig_component.load());
            // Two reasons to hide the arms: the unarmed T-pose stopgap (hide_arms, grace-gated),
            // and the standing "never show arms" preference (show_arms=0), which needs no grace.
            const bool want_hidden = g_cfg.enabled &&
                                     ((g_cfg.hide_arms && unarmed_held) || !g_cfg.show_arms);

            // A NEW rig component is a different object that we have never touched, so our claim
            // does not carry over to it. Dropping the claim rather than restoring is deliberate:
            // the component we hid is gone, and writing `visible` to its replacement would be
            // exactly the unsolicited assertion this block exists to avoid.
            if (rig_v != last_rig) { last_rig = rig_v; we_hid = false; }

            // Re-assert only while WE hold the hide. The game re-shows first-person components on
            // pickups and level events, so a one-shot hide gets silently undone; but a re-assert
            // that ran when we did not own the state would be the original bug on a timer.
            const bool due = we_hid && want_hidden && (tick - last_assert_tick >= 32u);

            if (rig_v != nullptr && (want_hidden != we_hid || due)) {
                if (want_hidden != we_hid) {
                    API::get()->log_info("[Halo-CampE-UEVR] FP arms %s (unarmed=%d hidearms=%d showarms=%d)",
                                         want_hidden ? (g_cfg.show_arms ? "HIDDEN -- unarmed, no arm IK yet"
                                                                        : "HIDDEN -- showarms=0")
                                                     : "restored -- our hide released",
                                         (int)g_on_foot_unarmed, (int)g_cfg.hide_arms, (int)g_cfg.show_arms);
                }
                // KEEP THE POSE ALIVE BEFORE HIDING IT. UE stops evaluating a hidden mesh's
                // animation by default, and the weapon rides this mesh's PrimaryWeapon socket --
                // so without this the gun stops animating the moment the arms go, which reads as
                // "the weapon has no recoil". Set BEFORE the hide so the mesh is never briefly
                // invisible under the default tick option.
                //
                // Arms.cpp has always done this on the armhide path; this one is on by DEFAULT
                // (hide_arms, and showarms=0) and never did, which is why the symptom shipped.
                if (want_hidden && g_cfg.arm_keep_pose) {
                    rig_set_always_tick_pose(rig_v);
                    if (auto* sh = shield_shell()) rig_set_always_tick_pose(sh);
                }
                rig_set_visible(rig_v, !want_hidden);
                if (auto* sh = shield_shell()) rig_set_visible(sh, !want_hidden);

                // The hide PROPAGATES, and propagation crosses actor attachment -- so it just
                // blanked the attached weapon's components too. When the weapon is wanted
                // visible (showarms=0 is "just the floating gun"), re-show its own subtree.
                // Fail-visible: if the root cannot be read the weapon stays hidden with the
                // arms, and one log line says so.
                if (want_hidden && g_cfg.show_weapon) {
                    if (auto* wa = fp_weapon_actor()) {
                        alignas(16) uint8_t rp[16] = {0};
                        wa->call_function(L"K2_GetRootComponent", rp);
                        auto* wroot = *reinterpret_cast<API::UObject**>(rp);
                        if (wroot != nullptr) {
                            rig_set_visible(wroot, true);
                        } else {
                            static bool s_root_warned = false;
                            if (!s_root_warned) {
                                s_root_warned = true;
                                API::get()->log_info("[Halo-CampE-UEVR] showarms=0: weapon root "
                                                     "unresolved -- the weapon hides with the arms");
                            }
                        }
                    }
                }
                we_hid = want_hidden;
                last_assert_tick = tick;

                // READ IT BACK. On this title a successful call is not evidence anything changed --
                // half the write paths here were accepted and inert -- and an anim graph that
                // re-asserts visibility every frame would leave this fighting it at 1 Hz, which
                // presents as flicker rather than as a failure. Reported ONCE so a real problem is
                // in the log without a stream of lines if the property simply is not readable.
                static bool s_readback_warned = false;
                if (!s_readback_warned) {
                    if (auto* vis = rig_v->get_property_data<bool>(L"bVisible")) {
                        if (!IsBadReadPtr(vis, sizeof(bool)) && *vis == want_hidden) {
                            s_readback_warned = true;
                            API::get()->log_info("[Halo-CampE-UEVR] FP arms: SetVisibility did not "
                                                 "stick (bVisible=%d, wanted %d) -- the arms are "
                                                 "being re-shown by something else. hidearms=0 to "
                                                 "stop trying.",
                                                 (int)*vis, (int)!want_hidden);
                        }
                    }
                }
            }
        }

        // ---- WEAPON VISIBILITY PREFERENCE (showweapon=0). SetActorHiddenInGame on the weapon
        // ACTOR -- deliberately a different flag from the bVisible tree the arms hide uses, so
        // the two preferences compose instead of undoing each other. Same claim rules as the
        // arms: track the actor, drop the claim when it swaps (a swap resets to a fresh actor
        // the game just showed, which re-applies the hide), restore only what we hid, and
        // re-assert at ~1 Hz while we hold it, since pickups and level events re-show things.
        {
            static bool     weap_hid = false;
            static void*    last_weap = nullptr;
            static uint32_t last_weap_assert = 0;
            auto* wa = fp_weapon_actor();
            if (static_cast<void*>(wa) != last_weap) { last_weap = wa; weap_hid = false; }
            const bool want_weap_hidden = g_cfg.enabled && !g_cfg.show_weapon;
            const bool wdue = weap_hid && want_weap_hidden && (tick - last_weap_assert >= 32u);
            if (wa != nullptr && (want_weap_hidden != weap_hid || wdue)) {
                if (want_weap_hidden != weap_hid) {
                    API::get()->log_info("[Halo-CampE-UEVR] FP weapon %s (showweapon=%d)",
                                         want_weap_hidden ? "HIDDEN" : "restored",
                                         (int)g_cfg.show_weapon);
                }
                alignas(16) uint8_t p[16] = {0};
                p[0] = want_weap_hidden ? 1 : 0;
                wa->call_function(L"SetActorHiddenInGame", p);
                weap_hid = want_weap_hidden;
                last_weap_assert = tick;
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

#if HALO_VR_DEV
        // DEV: force a FULL view-lock re-prime (see Config::lock_reprime). Clearing g_lock_ever as
        // well as g_lock_primed is what makes this a first-prime rather than a stick-mode re-prime:
        // the re-prime path deliberately KEEPS the base and folds into g_turn_offset, so clearing
        // only g_lock_primed would leave `locked` exactly where it was and test nothing.
        {
            static float last_reprime = 0.0f;
            static bool  reprime_seeded = false;
            if (!reprime_seeded) { last_reprime = g_cfg.lock_reprime; reprime_seeded = true; }
            else if (g_cfg.lock_reprime != last_reprime) {
                last_reprime = g_cfg.lock_reprime;
                const float was = g_locked_view_yaw.load();
                g_lock_ever = false; g_lock_primed = false;
                API::get()->log_info("[Halo-CampE-UEVR] DEV lockreprime: forcing a first-prime "
                                     "(was pinned=%.1f) -- reproduces a mid-mission injection frame", was);
            }
        }
#endif

        // Freeze on press: snapshot what the weapon looks like RIGHT NOW, then stop driving it.
        if (g_calib_start.exchange(false)) {
            g_calib_gun_world = g_last_gun_world;
            g_calib_off_world = g_last_off_world;
            // The separation is frozen too: the mesh stops being driven from here, so the value
            // live at release still describes the pose the player is aligning against.
            g_calib_sock_local = Vec3{g_dbg_sock_x.load(), g_dbg_sock_y.load(), g_dbg_sock_z.load()};
            g_calib_valid = true;

            // WHERE YOUR HEAD WAS, so the hold can ride any movement away from it and the solve can
            // cancel it. Room space and un-mapped on purpose: the transform into game space belongs
            // to the rig block, which owns the rotation offset and world scale, and doing it here
            // would freeze a mapping that a snap turn can change mid-hold.
            {
                Vec3 hp{}; Quat hq{};
                const auto hidx = API::VR::get_hmd_index();
                const auto so = API::VR::get_standing_origin();
                g_calib_have_delta = (hidx >= 0) && get_pose(hidx, &hp, &hq, /*use_aim=*/false);
                g_calib_delta_room = g_calib_have_delta
                    ? Vec3{hp.x - so.x, hp.y - so.y, hp.z - so.z}
                    : Vec3{0.0f, 0.0f, 0.0f};
            }
            API::get()->log_info("[Halo-CampE-UEVR] CALIBRATE: weapon frozen -- move your controller onto it, then release");
        }

        auto* rig = reinterpret_cast<API::UObject*>(g_rig_component.load());
        // The shield shell gets the IDENTICAL transform, because it is posed identically to the
        // arms by its own instance of the same anim blueprint -- it only lacks our write. Fanning
        // out here rather than at each call site means rotation, translation, the pivot-marker
        // scale and the reset-to-zero all stay in lockstep by construction; there is no path that
        // moves the arms and forgets the shell.
        auto* shell = g_cfg.shell_drive ? shield_shell() : nullptr;
        auto for_each_rig = [&](auto&& fn) {
            if (rig   != nullptr) fn(rig);
            if (shell != nullptr) fn(shell);
        };

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

            // ---- THE TWO-HANDED HOLD, on the rendered weapon.
            //
            // Applied in RAW VR SPACE, before the axis conversion below and before q_ro, so the
            // composition order is unambiguous. Do NOT try to express this rotation in UE space and
            // multiply it in after the -z,x,y,-w swizzle: that swizzle is a mirror composed with an
            // inversion, so it REVERSES composition order, and getting that wrong is exactly the
            // class of hand-derived VR-frame mistake this project has already paid for once.
            //
            // Same published swing the aim path used this tick -- one source, so the rendered gun
            // and the shots cannot disagree.
            Quat cq_2h = cq, gq_2h = gq;
            halo::two_hand_bend_orientation(&cq_2h);
            halo::two_hand_bend_orientation(&gq_2h);

            float a_pitch, a_yaw, a_roll;
            {
                const float ux = -cq_2h.z, uy = cq_2h.x, uz = cq_2h.y, uw = -cq_2h.w;   // aim pose, VR -> UE
                quat_to_rotator(ux, uy, uz, uw, &a_pitch, &a_yaw, &a_roll);
            }
            float g_pitch = a_pitch, g_yaw = a_yaw, g_roll = a_roll;
            if (have_grip) {
                // rotation_offset is applied in VR SPACE, before the axis conversion -- the same
                // order as UObjectHook.cpp:2056 (`right_hand_rotation = rotation_offset * ...`).
                // Applying it after conversion would rotate about the wrong axis.
                const Quat gqo = quat_mul(q_ro, gq_2h);
                const float ux = -gqo.z, uy = gqo.x, uz = gqo.y, uw = -gqo.w;   // VR -> UE
                quat_to_rotator(ux, uy, uz, uw, &g_pitch, &g_yaw, &g_roll);
            }

            // (An arm-bind release-pose SNAPSHOT lived here and has been removed. It was built on
            // the premise that the End key-up and this solve were separated by a tick; they are not
            // -- both run inside one update() call, and the instrument that settled it measured the
            // difference at 0.00 cm. It never earned its place, and while it was here the grip->aim
            // tilt below mixed a snapshot grip with a live aim, which is a way to be wrong that the
            // live path cannot be. Do not reintroduce it without a measurement that disagrees.)

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
#if HALO_VR_DEV
            float dbg_parent_pitch = 0.0f, dbg_parent_yaw = (float)aim_yaw;   // fallback = the model above
            bool  dbg_parent_read  = false;
#endif
            if (g_rig_parent != nullptr) {
                Vec3 prot{};
                if (call_ret_vec3(g_rig_parent, L"K2_GetComponentRotation", &prot)) {
#if HALO_VR_DEV
                    dbg_parent_pitch = prot.x; dbg_parent_yaw = prot.y; dbg_parent_read = true;
#endif
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
            // The calibration frame rides HERE, not in the grip trim: q_turn left-multiplies the
            // controller orientation, so this is a true world-yaw pre-rotation. Both terms are pure
            // yaws about the same axis, which is the one case where adding the scalars is correct.
            const Quat q_turn = rotator_to_quat(
                0.0f, g_cfg.rig_turn * g_turn_offset.load() + calib_frame_yaw_use(), 0.0f);

            const Quat q_ctrl = quat_mul(q_turn, rotator_to_quat(g_pitch, g_yaw, g_roll));
            // USE SITE 2 of 2: the grip trim is a yaw calibration too, and carries the same frame.
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
            } else if (g_cfg.rig_mode == 3) {
                // DIRECT-DRIVE RIG: the weapon is simply held by the controller. Same composition
                // as mode 2, but against the direct trim, so mode 2's fitted calibration is never
                // consulted here and cannot drag its folded pivot in with it.
                const Quat q_grip_dir = rotator_to_quat(g_cfg.rig_dir_grip_deg,
                                                        g_cfg.rig_dir_grip_yaw,
                                                        g_cfg.rig_dir_grip_roll);
                const Quat q_rel = quat_mul(quat_conj(q_parent), quat_mul(q_ctrl, q_grip_dir));
                quat_to_rotator(q_rel.x, q_rel.y, q_rel.z, q_rel.w, &rig_pitch, &rig_yaw, &c_roll);
            } else if (g_cfg.rig_mode == 2) {
                // q_rel = inverse(parent) * controller * trim. The trim multiplies on the RIGHT so
                // it is applied in the CONTROLLER's local frame -- a constant wrist angle at any
                // orientation. Adding degrees to a Euler axis instead fights gimbal near vertical.
                Quat q_rel  = quat_mul(quat_conj(q_parent), quat_mul(q_ctrl, q_grip));
                // CANCEL THE SOCKET'S ROTATION. The engine renders the weapon at
                // mesh_rotation * socket_rotation, so writing the desired orientation straight to
                // the mesh leaves the socket term uncancelled -- and it is not identity on every
                // load. Post-multiplying by its inverse makes the WEAPON land on the desired
                // orientation whichever mesh instance came up. See Config::rig_sock_rot.
                if (g_cfg.rig_sock_rot && g_dbg_sock_ok.load()) {
                    q_rel = quat_mul(q_rel, quat_conj(rotator_to_quat(
                        g_dbg_sock_p.load(), g_dbg_sock_yw.load(), g_dbg_sock_r.load())));
                }
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
            //
            // KNOWN LIMITATION: the shield shell is NOT carried in this mode -- it would need its
            // own UObjectHook attachment and a matching release, and attach_release/g_attached
            // track exactly one object. attach_mode ships at 0, so the shell follows on the path
            // that actually ships; anyone turning attach_mode on gets the old detached shield back
            // and should fix it here rather than assume it works.
            // MODE 2 PINS THE GUN ITSELF, not the arms it rides on.
            //
            // Mode 1 attaches the arms mesh and lets the weapon follow its PrimaryWeapon socket, so
            // everything the socket contributes -- its offset, its rotation, and the fact that both
            // change when a respawn hands us a different mesh instance -- lands on the weapon and
            // has to be cancelled. Measured: |sep| stepped 60.0 -> 67.0 cm across one respawn, and
            // 37.2 -> 37.6 -> 60.0 across three others.
            //
            // Pinning the weapon's own root deletes that entire chain rather than compensating for
            // it: UEVR writes the component's world transform, so the result is controller plus
            // calibration and nothing else. It is the right target here specifically because the
            // arms are hidden -- the mesh's only remaining job was to carry the gun.
            //
            // The target CHANGES, unlike mode 1's: a new actor per weapon swap and per respawn.
            // attach_apply releases the previous one, and a null target releases without falling
            // through to the manual path, because running both is the fight documented above.
            const int  amode  = g_cfg.attach_mode;
            API::UObject* atarget = (amode == 2) ? fp_weapon_root() : rig;
            if (amode == 2 && atarget == nullptr) {
                if (g_attached) attach_release(rig, "attachmode 2: no weapon in hand");
                g_dbg_rig_roll = c_roll;
            } else if (amode == 1 || amode == 2) {
                static API::UObject* s_last_target = nullptr;
                if (!g_attached || atarget != s_last_target) {
                    s_last_target = atarget;
                    attach_apply(atarget, q_grip, Vec3{g_cfg.off_x, g_cfg.off_y, g_cfg.off_z});
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] UObjectHook ATTACH(mode %d): %ls hand=RIGHT permanent=%d rot=(%.3f,%.3f,%.3f,%.3f) loc=(%.1f,%.1f,%.1f)cm",
                        amode, class_name_of(atarget).c_str(),
                        (int)g_cfg.attach_permanent, q_grip.x, q_grip.y, q_grip.z, q_grip.w,
                        g_cfg.off_x, g_cfg.off_y, g_cfg.off_z);
                } else if (tick % 64 == 0) {
                    // Re-apply periodically so calibration edits take effect live, and so a
                    // re-acquired component gets its offsets back.
                    attach_apply(atarget, q_grip, Vec3{g_cfg.off_x, g_cfg.off_y, g_cfg.off_z});
                }
                g_dbg_rig_roll = c_roll;
            } else {

            if (g_attached) attach_release(rig, "attachmode switched back to 0");   // release targets the RECORDED object, not this arg

            g_dbg_rig_roll = c_roll;
            {
                // Mode 3 writes the WORLD rotation. The relative write is measurably discarded on
                // this mesh (Rig.cpp), and the location half is left relative because that half
                // demonstrably does take -- fixing only what is broken.
                const Quat q_world = quat_mul(q_parent, rotator_to_quat(rig_pitch, rig_yaw, c_roll));
                float wp = 0.0f, wy = 0.0f, wr = 0.0f;
                quat_to_rotator(q_world.x, q_world.y, q_world.z, q_world.w, &wp, &wy, &wr);
                const bool world_mode = (g_cfg.rig_mode == 3);
                // THE MESH STANDS DOWN when the socket-cancelling drive owns the gun. Two writers
                // on one transform is the fight documented at the attach_mode block above, and here
                // it would be worse than indecisive: the whole point of that mode is that the mesh
                // is left where the game puts it, so writing it would re-parent the shoulders to
                // the controller and undo the thing being attempted.
                //
                // q_gun below is still computed from the same inputs -- it is the INTENDED world
                // rotation that the pivot arm and freeze snapshot consume, and it stays correct
                // whether the mesh or the weapon root is the thing carrying it.
                if (!halo::weapon_drive_owns() && !halo::palettearm_weapon_owns()) {
                    for_each_rig([&](API::UObject* r) {
                        if (world_mode) rig_set_world_rotation(r, (double)wp, (double)wy, (double)wr);
                        else            rig_set_rotation(r, (double)rig_pitch, (double)rig_yaw, (double)c_roll);
                    });
                }
            }

            // The weapon's ACTUAL world rotation: parent composed with what we just wrote. Derived
            // rather than assumed so it stays correct in every rig mode and while calibrating, and
            // it is what both the pivot arm and a freeze snapshot must use.
            const Quat q_gun = quat_mul(q_parent, rotator_to_quat(rig_pitch, rig_yaw, c_roll));

#if HALO_VR_DEV
            // CALIBJUMP -- CONSECUTIVE driven frames across a calibration release.
            //
            // The first version of this lived inside the RIGTRACK block, which is gated on
            // `tick % 45`, so it sampled 1.5 s apart and could not see a transient at all; and it
            // differenced the parent's WORLD location against `pose_off`, which is a clamped
            // controller-relative offset, so the "delta" was just the world position echoed back.
            // Both readings were worthless. This one samples every frame while armed and differences
            // each quantity against ITSELF on the previous frame, which is the only comparison that
            // can show a jump.
            //
            // Ruled out so far: the pivot lever (CALIBSOLVE delta was exactly zero) and parent
            // motion (par_loc was static to 0.1 cm across the whole window).
            // WINDOW MUST SPAN THE RELEASE. Arming in the release handler starts the window one
            // frame too late -- `calibrating` has already gone false by then, so the first sample is
            // the settled state and the transition is never seen. It is the transition that matters:
            // everything after it measured as a hand holding still (d_gun sub-cm, d_par exactly 0),
            // so the discontinuity is at or before the release frame. Sample during the HOLD too,
            // thinned so a multi-second hold does not bury the log, then every frame afterwards.
            const bool cj_hold = calibrating && ((tick % 4) == 0);
            if (cj_hold || g_calibjump_arm > 0) {
                if (!calibrating && g_calibjump_arm > 0) --g_calibjump_arm;
                Vec3 gl{}, pl{};
                const bool gl_ok = call_ret_vec3(rig, L"K2_GetComponentLocation", &gl);
                const bool pl_ok = (g_rig_parent != nullptr)
                    && call_ret_vec3(g_rig_parent, L"K2_GetComponentLocation", &pl);
                float gp = 0.0f, gy = 0.0f, gr = 0.0f;
                quat_to_rotator(q_gun.x, q_gun.y, q_gun.z, q_gun.w, &gp, &gy, &gr);
                if (g_calibjump_have_prev) {
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] CALIBJUMP[%d] calibrating=%d | gun_loc=(%.1f,%.1f,%.1f) "
                        "d_gun=(%.1f,%.1f,%.1f) | par_loc=(%.1f,%.1f,%.1f) d_par=(%.1f,%.1f,%.1f) | "
                        "gun_rot=(p%.1f,y%.1f) d_rot=(p%.1f,y%.1f) | off=(%.1f,%.1f,%.1f) ok%d%d",
                        g_calibjump_arm, (int)calibrating,
                        gl.x, gl.y, gl.z,
                        gl.x - g_calibjump_gun.x, gl.y - g_calibjump_gun.y, gl.z - g_calibjump_gun.z,
                        pl.x, pl.y, pl.z,
                        pl.x - g_calibjump_par.x, pl.y - g_calibjump_par.y, pl.z - g_calibjump_par.z,
                        gp, gy, gp - g_calibjump_rot_p, gy - g_calibjump_rot_y,
                        g_cfg.off_x, g_cfg.off_y, g_cfg.off_z, (int)gl_ok, (int)pl_ok);
                }
                g_calibjump_gun = gl; g_calibjump_par = pl;
                g_calibjump_rot_p = gp; g_calibjump_rot_y = gy;
                g_calibjump_have_prev = true;
            } else if (!calibrating && g_calibjump_arm == 0) {
                // Only forget the baseline once the whole window is over. Clearing it on the frames
                // the hold-thinning skips would drop have_prev between every sample, so nothing
                // would ever have a previous frame to difference against and the hold side of the
                // window would log nothing at all.
                g_calibjump_have_prev = false;
            }
#endif

#if HALO_VR_DEV
            // RIGTRACK -- the entire hand->arms chain on one line, in order, so the term that fails
            // to respond to head or hand motion can be READ rather than inferred. Reasoning about
            // this statically produced two confident explanations that the logs then refuted, so
            // every intermediate now gets printed: room-space hand, the room->game yaw applied to
            // it, game-space hand, the parent it is made relative to, the relative value actually
            // written, and the world rotation that composes out. Whichever column stops tracking
            // the controller is the bug.
            if ((tick % 45) == 0) {
                float gp = 0.0f, gy = 0.0f, gr = 0.0f, hp = 0.0f, hy = 0.0f, hr = 0.0f;
                quat_to_rotator(q_gun.x,  q_gun.y,  q_gun.z,  q_gun.w,  &gp, &gy, &gr);
                quat_to_rotator(q_ctrl.x, q_ctrl.y, q_ctrl.z, q_ctrl.w, &hp, &hy, &hr);
                const float qturn_yaw = g_cfg.rig_turn * g_turn_offset.load() + calib_frame_yaw_use();

                // MEASURED, not predicted. gun_world above is what the maths says the component
                // should end up at; these two are what the engine reports it IS. Every wrong
                // conclusion today came from trusting a computed intermediate over a read-back, so
                // the comparison the harness actually needs is ctrl-vs-ACTUAL, not ctrl-vs-intent.
                Vec3 mrot{}, mloc{}, ploc{};
                const bool mrot_ok = call_ret_vec3(rig, L"K2_GetComponentRotation", &mrot);
                const bool mloc_ok = call_ret_vec3(rig, L"K2_GetComponentLocation", &mloc);
                // The PARENT's world location. With L and G both zero the rig's offset reduces to
                // the controller's position, which is unchanged under a pure rotation -- yet the rig
                // still moves ~30 cm. That can only come from the parent, so measure it directly
                // rather than infer it a sixth time.
                const bool ploc_ok = (g_rig_parent != nullptr)
                    && call_ret_vec3(g_rig_parent, L"K2_GetComponentLocation", &ploc);

                // (CALIBJUMP moved out of this block -- it is gated on `tick % 45`, which sampled
                // 1.5 s apart and could never show a release transient. See the new site above.)

                API::get()->log_info(
                    "[Halo-CampE-UEVR] RIGTRACK m=%d hand_room=(p%.1f,y%.1f) qturn=%.1f hand_game=(p%.1f,y%.1f)"
                    " parent=(p%.1f,y%.1f,read%d) wrote=(p%.1f,y%.1f) gun_pred=(p%.1f,y%.1f)"
                    " gun_meas=(p%.1f,y%.1f,ok%d) gun_loc=(%.1f,%.1f,%.1f,ok%d)"
                    " par_loc=(%.1f,%.1f,%.1f,ok%d)"
                    " ctrl_pos=(%.3f,%.3f,%.3f) | aim=%.1f view=%.1f hmd=%.1f direct=%d rigmode=%d",
                    g_cfg.dbg_mark,
                    g_pitch, g_yaw, qturn_yaw, hp, hy,
                    dbg_parent_pitch, dbg_parent_yaw, (int)dbg_parent_read,
                    rig_pitch, rig_yaw, gp, gy,
                    mrot.x, mrot.y, (int)mrot_ok,
                    mloc.x, mloc.y, mloc.z, (int)mloc_ok,
                    ploc.x, ploc.y, ploc.z, (int)ploc_ok,
                    g_dbg_pos_x.load(), g_dbg_pos_y.load(), g_dbg_pos_z.load(),
                    (float)aim_yaw, g_dbg_view_out.load(), g_dbg_hmd_yaw.load(),
                    (int)g_cfg.aim_direct, g_cfg.rig_mode);
            }
#endif
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
                // (The position half of the removed arm-bind snapshot was here. Same story as the
                // orientation half above, and the BINDSNAP instrument that measured it read 0.00 cm
                // on every calibration -- the two samples were always from the same update() call.)
                // WHAT THE HAND IS MEASURED FROM. This one subtraction decides whether the arms
                // follow your controllers or stay bolted to the pawn, so it is worth being exact:
                // it is the same term UEVR uses for every UObjectHook controller attachment, and it
                // is NOT the head.
                //
                // UObjectHook.cpp:1963-2054 composes an attached component as
                //     hand_world = (view_location - gamespace(hmd - standing_origin))    <- final_position
                //                                 - gamespace(hand - hmd)
                //                = view_location - gamespace(hand - STANDING ORIGIN)
                // -- the head term cancels exactly. The anchor is the STANDING ORIGIN: a fixed room
                // point, the same body reference the aim sightline already uses (aimorigin=1).
                //
                // We add our offset to the rig's PARENT, the game's CameraComponent, which stands in
                // for view_location. UEVR moves the rendered EYE off that camera by
                // (hmd - standing_origin) and leaves the camera itself alone -- so measuring the hand
                // from the standing origin is precisely what makes the arms land in the right place
                // relative to your eye, whatever your head is doing.
                //
                // Subtracting the LIVE HMD instead (what this did before) is the same thing ONLY
                // while the standing origin sits on your head -- which is exactly the state a leash
                // maintains. That is why it looked correct for months and fell apart the moment
                // hmdleash=0 let the two separate: the arms then held station at the pawn while the
                // eye walked away, and riganchor=0 could not help because it drops the anchor
                // altogether rather than fixing which one is used.
                Vec3 hand = rigpos;
                if (g_cfg.rig_body_anchor) {
                    const auto so = API::VR::get_standing_origin();
                    hand = Vec3{rigpos.x - so.x, rigpos.y - so.y, rigpos.z - so.z};
                }

                // Never capture a neutral from a position that does not exist -- it would bake the
                // phantom offset in permanently, and survive the runtime recovering.
                if (!g_rig_neutral_valid.load() && g_have_ref.load() && !position_dead) {
                    g_rig_neutral_x = hand.x; g_rig_neutral_y = hand.y; g_rig_neutral_z = hand.z;
                    g_rig_neutral_valid = true;
                }

                // NO TRUSTWORTHY NEUTRAL, OR NO CONTROLLER POSITION TO MEASURE ONE FROM.
                //
                // Hold the rig at origin rather than fling it to the clamp: a wrong offset is far
                // more visible than no offset, and with position missing EVERY offset is wrong
                // because `hand` resolves to minus the standing origin.
                //
                // THIS USED TO BE AN `else` OVER THE NEXT ~1000 LINES, AND THAT WAS THE BUG.
                // Measured 2026-08-30 and reported from a headset as "the reticule disappears and
                // the world-space one comes back": a POSITION-only tracking failure was disabling
                // the entire reticule/trace/compositor-publish group, which needs no controller
                // position at all -- its origin is the rig parent and its direction is the aim
                // angles. This gate's own comment said so ("Aim is unaffected: it comes from the
                // rotation, which is still live") while the code did the opposite.
                //
                // position_dead arms after 120 consecutive zero-position ticks (~3.8 s) and only
                // clears on a non-zero translation, so the outage was unbounded -- 17 to 42 s in
                // the reported session, nine arms in thirteen minutes, triggered by OpenXR focus
                // loss making the runtime publish exact-zero controller translation.
                //
                // So the protective action stays, and the block below now runs REGARDLESS. Only the
                // one thing that actually consumes the missing position -- the rig TRANSLATION
                // write near the end -- is suppressed, by this same flag.
                const bool hold_rig_at_origin = (!g_rig_neutral_valid.load() || position_dead);

                // WHICH BRANCH ENGAGED THE HOLD, AND FOR HOW LONG.
                //
                // Both triggers were invisible: publish gate 5 says the hold happened, never why. The
                // two have nothing in common -- position_dead is a runtime tracking outage, while an
                // invalid neutral is OUR OWN state, dropped on a rig-component change, a player
                // controller change, a recenter, or the aim reference going away. Reported as "the
                // arms stop tracking", they look identical from a headset.
                //
                // That ambiguity cost a session: a whole investigation went to OpenXR focus loss
                // while the logs showed position_dead arming exactly ZERO times. Edge-triggered, so
                // this is two lines per outage, not a per-tick spew on the frame path.
                {
                    static bool     s_hold_prev  = false;
                    static uint32_t s_hold_since = 0;
                    if (hold_rig_at_origin && !s_hold_prev) {
                        s_hold_since = tick;
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] RIG ORIGIN-HOLD ENGAGED via %s "
                            "(neutral_valid=%d position_dead=%d have_ref=%d). The rig is pinned to "
                            "its origin, so the arms and weapon will sit still until this releases.",
                            position_dead ? "POSITION_DEAD (runtime publishing exact-zero translation)"
                                          : "NO VALID RIG NEUTRAL (ours: rig/PC change, recenter, or "
                                            "the aim reference dropped)",
                            (int)g_rig_neutral_valid.load(), (int)position_dead,
                            (int)g_have_ref.load());
                    } else if (!hold_rig_at_origin && s_hold_prev) {
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] RIG ORIGIN-HOLD RELEASED after %u ticks.",
                            (unsigned)(tick - s_hold_since));
                    }
                    s_hold_prev = hold_rig_at_origin;
                }

                if (hold_rig_at_origin) {
                    xrlayer_note_publish_gate(5);
                    for_each_rig([&](API::UObject* r) { rig_set_location(r, 0.0, 0.0, 0.0); });
                }
                {
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

                // ROOM DISPLACEMENT -> GAME-SPACE RIG DISPLACEMENT, as ONE named transform.
                //
                // It was inline until the calibration hold needed the same mapping for the head's
                // own displacement. Two copies of this would be two things to keep in step -- the
                // rotation offset, the world scale and the VR->UE axis swap -- and a hold that
                // mapped displacement even slightly differently from the live pose would put the
                // error straight into the fit, which is the one place it must not go.
                //
                // Same turn correction as the orientation: if only one of them gets it they
                // disagree and the weapon slides sideways with every snap.
                auto vr_to_rig = [&](const Vec3& v) {
                    const Vec3 r = quat_rotate(q_ro, v);
                    return quat_rotate(q_turn, Vec3{-r.z * g_cfg.rig_scale,
                                                     r.x * g_cfg.rig_scale,
                                                     r.y * g_cfg.rig_scale});
                };

                Vec3 pose_off = vr_to_rig(d_vr);

                // ---- THE CALIBRATION HOLD RIDES YOUR HEAD.
                //
                // While the weapon is frozen it is held at a fixed offset from the CAMERA, which is
                // fine only while your head is at the camera. Unleashed it is not: walk during the
                // hold and the weapon you are being asked to put your hand on floats away toward
                // your abandoned body, so the match is physically impossible.
                //
                // Worse, it silently corrupts the fit. The solve is
                //     L = R_ctrl^-1 * (off_frozen - pose_off_release + R_gun*G)
                // and pose_off carries the head displacement, so what lands in L is
                //     P(delta_freeze - delta_release)
                // -- HEAD MOVEMENT DURING THE HOLD, baked into the mount as a constant and carried
                // to disk. (A CONSTANT displacement was always harmless: it is present in both the
                // frozen capture and the release solve and cancels. Only the change matters.)
                //
                // So the held offset tracks that same displacement. The weapon then holds still
                // relative to YOU, and P(delta) appears on both sides of the solve and cancels
                // exactly, leaving a fit built only from head-RELATIVE hand positions. Correct by
                // construction rather than by asking the player to keep still.
                //
                // Zero, and skipped entirely, whenever the head is leashed -- the displacement is
                // then always zero, so this cannot alter an existing calibration.
                Vec3 hold_ride{0.0f, 0.0f, 0.0f};
                if (calibrating || g_calib_finish.load()) {
                    Vec3 hp{}; Quat hq{};
                    const auto hidx = API::VR::get_hmd_index();
                    if (g_calib_have_delta && hidx >= 0
                        && get_pose(hidx, &hp, &hq, /*use_aim=*/false)) {
                        const auto so2 = API::VR::get_standing_origin();
                        // Linear, so the difference of the two mapped displacements is the mapping
                        // of their difference -- taken against the LIVE frame, which is the one
                        // this frame's pose_off was built in.
                        hold_ride = vr_to_rig(Vec3{(hp.x - so2.x) - g_calib_delta_room.x,
                                                   (hp.y - so2.y) - g_calib_delta_room.y,
                                                   (hp.z - so2.z) - g_calib_delta_room.z});
                    }
                }
                const Vec3 calib_off_held{g_calib_off_world.x + hold_ride.x,
                                          g_calib_off_world.y + hold_ride.y,
                                          g_calib_off_world.z + hold_ride.z};

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
                // ---- LIVE WEAPON-TO-RIG SEPARATION.
                //
                // The weapon is a separate ACTOR socketed onto this mesh, so
                //     weapon_world = component_world + S
                // and S is the only term the placement below has to cancel to put the WEAPON --
                // rather than the mesh -- on the controller. Measuring it costs two reflected reads
                // and is exact; the alternative is g_cfg.piv_*, a constant that is only ever right
                // by luck.
                //
                // MEASURED, BECAUSE THE CONSTANT GOES STALE. Log evidence: |S| held at 65.9 cm for
                // 25 s, then at 37.2 cm for the 30 s after a respawn re-acquired the rig -- a
                // 28.7 cm step, with off/grip/piv byte-identical either side. |piv| is 37.5 cm, so
                // the cancellation was wrong before the death and right after it, and the fitted
                // mount had silently absorbed the difference. Pinning the pivot stops OUR value
                // drifting; it cannot stop the actual attachment from changing.
                //
                // |S| is constant within each era rather than varying with the animation, so these
                // are two component instances with different geometry -- which also fits the
                // author's "43.7 cm across one mission restart" note better than a mid-pose sample
                // does. Reading it live makes instance changes, respawns and weapon swaps stop
                // mattering, and feeds back nowhere: both terms move together, so S is independent
                // of where we put the mesh.
                // wpn_drive CONSUMES this measurement -- it is where the latched socket comes from,
                // so the mode cannot engage without it. Adding it to the gate rather than making the
                // player also remember rigsocket=1 is deliberate: a mode that silently never engages
                // because a second key was off is indistinguishable from a mode that does not work.
                // COOLING CHECK FIRST. socket_sample faults at the identical UEVR instruction as
                // the reticule trace (UObjectBase::update_offsets), and without this it retried
                // every tick forever -- which is what still cost the arms after the trace lane
                // had correctly backed off.
                if ((g_cfg.rig_socket || g_cfg.wpn_diag || g_cfg.rig_sock_rot || g_cfg.wpn_drive)
                    && !lane_cooling(PERF_SOCK, tick)) {
                    PerfScope _perf(PERF_SOCK);
                    Vec3 cw{}, ww{};
                    auto* wact = fp_weapon_actor();
                    const bool sep_ok = (wact != nullptr)
                                     && call_ret_vec3(rig,  L"K2_GetComponentLocation", &cw)
                                     && call_ret_vec3(wact, L"K2_GetActorLocation",     &ww);
                    g_dbg_wpn_ok = sep_ok;
                    if (sep_ok) {
                        g_dbg_wpn_dx = ww.x - cw.x;
                        g_dbg_wpn_dy = ww.y - cw.y;
                        g_dbg_wpn_dz = ww.z - cw.z;
                    }
                    // The same offset in the MESH's frame, plus the socket's rotation there.
                    // World-space sep rotates as you aim, so it cannot be compared between two
                    // samples taken at different poses; these can. Rotation is measured because
                    // the placement cancels the socket's TRANSLATION only -- if the socket is also
                    // mounted at a different ANGLE on the new instance, nothing cancels that and
                    // the weapon tilts, which would read as "slightly off" on every weapon at once.
                    Vec3 crot{}, wrot{};
                    if (sep_ok
                        && call_ret_vec3(rig,  L"K2_GetComponentRotation", &crot)
                        && call_ret_vec3(wact, L"K2_GetActorRotation",     &wrot)) {
                        const Quat q_comp = rotator_to_quat(crot.x, crot.y, crot.z);
                        const Quat q_wpn  = rotator_to_quat(wrot.x, wrot.y, wrot.z);
                        const Vec3 s_loc  = quat_rotate(quat_conj(q_comp),
                                                        Vec3{ww.x - cw.x, ww.y - cw.y, ww.z - cw.z});
                        g_dbg_sock_x = s_loc.x; g_dbg_sock_y = s_loc.y; g_dbg_sock_z = s_loc.z;
                        const Quat q_rel = quat_mul(quat_conj(q_comp), q_wpn);
                        float sp = 0.0f, sy = 0.0f, sr = 0.0f;
                        quat_to_rotator(q_rel.x, q_rel.y, q_rel.z, q_rel.w, &sp, &sy, &sr);
                        g_dbg_sock_p = sp; g_dbg_sock_yw = sy; g_dbg_sock_r = sr;
                        g_dbg_sock_ok = true;
                    } else {
                        g_dbg_sock_ok = false;
                    }

                    static API::UObject* s_prev_parent = nullptr;
                    if (g_rig_parent != s_prev_parent) {
                        s_prev_parent = g_rig_parent;
                        ++g_dbg_parent_changes;
                    }
                }

                // IMMEDIATELY AFTER the socket measurement, and not before it: this is what takes
                // the latch, and it may only take it from a reading made while the weapon root is
                // still unwritten. Resolving the targets here also keeps reflection off the render
                // callback, which is the only other place this module runs.
                halo::weapon_drive_tick(g_on_foot_unarmed);

                // ENGAGE EDGE -- hand the mesh back to the game exactly once.
                //
                // Our last write is still sitting in RelativeLocation/RelativeRotation, so merely
                // CEASING to write would freeze the arms at whatever pose the controller happened to
                // be in at handover. That reads as "the arms stopped tracking", which is the very
                // symptom this mode exists to fix, and it would look like the mode failing at the
                // instant it started working. Zero is the same neutral the no-trustworthy-offset
                // path above falls back to.
                {
                    static bool s_wd_engaged = false;
                    const bool wd_owns = halo::weapon_drive_owns();
                    if (wd_owns != s_wd_engaged) {
                        s_wd_engaged = wd_owns;
                        if (wd_owns) {
                            for_each_rig([&](API::UObject* r) {
                                rig_set_location(r, 0.0, 0.0, 0.0);
                                rig_set_rotation(r, 0.0, 0.0, 0.0);
                            });
                            API::get()->log_info("[Halo-CampE-UEVR] weapon drive: ENGAGED -- arm mesh "
                                                 "released to neutral, gun now on the weapon root");
                        } else {
                            // HAND THE GUN BACK before legacy takes over. Without this our last R
                            // stays written on the weapon root and legacy composes on top of it --
                            // the gun stays wrong while the log claims it fell back cleanly. Only a
                            // weapon swap (a fresh actor) used to clear it.
                            halo::weapon_drive_release();
                            API::get()->log_info("[Halo-CampE-UEVR] weapon drive: DISENGAGED -- "
                                                 "weapon root released to identity, legacy resumes");
                        }
                    }
                }
                // Usable only with a weapon in hand: with none there is nothing to cancel, and the
                // last reading describes a weapon that is gone. Fall back to the pivot then.
                // GROUND TRUTH: solve back for the mount the engine actually produced.
                //
                //   weapon_world = parent_world + pose_off + R_ctrl * L
                // so   L_actual = conj(R_ctrl) * (weapon_world - parent_world - pose_off)
                //
                // parent_world and weapon_world are both live reads and pose_off comes from the
                // controller, not from anything we wrote -- so this is independent of our output.
                // Checking against our own written `off` would be a tautology that reports success
                // no matter what the engine actually did with it.
                // wpn_drive NEEDS this: it is the only check on the weapon drive that does not
                // consume the drive's own S, and is therefore the only one that can catch S drifting.
                if ((g_cfg.wpn_diag || g_cfg.wpn_drive)
                    && g_dbg_wpn_ok.load() && g_rig_parent != nullptr) {
                    Vec3 pw{}, wwld{};
                    auto* wa = fp_weapon_actor();
                    if (wa != nullptr
                        && call_ret_vec3(g_rig_parent, L"K2_GetComponentLocation", &pw)
                        && call_ret_vec3(wa,           L"K2_GetActorLocation",     &wwld)) {
                        const Vec3 rel{wwld.x - pw.x - pose_off.x,
                                       wwld.y - pw.y - pose_off.y,
                                       wwld.z - pw.z - pose_off.z};
                        const Vec3 La = quat_rotate(quat_conj(q_ctrl), rel);
                        g_dbg_Lact_x = La.x; g_dbg_Lact_y = La.y; g_dbg_Lact_z = La.z;
                        g_dbg_Lact_ok = true;
                    } else {
                        g_dbg_Lact_ok = false;
                    }
                }

                // THE SOCKET IN THE MESH'S OWN FRAME, not the world-space separation.
                //
                // Both describe the same geometry, but the world one is read from the PREVIOUS
                // frame's transforms, so the moment the mesh rotates it is stale by exactly the
                // rotation that happened since -- the weapon lags, then settles when you stop.
                // Measured with the world value: |ERR| median 0.1 cm at rest, peaks of 4-5 cm while
                // turning, identical before and after a respawn (so it was never the respawn).
                //
                // The LOCAL offset is a fixed property of the skeleton -- logged constant at
                // (61.2,13.1,-23.5) across a whole session -- so reading it a frame late costs
                // nothing. Rotating it by the q_gun we are about to apply gives this frame's
                // separation with no lag by construction. Same shape as the pivot fallback below;
                // the only difference is that this constant is measured rather than guessed.
                const bool  sep_live = g_cfg.rig_socket && g_dbg_wpn_ok.load() && g_dbg_sock_ok.load();
                const Vec3  sock_local{g_dbg_sock_x.load(), g_dbg_sock_y.load(), g_dbg_sock_z.load()};
                const Vec3 G{g_cfg.piv_x, g_cfg.piv_y, g_cfg.piv_z};

                // ---- CALIBRATION RELEASE: solve the placement that puts the weapon where the
                // hand is. The model being solved is
                //     off = pose + C - R*G
                // so with R pinned to the frozen orientation by the rotation trim above,
                //     C = off_frozen - pose + R_frozen*G
                if (g_calib_finish.exchange(false) && g_calib_valid) {
                    // Strip the calibration frame as a ROTATION (see calib_frame_yaw_use). The use
                    // path renders  gun = R_yaw(frame) * q_ctrl * q_grip,  so inverting it gives
                    //   q_grip = conj(R_yaw(frame) * q_ctrl) * gun_world
                    // and the stored trim is frame-independent by construction. With the feature
                    // off this is the identity and the solve is byte-identical to before.
                    // The DELTA, not the whole frame: q_ctrl already carries calib_frame_yaw_use()
                    // via q_turn, so the solve strips that much for free. Only the difference is
                    // outstanding, and it is non-zero in exactly one situation -- upgrading a v1
                    // file, where write=locked and use=0. Subtracting the full frame there would
                    // double-count it for every already-v2 calibration.
                    const float frame_delta = calib_frame_yaw_write() - calib_frame_yaw_use();
                    const Quat  q_frame_w   = rotator_to_quat(0.0f, frame_delta, 0.0f);
                    // The frozen value is the MESH orientation. With the socket rotation cancelled
                    // on the apply side, the grip must be fitted against the WEAPON -- mesh times
                    // socket -- or the fit and the apply describe different objects and the solve
                    // is wrong by exactly the socket rotation.
                    Quat q_calib_target = g_calib_gun_world;
                    if (g_cfg.rig_sock_rot && g_dbg_sock_ok.load()) {
                        q_calib_target = quat_mul(q_calib_target, rotator_to_quat(
                            g_dbg_sock_p.load(), g_dbg_sock_yw.load(), g_dbg_sock_r.load()));
                    }
                    const Quat q_grip_new = quat_mul(quat_conj(quat_mul(q_frame_w, q_ctrl)),
                                                     q_calib_target);
                    const float roll_keep = g_cfg.grip_roll;
                    quat_to_rotator(q_grip_new.x, q_grip_new.y, q_grip_new.z, q_grip_new.w,
                                    &g_cfg.grip_deg, &g_cfg.grip_yaw, &g_cfg.grip_roll);

                    // OPTIONALLY HOLD ROLL (calibroll=0). Must be a ROTATION, not a scalar swap.
                    //
                    // Euler components are not independent: pitch/yaw/roll from one decomposition
                    // reproduce the fitted orientation only as a SET. Keeping the old roll while
                    // taking the new pitch and yaw builds a rotation that was never fitted, and it
                    // moves where the gun POINTS rather than only how it is twisted -- which is
                    // exactly what "calibration stopped working" looked like when this was first
                    // written that way.
                    //
                    // Post-multiplying rotates about the weapon OWN forward axis, so the pointing
                    // the solve just fitted survives by construction and only the twist changes.
                    if (!g_cfg.calib_roll) {
                        const Quat q_hold = quat_mul(q_grip_new,
                            rotator_to_quat(0.0f, 0.0f, roll_keep - g_cfg.grip_roll));
                        quat_to_rotator(q_hold.x, q_hold.y, q_hold.z, q_hold.w,
                                        &g_cfg.grip_deg, &g_cfg.grip_yaw, &g_cfg.grip_roll);
                    }

                    // MODE 3 READS A DIFFERENT TRIM. The direct-drive rig composes against
                    // rig_dir_grip_* (Plugin.cpp, the rig_mode==3 branch) specifically so it does
                    // not inherit mode 2's fitted pivot -- but nothing ever wrote those, so
                    // calibrating while in mode 3 fitted grip_* and then silently changed nothing,
                    // leaving the arms on a zeroed trim (i.e. pitched up by the whole fitted angle).
                    // The composition is identical in both branches, so the fit transfers exactly.
                    g_cfg.rig_dir_grip_deg  = g_cfg.grip_deg;
                    g_cfg.rig_dir_grip_yaw  = g_cfg.grip_yaw;
                    g_cfg.rig_dir_grip_roll = g_cfg.grip_roll;
                    // (The frame is stripped by the rotation above, NOT by adjusting gripyaw.)
                    // Stamp the schema so the use path knows these values are frame-relative.
                    if (g_cfg.calib_relative) g_cfg.calib_ver = 2;

                    // ONE SAMPLE IS SUFFICIENT. A held weapon is a RIGID ATTACHMENT, which has no
                    // free pivot parameter: physics fixes the rotation centre at the controller
                    // origin. Solving for a floating pivot invents a degree of freedom that does
                    // not exist, and any value found really absorbs a translation-SCALE error.
                    //
                    //   off = pose + R_ctrl*L - R_gun*G
                    // with G normally 0, so:
                    //   L = inverse(R_ctrl) * (off_frozen - pose + R_gun_frozen*G)
                    const Vec3 G_now{g_cfg.piv_x, g_cfg.piv_y, g_cfg.piv_z};
                    // calib_off_held, NOT g_calib_off_world: the hold rides the head, so the value
                    // the player was actually matching against is the ridden one. Solving against
                    // the raw capture instead would both re-import the displacement this is meant
                    // to cancel AND put a visible jump at release, since the weapon would leave the
                    // place it was just being held.
                    //
                    // And it must use whatever the LIVE path will subtract: with rigsocket on,
                    // that is the frozen socket in the mesh frame, not the animated G sampled
                    // now. Solving against one geometry and applying against another is the
                    // 28.7 cm post-respawn error rigsocket exists to remove (PR #7).
                    const Vec3 arm_frozen = quat_rotate(g_calib_gun_world,
                                                        sep_live ? g_calib_sock_local : G_now);
                    const Vec3 resid{calib_off_held.x - pose_off.x + arm_frozen.x,
                                     calib_off_held.y - pose_off.y + arm_frozen.y,
                                     calib_off_held.z - pose_off.z + arm_frozen.z};
                    const Vec3 L = quat_rotate(quat_conj(q_ctrl), resid);
                    g_cfg.off_x = clampf(L.x, -100.0f, 100.0f);
                    g_cfg.off_y = clampf(L.y, -100.0f, 100.0f);
                    g_cfg.off_z = clampf(L.z, -100.0f, 100.0f);

                    // MODE 3 READS A DIFFERENT MOUNT, exactly as it reads a different grip trim --
                    // and this is the other half of that same fix, which was missed.
                    //
                    // The use path takes its mount from rig_dir_off_* when rig_mode == 3, but the
                    // solve only ever wrote off_*. So in mode 3 the POSITION half of this gesture
                    // did nothing: rig_dir_off_* stayed at whatever it was (zero, for anyone who
                    // never hand-edited it), the weapon was unpinned from the frozen offset on
                    // release, and it jumped to where a zero mount puts it. The size of that jump
                    // is the distance the hand travelled during the hold -- which is precisely the
                    // gesture, so the calibration APPEARED to move the weapon by however much you
                    // moved to perform it.
                    //
                    // Rotation was unaffected and felt correct throughout, which is what made this
                    // hard to see: the grip half had already been fixed, the mount half had not.
                    g_cfg.rig_dir_off_x = g_cfg.off_x;
                    g_cfg.rig_dir_off_y = g_cfg.off_y;
                    g_cfg.rig_dir_off_z = g_cfg.off_z;

                    // PIN THE PIVOT TO THIS FIT.
                    //
                    // L was just solved against the G that is live RIGHT NOW. The use path applies
                    // (pose + R_ctrl*L - R_gun*G), so the moment G changes, L is describing a
                    // geometry that no longer exists and the arms move by |R_gun * dG|.
                    //
                    // And G does change, because pivauto re-reads the PrimaryWeapon socket on every
                    // full rig resolve -- off the LIVE ANIMATED skeleton, so it samples whatever the
                    // arms happened to be doing that frame. Measured across one mission restart:
                    // (61.6,13.1,-23.5) -> (20.2,12.6,-37.5), a 43.7 cm swing, which is exactly the
                    // "arms are offset after Restart Mission" report. Aim was unaffected because
                    // nothing in the aim path consumes G, which is what made it look like a rig bug
                    // rather than a shared one.
                    //
                    // So the calibration takes ownership of the pivot: freeze the value it was
                    // fitted against and stop re-deriving. write_calib_file() persists it as
                    // pivauto=0 plus pivx/y/z, so the pairing survives a restart too.
                    g_pivot_from_calib = true;
                    g_cfg.piv_auto     = false;

                    // Deliberately NOT touching g_have_ref. The aim loop ran normally throughout,
                    // so its hand-to-aim mapping is still valid -- re-referencing here would
                    // silently re-calibrate aim as a side effect of a mesh adjustment.
                    // PER-WEAPON CAPTURE CLAIMS THE RESULT.
                    //
                    // Both keys run the same solve, so by here g_cfg holds freshly fitted values
                    // either way. wpn_calib_capture() stores them as a delta for the weapon in
                    // hand and returns true, and the global calibration is then deliberately left
                    // untouched -- overwriting it would undo the base that every unlisted weapon
                    // depends on.
                    if (!wpn_calib_capture()) {
                    // Rebase before persisting: the solve measured this weapon WITH its delta on,
                    // so the base that file should hold is the fit minus that delta.
                    weapon_offset_adopt_solve();
                    write_calib_file();
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] CALIBRATED: grip=%.1f gripyaw=%.1f griproll=%.1f  mount=(%.1f,%.1f,%.1f)cm controller-local",
                        g_cfg.grip_deg, g_cfg.grip_yaw, g_cfg.grip_roll,
                        g_cfg.off_x, g_cfg.off_y, g_cfg.off_z);
                    }   // end: global write skipped when the per-weapon key claimed the capture

#if HALO_VR_DEV
                    // CALIBSOLVE -- every term of the mount solve, so it can be READ rather than
                    // inferred. `resid` and `L` are the useful fields.
                    //
                    // âš ï¸ THE `delta` FIELD IS TAUTOLOGICAL. IT CANNOT DETECT ANYTHING.
                    //
                    // It was written to compare the frozen gun rotation against the live one, on the
                    // theory that the pivot lever multiplies any difference into centimetres of
                    // snap. But `arm_live` is built from q_grip_new, which was solved one line
                    // earlier as conj(q_frame * q_ctrl) * gun_world_frozen -- so q_ctrl * q_grip_new
                    // is IDENTICALLY gun_world_frozen whenever frame_delta is 0, and delta can only
                    // ever print (0,0,0). It did, on every run, and that zero was then cited as
                    // evidence that the pivot was not a factor.
                    //
                    // It was. Not through this term -- the pivot bug was G being RE-DERIVED between
                    // the fit and its use (43.7 cm across a mission restart, since fixed by pinning
                    // G to the calibration). A frozen-vs-live comparison at the instant of the solve
                    // could never have seen that; it needed the value compared across sessions.
                    //
                    // Kept because G/pivauto/fromcalib/resid/L are all worth reading. Do not read
                    // `delta` as a measurement of anything.
                    {
                        // Gun world rotation that will apply AFTER release. Mode 3 writes
                        // q_world = q_parent * (conj(q_parent) * q_ctrl * q_grip) = q_ctrl * q_grip,
                        // so the parent cancels and this is the whole of it.
                        const Vec3 arm_live = quat_rotate(quat_mul(q_ctrl, q_grip_new), G_now);
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] CALIBSOLVE: piv G=(%.1f,%.1f,%.1f) pivauto=%d fromcalib=%d | "
                            "armG frozen=(%.1f,%.1f,%.1f) live=(%.1f,%.1f,%.1f) delta=(%.1f,%.1f,%.1f)cm | "
                            "resid=(%.1f,%.1f,%.1f) L=(%.1f,%.1f,%.1f)",
                            G_now.x, G_now.y, G_now.z, (int)g_cfg.piv_auto, (int)g_pivot_from_calib,
                            arm_frozen.x, arm_frozen.y, arm_frozen.z,
                            arm_live.x, arm_live.y, arm_live.z,
                            arm_live.x - arm_frozen.x, arm_live.y - arm_frozen.y,
                            arm_live.z - arm_frozen.z,
                            resid.x, resid.y, resid.z, L.x, L.y, L.z);
                    }

                    // CALIBJUMP -- arm the post-release comparison. Records the weapon's ACTUAL world
                    // transform across the calibrating 1 -> 0 frame, so a release discontinuity can
                    // be read instead of argued about.
                    //
                    // The release IS seamless on paper -- substituting the solve into the runtime
                    // application gives off = off_frozen exactly -- so anything visible at the
                    // transition is in a term the algebra does not cover. That reasoning stands; the
                    // conclusion originally drawn from it ("therefore not the pivot", from
                    // CALIBSOLVE's delta) does not: see the warning above about that field.
                    //
                    // The two release-frame jumps that were real turned out to be (a) rigmode 3
                    // reading a mount the solve never wrote, and (b) G changing between the fit and
                    // its use. Both are fixed; this stays for the next one.
                    //
                    // Frames to keep sampling AFTER the release. The pre-release side is covered by
                    // the `calibrating` branch at the sampling site, so the window spans 1 -> 0.
                    // Deliberately NOT clearing have_prev: the last hold sample is the frame we most
                    // need to difference the first post-release frame against.
                    g_calibjump_arm = 12;
#endif
                }

                // RIGID ATTACHMENT:  off = pose + R_ctrl*L - R_gun*G
                //
                // The mount offset L rotates WITH THE CONTROLLER -- that is what makes this a rigid
                // attachment rather than a weapon sliding around on a world-aligned offset, and it
                // puts the rotation centre at the controller origin where a held object's is.
                // G stays 0 unless someone deliberately overrides the pivot.
                // Mode 3 uses its OWN mount offset. Mode 2's off_* carries a folded-in pivot, so
                // borrowing it here would import exactly the error this mode exists to avoid.
                const Vec3 mount_local = (g_cfg.rig_mode == 3)
                    ? Vec3{g_cfg.rig_dir_off_x, g_cfg.rig_dir_off_y, g_cfg.rig_dir_off_z}
                    : Vec3{g_cfg.off_x, g_cfg.off_y, g_cfg.off_z};
                const Vec3 mount = quat_rotate(q_ctrl, mount_local);
                // MEASURED FIRST, pivot only as the fallback. With the measurement the identity is
                // weapon_world = parent + pose + R_ctrl*L exactly, every frame, whatever the mesh
                // instance underneath happens to be.
                const Vec3 arm   = quat_rotate(q_gun, sep_live ? sock_local : G);
                Vec3 off{pose_off.x + mount.x - arm.x,
                         pose_off.y + mount.y - arm.y,
                         pose_off.z + mount.z - arm.z};

                // ---- THE GRAB GUIDE, and the zone measurement it shares with the hold ----------
                //
                // HERE, immediately after `off`, and not earlier where it first lived. The identity
                // one line up is the whole reason:
                //
                //     weapon_world = parent + pose_off + mount - arm
                //
                // so the aim CONTROLLER sits at (parent + pose_off) while the RIG ORIGIN sits at
                // (parent + pose_off + mount - arm). They are NOT the same point -- they differ by
                // exactly the mount offset, which is what puts the gun in your hand rather than
                // through it.
                //
                // The first version measured the hand against the CONTROLLER and drew it relative
                // to the RIG, so the whole beam was displaced by (mount - arm). Field report:
                // "the cylinder is off somewhere high and to the left of my right controller" --
                // which is precisely where the gun sits relative to the hand holding it. Adding
                // (arm - mount) moves the origin onto the rig, so measurement and drawing finally
                // share a point.
                //
                // This still has to be in this block: vr_to_rig() is the single blessed VR-to-game
                // transform and mount/arm/q_gun exist nowhere else. two_hand_update() runs later in
                // the SAME tick, so what is published here is what the zone test sees.
                {
                    halo::TwoHandZoneMeas meas{};
                    if (g_rigw_valid.load()) {
                        const int32_t sidx = g_cfg.aim_left_hand ? API::VR::get_right_controller_index()
                                                                 : API::VR::get_left_controller_index();
                        Vec3 spos{}; Quat sq{};
                        Vec3 apos{}; Quat aq{};
                        // BOTH raw, from the same source. Their DIFFERENCE is what matters, and a
                        // difference is only meaningful if both sides share an origin -- `hand`
                        // above is head-relative and must not be mixed in here.
                        if (sidx >= 0 && get_pose(sidx, &spos, &sq, /*use_aim=*/false)
                                      && get_pose(ridx, &apos, &aq, /*use_aim=*/false)) {
                            const Quat qg{g_rigw_x.load(), g_rigw_y.load(),
                                          g_rigw_z.load(), g_rigw_w.load()};
                            const Vec3 d{spos.x - apos.x, spos.y - apos.y, spos.z - apos.z};
                            const Vec3 v = vr_to_rig(d);              // VR offset -> world axes
                            // ZONE: origin stays on the AIM GRIP, only the AXIS becomes the gun's.
                            // The bounds are authored as "this far forward of your hand", so
                            // moving the origin onto the rig shifted every reading by the
                            // grip-to-rig distance and drove along negative. See TwoHandZoneMeas.
                            meas.hand_gun = quat_rotate(quat_conj(qg), v);
                            // GUIDE: the grip-to-rig difference, carried separately because the
                            // guide draws relative to the rig component and does need it.
                            const Vec3 ro{arm.x - mount.x, arm.y - mount.y, arm.z - mount.z};
                            meas.rig_off  = quat_rotate(quat_conj(qg), ro);
                            meas.valid    = true;
                        }
                    }
                    halo::two_hand_set_zone_measurement(meas);

                    const auto& reach = halo::two_hand_reach();
                    // in_zone is the hold's own acquisition gate: true exactly when a grip press
                    // right now would latch. Showing the beam on anything looser would make it a
                    // hint rather than a promise, and a promise is what makes it worth drawing.
                    if (reach.valid && reach.in_zone && !reach.latched && meas.valid) {
                        // Both already in the gun's frame, which is the frame the component draws
                        // in. The target is simply "this far down the barrel" -- the SAME number
                        // the zone test used, so the beam cannot point somewhere that will not
                        // latch.
                        // PHYSICAL metres -> GAME centimetres, so rig_scale again and not 100 --
                        // the same conversion the zone test uses, inverted. Using 100 here would
                        // draw the target 31% short of the spot that actually latches, which is
                        // the guide lying about the one thing it exists to promise.
                        const float cm_per_m = (g_cfg.rig_scale > 1.0f) ? g_cfg.rig_scale : 100.0f;
                        // Both endpoints are measured from the AIM GRIP, but the component draws
                        // relative to the RIG -- so shift both by the grip-to-rig offset. Shifting
                        // BOTH keeps the line's length and direction identical; it only moves
                        // where it is anchored.
                        const Vec3 h{meas.hand_gun.x + meas.rig_off.x,
                                     meas.hand_gun.y + meas.rig_off.y,
                                     meas.hand_gun.z + meas.rig_off.z};
                        const Vec3 t{reach.clamped_along_m * cm_per_m + meas.rig_off.x,
                                     meas.rig_off.y, meas.rig_off.z};

                        // ---- THE COMPOSITOR ROUTE, PREFERRED.
                        //
                        // In the scene the beam was occluded by the very weapon it lies along, took
                        // the game's lighting and exposure so it dimmed exactly where it mattered,
                        // and an emissive bright enough to read would bloom. On the layer none of
                        // those exist, because it never enters the scene. Same argument the
                        // reticule and the navpoint markers are already there for.
                        //
                        // Falls back to the mesh when the layer is not live, rather than vanishing
                        // -- a player without the compositor route still gets the affordance.
                        bool on_layer = false;
                        Vec3 rig_world{};
                        const bool lay_live = halo::xrlayer_live();
                        const bool have_rw  = call_ret_vec3(rig, L"K2_GetComponentLocation",
                                                            &rig_world);
                        // WHICH ROUTE THE GUIDE TOOK, RE-STATED WHENEVER THE REASON CHANGES.
                        //
                        // This was a ONE-SHOT `static bool s_said` and that made it useless at the
                        // exact moment it was needed (2026-09-05). It fired early in a session,
                        // while the compositor layer was still coming up, and then went silent --
                        // so when the layer DID come up and the guide still failed to appear, the
                        // instrument added for precisely that question had already spent itself.
                        // An instrument that reports once reports about a moment, not a state.
                        //
                        // Edge-triggered on the REASON, so a steady state costs nothing and every
                        // transition is recorded -- including the transition INTO working, which
                        // the old version could never say at all. "It is on the layer now" is the
                        // line that was missing: absence of a complaint is not evidence of success,
                        // the same trap as inferring the atlas cell from a missing warning.
                        {
                            enum : int { R_LAYER = 0, R_NO_LAYER = 1, R_NO_RIG_WORLD = 2 };
                            const int reason = !lay_live ? R_NO_LAYER
                                             : !have_rw  ? R_NO_RIG_WORLD
                                                         : R_LAYER;
                            static int s_prev = -1;
                            if (reason != s_prev) {
                                s_prev = reason;
                                if (reason == R_LAYER) {
                                    API::get()->log_info(
                                        "[Halo-CampE-UEVR] GRABGUIDE: ON THE COMPOSITOR LAYER "
                                        "(slot %d) -- unoccluded, unlit, no bloom.",
                                        halo::XRLAYER_SLOT_GUIDE);
                                } else {
                                    API::get()->log_info(
                                        "[Halo-CampE-UEVR] GRABGUIDE: not on the compositor layer "
                                        "(xrlayer_live=%d rig_world_read=%d) -- drawing the in-scene "
                                        "mesh instead, which IS occluded by the weapon and IS lit by "
                                        "the scene. This line repeats whenever the reason changes.",
                                        (int)lay_live, (int)have_rw);
                                }
                            }
                        }
                        if (lay_live && have_rw) {
                            // Gun frame -> world. One rotation each; the endpoints are already in
                            // the gun's frame relative to the rig, which is where rig_world sits.
                            const Vec3 hw = quat_rotate(q_gun, h);
                            const Vec3 tw = quat_rotate(q_gun, t);
                            const Vec3 a{rig_world.x + hw.x, rig_world.y + hw.y, rig_world.z + hw.z};
                            const Vec3 b{rig_world.x + tw.x, rig_world.y + tw.y, rig_world.z + tw.z};
                            const Vec3 d{b.x - a.x, b.y - a.y, b.z - a.z};
                            const float len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);

                            // PERIODIC, NOT EDGE-TRIGGERED, and the difference is the whole point.
                            //
                            // The too-short line below fires on the TRANSITION into too-short, so
                            // every sample it can produce is taken at the instant the length
                            // crosses grabguidemin downward. Two such samples read 1.47 and 1.30
                            // and were reasonably-but-wrongly read as "the length is pinned near
                            // 1.4, so the endpoints must be the same point". They are just under
                            // the threshold BY CONSTRUCTION; an edge sampler cannot say anything
                            // about the distribution, and it cannot show a sweep.
                            //
                            // (It also quietly proves the opposite: to transition INTO too-short
                            // the beam must have been >= 1.50 just before, twice. So it does vary
                            // and it did publish.)
                            //
                            // ~2 s while IN-ZONE ONLY, which is rare and brief, so this is a
                            // handful of lines per grab rather than a stream. Reports the geometry
                            // it is derived from as well, so a constant `len` can be attributed to
                            // a constant lateral rather than guessed at.
                            //
                            // DEV-ONLY. This answers a question rather than playing the game, so
                            // per DevTools.hpp it is not compiled into a player build at all. It
                            // shipped un-gated at first and a RELEASE build was caught emitting
                            // ten of these lines in one three-minute session -- harmless in cost
                            // (a 2 s throttle, in-zone only, no array walk) but exactly the kind
                            // of thing the compile-time gate exists to make impossible rather than
                            // unlikely. A throttle is not a gate.
#if HALO_VR_DEV
                            {
                                static uint32_t s_last = 0;
                                if (tick - s_last >= 120u) {
                                    s_last = tick;
                                    // hand2latch IS THE OLD `len`, RENAMED TO WHAT IT MEASURES. It
                                    // no longer gates anything, but it is now the most useful
                                    // number here: it is the distance from the hand to the point
                                    // the zone says will latch. If the label ever shows up over
                                    // nothing grabbable, this is the field that says whether the
                                    // TARGET was wrong -- the complaint the beam made visible and
                                    // the label, by design, cannot.
                                    API::get()->log_info(
                                        "[Halo-CampE-UEVR] GRABGUIDE mode=%s%s size=%.1fcm | "
                                        "hand2latch=%.2fcm | along=%.3fm "
                                        "lat=%.3fm clamped=%.3fm | hand_gun=(%.1f,%.1f,%.1f)cm "
                                        "rig_off=(%.1f,%.1f,%.1f)cm",
                                        g_cfg.grab_guide_mode == 1 ? "beam basis=" : "label",
                                        g_cfg.grab_guide_mode == 1
                                            ? (g_cfg.grab_guide_beam_basis == 1 ? "1 (up flipped)"
                                             : g_cfg.grab_guide_beam_basis == 2 ? "2 (control, should look WRONG)"
                                             : g_cfg.grab_guide_beam_basis == 3 ? "3 (original)"
                                                                                : "0 (derived)")
                                            : "",
                                        g_cfg.grab_guide_mode == 1 ? g_cfg.grab_guide_thick_cm
                                                                   : g_cfg.grab_guide_label_cm,
                                        len,
                                        reach.along_m, reach.lateral_m, reach.clamped_along_m,
                                        meas.hand_gun.x, meas.hand_gun.y, meas.hand_gun.z,
                                        meas.rig_off.x, meas.rig_off.y, meas.rig_off.z);
                                }
                            }
#endif  // HALO_VR_DEV -- GRABGUIDE periodic sampler

                            // hold_cm 0 throughout = draw it AT its world position: it is a real
                            // thing at a real distance, an arm's length away, and vergence should
                            // match. PRIORITY 4 -- one rung BELOW the scope pane's 3, deliberately.
                            // Higher drops sooner, and the guide is the newest and least proven
                            // quad here, so when the runtime has fewer free layers than we have
                            // quads it must go before the pane rather than after. Sharing the
                            // pane's 3 would have left the ordering to the apparent-size tiebreak,
                            // which happens to give the right answer today and would stop doing so
                            // the moment either quad resized.
                            // THE MIN-LENGTH GATE BELONGS TO THE BEAM AND ONLY THE BEAM. A beam
                            // shorter than it is thick reads as a blob, which is why grabguidemin
                            // exists and why the catalog promises it hides "a stub of beam". The
                            // label has no length, so applying the same gate to it would hide the
                            // affordance exactly when the hand is CLOSEST to the grab point -- the
                            // moment you most want to be told you can grip.
                            if (g_cfg.grab_guide_mode == 1 && len >= g_cfg.grab_guide_min_cm) {
                                // ---- BEAM. Answers WHICH object, so it needs a real direction.
                                //
                                // THE ORIENTATION BUG THIS FIXES: the beam direction was passed as
                                // the quad's FACING. In xr_look_rotation the first argument becomes
                                // z = -fwd, i.e. the quad's NORMAL -- so the quad was turned edge-on
                                // to the viewer and its width axis, the one scaled to the beam's
                                // length, pointed off across the view. (The navpoint path is the
                                // proof of the convention: it passes the VIEW direction there.)
                                //
                                // Correct pair: face the viewer, and choose up so that
                                // x = cross(up, z) lands along the beam. With z = -view_fwd,
                                // up = cross(beam, view_fwd) gives x = beam - z(beam.z) -- the beam
                                // projected into the quad's plane, which is exactly its on-screen
                                // direction.
                                const Vec3 mid{a.x + d.x * 0.5f, a.y + d.y * 0.5f,
                                               a.z + d.z * 0.5f};
                                auto cross3 = [](const Vec3& u, const Vec3& w) {
                                    return Vec3{u.y * w.z - u.z * w.y,
                                                u.z * w.x - u.x * w.z,
                                                u.x * w.y - u.y * w.x};
                                };
                                Vec3 vfwd{}, vup{};
                                if (halo::xrlayer_view_basis(&vfwd, &vup)) {
                                    switch (g_cfg.grab_guide_beam_basis) {
                                    case 1:   // the same thing with up flipped -- finds a handedness
                                              // or sign flip introduced by ue_offset_to_xr
                                        halo::xrlayer_set_quad_orientation(
                                            halo::XRLAYER_SLOT_GUIDE, vfwd, cross3(vfwd, d));
                                        break;
                                    case 2:   // DELIBERATE CONTROL: beam as the quad's UP puts the
                                              // THIN axis along the beam. This should look plainly
                                              // wrong; if it does not, my reading of the convention
                                              // is wrong and basis 0 is right by accident.
                                        halo::xrlayer_set_quad_orientation(
                                            halo::XRLAYER_SLOT_GUIDE, vfwd, d);
                                        break;
                                    case 3:   // the original, kept so "better or worse" is a
                                              // comparison rather than a memory
                                        halo::xrlayer_set_quad_orientation(
                                            halo::XRLAYER_SLOT_GUIDE, d,
                                            quat_rotate(q_gun, Vec3{0,0,1}));
                                        break;
                                    default:  // 0 -- derived from xr_look_rotation's convention
                                        halo::xrlayer_set_quad_orientation(
                                            halo::XRLAYER_SLOT_GUIDE, vfwd, cross3(d, vfwd));
                                        break;
                                    }
                                } else {
                                    // No view basis yet (nothing composed a view this session).
                                    // Fall back to the old pair rather than dropping the beam --
                                    // a badly-oriented beam is still an affordance; none is not.
                                    halo::xrlayer_set_quad_orientation(
                                        halo::XRLAYER_SLOT_GUIDE, d,
                                        quat_rotate(q_gun, Vec3{0,0,1}));
                                }
                                // Width = the beam's length, height = its thickness.
                                halo::xrlayer_notice_quad(halo::XRLAYER_SLOT_GUIDE,
                                                          layer_anchor(halo::XRLAYER_SLOT_GUIDE, mid),
                                                          len, 0.0f,
                                                          /*priority=*/4,
                                                          g_cfg.grab_guide_thick_cm);
                                on_layer = true;
                            } else if (g_cfg.grab_guide_mode != 1) {
                                // NOTE THE CONDITION. A plain `else` here would have drawn the
                                // LABEL whenever beam mode was on and the beam was under
                                // grabguidemin -- silently swapping modes at close range, which
                                // would read as "the beam turns into text when I get near it".
                                // ---- LABEL, the default. AT THE HAND, NOT ALONG A DIRECTION.
                                // `a` is the off hand's world position -- the same endpoint the
                                // beam starts from -- so it rides the left controller. No length
                                // gate: a label has no length to be degenerate, and the old
                                // `len >= grabguidemin` test hid the affordance precisely when the
                                // hand was CLOSEST to the grab point, which is when you most want
                                // to be told you can grip.
                                //
                                // ORIENTATION CLEARED, NOT SET: text has to face the reader, and a
                                // slot with no orientation is head-facing.
                                halo::xrlayer_clear_quad_orientation(halo::XRLAYER_SLOT_GUIDE);
                                // No height argument -- omitting it means SQUARE, which is what
                                // the label's cell is.
                                halo::xrlayer_notice_quad(halo::XRLAYER_SLOT_GUIDE,
                                                          layer_anchor(halo::XRLAYER_SLOT_GUIDE, a),
                                                          g_cfg.grab_guide_label_cm, 0.0f,
                                                          /*priority=*/4);
                                on_layer = true;
                            }
                            // The "too short to draw" reporter that lived here is GONE WITH THE GATE
                            // IT WATCHED. It tested `!on_layer`, which is now unconditionally false
                            // on this path, so it could never fire again -- and a log line that
                            // cannot fire is worse than no line, because its absence still reads as
                            // "that case did not happen". Removing the gate means removing its
                            // instrument in the same edit.
                        }
                        if (on_layer) halo::interact_line_update(rig, Vec3{}, Vec3{}, false, 0.0f);
                        else          halo::interact_line_update(rig, h, t, true, 1.0f);
                    } else {
                        halo::interact_line_update(rig, Vec3{}, Vec3{}, false, 0.0f);
                        // Retire the layer quad too, or the beam stays composited at its last pose
                        // after the hand leaves the zone -- the compositor holds what it was last
                        // given. Idempotent, so calling it every non-showing tick costs nothing.
                        halo::xrlayer_retire_quad(halo::XRLAYER_SLOT_GUIDE);
                    }
                }

                // Marker mode: drop BOTH the mount offset and the pivot arm so the component origin
                // lands on the pivot itself -- the point the weapon rotates about.
                if (g_cfg.piv_viz) off = pose_off;

                // While calibrating, hold the exact world offset captured at freeze.
                if (calibrating) off = calib_off_held;

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
                            // Tell XrLayer WHICH gate stopped the publish, so its "layer DARK"
                            // line can name the cause instead of reporting that nothing arrived.
                            // g_rig_parent is the ARM RIG's attach parent, so this is the point at
                            // which the compositor reticule depends on the arm lane resolving.
                            if (!have_origin) xrlayer_note_publish_gate(3);

                            // ANCHOR THE RAY WHERE THE PLAYER'S EYE IS, once the eye has left the
                            // camera. The marker's job is to sit over the thing the shot will hit,
                            // as SEEN BY THE PLAYER, and that only survives being drawn short if the
                            // ray it sits on starts at the eye: every point on an eye-anchored ray
                            // projects onto the same background point, so the visibility cap costs
                            // nothing. Drawn short on a CAMERA-anchored ray it lies by
                            // offset * (1/drawn - 1/actual) -- at 1 m of divergence, a 6 m cap and a
                            // 30 m target, about 7 degrees. That is the "shots land at the reticule
                            // instead of at what I'm pointing at" report, and it is why the marker
                            // and the aim have to be built from ONE piece of geometry.
                            //
                            // Leashed, delta is ~0 and this is the same point as before.
                            bool conv = halo::aim_converge_engaged();
                            if (have_origin && conv) {
                                Vec3 eye_d{};
                                if (halo::aim_converge_delta(&eye_d)) {
                                    origin.x += eye_d.x; origin.y += eye_d.y; origin.z += eye_d.z;
                                } else {
                                    conv = false;
                                }
                            }

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
                                    // UE forward from the reticule's chosen angles -- game aim, or
                                    // the smoothed controller setpoint. See reticule_ray_angles.
                                    //
                                    // ENGAGED, THE RAY IS THE INTENT, NOT THE ACHIEVED AIM. Those
                                    // are different rays once convergence is on: the achieved aim
                                    // is the intent BENT so a shot from the camera lands on the
                                    // target, so following it from an eye-anchored origin would
                                    // apply the correction twice and mark the wrong point. The
                                    // intent ray from the eye is the one whose hit both the marker
                                    // and the aim are derived from -- one piece of geometry, two
                                    // consumers.
                                    //
                                    // Passing it in as the "aim" argument also keeps the divergence
                                    // guard honest: with src=1 it would otherwise measure the
                                    // correction itself as loop error and snap every frame.
                                    double ray_yaw = aim_yaw, ray_pitch = aim_pitch;
                                    if (conv) {
                                        ray_yaw   = (double)halo::g_desired_yaw.load();
                                        ray_pitch = (double)halo::g_desired_pitch.load();
                                    }
                                    float r_yaw = 0.0f, r_pitch = 0.0f;
                                    reticule_ray_angles(ray_yaw, ray_pitch, &r_yaw, &r_pitch);
                                    const float cp = std::cos(r_pitch * DEG2RAD);
                                    const Vec3 fwd{cp * std::cos(r_yaw * DEG2RAD),
                                                   cp * std::sin(r_yaw * DEG2RAD),
                                                   std::sin(r_pitch * DEG2RAD)};
                                    // TRACE FIRST, fixed distance as the fallback. On a hit the
                                    // marker lands on the surface, so it reads correctly from any
                                    // eye position -- which is the whole point, because the eye
                                    // moves with the player's head and the shot origin does not.
                                    // A failed resolve keeps the previous behaviour exactly.
                                    //
                                    // Only the DISTANCE is decided here; the target point is built
                                    // from it below. The traced hit is on this same ray by
                                    // construction, so deriving the point from d rather than using
                                    // the hit directly keeps the drawn position and the distance
                                    // that feeds the size compensation in agreement -- they must
                                    // not be able to disagree.
                                    float d = g_cfg.aim_reticule_dist;
                                    // THE SCOPE'S CONVERGENCE DISTANCE, kept SEPARATE from the
                                    // drawn one. `d` carries the 6 m visibility cap and the surface
                                    // standoff -- both display choices, made so the reticule sits at
                                    // a comfortable eye-convergence depth and does not pop between
                                    // depths when you pan onto sky. For a marker drawn along the ray
                                    // FROM THE EYE that is harmless: any point on the ray gives the
                                    // right DIRECTION, which is all a screen-space overlay needs.
                                    //
                                    // The scope capture is not on that ray. With scopecamorigin=1 it
                                    // sits on the WEAPON, so converging it on a 5-6 m point swings it
                                    // by the head-to-weapon parallax -- atan(40cm/500cm) is about
                                    // 4.6 degrees against a scope FOV of 4.38, i.e. MORE THAN THE
                                    // WHOLE FIELD OF VIEW. Measured 2026-09-07: RET PROJECT reported
                                    // z pinned at ~495 cm on every sample while the aim swept the
                                    // room, with the capture readback at err=0.00deg and the reticule
                                    // dead centre -- everything internally consistent, converging on
                                    // the wrong point.
                                    //
                                    // The aim lane already draws this exact distinction a few lines
                                    // below ("`h` and not `d`: the drawn distance carries the
                                    // visibility cap ... neither of which has anything to do with how
                                    // far the target actually is"). The capture is the second
                                    // consumer that needs the real range, and it was handed the
                                    // display one.
                                    float focus_d = g_cfg.aim_reticule_trace_max;
                                    // PER-LANE COOLDOWN, not a session kill. The first version
                                    // of this disabled the trace for the whole session after 3
                                    // faults, which pinned the reticule at aimreticuledist for
                                    // the rest of play -- trading a five-second transient for a
                                    // permanent regression. The fault window around a level
                                    // transition is ~5 s, so backing off and RETRYING is the
                                    // right shape; the backoff grows if it keeps failing, so a
                                    // genuinely broken lane still ends up parked on its own.
                                    if (tick < g_lane_retry_at[PERF_TRACE].load(std::memory_order_relaxed)) {
                                        static uint32_t s_said_at = 0;
                                        if (tick - s_said_at > 320) {
                                            s_said_at = tick;
                                            API::get()->log_info(
                                                "[Halo-CampE-UEVR] reticule: trace lane COOLING "
                                                "DOWN after %u fault(s) -- fixed distance "
                                                "(aimreticuledist=%.0f) until tick %u, then it "
                                                "retries. Everything else keeps running.",
                                                g_lane_faults[PERF_TRACE].load(std::memory_order_relaxed),
                                                g_cfg.aim_reticule_dist,
                                                g_lane_retry_at[PERF_TRACE].load(std::memory_order_relaxed));
                                        }
                                    } else if (g_cfg.aim_reticule_trace) {
                                        const float cap  = g_cfg.aim_reticule_max_dist;
                                        const float tmax = g_cfg.aim_reticule_trace_max;
                                        const Vec3 far_end{origin.x + fwd.x * tmax,
                                                           origin.y + fwd.y * tmax,
                                                           origin.z + fwd.z * tmax};
                                        // Never let the trace land on the player. The pawn is one
                                        // actor and the weapon is another, attached to the rig --
                                        // ignoring only the pawn leaves the gun to catch the ray
                                        // every time an animation swings it across the camera.
                                        API::UObject* ignore[2] = {nullptr, nullptr};
                                        int nignore = 0;
                                        if (auto* pw = API::get()->get_local_pawn(0)) {
                                            ignore[nignore++] = reinterpret_cast<API::UObject*>(pw);
                                        }
                                        if (auto* wa = fp_weapon_actor()) ignore[nignore++] = wa;

                                        Vec3 hit{};
                                        bool got = false;
                                        HALO_VR_DEV_ONLY(hit_trace_dev_report());
                                        {
                                            // Inside the gate, so `n` counts real traces and `mean`
                                            // is the cost of one -- not an average diluted by the
                                            // frames that never traced.
                                            PerfScope _pt(PERF_TRACE);
                                            got = hit_trace(origin, far_end, ignore, nignore, &hit);
                                        }
                                        float h = 0.0f;
                                        if (got) {
                                            const float hx = hit.x - origin.x;
                                            const float hy = hit.y - origin.y;
                                            const float hz = hit.z - origin.z;
                                            h = std::sqrt(hx * hx + hy * hy + hz * hz);
                                            if (h > cap) {
                                                // Beyond the cap: park at the cap EXACTLY. No
                                                // surface offset -- there is no surface here to
                                                // clip into, and subtracting one would just pull
                                                // the marker off the depth we chose.
                                                d = cap;
                                            } else {
                                                // Sit just in front of the wall rather than in it.
                                                // Floored so a muzzle-contact hit cannot put the
                                                // marker behind the eye.
                                                d = h - g_cfg.aim_reticule_surface_off;
                                                if (d < 20.0f) d = 20.0f;
                                            }
                                        } else {
                                            // Miss (sky, or past the trace length). The cap, not
                                            // aim_reticule_dist: panning off a wall onto sky should
                                            // not pop the reticule between two depths.
                                            d = cap;
                                        }
                                        // The capture converges on the REAL hit, and on a MISS it
                                        // converges far rather than at the cap. Far is the safe
                                        // failure here: at the trace limit the capture's line is
                                        // effectively parallel to the shot line, so the parallax
                                        // error goes to zero. The cap would reintroduce exactly the
                                        // 4.6-degree swing this fixes.
                                        focus_d = got ? h : tmax;

                                        // THE RANGE, handed to the aim. `h` and not `d`: the drawn
                                        // distance carries the visibility cap and the surface
                                        // standoff, neither of which has anything to do with how
                                        // far the target actually is. Aiming at the capped point
                                        // would put the shot short of the wall the marker is
                                        // painted on -- the exact confusion this feature exists to
                                        // remove. Fed on a MISS too (as "no measurement"), because
                                        // the filter's own miss policy is to hold, not to guess.
                                        halo::aim_converge_feed(h, got, g_last_dt);

#if HALO_VR_DEV
                                        // AIMCONV -- the whole correction on one line, so "my shots
                                        // land left of the reticule" can be READ. offset is how far
                                        // the eye has left the shot origin, range is what the trace
                                        // measured, and dyaw/dpitch is the bend those two produce.
                                        // If dyaw is 0 while offset is large, the feature is not
                                        // engaged and the reason is one of the gates in
                                        // aim_converge_engaged().
                                        if (g_cfg.aim_conv_log > 0
                                            && (tick % (uint32_t)g_cfg.aim_conv_log) == 0) {
                                            Vec3 dl{};
                                            const bool have_d = halo::aim_converge_delta(&dl);
                                            float cy = halo::g_desired_yaw.load();
                                            float cp2 = halo::g_desired_pitch.load();
                                            const float iy = cy, ip = cp2;
                                            const bool bent = halo::aim_converge_apply(&cy, &cp2);
                                            API::get()->log_info(
                                                "[Halo-CampE-UEVR] AIMCONV eng=%d offset=(%.1f,%.1f,%.1f)cm "
                                                "|off|=%.1f have=%d | hit=%d h=%.0f drawn=%.0f range=%.0fcm "
                                                "| intent=(y%.2f,p%.2f) cmd=(y%.2f,p%.2f) d=(y%.2f,p%.2f) bent=%d",
                                                (int)conv, dl.x, dl.y, dl.z,
                                                std::sqrt(dl.x*dl.x + dl.y*dl.y + dl.z*dl.z), (int)have_d,
                                                (int)got, h, d, halo::aim_converge_range(),
                                                iy, ip, cy, cp2, wrap180(cy - iy), cp2 - ip, (int)bent);
                                        }
#endif
                                    }
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
                                    // SCALE WITH DISTANCE. This used to be pinned to 1.0, which was
                                    // right only while the placement distance was a constant: the
                                    // scales were tuned at that one distance, so no compensation
                                    // was needed. Tracing removed the constant -- the reticule now
                                    // lands wherever the world is -- and at a fixed world size
                                    // apparent size goes as 1/distance, so it is overwhelming
                                    // against a near wall and invisible across a room.
                                    //
                                    // Proportional scaling cancels that. The floor stops the ring
                                    // vanishing when the muzzle is against a surface. See
                                    // aim_reticule_min_scale for how the two knobs set the anchor.
                                    // Anchored at BOTH ends -- see aim_reticule_min_scale. The far
                                    // anchor is the size this used to be pinned at, so the ring at
                                    // full distance is unchanged from before tracing existed and
                                    // only the near field is new.
                                    {
                                        const float ms = g_cfg.aim_reticule_min_scale;
                                        const float xs = g_cfg.aim_reticule_max_scale;
                                        const float md = g_cfg.aim_reticule_min_scale_dist;
                                        const float xd = g_cfg.aim_reticule_max_dist;
                                        float s = xs;
                                        if (xd > md) {
                                            float t = (d - md) / (xd - md);
                                            if (t < 0.0f) t = 0.0f;
                                            if (t > 1.0f) t = 1.0f;   // d is capped already; belt and braces
                                            s = ms + (xs - ms) * t;
                                        }
                                        g_ret_scale_mul = s;
                                    }

                                    if (g_cfg.aim_widget) {
                                        // rescan gate: the pick has a consumer this tick
                                        g_ret_ensure_seen_tick = g_ticks.load(std::memory_order_relaxed);
                                        reticule_widget_ensure(rig);
                                        reticule_widget_move(target, origin);
                                    } else {
                                        // Turning the feature off must give the crosshair back --
                                        // hosting it removed it from the HUD. Self-latching inside,
                                        // so the steady off state costs one bool test.
                                        reticule_widget_release();
                                    }

                                    // Plain sphere. Kept as the fallback that is known to render.
                                    // NOT gated on aim_mesh: the hide path lives inside
                                    // reticule_mesh_move, so gating the call would leave the quad
                                    // frozen and visible at its last position whenever the
                                    // feature is switched off.
                                    g_ret_origin = origin;
                                    g_have_ret_origin = true;

                                    // Publish the same target to the compositor reticule. It draws
                                    // ALONGSIDE the two above, not instead of them, and it needs the
                                    // CAMERA pose rather than the ray origin: it works by expressing
                                    // the reticule as an offset in the camera's own frame, which is
                                    // frame-invariant because the UE camera and the HMD are the same
                                    // object. Skipped without a composed view -- there is no camera
                                    // frame to be relative to yet.
                                    // The compositor reticule gets the TARGET only. Where that
                                    // point lands in stage space is decided at RENDER rate in
                                    // on_post_calculate_stereo_view_offset -- exactly like the
                                    // navpoint markers above, and for exactly the same reason.
                                    xrlayer_note_publish_gate(0);   // reached the publish
                                    xrlayer_notice_reticule(layer_anchor(halo::XRLAYER_SLOT_RETICULE, target),
                                                            g_ret_scale_mul.load());
                                    // Hand the scope THIS tick's aim ray, built RAW from the
                                    // game's live aim angles rather than from the reticule's
                                    // target: reticule_ray_angles layers display smoothing tuned
                                    // for a floating dot, and the first headset pass showed that
                                    // smoothing as the whole pane trailing the hand. Only the
                                    // direction matters; 500 cm is an arbitrary ray length.
                                    {
                                        const float cp_s = std::cos((float)aim_pitch * DEG2RAD);
                                        const Vec3 scope_target{
                                            origin.x + cp_s * std::cos((float)aim_yaw * DEG2RAD) * 500.0f,
                                            origin.y + cp_s * std::sin((float)aim_yaw * DEG2RAD) * 500.0f,
                                            origin.z + std::sin((float)aim_pitch * DEG2RAD) * 500.0f};
                                        scope_notice_ray(origin, scope_target, rig, tick);
                                        // The REAL traced hit, for the capture's convergence only.
                                        // scope_target above is an arbitrary 500 cm point and is
                                        // right for DIRECTION; converging on it aims the scope past
                                        // anything nearer, which shows up as the target sliding out
                                        // of the pane when you strafe. `target` is the same point
                                        // the reticule uses, which is why the reticule stayed on it
                                        // while the image did not.
                                        // NOT `target`: that is the DRAWN point, capped at 6 m for
                                        // display comfort. The capture needs the real range -- see
                                        // focus_d above.
                                        {
                                            const Vec3 focus_pt{origin.x + fwd.x * focus_d,
                                                                origin.y + fwd.y * focus_d,
                                                                origin.z + fwd.z * focus_d};
                                            scope_notice_focus(focus_pt, true, tick);
                                        }
                                        // The SAME ray to XrLayer, from the SAME site, so the two
                                        // cannot drift apart. It projects the reticule's (smoothed)
                                        // target through this (raw) capture axis for
                                        // xrlayerscopereticle=2 -- see xrlayer_note_scope_ray.
                                        xrlayer_note_scope_ray(origin, scope_target);
                                    }
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
                // Publish the WORLD-space offset BEFORE it is folded into the parent's frame, so the
                // render path can redo that fold against the live parent. Published even when the
                // conversion below does not apply, so the consumer never has to know the rig mode.
                g_rigw_off_x = off.x; g_rigw_off_y = off.y; g_rigw_off_z = off.z;
                // AND THE RENDER PATH MUST HONOUR THE ORIGIN-HOLD, or the hold does not exist.
                //
                // The consumer at on_pre_calculate_stereo_view_offset re-derives RelativeLocation
                // from these every FRAME. It never checked hold_rig_at_origin, so in rig_mode 2/3 --
                // and 3 is the shipped default -- the protective `rig_set_location(0,0,0)` on the
                // tick path was immediately overwritten by the render path. The hold was never
                // actually in force on the mode almost everyone runs.
                //
                // That was survivable while this block did not run during an outage: these globals
                // kept their LAST GOOD value, so the render path wrote a stale-but-sane offset and
                // the arms merely froze. Making the block run unconditionally (the fix above, for
                // the reticule dying on a position-only failure) turned that stale value into a
                // LIVE GARBAGE one -- `off` derives from `hand`, which with no controller position
                // resolves to minus the standing origin, i.e. the ~1.5 m phantom this whole gate
                // exists to keep off the screen. Frozen arms became arms flung across the room,
                // every time the runtime dropped controller translation.
                //
                // Marking the offset invalid is the honest fix rather than skipping the publish:
                // the consumer then writes ROTATION ONLY, which is exactly right -- rotation is
                // still live during a position-only outage, and that is the entire premise of
                // position_dead. Location is left to the tick path's origin write.
                // ---- THE HOLD TERM IS NOW A LIVE KEY, BECAUSE IT WAS NEVER OBSERVED ----
                //
                // `&& !hold_rig_at_origin` was added 2026-09-02 from CODE READING alone: the render
                // path re-derives location from these globals every frame and never checked the
                // origin-hold, so during a position-only outage it would write the ~1.5 m phantom
                // offset that hold_rig_at_origin exists to suppress. That reasoning still looks
                // right -- but NOBODY HAS EVER WATCHED IT HAPPEN, and the fix went in before anyone
                // could. Suppressing a symptom you have not seen is how a mechanism gets believed
                // without being confirmed, and this file already carries two entries that were
                // "obviously correct" and measured wrong.
                //
                // rigwoffhold=0 disables the term, so the render path writes the offset regardless
                // and the unguarded behaviour is visible. Live, so it can be flipped mid-outage:
                // the whole point is to A/B it inside one focus-loss window rather than across two
                // sessions where the outage may differ.
                //
                // DEFAULT IS 1 (guard on). Not a verdict on which is correct -- it is the
                // conservative shipping choice while the question is open, and it changes the
                // moment observation says otherwise.
                g_rigw_off_valid = (g_cfg.rig_mode == 2 || g_cfg.rig_mode == 3)
                                && (!hold_rig_at_origin || g_cfg.rigw_off_hold == 0);

                // WHILE THE OUTAGE IS LIVE, SAY WHAT IS BEING WRITTEN. The player can feel that the
                // arms moved; only this says HOW FAR and in which direction, which is the number
                // that decides whether the guard is worth having. Rate-limited, and it only prints
                // during an outage -- there is nothing to report on a healthy tick.
                if (hold_rig_at_origin) {
                    static uint32_t s_ph_log = 0;
                    if (tick - s_ph_log >= 60) {
                        s_ph_log = tick;
                        const float mag = std::sqrt(off.x * off.x + off.y * off.y + off.z * off.z);
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] RIGHOLD: position-dead outage, render offset "
                            "|off|=%.1fcm (%.1f,%.1f,%.1f) -- %s. rigwoffhold=%d",
                            mag, off.x, off.y, off.z,
                            (g_cfg.rigw_off_hold != 0) ? "SUPPRESSED (guard on)"
                                                       : "BEING WRITTEN (guard off)",
                            g_cfg.rigw_off_hold);
                    }
                }

                if (g_cfg.rig_mode == 2 || g_cfg.rig_mode == 3) {
                    off = quat_rotate(quat_conj(q_parent), off);
                }

                // rigclamp was already applied to the pose part above. What remains here is only a
                // sanity rail so a bad solve cannot fling the weapon out of the world -- it must
                // stay well clear of any legitimate mount offset or it becomes the same bug again.
                //
                // AND IT MUST NOT BOUND THE PLAYER'S OWN TRAVEL. `off` is pose + mount - arm, and
                // pose is as large as the room: at world scale 1.312 a 200 cm rail saturates after
                // ~1.5 m of real walking. Leashed that never happens, because the leash holds the
                // pose term near zero -- but with hmdleash=0 a fixed rail on the TOTAL silently
                // becomes a movement limit, i.e. "the arms stop following me once I go far enough",
                // which is the same complaint this whole anchor fix exists to answer.
                //
                // So bound the SOLVED part only, which is what a bad solve actually corrupts. The
                // pose term is folded into the parent's frame first so the two are subtractable.
                constexpr float SANITY_CM = 200.0f;
                Vec3 pose_par = pose_off;
                if (g_cfg.rig_mode == 2 || g_cfg.rig_mode == 3) {
                    pose_par = quat_rotate(quat_conj(q_parent), pose_off);
                }
                const float ex = pose_par.x + clampf(off.x - pose_par.x, -SANITY_CM, SANITY_CM);
                const float ey = pose_par.y + clampf(off.y - pose_par.y, -SANITY_CM, SANITY_CM);
                const float ez = pose_par.z + clampf(off.z - pose_par.z, -SANITY_CM, SANITY_CM);

                g_dbg_rig_x = ex; g_dbg_rig_y = ey; g_dbg_rig_z = ez;
                g_dbg_pos_x = rigpos.x; g_dbg_pos_y = rigpos.y; g_dbg_pos_z = rigpos.z;
                // Suppressed when the socket-cancelling drive owns the gun, for the same reason as
                // the rotation block above. THE READ-BACK VERDICT GOES WITH IT: it compares the
                // component's RelativeLocation against what we asked for, so with no write made it
                // would settle on a permanent "IS A NO-OP" that describes our own restraint rather
                // than anything the engine did -- a diagnostic that lies is worse than none.
                // hold_rig_at_origin is the OTHER half of the fix above: ex/ey/ez are derived from
                // `hand`, so with the controller position dead they are exactly the wrong offset the
                // protective branch exists to avoid writing. Everything else in this block --
                // the trace, the widget, the compositor publish -- is unaffected and has already run.
                if (!hold_rig_at_origin
                    && !halo::weapon_drive_owns() && !halo::palettearm_weapon_owns()) {
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
                }

#if HALO_VR_DEV
                // ROTATION READ-BACK -- the counterpart the location check has always had and the
                // rotation never did. rig_set_rotation() returns true unconditionally, so "it was
                // written" has never been evidence that it LANDED. On a skeletal mesh it may well
                // not: the animation system drives component rotation every frame and can overwrite
                // a relative rotation between our write and the next read, while leaving location
                // alone -- which presents exactly as "the arms translate but hardly rotate".
                //
                // Reported only when the verdict CHANGES, and it compares the residual against what
                // was asked, so a genuinely near-zero request is not mistaken for a no-op.
                {
                    auto* rr_read = rig->get_property_data<double>(L"RelativeRotation");
                    const float asked = std::fabs(rig_pitch) + std::fabs(rig_yaw) + std::fabs(c_roll);
                    // Same reasoning as the location verdict: with the mesh write suppressed there
                    // is nothing to have landed, so asking whether it landed can only mislead.
                    if (rr_read != nullptr && asked > 1.0f && !halo::weapon_drive_owns()) {
                        const float got = (float)(std::fabs(rr_read[0]) + std::fabs(rr_read[1])
                                                  + std::fabs(rr_read[2]));
                        const int verdict = (got > asked * 0.25f) ? 1 : 0;
                        static int s_rot_verdict = -1;
                        if (s_rot_verdict != verdict) {
                            s_rot_verdict = verdict;
                            API::get()->log_info(
                                "[Halo-CampE-UEVR] rig ROTATION %s (asked %.1f,%.1f,%.1f  read %.1f,%.1f,%.1f)",
                                verdict ? "APPLIES" : "IS A NO-OP", rig_pitch, rig_yaw, c_roll,
                                (float)rr_read[0], (float)rr_read[1], (float)rr_read[2]);
                        }
                    }
                }
#endif
                }   // end: neutral valid
            }
            }   // end: attachmode 0 -- the rig-driver path
        }
    }

    // ------------------------------------------------------------------ VEHICLE RETICULE
    // The world-space reticule, kept alive while seated and driven by the VEHICLE'S aim.
    //
    // On foot the reticule is placed inside the rig block above, because everything it needs comes
    // through the first-person weapon: the owning actor (the rig component's outer) and the ray
    // origin (the rig's camera parent). A vehicle has no first-person weapon, so that whole path is
    // skipped and the reticule simply vanished -- which is what this restores.
    //
    // The aim it tracks is deliberately NOT the controller. In stick mode the hand drives nothing;
    // ControlRotation IS where the game is aiming, and the vehicle's guns follow it, so the ray is
    // built from ControlRotation and the ray origin is the rendered view position published by the
    // stereo callback. Both survive a seat; neither needs the rig.
    if (g_stick_mode.load() && g_cfg.aim_reticule && g_have_view_pos.load() && !g_in_menu.load()) {
        // A component must belong to an actor, and ensure() derives that actor from its argument's
        // OUTER -- so this passes a component owned by the pawn, not the pawn itself. The pawn is
        // the same actor the on-foot reticule is outered to, so nothing is orphaned or duplicated
        // when the player dismounts.
        auto* pawn = API::get()->get_local_pawn(0);
        auto* pawn_root = follow_object(reinterpret_cast<API::UObject*>(pawn), L"RootComponent");

        if (pawn_root != nullptr) {
            const Vec3 origin{g_view_pos_x.load(), g_view_pos_y.load(), g_view_pos_z.load()};

            // UE forward, exactly as the on-foot path builds it -- including the reticule's source
            // toggle and smoothing, so the two paths cannot disagree about what the reticule means.
            float r_yaw = 0.0f, r_pitch = 0.0f;
            reticule_ray_angles(aim_yaw, aim_pitch, &r_yaw, &r_pitch);
            const float cp = std::cos(r_pitch * DEG2RAD);
            const Vec3 fwd{cp * std::cos(r_yaw * DEG2RAD),
                           cp * std::sin(r_yaw * DEG2RAD),
                           std::sin(r_pitch * DEG2RAD)};
            const float d = (g_cfg.aim_reticule_dist_veh > 0.0f)
                          ? g_cfg.aim_reticule_dist_veh : g_cfg.aim_reticule_dist;
            const Vec3 target{origin.x + fwd.x * d, origin.y + fwd.y * d, origin.z + fwd.z * d};

            g_ret_origin = origin;
            g_have_ret_origin = true;

            // Hold the APPARENT size steady, then apply the seated size ratio on top.
            //
            // Both reticules are drawn at a fixed world scale, so pushing them from the on-foot
            // distance out to the seated one shrank them by exactly that ratio -- at 30 m against a
            // 5 m tuning the ring is a sixth the size on screen, which reads as "the reticle is
            // gone" rather than "the reticle is small". `d / ref` cancels that, leaving the seated
            // reticule at on-foot apparent size; aim_reticule_scale_veh is then the deliberate
            // difference between the two, so infantry sizing is never disturbed by vehicle tuning.
            const float ref = (g_cfg.aim_reticule_dist > 1.0f) ? g_cfg.aim_reticule_dist : 500.0f;
            g_ret_scale_mul = (d / ref) * g_cfg.aim_reticule_scale_veh;

            if (g_cfg.aim_widget) {
                // rescan gate: the pick has a consumer this tick (vehicle branch)
                g_ret_ensure_seen_tick = g_ticks.load(std::memory_order_relaxed);
                reticule_widget_ensure(pawn_root);
                reticule_widget_move(target, origin);
            } else {
                reticule_widget_release();   // see the on-foot branch above
            }
            if (g_cfg.aim_mesh) reticule_mesh_ensure(pawn_root);
            reticule_mesh_move(target);

            // Compositor reticule, seated branch -- see the on-foot call for why it takes the
            // camera pose. g_ret_scale_mul already carries the seated distance compensation above,
            // so the layer inherits it and the three reticules stay the same apparent size.
            xrlayer_notice_reticule(layer_anchor(halo::XRLAYER_SLOT_RETICULE, target),
                                                            g_ret_scale_mul.load());   // seated; see the on-foot call

            // ---- FORCE VISIBLE WHILE SEATED, and prove where it landed.
            //
            // The reticule's components hang off the PAWN, and taking a seat tears down that pawn's
            // whole first-person presentation -- the weapon actor is destroyed, which is the very
            // signal stick mode detects. If the pawn (or our component with it) is hidden by that
            // teardown, the reticule is placed perfectly every tick and draws nothing, which is
            // exactly the reported symptom: huge and obvious on foot, absent in every seat.
            //
            // Re-asserted every tick rather than on the transition: whatever hides it may do so
            // repeatedly (seat changes, camera-mode flips), and these are two cheap setters.
            // Component-level visibility is not sufficient on its own if the OWNING ACTOR is
            // hidden -- bHidden suppresses every primitive under it regardless -- so the actor's
            // flag is read back below to say which of the two is happening.
            Vec3 actual{0.0f, 0.0f, 0.0f};
            int have = 0;
            const bool got = reticule_force_visible(&actual, &have);

            static uint32_t last_rep = 0;
            if (tick - last_rep >= 300) {
                last_rep = tick;
                const float err = got ? std::sqrt((actual.x - target.x) * (actual.x - target.x) +
                                                  (actual.y - target.y) * (actual.y - target.y) +
                                                  (actual.z - target.z) * (actual.z - target.z))
                                      : -1.0f;
                int pawn_hidden = -1;
                if (auto* pw = reinterpret_cast<API::UObject*>(pawn)) {
                    if (auto* h = pw->get_property_data<bool>(L"bHidden")) pawn_hidden = *h ? 1 : 0;
                }
                API::get()->log_info("[Halo-CampE-UEVR] vehicle reticule: aim=(%.1f,%.1f) dist=%.0f "
                                     "have=%d placeErr=%.0fcm pawnHidden=%d scale=%.2f",
                                     (float)aim_pitch, (float)aim_yaw, d, have, err, pawn_hidden,
                                     g_ret_scale_mul.load() * g_cfg.aim_mesh_scale);
            }
        }
    }

    // ------------------------------------------------------------------ TURNING
    // Consumes the player's raw right-stick X (sampled in the XInput hook before the aim value
    // replaced it). Adjusts the locked view yaw, so the world turns while aim stays on the gun.
    // Stands down in stick mode: the right stick IS the game's look input there, and consuming it
    // for snap turn would rotate the player twice. Also stands down the instant first-person foot
    // control is lost (g_fp_control_now) -- boarding a vehicle otherwise banks accidental snaps
    // during the enter debounce, because the player is already using the stick as a vehicle camera.
    // Standing unarmed keeps turning: it is still the player's own camera to turn.
    if (g_cfg.turn_mode != 0 && !g_stick_mode.load() && g_fp_control_now) {
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

    // ONE SECOND, not the ten the rig block uses. A respawn is over in well under ten seconds, so
    // the slow cadence cannot say whether a value changed across it or drifted before it -- which
    // is exactly the ambiguity this diagnostic exists to remove.
    if (g_cfg.wpn_diag && (g_ticks.load() % 60) == 0) {
        const float sx = g_dbg_wpn_dx.load(), sy = g_dbg_wpn_dy.load(), sz = g_dbg_wpn_dz.load();
        API::get()->log_info("[Halo-CampE-UEVR]   wpn: sep=(%.1f,%.1f,%.1f)cm |sep|=%.1f "
                             "parentchg=%d off=(%.1f,%.1f,%.1f) grip=(%.1f,%.1f,%.1f) "
                             "piv=(%.1f,%.1f,%.1f)%s",
                             sx, sy, sz, std::sqrt(sx * sx + sy * sy + sz * sz),
                             g_dbg_parent_changes.load(),
                             g_cfg.off_x, g_cfg.off_y, g_cfg.off_z,
                             g_cfg.grip_deg, g_cfg.grip_yaw, g_cfg.grip_roll,
                             g_cfg.piv_x, g_cfg.piv_y, g_cfg.piv_z,
                             g_dbg_wpn_ok.load() ? "" : " (STALE - read failed)");
        API::get()->log_info("[Halo-CampE-UEVR]   sock: local=(%.1f,%.1f,%.1f)cm "
                             "rot=(p%.1f y%.1f r%.1f)deg",
                             g_dbg_sock_x.load(), g_dbg_sock_y.load(), g_dbg_sock_z.load(),
                             g_dbg_sock_p.load(), g_dbg_sock_yw.load(), g_dbg_sock_r.load());        {
            const float ex = g_dbg_Lact_x.load() - g_cfg.off_x;
            const float ey = g_dbg_Lact_y.load() - g_cfg.off_y;
            const float ez = g_dbg_Lact_z.load() - g_cfg.off_z;
            API::get()->log_info("[Halo-CampE-UEVR]   PLACE: Lactual=(%.1f,%.1f,%.1f) "
                                 "want=(%.1f,%.1f,%.1f) ERR=(%.1f,%.1f,%.1f) |ERR|=%.1fcm%s",
                                 g_dbg_Lact_x.load(), g_dbg_Lact_y.load(), g_dbg_Lact_z.load(),
                                 g_cfg.off_x, g_cfg.off_y, g_cfg.off_z, ex, ey, ez,
                                 std::sqrt(ex * ex + ey * ey + ez * ez),
                                 g_dbg_Lact_ok.load() ? "" : " (STALE)");
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

    // ---- LATE, UNCONDITIONAL RE-ASSERT OF THE WORLD-RETICULE HIDE ---------------------------
    //
    // KEPT, BUT READ WHAT IT DOES NOT COVER BEFORE RELYING ON IT. The mechanism this was originally
    // added for was REFUTED within the hour, by the compositor lane, and the refutation is recorded
    // here rather than the claim, because a comment asserting a dead mechanism is how the next
    // reader inherits a wrong model.
    //
    // WHAT I CLAIMED (2026-09-06, WRONG): "when the reticule trace lane fault-cools, the aim pick
    // fails, so reticule_widget_move() -- the only LATE re-assert host -- is skipped."
    //
    // WHY IT IS WRONG: the cooling branch (~9412) does not fail the pick. `d` is assigned from
    // aim_reticule_dist BEFORE the branch, and cooling merely skips the TRACE that would refine it.
    // Its own log line says so: "fixed distance ... then it retries. Everything else keeps running."
    // The pick proceeds and reticule_widget_move() IS reached. Cooling costs trace accuracy, not the
    // re-assert.
    //
    // AND THIS HOST CANNOT COVER THE CASE THAT ACTUALLY OCCURS. A tick FAULT is not a skipped branch:
    // the __except filter at the bottom of this file uses EXCEPTION_CONTINUE_SEARCH, so it reports
    // and declines, and the exception unwinds straight out of update(). Everything after the fault
    // point is skipped for that tick. The faulting lanes sit EARLY -- reticule_trace ~9412,
    // socket_sample earlier still -- and this call is BELOW them, so a faulted tick never reaches it.
    // The reflection pause at the top (~5533) returns before every host, including this one.
    //
    // SO WHAT IT ACTUALLY BUYS: a late re-application on ticks that COMPLETE but never reach
    // reticule_widget_move() (the pick is taken through neither the on-foot nor the vehicle widget
    // branch). That is a narrow case and has not been observed. It is kept because it is genuinely
    // free -- a masked bit compare that writes only on disagreement -- and because LATE placement is
    // the property that matters: the widget rebuilds its material and transform during the reticule
    // work above, and a bit re-applied only early (~5757) is clobbered by that rebuild.
    //
    // THE ONLY HOST THAT SURVIVES AN ABORTED TICK is the early one at ~5757, which by construction
    // runs before the rebuild. That tension is unresolved: covering an aborted tick and running after
    // the rebuild are, as the code stands, mutually exclusive.
    //
    // STILL OPEN, and the discriminator the compositor lane proposed rather than either of us
    // guessing: does the doubling track FAULTS, or the widget REBUILD that clears the bit? They are
    // separable -- a rebuild with no fault should double if the bit-clearing model is right, and a
    // fault with no rebuild should not. The rehost counter in the mode-3 transition log is the
    // instrument. Do not write a mechanism here until that is measured.
    reticule_mode3_reassert();
}

SHORT to_raw(float v) {
    if (std::fabs(v) < 1e-3f) return 0;
    float raw = std::fabs(v) * 32767.0f;
    if (raw > 32767.0f) raw = 32767.0f;
    SHORT out = (SHORT)(v < 0.0f ? -raw : raw);

    // DITHER, to defeat the game's dispatch gate rather than its deadzone.
    //
    // The exe's input poll (exe+0x9776000) forwards an axis into UE only when it has CHANGED since
    // the previous frame, OR when |raw| exceeds XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE (0x21F1):
    //
    //     cmp  word ptr [rdi+0x104], r14w   ; same as last frame?
    //     jne  dispatch                     ; changed -> dispatch whatever the magnitude
    //     ...  cmp eax, 21F1h / jle skip    ; unchanged AND small -> suppressed
    //
    // So the deadzone never blocks a MOVING stick -- it blocks a STEADY small one. That is exactly
    // what a slow correction looks like coming out of the aim loop, and it matches the hard floor
    // measured on the plant curve at ~0.265 deflection: below it a held value is dispatched once
    // and then goes silent, so the aim stops.
    //
    // Alternating the low bit makes every frame differ from the last, so `jne dispatch` always
    // takes, and small deflections keep being delivered. Costs at most 1/32767 of deflection --
    // three orders of magnitude below the ~14 deg/s minimum correction it is there to remove.
    //
    // Default OFF: this is a behavioural hypothesis about someone else's dispatch logic, and it has
    // not been tested in a live session yet.
    if (g_cfg.stick_dither && out != 0) {
        static bool s_flip = false;
        s_flip = !s_flip;
        if (s_flip) {
            // Nudge AWAY from zero so the sign can never invert, and never past full scale.
            if (out > 0 && out < 32767) ++out;
            else if (out < 0 && out > -32767) --out;
            else if (out > 0) --out;
            else ++out;
        }
    }
    return out;
}

} // namespace


// ============================================================================================
// ENABLE OUR OPENXR API LAYER FOR THIS PROCESS ONLY -- no registry, no PDB, no user step.
// ============================================================================================
//
// THE PROBLEM THIS SOLVES. The compositor needs xrEndFrame. Two routes existed and both are bad:
//
//   PDB rung   -- resolve the symbol out of UEVRBackend.pdb and hook it. MEASURED 2026-09-05 on
//                 BOTH the pinned nightly-01138 and a locally-built backend: the symbol resolves,
//                 the hook installs, and it is NEVER CALLED. The watchdog fires every session.
//                 It also depends on a 149 MB PDB happening to be a release asset.
//   Registry   -- register the layer as an IMPLICIT layer under HKCU. That works, but it loads our
//                 layer into EVERY OpenXR application on the machine (relying on a process gate to
//                 stay inert), it persists until unregistered, and it is an install step the user
//                 reasonably calls suspicious.
//
// THE THIRD ROUTE, which is what this is. The OpenXR loader discovers EXPLICIT layers from
// XR_API_LAYER_PATH and enables them by name from XR_ENABLE_API_LAYERS, both read when the
// application calls xrCreateInstance. We are loaded long before that -- measured 700 ms of margin
// (plugin at 11:03:33.509, "Creating OpenXR instance" at 11:03:34.219) -- so setting them here is
// enough, and it is the same per-process discovery trick this project already uses for the Meta XR
// Simulator's XR_RUNTIME_JSON.
//
// WHY IT IS THE LEAST INVASIVE OF THE THREE: SetEnvironmentVariable writes to OUR OWN process
// environment block. Nothing on disk, nothing in the registry, nothing another process can see,
// gone when the game exits, nothing to uninstall. Scope is one process instead of every OpenXR
// application on the machine.
//
// APPENDS, NEVER CLOBBERS. Another tool may already be using these -- a developer, a different
// layer, a capture tool. Overwriting them would silently disable someone else's layer, which is
// exactly the class of "touches things outside its own folder" behaviour this route exists to
// avoid.
//
// HONOURS HALOVR_LAYER_DISABLE. That escape hatch is a PUBLIC PROMISE in the README, and
// disable_environment in the manifest only works for IMPLICIT layers -- so on this route we have to
// honour it ourselves or the promise quietly stops being true.
// RUNS AT DLL LOAD, NOT AT on_initialize -- and that is the whole point.
//
// MEASURED 2026-09-06 from this game's own startup log:
//     +0.002s  [PluginLoader] Loaded <our dll>   <- a static constructor runs about here
//     +0.659s  [VR] Creating OpenXR instance     <- the loader reads the layer env vars HERE
//     +3.117s  on_initialize()                   <- where this used to be called: 2.458s LATE
//
// The OpenXR loader reads XR_API_LAYER_PATH / XR_ENABLE_API_LAYERS exactly once, inside
// xrCreateInstance. Setting them afterwards is not merely unreliable, it can NEVER work -- and it
// did not: the log showed the vars being set, the layer never loading, and the attachment silently
// falling back to the DEV-ONLY PDB rung. That fallback is why the compositor overlay worked on a
// developer machine and would not have worked for a single player.
//
// NO LOGGING IN HERE. UEVR's API does not exist yet at static-init time; API::get() would be a null
// dereference during DLL_PROCESS_ATTACH -- a crash before the game has drawn a frame. The outcome
// is recorded into a buffer and printed from on_initialize instead.
//
// LOADER-LOCK DISCIPLINE: this runs under the loader lock, so it makes kernel32 calls and nothing
// else -- no LoadLibrary, no COM, no threads, no engine calls. GetFileAttributes is the heaviest
// thing here, and it is what keeps us from naming a layer that is not on disk (which makes
// xrCreateInstance FAIL on some runtimes -- turning "no overlay" into "no VR at all").
char     g_apilayer_note[600] = {0};
uint64_t g_apilayer_tick_ms   = 0;

void enable_api_layer_for_this_process() {
    g_apilayer_tick_ms = GetTickCount64();
    if (GetEnvironmentVariableA("HALOVR_LAYER_DISABLE", nullptr, 0) != 0) {
        sprintf_s(g_apilayer_note, sizeof(g_apilayer_note),
                  "HALOVR_LAYER_DISABLE is set -- not enabling the API layer for this process.");
        return;
    }

    char appdata[MAX_PATH] = {0};
    const DWORD an = GetEnvironmentVariableA("APPDATA", appdata, MAX_PATH);
    if (an == 0 || an >= MAX_PATH) {
        sprintf_s(g_apilayer_note, sizeof(g_apilayer_note),
                  "APPDATA unreadable at DLL load -- cannot locate the profile's apilayer folder.");
        return;
    }

    char dir[MAX_PATH];
    sprintf_s(dir, MAX_PATH, "%s\\UnrealVRMod\\HaloCampaignEvolved\\apilayer", appdata);

    // BOTH FILES OR NEITHER. Naming a layer that is not there makes xrCreateInstance fail on some
    // runtimes -- turning "no compositor overlay" into "no VR at all", which is not a trade we get
    // to make on a player's behalf.
    char json[MAX_PATH], dll[MAX_PATH];
    sprintf_s(json, MAX_PATH, "%s\\%s", dir, "XrApiLayer_HALOVR_reticule.json");
    sprintf_s(dll,  MAX_PATH, "%s\\%s", dir, "XrApiLayer_HALOVR_reticule.dll");
    if (GetFileAttributesA(json) == INVALID_FILE_ATTRIBUTES ||
        GetFileAttributesA(dll)  == INVALID_FILE_ATTRIBUTES) {
        sprintf_s(g_apilayer_note, sizeof(g_apilayer_note),
                  "no apilayer\\ in the profile (looked for %s) -- falling back to the PDB rung, "
                  "which only exists on a dev machine. Watch for the WATCHDOG line.", json);
        return;
    }

    auto append_env = [](const char* name, const char* value) {
        char cur[4096] = {0};
        const DWORD n = GetEnvironmentVariableA(name, cur, (DWORD)sizeof(cur));
        if (n == 0) { SetEnvironmentVariableA(name, value); return; }
        if (n >= sizeof(cur)) return;                       // absurdly long; leave it alone
        if (strstr(cur, value) != nullptr) return;          // already present
        char joined[8192];
        sprintf_s(joined, sizeof(joined), "%s;%s", cur, value);
        SetEnvironmentVariableA(name, joined);
    };

    append_env("XR_API_LAYER_PATH", dir);
    append_env("XR_ENABLE_API_LAYERS", "XR_APILAYER_HALOVR_reticule");

    sprintf_s(g_apilayer_note, sizeof(g_apilayer_note),
              "API layer enabled for THIS PROCESS ONLY -- XR_API_LAYER_PATH += %s, "
              "XR_ENABLE_API_LAYERS += XR_APILAYER_HALOVR_reticule. No registry, no PDB, nothing "
              "outside the profile folder. ENABLED IS NOT ATTACHED: watch for tier=apilayer.", dir);
}

// THE STATIC CONSTRUCTOR IS THE MECHANISM. A DLL's namespace-scope constructors run from the CRT's
// DLL_PROCESS_ATTACH path -- at LoadLibrary time, ~600 ms before this game reaches xrCreateInstance.
// We cannot write our own DllMain because uevr/Plugin.hpp already defines one (see the note at the
// top of this file), so this is the earliest hook available to us, and it is early enough with room
// to spare. If UEVR ever starts loading plugins after VR init, the tick delta logged from
// on_initialize is what will say so.
namespace {
struct ApiLayerEarlyInit {
    ApiLayerEarlyInit() { enable_api_layer_for_this_process(); }
};
const ApiLayerEarlyInit g_api_layer_early_init;
}   // namespace

class HaloAimDriverPlugin : public uevr::Plugin {
public:
    void on_initialize() override {
        // ALREADY DONE, AT DLL LOAD -- see enable_api_layer_for_this_process. All that is left here
        // is to say what happened, because logging was impossible that early.
        //
        // THE TICK DELTA IS THE INSTRUMENT that proves the move worked: it is how long before THIS
        // moment the env vars were actually set. on_initialize lands ~3.1 s into startup and
        // xrCreateInstance at ~0.66 s, so a delta above ~2.5 s means we beat the deadline. A small
        // number means the static constructor did not run when I think it did and the approach is
        // wrong. Read it; do not assume it -- assuming this exact thing is what cost the last round.
        {
            const uint64_t now = GetTickCount64();
            const uint64_t ago = (g_apilayer_tick_ms != 0 && now >= g_apilayer_tick_ms)
                               ? (now - g_apilayer_tick_ms) : 0;
            API::get()->log_info(
                "[Halo-CampE-UEVR] XRLAYER: %s (set at DLL load, %llu ms before on_initialize; "
                "needs to be >~2500 ms to have beaten xrCreateInstance)",
                g_apilayer_note[0] ? g_apilayer_note : "api-layer init did not run at all",
                (unsigned long long)ago);
        }

        // Name-the-fault table: ONE signature scan of the exe, so a TICK FAULT line can say
        // which engine function it landed in without a recorded RVA. See FNAME_TOSTRING_SIG.
        fault_names_init();

        // Config lives beside the UEVR profile so it is where a user would look for it.
        char appdata[MAX_PATH] = {0};
        DWORD n = GetEnvironmentVariableA("APPDATA", appdata, MAX_PATH);
        if (n > 0 && n < MAX_PATH) {
            sprintf_s(g_cfg_path, MAX_PATH,
                      "%s\\UnrealVRMod\\HaloCampaignEvolved\\halo_vr.cfg", appdata);
            sprintf_s(g_user_cfg_path, MAX_PATH,
                      "%s\\UnrealVRMod\\HaloCampaignEvolved\\halo_vr_user.cfg", appdata);
            sprintf_s(g_user_ref_path, MAX_PATH,
                      "%s\\UnrealVRMod\\HaloCampaignEvolved\\halo_vr_user_reference.txt", appdata);
            sprintf_s(g_dev_cfg_path, MAX_PATH,
                      "%s\\UnrealVRMod\\HaloCampaignEvolved\\halo_vr_dev.cfg", appdata);
            sprintf_s(g_calib_path, MAX_PATH,
                      "%s\\UnrealVRMod\\HaloCampaignEvolved\\halo_vr_calib.cfg", appdata);
            sprintf_s(g_data_dir, MAX_PATH,
                      "%s\\UnrealVRMod\\HaloCampaignEvolved\\data", appdata);
            // Per-weapon captures get their own machine-owned file, for the reason Config.cpp
            // gives for keeping halo_vr_calib.cfg separate: it is rewritten in full on every
            // capture, and doing that to halo_vr.cfg would destroy its comments.
            sprintf_s(g_wpn_calib_path, MAX_PATH,
                      "%s\\UnrealVRMod\\HaloCampaignEvolved\\halo_vr_weapons.cfg", appdata);
        } else {
            strcpy_s(g_cfg_path, MAX_PATH, "halo_vr.cfg");
            strcpy_s(g_user_cfg_path, MAX_PATH, "halo_vr_user.cfg");
            strcpy_s(g_user_ref_path, MAX_PATH, "halo_vr_user_reference.txt");
            strcpy_s(g_dev_cfg_path, MAX_PATH, "halo_vr_dev.cfg");
            strcpy_s(g_calib_path, MAX_PATH, "halo_vr_calib.cfg");
            strcpy_s(g_data_dir, MAX_PATH, "data");
            strcpy_s(g_wpn_calib_path, MAX_PATH, "halo_vr_weapons.cfg");
        }
        strcpy_s(g_calib_path_right, MAX_PATH, g_calib_path);

        // Settings-menu bridge files live under the profile's data\ folder -- the one location
        // UEVR's Lua sandbox lets scripts read and write (see menu_bridge_tick in Config.cpp).
        CreateDirectoryA(g_data_dir, nullptr);
        sprintf_s(g_menu_cmd_path, MAX_PATH, "%s\\halo_vr_menu_set.txt", g_data_dir);
        sprintf_s(g_user_mirror_path, MAX_PATH, "%s\\halo_vr_user_mirror.cfg", g_data_dir);
        sprintf_s(g_ref_mirror_path, MAX_PATH, "%s\\halo_vr_user_reference.txt", g_data_dir);
        sprintf_s(g_dev_mirror_path, MAX_PATH, "%s\\halo_vr_dev_mirror.cfg", g_data_dir);
        sprintf_s(g_calib_mirror_path, MAX_PATH, "%s\\halo_vr_calib_mirror.cfg", g_data_dir);
        sprintf_s(g_status_path, MAX_PATH, "%s\\halo_vr_status.txt", g_data_dir);

        ensure_user_cfg_template();   // all-comment template; never touches an existing file
        load_config();                // writes a commented default halo_vr.cfg if none exists

        // Every override layer now ships or is template-created, so file EXISTENCE says nothing --
        // log the layers actually DOING something. Dev loudest: the shipped halo_vr_dev.cfg is
        // all-commented, so an uncommented key there is the first thing a support log should show.
        {
            // NAME the keys. This warning used to say only that the file "has uncommented keys",
            // which on a machine where several sessions share halo_vr_dev.cfg is close to useless:
            // a stale scopesrc=8 once sat here for hours beating the user's own cfg on every
            // reload while this line fired every launch and named nothing.
            char keys[320];
            const int n = config_file_list_uncommented_keys(g_dev_cfg_path, keys, sizeof(keys));
            if (n > 0) {
                API::get()->log_info("[Halo-CampE-UEVR] DEV OVERRIDES ACTIVE: %s has %d uncommented "
                                     "key(s) -- %s -- (beats halo_vr_user.cfg on every ~2 s reload; "
                                     "the shipped file has none, and updates overwrite it)",
                                     g_dev_cfg_path, n, keys);
            }
            const int un = config_file_list_uncommented_keys(g_user_cfg_path, keys, sizeof(keys));
            if (un > 0) {
                API::get()->log_info("[Halo-CampE-UEVR] user overrides: %s applied, %d key(s) -- %s "
                                     "-- (survives updates; a dev key of the same name WINS over "
                                     "these)", g_user_cfg_path, un, keys);
            }
        }

        // Calibration is per-hand, and it must be resolved AFTER load_config -- aimhand lives in
        // halo_vr.cfg, so the correct file is not known until that has been read.
        //
        // Each hand keeps its own file so switching handedness cannot destroy the other hand's
        // tuning. If the left file does not exist yet, it is seeded by MIRRORING the right rather
        // than starting from zero: grip yaw/roll and the lateral offset flip sign between hands,
        // and a mirrored start is much closer to correct than a cold one.
        switch (select_calib_for_hand()) {
        case CALIB_HAND_LEFT_LOADED:
            API::get()->log_info("[Halo-CampE-UEVR] LEFT-HANDED aim: calibration from %s",
                                 g_calib_path);
            break;
        case CALIB_HAND_LEFT_SEEDED:
            API::get()->log_info(
                "[Halo-CampE-UEVR] LEFT-HANDED aim: no left calibration yet -- seeded %s by "
                "MIRRORING the right hand (grip yaw/roll, off X, pivot X, aim yaw negated). "
                "Re-run the End / Page Down calibrations for a proper left-hand fit.",
                g_calib_path);
            break;
        default:
            break;   // right-handed: nothing to report, this is the normal path
        }

        // Version first, on its own line: this is what a bug report needs to be actionable, and it
        // must survive even if the settings line below changes shape.
        API::get()->log_info("[Halo-CampE-UEVR] Halo: Campaign Evolved VR  v%s  (halo_vr.dll)",
                             HALO_VR_VERSION);

        // WHICH BACKEND ARE WE LOADED INTO. UEVR publishes its own identity to plugins, and until
        // now we logged none of it -- so establishing that a reporter was on a third-party fork
        // rather than the pinned nightly meant reading UEVR's own header out of their log and
        // recognising the tag by eye. It cost a round trip on a report that mattered.
        //
        // This is not a support boundary, it is a fact worth stating: the plugin hooks addresses
        // and callbacks that belong to whatever backend actually loaded it, so "which one" is the
        // first question any aim or stereo bug has to answer.
        {
            // get_build_date()/get_build_time() are deliberately NOT logged: measured on the pinned
            // nightly, this backend returns them UNEXPANDED ("~6,2.~4,2.~0,4"), so they would be
            // noise in every report forever. Tag, branch and commit are what identify a build --
            // and a fork announces itself in the first two (the report that prompted this read
            // "tag=UEVR_AFW_v1.0-beta.4 branch=AFW").
            const auto* fns = API::get()->param()->functions;
            API::get()->log_info(
                "[Halo-CampE-UEVR] UEVR backend: tag=%s branch=%s commit=%s (+%u past tag, %u total)",
                fns->get_tag(), fns->get_branch(), fns->get_commit_hash(),
                fns->get_commits_past_tag(), fns->get_total_commits());
        }
        API::get()->log_info("[Halo-CampE-UEVR] closed-loop controller aim. "
                             "enabled=%d floor=%.2f full=%.1fdeg max=%.2f xdist=%.0fcm fake_pad=%d",
                             (int)g_cfg.enabled, g_cfg.floor, g_cfg.full_deg, g_cfg.max_out,
                             g_cfg.xdist_m * 100.0f, (int)g_cfg.fake_pad);
        API::get()->log_info("[Halo-CampE-UEVR] KILL SWITCH: Ctrl+Home, or add enabled=0 to %s (re-read every ~2s)", g_user_cfg_path);
    }

    // ---- TEARDOWN -----------------------------------------------------------------------------
    //
    // WHY THIS EXISTS (2026-08-16): the plugin had NO shutdown path at all. On exit it left a
    // live INLINE HOOK on a sim-thread function called ~2600 times a second, an audio-listener
    // override on the game's own component, several dynamically created WidgetComponents, and
    // the game's controller settings still overridden -- `game_settings_restore()` was written,
    // documented as "called at shutdown", and had NO CALLERS.
    //
    // Every exit path (Alt+F4 AND the in-game Exit to Desktop) was ending in a STACK OVERFLOW
    // (c00000fd) inside UEVRBackend's `Framework shutting down...`, which dies dirty, writes
    // crash.dmp, and leaves Steam wedged at "Stopping" -- destroying the relaunch loop. Our
    // still-installed hook being torn out from under a running sim thread while UEVR unhooks
    // itself is a plausible contributor to that recursion, and leaving it installed is wrong
    // regardless of blame.
    //
    // So: release everything OURS at the first sign of shutdown, once, in a fixed order -- the
    // OpenXR layer first (it is the one thing wired into the host's OWN shutdown path), then the
    // sim-thread hook, then the writes we hold on game objects, then our own components.
    // Idempotent and safe to call from any thread.
    static void plugin_teardown(const char* why) {
        static std::atomic<bool> done{false};
        if (done.exchange(true)) return;
        g_shutting_down.store(true, std::memory_order_release);
        API::get()->log_info("[Halo-CampE-UEVR] TEARDOWN (%s): releasing hook, overrides and components", why);

        // EVERY STAGE ANNOUNCES ITSELF BEFORE IT RUNS. A teardown that hangs returns no value,
        // throws nothing and leaves no stack: the last line printed IS the diagnosis. A teardown
        // instrumented only at its END can say it finished, but when it does NOT finish it cannot
        // say where it stopped -- and that ambiguity cost an entire evening here. First the clean
        // "TEARDOWN complete" was read as proof the plugin was innocent while the hang sat
        // downstream of it; then, once xrlayer_shutdown() was added at the front, there was no way
        // to distinguish "the fix did not help" from "the fix now hangs EARLIER" -- which is a real
        // risk, because that call removes a detour and destroys XR/D3D12 objects from the
        // window-message thread while the submit thread may still be inside hooked_end_frame.
        // If the log ends on a "stage:" line, that stage is the one that blocked. UEVR's logger
        // flushes per call, so these survive a freeze.
        const auto stage = [](const char* name) {
            API::get()->log_info("[Halo-CampE-UEVR] TEARDOWN stage: %s", name);
        };

        // 0. THE OPENXR LAYER FIRST OF ALL. This one is not merely "ours" -- it is wired INTO the
        //    shutdown path we are racing: a detour on xrEndFrame inside UEVRBackend, plus an XR
        //    swapchain and D3D12 resources parented to the session UEVR is about to destroy.
        //    Leaving it up means the runtime tears down a session whose child handles are still
        //    alive and whose call path still runs through our trampoline. Every archived exit
        //    hang was captured with the layer Armed (state=2), and the log always stops inside
        //    UEVR's own teardown, AFTER our "TEARDOWN complete" -- which is exactly what a
        //    leaked layer looks like from the outside. xrlayer_shutdown() removes the hook
        //    before destroying what the hook points at, and is idempotent.
        stage("xrlayer_shutdown (remove_hook -> destroy_swapchain -> release_d3d)");
        xrlayer_shutdown();
        stage("xrlayer_shutdown RETURNED");

        // 1. THE INLINE HOOK NEXT. blam_drive_tick() removes it when blam_angles is 0, and it
        //    is the only other thing we own that runs off the game thread.
        stage("blam inline hook");
        g_cfg.blam_angles = 0;
        blam_drive_tick();
        stage("aim_watch_shutdown");
        aim_watch_shutdown();

        // 2. Writes we hold on the GAME'S objects: the audio listener override, and the
        //    controller settings the mod changes transiently (restoring these was already
        //    written and simply never called).
        stage("audio listener overrides");
        audio_fix_tick(false, 0, 0, 0, 0, 0);
        audio_comp_tick(false, 0, 0, 0, 0, 0, 0);
        stage("game_settings_restore");
        game_settings_restore();

        // 3. Our own components: hand the game's crosshair back, detach the hands from UEVR's
        //    motion-controller components, and park the markers. hands_release() was in the same
        //    position xrlayer_shutdown() was -- correct, but reachable only from hands_update()'s
        //    disabled branch, so it had never run on an exit.
        stage("reticule_widget_release");
        reticule_widget_release();
        stage("hands_release");
        hands_release();
        stage("navw_hide_all");
        navw_hide_all();
        stage("two_hand_reset");
        halo::two_hand_reset("teardown");

        API::get()->log_info("[Halo-CampE-UEVR] TEARDOWN complete");
    }

    // WM_CLOSE covers Alt+F4 and the window's X. WM_QUIT/WM_DESTROY cover the engine-initiated
    // path (the in-game Exit to Desktop), which never sends WM_CLOSE.
    bool on_message(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam) override {
        if (msg == WM_CLOSE || msg == WM_QUIT || msg == WM_DESTROY) {
            plugin_teardown(msg == WM_CLOSE ? "WM_CLOSE" : (msg == WM_QUIT ? "WM_QUIT" : "WM_DESTROY"));
        }
        return true;   // never swallow: the game must still process its own exit
    }

    // ---- NAME THE FAULT DIRECTLY. Stop inferring it. -----------------------------------------
    //
    // The RAII breadcrumb below was built on the claim that /EHsc keeps destructors from running on
    // an SEH fault. THAT CLAIM IS WRONG: MSVC emits unwind funclets for destructors regardless, so
    // ~TickDoneGuard can run during UEVR's unwind and mark the tick "finished" -- an instrument that
    // the fault it is measuring can switch off. TICK FAULTED read 0 through 8,477 exceptions and I
    // read that as "our callback is never called", which does not follow.
    //
    // An __except FILTER runs BEFORE any unwinding, so nothing can suppress it, and it is handed the
    // exception code and the faulting address. EXCEPTION_CONTINUE_SEARCH means we only OBSERVE:
    // UEVR still handles the exception exactly as before and behaviour is unchanged.
    //
    // The address is the answer this whole chain has been missing -- it says which MODULE faulted,
    // so "is it us, UEVR, or the game" stops being an argument about log ordering.
    static void report_tick_fault(EXCEPTION_POINTERS* xp) {
        // ATTRIBUTE THE FAULT TO ITS LANE, BEFORE THE REPORT CAP.
        //
        // This filter runs BEFORE unwinding, so g_tick_lane still names the lane that was
        // executing. Counting here -- above the `said` cap, which only limits LOGGING -- is what
        // lets a lane that keeps faulting be switched off instead of killing the tick forever.
        //
        // MEASURED 2026-09-06: 9,625 consecutive tick faults, every one 0xC0000005 reading 0x10
        // inside UEVRBackend.dll, every one in 'reticule_trace'. It began at 17:26:04 and never
        // recovered: on_pre_engine_tick died on entry to that lane on EVERY tick afterwards, so
        // the arm rig, the reticule and everything downstream simply stopped, while aim kept
        // working because it rides the XInput hook's separate dispatch. The player saw "arm
        // tracking and reticle are dead but my shots still follow my controller" -- and nothing
        // recovered it short of restarting the process.
        // Set BEFORE anything can unwind -- see g_tick_aborted. This is the abort signal;
        // g_tick_finished is not, and never could be.
        g_tick_aborted.store(true, std::memory_order_relaxed);
        const int flt_lane = g_tick_lane.load(std::memory_order_relaxed);
        if (flt_lane >= 0 && flt_lane < (int)PERF_COUNT) {
            const uint32_t n  = g_lane_faults[flt_lane].fetch_add(1, std::memory_order_relaxed) + 1;
            const uint32_t tk = g_tick_now.load(std::memory_order_relaxed);
            // Back off further the more a lane repeats, capped: a transient transition costs
            // ~3 s, a lane that is genuinely broken ends up effectively parked without ever
            // needing a separate "disable forever" rule.
            const uint32_t back = (n < 8) ? (96u * n) : 1024u;
            g_lane_retry_at[flt_lane].store(tk + back, std::memory_order_relaxed);
            // ...and pause ALL reflection briefly. A fault means the reflection subsystem is
            // walking something dead; the next lane in will hit it too, whichever lane that is.
            // 32 TICKS (~1 s), AND THE CEILING IS NOT ARBITRARY: while paused, update() returns
            // early, so the scope gets no aim ray -- and Scope.cpp closes the scope when the ray
            // is older than kRayGraceTicks = 48. The first version of this used 96 and produced
            // exactly that: "CLOSED BY RAY STALENESS -- ray last seen 96 ticks ago, grace is 48".
            // The pause fixed the fault storm (9,625 -> 7) and then shut the scope by starving it.
            // Any future increase here must stay below that grace, or raise the grace with it.
            g_reflect_ok_at.store(tk + 32u, std::memory_order_relaxed);   // ~1 s at ~32 Hz
            if (flt_lane == (int)PERF_TRACE) g_trace_faults.fetch_add(1, std::memory_order_relaxed);
        }
        // A CAP THAT GOES SILENT FOR THE REST OF THE PROCESS HIDES THE BURST YOU CARE ABOUT.
        //
        // This stopped dead at 8, so the 2026-09-08 release playthrough logged 8 reports against
        // 42 aborted ticks -- and the 34 it swallowed were every one after the first four minutes.
        // The silence then reads as "the fault stopped", which is the exact opposite of what had
        // happened, and it cost a round of this investigation reasoning about why the reports
        // ended. Reporting the ORDINAL as well means a single line now says how bad it is.
        //
        // First 8 in full, then one in every 64: enough to show a burst without a fault storm
        // turning the log into its own performance problem.
        static uint32_t said = 0;
        static uint32_t seen = 0;
        if (xp == nullptr || xp->ExceptionRecord == nullptr) return;
        const uint32_t nth = ++seen;
        if (said >= 8 && (nth % 64u) != 0u) return;
        ++said;
        const auto* er = xp->ExceptionRecord;
        void* addr = er->ExceptionAddress;

        char modname[MAX_PATH] = "(unknown)";
        void* modbase = nullptr;
        HMODULE hm = nullptr;
        if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS
                             | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                               (LPCSTR)addr, &hm) && hm != nullptr) {
            GetModuleFileNameA(hm, modname, MAX_PATH);
            modbase = (void*)hm;
        }
        const char* leaf = fault_module_leaf(modname);

        char extra[128] = "";
        if (er->ExceptionCode == EXCEPTION_ACCESS_VIOLATION && er->NumberParameters >= 2) {
            _snprintf_s(extra, sizeof(extra), _TRUNCATE, "  %s address %p",
                        (er->ExceptionInformation[0] == 0) ? "reading" :
                        (er->ExceptionInformation[0] == 1) ? "WRITING" : "executing",
                        (void*)er->ExceptionInformation[1]);
        }
        // WHICH FUNCTION, from the unwind metadata the OS itself dispatches with -- never a recorded
        // address. Named only when its start coincides with the signature-resolved FName::ToString
        // (see FNAME_TOSTRING_SIG); "?" otherwise, with the function's own start RVA so an offline
        // resolve (Scripts\Resolve-ExeRva.py against that build's exe) can finish the job.
        char fn[96] = "";
        {
            DWORD64 img = 0;
            const RUNTIME_FUNCTION* rf = RtlLookupFunctionEntry((DWORD64)(uintptr_t)addr, &img, nullptr);
            if (rf != nullptr) {
                const uintptr_t begin = (uintptr_t)img + rf->BeginAddress;
                const uintptr_t ts    = g_fname_tostring.load(std::memory_order_relaxed);
                _snprintf_s(fn, sizeof(fn), _TRUNCATE, " fn=%s+0x%llX (fn starts +0x%llX)",
                            (ts != 0 && begin == ts) ? "FName::ToString" : "?",
                            (unsigned long long)((uintptr_t)addr - begin),
                            (unsigned long long)(begin - (uintptr_t)img));
            }
        }
        // WHICH BUILD. The RVA is a measurement of THIS binary and means nothing against another,
        // and every field report so far has cost a round trip to establish which one it was. Same
        // fields, same order as the BLAMDRIVE line for the sim module, so the two compare.
        char build[128] = "";
        {
            addrcascade::ModuleIdentity id{};
            if (modbase != nullptr && addrcascade::module_identity(modbase, &id)) {
                _snprintf_s(build, sizeof(build), _TRUNCATE,
                            " build{SizeOfImage=0x%X stamp=0x%08X pdb=%s age=%u}",
                            id.size_of_image, id.timestamp, id.pdb_guid, id.pdb_age);
            }
        }
        // WHO CALLED IT. See fault_return_chain.
        char chain[512] = "";
        fault_return_chain(xp->ContextRecord, chain, sizeof(chain));

        const int   lane = g_tick_lane.load(std::memory_order_relaxed);
        const char* mark = g_navw_mark.load(std::memory_order_relaxed);
        API::get()->log_info(
            "[Halo-CampE-UEVR] TICK FAULT #%u: code 0x%08X at %p in %s (base %p, +0x%llX)%s%s%s | "
            "lane '%s' step '%s' | called from %s. Observed only -- the exception is passed on "
            "untouched, so this tick's remaining lanes (rig, arms, hands, two-hand, gestures) did "
            "NOT run.",
            (unsigned)nth, (unsigned)er->ExceptionCode, addr, leaf, modbase,
            (unsigned long long)((uintptr_t)addr - (uintptr_t)modbase), extra, fn, build,
            (lane >= 0 && lane < PERF_COUNT) ? kPerfName[lane] : "(none)",
            (mark != nullptr) ? mark : "-",
            chain[0] ? chain : "(no chain)");
    }

    void on_pre_engine_tick(API::UGameEngine* engine, float delta) override {
        // No C++ objects requiring unwinding may live in a function containing __try, so the real
        // body stays in its own function and this stays a thin observer.
        __try {
            on_pre_engine_tick_body(engine, delta);
        } __except (report_tick_fault(GetExceptionInformation()), EXCEPTION_CONTINUE_SEARCH) {
        }
    }

    void on_pre_engine_tick_body(API::UGameEngine* engine, float delta) {
        if (g_shutting_down.load(std::memory_order_acquire)) return;

        // ---- DID THE PREVIOUS TICK COME BACK? See g_tick_lane. ----
        {
            const bool prev_ok      = g_tick_finished.exchange(false, std::memory_order_relaxed);
            // The authoritative half. prev_ok alone is unreliable in exactly the case this exists
            // to catch -- see g_tick_aborted -- so an abort observed by the filter counts even
            // when the guard has already declared the tick finished.
            const bool prev_aborted = g_tick_aborted.exchange(false, std::memory_order_relaxed);
            if ((!prev_ok || prev_aborted) && g_tick_ever.load(std::memory_order_relaxed)) {
                const int lane = g_tick_lane.load(std::memory_order_relaxed);
                const char* name = (lane >= 0 && lane < PERF_COUNT) ? kPerfName[lane] : "(before any lane)";
                static uint32_t said = 0;
                static uint32_t since = 0;
                if (said < 10 || ++since >= 600) {
                    ++said; since = 0;
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] TICK FAULTED: the previous on_pre_engine_tick did not "
                        "return. Last lane entered: '%s'. This is the site UEVR's "
                        "\"one of the plugins has an error\" refuses to name.", name);
                }
            }
            g_tick_ever.store(true, std::memory_order_relaxed);
        }
        struct TickDoneGuard {
            ~TickDoneGuard() { g_tick_finished.store(true, std::memory_order_relaxed); }
        } _tick_done_guard;

        // ---- LEVEL-SCOPED RAW POINTERS: VERIFY ONCE, HERE, BEFORE ANYTHING DEREFERENCES THEM ----
        //
        // g_rig_parent and g_rig_component are raw UObject pointers read from ~10 call sites each,
        // every tick, and handed to UEVR (call_ret_vec3, get_outer). A level teardown frees the
        // component and leaves the pointer NON-NULL, so every one of those sites is a use-after-
        // free until something happens to reassign it.
        //
        // MEASURED 2026-09-04, after the same bug was fixed in XrSource: the remaining faults were
        //   TICK FAULT 0xC0000005 in UEVRBackend.dll  last lane 'resolve_shell '   <- g_rig_parent
        //   TICK FAULT 0xC0000005 in UEVRBackend.dll  last lane 'navw_newslot  '   <- g_rig_component
        // Both read live-looking heap addresses, which is the signature of a RECYCLED object rather
        // than a null one -- see uobject_live() for why no cheaper check can catch that.
        //
        // VALIDATING HERE RATHER THAN AT EACH SITE is the whole point: every downstream use already
        // guards on != nullptr, so nulling a dead pointer once makes all of them correct without
        // editing any of them. The index cache keeps the steady-state cost at one indexed compare.
        {
            static int32_t s_parent_idx = -1;
            if (g_rig_parent != nullptr && !uobject_live(g_rig_parent, &s_parent_idx)) {
                g_rig_parent = nullptr;
                s_parent_idx = -1;
                static uint32_t said = 0;
                if (said < 8) {
                    ++said;
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] LIVENESS: g_rig_parent was freed (level teardown) -- "
                        "dropped. It will be re-resolved; nothing downstream writes through it now.");
                }
            }

            static int32_t s_rigcomp_idx = -1;
            auto* rc = reinterpret_cast<API::UObject*>(g_rig_component.load(std::memory_order_relaxed));
            if (rc != nullptr && !uobject_live(rc, &s_rigcomp_idx)) {
                g_rig_component.store(nullptr, std::memory_order_relaxed);
                s_rigcomp_idx = -1;
                static uint32_t said2 = 0;
                if (said2 < 8) {
                    ++said2;
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] LIVENESS: g_rig_component was freed (level teardown) -- "
                        "dropped. navw_ensure_slot and the rig writers re-resolve on the next tick.");
                }
            }
        }

        // A null engine on the tick is the engine-loop teardown that never sends a window
        // message -- the in-game "Exit to Desktop" path. Treat it as the same signal.
        if (engine == nullptr) { plugin_teardown("engine null"); return; }
        if (delta > 0.0f && delta < 1.0f) {
            g_last_dt = delta;
            // WORST frame in the window. A hitch every few seconds is invisible in an
            // instantaneous dt and invisible in a mean; it only shows up as a peak.
            if (delta > g_dt_worst.load()) g_dt_worst = delta;
        }
        // The braces are load-bearing: PERF_TICK must CLOSE before perf_hitch_report() reads the
        // total it wrote. Left at function scope, the destructor would run after the report and the
        // hitch line would always see the PREVIOUS tick's number.
        {
            PerfScope _perf_tick(PERF_TICK);

            // THE TWO-HANDED HOLD, before update() because the rig block inside update() consumes
            // the swing it publishes. Outside the arm-driver branches below on purpose: this is an
            // aim feature, so it runs in every armdriver mode including 0.
            //
            // gameplay_active is false for anything that is not ordinary on-foot play. Each of
            // these would otherwise let a hold corrupt something permanent or fight a mechanism
            // that has already taken the camera:
            //   * menus -- g_menu_active defaults TRUE, so an unestablished state reads as "not
            //     gameplay" rather than as gameplay
            //   * stick mode / vehicles -- the player's own stick owns the camera there
            //   * either calibration gesture -- Page Down persists an aim offset to disk and End
            //     solves the weapon pose from the rendered result. A live blend at the release
            //     edge bakes itself into both, permanently.
            // These flags are refreshed inside update(), so they are one tick old. That is a state
            // which changes at most every few seconds; do NOT move this call after update() to
            // "fix" it, because that would put the rig a frame behind the aim -- the exact
            // divergence this whole design exists to prevent.
            {
                const bool two_hand_ok = !halo::g_menu_active.load(std::memory_order_relaxed)
                                      && !halo::g_stick_mode_active.load(std::memory_order_relaxed)
                                      && !halo::g_aim_calibrating.load(std::memory_order_relaxed)
                                      && !g_calib_held.load(std::memory_order_relaxed);
                PerfScope _perf(PERF_2HAND);
                halo::two_hand_update(delta, two_hand_ok, g_ticks.load(std::memory_order_relaxed));
            }

            update();
            // AFTER update(): the config reload and the stick-mode / calibration flags the detector
            // gates on are both refreshed in there, so running first would decide on stale state.
            { PerfScope _perf(PERF_GEST);  gesture_update(delta); }

            // AFTER gesture_update(): holster reads reload_state() to know whether to render the
            // magazine, and stands its melee detector down near a holster zone. Reset in menus so a
            // grip held across a pause cannot leave a grenade stuck to the hand.
            //
            // OUTSIDE the arm-driver branches below, for the same reason two_hand_update is: this
            // is a weapon-switch feature, not an arms feature, and it must run in every armdriver
            // mode. Placing it inside the UeRig branch would silently disable holsters for anyone
            // on the palette driver.
            { PerfScope _perf(PERF_HOLSTER);
              if (g_in_menu.load()) holster_reset(); else holster_update(delta); }

            // WHICH ARM DRIVER OWNS THE ARMS THIS FRAME. Must run before either driver: it is what
            // releases the outgoing one on a mode change, and a driver no longer being called cannot
            // release itself. See ArmDriver.hpp -- two drivers on one set of arms is a hard rule.
            arm_driver_arbitrate();

            if (arm_driver_owns(ArmDriverMode::UeRig)) {
                // Same ordering reason as gesture_update: the rig handle arms_update() reads is
                // resolved inside update().
                { PerfScope _perf(PERF_ARMS);  arms_update(); }
                // AFTER arms_update(): the hands only make sense once the game's own meshes are
                // hidden, and hands_update() reads the reload state gesture_update() just advanced.
                { PerfScope _perf(PERF_HANDS); hands_update(); }
            } else if (arm_driver_owns(ArmDriverMode::Palette)) {
                // The tick half only: install/remove the hook, capture poses, advance the two-hand
                // latch. The palette itself is rewritten later, on the game's own thread, inside the
                // detour -- which is exactly why this site must stay cheap.
                { PerfScope _perf(PERF_PALARM); palettearm_update(delta); }
            }
        }
        perf_hitch_report();
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
    // Guarded like every other callback: after teardown the rig/marker components this writes
    // through are being destroyed, and the view lock has nothing left to enforce.
    void on_pre_calculate_stereo_view_offset(UEVR_StereoRenderingDeviceHandle, int index, float,
                                             UEVR_Vector3f* position, UEVR_Rotatorf* rotation,
                                             bool is_double) override {
        if (g_shutting_down.load(std::memory_order_acquire)) return;
        // ---- THE VIEW POSITION, published for the vehicle reticule.
        //
        // On foot the reticule takes its ray origin from the rig's attach parent (the camera
        // component), reached THROUGH the first-person weapon. In a vehicle there is no
        // first-person weapon, so that route is gone -- and this callback is the one place the
        // camera's world position is handed to us regardless. Same discipline as the rotation
        // below: read, publish, no reflection on the render thread.
        if (position != nullptr) {
            float px, py, pz;
            if (is_double) {
                auto* p = reinterpret_cast<UEVR_Vector3d*>(position);
                px = (float)p->x; py = (float)p->y; pz = (float)p->z;
            } else {
                px = position->x; py = position->y; pz = position->z;
            }
            g_view_pos_x = px; g_view_pos_y = py; g_view_pos_z = pz;
            // Publish the rendered camera and base yaw for the holster body frame. Markers.cpp
            // converts room-space zones to world space with exactly these three positions plus
            // g_view_base_yaw, so if they go stale every shoulder/hip zone sits at the wrong
            // bearing -- which reads in headset as "the holsters are behind me".
            halo::g_cam_x.store(px, std::memory_order_relaxed);
            halo::g_cam_y.store(py, std::memory_order_relaxed);
            halo::g_cam_z.store(pz, std::memory_order_relaxed);
            g_view_base_yaw.store(g_dbg_view_out.load(std::memory_order_relaxed), std::memory_order_relaxed);
            g_have_view_pos = true;

            // THE SHOT-ORIGIN HALF of the eye offset. This callback runs BEFORE UEVR applies the
            // HMD transform (FFakeStereoRenderingHook.cpp: the pre loop, then the transform, then
            // the post loop), so `position` here is the GAME's camera -- where shots come from --
            // and the post callback's is the rendered EYE. Their difference is the whole of what
            // aim convergence needs. Measured at the source rather than reconstructed, because
            // reconstructing it means agreeing about world scale, rotation offset and which space
            // the standing origin lives in, and any one of those being wrong is silent.
            halo::aim_converge_note_pre(index, px, py, pz);
        }

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
                        // q_w is already the world target -- no parent composition needed, which
                        // is the point: the relative write is discarded on this mesh.
                        float wp = 0.0f, wy = 0.0f, wr2 = 0.0f;
                        quat_to_rotator(q_w.x, q_w.y, q_w.z, q_w.w, &wp, &wy, &wr2);

                        // LOCATION MUST BE RE-DERIVED HERE TOO, for the same reason the rotation is.
                        // RelativeLocation is interpreted in the parent's frame, so a location
                        // computed against the parent as it was at the tick is wrong by the parent's
                        // rotation since -- and that error is a LEVER: it scales with how far the rig
                        // sits from the parent origin, which is the controller's own offset (~57 cm
                        // here). Correcting rotation per frame while leaving location on the tick is
                        // what made rotating the controller translate the weapon.
                        //
                        // Resolved ONCE, above the fan-out, in whichever form this mode writes.
                        bool have_loc = false;
                        Vec3 loc{};
                        if (g_rigw_off_valid.load()) {
                            const Vec3 ow{g_rigw_off_x.load(), g_rigw_off_y.load(), g_rigw_off_z.load()};
                            if (g_cfg.rig_mode == 3) {
                                // WORLD location, for the same reason as the rotation above. With the
                                // mount and pivot both zero, a static parent, and an unchanged
                                // controller position, a relative write still moved the rig ~30 cm
                                // under pure rotation -- measured. Nothing we compute can do that, so
                                // the frame the engine composes RelativeLocation in is not the node we
                                // sampled: g_rig_parent is evidently not the immediate parent. An
                                // absolute write does not care how deep the hierarchy is.
                                Vec3 pl{};
                                if (call_ret_vec3(par, L"K2_GetComponentLocation", &pl)) {
                                    loc = Vec3{pl.x + ow.x, pl.y + ow.y, pl.z + ow.z};
                                    have_loc = std::isfinite(loc.x) && std::isfinite(loc.y)
                                            && std::isfinite(loc.z);
                                }
                            } else {
                                loc = quat_rotate(quat_conj(q_par), ow);
                                have_loc = std::isfinite(loc.x) && std::isfinite(loc.y)
                                        && std::isfinite(loc.z);
                            }
                        }

                        // ONE WRITER FOR BOTH MESHES, exactly as the tick does through for_each_rig.
                        // The shell used to get rotation here but NOT location, so once the location
                        // half was added for the arms the shield was left a tick behind in position
                        // only -- it tracked in rotation and lagged in translation, which reads as
                        // "the overshield moves late". Fanning both halves out from one place makes
                        // that class of drift structurally impossible rather than remembered.
                        // THE RENDER-RATE RIG ROTATION, for anything placed from a game-tick
                        // sample of a transform that rides this rig. The compositor scope pane is
                        // the first such consumer: it is parented to the weapon socket, so a ~32 Hz
                        // sample of it trails the drawn frame by however far the weapon turned --
                        // the same interval this function already corrects for the arm meshes.
                        // Publishing costs three atomic stores and is inert when nothing tracks.
                        {
                            // From the QUATERNION the render path already holds, not from the Euler
                            // triple beside it: an Euler round-trip degenerates near vertical and
                            // made the pane jitter when the controller rolled.
                            const Quat qr = q_w;
                            halo::xrlayer_note_rig(loc,
                                                   quat_rotate(qr, Vec3{1.0f, 0.0f, 0.0f}),
                                                   quat_rotate(qr, Vec3{0.0f, 1.0f, 0.0f}),
                                                   quat_rotate(qr, Vec3{0.0f, 0.0f, 1.0f}));
                        }

                        auto apply_render = [&](API::UObject* c) {
                            if (g_cfg.rig_mode == 3) {
                                rig_set_world_rotation(c, (double)wp, (double)wy, (double)wr2);
                                if (have_loc) {
                                    rig_set_world_location(c, (double)loc.x, (double)loc.y, (double)loc.z);
                                }
                            } else {
                                rig_set_rotation(c, (double)rp, (double)ry, (double)rr);
                                if (have_loc) {
                                    rig_set_location(c, (double)loc.x, (double)loc.y, (double)loc.z);
                                }
                            }
                        };

                        // THE FORK. Either the arm mesh carries the gun (legacy) or the weapon root
                        // does, never both -- two writers on one visual result is the fight the
                        // attach_mode block documents.
                        //
                        // The weapon drive needs C in WORLD terms, and only rig_mode 3 supplies that
                        // pairing here: q_w is already the world rotation target, and `loc` is a
                        // world location only on that branch. Rather than silently doing nothing on
                        // the other modes -- indistinguishable from the mode being broken -- say so
                        // once and leave the gun on the path that works.
                        // Either owner means the MESH stands down; only wpndrive also needs a
                        // write here. The palette owner does its work in the Blam palette on the
                        // game thread and wants nothing from this callback -- so it must not fall
                        // into wpndrive's rigmode warning, which would be a false report about a
                        // mode that is not running.
                        if (halo::palettearm_weapon_owns() && !halo::weapon_drive_owns()) {
                            // Mesh intentionally not driven. Nothing to do on the render path.
                        } else if (halo::weapon_drive_owns()) {
                            if (g_cfg.rig_mode == 3 && have_loc) {
                                halo::weapon_drive_apply(q_w, loc);
                            } else {
                                static bool s_wd_warned = false;
                                if (!s_wd_warned) {
                                    s_wd_warned = true;
                                    API::get()->log_info(
                                        "[Halo-CampE-UEVR] weapon drive: needs rigmode=3 with a valid "
                                        "location (have rigmode=%d loc=%d). Gun stays on the legacy "
                                        "mesh drive.", g_cfg.rig_mode, (int)have_loc);
                                }
                            }
                        } else {
                            apply_render(rig);

                            // Same q_rel/loc are correct for the shell: same parent (asserted at
                            // acquisition) and the same pose -- it is posed identically to the arms by
                            // its own instance of the same anim blueprint, it only lacks our write.
                            // Bare pointer by design -- see g_shell_component; validating it here would
                            // cost an FName->string per frame on the render thread.
                            //
                            // The shell follows the ARMS, so when the arms are body-anchored it must
                            // stay with them rather than chase the controller -- which is why it sits
                            // inside this branch rather than beside it.
                            if (g_cfg.shell_drive) {
                                if (auto* sh = reinterpret_cast<API::UObject*>(g_shell_component.load())) {
                                    apply_render(sh);
                                }
                            }
                        }

                        // NOT a residual -- this is the error being CORRECTED. It measures how far
                        // the parent moved since the tick that produced this target, i.e. exactly
                        // how wrong the arms WOULD be if the tick's write were the last word. A
                        // large number here means the fix is doing a lot of work, not that error
                        // remains: the relative rotation is recomputed against the live parent, so
                        // the world orientation is correct by construction at the moment of writing.
                        //
                        // DEV-ONLY: it answers a question rather than playing the game, which is
                        // the test in DevTools.hpp. It was running on the RENDER path in shipping
                        // builds -- a formatted log line every ~3.3 s, forever, for a number no
                        // player can act on.
#if HALO_VR_DEV
                        static float worst = 0.0f;
                        static uint32_t n = 0;
                        const float d = std::fabs(wrap180(prot.y - g_rigw_parent_yaw.load()));
                        if (d > worst) worst = d;
                        if (((++n) % 600) == 0) {
                            API::get()->log_info("[Halo-CampE-UEVR] RIGRENDER: corrected up to %.2f deg of parent motion "
                                                 "since tick, over last window", worst);
                            worst = 0.0f;
                        }
#endif
                    }
                }
            }
        }

        if (rotation == nullptr || !g_cfg.view_lock) return;

        // STICK MODE: the game camera must reach the eyes unmodified -- the chase camera turning
        // the rendered view IS the gamepad experience the mode exists to restore. The lock is held
        // UNPRIMED throughout; the first frame after stick mode ends re-primes by folding the
        // ride's net rotation into the turn offset (see the prime below). The debug view values
        // keep tracking the real camera, so a ride's net rotation stays measurable from the log.
        if (g_stick_mode.load()) {
            const float y = is_double
                ? (float)reinterpret_cast<UEVR_Rotatord*>(rotation)->yaw
                : rotation->yaw;
            g_dbg_view_in = y; g_dbg_view_out = y;
            g_lock_primed = false;
            return;
        }

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
                // Consumed HERE, not above, so the flag can only be eaten by a prime that actually
                // happens -- reading it every render frame would race the game thread setting it.
                if (!g_lock_ever.load()) {
                    // FIRST prime of the session: adopt the current camera yaw as the base.
                    g_locked_view_yaw = (float)r->yaw - g_turn_offset.load();
                    g_lock_ever = true;
                    invalidate_ref_for_frame();   // see below -- the aim reference BAKES this value
                } else {
                    // RE-PRIME after stick mode. A ride rotates the game camera by some net
                    // amount while the room stays put -- EXACTLY what a snap turn is. So the base
                    // is KEPT and the difference is folded into the turn offset, which every
                    // room->game anchor already consumes (aim via aim_turn, the rig via rig_turn,
                    // movement, and this lock). base + turn still lands on the current camera
                    // (seamless view), and the weapon/aim anchors rotate WITH it.
                    //
                    // The previous fix kept the turn and moved the base instead: view seamless,
                    // but every anchor was silently left rotated by the ride's net rotation --
                    // field-observed as the weapon sitting wrong after any ride that turned
                    // (Warthog driver, Pelican), with or without snap turns.
                    g_turn_offset = wrap180((float)r->yaw - g_locked_view_yaw.load());
                }
                g_lock_primed = true;
            } else {
                r->yaw = (double)locked;
            }
            g_dbg_view_out = (float)r->yaw;
            ::halo::g_view_pitch = (float)r->pitch;
            publish_view_lock_delta();
        } else {
            g_dbg_view_in = rotation->yaw;
            if (!g_lock_primed.load()) {
                // Same first-prime vs re-prime split as the double branch above.
                if (!g_lock_ever.load()) {
                    g_locked_view_yaw = rotation->yaw - g_turn_offset.load();
                    g_lock_ever = true;
                    invalidate_ref_for_frame();
                } else {
                    g_turn_offset = wrap180(rotation->yaw - g_locked_view_yaw.load());
                }
                g_lock_primed = true;
            } else {
                rotation->yaw = locked;
            }
            g_dbg_view_out = rotation->yaw;
            ::halo::g_view_pitch = rotation->pitch;
            publish_view_lock_delta();
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

        // ---- NAVFIX, RENDER-RATE APPLICATION (navrender). This callback owns both terms of the
        // delta in the SAME frame: viewIn (the aim camera, published above as g_dbg_view_in) and
        // `locked` (what the eyes get). The tick calibrated keff from the game's own projection;
        // this write is therefore lag-free by construction -- the fix for the jitter the tick-rate
        // version produced. Yaw-only (pitch is the DecoupledPitchUIAdjust double-count, navpitch).
        // g_nav_count/g_navpoints are tick-written; a stale read here costs one frame of a widget
        // list that TrackedObject::get() re-validates anyway. Same UFunction-from-this-callback
        // precedent as the rig re-apply above.
        if (g_nav_apply.load() && !g_stick_mode.load()) {
            const float ndyaw = wrap180(g_dbg_view_in.load() - locked);
            if (std::fabs(ndyaw) <= 89.0f) {
                const float npx = clampf(g_nav_keff.load() * std::tan(ndyaw * DEG2RAD),
                                         -g_cfg.nav_max, g_cfg.nav_max);
                if (std::isfinite(npx)) {
                    for (int i = 0; i < g_nav_count && i < 4; ++i) {
                        auto* o = g_navpoints[i].get();
                        if (o == nullptr) continue;
                        alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
                        if (g_cfg.hud_float) {
                            reinterpret_cast<float*>(q)[0] = npx;
                        } else {
                            reinterpret_cast<double*>(q)[0] = (double)npx;
                        }
                        o->call_function(L"SetRenderTranslation", q);
                    }
                }
            }
        }
    }

    // Minimal by design: reads two pre-computed scalars and writes the pad struct. No reflection,
    // no allocation, no logging -- this fires on the order of 200,000 times per second.
    // Read-only. Runs after UEVR has composed HMD tracking, so `rotation` here is the finished
    // view in GAME space -- what the player is actually looking along. Published for the movement
    // frame; nothing is written back.
    void on_post_calculate_stereo_view_offset(UEVR_StereoRenderingDeviceHandle, int index, float,
                                              UEVR_Vector3f* position, UEVR_Rotatorf* rotation,
                                              bool is_double) override {
        if (g_shutting_down.load(std::memory_order_acquire)) return;
        // THE EYE HALF of the eye-to-shot-origin offset. Same callback pair, same eye index, one
        // subtraction apart -- see the pre callback. `position` has been through UEVR's HMD
        // transform by now, so this IS the rendered eye.
        if (position != nullptr) {
            float ex = 0.0f, ey = 0.0f, ez = 0.0f;
            if (is_double) {
                auto* p = reinterpret_cast<UEVR_Vector3d*>(position);
                ex = (float)p->x; ey = (float)p->y; ez = (float)p->z;
            } else {
                ex = position->x; ey = position->y; ez = position->z;
            }
            halo::aim_converge_note_post(index, ex, ey, ez);
            // PUBLISHED FOR WORLD-SPACE MARKERS. This is the RENDERED EYE, which the pre-hook
            // position is not: any marker placed along a ray cast from the game camera appears
            // displaced once the player's head is elsewhere, and the error GROWS the closer the
            // marker is drawn (atan(offset/range)) -- the exact "offsets at different ranges and
            // angles" reported from the field, and the same parallax HitTrace.hpp documents for
            // the reticule. Markers must reason from here.
            g_eye_pos_x = ex; g_eye_pos_y = ey; g_eye_pos_z = ez;
            g_have_eye_pos = true;

            // ---- NAVPOINT MARKERS, RE-PLACED AT RENDER RATE.
            //
            // The tick decided WHICH objective and HOW FAR along the ray to draw (that needs the
            // navpoint map and a trace, neither of which belongs on this thread). What it cannot
            // do is decide WHERE: at a 4 m draw distance the eye-to-objective direction changes
            // materially between 32 Hz ticks, and a transform left stale for two frames reads as
            // jitter while walking. So the direction is re-derived here, from the eye this frame,
            // against the cached objective position -- the rig re-apply pattern exactly.
            //
            // Cheap by construction: two UFunction calls per VISIBLE marker (usually one), no
            // reflection lookups, no allocation, nothing that walks an array.
            if (g_cfg.nav_world && g_cfg.nav_world_src == 2 && g_cfg.nav_world_render) {
                // STABLE slots are non-contiguous: walk the placed MASK, not [0,n). A slot's bit
                // is set only after g_navw_placed[slot] was written this tick (release store on the
                // mask), so a set bit always has a fresh entry.
                const uint32_t mask = g_navw_placed_mask.load();
                const bool layer_nav = g_cfg.xr_layer && g_cfg.xr_layer_nav;
                for (int i = 0; i < 8; ++i) {
                    if ((mask & (1u << i)) == 0) continue;
                    // A MARKER THE COMPOSITOR IS DRAWING DOES NOT NEED THIS.
                    //
                    // Its in-scene component is at alpha 0, and the compositor quad gets its own
                    // render-rate placement from xrlayer_note_eye() a few lines below -- against
                    // the same eye, in the same callback. Re-placing the invisible component too
                    // would be two reflected UFunction calls per marker PER EYE PER FRAME to move
                    // something nobody can see, and reflected calls are the expensive operation in
                    // a UEVR plugin. Checked per marker rather than per lane, because a slot whose
                    // capture is not up is still drawing in the scene and still needs it.
                    if (layer_nav && halo::xrlayer_slot_ready(halo::XRLAYER_SLOT_NAV_BASE + i)) {
                        continue;
                    }
                    auto* comp = g_navw_pool[i].get_checked(L"WidgetComponent");
                    if (comp == nullptr) continue;
                    const NavwPlaced pl = g_navw_placed[i];
                    const float dx = pl.ox - ex, dy = pl.oy - ey, dz = pl.oz - ez;
                    const float d = std::sqrt(dx * dx + dy * dy + dz * dz);
                    if (!(d > 1.0f) || !std::isfinite(d)) continue;
                    const float k = pl.dist / d;
                    {
                        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                        auto* v = reinterpret_cast<double*>(p);
                        v[0] = (double)(ex + dx * k);
                        v[1] = (double)(ey + dy * k);
                        v[2] = (double)(ez + dz * k);
                        comp->call_function(L"K2_SetWorldLocation", p);
                    }
                    {
                        const float fh = std::sqrt(dx * dx + dy * dy) / d;
                        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                        auto* v = reinterpret_cast<double*>(p);
                        v[0] = (double)(std::atan2(-dz / d, fh) * RAD2DEG);
                        v[1] = (double)(std::atan2(-dy, -dx) * RAD2DEG);
                        v[2] = 0.0;
                        comp->call_function(L"K2_SetWorldRotation", p);
                    }
                }
            }
        }

        if (rotation == nullptr) return;
        // ROLL is read into a local rather than a global on purpose: the compositor reticule below is
        // its only consumer, and the existing yaw/pitch globals are a documented pair that other
        // subsystems read (movement, HUD follow, the scope). Adding a third global would invite
        // those to start using a value none of them were written against.
        float view_roll = 0.0f;
        if (is_double) {
            auto* r = reinterpret_cast<UEVR_Rotatord*>(rotation);
            g_render_view_yaw   = (float)r->yaw;
            g_render_view_pitch = (float)r->pitch;
            view_roll           = (float)r->roll;
        } else {
            g_render_view_yaw   = rotation->yaw;
            g_render_view_pitch = rotation->pitch;
            view_roll           = rotation->roll;
        }
        g_have_render_yaw = true;

        // ---- COMPOSITOR RETICULE, PLACED AT RENDER RATE.
        //
        // Same split as the navpoint markers above: the tick chose the world point, this decides
        // where it maps to in stage space. It must happen HERE because the eye position and the
        // view rotation are only simultaneous with the head pose at this moment -- computing it on
        // the 32 Hz tick paired a head pose sampled then against eye/view data from the last
        // rendered frame, and that mismatch is an angular error proportional to head speed. It is
        // what was left of the drift after the first fix.
        //
        // Last in the callback, after the rotation is parsed, so it sees this frame's finished view
        // rather than the previous one's.
        if (g_have_eye_pos.load()) {
            halo::xrlayer_note_eye(index,
                                   Vec3{g_eye_pos_x.load(), g_eye_pos_y.load(), g_eye_pos_z.load()},
                                   Vec3{g_view_pos_x.load(), g_view_pos_y.load(), g_view_pos_z.load()},
                                   g_render_view_yaw.load(), g_render_view_pitch.load(), view_roll);
        }
    }

    void on_xinput_get_state(uint32_t* retval, uint32_t user_index, XINPUT_STATE* state) override {
        if (g_shutting_down.load(std::memory_order_acquire)) return;
        g_xhits.fetch_add(1);
        if (state == nullptr || user_index != 0) return;

        // Presence: report a connected pad ourselves so the mod does not depend on an Oculus /
        // Virtual Desktop / ViGEm bus being installed. This is what Halo-MCC-VR means by "the mod
        // owns virtual slot 0 when no physical pad is present".
        if (g_cfg.fake_pad && retval != nullptr && *retval != ERROR_SUCCESS) {
            // DO NOT WIPE A STATE SOMETHING ELSE ALREADY FILLED IN.
            //
            // The unconditional ZeroMemory that used to sit here silently broke every automated
            // menu press whenever no physical pad was enumerated: UEVR-MCP writes the buttons into
            // `state`, this hook then zeroed them, and the game saw a connected pad with nothing
            // pressed. Symptom was 40 consecutive gamepad-A presses failing to leave the main menu
            // -- while the same presses worked earlier the same day, because a headset session had
            // a Virtual Desktop / Oculus pad enumerated, which makes *retval == ERROR_SUCCESS and
            // skips this branch entirely. That is why it looked like flaky injection rather than a
            // bug in our own code.
            //
            // The zero is still wanted for its original purpose -- a disconnected pad can leave
            // garbage in the struct -- so keep it, but only when the struct is actually neutral.
            const auto& g = state->Gamepad;
            const bool populated = g.wButtons != 0 || g.sThumbLX != 0 || g.sThumbLY != 0 ||
                                   g.sThumbRX != 0 || g.sThumbRY != 0 ||
                                   g.bLeftTrigger != 0 || g.bRightTrigger != 0;
            if (!populated) ZeroMemory(state, sizeof(XINPUT_STATE));
            *retval = ERROR_SUCCESS;
        }

        // Sample the player's own stick BEFORE we overwrite it -- this is the turn input.
        g_raw_stick_x = (float)state->Gamepad.sThumbRX / 32767.0f;

        // Publish the RAW buttons for the reload state machine, before any remapping below. It has
        // to react to what the player physically pressed, not to what the remapper turned it into.
        g_pad_buttons.store(state->Gamepad.wButtons, std::memory_order_relaxed);
        // Tap/hold tracking runs at POLL rate, not tick rate: a 250 ms threshold judged at ~32 Hz
        // would quantise to 31 ms steps and feel arbitrary in the hand.
        reload_note_buttons(state->Gamepad.wButtons);

        // ---- GRIP SWALLOW, BEFORE THE REBIND. The position is the whole point.
        //
        // Left controller X reports 0x2000, and this profile ALREADY rebinds 0x2000 -> 0x0100
        // (mapfrom/mapto), which is the mask the game reads as throw. So left X has always reached
        // the grenade through the author's own remap.
        //
        // Swallowing 0x0100 after that rebind ate BOTH sources indistinguishably -- the physical
        // grip and the remapped left X -- which is exactly why grenades went dead. Stripping it
        // here, before the rebind runs, removes only the PHYSICAL grip press; left X is still
        // 0x2000 at this point and converts to 0x0100 afterwards, untouched.
        // THE GRIP IS UNBOUND FROM THE GAME OUTRIGHT (gripswallow), independent of every other
        // feature. It is the VR interaction button -- two-handed aiming now, magazine grabs and
        // weapon holding later -- and on this game its native action is Throw Grenade, so a grab
        // lobbed a frag every time.
        //
        // A STANDALONE UNBIND, deliberately. This was briefly routed through the reload lane's
        // grip_exclusive, which meant turning off an unrelated feature handed the grip back to the
        // game and quietly restored the grenade. A binding must not depend on another lane's flag.
        //
        // reload_swallow_grip() still contributes its own narrower window, so the reload gesture
        // keeps working when gripswallow is off.
        if (g_cfg.reload_grip_mask != 0
            && (g_cfg.grip_swallow || reload_swallow_grip())) {
            state->Gamepad.wButtons &= (WORD)~g_cfg.reload_grip_mask;
        }

        // THE PHYSICAL BUTTONS, before anything of OURS is injected.
        //
        // Needed because a remap source and an injected destination can be the SAME mask. Grenade
        // ships on the right thumbstick (0x0080), which is also melee_mask -- the mask the swing
        // gesture injects further down. Testing the live state would let our own synthetic melee
        // satisfy the grenade remap, so every swing would throw a grenade instead of hitting
        // something. That is the same shape as the crouch/equipment ordering note below, and the
        // reload strip/re-inject pair above; this snapshot is the general answer to it.
        const WORD raw_btn = state->Gamepad.wButtons;

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

        // ---- BIND CAPTURE. "Press the button you want" for the in-game Controls panel.
        //
        // Sits HERE, on the RAW mask before any remapping, because the whole point is to record
        // what the controller actually sends. The physical-button -> XInput-mask mapping is not
        // stable across runtimes -- on Quest the right controller's B arrives as 0x4000, which
        // XInput (and our dropdown) calls "X" -- so a name picked from a list is a guess and this
        // is a measurement. It is the same lesson mapbtnlog was added for, made self-service.
        //
        // Arming is the bridge's job and so is the resulting file write (Config.cpp); this does
        // only the two things that must happen at poll rate: notice the edge, and eat it.
        {
            static WORD s_bind_prev = 0;
            static WORD s_bind_eat  = 0;
            const WORD  raw = state->Gamepad.wButtons;

            if (g_bind_capture.load(std::memory_order_acquire) != 0) {
                const WORD fresh = (WORD)(raw & ~s_bind_prev);   // bits that went DOWN this poll
                if (fresh != 0) {
                    // Lowest set bit only. A chord would record as a combined mask that no single
                    // press can ever reproduce, so the bind would read back fine and never fire.
                    const WORD one = (WORD)(fresh & (WORD)(~(unsigned)fresh + 1u));
                    g_bind_captured.store((int)one, std::memory_order_release);
                    g_bind_capture.store(0, std::memory_order_release);
                    s_bind_eat = one;
                }
            }
            s_bind_prev = raw;   // BEFORE the eat below: the edge detector tracks the pad, not us

            // Swallow the captured press for as long as it is HELD. Recording a bind must not
            // also fire whatever that button currently does -- and clearing on the capture poll
            // alone would let the tail of the same hold through on the very next one.
            if (s_bind_eat != 0) {
                if ((raw & s_bind_eat) == 0) s_bind_eat = 0;
                else state->Gamepad.wButtons &= (WORD)~s_bind_eat;
            }
        }

        // ---- RIGHT GRIP: TAKEN AWAY FROM THE GAME, because it is about to mean something else.
        //
        // The right grip natively reports 0x0200, which this game reads as EQUIPMENT. That is the
        // binding being retired -- the grip becomes over-the-shoulder weapon switching, and
        // reaching back to swap weapons must not also burn your overshield. Equipment keeps left X
        // and d-pad LEFT, so nothing is lost by taking this one away.
        //
        // ORDER MATTERS THREE WAYS and all three are why it sits exactly here:
        //   after mapbtnlog     -- the logger must keep reporting the PHYSICAL grip, or the next
        //                          person measuring it finds nothing and concludes the controller
        //                          sends nothing. Never blind the instrument that made the finding.
        //   after bind capture  -- a player must still be able to BIND an action to the right grip.
        //   before the rebind   -- left X injects 0x0200 further down. Swallowing after that would
        //                          eat the injection too and equipment would have no home at all.
        //
        // Not in menus: whatever RB does in the game's own UI is the game's business, and a mod
        // that eats a menu button to reserve it for a gameplay gesture has overreached.
        if (g_cfg.rgrip_swallow && g_cfg.rgrip_mask != 0 && !g_in_menu.load()) {
            state->Gamepad.wButtons &= (WORD)~(WORD)g_cfg.rgrip_mask;
        }

        // ---- MENU-ARMED CALIBRATION TRIGGERS. While the settings menu has armed a calibration,
        // the triggers ARE the gesture (RT = save & exit, LT = save & re-arm on release), so both
        // are sampled here and then EATEN -- ending a calibration must never fire the weapon
        // (same suppression lane as the scope's LT). With UEVR's own menu open this state was
        // already zeroed upstream by the VR mod, so an armed mode simply waits for the menu to
        // close; that is why the menu text says to close it.
        if (g_menu_calib_mode.load(std::memory_order_relaxed) != 0) {
            g_menu_calib_lt.store(state->Gamepad.bLeftTrigger  >= 128, std::memory_order_relaxed);
            g_menu_calib_rt.store(state->Gamepad.bRightTrigger >= 128, std::memory_order_relaxed);
            state->Gamepad.bLeftTrigger  = 0;
            state->Gamepad.bRightTrigger = 0;
        } else {
            g_menu_calib_lt.store(false, std::memory_order_relaxed);
            g_menu_calib_rt.store(false, std::memory_order_relaxed);
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
            // binddpadshift moves the shift off the stick and onto a held button. Set = the stick
            // gesture is REPLACED, not added to: leaving both live would mean a player who bound
            // it because the stick gesture misfires for them still has the stick gesture.
            // ---- HEAD PROXIMITY: the OTHER way to shift, and it moves the d-pad to the RIGHT
            // stick so the left one stays on locomotion. See Config.hpp for why every gate here
            // fails closed -- a false trigger costs turning, not a menu press.
            bool head_shift = false;
            {
                static bool s_near = false;
                static std::chrono::steady_clock::time_point s_since{};
                bool near_now = false;

                // The two-handed hold is excluded outright: the support hand sits on the barrel,
                // and raising the barrel puts it beside your face. That is a hold, never a d-pad.
                if (g_cfg.dpad_head && !g_stick_mode.load()
                    && !(g_in_menu.load() && g_cfg.menu_suppress)
                    && !halo::two_hand_latched()) {
                    const auto hi = API::VR::get_hmd_index();
                    Vec3 hp{}; Quat hq{};
                    if (hi >= 0 && get_pose((int32_t)hi, &hp, &hq, /*use_aim=*/false)) {
                        const int32_t idxs[2] = { API::VR::get_left_controller_index(),
                                                  API::VR::get_right_controller_index() };
                        float best_cm = 1.0e9f;
                        for (int32_t ci : idxs) {
                            Vec3 cp{}; Quat cq{};
                            if (ci < 0 || !get_pose(ci, &cp, &cq, /*use_aim=*/false)) continue;
                            // An EMPTY pose is exactly (0,0,0) and would read as a hand jammed
                            // against the head -- the same tracking dropout AimPoseGuard exists
                            // for. Skip it rather than shift on a pose that does not exist.
                            if (cp.x == 0.0f && cp.y == 0.0f && cp.z == 0.0f) continue;
                            const float dx = cp.x - hp.x, dy = cp.y - hp.y, dz3 = cp.z - hp.z;
                            const float d = std::sqrt(dx * dx + dy * dy + dz3 * dz3) * 100.0f;
                            if (d < best_cm) best_cm = d;
                        }
                        // Hysteresis: arming and releasing use different radii, so a hand hovering
                        // at the boundary cannot flicker the d-pad on and off.
                        const float arm_cm = g_cfg.dpad_head_cm;
                        const float rel_cm = g_cfg.dpad_head_cm + g_cfg.dpad_head_hyst_cm;
                        near_now = s_near ? (best_cm < rel_cm) : (best_cm < arm_cm);
                    }
                }

                const auto now = std::chrono::steady_clock::now();
                if (near_now && !s_near) s_since = now;
                s_near = near_now;
                if (s_near) {
                    // DWELL. A melee windup and a magazine grab both sweep a hand past the head;
                    // requiring it to STAY there is what separates a gesture in flight from a
                    // deliberate reach.
                    const auto held_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                             now - s_since).count();
                    head_shift = (held_ms >= (long long)g_cfg.dpad_head_dwell_ms);
                }
            }

            const bool shift_held = head_shift ? true
                : (g_cfg.bind_dpad_shift != 0)
                    ? ((state->Gamepad.wButtons & (WORD)g_cfg.bind_dpad_shift) != 0)
                    : (ry > g_cfg.map_rstick_dz);
            if ((g_cfg.map_dpad_shift || head_shift) && shift_held
                && !g_stick_mode.load()
                && !(g_in_menu.load() && g_cfg.menu_suppress)) {
                // WHICH STICK. Head-proximity takes the RIGHT one -- that is the entire point, so
                // the left stays free to walk with. The stick-up shift keeps taking the left,
                // because its trigger IS the right stick.
                const float lx = head_shift ? ((float)state->Gamepad.sThumbRX / 32767.0f)
                                            : ((float)state->Gamepad.sThumbLX / 32767.0f);
                const float ly = head_shift ? ry
                                            : ((float)state->Gamepad.sThumbLY / 32767.0f);
                const float dz = g_cfg.map_dpad_dz;

                // TURNING OFF while the right stick is a d-pad, or selecting a grenade would spin
                // you. g_raw_stick_x is what the turn lane reads (it was snapshotted before this
                // block), so clearing it here is the one place that reaches every consumer.
                if (head_shift) g_raw_stick_x = 0.0f;

                // A BOUND shift button is consumed, so holding it does not also do its native job
                // for the whole time you are selecting. (The stick gesture needs no equivalent --
                // the right stick is overwritten by the aim output regardless.) Stripped BEFORE
                // the injection below, or a shift bound onto a d-pad bit would eat the very press
                // it exists to produce.
                if (g_cfg.bind_dpad_shift != 0)
                    state->Gamepad.wButtons &= (WORD)~g_cfg.bind_dpad_shift;

                // Dominant axis only. Emitting both on a diagonal produces two simultaneous d-pad
                // presses, which menus read as a double input.
                if (std::fabs(ly) >= std::fabs(lx)) {
                    if (ly >  dz) state->Gamepad.wButtons |= XINPUT_GAMEPAD_DPAD_UP;
                    if (ly < -dz) state->Gamepad.wButtons |= XINPUT_GAMEPAD_DPAD_DOWN;
                } else {
                    if (lx >  dz) state->Gamepad.wButtons |= XINPUT_GAMEPAD_DPAD_RIGHT;
                    if (lx < -dz) state->Gamepad.wButtons |= XINPUT_GAMEPAD_DPAD_LEFT;
                }

                // ONLY THE STICK-UP SHIFT SUPPRESSES MOVEMENT, and the distinction is the whole
                // reason head-proximity exists.
                //
                // The stick-up shift turns the LEFT stick into the d-pad, so it must also stop it
                // walking you -- otherwise you stride off in the direction you are trying to
                // select. Head-proximity puts the d-pad on the RIGHT stick precisely so the left
                // one keeps working, so zeroing it here would delete the feature's entire point:
                // you would still be unable to move while switching grenades, which is what the
                // whole mode was built to fix.
                //
                // g_dpad_shift_active MEANS "THE LEFT STICK IS THE D-PAD", not "some shift is on".
                // Its other consumer is the movement-direction rotation further down, which is
                // skipped while the left stick is a d-pad. Under head-proximity the left stick is
                // a genuine movement stick, so that rotation MUST still run -- setting this true
                // would let you walk but in an uncorrected frame, which is worse than not walking:
                // the input works and goes the wrong way.
                if (!head_shift) {
                    state->Gamepad.sThumbLX = 0;
                    state->Gamepad.sThumbLY = 0;
                }
                g_dpad_shift_active = !head_shift;
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
            } else if (!g_stick_mode.load()) {
                // Every remap below is an ON-FOOT convenience, so none of them run while seated:
                // the promise stick mode makes (and the README documents) is that the pad behaves
                // exactly as the game's own layout in a vehicle. That matters beyond tidiness --
                // this rebind consumes a button whose native action while seated may be the one
                // that gets you OUT of the vehicle.
                // ---- ACTION BINDS (the in-game Controls panel).
                //
                // Each is a SOURCE the player chose for one of our actions; the destination is
                // whatever field already owns that action, so there is still exactly one place
                // that knows what the game reads for crouch/melee/reload.
                //
                // SPLIT ACROSS THE REBIND, and that is the whole subtlety. The source must be
                // consumed BEFORE mapfrom (so a bind beats the generic rebind when both name the
                // same mask, and so only the PHYSICAL press is eaten), but the destination must
                // be injected AFTER it -- for exactly the reason the stick-down crouch already
                // is. Doing both before would feed our own synthetic crouch (0x2000) into the
                // shipped mapfrom=0x2000 rebind and turn every bound crouch into equipment.
                //
                // All shipped as 0 = unbound, so this is inert until a player opts in and the
                // stick/swing gestures stay the designed default experience.
                WORD bind_inject = 0;
                {
                    const struct { int src; int dst; } binds[] = {
                        { g_cfg.bind_crouch, g_cfg.map_rstick_down },
                        { g_cfg.bind_melee,  g_cfg.melee_mask      },
                        { g_cfg.bind_reload, g_cfg.reload_mask     },
                        // Equipment's SECOND home. Its source (d-pad LEFT) is synthesised by the
                        // shift further up, so this reads the live mask rather than raw_btn -- the
                        // opposite of the grenade remap below, and for the opposite reason: there
                        // the injected mask must NOT satisfy the remap, here it is the whole point.
                        { g_cfg.bind_equip,  g_cfg.map_to          },
                    };
                    for (const auto& b : binds) {
                        if (b.src == 0 || (state->Gamepad.wButtons & (WORD)b.src) == 0) continue;
                        state->Gamepad.wButtons &= (WORD)~b.src;
                        if (b.dst != 0) bind_inject |= (WORD)b.dst;
                    }
                }

                // SCOPE is not a mask swap -- the toggle is ours, not the game's -- so it goes
                // through the same edge handler the trigger uses and injects nothing. Consumed
                // either way: a button bound to the scope must not also do its native job.
                if (g_cfg.bind_scope != 0) {
                    const bool down = (state->Gamepad.wButtons & (WORD)g_cfg.bind_scope) != 0;
                    scope_handle_button(down, g_in_menu.load(), g_stick_mode.load());
                    state->Gamepad.wButtons &= (WORD)~g_cfg.bind_scope;
                }

                if (g_cfg.map_from != 0 && (state->Gamepad.wButtons & (WORD)g_cfg.map_from) != 0) {
                    state->Gamepad.wButtons &= (WORD)~g_cfg.map_from;
                    if (g_cfg.map_to != 0) state->Gamepad.wButtons |= (WORD)g_cfg.map_to;
                }

                // Injected AFTER the rebind, so these masks reach the game untouched.
                // (Stick mode is already excluded by the branch condition above -- in a vehicle
                // the right stick is the camera, and looking down must not press crouch.)
                // NOT WHILE THE RIGHT STICK IS THE D-PAD. Under head-proximity the right stick is
                // the selector, so pushing it down means "d-pad DOWN" -- and without this gate it
                // would ALSO press crouch, every time, while you were choosing a grenade. Same
                // argument the stick-mode exclusion above makes for a vehicle camera: one stick
                // cannot mean two things at once, and the mode that borrowed it wins.
                if (g_cfg.map_rstick_down != 0 && ry < -g_cfg.map_rstick_dz && !head_shift) {
                    state->Gamepad.wButtons |= (WORD)g_cfg.map_rstick_down;
                }
                state->Gamepad.wButtons |= bind_inject;
            }
        }

        // ---- WEAPON SCOPE TRIGGER. The toggle edge lives in Scope.cpp; eating LT here is what
        // keeps Blam's native zoom (viewmodel hide, zoomed look speed) from ever engaging under
        // the VR presentation. Menus and vehicle seats are excluded inside, so LT still means
        // whatever the game says it means there.
        //
        // ---- AND THE TRIGGER IS MODAL (gripzoom).
        //
        // Holding the barrel is a MODE, and it is one the player can feel, so the off-hand trigger
        // can mean two things without ambiguity:
        //
        //   gripping     -> zoom toggle          (the only way to zoom; see below)
        //   not gripping -> THROW GRENADE
        //
        // This is why weapon_denied() no longer refuses the grip on one-handers: a Magnum you
        // cannot grip is a Magnum you cannot zoom, and the one-handers are exactly the weapons
        // that have a zoom. Consistency across weapons is the point -- grip then trigger zooms,
        // whatever you are holding.
        //
        // The consequence is deliberate and worth stating: while gripping, you have no grenade.
        // Let go, throw, re-grip. That is the trade for one button doing both jobs.
        //
        // DEFERRED, NOT INJECTED HERE -- see where this is consumed, below the holster steal. The
        // throw used to be written straight into wButtons at the edge, and the holster block then
        // stripped it right back out again a few lines later, because holster_throw_mask is the
        // SAME 0x0100. Trigger grenades therefore never reached the game at all.
        bool lt_throw_pending = false;
        {
            const bool gripping = g_cfg.grip_zoom && halo::two_hand_latched();

            // ZOOM DIES WITH THE GRIP. Releasing the barrel is an unambiguous "done aiming", and a
            // scope left on after the hand that opened it let go is a scope the player has to
            // remember to close with a button that no longer does that job.
            {
                static bool s_grip_prev = false;
                if (!gripping && s_grip_prev && g_cfg.grip_zoom) {
                    // SAY SO. This was the ONLY one of the three scope-close paths that logged
                    // nothing, which made it impossible to tell apart from the other two -- and I
                    // asserted it as the cause of a vanishing pane on exactly that non-evidence.
                    // The other two (RAY STALENESS in Scope.cpp, and the weapon watch) already name
                    // themselves; now all three do, so "why did the scope close" is a log read
                    // rather than an inference. Edge-triggered by construction: it only runs on the
                    // grip's falling edge.
                    const bool was_open = g_scope_active.load();
                    g_scope_active = false;
                    if (was_open) {
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] scope: CLOSED BY GRIP RELEASE (gripzoom=1). The "
                            "support grip let go, which this feature treats as 'done aiming'. Set "
                            "gripzoom=0 to decouple the scope from the grip.");
                    }
                }
                s_grip_prev = gripping;
            }

            if (gripping || !g_cfg.grip_zoom) {
                if (scope_handle_lt(state->Gamepad.bLeftTrigger, g_in_menu.load(),
                                    g_stick_mode.load())) {
                    state->Gamepad.bLeftTrigger = 0;
                }
            } else {
                // GRENADE on the press edge. Own hysteresis, matching the scope's, because this is
                // an analog axis and a wobble at the threshold must not double-throw. The mask is
                // injected rather than passed through: LT is not a button the game reads as throw,
                // and the physical grip that IS that mask has been swallowed upstream.
                static bool s_lt_down = false;
                const uint8_t on_t  = (uint8_t)(g_cfg.scope_thresh * 255.0f);
                const uint8_t off_t = (uint8_t)(on_t / 2);
                const uint8_t lt    = state->Gamepad.bLeftTrigger;
                const bool blocked  = g_in_menu.load() || g_stick_mode.load();
                if (!blocked && !s_lt_down && lt >= on_t) {
                    s_lt_down = true;
                    lt_throw_pending = true;   // injected below the holster steal, not here
                } else if (s_lt_down && lt <= off_t) {
                    s_lt_down = false;
                }
                // Eaten either way, so Blam's native zoom never engages under the VR presentation
                // -- the same reason the scope path eats it. Menus and seats keep the game's own
                // meaning, which is why `blocked` gates the throw but not this.
                if (!blocked && g_cfg.scope_eat_lt) state->Gamepad.bLeftTrigger = 0;
            }
        }
        // ---- HOLSTERS: TAKE THE BUTTONS WE SYNTHESISE, BEFORE WE SYNTHESISE THEM.
        //
        // The swap / throw / grenade-switch masks are real game buttons. Holster.cpp decides when
        // they should fire from a body-frame gesture, so the physical press must not ALSO reach the
        // game -- otherwise reaching for a shoulder both stows the weapon and does whatever that
        // button natively does. Stolen here, re-injected below, so ours is the only one that lands.
        //
        // His version also stole the grip while two-handed aiming could use it. That is NOT ported:
        // TwoHand came with the palette positioning hook and is deliberately left out of this
        // extraction, so the grip keeps its native meaning here.
        //
        // Gated on g_stick_mode rather than his g_unit_mounted (which belongs to his vehicle work
        // and does not exist in this tree). Stick mode already covers vehicles, cutscenes and death.
        if (g_cfg.holster_steal_buttons != 0 && g_cfg.holster_enabled
            && !g_in_menu.load(std::memory_order_relaxed)
            && !g_stick_mode.load(std::memory_order_relaxed)) {
            // SWAP IS DELIBERATELY NOT STOLEN. holster_swap_mask is Y, the game's own weapon
            // switch, and stealing it was pure loss: the over-the-shoulder gesture does not
            // require a button, so taking Y away bought nothing and simply removed the native
            // switch. Both work now, independently -- press Y, or reach over your shoulder.
            //
            // The inherited comment above ("reaching for a shoulder both stows the weapon and
            // does whatever that button natively does") describes a gesture that is triggered BY
            // that button. Ours is body-frame only, so the premise does not hold here. That is
            // what made this steal look justified while it was quietly disabling a control.
            const WORD steal = (WORD)((WORD)g_cfg.holster_throw_mask
                                    | (WORD)g_cfg.holster_gswitch_mask);
            const WORD before = state->Gamepad.wButtons;
            state->Gamepad.wButtons &= (WORD)~steal;
            if (g_cfg.map_btn_log && before != state->Gamepad.wButtons) {
                API::get()->log_info("[Halo-CampE-UEVR] HOLSTER STEAL: 0x%04X -> 0x%04X (removed 0x%04X)",
                                     (unsigned)before, (unsigned)state->Gamepad.wButtons,
                                     (unsigned)(WORD)(before & steal));
            }
        }
        // ---- MELEE BY SWING. One atomic load and a clock read; the decision was made on the
        // game thread (see Gesture.cpp). Placed after the rebind block for the same reason the
        // crouch mask is: a synthetic press must reach the game as itself, not get remapped.
        //
        // Deliberately OUTSIDE the !g_stick_mode branch above rather than relying on it -- the
        // detector already refuses to arm in stick mode, and duplicating that gate here would
        // mean two places to keep in agreement about when melee is legal.
        if (g_cfg.melee_mask != 0 && melee_press_active()) {
            state->Gamepad.wButtons |= (WORD)g_cfg.melee_mask;
        }
        // ...and put ours back. After the steal, so the steal cannot eat our own synthetic press.
        if (g_cfg.holster_swap_mask != 0 && holster_swap_press_active())
            state->Gamepad.wButtons |= (WORD)g_cfg.holster_swap_mask;
        if (g_cfg.holster_throw_mask != 0 && holster_throw_press_active())
            state->Gamepad.wButtons |= (WORD)g_cfg.holster_throw_mask;
        if (g_cfg.holster_gswitch_mask != 0 && holster_gswitch_press_active())
            state->Gamepad.wButtons |= (WORD)g_cfg.holster_gswitch_mask;

        // ---- THE TRIGGER'S GRENADE, landing here rather than at the edge that decided it.
        //
        // Same reason melee and the holster re-injects sit below the steal: a synthetic press has
        // to be written AFTER anything that strips its mask, or it is removed before the game ever
        // sees it. This one was written above and stripped here, because grenade_action and
        // holster_throw_mask are both 0x0100 -- so the throw was injected and eaten every time,
        // and trigger grenades did not work at all.
        //
        // It is deliberately NOT re-ordered by moving the steal instead: the steal must stay ahead
        // of the holster injections that follow it. Deferring the one injector that was on the
        // wrong side is the change that leaves every other ordering intact.
        if (lt_throw_pending && g_cfg.grenade_action != 0)
            state->Gamepad.wButtons |= (WORD)g_cfg.grenade_action;

        // ---- VR RELOAD.
        //
        // Two jobs, and the ORDER matters. Suppression first: while the magazine is out the
        // trigger is swallowed, which is the whole reason the gesture has stakes. Then the
        // synthesised press, which must survive that suppression -- it fires at the instant the
        // state machine returns to Idle, so it is not suppressed by its own condition, but doing
        // it in the other order would still be fragile to a future edit.
        if (reload_fire_suppressed() || holster_fire_suppressed()) {
            // Both paths: Halo reads fire from the analog trigger, but a pad or a remap can put it
            // on a button, and swallowing only one of the two leaves a hole.
            state->Gamepad.bRightTrigger = 0;
            state->Gamepad.wButtons &= (WORD)~XINPUT_GAMEPAD_RIGHT_SHOULDER;
        }

        // STRIP THE PLAYER'S OWN RELOAD PRESS.
        //
        // This is the whole mechanism and it was missing: deferring the synthetic press does
        // nothing on its own, because the physical press still reaches the game on the same poll.
        // First live test reloaded normally while the state machine ran alongside it, correctly,
        // and invisibly.
        //
        // The state machine already saw this button -- g_pad_buttons is published at the top of
        // this callback, before any remapping -- so removing it here costs no information.
        // SWALLOW ONLY WHAT WE ARE ACTUALLY USING.
        //
        // Both borrowed buttons have day jobs, and a full playthrough proved the cost of ignoring
        // that: the reload button is ALSO Interact and Enter Vehicle, so stripping it
        // unconditionally meant no vehicles and no interaction for an entire chapter, and the left
        // grip is Throw Grenade, so every magazine grab lobbed a frag.
        //
        // Reload button: withheld only while the press is still short enough to be a tap. Past the
        // hold threshold it is released to the game, which gets a press starting slightly late
        // rather than never.
        if (reload_swallow_reload_button()) {
            state->Gamepad.wButtons &= (WORD)~g_cfg.reload_mask;
        }
        // (The grip swallow deliberately runs EARLIER -- see the note by reload_note_buttons.)

        // ---- GRENADE ON A BUTTON. OFF BY DEFAULT (grenade_from = 0) -- the throw lives on the
        // modal off-hand trigger further up. This stays as the opt-in for putting it on a button.
        //
        // Pure remap: consume grenade_from and inject grenade_action, the mask the game already
        // reads as throw. After the grip swallow, so injecting that mask cannot be eaten by it.
        //
        // Gated on raw_btn, NOT the live state, so our own injected masks cannot satisfy it -- the
        // melee swing injects melee_mask, and reading live state would turn every swing into a
        // grenade request. See raw_btn's note above.
        //
        // THE COST OF THAT, and why grenade_from must name a mask nothing else claims: a raw_btn
        // consumer cannot be disarmed by an upstream strip. It sees the physical press whatever
        // earlier stages did with it, so it does not participate in "first match wins". This was
        // set to 0x2000 while map_from was also 0x2000, and one press of left X fired equipment
        // AND a grenade -- the rebind's strip was simply invisible to this test.
        if (g_cfg.grenade_from != 0 &&
            (raw_btn & (WORD)g_cfg.grenade_from) != 0) {
            state->Gamepad.wButtons &= (WORD)~g_cfg.grenade_from;
            if (g_cfg.grenade_action != 0) {
                state->Gamepad.wButtons |= (WORD)g_cfg.grenade_action;
            }
        }

        // ...and put it back only when WE decide the magazine is seated. After the strip, so the
        // strip cannot eat our own synthetic press.
        if (g_cfg.reload_mask != 0 && reload_press_active()) {
            state->Gamepad.wButtons |= (WORD)g_cfg.reload_mask;
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

                // movelive=2: take the AIM term live as well.
                //
                // The comment below ("aim moves far more slowly, so taking it at tick rate is
                // fine") was true when the aim was produced by the rate actuator. With blamangles
                // driving the sim's angular control state directly, the aim tracks the hand as
                // fast as the head does -- so latching it at ~32 Hz while the head term is live
                // puts up to ~30 ms of skew straight into the movement direction. Waving the
                // controller while pushing the stick then swings where you walk.
                //
                // g_desired_yaw is the COMMANDED aim, not a measurement of it, so it carries
                // neither sampling lag nor actuator settling -- and with blamangles the commanded
                // value is what the sim is actually using.
                // movelive=3: SAMPLE the aim term here instead of reading the published one.
                //
                // movelive=2 below assumes g_desired_yaw is current. It is not, on the default
                // configuration: with aimrate=1 the aim law runs in THIS callback but further
                // down (see the render-rate law), so the value read here is always the previous
                // poll's setpoint paired against the current rendered view. That is a skew of one
                // XInput poll whose ANGULAR SIZE grows with how fast the hand is turning -- which
                // is exactly the reported symptom: rotate the controller while walking and the
                // walk direction is perturbed, in proportion to the rotation rate, settling the
                // moment the hand stops. It also latches on any early-out further down (aim
                // calibration held, law disarmed, ControlRotation unreadable).
                //
                // desired_aim_now() is the same function the blamangles driver calls to write the
                // sim's angular control state, so this pairs the view against the value the sim is
                // actually being given, at one instant, with no publication in between.
                //
                // Costs one extra pose read per poll WHILE THE STICK IS DEFLECTED (the enclosing
                // branch), on a path that already does one per poll for the aim law.
                float sampled_aim = 0.0f, sampled_pitch = 0.0f;
                const bool have_sampled = (g_cfg.move_live >= 3)
                                       && desired_aim_now(&sampled_aim, &sampled_pitch);
                if (have_sampled && g_have_render_yaw.load()) {
                    delta = wrap180(g_render_view_yaw.load() - sampled_aim);
                } else
                if (g_cfg.move_live >= 2 && g_have_render_yaw.load() && g_aim_law_armed.load()) {
                    delta = wrap180(g_render_view_yaw.load() - g_desired_yaw.load());
                } else
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

#if HALO_VR_DEV
                // MOVERESID -- the movement-frame error, measured rather than felt. See the note on
                // move_resid in Config.hpp for what the shape of this number means; the short
                // version is that this residual IS the coupling, and delta is not.
                //
                // Both terms are taken here, in the same callback, on the same poll: the aim term
                // the rotation actually used, and the sim's actual aim read straight out of
                // ControlRotation. Anything sampled elsewhere would reintroduce the skew being
                // measured. The hand rate is printed alongside because the discriminator between
                // "lag" and "wrong frame" is whether the residual tracks it.
                if (g_cfg.move_resid > 0 &&
                    (g_move_applied.load() % (uint32_t)g_cfg.move_resid) == 0) {
                    const float aim_used = have_sampled ? sampled_aim
                                         : (g_cfg.move_live >= 2 ? g_desired_yaw.load()
                                                                 : g_move_aim_smooth.load());
                    double ap = 0.0, ay = 0.0;
                    const bool have_actual = read_control_rotation_hook(&ap, &ay);

                    // Hand rate, differentiated HERE across the logged samples rather than taken
                    // from g_setpoint_rate_dps: that one is only written on the feedforward path,
                    // which is gated on a measured plant gain > 10 and so can sit at 0 for a whole
                    // session -- a rate column that silently reads zero would make a lag look like
                    // a frame error, which is the one distinction this line exists to draw.
                    static float prev_used = 0.0f;
                    static std::chrono::steady_clock::time_point prev_t{};
                    static bool  have_prev_used = false;
                    const auto now_t = std::chrono::steady_clock::now();
                    float rate_dps = 0.0f;
                    if (have_prev_used) {
                        const float rdt = std::chrono::duration<float>(now_t - prev_t).count();
                        if (rdt > 1e-4f) rate_dps = wrap180(aim_used - prev_used) / rdt;
                    }
                    prev_used = aim_used; prev_t = now_t; have_prev_used = true;

                    API::get()->log_info(
                        "[Halo-CampE-UEVR] MOVERESID live=%d view=%.2f aimUsed=%.2f aimActual=%.2f(ok%d)"
                        " RESID=%.2f delta=%.2f th=%.2f stickIn=%.1f stickOut=%.1f handRate=%.0fdps",
                        g_cfg.move_live, g_render_view_yaw.load(), aim_used,
                        (float)ay, (int)have_actual,
                        have_actual ? wrap180((float)ay - aim_used) : 0.0f,
                        delta, th * RAD2DEG,
                        std::atan2(lx, ly) * RAD2DEG, std::atan2(nx, ny) * RAD2DEG,
                        rate_dps);
                }
#endif
            }
        }

        // ---- VEHICLE HARD BRAKE, pad-side delivery. Stick mode has already released the left
        // stick to the game (movement rotation and d-pad shift stand down there), so writing it
        // here is uncontested. Mode 2 = full stick-back, the pad's native brake input; mode 3 = a
        // button mask the user bound to hard brake in the game's own controller settings.
        if (g_brake_pad.load()) {
            if (g_cfg.brake_mode == 2) {
                state->Gamepad.sThumbLY = -32768;
                state->dwPacketNumber++;
            } else if (g_cfg.brake_mode == 3 && g_cfg.brake_mask != 0) {
                state->Gamepad.wButtons |= (WORD)g_cfg.brake_mask;
                state->dwPacketNumber++;
            }
        }

        // ---- WHAT THE GAME ACTUALLY RECEIVES. The companion to the raw logger far above, and the
        // whole reason it needed one.
        //
        // That logger samples BEFORE any of our injections, so every mask this mod SYNTHESISES --
        // the trigger's grenade, equipment, crouch, melee, the d-pad the shift makes -- is
        // invisible to it by construction. That blind spot was read as evidence twice in one day:
        // once concluding a head-prox d-pad press "reported 0x0200" when the injection it should
        // have shown happens later, and once leaving a grenade fix unverifiable because the mask
        // we inject could never appear. A logger that cannot see the thing you changed is worse
        // than no logger, because its silence looks like data.
        //
        // Placed after the last write to wButtons and before the aim work, which touches sticks
        // only. Prints on a change of the FINAL mask and names the DIFFERENCE from the physical
        // press, so +0x0100 is us injecting a throw and -0x8000 would be us eating a Y.
        if (g_cfg.map_btn_log) {
            static WORD s_prev_final = 0;
            const WORD final_btn = state->Gamepad.wButtons;
            if (final_btn != s_prev_final) {
                s_prev_final = final_btn;
                const WORD added   = (WORD)(final_btn & (WORD)~raw_btn);
                const WORD removed = (WORD)(raw_btn & (WORD)~final_btn);
                API::get()->log_info(
                    "[Halo-CampE-UEVR] BTN final=0x%04X (raw 0x%04X)  added=0x%04X removed=0x%04X%s",
                    (unsigned)final_btn, (unsigned)raw_btn, (unsigned)added, (unsigned)removed,
                    (added == 0 && removed == 0) ? "  [untouched]" : "");
            }
        }

        // (A pad binding for the pose-match calibration used to be published here. Removed -- see
        // the note at the calibration key poll for why a controller button is the wrong input for
        // a gesture that writes persistent state.)

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

            // MEASURE THE PLANT HERE, on the path that is actually driving.
            //
            // This is the fix for the gain having been stuck at 0: measurement only ever ran on the
            // tick path and was skipped outright under aim_rate_render, which is the default. The
            // pairing is honest here in a way it could not be there -- this call knows exactly what
            // deflection it applied, and update_gain_measurement integrates it across the interval
            // between aim changes rather than blaming one sample for a whole tick.
            //
            // Uses the same aim reading the law just consumed, so measurement and control never
            // disagree about where the aim was.
            {
                static GainMeasState hook_gain;
                update_gain_measurement(hook_gain, ay, law_rx, dt);
            }

            if (law_rx == 0.0f && law_ry == 0.0f) return;
            state->Gamepad.sThumbRX = to_raw(law_rx);
            state->Gamepad.sThumbRY = to_raw(law_ry);
            state->dwPacketNumber++;
            return;
        }

        // STICK MODE: give the player back the look feel their game settings no longer provide.
        //
        // The game is configured for the AIM LOOP -- look sensitivity high, look deadzone low --
        // because those are the loop's authority ceiling and its minimum correction size. Here the
        // player's own stick is driving, so those same settings read as a twitchy, drifting camera.
        // Scale it down and re-impose the deadzone the game is no longer applying.
        //
        // MENUS ARE EXCLUDED: menu navigation is a discrete "did the stick pass a threshold"
        // input, not a rate, so scaling it would just make menus harder to move through -- and a
        // re-imposed deadzone on top of the menu's own could swallow inputs entirely.
        //
        // Left stick is untouched: it is drive/throttle, and the settings compensated here are the
        // game's LOOK settings, which only apply to the right stick.
        if (g_stick_mode.load() && !g_in_menu.load()
            && (g_cfg.stick_scale < 1.0f || g_cfg.stick_dz > 0.0f)) {
            float sx = (float)state->Gamepad.sThumbRX / 32767.0f;
            float sy = (float)state->Gamepad.sThumbRY / 32767.0f;
            const float mag = std::sqrt(sx * sx + sy * sy);
            if (mag > 0.0001f) {
                // Radial deadzone, then RESCALE the remainder back over the full range, so the
                // player keeps full deflection at the rim instead of losing the top of their range.
                float m = mag;
                if (g_cfg.stick_dz > 0.0f) {
                    m = (mag <= g_cfg.stick_dz) ? 0.0f
                                                : (mag - g_cfg.stick_dz) / (1.0f - g_cfg.stick_dz);
                }
                m = clampf(m * g_cfg.stick_scale, 0.0f, 1.0f);
                const float k = m / mag;   // preserves direction; magnitude carries the shaping
                state->Gamepad.sThumbRX = to_raw(clampf(sx * k, -1.0f, 1.0f));
                state->Gamepad.sThumbRY = to_raw(clampf(sy * k, -1.0f, 1.0f));
                state->dwPacketNumber++;
            }
            return;   // stick mode never runs the aim output below
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
