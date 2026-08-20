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

// UE object/name helpers: TrackedObject (recycle-safe handles), FName resolution, class names.
#include "UeObject.hpp"

// First-person weapon rig: the gun/arms follow the hand via RELATIVE component transforms.
#include "Rig.hpp"

// Aim reticule: our own mesh reticule, plus one hosting the game's own reticle widget.
#include "Reticule.hpp"
#include "CutsceneHint.hpp"

// The weapon scope: LT-toggled magnified pane on the aim ray (native zoom stays suppressed).
#include "Scope.hpp"

// The aim control loop: Halo's own aim is steered to follow the controller via synthesized stick.
#include "MotionAimControl.hpp"
#include "AimTrace.hpp"
#include "MemScan.hpp"
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
#define HALO_VR_VERSION "0.3.3"

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
enum PerfSite { PERF_CFG = 0, PERF_RETICLE, PERF_RIG, PERF_SHELL, PERF_TRACE, PERF_BRIDGE,
                PERF_NAVHOST, PERF_COUNT };
const char* const kPerfName[PERF_COUNT] = { "load_config   ", "reticle_rescan", "resolve_rig   ",
                                            "resolve_shell ", "reticule_trace", "menu_bridge   ",
                                            "navw_rehost   " };

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
// Head displacement from the standing origin at the instant of the freeze, VR room space. The hold
// rides any change from this so the frozen weapon stays put relative to the PLAYER rather than to
// the camera, which is what keeps head movement during the hold out of the fitted mount. Always
// zero while the head is leashed. See the hold-ride block in the rig update.
Vec3 g_calib_delta_room{0.0f, 0.0f, 0.0f};
bool g_calib_have_delta = false;
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
        if (cn != L"SkeletalMeshComponent" && cn != L"StaticMeshComponent" &&
            cn != L"InstancedStaticMeshComponent") continue;

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
    g_reticle_count = 0;

    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;

    g_menu_candidate_count = 0;

    // ---- HOIST AND MEMOISE. This walks the entire UObject array and built TWO std::wstrings per
    // object: the object's class name, and -- inside the comparison below -- the wanted class name,
    // reconstructed from scratch on every iteration for a value that cannot change while the loop
    // runs.
    //
    // Measured at 83.9 ms per sweep. reticle_stray_check_due arms a ~12 s window and the 120-tick
    // throttle lets it fire four times inside that, so a HUD rebuild costs 332 ms of game-thread
    // stall spread over the following seconds. The demand gate above reduced how OFTEN this runs
    // without touching what it costs when it does.
    //
    // Pure de-duplication: the same strings are compared in the same order, results identical.
    // After: 29.8 ms.
    const std::wstring wanted = wanted_widget_class();
    std::unordered_map<const void*, std::wstring> name_of_class;

    // The navpoint class comes from config (navclass -- a pak-inventory guess until confirmed
    // live), so it is widened once per sweep rather than per object -- the same reasoning as the
    // hoist above, arrived at separately. The default lives HERE, not in the struct initializer:
    // a char-array string default on the global g_cfg does not survive MSVC's
    // constant-initialization (see the note on nav_class in Config.hpp).
    g_nav_count = 0;
    const char* nav_eff = (g_cfg.nav_class[0] != '\0') ? g_cfg.nav_class : "WBP_Navpoints";
    wchar_t nav_class_w[64] = {0};
    if (g_cfg.nav_fix || g_cfg.nav_world) {
        for (size_t k = 0; k < 63 && nav_eff[k] != '\0'; ++k) {
            nav_class_w[k] = (wchar_t)(unsigned char)nav_eff[k];
        }
    }

    const int32_t n = arr->get_object_count();
    for (int32_t i = 0; i < n; ++i) {
        auto* o = arr->get_object(i);
        if (o == nullptr) continue;
        // MEMOISED ON THE CLASS, not rebuilt per object. Objects outnumber classes by orders of
        // magnitude here, so this reconstructed the same handful of names tens of thousands of
        // times per sweep.
        auto* ocls = o->get_class();
        if (ocls == nullptr) continue;
        auto memo = name_of_class.find(ocls);
        if (memo == name_of_class.end()) memo = name_of_class.emplace(ocls, class_name_of(o)).first;
        const std::wstring& cn = memo->second;

        const bool is_reticle = cn.find(wanted) != std::wstring::npos;
        const bool is_menu    = g_cfg.menu_detect && is_menuish_class(cn);
        const bool is_nav     = nav_class_w[0] != 0 && cn.find(nav_class_w) != std::wstring::npos;
        if (!is_reticle && !is_menu && !is_nav && !g_cfg.menu_dump) continue;

        auto* cls = ocls;
        if (cls != nullptr && o == cls->get_class_default_object()) continue;   // never the CDO

        if (is_reticle && g_reticle_count < 8) {
            g_reticles[g_reticle_count].obj.set_at(o, i);
            g_reticles[g_reticle_count].found_tick = tick;
            ++g_reticle_count;
        }
        if (is_nav && g_nav_count < 4) {
            g_navpoints[g_nav_count].set_at(o, i);
            ++g_nav_count;
        }
        // (strays are collapsed after the loop, once the full list exists -- see below)
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

    // Navpoint resolution, logged on CHANGE for the same reason as the reticles above. "0
    // resolved" while navfix=1 means the navclass guess is wrong -- menudump the live widget
    // names and correct the config, not the code.
    if (g_cfg.nav_fix || g_cfg.nav_world) {
        static int last_nav = -1;
        if (g_nav_count != last_nav) {
            last_nav = g_nav_count;
            API::get()->log_info("[Halo-CampE-UEVR] NAVFIX: %d navpoint container(s) resolved "
                                 "(navclass='%s')", g_nav_count, nav_eff);
        }
    }

    // Hide any of the game's flat crosshairs that are not the one we host. Placed HERE, at the end
    // of the sweep, so it reuses the list that was just built and costs nothing of its own -- and
    // so it sees the complete list rather than deciding "stray" from a partial scan.
    reticle_collapse_strays();
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

NavwKind navw_classify(const std::wstring& cn) {
    if (cn.find(L"Objective")    != std::wstring::npos
     || cn.find(L"Scripted")     != std::wstring::npos) return NAVW_OBJECTIVE;
    // CO-OP PARTNERS / allies. The pak carries WBP_NavpointWidgetPlayer and the
    // MI_UI_Navmarker_Ally_* materials; without this they classify as "other" and draw at FULL
    // base size -- bigger than the objectives, which is backwards.
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
    const std::wstring wcn = class_name_of(w);
    const NavwKind kind = navw_classify(wcn);
    const float ov = navw_class_override(wcn);
    const float mult = (ov > 0.0f) ? ov : navw_kind_size(kind);
    if (slot >= 0 && slot < 8) {
        g_navw_slot_class[slot] = (void*)nav_wcls;
        g_navw_slot_kind[slot]  = kind;
        g_navw_slot_size[slot]  = mult;
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
API::UObject* navw_ensure_slot(int i) {
    if (auto* c = g_navw_pool[i].get_checked(L"WidgetComponent")) return c;
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
    // version of this marker stayed invisible. navw_host_class() supplies the art; passing
    // nullptr takes the configured/default class, and the placement loop re-hosts per navpoint
    // type as slots are reused.
    const bool hosted = navw_host_class(comp, i, nullptr);

    widget_quad_finish(owner, comp, /*bounds_scale=*/10.0f);

    g_navw_pool[i].set(comp);
    API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: marker slot %d created as WIDGET quad on %s "
                         "(mic=%d hosted=%d)", i, narrow(class_name_of(owner)).c_str(),
                         (int)g_navw_mid_ok[i], (int)hosted);
    return comp;
}

void navw_hide_all() {
    for (int i = 0; i < 8; ++i) {
        auto* c = g_navw_pool[i].get_checked(L"WidgetComponent");
        if (c == nullptr) continue;
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        p[0] = 0;
        c->call_function(L"SetVisibility", p);
    }
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
        if (auto* c = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.GameplayStatics"))
            gps = c->get_class_default_object();
        if (auto* c = API::get()->find_uobject<API::UClass>(L"Class /Script/UMG.WidgetLayoutLibrary"))
            wll = c->get_class_default_object();
    }
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
        auto* map = mgr->get_property_data<FMapRaw>(L"NavpointInstances");
        if (map == nullptr || map->data == nullptr || map->num <= 0) {
            if (g_navw_shown) { g_navw_shown = false; navw_hide_all(); g_navw_placed_n = 0; }
            return;
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

        int used = 0, seen_entries = 0;
        for (int32_t s = 0; s < map->max && s < 16 && used < 8; ++s) {
            const uint8_t* elem = reinterpret_cast<const uint8_t*>(map->data) + (size_t)s * stride;
            if (IsBadReadPtr(elem, (size_t)stride)) continue;
            Vec3 wp{};
            if (!try_chain(elem, 0x20, 0x28, &wp) && !try_chain(elem, 0x28, 0x38, &wp)) continue;
            ++seen_entries;

            auto* comp = navw_ensure_slot(used);
            if (comp == nullptr) break;

            // THIS navpoint's own art. Element+0x10 is its widget class (anatomy dump: +0x08 is
            // the live widget instance, +0x10 its WidgetBlueprintGeneratedClass), so an objective,
            // a co-op partner and a tracked enemy each keep their authored icon and colour
            // instead of every marker wearing the objective's.
            {
                API::UClass* ecls = nullptr;
                if (!IsBadReadPtr(elem + 0x10, 8)) {
                    auto* cand = *reinterpret_cast<API::UObject* const*>(elem + 0x10);
                    if (cand != nullptr && !IsBadReadPtr(cand, 0x30)
                        && class_name_of(cand).find(L"WidgetBlueprintGeneratedClass") != std::wstring::npos) {
                        ecls = reinterpret_cast<API::UClass*>(cand);
                    }
                }
                navw_host_class(comp, used, ecls);
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

            // PULL BACK toward the player, the reticule's surface-offset idea applied to the
            // objective itself: draw the marker navworldback cm SHORT of the thing it marks, so
            // it floats in front of its target rather than inside it. Applied before the clamp,
            // so it only bites when the objective is nearer than navworldmax.
            float draw_dist = true_dist - g_cfg.nav_world_back;
            if (draw_dist > g_cfg.nav_world_max) draw_dist = g_cfg.nav_world_max;
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
            if (g_cfg.nav_world_trace && hit_trace_ready()) {
                const Vec3 tstart{vo.x, vo.y, vo.z};
                const Vec3 tend{vo.x + dir.x * draw_dist, vo.y + dir.y * draw_dist,
                                vo.z + dir.z * draw_dist};
                Vec3 hit{};
                API::UObject* ignore[2] = {};
                int n_ignore = 0;
                if (auto* pawn = API::get()->get_local_pawn(0)) ignore[n_ignore++] = pawn;
                if (auto* rigc = reinterpret_cast<API::UObject*>(g_rig_component.load())) {
                    if (auto* wep = rigc->get_outer()) ignore[n_ignore++] = wep;
                }
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
                comp->call_function(L"K2_SetWorldRotation", p);
            }
            {
                // Scale from the DRAWN distance (constant apparent size however far the marker
                // was pulled in) times the PER-KIND multiplier: an objective should read from
                // across the level, a floor weapon should not compete with it.
                const float mult = (used < 8) ? g_navw_slot_size[used] : 1.0f;
                const double sc = (double)(g_cfg.nav_world_scale * mult * draw_dist / 1000.0f);
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                auto* d = reinterpret_cast<double*>(p);
                d[0] = sc; d[1] = sc; d[2] = sc;
                comp->call_function(L"SetWorldScale3D", p);
            }
            // Publish for the render-rate re-place (see g_navw_placed).
            if (used < 8) {
                g_navw_placed[used] = NavwPlaced{wp.x, wp.y, wp.z, draw_dist};
            }
            { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1; comp->call_function(L"SetVisibility", p); }
            {
                const float gain = g_navw_compensated
                                 ? 1.0f : g_cfg.aim_widget_gain * g_cfg.aim_widget_tint;
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                auto* c = reinterpret_cast<float*>(p);
                c[0] = gain * g_cfg.nav_world_cr;
                c[1] = gain * g_cfg.nav_world_cg;
                c[2] = gain * g_cfg.nav_world_cb;
                c[3] = 1.0f;
                comp->call_function(L"SetTintColorAndOpacity", p);
            }
#if HALO_VR_DEV
            if (g_cfg.nav_world_log && (tick % 64) == 0) {
                API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD[%d]: objective=(%.0f,%.0f,%.0f) "
                                     "true=%.0fcm drawn=%.0fcm%s", used, wp.x, wp.y, wp.z,
                                     true_dist, draw_dist, occluded ? " [pulled in front]" : "");
            }
#endif
            ++used;
        }
        for (int i = used; i < 8; ++i) {
            auto* c = g_navw_pool[i].get_checked(L"WidgetComponent");
            if (c == nullptr) continue;
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            p[0] = 0;
            c->call_function(L"SetVisibility", p);
        }
        g_navw_placed_n = used;   // arm/disarm the render-rate re-place to match
        g_navw_shown = (used > 0) || g_navw_shown;

        // ---- HIDE THE FLAT NAVPOINTS while the world markers are doing their job. Two sets of
        // waypoints for the same objectives is confusing, and the flat ones are the pair that is
        // wrong in VR (projected against the aim camera).
        //
        // SELF-HEALING, deliberately: the hide is tied to used > 0, so if this lane ever stops
        // placing -- no manager, empty map, stick mode, kill switch -- the game's own markers
        // come straight back rather than leaving the player with no waypoints at all. Re-asserted
        // every tick because the HUD re-shows its children on weapon swap, respawn and scope.
        if (g_cfg.nav_hide_flat) {
            static bool s_flat_hidden = false;
            const bool want_hidden = (used > 0);
            API::UObject* container = nullptr;
            for (int i = 0; i < g_nav_count; ++i) {
                auto* w = g_navpoints[i].get();
                if (w == nullptr || !navw_is_live_widget(w)) continue;
                if (auto* pp = w->get_property_data<void*>(L"NavpointsContainer")) {
                    if (*pp != nullptr) { container = reinterpret_cast<API::UObject*>(*pp); break; }
                }
            }
            if (container != nullptr) {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                p[0] = want_hidden ? 1 : 4;   // 1 = Collapsed, 4 = SelfHitTestInvisible
                container->call_function(L"SetVisibility", p);
                if (want_hidden != s_flat_hidden) {
                    s_flat_hidden = want_hidden;
                    API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: flat navpoint layer %s",
                                         want_hidden ? "hidden (world markers active)"
                                                     : "restored (no world markers)");
                }
            }
        }
        static uint32_t last_tlog = 0;
        if (tick - last_tlog >= 600) {
            last_tlog = tick;
            API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: %d marker(s) at OBJECTIVE world "
                                 "positions (map num=%d max=%d, %d entr(ies) resolved)",
                                 used, map->num, map->max, seen_entries);
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
                comp->call_function(L"K2_SetWorldLocation", p);
            }
            {
                // FACE THE VIEWER -- a widget quad is edge-on invisible without this.
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                auto* d = reinterpret_cast<double*>(p);
                d[0] = (double)(-m_pitch);            // normal back toward the viewer
                d[1] = (double)wrap180(m_yaw + 180.0f);
                d[2] = 0.0;
                comp->call_function(L"K2_SetWorldRotation", p);
            }
            {
                const double sc = (double)(g_cfg.nav_world_scale * clampf(dist, 300.0f, 20000.0f) / 1000.0f);
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                auto* d = reinterpret_cast<double*>(p);
                d[0] = sc; d[1] = sc; d[2] = sc;
                comp->call_function(L"SetWorldScale3D", p);
            }
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
            cont->call_function(L"GetChildrenCount", p);
            cn = *reinterpret_cast<int32_t*>(p);
        }
        int got = 0;
        for (int32_t ci = 0; ci < cn && got < cap; ++ci) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            *reinterpret_cast<int32_t*>(p) = ci;
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
                slot->call_function(L"GetPosition", p);
                const auto* d = reinterpret_cast<const double*>(p);
                cx = d[0]; cy2 = d[1];
            }
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
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
            comp->call_function(L"K2_SetWorldLocation", p);
        }
        {
            const double s = (double)(g_cfg.nav_world_scale * dist / 1000.0f);
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            auto* d = reinterpret_cast<double*>(p);
            d[0] = s; d[1] = s; d[2] = s;
            comp->call_function(L"SetWorldScale3D", p);
        }
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

    // Above every early-out below, so the numbers still arrive when the driver is disabled or
    // parked in a menu -- "it stutters at the frontend too" is a diagnosis, not a gap.
    perf_report(tick);

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
        const int want = g_cfg.hmd_leash ? 1 : 0;
        if (want != last_want) {
            last_want = want;
            API::VR::set_mod_value("VR_RoomscaleMovement", false);
            API::get()->log_info("[Halo-CampE-UEVR] LEASH: VR_RoomscaleMovement forced OFF -- it is a "
                                 "zero-radius lateral leash of UEVR's own, and hmdleash owns this "
                                 "behaviour now (hmdleash=%d, lat=%.2f vert=%.2f)",
                                 want, g_cfg.hmd_leash_lat * 100.0f, g_cfg.hmd_leash_vert * 100.0f);
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
                                         mode == 1 ? "weapon pose" : "aim ray");
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

        const bool held = (key_focus && (g_cfg.calib_key != 0) &&
                           ((GetAsyncKeyState(g_cfg.calib_key) & 0x8000) != 0)) ||
                          (menu_mode == 1 && !menu_lt);
        const bool was  = g_calib_held.exchange(held);
        if (held && !was) g_calib_start  = true;
        if (!held && was) g_calib_finish = true;

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
            if (halo::derive_ctrl_angles(&scy, &scp, cal_ridx)) {
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
        nav_world_tick(fixes_ok, tick);
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
            //
            // KNOWN LIMITATION: the shield shell is NOT carried in this mode -- it would need its
            // own UObjectHook attachment and a matching release, and attach_release/g_attached
            // track exactly one object. attach_mode ships at 0, so the shell follows on the path
            // that actually ships; anyone turning attach_mode on gets the old detached shield back
            // and should fix it here rather than assume it works.
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
            {
                // Mode 3 writes the WORLD rotation. The relative write is measurably discarded on
                // this mesh (Rig.cpp), and the location half is left relative because that half
                // demonstrably does take -- fixing only what is broken.
                const Quat q_world = quat_mul(q_parent, rotator_to_quat(rig_pitch, rig_yaw, c_roll));
                float wp = 0.0f, wy = 0.0f, wr = 0.0f;
                quat_to_rotator(q_world.x, q_world.y, q_world.z, q_world.w, &wp, &wy, &wr);
                const bool world_mode = (g_cfg.rig_mode == 3);
                for_each_rig([&](API::UObject* r) {
                    if (world_mode) rig_set_world_rotation(r, (double)wp, (double)wy, (double)wr);
                    else            rig_set_rotation(r, (double)rig_pitch, (double)rig_yaw, (double)c_roll);
                });
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
                    const Quat q_grip_new = quat_mul(quat_conj(quat_mul(q_frame_w, q_ctrl)),
                                                     g_calib_gun_world);
                    quat_to_rotator(q_grip_new.x, q_grip_new.y, q_grip_new.z, q_grip_new.w,
                                    &g_cfg.grip_deg, &g_cfg.grip_yaw, &g_cfg.grip_roll);

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
                    const Vec3 arm_frozen = quat_rotate(g_calib_gun_world, G_now);
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
                    write_calib_file();
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] CALIBRATED: grip=%.1f gripyaw=%.1f griproll=%.1f  mount=(%.1f,%.1f,%.1f)cm controller-local",
                        g_cfg.grip_deg, g_cfg.grip_yaw, g_cfg.grip_roll,
                        g_cfg.off_x, g_cfg.off_y, g_cfg.off_z);

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
                const Vec3 arm   = quat_rotate(q_gun, G);
                Vec3 off{pose_off.x + mount.x - arm.x,
                         pose_off.y + mount.y - arm.y,
                         pose_off.z + mount.z - arm.z};

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
                                    if (g_cfg.aim_reticule_trace) {
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
                g_rigw_off_valid = (g_cfg.rig_mode == 2 || g_cfg.rig_mode == 3);

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
                    if (rr_read != nullptr && asked > 1.0f) {
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

class HaloAimDriverPlugin : public uevr::Plugin {
public:
    void on_initialize() override {
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
        } else {
            strcpy_s(g_cfg_path, MAX_PATH, "halo_vr.cfg");
            strcpy_s(g_user_cfg_path, MAX_PATH, "halo_vr_user.cfg");
            strcpy_s(g_user_ref_path, MAX_PATH, "halo_vr_user_reference.txt");
            strcpy_s(g_dev_cfg_path, MAX_PATH, "halo_vr_dev.cfg");
            strcpy_s(g_calib_path, MAX_PATH, "halo_vr_calib.cfg");
            strcpy_s(g_data_dir, MAX_PATH, "data");
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
        if (config_file_has_uncommented_keys(g_dev_cfg_path)) {
            API::get()->log_info("[Halo-CampE-UEVR] DEV OVERRIDES ACTIVE: %s has uncommented keys "
                                 "(beats halo_vr_user.cfg on every ~2 s reload; the shipped file "
                                 "has none, and updates overwrite it)",
                                 g_dev_cfg_path);
        }
        if (config_file_has_uncommented_keys(g_user_cfg_path)) {
            API::get()->log_info("[Halo-CampE-UEVR] user overrides: %s applied (survives updates)",
                                 g_user_cfg_path);
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
    // So: release everything OURS at the first sign of shutdown, once, in a fixed order --
    // hook first (it is the one thing executing on another thread), then the writes we hold on
    // game objects, then our own components. Idempotent and safe to call from any thread.
    static void plugin_teardown(const char* why) {
        static std::atomic<bool> done{false};
        if (done.exchange(true)) return;
        g_shutting_down.store(true, std::memory_order_release);
        API::get()->log_info("[Halo-CampE-UEVR] TEARDOWN (%s): releasing hook, overrides and components", why);

        // 1. THE INLINE HOOK FIRST. blam_drive_tick() removes it when blam_angles is 0, and it
        //    is the only thing we own that runs on the sim thread.
        g_cfg.blam_angles = 0;
        blam_drive_tick();
        aim_watch_shutdown();

        // 2. Writes we hold on the GAME'S objects: the audio listener override, and the
        //    controller settings the mod changes transiently (restoring these was already
        //    written and simply never called).
        audio_fix_tick(false, 0, 0, 0, 0, 0);
        audio_comp_tick(false, 0, 0, 0, 0, 0, 0);
        game_settings_restore();

        // 3. Our own components: hand the game's crosshair back and park the markers.
        reticule_widget_release();
        navw_hide_all();

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

    void on_pre_engine_tick(API::UGameEngine* engine, float delta) override {
        if (g_shutting_down.load(std::memory_order_acquire)) return;
        // A null engine on the tick is the engine-loop teardown that never sends a window
        // message -- the in-game "Exit to Desktop" path. Treat it as the same signal.
        if (engine == nullptr) { plugin_teardown("engine null"); return; }
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

                        apply_render(rig);

                        // Same q_rel/loc are correct for the shell: same parent (asserted at
                        // acquisition) and the same pose -- it is posed identically to the arms by
                        // its own instance of the same anim blueprint, it only lacks our write.
                        // Bare pointer by design -- see g_shell_component; validating it here would
                        // cost an FName->string per frame on the render thread.
                        if (g_cfg.shell_drive) {
                            if (auto* sh = reinterpret_cast<API::UObject*>(g_shell_component.load())) {
                                apply_render(sh);
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
                const int n = g_navw_placed_n.load();
                for (int i = 0; i < n && i < 8; ++i) {
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
            } else if (!g_stick_mode.load()) {
                // Every remap below is an ON-FOOT convenience, so none of them run while seated:
                // the promise stick mode makes (and the README documents) is that the pad behaves
                // exactly as the game's own layout in a vehicle. That matters beyond tidiness --
                // this rebind consumes a button whose native action while seated may be the one
                // that gets you OUT of the vehicle.
                if (g_cfg.map_from != 0 && (state->Gamepad.wButtons & (WORD)g_cfg.map_from) != 0) {
                    state->Gamepad.wButtons &= (WORD)~g_cfg.map_from;
                    if (g_cfg.map_to != 0) state->Gamepad.wButtons |= (WORD)g_cfg.map_to;
                }

                // Injected AFTER the rebind, so this mask reaches the game untouched.
                // (Stick mode is already excluded by the branch condition above -- in a vehicle
                // the right stick is the camera, and looking down must not press crouch.)
                if (g_cfg.map_rstick_down != 0 && ry < -g_cfg.map_rstick_dz) {
                    state->Gamepad.wButtons |= (WORD)g_cfg.map_rstick_down;
                }
            }
        }

        // ---- WEAPON SCOPE TRIGGER. The toggle edge lives in Scope.cpp; eating LT here is what
        // keeps Blam's native zoom (viewmodel hide, zoomed look speed) from ever engaging under
        // the VR presentation. Menus and vehicle seats are excluded inside, so LT still means
        // whatever the game says it means there.
        if (scope_handle_lt(state->Gamepad.bLeftTrigger, g_in_menu.load(), g_stick_mode.load())) {
            state->Gamepad.bLeftTrigger = 0;
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
