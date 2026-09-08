// Live configuration and calibration persistence.
//
// THE DEFAULTS LIVE IN THIS STRUCT. Since 2026-08-11 no shipped file sets settings values: the
// compiled defaults below ARE the shipped configuration, so every field's default must equal
// what is meant to ship (release rule in CLAUDE.md). The files carry only overrides and
// calibration DATA. Read order every ~2 s (later wins; the parser is file-agnostic):
//   halo_vr.cfg      - SHIPPED CALIBRATION DATA ONLY (the Quest Touch fit + its schema stamps).
//                      Overwritten by every release. The stamps must stay file-borne -- see the
//                      calib_ver notes below for why they cannot move into compiled defaults.
//   halo_vr_user.cfg - the USER'S file, created on first run as a short pointer template. Never
//                      shipped, so it survives updates; deleting it reverts to the compiled
//                      defaults. The catalog of available keys is the shipped
//                      halo_vr_user_reference.txt (documentation only, never parsed) -- users
//                      copy lines across, so their file cannot go stale.
//   halo_vr_dev.cfg  - dev/troubleshooting catalog, SHIPPED all-commented (packaging asserts it
//                      is inert). Beats the user file on purpose: an uncommented key is a
//                      deliberate, temporary experiment; updates overwrite it.
//   halo_vr_calib.cfg - written by the in-game calibrations; applied LAST so it wins. Never
//                      shipped; survives updates.
//
// g_cfg is read from nearly every system in the plugin, so this header is the one most others
// include. Keep it free of UEVR API calls: the paths are filled in by the plugin at startup.

#pragma once

#include <Windows.h>
#include <cstdint>
#include <atomic>

namespace halo {
// ---------------------------------------------------------------- tunables
// Defaults are overridden at runtime by halo_vr.cfg next to the UEVR profile (see load_config
// below), which is re-read every ~2 s -- so a headset session can tune, or hit the kill switch,
// without a rebuild or a restart.
// One per-weapon adjustment. DELTAS on the calibrated base, not replacements -- see
// WeaponOffset.hpp for why.
struct WeaponAdjust {
    char  match[64] = "";      // substring of the weapon actor class, e.g. "AssaultRifle"
    float d_x = 0.0f, d_y = 0.0f, d_z = 0.0f;              // centimetres
    float d_grip = 0.0f, d_grip_yaw = 0.0f, d_grip_roll = 0.0f;  // degrees
};
constexpr int kMaxWeaponAdjust = 24;

// One per-weapon SCOPE trim. Deltas on the global scope fit, not replacements -- see
// ScopeOffset.hpp. d_zoom is a PLAIN MULTIPLIER: 1.5 = 1.5x the global magnification. ZERO means
// UNSET rather than 0x -- the parser is positional, so a line naming only a weapon leaves every
// field at 0, and nobody can want 0x zoom, so that spelling is free to mean "leave it alone".
struct ScopeAdjust {
    char  match[64] = "";        // substring of the weapon actor class, e.g. "FP_SniperRifle"
    float d_zoom  = 0.0f;        // plain multiplier; 0 = unset, 1.0 = unchanged
    float d_dist  = 0.0f, d_right = 0.0f, d_up = 0.0f;      // centimetres
    float d_rot_p = 0.0f, d_rot_y = 0.0f, d_rot_r = 0.0f;   // degrees
};
constexpr int kMaxScopeAdjust = 24;

// One per-weapon RIGID DELTA for the PALETTE weapon carry (src\palettearm\PaletteArm.cpp).
//
// WHY A SECOND TYPE AND NOT MORE WeaponAdjust FIELDS. WeaponAdjust adjusts the RIG path's fitted
// grip/mount -- degrees and centimetres added to grip_deg/off_x -- and the palette path consumes
// none of those. The palette carries the weapon branch by putting the AUTHORED marker node onto the
// controller, so what it needs is a rigid transform in the controller's own frame: a rotation and a
// translation, and no notion of a "calibrated base" to be a delta against. Sharing one struct would
// mean six fields that mean nothing on either half depending on which driver is running.
//
// ⚠️ THE FRAME IS OURS, NOT PR #1's, AND THE SHIPPED VALUES WERE CONVERTED INTO IT.
// blindcowboy24's wpnfix lines are a UE-convention quaternion (X forward / Y RIGHT / Z up) with
// UE-axis metres; ours are Blam axes (X forward / Y LEFT / Z up), the basis everything in
// src\palettearm\ works in. Relabelling Y is a REFLECTION, not a rotation, so it reverses the sense
// of a turn as well as flipping an axis: the conversion is q -> (-qx, qy, -qz, qw) and
// t -> (tx, -ty, tz). Copying his numbers across unconverted is a mirrored trim, and that is what
// kWeaponFixSchema and the per-file wpnfixver stamp exist to stop. See PaletteArm.cpp's weapon-fix
// block for the exact composition, and profile\halo_vr.cfg for the converted shipped values.
struct WeaponFix {
    char  match[64] = "";      // substring of the weapon actor class, e.g. "FP_AssaultRifle"
    float q[4] = {0.0f, 0.0f, 0.0f, 1.0f};   // x,y,z,w -- RIGHT-multiplied onto the trimmed pose
    float t[3] = {0.0f, 0.0f, 0.0f};         // METRES, in that same pose's frame
    // Did this entry come from the player's own capture file, or from the shipped baseline?
    //
    // Load-bearing, not bookkeeping. The table merges both tiers, and the capture writer rewrites
    // its file IN FULL -- so without this it would copy the shipped baseline into the player's file
    // on the first capture, where the copy would outlive the value it was copied from and quietly
    // defeat every future update to the baseline.
    bool  captured = false;
};

// Bumped whenever the meaning of a WeaponFix changes. Every file that carries wpnfix lines must
// stamp its own wpnfixver BEFORE them; lines under any other stamp are DROPPED AT PARSE, not
// believed. The failure a rigid delta from a foreign frame produces is a weapon hanging in the
// wrong place with nothing in the log -- and blindcowboy24's PR #1 writes wpnfix lines in a
// different frame with no stamp at all, so this is a file that exists rather than a hypothetical.
constexpr int kWeaponFixSchema = 1;

// Same idea, same reasoning, for the SUPPORT-HAND rigid fix (`handfix` in halo_vr_calib.cfg): the
// stamp is file-borne and a line under any other stamp is dropped at parse. One number, not two --
// the hand fix has a single writer and a single reader, so there is no second tier to version.
constexpr int kHandFixSchema = 1;

// ---- THE ONE PER-WEAPON LOOKUP -----------------------------------------------------------------
//
// PRECEDENCE: THE LAST MATCHING ENTRY WINS, because the config files are parsed weakest-first and
// this is the only rule that keeps the table's meaning the same as every other key's. In load order
// (kConfigFiles in Config.cpp):
//
//     halo_vr.cfg          SHIPPED per-weapon baseline        <- weakest
//     halo_vr_user.cfg     the player's hand-written overrides
//     halo_vr_dev.cfg      deliberate temporary experiments
//     halo_vr_calib.cfg    the global calibration gestures
//     halo_vr_weapons.cfg  THE IN-GAME CAPTURE                <- strongest, and user-owned
//
// So: a weapon the player has captured uses THEIR value; a weapon they have not uses the shipped
// one; a weapon in neither table gets no delta at all and lands exactly where the plain palette
// carry puts it. Deleting halo_vr_weapons.cfg therefore reverts every weapon to the shipped fit --
// the same "delete the file, get the shipped calibration back" contract as halo_vr_calib.cfg.
//
// ⚠️ NOTE THE INVERSION relative to wpnoff, which sits beside it in the same files: wpnoff takes the
// FIRST match, so its specific entries go ABOVE general ones. wpnfix takes the LAST, so within one
// file a specific entry goes BELOW. They differ because wpnoff has only ever had one tier.
//
// Takes the raw actor class name (e.g. "BP_FP_AssaultRifle_WeaponActor_C") so the per-tick caller
// can pass weapon_offset_current_class() -- a pointer read -- rather than paying for reflection.
// Returns nullptr when nothing matches.
const WeaponFix* weapon_fix_for(const char* class_name);

struct Config {
    bool  enabled      = true;
    bool  drive_pitch  = true;
    float floor        = 0.28f;   // deflection the game starts acting on (measured ~0.24)
    float full_deg     = 6.0f;    // error that saturates the stick (MCC-VR uses ~4.8)
    // Keep at 1.0. The stick curve is steeply exponential -- 0.70 -> ~39 deg/s but 0.90 ->
    // ~137 deg/s -- so the top of the range is where large corrections actually get closed;
    // capping it leaves the loop slew-limited on big movements only.
    float max_out      = 1.0f;
    float dead_deg     = 0.5f;    // angular deadband so it parks instead of hunting

    // ---- FEEDFORWARD + DAMPING ------------------------------------------------------------
    // A pure proportional loop against a RATE actuator has two unavoidable symptoms: tracking a
    // moving target needs a STANDING error to produce the matching rate (felt as the aim lagging
    // behind a fast turn), and when the hand stops, that accumulated error has to unwind (felt
    // as overshoot and rubberband for ~a quarter second).
    //
    // Feedforward fixes the cause rather than the symptom: if the target is moving at R deg/s and
    // the game turns at g_meas_rate deg/s per unit of deflection, then R/g_meas_rate is the
    // deflection that matches it outright -- no error required. Proportional is then left to
    // correct only the residual, so there is far less to unwind.
    //
    // Scaled by the MEASURED plant gain, not a constant, so it stays right across sensitivity
    // settings -- the same measurement the adaptive gain already maintains.
    float ff_gain      = 0.0f;    // 0 = off, the shipped state (direct-drive aim needs none);
                                  // raise toward 1.0-1.2 to re-enable for the steered fallback
    // Damps whatever the plant's own lag still overshoots by, opposing rate error rather than
    // position error so it does not fight the feedforward.
    float d_gain       = 0.40f;
    // The target rate is a difference of two noisy angles; unsmoothed it is jittery enough to buzz
    // the stick. Time constant in ms (see ema_alpha) -- LOWER is more responsive, higher calmer.
    float ff_smooth_ms = 16.0f;
    // Separate, heavier filter for the MEASURED aim rate that damping differentiates. Kept apart
    // from ff_smooth_ms on purpose: smoothing the feedforward costs responsiveness, smoothing the
    // damping input costs almost nothing and is where the buzz actually comes from.
    float d_smooth_ms  = 31.0f;

    // Cross-axis jitter guard. Yaw extracted from a controller quaternion gets noisier the further
    // it is from level, because the same angular error about the wrist maps to a larger yaw change
    // -- so a pitch sweep injects yaw noise (felt as side-to-side jitter while pitching the
    // weapon). Scaling the yaw feedforward down by cos(pitch) removes the part of that which is
    // purely a projection artefact, without touching genuine horizontal tracking.
    bool  ff_pitch_comp = true;

    // ---- DEADBAND HYSTERESIS ---------------------------------------------------------------
    // shape() is DISCONTINUOUS at the deadband edge: inside it returns 0, outside it returns at
    // least `floor` (~0.28), because anything smaller is below what the game acts on. So an error
    // hovering at the threshold slams the stick between 0 and 0.28 every time it crosses.
    //
    // That is a second source of cross-axis jitter: pitching drives err_pitch hard while err_yaw
    // sits near zero and chatters across its threshold, kicking the yaw stick sideways on each
    // crossing. Smoothing cannot touch it -- smoothing addresses noise; this is a switching
    // discontinuity.
    //
    // Once moving, keep moving until the error falls well INSIDE the band. Standard Schmitt trigger.
    float dead_hyst    = 0.5f;    // re-park threshold as a fraction of dead_deg; 1.0 = no hysteresis
    // Sightline length. STORED in metres (VR pose space); the `xdist` CONFIG KEY IS CENTIMETRES,
    // so the shipped value is 1000, not 10.
    float xdist_m      = 10.0f;
    // Set when ANY distance key arrived as a pre-2026-08-12 METRES value and was scaled up (see
    // cm_to_m in Config.cpp). The key name is a string literal, so there is no lifetime question.
    // Reported once by the tick, because this file deliberately makes no UEVR API calls.
    const char* legacy_units_key = nullptr;
    float       legacy_units_val = 0.0f;

    // AIM SIGHTLINE ORIGIN. Aim is the ray from this origin THROUGH the point the gun points at,
    // so BOTH controller translation and rotation feed the result -- sliding the gun sideways with
    // identical rotation changes aim, which is what makes it read as a held object rather than a
    // pure orientation sensor. UEVR: "construct a sightline from the standing origin ... so the
    // camera will be facing a more correct direction rather than the raw controller rotation."
    //
    //   1 = STANDING ORIGIN (UEVR's choice, default). Aim depends ONLY on the gun.
    //   0 = HMD position (MCC-VR's choice, because Halo spawns shots at the camera).
    //
    // Head-origin has a real drawback: moving your HEAD while the gun is still changes your aim
    // -- head/aim coupling, in miniature.
    int   aim_origin   = 1;

    // ---- AIM CONVERGENCE (6DoF) --------------------------------------------------------------
    // Aim at the point the sightline TRACE hits, instead of at an assumed range. Full derivation in
    // AimConverge.hpp; the short version is that a shot leaving the game camera and a sightline
    // leaving your eye are parallel but DISPLACED, so a commanded direction alone can only be
    // correct at one range -- xdist's -- and is wrong everywhere else by the eye-to-camera offset.
    // Measuring the range instead makes the shot land on what the reticule is over, at any range.
    //
    // Costs nothing when the head is leashed: the offset is then ~0 and the correction declines to
    // act at all (aim_converge_min_cm). This exists FOR hmdleash=0, where the offset is unbounded.
    // Needs aimreticuletrace, which supplies the measured range.
    bool  aim_converge = true;
    // Smoothing on the measured range. The correction goes as 1/range, so a depth discontinuity --
    // sweeping off a near crate onto a far wall -- is a step change in aim. This turns the step into
    // a short glide. Too long reads as aim lagging the hand when you change targets; too short and
    // the sweep pops.
    float aim_converge_tau_ms = 120.0f;
    // Eye-to-camera divergence, in cm, below which the correction does not engage AT ALL. Keeps the
    // leashed default numerically identical rather than merely nearly so.
    float aim_converge_min_cm = 3.0f;
    // Sanity rail on the correction, degrees. Anything past this is a bad range or a bad offset, not
    // geometry -- and a confidently wrong large correction is a weapon that fires sideways.
    float aim_converge_max_deg = 30.0f;
    // AIMCONV diagnostic: log the offset, the range and the resulting bend every N ticks. 0 = off.
    // Dev builds only.
    int   aim_conv_log = 0;

    // WHICH POSE THE AIM DIRECTION COMES FROM.
    //   0 = the OpenXR AIM pose (default, and what every calibration to date was made against).
    //   1 = the GRIP pose -- ROLL-INVARIANT.
    //
    // The aim pose is rigidly attached to the controller with its axis tilted well off the handle.
    // Rolling your WRIST rotates about the handle, so the aim vector sweeps a cone about that axis:
    // displacement is 2*asin(sin a * sin(t/2)), which for a ~35 deg tilt and a 90 deg roll is ~48
    // deg. The yaw extraction then amplifies it asymmetrically, because atan2(fwd.x,-fwd.z) grows
    // steeply as pitch steepens and the horizontal projection shrinks -- so rolling left and right
    // do not cost the same amount.
    //
    // The grip's forward IS the handle axis, so rolling about it moves nothing. The constant
    // difference between "where the handle points" and "where you feel you point" is exactly what
    // the Page Down calibration stores, so there is no tilt constant to tune: set 1, run Page Down
    // once, done. Measure first with aimrolllog -- if BOTH aim and grip yaw move with roll, the
    // coupling is upstream of this choice and switching sources will not help.
    int   aim_src = 0;

    // AIMROLL diagnostic: log the roll->aim coupling every N calls, 0 = off. Dev builds only.
    int   aim_roll_log = 0;

    // ---- HMD TRANSLATION LEASH ---------------------------------------------------------------
    // Bound how far the player's HEAD may get from the standing origin, by sliding the standing
    // origin to absorb the excess.
    //
    // THE PROBLEM. The game camera does not follow your physical head, so every centimetre you step
    // is a centimetre of divergence between where you are and where the simulation thinks you are,
    // and everything anchored to the standing origin inherits it. The aim sightline above is the
    // visible one: with aim_origin 1 it builds a ray from the standing origin through a point
    // xdist_m ahead of your hand, so translating by d rotates that ray by atan(d / xdist_m) with no
    // rotation on your part. 30 cm at 10 m is 1.7 degrees, and it persists until you reset the play
    // area -- which is exactly how this was found.
    //
    // WHY A LEASH AND NOT A LOCK. Pinning the view outright removes the divergence and causes
    // nausea: the inner ear reports motion the eyes do not. A leash keeps 1:1 roomscale inside a
    // radius, where the error is small and the comfort is real, and resists only past it. Pushing
    // the boundary slides the world at your own walking speed, which is self-caused and reads like
    // leaning on a wall rather than like being moved.
    //
    // It also BOUNDS THE AIM ERROR as a side effect: drift can no longer exceed the radius, so the
    // sightline swing is capped at atan(lat / xdist_m) -- 0.9 degrees at the defaults, against
    // unbounded today.
    //
    // Separate radii because the axes are not the same problem: lateral drift is what breaks aim,
    // while vertical is mostly crouching, which players do deliberately and often.
    //
    // DEFAULTS: ON, with lat = 0 -- lateral translation fully negated, no leeway.
    //
    // The received wisdom is that pinning the view causes nausea, and the first version of this
    // defaulted off for that reason. It did not hold here, and the reason is worth writing down:
    // only TRANSLATION is cancelled. Head ROTATION is untouched, which is where the great majority
    // of vestibular conflict lives, and lateral head movement in a seated shooter is small and
    // incidental rather than deliberate locomotion. Judged in-headset as feeling good, and better
    // than the alternative -- paired with the traced reticule it is what keeps aim honest, since
    // every centimetre of eye-to-camera divergence is an aiming error.
    //
    // VERTICAL IS ALSO 0. The obvious objection is crouching -- physically ducking no longer lowers
    // your view, because the origin follows your head down. But crouching in this game is a BUTTON,
    // and it moves the simulated player, which is the thing that actually matters for cover and for
    // where you are shot from. Physically ducking only ever moved the camera, never the player, so
    // what is lost is a decoupled view that was lying to you about your own position anyway.
    // Judged in-headset; raise it if you want the lean back.
    //
    // TURNING IT OFF (hmdleash=0) is a supported comfort option, for players who find any resistance
    // to head translation nauseating -- and it is genuinely off, which it was not before 2026-08-12:
    // UEVR's own VR_RoomscaleMovement is a zero-radius lateral leash and defaults ON, so the setting
    // has to take that over too (it does; see the LEASH block in Plugin.cpp).
    //
    // What an unleashed head changes, both handled rather than documented-around:
    //   * The STANDING ORIGIN stops being a body reference. Anything that treated it as "where the
    //     player is" is reading a stale point -- the arm rig (riganchor) and the aim sightline
    //     (aim_origin) both did. Neither breaks now, but new code must not assume it either.
    //   * Eye-to-camera divergence is unbounded, so the shot origin and your eye can be metres
    //     apart. This is why the reticule TRACES: a marker on the surface reads correctly from any
    //     eye position, where a fixed-distance one does not.
    //
    // ⚠️ ALL DISTANCES IN THIS FILE'S CONFIG KEYS ARE CENTIMETRES, matching Unreal. The FIELDS below
    // are stored in metres because every consumer works in VR pose space; the conversion happens once
    // at parse. A metres/cm mismatch here already cost a test -- hmdleashlat=50 was entered meaning
    // 50 cm, was obeyed as 50 METRES, and the setting looked broken while working perfectly.
    bool  hmd_leash      = true;
    float hmd_leash_lat  = 0.0f;    // stored m; CONFIG KEY IS CM. 0 = fully negated
    float hmd_leash_vert = 0.0f;    // stored m; CONFIG KEY IS CM. 0 = fully negated
    // WHICH HAND AIMS. false = right (default), true = left.
    //
    // The aim maths is hand-agnostic -- it turns a controller pose into angles -- so this only
    // changes which tracked device is sampled. What is NOT symmetric is CALIBRATION: grip roll/yaw
    // and the X offset are mirrored between hands, so each hand gets its own calibration file and
    // the left one seeds itself by mirroring the right (see g_calib_path / mirror_calib_for_left).
    //
    // NOT mirrored, and cannot be: the game's first-person arms and weapons are authored
    // right-handed. Left-handed aim puts the weapon in your left hand still looking like a
    // right-handed weapon. A plugin cannot mirror a skeletal mesh.
    bool  aim_left_hand = false;
    float yaw_sign     = 1.0f;
    float pitch_sign   = 1.0f;
    bool  fake_pad     = true;    // report a connected pad even if none is enumerated

    // VIEW LOCK. On this title ControlRotation IS the Blam camera, so steering aim would yaw the
    // whole VR world -- aiming would drag your head. UEVR's own controller-aim path solves this
    // by doing BOTH halves every frame:
    //     euler = ...                                    // point the game's view along the controller
    //     vr->set_rotation_offset(inverse(flatten(...))) // and cancel that yaw in VR space
    // With this on, the yaw the game has gained since the reference is cancelled in VR space, so
    // the game's aim moves and the headset view stays put. `flatten` in UEVR terms = yaw only,
    // which is why pitch does not have this problem.
    //
    // Consequence to expect: waving the controller no longer turns you. Turning the body is then
    // snap/smooth turn's job, which is how UEVR controller-aim titles already play.
    bool  view_lock    = true;
    float lock_sign    = -1.0f;   // legacy key: parsed but unused (the view lock assigns directly)

    // TURNING. With the view locked, waving the controller no longer turns the player, so a turn
    // control is mandatory rather than a nicety. Input is the player's own right-stick X, sampled
    // in the XInput hook before the aim value overwrites it.
    //   turn_mode 1 = SNAP  (discrete steps, lowest nausea)
    //   turn_mode 2 = SMOOTH (continuous, deg/sec)
    //   turn_mode 0 = off
    int   turn_mode    = 1;
    float snap_deg     = 45.0f;   // per flick
    float smooth_dps   = 90.0f;   // degrees per second at full deflection
    float turn_dz      = 0.30f;   // stick deflection required to register

    // STICK MODE. In vehicle seats the game binds the chase camera to the aim vector, so motion
    // aim swings the whole camera with the hand -- and the view lock half-fights it (yaw pinned
    // while the camera ORBITS, pitch not pinned at all). The playable answer there is the same
    // one every gamepad player gets: right stick = camera/aim, view rides the game camera.
    //
    // While stick mode is engaged the whole motion stack stands down -- aim law, view lock,
    // snap/smooth turn, movement rotation, rig driver, right-stick remaps -- and the sticks reach
    // the game untouched. Every lever degrades toward "stock game in stereo", never toward broken
    // aim, the same fail-closed direction as the ControlRotation validation.
    //
    // DETECTION is "the game is not rendering a first-person weapon for the pawn the rig was
    // resolved under": the FP weapon-actor route dies (seated, cutscene, dead, post-load) or the
    // local pawn stops being the one the rig belongs to (possession swap). Both are exact
    // object-identity checks; neither calls into Blam objects. g_rig_component itself CANNOT be
    // the signal -- it is a raw cached pointer that stays stale-non-null across a vehicle entry
    // (nothing clears it short of a PlayerController change).
    bool  stick_mode   = true;
    // 0 = auto-detect; 1 = always stick mode; 2 = never. 1/2 bypass the detector outright -- the
    // A/B lever, and the manual fallback if the detector misses on some vehicle. Live-reloaded.
    int   stick_force  = 0;
    // Debounce, seconds. Enter used to need 3 s because a weapon swap also kills the route for up
    // to the resolve cadence; the death-edge resolve BURST (see the detector) re-finds a swapped
    // weapon within a couple hundred ms, so a sub-second debounce is safe. Exit is near-instant:
    // the dismount watcher revives the route within a tick or two of the weapon re-attaching, and
    // the debounce is just a two-tick confirm. Both user-tuned in the field 2026-08-01.
    float stick_on_s   = 0.75f;
    float stick_off_s  = 0.05f;
    // ON FOOT WITH NO WEAPON -- the exception the detector above needs.
    //
    // "No first-person weapon" is four different situations wearing one costume: a vehicle seat, a
    // cutscene, death, and STANDING ON YOUR FEET HOLDING NOTHING. Only the first three want the
    // game's camera back. The campaign opens unarmed, so without this the first minutes of the game
    // play as flat gamepad -- motion aim and snap turn dead -- which is exactly where new players
    // got stuck and concluded the mod was broken.
    //
    // With this on, stick mode is SUPPRESSED while the game says it is presenting first person
    // (BlamPawn's own CurrentBlamCameraPerspective, learned rather than assumed -- see
    // fp_presentation_state in Rig.cpp) and the arms rig is live on the local pawn. Every term
    // fails closed: an unreadable perspective, or no rig, leaves the pre-existing weapon-route
    // behaviour exactly as it was.
    //
    // Set to 0 to restore the old behaviour outright -- the A/B lever if a vehicle or a scripted
    // sequence turns out to read as first person and keeps motion aim engaged where it should not.
    bool  stick_onfoot = true;

    // HIDE THE ARMS WHILE UNARMED.
    //
    // The game poses the empty first-person arms in a T-POSE -- it never expects you to look at
    // them, because on a flat screen holding nothing means the viewmodel is simply absent. In VR
    // they are right there, and once the rig follows your hand (which it now does while unarmed)
    // a T-pose swings around with it. Hiding them is strictly better than showing that.
    //
    // This is a STOPGAP with a known end date: it exists because there is no player arm IK yet. The
    // moment real hands are driven from the controllers this should become "show the VR hands",
    // not "show nothing" -- the reason to hide is the T-pose, not a preference for empty air.
    //
    // Scoped to the same on-foot-unarmed state the stick-mode exception uses, NOT to "no weapon":
    // that keeps it away from vehicles and cutscenes, where the game owns first-person presentation
    // and hiding things it chose to draw would be a regression rather than a fix.
    bool  hide_arms    = true;

    // ---- ARMS / WEAPON VISIBILITY PREFERENCES. Purely visual; aim and shots are unaffected.
    // show_arms=0 hides the first-person arms (and the shield shell with them) at all times --
    // the attached weapon is re-shown after the propagated hide, so "just the floating gun" is
    // exactly what you get. show_weapon=0 hides the weapon actor as well, via
    // SetActorHiddenInGame -- an independent flag from the bVisible tree, so the two
    // preferences compose instead of undoing each other.
    bool  show_arms   = true;
    bool  show_weapon = true;

    // ---- VEHICLE HARD BRAKE. Hold EITHER controller grip while stick mode is engaged. Distinct
    // from the quick-turn handbrake, which is the game's own left-trigger hold. Grips are read
    // from UEVR's ACTION state -- they do not exist in the XInput mapping at all, so no gameplay
    // binding changes meaning.
    //
    // DELIVERY, field-tested 2026-08-01: synthesized KEYBOARD does not reach this game's driving
    // input. Grip detection fired on every squeeze (80 BRAKE edges logged) and the scancode Ctrl
    // did nothing -- gameplay reads the device layer, which does not surface injected keyboard,
    // exactly like the aim finding. The pad path through our own XInput hook is the proven lane,
    // so that is the default now:
    //   brake_mode 3 = OR a pad button mask (brake_mask) into the state while gripping. DEFAULT:
    //                  mask 0x1000 = A, which IS this game's vehicle hard brake (field-confirmed
    //                  2026-08-01 -- on foot A is jump, but the brake only fires in stick mode)
    //   brake_mode 2 = LEFT STICK FULL BACK while gripping -- delivery-proven, but this game
    //                  reads it as brake-then-REVERSE, not the hard brake
    //   brake_mode 1 = the keyboard attempt (kept for reference / other titles)
    //   brake_mode 0 = off (same as brake=0)
    bool  brake_enabled = true;
    int   brake_mode    = 3;
    int   brake_mask    = 0x1000;   // XInput A -- this game's vehicle hard brake
    // Windows virtual-key held while gripping in mode 1. 0xA2 = Left Ctrl.
    int   brake_key     = 0xA2;

    // ---- CUTSCENE FLAT VIEW. This game's cutscene cameras render a stereo pair that does not
    // fuse (each eye gets a mismatched image), so while a cutscene plays the mod flattens the
    // view and restores it when gameplay returns.
    //
    // MODES -- what "flatten" means. The movies are PRE-RENDERED MP4s (Content\Movies\
    // CinematicsPreRenders) drawn by the engine's NATIVE fullscreen movie player, outside the
    // per-eye 3D render -- which is why they double in stereo and why no in-world fix exists.
    //   1 = UEVR's 2D SCREEN MODE (VR_2DScreenMode). THE DEFAULT -- the community-preferred
    //       behaviour. CAVEAT, field-tested: over SteamVR's OpenXR runtime the screen never
    //       composites in-headset (SteamVR drops UEVR's eye-visibility quad layers; they DO
    //       show behind the open dashboard) -- there the desktop mirror is the way to watch,
    //       and the cutscene hint overlay (below) says so in-headset.
    //   2 = MONO COLLAPSE. VR_WorldScale drops to its 0.01 floor for the scene, killing eye
    //       separation. Field-tested: fuses the live 3D WORLD but not the movie -- the movie's
    //       per-eye difference is not translational -- so the scene stays doubled and the
    //       mono world shows through the fades. Kept as a diagnostic lever.
    //   0 = off; cutscenes play as the game presents them (doubled).
    //
    // DETECTION, measured on this game: the camera manager's ViewTarget is a CineCameraActor
    // subclass (BP_dUniCineCam_*) -- sometimes for the whole scene, sometimes only its first
    // seconds (4 s of a 2-minute opener), so the camera class marks the START but cannot be
    // relied on to span it. The weapon-route death that drives stick mode DOES span it: the
    // first-person weapon stays unrendered for the whole scene and revives about a second after
    // it ends. So the flatten ENGAGES on a cinematic-camera sighting while stick mode holds,
    // HOLDS through menus (pausing drops stick mode, but the cutscene resumes on unpause), and
    // RELEASES when the weapon returns -- or immediately on a brake grip when no sighting is
    // recent, the escape for a cutscene that ends by seating you in a vehicle (the route never
    // revives there, and the grip says "I'm driving, give me the world back").
    // DEFAULT 0 pending one headset verification: mode 1 oscillated on cutscene exit (engage and
    // release disagreed at the scene boundary; each flip reallocates the view target, which is
    // ugly and uncomfortable to look at). Root-caused and fixed 2026-08-05 -- both edges now come
    // from one signal, engage is rate-limited, and a breaker latches the feature off if
    // transitions ever pile up -- but the fix has not been watched in a headset yet, and this is
    // a comfort-critical control. See docs\CUTSCENE_FINDINGS.md (private tree).
    int   cutscene_2d   = 0;

    // ---- CUTSCENE HINT OVERLAY. While the flatten is engaged, a SteamVR OVERLAY shows a
    // shipped PNG a little below where the cutscene screen sits, reading in effect "if nothing
    // shows here, watch the desktop mirror". A SteamVR overlay is composited by SteamVR itself,
    // above every app layer -- it displays even in the exact state where the 2D screen does not
    // -- and it never appears in the desktop mirror. Fail-open by construction: no SteamVR, no
    // openvr_api.dll, or any init failure just means no hint. On runtimes where the screen
    // works, the hint sits harmlessly below it (set cuthint=0 if you'd rather not see it).
    bool  cut_hint      = true;
    float cut_hint_dist = 1.8f;   // stored m; CONFIG KEY IS CM (180) -- ahead of the standing origin
    float cut_hint_drop = 0.75f;  // stored m; CONFIG KEY IS CM (75) -- below eye level
    float cut_hint_w    = 1.1f;   // stored m; CONFIG KEY IS CM (110) -- overlay width

    // WEAPON RIG -- drives the first-person rig from the controller so the gun follows the hand.
    bool  rig_enabled  = true;
    bool  rig_loc      = true;    // drive translation as well as rotation
    // ---- HOLD TRIM. How the weapon sits IN THE HAND: a fixed rotation and offset from the
    // controller, in the CONTROLLER's own frame, so it stays put at any orientation.
    //
    // 59 deg of pitch is the measured angle between the controller's forward axis and the way a
    // pistol grip actually points. The other five start at zero because they are per-person and
    // per-controller -- there is no defensible default, only a dial.
    float grip_deg     = 59.0f;   // pitch: muzzle up (+) / down (-)
    float grip_yaw     = 0.0f;    // yaw:   muzzle left/right
    float grip_roll    = 0.0f;    // roll:  cants the weapon in the hand

    // Constant placement offset, cm, solved by calibration. Applied in the same frame as the
    // pose-driven offset, BEFORE the parent-frame division.
    float off_x        = 0.0f;
    float off_y        = 0.0f;
    float off_z        = 0.0f;

    // PIVOT: the grip point in the rig component's LOCAL space, cm. The mesh turns about this
    // point instead of about the component origin (the arm root). Auto-read from the
    // `PrimaryWeapon` socket when pivauto=1; these are the manual override / the last value read.
    float piv_x        = 0.0f;
    float piv_y        = 0.0f;
    float piv_z        = 0.0f;
    bool  piv_auto     = true;
    // Per-tick weapon-to-rig separation logging. OFF by default: two reflected calls per tick.
    bool  wpn_diag     = false;
    // Store a per-weapon ROLL delta. ON: weapon models really are authored at different rolls,
    // and the Magnum needs several degrees the assault rifle does not.
    //
    // This was briefly defaulted OFF on the reasoning that the pose-match gesture cannot measure
    // roll -- a gun is near-symmetric about its own barrel, so overlaying a frozen one shows you
    // little. That reasoning was built on a bad statistic: the "8 deg of scatter" was measured
    // across captures taken against DIFFERENT BASES, and a delta is by definition relative to its
    // base, so those numbers were never comparable. Within a single session the spread is about
    // +/-2 deg, and a real per-weapon difference of ~7 deg sits well clear of it.
    //
    // The gesture IS the noisiest on this axis though, so a single capture can land a couple of
    // degrees out. Roll is easy to judge on a gun in your hand and hard to judge by overlay, so
    // prefer nudging it live (scripts/vr.ps1 wroll <weapon> <deg>) over re-capturing and hoping.
    bool  wpn_roll     = true;
    // Does the END calibration write griproll? NO.
    //
    // Roll is the one axis the pose-match gesture cannot see -- a gun is near-symmetric about its
    // own barrel -- so what END records is wherever the wrist happened to be. It is also global on
    // this title and best set by eye down the iron sights, which takes a couple of minutes and is
    // then correct for every weapon. Letting a gesture overwrite that is losing a good value to a
    // bad measurement; it happened twice in one session before this flag existed.
    // END still fits position, pitch and yaw, which it measures well.
    // DEFAULT 1 = END fits roll, the long-standing behaviour. Set 0 to hold roll across the
    // gesture; the solve does that as a rotation about the forward axis, not a scalar swap.
    bool  calib_roll   = true;
    // STATIC ROLL, in degrees. -999 = off.
    //
    // Lives in halo_vr.cfg, which the gestures never write -- halo_vr_calib.cfg is the machine
    // owned one. Re-applied every tick after the base and the per-weapon delta, so it is the last
    // word: END and HOME may still record whatever they like and it simply does not matter.
    //
    // Roll is the one axis the pose-match cannot measure (a gun is near-symmetric about its
    // barrel) and it is global on this title, so pinning it to a value set by eye down the sights
    // is strictly better than re-deriving it from a gesture that cannot see it.
    float roll_static  = -999.0f;
    // Log where the spawned hand/magazine components actually are in world space, once a second.
    bool  hands_diag   = false;
    // Cancel the socket's ROTATION as well as its offset.
    //
    // The weapon is socketed onto the mesh, so what you see is mesh_rotation * socket_rotation.
    // We drive the first term; the second is the engine's and differs between the two mesh
    // instances this title loads -- measured identity on one, (p3.7 y-4.2 r-1.8) on the other,
    // with the pitch matching the observed per-session grip shift to a tenth of a degree.
    // rigsocket already cancels the socket's translation, which is why position stopped drifting
    // while orientation kept flipping. This is the other half.
    // SHIPS ON (2026-08-23), together with rig_socket below -- they are two halves of one fix and
    // splitting them was an error on our side, not his design.
    bool  rig_sock_rot = true;
    // Cancel the MEASURED weapon-to-component separation instead of the pinned piv_* constant.
    // The weapon is socketed onto the rig mesh, so that separation is the only term standing
    // between placing the MESH and placing the WEAPON -- and piv_* is a guess at it that a
    // respawn invalidates. Falls back to piv_* whenever no weapon is in hand.
    //
    // SHIPS ON. The pinned piv_* is only pinned to YOUR machine after a calibration
    // (g_pivot_from_calib); before that it is whatever the SHIPPED halo_vr.cfg carries -- one
    // contributor machine, measured once, applied to everyone. Reading the live component fits
    // the instance actually in front of the player, with no calibration required. Measured cost
    // 0.004 ms mean against a 0.24 ms tick (~1.6%), which is the price of not shipping a guess.
    bool  rig_socket   = true;

    // WHICH socket is the pivot. You rotate your controller about your WRIST, so the pivot wants to
    // be the in-game hand -- which is not necessarily where the weapon mounts. `PrimaryWeapon` is
    // the weapon's attach point; `Wrist_R` is the arm socket nearer the hand. Both exist on this
    // rig, they are centimetres apart, and which one feels right is a question about the art, not
    // one this code can answer -- so it is a dial, and both are logged on acquisition.
    // ⚠️ EMPTY ON PURPOSE -- the real default ("PrimaryWeapon") lives at the consumer in
    // Plugin.cpp's pivot derivation, which already fell back on empty. That fallback existed
    // because this field was OBSERVED arriving empty with "cause not established" -- the cause
    // is now established: MSVC's constant-initialization of the global g_cfg drops char-array
    // string defaults (see nav_class below for the measurement).
    char  piv_socket[64] = "";

    // Manual nudge ADDED to the auto-read pivot, cm, component-local. Lets the socket stay
    // auto-detected per weapon while still correcting a constant bias.
    float piv_adj_x    = 0.0f;
    float piv_adj_y    = 0.0f;
    float piv_adj_z    = 0.0f;

    // ---- PIVOT MARKER. pivviz=1 turns the weapon rig itself into a visible marker sitting exactly
    // ON the pivot, so it can be READ rather than inferred:
    //   * scale shrunk to `pivvizscale` so the mesh becomes a small blob instead of a full rig,
    //   * relative rotation zeroed so nothing is thrown off-centre by orientation,
    //   * mount offset and pivot arm both dropped, so the component ORIGIN lands on the pivot.
    // The blob is then literally at the point the weapon rotates about -- hold your hand still and
    // see where it sits relative to your hand.
    bool  piv_viz      = false;
    float piv_viz_scale = 0.05f;

    // pivdraw: radius in cm of a debug sphere drawn AT THE PIVOT, leaving the arms and weapon
    // completely alone. 0 = off. Also draws a second, smaller sphere at the weapon's own origin so
    // the two can be compared directly.
    float piv_draw     = 0.0f;

    // pivcube=1 borrows a level StaticMeshActor, shrinks it, and parks it on the pivot every tick.
    // Works where DrawDebugSphere does not, because it is an object the renderer already draws.
    bool  piv_cube     = false;
    // TARGET SIZE IN CM (not a scale factor) -- the borrowed prop could be any size, so it is
    // measured and scaled to this.
    float piv_cube_scale = 6.0f;

    // ---- VR RETICULE on the aim ray.
    bool  aim_reticule      = true;
    // FALLBACK DISTANCE ONLY, once aim_reticule_trace is on (the default): the marker is placed on
    // the traced surface, and this applies solely when the resolve failed. A trace MISS uses
    // aim_reticule_max_dist instead, so that panning onto sky does not pop the reticule between two
    // depths. Still the whole story with tracing off.
    //
    // It also remains the reference distance the SEATED path scales against (see the seated
    // reticule block in Plugin.cpp), so changing it moves the vehicle reticule's size.
    float aim_reticule_dist = 500.0f;   // cm along the ray (5 m)

    // TRACE the reticule onto the surface instead of parking it at aim_reticule_dist.
    //
    // A world marker at a fixed distance only agrees with the impact point when viewed from the
    // origin the shot leaves. Your eye is not there and moves with your head, so it drifts -- see
    // HitTrace.hpp for the measured numbers. Tracing removes the free parameter entirely: the
    // marker sits on the surface, so it is correct from any eye position at any range, and
    // aim_reticule_dist stops mattering except as the fallback when nothing is hit.
    //
    // DEFAULT ON -- the shipping behaviour, proven in live sessions since 2026-08. It calls a
    // reflected engine function once per tick with a hand-built parameter block; every offset is
    // resolved from reflection rather than assumed, and perflog covers it if a cost is suspected.
    bool  aim_reticule_trace = true;
    // Max trace length in cm -- how far to look for a surface at all. Past this it is a miss.
    float aim_reticule_trace_max = 15000.0f;   // 150 m

    // ETraceTypeQuery index for the reticule trace.
    //   0 = TraceTypeQuery1, which is VISIBILITY by default -- and visibility is blocked by things
    //       a bullet ignores: invisible blocking volumes, trigger volumes, cull markers. The
    //       symptom is the reticule stopping in mid-air on nothing.
    //   1 = TraceTypeQuery2, CAMERA by default. Usually cleaner for this, since camera collision is
    //       authored to ignore clutter.
    //   2+ = whatever custom trace channels this project defines, in declaration order. A title
    //       like this very likely has a weapon/projectile channel, which would be the truthful one.
    // No value is right a priori and the mapping is per-project, so this is a live knob rather than
    // a guess baked into the code.
    int   aim_reticule_trace_channel = 1;   // Camera -- the shipping choice (see above)

    // CAP on how far the reticule is ever DRAWN, in cm. A marker on a hillside 80 m away is
    // technically correct and practically useless: it is small, it is washed out against the
    // distance, and it stops reading as your reticule. Past the cap the marker parks at the cap and
    // stays on the ray -- still the right DIRECTION, just at a readable depth.
    //
    // Capping is also what makes the surface offset below workable with a single fixed value: the
    // ring is scaled to hold its apparent size, so its WORLD size grows with distance, and bounding
    // the distance bounds the size it has to clear.
    //
    // A miss uses the cap too, rather than aim_reticule_dist -- otherwise panning from a distant
    // wall onto open sky pops the reticule between two depths for no reason the player can see.
    float aim_reticule_max_dist = 600.0f;   // 6 m, tuned in-headset

    // ---- DISTANCE SCALING (on foot) -----------------------------------------------------------
    // With tracing on, the reticule's distance is whatever the world is, from a wall at arm's reach
    // to a hillside at the cap. At a FIXED world size that reads as enormous up close and invisible
    // far away, because apparent size goes as 1/distance. Scaling the mesh in proportion to the
    // distance cancels that, so the ring subtends the same angle wherever it lands.
    //
    // TWO ANCHORS, interpolated linearly and clamped at both ends:
    //   min_scale at min_scale_dist  ..  max_scale at aim_reticule_max_dist
    //
    // max_scale is 1.0 because 1.0 is exactly what the on-foot reticule used to be pinned at, so
    // the ring at full distance is the size it has always been and only the near field changes.
    // An earlier version anchored solely on the near end and let the ramp run free upward, which
    // reached 7x at a 700 cm cap -- correct by its own formula and visibly absurd. Anchoring both
    // ends means neither can surprise you.
    //
    // The floor exists because a ring that keeps shrinking vanishes when you press the muzzle into
    // a wall; the ceiling is the old, known-good size.
    // aim_mesh_scale / aim_widget_scale still set the base size this multiplies.
    // min_scale 0.001 = effectively fade out entirely inside min_scale_dist (the parser's floor;
    // the old shipped cfg wrote 0 and the parse clamped it here, so this IS the shipped value).
    float aim_reticule_min_scale      = 0.001f;
    float aim_reticule_min_scale_dist = 70.0f;   // cm
    float aim_reticule_max_scale      = 1.0f;    // at aim_reticule_max_dist

    // Pull the reticule this many cm back along the ray from the surface it hit, so it sits just in
    // FRONT of the wall instead of intersecting it. Applied only when actually drawing at a hit:
    // if the surface is past aim_reticule_max_dist the marker parks at the cap exactly, not at
    // (cap - offset), because there is nothing there to clip into.
    float aim_reticule_surface_off = 50.0f;   // cm

    // Collapse any of the game's own flat crosshairs that are not the one we host. Hosting removes
    // a widget from its parent, which is not the same as stopping the HUD building another -- and a
    // mission transition rebuilds the HUD, leaving a screen-space crosshair pasted over the view
    // alongside the world-space one. In VR the flat one is never wanted.
    bool  aim_hide_native = true;

    // Which angles the reticule ray is built from.
    //   0 = the GAME's aim (ControlRotation). Truthful: it shows where shots actually go, including
    //       the control loop's residual. That residual is high-frequency jitter -- distracting on
    //       screen, and the reason this switch exists -- but it is also the only on-screen readout
    //       of loop error, so it stays the default and is what tuning runs must use.
    //   1 = the CONTROLLER's setpoint (g_desired_*). Smooth, because it is the intent rather than
    //       the achieved angle. Costs the error readout: if the loop is impaired, the reticule keeps
    //       looking perfect while the muzzle is elsewhere. Falls back to 0 whenever the aim law is
    //       not armed, so a stale setpoint is never drawn as if it were live.
    int   aim_reticule_src  = 0;

    // Divergence guard for src=1. If |setpoint - actual| exceeds div_deg continuously for div_ms,
    // the reticule snaps back to the true aim until the gap falls under half div_deg.
    //   div_deg 8: comfortably above the transient lag of a normal fast turn (the deadband alone is
    //     0.5 deg, and settling overshoot was measured in single digits), so ordinary play never
    //     trips it, while a loop that has actually stopped tracking diverges without bound.
    //   div_ms 600: "a moment" -- comfortably longer than a whip turn, whose error resolves within
    //     the loop's ~250 ms settle, so ordinary play never trips the guard; still short enough that
    //     a dead loop is caught before the player has fired more than a shot or two at the wrong
    //     place.
    float aim_reticule_div_deg = 8.0f;
    float aim_reticule_div_ms  = 600.0f;

    // First-order filter on the emitted reticule angles, as a TIME CONSTANT in ms (see ema_alpha).
    // 0 = off (raw). LOWER is snappier, higher is calmer -- the opposite sense to the old per-call
    // fraction this replaced, which is worth knowing if you are carrying tuning across.
    float aim_reticule_smooth_ms = 0.0f;   // raw: with direct-drive aim there is no residual to hide

    // Same filter, but for the CONTROLLER-sourced path (src=1). Separate because that path is
    // already responsive and only needs the small, fast jitter of hand tremor and tracking noise
    // taken off -- not the heavier filtering the loop's residual calls for. Also motion-gated, so a
    // deliberate swing is never filtered.
    float aim_reticule_smooth_ctrl_ms = 8.0f;

    // MOTION GATE for the filter above. Smoothing exists to hide jitter, and jitter only matters
    // when the hand is near-still; during a fast swing the filter buys nothing and costs visible
    // lag. So it disengages as the setpoint speeds up, on the rate the control law already measures:
    //   at or below smooth_slow_dps -> full smoothing (the configured time constant)
    //   at or above smooth_fast_dps -> none at all (raw, zero added lag)
    //   between                     -> linear blend, so there is no visible switch
    float aim_reticule_smooth_slow_dps = 20.0f;
    float aim_reticule_smooth_fast_dps = 120.0f;
    // Separate distance while seated. The SEATED path still uses a fixed distance -- tracing was
    // only wired into the on-foot path -- and too short a value parks the ring inside the vehicle's
    // own bodywork. Its APPARENT size does not change with this (the seated path compensates via
    // g_ret_scale_mul), so it is tuned purely for depth and occlusion: 22 m by in-headset
    // judgement. 0 = use aim_reticule_dist.
    //
    // Extending the trace to this path is the obvious follow-up: the same fixed-distance problem
    // that motivated it on foot applies here, and the 5 m / 22 m split exists only because no fixed
    // distance is right everywhere.
    float aim_reticule_dist_veh = 2200.0f;   // cm (22 m)
    // SEATED SIZE, as a fraction of the on-foot size. Distance compensation alone would render the
    // seated reticule at exactly the on-foot apparent size, but the right size is not the same in
    // both places: a vehicle wants a finer ring (more speed, more clutter, longer shots) than
    // infantry does. Kept as a RATIO rather than its own absolute scale so that retuning
    // aim_mesh_scale/aim_widget_scale still moves both together, and only the difference between
    // them lives here. 0.5 = half the on-foot apparent size, by in-headset judgement.
    float aim_reticule_scale_veh = 0.5f;
    float aim_reticule_cm   = 12.0f;     // borrowed-prop size at that distance

    // Where the reticule VISUAL comes from. NOTE: the shipping visual is neither of these -- it is
    // the mesh + widget reticule in Reticule.cpp (aimmesh / aimwidget), which needs nothing external.
    //   aimreticulelua  = publish the aim ray as a Lua event so an external uevrlib-based script can
    //                     draw it. No such script ships here, and it would additionally require
    //                     uevrlib to be installed; with nothing listening this is a harmless no-op.
    //   aimreticulecube = the borrowed level prop. Diagnostic only: it removes set dressing from
    //                     the map and looks like a misplaced object. Default OFF.
    bool  aim_reticule_lua  = false;   // no consumer ships; formatting the event is pure waste
    bool  aim_reticule_cube = false;

    // Bump this number (any change) to re-capture the neutral where your hand is RIGHT NOW.
    // A boolean cannot work here: the file is re-read every 2 s, so a sticky `recenter=1` would
    // re-zero forever and the gun would never move.
    float recenter     = 0.0f;

    // DEV ONLY (the ACTION is behind HALO_VR_DEV; this field and its parse are inert in a release
    // build). Edge-triggered like `recenter`: write any NEW number to force a full view-lock
    // re-prime, which re-adopts `locked` from the CURRENT camera yaw.
    //
    // Why it exists: mid-mission injection cannot be automated here -- reaching gameplay needs the
    // gamepad through MCP, which needs UEVR already injected, so the flow cannot inject after
    // gameplay starts. But what mid-mission injection actually DOES is prime `locked` from the
    // player's heading instead of the menu's 0. This reproduces that exact state from an ordinary
    // menu-injected session, which makes the calibration-survival property testable tonight.
    float lock_reprime = 0.0f;

    // XInput button mask that holds the pose-match calibration. Default 0x0020 = BACK / View /
    // the Menu-adjacent button, chosen because it is not a combat action -- holding it mid-fight
    // must not cost you anything. 0 disables calibration entirely.
    //   A=0x1000 B=0x2000 X=0x4000 Y=0x8000  LB=0x0100 RB=0x0200
    //   BACK=0x0020 START=0x0010  LTHUMB=0x0040 RTHUMB=0x0080
    // (`calibbtn` was here -- an XInput mask that ran the pose-match calibration. REMOVED entirely,
    // not defaulted off, and it should not be reintroduced.
    //
    // It defaulted to 0x0020 = BACK/View, picked because it is not a combat action. True, and beside
    // the point: on Touch controllers the MENU button commonly lands on BACK through UEVR, so
    // opening the pause menu ran a full calibration and wrote it to disk. Reported as "a
    // calibration file appeared and I never made one", and it silently moved the baseline under a
    // day of A/B tests.
    //
    // The general lesson, which is why this is a deletion rather than a default change: a gesture
    // that MUTATES PERSISTENT STATE must not be bound to an input the player presses for an
    // unrelated reason. The bug is not the mask value; it is the category of binding. A keyboard
    // key is the right shape -- distinct, deliberate, unreachable by a controller remap -- and the
    // eventual home is a UEVR Lua UI, which is deliberate by construction.)

    // Keyboard alternative, a Windows virtual-key code. Default 0x23 = END.
    // Either source works; whichever is held wins. A key is the more dependable of the two here,
    // because it does not depend on how UEVR happens to map your controller onto XInput -- a
    // binding that differs per controller and per profile.
    //   END=0x23  HOME=0x24  INSERT=0x2D  DELETE=0x2E  PGUP=0x21  PGDN=0x22
    //   F1..F12 = 0x70..0x7B                                            (0 disables)
    int   calib_key    = 0x23;

    // AIM calibration key. Default 0x22 = PAGE DOWN, next to END on purpose: two adjacent keys for
    // two adjacent-but-independent calibrations.
    int   aim_calib_key = 0x22;

    // (`bindcalsnap` was here and is gone. It solved the End bind from a pose captured at the
    // key-release edge, on the assumption that the solve read the controller a tick later. It does
    // not -- both run inside one update() call, and the BINDSNAP instrument measured the gap at
    // 0.00 cm on every calibration. The Page Down capture above is the genuine version of this,
    // because its consumer really is deferred. Do not add it back for End without a measurement.)

    // KILL SWITCH key, always used WITH Ctrl held. Toggles `enabled` on the press edge, so it works
    // with the headset on -- editing this file instead means finding a desktop, which is precisely
    // what you cannot do when aim is misbehaving.
    //
    // Default 0x24 = Ctrl+HOME. Deliberately NOT Ctrl+Esc: Windows owns that (Start menu), so it
    // would steal focus, and on OpenXR losing focus freezes controller poses -- the kill switch
    // would trigger the very failure it exists to rescue you from. Ctrl+Alt+Del and Ctrl+Shift+Esc
    // are reserved for the same reason. HOME sits in the same nav cluster as the calibration keys,
    // so it is findable by feel.                                          (0 disables the hotkey)
    int   kill_key = 0x24;

    // Ctrl+PAGE UP: swap aim actuation <-> direct drive, rig mode included. PAGE UP because it
    // completes the nav cluster the other three already use (HOME kill, END mesh calibrate,
    // PAGE DOWN aim calibrate), so all four are findable by feel in a headset.
    // The two modes are NOT independently selectable by this key on purpose: direct aim with the
    // actuation rig is the folded-pivot state, and there is no reason to be able to reach it.
    //                                                                    (0 disables the hotkey)
    int   mode_key = 0x21;

    // Scenario marker for scripted measurement runs. Dev-only in effect: the value is echoed into
    // the RIGTRACK line and logged when it changes, so a harness that poses the rig can label each
    // pose instead of correlating by wall clock -- which is what makes a transform comparison
    // across scenarios trustworthy rather than approximately aligned.
    int   dbg_mark = 0;

    // Persisted hand-to-aim mapping. What survives is the OFFSET (aim - controller), not the
    // absolute reference pair: the raw values are meaningless in a new level because the game's
    // yaw origin differs, whereas the offset is the actual calibration and is level-independent.
    float aim_off_yaw   = 0.0f;
    float aim_off_pitch = 0.0f;
    bool  aim_off_valid = false;

    // ---- THE CALIBRATION FRAME ---------------------------------------------------------------
    // Both persisted yaw calibrations -- `aimoffyaw` and `gripyaw` -- are measured against the yaw
    // the view lock pinned the world to (g_locked_view_yaw), because that is the constant relating
    // ROOM yaw to the yaw we render. Work the perceived angles through and the two conditions for
    // "it looks right" are:
    //     aim reticle sits where the hand points  ->  aimoffyaw == locked
    //     gun/arms sit where the hand is          ->  gripyaw   == locked
    // (g_turn_offset does NOT appear: `aimturn`/`rigturn` add it to both sides and it cancels.)
    //
    // `locked` is primed ONCE per session, from the game camera's yaw on the first stereo frame
    // after injection -- so it is a property of WHERE YOU INJECTED, not of the calibration:
    //     inject at the main menu -> the menu's camera. Measured here: 0.0, and reproducible,
    //                                which is why absolute values have worked for us all along.
    //     inject mid-mission      -> whatever direction the player happens to face. Arbitrary.
    // Users who inject mid-mission (some do -- injection is more reliable that way on their
    // hardware) therefore get a calibration wrong by exactly their heading at injection.
    //
    // With this ON, the calibration is stored MINUS `locked` and applied PLUS the live `locked`,
    // so it no longer encodes where you injected. Default OFF: our own `locked` measures 0.0, so
    // the two are numerically identical for menu injection, and OFF keeps a working population
    // byte-identical until the reproducibility of that 0.0 is confirmed across launches.
    // DEFAULT ON. The shipped gripyaw/aimoffyaw were calibrated at the MAIN MENU, where the frame
    // measures 0.0 (verified from a live session: `pinned=0.0` across a full menu-injected run).
    // At frame 0, "absolute" and "frame-relative" are THE SAME NUMBERS -- relative = absolute - 0 --
    // so the shipped defaults are already valid v2 values and need no recalibration to become
    // injection-point independent. Menu injection keeps adding 0 and is bit-for-bit unchanged;
    // mid-mission injection finally gets the frame it was always missing.
    bool  calib_relative = true;

    // Schema version of halo_vr_calib.cfg. 1 (or absent) = yaw values stored ABSOLUTE; 2 = stored
    // relative to `locked`. Recorded so a future change cannot silently reinterpret an old file --
    // the failure mode that would break every working user at once. v1 files are read as-is, which
    // is exactly correct for anything calibrated at the menu (where locked is 0).
    // ONE FLAG PER GESTURE, and this is not a nicety. The mesh calibration (hold + align) and the
    // aim calibration (Page Down) are deliberately SEPARATE gestures -- see the note at the
    // calibration hold about why two unrelated calibrations must not share one trigger. But
    // write_calib_file() persists BOTH sets whenever either one runs, so a single shared version
    // stamp lets the mesh gesture certify an aim offset that was never rebased. Observed exactly
    // that: gripyaw correctly stripped to ~0 while aimoffyaw kept its absolute value and then had
    // the frame added on top -- arms right, reticle wrong.
    // Default 2, for the reason above: ANY calibration performed at the main menu -- which is what
    // the README tells everyone to do, and what produced both the shipped defaults and every
    // existing halo_vr_calib.cfg in the wild -- was measured at frame 0 and is therefore already a
    // valid relative value. Only a calibration made MID-MISSION while `calibrelative` was off is
    // genuinely absolute-at-a-nonzero-frame, and that combination is broken today regardless.
    //
    // The two stay SEPARATE (do not merge them): the mesh gesture and the aim gesture rebase
    // different values, and write_calib_file() persists both, so a shared flag lets one gesture
    // certify the other's untouched value. That produced a real bug -- arms right, reticle wrong.
    // DEFAULT 1, NOT 2 -- "absent means absolute", which is what the note above already claimed and
    // what the code did not do. Nothing writes these except write_calib_file(), and that always
    // emits both stamps explicitly, so any file produced by a real calibration carries its own
    // version. The only values that arrive WITHOUT a stamp are hand-written or promoted into
    // halo_vr.cfg, and those are absolute by construction.
    //
    // Defaulting to 2 certified them as frame-relative anyway, so the live view-lock yaw was added
    // to an offset that never had it subtracted. That is a constant yaw error, it changes with
    // wherever you injected, and it then gets PERSISTED the next time any calibration writes the
    // file -- because write_calib_file() emits the whole struct, so the mesh gesture stamps the aim
    // block too. Which is precisely the "one gesture certifying the other's untouched value" the
    // note above warns about, arriving through the default rather than through the gesture.
    int   calib_ver     = 1;   // stamped 2 ONLY by the mesh calibration; gates gripyaw
    int   aim_calib_ver = 1;   // stamped 2 ONLY by the aim calibration;  gates aimoffyaw

    // ---- THE SUPPORT-HAND FIX (the "calibrate my off hand to my controller" gesture) ------------
    //
    // A rigid transform in the SUPPORT controller's OWN frame, applied to that controller's pose
    // before the palette arm driver builds a wrist target from it -- so it reads as "rotate/shift
    // the hand ON my controller", not "move it about the camera". Exactly the shape WeaponFix has
    // and for the same reason: the rotation right-multiplies, and the translation is carried by the
    // untrimmed basis, which is what makes the pair one rigid attachment rather than two knobs that
    // interact. src\palettearm\PaletteArm.cpp applies it and solves it.
    //
    // IDENTITY IS THE SHIPPED CONFIGURATION, and deliberately: unlike the weapon fit there is no
    // measured baseline to ship, because the quantity being calibrated is where a PERSON'S hand
    // sits inside their own controller. No shipped file sets these; a capture writes them to
    // halo_vr_calib.cfg, and deleting that file puts the hand back on the plain controller pose.
    //
    // WHICH HAND: the SUPPORT hand, i.e. the one that is not aiming -- the LEFT hand in the shipped
    // right-handed configuration, the right hand under aimhand=left. Not "the left controller":
    // the aim hand's placement is owned by the weapon calibration (the arm IKs to the controller
    // precisely because the gun is calibrated to it), so a second rigid trim there would fight it.
    //
    // METRES for the translation, matching WeaponFix and every other player-visible distance that
    // is not a cm-suffixed setting. The palette works in Blam units; the conversion is at use.
    float hand_fix_q[4]  = {0.0f, 0.0f, 0.0f, 1.0f};   // x,y,z,w -- identity
    float hand_fix_t[3]  = {0.0f, 0.0f, 0.0f};         // metres, in the support controller's frame
    // FALSE = "no capture exists", which is NOT the same as an identity transform that was
    // measured. write_calib_file() emits the block only when this is set, so a player who has never
    // run the gesture keeps a calib file with no handfix line in it -- and the menu can tell the
    // two apart well enough to show (or hide) its reset button.
    bool  hand_fix_valid = false;
    // cm of rig movement per metre of hand movement.
    //
    // MUST INCLUDE UEVR's WORLD SCALE. 100 is only correct at world scale 1.0; the accompanying
    // profile runs VR_WorldScale=1.312, so the game renders a metre of real movement as 131.2 cm
    // and a plain 100 under-translates the weapon by ~24%. That reads as BOTH "translation feels
    // slow" AND "the pivot is off" -- under-translating during a wrist roll moves the apparent
    // centre of rotation, which is very easy to mistake for a pivot bug.
    float rig_scale    = 131.2f;

    // Measure the game's turn rate at runtime and rescale the loop gain, instead of depending on
    // the player leaving controller sensitivity at the value this was hand-calibrated against.
    bool  gain_adapt   = true;

    // One-shot dump of MeteoriteHudVisibility's properties, to find the per-element HUD switches.
    bool  hud_dump     = false;

    // ---- HUD RETICLE FOLLOW --------------------------------------------------------------
    // Drive the game's OWN first-person reticle to the true aim point by writing its UMG render
    // translation, instead of drawing a second reticule in the world. This keeps the hit marker,
    // the weapon-specific art and the reload/scope states, all of which are children of that same
    // widget and would have to be re-implemented otherwise.
    //
    // Prerequisite (verified): SetRenderTranslation on this widget MOVES IT ON SCREEN. That
    // matters because on this title a property read-back alone proves nothing.
    //
    // OFF: superseded by the world-space reticule (aim_widget / aim_mesh), which has no angular
    // ceiling. Kept as a fallback lane.
    bool  hud_follow   = false;

    // Pixels per unit of tan(angle): px = hud_k * tan(offset). Equivalent to (canvasWidth/2) /
    // tan(hFOV/2), but neither of those is knowable from here -- the UMG canvas size and the FOV
    // UEVR reprojects the UI quad at are both unpublished -- so it is a tunable instead of a
    // derivation. Config is re-read live, so this can be dialled in without a rebuild.
    float hud_k        = 1000.0f;

    // Stop the reticle being flung off the canvas when the aim is far off view (a snap turn, or a
    // frame during a level transition where one of the two angles is stale).
    float hud_max      = 700.0f;

    // FVector2D is double under UE5 LWC, which is what this writes. If a build ever reports the
    // reticle jumping to a nonsense position, the type is the first suspect: set hudfloat=1.
    bool  hud_float    = false;

    // Degrees subtracted from the measured offset before it becomes pixels. A constant frame bias
    // between the Blam aim pitch and the composed view pitch looks identical to genuinely looking
    // down at the weapon; these exist so the difference can be trimmed out by observation instead
    // of settled by argument.
    // Use engine projection for the flat-HUD reticle offset instead of the k*tan(angle) constant.
    bool  hud_project    = true;
    float hud_trim_yaw   = 0.0f;
    float hud_trim_pitch = 0.0f;

    // ---- VIEW-CONSUMER FIXES (docs\CAMERA_CONSUMERS_FINDINGS.md) ---------------------------
    // Aim steers the game camera; the view lock cancels that for the EYES only. The audio
    // listener and the screen-space navpoint projection still read the game camera, so both are
    // wrong by the aim-vs-view delta on foot -- and RIGHT in stick mode, where the lock stands
    // down. Both fixes therefore engage only while the lock is actively cancelling, and RELEASE
    // (one-shot) on stick mode, frontend, kill switch, or the key going to 0.

    // Drive the audio listener to the rendered view via SetAudioListenerOverride (reflection).
    // SETTLED 2026-08-12: resolves and applies on this build, but does NOT move what Wwise
    // hears -- panning kept tracking the controller with it engaged (ear test). Kept because it
    // is harmless and may cover UE-side (non-Wwise) sounds; the real lane is audio_comp below.
    // ON as of 2026-08-16: this pair is the TESTED combination -- every session in which
    // positional audio was confirmed working in-headset ran with BOTH set. audio_fix alone was
    // measured NOT to move Wwise's spatialisation (audio_comp below is what does), so this one
    // is believed inert here; it ships on regardless because deviating from the configuration
    // that was actually validated is how "worked in testing" becomes "broken for players".
    bool  audio_fix   = true;

    // Drive the game's OWN listener -- the HaloAudioListenerComponent on BP_BlamCameraManager_C
    // (found by the audiodump survey) -- to the rendered view, world-write per tick, authored
    // relative transform restored on release. This is the lane the ear test points at.
    // ON as of 2026-08-16 -- THE positional-audio fix, player-confirmed in headset (controller
    // held still, head turning: panning follows the head). Releases cleanly in stick mode and
    // restores the listener's authored transform, so the failure mode is "audio as shipped".
    bool  audio_comp  = true;

    // Shift the navpoint container widget(s) by where the rendered view's forward lands in the
    // game's own projection -- first-order: exact at view centre, degrades off-centre, and
    // off-screen edge-clamped markers will still misbehave. The exact fix (feeding the
    // projection the rendered rotation at the source) is a recon item, not a tunable.
    bool  nav_fix     = false;

    // Correction source: 1 = project a view-forward probe through the game's own
    // ProjectWorldToScreen (no constant to calibrate; same lane as hudproject above); 0 = the
    // navk*tan(delta) fallback, which needs navk dialled in by observation.
    bool  nav_project = true;

    // Apply the shift at RENDER RATE from the stereo callback (yaw-only), with the tick probe
    // demoted to calibrating the pixels-per-tan constant. The tick-rate writer lagged the
    // per-frame delta and read as jitter (player-tested 2026-08-12); this is the fix. 0 = the
    // old tick-rate direct write, kept for A/B.
    bool  nav_render  = true;

    // WORLD-SPACE NAVPOINTS -- the lane with no angular ceiling (the flat quad spans only
    // ±20.7°×±12°, so no screen-space math can put a marker on an enemy 30° off-centre). One
    // mesh marker per live navpoint, placed along the true world direction recovered by
    // inverting the game's own projection. Supersedes the nav_fix shift while on (the flat
    // layer stays game-native). See the nav_world_tick banner in Plugin.cpp.
    // ON as of 2026-08-16 -- world-space waypoints ship enabled. Player-confirmed end to end:
    // markers sit at the objective's TRUE world position (read from the game's own navpoint
    // map, aim nowhere in the maths), wear the game's per-type art at per-kind sizes, draw
    // close enough to survive geometry, track smoothly, and hide the flat layer while they are
    // up. Every failure path hides the markers and restores the game's own HUD, so the worst
    // case is stock behaviour rather than a wrong waypoint.
    bool  nav_world       = true;
    float nav_world_dist  = 1500.0f;   // legacy fixed distance (lanes 0/1 only; lane 2 clamps)

    // ---- LANE 2 PLACEMENT: clamp + occlusion pull-in (the reticule's doctrine, applied to
    // waypoints). The objective's TRUE position is known now, but drawing the marker AT it is
    // not what a waypoint wants: a 300 m objective renders as a speck, and anything between
    // you and it hides the marker completely -- and a waypoint you cannot see through terrain
    // is not doing its job.
    //
    // So the marker is drawn along the true direction at min(true distance, nav_world_max),
    // and if the trace finds geometry nearer than that, it is pulled in front of the surface
    // by nav_world_surf. Angular size is held constant, so the player cannot tell where along
    // the ray it actually sits -- only the direction, which is exact.
    // 400 cm, not 2000. A single trace ray pulls the marker in front of what THAT RAY hits, but
    // the marker is a quad with width and the world is not: a warthog beside the ray, a rock lip
    // under it, terrain at the quad's edge all still occlude (field-reported: "doesn't block
    // properly on a good amount of things"). Drawing CLOSE is what makes occlusion a non-problem
    // -- at 4 m only geometry within 4 m can hide it, and constant angular size means the player
    // cannot tell the difference. The trace then handles just the wall-in-your-face case.
    float nav_world_max   = 400.0f;    // furthest the marker is ever drawn, cm
    bool  nav_world_trace = true;      // pull in front of intervening geometry
    // Stand-off from a traced surface: the LARGER of this and 20% of the hit distance. A fixed
    // 60 cm is not enough at short range, where the quad's own corners reach past it.
    float nav_world_surf  = 60.0f;
    // 0.5 -- PLAYER-TUNED IN HEADSET (2026-08-16), superseding the 0.35 that superseded the
    // original 0.12 (which made a ~0.7-degree dot and read as "no markers at all"). This is the
    // base; the per-kind multipliers below scale off it.
    float nav_world_scale = 0.5f;      // marker scale at 10 m, distance-normalised
    // WHITE, i.e. no tint of our own -- only the exposure gain. The hosted art is the GAME'S
    // navpoint icon and it already carries meaning in its colour (objective / hostile / ally);
    // a gold tint multiplied that to green in the field and threw the semantics away. These
    // remain as a knob for anyone who wants a deliberately different colour.
    float nav_world_cr    = 1.0f;
    float nav_world_cg    = 1.0f;
    float nav_world_cb    = 1.0f;
    int   nav_world_log   = 0;         // dev: per-marker inversion samples (2 = one-shot deep dump)

    // The widget class each marker hosts. Empty = the consumer's default
    // (WBP_NavpointObjective_C). The objective widget is authored for a HUD canvas -- it lays
    // out an icon PLUS a distance/label block, so hosted standalone it frames badly (field:
    // "still see the same text, cropping incorrect"). Live-tunable so the alternatives in
    // /Game/UI/Hud/Navpoints/ can be A/B'd without a rebuild; the resolved desired size is
    // logged at creation so the framing is measurable rather than guessed.
    //   candidates: WBP_NavpointWidgetPlayer_C, WBP_TrackedTargetNavpoint_C,
    //               WBP_NavpointWidgetBase_C, WBP_NavpointDestination_C
    char  nav_world_class[96] = "";
    // Draw size in pixels for the hosted quad; 0 = the consumer's default (128).
    // NOTE: sizing from the widget's own desired size does NOT work -- a widget that never
    // enters a viewport never lays out and reports (0,0) (measured 2026-08-16).
    float nav_world_draw  = 0.0f;

    // Re-derive each marker's transform in the STEREO CALLBACK from the current eye, instead of
    // leaving the tick's transform in place for the intervening frames. At a 4 m draw distance
    // the direction moves enough between 32 Hz ticks to read as jitter while walking. 0 restores
    // tick-rate placement, for A/B.
    bool  nav_world_render = true;

    // Hide the game's FLAT navpoint layer while world markers are actually being placed. Two
    // sets of waypoints for the same objectives is confusing, and the flat ones are the pair
    // that is wrong in VR. Tied to markers actually existing, so any failure of the world lane
    // restores the game's own markers rather than leaving the player with none.
    bool  nav_hide_flat    = true;

    // PER-KIND SIZE, as a multiplier of nav_world_scale. Kind is classified from the navpoint's
    // own widget class name (objective / enemy / item), so these follow the game's own typing
    // rather than anything we invent. Field-chosen values: an objective should read from across
    // the level; a last-known-enemy mark is situational; a floor weapon should not compete with
    // either. Anything unclassified keeps the base scale.
    // ALL FOUR PLAYER-TUNED IN HEADSET (2026-08-16) and canonical as of that session.
    float nav_size_obj     = 0.8f;
    // 0.17, not the 0.5 I guessed: a partner marker wants to be findable, not prominent -- it
    // is context, and at 0.5 it competed with the objective. Measured preference now, and the
    // one my reasoning ("navigational, so bigger than enemy") got badly wrong.
    float nav_size_ally    = 0.17f;
    float nav_size_enemy   = 0.25f;   // 0.3 read slightly large in headset; trimmed 2026-08-16
    float nav_size_item    = 0.2f;
    // Anything the classifier does not recognise. NOT 1.0: an unknown navpoint drawing larger
    // than every classified one is the wrong failure -- default it to the objective size and let
    // the log's [kind=other] tell us a class name we should be classifying.
    float nav_size_other   = 0.8f;

    // PER-CLASS SIZE OVERRIDE, "ClassNameSubstring:mult" comma-separated. Beats the kind
    // defaults above. Exists because the kind buckets are a GUESS at this game's taxonomy and
    // one was already wrong in the field (WBP_NavpointDestination_C filed as an objective at
    // x0.8 when it reads as an enemy mark) -- this addresses a class directly instead of
    // arguing about which bucket it belongs in. Live, like everything else.
    //   e.g. navsizeclass=WBP_NavpointDestination_C:0.3,Recon:0.4
    char  nav_size_class[192] = "";

    // Draw the marker this many cm SHORT of the thing it marks -- the reticule's surface offset,
    // applied to the objective. Only bites when the objective is nearer than nav_world_max.
    // 50 cm: PLAYER-TUNED IN HEADSET (2026-08-16). Enough that a marker on a close objective
    // floats in front of it rather than inside it.
    float nav_world_back   = 50.0f;

    // WHICH image child to host when a navpoint layout carries more than one (an on-screen icon
    // AND an off-screen arrow, say). 0 = the first found. The creation log prints how many were
    // found, so a wrong-looking icon is a one-line config change rather than a rebuild.
    int   nav_world_img    = 0;

    // Host only the navpoint widget's IMAGE child (the icon) rather than the whole HUD layout,
    // which carries a distance/label block and frames as cropped text on a square quad.
    // 0 hosts the whole widget, for comparison.
    bool  nav_world_icon  = true;

    // Image-child collection lane. 1 = verify-then-trust: the first host of each marker class
    // runs the object-array walk AND the reflection tree walk, serves the array result, and
    // certifies the tree lane only when both return the identical sequence -- after which that
    // class hosts via the tree walk (microseconds instead of a ~10 ms array pass, i.e. no
    // one-frame blip). 0 = array walk everywhere: the live A/B, and the drill that proves the
    // fallback lane still works.
    bool  navw_tree       = true;

    // Marker position source.
    //   2 (DEFAULT) = the OBJECTIVE'S TRUE WORLD POSITION, via the manager chain
    //       (NavpointInstances element -> +0x20/+0x28 -> FVector3f). Settled and live-verified
    //       2026-08-16: constant while the player moves, ~1.5 m from where the player stood at
    //       an objective they cannot approach closer than ~2 m. Aim is nowhere in the maths.
    //   1 = the map's SCREEN positions inverted through the measured projection (aim-poisoned;
    //       kept only for A/B).
    //   0 = the widget-tree screen-inversion fallback (also aim-poisoned).
    int   nav_world_src    = 2;
    // Map element stride, bytes. Raw-iterated (MapProperty's element type is unreachable from
    // the plugin API); every slot is validated, so a wrong stride skips rather than corrupts.
    // 0x78 was measured from the anatomy dumps (screen-pos signature at absolute 0x50 and 0xC8).
    int   nav_world_stride = 0x78;

    // BUG 2 (phantom markers): draw only navpoints the GAME's own HUD is showing. The compositor
    // lane resolves straight from the manager map and used to draw every entry with a valid world
    // position -- including ones the game had collapsed, which surfaced as objective/item markers
    // floating over walls while the live enemy marks were correct. This gate reads each entry's
    // live widget (element+0x08) and honours its reflected Slate Visibility; it FAILS OPEN, so it
    // can only ever suppress a marker it can positively prove the game hid.
    //   1 (default) = gate on reflected UWidget::Visibility (hide Collapsed/Hidden).
    //   0           = off: draw every resolved entry (the pre-fix behaviour, for A/B).
    int   nav_world_visgate  = 3;
    // STOPGAP kind filter, a fallback for the day the visibility flag cannot be resolved -- NOT
    // the primary gate. 0 (default) = draw all kinds. Otherwise a bitmask of NavwKind bits to
    // ALLOW: 1<<0 other, 1<<1 objective, 1<<2 ally, 1<<3 enemy, 1<<4 item. e.g. 0x0C = enemy+ally
    // only (drops objective + item, the two phantom-prone kinds), 0x08 = enemy only.
    int   nav_world_kindmask = 0;

    // DEV: the waypoint-position hunt (see nav_scan_tick). 1/2/3 = scan for the CURRENT view
    // position under a unit hypothesis (cm / Blam wu / m); 0 between steps re-arms the scanner.
    int   nav_scan         = 0;

    // Include the pitch term. Separately switchable because UEVR's DecoupledPitchUIAdjust
    // already moves the whole UI quad for pitch -- if the two double-count, markers overshoot
    // vertically and this is the knob that isolates it.
    bool  nav_pitch   = true;

    // Fallback pixels-per-tan constant (same meaning as hud_k) and the shift clamp.
    float nav_k       = 1000.0f;
    float nav_max     = 2000.0f;

    // Class-name substring the widget scan collects as the navpoint layer. A guess from the pak
    // inventory (/Game/UI/Hud/Navpoints/WBP_Navpoints) until a live session confirms; if the
    // scan logs "0 resolved", menudump the live widget names and correct this, not the code.
    //
    // ⚠️ EMPTY ON PURPOSE -- the real default lives at the consumer (the widget scan falls back
    // to "WBP_Navpoints" when this is empty), the aim_widget_class pattern. A string-literal
    // initializer here DOES NOT SHIP: MSVC's constant-initialization of the global g_cfg drops
    // char-array string defaults while keeping numeric ones (measured 2026-08-12 -- a host
    // printing the global saw '', the same struct on the stack saw the literal). piv_socket and
    // aim_mesh_path above have the same latent problem.
    char  nav_class[64] = "";

    // THE CULLING FIX (settled live 2026-08-12): re-apply
    // `Blam.Synchronization.Relevancy.OutOfViewCullDistance culldist` per level. The distance
    // term ALONE fixes aim-keyed body culling with no side effects; its visibility sibling is
    // deliberately untouched (zeroing it froze ambient animation and the FP arms -- it wakes as
    // well as culls). Default off until the perf soak; the cvar's stock value is unreadable on
    // this build, so cullfix=0 only stops re-applying and a restart restores stock.
    // SHIPPED ON (2026-08-15, user decision): aim-keyed body-culling is a shipped-game defect
    // in VR (actors vanish when the hand points away) and the distance override is the only
    // lever that cures it -- the head-keyed relevancy terms were REFUTED in testing (cullhead
    // below). 40000 cm (400 m) was chosen as the shipping value: far enough to cure the
    // observed culling, bounded enough to limit the perf cost of never distance-culling
    // out-of-view actors. Release notes must carry this (behaviour change, tunable via
    // cullfix/culldist). The wake side (creatures freezing when aimed away) is NOT fixed by
    // this and continues post-release -- the whole named cvar surface was exhausted without
    // effect (EffectThrottling.Enabled, DisableAnimationOnNoMotion, Throttling.Enabled).
    bool  cull_fix    = true;
    float cull_dist   = 40000.0f;

    // REFUTED 2026-08-15, kept for the record: the UE-side relevancy terms (ByUEVisibility,
    // CheckUELastRenderedTime) are registered cvars and the writes land, but restart-disciplined
    // in-headset testing showed they govern NEITHER the cull (bodies still culled with
    // cull_head alone from boot, baseline repro verified first) NOR the wake (birds aim-keyed
    // in both directions, proven by the displacement method). Do not re-enable expecting either
    // effect; the key stays only so a future build with different semantics can be probed
    // without a code change.
    bool  cull_head   = false;

    // DEV: one-shot navpoint/CHUD recon dump (edge-triggered on value change) -- the waypoint
    // lever-1 design input. Parsed always, acts only in HALO_VR_DEV builds.
    int   nav_dump    = 0;

    // DEV: one-shot Ak/Audio/Listener/Wwise object survey (edge-triggered on value change).
    // Parsed always, acts only in HALO_VR_DEV builds.
    int   audio_dump  = 0;

    // DEV: on-change console-command harness, devexec1..devexec4 -- the rig for testing whether
    // the Blam relevancy cvars accept writes (the culling hunt). Executed only in HALO_VR_DEV
    // builds; there is no un-exec, clearing a key just stops it being re-applied.
    char  dev_exec[4][192] = {};

    // ---- MENU INPUT ----------------------------------------------------------------------
    // Moving crouch to right-stick-down has a consequence that only shows up in menus: XInput B is
    // BOTH crouch in gameplay and "back" in every menu, so vacating the button that produced it
    // left menu-back reachable only by shoving the stick down. In a menu, reload means nothing, so
    // the physical right-controller B is free to be back -- which is also where the muscle memory
    // already is.
    //
    // Everything here is gated on menu state, so gameplay bindings are untouched.
    int   map_menu_back = 0x4000;   // right-controller B (measured) -> emits XInput B in menus

    // Suppress the gameplay-only remaps while a menu is up. Without this, pushing the right stick
    // down to navigate would fire "back", which is worse than the problem being fixed.
    bool  menu_suppress = true;

    // Pause menus keep the gameplay PlayerController and the rig, so the frontend test cannot see
    // them. This adds a widget-based signal: a candidate menu widget that reports IsInViewport().
    bool  menu_detect   = true;

    // One-shot discovery: log live widget classes whose names look menu-ish, so the match list
    // below can be corrected from observation rather than guessed at.
    bool  menu_dump     = false;

    // ---- WORLD-SPACE RETICULE (drawn by THIS plugin) --------------------------------------
    // Drawn from C++ rather than Lua because UEVR's Lua VM has NO usable diagnostic channel: no
    // `io` library (ScriptState.cpp:26), `print` goes to discarded stdout, and log_info is a
    // variadic C function pointer that sol2 cannot marshal, so calling it throws. Here, logging
    // works and failures are visible.
    //
    // Unlike the HUD widget, this has no angular ceiling: the flat HUD quad spans only +/-20.7 deg
    // horizontally and +/-12.0 deg vertically, while aim offsets can reach 27 deg.
    //
    // OFF: UE compiles DrawDebug* out of shipping builds, so on this game the call succeeds and
    // draws nothing. The mesh + widget reticule below is the one that renders.
    bool  aim_draw      = false;
    float aim_draw_r    = 5.0f;     // cm at the reticule's distance
    int   aim_draw_seg  = 16;
    // Must exceed the ~31 ms engine-tick interval or the sphere strobes; a little longer is
    // harmless (the overlap is two near-identical spheres) and much safer than a gap.
    float aim_draw_dur  = 0.05f;
    float aim_draw_th   = 3.0f;
    float aim_draw_cr   = 0.1f, aim_draw_cg = 1.0f, aim_draw_cb = 0.2f;   // green: reads on Halo's palette

    // ---- REAL COMPONENT RETICULE ----------------------------------------------------------
    // DrawDebugSphere renders NOTHING here: UE compiles DrawDebug* out of shipping builds, and
    // this is a shipping build -- the call succeeds, the coordinates are correct, and no pixels
    // are produced. So the reticule has to be an object the renderer already draws.
    //
    // This builds our OWN StaticMeshComponent via the plugin API's add_component_by_class, so it
    // borrows nothing from the level (the objection to the borrowed-prop marker) and lives in C++
    // where failures are loggable, unlike the Lua VM.
    // OFF in the shipping configuration: the hosted-widget crosshair (aim_widget) with its
    // gain/tint lift carries the reticule on its own, and the ring reads as clutter on top of
    // it. Kept one switch away for players who prefer a geometric marker.
    bool  aim_mesh      = false;
    float aim_mesh_scale = 0.14f;
    // Unlit pass-through: an unlit material keeps the reticule readable against dark geometry, and
    // is what uevrlib uses for the same purpose.
    bool  aim_mesh_unlit = true;
    // One-shot sweep for a COOKED GAME material carrying a colour parameter, so the mesh reticule
    // can be tinted. Engine materials are exhausted: BasicShapeMaterial has no colour parameter and
    // this build ships only a handful of others, none tintable.
    bool  mat_hunt      = false;
    // One-shot census of the LIVE shield primitives (Elite shield meshes, cover/portable shields).
    // The scope shows them as unshaded outlines and the leading explanation is that their look is
    // produced in POST -- which the engine forces off for every scene-colour capture. That stays a
    // theory until the primitives are read: bRenderCustomDepth set means a post-process pass keyed
    // on custom depth/stencil is drawing them, which would confirm it outright, and the material's
    // domain/blend mode says whether a capture-only stand-in could ever look right. Also reads the
    // two scene-capture visibility flags, because bHiddenInSceneCapture being set on shields would
    // be a far simpler explanation than any of this and has never been checked.
    bool  shield_census = false;
    // TEST MODE: park the mesh reticule on the VIEW axis instead of the aim axis, so it sits dead
    // centre of the frame no matter where the aim points. Purely for automated measurement: a
    // quad that has drifted out of frame produces all-zero readings, indistinguishable from a
    // real negative.
    bool  aim_park_view = false;

    // Comma-separated material object paths to parameter-dump once (names + values of their
    // Scalar/Vector/Texture parameter overrides). Ends guessing at parameter names.
    char  mat_dump[512] = "";

    // Use the hosted widget's RENDER TARGET as the mesh reticule's texture. The widget component's
    // own quad renders through Widget3DPassThrough, which outputs black in this game's cook -- but
    // the target it draws into may be fully coloured. Sampling that target through a material that
    // provably passes texture RGB shows the widget's TRUE pixels: per-weapon art, animation and
    // hit marker included, driven by the game itself.
    bool  aim_rt_from_widget = false;

    // One-shot sweep for materials that OVERRIDE a texture parameter (a usable texture slot with a
    // known name) and also expose a colour parameter -- the combination needed to display an
    // arbitrary texture, tinted, on a mesh.
    bool  tex_hunt = false;

    // Exact texture parameter name to bind the reticule texture under (from texhunt/matdump).
    // Empty = the shotgun candidate list.
    char  aim_tex_param[64] = "";

    // ---- RUNTIME-GENERATED RETICULE TEXTURE ------------------------------------------------
    // Halo's UI textures are alpha-only (art in alpha, RGB black), so Widget3DPassThrough's
    // colour = SlateUI.rgb * Tint can never produce colour from them. Rather than extract and
    // repack an asset, make our own texture at runtime: CreateRenderTarget2D + ClearRenderTarget2D
    // are BlueprintCallable, so the plugin can build a render target of a chosen colour and bind it
    // as SlateUI. Step one is a solid fill -- unambiguous proof the path works before attempting to
    // draw a crosshair shape into it with Canvas calls.
    bool  aim_rt        = false;
    int   aim_rt_size   = 128;

    // Mesh asset for the reticule. Empty = pick automatically. A TORUS is wanted so the centre is
    // see-through and the ring does not hide what you are shooting -- but /Engine/BasicShapes ships
    // only Cube/Cone/Cylinder/Plane/Sphere, so the asset has to be found rather than assumed.
    //
    // ⚠️ EMPTY ON PURPOSE -- the intended default (the game's SM_Torus_ThinDense_01) leads the
    // candidate list at the consumer in Reticule.cpp instead. A string literal here DOES NOT
    // SHIP: MSVC's constant-initialization of the global g_cfg drops char-array string defaults
    // (see nav_class below for the measurement) -- which means the shipped reticule had silently
    // been falling back to the engine torus the whole time this literal sat here.
    char  aim_mesh_path[192] = "";

    // Raise UEVR's motion-controls inactivity timer at startup. Below its expiry UEVR declares the
    // controllers unused and their poses freeze -- which reads as 3DoF while gameplay input keeps
    // working, because the game reads XInput while UEVR reads its own VR action state. Those are
    // separate paths, which is why no amount of in-game shooting refreshes it.
    // 100 is the slider's maximum (VR.hpp:1400, range 30..100); the profile default is the MINIMUM
    // of 30. 0 here = leave UEVR's setting alone.
    float vr_inactivity = 100.0f;

    // ---- RENDER-RATE AIM ------------------------------------------------------------------
    // 1 = compute the aim control law inside the XInput hook, at the rate the game consumes the
    // stick, from a fresh controller pose and a fresh ControlRotation read. The engine tick runs at
    // ~32 Hz on this title while input is consumed at 90+, so a tick-computed deflection is a
    // staircase held for ~31 ms -- a source of fast-motion jitter: by the time a step lands, the
    // error it was computed from is up to a frame-third stale.
    // 0 = the tick-rate path.
    bool  aim_rate_render = true;

    // ---- TEXTURED MESH RETICULE ------------------------------------------------------------
    // Slate loses colour offscreen on this title (even TEXT renders black), so the reticule
    // cannot come from a widget. Meshes are unaffected -- WorldGridMaterial and a plain sphere
    // both render in colour. What is needed is a material that carries an image AND is cooked
    // in this build.
    //
    // Widget3DPassThrough_Masked is exactly that: it samples a texture parameter named "SlateUI",
    // it is cooked (widget components build their own MID from it), and it renders on a primitive.
    // With SlateUI unset it renders BLACK -- not a broken material, but the material faithfully
    // sampling an unset texture. It must be given a real texture.
    bool  aim_tex       = true;                     // use the textured-quad reticule
    char  aim_tex_path[192] = "";                   // Texture2D object path; empty = probe candidates
    // Image FILE to load as the reticule texture (PNG with alpha), via
    // UKismetRenderingLibrary::ImportFileAsTexture2D. Takes precedence over aimtexpath. This is the
    // no-pak route to custom reticle art: the game's own UI textures carry their art in ALPHA with
    // black RGB, so tinting them can never produce colour -- an imported PNG can have white RGB.
    char  aim_tex_file[192] = "";
    // 0 = keep the mesh asset's own material (the FX torus ships with a game material that may
    // render colour where the widget pass-through family does not).
    bool  aim_mesh_override = true;
    // Which Widget3DPassThrough variant to build the MID from: masked | translucent | opaque.
    // Opaque ignores alpha entirely, which makes it the cleanest COLOUR test: if opaque shows the
    // tint and the others stay black, the blend path is what eats the colour.
    char  aim_mesh_parent[192] =             // holds either a keyword or a full object path
        "/Game/FX/Meshes/Debug/Materials/MI_Arrow.MI_Arrow";
    float aim_tex_rot_p = -90.0f, aim_tex_rot_y = 0.0f, aim_tex_rot_r = 0.0f;   // torus axis faces the viewer
    // Halo's own reticle blue, and the hit-marker red. Both live-tunable so they can be matched by
    // eye against the flat HUD rather than guessed from a screenshot.
    float aim_mesh_cr = 0.20f, aim_mesh_cg = 0.60f, aim_mesh_cb = 1.00f;

    // ---- HIDE THE FLAT CROSSHAIR ----------------------------------------------------------
    // NOT via UI.HudState.EngineHidingCrosshair. Those states are set BY the engine and only read by
    // the visibility testers, there is no reflection-reachable setter, the matching bVisibleCrosshair
    // is a non-reflected C++ member, and HideElement()'s element tags are not in the binary at all.
    // SetVisibility on the widget is the mechanism that is actually reachable, and it is
    // verified working on this exact widget.
    //
    // Default OFF: without a world-space reticule active, turning this on removes the only
    // reticule on screen. Live-reloaded, so it is one edit away with no rebuild.
    bool  hud_hide      = false;

    // ---- WIDGET RETICULE: the game's OWN crosshair, in world space --------------------------
    // A UWidgetComponent hosting the live WBP_FirstPersonReticle. The per-weapon art and the hit
    // marker are CHILDREN of that widget, so hosting the widget brings them along -- which is why
    // this beats drawing our own shape.
    //
    // WORLD space, not Screen. SetWidgetSpace(Screen) renders through the viewport, and UEVR maps
    // the viewport onto the flat HUD quad -- which would put us straight back inside the +/-20.7 deg
    // by +/-12.0 deg ceiling this whole approach exists to escape. World space has no such bound, at
    // the cost of having to face the widget at the view ourselves each tick.
    bool  aim_widget      = true;
    float aim_widget_draw = 256.0f;   // UMG draw size, square
    float aim_widget_scale = 0.24f;   // world scale of that quad
    // If the crosshair renders mirrored or edge-on, the widget plane's facing convention is the
    // suspect; flip adds 180 deg of yaw.
    bool  aim_widget_flip = false;
    // EWidgetBlendMode: 0 Opaque, 1 Masked (engine default -- eats thin crosshair lines), 2 Transparent.
    // NOTE: setting this on the CDO before construction does NOT propagate (verified by reading
    // the live component's byte at offset 1428). Kept for reference; the material swap below is
    // what actually controls blending.
    int   aim_widget_blend = 2;

    // Build our own translucent MID and bind the component's render target to it, instead of
    // relying on BlendMode. UWidgetComponent picks its material from BlendMode AT CONSTRUCTION and
    // offers no way to rebuild afterwards, so overriding the material outright is the only route
    // that does not depend on winning a race with the constructor.
    //
    // OFF in the shipping configuration: the stock pass-through material plus aim_widget_gain /
    // aim_widget_tint is what renders today; the MID swap is the fallback lane.
    bool  aim_widget_mat  = false;

    // DIAGNOSTIC. Paints the widget quad's background opaque red. One run then distinguishes the
    // three remaining possibilities, which no amount of further reasoning can separate:
    //   red square + crosshair -> fixed
    //   red square, no crosshair -> the quad renders; the WIDGET CONTENT is not reaching the target
    //   nothing at all -> the quad itself is not rendering (transform, scale or material)
    bool  aim_widget_bg   = false;   // diagnostic -- must never default on

    // Control test: host a NEWLY CREATED reticle instead of the HUD's live one. Loses the hit
    // marker, so it is a diagnostic only -- it isolates re-parenting from the component setup.
    // OFF = host the HUD's live reticle, hit marker included, which is the shipping behaviour.
    bool  aim_widget_fresh = false;

    // DIAGNOSTIC: host a DIFFERENT widget class by name substring. Separates two very different
    // causes of the black reticule that no amount of component tuning can tell apart:
    //   a plainly-coloured widget (e.g. the shield bar) renders in colour -> the framework is fine
    //   and only the reticle's own material fails offscreen;
    //   everything renders black -> Halo's UI cannot draw colour into an offscreen target at all.
    // Empty = the first-person reticle (normal operation).
    char  aim_widget_class[64] = "";

    // Brightness control for the hosted widget. UMG renders into the component's target with
    // PREMULTIPLIED alpha while the passthrough material samples it straight, which darkens exactly
    // the semi-transparent pixels a thin antialiased crosshair is mostly made of -- a plausible
    // cause of it reading as black. Tint multiplies the sampled colour, so values above 1 can lift
    // it back without touching the widget itself.
    // Shipped at the 1024 ceiling: gain and tint MULTIPLY, and this pair (tint 1024, gain 5) is
    // the in-headset compromise between bright-beach readability and blowing out in shade.
    float aim_widget_tint  = 1024.0f;   // applied to R,G,B
    float aim_widget_alpha = 1.0f;
    // EMISSIVE GAIN for the hosted crosshair. The stock Widget3D pass is unlit but its output is
    // still multiplied by the scene's PRE-EXPOSURE before tonemapping, so Halo's authored cyan
    // lands near black in bright scenes -- the long-standing "dark crosshair". This multiplies it
    // back up. Live-tunable: raise if the crosshair still reads dark outdoors, lower if it blows
    // out or looks washed. Forced to 1.0 automatically when the exposure-compensated VREditor
    // material is in use (that one preserves authored colour at unit tint).
    float aim_widget_gain  = 5.0f;

    // ---- COMPOSITOR RETICULE (XrLayer.cpp) ----------------------------------------------------
    //
    // The two settings above are a CONSTANT fighting a VARYING term, which is why the pair is a
    // "compromise" rather than a fit. An OpenXR quad layer is composited after the whole post
    // chain, so exposure and tonemapping never reach it -- immune rather than compensating.
    //
    // DEV/RESEARCH ONLY, DEFAULT OFF, and it belongs in halo_vr_dev.cfg rather than the player
    // catalog. Stage 1 draws a reticule WE generate: bright and exposure-proof, but STATIC -- no
    // firing bloom, no reload state, no hit marker, because that animation belongs to the game's
    // own widget and only stage 2 (presenting the widget's render target) brings it back. It draws
    // ALONGSIDE the in-scene reticule, never instead of it. Full doctrine in XrLayer.hpp.
    // CANONICAL AS OF 2026-08-23, after the feature was proven end to end in a headset: real Halo
    // reticule art, animating, bright, exposure-proof, tracking through rotation and roll, and
    // surviving level changes. The struct initialiser IS the configuration -- no shipped file sets
    // it -- so this line is the statement that the feature is on by default.
    //
    // HONEST CAVEAT: the current attachment resolves UEVR's statically-linked xrEndFrame out of
    // UEVRBackend.pdb, and only a UEVR checkout has that PDB. On a player install the resolve fails,
    // the module latches off with a log line, and the in-scene reticule is untouched -- so this
    // default is meaningful for DEV builds and inert for players until the API-layer attachment
    // exists. It is safe to ship on because it fails closed, not because it works there.
    bool  xr_layer = true;
    // SPACE LADDER -- a diagnostic, not a preference. Whether UEVR's get_pose() reports in the same
    // space get_stage_space() names is an assumption we have not measured, and each mode fails in a
    // different, recognisable way. 0 = stage space, raw HMD pose. 1 = stage space with the
    // recentre correction UEVR applies to its own quads. 2 = view space (head-locked; WRONG for a
    // reticule on purpose -- if 2 draws and 0/1 do not, the pose math is the only thing left).
    // Start at 2 when nothing appears at all. See build_pose() for what each answer means.
    int   xr_layer_space = 0;
    // RE-ANCHOR PLAYER-ATTACHED QUADS AT RENDER RATE. 1 = on (default).
    //
    // The aim reticule and the grab guide are not world-fixed: the reticule is origin + fwd*d
    // measured from the WEAPON, and the guide sits at your hand. Both are chosen on the 32 Hz
    // tick, then placed against the eye of the frame being drawn -- so while you locomote, a
    // fresh eye is differenced against a stale anchor and they trail a tick of travel behind.
    // With this on, the offset is captured at publish and the eye added back at render rate, so
    // both sides come from the same instant.
    //
    // THE TRADE, so it is a choice and not a surprise: with a traced reticule, the DISTANCE is
    // still a tick old. Walk straight at a near wall and the reticule sits a few cm off it until
    // the next tick, instead of lagging your aim by the same few cm. Set 0 for the old behaviour.
    int   xr_layer_head_rel = 1;
    // MULTIPLIER on the in-scene widget's own world size, not a length. 1.0 = exactly the size of
    // the hosted crosshair (aim_widget_draw x aim_widget_scale, with the same 1/distance apparent-size
    // compensation), so the two agree by construction and keep agreeing when either is retuned.
    //
    // It was an absolute 0.06 m and came out ~8x too small in headset -- 0.06 m is ~7.9 UE cm at this
    // world scale against the widget's 61.4. A number that has to be dialled in until two things look
    // alike silently stops matching the moment either side moves.
    float xr_layer_size = 1.0f;
    // UE centimetres per VR metre. 0 = derive from UEVR's VR_WorldScale.
    // RECON-NEEDED: the DIRECTION of the VR_WorldScale relationship is inferred, not measured, and
    // a wrong factor does not look broken -- it looks like a reticule at the wrong depth, which
    // reads as a tuning problem and sends the investigation elsewhere. Sweep this over a decade in
    // headset once, then write the answer into ue_cm_per_metre() as a measurement.
    float xr_layer_cm_per_m = 0.0f;
    float xr_layer_alpha = 1.0f;
    // Colour, 0..1 linear. Bright cyan by default -- near Halo's authored crosshair but pushed up,
    // since the entire point is that nothing downstream will dim it.
    float xr_layer_cr = 0.35f, xr_layer_cg = 0.95f, xr_layer_cb = 1.00f;
    // Hide the IN-SCENE crosshair so the compositor layer can be judged on its own.
    //
    // Applied only while xrlayer_live() is true -- the layer must be PROVEN reaching the compositor,
    // not merely enabled. If it is not, this does nothing and the in-scene crosshair stays, because
    // the failure mode of getting that wrong is a player with no reticule at all.
    //
    // The widget keeps ticking and keeps rendering to its target either way: the compositor layer is
    // PRESENTING that target, so stopping it would freeze the layer's texture. See
    // reticule_widget_set_scene_hidden().
    // 0 = off. 1 = ALPHA (component stays rendered, multiplies to zero pixels).
    // 2 = SCALE (component stays rendered at sub-pixel size).
    //
    // A MODE, not a bool, because the mechanism is genuinely undecided: hiding via
    // SetHiddenInGame froze the compositor layer's texture (measured twice in a headset -- the
    // widget stops redrawing once the engine stops rendering it), and TickWhenOffscreen is NOT the
    // gate (it has been set since creation). Both surviving mechanisms keep the component RENDERED
    // and differ in how they make it invisible, so they fail for different reasons -- keeping both
    // switchable live means one deploy can test both rather than one game restart each.
    // Canonical at 1 (ALPHA). Proven in a headset; mode 2 (scale) is kept only as a fallback that
    // fails differently. Gated on xrlayer_live() at the call site, so on a build where the layer
    // cannot attach this does nothing and the in-scene crosshair stays -- the player is never left
    // without a reticule.
    //
    // Worth recording WHY hiding the in-scene one matters beyond evaluation: the in-scene widget was
    // being driven at aim_widget_tint 1024 x aim_widget_gain 5 to out-shout the tonemapper, which
    // made the crosshair a light source -- it bloomed the environment and lit nearby surfaces. The
    // compositor layer is composited AFTER the tonemapper and cannot bloom by construction, so
    // hiding the in-scene copy removes artificial lighting that the brightness workaround had been
    // adding all along. Noticed in a headset the moment the in-scene one went away.
    // 1 = the reticule quad follows head ROLL (billboarded to the head, the original behaviour).
    // 0 = it stays WORLD-UPRIGHT: still faces you, but tilting your head no longer tilts the
    // crosshair. Costs nothing -- it reuses the per-slot orientation override built for the scope
    // pane, feeding it the view forward with world +Z as the up reference.
    //
    // Degenerate case, stated because it is visible rather than hidden: looking straight up or down
    // makes the view forward parallel to world up, the basis has no roll reference, and
    // xr_look_rotation REFUSES rather than emitting a sheared quad -- so the reticule falls back to
    // head-oriented at extreme pitch instead of flipping.
    // Trim on the AUTOMATIC zoom-fit that shrinks the world reticule so it looks right inside the
    // scope pane (mode 3 only -- see Reticule.cpp). 1.0 = take the computed factor as-is; raise or
    // lower to taste. The computed factor is logged whenever it changes, so this is a correction to
    // a stated number rather than a blind multiplier.
    float xr_layer_zoom_fit = 4.4f;  // CANONICALISED 2026-08-31 from the user live profile
    // SCOPE PANE RETICULE. Draw the reticule a second time, as a small quad sitting just in front
    // of the scope pane, so the zoom view has a crosshair.
    //
    // WHY A QUAD AND NOT A BLIT INTO THE PANE CELL: the compositor already alpha-blends layers in
    // submission order, so a quad submitted after the pane draws over it for free. Compositing into
    // the atlas cell instead would need a real blend -- CopyTextureRegion does not blend, it would
    // stamp an opaque square -- which means a shader, a root signature and a second GPU path in the
    // capture ring. This costs one extra XrCompositionLayerQuad per frame and NO new GPU work at
    // all: it reuses the reticule's existing atlas cell, which is already captured every frame.
    //
    // WHY IT IS CORRECT TO PUT IT AT THE PANE'S CENTRE: a scope's crosshair marks the optical axis,
    // and the pane is mounted from the aim ray, so centre is where the shot goes. That is the
    // conventional answer as well as the geometric one. If the capture is ever mounted off-axis
    // this becomes a confident lie and the size key is not the fix -- the mount is.
    //
    // DEFAULT OFF. It cannot regress anything while it is off, and it is new.
    int   xr_layer_scope_reticle = 1;  // CANONICALISED 2026-08-31 from the user live profile
    // Its size as a fraction of the pane quad's own size.
    float xr_layer_scope_reticle_size = 0.5f;
    // Where the reticule sits relative to the pane's surface, in CENTIMETRES (this depot's unit for
    // every length, matching Unreal). Positive is toward the viewer; NEGATIVE is behind the pane,
    // i.e. further away, which is the direction that simulates a collimated/holographic sight.
    //
    // It exists at all because the reticule and the pane are two separate compositor quads on the
    // same plane, so a nudge along the pane's normal is what stops them z-fighting. 1 cm was the
    // hardcoded value and is a floor rather than a preference: below roughly half of it the two
    // quads start to tear against each other at the edges.
    //
    // It is a real comfort control above that, not just a z-fight guard. The reticule sits at a
    // different stereo depth from the pane image, so this is the knob that decides whether the
    // crosshair reads as etched ON the glass or floating in front of it -- and that judgement is
    // per-eye and per-person, which is exactly why it cannot be a constant.
    float xr_layer_scope_reticle_depth = 0.2f;
    int   xr_layer_roll = 0;      // CANONICALISED 2026-08-31 from the user live profile
    // CANONICALISED 2026-09-01 from the user live profile, after the objection below was DISPROVED.
    //
    // The history matters, because this key was set to 3, reverted, and restored inside 48 hours and
    // the next person to see the symptom will be tempted to revert it again:
    //
    // Set to 3 on 2026-08-31 (the user runs 3 and asked for their live values to be canonical).
    // Reverted the same night on the theory that mode 3 caused a freeze in which the GUN, ARMS and
    // RETICLE all stop updating together while aim keeps working -- the reasoning being that mode 3
    // sets bVisibleInSceneCaptureOnly on a first-person component, and that hiding a SKELETAL MESH
    // stops OnlyTickPoseWhenRendered and freezes the socket the gun and arms hang off.
    //
    // THAT MECHANISM CANNOT OPERATE HERE, and it is refuted by construction rather than by testing.
    // Both callers of reticule_set_capture_only() pass g_ret_widget_comp -- the reticule's
    // UWidgetComponent. OnlyTickPoseWhenRendered and sockets are USkeletalMeshComponent concepts; a
    // widget component has no pose to tick and no socket to freeze. Mode 3 writes ONE bitfield bool
    // on the reticule widget and touches no part of the arm/weapon rig.
    //
    // The freeze had a different cause, since corroborated from two independent instruments: the
    // OpenXR runtime publishes EXACT (0,0,0) controller translation on focus loss, position_dead
    // arms after ~120 such ticks, and the render path in on_pre_calculate_stereo_view_offset was
    // re-writing the rig from a phantom offset. See the g_rigw_off_valid guard in Plugin.cpp.
    //
    // ** CORRECTED 2026-09-06: MODE 3 WORKS. The note that used to sit here was wrong, and it cost
    // a full session. **
    //
    // It read: "it does not actually hide the widget from the main view -- measured in a headset
    // with active=0 and both flags set. The flag does not govern a UWidgetComponent's main-view
    // draw at all. So mode 3 is INEFFECTIVE at its stated job." An agent read that, concluded the
    // behaviour the player wanted was unreachable through mode 3, and spent hours building the
    // COMPOSITOR scope pane as an alternative route to it. None of that was needed.
    //
    // REPRODUCED AND CONFIRMED IN A HEADSET, with the log preserved at
    // _Builds\_logs\log.ANOMALY-REPRODUCED-20260906-154556.txt and its exact cfg beside it:
    //   unscoped, main view ............ reticule NOT visible   (mode 3 hiding it)
    //   scoped, main view around scope . NOT visible
    //   scoped, through the scope ...... VISIBLE                (the SceneCapture sees it)
    //   compositor pane ................ 0 PRESENTING, 3 REFUSING -- NOT INVOLVED
    // That is mode 3's stated job, done.
    //
    // CONFIRMED A SECOND TIME, independently, by the scope-pane lane on 2026-09-06 after it restored
    // a re-assert host it had removed. Two headsets, two sessions, same result. Mode 3 works.
    //
    // AND THE THING THAT MAKES A NEGATIVE RESULT HERE UNTRUSTWORTHY: vsco is a single bool written
    // once. Anything that rebuilds or re-hosts the widget clears it, and it stays clear until
    // reticule_mode3_reassert() puts it back -- which needs BOTH of its hosts (the late one in
    // reticule_widget_move AND the early one in update()). Every "mode 3 does not hide it"
    // measurement so far was taken while that machinery was incomplete, and in each case the flag
    // READ BACK as true, which is what made the wrong conclusion so convincing.
    //
    // DO NOT USE THE HIDDEN/restored TRANSITION LOG AS A VISIBILITY PROXY. Measured across ten
    // sessions: the one the player confirmed as WORKING logged TWO restores, while three sessions
    // with hundreds of tick faults logged NONE. An earlier version of this note reasoned from
    // "17 transitions vs 5" and that inference does not hold.
    //
    // MODE 3 DEPENDS ON THE COMPOSITOR PANE BEING OFF. scopelayerhidepane hides the in-world pane
    // when the compositor pane presents -- and the in-world pane's SceneCapture is the very thing
    // showing the reticule in the scope. So making the compositor pane work (scopesrc=2) REMOVES
    // the reticule from the scope unless the compositor draws its own. These two features are
    // alternatives, not layers.
    // MODE 3 WORKS. The table above is correct, and this note exists because I briefly "measured"
    // otherwise and was wrong -- READ THIS BEFORE TRUSTING A NEGATIVE RESULT ABOUT MODE 3.
    //
    // On 2026-09-06 mode 3 was reported doubled in the main view with bVisibleInSceneCaptureOnly
    // confirmed true by readback. That looked like hard evidence the flag does nothing, and it was
    // not: the same session had just moved reticule_mode3_reassert() OFF reticule_widget_move() onto
    // an early host in update(). The early host is a real fix for a real hole (that function is
    // gated behind a successful aim pick, so failing ticks skipped the repair) -- but the LATE host
    // matters independently, because it re-applies the bit after the widget rebuilds its material
    // and transform, which happens later in the same tick. Re-asserting only early let the rebuild
    // win. Restoring BOTH hosts made the reticule vanish from the main view immediately, confirmed
    // in a headset.
    //
    // The lesson is the reusable part: a negative result about a FLAG is only as good as the
    // machinery that keeps the flag applied. Check the re-assert path before concluding a property
    // is inert -- and note that the transition log is NOT a visibility proxy (the compositor lane
    // measured a session the user called correct that logged two "restored" lines).
    //
    // MODE 4: mode 3 PLUS bOwnerNoSee, kept as an A/B lever rather than a fix. bOwnerNoSee is
    // evaluated against the view's ViewActor -- the pawn in the main view, unset for a
    // SceneCaptureComponent2D -- so in principle it also hides from the main view while the scope
    // capture still draws it. UNVERIFIED IN A HEADSET: mode 3 was fixed before mode 4 was ever
    // needed, so nothing here rests on it. It reads the bit back and degrades to mode 3's behaviour
    // if SetOwnerNoSee does not resolve, saying so rather than silently doing nothing.
    int   xr_layer_hide_ws = 1;

    // Hide the COMPOSITOR RETICULE while the scope pane is up. Both reticules are correct -- they
    // just sit at different ranges -- and two crosshairs a few degrees apart read as noise. The
    // scope pane carries its own reticule at the magnified range, so this one has nothing to add
    // while the scope is raised. Retires the quad outright rather than letting it fade out on the
    // retirement grace, which would leave it hanging for about a second each time you scope in.
    bool  xr_layer_hide_scope = true;
    // Mode 2 only: multiplier on the quad's world scale while hidden. Small enough to be invisible,
    // large enough not to be screen-size-culled -- if the art reappears frozen in mode 2, raise it.
    float xr_layer_hide_scale = 0.004f;
    bool  xr_layer_log = false;
    // STAGE 2 -- present the GAME'S OWN crosshair through the compositor layer instead of the ring
    // we generate. Both keys are DEV/RESEARCH, default off, and belong in halo_vr_dev.cfg.
    //
    // xr_layer_src_probe is a LADDER, not a bool, because the two halves have different risk:
    //   0  off.
    //   1  WALK AND LOG ONLY. Bounded pointer walk from the widget's render target, reporting every
    //      candidate offset triple it finds. Dereferences nothing it has not first classified as a
    //      heap object with a vtable in a mapped image. Cannot make an indirect call.
    //   2  the same walk, and then ATTEMPT UEVR's get_native_resource() on each candidate. That is a
    //      virtual call on a pointer we inferred, so it is a separate, deliberate step taken by an
    //      operator who has already read the mode-1 list. See XrSource.hpp.
    // The walk is compiled out of a shipping build entirely (HALO_VR_DEV).
    int   xr_layer_src_probe = 0;
    // PROVE THE CHAIN OFFSETS ARE CLASS-LEVEL -- opt-in, and OFF by the same argument the whole
    // dev-tooling split rests on. This is a one-shot proving harness: it walks a SECOND component's
    // render target with the full triple-nested discovery probe and compares the offsets, to show
    // that one measured chain may serve all nine slots. That walk is the expensive one (tens to
    // ~200 ms), and it used to run on EVERY tick until it reached a verdict -- measured live as
    // xrsource_tick 87-197 ms while markers were up, which in VR is nausea. HALO_VR_DEV alone did
    // not protect anyone because we PLAYTEST on dev builds, so it gets its own key: default 0, so it
    // never runs unless a developer explicitly asks, and throttled to once a second even then.
    bool  xr_layer_src_xcheck = false;
    // PER-TICK NATIVE-RESOLVE CACHE (default ON -- this is the perf fix, and it ships on).
    //
    // resolve_latched() re-validates the widget's chain every tick per fed slot. MEASURED (2026-08-24,
    // live): the cost is NOT get_native_resource/GetDesc (both ~0.00 ms) but the pointer WALK --
    // looks_like_object()'s VirtualQuery-based classification, ~42 VirtualQuery/slot, ~14.5 ms and
    // rising with the session's VAD tree (6 markers = ~100 ms xrsource_tick, a VR-nausea hitch). The
    // vtable classification only exists to make get_native_resource's virtual call safe, so on a CACHE
    // HIT (where the cached ID3D12Resource is re-used and no virtual call is made) it is skipped: the
    // hit path reaches the FRHITexture with readable_bytes guards only (~4 VirtualQuery) and re-uses
    // the cache while the chain still leads to the SAME FRHITexture reading back its exact draw size.
    // A miss (first resolve, re-host, FRHITexture pointer changed) takes the full classified walk.
    // Crash-fix invariant preserved: RT-identity + component-change guards run every tick and clear
    // the cache on any re-host, reads stay readable_bytes-guarded, the game thread issues no capture
    // during a level load, and only the vtable sweep (which guards a call the hit path skips) is
    // dropped. See the CACHE section in XrSource.hpp.
    //
    // DEV/RESEARCH toggle: set it to 0 to force the full classified walk EVERY tick (the pre-fix
    // behaviour) -- the safe fallback, and the A/B control the dev split instrument measures against.
    bool  xr_layer_src_cache = true;
    // Hand the resolved-and-validated ID3D12Resource to the layer. Does nothing until the chain has
    // been measured and re-validates; there is no compiled-in offset to fall back on, on purpose.
    // Canonical: present the game's OWN reticle render target rather than our generated ring. This
    // is what gives the layer real per-weapon art and live firing/reload animation.
    bool  xr_layer_src = true;
    // ms. HOW LONG THE LAYER KEEPS SHOWING THE LAST CAPTURED CROSSHAIR once the game thread stops
    // capturing, before it gives up and draws the generated ring instead.
    //
    // The capture stops for two very different reasons and this number has to separate them:
    //   - a TRANSIENT drop (a weapon pickup re-hosts the crosshair on a new render target; a
    //     game-thread hitch of 600-750 ms, which is what this title actually does) -- re-resolves
    //     within a few ticks, and showing a placeholder ring for it is strictly worse than showing
    //     the previous frame;
    //   - a LEVEL CHANGE -- seconds long, and a crosshair hanging frozen in space through it looks
    //     worse than the ring.
    // 1500 sits between the two with room on both sides. It was a hardcoded 250 ms, which is BELOW
    // this title's worst hitch, so every hitch flashed the ring. Lower it only to reproduce that.
    int   xr_layer_hold_ms = 1500;

    // ---- WORLD NAVPOINT MARKERS ON THE COMPOSITOR LAYER (xrlayernav) -----------------------
    //
    // The markers want this for two reasons the reticule already demonstrated, plus one it does
    // not have:
    //   1. A composition layer is submitted AFTER the whole post chain, so exposure and the
    //      tonemapper never touch it. The in-scene markers are driven at aim_widget_gain x
    //      aim_widget_tint for the same reason the crosshair was -- which made the crosshair a
    //      light source that bloomed the scene, measured in a headset once it was removed.
    //   2. A COMPOSITOR QUAD IS NEVER OCCLUDED. That is the entire job of the navpoint line trace
    //      in lane 2, so on this lane the trace can simply be skipped -- and it is skipped
    //      CONDITIONALLY, on xrlayer_live() and the slot actually having art, never deleted. The
    //      in-scene lane is still the shipping path (the compositor attachment needs a PDB no
    //      player has), and it must not be degraded to make this one look better.
    //   3. The RETICULE'S trace is NOT redundant in the same way and stays: it puts the reticle on
    //      the surface the shot will hit, which is real information rather than an occlusion
    //      workaround. The asymmetry is deliberate; do not "tidy" it.
    //
    // DEFAULT ON since 2026-08-30, by the user's decision after running it in a headset. It was
    // default-off "like every stage of the layer before it was proven", and it is now proven.
    //
    // WHAT THIS COSTS A PLAYER WHO CANNOT USE IT: nothing. The compositor attachment fails closed
    // -- the PDB tier needs a UEVRBackend.pdb no player install has, and the shipping route is the
    // OpenXR API layer, which a player has to register. When neither is present the layer never
    // arms, xrlayer_live() stays false, navw_layer_owns() is false, and the in-scene marker lane
    // runs exactly as it did before. So this default turns the feature ON for anyone who CAN run
    // it and is inert for everyone else, which is the only reason it is safe to flip.
    bool  xr_layer_nav = true;
    // How many marker quads may be submitted, 1..8. Does NOT change the atlas -- all eight cells
    // exist whenever xrlayernav is on -- so this is live-tunable without a swapchain rebuild.
    int   xr_layer_nav_max = 8;
    // cm. The FURTHEST a compositor marker is placed from the eye.
    //
    // The trace used to supply this number as a side effect of avoiding occlusion, and a quad still
    // needs a depth because depth is what sets VERGENCE -- the eyes physically converge on it. So
    // it becomes a deliberate choice rather than an inherited raycast result: the objective's true
    // distance, clamped into a comfort band (near clamp is fixed at 1 m, which is about as close as
    // a quad can sit without being uncomfortable to fuse). Angular size is held constant across the
    // clamp exactly as the in-scene lane does, so this changes where the marker LIVES, not how big
    // it looks.
    float xr_layer_nav_dist = 400.0f;
    // FORCED composition-layer budget, 0 = use what xrGetSystemProperties reported.
    //
    // Its real job is testability. The drop ordering only runs when a runtime is short of layers,
    // which on a healthy machine is never -- and this feature already has one rung (`skips=` in the
    // XRLAYER state line) that has never executed in its life and is therefore unproven. Setting
    // this to 2 or 3 forces the ORDERING to run, which is the interesting path; 0 layers is only
    // the early-out. Watch drops= move in the state line.
    int   xr_layer_budget = 0;

    // cm, per axis. 0 = NO CLAMP (default) -- see the note at the clamp site: per-axis clamping
    // rotates the offset vector once any axis saturates, so it corrupts direction, not just reach.
    float rig_clamp    = 0.0f;

    // rigtest: write a CONSTANT relative offset of this many cm on Z (up) instead of the
    // controller-derived one. 0 = off (normal tracking).
    //
    // Reduces rig translation to one unambiguous observation: set rigtest=30 and the gun either
    // sits 30 cm higher or it does not. If it moves, the write lever works and any fault is in
    // the pose/neutral math. If it does not, RelativeLocation cannot move this mesh at render
    // time and the weapon actor's own root component is the next lever. No amount of tracking
    // instrumentation separates those two; this does, immediately.
    float rig_test_cm  = 0.0f;

    // rigmode: what frame the rig's RELATIVE transform is expressed in. The first two each fail
    // in an opposite, informative way:
    //   0 = AIM ERROR   -- correct composition, but the error is ~0 once the loop converges, so
    //                      the rig never rolls or translates.
    //   1 = ABSOLUTE    -- roll and translation work, but the parent is ALSO rotated by the aim,
    //                      so the orientation is counted twice and the mesh drifts away from the
    //                      aim ray -- bullets stop following the barrel.
    //   2 = PARENT-RELATIVE -- inverse(parent) * controller. Correct composition like
    //                      mode 0, full roll and translation like mode 1.
    //   3 = DIRECT DRIVE (default) -- the weapon held rigidly by the controller, with the pivot
    //                      MEASURED rather than folded into the mount. See the DIRECT-DRIVE RIG
    //                      block below; pairs with aim_direct, and Ctrl+PageUp swaps both.
    int   rig_mode     = 3;
    // Whether the render path honours the position-dead origin-hold. 1 = honour it (the arms hold
    // rotation and stay put during a controller-position outage); 0 = write the offset anyway, so
    // the unguarded behaviour is visible. LIVE, so it can be flipped inside one outage.
    //
    // Default 1 is the conservative shipping choice, NOT a verdict: the guard was derived from code
    // reading and has never been watched failing. See the banner at the g_rigw_off_valid write.
    int   rigw_off_hold = 0;

    // Anchor hand TRANSLATION to the body rather than to a fixed point in the room.
    //
    // With a fixed neutral, the measured offset is the hand's distance from wherever it happened to
    // be at reference capture -- so walking across the room drags the gun to the clamp and holds it
    // there. Subtracting a body reference first leaves only hand-relative-to-body motion, which
    // is what a held object should follow.
    //
    // THE BODY REFERENCE IS THE STANDING ORIGIN, NOT THE LIVE HEAD -- this is UEVR's own model for
    // controller attachments and the head term genuinely cancels out of it (derivation at the use
    // site in Plugin.cpp, from UObjectHook.cpp:1963-2054). Subtracting the live HMD is equivalent
    // ONLY while the standing origin sits on your head, i.e. while a leash is holding it there,
    // which is why the old head-relative form looked right until hmdleash=0 separated the two and
    // left the arms parked at the pawn.
    bool  rig_body_anchor = true;

    // Sign of the room->game view-yaw correction:  1 = apply, -1 = apply negated, 0 = off.
    // The sign is a convention question about which way UEVR's pinned view yaw runs relative to
    // our axis mapping, and one in-headset A/B settles it faster than deriving it -- a wrong
    // guess produces drift that looks identical to having no correction at all.
    float rig_view_yaw = 1.0f;

    // Subtract a captured neutral from the hand offset. OFF by default and kept only for A/B:
    // it displaces the centre of rotation by the neutral vector (see the note at the use site),
    // and UEVR's own attachment path subtracts nothing.
    bool  rig_neutral  = false;

    // Sign of the snap/smooth-turn correction applied to the rig frame. 1 = apply, -1 = negated,
    // 0 = off (kept for A/B).
    float rig_turn     = 1.0f;

    // Same correction for the AIM mapping. Separate from rig_turn because the two are derived in
    // different frames -- the rig composes a UE-space quaternion, the aim takes a VR-space atan2 --
    // so their sign conventions are not guaranteed to agree. Independent dials, one A/B each.
    float aim_turn     = 1.0f;

    // ---- MOVEMENT DIRECTION. Rotates the LEFT stick so "forward" means where you are LOOKING,
    // not where the game's camera happens to point.
    //
    // Blam moves relative to its own camera yaw, which the view lock has deliberately decoupled
    // from the rendered view -- so without correction, stick-forward follows the AIM instead of
    // the player (movement direction depends on aim direction). The correction is the angle
    // between the two, applied to the stick vector before the game sees it.
    // 1 = apply, -1 = negated, 0 = off. -1 is what measures correct on this title (in-headset
    // A/B), hence the default.
    float move_rot     = -1.0f;

    // Which view-yaw source feeds the movement frame:
    //   0 = UEVR's post-stereo rotation (unconfirmed whether it carries the head term)
    //   1 = rebuilt as pinned + turn + HMD yaw (definitely carries it)
    // Exists because the two cannot be told apart from the resulting angle alone.
    int   move_src     = 0;

    // Recompute the movement angle in the XInput hook from the render-rate view yaw (1) instead of
    // using the value latched on the engine tick (0). 1 removes the head-motion lag.
    // 0 = tick-latched delta, 1 = live head + smoothed aim, 2 = live head + COMMANDED aim
    // (no sampling lag, no actuator settling -- correct once blamangles drives the aim).
    // 3 = live head + SAMPLED aim: the setpoint computed in the movement block itself rather than
    // read from where the aim law last published it. 2 still carries the aim law's call-order --
    // with aimrate=1 that law runs later in the same XInput callback, so 2 pairs the current view
    // against the previous poll's setpoint, an error that grows with hand rotation RATE. 3 has no
    // publication between the two terms and calls the same function the blamangles driver writes
    // the sim from. Kept as a separate rung so 2-vs-3 is a live A/B, not a rebuild.
    // Default 3 = the sampled rung, which is what shipped (older configs said 4; every test is
    // >=, so 3 and 4 are the same behaviour).
    int   move_live    = 3;

    // MOVERESID (dev builds only): log the movement-frame residual every N applications, 0 = off.
    //
    // The residual is  wrap180(actual sim aim - the aim term the rotation used).  It is the whole
    // error: the walk direction comes out as (blam_frame + stick_in - delta*move_rot), so with
    // moverot=-1 it reduces to (view + stick_in) exactly when that residual is zero, and misses by
    // it otherwise. Logging DELTA on its own cannot show this -- delta is SUPPOSED to change when
    // the controller rotates (it is what cancels the aim), so a changing delta proves nothing.
    //
    // Reading the shape of the residual is the diagnosis:
    //   transient, scales with how fast the hand is turning, settles to ~0  -> sampling/order lag
    //   persistent offset proportional to hand rotation                     -> wrong frame or gain
    //   stays ~0 while the walk still bends  -> the sim does NOT move in the aim frame; it moves in
    //       a rate-limited body facing, and no latency work will fix it. The MOVE PROBE on the tick
    //       (which derives the frame from observed motion) is then the instrument, not this.
    int   move_resid   = 0;

    // Per-tick smoothing factor for the aim term in the movement frame (0 = off/instant, 1 = no
    // damping). At ~32 Hz, 0.25 settles in roughly a tenth of a second: enough to swallow the aim
    // actuator's slew and overshoot without movement feeling like it lags a deliberate turn.
    // Default 0: with movelive=3 the view and aim terms are sampled in the same call, so there is
    // no inter-source lag left to damp and smoothing only adds walk-direction latency.
    float move_smooth  = 0.0f;

    // Require is_hmd_active() before driving. Keep 1 for real use; 0 is for headless runs under
    // a null/simulated SteamVR driver, which reports the HMD inactive regardless.
    bool  require_hmd  = true;

    // ---- ATTACHMENT MECHANISM.
    //   0 = RIG DRIVER (default): we write the component's relative transform ourselves.
    //   1 = UOBJECTHOOK: hand the arms to UEVR's own motion-controller attachment and let it do
    //       the per-eye transform, the same path every other UEVR mod uses.
    //
    // Kept as a switch, not a replacement, so a bad result is one config line away from reverting.
    //
    // PERSISTENCE IS OURS, NOT UEVR's. UEVR only writes uobjecthook/*_mc_state.json from a Save
    // button inside its own ImGui (UObjectHook.cpp:4520) -- the plugin API can set attachment state
    // but cannot persist it. So we re-apply the attachment from halo_vr_calib.cfg every session
    // instead. That is strictly better here: it also avoids UEVR's saved-StatePath resolution,
    // which is the machinery that races on map load.
    // Re-apply the rig's rotation at RENDER rate instead of only on the ~32 Hz tick.
    //
    // WHY. The rig is a child of the CameraComponent and we write it a RELATIVE rotation, which is
    // only correct for the parent orientation sampled at that instant. The parent yaw can move
    // ~14 deg BETWEEN TICKS during an aim turn, so the arms' world orientation drifts by that much
    // and snaps back on the next write -- a 14 deg sawtooth at 32 Hz, felt as arm shake.
    // Recomputing the relative rotation against the LIVE parent each render frame removes it at
    // source, and keeps the existing maths and calibration untouched.
    bool  rig_render   = true;
    int   attach_mode  = 0;
    bool  attach_permanent = false;

    // ---- CONTROL REMAPPING (XInput button masks; 0 disables a mapping)
    //
    // Done here rather than in UEVR's binding files because the right stick's Y axis is free for us
    // specifically: we read only X (turning), and both axes are overwritten by the aim output later
    // in the same callback. So the player's right-stick Y can be repurposed with nothing lost.
    //
    // Replaces UEVR's DPadShifting, whose default method is "Right Thumbrest + Left Joystick" --
    // resting a thumb on the capacitive pad silently converts the LEFT stick to a d-pad and
    // movement stops. Set VR_DPadShifting=false in config.txt alongside this.
    //   DPAD_UP 0x0001  DOWN 0x0002  LEFT 0x0004  RIGHT 0x0008
    //   START 0x0010  BACK 0x0020  LTHUMB 0x0040  RTHUMB 0x0080
    //   LB 0x0100  RB 0x0200  A 0x1000  B 0x2000  X 0x4000  Y 0x8000
    // Right stick UP acts as the D-PAD SHIFT, replacing the capacitive thumbrest: while held, the
    // LEFT stick's directions become d-pad presses and left-stick movement is suppressed. Same
    // behaviour as UEVR's "Right Thumbrest + Left Joystick", but on a deliberate input rather than
    // one a resting thumb triggers by accident.
    bool  map_dpad_shift  = true;

    // ---- HEAD-PROXIMITY D-PAD SHIFT ------------------------------------------------------------
    //
    // Bring either hand near your head and the RIGHT stick becomes the d-pad, leaving the LEFT
    // stick on locomotion. The stick-up shift takes the left stick instead, so today you cannot
    // move while switching grenades, dropping a weapon or triggering equipment -- which is exactly
    // when you most want to be moving.
    //
    // OFF BY DEFAULT, and it should stay off until the radius has been tuned in a headset. The
    // failure is ASYMMETRIC: a false trigger costs you TURNING mid-fight, with no visible cause,
    // which is far worse than a missed d-pad press. So every gate here fails closed.
    //
    // Four things routinely put a hand near the head and none of them mean "d-pad": the two-handed
    // hold with the barrel raised (excluded explicitly -- two_hand_latched()), the reload
    // magazine grab, the melee swing windup, and simply resting a hand. Hysteresis plus a dwell
    // are what stop the first two transients; the radius is what stops the rest, and it is a
    // comfort number that cannot be picked from outside a headset.
    bool  dpad_head       = true;
    float dpad_head_cm    = 18.0f;   // hand-to-head distance that ARMS the shift
    float dpad_head_hyst_cm = 8.0f;  // extra distance before it releases, so it cannot chatter
    int   dpad_head_dwell_ms = 120;  // how long the hand must stay there before it commits
    int   map_rstick_down = 0x2000;   // right stick DOWN -> B, crouch on this game's pad map
    float map_rstick_dz   = 0.65f;    // deflection needed; high so turning never trips it
    float map_dpad_dz     = 0.50f;    // left-stick deflection needed to count as a d-pad direction

    // Rebind a button to a different one. `mapfrom` is suppressed and `mapto` sent instead.
    // NEVER GUESS THESE MASKS: on this profile XInput's labels do not match the controller's, so a
    // wrong value silently unbinds a combat action. Measure with `mapbtnlog=1`.
    //
    // CORRECTED 2026-09-04. This block used to say 0x0100 was "LB = equipment". It is not -- 0x0100
    // is the GRENADE throw mask, which is also what the physical left grip sends, and the code in
    // Plugin.cpp's grip-swallow note says so plainly. Acting on the wrong comment put grenades on
    // d-pad left for a while. The measured truth, logged this session:
    //   left X      -> 0x2000     (XInput calls it B)
    //   right grip  -> 0x0200     the game's EQUIPMENT button (overshield / active camo)
    //   left grip   -> 0x0100     the game's GRENADE throw
    // EQUIPMENT (overshield / active camo) ON LEFT X.
    //
    // MEASURED 2026-09-04: left X reports 0x2000, and the game's equipment button is the RIGHT
    // GRIP, 0x0200 (logged five times). So this is left X -> equipment.
    //
    // Left X is free because the grenade moved onto the off-hand trigger (see grip_zoom). Before
    // that, mapto was 0x0100 -- which is the THROW mask, not equipment, whatever the old comment
    // here claimed. That mistake is why d-pad left threw grenades for a while.
    //
    // The right grip keeps working natively; this adds a binding rather than moving one.
    int   map_from        = 0x2000;
    int   map_to          = 0x0200;

    // ---- RIGHT GRIP IS RESERVED, AND NO LONGER EQUIPMENT ---------------------------------------
    //
    // The right grip natively sends 0x0200, which this game reads as EQUIPMENT (overshield /
    // active camo). That native binding is being retired: the right grip becomes OVER-THE-SHOULDER
    // WEAPON SWITCHING, and a button cannot mean two things at once -- reaching back to swap
    // weapons would pop your overshield every time.
    //
    // Equipment does not lose a home by this; it gains one. It is on LEFT X (mapfrom/mapto above)
    // and on d-pad LEFT under the shift.
    //
    // WHY A SWALLOW AND NOT A REMAP. There is nothing to remap the grip TO -- the point is that the
    // GAME must stop seeing the press, while the grip stays readable by us for the gesture. Set
    // rgripswallow=0 to hand it straight back to the game.
    //
    // MEASURED 2026-09-05 with mapbtnlog=1, five deliberate presses: right grip -> 0x0200 RB.
    // Not reasoned about -- on this profile XInput's labels and the controller's do not line up,
    // and guessing a mask here silently unbinds a combat action.
    //
    // APPLIED AFTER THE BUTTON LOGGER, on purpose. Swallowing before it would blind the one
    // instrument that established this mapping in the first place, and the next person measuring
    // the right grip would find nothing and conclude the controller was not sending anything.
    int   rgrip_mask      = 0x0200;
    bool  rgrip_swallow   = true;

    // Log every XInput button-mask change, so the Quest->XInput mapping can be READ rather than
    // assumed. Off by default; it is noisy.
    bool  map_btn_log     = false;

    // ---- ACTION BINDS (the in-game "Halo VR Controls" panel) --------------------------------
    //
    // Each of these names a MOD ACTION and holds the SOURCE button the player presses for it.
    // The DESTINATION is not repeated here -- it is whatever field already owns that action
    // (maprstickdown for crouch, meleemask, reloadmask, grenadeaction), so there is exactly one
    // place that knows what the game reads for each action and these only say who triggers it.
    //
    // 0 = KEEP THE BUILT-IN MECHANISM, which is why 0 is the shipped value for all of them: the
    // stick gestures and the swing/reload gestures are the designed experience, and a bind is an
    // ADDITION for players whose controller or comfort needs one, never a replacement that has
    // to be configured before the mod works.
    //
    // WHY A SOURCE MASK AND NOT A NAME. The physical-button -> XInput-mask mapping is NOT stable
    // across runtimes: on Quest the right controller's B reports as 0x4000, which XInput (and
    // this file, and the menu's dropdown) calls "X". A mask is the only unambiguous form, which
    // is also why the menu's primary way to set one is CAPTURE (press the button, we record what
    // arrived) rather than a dropdown -- see bind_capture_key below. Nothing here should ever be
    // guessed; measure with mapbtnlog=1 or let the capture do it.
    //
    // Applied BEFORE mapfrom/mapto, so a bind wins over the generic rebind if both name the same
    // mask. That is a conflict the menu warns about rather than a feature -- first match wins is
    // a tie-break, not a layering scheme.
    int   bind_crouch     = 0x0000;   // -> injects maprstickdown's mask (0 = right stick down)
    int   bind_melee      = 0x0000;   // -> injects meleemask          (0 = swing gesture only)
    int   bind_reload     = 0x0000;   // -> injects reloadmask         (0 = reload gesture only)
    int   bind_scope      = 0x0000;   // -> toggles the VR scope       (0 = left trigger)

    // EQUIPMENT on d-pad LEFT -- and the one member of this family that does NOT default to 0.
    //
    // The 0-default rule above exists because every other action here already has a built-in
    // mechanism a bind would merely duplicate. Equipment has none: it used to ride mapfrom/mapto,
    // and when that pair moved to LEFT X the d-pad-left binding ceased to exist entirely. Several
    // comments in this file went on describing it as though it were still there, which is exactly
    // how it survived unnoticed -- the mechanism was gone and only the documentation remained.
    //
    // So this is not a convenience bind, it is the second home the design calls for: equipment on
    // left X AND on d-pad left, now that the right grip is reserved (see rgrip_swallow above).
    //
    // 0x0004 is XINPUT_GAMEPAD_DPAD_LEFT, which is what the shift SYNTHESISES -- our own injection
    // upstream, not a physical button, since neither Touch controller has a d-pad. It is read
    // after that injection for exactly that reason. Set bindequip=0 to give d-pad left back to
    // whatever the game does with it natively.
    int   bind_equip      = 0x0004;   // -> injects mapto's mask       (0 = no d-pad binding)
    int   bind_dpad_shift = 0x0000;   // -> holds the d-pad shift      (0 = right stick up)

    // How long an armed capture waits for a press before giving up, milliseconds.
    //
    // A capture EATS the press it records, so an arm that never expires is a booby trap: the
    // player gets distracted, plays on, and the next button they touch is silently swallowed and
    // rebound. Bounded instead, and the menu reports the armed state on the way back in.
    int   bind_capture_ms = 20000;

    // ---- FRAME-TIME INSTRUMENTATION --------------------------------------------------------
    // Time the PERIODIC work in update() and report max/mean per site every ~600 ticks.
    //
    // Exists because the periodic microstutter report had three plausible suspects at similar
    // periods and no way to tell them apart: nothing in this plugin had ever been frame-timed.
    // Guessing between them costs a rebuild per guess and can "fix" the wrong one by coincidence.
    //
    // Ships OFF. Cost when on is one QueryPerformanceCounter pair per instrumented site per
    // period (not per frame), which is noise -- but the log lines are not, and a user who is not
    // debugging should not pay for them. Turn on to diagnose a stutter report from someone whose
    // machine reproduces it and ours does not.
    bool  perf_log        = false;

    // ---- WIDGET-SWEEP SLICE BUDGET, in milliseconds of one game-thread tick.
    //
    // reticle_rescan() walks the whole UObject array looking for the HUD reticle, the navpoint
    // layer and menu candidates. That walk costs one cache miss per object and the object array is
    // a HIGH-WATER MARK that only grows for the life of the session, so its cost climbs with
    // playtime no matter how the inner loop is written: 83.9 ms, optimised to 29.8 ms in
    // 2026-08-12, measured back at 55.3 ms on 2026-08-23 with the optimisation fully intact.
    // A 55 ms game-thread stall in VR is a dropped frame, which is a nausea event.
    //
    // So the walk is spread over consecutive ticks, at most this many milliseconds per tick, and
    // its results are published atomically when a pass finishes. Total CPU is unchanged; what
    // changes is that none of it lands in one frame.
    //
    // 0 disables slicing and does the whole array in a single tick -- the pre-2026-08-23
    // behaviour, kept so this can be A/B'd in a headset without a rebuild, the same way rigfast=0
    // A/Bs resolve_rig's fast path. Raising it above ~4 costs frames again; lowering it below
    // ~0.5 makes a pass take long enough that a fresh reticle binds noticeably late after a load.
    float ret_sweep_ms    = 2.0f;

    // AIM TRACE. 0->1 arms a recording of the aim loop; 1->0 writes it to halo_vr_trace_NNN.csv
    // beside this config. Exists so the aim law can be tuned by measurement instead of by feel:
    // lag and jitter feel alike in a headset but are trivially separable in a trace.
    //
    // DEV BUILDS ONLY -- in a release build the recorder is not compiled in and this key does
    // nothing, so leaving it set in a config can never cost a player anything.
    bool  aim_trace       = false;

    // HEAP VALUE SCAN (dev builds only -- see MemScan.hpp).
    //
    // 0 -> 1 starts one background scan for the comma-separated float sequence in memscanvals,
    // logging every hit with its address and the surrounding floats. Exists to locate Blam-side
    // data that UE reflection cannot reach and UEVR's pattern scanner cannot see, because it lives
    // in heap rather than the executable image.
    //
    // Example, to find `globals -> player control` via its documented magnetism pair:
    //     memscanvals=0.625,0.625
    //     memscan=1
    // HARDWARE WRITE-WATCHPOINT on the aim (dev builds only -- see AimWatch.hpp).
    // 0 -> 1 arms a debug-register watchpoint and logs the distinct instructions that write
    // ControlRotation.Yaw, which is the one place the real aim provably reaches. Auto-disarms.
    // aim_watch_addr 0 = use the live PlayerController + CONTROL_ROTATION_OFFSET + 8.
    bool     aim_watch      = false;
    uint64_t aim_watch_addr = 0;

    bool  mem_scan        = false;

    // AIMDIG (dev builds): 0 -> 1 with aimdirect resolved runs one off-thread derivation-chain
    // report -- allocation census for L2/obj/simTLS, exe-data static-root scan, pointer-graph
    // walks from the sim TLS block and the quat-source object toward L2. See MemScan.hpp.
    bool  aim_dig         = false;
    char  mem_scan_vals[128] = {0};

    // Hit cap for one scan. 0 = the built-in default (512). The old hard-coded 64 made every busy
    // search silently partial -- see the note in MemScan.cpp. Raise it when a signature is short
    // enough to collide often; a long signature is the better fix.
    int   mem_scan_max = 0;

    // Differential scan stage: 1 snapshot, 2 changed-blocks -> candidates, 3+ narrow, 0 clear.
    int   mem_diff = 0;

    // One-shot: resolve the Blam object table and report which object's +0x50 matches the aim.
    // Read-only diagnostic. See BlamAim.hpp for why this is a pointer chain rather than a search --
    // the aim has many derived copies and value scanning cannot tell them from the source.
    int   blam_aim = 0;

    // TRIM on the redirected shot direction (blamaim>=3), in degrees. Two jobs, same mechanism:
    //   * calibration -- null out a constant gap between where the rounds land and the reticle;
    //   * causality test -- a large value proves the write is really steering the projectile,
    //     which is otherwise indistinguishable from doing nothing.
    // Float, not int: the gap being trimmed is a fraction of a degree once close, and whole-degree
    // steps cannot express it. Live-tunable (polled ~2 s, applied on change).
    float blam_yaw_off   = 0.0f;
    float blam_pitch_off = 0.0f;

    // WHICH call site may be redirected, as an RVA into HaloSimulation_tag_release.dll.
    //
    // create_projectile is shared by EVERYTHING that fires -- AI, teammates in co-op, scenery. An
    // unfiltered redirect points all of it along the player's controller, which is invisible in an
    // empty corridor and chaos in a firefight. dll+0x5D124E is the player-weapon path, observed
    // live; dll+0x5B4FA5 is a different site that emitted 134 spawns at level load.
    //
    // Fails CLOSED: an unrecognised site is left alone rather than redirected. If a weapon stops
    // following the controller, its site is simply not in the list yet -- read `caller=` in the
    // BLAMSPAWN log and add it. 0 disables the filter entirely (redirect everything; diagnostic
    // only). Accepts hex, e.g. blamcaller=0x5D124E.
    // Dump the head of the projectile spawn params (dev diagnostic, first 40 spawns) to locate the
    // owner/shooter field. Needed because call-site filtering cannot separate player from AI.
    int   blam_dump = 0;

    // Object handle whose float fields to dump (~0.7 Hz), for finding the sim's own aim field.
    // e.g. blamobj=0x0001. 0 = off.
    int   blam_obj = 0;

    // Bytes of the object to dump (0x80..0x800, default 0x200). The aim state the firing solution
    // consumes is past 0x200, and a diff only finds what it dumps.
    int   blam_obj_len = 0x200;

    // One-shot: find the player's Blam object by matching a spawn's muzzle origin. Self-deriving,
    // unlike a handle noted in a previous session.
    int   blam_find = 0;

    // Sweep every object for a unit vector equal to the aim. 1 = collect, 2 = verify after moving
    // the aim. Finds the aim field without needing to know which object is the player.
    int   blam_scan = 0;

    // Second object for blamwrite=7 (the node-transform holder BLAMSCAN flagged).

    // Scalar (Euler yaw/pitch) aim scan. 1 = collect, 2 = verify after moving the aim.
    int   blam_scan2 = 0;

    // BLAMSCAN3: hunt the aim in the sim module's writable GLOBALS rather than the object table --
    // the search space every earlier scan missed, and where Blam keeps player control state.
    // 1 = collect candidates at this pose, 2 = confirm at a second pose. Dev builds only.
    int   blam_scan3 = 0;

    // BLAMNODE: log the base direction the game hands the spread function, against ControlRotation.
    // Needs blamwrite != 5, or we overwrite the value we are trying to read. Dev builds only.
    int   blam_node = 0;

    // BLAMANGLES: drive the sim's ANGULAR CONTROL STATE (the player control record at
    // TLS+0xB8, stride 0x198, yaw at +0x94 wrapped [0,2pi), pitch at +0x98) from the
    // controller every frame. The address is resolved deterministically -- no scan needed.
    //   0 = off, 1 = drive from the controller, 3 = pin a fixed angle (probe; PINS THE VIEW).
    // This is the injection point that is past input conditioning (acceleration, deadzone,
    // magnetism) but upstream of the aim vector and of replication -- i.e. 1:1 and it
    // reaches the wire. See docs\BLAM_AIM_FINDINGS.md.
    //
    // DEFAULTS ON. This is the aim solution, not an experiment: it is what makes hand-aim 1:1
    // instead of fighting the stick's acceleration curve and deadzone. It lives in BlamDrive.cpp,
    // which is NOT dev-gated, so it is present in a release build -- and a default of 0 would have
    // shipped that code compiled in and switched off, which is the same as not shipping it.
    // Set to 0 to fall back to the steered stick path.
    //
    // CANONICAL PAIRING: blamangles=1 WITH aimdirect=1. They do different halves of one job --
    // this one is 1:1 and reaches the host, aimdirect steadies the local view. Running this
    // without aimdirect leaves the closed loop still steering toward a setpoint that is also being
    // assigned exactly, and the two fighting reads as heavy jitter (measured live, not predicted).
    // blam_drive_tick() logs a warning on that combination rather than letting it be discovered
    // as "the mod feels bad".
    int   blam_angles = 1;

    // FAULT INJECTION for the aim cascades -- a BITMASK. DEV BUILDS ONLY: every effect site is
    // #if HALO_VR_DEV, so a release build ignores this entirely and it cannot reach a player.
    //
    // Both cascades exist so that a build whose addresses have moved still works. Every tier below
    // the first therefore executes ONLY when something is broken, which makes them exactly the code
    // that rots unnoticed -- the failure this whole lane was opened for. These bits force each
    // branch to run on a HEALTHY machine so it can be proven rather than hoped for.
    //
    //   0x001  hook installs but is NEVER CALLED   -> watchdog fires, tier 2 (TEB scan) takes over.
    //                                                 Tier 1 stays silent, so any result is
    //                                                 unambiguously tier 2's doing.
    //   0x002  getter signature finds NOTHING      -> falls back to the recorded RVA
    //   0x004  getter signature finds TWO matches  -> ambiguous, must refuse and fall back
    //   0x008  _tls_index lands at a DIFFERENT rva -> BUILD DIFFERS warning (does not disable)
    //   0x010  scan result != recorded RVA         -> "getter MOVED" (checks the REPORT; the real
    //                                                 address is still hooked, so this is safe)
    //   0x020  TLS directory read FAILS            -> hard stop, aim write disabled loudly
    //   0x040  announce every periodic re-resolve  -> makes a deliberately silent timer observable
    //   0x080  corrupt AimDirect's cached hint     -> "cached offset did not validate", falls
    //                                                 through to the full watch
    //   0x100  AimDirect locate always fails       -> exercises the MAX_ATTEMPTS give-up
    //   0x200  freeze the layout guard's candidate -> what a MOVED STRUCT OFFSET looks like: the
    //                                                 watched field stops tracking the aim, so the
    //                                                 guard must refuse to arm the sim write
    //   0x400  starve AimDirect of evidence        -> no candidate ever validates AND no motion is
    //                                                 credited, so each stage must TIME OUT rather
    //                                                 than fail fast. Exercises the ~24 s per-stage
    //                                                 timeout branches, which 0x100 does not: that
    //                                                 one advances the attempt count by REJECTING
    //                                                 candidates, a different path.
    //
    // Install-time bits (0x002..0x020) can be re-tested without rebooting: toggling `blamangles`
    // 1 -> 0 -> 1 unregisters the hook and re-runs the entire install path.
    //
    // Pair 0x001 with blamangles=3 to test tier 2's WRITE without controllers -- that path stores a
    // fixed (1.50, 0.30) before desired_aim_now() is consulted, so it needs no poses.
    int   blam_fault = 0;

    // LAYOUT GUARD for the control record's struct offsets. 1 = on (default), 0 = off.
    //
    // SHIPS ENABLED, and unlike the fault mask this one is in every build. The address cascade
    // protects the CODE address; nothing in it checks that yaw is still at +0x94. A field that moved
    // four bytes passes the signature check, the record resolve, IsBadWritePtr, and a read-back of
    // our own write -- and we would then stamp aim into whatever now lives there, every frame.
    //
    // So the write is held off until the record's yaw has been observed tracking the game's own aim
    // (about 25 degrees of look-around while neither of our drivers is writing). Costs a short
    // unarmed window at the start of a level; the local view is unaffected because aimdirect owns
    // that independently.
    //
    // Set 0 if the guard ever misjudges a build -- it is the A/B for its own behaviour, and it is
    // the one switch that turns a false positive from "no sim aim" back into "sim aim as before".
    int   blam_layout = 1;

    // BLAMCTL REPORT rate, in writes between log lines. 0 = off. DEV BUILDS ONLY -- the field is
    // parsed in a release build but nothing reads it, because the report lives behind HALO_VR_DEV.
    //
    // Exists for the reticle-vs-shot offset: the reticle is drawn from ControlRotation and the shot
    // from this record, so when they disagree the question is which one moved, and by how much. The
    // report prints both plus the record's read-back at one instant. Default 4000 (~1.5 s at the
    // getter's ~2600 calls/sec) because the fault is intermittent and per-session -- waiting to
    // enable it after noticing means reproducing it again.
    int   blam_ctl_log = 4000;

    // Yaw sign for the above. The record stores BLAM yaw, the negation of the UE-convention yaw
    // that desired_aim_now() returns, so this defaults to -1. Separate knob so it can be flipped
    // live rather than rebuilt if a future build changes the convention.
    int   blam_angles_ysign = -1;

    // (blamwrite, blamwriteysign, blamplayeridx, blamshooter, blamobj2, blamcaller and blamcaller2
    //  were here. All seven existed to aim or filter one of the WRITE paths -- the obj+0x1D4 vector
    //  write, the fire-function target patch, the spawn redirect -- and all of those are removed as
    //  refuted. Nothing read these afterwards; they were parsed in every build and consumed by
    //  nothing. The negative results they belong to are in docs\BLAM_AIM_FINDINGS.md.)


    // PINNED PLANT GAIN, deg/s per unit of stick. 0 = measure it live (the historical behaviour).
    //
    // Feedforward and damping are both gated on a measured plant gain above 10, and the live
    // measurement only runs on the TICK path -- it is explicitly skipped when aim_rate_render is
    // set, because sampling the applied deflection once per tick misestimates a value that changes
    // many times within one tick. But aim_rate_render DEFAULTS ON. So on the default configuration
    // the gain stays 0 forever, and feedforward and damping have never executed at all: the loop
    // has been running pure proportional, which is exactly the arrangement whose tracking lag the
    // feedforward was added to remove.
    //
    // Pinning the gain restores both without having to re-derive the measurement for the
    // render-rate path. It also makes the plant CONSTANT, which live adaptation does not: an
    // adapting gain is a slow feedback loop that resettles between measurements, so every
    // configuration in a tuning run would otherwise be evaluated against a different plant.
    //
    // ~226 was measured on this title from recorded traces (stick deflection vs achieved turn
    // rate). It is sensitivity-dependent, so it is not a universal constant -- a player on a very
    // different look sensitivity wants their own number, or 0 to fall back to measuring.
    float meas_rate_fixed = 0.0f;

    // STARTING plant gain, before anything has been measured. 0 = use REFERENCE_RATE_DPS.
    //
    // The first admissible measurement used to be taken RAW -- no averaging -- so the loop's very
    // first gain was one interval's reading. Feedforward divides by that gain, so a first sample
    // reading 27 where the truth is ~105 makes feedforward roughly 4x too LARGE for a few hundred
    // milliseconds: a slam to full stick where a quarter was wanted. The stick response is
    // exponential, so a single sample at one deflection is a point on a curve, not the gain.
    //
    // Seeding high is the safe direction: too high makes feedforward too SMALL, which is mild and
    // converges away, where too low slams. Feedforward is therefore live from the first frame,
    // approximately right, rather than exactly right after a lurch.
    //
    // LIVE, and applied ON CHANGE -- writing a new value RESETS the learned gain to it and lets
    // adaptation re-converge. That is deliberate: the startup transient otherwise happens once per
    // launch and is nearly impossible to observe on purpose. Changing this mid-session is how you
    // reproduce it. Editing it to the value it already has does nothing, so the ~2 s config poll
    // does not re-seed continuously.
    float gain_seed = 0.0f;

    // FULL-DEFLECTION TURN RATE, deg/s. 0 = estimate it from the adaptive gain.
    //
    // Feedforward now inverts the MEASURED plant curve (see Math.hpp / docs\PlantCurve.md) instead
    // of dividing by a single deg/s-per-unit scalar, because the real gain varies about 7x across
    // the stick and a scalar is therefore wrong at every deflection but one. The curve is stored as
    // fractions of the full-deflection rate, so this one number anchors it to whatever sensitivity
    // is actually configured.
    //
    // 363.6 was measured at LookSensitivity90 / deadzone 0 / acceleration min. A different
    // sensitivity wants its own value -- run Scripts\AimTune\Measure-PlantCurve.ps1 and take the
    // rate at deflection 1.0. The measured value is the default; set 0 to fall back to estimating
    // it live from the adaptive gain, which is serviceable but coarse.
    // DEADBEAT setpoint. 0 = off (aim_decel / aim_tau_s pace the approach as before).
    // >0 = ON, and the value is the per-tick SAFETY FACTOR: 0.85 closes 85% of the remaining error
    // every tick, 1.0 is true deadbeat. Justified only because the plant measured MEMORYLESS
    // (2026-08-06), which is what makes the curve inversion meaningful.
    //
    // Trade-off to remember when tuning: tau also low-pass filtered hand tremor. Deadbeat does not.
    // If the aim reads jittery, smooth the TARGET rather than lowering this back toward tau.
    float aim_deadbeat = 0.0f;

    // AIMSTAT: report mean/max |yaw error| every N ticks. 0 = off. Dev builds only.
    // Exists because 600-tick spot samples cannot distinguish controller settings.
    int   aim_stat = 0;

    // Target smoothing time constant in ms, applied to the CONTROLLER angles the aim law chases.
    // 0 = off. This is the tremor filter aim_deadbeat gives up when it stops using aim_tau_s to
    // pace the approach. Try ~20-40 ms if the aim reads jittery in a headset; raising it costs
    // target latency, NOT convergence speed, which is the trade tau got wrong.
    float aim_target_smooth_ms = 0.0f;

    float plant_full_dps = 363.6f;

    // AIM TIME CONSTANT, seconds. 0 = use the legacy linear error-to-deflection map.
    //
    // With this set, the proportional term asks for a closing RATE of err/tau and the measured
    // plant curve converts that to a deflection. The aim then closes ~63% of any remaining error
    // in tau seconds -- the same behaviour for a 2 degree correction as for a 60 degree one, and
    // the same at any game sensitivity.
    //
    // The legacy map instead converts error straight to deflection, so the real closing gain rides
    // the plant's 7x nonlinearity AND scales with the sensitivity setting. That is why raising
    // sensitivity introduced springiness with no config change: the loop gain roughly tripled.
    //
    // TUNING: larger = calmer and slower to arrive; smaller = snappier and more prone to overshoot.
    // It cannot go below the loop's own latency (~30-60 ms) without ringing, since the loop cannot
    // stop a rate it only learns about a frame later.
    float aim_tau_s = 0.0f;

    // CONSTANT-DECELERATION closing profile, deg/s^2. 0 = use the exponential aim_tau_s profile.
    //
    // With tau, rate = err/tau: the aim decelerates in proportion to remaining error, so it leaves
    // fast and creeps in. With aim_decel, rate = sqrt(2*a*err): it holds speed and then brakes at a
    // fixed rate, arriving in finite time. That is the difference between "slows down before it
    // gets there" and a crisp arrival, and it is why this exists rather than three hand-tuned
    // speed bands for small / mid / large movements.
    //
    // Bounded by the loop's own reaction time, not by taste: the aim law sees its rate a frame
    // late, so beyond some `a` it cannot brake in time and overshoots. Tune against the sweep-stop
    // case, which is the one that exposes it.
    float aim_decel = 0.0f;

    // ASSIGN the aim instead of steering it through the stick. 0 = the closed loop (shipping).
    //
    // Blam turns out to expose a writable rotator upstream of ControlRotation, so aim can be set
    // exactly and instantly rather than driven toward (docs\BLAM_AIM_DIRECT_WRITE.md). With this on,
    // every stick-path workaround stops applying: no hardware deadzone to clear, no nonlinear
    // response curve, no ~14 deg/s minimum correction, no dead time, no overshoot, nothing to tune.
    //
    // DEFAULTS ON as of 2026-08-08. It previously defaulted off pending a multiplayer answer -- the
    // worry being that direct assignment writes the value the game replicates. That worry is
    // resolved, and not the way it was framed: this rotator is a LOCAL mirror and does not reach
    // the wire at all. Replication is carried by `blamangles`, which drives the sim's angular
    // control record (BlamDrive.cpp), and the two were verified together in co-op -- local view and
    // host view agreeing once the yaw sign was flipped.
    //
    // So the pairing is: blamangles reaches the host, aimdirect steadies what the player sees.
    // Turning this off with blamangles on was tried live and reads as heavy jitter, because the aim
    // is then being steered toward the setpoint by the loop while also being written exactly.
    //
    // CANONICAL PAIRING: aimdirect=1 WITH blamangles=1. Treat them as one setting with two halves,
    // not two independent toggles -- see the note on blam_angles. Either alone is a downgrade, and
    // the combination is what the whole aim solution is.
    bool  aim_direct = true;

    // FREEZE AIM WHEN THE CONTROLLER POSE GOES EMPTY (position AND rotation exactly zero), rather
    // than driving aim from it. On an OpenXR focus loss the runtime stops updating action poses but
    // UEVR still returns success, so without this the direct write assigns aim to wherever identity
    // points -- measured at a 76-degree slam, then held for the 3m38s the session stayed unfocused
    // (2026-09-02 log). Costs one comparison per frame.
    //
    // ON by default and there is no good reason to turn it off; it exists as a key only so a
    // misfire can be ruled out live without a rebuild. If aim ever freezes while your hands are
    // visibly tracking, set aimfreezelost=0 and say so -- that would mean a real pose is landing on
    // bit-exact zero in position and both angles at once, which should not be possible.
    bool  aim_freeze_lost = true;

    // ---- THE GRAB GUIDE ------------------------------------------------------------------------
    // A thin translucent beam from the support hand to the point it would grab, shown ONLY while a
    // grip press would actually latch the two-handed hold. Player IK is off in release, so there is
    // no left arm on screen and nothing otherwise tells the player whether their hand is on the
    // barrel -- the hold latches on a zone they cannot see. See InteractLine.hpp.
    bool  grab_guide = true;
    // HUD-ish cyan by default. These are the beam's own colour rather than a reference to the
    // reticule's, because the two are read at different distances and want different weights.
    float grab_guide_r = 0.35f, grab_guide_g = 0.85f, grab_guide_b = 1.00f;
    // WHAT THE GRAB GUIDE DRAWS. 0 = the "Grip" LABEL at the off hand (default: it makes no claim
    // about WHICH object, so it cannot be wrong about one). 1 = the BEAM from hand to latch point,
    // which answers "which one" and becomes the better answer once grenades and magazines are
    // grabbable and there is more than one candidate.
    int   grab_guide_mode = 0;
    // WHICH BASIS THE BEAM QUAD IS BUILT FROM. 0 is derived from xr_look_rotation's actual
    // convention (z = -fwd, x = cross(up, z)), so x -- the quad's WIDTH, which is the axis scaled
    // to the beam's length -- lands along the beam while the quad still faces the viewer.
    //
    // It is a TUNABLE rather than a constant because the pair goes through ue_offset_to_xr on the
    // way in, and a sign or handedness flip in that mapping is not visible from reading either
    // function alone. At ~70 s a build, four rebuilds to find a sign is minutes lost; four live
    // values is seconds. 1 = up flipped, 2 = beam as the quad's UP (a deliberate sanity check --
    // this should look WRONG, thin-axis-along-beam), 3 = the original pre-fix basis for comparison.
    int   grab_guide_beam_basis = 0;
    // LABEL SIZE, in game cm, on the compositor quad anchored to the off hand. Square, so this is
    // both edges. 6 cm reads at arm's length without covering the thing you are reaching for.
    float grab_guide_label_cm = 2.5f;
    // Beam thickness; it lies along a weapon, so keep it thin. LIVE ONLY IN grabguidemode=1 -- the
    // label is square and ignores it, so a player who sets this and sees nothing is on the default
    // mode, not looking at a broken key.
    float grab_guide_thick_cm = 0.5f;
    // Below this length the beam is shorter than it is thick and reads as a blob -- and that is
    // exactly the moment the grab is perfect and the player no longer needs telling.
    float grab_guide_min_cm = 1.5f;
    // Full material object path, for a FEATHERED beam. Empty uses Widget3DPassThrough_Translucent,
    // which gives a clean translucent bar; there is no way to author an alpha falloff in-process
    // (make_color_rt is a flat fill), so a real laser look needs a material from an added pak.
    char  grab_guide_mat[256] = "";

    // Persist the located rotator (pc-relative offset + absolute VA, keyed to the exe's build
    // stamp) in halo_vr_aimcache.txt, so a relaunch on an unchanged binary warm-starts through
    // the normal hint validation instead of paying the full watch hunt. 0 = neither read nor
    // write the file; deleting the file is always a safe reset. See AimDirect.cpp.
    bool  aim_cache = true;

    // RUNG 1: resolve the aim rotator by walking the derivation chain from the PlayerController
    // (PC+0x318 -> +0x3C8 = the owning object, then +0x1C0 -> +0x20 = the rotator) instead of
    // hunting it with hardware watchpoints. Microseconds, no thread suspension, and it does not
    // need the player to move first. Every candidate still passes the same before/after-motion
    // validation, so a moved struct offset falls back to the watch. 0 = skip the chain (the live
    // A/B, and how the watch path gets exercised). See AimDirect.cpp.
    bool  aim_chain = true;

    // Direction of the hand->aim mapping in the DIRECT path only, per axis. +1 writes exactly the
    // setpoint the steered loop converges to; -1 mirrors the hand's motion about the calibration
    // reference. These exist because the first live direct-write test felt responsive but inverted
    // on both axes, which the maths does not predict -- so the mapping is settled by experiment,
    // live, instead of by a rebuild per guess. Does not affect the stick path.
    float aim_direct_sign_x = 1.0f;
    float aim_direct_sign_y = 1.0f;

    // Also write the QUATERNION the Euler aim is derived from. DEFAULT OFF: measured worse.
    //
    // The chain is quat -> Euler(L2) -> L1 -> ControlRotation, so writing the quaternion looked like
    // the fix for the game discarding our Euler on firing. It is not: with the quaternion written,
    // commanded yaw swept -10.7 -> +0.3 while the achieved aim sat frozen at -6.5. The quaternion is
    // itself recomputed from the game's own input integration, so the write is overwritten a level
    // further down and tracking is lost entirely -- strictly worse than Euler-only, which tracks and
    // only loses on resync. Kept because it is the correct place to write IF the deeper store is
    // ever found and neutralised.
    // Alternate the emitted stick value by one raw LSB each frame, so the exe's input poll always
    // sees a CHANGED axis and dispatches it. Its deadzone check only suppresses UNCHANGED small
    // values, so this targets the dispatch gate rather than the deadzone itself. Default off until
    // measured live -- see the note on to_raw() in Plugin.cpp.
    bool  stick_dither = false;

    bool  aim_quat = false;

    // Write the quaternion SOURCE, [obj+0x1D0] -- the field the change-guarded sync copies into the
    // cache. This is the only level where a resync can land on OUR value instead of reverting to the
    // game's, so it is the candidate fix for the aim snapping back when you fire.
    bool  aim_quat_src = false;

    // ---- DIRECT-DRIVE RIG (rigmode 3) ------------------------------------------------------
    // The weapon held rigidly by the controller:
    //     gun_world = R_ctrl * R_trim
    //     origin    = ctrl_pos + R_ctrl*L - gun_world*G
    // which is the same model rigmode 2 uses. What differs is the PARAMETERISATION.
    //
    // Mode 2's single-sample calibration cannot separate L from G -- both rotate with the
    // controller, differing only by the fixed trim, so only (L - trim*G) is identifiable. It
    // therefore fits their sum at one frozen pose and folds the pivot into the mount, which is
    // exact at that pose and drifts everywhere else. That was survivable while the aim crawled
    // behind the hand; with the aim assigned directly you move through the whole pose space at
    // hand speed and the fold is plainly visible as the weapon translating when you only rotated.
    //
    // So mode 3 does not fit those two at all: the pivot is MEASURED from the weapon socket, the
    // mount stays near zero, and only the rotation trim is a matter of taste. Separate keys from
    // mode 2 on purpose -- sharing them would reintroduce the fold the moment either is calibrated.
    float rig_dir_grip_deg  = 0.0f;   // pitch trim: muzzle up (+) / down (-)
    float rig_dir_grip_yaw  = 0.0f;
    float rig_dir_grip_roll = 0.0f;
    float rig_dir_off_x     = 0.0f;   // mount offset, controller-local cm. Should stay SMALL:
    float rig_dir_off_y     = 0.0f;   // a large value here means the pivot is being absorbed
    float rig_dir_off_z     = 0.0f;   // again, which is the failure mode this mode exists to avoid.

    // ---- CONTROLLER SETTINGS APPLIED ONLY WHILE MOTION AIM IS DRIVING ------------------------
    // The aim loop and a human driving a Warthog want opposite things from the same global
    // settings, so the plugin swaps them on the stick-mode transition and puts the player's own
    // values back. Changes are TRANSIENT: the game's apply path does not write to disk (verified),
    // and the save path is never called, so a crash restores their settings by doing nothing.
    //
    // vr_sens is the EBlamLookSensitivity index -- index x 5 is the number the menu shows, so 18 is
    // "LookSensitivity90". 0 = leave the player's sensitivity alone.
    // vr_deadzone is the look dead zone AS A PERCENTAGE (0-100, matching the game's own field --
    // a capture read back 12.0 for a 12% setting, not 0.12). Negative = leave the player's alone.
    //
    // NOTE these do not change the maximum turn rate. Full deflection gives the same ~359 deg/s at
    // any sensitivity (measured); what a higher setting buys is more rate at MID deflection, which
    // is where the loop actually operates.
    int   vr_sens     = 0;
    float vr_deadzone = -1.0f;

    // vr_accel is the EBlamLookAcceleration index. -1 = leave the player's setting alone.
    // 0 = minimum. This MUST be pinned for any plant inversion to be valid: the measured
    // 363.6 deg/s constant was taken at "acceleration min", but the shipped user default is
    // LookAcceleration5, which makes turn rate depend on how long the stick has been held.
    int   vr_accel    = -1;

    // ---- STICK-MODE LOOK COMPENSATION --------------------------------------------------------
    // Motion aim wants the game's controller LOOK sensitivity at maximum and its look deadzone at
    // minimum, for reasons that are structural rather than preference: the aim loop drives a rate
    // actuator, so the game's sensitivity IS the loop's authority ceiling, and with it low the
    // stick saturates on fast sweeps and no amount of tuning can recover. The deadzone sets the
    // SMALLEST correction the loop can make (nothing below `floor` reaches the game at all), so a
    // large one makes fine aim hunt.
    //
    // But those settings are global, and in STICK MODE -- vehicles, cutscenes -- the player's own
    // right stick reaches the game directly. Max sensitivity with no deadzone makes a Warthog
    // camera unusable. The player's preference has to be honoured there and only there.
    //
    // So the game stays configured for the aim loop, and stick mode is compensated in software:
    // scale the player's look stick down, and re-impose a deadzone the game no longer applies.
    //
    // WHY NOT switch the game setting on entering/leaving stick mode: writing the setting object
    // at runtime does NOT reach this game's aim path (verified -- the value stores and persists,
    // and the turn rate does not change; on this Blam hybrid the UE settings object is a UI mirror
    // that only pushes into the sim when the menu applies it). Per-mode switching would therefore
    // need a Blam-side memory write, and would fire on every mount and dismount.
    //
    // stick_scale is a straight multiplier, which is an APPROXIMATION: the stick response curve is
    // steeply exponential, so scaling deflection is not the same as scaling the resulting turn
    // rate. Close enough to tune by feel; a measured inverse curve would be exact.
    float stick_scale = 1.0f;   // 1.0 = passthrough unchanged (safe default)
    float stick_dz    = 0.0f;   // software deadzone re-imposed in stick mode; 0 = none

    // ---- RIG RESOLUTION FAST PATH ----------------------------------------------------------
    // 1 = re-derive the rig through a CACHED first-person weapon actor, falling back to the full
    //     object-array sweep whenever that handle fails to produce a rig.
    // 0 = full sweep every time (the original behaviour, kept for A/B).
    //
    // The invariant from Rig.cpp is preserved either way: the rig is still reached THROUGH a live
    // weapon actor, never adopted by class match, so binding to pooled residue stays structurally
    // impossible. Only the SEARCH for the weapon actor is skipped, and only while the cached
    // handle still resolves.
    //
    // Live-switchable on purpose -- it is the A/B for the stutter, and flipping it in the file
    // beats a rebuild.
    bool  rig_fast        = true;

    // Drive the first-person SHIELD SHELL (BPC_FP_TranslucentSkeletalMesh_C) with the same
    // transform as the arms, so the shield/overshield stays on the hands instead of hanging in
    // space where the arms used to be. See the FP SHIELD SHELL block in Rig.cpp.
    //
    // Live-switchable so it can be A/B'd in the headset without a rebuild, and so it is a kill
    // switch if the shell ever turns out to be wrong for a given pawn.
    bool  shell_drive     = true;

    // ---- WEAPON SCOPE ------------------------------------------------------------------------
    // Left trigger toggles a magnified view on a floating pane; Halo's native zoom stays
    // suppressed (it hides the viewmodel and bends the aim plant -- see Scope.hpp). All of these
    // are live-tunable; the capture only runs while the pane is visible.
    bool  scope_enabled = true;    // master. Also eats LT on foot so Blam never sees it.
    // How the pane is mounted:
    //   1 = RELATIVE to the rig (default). scopedist/right/up are literally the pane's offset
    //       from the controller -- forward / right / up in rig space -- and the facing trims are
    //       a relative rotation. The two are independent, so a rotation cannot move the pane.
    //   0 = the original aim-ray world placement, anchored once. Kept as the escape hatch: it
    //       positions against the RAY rather than the hand, which is a different (and, if a rig's
    //       local axes are unusual, possibly more predictable) mental model.
    int   scope_mount   = 1;
    // Pane shape: 0 = square (Engine Plane), 1 = round lens (Engine Cylinder squashed flat, the
    // uevrlib ocular-lens trick). Round is the scope look; square shows the most image per pixel.
    // Live: changing it rebuilds the pane component.
    int   scope_shape   = 1;
    // SCOPE PLACEMENT CALIBRATION. Hold this key: the pane freezes where it is in the world, so
    // you can move your weapon hand around it until the pane sits where you want it relative to
    // the gun; release captures that offset. Same shape as the End pose-match, and it removes the
    // part that is genuinely hard to hand-tune -- the facing.
    // 0x2E = DELETE. Chosen over HOME because Ctrl+HOME is the kill switch: a stray Ctrl there
    // would stand the aim driver down mid-calibration.
    int   scope_calib_key = 0x2E;
    // True once a scope calibration has been captured, so the calib file only carries a scope
    // block when the gesture actually produced one (same rule as aim_off_valid).
    bool  scope_calib_valid = false;
    float scope_zoom    = 16.0f;    // magnification; pane lens = scope_base_fov / this.
                                    // 16 is the canonical in-headset fit for the default lens
                                    // size -- effectiveness scales with the pane, so this sits
                                    // far above Halo's flat-screen 2x/8x on purpose.
    int   scope_rt_size = 1024;     // render-target edge in px; rebuilt live on change
    int   scope_div     = 1;       // capture every Nth tick (~32 Hz / N) -- the perf valve
    float scope_dist    = 63.57f;   // pane distance along the aim ray, cm (headset-fitted)
    // Where the CAPTURE CAMERA sits along the ray, cm from the origin. It must be FURTHER out
    // than the pane (scope_dist) or it looks straight at the back of the pane and captures a
    // black surface -- the first headset pass hit exactly that. The pane is additionally marked
    // hidden-in-scene-capture in code, so this is the comfort margin, not the only guard.
    // Cost of pushing it out: the view origin leaves the collision-safe camera origin, so very
    // close cover can be seen "through" -- keep the margin modest.
    float scope_cam_dist = 90.0f;
    float scope_size    = 11.84f;   // pane width, cm (the Engine Plane is 100 cm across)
    float scope_right   = 12.92f;    // pane offset right of the ray, cm
    float scope_up      = -0.32f;   // pane offset above the ray, cm (keeps the reticule visible)
    float scope_bright  = 1.0f;    // pane tint multiplier (tonemap compensation headroom)
    // Pane facing trims, deg. rot_p = -90 turns the Plane's +Z face toward the viewer (same
    // convention as aimtexrotp). These move the PANE GEOMETRY only -- the first headset pass
    // confirmed the pane transform is already right, so they ship neutral. The sideways-image
    // fix lives in scope_cam_roll below, at the content layer where the defect is.
    // FITTED IN A HEADSET (2026-08-13), round lens. The earlier -90/0/0 pointed the disc's FRONT
    // face away from the player: the visible side was its back, so the image was mirrored and
    // "moved opposite to my aim". +90 with a 180 roll turns the front face to the eye.
    // PLACEMENT BELOW IS A REAL DELETE-KEY CALIBRATION captured in headset 2026-08-16 and
    // promoted to canonical: mount=1 rig-relative dist/right/up plus these facing trims.
    // They are a matched SET -- a calibration is one fit, so changing one value alone
    // (as the old neutral 90/0/180 trims invited) puts the pane back out of alignment.
    float scope_rot_p   = 79.72f;
    float scope_rot_y   = 173.83f;
    float scope_rot_r   = 176.68f;
    // CONTENT rotation: roll of the capture camera about the aim ray, deg. The first headset
    // pass (and the sim screenshots, in hindsight) showed the pane displaying its texture
    // rotated 90 deg clockwise while the pane itself sat correctly -- so the counter-rotation
    // must be applied to what gets WRITTEN, not to the pane: rolling the capture keeps the RT
    // upright for ANY consumer, and a future non-square pane cannot inherit a sideways spin
    // the way a pane-yaw "fix" would have forced. If a setup still reads rotated, flip the
    // sign (or +/-180) live -- ~2 s reload.
    float scope_cam_roll = -90.0f;   // headset-fitted 2026-08-13, round lens
    // ROLL LOCK. 1 = the captured image rolls WITH the lens, as a real scope does: cant the
    // weapon and the picture stays square in the tube instead of spinning inside it. Measured as
    // a DELTA from the pane's roll when it was placed, so scope_cam_roll keeps meaning exactly
    // what it did and an existing trim survives this change.
    // It also decouples calibration from wrist roll: calibrating with the weapon slightly canted
    // used to bake that cant into the image, because the image was pinned to the WORLD while the
    // lens was pinned to the hand.
    // 0 = the old world-pinned image, kept as an escape hatch.
    // OFF as of 2026-08-16 (player decision): the roll lock "is not behaving right" in headset,
    // so it ships disabled and the image stays world-pinned as it did before the feature. Set 1
    // to re-enable. NOTE the value is a FLAG, 0 or 1 -- a live config was found carrying -53,
    // which reads as simply "on" and was almost certainly meant for scopecamroll (an angle).
    int   scope_cam_lock = 0;
    // Fixed exposure for the capture, applied whenever a POST-PROCESSED source is selected
    // (scope_capture_src 8 or 9). Those sources are what carry bloom -- and therefore the shield
    // shimmer and tracer glow -- but they also enable the capture's own auto-exposure, which
    // measured in-headset as the pane fading to solid black. 0 disables the pin (auto-exposure
    // back on) for anyone who wants to see that behaviour.
    float scope_exposure = 0.0f;
    // scopeautoexposurebias -> FPostProcessSettings::AutoExposureBias, in STOPS (EV). This is
    // exposure COMPENSATION, applied AFTER adaptation, so it is the one brightness control that
    // survives auto-exposure instead of replacing it: the capture still adapts per scene, the
    // whole result just lands N stops brighter. +1 is twice as bright, +2 four times, -1 half.
    //
    // It exists because the alternative was unusable. With scopeexposure=0 the pin does not run,
    // and the pin was the ONLY thing in this plugin that wrote any exposure field -- so an
    // auto-exposed capture had no brightness control at all except scopebright, which is a tint
    // applied AFTER the tonemapper and after 8-bit quantisation. That stretches an already
    // compressed signal rather than feeding the curve higher, which is why it washes out (the
    // measured symptom: a yellow/green cast as bright colours hit the tonemapper's shoulder)
    // long before it makes the FX readable. Bias moves the exposure BEFORE the curve, where the
    // range actually is.
    //
    // Works with the pin ON as well -- pin_capture_exposure() writes this value instead of the 0
    // it used to hardcode, so pinned and auto configurations are tuned by the same key.
    //
    // NOTE the write is one-way per session: setting it back to 0 pins the bias to 0 rather than
    // restoring whatever the capture would otherwise inherit, because the bOverride_ bit stays
    // set once written. Delete the key and restart for a true inherit. 0 = never written.
    float scope_autoexposure_bias = 2.5f;
    // scopelumen -> DynamicGlobalIlluminationMethod + ReflectionMethod on the capture.
    //   -1 = leave alone (DEFAULT, i.e. the engine's forced-off state)
    //    0 = None   1 = Lumen   2 = ScreenSpace   3 = Plugin (GI only)
    //
    // UE turns Lumen OFF for every scene capture -- SceneCaptureRendering.cpp:880-885 sets both
    // methods to None and halves LumenSurfaceCacheResolution, with the comment "By default, Lumen
    // is disabled in scene captures, but can be re-enabled with the post process settings in the
    // component." So the main view renders with Lumen GI and Lumen reflections and the scope does
    // not: same scene, same camera, a DIFFERENT LIGHTING MODEL.
    //
    // Reported in a headset as "it is like the lighting is different for the scene through the
    // scope", which is what losing indirect bounce and specular reflection looks like -- flatter,
    // warmer, and darker specifically where the light was indirect. It had been chased as a colour
    // cast for hours, and it is not one: this is BASE-PASS lighting, upstream of exposure, bloom,
    // tone curve and tint, which is why none of those could ever reach it.
    //
    // 1 (Lumen) is the value that matches the main view on this title. Costs real GPU time -- the
    // capture then runs a second Lumen scene -- so it is opt-in rather than defaulted on.
    int   scope_lumen = -1;
    // scopeppgrade: copy the game's COLOUR GRADE from its camera onto the capture -- the LUT, white
    // balance, the saturation/contrast/gamma/gain/offset sets, and the film curve, each with its
    // paired bOverride_ bit.
    //
    // MEASURED 2026-09-07: the same corridor renders purple-lit in the main view and warm tan in
    // the scope. Not a tint -- a different colour family, i.e. raw albedo with the grade missing.
    // PostProcess VOLUMES reach a capture identically, but a capture never goes through
    // APlayerCameraManager, so a look configured on the CAMERA COMPONENT is invisible to it.
    //
    // Distinct from scopeppcopy, which copies BLENDABLES only. That one originally memcpy'd the
    // whole struct (which would have carried the grade) and was cut back for safety, because
    // FPostProcessSettings contains a TArray and byte-copying it gives two owners one allocation.
    // This copies the grading fields individually -- all POD or one object pointer, never the array.
    bool  scope_pp_grade = false;
    // scopegain -> ColorGain on the capture: a flat brightness multiply in the grading chain.
    // 0 = do not touch (DEFAULT). 1.0 = neutral.
    //
    // This is the COMPOSITOR QUAD's equivalent of scopebright. scopebright tints the in-world
    // pane's MATERIAL, so it does nothing once the layer presents and the mesh is hidden, and the
    // quad has no material to tint. OpenXR's per-layer gain is not available either: extensions
    // must be enabled when the XrInstance is created and UEVR creates it, so the gain has to live
    // in the capture instead.
    //
    // Distinct from scopeautoexposurebias on purpose: bias moves EXPOSURE, so it changes what the
    // tonemapper's shoulder does to highlights; this scales the GRADED colour and leaves the
    // exposure decision alone. Reach for bias first (it uses the curve's range properly) and this
    // only when you want a plain multiply on top.
    float scope_gain = 4.0f;
    // scopelayerfollowpane: place the compositor quad AT the in-world pane's own world transform
    // instead of from scopelayerfwd/right/up/width. 1 = follow (default), 0 = use the offsets.
    //
    // The pane is where every calibration lands -- scopedist/right/up, the per-weapon wpnscope
    // trims, the socket handshake. Duplicating that into a parallel set of layer offsets guarantees
    // the two drift apart the moment anyone calibrates, and the drift is silent. Reading the PLACED
    // component's transform inherits all of it and cannot go stale, because it is the answer rather
    // than a copy of the inputs to it. Falls back to the offsets when the pane's location cannot be
    // read, and says so once in the log.
    int   scope_layer_follow_pane = 1;
    // scopemask: feather the compositor pane's edge into a soft OVAL by writing the atlas cell's
    // alpha with a compute shader. 0 = off (DEFAULT).
    //
    // Read AT ATLAS CREATION TIME, because it needs D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS and
    // that is a creation-time decision -- with this off the atlas is created byte-identically to
    // before, so an unconfigured build carries none of the machinery.
    //
    // WHY A SHADER AND NOT fill_upload(): every other thing the layer draws is CPU-generated into a
    // staging buffer and uploaded (the reticule ring, the markers, the guide beam, GDI text), so
    // authoring art is a solved problem here. The scope pane is different in kind -- its pixels
    // arrive by CopyTextureRegion from the game's render target, GPU to GPU, and CopyTextureRegion
    // cannot copy a single channel. A second quad cannot mask it either: composition layers blend
    // OVER one another, they cannot cut a hole. That leaves a per-pixel write, which is a shader.
    //
    // COUPLED TO SLOT 9'S BLEND FLAG. Slot 9 is submitted OPAQUE because a scene capture's alpha is
    // ~0 and blending by it made the pane vanish except where the captured reticule wrote alpha.
    // This mask is what makes that alpha meaningful, so blending is re-enabled ONLY on frames the
    // mask actually ran (scopemask_applied()). Re-enabling it without the mask reproduces the
    // invisible pane exactly.
    // scoperetproject: place the compositor scope reticule at the PROJECTED impact point instead
    // of dead pane centre. 1 = project (default), 0 = centre (the old behaviour).
    //
    // Centre is only correct while the capture camera's axis and the traced impact point agree.
    // That holds for scopecamtrack=0 at rest and breaks under recoil, sway, or any camera that
    // rides the weapon -- and a reticule that is confidently wrong is worse than one that is
    // obviously approximate, because it is aimed with.
    int   scope_ret_project = 1;
    // RECONCILE THE RETICULE OFFSET'S FRAME WITH THE IMAGE'S. 0 = off (the old behaviour),
    // 1 = on, 2 = on with the opposite sign.
    //
    // The offset that slides the pane reticule is computed in one frame and applied in another,
    // and nothing rotated between them. The projection uses the CAPTURE CAMERA'S right/up, which
    // is correct for naming a pixel -- the image is rendered in that frame -- but the offset is
    // then applied along the compositor QUAD'S local axes. Those two differ by the camera's roll,
    // which is scope_cam_roll + the per-shape uv_roll + THE ROLL LOCK.
    //
    // The roll lock is the part that makes this more than a constant: it moves continuously to keep
    // the image upright as the weapon cants. So the reticule's slide direction was wrong by an angle
    // that CHANGES AS YOU ROLL THE GUN, which is exactly why scoperetflipx/scoperetflipy could never
    // fix it -- a sign flip corrects 180 degrees, not a moving angle.
    //
    // The correction is MEASURED, not assumed: the angle between the capture's up and the quad's up
    // about their shared forward. That makes it immune to the pane's orientation drift, and it
    // self-corrects as the roll lock moves.
    //
    // ONLY AFFECTS A NON-ZERO OFFSET. With xrlayerscopereticle=1 the reticule is pinned at centre
    // (the capture looks AT the aim point, so the projection computes ~0,0), and rotating zero is
    // zero. A/B this on mode 2, the sliding mode, or it is a no-op by construction.
    //
    // Mode 2 exists because the SIGN is not provable from here -- the same admission
    // scoperetflipx/y already makes about the UE->XR mapping. One headset test beats a rebuild.
    int   scope_ret_roll    = 1;
    // Sign flips for the projected offset. UE camera right/up and the quad's local +X/+Y are both
    // "right and up", but nothing guarantees they still agree after the pose has crossed the layer
    // module's UE->XR mapping (which has determinant -1). A mirrored reticule looks plausible until
    // you aim off-axis, so this is config rather than a guess baked into the maths: one edit in a
    // headset instead of a rebuild.
    int   scope_ret_flip_x = 0;
    int   scope_ret_flip_y = 0;
    int   scope_mask = 1;
    // Oval aspect: >1 is WIDER THAN TALL (the X radius stays at the cell's half-width and Y is
    // divided by this). Clamped 0.2..5; anything outside falls back to 1.35.
    float scope_mask_aspect = 1.0f;
    // Edge softness as a fraction of the radius. 0.15 = the outer 15% fades. Clamped 0..1.
    float scope_mask_feather = 0.3f;
    // Force an anti-aliasing method on the CAPTURE only. -1 = leave the game's choice alone
    // (default). 0 None, 1 FXAA, 2 TAA, 3 MSAA, 4 TSR. Exists to test whether the black output
    // from post-processed capture sources is a temporal-upscaler interaction; a non-temporal
    // method is the probe. Applied on change, alongside the capture source.
    int   scope_aa = -1;
    float scope_thresh  = 0.55f;   // LT deflection that fires the toggle (release at half)

    // Research knobs (catalogued in halo_vr_dev.cfg, not shipped in halo_vr.cfg):
    float scope_base_fov    = 70.0f; // pane lens at 1x, deg horizontal
    int   scope_capture_src = 2;     // ESceneCaptureSource byte; 0 = SCS_SceneColorHDR (linear,
                                     // exposure-free -- pairs with the HDR RT + emissive lens;
                                     // 2 = FinalColorLDR was measured orders-of-magnitude dark
                                     // here, the capture's own eye adaptation never converging)
    bool  scope_eat_lt      = true;  // 0 = pass LT through to Blam as well (native-zoom research)
    // Where the CAPTURE CAMERA gets its motion from, once attached to the rig:
    //   0 = re-anchor to the live aim ray every tick (default). The image looks exactly down the
    //       shot line, so the in-pane reticle stays truthful; any residual aim-signal jitter is
    //       visible inside the pane, magnified by the lens.
    //   1 = anchor once and stay rigid, exactly like the pane. Smoothest possible image (pure
    //       controller motion), at the cost of the image drifting off the true shot line as the
    //       aim-vs-rig delta changes.
    // The PANE is rigid in both modes; this only decides what the pane is showing.
    int   scope_cam_track   = 0;
    // Where the capture camera's ORIGIN is, as opposed to its direction:
    //   0 = the aim ray's origin (DEFAULT, and the old behaviour) -- which is THE HEAD. The
    //       SIGHTLINE log's `origin` equals `hmd` every tick, so the scope is effectively a
    //       head-mounted camera aimed down the shot line.
    //   1 = the PANE, i.e. the weapon. The tube then moves with the rifle rather than the skull,
    //       which is what a real optic does.
    //
    // This only became visible when hmdleash was turned off: the leash pins the sightline origin to
    // the standing reference and absorbs the difference, so with it on the two origins agree. With
    // it off, a 24 cm body step displaces the capture 24 cm and the pane shows the ground beside
    // what you are aiming at.
    //
    // Default is 0 because that is the behaviour everything else was calibrated against; 1 is the
    // correct optic and is expected to become the default once it has been flown.
    int   scope_cam_origin  = 1;
    bool  scope_dev_ray     = false; // [dev build] synthesize the scope ray from the rendered view
                                     // -- SimVR-only verification; the null driver never validates
                                     // the controller aim pose, which parks the real ray source
    // ---- DRAW THE PANE IN THE FOREGROUND DEPTH GROUP.
    //
    // The first-person arms and weapon render in SDPG_Foreground so they are never clipped by
    // world geometry. Our pane is an ordinary spawned StaticMeshComponent and therefore sits in
    // SDPG_World, so the arms draw straight over it -- which is what a scope pane must never be
    // behind. Putting the pane in the same group fixes the ordering with no material work.
    //
    // ESceneDepthPriorityGroup: 0 = SDPG_World, 1 = SDPG_Foreground.
    //
    // Live-tunable, and applied ON CHANGE only, because it is a reflected property write.
    // ---- WHAT THE SCOPE PANE HANGS OFF.
    //   0 = the arms rig's ORIGIN (the original behaviour)
    //   1 = the arms rig at socket `PrimaryWeapon` -- the socket the weapon itself rides  [default]
    //   2 = the weapon actor's root component
    //
    // The problem: on the rig ORIGIN the pane does not follow recoil, and sniper recoil then drives
    // the gun straight through the pane.
    //
    // WHY THE SOCKET AND NOT THE WEAPON. A component root does not move when its mesh animates --
    // bones do. Rig.cpp establishes that the first-person arms mesh carries socket `PrimaryWeapon`
    // and the weapon actor is attached there, so parenting the pane to that same socket makes it
    // ride exactly what the gun rides. That is animated (proven the hard way: freezing this socket
    // by hiding the arms is what removed the weapon's recoil in the armhide bug), and it costs no
    // dependency on the weapon actor at all -- a swap, holster, death or vehicle changes which
    // actor sits at the socket and changes nothing about our attachment. Mode 2 keeps the
    // weapon-actor attachment for A/B in-headset; it is not the default because it depends on a
    // pooled actor's lifetime, and pooled actors are recycled under a held pointer.
    //
    // Any weapon-following attach that is refused falls back to mode 0 automatically and says so:
    // a refused attach leaves the component parented to NOTHING, which does not look like a failure
    // -- it looks like a pane hanging in the world that the player can walk away from.
    // WHAT HAPPENS WHEN THE WEAPON BONE NEVER SETTLES. 1 = keep waiting (default), 0 = convert
    // anyway once the deadline passes (the older behaviour).
    //
    // The space-switch conversion is only valid if the bone is near its rest pose when it is taken
    // -- KeepWorld bakes socket^-1 at that instant, and the pane rides socket(t) * rel forever after.
    // A conversion taken mid-animation is wrong PERMANENTLY, and equip is not the only offender:
    // reload has slow phases and firing pauses between shots, so a velocity test alone lets bad
    // moments through.
    //
    // So the deadline no longer FORCES one. If the bone never reaches rest the pane simply stays on
    // the rig, where it is CORRECTLY PLACED and merely does not ride recoil yet. That is a graceful
    // degradation -- the scope works, it is just not weapon-mounted for a moment -- and it is
    // strictly better than the alternative, which is a pane sitting visibly in the wrong place until
    // the player cycles weapons. "Could not verify" must not become "refuse to work", and it does
    // not here: nothing is disabled, one refinement is deferred.
    //
    // Kept as a key rather than hardcoded so the old behaviour is one live edit away if this ever
    // strands a weapon whose bone genuinely never rests.
    int   scope_socket_wait = 1;
    // POST-CONVERSION AUDIT. DEFAULT OFF -- it was unsound, and the log said so.
    //
    // The idea was to catch a first conversion taken against a wrongly-detected rest pose: remember
    // where "rest" was believed to be, and redo the conversion if that belief later moved. The flaw
    // is the proxy. The rest estimate is a slow average of the socket's position, and a RELOAD moves
    // it legitimately -- seconds of off-rest samples drag the average several centimetres without
    // saying anything at all about whether the original conversion was good.
    //
    // MEASURED 2026-09-07, immediately after a reload: "the weapon's rest pose has moved 6.5 cm
    // since the space-switch conversion was taken ... being REDONE". It redid a CORRECT conversion
    // and the pane came back offset -- the audit was the only thing that had changed.
    //
    // So it fires precisely when it must not: after exactly the long animations whose bad moments it
    // was written to protect against. Kept behind a key rather than deleted because the hole it aimed
    // at is real -- a mis-detected first settle IS permanent now -- but the honest closure for that is
    // to compare the pane's ACTUAL placement against the authored one at rest, which measures the
    // outcome instead of a proxy for it. That is a bigger change than belongs near a release.
    int   scope_socket_audit = 0;
    int   scope_parent      = 1;

    bool  scope_fp_depth    = true;
    bool  scope_force       = false; // hold the pane ON without any trigger input. Harness/support
                                     // diagnostic: under SimVR no input path can reach the LT
                                     // hook at all, and this is the config-file automation channel
    int   scope_cap_mode    = 1;     // 0 = manual CaptureScene every scopediv ticks (the perf
                                     // valve); 1 = bCaptureEveryFrame while the pane is shown
                                     // (render-rate captures -- costlier, diagnosis + smoothness A/B)
    // ---- [dev build] TEST OBJECT: does a TRANSLUCENT primitive reach the capture at all? ------
    // The scope is missing shields, cover shields, tracers and the ocean. Every explanation for
    // that is a guess until a primitive whose blend mode WE choose is put in front of the capture
    // camera and looked for in the pane. This spawns exactly that -- a cube parented to the
    // capture, so it is always dead ahead of it -- and the mode picks the material family:
    //   0 = off
    //   1 = Widget3DPassThrough_TRANSLUCENT (authored BLEND_Translucent)
    //   2 = Widget3DPassThrough_OPAQUE      (the CONTROL for 1: same family, opaque)
    //   3 = EmissiveMeshMaterial at BLEND_Additive -- its AUTHORED mode (verified in the engine
    //       asset). Additive is in the translucency family and is what plasma/shield/tracer
    //       effects use, so it probes the actual question more directly than plain translucent.
    //   4 = EmissiveMeshMaterial forced BLEND_Opaque (the CONTROL for 3 -- the pane's own recipe)
    // MEASURED: forcing a blend mode at runtime does NOT recompile shaders. Writing
    // BLEND_Translucent onto EmissiveMeshMaterial (authored ADDITIVE) put the cube in the
    // translucent pass with a shader that writes no alpha, so it vanished from the MAIN VIEW too
    // and measured nothing. Only blend modes whose cooked shader exists are testable. The pane's
    // forced-Opaque write works for the opposite reason: an alpha-blended shader drawn in the
    // opaque pass just ignores the alpha it produces.
    // 1 vs 2 and 3 vs 4 are the A/B. If the opaque cube shows in the pane and the translucent one
    // does not, translucency is being dropped from the capture; if BOTH show, translucency is
    // fine and the missing effects are missing for some other reason.
    int   scope_test_obj    = 0;
    float scope_test_dist   = 300.0f;  // cm ahead of the capture camera (its local +X)
    float scope_test_size   = 0.5f;    // uniform scale on the 100 cm engine Cube
    // WHICH TEXTURE THE PROBE CUBE'S EMISSIVE SAMPLES. This is the difference between an
    // instrument and a black rectangle, and it took three wrong diagnoses to find.
    //
    // The pane and the cube build their material by an IDENTICAL recipe -- same
    // EmissiveMeshMaterial, same BlendMode write, same CreateDynamicMaterialInstance, same
    // "LinearColor"/"SlateUI" binds, same tint. The pane displays its render target perfectly.
    // The cube was black in BOTH views. The only substantive difference was the texture OBJECT:
    //   0 = /Engine/EngineResources/WhiteSquareTexture. Resolves to a live UObject (verified in the
    //       object array), and renders BLACK anyway -- an /Engine/EngineResources asset need not
    //       carry usable cooked texture data in a shipped build. This is the old default and it is
    //       why scopetest 3 and 4 measured nothing, twice.
    //   1 = a solid white render target built at runtime by make_color_rt (CreateRenderTarget2D +
    //       an explicit ClearRenderTarget2D). THE SAME OBJECT TYPE the pane proves samples
    //       correctly, with content we choose. The default.
    //   2 = the scope's OWN render target. Guaranteed to sample -- it is literally what the pane
    //       displays -- but it feeds the capture back into itself, so read it in the MAIN VIEW only
    //       and treat the pane image as meaningless. A last-resort control.
    int   scope_test_tex    = 1;
    // [dev build] SCENE RENDER TARGET PROBE -- the feasibility question for "digital zoom".
    // Instead of re-rendering the scene into a capture (which loses the FX we have never got
    // back), sample the frame the engine ALREADY rendered and magnify a crop of it. UEVR exposes
    // uevr::API::StereoHook::get_scene_render_target() -> FRHITexture2D, and
    // FRHITexture2D::get_native_resource() -> the native D3D12 resource. This logs, once, whether
    // both answer on this title and what the texture actually is (size/format), because the
    // resolution of that surface is the hard ceiling on how far a crop can be magnified before it
    // turns to mush.
    bool  scene_rt_probe    = false;
    // [dev build] IS THE GLOW ACTUALLY IN THE CAPTURE? -- the test that must come before any
    // display-side work. The current hypothesis is that the pane (an in-scene emissive mesh) is
    // destroying contrast via pre-exposure + tonemapping, so effects arrive with the right hue and
    // no halo. That may be true. But moving the DISPLAY to an OpenXR composition layer cannot add
    // a halo the CAPTURE never rendered, so believing it without checking risks a large piece of
    // compositor work that ends with a sharp, correctly-exposed image and still no glow.
    //
    // This reads the render target's own pixels through UKismetRenderingLibrary
    // ReadRenderTargetRawPixel (unnormalised, so HDR values come back un-clamped) along a
    // horizontal line through the middle, and logs the luminance profile. Point the scope at a
    // bright emissive object first.
    //   bright core with values falling off over several samples -> BLOOM IS IN THE RT, the pane
    //     is the problem, and the composition layer is the fix.
    //   bright core with an abrupt edge into darkness      -> no bloom was ever captured; the bug
    //     is upstream and no amount of display work helps.
    bool  scope_rt_scan     = false;
    // ---- DIGITAL ZOOM (ScopeBlit.cpp) ---------------------------------------------------------
    // Magnify a crop of the frame the engine ALREADY rendered, instead of re-rendering the scene
    // into a capture that arrives without its post-processing. Bloom, tonemapping and every other
    // effect are correct by construction because it is the real main-view image, and there is no
    // second scene pass at all.
    // STAGE 1: a screen-space quad at a fixed position -- NOT yet locked to the gun. Off by
    // default, and it disables itself permanently on the first failure rather than retrying on a
    // render callback.
    // ---- MAKE THE CAPTURE POST-PROCESS LIKE THE MAIN VIEW DOES --------------------------------
    // The symptom that started this: FX arrive in the pane with the RIGHT HUE and NO GLOW. Bloom
    // is post-processing, and a capture only runs post at all on a final-colour source
    // (scopesrc=8). But running post is not enough on its own -- the capture carries its OWN
    // FPostProcessSettings, freshly defaulted by the component, NOT the player camera's. If bloom
    // is off or unset there, a post-processed capture still renders everything except the glow.
    // BOTH of these need scopesrc=8 to mean anything.
    //
    // scopebloom: force BloomIntensity (and drop the threshold so dim effects still bloom) onto
    //   the capture, with the paired bOverride_ bits. 0 = leave the capture's bloom alone.
    // ---- MAKE THE CAPTURE PART OF THE MAIN VIEW FAMILY ---------------------------------------
    // UE 5.5 has properties for exactly the question "why does this capture not render like the
    // main view does". bRenderInMainRenderer is the strongest of them and is USELESS to us --
    // ShouldRenderInMainRenderer() restricts it to depth/basecolor/normal sources, never colour.
    // But these three carry NO capture-source restriction and have never been tried:
    //
    // scopemainfamily -> bMainViewFamily. "Render with main view family (bIsMainViewFamily ==
    //   true)". Rendering features that skip non-main families would then run for our capture.
    //   The cheapest of the three and the one to try first.
    int   scope_main_family = 1;   // -1 = leave alone, 0/1 = force
    // scopemainres -> bMainViewResolution. Renders at the MAIN VIEW's resolution, ignoring the
    //   render target's own dimensions, and implies main view family. Costs more, but would also
    //   hand the pane a far sharper image than scoperes ever could.
    int   scope_main_res    = -1;
    // scopemaincam -> bMainViewCamera. Renders from the MAIN CAMERA, which DESTROYS the whole
    //   point of the scope (it would show where the head looks, not where the gun points). It is
    //   here purely as a CONTROL: if the capture suddenly renders correctly with the main camera,
    //   the path works and the difference is something about our own camera setup. Do not ship it.
    int   scope_main_cam    = 0;
    float scope_bloom     = 0.0f;
    // scopeppcopy: copy an ENTIRE FPostProcessSettings struct onto the capture -- every field and
    //   every bOverride_ flag at once -- from the game's own source, so nothing is missed by
    //   guessing field names. Prefers an unbound PostProcessVolume (where a shipped UE game
    //   usually keeps bloom), falling back to a camera component. Logs which it used.
    bool  scope_pp_copy   = true;
    bool  scope_blit      = false;
    // Magnification. Blam reports 2.00 for the magnum via GetZoomMagnification (measured in
    // headset), so this is what per-weapon zoom will eventually be driven from.
    float scope_blit_mag  = 2.0f;
    // Quad size as a fraction of frame HEIGHT, and its centre within each eye's half, normalised.
    float scope_blit_size = 0.35f;
    float scope_blit_x    = 0.5f;
    float scope_blit_y    = 0.5f;

    // ---- [dev build] THE BLACK-FINAL-COLOUR LEVERS --------------------------------------------
    // The whole scope-FX question reduces to one unanswered thing: scene-colour sources render but
    // have post-processing FORCED OFF by the engine (SceneCaptureRendering.cpp:853), so anything
    // whose look is produced in post never arrives; final-colour sources keep post but come out
    // SOLID BLACK on this title. These are per-capture properties that plausibly cause that black
    // and have never been tested. Each was verified as a real Blueprint-accessible UPROPERTY in
    // the UE 5.5 source before being wired here, so a null result means "not the cause" rather
    // than "the write went nowhere".
    //
    // bAlwaysPersistRenderingState. We force it TRUE so exposure/TAA state survives between manual
    // captures and a low-rate scope holds a steady image. But that persistent state IS a temporal
    // history, and the reported symptom -- "an image I can't quite recognize, then fades to
    // complete black" -- is the signature of a history converging to black, not of a frame that
    // renders black. 0 drops it. Costs the steady image: a diagnosis knob, not a comfort one.
    int   scope_persist    = 1;
    // bCameraCutThisFrame, written EVERY tick. The renderer resets it to false after each capture
    // (SceneCaptureRendering.cpp:1410), so a one-shot write measures nothing. A camera cut
    // invalidates the temporal history every frame -- the same hypothesis as scopepersist from the
    // other side, and the side that KEEPS the persistent state the exposure pin needs.
    int   scope_cam_cut    = 0;
    // PostProcessBlendWeight. At 0 the capture's own PostProcessSettings do not blend in at all,
    // so if the black survives, nothing WE wrote into those settings caused it -- which is the
    // control the exposure pin never had. <0 leaves the engine default untouched.
    float scope_pp_weight  = -1.0f;

    // ---- DEPTH OF FIELD ON THE CAPTURE: TRIED, DID NOT WORK ----------------------------------
    // MEASURED 2026-08-16: applied cleanly ("capture DOF -- 2 of 2 fields applied") with
    // scopesrc=8 and scopepersist=0, and the shields' colour STILL did not appear. Kept because
    // the write works and the lever may be useful for something else; it is NOT the fix.
    //
    // The reasoning that produced it was WRONG, recorded here so it is not rebuilt. The shield
    // census reports the material's TranslucencyPass as 1, which was decoded as
    // TPT_TranslucencyAfterDOFModulate -- a value of ETranslucencyPass, the RENDERER'S INTERNAL
    // enum. The field is EMaterialTranslucencyPass (Material.h:140), whose members are
    // MTP_BeforeDOF=0, MTP_AfterDOF=1, MTP_AfterMotionBlur=2. So 1 means plain AFTER DOF, and the
    // separate MODULATE buffer that the whole argument rested on is not involved at all -- the
    // modulate pass is not selectable from a material, it is derived from BLEND_Modulate, and this
    // material is Additive. Decoding a reflected byte against a same-named enum from a different
    // header is an easy and completely silent mistake.
    //
    // ONLY MEANINGFUL WITH A POST-PROCESSED SOURCE (scopesrc=8): a scene-colour capture skips the
    // post chain entirely, so the DOF pass cannot run there whatever these say. A HIGH f-stop runs
    // the pass with a tiny aperture, so the image stays essentially in focus.
    float scope_dof        = 0.0f;        // f-stop; 0 = leave DOF alone
    float scope_dof_focus  = 100000.0f;   // focal distance, cm -- far, so nothing blurs

    // ---- THE SCOPE AS AN OpenXR QUAD COMPOSITION LAYER (scopelayer*) ---------------------------
    //
    // Present the scope's render target through the COMPOSITOR instead of on the in-world pane, as
    // a quad mounted on the gun. Submitted after the whole post chain, so scene pre-exposure, the
    // tonemapper and TAA/TSR never touch the one surface in this mod whose whole job is to be read.
    // Mechanism, the pose construction and the current blockers: src\ScopeLayer.hpp.
    //
    // EXPERIMENTAL AND DEFAULT OFF. With scopelayer=0 nothing in this family runs at all -- one
    // int test per tick -- and the in-world pane behaves exactly as it always has.
    //
    //   0  off (default)
    //   1  gun-mounted: the feature. Orientation comes from the aim ray.
    //   2  [dev build] SELF-CHECK. Feed the layer the finished VIEW's own forward and up instead of
    //      the aim ray's. The layer's UE->XR basis map then converts to identity, so the quad MUST
    //      render exactly head-locked. If our vectors are in a frame we did not think they were,
    //      this fails totally and obviously rather than as the near-zero-on-axis error a handedness
    //      mistake produces. Hold your head LEVEL: the view roll is not published, so this arm
    //      assumes roll 0. On a release build 2 and 3 fall back to 1 and say so once.
    //   3  [dev build] CONTROL for arm 2: publish no orientation at all, which IS head-oriented.
    //      2 and 3 must be indistinguishable. That is the test.
    int   scope_layer       = 1;
    // Which compositor slot the scope takes. 0 is the reticule, 1..8 are the navpoint markers, and
    // 9 is XRLAYER_SLOT_PANE -- allocated for exactly this, with its cell sized by
    // xrlayer_pane_configure(). The literal is here rather than the constant because Config.hpp
    // must not include another module's header; ScopeLayer.cpp static_asserts that the two agree,
    // so they cannot drift in silence. Pointing this anywhere else lands on a marker-sized cell and
    // the source is refused -- which the log says, in numbers.
    int   scope_layer_slot  = 9;
    // GEOMETRY. Halo-MCC-VR's shipped gun-mounted zoom screen, transcribed from
    // docs\GameRecon\MCC-VR-Study.md:232: width 0.159 m, right -0.058 m, up 0.216 m, forward
    // 0.050 m. Field-proven numbers from a shipping mod rather than guesses -- but READ ALL THREE
    // CAVEATS before treating them as fitted here:
    //   1. CENTIMETRES, because every distance key in this depot is (matching Unreal's own unit).
    //      These are UE cm, so the PHYSICAL size is width / (100 x VR_WorldScale) -- at this game's
    //      1.312 that is 0.121 m, ~24% under MCC's 0.159 m. Multiply by the world scale to match
    //      their physical fit exactly. The tick logs both numbers side by side with MCC's, so this
    //      never has to be worked out from memory again.
    //   2. THEIR OFFSETS ARE CONTROLLER-LOCAL; OURS ARE IN THE AIM FRAME. Deliberate: this scope's
    //      premise is that it shows where the shot goes, not where the gun model points, so the
    //      quad hangs off the same ray the capture camera uses.
    //   3. OUR OWN PANE IS FITTED SOMEWHERE QUITE DIFFERENT -- scopedist 63.6 / scoperight 12.9 /
    //      scopeup -0.3 cm, rig-relative. MCC's placement floats a small screen just above the
    //      hand; ours sits well out in front. Expect an in-headset fit to move these a long way.
    float scope_layer_width = 30.0f;
    float scope_layer_fwd   = 64.0f;
    float scope_layer_right = 13.0f;
    float scope_layer_up    = 0.0f;
    // Does the quad ROLL with the weapon? 1 = yes (MCC's construction: the lens's own up vector,
    // perpendicularised against the aim axis, so cant rides the gun and no Euler decomposition is
    // involved). 0 = world-levelled, which never cants.
    //
    // PAIR IT WITH scopecamlock. That key decides whether the captured IMAGE rolls with the lens,
    // and it ships at 0 (world-pinned). scopelayerroll=1 with scopecamlock=0 rolls the screen while
    // leaving the picture level, so canting the weapon appears to counter-rotate the image inside
    // the quad. Set both to 1, or both to 0.
    // 0 = world-upright (level, ignores the weapon's cant)
    // 1 = the lens's up, so the image cants with the weapon (default)
    // 2 = the PANE's own up -- the quad inherits the pane's FULL orientation, facing and roll.
    //     Use this when the quad's image is rotated relative to the in-world pane: the capture
    //     bakes its roll compensation for the PANE's frame, so a quad rolled differently shows that
    //     content rotated by the difference. Measured 2026-09-07 -- the facing was already correct
    //     (pane forward 0.968 against the aim), which left roll as the only remaining term.
    // Degrees, applied about the quad's OWN facing axis, after whatever scopelayerroll chose.
    //
    // The quad and the mesh pane show the same render target, but the mesh displays it through its
    // UVs and the quad does not -- and the round lens's UVs are rotated 90 degrees, which is why
    // scope_cam_roll is -90 against a uv_roll of +90 (that pair cancels for the MESH only). A quad
    // has no UV stage, so this difference is not recoverable by inheriting the pane's basis; it has
    // to be trimmed, exactly as scope_cam_roll itself was fitted in a headset.
    // Try +/-90 first if the quad's image is square to the mesh's but rotated.
    float scope_layer_roll_trim = 0.0f;
    // Degrees, applied about the quad's OWN facing axis, after whatever scopelayerroll chose.
    //
    // The quad and the mesh pane show the same render target, but the mesh displays it through its
    // UVs and the quad does not -- and the round lens's UVs are rotated 90 degrees, which is why
    // scope_cam_roll is -90 against a uv_roll of +90 (that pair cancels for the MESH only). A quad
    // has no UV stage, so this difference is not recoverable by inheriting the pane's basis; it has
    // to be trimmed, exactly as scope_cam_roll itself was fitted in a headset.
    // Try +/-90 first if the quad's image is square to the mesh's but rotated.
    int   scope_layer_roll  = 0;
    // 1 = hide the in-world pane and let the quad be the scope, instead of drawing both.
    //
    // Gated on the layer actually PRESENTING -- our source accepted and xrlayer_slot_ready() true --
    // never on "we asked it to". That distinction is the reticule lane's expensive lesson: a
    // refused layer must never leave the player looking at nothing.
    int   scope_layer_hide_pane = 1;

    // ---- MELEE BY SWING --------------------------------------------------------------------
    // Swing the aim hand and the game melees. The detector runs on the GAME THREAD tick and
    // publishes a deadline; the XInput hook only compares a clock against it (see Gesture.hpp),
    // which is the same produce-at-tick / consume-at-poll split the rest of this plugin uses.
    //
    // WHY LINEAR SPEED AND NOT ANGULAR. The obvious false positive is a fast aim turn: the hand
    // is moving quickly but the player means to look, not hit. A turn is mostly ROTATION about
    // the wrist/elbow with little travel, while a strike is mostly TRANSLATION. So the trigger
    // is metres-per-second of controller travel, not deg/s -- g_setpoint_rate_dps already
    // measures the angular rate and is deliberately NOT what gates this.
    //
    // Poses are in METRES (see xdist_m, added directly to a pose position in MotionAimControl),
    // so melee_speed is genuinely m/s and not an abstract unit.
    bool  melee_swing     = true;

    // ---- WHAT COUNTS AS A STRIKE: ARM EXTENSION, NOT SPEED.
    //
    // The first version gated on world-space hand speed plus |dot(vel, hmd_fwd)| and fired on fast
    // aiming. Measured, the reason is structural: when you turn, your HEAD TURNS WITH YOUR HAND,
    // so hand velocity stays aligned with the view forward it is being compared against. Logged
    // false positives sat at along=0.46-0.57 -- squarely inside a gate meant to exclude them.
    //
    // Head-to-hand DISTANCE has none of that failure mode, because it is a scalar and therefore
    // invariant to rotation:
    //   * a thrust      -- distance grows fast, and ends large
    //   * an aim turn   -- the hand orbits the body at roughly constant radius, so ~0
    //   * walking       -- head and hand travel together, so ~0
    // Measuring the hand RELATIVE TO THE HEAD also removes whole-body motion for free.
    //
    // All distances in metres (poses are metres -- see xdist_m).

    // Minimum RELATIVE speed, m/s: hand velocity with the head's own motion subtracted out. A
    // floor, not the discriminator -- melee_ext below is what actually distinguishes a strike.
    float melee_speed     = 1.50f;

    // Rate the arm must be EXTENDING at, m/s -- d/dt of head-to-hand distance. THE test, and
    // measurement says it is a very clean one. Over a logged session, ordinary play motion topped
    // out at ext=0.80 while deliberate swings ran 2.24 to 4.51 -- a gap with nothing in it. 1.50
    // sits in that gap, comfortably clear of both sides.
    float melee_ext       = 1.50f;

    // How far the hand must be from the head when it fires, metres. A floor, not the test.
    //
    // WHY IT IS LOW. Reach is evaluated the instant extension crosses the threshold, which is
    // MID-SWING while the arm is still travelling -- not at full stretch. Real swings measured
    // 0.37 to 0.57 at that moment despite finishing much further out, and a 0.50 floor rejected
    // three of five genuine strikes. This only needs to exclude a twitch made with the hand
    // tucked against the chest.
    float melee_reach     = 0.30f;

    // Sanity ceiling on relative speed, m/s. Sessions logged 26 m/s and 97 m/s "swings", which no
    // arm produces -- those are tracking discontinuities. Above this the velocity history is
    // dropped rather than fired on, because the sample after a teleport is garbage too.
    float melee_max_speed = 12.0f;

    // Sanity ceiling on reach, metres. After the 97 m/s spike above, head-to-hand distance read
    // 2.45-2.51 m for several consecutive samples -- the velocity guard caught the jump but the
    // POSITION stayed wrong, so a position check is needed as well as a rate one. No human arm is
    // this long; anything beyond it means tracking is lying about where the hand is.
    float melee_max_reach = 1.20f;

    // Legacy forward-alignment gate, |dot(vel_dir, hmd_fwd)|, 0..1. DEFAULTS OFF: it is the check
    // that proved unsound above. Kept because it is harmless when zero and someone may want it.
    float melee_fwd       = 0.0f;

    // Velocity smoothing time constant, ms. Tracking noise at 90 Hz is enough to spike a raw
    // per-tick derivative; this is short enough not to blunt a real strike's leading edge.
    float melee_tau_ms    = 20.0f;

    // Refractory period after a fired melee, ms. Halo's melee animation is not interruptible, so
    // a second trigger inside it is always spurious -- one swing crossing the threshold on
    // several consecutive ticks must still be ONE press.
    int   melee_cooldown_ms = 500;

    // ---- DIRECTIONAL MELEE (blindcowboy24 PR-1) ----------------------------------------------
    // Halo lunges along your AIM, and during a swing your aim is the flailing hand -- measured at
    // a median 83 deg of travel by the time the hit lands, which is why stock-butt strikes used to
    // whiff at a target you were looking straight at. So the strike is aimed along the SWING.
    //
    // 1 = aim the strike along the swing direction (shipped). 0 = leave it where you were looking.
    int   melee_aim_mode  = 1;
    // How long the aim is pinned to the swing direction, then how long it blends back to the live
    // hand. The ramp exists so the reticule RETURNS rather than teleports: the hand has travelled a
    // long way by then, and an instant handback is a visible snap in the opposite direction.
    int   melee_aim_hold_ms = 250;
    int   melee_aim_ramp_ms = 150;

    // How long the synthetic button is held, ms. A single poll can land between the game's own
    // input samples and be missed entirely, so the press is held across several.
    int   melee_hold_ms   = 80;

    // Pad mask ORed in to melee. 0x0080 = RTHUMB, which is this profile's melee (right stick
    // click). Confirm against your own mapping with mapbtnlog=1 before changing it.
    int   melee_mask      = 0x0080;

    // Log every swing's peak speed and whether it fired. This is the tuning instrument for
    // melee_speed and melee_fwd -- expect to set it once, swing a dozen times, and turn it off.
    bool  melee_log       = false;

    // ---- TWO-ARM RECON ---------------------------------------------------------------------
    // Dump the first-person skeleton -- every bone with its parent -- and report which bone
    // functions this build actually exposes. Read-only; nothing is written to the game.
    //
    // Fires on the RISING EDGE, so leaving it set does not re-dump every config reload. Set it
    // while standing in gameplay with a weapon drawn: the rig does not exist in menus, in
    // vehicles, or for ~5 s after a level load.
    //
    // This is the instrument that decides whether independent arms are buildable at all. See
    // Arms.hpp for what the answer gates.
    bool  bone_dump       = false;

    // ---- LEFT ARM HIDE ---------------------------------------------------------------------
    // Hide the left arm chain, so an independently tracked left hand can replace it.
    //
    // The bone dump established that this is possible: HideBoneByName and UnHideBoneByName are
    // both PRESENT, and Shoulder_L is the single root of all 34 left bones (hiding a bone hides
    // its children). Weapon_M hangs off Chest_M rather than either wrist, so the gun is not
    // dragged along with the arm.
    //
    // WHAT THIS IS FOR RIGHT NOW. Present in the reflection table is not the same as working on
    // this mesh -- Rig.cpp documents a relative-rotation write that is present and silently does
    // nothing, which is the whole reason rigmode 3 exists. So this toggle exists first as PROOF,
    // before anything is built on top of it. Fully reversible: setting it back to 0 unhides.
    bool  arm_hide        = false;

    // WHICH ARM DRIVER RUNS -- exactly one, ever. See ArmDriver.hpp for why this is one enum and
    // not two independent switches.
    //
    //   0 = off      the game's stock first-person rig, untouched
    //   1 = UeRig    Rig.cpp + Arms.cpp + Hands.cpp -- UE reflection, the route this project built
    //   2 = Palette  src\palettearm\ -- the Blam node palette, ported from elliotttate's project
    //
    // DEFAULT 1: the shipped behaviour is exactly what it was before the palette route existed.
    // Mode 2 has NEVER been verified against a running game -- its offset chain was measured on
    // someone else's copy of the simulation DLL -- so it is opt-in, and it exists to be A/B'd in a
    // headset against mode 1. Switching is live: the arbiter tears the outgoing driver down before
    // the incoming one gets a frame, so it is safe to flip mid-session while wearing the headset.
    int   arm_driver      = 1;

    // ---- TWO-HANDED AIMING (src\TwoHandAim.hpp) --------------------------------------------
    //
    // Support hand on the barrel, squeeze its grip: aim eases onto the line between your hands,
    // roll still taken from the aim hand. Independent of armdriver -- it is an aim feature, and
    // it runs in every arm-driver mode including 0.
    //
    // DEFAULT OFF for the release that introduces it. Nothing here has been watched working in a
    // headset; turn it on deliberately, and only flip this default once a session has confirmed
    // it. Live-reloaded like every tunable, so it is an A/B you can do without leaving the game.
    // ONE-SHOT PALETTE DUMP (dev builds only). Rising edge logs all 76 first-person node matrices
    // from a single frame, then disarms itself.
    //
    // This is the measurement that settles "wrong bone indices" vs "wrong palette pointer" -- the
    // two failures that look identical from outside, and the reason armdriver=2 has burned several
    // headset sessions. Node discovery reporting hands-found-but-no-arm-above-them says the memory
    // is structured; only the actual numbers say what it is structured AS.
    //
    // 76 log lines from inside the render hook is a deliberate one-frame stall. It is compiled out
    // of release builds entirely and disarms after one frame, so it cannot reach a player.
    bool  pa_dump         = false;

    // SMOKE TEST for the palette write (dev builds only). Metres of upward displacement applied to
    // the ENTIRE first-person node set, before any arm solving.
    //
    // WHY A DELIBERATE ABSURDITY. The drive currently reports success on every single call --
    // "drive stage=DRIVING ok=9992" against "hook 9992 calls" -- and the arms still do not move.
    // That is either (a) we are writing a palette nothing renders, or (b) we are writing the right
    // one and the pose we compute happens to look like the stock pose. Subtle changes cannot tell
    // those apart. Half a metre straight up can: if the weapon and hands do not visibly leap, the
    // memory we are writing is not what the renderer reads, and no amount of IK tuning will help.
    //
    // 0 disables. 0.5 is the recommended value -- unmissable, and still on screen.
    float pa_test_lift    = 0.0f;

    // WHICH ARMS THE PALETTE ROUTE POSES. Bit 1 = aim hand, bit 2 = support hand. Default 3 = both.
    //
    // paarms=2 is the experiment worth running: leave the AIM arm completely stock and pose only
    // the support hand. The stock aim hand is already authored ONTO the weapon, and the weapon has
    // its own calibrated driver -- so not touching it may be the correct way to make it "ride the
    // gun", rather than computing a target for it and hoping the frames agree.
    int   pa_arms         = 3;

    // WHICH FRAME THE SHOULDERS HANG OFF. This is the uncertain one, so it is switchable rather
    // than guessed:
    //   0 = the palette root as-is. The rig is already expressed in camera-local space (measured:
    //       root sits at (0,0,0) every frame), so this adds nothing and may be correct by default.
    //   1 = root composed with the HEAD. Intended to stop the shoulders following the AIM, but if
    //       palette space is already view-relative it double-counts the head and the arms rotate
    //       EXTRA as you yaw -- which is what was observed.
    //   2 = root composed with the inverse HEAD, i.e. actively removing head yaw.
    int   pa_torso_frame  = 6;

    // TORSO YAW BLEND (patorsoframe=6). 1 = face where the HEAD faces. 0 = face where the HANDS
    // are (the midpoint of both controllers, or the single tracked one). Anything between blends
    // the two along the shortest arc.
    //
    // Both terms are measured RELATIVE TO THE CAMERA via the recenter composition, so when the aim
    // drive yaws the camera by t both terms counter-rotate by -t and the torso stays world-fixed.
    // That makes this construction aim-independent BY CONSTRUCTION rather than by subtracting a
    // correction -- which is why it needs no view-lock term, unlike modes 3/4/5.
    float pa_head_shoulders_yaw_influence = 0.5f;

    // PALETTE WEAPON DRIVE -- carry the weapon branch (nodes 7, 8, 22) onto the aim controller with
    // ONE rigid transform, so the stock animation inside the branch survives.
    //
    // This is route (c): it stays entirely inside the Blam palette, where the arms are already
    // posed, and never crosses the UE/Blam boundary. Every defect in the wpndrive lane lived on
    // that boundary -- the socket recomputing our write, the cross-thread feedback race, the stale
    // pooled actor, the per-eye divergence. None of them can exist here, because there is no socket
    // in the composition and no UE component being fought over.
    //
    // Independently arrived at by blindcowboy24's PR #8 (BlamPalette.cpp: apply_weapon_branch, the
    // same nodes {7,8,22}, the same one-rigid-delta shape) -- which is corroboration that the
    // branch and the method are right, not a port. His notes are worth heeding on two points: he
    // deliberately removed all trim knobs from this path ("hand-tunable knobs on a solved frame are
    // how six coupled wrongs impersonate one right"), and he bounds every written node to arm's
    // reach after a crash where a sleeping controller read (0,0,0) and produced a 2.4 m hand.
    //
    // DEFAULT OFF. Requires armdriver=2. Suppresses the legacy mesh drive while it owns the weapon,
    // because two writers on one gun is the fight that has cost this project several sessions.
    bool  pa_weapon       = false;

    // WEAPON GRIP TRIM for the palette path, DEGREES, applied in the CONTROLLER's frame.
    //
    // The palette carry puts the AUTHORED marker (node 8) straight onto the controller, so whatever
    // fixed rotation the artist baked between that marker and the barrel is currently discarded --
    // reported in-headset as the weapon "yawed counterclockwise", and consistent with the socket
    // measuring a 90 deg yaw (sock rot=(p-0.0 y90.1 r0.3)).
    //
    // THIS IS A MEASUREMENT STEP, NOT A CALIBRATION, and it should not survive. The existing
    // grip_deg/grip_yaw/grip_roll already encode the hand->gun rotation the player tuned, but they
    // are a UE ROTATOR and this path works in Blam's forward/left/up basis; composing them needs a
    // convention conversion that is worth deriving rather than guessing. So: find the value that
    // looks right here, compare it against the calibrated grip yaw, and if they agree the mapping is
    // confirmed and this knob gets replaced by reading the calibration directly.
    //
    // blindcowboy24's warning applies and is the reason for that plan: "hand-tunable knobs on a
    // solved frame are how six coupled wrongs impersonate one right."
    // THE LOCK-GAP LIFT. 1 = on (correct), 0 = off (the old double-counting behaviour, kept only
    // as an escape hatch if the sign is ever wrong on another build).
    //
    // The hand pose is published in the ROOM frame, which the view lock pins to the VIEW's yaw.
    // The palette is CAMERA-local. Lifting the hand by the full camera yaw therefore counts the aim
    // twice, because our aim drive writes the camera's yaw FROM that same controller -- reported
    // in-headset as "the weapon moves extra, like there's a multiplier on the rotation".
    //
    // blindcowboy24 hit this and measured it (BlamPalette.cpp:1496): "The correct lift is
    // (camera - view): the room hand already sits at view yaw, so only the difference between the
    // two frames is missing. NO HEAD TERM. NO VIEW TERM. The camera's yaw, and nothing else." He
    // also records that two stacked corrections here were BOTH wrong and a controlled sweep caught
    // it using the arms as ground truth -- so this applies exactly ONE term.
    //
    // (camera - view) is the negation of g_view_lock_delta, which is published as (view - camera).
    int   pa_wpn_lift     = 1;

    // THE SAME LOCK-GAP LIFT, FOR THE HANDS. 1 = on, 0 = the old double-counting behaviour.
    //
    // The wrist target is built from the controller pose composed through the recenter offset --
    // stage-relative and constant when the hand is still -- and then lifted by the FULL camera.
    // But our aim drive WRITES that camera yaw from this same controller, so yawing the controller
    // by t moves the hand target by 2t in the world. Identical to the weapon bug fixed by
    // pa_wpn_lift; the arms were simply never given the same treatment.
    //
    // This is what was left of "the shoulders move with my aim" after the torso frame was fixed:
    // the shoulder anchor was correct, but the HAND dragged the whole arm around with the aim, and
    // the hand is what the eye follows.
    int   pa_arm_lift     = 1;
    // PITCH half of the arm lift. pa_arm_lift removes the game camera YAW from the arm frame;
    // nothing removed its PITCH. Measured 2026-08-30: the game camera pitches 1:1 with the aim
    // (PlayerCameraManager rotation.x -56.56 at aimpitch -56.6, -14.82 at -14.8) while the HMD sits
    // level, and UEVR decoupled pitch keeps that out of the VIEW -- so a hand placed correctly in
    // the pitched camera frame swings by the whole aim pitch in ROOM space. Pitch-only, because the
    // yaw is already compensated and there is no aim roll. 0 = off, 1 = +gap, 2 = -gap (handedness
    // is settled by measurement, exactly as the yaw gap was).
    // MEASURED 2026-08-30 in sim, left controller pinned in room space: sweeping the aim across
    // 74 deg, (sup XZ angle + aimpitch) held to 0.7 deg at mode 2, where at mode 0 it varied by the
    // full 74 deg -- i.e. mode 2 leaves the hand ROOM-FIXED. Mode 1 rotated the wrong way, doubling
    // the error. |sup| stayed constant throughout, confirming a pure rotation.
    int   pa_arm_pitch    = 2;

    // HANDS-ONLY (Half-Life Alyx style): collapse everything outside the two hands to invisible,
    // keeping the full IK solve intact. 0 = full arms, 1 = hands only. Live-tunable.
    //
    // A post-solve FILTER, not a second code path -- the arm is still solved, so aim, the two-hand
    // hold and the weapon carry cannot drift apart between the two modes. Lifted from pancreations
    // MCC VR floating_hands, which ships this ON by default. The weapon nodes stay in the keep set,
    // so the gun does not vanish with the arm.
    //
    // NOTE this is NOT the same thing as ArmSolve place_wrist_subtree ("no IK"): skipping the solve
    // would make hands-only a second behaviour to keep in sync forever.
    // Gates ONLY the arm rest-pose lift (the rotation about the SHOULDER), not the wrist-target
    // lift. 1 = on (the behaviour to date), 0 = off.
    //
    // The two lifts apply the SAME gap about DIFFERENT pivots -- target about the head, rest pose
    // about the shoulder -- so they disagree by roughly (pivot separation) x gap, about 22 cm at
    // the 47 degree gaps measured in play. The IK bridges that by folding the arm. Measured:
    // per-window bend peak vs lockdelta peak correlates 0.941.
    //
    // Separate from pa_arm_lift on purpose: that one gates BOTH halves, and turning it off brings
    // back the arms-follow-the-aim problem this whole lane exists to fix.
    // Which frame the WRIST TARGET is built in. 0 = live camera basis times the live lock gap
    // (behaviour to date), 1 = the levelled TORSO frame.
    //
    // Mode 0 swings the target by the whole lock gap, and that gap reaches 98-125 degrees on this
    // title. Measured with the support controller PINNED: hand excursions to 101 cm correlating with
    // the gap at 1.000, and 10.8 cm of forearm STRETCH because the wrist is placed exactly even when
    // the arm cannot reach. Mode 1 lets the gap enter the arm once, via a frame that is already
    // gap-corrected and levelled, and makes the target share a frame with the rest-pose lift.
    //
    // pancreations MCC VR go further: their wrist target uses NO live camera orientation at all,
    // only piecewise-constant references. Mode 1 is the nearest thing that fits our direct-drive
    // camera without rewriting the aim architecture.
    // DEFAULT 1 as of 2026-08-31. Measured, aim controller sweeping, support controller PINNED:
    //   mode 0 -> hand excursion 102.4 cm peak, forearm stretched 9.2 cm
    //   mode 1 -> hand excursion  0.002 cm peak, forearm stretch 0.001 cm
    // Verified it is not simply switched off: 324 posed frames, zero bails, and the hand still
    // tracks the controller (gotL follows leftZ monotonically and returns exactly on the way back).
    // NOT yet confirmed in a headset -- patgtframe=0 restores the old behaviour live, no restart.
    // 2 additionally puts the wrist ORIENTATION in the torso frame. Mode 1 pinned the hand POSITION
    // to 0.002 cm but left its facing swinging 49.9 deg per frame, because the orientation still
    // composed the live camera basis with the gapped controller. Mode 2 is NOT the default because
    // the wrist convention is learned against root_basis, so it risks a constant mis-facing -- which
    // is an offset, not jitter, and therefore invisible to every headless check here.
    // DEFAULT 2. Measured with the aim sweeping and the support controller PINNED:
    //   mode 0 -> hand 102.4 cm peak, forearm stretched 9.2 cm, correlates with the gap at 1.000
    //   mode 1 -> hand  0.002 cm, but facing still swung 61.5 deg per frame
    //   mode 2 -> hand  0.002 cm AND facing 0.000 deg
    // Not switched off: 333 posed frames, zero bails, and the hand still tracks its controller.
    // FIRST THING TO CHECK IN A HEADSET: whether the support hand FACES correctly. A constant
    // mis-facing is an offset rather than jitter, so nothing measured here could have caught it.
    // patgtframe=1 reverts the orientation half live; 0 reverts both.
    int   pa_target_frame  = 2;

    // Does the SUPPORT hand ride the weapon while two-handing? 0 = no (default), 1 = yes.
    //
    // 0 follows pancreations MCC VR, who never attach it: the support arm is solved onto its own
    // controller and the hands couple ONE WAY, through the aim basis only. Attaching it makes the
    // hand inherit everything the weapon inherits, including the aim hand rotation, which is why
    // "grabbing" kept surfacing as a separate complaint from the arm itself.
    int   pa_grab_weapon   = 0;

    int   pa_arm_rest_lift = 1;

    int   pa_hands_only   = 0;

    float pa_wpn_yaw      = -90.0f;
    float pa_wpn_pitch    = 0.0f;
    float pa_wpn_roll     = 0.0f;

    // ---- THE BARREL LOCK. One shared aim direction: the rendered gun points where it shoots.
    //
    // WHAT IT DOES, in one sentence: the rigid transform that carries the weapon branch onto the
    // controller is constrained so it cannot move the AIM AXIS -- it may slide the gun and roll it
    // about the ray, and nothing else. Ported from blindcowboy24's PR #1
    // (BlamPalette.cpp:1574, `palettebarrellock`, which he ships ON).
    //
    // WHY IT IS WANTED. pa_wpn_yaw/pitch/roll and the per-weapon wpnfix are STATIC trims: they can
    // be right for one wrist angle on one weapon. Everything that moves the gun off the reticle
    // dynamically survives them -- the aim drive's own filtering lag, the dead zone, ffpitchcomp,
    // gain adaptation, and whatever each weapon's authored marker does differently from the last.
    // The lock removes all of it every frame, which is why "it felt better across weapons" cannot
    // be explained by his per-weapon file (one non-identity entry in it).
    //
    // THE COARSE/FINE SPLIT, and it is the whole reason this composes rather than competes:
    // this is the COARSE alignment and wpnfix is the FINE trim riding on top. Our lock is applied
    // AFTER both trims (his is applied before his), so an existing wpnfix keeps working exactly as
    // it did -- it still points the barrel at the ray, and the lock only removes what it missed.
    // Reversing that order would make every captured wpnfix a double correction. See PaletteArm.cpp.
    //
    // WHERE THE AIM AXIS COMES FROM, and this is what had to be adapted rather than transplanted.
    // He reads the game's ControlRotation on the sim thread and builds a WORLD aim ray, because his
    // pose lives in world. Ours never leaves the palette: the palette ROOT *is* the game camera (see
    // the torso-frame note above), so the aim ray in the frame the weapon basis lives in is just the
    // camera's own forward -- +X, one term, no camera read, no thread bridge, no staleness. Both
    // builds run aimdirect=1, so in both cases this constrains the model to a ray we are already
    // steering; it is not a second aim source and it writes nothing back into aim.
    //
    // WHERE THE BARREL DIRECTION COMES FROM -- the one empirical assumption, worth stating plainly:
    // THE GAME'S OWN FIRST-PERSON POSE POINTS THE GUN DOWN THE CAMERA FORWARD. That is what makes
    // hipfire work in the stock game, and it means the barrel's direction in the marker bone's own
    // frame is readable straight off the stock pose we are about to overwrite -- same frame, same
    // instant, no measurement machinery at all. He instead reads the posed socket's world rotation
    // back through reflection and EMAs it over still frames; that needs a game-thread reflection
    // call, a cross-thread publish and an outlier filter that (in his build) can never adapt to a
    // weapon whose barrel differs from the running mean by more than 10 degrees. Ours adapts per
    // weapon for free and costs three vector operations.
    //
    // AND IT KEEPS RECOIL. Because the barrel direction is taken from the live stock pose rather
    // than a latched constant, a stock pose that is deliberately off the ray -- recoil, sway, the
    // reload animation -- stays off the ray by exactly the angle the animation asked for.
    //
    // ⚠️ pa_wpn_yaw IS STILL REQUIRED, and is NOT made redundant by this. The trim's job is to map
    // the camera frame onto the ARTIST'S bone frame (a ~90 degree structural rotation on this
    // title); the lock's job is the few degrees left over. With the trim at 0 the residual is far
    // outside pa_barrel_release below and the lock simply stands down -- it can never supply a
    // structural 90.
    //
    // DEFAULT OFF, deliberately, and this is the honest reason: the "stock pose points at the
    // crosshair" assumption above has not been checked in a headset on THIS build, and Config.hpp
    // defaults are the shipped configuration. Turn it on in halo_vr_dev.cfg, confirm, and only then
    // move the default -- which is itself a settings change worth a release note.
    bool  pa_barrel_lock  = false;

    // HOW FAR OFF THE RAY THE LOCK STILL CORRECTS IN FULL, degrees. Above this it fades out to
    // nothing at pa_barrel_release, so during a fast flick -- where the camera legitimately lags the
    // hand -- the gun goes back to following the hand instead of being pinned to a stale ray.
    //
    // 25 is his engagement threshold, kept so the two builds compare. His is a HARD cap, i.e. a
    // 25-degree snap of the model at the boundary, which is precisely where a flick lives; setting
    // pa_barrel_release to 0 (or anything <= this) reproduces that exactly for an A/B.
    float pa_barrel_full    = 25.0f;
    float pa_barrel_release = 40.0f;

    // CANONICALISED 2026-08-31: two-handed aiming ON, matching what the user plays with.
    int   two_hand        = 1;

    // WHICH BUTTON engages the hold. 0 = the OpenXR grip ACTION on the support hand.
    //
    // Non-zero = read that XInput pad mask instead (0x0100 = LEFT_SHOULDER, which is this
    // profile's off-hand grip -- the same bit reloadgrip uses).
    //
    // WHY THE ALTERNATIVE EXISTS. The OpenXR action path is used by exactly two things in this
    // plugin: this feature and the vehicle hard brake. Everything else -- melee, reload, the whole
    // gesture stack -- reads the XInput mask, and that path is PROVEN on this profile while the
    // per-hand action path is not. If the hold never latches, the status line says
    // `griphandle=1 gripever=0`, and this key is the way to move to the proven path without
    // waiting for a fix.
    //
    // Note the mask route cannot tell your hands apart -- it sees a button, not a side. That is
    // acceptable here because the ZONE already requires the hand to be at the barrel.
    int   bind_two_hand   = 0;

    // Weapons that must NEVER two-hand, as comma-separated SUBSTRINGS of the weapon class name
    // (same matching as wpnoff -- "Magnum" matches BP_Magnum_WeaponActor_C).
    //
    // A one-handed weapon held with a second hand on a barrel it does not have is not a feature,
    // it is the aim wandering off for no reason the player can see. Halo's one-handers are the
    // pistols, the Needler and the sword.
    //
    // ⚠️ THESE NAMES ARE UNVERIFIED against this title's actual class names -- they are the
    // obvious spellings, not measured ones. Check them with a weapon in hand (the per-weapon
    // offset log prints the class name) before trusting the defaults. A wrong entry fails in the
    // safe direction -- the weapon simply is not denied -- which is exactly why it needs checking
    // rather than assuming.
    //
    // Substrings are chosen not to collide: "PlasmaRifle" rather than "Rifle", which would also
    // deny the Assault Rifle.
    char  two_hand_deny[256] = "Magnum,PlasmaPistol,PlasmaRifle,Needler,Sword";

    // THE NEAR EDGE OF THE GRAB ZONE FOR ONE-HANDED WEAPONS, in metres along the aim ray.
    //
    // The rifle zone starts 8 cm FORWARD of the firing grip, because that is where a foregrip is.
    // A pistol has no foregrip: you cup it AT or just under your firing hand, which is `along`
    // around zero or slightly negative -- so the most natural two-handed pistol hold was the exact
    // position that could never register.
    //
    // Negative means "behind the firing grip", i.e. cupping under it. -0.10 covers a hand wrapped
    // around the base of the grip without reaching so far back that a hand resting at your side
    // starts latching. The FAR edge is unchanged: a hand a long way down a barrel that is not there
    // should still not latch.
    //
    // Only applies to weapons matched by two_hand_deny -- the same list that withholds aim
    // authority. Those two now travel together: a one-hander grips (for zoom) but never bends aim.
    float two_hand_onehand_min_m = -0.10f;

    // NO twohandhidereticule KEY YET, deliberately. elliotttate hides his floating reticule during
    // a hold because it is drawn from the one-handed ray and therefore lies; ours is drawn
    // downstream of the blend and stays truthful, so there is no bug to fix. If a player does want
    // it hidden there are THREE surfaces -- the hosted crosshair, the ring mesh and the compositor
    // reticule -- and hiding one of the three is worse than hiding none. A key that parses and
    // half-works is the silent no-op this project keeps paying for, so it lands when all three do.

    // ---- dev only: which consumer receives the blend.
    //
    // Both default ON and the shipped answer is that they are always both on. Setting
    // twohandrig=1 twohandaim=0 reproduces the "gun points where the shots do not go" bug ON
    // PURPOSE -- it is the only way to demonstrate in a headset that the two really are moving
    // together, which is the single most important property of this feature.
    bool  two_hand_aim    = true;
    bool  two_hand_rig    = true;
    bool  two_hand_log    = false;

    // Which bone to hide. Shoulder_L takes the whole arm; Elbow_L leaves the upper arm in place
    // and takes forearm downwards; Wrist_L takes just the hand. Configurable because which one
    // looks right is a judgement to make in the headset, not from a bone list.
    char  arm_hide_bone[64] = "Shoulder_L";

    // HOW to hide. HideBoneByName reported success on all three meshes and changed nothing
    // visible, so per-bone hiding may simply be inert here -- the same silent no-op Rig.cpp
    // records for the relative rotation write.
    //
    //   0 = HideBoneByName            per-bone, keeps the right arm. What we want if it works.
    //   1 = SetVisibility(false)      whole component. Takes BOTH arms.
    //   2 = SetHiddenInGame(true)     whole component, different path to the same thing.
    //
    // Modes 1 and 2 are blunt, but they are the route to genuinely independent hands: with
    // SetBoneTransformByName absent we cannot pose the game's arms, so the only way to two free
    // arms is to remove the game's pair and attach our own to the controllers. Both pass
    // bPropagateToChildren=false, so the weapon actor socket-attached to this mesh stays put.
    int   arm_hide_mode   = 2;

    // Apply to every first-person skeletal mesh, not just the arms.
    //
    // ON BY DEFAULT because the dump found THREE that carry this pose:
    // BPC_FP_SkeletalMesh_C, BPC_FP_TranslucentSkeletalMesh_C (the shield shell, which Rig.cpp
    // records as running its own instance of the same anim blueprint) and
    // BPC_FP_ShadowSkeletalMesh_C. Hiding the bone on the arms alone leaves a floating shield
    // limb and an arm-shaped shadow. Set 0 only to isolate which mesh is which while testing.
    bool  arm_hide_all    = true;

    // Force hidden skeletal meshes to keep evaluating their pose.
    //
    // UE defaults to OnlyTickPoseWhenRendered, so a hidden mesh can stop animating entirely --
    // and Rig.cpp positions the weapon from this mesh's PrimaryWeapon SOCKET. A frozen pose means
    // a frozen socket and a gun parked wherever the animation stopped, appearing intermittently
    // depending on when the hide lands relative to the rig resolving.
    //
    // Set 0 to test whether the arm hide is what is moving your weapon.
    bool  arm_keep_pose   = true;

    // ---- PER-WEAPON OFFSETS ------------------------------------------------------------------
    // One calibration cannot fit a pistol, an assault rifle and a rocket launcher: they do not
    // share a grip geometry. These adjust the calibrated base per weapon, so the pose-match
    // calibration still does the work and an entry is only needed where a weapon disagrees.
    //
    // A weapon with no entry behaves exactly as it does today, so enabling this changes nothing
    // until something is tuned.
    //
    // SHIPS ON (2026-08-23). Safe to default on precisely because of the line above: the base is
    // applied by ASSIGNMENT and a weapon with no entry lands exactly on the calibration, so with
    // an empty table this is a no-op. What it buys by being on is that the INSERT capture key is
    // live for everyone -- a player who finds one weapon sitting wrong can fix that weapon alone,
    // without a hidden setting to discover first.
    //
    // Cost measured in-session: weapon_offset 0.004 ms mean / 0.064 ms max against a 0.2-0.5 ms
    // tick. It is not a full-array sweep -- one class-name read on the held weapon per tick.
    bool  wpn_offsets     = true;

    // SOCKET-CANCELLING WEAPON DRIVE -- see WeaponDrive.hpp for the algebra.
    //
    // 0 (default) = legacy: controller tracking is delivered by placing the arm mesh, which per
    // Rig.hpp:15 carries ARMS + GUN together and so parents the shoulders to the aim controller.
    // 1 = drive the weapon root's relative transform instead and leave the mesh undriven, so the
    // shoulders can be body-anchored by palettearm while the gun keeps tracking the hand.
    //
    // DEFAULTS OFF ON PURPOSE. An unflagged build behaves exactly as it always has; this is the
    // structural half of "the legacy setup does not break", the other half being that the
    // calibration (C) is passed through untouched so values tuned under either path are valid
    // under both. The drive self-checks against the legacy result every frame and hands the gun
    // back if it ever diverges, so leaving it on is safe even when it cannot solve.
    bool  wpn_drive       = false;

    // Per-weapon CAPTURE key, a Windows virtual-key code. Same gesture as the global calibration
    // on calib_key (END): hold it, the weapon freezes, line your controller up with it, release.
    // Only the destination differs -- this one stores a delta for the weapon in hand instead of
    // rewriting the global calibration.
    //   HOME=0x24  INSERT=0x2D  DELETE=0x2E  PGUP=0x21  PGDN=0x22   (0 disables)
    //
    // ⚠️ INSERT, NOT HOME. PR #7 shipped this as 0x24, which is kill_key -- the mod's KILL
    // SWITCH. Both are polled with GetAsyncKeyState in the same block, so HOME would have
    // captured a weapon delta AND toggled the kill switch on the same press. The collision was
    // invisible in the fork rather than introduced by the rebase: kill_key was already 0x24 on
    // the branch point too, so the clash shipped in PR #7 as authored and simply went unchecked.
    // Keep new hotkeys off the reserved set: 0x21 mode, 0x22 aim-calib, 0x23 calib,
    // 0x24 KILL, 0x2E scope-calib.
    int   wpn_calib_key   = 0x2D;
    bool  wpn_calib_pending = false;

    // The calibrated base, captured by WeaponOffset on each config reload. Published here so the
    // capture can measure a DELTA against it: g_cfg's live values already carry this weapon's
    // existing adjustment, so differencing against those would shrink toward zero on every
    // recalibration.
    float wpn_base_grip = 0.0f, wpn_base_grip_yaw = 0.0f, wpn_base_grip_roll = 0.0f;
    float wpn_base_off_x = 0.0f, wpn_base_off_y = 0.0f, wpn_base_off_z = 0.0f;
    bool  wpn_log         = false;
    WeaponAdjust wpn[kMaxWeaponAdjust];
    int   wpn_count       = 0;

    // ---- PER-WEAPON SCOPE TRIMS (ScopeOffset.hpp) -------------------------------------------
    // Zoom and pane placement as DELTAS on the global scope fit, so one calibration still does
    // the work and an entry is only needed where a weapon disagrees. A weapon with no entry lands
    // exactly on the global fit, which is why this can ship ON: with an empty table it is a no-op.
    //
    // Trims the IN-SCENE PANE path (Scope.cpp). The compositor layer (ScopeLayer.cpp,
    // scope_layer_*) is a second placement system with its own frame and units; when it ships it
    // wants its own delta set rather than this one overloaded onto it.
    bool  scope_offsets   = true;
    ScopeAdjust wpn_scope[kMaxScopeAdjust];
    int   scope_count     = 0;
    // Log which weapon matched which trim, on every weapon CHANGE. Off by default; this is the
    // instrument for "is the trim being applied at all", answerable without a capture.
    bool  scope_wpn_log   = false;

    // The global fit as the FILE supplied it, republished by ScopeOffset every reload. The capture
    // measures its delta against THESE, not against the live g_cfg.scope_* -- which already carry
    // the current weapon trim, and would therefore fold the old trim into the new one.
    float scope_base_zoom  = 0.0f;
    float scope_base_dist  = 0.0f, scope_base_right = 0.0f, scope_base_up = 0.0f;
    float scope_base_rot_p = 0.0f, scope_base_rot_y = 0.0f, scope_base_rot_r = 0.0f;

    // ---- PER-WEAPON RIGID DELTA FOR THE PALETTE CARRY (wpnfix) -------------------------------
    // TWO TIERS, ONE TABLE: the shipped baseline in halo_vr.cfg and the player's own captures in
    // halo_vr_weapons.cfg, merged in load order and resolved by weapon_fix_for() above -- read its
    // precedence note before touching either half.
    //
    // Captured by the same INSERT gesture as wpnoff above; which of the two a press writes is
    // decided by WHICH DRIVER OWNS THE WEAPON, not by a second hotkey. Under the palette driver
    // (armdriver=2 + pawpn=1) the rig's grip/mount fit is not in the chain at all, so a wpnoff delta
    // would adjust nothing a player can see; under the rig driver the reverse. One key, one meaning:
    // "fix the weapon in my hand".
    //
    // NO STRUCT DEFAULTS, AND THAT IS THE RULE WORKING RATHER THAN AN EXCEPTION TO IT. These are
    // CALIBRATION DATA, not settings: their home is halo_vr.cfg, the one shipped file that still
    // carries values, exactly as grip/offx/pivx do. An empty table is a valid state -- every weapon
    // then lands where the plain palette carry puts it.
    WeaponFix wpnfix[kMaxWeaponAdjust];
    int   wpnfix_count    = 0;
    // How many wpnfix lines were DROPPED this load for carrying the wrong schema stamp. Reported
    // once per reload rather than per tick: a silently ignored calibration file is indistinguishable
    // from a feature that does not work, and that is the report nobody can act on.
    int   wpnfix_dropped  = 0;

    // ---- VR RELOAD -------------------------------------------------------------------------
    // Two-stage reload: press reload to drop the mag, then physically fetch a fresh one from your
    // belt and bring it to the gun. See the state machine in Gesture.hpp for why the game's own
    // reload is DEFERRED rather than driven -- Halo's reload is one atomic animation with no
    // magazine object to manipulate, so the physicality has to come from making you earn it.
    bool  reload_vr       = false;

    // Pad mask that STARTS the reload. Default 0x4000 = X, this game's reload.
    //
    // UNCONFIRMED -- verify with mapbtnlog=1 before trusting it. This profile's mapping is not
    // obvious (mapfrom=0x2000 -> mapto=0x0100, mapmenuback=0x4000), and a wrong mask here means
    // the reload either never starts or hijacks a button you needed.
    int   reload_mask     = 0x4000;

    // Pad mask for the left GRIP -- what you hold to keep hold of the magazine. Also unconfirmed.
    int   reload_grip_mask = 0x0100;

    // BELT ZONE, measured from the head because that is the only body reference VR gives us.
    // Hand must be at least this far BELOW head height, in metres.
    float reload_belt_drop = 0.55f;

    // ...and within this horizontal radius of the head, so a hand dropped straight down at your
    // side counts but one flung out sideways does not.
    float reload_belt_radius = 0.50f;

    // INSERT: how close the left hand must come to the aim hand to seat the magazine, metres.
    // Hand-to-hand rather than hand-to-weapon: the gun is a separate actor whose grip point moves
    // per weapon, while the two controllers are always both known.
    float reload_join_dist = 0.30f;

    // Swallow the trigger while the magazine is out. THIS is what gives the gesture stakes -- you
    // are genuinely defenceless until you finish. Off means the reload is cosmetic.
    bool  reload_suppress_fire = true;

    // Pressing reload again while the mag is out re-seats it and aborts.
    //
    // OFF, because that is not how VR reloading works. Onward, Pavlov, H3VR: once the magazine is
    // out you deal with it -- there is no take-backs button, because a cancel is a menu concept
    // and this is a physical action. Leaving it on also meant a double-tap silently skipped the
    // entire gesture, which reads as an exploit even though no ammo is gained by it.
    //
    // It was originally on as a safety hatch against a stuck MAG_OUT leaving the trigger dead in
    // a headset. Six-for-six on the first live session made that argument much weaker, and the
    // involuntary cases are covered anyway: the kill switch, stick mode (vehicles, cutscenes,
    // death) and calibration all reset to Idle and release the suppression. What is NOT covered
    // is a gesture the player simply cannot complete -- an awkward seating position, tracking
    // loss on the off hand -- and the only exit there is enabled=0 in the file.
    bool  reload_cancel   = false;

    // WATCHDOG on an unfinishable gesture. Seconds in MAG_OUT/MAG_HELD before the machine gives up
    // and returns to Idle. 0 disables it.
    //
    // This closes the hole the comment above names. A tap that never becomes a completed reload --
    // awkward seating, tracking loss on the off hand, or simply changing your mind mid-firefight --
    // used to latch MAG_OUT forever, and with reload_suppress_fire on that means bRightTrigger is
    // zeroed on every poll for the rest of the session. Reported from a live session as "I wasn't
    // able to shoot after a bit", with the tell that a MOUSE click still fired: mouse input never
    // passes through our XInput hook, so it is the one path the suppression cannot reach.
    //
    // Deliberately NOT a cancel-and-reload: it restores the trigger and says so in the log, and the
    // player reloads again if they still want to. Firing a reload the player did not ask for, in a
    // firefight, seconds after they stopped gesturing, would be its own bug.
    //
    // 6 s is several times the ~1-2 s a completed gesture takes, so it cannot cut short a reload
    // that is merely slow.
    float reload_timeout_s = 6.0f;

    // The left grip belongs to US, not to the game.
    //
    // It is the VR interaction button -- magazine grabs now, weapon holding later -- and a button
    // that throws a grenade when you reach for something cannot carry physical interactions. With
    // this on the game never sees the grip at all, which means GRENADES HAVE NO BINDING until one
    // is given to them elsewhere. That is a deliberate trade, not an oversight.
    //
    // Set 0 for the narrower behaviour: grip swallowed only while a magazine is expected or held.
    // Only meaningful with reloadvr=1, and only when gripswallow is OFF -- otherwise the grip is
    // already unbound outright and this window is moot.
    bool  grip_exclusive  = true;

    // THE LEFT GRIP NEVER REACHES THE GAME. A plain unbind, owned by nothing else.
    //
    // Its native action on this game is Throw Grenade, and the grip is our VR interaction button
    // -- two-handed aiming now, magazine grabs and weapon holding later -- so every grab threw a
    // frag. Grenades are not lost: left X is rebound onto the throw mask by mapfrom/mapto, and
    // that rebind runs AFTER this swallow, so only the physical grip is removed.
    //
    // STANDALONE ON PURPOSE. This was first implemented by routing through the reload lane's
    // grip_exclusive, so switching off an unrelated feature silently gave the grip back to the
    // game and the grenade came with it. A binding must not be a side effect of another lane's
    // flag. Set 0 only if you actually want the grip to throw grenades again.
    bool  grip_swallow    = true;

    // THE OFF-HAND TRIGGER IS MODAL, on whether you are gripping the weapon.
    //   gripping     -> zoom toggle, and this is the ONLY way to zoom
    //   not gripping -> throw grenade
    // Releasing the grip also closes the zoom, because letting go is an unambiguous "done".
    //
    // The grip is a mode the player can feel, so one button carries both jobs without ambiguity.
    // It is also why two_hand_deny no longer refuses the grip on one-handed weapons -- it now
    // withholds only the AIM BLEND. A Magnum you cannot grip is a Magnum you cannot zoom, and the
    // one-handers are exactly the weapons with a zoom; the interaction is the same on everything.
    //
    // Deliberate consequence: while gripping you have no grenade. Let go, throw, re-grip.
    //
    // 0 restores the old split: trigger always toggles zoom, grenade lives on whatever
    // grenadefrom/mapfrom point at.
    bool  grip_zoom       = true;

    // GRENADE, moved off the grip.
    //
    // With grip_exclusive the game never sees the grip, so grenades need a home. This is a pure
    // remap: press the button named by grenade_from, and the plugin injects grenade_action in its
    // place.
    //
    // BOTH MASKS ARE UNCONFIRMED. Controller buttons and XInput masks do not line up on this
    // profile -- the right controller B reports as 0x4000, which the game labels X -- so these
    // must be measured with mapbtnlog=1 rather than reasoned about. grenade_from should be
    // whatever your LEFT controller X reports; grenade_action should be whatever the grip used to
    // report, since that is the mask the game already reads as throw.
    //
    // MEASURED 2026-09-04 with mapbtnlog=1 on this profile, not guessed:
    //   right thumbstick click -> 0x0080
    //   left controller X      -> 0x2000   (XInput calls it B; the labels do not line up here)
    //
    // GRENADE IS THE OFF-HAND TRIGGER, so this remap is OFF. 0 = no button remap; the throw comes
    // from the modal LT path in Plugin.cpp (trigger throws when you are not two-hand gripping,
    // zooms when you are), which is the designed input.
    //
    // IT WAS 0x2000 -- LEFT X -- AND THAT DOUBLE-BOUND THE BUTTON. map_from is also 0x2000, so one
    // press of left X did both jobs: the rebind stripped 0x2000 and injected equipment, and then
    // this remap injected the throw as well, because it reads raw_btn (the pre-injection snapshot)
    // and the strip therefore never hid the press from it. Equipment AND a grenade, every time.
    //
    // That is worth understanding rather than just fixing, because raw_btn is CORRECT here and the
    // collision is not its fault. Reading the live state instead would make our own injected masks
    // satisfy the remap -- the melee swing would throw grenades. raw_btn deliberately sees the
    // physical press regardless of what any earlier stage did with it, which is exactly what makes
    // two features pointed at one physical button both fire. The lesson is that a raw_btn consumer
    // does not participate in "first match wins": it cannot be disarmed by an upstream strip, so
    // its source mask must be unique by construction. Check that before pointing another one at a
    // button that already has a job.
    //
    // Set grenade_from to a mask to put the throw back on a button. Do not reuse 0x2000 (equipment
    // via map_from), 0x0080 (melee_mask -- costs button-melee entirely, leaving only the swing) or
    // map_rstick_down's mask (every crouch would throw).
    int   grenade_from    = 0x0000;
    int   grenade_action  = 0x0100;

    // How long the reload button must be held before it stops being a reload and becomes the
    // game's own action, milliseconds.
    //
    // Both buttons this feature borrows already have jobs. The reload button is ALSO Interact and
    // Enter Vehicle; a full playthrough with it swallowed unconditionally meant no vehicles and no
    // interaction for the whole chapter. Tap starts the VR reload, hold passes through -- and past
    // the threshold the press is released to the game, so it arrives slightly late rather than
    // never.
    int   reload_hold_ms  = 500;

    // Log every state transition. The tuning instrument for the belt zone and join distance.
    bool  reload_log      = false;

    // ---- OUR OWN HANDS -----------------------------------------------------------------------
    // With the game's first-person meshes hidden there is nothing on screen but a floating gun.
    // These are the replacement: spawned StaticMeshComponents attached to the motion controllers
    // through UObjectHook, the same mechanism Rig.cpp uses for the weapon.
    //
    // Spawned rather than borrowed because the bone dump closed every other route:
    // SetBoneTransformByName is ABSENT, so the game's hand bones cannot be posed, and both arms
    // share one mesh driven by one anim blueprint.
    bool  hands_vr        = false;

    // Mesh for a hand. Empty falls back to an engine primitive (a small sphere), which is a
    // placeholder and looks like one -- Halo ships no standalone hand asset we can borrow.
    char  hand_mesh_path[192] = "";

    // Uniform scale. The engine sphere is 100 cm radius, hence the very small default.
    float hand_scale      = 0.06f;

    // Offset from the controller, centimetres, in the controller's own frame. The tracked point
    // sits behind and below where a real palm is, so a hand mesh placed at the raw pose floats
    // off the wrist.
    float hand_off_x      = 0.0f;
    float hand_off_y      = 0.0f;
    float hand_off_z      = 0.0f;

    // Show a mesh on the AIM hand too. Off by default: the weapon already tracks that controller,
    // so a second object there mostly intersects the gun.
    bool  hand_show_aim   = false;

    // ---- THE MAGAZINE ------------------------------------------------------------------------
    // Visible only while the reload state machine has one in your hand. Hidden rather than
    // destroyed between reloads -- spawning a component per reload would churn objects on the
    // game thread for nothing, and a component we keep is one we can still clean up.
    bool  mag_show        = true;
    char  mag_mesh_path[192] = "";
    float mag_scale       = 0.04f;
    float mag_off_x       = 0.0f;
    float mag_off_y       = 0.0f;
    float mag_off_z       = 0.0f;
};

extern Config g_cfg;
extern char     g_cfg_path[MAX_PATH];
// The user's own settings (halo_vr_user.cfg) -- never shipped, survives updates. The catalog of
// available keys is the shipped halo_vr_user_reference.txt, which is documentation, not parsed.
extern char     g_user_cfg_path[MAX_PATH];
// Dev/troubleshooting overrides (halo_vr_dev.cfg) -- shipped all-commented, beats the user file.
extern char     g_dev_cfg_path[MAX_PATH];
extern char     g_calib_path[MAX_PATH];
extern char     g_calib_path_right[MAX_PATH];
// Result of select_calib_for_hand(). Returned rather than logged because this file is kept free
// of UEVR API calls (see the header comment) -- the caller does the reporting.
enum { CALIB_HAND_RIGHT = 0, CALIB_HAND_LEFT_LOADED = 1, CALIB_HAND_LEFT_SEEDED = 2 };
int select_calib_for_hand();
extern uint32_t g_cfg_check_tick;

// Incremented ONLY by a load_config() that parsed the main file successfully and therefore
// restored the calibrated values from disk.
//
// g_cfg_check_tick is NOT a substitute: it advances on every ~2 s attempt, including the early
// return taken when halo_vr.cfg fails to parse (a momentary file lock while editing is enough).
// Anything that needs to know "g_cfg now holds clean on-disk values" must watch THIS, because on
// the early-return path it does not.
extern uint32_t g_cfg_load_gen;
extern bool     g_pivot_from_calib;
// TRUE when an aim offset loaded WITHOUT an explicit aimcalibver stamp, so its schema is inferred
// rather than known. Set by load_config(); the tick reports it once (this file makes no API calls).
extern bool     g_calib_stamp_ambiguous;

void write_default_config();
bool parse_config_file(const char* path);
void load_config();
void write_calib_file();
// Create the halo_vr_user.cfg template if absent (exclusive create; never touches an existing file).
void ensure_user_cfg_template();
// TRUE if the file exists and contains at least one active (uncommented) key line.
bool config_file_has_uncommented_keys(const char* path);
// Same, but NAMES the keys: fills a comma-separated list and returns the count. A warning that a
// shared file "has uncommented keys" without saying which is nearly useless -- see Config.cpp.
int  config_file_list_uncommented_keys(const char* path, char* out, size_t out_sz);

// Keyboard gestures poll GetAsyncKeyState, which reads GLOBAL key state -- keys typed into ANY
// app register, game focused or not. On a machine that keeps working while the game runs, End /
// Page Down / Delete / Ctrl+Home / Ctrl+PgUp are ordinary editing keys, and a background session
// silently ran calibrations on them and toggled the kill switch ("something keeps recreating my
// calibration file", 2026-08-15: four garbage aim calibrations in ten seconds of desktop
// scrolling). Every keyboard gesture therefore requires the game to OWN THE FOREGROUND WINDOW.
// The in-headset workflow is unaffected -- a played game is the foreground window -- and the
// menu-armed calibration path never needed the keyboard at all.
inline bool game_window_focused() {
    DWORD pid = 0;
    GetWindowThreadProcessId(GetForegroundWindow(), &pid);
    return pid != 0 && pid == GetCurrentProcessId();
}

// ---- In-game settings-menu bridge (see Config.cpp). The menu script's file access is
// sandboxed to <profile>\data\, so the plugin mirrors the catalogs + live files there and
// applies the script's command file back into the real cfg files on the ~2 s poll.
extern char g_data_dir[MAX_PATH];
extern char g_user_ref_path[MAX_PATH];
extern char g_menu_cmd_path[MAX_PATH];
extern char g_user_mirror_path[MAX_PATH];
extern char g_ref_mirror_path[MAX_PATH];
extern char g_dev_mirror_path[MAX_PATH];
extern char g_calib_mirror_path[MAX_PATH];
extern char g_status_path[MAX_PATH];
// Menu-armed calibration mode: 0 = off, 1 = pose-match armed, 2 = aim-ray armed, 3 = SUPPORT HAND
// armed. Written by the bridge (calib:pose / calib:aim / calib:hand / calib:off commands) and by
// the trigger edges in Plugin.cpp; the update() gesture block treats an armed mode as a held
// calibration key.
extern std::atomic<int> g_menu_calib_mode;

// The SUPPORT-HAND gesture's hold, published by the same game-thread block that derives the pose
// and aim holds -- because the left-trigger "save and re-arm" half of the gesture is only visible
// there (g_menu_calib_lt is TU-local to Plugin.cpp, sampled inside the XInput hook).
//
// READ IT GATED ON THE MODE, never alone: `mode == 3 && g_hand_calib_held`. The mode is the
// authoritative half -- it is what the right trigger and the menu's Cancel clear -- so gating on it
// means a publisher that stops running (an update() early-out, a driver release) can never leave a
// freeze latched with the hand pinned in mid-air.
extern std::atomic<bool> g_hand_calib_held;

// ---- BIND CAPTURE ("press the button you want").
//
// Same shape as the armed calibration above, and for the same reason: with UEVR's overlay open
// the VR mod zeroes the pad upstream, so NO controller input reaches this plugin while the menu
// is on screen. A listen-for-a-press widget inside the menu is therefore impossible -- the menu
// can only ARM, and the player closes it and presses the button.
//
// Ownership is split so no lock is needed. The BRIDGE (game thread, ~2 s poll) owns the key name
// and only touches it while disarmed; the XINPUT HOOK owns the captured mask and disarms itself.
//   bridge: writes g_bind_capture_key, then stores 1 to g_bind_capture (release)
//   hook:   sees armed, records the first NEW button bit into g_bind_captured, stores 0 (release)
//   bridge: sees g_bind_captured != 0, writes `<key>=0xNNNN` into halo_vr_user.cfg, clears it
// The hook does no file I/O -- writing a cfg from an input callback is exactly the stall class
// the load_config gate exists to avoid.
extern std::atomic<int>      g_bind_capture;     // 0 = idle, 1 = armed
extern std::atomic<int>      g_bind_captured;    // mask the hook caught, 0 = nothing yet
extern std::atomic<uint64_t> g_bind_capture_deadline;   // GetTickCount64 expiry, 0 = none
extern char g_bind_capture_key[32];              // cfg key the capture writes to

// Consume menu commands + refresh the data\ mirrors. Returns how many commands were applied.
int menu_bridge_tick();

} // namespace halo
