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
    bool  xr_layer = false;
    // SPACE LADDER -- a diagnostic, not a preference. Whether UEVR's get_pose() reports in the same
    // space get_stage_space() names is an assumption we have not measured, and each mode fails in a
    // different, recognisable way. 0 = stage space, raw HMD pose. 1 = stage space with the
    // recentre correction UEVR applies to its own quads. 2 = view space (head-locked; WRONG for a
    // reticule on purpose -- if 2 draws and 0/1 do not, the pose math is the only thing left).
    // Start at 2 when nothing appears at all. See build_pose() for what each answer means.
    int   xr_layer_space = 0;
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
    bool  xr_layer_log = false;

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
    int   map_rstick_down = 0x2000;   // right stick DOWN -> B, crouch on this game's pad map
    float map_rstick_dz   = 0.65f;    // deflection needed; high so turning never trips it
    float map_dpad_dz     = 0.50f;    // left-stick deflection needed to count as a d-pad direction

    // Rebind a button to a different one. `mapfrom` is suppressed and `mapto` sent instead.
    // Defaults are the MEASURED Quest mapping (never guess masks -- on Quest, XInput "X" is the
    // RIGHT controller's B button, so a wrong mask silently unbinds a combat action; measure
    // with `mapbtnlog=1` first): the physical crouch button (0x2000, vacated by the right-stick-
    // down crouch above) becomes LB = equipment. 0/0 disables the rebind.
    int   map_from        = 0x2000;
    int   map_to          = 0x0100;

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
    float scope_exposure = 1.0f;
    // Force an anti-aliasing method on the CAPTURE only. -1 = leave the game's choice alone
    // (default). 0 None, 1 FXAA, 2 TAA, 3 MSAA, 4 TSR. Exists to test whether the black output
    // from post-processed capture sources is a temporal-upscaler interaction; a non-temporal
    // method is the probe. Applied on change, alongside the capture source.
    int   scope_aa = -1;
    float scope_thresh  = 0.55f;   // LT deflection that fires the toggle (release at half)

    // Research knobs (catalogued in halo_vr_dev.cfg, not shipped in halo_vr.cfg):
    float scope_base_fov    = 70.0f; // pane lens at 1x, deg horizontal
    int   scope_capture_src = 0;     // ESceneCaptureSource byte; 0 = SCS_SceneColorHDR (linear,
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
    bool  scope_dev_ray     = false; // [dev build] synthesize the scope ray from the rendered view
                                     // -- SimVR-only verification; the null driver never validates
                                     // the controller aim pose, which parks the real ray source
    bool  scope_force       = false; // hold the pane ON without any trigger input. Harness/support
                                     // diagnostic: under SimVR no input path can reach the LT
                                     // hook at all, and this is the config-file automation channel
    int   scope_cap_mode    = 0;     // 0 = manual CaptureScene every scopediv ticks (the perf
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
    // Set 0 for the narrower behaviour: grip swallowed only while a magazine is expected or held,
    // grenades working the rest of the time.
    bool  grip_exclusive  = true;

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
    // grenade_from = 0 disables the remap entirely.
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

    // ===== FROM blindcowboy24 PR-1, TAKEN ALONE =====================================
    // Over-the-shoulder weapon switching (stow / draw / exchange) and the grenade pouches that
    // share its body frame. Extracted WITHOUT the palette weapon-positioning hook, two-handed
    // aiming or the VR reload from that PR -- Holster.cpp depends on none of them; its only
    // cross-subsystem calls are markers_hide_all() and reload_state()/weapon_key(), all of which
    // this tree already has.
    // ---- HOLSTERS (Holster.hpp). Head-frame offsets in METRES: x right, y up, z BACK.
    bool  holster_enabled = true;
    float holster_radius  = 0.16f;
    // GRENADE POUCHES. Measured 2026-08-24: of 15 grip presses that caught nothing, FOURTEEN were
    // within 16 cm of a pouch and eleven within 12 cm, against a catch radius of 7 cm. They were
    // near misses, not wild reaches -- the hand was arriving at the right place and the sphere was
    // too small to be there. The frag-side misses centre 7.7 cm BEHIND the pouch, which is larger
    // than the entire old radius: the hand sweeps in from the front, and by the time the grip is
    // actually pressed it has settled back against the chest.
    //
    // 0.13 catches 12 of the 15 recorded misses and absorbs that 7.7 cm bias without moving the
    // centre, so the reaches that already worked keep working. It stays clear of the right hip
    // slot too (centres 0.329 apart, 0.13 + 0.16 = 0.29), and where the two pouches now overlap
    // slightly at the sternum the nearest centre wins, which is just "whichever side you are on".
    float holster_gradius = 0.13f;
    float holster_rs[3] = { 0.20f, -0.10f,  0.22f};   // right shoulder (behind)
    float holster_ls[3] = {-0.20f, -0.10f,  0.22f};   // left shoulder (behind)
    float holster_rh[3] = { 0.22f, -0.65f,  0.05f};   // right hip
    float holster_lc[3] = {-0.12f, -0.30f, -0.12f};   // left chest (in front): FRAG
    float holster_rc[3] = { 0.12f, -0.30f, -0.12f};   // right chest (in front): PLASMA
    int   holster_gswitch_mask = 0;                   // "Switch Grenade" pad mask; 0 = not wired yet
    int   holster_melee_veto_ms = 400;                // no melee this long after a holster action
    // ---- HOW FAR OUTSIDE A HOLSTER ZONE STILL VETOES A MELEE.
    //
    // Was 0.10, giving a veto sphere of holster_radius + 0.10 = 0.26 m around EACH of the five
    // zones, which is a large part of the space a punch travels through.
    //
    // The first session's 16 vetoes looked like the veto doing its job, because ten of them sat
    // between a "grenade armed" and a "HOLSTER THROW" -- the detector firing on grenade throws,
    // correctly suppressed. That reading was right about those ten and wrong as a verdict on the
    // rule: the grenades were masking it. The next session threw NO grenades, and all ten vetoes
    // were proximity, against 24 fired -- 29% of qualifying strikes killed, every one with genuine
    // strike kinematics (speed 2.74-5.46, ext 1.89-3.98). They sat at 0.147 to 0.256 m from a zone.
    //
    // 0.00 keeps the veto at holster_radius itself, so a hand actually INSIDE a zone still cannot
    // melee, and saves nine of those ten. What the margin was really protecting is the DRAW -- a
    // weapon coming out of a slot is a fast extension away from the head, indistinguishable from a
    // strike -- and a draw is already covered by holster_melee_veto_ms, which runs from the grip
    // press that started it. The margin was a second, much blunter guard on the same event.
    float holster_melee_margin  = 0.00f;

    // ---- GRENADE VISUALS. From the headset: "it's hard to tell when I actually grab it". Three spheres,
    // spawned with the wheel-marker recipe: one on each chest pouch so the grab target is visible,
    // and one ON THE HAND while a grenade is armed, so a successful grab is unmissable. Spheres
    // for now -- real grenade meshes need their asset names, which grenademeshdump discovers.
    //   holstermarkers: 0 off, 1 = pouches appear as the hand approaches (default), 2 = always on.
    int   holster_markers = 1;
    float holster_marker_scale = 0.08f;   // sphere diameter in engine scale (~8 cm)

    // ---- THE THROW GOES WHERE YOU THREW IT. Same disease, same cure as melee: the game lobs the
    // grenade along the AIM, and the aim during a throw is the flailing hand itself. On a fired
    // throw the aim is pinned to the SWING'S OWN DIRECTION -- sampled at the velocity peak, the
    // same instant the throw gate reads -- for this many ms, through the identical hold machinery
    // the melee uses. 0 disables (grenade flies wherever the aim happens to point, old behaviour).
    // 350 covers the 120 ms synthetic press plus the game's wind-up; how long the game actually
    // needs is NOT measured -- raise this first if grenades fly off-line.
    int   holster_aim_hold_ms = 350;

    // Gesture aim-hold shape (shared with later gesture features): how long the pinned aim holds
    // past the gesture, and the ramp back to the live hand so the reticle returns, not teleports.
    int   melee_aim_hold_ms = 250;
    int   melee_aim_ramp_ms = 150;

    // ---- THE ZONES HANG ON A TORSO, NOT ON THE HEAD. From the headset: turn your head
    // right and the plasma pouch is inside your body -- because the zones rotated one-to-one with
    // head yaw, as if the player were a 2D square. The standard VR fix (and what body-holster
    // games actually ship) is a lagged body yaw with a leash: the body stays put while the head
    // looks around inside a dead zone, gets DRAGGED once the head passes the limit (gear is never
    // fully behind you), and re-centres slowly toward where you face, so a sustained turn brings
    // the gear around while a glance moves nothing.
    //   holsteryawdead: half-width of the free-look cone, degrees.
    //   holsteryawrate: recentre speed, deg/s (0 = only the drag moves the body).
    float holster_yaw_dead = 45.0f;
    float holster_yaw_rate = 10.0f;

    // ---- NECK PIVOT (torso tier 1). The zones used to hang off the head's POSITION, and the
    // head's position arcs ~20 cm forward when you nod -- the eyes rotate about the neck, the
    // chest does not, so looking down dragged the pouches forward off the body. The anchor is now
    // a computed NECK point (head position + this offset rotated by the full head orientation),
    // lifted back to head height in the BODY frame -- for a level head the zones land exactly
    // where they always did, and a nod moves them barely at all. Metres; both 0 = old behaviour.
    float holster_neck_down = 0.15f;   // eyes to neck pivot, straight down in the head frame
    float holster_neck_back = 0.08f;   // ...and slightly behind the eyes

    // With holsters on, the PHYSICAL buttons for weapon swap / grenade switch / grenade throw are
    // swallowed (the gesture system owns those actions; a stray Y press must not swap). Synthetic
    // holster presses are injected AFTER the mask, so they still work. Menus and stick mode keep
    // the buttons -- Y navigates UI, and stick mode has no holsters to replace it.
    int   holster_steal_buttons = 1;
    // WHICH HAND WORKS THE GRENADE POUCHES. 2 = BOTH (default): either grip grabs, and the
    // behaviour follows the hand that is carrying -- in the OFF hand the gun stays live and
    // visible in the aim hand (the natural two-handed carry); in the AIM hand the weapon hides
    // and fire suppresses while the grenade shares it (the original single-hand behaviour).
    // 1 = off hand only; 0 = aim hand only. Weapon-swap holsters stay on the aim hand
    // regardless -- only the pouches are handed.
    int   holster_gren_hand = 2;

    int   holster_swap_mask  = 0x8000;                // Y = switch weapon on the default pad map
    int   holster_throw_mask = 0x0100;                // LB = throw grenade (grenade_action)
    int   holster_press_ms   = 120;
    float holster_throw_speed = 1.2f;                 // m/s forward at release
    bool  holster_haptic = true;
    bool  holster_log    = false;

    // ---- THE VISIBLE MAGAZINE (reloadmag). The belt grab used to be a half-metre invisible ring
    // around the waist: reach anywhere low and squeeze. With this on, dropping the mag SPAWNS a
    // magazine mesh at a fixed point on the belt, and the grab must take THAT -- the fetch hand
    // within reloadmagrad of the mag itself. While carried, the mesh rides the fetch hand until it
    // seats. The mesh is the game's own (resolved from the loaded-object list, same recipe as the
    // grenade pouch markers); the resolve log prints every magazine-ish StaticMesh it finds, which
    // is also the asset survey the per-weapon-mag step needs. Falls back to the frag grenade mesh
    // rather than an invisible point -- a stand-in you can see beats a shape you cannot.
    // The point and the grab test live in the HOLSTER's body frame (torso leash, neck pivot), so
    // the mag hangs on your hip exactly as the pouches do; with holster=0 the frame does not run
    // and the grab silently falls back to the legacy ring.
    // HIDE THE WEAPON'S OWN MAGAZINE while the VR reload has it out. The gun keeping its mag
    // during MAG_OUT was reported from the field as "the magazine still appears to be in the
    // rifle" -- the game ships the mag as its own component on the weapon actor, so it can be
    // hidden for real. Substring of the component's class or object name; magdump surveys the
    // held weapon's components one-shot on value change (dev instrument, harmless in release).
    bool  mag_hide = true;
    char  mag_hide_name[64] = "Magazine";
    int   mag_dump = 0;

    int   reload_mag = 1;
    // Body-frame belt point (x right, y up, z back, metres) -- default mirrors holsterrh onto the
    // LEFT hip, the fetch hand's side.
    float reload_mag_off[3] = {-0.22f, -0.65f, 0.05f};
    float reload_mag_radius = 0.15f;
    // World scale on the mag mesh. 1.0 = the asset's authored size.
    float reload_mag_scale = 1.0f;
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
// Menu-armed calibration mode: 0 = off, 1 = pose-match armed, 2 = aim-ray armed. Written by the
// bridge (calib:pose / calib:aim / calib:off commands) and by the trigger edges in Plugin.cpp;
// the update() gesture block treats an armed mode as a held calibration key.
extern std::atomic<int> g_menu_calib_mode;

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
