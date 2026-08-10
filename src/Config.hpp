// Live configuration and calibration persistence.
//
// Everything that touches the two files next to the UEVR profile lives here:
//   halo_vr.cfg    - user tunables, re-read every ~2 s. Holds the KILL SWITCH (enabled=0).
//   halo_vr_calib.cfg - written by the in-game calibrations; applied AFTER halo_vr.cfg so it wins.
//
// g_cfg is read from nearly every system in the plugin, so this header is the one most others
// include. Keep it free of UEVR API calls: the paths are filled in by the plugin at startup.

#pragma once

#include <Windows.h>
#include <cstdint>

namespace halo {
// ---------------------------------------------------------------- tunables
// Defaults are overridden at runtime by halo_vr.cfg next to the UEVR profile (see load_config
// below), which is re-read every ~2 s -- so a headset session can tune, or hit the kill switch,
// without a rebuild or a restart.
struct Config {
    bool  enabled      = true;
    bool  drive_pitch  = true;
    float floor        = 0.28f;   // deflection the game starts acting on (measured ~0.24)
    float full_deg     = 6.0f;    // error that saturates the stick (MCC-VR uses ~4.8)
    // Keep at 1.0. The stick curve is steeply exponential -- 0.70 -> ~39 deg/s but 0.90 ->
    // ~137 deg/s -- so the top of the range is where large corrections actually get closed;
    // capping it leaves the loop slew-limited on big movements only.
    float max_out      = 1.0f;
    float dead_deg     = 1.0f;    // angular deadband so it parks instead of hunting

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
    float ff_gain      = 1.0f;    // 0 disables (pure-P behaviour)
    // Damps whatever the plant's own lag still overshoots by, opposing rate error rather than
    // position error so it does not fight the feedforward.
    float d_gain       = 0.15f;
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
    float xdist_m      = 10.0f;   // sightline length (m); UEVR uses a flat 1000 units

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
    bool  hmd_leash      = true;
    float hmd_leash_lat  = 0.0f;    // metres, horizontal -- 0 = fully negated
    float hmd_leash_vert = 0.0f;    // metres, vertical   -- 0 = fully negated
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
    float snap_deg     = 30.0f;   // per flick
    float smooth_dps   = 90.0f;   // degrees per second at full deflection
    float turn_dz      = 0.5f;    // stick deflection required to register

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
    float cut_hint_dist = 1.8f;   // metres ahead of the standing origin
    float cut_hint_drop = 0.75f;  // metres below eye level
    float cut_hint_w    = 1.1f;   // overlay width, metres

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

    // WHICH socket is the pivot. You rotate your controller about your WRIST, so the pivot wants to
    // be the in-game hand -- which is not necessarily where the weapon mounts. `PrimaryWeapon` is
    // the weapon's attach point; `Wrist_R` is the arm socket nearer the hand. Both exist on this
    // rig, they are centimetres apart, and which one feels right is a question about the art, not
    // one this code can answer -- so it is a dial, and both are logged on acquisition.
    char  piv_socket[64] = "PrimaryWeapon";

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
    float aim_reticule_dist = 1000.0f;   // cm along the ray (10 m)

    // TRACE the reticule onto the surface instead of parking it at aim_reticule_dist.
    //
    // A world marker at a fixed distance only agrees with the impact point when viewed from the
    // origin the shot leaves. Your eye is not there and moves with your head, so it drifts -- see
    // HitTrace.hpp for the measured numbers. Tracing removes the free parameter entirely: the
    // marker sits on the surface, so it is correct from any eye position at any range, and
    // aim_reticule_dist stops mattering except as the fallback when nothing is hit.
    //
    // DEFAULT OFF, and honestly so: this calls a reflected engine function once per tick with a
    // hand-built parameter block, and while every offset is resolved from reflection rather than
    // assumed, it has not run in a live session yet. Measure it with perflog before trusting it.
    bool  aim_reticule_trace = false;
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
    int   aim_reticule_trace_channel = 0;

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
    float aim_reticule_max_dist = 1000.0f;   // 10 m

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
    float aim_reticule_min_scale      = 0.1f;
    float aim_reticule_min_scale_dist = 10.0f;   // cm
    float aim_reticule_max_scale      = 1.0f;    // at aim_reticule_max_dist

    // Pull the reticule this many cm back along the ray from the surface it hit, so it sits just in
    // FRONT of the wall instead of intersecting it. Applied only when actually drawing at a hit:
    // if the surface is past aim_reticule_max_dist the marker parks at the cap exactly, not at
    // (cap - offset), because there is nothing there to clip into.
    float aim_reticule_surface_off = 10.0f;   // cm

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
    //   div_ms 500: "a moment" -- comfortably longer than a whip turn, whose error resolves within
    //     the loop's ~250 ms settle, so ordinary play never trips the guard; still short enough that
    //     a dead loop is caught before the player has fired more than a shot or two at the wrong
    //     place.
    float aim_reticule_div_deg = 8.0f;
    float aim_reticule_div_ms  = 500.0f;

    // First-order filter on the emitted reticule angles, as a TIME CONSTANT in ms (see ema_alpha).
    // 0 = off (raw). LOWER is snappier, higher is calmer -- the opposite sense to the old per-call
    // fraction this replaced, which is worth knowing if you are carrying tuning across.
    float aim_reticule_smooth_ms = 26.0f;

    // Same filter, but for the CONTROLLER-sourced path (src=1). Separate because that path is
    // already responsive and only needs the small, fast jitter of hand tremor and tracking noise
    // taken off -- not the heavier filtering the loop's residual calls for. Also motion-gated, so a
    // deliberate swing is never filtered.
    float aim_reticule_smooth_ctrl_ms = 12.0f;

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
    bool  aim_reticule_lua  = true;
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
    bool  hud_follow   = true;

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
    bool  aim_draw      = true;
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
    bool  aim_mesh      = true;
    float aim_mesh_scale = 0.06f;   // Engine sphere is 100 cm radius, so this is ~6 cm
    // Unlit pass-through: an unlit material keeps the reticule readable against dark geometry, and
    // is what uevrlib uses for the same purpose.
    bool  aim_mesh_unlit = true;
    // One-shot sweep for a COOKED GAME material carrying a colour parameter, so the mesh reticule
    // can be tinted. Engine materials are exhausted: BasicShapeMaterial has no colour parameter and
    // this build ships only a handful of others, none tintable.
    bool  mat_hunt      = false;
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
    bool  aim_tex       = false;                    // use the textured-quad reticule
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
    char  aim_mesh_parent[192] = "masked";   // holds either a keyword or a full object path
    float aim_tex_rot_p = 0.0f, aim_tex_rot_y = 0.0f, aim_tex_rot_r = 0.0f;
    // Halo's own reticle blue, and the hit-marker red. Both live-tunable so they can be matched by
    // eye against the flat HUD rather than guessed from a screenshot.
    float aim_mesh_cr = 0.35f, aim_mesh_cg = 0.80f, aim_mesh_cb = 1.00f;

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
    float aim_widget_scale = 0.12f;   // world scale of that quad
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
    bool  aim_widget_mat  = true;

    // DIAGNOSTIC. Paints the widget quad's background opaque red. One run then distinguishes the
    // three remaining possibilities, which no amount of further reasoning can separate:
    //   red square + crosshair -> fixed
    //   red square, no crosshair -> the quad renders; the WIDGET CONTENT is not reaching the target
    //   nothing at all -> the quad itself is not rendering (transform, scale or material)
    bool  aim_widget_bg   = true;

    // Control test: host a NEWLY CREATED reticle instead of the HUD's live one. Loses the hit
    // marker, so it is a diagnostic only -- it isolates re-parenting from the component setup.
    bool  aim_widget_fresh = true;

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
    float aim_widget_tint  = 1.0f;   // applied to R,G,B
    float aim_widget_alpha = 1.0f;
    // EMISSIVE GAIN for the hosted crosshair. The stock Widget3D pass is unlit but its output is
    // still multiplied by the scene's PRE-EXPOSURE before tonemapping, so Halo's authored cyan
    // lands near black in bright scenes -- the long-standing "dark crosshair". This multiplies it
    // back up. Live-tunable: raise if the crosshair still reads dark outdoors, lower if it blows
    // out or looks washed. Forced to 1.0 automatically when the exposure-compensated VREditor
    // material is in use (that one preserves authored colour at unit tint).
    float aim_widget_gain  = 4.0f;
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
    //   2 = PARENT-RELATIVE (default) -- inverse(parent) * controller. Correct composition like
    //                      mode 0, full roll and translation like mode 1.
    int   rig_mode     = 2;

    // Anchor hand TRANSLATION to the body rather than to a fixed point in the room.
    //
    // With a fixed neutral, the measured offset is the hand's distance from wherever it happened to
    // be at reference capture -- so walking across the room drags the gun to the clamp and holds it
    // there. Subtracting the head position first leaves only hand-relative-to-body motion, which
    // is what a held object should follow.
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
    // 1 = apply, -1 = negated, 0 = off.
    float move_rot     = 1.0f;

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
    int   move_live    = 1;

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
    float move_smooth  = 0.25f;

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
    int   map_rstick_down = 0x0040;   // right stick DOWN  -> LTHUMB (crouch on this game's pad map)
    float map_rstick_dz   = 0.65f;    // deflection needed; high so turning never trips it
    float map_dpad_dz     = 0.50f;    // left-stick deflection needed to count as a d-pad direction

    // Rebind a button to a different one. `mapfrom` is suppressed and `mapto` sent instead.
    // BOTH DEFAULT TO 0 (disabled) UNTIL THE MAPPING IS MEASURED: how a headset's controllers
    // land on XInput masks is not guessable (on Quest, XInput "X" is the RIGHT controller's B
    // button -- reload), so a guessed mask silently unbinds a combat action. Set `mapbtnlog=1`,
    // press each button, and read the measured masks out of the log before assigning anything
    // here.
    int   map_from        = 0;
    int   map_to          = 0;

    // Log every XInput button-mask change, so the Quest->XInput mapping can be READ rather than
    // assumed. Off by default; it is noisy.
    bool  map_btn_log     = false;

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
    // rate at deflection 1.0. Left at 0 the loop estimates it from the adaptive gain, which is
    // serviceable but coarse.
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

    float plant_full_dps = 0.0f;

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
};

extern Config g_cfg;
extern char     g_cfg_path[MAX_PATH];
extern char     g_calib_path[MAX_PATH];
extern char     g_calib_path_right[MAX_PATH];
// Result of select_calib_for_hand(). Returned rather than logged because this file is kept free
// of UEVR API calls (see the header comment) -- the caller does the reporting.
enum { CALIB_HAND_RIGHT = 0, CALIB_HAND_LEFT_LOADED = 1, CALIB_HAND_LEFT_SEEDED = 2 };
int select_calib_for_hand();
extern uint32_t g_cfg_check_tick;
extern bool     g_pivot_from_calib;
// TRUE when an aim offset loaded WITHOUT an explicit aimcalibver stamp, so its schema is inferred
// rather than known. Set by load_config(); the tick reports it once (this file makes no API calls).
extern bool     g_calib_stamp_ambiguous;

void write_default_config();
bool parse_config_file(const char* path);
void load_config();
void write_calib_file();

} // namespace halo
