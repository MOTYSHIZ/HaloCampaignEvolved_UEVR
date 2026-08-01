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
    // The target rate is a difference of two noisy angles at ~32 Hz; unsmoothed it is jittery
    // enough to buzz the stick.
    float ff_smooth    = 0.5f;
    // Separate, heavier filter for the MEASURED aim rate that damping differentiates. Kept apart
    // from ff_smooth on purpose: smoothing the feedforward costs responsiveness, smoothing the
    // damping input costs almost nothing and is where the buzz actually comes from.
    float d_smooth     = 0.30f;

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
    float aim_reticule_dist = 1000.0f;   // cm along the ray (10 m)
    // Separate distance while seated: the reticule sits at a fixed distance along the aim ray
    // rather than on a traced hit, and too short a distance parks it inside the vehicle's own
    // bodywork. Its APPARENT size does not change with this (see g_ret_scale_mul), so it is tuned
    // purely for depth and occlusion -- 22 m by in-headset judgement. 0 = use aim_reticule_dist.
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

    // XInput button mask that holds the pose-match calibration. Default 0x0020 = BACK / View /
    // the Menu-adjacent button, chosen because it is not a combat action -- holding it mid-fight
    // must not cost you anything. 0 disables calibration entirely.
    //   A=0x1000 B=0x2000 X=0x4000 Y=0x8000  LB=0x0100 RB=0x0200
    //   BACK=0x0020 START=0x0010  LTHUMB=0x0040 RTHUMB=0x0080
    int   calib_btn    = 0x0020;

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

    // Persisted hand-to-aim mapping. What survives is the OFFSET (aim - controller), not the
    // absolute reference pair: the raw values are meaningless in a new level because the game's
    // yaw origin differs, whereas the offset is the actual calibration and is level-independent.
    float aim_off_yaw   = 0.0f;
    float aim_off_pitch = 0.0f;
    bool  aim_off_valid = false;
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
    bool  move_live    = true;

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
};

extern Config g_cfg;
extern char     g_cfg_path[MAX_PATH];
extern char     g_calib_path[MAX_PATH];
extern uint32_t g_cfg_check_tick;
extern bool     g_pivot_from_calib;

void write_default_config();
bool parse_config_file(const char* path);
void load_config();
void write_calib_file();

} // namespace halo