// THE FORK'S CFG FIELDS, included by the author's Config.hpp at the end of struct Config. Every block below
// sat inside the struct after the author's line named in its heading; moved verbatim, in that order.
// Member declarations only: no includes, no namespace.

    // ---- (after the author's Config.hpp line 261)
    // AIMBORE (2026-09-13). One calibration for the gun, the shot and the reticle.
    //   0 = the aim is the aim-fixed hand forward. The drawn weapon also carries the global grip
    //       rotation and the held weapon's trim, so the barrel sits off the shot by that trim
    //       (measured: the AR's 2.0 deg pitch trim = barrel 2.0 deg above the aim).
    //   1 = the aim is the drawn barrel's axis: the aim-fixed hand composed with the same grip
    //       rotation and weapon trim the palette applies. The gun is NOT moved; the aim and the
    //       reticle (which follows the aim intent) move onto it.
    //   The composed rotation includes paletterolltrim, which the palette applies between the grip
    //   rotation and the weapon trim (fitted 2026-09-13: without it 0.91 deg rms against five
    //   measured trims, with it 0.15 deg).
    //   2 = mode 1 aimed along a fixed barrel axis in the trimmed pose frame, aimboreaxis=pitch,yaw
    //       degrees (fitted socket barrel -Y offset from the pose forward: +2.10, -0.15).
    //   3 = mode 1 aimed along the live measured barrel axis (the BARRELAXIS EMA), falling back to
    //       aimboreaxis until it is valid.
    int   aim_bore = 0;
    float aim_bore_axis[2] = {2.10f, -0.15f};

    // ---- (after the author's Config.hpp line 385)
    // ---- ROOMSCALE LOCOMOTION THROUGH THE GAME'S OWN MOVEMENT.
    //
    // Native roomscale that sets the pawn location through UE gets overwritten by the sim every
    // tick (same ownership problem as the FP palette), so physically walking moves nothing -- and
    // the leash above then cancels the head offset outright. This goes through the channel that
    // works here: the lateral head offset from the standing origin becomes a movement command in
    // the game's own frame, the game moves the player with its own physics, collision and speed
    // cap, and the standing origin is then slid by however far the player's eye ACTUALLY moved,
    // so the offset shrinks by what was achieved and never double-counts. Only when the player's
    // own stick is idle (a pushed stick is locomotion, not roomscale), never in stick mode or
    // menus. The lateral leash stands down inside roomscale_leash so there is an offset left to
    // walk out; beyond it the origin is dragged as before, so the eye can never run away from
    // the body.
    bool  roomscale       = false;
    // Default: the value roomscale is tuned with (halo_vr.cfg ships the same line).
    float roomscale_gain  = 4.0f;    // desired eye speed (m/s) per metre of head offset
    float roomscale_dead  = 0.03f;   // metres, no command inside this
    float roomscale_leash = 1.0f;    // metres, lateral leash radius while roomscale is on
    float roomscale_stick = 0.15f;   // player stick magnitude above which roomscale yields
    bool  roomscale_log   = false;   // ROOMSCALE line ~8x/s: offset, command, eye delta, slide
    // Default: the value roomscale is tuned with (halo_vr.cfg ships the same line).
    float roomscale_min   = 0.36f;   // stick magnitude floor while a command stands (clears the game deadzone)
    float roomscale_speed = 3.3f;    // metres/s of eye travel at full stick (measured 3.0-4.7)
    float roomscale_lat   = 0.06f;   // seconds from command to visible eye motion (measured 20-57 ms)
    // Default: the value roomscale is tuned with (halo_vr.cfg ships the same line).
    float roomscale_dz    = 0.30f;   // the game's own stick deadzone (measured: 0.30 -> 0.22 m/s)
    int   roomscale_pulse = 2;       // ticks per on-block when duty-cycling below the floor
    float roomscale_ff    = 0.0f;    // head-velocity feed-forward gain (1 = command the head's own speed)
    // INVOLUNTARY-MOTION STAND-DOWN. Roomscale can only move the eye at roomscale_speed, so an
    // eye travelling well above that is being thrown, not walked -- a jump, a rock, knockback.
    // Commanding through those compounds with the game's own momentum (air control accepts stick
    // input), which is the "my body flies away from me" case. Above the limit, stop commanding
    // for standdown seconds and let the physics finish. Strictly subtractive. 0 disables.
    float roomscale_max_speed = 6.0f;    // m/s of eye travel that means "thrown"
    float roomscale_standdown = 0.35f;   // seconds of silence after one
    // ROOMSCALE THROTTLE WRITE. 0 = deliver roomscale as an XInput left stick (subject to the
    // game's deadzone and minimum speed -- the "snapping"). 3 = write the UNIT object's throttle
    // vectors from the sim-side hook -- THE ONE THAT MOVES THE BIPED (measured: eye speed =
    // 6.65 m/s x throttle, linear from 0.02 up, no floor). Modes 1/2 wrote the control record,
    // which survives but moves nothing (consumed before the write lands); not carried here.
    int   roomscale_throttle = 3;
    float roomscale_thr_speed = 6.65f;  // metres/s of eye travel per unit of unit-throttle (measured)
    // THROTTLE-FRAME PROBE (discovery): non-zero writes a constant forward throttle of this/100
    // into the unit object; the ROOMSCALE-PROBE log gives the world direction the pawn moves vs
    // aim/view, which pins the biped's throttle frame. 0 = off.
    int   roomscale_thr_probe = 0;
    // Mode 3 targets: unit object throttle vector (fwd,left) and its second copy. 0 = skip.
    int   blam_unit_throttle_off  = 0x250;
    int   blam_unit_throttle_off2 = 0x25C;
    // Default -1, the measured handedness roomscale is tuned with (halo_vr.cfg ships the same line). With
    // 1 the right component is mirrored and the pawn walks away from the head instead of closing on it.
    int   blam_throttle_ysign = -1;   // sign applied to the RIGHT component (frame handedness, measured)
    // CAMERA BOB CANCEL: remove the walk-animation bob (fast part of camera-vs-pawn, tau seconds
    // low-pass keeps eye height / crouch) from the rendered view and the palette hand.
    bool  bob_cancel = false;
    float bob_tau    = 0.4f;
    bool  bob_log    = false;
    // ---- AUTO HEIGHT (HeightCal.hpp). Writes the standing origin's Y so the rendered eye's height above
    // the GAME floor follows the head's height above the REAL floor. While it is on, the vertical
    // leash never acts. Head-side keys are CM (fields metres); game-side keys are UE cm.
    int   height_cal      = 0;       // heightcal: 0 off, 1 on
    int   height_mode     = 0;       // heightmode: absolute (0), seated (1), eyes (2, also the floor-unknown fallback)
    int   height_scale    = 0;       // heightscale: 0 = UEVR world scale (real m x 100 x VR_WorldScale), 1 = real cm
    int   height_src      = 0;       // heightsrc: 0 auto, 1 OpenXR STAGE, 2 OpenVR standing, 3 UEVR pose only
    int   height_eye      = 1;       // heighteye: E_game source, 1 floor trace, 2 Blam biped, 3 UE pawn
    int   height_trace_channel = 0;  // heighttracechannel: ETraceTypeQuery index for the floor trace (0 Visibility)
    float height_trace_max = 400.0f; // heighttracemax: UE cm below the camera the floor trace reaches
    float height_hold_ms  = 400.0f;  // heightholdms: a jump in E_game must persist this long to be taken
    float height_e_step   = 3.0f;    // heightestep: UE cm per tick of E_game change taken at once
    float height_biped_scale = 304.8f; // heightbipedscale: UE cm per Blam world unit for the biped Z
    float height_biped_feet = 0.0f;  // heightbipedfeet: UE cm from the biped origin to the feet, 0 = learn from the trace
    float height_pawn_feet = 0.0f;   // heightpawnfeet: UE cm from the pawn root to the feet, 0 = capsule half-height, else learned
    float height_seat_target = 0.0f; // heightseattarget: UE cm seated view target, 0 = the character's eye height (E_game)
    int   height_auto_seat = 0;      // heightautoseat: one-shot seated detection at startup (absolute mode)
    float height_seat_below = 1.00f; // heightseatbelow (cm): resting head under this above the real floor = seated
    float height_seat_dwell = 5.0f;  // heightseatdwell (s): still time the startup detection needs
    int   height_sample   = 0;       // heightsample (eyes): 0 window once, 1 continuous envelope, 2 key only
    float height_window_s = 5.0f;    // heightwindow (eyes): seconds of still, on-foot samples per window
    int   height_key      = 0;       // heightkey: VK code of the recalibrate key (0 = none; 0x2D is refused, UEVR's menu)
    float height_trim     = 0.0f;    // heighttrim (eyes, cm): + raises the view
    float height_min_abs  = 1.20f;   // heightmin (cm): retained key, unused by the absolute and seated modes
    float height_band     = 0.10f;   // heightband (eyes, cm): continuous mode, drops larger than this are crouches
    float height_slew     = 0.5f;    // heightslew (cm/s): how fast a mode switch or new calibration is walked in (0 = snap)
    int   height_log      = 0;       // heightlog: 0 silent, 1 events, 2+ events + a HEIGHT line once a second
    // ---- HEAD BLOCK (HeadBlock.hpp). Keeps the rendered head out of geometry. UE cm.
    int   head_block         = 0;      // headblock: 0 off, 1 line trace, 2 sphere sweep, 3 lean limit (no trace)
    float head_block_radius  = 12.0f;  // headblockradius: clearance kept from the surface
    float head_block_lean    = 25.0f;  // headblocklean: mode 3, max horizontal head offset from the body
    int   head_block_channel = 1;      // headblockchannel: ETraceTypeQuery index (0 Visibility, 1 Camera)
    float head_block_release = 150.0f; // headblockrelease: cm/s the allowed distance grows back after a hit clears
    int   head_block_log     = 0;      // headblocklog: 0 silent, 1 events, N>1 events + a line every N ticks

    // ---- (after the author's Config.hpp line 569)
    // FRAMING is the comfort choice, and for a flat picture it is entirely SIZE: 1.0 = as the


    // ---- (after the author's Config.hpp line 802)
    // Reticule ray timing. The publish builds its ray from the ControlRotation read at the top of
    // the aim path, which is taken BEFORE that tick's aim write, so the drawn reticule trails the
    // aim by a tick on top of the game's own delay.
    //   0 = current behaviour (the early read).
    //   1 = re-read ControlRotation at the publish site, after the tick's aim law has run, and build
    //       the reticule ray from that. Logs the early-vs-fresh difference every ~2 s so the extra
    //       tick can be measured rather than assumed.
    int   aim_reticule_fresh = 0;
    // RETSTAMP (2026-09-13). Where the compositor reticle gets its aim and WHEN it is placed.
    // The tick placement read ControlRotation before the tick's aim write (3 ticks behind the
    // hand), started the ray at the rig parent (camera bob), and anchored it to a view position
    // from a different moment than the head the layer adds each frame.
    //   0 = tick placement (the ported behaviour)
    //   1 = render placement: this frame's view position + the stamped intent from two snapshots
    //       back (exactly the aim the game shows at render, where shots go) x the latest trace depth
    //   2 = render placement with the intent from one snapshot back (matches the drawn weapon)
    int   aim_reticule_stamp = 0;

    // ---- (after the author's Config.hpp line 1194)
    // Two instruments that shipped UNGATED and were found spamming a play session (2026-09-02:
    // 2,684 TINT COMPARE lines, plus a movement-frame probe on every step). Both are engine
    // calls per sample, not just log lines, so they cost frame time as well as disk.
    bool  widget_log = false;          // dev: TINT COMPARE (hosted-widget MID tint readback)
    bool  move_probe = false;          // dev: PROBE stickIn/stickOut/actualMove movement frame

    // ---- (after the author's Config.hpp line 2355)
    // THROW WINDUP DUMP (Route A of the grenade-delay hunt). Diff-logs the unit object across a
    // grenade throw: while no throw is in flight it LEARNS which dwords of unit+0x000..0x5FF churn
    // every sim call (position, anim floats) and masks them; the synthetic throw press then opens a
    // ~1.2 s window in which every change OUTSIDE the mask logs as +0xOFF old->new. The grenade
    // count byte is watched explicitly -- its decrement IS the game's release moment, so one throw
    // yields the press-to-release latency AND the candidate state/timer fields that ramp during the
    // windup. Bump the value (1 -> 2 -> ...) to re-arm the mask from scratch. Sim thread, dev only.
    int   throw_dump = 0;

    // WRIST-RADAR SURVEY (blipdump): one-shot on value CHANGE -- walks the sim object table for
    // objects near the player and dumps their headers. Run once standing near MARINES ONLY, once
    // near COVENANT ONLY (bump the value): the byte constant within each capture but different
    // across them is the TEAM field the wrist radar's own blips will be built on.
    int   blip_dump = 0;

    // FACTION-FIELD SURVEY (blipbytes): one-shot on value CHANGE. Dumps a compact hex window of
    // every tracked contact, tagged with its id and its +0x177 byte. Run once among COVENANT ONLY
    // and once among MARINES ONLY; the offsets that are constant within each capture and differ
    // between them are faction candidates. +0x177 is NOT one: it reads 0x0E for marines AND for
    // Elites, 0x0D for Grunts, so it is a body/biped class, and the original "team byte" call was
    // three marines and one Grunt of coincidence.
    int   blip_bytes = 0;

    // Per-species blip palette: repeatable `blipcolor=<hex species id>,<r>,<g>,<b>`, e.g.
    // blipcolor=E1870013,0,0.6,1. The id is the object's leading dword (see g_blip_type); an
    // unlisted species takes blip_color_other and logs its id once, so the palette fills in from
    // play instead of needing a survey per level.
    BlipColor blip_color[kMaxBlipColor];
    int       blip_color_n = 0;
    float     blip_color_other[3] = {0.85f, 0.85f, 0.95f};
    // Bumped by every palette edit. The renderer builds one render target per species and caches
    // it for the session, so without this a live blipcolor change has nothing to act on -- the
    // colour only appeared on a species' FIRST sighting, which made the palette effectively
    // write-once and looked like the edit was ignored.
    int       blip_color_gen = 0;
    // Name-keyed palette: repeatable `blipname=<class substring>,<r>,<g>,<b>`, e.g.
    // blipname=Grunt,1,0,0. Checked BEFORE the id palette, first match wins, and it survives
    // level loads where the id table does not.
    BlipName  blip_name[kMaxBlipName];
    int       blip_name_n = 0;

    // NATIVE-BLIP INVESTIGATION (trackerdump): one-shot on value CHANGE. Finds the live
    // WBP_MotionTracker instance and logs every UObject whose outer chain reaches it, plus every
    // live widget anywhere whose class name contains "Blip" (dynamically created widgets are
    // often outer'd to the world, not their visual parent -- the second sweep catches those).
    // Run once with the tracker NATIVE near enemies, once HOSTED near enemies: blip children
    // present-but-unrendered and blip children absent want OPPOSITE fixes (render the feed we
    // have vs leave the tracker in the HUD and mirror its feed), and only this census separates
    // them -- "the feed queries the HUD tree" has so far been theory, not measurement.
    int   tracker_dump = 0;

    // NATIVE-BLIP INVESTIGATION, ROUND 2 (trackermid): one-shot on value CHANGE. The tree census
    // proved blips are NEVER child widgets (19 objects, 0 blip classes, dots visibly on screen),
    // so the feed is inside HaloUIMotionTrackerImage / its MaterialInstanceDynamic. This probe
    // snapshots those objects' memory -- plus every plausible embedded TArray's payload, because
    // a material's parameter values live behind a pointer the object diff cannot see -- and diffs
    // twice, ~0.5 s apart. Blip data churning in the NATIVE capture but frozen in the HOSTED one
    // = the feed is gated upstream; churning in both = the data arrives and the failure is in
    // rendering it.
    int   tracker_mid = 0;

    // WRIST RADAR (wristradar): our own blips on the wrist-hosted motion tracker -- the hosted
    // widget's native blip feed is severed (mechanism under investigation, see trackerdump), so
    // the dots come from the sim's object table instead: team byte unit+0x177
    // (0x0E human / 0x0D covenant), 25 m range, moving-only, rotating with your facing.
    bool  wrist_radar = true;
    // Blip mesh scale, and the radar's world radius as a fraction of the tracker panel.
    // Default: the value the wrist HUD is tuned with (halo_vr.cfg ships the same line).
    float wrist_radar_blip = 0.004f;
    // Emissive gain on the coloured radar dots -- same pre-exposure problem as the hosted panels
    // (aimwidgetgain), separate knob because the dots are solid colour and saturate earlier.
    // Default: the value the wrist HUD is tuned with (halo_vr.cfg ships the same line).
    float wrist_radar_gain = 1500.0f;
    // In-plane rotation of the blip field on the panel, degrees, and a left/right mirror. The
    // blip frame rides the PANEL's pose, and the panel itself is rolled to sit on the forearm
    // (wristhudrotr), so "panel up" is not "ahead of you" by exactly that roll -- this is the
    // steer-by-feel corrector, same method as every placement knob here.
    // Sign the AIM term enters the blip bearing with. A live knob on purpose: it is the one
    // remaining unknown in the chain (whether blam bearings and g_desired_yaw share a rotation
    // sense), a wrong choice is invisible at one heading and doubles as you turn away from it,
    // and every attempt to settle it from a single logged sample picked the wrong contact.
    float wrist_radar_aimsign = -1.0f;
    float wrist_radar_rot  = 0.0f;
    int   wrist_radar_flip = 1;
    // RADAR CHAIN LOG (~1 Hz, first blip only): base yaw, head facing, raw blam offset, room
    // direction, final panel coords. The turn-response bug survived three sign "fixes" derived
    // from o'clock reports; this is the fit-against-logged-numbers instrument that ends that.
    bool  wrist_radar_log  = false;
    // CALIBRATION STAR (wristradartest): replaces the live blips with ONE synthetic contact at a
    // fixed world direction (blam +X, 15 m). Every real blip is a MOVING enemy by construction
    // (the moving-only rule), so every turn-response reading taken against one is contaminated
    // by the enemy's own motion -- the confound that kept the o'clock fits inconsistent all
    // night. Against the star: sweeping the aim right must sweep the dot the opposite way around
    // the ring, degree for degree, and any constant offset is wristradarrot's to absorb.
    int   wrist_radar_test = 0;
    // Centre of the blip orbit on the panel, as fractions of the panel half-width along panel
    // right / panel up. The star proved the published coords are a perfect unit circle while the
    // dot visibly wandered off the RING -- the widget draws its ring art off-centre in its
    // canvas, so the orbit centre is a knob, steered by eye against the star like every
    // placement constant here.
    float wrist_radar_center[2] = {0.0f, 0.0f};
    // Pitch of the DOT PLANE about the panel's horizontal axis, degrees. DEFAULT 0, and it
    // should stay there: the -34 this once held was a MISDIAGNOSIS. U.view = -0.56 in the axis
    // log is just a wrist panel being viewed obliquely, which is normal, and "correcting" it
    // lifted the dots OUT of the art plane so their apparent position moved with the head and
    // wrist. The real fault was the room/world frame mismatch (see the blip block). Kept only as
    // a knob in case a future panel really is off-plane.
    float wrist_radar_tilt = 0.0f;

    // GRENADE-AT-HAND EXPERIMENT (grenhand, dev): while the synthetic throw press is active,
    // overwrite the projectile spawn ORIGIN (params+0x1C) with the carrier hand's position in
    // Blam units (UE world / 304.8, Y negated -- the unit-position fit). The +0x28 DIRECTION
    // write was re-derived downstream and ignored; whether the ORIGIN takes is exactly what this
    // key answers, by eye and by GRENTRACK. Gated on the press window so NPC and gunfire spawns
    // are never touched.
    int   gren_hand_spawn = 0;

    // INSTANT GRENADE RELEASE (greninstant, dev). GRENSNAP decoded the hold: the projectile is
    // created ~42 ms after the press but PARENTED to the biped's throw-hand bone (+0x0C parent
    // datum, +0x18 low byte = bone 0x47) with zero velocity until the animation keyframe at
    // ~250 ms detaches it (+0x0C/+0x14 -> FFFFFFFF, +0x18 -> ..FF, +0x04 |= 0x80, +0x08 -> 4)
    // and writes velocity (+0x68..0x70). This performs that release AT SPAWN, with the velocity
    // pointed along the player's own swing, and re-asserts the velocity for 400 ms at the sim-call
    // cadence so the keyframe's own late release is overwritten within ~0.12 ms of landing.
    // Mode 1 (field-poked release at spawn) is REFUTED: the grenade detaches but never enters
    // the physics simulation (registration handles only the game's release code can create),
    // and the fight wedged the unit's action queue. Mode 2 BACKDATES the throw-start tick stamp
    // at unit+0x38C the moment the game writes it: the four snapshot-hunt "clocks" were mirrors
    // of the global 60 Hz tick counter, and the stamp is a timestamp against it -- if the release
    // is "started N ticks ago", the game's own full release (registration, fuse, bookkeeping)
    // fires immediately. One write to a field the game wrote 1 ms earlier; nothing is fought.
    int   gren_instant = 0;
    // Mode 2: how many ticks to backdate the stamp (60 Hz; the real windup is ~15).
    int   gren_backdate = 20;
    // Release speed in Blam units/s. Measured game throws: 8.7-9.8. (Mode 1 only.)
    float gren_speed = 9.0f;

    // ---- FORCETUBE HAPTIC GUNSTOCK (forcetube) -------------------------------------------------
    // A recoil kick on every round the player fires, through the vendor's C API DLL placed beside
    // halo_vr.dll. Off by default: most players do not own the hardware, and the feature loads a
    // third-party DLL and installs the spawn hook only when asked. See ForceTube.hpp.
    bool  force_tube = false;
    // Kick strength 0-255.
    int   force_tube_kick = 200;
    // Vendor channel: 0 all, 1 rifle, 2 rifleButt, 3 rifleBolt, 4/5 pistols, 6 other, 7 vest.
    // Player-shot filter. The radius is in BLAM UNITS (1 u ~ 3.05 m), which is why the original
    // 1.5 kicked on every NPC muzzle within four and a half metres. force_tube_fire_ms is how long
    // after your own trigger a spawn still counts as yours.
    // 1.0 blam unit (~3 m), field-verified. 0.35 rejected the player's OWN shots: the origin is
    // the muzzle, at chest height and forward, while the reference is the unit's origin at the
    // feet, so a metre is not enough. The trigger gate below is what actually excludes NPC fire,
    // which is why this can be loose.
    // Default 0.5, the value the gunstock is tuned with (halo_vr.cfg ships the same line).
    float force_tube_radius = 0.5f;
    int   force_tube_fire_ms = 250;
    int   force_tube_channel = 0;

    // ---- WRIST HUD (wristhud, doctrine in WristHud.hpp) ----------------------------------------
    // The game's own status widgets re-hosted onto the off-hand forearm. wristhudclasses is a
    // comma list of widget-class substrings to host (max 4); EMPTY runs the census only -- one
    // log line per distinct live WBP_ class -- because hosting REMOVES the widget from the flat
    // HUD and a guessed name would eat the wrong element. Fill it from the census log.
    bool  wrist_hud = false;
    // Default: the three left-wrist panels the HUD is tuned with (halo_vr.cfg ships the same line), so
    // the feature hosts panels when switched on from the menu; set it empty for the census.
    char  wrist_hud_classes[256] = "WBP_ShieldHealthBar,WBP_WeaponCradle,WBP_GrenadeCradle";
    // Second list, anchored to the AIM hand's wrist (the motion tracker's natural home), with its
    // own placement keys below -- the two forearms are mirror poses, so shared offsets fit neither.
    // Default: the value the wrist HUD is tuned with (halo_vr.cfg ships the same line).
    char  wrist_hud_classes_r[256] = "WBP_MotionTracker";
    // Controller-local anchor offset, metres (x right, y up, z back -- +z runs up the forearm).
    // Default: the value the wrist HUD is tuned with (halo_vr.cfg ships the same line).
    float wrist_hud_off[3] = {-0.14f, 0.06f, -0.41f};
    // Local orientation trim, degrees (pitch, yaw, roll), composed as a quaternion on the pose.
    // Default: the value the wrist HUD is tuned with (halo_vr.cfg ships the same line).
    float wrist_hud_rot[3] = {0.0f, 120.0f, 105.0f};
    // World scale of each quad, and the along-forearm gap (m) between stacked slots.
    float wrist_hud_scale = 0.04f;
    // Default: the value the wrist HUD is tuned with (halo_vr.cfg ships the same line).
    float wrist_hud_gap = 0.18f;
    // DrawSize (px) requested for each hosted widget's render target.
    float wrist_hud_draw = 512.0f;
    // Quad blend mode: 0 = opaque (full rect, transparent art padding goes black), 1 = masked
    // (alpha cutoff), 2 = TRANSLUCENT -- the widget's own soft alpha, and the shipped choice:
    // the masked experiment ate the HUD art's glows ("incredibly hard to see, not colorful") and
    // the field verdict was "put it back how it was, without the masked". Applied when a quad is
    // BUILT; the parser folds both values into the class-list change detection, so editing them
    // live rebuilds the panels in place. Per side, because the art differs.
    int   wrist_hud_blend   = 2;
    int   wrist_hud_blend_r = 2;
    // Gain MULTIPLIER for the right-wrist panels on top of aimwidgetgain. The left panels are
    // "perfect" at the shared gain while the tracker's dimmer glow art stays washed out dark --
    // per-side art needs per-side lift.
    float wrist_hud_gain_r = 3.0f;
    // Glance gate: 1 = the panels show only while the LEFT TRIGGER is held (and the trigger is
    // swallowed from the game while it serves the HUD); 0 = always visible.
    bool  wrist_hud_trigger = true;
    // Right-wrist placement, tuned independently of the left (wristhudoffr / wristhudrotr /
    // wristhudgapr). Same axes as the left keys, in the RIGHT controller's local frame.
    // Default: the value the wrist HUD is tuned with (halo_vr.cfg ships the same line).
    float wrist_hud_off_r[3] = {0.17f, 0.01f, -0.11f};
    // Default: the value the wrist HUD is tuned with (halo_vr.cfg ships the same line).
    float wrist_hud_rot_r[3] = {-15.0f, 195.0f, 105.0f};
    float wrist_hud_gap_r = 0.18f;
    // HUD PLACEMENT (hudplacement, a sub-setting of wristhud): 0 = on the wrists (everything above,
    // unchanged), 1 = on the sides of the held weapon. On the weapon, every panel rides the drawn weapon
    // each render frame from one anchor transform; with no weapon in hand, in a menu, a seat or a
    // cutscene it hides or falls back (hudwpnfallback). All of it lives in WristHud.cpp.
    int   hud_placement = 0;
    // The anchor the weapon-mounted panels read (hudwpnanchor):
    //   1 = the weapon actor's RootComponent world transform (the actor is attached to the arms rig at
    //       socket PrimaryWeapon, Rig.cpp).
    //   2 = the PrimaryWeapon socket of the first-person arms skeleton, read from the posed skeleton
    //       each frame (the socket the project's render-output instruments record as the drawn gun).
    //   3 = the palette weapon pose rotation at that socket (armdriver 3 only; otherwise the same as 2).
    int   hud_wpn_anchor = 2;
    // Per-panel slot in the anchor's local frame: x, y, z (cm), then pitch, yaw, roll (deg) composed on
    // the anchor rotation. The defaults assume the socket frame the render instruments use (barrel
    // along -Y, so +X is the weapon's right and +Y points back at the shooter) and turn each panel's
    // face (+X, the side a widget is read from) back toward the shooter, 35 deg outward and 25 deg up.
    // Motion tracker on the RIGHT; shield, ammo and grenades stacked down the LEFT.
    float hud_wpn_tracker[6] = { 9.0f, -4.0f,  3.0f, 25.0f,  55.0f, 0.0f};
    float hud_wpn_shield[6]  = {-9.0f, -2.0f,  9.0f, 25.0f, 125.0f, 0.0f};
    float hud_wpn_ammo[6]    = {-9.0f, -2.0f,  2.5f, 25.0f, 125.0f, 0.0f};
    float hud_wpn_grenade[6] = {-9.0f, -2.0f, -4.0f, 25.0f, 125.0f, 0.0f};
    // Any other hosted panel goes on the left below the grenade slot, this many cm apart.
    float hud_wpn_gap = 6.5f;
    // World scale of each weapon-mounted panel (the wrists use wristhudscale). The radar dots scale
    // with it.
    float hud_wpn_scale = 0.015f;
    // No usable weapon anchor (unarmed, a weapon not yet resolved): 0 = hide the panels, 1 = show them
    // on the wrists. Menus always hide them while hudplacement is 1.
    int   hud_wpn_fallback = 1;
    // About once a second: the anchor used, its world position, rotation and axes, and each panel.
    bool  hud_wpn_log = false;


    // ---- (after the author's Config.hpp line 2802)
    int   scope_res = 512;
    // 2 = SCS_FinalColorLDR: fully post-processed (bloom, reflections, tonemap); exposure forced manual below.
    // Default: the value the scope lens is tuned with (halo_vr.cfg ships the same line).
    int   scope_capture_source = 9;
    // 2 = RTF_RGBA8, 6 = RGBA16f.
    // Default: the value the scope lens is tuned with (halo_vr.cfg ships the same line).
    int   scope_rt_format = 6;
    // Manual exposure bias (EV) on the capture -- the brightness knob (live).
    // Default: the value the scope lens is tuned with (halo_vr.cfg ships the same line).
    float scope_ev = 0.5f;
    int   scope_pp_override = 1;        // 0 = do not touch the capture's PostProcessSettings
    bool  scope_cvar_dump = false;      // one-shot log of Lumen/SceneCapture console variables
    // Display multiplier on the lens (live); exposure itself is scope_ev.
    // Default: the value the scope lens is tuned with (halo_vr.cfg ships the same line).
    float scope_tint = 11.0f;
    int   scope_round = 1;              // 1 = round lens (flattened Cylinder cap), 0 = square Plane

    // ---- (after the author's Config.hpp line 2924)
    // 1 = build the capture deferred with ShowFlagSettings Lumen flags on (build-time only).
    // Default: the value the scope lens is tuned with (halo_vr.cfg ships the same line).
    int   scope_showflags = 1;
    int   scope_probe = 0;              // 1 = log the render target's centre pixel once a second (GPU readback; dev only)
    // Capture ToneCurveAmount (live): 0 = linear output (undoes the lens double-tonemap), <0 = leave alone.
    // Default: the value the scope lens is tuned with (halo_vr.cfg ships the same line).
    float scope_tone_curve = 0.5f;
    // Measured 2026-08-19: the tone-curve-0 capture path drops SEPARATE translucency (the shield
    // wall renders in the main view but not in the lens; A/B on the same spot flipped only by the
    // ToneCurveAmount write). r.SeparateTranslucency=0 folds translucency into scene colour so the
    // linear capture keeps it. GLOBAL cvar (main view too). -1 = leave alone.
    // Default: the value the scope lens is tuned with (halo_vr.cfg ships the same line).
    int   scope_sep_trans = 0;
    float scope_cam_fwd = 40.0f;        // capture camera push forward along the aim ray (cm, live) so the gun's own scope housing is not in its view
    // Real scope optics are rotationally symmetric: rolling the rifle must NOT roll the image.
    // The lens disc rolls with the gun, so the capture gets the gun's roll to cancel it out.
    // 1 = on, -1 = flipped sign (if the image rolls double instead of holding level), 0 = off.
    int   scope_roll_fix = 1;
    // Etched scope reticle: a small quad just in front of the lens showing scope_reticle.png
    // (profile dir, alpha = shape). Rolls with the gun like a real etched reticle.
    int   scope_reticle = 1;
    float scope_reticle_scale = 0.8f;   // relative to the lens size
    float scope_reticle_tint[3] = { 1.0f, 1.0f, 1.0f };
    // GPU measurement (CPU timers cannot see render cost):
    // concmd: any engine console command, executed once each time the VALUE CHANGES (live) --
    // "stat unit" / "stat GPU" put the engine's own GPU pass timings on screen if this build
    // kept its stat system. scopeabtest: capture toggles 4 s on / 4 s off while a scope is held
    // and the frame-dt of each phase is logged (SCOPE-AB) -- the delta is the capture's cost.
    char  con_cmd[256] = "";
    int   scope_ab_test = 0;
    // Eye-proximity gate: the capture (the +3-12 ms/frame second scene render, measured 2026-08-20)
    // runs ONLY while the lens is within this many cm of the camera -- i.e. while actually aiming.
    // Hip-carried, the lens keeps its last frame, like a real scope you are not looking through.
    // +8 cm hysteresis on the way out. 0 = gate off, capture always on while a scope is held. Live.
    // Default: the value the scope lens is tuned with (halo_vr.cfg ships the same line).
    float scope_eye_dist = 15.0f;
    // Capture rate cap (Hz). The engine tick runs uncapped (measured 91-133 fps) while the headset
    // shows 72 -- captures above the display rate are wasted GPU. >0 = manual CaptureScene calls at
    // this cadence (never ABOVE the engine rate; at/below it, every frame, so no strobing);
    // 0 = capture every engine frame (bCaptureEveryFrame). Live.
    float scope_hz = 72.0f;
    // Comma-separated FEngineShowFlags names forced ON for the capture when scope_showflags=1.
    // Unknown names are ignored by the engine. Applied at scope BUILD (level reload), no deploy.
    // Default: the four names the tuned scope applied. Its cfg line also listed Lighting last, but the
    // name list was split on commas and spaces only, so the line ending stayed on that last name and
    // the engine never matched it.
    char  scope_sf_names[512] = "LumenGlobalIllumination,LumenReflections,GlobalIllumination,DynamicShadows";
    // Default: the two scoped weapons the lens is tuned for, the same as the scopewpn lines halo_vr.cfg
    // ships. A scopewpn line for the same weapon replaces its entry.
    ScopeCfg scopes[8] = {
        { "FP_BattleRifle", 10.5f, { 11.5f, 0.0f, 28.5f }, { 90.0f, 85.0f, 90.0f }, 0.033f },
        { "FP_SniperRifle", 5.0f, { 0.0f, -20.0f, 9.0f }, { 0.0f, 0.0f, 0.0f }, 0.04f },
    };
    // Entries in scopes[] (scopewpn= lines). Its own counter: scope_count below is the per-weapon
    // SCOPE TRIM table (wpn_scope[]), a different table with a different element type.
    int   scope_cfg_count = 2;
    // scopelens (EXPERIMENTAL, fork, default off): the physical lens scopes above (scopewpn entries).
    // Its own master because `scope` is the author's pane: with scopelens=1 the lens owns every scoped
    // weapon it has an entry for and the author's pane toggle stands down, so two scope systems never
    // run on one weapon.
    bool  scope_lens = false;

    // ---- (after the author's Config.hpp line 3558)
    // The OFF hand punches too. Same three tests and thresholds, own detector state, shared
    // cooldown; stands down while the off hand is doing its actual jobs (mag out, grenade in
    // pouch or hand, two-hand brace, holster veto window).
    bool  melee_left      = false;

    // ---- (after the author's Config.hpp line 3634)
    // OFF-HAND shot window, ForceTube only: while the trigger is down (+ this tail) the gunstock's
    // recoil kick jolts the off hand riding it into a barely-over-threshold "swing" (measured
    // 2.31 and 2.46 against the 2.20 gate, versus 6.7-9.7 for deliberate punches). For that
    // window the swing must also have TRAVELLED melee_shot_dist metres from where it began --
    // a jolt clears the velocity gate without going anywhere, a punch goes somewhere. The main
    // hand never false-fired and is not gated. ms=0 disables.
    int   melee_shot_ms    = 250;
    float melee_shot_dist  = 0.25f;
    // OFF-HAND chop rescue: extension OR this much travel since the swing began. A vertical chop
    // arcs around the shoulder -- huge speed and over a metre of travel with the extension gate
    // never passing (measured: ext 1.43/1.80 vs the 2.20 gate, disp 1.13/0.77). Noise and the
    // gunstock kick stay under 0.1 m of travel. 0 disables, restoring extension-only.
    float melee_disp       = 0.40f;

    // ---- (after the author's Config.hpp line 3642)
    // Turning why-not instrument: one line per deadzone crossing that no gate lets through
    // (naming the gate), one line per snap that lands. For "sometimes turning works".
    bool  turn_log        = false;

    // ---- (after the author's Config.hpp line 3959)
    // THE GRAB ZONE: a cylinder along the aim ray, measured from the AIM HAND'S GRIP, which is the
    // weapon's rear hand. Along the ray this is roughly where a forestock is; across it, a hand's
    // width. Metres.
    //
    // Reusing reload_grip_mask for the button is deliberate -- it is the same physical grip. The
    // one overlap worth knowing about is the magazine fetch, which is the same button in the BELT
    // zone; the two zones only collide if you aim steeply down at your own hip.
    float two_hand_min_m    = 0.08f;
    float two_hand_max_m    = 0.80f;
    float two_hand_radius_m = 0.09f;
    // GRAB-ZONE DOT (2026-09-01, from a tester video: testers could not find the grip). A small
    // dot at the zone middle while the off hand approaches unlatched; gone once held or withdrawn.
    // Default off, the value the weapon placement is tuned with (halo_vr.cfg ships the same line).
    bool  two_hand_marker = false;
    float two_hand_marker_scale = 0.05f;

    // AGREEMENT BAND, as a dot product between the hand-to-hand line and where the weapon already
    // points. Below the minimum the line has no authority; at the full value it has all of it.
    //
    // A band rather than a threshold, because the hold is LATCHED: a latched support hand crossing
    // a hard cutoff flips the weapon between two headings in a single frame, and those headings can
    // be most of a right angle apart. Smoothstepped between the two.
    float two_hand_agree_min  = 0.35f;
    float two_hand_agree_full = 0.50f;

    // Ramp for the latch itself, milliseconds. Short enough to feel immediate, long enough that
    // grabbing the barrel sweeps the aim instead of cutting it.
    float two_hand_blend_ms = 150.0f;

    // Buzz the support hand on grab and release. It is the only feedback that the hold engaged --
    // there is nothing to see, because a correct grab barely moves the weapon.
    bool  two_hand_haptic = true;


    // ---- (after the author's Config.hpp line 4025)
    // ---- FIRST-PERSON PALETTE DISCOVERY --------------------------------------------------------
    // One-shot scan for Blam's first-person bone palette. See BlamPalette.hpp for why owning that
    // array makes the whole spawned-arms approach unnecessary.
    //
    // Re-arms on every 0 -> non-zero transition, so it can be run again with a different weapon in
    // hand without restarting: a weapon's nodes are only in the palette while it is held, which is
    // exactly the question a single boot-time scan cannot answer.
    //
    // Costs one visible hitch on the sim thread while it sweeps. That is acceptable for a discovery
    // pass and is the reason it is one-shot rather than periodic.
    int   palette_scan = 0;

    // THE PROOF WRITE. Which scanned candidate to displace, by the index the scan logged. 0 = off.
    //
    // A live session holds several 76-node first-person palettes, and shape cannot say which one is
    // on screen. Poking one and looking can. See BlamPalette.hpp.
    int   palette_poke      = 0;
    int   palette_poke_node = 0;      // 0 = the view root, so the whole rig should move
    int   palette_poke_count = 1;     // poke this many consecutive nodes from palette_poke_node
    // WEAPON OBJECT NODES (dev): find the held weapon's own Blam node block by shape and log it;
    // poke one of its nodes along Y to name the slide by eye. -1 = no poke.
    bool  wpn_node_dump = false;
    bool  wpn_node_copy_scan = false;   // dev: find render-side copies of the weapon's node block

    int   wpn_node_poke = -1;
    float wpn_node_poke_amt = 0.05f;
    // THE SLIDE RACK (2026-09-03). The off hand grips within slide_radius (m) of the pistol's
    // slide node and pulls it rearward; the node follows the hand up to slide_travel (Blam
    // units; 0.012 ~ 3.7 cm), clicks at the end, springs home on release. slide_node is the
    // index within the weapon object's own node block (Magnum: 0, named by eye).
    // slide_weapons: substring of the weapon key the rack applies to. slide_sign flips the
    // pull direction if a weapon's forward runs the other way.
    bool  slide_vr = false;
    char  slide_weapons[200] = "Magnum,Shotgun,BattleRifle,FuelRod,AssaultRifle,SniperRifle,SMG,SpikeRifle,NeedleRifle,RocketLauncher,Stanchion";
    // The PISTOL rules -- empty lock-back until racked, the phantom round, the rack firing the
    // reload -- belong to a slide, not to a pump or a charging handle. This list gates them.
    // Every weapon with a magazine part in the pak (2026-09-04 survey). Plasma pistol, plasma
    // rifle and sentinel beam: no rack, no magazine (field-observed), untouched.
    char  slide_chamber_weapons[200] = "Magnum,Shotgun,BattleRifle,FuelRod,AssaultRifle,SniperRifle,SMG,SpikeRifle,NeedleRifle,RocketLauncher,Stanchion";
    // Weapons with NO magazine at all (from the headset, 2026-09-06: the plasma rifle must not rack or
    // drop a mag): the reload button does nothing to them, the gesture machine stays idle. The
    // rack is already excluded by slide_weapons; this covers the mag drop, which applied to every
    // weapon before. Substring match on the weapon key, like the other lists.
    // Beam rifle and energy sword are heat / battery weapons with no reload (pak survey: the FP
    // roster is BP_FP_<name>_WeaponActor for AssaultRifle, BattleRifle, BeamRifle, EnergySword,
    // FlakCannon, Magnum, Needler, PlasmaPistol, PlasmaRifle, PlasmaRifle_Red, RocketLauncher,
    // SMG, Shotgun, SniperRifle, SpikeRifle, Stanchion). Every list matches the weapon key OR the
    // mesh stem (the FlakCannon class renders the FuelRodCannon mesh).
    char  reload_skip_weapons[128] = "PlasmaPistol,PlasmaRifle,SentinelBeam,BeamRifle,EnergySword";
    int   slide_node = 0;
    // Default: the value the rack is tuned with (halo_vr.cfg ships the same line).
    float slide_travel = 0.03f;
    // Default: the value the rack is tuned with (halo_vr.cfg ships the same line).
    float slide_radius = 0.16f;
    float slide_sign = 1.0f;
    // Where the slide sits relative to the AIM hand, metres in the aim-fixed frame (right, up,
    // forward). The node's own world position is the third-person gun's, not the rendered one.
    // Default: the value the rack is tuned with (halo_vr.cfg ships the same line).
    float slide_off[3] = {0.0f, 0.06f, -0.14f};
    // slide_zone: where the grab zone is. 1 = the slide PART's own rendered position (its bounds
    // centre, published every tick by the parts rebuild) -- no hand-relative guess; 0 = slide_off
    // from the aim hand (the fallback when no part exists). slide_zone_back: metres further back
    // along the barrel from the part's centre, where the serrations are.
    // slide_zone (2026-09-06, the shotgun's zone sat at the stock): 1 = the part's BOUNDS origin,
    // 2 = the part's SOCKET location on the weapon mesh, 3 = the part COMPONENT's own location.
    // Three reference points; slidelog prints all three beside the hand every second.
    int   slide_zone = 1;
    // THE RACK ZONE WINS OVER THE TWO-HAND GRIP (from the headset, 2026-09-06: the shotgun's pump is also
    // its foregrip). While the off hand sits inside the rack zone the two-hand hold does not
    // latch, so the grip press goes to the rack; a hold already latched is left alone. Per weapon
    // the zone itself can be moved on the slidebones entry: @back=m (rearward), @up=m, @right=m;
    // @everyshot locks the gun after every shot until the rack (a pump between shots).
    // @pulldown: the pull is measured downward instead of rearward (the launcher's clamp).
    // @insert=m: this weapon's own mag-in start below the seat (else the global reloadinsert).
    // @pump: the hand in the zone with the grip held takes the rack (no press), back all the way
    // then forward all the way is the cycle, no grip release and no re-grab.
    // @reloadonly makes the rack live only in the reload state (the shotgun: a chambered round
    // means the pump is just the foregrip, the hold takes it).
    bool  slide_zone_priority = true;
    // Default: the value the rack is tuned with (halo_vr.cfg ships the same line).
    float slide_zone_back = 0.0f;
    // After a reload the slide stays locked back and the trigger is dead until it is racked.
    bool  slide_lock_reload = true;
    // SLIDECHAMBER (2026-09-03): on an EMPTY gun the seat does not fire the game's reload; the
    // rack does. Seat the mag (slide stays locked back, trigger dead), rack once, and the
    // release fires the reload as the slide snaps home. A chambered-round reload is unchanged.
    bool  slide_chamber = true;
    // SLIDEPHANTOM (2026-09-03): the sim auto-reloads 375 ms after the last round leaves (measured:
    // no input, state 19 -> 5, refill 1.1 s later). To keep the manual loop, the rounds counter is
    // never allowed to read 0 at a shot: 1 -> 0 is written back to 1, the gun is remembered as
    // truly empty (trigger dead, slide held back), and the rack-fired reload puts the 0 back
    // first so the refill accounts correctly. Any failure leaves 0 in place and the game's own
    // auto-reload is the fallback. rounds_off is the u16 rounds-loaded field in the weapon
    // object (pistol: +0x2BE, read 12 -> 11 at a shot and 0 -> 12 at the refill).
    // THREE WAYS TO STOP THE AUTO-RELOAD (2026-09-04, the AR reloaded 1.2 s after a phantom):
    //   slide_phantom 1 = the 1 is written once at the 1 -> 0; 2 = re-asserted every tick while
    //   the gun is truly empty (the sim never sees 0);
    //   slide_undo_reload = a refill the player did not ask for is undone: rounds back to 0 and
    //   the refill returned to the reserve counter (found at that moment: the u16 that dropped by
    //   the refill amount, logged; reserve_off overrides), with the reload animation held idle.
    int   slide_phantom = 2;
    // COOP (2026-09-06, the log: "undid a game auto-reload" every second in a networked game and
    // the gun stayed empty): the host owns the ammo, so the phantom round and the auto-reload
    // undo only fight the replicated refill. When the session is not standalone
    // (KismetSystemLibrary.IsStandalone, checked every 2 s) both stand down; the manual reload
    // gestures and the trigger gate stay. 0 = never stand down.
    bool  coop_auto = true;
    // THE COOP DRY STOP (from the headset, 2026-09-06: "keep a record of ammo, stop before the last round
    // is spent, force our reload"): in a networked session the counter is never written; when it
    // is down to coopstopat rounds on a rack weapon the trigger locks and the slide goes back, so
    // the game never sees 0 and never auto-reloads. Our reload presses the game's from that last
    // round, the host refills, the lock clears when the count comes back up. 0 = off.
    int   coop_stop_at = 1;
    // THE HIDDEN RELOAD (from the headset, 2026-09-06: "capture the animation and freeze the pose"): in
    // coop the last round is fired and the host reloads underneath, but the moment the count hits
    // 0 the pose freezes, the reload sounds are muted, the counter display and slide sit at empty,
    // and the trigger is dead until OUR reload gesture, which is what gives the gun back. Nothing
    // is written and nothing fights the host. When on, the stop-at-one lock is not used.
    bool  coop_hide = true;
    // The hidden reload in SOLO as well (from the headset, 2026-09-07): the real last round is fired, the
    // game reloads, and it is hidden the same way until our gesture. Replaces the phantom round
    // (which stays for hidesolo=0).
    bool  hide_solo = true;
    // The hands under the hidden reload: the first-person pose can be held (blam_palette_hold_pose)
    // from the last shot until our gesture takes over. OFF by default: the whole-palette hold
    // wrecked the weapon rotation under the live placement (from the headset, 2026-09-07); a hands-only
    // hold needs the palette's node map. 0 = let the hands animate.
    int   coop_mask_ms = 0;
    // The cradle's number field by name (empty = the first all-digit TextBlock in the widget).
    char  wrist_hud_ammo_text[64] = "CurrentAmmoCountTextBlock";   // read off the cradle's tree dump, 2026-09-07
    // The animation hold in coop: the reload starts when the host answers, so the 1.2 s hold
    // could close before the game's animation began. In a networked session the hold runs
    // reloadanimmscoop, and past that it stays up until the refill has arrived (capped +4 s).
    int   reload_anim_ms_coop = 2500;
    // The synthesized reload press, ms: standalone, and networked. 90 ms stopped reloading in
    // standalone too (2026-09-06 evening: the seat press went out, the game ran nothing, a held
    // button reloaded); at 350 ms every seat press ran the game's full reload event sequence.
    // 110 ms for both (2026-09-13): in co-op, 350 ms picked up ground weapons 278-328 ms into the
    // press, and 110 ms reloaded with no pickup (no weapon change, host refill 55 -> 60 rounds).
    int   reload_press_ms = 110;
    int   reload_press_ms_coop = 110;
    bool  slide_undo_reload = true;
    int   reserve_off = -1;
    // SLIDECOPY (2026-09-03): the rack is rendered on OUR OWN copy of the weapon mesh. The real
    // first-person weapon's slide is a bone posed only by its Animation Blueprint (no Slot node,
    // a fire state the game re-asserts every 300 ms, a one-frame slide travel), so a
    // PoseableMeshComponent with the same skeletal mesh is spawned on the weapon actor, attached
    // where the real mesh is attached with the same relative transform, given the real mesh's
    // pose every tick, and its slide bone is moved by the pull; the real mesh is hidden while
    // the copy lives. slide_copy_bone: substring of the bone to move (names logged once).
    // slide_copy_axis/sign: the bone-local axis the slide travels along (0 x, 1 y, 2 z), by eye.
    // slide_copy_test: a constant offset in cm to find that axis without racking (0 = off).
    bool  slide_copy = false;
    char  slide_copy_bone[32] = "slide";
    int   slide_copy_axis = 0;
    float slide_copy_sign = 1.0f;
    float slide_copy_test = 0.0f;
    // How the real mesh is taken out of view: 1 = SetVisibility(false) (measured 2026-09-04:
    // ignored by this first-person draw), 2 = scale to 0.001 (the lever the arms hide uses).
    int   slide_copy_hide = 2;
    // slide_copy_fp: the 5.5 FirstPersonPrimitiveType given to the copy (0 none, 1 first person,
    // 2 world-space representation). slide_copy_class: 0 = PoseableMeshComponent (bones can be
    // set), 1 = SkeletalMeshComponent (reference pose only; a draw test for the class).
    int   slide_copy_fp = 0;
    int   slide_copy_class = 0;
    // The magnum asset is Nanite-skinned (NaniteSettings.bEnabled=1, 2026-09-04) and the copy
    // exposes bForceDisableNanite; 5.5's Nanite skinning is a SkeletalMeshComponent path, so a
    // poseable copy draws nothing unless it takes the classic skinned path instead.
    bool  slide_copy_no_nanite = true;
    // SLIDECOPYFOLLOWER (2026-09-04): the asset is Nanite-only, so only a SkeletalMeshComponent
    // draws it, and a SkeletalMeshComponent cannot have its bones set. Two components: the
    // poseable copy is an invisible LEADER (real pose copied in, slide bone offset), and a
    // SkeletalMeshComponent FOLLOWER renders the leader's transforms. 0 = the poseable alone.
    bool  slide_copy_follower = true;
    // Whose transforms the follower takes: 0 = the poseable leader (the design), 1 = the real
    // weapon mesh (a draw test: does a Nanite follower draw at all under a leader?).
    int   slide_copy_leader = 0;
    // SLIDECOPYMODE (2026-09-04): 0 = poseable leader + follower (dead: a Nanite follower under
    // any leader draws nothing); 1 = a SkeletalMeshComponent copy in single-node animation mode
    // playing slide_copy_seq, its time set every tick: the sequence's END is the gun at rest,
    // and the slide's release lives in its last slide_copy_rel seconds, scrubbed backward by the
    // pull. slide_copy_sweep walks the last 0.6 s slowly (dev) to find slide_copy_rel by eye.
    int   slide_copy_mode = 1;
    char  slide_copy_seq[64] = "Magnum_first_person_reload_empty";
    float slide_copy_rel = 0.25f;
    bool  slide_copy_sweep = false;
    // The sequence's root bone carries the gun's own offset ("single-node loses placement"), so
    // the copy's component is moved every tick so its root bone lands on the real gun's root
    // bone (slide_copy_root = the bone name). 0 = leave the component where it was attached.
    // slide_copy_play (draw test): 0 = paused and scrubbed (the design), 1 = the sequence just
    // plays looping, untouched, 2 = scrubbed at a near-zero rate instead of zero.
    int   slide_copy_play = 0;
    bool  slide_copy_align = true;
    // SLIDEPART (2026-09-04): the game ships the slide as its own static mesh
    // (SM_Magnum_Slide_Default). The real slide BONE is hidden (it has no children, so nothing
    // else goes), and that static mesh is spawned as our own part on the weapon actor, attached
    // where the real mesh is attached, placed every tick at the real slide bone's offset from
    // its rest (so the game's own fire cycle still shows) plus the pull along slide_part_axis.
    // Static meshes draw for us already (the dropped magazine is one); no skinning involved.
    // v2 (2026-09-04): HideBoneByName does not reach this Nanite skinning (two slides rendered),
    // so the WHOLE real mesh is scaled away and the gun is rebuilt from the shipped static parts:
    // for every bone <Stem>_M of the real mesh, the StaticMesh SM_<Weapon>_<Stem>_Default is
    // spawned as a part and placed every tick at (bone now) x inverse(bone bind pose) in the
    // mesh's frame, so every game animation carries through per bone. The slide part also gets
    // the pull; the magazine part hides while the reload has the mag out.
    bool  slide_part = true;
    char  slide_part_bone[32] = "Slide_M";
    int   slide_part_axis = 0;
    float slide_part_sign = 1.0f;           // measured: -X is rearward on the magnum
    float slide_part_test = 0.0f;           // cm, constant offset on the slide part (dev)
    int   slide_part_hide = 7;              // 7 = NATIVE (verified 2026-09-04: the game's own part components); 2 = the rebuild (fallback); 1 = HideBoneByName (dead: Nanite drops bone scale)
    // 3 = hide one material SECTION of the real mesh (ShowMaterialSection) -- slide_hide_section
    // is the material index to hide, stepped live while the slots are read off the log.
    int   slide_hide_section = -1;
    // 4 = swap the slide's material slot for an invisible material: slide_hide_mat is a name
    // substring of a loaded material to use (candidates are logged), slide_hide_mat_slot the
    // slot (-1 = slide_hide_section). Mode changes re-spawn live. slide_hide_pbo: HideBoneByName's
    // physics option for mode 1 (0 none, 1 terminate).
    char  slide_hide_mat[48] = "Invisible";
    int   slide_hide_mat_slot = -1;
    int   slide_hide_pbo = 0;
    // slide_part_bind: where a part's rest transform comes from. 0 = the real mesh's LIVE pose
    // at spawn (measured 2026-09-04: the shipped static parts are authored at the live idle
    // pose; the slide bone sits 6.5 cm higher there than in the skeleton's bind pose), 1 = a
    // reference-pose copy's bind pose. slide_part_maghide: the magazine part hides while the
    // reload has the mag out. slide_part_shadow: the parts cast shadows.
    int   slide_part_bind = 1;   // unused when parts sit at their bones (pivot 1); kept for the delta rule
    // slide_part_pivot: how a part's mesh is authored. -1 = decide per part from its bounding box
    // (geometry centred near the pivot = authored around its BONE, placed at the bone; geometry
    // far from the pivot = authored around the gun's ORIGIN, placed by the bone's delta from
    // rest); 0 = treat every part as origin-authored; 1 = every part as bone-authored.
    int   slide_part_pivot = 1;    // measured 2026-09-04: every magnum part is authored around its own bone
    // slide_part_mat: the parts take the real first-person mesh's material (slot 0) instead of
    // their own defaults (the shipped parts carry a different instance; textures looked wrong).
    // slide_part_orphans: shipped parts no bone claims by name (the magnum's Shroud) are spawned
    // too, riding the body bone by delta; 0 leaves them out.
    bool  slide_part_mat = true;
    bool  slide_part_orphans = true;
    // THREE APPROACHES IN ONE BUILD (the house rule, 2026-09-04):
    //   A  slideparthide=2  the gun rebuilt from its shipped parts (real mesh scaled away);
    //   B  slideparthide=5  the real slide BONE hidden and PARKED FAR AWAY: a hidden bone is
    //      dropped from evaluation, so its local transform is never rewritten; the real mesh's
    //      bone-space transform array is found by shape in the component and the slide's entry
    //      is pushed slide_far_cm away every tick (Nanite drops scale, not translation). The
    //      slide part rides on top; the rest of the gun stays the game's own.
    //      slide_far_array: 0 = the bone-space array, 1 = the component-space arrays (a control).
    //   C  slideparthide=6  a MORPH TARGET on the real mesh: names are logged; slide_morph =
    //      "Name,Weight" applies one every tick.
    float slide_far_cm = -1000.0f;
    int   slide_far_array = 0;
    char  slide_morph[64] = "";
    bool  slide_part_maghide = false;   // superseded by slide_part_magdrop
    // slide_part_magdrop: on the reload press the gun's own MAGAZINE PART detaches with physics
    // and falls (the copy drop stands down); a fresh part appears on the seat.
    bool  slide_part_magdrop = true;
    // How the dropped magazine part gets its physics (2026-09-04: the copy drop drifted through
    // everything -- simulate with no collision shapes). 0 = simulate the part directly, its
    // collision shape counts and IsSimulatingPhysics logged; 1 = an invisible BOX proxy sized
    // from the part's bounds simulates and the part rides it; 2 = direct, with the object type
    // and channel responses set explicitly rather than by profile name.
    int   slide_part_dropmode = 1;
    // THE REAL MESH'S CHILDREN (2026-09-04: the AR's ammo counter is a component attached to the
    // real mesh and shrank away with it). slide_part_kids: 0 = leave them; 1 = re-parent every
    // component attached to the real mesh onto our body part, keeping world, restored on
    // teardown; 2 = give them absolute scale only (a control). slide_part_ui: also spawn the
    // shipped UI_ meshes as parts (their counters would not be live).
    int   slide_part_kids = 0;
    // slide_part_frame: the frame the pull axis is defined in. 1 = the gun mesh's own frame
    // (-X rearward on every weapon measured), converted into each rack socket's local frame;
    // 0 = the socket's local axis directly (the AR's charging handle went up and down).
    int   slide_part_frame = 1;
    // NATIVE PARTS (2026-09-04, found in the child list): the game's first-person gun is ALREADY
    // separate static mesh components (BPC_FP_StaticMesh_C) attached to the skeleton's bone
    // sockets -- body, magazine, shroud, slide, trigger, bullet. slideparthide=7 touches nothing
    // but the game's own rack component: its relative location is offset from its socket by the
    // pull (the socket still carries the game's fire cycle and lock-back), and the magazine
    // drop is a physics copy on a box proxy. No hide, no rebuild, no materials, no counters lost.
    bool  slide_part_ui = false;
    bool  slide_part_shadow = false;
    // Per-weapon rackable bone: "WeaponStem:Bone,WeaponStem:Bone,..." (stem = the SK_<Stem>_
    // asset name's middle, e.g. Magnum, Shotgun, BattleRifle). A weapon not listed falls back
    // to slide_part_bone. The pak (2026-09-04) names: Magnum Slide_M, Shotgun PumpJnt_M,
    // BattleRifle Ophandle_M, FuelRodCannon Slider_M.
    // slidebones in the cfg is ADDITIVE (2026-09-06): each line appends per-weapon entries that
    // take precedence over the shipped table, so one short line retunes one gun
    // ("slidebones=Shotgun:PumpJnt_M@reloadonly@back=-0.08") without retyping the table.
    char  slide_bones_override[512] = "";
    char  slide_bones[320] = "Magnum:Slide_M@back=0.03@insert=0.20,Shotgun:PumpJnt_M@reloadonly@everyshot@pump,BattleRifle:Ophandle_M,FuelRodCannon:Slider_M,AssaultRifle:Ejector_L,SniperRifle:Ejector_L,SMG:EjectorJnt_R,SpikeRifle:Hammer_M+HammerPin_M,NeedleRifle:EnergyRotatorJnt_M,RocketLauncher:Upperpart_R@rot@pulldown@up=-0.06@right=-0.05";
    // A rack spec may name several parts joined by + (they move together: spiker hammer and
    // pin) and end in @rot for a HINGE (the rocket launcher's clamp): the parts rotate about
    // slide_part_rot_axis (0 pitch, 1 yaw, 2 roll) by slide_part_rot_deg per cm of pull.
    // Default: the value the rack is tuned with (halo_vr.cfg ships the same line).
    int   slide_part_rot_axis = 2;
    float slide_part_rot_deg = 6.0f;
    float slide_part_open_deg = 60.0f;   // a hinge's OPEN angle on the reload press (held until racked closed)
    // Weapons whose rack is required after EVERY seat, not only an empty one (the rocket
    // launcher's clamp is part of the reload).
    char  slide_always_weapons[96] = "RocketLauncher,Shotgun";   // the shotgun: every shell wants the pump (2026-09-06)
    char  slide_copy_root[32] = "Root_M";
    int   rounds_off = 0x2BE;
    // SLIDEHOOK: apply the pull inside a pre-hook on the sim's object-node render capture, right
    // before it copies the weapon's node block for the renderer (found 2026-09-03). Prologue-
    // gated; 0 falls back to the sim-hook and game-thread writes, which do not render.
    bool  slide_hook = true;
    // SLIDEMONTAGE (2026-09-03): the first-person slide is a UE bone posed only by the weapon's
    // Animation Blueprint (the Blam object nodes drive the hidden third-person gun: a pre-copy
    // write proven landing every tick changed nothing on screen). So the rack drives it through
    // the blueprint: a real weapon animation played as a dynamic montage in the ABP's slot at
    // play rate ~0, its position scrubbed by the hand between slide_seq_fwd (slide home) and
    // slide_seq_back (slide fully back) seconds. slide_seq_sweep > 0 (dev) ignores the hand and
    // sweeps the whole sequence over that many seconds, logging the time, to find those two.
    bool  slide_montage = false;   // DEAD: the weapon ABP has no Slot node (2026-09-03)
    char  slide_seq[64] = "Magnum_first_person_fire";
    char  slide_slot[32] = "DefaultSlot";
    float slide_seq_fwd = 0.0f;
    float slide_seq_back = 0.08f;
    float slide_seq_sweep = 0.0f;
    // SLIDEFIRE: drive the slide through the weapon ABP's own FIRE state. The state variable is
    // held at slide_fire_state (the enum value the ANIMSTATE probe names at a shot) and the
    // mesh's GlobalAnimRateScale is steered so the fire animation's time follows the hand from
    // 0 to slide_fire_back seconds (slide fully back), then runs to the end on release.
    // Default: the value the rack is tuned with (halo_vr.cfg ships the same line).
    bool  slide_fire = false;
    int   slide_fire_state = 19;     // measured 2026-09-03: a shot takes FirstPersonState 0 -> 19 for 0.8 s
    // Default: the value the rack is tuned with (halo_vr.cfg ships the same line).
    float slide_fire_back = 0.10f;
    // The idle-to-fire transition needs rate-scaled time to blend in; a frozen rate at entry
    // stalls it and nothing ever shows. The first slide_fire_entry seconds run at rate 1.
    // Default: the value the rack is tuned with (halo_vr.cfg ships the same line).
    float slide_fire_entry = 0.0f;
    // Where the slide is HOME again on the way forward. On release the animation runs to here
    // and the state is handed back -- not to the end of the recoil, where the game's 300 ms
    // idle re-assertions restart the fire and the slide cycles again (seen 2026-09-03).
    float slide_fire_fwd = 0.15f;
    bool  slide_marker = false;   // a dot at the grab zone, to set the zone by eye
    float slide_marker_size = 0.06f;   // the dot's size (metres)
    // WPNAMMODUMP (dev): log 16-bit fields of the weapon object that drop by one per tick --
    // fire a few rounds to name the rounds-loaded counter.
    bool  wpn_ammo_dump = false;
    // SLIDEWATCH (dev): a READ+WRITE hardware watch on the slide node's position.y in the weapon
    // object. Names the sim's rebuild (writer) and the mesh sync's read (reader) by module+RVA,
    // which is where a hook goes. Shares DR0 with palettewatch (palettewatch wins).
    bool  slide_watch = false;
    // AMMOSEQ / AMMOSCRUB (dev): point the weapon AnimBP's FirstPersonPrimaryAmmunition slot at
    // the AnimSequence whose name contains ammo_seq (empty/off = the game's own restored), and
    // sweep PrimaryAmmunition_ExplicitFrame 0..N-1 at ammo_scrub frames per second (0 = no sweep).
    char  ammo_seq[96] = "";
    float ammo_scrub = 0.0f;
    // THE DROPPED MAGAZINE: on the reload press a physics copy of the gun's own magazine mesh
    // falls from the magwell; removed after mag_drop_ms. Presentation only.
    bool  mag_drop = true;
    int   mag_drop_ms = 5000;
    bool  slide_log = false;
    float palette_poke_amt  = 0.35f;  // Blam units, chosen to be unmissable rather than subtle

    // Hardware write-watch on one node of candidate N, to catch WHICH function poses the rig.
    // 0 = off, and turning it off is what prints the ranked list of writers. Uses palettepokenode
    // to pick the node. See BlamPalette.hpp.
    int   palette_watch = 0;

    // Own one of the two pose passes the watch found. 0 = off, 1 = dll+0x257680, 2 = dll+0x257ED0.
    // Transforming after the wrong one is overwritten by the other, so this is decided by trying.
    // The only setting here that can crash the game; see BlamPalette.hpp.
    int   palette_hook = 0;
    // Which scanned candidate the hook displaces, as a visible proof. 0 = hook installed but inert.
    int   palette_hook_test = 0;

    // Move the weapon branch (nodes 7, 8, 22) to follow the aim hand, by one rigid delta so the
    // stock animation inside the branch survives. Replaces rigmode entirely when it works.
    //
    // NO trim knobs and NO borrowed rig calibration on this path, deliberately (they existed and
    // were removed 2026-08-14): the composition is the 0.5 implementation's, field-proven, plus
    // one measured view-lock yaw -- and every constant that remains is the Page Up fix's job.
    // Hand-tunable knobs on a solved frame are how six coupled wrongs impersonate one right.
    bool  palette_weapon = false;
    // Grip offset in the WEAPON's own frame, centimetres: X along the barrel, Y left, Z up.
    // The weapon's authored origin is not at its grip, so placing it at the hand hangs it forward.
    float palette_weapon_off_x = 0.0f;
    float palette_weapon_off_y = 0.0f;
    float palette_weapon_off_z = 0.0f;
    // UEVR WORLD SCALE, as the palette path applies it: metres of real hand movement -> game cm
    // is 100 x this. It is the SAME physical quantity as rigscale/100 and as UEVR's own
    // VR_WorldScale, and it must agree with whatever UEVR is actually applying -- a mismatch
    // reads as the weapon sitting a fixed fraction too close or too far, growing with reach.
    // The live value was hand-fitted while the FirstPersonScale squash and the second writer
    // were both live; both are gone, so that fit is stale by construction. Re-derive from the
    // effective world scale (config.txt VR_WorldScale, or the active camera preset), do not
    // carry the old number forward.
    float palette_weapon_scale = 1.0f;

    // THE RIGID GRIP OFFSET -- 0.5's PoseOffset, applied UPSTREAM to the controller pose in the
    // publisher, so the palette arithmetic never learns a calibration exists. Quaternion (4,
    // right-multiplied: a fixed wrist angle at any orientation) then translation (3, METRES, in
    // the CONTROLLER's frame, rotated by the corrected pose at apply time -- the rigid attachment).
    // Machine-written by the Page Up freeze-and-align gesture into the calibration file. Repeated
    // captures COMPOSE onto the existing offset, so a match refines rather than resets.
    float grip_fix[7] = {0,0,0,1, 0,0,0};
    bool  grip_fix_valid = false;
    // Same shape, for the LEFT hand (the arms' free hand): quaternion x,y,z,w then translation
    // x,y,z in METRES, left controller frame. Solved by the Insert freeze-and-align on the rendered
    // left hand; repeated captures compose. A measurement -- do not hand-edit.
    // Hold to freeze the weapon, align your hand, release to solve. Page Up by default: End, Page
    // Down, Home and Delete are all taken by the existing calibrations.
    int   palette_calib_key = 0x21;
    // Report where the game puts node 8 versus where we put it. The one measurement that settles
    // whether the hand-to-Blam conversion is right.
    bool  palette_weapon_log = false;
    // JUDDERLOG (dev): the TRACE / TRACE-JUDGE / TRACE-NODE8 lines EVERY tick instead of every
    // 45, for this many lines, then silent. The judder is a per-tick event; a once-a-second
    // trace measured only the steady offset (2026-09-03).
    int   judder_log = 0;
    // PALETTELERP (2026-09-03): write the fresh pose into the CURRENT capture bank only and the
    // pose this hook wrote one build ago into the other, so the renderer's previous/current
    // blend has two real endpoints. Both banks holding the same fresh pose collapses the blend
    // and the gun and arms step at sim rate -- the judder. 0 = fresh pose in both banks.
    bool  palette_lerp = false;
    // PALETTEBANK: which capture bank(s) receive the fresh pose when palettelerp is 0.
    // -1 = both (the long-standing default), 0 / 1 = that bank only, 2 = the bank the capture
    // context names current only, 3 = the other bank only. The bank not written keeps whatever
    // the game put there. With palettelerp=1 this key is ignored (current fresh, other snapshot).
    int   palette_bank = -1;
    // PALETTECAM (2026-09-03): which camera the weapon pose is embedded against in the hook.
    // The per-tick trace fitted the rendered gun shifting by 0.86 of one tick of camera yaw in
    // the same tick and pulling a third of it back the next: the ControlRotation the hook reads
    // at build time is one tick behind the camera the frame renders under. 0 = ControlRotation
    // as read (the old behaviour); 1 = the commanded aim (g_desired_yaw/pitch, what the sim is
    // being given for this tick); 2 = the commanded aim sampled fresh from the controller in the
    // hook; 3 = ControlRotation extrapolated by its own last step.
    int   palette_cam = 14;
    // ONE SHARED AIM DIRECTION. The reticle and the barrel are computed on two paths from the same
    // controller (aim: aim pose + aimfix + sightline; barrel: palette pose + gripfix), so they
    // disagree by ~1 deg at rest and by whatever a frame of motion is worth. With this on, the
    // pullback rotates the palette pose so its measured barrel axis lies exactly on the camera
    // forward every build (roll about that axis kept from the controller). 0.5's
    // effective_controller_basis, shared between palette and fire path, is the same idea.
    bool  palette_barrel_lock = false;
    // Fixed roll of the gun about its barrel, degrees, every weapon (+ = clockwise seen from
    // behind). The global grip capture keeps pitch only, so this is the deliberate global roll knob.
    // Default: the value the weapon placement is tuned with (halo_vr.cfg ships the same line).
    float palette_roll_trim = 6.0f;
    // (palettewpnlockgain / palettewpnlockpitch / palettewpnsweep / palettewpnbasis /
    // palettewpnfix / palettewpnfixframe are RETIRED: each was a live A/B for a question the
    // world-space pullback and the upstream grip offset have since answered. Accepted by the
    // parser, ignored, no field. Do not resurrect them as knobs -- 0.5 has none, for good reason.)

    // Axis probe: replace the weapon branch with stock-plus-one-axis-nudge cycling through bone
    // X/Y/Z while the game thread reads back the rendered weapon's world response. Measures the
    // bone-to-world axis map -- the pullback's one assumed term. Off for play; stand still while
    // it runs.
    bool  palette_probe = false;
    // Centimetres per palette unit. 304.8 (10 ft per Blam unit) -- the reference's value, and
    // their pose-matrix validation puts the palette node within 4e-8 m of the 3.048-based
    // prediction across every pose. The axis probe measured ~456 on 2026-08-14, but that read
    // the UE weapon socket, which renders through the engine's first-person primitive scale
    // (Halo sets FirstPersonScale=0.15; the reference disables it with a script) AND under a
    // second, stale world-transform writer that was still running. Neither is the palette's
    // truth. Knob kept: if a clean readback still disagrees, the number goes here, not in code.
    float palette_units_cm = 304.8f;
    // Disable the engine's first-person primitive scale (Halo: FirstPersonScale=0.15) and FOV
    // override on the CameraComponent -- the reference does this in a dedicated script before
    // its palette math is even evaluated. ON by default: it is a prerequisite, not a tweak.
    // Off restores stock behaviour for A/B.
    bool  fp_scale_fix = true;
    // Keep UEVR's decoupled pitch ON on a 0.5 Hz watchdog. The OPPOSITE of 0.5's pin, and
    // deliberately: their controller never drives the camera, ours does, and with decoupled pitch
    // off the aim pitch reaches the rendered view -- it made the player sick in one session. See
    // the UEVRPIN block in the tick.
    bool  pin_uevr_frame = true;


    // ---- (after the author's Config.hpp line 4227)
    // LIFT GATE + WELL TARGET (2026-08-16). Logged: the mag "seated" 214-224 ms after the belt grab,
    // at the hip -- both hands are already within 30 cm there. Seating now needs the mag to have
    // RISEN this far (m) above where it was grabbed, and to reach the magazine WELL: a point this
    // far (m) forward of the aim hand along the aim direction, within reload_join_dist.
    // Default: the value manual reload is tuned with (halo_vr.cfg ships the same line).
    float reload_lift = 0.08f;
    float reload_well_fwd = 0.15f;
    // PER-WEAPON WELL + INSERT MARKER (2026-09-01, from a tester video: testers had no idea
    // where the mag goes). The seat target is now the weapon's own magazine component when one
    // resolved (the exact insert point, per weapon, nothing to calibrate; reload_well_fwd is the
    // fallback), and a small ring marks it while the mag is in hand.
    // Default: the value manual reload is tuned with (halo_vr.cfg ships the same line).
    bool  reload_well_marker = false;
    float reload_well_marker_scale = 0.08f;
    // MARKER COLOURS (from the headset, 2026-09-06): "r,g,b" 0..1, each zone dot its own colour so they
    // tell apart at a glance. Red = the rack zone, blue = the two-hand zone, green = the mag
    // well, amber = the grenade pouches. Empty = the mesh's own material.
    bool  marker_tint_on = false;  // fork marker colours, experimental (default off: the author's markers keep their material)   // 0 = leave every marker its own material (if a tint ever hides one)
    char  slide_marker_color[32]   = "1,0.15,0.15";
    char  two_hand_marker_color[32] = "0.2,0.5,1";
    char  well_marker_color[32]    = "0.2,1,0.3";
    char  holster_marker_color[32] = "1,0.8,0.2";
    // THE SLIDE: inside reload_join_dist (now the CAPTURE radius) the held mag leaves the hand
    // and travels into the well over this many ms, landing on the weapon's own magazine pose;
    // the reload fires on arrival. Replaces the four-tick debounce (the travel time is one).
    // Default: the value manual reload is tuned with (halo_vr.cfg ships the same line).
    int   reload_slide_ms = 100;
    // After the seat, hold the first-person pose this long so the game's own reload animation
    // does not play out over the reload the player just performed. Covers the animation's length;
    // tune per weapon if a long one shows its tail. 0 = let the animation play.
    int   reload_mask_ms = 0;
    // FAST-FORWARD the game's own reload animation after the seat (it lives in the weapon's and
    // arms' Animation Blueprints, ANIMDUMP 2026-09-02): GlobalAnimRateScale on those components
    // is set to reload_anim_rate for reload_anim_ms, then restored. The reload completes in a
    // blink instead of playing out over a magazine the player already seated. rate 0 = off.
    // Default: the value manual reload is tuned with (halo_vr.cfg ships the same line).
    float reload_anim_rate = 0.0f;
    int   reload_anim_ms   = 1200;
    // HOLD the weapon AnimBP's FirstPersonState at idle for reload_anim_ms after the seat, so the
    // reload state never enters and the animation (and its notify-driven sound) never runs.
    // ANIMVARS 2026-09-02 found the variable; the log's STATEHOLD line reports whether the game
    // re-asserts it. Shares the window with the rate fast-forward; either or both may be on.
    bool  reload_hold_state = true;
    // PAUSE the weapon's and arms' animation update (bPauseAnims) for reload_anim_ms after the
    // seat. The clamp alone loses one frame to the game's own write and that frame fires the
    // reload sound; a paused update never evaluates the transition at all.
    // MEASURED 2026-09-03: the sound still played with the update paused, so it is the Blam sim's
    // own reload event, and the pause froze the player's pose for the window. Off; the state
    // hold alone is the animation fix.
    bool  reload_pause_anim = false;

    // ---- (after the author's Config.hpp line 4451)

    // ---- POLL-RATE THROW RELEASE (holsterpollthrow). The grip release used to be noticed on the
    // ~32 Hz tick: up to ~31 ms after your fingers opened, and the game sampled the synthetic
    // press up to one input frame later still. Measured end-to-end 2026-08-27: the GAME only costs
    // ~42 ms (press to projectile spawn, 9 throws), so our detection window was the one remaining
    // shavable term. Now the tick publishes a standing verdict every tick a grenade is armed --
    // "if the grip opened right now, it is a throw, in this direction" -- and the XInput hook
    // fires the press on the grip bit's falling edge, in the SAME poll (the release note runs
    // before the press mask is composed). The hook stays clocks-and-atomics only; every judgement
    // (put-back zone, peak, direction) is the tick's, at most one tick old -- a put-back moves at
    // 0.16 m/s, ~5 mm per tick, so the staleness cannot reclassify anything.
    bool  holster_poll_throw = false;   // fork grenade-gesture variant, experimental (default off)
    // The grips as the game's pad sees them (XInput shoulder bits; 0x0100 is documented as the
    // left grip at reloadgrip). Set to 0 to disable a side.
    int   grip_mask_l = 0x0100;
    int   grip_mask_r = 0x0200;

    // ---- (after the author's Config.hpp line 4498)
    // Extra pad bits stripped from the physical input alongside the holster steal (2026-09-04:
    // left X still threw a grenade; the code it carries is stripped here once the button log
    // names it). Steal only -- never injected.
    int   steal_extra_mask = 0;
    // THE GRENADE BUTTON, through UEVR's input action like the grip (from the headset, 2026-09-04): while
    // the LEFT hand's A-face action is down, grenade_code is stripped from the pad (measured: left
    // X arrives as 0x2000 and the game throws on it). The raw code is logged the first time.
    int   grenade_swallow = 0;
    int   grenade_code = 0x2000;

    // ---- (after the author's Config.hpp line 4529)
    // ---- HANDS ON THE WHEEL (Vehicle.hpp). The record's movement pair steers the vehicle while
    // mounted (+0xAC forward / +0xB0 right, measured by correlation 2026-08-20), so the wheel is
    // a second writer on the movement address.
    int   vehicle_wheel = 0;
    // MEASURED ON THE WARTHOG, 2026-08-21: the zone sits on the drawn steering wheel. Metres in
    // the HULL's frame -- x forward, y right, z up, from the hull origin. Not head-relative: a
    // point bolted to the vehicle must not depend on where the player's head is or points.
    float veh_wheel_pos[3] = { 0.45f, -0.535f, 1.697f };
    float veh_wheel_radius = 0.24f;   // metres; the catch reaches 1.3x this, for rim grabs
    float veh_wheel_lock = 200.0f;   // hand rotation (deg) for full lock -- 90 was twitchy
    int   veh_steer_sign = 1;        // flip if the hog steers the wrong way
    int   veh_wheel_grip = 1;        // 1 = grip holds the wheel (the natural gesture); 0 = hands-in-zone alone
    // Grip is ALSO the Warthog's brake (measured 0x0200 RB, 2026-08-20), so while the wheel is
    // held those bits are swallowed -- otherwise every steering input brakes. Let go of the
    // wheel and grip brakes normally again.
    int   veh_wheel_hand = 2;   // 0 left, 1 right, 2 BOTH
    int   veh_brake_mask = 0x0000;
    // Wheel-plane tilt, degrees. POSITIVE = the top leans TOWARD the driver, NEGATIVE = away.
    // 0 = a bus wheel facing the driver, 90 = flat like a table. The hand angle is measured IN
    // this plane, which is what makes the arc feel like the drawn wheel instead of a hoop
    // floating in front of your face.
    float veh_wheel_tilt = 5.0f;
    // Visible wheel-zone ring + hand dots. Calibration scaffolding: with no driver body and no
    // FP arms in vehicles, locating an invisible zone is impossible without it. Once the zone
    // sits on the drawn wheel the wheel itself is the marker, so this is off by default.
    int   veh_wheel_marker = 0;
    // FIRST PERSON IN A VEHICLE. Halo has no first-person vehicle camera to switch on, so the
    // rendered view is MOVED to the seat: the pre-stereo callback hands us the camera position
    // as a writable pointer. 0 = off, 1 = while mounted, 2 = always (on-foot validation).
    int   veh_cam = 0;
    float veh_cam_off[3] = { 0.0f, 0.0f, 78.0f };   // cm, added after conversion
    // Seat-camera position source: 0 = the rider's own position, 1 = the VEHICLE's. 0 measured
    // better 2026-08-21: the rider position lands on 95-100% of rendered frames against 91-97%
    // for the vehicle, and it is ALREADY at the seat.
    int   veh_cam_src = 0;
    // CHASE-CAM ANCHOR PROBE: log the engine's own camera (before we overwrite it) every N
    // calls. 0 = off. Read-only measurement.
    int   veh_anchor = 0;
    // SEAT FROM THE ENGINE'S CAMERA. 1 = seat is the chase cam moved forward along the aim boom
    // (measured rigid in the aim frame: lateral scatter +-10 cm, against +-513 in world axes);
    // 0 = the old synthesised path. 2 = RIGID: camera bolted to the hog's drawn Body component,
    // seat offset learned as a slow EMA of a constant. No boom, no crossover -- normal VR
    // driving. Falls back to 1 when no VehicleActor .Body resolves.
    int   veh_cam_anchor = 2;
    // One-shot dump of the local pawn and its components while SEATED, on the rising edge.
    bool  veh_body_dump = false;
    // One-shot on mount: every component named for a vehicle plus everything within 20 m of the
    // eye. Finds the transform the hog is DRAWN from.
    bool  veh_hog_dump = false;
    // One-shot skeleton dump of the resolved hull: bone names + which bone functions exist.
    bool  veh_hog_bones = false;
    // Hide the driver's third-person body while seated (the camera sits inside it). 0 = off,
    // 1 = SetHiddenInGame + SetVisibility (measured insufficient on this build), 2 = the above
    // plus scale to 0.001, which the draw demonstrably honours.
    int   veh_hide_body = 0;
    // Low pass on the BOOM LENGTH only, seconds (anchor mode 1). The boom extends with speed
    // and that must be tracked; the frame-scale curve gap must not be.
    float veh_cam_boom_tau = 0.5f;
    // Filter order on the boom: 1 = single pole, 2 = two poles at half the time constant each --
    // same group delay, twice the rolloff. Cascaded one-poles, so it cannot ring.
    int   veh_boom_order = 2;
    // Read the mounted vehicle's facing straight out of its object (+0x1D4, found by spin test:
    // swept 3226 deg as a unit vector) instead of inferring it from travel.
    int   veh_facing = 0;
    int   veh_facing_off = 0x1D4;
    float veh_facing_bias = 0.0f;
    float veh_cam_scale = 304.8f;                   // cm per Blam world unit
    // IN-VEHICLE VIEW ORIENTATION. The vehicle can spin or roll under you; in VR that swings
    // the world around your head. With this on, the BASE view yaw follows the vehicle (forward
    // stays forward) and pitch/roll are flattened, with the headset adding free-look on top.
    int   veh_view = 0;
    int   veh_view_flat = 1;    // zero the pitch/roll the vehicle contributes
    int   veh_log = 0;


    // ---- (after the author's Config.hpp line 4533)
    // Peak speed a release must have made to count as a throw; below it the grenade goes back
    // wherever the hand is (2026-09-01, for new players who miss the pouch-cancel). 0 = off,
    // the positional-only doctrine. Throws measured >=2.04 peak, put-back 0.16 -- 1.2 is safe.
    float gren_min_throw = 0.0f;

    // ---- (after the author's Config.hpp line 4555)
    // ANIMDUMP: for 2.5 s after each seat, log the weapon's and arms' skeletal components'
    // animation state (mode, anim instance, active montage). Finds where the reload animation
    // lives, since the Blam palette hold proved it is not there.
    bool  anim_dump = false;
    // ANIMVARS: snapshot the weapon's and arms' anim-instance scalar variables at the seat and
    // log what changed at +350/+900 ms -- finds the variable that IS the reload state.
    bool  anim_vars = false;
    // ANIMOBJS (with animvars): also list every object reference on the weapon's skeletal
    // component, its anim instance and the weapon actor -- the copy-pose source mesh trail.
    bool  anim_objs = false;
    // RELOADAUDIODUMP: after the seat, list PLAYING AudioComponents (owner, sound) at +120/+400 ms
    // and the weapon's audio-tracking component's fields once. Finds the reload sound's mute target.
    bool  reload_audio_dump = false;
    // THE RELOAD SOUND IS WWISE (2026-09-04: the exe carries AkAudioEvent/AkComponent/PostEvent,
    // Engine/Plugins/Wwise ships the effect DLLs; the UE AudioComponent sweep found nothing
    // because nothing UE-side plays it). Three approaches in one build, all live:
    //   reloadwwisedump=1  at the mag drop and the seat (twice per launch), list every Ak* object
    //                      (AkAudioEvents whose name mentions the weapon, reload, mag, chamber,
    //                      bolt, slide, rack, pump) and every AkComponent with its owner, plus the
    //                      reflected signatures of AkGameplayStatics.PostEvent /
    //                      AkComponent.PostAkEvent / AkComponent.SetOutputBusVolume, once.
    //   reloadakmute=1     when the plugin presses the game's reload, every AkComponent owned by the
    //                      FP weapon (or its owner actor) gets SetOutputBusVolume(0) for
    //                      reloadanimms, then 1 -- if the sim's reload posts on the weapon's game
    //                      object this silences it and nothing else.
    //   reloadstepsound    "Weapon:drop=Event,seat=Event,rack=Event;Weapon2:...;*:drop=Event" --
    //                      the named AkAudioEvent is posted on the FP weapon at that step via
    //                      AkGameplayStatics.PostEvent. Event names come from the dump.
    bool  reload_wwise_dump = false;
    // reloadakmute (2026-09-05, three mechanisms): 1 = the event's Wwise id (inside
    // EventCookedData, located at runtime by matching GetWwiseShortId) is zeroed for the window;
    // 2 = UnloadData() at the press, LoadData() at the window's end; 3 = ExecuteAction(Stop) on
    // the weapon's reload events every tick of the window. 0 = off.
    int   reload_ak_mute = 4;
    // The mute window's own length: the sim's reload runs 2-3 s on the rifles and its sounds land
    // late in it (the AR's handle sounds came through after a 1200 ms window, 2026-09-05).
    int   reload_mute_ms = 3500;
    // WHICH VARIANT the mute covers (2026-09-05): the "player" events are what the FP animation
    // notifies post (and the state hold already keeps those from playing); the sound that still
    // came through is suspected to be the "nonplayer" variant, posted by the Blam sim for its own
    // weapon object. 0 = player, 1 = nonplayer, 2 = both. A marine reloading the same weapon type
    // inside the 3.5 s window shares the nonplayer event; that is the cost of 1 and 2.
    int   reload_mute_variant = 2;
    // AKVTDUMP (dev, 2026-09-05): the Wwise sound-engine interface object lives at exe+akvtglobal
    // (found offline: the integration's post path loads it, tests it, and calls slot 1,
    // IsInitialized). Once per launch, at the first reload press, its vtable's first akvtcount
    // slots are logged as exe RVAs so PostEvent can be identified offline and hooked.
    bool  ak_vt_dump = false;
    // THE ENGINE HOOK (2026-09-05): AK::SoundEngine::PostEvent(AkUniqueID, AkGameObjectID, flags,
    // callback, cookie, cExternals, pExternals, playingID) at exe+akpostrva, found from the
    // interface vtable dump (slot 32, the ID overload; 31 = char*, 30 = wchar_t*, MSVC lays
    // overloads out in reverse). Prologue-gated like the Blam hooks: a mismatch means no hook.
    //   reloadakmute=4  every post inside the mute window whose id is one of the weapon's reload
    //                   events (both variants, read off the event objects) is refused with
    //                   AK_INVALID_PLAYING_ID -- unless the plugin posted it. Nothing else is
    //                   touched: no ids are zeroed, no data unloaded.
    //   aklog=1         every post inside the window is logged (id, game object, flags, verdict),
    //                   capped, so a reload sound posted under an id we do not know shows up.
    int   ak_post_rva = 0xA551FA0;
    bool  ak_log = false;
    // FOLEY POSTED WITH THE RELOAD (aklog 2026-09-05, ids resolved by hashing the pak names): the
    // hands' weapongrab / weaponsettle / mechsettle events ride the sim's reload on the same
    // emitters as the mag sounds. Names hashed at runtime (Wwise's FNV-1 32 of the lowercase
    // name) and refused inside the window with the weapon's own reload ids. Comma-separated.
    char  ak_mute_names[512] = "Play_006_chm_un_spartan_player_weapongrab,Play_006_chm_un_spartan_player_weaponsettle,"
                               "Play_006_chm_ge_weaanim_player_mechsettle_small,Play_006_chm_ge_weaanim_player_mechsettle_medium,Play_006_chm_ge_weaanim_player_mechsettle_large,"
                               "Play_006_chm_ge_weaanim_player_mechmvmt_small,Play_006_chm_ge_weaanim_player_mechmvmt_medium,Play_006_chm_ge_weaanim_player_mechmvmt_large,"
                               "Play_WEP_SpikeRifle_Ready_Cockback_A,Play_WEP_SpikeRifle_Ready_Cockback_B";   // the spiker's hammer, outside its reload_ name family
    int   ak_vt_global = 0xD5710F8;
    int   ak_vt_count = 200;
    // WHEN THE GAME'S RELOAD IS PRESSED (from the headset, 2026-09-05): 1 = at the MAG DROP, so the sim's
    // reload timer (and its sounds, inside the mute window) run underneath the belt grab and the
    // seat, and the rack is the only thing left between the seat and the first shot -- no double
    // wait. 0 = the old order: at the seat, or at the rack on an empty gun. The counter is held
    // at frame 0 while the mag is out so the early refill does not show.
    int   reload_press_at = 1;
    // reloadstepsound in the cfg is ADDITIVE (2026-09-05): each line appends weapon entries that
    // take precedence over the shipped table, per weapon and step, so one short line can retune
    // one gun without retyping the table (the cfg line buffer is 256 chars).
    char  reload_step_override[1024] = "";
    // reloadstepvia: 0 = AkGameplayStatics.PostEvent on the FP WEAPON actor; 1 = the event's own
    // PostOnActor on the weapon; 2 = PostEvent on the weapon's OWNER (the pawn, which sits at the
    // player; the FP weapon actor's own transform is the third-person gun's and may be far away,
    // which would explain accepted posts nobody hears -- 2026-09-05). Each post logs the
    // distance from the camera to the actor it was posted on.
    // 3 = PostEventAtLocation AT THE GUN: the rendered FP weapon component's world location (it
    // rides the hand), so the sound sits on the firearm itself (from the headset, 2026-09-05).
    // THREE IDEAS for posts that are accepted yet inaudible (2026-09-05 evening):
    //   4 = the SIM'S OWN EMITTER: the engine PostEvent called directly on the last game object
    //       the sim posted this weapon's reload on (seen by the hook) -- whatever switch, RTPC or
    //       listener setup the sim gives its emitters comes for free. Falls back to 3 before the
    //       sim has posted anything this window.
    //   5 = the LISTENER: PostEvent on the player controller's camera manager actor, a 2D post
    //       at the ear; if this is silent the event itself is gated, not its position.
    //   reload_step_variant 1 = post the NONPLAYER variant of each event (the third-person
    //       sound, which may not be gated on a player switch the sim sets).
    int   reload_step_via = 0;
    int   reload_step_variant = 0;
    // THE GATE, most likely (2026-09-05 late): the game has an RTPC named RTPC_PlayerNonPlayer.
    // The sim sets it on the emitters it makes; the "player" reload events are surely mixed by
    // it, and an emitter left at the default is the silent one. Three ways to set it:
    //   akrtpc / akrtpcvalue  the RTPC (a loaded AkRtpc asset, by name) set on the actor of each
    //                         post just before it, through AkGameplayStatics.SetRTPCValue;
    //   akrtpcglobal=1        set globally (no actor) for the mute window, akrtpcrestore after;
    //   akstack=1             the sim's posts log a 6-frame stack as exe offsets, so the bridge
    //                         function that sets up its emitters can be read offline.
    char  ak_rtpc[64] = "RTPC_PlayerNonPlayer";
    float ak_rtpc_value = 1.0f;
    bool  ak_rtpc_global = false;
    float ak_rtpc_restore = 0.0f;
    bool  ak_stack = true;
    // THE SIM'S EMITTER RECIPE (read offline from its bridge, 2026-09-05 night): for every sound it
    // RegisterGameObj(fresh id, name) [slot 66], SetPosition(id, AkSoundPosition, 3) [70],
    // SetListeners(id, its own listener ids) [147], SetSwitch(group, state, id) [140],
    // SetRTPCValue(rtpc, value, id) [131], PostEvent [32], UnregisterGameObj [68] when done. The
    // UE integration gives its components the UE-side listener instead, which is why every post
    // through it stayed silent. Four engine functions are hooked (prologue-gated) to CAPTURE the
    // recipe the sim applies to its reload emitter (listeners, switch, RTPCs, position), then:
    //   akmimic=1  a fresh emitter of our own, set up from the captured recipe, posts each step
    //              (the drop post falls back to the actor until the sim has posted once);
    //   akmimic=2  only the captured LISTENERS are set on our AkComponent emitter before a post;
    //   akmimic=3  only the captured SWITCH and RTPCs are set on it.
    //   akmimic=4  the sim's own reload posts are NOT blocked but re-issued by the plugin, same
    //              id, same emitter, same instant, with flags 0 and no callback: if that is heard,
    //              a raw post from our code works and the emitter recipe is what fails;
    //   akmimic=5  each step's event rides the NEXT emitter the sim posts anything on inside the
    //              window (footsteps, gear), posted by the plugin on that emitter at that instant.
    // (2026-09-06: the full recipe, 1, was silent with flags=1 and no callback; 1 now posts flags 0.)
    //   akmimic=6  THE SIM'S OWN RELOAD EMITTER, KEPT ALIVE (2026-09-06, after mode 4 proved a raw
    //              post from the plugin is heard on it): the sim makes one emitter per reload
    //              sequence and unregisters it when its sequence ends; UnregisterGameObj is
    //              hooked and that one emitter's unregister is deferred to the window's end, and
    //              every step posts on it (flags 0). Before the sim has posted this window, the
    //              step is parked and rides the sim's next emitter (mode 5's mechanism).
    //   akmimic=7  AN ADOPTED SIM EMITTER, ANY TIME (from the headset, 2026-09-06: a rack or a drop makes
    //              its sound whenever it happens, reload or not): the plugin holds the NEWEST
    //              emitter the sim registers, defers its unregister, re-positions it to the sim's
    //              own listener (the head) whenever the sim positions that listener, and posts
    //              every step on it. A newer sim emitter replaces it (the old one is then
    //              unregistered for real), so it never goes stale across rooms.
    // Default: the value manual reload is tuned with (halo_vr.cfg ships the same line).
    int   ak_mimic = 7;
    // akmimic4event: in mode 4 the plugin posts THIS event (hashed) on the sim's emitter in place
    // of the game's reload sound, so the test cannot be mistaken for the game's own audio. Empty =
    // re-issue the game's sound.
    char  ak_mimic4_event[96] = "Play_WEP_Magnum_DryFire";
    int   ak_fn_setlisteners = 0xA557A70;
    int   ak_fn_setswitch = 0xA558E90;
    int   ak_fn_setrtpc = 0xA558690;
    int   ak_fn_setposition = 0xA5584D0;   // the inner function; the public one is a 12-byte stub
    int   ak_fn_register = 0xA554C50;
    int   ak_fn_unregister = 0xA55AA20;
    // THE GAME SHIPS THE RELOAD AS SEPARATE PER-STEP WWISE EVENTS (pak index, 2026-09-05):
    // "006_chm_ge_weaanim_<weapon>_<step>_player" for the magnum, AR, BR, sniper, needler, plasma
    // pistol, sentinel beam and rocket launcher ("spnker"), and "Play_WEP_..._1P" for the shotgun,
    // spiker and fuel rod. A short value expands to the 006_chm_ge_weaanim_<value>_player object; a
    // value beginning with "Play_" is an exact object name. Steps: drop (mag leaves), seat (the mag
    // is captured by the well), seated (home), rackback (pull reaches the end), rack (release),
    // open (with the drop). "<step>empty" is preferred when the gun was empty at the drop.
    // The events load lazily: the first game reload of a weapon per launch loads them, so the
    // first manual reload may find none (logged). reloadstepsound in the cfg replaces the table.
    char  reload_step_sound[2048] =
        "Magnum:drop=magnum_reloadmagout,seat=magnum_reloadfullmagin,seatempty=magnum_reloademptymagin,rackback=magnum_readyintpullback,rack=magnum_readyintrelease;"
        "AssaultRifle:drop=ar_reloadmagouta,seat=ar_reloadmagin,seated=ar_reloadmaghit,rackback=ar_reloadswitchback,rack=ar_reloadswitchfront;"
        "SMG:drop=ar_reloadmagouta,seat=ar_reloadmagin,seated=ar_reloadmaghit,rackback=ar_reloadswitchback,rack=ar_reloadswitchfront;"
        "BattleRifle:drop=br_reloadmagoutb,seat=br_reloadmagin,seated=br_reloadmaghit,rackback=br_switchback,rack=br_switchfront;"
        "SniperRifle:drop=sniperrifle_reloadmagout,seat=sniperrifle_reloadmagin,seated=sniperrifle_reloadmagsnapin,rackback=sniperrifle_reloadswitchback,rack=sniperrifle_reloadswitchfront;"
        "Needler:drop=needler_reloadfullneedleout,dropempty=needler_reloademptyneedleout,seat=needler_reloadfullstart,seatempty=needler_reloadempty_start;"
        "RocketLauncher:drop=spnker_rocket_launcher_chambereject,open=spnker_rocket_launcher_hatchopen,seat=spnker_rocket_launcher_chamberinsert,rack=spnker_rocket_launcher_hatchclose;"
        "Shotgun:seat=Play_WEP_FOL_Shotgun_Reload_ShellIn_1P,rackback=Play_WEP_FOL_Shotgun_Chamber_PumpOut_1P,rack=Play_WEP_FOL_Shotgun_Chamber_PumpIn_1P;"
        "SpikeRifle:drop=Play_WEP_SpikeRifle_Reload_MagOut_1P,seat=Play_WEP_SpikeRifle_Reload_MagIn_1P,rackback=Play_WEP_SpikeRifle_Ready_Cockback_A,rack=Play_WEP_SpikeRifle_Ready_Cockback_B;"
        "FuelRodCannon:drop=Play_WEP_FOL_FuelRodGun_Reload_Empty_A,seat=Play_WEP_FOL_FuelRodGun_Reload_Empty_C";
    // reloadakmute (second meaning, 2026-09-05): the weapon owns no AkComponent (dump), so the mute
    // is now the events themselves -- every loaded reload event of the weapon in hand has its
    // ShortId zeroed for reloadanimms around the plugin's reload press (the integration drops a
    // post with an invalid id), and a step post restores the id for the call.
    // ANIMVARSET (dev): "Name,Value" written onto the weapon's anim instance every tick. Empty = off.
    char  anim_var_set[96] = "";
    // ANIMSEQSET (dev): "SequenceNameSubstring,Time" -- weapon mesh in single-node mode on that
    // AnimSequence, held at that time. Empty = back to the AnimBP.
    char  anim_seq_set[96] = "";

    // ---- (after the author's Config.hpp line 4563)
    // THE MAG IN THE HAND (live, 2026-09-04): an offset in the hand's own frame (right, up,
    // forward; metres) and a rotation (pitch, yaw, roll; degrees) applied to the held mag before
    // the slide-in, so the seat target is untouched.
    // Default: the value manual reload is tuned with (halo_vr.cfg ships the same line).
    float reload_hand_off[3] = {0.03f, -0.03f, 0.03f};
    // ROOM ANCHOR for rendered markers (belt mag, zone dots, dropped mags): 1 = the STANDING
    // ORIGIN (the frame UEVR renders the head's offset from; the palette publisher uses it, and
    // the arms hold still when the head moves), 0 = the HMD (the old helper: a marker then
    // shifts opposite to every head motion -- "the mag moves when I move my head").
    // Default 1, the value the reload markers are tuned with (halo_vr.cfg ships the same line). Read
    // only by the fork's marker placement (manual reload, rack, wrist HUD, the marker anchor service).
    int   room_anchor = 1;   // 0 = the HMD (development's behaviour); 1 = the standing origin, what roomscale needs
    // THE INSERT IS THE HAND'S (from the headset, 2026-09-04): once the mag reaches the well it LOCKS onto
    // the well's axis, and the hand's travel up that axis is what pushes it in; it seats when it
    // is home. reload_insert_mode 1 = that; 0 = the old timed slide (reload_slide_ms), kept.
    // reload_insert: metres below the seat where the axis tracking starts clamping;
    // reload_insert_done: metres short of home that counts as seated; reload_insert_sign flips
    // the axis if a weapon's magazine goes in the other way. reload_slide_ms is then only the
    // short snap from the hand onto the axis.
    int   reload_insert_mode = 1;
    float reload_insert = 0.08f;
    float reload_insert_done = 0.01f;
    float reload_insert_sign = 1.0f;
    // Default: the value manual reload is tuned with (halo_vr.cfg ships the same line).
    float reload_hand_rot[3] = {-90.0f, 0.0f, 0.0f};

    // ================================================================================================
    // EXPERIMENTAL, PORTED FROM THE FORK: tuning fields of the palette weapon stack (armdriver mode 3),
    // the manual reload state, and the vehicle seat camera. Every one of them is inert unless its
    // feature's master is on.
    // ================================================================================================
    // SEAT PUBLISH WHILE SEATED -- three independent answers to "the camera stays behind the hog".
    //
    // The cause: a seat is stick mode, and stick mode publishes the rider only from the cached
    // control record. blam_drive_tick() drops that record every RERESOLVE_TICKS calls (~14 s at
    // the measured 46 ticks/s), and the only re-resolve sat below the stick-mode hold, so from the
    // first drop in a ride the rider position, mounted flag and speed froze. The rigid camera then
    // learned (speed read 0) and snapped (>5 m) onto that frozen point while the hull drove away.
    //
    // vehseatpub (A, default 1): the stick-mode branch re-resolves the record itself on the sim
    //   thread, exactly as the 0.2.0 extras publish did. 0 = the old record-only publish (repro).
    int   veh_seat_pub = 1;
    // vehseatdirect (B, default 0): the game tick and the render callback read the rider (+0x20,
    //   +0x0C) and the vehicle facing straight out of the object pointers the sim publish cached,
    //   so the seat does not depend on the hook or the record at all while seated.
    int   veh_seat_direct = 0;
    // vehcamguard (C, default 1): the rigid camera refuses to learn or snap its seat offset while
    //   the DRAWN hull is moving (its own measured speed, not the rider's) or while the rider
    //   publish is stale, so a frozen rider can never re-pin the camera; it holds the offset and
    //   rides the hull.
    int   veh_cam_guard = 1;
    float veh_cam_guard_speed = 150.0f;   // cm/s of hull travel that counts as moving (~0.5 wu/s)
    int   veh_cam_stale_ms = 250;         // rider publish silence that counts as stale
    // vehcamhullcheck (D, default 1): the rigid camera trusts the hull component only while it
    //   MOVES WITH the Blam vehicle. If the Blam side reports driving speed while the hull
    //   component stands still for vehcamhulldead seconds, the component is not the drawn vehicle
    //   and the ride drops to the chase-cam anchor (logged VEHCAMHULL), re-armed on the next hull.
    int   veh_cam_hull_check = 1;
    float veh_cam_hull_dead_s = 0.5f;
    // RELOAD STATE PER WEAPON. Every step of a weapon's manual reload (magazine out or in the
    // hand, the chambered round, the lock-back, the seat waiting for the rack, the press that
    // already went out, the phantom round, the hidden-reload display lock) belongs to that
    // weapon and comes back exactly as it was left when the weapon is in hand again: after a
    // swap, a vehicle ride, a cutscene, a level load.
    // reload_state_id (reloadstate): the identity a state is kept under.
    //   0 = the legacy memory (by type name, saved only when a weapon changes mid-reload,
    //       one-shot magazine hide) -- kept for A/B only;
    //   1 = per weapon TYPE name (the default: Halo carries at most one weapon of each type, so
    //       the type IS the weapon; no datum is ever waited for);
    //   2 = per weapon INSTANCE: the Blam object datum from the weapon actor's
    //       BlamObjectSynchronization.BlamObjectIndex, the type name only once the datum has
    //       stayed unreadable for reload_state_wait_ms;
    //   3 = instance, and after a level change (the PlayerController is replaced) the two
    //       weapons held last before it adopt their records by type name, once each, when their
    //       new instances have none (a load may recreate the objects under new datums).
    // reload_state_save (reloadstatesave): 1 = the live state is mirrored into its weapon's
    //   record every tick; 2 = written only on edges (the weapon leaves the hand, gesture reset).
    // reload_state_hide (reloadstatehide): how a magazine that is out stays hidden on whichever
    //   actor renders the weapon NOW (a swap spawns new components):
    //   0 = one-shot at the state change (legacy);
    //   1 = SetVisibility, re-hidden whenever the component reads visible again;
    //   2 = SetHiddenInGame re-asserted every tick (a flag visibility toggles do not clear);
    //   3 = the component scaled to 0.001, re-asserted every tick.
    // reload_state_drop (reloadstatedrop), per type only: what happens to the record of a weapon
    //   swapped for a DIFFERENT type on the ground (a same-type pickup only adds ammo, the gun in
    //   hand never changes, so nothing happens):
    //   0 = kept until a death;
    //   1 = forgotten when a type that was not among the two carried comes into the hand (the
    //       gun that left the hand is the one a pickup swaps out); needs both carried types seen;
    //   2 = kept, and forgotten only when that type returns as a PROVABLY different object: the
    //       record's saved datum and the one in hand are both readable and differ (a later pickup
    //       of another rifle). An unreadable datum never forgets anything. Not applied right
    //       after a gesture reset (vehicle) or to a record saved before a level change.
    // reload_state_death (reloadstatedeath): 0 = records survive a death; 1 = every record is
    //   forgotten when the death camera shows (raw perspective 2 during stick mode); 2 = forgotten
    //   when the first weapon in hand after a gesture reset is a different object than its record.
    // reload_state_level (reloadstatelevel): 1 = records survive a level change (the same type
    //   in hand afterwards gets its state back); 0 = a level change forgets every record.
    // reload_state_log (reloadstatelog): one line for every save, restore, identity change and
    //   re-hide, with the weapon identity and the full state.
    int   reload_state_id      = 1;
    int   reload_state_save    = 1;
    int   reload_state_hide    = 1;
    int   reload_state_wait_ms = 1500;
    int   reload_state_drop    = 2;
    int   reload_state_death   = 1;
    int   reload_state_level   = 1;
    bool  reload_state_log     = false;
    // PER WEAPON (from the headset, 2026-09-10: "the magnum mag offset needs to be up higher"). One belt
    // point does not suit every magazine: the assets have their own pivots, so a pistol mag hangs
    // lower off the same point than a rifle mag does. Entries are "WeaponSubstring:x/y/z" in the
    // same body frame and metres as reloadmagoff, separated by commas, matched the way the wpnoff
    // table matches (case-insensitive substring of the weapon key). No entry = the global above.
    // Live: the cfg is re-read in play, so this tunes in the headset without a rebuild.
    // Default: the value manual reload is tuned with (halo_vr.cfg ships the same line).
    char  reload_mag_off_w[256] = "Magnum:-0.22/-0.50/0.00";
    // THE RELOAD WHILE RUNNING (from the headset, 2026-09-11: the mag is not keeping up, reloading on the
    // move is nearly impossible). Root cause, verified in code: the seat and rack tests map the
    // hand to world through the RENDERED camera (g_cam_*, stored per frame on the render path)
    // and compare against weapon component positions read on the GAME tick. Standing still the
    // two agree; sprinting they are up to one engine tick (~31 ms) apart, and at ~5 m/s that is
    // ~15 cm of oscillating error against a 7 cm join gate.
    //   1  the hand goes through the GAME camera (PlayerCameraManager.GetCameraLocation, read on
    //      the same tick as the component positions) so both sides share one time base  [default]
    //   2  legacy transform, but the join and rack gates WIDEN by the camera's per-tick travel
    //      (capped at 30 cm), so the gate absorbs the phase error instead of measuring past it
    //   0  legacy exact (the old behaviour)
    int   reload_frame = 1;
    // RENDER-PATH MARKER RE-ANCHOR (from the headset, 2026-09-11: the mag in the hand and the grenade
    // spheres judder back and forth on the move). holster_update places markers on the ~32 Hz
    // engine tick through a camera that renders at 90 -- the wrist HUD's old defect exactly, so
    // this is the wrist HUD's fix: the tick keeps every decision and caches each visible
    // marker's ROOM pose, and the stereo callback re-anchors it on the frame's own camera. Room
    // poses are camera-invariant, so the re-anchor is exact. 0 = tick placement only (legacy).
    int   mag_render = 1;
    // HAND-ANCHORED GESTURE GEOMETRY (from the headset, 2026-09-11: racking is hard while sprinting,
    // fine when stopped). The rack zone and the seat well come from GAME component positions,
    // and while sprinting the game plays its own weapon animation on those components -- the
    // rendered gun ignores it (the palette owns the pose), so the zones swing with an animation
    // the player cannot see. With this on, each component position is converted into the AIM
    // HAND's frame, low-passed (the per-weapon offset is what survives; the bob is what dies),
    // and the zone is rebuilt from the live hand pose every tick -- anchored to the gun you SEE,
    // exactly like the slideoff fallback, but with the component's per-weapon accuracy and
    // nothing to hand-tune. 0 = raw component positions (the old behaviour).
    int   zone_hand_rel = 1;
    // WPNERR (from the headset, 2026-09-11: "lets get err down to 0"). The weapon-vs-controller error
    // instrument. Every rendered frame the sampler compares the controller's FRESH raw pose
    // against the raw pose the palette publisher last sent (and the hook last consumed), in room
    // space -- calibration cancels out, so 0 means the gun is drawn from exactly where the hand
    // is. Camera and hand speed ride along so sway can be fitted against movement. 1 = one
    // summary line per second in log.txt + a per-frame ring flushed to halo_vr_wpnerr_NNN.csv
    // beside the cfg when the key is turned back to 0 (or the ring fills). 0 = off.
    int   wpn_err_log = 0;
    // PER-FRAME POSE REPUBLISH (from the headset, 2026-09-11: "we need everything to be 0", "as smooth as
    // possible"). The publisher runs on the engine tick; the WPNERR baseline measured the pose
    // the gun draws from at ~3 ms old, err rising with hand speed (corr 0.84) and with nothing
    // else. This re-runs the publish MATH on the render path, right before the frame: fresh
    // poses, the same q_ro/anchor/swizzle chain, the gripfix translation and rotation replayed
    // from stashes the tick publisher writes. Aim hand only (the weapon is the smoothness ask);
    // the left hand and arms stay tick-published. It only refreshes a pose the tick publisher
    // has already validated -- never creates one -- and the dead-tracking reach gate is
    // replicated. 0 = tick publishing only (the old behaviour). Default 0: the value the weapon
    // placement is tuned with (halo_vr.cfg ships the same line).
    int   pal_pub_frame = 0;
    // Mode 5's lead fraction of one build interval along the smoothed camera rate. The 2026-09-03
    // fit measured the gun missing by 0.86 of a tick; dial live in the headset.
    // Default: the value the weapon placement is tuned with (halo_vr.cfg ships the same line).
    float palette_cam_lead = 0.3f;
    // ---- CAMLEAD (2026-09-12). From the headset: "any mopvement i make overshoots and comes back".
    // That is a VELOCITY-PROPORTIONAL error, zero at rest, growing with speed, settling when he
    // stops. Not noise. And the miss was already fitted once, on 2026-09-11, at 0.86 OF A TICK,
    // which is what palette_cam_lead is -- but that compensation only ever ran inside camera
    // mode 5, and mode 5 replaced the camera wholesale, throwing away the mesh-recovered camera
    // that mode 7 exists for. Mode 7 cured the two-writer race and left the lag uncorrected;
    // mode 5 corrected the lag and reopened the race. Nobody ran both.
    // Independently measured today: the mesh rotation at the render callback differs from the
    // tick's published value by 0.021 deg at rest, rising to 2.585 deg median above 200 deg/s,
    // agreeing on only 16.7% of frames. Same velocity-proportional miss, found twice.
    // This applies the mode-5 lead to WHATEVER camera the selected mode produced, in both
    // contexts, so the two fixes compose instead of excluding each other.
    //   0 = off (the behaviour of every test before now)
    //   1 = lead by palette_cam_lead ticks of the EMA-smoothed camera rate
    int   cam_lead_all = 0;
    // ---- PALETTELOCAL (2026-09-12). THE STRUCTURAL FIX, aimed at the measurement rather than
    // at another camera mode.
    //
    // Measured on the written pose: divisor -> write correlation 0.8909, hand -> write 0.4690,
    // and **hand -> divisor only 0.2142**. Gain write/hand by band: 1.71 at 1 Hz, 2.70 at 7,
    // 10.66 at 10, 28.47 at 14. So what we write is explained mostly by the DIVISOR, and the
    // divisor barely relates to the hand. The FP mesh carries large motion of its own -- idle
    // sway, breathing, weapon bob, animation -- that has nothing to do with the wrist.
    //
    // We divide that whole animated rotation out and the game multiplies it back in, so it
    // cancels ONLY if our divisor equals the mesh at the instant of the DRAW. It never does
    // (Q_render vs Q_tick agree on 16.7% of frames). The uncancelled remainder is the mesh's own
    // animation, amplified 10x at 10 Hz and 28x at 14 Hz. That is why all twelve camera modes
    // failed: every one of them was a different guess at the CAMERA, and the camera is not the
    // part that is uncorrelated with the hand.
    //
    // So stop cancelling. Write the orientation as a PURE LOCAL DELTA on the stock bone:
    //     node = stock_node (x) (rest_hand^-1 (x) hand_now)
    // No camera, no mesh rotation, no division, nothing to fail to cancel. Whatever the mesh
    // does, our bone rides it exactly as the stock weapon does -- and the stock weapon is smooth,
    // which is the whole point.
    //
    // The known cost, from the .5 reference note: the gun's WORLD orientation is then mesh times
    // wrist delta rather than the wrist absolutely, so it can be mis-aimed as the mesh moves.
    // Smooth and mis-aimed is a far better starting point than correct and juddering, because an
    // aim error that is a function of mesh state is solvable and judder has resisted everything.
    // ORIENTATION ONLY for now; position keeps its existing path, which measured residual ~0.
    //   0 = off, the camera-divided absolute orientation (every test up to now)
    //   1 = local delta on the stock bone. Flipping 0 -> 1 re-captures the rest pose, so hold
    //       your wrist where the gun should sit when I turn it on.
    int   palette_local = 0;
    // ---- PALETTELATCH (2026-09-12). ONE HAND PER FRAME.
    //
    // MEASURED by the INTRAFRAME probe: apply_weapon_branch runs ~267 times a second while the
    // game renders at 43 Hz, so about six writes land per rendered frame (the precompose stages
    // two banks, the generic path writes the live palette plus two banks, the refresh writes
    // again), and resolve_world_pullback re-reads the published pose on EVERY one. Result:
    //   same-frame writes 198-201, DIFFERING 8.0% to 17.4%, mean 0.12-0.65 deg, WORST 2.90 deg.
    // 2.90 deg is 3.0 cm of muzzle at a 60 cm lever, four to seven times a second.
    //
    // So the live palette and the two capture banks hold DIFFERENT HANDS inside one frame, and
    // the engine blends the banks per node, so the drawn weapon lands somewhere between two
    // hands with a mix that changes frame to frame. Each individual write is correct. The set of
    // them is inconsistent, and consistency is what was never checked.
    //
    // This is why palettelocal still juddered: it removed the camera and the mesh divide
    // entirely, changing the MATH, and left untouched the question of WHICH pose each write used.
    // It is also why the g_p_seq seqlock did not help. That seqlock stops one pose being mixed
    // from two frames; it cannot stop six separate reads each getting a different, individually
    // valid pose.
    //
    //   0 = off, every write re-reads the publisher (the behaviour of every test before now)
    //   1 = latch, all writes inside palette_latch_ms share one hand
    // ---- COMPGAIN (2026-09-12). FITTED, not derived. ENDERR measured the total end-to-end
    // error E = socket_world x conj(hand_world), which by construction equals
    // mesh_actual x conj(mesh_estimate), and it needs no frame conversion or buffer assumption.
    //
    //   |E| mean 90.47 / 90.30 / 90.72 / 90.86 deg, range +-15 to +-18 around 90
    //   E per-frame movement mean 5.44 / 0.97 / 3.11 / 2.98, worst 26.88 / 7.61 / 33.50 / 9.94
    //   E / hand   1.931  2.065  2.176  1.996
    //   E / camera 1.568  1.356  1.526  1.756
    //   E / mesh   2.170  1.764  1.833  2.087
    //
    // E moves at 1.96x the MESH, and that single model predicts the other two columns with no
    // free parameters: the mesh ran at 0.72-0.84 of the camera in those windows, so E = mesh^2
    // predicts E/camera of 1.44, 1.54, 1.66, 1.68 against a measured 1.57, 1.36, 1.53, 1.76.
    //
    // E = mesh x mesh has exactly one cause: the divisor is applied with the WRONG SENSE, so the
    // mesh rotation is DOUBLED instead of cancelled. If mesh_estimate = conj(mesh) then
    // E = mesh x conj(conj(mesh)) = mesh^2. It is invisible at rest, because at rest the mesh is
    // identity and so is its inverse, which is exactly why every "at rest M = identity and
    // comp_rot == cam to the decimal" check has passed for months while the judder persisted.
    //
    // Rather than hard-flip it, expose the exponent so it can be hunted live without a rebuild.
    // The applied divisor is slerp(identity, comp_inv, compgain):
    //   +1.0 = the behaviour of every test until now
    //   -1.0 = the flip this fit predicts is correct
    //    0.0 = no divide at all (equivalent to the old comp = identity experiment)
    // Intermediate values are meaningful: a partial failure to cancel lands between them.
    // VERIFY WITH SOCKROT, which must move from 1.48x toward 1.00x, and with ENDERR's E/mesh,
    // which must fall from ~2.0 toward 0.
    float comp_gain = 1.0f;
    // ---- LIFTYAW (2026-09-12). THE LOCK GAP. Fitted, and measured twice from opposite ends.
    //
    // ERRAXIS decomposed the total end-to-end error into its rotation AXIS. In the WORLD frame
    // the axis drifts, but in the CAMERA frame it is pinned on +Z at 0.957 to 0.997 in every
    // window, with |mean| 0.961 to 0.996 (1.000 would be perfectly fixed). Camera Z is up. So
    // the ENTIRE error is a YAW in the camera frame -- which is also why both compgain sweeps
    // failed: the divisor's magnitude was never the problem, its yaw is.
    //
    // And the varying part was measured at the START of this session and written off as "by
    // design": view_yaw minus cam_y has a spread of 69.68 deg, a per-frame change of p95 1.2637
    // and max 25.4826 deg, and it scales monotonically with the right hand's rate (0.016 while
    // still, 12.85 at p95 above 200 deg/s). ENDERR's worst E movement is 26.88 and 33.50 deg.
    // Same axis, same magnitude, same scaling against the hand. It is the same defect seen from
    // both ends.
    //
    // The cause is structural. The room-to-world lift uses the LOCKED view yaw (a constant, and
    // measured constant: one value, -90.0000, on all 9471 rows), while the world-to-mesh divide
    // uses the LIVE camera. Each is defensible alone. Composed, the view lock's own gap is left
    // standing in the middle, it moves as the player aims, and it lands on every bone we write.
    //
    //   0 = the locked view yaw. Every test until now.
    //   1 = the camera yaw the divisor was built from, so lift and divide share one yaw and the
    //       gap cancels by construction.
    //   2 = the yaw taken from comp_rot itself, which is exact even in the modes where the
    //       divisor is the mesh rotation rather than cam * M (mode 12 is one of those).
    // VERIFY WITH ENDERR: E per-frame movement must collapse toward 0, and SOCKROT's ratio must
    // fall from 1.48x toward 1.00x. Expect the standing 90 deg offset to REMAIN, since that is a
    // separate convention constant and a constant cannot judder.
    int   lift_yaw = 0;
    // ---- COMPLATCH (2026-09-12). ONE DIVISOR PER FRAME. The same trade palettelatch makes for
    // the hand, applied to the term that actually needed it.
    //
    // TWO WRITERS, TWO DIFFERENT DIVISOR SAMPLES. The build-time landing runs on the sim thread
    // and consumes the mesh rotation published by the PREVIOUS render callback. The render-time
    // refresh (rr_apply_banks) composes from the stock snapshot and calls
    // resolve_world_pullback(render_ctx=true), which reads the mesh LIVE in that same callback.
    // So the build writes with last frame's divisor and the refresh writes with this frame's, and
    // the renderer consumes the banks BETWEEN them -- which is the whole reason the build-time
    // landing exists. The drawn pose therefore alternates between a stale-divisor pose and a
    // fresh-divisor pose, frame to frame.
    //
    // That alternation is the judder, and it explains the numbers that survived everything else:
    //   * SOCKROT 1.71-1.82x with the barrel lock OFF and the probe's stale-sample bug FIXED.
    //   * E/mesh ~2.0. For a signal x against a HELD estimate, mean|d(x - hold(x))| / mean|dx|
    //     tends to exactly 2.0 when the hold updates at half the sample rate. That is an
    //     arithmetic identity of a stale divisor, NOT a rotation applied twice, and it is why the
    //     ratio ignored compgain (which scales magnitude, while this is a shape statistic) and
    //     ignored liftyaw (which changes a frame, not the sampling).
    //   * Q_render agrees with Q_tick on only 16.7% of frames, and the gap grows from 0.021 deg
    //     at rest to 2.585 deg above 200 deg/s. That is the staleness, already measured.
    //   * palettelatch gave every write ONE HAND and changed nothing, because the hand was never
    //     the term that differed between the writers.
    //
    // THE TRADE. Latching the divisor makes both writers use one identical sample, so they can no
    // longer disagree. The cost is that the divisor is up to one frame old for BOTH of them, i.e.
    // a UNIFORM lag. A uniform lag is smooth; alternating between no lag and one frame of lag is
    // judder. Trading the second for the first is the entire point.
    //   0 = off, each context reads its own divisor (every test until now)
    //   1 = latch, all writes inside comp_latch_ms share one divisor
    int   comp_latch = 1;
    // ---- PALETTESYNC (2026-09-12). From the headset: "why not build the weapon placement at the same exact
    // time as aiming the character?"
    // The chain, read from source: the AIM comes from derive_ctrl_angles() reading the controller's
    // aim pose in the XInput hook (MotionAimControl.cpp:332, render-rate under aimrate=1) paired with
    // a ControlRotation read in that same callback. The WEAPON's hand comes from a SEPARATE controller
    // read in blam_palette_publish_poses() on the game tick (BlamPalette.cpp:4751-4753), and the
    // camera it is divided by is read at yet another moment in resolve_world_pullback. So the hand the
    // game aimed from, the hand we place the gun at, and the aim we cancel are three samples. Only
    // the right hand drives the aim, so only right-hand motion makes them disagree -- and objects
    // placed straight from the controller, with no aim involved, are measured smooth.
    // The reference .5 build reads the pose inside the call that writes and never lets the wrist move
    // the camera, and it does not judder.
    //   0 = off (every test before now)
    //   1 = one matched pair: the aim callback snapshots the controller pose AND ControlRotation
    //       together; the publisher takes that hand, and every placement divides by that same aim.
    int   palette_sync = 0;
    // ---- POSELATCH (2026-09-12). One controller sample per frame for EVERY reader (both aim
    // writers, the placement, the republish). Doctrine at get_pose() in MotionAimControl.cpp.
    //   0 = live reads  1 = controllers at tick start  2 = controllers+HMD at tick start
    //   3 = controllers at the aim law's XInput sample
    int   pose_latch = 0;
    // ---- PALSTEP (2026-09-12). Signed cancel of the aim's last step at the gun's divisor.
    // Fit and doctrine at resolve_world_pullback. 0 = off. Try +0.94 and -0.94.
    float palette_step = 0.0f;
    int   palette_step_src = 0;   // 0 = ControlRotation step, 1 = intent step
    int   palette_step_ctx = 0;   // 0 = both writers, 1 = render refresh only, 2 = build only
    // ---- AIMDIRECTWRITE (2026-09-12). 1 = the UE-side direct aim write runs (normal).
    // 0 = skip it but keep the stick at zero, so blamangles' Blam record is the ONLY aim writer.
    int   aim_direct_write = 1;
    // The window that counts as one frame. The game renders at ~43 Hz (23 ms), so 6 ms sits
    // inside a frame and cannot span two.
    float comp_latch_ms = 6.0f;
    int   palette_latch = 1;
    // The window that counts as one frame. The game renders at ~43 Hz (23 ms), so 6 ms sits
    // comfortably inside a frame and cannot span two.
    float palette_latch_ms = 6.0f;
    // PALETTECAMSMOOTH (2026-09-11 evening, fitted by elimination): the renderer interpolates
    // the mesh transform continuously on its own side, so NO discrete counter-camera can match
    // it -- current-frame and prev-frame embeds both judder, and the stock bones are smooth
    // precisely because they counter nothing. This EMA (time constant, ms) low-passes the embed
    // camera so consecutive residuals are CONTINUOUS: the drawn gun rides the camera chase like
    // stock instead of fighting it, converging to exact whenever the camera settles. The aim ray
    // stays one-to-one throughout -- this touches only the drawn pose. 0 = off (exact embed).
    float palette_cam_smooth_ms = 0.0f;
    // PALRENDER (from the headset, 2026-09-11: "smooth as butter"). The mechanism-complete wrist-judder
    // fix. The sim builds the FP palette every ~16 ms while frames render every ~11 ms, and the
    // camera the gun is counter-rotated against keeps chasing the aim between builds, so the
    // gun's error SAWTOOTHS within every build interval and no choice of build-time camera
    // (modes 0/1/3/5, all tried in the headset, all the same) can remove it. Only re-running
    // the write per frame can. The hook snapshots each render bank's STOCK pose at build time;
    // this replays pullback + weapon branch + arms from that snapshot with the camera read
    // FRESH, every rendered frame, banks only -- the sim's live palette is never touched from
    // the render thread. Guards: seq-locked snapshot, pointers age out at 40 ms, the weapon
    // generation must match, banks re-checked writable. 0 = builds only (the old behaviour);
    // 1 = fix + numbers; 2 = NUMBERS ONLY (measure the sawtooth, write nothing) -- the baseline
    // in the same units. Either way one PALRENDER line per second: applied/skipped counts and
    // the build-to-render camera delta (mean/p95/max deg), which IS the judder amplitude.
    int   pal_render = 1;
    // FPMESH METER (2026-09-11, the decisive observation: rotating the RIGHT wrist judders the
    // LEFT hand too, so the WHOLE FP mesh steps with the camera). Read-only: every rendered
    // frame the FP mesh component's world rotation is read, the camera embedded in it recovered
    // through the measured M constant, and the per-frame step pattern printed once a second --
    // how many frames the mesh MOVED (against frames rendered: ~2/3 proves sim-rate stepping
    // under a 90 Hz view), the step size, and the gap to the yaw the frame renders at. This is
    // the numeric proof the mesh-pin fix must beat. 0 = off.
    int   fpmesh_log = 0;
    // POSE FILTER (from the headset, 2026-09-11: "cure it"). The measured remainder of the wrist judder:
    // with the hand STILL the rendered gun wanders ~0.5 cm/tick, tracking noise and wrist tremor
    // amplified by the grip lever. The cure is the ONE-EURO filter on the published pose --
    // adaptive: cutoff fcmin + beta * |filtered speed|, so at rest the cutoff is low and tremor
    // dies before the lever sees it, and the instant the hand genuinely moves the cutoff opens
    // and tracking is one-to-one. Applied in the publisher to the aim hand's grip position and
    // both pose quats, and to the left hand, so the whole rendered rig calms together. All
    // knobs live. posefilter=0 bypasses everything.
    // FPPIN (from the headset, 2026-09-11: rotating the right wrist still moves the left hand and the
    // weapon). The last structural term: the FP mesh's transform tracks the game camera, which
    // chases the aim in steps the locked view never shows, so camera rotation drags the whole
    // rendered rig. With the pin, every rendered frame sets the mesh's rotation to R(ctl pitch,
    // LOCKED VIEW yaw) * M and its position to parent + R * v0 -- the measured attachment shape,
    // but built on the yaw the frame actually renders, which is ours and never steps. The
    // palrender refresh runs right after and reads the mesh back, so the bones embed against
    // exactly what was set: mesh and bones live in one frame WE own. Requires palrender=1 (the
    // bones must be rebuilt against the pinned transform every frame); fails closed without it.
    // 0 = off (the game keeps stomping its own camera-tracked transform, the old behaviour).
    int   fp_pin = 0;
    // STOMPLOG (2026-09-11, after the pin was refuted in-headset: the game re-stomps the mesh
    // transform and fights any late write). Read-only hunt instrument: the mesh's embedded
    // camera yaw is sampled at FOUR points of every frame -- engine-tick start, stereo pre eye
    // 0, stereo pre eye 1, stereo post -- into a ring flushed to halo_vr_stomp_NNN.csv beside
    // the cfg when the key drops to 0. The transitions name WHEN the game writes the transform
    // and which camera value it carries; that timing picks the hook point for the real cure.
    int   stomp_log = 0;
    // PALBUILDGATE (2026-09-11, the flick hunt's last door). The audit found hooked_pose IGNORES
    // its capture_render_palette argument: every call -- capture or not, weapon slot 0 or 1 --
    // runs the full camera read and bone build. Two builds of one tick read the camera at two
    // different moments, and during a right-wrist chase those moments hold different cameras, so
    // the banks can be written twice per tick against two cameras: single-frame excursions, only
    // while the aim camera moves, which is the flick's exact signature. Each mode drops a class
    // of calls so the survivor set can be found in-headset, live-flippable:
    //   0 = off, every call applies (the original behaviour)
    //   1 = capture calls only (capture_render_palette == true)
    //   2 = weapon slot 0 only
    //   3 = capture AND slot 0
    //   4 = first call per slot per engine tick (later same-tick calls skipped)
    //   5 = the build writes the LIVE palette only and leaves the render banks to the per-frame
    //       refresh -- REFUTED 2026-09-11 in one try: 69% of frame starts held full stock and the
    //       rig visibly flipped stock/ours, which PROVED the renderer consumes the banks between
    //       the game's build and the next refresh. The refresh can never win that race; the
    //       build-time overwrite is essential, and the flick is the renderer catching the
    //       microseconds while the in-place recompose runs.
    //   6 = THE WINDOW CURE: the banks' next content is composed BEFORE the game's build runs
    //       (from the last stock snapshot + the current pose, mode-7 camera), and landing it
    //       after the build is snapshot-out + memcpy-in, a few microseconds instead of a full
    //       camera read and per-node recompose. Falls back to the in-place path on the first
    //       build after a weapon swap or a stale snapshot.
    // Default 6, the window cure the weapon placement is tuned with (halo_vr.cfg ships the same line).
    int   pal_build_gate = 6;
    // POSEFREEZE (2026-09-11, the bisection): 1 = the publisher stops updating the published
    // pose, so the target side of the whole pipeline is a CONSTANT by construction -- no
    // calibration solve involved (Page Up's freeze fires one on release; this does not). If a
    // frozen pose renders smooth under a right-wrist chase, the input side is guilty; if it
    // still judders, writing anything at all judders under chase and the war is write-vs-
    // consumption timing. Test key, not a feature.
    int   pose_freeze = 0;
    // TERMLOG (from the headset, 2026-09-12: "measure EVERY SINGLE LITTLE TINY BIT OF IT"). Every
    // intermediate value in the weapon chain, per build, one CSV row: the raw ControlRotation,
    // the camera actually used, the mesh quaternion, M, the composition, the view yaw, the bob,
    // the room hand, the world hand, the published aim, the world pose at each of its three
    // stages (lift, barrel lock, trim+wrot), the mesh-frame offset, and the two FINAL outputs
    // the branch consumes. Nothing is assumed innocent. Walk the columns and the first term
    // whose jitter its own inputs do not have IS the culprit -- diagnosis by arithmetic, not by
    // switch flipping. CSV beside the cfg when the key drops to 0.
    int   term_log = 0;
    // PALETTEFINAL (2026-09-11 night, the consumption hook). The probes measured the whole
    // delivery chain alternating stock/ours in lockstep -- live palette, sim banks, and the
    // render-side arena the screen consumes -- because the renderer's bank blend reads the sim
    // banks at its own instant on the render thread, and whichever writer it catches wins the
    // frame. This hooks the blend's per-node slerp (sim+0x23BF40): after each original call,
    // if the destination lies in a render arena's local-player FP palette, the node is
    // overwritten from a seqlocked GOLDEN copy of our last landing. Our bytes then win at the
    // exact consumption instant, every node, every frame -- no race left to lose. 0 = off.
    int   palette_final = 0;
    // FPANIMKILL (the player's cut, 2026-09-11 night: "just disable all fp animations"). The
    // live palette holds STOCK on 21% of hook entries because the game's animation pose-apply
    // (sim+0x25E0C0, found by the stride/offset scan: one of only four functions that touch
    // the live palette, 94 stores) re-poses it between our writes. Racing it failed all
    // night; this stops it existing: with the kill on, the pose-apply returns without
    // running, our write stands unopposed, and every FP animation problem -- reload anims
    // included -- goes with it. We already overwrite gun+arms every build and the gesture
    // suite replaced the interactions. Live-flippable; 0 = stock behaviour.
    int   fp_anim_kill = 0;
    // PALSNIFF (2026-09-11 night): the remaining stock writer passes pointers and carries no
    // addressing constants -- invisible to the static scan. This samples the live palette's
    // node 8 from a dedicated spin thread at ~microsecond cadence (read-only, SEH-guarded)
    // and records every OURS<->STOCK transition with a QPC timestamp, beside the landing and
    // build stamps. The intruder's schedule -- period, phase in the tick -- identifies it
    // behaviorally and names where the killing overwrite belongs. CSV beside the cfg on
    // key drop to 0. Hunt instrument, not a feature.
    int   pal_sniff = 0;
    // palbuildgate=7 (2026-09-11, the freeze-ladder verdict): CONSTANT bytes shiver through
    // every write path we own while writing nothing is smooth -- the shiver IS the existence
    // of two writers, the game's stock build and our overwrite, alternating at the renderer's
    // consumption instant. Mode 7 is ONE WRITER BY CONSTRUCTION: once a weapon's stock
    // snapshot is captured, g_pose_original is not called for the local FP slot at all -- our
    // precomposed bytes land on our own bytes, and stock never enters the banks again. The
    // original still runs on weapon swaps (fresh skeleton, tag, counts) and whenever the
    // precompose cannot (stale snapshot). If the capture context turns out to be armed BY the
    // original, the gun freezes under this mode -- that is the fallback signal, not a crash.
    // palrender=3 (2026-09-11, after the MESHCONST meter finally ran): M drifts up to 68.8 deg
    // during aim-hand rotation -- the mesh follows the wrist through the game's aim smoothing,
    // not directly. UE draws from a transform snapshot taken at the end of the game frame, one
    // tick older than the live read, so bones countering the LIVE camera render under the
    // SNAPSHOT camera and miss by one tick of chase -- centimetres at the muzzle, every frame,
    // only while the smoothing chases, which is only the aim wrist. Mode 3 embeds the camera
    // ONE FRAME DELAYED to match the snapshot; everything else identical to palrender=1.
    int   pose_filter = 0;
    float pose_filter_min  = 1.5f;   // Hz, the resting cutoff -- lower = calmer rest, laggier creep
    float pose_filter_beta = 40.0f;  // Hz per (m/s): how fast motion re-opens the cutoff
    float pose_filter_rbeta = 6.0f;  // Hz per (unit/s of quat rate): the rotational reopen
    float pose_filter_dcut = 1.0f;   // Hz, the derivative's own filter (the standard 1)
    // ---- MESHCONST (2026-09-12). THE JUDDER. M is called "the mesh constant" and it is not one.
    // It is measured as M = conj(cam) (x) mesh_rotation, but `cam` comes from
    // read_control_rotation and `mesh_rotation` from a reflected K2_GetComponentRotation issued
    // AFTER it, with two more reflected calls in between. Reflected calls are not free, so the two
    // halves describe two different instants. Because the FP mesh rides the camera,
    // mesh(t1) = cam(t1) (x) M_true, and therefore
    //
    //     M_measured = conj(cam(t0)) (x) cam(t1) (x) M_true = [camera turn over dt] (x) M_true
    //
    // M absorbs the camera's ANGULAR VELOCITY times the reflected-call latency. Measured on the
    // 2026-09-12 capture: M is bit-for-bit unchanged on 80.9% of frames, and when it does move its
    // per-frame change is p95 1.058 deg / max 12.71 deg, scaling monotonically with the right
    // hand's rotation rate (p95 0.050 deg while still, 0.509 at 10-30 deg/s, 1.527 at 30-80,
    // 2.354 at 80-200, 3.428 above 200). A mostly-frozen value that jumps is a single-frame
    // excursion, which is the shape of the complaint.
    //
    // M sits inside comp = cam * M, and conj(comp) is the ONE factor shared by the weapon, the
    // right wrist, the torso AND the left hand. That accounts for every observation at once:
    // both hands judder, only while the RIGHT hand moves (only the right hand drives the aim, so
    // only it moves the camera), the head is smooth (the view never goes through comp), the left
    // wrist alone is smooth (it moves no camera), and palettewpn=0 is smooth (nothing is written).
    // 1.058 deg is 1.11 cm of swing at a 60 cm muzzle; 12.71 deg is 13.3 cm.
    //
    //   0 = legacy, the contaminated measurement, kept for A/B only
    //   1 = REJECT (default): bracket the mesh read with a SECOND ControlRotation read and throw
    //       the sample away if the camera moved between the brackets. M is a constant, so a sample
    //       taken while the camera is still is the only kind worth keeping; the last good one is
    //       held meanwhile. Removes the contamination at its source instead of smoothing it.
    //   2 = PAIR: keep every sample but pair the mesh read with the MIDPOINT of the two bracket
    //       reads, the best unbiased estimate of the camera at the instant the mesh was read.
    //       Cancels the error to first order without ever rejecting a sample.
    //   3 = SEED AND FREEZE: accept under the same gate as 1, then stop re-measuring once eight
    //       clean samples have landed. If M is genuinely constant this is strictly correct and
    //       immune to the whole class of error.
    // ---- REVCLAMP (2026-09-12). THE JUDDER, and the first fix aimed at what was measured
    // on the DRAWN weapon rather than at an intermediate.
    //
    // Reversal amplitude of the drawn PrimaryWeapon socket (point 12, the only recorded
    // quantity that is the drawn result): p95 1.3795 cm, p99 4.8101, max 34.7380, with 12.5% of
    // frames reversing more than 0.5 cm, i.e. 5.4 reversals per second, on a median of exactly
    // 0.0000. Mostly nothing, then a jump.
    // The controller, converted fairly (its translation PLUS its rotation through a 60 cm lever):
    // p95 1.0689 cm, p99 3.8227, max 17.8365. Ratio drawn/hand 1.29x at p95, 1.26x at p99.
    // Our placement chain therefore adds almost nothing. The judder IS the wrist's own angular
    // reversal, multiplied by the lever arm, drawn faithfully.
    //
    // Why only the gun shows it: THE HEAD AND THE LEFT HAND HAVE NO LEVER ARM. The head draws at
    // the head, the left hand at the hand, the muzzle ~60 cm from the wrist pivot, so identical
    // angular noise is ~60x the linear travel at the muzzle. Same reason palettewpn=0 is smooth
    // (the stock gun does not follow the wrist at all) and why 90 degrees of roll is worse (roll
    // remaps which wrist axis feeds the long arm).
    //
    // And it is not hand motion. p99 is 3.6 deg of REVERSAL inside one ~20 ms frame and the max
    // is 16.7 deg. No wrist reverses 16 degrees in 20 ms, repeatedly, while trying to hold still.
    // So the orientation stream carries physically impossible spikes.
    //
    // This is NOT a smoothing filter and must not become one. Aim stays one to one. Only the
    // component of angular motion that points AGAINST the previous frame's rotation is touched,
    // and only past a threshold no wrist can reach. Forward motion, however fast, is never
    // altered; curvature, which is perpendicular, is never altered.
    //   0 = off
    //   1 = CAUSAL CLAMP (default). Limit the backward component to rev_clamp_dps * dt. Adds no
    //       latency at all, because nothing forward is ever delayed. A genuine sustained reversal
    //       still gets through, because after a frame or two the reference direction itself flips
    //       and the motion is forward again; only an ISOLATED spike is removed.
    //   2 = 3-TAP MEDIAN on the angular step vector. The textbook tool for single-sample
    //       excursions, exact on ramps and edges, but it costs one frame (~20 ms) of latency on
    //       everything, so it is the comparison arm rather than the default.
    //   3 = both, median first then the clamp.
    // ---- TREMOR (2026-09-12). THE ANSWER. Measured, on the controller's own orientation.
    //
    // The reversal clamp below caught only 0 to 4.6% of frames, so the judder is not in the
    // impossible tail. A spectrum of the angular step says why. On the dominant axis, at a
    // measured 48.0 Hz sample rate:
    //     2 Hz 0.0738   4 Hz 0.0576   6 Hz 0.0675   **8 Hz 0.1115**   10 Hz 0.0309
    //    12 Hz 0.0404  14 Hz 0.0222  16 Hz 0.0380   20 Hz 0.0403   (deg per frame)
    // A clear peak at 8 Hz, near double its neighbours, on top of a low-frequency shelf at 2 Hz.
    // Same-sign runs average 3.09 frames, which is a dominant 7.8 Hz. Lag-1 autocorrelation is
    // +0.4957, strongly POSITIVE, so this is not white tracking noise (that would be negative or
    // zero) -- it is a periodic oscillation.
    //
    // 8 Hz is PHYSIOLOGICAL HAND TREMOR, which is textbook 8-12 Hz. Volitional aiming lives
    // under ~5 Hz and shows up as the 2 Hz shelf. So the judder is the player's own hand tremor,
    // multiplied by the ~60 cm lever from wrist pivot to muzzle, drawn faithfully. It has been
    // there since the weapon was first placed on the controller because it is not a bug in the
    // placement, it is what rigid placement on a human wrist does.
    // Corroborated: the drawn gun reverses only 1.29x what the wrist does once the lever is
    // accounted for, so the chain adds nothing; and the head and the LEFT hand are smooth
    // because neither has a lever arm.
    //
    // WHY posefilter DOES NOT FIX THIS, and why it was rightly refuted. One-euro is ADAPTIVE,
    // fc = min + beta * |velocity|, so any real motion opens it wide -- exactly when the judder
    // is worst. The tremor rides ON TOP of the motion, so an adaptive filter is transparent to
    // precisely the thing that needs removing.
    //
    // The right tool separates by FREQUENCY, because intent (2 Hz) and tremor (8 Hz) are 4x
    // apart. A notch leaves DC through ~5 Hz and everything above ~12 Hz at unity gain, so aim
    // stays one to one for anything the player can intend, and only the tremor band is removed.
    //   0 = off
    //   1 = NOTCH (default) at tremor_hz with tremor_q, on the orientation. Narrow, so volitional
    //       motion passes at unity gain with almost no phase shift. The one-to-one choice.
    //   2 = FIXED single-pole low-pass at tremor_hz. Blunter and stronger, and unlike posefilter
    //       it does NOT reopen when you move, which is the whole point. Costs some lag on fast
    //       sweeps, so it is the comparison arm.
    //   3 = notch then low-pass, for the case where one notch is not enough.
    int   tremor = 0;
    // Notch centre, or low-pass cutoff for mode 2. The measured peak is 8 Hz. For mode 2 prefer
    // about 6, which keeps 2 Hz intent nearly untouched while cutting 8 Hz meaningfully.
    float tremor_hz = 8.0f;
    // Notch width. Lower Q is wider. 1.2 covers roughly 5 to 13 Hz, which spans the tremor band
    // without reaching down into volitional aiming.
    float tremor_q = 1.2f;
    int   rev_clamp = 0;
    // Degrees per second of angular REVERSAL allowed before it is treated as impossible. At ~50
    // Hz, 120 means about 2.4 deg of reversal per frame, which clips the measured tail (p99 3.6
    // deg, max 16.7) while leaving the bulk (p95 ~1.1 deg) untouched.
    float rev_clamp_dps = 120.0f;
    // Default: the value the weapon placement is tuned with (halo_vr.cfg ships the same line).
    int   mesh_const = 0;
    // Degrees of camera movement between the brackets above which the sample is contaminated.
    // Modes 1 and 3 only. The reflected-call gap runs ~10-20 ms, so at 100 deg/s of camera motion
    // a clean sample needs the camera essentially parked; this is deliberately tight.
    float mesh_const_gate = 0.05f;

    // STABILITY FIXES (stabilityfixes, Experimental): robustness fixes to the base mod's own code paths
    // that belong to no feature (features/stabilityfixes/StabilityFixes.hpp lists them). Off = as released.
    bool  stability_fixes = false;

