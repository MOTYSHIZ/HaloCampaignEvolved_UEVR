# HaloCampaignEvolved_UEVR

**[UEVR](https://github.com/praydog/UEVR) mod for Halo: Campaign Evolved.**

[![Halo Campaign Evolved VR — VR aiming and movement test](https://img.youtube.com/vi/0A8k74tC-SY/maxresdefault.jpg)](https://www.youtube.com/watch?v=0A8k74tC-SY)

*▶ [VR Aiming and Movement Test](https://www.youtube.com/watch?v=0A8k74tC-SY) — click to watch on YouTube.*

Halo in VR in some form has been a sort of white whale for me when it comes to development, as I've
made some attempts in the past for standalone titles over the years that didn't fully pan out. So I'm
happy to finally be working on something in that direction for release.

For Halo Campaign Evolved, Unreal is essentially the presentation layer, and a lot is still dependent
on Blam. So a number of workarounds were required to get aim and movement working in VR
appropriately.

> **Status: early access.** Built and tested against the Steam release of Halo: Campaign Evolved,
> on the [latest tested UEVR build](#uevr-version). Works in multiplayer!

## Features

- **6DOF motion-controller aim** — a closed-loop controller drives Halo's aim through the game's own
  input path, so projectiles, target logic, and vehicles keep working exactly as the game intends.
  Translation counts too: moving your hand moves the aim, not just rotating it.
- **World-space aim reticule** — a colored ring floating at the true aim point, with the game's own
  crosshair (and its hit marker) hosted alongside it. Colour, size, and distance are configurable.
- **Head-relative movement** — push the stick where you look, walk where you look, independent of
  where the gun points. Snap turn supported.
- **VR control layout** — crouch on right-stick-down, equipment on left-X, d-pad access via
  right-stick-up shift, all remappable. Menu-aware: in menus your right controller's B acts as
  *back* and the gameplay remaps stand down.
- **Pose-match calibration** — optional per-hardware tuning: line your controller up with the visual
  weapon to calibrate grip, and align the aim ray, both persisting across sessions (see
  [Custom calibration](#custom-calibration)). The shipped defaults work without it.
- **Live configuration** — nearly every tunable re-reads from disk within ~2 seconds, no restart.

## Requirements

- **Halo: Campaign Evolved** (Steam).
- **UEVR** — the [latest tested build](#uevr-version). Older builds will not render this game.
- A VR headset set to the **OpenXR** runtime — the shipped config selects this for you. OpenVR
  mis-assigns controller bindings; see [Runtime](#runtime-use-openxr) if yours ends up on it.
- A VR controller pair (developed against Quest touch controllers over Steam Link).

## UEVR version

**Latest tested: [nightly-01138](https://github.com/praydog/UEVR/releases/tag/nightly-01138).**

This is the build the mod is developed and tested against, and the one to use if you want a known-good
setup.

**Older builds will not work.** Halo: Campaign Evolved runs a nonstandard UE 5.5.4 that needs the
double-precision view-matrix handling introduced in 01138 — before that, the scene renders black with
a working HUD.

**Newer nightlies are untested, not unsupported.** UEVR moves quickly and later builds may well work
fine; we simply haven't verified them against this mod. If you try one, we'd genuinely like to hear
how it went — that's how this line gets updated.

## Install

1. Install the [latest tested UEVR build](#uevr-version) and run `UEVRInjector.exe` once so it
   creates its folders.
2. Download `HaloCampaignEvolved.zip` from this repo's **Releases** page.
3. In the UEVR frontend, click **Import Config** and select the downloaded zip.
   - **Manual install (no Import Config):** instead of clicking Import Config, extract the zip into
     `%APPDATA%\UnrealVRMod\HaloCampaignEvolved\` so that `config.txt` sits directly in that folder.
4. Launch Halo: Campaign Evolved flat, and wait at the **main menu**.
5. In UEVR, select `HaloCampaignEvolved` and click **Inject**.

> **Inject at the main menu only.** Injecting mid-mission is more likely to result in rendering
> issues or hangs.

## Uninstall

Delete the `%APPDATA%\UnrealVRMod\HaloCampaignEvolved\` folder.

## Required game settings

| Setting | Value | Why |
|---|---|---|
| Frame generation | **Off** | Interpolated frames fight the VR reprojection |
| Motion blur | Off (recommended) | Comfort |
| HDR | Off (recommended) | UI compositing artifacts |

## Runtime: use OpenXR

**The shipped config already selects OpenXR, so a fresh install needs nothing from you.**

If you end up on OpenVR, your controller buttons come through mapped wrong and the control layout
below won't match what your controllers actually do. Two ways that happens: you installed an earlier
build, or your UEVR frontend has a runtime selected from another game.

To fix it, either pick **OpenXR** in the UEVR frontend before injecting, or set this in
`%APPDATA%\UnrealVRMod\HaloCampaignEvolved\config.txt`:

```
Frontend_RequestedRuntime=openxr_loader.dll
```

OpenXR does have one trade-off worth knowing: losing session focus (the SteamVR dashboard, an overlay
app, a remote-desktop connection) stops controller poses updating until the game has focus and input
again — see [Known issues](#known-issues). That's an annoyance; wrong button mappings are a blocker.

## Controls (Quest-style controllers)

| Input | Action |
|---|---|
| Right controller aim | Weapon aim (the game's aim follows it) |
| Left stick | Move, relative to where you look |
| Left stick **click** | Sprint |
| Right stick left/right | Snap turn |
| Right stick **up (hold)** | Shift layer: left stick becomes the d-pad:<br>**D-pad up** — Flashlight<br>**D-pad right** — Switch grenade<br>**D-pad down (hold)** — Drop weapon |
| Right stick **down** | Crouch |
| Right stick **click** | Melee |
| Right trigger | Fire |
| Left trigger | Weapon zoom — **not working yet**, see [Known issues](#known-issues) |
| Left grip | Throw grenade |
| Right A | Jump |
| Right Y | Switch weapon |
| Left X | Equipment / overshield |
| Right B | Reload (in menus: **Back**) |

The mod ships with a working calibration out of the box — try it as-is first. If the weapon doesn't
sit right in your hand, or shots don't land where you're pointing, see **Custom calibration** below.

## Controls in vehicles and turrets

Halo aims its vehicle camera at whatever you're aiming at, so hand-aim would drag the whole camera
around with your hand. **For now, getting into any vehicle seat or mounted turret switches the mod
to classic gamepad controls**, and motion aim steps aside until you get out. Your head still looks
around freely, and motion aim comes back on its own — calibration intact — the moment you're back
on foot.

| Input | Action |
|---|---|
| Right stick | Aim and swing the camera (the game's own vehicle look) |
| Left stick | Drive — throttle and steering |
| **Either grip** | Hard brake |
| Right A | Hard brake (the game's own binding — grip does the same thing) |
| Left trigger (hold) | Handbrake — the sharp, quick turn |
| Right trigger | Fire |
| Head | Free look, as always |

Motion aim, snap turn and the VR-specific button remaps all stand down while you're seated, so
every other button does exactly what the game's normal gamepad layout does.

The switch is automatic, and it also applies during cutscenes — the mod works it out from the game
taking your first-person weapon away. If you ever find a seat it misses, set `stickforce=1` in
`halo_vr.cfg` to force these controls on and `0` to go back to automatic; that file is read while
you play, so it takes effect without restarting.

## Configuration

Everything lives in `%APPDATA%\UnrealVRMod\HaloCampaignEvolved\halo_vr.cfg`, re-read live. The
important knobs:

| Key | Default | Meaning |
|---|---|---|
| `aimhand` | right | Which hand aims — `right` or `left` (see [Left-handed aim](#left-handed-aim)) |
| `aimrate` | 1 | Aim control at render rate (0 = engine-tick rate) |
| `ffgain` | 1.0 | Velocity feedforward strength; raise if fast sweeps lag, lower if they overshoot |
| `dgain` | 0.15 | Damping against overshoot/rubber-banding |
| `dead` | 0.5 | Aim deadband in degrees |
| `aimmesh` | 0 | Optional geometric ring at the aim point (off; the game's own crosshair now shows in colour) |
| `aimmeshcr/cg/cb` | blue | Ring colour, if `aimmesh=1` (RGB 0–1) |
| `aimmeshscale` | 0.14 | Ring size |
| `aimreticuledist` | 500 | Ring distance from you, in cm |
| `aimwidget` | 1 | Host the game's own crosshair (with hit marker) at the aim point |
| `aimwidgetgain` | 256 | Brightness of the hosted crosshair. Unlit UI is scaled down by the scene's exposure, so it needs a large multiplier; lower it if the crosshair blooms |
| `hudhide` | 0 | Collapse the flat HUD crosshair once you trust the ring |
| `turnmode` / `snapdeg` | 1 / 45 | Snap turn on, 45° per step |
| `stickmode` | 1 | Switch to gamepad controls in vehicles, turrets and cutscenes (see [Controls in vehicles and turrets](#controls-in-vehicles-and-turrets)) |
| `stickforce` | 0 | `1` forces those controls on, `2` never uses them, `0` decides automatically |
| `stickon` / `stickoff` | 0.75 / 0.05 | Seconds before switching in and out |
| `brake` / `brakemode` | 1 / 3 | Grip hard brake, and how it's sent (`3` presses A) |
| `vrinactivity` | 100 | Raises UEVR's motion-controls inactivity timeout to its maximum |

The full key list with comments is in the shipped `halo_vr.cfg`.

## Left-handed aim

Set `aimhand=left` in `halo_vr.cfg`. The aim, weapon rig and reticule all follow your left
controller instead of your right.

Two things to know:

- **Each hand keeps its own calibration.** Switching handedness never overwrites the other hand's
  tuning. The first time you select left, it is seeded by mirroring your right-hand calibration,
  which gets you close — but re-run both calibrations (see [Custom calibration](#custom-calibration))
  for a proper left-hand fit.
- **The weapon model stays right-handed.** Halo's first-person arms and weapons are authored for a
  right hand, and a mod cannot mirror a skeletal mesh. The gun will be held in your left hand but
  still *look* like a right-handed weapon — magazine and ejection port on the usual side.

The control layout is unchanged: movement stays on the left stick and turning on the right. If you
would prefer those swapped for left-handed play, say so — it is a small addition, but it is a
preference rather than an obvious default.

## Custom calibration

The shipped calibration was measured on **Quest Touch controllers**, so it encodes one particular set
of hardware and one particular way of holding it. It's a starting point, not a universal fit — try
the mod as-is first, and expect to want this section if you're on different controllers.

If the visual weapon doesn't sit right in your hand, or shots don't land where you're pointing, two
keyboard-driven calibrations let you match the mod to your own hardware and grip. Both run in-mission
and **persist across sessions** once set.

| Key | Calibration | Workflow |
|---|---|---|
| `End` | **Pose-match** (grip) | Hold the key, physically line your controller up with the on-screen weapon, then release. This aligns the weapon's grip to how you actually hold your controller. |
| `Page Down` | **Aim ray** | Hold the key, point at the frozen reticle, then release. This aligns the direction shots travel with where the weapon points. The magnum was used in the original calibration. With both eyes open, I lined up the magnum sight picture with the floating reticle's center. |

Do the pose-match first (it sets where the weapon sits), then the aim-ray calibration (it sets where
that weapon shoots). If a calibration ever feels off, just repeat it — the latest one wins.

**One calibration covers every weapon.** Per-weapon calibration isn't supported yet — it's planned.
Until then, a grip tuned on one weapon is the grip used for all of them, which is why the original
calibration was done on the magnum: a middle-of-the-road result beats one that's perfect on a pistol
and wrong on a rocket launcher.

### Where your calibration is stored

Both calibrations write to a separate file next to the config:

```
%APPDATA%\UnrealVRMod\HaloCampaignEvolved\halo_vr_calib.cfg
```

It's applied *after* `halo_vr.cfg`, so anything in it overrides the matching key there. Keeping it
separate is deliberate: rewriting `halo_vr.cfg` in place would destroy the comments documenting every
other setting.

**To go back to the shipped calibration, delete `halo_vr_calib.cfg`.** It isn't part of the download —
it only exists once you've calibrated — so there's no original copy to restore, and deleting it simply
lets the shipped defaults apply again. The mod recreates it next time you calibrate.

Once you have a calibration you like, it's worth copying that file somewhere safe. It's small, it's
plain text, and it's the only thing in the profile that's specific to *you* — everything else can be
re-downloaded.

## Known issues

- **Sudden 3DoF (frozen hands) that recovers when you press something.** Three separate causes, all
  with the same symptom:
  1. **Motion controllers going to sleep.** If you set the controllers down or hold still long
     enough — a long cutscene, a pause in the menus — they sleep to save battery and stop reporting
     poses, so your weapon freezes in place. Move or press something to wake them. Most likely to
     bite during cutscenes, exactly when you're least likely to be moving.
  2. **UEVR retiring idle controllers** after its own inactivity timeout. The mod raises this to its
     maximum (100 s) via `vrinactivity`, which covers the common case.
  3. **Loss of session focus on OpenXR** — the SteamVR dashboard, an overlay app, or a remote-desktop
     connection stops controller poses updating at the runtime level. Give the game focus and press
     something to recover. Though this doesn't always appear to work. Any feedback on reliable fixes are welcome.
- **Changing your real-world stance breaks tracking.** Standing up if you started seated, or sitting
  down if you started standing, throws the weapon and the aim off — the mod's reference is tied to
  where your play space was when it started, and moving your head to a different height invalidates
  it. **Reset your play area / recentre** and it comes back. Pick a stance before you inject and stay
  in it; if you want to switch, expect to recentre afterwards.
- **Buttons mapped wrong?** You're on the OpenVR runtime. See [Runtime](#runtime-use-openxr) for the
  one-line fix.
- **No pause binding when playing over Steam Link.** Press **`Esc`** on your keyboard to pause. A
  controller binding is coming.
- **Cutscenes display oddly** — the picture doesn't resolve correctly in stereo. Closing one eye
  makes them watchable until this is fixed.
- **No zoom yet.** Pressing left trigger will hardly do anything for you. This is something that I intend to target relatively soon.
- **Your arms are in a T-pose at the very start**, before you pick up your first weapon. It corrects
  itself once you're armed.
- **The shield and overshield effects don't follow your arms.** They're driven separately from the
  first-person rig the mod moves, so when your hands go where you point them, the shield FX stay
  where the game originally put them. Most visible when the overshield is active.
- **Right-hand aiming only.** There's no left-handed mode yet.
- **Vehicles are strongly motion-sickness inducing.** Both the camera and the aim are currently
  driven by the right controller, so looking around and aiming can't be separated while driving.
  Treat vehicle sections with caution; this needs a dedicated vehicle control scheme.
- **Aim reticle jitter during fast movement** *(work in progress)*. The mod steers the game's flat
  aim very quickly to follow your hand, and the overshoot/damping tuning isn't finished. Swinging
  fast can make the reticle wobble before it settles. `ffgain` and `dgain` let you tune this
  yourself in the meantime.
- **The hosted crosshair and hit marker render dark**, and the game's own reticle reads as black —
  still being investigated. The game draws its UI into an offscreen target whose colour no material
  this game ships can reproduce on a world surface. In the meantime the mod adds its own **light
  blue ring** at the true aim point, which is the reticle you should actually use. The game's
  crosshair still tracks aim, animates, and changes per weapon; it just reads dark against bright
  scenery. Adjust the ring with `aimmeshcr/cg/cb` and `aimmeshscale`, or hide the flat crosshair
  entirely with `hudhide=1`.
- **The aim ring can be occluded by geometry.** It's drawn at a fixed distance (`aimreticuledist`,
  500 cm by default), so when you aim at something closer than that the ring sits *inside* the
  surface and disappears. Lowering `aimreticuledist` helps in tight spaces. The planned fix is to
  trace for the first surface along the aim ray and pull the ring in to meet it.
- **UI waypoints and objective markers are misplaced**, and drift with your right-hand aim rather
  than staying pinned to the world. They're positioned against the game's flat view, which the mod
  now steers with your controller — so the marker follows your hand instead of the objective.
  Navigate by the world rather than the markers for now.
- **Directional sound doesn't follow your head.** Positional audio is spatialised against the game's
  own view, not against where you're actually looking, so turning your head leaves the sound field
  behind: a firefight to your left keeps sounding like it's to your left even after you turn to face
  it. Turning with the stick *does* realign it, because that rotates the game's view as well — so
  sound stays correct relative to your body and drifts only by however far your head is turned off
  it. Worst when you rely on audio to locate something off-screen. No workaround beyond stick-turning
  toward what you're listening for; a proper fix means moving the game's audio listener onto the
  headset pose, which the mod doesn't currently touch.
- The reticule ring appears a few seconds after a mission loads (its material streams in late).
- **Injection sometimes fails even at the main menu.** It either hangs the game during injection, or
  comes up rendering **only one eye** once a mission is entered. Force-kill the game, relaunch, and inject again — it's
  intermittent, and a retry normally works. As far as we can tell this is a UEVR issue rather than a
  mod bug: the hang has been observed with the plugin removed from the profile entirely, and stereo
  rendering is UEVR's own hook, which this mod never touches.
- Mid-mission injection is unreliable — expect rendering glitches or a hang. Always inject at the
  main menu.
- Aim feel is tuned against the game's default **controller look sensitivity** — the mod steers aim
  by synthesizing stick input, so that setting scales the whole loop. It measures the real turn rate
  and adapts to other values, but give it a few seconds of turning before judging.

More in [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md).

## Building from source

C++ under `src/`, built with MSVC against the UEVR plugin SDK — no CMake, no other dependencies:

```powershell
scripts\build.ps1 -SdkPath <path-to-a-praydog/UEVR-checkout>
```

Every `.cpp` under `src/` is compiled automatically, so **adding a file needs no build-script edit**.
`scripts\build.ps1 -Deploy` also installs the DLL into your live profile, and `scripts\package.ps1`
assembles the release zip.

Full instructions, the source layout, and the conventions for splitting a new module out of
`Plugin.cpp` are in [COMPILING.md](COMPILING.md). **Contributions welcome** — the code is being
split by feature to make that easier.

## Credits

**Special thanks to [elliotttate](https://github.com/elliotttate)** — for hosting the Flat2VR
community, without which none of this work would have found the people who made it possible, and for
writing **CutsceneDetectionPlugin**, which ships in this profile's `plugins\` folder and handles
cutscene comfort (disabling decoupled pitch, switching to a 2D view and damping camera shake while a
cutscene plays). That plugin is his work, not ours.

**Special thanks to [praydog](https://github.com/praydog/UEVR)** — for UEVR itself, which every one
of these projects stands on, for the nonstandard 5.5.4 fix that had this game rendering in VR within
days of its release, and for the plugin SDK this mod is written against. Hail to the king.

- **[Pande4360](https://github.com/Pande4360) and [deterministicj](https://github.com/deterministicj)** — For welcoming me into the Flat2VR community as a modder and providing useful learning/community resources!
- **[pancreations / Halo-MCC-VR](https://github.com/pancreations/Halo-MCC-VR)** — independent prior
  art for Halo VR aim doctrine ("steer the game's own aim") and the authored-reticle approach.
- **[LunchAndVR](https://www.youtube.com/@LunchAndVR)** — For creating the community UEVR profile this
  configuration descends from, and for bundling elliotttate's cutscene plugin with it, which is how
  this project found it. Also for reporting the periodic microstutter fixed in this release, and —
  more usefully than the report itself — for pinning it to *this* profile rather than his own. That
  one observation is what turned an open-ended performance hunt into a search of our own plugin,
  where the cause turned out to be two full object-array sweeps burning ~5% of game-thread time.

### Referenced mod credit

Techniques taken directly from these projects, and where they ended up:

- **[OblivionVR](https://github.com/Pande4360/OblivionVR)** (Pande4360) — the widget-component
  construction sequence from `Profile/scripts/VRHud.lua`: deferred creation, with the material and
  blend mode set *before* registration. That sequence is what makes hosting the game's own crosshair
  possible, and it was solved here only after finding it solved there. Credited again at the point of
  use in `src/Reticule.cpp`.
- **[uevrlib](https://github.com/jbusfield/uevrlib)** (jbusfield) — its reticule module is the model
  for how the world-space reticule is built, including the bounds-scale fix that stops small widget
  components being frustum-culled; its widget-lifecycle handling informed ours. uevrlib is itself a
  community effort, and its own credit should carry across to here — it names **Pande4360, qwizdek,
  markmon, DJ, Mutar, CJ117, Ashok, Rusty Gere, lobotomy, letmein and Lukasblaster** as having
  provided code, ideas and inspiration.

#### Studied while building this

No code from these is used, but each was read closely enough to change how this mod is written, and
that's worth saying out loud:

- **[Dead Island 2 VR](https://github.com/Ashmwell/DeadIsland2VR)** — a model for composing a VR
  profile out of library modules rather than one monolithic script.
- **[SH2R-UEVR](https://github.com/praydog/SH2R-UEVR)** — the reference for getting aim right on a
  title that fights you, and the clearest example of the "fix the aim, not the mesh" doctrine.
- **[stalker2-uevr](https://github.com/praydog/stalker2-uevr)** — how to test VR mod logic without
  launching the game.
- **[AtomicHeart-UEVR](https://github.com/praydog/AtomicHeart-UEVR)** — hand-rolled VR feature work
  and comfort options, useful precisely because it does not lean on a framework.

## License & Disclaimer

Licensed under the **MIT License** — see [LICENSE.md](LICENSE.md). Fork it, build on it, ship it;
just keep the copyright notice. Note that the UEVR plugin SDK this builds against is praydog's and
carries its own terms, which is why it is fetched at build time rather than vendored here.

This is an unofficial fan project, not affiliated with or endorsed by Microsoft, Xbox Game Studios,
or Halo Studios. It distributes **no game assets** — only configuration, original code, original art,
and elliotttate's community cutscene plugin (`plugins\CutsceneDetectionPlugin.dll`, credited above,
which is his work rather than ours and so isn't covered by this project's MIT licence). Halo is a
trademark of Microsoft Corporation.

### AI Usage

Claude Code was used extensively in the development of this.

I share sentiments when it comes to how AI has ruined many spaces and has so many more ethical
problems. Not going to make excuses, it's become a daily use tool in my professional work. It's a
crazy powerful development tool and I wouldn't have been able to do this nearly as quickly without
it.

Totally understandable if AI usage here is too "fly in the soup" for you, and honestly, respect to
that.

For Software Engineering, I don't consider it to be the same kind of issue as with art, as we are
constantly building off of "stolen" and open-source work anyways. Engineering is very
results-oriented, while for the arts the personal journey part of it is more essential for the
individual and shape of the end work.
