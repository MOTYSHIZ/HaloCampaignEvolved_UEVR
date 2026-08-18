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

- **True 1:1 motion aim** — the mod writes Halo's own aim state directly, upstream of the game's
  aim acceleration, deadzone and aim-assist, so projectiles, target logic, and vehicles keep
  working exactly as the game intends. Shots converge to your sightline, so what you're pointing
  at is what you hit — even leaning or stepping around the room.
- **On-target reticle** — the game's own crosshair (hit marker included) placed on the actual
  surface you're aiming at, traced the same way bullets are. An optional colored ring is available
  too; both are configurable.
- **Weapon scope** — left trigger raises a magnified lens on the gun, aimed down the ray your
  shots actually follow. Halo's flat zoom (which hides the weapon and masks your view) stays
  suppressed.
- **Head-relative movement** — push the stick where you look, walk where you look, independent of
  where the gun points. Snap turn supported.
- **VR control layout** — crouch on right-stick-down, equipment on left-X, d-pad access via
  right-stick-up shift, all remappable. Menu-aware: in menus your right controller's B acts as
  *back* and the gameplay remaps stand down.
- **Pose-match calibration** — optional per-hardware tuning: line your controller up with the visual
  weapon to calibrate grip, and align the aim ray, both persisting across sessions (see
  [Custom calibration](#custom-calibration)). The shipped defaults work without it.
- **In-game settings, and settings that survive updates** — a settings menu in the UEVR overlay
  (Script UI) edits your personal `halo_vr_user.cfg` live, no restart; updates never touch that
  file. Calibration can be run from the menu too — no keyboard needed.

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
| Left trigger | Weapon scope — magnified lens on the gun (`scope`/`scopezoom` to tune) |
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
taking your first-person weapon away. If you ever find a seat it misses, please report it; as a
stopgap you can add `stickforce=1` to `halo_vr_user.cfg` (the key is documented in
`halo_vr_dev.cfg`) to force these controls on, and remove it to go back to automatic.

## Configuration

Your settings file is `%APPDATA%\UnrealVRMod\HaloCampaignEvolved\halo_vr_user.cfg` (created on
first run). Every available setting — with comments and its default — is listed in
`halo_vr_user_reference.txt` next to it: copy the keys you want into your file, remove the
leading `#`, set your value, save. It applies live while you play (~2 s), and the file is yours:
**updates never touch it**. The important knobs:

| Key | Default | Meaning |
|---|---|---|
| `aimmesh` | 0 | Optional geometric ring at the aim point (off; the game's own crosshair now shows in colour) |
| `aimmeshcr/cg/cb` | blue | Ring colour, if `aimmesh=1` (RGB 0–1) |
| `aimmeshscale` | 0.14 | Ring size |
| `aimreticuledist` | 500 | Ring distance from you, in cm |
| `aimwidget` | 1 | Host the game's own crosshair (with hit marker) at the aim point |
| `aimwidgetgain` | 5 | Brightness of the hosted crosshair (multiplies with `aimwidgettint`). Unlit UI is scaled down by the scene's exposure, so it needs lifting back; lower it if the crosshair blooms |
| `hudhide` | 0 | Collapse the flat HUD crosshair once you trust the ring |
| `turnmode` / `snapdeg` | 1 / 45 | Snap turn on, 45° per step (`turnmode=2` for smooth turning) |
| `scope` | 1 | Left-trigger weapon scope (magnified lens on the gun) |
| `scopezoom` | 16 | Scope magnification |

[`halo_vr_user_reference.txt`](profile/halo_vr_user_reference.txt) is the full player-facing
catalog — organised by concern, safe to explore, and refreshed by every update, so newly added
settings always appear there. Your `halo_vr_user.cfg` stays short: just the keys you chose to
change. Because it is generated rather than shipped, an update can never reset it — and
**deleting it resets every setting to the built-in defaults** (a fresh template regenerates on
the next launch).

**Prefer menus?** Open the UEVR overlay (Insert on the keyboard, or press both thumbsticks) and
scroll to **Script UI** — three panels live there:

- **Halo VR User Settings** — every player setting, grouped exactly as in the catalog with the
  catalog's own comments as tooltips; overridden settings get an `x` button back to the default.
  Saves to `halo_vr_user.cfg` (so it survives updates like any hand edit) and applies live
  within a couple of seconds.
- **Halo VR Calibration** — the two calibration gestures as buttons, no keyboard needed: arm
  one, close the menu (controllers don't reach the game while it's open), line your controller
  up, then **right trigger saves & finishes** — or **left trigger saves & re-arms** on release,
  for consecutive passes. Triggers won't fire your weapon while a calibration is armed. One-press
  resets take you back to the shipped fit, per gesture or wholesale.
- **Halo VR DEV Settings** — the internal research knobs, behind a warning. Leave them alone
  unless troubleshooting asks.

This ships as `scripts/halo_vr_settings.lua` and needs nothing extra: UEVR's built-in Lua
scripting loads it automatically.

The defaults themselves are built into the plugin — there is no shipped settings file to hand-edit
or lose. Two other files do ship next to yours:

- `halo_vr.cfg` now holds **only the shipped weapon calibration**. Don't edit it — recalibrate
  instead (see [Custom calibration](#custom-calibration)); your results override it and survive
  updates.
- [`halo_vr_dev.cfg`](profile/halo_vr_dev.cfg) is the catalog of internal tuning, research and
  diagnostic knobs — including the aim-drive tunables — every line commented out. **Leave it alone
  unless you know exactly what you are doing**: wrong values there can wreck performance or aim.
  Its one everyday use is troubleshooting, where you may be asked to uncomment a key (for example
  `perflog=1` for a stutter report). Updates overwrite it, so experiments never linger.

## Left-handed aim

Not properly supported yet. An `aimhand` setting exists in the mod's internals, but the newer
systems don't honour it end to end, so it's hidden from the settings until left-handed play
actually works — it's on the list.

<details><summary>What it was intended to do</summary>

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

</details>

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

**Reset your play area first, and calibrate standing where you normally play.** This matters only if
you've turned the head leash off (`hmdleash=0`); with the default leash it's automatic. Unleashed,
your eye can be metres from where the game thinks you are, and the mod bends your aim to compensate —
so calibrating from over there measures your grip through that correction instead of measuring your
grip. Recentre, calibrate from your neutral position, and the result is exact and stays correct
wherever you wander afterwards. If you skip this, aim will settle for a moment after you release
`Page Down` and the fit will be a little noisier; nothing is broken, it's just not as good as it
could be.

**One calibration covers every weapon.** Per-weapon calibration isn't supported yet — it's planned.
Until then, a grip tuned on one weapon is the grip used for all of them, which is why the original
calibration was done on the magnum: a middle-of-the-road result beats one that's perfect on a pistol
and wrong on a rocket launcher.

### Where your calibration is stored

Both calibrations write to a separate file next to the config:

```
%APPDATA%\UnrealVRMod\HaloCampaignEvolved\halo_vr_calib.cfg
```

It's applied after every other config file, so it overrides the shipped calibration in
`halo_vr.cfg`. Keeping it separate is deliberate: updates refresh the shipped calibration freely
while your measured fit is never touched.

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
- **Buttons mapped wrong?** You're on the OpenVR runtime. See [Runtime](#runtime-use-openxr) for the
  one-line fix.
- **No pause binding when playing over Steam Link.** Press **`Esc`** on your keyboard to pause. A
  controller binding is coming.
- **Cutscenes display doubled.** They're pre-rendered movies composited outside the game's 3D
  render, so they don't resolve in stereo; closing one eye makes them watchable. A proper in-VR
  cinema screen for them is being worked on — an experimental flat-screen mode exists in the
  internals (`cutscene2d` in `halo_vr_dev.cfg`) but is hidden from the settings until it's been
  verified in a headset.
- **Your hands are invisible until you pick up your first weapon.** Motion aim and turning work
  normally while you're unarmed — but you won't see arms. The game T-poses the empty first-person
  arms, because on a flat screen holding nothing means there's simply no viewmodel to draw; in VR
  they'd be right in front of you, following your hand, T-pose and all. They stay hidden until
  there are proper VR hands to show instead.
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
- **Injection sometimes fails even at the main menu.** It either hangs the game during injection, or
  comes up rendering **only one eye** once a mission is entered. Force-kill the game, relaunch, and inject again — it's
  intermittent, and a retry normally works. As far as we can tell this is a UEVR issue rather than a
  mod bug: the hang has been observed with the plugin removed from the profile entirely, and stereo
  rendering is UEVR's own hook, which this mod never touches.
- Mid-mission injection is unreliable — expect rendering glitches or a hang. Always inject at the
  main menu.

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
cutscene comfort (disabling decoupled pitch and camera offsets around cutscene camera cuts). That
plugin is his work, not ours — and this mod's own cutscene handling (the flattened cutscene view,
`cutscene2d`) follows the approach his plugin pioneered.

He is also the reason this mod's crosshair has colour at all. The world-space crosshair rendered
near-black for a long time, and the diagnosis that fixed it is his: an unlit widget's output is
still multiplied by the scene's **pre-exposure** before tonemapping, so an authored colour lands
close to black in a bright scene. Nothing about the code that writes that colour hints at it. Once
the mechanism was named the fix followed quickly, and the shipping crosshair here is a direct
result. His own remedy — Unreal's exposure-compensating `EyeAdaptationInverse` widget material,
delivered as an asset pak — is the more principled one; it could not be made to work in this
configuration, so this mod compensates with a plain gain instead (`aimwidgetgain`). That is a
limitation on our side, not a defect in his approach.

**Special thanks to [praydog](https://github.com/praydog/UEVR)** — for UEVR itself, which every one
of these projects stands on, for the nonstandard 5.5.4 fix that had this game rendering in VR within
days of its release, and for the plugin SDK this mod is written against. Hail to the king.

- **[Pande4360](https://github.com/Pande4360) and [deterministicj](https://github.com/deterministicj)** — For welcoming me into the Flat2VR community as a modder and providing useful learning/community resources!
- **[pancreations / Halo-MCC-VR](https://github.com/pancreations/Halo-MCC-VR)** — independent prior
  art for Halo VR aim doctrine ("steer the game's own aim") and the authored-reticle approach. The
  weapon scope follows his design too: suppress Halo's own zoom and mount a synthetic magnified lens
  on the gun, and our scope camera's roll lock adapts a construction from his MIT-licensed
  `scope_logic.cpp` ("preserve the rifle's roll while keeping up perpendicular to the actual bullet
  direction") — credited at the point of use in `src/Scope.cpp`.
- **[LunchAndVR](https://www.youtube.com/@LunchAndVR)** — For creating the community UEVR profile this
  configuration descends from, and for bundling elliotttate's cutscene plugin with it, which is how
  this project found it.
- **ShadowNK** — for the field report and logs on 0.2's movement direction. Those logs exposed that
  the mod's recorded memory addresses were measurements of one specific game build — silently wrong
  on any other — and drove the move to verified, self-reporting address resolution that can survive
  game updates. Exactly the kind of report that makes the mod better for everyone. Also for reporting the periodic microstutter fixed in 0.2, and —
  more usefully than the report itself — for pinning it to *this* profile rather than his own. That
  one observation is what turned an open-ended performance hunt into a search of our own plugin,
  where the cause turned out to be two full object-array sweeps burning ~5% of game-thread time.
- **[blindcowboy24](https://github.com/blindcowboy24)** — for
  [PR #6](https://github.com/MOTYSHIZ/HaloCampaignEvolved_UEVR/pull/6), the first outside code
  contribution to this mod: a second, unrelated set of periodic game-thread stalls, found by
  *measuring* rather than reasoning — `QueryPerformanceCounter` around each site across a 78-minute
  session, reported with before-and-after numbers. The largest was invisible to code review:
  `load_config` re-read the config files off disk every ~60 ticks, forever — normally ~0.45 ms and
  unnoticeable, but 143.9 ms under disk contention. The other two were the same mistake in two
  full-array sweeps, rebuilding a class-name string for every one of ~296,000 objects, cut from
  83.9 ms to 29.8 ms and from 78.5 ms to 21.4 ms. That technique is now used in two further sweeps
  he never touched.

### Referenced mod credit

Techniques taken directly from these projects, and where they ended up:

- **[HaloCampaignEvolved-UEVR](https://github.com/elliotttate/HaloCampaignEvolved-UEVR)**
  (elliotttate) — a **separate, independently developed** Halo: Campaign Evolved UEVR project,
  running in parallel with this one. Studied with his permission. Its identification of
  pre-exposure as the cause of the dark world-space crosshair is what made this mod's coloured
  crosshair possible; see the Credits section above. Two projects, two sets of trade-offs — worth
  looking at directly rather than assuming this one supersedes it.
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
