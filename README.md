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
> on [UEVR nightly-01138 or newer](#uevr-version) — best on [our UEVR build](#uevr-version), attached
> to each release. Works in multiplayer!

## Features

- **True 1:1 motion aim** — the mod writes Halo's own aim state directly, upstream of the game's
  aim acceleration, deadzone and aim-assist, so projectiles, target logic, and vehicles keep
  working exactly as the game intends. Shots converge to your sightline, so what you're pointing
  at is what you hit — even leaning or stepping around the room.
- **On-target reticle** — the game's own crosshair (hit marker included) placed on the actual
  surface you're aiming at, traced the same way bullets are. It's drawn by your headset's compositor
  rather than into the game's scene, so the game's exposure and bloom can't wash it out (see
  [Crisp reticule and waypoints](#crisp-reticule-and-waypoints)). An optional colored ring is
  available too; both are configurable.
- **Waypoints in the world** — objective markers sit out in the world in the direction of the
  objective instead of sliding around with your aim, and they're compositor-drawn too, so a wall
  can't hide them.
- **Two-handed aiming** — bring your off hand to the weapon's front handle and squeeze its grip: the
  gun is held at both points and steers like a rifle, including weapons whose handle sits off to the
  side of the barrel. A "Grip" prompt shows when your hand is in reach.
- **Aim follows the barrel** — shots go where the weapon's barrel actually points, measured from the
  weapon itself, so there is no aim calibration to do. Most weapons ship with their barrel
  pre-measured.
- **Weapon scope** — while you're gripping two-handed (pistols too), the left trigger raises a
  magnified lens on the gun, aimed down the ray your shots actually follow. Without the grip, the
  same trigger throws a grenade. Halo's flat zoom (which hides the weapon and masks your view) stays
  suppressed.
- **Player IK** — your first-person arms follow your real hands: the game's own arm skeleton is
  solved to your controllers, with elbows, forearm twist, weapon recoil, the off hand on the
  weapon's grip when you hold it two-handed, and the off hand joining reloads, melees and grenade
  throws.
- **Hand gestures** — a free hand shapes its fingers from your controller: fist, point, thumbs up and
  OK, from the grip, the trigger and the thumb sensors. With [our UEVR build](#uevr-version) a finger
  resting on the trigger or a thumb resting on the stick counts too. The poses are yours to tune in
  `halo_vr_handposes.json`.
- **Physical melee and weapon switching** — squeeze the grip and swing either hand to melee, and the
  strike follows the swing rather than wherever the gun ended up pointing. Reach over your right
  shoulder and squeeze the grip to switch weapons; over your left shoulder to put the weapon away and
  free your gun hand for gestures (squeeze there again to bring it back).
- **Head-relative movement** — push the stick where you look, walk where you look, independent of
  where the gun points. Snap turn supported.
- **VR control layout** — crouch on right-stick-down, equipment on left-X, and the d-pad on the
  right stick whenever a hand is near your head (or on the left stick while you hold
  right-stick-up), all remappable. Menu-aware: in menus your right controller's B acts as *back* and
  the gameplay remaps stand down.
- **Cutscenes and sound that work in VR** — pre-rendered cutscenes play on a single flat screen in
  front of you (size adjustable), and positional audio follows your head rather than your gun.
- **Pose-match calibration** — optional per-hardware tuning: line your controller up with the visual
  weapon to calibrate grip, record where a weapon's front handle is, and place the scope lens — all
  armed from the in-game menu and persisting across sessions, with per-weapon adjustments on top
  (see [Custom calibration](#custom-calibration)). The shipped defaults work without it.
- **In-game settings, and settings that survive updates** — a settings menu in the UEVR overlay
  (Script UI) edits your personal `halo_vr_user.cfg` live, no restart; updates never touch that
  file. Calibration can be run from the menu too — no keyboard needed.

## Requirements

- **Halo: Campaign Evolved** (Steam).
- **UEVR** — [our UEVR build](#uevr-version) (recommended, attached to each release), or stock
  [nightly-01138 or newer](#uevr-version). Older builds render this game black.
- A VR headset set to the **OpenXR** runtime — the shipped config selects this for you. OpenVR
  mis-assigns controller bindings; see [Runtime](#runtime-use-openxr) if yours ends up on it.
- A VR controller pair. Developed against Quest touch controllers over Steam Link, and used on
  several other setups since.

> **If motion controls do nothing, press a trigger or a face button once, in game.** UEVR withholds
> real controller poses until it has seen controller *input* — waving a controller around is not
> enough to start it. One click is often the whole fix, and it costs nothing to try before anything
> else on this page.

## UEVR version

**Recommended: our UEVR build, `UEVR-HaloVR-01139.zip`, attached to each release.** It is stock UEVR
nightly-01139 plus a handful of changes made for this mod (all source diffs ship inside the zip):

- **Capacitive touch on the trigger and the thumbstick.** Stock UEVR does not expose them, so a finger
  resting on the trigger or a thumb resting on the stick is invisible to a mod. With them, your
  in-game index finger follows your real one without pulling the trigger, and your thumb goes down
  when it rests on the stick. Quest-style controllers; others simply use the trigger pull.
- **Mono rendering** (optional) — one view shown to both eyes, for roughly one eye's cost.
- **Fixes** for injection reliability on this game, a plugin deadlock during hitches, and the crash
  at game exit.

Unzip it into its own folder (not inside your normal UEVR) and inject with its `UEVRInjector.exe`.
It is an unofficial build — please do not report problems with it to the UEVR project.

**Stock UEVR also works.** The minimum is below.

**Minimum: [nightly-01138](https://github.com/praydog/UEVR/releases/tag/nightly-01138). Newer is
fine.**

**Anything older will not work**, and this is the one hard rule here. Halo: Campaign Evolved runs a
nonstandard UE 5.5.4 that needs the double-precision view-matrix handling introduced in 01138 —
before that, the scene renders black with a working HUD. That is a floor, not a ceiling.

**Newer nightlies and forks work in practice.** 01138 is what the mod is *developed against*, so it
is the safest answer if something is behaving oddly and you want to remove a variable. But later
nightlies (01139 among them) and PureDark's **AFW** fork are in regular use with this mod, including
by the people who build it. Being on one of those is not a red flag and is not the first thing to
suspect when something breaks.

If you do hit a problem, the version is worth *mentioning* — the mod logs it at startup, so it is
already in your `log.txt` — but please do not assume it is the cause and reinstall on that basis.

## Install

1. From this repo's **Releases** page download both `HaloCampaignEvolved.zip` (the mod) and
   `UEVR-HaloVR-01139.zip` (our [UEVR build](#uevr-version), recommended).
2. Extract `UEVR-HaloVR-01139.zip` into a folder of its own and run its `UEVRInjector.exe` once so
   it creates its folders. (Stock [UEVR nightly-01138 or newer](#uevr-version) works too.)
3. In the UEVR frontend, click **Import Config** and select the downloaded zip.
   - **Manual install (no Import Config):** instead of clicking Import Config, extract the zip into
     `%APPDATA%\UnrealVRMod\HaloCampaignEvolved\` so that `config.txt` sits directly in that folder.
4. Launch Halo: Campaign Evolved flat, and wait at the **main menu**.
5. In UEVR, select `HaloCampaignEvolved` and click **Inject**.

> **Inject at the main menu only.** Injecting mid-mission is more likely to result in rendering
> issues or hangs.

## Uninstall

Delete the `%APPDATA%\UnrealVRMod\HaloCampaignEvolved\` folder.

If — and only if — you registered the OpenXR layer by hand under the v0.4.0–v0.4.2 instructions
(see [Crisp reticule and waypoints](#crisp-reticule-and-waypoints)), run
`apilayer\Unregister-XrApiLayer.ps1` from inside that folder **before** deleting it. That
registration is the one thing this mod ever put outside its own folder, and the script is the way to
take it back out. If you have already deleted the folder, re-extract the zip anywhere and run the
script from there with `-All`.

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

## Crisp reticule and waypoints

**On by default, with nothing to set up.** The reticule and the mission waypoints are drawn by your
headset's own compositor instead of being painted into the game's scene. Halo's exposure and
tonemapping apply to everything *in* the scene — a crosshair drawn there dims on a bright beach and
blows out in shade — but the compositor receives these *after* all of that image processing, so they
stay readable everywhere, and a waypoint is never hidden behind a wall.

Reaching that stage of the pipeline takes an **OpenXR API layer**, which ships in the profile's
`apilayer\` folder. The mod switches it on for the game's own process as it loads, so there is
nothing to register and nothing is written outside the mod's folder.

- The layer checks which program it is in the moment it starts, and in anything that is not Halo it
  does nothing at all — no interception, no work, no files written.
- To go back to drawing them in the world, set `xrlayer=0` (reticule) and `xrlayernav=0`
  (waypoints) in `halo_vr_user.cfg`. Setting the environment variable `HALOVR_LAYER_DISABLE=1`
  stops the layer loading at all.
- **Run the game normally, not as administrator.** The OpenXR loader ignores layers from per-user
  locations in elevated programs, so that ordinary software cannot inject code into elevated
  software.

**How to tell whether it's working:** a `halo_vr_layer.log` file appears in the `apilayer` folder the
first time the layer loads into the game.

> **Registered the layer by hand under the v0.4.0–v0.4.2 instructions?** That registration is no
> longer needed. Run `apilayer\Unregister-XrApiLayer.ps1` once (right-click → **Run with
> PowerShell**) to remove it. It removes only what it added and leaves other software's OpenXR layers
> alone.

## Controls (Quest-style controllers)

| Input | Action |
|---|---|
| Right controller aim | Weapon aim (the game's aim follows it) |
| Left stick | Move, relative to where you look |
| Left stick **click** | Sprint |
| Right stick left/right | Snap turn |
| Hand **near your head** | Right stick becomes the d-pad (turning and crouch pause until you lower the hand):<br>**D-pad up** — Flashlight<br>**D-pad right** — Switch grenade<br>**D-pad left** — Equipment<br>**D-pad down (hold)** — Drop weapon |
| Right stick **up (hold)** | The same d-pad, on the left stick instead |
| Right stick **down** | Crouch |
| Right stick **click** | Melee — or squeeze a grip and swing that hand (either hand; `meleegrip=0` to swing without the grip) |
| Right trigger | Fire |
| Left grip | Grip the weapon two-handed — reach for the barrel; a "Grip" prompt shows when you're in range |
| Left trigger | Throw grenade — or, while gripping two-handed, toggle the scope (`scope`/`scopezoom` to tune) |
| Right grip | Reach over your right shoulder and squeeze to switch weapons; over your left shoulder to put the weapon away and free your hand (squeeze there again to bring it back) |
| Right A | Jump |
| Left Y | Switch weapon — with a controller near your head, **pause** instead |
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

The switch is automatic, and it also applies during cutscenes — the mod works it out from the game's
camera leaving first person, so standing on foot with no weapon keeps motion controls. If you ever
find a seat it misses, please report it; as a
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
| `xrlayer` / `xrlayernav` | 1 / 1 | Draw the reticule / waypoints with the headset's compositor (0 = draw them in the world) |
| `cutscenesize` | 0.75 | Size of the cutscene screen (1.0 = edge to edge) |

[`halo_vr_user_reference.txt`](profile/halo_vr_user_reference.txt) is the full player-facing
catalog — organised by concern, safe to explore, and refreshed by every update, so newly added
settings always appear there. Your `halo_vr_user.cfg` stays short: just the keys you chose to
change. Because it is generated rather than shipped, an update can never reset it — and
**deleting it resets every setting to the built-in defaults** (a fresh template regenerates on
the next launch).

**Prefer menus?** Open the UEVR overlay (Insert on the keyboard, or press both thumbsticks) and
scroll to **Script UI** — four panels live there:

- **Halo VR User Settings** — every player setting, grouped exactly as in the catalog with the
  catalog's own comments as tooltips; overridden settings get an `x` button back to the default.
  Saves to `halo_vr_user.cfg` (so it survives updates like any hand edit) and applies live
  within a couple of seconds.
- **Halo VR Controls (rebinding)** — put one of the mod's own actions (crouch, melee, reload, the
  scope, equipment, the d-pad shift) on a button of your choice: press Rebind, close the menu, then
  press the button, and it records what your controller actually sends.
- **Halo VR Calibration** — the calibration gestures as buttons, no keyboard needed: arm one,
  close the menu (controllers don't reach the game while it's open), line your controller up, then
  **right trigger saves & finishes** — or **left trigger saves & re-arms** on release, for
  consecutive passes. Triggers won't fire your weapon while a calibration is armed. The scope's
  placement is armed from here too — for every weapon, or as a trim for just the one in your hands.
  One-press resets take you back to the shipped fit, per gesture or wholesale.
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
  Troubleshooting may ask you to uncomment a key in it; the everyday diagnostics, like `perflog=1`
  for a stutter report, are in the player catalog instead. Updates overwrite it, so experiments
  never linger.

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

If the visual weapon doesn't sit right in your hand, these calibrations let you match the mod to your
own hardware and grip. They run in-mission, **persist across sessions** once set, and are armed from
the **Halo VR Calibration** panel (see [Configuration](#configuration)): arm one, close the menu, line
up, then **right trigger saves & finishes** or **left trigger saves & re-arms**. Nothing calibrates
from the keyboard alone, so a stray key press can't overwrite your fit.

| Panel button | Calibration | Workflow |
|---|---|---|
| **Calibrate weapon pose** | **Pose-match** (grip) | Line your controller up with the frozen on-screen weapon, then save. This aligns the weapon's grip to how you actually hold your controller. `End` works in place of the trigger while this is armed. |
| **Calibrate weapon pose — THIS WEAPON only** | **Per-weapon pose** | The same gesture, stored as an adjustment for the weapon in your hands. Every other weapon keeps the global fit. |
| **Calibrate weapon grip (off-centre handles)** | **Front handle** | Click it and the weapon freezes. Put your support hand where that weapon's front handle really is, then click **SAVE grip**. Only the two-handed hold changes. |
| **Arm per-weapon scope trim** / **Arm BASE scope calibration** | **Scope placement** | The scope pane stays visible while either is armed. Hold `Delete`, move the lens to where you want it on the gun, then release — stored for the weapon in your hands, or for every weapon. |

**Aim needs no calibration.** Shots follow the weapon's own barrel. If a particular weapon still
shoots off its barrel for you, hold it steady where it should point and tap `Page Down` to capture
that weapon's barrel line; it's remembered from then on.

Do the weapon pose first — it sets where the weapon sits. If a calibration ever feels off, just repeat
it — the latest one wins.

**Reset your play area first, and calibrate standing where you normally play.** This matters only if
you've turned the head leash off (`hmdleash=0`); with the default leash it's automatic. Unleashed,
your eye can be metres from where the game thinks you are, and the mod bends your aim to compensate —
so calibrating from over there measures your grip through that correction instead of measuring your
grip. Recentre, calibrate from your neutral position, and the result is exact and stays correct
wherever you wander afterwards. If you skip this, the fit will be a little noisier; nothing is
broken, it's just not as good as it could be.

**One global fit, adjusted per weapon.** The global calibration is what every weapon starts from,
which is why the original calibration was done on the magnum: a middle-of-the-road result beats one
that's perfect on a pistol and wrong on a rocket launcher. Where a particular weapon still sits
wrong, hold that weapon and store an adjustment for it alone with the per-weapon calibration.

### Where your calibration is stored

Calibrations write to two files next to the config:

```
%APPDATA%\UnrealVRMod\HaloCampaignEvolved\halo_vr_calib.cfg      (the global fit, captured barrel lines)
%APPDATA%\UnrealVRMod\HaloCampaignEvolved\halo_vr_weapons.cfg    (per-weapon adjustments, front handles)
```

They override the shipped calibration in `halo_vr.cfg`. Keeping them separate is deliberate: updates
refresh the shipped calibration freely while your measured fit is never touched.

**To go back to the shipped calibration, delete `halo_vr_calib.cfg` — and `halo_vr_weapons.cfg` to
clear every per-weapon adjustment.** Neither is part of the download — each only exists once you've
calibrated — so there's no original copy to restore, and deleting one simply lets the shipped
defaults apply again. The mod recreates them next time you calibrate.

Once you have a calibration you like, it's worth copying both files somewhere safe, along with
`halo_vr_user.cfg` if you've changed settings. They're small, plain text, and they're the only things
in the profile that are specific to *you* — everything else can be re-downloaded.

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
- **Your hands are invisible until you pick up your first weapon.** Motion aim and turning work
  normally while you're unarmed — but you won't see arms. The game T-poses the empty first-person
  arms, because on a flat screen holding nothing means there's simply no viewmodel to draw; in VR
  they'd be right in front of you, following your hand, T-pose and all. They stay hidden until
  there are proper VR hands to show instead.
- **The scope view can be too bright**, and its lighting often doesn't match the scene around it.
  Work in progress.
- **The scope pane can jump out of place after shooting or reloading.** To fix it for now: switch
  weapons, then toggle the scope off and on again. Also work in progress.
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

The OpenXR API layer that draws the crisp reticule and waypoints is a separate DLL under
`apilayer/`, built by `scripts\build-apilayer.ps1`. It needs nothing but the Windows SDK and the
OpenXR headers vendored in the repo — no UEVR checkout — and `scripts\package.ps1` builds it for you.

Full instructions, the source layout, and the conventions for splitting a new module out of
`Plugin.cpp` are in [COMPILING.md](COMPILING.md). **Contributions welcome** — the code is being
split by feature to make that easier.

## Credits

**Special thanks to [elliotttate](https://github.com/elliotttate)** — for hosting the Flat2VR
community, without which none of this work would have found the people who made it possible, and for
writing **CutsceneDetectionPlugin**, which this profile bundled up to v0.4.1 to handle cutscene
comfort (disabling decoupled pitch and camera offsets around cutscene camera cuts). That plugin is
his work, not ours — and this mod's first cutscene handling (the flattened cutscene view,
`cutscene2d`) followed the approach his plugin pioneered. Since v0.4.2 the mod presents cutscenes
itself, so the plugin is no longer bundled.

He also started this mod's **Player IK**: the arms are posed through Halo's own first-person arm
skeleton by a route ported from his project, and everything the arms do here was built on top of
that start.

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

**Special thanks to [blindcowboy24](https://github.com/blindcowboy24)** — My main direct contributor to the project! He's responsible for many neat feature additions and fixes (per-weapon calibration, directional melee including the left-hand directional melee, over the shoulder holster, and more on the way). 
Unparalleled patience and politeness in his manner of contribution, with great communication and flexibility in direction. Honestly wasn't expecting this kind of big help. Can't thank you enough, bro! 

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
or Halo Studios. It distributes **no game assets** — only configuration, original code and original
art. (Releases up to v0.4.1 also bundled elliotttate's community cutscene plugin,
`plugins\CutsceneDetectionPlugin.dll`, credited above, which is his work rather than ours and so
isn't covered by this project's MIT licence.) Halo is a trademark of Microsoft Corporation.

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
