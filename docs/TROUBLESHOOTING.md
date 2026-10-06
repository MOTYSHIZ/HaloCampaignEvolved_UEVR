# Troubleshooting

## Install

**Nothing the mod does is happening — no motion controls, and changing settings has no effect.**

Before anything else, confirm the plugin actually loaded. Near the top of `log.txt` you should see
UEVR load it, followed by the mod's own banner and the UEVR build it is running on:

```
[PluginLoader] Loaded ...\UnrealVRMod\HaloCampaignEvolved\plugins\halo_vr.dll
[Halo-CampE-UEVR] Halo: Campaign Evolved VR  v0.7.0  (halo_vr.dll)
[Halo-CampE-UEVR] UEVR backend: tag=... branch=... commit=...
```

If `Loading plugins...` is followed by **no `Loaded` lines at all**, the plugin is not installed, and
nothing else in this document applies — a profile can look completely correct in the UEVR frontend
while `plugins\` is empty. Re-run **Import Config** on the release zip, or copy `halo_vr.dll` into
`%APPDATA%\UnrealVRMod\HaloCampaignEvolved\plugins\` yourself.

Searching `log.txt` for `Halo-CampE-UEVR` is the quickest form of this check: no matches means the
mod never ran, whatever else the log says.

Do **not** read anything into `[PluginLoader] Created directory for plugins` — UEVR logs that line on
every launch, including ones where the plugin loads perfectly. It is not evidence of a missing folder.

**I'm installing a new release over an older one.**

Don't — delete `%APPDATA%\UnrealVRMod\HaloCampaignEvolved\` first, then install the new release as if
it were your first time. Installing over the top only adds and overwrites files, so anything an
earlier version left behind stays and keeps applying, and that mixed folder is not what any release
was tested as. If the mod behaves oddly right after an update, this is the first thing to rule out.

You can copy `halo_vr_user.cfg` (your settings) and your calibration files — `halo_vr_calib.cfg`,
`halo_vr_weapons.cfg`, `halo_vr_handposes.json`, and `halo_vr_vehcams.json` if you edited your vehicle
cameras — out first and put them back afterwards, but that is
a convenience and not a guarantee: settings change between releases, and an old value can behave
differently on a new build. If anything feels wrong after restoring them, delete them and use the
shipped defaults.

**I have the Game Pass / Microsoft Store version.**

It is supported, and we test on it as well as on Steam. No renaming is needed. UEVR names the profile
folder after the executable with its extension stripped, and this game's executable is
`HaloCampaignEvolved.exe` in both the `Win64` (Steam) and `WinGDK` (Game Pass) builds — so the
profile is `%APPDATA%\UnrealVRMod\HaloCampaignEvolved\` either way and the release zip imports to the
right place unmodified.

The advice you may have seen to swap `Win64` for `WinGDK` is real, but it belongs to games whose Game
Pass executable has a different *name* (`Foo-Win64-Shipping.exe` versus `Foo-WinGDK-Shipping.exe`).
This game does not. If you want to check for yourself rather than take our word for it: inject once,
then look at which folder appears under `%APPDATA%\UnrealVRMod\` and make your profile match it.

The two builds are genuinely different binaries, so if something misbehaves on only one of them,
please say which store you play on when you report it.

## Injection

**The game hangs when I inject.**
Inject at the **main menu**, never mid-mission. If it hung, kill the game and start over.

**Injection failed even though I did it at the main menu.**
It happens, and it is intermittent rather than a setup problem. Two shapes:

- the game **hangs** during injection and never reaches VR, or
- it comes up **rendering only one eye**.

Same remedy for both: force-kill the game, relaunch, and inject again at the main menu. A retry
normally works, and nothing needs reconfiguring in between.

This is a **UEVR-side problem, not a mod bug**, on the evidence we have. The hang has been seen with
the plugin removed from the profile entirely, so it happens with no mod code loaded at all; and
stereo rendering belongs to UEVR's hook, which this plugin never touches — it only starts once
stereo is already running. Its signature in `log.txt` is `Hooked DirectX 12` followed by
`Received frontend command: 2` and then silence, with no further plugin lines.

If you can reproduce either reliably — especially a sequence that triggers it every time — that is
worth reporting upstream to UEVR rather than here.

**UEVR says the runtime failed / SteamVR's compositor never starts.**
Check the runtime selection in the UEVR frontend before assuming SteamVR is broken — a mismatch
between the requested runtime and what is actually running looks exactly like a corrupt install.
On a SteamVR-only setup, an OpenXR selection has been seen to leave `vrcompositor` never starting.
That is the one case where trying `Frontend_RequestedRuntime=openvr_api.dll` in the profile's
`config.txt` is worth it — but expect the control layout to come through wrong on OpenVR, so treat
it as a diagnostic rather than a fix, and report it. **OpenXR remains the supported runtime**; see
the README's runtime section.

**My antivirus flags the DLL.**
Injection tooling is exactly what AV heuristics look for. Verify your download against the SHA-256
published in the release notes; if it matches, add an exclusion or build from source yourself.

## In VR

**Motion controls never worked at all — the gun ignores my hand from the moment I load in.**

**Try this first: press a trigger or a face button once, in game.** UEVR withholds real controller
poses until it has observed controller *input*; waving a controller is not enough to start it. This
is the single most common cause and it costs nothing to rule out.

The mod logs UEVR's controller state whenever it changes:

```
[Halo-CampE-UEVR] VR STATE CHANGE: hmd_active=1 using_controllers=0  <-- controllers dropped: ...
```

`using_controllers=0` means UEVR is not handing the mod controller poses right now. Before your first
button press that is expected — it is exactly what the press above fixes. If it drops to 0 in the
middle of play, you are looking at the frozen-hands problem below, and a press brings it back.

If controllers read 1 and the gun still ignores you, `log.txt` can tell you whether poses are
arriving at all. About every 20 seconds it logs a line like:

```
[Halo-CampE-UEVR]   rig: travel=0.000m rigOff=(0.0,0.0,0.0)cm ...
```

`travel` is the furthest your hand got from its resting point during that window. **Wave your hands
about, then check the most recent few:** varying, non-zero numbers mean poses are live and the
problem is elsewhere (calibration, or the mod standing down — see the vehicle and stick-mode notes
in the README). `0.000` throughout, or the *same* number repeated exactly, means the poses are frozen
and nothing the mod does downstream can help. It only updates while you are on foot with a weapon,
so it reads 0 in a vehicle or a cutscene whatever your hands are doing.

**My hands suddenly freeze (3DoF) but buttons still work — pressing something fixes it.**
Three known causes:
1. The controllers themselves going to sleep after you hold still or set them down — a long
   cutscene is the usual trigger. Move or press something to wake them.
2. UEVR's motion-controls inactivity timeout. The mod raises it to its maximum (100 s) at startup,
   but the ceiling is UEVR's, not ours.
3. On the **OpenXR** runtime, losing session focus (opening the SteamVR dashboard, an overlay app,
   remoting into the PC) stops controller poses updating at the runtime level. Give the game focus
   and press something to recover. OpenVR is not exposed to this, since it sources poses from the
   compositor — but it mis-assigns the control layout, which is the worse problem, so OpenXR is
   the supported runtime and this is a known trade-off rather than a reason to switch.

**My view is at the wrong height, or Chief crouches when I sit down.**
Auto height (on by default) puts your view at your real head height above your real floor, and
physical crouch crouches Chief whenever your head drops below 80% of your standing height. Both
assume you play standing, so sitting down reads as a crouch.

- **Playing seated:** in **Halo VR User Settings**, set **Height fit** to `seated`
  (`heightmode=seated`), or untick **Crouch when you duck** (`heightcrouch=0`).
- **The height feels off:** press **Calibrate height** (beside the Auto height switch), close the
  menu, and stand up straight for a moment.
- **Switching between standing and seated mid-session:** change **Height fit** to match, then
  calibrate again. Your standing height is only ever raised, never lowered, so a standing
  measurement still applies after you sit.
- **Chief stays crouched after a very quick duck:** the game's controller crouch is a toggle by
  default. Duck again, or turn on the game's **Hold to crouch** controller setting.
- **Right after a system recenter,** physical crouch can press or release for up to a second while
  the mod re-measures your floor. That settles on its own.
- **Auto height never seems to measure anything on OpenXR:** it needs the headset-drawn reticule
  (`xrlayer=1`, the default) to find your real floor. With it off, auto height follows your eye
  level instead and physical crouch stays off.

If the weapon or aim still feels offset after changing stance, recentre through your runtime's
play-area reset (SteamVR: long-press the system button, or Settings → Play Area).

**Aim feels laggy or overshoots on fast sweeps.**
With the shipping aim this should not happen — pointing is written directly into the game, 1:1,
with no control loop to lag or overshoot, and your in-game sensitivity setting has no effect on it.
If it does: check `log.txt` for `DEV OVERRIDES ACTIVE` (a leftover experiment in `halo_vr_dev.cfg`
may have switched the aim path), and try deleting `halo_vr_user.cfg` to rule out a stale setting.
Only the fallback stick-steer path (`aimdirect=0`) has feel tuning — `ffgain`, `dgain`, `dead` in
`halo_vr_dev.cfg` — and only that path is affected by in-game sensitivity.

**The ring reticule is missing at mission start.**
The ring is off by default (`aimmesh=0`); this only applies if you turned it on. Its material
streams in a few seconds after spawn and the mod rebinds automatically. If it never appears, check
`log.txt` for `reticule mesh asset` / `textured reticule` lines.

**The ring reticule disappeared after a game update.**
Most likely the game moved or removed an asset, not a bug in the mod. The ring is not shipped art —
it is built from two assets inside the game's own content, named by path in `halo_vr_dev.cfg`
(uncomment a key there to point it elsewhere):

```
aimmeshpath=StaticMesh /Game/FX/Meshes/Primitives/Torus/SM_Torus_ThinDense_01.SM_Torus_ThinDense_01
aimmeshparent=/Game/FX/Meshes/Debug/Materials/MI_Arrow.MI_Arrow
```

If a patch renames or strips either, the ring stops appearing. `log.txt` will show the load failing
rather than nothing happening. `MI_Arrow` is the likelier casualty — it lives under a `Debug/` path,
which is the sort of content a shipping patch removes.

Workarounds, in order of preference: point `aimmeshparent` at another material the game still has;
or set `aimmesh=0` (its default) in `halo_vr_user.cfg` and rely on the hosted crosshair
(`aimwidget=1`), which depends on no fixed asset path. Please report the game version if you hit
this — the built-in default should be changed to whatever survives.

**The reticle/ring is the wrong colour or too big.**
`aimmeshcr/cg/cb` and `aimmeshscale` in `halo_vr_user.cfg`, live.

**The scope won't open.**
The scope needs both hands on the weapon: put your left hand where you'd really hold the front of
the gun — the foregrip on a rifle, the front handle on weapons like the rocket launcher and sentinel
beam, just under your firing hand on a pistol — and squeeze the left grip until the "Grip" prompt
takes hold, then hold the left trigger. Without the grip, the left trigger throws a grenade. If a
weapon never takes hold where you naturally hold it, record its handle with **Calibrate weapon
grip** in the Halo VR Calibration panel.
If it never opens, check that `scope=1` and `twohand=1` (both the defaults) — an old
`halo_vr_user.cfg` carried over from a previous release is the usual reason for either to be off.

**Menus respond to the wrong buttons.**
Menu handling needs `menudetect=1` (default). In menus, right-controller B is Back and the
gameplay remaps are suspended; if a specific menu misbehaves, report which one.

## Crisp reticule and waypoints (the OpenXR layer)

The reticule and waypoints are drawn by your headset's compositor through a small OpenXR API layer
that ships in the profile's `apilayer\` folder. The mod switches it on for the game's own process as
it loads — there is nothing to register — and when the layer is not available the mod falls back to
drawing the reticule in the world.

**The reticule looks dim or washed out, or waypoints hide behind walls — the layer isn't working.**
Check for `halo_vr_layer.log` in `%APPDATA%\UnrealVRMod\HaloCampaignEvolved\apilayer\`. The layer
writes it the first time it loads into the game, so its absence and its presence point in different
directions:

- **No file at all** — the layer never loaded. `log.txt` has an `XRLAYER:` line from the mod saying
  what it did at startup:
  - `HALOVR_LAYER_DISABLE is set` — that environment variable switches the layer off; remove it.
  - `no apilayer\ in the profile` — the layer files are missing. Reinstall the release zip.
  - `API layer enabled for THIS PROCESS ONLY` — the mod did its part, so the OpenXR loader declined.
    Most likely the game is running on **OpenVR** rather than OpenXR (the layer only exists on the
    OpenXR side — see *Runtime: use OpenXR* in the README), or it is being run **as
    administrator**, where the loader ignores the layer path by design.
  - No `XRLAYER:` line at all — the plugin itself never loaded; see the top of this page.
- **The file exists** — the layer is loading and the problem is downstream. Send `log.txt` and this
  file together; the mod's log also carries an `[XRBRIDGE]` line saying whether the plugin and the
  layer agreed on a version.

To turn the compositor drawing off and go back to the in-world reticule and waypoints, set
`xrlayer=0` and `xrlayernav=0` in `halo_vr_user.cfg`.

**I registered the layer by hand under the v0.4.0–v0.4.2 instructions.**
That registration is no longer needed, and it still points at whichever copy of the layer you
registered — possibly an older one than the release you now run. Run
`apilayer\Unregister-XrApiLayer.ps1` once (right-click → **Run with PowerShell**) — it removes only
what it added and leaves other software's OpenXR layers alone. If you have since deleted or moved
the folder you registered from, run it with `-All` from any copy of the release zip.

**Another VR game started behaving oddly.**
The layer checks which program it is in the moment it starts and does nothing at all outside this
game, and the mod only switches it on inside the game's own process. If you registered it by hand on
v0.4.0–v0.4.2, run `Unregister-XrApiLayer.ps1` as above. You can also set the environment variable
`HALOVR_LAYER_DISABLE=1`, which switches it off everywhere. Please report it either way — the layer
is meant to be inert outside this game.

## Logs

Nearly everything the mod does is logged with a `[Halo-CampE-UEVR]` prefix (the layer bridge logs as
`[XRBRIDGE]`) in
`%APPDATA%\UnrealVRMod\HaloCampaignEvolved\log.txt`. Include the tail of that file in bug reports,
and say whether you play on Steam or Game Pass.

If the reticule or waypoints misbehave, `apilayer\halo_vr_layer.log` in the same profile folder is
worth attaching too.

## Uninstall

Delete `%APPDATA%\UnrealVRMod\HaloCampaignEvolved\`.

That is everything — **unless you registered the OpenXR layer by hand under the v0.4.0–v0.4.2
instructions**, which recorded one value under `HKEY_CURRENT_USER` pointing at the mod's folder. In
that case run `apilayer\Unregister-XrApiLayer.ps1` before deleting the folder and that value goes
too. If the folder is already gone, re-extract the zip anywhere and run the script from there with
`-All`.
