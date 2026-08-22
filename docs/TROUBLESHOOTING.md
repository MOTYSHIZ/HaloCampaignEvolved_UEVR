# Troubleshooting

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

If that doesn't do it, `log.txt` can tell you whether poses are arriving at all. Look for a line like:

```
rig: travel=0.000m rigOff=(0.0,0.0,0.0)cm ...
```

`travel` is how far your controller moved during that window. **Wave your hands about, then check
the most recent few:** varying, non-zero numbers mean poses are live and the problem is elsewhere
(calibration, or the mod standing down — see the vehicle and stick-mode notes below). `0.000`
throughout, or the *same* number repeated exactly, means the poses are frozen and nothing the mod
does downstream can help.

One thing **not** to read too much into: a `using_controllers=0` line. On the OpenXR runtime that
flag is only refreshed when a UEVR *action* fires, so it can read 0 on a perfectly healthy setup
where nothing has pressed anything yet. It is not proof your controllers are unbound, and it is not
worth reinstalling over. The `travel` number above is the reliable signal.

**My hands suddenly freeze (3DoF) but buttons still work — pressing something fixes it.**
Two known causes:
1. UEVR's motion-controls inactivity timeout. The mod raises it to its maximum (100 s) at startup,
   but the ceiling is UEVR's, not ours.
2. On the **OpenXR** runtime, losing session focus (opening the SteamVR dashboard, an overlay app,
   remoting into the PC) stops controller poses updating at the runtime level. Give the game focus
   and press something to recover. OpenVR is not exposed to this, since it sources poses from the
   compositor — but it mis-assigns the control layout, which is the worse problem, so OpenXR is
   the supported runtime and this is a known trade-off rather than a reason to switch.

**Tracking went wrong after I stood up / sat down.**
Expected, and it needs a **play area reset / recentre** to fix. The weapon and aim are referenced to
the play space as it was when the session started, so changing your real-world stance moves your head
to a height that reference does not account for and everything reads off.

Recentre through your runtime's play-area reset (SteamVR: long-press the system button, or Settings →
Play Area), and it comes back. Pick standing or seated before you inject and stay in it — if you do
want to switch mid-session, just expect to recentre afterwards.

**Aim feels laggy or overshoots on fast sweeps.**
With the shipping direct-drive aim this should not happen — pointing is written 1:1, with no
control loop to lag or overshoot. If it does: check `log.txt` for `DEV OVERRIDES ACTIVE` (a
leftover experiment may have switched the aim path), and try deleting `halo_vr_user.cfg` to rule
out a stale setting. Only the fallback stick-steer path (`blamangles=0`) has feel tuning —
`ffgain`, `dgain`, `dead` in `halo_vr_dev.cfg`.

**The ring reticule is missing at mission start.**
Its material streams in a few seconds after spawn; the mod rebinds automatically. If it never
appears, check `log.txt` for `reticule mesh asset` / `textured reticule` lines.

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

**Menus respond to the wrong buttons.**
Menu handling needs `menudetect=1` (default). In menus, right-controller B is Back and the
gameplay remaps are suspended; if a specific menu misbehaves, report which one.

**My in-game sensitivity changed the aim feel.**
The loop measures the game's turn rate and adapts (`gainadapt=1`). Give it a few seconds of
turning after changing sensitivity.

## Logs

Everything the mod does is logged with a `[Halo-CampE-UEVR]` prefix in
`%APPDATA%\UnrealVRMod\HaloCampaignEvolved\log.txt`. Include the tail of that file in bug reports.

## Uninstall

Delete `%APPDATA%\UnrealVRMod\HaloCampaignEvolved\`. Nothing else is touched.
