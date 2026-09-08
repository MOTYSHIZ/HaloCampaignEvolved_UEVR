# `XrApiLayer_HALOVR_reticule` — the OpenXR API layer

This folder builds a **second, separate DLL**. It is not part of `halo_vr.dll` and must never be
compiled into it.

| | |
|---|---|
| Source | `apilayer/src/LayerMain.cpp` — one file, no dependencies beyond the Windows SDK |
| Shared contract | `src/XrLayerAbi.h` — **one copy**, compiled by both sides |
| Manifest | `profile/apilayer/XrApiLayer_HALOVR_reticule.json` (ships) |
| Build | `scripts/build-apilayer.ps1` |
| Test | `scripts/Verify-XrApiLayer.ps1` — **run it after any change here** |
| Install tooling | `profile/apilayer/Register-XrApiLayer.ps1` / `Unregister-XrApiLayer.ps1` |

## Why it exists

`src/XrLayer.cpp` draws the aim reticule as an OpenXR **quad composition layer**. A composition
layer is handed to the runtime after the whole post chain, so scene exposure, eye adaptation and the
tonemapper never touch it — it is bright by construction instead of fighting a varying term with a
constant multiplier. That part works and is verified in a headset.

Getting *called* was the problem. UEVR is the OpenXR application and the plugin is a guest inside it;
UEVR's plugin API offers no way to add a layer to the list it passes to `xrEndFrame`, and UEVR
**statically links the OpenXR loader into `UEVRBackend.dll`**, so there is no exported `xrEndFrame`
to hook. The working attachment resolves the address by symbol out of `UEVRBackend.pdb`
(`src/XrLayerAttach.hpp`) — exact, correct, and impossible to ship, because only a UEVR checkout has
that PDB.

An **API layer** is handed `xrEndFrame` by the loader through the ordinary dispatch chain. No hook,
no symbol, no recorded address, nothing to rot when UEVR's pin moves. That is what this folder is.

## The division of labour, and where the line is

The layer owns exactly two things the plugin cannot obtain on its own:

1. **`xrEndFrame`.** It calls the plugin back from inside it so the plugin can append its quads.
2. **A down-chain `xrGetInstanceProcAddr`.** So the plugin's `xrCreateSwapchain` and friends come
   from the *same loader instance* as the session UEVR handed it. `XrLayerAttach.hpp` is emphatic
   about why: entry points from one loader driving another loader's session is "an unrelated pointer
   being dereferenced as a dispatch table", and it fails as memory corruption rather than as an
   error code.

Everything else stays in `XrLayer.cpp`, where it already works: swapchain creation, the texture
atlas, the blit, the pose maths, the owned-texture capture, the fail-open contract. **Do not migrate
any of it here.** The layer is thin on purpose — code in this DLL is code that also gets loaded into
every other OpenXR application on the user's machine.

## The rule that governs every edit in this folder

An implicit API layer is registered once, per user, and from then on the OpenXR loader loads it into
**every OpenXR application that user runs** — their other VR games, their headset's utilities,
SteamVR Home. That is the price of not needing per-process environment variables, and it is why the
layer's first act is to check whether it is in the game and, if not, become a pure pass-through:
nothing wrapped, no callback, no allocation, no file touched.

Concretely:

- **Nothing may happen before the gate.** No file I/O in `DllMain`, no registry reads, no threads,
  no static-initialiser work. If you add something and are unsure whether it runs in a non-Halo
  process, it does.
- **Negotiation must always succeed.** Declining to negotiate is the tidier-looking opt-out, and a
  compliant loader is supposed to skip a layer that does it — but the blast radius of that belief
  being wrong is every OpenXR application the user owns failing to start because of a mod for a
  different game. A pass-through cannot break an application even if the loader's error handling is
  not what we believe.
- **Never log per frame.** A hitch here is nausea, and this code runs on the submit thread.

## Gate and kill switches

| | |
|---|---|
| Active when | the host executable is `HaloCampaignEvolved.exe` or `Meteorite-Win64-Shipping.exe` |
| `HALOVR_LAYER_FORCE=1` | force active regardless of host — for the self-test and for debugging in a sample app. Per-process, so it cannot leak to a player |
| `HALOVR_LAYER_DISABLE=1` | force inert. This is the manifest's `disable_environment`, so the loader honours it for an implicit registration *and* the layer honours it itself on the environment-variable route |

The process-name gate is permissive by nature — another program could share the executable name. The
consequence is bounded: the layer would wrap `xrEndFrame` and add nothing, because no callback is
ever registered without `halo_vr.dll` also being in the process.

## Activating it

**Development — per process, no registry, zero machine-wide effect.** The same mechanism
`Scripts\Enter-Mission-OpenXR.ps1` already uses for the Meta XR Operator layer:

```powershell
$env:XR_API_LAYER_PATH    = '<folder containing the DLL and its .json>'
$env:XR_ENABLE_API_LAYERS = 'XR_APILAYER_HALOVR_reticule'
# ...then launch the game from this same environment.
```

Use this for everything you test yourself. It affects one launch and nothing else.

**Shipping — implicit registration.** Players launch from Steam and cannot set a per-process
environment variable, so the loader has to be told about the layer once, in the registry:
`profile\apilayer\Register-XrApiLayer.ps1`. **HKCU only, never HKLM** — per-user, no administrator
rights, reversible by a script that ships beside it.

Two things about that registration are worth knowing before debugging it:

- The loader reads `HKEY_CURRENT_USER` **only for non-elevated processes**. This is deliberate on
  the loader's part — it stops a normal-privilege program injecting code into an elevated one. If
  the game is run as administrator, a per-user registration is ignored and there is nothing this
  repo can do about it short of asking for admin rights, which it will not.
- The value name is the **full path to the manifest**; the data is a `DWORD` and must be `0`
  (anything else means "skip"). The manifest's `library_path` is relative, so the loader looks for
  the DLL beside the manifest — the two must never be separated.

## Testing

`scripts/Verify-XrApiLayer.ps1` builds the layer and drives it through a **fake OpenXR loader**
(`apilayer/test/LayerSelfTest.cpp`): it negotiates by hand, walks the layer through
`xrCreateApiLayerInstance` with a stub chain underneath, submits frames, and asserts on **what the
stub runtime received** rather than on what the layer says it did. No game, no headset, no runtime,
a few seconds.

It runs three times, because the process-name gate is decided once per process and cached: **active**,
**gated**, and **disable-override**. The gated run matters more than the active one — it is the half
that runs in other people's games.

What it deliberately does **not** prove, so that a green run is not read as more than it is:

- That the **real** loader accepts this layer. The negotiation ABI here is compiled from the same
  header the loader is built from (`src/thirdparty/openxr/loader_interfaces.h`, taken verbatim from
  UEVR's own OpenXR dependency), but the harness cannot catch a disagreement between that header and
  the loader binary on a player's machine.
- That **UEVR's statically-linked loader walks the implicit-layer registry**. The evidence is strong
  — a third-party implicit layer is observed loaded inside the game process alongside UEVR — but
  evidence is not proof, and only a live run settles it.
- Anything about D3D12, swapchains, poses or pixels. None of that is in this DLL.
