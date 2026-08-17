# Building from source

The plugin is a handful of C++ files under `src/`, compiled with MSVC against the UEVR plugin SDK
headers. No CMake, no dependencies beyond the SDK.

**Adding a file needs no build-script change.** `scripts\build.ps1` compiles every `.cpp` under
`src/` recursively and prints the list each run, so a new file is picked up automatically — the same
way UEVR globs its own plugin targets.

## Prerequisites

- Visual Studio 2022+ Build Tools (any edition; only `cl.exe` for x64 is used)
- A checkout of [praydog/UEVR](https://github.com/praydog/UEVR) for its `include/` directory.
  Match the release you inject with — this mod targets **nightly-01138**:

  ```
  git clone https://github.com/praydog/UEVR
  git -C UEVR checkout 158232a
  ```

  The SDK headers are **not** vendored into this repository because UEVR's license is
  all-rights-reserved; consume them from praydog's repo directly.

## Build

```powershell
scripts\build.ps1 -SdkPath <path-to-UEVR-checkout>
```

or set `UEVR_SDK` in the environment and just run `scripts\build.ps1`. The output is
`build\halo_vr.dll`.

To build **and** install into your live UEVR profile in one step:

```powershell
scripts\build.ps1 -Deploy
```

which copies the DLL to `%APPDATA%\UnrealVRMod\HaloCampaignEvolved\plugins\`.

## Package a release

```powershell
scripts\package.ps1
```

builds the DLL, assembles `profile/` + the DLL into `build\HaloCampaignEvolved.zip` (zip root =
profile contents, so UEVR's Import Config lands everything in the right place), and prints the
zip's SHA-256 for the release notes.

## Source layout

| File | What lives there |
|---|---|
| `src/Math.hpp` / `Math.cpp` | Pure maths — quaternions, rotators, the calibration least-squares solve. **No plugin state, no UEVR API.** Per-frame helpers are `inline` in the header; calibration-only routines are in the `.cpp` |
| `src/Config.hpp` / `Config.cpp` | The `Config` struct, `g_cfg`, and everything touching the four files beside the profile, in read order: `halo_vr.cfg` (shipped **calibration data only**), `halo_vr_user.cfg` (the user's settings — generated on first run as the full commented catalog; never shipped, survives updates), `halo_vr_dev.cfg` (dev/troubleshooting catalog — ships fully commented) and `halo_vr_calib.cfg` (calibration results, applied last so they win). **The settings defaults are the struct's field initialisers** — changing one changes shipped behaviour |
| `src/UeObject.hpp` / `UeObject.cpp` | UE object plumbing: `TrackedObject` (recycle-safe handles), `FName` resolution, class-name lookup |
| `src/Rig.hpp` / `Rig.cpp` | The first-person weapon rig — making the visible gun and arms follow your hand, via **relative** component transforms |
| `src/Reticule.hpp` / `Reticule.cpp` | Both aim reticules (our own mesh one, and one hosting the game's own reticle widget) plus the asset-loading helpers that feed them |
| `src/MotionAimControl.hpp` / `.cpp` | The aim control loop — reading where the game is aiming, and the control law that steers it to follow your controller. Start here to understand the aim |
| `src/Plugin.cpp` | Everything else: `update()` (the per-tick loop), the stereo view lock, turning, movement, HUD discovery and follow, menu detection, and the plugin class |

Shared code lives in `namespace halo`. `Plugin.cpp` does `using namespace halo;` inside its anonymous
namespace, so call sites read the same whether a helper is local or in a module.

Two things to know before adding a module, both learned the hard way:

- **Include `uevr/API.hpp`, never `uevr/Plugin.hpp`.** `Plugin.hpp` *defines* the plugin entry points
  (`DllMain`, `uevr_plugin_initialize`), so including it from a second translation unit is a
  duplicate-symbol link error. Only `Plugin.cpp` may include it.
- **`Plugin.cpp`'s body sits in an anonymous namespace.** Anything defined there has internal linkage
  and cannot be linked against from a module. Shared state must be *defined* in a module's `.cpp` at
  `namespace halo` scope and declared `extern` in its header — not the other way round.

`Plugin.cpp` is still the largest file. Most of what remains is `update()`, the per-tick loop that
drives every module in order, so the next useful step is carving coherent stages out of *that*
function rather than moving more state around — the view lock and turning are the obvious candidates,
but they are read by the stereo render callback and the XInput hook, so they are best moved together
with the callbacks that own them.

If you split one, the pattern is:

1. Move the code into `Foo.hpp` / `Foo.cpp` inside `namespace halo`.
2. Declare shared globals `extern` in the header; define them once in the `.cpp`.
3. `#include "Foo.hpp"` from `Plugin.cpp` — no call-site edits needed.
4. Rebuild. **Watch for duplicate forward declarations**: a symbol that was fine as a local
   declaration in one translation unit becomes an ambiguous overload once the header also declares
   it. The compiler catches this, but the message points at the *call site*, not the declaration.

## Development builds (`HALO_VR_DEV`)

The plugin carries diagnostic tooling — material hunts, reflection dumps, render-target pixel
probes — used to investigate how the game renders. Each walks the entire UObject array (~296,000
objects on this title), so **none of it is compiled into a normal build**: it sits behind a
compile-time switch, in the spirit of Unreal's `WITH_EDITOR`.

```cpp
#include "DevTools.hpp"
#if HALO_VR_DEV
    ... diagnostic-only code ...
#endif
```

`HALO_VR_DEV` defaults to **0**, so `scripts\build.ps1` and CI produce a clean player build with
nothing to remember or strip. To compile the diagnostics in, add `/DHALO_VR_DEV=1` to the compiler
arguments in `scripts\build.ps1`.

The diagnostic and research keys themselves are catalogued in `profile/halo_vr_dev.cfg`, which
ships with the mod fully commented (packaging asserts the shipped copy is inert — uncommenting a
key is a deliberate, local act). The ones tagged `[dev build]` there only act when this switch is
compiled in — on a normal build they parse and do nothing, which is expected, not broken. Rebuild
with the switch defined.

## Notes for contributors

- The game must be closed while deploying — the DLL is locked while injected.
- Log output goes to `%APPDATA%\UnrealVRMod\HaloCampaignEvolved\log.txt`, prefixed `[Halo-CampE-UEVR]`.
- Nearly all tunables in `halo_vr.cfg` are re-read live (~2 s); iterate there before recompiling.
- The plugin deliberately reads the Blam camera's rotation as raw memory (`ControlRotation`,
  validated and fail-closed) because UE reflection access-violates on this game's Blam-backed
  objects. Do not "simplify" those reads into reflection calls.
