# OpenXR headers (vendored)

Khronos OpenXR SDK headers, copied verbatim. Consumed by `src/XrLayer.cpp` and by the API layer in
`apilayer/`.

| | |
|---|---|
| Version | `XR_CURRENT_API_VERSION` = 1.0.22 |
| License | Apache-2.0 OR MIT (see the copyright banner in each file) |
| Source | The `openxr-src` dependency of the pinned praydog/UEVR checkout — `Tools\UEVR-src\UEVR\build\_deps\openxr-src\` in the private workspace |
| Files | `openxr.h`, `openxr_platform.h`, `openxr_platform_defines.h`, `loader_interfaces.h` |

`loader_interfaces.h` comes from a **different directory** of the same checkout —
`src/common/`, not `include/openxr/`. That is not an oversight in the SDK: it is the
loader↔layer negotiation ABI, which OpenXR 1.0 treats as an internal contract between the loader
and a layer author rather than part of the public application API (it only became a public header,
`openxr_loader_negotiation.h`, in 1.1). It is nonetheless the ABI every OpenXR API layer on Windows
is compiled against, and it is what `apilayer/src/LayerMain.cpp` implements. Copying it here rather
than retyping the structs from the specification is deliberate — a silent field-order or
`size_t`-vs-`uint32_t` mistake in `XrNegotiateApiLayerRequest` would not fail to compile, it would
hand the loader a garbage function pointer.

It `#include`s `<openxr/openxr.h>`, so anything that consumes it needs `src/thirdparty` on the
include path (the layer build does; the plugin build does not need this header at all).

Vendored rather than referenced for two reasons.

**The public repo has to build on its own.** `COMPILING.md` asks a contributor for a praydog/UEVR
checkout and nothing else; requiring them to also produce a configured UEVR *build tree* (these
headers only exist after CMake has fetched dependencies) would be a second, undocumented
prerequisite. `src/thirdparty/openvr.h` is here for the same reason.

**The version must match the loader UEVR actually calls.** We hook `xrEndFrame` and hand the runtime
structures we filled in ourselves, so our struct layouts have to agree with the ones UEVR was
compiled against. Taking the headers from UEVR's own dependency tree makes that agreement a fact
rather than a hope. If the UEVR pin in `Tools\UEVR\LATEST.txt` moves to a build with a different
OpenXR version, re-copy these files from the new checkout.

Do not edit them. They are generated from the Khronos XML registry.
