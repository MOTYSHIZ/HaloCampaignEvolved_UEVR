// XrLayerBridge -- the plugin's side of the OpenXR API layer, and a THIRD attachment tier.
//
// ============================================================================================
// WHERE THIS SITS
// ============================================================================================
// XrLayerAttach.hpp lists how the plugin can reach the OpenXR entry points UEVR actually calls:
//
//   BackendPdb    symbol lookup in UEVRBackend.pdb. Exact, and it cannot succeed for a player,
//                 who has no PDB -- though it IS compiled into release builds and is still
//                 asked as a fallback. See XrLayerAttach.hpp.
//   LoaderExport  openxr_loader.dll exports. Real addresses, wrong loader, kept as a rung for a
//                 host that dynamically links.
//
// Neither can SUCCEED on a player's machine. This module is the tier that can:
//
//   ApiLayer      the entry points are handed to us by XrApiLayer_HALOVR_reticule.dll, which the
//                 OpenXR loader loaded into this process through the ordinary API-layer chain.
//
// It needs no PDB, no signature, no recorded address and no hook. The layer is IN THIS PROCESS --
// an implicit API layer loads into the application, so this is two DLLs in one address space and
// the bridge is GetModuleHandleW plus one GetProcAddress. There is no IPC here and none is needed.
//
// ============================================================================================
// WHAT IT DELIBERATELY DOES NOT DO
// ============================================================================================
// It does not draw, it does not create a swapchain, it does not know what a reticule is. It answers
// two questions and stops:
//
//   1. Is the layer present in this process, and does it speak our ABI?
//   2. If so, what is the API table?
//
// Everything else -- the swapchain, the atlas, the pose maths, the capture, the fail-open contract
// -- already exists in XrLayer.cpp and is attachment-agnostic by construction. The intended wiring
// is that XrLayer.cpp's resolve_openxr() gains this as a tier ABOVE the PDB one (it is the shipping
// route, so it should win when both are available), and that its inline xrEndFrame hook is replaced
// by set_end_frame_callback() with the EXISTING hook body as the callback -- same thread, same
// moment, same code. That edit is not made here on purpose: this module is complete and inert on
// its own, so it can land without touching a file another session is holding.
//
// ============================================================================================
// NOTHING HERE MAY BE FATAL
// ============================================================================================
// The layer is OPTIONAL and always will be. A player who never registers it, a player on a runtime
// that does not walk the layer registry, a player whose registered path points at an old unpacked
// copy -- all of them must get exactly the behaviour they had before, which is the in-scene
// reticule, working. Every function here answers "no" cheaply and says why once.

#pragma once

#include "XrLayerAbi.h"

namespace halo {

// Probe for the layer. Cheap, idempotent, and safe to call from any thread at any time; the answer
// is computed once and cached. Safe to call before OpenXR exists -- it only looks for the module.
//
// Returns true if the layer is loaded in this process AND agreed our ABI version. Everything else --
// module absent, export missing, ABI refused -- is false with one log line saying which.
//
// NOTE what "true" does NOT mean: it does not mean an XrInstance exists yet, it does not mean the
// compositor is reachable, and it emphatically does not mean our layers are being presented. The
// layer's own frames_seen()/layers_appended() counters are the only honest answer to that, for the
// same reason XrLayer.hpp's xrlayer_live() exists: "installed" is not "running".
bool xrbridge_available();

// The API table, or nullptr. Never cache the pointer across a teardown -- the table itself is
// static in the layer, but the handles behind it are not, so ask each time you need one.
const HaloVrLayerApi* xrbridge_api();

// One line for the log: whether the layer was found, which build, and what its own status says.
// Never null, always safe to print.
const char* xrbridge_status();

}   // namespace halo
