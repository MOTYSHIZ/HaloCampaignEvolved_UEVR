#pragma once
//
// THE SCOPE PANE'S CLEARED-FRAME GUARD -- why the compositor scope used to flicker to black.
//
// THE RACE. A SceneCaptureComponent2D does not write its render target once. Every capture begins
// by CLEARING the target to black (SceneCaptureRendering.cpp, UE 5.5.4: `bClearRenderTarget =
// !bIsCompositing && !bEnableOrthographicTiling`, then AddClearRenderTargetPass(..., Black, ...))
// and only writes the finished image at the END of its render graph. Compositing -- the one mode
// that skips the clear -- exists only for the HDR scene-colour source, never for FinalColorLDR, so
// no capture property can turn it off for us.
//
// XrLayer copies that target into the compositor atlas from the GAME THREAD, onto the same queue,
// at a moment that is not synchronised with the render thread's work. When the copy lands between
// the clear and the final write, the atlas takes a BLACK frame and the scope shows black until the
// next copy -- "the scope flickers to black from time to time". The in-world pane never had this:
// the main view samples the target after the capture has finished, in the same frame's graph.
//
// THE GUARD. Immediately before the pane's copy, a one-thread compute probe reads nine points of the
// render target. If all nine are exactly black (the clear colour -- a lit, auto-exposed frame is
// essentially never pure zero at nine spread-out points), it writes "skip" into a tiny GPU buffer,
// and the copy is recorded under D3D12 PREDICATION on that buffer: the GPU skips the copy and the
// atlas keeps the previous good frame. Resource barriers are not predicated, so the state tracking
// around the copy is unchanged whether or not it runs. No CPU readback, no added latency.
//
// A GENUINELY BLACK SCENE MUST NOT FREEZE THE SCOPE: the probe counts consecutive skips in the same
// buffer and lets a frame through after `kMaxHold` of them, so a truly black view still shows.
//
// SAFETY, mirroring ScopeMask: every failure (compile, root signature, PSO, heap, buffer) latches
// the guard OFF, logs once, and leaves the plain copy exactly as it was -- fail open, never fail the
// pane. The probe only READS the engine's target, in the ENGINE_SRC_COLOR state the copy already
// assumes (which includes NON_PIXEL_SHADER_RESOURCE); it writes only our own buffer.
//
// GAME THREAD, called from xrlayer_capture_record() with the open capture list.

#include <cstdint>

struct ID3D12Device;
struct ID3D12GraphicsCommandList;
struct ID3D12Resource;

namespace halo {

// One descriptor pair per capture ring slot. MUST equal XrLayer's GT_RING (static_asserted there):
// the "rewrite only after capture_begin proved the slot's last list complete" argument is per slot.
constexpr int kScopeGuardSlots = 2;

// Record the probe and turn predication ON for what follows. `ring_slot` is the capture ring slot
// (0/1) whose previous GPU work xrlayer_capture_begin() has already proven complete -- the guard
// keeps one descriptor pair per slot, so rewriting them can never race a list still in flight.
// Returns true when predication was set; the caller must then call scopeguard_end() right after
// the copy it wants guarded. Returns false (and records nothing) when the guard is off or failed,
// in which case the copy simply runs unguarded, as it always did.
bool scopeguard_begin(ID3D12Device* device, ID3D12GraphicsCommandList* list, ID3D12Resource* src,
                      int dim, int ring_slot);

// Turn predication OFF again. Only after a scopeguard_begin() that returned true.
void scopeguard_end(ID3D12GraphicsCommandList* list);

}  // namespace halo
