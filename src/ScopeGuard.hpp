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
// A GENUINELY BLACK SCENE MUST NOT FREEZE THE SCOPE -- AND THAT ESCAPE HATCH MUST NOT OPEN FOR THE
// CLEAR ITSELF. The first version counted consecutive skips and let a frame through after 3 of
// them, whatever it was. Right after the scope opens the copy lands on the clear up to a quarter of
// the time (16 of 64 guarded copies, measured 2026-09-28), so runs of four happened and the fourth
// -- the clear -- went to the compositor: the "occasional black flicker" that survived the guard.
//
// THE ALPHA TELLS THE TWO APART. The clear is FLinearColor::Black, which is (0,0,0,1) -- alpha ONE;
// a finished FinalColorLDR frame carries alpha 0 (the desktop tonemapper starts from OutColor = 0 and
// only its DIM_ALPHA_CHANNEL permutation writes .a; ScopeMask exists because that alpha reads ~0).
//   * exactly (0,0,0,1) at all nine points = the clear. Held up to kMaxHoldClear (30) copies: no
//     race outlasts that, and a capture that cleared and then never wrote still shows within ~0.5 s.
//   * any other all-black reading = possibly a real black view. The old short hold, kMaxHold (3).
// A frame let through resets the hold, but the atlas then keeps THAT frame while the next ones are
// refused, so a capture stuck on its clear shows steady black rather than flashing.
//
// SAFETY, mirroring ScopeMask: every failure (compile, root signature, PSO, heap, buffer) latches
// the guard OFF, logs once, and leaves the plain copy exactly as it was -- fail open, never fail the
// pane. The probe only READS the engine's target, in the ENGINE_SRC_COLOR state the copy already
// assumes (which includes NON_PIXEL_SHADER_RESOURCE); it writes only our own buffer.
//
// GAME THREAD, called from xrlayer_capture_record() with the open capture list, or from
// xrlayer_pane_record() with the pane's own list (whose SUBMISSION happens later, at Present --
// the probe then reads the target when that list executes, which is the moment that matters).

#include <cstdint>

struct ID3D12Device;
struct ID3D12GraphicsCommandList;
struct ID3D12Resource;

namespace halo {

// One descriptor pair per list that can carry a guarded copy: XrLayer's game-thread batch ring
// (GT_RING, pairs 0..1) plus the pane's present-submitted ring (PANE_RING, pairs 2..5). MUST equal
// GT_RING + PANE_RING (static_asserted there): the "rewrite a pair only once its list is proven
// complete, or was never submitted" argument is per pair, and the two rings are live at once while
// scopepresentcopy is being toggled, so they cannot share pairs.
constexpr int kScopeGuardSlots = 6;

// Record the probe and turn predication ON for what follows. `ring_slot` is the descriptor pair
// (0..kScopeGuardSlots-1) of a list whose previous GPU work is already proven complete or was never
// submitted -- the guard keeps one pair per list, so rewriting it can never race a list in flight.
// Returns true when predication was set; the caller must then call scopeguard_end() right after
// the copy it wants guarded. Returns false (and records nothing) when the guard is off or failed,
// in which case the copy simply runs unguarded, as it always did.
bool scopeguard_begin(ID3D12Device* device, ID3D12GraphicsCommandList* list, ID3D12Resource* src,
                      int dim, int ring_slot);

// Turn predication OFF again. Only after a scopeguard_begin() that returned true.
void scopeguard_end(ID3D12GraphicsCommandList* list);

}  // namespace halo
