#pragma once
//
// FEATHERED OVAL ALPHA FOR THE SCOPE PANE'S ATLAS CELL.
//
// WHY THIS EXISTS AT ALL, AND WHY IT IS NOT fill_upload().
//
// Every other thing XrLayer draws -- the reticule ring, the navpoint markers, the grab guide beam,
// even GDI-rasterised text -- is CPU-GENERATED into a staging buffer and uploaded into its atlas
// cell. Authoring art is therefore a solved problem in this module, and the first answer to "can we
// have a feathered oval" was wrongly "we cannot author one". We can; that is what fill_upload does.
//
// The scope pane is different IN KIND. Its pixels do not come from the CPU: they arrive by
// CopyTextureRegion from the game's own render target, GPU to GPU. So the cell holds scene RGB that
// no CPU path ever touches, and CopyTextureRegion cannot copy a single channel -- there is no way to
// keep that RGB and substitute a CPU-authored alpha in one copy.
//
// A second quad cannot do it either: composition layers BLEND OVER one another, so an oval-alpha
// quad laid on top tints the pane, it cannot cut a hole in it.
//
// That leaves a per-pixel write to the cell after the copy, which is a shader. Hence this file.
//
// THE OVAL IS ANALYTIC, so there is no mask texture and no upload. The shader turns the cell's
// pixel coordinate into -1..1, divides by per-axis radii (rx > ry gives WIDER THAN TALL, which is
// what a scope wants) and smoothsteps the edge. Changing shape is a constant, not an asset.
//
// SAFETY, and it is the reason for the shape of this API:
//   * The atlas is created ONCE at bring-up and has crashed this module three times when its
//     lifetime was got wrong. Writing to it needs D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS, which
//     is a creation-time decision -- so scopemask_wanted() is consulted AT BRING-UP, and with the
//     feature off the atlas is created byte-identically to before. An unconfigured build carries
//     none of this.
//   * Every failure here is non-fatal and latching: a shader that will not compile, a heap that
//     will not allocate, a device that refuses the PSO -- all of them disable the mask, log once,
//     and leave the pane exactly as it is without the mask. A cosmetic feature must never be able
//     to take the pane down with it.
//   * D3DCompile is resolved with GetProcAddress, deliberately, so this adds no link dependency --
//     the same choice ScopeBlit.cpp made and for the same reason (both build scripts link only
//     user32.lib, and neither should have to change for a cosmetic mask).
//
// THE ALPHA INTERACTION, WHICH IS THE THING TO GET WRONG. Slot 9 is currently submitted OPAQUE
// (layerFlags 0) because a scene capture's alpha is ~0 and blending by it made the whole pane
// disappear except where the captured reticule had written alpha. This mask is what makes that
// alpha MEANINGFUL, so blending may only be re-enabled for slot 9 when the mask actually ran this
// frame. scopemask_applied() reports that, and XrLayer gates on it. Re-enabling blending without a
// mask that wrote alpha reproduces the invisible pane exactly.

#include <cstdint>

struct ID3D12Device;
struct ID3D12GraphicsCommandList;
struct ID3D12Resource;

namespace halo {

// Read at ATLAS CREATION TIME, because the UAV flag cannot be added later. Off => the atlas is
// created exactly as it always was.
bool scopemask_wanted();

// MUST be called when the atlas is created, saying whether it actually got
// ALLOW_UNORDERED_ACCESS. Without this the apply path cannot tell a UAV-capable atlas from one
// built before the key was set -- and CreateUnorderedAccessView on a resource that lacks the flag
// is invalid usage: device removed, process gone. That is exactly what a LIVE scopemask=1 did on
// 2026-09-07, because the flag is a creation-time decision and the atlas was already up.
//
// A live toggle is therefore recorded and takes effect at the NEXT bring-up, the same contract
// xrlayer_pane_configure() already has for the cell size, and for the same reason.
void scopemask_note_atlas(bool created_with_uav);

// Multiply the alpha of one square atlas cell by a feathered oval. Call AFTER the scene has been
// copied into the cell and while `atlas` is in D3D12_RESOURCE_STATE_COPY_DEST -- the transitions to
// UNORDERED_ACCESS and back are made and unmade here, so the caller's state tracking is unchanged
// on return.
//
// Returns false if the mask did not run, for any reason. False means the cell still holds the
// capture's own alpha and blending must NOT be enabled for it.
bool scopemask_apply(ID3D12Device* device, ID3D12GraphicsCommandList* list, ID3D12Resource* atlas,
                     int cell_x, int cell_y, int cell_dim);

// True only if the most recent scopemask_apply() actually wrote alpha. This is what gates the
// per-slot blend flag; it is deliberately not "the feature is enabled".
bool scopemask_applied();

// Release the shader, PSO, root signature and heap. Safe to call more than once, and safe when
// nothing was ever created.
void scopemask_shutdown();

}   // namespace halo
