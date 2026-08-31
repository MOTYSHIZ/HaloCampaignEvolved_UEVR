// XrSource -- resolve the widget reticule's UE render target down to an ID3D12Resource.
//
// ============================================================================================
// WHAT THIS IS FOR
// ============================================================================================
// XrLayer (stage 1) draws a ring we generate on the CPU. It is bright and exposure-proof but
// STATIC: no firing bloom, no reload state, no hit marker, and it cannot match per-weapon reticle
// art. All of that animation belongs to the game's own crosshair widget, which Reticule.cpp
// re-parents onto a world-space UWidgetComponent.
//
// STAGE 2 makes the compositor layer PRESENT that widget's render target instead of our bitmap, so
// UE keeps drawing the animation and the layer becomes a presenter rather than a renderer. The seam
// is xrlayer_set_source(); this module is what fills it.
//
// ============================================================================================
// THE CHAIN, AND WHY IT IS DANGEROUS
// ============================================================================================
//   UWidgetComponent            g_ret_widget_comp        (Reticule.hpp)
//     -> GetRenderTarget()      UTextureRenderTarget2D*  REFLECTED -- a UFunction, safe
//     -> FTextureResource*                               NOT reflected: a native UTexture member
//     -> FRHITexture*                                    NOT reflected: FTexture::TextureRHI
//     -> ID3D12Resource*        via UEVR's frhitexture2d->get_native_resource()
//
// The last hop makes a VIRTUAL CALL on whatever pointer it is handed. A wrong candidate is not a
// bad pixel, it is an arbitrary vtable dispatch. So every rung below the reflected one is treated
// as an address-hygiene problem in the sense of the repo's rule and src\addrcascade\README.md:
//
//   1. NOTHING IS ASSUMED. The offsets are DISCOVERED by a bounded walk (dev-only), never written
//      down first and checked later. UEVR's own SDK records that this title's FD3D12Texture does
//      not carry FRHITextureDesc at any of the standard UE 5.5/5.6 offsets (0x20 / 0xE0 / 0xF0)
//      -- see StereoStuff.cpp, halo_meteorite_is_current_game() -- so a table of "known" offsets
//      would have been wrong here in exactly the silent way the rule exists to prevent.
//
//   2. CANDIDATES ARE FILTERED BEFORE THE VIRTUAL CALL, never after. A candidate FRHITexture is
//      only handed to get_native_resource() if its own memory already reads back the widget's
//      draw size as a pair of adjacent int32s, and its first qword points into a mapped image
//      (a vtable) while the object itself sits in private memory (a heap allocation).
//
//   3. THE RESULT IS VALIDATED AGAINST A NUMBER WE INDEPENDENTLY KNOW. The returned resource's
//      GetDesc() must report TEXTURE2D at exactly aim_widget_draw x aim_widget_draw, on the same
//      ID3D12Device UEVR reports. That is a ValueAgreement, not "does it look like a pointer".
//
//   4. IT FAILS CLOSED. Any doubt at any rung leaves the resolved resource null, XrLayer keeps
//      drawing the generated ring, and the in-scene reticule is untouched. "Could not verify"
//      never becomes "crash", and never becomes "refuse to draw anything".
//
// ============================================================================================
// USE A DISTINCTIVE DRAW SIZE WHEN PROBING
// ============================================================================================
// The filter keys on aim_widget_draw, whose default is 256 -- a number that occurs by coincidence
// all over a game's memory. Set aimwidgetdraw to something unusual (300, 260, 324) for the probe
// run and the coincidences vanish; the log says so at the top of every probe.
//
// The DISCOVERY WALK is dev-only (HALO_VR_DEV), because it is a bounded memory sweep that exists to
// answer a question. Re-resolving through already-latched offsets is cheap and stays in, so a
// shipping build can still USE offsets a dev build measured -- but it can never go looking for new
// ones, and it still re-validates the dimensions on every resolve.

// ============================================================================================
// THE PER-TICK NATIVE CACHE (xrlayersrccache, default on) -- WHAT IT COSTS AND WHY IT IS SAFE
// ============================================================================================
// MEASURED, not assumed (2026-08-24, live, dev split instrument): the ~16 ms/slot per-tick cost is
// NOT get_native_resource + GetDesc -- both are ~0.00 ms -- it is the pointer WALK, specifically
// looks_like_object()'s VirtualQuery-based classification. It classifies each hop (rt -> res -> rhi)
// with VirtualQuery via mem_is_private/mem_is_image plus a 16-slot vtable sweep, ~42 VirtualQuery per
// slot; VirtualQuery walks the kernel VAD tree, which on this game (~294k UObjects, a huge, growing
// address space) costs hundreds of microseconds each. So the steady cost is ~14.5 ms/slot and RISES
// with the session (16 -> 42 ms observed for one slot); six markers is 6x = the ~100 ms xrsource_tick
// the headset reported. (The earlier belief that get_native_resource synchronised the render thread
// was the hypothesis this instrument was built to test, and it disproved it.)
//
// THE VTABLE CLASSIFICATION EXISTS ONLY TO MAKE get_native_resource'S VIRTUAL CALL SAFE. On a CACHE
// HIT the cached ID3D12Resource is re-used and get_native_resource is NOT called, so that
// classification -- the whole cost -- is unnecessary. The hit path reaches rhi with readable_bytes
// guards only (~4 VirtualQuery) and re-uses the cache once the chain still leads to the SAME
// FRHITexture reading back want x want with a plausible desc.
//
// It does NOT weaken the 2026-08-23 crash fix (three access violations barriering an engine render
// target whose GPU backing had died across a level change -- see the g_owned commentary in
// XrLayer.cpp). Every guard that fix relied on still runs:
//   * A cache hit CANNOT happen after a re-host: service_slot drops the slot (clears t.native) the
//     moment the render-target UObject identity changes, and xrsource_tick the moment the component
//     changes -- both every tick, both cheap pointer compares. During a level load the game thread is
//     parked in LoadMap and this function is not called at all.
//   * Every dereference on the hit path is still guarded by readable_bytes (no fault on freed
//     memory); rhi == cached_rhi + extent == want + desc_plausible still prove it is the same texture.
//     Only the vtable sweep is dropped -- and it guards a virtual call the hit path does not make.
//   * An FRHITexture owns a fixed ID3D12Resource for its lifetime and UE allocates a NEW FRHITexture
//     when it recreates the GPU resource, so a cache keyed on the FRHITexture POINTER is equivalent
//     to re-deriving -- it just skips ~40 VirtualQuery.
//   * The dangerous barrier (xrlayer_capture_record) is on the GAME THREAD; the cache does not widen
//     any check-then-use window (the capture still immediately follows this validation).
// A CACHE MISS (first resolve, re-host, FRHITexture pointer changed) takes the FULL classified walk +
// get_native_resource + GetDesc, exactly as before. Set xrlayersrccache=0 to force that full walk
// EVERY tick (the pre-fix behaviour): the safe fallback and the A/B control for the measurement.

#pragma once

#include <cstdint>

namespace halo {

// GAME THREAD, once per tick from update(). Costs one bool test while both keys are off.
//
// Does the whole job: re-validates a latched chain, re-probes when it breaks, and hands the result
// to XrLayer when xrlayersrc is on. Throttled internally -- the walk never runs more than once a
// second and the cheap re-validation never runs more than a few times a second.
void xrsource_tick(uint32_t tick);

// ---- MORE THAN ONE COMPONENT ------------------------------------------------------------------
//
// The reticule's UWidgetComponent is found by this module itself (it is Reticule.cpp's, and there
// is exactly one). The navpoint markers are Plugin.cpp's pool and there are eight, so they are
// PUSHED IN rather than reached for: this module has no business knowing what a navpoint is, and
// the alternative -- exporting the pool -- would make its lifetime rules everybody's problem.
//
// Call every tick for every slot that should be resolved, and call with nullptr for the ones that
// should not. A slot whose component changes is dropped and re-resolved, the same way the reticule's
// is when the HUD re-hosts its crosshair.
//
// `want_dim` is the component's SetDrawSize edge. It is the ValueAgreement the whole chain is
// checked against, so it must be the number the caller actually passed to SetDrawSize -- not a
// guess, and not a default that happens to be right today.
//
// GAME THREAD, and it only records; the work happens in xrsource_tick().
void xrsource_set_slot_component(int slot, void* widget_component, int want_dim);

// SAME CONTRACT, ONE RUNG LOWER: for a caller that already holds the UTextureRenderTarget2D itself
// and has no UWidgetComponent in the picture at all -- the weapon scope, whose SceneCaptureComponent2D
// target is created directly (Scope.cpp owns that; this module only resolves and presents it).
//
// The chain below the entry point is IDENTICAL and deliberately shared rather than duplicated:
// UTextureRenderTarget2D -> FTextureResource -> FRHITexture -> get_native_resource(), the same
// offsets, the same per-tick re-validation, the same `want_dim` ValueAgreement, the same refusal to
// capture from a resource whose identity changed under it. Only the reflected GetRenderTarget call
// is skipped, because there is no widget to call it on.
//
// That this works at all is not an assumption: g_probe_rt inside this module has always walked a
// UTextureRenderTarget2D that the module CREATES ITSELF, precisely to learn those three offsets
// independently of any widget. This entry point is that same path, opened to a caller.
//
// `want_dim` is the render target's own edge (its SizeX, which for a square target is also SizeY).
// A source whose real dimensions disagree is REFUSED, not scaled -- see xrlayer_set_slot_source.
//
// GAME THREAD, records only; the work happens in xrsource_tick().
void xrsource_set_slot_render_target(int slot, void* render_target, int want_dim);

// THE OFFSETS ARE CLASS-LEVEL, NOT PER-INSTANCE -- and this proves it rather than assuming it.
//
// The claim the multi-component design rests on is that the FTextureResource / TextureRHI members
// live at the same offsets in every UTextureRenderTarget2D, because they are members of UTexture
// and FTexture rather than of any particular instance. That is almost certainly true and it is
// still an assumption, so a DEV build runs the discovery walk a SECOND time against a different
// component's render target and compares the two chains. Agreement is logged once; disagreement is
// logged loudly and the second chain is NOT adopted.
//
// Returns 0 = not attempted yet, 1 = agreed, -1 = disagreed. Always 0 in a shipping build.
int xrsource_chain_crosscheck_state();

// The validated ID3D12Resource for the widget's render target, or nullptr when unresolved.
// Never returns an unvalidated pointer.
void* xrsource_resource();

// The square edge length that resource was validated at, or 0.
int xrsource_dim();

// One-line state for the diagnostics block in Plugin.cpp.
const char* xrsource_status();

// Drop everything and forget the latched offsets. Called when the widget component is rebuilt (a
// mission transition re-hosts the crosshair and allocates a NEW render target, so the old
// ID3D12Resource is a dangling pointer we must never hand to the compositor).
void xrsource_reset();

}   // namespace halo
