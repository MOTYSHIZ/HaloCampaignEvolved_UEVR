// DIGITAL ZOOM -- magnify a crop of the frame the engine ALREADY rendered.
//
// WHY THIS EXISTS
// The scope's other path re-renders the scene into a SceneCaptureComponent2D. That capture loses
// effects: a scene-colour source has EngineShowFlags.PostProcessing force-disabled by the engine
// (SceneCaptureRendering.cpp:853), so an emissive object arrives with its base colour and without
// its bloom -- which against dark geometry reads as "the effect is missing entirely", and cost a
// week of chasing six refuted mechanisms.
//
// This path does not re-render anything. UEVR hands us the scene render target
// (uevr::API::StereoHook::get_scene_render_target), which on this title MEASURED as 3840x1080 --
// the stereo pair side by side, 1920x1080 per eye -- in B8G8R8A8_TYPELESS, i.e. the FINAL
// post-processed image. Sampling that gets bloom, tonemapping and every other effect correct BY
// CONSTRUCTION, and costs no second scene pass at all.
//
// The magnification is Blam's own: GetZoomMagnification() reports 2.00 for the magnum
// (measured in headset 2026-08-23), so per-weapon zoom comes free and authored.
//
// STATUS: STAGE 1. This draws the magnified crop as a SCREEN-SPACE quad at a configurable
// position. It is NOT yet locked to the gun -- stage 2 projects the world-space pane's position
// per eye and puts the quad there. Screen-space first because the D3D12 machinery is the risky
// part and is worth proving on its own.
//
// SAFETY: defaults OFF, initialises once, and disables itself permanently on the first failure
// rather than retrying on a render callback. Nothing here runs unless scopeblit is set.

#pragma once

#include <cstdint>

namespace halo {

// Called from the game-thread tick. Registers the render callback exactly once, the first time
// the feature is switched on -- kept out of Plugin.cpp deliberately so this feature can land
// without touching a file other sessions are editing.
// REGISTER THE RENDER CALLBACK. Call ONCE, FROM on_initialize, AND NEVER FROM A TICK.
//
// This was scope_blit_tick() and was called from update(). That is a GUARANTEED SELF-DEADLOCK and
// it hung the game twice on 2026-09-08:
//
//   PluginLoader::on_pre_engine_tick()  takes std::shared_lock{m_api_cb_mtx}
//     -> our update() runs INSIDE that lock
//       -> add_on_post_render_vr_framework_dx12() takes std::unique_lock{m_api_cb_mtx}
//
// Same std::shared_mutex, same thread. It is neither recursive nor upgradeable, so the writer waits
// for every reader to release and one of those readers is the thread now blocked in the writer.
// The general rule, and it is not specific to this callback: YOU CANNOT REGISTER A UEVR API
// CALLBACK FROM INSIDE A UEVR API CALLBACK. All 17 dispatchers share that one mutex.
//
// Registration is UNCONDITIONAL -- deliberately not gated on scopeblit/cutsceneblit. The callback
// body already tests both, so an unused registration costs one bool per frame, and gating it here
// is what made the feature need a tick-time registration in the first place. It also means either
// key can be switched on at RUNTIME and simply start working.
void scope_blit_register();

// CUTSCENE MODE. Published from the game thread by the ONE cutscene predicate in Plugin.cpp
// (cine_signal -- see the oscillation note there: engage and release must derive from the same
// expression), read on the RENDER thread by the blit callback. Atomic because those are two
// different threads; a plain bool here would be the two-clocks bug in miniature.
//
// Shares this file's device/root-signature/PSO/SRV state deliberately rather than standing up a
// second copy in its own translation unit: the callback list is additive (PluginLoader
// push_back + dispatch loop), so a separate file WOULD work -- it would just duplicate ~200 lines
// of D3D12 setup for no gain, which this repo's one-copy rule exists to prevent.
void cutscene_blit_set_active(bool on);

} // namespace halo
