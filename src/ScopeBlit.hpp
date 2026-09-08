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
void scope_blit_tick();

} // namespace halo
