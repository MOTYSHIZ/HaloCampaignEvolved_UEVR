// The weapon scope -- left-trigger magnified view on a floating pane.
//
// Halo's native zoom is deliberately NOT used. The game's own zoom state exists and is even
// reflected (BlamUnitComponent.GetZoomLevelAbsolute / GetZoomMagnification), but engaging it has
// flat-screen side effects that are hostile in VR: weapon tags carry bHideWeaponOnZoom (the
// viewmodel vanishes), the HUD overlays a fullscreen zoom mask, and the sim halves look speed
// while zoomed ("zoomed look speed" in the tag data) -- which would bend the measured aim plant
// this driver's control law is tuned against. Halo-MCC-VR reached the same conclusion on the
// original engine and ships a synthetic gun-mounted zoom screen with native zoom suppressed at
// the input layer; this module is that shape, built from parts this plugin already proves in-game:
//
//   * a UTextureRenderTarget2D made with KismetRenderingLibrary (make_color_rt, as the reticule
//     colour target already does),
//   * our own SceneCaptureComponent2D aimed down the SAME ray the reticule traces -- the scope
//     shows where the shot goes, not where the gun model points,
//   * a Plane StaticMeshComponent pane displaying the target through a Widget3DPassThrough_Opaque
//     MID ("SlateUI" texture parameter -- the exact path the textured reticule uses, including
//     the TintColorAndOpacity-defaults-to-black trap).
//
// LEFT TRIGGER toggles the pane (rising edge past scopethresh). While the feature is enabled and
// the player is on foot outside menus, LT is EATEN before the game sees it, so Blam's own zoom
// can never engage underneath us. In menus and vehicle seats LT passes through untouched.
//
// Perf: the capture renders the scene again, so it only happens while the pane is visible, only
// every scopediv-th tick (~32 Hz / N), at a scoperes-squared target (512 default). Everything
// else in the steady state is the same class of cheap component writes the reticule makes.

#pragma once

// API.hpp, NOT Plugin.hpp -- Plugin.hpp defines the plugin entry points and may only be included
// by Plugin.cpp. See UeObject.hpp.
#include "uevr/API.hpp"
#include "Math.hpp"
#include "UeObject.hpp"

#include <atomic>
#include <cstdint>

namespace halo {

// The pane's on/off state. Written by the XInput hook (toggle), forced off by the tick when the
// feature is disabled or the first-person route dies. Read by both.
extern std::atomic<bool> g_scope_active;

// Diagnostics for the dev status line (written lock-free from the XInput hook, which must not
// log): how many toggle edges have fired, and the largest raw LT value the hook has seen.
extern std::atomic<uint32_t> g_scope_lt_edges;
extern std::atomic<uint8_t>  g_scope_lt_max;
// How many CaptureScene calls have been issued (manual mode) -- proves the call site runs.
extern std::atomic<uint32_t> g_scope_captures;

// XInput-hook side. Feed it the raw left trigger every poll; it handles the toggle edge and
// answers whether LT must be eaten (zeroed) before the game reads the state. No engine calls, no
// allocation -- same rule as everything else in that callback.
bool scope_handle_lt(uint8_t lt_raw, bool in_menu, bool stick_mode);

// Tick side. notice_ray() is called from the on-foot rig block with the RAW aim ray (origin ->
// target, unsmoothed -- the reticule's own display smoothing read as pane lag in the headset)
// and the rig component (its outer owns our components); it applies placement and capture
// IMMEDIATELY, same tick. frame_end() runs every update() tick above the early-outs and only
// parks the pane when no ray arrived, which is what makes menus, seats and death safe by
// construction.
void scope_notice_ray(const Vec3& origin, const Vec3& target, uevr::API::UObject* rig,
                      uint32_t tick);
void scope_frame_end(uint32_t tick);

} // namespace halo
