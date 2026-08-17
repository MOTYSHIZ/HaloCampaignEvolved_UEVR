// The aim reticule -- the public surface consumed by update() in Plugin.cpp.
//
// Two independent reticules, either of which can be off: a mesh reticule we create ourselves, and a
// widget reticule that hosts one of the game's own reticle widgets. See Reticule.cpp for why both
// exist and why the debug-draw route is not an option.

#pragma once

// API.hpp, NOT Plugin.hpp -- Plugin.hpp defines the plugin entry points and may only be included
// by Plugin.cpp. See UeObject.hpp.
#include "uevr/API.hpp"
#include "Math.hpp"
#include "UeObject.hpp"

#include <atomic>
#include <cstdint>
#include <string>

namespace halo {

// ---- resolved HUD reticle widgets ------------------------------------------------------------
// Populated by reticle_rescan() in Plugin.cpp, consumed here by the widget reticule (which hosts
// one) and there by the HUD-follow code (which moves them). The storage is in Reticule.cpp: it
// cannot live in Plugin.cpp, whose body is an anonymous namespace and so would give it internal
// linkage that this module could not link against.
struct ReticleTarget {
    TrackedObject obj;
    uint32_t      found_tick = 0;
};
extern ReticleTarget g_reticles[8];
extern int           g_reticle_count;
extern uint32_t      g_reticle_scan_tick;

// ---- ray origin ------------------------------------------------------------------------------
// Published so the mesh reticule can face the viewer the same way the widget does.
extern Vec3 g_ret_origin;
extern std::atomic<bool> g_have_ret_origin;

// APPARENT-SIZE COMPENSATION. Both reticules are placed at a fixed WORLD scale, so their on-screen
// size is inversely proportional to how far away they are put. The seated reticule sits much
// further out than the on-foot one (a close ring would be inside your own vehicle), which shrank it
// to a speck -- it was being drawn correctly and simply could not be seen. Callers set this to
// (actual distance / the distance the scale was tuned at) and the size holds steady.
extern std::atomic<float> g_ret_scale_mul;

// Re-assert visibility on both reticule components, and report the mesh's LIVE world position.
//
// Taking a vehicle seat tears down the pawn's first-person presentation -- the same teardown stick
// mode detects. Our components hang off that pawn, so if they are hidden along with it the
// reticule is placed correctly every tick and still draws nothing. This forces them back on and
// hands back what the component itself says about its position, because on this title a
// successful call is not evidence the object moved.
//
// `out_have` receives a bitmask: 1 = mesh component live, 2 = widget component live.
// Returns true when out_pos was filled from the mesh's own getter.
bool reticule_force_visible(Vec3* out_pos, int* out_have);

// The hosted widget component, exposed because a debug command in Plugin.cpp inspects it.
extern TrackedObject g_ret_widget_comp;

// True when the material the component is ACTUALLY rendering with cancels scene exposure itself, so
// the gain must be 1.0 rather than aimwidgetgain. Exposed here so the brightness instrument in
// Plugin.cpp can report it alongside everything else in the chain.
extern bool g_ret_widget_exposure_compensated;

// ---- mesh reticule ---------------------------------------------------------------------------
// ensure() creates on demand and latches on failure; move() repositions. Safe to call every tick.
void reticule_mesh_ensure(uevr::API::UObject* rig);
void reticule_mesh_move(const Vec3& p);

// Find a material by path, loading it from disk if necessary (bare object path or class-prefixed).
// Exposed for the navpoint markers, which need the same exposure-compensated pass-through chain
// the widget reticule uses -- the stock Widget3D material tonemaps to near-black in bright scenes.
uevr::API::UObject* find_or_load_material(const std::string& object_path);

// ---- WORLD-SPACE WIDGET QUAD -- THE ONE COPY OF THIS RECIPE ------------------------------------
//
// Building a UWidgetComponent that actually renders in world space on this title is a sequence of
// non-obvious, order-dependent steps (deferred construction credited to OblivionVR, a BlendMode
// property write at a MEASURED offset because there is no setter, the exposure-compensated
// pass-through MIC with a stock fallback, and registration LAST so the render target and scene
// proxy are built from finished state). It took the whole 2026-08-14 render hunt to establish, and
// the navpoint markers need exactly the same thing as the reticule.
//
// So it lives here ONCE and both callers use it. A second hand-rolled copy would put that measured
// offset and that ordering in two places, where a game patch fixes one and silently rots the other
// -- the same trap the repo's "never a second copy" rule exists for.
//
// USE:  comp = widget_quad_begin(owner, blend, &compensated);
//       ... caller sets ITS widget (SetWidget / host_widget) and draw size ...
//       widget_quad_finish(owner, comp, bounds_scale);
//
// `out_exposure_compensated` reports whether the VREditor MIC bound (unity tint gain) or the stock
// one did (the caller must apply gain itself). Returns nullptr on failure, having logged why.
uevr::API::UObject* widget_quad_begin(uevr::API::UObject* owner, int blend_mode,
                                      bool* out_exposure_compensated);

// Register the deferred component and apply the anti-cull bounds. MUST be called after the widget
// is set: registration builds the render target, and a component registered with no widget builds a
// DEGENERATE (0,0) quad that renders nothing -- a failure that cost a full field session.
void widget_quad_finish(uevr::API::UObject* owner, uevr::API::UObject* comp, float bounds_scale);

// ---- widget reticule -------------------------------------------------------------------------
void reticule_widget_ensure(uevr::API::UObject* rig);
void reticule_widget_move(const Vec3& target, const Vec3& origin);

// Hand the game's crosshair back to the HUD and park our quad. Call when aimwidget goes 0 -- hosting
// removes the widget from the HUD, so without this the toggle is one-way and leaves the player with
// no crosshair. Safe and cheap to call when nothing is bound (every handle is re-validated first).
void reticule_widget_release();

// The widget class the config asks us to host.
std::wstring wanted_widget_class();

// True while the widget reticule still has to CHOOSE a widget out of g_reticles -- i.e. while the
// object-array scan that fills g_reticles still has a consumer.
//
// Exposed so reticle_rescan() can stop sweeping once nothing reads its output. Picking is one-shot:
// after the component binds (or latches failed) this goes false and stays false, unless the binding
// is dropped, in which case it goes true again and the scan resumes on its own.
bool reticle_widget_needs_pick();

// STRAY NATIVE CROSSHAIRS. Hosting removes the game's crosshair from its parent; it does not stop
// the HUD building another one, which a mission transition does. These let the existing scan clean
// that up without becoming a standing poll -- see the block above their definitions in Reticule.cpp.
void reticle_arm_stray_check();              // one-shot window, armed when a widget is hosted
bool reticle_stray_check_due(uint32_t tick); // is that window open?
void reticle_collapse_strays();              // hide every scanned reticle that is not ours

} // namespace halo
