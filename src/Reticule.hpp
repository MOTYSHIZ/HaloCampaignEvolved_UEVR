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

// The hosted widget component, exposed because a debug command in Plugin.cpp inspects it.
extern TrackedObject g_ret_widget_comp;

// ---- mesh reticule ---------------------------------------------------------------------------
// ensure() creates on demand and latches on failure; move() repositions. Safe to call every tick.
void reticule_mesh_ensure(uevr::API::UObject* rig);
void reticule_mesh_move(const Vec3& p);

// ---- widget reticule -------------------------------------------------------------------------
void reticule_widget_ensure(uevr::API::UObject* rig);
void reticule_widget_move(const Vec3& target, const Vec3& origin);

// The widget class the config asks us to host.
std::wstring wanted_widget_class();

// True while the widget reticule still has to CHOOSE a widget out of g_reticles -- i.e. while the
// object-array scan that fills g_reticles still has a consumer.
//
// Exposed so reticle_rescan() can stop sweeping once nothing reads its output. Picking is one-shot:
// after the component binds (or latches failed) this goes false and stays false, unless the binding
// is dropped, in which case it goes true again and the scan resumes on its own.
bool reticle_widget_needs_pick();

} // namespace halo
