#pragma once

// UEVR's VR_WorldScale AS THE PLAYER SET IT -- the one reader every feature uses.
//
// UEVR draws a real metre of head or hand movement as 100 x VR_WorldScale UE centimetres, and
// players choose their own scale: the profile ships 1.312, plenty run 1.0 or somewhere between.
// Anything that carries a length or a speed between the ROOM (real metres) and the WORLD (UE cm)
// has to go through this value. A bare x0.01 is right at 1.0 and quietly wrong everywhere else --
// roomscale used exactly that, and at 1.312 slid the view back by 31% of every step it took.
//
// WHY NOT READ IT WHERE IT IS NEEDED: four places did, with three different guards.
//   * It is a string round-trip through UEVR's config, so it is re-read every 2 s and cached, like
//     every other live tunable. A change in UEVR's menu lands within that.
//   * THE PLUGIN ITSELF REWRITES IT. The cutscene mono collapse (cutscene2d=2) drops VR_WorldScale
//     to 0.01 for the scene and restores it afterwards. That is the scene, not the player's
//     setting, so a value under 0.1 is never taken and the last good one is kept. Two of the old
//     readers fell back to 1.0 there instead, losing the player's scale for the whole cutscene.
//   * Every consumer sees the SAME value at the same moment, so the arms, the height, roomscale
//     and the compositor layer cannot disagree for a poll after the player changes scale.
//
// Mono RENDERING (VR_RenderingMethod=3) is a different thing and needs nothing here: it renders one
// view, but head motion still maps into the world through this same scale. Only the collapse
// rewrites the value.
//
// THREADS. uevr_world_scale() may refresh, and a refresh calls into UEVR, so it is for the GAME
// THREAD. Anything else -- the render pass, the sim thread, the XInput hook -- uses the _cached form,
// which is one atomic load and never touches UEVR. The cache cannot go stale while the plugin runs:
// world_scale_resolve_config() refreshes it on every config poll, whichever features are on.

namespace halo {

// GAME THREAD. The scale, e.g. 1.312. 1.0 until UEVR has answered once -- exactly the old bare-x0.01
// behaviour, so nothing is worse before the first read than it was.
float uevr_world_scale();

// GAME THREAD. UE cm per real metre: 100 x uevr_world_scale(). What most conversions want.
inline float uevr_cm_per_metre() { return 100.0f * uevr_world_scale(); }

// ANY THREAD. The cached value; never refreshes, never calls UEVR.
float uevr_world_scale_cached();
inline float uevr_cm_per_metre_cached() { return 100.0f * uevr_world_scale_cached(); }

// GAME THREAD. True once a real value has been read -- for a caller that must not act on the 1.0
// fallback.
bool uevr_world_scale_known();

// GAME THREAD, called by the registry's features_apply() as the last write of every config reload,
// BEFORE hook threads go back to reading g_cfg. Resolves rig_scale: it follows UEVR's world scale
// unless rigscale was set explicitly (worldscalefollow=1 makes the world scale win even then).
// Resolving INSIDE the reload is the point: set afterwards, a sim-thread reader could catch the
// compiled default between the reset and the fix-up -- a one-frame arm jump every poll at any
// scale but the profile's.
void world_scale_resolve_config();

} // namespace halo
