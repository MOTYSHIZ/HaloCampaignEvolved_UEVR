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
// Every caller today is on the game thread. The cache is atomic, so a read from another thread is
// safe; only the refresh calls into UEVR.

namespace halo {

// The scale, e.g. 1.312. 1.0 until UEVR has answered once -- exactly the old bare-x0.01 behaviour,
// so nothing is worse before the first read than it was.
float uevr_world_scale();

// UE cm per real metre: 100 x uevr_world_scale(). What most conversions actually want.
inline float uevr_cm_per_metre() { return 100.0f * uevr_world_scale(); }

// True once a real value has been read. For a caller that must not act on the 1.0 fallback
// (WorldScaleFollow writes rig_scale from it).
bool uevr_world_scale_known();

} // namespace halo
