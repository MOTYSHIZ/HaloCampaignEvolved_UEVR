// HeadBlock -- keep the rendered head out of world geometry.
//
// In roomscale the body stops at a wall but the head does not: lean with your feet beside the
// wall and the camera goes through it. The body's own eye (the engine camera, before UEVR adds the
// head offset) is always in free space, because the character collides. So the rendered eye is
// pulled back along the body->head offset until it is clear.
//
// ONLY THE RENDERED EYE MOVES. The standing origin, the aim, the hands and the roomscale walk are
// untouched: nothing that aims or moves reads the adjusted position except the eye publish that
// markers and the reticule already derive from, which is what they should see.
//
// MODES (headblock):
//   1 LINE TRACE  -- body eye -> head, extended by the radius; clamp to the hit minus the radius.
//   2 SPHERE SWEEP -- a sphere of the radius swept body eye -> head; clamp to where it stops.
//   3 LEAN LIMIT  -- no trace: cap the HORIZONTAL head offset from the body at headblocklean.
//
// The trace runs on the game tick; the clamp is applied per eye in the stereo view callback with
// the latest allowed distance, so it follows the head at render rate. Tightening is immediate
// (nothing may be seen through a wall), loosening is rate-limited (no pop when a hit clears).

#pragma once

#include "Math.hpp"
#include "uevr/API.hpp"

namespace halo {

// Stereo view callbacks. `pre` = the engine camera before UEVR's HMD transform (the body eye);
// `post` = the eye UEVR composed. apply_post returns true when it moved the eye.
void headblock_note_pre(int index, double x, double y, double z);
bool headblock_apply_post(int index, double* x, double* y, double* z);

// GAME THREAD. `active` = on-foot gameplay; `ignore` = actors the trace must not hit.
void headblock_tick(bool active, uevr::API::UObject* const* ignore, int n_ignore, float dt);

}  // namespace halo
