// Two independent arms -- the hybrid route.
//
// WHAT THIS IS TRYING TO DO. Today the whole first-person rig is ONE skeletal mesh
// (BPC_FP_SkeletalMesh_C) carrying both arms, with the weapon socket-attached to it as a separate
// actor. Rig.cpp drives that mesh at the COMPONENT level, deliberately: a scene component's
// relative transform sits a level above the pose, so the anim node never fights it. That seam is
// why the rig works -- and it is exactly why one arm cannot move without the other.
//
// The hybrid plan keeps everything that already works and frees only the left hand:
//   * right arm + weapon  -- untouched, still driven by Rig.cpp as today
//   * left arm chain      -- hidden at the bone level (HideBoneByName)
//   * left hand           -- our own component, attached to the left controller
//
// WHY RECON FIRST. Two facts decide whether that plan is even buildable, and neither can be
// established by reading source:
//   1. The exact bone names and where the left chain branches. log_pivot_candidates() proves
//      Wrist_L and Wrist_R exist, so it IS a two-sided skeleton -- but a chain needs its root.
//   2. Whether the bone functions are actually CALLABLE on this build. Reachable-in-theory and
//      callable-here are different things: Rig.cpp already documents that the relative rotation
//      write silently does not take on this very mesh, which is the whole reason rigmode 3 exists.
//
// So this module starts as an instrument, not a feature. Nothing here writes to the game.

#pragma once

// API.hpp, NOT Plugin.hpp -- see UeObject.hpp.
#include "uevr/API.hpp"

namespace halo {

// Game thread, once per tick. Fires the skeleton dump on the RISING EDGE of bonedump, and applies
// or releases the left-arm hide when arm_hide changes.
void arms_update();

// Enumerate every bone on the rig with its parent, and report which bone functions this build
// actually exposes. Read-only: no HideBoneByName call is made, nothing is written.
void arms_dump_skeleton(uevr::API::UObject* rig);

// Release the arm hide immediately, on every mesh we hid it on.
//
// Called when arm_hide goes false, and on any transition that invalidates our handles. IsBoneHidden
// is ABSENT on this build, so we cannot ask the engine what is hidden -- we can only remember what
// we hid. That makes releasing on the way out non-optional: a forgotten hide is an arm that stays
// invisible with nothing left that knows how to bring it back.
void arms_release_hide();

} // namespace halo
