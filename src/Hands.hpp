// Our own hands, and the reload magazine.
//
// WHY THIS EXISTS. With the game's first-person meshes hidden (see Arms.hpp), there is nothing on
// screen but a floating weapon. These are the replacements: spawned StaticMeshComponents attached
// to the motion controllers, so what you see tracks what you are actually holding.
//
// WHY IT IS SPAWNED RATHER THAN BORROWED. The bone dump settled it -- SetBoneTransformByName is
// ABSENT on this build, so the game's own hand bones cannot be posed. Both arms live in one
// skeletal mesh driven by one anim blueprint, and Rig.cpp can only move that mesh as a unit. There
// is no way to get two independently tracked hands out of the game's rig; they have to be ours.
//
// HOW THEY TRACK. Not by writing transforms per tick -- via UObjectHook's motion controller state,
// the same mechanism Rig.cpp already uses in attach_apply(). UEVR then owns the VR-space to
// world-space conversion, which is the part that would otherwise need the view transform, the HMD
// pose and a metres-to-centimetres scale all agreeing with each other every frame.
//
// THE MAGAZINE is the same machinery with different rules: right hand becomes hand 0/1 by config,
// the mag is hand-attached only while the reload state machine says it is held, and it is hidden
// the rest of the time rather than destroyed and respawned.

#pragma once

// API.hpp, NOT Plugin.hpp -- see UeObject.hpp.
#include "uevr/API.hpp"

namespace halo {

// Game thread, once per tick. Creates what is missing, attaches what is unattached, and shows or
// hides the magazine according to the reload state.
void hands_update();

// Destroy nothing, but detach and hide everything we own. Called when the feature is switched off
// and on any transition that invalidates our handles.
void hands_release();

} // namespace halo
