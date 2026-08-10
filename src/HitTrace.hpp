// ============================================================================================
// LINE TRACE -- put the reticule on the surface you are about to hit, not at a guessed distance.
//
// WHY THIS EXISTS
//   The reticule is a WORLD-SPACE object. A world marker at a FIXED distance along the aim ray only
//   lines up with the impact point when viewed from the origin the shot leaves. In VR your eye is
//   not there and does not stay put: UEVR moves the rendered eye with your physical head while the
//   game camera stays where the simulation put it. Step 30 cm and a marker at 5 m appears to shift
//   by atan(0.30 / 5) ~ 3.4 degrees against a shot that has not moved -- measured in-headset, and
//   it clears exactly on a play-area reset, which is the tell.
//
//   No distance is right, which is why the shipped profile carries two hand-tuned ones (5 m on
//   foot, 22 m seated) and both are still wrong at any other range. Placing the marker on the
//   TRACED HIT removes the free parameter: the marker is on the surface, so it reads correctly
//   from any eye position and at any range.
//
// WHY EVERY OFFSET IS RESOLVED AT RUNTIME
//   This calls UKismetSystemLibrary::LineTraceSingle through reflection, which means building its
//   parameter block by hand. FHitResult is large and version-sensitive -- the note on Rig.cpp's
//   marshalling says as much -- and here we must both WRITE arguments at several offsets and READ
//   the impact point back out. Hard-coding that layout is a crash in someone's headset the first
//   time the engine version moves.
//
//   So nothing is assumed: the parameter offsets come from the UFunction's own child properties and
//   the impact-point offset from the HitResult script struct, resolved once and logged. Any
//   resolution failure disables the feature and says which field was missing, rather than leaving a
//   plausible-looking pointer to be trusted on faith.
// ============================================================================================

#pragma once

#include "Math.hpp"
#include "uevr/API.hpp"

namespace halo {

// Resolve the function and every offset. Safe to call repeatedly -- it latches, and a failure is
// sticky so a broken resolve does not retry on the hot path. Returns true when tracing is usable.
bool hit_trace_ready();

// Trace from `start` towards `end` (world cm). Returns true and fills `out_hit` on a blocking hit;
// false on a miss, or whenever the feature could not be resolved -- callers fall back to their
// fixed distance, which is the previous behaviour exactly.
//
// `ignore` is an array of ACTOR pointers the trace must not hit, `ignore_count` its length; pass
// nullptr/0 for none. This is not optional in practice: the player's own weapon is a separate actor
// attached to the rig, so a trace from the eye hits it the moment an animation swings it across the
// camera, and the reticule lands on the gun instead of on the world.
bool hit_trace(const Vec3& start, const Vec3& end,
               uevr::API::UObject* const* ignore, int ignore_count, Vec3* out_hit);

#if HALO_VR_DEV
// Log the project's named collision channels and the aimreticuletracechannel index for each.
//
// ⚠️ None of them is "the projectile channel". Halo's projectiles collide against the BLAM SIM'S OWN
// collision BSP -- `HaloSimulation_tag_release.dll` contains global_collision_bsp_struct,
// collision_model and leaf/surface traversal, and references NO UE collision symbols at all
// (no ECC_, no TraceTypeQuery, no LineTraceSingle). UE collision is a parallel representation of
// roughly the same world, so the reticule trace can only ever approximate where a round goes. This
// dump exists to pick the closest approximation deliberately rather than by trying indices.
void hit_trace_dump_channels();
#endif

} // namespace halo
