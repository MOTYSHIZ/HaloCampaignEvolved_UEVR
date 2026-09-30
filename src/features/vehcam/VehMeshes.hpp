#pragma once

// THE VEHICLE'S OWN PARTS, HIDDEN PER SEAT ("hideMeshes" in halo_vr_vehcams.json) -- and, in a dev build,
// LISTED (vehmeshdump).
//
// WHY. Some cameras sit INSIDE the vehicle and see out through the hull's back faces, which the engine does
// not draw. A damaged vehicle shows its insides, and from in there they come up right across that view (the
// Banshee's, reported 2026-09-29). Measured on the Banshee the same day: damage SWAPS a piece's mesh from its
// _Default to its _Damage asset, on the same component, and splits the canopy and the wings into more pieces
// (docs: VEHICLE_MESHES.md in the private tree) -- so a part is judged again whenever its mesh changes, not
// only when it first appears. A seat, or one of its cameras, names the parts to hide; they are hidden while
// that camera is up and put back when it is not.
//
// HOW. By SCALE, as the Chief's helmet is hidden (VehCam.cpp, head_item_hide): a part shrunk to 0.001 draws
// nothing whatever its visibility flags say, so the flags stay the game's -- a damage state that reveals a
// part changes only them -- and the restore owns the scale, and only while it is still ours. The game never
// touched the helmet's scale; if it touches a vehicle part's, the hold puts it back within a quarter second
// and the log says so once. A part that
// CARRIES others (anything hangs from it) is hidden by its own visibility flags instead, never propagated,
// so whatever hangs from it keeps its size: the Chief rides a seat, and a shrunk seat would shrink him.
// The cameras' frame (the chassis) is never shrunk for the same reason: every seat and socket sits on it.
// Only MESHES belonging to the vehicle's own actors are ever touched -- never the Chief riding it, nor
// anything else merely hanging on it.

#include <cstdint>

#include "uevr/API.hpp"

namespace halo {

// GAME thread, every vehicle tick, after the camera selection (VehCam.cpp). riding = the vehicle cameras run
// in a vehicle (their ride, the vehtp setting and the kill switch all say yes); seat_actor = the actor your
// seat belongs to, as the game names it (nullptr = not yet); chassis / chassis_idx = the mesh the cameras use
// as the vehicle's frame and its object-array slot (0 = not resolved). Hides what the selected camera's
// "hideMeshes" names, finds parts that appear later (a damage state) within a quarter second, holds them
// hidden, and puts everything back the moment the list, the vehicle or the ride changes. Runs vehmeshdump in
// a dev build.
void vehmesh_tick(bool riding, uevr::API::UObject* seat_actor, uintptr_t chassis, int32_t chassis_idx);

} // namespace halo
