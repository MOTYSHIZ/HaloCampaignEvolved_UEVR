#pragma once

// HOOK POINTS IN Markers.cpp. Definitions: src/features/FeatureList.cpp.

#include "Math.hpp"
#include "core/host/MarkersState.hpp"   // HALO_MARKERS_STATE_BRIDGE

namespace halo {

// room_to_world (its caller's thread: the game tick for the holster markers, the render thread for the
// marker re-anchor pass), as its first statement. True = the room anchor is the standing origin
// (reloadroomanchor 1, core/MarkerFaces) and *out holds the point; false = the author's HMD-anchored
// transform runs as written.
bool features_room_to_world(const Vec3& room, const Vec3& hmd_room, Vec3* out);

} // namespace halo
