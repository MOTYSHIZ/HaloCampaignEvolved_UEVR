#pragma once

// MARKERS.CPP'S FILE-LOCAL HELPERS THAT CORE AND FEATURES CALL. Same doctrine as
// core/host/PluginState.hpp: HALO_MARKERS_STATE_BRIDGE, expanded once in Markers.cpp after its
// anonymous namespace, defines the addresses of the originals, constant-initialised.

#include "Math.hpp"
#include "uevr/API.hpp"

namespace halo::host {

struct MarkersState {
    uevr::API::UObject* (*spawn_marker)(uevr::API::UObject* owner, const wchar_t* const* meshes, int nmesh,
                                        double sx, double sy, double sz);
    void (*marker_place)(uevr::API::UObject* comp, const Vec3& world, float pitch, float yaw);
    void (*marker_scale)(uevr::API::UObject* comp, double sx, double sy, double sz);
    bool (*hull_local_to_world)(uevr::API::UObject* hull, const Vec3& local, Vec3* out, float* hull_yaw);
    bool (*world_to_hull_local)(uevr::API::UObject* hull, const Vec3& world, Vec3* out);
};

extern const MarkersState g_markers_state;

} // namespace halo::host

// Expanded inside namespace halo, so the declarator is written halo::host:: (a leading :: would be
// parsed as a continuation of the type's own name).
#define HALO_MARKERS_STATE_BRIDGE                                              \
    constinit const ::halo::host::MarkersState halo::host::g_markers_state{   \
        &spawn_marker,                                                          \
        &marker_place,                                                          \
        &marker_scale,                                                          \
        &hull_local_to_world,                                                   \
        &world_to_hull_local,                                                   \
    };
