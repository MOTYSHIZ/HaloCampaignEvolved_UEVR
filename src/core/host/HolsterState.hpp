#pragma once

// HOLSTER.CPP'S FILE-LOCAL STATE THAT FEATURES READ. Same doctrine as core/host/PluginState.hpp:
// HALO_HOLSTER_STATE_BRIDGE, expanded once in Holster.cpp after its anonymous namespaces, defines the
// addresses of the originals, constant-initialised.

#include "UeObject.hpp"

namespace halo::host {

struct HolsterState {
    TrackedObject* mesh_frag;     // s_mesh_frag: the game's own frag grenade mesh, resolved for the pouch markers
    TrackedObject* mesh_plasma;   // s_mesh_plasma: the plasma grenade mesh
};

extern const HolsterState g_holster_state;

} // namespace halo::host

// Expanded inside namespace halo, so the declarator is written halo::host:: (a leading :: would be
// parsed as a continuation of the type's own name).
#define HALO_HOLSTER_STATE_BRIDGE                                              \
    constinit const ::halo::host::HolsterState halo::host::g_holster_state{   \
        &s_mesh_frag,                                                           \
        &s_mesh_plasma,                                                         \
    };
