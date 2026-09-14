#pragma once

// BLAMAIM.CPP'S FILE-LOCAL HOOK STATE THAT FEATURES USE (dev builds: BlamAim.cpp compiles only with
// HALO_VR_DEV). Same doctrine as core/host/PluginState.hpp: HALO_BLAMAIM_STATE_BRIDGE, expanded once
// in BlamAim.cpp after its anonymous namespace, defines the addresses of the originals,
// constant-initialised.

#include <cstdint>

namespace halo::host {

struct BlamAimState {
    uintptr_t p_vec1;                                   // P_VEC1: the spawn origin's offset in the params
    int* hook_id;                                       // g_hook_id: blamaim's getter hook
    uintptr_t (**orig_create)(uintptr_t params);        // g_orig_create: the create_projectile trampoline
    int* create_hook_id;                                // g_create_hook_id
    uintptr_t* sim_base;                                // g_sim_base
    uintptr_t rva_create_projectile;                    // RVA_CREATE_PROJECTILE
    const uint8_t (*create_projectile_prologue)[16];    // CREATE_PROJECTILE_PROLOGUE
    uintptr_t (*hooked_create_projectile)(uintptr_t params);   // hooked_create_projectile()
};

extern const BlamAimState g_blamaim_state;

} // namespace halo::host

// Expanded inside namespace halo, so the declarator is written halo::host:: (a leading :: would be
// parsed as a continuation of the type's own name).
#define HALO_BLAMAIM_STATE_BRIDGE                                              \
    constinit const ::halo::host::BlamAimState halo::host::g_blamaim_state{   \
        P_VEC1,                                                                 \
        &g_hook_id,                                                             \
        &g_orig_create,                                                         \
        &g_create_hook_id,                                                      \
        &g_sim_base,                                                            \
        RVA_CREATE_PROJECTILE,                                                  \
        &CREATE_PROJECTILE_PROLOGUE,                                            \
        &hooked_create_projectile,                                              \
    };
