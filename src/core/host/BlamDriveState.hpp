#pragma once

// BLAMDRIVE.CPP'S FILE-LOCAL STATE THAT CORE AND FEATURES READ. Same doctrine as
// core/host/PluginState.hpp: HALO_BLAMDRIVE_STATE_BRIDGE, expanded once in BlamDrive.cpp after its
// anonymous namespace, defines the addresses of the originals, constant-initialised.

#include <atomic>
#include <cstdint>

namespace halo::host {

struct BlamDriveState {
    bool (*read_ptr)(uintptr_t p, uintptr_t* out);   // read_ptr(): a guarded pointer read
    uint32_t* tls_index;                              // g_tls_index: the sim module's _tls_index
    std::atomic<uintptr_t>* ctl_rec;                  // g_ctl_rec: the resolved control record (at its yaw field)
    uintptr_t (*resolve_control_record)(const char** why, int* out_index);   // resolve_control_record()
    uintptr_t off_ctl_yaw;                            // OFF_CTL_YAW: the yaw field's offset in the record
};

extern const BlamDriveState g_blamdrive_state;

} // namespace halo::host

// Expanded inside namespace halo, so the declarator is written halo::host:: (a leading :: would be
// parsed as a continuation of the type's own name).
#define HALO_BLAMDRIVE_STATE_BRIDGE                                                \
    constinit const ::halo::host::BlamDriveState halo::host::g_blamdrive_state{   \
        &read_ptr,                                                                  \
        &g_tls_index,                                                               \
        &g_ctl_rec,                                                                 \
        &resolve_control_record,                                                    \
        OFF_CTL_YAW,                                                                \
    };
