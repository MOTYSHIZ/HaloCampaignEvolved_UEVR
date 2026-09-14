#pragma once

// ARMS.CPP'S FILE-LOCAL HELPERS THAT FEATURES CALL. Same doctrine as core/host/PluginState.hpp:
// HALO_ARMS_STATE_BRIDGE, expanded once in Arms.cpp after its anonymous namespace, defines the
// addresses of the originals, constant-initialised.

#include "uevr/API.hpp"

namespace halo::host {

struct ArmsState {
    void (*call_set_visibility)(uevr::API::UObject* comp, bool visible);   // call_set_visibility()
    void (*call_set_hidden)(uevr::API::UObject* comp, bool hidden);        // call_set_hidden()
    void (*call_hide_bone)(uevr::API::UObject* comp, const wchar_t* bone);    // call_hide_bone()
    void (*call_unhide_bone)(uevr::API::UObject* comp, const wchar_t* bone);  // call_unhide_bone()
    bool* any_hidden;                                                     // s_any_hidden: a hide is applied
};

extern const ArmsState g_arms_state;

} // namespace halo::host

namespace halo {
// Arms.cpp's arm hide pass (the author's function, external linkage, not in his header). The palette
// weapon's own tick calls it while mode 3 owns placement.
void arms_hide_update();
} // namespace halo

// Expanded inside namespace halo, so the declarator is written halo::host:: (a leading :: would be
// parsed as a continuation of the type's own name).
#define HALO_ARMS_STATE_BRIDGE                                        \
    constinit const ::halo::host::ArmsState halo::host::g_arms_state{ \
        &call_set_visibility,                                         \
        &call_set_hidden,                                             \
        &call_hide_bone,                                              \
        &call_unhide_bone,                                            \
        &s_any_hidden,                                                \
    };
