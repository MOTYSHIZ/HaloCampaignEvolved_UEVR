#pragma once

// PLUGIN.CPP'S FILE-LOCAL STATE THAT FEATURES READ.
//
// Plugin.cpp keeps its state in an anonymous namespace, which no other translation unit can name.
// HALO_PLUGIN_STATE_BRIDGE, expanded once in Plugin.cpp right after that namespace closes, defines
// g_plugin_state as the addresses of the originals, so every read and write lands on exactly the
// object the code used when it was a textual fragment of Plugin.cpp. A feature binds a reference to
// each object it uses under the object's own name, so the code reads the same as it did in place.
//
// Pointers, not reference members: MSVC 19.29 refuses constinit on an aggregate of references, and
// constinit is what guarantees the bridge is initialised statically -- valid from any thread from the
// moment the DLL is loaded, with no initialisation order against any other translation unit.

#include <atomic>

namespace halo::host {

struct PluginState {
    std::atomic<bool>*  in_menu;            // g_in_menu
    std::atomic<bool>*  cut2d_engaged;      // g_cut2d_engaged
    std::atomic<float>* last_dt;            // g_last_dt
    std::atomic<bool>*  stick_mode;         // g_stick_mode
    std::atomic<bool>*  dpad_shift_active;  // g_dpad_shift_active
    short (*to_raw)(float v);               // to_raw()
};

extern const PluginState g_plugin_state;

} // namespace halo::host

#define HALO_PLUGIN_STATE_BRIDGE                                          \
    constinit const ::halo::host::PluginState halo::host::g_plugin_state{ \
        &g_in_menu,                                                        \
        &g_cut2d_engaged,                                                  \
        &g_last_dt,                                                        \
        &g_stick_mode,                                                     \
        &g_dpad_shift_active,                                              \
        &to_raw,                                                           \
    };
