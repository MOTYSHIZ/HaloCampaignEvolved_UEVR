#pragma once

// GESTURE.CPP'S FILE-LOCAL STATE THAT CORE AND FEATURES READ. Same doctrine as
// core/host/PluginState.hpp: HALO_GESTURE_STATE_BRIDGE, expanded once in Gesture.cpp after its first
// anonymous namespace, defines the addresses of the originals, constant-initialised.

#include "Gesture.hpp"   // ReloadState
#include "Math.hpp"      // Vec3

namespace halo::host {

struct GestureState {
    long long*   cooldown_until;          // s_cooldown_until: the melee cooldown both hands share
    ReloadState* reload;                  // s_reload: the reload machine's state
    float        rest_speed_mps;          // REST_SPEED_MPS: below this the hand counts as at rest
    long long (*now_ticks)();             // now_ticks(): the steady clock in its own ticks
    long long (*ms_to_ticks)(int ms);     // ms_to_ticks()
    Vec3*        vel;                     // s_vel: the aim hand's smoothed hand-minus-head velocity
    float*       ext;                     // s_ext: the aim hand's smoothed extension rate
};

extern const GestureState g_gesture_state;

} // namespace halo::host

// Expanded inside namespace halo, so the declarator is written halo::host:: (a leading :: would be
// parsed as a continuation of the type's own name).
#define HALO_GESTURE_STATE_BRIDGE                                              \
    constinit const ::halo::host::GestureState halo::host::g_gesture_state{   \
        &s_cooldown_until,                                                      \
        &s_reload,                                                              \
        REST_SPEED_MPS,                                                         \
        &now_ticks,                                                             \
        &ms_to_ticks,                                                           \
        &s_vel,                                                                 \
        &s_ext,                                                                 \
    };
