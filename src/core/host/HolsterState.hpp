#pragma once

// HOLSTER.CPP'S FILE-LOCAL STATE THAT FEATURES READ. Same doctrine as core/host/PluginState.hpp:
// HALO_HOLSTER_STATE_BRIDGE, expanded once in Holster.cpp after its anonymous namespaces, defines the
// addresses of the originals, constant-initialised.

#include "Math.hpp"       // Vec3
#include "UeObject.hpp"

namespace halo::host {

struct HolsterState {
    TrackedObject* mesh_frag;     // s_mesh_frag: the game's own frag grenade mesh, resolved for the pouch markers
    TrackedObject* mesh_plasma;   // s_mesh_plasma: the plasma grenade mesh
    long long* last_action;       // s_last_action: any holster action (melee veto window)
    long long (*now_ticks)();     // now_ticks(): the steady clock in its own ticks
    long long (*ms_to_ticks)(int ms);   // ms_to_ticks()
    bool*      grenade_armed;     // s_grenade_armed: a grenade is in hand
    bool*      carry_off;         // s_carry_off: which hand holds the armed grenade
    bool*      unarmed;           // s_unarmed: hand empty (hide + swallow fire)
    int*       unhide_ticks;      // s_unhide_ticks: re-assert the weapon's visibility briefly after a draw
    Vec3*      peak_velw;         // s_peak_velw: the aim hand's world velocity at its forward peak
    Vec3*      gpeak_velw;        // s_gpeak_velw: the off hand's
    bool (*aim_is_right)();       // aim_is_right()
    bool (*off_is_right)();       // off_is_right()
    void (*haptic_on)(bool right, float dur, float amp);   // haptic_on()
    void (*set_weapon_hidden)(bool hidden);                // set_weapon_hidden()
};

extern const HolsterState g_holster_state;

} // namespace halo::host

// Expanded inside namespace halo, so the declarator is written halo::host:: (a leading :: would be
// parsed as a continuation of the type's own name).
#define HALO_HOLSTER_STATE_BRIDGE                                              \
    constinit const ::halo::host::HolsterState halo::host::g_holster_state{   \
        &s_mesh_frag,                                                           \
        &s_mesh_plasma,                                                         \
        &s_last_action,                                                         \
        &now_ticks,                                                             \
        &ms_to_ticks,                                                           \
        &s_grenade_armed,                                                       \
        &s_carry_off,                                                           \
        &s_unarmed,                                                             \
        &s_unhide_ticks,                                                        \
        &s_peak_velw,                                                           \
        &s_gpeak_velw,                                                          \
        &aim_is_right,                                                          \
        &off_is_right,                                                          \
        &haptic_on,                                                             \
        &set_weapon_hidden,                                                     \
    };
