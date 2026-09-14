#pragma once

// CORE SERVICES AND THEIR GATES.
//
// A core service is shared machinery more than one feature consumes. It runs while ANY feature that
// declares it (FeatureHooks::services) is enabled, and never because of one particular feature: no core
// code names a feature or reads a feature's master key. With every feature off, every service is
// inactive and the author's code runs as he shipped it.
//
// service_active() walks the feature list (src/features/FeatureList.cpp) and asks each table whose
// services include the bit whether its feature is enabled, so a build with feature folders removed
// simply has fewer consumers. Any thread: the enabled predicates read g_cfg master fields, the same
// unsynchronised reads the features themselves make.

#include <cstdint>
#include <string>

namespace halo {

enum Service : uint32_t {
    SVC_UNIT_STATE        = 1u << 0,   // core/UnitState: the sim-thread unit publish (mounted, position, facing)
    SVC_SEAT              = 1u << 1,   // core/UnitState: the vehicle half and the stick-mode seat publish
    SVC_FIRE_INPUT        = 1u << 2,   // core/FireInput: when the player last pulled the trigger
    SVC_EYE_TRACE         = 1u << 3,   // core/EyeTrace: the body eye and head offset measurement
    SVC_LEASH_GATE        = 1u << 4,   // core/fixes/HmdPoseGate: the implausible-pose gate in the leash block
    SVC_MARKER_ANCHOR     = 1u << 5,   // core/MarkerFaces: render-rate re-anchor, room anchor, marker tint
    SVC_MELEE_INSTRUMENTS = 1u << 6,   // core/fixes/MeleeInstruments: veto, aim pin, hold check, FIRED line
    SVC_CAMERA_BOB        = 1u << 7,   // core/CameraBob: the measured camera bob
    SVC_HIDDEN_RELOAD     = 1u << 8,   // core/HiddenReload: the hidden-reload ammo cradle flag
    SVC_RETICULE_FIXES    = 1u << 9,   // core/fixes/ReticuleFixes: asset-load memo, widget re-assert
    SVC_WIDGET_HOSTS      = 1u << 10,  // core/fixes/ReticuleFixes: the alpha hide limited to the reticule widget
    SVC_HOST_FIXES        = 1u << 11,  // core/fixes: robustness fixes to the author's code (rig guard, nav lane, ...)
};

constexpr int kServiceCount = 12;

// True while at least one enabled feature declares the service.
bool service_active(uint32_t service);

// "unitstate" etc., for the resolve log.
const char* service_name(uint32_t service);

// The enabled features that turn the service on, comma separated ("" = inactive).
std::string service_enablers(uint32_t service);

} // namespace halo
