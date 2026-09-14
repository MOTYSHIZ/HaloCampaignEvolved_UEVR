#pragma once

// HOOK POINTS IN Arms.cpp. Definitions: src/features/FeatureList.cpp. Game thread.

#include "core/host/ArmsState.hpp"   // HALO_ARMS_STATE_BRIDGE

#include "uevr/API.hpp"

namespace halo {

// sweep_fp_meshes, per swept component, before the author's mode switch: true = a feature hid or unhid it.
bool features_arm_hide_component(uevr::API::UObject* comp, bool hide, int mode);
// arms_hide_update, before the rig gate: true = a feature holds the sweep off this tick.
bool features_arm_hide_held_off();
// arms_hide_update, the rig gate, beside armhidemode 0: true = the current mode needs the rig.
bool features_arm_hide_needs_rig();

} // namespace halo
