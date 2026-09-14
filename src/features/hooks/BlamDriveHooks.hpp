#pragma once

// HOOK POINTS IN BlamDrive.cpp. Definitions: src/features/FeatureList.cpp.

#include "core/host/BlamDriveState.hpp"   // HALO_BLAMDRIVE_STATE_BRIDGE

#include <cstdint>

namespace halo {

// drive_angles_impl (the sim thread from the orientation getter hook, or the game thread's off-thread
// write), in the stick-mode hold, as the statement the hold returns. The unit publish that must not
// stop while the aim write holds off (core/UnitState).
void features_sim_stick_mode_hold(bool off_thread);

// drive_angles_impl, once the control record is resolved and writable, before the record is read or
// written. rec points at the record's yaw field. The unit state publish (core/UnitState).
void features_sim_record_ready(uintptr_t rec, bool off_thread);

// The orientation getter hook, right after the original getter returns and before the aim write. SIM
// THREAD. The weapon object resolve off the builder hook (core/WeaponObject), rate limited there.
void features_sim_orientation_returned();

} // namespace halo
