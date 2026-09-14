#pragma once

// HOOK POINTS IN ArmDriver.cpp. Definitions: src/features/FeatureList.cpp. Game thread (the arbiter).

namespace halo {

// arm_driver_name, first: a feature mode's name, or null for the author's modes.
const char* features_arm_driver_name(int mode);
// arm_driver_mode, after the kill switch: a feature mode that is asked for (> 0), or 0.
int  features_arm_driver_mode_wanted();
// effective_mode, after the palettearm fallback: true = the wanted feature mode is unavailable (UeRig).
bool features_arm_driver_mode_unavailable(int wanted);
// arm_driver_arbitrate, the retry key: true = a feature's own key changed (every feature is asked).
bool features_arm_driver_key_changed();
// arm_driver_arbitrate, the steady state (the mode did not change), with the active mode.
void features_arm_driver_steady(int active);
// arm_driver_arbitrate, right after s_active is assigned: at the first announce (switched false) and after a
// switch's release (switched true).
void features_arm_driver_active(int mode, bool switched);
// arm_driver_release_all, after the palettearm release, with its reason (null from the arbiter's switch).
void features_arm_driver_release_all(const char* why);

} // namespace halo
