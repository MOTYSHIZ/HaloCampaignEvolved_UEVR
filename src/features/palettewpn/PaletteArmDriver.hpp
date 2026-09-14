#pragma once

// THE PALETTE WEAPON AS AN ARM DRIVER (palettewpn). The author's arbiter (ArmDriver.cpp) knows modes 0-2;
// this feature adds mode 3 through the arbiter's hooks (features/hooks/ArmDriverHooks.hpp), so it switches
// through the same release-then-install path and never behind the arbiter's back.
//
// EXPERIMENTAL (fork port, default off). BlamPalette.cpp places the WEAPON on the Blam node
// palette and the aim follows the drawn barrel (pose latch, DRAWAIM, stamped reticle, aimbore,
// the fork's two-hand hold). Hooks the same builder as Palette, so the two are exclusive by
// construction: only the owning mode may have its builder hook installed. Selected by
// armdriver=3, or by the fork's palettewpn=1 (an alias the arbiter reads, so it switches through
// the same release-then-install path and never behind the arbiter's back).

#include "ArmDriver.hpp"
#include "uevr/API.hpp"

namespace halo {

constexpr ArmDriverMode kPaletteWeaponMode = static_cast<ArmDriverMode>(3);

// True while PaletteWeapon owns the arms and the aim. ANY THREAD (an atomic mirror of the arbiter's
// decision): the sim-thread palette hook and the XInput aim derivation both ask it.
bool palette_weapon_mode();

// The arbiter's hooks (the palettewpn table's slots).
const char* palette_wpn_arm_driver_name(int mode);
int  palette_wpn_arm_driver_mode_wanted();
bool palette_wpn_arm_driver_mode_unavailable(int wanted);
bool palette_wpn_arm_driver_key_changed();
void palette_wpn_arm_driver_steady(int active);
void palette_wpn_arm_driver_active(int mode, bool switched);
void palette_wpn_arm_driver_release_all(const char* why);

// Arms.cpp's hooks (the arm hide) and the release on the feature's off edge.
bool palette_wpn_arm_hide_component(uevr::API::UObject* comp, bool hide, int mode, bool enabled);
bool palette_wpn_arm_hide_held_off();
bool palette_wpn_arm_hide_needs_rig();
void palette_wpn_arm_hide_released();

} // namespace halo
