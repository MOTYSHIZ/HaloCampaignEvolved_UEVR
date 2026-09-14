#include "features/palettewpn/PaletteWpn.hpp"

#include "features/palettewpn/PaletteArmDriver.hpp"   // palette_weapon_mode(), the arbiter and arm hide slots
#include "Config.hpp"
#include "PaletteTwoHand.hpp"   // palette_two_hand_reset()
#include "features/palettewpn/PaletteReadbacks.hpp"
#include "core/Services.hpp"

namespace halo {

namespace {
bool palette_wpn_enabled() { return g_cfg.palette_weapon || g_cfg.arm_driver == 3; }
// The palette weapon's own hold (armdriver mode 3); a no-op when idle.
void palette_wpn_gesture_reset() { palette_two_hand_reset(); }
// The FP weapon-actor route and the rig component, while mode 3 owns.
bool palette_wpn_rig_resolve_wanted() { return palette_weapon_mode(); }
}  // namespace

constinit const FeatureHooks kPaletteWpnHooks{
    .key      = "palettewpn",
    .gesture_reset      = &palette_wpn_gesture_reset,
    .rig_resolve_wanted = &palette_wpn_rig_resolve_wanted,
    .sim_record_written = &palette_wpn_sim_record_written,
    .arm_driver_name             = &palette_wpn_arm_driver_name,
    .arm_driver_mode_wanted      = &palette_wpn_arm_driver_mode_wanted,
    .arm_driver_mode_unavailable = &palette_wpn_arm_driver_mode_unavailable,
    .arm_driver_key_changed      = &palette_wpn_arm_driver_key_changed,
    .arm_driver_steady           = &palette_wpn_arm_driver_steady,
    .arm_driver_active           = &palette_wpn_arm_driver_active,
    .arm_driver_release_all      = &palette_wpn_arm_driver_release_all,
    .arm_hide_component          = &palette_wpn_arm_hide_component,
    .arm_hide_held_off           = &palette_wpn_arm_hide_held_off,
    .arm_hide_needs_rig          = &palette_wpn_arm_hide_needs_rig,
    .enabled  = &palette_wpn_enabled,
    .services = SVC_MARKER_ANCHOR | SVC_CAMERA_BOB | SVC_WEAPON_OBJECT,
    .released = &palette_wpn_arm_hide_released,
};

} // namespace halo
