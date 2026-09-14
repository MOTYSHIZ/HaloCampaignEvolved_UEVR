#include "features/palettewpn/PaletteWpn.hpp"

#include "ArmDriver.hpp"        // palette_weapon_mode()
#include "Config.hpp"
#include "PaletteTwoHand.hpp"   // palette_two_hand_reset()
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
    .enabled  = &palette_wpn_enabled,
    .services = SVC_MARKER_ANCHOR | SVC_CAMERA_BOB | SVC_WEAPON_OBJECT,
};

} // namespace halo
