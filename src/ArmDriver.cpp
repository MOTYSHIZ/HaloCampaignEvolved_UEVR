#include "ArmDriver.hpp"

#include "Arms.hpp"
#include "Config.hpp"
#include "Hands.hpp"
#include "palettearm/PaletteArm.hpp"

#include "uevr/API.hpp"

using uevr::API;

namespace halo {
namespace {

// Last mode we actually ran, so a change can be detected and the loser released. Starts at Off
// because that is the truth before the first tick: nothing is driving anything yet.
ArmDriverMode s_active = ArmDriverMode::Off;
bool          s_announced = false;

} // namespace

const char* arm_driver_name(ArmDriverMode mode) {
    switch (mode) {
        case ArmDriverMode::UeRig:   return "UeRig (Rig/Arms/Hands)";
        case ArmDriverMode::Palette: return "Palette (palettearm)";
        case ArmDriverMode::Off:
        default:                     return "Off";
    }
}

ArmDriverMode arm_driver_mode() {
    // The kill switch outranks the selector: g_cfg.enabled false means the plugin is standing
    // down entirely, and an arm driver is exactly the kind of thing that must stop when it does.
    if (!g_cfg.enabled) return ArmDriverMode::Off;
    switch (g_cfg.arm_driver) {
        case 1:  return ArmDriverMode::UeRig;
        case 2:  return ArmDriverMode::Palette;
        default: return ArmDriverMode::Off;
    }
}

bool arm_driver_owns(ArmDriverMode mode) {
    // Deliberately reads s_active, NOT the config. Between the config changing and arbitrate()
    // running, the outgoing driver is still the one holding the handles -- answering from the
    // config here would let the incoming driver take a frame before the outgoing one released,
    // which is precisely the overlap this file exists to prevent.
    return s_active == mode && mode != ArmDriverMode::Off;
}

void arm_driver_release_all(const char* why) {
    // Order matters: hands are attached to things arms_release_hide() may invalidate.
    hands_release();
    arms_release_hide();
    palettearm_release();
    if (why != nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] ARMDRIVER: released all (%s)", why);
    }
}

void arm_driver_arbitrate() {
    const ArmDriverMode wanted = arm_driver_mode();

    if (!s_announced) {
        API::get()->log_info("[Halo-CampE-UEVR] ARMDRIVER: %s", arm_driver_name(wanted));
        s_announced = true;
        s_active = wanted;
        return;
    }

    if (wanted == s_active) return;

    // Release EVERYTHING, not just the outgoing driver. Releasing only the loser assumes the
    // winner holds nothing stale, which is false after a level load or a failed frame -- and the
    // cost of an extra release on a mode change, which happens by hand a few times a session, is
    // nothing.
    API::get()->log_info("[Halo-CampE-UEVR] ARMDRIVER: %s -> %s",
                         arm_driver_name(s_active), arm_driver_name(wanted));
    arm_driver_release_all(nullptr);

    // Only now does the incoming driver start answering true.
    s_active = wanted;
}

} // namespace halo
