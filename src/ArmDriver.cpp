#include "ArmDriver.hpp"

#include "Arms.hpp"
#include "Config.hpp"
#include "Hands.hpp"
#include "palettearm/PaletteArm.hpp"
#include "palettearm/PaletteHook.hpp"   // palettehook_installed(): the switch line names both builder hooks
#include "BlamPalette.hpp"              // PaletteWeapon: the fork's builder hook, its release and fallback
#include "PaletteTwoHand.hpp"                 // palette_two_hand_reset: the fork's two-hand hold
#include "TwoHandAim.hpp"              // two_hand_reset: drop the author's hold when the aim owner changes

#include "uevr/API.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>

using uevr::API;

namespace halo {
namespace {

// Last mode we actually ran, so a change can be detected and the loser released. Starts at Off
// because that is the truth before the first tick: nothing is driving anything yet.
ArmDriverMode s_active = ArmDriverMode::Off;
bool          s_announced = false;

// The raw config value we last saw, so a CHANGE can be detected. A change is the signal to
// re-arm a route that gave up: writing armdriver again (2 -> 1 -> 2) retries without a game
// restart. Initialised out of range so the first tick always counts as a change.
int           s_last_cfg = -1;

// ANY-THREAD MIRROR of s_active (PaletteWeapon is read by the sim-thread palette hook and by the XInput
// aim derivation, neither of which may read the tick-thread value directly).
std::atomic<int> s_active_mirror{0};

double now_ms_d() {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

} // namespace

const char* arm_driver_name(ArmDriverMode mode) {
    switch (mode) {
        case ArmDriverMode::UeRig:   return "UeRig (Rig/Arms/Hands)";
        case ArmDriverMode::Palette: return "Palette (palettearm)";
        case ArmDriverMode::PaletteWeapon: return "PaletteWeapon (palette weapon + aim, experimental)";
        case ArmDriverMode::Off:
        default:                     return "Off";
    }
}

ArmDriverMode arm_driver_mode() {
    // The kill switch outranks the selector: g_cfg.enabled false means the plugin is standing
    // down entirely, and an arm driver is exactly the kind of thing that must stop when it does.
    if (!g_cfg.enabled) return ArmDriverMode::Off;
    // PaletteWeapon: armdriver=3, or the fork's palettewpn=1 read as an alias HERE and nowhere else,
    // so flipping either one goes through the same release-then-install path below.
    if (g_cfg.arm_driver == 3 || g_cfg.palette_weapon) return ArmDriverMode::PaletteWeapon;
    switch (g_cfg.arm_driver) {
        case 1:  return ArmDriverMode::UeRig;
        case 2:  return ArmDriverMode::Palette;
        default: return ArmDriverMode::Off;
    }
}

namespace {

// The mode that will actually run, which is not always the one asked for.
//
// WHY THE FALLBACK EXISTS. Selecting the palette route stands the UE route DOWN -- no spawned
// hands, no arm hiding, and Rig.cpp releases the weapon's controller attachment. If the palette
// route then cannot install its hook, nothing is driving the arms OR holding the weapon, and
// the player is left worse off than if they had never enabled it, with no way back except
// editing a config file. Falling back is not politeness; it is the difference between an
// experiment that fails safe and one that breaks the game for whoever tried it.
//
// Note this is checked EVERY tick, not only on a config change: the palette route can fail
// LATE. The watchdog only decides the hook is dead after enough gameplay has passed for a call
// to have been possible, which is minutes after the mode was selected.
ArmDriverMode effective_mode() {
    const ArmDriverMode wanted = arm_driver_mode();
    if (wanted == ArmDriverMode::Palette && palettearm_unavailable()) {
        return ArmDriverMode::UeRig;
    }
    // Same fallback for the palette weapon: a builder hook that refused (prologue mismatch, failed
    // install) leaves nothing placing the gun, so the UE route takes it back.
    if (wanted == ArmDriverMode::PaletteWeapon && blam_palette_unavailable()) {
        return ArmDriverMode::UeRig;
    }
    return wanted;
}

} // namespace

bool arm_driver_owns(ArmDriverMode mode) {
    // Deliberately reads s_active, NOT the config. Between the config changing and arbitrate()
    // running, the outgoing driver is still the one holding the handles -- answering from the
    // config here would let the incoming driver take a frame before the outgoing one released,
    // which is precisely the overlap this file exists to prevent.
    return s_active == mode && mode != ArmDriverMode::Off;
}

bool palette_weapon_mode() {
    return s_active_mirror.load(std::memory_order_relaxed) == (int)ArmDriverMode::PaletteWeapon;
}

void aim_reanchor_request(const char* why);   // Plugin.cpp: drop the aim reference so it re-captures

void arm_driver_release_all(const char* why) {
    // Order matters: hands are attached to things arms_release_hide() may invalidate.
    hands_release();
    arms_release_hide();
    palettearm_release();
    // The palette weapon's builder-path hooks, pose, holds and freezes (logged by itself when it held
    // anything). Released with the others so no two builder hooks can ever be live at once.
    blam_palette_release(why != nullptr ? why : "arm driver release", nullptr, 0);
    palette_two_hand_reset();
    if (why != nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] ARMDRIVER: released all (%s)", why);
    }
}

void arm_driver_arbitrate() {
    // A change to the key re-arms a route that gave up, BEFORE the effective mode is computed --
    // otherwise the latch would still be set and the retry would be swallowed on the same tick.
    const int key_now = g_cfg.arm_driver * 2 + (g_cfg.palette_weapon ? 1 : 0);
    if (key_now != s_last_cfg) {
        s_last_cfg = key_now;
        palettearm_retry();
        blam_palette_retry();
    }

    const ArmDriverMode wanted = effective_mode();

    if (!s_announced) {
        API::get()->log_info("[Halo-CampE-UEVR] ARMDRIVER: %s", arm_driver_name(wanted));
        s_announced = true;
        s_active = wanted;
        s_active_mirror.store((int)wanted, std::memory_order_relaxed);
        if (s_active == ArmDriverMode::PaletteWeapon) blam_palette_pose_hook_sync();
        return;
    }

    // While the palette weapon owns, keep its builder hook in step every tick (a palettehook pass
    // change, a hook the game dropped at a level load). Cheap: an id compare when nothing changed.
    if (wanted == s_active) {
        if (s_active == ArmDriverMode::PaletteWeapon) blam_palette_pose_hook_sync();
        return;
    }

    // Release EVERYTHING, not just the outgoing driver. Releasing only the loser assumes the
    // winner holds nothing stale, which is false after a level load or a failed frame -- and the
    // cost of an extra release on a mode change, which happens by hand a few times a session, is
    // nothing.
    // Name the REQUESTED mode too when they differ, so a fallback reads as a fallback in the log
    // rather than as the user's setting being silently ignored.
    const ArmDriverMode requested = arm_driver_mode();
    if (requested != wanted) {
        API::get()->log_info("[Halo-CampE-UEVR] ARMDRIVER: %s -> %s (FELL BACK; %s was requested "
                             "but is unavailable this session)",
                             arm_driver_name(s_active), arm_driver_name(wanted),
                             arm_driver_name(requested));
    } else {
        API::get()->log_info("[Halo-CampE-UEVR] ARMDRIVER: %s -> %s",
                             arm_driver_name(s_active), arm_driver_name(wanted));
    }
    // ONE LINE PER SWITCH: what was removed, what was installed, and how long between the two.
    const ArmDriverMode prev = s_active;
    const bool his_hook_before = halo::palettearm::palettehook_installed();
    char ours_removed[128] = "";
    const double t_release = now_ms_d();
    hands_release();
    arms_release_hide();
    palettearm_release();
    blam_palette_release("arm driver switch", ours_removed, sizeof(ours_removed));
    palette_two_hand_reset();
    if (two_hand_latched()) two_hand_reset("arm driver switch");
    const double t_released = now_ms_d();

    // Only now does the incoming driver start answering true.
    s_active = wanted;
    s_active_mirror.store((int)wanted, std::memory_order_relaxed);

    // The aim owner may have changed (his shotpoint/rig aim <-> the palette weapon's): drop the aim
    // reference so the next tick re-captures it against where the game is aiming now, instead of
    // applying the previous owner's offset for a frame. Calibration files are not touched.
    aim_reanchor_request("arm driver switch");

    char installed[96] = "none this tick";
    if (s_active == ArmDriverMode::PaletteWeapon) {
        const int id = blam_palette_pose_hook_sync();
        if (id >= 0) std::snprintf(installed, sizeof(installed), "palettewpn builder hook id=%d", id);
        else         std::snprintf(installed, sizeof(installed), "palettewpn builder hook REFUSED (see PALETTEHOOK)");
    } else if (s_active == ArmDriverMode::Palette) {
        std::snprintf(installed, sizeof(installed), "palettearm builder hook installs in its own update (PALETTEARM line)");
    }
    const double t_installed = now_ms_d();
    API::get()->log_info("[Halo-CampE-UEVR] ARMDRIVER SWITCH %s -> %s | removed: palettearm hook %s, %s | "
                         "installed: %s | release %.2f ms, release->install %.2f ms",
                         arm_driver_name(prev), arm_driver_name(s_active),
                         his_hook_before ? "yes" : "none",
                         ours_removed[0] ? ours_removed : "palettewpn hooks none",
                         installed, t_released - t_release, t_installed - t_released);
}

} // namespace halo
