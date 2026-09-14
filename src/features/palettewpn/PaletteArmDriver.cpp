#include "features/palettewpn/PaletteArmDriver.hpp"

#include "Arms.hpp"
#include "BlamPalette.hpp"
#include "Config.hpp"
#include "PaletteTwoHand.hpp"
#include "Rig.hpp"
#include "TwoHandAim.hpp"
#include "core/host/ArmsState.hpp"
#include "palettearm/PaletteHook.hpp"   // palettehook_installed(): the switch line names both builder hooks
#include "uevr/API.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>

using uevr::API;

namespace halo {

void aim_reanchor_request(const char* why);   // Plugin.cpp: drop the aim reference so it re-captures

namespace {

// ANY-THREAD MIRROR of the arbiter's active mode (PaletteWeapon is read by the sim-thread palette hook and
// by the XInput aim derivation, neither of which may read the tick-thread value directly). Written by the
// arbiter's active-mode hook, so it moves exactly when the arbiter's own s_active does.
std::atomic<int> s_active_mirror{0};
int  s_last_key = -1;                  // armdriver and palettewpn together, as the arbiter's retry key
char s_removed[128] = "";              // the palette hooks the last release removed, for the switch line
bool s_his_hook_before = false;        // the palettearm builder hook was installed when that release began
double s_t_release = 0.0, s_t_released = 0.0;
bool s_arm_hide_applied = false;       // the current arm hide was applied by this feature

double now_ms_d() {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

} // namespace

bool palette_weapon_mode() {
    return s_active_mirror.load(std::memory_order_relaxed) == (int)kPaletteWeaponMode;
}

const char* palette_wpn_arm_driver_name(int mode) {
    return mode == (int)kPaletteWeaponMode ? "PaletteWeapon (palette weapon + aim, experimental)" : nullptr;
}

// PaletteWeapon: armdriver=3, or the fork's palettewpn=1 read as an alias HERE and nowhere else,
// so flipping either one goes through the same release-then-install path.
int palette_wpn_arm_driver_mode_wanted() {
    return (g_cfg.arm_driver == 3 || g_cfg.palette_weapon) ? (int)kPaletteWeaponMode : 0;
}

// Same fallback as the palettearm route: a builder hook that refused (prologue mismatch, failed
// install) leaves nothing placing the gun, so the UE route takes it back.
bool palette_wpn_arm_driver_mode_unavailable(int wanted) {
    return wanted == (int)kPaletteWeaponMode && blam_palette_unavailable();
}

// A change to armdriver or palettewpn re-arms a builder hook that gave up, and (true) re-arms the
// palettearm route the same way the author's armdriver change does.
bool palette_wpn_arm_driver_key_changed() {
    const int key_now = g_cfg.arm_driver * 2 + (g_cfg.palette_weapon ? 1 : 0);
    if (key_now == s_last_key) return false;
    s_last_key = key_now;
    blam_palette_retry();
    return true;
}

// While the palette weapon owns, keep its builder hook in step every tick (a palettehook pass
// change, a hook the game dropped at a level load). Cheap: an id compare when nothing changed.
void palette_wpn_arm_driver_steady(int active) {
    if (active == (int)kPaletteWeaponMode) blam_palette_pose_hook_sync();
}

// The palette weapon's builder-path hooks, pose, holds and freezes (logged by itself when it held
// anything). Released with the others so no two builder hooks can ever be live at once. The arbiter's
// switch releases with no reason; every other caller names one.
void palette_wpn_arm_driver_release_all(const char* why) {
    s_his_hook_before = halo::palettearm::palettehook_installed();
    s_t_release = now_ms_d();
    s_removed[0] = 0;
    blam_palette_release(why != nullptr ? why : "arm driver switch", s_removed, sizeof(s_removed));
    palette_two_hand_reset();
    s_t_released = now_ms_d();
}

void palette_wpn_arm_driver_active(int mode, bool switched) {
    const int prev = s_active_mirror.exchange(mode, std::memory_order_relaxed);
    if (!switched) {
        if (mode == (int)kPaletteWeaponMode) blam_palette_pose_hook_sync();
        return;
    }
    // Only a switch into or out of the palette weapon changes the aim owner.
    if (prev != (int)kPaletteWeaponMode && mode != (int)kPaletteWeaponMode) return;
    if (two_hand_latched()) two_hand_reset("arm driver switch");
    // The aim owner may have changed (his shotpoint/rig aim <-> the palette weapon's): drop the aim
    // reference so the next tick re-captures it against where the game is aiming now, instead of
    // applying the previous owner's offset for a frame. Calibration files are not touched.
    aim_reanchor_request("arm driver switch");
    // ONE LINE PER SWITCH: what was removed, what was installed, and how long between the two.
    const ArmDriverMode s_active = (ArmDriverMode)mode;
    const ArmDriverMode prev_mode = (ArmDriverMode)prev;
    const bool his_hook_before = s_his_hook_before;
    const char* ours_removed = s_removed;
    const double t_release = s_t_release, t_released = s_t_released;
    char installed[96] = "none this tick";
    if (s_active == kPaletteWeaponMode) {
        const int id = blam_palette_pose_hook_sync();
        if (id >= 0) std::snprintf(installed, sizeof(installed), "palettewpn builder hook id=%d", id);
        else         std::snprintf(installed, sizeof(installed), "palettewpn builder hook REFUSED (see PALETTEHOOK)");
    } else if (s_active == ArmDriverMode::Palette) {
        std::snprintf(installed, sizeof(installed), "palettearm builder hook installs in its own update (PALETTEARM line)");
    }
    const double t_installed = now_ms_d();
    API::get()->log_info("[Halo-CampE-UEVR] ARMDRIVER SWITCH %s -> %s | removed: palettearm hook %s, %s | "
                         "installed: %s | release %.2f ms, release->install %.2f ms",
                         arm_driver_name(prev_mode), arm_driver_name(s_active),
                         his_hook_before ? "yes" : "none",
                         ours_removed[0] ? ours_removed : "palettewpn hooks none",
                         installed, t_released - t_release, t_installed - t_released);
}

// ---- THE ARM HIDE (Arms.cpp's hooks)

// Handles the bone modes while the feature is enabled (armhidemode 0 and 3), and releases a hide it
// applied even after the feature switched off. Modes 1 and 2 stay the author's.
bool palette_wpn_arm_hide_component(API::UObject* comp, bool hide, int mode, bool enabled) {
    if (hide) {
        s_arm_hide_applied = enabled && mode != 1 && mode != 2;
        if (!s_arm_hide_applied) return false;
    } else if (!s_arm_hide_applied || mode == 1 || mode == 2) {
        return false;
    }
    const auto call_hide_bone   = host::g_arms_state.call_hide_bone;
    const auto call_unhide_bone = host::g_arms_state.call_unhide_bone;
    const auto call_set_hidden  = host::g_arms_state.call_set_hidden;
    // COMMA-SEPARATED bone list (fork addition). A single name behaves exactly as before; the palette
    // weapon presentation passes "Shoulder_L,Shoulder_R" so both arms hide by bone while the weapon
    // branch stays drawn. Everything at or below space is trimmed: a CRLF file leaves '\r' on the last
    // token, and FName Add-mode would CREATE that bogus name and hide nothing.
    std::wstring bones[8];
    int nbones = 0;
    {
        const char* s2 = g_cfg.arm_hide_bone;
        while (*s2 != 0 && nbones < 8) {
            const char* e = s2;
            while (*e != 0 && *e != ',') ++e;
            std::string one(s2, e);
            while (!one.empty() && (unsigned char)one.back() <= ' ') one.pop_back();
            while (!one.empty() && (unsigned char)one.front() <= ' ') one.erase(one.begin());
            if (!one.empty()) bones[nbones++] = std::wstring(one.begin(), one.end());
            s2 = (*e == ',') ? e + 1 : e;
        }
    }
    // Mode 3 (weapon-only, fork addition, only when armhidemode=3): the FP pawn is MODULAR -- armour
    // pieces are separate skeletal-mesh components of the same classes, and an arm bone name is a
    // silent no-op on them. So whole-hide every swept component EXCEPT the one the rig tracks (it
    // carries the weapon bones), and bone-hide the arm list on that one.
    API::UObject* keep = (mode == 3) ? rig_tracked_component() : nullptr;
    switch (mode) {
        case 3:  if (comp == keep) {
                     for (int bi = 0; bi < nbones; ++bi) {
                         if (hide) call_hide_bone(comp, bones[bi].c_str());
                         else      call_unhide_bone(comp, bones[bi].c_str());
                     }
                     if (!hide) call_set_hidden(comp, false);
                 } else {
                     call_set_hidden(comp, hide);
                 }
                 break;
        default: for (int bi = 0; bi < nbones; ++bi) {
                     if (hide) call_hide_bone(comp, bones[bi].c_str());
                     else      call_unhide_bone(comp, bones[bi].c_str());
                 }
                 break;
    }
    return true;
}

bool palette_wpn_arm_hide_held_off() {
    bool& s_any_hidden = *host::g_arms_state.any_hidden;
    // ADDITION -- HOLD OFF WHILE THE FP BUILD IS DARK, AND FOR TWO SECONDS AFTER IT RETURNS.
    // Bisected 2026-08-26: with the sweep running, every respawn froze the palette-driven
    // PrimaryWeapon socket at the stock pose ~1 s in -- the hide lands on the freshly rebuilt
    // components mid-initialization and the FP palette sync never binds, so the weapon actor
    // rides the camera-parented mesh at its stock socket ("attached to the rig"). The same death
    // with armhide=0 tracks perfectly, and the pre-death sweep on settled components never hurt
    // anything.
    //
    // A pawn-identity watch was the first cut and it MISSED: this build reuses the pawn object
    // across a death and rebuilds only its COMPONENTS (the respawn logged a new rig component
    // and no pawn change). The one signal that reliably goes dark at every rebuild window is
    // the FP palette build itself -- it stops within a frame on death, seats and cutscenes,
    // which are exactly the moments components get torn down. So the sweep runs only once the
    // build has been back for ~2 s, and a dark spell forgets the hide state (the components it
    // covered are being torn down; releasing would sweep whatever replaced them).
    {
        static uint32_t s_hold = 0;
        // Only while the palette weapon (armdriver mode 3) owns placement: under the author's arm
        // drivers the palette build stamp never runs, and this gate would hold the hide off forever.
        if (palette_weapon_mode() && !blam_palette_fp_live()) {
            if (s_hold == 0 && s_any_hidden) {
                API::get()->log_info("[Halo-CampE-UEVR] ARMHIDE: FP build went dark -- holding "
                                     "the sweep until it is back ~2 s");
            }
            s_hold = 64;   // ~2 s at the ~32 Hz tick, restarted while dark
            s_any_hidden = false;
            return true;
        }
        if (s_hold != 0) { --s_hold; return true; }
    }
    return false;
}

// Mode 3 bone-hides on the component the rig tracks, so it needs the rig like mode 0.
bool palette_wpn_arm_hide_needs_rig() { return g_cfg.arm_hide_mode == 3; }

// The feature switched off: a hide it applied is released now (its mode 3 whole-hides are not the
// author's to undo), and the author's pass re-applies his own on its next tick.
void palette_wpn_arm_hide_released() {
    if (s_arm_hide_applied) arms_release_hide();
}

} // namespace halo
