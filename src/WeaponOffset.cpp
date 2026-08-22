#include "WeaponOffset.hpp"

#include "Config.hpp"
#include "Rig.hpp"
#include "WeaponCalib.hpp"
#include "UeObject.hpp"

#include <cstring>
#include <string>

using uevr::API;

namespace halo {
namespace {

// The calibrated values exactly as the FILE supplies them, before any delta.
struct Base {
    float grip = 0.0f, grip_yaw = 0.0f, grip_roll = 0.0f;
    float off_x = 0.0f, off_y = 0.0f, off_z = 0.0f;
    bool  valid = false;
};
Base s_base;

// The load generation the base was taken from. NOT g_cfg_check_tick -- see Config.hpp.
uint32_t s_last_gen = 0xFFFFFFFFu;
std::wstring s_last_weapon;

// The adjustment currently folded into g_cfg, for the calibration handoff. Not used to undo
// anything: load_config() does the undoing, by overwriting g_cfg from disk.
WeaponAdjust s_applied{};
bool         s_have_applied = false;

void publish_base() {
    g_cfg.wpn_base_grip      = s_base.grip;
    g_cfg.wpn_base_grip_yaw  = s_base.grip_yaw;
    g_cfg.wpn_base_grip_roll = s_base.grip_roll;
    g_cfg.wpn_base_off_x     = s_base.off_x;
    g_cfg.wpn_base_off_y     = s_base.off_y;
    g_cfg.wpn_base_off_z     = s_base.off_z;
}

} // namespace

void weapon_offset_update() {
    if (!g_cfg.wpn_offsets) {
        if (s_base.valid) {
            g_cfg.grip_deg = s_base.grip;   g_cfg.grip_yaw = s_base.grip_yaw;
            g_cfg.grip_roll = s_base.grip_roll;
            g_cfg.off_x = s_base.off_x; g_cfg.off_y = s_base.off_y; g_cfg.off_z = s_base.off_z;
            s_base.valid = false;
            s_have_applied = false;
            s_last_weapon.clear();
        }
        return;
    }

    // RE-CAPTURE ONLY WHEN THE CONFIG ACTUALLY RELOADED FROM DISK.
    //
    // A successful load_config() reparses halo_vr_calib.cfg and so has ALREADY put the clean
    // calibrated values back into g_cfg -- it runs before this function every ~2 s. Reading them
    // here is therefore reading the file, and no undo of our own delta is needed or wanted.
    //
    // The earlier version tried to be defensive and subtracted the applied delta before capturing,
    // to cover the early return load_config() takes when the main file fails to parse. That undo
    // fires on the NORMAL path too, where there is nothing to undo, so the base walked by -delta
    // every reload: fixed direction, growing magnitude, weapon receding. The correct answer to a
    // failed parse is to not capture at all, which is what watching the load generation gives --
    // it is bumped past every early return, unlike g_cfg_check_tick.
    if (g_cfg_load_gen != s_last_gen || !s_base.valid) {
        s_last_gen = g_cfg_load_gen;
        s_base.grip      = g_cfg.grip_deg;
        s_base.grip_yaw  = g_cfg.grip_yaw;
        s_base.grip_roll = g_cfg.grip_roll;
        s_base.off_x     = g_cfg.off_x;
        s_base.off_y     = g_cfg.off_y;
        s_base.off_z     = g_cfg.off_z;
        s_base.valid     = true;
        s_have_applied   = false;
        publish_base();
    }

    // Which weapon. Every FP weapon class on this title is <something>_WeaponActor_C, per Rig.cpp,
    // so the class name is the identity -- there is no need for a handle that survives the swap.
    std::wstring wpn;
    if (auto* actor = fp_weapon_actor()) wpn = class_name_of(actor);

    // Find a matching entry. SUBSTRING, not exact: class names on this title are long and
    // decorated, and a config that has to reproduce them character-perfect is one nobody will
    // successfully write. "AssaultRifle" should match BP_AssaultRifle_WeaponActor_C.
    const WeaponAdjust* hit = nullptr;
    if (!wpn.empty()) {
        for (int i = 0; i < g_cfg.wpn_count; ++i) {
            const auto& w = g_cfg.wpn[i];
            if (w.match[0] == 0) continue;
            const std::string m{w.match};
            const std::wstring wm(m.begin(), m.end());
            if (wpn.find(wm) != std::wstring::npos) { hit = &w; break; }
        }
    }

    // Base first, always, by ASSIGNMENT. A weapon with no entry must land exactly on the
    // calibration, and assigning rather than skipping the add means a swap from an adjusted weapon
    // to an unadjusted one cannot leave the previous adjustment in place -- and that a tick which
    // ran on the parse-failure path cannot accumulate either.
    g_cfg.grip_deg  = s_base.grip;
    g_cfg.grip_yaw  = s_base.grip_yaw;
    g_cfg.grip_roll = s_base.grip_roll;
    g_cfg.off_x     = s_base.off_x;
    g_cfg.off_y     = s_base.off_y;
    g_cfg.off_z     = s_base.off_z;

    if (hit != nullptr) {
        g_cfg.grip_deg  += hit->d_grip;
        g_cfg.grip_yaw  += hit->d_grip_yaw;
        // Gated even though capture now stores zero: hand-written wpnoff lines in halo_vr.cfg are
        // loaded into this same table and predate the rule.
        if (g_cfg.wpn_roll) g_cfg.grip_roll += hit->d_grip_roll;
        g_cfg.off_x     += hit->d_x;
        g_cfg.off_y     += hit->d_y;
        g_cfg.off_z     += hit->d_z;
        s_applied = *hit;
        s_have_applied = true;
    } else {
        s_have_applied = false;
    }

    // STATIC ROLL WINS. Last write before the rig reads these, so nothing upstream can beat it.
    if (g_cfg.roll_static > -998.0f) g_cfg.grip_roll = g_cfg.roll_static;

    if (wpn != s_last_weapon) {
        s_last_weapon = wpn;
        if (g_cfg.wpn_log) {
            if (hit != nullptr) {
                API::get()->log_info("[Halo-CampE-UEVR] WPNOFF %ls -> '%s' d=(%.2f,%.2f,%.2f) "
                                     "grip=(%.2f,%.2f,%.2f)",
                                     wpn.empty() ? L"<none>" : wpn.c_str(), hit->match,
                                     hit->d_x, hit->d_y, hit->d_z,
                                     hit->d_grip, hit->d_grip_yaw, hit->d_grip_roll);
            } else {
                API::get()->log_info("[Halo-CampE-UEVR] WPNOFF %ls -> no entry, using calibration",
                                     wpn.empty() ? L"<none>" : wpn.c_str());
            }
        }
    }
}

void weapon_offset_adopt_solve() {
    if (!g_cfg.wpn_offsets || !s_base.valid) return;

    // END SETS THE GLOBAL. Full stop.
    //
    // This used to store fit MINUS the held weapon delta, which is self-consistent but makes END
    // output depend on HOME output -- and HOME measures against the base END just wrote. Two
    // gestures defined in terms of each other: an error in either survives forever, invisibly,
    // because only their SUM is ever displayed. Measured live: an AR delta of zero came back as
    // +2.3 after one HOME, then the base moved -2.3 on the next END, repeatedly.
    //
    // So the fit becomes the global verbatim and the weapon in hand loses its delta -- it IS the
    // baseline now, which is what "set the global from this weapon" means. HOME then only ever
    // moves a weapon away from a baseline chosen deliberately. No feedback path exists.
    if (s_have_applied) {
        for (int i = 0; i < g_cfg.wpn_count; ++i) {
            if (_stricmp(g_cfg.wpn[i].match, s_applied.match) != 0) continue;
            g_cfg.wpn[i].d_x = 0.0f; g_cfg.wpn[i].d_y = 0.0f; g_cfg.wpn[i].d_z = 0.0f;
            g_cfg.wpn[i].d_grip = 0.0f; g_cfg.wpn[i].d_grip_yaw = 0.0f;
            g_cfg.wpn[i].d_grip_roll = 0.0f;
            API::get()->log_info("[Halo-CampE-UEVR] END: '%s' is the baseline now, delta cleared",
                                 g_cfg.wpn[i].match);
            break;
        }
        s_have_applied = false;
        wpn_calib_write_file();
    }

    s_base.grip      = g_cfg.grip_deg;
    s_base.grip_yaw  = g_cfg.grip_yaw;
    s_base.grip_roll = g_cfg.grip_roll;
    s_base.off_x     = g_cfg.off_x;
    s_base.off_y     = g_cfg.off_y;
    s_base.off_z     = g_cfg.off_z;
    publish_base();

    // The file is about to receive this, so the next reload reads back the same numbers and the
    // generation check finds nothing to change.
}

} // namespace halo
