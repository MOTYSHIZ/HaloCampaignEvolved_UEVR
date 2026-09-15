#include "core/registry/ArmHideDerive.hpp"

#include "Config.hpp"

#include <cstring>

namespace halo {

// THE PALETTE WEAPON TURNS THE AUTHOR'S RIG OFF AND HIDES HIS ARMS (palettehidearms, a sub-setting of
// palettewpn). The palette weapon places the gun on the first-person mesh itself, so his rig driver
// only stands down under it (the on-foot reticule and the unarmed hide live inside that driver), and
// his arms keep their canned pose beside a weapon that follows the hand. While the palette weapon is
// requested, rig is derived off and his arm hide on, each only where no cfg file sets that key, so a
// player's own rig, armhide, armhidemode or armhidebone always wins. Nothing is remembered between
// loads: load_config rebuilds the Config from its defaults and every file before this runs, so
// switching the palette weapon off gives back exactly the layered values.
//
// MANUAL RELOAD HIDES THE AUTHOR'S STOCK ARMS (reloadhidearms, a sub-setting of reloadvr), through his
// own arm hide: armhide on, and for approach 2 his SetVisibility mode, each only where no file sets
// that key. While the palette weapon is requested it owns the arm hide, so this stands down. Approach
// 3 (only during a reload) is held off between reloads by reloadvr's arm hide hold-off. With reloadvr
// off nothing is derived.
ArmHideResolution arm_hide_derive(Config& c, const ArmHideLayers& set) {
    ArmHideResolution r{};
    const bool palette = c.palette_weapon || c.arm_driver == 3;

    if (!palette)     r.rig_reason = 1;
    else if (set.rig) r.rig_reason = 2;
    else { c.rig_enabled = false; r.rig_off = true; r.rig_reason = 3; }

    if (!palette)                        r.pal_reason = 1;
    else if (c.palette_hide_arms <= 0)   r.pal_reason = 2;
    else if (set.armhide)                r.pal_reason = 3;
    else {
        r.pal_active = c.palette_hide_arms;
        c.arm_hide = true;
        if (!set.armhidemode) c.arm_hide_mode = (r.pal_active == 3) ? 1 : 3;
        if (r.pal_active != 3 && !set.armhidebone) strcpy_s(c.arm_hide_bone, "Shoulder_L,Shoulder_R");
        r.pal_reason = set.armhidemode ? 7 : (3 + r.pal_active);
    }
    c.palette_hide_arms_active = r.pal_active;

    if (!c.reload_vr)                    r.reload_reason = 1;
    else if (c.reload_hide_arms <= 0)    r.reload_reason = 2;
    else if (palette)                    r.reload_reason = 3;
    else if (set.armhide)                r.reload_reason = 4;
    else {
        r.reload_active = c.reload_hide_arms;
        c.arm_hide = true;
        if (r.reload_active == 2 && !set.armhidemode) c.arm_hide_mode = 1;
        r.reload_reason = set.armhidemode ? 6 : 5;
    }
    c.reload_hide_arms_active = r.reload_active;
    return r;
}

const char* arm_hide_rig_reason_text(int reason) {
    switch (reason) {
    case 1:  return "the palette weapon is off, rig as the cfg files and defaults say";
    case 2:  return "rig is set in a cfg file";
    case 3:  return "off, the palette weapon places the gun";
    default: return "?";
    }
}

const char* arm_hide_pal_reason_text(int reason) {
    switch (reason) {
    case 1:  return "the palette weapon is off";
    case 2:  return "palettehidearms=0";
    case 3:  return "armhide is set in a cfg file";
    case 4:  return "hidden by bone on the mesh the rig tracks, armour hidden whole";
    case 5:  return "hidden by bone on the mesh the weapon is attached to, armour hidden whole";
    case 6:  return "hidden whole through SetVisibility";
    case 7:  return "hidden with the armhidemode a cfg file sets";
    default: return "?";
    }
}

const char* arm_hide_reload_reason_text(int active, int reason) {
    switch (reason) {
    case 1:  return "manual reload is off";
    case 2:  return "reloadhidearms=0";
    case 3:  return "the palette weapon owns the arm hide";
    case 4:  return "armhide is set in a cfg file";
    case 5:  return (active == 3) ? "hidden only while a reload is in progress" : "hidden while manual reload is on";
    case 6:  return (active == 3) ? "hidden only while a reload is in progress"
                                  : "hidden while manual reload is on, with the armhidemode a cfg file sets";
    default: return "?";
    }
}

} // namespace halo
