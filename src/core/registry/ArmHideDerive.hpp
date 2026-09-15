#pragma once

// THE ARM HIDE AND RIG DERIVATION for the palette weapon (palettehidearms) and manual reload
// (reloadhidearms). Pure: it reads and writes one Config and which of the author's keys a cfg file
// set, and nothing else, so features_apply runs it on every load and a host-side test can run it on
// any combination of settings.

namespace halo {

struct Config;

// Which of the author's keys a cfg file set this load. A set key is the player's and is never derived.
struct ArmHideLayers {
    bool rig         = false;
    bool armhide     = false;
    bool armhidemode = false;
    bool armhidebone = false;
};

struct ArmHideResolution {
    bool rig_off       = false;   // rig_enabled derived false
    int  rig_reason    = 0;       // 1 palette weapon not requested, 2 rig set in a cfg file, 3 derived off
    int  pal_active    = 0;       // the palettehidearms approach in effect (0 = none)
    int  pal_reason    = 0;       // see arm_hide_pal_reason_text
    int  reload_active = 0;       // the reloadhidearms approach in effect (0 = none)
    int  reload_reason = 0;       // see arm_hide_reload_reason_text
};

// Run after every cfg layer has been parsed and the feature masters are resolved.
ArmHideResolution arm_hide_derive(Config& c, const ArmHideLayers& set);

const char* arm_hide_rig_reason_text(int reason);
const char* arm_hide_pal_reason_text(int reason);
const char* arm_hide_reload_reason_text(int active, int reason);

} // namespace halo
