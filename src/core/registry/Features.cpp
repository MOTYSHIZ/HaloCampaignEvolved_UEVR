#include "core/registry/Features.hpp"

#include "Config.hpp"
#include "features/FeatureList.hpp"
#include "features/hooks/ConfigHooks.hpp"   // features_menu_status_line
#include "core/config/CfgRead.hpp"
#include "uevr/API.hpp"

#include <windows.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace halo {
namespace {

// =================================================================================================
// FEATURE REGISTRY
//
// TIER ASSIGNMENT IS THE UPSTREAM AUTHOR'S CALL. Rows contributed from the fork land as
// Tier::Experimental (off by default), and only he promotes a row, by editing its tier. That one
// edit is the whole promotion: defaults, the menu section and the logs all follow the tier.
//
// A ROW: the feature's ONE master key (the single switch the menu toggle writes), the value that
// key takes when the feature is on, the tier, the menu group and name, a one-line plain-language
// description, its sub-settings (keys that only act while the master is on, shown nested under it
// in the menu), the master keys it needs, and accessors for the master field.
//
// TIERS AND DEFAULTS
//   Stable        the author's released features. The registry never writes them: the built-in
//                 default in Config.hpp stands exactly as he shipped it.
//   Beta          on, unless tierbeta=0.
//   Preview       off, unless tierpreview=1.
//   Experimental  off, unless tierexperimental=1.
//
// PRECEDENCE, strongest first
//   1. The master key set in any cfg file. Between files the usual layer order applies
//      (halo_vr_dev.cfg > halo_vr_user.cfg > halo_vr.cfg); the settings menu writes
//      halo_vr_user.cfg.
//   2. The tier switch of the row's tier (tierbeta / tierpreview / tierexperimental), itself set in
//      any cfg file. A single master key always beats its tier switch.
//   3. The tier default above.
//
// A config that sets a master key keeps working exactly as before: the key is explicit, so the tier
// never touches it.
// =================================================================================================

enum class Tier { Stable, Beta, Preview, Experimental };

struct FeatureRow {
    const char* key;
    int         on;
    Tier        tier;
    const char* group;
    const char* name;
    const char* desc;
    const char* subkeys;
    const char* needs;
    int  (*get)(const Config&);
    void (*set)(Config&, int);
};

#define FEATURE_BOOL(f) [](const Config& c) { return c.f ? 1 : 0; }, [](Config& c, int v) { c.f = (v != 0); }
#define FEATURE_INT(f)  [](const Config& c) { return (int)c.f; },     [](Config& c, int v) { c.f = v; }

const FeatureRow kFeatures[] = {
    // ---- The author's released features (Stable: never written by the registry; his keys, his
    // defaults). Listed so the menu and the logs show them beside the rest.
    { "aimreticule",  1, Tier::Stable, "", "Reticule in the world",
      "The game's crosshair drawn out in the world, where your shots land.",
      "", "", FEATURE_BOOL(aim_reticule) },
    { "xrlayer",      1, Tier::Stable, "", "Headset-drawn reticule",
      "The reticule drawn by the headset itself, so the game's lighting never dims it.",
      "", "aimreticule", FEATURE_BOOL(xr_layer) },
    { "cutscenemono", 6, Tier::Stable, "", "Cutscene screen",
      "Cutscenes shown on one flat screen in front of you.",
      "cutscenesize", "", FEATURE_INT(cutscene_mono) },
    { "cullfix",      1, Tier::Stable, "", "Distant detail",
      "Keeps distant objects drawn at VR view distances.",
      "culldist", "", FEATURE_BOOL(cull_fix) },
    { "meleeswing",   1, Tier::Stable, "", "Melee by swinging",
      "Swing your weapon hand to melee.",
      "", "", FEATURE_BOOL(melee_swing) },
    { "holster",      1, Tier::Stable, "", "Holsters",
      "Reach over a shoulder to stow or draw a weapon, and to your chest for a grenade.",
      "", "", FEATURE_BOOL(holster_enabled) },
    { "twohand",      1, Tier::Stable, "", "Two-handed aiming",
      "Bring your other hand to the barrel and squeeze its grip to steady your aim.",
      "", "", FEATURE_INT(two_hand) },
    { "scope",        1, Tier::Stable, "", "Scope",
      "Squeeze the left trigger to zoom through a lens on your aim line.",
      "", "", FEATURE_BOOL(scope_enabled) },
    { "armhide",      1, Tier::Stable, "", "Hide the stock arms",
      "Hides the game's own first-person arms.",
      "", "", FEATURE_BOOL(arm_hide) },

    // ---- Contributed from the fork (Experimental until the author promotes them). Where a fork
    // feature would collide with one of his, it has its own master key.
    { "palettewpn",   1, Tier::Experimental, "Weapon", "Weapon follows your hand",
      "The weapon you see is placed on your hand and the aim follows the drawn barrel. Replaces the standard weapon placement while on.",
      "palettecam,poselatch,paletterolltrim,palbuildgate,palpubframe,twohandmarker,palettecamlead,meshconst", "", FEATURE_BOOL(palette_weapon) },
    { "aimbore",      1, Tier::Experimental, "Weapon", "Aim along the drawn barrel",
      "Shots follow the barrel of the weapon you see, not only your hand.",
      "", "palettewpn", FEATURE_INT(aim_bore) },
    { "aimreticulestamp", 1, Tier::Experimental, "Weapon", "Reticule placed every frame",
      "The headset-drawn reticule is placed each frame on exactly where your shots go.",
      "", "palettewpn,aimreticule,xrlayer", FEATURE_INT(aim_reticule_stamp) },
    { "scopelens",    1, Tier::Experimental, "Weapon", "Scope lens on the weapon",
      "A magnifying lens in the scope housing of the scoped weapons. The standard scope stands down while on.",
      "scopeev,scopetint,scopetonecurve,scopeeyedist,scopesource,scopertfmt,scopeshowflags,scopesfflags,scopeseptrans", "scope", FEATURE_BOOL(scope_lens) },
    { "reloadvr",     1, Tier::Experimental, "Reload", "Manual reload",
      "Drop the magazine, fetch a fresh one from your belt and push it into the gun.",
      "gripexclusive,reloadakmute,akmimic,coophide,hidesolo,reloadstate,reloadstatesave,reloadstatehide,reloadstatewaitms,reloadstatedrop,reloadstatedeath,reloadstatelevel,reloadstatelog,reloadlift,reloadslidems,reloadmagoffw,reloadhandoff,reloadhandrot,reloadanimrate,reloadwellmarker,roomanchor",
      "", FEATURE_BOOL(reload_vr) },
    { "slidevr",      1, Tier::Experimental, "Reload", "Rack the slide",
      "Rack the slide, pump or charging handle with your other hand.",
      "slideradius,slidetravel,slideoff,slidezoneback,slidepartrotaxis,slidefire,slidefireback,slidefireentry", "", FEATURE_BOOL(slide_vr) },
    { "meleeleft",    1, Tier::Experimental, "Melee and grenades", "Punch with your other hand",
      "Your other hand can melee too.",
      "", "meleeswing", FEATURE_BOOL(melee_left) },
    { "grenadeswallow", 1, Tier::Experimental, "Melee and grenades", "Grenades from the pouches only",
      "The left face button stops throwing grenades; grenades come from your chest pouches.",
      "", "", FEATURE_INT(grenade_swallow) },
    { "holsterpollthrow", 1, Tier::Experimental, "Melee and grenades", "Grenade throw on release",
      "A grenade leaves your hand the instant you open your grip, and the pouches show your real grenade counts.",
      "", "holster,blamangles", FEATURE_BOOL(holster_poll_throw) },
    { "wristhud",     1, Tier::Experimental, "HUD", "Wrist HUD",
      "Shield, weapon and grenade readouts on your forearm, and the motion tracker on your wrist.",
      "wristradar,hudplacement,wristhudclasses,wristhudoff,wristhudrot,wristhudgap,wristhudclassesr,wristhudoffr,wristhudrotr,wristradarblip,wristradargain", "", FEATURE_BOOL(wrist_hud) },
    { "forcetube",    1, Tier::Experimental, "Haptics", "ForceTube gunstock",
      "A kick in the ForceTube gunstock on every round you fire.",
      "forcetubekick,forcetuberadius", "", FEATURE_BOOL(force_tube) },
    { "vehcam",       1, Tier::Experimental, "Vehicles", "Vehicle seat camera",
      "A first-person view from your seat in vehicles.",
      "vehview,vehhidebody,vehcamguard,vehcamhullcheck", "blamangles", FEATURE_INT(veh_cam) },
    { "vehiclewheel", 1, Tier::Experimental, "Vehicles", "Steering wheel",
      "Hands on the steering wheel. Steering is not sent to the vehicle yet.",
      "", "blamangles", FEATURE_INT(vehicle_wheel) },
    { "roomscale",    1, Tier::Experimental, "Roomscale", "Roomscale",
      "Walk around your play space and your steps move the Spartan.",
      "blamthrottleysign,roomscalethrottle,roomscalegain,roomscalemin,roomscaledz", "blamangles", FEATURE_BOOL(roomscale) },
    { "heightcal",    1, Tier::Experimental, "Roomscale", "Auto height",
      "Your view height above the game floor follows your head above the real floor, so a real crouch lowers it.",
      "heightmode,heightsrc,heightsample,heighttrim,heightmin,heightkey", "", FEATURE_INT(height_cal) },
    { "headblock",    1, Tier::Experimental, "Roomscale", "Head block",
      "Keeps your head out of walls when you lean into them.",
      "headblockradius", "", FEATURE_INT(head_block) },
    { "stabilityfixes", 1, Tier::Experimental, "Stability", "Stability fixes",
      "Guards for the base mod: nav marker fault quarantine, fault recovery and stale rig guard, head tracking dropout gate, stick mode exit after a death, UI and reticle sweep throttles, asset load failure memo, reticle re-assert, early compositor reticule tick, teardown order, aim-hand melee holster veto and aim pin, two-handed hold release on a gesture reset, menu command file poll gate, holster marker tint and minimum throw speed.",
      "turnlog,widgetlog,markertint,holstermarkercolor,grenminthrow", "", FEATURE_BOOL(stability_fixes) },
};

#undef FEATURE_BOOL
#undef FEATURE_INT

constexpr int kCount = (int)(sizeof(kFeatures) / sizeof(kFeatures[0]));
constexpr int kTierCount = 4;

const char* const kTierName[kTierCount]   = { "stable", "beta", "preview", "experimental" };
const char* const kTierSwitch[kTierCount] = { nullptr, "tierbeta", "tierpreview", "tierexperimental" };
constexpr bool    kTierDefaultOn[kTierCount] = { true, true, false, false };

// The kConfigFiles index that last set each master key this load (-1 = no file set it).
int  s_layer = 0;
int  s_key_layer[kCount];
int  s_switch_layer[kTierCount];
bool s_switch_on[kTierCount];
// Shared infrastructure the resolver turns on for its consumers, unless a file sets it.
int  s_palette_hook_layer = -1;
int  s_pose_latch_layer   = -1;

int tier_index(Tier t) { return (int)t; }

const char* layer_source(int layer) {
    switch (layer) {
    case 0:  return "halo_vr.cfg";
    case 1:  return "user override";
    case 2:  return "halo_vr_dev.cfg";
    case 3:  return "halo_vr_calib.cfg";
    case 4:  return "halo_vr_weapons.cfg";
    default: return "cfg";
    }
}

// Short, stable source codes for the menu data file.
const char* layer_code(int layer) {
    switch (layer) {
    case 0:  return "base";
    case 1:  return "user";
    case 2:  return "dev";
    default: return "file";
    }
}

bool row_from_tier(int i) {
    return kFeatures[i].tier != Tier::Stable && s_key_layer[i] < 0;
}

bool tier_on(Tier t) {
    const int ti = tier_index(t);
    return (s_switch_layer[ti] >= 0) ? s_switch_on[ti] : kTierDefaultOn[ti];
}

const char* source_code(int i) {
    if (s_key_layer[i] >= 0) return layer_code(s_key_layer[i]);
    if (kFeatures[i].tier == Tier::Stable) return "builtin";
    return (s_switch_layer[tier_index(kFeatures[i].tier)] >= 0) ? "switch" : "tier";
}

std::string source_text(int i) {
    if (s_key_layer[i] >= 0) return layer_source(s_key_layer[i]);
    if (kFeatures[i].tier == Tier::Stable) return "built-in default";
    const int ti = tier_index(kFeatures[i].tier);
    if (s_switch_layer[ti] >= 0) return std::string(kTierSwitch[ti]) + " (" + layer_source(s_switch_layer[ti]) + ")";
    return "tier default";
}

bool write_text(const char* path, const std::string& text) {
    FILE* f = nullptr;
    if (fopen_s(&f, path, "wb") != 0 || f == nullptr) return false;
    fwrite(text.data(), 1, text.size(), f);
    fclose(f);
    return true;
}

void features_path(const char* data_dir, char* out, size_t cap) {
    sprintf_s(out, cap, "%s\\halo_vr_features.txt", data_dir);
}

} // namespace

void features_begin_load() {
    s_layer = 0;
    for (int i = 0; i < kCount; ++i) s_key_layer[i] = -1;
    for (int t = 0; t < kTierCount; ++t) { s_switch_layer[t] = -1; s_switch_on[t] = false; }
    s_palette_hook_layer = -1;
    s_pose_latch_layer = -1;
}

void features_config_reload_begin() { cfg_reload_begin(); }

void features_set_layer(int layer) { s_layer = layer; }

void features_note_key(const char* key, const char* val) {
    if (key == nullptr) return;
    for (int i = 0; i < kCount; ++i) {
        if (_stricmp(key, kFeatures[i].key) == 0) { s_key_layer[i] = s_layer; return; }
    }
    for (int t = 0; t < kTierCount; ++t) {
        if (kTierSwitch[t] != nullptr && _stricmp(key, kTierSwitch[t]) == 0) {
            s_switch_layer[t] = s_layer;
            s_switch_on[t] = (val != nullptr) && (atof(val) != 0.0);
            return;
        }
    }
    if (_stricmp(key, "palettehook") == 0) { s_palette_hook_layer = s_layer; return; }
    if (_stricmp(key, "poselatch") == 0)   { s_pose_latch_layer = s_layer; return; }
}

void features_apply() {
    for (int i = 0; i < kCount; ++i) {
        if (!row_from_tier(i)) continue;
        kFeatures[i].set(g_cfg, tier_on(kFeatures[i].tier) ? kFeatures[i].on : 0);
    }

    // SUB-SETTINGS FOLLOW THEIR MASTER where the code would otherwise act without it.
    //   The hidden reload (co-op, and solo) keeps the gun dead until the manual reload gesture gives
    //   it back; with manual reload off nothing ever would.
    if (!g_cfg.reload_vr) { g_cfg.coop_hide = false; g_cfg.hide_solo = false; }
    //   Off-hand melee is a setting of melee by swinging.
    if (!g_cfg.melee_swing) g_cfg.melee_left = false;

    // SHARED INFRASTRUCTURE, turned on for the features that need it unless a file sets it.
    //   The weapon placement (palettewpn = armdriver mode 3) moves the drawn weapon through the
    //   first-person pose hook and composes against the pose latch's stamped intent. Both act only
    //   while mode 3 owns (the arbiter installs the hook, the latch checks the mode), so with the
    //   placement off they stay at 0 and the author's pose path is untouched.
    if (s_palette_hook_layer < 0) g_cfg.palette_hook = g_cfg.palette_weapon ? 1 : 0;
    if (s_pose_latch_layer < 0)   g_cfg.pose_latch   = g_cfg.palette_weapon ? 2 : 0;

    // The reload's last write: hook threads read g_cfg again.
    cfg_reload_end();
}

// ---- WHAT IS ACTUALLY RUNNING (data\halo_vr_effective.txt). The catalog carries each switch's
// DEFAULT, but halo_vr.cfg, the dev file and the dependency rules can all change the value the plugin
// runs with, so a menu that shows the default can show a checkbox that disagrees with the game. This
// mirror is the running value of every feature switch, rewritten after every real reload and
// re-created if deleted. Same key=value shape as the cfg files.
struct EffectiveKey { const char* key; double (*get)(); };
static const EffectiveKey kEffectiveKeys[] = {
    { "enabled",          [] { return (double)g_cfg.enabled; } },
    { "armdriver",        [] { return (double)g_cfg.arm_driver; } },
    { "aimdirect",        [] { return (double)g_cfg.aim_direct; } },
    { "blamangles",       [] { return (double)g_cfg.blam_angles; } },
    { "stickmode",        [] { return (double)g_cfg.stick_mode; } },
    { "aimbore",          [] { return (double)g_cfg.aim_bore; } },
    { "aimreticule",      [] { return (double)g_cfg.aim_reticule; } },
    { "aimreticulestamp", [] { return (double)g_cfg.aim_reticule_stamp; } },
    { "xrlayer",          [] { return (double)g_cfg.xr_layer; } },
    { "cutscenemono",     [] { return (double)g_cfg.cutscene_mono; } },
    { "cullfix",          [] { return (double)g_cfg.cull_fix; } },
    { "roomscale",        [] { return (double)g_cfg.roomscale; } },
    { "heightcal",        [] { return (double)g_cfg.height_cal; } },
    { "headblock",        [] { return (double)g_cfg.head_block; } },
    { "palettehook",      [] { return (double)g_cfg.palette_hook; } },
    { "palettewpn",       [] { return (double)g_cfg.palette_weapon; } },
    { "twohand",          [] { return (double)g_cfg.two_hand; } },
    { "armhide",          [] { return (double)g_cfg.arm_hide; } },
    { "reloadvr",         [] { return (double)g_cfg.reload_vr; } },
    { "slidevr",          [] { return (double)g_cfg.slide_vr; } },
    { "coophide",         [] { return (double)g_cfg.coop_hide; } },
    { "hidesolo",         [] { return (double)g_cfg.hide_solo; } },
    { "meleeswing",       [] { return (double)g_cfg.melee_swing; } },
    { "meleeleft",        [] { return (double)g_cfg.melee_left; } },
    { "grenadeswallow",   [] { return (double)g_cfg.grenade_swallow; } },
    { "holster",          [] { return (double)g_cfg.holster_enabled; } },
    { "holsterpollthrow", [] { return (double)g_cfg.holster_poll_throw; } },
    { "scope",            [] { return (double)g_cfg.scope_enabled; } },
    { "scopelens",        [] { return (double)g_cfg.scope_lens; } },
    { "wristhud",         [] { return (double)g_cfg.wrist_hud; } },
    { "forcetube",        [] { return (double)g_cfg.force_tube; } },
    { "vehcam",           [] { return (double)g_cfg.veh_cam; } },
    { "vehview",          [] { return (double)g_cfg.veh_view; } },
    { "vehiclewheel",     [] { return (double)g_cfg.vehicle_wheel; } },
    { "vehhidebody",      [] { return (double)g_cfg.veh_hide_body; } },
    { "stabilityfixes",   [] { return (double)g_cfg.stability_fixes; } },
};

static void effective_mirror_path(char* out, size_t cap) {
    sprintf_s(out, cap, "%s\\halo_vr_effective.txt", g_data_dir);
}

static void publish_effective_values() {
    if (g_data_dir[0] == 0) return;
    std::string text = "# Running value of every feature switch (all cfg layers + dependency rules). Written by\r\n"
                       "# halo_vr.dll for the settings menu; editing it changes nothing.\r\n";
    char line[96];
    for (const auto& e : kEffectiveKeys) {
        sprintf_s(line, sizeof(line), "%s=%g\r\n", e.key, e.get());
        text += line;
    }
    char path[MAX_PATH] = {0};
    effective_mirror_path(path, sizeof(path));
    write_text(path, text);
}

namespace {
int s_last_ref = -1, s_last_dev = -1;   // the menu status extras as last written
std::string s_last_height;
void publish_feature_list(const char* data_dir);
} // namespace

// Whether the two shipped catalogs exist: a missing catalog mirrors as EMPTY, which on its own
// looks exactly like "plugin not loaded"; the flags let the menu name the missing file instead.
// Plus the fork's auto-height line (height=<mode> <view height above game floor> m), empty while
// heightcal is off.
bool features_menu_status_changed() {
    const int ref_missing = (GetFileAttributesA(g_user_ref_path) == INVALID_FILE_ATTRIBUTES) ? 1 : 0;
    const int dev_missing = (GetFileAttributesA(g_dev_cfg_path) == INVALID_FILE_ATTRIBUTES) ? 1 : 0;
    return features_menu_status_line() != s_last_height || ref_missing != s_last_ref || dev_missing != s_last_dev;
}
std::string features_menu_status_text(const char* status) {
    const int ref_missing = (GetFileAttributesA(g_user_ref_path) == INVALID_FILE_ATTRIBUTES) ? 1 : 0;
    const int dev_missing = (GetFileAttributesA(g_dev_cfg_path) == INVALID_FILE_ATTRIBUTES) ? 1 : 0;
    const std::string height_line = features_menu_status_line();
    char extra[64];
    sprintf_s(extra, sizeof(extra), "refmissing=%d\r\ndevmissing=%d\r\n", ref_missing, dev_missing);
    s_last_ref = ref_missing;
    s_last_dev = dev_missing;
    s_last_height = height_line;
    std::string status_text = status;
    status_text += extra;
    if (!height_line.empty()) status_text += height_line + "\r\n";
    return status_text;
}

void features_publish(const char* data_dir) {
    publish_effective_values();
    publish_feature_list(data_dir);
}

namespace {
void publish_feature_list(const char* data_dir) {
    if (data_dir == nullptr || data_dir[0] == 0) return;
    std::string text = "# Feature list for the settings menu, generated by halo_vr.dll from its feature registry.\r\n"
                       "# Editing it changes nothing.\r\n";
    char line[256];
    for (int t = 1; t < kTierCount; ++t) {
        sprintf_s(line, sizeof(line), "tier|%s|%d|%s\r\n", kTierName[t], tier_on((Tier)t) ? 1 : 0,
                  s_switch_layer[t] >= 0 ? layer_code(s_switch_layer[t]) : "tier");
        text += line;
    }
    for (int i = 0; i < kCount; ++i) {
        const auto& r = kFeatures[i];
        text += "feature|";
        text += r.key;                        text += "|";
        text += std::to_string(r.on);         text += "|";
        text += kTierName[tier_index(r.tier)]; text += "|";
        text += r.group;                      text += "|";
        text += r.name;                       text += "|";
        text += r.desc;                       text += "|";
        text += r.subkeys;                    text += "|";
        text += r.needs;                      text += "|";
        text += std::to_string(r.get(g_cfg)); text += "|";
        text += source_code(i);               text += "\r\n";
    }
    char path[MAX_PATH] = {0};
    features_path(data_dir, path, sizeof(path));
    write_text(path, text);
}
} // namespace
void features_publish_if_missing(const char* data_dir) {
    // The running-values mirror is written by load_config on every real reload; this only covers
    // someone deleting data\ mid-session, which must not blank the menu until the next cfg edit.
    {
        char eff[MAX_PATH] = {0};
        effective_mirror_path(eff, sizeof(eff));
        if (GetFileAttributesA(eff) == INVALID_FILE_ATTRIBUTES) publish_effective_values();
    }
    if (data_dir == nullptr || data_dir[0] == 0) return;
    char path[MAX_PATH] = {0};
    features_path(data_dir, path, sizeof(path));
    if (GetFileAttributesA(path) == INVALID_FILE_ATTRIBUTES) publish_feature_list(data_dir);
}

void features_append_dev_reference(std::string& text) {
    static const Config kBuiltin{};
    if (!text.empty() && text.back() != '\n') text += "\r\n";
    text += "\r\n# =============================================================== FEATURE REGISTRY (developer reference, generated)\r\n"
            "# Every feature switch, generated from the registry in Features.cpp. Tier assignment is the\r\n"
            "# upstream author's call. Defaults: experimental and preview off, beta on, stable = built-in.\r\n"
            "# A master key set in any cfg file beats its tier; a tier switch turns a whole tier on or off,\r\n"
            "# and a single master key still beats its tier switch. Sub-settings act only while the master\r\n"
            "# is on.\r\n"
            "#\r\n";
    char line[512];
    for (int i = 0; i < kCount; ++i) {
        const auto& r = kFeatures[i];
        const int def = (r.tier == Tier::Stable) ? r.get(kBuiltin)
                                                 : (kTierDefaultOn[tier_index(r.tier)] ? r.on : 0);
        // Built as a string: a row's description and sub-setting list have no length limit, and a
        // fixed buffer that a long row overflows makes sprintf_s end the process.
        text += "# [";
        text += kTierName[tier_index(r.tier)];
        text += "] ";
        text += r.desc;
        if (r.subkeys[0]) { text += " Sub-settings: "; text += r.subkeys; }
        if (r.needs[0])   { text += ". Needs: ";       text += r.needs; }
        text += "\r\n#";
        text += r.key;
        text += "=";
        text += std::to_string(def);
        text += "\r\n";
    }
    for (int t = 1; t < kTierCount; ++t) {
        sprintf_s(line, sizeof(line), "# Tier switch: every %s feature whose own key is not set.\r\n#%s=%d\r\n",
                  kTierName[t], kTierSwitch[t], kTierDefaultOn[t] ? 1 : 0);
        text += line;
    }
}

void features_log_resolved() {
    uevr::API::get()->log_info("[Halo-CampE-UEVR] FEATURES: %d in the registry (key = running value [tier] source)", kCount);
    for (int i = 0; i < kCount; ++i) {
        const auto& r = kFeatures[i];
        uevr::API::get()->log_info("[Halo-CampE-UEVR] FEATURE %-17s = %d  [%s]  %s", r.key, r.get(g_cfg),
                             kTierName[tier_index(r.tier)], source_text(i).c_str());
    }
    features_log_runtime();
}

} // namespace halo
