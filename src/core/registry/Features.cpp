#include "core/registry/Features.hpp"
#include "core/registry/ArmHideDerive.hpp"
#include "core/registry/OwnedKeyDerive.hpp"

#include "Config.hpp"
#include "features/FeatureList.hpp"
#include "features/hooks/ConfigHooks.hpp"   // features_menu_status_line
#include "core/config/CfgRead.hpp"
#include "core/config/KeyAlias.hpp"
#include "core/fixes/HostFixes.hpp"   // stability_log_off_parity
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
// in the menu), its dev keys, the master keys it needs, and accessors for the master field.
//
// EVERY FORK KEY HAS EXACTLY ONE OWNER: a row's sub-settings, a row's dev keys, or kCoreKeys below.
// The split between the two lists is the split between the two shipped catalogs -- a sub-setting is
// documented in halo_vr_user_reference.txt and gets a menu row, a dev key is documented in
// halo_vr_dev.cfg and never gets one.
//
// WHERE A KEY IS PARSED IS NOT WHERE IT IS OWNED. A key a core service reads is parsed in core
// (core/CoreKeys.cpp, core/reload/ReloadKeys.cpp) so that it still parses in a build with that
// feature's folder removed, while the row below records which feature it acts for: vehfacing,
// vehfacingoff, vehlog and vehseatpub are core/UnitState's, listed under vehcam;
// holsterpollthrowlog is core/UnitState's evidence gate, listed under holsterpollthrow.
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
    const char* subkeys;   // its PLAYER SETTINGS: one player-catalog entry each, drawn under it in the menu
    const char* devkeys;   // its DEV KEYS: probes, dumps, logs and captured calibration. Dev catalog only,
                           // never a menu row -- see the split in features_append_dev_reference.
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
      "",
      "",
      "", FEATURE_BOOL(aim_reticule) },
    { "xrlayer",      1, Tier::Stable, "", "Headset-drawn reticule",
      "The reticule drawn by the headset itself, so the game's lighting never dims it.",
      "",
      "",
      "aimreticule", FEATURE_BOOL(xr_layer) },
    { "cutscenemono", 6, Tier::Stable, "", "Cutscene screen",
      "Cutscenes shown on one flat screen in front of you.",
      "cutscenesize",
      "",
      "", FEATURE_INT(cutscene_mono) },
    { "cullfix",      1, Tier::Stable, "", "Distant detail",
      "Keeps distant objects drawn at VR view distances.",
      "culldist",
      "",
      "", FEATURE_BOOL(cull_fix) },
    { "meleeswing",   1, Tier::Stable, "", "Melee by swinging",
      "Swing your weapon hand to melee.",
      "",
      "",
      "", FEATURE_BOOL(melee_swing) },
    { "holster",      1, Tier::Stable, "", "Holsters",
      "Reach over a shoulder to stow or draw a weapon, and to your chest for a grenade.",
      "",
      "",
      "", FEATURE_BOOL(holster_enabled) },
    { "twohand",      1, Tier::Stable, "", "Two-handed aiming",
      "Bring your other hand to the barrel and squeeze its grip to steady your aim.",
      "",
      "",
      "", FEATURE_INT(two_hand) },
    { "scope",        1, Tier::Stable, "", "Scope",
      "Squeeze the left trigger to zoom through a lens on your aim line.",
      "",
      "",
      "", FEATURE_BOOL(scope_enabled) },
    { "armhide",      1, Tier::Stable, "", "Hide the stock arms",
      "Hides the game's own first-person arms.",
      "",
      "",
      "", FEATURE_BOOL(arm_hide) },

    // ---- Contributed from the fork (Experimental until the author promotes them). Where a fork
    // feature would collide with one of his, it has its own master key.
    { "palettewpn",   1, Tier::Experimental, "Weapon", "Weapon follows your hand",
      "The weapon you see is placed on your hand and the aim follows the drawn barrel. Replaces the standard weapon placement while on. After turning it on or off, exit to the main menu and load back in for the change to take effect.",
      "aimbore,palettebuildgate,palettecalibkey,palettecamlead,palettehidearms,palettemeshconst,"
      "palettepubframe,paletterolltrim,palettesocketfix,palettetwohandagreefull,palettetwohandagreemin,palettetwohandmarker,"
      "palettetwohandmax,palettetwohandmin,palettewpncalibkey",
      "aimboreaxis,camleadall,compgain,complatch,complatchms,fpanimkill,fpmeshlog,fppin,fpscalefix,"
      "judderlog,liftyaw,palaimcalibver,palaimfix,palaimoffpitch,palaimoffyaw,paletteaimdirectwrite,"
      "paletteaimreticulefresh,palettebank,palettebarrellock,palettecam,palettecamsmooth,palettefinal,"
      "palettehook,palettehooktest,palettelatch,palettelatchms,palettelerp,palettelocal,"
      "palettemeshconstgate,palettepoke,palettepokeamt,palettepokecount,palettepokenode,paletteposelatch,"
      "palettescan,paletteslidewatch,paletteslidezonepriority,palettesocketfixsamples,palettesocketfixtol,"
      "palettesync,palettetwohandblendms,"
      "palettetwohandhaptic,palettetwohandlog,palettetwohandmarkercolor,palettetwohandmarkerscale,"
      "palettetwohandrad,palettewatch,palettewpnoffx,palettewpnoffy,palettewpnoffz,palettewpnscale,"
      "palgripfix,palrender,palsniff,palstep,palstepctx,palstepsrc,palwpnfix,pinuevrframe,posefilter,"
      "posefilterbeta,posefilterdcut,posefiltermin,posefilterrbeta,posefreeze,revclamp,revclampdps,stomplog,"
      "termlog,tremor,tremorhz,tremorq,wpnerrlog",
      "", FEATURE_BOOL(palette_weapon) },
    { "aimreticulestamp", 1, Tier::Experimental, "Weapon", "Reticule placed every frame",
      "The headset-drawn reticule is placed each frame on exactly where your shots go.",
      "",
      "",
      "palettewpn,aimreticule,xrlayer", FEATURE_INT(aim_reticule_stamp) },
    { "scopelens",    1, Tier::Experimental, "Weapon", "Scope lens on the weapon",
      "A magnifying lens in the scope housing of the scoped weapons. The standard scope stands down while on.",
      "scopeev,scopeeyedist,scopelenslumen,scopereticletint,scopertfmt,scopeseptrans,scopesfflags,scopeshowflags,"
      "scopesource,scopetint,scopetonecurve",
      "scopeabtest,scopecamfwd,scopecvardump,scopehz,scopepp,scopeprobe,scopereticle,scopereticlescale,"
      "scoperollfix,scoperound,scopewpn",
      "scope", FEATURE_BOOL(scope_lens) },
    { "reloadvr",     1, Tier::Experimental, "Reload", "Manual reload",
      "Drop the magazine, fetch a fresh one from your belt and push it into the gun.",
      "gripexclusive,reloadakmimic,reloadakmute,reloadanimrate,reloadcoophide,reloadhandoff,reloadhandrot,"
      "reloadhidearms,reloadhidesolo,reloadholdfire,reloadlift,reloadmagbelt,reloadmagoffw,reloadmagpick,reloadresetholds,"
      "reloadroomanchor,reloadseat,reloadslidems,reloadstate,reloadstatedeath,reloadstatedrop,"
      "reloadstatehide,reloadstatelevel,reloadstatesave,reloadstatewaitms,reloadwellmarker,zonesnap",
      "akfnregister,akfnsetlisteners,akfnsetposition,akfnsetrtpc,akfnsetswitch,akfnunregister,aklog,"
      "akmimic4event,akmutenames,akpostrva,akrtpc,akrtpcglobal,akrtpcrestore,akrtpcvalue,akstack,akvtcount,"
      "akvtdump,akvtglobal,ammoscrub,ammoseq,animdump,animobjs,animseqset,animvars,animvarset,coopauto,"
      "coopmaskms,coopstopat,magdrop,magdropms,magdump,maghide,maghidename,reloadanimms,reloadanimmscoop,"
      "reloadaudiodump,reloadframe,reloadholdstate,reloadinsert,reloadinsertdone,reloadinsertmode,"
      "reloadinsertsign,reloadmaskms,reloadmutems,reloadmutevariant,reloadpauseanim,reloadpressat,"
      "reloadpressms,reloadpressmscoop,reloadshotgunlog,reloadskipweapons,reloadstatelog,reloadstepsound,"
      "reloadstepvariant,reloadstepvia,reloadvrlog,reloadwellfwd,reloadwellmarkerscale,reloadwwisedump,"
      "reserveoff,roundsoff,wellmarkercolor,wpnammodump,zonehandrel",
      "", FEATURE_BOOL(reload_vr) },
    { "slidevr",      1, Tier::Experimental, "Reload", "Rack the slide",
      "Rack the slide, pump or charging handle with your other hand.",
      "slidefire,slidefireback,slidefireentry,slideoff,slidepartrotaxis,slideradius,slidetravel,"
      "slidezoneback",
      "slidealways,slidebones,slidechamber,slidechamberweapons,slidecopy,slidecopyalign,slidecopyaxis,"
      "slidecopybone,slidecopyclass,slidecopyfollower,slidecopyfp,slidecopyhide,slidecopyleader,"
      "slidecopymode,slidecopynonanite,slidecopyplay,slidecopyrel,slidecopyroot,slidecopyseq,slidecopysign,"
      "slidecopysweep,slidecopytest,slidefararray,slidefarcm,slidefirefwd,slidefirestate,slidehidemat,"
      "slidehidematslot,slidehidepbo,slidehidesection,slidelockreload,slidelog,slidemarker,slidemarkercolor,"
      "slidemarkersize,slidemontage,slidemorph,slidepart,slidepartaxis,slidepartbind,slidepartbone,"
      "slidepartdropmode,slidepartframe,slideparthide,slidepartkids,slidepartmagdrop,slidepartmaghide,"
      "slidepartmat,slidepartopendeg,slidepartorphans,slidepartpivot,slidepartrotdeg,slidepartshadow,"
      "slidepartsign,slideparttest,slidepartui,slidephantom,slideseq,slideseqback,slideseqfwd,slideseqsweep,"
      "slidesign,slideslot,slideundoreload,slideweapons,slidezone",
      "", FEATURE_BOOL(slide_vr) },
    { "meleeleft",    1, Tier::Experimental, "Melee and grenades", "Punch with your other hand",
      "Your other hand can melee too, on its own swing thresholds.",
      "meleeleftcooldown,meleeleftext,meleelefthold,meleeleftmaxreach,meleeleftmaxspeed,meleeleftreach,"
      "meleeleftspeed,meleelefttau",
      "meleeleftdisp,meleeleftlog,meleeleftshotdist,meleeleftshotms",
      "meleeswing", FEATURE_BOOL(melee_left) },
    { "grenadeswallow", 1, Tier::Experimental, "Melee and grenades", "Grenades from the pouches only",
      "The left face button stops throwing grenades; grenades come from your chest pouches.",
      "",
      "grenadecode,grenadeswallowlog",
      "", FEATURE_INT(grenade_swallow) },
    { "holsterpollthrow", 1, Tier::Experimental, "Melee and grenades", "Grenade throw on release",
      "A grenade leaves your hand the instant you open your grip, and the pouches show your real grenade counts.",
      "",
      "holsterpollthrowbackdate,holsterpollthrowdump,holsterpollthrowgripmaskl,holsterpollthrowgripmaskr,"
      "holsterpollthrowhand,holsterpollthrowinstant,holsterpollthrowlog,holsterpollthrowspeed",
      "holster,blamangles", FEATURE_BOOL(holster_poll_throw) },
    { "wristhud",     1, Tier::Experimental, "HUD", "Wrist HUD",
      "Shield, weapon and grenade readouts on your forearm, and the motion tracker on your wrist.",
      "wristblipcolorother,wristhudclasses,wristhudclassesr,wristhudgap,wristhudoff,wristhudoffr,"
      "wristhudplacement,wristhudrot,wristhudrotr,wristradar,wristradarblip,wristradargain",
      "wristblipbytes,wristblipcolor,wristblipdump,wristblipname,wristhudammotext,wristhudblend,"
      "wristhudblendr,wristhuddraw,wristhudgainr,wristhudgapr,wristhudscale,wristhudtrigger,wristhudwpnammo,"
      "wristhudwpnanchor,wristhudwpnfallback,wristhudwpngap,wristhudwpngrenade,wristhudwpnlog,"
      "wristhudwpnscale,wristhudwpnshield,wristhudwpntracker,wristradaraimsign,wristradarcenter,"
      "wristradarflip,wristradarlog,wristradarrot,wristradartest,wristradartilt,wristtrackerdump,"
      "wristtrackermid",
      "", FEATURE_BOOL(wrist_hud) },
    { "forcetube",    1, Tier::Experimental, "Haptics", "ForceTube gunstock",
      "A kick in the ForceTube gunstock on every round you fire.",
      "forcetubekick,forcetuberadius",
      "forcetubechannel,forcetubefirems",
      "", FEATURE_BOOL(force_tube) },
    { "vehcam",       1, Tier::Experimental, "Vehicles", "Vehicle seat camera",
      "A first-person view from your seat in vehicles.",
      "vehcamboomtau,vehcamguard,vehcamhullcheck,vehcamoff,vehfacing,vehhidebody,vehview",
      "vehanchor,vehboomorder,vehcamanchor,vehcamguardspeed,vehcamhulldead,vehcamscale,vehcamsrc,"
      "vehcamstalems,vehfacingbias,vehfacingoff,vehlog,vehseatdirect,vehseatpub,vehviewflat",
      "blamangles", FEATURE_INT(veh_cam) },
    { "vehiclewheel", 1, Tier::Experimental, "Vehicles", "Steering wheel",
      "Hands on the steering wheel. Steering is not sent to the vehicle yet.",
      "vehwheelsteersign",
      "vehwheelgrip,vehwheelhand,vehwheellock,vehwheelmarker,vehwheelpos,vehwheelrad,vehwheeltilt",
      "blamangles", FEATURE_INT(vehicle_wheel) },
    { "roomscale",    1, Tier::Experimental, "Roomscale", "Roomscale",
      "Walk around your play space and your steps move the Spartan.",
      "roomscaledz,roomscalegain,roomscalemin,roomscalethrottle,roomscalethrottleysign",
      "roomscaledead,roomscaleff,roomscalelat,roomscaleleash,roomscalelog,roomscalemaxspeed,roomscalepulse,"
      "roomscalespeed,roomscalestanddown,roomscalestick,roomscalethrottleoff,roomscalethrottleoff2,"
      "roomscalethrprobe,roomscalethrspeed",
      "blamangles", FEATURE_BOOL(roomscale) },
    { "heightcal",    1, Tier::Experimental, "Roomscale", "Auto height",
      "Your view height above the game floor follows your head above the real floor, so a real crouch lowers it.",
      "heightkey,heightmode,heightsample,heightsrc,heighttrim",
      "heightautoseat,heightband,heightbipedfeet,heightbipedscale,heightestep,heighteye,heightholdms,"
      "heightlog,heightpawnfeet,heightscale,heightseatbelow,heightseatdwell,heightseattarget,heightslew,"
      "heighttracechannel,heighttracemax,heightwindow",
      "", FEATURE_INT(height_cal) },
    { "headblock",    1, Tier::Experimental, "Roomscale", "Head block",
      "Keeps your head out of walls when you lean into them.",
      "headblockradius",
      "headblockchannel,headblocklean,headblocklog,headblockrelease",
      "", FEATURE_INT(head_block) },
    { "stabilityfixes", 1, Tier::Experimental, "Stability", "Stability fixes",
      "Guards for the base mod: nav marker fault quarantine, fault recovery and stale rig guard, head tracking dropout gate, stick mode exit after a death, UI and reticle sweep throttles, asset load failure memo, reticle re-assert, early compositor reticule tick, teardown order, aim-hand melee holster veto and aim pin, two-handed hold release on a gesture reset, menu command file poll gate, holster marker tint and minimum throw speed.",
      "",
      "stabilitygrenminthrow,stabilityholstermarkercolor,stabilityturnlog,stabilitywidgetlog",
      "", FEATURE_BOOL(stability_fixes) },
};

#undef FEATURE_BOOL
#undef FEATURE_INT

// CORE KEYS: the fork keys that belong to no single feature, so every fork key has exactly one
// owner -- a row above, or this list. A key is here when its reader is a core service that runs for
// whichever consumer keyed it, or when more than one feature reads it:
//   bobcancel/bobtau/boblog          core/CameraBob, the camera bob cancel
//   cutscenegrab                     core/dev/CutsceneDump
//   magrender                        core/WeaponObject, the drawn-magazine pass
//   markertint                       core/MarkerFaces -- the tint gate for EVERY marker: the holster
//                                    pouches (stabilityfixes), the reload well, the rack part and
//                                    the placement's grab dot. FOUR features, so it is not any one
//                                    feature's sub-setting.
//   moveprobe / stealextra           core/fixes/HostFixes, two gates on the author's own code
//   palettewpnlog                    read by palettewpn AND by the wrist HUD's one-shot widget log
//   slidehook / slidenode            core/WeaponObject, the node the rack drives
//   wpnnode*                         core/WeaponObject node probes, also read by the reload engine
const char* const kCoreKeys =
    "bobcancel,boblog,bobtau,cutscenegrab,magrender,markertint,moveprobe,palettewpnlog,slidehook,"
    "slidenode,stealextra,wpnnodecopyscan,wpnnodedump,wpnnodepoke,wpnnodepokeamt";

constexpr int kCount = (int)(sizeof(kFeatures) / sizeof(kFeatures[0]));
constexpr int kTierCount = 4;

const char* const kTierName[kTierCount]   = { "stable", "beta", "preview", "experimental" };
const char* const kTierSwitch[kTierCount] = { nullptr, "tierbeta", "tierpreview", "tierexperimental" };
constexpr bool    kTierDefaultOn[kTierCount] = { true, true, false, false };

// The kConfigFiles index that last set each master key this load (-1 = no file set it).
int  s_layer = 0;
bool s_layer_open = false;   // load_config's layer walk is running (features_layer)
int  s_key_layer[kCount];
int  s_switch_layer[kTierCount];
bool s_switch_on[kTierCount];
// Shared infrastructure the resolver turns on for its consumers, unless a file sets it.
int  s_palette_hook_layer = -1;
int  s_pose_latch_layer   = -1;
// The fork's reload log, so a player who switched it OFF explicitly keeps it off even with the
// author's reloadlog on. Without this the raise below is one-way.
int  s_reload_vr_log_layer = -1;
// The author's rig, arm hide mode and bone list, left alone by the arm hide and rig derivation when a
// file sets them.
int  s_arm_hide_mode_layer = -1;
int  s_arm_hide_bone_layer = -1;
int  s_rig_layer = -1;
// The author's keys a fork feature owns while it is on (OwnedKeyDerive.cpp): which cfg layer set each
// one, and the resolution each was last logged with (OWNED_UNRESOLVED before the first load).
int  s_owned_layer[OWNED_KEY_COUNT];
int  s_owned_logged[OWNED_KEY_COUNT] = {};
// The reloadhidearms resolution last logged (approach * 16 + reason), -1 before the first load.
int  s_hide_arms_logged = -1;
// The palette weapon's rig and arm hide resolution last logged, -1 before the first load.
int  s_pal_hide_logged = -1;

int tier_index(Tier t) { return (int)t; }

const char* layer_source(int layer) {
    switch (layer) {
    case 0:  return "halo_vr.cfg";
    case 1:  return "user override";
    case 2:  return "halo_vr_dev.cfg";
    case 3:  return "halo_vr_calib.cfg";
    case 4:  return "halo_vr_weapons.cfg";
    case 5:  return "halo_vr_palette_calib.cfg";
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

char g_pal_calib_path[MAX_PATH] = {0};

void features_begin_load() {
    s_layer = 0;
    for (int i = 0; i < kCount; ++i) s_key_layer[i] = -1;
    for (int t = 0; t < kTierCount; ++t) { s_switch_layer[t] = -1; s_switch_on[t] = false; }
    s_palette_hook_layer = -1;
    s_pose_latch_layer = -1;
    s_reload_vr_log_layer = -1;
    s_arm_hide_mode_layer = -1;
    s_arm_hide_bone_layer = -1;
    s_rig_layer = -1;
    for (int i = 0; i < OWNED_KEY_COUNT; ++i) s_owned_layer[i] = -1;
}

void features_config_reload_begin() { cfg_reload_begin(); }

void features_set_layer(int layer) { s_layer = layer; s_layer_open = true; }

int features_layer() { return s_layer_open ? s_layer : -1; }

void features_note_key(const char* key, const char* val) {
    if (key == nullptr) return;
    // The same translation the fork's parser does, so a retired name records its layer under the
    // current one and the menu's source column is right whichever spelling the file used.
    key = key_current_name(key);
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
    if (_stricmp(key, "paletteposelatch") == 0)   { s_pose_latch_layer = s_layer; return; }
    if (_stricmp(key, "reloadvrlog") == 0)        { s_reload_vr_log_layer = s_layer; return; }
    if (_stricmp(key, "armhidemode") == 0) { s_arm_hide_mode_layer = s_layer; return; }
    if (_stricmp(key, "armhidebone") == 0) { s_arm_hide_bone_layer = s_layer; return; }
    if (_stricmp(key, "rig") == 0)         { s_rig_layer = s_layer; return; }
    for (int i = 0; i < OWNED_KEY_COUNT; ++i) {
        if (_stricmp(key, owned_key_name(i)) == 0) { s_owned_layer[i] = s_layer; return; }
    }
}

void features_apply() {
    s_layer_open = false;
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

    // THE FORK'S RELOAD EVIDENCE ANSWERS TO EITHER LOG SWITCH. About forty fork lines moved from
    // the author's reloadlog onto the fork's own reloadvrlog when the manual reload became a
    // feature. reloadlog is HIS key, parsed by HIS parser, so it cannot go in the alias table --
    // that table holds fork names only, and claiming one of his would be the very thing the rule
    // forbids. The translation goes the other way instead: his switch also raises OURS, which is a
    // write to a fork key and never to his, so a player told "turn the reload log on" gets the
    // fork's evidence whichever of the two switches they reach for.
    //
    // CORRECTION to the reasoning this shipped with, which claimed reloadlog is "what the settings
    // menu writes": it is not. The menu's Record the reload gesture writes reloadvrlog
    // (profile/scripts/halo_vr_settings.lua), so the everyday switch is OURS and his is the one a
    // troubleshooting instruction names. That leaves the direction of the raise right and the old
    // one-way form wrong: with reloadlog on in a cfg file, turning the menu switch off wrote
    // reloadvrlog=0 and nothing happened, because the raise ran afterwards and overrode it.
    //
    // EITHER OR, with the explicit value winning: if any cfg file set reloadvrlog, that is the
    // player's answer and it stands, on or off. Only when nobody set it does his switch raise it.
    if (s_reload_vr_log_layer < 0 && g_cfg.reload_log) g_cfg.reload_vr_log = true;

    // SHARED INFRASTRUCTURE, turned on for the features that need it unless a file sets it.
    //   The weapon placement (palettewpn = armdriver mode 3) moves the drawn weapon through the
    //   first-person pose hook and composes against the pose latch's stamped intent. Both act only
    //   while mode 3 owns (the arbiter installs the hook, the latch checks the mode), so with the
    //   placement off they stay at 0 and the author's pose path is untouched.
    if (s_palette_hook_layer < 0) g_cfg.palette_hook = g_cfg.palette_weapon ? 1 : 0;
    if (s_pose_latch_layer < 0)   g_cfg.pose_latch   = g_cfg.palette_weapon ? 2 : 0;

    // THE PALETTE WEAPON'S RIG AND ARM HIDE, AND MANUAL RELOAD'S ARM HIDE (ArmHideDerive.cpp). Each
    // derives only the author's keys no cfg file sets, and load_config rebuilds the Config before this,
    // so a feature switched off gives back the layered values with nothing to undo here. The hides
    // themselves are applied and released by his arm hide pass (Arms.cpp) through the feature hooks.
    {
        ArmHideLayers set;
        set.rig         = s_rig_layer >= 0;
        set.armhidemode = s_arm_hide_mode_layer >= 0;
        set.armhidebone = s_arm_hide_bone_layer >= 0;
        for (int i = 0; i < kCount; ++i)
            if (_stricmp(kFeatures[i].key, "armhide") == 0) set.armhide = s_key_layer[i] >= 0;
        const ArmHideResolution r = arm_hide_derive(g_cfg, set);

        // ONE LINE PER ENGAGE OR REVERT of the palette weapon's rig and arm hide, with the reason.
        const int pal_stamp = (r.rig_reason * 16 + r.pal_reason) * 16 + r.pal_active;
        if (pal_stamp != s_pal_hide_logged) {
            const bool was_engaged = (s_pal_hide_logged >= 0)
                && ((s_pal_hide_logged % 16) != 0 || (s_pal_hide_logged / 256) == 3);
            const bool engaged = r.rig_off || r.pal_active != 0;
            const bool quiet = (s_pal_hide_logged < 0 && r.rig_reason == 1);   // startup with the palette weapon simply off
            s_pal_hide_logged = pal_stamp;
            if (!quiet) {
                uevr::API::get()->log_info("[Halo-CampE-UEVR] ARMHIDE/RIG palettewpn %s: rig %d (%s); arms %s (palettehidearms=%d, approach %d, armhide %d, armhidemode %d, armhidebone %s)",
                    engaged ? "engages" : (was_engaged ? "reverts" : "stands down"),
                    g_cfg.rig_enabled ? 1 : 0, arm_hide_rig_reason_text(r.rig_reason),
                    arm_hide_pal_reason_text(r.pal_reason), g_cfg.palette_hide_arms, r.pal_active,
                    g_cfg.arm_hide ? 1 : 0, g_cfg.arm_hide_mode, g_cfg.arm_hide_bone);
            }
        }

        const int stamp = r.reload_active * 16 + r.reload_reason;
        if (stamp != s_hide_arms_logged) {
            const int prev_active = (s_hide_arms_logged < 0) ? 0 : (s_hide_arms_logged / 16);
            const bool quiet = (s_hide_arms_logged < 0 && r.reload_reason == 1);   // startup with manual reload simply off
            s_hide_arms_logged = stamp;
            if (!quiet) {
                uevr::API::get()->log_info("[Halo-CampE-UEVR] ARMHIDE reloadhidearms=%d %s: %s (approach %d, armhide %d, armhidemode %d)",
                    g_cfg.reload_hide_arms, r.reload_active != 0 ? "engages" : (prev_active != 0 ? "releases" : "stands down"),
                    arm_hide_reload_reason_text(r.reload_active, r.reload_reason), r.reload_active,
                    g_cfg.arm_hide ? 1 : 0, g_cfg.arm_hide_mode);
            }
        }
    }

    // A FORK SETTING THAT FOLLOWS ITS FEATURE'S MASTER KEY (OwnedKeyDerive.cpp). Fork keys only:
    // the author's keys are never written here. A key a cfg layer sets is the player's and is left
    // alone, and load_config rebuilds the Config before this, so a feature switched off gives back
    // exactly the layered values.
    {
        OwnedKeyLayers set;
        for (int i = 0; i < OWNED_KEY_COUNT; ++i) set.set[i] = s_owned_layer[i] >= 0;
        const OwnedKeyResolution r = owned_keys_derive(g_cfg, set);

        // ONE LINE PER DERIVE OR REVERT, with the feature, the key, the value and the reason.
        for (int i = 0; i < OWNED_KEY_COUNT; ++i) {
            if (r.state[i] == s_owned_logged[i]) continue;
            const bool was_derived = (s_owned_logged[i] == OWNED_DERIVED);
            const bool quiet = (s_owned_logged[i] == OWNED_UNRESOLVED && r.state[i] == OWNED_FEATURE_OFF);
            s_owned_logged[i] = r.state[i];
            if (quiet) continue;   // startup with the owning feature simply off
            uevr::API::get()->log_info("[Halo-CampE-UEVR] OWNEDKEY %s %s = %s, %s: %s",
                owned_key_feature(i), owned_key_name(i), r.value[i],
                (r.state[i] == OWNED_DERIVED) ? "derived" : (was_derived ? "reverts" : "not derived"),
                owned_key_reason(i, r.state[i]));
        }
    }

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
    { "armhidemode",      [] { return (double)g_cfg.arm_hide_mode; } },
    { "rig",              [] { return (double)g_cfg.rig_enabled; } },
    { "reloadvr",         [] { return (double)g_cfg.reload_vr; } },
    { "reloadseat",       [] { return (double)g_cfg.reload_seat_dist; } },
    { "slidevr",          [] { return (double)g_cfg.slide_vr; } },
    { "reloadcoophide",         [] { return (double)g_cfg.coop_hide; } },
    { "reloadhidesolo",         [] { return (double)g_cfg.hide_solo; } },
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
        text += source_code(i);               text += "|";
        // Field 12, after the fields the menu reads: its dev keys. The menu draws a row for a
        // SUB-SETTING, never for one of these, so appending them cannot add a row to anyone's menu.
        text += r.devkeys;                    text += "\r\n";
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
            "#\r\n"
            "# Each feature's keys are listed twice over: its SUB-SETTINGS are the ones a player chooses,\r\n"
            "# one entry each in halo_vr_user_reference.txt and one row each in the settings menu; its DEV\r\n"
            "# KEYS are probes, dumps, logs and captured calibration, listed here only. Both act only while\r\n"
            "# the feature is on. Every fork key is under exactly one feature, or in the core list at the end.\r\n"
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
        // The dev keys are listed but never offered as a menu row: they are probes, dumps, logs and
        // captured calibration, and they act only while this feature is on, exactly as its
        // sub-settings do. Their catalog entries are in this file, under this feature's heading.
        if (r.devkeys[0]) { text += "#   dev keys: "; text += r.devkeys; text += "\r\n"; }
    }
    text += "# Core keys. Read by a core service, or by more than one feature, so they belong to no\r\n"
            "# single feature and act whenever their reader runs.\r\n#   ";
    text += kCoreKeys;
    text += "\r\n";
    for (int t = 1; t < kTierCount; ++t) {
        sprintf_s(line, sizeof(line), "# Tier switch: every %s feature whose own key is not set.\r\n#%s=%d\r\n",
                  kTierName[t], kTierSwitch[t], kTierDefaultOn[t] ? 1 : 0);
        text += line;
    }
    key_alias_append_reference(text);
}

void features_log_resolved() {
    uevr::API::get()->log_info("[Halo-CampE-UEVR] FEATURES: %d in the registry (key = running value [tier] source)", kCount);
    for (int i = 0; i < kCount; ++i) {
        const auto& r = kFeatures[i];
        uevr::API::get()->log_info("[Halo-CampE-UEVR] FEATURE %-17s = %d  [%s]  %s", r.key, r.get(g_cfg),
                             kTierName[tier_index(r.tier)], source_text(i).c_str());
    }
    features_log_runtime();
    // With stabilityfixes off, every gate it puts on the author's code must reproduce HIS
    // expression, and several of his are ungated -- so the off-value alone proves nothing and the
    // parity block prints his expression beside each one (core/fixes/HostFixes.cpp).
    if (!g_cfg.stability_fixes) stability_log_off_parity();
}

} // namespace halo
