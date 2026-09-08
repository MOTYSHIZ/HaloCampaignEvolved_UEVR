#include "Config.hpp"
#include "Math.hpp"
// wpn_calib_load(): captured per-weapon deltas are a third source feeding the same table.
#include "WeaponCalib.hpp"
#include "ScopeOffset.hpp"
// palettearm_parse_key(): the palette arm driver owns its own keys, in its own folder.
#include "palettearm/PaletteArm.hpp"
// two_hand_parse_key(): the two-handed hold owns its own keys.
#include "TwoHandAim.hpp"
// interact_line_parse_key(): the grab guide owns its own keys.
#include "InteractLine.hpp"
// Logging only, and deliberately in the .cpp: Config.hpp is included by everything and stays
// free of the engine API. The bind-capture result is worth a log line -- it is the one record of
// what mask a player's controller actually sent, which is the first thing a support report needs.
#include "uevr/API.hpp"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>

namespace halo {
using uevr::API;

Config g_cfg{};

// Every smoothing setting used to be a fraction of the gap closed PER CALL, which is not a filter
// setting so much as a filter setting entangled with the frame rate -- the same number smoothed
// about three times harder at 30 fps than at 90. They are time constants now (see ema_alpha), but
// old config files still in the wild carry the fractions, so those keys are still accepted and
// converted here rather than being ignored into a silent default.
//
// The conversion needs the frame rate the old value was tuned at, which nothing recorded. 90 Hz is
// the assumption: it is what this game runs at in the headset it was tuned in. A file tuned at a
// very different rate converts to a time constant that FEELS the same as it did there -- which is
// the point -- but is not what its author would pick today. Move such a file to the *ms keys.
static constexpr float LEGACY_REF_HZ = 90.0f;
static float alpha_to_tau_ms(float alpha) {
    alpha = clampf(alpha, 0.0f, 1.0f);
    if (alpha >= 0.999f) return 0.0f;   // "take the new value whole" in the old scheme
    return clampf(-1000.0f / (LEGACY_REF_HZ * std::log(1.0f - alpha)), 0.0f, 500.0f);
}

// The scope key family, hoisted out of the main parse chain: MSVC counts every `else if` toward
// its 128-deep block limit (fatal C1061), and the chain hit it when these were added inline.
// Returns true when the key belonged to the scope and was consumed.
static bool parse_scope_key(const char* key, double v) {
    if      (_stricmp(key, "scope")       == 0) g_cfg.scope_enabled  = (v != 0.0);
    else if (_stricmp(key, "scopemount")  == 0) g_cfg.scope_mount    = (int)clampf((float)v, 0.0f, 1.0f);
    else if (_stricmp(key, "scopeshape")  == 0) g_cfg.scope_shape    = (int)clampf((float)v, 0.0f, 1.0f);
    else if (_stricmp(key, "scopecalibkey")==0) g_cfg.scope_calib_key = (int)v;
    // Ceiling 300, not a "sane magnification": the scope's effectiveness depends on the pane
    // SCALE, so a small far pane legitimately wants very high magnification to matter.
    else if (_stricmp(key, "scopezoom")   == 0) g_cfg.scope_zoom     = clampf((float)v, 1.05f, 300.0f);
    else if (_stricmp(key, "scoperes")    == 0) g_cfg.scope_rt_size  = (int)clampf((float)v, 128.0f, 2048.0f);
    else if (_stricmp(key, "scopediv")    == 0) g_cfg.scope_div      = (int)clampf((float)v, 1.0f, 8.0f);
    else if (_stricmp(key, "scopedist")   == 0) g_cfg.scope_dist     = clampf((float)v, 25.0f, 400.0f);
    else if (_stricmp(key, "scopecamdist")== 0) g_cfg.scope_cam_dist = clampf((float)v, 25.0f, 600.0f);
    else if (_stricmp(key, "scopesize")   == 0) g_cfg.scope_size     = clampf((float)v, 5.0f, 100.0f);
    else if (_stricmp(key, "scoperight")  == 0) g_cfg.scope_right    = clampf((float)v, -100.0f, 100.0f);
    else if (_stricmp(key, "scopeup")     == 0) g_cfg.scope_up       = clampf((float)v, -100.0f, 100.0f);
    // CEILING RAISED 8 -> 8192 (2026-08-25). The old ceiling silently capped every brightness
    // test at 8x. The hosted-widget RETICULE needed aim_widget_tint 1024 x aim_widget_gain 5 --
    // roughly 5000x -- to survive the same scene pre-exposure and tonemapper this pane feeds
    // through, so 8 was never in the right order of magnitude for this material family. A knob
    // whose useful range sits outside its own clamp reads as "the setting does nothing".
    else if (_stricmp(key, "scopebright") == 0) g_cfg.scope_bright   = clampf((float)v, 0.0f, 8192.0f);
    else if (_stricmp(key, "scoperotp")   == 0) g_cfg.scope_rot_p    = (float)v;
    else if (_stricmp(key, "scoperoty")   == 0) g_cfg.scope_rot_y    = (float)v;
    else if (_stricmp(key, "scoperotr")   == 0) g_cfg.scope_rot_r    = (float)v;
    else if (_stricmp(key, "scopecamroll")== 0) g_cfg.scope_cam_roll = (float)v;
    else if (_stricmp(key, "scopecamtrack")==0) g_cfg.scope_cam_track = (int)clampf((float)v, 0.0f, 1.0f);
    else if (_stricmp(key, "scopecamorigin")==0)
        g_cfg.scope_cam_origin = (int)clampf((float)v, 0.0f, 1.0f);
    else if (_stricmp(key, "scopecamlock")== 0) g_cfg.scope_cam_lock  = (int)clampf((float)v, 0.0f, 1.0f);
    else if (_stricmp(key, "scopeexposure")==0) g_cfg.scope_exposure  = clampf((float)v, 0.0f, 64.0f);
    // STOPS, and the range is deliberately signed and wide: this is the only pre-tonemap
    // brightness control, and clamping it narrow is how a lever gets written off as "no effect"
    // when it was simply never allowed to reach the right order of magnitude.
    else if (_stricmp(key, "scopeautoexposurebias")==0)
        g_cfg.scope_autoexposure_bias = clampf((float)v, -16.0f, 16.0f);
    // -1 = leave the engine's forced-off state alone; 0..3 = force a method. See Config.hpp.
    else if (_stricmp(key, "scopelumen")==0)
        g_cfg.scope_lumen = (int)clampf((float)v, -1.0f, 3.0f);
    else if (_stricmp(key, "scopeppgrade")==0) g_cfg.scope_pp_grade = (v != 0.0);
    else if (_stricmp(key, "scopegain")==0) g_cfg.scope_gain = clampf((float)v, 0.0f, 256.0f);
    else if (_stricmp(key, "scopelayerfollowpane")==0)
        g_cfg.scope_layer_follow_pane = (int)clampf((float)v, 0.0f, 1.0f);
    else if (_stricmp(key, "scoperetproject")==0)
        g_cfg.scope_ret_project = (int)clampf((float)v, 0.0f, 1.0f);
    else if (_stricmp(key, "scopesocketaudit")==0)
        g_cfg.scope_socket_audit = (int)clampf((float)v, 0.0f, 1.0f);
    else if (_stricmp(key, "scopesocketwait")==0)
        g_cfg.scope_socket_wait = (int)clampf((float)v, 0.0f, 1.0f);
    else if (_stricmp(key, "scoperetroll")==0)
        g_cfg.scope_ret_roll = (int)clampf((float)v, 0.0f, 2.0f);
    else if (_stricmp(key, "scoperetflipx")==0)
        g_cfg.scope_ret_flip_x = (int)clampf((float)v, 0.0f, 1.0f);
    else if (_stricmp(key, "scoperetflipy")==0)
        g_cfg.scope_ret_flip_y = (int)clampf((float)v, 0.0f, 1.0f);
    else if (_stricmp(key, "scopemask")==0) g_cfg.scope_mask = (int)clampf((float)v, 0.0f, 1.0f);
    else if (_stricmp(key, "scopemaskaspect")==0)
        g_cfg.scope_mask_aspect = clampf((float)v, 0.2f, 5.0f);
    else if (_stricmp(key, "scopemaskfeather")==0)
        g_cfg.scope_mask_feather = clampf((float)v, 0.0f, 1.0f);
    else if (_stricmp(key, "scopeaa")     == 0) g_cfg.scope_aa       = (int)clampf((float)v, -1.0f, 4.0f);
    else if (_stricmp(key, "scopethresh") == 0) g_cfg.scope_thresh   = clampf((float)v, 0.10f, 0.95f);
    else if (_stricmp(key, "scopebase")   == 0) g_cfg.scope_base_fov = clampf((float)v, 5.0f, 120.0f);
    else if (_stricmp(key, "scopesrc")    == 0) g_cfg.scope_capture_src = (int)clampf((float)v, 0.0f, 9.0f);
    else if (_stricmp(key, "scopeeat")    == 0) g_cfg.scope_eat_lt   = (v != 0.0);
    else if (_stricmp(key, "scopedevray") == 0) g_cfg.scope_dev_ray  = (v != 0.0);
    else if (_stricmp(key, "scopeforce")  == 0) g_cfg.scope_force    = (v != 0.0);
    else if (_stricmp(key, "scopefpdepth")== 0) g_cfg.scope_fp_depth = (v != 0.0);
    else if (_stricmp(key, "scopeparent")== 0) g_cfg.scope_parent = (int)clampf((float)v, 0.0f, 2.0f);
    else if (_stricmp(key, "scopecapmode")== 0) g_cfg.scope_cap_mode = (int)clampf((float)v, 0.0f, 1.0f);
    else if (_stricmp(key, "scopetest")   == 0) g_cfg.scope_test_obj  = (int)clampf((float)v, 0.0f, 6.0f);
    else if (_stricmp(key, "scopetestdist")==0) g_cfg.scope_test_dist = clampf((float)v, 30.0f, 5000.0f);
    else if (_stricmp(key, "scopetestsize")==0) g_cfg.scope_test_size = clampf((float)v, 0.02f, 20.0f);
    else if (_stricmp(key, "scopetesttex")== 0) g_cfg.scope_test_tex  = (int)clampf((float)v, 0.0f, 2.0f);
    else if (_stricmp(key, "scenert")     == 0) g_cfg.scene_rt_probe  = (v != 0.0);
    else if (_stricmp(key, "scopertscan") == 0) g_cfg.scope_rt_scan   = (v != 0.0);
    else if (_stricmp(key, "scopeblit")    == 0) g_cfg.scope_blit      = (v != 0.0);
    else if (_stricmp(key, "scopebloom")   == 0) g_cfg.scope_bloom     = clampf((float)v, 0.0f, 32.0f);
    else if (_stricmp(key, "scopemainfamily")==0) g_cfg.scope_main_family = (int)clampf((float)v, -1.0f, 1.0f);
    else if (_stricmp(key, "scopemainres") == 0) g_cfg.scope_main_res   = (int)clampf((float)v, -1.0f, 1.0f);
    else if (_stricmp(key, "scopemaincam") == 0) g_cfg.scope_main_cam   = (int)clampf((float)v, -1.0f, 1.0f);
    else if (_stricmp(key, "scopeppcopy")  == 0) g_cfg.scope_pp_copy   = (v != 0.0);
    else if (_stricmp(key, "cutsceneblit") == 0) g_cfg.cutscene_blit   = (v != 0.0);
    else if (_stricmp(key, "cutscenemono") == 0) g_cfg.cutscene_mono   = (v != 0.0);
    else if (_stricmp(key, "cutsceneblitfill") == 0) g_cfg.cutscene_blit_fill = clampf((float)v, 0.2f, 1.0f);
    else if (_stricmp(key, "scopeblitmag") == 0) g_cfg.scope_blit_mag  = clampf((float)v, 1.05f, 32.0f);
    else if (_stricmp(key, "scopeblitsize")== 0) g_cfg.scope_blit_size = clampf((float)v, 0.05f, 1.0f);
    else if (_stricmp(key, "scopeblitx")   == 0) g_cfg.scope_blit_x    = clampf((float)v, 0.0f, 1.0f);
    else if (_stricmp(key, "scopeblity")   == 0) g_cfg.scope_blit_y    = clampf((float)v, 0.0f, 1.0f);
    else if (_stricmp(key, "scopepersist") == 0) g_cfg.scope_persist   = (int)clampf((float)v, 0.0f, 1.0f);
    else if (_stricmp(key, "scopecamcut")  == 0) g_cfg.scope_cam_cut   = (int)clampf((float)v, 0.0f, 1.0f);
    else if (_stricmp(key, "scopeppweight")== 0) g_cfg.scope_pp_weight = clampf((float)v, -1.0f, 1.0f);
    else if (_stricmp(key, "scopedof")     == 0) g_cfg.scope_dof       = clampf((float)v, 0.0f, 64.0f);
    else if (_stricmp(key, "scopedoffocus")== 0) g_cfg.scope_dof_focus = clampf((float)v, 10.0f, 1000000.0f);
    // ---- the scope as an OpenXR quad composition layer (src\ScopeLayer.hpp) --------------------
    // 0 off, 1 gun-mounted, 2 [dev] view-basis self-check, 3 [dev] head-oriented control.
    else if (_stricmp(key, "scopelayer")   == 0) g_cfg.scope_layer     = (int)clampf((float)v, 0.0f, 3.0f);
    // Ceiling 63, not XRLAYER_SLOTS: Config.cpp deliberately knows nothing about the layer module,
    // and the adapter refuses an out-of-range slot with the real bound in the log. Clamping to a
    // constant copied from another module's header is how the two silently drift apart.
    else if (_stricmp(key, "scopelayerslot")==0) g_cfg.scope_layer_slot = (int)clampf((float)v, 0.0f, 63.0f);
    // Centimetres, like every other distance key here. MCC's field-proven fit is 15.9 / 5.0 /
    // -5.8 / 21.6 cm; the ranges are wide enough to sweep a placement without a rebuild.
    else if (_stricmp(key, "scopelayerwidth")==0) g_cfg.scope_layer_width = clampf((float)v, 1.0f, 200.0f);
    else if (_stricmp(key, "scopelayerfwd") == 0) g_cfg.scope_layer_fwd   = clampf((float)v, -100.0f, 400.0f);
    else if (_stricmp(key, "scopelayerright")==0) g_cfg.scope_layer_right = clampf((float)v, -100.0f, 100.0f);
    else if (_stricmp(key, "scopelayerup")  == 0) g_cfg.scope_layer_up    = clampf((float)v, -100.0f, 100.0f);
    else if (_stricmp(key, "scopelayerroll")== 0) g_cfg.scope_layer_roll  = (int)clampf((float)v, 0.0f, 2.0f);
    else if (_stricmp(key, "scopelayerrolltrim")==0)
        g_cfg.scope_layer_roll_trim = clampf((float)v, -180.0f, 180.0f);
    else if (_stricmp(key, "scopelayerhidepane")==0) g_cfg.scope_layer_hide_pane = (int)clampf((float)v, 0.0f, 1.0f);
    // Lives in THIS helper rather than the main chain purely for the compiler: that chain already
    // hit MSVC's 128-level nesting limit (C1061) once, which is why the scope keys were hoisted
    // here in the first place. Adding to it is what breaks the build; adding here is free.
    else if (_stricmp(key, "scopeshieldcensus")==0) g_cfg.shield_census = (v != 0.0);
    else return false;
    return true;
}


// ---------------------------------------------------------------- live config
// KILL SWITCH FIRST, tuning second. There is no plugin on_message callback in this API version,
// so without the file there would be no off switch at all: if aim misbehaved with a headset on,
// the only recourse would be killing the game. That is not an acceptable state to hand to a
// person in VR, so the file is re-read every ~2 s and `enabled=0` neutralises the stick on the
// next tick.
//
// It doubles as live tuning: floor/full/max/dead/xdist can be changed without a rebuild.
char g_cfg_path[MAX_PATH] = {0};

// THE USER'S OWN SETTINGS (halo_vr_user.cfg). Never shipped: a release zip OVERWRITES the files
// it contains on upgrade (UEVR's Import Config merges file-by-file), so this file is where
// settings that must SURVIVE updates live -- same lifecycle as halo_vr_calib.cfg. Created on
// first run as a SHORT pointer template: the catalog of available keys lives in the shipped
// halo_vr_user_reference.txt (refreshed by every update, never parsed), and users copy the
// lines they want across -- so this file holds only deliberate changes and can never go stale.
char g_user_cfg_path[MAX_PATH] = {0};

// DEV / TROUBLESHOOTING OVERRIDES (halo_vr_dev.cfg). SHIPS with the mod as a fully-commented
// catalog of the internal knobs; packaging asserts it is inert (no uncommented keys), so its
// presence changes nothing. Uncommenting a key is the dev/support workflow -- and updates
// overwrite the file, so experiments cannot linger past a release.
char g_dev_cfg_path[MAX_PATH] = {0};
uint32_t g_cfg_check_tick = 0;
uint32_t g_cfg_load_gen   = 0;

// CENTIMETRES IN, METRES STORED. Every distance CONFIG KEY is centimetres, matching Unreal's own
// world unit; the FIELDS behind a few of them stay in metres because their consumers work in VR
// pose space. Converting here, once, is the point -- a conversion spread across call sites is a
// conversion one site will eventually miss.
//
// LEGACY GUARD. These keys were metres until 2026-08-12, so a value that is IMPLAUSIBLE AS
// CENTIMETRES is a config written before the change. `cm_floor` is the smallest value the key can
// take and still mean anything: below it, a cm reading cannot be what anyone intended, so the
// value is scaled instead of obeyed. Obeying it is the failure mode this whole change exists to
// prevent -- hmdleashlat=0.30 read as 0.3 cm is a HARD leash, the exact opposite of the 30 cm
// bubble asked for, and it would present as "the setting does nothing" rather than as a wrong
// number. Recorded, never silent: the tick logs it once (this file makes no UEVR API calls).
static float cm_to_m(double v, float cm_floor, float lo_cm, float hi_cm, const char* key) {
    float cm = (float)v;
    const float mag = (cm < 0.0f) ? -cm : cm;
    if (mag > 0.0f && mag < cm_floor) {
        g_cfg.legacy_units_key = key;   // a string literal, so no lifetime question
        g_cfg.legacy_units_val = cm;
        cm *= 100.0f;
    }
    return clampf(cm, lo_cm, hi_cm) * 0.01f;
}

// TRUE if the file exists and has at least one ACTIVE (uncommented) key line. With every layer
// shipped or template-created, mere file existence says nothing -- the plugin logs the layers
// that are actually doing something. Line test mirrors parse_config_file exactly.
// NAME the uncommented keys, do not merely count them.
//
// The bare has-keys check produced a warning that said a file "has uncommented keys" without
// saying WHICH. That is most of the way to useless in a file several sessions share: on
// 2026-08-25 a stale scopesrc=8 sat in the live halo_vr_dev.cfg for hours, silently beating the
// user's own halo_vr_user.cfg on every reload, while the warning dutifully fired every launch and
// named nothing. Hours went into explaining the resulting darkness as a rendering problem. Had the
// log read "scopesrc, scopepersist", it would have been a one-line read.
//
// Fills `out` with a comma-separated key list (truncated with an ellipsis if it does not fit) and
// returns how many were found.
int config_file_list_uncommented_keys(const char* path, char* out, size_t out_sz) {
    if (out != nullptr && out_sz > 0) out[0] = '\0';
    FILE* f = nullptr;
    if (fopen_s(&f, path, "rb") != 0 || f == nullptr) return 0;
    char line[256];
    int count = 0;
    size_t used = 0;
    while (fgets(line, sizeof(line), f) != nullptr) {
        if (line[0] == '#' || line[0] == '\r' || line[0] == '\n') continue;
        const char* eq = strchr(line, '=');
        if (eq == nullptr) continue;
        ++count;
        if (out == nullptr || used + 1 >= out_sz) continue;
        // Key name only -- the VALUE is deliberately omitted. A calibration line can be long and
        // the point of this list is "what is in force", answerable at a glance.
        size_t klen = (size_t)(eq - line);
        while (klen > 0 && (line[klen - 1] == ' ' || line[klen - 1] == '\t')) --klen;
        if (klen == 0) continue;
        const size_t need = klen + (used > 0 ? 2u : 0u);
        if (used + need + 6 >= out_sz) {
            if (used + 6 < out_sz) {
                memcpy(out + used, ", ...", 5);
                used += 5;
                out[used] = '\0';
            }
            continue;
        }
        if (used > 0) { out[used++] = ','; out[used++] = ' '; }
        memcpy(out + used, line, klen);
        used += klen;
        out[used] = '\0';
    }
    fclose(f);
    return count;
}

bool config_file_has_uncommented_keys(const char* path) {
    FILE* f = nullptr;
    if (fopen_s(&f, path, "rb") != 0 || f == nullptr) return false;
    char line[256];
    bool found = false;
    while (!found && fgets(line, sizeof(line), f) != nullptr) {
        if (line[0] == '#' || line[0] == '\r' || line[0] == '\n') continue;
        if (strchr(line, '=') != nullptr) found = true;
    }
    fclose(f);
    return found;
}

// Create halo_vr_user.cfg if it does not exist ("wbx" = exclusive, so an existing file is never
// touched). Deliberately a SHORT pointer, not a copy of the catalog: a snapshot of the catalog
// in a never-overwritten file would go stale the first time a release adds a key, while the
// shipped reference next door is refreshed by every update. All comments, so creating it
// changes no behaviour.
void ensure_user_cfg_template() {
    FILE* f = nullptr;
    if (fopen_s(&f, g_user_cfg_path, "wbx") != 0 || f == nullptr) return;
    fprintf(f,
        "# halo_vr_user.cfg -- YOUR settings for the Halo: Campaign Evolved VR mod.\r\n"
        "#\r\n"
        "# The mod reads this file live (~2 s), and updates NEVER touch it: whatever you set\r\n"
        "# here overrides the built-in defaults and survives every update.\r\n"
        "#\r\n"
        "# The full list of available settings -- with comments and each one's default -- is in\r\n"
        "# halo_vr_user_reference.txt, next to this file (kept up to date by every update).\r\n"
        "# Copy the lines you want from there into here, remove the leading #, and set your\r\n"
        "# values.\r\n"
        "#\r\n"
        "# To RESET everything to the defaults, delete this file. The reset applies from the\r\n"
        "# next launch; a value already applied this session stays until then.\r\n"
        "#\r\n"
        "# Example (remove the # to use):\r\n"
        "#snapdeg=30\r\n");
    fclose(f);
}

// ---------------------------------------------------------------- in-game settings-menu bridge
// UEVR's built-in Lua host sandboxes script file I/O to <profile>\data\, so the settings menu
// (profile scripts\halo_vr_settings.lua) cannot touch halo_vr_user.cfg directly. The plugin
// bridges on the same ~2 s poll as the config reload:
//   * MIRRORS the settings catalog and halo_vr_user.cfg into data\ for the script to read;
//   * APPLIES the script's command file (data\halo_vr_menu_set.txt: `key=value` upserts an
//     override, `-key` removes one) into the real halo_vr_user.cfg, then deletes it.
// The menu is therefore just another writer of the user file -- the same live reload applies
// its changes, and what it sets survives updates exactly like a hand edit.
char g_data_dir[MAX_PATH]          = {0};
char g_user_ref_path[MAX_PATH]     = {0};   // profile-root catalog (the mirror's source)
char g_menu_cmd_path[MAX_PATH]     = {0};
char g_user_mirror_path[MAX_PATH]  = {0};
char g_ref_mirror_path[MAX_PATH]   = {0};
char g_dev_mirror_path[MAX_PATH]   = {0};
char g_calib_mirror_path[MAX_PATH] = {0};
char g_status_path[MAX_PATH]       = {0};

std::atomic<int> g_menu_calib_mode{0};
std::atomic<bool> g_hand_calib_held{false};

// Bind capture (see the ownership note in Config.hpp). The key buffer is written only here, on
// the game thread, and only while the capture is disarmed.
std::atomic<int>      g_bind_capture{0};
std::atomic<int>      g_bind_captured{0};
std::atomic<uint64_t> g_bind_capture_deadline{0};
char g_bind_capture_key[32] = {0};

// The cfg keys a capture is allowed to write. An ALLOW-LIST, not a validity check: the command
// file is written by a Lua script inside the profile, so this is not a security boundary -- it
// is there so a typo'd or stale menu (an older script against a newer plugin, or the reverse)
// cannot inject an arbitrary key into the player's cfg and leave them with a file whose stray
// line survives every update and never does anything.
static const char* const BIND_KEYS[] = {
    "bindcrouch", "bindmelee", "bindreload", "bindscope", "binddpadshift", "bindequip",
    "mapfrom", "mapto", "maprstickdown", "mapmenuback", "grenadefrom", "grenadeaction",
    "rgripmask",
};

static bool is_bind_key(const char* key) {
    for (size_t i = 0; i < _countof(BIND_KEYS); ++i)
        if (_stricmp(key, BIND_KEYS[i]) == 0) return true;
    return false;
}

// Which calibration keys belong to which gesture, for the menu's per-gesture reset. Kept next
// to the bridge that consumes them; write_calib_file() is the authority on what each gesture
// persists.
static const char* const POSE_CALIB_KEYS[] = {
    "calibver", "grip", "gripyaw", "griproll",
    "dirgrip", "dirgripyaw", "dirgriproll", "diroffx", "diroffy", "diroffz",
    "offx", "offy", "offz", "pivauto", "pivx", "pivy", "pivz",
};
static const char* const AIM_CALIB_KEYS[] = { "aimcalibver", "aimoffyaw", "aimoffpitch" };
// The SUPPORT-HAND gesture. Its stamp goes in the list too: stripping the line and leaving
// `handfixver` behind would leave a stamp certifying nothing, which is the file equivalent of a
// dangling reference and reads in a diff as though a calibration is still present.
static const char* const HAND_CALIB_KEYS[] = { "handfixver", "handfix" };
// The Delete-key SCOPE PLACEMENT gesture. Must match what write_calib_file() emits for the scope
// block below (search "scopemount=1"), including scopemount itself: stripping the placement and
// leaving the mount MODE behind is the worst of both, because mount=1 reinterprets the frame the
// remaining defaults are expressed in. Reported 2026-09-07 by a player who commented the six
// placement lines out by hand to recover a lost pane and found it did not help -- scopemount=1
// was still active underneath.
static const char* const SCOPE_CALIB_KEYS[] = {
    "scopemount", "scopedist", "scoperight", "scopeup", "scoperotp", "scoperoty", "scoperotr",
};

static bool read_text_file(const char* path, std::string& out) {
    out.clear();
    FILE* f = nullptr;
    if (fopen_s(&f, path, "rb") != 0 || f == nullptr) return false;
    char buf[4096];
    size_t n = 0;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) out.append(buf, n);
    fclose(f);
    return true;
}

static bool write_text_file(const char* path, const std::string& text) {
    FILE* f = nullptr;
    if (fopen_s(&f, path, "wb") != 0 || f == nullptr) return false;
    fwrite(text.data(), 1, text.size(), f);
    fclose(f);
    return true;
}

// Replace the ACTIVE `key=` line in the user-cfg text, remove it, or append it under a marker
// naming the menu as the writer. Commented lines never match, so hand-written docs are kept.
static void upsert_user_key(std::string& text, const std::string& key, const std::string& line, bool remove) {
    size_t pos = 0;
    while (pos < text.size()) {
        size_t eol = text.find('\n', pos);
        const size_t line_end = (eol == std::string::npos) ? text.size() : eol + 1;
        size_t len = ((eol == std::string::npos) ? text.size() : eol) - pos;
        if (len > 0 && text[pos + len - 1] == '\r') --len;   // compare without the CR
        if (len > key.size() && _strnicmp(text.c_str() + pos, key.c_str(), key.size()) == 0 &&
            text[pos + key.size()] == '=') {
            if (remove) text.erase(pos, line_end - pos);
            else        text.replace(pos, len, line);
            return;
        }
        pos = line_end;
    }
    if (remove) return;   // no such override; nothing to do
    static const char* MENU_MARKER = "# ---- Set from the in-game settings menu (UEVR overlay) ----";
    if (!text.empty() && text.back() != '\n') text += "\r\n";
    if (text.find(MENU_MARKER) == std::string::npos) {
        text += "\r\n";
        text += MENU_MARKER;
        text += "\r\n";
    }
    text += line;
    text += "\r\n";
}

// TRUE if the (possibly stripped) calibration text still carries any active key.
static bool text_has_active_keys(const std::string& text) {
    size_t pos = 0;
    while (pos < text.size()) {
        size_t eol = text.find('\n', pos);
        if (eol == std::string::npos) eol = text.size();
        const char c = text[pos];
        if (c != '#' && c != '\r' && c != '\n') {
            if (text.find('=', pos) != std::string::npos && text.find('=', pos) < eol) return true;
        }
        pos = eol + 1;
    }
    return false;
}

// Remove one gesture's keys from halo_vr_calib.cfg (the menu's per-gesture 'x'). The shipped
// values in halo_vr.cfg stand back up on the same poll's load_config. If nothing active is
// left, delete the file outright so "no calib file = shipped fit" stays literally true.
static void strip_calib_keys(const char* const* keys, size_t count) {
    std::string calib;
    if (!read_text_file(g_calib_path, calib)) return;
    for (size_t i = 0; i < count; ++i) upsert_user_key(calib, keys[i], "", true);
    if (text_has_active_keys(calib)) write_text_file(g_calib_path, calib);
    else                             DeleteFileA(g_calib_path);
}

// Defined with load_config's change gate further down; the bridge reuses it so both pollers
// share one definition of "changed".
uint64_t cfg_file_stamp(const char* path);

int menu_bridge_tick() {
    if (g_data_dir[0] == 0) return 0;

    int applied = 0;
    std::string cmd;
    if (read_text_file(g_menu_cmd_path, cmd)) {
        std::string user, dev;
        bool user_loaded = false, user_changed = false;
        bool dev_loaded = false,  dev_changed = false;
        size_t pos = 0;
        while (pos < cmd.size()) {
            size_t eol = cmd.find_first_of("\r\n", pos);
            if (eol == std::string::npos) eol = cmd.size();
            std::string line = cmd.substr(pos, eol - pos);
            pos = eol + 1;
            while (!line.empty() && (line.back() == ' ' || line.back() == '\t')) line.pop_back();
            size_t b = 0;
            while (b < line.size() && (line[b] == ' ' || line[b] == '\t')) ++b;
            if (b > 0) line.erase(0, b);
            if (line.empty() || line[0] == '#') continue;

            // One-shot calibration actions.
            if (line == "calib:pose")      { g_menu_calib_mode.store(1, std::memory_order_relaxed); ++applied; continue; }
            if (line == "calib:aim")       { g_menu_calib_mode.store(2, std::memory_order_relaxed); ++applied; continue; }
            if (line == "calib:hand")      { g_menu_calib_mode.store(3, std::memory_order_relaxed); ++applied; continue; }
            // Arm the NEXT scope-calibration release to land in the held weapon's trim rather
            // than the global fit. Not a g_menu_calib_mode: the scope gesture has its own hold in
            // Scope.cpp, so this only chooses the DESTINATION of a capture that gesture already makes.
            if (line == "calib:wpnscope")  { scope_offset_arm(true);  ++applied; continue; }
            if (line == "calib:wpnscopeoff"){ scope_offset_arm(false); ++applied; continue; }
            if (line == "calib:scopebase")   { scope_base_arm(true);   ++applied; continue; }
            if (line == "calib:scopebaseoff"){ scope_base_arm(false);  ++applied; continue; }
            if (line == "calib:off")       { g_menu_calib_mode.store(0, std::memory_order_relaxed); ++applied; continue; }
            if (line == "calibreset:all")  { DeleteFileA(g_calib_path); ++applied; continue; }
            if (line == "calibreset:pose") { strip_calib_keys(POSE_CALIB_KEYS, _countof(POSE_CALIB_KEYS)); ++applied; continue; }
            if (line == "calibreset:aim")  { strip_calib_keys(AIM_CALIB_KEYS,  _countof(AIM_CALIB_KEYS));  ++applied; continue; }
            if (line == "calibreset:hand") { strip_calib_keys(HAND_CALIB_KEYS, _countof(HAND_CALIB_KEYS)); ++applied; continue; }
            // GLOBAL scope placement -> back to the built-in fit in Config.hpp. Same shape as the
            // three above; the keys are the scope block write_calib_file() emits.
            if (line == "calibreset:scope") { strip_calib_keys(SCOPE_CALIB_KEYS, _countof(SCOPE_CALIB_KEYS)); ++applied; continue; }
            // PER-WEAPON scope trim for whatever is in hand right now -> back to the global fit.
            // Deliberately a different file and a different mechanism: the global fit lives in
            // halo_vr_calib.cfg as plain keys, while per-weapon trims are rows in the machine-owned
            // halo_vr_weapons.cfg, so this goes through the table + its own writer rather than
            // strip_calib_keys. Returns false when there was nothing to clear, which is not an
            // error -- it logs why and the menu button simply does nothing.
            if (line == "calibreset:wpnscope") { scope_offset_clear_current(); ++applied; continue; }

            // ---- BIND CAPTURE. `bind:capture=<cfgkey>` arms; `bind:capture=off` disarms.
            //
            // Generic in the KEY rather than an action enum: the plugin needs to know only "put
            // the next press in this cfg key", so adding an action later is a menu-side change
            // and a parse line, with nothing here to keep in sync.
            if (line.rfind("bind:capture=", 0) == 0) {
                const std::string arg = line.substr(13);
                if (arg == "off" || arg.empty()) {
                    g_bind_capture.store(0, std::memory_order_release);
                    g_bind_captured.store(0, std::memory_order_relaxed);
                    g_bind_capture_deadline.store(0, std::memory_order_relaxed);
                    g_bind_capture_key[0] = 0;
                } else if (is_bind_key(arg.c_str()) && g_bind_capture.load(std::memory_order_acquire) == 0) {
                    // Key first, THEN arm -- the hook reads the buffer only once it sees armed.
                    strncpy_s(g_bind_capture_key, arg.c_str(), _TRUNCATE);
                    g_bind_captured.store(0, std::memory_order_relaxed);
                    g_bind_capture_deadline.store(
                        GetTickCount64() + (uint64_t)(g_cfg.bind_capture_ms > 0 ? g_cfg.bind_capture_ms : 20000),
                        std::memory_order_relaxed);
                    g_bind_capture.store(1, std::memory_order_release);
                }
                ++applied;
                continue;
            }

            // `dev:` routes into halo_vr_dev.cfg (experiments that updates overwrite); everything
            // else is a halo_vr_user.cfg override, exactly as if the player typed it there.
            std::string* text = &user;
            bool* loaded = &user_loaded;
            bool* changed = &user_changed;
            if (line.rfind("dev:", 0) == 0) {
                line.erase(0, 4);
                if (line.empty()) continue;
                text = &dev; loaded = &dev_loaded; changed = &dev_changed;
            }
            if (!*loaded) { read_text_file(text == &dev ? g_dev_cfg_path : g_user_cfg_path, *text); *loaded = true; }

            if (line[0] == '-') {
                if (line.size() > 1) { upsert_user_key(*text, line.substr(1), "", true); ++applied; *changed = true; }
                continue;
            }
            const size_t eq = line.find('=');
            if (eq == std::string::npos || eq == 0) continue;
            upsert_user_key(*text, line.substr(0, eq), line, false);
            ++applied;
            *changed = true;
        }
        if (user_changed) write_text_file(g_user_cfg_path, user);
        if (dev_changed)  write_text_file(g_dev_cfg_path, dev);
        DeleteFileA(g_menu_cmd_path);   // consumed -- this is also the script's "done" signal
    }

    // ---- COMPLETED / EXPIRED BIND CAPTURE.
    //
    // Runs outside the command block on purpose: the press lands on some LATER poll than the one
    // that armed it (the player has to close the menu first), so this is the only place that can
    // see it. The hook has already disarmed itself; all that is left is the file write it must
    // not do.
    const int captured = g_bind_captured.load(std::memory_order_acquire);
    if (captured != 0) {
        if (g_bind_capture_key[0] != 0) {
            char line[64];
            sprintf_s(line, sizeof(line), "%s=0x%04X", g_bind_capture_key, (unsigned)(captured & 0xFFFF));
            std::string user;
            read_text_file(g_user_cfg_path, user);
            upsert_user_key(user, g_bind_capture_key, line, false);
            write_text_file(g_user_cfg_path, user);
            API::get()->log_info("[Halo-CampE-UEVR] BIND captured %s = 0x%04X",
                                 g_bind_capture_key, (unsigned)(captured & 0xFFFF));
            ++applied;
        }
        g_bind_captured.store(0, std::memory_order_relaxed);
        g_bind_capture_deadline.store(0, std::memory_order_relaxed);
        g_bind_capture_key[0] = 0;
    } else if (g_bind_capture.load(std::memory_order_acquire) != 0) {
        // Expiry is judged HERE as well as in the hook, because the hook only runs while the game
        // is polling the pad: an arm left running with the overlay open would otherwise sit armed
        // forever and eat the first press after the menu closes, long past its deadline.
        const uint64_t deadline = g_bind_capture_deadline.load(std::memory_order_relaxed);
        if (deadline != 0 && GetTickCount64() > deadline) {
            g_bind_capture.store(0, std::memory_order_release);
            g_bind_capture_deadline.store(0, std::memory_order_relaxed);
            g_bind_capture_key[0] = 0;
            API::get()->log_info("[Halo-CampE-UEVR] BIND capture timed out with no press");
        }
    }

    // Mirrors, rewritten only when the source changed -- and since the v0.3.1 freeze audit,
    // "changed" is decided by a STAT, not by reading both sides back every poll. The old loop
    // did 8 synchronous opens (~130 KB, halo_vr_dev.cfg alone is ~56 KB) every ~2 s on the game
    // thread: exactly the I/O class whose measured 143.9/108.7 ms contention stalls got
    // load_config its gate (see the DO-NOT-RE-READ note there). Now an unchanged poll costs
    // four GetFileAttributesEx calls plus four existence checks, no opens.
    //
    // Semantics preserved deliberately:
    //   * A missing SOURCE mirrors as EMPTY -- load-bearing for halo_vr_calib.cfg, whose
    //     deletion (reset) must reach the menu. cfg_file_stamp() returns 0 for a missing file,
    //     which differs from any real write time, so the delete itself is a "change".
    //   * A missing MIRROR is re-created even when the source is unchanged (the dst existence
    //     check below) -- someone clearing data\ mid-session must not kill the menu until the
    //     next cfg edit.
    //   * The dst read-back+compare is gone: on a source change the mirror is simply rewritten.
    struct { const char* src; const char* dst; } mirrors[] = {
        { g_user_ref_path, g_ref_mirror_path },
        { g_user_cfg_path, g_user_mirror_path },
        { g_dev_cfg_path,  g_dev_mirror_path },
        { g_calib_path,    g_calib_mirror_path },
    };
    static uint64_t s_mirror_stamp[_countof(mirrors)] = { ~0ull, ~0ull, ~0ull, ~0ull };
    std::string src;
    for (size_t i = 0; i < _countof(mirrors); ++i) {
        const auto& m = mirrors[i];
        const uint64_t st = cfg_file_stamp(m.src);
        const bool dst_missing = (GetFileAttributesA(m.dst) == INVALID_FILE_ATTRIBUTES);
        if (st == s_mirror_stamp[i] && !dst_missing) continue;
        read_text_file(m.src, src);   // failure leaves src empty, which is exactly what we mirror
        write_text_file(m.dst, src);
        s_mirror_stamp[i] = st;
    }

    // Status for the menu (armed calibration mode + armed bind capture), written on change only.
    // `bindkey` is what the menu needs to show WHICH row is waiting -- without it a reopened menu
    // can say "something is armed" but not what, which is the state that gets a press eaten.
    static int  s_last_status = -1;
    static int  s_last_bind   = -1;
    static int  s_last_hand   = -1;
    static char s_last_bindkey[32] = {0};
    const int mode = g_menu_calib_mode.load(std::memory_order_relaxed);
    const int barmed = g_bind_capture.load(std::memory_order_relaxed);
    // Whether the SUPPORT-HAND gesture would do anything if armed: 0 not configured (the menu hides
    // the entry), 1 configured but nothing posed yet, 2 ready. Published so the menu can refuse to
    // arm a mode the running configuration will never look at -- see palettearm_support_hand_ready()
    // for why the two positive states are distinct. Two field reads and an atomic load, ~2 s poll.
    const int hready = palettearm_support_hand_ready();
    // Per-weapon scope trim armed? Published so the Script UI button can SHOW that it is armed
    // rather than the player having to remember they pressed it. The plugin is the authority --
    // the arm is consumed by the capture, so a button that tracked its own click would keep
    // claiming "armed" after the gesture had already spent it.
    static int  s_last_scopearm = -1;
    static int  s_last_scopebase = -1;
    const int scopearm  = scope_offset_armed() ? 1 : 0;
    const int scopebase = scope_base_armed()   ? 1 : 0;
    if (mode != s_last_status || barmed != s_last_bind || hready != s_last_hand ||
        scopearm != s_last_scopearm || scopebase != s_last_scopebase ||
        strncmp(s_last_bindkey, g_bind_capture_key, sizeof(s_last_bindkey)) != 0) {
        s_last_status = mode;
        s_last_bind   = barmed;
        s_last_hand   = hready;
        s_last_scopearm = scopearm;
        s_last_scopebase = scopebase;
        strncpy_s(s_last_bindkey, g_bind_capture_key, _TRUNCATE);
        char status[320];   // widened for scopearm= and scopebasearm=
        sprintf_s(status, sizeof(status),
                  "calibmode=%d\r\nbindcapture=%d\r\nbindkey=%s\r\nhandready=%d\r\nscopearm=%d\r\n",
                  mode, barmed, g_bind_capture_key, hready, scopearm);
        write_text_file(g_status_path, status);
    }
    return applied;
}

// Recreate halo_vr.cfg if it is missing. Since the config split that file carries ONLY the
// shipped calibration data, and those values are not compiled in (they are measured data with
// file-borne schema stamps -- see the calib_ver notes in Config.hpp), so a missing file cannot
// be reconstructed here. Write a stub that says what happened and how to recover; until then
// the neutral built-in fit applies, which plays fine and self-heals on the first End/Page Down
// calibration.
void write_default_config() {
    FILE* f = nullptr;
    if (fopen_s(&f, g_cfg_path, "wb") != 0 || f == nullptr) return;
    fprintf(f,
        "# halo_vr.cfg -- SHIPPED CALIBRATION DATA for the Halo: Campaign Evolved VR plugin.\r\n"
        "#\r\n"
        "# This file was missing, so the plugin recreated it EMPTY. The shipped Quest Touch\r\n"
        "# calibration that normally lives here is gone -- reinstall the mod to restore it, or\r\n"
        "# simply calibrate yourself: in a mission, hold End (pose match), then Page Down (aim\r\n"
        "# ray). Results are saved to halo_vr_calib.cfg, which overrides this file anyway.\r\n"
        "#\r\n"
        "# Settings do NOT live here: defaults are built into the plugin, halo_vr_user.cfg is\r\n"
        "# where to change them, and halo_vr_user_reference.txt lists everything available.\r\n");
    fclose(f);
}


// Second half of the key table. Split out because MSVC hits "blocks nested too deeply"
// (C1061) on one long else-if chain, and this table keeps growing.
// BLAM diagnostic/override keys, in their own function with early returns.
// MSVC C1061 (blocks nested too deeply) has now bitten this file TWICE: one long else-if chain
// overflows, and merely moving keys to a second chain just moves the overflow. Returning early
// flattens the nesting instead of relocating it -- put new blam keys here.
static bool parse_blam_key(const char* key, const char* val, double v) {
    if (_stricmp(key, "blampitch")   == 0) { g_cfg.blam_pitch_off = (float)v; return true; }
    if (_stricmp(key, "blamdump")    == 0) { g_cfg.blam_dump      = (int)v; return true; }
    if (_stricmp(key, "blamobj")     == 0) { g_cfg.blam_obj       = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "blamobjlen")   == 0) { g_cfg.blam_obj_len   = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "blamfind")      == 0) { g_cfg.blam_find      = (int)v; return true; }
    if (_stricmp(key, "blamscan")      == 0) { g_cfg.blam_scan      = (int)v; return true; }
    if (_stricmp(key, "blamscan2")     == 0) { g_cfg.blam_scan2     = (int)v; return true; }
    if (_stricmp(key, "blamscan3")     == 0) { g_cfg.blam_scan3     = (int)v; return true; }
    if (_stricmp(key, "blamnode")      == 0) { g_cfg.blam_node      = (int)v; return true; }
    if (_stricmp(key, "blamangles")    == 0) { g_cfg.blam_angles    = (int)v; return true; }
    if (_stricmp(key, "blamanglesysign") == 0) { g_cfg.blam_angles_ysign = (int)v; return true; }
    if (_stricmp(key, "blamfault") == 0) { g_cfg.blam_fault = (int)v; return true; }
    if (_stricmp(key, "blamlayout") == 0) { g_cfg.blam_layout = (int)v; return true; }
    if (_stricmp(key, "vraccel")       == 0) { g_cfg.vr_accel       = (int)v; return true; }
    if (_stricmp(key, "aimdeadbeat")   == 0) { g_cfg.aim_deadbeat   = (float)v; return true; }
    if (_stricmp(key, "aimstat")       == 0) { g_cfg.aim_stat       = (int)v; return true; }
    if (_stricmp(key, "aimtargetsmoothms") == 0) { g_cfg.aim_target_smooth_ms = (float)v; return true; }
    return false;
}

// String values arrive with the file's CRLF still attached; an untrimmed value matches no class
// and executes no command. Trailing-only on purpose -- devexec commands carry interior spaces.
static void copy_trim(char* dst, size_t cap, const char* val) {
    strncpy_s(dst, cap, val, _TRUNCATE);
    size_t n = strlen(dst);
    while (n > 0) {
        const unsigned char c = (unsigned char)dst[n - 1];
        if (c != 13 && c != 10 && c != 32 && c != 9) break;   // CR, LF, space, tab
        dst[--n] = 0;
    }
}

// VIEW-CONSUMER FIX keys (audio listener / navpoints / dev exec), early-return per the C1061
// note above. See the matching Config.hpp section for what each one means.
static bool parse_viewfix_key(const char* key, const char* val, double v) {
    if (_stricmp(key, "audiofix")   == 0) { g_cfg.audio_fix   = (v != 0.0); return true; }
    if (_stricmp(key, "audiocomp")  == 0) { g_cfg.audio_comp  = (v != 0.0); return true; }
    if (_stricmp(key, "audiodump")  == 0) { g_cfg.audio_dump  = (int)v; return true; }
    if (_stricmp(key, "cullfix")    == 0) { g_cfg.cull_fix    = (v != 0.0); return true; }
    if (_stricmp(key, "cullhead")   == 0) { g_cfg.cull_head   = (v != 0.0); return true; }
    if (_stricmp(key, "culldist")   == 0) { g_cfg.cull_dist   = clampf((float)v, 10000.0f, 100000000.0f); return true; }
    if (_stricmp(key, "navdump")    == 0) { g_cfg.nav_dump    = (int)v; return true; }
    if (_stricmp(key, "navfix")     == 0) { g_cfg.nav_fix     = (v != 0.0); return true; }
    if (_stricmp(key, "navproject") == 0) { g_cfg.nav_project = (v != 0.0); return true; }
    if (_stricmp(key, "navrender")  == 0) { g_cfg.nav_render  = (v != 0.0); return true; }
    if (_stricmp(key, "navworld")      == 0) { g_cfg.nav_world       = (v != 0.0); return true; }
    if (_stricmp(key, "navworlddist")  == 0) { g_cfg.nav_world_dist  = clampf((float)v, 200.0f, 20000.0f); return true; }
    if (_stricmp(key, "navworldscale") == 0) { g_cfg.nav_world_scale = clampf((float)v, 0.01f, 2.0f); return true; }
    if (_stricmp(key, "navworldcr")    == 0) { g_cfg.nav_world_cr    = clampf((float)v, 0.0f, 4.0f); return true; }
    if (_stricmp(key, "navworldcg")    == 0) { g_cfg.nav_world_cg    = clampf((float)v, 0.0f, 4.0f); return true; }
    if (_stricmp(key, "navworldcb")    == 0) { g_cfg.nav_world_cb    = clampf((float)v, 0.0f, 4.0f); return true; }
    if (_stricmp(key, "navworldlog")   == 0) { g_cfg.nav_world_log   = (int)v; return true; }
    if (_stricmp(key, "navworldsrc")   == 0) { g_cfg.nav_world_src   = (int)v; return true; }
    if (_stricmp(key, "navworldmax")   == 0) { g_cfg.nav_world_max   = clampf((float)v, 100.0f, 100000.0f); return true; }
    if (_stricmp(key, "navworldtrace") == 0) { g_cfg.nav_world_trace = (v != 0.0); return true; }
    if (_stricmp(key, "navworldsurf")  == 0) { g_cfg.nav_world_surf  = clampf((float)v, 0.0f, 1000.0f); return true; }
    if (_stricmp(key, "navworlddraw")  == 0) { g_cfg.nav_world_draw  = clampf((float)v, 0.0f, 2048.0f); return true; }
    if (_stricmp(key, "navworldicon")  == 0) { g_cfg.nav_world_icon  = (v != 0.0); return true; }
    if (_stricmp(key, "navwtree")      == 0) { g_cfg.navw_tree       = (v != 0.0); return true; }
    if (_stricmp(key, "navworldrender")== 0) { g_cfg.nav_world_render= (v != 0.0); return true; }
    if (_stricmp(key, "navhideflat")   == 0) { g_cfg.nav_hide_flat   = (v != 0.0); return true; }
    if (_stricmp(key, "navsizeobj")    == 0) { g_cfg.nav_size_obj    = clampf((float)v, 0.01f, 10.0f); return true; }
    if (_stricmp(key, "navsizeally")   == 0) { g_cfg.nav_size_ally   = clampf((float)v, 0.01f, 10.0f); return true; }
    if (_stricmp(key, "navsizeother")  == 0) { g_cfg.nav_size_other  = clampf((float)v, 0.01f, 10.0f); return true; }
    if (_stricmp(key, "navsizeclass")  == 0) { copy_trim(g_cfg.nav_size_class, sizeof(g_cfg.nav_size_class), val); return true; }
    if (_stricmp(key, "navsizeenemy")  == 0) { g_cfg.nav_size_enemy  = clampf((float)v, 0.01f, 10.0f); return true; }
    if (_stricmp(key, "navsizeitem")   == 0) { g_cfg.nav_size_item   = clampf((float)v, 0.01f, 10.0f); return true; }
    if (_stricmp(key, "navworldback")  == 0) { g_cfg.nav_world_back  = clampf((float)v, 0.0f, 10000.0f); return true; }
    if (_stricmp(key, "navworldimg")   == 0) { g_cfg.nav_world_img   = (int)clampf((float)v, 0.0f, 7.0f); return true; }
    if (_stricmp(key, "navworldclass") == 0) { copy_trim(g_cfg.nav_world_class, sizeof(g_cfg.nav_world_class), val); return true; }
    if (_stricmp(key, "navworldstride")== 0) { g_cfg.nav_world_stride= (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "navworldvisgate")==0) { g_cfg.nav_world_visgate= (int)v; return true; }
    if (_stricmp(key, "navworldkindmask")==0){ g_cfg.nav_world_kindmask=(int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "navscan")       == 0) { g_cfg.nav_scan        = (int)v; return true; }
    if (_stricmp(key, "navpitch")   == 0) { g_cfg.nav_pitch   = (v != 0.0); return true; }
    if (_stricmp(key, "navk")       == 0) { g_cfg.nav_k       = clampf((float)v, 50.0f, 10000.0f); return true; }
    if (_stricmp(key, "navmax")     == 0) { g_cfg.nav_max     = clampf((float)v, 50.0f, 8000.0f); return true; }
    if (_stricmp(key, "navclass")   == 0) { copy_trim(g_cfg.nav_class, sizeof(g_cfg.nav_class), val); return true; }
    if (_stricmp(key, "devexec1")   == 0) { copy_trim(g_cfg.dev_exec[0], sizeof(g_cfg.dev_exec[0]), val); return true; }
    if (_stricmp(key, "devexec2")   == 0) { copy_trim(g_cfg.dev_exec[1], sizeof(g_cfg.dev_exec[1]), val); return true; }
    if (_stricmp(key, "devexec3")   == 0) { copy_trim(g_cfg.dev_exec[2], sizeof(g_cfg.dev_exec[2]), val); return true; }
    if (_stricmp(key, "devexec4")   == 0) { copy_trim(g_cfg.dev_exec[3], sizeof(g_cfg.dev_exec[3]), val); return true; }
    return false;
}

// Melee-by-swing keys. Split into their own function for the same reason parse_config_key_2
// exists at all: MSVC's else-if chains in this file are already at the size where adding to them
// starts costing compile time for no readability.
// wpnoff=<match>,<dx>,<dy>,<dz>,<dgrip>,<dyaw>,<droll>
//
// One line per weapon, repeatable. Parsed positionally with everything after the match optional,
// so a line that only nudges X is "wpnoff=Pistol,1.5" rather than six trailing zeroes.

// wpnscope=<match>,<dzoom>,<ddist>,<dright>,<dup>,<dpitch>,<dyaw>,<droll>
//
// One line per weapon, repeatable, parsed POSITIONALLY with everything after the match optional --
// so a line that only changes magnification is "wpnscope=FP_SniperRifle,0.5" rather than seven
// trailing zeroes. Deltas on the global scope fit; see ScopeOffset.hpp.
static bool parse_scope_offset(const char* val) {
    if (val == nullptr || val[0] == 0) return false;
    if (g_cfg.scope_count >= kMaxScopeAdjust) return true;   // full: ignore rather than overflow

    char buf[256] = {0};
    strncpy_s(buf, sizeof(buf), val, _TRUNCATE);
    for (int i = (int)strlen(buf) - 1; i >= 0 && (unsigned char)buf[i] <= 0x20; --i) buf[i] = 0;

    char* ctx = nullptr;
    char* tok = strtok_s(buf, ",", &ctx);
    if (tok == nullptr || tok[0] == 0) return true;

    ScopeAdjust s{};
    strncpy_s(s.match, sizeof(s.match), tok, _TRUNCATE);

    float* fields[] = { &s.d_zoom, &s.d_dist, &s.d_right, &s.d_up,
                        &s.d_rot_p, &s.d_rot_y, &s.d_rot_r };
    for (int i = 0; i < 7; ++i) {
        tok = strtok_s(nullptr, ",", &ctx);
        if (tok == nullptr) break;
        *fields[i] = (float)atof(tok);
    }

    // REPLACE BY MATCH, DO NOT APPEND BLINDLY.
    //
    // find_entry() returns the FIRST match, and the config files are read shipped -> user -> dev ->
    // calib -> halo_vr_weapons.cfg. Appending therefore made the EARLIEST file win, which is exactly
    // backwards: kConfigFiles puts the per-weapon captures last precisely so "a captured delta for a
    // weapon that also has a hand-written entry must replace it in the matcher, not sit behind it".
    // That intent was written down and never implemented for this key.
    //
    // It only became load-bearing when a per-weapon scope calibration was SHIPPED in halo_vr.cfg: an
    // appended shipped entry would have out-ranked every player's own capture and silently reverted
    // their calibration on update -- the precise class of breakage the release notes promise to warn
    // about, arriving with no warning because nothing would have looked wrong from our side.
    //
    // Case-insensitive, matching how the entries are compared elsewhere.
    for (int i = 0; i < g_cfg.scope_count; ++i) {
        if (_stricmp(g_cfg.wpn_scope[i].match, s.match) == 0) {
            g_cfg.wpn_scope[i] = s;
            return true;
        }
    }
    g_cfg.wpn_scope[g_cfg.scope_count++] = s;
    return true;
}
static bool parse_weapon_offset(const char* val) {
    if (val == nullptr || val[0] == 0) return false;
    if (g_cfg.wpn_count >= kMaxWeaponAdjust) return true;   // full: ignore rather than overflow

    char buf[256] = {0};
    strncpy_s(buf, sizeof(buf), val, _TRUNCATE);

    // Trim trailing whitespace, including the CR this CRLF file leaves on every value.
    for (int i = (int)strlen(buf) - 1; i >= 0 && (unsigned char)buf[i] <= ' '; --i) buf[i] = 0;

    char* ctx = nullptr;
    char* tok = strtok_s(buf, ",", &ctx);
    if (tok == nullptr || tok[0] == 0) return true;

    WeaponAdjust w{};
    strncpy_s(w.match, sizeof(w.match), tok, _TRUNCATE);

    float* fields[] = { &w.d_x, &w.d_y, &w.d_z, &w.d_grip, &w.d_grip_yaw, &w.d_grip_roll };
    for (int i = 0; i < 6; ++i) {
        tok = strtok_s(nullptr, ",", &ctx);
        if (tok == nullptr) break;
        *fields[i] = (float)atof(tok);
    }

    // REPLACE BY MATCH -- same rule and same reason as parse_scope_offset above. The lookup in
    // WeaponOffset.cpp takes the FIRST match ("hit = &w; break;") and the files load shipped ->
    // user -> dev -> calib -> halo_vr_weapons.cfg, so appending made the EARLIEST file win. That is
    // backwards from what kConfigFiles says it is doing, and it becomes destructive the moment a
    // wpnoff baseline is SHIPPED: the shipped entry would out-rank the player's own capture for the
    // same weapon and silently revert their calibration on update.
    for (int i = 0; i < g_cfg.wpn_count; ++i) {
        if (_stricmp(g_cfg.wpn[i].match, w.match) == 0) {
            g_cfg.wpn[i] = w;
            return true;
        }
    }
    g_cfg.wpn[g_cfg.wpn_count++] = w;
    return true;
}

// The wpnfix schema stamp currently in force, and whether the file being parsed is the capture
// file. Both are PER FILE and reset at the top of parse_config_file() -- see the note there.
static int  s_wpnfix_file_ver     = 0;
static bool s_wpnfix_from_capture = false;

// wpnfix=<match>,qx,qy,qz,qw,tx,ty,tz -- the PALETTE path's per-weapon rigid delta.
//
// Two sources feed this one table: the shipped baseline in halo_vr.cfg and the player's captures in
// halo_vr_weapons.cfg. Both parse through here; weapon_fix_for() decides which wins.
//
// UNLIKE wpnoff EVERY FIELD IS REQUIRED. A partially specified quaternion is not a rotation, and
// defaulting the missing components would silently invent one, so a short line is dropped rather
// than padded.
//
// Deliberately NOT range-clamped the way the scalar keys are. The sanity bounds that matter here
// are geometric (how far a capture moved the weapon, how far it rotated it) and they are applied at
// the CAPTURE, where the delta being judged is a single gesture's worth. A file value has already
// passed that gate; re-judging the accumulated total here would reject a legitimately large trim.
// What IS enforced is the schema stamp below, and that the quaternion is a rotation -- checked by
// the consumer, which drops an entry it cannot turn into a valid basis.
static bool parse_weapon_fix(const char* val) {
    if (val == nullptr || val[0] == 0) return false;

    // THE STAMP GATE, AT PARSE. Refusing here rather than at lookup is what keeps the resolver a
    // plain search: by the time weapon_fix_for() runs, every entry in the table is one this build
    // vouches for, and there is no second rule for a reader to get wrong.
    if (s_wpnfix_file_ver != kWeaponFixSchema) {
        if (g_cfg.wpnfix_dropped < 0x7FFF) ++g_cfg.wpnfix_dropped;
        return true;
    }
    if (g_cfg.wpnfix_count >= kMaxWeaponAdjust) return true;   // full: ignore rather than overflow

    char buf[256] = {0};
    strncpy_s(buf, sizeof(buf), val, _TRUNCATE);
    for (int i = (int)strlen(buf) - 1; i >= 0 && (unsigned char)buf[i] <= ' '; --i) buf[i] = 0;

    char* ctx = nullptr;
    char* tok = strtok_s(buf, ",", &ctx);
    if (tok == nullptr || tok[0] == 0) return true;

    WeaponFix w{};
    strncpy_s(w.match, sizeof(w.match), tok, _TRUNCATE);
    w.captured = s_wpnfix_from_capture;

    float* fields[] = { &w.q[0], &w.q[1], &w.q[2], &w.q[3], &w.t[0], &w.t[1], &w.t[2] };
    for (float* field : fields) {
        tok = strtok_s(nullptr, ",", &ctx);
        if (tok == nullptr) return true;      // short line: drop it, do not half-apply a rotation
        *field = (float)atof(tok);
    }

    // REPLACE BY MATCH. weapon_fix_for() also takes the first match (strstr ... return &w), and a
    // wpnfix baseline is ALREADY shipped in halo_vr.cfg -- so unlike the other two this was not a
    // latent trap but a live one: a player who captured their own fix for a weapon we ship a fix for
    // was already being ignored in favour of ours.
    for (int i = 0; i < g_cfg.wpnfix_count; ++i) {
        if (_stricmp(g_cfg.wpnfix[i].match, w.match) == 0) {
            g_cfg.wpnfix[i] = w;
            return true;
        }
    }
    g_cfg.wpnfix[g_cfg.wpnfix_count++] = w;
    return true;
}

// The handfix schema stamp currently in force, PER FILE, reset with the wpnfix one below.
static int s_handfix_file_ver = 0;

// handfix=qx,qy,qz,qw,tx,ty,tz -- the SUPPORT HAND's rigid trim, in that controller's own frame.
//
// Written only by the menu-armed capture (halo_vr_calib.cfg); there is no shipped baseline, because
// the quantity is where a person's hand sits inside their own controller. Same all-or-nothing rule
// as wpnfix: a short line is DROPPED rather than padded, since a partially specified quaternion is
// not a rotation and defaulting the missing components would silently invent one.
//
// Not range-clamped here for the same reason wpnfix is not: the geometric sanity bound belongs at
// the CAPTURE, where what is being judged is one gesture's worth of movement. A stored total that
// several refining captures have accumulated is legitimately larger than any single one of them.
static bool parse_hand_fix(const char* val) {
    if (val == nullptr || val[0] == 0) return false;

    // THE STAMP GATE, AT PARSE -- so by the time anything reads g_cfg.hand_fix_* it is a value this
    // build vouches for, with no second rule for a consumer to get wrong.
    if (s_handfix_file_ver != kHandFixSchema) return true;

    char buf[128] = {0};
    strncpy_s(buf, sizeof(buf), val, _TRUNCATE);
    for (int i = (int)strlen(buf) - 1; i >= 0 && (unsigned char)buf[i] <= ' '; --i) buf[i] = 0;

    float q[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    float t[3] = {0.0f, 0.0f, 0.0f};
    float* fields[] = { &q[0], &q[1], &q[2], &q[3], &t[0], &t[1], &t[2] };

    char* ctx = nullptr;
    char* tok = strtok_s(buf, ",", &ctx);
    for (float* field : fields) {
        if (tok == nullptr) return true;      // short line: drop it, do not half-apply a rotation
        *field = (float)atof(tok);
        tok = strtok_s(nullptr, ",", &ctx);
    }
    for (int i = 0; i < 4; ++i) g_cfg.hand_fix_q[i] = q[i];
    for (int i = 0; i < 3; ++i) g_cfg.hand_fix_t[i] = t[i];
    g_cfg.hand_fix_valid = true;
    return true;
}

// THE ONE PER-WEAPON LOOKUP. Precedence, and why it is a BACKWARDS scan, is documented on the
// declaration in Config.hpp -- read it there rather than re-deriving it from this loop.
const WeaponFix* weapon_fix_for(const char* class_name) {
    if (class_name == nullptr || class_name[0] == 0) return nullptr;
    for (int i = g_cfg.wpnfix_count - 1; i >= 0; --i) {
        const WeaponFix& w = g_cfg.wpnfix[i];
        if (w.match[0] == 0) continue;
        if (strstr(class_name, w.match) != nullptr) return &w;
    }
    return nullptr;
}

// Compositor reticule (XrLayer.cpp). Dev/research keys -- default off, catalogued in
// halo_vr_dev.cfg, deliberately absent from the player reference.
//
// Its own function rather than nine more rungs on parse_config_key_2's else-if chain: that chain is
// AT MSVC's block-nesting ceiling. Adding these to it inline is what produced
// "fatal error C1061: compiler limit: blocks nested too deeply" -- a build break with no obvious
// connection to the keys that caused it. Extract the next group too; do not extend the chain.
static bool parse_xrlayer_key(const char* key, double v) {
    if (_stricmp(key, "xrlayer")       == 0) { g_cfg.xr_layer       = (v != 0.0); return true; }
    if (_stricmp(key, "xrlayerspace")  == 0) { g_cfg.xr_layer_space = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "xrlayerhrel")   == 0) { g_cfg.xr_layer_head_rel = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "xrlayersize")   == 0) { g_cfg.xr_layer_size  = clampf((float)v, 0.05f, 20.0f); return true; }
    // No upper clamp worth guessing: the whole reason this key exists is that the right value is
    // unmeasured, and a clamp below the true value would look exactly like "the setting does
    // nothing" -- the failure mode aimwidgettint already taught us once.
    if (_stricmp(key, "xrlayercmpm")   == 0) { g_cfg.xr_layer_cm_per_m = (float)((v < 0.0) ? 0.0 : v); return true; }
    if (_stricmp(key, "xrlayeralpha")  == 0) { g_cfg.xr_layer_alpha = clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "xrlayercr")     == 0) { g_cfg.xr_layer_cr    = clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "xrlayercg")     == 0) { g_cfg.xr_layer_cg    = clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "xrlayercb")     == 0) { g_cfg.xr_layer_cb    = clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "xrlayerzoomfit") == 0) { g_cfg.xr_layer_zoom_fit   = clampf((float)v, 0.05f, 20.0f); return true; }
    if (_stricmp(key, "xrlayerscopereticle")     == 0) { g_cfg.xr_layer_scope_reticle      = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "xrlayerscopereticlesize") == 0) { g_cfg.xr_layer_scope_reticle_size = clampf((float)v, 0.01f, 1.0f);     return true; }
    // CENTIMETRES, SIGNED, and deliberately wide. Positive is toward the viewer, NEGATIVE pushes the
    // reticule BEHIND the pane's plane -- further from the eye, which is the direction a collimated
    // (holographic) sight needs: its reticule sits at optical infinity, so it holds still against the
    // target while the pane's own image parallaxes with head movement.
    //
    // A NEGATIVE DEPTH STILL DRAWS. Verified in this file rather than assumed: the pane's slot quads
    // are appended to the layer list before g_scope_ret_quad, and OpenXR composites quad layers in
    // SUBMISSION ORDER with no depth test between them, so the reticule is painted over the pane at
    // any offset. Depth here buys stereo disparity -- which is the whole point -- not occlusion.
    //
    // The earlier 0.2 floor was mine and was over-cautious: it guarded a z-fight that cannot happen
    // between two composition layers for the reason just given. Bounds are kept only wide enough to
    // reject garbage and NaN, not to express an opinion about useful values.
    //
    // WATCH THE ANGULAR SIZE. The quad's world size is fixed at pane_size * xrlayerscopereticlesize,
    // so pushing it away shrinks it on screen exactly as a real object would. Simulating infinity
    // means raising the size key as the depth goes more negative; the pair is the knob, not either
    // one alone.
    if (_stricmp(key, "xrlayerscopereticledepth")== 0) { g_cfg.xr_layer_scope_reticle_depth = clampf((float)v, -10000.0f, 10000.0f); return true; }
    if (_stricmp(key, "xrlayerroll")    == 0) { g_cfg.xr_layer_roll       = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "xrlayerhidescope")== 0) { g_cfg.xr_layer_hide_scope = (v != 0.0); return true; }
    if (_stricmp(key, "xrlayerhidews")  == 0) { g_cfg.xr_layer_hide_ws    = (int)clampf((float)v, 0.0f, 4.0f); return true; }   // 3 = bVisibleInSceneCaptureOnly, 4 = that plus bOwnerNoSee (adding a mode means widening its validator)
    if (_stricmp(key, "xrlayerhidescale") == 0) { g_cfg.xr_layer_hide_scale = clampf((float)v, 0.0001f, 1.0f); return true; }
    if (_stricmp(key, "xrlayerlog")    == 0) { g_cfg.xr_layer_log   = (v != 0.0); return true; }
    // Stage 2. The probe key is a ladder (0 off / 1 walk+log / 2 walk+log+virtual call), so it is
    // clamped rather than coerced to a bool -- writing 1 must NOT be able to reach the rung that
    // makes an indirect call on an inferred pointer.
    if (_stricmp(key, "xrlayersrcprobe") == 0) { g_cfg.xr_layer_src_probe = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    // Opt-in dev harness (default 0). Proves the chain offsets are class-level by walking a second
    // component with the full discovery probe -- expensive, so it stays off unless a dev asks, even
    // on a dev build. It is NOT tied to xrlayersrc: the feed does not need the proof to run.
    if (_stricmp(key, "xrlayersrcxcheck") == 0) { g_cfg.xr_layer_src_xcheck = (v != 0.0); return true; }
    // Perf fix, ships ON. 0 forces per-tick get_native_resource()+GetDesc (pre-fix) for A/B measuring.
    if (_stricmp(key, "xrlayersrccache") == 0) { g_cfg.xr_layer_src_cache  = (v != 0.0); return true; }
    if (_stricmp(key, "xrlayersrc")      == 0) { g_cfg.xr_layer_src       = (v != 0.0); return true; }
    // 0 is a real setting -- "revert to the ring the instant a capture is missed", i.e. the old
    // hardcoded behaviour with the window shut. The upper bound is a minute, well past any level
    // load, because the failure at that end is only cosmetic (a frozen crosshair) while the failure
    // at the low end is the flash this key exists to remove.
    if (_stricmp(key, "xrlayerhold")     == 0) { g_cfg.xr_layer_hold_ms   = (int)clampf((float)v, 0.0f, 60000.0f); return true; }
    // ---- world navpoint markers on the layer. All four live HERE and nowhere else: the general
    // else-if chain in parse_config_key_2 is already at MSVC's 128-deep block limit, and adding to
    // it fails the build (C1061) pointing at an unrelated line.
    if (_stricmp(key, "xrlayernav")      == 0) { g_cfg.xr_layer_nav       = (v != 0.0); return true; }
    // 1..8, never 0: "no markers" is xrlayernav=0, and letting this key mean the same thing would
    // give the feature two off switches that fail differently.
    if (_stricmp(key, "xrlayernavmax")   == 0) { g_cfg.xr_layer_nav_max   = (int)clampf((float)v, 1.0f, 8.0f); return true; }
    // Lower bound is the fixed 1 m near clamp -- below that the quad is uncomfortable to fuse, and
    // a clamp that lets a setting reach an unusable value is a setting that looks broken.
    if (_stricmp(key, "xrlayernavdist")  == 0) { g_cfg.xr_layer_nav_dist  = clampf((float)v, 100.0f, 20000.0f); return true; }
    // Upper bound is the local layer-array bound in hooked_end_frame. Forcing a budget ABOVE what
    // the runtime accepts would just make xrEndFrame reject the frame, which is a worse way to
    // learn the number than reading the ARMED line.
    if (_stricmp(key, "xrlayerbudget")   == 0) { g_cfg.xr_layer_budget    = (int)clampf((float)v, 0.0f, 32.0f); return true; }
    return false;
}

// Holster / grenade-pouch keys (blindcowboy24 PR-1, taken alone). Early-return parser for the
// same C1061 reason as its siblings above.
static bool parse_holster_key(const char* key, const char* val, double v) {
    if (_stricmp(key, "holster")        == 0) { g_cfg.holster_enabled = (v != 0.0); return true; }
    if (_stricmp(key, "holsterradius")  == 0) { g_cfg.holster_radius  = clampf((float)v, 0.05f, 0.5f); return true; }
    if (_stricmp(key, "holstergradius") == 0) { g_cfg.holster_gradius = clampf((float)v, 0.03f, 0.5f); return true; }
    if (_stricmp(key, "holsterrs")      == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.holster_rs[0], &g_cfg.holster_rs[1], &g_cfg.holster_rs[2]); return true; }
    if (_stricmp(key, "holsterls")      == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.holster_ls[0], &g_cfg.holster_ls[1], &g_cfg.holster_ls[2]); return true; }
    if (_stricmp(key, "holsterrh")      == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.holster_rh[0], &g_cfg.holster_rh[1], &g_cfg.holster_rh[2]); return true; }
    if (_stricmp(key, "holsterlc")      == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.holster_lc[0], &g_cfg.holster_lc[1], &g_cfg.holster_lc[2]); return true; }
    if (_stricmp(key, "holsterrc")      == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.holster_rc[0], &g_cfg.holster_rc[1], &g_cfg.holster_rc[2]); return true; }
    if (_stricmp(key, "holstergswitchmask") == 0) { g_cfg.holster_gswitch_mask = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "holstermeleevetoms") == 0) { g_cfg.holster_melee_veto_ms = (int)clampf((float)v, 0.0f, 2000.0f); return true; }
    if (_stricmp(key, "holstermeleemargin") == 0) { g_cfg.holster_melee_margin = clampf((float)v, 0.0f, 0.5f); return true; }
    if (_stricmp(key, "holstermarkers") == 0) { g_cfg.holster_markers = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "holstermarkerscale") == 0) { g_cfg.holster_marker_scale = clampf((float)v, 0.02f, 0.5f); return true; }
    if (_stricmp(key, "holsteraimhold") == 0) { g_cfg.holster_aim_hold_ms = (int)clampf((float)v, 0.0f, 2000.0f); return true; }
    if (_stricmp(key, "meleeaimhold") == 0) { g_cfg.melee_aim_hold_ms = (int)clampf((float)v, 0.0f, 2000.0f); return true; }
    if (_stricmp(key, "meleeaimramp") == 0) { g_cfg.melee_aim_ramp_ms = (int)clampf((float)v, 1.0f, 2000.0f); return true; }
    if (_stricmp(key, "holsteryawdead") == 0) { g_cfg.holster_yaw_dead = clampf((float)v, 0.0f, 180.0f); return true; }
    if (_stricmp(key, "holsteryawrate") == 0) { g_cfg.holster_yaw_rate = clampf((float)v, 0.0f, 360.0f); return true; }
    if (_stricmp(key, "holsterneckdown") == 0) { g_cfg.holster_neck_down = clampf((float)v, 0.0f, 0.5f); return true; }
    if (_stricmp(key, "holsterneckback") == 0) { g_cfg.holster_neck_back = clampf((float)v, 0.0f, 0.5f); return true; }
    if (_stricmp(key, "holsterswapmask")  == 0) { g_cfg.holster_swap_mask  = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "holsterstealbtns") == 0) { g_cfg.holster_steal_buttons = (int)v; return true; }
    if (_stricmp(key, "holstergren")    == 0) { g_cfg.holster_grenades = (v != 0.0); return true; }
    if (_stricmp(key, "holstergrenhand") == 0) { g_cfg.holster_gren_hand = (int)v; return true; }
    if (_stricmp(key, "holsterthrowmask") == 0) { g_cfg.holster_throw_mask = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "holsterpressms")   == 0) { g_cfg.holster_press_ms   = (int)clampf((float)v, 30.0f, 500.0f); return true; }
    if (_stricmp(key, "holsterthrowspeed") == 0) { g_cfg.holster_throw_speed = clampf((float)v, 0.2f, 6.0f); return true; }
    if (_stricmp(key, "holsterhaptic")  == 0) { g_cfg.holster_haptic  = (v != 0.0); return true; }
    if (_stricmp(key, "holsterlog")     == 0) { g_cfg.holster_log     = (v != 0.0); return true; }
    // The visible magazine rides the same marker machinery as the pouches, so its keys come
    // across with the holster set rather than with the reload gesture (which is NOT ported here).
    if (_stricmp(key, "reloadmag")      == 0) { g_cfg.reload_mag = (int)v; return true; }
    if (_stricmp(key, "reloadmagoff")   == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.reload_mag_off[0], &g_cfg.reload_mag_off[1], &g_cfg.reload_mag_off[2]); return true; }
    if (_stricmp(key, "reloadmagrad")   == 0) { g_cfg.reload_mag_radius = clampf((float)v, 0.05f, 0.5f); return true; }
    if (_stricmp(key, "reloadmagscale") == 0) { g_cfg.reload_mag_scale = clampf((float)v, 0.05f, 20.0f); return true; }
    return false;
}

static bool parse_melee_key(const char* key, const char* val, double v) {
    if (_stricmp(key, "meleeswing")     == 0) { g_cfg.melee_swing    = (v != 0.0); return true; }
    if (_stricmp(key, "meleespeed")     == 0) { g_cfg.melee_speed    = clampf((float)v, 0.0f, 20.0f); return true; }
    if (_stricmp(key, "meleeext")       == 0) { g_cfg.melee_ext      = clampf((float)v, 0.0f, 20.0f); return true; }
    if (_stricmp(key, "meleereach")     == 0) { g_cfg.melee_reach    = clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "meleemaxspeed")  == 0) { g_cfg.melee_max_speed = clampf((float)v, 1.0f, 100.0f); return true; }
    if (_stricmp(key, "meleemaxreach")  == 0) { g_cfg.melee_max_reach = clampf((float)v, 0.3f, 5.0f); return true; }
    if (_stricmp(key, "meleefwd")       == 0) { g_cfg.melee_fwd      = clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "meleetau")       == 0) { g_cfg.melee_tau_ms   = clampf((float)v, 0.0f, 200.0f); return true; }
    if (_stricmp(key, "meleecooldown")  == 0) { g_cfg.melee_cooldown_ms = (int)clampf((float)v, 0.0f, 5000.0f); return true; }
    if (_stricmp(key, "meleeaimmode")   == 0) { g_cfg.melee_aim_mode  = (int)v; return true; }
    if (_stricmp(key, "meleeaimhold")   == 0) { g_cfg.melee_aim_hold_ms = (int)clampf((float)v, 0.0f, 2000.0f); return true; }
    if (_stricmp(key, "meleeaimramp")   == 0) { g_cfg.melee_aim_ramp_ms = (int)clampf((float)v, 1.0f, 2000.0f); return true; }
    if (_stricmp(key, "meleehold")      == 0) { g_cfg.melee_hold_ms  = (int)clampf((float)v, 8.0f, 1000.0f); return true; }
    // base 0 so the mask can be written 0x0080 (readable) or 128 (not), as elsewhere in this file.
    if (_stricmp(key, "meleemask")      == 0) { g_cfg.melee_mask     = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "meleelog")       == 0) { g_cfg.melee_log      = (v != 0.0); return true; }
    // ---- ACTION BINDS. Their own block, in a function rather than the main else-if chain, for
    // the reason parse_scope_key documents: that chain is already near MSVC's 128-deep block
    // limit (C1061) and adding six more to it is how the build breaks in a way that looks
    // unrelated to the change. Base-0 strtol so a captured bind round-trips as the 0xNNNN the
    // capture wrote.
    if (_stricmp(key, "bindcrouch")     == 0) { g_cfg.bind_crouch     = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "bindmelee")      == 0) { g_cfg.bind_melee      = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "bindreload")     == 0) { g_cfg.bind_reload     = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "bindscope")      == 0) { g_cfg.bind_scope      = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "bindequip")      == 0) { g_cfg.bind_equip      = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "binddpadshift")  == 0) { g_cfg.bind_dpad_shift = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "bindcapturems")  == 0) { g_cfg.bind_capture_ms = (int)clampf((float)v, 2000.0f, 120000.0f); return true; }
    if (_stricmp(key, "bonedump")       == 0) { g_cfg.bone_dump      = (v != 0.0); return true; }
    if (_stricmp(key, "armhide")        == 0) { g_cfg.arm_hide       = (v != 0.0); return true; }
    if (_stricmp(key, "armhideall")     == 0) { g_cfg.arm_hide_all   = (v != 0.0); return true; }
    // Clamped to the enum's range rather than passed through: an out-of-range value here would
    // silently mean "off", and losing both arm drivers to a typo is not a failure anyone would
    // connect back to this key.
    if (_stricmp(key, "armdriver")      == 0) { g_cfg.arm_driver     = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    // One chain link for the whole palettearm family -- same C1061 reasoning as parse_scope_key().
    if (palettearm_parse_key(key, v))         { return true; }
    if (_stricmp(key, "padump")         == 0) { g_cfg.pa_dump       = (v != 0.0); return true; }
    if (_stricmp(key, "patestlift")     == 0) { g_cfg.pa_test_lift  = (float)v; return true; }
    if (_stricmp(key, "paarmlift")     == 0) { g_cfg.pa_arm_lift      = (int)v; return true; }
    if (_stricmp(key, "paarmpitch")    == 0) { g_cfg.pa_arm_pitch     = (int)v; return true; }
    if (_stricmp(key, "pahands")       == 0) { g_cfg.pa_hands_only    = (int)v; return true; }
    if (_stricmp(key, "paarmrest")     == 0) { g_cfg.pa_arm_rest_lift = (int)v; return true; }
    if (_stricmp(key, "patgtframe")    == 0) { g_cfg.pa_target_frame   = (int)v; return true; }
    if (_stricmp(key, "pagrabwpn")     == 0) { g_cfg.pa_grab_weapon    = (int)v; return true; }
    if (_stricmp(key, "pawpnlift")     == 0) { g_cfg.pa_wpn_lift      = (int)v; return true; }
    if (_stricmp(key, "pawpnyaw")      == 0) { g_cfg.pa_wpn_yaw       = (float)v; return true; }
    if (_stricmp(key, "pawpnpitch")    == 0) { g_cfg.pa_wpn_pitch     = (float)v; return true; }
    if (_stricmp(key, "pawpnroll")     == 0) { g_cfg.pa_wpn_roll      = (float)v; return true; }
    // THE BARREL LOCK (Config.hpp pa_barrel_lock). pabarrelrelease <= pabarrelfull is a hard cap
    // rather than a fade -- blindcowboy24's exact behaviour -- and is a supported setting, so it is
    // deliberately NOT clamped up to pabarrelfull here.
    if (_stricmp(key, "pabarrellock")  == 0) { g_cfg.pa_barrel_lock   = (v != 0.0); return true; }
    if (_stricmp(key, "pabarrelfull")  == 0) { g_cfg.pa_barrel_full    = clampf((float)v, 0.0f, 90.0f); return true; }
    if (_stricmp(key, "pabarrelrelease") == 0) { g_cfg.pa_barrel_release = clampf((float)v, 0.0f, 90.0f); return true; }
    if (_stricmp(key, "pawpn")         == 0) { g_cfg.pa_weapon       = (v != 0.0); return true; }
    if (_stricmp(key, "paarms")         == 0) { g_cfg.pa_arms       = (int)clampf((float)v, 0.0f, 3.0f); return true; }
    if (_stricmp(key, "headshouldersyawinfluence") == 0) { g_cfg.pa_head_shoulders_yaw_influence = clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "patorsoframe")   == 0) { g_cfg.pa_torso_frame= (int)clampf((float)v, 0.0f, 6.0f); return true; }   // 0..4: adding a mode means widening THIS clamp too -- 3 and 4 silently became 2 for a whole session
    if (_stricmp(key, "twohand")        == 0) { g_cfg.two_hand      = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "twohandaim")     == 0) { g_cfg.two_hand_aim  = (v != 0.0); return true; }
    if (_stricmp(key, "twohandrig")     == 0) { g_cfg.two_hand_rig  = (v != 0.0); return true; }
    if (_stricmp(key, "twohandlog")     == 0) { g_cfg.two_hand_log  = (v != 0.0); return true; }
    // base 0 so the mask can be written 0x0100 (readable) or 256 (not), as elsewhere in this file.
    if (_stricmp(key, "bindtwohand")    == 0) { g_cfg.bind_two_hand = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "twohanddeny")    == 0) {
        strncpy_s(g_cfg.two_hand_deny, sizeof(g_cfg.two_hand_deny), val, _TRUNCATE);
        return true;
    }
    // One chain link for the two-hand tuning family -- same C1061 reasoning as parse_scope_key().
    if (two_hand_parse_key(key, v))           { return true; }
    // Flat links, not else-if rungs: this chain has already hit C1061 twice.
    if (_stricmp(key, "dpadhead")      == 0) { g_cfg.dpad_head         = (v != 0.0); return true; }
    if (_stricmp(key, "dpadheadcm")    == 0) { g_cfg.dpad_head_cm      = clampf((float)v, 5.0f, 80.0f); return true; }
    if (_stricmp(key, "dpadheadhyst")  == 0) { g_cfg.dpad_head_hyst_cm = clampf((float)v, 0.0f, 40.0f); return true; }
    if (_stricmp(key, "dpadheaddwell") == 0) { g_cfg.dpad_head_dwell_ms = (int)clampf((float)v, 0.0f, 2000.0f); return true; }
    // THE BUTTON-REMAP FAMILY, moved off parse_config_key_2's else-if chain (2026-09-05) when
    // adding the rgrip* pair tripped C1061 for the third time. The whole family moved rather than
    // just the two new keys, so the next control binding is an edit and not another refactor --
    // control work is exactly where keys keep getting added, which is why that chain keeps
    // overflowing here. base 0 on every mask so they stay readable as 0x0200 rather than 512.
    if (_stricmp(key, "mapdpadshift")  == 0) { g_cfg.map_dpad_shift  = (v != 0.0); return true; }
    if (_stricmp(key, "mapdpaddz")     == 0) { g_cfg.map_dpad_dz     = clampf((float)v, 0.2f, 0.95f); return true; }
    if (_stricmp(key, "maprstickdown") == 0) { g_cfg.map_rstick_down = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "maprstickdz")   == 0) { g_cfg.map_rstick_dz   = clampf((float)v, 0.2f, 0.95f); return true; }
    if (_stricmp(key, "mapfrom")       == 0) { g_cfg.map_from        = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "mapto")         == 0) { g_cfg.map_to          = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "mapbtnlog")     == 0) { g_cfg.map_btn_log     = (v != 0.0); return true; }
    // The right grip is reserved for over-the-shoulder weapon switching, so the game stops seeing
    // it. Config.hpp carries the measurement and the ordering argument.
    if (_stricmp(key, "rgripmask")     == 0) { g_cfg.rgrip_mask      = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "rgripswallow")  == 0) { g_cfg.rgrip_swallow   = (v != 0.0); return true; }
    // And one for the grab guide, same reasoning again -- it owns its own keys next to the code
    // that reads them, and this stays a flat link rather than another rung on the else-if chain.
    if (interact_line_parse_key(key, v, val)) { return true; }
    if (_stricmp(key, "wpncalibkey")   == 0) { g_cfg.wpn_calib_key   = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "wpnoffsets")    == 0) { g_cfg.wpn_offsets     = (v != 0.0); return true; }
    if (_stricmp(key, "wpnlog")        == 0) { g_cfg.wpn_log         = (v != 0.0); return true; }
    if (_stricmp(key, "wpndrive")      == 0) { g_cfg.wpn_drive       = (v != 0.0); return true; }
    if (_stricmp(key, "wpnoff")        == 0) { return parse_weapon_offset(val); }
    if (_stricmp(key, "wpnscope")      == 0) { return parse_scope_offset(val); }
    if (_stricmp(key, "scopeoffsets")  == 0) { g_cfg.scope_offsets = (v != 0.0); return true; }
    if (_stricmp(key, "scopewpnlog")   == 0) { g_cfg.scope_wpn_log = (v != 0.0); return true; }
    if (_stricmp(key, "wpnfix")        == 0) { return parse_weapon_fix(val); }
    // Scoped to the file being parsed, NOT stored in g_cfg -- see parse_config_file()'s reset.
    if (_stricmp(key, "wpnfixver")     == 0) { s_wpnfix_file_ver     = (int)v; return true; }
    if (_stricmp(key, "handfix")       == 0) { return parse_hand_fix(val); }
    if (_stricmp(key, "handfixver")    == 0) { s_handfix_file_ver    = (int)v; return true; }
    if (_stricmp(key, "armkeeppose")   == 0) { g_cfg.arm_keep_pose   = (v != 0.0); return true; }
    if (_stricmp(key, "armhidemode")    == 0) { g_cfg.arm_hide_mode  = (int)v; return true; }
    if (_stricmp(key, "armhidebone")    == 0) {
        strncpy_s(g_cfg.arm_hide_bone, sizeof(g_cfg.arm_hide_bone), val, _TRUNCATE);
        return true;
    }
    if (_stricmp(key, "reloadvr")       == 0) { g_cfg.reload_vr        = (v != 0.0); return true; }
    // base 0 so masks can be written 0x4000 (readable) or 16384 (not), as elsewhere in this file.
    if (_stricmp(key, "reloadmask")     == 0) { g_cfg.reload_mask      = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "reloadgrip")     == 0) { g_cfg.reload_grip_mask = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "reloadbeltdrop") == 0) { g_cfg.reload_belt_drop = clampf((float)v, 0.0f, 1.5f); return true; }
    if (_stricmp(key, "reloadbeltrad")  == 0) { g_cfg.reload_belt_radius = clampf((float)v, 0.05f, 1.5f); return true; }
    if (_stricmp(key, "reloadjoin")     == 0) { g_cfg.reload_join_dist = clampf((float)v, 0.05f, 1.0f); return true; }
    if (_stricmp(key, "reloadnofire")   == 0) { g_cfg.reload_suppress_fire = (v != 0.0); return true; }
    if (_stricmp(key, "reloadcancel")   == 0) { g_cfg.reload_cancel    = (v != 0.0); return true; }
    if (_stricmp(key, "reloadtimeout")  == 0) { g_cfg.reload_timeout_s = clampf((float)v, 0.0f, 120.0f); return true; }
    if (_stricmp(key, "grenadefrom")   == 0) { g_cfg.grenade_from    = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "grenadeaction") == 0) { g_cfg.grenade_action  = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "gripexclusive") == 0) { g_cfg.grip_exclusive   = (v != 0.0); return true; }
    if (_stricmp(key, "gripswallow")   == 0) { g_cfg.grip_swallow     = (v != 0.0); return true; }
    if (_stricmp(key, "gripzoom")      == 0) { g_cfg.grip_zoom        = (v != 0.0); return true; }
    if (_stricmp(key, "reloadhold")     == 0) { g_cfg.reload_hold_ms   = (int)clampf((float)v, 60.0f, 2000.0f); return true; }
    if (_stricmp(key, "reloadlog")      == 0) { g_cfg.reload_log       = (v != 0.0); return true; }
    if (_stricmp(key, "handsvr")        == 0) { g_cfg.hands_vr         = (v != 0.0); return true; }
    if (_stricmp(key, "handscale")      == 0) { g_cfg.hand_scale       = clampf((float)v, 0.001f, 5.0f); return true; }
    if (_stricmp(key, "handoffx")       == 0) { g_cfg.hand_off_x       = (float)v; return true; }
    if (_stricmp(key, "handoffy")       == 0) { g_cfg.hand_off_y       = (float)v; return true; }
    if (_stricmp(key, "handoffz")       == 0) { g_cfg.hand_off_z       = (float)v; return true; }
    if (_stricmp(key, "handshowaim")    == 0) { g_cfg.hand_show_aim    = (v != 0.0); return true; }
    if (_stricmp(key, "handmesh")       == 0) {
        strncpy_s(g_cfg.hand_mesh_path, sizeof(g_cfg.hand_mesh_path), val, _TRUNCATE); return true;
    }
    if (_stricmp(key, "magshow")        == 0) { g_cfg.mag_show         = (v != 0.0); return true; }
    if (_stricmp(key, "magscale")       == 0) { g_cfg.mag_scale        = clampf((float)v, 0.001f, 5.0f); return true; }
    if (_stricmp(key, "magoffx")        == 0) { g_cfg.mag_off_x        = (float)v; return true; }
    if (_stricmp(key, "magoffy")        == 0) { g_cfg.mag_off_y        = (float)v; return true; }
    if (_stricmp(key, "magoffz")        == 0) { g_cfg.mag_off_z        = (float)v; return true; }
    if (_stricmp(key, "magmesh")        == 0) {
        strncpy_s(g_cfg.mag_mesh_path, sizeof(g_cfg.mag_mesh_path), val, _TRUNCATE); return true;
    }
    return false;
}

void parse_config_key_2(const char* key, const char* val, double v) {
        if (parse_blam_key(key, val, v)) return;
        if (parse_viewfix_key(key, val, v)) return;
        if (parse_melee_key(key, val, v)) return;
        if (parse_xrlayer_key(key, v)) return;
        if (parse_holster_key(key, val, v)) return;
        if (_stricmp(key, "attachpermanent") == 0) g_cfg.attach_permanent = (v != 0.0);
        else if (_stricmp(key, "gainadapt")   == 0) g_cfg.gain_adapt    = (v != 0.0);
        else if (_stricmp(key, "huddump")    == 0) g_cfg.hud_dump      = (v != 0.0);
        else if (_stricmp(key, "hudfollow")  == 0) g_cfg.hud_follow    = (v != 0.0);
        else if (_stricmp(key, "hudk")       == 0) g_cfg.hud_k         = (float)v;
        else if (_stricmp(key, "hudmax")     == 0) g_cfg.hud_max       = clampf((float)v, 50.0f, 4000.0f);
        else if (_stricmp(key, "hudfloat")   == 0) g_cfg.hud_float     = (v != 0.0);
        else if (_stricmp(key, "hudproject")   == 0) g_cfg.hud_project    = (v != 0.0);
        else if (_stricmp(key, "hudtrimyaw")   == 0) g_cfg.hud_trim_yaw   = (float)v;
        else if (_stricmp(key, "hudtrimpitch") == 0) g_cfg.hud_trim_pitch = (float)v;
        else if (_stricmp(key, "mapmenuback")  == 0) g_cfg.map_menu_back  = (int)strtol(val, nullptr, 0);
        else if (_stricmp(key, "menusuppress") == 0) g_cfg.menu_suppress  = (v != 0.0);
        else if (_stricmp(key, "menudetect")   == 0) g_cfg.menu_detect    = (v != 0.0);
        else if (_stricmp(key, "menudump")     == 0) g_cfg.menu_dump      = (v != 0.0);
        else if (_stricmp(key, "aimdraw")      == 0) g_cfg.aim_draw       = (v != 0.0);
        else if (_stricmp(key, "aimdrawr")     == 0) g_cfg.aim_draw_r     = clampf((float)v, 0.5f, 200.0f);
        else if (_stricmp(key, "aimdrawseg")   == 0) g_cfg.aim_draw_seg   = (int)v;
        else if (_stricmp(key, "aimdrawdur")   == 0) g_cfg.aim_draw_dur   = clampf((float)v, 0.0f, 1.0f);
        else if (_stricmp(key, "aimdrawth")    == 0) g_cfg.aim_draw_th    = clampf((float)v, 0.5f, 20.0f);
        else if (_stricmp(key, "aimdrawcr")    == 0) g_cfg.aim_draw_cr    = clampf((float)v, 0.0f, 1.0f);
        else if (_stricmp(key, "aimdrawcg")    == 0) g_cfg.aim_draw_cg    = clampf((float)v, 0.0f, 1.0f);
        else if (_stricmp(key, "aimdrawcb")    == 0) g_cfg.aim_draw_cb    = clampf((float)v, 0.0f, 1.0f);
        else if (_stricmp(key, "aimmesh")      == 0) g_cfg.aim_mesh       = (v != 0.0);
        else if (_stricmp(key, "aimmeshscale") == 0) g_cfg.aim_mesh_scale = clampf((float)v, 0.005f, 5.0f);
        else if (_stricmp(key, "aimmeshunlit") == 0) g_cfg.aim_mesh_unlit = (v != 0.0);
        else if (_stricmp(key, "mathunt")      == 0) g_cfg.mat_hunt       = (v != 0.0);
        else if (_stricmp(key, "aimparkview")  == 0) g_cfg.aim_park_view  = (v != 0.0);
        else if (_stricmp(key, "aimrtwidget")  == 0) g_cfg.aim_rt_from_widget = (v != 0.0);
        else if (_stricmp(key, "texhunt")      == 0) g_cfg.tex_hunt          = (v != 0.0);
        else if (_stricmp(key, "aimtexparam")  == 0) {
            strncpy_s(g_cfg.aim_tex_param, sizeof(g_cfg.aim_tex_param), val, _TRUNCATE);
            size_t n = strlen(g_cfg.aim_tex_param);
            while (n > 0) {
                const unsigned char c = (unsigned char)g_cfg.aim_tex_param[n - 1];
                if (c != 13 && c != 10 && c != 32 && c != 9) break;
                g_cfg.aim_tex_param[--n] = 0;
            }
        }
        else if (_stricmp(key, "matdump")      == 0) {
            strncpy_s(g_cfg.mat_dump, sizeof(g_cfg.mat_dump), val, _TRUNCATE);
            size_t n = strlen(g_cfg.mat_dump);
            while (n > 0) {
                const unsigned char c = (unsigned char)g_cfg.mat_dump[n - 1];
                if (c != 13 && c != 10 && c != 32 && c != 9) break;
                g_cfg.mat_dump[--n] = 0;
            }
        }
        else if (_stricmp(key, "aimrt")        == 0) g_cfg.aim_rt          = (v != 0.0);
        else if (_stricmp(key, "aimrtsize")    == 0) g_cfg.aim_rt_size     = (int)v;
        else if (_stricmp(key, "vrinactivity")  == 0) g_cfg.vr_inactivity = clampf((float)v, 0.0f, 100.0f);
        else if (_stricmp(key, "aimrate")       == 0) g_cfg.aim_rate_render = (v != 0.0);
        else if (_stricmp(key, "aimmeshpath")  == 0) {
            strncpy_s(g_cfg.aim_mesh_path, sizeof(g_cfg.aim_mesh_path), val, _TRUNCATE);
            size_t n = strlen(g_cfg.aim_mesh_path);
            while (n > 0) {
                const unsigned char c = (unsigned char)g_cfg.aim_mesh_path[n - 1];
                if (c != 13 && c != 10 && c != 32 && c != 9) break;
                g_cfg.aim_mesh_path[--n] = 0;
            }
        }
        else if (_stricmp(key, "aimtex")       == 0) g_cfg.aim_tex        = (v != 0.0);
        else if (_stricmp(key, "aimmeshmat")    == 0) g_cfg.aim_mesh_override = (v != 0.0);
        else if (_stricmp(key, "aimmeshparent") == 0) {
            strncpy_s(g_cfg.aim_mesh_parent, sizeof(g_cfg.aim_mesh_parent), val, _TRUNCATE);
            size_t n = strlen(g_cfg.aim_mesh_parent);
            while (n > 0) {
                const unsigned char c = (unsigned char)g_cfg.aim_mesh_parent[n - 1];
                if (c != 13 && c != 10 && c != 32 && c != 9) break;
                g_cfg.aim_mesh_parent[--n] = 0;
            }
        }
        else if (_stricmp(key, "aimtexfile")   == 0) {
            strncpy_s(g_cfg.aim_tex_file, sizeof(g_cfg.aim_tex_file), val, _TRUNCATE);
            size_t n = strlen(g_cfg.aim_tex_file);
            while (n > 0) {
                const unsigned char c = (unsigned char)g_cfg.aim_tex_file[n - 1];
                if (c != 13 && c != 10 && c != 32 && c != 9) break;
                g_cfg.aim_tex_file[--n] = 0;
            }
        }
        else if (_stricmp(key, "aimtexrotp")   == 0) g_cfg.aim_tex_rot_p  = (float)v;
        else if (_stricmp(key, "aimtexroty")   == 0) g_cfg.aim_tex_rot_y  = (float)v;
        else if (_stricmp(key, "aimtexrotr")   == 0) g_cfg.aim_tex_rot_r  = (float)v;
        else if (_stricmp(key, "aimtexpath")   == 0) {
            strncpy_s(g_cfg.aim_tex_path, sizeof(g_cfg.aim_tex_path), val, _TRUNCATE);
            size_t n = strlen(g_cfg.aim_tex_path);
            while (n > 0) {
                const unsigned char c = (unsigned char)g_cfg.aim_tex_path[n - 1];
                if (c != 13 && c != 10 && c != 32 && c != 9) break;
                g_cfg.aim_tex_path[--n] = 0;
            }
        }
        else if (_stricmp(key, "aimmeshcr")    == 0) g_cfg.aim_mesh_cr    = clampf((float)v, 0.0f, 1.0f);
        else if (_stricmp(key, "aimmeshcg")    == 0) g_cfg.aim_mesh_cg    = clampf((float)v, 0.0f, 1.0f);
        else if (_stricmp(key, "aimmeshcb")    == 0) g_cfg.aim_mesh_cb    = clampf((float)v, 0.0f, 1.0f);
        // One chain link for the whole scope family: MSVC counts every `else if` toward its
        // 128-deep block limit (C1061), and this chain is already close to it. New key FAMILIES
        // should follow this pattern rather than growing the chain.
        else if (parse_scope_key(key, v)) {}
        else if (_stricmp(key, "hudhide")      == 0) g_cfg.hud_hide       = (v != 0.0);
        else if (_stricmp(key, "aimwidget")      == 0) g_cfg.aim_widget      = (v != 0.0);
        else if (_stricmp(key, "aimwidgetdraw")  == 0) g_cfg.aim_widget_draw = clampf((float)v, 16.0f, 2048.0f);
        else if (_stricmp(key, "aimwidgetscale") == 0) g_cfg.aim_widget_scale= clampf((float)v, 0.005f, 5.0f);
        else if (_stricmp(key, "aimwidgetflip")  == 0) g_cfg.aim_widget_flip = (v != 0.0);
        else if (_stricmp(key, "aimwidgetblend") == 0) g_cfg.aim_widget_blend = (int)v;
        else if (_stricmp(key, "aimwidgetmat")   == 0) g_cfg.aim_widget_mat   = (v != 0.0);
        else if (_stricmp(key, "aimwidgetbg")    == 0) g_cfg.aim_widget_bg    = (v != 0.0);
        else if (_stricmp(key, "aimwidgetfresh") == 0) g_cfg.aim_widget_fresh = (v != 0.0);
        else if (_stricmp(key, "aimwidgetclass") == 0) {
            // Trim trailing CR/LF/space. The file is CRLF, so an untrimmed value carries a CR
            // and matches no class at all.
            strncpy_s(g_cfg.aim_widget_class, sizeof(g_cfg.aim_widget_class), val, _TRUNCATE);
            size_t n = strlen(g_cfg.aim_widget_class);
            while (n > 0) {
                const unsigned char c = (unsigned char)g_cfg.aim_widget_class[n - 1];
                if (c != 13 && c != 10 && c != 32 && c != 9) break;   // CR, LF, space, tab
                g_cfg.aim_widget_class[--n] = 0;
            }
        }
        // Ceiling raised 64 -> 1024 on 2026-08-09 to match aimwidgetgain. The two multiply, and the
        // scene term they fight varies by roughly 64x between a bright exterior and shade -- so the
        // BRIGHT end is where headroom runs out, and tuning had reached the old ceiling with the
        // beach still reading dark. A clamp that low also failed silently: values above it were
        // reduced with no log line, so raising the number appeared to do nothing and sent the
        // investigation looking for a phantom multiplier.
        else if (_stricmp(key, "aimwidgettint")  == 0) g_cfg.aim_widget_tint  = clampf((float)v, 0.0f, 1024.0f);
        else if (_stricmp(key, "aimwidgetalpha") == 0) g_cfg.aim_widget_alpha = clampf((float)v, 0.0f, 1.0f);
        else if (_stricmp(key, "aimwidgetgain")  == 0) g_cfg.aim_widget_gain  = clampf((float)v, 0.0f, 1024.0f);
        // The map*/rgrip* family moved to parse_melee_key's flat region -- see the note there.
        else if (_stricmp(key, "perflog")       == 0) g_cfg.perf_log        = (v != 0.0);
        else if (_stricmp(key, "retsweepms")    == 0) g_cfg.ret_sweep_ms    = clampf((float)v, 0.0f, 250.0f);
        else if (_stricmp(key, "aimtrace")      == 0) g_cfg.aim_trace       = (v != 0.0);
        else if (_stricmp(key, "memscan")       == 0) g_cfg.mem_scan        = (v != 0.0);
        else if (_stricmp(key, "aimdig")        == 0) g_cfg.aim_dig         = (v != 0.0);
        else if (_stricmp(key, "aimwatch")      == 0) g_cfg.aim_watch       = (v != 0.0);
        else if (_stricmp(key, "aimwatchaddr")  == 0) g_cfg.aim_watch_addr  = (uint64_t)strtoull(val, nullptr, 0);
        else if (_stricmp(key, "memscanvals")   == 0) strncpy_s(g_cfg.mem_scan_vals, val, _TRUNCATE);
        else if (_stricmp(key, "measrate")      == 0) g_cfg.meas_rate_fixed = clampf((float)v, 0.0f, 1000.0f);
        else if (_stricmp(key, "gainseed")      == 0) g_cfg.gain_seed       = clampf((float)v, 0.0f, 1000.0f);
        else if (_stricmp(key, "stickscale")    == 0) g_cfg.stick_scale     = clampf((float)v, 0.05f, 1.0f);
        else if (_stricmp(key, "stickdz")       == 0) g_cfg.stick_dz        = clampf((float)v, 0.0f, 0.5f);
        else if (_stricmp(key, "plantfulldps")  == 0) g_cfg.plant_full_dps  = clampf((float)v, 0.0f, 2000.0f);
        else if (_stricmp(key, "aimtau")        == 0) g_cfg.aim_tau_s       = clampf((float)v, 0.0f, 2.0f);
        else if (_stricmp(key, "aimdecel")      == 0) g_cfg.aim_decel       = clampf((float)v, 0.0f, 20000.0f);
        else if (_stricmp(key, "aimdirect")     == 0) g_cfg.aim_direct      = (v != 0.0);
        else if (_stricmp(key, "aimfreezelost") == 0) g_cfg.aim_freeze_lost = (v != 0.0);
        else if (_stricmp(key, "aimcache")      == 0) g_cfg.aim_cache       = (v != 0.0);
        else if (_stricmp(key, "aimchain")      == 0) g_cfg.aim_chain       = (v != 0.0);
        else if (_stricmp(key, "dirgrip")     == 0) g_cfg.rig_dir_grip_deg  = clampf((float)v, -180.0f, 180.0f);
        else if (_stricmp(key, "dirgripyaw")  == 0) g_cfg.rig_dir_grip_yaw  = clampf((float)v, -180.0f, 180.0f);
        else if (_stricmp(key, "dirgriproll") == 0) g_cfg.rig_dir_grip_roll = clampf((float)v, -180.0f, 180.0f);
        else if (_stricmp(key, "diroffx")     == 0) g_cfg.rig_dir_off_x     = clampf((float)v, -100.0f, 100.0f);
        else if (_stricmp(key, "diroffy")     == 0) g_cfg.rig_dir_off_y     = clampf((float)v, -100.0f, 100.0f);
        else if (_stricmp(key, "diroffz")     == 0) g_cfg.rig_dir_off_z     = clampf((float)v, -100.0f, 100.0f);
        else if (_stricmp(key, "memscanmax")  == 0) g_cfg.mem_scan_max   = (int)v;
        else if (_stricmp(key, "memdiff")     == 0) g_cfg.mem_diff       = (int)v;
        else if (_stricmp(key, "blamaim")     == 0) g_cfg.blam_aim       = (int)v;
        else if (_stricmp(key, "blamyaw")     == 0) g_cfg.blam_yaw_off   = (float)v;
        else if (_stricmp(key, "stickdither") == 0) g_cfg.stick_dither    = (v != 0.0);
        else if (_stricmp(key, "aimquat")      == 0) g_cfg.aim_quat        = (v != 0.0);
        else if (_stricmp(key, "aimquatsrc")   == 0) g_cfg.aim_quat_src    = (v != 0.0);
        else if (_stricmp(key, "aimdirectsignx") == 0) g_cfg.aim_direct_sign_x = (v < 0.0) ? -1.0f : 1.0f;
        else if (_stricmp(key, "aimdirectsigny") == 0) g_cfg.aim_direct_sign_y = (v < 0.0) ? -1.0f : 1.0f;
        else if (_stricmp(key, "vrsens")        == 0) g_cfg.vr_sens         = (int)clampf((float)v, 0.0f, 20.0f);
        else if (_stricmp(key, "vrdeadzone")    == 0) g_cfg.vr_deadzone     = clampf((float)v, -1.0f, 100.0f);
        else if (_stricmp(key, "rigfast")       == 0) g_cfg.rig_fast        = (v != 0.0);
        else if (_stricmp(key, "shell")         == 0) g_cfg.shell_drive     = (v != 0.0);
        else if (_stricmp(key, "stickmode")     == 0) g_cfg.stick_mode      = (v != 0.0);
        else if (_stricmp(key, "stickforce")    == 0) g_cfg.stick_force     = (int)v;
        else if (_stricmp(key, "stickonfoot")   == 0) g_cfg.stick_onfoot    = (v != 0.0);
        else if (_stricmp(key, "hidearms")      == 0) g_cfg.hide_arms       = (v != 0.0);
        else if (_stricmp(key, "showarms")      == 0) g_cfg.show_arms       = (v != 0.0);
        else if (_stricmp(key, "showweapon")    == 0) g_cfg.show_weapon     = (v != 0.0);
        else if (_stricmp(key, "stickon")       == 0) g_cfg.stick_on_s      = clampf((float)v, 0.1f, 30.0f);
        else if (_stricmp(key, "stickoff")      == 0) g_cfg.stick_off_s     = clampf((float)v, 0.03f, 30.0f);
        else if (_stricmp(key, "brake")         == 0) g_cfg.brake_enabled   = (v != 0.0);
        else if (_stricmp(key, "brakemode")     == 0) g_cfg.brake_mode      = (int)v;
        else if (_stricmp(key, "brakemask")     == 0) g_cfg.brake_mask      = (int)strtol(val, nullptr, 0);
        else if (_stricmp(key, "brakekey")      == 0) g_cfg.brake_key       = (int)strtol(val, nullptr, 0);
        else if (_stricmp(key, "cutscene2d")    == 0) g_cfg.cutscene_2d     = (int)v;
        else if (_stricmp(key, "cuthint")       == 0) g_cfg.cut_hint        = (v != 0.0);
        else if (_stricmp(key, "cuthintdist")   == 0) g_cfg.cut_hint_dist   = cm_to_m(v, 50.0f,  50.0f,  500.0f, "cuthintdist");
        else if (_stricmp(key, "cuthintdrop")   == 0) g_cfg.cut_hint_drop   = cm_to_m(v,  3.0f, -200.0f, 200.0f, "cuthintdrop");
        else if (_stricmp(key, "cuthintw")      == 0) g_cfg.cut_hint_w      = cm_to_m(v, 30.0f,  30.0f,  300.0f, "cuthintw");
}

bool parse_config_file(const char* path) {
    FILE* f = nullptr;
    if (fopen_s(&f, path, "rb") != 0 || f == nullptr) return false;

    // ---- PER-FILE SCOPE FOR THE wpnfix SCHEMA STAMP.
    //
    // RESET IT HERE, and the reset is the point. g_cfg is not wiped between files in one load pass,
    // so a stamp read out of the shipped halo_vr.cfg would still be standing when the NEXT file's
    // wpnfix lines arrive -- and would certify them. That is not hypothetical: a player who drops
    // blindcowboy24's halo_vr_weapons.cfg into their profile has a file full of UE-frame wpnfix
    // lines and no stamp of its own, and inheriting ours would apply every one of them in the wrong
    // frame. "File-borne stamp" has to mean each file stamps its own lines or it means nothing.
    s_wpnfix_file_ver = 0;
    // Same scope, same reset, for the support-hand fix. `handfix` only ever appears in
    // halo_vr_calib.cfg today, but "each file stamps its own lines" has to hold unconditionally or
    // it holds for nothing -- the wpnfix note above is the whole argument.
    s_handfix_file_ver = 0;
    // Is THIS the capture file? Decides whether the entries it contributes are the player's own and
    // therefore belong in the rewrite -- see WeaponFix::captured.
    s_wpnfix_from_capture =
        (g_wpn_calib_path[0] != 0) && (_stricmp(path, g_wpn_calib_path) == 0);

    char line[256];
    while (fgets(line, sizeof(line), f) != nullptr) {
        if (line[0] == '#' || line[0] == '\r' || line[0] == '\n') continue;
        char* eq = strchr(line, '=');
        if (eq == nullptr) continue;
        *eq = '\0';
        const char* key = line;
        const char* val = eq + 1;
        const double v = atof(val);

        if      (_stricmp(key, "enabled") == 0) g_cfg.enabled     = (v != 0.0);
        else if (_stricmp(key, "pitch")   == 0) g_cfg.drive_pitch = (v != 0.0);
        else if (_stricmp(key, "floor")   == 0) g_cfg.floor       = clampf((float)v, 0.0f, 1.0f);
        else if (_stricmp(key, "full")    == 0) g_cfg.full_deg    = (v > 0.1) ? (float)v : g_cfg.full_deg;
        else if (_stricmp(key, "max")     == 0) g_cfg.max_out     = clampf((float)v, 0.0f, 1.0f);
        else if (_stricmp(key, "dead")    == 0) g_cfg.dead_deg    = clampf((float)v, 0.0f, 45.0f);
        else if (_stricmp(key, "ffgain")  == 0) g_cfg.ff_gain     = clampf((float)v, 0.0f, 2.0f);
        else if (_stricmp(key, "dgain")   == 0) g_cfg.d_gain      = clampf((float)v, 0.0f, 1.0f);
        else if (_stricmp(key, "ffsmoothms") == 0) g_cfg.ff_smooth_ms = clampf((float)v, 0.0f, 500.0f);
        else if (_stricmp(key, "dsmoothms")  == 0) g_cfg.d_smooth_ms  = clampf((float)v, 0.0f, 500.0f);
        else if (_stricmp(key, "ffsmooth")== 0) g_cfg.ff_smooth_ms = alpha_to_tau_ms((float)v);
        else if (_stricmp(key, "dsmooth") == 0) g_cfg.d_smooth_ms  = alpha_to_tau_ms((float)v);
        else if (_stricmp(key, "deadhyst")== 0) g_cfg.dead_hyst   = clampf((float)v, 0.0f, 1.0f);
        else if (_stricmp(key, "ffpitchcomp") == 0) g_cfg.ff_pitch_comp = (v != 0.0);
        else if (_stricmp(key, "xdist") == 0)
            g_cfg.xdist_m = cm_to_m(v, 50.0f, 100.0f, 10000.0f, "xdist");
        else if (_stricmp(key, "aimconverge")    == 0) g_cfg.aim_converge = (v != 0.0);
        else if (_stricmp(key, "aimconvergetau") == 0)
            g_cfg.aim_converge_tau_ms  = clampf((float)v, 0.0f, 1000.0f);
        else if (_stricmp(key, "aimconvergemin") == 0)
            g_cfg.aim_converge_min_cm  = clampf((float)v, 0.0f, 200.0f);
        else if (_stricmp(key, "aimconvergemax") == 0)
            g_cfg.aim_converge_max_deg = clampf((float)v, 0.0f, 90.0f);
        else if (_stricmp(key, "aimconvlog")     == 0) g_cfg.aim_conv_log = (int)v;
        else if (_stricmp(key, "viewlock")  == 0) g_cfg.view_lock  = (v != 0.0);
        else if (_stricmp(key, "locksign")  == 0) g_cfg.lock_sign  = (v < 0.0) ? -1.0f : 1.0f;
        else if (_stricmp(key, "aimorigin") == 0) g_cfg.aim_origin = (v != 0.0) ? 1 : 0;
        // aimhand=left|right, also accepting 1/0 so it behaves like every other key here.
        else if (_stricmp(key, "aimhand")  == 0) {
            g_cfg.aim_left_hand = (_stricmp(val, "left") == 0) || (_stricmp(val, "l") == 0) ||
                                  (val[0] >= '1' && val[0] <= '9');
        }
        else if (_stricmp(key, "turnmode")  == 0) g_cfg.turn_mode  = (int)v;
        else if (_stricmp(key, "snapdeg")   == 0) g_cfg.snap_deg   = clampf((float)v, 5.0f, 90.0f);
        else if (_stricmp(key, "smoothdps") == 0) g_cfg.smooth_dps = clampf((float)v, 10.0f, 360.0f);
        else if (_stricmp(key, "turndz")    == 0) g_cfg.turn_dz    = clampf((float)v, 0.1f, 0.95f);
        else if (_stricmp(key, "rig")       == 0) g_cfg.rig_enabled = (v != 0.0);
        else if (_stricmp(key, "rigloc")    == 0) g_cfg.rig_loc     = (v != 0.0);
        else if (_stricmp(key, "grip")      == 0) g_cfg.grip_deg    = clampf((float)v, -180.0f, 180.0f);
        else if (_stricmp(key, "rigscale")  == 0) g_cfg.rig_scale   = clampf((float)v, 0.0f, 500.0f);
        else if (_stricmp(key, "rigclamp")  == 0) g_cfg.rig_clamp   = clampf((float)v, 0.0f, 200.0f);
        else if (_stricmp(key, "rigtest")   == 0) g_cfg.rig_test_cm = clampf((float)v, -200.0f, 200.0f);
        else if (_stricmp(key, "gripyaw")   == 0) g_cfg.grip_yaw    = (float)v;
        else if (_stricmp(key, "griproll")  == 0) g_cfg.grip_roll   = (float)v;
        else if (_stricmp(key, "offx")      == 0) g_cfg.off_x       = clampf((float)v, -100.0f, 100.0f);
        else if (_stricmp(key, "offy")      == 0) g_cfg.off_y       = clampf((float)v, -100.0f, 100.0f);
        else if (_stricmp(key, "offz")      == 0) g_cfg.off_z       = clampf((float)v, -100.0f, 100.0f);
        else if (_stricmp(key, "pivx")      == 0) g_cfg.piv_x       = clampf((float)v, -300.0f, 300.0f);
        else if (_stricmp(key, "pivy")      == 0) g_cfg.piv_y       = clampf((float)v, -300.0f, 300.0f);
        else if (_stricmp(key, "pivz")      == 0) g_cfg.piv_z       = clampf((float)v, -300.0f, 300.0f);
        else if (_stricmp(key, "pivauto")   == 0) g_cfg.piv_auto    = (v != 0.0);
    else if (_stricmp(key, "wpndiag")   == 0) g_cfg.wpn_diag    = (v != 0.0);
    else if (_stricmp(key, "wpnroll")   == 0) g_cfg.wpn_roll    = (v != 0.0);
    else if (_stricmp(key, "calibroll") == 0) g_cfg.calib_roll  = (v != 0.0);
    else if (_stricmp(key, "rollstatic")== 0) g_cfg.roll_static = clampf((float)v, -999.0f, 180.0f);
    else if (_stricmp(key, "handsdiag") == 0) g_cfg.hands_diag  = (v != 0.0);
    else if (_stricmp(key, "rigsockrot")== 0) g_cfg.rig_sock_rot= (v != 0.0);
    else if (_stricmp(key, "rigsocket") == 0) g_cfg.rig_socket  = (v != 0.0);
        else if (_stricmp(key, "pivviz")    == 0) g_cfg.piv_viz     = (v != 0.0);
        else if (_stricmp(key, "pivdraw")   == 0) g_cfg.piv_draw    = clampf((float)v, 0.0f, 50.0f);
        else if (_stricmp(key, "aimreticule")     == 0) g_cfg.aim_reticule      = (v != 0.0);
        else if (_stricmp(key, "aimreticulesrc") == 0) g_cfg.aim_reticule_src = (int)v;
        else if (_stricmp(key, "aimhidenative") == 0) g_cfg.aim_hide_native = (v != 0.0);
        else if (_stricmp(key, "aimsrc")       == 0) g_cfg.aim_src         = (int)v;
        else if (_stricmp(key, "aimrolllog")   == 0) g_cfg.aim_roll_log    = (int)v;
        else if (_stricmp(key, "aimreticuletrace") == 0) g_cfg.aim_reticule_trace = (v != 0.0);
        else if (_stricmp(key, "aimreticuletracemax") == 0)
            g_cfg.aim_reticule_trace_max = clampf((float)v, 100.0f, 100000.0f);
        else if (_stricmp(key, "aimreticuletracechannel") == 0)
            g_cfg.aim_reticule_trace_channel = (int)v;
        else if (_stricmp(key, "aimreticulemaxdist") == 0)
            g_cfg.aim_reticule_max_dist = clampf((float)v, 50.0f, 100000.0f);
        else if (_stricmp(key, "aimreticuleminscale") == 0)
            g_cfg.aim_reticule_min_scale = clampf((float)v, 0.001f, 10.0f);
        else if (_stricmp(key, "aimreticulemaxscale") == 0)
            g_cfg.aim_reticule_max_scale = clampf((float)v, 0.001f, 20.0f);
        else if (_stricmp(key, "aimreticuleminscaledist") == 0)
            g_cfg.aim_reticule_min_scale_dist = clampf((float)v, 1.0f, 10000.0f);
        else if (_stricmp(key, "aimreticulesurfaceoff") == 0)
            g_cfg.aim_reticule_surface_off = clampf((float)v, 0.0f, 500.0f);
        else if (_stricmp(key, "aimreticuledivdeg") == 0) g_cfg.aim_reticule_div_deg = clampf((float)v, 0.5f, 90.0f);
        else if (_stricmp(key, "aimreticuledivms") == 0) g_cfg.aim_reticule_div_ms = clampf((float)v, 0.0f, 5000.0f);
        else if (_stricmp(key, "aimreticulesmoothslow") == 0) g_cfg.aim_reticule_smooth_slow_dps = clampf((float)v, 0.0f, 500.0f);
        else if (_stricmp(key, "aimreticulesmoothfast") == 0) g_cfg.aim_reticule_smooth_fast_dps = clampf((float)v, 1.0f, 1000.0f);
        else if (_stricmp(key, "aimreticulesmoothctrlms") == 0) g_cfg.aim_reticule_smooth_ctrl_ms = clampf((float)v, 0.0f, 500.0f);
        else if (_stricmp(key, "aimreticulesmoothms") == 0) g_cfg.aim_reticule_smooth_ms = clampf((float)v, 0.0f, 500.0f);
        else if (_stricmp(key, "aimreticulesmoothctrl") == 0) g_cfg.aim_reticule_smooth_ctrl_ms = alpha_to_tau_ms((float)v);
        else if (_stricmp(key, "aimreticulesmooth") == 0) g_cfg.aim_reticule_smooth_ms = alpha_to_tau_ms((float)v);
        else if (_stricmp(key, "aimreticuledist") == 0) g_cfg.aim_reticule_dist = clampf((float)v, 100.0f, 10000.0f);
        else if (_stricmp(key, "aimreticuledistveh") == 0) g_cfg.aim_reticule_dist_veh = clampf((float)v, 0.0f, 20000.0f);
        else if (_stricmp(key, "aimreticulescaleveh") == 0) g_cfg.aim_reticule_scale_veh = clampf((float)v, 0.05f, 5.0f);
        else if (_stricmp(key, "aimreticulecm")   == 0) g_cfg.aim_reticule_cm   = clampf((float)v, 1.0f, 200.0f);
        else if (_stricmp(key, "aimreticulelua")  == 0) g_cfg.aim_reticule_lua  = (v != 0.0);
        else if (_stricmp(key, "aimreticulecube") == 0) g_cfg.aim_reticule_cube = (v != 0.0);
        else if (_stricmp(key, "pivcube")   == 0) g_cfg.piv_cube    = (v != 0.0);
        else if (_stricmp(key, "pivcubescale") == 0) g_cfg.piv_cube_scale = clampf((float)v, 1.0f, 100.0f);
        else if (_stricmp(key, "pivvizscale") == 0) g_cfg.piv_viz_scale = clampf((float)v, 0.01f, 1.0f);
        else if (_stricmp(key, "pivadjx")   == 0) g_cfg.piv_adj_x   = clampf((float)v, -100.0f, 100.0f);
        else if (_stricmp(key, "pivadjy")   == 0) g_cfg.piv_adj_y   = clampf((float)v, -100.0f, 100.0f);
        else if (_stricmp(key, "pivadjz")   == 0) g_cfg.piv_adj_z   = clampf((float)v, -100.0f, 100.0f);
        else if (_stricmp(key, "pivsocket") == 0) {
            // The one string-valued key: copy the raw text and trim the line ending.
            strncpy_s(g_cfg.piv_socket, sizeof(g_cfg.piv_socket), val, _TRUNCATE);
            for (char* p = g_cfg.piv_socket; *p != '\0'; ++p) {
                if (*p == '\r' || *p == '\n' || *p == ' ' || *p == '\t') { *p = '\0'; break; }
            }
        }
        else if (_stricmp(key, "recenter")  == 0) g_cfg.recenter    = (float)v;
        else if (_stricmp(key, "lockreprime") == 0) g_cfg.lock_reprime = (float)v;
        // base 0 so the mask can be written as 0x0020 (readable) or 32 (not).
        else if (_stricmp(key, "calibkey")  == 0) g_cfg.calib_key   = (int)strtol(val, nullptr, 0);
        else if (_stricmp(key, "aimcalibkey") == 0) g_cfg.aim_calib_key = (int)strtol(val, nullptr, 0);
        else if (_stricmp(key, "blamctllog")  == 0) g_cfg.blam_ctl_log  = (int)v;
        else if (_stricmp(key, "hmdleash")     == 0) g_cfg.hmd_leash      = (v != 0.0);
        // cm_floor 5: a leash radius under 5 cm is behaviourally identical to 0, so no one can have
        // MEANT one in centimetres -- which makes the reinterpretation safe as well as necessary.
        else if (_stricmp(key, "hmdleashlat")  == 0) g_cfg.hmd_leash_lat  = cm_to_m(v, 5.0f, 0.0f, 500.0f, "hmdleashlat");
        else if (_stricmp(key, "hmdleashvert") == 0) g_cfg.hmd_leash_vert = cm_to_m(v, 5.0f, 0.0f, 500.0f, "hmdleashvert");
        else if (_stricmp(key, "modekey")   == 0) g_cfg.mode_key    = (int)strtol(val, nullptr, 0);
        else if (_stricmp(key, "dbgmark")   == 0) g_cfg.dbg_mark    = (int)v;
    else if (_stricmp(key, "killkey")    == 0) g_cfg.kill_key    = (int)strtol(val, nullptr, 0);
        else if (_stricmp(key, "aimoffyaw")   == 0) { g_cfg.aim_off_yaw   = (float)v; g_cfg.aim_off_valid = true; }
        else if (_stricmp(key, "aimoffpitch") == 0) { g_cfg.aim_off_pitch = (float)v; g_cfg.aim_off_valid = true; }
        else if (_stricmp(key, "calibrelative") == 0) g_cfg.calib_relative = (v != 0.0);
        else if (_stricmp(key, "calibver")      == 0) g_cfg.calib_ver      = (int)v;
        else if (_stricmp(key, "aimcalibver")   == 0) g_cfg.aim_calib_ver  = (int)v;
        else if (_stricmp(key, "rigmode")   == 0) g_cfg.rig_mode    = (int)v;
        else if (_stricmp(key, "rigwoffhold") == 0) g_cfg.rigw_off_hold = (int)v;
        else if (_stricmp(key, "riganchor") == 0) g_cfg.rig_body_anchor = (v != 0.0);
        else if (_stricmp(key, "rigviewyaw") == 0) g_cfg.rig_view_yaw  = clampf((float)v, -1.0f, 1.0f);
        else if (_stricmp(key, "rigneutral") == 0) g_cfg.rig_neutral   = (v != 0.0);
        else if (_stricmp(key, "rigturn")   == 0) g_cfg.rig_turn     = clampf((float)v, -1.0f, 1.0f);
        else if (_stricmp(key, "aimturn")   == 0) g_cfg.aim_turn     = clampf((float)v, -1.0f, 1.0f);
        else if (_stricmp(key, "moverot")   == 0) g_cfg.move_rot     = clampf((float)v, -1.0f, 1.0f);
        else if (_stricmp(key, "movesrc")   == 0) g_cfg.move_src     = (int)v;
        else if (_stricmp(key, "movelive")  == 0) g_cfg.move_live    = (int)v;
        else if (_stricmp(key, "movesmooth") == 0) g_cfg.move_smooth = clampf((float)v, 0.0f, 1.0f);
        else if (_stricmp(key, "moveresid") == 0) g_cfg.move_resid   = (int)v;
        else if (_stricmp(key, "requirehmd") == 0) g_cfg.require_hmd  = (v != 0.0);
        else if (_stricmp(key, "attachmode") == 0) g_cfg.attach_mode  = (int)v;
        else if (_stricmp(key, "rigrender")  == 0) g_cfg.rig_render   = (v != 0.0);
        else parse_config_key_2(key, val, v);
    }
    fclose(f);
    return true;
}

// Calibration results live in their OWN file, applied after the main config so they win.
// Separate on purpose: the plugin rewrites this file every time you calibrate, and rewriting
// halo_vr.cfg would destroy the comments that document every other knob.
// Delete it to go back to hand-tuned values.
char g_calib_path[MAX_PATH] = {0};

// The right-hand file, kept separately so the left hand can be SEEDED from it. Set at startup
// before handedness is resolved.
char g_calib_path_right[MAX_PATH] = {0};

// Set once a pivot has been MEASURED from two calibration samples. From then on the socket guess
// is switched off -- otherwise the next rig acquisition would overwrite a measured value with an
// estimated one, which would look like the calibration silently degrading.
bool g_pivot_from_calib = false;

// TRUE when an aim offset is loaded with no explicit aimcalibver stamp, so its schema is a guess.
// See load_config() below; the tick reports it once.
bool g_calib_stamp_ambiguous = false;

// Last-write time of one file, as an opaque comparable. Absent reads as 0, which is deliberately a
// legitimate value rather than an error: deleting halo_vr_calib.cfg is a documented way to drop back
// to the shipped calibration, so its disappearance has to register as a change.
uint64_t cfg_file_stamp(const char* path) {
    if (path == nullptr || path[0] == 0) return 0;
    WIN32_FILE_ATTRIBUTE_DATA fad{};
    if (!GetFileAttributesExA(path, GetFileExInfoStandard, &fad)) return 0;
    return ((uint64_t)fad.ftLastWriteTime.dwHighDateTime << 32) |
            (uint64_t)fad.ftLastWriteTime.dwLowDateTime;
}

// EVERY config file, in load order, weakest override first. THE single source of truth for both
// halves of load_config(): the change-detection gate stamps this list, and the parse sequence
// walks this list. Keeping them derived from one array is what stops a file from being parsed
// without being watched -- the failure mode there is silent (see the warning in load_config).
//
// These are ADDRESSES of the path buffers, not copies of their contents, which matters because
// g_calib_path is rewritten in place by select_calib_for_hand() when the player switches to the
// left hand. The list follows that change with nothing to re-register.
//
// Order is load order and must stay that way: index 0 is the required base file, everything after
// it is an override applied on top.
const char* const kConfigFiles[] = {
    g_cfg_path,        // shipped calibration data -- REQUIRED, see load_config
    g_user_cfg_path,   // the player's own overrides; never shipped, survives updates
    g_dev_cfg_path,    // dev/troubleshooting catalog; ships all-commented, beats the user file
    g_calib_path,      // in-game calibration gestures; own their keys outright
    // Per-weapon captures (halo_vr_weapons.cfg). LAST on purpose: a captured delta for a weapon
    // that also has a hand-written wpnoff entry must replace it in the matcher, not sit behind it.
    // Listed here rather than loaded by its own call so the gate above stamps it too -- an
    // unstamped file is one whose edits are only ever read when some OTHER file happens to change.
    g_wpn_calib_path,
};
constexpr int kConfigFileCount = (int)(sizeof(kConfigFiles) / sizeof(kConfigFiles[0]));

void load_config() {
    // ---- DO NOT RE-READ FILES THAT HAVE NOT CHANGED.
    //
    // This runs every ~60 ticks, forever, and unconditionally parsed three files off disk on the
    // GAME THREAD. Normally that is ~0.45 ms and invisible. Measured under disk contention it hit
    // 143.9 ms and 108.7 ms -- roughly 300x -- and a blocking read every ~1.7 s that occasionally
    // costs 100 ms+ is a periodic hitch you can set your watch by. It was the largest single
    // contributor to TICK (all) in two of three measured windows.
    //
    // Stat calls instead. Live editing is unaffected: touching a file changes its write time and
    // the next check parses immediately, so the edit-and-see loop still works, and writes made by
    // the plugin itself (calibration captures, and the settings menu's writes into
    // halo_vr_user.cfg) reload exactly as before.
    //
    // !! THE GATE AND THE PARSE SEQUENCE WALK THE SAME LIST, and they have to. A file that is
    // parsed but not stamped can never trigger a reload: its edits are read only when some OTHER
    // file happens to change, which reads as "live tuning silently stopped working" with no error
    // and no log line. The in-game settings menu is the sharp edge -- its whole mechanism is
    // writing halo_vr_user.cfg and waiting for the next poll to pick it up, so leaving that file
    // unstamped would make the menu appear to do nothing at all.
    //
    // So kConfigFiles below is the single source of truth for BOTH, and a fifth config file is one
    // edit here rather than two in different functions. See the parse sequence further down.
    {
        uint64_t now[kConfigFileCount];
        for (int i = 0; i < kConfigFileCount; ++i) now[i] = cfg_file_stamp(kConfigFiles[i]);

        static uint64_t prev[kConfigFileCount] = {0};
        static bool     primed = false;

        bool unchanged = primed;
        for (int i = 0; unchanged && i < kConfigFileCount; ++i) unchanged = (now[i] == prev[i]);
        if (unchanged) return;

        for (int i = 0; i < kConfigFileCount; ++i) prev[i] = now[i];
        primed = true;
    }

    // RESET THE WHOLE STRUCT, not three fields of it.
    //
    // parse_config_file() only ASSIGNS when it sees a key, so without this a COMMENTED-OUT value
    // keeps whatever was last parsed for the rest of the session. The file says one thing, the
    // running mod does another, and re-reading never fixes it because there is nothing to read.
    //
    // FIELD-MEASURED 2026-08-13: `blamangles=0` was uncommented for an A/B, commented out again,
    // and stayed 0 for the next twenty minutes -- silently invalidating the control arm of the
    // experiment, and taking a "movement no longer follows my head" observation with it. Every
    // "turn it back off by re-commenting" A/B this file has ever served was contaminated the same
    // way, which is the more expensive half: a moving baseline produces a run of well-reasoned
    // hypotheses that each die on the next test.
    //
    // The three stamps that used to be reset here had already reasoned their way to exactly this
    // conclusion -- "once a file that HAD a stamp is edited to remove it ... the old value would
    // persist in memory and keep certifying data it no longer describes" -- the fix was just
    // scoped to three fields instead of to the struct. Their defaults now come from the struct.
    //
    // CONSEQUENCE, and it is a hard rule: a RUNTIME decision must not live in g_cfg, because this
    // wipes it every ~2 s. The two that are already correct re-apply themselves after the parse --
    // the kill switch (g_kill_override) and the aim-mode toggle (g_mode_override), both in
    // update(). A driver that wants to disable itself must LATCH LOCALLY instead; see
    // BlamDrive's s_offsets_rejected.
    // ONE DELIBERATE EXCEPTION, and it is technical debt rather than a design: `blamaim` is used as
    // both a config key AND the runtime ownership handback between BlamAim and BlamDrive ("if
    // BlamAim fails to install it sets blamaim=0, which flips ownership back here" -- BlamDrive.hpp).
    // Resetting it would restore the file's 1 every ~2 s and turn that self-healing into a
    // retry-and-log loop. Preserved until the two meanings are split into separate values.
    // Costs users nothing: `yield` is a `constexpr false` in release builds, so this is dev-only.
    const int keep_blam_aim = g_cfg.blam_aim;

    g_cfg = Config{};
    g_cfg.blam_aim = keep_blam_aim;

    // The base file is the only REQUIRED one: with it missing there is nothing to override, so we
    // write a default and come back next poll.
    if (!parse_config_file(kConfigFiles[0])) { write_default_config(); return; }

    // Override layers, weakest to strongest: the user's persistent overrides (never shipped, so
    // they survive updates), then the dev/troubleshooting file (ships all-commented; an
    // uncommented key there is a deliberate, temporary experiment and beats the user file on
    // purpose -- and updates overwrite it, so it cannot linger), then the calibration file:
    // the in-game gestures own their keys outright.
    //
    // Walked from kConfigFiles rather than listed again, so this can never fall out of step with
    // the change-detection gate at the top -- see the warning there.
    for (int i = 1; i < kConfigFileCount; ++i) parse_config_file(kConfigFiles[i]);

    // An offset with no version stamp is AMBIGUOUS, and guessing wrong is a constant, invisible yaw
    // error that then gets written back to disk. Raised here, reported by the tick -- this file has
    // no UEVR API dependency and is worth keeping that way.
    g_calib_stamp_ambiguous =
        g_cfg.aim_off_valid && g_cfg.aim_calib_ver < 2 && g_cfg.calib_relative;

    // wpnfix lines that carried the wrong schema stamp were dropped at parse. Say so ONCE, here:
    // load_config only reaches this line when a file actually changed, so this cannot spam, and a
    // per-weapon calibration file that is being ignored in silence is indistinguishable from a
    // feature that does not work -- which is the support report nobody can act on.
    if (g_cfg.wpnfix_dropped > 0) {
        API::get()->log_info(
            "[Halo-CampE-UEVR] WPNFIX: ignored %d per-weapon line(s) -- their file did not stamp "
            "wpnfixver=%d before them. A file from another fork uses a different frame and would "
            "put the weapon in the wrong place; the shipped per-weapon calibration is being used "
            "instead. Delete or re-capture those lines.",
            g_cfg.wpnfix_dropped, kWeaponFixSchema);
    }

    // Only here, past every early return: g_cfg now holds the values that are actually on disk.
    ++g_cfg_load_gen;
}

// Point g_calib_path at the file for the configured hand, and load it.
//
// Called once at startup, after load_config, because `aimhand` lives in halo_vr.cfg and so is not
// known until that has been parsed.
//
// The right hand keeps the original filename, so existing installs are untouched and a user who
// never goes left-handed sees no change at all. The left hand gets `_left`, so the two calibrations
// coexist and switching back and forth never destroys either.
//
// SEEDING: if the left file does not exist, the right-hand values are MIRRORED into it rather than
// left at defaults. Grip yaw and roll, and the lateral (X) offset, flip sign between hands; pitch,
// forward/up offsets and the aim-ray offset do not. That gets a left-handed player close enough to
// judge, instead of starting from a cold zero that reads as "the mod is broken".
int select_calib_for_hand() {
    if (!g_cfg.aim_left_hand) {
        // Right hand: g_calib_path already points at the original file, loaded by load_config.
        return CALIB_HAND_RIGHT;
    }

    char left[MAX_PATH] = {0};
    const char* dot = strrchr(g_calib_path_right, '.');
    if (dot != nullptr) {
        const size_t stem = (size_t)(dot - g_calib_path_right);
        if (stem < MAX_PATH - 8) {
            memcpy(left, g_calib_path_right, stem);
            left[stem] = 0;
            strcat_s(left, MAX_PATH, "_left");
            strcat_s(left, MAX_PATH, dot);
        }
    }
    if (left[0] == 0) return CALIB_HAND_RIGHT;   // unexpected path shape: stay on the right file

    strcpy_s(g_calib_path, MAX_PATH, left);

    if (parse_config_file(g_calib_path)) return CALIB_HAND_LEFT_LOADED;

    // No left-hand calibration yet. g_cfg currently holds the RIGHT-hand values (load_config just
    // applied them), so mirror them in place and persist, giving the left hand a sane start.
    g_cfg.grip_yaw  = -g_cfg.grip_yaw;
    g_cfg.grip_roll = -g_cfg.grip_roll;
    g_cfg.off_x     = -g_cfg.off_x;
    g_cfg.piv_x     = -g_cfg.piv_x;
    g_cfg.aim_off_yaw = -g_cfg.aim_off_yaw;
    write_calib_file();
    return CALIB_HAND_LEFT_SEEDED;
}

// Persist a calibration result. Written whole every time, so the newest calibration always wins.
void write_calib_file() {
    FILE* f = nullptr;
    if (fopen_s(&f, g_calib_path, "wb") != 0 || f == nullptr) return;

    // PERSIST THE BASE, NOT THE ADJUSTED VALUES.
    //
    // When per-weapon offsets are active, g_cfg.grip_* and off_* carry the delta for whatever is
    // currently in hand. Writing those here would fold one weapon's adjustment permanently into
    // the global calibration that every OTHER weapon depends on -- and this function is also
    // called by the AIM calibration (Page Down), which has nothing to do with the mesh fit and
    // would silently bake it in.
    //
    // wpn_base_* is what the config supplied before any delta, republished by WeaponOffset on
    // every capture, so it is the correct thing to round-trip.
    const float w_grip  = g_cfg.wpn_offsets ? g_cfg.wpn_base_grip      : g_cfg.grip_deg;
    const float w_gyaw  = g_cfg.wpn_offsets ? g_cfg.wpn_base_grip_yaw  : g_cfg.grip_yaw;
    const float w_groll = g_cfg.wpn_offsets ? g_cfg.wpn_base_grip_roll : g_cfg.grip_roll;
    const float w_ox    = g_cfg.wpn_offsets ? g_cfg.wpn_base_off_x     : g_cfg.off_x;
    const float w_oy    = g_cfg.wpn_offsets ? g_cfg.wpn_base_off_y     : g_cfg.off_y;
    const float w_oz    = g_cfg.wpn_offsets ? g_cfg.wpn_base_off_z     : g_cfg.off_z;
    fprintf(f,
        "# halo_vr - CALIBRATION RESULT. Written by the pose-match calibration; applied\r\n"
        "# AFTER halo_vr.cfg, so these override the values in that file.\r\n"
        "# Delete this file to fall back to your hand-tuned settings.\r\n"
        "#\r\n"
        "# calibver: 1 (or absent) = yaw values are ABSOLUTE; 2 = measured relative to the view-lock\r\n"
        "# yaw, so the calibration no longer encodes where you injected. See `calibrelative`.\r\n"
        "calibver=%d\r\n"
        "grip=%.3f\r\ngripyaw=%.3f\r\ngriproll=%.3f\r\n"
        "# Same fit for the direct-drive rig (rigmode=3), which composes against its OWN trim and\r\n"
        "# its OWN mount -- BOTH, not just the rotation. Persisting the grip and leaving the mount\r\n"
        "# behind is how a correct fit still failed to survive a restart in mode 3.\r\n"
        "dirgrip=%.3f\r\ndirgripyaw=%.3f\r\ndirgriproll=%.3f\r\n"
        "diroffx=%.3f\r\ndiroffy=%.3f\r\ndiroffz=%.3f\r\n"
        "offx=%.3f\r\noffy=%.3f\r\noffz=%.3f\r\n",
        // The TRACKED version, not `calib_relative ? 2 : 1`. Deriving it from the feature flag
        // certifies values the gesture never actually rebased -- which is how an untouched
        // absolute aimoffyaw got stamped v2 by a mesh calibration.
        g_cfg.calib_ver,
        w_grip, w_gyaw, w_groll,
        g_cfg.rig_dir_grip_deg, g_cfg.rig_dir_grip_yaw, g_cfg.rig_dir_grip_roll,
        g_cfg.rig_dir_off_x, g_cfg.rig_dir_off_y, g_cfg.rig_dir_off_z,
        w_ox, w_oy, w_oz);

    if (g_cfg.aim_off_valid) {
        fprintf(f,
            "# Hand-to-aim mapping from the Page Down calibration. Stored as an OFFSET so it\r\n"
            "# survives level loads and respawns, which reset the absolute reference.\r\n"
            "# aimcalibver is SEPARATE from calibver: this offset is only frame-relative once the\r\n"
            "# aim calibration itself has been re-run, whatever the mesh calibration did.\r\n"
            "aimcalibver=%d\r\naimoffyaw=%.3f\r\naimoffpitch=%.3f\r\n",
            g_cfg.aim_calib_ver, g_cfg.aim_off_yaw, g_cfg.aim_off_pitch);
    }

    if (g_pivot_from_calib) {
        fprintf(f,
            "# Pivot MEASURED from two calibration samples -- pivauto is off so the socket\r\n"
            "# estimate cannot overwrite it.\r\n"
            "pivauto=0\r\npivx=%.3f\r\npivy=%.3f\r\npivz=%.3f\r\n",
            g_cfg.piv_x, g_cfg.piv_y, g_cfg.piv_z);
    } else {
        fprintf(f,
            "# Pivot still ESTIMATED from the '%s' socket. Calibrate a second time at a\r\n"
            "# clearly different wrist angle to measure it instead.\r\n"
            "# pivx=%.3f pivy=%.3f pivz=%.3f\r\n",
            g_cfg.piv_socket, g_cfg.piv_x, g_cfg.piv_y, g_cfg.piv_z);
    }

    if (g_cfg.hand_fix_valid) {
        // THE STAMP GOES FIRST, and it must: the parser scopes handfixver to the file it appears in
        // and resets it per file, so a line above the stamp is a line with no stamp and is dropped.
        // Same rule, same reason, as the wpnfix block in halo_vr_weapons.cfg.
        fprintf(f,
            "# SUPPORT HAND from the in-game menu's 'Calibrate support hand' gesture: the hand\r\n"
            "# freezes, you put your real hand where it is, and the trigger saves the difference.\r\n"
            "# A rigid transform in the SUPPORT controller's OWN frame -- the off hand, i.e. the\r\n"
            "# LEFT hand unless aimhand=left. Applied by the palette arm driver (armdriver=2).\r\n"
            "#\r\n"
            "# handfix=qx,qy,qz,qw,tx,ty,tz -- quaternion then METRES, in BLAM axes (X forward,\r\n"
            "#   Y LEFT, Z up). The rotation right-multiplies the controller pose; the translation\r\n"
            "#   is carried by the untrimmed basis, so the pair is one rigid attachment.\r\n"
            "# handfixver stamps that convention and must stay ABOVE the line it covers.\r\n"
            "# Delete these two lines (or the whole file) to put the hand back on the plain\r\n"
            "# controller pose, which is the built-in default.\r\n"
            "handfixver=%d\r\n"
            "handfix=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
            kHandFixSchema,
            g_cfg.hand_fix_q[0], g_cfg.hand_fix_q[1], g_cfg.hand_fix_q[2], g_cfg.hand_fix_q[3],
            g_cfg.hand_fix_t[0], g_cfg.hand_fix_t[1], g_cfg.hand_fix_t[2]);
    }

    if (g_cfg.scope_calib_valid) {
        fprintf(f,
            "# SCOPE PLACEMENT from the Delete-key calibration: hold Delete, the pane freezes in\r\n"
            "# the world, move your weapon hand until it sits where you want it, release.\r\n"
            "# These are the pane's offset and facing RELATIVE to the weapon rig, so they are\r\n"
            "# mount=1 values by construction. Delete this block (or the whole file) to go back\r\n"
            "# to the built-in defaults.\r\n"
            "scopemount=1\r\n"
            "scopedist=%.2f\r\nscoperight=%.2f\r\nscopeup=%.2f\r\n"
            "scoperotp=%.2f\r\nscoperoty=%.2f\r\nscoperotr=%.2f\r\n",
            g_cfg.scope_dist, g_cfg.scope_right, g_cfg.scope_up,
            g_cfg.scope_rot_p, g_cfg.scope_rot_y, g_cfg.scope_rot_r);
    }
    fclose(f);
}

} // namespace halo
