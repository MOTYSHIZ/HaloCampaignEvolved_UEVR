#include "Config.hpp"
#include "Math.hpp"

#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace halo {
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


// ---------------------------------------------------------------- live config
// KILL SWITCH FIRST, tuning second. There is no plugin on_message callback in this API version,
// so without the file there would be no off switch at all: if aim misbehaved with a headset on,
// the only recourse would be killing the game. That is not an acceptable state to hand to a
// person in VR, so the file is re-read every ~2 s and `enabled=0` neutralises the stick on the
// next tick.
//
// It doubles as live tuning: floor/full/max/dead/xdist can be changed without a rebuild.
char g_cfg_path[MAX_PATH] = {0};
uint32_t g_cfg_check_tick = 0;

void write_default_config() {
    FILE* f = nullptr;
    if (fopen_s(&f, g_cfg_path, "wb") != 0 || f == nullptr) return;
    fprintf(f,
        "# halo_vr - live config. Re-read about every 2 seconds; no restart needed.\r\n"
        "#\r\n"
        "# enabled=0 is the KILL SWITCH: the stick goes neutral on the next tick and the game\r\n"
        "# plays exactly as if the plugin were not loaded. Set this if aim ever misbehaves.\r\n"
        "enabled=1\r\n"
        "\r\n"
        "# pitch=0 drives yaw only (aim up/down stays on the HMD / game default).\r\n"
        "pitch=1\r\n"
        "\r\n"
        "# floor : deflection the game starts responding to. Measured deadzone here is ~0.24,\r\n"
        "#         so below ~0.25 the game ignores the stick entirely.\r\n"
        "floor=%.3f\r\n"
        "# full  : degrees of error that saturate the stick. Smaller = snappier.\r\n"
        "full=%.2f\r\n"
        "# max   : ceiling on deflection. The response curve is steeply exponential, so this\r\n"
        "#         mostly controls how fast LARGE corrections close.\r\n"
        "max=%.2f\r\n"
        "# dead  : angular deadband in degrees - stop instead of hunting.\r\n"
        "dead=%.2f\r\n"
        "# xdist : sightline length in metres. Aim is the ray from the origin below THROUGH the\r\n"
        "#         point the gun points at, so gun TRANSLATION affects aim, not just rotation.\r\n"
        "xdist=%.1f\r\n"
        "\r\n"
        "# viewlock : 1 = aiming does NOT turn your headset view (cancels the induced yaw in VR\r\n"
        "#            space, the way UEVR's own controller-aim does). 0 = old behaviour, where\r\n"
        "#            aiming drags your head around. Turn the body with snap/smooth turn.\r\n"
        "viewlock=%d\r\n"
        "# locksign : flip to 1 if aiming turns the view the WRONG way instead of holding it still.\r\n"
        "locksign=%d\r\n"
        "\r\n"
        "# aimorigin : 1 = standing origin (UEVR's choice) - aim depends only on the gun.\r\n"
        "#             0 = HMD position - moving your HEAD also changes aim.\r\n"
        "aimorigin=%d\r\n"
        "\r\n"
        "# TURNING. With the view locked, waving the controller no longer turns you, so this is how\r\n"
        "# you turn. Input is your own RIGHT STICK (sampled before the aim value overwrites it).\r\n"
        "#   turnmode 1 = snap (discrete, least nausea)   2 = smooth   0 = off\r\n"
        "turnmode=%d\r\n"
        "snapdeg=%.0f\r\n"
        "smoothdps=%.0f\r\n"
        "turndz=%.2f\r\n"
        "\r\n"
        "# WEAPON RIG - drives the first-person rig from the controller so the gun follows your\r\n"
        "# hand. Ported from the FPArmTransformDriver Lua mod: no F8, no LuaVR, no arming step.\r\n"
        "rig=%d\r\n"
        "# rigloc : drive TRANSLATION as well as rotation.\r\n"
        "rigloc=%d\r\n"
        "# grip   : grip-to-aim correction in degrees.\r\n"
        "grip=%.0f\r\n"
        "# rigscale : cm of rig movement per metre of hand movement.  rigclamp : cm limit per axis.\r\n"
        "rigscale=%.0f\r\n"
        "rigclamp=%.0f\r\n",
        g_cfg.floor, g_cfg.full_deg, g_cfg.max_out, g_cfg.dead_deg, g_cfg.xdist_m,
        (int)g_cfg.view_lock, (int)(g_cfg.lock_sign < 0.0f ? -1 : 1), g_cfg.aim_origin,
        g_cfg.turn_mode, g_cfg.snap_deg, g_cfg.smooth_dps, g_cfg.turn_dz,
        (int)g_cfg.rig_enabled, (int)g_cfg.rig_loc, g_cfg.grip_deg,
        g_cfg.rig_scale, g_cfg.rig_clamp);
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
    if (_stricmp(key, "vraccel")       == 0) { g_cfg.vr_accel       = (int)v; return true; }
    if (_stricmp(key, "aimdeadbeat")   == 0) { g_cfg.aim_deadbeat   = (float)v; return true; }
    if (_stricmp(key, "aimstat")       == 0) { g_cfg.aim_stat       = (int)v; return true; }
    if (_stricmp(key, "aimtargetsmoothms") == 0) { g_cfg.aim_target_smooth_ms = (float)v; return true; }
    return false;
}

void parse_config_key_2(const char* key, const char* val, double v) {
        if (parse_blam_key(key, val, v)) return;
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
        // base 0 so masks can be written readably as 0x0040
        else if (_stricmp(key, "mapdpadshift")  == 0) g_cfg.map_dpad_shift  = (v != 0.0);
        else if (_stricmp(key, "mapdpaddz")     == 0) g_cfg.map_dpad_dz     = clampf((float)v, 0.2f, 0.95f);
        else if (_stricmp(key, "maprstickdown") == 0) g_cfg.map_rstick_down = (int)strtol(val, nullptr, 0);
        else if (_stricmp(key, "maprstickdz")   == 0) g_cfg.map_rstick_dz   = clampf((float)v, 0.2f, 0.95f);
        else if (_stricmp(key, "mapfrom")       == 0) g_cfg.map_from        = (int)strtol(val, nullptr, 0);
        else if (_stricmp(key, "mapbtnlog")   == 0) g_cfg.map_btn_log    = (v != 0.0);
        else if (_stricmp(key, "mapto")         == 0) g_cfg.map_to          = (int)strtol(val, nullptr, 0);
        else if (_stricmp(key, "perflog")       == 0) g_cfg.perf_log        = (v != 0.0);
        else if (_stricmp(key, "aimtrace")      == 0) g_cfg.aim_trace       = (v != 0.0);
        else if (_stricmp(key, "memscan")       == 0) g_cfg.mem_scan        = (v != 0.0);
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
        else if (_stricmp(key, "stickon")       == 0) g_cfg.stick_on_s      = clampf((float)v, 0.1f, 30.0f);
        else if (_stricmp(key, "stickoff")      == 0) g_cfg.stick_off_s     = clampf((float)v, 0.03f, 30.0f);
        else if (_stricmp(key, "brake")         == 0) g_cfg.brake_enabled   = (v != 0.0);
        else if (_stricmp(key, "brakemode")     == 0) g_cfg.brake_mode      = (int)v;
        else if (_stricmp(key, "brakemask")     == 0) g_cfg.brake_mask      = (int)strtol(val, nullptr, 0);
        else if (_stricmp(key, "brakekey")      == 0) g_cfg.brake_key       = (int)strtol(val, nullptr, 0);
        else if (_stricmp(key, "cutscene2d")    == 0) g_cfg.cutscene_2d     = (int)v;
        else if (_stricmp(key, "cuthint")       == 0) g_cfg.cut_hint        = (v != 0.0);
        else if (_stricmp(key, "cuthintdist")   == 0) g_cfg.cut_hint_dist   = clampf((float)v, 0.5f, 5.0f);
        else if (_stricmp(key, "cuthintdrop")   == 0) g_cfg.cut_hint_drop   = clampf((float)v, -2.0f, 2.0f);
        else if (_stricmp(key, "cuthintw")      == 0) g_cfg.cut_hint_w      = clampf((float)v, 0.3f, 3.0f);
}

bool parse_config_file(const char* path) {
    FILE* f = nullptr;
    if (fopen_s(&f, path, "rb") != 0 || f == nullptr) return false;

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
        else if (_stricmp(key, "xdist")     == 0) g_cfg.xdist_m    = clampf((float)v, 1.0f, 100.0f);
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
        else if (_stricmp(key, "hmdleashlat")  == 0) g_cfg.hmd_leash_lat  = clampf((float)v, 0.0f, 5.0f);
        else if (_stricmp(key, "hmdleashvert") == 0) g_cfg.hmd_leash_vert = clampf((float)v, 0.0f, 5.0f);
        else if (_stricmp(key, "modekey")   == 0) g_cfg.mode_key    = (int)strtol(val, nullptr, 0);
        else if (_stricmp(key, "dbgmark")   == 0) g_cfg.dbg_mark    = (int)v;
    else if (_stricmp(key, "killkey")    == 0) g_cfg.kill_key    = (int)strtol(val, nullptr, 0);
        else if (_stricmp(key, "aimoffyaw")   == 0) { g_cfg.aim_off_yaw   = (float)v; g_cfg.aim_off_valid = true; }
        else if (_stricmp(key, "aimoffpitch") == 0) { g_cfg.aim_off_pitch = (float)v; g_cfg.aim_off_valid = true; }
        else if (_stricmp(key, "calibrelative") == 0) g_cfg.calib_relative = (v != 0.0);
        else if (_stricmp(key, "calibver")      == 0) g_cfg.calib_ver      = (int)v;
        else if (_stricmp(key, "aimcalibver")   == 0) g_cfg.aim_calib_ver  = (int)v;
        else if (_stricmp(key, "rigmode")   == 0) g_cfg.rig_mode    = (int)v;
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
// legitimate value rather than an error: deleting halo_vr_calib.cfg is a documented way to drop
// back to the shipped calibration, so its disappearance has to register as a change.
uint64_t cfg_file_stamp(const char* path) {
    if (path == nullptr || path[0] == 0) return 0;
    WIN32_FILE_ATTRIBUTE_DATA fad{};
    if (!GetFileAttributesExA(path, GetFileExInfoStandard, &fad)) return 0;
    return ((uint64_t)fad.ftLastWriteTime.dwHighDateTime << 32) |
            (uint64_t)fad.ftLastWriteTime.dwLowDateTime;
}

void load_config() {
    // ---- DO NOT RE-READ FILES THAT HAVE NOT CHANGED.
    //
    // This runs every ~60 ticks, forever, and unconditionally parsed the config files off disk on
    // the GAME THREAD. Normally that is ~0.45 ms and invisible. Measured under disk contention it
    // reached 143.9 ms and 108.7 ms -- roughly 300x -- and a blocking read every ~1.7 s that
    // occasionally costs 100 ms+ is a periodic hitch you can set your watch by.
    //
    // Stat calls instead. Live editing is unaffected: touching a file changes its write time and
    // the next check parses immediately, so the edit-and-see loop still works, and writes made by
    // the plugin itself (calibration captures) reload exactly as before.
    {
        const uint64_t s_main  = cfg_file_stamp(g_cfg_path);
        const uint64_t s_calib = cfg_file_stamp(g_calib_path);
        static uint64_t p_main = 0, p_calib = 0;
        static bool     primed = false;
        if (primed && s_main == p_main && s_calib == p_calib) return;
        p_main = s_main; p_calib = s_calib;
        primed = true;
    }

    // Reset the schema stamps before parsing so "absent" genuinely means absent. Without this they
    // are sticky across the ~2 s reload: once a file that HAD a stamp is edited to remove it, or a
    // calibration file is deleted so only the unstamped halo_vr.cfg fallbacks remain, the old value
    // would persist in memory and keep certifying data it no longer describes.
    g_cfg.calib_ver     = 1;
    g_cfg.aim_calib_ver = 1;

    if (!parse_config_file(g_cfg_path)) { write_default_config(); return; }
    parse_config_file(g_calib_path);

    // An offset with no version stamp is AMBIGUOUS, and guessing wrong is a constant, invisible yaw
    // error that then gets written back to disk. Raised here, reported by the tick -- this file has
    // no UEVR API dependency and is worth keeping that way.
    g_calib_stamp_ambiguous =
        g_cfg.aim_off_valid && g_cfg.aim_calib_ver < 2 && g_cfg.calib_relative;
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
        g_cfg.grip_deg, g_cfg.grip_yaw, g_cfg.grip_roll,
        g_cfg.rig_dir_grip_deg, g_cfg.rig_dir_grip_yaw, g_cfg.rig_dir_grip_roll,
        g_cfg.rig_dir_off_x, g_cfg.rig_dir_off_y, g_cfg.rig_dir_off_z,
        g_cfg.off_x, g_cfg.off_y, g_cfg.off_z);

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
    fclose(f);
}

} // namespace halo
