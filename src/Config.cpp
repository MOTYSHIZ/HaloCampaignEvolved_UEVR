#include "Config.hpp"
#include "Math.hpp"

#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace halo {
Config g_cfg{};


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
void parse_config_key_2(const char* key, const char* val, double v) {
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
        else if (_stricmp(key, "aimwidgettint")  == 0) g_cfg.aim_widget_tint  = clampf((float)v, 0.0f, 64.0f);
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
        else if (_stricmp(key, "rigfast")       == 0) g_cfg.rig_fast        = (v != 0.0);
        else if (_stricmp(key, "stickmode")     == 0) g_cfg.stick_mode      = (v != 0.0);
        else if (_stricmp(key, "stickforce")    == 0) g_cfg.stick_force     = (int)v;
        else if (_stricmp(key, "stickon")       == 0) g_cfg.stick_on_s      = clampf((float)v, 0.1f, 30.0f);
        else if (_stricmp(key, "stickoff")      == 0) g_cfg.stick_off_s     = clampf((float)v, 0.03f, 30.0f);
        else if (_stricmp(key, "brake")         == 0) g_cfg.brake_enabled   = (v != 0.0);
        else if (_stricmp(key, "brakemode")     == 0) g_cfg.brake_mode      = (int)v;
        else if (_stricmp(key, "brakemask")     == 0) g_cfg.brake_mask      = (int)strtol(val, nullptr, 0);
        else if (_stricmp(key, "brakekey")      == 0) g_cfg.brake_key       = (int)strtol(val, nullptr, 0);
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
        else if (_stricmp(key, "ffsmooth")== 0) g_cfg.ff_smooth   = clampf((float)v, 0.0f, 1.0f);
        else if (_stricmp(key, "dsmooth") == 0) g_cfg.d_smooth    = clampf((float)v, 0.02f, 1.0f);
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
        // base 0 so the mask can be written as 0x0020 (readable) or 32 (not).
        else if (_stricmp(key, "calibbtn")  == 0) g_cfg.calib_btn   = (int)strtol(val, nullptr, 0);
        else if (_stricmp(key, "calibkey")  == 0) g_cfg.calib_key   = (int)strtol(val, nullptr, 0);
        else if (_stricmp(key, "aimcalibkey") == 0) g_cfg.aim_calib_key = (int)strtol(val, nullptr, 0);
    else if (_stricmp(key, "killkey")    == 0) g_cfg.kill_key    = (int)strtol(val, nullptr, 0);
        else if (_stricmp(key, "aimoffyaw")   == 0) { g_cfg.aim_off_yaw   = (float)v; g_cfg.aim_off_valid = true; }
        else if (_stricmp(key, "aimoffpitch") == 0) { g_cfg.aim_off_pitch = (float)v; g_cfg.aim_off_valid = true; }
        else if (_stricmp(key, "rigmode")   == 0) g_cfg.rig_mode    = (int)v;
        else if (_stricmp(key, "riganchor") == 0) g_cfg.rig_body_anchor = (v != 0.0);
        else if (_stricmp(key, "rigviewyaw") == 0) g_cfg.rig_view_yaw  = clampf((float)v, -1.0f, 1.0f);
        else if (_stricmp(key, "rigneutral") == 0) g_cfg.rig_neutral   = (v != 0.0);
        else if (_stricmp(key, "rigturn")   == 0) g_cfg.rig_turn     = clampf((float)v, -1.0f, 1.0f);
        else if (_stricmp(key, "aimturn")   == 0) g_cfg.aim_turn     = clampf((float)v, -1.0f, 1.0f);
        else if (_stricmp(key, "moverot")   == 0) g_cfg.move_rot     = clampf((float)v, -1.0f, 1.0f);
        else if (_stricmp(key, "movesrc")   == 0) g_cfg.move_src     = (int)v;
        else if (_stricmp(key, "movelive")  == 0) g_cfg.move_live    = (v != 0.0);
        else if (_stricmp(key, "movesmooth") == 0) g_cfg.move_smooth = clampf((float)v, 0.0f, 1.0f);
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

void load_config() {
    if (!parse_config_file(g_cfg_path)) { write_default_config(); return; }
    parse_config_file(g_calib_path);
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
        "grip=%.3f\r\ngripyaw=%.3f\r\ngriproll=%.3f\r\n"
        "offx=%.3f\r\noffy=%.3f\r\noffz=%.3f\r\n",
        g_cfg.grip_deg, g_cfg.grip_yaw, g_cfg.grip_roll,
        g_cfg.off_x, g_cfg.off_y, g_cfg.off_z);

    if (g_cfg.aim_off_valid) {
        fprintf(f,
            "# Hand-to-aim mapping from the Page Down calibration. Stored as an OFFSET so it\r\n"
            "# survives level loads and respawns, which reset the absolute reference.\r\n"
            "aimoffyaw=%.3f\r\naimoffpitch=%.3f\r\n",
            g_cfg.aim_off_yaw, g_cfg.aim_off_pitch);
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
