#include "features/palettewpn/PaletteKeys.hpp"

#include "Config.hpp"
#include "Math.hpp"   // clampf
#include "features/palettewpn/PaletteCalib.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace halo {

// Moved from Config.cpp's fork parsers, statements verbatim, in their original order.
bool palette_wpn_parse_key(const char* key, const char* val, double v) {
    (void)val; (void)v;
    if (_stricmp(key, "aimreticulefresh") == 0) { g_cfg.aim_reticule_fresh = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "aimdirectwrite") == 0) { g_cfg.aim_direct_write = (v != 0.0) ? 1 : 0; return true; }
    if (_stricmp(key, "camleadall") == 0) { g_cfg.cam_lead_all = (v != 0.0) ? 1 : 0; return true; }
    if (_stricmp(key, "compgain") == 0) { g_cfg.comp_gain = clampf((float)v, -2.0f, 2.0f); return true; }
    if (_stricmp(key, "complatch") == 0) { g_cfg.comp_latch = (v != 0.0) ? 1 : 0; return true; }
    if (_stricmp(key, "complatchms") == 0) { g_cfg.comp_latch_ms = clampf((float)v, 0.0f, 40.0f); return true; }
    if (_stricmp(key, "fpanimkill")     == 0) { g_cfg.fp_anim_kill = (int)v; return true; }
    if (_stricmp(key, "fpmeshlog")      == 0) { g_cfg.fpmesh_log = (int)v; return true; }
    if (_stricmp(key, "fppin")          == 0) { g_cfg.fp_pin = (int)v; return true; }
    if (_stricmp(key, "liftyaw") == 0) {
        int m = (int)v; if (m < 0) m = 0; if (m > 2) m = 2;
        g_cfg.lift_yaw = m; return true;
    }
    if (_stricmp(key, "palbuildgate")   == 0) { g_cfg.pal_build_gate = (int)v; return true; }
    if (_stricmp(key, "palettecamlead") == 0) { g_cfg.palette_cam_lead = clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "palettecamsmooth") == 0) { g_cfg.palette_cam_smooth_ms = clampf((float)v, 0.0f, 2000.0f); return true; }
    if (_stricmp(key, "palettefinal")   == 0) { g_cfg.palette_final = (int)v; return true; }
    if (_stricmp(key, "palettelatch") == 0) { g_cfg.palette_latch = (v != 0.0) ? 1 : 0; return true; }
    if (_stricmp(key, "palettelatchms") == 0) { g_cfg.palette_latch_ms = clampf((float)v, 0.0f, 40.0f); return true; }
    if (_stricmp(key, "palettelocal") == 0) { g_cfg.palette_local = (v != 0.0) ? 1 : 0; return true; }
    if (_stricmp(key, "palettesync") == 0) { g_cfg.palette_sync = (v != 0.0) ? 1 : 0; return true; }
    if (_stricmp(key, "palpubframe")    == 0) { g_cfg.pal_pub_frame = (int)v; return true; }
    if (_stricmp(key, "palrender")      == 0) { g_cfg.pal_render = (int)v; return true; }
    if (_stricmp(key, "palsniff")       == 0) { g_cfg.pal_sniff = (int)v; return true; }
    if (_stricmp(key, "palstep") == 0) { g_cfg.palette_step = clampf((float)v, -2.0f, 2.0f); return true; }
    if (_stricmp(key, "palstepctx") == 0) { g_cfg.palette_step_ctx = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "palstepsrc") == 0) { g_cfg.palette_step_src = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "posefilter")     == 0) { g_cfg.pose_filter = (int)v; return true; }
    if (_stricmp(key, "posefilterbeta") == 0) { g_cfg.pose_filter_beta = clampf((float)v, 0.0f, 500.0f); return true; }
    if (_stricmp(key, "posefilterdcut") == 0) { g_cfg.pose_filter_dcut = clampf((float)v, 0.05f, 30.0f); return true; }
    if (_stricmp(key, "posefiltermin")  == 0) { g_cfg.pose_filter_min = clampf((float)v, 0.05f, 60.0f); return true; }
    if (_stricmp(key, "posefilterrbeta")== 0) { g_cfg.pose_filter_rbeta = clampf((float)v, 0.0f, 500.0f); return true; }
    if (_stricmp(key, "posefreeze")     == 0) { g_cfg.pose_freeze = (int)v; return true; }
    if (_stricmp(key, "poselatch") == 0) { g_cfg.pose_latch = (int)clampf((float)v, 0.0f, 3.0f); return true; }
    if (_stricmp(key, "stomplog")       == 0) { g_cfg.stomp_log = (int)v; return true; }
    if (_stricmp(key, "termlog")        == 0) { g_cfg.term_log = (int)v; return true; }
    if (_stricmp(key, "wpnerrlog")      == 0) { g_cfg.wpn_err_log = (int)v; return true; }
    if (_stricmp(key, "twohandmin")     == 0) { g_cfg.two_hand_min_m     = clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "twohandmax")     == 0) { g_cfg.two_hand_max_m     = clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "twohandmarker")  == 0) { g_cfg.two_hand_marker    = (v != 0.0); return true; }
    if (_stricmp(key, "twohandmarkerscale") == 0) { g_cfg.two_hand_marker_scale = clampf((float)v, 0.01f, 0.5f); return true; }
    if (_stricmp(key, "twohandrad")     == 0) { g_cfg.two_hand_radius_m  = clampf((float)v, 0.01f, 1.0f); return true; }
    if (_stricmp(key, "twohandagreemin")== 0) { g_cfg.two_hand_agree_min = clampf((float)v, -1.0f, 1.0f); return true; }
    if (_stricmp(key, "twohandagreefull")==0) { g_cfg.two_hand_agree_full= clampf((float)v, -1.0f, 1.0f); return true; }
    if (_stricmp(key, "twohandblendms") == 0) { g_cfg.two_hand_blend_ms  = clampf((float)v, 1.0f, 2000.0f); return true; }
    if (_stricmp(key, "twohandhaptic")  == 0) { g_cfg.two_hand_haptic    = (v != 0.0); return true; }
    if (_stricmp(key, "palettescan")    == 0) { g_cfg.palette_scan       = (int)v; return true; }
    if (_stricmp(key, "palettepoke")    == 0) { g_cfg.palette_poke       = (int)v; return true; }
    if (_stricmp(key, "palettepokenode")==0) { g_cfg.palette_poke_node   = (int)v; return true; }
    if (_stricmp(key, "palettepokecount")==0){ g_cfg.palette_poke_count  = (int)clampf((float)v, 1.0f, 76.0f); return true; }
    if (_stricmp(key, "twohandmarkercolor") == 0) { strncpy_s(g_cfg.two_hand_marker_color, val, _TRUNCATE); return true; }
    if (_stricmp(key, "slidezonepriority") == 0) { g_cfg.slide_zone_priority = (v != 0.0); return true; }
    if (_stricmp(key, "slidewatch")     == 0) { g_cfg.slide_watch = (v != 0.0); return true; }
    if (_stricmp(key, "slidelog")       == 0) { g_cfg.slide_log = (v != 0.0); return true; }
    if (_stricmp(key, "palettepokeamt") == 0) { g_cfg.palette_poke_amt   = clampf((float)v, -10.0f, 10.0f); return true; }
    if (_stricmp(key, "palettewatch")   == 0) { g_cfg.palette_watch      = (int)v; return true; }
    if (_stricmp(key, "palettehook")    == 0) { g_cfg.palette_hook       = (int)v; return true; }
    if (_stricmp(key, "palettehooktest")==0) { g_cfg.palette_hook_test   = (int)v; return true; }
    if (_stricmp(key, "palettewpn")     == 0) { g_cfg.palette_weapon      = (v != 0.0); return true; }
    if (_stricmp(key, "palettewpnoffx") == 0) { g_cfg.palette_weapon_off_x = clampf((float)v,-200.0f,200.0f); return true; }
    if (_stricmp(key, "palettewpnoffy") == 0) { g_cfg.palette_weapon_off_y = clampf((float)v,-200.0f,200.0f); return true; }
    if (_stricmp(key, "palettewpnoffz") == 0) { g_cfg.palette_weapon_off_z = clampf((float)v,-200.0f,200.0f); return true; }
    if (_stricmp(key, "palettewpnscale")== 0) { g_cfg.palette_weapon_scale = clampf((float)v, 0.05f, 20.0f); return true; }
    if (_stricmp(key, "palettecalibkey")== 0) { g_cfg.palette_calib_key = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "judderlog")      == 0) { g_cfg.judder_log           = (int)clampf((float)v, 0.0f, 20000.0f); return true; }
    if (_stricmp(key, "palettelerp")    == 0) { g_cfg.palette_lerp         = (v != 0.0); return true; }
    if (_stricmp(key, "palettebank")    == 0) { g_cfg.palette_bank         = (int)clampf((float)v, -1.0f, 3.0f); return true; }
    if (_stricmp(key, "palettecam")     == 0) { g_cfg.palette_cam          = (int)clampf((float)v, 0.0f, 15.0f); return true; }
    if (_stricmp(key, "palettebarrellock") == 0) { g_cfg.palette_barrel_lock = (v != 0.0); return true; }
    if (_stricmp(key, "tremor") == 0) {
        int m = (int)v; if (m < 0) m = 0; if (m > 3) m = 3;
        g_cfg.tremor = m; return true;
    }
    if (_stricmp(key, "tremorhz") == 0) { g_cfg.tremor_hz = clampf((float)v, 0.5f, 20.0f); return true; }
    if (_stricmp(key, "tremorq")  == 0) { g_cfg.tremor_q  = clampf((float)v, 0.2f, 20.0f); return true; }
    if (_stricmp(key, "revclamp") == 0) {
        int m = (int)v; if (m < 0) m = 0; if (m > 3) m = 3;
        g_cfg.rev_clamp = m; return true;
    }
    if (_stricmp(key, "revclampdps") == 0) { g_cfg.rev_clamp_dps = clampf((float)v, 0.0f, 20000.0f); return true; }
    if (_stricmp(key, "meshconst") == 0) {
        int m = (int)v; if (m < 0) m = 0; if (m > 3) m = 3;
        g_cfg.mesh_const = m; return true;
    }
    if (_stricmp(key, "meshconstgate") == 0) {
        float g = (float)v; if (g < 0.0f) g = 0.0f; if (g > 45.0f) g = 45.0f;
        g_cfg.mesh_const_gate = g; return true;
    }
    if (_stricmp(key, "paletterolltrim")   == 0) { g_cfg.palette_roll_trim = clampf((float)v, -180.0f, 180.0f); return true; }
    if (_stricmp(key, "fpscalefix")         == 0) { g_cfg.fp_scale_fix = (v != 0.0); return true; }
    if (_stricmp(key, "pinuevrframe")       == 0) { g_cfg.pin_uevr_frame = (v != 0.0); return true; }
    if (_stricmp(key, "palettewpnlockgain")  == 0) return true;
    if (_stricmp(key, "palettewpnlockpitch") == 0) return true;
    if (_stricmp(key, "palettewpnlockcorr")  == 0) return true;
    if (_stricmp(key, "palettewpnsweep")     == 0) return true;
    if (_stricmp(key, "palettewpnfix")       == 0) return true;
    if (_stricmp(key, "palettewpnfixframe")  == 0) return true;
    if (_stricmp(key, "gripfix") == 0) {
        // 7 comma-separated floats: quaternion x,y,z,w then translation x,y,z in metres,
        // controller frame. Written by the Page Up gesture; see Config::grip_fix.
        const char* s = val; int n = 0;
        while (n < 7 && s != nullptr && *s != 0) {
            g_cfg.grip_fix[n++] = (float)atof(s);
            s = strchr(s, (int)0x2C); if (s != nullptr) ++s;
        }
        g_cfg.grip_fix_valid = (n == 7); return true;
    }
    // aimfix is not parsed here: the author's parse_config_file rung owns it, and that rung runs first.
    if (palette_calib_parse_key(key, val, v)) return true;
    return false;
}

} // namespace halo
