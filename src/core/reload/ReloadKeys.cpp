#include "core/reload/ReloadKeys.hpp"

#include "Config.hpp"
#include "Math.hpp"   // clampf

#include <cstdlib>
#include <cstring>
#include <string>

namespace halo {

bool parse_reloadstate_key(const char* key, double v);   // defined below

// Moved from Config.cpp's fork parsers (parse_melee_key's additions, parse_weaponvr_key, parse_fork_port_key),
// statements verbatim, in their original order.
bool reload_engine_parse_key(const char* key, const char* val, double v) {
    (void)val; (void)v;
    // First, as the fork's parse_config_key_2 ran it: the per-weapon reload state keys.
    if (parse_reloadstate_key(key, v)) return true;
    if (_stricmp(key, "reloadframe")    == 0) { g_cfg.reload_frame = (int)v; return true; }
    if (_stricmp(key, "reloadmagoffw")  == 0) { strncpy_s(g_cfg.reload_mag_off_w, val, _TRUNCATE); return true; }
    // Parsed exactly as the author parses reloadjoin and reloadmagoff, so a value means the same
    // thing in either key: the same clamp on the one, a bare three-float read on the other.
    if (_stricmp(key, "reloadseat")     == 0) { g_cfg.reload_seat_dist = clampf((float)v, 0.05f, 1.0f); return true; }
    if (_stricmp(key, "reloadmagbelt")  == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.reload_mag_belt[0], &g_cfg.reload_mag_belt[1], &g_cfg.reload_mag_belt[2]); return true; }
    if (_stricmp(key, "reloadholdfire") == 0) { g_cfg.reload_hold_fire = (v != 0.0); return true; }
    if (_stricmp(key, "reloadvrlog")    == 0) { g_cfg.reload_vr_log = (v != 0.0); return true; }
    if (_stricmp(key, "reloadresetholds") == 0) { g_cfg.reload_reset_holds = (v != 0.0); return true; }
    if (_stricmp(key, "reloadhidearms") == 0) { g_cfg.reload_hide_arms = (int)clampf((float)v, 0.0f, 3.0f); return true; }
    if (_stricmp(key, "zonehandrel")    == 0) { g_cfg.zone_hand_rel = (int)v; return true; }
    if (_stricmp(key, "maghide")     == 0) { g_cfg.mag_hide = (v != 0.0); return true; }
    if (_stricmp(key, "maghidename") == 0) { strncpy_s(g_cfg.mag_hide_name, val, _TRUNCATE); return true; }
    if (_stricmp(key, "animvarset")  == 0) { strncpy_s(g_cfg.anim_var_set, val, _TRUNCATE); return true; }
    if (_stricmp(key, "animseqset")  == 0) { strncpy_s(g_cfg.anim_seq_set, val, _TRUNCATE); return true; }
    if (_stricmp(key, "magdump")     == 0) { g_cfg.mag_dump = (int)v; return true; }
    if (_stricmp(key, "animdump")    == 0) { g_cfg.anim_dump = (v != 0.0); return true; }
    if (_stricmp(key, "animvars")    == 0) { g_cfg.anim_vars = (v != 0.0); return true; }
    if (_stricmp(key, "animobjs")    == 0) { g_cfg.anim_objs = (v != 0.0); return true; }
    if (_stricmp(key, "reloadaudiodump") == 0) { g_cfg.reload_audio_dump = (v != 0.0); return true; }
    if (_stricmp(key, "reloadwwisedump") == 0) { g_cfg.reload_wwise_dump = (v != 0.0); return true; }
    if (_stricmp(key, "reloadakmute")    == 0) { g_cfg.reload_ak_mute = (int)clampf((float)v, 0.0f, 4.0f); return true; }
    if (_stricmp(key, "reloadstepsound") == 0) {
        std::string cur = g_cfg.reload_step_override;
        if (!cur.empty() && cur.back() != ';') cur += ';';
        cur += val;
        strncpy_s(g_cfg.reload_step_override, cur.c_str(), _TRUNCATE);
        return true;
    }
    if (_stricmp(key, "reloadpressat")   == 0) { g_cfg.reload_press_at = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "akpostrva")      == 0) { g_cfg.ak_post_rva = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "akmutenames")    == 0) { strncpy_s(g_cfg.ak_mute_names, val, _TRUNCATE); return true; }
    if (_stricmp(key, "akrtpc")         == 0) { strncpy_s(g_cfg.ak_rtpc, val, _TRUNCATE); return true; }
    if (_stricmp(key, "akrtpcvalue")    == 0) { g_cfg.ak_rtpc_value = (float)v; return true; }
    if (_stricmp(key, "akrtpcglobal")   == 0) { g_cfg.ak_rtpc_global = (v != 0.0); return true; }
    if (_stricmp(key, "akrtpcrestore")  == 0) { g_cfg.ak_rtpc_restore = (float)v; return true; }
    if (_stricmp(key, "akmimic")        == 0) { g_cfg.ak_mimic = (int)clampf((float)v, 0.0f, 7.0f); return true; }
    if (_stricmp(key, "akmimic4event")  == 0) { strncpy_s(g_cfg.ak_mimic4_event, val, _TRUNCATE); return true; }
    if (_stricmp(key, "akfnsetlisteners") == 0) { g_cfg.ak_fn_setlisteners = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "akfnsetswitch")  == 0) { g_cfg.ak_fn_setswitch = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "akfnsetrtpc")    == 0) { g_cfg.ak_fn_setrtpc = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "akfnsetposition") == 0) { g_cfg.ak_fn_setposition = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "akfnregister")   == 0) { g_cfg.ak_fn_register = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "akfnunregister") == 0) { g_cfg.ak_fn_unregister = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "akstack")        == 0) { g_cfg.ak_stack = (v != 0.0); return true; }
    if (_stricmp(key, "aklog")          == 0) { g_cfg.ak_log = (v != 0.0); return true; }
    if (_stricmp(key, "akvtdump")       == 0) { g_cfg.ak_vt_dump = (v != 0.0); return true; }
    if (_stricmp(key, "akvtglobal")     == 0) { g_cfg.ak_vt_global = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "akvtcount")      == 0) { g_cfg.ak_vt_count = (int)clampf((float)v, 1.0f, 400.0f); return true; }
    if (_stricmp(key, "reloadmutevariant") == 0) { g_cfg.reload_mute_variant = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "reloadmutems")    == 0) { g_cfg.reload_mute_ms = (int)clampf((float)v, 0.0f, 10000.0f); return true; }
    if (_stricmp(key, "reloadstepvia")   == 0) { g_cfg.reload_step_via = (int)clampf((float)v, 0.0f, 5.0f); return true; }
    if (_stricmp(key, "reloadstepvariant") == 0) { g_cfg.reload_step_variant = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "reloadlift")     == 0) { g_cfg.reload_lift = clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "reloadwellfwd")  == 0) { g_cfg.reload_well_fwd = clampf((float)v, 0.0f, 0.6f); return true; }
    if (_stricmp(key, "reloadwellmarker") == 0) { g_cfg.reload_well_marker = (v != 0.0); return true; }
    if (_stricmp(key, "reloadslidems")  == 0) { g_cfg.reload_slide_ms = (int)clampf((float)v, 0.0f, 2000.0f); return true; }
    if (_stricmp(key, "reloadmaskms")   == 0) { g_cfg.reload_mask_ms  = (int)clampf((float)v, 0.0f, 6000.0f); return true; }
    if (_stricmp(key, "reloadanimrate") == 0) { g_cfg.reload_anim_rate = clampf((float)v, 0.0f, 200.0f); return true; }
    if (_stricmp(key, "reloadanimms")   == 0) { g_cfg.reload_anim_ms   = (int)clampf((float)v, 0.0f, 6000.0f); return true; }
    if (_stricmp(key, "reloadholdstate") == 0) { g_cfg.reload_hold_state = (v != 0.0); return true; }
    if (_stricmp(key, "reloadpauseanim") == 0) { g_cfg.reload_pause_anim = (v != 0.0); return true; }
    if (_stricmp(key, "reloadwellmarkerscale") == 0) { g_cfg.reload_well_marker_scale = clampf((float)v, 0.01f, 0.5f); return true; }
    if (_stricmp(key, "reloadinsertmode") == 0) { g_cfg.reload_insert_mode = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "reloadinsert")   == 0) { g_cfg.reload_insert = clampf((float)v, 0.01f, 0.5f); return true; }
    if (_stricmp(key, "reloadinsertdone") == 0) { g_cfg.reload_insert_done = clampf((float)v, 0.0f, 0.1f); return true; }
    if (_stricmp(key, "reloadinsertsign") == 0) { g_cfg.reload_insert_sign = (v < 0.0) ? -1.0f : 1.0f; return true; }
    if (_stricmp(key, "reloadhandoff")  == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.reload_hand_off[0], &g_cfg.reload_hand_off[1], &g_cfg.reload_hand_off[2]); return true; }
    if (_stricmp(key, "reloadhandrot")  == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.reload_hand_rot[0], &g_cfg.reload_hand_rot[1], &g_cfg.reload_hand_rot[2]); return true; }
    if (_stricmp(key, "slideweapons")   == 0) { strncpy_s(g_cfg.slide_weapons, val, _TRUNCATE); return true; }
    if (_stricmp(key, "slidechamberweapons") == 0) { strncpy_s(g_cfg.slide_chamber_weapons, val, _TRUNCATE); return true; }
    if (_stricmp(key, "reloadskipweapons")   == 0) { strncpy_s(g_cfg.reload_skip_weapons, val, _TRUNCATE); return true; }
    if (_stricmp(key, "slidetravel")    == 0) { g_cfg.slide_travel = clampf((float)v, 0.001f, 0.2f); return true; }
    if (_stricmp(key, "slideradius")    == 0) { g_cfg.slide_radius = clampf((float)v, 0.02f, 0.5f); return true; }
    if (_stricmp(key, "slidesign")      == 0) { g_cfg.slide_sign = (v < 0.0) ? -1.0f : 1.0f; return true; }
    if (_stricmp(key, "slideoff")       == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.slide_off[0], &g_cfg.slide_off[1], &g_cfg.slide_off[2]); return true; }
    if (_stricmp(key, "slidezone")      == 0) { g_cfg.slide_zone = (int)clampf((float)v, 0.0f, 3.0f); return true; }
    if (_stricmp(key, "slidemarkersize")    == 0) { g_cfg.slide_marker_size = clampf((float)v, 0.01f, 0.5f); return true; }
    if (_stricmp(key, "slidemarkercolor")   == 0) { strncpy_s(g_cfg.slide_marker_color, val, _TRUNCATE); return true; }
    if (_stricmp(key, "wellmarkercolor")    == 0) { strncpy_s(g_cfg.well_marker_color, val, _TRUNCATE); return true; }
    if (_stricmp(key, "slidezoneback")  == 0) { g_cfg.slide_zone_back = clampf((float)v, -0.2f, 0.2f); return true; }
    if (_stricmp(key, "slidelockreload") == 0) { g_cfg.slide_lock_reload = (v != 0.0); return true; }
    if (_stricmp(key, "slidechamber")   == 0) { g_cfg.slide_chamber = (v != 0.0); return true; }
    if (_stricmp(key, "reloadpressms")  == 0) { g_cfg.reload_press_ms = (int)clampf((float)v, 30.0f, 2000.0f); return true; }
    if (_stricmp(key, "reloadpressmscoop") == 0) { g_cfg.reload_press_ms_coop = (int)clampf((float)v, 30.0f, 2000.0f); return true; }
    if (_stricmp(key, "coopauto")       == 0) { g_cfg.coop_auto = (v != 0.0); return true; }
    if (_stricmp(key, "coopstopat")     == 0) { g_cfg.coop_stop_at = (int)clampf((float)v, 0.0f, 10.0f); return true; }
    if (_stricmp(key, "coophide")       == 0) { g_cfg.coop_hide = (v != 0.0); return true; }
    if (_stricmp(key, "hidesolo")       == 0) { g_cfg.hide_solo = (v != 0.0); return true; }
    if (_stricmp(key, "coopmaskms")     == 0) { g_cfg.coop_mask_ms = (int)clampf((float)v, 0.0f, 20000.0f); return true; }
    if (_stricmp(key, "reloadanimmscoop") == 0) { g_cfg.reload_anim_ms_coop = (int)clampf((float)v, 0.0f, 8000.0f); return true; }
    if (_stricmp(key, "slidephantom")   == 0) { g_cfg.slide_phantom = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "slideundoreload") == 0) { g_cfg.slide_undo_reload = (v != 0.0); return true; }
    if (_stricmp(key, "reserveoff")     == 0) { g_cfg.reserve_off = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "slidecopy")      == 0) { g_cfg.slide_copy = (v != 0.0); return true; }
    if (_stricmp(key, "slidecopybone")  == 0) { strncpy_s(g_cfg.slide_copy_bone, val, _TRUNCATE); return true; }
    if (_stricmp(key, "slidecopyaxis")  == 0) { g_cfg.slide_copy_axis = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "slidecopysign")  == 0) { g_cfg.slide_copy_sign = (v < 0.0) ? -1.0f : 1.0f; return true; }
    if (_stricmp(key, "slidecopytest")  == 0) { g_cfg.slide_copy_test = clampf((float)v, -50.0f, 50.0f); return true; }
    if (_stricmp(key, "slidecopyhide")  == 0) { g_cfg.slide_copy_hide = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "slidecopyfp")    == 0) { g_cfg.slide_copy_fp = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "slidecopyclass") == 0) { g_cfg.slide_copy_class = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "slidecopynonanite") == 0) { g_cfg.slide_copy_no_nanite = (v != 0.0); return true; }
    if (_stricmp(key, "slidecopyfollower") == 0) { g_cfg.slide_copy_follower = (v != 0.0); return true; }
    if (_stricmp(key, "slidecopyleader") == 0) { g_cfg.slide_copy_leader = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "slidecopymode")  == 0) { g_cfg.slide_copy_mode = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "slidecopyseq")   == 0) { strncpy_s(g_cfg.slide_copy_seq, val, _TRUNCATE); return true; }
    if (_stricmp(key, "slidecopyrel")   == 0) { g_cfg.slide_copy_rel = clampf((float)v, 0.01f, 3.0f); return true; }
    if (_stricmp(key, "slidecopysweep") == 0) { g_cfg.slide_copy_sweep = (v != 0.0); return true; }
    if (_stricmp(key, "slidecopyalign") == 0) { g_cfg.slide_copy_align = (v != 0.0); return true; }
    if (_stricmp(key, "slidepart")      == 0) { g_cfg.slide_part = (v != 0.0); return true; }
    if (_stricmp(key, "slidepartbone")  == 0) { strncpy_s(g_cfg.slide_part_bone, val, _TRUNCATE); return true; }
    if (_stricmp(key, "slidepartaxis")  == 0) { g_cfg.slide_part_axis = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "slidepartsign")  == 0) { g_cfg.slide_part_sign = (v < 0.0) ? -1.0f : 1.0f; return true; }
    if (_stricmp(key, "slideparttest")  == 0) { g_cfg.slide_part_test = clampf((float)v, -50.0f, 50.0f); return true; }
    if (_stricmp(key, "slideparthide")  == 0) { g_cfg.slide_part_hide = (int)clampf((float)v, 0.0f, 7.0f); return true; }
    if (_stricmp(key, "slidefarcm")     == 0) { g_cfg.slide_far_cm = clampf((float)v, -100000.0f, 100000.0f); return true; }
    if (_stricmp(key, "slidefararray")  == 0) { g_cfg.slide_far_array = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "slidemorph")     == 0) { strncpy_s(g_cfg.slide_morph, val, _TRUNCATE); return true; }
    if (_stricmp(key, "slidehidemat")   == 0) { strncpy_s(g_cfg.slide_hide_mat, val, _TRUNCATE); return true; }
    if (_stricmp(key, "slidehidematslot") == 0) { g_cfg.slide_hide_mat_slot = (int)clampf((float)v, -1.0f, 31.0f); return true; }
    if (_stricmp(key, "slidehidepbo")   == 0) { g_cfg.slide_hide_pbo = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "slidepartbind")  == 0) { g_cfg.slide_part_bind = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "slidepartpivot") == 0) { g_cfg.slide_part_pivot = (int)clampf((float)v, -1.0f, 1.0f); return true; }
    if (_stricmp(key, "slidepartmat")   == 0) { g_cfg.slide_part_mat = (v != 0.0); return true; }
    if (_stricmp(key, "slidepartorphans") == 0) { g_cfg.slide_part_orphans = (v != 0.0); return true; }
    if (_stricmp(key, "slidepartmaghide") == 0) { g_cfg.slide_part_maghide = (v != 0.0); return true; }
    if (_stricmp(key, "slidepartmagdrop") == 0) { g_cfg.slide_part_magdrop = (v != 0.0); return true; }
    if (_stricmp(key, "slidepartdropmode") == 0) { g_cfg.slide_part_dropmode = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "slidepartkids")  == 0) { g_cfg.slide_part_kids = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "slidepartframe") == 0) { g_cfg.slide_part_frame = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "slidepartui")    == 0) { g_cfg.slide_part_ui = (v != 0.0); return true; }
    if (_stricmp(key, "slidepartshadow") == 0) { g_cfg.slide_part_shadow = (v != 0.0); return true; }
    if (_stricmp(key, "slidebones")     == 0) {
        std::string cur = g_cfg.slide_bones_override;
        if (!cur.empty() && cur.back() != ',') cur += ',';
        cur += val;
        strncpy_s(g_cfg.slide_bones_override, cur.c_str(), _TRUNCATE);
        return true;
    }
    if (_stricmp(key, "slidepartrotaxis") == 0) { g_cfg.slide_part_rot_axis = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "slidepartrotdeg") == 0) { g_cfg.slide_part_rot_deg = clampf((float)v, -90.0f, 90.0f); return true; }
    if (_stricmp(key, "slidepartopendeg") == 0) { g_cfg.slide_part_open_deg = clampf((float)v, -180.0f, 180.0f); return true; }
    if (_stricmp(key, "slidealways")    == 0) { strncpy_s(g_cfg.slide_always_weapons, val, _TRUNCATE); return true; }
    if (_stricmp(key, "slidehidesection") == 0) { g_cfg.slide_hide_section = (int)clampf((float)v, -1.0f, 31.0f); return true; }
    if (_stricmp(key, "slidecopyplay")  == 0) { g_cfg.slide_copy_play = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "slidecopyroot")  == 0) { strncpy_s(g_cfg.slide_copy_root, val, _TRUNCATE); return true; }
    if (_stricmp(key, "roundsoff")      == 0) { g_cfg.rounds_off = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "slidemontage")   == 0) { g_cfg.slide_montage = (v != 0.0); return true; }
    if (_stricmp(key, "slideseq")       == 0) { strncpy_s(g_cfg.slide_seq, val, _TRUNCATE); return true; }
    if (_stricmp(key, "slideslot")      == 0) { strncpy_s(g_cfg.slide_slot, val, _TRUNCATE); return true; }
    if (_stricmp(key, "slideseqfwd")    == 0) { g_cfg.slide_seq_fwd = clampf((float)v, 0.0f, 30.0f); return true; }
    if (_stricmp(key, "slideseqback")   == 0) { g_cfg.slide_seq_back = clampf((float)v, 0.0f, 30.0f); return true; }
    if (_stricmp(key, "slideseqsweep")  == 0) { g_cfg.slide_seq_sweep = clampf((float)v, 0.0f, 60.0f); return true; }
    if (_stricmp(key, "slidefire")      == 0) { g_cfg.slide_fire = (v != 0.0); return true; }
    if (_stricmp(key, "slidefirestate") == 0) { g_cfg.slide_fire_state = (int)clampf((float)v, 0.0f, 255.0f); return true; }
    if (_stricmp(key, "slidefireback")  == 0) { g_cfg.slide_fire_back = clampf((float)v, 0.0f, 5.0f); return true; }
    if (_stricmp(key, "slidefireentry") == 0) { g_cfg.slide_fire_entry = clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "slidefirefwd")   == 0) { g_cfg.slide_fire_fwd = clampf((float)v, 0.0f, 5.0f); return true; }
    if (_stricmp(key, "slidemarker")    == 0) { g_cfg.slide_marker = (v != 0.0); return true; }
    if (_stricmp(key, "wpnammodump")    == 0) { g_cfg.wpn_ammo_dump = (v != 0.0); return true; }
    if (_stricmp(key, "ammoseq")        == 0) { strncpy_s(g_cfg.ammo_seq, val, _TRUNCATE); return true; }
    if (_stricmp(key, "ammoscrub")      == 0) { g_cfg.ammo_scrub = clampf((float)v, 0.0f, 120.0f); return true; }
    if (_stricmp(key, "magdrop")        == 0) { g_cfg.mag_drop = (v != 0.0); return true; }
    if (_stricmp(key, "magdropms")      == 0) { g_cfg.mag_drop_ms = (int)clampf((float)v, 500.0f, 30000.0f); return true; }
    return false;
}

// Per-weapon reload state (Gesture.cpp). Hoisted for the same C1061 reason as its siblings.
bool parse_reloadstate_key(const char* key, double v) {
    if (_stricmp(key, "reloadstate")       == 0) { g_cfg.reload_state_id      = (int)clampf((float)v, 0.0f, 3.0f); return true; }
    if (_stricmp(key, "reloadstatesave")   == 0) { g_cfg.reload_state_save    = (int)clampf((float)v, 1.0f, 2.0f); return true; }
    if (_stricmp(key, "reloadstatehide")   == 0) { g_cfg.reload_state_hide    = (int)clampf((float)v, 0.0f, 3.0f); return true; }
    if (_stricmp(key, "reloadstatewaitms") == 0) { g_cfg.reload_state_wait_ms = (int)clampf((float)v, 0.0f, 10000.0f); return true; }
    if (_stricmp(key, "reloadstatedrop")   == 0) { g_cfg.reload_state_drop    = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "reloadstatedeath")  == 0) { g_cfg.reload_state_death   = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "reloadstatelevel")  == 0) { g_cfg.reload_state_level   = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "reloadstatelog")    == 0) { g_cfg.reload_state_log     = (v != 0.0); return true; }
    return false;
}

} // namespace halo
