// vehcam (fork feature, Experimental): the vehicle key family (vehiclewheel, veh*; meleetargetdump and grenademeshdump ride in the same function).
// Textual fragment, included by Config.cpp at namespace halo scope, beside the other hoisted key families. Moved verbatim; not compiled on its own.
// ---- VEHICLE WHEEL + SEAT CAMERA (Vehicle.hpp). Hoisted, early-return, same C1061 reasoning.
static bool parse_veh_key(const char* key, const char* val, double v) {
    if (_stricmp(key, "vehiclewheel")   == 0) { g_cfg.vehicle_wheel = (int)v; return true; }
    if (_stricmp(key, "vehwheelpos")    == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.veh_wheel_pos[0], &g_cfg.veh_wheel_pos[1], &g_cfg.veh_wheel_pos[2]); return true; }
    if (_stricmp(key, "vehwheelrad")    == 0) { g_cfg.veh_wheel_radius = clampf((float)v, 0.05f, 1.0f); return true; }
    if (_stricmp(key, "vehwheellock")   == 0) { g_cfg.veh_wheel_lock = clampf((float)v, 10.0f, 360.0f); return true; }
    if (_stricmp(key, "vehsteersign")   == 0) { g_cfg.veh_steer_sign = (v < 0.0) ? -1 : 1; return true; }
    if (_stricmp(key, "vehwheelgrip")   == 0) { g_cfg.veh_wheel_grip = (int)v; return true; }
    if (_stricmp(key, "vehwheelhand")   == 0) { g_cfg.veh_wheel_hand = (int)v; return true; }
    if (_stricmp(key, "vehbrakemask")   == 0) { g_cfg.veh_brake_mask = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "vehwheeltilt")   == 0) { g_cfg.veh_wheel_tilt = clampf((float)v, -80.0f, 80.0f); return true; }
    if (_stricmp(key, "vehwheelmarker") == 0) { g_cfg.veh_wheel_marker = (int)v; return true; }
    if (_stricmp(key, "vehcam")         == 0) { g_cfg.veh_cam = (int)v; return true; }
    if (_stricmp(key, "vehcamoff")      == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.veh_cam_off[0], &g_cfg.veh_cam_off[1], &g_cfg.veh_cam_off[2]); return true; }
    if (_stricmp(key, "vehcamscale")    == 0) { g_cfg.veh_cam_scale = (float)v; return true; }
    if (_stricmp(key, "vehcamsrc")      == 0) { g_cfg.veh_cam_src = (int)v; return true; }
    if (_stricmp(key, "vehanchor")      == 0) { g_cfg.veh_anchor = (int)v; return true; }
    if (_stricmp(key, "vehcamanchor")   == 0) { g_cfg.veh_cam_anchor = (int)v; return true; }
    if (_stricmp(key, "vehbodydump")    == 0) { g_cfg.veh_body_dump = (v != 0.0); return true; }
    if (_stricmp(key, "vehhogdump")     == 0) { g_cfg.veh_hog_dump = (v != 0.0); return true; }
    if (_stricmp(key, "vehhogbones")    == 0) { g_cfg.veh_hog_bones = (v != 0.0); return true; }
    if (_stricmp(key, "vehhidebody")    == 0) { g_cfg.veh_hide_body = (int)v; return true; }
    if (_stricmp(key, "vehcamboomtau")  == 0) { g_cfg.veh_cam_boom_tau = clampf((float)v, 0.02f, 3.0f); return true; }
    if (_stricmp(key, "vehboomorder")   == 0) { g_cfg.veh_boom_order = (int)v; return true; }
    if (_stricmp(key, "vehfacing")      == 0) { g_cfg.veh_facing = (int)v; return true; }
    if (_stricmp(key, "vehfacingoff")   == 0) { g_cfg.veh_facing_off = (int)strtol(val, nullptr, 0); return true; }
    if (_stricmp(key, "vehfacingbias")  == 0) { g_cfg.veh_facing_bias = (float)v; return true; }
    if (_stricmp(key, "vehview")        == 0) { g_cfg.veh_view = (int)v; return true; }
    if (_stricmp(key, "vehviewflat")    == 0) { g_cfg.veh_view_flat = (int)v; return true; }
    if (_stricmp(key, "vehlog")         == 0) { g_cfg.veh_log = (int)v; return true; }
    if (_stricmp(key, "vehseatpub")     == 0) { g_cfg.veh_seat_pub = (int)v; return true; }
    if (_stricmp(key, "vehseatdirect")  == 0) { g_cfg.veh_seat_direct = (int)v; return true; }
    if (_stricmp(key, "vehcamguard")    == 0) { g_cfg.veh_cam_guard = (int)v; return true; }
    if (_stricmp(key, "vehcamguardspeed") == 0) { g_cfg.veh_cam_guard_speed = clampf((float)v, 1.0f, 5000.0f); return true; }
    if (_stricmp(key, "vehcamstalems")  == 0) { g_cfg.veh_cam_stale_ms = (int)clampf((float)v, 20.0f, 5000.0f); return true; }
    if (_stricmp(key, "vehcamhullcheck") == 0) { g_cfg.veh_cam_hull_check = (int)v; return true; }
    if (_stricmp(key, "vehcamhulldead") == 0) { g_cfg.veh_cam_hull_dead_s = clampf((float)v, 0.1f, 10.0f); return true; }
    if (_stricmp(key, "meleetargetdump")== 0) { g_cfg.melee_target_dump = (v != 0.0); return true; }
    if (_stricmp(key, "grenademeshdump")== 0) { g_cfg.grenade_mesh_dump = (v != 0.0); return true; }
    return false;
}
