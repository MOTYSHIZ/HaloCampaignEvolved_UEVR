#include "features/palettewpn/PaletteCalib.hpp"

#include "Config.hpp"
#include "core/config/CfgRead.hpp"
#include "core/registry/Features.hpp"   // features_layer
#include "Math.hpp"

#include <cstdlib>
#include <cstring>
#include <string>

namespace halo {

namespace {

// A value parsed out of halo_vr_palette_calib.cfg is a capture, and only captures are written back to it.
bool from_capture_file() { return features_layer() == kPalCalibLayer; }

// A comma list of n floats, parsed as the palette weapon always parsed its calibration: each field is written as
// it parses, and the caller treats the list as valid only when all n did.
int parse_floats(const char* val, float* out, int n) {
    const char* s = val; int k = 0;
    while (k < n && s != nullptr && *s != 0) {
        out[k++] = (float)atof(s);
        s = strchr(s, (int)0x2C); if (s != nullptr) ++s;
    }
    return k;
}

// palwpnfix=<match>,qx,qy,qz,qw,tx,ty,tz. A line without a full rotation is ignored and a missing translation is
// zero. An entry whose match is already in the table replaces it, so every weapon has one entry and the lookup
// finds the same one whichever end it searches from.
void parse_wpnfix(const char* val) {
    if (val == nullptr || val[0] == 0) return;
    char buf[256] = {0};
    strncpy_s(buf, sizeof(buf), val, _TRUNCATE);
    for (int i = (int)strlen(buf) - 1; i >= 0 && (unsigned char)buf[i] <= ' '; --i) buf[i] = 0;
    char* ctx = nullptr;
    char* tok = strtok_s(buf, ",", &ctx);
    if (tok == nullptr || tok[0] == 0) return;
    WeaponFix w{};
    strncpy_s(w.match, sizeof(w.match), tok, _TRUNCATE);
    float* fields[] = { &w.q[0], &w.q[1], &w.q[2], &w.q[3], &w.t[0], &w.t[1], &w.t[2] };
    int n = 0;
    for (; n < 7; ++n) {
        tok = strtok_s(nullptr, ",", &ctx);
        if (tok == nullptr) break;
        *fields[n] = (float)atof(tok);
    }
    if (n < 4) return;
    w.captured = from_capture_file();
    for (int i = 0; i < g_cfg.pal_wpnfix_count; ++i) {
        if (_stricmp(g_cfg.pal_wpnfix[i].match, w.match) == 0) { g_cfg.pal_wpnfix[i] = w; return; }
    }
    if (g_cfg.pal_wpnfix_count >= kMaxWeaponAdjust) return;
    g_cfg.pal_wpnfix[g_cfg.pal_wpnfix_count++] = w;
}

} // namespace

bool palette_calib_parse_key(const char* key, const char* val, double v) {
    if (_stricmp(key, "palgripfix") == 0) {
        g_cfg.pal_grip_fix_valid = (parse_floats(val, g_cfg.pal_grip_fix, 7) == 7);
        if (from_capture_file()) g_cfg.pal_grip_fix_captured = true;
        return true;
    }
    if (_stricmp(key, "palaimfix") == 0) {
        g_cfg.pal_aim_fix_valid = (parse_floats(val, g_cfg.pal_aim_fix, 4) == 4);
        if (from_capture_file()) g_cfg.pal_aim_fix_captured = true;
        return true;
    }
    if (_stricmp(key, "palaimoffyaw") == 0) {
        g_cfg.pal_aim_off_yaw = (float)v; g_cfg.pal_aim_off_valid = true;
        if (from_capture_file()) g_cfg.pal_aim_off_captured = true;
        return true;
    }
    if (_stricmp(key, "palaimoffpitch") == 0) {
        g_cfg.pal_aim_off_pitch = (float)v; g_cfg.pal_aim_off_valid = true;
        if (from_capture_file()) g_cfg.pal_aim_off_captured = true;
        return true;
    }
    if (_stricmp(key, "palaimcalibver") == 0) {
        g_cfg.pal_aim_calib_ver = (int)v;
        if (from_capture_file()) g_cfg.pal_aim_off_captured = true;
        return true;
    }
    if (_stricmp(key, "palwpnfix") == 0) { parse_wpnfix(val); return true; }
    if (_stricmp(key, "palwpncalibkey") == 0) { g_cfg.pal_wpn_calib_key = (int)strtol(val, nullptr, 0); return true; }
    return false;
}

Quat pal_apply_aim_fix(const Quat& q_src) {
    CFG_HOOK_READ;   // the aim derivation calls this off the game thread: see core/config/CfgRead.hpp
    if (!g_cfg.pal_aim_fix_valid) return q_src;
    const Quat f{g_cfg.pal_aim_fix[0], g_cfg.pal_aim_fix[1], g_cfg.pal_aim_fix[2], g_cfg.pal_aim_fix[3]};
    return quat_mul(q_src, f);
}

const WeaponFix* pal_wpnfix_find(const std::string& key) {
    if (key.empty()) return nullptr;
    for (int i = 0; i < g_cfg.pal_wpnfix_count; ++i) {
        const auto& w = g_cfg.pal_wpnfix[i];
        if (w.match[0] == 0) continue;
        if (key.find(w.match) != std::string::npos) return &w;
    }
    return nullptr;
}

} // namespace halo
