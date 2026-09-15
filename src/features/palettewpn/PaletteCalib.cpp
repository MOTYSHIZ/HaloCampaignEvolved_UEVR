#include "features/palettewpn/PaletteCalib.hpp"

#include "Config.hpp"
#include "core/config/CfgRead.hpp"
#include "core/registry/Features.hpp"   // features_layer
#include "Math.hpp"

#include <cstdio>
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

void pal_wpnfix_set(const std::string& key, const float q[4], const float t[3]) {
    if (key.empty()) return;
    int slot = -1;
    for (int i = 0; i < g_cfg.pal_wpnfix_count; ++i) {
        if (g_cfg.pal_wpnfix[i].match[0] != 0 && key.find(g_cfg.pal_wpnfix[i].match) != std::string::npos) { slot = i; break; }
    }
    if (slot < 0) {
        if (g_cfg.pal_wpnfix_count >= kMaxWeaponAdjust) return;
        slot = g_cfg.pal_wpnfix_count++;
        strncpy_s(g_cfg.pal_wpnfix[slot].match, sizeof(g_cfg.pal_wpnfix[slot].match), key.c_str(), _TRUNCATE);
    }
    for (int i = 0; i < 4; ++i) g_cfg.pal_wpnfix[slot].q[i] = q[i];
    for (int i = 0; i < 3; ++i) g_cfg.pal_wpnfix[slot].t[i] = t[i];
    g_cfg.pal_wpnfix[slot].captured = true;
    pal_calib_write_file();
}

// Machine-owned and rewritten whole, like halo_vr_weapons.cfg: only a captured value is written, so a shipped value
// is never copied into the player's file, where the copy would outlive a later release's better one.
void pal_calib_write_file() {
    FILE* f = nullptr;
    if (g_pal_calib_path[0] == 0 || fopen_s(&f, g_pal_calib_path, "wb") != 0 || f == nullptr) return;
    fprintf(f, "# halo_vr - PALETTE WEAPON CALIBRATION. Written by the weapon placement's captures (Page Up = grip,\r\n"
               "# Home = the weapon in hand, Page Down = aim) while the weapon follows your hand. Parsed after every\r\n"
               "# other file. Machine-owned: rewritten in full on every capture, so do not hand-edit it. It holds only\r\n"
               "# what was captured; everything else stays on the shipped values in halo_vr.cfg. Delete this file to go\r\n"
               "# back to the shipped calibration.\r\n");
    if (g_cfg.pal_grip_fix_captured && g_cfg.pal_grip_fix_valid) {
        fprintf(f, "\r\n# Rigid grip offset (Page Up): quaternion x,y,z,w then translation x,y,z in metres, UE pose frame.\r\n"
                   "palgripfix=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
                g_cfg.pal_grip_fix[0], g_cfg.pal_grip_fix[1], g_cfg.pal_grip_fix[2], g_cfg.pal_grip_fix[3],
                g_cfg.pal_grip_fix[4], g_cfg.pal_grip_fix[5], g_cfg.pal_grip_fix[6]);
    }
    if (g_cfg.pal_aim_fix_captured && g_cfg.pal_aim_fix_valid) {
        fprintf(f, "\r\n# Aim correction: quaternion x,y,z,w, right-multiplied onto the aim pose.\r\n"
                   "palaimfix=%.6f,%.6f,%.6f,%.6f\r\n",
                g_cfg.pal_aim_fix[0], g_cfg.pal_aim_fix[1], g_cfg.pal_aim_fix[2], g_cfg.pal_aim_fix[3]);
    }
    if (g_cfg.pal_aim_off_captured && g_cfg.pal_aim_off_valid) {
        fprintf(f, "\r\n# Hand-to-aim offset (Page Down), degrees. palaimcalibver 2 = the yaw is relative to the view-lock yaw.\r\n"
                   "palaimcalibver=%d\r\npalaimoffyaw=%.3f\r\npalaimoffpitch=%.3f\r\n",
                g_cfg.pal_aim_calib_ver, g_cfg.pal_aim_off_yaw, g_cfg.pal_aim_off_pitch);
    }
    bool wpn_header = false;
    for (int i = 0; i < g_cfg.pal_wpnfix_count; ++i) {
        const auto& e = g_cfg.pal_wpnfix[i];
        if (!e.captured || e.match[0] == 0) continue;
        if (!wpn_header) {
            fprintf(f, "\r\n# Per-weapon rigid delta (Home): palwpnfix=<match>,qx,qy,qz,qw,tx,ty,tz, quaternion then metres,\r\n"
                       "# UE axes (X forward, Y right, Z up). Delete a line to put that weapon back on the shipped value.\r\n");
            wpn_header = true;
        }
        fprintf(f, "palwpnfix=%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
                e.match, e.q[0], e.q[1], e.q[2], e.q[3], e.t[0], e.t[1], e.t[2]);
    }
    fclose(f);
}

} // namespace halo
