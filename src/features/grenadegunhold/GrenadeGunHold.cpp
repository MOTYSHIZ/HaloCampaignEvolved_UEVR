#include "GrenadeGunHold.hpp"
#include "core/config/CfgRead.hpp"

#include "Config.hpp"
#include "Holster.hpp"                 // g_holster_throw_until: the throw press deadline
#include "Math.hpp"                    // clampf
#include "uevr/API.hpp"

#include <algorithm>
#include <chrono>

namespace halo {

bool grenadegunhold_parse_key(const char* key, const char* val, double v) {
    (void)val;
    if (_stricmp(key, "grenadegunhold")    == 0) { g_cfg.grenade_gun_hold = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "grenadegunholdms")  == 0) { g_cfg.grenade_gun_hold_ms = (int)clampf((float)v, 0.0f, 4000.0f); return true; }
    if (_stricmp(key, "grenadegunholdlog") == 0) { g_cfg.grenade_gun_hold_log = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    return false;
}

namespace {

long long ms_ticks(int ms) {
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::milliseconds(ms)).count();
}

// Sim thread only (the base mod's pose build). The live slot samples, the capture banks reuse it.
bool      s_busy     = false;
bool      s_engaged  = false;
long long s_start    = 0;

void grenadegunhold_pa_anim_gates(bool is_capture_bank, float& join_w, float& off_w, float& stock_w_all,
                                  float& hold_w) {
    (void)join_w;
    (void)stock_w_all;
    if (!is_capture_bank) {
        CFG_HOOK_READ;
        const long long until = g_holster_throw_until.load(std::memory_order_relaxed);
        const long long now   = std::chrono::steady_clock::now().time_since_epoch().count();
        const long long start = until - ms_ticks(g_cfg.holster_press_ms);
        s_busy = until != 0 && now >= start && now < start + ms_ticks(g_cfg.grenade_gun_hold_ms);
        if (s_busy != s_engaged) {
            s_engaged = s_busy;
            if (s_busy) s_start = start;
            if (g_cfg.grenade_gun_hold_log) {
                const float ms = (float)std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::duration(now - s_start)).count();
                uevr::API::get()->log_info(s_busy
                    ? "[Halo-CampE-UEVR] GRENADEHOLD engages: the throw press went to the game %.0f ms ago"
                    : "[Halo-CampE-UEVR] GRENADEHOLD releases %.0f ms after the throw press", ms);
            }
        }
    }
    if (!s_busy) return;
    hold_w = (std::max)(hold_w, 1.0f);
    off_w  = (std::max)(off_w,  1.0f);
}

bool grenade_gun_hold_enabled() { CFG_HOOK_READ; return g_cfg.grenade_gun_hold != 0; }

}  // namespace

constinit const FeatureHooks kGrenadeGunHoldHooks{
    .key           = "grenadegunhold",
    .parse_key     = &grenadegunhold_parse_key,
    .pa_anim_gates = &grenadegunhold_pa_anim_gates,
    .enabled       = &grenade_gun_hold_enabled,
    .services      = 0,
};

} // namespace halo
