#include "ForceTube.hpp"

#include "BlamDrive.hpp"      // unit position (the player filter) + sim_tls layout doctrine
#include "Config.hpp"
#include "uevr/API.hpp"

#include <Windows.h>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <chrono>

using uevr::API;

namespace halo {

namespace {

// ---- the vendor API, resolved from ForceTubeVR_API_x64.dll ------------------------------------
// Channel ints per their UE plugin: 0 all, 1 rifle, 2 rifleButt, 3 rifleBolt, 4/5 pistols,
// 6 other, 7 vest.
typedef void (*FtInitRifle)();
typedef void (*FtKick)(uint8_t power, int channel);
typedef void (*FtRumble)(uint8_t power, float seconds, int channel);
typedef uint8_t (*FtBattery)();
typedef char* (*FtList)();

HMODULE      s_dll = nullptr;
FtInitRifle  s_init = nullptr;
FtKick       s_kick = nullptr;
FtRumble     s_rumble = nullptr;
FtBattery    s_battery = nullptr;
FtList       s_list = nullptr;
bool s_load_tried = false;
bool s_inited = false;
uint32_t s_init_tick = 0;

// ---- the per-shot signal ----------------------------------------------------------------------
// Written by the spawn hook (sim thread), drained by the tick. A count, not a flag: a fast
// automatic can land two rounds inside one ~31 ms tick and both deserve their kick (collapsed
// into one stronger pulse below, because the stock cannot physically fire twice in 31 ms anyway).
using ft_clock = std::chrono::steady_clock;
inline long long now_ticks() { return ft_clock::now().time_since_epoch().count(); }
inline long long ms_to_ticks(int ms) {
    return std::chrono::duration_cast<ft_clock::duration>(std::chrono::milliseconds(ms)).count();
}

std::atomic<uint32_t> g_ft_shots{0};

// create_projectile: same address + prologue the dev tooling verified 2026-08-27. Duplicated here
// KNOWINGLY (BlamAim is compiled out of shipping builds); the prologue gate makes a stale copy
// fail closed with a log line instead of crashing -- see halo-uevr-game-update-recovery.
constexpr uintptr_t FT_RVA_CREATE_PROJECTILE = 0x5A0FC0;
constexpr uint8_t   FT_CREATE_PROLOGUE[] = {
    0x48,0x89,0x4C,0x24,0x08, 0x41,0x54, 0x41,0x55, 0x48,0x81,0xEC,0x98,0x04,0x00,0x00
};
constexpr uintptr_t FT_P_VEC1 = 0x1C;   // muzzle origin, Blam units

typedef uintptr_t (*CreateFn)(uintptr_t);
CreateFn s_orig_create = nullptr;
int      s_hook_id = -1;
bool     s_hook_refused = false;

uintptr_t hooked_create_for_haptics(uintptr_t params) {
    // SIM THREAD: atomics and arithmetic only. A spawn whose muzzle origin sits on the player is
    // the player's round; NPC fire from further than ~4.5 m never matches, and the rare enemy
    // shooting from inside your chest has bigger problems than a phantom kick.
    if (g_cfg.force_tube && g_unit_pvalid.load(std::memory_order_relaxed) &&
        params != 0 && !IsBadReadPtr((const void*)(params + FT_P_VEC1), 12)) {
        float v1[3];
        memcpy(v1, (const void*)(params + FT_P_VEC1), sizeof(v1));
        const float dx = v1[0] - g_unit_px.load(std::memory_order_relaxed);
        const float dy = v1[1] - g_unit_py.load(std::memory_order_relaxed);
        const float dz = v1[2] - g_unit_pz.load(std::memory_order_relaxed);
        // 1.5 blam units is ~4.5 METRES, which is most of a firefight -- every NPC muzzle inside
        // that ring kicked the stock. Tightened to ftradius (0.35 u ~ 1.1 m), and gated on the
        // player's own trigger having been down within ftfirems, which is what actually
        // separates your shot from someone else's.
        const float rr = g_cfg.force_tube_radius;
        const long long fired = g_ft_fire_at.load(std::memory_order_relaxed);
        const bool mine = fired != 0 &&
                          (now_ticks() - fired) < ms_to_ticks(g_cfg.force_tube_fire_ms);
        if (mine && dx * dx + dy * dy + dz * dz < rr * rr) {
            g_ft_shots.fetch_add(1, std::memory_order_relaxed);
        }
    }
    return s_orig_create ? s_orig_create(params) : 0;
}

bool load_dll() {
    if (s_dll != nullptr) return true;
    if (s_load_tried) return false;
    s_load_tried = true;
    // In its own subdirectory, NOT in plugins\ -- UEVR LoadLibrary's every DLL it finds there and
    // would probe the vendor DLL as a plugin.
    char path[MAX_PATH];
    if (ExpandEnvironmentStringsA(
            "%APPDATA%\\UnrealVRMod\\HaloCampaignEvolved\\forcetube\\ForceTubeVR_API_x64.dll",
            path, sizeof(path)) == 0) return false;
    s_dll = LoadLibraryA(path);
    if (s_dll == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] FORCETUBE: ForceTubeVR_API_x64.dll not found at %s -- haptics off", path);
        return false;
    }
    s_init    = (FtInitRifle)GetProcAddress(s_dll, "InitRifle");
    s_kick    = (FtKick)GetProcAddress(s_dll, "KickChannel");
    s_rumble  = (FtRumble)GetProcAddress(s_dll, "RumbleChannel");
    s_battery = (FtBattery)GetProcAddress(s_dll, "GetBatteryLevel");
    s_list    = (FtList)GetProcAddress(s_dll, "ListConnectedForceTube");
    API::get()->log_info("[Halo-CampE-UEVR] FORCETUBE: DLL loaded (init=%d kick=%d rumble=%d)",
                         s_init != nullptr, s_kick != nullptr, s_rumble != nullptr);
    return s_kick != nullptr;
}

void install_hook() {
    if (s_hook_id >= 0 || s_hook_refused) return;
    HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
    if (sim == nullptr) return;   // arrives late; retry next tick
    void* target = (void*)((uintptr_t)sim + FT_RVA_CREATE_PROJECTILE);
    if (IsBadReadPtr(target, sizeof(FT_CREATE_PROLOGUE)) ||
        memcmp(target, FT_CREATE_PROLOGUE, sizeof(FT_CREATE_PROLOGUE)) != 0) {
        s_hook_refused = true;
        API::get()->log_info("[Halo-CampE-UEVR] FORCETUBE: spawn prologue mismatch at dll+0x%llX -- "
                             "the game moved; per-shot kicks stay off",
                             (unsigned long long)FT_RVA_CREATE_PROJECTILE);
        return;
    }
    const int id = API::get()->param()->functions->register_inline_hook(
        target, (void*)&hooked_create_for_haptics, (void**)&s_orig_create);
    if (id < 0 || s_orig_create == nullptr) {
        s_hook_refused = true;
        API::get()->log_info("[Halo-CampE-UEVR] FORCETUBE: spawn hook failed (id=%d)", id);
        return;
    }
    s_hook_id = id;
    API::get()->log_info("[Halo-CampE-UEVR] FORCETUBE: spawn hook installed (dll+0x%llX)",
                         (unsigned long long)FT_RVA_CREATE_PROJECTILE);
}

} // namespace

// Exported (halo scope, not the anonymous namespace above): the XInput hook publishes into this.
std::atomic<long long> g_ft_fire_at{0};

void forcetube_note_fire(bool firing) {
    if (firing) g_ft_fire_at.store(ft_clock::now().time_since_epoch().count(),
                                   std::memory_order_relaxed);
}

void forcetube_tick() {
    if (!g_cfg.force_tube) return;
    if (!load_dll()) return;
    install_hook();

    if (!s_inited) {
        s_inited = true;
        if (s_init != nullptr) s_init();   // async BLE connect inside the vendor DLL
        s_init_tick = 1;
        return;
    }
    // One battery/connection report ~10 s after init, once -- long enough for the async connect.
    if (s_init_tick != 0 && ++s_init_tick == 320) {
        s_init_tick = 0;
        const int batt = s_battery != nullptr ? (int)s_battery() : -1;
        const char* who = s_list != nullptr ? s_list() : nullptr;
        API::get()->log_info("[Halo-CampE-UEVR] FORCETUBE: connected='%s' battery=%d",
                             who != nullptr ? who : "(n/a)", batt);
    }

    const uint32_t shots = g_ft_shots.exchange(0, std::memory_order_relaxed);
    if (shots == 0 || s_kick == nullptr) return;
    // Multiple rounds in one tick collapse into one pulse at slightly higher power -- the stock
    // cannot recycle in 31 ms, and queueing kicks would smear the cadence instead of keeping it.
    uint32_t power = (uint32_t)g_cfg.force_tube_kick;
    if (shots > 1) power = power + (255u - power) / 2u;
    s_kick((uint8_t)(power > 255u ? 255u : power), g_cfg.force_tube_channel);
}

} // namespace halo
