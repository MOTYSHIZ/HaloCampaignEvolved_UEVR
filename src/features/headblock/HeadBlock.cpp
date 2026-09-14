#include "HeadBlock.hpp"

#include "BlamDrive.hpp"               // g_unit_mounted: the trace stands down while mounted
#include "core/UnitState.hpp"
#include "Config.hpp"
#include "core/Services.hpp"
#include "core/ViewState.hpp"
#include "Math.hpp"                    // clampf
#include "Rig.hpp"                     // g_rig_component: the weapon the trace ignores
#include "core/EyeTrace.hpp"
#include "core/host/PluginState.hpp"
#include "uevr/API.hpp"

#include <Windows.h>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>

using namespace uevr;

namespace halo {
namespace {

using namespace eyetrace;

// ---- published by the tick, read by the view callbacks
std::atomic<float> g_allow{-1.0f};    // how far the head may extend from the body, UE cm; < 0 = unlimited
std::atomic<int>   g_eff_mode{0};     // the mode actually running (a trace mode falls back when unresolved)
std::atomic<float> g_pushed{0.0f};    // last applied pull-back, UE cm, for the log

// ---- view-callback side (one thread)
double g_shift[3]{};

float dist(const Vec3& a, const Vec3& b) {
    const float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

const char* mode_name(int m) {
    switch (m) {
    case 1: return "line-trace";
    case 2: return "sphere-sweep";
    case 3: return "lean-limit";
    default: return "off";
    }
}

// ---- THE CLAMP, run by the post-callback measurement (core/EyeTrace.cpp) at the points it always
// ran: the mode first, the shift when the frame's head offset `c` is computed, the move after it.
int headblock_clamp_mode() {
    return g_eff_mode.load(std::memory_order_relaxed);
}

void headblock_clamp_shift(int mode, const double c[3]) {
        double s[3] = {0.0, 0.0, 0.0};
        if (mode == 3) {
            // Horizontal only (UE Z is up): a crouch is not a lean.
            const double lean = g_cfg.head_block_lean;
            const double lh = std::sqrt(c[0] * c[0] + c[1] * c[1]);
            if (lh > lean && lh > 1e-6) {
                const double k = 1.0 - lean / lh;
                s[0] = c[0] * k; s[1] = c[1] * k;
            }
        } else if (mode == 1 || mode == 2) {
            const double L = g_allow.load(std::memory_order_relaxed);
            const double len = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
            if (L >= 0.0 && len > L && len > 1e-6) {
                const double k = 1.0 - L / len;
                for (int k2 = 0; k2 < 3; ++k2) s[k2] = c[k2] * k;
            }
        }
        for (int k = 0; k < 3; ++k) g_shift[k] = s[k];
        g_pushed.store((float)std::sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]), std::memory_order_relaxed);
}

bool headblock_clamp_apply(int mode, const double raw[3], double* x, double* y, double* z) {
    if (mode != 0 && (g_shift[0] != 0.0 || g_shift[1] != 0.0 || g_shift[2] != 0.0)) {
        *x = raw[0] - g_shift[0];
        *y = raw[1] - g_shift[1];
        *z = raw[2] - g_shift[2];
        return true;
    }
    return false;
}

constinit const HeadClamp kHeadBlockClamp{
    &headblock_clamp_mode,
    &headblock_clamp_shift,
    &headblock_clamp_apply,
};

}  // namespace

void headblock_tick(bool active, API::UObject* const* ignore, int n_ignore, float dt) {
    static int   s_cfg_mode = 0;
    static int   s_eff = -1;
    static float s_L = -1.0f;
    static bool  s_hit_prev = false;

    const int cfg_mode = (g_cfg.head_block >= 1 && g_cfg.head_block <= 3) ? g_cfg.head_block : 0;
    if (cfg_mode != s_cfg_mode) {
        if (g_cfg.head_block_log > 0) hblog("HEADBLOCK: headblock %d -> %d (%s)", s_cfg_mode, cfg_mode, mode_name(cfg_mode));
        s_cfg_mode = cfg_mode;
        s_L = -1.0f;
    }

    int eff = active ? cfg_mode : 0;
    // Automatic fallback: sphere -> line -> lean limit, when the reflected trace is not usable.
    if (eff == 1 || eff == 2) {
        if (!traces_ready()) eff = 3;
        else if (eff == 2 && !g_sphere.ok) eff = g_line.ok ? 1 : 3;
        else if (eff == 1 && !g_line.ok) eff = g_sphere.ok ? 2 : 3;
    }
    if (eff != s_eff) {
        if ((s_eff != -1 || eff != 0) && g_cfg.head_block_log > 0) {
            hblog("HEADBLOCK: running %s (requested %s, %s)", mode_name(eff), mode_name(cfg_mode),
                  active ? "on foot" : "standing down: menu, vehicle or cutscene");
        }
        s_eff = eff;
        s_L = -1.0f;
    }
    g_eff_mode.store(eff, std::memory_order_relaxed);
    if (eff == 0 || eff == 3) {
        g_allow.store(-1.0f, std::memory_order_relaxed);
        if (eff == 3 && g_cfg.head_block_log > 1) {
            static uint32_t s_n3 = 0;
            if ((s_n3++ % (uint32_t)g_cfg.head_block_log) == 0u) {
                hblog("HEADBLOCK lean-limit head=(%.1f %.1f %.1f) limit=%.1f pushed=%.1f cm",
                      g_head_cx.load(), g_head_cy.load(), g_head_cz.load(), g_cfg.head_block_lean,
                      g_pushed.load());
            }
        }
        return;
    }
    if (!g_have_head.load(std::memory_order_relaxed)) return;

    const Vec3 B{g_body_x.load(), g_body_y.load(), g_body_z.load()};
    const Vec3 c{g_head_cx.load(), g_head_cy.load(), g_head_cz.load()};
    const float len = std::sqrt(c.x * c.x + c.y * c.y + c.z * c.z);
    const float r = g_cfg.head_block_radius;
    const int ch = g_cfg.head_block_channel;

    float L_raw = -1.0f;
    bool hit = false;
    Vec3 loc{}, imp{};
    if (len > 0.5f) {
        if (eff == 2) {
            // The sphere centre where the sweep stopped is the furthest the head centre can go.
            const Vec3 end{B.x + c.x, B.y + c.y, B.z + c.z};
            if (run_trace(g_sphere, B, end, r, ch, ignore, n_ignore, &loc, &imp)) {
                hit = true;
                L_raw = dist(loc, B);
            }
        } else {
            // A ray to the head plus the radius; stop the head a radius short of the surface.
            const float k = (len + r) / len;
            const Vec3 end{B.x + c.x * k, B.y + c.y * k, B.z + c.z * k};
            if (run_trace(g_line, B, end, 0.0f, ch, ignore, n_ignore, &loc, &imp)) {
                hit = true;
                L_raw = (std::max)(0.0f, dist(imp, B) - r);
            }
        }
    }

    // Tighten at once -- geometry must never be seen through. Loosen at headblockrelease cm/s, so
    // a hit that clears (a corner passed, a grazing edge) eases the head out instead of popping it.
    const float rel = g_cfg.head_block_release * ((dt > 0.0f && dt < 0.5f) ? dt : 0.0f);
    if (L_raw >= 0.0f && (s_L < 0.0f || L_raw < s_L)) {
        s_L = L_raw;
    } else if (s_L >= 0.0f) {
        s_L += rel;
        if (L_raw >= 0.0f && s_L > L_raw) s_L = L_raw;
        if (L_raw < 0.0f && s_L > len + r) s_L = -1.0f;
    }
    g_allow.store(s_L, std::memory_order_relaxed);

    if (hit != s_hit_prev) {
        s_hit_prev = hit;
        if (g_cfg.head_block_log > 0) {
            if (hit) hblog("HEADBLOCK: contact (%s) head |%.1f| cm from body, allowed %.1f cm", mode_name(eff), len, L_raw);
            else     hblog("HEADBLOCK: clear (%s), releasing from %.1f cm", mode_name(eff), s_L);
        }
    }
    if (g_cfg.head_block_log > 1) {
        static uint32_t s_n = 0;
        if ((s_n++ % (uint32_t)g_cfg.head_block_log) == 0u) {
            hblog("HEADBLOCK %s body=(%.0f %.0f %.0f) head=(%.1f %.1f %.1f)|%.1f| hit=%d loc=(%.0f %.0f %.0f) "
                  "Lraw=%.1f L=%.1f pushed=%.1f cm r=%.1f ch=%d ignore=%d",
                  mode_name(eff), B.x, B.y, B.z, c.x, c.y, c.z, len, (int)hit, loc.x, loc.y, loc.z,
                  L_raw, s_L, g_pushed.load(), r, ch, n_ignore);
        }
    }
}

bool headblock_parse_key(const char* key, const char* val, double v) {
    (void)val;
    if (_stricmp(key, "headblock")        == 0) { g_cfg.head_block         = (int)clampf((float)v, 0.0f, 3.0f); return true; }
    if (_stricmp(key, "headblockchannel") == 0) { g_cfg.head_block_channel = (int)clampf((float)v, 0.0f, 32.0f); return true; }
    if (_stricmp(key, "headblocklean")    == 0) { g_cfg.head_block_lean    = clampf((float)v, 0.0f, 200.0f); return true; }
    if (_stricmp(key, "headblocklog")     == 0) { g_cfg.head_block_log     = (int)clampf((float)v, 0.0f, 100000.0f); return true; }
    if (_stricmp(key, "headblockradius")  == 0) { g_cfg.head_block_radius  = clampf((float)v, 0.0f, 50.0f); return true; }
    if (_stricmp(key, "headblockrelease") == 0) { g_cfg.head_block_release = clampf((float)v, 1.0f, 2000.0f); return true; }
    return false;
}

namespace {

void headblock_game_tick_after_leash() {
    // Plugin.cpp's own state, through the bridge: the same objects under the same names.
    const auto& g_in_menu       = *host::g_plugin_state.in_menu;
    const auto& g_cut2d_engaged = *host::g_plugin_state.cut2d_engaged;
    const auto& g_last_dt       = *host::g_plugin_state.last_dt;

    // ---- HEAD BLOCK: trace the body-eye -> head offset the view callbacks publish and hand back
    // how far the head may extend. Stands down in menus, vehicles and the 2D cutscene screen, where
    // the engine camera is not the body's eye.
    {
        const bool hb_active = (g_cfg.head_block != 0) && !g_in_menu.load() && !g_cut2d_engaged.load()
                            && !halo::g_unit_mounted.load(std::memory_order_relaxed)
                            && !g_view_seat_always.load(std::memory_order_relaxed);
        API::UObject* hb_ignore[2] = {};
        int hb_n = 0;
        if (hb_active && (g_cfg.head_block == 1 || g_cfg.head_block == 2)) {
            if (auto* pawn = API::get()->get_local_pawn(0)) hb_ignore[hb_n++] = pawn;
            if (auto* rigc = reinterpret_cast<API::UObject*>(g_rig_component.load())) {
                if (auto* wep = rigc->get_outer()) hb_ignore[hb_n++] = wep;
            }
        }
        halo::headblock_tick(hb_active, hb_ignore, hb_n, g_last_dt.load());
    }
}

}  // namespace

namespace {
bool head_block_enabled() { return g_cfg.head_block != 0; }
}  // namespace

constinit const FeatureHooks kHeadBlockHooks{
    .key                   = "headblock",
    .parse_key             = &headblock_parse_key,
    .game_tick_after_leash = &headblock_game_tick_after_leash,
    .head_clamp            = &kHeadBlockClamp,
    .enabled                    = &head_block_enabled,
    .services                   = SVC_UNIT_STATE | SVC_EYE_TRACE | SVC_HOST_FIXES,
};

}  // namespace halo
