#include "core/CameraBob.hpp"

#include "Config.hpp"
#include "Math.hpp"
#include "MotionAimControl.hpp"   // read_control_rotation
#include "Rig.hpp"                // g_rig_parent, call_ret_vec3
#include "core/host/PluginState.hpp"
#include "uevr/API.hpp"

#include <atomic>
#include <chrono>
#include <cmath>

using namespace uevr;

namespace halo {

std::atomic<float> g_bob_x{0.0f}, g_bob_y{0.0f}, g_bob_z{0.0f};

void camera_bob_tick() {
    // Plugin.cpp's own state, through the bridge: the same object under the same name.
    const auto& g_stick_mode = *host::g_plugin_state.stick_mode;

    // ---- CAMERA BOB: measure the camera component against the pawn root in the aim-yaw frame,
    // low-pass the slow part (eye height, crouch), publish the fast remainder as the bob.
    {
        const bool want_bob = g_cfg.bob_cancel || g_cfg.bob_log;
        Vec3 bob{0.0f, 0.0f, 0.0f};
        if (want_bob && g_rig_parent != nullptr && !g_stick_mode.load()) {
            auto* pawn_b = reinterpret_cast<API::UObject*>(API::get()->get_local_pawn(0));
            Vec3 root{}, camb{}; double cpb = 0.0, cyb = 0.0;
            if (pawn_b != nullptr && call_ret_vec3(pawn_b, L"K2_GetActorLocation", &root)
                && call_ret_vec3(g_rig_parent, L"K2_GetComponentLocation", &camb)
                && read_control_rotation(&cpb, &cyb, nullptr)) {
                static Vec3 s_lp{}; static bool s_have = false;
                static std::chrono::steady_clock::time_point s_t{};
                const auto nowb = std::chrono::steady_clock::now();
                const float dtb = s_have ? std::chrono::duration<float>(nowb - s_t).count() : 0.0f;
                s_t = nowb;
                const float ayb = (float)cyb * DEG2RAD, cb = std::cos(ayb), snb = std::sin(ayb);
                const Vec3 dw{camb.x - root.x, camb.y - root.y, camb.z - root.z};
                const Vec3 dl{ dw.x * cb + dw.y * snb, -dw.x * snb + dw.y * cb, dw.z};   // aim-yaw frame
                const float taub = (g_cfg.bob_tau > 0.02f) ? g_cfg.bob_tau : 0.4f;
                if (!s_have || dtb <= 0.0f || dtb > 0.5f) { s_lp = dl; s_have = true; }
                else { const float ab = clampf(dtb / taub, 0.0f, 1.0f); s_lp.x += (dl.x - s_lp.x) * ab; s_lp.y += (dl.y - s_lp.y) * ab; s_lp.z += (dl.z - s_lp.z) * ab; }
                const Vec3 bl{dl.x - s_lp.x, dl.y - s_lp.y, dl.z - s_lp.z};
                bob = Vec3{bl.x * cb - bl.y * snb, bl.x * snb + bl.y * cb, bl.z};        // back to world
                if (g_cfg.bob_log) {
                    static uint32_t nlog = 0;
                    if ((nlog++ % 2u) == 0u) {
                        API::get()->log_info("[Halo-CampE-UEVR] BOB dt=%.1fms d_local=(%.2f %.2f %.2f) lp=(%.2f %.2f %.2f) bob=(%.2f %.2f %.2f)cm aim=%.1f",
                                             dtb * 1000.0f, dl.x, dl.y, dl.z, s_lp.x, s_lp.y, s_lp.z, bl.x, bl.y, bl.z, (float)cyb);
                    }
                }
            }
        }
        if (!g_cfg.bob_cancel) bob = Vec3{0.0f, 0.0f, 0.0f};
        halo::g_bob_x.store(bob.x, std::memory_order_relaxed);
        halo::g_bob_y.store(bob.y, std::memory_order_relaxed);
        halo::g_bob_z.store(bob.z, std::memory_order_relaxed);
    }
}

void camera_bob_reset() {
    g_bob_x.store(0.0f, std::memory_order_relaxed);
    g_bob_y.store(0.0f, std::memory_order_relaxed);
    g_bob_z.store(0.0f, std::memory_order_relaxed);
}

} // namespace halo
