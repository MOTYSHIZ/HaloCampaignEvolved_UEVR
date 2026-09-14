#include "core/fixes/HmdPoseGate.hpp"

#include <string_view>

#include "uevr/API.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>

using namespace uevr;

namespace halo {

bool hmd_pose_plausible(const Vec3& hp) {
    bool hp_ok = true;
        // HMD POSE PLAUSIBILITY GATE. One dropped tracking frame (measured: hp jumped 5.6 m for
        // ONE 18 ms tick, then back) went straight into the leash, which yanked the standing
        // origin 4.6 m to clamp it; the pose recovered and roomscale sprinted the biped a metre
        // to walk out the phantom offset. A head cannot move faster than ~5 m/s, so a tick that
        // claims it is a dropout -- skip the leash and roomscale for that tick and do NOT advance
        // the reference, so the next good pose is judged against the last good one.
        if (hp_ok) {
            static Vec3 s_hp_good{}; static bool s_hp_have = false;
            static std::chrono::steady_clock::time_point s_hp_t{};
            const auto now_hp = std::chrono::steady_clock::now();
            const float dth = s_hp_have ? std::chrono::duration<float>(now_hp - s_hp_t).count() : 0.0f;
            const float jump = s_hp_have ? std::sqrt((hp.x - s_hp_good.x) * (hp.x - s_hp_good.x) + (hp.y - s_hp_good.y) * (hp.y - s_hp_good.y) + (hp.z - s_hp_good.z) * (hp.z - s_hp_good.z)) : 0.0f;
            // Reject a >5 m/s jump; after 0.5 s without a good pose accept whatever comes (a real
            // recenter/teleport must be able to win eventually).
            const bool implausible = s_hp_have && dth > 0.0f && dth < 0.5f && jump > 5.0f * dth && jump > 0.05f;
            if (implausible) {
                static uint32_t s_rej = 0;
                if ((s_rej++ % 16u) == 0u) {
                    API::get()->log_info("[Halo-CampE-UEVR] HMD pose rejected: %.2f m in %.1f ms (%.1f m/s) -- dropped tracking frame, leash/roomscale skipped",
                                         jump, dth * 1000.0f, jump / dth);
                }
                hp_ok = false;
            } else { s_hp_good = hp; s_hp_t = now_hp; s_hp_have = true; }
        }
    return hp_ok;
}

} // namespace halo
