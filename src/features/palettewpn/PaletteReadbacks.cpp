#include "features/palettewpn/PaletteReadbacks.hpp"
#include "core/config/CfgRead.hpp"

#include "Config.hpp"
#include "features/palettewpn/PoseLatch.hpp"   // pose_latch_last_gen, aim_writer_note_blam
#include "Rig.hpp"                // call_socket_location
#include "UeObject.hpp"           // RIG_PARAM_BUF, make_fname
#include "core/host/BlamDriveState.hpp"   // g_ctl_rec
#include "uevr/API.hpp"

#include <Windows.h>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>

using namespace uevr;

namespace halo {

void stomp_mark(int point, float yaw, float e0, float e1, float e2);   // Plugin.cpp, STOMPLOG ring
extern std::atomic<unsigned> g_tick_id;                                 // Plugin.cpp

// The socket's WORLD position, straight from the posed skeleton. Same read derive_pivot does, minus
// the component-relative step -- kept separate rather than folded in because the two answer
// different questions and derive_pivot's fail-closed length check is about pivot sanity, not about
// whether a readback succeeded.
bool rig_socket_world(API::UObject* rig, const wchar_t* socket, Vec3* out) {
    if (rig == nullptr) return false;
    Vec3 sock{};
    if (!call_socket_location(rig, socket, &sock)) return false;
    if (!std::isfinite(sock.x) || !std::isfinite(sock.y) || !std::isfinite(sock.z)) return false;
    *out = sock;
    return true;
}

// GetSocketRotation(FName) -> FRotator (pitch, yaw, roll as doubles), same layout rules as the
// location call: FName at 0, return at offset 8. The socket's WORLD rotation from the posed
// skeleton -- the gun's pointing direction as rendered, not as written. Position alone proved the
// bone lands where we put it; this is what makes the barrel-vs-aim-ray angle a logged number.
bool rig_socket_world_rot(API::UObject* rig, const wchar_t* socket, Vec3* out_pyr) {
    if (rig == nullptr || out_pyr == nullptr) return false;
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    API::FName name = make_fname(socket);
    memcpy(params, &name, sizeof(int32_t) * 2);
    rig->call_function(L"GetSocketRotation", params);
    auto* d = reinterpret_cast<double*>(params + 8);
    if (!std::isfinite(d[0]) || !std::isfinite(d[1]) || !std::isfinite(d[2])) return false;
    *out_pyr = Vec3{(float)d[0], (float)d[1], (float)d[2]};
    return true;
}

// FRAMEAUDIT: the Blam control record's current angles, converted back to the UE-convention
// degrees desired_aim_now() uses, so they can be matched against the generation table.
bool blam_ctl_read_ue_deg(float* yaw_deg, float* pitch_deg) {
    const uintptr_t rec = host::g_blamdrive_state.ctl_rec->load(std::memory_order_relaxed);
    if (rec == 0 || IsBadReadPtr((void*)rec, 8)) return false;
    const float* fp = (const float*)rec;
    if (!std::isfinite(fp[0]) || !std::isfinite(fp[1])) return false;
    const float asign = (g_cfg.blam_angles_ysign >= 0) ? 1.0f : -1.0f;
    float y = asign * fp[0] * RAD2DEG - g_cfg.blam_yaw_off;
    while (y > 180.0f) y -= 360.0f;
    while (y < -180.0f) y += 360.0f;
    *yaw_deg = y;
    *pitch_deg = fp[1] * RAD2DEG - g_cfg.blam_pitch_off;
    return true;
}

void palette_wpn_sim_record_written(float yaw, float pitch) {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    aim_writer_note_blam(yaw, pitch);
    if (g_cfg.stomp_log != 0) {   // Point 23: the Blam record write, once per snapshot generation (this runs ~2600/s).
        static uint32_t s_last_gen = 0xFFFFFFFFu;
        const uint32_t g = pose_latch_last_gen();
        if (g != s_last_gen) {
            s_last_gen = g;
            stomp_mark(23, yaw, pitch, (float)g, (float)g_tick_id.load(std::memory_order_relaxed));
        }
    }
}

} // namespace halo
