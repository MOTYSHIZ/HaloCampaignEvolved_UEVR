#include "features/palettewpn/PalettePoseProvider.hpp"
#include "core/config/CfgRead.hpp"

#include "features/palettewpn/BlamPalette.hpp"      // palette_trim_rotations
#include "features/palettewpn/PaletteTwoHand.hpp"   // palette_two_hand_blend
#include "Config.hpp"
#include "features/palettewpn/PaletteArmDriver.hpp"   // palette_weapon_mode
#include "features/palettewpn/PaletteFrame.hpp"      // stomp_mark
#include "features/palettewpn/PoseLatch.hpp"         // the stamped intents

#include <atomic>

namespace halo {

// The palette's measured barrel axis in the trimmed pose frame (Plugin.cpp BARRELAXIS), for aimbore=3.
extern std::atomic<float> g_barrel_axis_x, g_barrel_axis_y, g_barrel_axis_z;
extern std::atomic<bool>  g_barrel_axis_valid;
extern std::atomic<float> g_dbg_pose_w_x, g_dbg_pose_w_y, g_dbg_pose_w_z, g_dbg_pose_w_w;   // PaletteFrame.cpp

namespace {
bool barrel_axis(Vec3* out) {
    if (!g_barrel_axis_valid.load(std::memory_order_relaxed)) return false;
    *out = Vec3{g_barrel_axis_x.load(std::memory_order_relaxed), g_barrel_axis_y.load(std::memory_order_relaxed),
                g_barrel_axis_z.load(std::memory_order_relaxed)};
    return true;
}
// RETSTAMP render placement (aimreticulestamp 1/2) draws the STAMPED hand intent, and that stamp is
// only taken while the pose latch runs in palette weapon mode: poselatch 0 never stores it, and
// poselatch 3 stores it from the XInput-rate law (aimrate=1) only.
bool stamp_available() {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    if (g_cfg.pose_latch == 0) return false;
    if (g_cfg.pose_latch == 3 && !g_cfg.aim_rate_render) return false;
    return true;
}
bool stamped_intent(bool two_back, float* yaw, float* pitch) {
    const bool ok = two_back ? g_intent_prev2_ok.load(std::memory_order_relaxed)
                             : g_intent_prev_ok.load(std::memory_order_relaxed);
    if (!ok) return false;
    *yaw   = two_back ? g_intent_prev2_y.load(std::memory_order_relaxed) : g_intent_prev_y.load(std::memory_order_relaxed);
    *pitch = two_back ? g_intent_prev2_p.load(std::memory_order_relaxed) : g_intent_prev_p.load(std::memory_order_relaxed);
    return true;
}
float roll_trim_deg() { CFG_HOOK_READ; return g_cfg.palette_roll_trim; }
// The palette's WORLD pose as last resolved (PaletteFrame publishes), read raw: the consumer normalizes.
bool weapon_quat(Quat* out) {
    *out = Quat{g_dbg_pose_w_x.load(std::memory_order_relaxed), g_dbg_pose_w_y.load(std::memory_order_relaxed),
                g_dbg_pose_w_z.load(std::memory_order_relaxed), g_dbg_pose_w_w.load(std::memory_order_relaxed)};
    return true;
}
} // namespace

constinit const PalettePoseProvider kPalettePoseProvider{
    &palette_weapon_mode,
    &palette_trim_rotations,
    &palette_two_hand_blend,
    &barrel_axis,
    &stamp_available,
    &stamped_intent,
    &stomp_mark,
    &roll_trim_deg,
    &weapon_quat,
};

} // namespace halo
