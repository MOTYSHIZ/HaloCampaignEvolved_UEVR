#include "features/palettewpn/PalettePoseProvider.hpp"

#include "BlamPalette.hpp"      // palette_trim_rotations
#include "PaletteTwoHand.hpp"   // palette_two_hand_blend
#include "features/palettewpn/PaletteArmDriver.hpp"   // palette_weapon_mode

#include <atomic>

namespace halo {

// The palette's measured barrel axis in the trimmed pose frame (Plugin.cpp BARRELAXIS), for aimbore=3.
extern std::atomic<float> g_barrel_axis_x, g_barrel_axis_y, g_barrel_axis_z;
extern std::atomic<bool>  g_barrel_axis_valid;

namespace {
bool barrel_axis(Vec3* out) {
    if (!g_barrel_axis_valid.load(std::memory_order_relaxed)) return false;
    *out = Vec3{g_barrel_axis_x.load(std::memory_order_relaxed), g_barrel_axis_y.load(std::memory_order_relaxed),
                g_barrel_axis_z.load(std::memory_order_relaxed)};
    return true;
}
} // namespace

constinit const PalettePoseProvider kPalettePoseProvider{
    &palette_weapon_mode,
    &palette_trim_rotations,
    &palette_two_hand_blend,
    &barrel_axis,
};

} // namespace halo
