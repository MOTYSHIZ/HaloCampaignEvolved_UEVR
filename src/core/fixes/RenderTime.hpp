#pragma once

// RENDER TIME FOR THE BASE MOD'S ARMS (stabilityrendertime, a stability fix).
//
// The base mod's arm driver (armdriver 2) places the gun and both hands inside the first-person pose
// build: the gun and the aim hand from the game tick's controller pose (the rig target), the support
// hand from the pose sampled at the build. The frame is drawn later, so every drawn pose trails the
// hand by the time from that sample to the draw. Measured 2026-09-25 on a clean run: the rotation
// error follows the hand's angular speed (0.2 deg with the hand steady, 6.6 deg median above
// 90 deg/s) and not the body's movement, which is the signature of a gun behind in time; the fork's
// own driver, which re-places the gun at render, measured 20.5 ms against this driver's 30.3 ms.
//
// This re-places them at RENDER TIME. The build hands over (hook calls in PaletteArm.cpp) the frame
// it maps controller poses through and the pose each group was placed with; each render-bank copy
// hands over its bank. At the render pass the group is moved by exactly how far its controller has
// moved since, in the build's own frame:
//     basis(q) = stage_basis * blam_basis(composition * q)
//     point(p) = root + stage_basis * blam(rotate(composition, p - hmd)) * scale / 3.048
//     node'    = T(now) * T(built)^-1 * node
// The gun and the aim hand ride the aim controller; the support hand rides its own, or the gun
// while the two-handed hold is latched. The rest of the rig is left as the build drew it.
//
// Modes (stabilityrendertime): 0 off, 1 measure only (log, write nothing), 2 full, 3 rotation only,
// 4 the gun and the aim hand only. stabilityrendertimelog: one line a second.

#include "palettearm/PaletteMath.hpp"

#include <cstddef>
#include <cstdint>

namespace halo {

struct PaDriveDone {
    palettearm::BlamMatrix4x3* palette = nullptr;
    std::uint32_t node_count = 0;
    std::int32_t  model_tag = 0;
    std::int32_t  weapon_slot = 0;
    const std::uint8_t* aim_nodes = nullptr;  std::size_t aim_count = 0;   // aim wrist subtree
    const std::uint8_t* sup_nodes = nullptr;  std::size_t sup_count = 0;   // support wrist subtree
    const std::uint8_t* wpn_nodes = nullptr;  std::size_t wpn_count = 0;   // weapon branch
    palettearm::Vec3 root_position{};
    palettearm::Mat3 stage_basis{};
    palettearm::Quat composition{};
    float            wscale = 1.0f;
    // The poses each group was placed with, OpenXR stage frame (metres).
    palettearm::Vec3 aim_hmd{};  palettearm::Vec3 aim_pos{};  palettearm::Quat aim_rot{};   // the tick's sample
    palettearm::Vec3 sup_hmd{};  palettearm::Vec3 sup_pos{};  palettearm::Quat sup_rot{};   // the build's sample
    bool             sup_valid = false;
};

// SIM THREAD, the end of the base mod's live solve.
void render_time_note_build(const PaDriveDone& d);
// SIM THREAD, a render bank has just copied the live solve.
void render_time_note_bank(palettearm::BlamMatrix4x3* bank, std::uint32_t node_count, std::int32_t model_tag,
                           std::int32_t weapon_slot, std::uint8_t bank_index);
// The stereo pre-callback's render pass (features render_refresh), eye 0.
void render_time_refresh();

} // namespace halo
