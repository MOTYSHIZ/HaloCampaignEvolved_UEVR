#include "NodeMap.hpp"

namespace halo::palettearm {
namespace {

const char* s_failure = "not run";

bool in_range(std::uint8_t node, std::uint32_t node_count) {
    return static_cast<std::uint32_t>(node) < node_count;
}

// Every node this folder can write, so the "is it reasonable" sweep covers exactly the write set
// rather than the whole palette. Cheaper, and it keeps an unrelated broken node elsewhere in the
// rig from disabling the arms.
bool check_arm(const BlamMatrix4x3* palette, std::uint32_t node_count, const ArmNodes& arm) {
    for (std::size_t i = 0; i < arm.shoulder_count; ++i) {
        const std::uint8_t node = arm.shoulder_subtree[i];
        if (!in_range(node, node_count))            { s_failure = "arm node out of range"; return false; }
        if (!reasonable_palette_node(palette[node])) { s_failure = "arm node not reasonable"; return false; }
    }
    if (!in_range(arm.shoulder, node_count) || !in_range(arm.elbow, node_count) ||
        !in_range(arm.wrist, node_count)) {
        s_failure = "arm joint out of range";
        return false;
    }

    // The wrist has to be a wrist. Walk each finger outward and require the distance from the
    // wrist to grow at every joint, then land inside a hand's reach.
    const Vec3 wrist = palette[arm.wrist].position;
    const FingerChain* fingers[] = {&arm.index, &arm.middle, &arm.ring, &arm.pinky, &arm.thumb};
    for (const FingerChain* chain : fingers) {
        float previous = -1.0f;
        for (const std::uint8_t node : *chain) {
            if (!in_range(node, node_count))             { s_failure = "finger node out of range"; return false; }
            if (!reasonable_palette_node(palette[node])) { s_failure = "finger node not reasonable"; return false; }
            const float distance = length(palette[node].position - wrist);
            if (!std::isfinite(distance) || distance <= previous) {
                s_failure = "finger chain not ordered outward from the wrist";
                return false;
            }
            previous = distance;
        }
        // `previous` is now the fingertip distance, in Blam units.
        const float tip_metres = previous * kMetresPerBlamUnit;
        if (tip_metres < kMinTipMetres || tip_metres > kMaxTipMetres) {
            s_failure = "fingertip is not a hand's reach from the wrist";
            return false;
        }
    }
    return true;
}

} // namespace

const char* nodemap_last_failure() { return s_failure; }

bool nodemap_validate(const BlamMatrix4x3* palette, std::uint32_t node_count) {
    if (palette == nullptr) { s_failure = "null palette"; return false; }
    if (node_count != kFirstPersonNodeCount) {
        s_failure = "node count is not the 76-node first-person skeleton";
        return false;
    }
    if (!reasonable_palette_node(palette[kRootNode]) ||
        !reasonable_palette_node(palette[kWeaponMarkerNode])) {
        s_failure = "root or weapon marker node not reasonable";
        return false;
    }
    for (const std::uint8_t node : kWeaponNodes) {
        if (!in_range(node, node_count) || !reasonable_palette_node(palette[node])) {
            s_failure = "weapon node not reasonable";
            return false;
        }
    }
    if (!check_arm(palette, node_count, kRightArm)) return false;
    if (!check_arm(palette, node_count, kLeftArm))  return false;

    // Two wrists about shoulder-width apart. The single cheapest test that the LEFT map did not
    // get pointed at the right arm, or vice versa.
    const float span =
        length(palette[kRightArm.wrist].position - palette[kLeftArm.wrist].position) *
        kMetresPerBlamUnit;
    if (!std::isfinite(span) || span < kMinWristSpanMetres || span > kMaxWristSpanMetres) {
        s_failure = "the two wrists are not a plausible distance apart";
        return false;
    }

    s_failure = "ok";
    return true;
}

} // namespace halo::palettearm
