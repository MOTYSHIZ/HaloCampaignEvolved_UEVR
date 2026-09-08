#include "NodeMap.hpp"

namespace halo::palettearm {
namespace {

NodeMapFailure s_failure{};

// Record a failure with no location. Clears any stale site from a previous run -- a half-updated
// site is worse than none, because it reads as a precise answer to the wrong question.
bool fail(const char* reason) {
    s_failure = NodeMapFailure{};
    s_failure.reason = reason;
    return false;
}

bool fail_at(const char* reason, bool right_arm, std::size_t chain, std::size_t joint,
             std::uint8_t node) {
    s_failure = NodeMapFailure{};
    s_failure.reason    = reason;
    s_failure.has_site  = true;
    s_failure.right_arm = right_arm;
    s_failure.chain     = static_cast<std::uint8_t>(chain);
    s_failure.joint     = static_cast<std::uint8_t>(joint);
    s_failure.node      = node;
    return false;
}

bool in_range(std::uint8_t node, std::uint32_t node_count) {
    return static_cast<std::uint32_t>(node) < node_count;
}

// Every node this folder can write, so the "is it reasonable" sweep covers exactly the write set
// rather than the whole palette. Cheaper, and it keeps an unrelated broken node elsewhere in the
// rig from disabling the arms.
bool check_arm(const BlamMatrix4x3* palette, std::uint32_t node_count, const ArmNodes& arm,
               bool right_arm) {
    if (arm.shoulder_subtree == nullptr || arm.shoulder_count == 0 ||
        arm.elbow_subtree == nullptr    || arm.elbow_count == 0 ||
        arm.wrist_subtree == nullptr    || arm.wrist_count == 0) {
        return fail("arm has no subtree");
    }
    for (std::size_t i = 0; i < arm.shoulder_count; ++i) {
        const std::uint8_t node = arm.shoulder_subtree[i];
        if (!in_range(node, node_count))            { return fail("arm node out of range"); }
        if (!reasonable_palette_node(palette[node])) { return fail("arm node not reasonable"); }
    }
    if (!in_range(arm.shoulder, node_count) || !in_range(arm.elbow, node_count) ||
        !in_range(arm.wrist, node_count)) {
        return fail("arm joint out of range");
    }

    // The wrist has to be a wrist -- tested on BONE SEGMENTS, not on distance from the wrist.
    //
    // THIS TEST USED TO REQUIRE THE DISTANCE FROM THE WRIST TO GROW AT EVERY JOINT, AND THAT WAS
    // SIMPLY WRONG. Halo's first-person hands are authored CURLED AROUND A WEAPON GRIP -- this
    // folder's own apply_finger_openness() says so in as many words, and builds its open pose by
    // UNcurling them. A curled finger's tip comes back TOWARD the wrist, so "monotonically
    // outward" describes an open hand and rejects the real one.
    //
    // Measured on a live 76-node palette from this build (docs\Perf\logs60824-232806-padump.txt),
    // right middle finger, distance from wrist in cm:
    //
    //     12.0  ->  15.1  ->  13.1  ->  9.9        out, then curling back in
    //
    // The old rule accepted 3 of the 10 real chains. It was rejecting a CORRECT map, and that is
    // what kept armdriver=2 dark: the arms were never posed because the guard refused a skeleton
    // that was right all along.
    //
    // What is actually invariant under curl is the SKELETON, not the pose: a metacarpal reaches a
    // hand-ish distance from the wrist, and each bone after it is a finger-bone length. Bending
    // the joints moves the tips; it cannot change how long the bones are. On the same measured
    // palette this accepts 10 of 10 real chains while accepting only 0.28% of random 4-node
    // chains drawn from the same palette -- so it is not a relaxation, it is a stricter test of
    // the right property.
    const Vec3 wrist = palette[arm.wrist].position;
    const FingerChain* fingers[] = {&arm.index, &arm.middle, &arm.ring, &arm.pinky, &arm.thumb};
    for (std::size_t chain_index = 0; chain_index < 5; ++chain_index) {
        const FingerChain& chain = *fingers[chain_index];
        for (std::size_t joint = 0; joint < chain.size(); ++joint) {
            const std::uint8_t node = chain[joint];
            if (!in_range(node, node_count)) {
                return fail_at("finger node out of range", right_arm, chain_index, joint, node);
            }
            if (!reasonable_palette_node(palette[node])) {
                return fail_at("finger node not reasonable", right_arm, chain_index, joint, node);
            }
        }

        // Wrist -> metacarpal. Spans the palm, so it is longer than a bone and shorter than a hand.
        const float palm_m =
            length(palette[chain[0]].position - wrist) * kMetresPerBlamUnit;
        if (!std::isfinite(palm_m) || palm_m < kValidPalmMinMetres || palm_m > kValidPalmMaxMetres) {
            return fail_at("wrist-to-metacarpal is not a palm's width",
                           right_arm, chain_index, 0, chain[0]);
        }

        // The three phalanx bones. Length only -- direction is pose, and pose is not ours to judge.
        for (std::size_t joint = 0; joint + 1 < chain.size(); ++joint) {
            const float bone_m =
                length(palette[chain[joint + 1]].position - palette[chain[joint]].position) *
                kMetresPerBlamUnit;
            if (!std::isfinite(bone_m) || bone_m < kValidBoneMinMetres || bone_m > kValidBoneMaxMetres) {
                return fail_at("finger bone is not a finger-bone length",
                               right_arm, chain_index, joint + 1, chain[joint + 1]);
            }
        }

        // The fingertip still has to be within reach of the wrist. Unchanged, and it holds for a
        // curled hand too -- curling shortens the reach, it does not send the tip to another room.
        const float tip_metres =
            length(palette[chain.back()].position - wrist) * kMetresPerBlamUnit;
        if (tip_metres < kMinTipMetres || tip_metres > kMaxTipMetres) {
            return fail_at("fingertip is not a hand's reach from the wrist",
                           right_arm, chain_index, chain.size() - 1, chain.back());
        }
    }
    return true;
}

} // namespace

const char* nodemap_last_failure() { return s_failure.reason; }

const NodeMapFailure& nodemap_last_failure_site() { return s_failure; }

const char* nodemap_chain_name(std::uint8_t chain) {
    switch (chain) {
        case 0: return "index";
        case 1: return "middle";
        case 2: return "ring";
        case 3: return "pinky";
        case 4: return "thumb";
        default: return "?";
    }
}

bool nodemap_validate(const BlamMatrix4x3* palette, std::uint32_t node_count, const NodeMap& map) {
    if (palette == nullptr) return fail("null palette");
    if (map.node_count == 0) return fail("the map declares no node count");
    if (node_count != map.node_count) {
        // The map is only meaningful on the skeleton it was authored or resolved for. For the
        // hardcoded table that count is 76, so this is bit-for-bit the old check.
        return fail("node count does not match the map's skeleton");
    }
    if (!in_range(map.root, node_count) || !reasonable_palette_node(palette[map.root])) {
        return fail("root node not reasonable");
    }
    if (map.weapon_marker != kNoNode) {
        if (!in_range(map.weapon_marker, node_count) ||
            !reasonable_palette_node(palette[map.weapon_marker])) {
            return fail("root or weapon marker node not reasonable");
        }
    }
    for (std::size_t i = 0; i < map.weapon_count; ++i) {
        const std::uint8_t node = map.weapon_nodes[i];
        if (!in_range(node, node_count) || !reasonable_palette_node(palette[node])) {
            return fail("weapon node not reasonable");
        }
    }
    if (!check_arm(palette, node_count, map.right, /*right_arm=*/true))  return false;
    if (!check_arm(palette, node_count, map.left,  /*right_arm=*/false)) return false;

    // The two arms must not be the same arm. A derived map that pointed both sides at one hand
    // would pass everything above -- the wrist-span test below would catch it too, but only by
    // arithmetic accident, and an index collision deserves to be named as one.
    if (map.right.wrist == map.left.wrist || map.right.shoulder == map.left.shoulder ||
        map.right.elbow == map.left.elbow) {
        return fail("the two arms share a joint");
    }

    // Two wrists about shoulder-width apart. The single cheapest test that the LEFT map did not
    // get pointed at the right arm, or vice versa.
    const float span =
        length(palette[map.right.wrist].position - palette[map.left.wrist].position) *
        kMetresPerBlamUnit;
    if (!std::isfinite(span) || span < kMinWristSpanMetres || span > kMaxWristSpanMetres) {
        return fail("the two wrists are not a plausible distance apart");
    }

    s_failure = NodeMapFailure{};
    s_failure.reason = "ok";
    return true;
}

} // namespace halo::palettearm
