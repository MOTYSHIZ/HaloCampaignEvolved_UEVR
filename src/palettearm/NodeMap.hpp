// Which palette index is which bone, for Halo's 76-node first-person skeleton.
//
// THESE ARE STRUCT OFFSETS WEARING A DIFFERENT HAT. Read the "HARDCODING AN ADDRESS OR STRUCT
// OFFSET" section of the project CLAUDE.md before touching them. Every number in this file is a
// measurement of ONE skeleton, read off the shipped Spartan assault-rifle rig with Baboon
// (elliotttate's measurement, carried over with permission; he reports all 16 shipped Campaign
// Evolved first-person skeletons share this topology). If the game repacks its first-person tags,
// index 19 stops being the right wrist and becomes something else that is also a perfectly valid
// matrix -- and we would write a hand pose into it, every frame, for every user, silently.
//
// Which is why nothing in this folder writes through these without nodemap_validate() passing
// first. See its comment: it does not ask "did the numbers change", it asks "does the thing at
// index 19 still behave like a wrist".
//
// ⚠️ THESE ARE NO LONGER THE SOURCE -- THEY ARE RUNG 2. As of the node-discovery lane
// (NodeDiscovery.hpp, docs\PALETTEARM-NODEMAP.md) the map that gets used is DERIVED from the
// palette in front of us, and this table is what we fall back to when derivation cannot decide.
// That is question 1 of the CLAUDE.md hardcoding section applied to a table of bone indices:
// resolve it, keep the constant as the fallback. The reason for the change is not theoretical --
// on the shipped build these indices are REJECTED by the validator below, at the finger-ordering
// test, which is what made `armdriver=2` do nothing at all.
//
// Ported from elliotttate's HaloCampaignEvolved-UEVR (main.cpp @ 62ee34f) with permission.

#pragma once

#include "PaletteMath.hpp"

#include <array>
#include <cstdint>

namespace halo::palettearm {

// The engine builds exactly this many nodes for a first-person weapon. A count, not an address --
// but it is the cheapest single thing that says "this is the skeleton we measured": a different
// node count is a different rig and every index below is void.
//
// Note this count belongs to THE TABLE BELOW, not to the game: it is the count elliotttate's
// indices were measured against. NodeMap::node_count carries it per-map for exactly that reason,
// so a derived map is checked against ITS OWN count rather than against this one.
constexpr std::uint32_t kFirstPersonNodeCount = 76;

// "no such node". Palette indices are 8-bit here, so 0xFF is both out of range for any rig we
// would touch and impossible to confuse with a real index.
constexpr std::uint8_t kNoNode = 0xFF;

// The largest palette any of this will look at. Indices are 8-bit and 0xFF is the sentinel, so
// this is a representability ceiling, not a measurement.
constexpr std::size_t kMaxPaletteNodes = 255;

// The three sibling trees hanging off the root: right shoulder 5, left shoulder 6, weapon 7.
// Node 0 is the camera-control root and is NEVER written -- it is the frame everything else is
// expressed in, and moving it moves the view.
constexpr std::uint8_t kRootNode        = 0;
constexpr std::uint8_t kWeaponMarkerNode = 8;                  // authored "primaryweapon" marker
inline constexpr std::array<std::uint8_t, 3> kWeaponNodes{7, 8, 22};

// One finger: metacarpal then three phalanges, root-first. Rotating joint N rotates N and every
// index after it, so the order is data, not presentation.
using FingerChain = std::array<std::uint8_t, 4>;

// The joints of one arm, and the subtrees each rotates.
//
// `shoulder_subtree` is every node that moves rigidly with the whole arm; `elbow_subtree` is what
// hangs below the elbow; `wrist_subtree` is the hand. They are subsets of each other by
// construction, and the solver relies on that -- rotating about the shoulder must carry the elbow
// and wrist sets with it.
struct ArmNodes {
    std::uint8_t shoulder{};
    std::uint8_t elbow{};
    std::uint8_t wrist{};

    const std::uint8_t* shoulder_subtree{};
    std::size_t         shoulder_count{};
    const std::uint8_t* elbow_subtree{};
    std::size_t         elbow_count{};
    const std::uint8_t* wrist_subtree{};
    std::size_t         wrist_count{};

    FingerChain index{};
    FingerChain middle{};
    FingerChain ring{};
    FingerChain pinky{};
    FingerChain thumb{};
};

// ---- RIGHT ARM: shoulder 5, elbow 16, wrist 19 -------------------------------------------------

inline constexpr std::array<std::uint8_t, 34> kRightShoulderSubtree{
    5, 10, 11, 12, 16, 17, 18, 19, 20, 21, 30, 31, 34, 36, 37, 40,
    41, 43, 44, 45, 48, 49, 53, 55, 56, 59, 60, 63, 65, 66, 69, 70,
    73, 74};
inline constexpr std::array<std::uint8_t, 29> kRightElbowSubtree{
    16, 17, 18, 19, 20, 30, 31, 34, 36, 37, 40, 41, 43, 44, 45,
    48, 49, 53, 55, 56, 59, 60, 63, 65, 66, 69, 70, 73, 74};
inline constexpr std::array<std::uint8_t, 24> kRightWristSubtree{
    19, 30, 31, 36, 37, 40, 41, 43, 44, 45, 48, 49,
    53, 55, 56, 59, 60, 63, 65, 66, 69, 70, 73, 74};

// ---- LEFT ARM: shoulder 6, elbow 9, wrist 25 ---------------------------------------------------

inline constexpr std::array<std::uint8_t, 34> kLeftShoulderSubtree{
    6, 9, 13, 14, 15, 23, 24, 25, 26, 27, 28, 29, 32, 33, 35, 38,
    39, 42, 46, 47, 50, 51, 52, 54, 57, 58, 61, 62, 64, 67, 68, 71,
    72, 75};
inline constexpr std::array<std::uint8_t, 29> kLeftElbowSubtree{
    9, 24, 25, 26, 27, 28, 29, 32, 33, 35, 38, 39, 42, 46, 47,
    50, 51, 52, 54, 57, 58, 61, 62, 64, 67, 68, 71, 72, 75};
inline constexpr std::array<std::uint8_t, 24> kLeftWristSubtree{
    25, 28, 29, 32, 33, 38, 39, 42, 46, 47, 50, 51,
    52, 54, 57, 58, 61, 62, 64, 67, 68, 71, 72, 75};

inline constexpr ArmNodes kRightArm{
    5, 16, 19,
    kRightShoulderSubtree.data(), kRightShoulderSubtree.size(),
    kRightElbowSubtree.data(),    kRightElbowSubtree.size(),
    kRightWristSubtree.data(),    kRightWristSubtree.size(),
    /*index */ {41, 53, 63, 73},
    /*middle*/ {36, 49, 60, 70},
    /*ring  */ {31, 44, 56, 66},
    /*pinky */ {43, 55, 65, 74},
    /*thumb */ {37, 48, 59, 69},
};

inline constexpr ArmNodes kLeftArm{
    6, 9, 25,
    kLeftShoulderSubtree.data(), kLeftShoulderSubtree.size(),
    kLeftElbowSubtree.data(),    kLeftElbowSubtree.size(),
    kLeftWristSubtree.data(),    kLeftWristSubtree.size(),
    /*index */ {28, 42, 54, 64},
    /*middle*/ {33, 46, 57, 67},
    /*ring  */ {38, 51, 61, 71},
    /*pinky */ {52, 62, 72, 75},
    /*thumb */ {32, 47, 58, 68},
};

// ---- A WHOLE SKELETON'S MAP --------------------------------------------------------------------
//
// One of these is the hardcoded table above; another comes out of NodeDiscovery. Everything
// downstream takes a NodeMap rather than reaching for the constants, which is what lets the SAME
// validator judge both -- discovery does not get a softer test than elliotttate's numbers get.
//
// `node_count` is the count the map was authored or resolved FOR. Validation requires the palette
// in hand to match it exactly, so the hardcoded map still refuses anything that is not 76 nodes.
struct NodeMap {
    std::uint32_t node_count{};
    std::uint8_t  root{kRootNode};
    std::uint8_t  weapon_marker{kNoNode};        // kNoNode when the map cannot name one
    const std::uint8_t* weapon_nodes{};          // may be null/empty for a derived map
    std::size_t         weapon_count{};
    ArmNodes right{};
    ArmNodes left{};

    const ArmNodes& arm(bool is_right) const { return is_right ? right : left; }
};

inline constexpr NodeMap kHardcodedNodeMap{
    kFirstPersonNodeCount, kRootNode, kWeaponMarkerNode,
    kWeaponNodes.data(), kWeaponNodes.size(),
    kRightArm, kLeftArm};

// elliotttate's measured table, as a NodeMap. RUNG 2 of the resolution cascade; see
// NodeDiscovery.hpp for rung 1 and for what happens when both fail.
inline const NodeMap& hardcoded_node_map() { return kHardcodedNodeMap; }

inline const ArmNodes& arm_nodes(bool right) { return right ? kRightArm : kLeftArm; }

// Reach limits used by the validator, in metres. Generous on purpose: this separates "a hand" from
// "not a hand", and rejecting a real hand costs the entire feature.
// Bone-length bounds, in metres. These test the SKELETON, which curl cannot change -- see the
// long note in NodeMap.cpp's check_arm(). Measured on a live palette from this build: palm spans
// were 5.8-12.0 cm and phalanx bones 2.2-6.1 cm, so these bounds sit comfortably around real
// values without being loose enough to admit arbitrary node pairs (measured false-accept rate on
// random 4-node chains from the same palette: 0.28%).
// 0.02 rather than 0.03: the measured rig's shortest palm span is 5.8 cm, but a compact rig
// (the suite's synthetic hand) puts its inner metacarpals 2.77 cm out, and rejecting those
// would make this a test of hand SIZE rather than hand SHAPE.
constexpr float kValidPalmMinMetres = 0.02f;
constexpr float kValidPalmMaxMetres = 0.15f;
constexpr float kValidBoneMinMetres = 0.005f;
constexpr float kValidBoneMaxMetres = 0.08f;

constexpr float kMinTipMetres       = 0.04f;
constexpr float kMaxTipMetres       = 0.30f;
constexpr float kMinWristSpanMetres = 0.05f;
constexpr float kMaxWristSpanMetres = 1.20f;

// Why the guard cannot be "did anything move".
//
// The likeliest way this file rots is not the indices going wild -- it is a tag repack landing
// index 19 on the NEIGHBOURING bone. A neighbouring bone moves exactly as much as the right one,
// is exactly as finite, and passes every cheap sanity test there is. So the guard has to test a
// property only a hand has.
//
// Checked against `node_count` nodes at `palette`:
//   * node_count == kFirstPersonNodeCount, and every index used is inside it
//   * every node we intend to touch is reasonable_palette_node()
//   * each finger chain has PLAUSIBLE BONE LENGTHS -- a palm-width to the metacarpal, then
//     three finger-bone segments. NOT "monotonically outward": these hands are authored
//     curled around a grip, so the tip comes back toward the wrist
//   * each fingertip sits a hand's reach from its wrist (kMinTipMetres..kMaxTipMetres)
//   * the two wrists are kMinWristSpanMetres..kMaxWristSpanMetres apart -- two bones that are not
//     a left and a right wrist are rarely about shoulder-width apart
//
// Returns false and touches nothing on any failure. Callers must fail closed and say so loudly:
// under this project's doctrine, disabling the arms with a clear log beats writing a hand pose
// into whatever now lives at index 19.
bool nodemap_validate(const BlamMatrix4x3* palette, std::uint32_t node_count, const NodeMap& map);

// The hardcoded table, for callers that have no map of their own.
inline bool nodemap_validate(const BlamMatrix4x3* palette, std::uint32_t node_count) {
    return nodemap_validate(palette, node_count, hardcoded_node_map());
}

// Why the last validation failed, for the log. Points at a string literal; never null.
const char* nodemap_last_failure();

// WHERE the last validation failed, when the failure has a location.
//
// The shipped build's rejection was reported as one string and nothing else, which cannot
// distinguish "one finger chain is mis-mapped" from "the wrist index is wrong so all five chains
// are" -- and those are different diseases. See docs\PALETTEARM-NODEMAP.md section A.3. This costs
// nothing on the passing path and turns the next in-headset run into a diagnosis.
struct NodeMapFailure {
    const char*  reason{"not run"};   // string literal, never null; same value as nodemap_last_failure()
    bool         has_site{false};     // true when the three fields below mean anything
    bool         right_arm{false};
    std::uint8_t chain{kNoNode};      // 0..4 -> index, middle, ring, pinky, thumb
    std::uint8_t joint{kNoNode};      // 0..3 along the chain
    std::uint8_t node{kNoNode};       // the palette index that failed
};
const NodeMapFailure& nodemap_last_failure_site();

// "index" / "middle" / "ring" / "pinky" / "thumb", or "?" for anything else. Never null.
const char* nodemap_chain_name(std::uint8_t chain);

} // namespace halo::palettearm
