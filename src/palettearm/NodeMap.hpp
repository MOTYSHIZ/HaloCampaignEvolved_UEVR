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
// Ported from elliotttate's HaloCampaignEvolved-UEVR (main.cpp @ 62ee34f) with permission.

#pragma once

#include "PaletteMath.hpp"

#include <array>
#include <cstdint>

namespace halo::palettearm {

// The engine builds exactly this many nodes for a first-person weapon. A count, not an address --
// but it is the cheapest single thing that says "this is the skeleton we measured": a different
// node count is a different rig and every index below is void.
constexpr std::uint32_t kFirstPersonNodeCount = 76;

// The three sibling trees hanging off the root: right shoulder 5, left shoulder 6, weapon 7.
// Node 0 is the camera-control root and is NEVER written -- it is the frame everything else is
// expressed in, and moving it moves the view.
constexpr std::uint8_t kRootNode        = 0;
constexpr std::uint8_t kWeaponMarkerNode = 8;                  // authored "primaryweapon" marker
constexpr std::array<std::uint8_t, 3> kWeaponNodes{7, 8, 22};

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

inline const ArmNodes& arm_nodes(bool right) { return right ? kRightArm : kLeftArm; }

// Reach limits used by the validator, in metres. Generous on purpose: this separates "a hand" from
// "not a hand", and rejecting a real hand costs the entire feature.
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
//   * each finger chain gets MONOTONICALLY farther from its wrist joint by joint -- knuckles are
//     ordered outward from the wrist, and a mis-mapped chain is not
//   * each fingertip sits a hand's reach from its wrist (kMinTipMetres..kMaxTipMetres)
//   * the two wrists are kMinWristSpanMetres..kMaxWristSpanMetres apart -- two bones that are not
//     a left and a right wrist are rarely about shoulder-width apart
//
// Returns false and touches nothing on any failure. Callers must fail closed and say so loudly:
// under this project's doctrine, disabling the arms with a clear log beats writing a hand pose
// into whatever now lives at index 19.
bool nodemap_validate(const BlamMatrix4x3* palette, std::uint32_t node_count);

// Why the last validation failed, for the log. Points at a string literal; never null.
const char* nodemap_last_failure();

} // namespace halo::palettearm
