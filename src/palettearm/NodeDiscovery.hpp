// Deriving the node map from the palette, instead of remembering it from someone else's binary.
//
// WHY THIS EXISTS. NodeMap.hpp's 76 bone indices are a measurement of ONE skeleton on ONE build,
// and on the build we ship for they are WRONG: the validator rejects them at the finger-ordering
// test and the arms stay stock, which is what `armdriver=2` doing nothing actually was. That is the
// textbook rot the "HARDCODING AN ADDRESS OR STRUCT OFFSET" section of the project CLAUDE.md is
// about, and its first question is "can it be RESOLVED instead?".
//
// It can. The palette is a flat array of BlamMatrix4x3 -- scale, forward/left/up, position -- so we
// have GEOMETRY and no names. Geometry is enough, because a hand is a shape nothing else in a
// first-person rig has: five four-node chains, each marching outward from a shared node at
// finger-bone spacings with bone lengths that shorten as they go.
//
// PURE. No UEVR, no Config, no Windows, no globals -- palette in, map out, and every piece of
// working state lives in an object the caller owns. That is what lets Scripts\Verify-PaletteArm.ps1
// drive this with synthetic palettes and no game, which is the only way any of it gets tested
// before a headset session. Keep it that way.
//
// FULL DESIGN, FAILURE MODES AND THE "WHAT I COULD NOT DETERMINE" LIST:
//   docs\PALETTEARM-NODEMAP.md
//
// The cascade this participates in, top rung first:
//   1. discover_node_map()   -- derived from the palette in front of us; immune to a tag repack
//   2. hardcoded_node_map()  -- elliotttate's measured table (NodeMap.hpp)
//   3. nothing               -- arms stay stock, and we say so loudly
// Every rung is judged by the SAME nodemap_validate(). Discovery does not get an easier test.

#pragma once

#include "NodeMap.hpp"
#include "PaletteMath.hpp"

#include <array>
#include <cstdint>

namespace halo::palettearm {

// ---- WHAT DISCOVERY LOOKS FOR ------------------------------------------------------------------
//
// Anthropometric, in metres, and deliberately loose in the same spirit as the validator's reach
// limits: these separate "a hand" from "not a hand", and rejecting a real hand costs the feature.
// They are NOT measurements of a binary and do not rot when the game patches -- that is the whole
// point of the exercise.

constexpr std::size_t kFingerCount   = 5;    // index, middle, ring, pinky, thumb
constexpr std::size_t kFingerJoints  = 4;    // metacarpal + three phalanges -- FingerChain's size
constexpr std::size_t kHandNodeCount = kFingerCount * kFingerJoints;   // 20, excluding the wrist

// A palette smaller than this cannot hold two arms and a weapon; larger than this is not a
// first-person weapon rig, and the 8-bit indices could not address it anyway.
constexpr std::uint32_t kMinDiscoverableNodes = 24;
constexpr std::uint32_t kMaxDiscoverableNodes = 128;

// The chain-start edge (wrist -> metacarpal) and the in-chain edges (bone to bone). The chain edge
// is the tighter one on purpose: it is what stops "any node within a hand's radius" from being a
// plausible neighbour of any other, which is what makes the five-chain shape selective at all.
// NOTE: these are TIGHTER THAN THIS GAME'S ACTUAL RIG, and that is a known limitation, not an
// oversight. Measured on a live 76-node palette (docs\Perf\logs60824-232806-padump.txt):
//
//     wrist -> metacarpal   5.8 .. 12.0 cm      exceeds the 9.0 cm cap below
//     phalanx bones         2.2 ..  6.1 cm      exceeds the 6.0 cm cap by 1 mm
//
// So discovery cannot currently read THIS rig and reports "no hand-shaped cluster". Widening the
// two caps to 0.14 / 0.075 was tried on 2026-08-24 and BROKE 24 of the suite's discovery checks:
// the tight bounds are doing real discrimination work, and loosening them lets palm nodes pass as
// phalanges, which is exactly the mis-attribution the tie-break notes below warn about. Reverted.
//
// This is not blocking: nodemap_validate() was independently found to be rejecting the CORRECT
// hardcoded map (it required fingers to point outward, but Halo authors them curled around a
// grip), and with that fixed rung 2 resolves. Discovery stays rung 1 for patch survival. Fixing it
// properly means reworking the attribution so it does not lean on bone-length caps for
// discrimination -- do that with the dump above as the reference rig, and re-run the suite.
constexpr float kMaxMetacarpalMetres = 0.09f;
constexpr float kMaxPhalanxMetres    = 0.06f;
constexpr float kMinBoneMetres       = 0.002f;   // two coincident nodes are not a bone

// Phalanges SHORTEN as you go outward. This is the one test that knows which END of a finger is
// which, and it is what stops a fingertip from being mistaken for a wrist (from a fingertip the
// same nodes form chains that run inward, and inward-running chains get LONGER). The factor is
// permissive: a rig whose bones happen to be equal length degrades this to no information, which
// costs an ambiguous refusal rather than a wrong answer.
constexpr float kBoneGrowthTolerance = 1.4f;

// The upper arm and the forearm, for finding the shoulder and elbow above a discovered wrist.
constexpr float kMinLimbSegmentMetres = 0.15f;
constexpr float kMaxLimbSegmentMetres = 0.50f;

// A node this close to the line between two joints is part of that limb (twist and roll bones).
constexpr float kLimbRadiusMetres = 0.07f;

// Palm nodes that are in no finger chain but plainly belong to the hand. Bounded hard: this is the
// one place an unidentified node gets written, so a rig that offers a crowd of them is telling us
// we do not understand it, and we take none.
constexpr float       kPalmShellMetres = 0.05f;
constexpr std::size_t kMaxHandExtras   = 6;

// The shoulders decide which arm is which -- NOT the wrists. In a two-handed weapon hold both
// hands sit near the centreline and can even cross; shoulders never do.
constexpr float kMinSideSeparationMetres = 0.05f;

// Bounds the five-disjoint-chains search, PER CANDIDATE. Exceeding it is treated as "not a hand",
// never as a hang: this runs on the game thread inside a hook, and in VR a hitch is nausea.
//
// The number is CALIBRATED, not guessed, and the two ends of the calibration are:
//   * a real wrist decomposes in ~35 steps with the move ordering in NodeDiscovery.cpp, so this is
//     roughly 17x headroom on the case that must not be lost;
//   * what actually spends the budget is the ~40 nodes per rig that are NOT wrists and have to be
//     turned down, and each of them spends ALL of it. Cost is therefore linear in this number:
//     a 76-node two-handed palette measured ~1.3 ms at 400 and ~4.1 ms at 1600.
// A whole resolve has to fit inside a frame, which at 90 Hz is 11 ms, and it happens once per
// skeleton -- so a weapon swap pays it once. NodeMapResolver backs the retry interval off so a rig
// that will never resolve does not pay it on a loop.
//
// Exceeding it is counted and reported (DiscoveredNodeMap::budget_exhausted) rather than silently
// meaning "not a hand", because those two are otherwise the same answer.
constexpr std::uint32_t kChainSearchBudget = 600;

// The most hand candidates we will consider before declaring the rig unreadable. More than a
// couple per hand means the shape is not selective on this skeleton and we should not be guessing.
constexpr std::size_t kMaxHandCandidates = 24;

// ---- RESULTS -----------------------------------------------------------------------------------

enum class Discovery {
    Resolved,           // a map came out AND passed nodemap_validate()
    PaletteUnusable,    // null, wrong size, or a node that is not a plausible matrix
    NoHandFound,        // fewer than two hand-shaped clusters
    Ambiguous,          // more than two -- REFUSED, the same answer addrcascade gives an
                        // ambiguous signature match
    NoArmChain,         // no shoulder/elbow pair of limb-scale segments above a wrist
    SidesUnclear,       // the two arms are not on opposite sides of the root
    ValidationFailed,   // a map came out and the validator threw it back
};

// Human-readable, for the log. Never null.
const char* discovery_name(Discovery result);

// A map plus the storage its ArmNodes point into.
//
// The pointers inside `map` point into THIS object, so a copy has to be re-bound. Copying is
// allowed (it is a plain aggregate) but the copy is unusable until bind() runs -- which is exactly
// the trap the addrcascade TierReporter comment describes, so it is spelled out rather than
// prevented, and bind() is idempotent and cheap.
struct DiscoveredNodeMap {
    std::array<std::uint8_t, kMaxDiscoverableNodes> right_shoulder{};
    std::array<std::uint8_t, kMaxDiscoverableNodes> right_elbow{};
    std::array<std::uint8_t, kMaxDiscoverableNodes> right_wrist{};
    std::array<std::uint8_t, kMaxDiscoverableNodes> left_shoulder{};
    std::array<std::uint8_t, kMaxDiscoverableNodes> left_elbow{};
    std::array<std::uint8_t, kMaxDiscoverableNodes> left_wrist{};
    std::size_t right_shoulder_count{};
    std::size_t right_elbow_count{};
    std::size_t right_wrist_count{};
    std::size_t left_shoulder_count{};
    std::size_t left_elbow_count{};
    std::size_t left_wrist_count{};

    NodeMap map{};

    // How many nodes were left stock because they could not be attributed to one arm. Reported,
    // not hidden: a big number here is the shape of "the weapon got attached to a hand" and is the
    // first thing to look at if the arms move but the gun does something odd.
    std::size_t unattributed{};

    // How many candidates ran out of search budget rather than being genuinely turned down.
    //
    // These two outcomes are INDISTINGUISHABLE inside the search, and that is the one way a real
    // hand can be silently lost. Counting them is what makes the difference visible in a log
    // instead of looking like "this rig has no hands": a resolve that failed with a non-zero count
    // here should be read as "raise kChainSearchBudget and try again", not as "the rig is wrong".
    std::size_t budget_exhausted{};

    void bind();
};

// Derive a map from one palette. Does NOT write to the palette.
//
// On Discovery::Resolved, `*out` holds a bound, validated map. On anything else `*out` is left in
// an unusable state and the caller must not touch it.
//
// Cost is O(n^2 log n) over the node count with a bounded search on top -- a few tens of
// microseconds for a 76-node rig. It is still not something to run every frame; see
// NodeMapResolver, which runs it once and then re-validates cheaply.
Discovery discover_node_map(const BlamMatrix4x3* palette, std::uint32_t node_count,
                            DiscoveredNodeMap* out);

// ---- THE CASCADE, WITH A CACHE -----------------------------------------------------------------

enum class NodeMapTier {
    None,        // nothing usable -- arms stay stock
    Discovered,  // rung 1: derived from this palette
    Hardcoded,   // rung 2: elliotttate's table, which still had to pass the validator
};

const char* nodemap_tier_name(NodeMapTier tier);

// Resolve once, then validate every frame.
//
// The expensive part happens on a cache miss. Every subsequent call re-runs nodemap_validate()
// against the cached map, which is ~110 node reads and is the same guard that was there before --
// so a weapon swap to a different skeleton invalidates the cache by failing that check, and
// re-derivation happens on the next attempt rather than silently posing the wrong rig.
//
// Re-derivation is rate limited. Without that, a rig this cannot read would pay the full search on
// every single frame forever, which is precisely the kind of unbounded per-frame work the project's
// dev-tooling rule exists to keep out of a player build.
class NodeMapResolver {
public:
    NodeMapResolver() = default;

    // NOT COPYABLE, and deliberately so. The cached NodeMap borrows pointers into m_discovered,
    // which lives inside this object -- a copy would hand out an ArmNodes pointing into a dead
    // temporary, and it would only bite once a map had actually been discovered. That is the exact
    // shape of the bug addrcascade::TierReporter's comment records; here it is prevented rather
    // than documented.
    NodeMapResolver(const NodeMapResolver&) = delete;
    NodeMapResolver& operator=(const NodeMapResolver&) = delete;

    // The map to pose with, or nullptr if neither rung produced one. Never writes to the palette.
    const NodeMap* resolve(const BlamMatrix4x3* palette, std::uint32_t node_count);

    NodeMapTier tier() const { return m_tier; }

    // Why the last resolve ended where it did. Never null. For the Discovered/Hardcoded tiers this
    // is "ok"; otherwise it names the rung that failed and how.
    const char* detail() const { return m_detail; }

    // How the discovery rung ended last time it was actually attempted.
    Discovery last_discovery() const { return m_last_discovery; }

    std::uint64_t resolve_attempts() const { return m_attempts; }

    // Number of nodes BOTH arms wanted, so neither got them; 0 on the hardcoded tier. A large
    // number here is the shape of "the weapon sits between the hands", which is expected -- it is
    // a growing number across weapons that would say the attribution rules are too greedy.
    std::size_t unattributed() const;

    // Candidates the last discovery gave up on for want of search budget rather than because they
    // were not hands. Non-zero on a failed resolve means the budget, not the rig, may be the
    // problem. See DiscoveredNodeMap::budget_exhausted.
    std::size_t budget_exhausted() const { return m_discovered.budget_exhausted; }

    // Forget the cache. For a deliberate re-resolve (an armdriver flip, a config change).
    void reset();

    // Frames to wait before the FIRST re-derivation attempt after a failure, and the ceiling the
    // wait backs off to.
    //
    // A resolve costs on the order of a millisecond, which is a fifth of a 90 Hz frame. Paying that
    // at a fixed interval forever on a rig this cannot read would be a periodic hitch -- the exact
    // thing the project's dev-tooling rule exists to keep out of a player build -- so the interval
    // DOUBLES on each consecutive failure. A weapon swap is still picked up in a third of a second;
    // an unreadable rig settles into costing nothing.
    static constexpr std::uint32_t kRetryInterval    = 30;
    static constexpr std::uint32_t kMaxRetryInterval = 900;

private:
    DiscoveredNodeMap m_discovered{};
    NodeMap           m_cached{};
    NodeMapTier       m_tier{NodeMapTier::None};
    const char*       m_detail{"not run"};
    Discovery         m_last_discovery{Discovery::PaletteUnusable};
    std::uint32_t     m_cooldown{0};
    std::uint32_t     m_cooldown_span{kRetryInterval};
    std::uint64_t     m_attempts{0};
    bool              m_have_cache{false};
};

} // namespace halo::palettearm
