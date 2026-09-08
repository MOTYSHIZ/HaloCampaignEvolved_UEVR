#include "NodeDiscovery.hpp"

#include <algorithm>
#include <cmath>

namespace halo::palettearm {
namespace {

constexpr float to_blam(float metres) { return metres / kMetresPerBlamUnit; }

// How many of the nearest nodes a hand candidate will consider. Two hands plus a forearm plus a
// weapon can all sit inside one hand's radius, so this is generous; it exists to bound the search,
// not to express anything about anatomy.
constexpr std::size_t kMaxNearNodes = 64;

float node_distance(const BlamMatrix4x3* palette, std::uint8_t a, std::uint8_t b) {
    return length(palette[a].position - palette[b].position);
}

// Distance from a point to the SEGMENT ab, not to the infinite line. Twist bones sit along a limb;
// a node beyond the shoulder is not on the upper arm however well it lines up with it.
float point_to_segment(const Vec3& p, const Vec3& a, const Vec3& b) {
    const Vec3  ab = b - a;
    const float len2 = length_squared(ab);
    if (!std::isfinite(len2) || len2 < 1.0e-12f) return length(p - a);
    float t = dot(p - a, ab) / len2;
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    return length(p - (a + ab * t));
}

struct NearNode {
    float        distance{};      // from the candidate wrist, in Blam units
    std::uint8_t index{kNoNode};
};

// One legal next bone in a finger chain, with what the search orders on.
struct Candidate {
    std::uint8_t node{kNoNode};
    float        segment{};       // its length, in Blam units
    float        alignment{};     // how well it continues the chain's heading, -1 .. 1
};

// Insertion sort. These arrays hold a handful of entries and sit in the innermost loop of a
// backtracking search that runs on the frame path; at this size std::sort's setup dominates and
// measured as most of one resolve's cost.
template <typename Less>
void small_sort(Candidate* items, std::size_t count, Less less) {
    for (std::size_t i = 1; i < count; ++i) {
        const Candidate key = items[i];
        std::size_t j = i;
        while (j > 0 && less(key, items[j - 1])) { items[j] = items[j - 1]; --j; }
        items[j] = key;
    }
}

// One hand-shaped cluster: a node from which five disjoint four-node chains march outward.
struct HandCluster {
    std::uint8_t wrist{kNoNode};
    std::uint8_t chain[kFingerCount][kFingerJoints]{};

    // HOW WIDELY THE FIVE CHAINS FAN OUT, as 1 - (smallest pairwise dot of the start directions).
    // This is what picks the real wrist out of the several nodes in one hand that can all satisfy
    // the shape test, and the choice of measure matters more than it looks:
    //
    //   * "most central" picks a metacarpal -- the palm centre is more central than the wrist;
    //   * "most proximal / longest reach" picks whatever sits FURTHEST BACK and can still reach the
    //     hand, which on a real rig is a forearm twist bone. It then adopts the true wrist as a
    //     finger joint, and every other test in this folder still passes.
    //
    // The wrist is neither the centre nor the back of a hand -- it is where the fingers DIVERGE,
    // and it sees them under a wider angle than any node in front of or behind it.
    float start_spread{};
};

// ---- THE FIVE-DISJOINT-CHAINS SEARCH -----------------------------------------------------------

struct ChainSearch {
    const BlamMatrix4x3* palette{};
    const NearNode*      near{};
    std::size_t          near_count{};
    std::uint8_t         wrist{kNoNode};
    float                max_metacarpal{};
    float                max_phalanx{};
    float                min_bone{};
    // Which way the hand points, taken from where its fingertips are. Chain STARTS are tried in
    // this direction first. Without it the starts are tried nearest-first, and a rig with helper
    // nodes just BEHIND the wrist gets them adopted as metacarpals -- they are nearer than the real
    // ones and the search does not require the chains to cover the hand, so it never has to
    // reconsider. The result passes every other test in this folder while curling palm bones.
    Vec3                 outward{};
    bool                 used[kMaxDiscoverableNodes]{};
    std::uint8_t         chain[kFingerCount][kFingerJoints]{};
    std::uint32_t        steps{};

    // Which near-slots are a bone's length from which, worked out ONCE per candidate. Rescanning
    // the whole neighbourhood at every step of the search instead measured 3 ms for one 76-node
    // resolve; this is the same answer for a tenth of the frame time.
    std::uint8_t adj[kMaxNearNodes][kMaxNearNodes]{};
    std::uint8_t adj_count[kMaxNearNodes]{};
    std::uint8_t slot_of[kMaxDiscoverableNodes]{};    // palette node -> near slot, or kNoNode

    bool over_budget() const { return steps > kChainSearchBudget; }
};

bool build_chains(ChainSearch& s, std::size_t c);

// Extend chain `c` from `depth` outward. `previous_segment` is the bone length just laid down, or
// negative at the first segment.
//
// A COMPLETED CHAIN CONTINUES INTO THE NEXT ONE from in here rather than returning to the caller,
// and that is not a stylistic choice. The five chains must be vertex-disjoint, so the search has to
// be able to reject a finished chain because of what it leaves for the chains after it -- and to do
// that it has to be able to unwind the WHOLE chain, not just its first node. Splitting the two
// halves across two functions left the used[] set corrupted on every backtrack, and the symptom was
// that a perfectly ordinary hand simply could not be decomposed.
bool extend_chain(ChainSearch& s, std::size_t c, std::size_t depth, float previous_segment) {
    if (depth >= kFingerJoints) {
        // The fingertip has to land a hand's reach from the wrist. Tested HERE, inside the search,
        // so a chain that fails it is backtracked rather than condemning the whole candidate --
        // testing it afterwards would reject a hand because of the first decomposition tried.
        const float tip = node_distance(s.palette, s.chain[c][kFingerJoints - 1], s.wrist) *
                          kMetresPerBlamUnit;
        if (tip < kMinTipMetres || tip > kMaxTipMetres) return false;
        return build_chains(s, c + 1);
    }
    if (s.over_budget()) return false;

    const std::uint8_t tip = s.chain[c][depth - 1];
    const Vec3  tip_position   = s.palette[tip].position;
    const float tip_from_wrist = node_distance(s.palette, tip, s.wrist);

    // Where this chain was already heading. At the first joint that is the direction the metacarpal
    // leaves the wrist, which is the best guess available.
    const Vec3 heading = normalized(
        depth >= 2 ? tip_position - s.palette[s.chain[c][depth - 2]].position
                   : tip_position - s.palette[s.wrist].position);

    // Collect the legal next bones, then take them MOST-ALIGNED FIRST.
    //
    // The ordering is not an optimisation, it is what makes the search finish. Adjacent metacarpal
    // bases are nearer to each other than to their own knuckles, so both "nearest node" and
    // "nearest to the wrist" send the search sideways across the palm and the disjointness
    // constraint then forces it to unwind almost everything. A finger is the chain that KEEPS
    // GOING, so continuing the heading finds the real one first and the budget is never touched.
    Candidate options[kMaxNearNodes];
    std::size_t option_count = 0;
    const std::uint8_t tip_slot = s.slot_of[tip];
    if (tip_slot == kNoNode) return false;
    for (std::size_t a = 0; a < s.adj_count[tip_slot]; ++a) {
        const std::size_t  k = s.adj[tip_slot][a];
        const std::uint8_t j = s.near[k].index;
        if (s.used[j]) continue;

        // Strictly outward from the wrist -- the same property nodemap_validate() will test, so a
        // chain that cannot be built this way is one the validator would reject anyway.
        if (!(s.near[k].distance > tip_from_wrist)) continue;

        const Vec3  step    = s.palette[j].position - tip_position;
        const float segment = length(step);
        // Bones shorten outward. This is the only test that knows which end of a finger is which.
        if (previous_segment > 0.0f && segment > previous_segment * kBoneGrowthTolerance) continue;

        options[option_count].node      = j;
        options[option_count].segment   = segment;
        options[option_count].alignment = dot(normalized(step), heading);
        ++option_count;
    }
    small_sort(options, option_count, [](const Candidate& a, const Candidate& b) {
        return a.alignment > b.alignment;
    });

    for (std::size_t k = 0; k < option_count; ++k) {
        if (++s.steps > kChainSearchBudget) return false;
        const std::uint8_t j = options[k].node;
        s.used[j] = true;
        s.chain[c][depth] = j;
        if (extend_chain(s, c, depth + 1, options[k].segment)) return true;
        s.used[j] = false;
        if (s.over_budget()) return false;
    }
    return false;
}

bool build_chains(ChainSearch& s, std::size_t c) {
    if (c >= kFingerCount) return true;
    if (s.over_budget()) return false;

    // Eligible chain starts, ORDERED OUTWARD-FIRST. See ChainSearch::outward.
    Candidate starts[kMaxNearNodes];
    std::size_t start_count = 0;
    for (std::size_t k = 0; k < s.near_count; ++k) {
        const std::uint8_t j = s.near[k].index;
        // `near` is sorted, so once the chain-start edge is too long nothing later can serve.
        if (s.near[k].distance > s.max_metacarpal) break;
        if (s.used[j]) continue;
        if (s.near[k].distance < s.min_bone) continue;
        starts[start_count].node      = j;
        starts[start_count].segment   = s.near[k].distance;
        starts[start_count].alignment =
            dot(normalized(s.palette[j].position - s.palette[s.wrist].position), s.outward);
        ++start_count;
    }
    small_sort(starts, start_count, [](const Candidate& a, const Candidate& b) {
        // TWO BUCKETS, then nearest-first inside each. Outward-or-not is the only thing the
        // direction is asked -- ranking BY how outward a node is prefers a proximal phalanx over
        // its own metacarpal, because a phalanx points straighter down the finger while a
        // metacarpal fans off to the side. Nearest-first inside the outward bucket is what puts
        // metacarpals ahead of the phalanges behind them; the backward bucket, where palm helpers
        // live, is only reached if nothing else works.
        const bool a_out = a.alignment > 0.0f;
        const bool b_out = b.alignment > 0.0f;
        if (a_out != b_out) return a_out;
        return a.segment < b.segment;
    });

    for (std::size_t k = 0; k < start_count; ++k) {
        if (++s.steps > kChainSearchBudget) return false;
        const std::uint8_t j = starts[k].node;
        s.used[j] = true;
        s.chain[c][0] = j;
        if (extend_chain(s, c, 1, -1.0f)) return true;      // which continues into chain c + 1
        s.used[j] = false;
        if (s.over_budget()) return false;
    }
    return false;
}

// Is `w` a wrist? Fills `out` on success. `*budget_hit` is set when the answer is really "we ran
// out of time deciding" -- see DiscoveredNodeMap::budget_exhausted for why that has to be visible.
bool try_hand(const BlamMatrix4x3* palette, std::uint32_t node_count, std::uint8_t w,
              HandCluster* out, bool* budget_hit) {
    NearNode near[kMaxDiscoverableNodes];
    std::size_t near_count = 0;

    const float hand_radius = to_blam(kMaxTipMetres);
    for (std::uint32_t i = 0; i < node_count; ++i) {
        if (i == w) continue;
        const float d = node_distance(palette, static_cast<std::uint8_t>(i), w);
        if (!std::isfinite(d) || d > hand_radius) continue;
        near[near_count].distance = d;
        near[near_count].index    = static_cast<std::uint8_t>(i);
        ++near_count;
    }
    if (near_count < kHandNodeCount) return false;

    // CHEAP PREFILTER, and it is not an optimisation for its own sake: without it every node in the
    // rig pays a full backtracking search before being turned down, which measured at 41 ms for a
    // 76-node palette. This runs inside a hook on the frame path, and in VR a hitch is nausea.
    // A wrist has five bones leaving it, so five nodes within a metacarpal is the floor.
    {
        const float metacarpal = to_blam(kMaxMetacarpalMetres);
        std::size_t close = 0;
        for (std::size_t k = 0; k < near_count; ++k) {
            if (near[k].distance <= metacarpal && near[k].distance >= to_blam(kMinBoneMetres)) ++close;
        }
        if (close < kFingerCount) return false;
    }

    std::sort(near, near + near_count,
              [](const NearNode& a, const NearNode& b) { return a.distance < b.distance; });
    if (near_count > kMaxNearNodes) near_count = kMaxNearNodes;

    ChainSearch search{};
    search.palette        = palette;
    search.near           = near;
    search.near_count     = near_count;
    search.wrist          = w;
    search.max_metacarpal = to_blam(kMaxMetacarpalMetres);
    search.max_phalanx    = to_blam(kMaxPhalanxMetres);
    search.min_bone       = to_blam(kMinBoneMetres);

    // Which way this hand points: the mean direction to everything sitting at fingertip range.
    // Nodes nearer than a fingertip -- palm helpers, the wrist's own neighbours -- are excluded on
    // purpose, since they are exactly what this has to be able to out-rank.
    {
        const float tip_min = to_blam(kMinTipMetres);
        const float tip_max = to_blam(kMaxTipMetres);
        Vec3 sum{};
        for (std::size_t k = 0; k < near_count; ++k) {
            if (near[k].distance < tip_min || near[k].distance > tip_max) continue;
            sum = sum + normalized(palette[near[k].index].position - palette[w].position);
        }
        search.outward = normalized(sum);   // zero if it cancels out, which orders starts by
                                            // distance alone -- the behaviour before this existed
    }

    for (std::size_t i = 0; i < kMaxDiscoverableNodes; ++i) search.slot_of[i] = kNoNode;
    for (std::size_t k = 0; k < near_count; ++k) {
        search.slot_of[near[k].index] = static_cast<std::uint8_t>(k);
    }
    // Which near-slots could be joined by a bone. O(near^2) once, instead of O(near) on every step
    // of a backtracking search.
    for (std::size_t k = 0; k < near_count; ++k) {
        std::uint8_t count = 0;
        for (std::size_t o = 0; o < near_count; ++o) {
            if (o == k) continue;
            const float segment = node_distance(palette, near[k].index, near[o].index);
            if (!std::isfinite(segment) || segment < search.min_bone ||
                segment > search.max_phalanx) {
                continue;
            }
            search.adj[k][count++] = static_cast<std::uint8_t>(o);
        }
        search.adj_count[k] = count;
    }

    const bool decomposed = build_chains(search, 0);
    if (search.over_budget()) *budget_hit = true;
    if (!decomposed) return false;

    // Every fingertip is already inside the reach window -- extend_chain enforced it as each chain
    // closed. What is left to measure is how widely the five chains fan out from here, which is the
    // tie-break between the several nodes of one hand that can all pass the shape test.
    Vec3 start_direction[kFingerCount];
    for (std::size_t c = 0; c < kFingerCount; ++c) {
        start_direction[c] =
            normalized(palette[search.chain[c][0]].position - palette[w].position);
    }
    // ALL FIVE DIGITS LEAVE A WRIST THE SAME WAY. Not within any tuned angle -- simply into the
    // same half-space as their own mean direction. This is what separates the wrist from a node
    // INSIDE the hand, which can also decompose into five outward chains but sends some of them
    // backwards; without it the widest-fan tie-break below prefers exactly those nodes, because
    // two chains pointing opposite ways is the widest fan there is.
    Vec3 mean{};
    for (std::size_t c = 0; c < kFingerCount; ++c) mean = mean + start_direction[c];
    mean = normalized(mean);
    if (length_squared(mean) < 0.8f) return false;         // they cancel out: not a wrist
    for (std::size_t c = 0; c < kFingerCount; ++c) {
        if (!(dot(start_direction[c], mean) > 0.0f)) return false;
    }

    float narrowest = 1.0f;
    for (std::size_t a = 0; a < kFingerCount; ++a) {
        for (std::size_t b = a + 1; b < kFingerCount; ++b) {
            const float agreement = dot(start_direction[a], start_direction[b]);
            if (agreement < narrowest) narrowest = agreement;
        }
    }

    out->wrist        = w;
    out->start_spread = 1.0f - narrowest;
    for (std::size_t c = 0; c < kFingerCount; ++c) {
        for (std::size_t k = 0; k < kFingerJoints; ++k) out->chain[c][k] = search.chain[c][k];
    }
    return true;
}

// ---- NAMING THE FINGERS ------------------------------------------------------------------------
//
// Geometry cannot tell us a bone's authored name, but it can tell us which digit is the odd one
// out: the thumb's metacarpal sits away from the other four, which are in a row. Then the four are
// ordered by how near the thumb they are -- index, middle, ring, pinky.
//
// Getting this wrong is COSMETIC and bounded: it decides which chain a trigger curls, not where the
// hand goes. Finger curl inputs are not wired yet (see PaletteArm.cpp), so today it decides
// nothing at all -- but it will, and an arbitrary order would be a silent trap when it does.
void order_fingers(const BlamMatrix4x3* palette, HandCluster& hand, std::size_t order[kFingerCount]) {
    std::size_t thumb = 0;
    float worst = -1.0f;
    for (std::size_t c = 0; c < kFingerCount; ++c) {
        float spread = 0.0f;
        for (std::size_t o = 0; o < kFingerCount; ++o) {
            if (o == c) continue;
            spread += node_distance(palette, hand.chain[c][0], hand.chain[o][0]);
        }
        if (spread > worst) { worst = spread; thumb = c; }
    }

    std::size_t rest[kFingerCount - 1];
    std::size_t rest_count = 0;
    for (std::size_t c = 0; c < kFingerCount; ++c) {
        if (c != thumb) rest[rest_count++] = c;
    }
    std::sort(rest, rest + rest_count, [&](std::size_t a, std::size_t b) {
        return node_distance(palette, hand.chain[a][0], hand.chain[thumb][0]) <
               node_distance(palette, hand.chain[b][0], hand.chain[thumb][0]);
    });

    order[0] = rest[0];   // index
    order[1] = rest[1];   // middle
    order[2] = rest[2];   // ring
    order[3] = rest[3];   // pinky
    order[4] = thumb;
}

// ---- SHOULDER AND ELBOW ------------------------------------------------------------------------
//
// The longest two-segment path above the wrist whose segments are both limb-scale, and which stays
// on this arm's side of the rig.
bool find_arm_chain(const BlamMatrix4x3* palette, std::uint32_t node_count, std::uint8_t root,
                    std::uint8_t wrist, std::uint8_t other_wrist,
                    const bool* in_a_hand, std::uint8_t* shoulder, std::uint8_t* elbow) {
    const float min_seg = to_blam(kMinLimbSegmentMetres);
    const float max_seg = to_blam(kMaxLimbSegmentMetres);

    float best = -1.0f;
    for (std::uint32_t e = 0; e < node_count; ++e) {
        const std::uint8_t E = static_cast<std::uint8_t>(e);
        if (E == root || in_a_hand[E]) continue;
        const float forearm = node_distance(palette, E, wrist);
        if (!std::isfinite(forearm) || forearm < min_seg || forearm > max_seg) continue;
        // Side coherence: an elbow belongs to the hand it is nearer to. Without this the search
        // will happily reach across the body for a longer total.
        if (node_distance(palette, E, other_wrist) < forearm) continue;

        for (std::uint32_t s = 0; s < node_count; ++s) {
            const std::uint8_t S = static_cast<std::uint8_t>(s);
            if (S == root || S == E || in_a_hand[S]) continue;
            const float upper = node_distance(palette, S, E);
            if (!std::isfinite(upper) || upper < min_seg || upper > max_seg) continue;
            if (node_distance(palette, S, other_wrist) < node_distance(palette, S, wrist)) continue;
            // The elbow is BETWEEN the shoulder and the hand. Without this the search happily
            // returns the pair inverted -- shoulder read as elbow -- because the inverted pair
            // measures LONGER, and then the IK bends the arm at the shoulder.
            if (node_distance(palette, S, wrist) <= forearm) continue;

            const float total = forearm + upper;
            if (total > best) { best = total; *elbow = E; *shoulder = S; }
        }
    }
    return best > 0.0f;
}

// ---- ASSEMBLY ----------------------------------------------------------------------------------

void push_unique(std::array<std::uint8_t, kMaxDiscoverableNodes>& list, std::size_t& count,
                 std::uint8_t node) {
    for (std::size_t i = 0; i < count; ++i) {
        if (list[i] == node) return;
    }
    if (count < list.size()) list[count++] = node;
}

} // namespace

// ---- PUBLIC ------------------------------------------------------------------------------------

const char* discovery_name(Discovery result) {
    switch (result) {
        case Discovery::Resolved:         return "resolved";
        case Discovery::PaletteUnusable:  return "palette unusable";
        case Discovery::NoHandFound:      return "no hand-shaped cluster";
        case Discovery::Ambiguous:        return "more than two hands -- refused";
        case Discovery::NoArmChain:       return "no shoulder/elbow above a wrist";
        case Discovery::SidesUnclear:     return "left and right are not separable";
        case Discovery::ValidationFailed: return "derived map failed validation";
    }
    return "?";
}

const char* nodemap_tier_name(NodeMapTier tier) {
    switch (tier) {
        case NodeMapTier::None:       return "none";
        case NodeMapTier::Discovered: return "discovered";
        case NodeMapTier::Hardcoded:  return "hardcoded";
    }
    return "?";
}

void DiscoveredNodeMap::bind() {
    map.right.shoulder_subtree = right_shoulder.data();
    map.right.shoulder_count   = right_shoulder_count;
    map.right.elbow_subtree    = right_elbow.data();
    map.right.elbow_count      = right_elbow_count;
    map.right.wrist_subtree    = right_wrist.data();
    map.right.wrist_count      = right_wrist_count;
    map.left.shoulder_subtree  = left_shoulder.data();
    map.left.shoulder_count    = left_shoulder_count;
    map.left.elbow_subtree     = left_elbow.data();
    map.left.elbow_count       = left_elbow_count;
    map.left.wrist_subtree     = left_wrist.data();
    map.left.wrist_count       = left_wrist_count;
}

Discovery discover_node_map(const BlamMatrix4x3* palette, std::uint32_t node_count,
                            DiscoveredNodeMap* out) {
    if (out == nullptr) return Discovery::PaletteUnusable;
    *out = DiscoveredNodeMap{};

    if (palette == nullptr) return Discovery::PaletteUnusable;
    if (node_count < kMinDiscoverableNodes || node_count > kMaxDiscoverableNodes) {
        return Discovery::PaletteUnusable;
    }
    for (std::uint32_t i = 0; i < node_count; ++i) {
        if (!reasonable_palette_node(palette[i])) return Discovery::PaletteUnusable;
    }

    // ---- 1. every node that could be a wrist ---------------------------------------------------
    HandCluster candidates[kMaxHandCandidates];
    std::size_t candidate_count = 0;
    bool budget_hit = false;
    for (std::uint32_t i = 0; i < node_count; ++i) {
        HandCluster hand{};
        if (!try_hand(palette, node_count, static_cast<std::uint8_t>(i), &hand, &budget_hit)) {
            if (budget_hit) { ++out->budget_exhausted; budget_hit = false; }
            continue;
        }
        budget_hit = false;
        if (candidate_count >= kMaxHandCandidates) return Discovery::Ambiguous;
        candidates[candidate_count++] = hand;
    }
    if (candidate_count < 2) return Discovery::NoHandFound;

    // ---- 2. candidates that claim the same nodes are the same hand ------------------------------
    //
    // Several nodes of one hand can pass the shape test -- a metacarpal sees the rest of the hand
    // laid out outward from it too. They overlap, so they collapse into one cluster, and the
    // survivor is the most PROXIMAL of them (largest total fingertip reach).
    bool claims[kMaxHandCandidates][kMaxDiscoverableNodes]{};
    for (std::size_t c = 0; c < candidate_count; ++c) {
        claims[c][candidates[c].wrist] = true;
        for (std::size_t f = 0; f < kFingerCount; ++f) {
            for (std::size_t k = 0; k < kFingerJoints; ++k) claims[c][candidates[c].chain[f][k]] = true;
        }
    }

    std::size_t group[kMaxHandCandidates];
    for (std::size_t c = 0; c < candidate_count; ++c) group[c] = c;
    const auto find_group = [&](std::size_t c) {
        while (group[c] != c) c = group[c];
        return c;
    };
    for (std::size_t a = 0; a < candidate_count; ++a) {
        for (std::size_t b = a + 1; b < candidate_count; ++b) {
            bool overlap = false;
            for (std::uint32_t n = 0; n < node_count && !overlap; ++n) {
                overlap = claims[a][n] && claims[b][n];
            }
            if (!overlap) continue;
            const std::size_t ra = find_group(a);
            const std::size_t rb = find_group(b);
            if (ra != rb) group[rb] = ra;
        }
    }

    std::size_t best_of_group[kMaxHandCandidates];
    std::size_t group_count = 0;
    for (std::size_t c = 0; c < candidate_count; ++c) {
        const std::size_t root_of = find_group(c);
        std::size_t slot = kMaxHandCandidates;
        for (std::size_t g = 0; g < group_count; ++g) {
            if (find_group(best_of_group[g]) == root_of) { slot = g; break; }
        }
        if (slot == kMaxHandCandidates) {
            best_of_group[group_count++] = c;
        } else if (candidates[c].start_spread > candidates[best_of_group[slot]].start_spread) {
            best_of_group[slot] = c;
        }
    }

    if (group_count < 2) return Discovery::NoHandFound;
    // MORE than two hands is a refusal, not a guess. Same doctrine as an ambiguous signature match
    // in src\addrcascade: a second plausible answer means we do not have one.
    if (group_count > 2) return Discovery::Ambiguous;

    HandCluster hand_a = candidates[best_of_group[0]];
    HandCluster hand_b = candidates[best_of_group[1]];
    if (hand_a.wrist == hand_b.wrist) return Discovery::NoHandFound;

    bool in_a_hand[kMaxDiscoverableNodes]{};
    const auto mark_hand = [&](const HandCluster& hand) {
        in_a_hand[hand.wrist] = true;
        for (std::size_t f = 0; f < kFingerCount; ++f) {
            for (std::size_t k = 0; k < kFingerJoints; ++k) in_a_hand[hand.chain[f][k]] = true;
        }
    };
    mark_hand(hand_a);
    mark_hand(hand_b);

    // ---- 3. the arm above each wrist ------------------------------------------------------------
    const std::uint8_t root = kRootNode;
    if (static_cast<std::uint32_t>(root) >= node_count) return Discovery::PaletteUnusable;

    std::uint8_t shoulder_a = kNoNode, elbow_a = kNoNode;
    std::uint8_t shoulder_b = kNoNode, elbow_b = kNoNode;
    if (!find_arm_chain(palette, node_count, root, hand_a.wrist, hand_b.wrist, in_a_hand,
                        &shoulder_a, &elbow_a) ||
        !find_arm_chain(palette, node_count, root, hand_b.wrist, hand_a.wrist, in_a_hand,
                        &shoulder_b, &elbow_b)) {
        return Discovery::NoArmChain;
    }
    if (shoulder_a == shoulder_b || elbow_a == elbow_b || shoulder_a == elbow_b ||
        shoulder_b == elbow_a) {
        return Discovery::NoArmChain;
    }

    // ---- 4. which one is the left ---------------------------------------------------------------
    //
    // From the SHOULDERS, never the wrists: in a two-handed hold both hands sit on the centreline
    // and can cross, and a mirrored map would pass every other test in this file.
    const Mat3 root_basis = orthonormal_basis(palette[root]);
    if (!valid_basis(root_basis)) return Discovery::SidesUnclear;
    const Vec3 root_position = palette[root].position;
    const float lateral_a = dot(palette[shoulder_a].position - root_position, root_basis.left);
    const float lateral_b = dot(palette[shoulder_b].position - root_position, root_basis.left);
    if (!std::isfinite(lateral_a) || !std::isfinite(lateral_b)) return Discovery::SidesUnclear;
    if (std::fabs(lateral_a - lateral_b) < to_blam(kMinSideSeparationMetres)) {
        return Discovery::SidesUnclear;
    }
    // Blam's second basis axis is LEFT, so more-left is the left arm.
    const bool a_is_left = lateral_a > lateral_b;

    HandCluster& left_hand  = a_is_left ? hand_a : hand_b;
    HandCluster& right_hand = a_is_left ? hand_b : hand_a;
    const std::uint8_t left_shoulder_node  = a_is_left ? shoulder_a : shoulder_b;
    const std::uint8_t left_elbow_node     = a_is_left ? elbow_a    : elbow_b;
    const std::uint8_t right_shoulder_node = a_is_left ? shoulder_b : shoulder_a;
    const std::uint8_t right_elbow_node    = a_is_left ? elbow_b    : elbow_a;

    // ---- 5. who owns every other node -----------------------------------------------------------
    //
    // Whitelist, not "everything below the shoulder": a node we cannot attribute to exactly one arm
    // is LEFT STOCK. That is what keeps the weapon -- which sits between the two hands and would
    // qualify for both -- from being dragged along by the aim arm and double-transformed.
    // Unclaimed nodes look slightly wrong; a double-driven weapon points where the shots do not go.
    // Six bits, three per arm: palm / forearm / upper arm. Three and not one, because a node on the
    // upper arm must ride the SHOULDER subtree only -- put it in the elbow subtree and the elbow
    // rotation drags the bicep with it.
    constexpr std::uint8_t kRightPalm = 0x01, kRightFore = 0x02, kRightUpper = 0x04;
    constexpr std::uint8_t kLeftPalm  = 0x08, kLeftFore  = 0x10, kLeftUpper  = 0x20;
    constexpr std::uint8_t kRightAny  = kRightPalm | kRightFore | kRightUpper;
    constexpr std::uint8_t kLeftAny   = kLeftPalm  | kLeftFore  | kLeftUpper;

    std::uint8_t claim[kMaxDiscoverableNodes]{};
    const auto claim_arm = [&](std::uint8_t node, std::uint8_t bit) {
        if (in_a_hand[node] || node == root) return;
        claim[node] = static_cast<std::uint8_t>(claim[node] | bit);
    };

    struct ArmPick {
        std::uint8_t palm_bit;
        std::uint8_t fore_bit;
        std::uint8_t upper_bit;
        std::uint8_t wrist;
        std::uint8_t elbow;
        std::uint8_t shoulder;
        std::uint8_t other_wrist;
    };
    const ArmPick picks[2] = {
        {kRightPalm, kRightFore, kRightUpper, right_hand.wrist, right_elbow_node,
         right_shoulder_node, left_hand.wrist},
        {kLeftPalm,  kLeftFore,  kLeftUpper,  left_hand.wrist,  left_elbow_node,
         left_shoulder_node,  right_hand.wrist},
    };

    const float palm_shell  = to_blam(kPalmShellMetres);
    const float limb_radius = to_blam(kLimbRadiusMetres);
    for (const ArmPick& pick : picks) {
        // Palm nodes: in no finger chain, but plainly inside this hand and not the other one.
        NearNode extras[kMaxDiscoverableNodes];
        std::size_t extra_count = 0;
        for (std::uint32_t i = 0; i < node_count; ++i) {
            const std::uint8_t n = static_cast<std::uint8_t>(i);
            if (in_a_hand[n] || n == root || n == pick.elbow || n == pick.shoulder) continue;
            const float d = node_distance(palette, n, pick.wrist);
            if (!std::isfinite(d) || d > palm_shell) continue;
            if (node_distance(palette, n, pick.other_wrist) < d) continue;
            extras[extra_count].distance = d;
            extras[extra_count].index    = n;
            ++extra_count;
        }
        std::sort(extras, extras + extra_count,
                  [](const NearNode& a, const NearNode& b) { return a.distance < b.distance; });
        // A crowd of unnamed nodes inside a hand means we do not understand this rig. Take none
        // rather than the nearest few -- guessing is what this whole lane exists to stop.
        if (extra_count <= kMaxHandExtras) {
            for (std::size_t k = 0; k < extra_count; ++k) claim_arm(extras[k].index, pick.palm_bit);
        }

        // Twist and roll bones along the two limb segments. Forearm wins a tie: the lower segment
        // is the one whose subtree is nested inside the other, so mis-filing outward is safe and
        // mis-filing inward is not.
        const Vec3 wrist_p    = palette[pick.wrist].position;
        const Vec3 elbow_p    = palette[pick.elbow].position;
        const Vec3 shoulder_p = palette[pick.shoulder].position;
        for (std::uint32_t i = 0; i < node_count; ++i) {
            const std::uint8_t n = static_cast<std::uint8_t>(i);
            if (in_a_hand[n] || n == root || n == pick.elbow || n == pick.shoulder) continue;
            if (claim[n] & pick.palm_bit) continue;
            const Vec3& p = palette[n].position;
            if (point_to_segment(p, elbow_p, wrist_p) <= limb_radius) {
                claim_arm(n, pick.fore_bit);
            } else if (point_to_segment(p, shoulder_p, elbow_p) <= limb_radius) {
                claim_arm(n, pick.upper_bit);
            }
        }
    }

    // A node that both arms want is a node we cannot attribute -- most often the weapon, which sits
    // between the hands. It is left STOCK rather than assigned to a guess.
    std::size_t ambiguous_nodes = 0;
    for (std::uint32_t i = 0; i < node_count; ++i) {
        if ((claim[i] & kRightAny) != 0 && (claim[i] & kLeftAny) != 0) {
            claim[i] = 0;
            ++ambiguous_nodes;
        }
    }

    // ---- 6. assemble ----------------------------------------------------------------------------
    //
    // The three subtrees are NESTED by construction -- wrist ⊂ elbow ⊂ shoulder -- which is a
    // property ArmSolve.cpp relies on: rotating about the shoulder has to carry the elbow and the
    // wrist sets with it.
    std::size_t order[kFingerCount];

    struct ArmStore {
        std::array<std::uint8_t, kMaxDiscoverableNodes>* wrist_list;
        std::size_t* wrist_count;
        std::array<std::uint8_t, kMaxDiscoverableNodes>* elbow_list;
        std::size_t* elbow_count;
        std::array<std::uint8_t, kMaxDiscoverableNodes>* shoulder_list;
        std::size_t* shoulder_count;
    };

    const auto fill_arm = [&](const HandCluster& hand, const ArmPick& pick, const ArmStore& store,
                              ArmNodes& arm) {
        *store.wrist_count = *store.elbow_count = *store.shoulder_count = 0;

        push_unique(*store.wrist_list, *store.wrist_count, hand.wrist);
        for (std::size_t f = 0; f < kFingerCount; ++f) {
            for (std::size_t k = 0; k < kFingerJoints; ++k) {
                push_unique(*store.wrist_list, *store.wrist_count, hand.chain[f][k]);
            }
        }
        for (std::uint32_t i = 0; i < node_count; ++i) {
            const std::uint8_t n = static_cast<std::uint8_t>(i);
            if (claim[n] & pick.palm_bit) push_unique(*store.wrist_list, *store.wrist_count, n);
        }

        for (std::size_t i = 0; i < *store.wrist_count; ++i) {
            push_unique(*store.elbow_list, *store.elbow_count, (*store.wrist_list)[i]);
        }
        push_unique(*store.elbow_list, *store.elbow_count, pick.elbow);
        for (std::uint32_t i = 0; i < node_count; ++i) {
            const std::uint8_t n = static_cast<std::uint8_t>(i);
            if (claim[n] & pick.fore_bit) push_unique(*store.elbow_list, *store.elbow_count, n);
        }

        for (std::size_t i = 0; i < *store.elbow_count; ++i) {
            push_unique(*store.shoulder_list, *store.shoulder_count, (*store.elbow_list)[i]);
        }
        push_unique(*store.shoulder_list, *store.shoulder_count, pick.shoulder);
        for (std::uint32_t i = 0; i < node_count; ++i) {
            const std::uint8_t n = static_cast<std::uint8_t>(i);
            if (claim[n] & pick.upper_bit) {
                push_unique(*store.shoulder_list, *store.shoulder_count, n);
            }
        }

        arm.shoulder = pick.shoulder;
        arm.elbow    = pick.elbow;
        arm.wrist    = hand.wrist;
    };

    // Fingers get named before the arms are filled: order_fingers() only permutes which chain is
    // called which, and every chain is written either way.
    HandCluster ordered_right = right_hand;
    order_fingers(palette, right_hand, order);
    for (std::size_t f = 0; f < kFingerCount; ++f) {
        for (std::size_t k = 0; k < kFingerJoints; ++k) {
            ordered_right.chain[f][k] = right_hand.chain[order[f]][k];
        }
    }
    HandCluster ordered_left = left_hand;
    order_fingers(palette, left_hand, order);
    for (std::size_t f = 0; f < kFingerCount; ++f) {
        for (std::size_t k = 0; k < kFingerJoints; ++k) {
            ordered_left.chain[f][k] = left_hand.chain[order[f]][k];
        }
    }

    const auto set_chains = [](ArmNodes& arm, const HandCluster& hand) {
        const auto copy = [&](FingerChain& dst, std::size_t f) {
            for (std::size_t k = 0; k < kFingerJoints; ++k) dst[k] = hand.chain[f][k];
        };
        copy(arm.index, 0);
        copy(arm.middle, 1);
        copy(arm.ring, 2);
        copy(arm.pinky, 3);
        copy(arm.thumb, 4);
    };

    const ArmStore right_store{&out->right_wrist,    &out->right_wrist_count,
                               &out->right_elbow,    &out->right_elbow_count,
                               &out->right_shoulder, &out->right_shoulder_count};
    const ArmStore left_store {&out->left_wrist,     &out->left_wrist_count,
                               &out->left_elbow,     &out->left_elbow_count,
                               &out->left_shoulder,  &out->left_shoulder_count};

    fill_arm(ordered_right, picks[0], right_store, out->map.right);
    set_chains(out->map.right, ordered_right);

    fill_arm(ordered_left, picks[1], left_store, out->map.left);
    set_chains(out->map.left, ordered_left);

    out->unattributed = ambiguous_nodes;

    out->map.node_count    = node_count;
    out->map.root          = root;
    out->map.weapon_marker = kNoNode;    // geometry cannot name a marker; nothing here writes one
    out->map.weapon_nodes  = nullptr;
    out->map.weapon_count  = 0;
    out->bind();

    // THE SAME GUARD, NOT A SOFTER ONE. If the derived map cannot pass the test elliotttate's
    // table is held to, it does not get used either.
    if (!nodemap_validate(palette, node_count, out->map)) {
        const std::size_t exhausted = out->budget_exhausted;   // survives the wipe; it is a report
        *out = DiscoveredNodeMap{};
        out->budget_exhausted = exhausted;
        return Discovery::ValidationFailed;
    }
    return Discovery::Resolved;
}

// ---- THE RESOLVER ------------------------------------------------------------------------------

const NodeMap* NodeMapResolver::resolve(const BlamMatrix4x3* palette, std::uint32_t node_count) {
    // The cheap path, and the one that runs on nearly every frame: re-prove the cached map against
    // the palette in front of us. A weapon swap to a different skeleton fails here, which is what
    // drops the cache instead of posing the wrong rig.
    if (m_have_cache && nodemap_validate(palette, node_count, m_cached)) {
        m_detail = "ok";
        return &m_cached;
    }

    m_have_cache = false;
    m_tier       = NodeMapTier::None;

    if (m_cooldown > 0) {
        --m_cooldown;
        m_detail = "waiting to retry";
        return nullptr;
    }
    m_cooldown = m_cooldown_span;
    // Back off. A resolve is on the order of a millisecond; repeating it at a fixed interval on a
    // rig that will never resolve is a hitch every few hundred milliseconds forever.
    m_cooldown_span = m_cooldown_span >= kMaxRetryInterval ? kMaxRetryInterval
                                                           : m_cooldown_span * 2;
    ++m_attempts;

    m_last_discovery = discover_node_map(palette, node_count, &m_discovered);
    if (m_last_discovery == Discovery::Resolved) {
        m_discovered.bind();               // the copy below borrows these pointers
        m_cached        = m_discovered.map;
        m_tier          = NodeMapTier::Discovered;
        m_have_cache    = true;
        m_detail        = "ok";
        m_cooldown      = 0;
        m_cooldown_span = kRetryInterval;
        return &m_cached;
    }

    if (nodemap_validate(palette, node_count, hardcoded_node_map())) {
        m_cached        = hardcoded_node_map();
        m_tier          = NodeMapTier::Hardcoded;
        m_have_cache    = true;
        m_detail        = "ok";
        m_cooldown      = 0;
        m_cooldown_span = kRetryInterval;
        return &m_cached;
    }

    // Neither rung. Fail closed: the caller leaves the stock pose alone and says so.
    m_detail = nodemap_last_failure();
    return nullptr;
}

std::size_t NodeMapResolver::unattributed() const {
    return m_tier == NodeMapTier::Discovered ? m_discovered.unattributed : 0;
}

void NodeMapResolver::reset() {
    m_have_cache    = false;
    m_tier          = NodeMapTier::None;
    m_detail        = "not run";
    m_cooldown      = 0;
    m_cooldown_span = kRetryInterval;
}

} // namespace halo::palettearm
