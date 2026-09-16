#include "core/registry/OwnedKeyDerive.hpp"

#include "Config.hpp"

#include <cstdio>

namespace halo {
namespace {

// THE VALUE THE FEATURE IS TUNED FOR, as the fork's own parser would leave it: the aimbore parse
// clamps to 0 .. 3, so 1 arrives as 1.
constexpr int kAimBore = 1;

const char* const kName[OWNED_KEY_COUNT]    = { "aimbore" };
const char* const kFeature[OWNED_KEY_COUNT] = { "palettewpn" };

// WHY THIS KEY IS THE FEATURE'S TO SET, in one line, written to survive being read by the author.
const char* const kOwned[OWNED_KEY_COUNT] = {
    "the aim along the drawn barrel, and the placement is what draws it"
};

} // namespace

OwnedKeyResolution owned_keys_derive(Config& c, const OwnedKeyLayers& set) {
    OwnedKeyResolution r{};

    // THE AIM ALONG THE BARREL IS PART OF THE PLACEMENT, not a switch beside it. armdriver=3 is the
    // arbiter's alias for the same request, as the rig and arm hide derivation already reads it. The
    // bore path is reached only from the palette's own aim branch (aim_bore_forward), so with the
    // placement off the key is dead whatever it holds.
    const bool palette = c.palette_weapon || c.arm_driver == 3;
    if (!palette)                    r.state[OWNED_AIMBORE] = OWNED_FEATURE_OFF;
    else if (set.set[OWNED_AIMBORE]) r.state[OWNED_AIMBORE] = OWNED_PLAYER_SET;
    else { c.aim_bore = kAimBore; r.state[OWNED_AIMBORE] = OWNED_DERIVED; }
    sprintf_s(r.value[OWNED_AIMBORE], sizeof(r.value[0]), "%d", c.aim_bore);

    return r;
}

const char* owned_key_name(int id) {
    return (id >= 0 && id < OWNED_KEY_COUNT) ? kName[id] : "?";
}

const char* owned_key_feature(int id) {
    return (id >= 0 && id < OWNED_KEY_COUNT) ? kFeature[id] : "?";
}

const char* owned_key_reason(int id, int state) {
    if (id < 0 || id >= OWNED_KEY_COUNT) return "?";
    switch (state) {
    case OWNED_FEATURE_OFF: return "the feature that owns it is off, so its own default stands";
    case OWNED_PLAYER_SET:  return "set in a cfg file, so your value stands";
    case OWNED_DERIVED:     return kOwned[id];
    default:                return "?";
    }
}

} // namespace halo
