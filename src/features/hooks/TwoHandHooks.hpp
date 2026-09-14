#pragma once

// HOOK POINTS IN palettearm/TwoHand.cpp. Definitions: src/features/FeatureList.cpp.

namespace halo {

// TwoHandHold::update (game thread), the latch: true = the support hand may not acquire a NEW hold, it is
// inside a live rack zone of the reload engine; an existing hold keeps.
bool features_two_hand_support_blocked();

} // namespace halo
