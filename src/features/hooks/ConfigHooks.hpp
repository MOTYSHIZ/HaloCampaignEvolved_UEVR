#pragma once

// HOOK POINTS IN Config.cpp. Definitions: src/features/FeatureList.cpp.

namespace halo {

// parse_config_key_2 (the thread running load_config), right after parse_holster_key(): a key none
// of the author's parsers before it took. True = a feature took it.
//
// POSITION DOES NOT CHANGE ANY RESULT: every key a feature parses is unique among all keys, apart
// from twohandmin/twohandmax/twohandagreemin/twohandagreefull, which the author's two_hand_parse_key
// (reached from parse_melee_key, earlier in the chain) takes first, as it always did; and every
// parser earlier in the chain is side-effect free for a key it does not take.
bool features_parse_key(const char* key, const char* val, double v);

} // namespace halo
