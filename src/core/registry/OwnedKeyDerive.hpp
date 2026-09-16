#pragma once

// A SETTING THAT FOLLOWS ITS FEATURE'S MASTER KEY.
//
// The owner's rule is that a feature has ONE master key -- the single switch the menu toggle writes
// -- so a setting that is not a choice the player makes separately has no switch of its own: it
// follows the master. aimbore is that case. The weapon placement puts the gun on your hand, and the
// aim along the barrel it draws is part of that, not a switch beside it, so choosing the placement
// chooses the bore.
//
// This derivation writes only the FORK's OWN keys. The author's keys are never touched: his defaults
// are what his code runs on, and every setting a fork feature reads is a fork key of its own.
//
// Any cfg layer that sets the key explicitly wins outright -- the derivation never overwrites a value
// a player chose. NOTHING IS REMEMBERED BETWEEN LOADS: load_config rebuilds the Config from its
// defaults and re-parses every file before this runs, so a feature switched off gives back exactly
// the layered values with nothing to undo here.
//
// Pure: it reads and writes one Config and which of the keys a cfg file set, and nothing else, so
// features_apply runs it on every load and a host-side test can run it on any combination.

namespace halo {

struct Config;

enum OwnedKeyId : int {
    OWNED_AIMBORE = 0,   // palettewpn -- the aim along the drawn barrel
    OWNED_KEY_COUNT
};

// Which of these keys a cfg file set this load. A set key is the player's and is never derived.
struct OwnedKeyLayers {
    bool set[OWNED_KEY_COUNT] = { false };
};

enum OwnedKeyState : int {
    OWNED_UNRESOLVED  = 0,   // before the first load
    OWNED_FEATURE_OFF = 1,   // the feature that owns the key is off, so its own default stands
    OWNED_PLAYER_SET  = 2,   // a cfg file sets the key, so the player's value stands
    OWNED_DERIVED     = 3    // the key took the value its feature is tuned for
};

struct OwnedKeyResolution {
    int  state[OWNED_KEY_COUNT] = { OWNED_UNRESOLVED };
    char value[OWNED_KEY_COUNT][48] = {};   // what the key is running with now, formatted for the log
};

// Run after every cfg layer has been parsed and the feature masters are resolved.
OwnedKeyResolution owned_keys_derive(Config& c, const OwnedKeyLayers& set);

const char* owned_key_name(int id);      // the cfg key
const char* owned_key_feature(int id);   // the fork feature that owns it
const char* owned_key_reason(int id, int state);

} // namespace halo
