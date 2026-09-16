#pragma once

#include <string>

// THE FORK'S RETIRED KEY NAMES.
//
// Every key the fork reads is named <feature stem><setting>, so the name says which feature owns it
// and a player can guess it. Where a fork key was named before that rule -- carrying no stem at all,
// or carrying one of the AUTHOR'S stems, which made it read as his -- it was renamed, and the old
// name is kept here as an accepted alias so cfg files written before the rename keep working.
//
// HOW AN ALIAS IS HANDLED: the parser accepts BOTH names, and only the new name appears in the
// catalogs, the settings menu and the feature registry. An alias is a translation, not a second key:
// the old name is rewritten to the new one before anything looks at it, so both names land in the
// same field with the same clamp, and a file that sets both simply sets the field twice in file
// order. Nothing is ever written back under an old name.
//
// WHERE IT IS APPLIED: features_parse_key (the fork's only parse entry point, which the author's
// parse_config_key_2 calls after all of his own parsers) and features_note_key (the registry's
// per-line record of which file set which key). Both are the fork's. The table holds fork names
// only -- no name here is one the author's parsers claim -- so his keys never pass through it.
//
// This list only grows when a key is renamed, and an entry is only ever removed by the author.

namespace halo {

// The current name for a retired one, or the key unchanged. The returned pointer is either `key`
// itself or a string literal, so it outlives the call either way.
const char* key_current_name(const char* key);

// The retired names, appended to the generated developer reference so the list can never drift
// from the table above.
void key_alias_append_reference(std::string& text);

} // namespace halo
