#pragma once

#include <string>

// FEATURE REGISTRY: one row per player-facing feature, with its tier, its one master key and the
// sub-settings that only act while the master is on. The table, the tier doctrine and the
// precedence rules live in Features.cpp.

namespace halo {

// load_config: before the first file is parsed.
void features_begin_load();
// load_config: the kConfigFiles index about to be parsed, so an explicit key knows its source.
void features_set_layer(int layer);
// parse_config_file: every key line, before it is applied.
void features_note_key(const char* key, const char* val);
// load_config: after every layer is parsed. Unset masters take their tier value; sub-settings and
// shared infrastructure follow their masters.
void features_apply();

// data\halo_vr_features.txt, the feature list the settings menu draws. Written on every real
// reload; features_publish_if_missing re-creates it when data\ was cleared mid-session.
void features_publish(const char* data_dir);
void features_publish_if_missing(const char* data_dir);

// The generated developer reference, appended to the DEV panel's mirror of halo_vr_dev.cfg.
void features_append_dev_reference(std::string& text);

// Once at startup: every feature's key, running value and where the value came from.
void features_log_resolved();

} // namespace halo
