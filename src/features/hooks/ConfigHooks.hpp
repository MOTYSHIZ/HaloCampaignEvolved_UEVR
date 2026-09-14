#pragma once

#include <cstdio>

// HOOK POINTS IN Config.cpp. Definitions: src/features/FeatureList.cpp.

#include <string>

namespace halo {

// parse_config_key_2 (the thread running load_config), right after parse_holster_key(): a key none
// of the author's parsers before it took. True = a feature took it.
//
// POSITION DOES NOT CHANGE ANY RESULT: every key a feature parses is unique among all keys, apart
// from twohandmin/twohandmax/twohandagreemin/twohandagreefull, which the author's two_hand_parse_key
// (reached from parse_melee_key, earlier in the chain) takes first, as it always did; and every
// parser earlier in the chain is side-effect free for a key it does not take.
bool features_parse_key(const char* key, const char* val, double v);

// menu_bridge_tick (game thread), in the command-file loop, where calib:height has always been
// handled: after calib:off, before calibreset:all. True = a feature handled the line (the caller
// counts it and moves to the next line). Every command string is unique, so no author command can be
// taken.
bool features_menu_command(const std::string& line);

// menu_bridge_tick (game thread), with the other status inputs, before the change check: the feature
// line for the menu status file (empty = none).
std::string features_menu_status_line();

// menu_bridge_tick, before the command file read: true = the file is absent, skip the open.
bool features_menu_command_file_absent(const char* path);
// write_calib_file, after the rig fit block: a feature's calibration lines.
void features_calib_file_write(std::FILE* f);

} // namespace halo
