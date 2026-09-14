#pragma once

// HOOK POINTS IN Plugin.cpp. Each is called from exactly one place, named here with its thread.
// Definitions: src/features/FeatureList.cpp.

#include "core/fixes/TickStage.hpp"

struct _XINPUT_STATE;

namespace halo {

// on_xinput_get_state (the XInput hook's thread), right after holster_note_buttons(): the RAW pad,
// before any of the plugin's own swallowing, rebinding and synthetic presses.
void features_xinput_raw_pad(_XINPUT_STATE* state);

// update() (game thread), right after scope_frame_end(tick).
void features_game_tick_late();

} // namespace halo
