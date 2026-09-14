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

// update() (game thread), where the physical scope's tick has always run: after the second
// weapon_offset_update() and palettewpn's per-tick block. dt is g_last_dt, whose only writer is
// on_pre_engine_tick, before update() runs on the same thread.
void features_game_tick_after_offsets(float dt);

// update() (game thread), in the stale rig guard, right after it drops the rig and its parent.
void features_rig_lost();

} // namespace halo
