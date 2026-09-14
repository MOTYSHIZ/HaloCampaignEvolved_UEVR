#pragma once

// HOOK POINTS IN Plugin.cpp. Each is called from exactly one place, named here with its thread.
// Definitions: src/features/FeatureList.cpp.

#include "core/fixes/TickStage.hpp"
#include "core/host/PluginState.hpp"   // HALO_PLUGIN_STATE_BRIDGE
#include "uevr/API.h"                  // UEVR_Vector3f

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

// update() (game thread), right after the HMD translation leash block, before the calibrate key.
void features_game_tick_after_leash();

// on_pre_calculate_stereo_view_offset (render thread), inside `position != nullptr`, right after
// g_have_view_pos is set and before aim_converge_note_pre: the body eye.
void features_stereo_pre_eye(int index, UEVR_Vector3f* position, bool is_double);

// on_post_calculate_stereo_view_offset (render thread), right after the STOMPLOG sample and before
// aim_converge_note_post: the head offset, and a feature's clamp of the rendered eye.
void features_stereo_post_eye(int index, UEVR_Vector3f* position, bool is_double);

} // namespace halo
