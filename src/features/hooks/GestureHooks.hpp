#pragma once

// HOOK POINTS IN Gesture.cpp. Definitions: src/features/FeatureList.cpp.

#include "core/host/GestureState.hpp"   // HALO_GESTURE_STATE_BRIDGE

namespace halo {

// gesture_update (game thread), near its top, after the reload family's ticks and before the global
// stand-down: the deferred melee hold check (core/fixes/MeleeInstruments).
void features_melee_hold_check();

// gesture_update (game thread), in the aim hand's swing segmentation, the statement right before
// `s_in_swing = true;`, with the value s_in_swing has then: the swing-start aim capture (core).
void features_melee_swing_moving(bool in_swing);

// gesture_update (game thread), right after the aim hand's trigger gates (the legacy forward gate
// last). True = the holster veto stood the strike down and armed the short cooldown; the tick
// returns (core).
bool features_melee_vetoed(long long now, float speed, float reach);

// gesture_update (game thread), right after a fired strike arms the cooldown: the aim pin, the hold
// check schedule and the FIRED log line (core). True = the line is logged there, so the tick returns
// before the author's own line.
bool features_melee_fired(long long now, float speed, float reach);

// ---- THE RELOAD ENGINE (core/reload/ReloadEngine.cpp), all game thread unless noted.
// gesture_update, first statement: the engine's restore windows and, while it runs, its ticks.
void features_gesture_tick_begin(float dt);
// set_state, right before the state is assigned: the state being left and the next.
void features_reload_state_set(ReloadState prev, ReloadState next);
// reload_fire_suppressed (any thread), first statement: -1 = the author's rule decides, else 0 or 1.
int  features_reload_fire_suppressed();
// reload_update: in the disabled block before the state is forced Idle; right after that block.
void features_reload_disabled();
void features_reload_update_begin();
// reload_update, the watchdog, right before the timed-out state change.
void features_reload_timed_out();
// reload_update, right after the pad buttons are read: the weapon swap cancel and restore.
void features_reload_buttons_read();
// reload_update, the grip: true = the fetch hand's grip action is held (the XInput mask still counts).
bool features_reload_grip_held();
// reload_update, the fetch hand: the author's get_pose result in, whether the hand counts as present.
bool features_reload_fetch_pose(bool pose_ok, Vec3* hand_l, const Vec3* head);
// reload_update, Idle: true = this weapon's press is ignored; accepted = right before MAG_OUT.
bool features_reload_press_ignored();
void features_reload_press_accepted();
// reload_update, MAG_OUT: the author's belt test in, whether the grab zone holds; grabbed = before MAG_HELD.
bool features_reload_belt_grab_ok(bool belt);
void features_reload_grabbed(float hand_y);
// reload_update, MAG_HELD after the grip check: true = the engine ran the well and the seat (the case breaks).
bool features_reload_seat(bool have_left, const Vec3& hand_l, const Vec3* hand_r, const Vec3* head);
// gesture_reset, first statement: the engine's reset, the two-handed hold (stabilityfixes), feature slots.
void features_gesture_reset();
// gesture_update, right after reload_update: the engine's ticks.
void features_reload_ticks(bool poses_ok, const Vec3& hpos);

} // namespace halo
