// Motion gestures -- swing to melee, and (later) the two-stage reload.
//
// THE SPLIT THIS MODULE EXISTS TO ENFORCE. Detection needs poses, config and logging; injection
// needs to happen inside the XInput hook, which this codebase keeps free of reflection, allocation
// and logging (see the note above on_xinput_get_state in Plugin.cpp). So detection runs on the game
// thread at tick rate and publishes ONE atomic deadline, and the hook does nothing but compare a
// clock against it. Same produce-at-tick / consume-at-poll shape as g_move_rot_deg.
//
// Everything here reads poses through get_pose(), which returns METRES -- see xdist_m in Config,
// which is documented in metres and added directly to a pose position in MotionAimControl.cpp.

#pragma once

// API.hpp, NOT Plugin.hpp -- Plugin.hpp defines the plugin entry points and may only be included
// by Plugin.cpp. See UeObject.hpp.
#include "uevr/API.hpp"

#include <atomic>
#include <cstdint>

namespace halo {

// Deadline, in steady_clock::time_point::duration ticks since epoch, until which the melee mask is
// held down. Zero means "not pressing". Written by the tick, read by the XInput hook.
//
// A DEADLINE rather than a bool because the hook must not own state: it can be called at any rate,
// can be skipped entirely while the game is not polling, and must never be the thing responsible
// for releasing a button. Time is the one shared reference both threads already agree on.
extern std::atomic<long long> g_melee_hold_until;

// TRUE while the synthetic melee press should be asserted. Safe from any thread; does not allocate.
// This is the whole surface the XInput hook consumes.
bool melee_press_active();

// ---- VR RELOAD ---------------------------------------------------------------------------------
//
// Halo's reload is ONE atomic animation on ONE button -- there is no eject-and-wait state to hook,
// and no magazine object to manipulate. So the physicality cannot come from the game; it has to
// come from DEFERRING the game's own reload until the player has performed the motion:
//
//   IDLE      --  reload pressed        -->  MAG_OUT    fire suppressed, game NOT told to reload
//   MAG_OUT   --  left hand at belt + grip -> MAG_HELD
//   MAG_OUT   --  reload pressed again  -->  IDLE       deliberate cancel, mag re-seats
//   MAG_HELD  --  grip released         -->  MAG_OUT    you dropped it
//   MAG_HELD  --  left hand to weapon   -->  fires the real reload button, back to IDLE
//
// What sells it is not an animation -- it is step one taking the gun away. You press reload, you
// genuinely cannot shoot, and you have to do something physical about it.
//
// NO TIMEOUT, by request: it holds indefinitely. But indefinitely must not mean inescapable, so
// every transition the player did NOT choose (kill switch, stick mode, calibration, losing the
// pose) resets to IDLE and releases the fire suppression.
enum class ReloadState { Idle = 0, MagOut = 1, MagHeld = 2 };

// Raw pad buttons as they arrived from the player, published by the XInput hook BEFORE any of the
// plugin's own remapping. The state machine must react to what was physically pressed, not to what
// the remapper turned it into.
extern std::atomic<unsigned short> g_pad_buttons;

// TRUE while the trigger should be swallowed -- the magazine is out.
bool reload_fire_suppressed();

// Deadline for the synthesised reload press, same scheme as melee.
extern std::atomic<long long> g_reload_hold_until;
bool reload_press_active();

// ---- BUTTON SHARING ----------------------------------------------------------------------------
//
// Both buttons this feature uses already have jobs, and a full playthrough proved it: the reload
// button is also Interact / Enter Vehicle, and the left grip is Throw Grenade. Swallowing them
// unconditionally cost an entire chapter of vehicles and threw a frag on every magazine grab.
//
// TAP versus HOLD on the reload button. A tap starts the VR reload and the game never sees it; a
// hold passes straight through, so Interact and Enter Vehicle work as they always did. The press
// is only withheld until the hold threshold -- past that it is released to the game, which sees a
// press starting slightly late rather than not at all.
//
// THE GRIP is swallowed only while a magazine is expected or held. In Idle it passes through, so
// grenades work normally whenever you are not mid-reload.
bool reload_swallow_reload_button();
bool reload_swallow_grip();

// Called from the XInput hook with the raw pad state, before any remapping. Returns nothing; it
// updates the tap/hold tracking that the two predicates above report on.
void reload_note_buttons(unsigned short buttons);

// Current state, for logging and for anything that needs to know the gun is empty-handed.
ReloadState reload_state();

// Game thread, once per tick, with the tick's dt in seconds. Samples the aim hand, decides whether
// a swing happened, and arms the deadline above.
void gesture_update(float dt);

// Drop all gesture state and release any held press immediately.
//
// Called on every transition the PLAYER did not choose -- kill switch, stick mode (vehicles,
// cutscenes, death), calibration, or a tracking stall. The rule this enforces is the same one
// Config.cpp states for the kill switch: a person wearing a headset must never be left holding a
// synthetic button because a state machine got stuck.
void gesture_reset();

} // namespace halo
