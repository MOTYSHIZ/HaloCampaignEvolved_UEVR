// ============================================================================================
// HOLSTERS -- body-anchored slots the RIGHT hand reaches into.
//
// Three WEAPON zones (right shoulder, left shoulder, right hip) and the GRENADE pouches (chest).
// Halo carries two weapons; the zones no longer book-keep which one "lives" where (2026-09-18, by
// request -- the old stow/draw slots let an empty-hand state travel between weapons):
//   right shoulder            -> SWAP weapons, always (the game's swap press). Also ends emote
//                                mode, so the weapon you swap to never comes out hidden.
//   left shoulder / right hip -> EMOTE MODE toggle: the weapon is put away (FP weapon hidden, fire
//                                swallowed, the two-handed hold stood down) and the RIGHT hand
//                                gestures from the pose table like the left one. The same grip
//                                there again brings the weapon back.
// Halo has no native unarmed pose while it still holds a weapon, so emote mode is made here.
//
// The grenade slot arms a grenade on grip; releasing the grip with a forward swing throws it
// (the game's throw press), releasing still puts it back. Grenade-type switching is not here yet.
//
// Zones are anchored to the HEAD (position + yaw only), so they follow the body around the room
// and turn with you, and are expressed in a head-relative frame: x right, y up, z BACK (VR
// convention: forward is -z). Offsets and radius are config, in metres.
//
// Same produce-at-tick / consume-at-poll split as melee: detection here publishes press
// deadlines; the XInput hook only compares a clock. The grip is read through UEVR's action API
// per hand, so it does not depend on how this headset's grip lands on an XInput mask.
// ============================================================================================
#pragma once

#include "uevr/API.hpp"

#include <atomic>

namespace halo {

enum class HolsterSlot : int { None = -1, RightShoulder = 0, LeftShoulder = 1, RightHip = 2, LeftChest = 3, RightChest = 4 };

// Deadlines (steady_clock ticks since epoch) until which the swap / throw masks are asserted.
extern std::atomic<long long> g_holster_swap_until;
extern std::atomic<long long> g_holster_throw_until;
bool holster_swap_press_active();
bool holster_throw_press_active();

// EMOTE MODE: the weapon is put away (left shoulder / right hip grip) and the aim hand is free to
// gesture. Ends at the right shoulder (swap), the same grip again, or any holster_reset().
bool holster_emote_active();
// True while a grenade is "in hand" (grip held after arming at the chest).
bool holster_grenade_armed();

// Distance in metres from the right hand to the NEAREST holster zone centre, and whether the veto
// is currently standing on proximity rather than on a recent action. The melee veto reports a bare
// "vetoed" today, which cannot tell a hand parked in a pouch from a strike clipping the edge of
// one -- and those want opposite fixes. Reported so the log can separate them.
float holster_nearest_dist();
bool  holster_veto_by_proximity();
// True while the hand is EMPTY (weapon stowed): the XInput hook swallows the fire input.
bool holster_fire_suppressed();
// The off hand is mid-grenade (armed, or grip closed in a pouch). The two-hand latch yields to
// this so one squeeze cannot brace the weapon AND pull a grenade. Always false with
// holster_gren_hand=0, where the pouches live on the aim hand.
bool holster_offhand_busy();
// The fetch hand is inside the visible magazine's grab radius (reloadmag). Published by the
// holster tick because the mag lives in ITS body frame (torso leash, neck pivot); consumed by the
// reload state machine's MAG_OUT grab in Gesture.cpp.
bool holster_mag_hand_in();
// True while the melee detector must stand down: the hand is in or near a holster zone, or a
// holster action happened in the last few hundred ms. The reach over a shoulder IS a strike to
// the swing detector; this is what separates "swapping" from "hitting".
// The GRIP, read through UEVR's per-hand action API rather than an XInput mask. Exported because
// reload and the two-hand hold were the only features still gating on a mask (reloadgrip,
// default 0x0100): whether a physical grip produces that bit depends on the controller and the
// runtime, so on a setup where it does not, reload stalls in MAG_OUT with the trigger suppressed
// and two-handing never latches -- both silently, and both reported from the field.
bool holster_grip_held(bool right);

bool holster_melee_veto();
// The OFF hand's melee veto: its own pouch occupancy plus the recent-action window (the aim hand's
// veto above would stand every left punch down while the rifle sits at chest height).
bool holster_offhand_melee_veto();
// Deadline for the synthesised grenade-type switch press.
extern std::atomic<long long> g_holster_gswitch_until;
bool holster_gswitch_press_active();

// Game thread, once per tick, after gesture_update().
void holster_update(float dt);
// Any transition the player did not choose (stick mode, menu, calibration, kill switch).
void holster_reset();

} // namespace halo
