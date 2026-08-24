// WHICH arm driver is running. Exactly one, ever.
//
// There are two complete, independent solutions to "where are the player's arms" in this tree, and
// they operate at different layers:
//
//   UeRig    Rig.cpp + Arms.cpp + Hands.cpp -- UE reflection above the anim graph. The game's
//            first-person mesh is driven as one unit, the left arm chain is bone-hidden, and the
//            hands are our own spawned components on the controllers.
//   Palette  src\palettearm\ -- the Blam node palette, below UE entirely. The native first-person
//            weapon builder is detoured and the game's own arm, hand and finger nodes are
//            rewritten before the renderer reads them.
//
// They are not layers of one design; they are alternatives. Running both means two drivers fighting
// over the same visible arms, which is the hard rule the project CLAUDE.md already states ("two arm
// drivers must never be enabled at once"). This file is that rule made structural: every entry
// point asks arm_driver_owns() first, and the arbiter releases the outgoing driver before the
// incoming one gets a frame.
//
// WHY AN ARBITER AND NOT A BOOL IN EACH MODULE. Two independent bools have four states, two of
// which are wrong, and nothing stops a config file reaching them. One enum has three, all valid.
// The A/B this exists for -- switch mid-session, in a headset, and feel the difference -- is only
// safe if the switch is atomic and the loser is actually torn down.

#pragma once

namespace halo {

enum class ArmDriverMode {
    Off     = 0,   // no arm driver; the game's stock first-person rig, untouched
    UeRig   = 1,   // Rig.cpp / Arms.cpp / Hands.cpp        (the shipped route)
    Palette = 2,   // src\palettearm\                        (the ported route)
};

// The configured mode, clamped. Reads g_cfg.arm_driver, so it follows the ~2 s live config reload
// like every other tunable.
ArmDriverMode arm_driver_mode();

// True when `mode` is the one that owns the arms this frame.
//
// This is the ONLY question a driver should ask. Do not also test the config key -- that is how the
// two get out of step.
bool arm_driver_owns(ArmDriverMode mode);

// Detect a mode change and tear down whichever driver just lost.
//
// MUST run before either driver's update in the same tick, and must run even when the mode is Off:
// switching to Off is exactly when a release is most needed, and a driver that is not being called
// any more cannot release itself. Cheap -- an int compare on the common path.
void arm_driver_arbitrate();

// Release BOTH drivers, whatever the mode. For teardown and for any transition that invalidates
// the handles either of them holds (level load, plugin shutdown, kill switch).
void arm_driver_release_all(const char* why);

// Name of the active mode, for logs.
const char* arm_driver_name(ArmDriverMode mode);

} // namespace halo
