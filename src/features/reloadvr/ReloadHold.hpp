#pragma once

// RELOADHOLD: THE WEAPON HELD STILL THROUGH A MANUAL RELOAD, UNDER THE BASE MOD'S ARM DRIVER.
//
// A SUB-BEHAVIOUR OF MANUAL RELOAD, not a feature beside it. The key is a sub-setting of reloadvr
// in the registry, it is parsed in core/reload with the rest of the reload's keys, and every slot
// below is dispatched from the reloadvr table -- so with manual reload off none of it is reached.
// The doctrine, the three mechanisms and the compiled default are at the Config key
// (core/config/ConfigFields.inl, reload_hold).
//
// WHY IT EXISTS. reloadposefreeze already holds the weapon for the fork's own placement driver
// (armdriver 3) by not publishing a new pose while a reload runs. Under armdriver 2 the weapon is
// drawn by the base mod's own carry, that publisher never runs, and the reload gesture drags the
// gun around by the aim hand while the player is trying to push a magazine into it.
//
// EVERY SLOT RUNS ON THE SIM THREAD, inside the base mod's first-person pose build, so none of it
// exists in any other arm driver mode. ONE SNAPSHOT: the window, the mechanism and the ramp weight
// are sampled once per pose build, by the gates slot, and every slot after it in that build reads
// that one sample. The capture banks reuse the live slot's sample, so every bank draws one pose.

#include "palettearm/PaletteMath.hpp"

namespace halo {

// features_pa_anim_gates. Takes the build's snapshot, then (mechanism 1) raises his hold and off
// weights so his own per-action hold does the holding.
void reload_hold_pa_anim_gates(bool is_capture_bank, float& join_w, float& off_w, float& stock_w_all,
                               float& hold_w);

// features_pa_rig_target. Latches the base mod's weapon target at the start of the reload; measures
// how far it drifts while held (every mechanism, for the log) and, under mechanisms 2 and 3, feeds
// the latched one back in.
void reload_hold_pa_rig_target(bool is_capture_bank, bool have_rt, palettearm::Mat3& basis,
                               palettearm::Vec3& position);

// features_pa_aim_wrist_note / _keep. Mechanism 3 only: the aim wrist the player's controller asked
// for, kept through his hand-rides-the-gun block, so the gun is pinned and the hand is not.
void reload_hold_pa_aim_wrist_note(bool is_aim, bool is_capture_bank,
                                   const palettearm::Vec3& wrist_target,
                                   const palettearm::Mat3& desired_wrist);
void reload_hold_pa_aim_wrist_keep(bool is_aim, bool is_capture_bank, palettearm::Vec3& wrist_target,
                                   palettearm::Mat3& desired_wrist);

} // namespace halo
