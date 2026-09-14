#pragma once

// MELEE INSTRUMENTS AND THE AIM PIN: the fork's changes to the author's aim-hand melee detector
// (meleeswing), which have no feature row of their own. Called from Gesture.cpp's hook points
// (features/hooks/GestureHooks.hpp), game thread.
//
//   melee_hold_check    the deferred MELEE HOLD CHECK line, 80 ms into a hold (meleelog)
//   melee_swing_moving  the aim setpoint and controller angles captured when a swing starts moving
//   melee_vetoed        the holster veto on the aim hand's strike, with its MELEE VETOED evidence line
//   melee_fired         the aim pin (swing direction in mode 1, swing-start aim in mode 0), the hold
//                       check schedule, and the MELEE FIRED line with the aim drift since the swing began

namespace halo {

void melee_hold_check();
void melee_swing_moving(bool s_in_swing);
bool melee_vetoed(long long now, float speed, float reach);
bool melee_fired(long long now, float speed, float reach);

} // namespace halo
