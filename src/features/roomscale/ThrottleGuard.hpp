#pragma once

// THE THROTTLE GUARD -- roomscale's mode 3 writes the Blam unit object's throttle vectors
// (blam_unit_throttle_off / _off2, +0x250 and +0x25C). Those offsets were measured on ONE binary, the
// Steam build, and IsBadWritePtr only proves an address is writable, not that it is the throttle. On
// the Game Pass (WinGDK) binary, or after a patch, the write could land in a neighbouring field every
// tick the player walks. So nothing is written there until this guard has seen, on the binary it is
// running on, that the game's OWN throttle at those offsets follows the stick the game receives.
// Until then roomscale moves the player through the left stick (mode 0), which needs no offset.
//
// UNVERIFIED IS NOT KNOWN BAD. A build nobody has walked on yet stays on the stick and keeps
// measuring; only sustained, repeated disagreement condemns the offsets for the session. How it
// decides, and why each number is what it is, is in ThrottleGuard.cpp.

#include <cstdint>

namespace halo {

// XInput hook, after roomscale's own injection: the left stick exactly as the game receives it.
// `movement` is false where that stick is not walking -- menus, stick mode, the d-pad shift.
void rs_thr_guard_publish_stick(float lx, float ly, bool movement);

// Sim thread, the unit-state publish, BEFORE any write to the unit: compare the game's own throttle
// with the stick. Throttled inside to about one sample per sim tick (the publish runs ~2600/s).
void rs_thr_guard_sample(uintptr_t obj);

// Any thread: may roomscale write the unit throttle with the offsets currently configured? True once
// they are VERIFIED, or with the guard switched off (roomscalethrguard=0).
bool rs_thr_guard_allows_write();

// Game thread, while roomscale is commanding and mode 3 is configured but not allowed: says ONCE per
// stretch, loudly, why roomscale is on the stick instead.
void rs_thr_guard_note_wanted();

} // namespace halo
