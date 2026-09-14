#pragma once

// THE CAMERA BOB (core service SVC_CAMERA_BOB): the fast part of (camera - pawn root), world cm,
// measured on the game tick and subtracted from the palette weapon's world hand, so the view and the
// gun stay consistent. Zero while the service is inactive. Tuned by bobcancel / boblog / bobtau
// (core keys, core/CoreKeys.cpp).

#include <atomic>

namespace halo {

extern std::atomic<float> g_bob_x, g_bob_y, g_bob_z;

// Game thread, once per tick, right before the leash block, while SVC_CAMERA_BOB is active.
void camera_bob_tick();

// The service went inactive: publish zero.
void camera_bob_reset();

} // namespace halo
