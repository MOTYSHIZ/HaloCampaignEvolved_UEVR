#pragma once

// THE CAMERA BOB: the fast part of (camera - pawn root), world cm, published by the game tick.
// Measured by roomscale's bob probe (bobcancel / boblog) and subtracted from the palette weapon's
// world hand, so the view and the gun stay consistent. Zero unless something publishes it.

#include <atomic>

namespace halo {

extern std::atomic<float> g_bob_x, g_bob_y, g_bob_z;

} // namespace halo
