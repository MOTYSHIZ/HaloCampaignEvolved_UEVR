#include "core/CameraBob.hpp"

namespace halo {

// CAMERA BOB: the fast part of (camera - pawn root), world cm, published by the game tick.
// Subtracted from the palette's world hand so view and gun stay consistent.
std::atomic<float> g_bob_x{0.0f}, g_bob_y{0.0f}, g_bob_z{0.0f};

} // namespace halo
