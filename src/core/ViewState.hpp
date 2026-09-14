#pragma once

// VIEW STATE SHARED ACROSS FEATURES WITHOUT NAMING ONE ANOTHER.

#include <atomic>

namespace halo {

// True while a feature keeps the rendered camera on a vehicle seat even unmounted (the seat camera's
// always mode). The rendered eye is then not the body's eye, so body-eye measurements stand down.
// Written on the game tick by the feature that moves the camera; false when no such feature is on.
extern std::atomic<bool> g_view_seat_always;

} // namespace halo
