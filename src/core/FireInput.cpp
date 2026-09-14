#include "core/FireInput.hpp"

#include <Windows.h>
#include <Xinput.h>

#include <atomic>
#include <chrono>

namespace halo {

std::atomic<long long> g_ft_fire_at{0};
std::atomic<bool>      g_fire_kick_live{false};

void fire_input_note(bool firing) {
    if (firing) g_ft_fire_at.store(std::chrono::steady_clock::now().time_since_epoch().count(),
                                   std::memory_order_relaxed);
}

void fire_input_note_pad(_XINPUT_STATE* state) {
        // The player's OWN fire input, sampled here because this is still the RAW pad -- our own
        // reload/holster suppression and the synthetic presses all happen further down, and the
        // haptics must follow the finger, not the composed state.
        fire_input_note(state->Gamepad.bRightTrigger >= 64 ||
                            (state->Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER) != 0);
}

} // namespace halo
