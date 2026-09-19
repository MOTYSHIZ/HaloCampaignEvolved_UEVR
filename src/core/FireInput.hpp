#pragma once

// THE PLAYER'S OWN FIRE INPUT, shared by the ForceTube per-shot filter, the rack and the
// rounds-loaded probe.

#include <atomic>

struct _XINPUT_STATE;

namespace halo {

// The last moment the player's own FIRE input was down, as a steady_clock tick count. Written by
// the XInput hook from the RAW pad, before the plugin's own suppression and synthetic presses, so
// it follows the finger rather than the composed state. create_projectile is shared by everything
// that shoots, and a proximity test cannot tell your rifle from a marine's at arm's length; your
// finger can.
extern std::atomic<long long> g_ft_fire_at;
void fire_input_note(bool firing);

// XInput hook thread, once per poll, on the raw pad: right trigger past 64 or the right shoulder.
void fire_input_note_pad(_XINPUT_STATE* state);

} // namespace halo
