#pragma once

// THE TICK STAGE MARKER, read by the engine tick's exception filter.

namespace halo {

// The last stage marker the game tick passed. A crash inside a 4000-line tick is unlocatable from
// "one of the plugins has an error"; the fault report names this stage. Plain pointer to a literal,
// written by the game thread before each major stage, never freed.
extern const char* volatile g_tick_stage;

} // namespace halo
