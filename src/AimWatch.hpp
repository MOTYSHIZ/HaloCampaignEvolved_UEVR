// Hardware write-watchpoint: find the code that STAMPS the aim.
//
// WHY THIS, RATHER THAN MORE SCANNING
//   Every attempt to locate Blam's authoritative aim by value has the same weakness: it only finds
//   the encodings we thought to guess. Searching for the forward vector found ~64 frame-local
//   buffers that are recycled every frame; searching for the angle pair in radians found one stable
//   copy that is inert -- writes to it stick and change nothing. Both negatives are only as strong
//   as the guess behind them.
//
//   ControlRotation at PlayerController+0x350 is different: it demonstrably RECEIVES the aim every
//   frame, because we read it there and it always agrees with what the game is doing. So something
//   writes it, and that writer necessarily knows where the real value lives. Catching the write
//   gives us the instruction, and the instruction gives us the source -- no guessing required.
//
// HOW
//   x86 debug registers. Dr0 holds the address, Dr7 arms it for WRITES of a given length, and the
//   CPU raises a single-step exception after any instruction that writes there. A vectored
//   exception handler records the faulting RIP.
//
//   Debug registers are PER-THREAD, so every thread in the process is armed -- the writer may not
//   be the thread we happen to be running on, and missing it would look exactly like "nothing
//   writes here".
//
//   Preferred over a PAGE_GUARD approach because a guard page traps every access to the whole 4 KB
//   page, reads included, which on a live PlayerController is a firehose and slows the game enough
//   to change what it does.
//
// SAFETY
//   Read-only observation: the handler records RIP and continues. It self-disarms after a small
//   number of distinct hits, because the aim is stamped every frame and an unbounded log would be
//   both useless and a performance problem in a headset.
//
// DEV-ONLY (DevTools.hpp). Nothing here is compiled into a release build.

#pragma once

#include "DevTools.hpp"

namespace halo {

#if HALO_VR_DEV

// Watches the `aimwatch` config key. 0 -> 1 arms a write-watchpoint at `aimwatchaddr` (or, if that
// is 0, at the live PlayerController + CONTROL_ROTATION_OFFSET) and logs the distinct instruction
// addresses that write there. Auto-disarms after a handful of hits.
void aim_watch_tick();

// Disarm and restore debug registers. Called at shutdown so a watchpoint never outlives the
// session that set it.
void aim_watch_shutdown();

#else

inline void aim_watch_tick() {}
inline void aim_watch_shutdown() {}

#endif

} // namespace halo
