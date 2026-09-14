// CutsceneDump -- DEV-ONLY one-shot readback of the side-by-side scene render target
// (cutscenedump, Config.hpp).
//
// WHY A PICTURE AND NOT ANOTHER LOG LINE. The cutscene-mono modes (Config.hpp, cutscene_mono)
// proved the movie lives in the projection eye images and that both eyes carry the same thing --
// so whatever "doubled" means is a property of ONE eye image, and no counter can describe it.
// The eye dump is how upstream settled it: each eye holds one copy, the two are pixel-identical,
// and the doubling is the headset's asymmetric per-eye FOV. It stays here so the next question
// about the eye images can be answered the same way, in one launch.
//
// Compiled out of player builds: the whole lane sits behind HALO_VR_DEV, and the register call
// below is a no-op stub there.

#pragma once

namespace halo {

// Register the render callback. CALL ONCE FROM on_initialize, NEVER FROM A TICK: adding a render
// callback takes a unique_lock on the shared_mutex the tick dispatch already holds shared on this
// thread (a self-deadlock, measured upstream). Idempotent.
void cutscene_dump_register();

}   // namespace halo
