// First-person arm hiding -- see the header comment in Arms.cpp.
#pragma once

#include "uevr/API.hpp"

namespace halo {

// Game thread, once per tick. Applies or releases the hide per config, re-asserting at ~8 Hz.
void arms_hide_update();

// One-shot FP-skeleton dump (bone names + parents); see the trigger in arms_hide_update.
void arms_dump_skeleton(uevr::API::UObject* rig);

} // namespace halo
