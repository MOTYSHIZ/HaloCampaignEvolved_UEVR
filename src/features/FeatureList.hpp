#pragma once

// The feature list's runtime state, for the registry's startup log (Features.cpp).

namespace halo {

// Every feature's on/off state and, for each core service, whether it is active and which enabled
// features turned it on. Logged once at startup (behind the registry's FEATURE lines) and again
// whenever a config reload changes a feature's state.
void features_log_runtime();

} // namespace halo
