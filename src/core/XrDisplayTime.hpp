#pragma once

// THE XR DISPLAY TIME. XrFrameEndInfo::displayTime is the only valid XrTime the plugin ever sees,
// and locating one OpenXR space in another needs one (native height's STAGE floor probe).
// The author's end-frame paths (XrLayer.cpp: the API layer callback and the dev inline hook) note
// it here; any reader takes the last one. 0 = none seen yet.

#include <cstdint>

namespace halo {

// XR submit thread, once per frame, from the end-frame paths. A zero time is ignored.
void xr_display_time_note(int64_t display_time);

// Any thread: the last display time noted, or 0.
int64_t xr_display_time();

} // namespace halo
