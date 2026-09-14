#pragma once

// THE ON-FOOT RETICULE'S LATEST TRACE DEPTH (cm) and when it was taken: published by the reticule's trace,
// read by the render-rate stamped placement (aimreticulestamp). Any thread.

namespace halo {

void  reticule_depth_note(float d);
float reticule_depth();
long long reticule_depth_ms();   // steady clock milliseconds, 0 = never

} // namespace halo
