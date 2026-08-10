// Aim-loop trace recorder -- the measurement half of automated tuning.
//
// WHY THIS EXISTS
//   Tuning the aim law by feel means one person, in a headset, holding two configurations in
//   memory and comparing them from recollection. That is slow, it cannot see anything smaller than
//   a person can notice, and it cannot tell the two failure modes apart -- a loop that lags and a
//   loop that jitters both just feel "off". A recorded trace of setpoint versus achieved aim
//   separates them numerically: lag is a time shift, jitter is high-frequency residual energy.
//
// WHY IT BUFFERS INSTEAD OF WRITING
//   Samples are taken inside aim_control_law, whose hot caller is the XInput hook -- the game's
//   INPUT PATH, running at render rate. A file write there would stall the game thread, and in VR
//   a stall is nausea rather than a blemish. So sampling only stores into a preallocated ring
//   (no allocation, no I/O, no lock) and the CSV is written once, later, from the tick thread when
//   the capture is disarmed.
//
//   Single producer, single consumer by construction: only aim_control_law writes, only the
//   disarm path reads, and the two never run together because the disarm happens on a tick after
//   the config flag has already stopped further sampling.
//
// DEV-ONLY, per DevTools.hpp: this answers questions rather than playing the game. In a release
// build the ring is not allocated and every entry point compiles to nothing.

#pragma once

#include "DevTools.hpp"

namespace halo {

#if HALO_VR_DEV

// One sample of the loop. Kept small and POD -- at render rate a 60 s capture is tens of thousands
// of these, and the point is to disturb the thing being measured as little as possible.
struct AimTraceSample {
    float  t;              // seconds since capture armed
    float  dt;             // the dt the law was handed, so frame-rate effects stay visible
    float  des_yaw, des_pitch;    // setpoint: where the controller is asking aim to be
    float  aim_yaw, aim_pitch;    // achieved: the Blam aim actually read back
    float  out_rx, out_ry;        // stick the law emitted -- jitter shows here before it shows in aim
    float  ff_rate_yaw;    // the loop's own idea of how fast the target is moving
    float  meas_rate;      // measured plant gain, deg/s per unit; context for the above
};

// Called from aim_control_law. Cheap and non-blocking: a bounds check and a store.
void aim_trace_sample(float dt, float des_yaw, float des_pitch,
                      double aim_yaw, double aim_pitch,
                      float out_rx, float out_ry, float ff_rate_yaw);

// Called from the tick. Watches the `aimtrace` config key: 0->1 arms and resets the ring, 1->0
// writes the CSV next to the config and disarms. Doing the transition here, on the tick thread,
// is what keeps file I/O off the input path.
void aim_trace_tick();

#else   // release: every entry point disappears

inline void aim_trace_sample(float, float, float, double, double, float, float, float) {}
inline void aim_trace_tick() {}

#endif

} // namespace halo
