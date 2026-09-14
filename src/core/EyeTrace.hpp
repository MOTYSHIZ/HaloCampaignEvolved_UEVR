#pragma once

// THE BODY EYE, THE HEAD OFFSET AND THE REFLECTED KISMET TRACES.
//
// Measured by the stereo view callbacks on every frame, whether or not any feature uses them:
//   * the body eye: the engine camera before UEVR's HMD transform (stereo pre, eye 0), UE world cm;
//   * the head offset: the rendered head centre minus the body eye (stereo post), UE world cm.
// The traces are reflection-resolved Kismet LineTraceSingle / SphereTraceSingle calls: every offset
// comes from the UFunction and the HitResult script struct, never from a written-down layout (the
// same discipline as HitTrace.cpp). GAME THREAD for the traces.
//
// Consumers: headblock (clamps the rendered eye out of geometry) and heightcal (the measured view
// height and the floor trace).

#include <string_view>

#include "Math.hpp"
#include "uevr/API.hpp"

#include <atomic>
#include <cstdint>

namespace halo {

// The published measurements. False until the first sample.
bool eye_body_world(Vec3* out);
bool eye_head_offset(Vec3* out);

// A reflected LineTraceSingle on the given ETraceTypeQuery index. GAME THREAD. False on a miss or
// when the reflection did not resolve.
bool kismet_line_trace(const Vec3& a, const Vec3& b, uevr::API::UObject* const* ignore, int n_ignore,
                       int channel, Vec3* out_impact);

// A feature that clamps the rendered eye supplies these. The post-callback measurement calls them at
// the exact points the clamp has always run: mode first, shift when the frame's head offset is
// computed (eye 0, or eye 1 before eye 0 was ever seen), apply after that.
struct HeadClamp {
    int  (*mode)();
    void (*shift)(int mode, const double c[3]);
    bool (*apply)(int mode, const double raw[3], double* x, double* y, double* z);
};

// Stereo pre (render thread), inside the view position publish: note the body eye.
void eye_note_pre_view(int index, UEVR_Vector3f* position, bool is_double);

// Stereo post (render thread), right after the STOMPLOG sample: measure the head offset and let the
// clamp (nullable) move the rendered eye.
void eye_note_post_view(int index, UEVR_Vector3f* position, bool is_double, const HeadClamp* clamp);

namespace eyetrace {

// The published values themselves (UE world cm), for the head block tick's trace and log.
extern std::atomic<float> g_body_x, g_body_y, g_body_z;
extern std::atomic<bool>  g_have_body;
extern std::atomic<float> g_head_cx, g_head_cy, g_head_cz;
extern std::atomic<bool>  g_have_head;

struct TraceFn {
    const wchar_t*  name = nullptr;
    uevr::API::UFunction* fn = nullptr;
    int32_t size = 0;
    int32_t ctx = -1, start = -1, end = -1, radius = -1, channel = -1, ignore = -1, out_hit = -1,
            self = -1, ret = -1;
    bool radius_double = false;
    bool ok = false;
};

extern TraceFn g_line, g_sphere;

bool traces_ready();
bool run_trace(const TraceFn& t, const Vec3& a, const Vec3& b, float radius, int channel,
               uevr::API::UObject* const* ignore, int n_ignore, Vec3* out_loc, Vec3* out_impact);

// The HEADBLOCK log line, enabled by headblocklog or heightlog.
void hblog(const char* fmt, ...);

} // namespace eyetrace

} // namespace halo
