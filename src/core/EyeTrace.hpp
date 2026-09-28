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
//
// shift RETURNS THE CLAMP AS A FRACTION of the head offset it was given: the eye is moved by
// fraction x c, back toward the body. *horizontal_only says it moved the horizontal part of c only
// (UE Z is up). 0 = no clamp this frame. See eye_clamped_standing_origin, which is why it is returned.
struct HeadClamp {
    int   (*mode)();
    float (*shift)(int mode, const double c[3], bool* horizontal_only);
    bool  (*apply)(int mode, const double raw[3], double* x, double* y, double* z);
};

// ---- THE STANDING ORIGIN THE RENDERED EYE IS ACTUALLY AT.
//
// UEVR renders the eye at camera + gamespace(hmd - standing_origin), and everything this plugin
// places from ROOM positions -- the weapon, the arms, holsters, the wrist HUD, reload zones -- is
// built the same way from the same origin, so it agrees with the eye. A clamp breaks that: the head
// block moves the rendered EYE back out of a wall and nothing else, so the view stopped at the wall
// while the hands carried on into it by exactly the amount the head was held back ("my head is
// blocked, but my arms are still free to move into the geo"). With roomscale walking the body after
// the unclamped head it read as the arms moving double.
//
// The clamp is a fraction of the head offset c, and c is a linear image of (hmd - standing_origin),
// so the SAME fraction of the room offset is exactly the room-space form of the clamp -- no rotation,
// no scale, no frame to get wrong. This returns `so` pulled toward `hmd` by that fraction (vertical
// untouched when the clamp was horizontal only). Building from it instead of `so` moves anything by
// the same vector the eye moved. Room metres in and out, UEVR tracking space.
//
// Returns `so` unchanged when nothing clamps, or when the clamp is stale (no rendered frame for
// 250 ms: menus, loads, the service off). ANY THREAD: reads atomics only.
//
// FOR PLACEMENT ONLY. Whatever WRITES the standing origin -- the leash, roomscale, auto height --
// must keep reading the real one, or it would chase its own correction.
Vec3 eye_clamped_standing_origin(const Vec3& so, const Vec3& hmd);

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

// run_trace's report of WHAT it hit, for a caller's log. Pointers are valid for the calling tick only
// (resolved through the object array just now) -- never keep them.
struct TraceHit {
    uevr::API::UObject* component = nullptr;   // the blocking hit's component, if it resolved
    uevr::API::UObject* inside    = nullptr;   // an actor the trace STARTED inside and looked past
    int                 retries   = 0;         // how many start-inside hits were looked past
};

// A trace that STARTS INSIDE A BODY reports it as a hit at the start (Time 0, the engine's
// bStartPenetrating case), and a body around the camera is not a surface in front of the head or a
// floor under the eye. The candidate is the player's own: his biped (BP_SpartansBipedActor_C) is a
// separate actor from the camera pawn (BP_MeteoritePawn_C), so the pawn in ActorsToIgnore does not
// cover it, and VehCam found the camera inside it in a seat ("your head is inside it, so it clips
// constantly"). Whether that is what held the view on T&R is NOT measured -- the aim trace, which
// ignores the same two actors, never once logged a hit on the biped -- so the log names what every
// look-past was inside (headblocklog / heightlog). A start-inside hit on a body is looked past: the
// trace runs again with that actor ignored, up to twice, and a trace that only ever starts inside
// bodies reports no hit. Per trace, never remembered.
//
// ONLY bodies (skeletal meshes) and ONLY line traces. A static mesh the camera is inside is a wall it
// clipped into, and a sphere sweep starts inside a real wall whenever the body stands within its
// radius of one: in both the start hit is the right answer, and looking past it would put the head
// through. If the engine does not report start-inside hits for a line trace at all, none of this runs.
bool run_trace(const TraceFn& t, const Vec3& a, const Vec3& b, float radius, int channel,
               uevr::API::UObject* const* ignore, int n_ignore, Vec3* out_loc, Vec3* out_impact,
               TraceHit* out_hit = nullptr);

// The HEADBLOCK log line, enabled by headblocklog or heightlog.
void hblog(const char* fmt, ...);

} // namespace eyetrace

} // namespace halo
