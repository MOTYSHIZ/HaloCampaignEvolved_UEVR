// HeightCal -- measure the player's real standing eye height and put the character's eyes there.
//
// THE MECHANISM. UEVR renders the eye at game_camera - world_scale * (hmd - standing_origin)
// (FFakeStereoRenderingHook.cpp, standing_delta). The game camera IS the character's eye, so the
// view sits exactly at the character's eyes whenever hmd.y == standing_origin.y. Setting the
// standing origin's Y to the measured STANDING head height therefore maps any player's standing
// eyes onto the character's, and a physical crouch (hmd.y below it) shows as the view dropping.
// This is UEVR's own "Set Standing Height" button (VR.cpp: m_standing_origin.y = hmd.y), done
// from a robust measurement instead of one press at one instant.
//
// THE HEIGHT SOURCES (heightsrc). All three are sampled in UEVR's pose space, because that is the
// space the standing origin lives in. What differs is whether the FLOOR is known, which is what
// lets a seated or kneeling start be refused instead of calibrated:
//   1 OpenXR  -- our own STAGE reference space, created on UEVR's session through the HALOVR API
//                layer (the plugin cannot reach OpenXR entry points any other way). Locating
//                UEVR's pose space in it gives the floor offset of that space.
//   2 OpenVR  -- IVRSystem::GetDeviceToAbsoluteTrackingPose(TrackingUniverseStanding), the HMD
//                height above the calibrated floor.
//   3 UEVR    -- the plugin API's HMD pose alone. The floor is unknown on OpenXR (the UEVR source
//                creates its pose space as LOCAL), so only the relative measurement is available.
// 0 = auto: OpenXR, then OpenVR, then UEVR, falling back when one is unavailable.

#pragma once

#include "Math.hpp"

#include <atomic>
#include <cstdint>

namespace halo {

// The last XrFrameEndInfo::displayTime seen on the submit path (XrLayer.cpp). xrLocateSpace needs
// a valid XrTime and the plugin has no other source of one. 0 = none seen yet.
extern std::atomic<int64_t> g_xr_last_display_time;

// GAME THREAD, once per tick, with the plausibility-gated HMD pose (UEVR pose space, metres).
// `gameplay` permits sampling (on foot, not in a menu), `key_focus` permits the recalibrate key.
// Returns true when auto height owns the standing origin's Y this tick; *out_y is the Y to write.
// Returns false before the first calibration, so whatever owned Y before keeps owning it.
bool height_tick(const Vec3& hmd, float so_y, bool gameplay, bool key_focus, float dt, float* out_y);

}  // namespace halo
