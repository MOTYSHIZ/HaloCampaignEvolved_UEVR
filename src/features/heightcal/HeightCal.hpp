// HeightCal -- the rendered eye's height above the GAME floor, from the head's height above the
// REAL floor.
//
// THE MECHANISM, fitted not assumed. UEVR renders the eye at the game camera plus world_scale x
// (hmd - standing_origin) (FFakeStereoRenderingHook.cpp, standing_delta; VR_DecoupledPitch
// flattens the rotation so the vertical term is not tilted by game pitch). Fitted from the
// 2026-09-13 16:42-16:48 log, 130 samples over +-0.4 m of head travel: rendered-eye Z minus camera
// Z = 112.7 x (hmd.y - origin.y) + 0.04 cm, residual 0.53 cm, against 100 x VR_WorldScale = 112.6.
// So with E = the camera's height above the character's feet (UE cm) and S = UE cm per VR metre:
//
//     view height above game floor  V = E + S x (hmd.y - origin.y)
//     =>  origin.y = hmd.y + (E - V) / S
//
// MODES (heightmode):
//   absolute (0, default) -- V = K x head_abs, head_abs = hmd.y + floor offset (metres above the real
//                            floor). With K = S (heightscale 0) the head cancels: origin.y = E/S - floor.
//   seated   (1)          -- V = K x head_abs + O, one constant O set at calibration so the view is
//                            at the target height at that moment; afterwards every movement is 1:1.
//   eyes     (2)          -- the standing head is mapped to the character's eyes (calibrated H). Also
//                            the automatic fallback while the floor is unknown.
//
// THE FLOOR comes from heightsrc: 0 auto, 1 OpenXR STAGE (own spaces on UEVR's session through the
// HALOVR API layer), 2 OpenVR standing universe, 3 UEVR pose only (floor unknown on OpenXR).

//
// FEATURE heightcal (Experimental). Hook slots: parse_key (height*), leash_block_wanted,
// leash_vertical (the origin's Y, where the author's vertical leash is), menu_command (calib:height)
// and menu_status_line. Table: kHeightCalHooks.

#pragma once

#include <string_view>

#include "Math.hpp"
#include "features/FeatureHooks.hpp"
#include "uevr/API.hpp"

#include <atomic>
#include <cstdint>
#include <string>

namespace halo {

extern const FeatureHooks kHeightCalHooks;

// The auto height keys (height*).
bool heightcal_parse_key(const char* key, const char* val, double v);

// The last XrFrameEndInfo::displayTime seen on the submit path (XrLayer.cpp). 0 = none yet.
extern std::atomic<int64_t> g_xr_last_display_time;

// GAME THREAD, once per tick, with the plausibility-gated HMD pose (UEVR pose space, metres).
// `active` = on-foot gameplay (not a menu, vehicle or cutscene); while inactive the last origin Y is
// held. `ignore` = actors the floor trace must not hit. Returns true when this feature owns the
// standing origin's Y this tick; *out_y is the Y to write.
bool height_tick(const Vec3& hmd, float so_y, bool active, bool key_focus, float dt,
                 uevr::API::UObject* const* ignore, int n_ignore, float* out_y);

// The menu's one-shot recalibrate (command calib:height). Seated: a fresh offset. Eyes: the 1 s
// standing sample. Absolute: nothing to calibrate (logged). Returns true when queued.
bool height_request_calibrate();

// "height=<mode> <view height above game floor> m" for the menu status file; empty when off.
std::string height_status_line();

}  // namespace halo
