#pragma once

// THE PALETTE WEAPON'S PLUGIN-TICK AND RENDER WORK (palettewpn): the publishes the sim-thread palette hook
// composes from, the STOMPLOG ring, the per-tick placement instruments, the tick-start and render-rate
// instruments, and the aim law's latch. The author's Plugin.cpp reaches them through hooks
// (features/hooks/PluginHooks.hpp); the palettewpn table's slots below.

#include "uevr/API.hpp"

#include <atomic>
#include <cstdint>

namespace halo {

void stomp_mark(int point, float yaw, float e0, float e1, float e2);   // STOMPLOG, any thread
void stomp_flush();
extern std::atomic<unsigned> g_tick_id;
extern uevr::API::UObject* g_fpscale_camera;
void aim_reanchor_request(const char* why);

void palette_wpn_game_tick_after_blam_drive();
void palette_wpn_game_tick_before_vehicle();
void palette_wpn_game_tick_after_vehicle(uint32_t tick);
void palette_wpn_game_tick_after_gestures(float dt);
void palette_wpn_engine_tick_start();
void palette_wpn_engine_tick_end();
void palette_wpn_post_engine_tick();
bool palette_wpn_fp_weapon_live();
void palette_wpn_rig_parent_dropped();
bool palette_wpn_rig_driver_stood_down();
void palette_wpn_stereo_pre_eye_instruments(int index);
void palette_wpn_render_refresh();
void palette_wpn_stereo_pre_eye_meters(int index);
void palette_wpn_stereo_post_eye_sample(int index);
void palette_wpn_stereo_post_eye_late(int index);
void palette_wpn_teardown();
void palette_wpn_aim_law_sampling();
void palette_wpn_aim_law_sampled(double ay, double ap);
void palette_wpn_game_tick_after_rig_driver(double aim_yaw, double aim_pitch, uint32_t tick);

} // namespace halo
