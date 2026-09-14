#pragma once

// THE POSE LATCH, THE FRAME AUDIT AND THE AIM WRITER AGREEMENT (palettewpn). Moved from MotionAimControl; the
// author's get_pose and aim law reach them through hooks (features/hooks/MotionAimHooks.hpp).

#include "uevr/API.hpp"

#include <atomic>
#include <cstdint>

namespace halo {

// POSELATCH: snapshot every device once so all readers share one sample. site 1 = engine tick
// start, site 3 = the aim law's XInput sample. See the definition for the modes.
void pose_latch_refresh(int site);
// DRAWAIM: hand intent (UE degrees) of the outgoing snapshot (prev) and the new one (cur), stored
// at each latch refresh. palettecam 14 divides by prev, 15 by cur.
extern std::atomic<float> g_intent_prev_y, g_intent_prev_p, g_intent_cur_y, g_intent_cur_p;
extern std::atomic<bool>  g_intent_prev_ok, g_intent_cur_ok;
extern std::atomic<float> g_intent_prev2_y, g_intent_prev2_p;   // RETSTAMP: intent two snapshots back
extern std::atomic<bool>  g_intent_prev2_ok;
// FRAMEAUDIT: snapshot generation counter, the generation of the stored prev intent, and the
// generation the CALLING THREAD was last served by get_pose.
extern std::atomic<uint32_t> g_latch_gen, g_intent_prev_gen, g_intent_cur_gen;
uint32_t pose_latch_last_gen();
// The Blam aim writer notes the UE-convention angle it wrote, for the AIMWRITERS agreement line.
void aim_writer_note_blam(float yaw_deg, float pitch_deg);
// The latched pose for idx, or false when the live read must be used (the get_pose hook).
bool pose_latch_lookup(UEVR_TrackedDeviceIndex idx, bool use_aim, uevr::API::VR::Pose* out);
// AIMWRITERS: the UE direct writer's angle against the Blam writer's note (the direct-write hook).
void aim_writer_compare_direct(float yaw_deg, float pitch_deg);
// The aim law's direct-write hooks (the palettewpn table's slots).
bool palette_wpn_aim_direct_write_skipped();
void palette_wpn_aim_direct_written(float yaw, float pitch);

} // namespace halo
