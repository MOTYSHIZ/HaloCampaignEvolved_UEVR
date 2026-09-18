#pragma once

// THE SHARED RELOAD ENGINE. Core machinery, running while reloadvr or slidevr is enabled.
//
// The author's gesture reload (Gesture.cpp) stays his: its state machine, its reload key, its belt and
// its seat. The engine adds to it through named hooks (features/hooks/GestureHooks.hpp, HolsterHooks.hpp,
// TwoHandHooks.hpp) and runs its own ticks: the weapon's own magazine, the well and the slide into it,
// the per-weapon reload state, the rack, the pump, the chamber and the phantom round, the hidden reload's
// display, the reload sound and animation holds.
//
// The branches read AVAILABILITY, never a feature's master key: reloadvr publishes the manual magazine
// reload (SVC_MANUAL_RELOAD_AVAILABLE), slidevr the rack (SVC_RACK_AVAILABLE). Rack alone, manual alone,
// both, or neither (the engine is off and the author's reload runs as he shipped it).

#include "Gesture.hpp"   // ReloadState
#include "Math.hpp"      // Vec3
#include "uevr/API.hpp"

#include <atomic>

namespace halo {

// ---- AVAILABILITY (core/Services.hpp). Any thread.
bool reload_manual_available();   // a feature publishes the manual magazine reload
bool reload_rack_available();     // a feature publishes the rack
bool reload_engine_active();      // either: the engine runs

// ---- PUBLISHED
extern std::atomic<bool> g_slide_zone_hot;   // the off hand is inside a live rack zone (the two-handed holds must not latch)
// THE SLIDE. Once the held magazine comes inside the capture radius, it leaves the hand and
// travels into the well over reload_slide_ms, landing on the weapon's own magazine transform.
// Published for the belt-mag marker (Holster.cpp) to render: t in [0,1] while sliding, < 0
// otherwise; target = world location + UE rotator of the well (rot_valid false = keep the hand's
// rotation and slide position only, the fallback well has no orientation of its own).
extern std::atomic<float> g_reload_slide_t;
extern std::atomic<float> g_reload_slide_x, g_reload_slide_y, g_reload_slide_z;
extern std::atomic<float> g_reload_slide_pitch, g_reload_slide_yaw, g_reload_slide_roll;
extern std::atomic<bool>  g_reload_slide_rot_valid;
// The first-person pose hold the rack and the hidden reload request (the palette weapon applies it).
// Steady clock ticks, 0 = none.
extern std::atomic<long long> g_reload_pose_hold_until;
void reload_pose_hold(int ms);   // 0 clears
// Render path (once per frame, from the stereo callback): the on-weapon ammo display is written
// to 0 while the hidden reload is pending, after the game's own tick has set it.
void gesture_render_tick();

// ---- HOOK IMPLEMENTATIONS (dispatched by src/features/FeatureList.cpp). Game thread unless noted.
void reload_engine_tick_begin(float dt, bool active);   // restore windows always, the ticks while active
void reload_engine_disabled();
void reload_engine_update_begin();
void reload_engine_timed_out();
void reload_engine_swap_cancel();
bool reload_engine_grip_held();
bool reload_engine_fetch_pose(bool pose_ok, const Vec3& hand_l, const Vec3* head_p);
bool reload_engine_press_ignored();
void reload_engine_press_accepted();
bool reload_engine_belt_grab_ok(bool belt);
void reload_engine_grabbed(float hand_y);
bool reload_engine_seat(bool have_left, const Vec3& hand_l, const Vec3* hand_r_p, const Vec3* head_p);
void reload_engine_state_set(ReloadState prev, ReloadState next);
int  reload_engine_fire_suppressed();   // any thread
void reload_engine_gesture_reset();
void reload_engine_ticks(bool poses_ok, const Vec3& hpos);
// The engine switched off (both availabilities gone): every lock, record, hidden part and marker let go.
void reload_engine_released();
// A reload is in progress: the engine runs and the magazine is out, the seat is pending, or the lock
// waits for the rack. Game thread.
bool reload_engine_reload_busy();

// The held weapon's OWN magazine mesh: the first-person gun is separate static mesh
// components on the skeleton's sockets, and one of them is the magazine. nullptr when the
// weapon has none (plasma weapons) or nothing is held. Exact per weapon, no survey.
uevr::API::UObject* reload_engine_mag_mesh(int* out_rank);   // rank 4
Vec3 reload_engine_mag_belt_point();
bool reload_engine_mag_cands_stale(int rank);
// reloadmagasset: the four gates on the author's name survey, and the tick's last word on the marker.
bool reload_engine_mag_survey_off();
bool reload_engine_mag_resurvey(int rank);
bool reload_engine_mag_pick_final(int rank);
uevr::API::UObject* reload_engine_mag_spawn_mesh(uevr::API::UObject* survey, uevr::API::UObject* frag);
void reload_engine_mag_drawn(uevr::API::UObject* m, bool wanted);
bool reload_engine_mag_in_hand(uevr::API::UObject* m, const Vec3& gpos, const Vec3& hpos, float pitchr, float yawr, float rollr);

} // namespace halo
