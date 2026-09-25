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
// RELOADPOSEFREEZE (doctrine at the Config key). TRUE while the published VR pose should be left
// exactly where it is, because a manual reload is running and the gesture would otherwise drag the
// weapon placement about. A SUB-BEHAVIOUR OF MANUAL RELOAD: false whenever reloadvr is off, whatever
// the rack is doing, and false with the key off. Asked by whoever publishes the pose, at the instant
// it would publish it, on the game thread -- the answer and the pose are then one snapshot, and no
// copy of the window can go stale between the two.
bool reload_pose_freeze_wanted();
// RELOADHOLD's window (doctrine at the Config key, definition beside reload_pose_freeze_wanted).
// The same "a manual reload is running" window, for the arm driver whose weapon the pose freeze
// above cannot reach. Published rather than asked, because its reader is the base mod's palette
// builder on the SIM thread: `busy` is the level, `edge_ticks` the steady_clock instant it last
// changed, and one read of the pair is one snapshot -- the reader derives its ramp from the age of
// that edge instead of running a clock of its own.
struct ReloadHoldWindow { bool busy; long long edge_ticks; };
void             reload_hold_window_publish();   // game thread, once a tick
void             reload_hold_note_seat();        // the magazine was just fully seated
ReloadHoldWindow reload_hold_window();           // any thread
// ---- HAPTICS (core/reload/ReloadHaptics.cpp). Hands: 0 left, 1 right.
int  reload_gun_hand();
void reload_haptic_raw(int hand, float seconds, float amp);                     // no keys: the rack keeps its own pulses
void reload_haptic_step(const char* step, int hands, float seconds, float amp); // hands: bit 1 left, bit 2 right; reloadhaptic*
void reload_rumble_tick();                                            // game thread, once a tick, after reload_hold_window_publish
void reload_rumble_set_state(uint32_t user_index, void* vibration);   // the XInputSetState callback, after UEVR's own pulse
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
// The fetch hand, and the ONE SNAPSHOT rule applied to it (zonesnap). `hand_l` is IN/OUT: the
// caller passes its own live read and, with the snapshot on, gets the snapshot's fetch hand back,
// so the seat test and the belt grab measure the same instant the zones were built from. The
// difference between the two reads is kept and printed on the RELOAD held line.
bool reload_engine_fetch_pose(bool pose_ok, Vec3* hand_l, const Vec3* head_p);
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
// The belt magazine's grab zone, measured: one evidence line naming the frame of each side and
// re-deriving the drawn point's body-frame image, so the invariance is a number in the log.
void reload_engine_mag_zone_measured(float dist_m, const Vec3& hand_body, const Vec3& belt_body,
                                     const Vec3& anchor, float yaw_cos, float yaw_sin);
bool reload_engine_mag_cands_stale(int rank);
// reloadmagpick: the rank scale, when to ask again, what may be drawn, what may latch, and the
// tick's last word on the marker. The rank passed in is the AUTHOR'S survey rank.
int  reload_engine_mag_rank(uevr::API::UObject* mesh, int his_rank);
bool reload_engine_mag_repick(const char* wk, const char* stored);
bool reload_engine_mag_pick_use(const char* wk, uevr::API::UObject* mesh, int his_rank);
bool reload_engine_mag_pick_final(uevr::API::UObject* mesh, int his_rank);
bool reload_engine_mag_resurvey(int rank);
void reload_engine_mag_drawn(uevr::API::UObject* m, bool wanted);
bool reload_engine_mag_in_hand(uevr::API::UObject* m, const Vec3& gpos, const Vec3& hpos, float pitchr, float yawr, float rollr);

} // namespace halo
