#include "Gesture.hpp"
#include "TwoHandAim.hpp"
#include "Holster.hpp"

#include "BlamPalette.hpp"
#include "PaletteTwoHand.hpp"   // palette_two_hand_reset / g_th_latched: the palette weapon mode hold
#include "core/HiddenReload.hpp"   // g_wristhud_hide_cradle: the hidden reload hides the ammo cradle
#include "Config.hpp"
#include "core/FireInput.hpp"      // g_ft_fire_at: the off hand stands down while the stock kicks
#include "Markers.hpp"            // the magwell insert marker rides the holster marker machinery
#include "Math.hpp"
#include "MotionAimControl.hpp"
#include "Rig.hpp"                // fp_weapon_actor -- the mag hide walks its components
#include "UeObject.hpp"
#include "WeaponCalib.hpp"
#include <intrin.h>

#include <string>
#include <vector>
#include <algorithm>

#include <chrono>
#include <cmath>

using uevr::API;

namespace halo {
std::atomic<bool> g_slide_zone_hot{false};   // read by TwoHand: the off hand is inside a live rack zone

std::atomic<long long> g_melee_hold_until{0};
std::atomic<long long> g_reload_hold_until{0};
std::atomic<float> g_reload_slide_t{-1.0f};
std::atomic<float> g_reload_slide_x{0.0f}, g_reload_slide_y{0.0f}, g_reload_slide_z{0.0f};
std::atomic<float> g_reload_slide_pitch{0.0f}, g_reload_slide_yaw{0.0f}, g_reload_slide_roll{0.0f};
std::atomic<bool>  g_reload_slide_rot_valid{false};
// beside the derivations they feed. This module consumes both through their headers.

namespace {

using clock_t_ = std::chrono::steady_clock;

inline long long now_ticks() { return clock_t_::now().time_since_epoch().count(); }
inline long long ms_to_ticks(int ms) {
    return std::chrono::duration_cast<clock_t_::duration>(std::chrono::milliseconds(ms)).count();
}

// Previous sample. Everything is stored HEAD-RELATIVE: s_prev_rel is (hand - head), which removes
// whole-body motion before any derivative is taken.
bool  s_have_prev  = false;
Vec3  s_prev_rel{0.0f, 0.0f, 0.0f};
float s_prev_reach = 0.0f;

// Smoothed relative velocity, and the smoothed rate the arm is extending at.
Vec3  s_vel{0.0f, 0.0f, 0.0f};
float s_ext = 0.0f;

// Refractory deadline, same clock as the hold deadline.
long long s_cooldown_until = 0;

// Peaks since the last time a swing settled, purely for melee_log. Reported when the hand comes
// back to rest so one line describes one swing, rather than a line per tick.
float s_peak_speed = 0.0f;
float s_peak_ext   = 0.0f;
float s_peak_reach = 0.0f;
bool  s_in_swing   = false;
// ---- MEASUREMENT (meleelog). Two different faults both fit "the swing melee does not connect
// but the stick click does", and they want opposite fixes:
//   A. the strike never fires at all, because the holster veto stands the detector down;
//   B. it fires, but the AIM has been dragged off the target by the swing itself, so Halo's
//      lunge tracks empty air. Nothing in this plugin freezes or compensates aim during a melee.
// The aim setpoint is captured when the swing starts moving and compared at the instant the
// strike fires. Which one is happening is a number, not an argument.
// The weapon whose magazine the in-flight reload belongs to (weapon_key at the Idle exit).
std::string s_reload_weapon;
float s_swing_yaw0 = 0.0f, s_swing_pitch0 = 0.0f;
bool  s_swing_aim_ok = false;
// The CONTROLLER angles at the swing's start, which is what the aim law actually consumes.
// Freezing the input rather than overriding the output means the stick loop, the direct write and
// the sim driver all inherit the hold from one place instead of three that can drift apart.
float s_swing_ctrl_yaw = 0.0f, s_swing_ctrl_pitch = 0.0f;
bool  s_swing_ctrl_ok = false;
// ---- DOES THE HOLD ACTUALLY HOLD?
// The "AIM MOVED" figure on the FIRED line is sampled before the hold has been applied, so it
// measures the drift the hold is meant to cancel and can never show whether it was cancelled.
// Confirming the fix needs a reading from INSIDE the hold window: this fires one deferred log a
// short time after the strike, comparing the live setpoint against the swing's start. Near zero
// means the aim is being held where the punch was launched from. Anything else means it is not,
// and no amount of the fix looking right in the source changes that.
float     s_hold_ref_yaw = 0.0f, s_hold_ref_pitch = 0.0f;
long long s_hold_check_at = 0;
bool      s_hold_was_armed = false;
int   s_veto_count = 0;

// Below this relative speed the hand counts as at rest and a swing is considered over. Not
// configurable on purpose: it exists to segment the LOG, and a knob to tune the tuning instrument
// is not worth the config surface.
constexpr float REST_SPEED_MPS = 0.35f;

// ---- RELOAD STATE ------------------------------------------------------------------------------
ReloadState s_reload = ReloadState::Idle;
// A MAGAZINE THAT IS OUT STAYS OUT ACROSS A WEAPON SWAP (from the headset, 2026-09-04): drop gun A's mag,
// swap to B, fire, swap back -- A still needs the whole reload. What the machine knew about A
// when it left the hand, by weapon key, restored when A returns.
struct WpnMem { std::string key; bool mag_out; int chamber_left; bool empty; bool lock_pending; bool true_empty; };
WpnMem s_wpn_mem[8];
WpnMem* wpn_mem_find(const std::string& key) { for (auto& m : s_wpn_mem) if (!m.key.empty() && m.key == key) return &m; return nullptr; }
WpnMem* wpn_mem_slot(const std::string& key) { if (auto* m = wpn_mem_find(key)) return m; for (auto& m : s_wpn_mem) if (m.key.empty()) return &m; return &s_wpn_mem[0]; }
void wpn_mem_clear(const std::string& key) { if (auto* m = wpn_mem_find(key)) *m = WpnMem{}; }
bool s_restoring_mag_out = false;   // set_state must not drop a second magazine on a restore
unsigned short s_prev_buttons = 0;

// How long the synthesised reload press is held. Same reasoning as melee_hold_ms: one poll can
// land between the game's own input samples and be missed entirely.
constexpr int RELOAD_HOLD_MS = 90;

#include "features/reloadvr/Gesture_mag_anim_sound.inl"   // fork feature: reloadvr (magazine, animation, sound)

#include "features/slidevr/Gesture_rack.inl"   // fork feature: slidevr (the rack)

#include "features/reloadvr/Gesture_ammo_dump_magdrop.inl"   // fork feature: reloadvr (ammo probe + dropped magazine)

// When the CURRENT state was entered. Only meaningful outside Idle, and only the watchdog reads it.
long long s_reload_since = 0;

const char* state_name(ReloadState s) {
    switch (s) {
        case ReloadState::MagOut:  return "MAG_OUT";
        case ReloadState::MagHeld: return "MAG_HELD";
        default:                   return "IDLE";
    }
}

void set_state(ReloadState next, const char* why) {
    if (s_reload == next) return;
    if (g_cfg.reload_log) {
        API::get()->log_info("[Halo-CampE-UEVR] RELOAD %s -> %s (%s)",
                             state_name(s_reload), state_name(next), why);
    }
    const ReloadState prev = s_reload;
    s_reload = next;
    // The weapon's own mag leaves with the state and returns with it -- Idle is the only state
    // where the gun should be showing a magazine.
    mag_hide_apply(next != ReloadState::Idle);
    // ...and on the way out it FALLS: the drop is spawned from the component just hidden.
    if (prev == ReloadState::Idle && next == ReloadState::MagOut && !s_restoring_mag_out) { if (!slide_parts_mag_drop()) mag_drop_spawn(); ak_dump("drop"); ak_step_sound("drop"); ak_step_sound("open");
        if (g_cfg.reload_press_at == 1 && slide_weapon_ok()) {
            const bool empty_now = weapon_empty_now();
            if (empty_now) { s_sl_pressed_early = true; reload_press_now("mag drop"); }
            else { s_sl_press_pending = true; if (g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD press waits for the chambered shot or the seat"); }
        }
    }
    // Restart the watchdog on EVERY transition, not only on leaving Idle: MAG_HELD -> MAG_OUT
    // (fumbling the magazine) is real progress and should buy the player the full window again.
    s_reload_since = now_ticks();
}

// Give up on a gesture that cannot be finished, and give the trigger back.
//
// Runs from reload_update() BEFORE anything that needs a pose, so a tracking dropout on the off
// hand -- one of the named ways to get stuck -- cannot also disable the escape hatch.
bool reload_watchdog_expired() {
    if (s_reload == ReloadState::Idle) return false;
    if (!(g_cfg.reload_timeout_s > 0.0f)) return false;
    const long long limit = ms_to_ticks((int)(g_cfg.reload_timeout_s * 1000.0f));
    return (now_ticks() - s_reload_since) >= limit;
}

} // namespace

bool melee_press_active() {
    const long long until = g_melee_hold_until.load(std::memory_order_relaxed);
    return until != 0 && now_ticks() < until;
}

bool reload_press_active() {
    const long long until = g_reload_hold_until.load(std::memory_order_relaxed);
    return until != 0 && now_ticks() < until;
}

// ---- TAP / HOLD TRACKING -----------------------------------------------------------------------
//
// Lives in the XInput hook's cadence rather than the ~32 Hz tick, because a hold threshold judged
// at 31 ms granularity feels arbitrary to the hand. Only clocks and atomics here -- no reflection,
// no allocation, per the rule on that callback.
namespace {
std::atomic<long long>  s_reload_down_at{0};    // when the reload button went down, 0 = up
std::atomic<bool>       s_reload_passing{false};// hold threshold crossed: stop swallowing
std::atomic<bool>       s_reload_tap{false};    // a completed tap, waiting for the tick to consume
static float            s_grab_y = 0.0f;         // left-hand height (VR y) at the belt grab
}

void reload_note_buttons(unsigned short buttons) {
    if (!g_cfg.reload_vr || g_cfg.reload_mask == 0) {
        s_reload_down_at.store(0, std::memory_order_relaxed);
        s_reload_passing.store(false, std::memory_order_relaxed);
        return;
    }

    const bool down = (buttons & (unsigned short)g_cfg.reload_mask) != 0;
    const long long at = s_reload_down_at.load(std::memory_order_relaxed);

    if (down && at == 0) {
        s_reload_down_at.store(now_ticks(), std::memory_order_relaxed);
        s_reload_passing.store(false, std::memory_order_relaxed);
        return;
    }
    if (down) {
        // Still held. Once past the threshold this is an Interact / Enter Vehicle press, not a
        // reload, so stop withholding it and let the game have the rest of the hold.
        if (!s_reload_passing.load(std::memory_order_relaxed) &&
            now_ticks() - at >= ms_to_ticks(g_cfg.reload_hold_ms)) {
            s_reload_passing.store(true, std::memory_order_relaxed);
        }
        return;
    }

    // Released. A short press that was never promoted to a hold is a tap: the reload gesture.
    if (at != 0) {
        if (!s_reload_passing.load(std::memory_order_relaxed)) {
            s_reload_tap.store(true, std::memory_order_relaxed);
        }
        s_reload_down_at.store(0, std::memory_order_relaxed);
        s_reload_passing.store(false, std::memory_order_relaxed);
    }
}

bool reload_swallow_reload_button() {
    if (!g_cfg.reload_vr || g_cfg.reload_mask == 0) return false;
    // Withhold only while the press is still short enough to be a tap.
    return s_reload_down_at.load(std::memory_order_relaxed) != 0 &&
           !s_reload_passing.load(std::memory_order_relaxed);
}

// RELOAD'S OWN grip swallow, and nothing more.
//
// Whether the grip reaches the game AT ALL is not decided here any more -- that is `gripswallow`,
// a standalone unbind applied at the XInput call site. It used to be routed through this function
// via grip_exclusive, which meant switching off an unrelated reload feature silently handed the
// grip back to the game and put grenades on it again. A binding should not depend on a feature
// flag from another lane.
//
// What is left here is genuinely reload's business: withholding the grip for the WINDOW in which a
// magazine is expected or held, which only the reload state machine can define.
bool reload_swallow_grip() {
    if (!g_cfg.reload_vr || g_cfg.reload_grip_mask == 0) return false;
    if (g_cfg.grip_exclusive) return true;
    return s_reload != ReloadState::Idle;
}

#include "features/reloadvr/Gesture_hidden_display.inl"   // fork feature: reloadvr (hidden reload display)
ReloadState reload_state() { return s_reload; }

bool reload_fire_suppressed() {
    // Slide weapons keep the trigger LIVE with the magazine out: whatever the game still has
    // loaded is the chambered round(s), and the game locks the slide back itself when they run
    // out. Everything else keeps the original suppression.
    if (g_cfg.reload_suppress_fire && s_reload != ReloadState::Idle) {
        if (!(g_cfg.slide_vr && slide_weapon_ok() && slide_chamber_ok())) return true;
        if (s_sl_chamber_left <= 0) return true;   // the one chambered round is spent (or never was)
    }
    // THE SLIDE: no shot with the chamber open. Dead while the slide is pulled more than a
    // third of its travel, and dead from a reload's seat until the rack completes.
    // Truly empty never fires, whatever slidevr says: the phantom re-asserts one round every tick,
    // so a trigger left live with slidevr off fired it forever; the dry stop and the hidden reload
    // promise a dead trigger too.
    if (s_true_empty) return true;
    if (g_cfg.slide_vr) {
        if (s_sl_lock_pending) return true;
        if (s_sl_held && g_slide_pull.load(std::memory_order_relaxed) > g_cfg.slide_travel * 0.33f) return true;
    }
    return false;
}

#include "features/reloadvr/Gesture_reload_state.inl"   // fork feature: reloadvr (per-weapon reload state)

// The reload half of the tick. Separate from the melee detector because it is a state machine over
// BUTTONS and ZONES, not a derivative over velocity -- sharing a function would only tangle them.
//
// The poses are NULLABLE. Both are needed to make progress, but the watchdog below must run even
// when they are missing -- losing tracking mid-gesture is one of the ways a player gets stuck, so
// that is the last moment to stop servicing the state machine.
static void reload_update(const Vec3* hand_r_p, const Vec3* head_p) {
    static bool s_reload_on = true;   // one release on the off edge (at a boot with reloadvr 0 there is nothing to release)
    if (!g_cfg.enabled || !g_cfg.reload_vr) {
        if (s_reload_on) { s_reload_on = false; reload_release_all("manual reload switched off"); }
        if (s_reload != ReloadState::Idle) set_state(ReloadState::Idle, "disabled");
        return;
    }
    s_reload_on = true;
    // The well marker and the slide exist only while the magazine is in hand; every other state
    // hides the one and resets the other.
    if (s_reload != ReloadState::MagHeld) {
        reload_well_marker_update(false, Vec3{});
        reload_slide_reset();
    }

    // WATCHDOG FIRST. Logged unconditionally rather than under reload_log, because the symptom it
    // explains -- the fire trigger going dead -- is one a player will otherwise report as the mod
    // being broken, and a line they already have beats a setting they have to be told to enable.
    if (reload_watchdog_expired()) {
        API::get()->log_info(
            "[Halo-CampE-UEVR] RELOAD TIMED OUT after %.1fs in %s -- gesture never completed, "
            "returning to IDLE and releasing the fire trigger. (reloadtimeout=0 disables this.)",
            (double)g_cfg.reload_timeout_s, state_name(s_reload));
        // The slide bookkeeping goes the way the weapon-swap cancel below clears it, so a timed-out
        // reload does not leave a lock pending on a gun the sim never emptied.
        s_sl_lock_pending = false; s_sl_reload_due = false; s_sl_pressed_early = false; s_sl_locked_back = false; s_sl_press_pending = false; s_sl_press_due_at = 0;
        set_state(ReloadState::Idle, "timed out");
        return;
    }

    const unsigned short btn = g_pad_buttons.load(std::memory_order_relaxed);
    const unsigned short rising = (unsigned short)(btn & ~s_prev_buttons);
    s_prev_buttons = btn;

    #include "features/reloadvr/Gesture_update_swap_cancel.inl"   // fork feature: reloadvr (swap cancel)

    // A completed TAP, not a rising edge. The rising edge cannot tell a reload from the start of
    // an Interact hold, and treating them alike is what cost a chapter of vehicles.
    const bool reload_pressed = s_reload_tap.exchange(false, std::memory_order_relaxed);
    // THE GRIP: UEVR's per-hand ACTION, with the XInput mask kept only as a fallback. The mask
    // (reloadgrip, default 0x0100 = left shoulder) was the ONLY path here, and whether a physical
    // grip produces that bit depends on the controller and the runtime -- the plugin even ships a
    // remap that puts left X on the same bit. Field report from another machine: reload stalled in
    // MAG_OUT with the trigger suppressed and two-handing never latched, together, because both
    // gated on it. Holsters never had this problem; they have always read the action.
    const bool fetch_right    = g_cfg.aim_left_hand;   // the hand that is NOT aiming fetches
    const bool grip_held      = holster_grip_held(fetch_right) ||
                                ((g_cfg.reload_grip_mask != 0) &&
                                 ((btn & (unsigned short)g_cfg.reload_grip_mask) != 0));

    // The LEFT hand is the one that fetches. If aim is left-handed the roles swap, so this asks
    // for "the hand that is not aiming" rather than hardcoding a side.
    const auto lidx = g_cfg.aim_left_hand ? API::VR::get_right_controller_index()
                                          : API::VR::get_left_controller_index();
    Vec3 hand_l{}; Quat lrot{};
    #include "features/reloadvr/Gesture_update_dead_pose.inl"   // fork feature: reloadvr (fetch hand pose)

    switch (s_reload) {
    case ReloadState::Idle:
        if (reload_pressed && weapon_in_list(g_cfg.reload_skip_weapons)) {
            // No magazine on this weapon (plasma rifle, plasma pistol, sentinel beam): nothing to
            // drop, nothing to seat, and the game's own reload does nothing to it either.
            if (g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD ignored: %s has no magazine", weapon_key().c_str());
        } else if (reload_pressed) {
            // Deliberately does NOT forward the press. The game is told to reload only when the
            // magazine goes in -- that deferral IS the feature.
            s_reload_weapon = weapon_key();   // whose magazine this is (see the swap-cancel above)
            s_sl_empty_at_drop = weapon_empty_now();   // decides whether the seat wants a rack
            s_sl_locked_back = s_sl_empty_at_drop;
            s_sl_chamber_left  = s_sl_empty_at_drop ? 0 : 1;   // the one round already chambered
            set_state(ReloadState::MagOut, "reload pressed, mag dropped");
        }
        break;

    case ReloadState::MagOut: {
        if (reload_pressed && g_cfg.reload_cancel) {
            set_state(ReloadState::Idle, "cancelled, mag re-seated");
            break;
        }
        if (!have_left) break;
        const Vec3& head = *head_p;   // have_left implies a head pose (see the gate above)
        // The grab. With the visible magazine on (and the holster tick alive to run its body
        // frame), the hand must take the MAG -- the point the mesh is drawn at, published by
        // Holster. Otherwise the legacy belt ring: below the head, near the body's vertical axis.
        // Y is up in this space -- the same convention quat_forward assumes for the VR frame.
        bool grab_ok;
        if (g_cfg.reload_mag != 0 && g_cfg.holster_enabled) {
            grab_ok = holster_mag_hand_in();
        } else {
            const float drop = head.y - hand_l.y;
            const float dx = hand_l.x - head.x, dz = hand_l.z - head.z;
            const float horiz = std::sqrt(dx * dx + dz * dz);
            grab_ok = drop >= g_cfg.reload_belt_drop && horiz <= g_cfg.reload_belt_radius;
        }
        // NOT WHILE BOTH HANDS ARE ON THE GUN. The two-handed hold and this state machine read
        // the SAME physical grip. They do not collide in code; they collide in the player's hand,
        // and a support hand that dips past the belt zone mid-hold would silently start a reload.
        //
        // Zone-disjointness rather than a new binding: you cannot pull a magazine with both hands
        // on the weapon, so the suppression is also what a player expects. bindtwohand exists for
        // anyone whose grip is genuinely double-booked.
        if (grip_held && grab_ok && !two_hand_latched()) {
            s_grab_y = hand_l.y;   // where the mag was picked up: the lift gate measures from here
            set_state(ReloadState::MagHeld, "grabbed from belt");
        }
        break;
    }

    case ReloadState::MagHeld: {
        if (!grip_held) {
            set_state(ReloadState::MagOut, "grip released, dropped it");
            break;
        }
        #include "features/reloadvr/Gesture_update_seat.inl"   // fork feature: reloadvr (well and seat)
        break;
    }
    }
}

// Melee-only. Split out so that turning melee off does not also tear down the reload machine --
// the two features share a tick for pose-reuse reasons, not because they are one feature.
static void melee_reset() {
    g_melee_hold_until.store(0, std::memory_order_relaxed);
    s_have_prev = false;
    s_vel = Vec3{0.0f, 0.0f, 0.0f};
    s_ext = 0.0f;
    s_in_swing = false;
    s_peak_speed = 0.0f;
    s_peak_ext = 0.0f;
    s_peak_reach = 0.0f;
}

void gesture_reset() {
    reload_state_on_reset();   // before anything below clears it: the state goes to its weapon
    s_sl_reload_due = false; s_sl_pressed_early = false; s_sl_locked_back = false; s_sl_press_pending = false; s_sl_press_due_at = 0;
    s_true_empty = false;
    sc_teardown("reset");
    sp_teardown("reset");
    // sp_teardown clears the rack flag for the rebuild mode, but the NATIVE part is not torn down
    // here and slide_part_tick only re-finds it on a weapon or component change: after a ride or a
    // cutscene the held gun read "no rack" (no lock-back at the seat, no phantom) until a swap.
    if (s_np_have && s_np_slide.get() != nullptr) g_slide_rack_found = true;
    // The two-handed hold rides along. It is latched on a button and blended into aim, so a
    // transition the player did not choose (kill switch, stick mode, a tracking stall) must drop it
    // too -- otherwise the aim stays blended toward a support hand nothing is tracking any more.
    if (two_hand_latched()) two_hand_reset("gesture reset");   // his reset logs; only drop a hold that exists
    palette_two_hand_reset();   // the palette weapon's own hold (armdriver mode 3); a no-op when idle
    melee_reset();
    // Releasing the reload state is not optional: leaving it in MAG_OUT would keep the trigger
    // suppressed with no way for the player to notice why. Same rule as the arm hide.
    g_reload_hold_until.store(0, std::memory_order_relaxed);
    if (s_reload != ReloadState::Idle) set_state(ReloadState::Idle, "gesture reset");
    s_prev_buttons = 0;
}

#include "features/meleeleft/Gesture_offhand.inl"   // fork feature: meleeleft (off-hand detector)

API::UObject* native_mag_mesh() { return native_mag_mesh_impl(); }

void gesture_update(float dt) {
    s_gest_dt = dt;
    // The deferred hold check runs BEFORE the gates, because the gates are exactly the states that
    // would swallow it (a melee into a vehicle, a death) and a check that only reports when nothing
    // went wrong is not a check.
    // The fork's manual reload family (reloadvr / slidevr, experimental, default off). Every tick below
    // is idle-cheap on its own, but several resolve the weapon actor by reflection, so with both
    // masters off none of them runs and this function is the author's again. The two restore
    // windows (anim rate, state hold) stay outside: a window armed before a switch-off must close.
    const bool fork_reload = g_cfg.reload_vr || g_cfg.slide_vr;
    reload_anim_rate_tick();
    reload_state_hold_tick();
    if (fork_reload) {
    mag_dump_probe();
    anim_dump_tick();
    anim_vars_tick();
    audio_dump_tick();
    ak_mute_tick();
    anim_var_set_tick();
    anim_seq_set_tick();
    wpn_ammo_dump_tick();
    ammo_seq_tick();
    mag_drop_tick();
    // The held weapon's Blam object index, for the sim-side node probe (every ~30 ticks).
    {
        static int s_n = 0;
        // Every tick while the reload state tracker runs: the phantom's object guard compares it.
        if ((++s_n % 30) == 0 || g_cfg.reload_state_id != 0) {
            int32_t idx = -1;
            if (auto* wpn = fp_weapon_actor()) {
                auto** pc = wpn->get_property_data<API::UObject*>(L"BlamObjectSynchronization");
                if (pc != nullptr && !IsBadReadPtr(pc, sizeof(void*)) && *pc != nullptr && !IsBadReadPtr(*pc, sizeof(void*))) {
                    auto* p = (*pc)->get_property_data<int32_t>(L"BlamObjectIndex");
                    if (p != nullptr && !IsBadReadPtr(p, sizeof(int32_t))) idx = *p;
                }
            }
            g_wpn_obj_index.store(idx, std::memory_order_relaxed);
        }
    }
    }   // fork_reload

    if (s_hold_check_at != 0 && now_ticks() >= s_hold_check_at) {
        s_hold_check_at = 0;
        if (g_cfg.melee_log) {
            float dy = g_desired_yaw.load(std::memory_order_relaxed) - s_hold_ref_yaw;
            while (dy > 180.0f) dy -= 360.0f;
            while (dy < -180.0f) dy += 360.0f;
            const float dp = g_desired_pitch.load(std::memory_order_relaxed) - s_hold_ref_pitch;
            API::get()->log_info("[Halo-CampE-UEVR] MELEE HOLD CHECK: armed=%s -- aim sits %+.1f yaw "
                                 "%+.1f pitch from where the swing began (0 = held on target)",
                                 s_hold_was_armed ? "yes" : "NO", dy, dp);
        }
    }

    // ---- GLOBAL STAND-DOWN. States the player did not ask to gesture in AT ALL, so both features
    // go down together and the reload machine is reset (which releases any fire suppression).
    //
    // stick mode covers vehicles, cutscenes and death; calibration means the player is holding
    // the controller still against a frozen reticle and any motion is measurement, not intent.
    //
    // melee_swing is deliberately NOT in this list any more. It used to be, which meant turning
    // melee off silently disabled VR reload and re-reset its state every tick -- two features
    // share this function only because they share two pose reads.
    if (!g_cfg.enabled ||
        g_aim_calibrating.load(std::memory_order_relaxed) ||
        g_stick_mode_active.load(std::memory_order_relaxed)) {
        gesture_reset();
        return;
    }

    // Melee follows the AIM hand: whichever hand holds the gun is the one you would hit with.
    const auto ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                          : API::VR::get_right_controller_index();

    // Grip pose, not aim pose. get_aim_pose() is documented in Plugin.cpp as producing
    // teleport-scale travel readings, which is exactly the signal this must not confuse for a
    // strike.
    //
    // BOTH poses are fetched before either feature runs, and a failure is no longer an early
    // return: reload_update() has to be called even when they are missing, because losing tracking
    // mid-gesture is one of the ways MAG_OUT used to latch and swallow the fire trigger for good.
    Vec3 pos{}; Quat rot{};
    Vec3 hpos{}; Quat hrot{};
    const bool have_hand = get_pose(ridx, &pos, &rot, /*use_aim=*/false);
    const bool have_head = get_pose(API::VR::get_hmd_index(), &hpos, &hrot, /*use_aim=*/false);
    const bool poses_ok  = have_hand && have_head;

    // Reload runs off the same two poses the melee detector uses, so it costs no extra reads. It is
    // driven from here rather than from its own tick entry for exactly that reason.
    reload_update(poses_ok ? &pos : nullptr, poses_ok ? &hpos : nullptr);
    #include "features/reloadvr/Gesture_ticks.inl"   // fork feature: reloadvr (reload ticks)

    // ---- MELEE ONLY from here down.
    if (!g_cfg.melee_swing) {
        melee_reset();
        return;
    }

    // A stalled or absurd dt turns a stationary hand into a teleport. Drop the history rather
    // than differentiate across the gap -- the next tick re-seeds cleanly.
    if (!(dt > 0.0f) || dt > 0.25f) {
        s_have_prev = false;
        return;
    }

    // The off hand reads its own poses, so it does not wait on the aim hand's below.
    offhand_melee_update(dt);

    // HEAD-RELATIVE, and the head pose is REQUIRED -- no fail-open here. Without it there is no
    // extension measurement at all, and the previous version's fallback (assume the gate passes)
    // is precisely how it ended up firing on fast aiming.
    if (!poses_ok) {
        s_have_prev = false;
        return;
    }

    // Subtracting the head removes walking, strafing and vehicle motion before any derivative is
    // taken: those move hand and head together, so they vanish from `rel` entirely.
    const Vec3  rel{pos.x - hpos.x, pos.y - hpos.y, pos.z - hpos.z};
    const float reach = std::sqrt(rel.x * rel.x + rel.y * rel.y + rel.z * rel.z);

    // ---- POSITION SANITY, checked before the derivative rather than after.
    // The velocity guard below catches the JUMP, but a session logged reach holding at 2.45-2.51 m
    // for several samples afterwards: the spike was caught and the position was still wrong. No
    // arm is that long, so treat an impossible reach as "tracking is lying" and refuse to build
    // any history on it.
    if (reach > g_cfg.melee_max_reach) {
        if (g_cfg.melee_log && s_have_prev) {
            API::get()->log_info("[Halo-CampE-UEVR] MELEE ignored impossible reach: %.2f m (cap %.2f)",
                                 reach, g_cfg.melee_max_reach);
        }
        s_have_prev = false;
        s_vel = Vec3{0.0f, 0.0f, 0.0f};
        s_ext = 0.0f;
        return;
    }

    if (!s_have_prev) {
        s_prev_rel = rel;
        s_prev_reach = reach;
        s_have_prev = true;
        return;
    }

    const Vec3 raw{(rel.x - s_prev_rel.x) / dt,
                   (rel.y - s_prev_rel.y) / dt,
                   (rel.z - s_prev_rel.z) / dt};
    const float ext_raw = (reach - s_prev_reach) / dt;   // radial: + is reaching away from the head
    s_prev_rel = rel;
    s_prev_reach = reach;

    const float a = ema_alpha(g_cfg.melee_tau_ms, dt);
    s_vel.x += (raw.x - s_vel.x) * a;
    s_vel.y += (raw.y - s_vel.y) * a;
    s_vel.z += (raw.z - s_vel.z) * a;
    s_ext   += (ext_raw - s_ext) * a;

    const float speed = std::sqrt(s_vel.x * s_vel.x + s_vel.y * s_vel.y + s_vel.z * s_vel.z);

    // ---- TRACKING DISCONTINUITY. A 26 m/s "swing" was logged in the first session; no arm does
    // that. Drop the history rather than merely refusing to fire, because the sample AFTER a
    // teleport is derived from the same bad position and would be garbage too.
    if (speed > g_cfg.melee_max_speed) {
        if (g_cfg.melee_log) {
            API::get()->log_info("[Halo-CampE-UEVR] MELEE ignored tracking spike: %.1f m/s (cap %.1f)",
                                 speed, g_cfg.melee_max_speed);
        }
        s_have_prev = false;
        s_vel = Vec3{0.0f, 0.0f, 0.0f};
        s_ext = 0.0f;
        return;
    }

    // ---- LEGACY FORWARD GATE. Off by default (melee_fwd = 0); see Config.hpp for why it is
    // unsound. Retained only so an existing config that sets it keeps working.
    float along = 1.0f;
    if (g_cfg.melee_fwd > 0.0f && speed > 1e-3f) {
        const Vec3 fwd = quat_forward(hrot);
        const float fl = std::sqrt(fwd.x * fwd.x + fwd.y * fwd.y + fwd.z * fwd.z);
        if (fl > 1e-4f) {
            const float d = (s_vel.x * fwd.x + s_vel.y * fwd.y + s_vel.z * fwd.z) / (speed * fl);
            along = std::fabs(d);
        }
    }

    // ---- SWING SEGMENTATION, for the log only. Peaks are tracked on EXTENSION rate, since that
    // is now the discriminator -- reach is sampled at that same instant so the log line describes
    // one coherent moment rather than three unrelated maxima.
    // Each peak tracked INDEPENDENTLY. Tying peak reach to the instant extension peaked produced
    // "ext=0.00 reach=0.00" lines whenever extension never went positive -- a hand at zero
    // distance from the head, which is impossible and made the log actively misleading.
    if (speed > REST_SPEED_MPS) {
        // The aim reference is taken on the FIRST tick of the swing -- where you were pointing
        // when you decided to hit something, which is the direction Halo's melee should lunge in.
        if (!s_in_swing) {
            s_swing_yaw0   = g_desired_yaw.load(std::memory_order_relaxed);
            s_swing_pitch0 = g_desired_pitch.load(std::memory_order_relaxed);
            s_swing_aim_ok = true;
            s_swing_ctrl_ok = derive_ctrl_angles(&s_swing_ctrl_yaw, &s_swing_ctrl_pitch);
        }
        s_in_swing = true;
        if (speed > s_peak_speed) s_peak_speed = speed;
        if (s_ext  > s_peak_ext)  s_peak_ext   = s_ext;
        if (reach  > s_peak_reach) s_peak_reach = reach;
    } else if (s_in_swing) {
        s_in_swing = false;
        if (g_cfg.melee_log) {
            const bool would = (s_peak_speed >= g_cfg.melee_speed) &&
                               (s_peak_ext   >= g_cfg.melee_ext)   &&
                               (s_peak_reach >= g_cfg.melee_reach);
            API::get()->log_info(
                "[Halo-CampE-UEVR] MELEE swing  speed=%.2f  ext=%.2f  reach=%.2f   "
                "(need spd>=%.2f ext>=%.2f reach>=%.2f) -- %s",
                s_peak_speed, s_peak_ext, s_peak_reach,
                g_cfg.melee_speed, g_cfg.melee_ext, g_cfg.melee_reach,
                would ? "FIRED" : "no");
        }
        s_peak_speed = 0.0f;
        s_peak_ext = 0.0f;
        s_peak_reach = 0.0f;
    }

    // ---- TRIGGER. Extension is the test; speed is a floor; reach proves the arm is actually out.
    const long long now = now_ticks();
    if (now < s_cooldown_until)    return;
    if (speed < g_cfg.melee_speed) return;
    if (s_ext < g_cfg.melee_ext)   return;
    if (reach < g_cfg.melee_reach) return;
    if (along < g_cfg.melee_fwd)   return;
    // HOLSTER VETO: a reach over the shoulder is a strike to this detector. Stand down while the
    // hand is in or near a holster zone and briefly after any holster action.
    if (holster_melee_veto()) {
        ++s_veto_count;
        if (g_cfg.melee_log) {
            // WHICH veto, and by how much. Proximity covers holster_radius + margin around all
            // FIVE zones at once, which is a large volume of the space directly in front of the
            // chest -- exactly where a punch travels. If strikes are dying here, the number says
            // so and says by what margin.
            API::get()->log_info("[Halo-CampE-UEVR] MELEE VETOED (#%d) by %s: nearest zone %.3f m (veto radius %.3f) speed=%.2f ext=%.2f reach=%.2f",
                                 s_veto_count,
                                 holster_veto_by_proximity() ? "PROXIMITY" : "recent-action/grenade",
                                 holster_nearest_dist(),
                                 g_cfg.holster_radius + g_cfg.holster_melee_margin,   // holstermeleemargin
                                 speed, s_ext, reach);
        }
        s_cooldown_until = now + ms_to_ticks(150);
        return;
    }

    g_melee_hold_until.store(now + ms_to_ticks(g_cfg.melee_hold_ms), std::memory_order_relaxed);

    // ---- AIM THE STRIKE ALONG THE SWING (melee_aim_mode 1, shipped).
    //
    // Halo lunges along the AIM, and during a swing the aim IS the flailing hand -- so a strike at
    // something you were looking straight at lands wherever the hand happened to be pointing. The
    // hold pins the aim to the swing DIRECTION for melee_aim_hold_ms, then ramps back.
    //
    // Yaw is atan2(vx, -vz) in the same convention the rest of this file uses, plus the turn offset
    // so the direction is expressed in the frame the aim path consumes. Pitch is the velocity's
    // elevation, clamped before asin because a normalised-looking ratio can still land at 1.0000001
    // and produce a NaN that would poison the aim for the rest of the session.
    if (g_cfg.melee_aim_mode == 1 && g_cfg.melee_aim_hold_ms > 0 && speed > 0.0001f) {
        const float hy = wrap180(std::atan2(s_vel.x, -s_vel.z) * RAD2DEG
                                 + g_cfg.aim_turn * g_turn_offset.load(std::memory_order_relaxed));
        const float ratio = s_vel.y / speed;
        const float hp = std::asin(std::fmax(-1.0f, std::fmin(1.0f, ratio))) * RAD2DEG;
        g_melee_aim_ctrl_yaw.store(hy, std::memory_order_relaxed);
        g_melee_aim_ctrl_pitch.store(hp, std::memory_order_relaxed);
        g_melee_aim_hold_until.store(now + ms_to_ticks(g_cfg.melee_aim_hold_ms),
                                     std::memory_order_relaxed);
    }
    s_cooldown_until = now + ms_to_ticks(g_cfg.melee_cooldown_ms);

    // PIN THE AIM BACK to where it was when the swing began, for as long as the game needs to
    // resolve the strike. The snap is instantaneous rather than servo-limited because aimdirect is
    // the live path (confirmed driving in the same session's log): the rotator is assigned, not
    // steered, so there is no turn rate to wait on. With the stick loop it would degrade to a fast
    // turn instead of a jump, which is worse but not wrong.
    // Choose what the hold points at. Mode 1: the swing itself. s_vel is the hand-minus-head
    // velocity in the VR WORLD frame -- the same frame derive_ctrl_angles extracts its angles
    // from -- and `speed` has already cleared melee_speed, so the direction is well-defined.
    // Locomotion is subtracted by construction (rel = hand - head), so this is the punch's own
    // travel, not the player walking.
    float hy = s_swing_ctrl_yaw, hp = s_swing_ctrl_pitch;
    bool  hold_src_ok = s_swing_ctrl_ok;
    if (g_cfg.melee_aim_mode == 1) {
        hy = wrap180(std::atan2(s_vel.x, -s_vel.z) * RAD2DEG
                     + g_cfg.aim_turn * g_turn_offset.load(std::memory_order_relaxed));
        hp = std::asin(std::fmax(-1.0f, std::fmin(1.0f, s_vel.y / speed))) * RAD2DEG;
        hold_src_ok = true;
    }
    s_hold_was_armed = (g_cfg.melee_aim_hold_ms > 0 && hold_src_ok);
    if (s_hold_was_armed) {
        g_melee_aim_ctrl_yaw.store(hy, std::memory_order_relaxed);
        g_melee_aim_ctrl_pitch.store(hp, std::memory_order_relaxed);
        g_melee_aim_hold_until.store(now + ms_to_ticks(g_cfg.melee_aim_hold_ms),
                                     std::memory_order_relaxed);
    }
    // Read back from inside the window rather than at its edges: 80 ms in, the hold is at full
    // weight and the ramp has not begun, so the number is the hold's own doing and nothing else's.
    // The reference is the INTENDED setpoint -- the same reference-plus-delta arithmetic the hold
    // consumers apply -- so "0 = held on target" stays true in both modes. Comparing against the
    // swing's start was only correct when the start WAS the target.
    s_hold_ref_yaw   = s_hold_was_armed
                     ? g_ref_aim_yaw.load(std::memory_order_relaxed)
                       + wrap180(hy - g_ref_ctrl_yaw.load(std::memory_order_relaxed))
                     : s_swing_yaw0;
    s_hold_ref_pitch = s_hold_was_armed
                     ? g_ref_aim_pitch.load(std::memory_order_relaxed)
                       + (hp - g_ref_ctrl_pitch.load(std::memory_order_relaxed))
                     : s_swing_pitch0;
    s_hold_check_at = now + ms_to_ticks(80);

    if (g_cfg.melee_log) {
        // HOW FAR THE AIM MOVED during the swing. The synthesised button is byte-identical to the
        // stick click, so if one connects and the other does not, the difference is not the press
        // -- it is the state of the game at the moment of the press, and aim is the obvious part
        // of that state which the swing itself changes. Degrees, since the swing began.
        float dyaw = 0.0f, dpitch = 0.0f;
        if (s_swing_aim_ok) {
            dyaw = g_desired_yaw.load(std::memory_order_relaxed) - s_swing_yaw0;
            while (dyaw > 180.0f) dyaw -= 360.0f;
            while (dyaw < -180.0f) dyaw += 360.0f;
            dpitch = g_desired_pitch.load(std::memory_order_relaxed) - s_swing_pitch0;
        }
        API::get()->log_info("[Halo-CampE-UEVR] MELEE FIRED: speed=%.2f ext=%.2f reach=%.2f "
                             "mask=0x%04X hold=%dms | AIM MOVED yaw %+.1f deg pitch %+.1f deg "
                             "since the swing began | mode=%d swing-dir=(y%+.1f p%+.1f)",
                             speed, s_ext, reach, (unsigned)g_cfg.melee_mask, g_cfg.melee_hold_ms,
                             dyaw, dpitch, g_cfg.melee_aim_mode,
                             g_cfg.melee_aim_mode == 1 ? hy : s_swing_ctrl_yaw,
                             g_cfg.melee_aim_mode == 1 ? hp : s_swing_ctrl_pitch);
    }
}

} // namespace halo
