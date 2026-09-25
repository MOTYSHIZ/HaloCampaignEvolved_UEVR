// core reload engine: the reload's own haptic pulses, and the game's rumble held off while its
// reload runs hidden under the manual one.
//
// WHY THE RUMBLE CANNOT BE ZEROED IN THE CALLBACK (UEVR source 1.05+190, XInputHook.cpp and Mods.cpp).
// The XInputSetState hook calls the real function FIRST, then every mod in order: FrameworkConfig, VR,
// UObjectHook, PluginLoader. VR::on_xinput_set_state turns the motor speeds into a 0.1 s controller
// pulse, so by the time a plugin sees the vibration the pulse is already on the controllers. Three
// independent ways past that, one bit each of reloadrumblemute:
//   1  REPLACE: an OpenXR haptic applied to a hand replaces the one running there, so right after
//      UEVR's pulse we apply our own: the rest of our current reload pulse on that hand, or silence.
//   2  SOURCE:  the player controller's ForceFeedbackScale is 0 while the window is open, so the
//      game's force feedback asks for nothing (only if the rumble goes through the engine's force
//      feedback; the log says whether the property was found and whether rumble still arrived).
//   4  STOP:    ClientStopForceFeedback(None, None) every tick of the window, which ends every force
//      feedback effect the controller is playing.
// THE WINDOW is armdriver 2's gun hold window (reload_hold_window): the rumble is held off exactly
// while the game's reload animation is, and one published level serves both.

#include "core/reload/ReloadEngine.hpp"
#include "Config.hpp"
#include "uevr/API.hpp"

#include <Windows.h>
#include <Xinput.h>

#include <atomic>
#include <chrono>
#include <cstdint>

using uevr::API;

namespace halo {

namespace {

long long steady_now() { return std::chrono::steady_clock::now().time_since_epoch().count(); }
long long steady_ms(double ms) {
    return (long long)(ms * (double)std::chrono::steady_clock::period::den / (1000.0 * (double)std::chrono::steady_clock::period::num));
}

// The pulse each hand is playing, for the replace to put back. 0 = left, 1 = right.
std::atomic<long long> s_pulse_end[2]{};
std::atomic<float>     s_pulse_amp[2]{};

std::atomic<bool>      s_mute{false};
std::atomic<uint32_t>  s_rumble_calls{0};
std::atomic<uint32_t>  s_rumble_replaced{0};
std::atomic<uint32_t>  s_rumble_peak_l{0};
std::atomic<uint32_t>  s_rumble_peak_r{0};
std::atomic<long long> s_rumble_first{0};
long long s_mute_since = 0;
bool      s_rumble_on_prev = false;   // XInput thread only: the previous call's motors were running

UEVR_InputSourceHandle hand_source(int hand) {
    return hand != 0 ? API::VR::get_right_joystick_source() : API::VR::get_left_joystick_source();
}
void apply(int hand, float seconds, float amp) {
    API::VR::trigger_haptic_vibration(0.0f, seconds, 0.0f, amp, hand_source(hand));
}

// SOURCE (bit 2): the controller the scale was written on and its own value, to put back.
API::UObject* s_ff_pc = nullptr;
float         s_ff_orig = 1.0f;
bool          s_ff_written = false;
bool          s_ff_missing_logged = false;

void ff_restore() {
    if (!s_ff_written) return;
    s_ff_written = false;
    auto* pc = API::get()->get_player_controller(0);
    if (pc == nullptr || pc != s_ff_pc) return;   // a new controller never saw our 0
    if (auto* p = pc->get_property_data<float>(L"ForceFeedbackScale")) *p = s_ff_orig;
}
void ff_hold() {
    auto* pc = API::get()->get_player_controller(0);
    if (pc == nullptr) return;
    auto* p = pc->get_property_data<float>(L"ForceFeedbackScale");
    if (p == nullptr) {
        if (!s_ff_missing_logged) {
            s_ff_missing_logged = true;
            API::get()->log_info("[Halo-CampE-UEVR] RUMBLE: the player controller has no ForceFeedbackScale, the source mute (reloadrumblemute 2) does nothing here");
        }
        return;
    }
    if (!s_ff_written || pc != s_ff_pc) { s_ff_pc = pc; s_ff_orig = *p; s_ff_written = true; }
    *p = 0.0f;
}
void ff_stop() {
    auto* pc = API::get()->get_player_controller(0);
    if (pc == nullptr) return;
    struct { API::UObject* effect; uint64_t tag; uint8_t pad[16]; } params{};   // (None, None): every effect
    pc->call_function(L"ClientStopForceFeedback", &params);
}
}  // namespace

int reload_gun_hand() { return g_cfg.aim_left_hand ? 0 : 1; }

void reload_haptic_raw(int hand, float seconds, float amp) {
    if (hand < 0 || hand > 1 || seconds <= 0.0f || amp <= 0.0f) return;
    s_pulse_amp[hand].store(amp, std::memory_order_relaxed);
    s_pulse_end[hand].store(steady_now() + steady_ms(seconds * 1000.0), std::memory_order_release);
    apply(hand, seconds, amp);
}

void reload_haptic_step(const char* step, int hands, float seconds, float amp) {
    if (g_cfg.reload_haptic == 0) return;
    if (g_cfg.reload_haptic_ms > 0) seconds = (float)g_cfg.reload_haptic_ms / 1000.0f;
    amp *= g_cfg.reload_haptic_amp;
    if (amp > 1.0f) amp = 1.0f;
    if (amp <= 0.0f || seconds <= 0.0f) return;
    if (hands & 1) reload_haptic_raw(0, seconds, amp);
    if (hands & 2) reload_haptic_raw(1, seconds, amp);
    if (g_cfg.reload_vr_log)
        API::get()->log_info("[Halo-CampE-UEVR] HAPTIC %s: %s, %.0f ms at %.2f", step,
                             hands == 3 ? "both hands" : (hands & 2) ? "right hand" : "left hand", seconds * 1000.0f, amp);
}

void reload_rumble_tick() {
    const int mode = g_cfg.reload_rumble_mute;
    const bool want = mode != 0 && reload_hold_window().busy;
    const bool was = s_mute.load(std::memory_order_relaxed);
    const bool log = g_cfg.reload_vr_log || g_cfg.reload_rumble_log;
    if (want != was) {
        const long long nowt = steady_now();
        const double per_ms = (double)steady_ms(1.0);
        if (want) {
            s_rumble_calls.store(0); s_rumble_replaced.store(0); s_rumble_peak_l.store(0); s_rumble_peak_r.store(0); s_rumble_first.store(0);
            s_mute_since = nowt;
            if (log) API::get()->log_info("[Halo-CampE-UEVR] RUMBLE mute engages (mode %d)", mode);
        } else {
            ff_restore();
            if (log) {
                const long long first = s_rumble_first.load();
                API::get()->log_info("[Halo-CampE-UEVR] RUMBLE mute released after %.0f ms: %u game rumble calls held off (first at +%.0f ms, peak L %u R %u), %u replaced",
                                     (double)(nowt - s_mute_since) / per_ms, s_rumble_calls.load(),
                                     first != 0 ? (double)(first - s_mute_since) / per_ms : -1.0,
                                     s_rumble_peak_l.load(), s_rumble_peak_r.load(), s_rumble_replaced.load());
            }
        }
        s_mute.store(want, std::memory_order_release);
    }
    if (!want) { if (s_ff_written) ff_restore(); return; }
    if (mode & 2) ff_hold(); else if (s_ff_written) ff_restore();
    if (mode & 4) ff_stop();
}

void reload_rumble_set_state(uint32_t user_index, void* vibration) {
    auto* v = static_cast<XINPUT_VIBRATION*>(vibration);
    if (v == nullptr) return;
    const bool on = v->wLeftMotorSpeed != 0 || v->wRightMotorSpeed != 0;
    const bool muting = s_mute.load(std::memory_order_acquire);
    if (g_cfg.reload_rumble_log && on && !s_rumble_on_prev)
        API::get()->log_info("[Halo-CampE-UEVR] RUMBLE game starts: pad %u, L %u R %u, %s", user_index,
                             (unsigned)v->wLeftMotorSpeed, (unsigned)v->wRightMotorSpeed, muting ? "held off" : "passed through");
    s_rumble_on_prev = on;
    if (!muting || !on) return;
    s_rumble_calls.fetch_add(1, std::memory_order_relaxed);
    long long zero = 0;
    s_rumble_first.compare_exchange_strong(zero, steady_now(), std::memory_order_relaxed);
    if (v->wLeftMotorSpeed > s_rumble_peak_l.load(std::memory_order_relaxed)) s_rumble_peak_l.store(v->wLeftMotorSpeed, std::memory_order_relaxed);
    if (v->wRightMotorSpeed > s_rumble_peak_r.load(std::memory_order_relaxed)) s_rumble_peak_r.store(v->wRightMotorSpeed, std::memory_order_relaxed);
    if (g_cfg.reload_rumble_mute & 1) {
        // REPLACE: our own pulse still running on a hand goes back on for what is left of it (the
        // seat's pulse lands at the same instant the game's reload rumbles), otherwise silence.
        const long long nowt = steady_now();
        for (int hand = 0; hand < 2; ++hand) {
            const long long left_ticks = s_pulse_end[hand].load(std::memory_order_acquire) - nowt;
            if (left_ticks > 0) apply(hand, (float)((double)left_ticks / (double)steady_ms(1000.0)), s_pulse_amp[hand].load(std::memory_order_relaxed));
            else                apply(hand, 0.01f, 0.0f);
        }
        s_rumble_replaced.fetch_add(1, std::memory_order_relaxed);
    }
    // Anything after the plugins (the Lua scripts) sees no rumble either.
    v->wLeftMotorSpeed = 0;
    v->wRightMotorSpeed = 0;
}

}  // namespace halo
