// Live controller settings. See GameSettings.hpp for the mechanism and why it is safe.

#include "GameSettings.hpp"
#include "Config.hpp"
#include "MotionAimControl.hpp"
#include "UeObject.hpp"
#include "uevr/API.hpp"

#include <cstdint>
#include <string>

using namespace uevr;

namespace halo {

float g_invert_cancel_x = 1.0f;
float g_invert_cancel_y = 1.0f;

namespace {

API::UObject* g_settings = nullptr;

// The player's own values, captured the FIRST time we resolve the object -- before we have changed
// anything. Everything we restore comes from here, so a mis-ordered transition can never "restore"
// one of our own values as though it were theirs.
bool  g_have_user = false;
int   g_user_sens_h = 0, g_user_sens_v = 0;
float g_user_axial = 0.0f, g_user_radial = 0.0f;
int   g_user_accel = -1;

bool  g_ours_applied = false;   // are OUR values currently in force?

// Several MeteoriteGameUserSettings instances exist (menu working copies). The earliest-created is
// the real singleton; transient object indices count DOWN, so the HIGHEST index was created first.
API::UObject* resolve_settings() {
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return nullptr;

    API::UObject* best = nullptr;
    long long best_idx = -1;
    const int32_t n = arr->get_object_count();
    for (int32_t i = 0; i < n; ++i) {
        auto* o = arr->get_object(i);
        if (o == nullptr) continue;
        if (class_name_of(o) != L"MeteoriteGameUserSettings") continue;
        const auto* fn = o->get_fname();
        if (fn == nullptr) continue;
        const std::string nm = narrow(fn->to_string());
        const auto us = nm.rfind('_');
        long long idx = 0;
        if (us != std::string::npos) { try { idx = std::stoll(nm.substr(us + 1)); } catch (...) { idx = 0; } }
        if (idx > best_idx) { best_idx = idx; best = o; }
    }
    return best;
}

template <typename T>
T read_field(API::UObject* o, const wchar_t* name, T fallback) {
    if (o == nullptr) return fallback;
    auto* p = o->get_property_data<T>(name);
    return (p != nullptr) ? *p : fallback;
}

void call_void(API::UObject* o, const wchar_t* fn) {
    if (o == nullptr) return;
    // An OVERSIZED ZEROED frame even for a no-arg call. This is the same discipline the reticule
    // code already uses: we are not modelling the exact parameter block, and a function that turns
    // out to take a hidden return slot would otherwise write past whatever we passed.
    uint8_t frame[32]{};
    o->call_function(fn, frame);
}

// Enum params are byte-sized in the UFunction frame; float params are 4 bytes. One oversized zeroed
// frame covers both without hand-modelling each signature.
void call_one_arg(API::UObject* o, const wchar_t* fn, const void* arg, size_t argSize) {
    if (o == nullptr || arg == nullptr) return;
    uint8_t frame[32]{};
    if (argSize > sizeof(frame)) return;
    memcpy(frame, arg, argSize);
    o->call_function(fn, frame);
}

void apply_sens_and_dz(int sensH, int sensV, float axial, float radial, int accel) {
    if (g_settings == nullptr) return;
    const uint8_t h = (uint8_t)sensH, v = (uint8_t)sensV;
    call_one_arg(g_settings, L"SetControllerLookSensitivityHorizontal", &h, 1);
    call_one_arg(g_settings, L"SetControllerLookSensitivityVertical",   &v, 1);

    // LOOK ACCELERATION. Left untouched until now, which quietly invalidated the plant model:
    // the measured 363.6 deg/s constant was taken at "acceleration min", but the saved user value
    // is LookAcceleration5, so rate depends on how long the stick has been held. A stateful plant
    // cannot be inverted, and prior Halo VR art (Halo-MCC-VR) says outright to turn it off.
    // accel < 0 means "leave the player's setting alone".
    if (accel >= 0) {
        const uint8_t a = (uint8_t)accel;
        call_one_arg(g_settings, L"SetControllerLookAcceleration", &a, 1);
    }
    call_one_arg(g_settings, L"SetControllerLookAxialDeadZone",  &axial,  sizeof(float));
    call_one_arg(g_settings, L"SetControllerLookRadialDeadZone", &radial, sizeof(float));
    // The setters alone do nothing -- writing the property never reaches the aim path. Apply is
    // what pushes the values into the running game. Save is deliberately NOT called.
    call_void(g_settings, L"ApplyHaloUserSettings");
}

} // namespace

// The settings object EXISTS long before it holds the player's settings: at plugin init it still
// carries defaults, and the saved values land later. Capturing then records 30 / 12% and calls them
// "the player's", so every restore would quietly set a stranger's preferences onto their session.
// That happened -- the first run of this captured sens=6 dz=12 for a player whose settings are 18
// and 0.
//
// Gameplay is the reliable gate: reaching a gameplay player controller is well after the settings
// load, whereas any fixed delay is a guess about load time on someone else's machine.
bool settings_are_loaded() {
    // g_aim_law_armed rather than a reflection call on the PlayerController. UEVR's reflection
    // ACCESS-VIOLATES on this game's Blam objects -- that is why aim is read by raw memory at a
    // validated offset -- and class_name_of(pc) is precisely that forbidden call. It appeared to
    // work here only because this runs once, before the capture succeeds; the same pattern in
    // AimWatch ran every tick and destabilised level transitions.
    //
    // The flag is set only once the loop is genuinely live in gameplay, which is a strictly better
    // signal than "the controller class is not the frontend one" anyway.
    return g_aim_law_armed.load();
}

void game_settings_tick(bool in_stick_mode) {
    // NOTHING HERE MAY RUN OUTSIDE GAMEPLAY. resolve_settings() walks the whole ~296k UObject array
    // calling class_name_of on every entry; doing that while a level streams is both the sweep this
    // project bans and a walk over objects that are still being constructed. Gate first, work second.
    if (!settings_are_loaded()) { g_settings = nullptr; return; }

    // Re-resolve on a SLOW cadence rather than validating the cached pointer, because validating it
    // means dereferencing it -- and if the object was freed by a level transition, that dereference
    // IS the crash. Re-resolving looks the address up in the live object array instead, which is
    // safe by construction. The full walk is the expensive part, so it is rationed.
    static int s_since_resolve = 0;
    if (g_settings == nullptr || ++s_since_resolve >= 30) {
        s_since_resolve = 0;
        g_settings = resolve_settings();
        if (g_settings == nullptr) return;
    }

    if (!g_have_user) {
        // Nothing may be applied before a TRUSTWORTHY capture, or there is no correct value to put
        // back. Waiting costs a few seconds of the player's own settings being in force, which is
        // the safe direction to fail.
        if (!settings_are_loaded()) return;
        g_user_sens_h = (int)read_field<uint8_t>(g_settings, L"ControllerLookSensitivityHorizontal", 6);
        g_user_sens_v = (int)read_field<uint8_t>(g_settings, L"ControllerLookSensitivityVertical",   6);
        g_user_axial  = read_field<float>(g_settings, L"ControllerLookAxialDeadZone",  0.125f);
        g_user_radial = read_field<float>(g_settings, L"ControllerLookRadialDeadZone", 0.125f);
        g_user_accel  = (int)read_field<uint8_t>(g_settings, L"ControllerLookAcceleration", 0);
        g_have_user = true;
        API::get()->log_info("[Halo-CampE-UEVR] game settings captured (in gameplay): sens=%d/%d dz=%.3f/%.3f accel=%d",
                             g_user_sens_h, g_user_sens_v, g_user_axial, g_user_radial, g_user_accel);
    }

    // INVERSION -- cancelled in our own output, never written to the game. In stick mode we cancel
    // nothing, so the player's inverted look applies to vehicles exactly as they configured it.
    if (in_stick_mode) {
        g_invert_cancel_x = 1.0f;
        g_invert_cancel_y = 1.0f;
    } else {
        g_invert_cancel_x = read_field<bool>(g_settings, L"bControllerInvertX", false) ? -1.0f : 1.0f;
        g_invert_cancel_y = read_field<bool>(g_settings, L"bControllerInvertY", false) ? -1.0f : 1.0f;
    }

    // SENSITIVITY / DEAD ZONE -- only these need the game's own setters, and only on a CHANGE of
    // mode. Re-applying every tick would spam ApplyHaloUserSettings for no reason; this codebase
    // already treats "apply a live tunable on change, not per frame" as the rule.
    if (!g_have_user) return;
    const bool want_ours = !in_stick_mode &&
                           (g_cfg.vr_sens > 0 || g_cfg.vr_deadzone >= 0.0f || g_cfg.vr_accel >= 0);
    if (want_ours == g_ours_applied) return;

    if (want_ours) {
        const int   s  = (g_cfg.vr_sens > 0) ? g_cfg.vr_sens : g_user_sens_h;
        const float dz = (g_cfg.vr_deadzone >= 0.0f) ? g_cfg.vr_deadzone : g_user_axial;
        apply_sens_and_dz(s, s, dz, dz, g_cfg.vr_accel);
    } else {
        apply_sens_and_dz(g_user_sens_h, g_user_sens_v, g_user_axial, g_user_radial, g_user_accel);
    }
    g_ours_applied = want_ours;
    // READ BACK. If SetControllerLookAcceleration is not the real UFunction name the call is a
    // silent no-op, which would look exactly like success. Verify rather than assume.
    const int accel_now = (int)read_field<uint8_t>(g_settings, L"ControllerLookAcceleration", 255);
    API::get()->log_info("[Halo-CampE-UEVR] controller settings -> %s | accel now=%d (user had %d)",
                         want_ours ? "VR (motion aim)" : "player's own (stick mode)",
                         accel_now, g_user_accel);
}

void game_settings_restore() {
    if (g_settings != nullptr && g_have_user && g_ours_applied) {
        apply_sens_and_dz(g_user_sens_h, g_user_sens_v, g_user_axial, g_user_radial, g_user_accel);
        g_ours_applied = false;
    }
    g_settings = nullptr;
}

} // namespace halo
