#include "Gesture.hpp"
#include "TwoHandAim.hpp"
#include "Holster.hpp"

#include "BlamPalette.hpp"
#include "PaletteTwoHand.hpp"   // palette_two_hand_reset / g_th_latched: the palette weapon mode hold
#include "WristHud.hpp"           // g_wristhud_hide_cradle: the hidden reload hides the ammo cradle
#include "Config.hpp"
#include "ForceTube.hpp"          // g_ft_fire_at: the off hand stands down while the stock kicks
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

// ---- THE WEAPON'S OWN MAGAZINE -----------------------------------------------------------------
// The gun ships its magazine as a component on the weapon actor, so during MAG_OUT it can be
// genuinely removed instead of the rifle pretending nothing happened ("the magazine still appears
// to be in the assault rifle" -- field report). Found by substring on the component's class or
// object name (maghidename, default "Magazine"); magdump surveys the held weapon's components so
// the right substring can be read off rather than guessed.
struct FRawArrayRO_ { void* data; int32_t num; int32_t max; };

TrackedObject s_mag_hidden;   // the component we hid -- what release must undo

template <typename F>
void weapon_components(F&& fn) {
    auto* wpn = fp_weapon_actor();
    if (wpn == nullptr) return;
    for (const wchar_t* prop : {L"BlueprintCreatedComponents", L"InstanceComponents"}) {
        auto* arr = wpn->get_property_data<FRawArrayRO_>(prop);
        if (arr == nullptr || IsBadReadPtr(arr, sizeof(FRawArrayRO_))) continue;
        if (arr->data == nullptr || arr->num <= 0 || arr->num > 4096) continue;
        auto** elems = reinterpret_cast<API::UObject**>(arr->data);
        if (IsBadReadPtr(elems, sizeof(void*) * (size_t)arr->num)) continue;
        for (int32_t i = 0; i < arr->num; ++i) {
            auto* c = elems[i];
            if (c == nullptr || IsBadReadPtr(c, sizeof(void*))) continue;
            if (!fn(c)) return;
        }
    }
}

// The mesh ASSET a static-mesh component renders, lowercased -- the components themselves are
// anonymous (six numbered BPC_FP_StaticMesh_C on the assault rifle, field-surveyed), and the
// magazine's identity lives entirely in the assigned SM_*_Magazine asset.
std::wstring comp_mesh_lname(API::UObject* c) {
    auto** mesh = c->get_property_data<API::UObject*>(L"StaticMesh");
    if (mesh == nullptr || IsBadReadPtr(mesh, sizeof(void*))) return L"";
    auto* m = *mesh;
    if (m == nullptr || IsBadReadPtr(m, sizeof(void*))) return L"";
    std::wstring full = m->get_full_name();
    for (auto& ch : full) ch = (wchar_t)towlower(ch);
    return full;
}

void mag_set_visible(API::UObject* comp, bool visible) {
    if (comp == nullptr) return;
    alignas(16) uint8_t p[64] = {0};
    p[0] = visible ? 1 : 0;
    p[1] = 1;   // bPropagateToChildren -- the mag may carry child meshes
    comp->call_function(L"SetVisibility", p);
}

// How the component in s_mag_hidden was hidden (reload_state_hide), so the release undoes that
// and only that; the actor it was found on (compared, never dereferenced); its own scale.
int           s_mag_hid_mode = -1;
API::UObject* s_mag_hid_actor = nullptr;
double        s_mag_hid_scale[3] = {1.0, 1.0, 1.0};
long long     s_mag_find_at = 0;
uint32_t      s_mag_rehides = 0;

void mag_set_hidden_in_game(API::UObject* comp, bool hidden) {
    if (comp == nullptr) return;
    alignas(16) uint8_t p[64] = {0};
    p[0] = hidden ? 1 : 0;
    p[1] = 1;   // bPropagateToChildren
    comp->call_function(L"SetHiddenInGame", p);
}
void mag_set_rel_scale(API::UObject* comp, const double s[3]) {
    if (comp == nullptr) return;
    alignas(16) uint8_t p[64] = {0};
    memcpy(p, s, sizeof(double) * 3);   // FVector, three doubles
    comp->call_function(L"SetRelativeScale3D", p);
}
double* mag_rel_scale(API::UObject* comp) {
    auto* s = comp->get_property_data<double>(L"RelativeScale3D");
    if (s == nullptr || IsBadReadPtr(s, sizeof(double) * 3)) return nullptr;
    return s;
}
void mag_hide_comp(API::UObject* c, int mode, bool first) {
    if (mode == 2) { mag_set_hidden_in_game(c, true); return; }
    if (mode == 3) {
        if (first) {
            if (auto* s = mag_rel_scale(c))
                for (int i = 0; i < 3; ++i) s_mag_hid_scale[i] = (s[i] > 0.01) ? s[i] : 1.0;   // never capture our own 0.001
        }
        const double z[3] = {0.001, 0.001, 0.001};
        mag_set_rel_scale(c, z);
        return;
    }
    mag_set_visible(c, false);
}
void mag_unhide_comp(API::UObject* c, int mode) {
    if (c == nullptr) return;
    if (mode == 2) { mag_set_hidden_in_game(c, false); return; }
    if (mode == 3) { mag_set_rel_scale(c, s_mag_hid_scale); return; }
    mag_set_visible(c, true);
}

void mag_hide_apply(bool out) {
    if (!g_cfg.mag_hide || g_cfg.mag_hide_name[0] == 0) return;
    if (!out) {
        // Restore whatever we hid, wherever the weapon has since gone. A dead handle just means
        // the actor was torn down, which hid it more thoroughly than we ever could.
        if (auto* c = s_mag_hidden.get()) mag_unhide_comp(c, s_mag_hid_mode);
        s_mag_hidden = TrackedObject{};
        s_mag_hid_actor = nullptr; s_mag_hid_mode = -1;
        return;
    }
    if (s_mag_hidden.get() != nullptr) return;
    std::wstring want;
    for (const char* q = g_cfg.mag_hide_name; *q != 0; ++q)
        want.push_back((wchar_t)towlower((unsigned char)*q));
    const int mode = (g_cfg.reload_state_id == 0 || g_cfg.reload_state_hide == 0) ? 1 : g_cfg.reload_state_hide;
    weapon_components([&](API::UObject* c) {
        // Match on the component's class or name, OR on the ASSET its mesh renders -- the
        // components are anonymous and the magazine's name lives in the asset.
        const auto* fn = c->get_fname();
        std::wstring nm = (fn != nullptr) ? fn->to_string() : L"";
        std::wstring cl = class_name_of(c);
        for (auto& ch : nm) ch = (wchar_t)towlower(ch);
        for (auto& ch : cl) ch = (wchar_t)towlower(ch);
        const std::wstring mesh = comp_mesh_lname(c);
        // The classic assault rifle's asset is spelled "Megazine" (pak survey); the default name
        // finds it too.
        const bool mega = (want == L"magazine") && mesh.find(L"megazine") != std::wstring::npos;
        if (cl.find(want) == std::wstring::npos && nm.find(want) == std::wstring::npos &&
            mesh.find(want) == std::wstring::npos && !mega) return true;
        mag_hide_comp(c, mode, true);
        s_mag_hidden.set(c);
        s_mag_hid_mode = mode;
        s_mag_hid_actor = fp_weapon_actor();
        return false;   // first match wins
    });
}

// THE MAGAZINE STAYS OUT ON WHATEVER RENDERS THE WEAPON NOW (reload_state_hide). The hide used to
// land once, on one component, at the state change. A weapon swap spawns new components (Arms.cpp
// and Rig.cpp both record it), and the weapon actor is re-resolved only every ~60 ticks, so a swap
// away and back inside that window presents a fresh actor under the SAME weapon key: no state
// change, so no hide, and the fresh magazine is drawn while the machine still says MAG_OUT. The
// same holds for any rebuild of the first-person weapon (a seat, a respawn of the FP build).
void mag_hide_enforce() {
    if (!g_cfg.mag_hide || g_cfg.mag_hide_name[0] == 0) return;
    if (g_cfg.reload_state_id == 0 || g_cfg.reload_state_hide == 0) return;
    if (s_reload == ReloadState::Idle) return;
    auto* actor = fp_weapon_actor();
    if (actor == nullptr) return;   // nothing renders the weapon this tick
    const int mode = g_cfg.reload_state_hide;
    auto* c = s_mag_hidden.get();
    if (c != nullptr && (s_mag_hid_actor != actor || s_mag_hid_mode != mode)) {
        if (g_cfg.reload_state_log)
            API::get()->log_info("[Halo-CampE-UEVR] RSTATE maghide: %s, the old component is released and the magazine re-found",
                                 s_mag_hid_actor != actor ? "a different actor renders the weapon" : "the hide mode changed");
        mag_unhide_comp(c, s_mag_hid_mode);
        s_mag_hidden = TrackedObject{}; s_mag_hid_actor = nullptr; s_mag_hid_mode = -1;
        c = nullptr;
    }
    if (c == nullptr) {
        const long long nowt = now_ticks();
        if (nowt - s_mag_find_at < ms_to_ticks(250)) return;
        s_mag_find_at = nowt;
        s_mag_hidden = TrackedObject{};
        mag_hide_apply(true);
        if (g_cfg.reload_state_log) {
            auto* h = s_mag_hidden.get();
            API::get()->log_info("[Halo-CampE-UEVR] RSTATE maghide: %s on %ls (mode %d, state %d)",
                                 h ? "magazine hidden" : "NO magazine component found", class_name_of(actor).c_str(), mode, (int)s_reload);
            if (h != nullptr && mode == 1) {
                static bool s_said = false;
                if (!s_said) { s_said = true; API::get()->log_info("[Halo-CampE-UEVR] RSTATE maghide: IsVisible is %s on %ls", h->get_class()->find_function(L"IsVisible") ? "reflected" : "NOT reflected (re-hide cannot detect a re-show)", class_name_of(h).c_str()); }
            }
        }
        return;
    }
    bool rehid = false;
    if (mode == 1) {
        alignas(16) uint8_t p[64] = {0};
        c->call_function(L"IsVisible", p);
        if (p[0] != 0) { mag_set_visible(c, false); rehid = true; }
    } else if (mode == 2) {
        mag_set_hidden_in_game(c, true);
    } else if (mode == 3) {
        if (auto* s = mag_rel_scale(c)) { if (s[0] > 0.01 || s[1] > 0.01 || s[2] > 0.01) { mag_hide_comp(c, 3, false); rehid = true; } }
    }
    if (rehid) {
        ++s_mag_rehides;
        if (g_cfg.reload_state_log && (s_mag_rehides <= 20u || (s_mag_rehides % 100u) == 0u))
            API::get()->log_info("[Halo-CampE-UEVR] RSTATE maghide: the magazine was showing again on %ls, re-hidden (mode %d, %u so far)",
                                 class_name_of(actor).c_str(), mode, s_mag_rehides);
    }
}

// ---- THE WELL MARKER: a small ring at the insert point while the magazine is in hand. The
// seat test has always known where the well is; the player could not see it (a tester video,
// 2026-09-01: testers had no idea where the mag goes). TrackedObject + paced respawn, the same
// lifecycle every holster marker uses -- a level load recycles the component and the next
// MagHeld tick rebuilds it.
TrackedObject s_well_marker;
long long s_well_try_at = 0;
void reload_well_marker_update(bool show, const Vec3& world) {
    if (!g_cfg.reload_well_marker) show = false;
    auto* m = s_well_marker.get();
    if (!show) { if (m != nullptr) { holster_marker_show(m, false); marker_render_drop(m); } return; }
    if (m == nullptr) {
        const long long nowt = now_ticks();
        if (nowt - s_well_try_at < ms_to_ticks(2000)) return;
        s_well_try_at = nowt;
        auto* owner = API::get()->get_local_pawn(0);
        if (owner == nullptr) return;
        static const wchar_t* kWellRing[] = {
            L"StaticMesh /Engine/BasicShapes/Torus.Torus",
            L"StaticMesh /Engine/BasicShapes/Sphere.Sphere",
        };
        const double s = (double)g_cfg.reload_well_marker_scale;
        m = marker_spawn_list(owner, kWellRing, 2, s, s, s * 0.5);
        if (m == nullptr) return;
        marker_tint(m, g_cfg.well_marker_color);
        s_well_marker.set(m);
    }
    holster_marker_place(m, world);
    holster_marker_show(m, true);
    {   // the one solve: room-anchored like everything else, re-placed per frame (Markers.hpp)
        Vec3 hp{}; Quat hr{};
        if (get_pose(API::VR::get_hmd_index(), &hp, &hr, /*use_aim=*/false))
            marker_render_anchor(m, holster_world_to_room(world, hp));
    }
}

// The slide's clock: 0 = not sliding. Reset whenever the reload leaves MagHeld by any route.
long long s_slide_start = 0;
void reload_slide_reset() {
    s_slide_start = 0;
    g_reload_slide_t.store(-1.0f, std::memory_order_relaxed);
}

// MAGDUMP: the held weapon's component roster, one-shot on value change.
void mag_dump_probe() {
    static int s_armed_as = 0;
    if (g_cfg.mag_dump == s_armed_as) return;
    s_armed_as = g_cfg.mag_dump;
    if (s_armed_as == 0) return;
    auto* wpn = fp_weapon_actor();
    API::get()->log_info("[Halo-CampE-UEVR] MAGDUMP %d: weapon %ls",
                         s_armed_as, wpn ? class_name_of(wpn).c_str() : L"(none)");
    int n = 0;
    weapon_components([&](API::UObject* c) {
        const auto* fn = c->get_fname();
        const std::wstring mesh = comp_mesh_lname(c);
        API::get()->log_info("[Halo-CampE-UEVR] MAGDUMP   %ls '%ls'%s%ls",
                             class_name_of(c).c_str(),
                             (fn != nullptr) ? fn->to_string().c_str() : L"?",
                             mesh.empty() ? "" : "  mesh=",
                             mesh.c_str());
        return ++n < 64;
    });
    API::get()->log_info("[Halo-CampE-UEVR] MAGDUMP %d done: %d components", s_armed_as, n);
}

// ANIMDUMP: where does the reload animation come from? The palette hold wrote 89 frames over
// both render banks and the magazine still cycled (2026-09-02), so that motion is NOT in the
// Blam palette we own. For ~2.5 s after a seat, sample every skeletal component on the weapon
// and the arms rig: animation mode, anim-instance class, the active montage, IsPlaying. A
// montage name appearing here IS the answer, and the fix follows from it.
long long s_anim_dump_until = 0;
void anim_dump_tick() {
    if (s_anim_dump_until == 0) return;
    const long long nowt = now_ticks();
    if (nowt > s_anim_dump_until) {
        s_anim_dump_until = 0;
        API::get()->log_info("[Halo-CampE-UEVR] ANIMDUMP end");
        return;
    }
    static long long s_last = 0;
    if (nowt - s_last < ms_to_ticks(120)) return;
    s_last = nowt;
    auto report = [&](API::UObject* c, const char* who) {
        if (c == nullptr || IsBadReadPtr(c, sizeof(void*))) return;
        const std::wstring cl = class_name_of(c);
        // Audio components too -- the reload SOUND's home is the next question, and whether the
        // weapon actor owns an AudioComponent decides whether a local mute is even on the table.
        if (cl.find(L"Audio") != std::wstring::npos) {
            const auto* fn = c->get_fname();
            API::get()->log_info("[Halo-CampE-UEVR] ANIMDUMP %s AUDIO %ls '%ls'", who, cl.c_str(),
                                 fn ? fn->to_string().c_str() : L"?");
            return;
        }
        if (cl.find(L"Skeletal") == std::wstring::npos && cl.find(L"Skinned") == std::wstring::npos) return;
        int mode = -1;
        if (auto* pm = c->get_property_data<uint8_t>(L"AnimationMode"))
            if (!IsBadReadPtr(pm, 1)) mode = (int)*pm;
        API::UObject* animbp = nullptr;
        if (auto** pai = c->get_property_data<API::UObject*>(L"AnimScriptInstance"))
            if (!IsBadReadPtr(pai, sizeof(void*))) animbp = *pai;
        std::wstring aicl = L"-", mont = L"-";
        if (animbp != nullptr && !IsBadReadPtr(animbp, sizeof(void*))) {
            aicl = class_name_of(animbp);
            alignas(16) uint8_t p[64] = {0};
            animbp->call_function(L"GetCurrentActiveMontage", p);
            auto* m = *reinterpret_cast<API::UObject**>(p);
            if (m != nullptr && !IsBadReadPtr(m, sizeof(void*))) mont = m->get_full_name();
        }
        bool playing = false;
        { alignas(16) uint8_t p[64] = {0}; c->call_function(L"IsPlaying", p); playing = p[0] != 0; }
        float rate = -1.0f;
        if (auto* pr = c->get_property_data<float>(L"GlobalAnimRateScale"))
            if (!IsBadReadPtr(pr, sizeof(float))) rate = *pr;
        const auto* fn = c->get_fname();
        API::get()->log_info("[Halo-CampE-UEVR] ANIMDUMP %s %ls '%ls' mode=%d anim=%ls montage=%ls playing=%d rate=%.1f",
                             who, cl.c_str(), fn ? fn->to_string().c_str() : L"?",
                             mode, aicl.c_str(), mont.c_str(), (int)playing, rate);
    };
    weapon_components([&](API::UObject* c) { report(c, "weapon"); return true; });
    report(rig_tracked_component(), "arms");
}

// ---- FAST-FORWARD THE GAME'S RELOAD ANIMATION. ANIMDUMP (2026-09-02) settled where it lives:
// an Animation Blueprint on the weapon's skeletal mesh (ABP_FP_<Weapon>_Default_C) and one on
// the arms, no montage, nothing in the Blam palette. The player has just seated the magazine
// by hand; the arms then acting it out again is the wrong film, and freezing the pose was
// refused ("I don't want anything frozen"). So the animation is not stopped -- it is run at
// reload_anim_rate x for reload_anim_ms after the seat: GlobalAnimRateScale on every skeletal
// component of the weapon and the arms, restored to 1.0 when the window closes. The reload
// still happens, the ammo still lands, the animation is a blink. Components are remembered so
// the restore hits exactly what was touched, and a dead handle simply means the actor is gone.
TrackedObject s_anim_rate_comps[6];
int           s_anim_rate_n = 0;
long long     s_anim_rate_until = 0;
void reload_anim_rate_set(API::UObject* c, float rate) {
    if (c == nullptr || IsBadReadPtr(c, sizeof(void*))) return;
    auto* p = c->get_property_data<float>(L"GlobalAnimRateScale");
    if (p == nullptr || IsBadWritePtr(p, sizeof(float))) return;
    *p = rate;
}
// A bool UPROPERTY written through its bitfield mask -- bPauseAnims shares a byte with its
// neighbours, and a raw byte write would flip them too (the read side of this lesson is
// read_bool_prop in Plugin.cpp). Returns false when the class has no such property.
bool reload_set_bool_prop(API::UObject* obj, const wchar_t* name, bool value) {
    if (obj == nullptr || IsBadReadPtr(obj, sizeof(void*))) return false;
    auto* cls = obj->get_class();
    if (cls == nullptr) return false;
    auto* prop = cls->find_property(name);
    if (prop == nullptr) return false;
    static_cast<API::FBoolProperty*>(prop)->set_value_in_object(obj, value);
    return true;
}
// PAUSE THE ANIMATION UPDATE for the window (reloadpauseanim). The state clamp alone loses one
// frame every time: the game writes the reload state during its actor tick, after ours, and
// the AnimBP evaluates that frame before the next clamp -- measured "re-asserted 1 times" on
// every reload after a restart, and that single frame is what fires the Blam-side reload sound.
// bPauseAnims skips the animation update entirely, so the transition is never evaluated; by the
// time the update resumes the clamp has the variable back at idle. The gun still rides the hand
// (that is the component transform, not the animation).
void reload_anim_rate_begin() {
    const bool want_rate  = g_cfg.reload_anim_rate > 0.0f;
    const bool want_pause = g_cfg.reload_pause_anim;
    if ((!want_rate && !want_pause) || g_cfg.reload_anim_ms <= 0) return;
    s_anim_rate_n = 0;
    int paused = 0;
    auto grab = [&](API::UObject* c) {
        if (c == nullptr || s_anim_rate_n >= 6) return;
        const std::wstring cl = class_name_of(c);
        if (cl.find(L"Skeletal") == std::wstring::npos && cl.find(L"Skinned") == std::wstring::npos) return;
        if (want_rate)  reload_anim_rate_set(c, g_cfg.reload_anim_rate);
        if (want_pause && reload_set_bool_prop(c, L"bPauseAnims", true)) ++paused;
        s_anim_rate_comps[s_anim_rate_n++].set(c);
    };
    weapon_components([&](API::UObject* c) { grab(c); return s_anim_rate_n < 6; });
    grab(rig_tracked_component());
    s_anim_rate_until = now_ticks() + ms_to_ticks(g_cfg.reload_anim_ms);
    if (g_cfg.reload_log)
        API::get()->log_info("[Halo-CampE-UEVR] RELOAD anim window: %d components, rate x%.0f%s, paused %d, for %d ms",
                             s_anim_rate_n, want_rate ? g_cfg.reload_anim_rate : 1.0f,
                             want_rate ? "" : " (off)", paused, g_cfg.reload_anim_ms);
}
void reload_anim_rate_tick() {
    if (s_anim_rate_until == 0) return;
    if (now_ticks() < s_anim_rate_until) return;
    for (int i = 0; i < s_anim_rate_n; ++i) {
        auto* c = s_anim_rate_comps[i].get();
        reload_anim_rate_set(c, 1.0f);
        reload_set_bool_prop(c, L"bPauseAnims", false);
        s_anim_rate_comps[i] = TrackedObject{};
    }
    s_anim_rate_n = 0;
    s_anim_rate_until = 0;
}

// ---- ANIMVARS: which Animation Blueprint variable IS the reload? The AnimBPs (ANIMDUMP) hold
// the reload as a state in their graph, driven by variables on the anim instance. Snapshot every
// scalar property of the weapon's and arms' anim instances at the seat, re-read at +350 ms and
// +900 ms, log only what changed. Holding the one that flips is "the animation does not run".
struct AnimVar { std::wstring name; std::wstring cls; int32_t off; int kind; double v; };
std::vector<AnimVar> s_av_base[2];
TrackedObject        s_av_obj[2];
const char*          s_av_who[2] = {"weapon", "arms"};
long long            s_av_at = 0;
int                  s_av_phase = 0;

int av_kind(const std::wstring& c) {
    if (c == L"BoolProperty")   return 1;
    if (c == L"ByteProperty" || c == L"EnumProperty") return 2;
    if (c == L"IntProperty")    return 3;
    if (c == L"FloatProperty")  return 4;
    if (c == L"DoubleProperty") return 5;
    return 0;
}
double av_read(API::UObject* o, int32_t off, int kind) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(o) + off;
    if (off < 0 || off > 0x4000 || IsBadReadPtr(p, 8)) return -999999.0;
    switch (kind) {
        case 1: case 2: return (double)*p;
        case 3: return (double)*reinterpret_cast<const int32_t*>(p);
        case 4: return (double)*reinterpret_cast<const float*>(p);
        case 5: return *reinterpret_cast<const double*>(p);
    }
    return 0.0;
}
void av_collect(API::UObject* animbp, std::vector<AnimVar>& out) {
    out.clear();
    if (animbp == nullptr || IsBadReadPtr(animbp, sizeof(void*))) return;
    for (API::UStruct* s = animbp->get_class(); s != nullptr && out.size() < 600; s = s->get_super_struct()) {
        if (IsBadReadPtr(s, sizeof(void*))) break;
        for (API::FField* f = s->get_child_properties(); f != nullptr; f = f->get_next()) {
            if (IsBadReadPtr(f, sizeof(void*))) break;
            auto* fc = f->get_class();
            if (fc == nullptr || IsBadReadPtr(fc, sizeof(void*))) continue;
            const std::wstring cls = fc->get_name();
            const int kind = av_kind(cls);
            if (kind == 0) continue;
            auto* fp = static_cast<API::FProperty*>(f);
            const int32_t off = fp->get_offset();
            const auto* fn = f->get_fname();
            out.push_back(AnimVar{fn ? fn->to_string() : L"?", cls, off, kind, av_read(animbp, off, kind)});
            if (out.size() >= 600) break;
        }
    }
}
API::UObject* av_instance_of(API::UObject* comp) {
    if (comp == nullptr || IsBadReadPtr(comp, sizeof(void*))) return nullptr;
    auto** pai = comp->get_property_data<API::UObject*>(L"AnimScriptInstance");
    if (pai == nullptr || IsBadReadPtr(pai, sizeof(void*))) return nullptr;
    return *pai;
}
// ANIMOBJS: the OBJECT references on an object -- every ObjectProperty / WeakObjectProperty with
// the class and name of what it points at. The scalar roster cannot show where an AnimBP copies
// its pose FROM (bUsingCopyPoseFromMesh=1 on both), and that source mesh is the trail to the
// weapon's own Blam palette, where the slide bone really lives (2026-09-03).
void anim_objs_dump(API::UObject* o, const char* who) {
    if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) return;
    int n = 0;
    for (API::UStruct* s = o->get_class(); s != nullptr && n < 120; s = s->get_super_struct()) {
        if (IsBadReadPtr(s, sizeof(void*))) break;
        const std::wstring sn = s->get_fname() ? s->get_fname()->to_string() : L"?";
        for (API::FField* f = s->get_child_properties(); f != nullptr && n < 120; f = f->get_next()) {
            if (IsBadReadPtr(f, sizeof(void*))) break;
            auto* fc = f->get_class();
            if (fc == nullptr) continue;
            const std::wstring cls = fc->get_name();
            if (cls != L"ObjectProperty" && cls != L"WeakObjectProperty" && cls != L"SoftObjectProperty") continue;
            auto* fp = static_cast<API::FProperty*>(f);
            const int32_t off = fp->get_offset();
            if (off < 0 || off > 0x8000) continue;
            auto** pp = reinterpret_cast<API::UObject**>(reinterpret_cast<uint8_t*>(o) + off);
            if (IsBadReadPtr(pp, sizeof(void*))) continue;
            API::UObject* v = *pp;
            std::wstring vdesc = L"null";
            if (v != nullptr && !IsBadReadPtr(v, sizeof(void*))) {
                const auto* vn = v->get_fname();
                vdesc = class_name_of(v) + L" '" + (vn ? vn->to_string() : L"?") + L"'";
            }
            const auto* fn = f->get_fname();
            API::get()->log_info("[Halo-CampE-UEVR] ANIMOBJS %s %ls::%ls (%ls@0x%X) -> %ls",
                                 who, sn.c_str(), fn ? fn->to_string().c_str() : L"?", cls.c_str(),
                                 (unsigned)off, vdesc.c_str());
            ++n;
        }
        if (sn == L"Object") break;
    }
}

void audio_dump_fields(API::UObject* c);   // defined below; the bridge dump borrows its lister
void anim_vars_begin() {
    API::UObject* wcomp = nullptr;
    weapon_components([&](API::UObject* c) {
        const auto* fn = c->get_fname();
        if (class_name_of(c) == L"SkeletalMeshComponent" && fn != nullptr && fn->to_string() == L"Default") {
            wcomp = c; return false;
        }
        return true;
    });
    API::UObject* inst[2] = { av_instance_of(wcomp), av_instance_of(rig_tracked_component()) };
    if (g_cfg.anim_objs) {
        anim_objs_dump(wcomp, "weapon-comp");
        anim_objs_dump(inst[0], "weapon-anim");
        anim_objs_dump(fp_weapon_actor(), "weapon-actor");
        // THE BRIDGE. BlamMeshSynchronizationComponent (found 2026-09-03) is what carries Blam's
        // weapon-object node matrices into the UE mesh -- the slide's continuous pose during the
        // game's own animations comes through it. Its fields say what it syncs FROM.
        if (auto* wpn = fp_weapon_actor()) {
            for (const wchar_t* prop : {L"BlamMeshSynchronization", L"BlamObjectSynchronization", L"BlamWeapon"}) {
                auto** pc = wpn->get_property_data<API::UObject*>(prop);
                if (pc == nullptr || IsBadReadPtr(pc, sizeof(void*)) || *pc == nullptr) continue;
                API::get()->log_info("[Halo-CampE-UEVR] ANIMOBJS ---- %ls (%ls)", prop, class_name_of(*pc).c_str());
                anim_objs_dump(*pc, "bridge");
                audio_dump_fields(*pc);
                // VALUES of the scalar fields too -- BlamObjectIndex is the Blam object datum
                // this actor mirrors, the handle to the weapon object's own node matrices.
                std::vector<AnimVar> vals;
                av_collect(*pc, vals);
                for (const auto& v : vals)
                    if (v.cls == L"IntProperty" || v.cls == L"FloatProperty")
                        API::get()->log_info("[Halo-CampE-UEVR] ANIMOBJS %ls value %ls = %.0f (0x%X)",
                                             prop, v.name.c_str(), v.v, (unsigned)(int32_t)v.v);
            }
        }
    }
    for (int i = 0; i < 2; ++i) {
        s_av_obj[i] = TrackedObject{};
        if (inst[i] != nullptr) { s_av_obj[i].set(inst[i]); av_collect(inst[i], s_av_base[i]); }
        API::get()->log_info("[Halo-CampE-UEVR] ANIMVARS %s: %s, %zu scalar vars snapshotted at the seat",
                             s_av_who[i], inst[i] ? narrow(class_name_of(inst[i])).c_str() : "(no instance)",
                             s_av_base[i].size());
        // The full roster, not just what changed: a slide/bolt/chamber variable that the reload
        // does not touch is exactly what a hand-racked slide would drive (2026-09-03).
        for (const auto& v : s_av_base[i])
            API::get()->log_info("[Halo-CampE-UEVR] ANIMVARS %s   %ls (%ls@0x%X) = %.3f",
                                 s_av_who[i], v.name.c_str(), v.cls.c_str(), (unsigned)v.off, v.v);
    }
    s_av_at = now_ticks();
    s_av_phase = 1;
}
void anim_vars_tick() {
    if (s_av_phase == 0) return;
    const long long since = now_ticks() - s_av_at;
    const int want_ms = (s_av_phase == 1) ? 350 : 900;
    if (since < ms_to_ticks(want_ms)) return;
    for (int i = 0; i < 2; ++i) {
        auto* animbp = s_av_obj[i].get();
        if (animbp == nullptr) continue;
        std::vector<AnimVar> now;
        av_collect(animbp, now);
        int changed = 0;
        for (size_t k = 0; k < now.size() && k < s_av_base[i].size(); ++k) {
            if (now[k].off != s_av_base[i][k].off) continue;
            if (now[k].v == s_av_base[i][k].v) continue;
            ++changed;
            if (changed <= 40)
                API::get()->log_info("[Halo-CampE-UEVR] ANIMVARS %s +%dms: %ls (%ls@0x%X) %.3f -> %.3f",
                                     s_av_who[i], want_ms, now[k].name.c_str(), now[k].cls.c_str(),
                                     (unsigned)now[k].off, s_av_base[i][k].v, now[k].v);
        }
        API::get()->log_info("[Halo-CampE-UEVR] ANIMVARS %s +%dms: %d of %zu changed", s_av_who[i], want_ms, changed, now.size());
    }
    s_av_phase = (s_av_phase == 1) ? 2 : 0;
}

// ---- HOLD THE RELOAD STATE. ANIMVARS (2026-09-02, assault rifle): the weapon's AnimBP carries
// FirstPersonState (enum, 0 idle -> 6 for the reload) and flips AnimToggle to retrigger; the
// arms' instance changes nothing, it follows the weapon. Holding FirstPersonState at the idle
// value for the window is "the animation does not run" -- IF the game does not re-assert the
// variable after us each frame (our tick runs before the actors'). Not assumed: the value is
// read BEFORE every write and a non-idle read is counted as a re-assertion, and the summary
// line at the window's end says how many times the game fought back. Zero means the hold wins.
TrackedObject s_sh_inst;
long long     s_sh_until = 0;
long long     s_sh_press_at = 0;      // when this hold began
long long     g_last_refill_at = 0;   // the rounds counter last jumped up (slide_phantom_tick)
bool net_is_coop();
uint8_t       s_sh_idle = 0;
int           s_sh_reasserts = 0, s_sh_ticks = 0;
API::UObject* reload_weapon_default_comp() {
    // Memoised per weapon actor for 8 ms: the tick asks for this component eight or more times,
    // and each walk built two strings per component (perf audit, 2026-09-06). The actor itself is
    // validated by fp_weapon_actor(), so a component of a live actor is live.
    static API::UObject* s_memo_actor = nullptr; static API::UObject* s_memo = nullptr; static long long s_memo_at = 0;
    auto* actor = fp_weapon_actor();
    const long long nowt = now_ticks();
    if (actor != nullptr && actor == s_memo_actor && s_memo != nullptr && nowt - s_memo_at < ms_to_ticks(8)) return s_memo;
    API::UObject* wcomp = nullptr;
    weapon_components([&](API::UObject* c) {
        const auto* fn = c->get_fname();
        if (class_name_of(c) == L"SkeletalMeshComponent" && fn != nullptr && fn->to_string() == L"Default") {
            wcomp = c; return false;
        }
        return true;
    });
    s_memo_actor = actor; s_memo = wcomp; s_memo_at = nowt;
    return wcomp;
}
API::UObject* reload_weapon_anim_instance() {
    return av_instance_of(reload_weapon_default_comp());
}

// ---- ANIMSEQSET (dev): pose the weapon mesh from a named AnimSequence at a fixed time --
// "animseqset=Magnum_first_person_fire,0.05". The utoc (2026-09-03) shows the first-person
// weapon animations are UE AnimSequence assets (A_Magnum_first_person_fire_1_var1, the reloads,
// the two-state ammunition pose), so the slide is a UE bone those sequences pose. Single-node
// mode on the FIRE sequence, time scrubbed by the hand, IS smooth slide travel; this key finds
// the time where the slide sits fully back. Clearing the key hands the mesh back to its AnimBP.
TrackedObject s_as_seq;
std::string   s_as_last;
bool          s_as_active = false;
API::UObject* find_anim_sequence(const std::string& sub) {
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return nullptr;
    std::wstring want(sub.begin(), sub.end());
    for (auto& ch : want) ch = (wchar_t)towlower(ch);
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"AnimSequence") continue;
        const auto* fn = o->get_fname();
        if (fn == nullptr) continue;
        std::wstring nm = fn->to_string();
        for (auto& ch : nm) ch = (wchar_t)towlower(ch);
        if (nm.find(want) != std::wstring::npos) return o;
    }
    return nullptr;
}
void anim_seq_release(API::UObject* comp) {
    if (comp != nullptr && !IsBadReadPtr(comp, sizeof(void*))) {
        alignas(16) uint8_t p[64] = {0}; p[0] = 0;   // EAnimationMode::AnimationBlueprint
        comp->call_function(L"SetAnimationMode", p);
    }
    s_as_active = false; s_as_last.clear(); s_as_seq = TrackedObject{};
}
void anim_seq_set_tick() {
    const std::string spec = g_cfg.anim_seq_set;
    auto* comp = reload_weapon_default_comp();
    if (spec.empty()) { if (s_as_active) { anim_seq_release(comp); API::get()->log_info("[Halo-CampE-UEVR] ANIMSEQSET: released to AnimBP"); } return; }
    if (comp == nullptr) return;
    const size_t comma = spec.find(',');
    if (comma == std::string::npos) return;
    const std::string name(spec.substr(0, comma));
    const float time = (float)atof(spec.c_str() + comma + 1);
    if (spec != s_as_last) {
        s_as_last = spec;
        auto* seq = find_anim_sequence(name);
        if (seq == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] ANIMSEQSET: no AnimSequence matching '%s'", name.c_str()); return; }
        s_as_seq.set(seq);
        { alignas(16) uint8_t p[64] = {0}; p[0] = 1;   // EAnimationMode::AnimationSingleNode
          comp->call_function(L"SetAnimationMode", p); }
        { alignas(16) uint8_t p[64] = {0}; *reinterpret_cast<API::UObject**>(p) = seq;
          comp->call_function(L"SetAnimation", p); }
        s_as_active = true;
        API::get()->log_info("[Halo-CampE-UEVR] ANIMSEQSET: %ls on %ls, single-node, time=%.3f",
                             seq->get_full_name().c_str(), class_name_of(comp).c_str(), time);
    }
    if (s_as_seq.get() == nullptr) return;
    alignas(16) uint8_t p[64] = {0};
    *reinterpret_cast<float*>(p) = time;
    p[4] = 0;   // bFireNotifies = false
    comp->call_function(L"SetPosition", p);
}
void reload_state_hold_begin() {
    if (!g_cfg.reload_hold_state || g_cfg.reload_anim_ms <= 0) return;
    auto* animbp = reload_weapon_anim_instance();
    if (animbp == nullptr) return;
    auto* p = animbp->get_property_data<uint8_t>(L"FirstPersonState");
    if (p == nullptr || IsBadReadPtr(p, 1)) {
        if (g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] STATEHOLD: no FirstPersonState on %ls", class_name_of(animbp).c_str());
        return;
    }
    s_sh_idle = *p;
    s_sh_inst.set(animbp);
    s_sh_press_at = now_ticks();
    s_sh_until = s_sh_press_at + ms_to_ticks((g_cfg.coop_auto && net_is_coop()) ? g_cfg.reload_anim_ms_coop : g_cfg.reload_anim_ms);
    s_sh_reasserts = 0; s_sh_ticks = 0;
}
bool reload_gestures_busy();   // the mag is out, the seat is pending, or the lock waits for the rack (defined below)
void reload_state_hold_tick() {
    if (s_sh_until == 0) return;
    auto* animbp = s_sh_inst.get();
    // The hold outlives reload_anim_ms while OUR reload is still in progress (the shotgun's
    // shell-by-shell animation ran on past 1.2 s and showed, 2026-09-06), capped at +6 s.
    const long long nowt = now_ticks();
    // Coop: the refill is the proof the game's reload ran; until it lands the hold stays (capped).
    const bool coop_wait = g_cfg.coop_auto && net_is_coop() && g_last_refill_at < s_sh_press_at && nowt < s_sh_until + ms_to_ticks(4000);
    const bool over = nowt >= s_sh_until && !coop_wait && (!reload_gestures_busy() || nowt >= s_sh_until + ms_to_ticks(6000));
    if (animbp == nullptr || over) {
        if (g_cfg.reload_log)
            API::get()->log_info("[Halo-CampE-UEVR] STATEHOLD: held FirstPersonState=%d for %d ticks, game re-asserted %d times%s",
                                 (int)s_sh_idle, s_sh_ticks, s_sh_reasserts,
                                 animbp == nullptr ? " (instance gone)" : "");
        s_sh_until = 0; s_sh_inst = TrackedObject{};
        return;
    }
    auto* p = animbp->get_property_data<uint8_t>(L"FirstPersonState");
    if (p == nullptr || IsBadWritePtr(p, 1)) return;
    ++s_sh_ticks;
    if (*p != s_sh_idle) { ++s_sh_reasserts; *p = s_sh_idle; }
}

// ---- AUDIODUMP: where does the reload SOUND come from? The state hold killed the animation
// and the sound still played (2026-09-02), so it is not an anim notify; the weapon carries a
// HaloAudioTrackingComponent, which suggests the Blam sim's sound event lands in UE audio via
// that. Two sweeps after the seat (+120 ms, +400 ms) over the object array for AudioComponents
// that are PLAYING -- owner, sound asset -- plus a one-shot field listing of the tracking
// component. Whatever is playing on the weapon during the window is the mute target.
long long s_ad_at = 0;
int       s_ad_phase = 0;
void audio_dump_fields(API::UObject* c) {
    if (c == nullptr || IsBadReadPtr(c, sizeof(void*))) return;
    int n = 0;
    for (API::UStruct* s = c->get_class(); s != nullptr && n < 80; s = s->get_super_struct()) {
        if (IsBadReadPtr(s, sizeof(void*))) break;
        const std::wstring sn = s->get_fname() ? s->get_fname()->to_string() : L"?";
        if (sn.find(L"Actor") != std::wstring::npos && sn.find(L"Component") == std::wstring::npos) break;
        for (API::FField* f = s->get_child_properties(); f != nullptr && n < 80; f = f->get_next()) {
            if (IsBadReadPtr(f, sizeof(void*))) break;
            auto* fc = f->get_class();
            const auto* fn = f->get_fname();
            API::get()->log_info("[Halo-CampE-UEVR] AUDIODUMP field %ls::%ls (%ls @0x%X)",
                                 sn.c_str(), fn ? fn->to_string().c_str() : L"?",
                                 fc ? fc->get_name().c_str() : L"?",
                                 (unsigned)static_cast<API::FProperty*>(f)->get_offset());
            ++n;
        }
        if (sn == L"SceneComponent" || sn == L"ActorComponent") break;
    }
}
std::string trim_cfg(const char* v);
extern bool s_sl_empty_at_drop;
bool net_is_coop();
// ---- WWISE (see Config.hpp reload_wwise_dump). ------------------------------------------------
// THE WEAPON STEM from a skeletal mesh name (2026-09-06): "SK_" stripped, the name cut at the
// first variant suffix (_Default, _Translucent, _Mech, _Shadow, _FP), and any underscore left
// INSIDE the name removed -- the rocket launcher's mesh is SK_Rocket_Launcher, and cutting at the
// first underscore made it "Rocket", which no table matched, so its hinge never spawned.
std::wstring weapon_stem_from_mesh(std::wstring stem) {
    if (stem.rfind(L"SK_", 0) == 0) stem = stem.substr(3);
    size_t cut = std::wstring::npos;
    for (const wchar_t* suf : { L"_Default", L"_Translucent", L"_Mech", L"_Shadow", L"_FP" }) {
        const size_t at = stem.find(suf);
        if (at != std::wstring::npos && at < cut) cut = at;
    }
    if (cut != std::wstring::npos) stem = stem.substr(0, cut);
    std::wstring out; for (wchar_t ch : stem) if (ch != L'_') out += ch;
    return out;
}
std::wstring ak_weapon_stem() {
    std::wstring stem;
    auto* src = reload_weapon_default_comp();
    if (src == nullptr) return stem;
    API::UObject* mesh = nullptr;
    for (const wchar_t* nm : { L"SkinnedAsset", L"SkeletalMesh" }) {
        auto** pm = src->get_property_data<API::UObject*>(nm);
        if (pm != nullptr && !IsBadReadPtr(pm, sizeof(void*)) && *pm != nullptr) { mesh = *pm; break; }
    }
    if (mesh == nullptr || IsBadReadPtr(mesh, sizeof(void*)) || mesh->get_fname() == nullptr) return stem;
    stem = mesh->get_fname()->to_string();
    if (stem.rfind(L"SK_", 0) == 0) stem = stem.substr(3);
    stem = weapon_stem_from_mesh(stem);
    for (auto& ch : stem) ch = (wchar_t)towlower(ch);
    return stem;
}
void ak_dump_function(const wchar_t* path) {
    auto* fn = API::get()->find_uobject<API::UFunction>(path);
    if (fn == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] WWISE fn %ls: NOT FOUND", path); return; }
    std::wstring sig;
    for (API::FField* f = fn->get_child_properties(); f != nullptr; f = f->get_next()) {
        if (IsBadReadPtr(f, sizeof(void*))) break;
        const auto* nm = f->get_fname(); auto* fc = f->get_class();
        if (nm == nullptr || fc == nullptr) continue;
        wchar_t buf[160]; swprintf_s(buf, L" %ls:%ls@0x%X", nm->to_string().c_str(), fc->get_name().c_str(), (unsigned)static_cast<API::FProperty*>(f)->get_offset());
        sig += buf;
    }
    API::get()->log_info("[Halo-CampE-UEVR] WWISE fn %ls (params 0x%X):%ls", path, (unsigned)fn->get_properties_size(), sig.c_str());
}
void ak_dump(const char* when) {
    if (!g_cfg.reload_wwise_dump) return;
    static int s_dumps = 0;
    if (s_dumps >= 2) return;
    ++s_dumps;
    static bool s_sigs = false;
    if (!s_sigs) {
        s_sigs = true;
        ak_dump_function(L"Function /Script/AkAudio.AkGameplayStatics.PostEvent");
        ak_dump_function(L"Function /Script/AkAudio.AkGameplayStatics.PostEventByName");
        ak_dump_function(L"Function /Script/AkAudio.AkComponent.PostAkEvent");
        ak_dump_function(L"Function /Script/AkAudio.AkComponent.SetOutputBusVolume");
        ak_dump_function(L"Function /Script/AkAudio.AkComponent.Stop");
        ak_dump_function(L"Function /Script/AkAudio.AkGameplayStatics.StopActor");
    }
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;
    auto* wpn = fp_weapon_actor();
    const std::wstring stem = ak_weapon_stem();
    const wchar_t* keys[] = {L"reload", L"mag", L"clip", L"chamber", L"bolt", L"slide", L"rack", L"cock", L"pump", L"charg"};
    int n_ak = 0, n_ev = 0, n_comp = 0, shown = 0;
    std::wstring classes;
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        const std::wstring cl = class_name_of(o);
        if (cl.rfind(L"Ak", 0) != 0) continue;
        ++n_ak;
        if (classes.find(L" " + cl + L" ") == std::wstring::npos && classes.size() < 900) classes += L" " + cl + L" ";
        const auto* fnm = o->get_fname(); if (fnm == nullptr) continue;
        std::wstring nm = fnm->to_string(); std::wstring lo = nm; for (auto& ch : lo) ch = (wchar_t)towlower(ch);
        if (cl == L"AkAudioEvent") {
            ++n_ev;
            bool hit = (!stem.empty() && lo.find(stem) != std::wstring::npos);
            for (const wchar_t* k : keys) if (lo.find(k) != std::wstring::npos) { hit = true; break; }
            if (hit && shown < 160) { ++shown; API::get()->log_info("[Halo-CampE-UEVR] WWISE event %ls", nm.c_str()); }
        } else if (cl.find(L"Component") != std::wstring::npos) {
            ++n_comp;
            API::UObject* owner = nullptr;
            { alignas(16) uint8_t p[64] = {0}; o->call_function(L"GetOwner", p); owner = *reinterpret_cast<API::UObject**>(p); }
            const bool on_wpn = (owner != nullptr && wpn != nullptr && owner == wpn);
            if (shown < 200) { ++shown; API::get()->log_info("[Halo-CampE-UEVR] WWISE comp %ls '%ls' owner=%ls%s", cl.c_str(), nm.c_str(), owner ? class_name_of(owner).c_str() : L"(none)", on_wpn ? " [FP WEAPON]" : ""); }
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] WWISE dump at %s: %d Ak objects, %d events, %d components, stem '%ls'; classes:%ls", when, n_ak, n_ev, n_comp, stem.c_str(), classes.c_str());
}
// The AkAudioEvent's Wwise id property (ShortId in the 2022+ integration; a few names tried).
// The 2023 integration keeps the id in FWwiseEventCookedData (a StructProperty, no reflected
// field for the id itself). GetWwiseShortId() is reflected: the uint32 inside the cooked data
// that equals it is the id, found once per launch and the offset reused.
uint32_t ak_event_short_id(API::UObject* ev) {
    if (ev == nullptr || IsBadReadPtr(ev, sizeof(void*))) return 0;
    alignas(16) uint8_t p[64] = {0};
    ev->call_function(L"GetWwiseShortId", p);
    return *reinterpret_cast<uint32_t*>(p);
}
uint32_t* ak_event_id_ptr(API::UObject* ev) {
    static int s_off = -1;   // offset within EventCookedData
    if (ev == nullptr || IsBadReadPtr(ev, sizeof(void*))) return nullptr;
    auto* cd = ev->get_property_data<uint8_t>(L"EventCookedData");
    if (cd == nullptr || IsBadWritePtr(cd, 0x60)) return nullptr;
    if (s_off >= 0) return reinterpret_cast<uint32_t*>(cd + s_off);
    const uint32_t id = ak_event_short_id(ev);
    if (id == 0) return nullptr;
    for (int o = 0; o + 4 <= 0x60; o += 4) {
        if (*reinterpret_cast<uint32_t*>(cd + o) == id) {
            s_off = o;
            API::get()->log_info("[Halo-CampE-UEVR] WWISE: event id lives at EventCookedData+0x%X (id %u)", (unsigned)o, id);
            return reinterpret_cast<uint32_t*>(cd + o);
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] WWISE: id %u not found inside EventCookedData", id);
    return nullptr;
}
// ExecuteAction(Stop) on an event: the 2023 integration's reflected signature, placed by name.
void ak_event_stop(API::UObject* ev, API::UObject* actor) {
    if (ev == nullptr || IsBadReadPtr(ev, sizeof(void*))) return;
    auto* fn = ev->get_class() ? ev->get_class()->find_function(L"ExecuteAction") : nullptr;
    if (fn == nullptr) return;
    alignas(16) uint8_t p[128] = {0};
    if ((size_t)fn->get_properties_size() > sizeof(p)) return;
    auto put = [&](const wchar_t* name, const void* v, size_t n) { auto* pr = fn->find_property(name); if (pr == nullptr) return; const int32_t off = pr->get_offset(); if (off >= 0 && (size_t)off + n <= sizeof(p)) memcpy(p + off, v, n); };
    const uint8_t stop = 0;   // AkActionOnEventType::Stop = 0
    const int32_t dur = 0;
    put(L"ActionType", &stop, 1);
    put(L"Actor", &actor, sizeof(void*));
    put(L"TransitionDuration", &dur, sizeof(int32_t));
    ev->call_function(L"ExecuteAction", p);
}
void ak_roster(const wchar_t* cls_path) {
    auto* cls = API::get()->find_uobject<API::UClass>(cls_path);
    if (cls == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] WWISE roster %ls: NOT FOUND", cls_path); return; }
    std::wstring props, funcs;
    int np = 0;
    for (API::UStruct* st = cls; st != nullptr && np < 200; st = st->get_super_struct()) {
        if (IsBadReadPtr(st, sizeof(void*))) break;
        for (API::FField* f = st->get_child_properties(); f != nullptr && np < 200; f = f->get_next()) {
            if (IsBadReadPtr(f, sizeof(void*))) break;
            const auto* nm = f->get_fname(); auto* fc = f->get_class();
            if (nm == nullptr || fc == nullptr) continue;
            wchar_t buf[160]; swprintf_s(buf, L" %ls:%ls@0x%X", nm->to_string().c_str(), fc->get_name().c_str(), (unsigned)static_cast<API::FProperty*>(f)->get_offset());
            props += buf; ++np;
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] WWISE roster %ls props:%ls", cls_path, props.c_str());
    // functions: the class's children chain
    int nf = 0;
    for (API::UStruct* st = cls; st != nullptr && nf < 200; st = st->get_super_struct()) {
        if (IsBadReadPtr(st, sizeof(void*))) break;
        for (auto* c = st->get_children(); c != nullptr && nf < 200; c = c->get_next()) {
            if (IsBadReadPtr(c, sizeof(void*))) break;
            const auto* nm = c->get_fname(); if (nm == nullptr) continue;
            funcs += L" " + nm->to_string(); ++nf;
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] WWISE roster %ls funcs:%ls", cls_path, funcs.c_str());
}
long long s_akd_at = 0; int s_akd_count = 0;
void ak_dump_after_press() {
    if (!g_cfg.reload_wwise_dump || s_akd_count >= 4) return;
    s_akd_at = now_ticks() + ms_to_ticks(900);
}
void ak_dump_after_press_tick() {
    if (s_akd_at == 0 || now_ticks() < s_akd_at) return;
    s_akd_at = 0; ++s_akd_count;
    static bool s_rosters = false;
    if (!s_rosters) { s_rosters = true; ak_dump_function(L"Function /Script/AkAudio.AkAudioEvent.PostOnActor"); ak_dump_function(L"Function /Script/AkAudio.AkAudioEvent.ExecuteAction"); ak_dump_function(L"Function /Script/AkAudio.AkAudioEvent.GetWwiseShortId"); ak_dump_function(L"Function /Script/AkAudio.AkAudioEvent.UnloadData"); ak_dump_function(L"Function /Script/AkAudio.AkAudioEvent.LoadData"); }
    auto* arr = API::get()->get_uobject_array(); if (arr == nullptr) return;
    int n = 0;
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn && n < 300; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"AkAudioEvent") continue;
        const auto* fnm = o->get_fname(); if (fnm == nullptr) continue;
        std::wstring nm = fnm->to_string(); std::wstring lo = nm; for (auto& ch : lo) ch = (wchar_t)towlower(ch);
        if (lo.find(L"weaanim_player") == std::wstring::npos && lo.find(L"wep_") == std::wstring::npos) continue;
        if (lo.find(L"_fire") != std::wstring::npos || lo.find(L"impact") != std::wstring::npos || lo.find(L"projectile") != std::wstring::npos) continue;
        ++n;
        uint32_t* idp = ak_event_id_ptr(o);
        API::get()->log_info("[Halo-CampE-UEVR] WWISE loaded event %ls id=%s%u", nm.c_str(), idp ? "" : "?", idp ? *idp : 0u);
    }
    API::get()->log_info("[Halo-CampE-UEVR] WWISE loaded reload events after the press: %d", n);
    // The game's RTPC / switch / state assets (names), once: a player-only switch or a volume
    // RTPC the sim sets on its emitters would explain accepted-yet-silent posts elsewhere.
    static bool s_groups = false;
    if (!s_groups) {
        s_groups = true;
        std::wstring line; int k = 0;
        for (int32_t i = 0; i < nn && k < 400; ++i) {
            auto* o = static_cast<API::UObject*>(arr->get_object(i));
            if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
            const std::wstring cl = class_name_of(o);
            if (cl != L"AkRtpc" && cl != L"AkSwitchValue" && cl != L"AkStateValue" && cl != L"AkTrigger" && cl != L"AkAuxBus") continue;
            const auto* fnm = o->get_fname(); if (fnm == nullptr) continue;
            line += L" " + cl.substr(2) + L":" + fnm->to_string(); ++k;
            if (line.size() > 900) { API::get()->log_info("[Halo-CampE-UEVR] WWISE groups%ls", line.c_str()); line.clear(); }
        }
        API::get()->log_info("[Halo-CampE-UEVR] WWISE groups%ls (%d total)", line.c_str(), k);
    }
}
// ---- THE ENGINE HOOK (see Config.hpp ak_post_rva). --------------------------------------------
using AkPostFn = uint32_t (*)(uint32_t id, uint64_t go, uint32_t flags, void* cb, void* cookie, uint32_t next, void* pext, uint32_t playing);
AkPostFn s_ak_post_orig = nullptr;
int      s_ak_hook_id = -1;
bool     s_ak_hook_tried = false;
thread_local bool t_ak_our_post = false;   // set around the plugin's own posts
std::atomic<int> s_ak_log_left{0};
// The muted id set and window, published for the audio thread.
std::atomic<long long> g_ak_win_until{0};
std::atomic<uint32_t>  g_ak_mute_ids[48];
// Wwise's id of a name: FNV-1 32 over the lowercase bytes (verified against 12 logged ids).
uint32_t ak_fnv(const std::string& name) { uint32_t h = 0x811C9DC5u; for (unsigned char ch : name) { h *= 0x01000193u; h ^= (uint32_t)tolower(ch); } return h; }
std::atomic<int>       g_ak_mute_n{0};
std::atomic<uint64_t>  g_ak_sim_go{0};        // the last game object the sim posted a muted id on
std::atomic<long long> g_ak_sim_go_at{0};
// ---- THE RECIPE (see Config.hpp ak_mimic) ----------------------------------------------------
using AkSetListenersFn = int (*)(uint64_t go, const uint64_t* ids, uint32_t n);
using AkSetSwitchFn    = int (*)(uint32_t group, uint32_t state, uint64_t go);
using AkSetRtpcFn      = int (*)(uint32_t rtpc, float value, uint64_t go, int32_t ms, int32_t curve, bool bypass);
using AkSetPositionFn  = int (*)(uint64_t go, const void* pos, uint8_t flags);
using AkRegisterFn     = int (*)(uint64_t go, const char* name);
using AkUnregisterFn   = int (*)(uint64_t go);
AkSetListenersFn s_ak_setlisteners_orig = nullptr;
AkSetSwitchFn    s_ak_setswitch_orig = nullptr;
AkSetRtpcFn      s_ak_setrtpc_orig = nullptr;
AkSetPositionFn  s_ak_setposition_orig = nullptr;
AkRegisterFn     s_ak_register = nullptr;      // called, not hooked
AkUnregisterFn   s_ak_unregister = nullptr;
struct AkRecipe {
    uint64_t go = 0;
    uint64_t listeners[8] = {0}; uint32_t nlisteners = 0; bool has_listeners = false;
    uint32_t sw_group[6] = {0}, sw_state[6] = {0}; int nsw = 0;
    uint32_t rtpc[8] = {0}; float rtpc_val[8] = {0}; int nrtpc = 0;
    uint8_t  pos[48] = {0}; bool has_pos = false;
};
AkRecipe s_ak_ring[16]; int s_ak_ring_i = 0;
AkRecipe* ak_recipe_for(uint64_t go, bool create) {
    for (auto& r : s_ak_ring) if (r.go == go && go != 0) return &r;
    if (!create) return nullptr;
    AkRecipe& r = s_ak_ring[s_ak_ring_i]; s_ak_ring_i = (s_ak_ring_i + 1) % 16;
    r = AkRecipe{}; r.go = go; return &r;
}
AkRecipe s_ak_template;            // the sim's reload emitter, copied at its (blocked) post
bool     s_ak_template_ok = false;
std::atomic<uint64_t> g_ak_our_go{0};   // our AkComponent's emitter, seen at our own post
std::atomic<uint32_t> g_ak_pending_id{0};  // akmimic 5: a step event waiting for the sim's next emitter
std::atomic<long long> g_ak_pending_at{0};
std::atomic<uint64_t> g_ak_adopted{0};          // mode 7: the sim emitter we hold
std::atomic<uint64_t> g_ak_newest{0};           // the newest sim emitter seen posting (candidate)
std::atomic<uint64_t> g_ak_listener{0};         // the sim's listener id (from its SetListeners calls)
uint8_t  g_ak_head_pos[48] = {0}; std::atomic<bool> g_ak_head_pos_ok{false};
bool ak_window_open() { const long long u = g_ak_win_until.load(std::memory_order_relaxed); return u != 0 && now_ticks() < u; }
int ak_setlisteners_detour(uint64_t go, const uint64_t* ids, uint32_t n) {
    if (!t_ak_our_post && ids != nullptr && n >= 1 && !IsBadReadPtr(ids, 8)) g_ak_listener.store(ids[0], std::memory_order_relaxed);
    if (ak_window_open() && !t_ak_our_post && ids != nullptr && n <= 8) {
        if (auto* r = ak_recipe_for(go, true)) { r->nlisteners = n; for (uint32_t i = 0; i < n; ++i) r->listeners[i] = ids[i]; r->has_listeners = true; }
    }
    return s_ak_setlisteners_orig(go, ids, n);
}
int ak_setswitch_detour(uint32_t group, uint32_t state, uint64_t go) {
    if (ak_window_open() && !t_ak_our_post) { if (auto* r = ak_recipe_for(go, true)) if (r->nsw < 6) { r->sw_group[r->nsw] = group; r->sw_state[r->nsw] = state; ++r->nsw; } }
    return s_ak_setswitch_orig(group, state, go);
}
int ak_setrtpc_detour(uint32_t rtpc, float value, uint64_t go, int32_t ms, int32_t curve, bool bypass) {
    if (ak_window_open() && !t_ak_our_post && go != 0) { if (auto* r = ak_recipe_for(go, true)) if (r->nrtpc < 8) { r->rtpc[r->nrtpc] = rtpc; r->rtpc_val[r->nrtpc] = value; ++r->nrtpc; } }
    return s_ak_setrtpc_orig(rtpc, value, go, ms, curve, bypass);
}
int ak_setposition_detour(uint64_t go, const void* pos, uint8_t flags) {
    if (!t_ak_our_post && pos != nullptr && go != 0 && go == g_ak_listener.load(std::memory_order_relaxed) && !IsBadReadPtr(pos, 48)) {
        memcpy(g_ak_head_pos, pos, 48); g_ak_head_pos_ok.store(true, std::memory_order_relaxed);
        const uint64_t held = g_ak_adopted.load(std::memory_order_relaxed);
        if (g_cfg.ak_mimic == 7 && held != 0 && s_ak_setposition_orig) { t_ak_our_post = true; s_ak_setposition_orig(held, pos, flags); t_ak_our_post = false; }
    }
    if (ak_window_open() && !t_ak_our_post && pos != nullptr && !IsBadReadPtr(pos, 48)) { if (auto* r = ak_recipe_for(go, true)) { memcpy(r->pos, pos, 48); r->has_pos = true; } }
    return s_ak_setposition_orig(go, pos, flags);
}
AkUnregisterFn s_ak_unregister_orig = nullptr;
std::atomic<uint64_t> g_ak_deferred_unreg{0};   // the sim's reload emitter whose unregister we held back
int ak_unregister_detour(uint64_t go) {
    if (g_cfg.ak_mimic == 7 && !t_ak_our_post && go != 0 && go < 0xFFFFFFull) {
        const uint64_t held = g_ak_adopted.load(std::memory_order_relaxed);
        if (go == held) return 1;   // ours now; the sim thinks it is gone
        const uint64_t newest = g_ak_newest.load(std::memory_order_relaxed);
        if (go == newest && go != held) {
            // The sim is done with its newest emitter: adopt it, release the one we held.
            g_ak_adopted.store(go, std::memory_order_relaxed);
            if (held != 0) s_ak_unregister_orig(held);
            if (g_ak_head_pos_ok.load(std::memory_order_relaxed) && s_ak_setposition_orig) { t_ak_our_post = true; s_ak_setposition_orig(go, g_ak_head_pos, 3); t_ak_our_post = false; }
            if (g_cfg.ak_log) { static int s_said = 0; if (s_said++ < 12) API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC7: adopted the sim emitter 0x%llX (released 0x%llX)", (unsigned long long)go, (unsigned long long)held); }
            return 1;
        }
    }
    if (g_cfg.ak_mimic == 6 && ak_window_open() && !t_ak_our_post && go != 0 && go == g_ak_sim_go.load(std::memory_order_relaxed)) {
        g_ak_deferred_unreg.store(go, std::memory_order_relaxed);
        if (g_cfg.ak_log) API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC6: the sim's unregister of its reload emitter 0x%llX deferred to the window's end", (unsigned long long)go);
        return 1;   // AK_Success, as far as the sim is concerned
    }
    return s_ak_unregister_orig(go);
}
bool ak_hook_one(int rva, const uint8_t* prologue, size_t n, void* detour, void** orig, const char* what) {
    if (rva == 0) return false;
    const uint8_t* exe = reinterpret_cast<const uint8_t*>(GetModuleHandleA(nullptr));
    void* target = (void*)(exe + (uintptr_t)(uint32_t)rva);
    if (IsBadReadPtr(target, n) || memcmp(target, prologue, n) != 0) { API::get()->log_info("[Halo-CampE-UEVR] AKHOOK: %s prologue mismatch at exe+0x%X -- not hooked", what, (unsigned)rva); return false; }
    const int id = API::get()->param()->functions->register_inline_hook(target, detour, orig);
    if (id < 0 || *orig == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] AKHOOK: %s register FAILED (id=%d)", what, id); *orig = nullptr; return false; }
    API::get()->log_info("[Halo-CampE-UEVR] AKHOOK: %s hooked at exe+0x%X id=%d", what, (unsigned)rva, id);
    return true;
}
void ak_recipe_hooks_install() {
    static bool s_tried = false;
    if (s_tried) return;
    s_tried = true;
    static const uint8_t P_LSP[16] = {0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x10,0x57,0x48,0x83,0xEC,0x20,0x80};   // SetListeners / SetSwitch / SetPosition(inner) share it
    static const uint8_t P_RTPC[16] = {0x48,0x83,0xEC,0x48,0x0F,0xB6,0x44,0x24,0x78,0x88,0x44,0x24,0x30,0x8B,0x44,0x24};
    static const uint8_t P_REG[16] = {0x40,0x53,0x48,0x83,0xEC,0x20,0x80,0x3D,0xA7,0xF5,0xD6,0x02,0x00,0x48,0x8B,0xD9};
    static const uint8_t P_UNREG[16] = {0x40,0x53,0x48,0x83,0xEC,0x20,0x80,0x3D,0xD7,0x97,0xD6,0x02,0x00,0x48,0x8B,0xD9};
    ak_hook_one(g_cfg.ak_fn_setlisteners, P_LSP, 16, (void*)&ak_setlisteners_detour, (void**)&s_ak_setlisteners_orig, "SetListeners");
    ak_hook_one(g_cfg.ak_fn_setswitch, P_LSP, 16, (void*)&ak_setswitch_detour, (void**)&s_ak_setswitch_orig, "SetSwitch");
    ak_hook_one(g_cfg.ak_fn_setrtpc, P_RTPC, 16, (void*)&ak_setrtpc_detour, (void**)&s_ak_setrtpc_orig, "SetRTPCValue");
    ak_hook_one(g_cfg.ak_fn_setposition, P_LSP, 16, (void*)&ak_setposition_detour, (void**)&s_ak_setposition_orig, "SetPosition");
    const uint8_t* exe = reinterpret_cast<const uint8_t*>(GetModuleHandleA(nullptr));
    auto check = [&](int rva, const uint8_t* pro, const char* what) -> void* {
        void* t = (void*)(exe + (uintptr_t)(uint32_t)rva);
        if (IsBadReadPtr(t, 16) || memcmp(t, pro, 16) != 0) { API::get()->log_info("[Halo-CampE-UEVR] AKHOOK: %s prologue mismatch at exe+0x%X -- not used", what, (unsigned)rva); return nullptr; }
        return t;
    };
    s_ak_register = (AkRegisterFn)check(g_cfg.ak_fn_register, P_REG, "RegisterGameObj");
    if (ak_hook_one(g_cfg.ak_fn_unregister, P_UNREG, 16, (void*)&ak_unregister_detour, (void**)&s_ak_unregister_orig, "UnregisterGameObj"))
        s_ak_unregister = s_ak_unregister_orig;   // our own unregisters go straight to the engine
    else
        s_ak_unregister = (AkUnregisterFn)check(g_cfg.ak_fn_unregister, P_UNREG, "UnregisterGameObj");
}
// Our emitters, unregistered a while after their post.
struct AkOurs { uint64_t go; long long at; };
AkOurs s_ak_ours[8]; int s_ak_ours_i = 0; uint64_t s_ak_next_go = 0x7A5E000000000100ull;
void ak_ours_tick() {
    if (s_ak_unregister == nullptr) return;
    const long long t = now_ticks();
    for (auto& o : s_ak_ours) if (o.go != 0 && t - o.at > ms_to_ticks(4000)) { s_ak_unregister(o.go); o.go = 0; }
}
uint32_t ak_post_detour(uint32_t id, uint64_t go, uint32_t flags, void* cb, void* cookie, uint32_t next, void* pext, uint32_t playing) {
    const long long until = g_ak_win_until.load(std::memory_order_relaxed);
    const bool in_window = until != 0 && now_ticks() < until;
    bool muted = false;
    if (in_window && !t_ak_our_post) {
        const int n = g_ak_mute_n.load(std::memory_order_relaxed);
        for (int i = 0; i < n && !muted; ++i) if (g_ak_mute_ids[i].load(std::memory_order_relaxed) == id) muted = true;
    }
    const bool block = muted && g_cfg.reload_ak_mute == 4;
    if (muted) {
        g_ak_sim_go.store(go, std::memory_order_relaxed); g_ak_sim_go_at.store(now_ticks(), std::memory_order_relaxed);
        if (auto* r = ak_recipe_for(go, false)) { s_ak_template = *r; s_ak_template_ok = true; }
    }
    if (t_ak_our_post && go > 0xFFFFFFull) g_ak_our_go.store(go, std::memory_order_relaxed);
    if (!t_ak_our_post && go != 0 && go < 0xFFFFFFull) g_ak_newest.store(go, std::memory_order_relaxed);
    if (in_window && g_cfg.ak_log && s_ak_log_left.fetch_sub(1) > 0) {
        // WHO posted: the return address as an exe offset (the sim's bridge vs the UE integration),
        // and the emitter kind (a small integer is the sim's own numbering, a pointer-sized value
        // is a UE AkComponent) -- 2026-09-05, to learn how the sim makes its posts audible.
        const uint8_t* exe = reinterpret_cast<const uint8_t*>(GetModuleHandleA(nullptr));
        const uint8_t* ra = reinterpret_cast<const uint8_t*>(_ReturnAddress());
        const long long off = (exe != nullptr) ? (long long)(ra - exe) : -1;
        API::get()->log_info("[Halo-CampE-UEVR] AKPOST id=%u go=0x%llX flags=0x%X cb=%p cookie=%p ext=%u ret=exe+0x%llX %s %s", id, (unsigned long long)go, flags, cb, cookie, next, off,
                             go > 0xFFFFFFull ? "[ptr-emitter]" : "[sim-emitter]",
                             t_ak_our_post ? "OURS" : (block ? "BLOCKED" : (muted ? "known (mode<4, passed)" : "other")));
        if (g_cfg.ak_stack && muted && !t_ak_our_post) {
            void* frames[10] = {0};
            const USHORT n = RtlCaptureStackBackTrace(1, 10, frames, nullptr);
            const uint8_t* sim = reinterpret_cast<const uint8_t*>(GetModuleHandleA("HaloSimulation_tag_release.dll"));
            std::string line;
            for (USHORT i = 0; i < n; ++i) {
                const uint8_t* f = reinterpret_cast<const uint8_t*>(frames[i]);
                char buf[64];
                if (exe != nullptr && f >= exe && f < exe + 0x10000000ull) snprintf(buf, sizeof(buf), " exe+0x%llX", (unsigned long long)(f - exe));
                else if (sim != nullptr && f >= sim && f < sim + 0x4000000ull) snprintf(buf, sizeof(buf), " sim+0x%llX", (unsigned long long)(f - sim));
                else snprintf(buf, sizeof(buf), " %p", (const void*)f);
                line += buf;
            }
            API::get()->log_info("[Halo-CampE-UEVR] AKSTACK id=%u:%s", id, line.c_str());
        }
    }
    if (!t_ak_our_post && (in_window || g_cfg.ak_mimic == 7) && (g_cfg.ak_mimic == 5 || g_cfg.ak_mimic == 6 || g_cfg.ak_mimic == 7)) {
        const uint32_t pend = g_ak_pending_id.load(std::memory_order_relaxed);
        if (pend != 0 && now_ticks() - g_ak_pending_at.load(std::memory_order_relaxed) < ms_to_ticks(1500)) {
            g_ak_pending_id.store(0, std::memory_order_relaxed);
            const uint32_t pl = s_ak_post_orig(pend, go, 0u, nullptr, nullptr, 0u, nullptr, 0u);
            if (g_cfg.ak_log) API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC5: step event %u rode the sim's emitter 0x%llX (its %u) -> playing %u", pend, (unsigned long long)go, id, pl);
        }
    }
    if (block) {
        if (g_cfg.ak_mimic == 4) {
            const std::string sub = trim_cfg(g_cfg.ak_mimic4_event);
            const uint32_t post_id = sub.empty() ? id : ak_fnv(sub);
            const uint32_t pl = s_ak_post_orig(post_id, go, 0u, nullptr, nullptr, 0u, nullptr, 0u);
            if (g_cfg.ak_log) API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC4: in place of the sim's %u, %u ('%s') posted by the plugin on 0x%llX, flags 0 -> playing %u", id, post_id, sub.c_str(), (unsigned long long)go, pl);
            return pl;
        }
        return 0;   // AK_INVALID_PLAYING_ID
    }
    return s_ak_post_orig(id, go, flags, cb, cookie, next, pext, playing);
}
void ak_hook_install() {
    if (s_ak_hook_tried) return;
    s_ak_hook_tried = true;
    if (g_cfg.ak_post_rva == 0) return;
    const uint8_t* exe = reinterpret_cast<const uint8_t*>(GetModuleHandleA(nullptr));
    if (exe == nullptr) return;
    void* target = (void*)(exe + (uintptr_t)(uint32_t)g_cfg.ak_post_rva);
    static const uint8_t PROLOGUE[16] = {0x48,0x8B,0xC4,0x48,0x89,0x58,0x08,0x48,0x89,0x68,0x10,0x48,0x89,0x70,0x18,0x4C};
    if (IsBadReadPtr(target, sizeof(PROLOGUE)) || memcmp(target, PROLOGUE, sizeof(PROLOGUE)) != 0) {
        API::get()->log_info("[Halo-CampE-UEVR] AKHOOK: prologue mismatch at exe+0x%X -- the game moved; NOT hooking", (unsigned)g_cfg.ak_post_rva);
        return;
    }
    const int id = API::get()->param()->functions->register_inline_hook(target, (void*)&ak_post_detour, (void**)&s_ak_post_orig);
    if (id < 0 || s_ak_post_orig == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] AKHOOK: register_inline_hook FAILED (id=%d)", id); s_ak_post_orig = nullptr; return; }
    s_ak_hook_id = id;
    API::get()->log_info("[Halo-CampE-UEVR] AKHOOK: PostEvent hooked at exe+0x%X id=%d", (unsigned)g_cfg.ak_post_rva, id);
}
void ak_set_rtpc(API::UObject* actor, float value, const char* why);   // defined with the step posts below
// reloadakmute: the weapon's reload events with their ids zeroed for the window.
struct AkMuted { TrackedObject ev; uint32_t id; };
AkMuted s_akm[24]; int s_akm_n = 0;
long long s_akm_until = 0;
// The set is remembered per weapon stem: the first press pays the object-array walk, the rest
// re-check the handles (one slot compare each). Up to 25 walks per press before (audit, 2026-09-06).
std::string s_akm_cache_stem; AkMuted s_akm_cache[24]; int s_akm_cache_n = 0;
const char* ak_weapon_token(const std::string& stem) {
    if (stem.rfind("magnum", 0) == 0) return "magnum_";
    if (stem.rfind("assaultrifle", 0) == 0) return "ar_";
    if (stem.rfind("smg", 0) == 0) return "ar_";   // the SMG reloads with the AR's events (post log, 2026-09-06)
    if (stem.rfind("battlerifle", 0) == 0) return "br_";
    if (stem.rfind("sniperrifle", 0) == 0) return "sniperrifle_";
    if (stem.rfind("needler", 0) == 0) return "needler_";
    if (stem.rfind("rocketlauncher", 0) == 0) return "spnker_rocket_launcher_";
    if (stem.rfind("plasmapistol", 0) == 0) return "plasmapistol_";
    if (stem.rfind("shotgun", 0) == 0) return "wep_fol_shotgun_";
    if (stem.rfind("spikerifle", 0) == 0) return "wep_spikerifle_reload";
    if (stem.rfind("fuelrodcannon", 0) == 0) return "fuelrodgun_reload";
    return nullptr;
}
void ak_id_mute_begin() {
    if (g_cfg.reload_ak_mute == 0 || g_cfg.reload_mute_ms <= 0) return;
    if (s_akm_n > 0) return;   // a window is already open
    std::string stem; { const std::wstring w = ak_weapon_stem(); stem.assign(w.begin(), w.end()); }
    const char* tok = ak_weapon_token(stem);
    if (tok == nullptr) {
        // A weapon with no known reload events (the SMG, 2026-09-06): the window still opens with
        // nothing to refuse, so the post log shows what the game plays for it.
        s_akm_until = now_ticks() + ms_to_ticks(g_cfg.reload_mute_ms);
        g_ak_mute_n.store(0, std::memory_order_relaxed);
        g_ak_win_until.store(s_akm_until, std::memory_order_relaxed);
        s_ak_log_left.store(60, std::memory_order_relaxed);
        if (g_cfg.reload_ak_mute == 4 || g_cfg.ak_log) ak_hook_install();
        if (g_cfg.reload_log || g_cfg.reload_wwise_dump || g_cfg.ak_log) API::get()->log_info("[Halo-CampE-UEVR] WWISE mute: no reload events known for '%s', window open for the log only", stem.c_str());
        return;
    }
    std::wstring wtok(tok, tok + strlen(tok));
    const bool chm = wtok.find(L"wep_") == std::wstring::npos && wtok.find(L"fuelrod") == std::wstring::npos;
    const std::wstring t_player = chm ? (L"weaanim_player_" + wtok) : wtok;
    const std::wstring t_nonplayer = chm ? (L"weaanim_nonplayer_" + wtok) : (wtok + L"_nonplayer");
    bool from_cache = false;
    if (s_akm_cache_n > 0 && s_akm_cache_stem == stem) {
        from_cache = true;
        for (int i = 0; i < s_akm_cache_n; ++i) if (s_akm_cache[i].ev.get() == nullptr) { from_cache = false; break; }
        if (from_cache) {
            for (int i = 0; i < s_akm_cache_n; ++i) {
                s_akm[i] = s_akm_cache[i];
                if (auto* o = s_akm[i].ev.get()) {
                    if (g_cfg.reload_ak_mute == 1) { if (auto* idp = ak_event_id_ptr(o)) *idp = 0; }
                    else if (g_cfg.reload_ak_mute == 2) { alignas(16) uint8_t p[32] = {0}; o->call_function(L"UnloadData", p); }
                }
            }
            s_akm_n = s_akm_cache_n;
        }
    }
    if (!from_cache) {
    auto* arr = API::get()->get_uobject_array(); if (arr == nullptr) return;
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn && s_akm_n < 24; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"AkAudioEvent") continue;
        const auto* fnm = o->get_fname(); if (fnm == nullptr) continue;
        std::wstring lo = fnm->to_string(); for (auto& ch : lo) ch = (wchar_t)towlower(ch);
        const bool is_player = lo.find(t_player) != std::wstring::npos;
        const bool is_nonplayer = lo.find(t_nonplayer) != std::wstring::npos || (chm && lo.find(L"nonplayer") != std::wstring::npos && lo.find(wtok) != std::wstring::npos);
        if (!(is_player || is_nonplayer)) continue;
        if (g_cfg.reload_mute_variant == 0 && !is_player) continue;
        if (g_cfg.reload_mute_variant == 1 && !is_nonplayer) continue;
        if (lo.find(L"_fire") != std::wstring::npos || lo.find(L"dryfire") != std::wstring::npos) continue;
        uint32_t* idp = ak_event_id_ptr(o); if (idp == nullptr || *idp == 0) continue;
        s_akm[s_akm_n].ev.set_at(o, i); s_akm[s_akm_n].id = *idp; ++s_akm_n;
        if (g_cfg.reload_wwise_dump) API::get()->log_info("[Halo-CampE-UEVR] WWISE mute covers %ls (id %u)", fnm->to_string().c_str(), *idp);
        if (g_cfg.reload_ak_mute == 1) *idp = 0;
        else if (g_cfg.reload_ak_mute == 2) { alignas(16) uint8_t p[32] = {0}; o->call_function(L"UnloadData", p); }
    }
    s_akm_cache_stem = stem; s_akm_cache_n = s_akm_n;
    for (int i = 0; i < s_akm_n; ++i) s_akm_cache[i] = s_akm[i];
    }
    s_akm_until = now_ticks() + ms_to_ticks(g_cfg.reload_mute_ms);
    int nid = 0;
    for (int i = 0; i < s_akm_n && nid < 48; ++i) g_ak_mute_ids[nid++].store(s_akm[i].id, std::memory_order_relaxed);
    {   // the foley names, hashed
        const std::string lst = trim_cfg(g_cfg.ak_mute_names);
        size_t pos = 0;
        while (pos < lst.size() && nid < 48) {
            size_t comma = lst.find(',', pos); if (comma == std::string::npos) comma = lst.size();
            std::string n = lst.substr(pos, comma - pos);
            while (!n.empty() && (unsigned char)n.back() <= ' ') n.pop_back();
            while (!n.empty() && (unsigned char)n.front() <= ' ') n.erase(n.begin());
            if (!n.empty()) g_ak_mute_ids[nid++].store(ak_fnv(n), std::memory_order_relaxed);
            pos = comma + 1;
        }
    }
    g_ak_mute_n.store(nid, std::memory_order_relaxed);
    g_ak_win_until.store(s_akm_until, std::memory_order_relaxed);
    s_ak_log_left.store(60, std::memory_order_relaxed);
    if (g_cfg.reload_ak_mute == 4 || g_cfg.ak_log) ak_hook_install();
    if (g_cfg.ak_mimic != 0) ak_recipe_hooks_install();
    if (g_cfg.ak_rtpc_global) ak_set_rtpc(nullptr, g_cfg.ak_rtpc_value, "window open");
    if (g_cfg.reload_log || g_cfg.reload_wwise_dump) API::get()->log_info("[Halo-CampE-UEVR] WWISE mute mode %d: %d reload event(s) of '%s' for %d ms", g_cfg.reload_ak_mute, s_akm_n, stem.c_str(), g_cfg.reload_mute_ms);
}
void ak_id_mute_end() {
    for (int i = 0; i < s_akm_n; ++i) {
        if (auto* o = s_akm[i].ev.get()) {
            if (g_cfg.reload_ak_mute == 1) { if (auto* idp = ak_event_id_ptr(o)) *idp = s_akm[i].id; }
            else if (g_cfg.reload_ak_mute == 2) { alignas(16) uint8_t p[32] = {0}; o->call_function(L"LoadData", p); }
        }
        s_akm[i] = AkMuted{};
    }
    if (s_akm_n > 0 && (g_cfg.reload_log || g_cfg.reload_wwise_dump)) API::get()->log_info("[Halo-CampE-UEVR] WWISE mute: %d event(s) restored", s_akm_n);
    if (s_ak_template_ok && g_cfg.ak_log) {
        const AkRecipe& r = s_ak_template;
        std::string l; for (uint32_t i = 0; i < r.nlisteners; ++i) { char b2[32]; snprintf(b2, sizeof(b2), " 0x%llX", (unsigned long long)r.listeners[i]); l += b2; }
        std::string sw; for (int i = 0; i < r.nsw; ++i) { char b2[48]; snprintf(b2, sizeof(b2), " %u=%u", r.sw_group[i], r.sw_state[i]); sw += b2; }
        std::string rt; for (int i = 0; i < r.nrtpc; ++i) { char b2[48]; snprintf(b2, sizeof(b2), " %u=%.2f", r.rtpc[i], r.rtpc_val[i]); rt += b2; }
        const float* pf = reinterpret_cast<const float*>(r.pos);
        API::get()->log_info("[Halo-CampE-UEVR] AKRECIPE go=0x%llX listeners(%u):%s switches:%s rtpcs:%s pos=%s[%.2f %.2f %.2f | %.2f %.2f %.2f | %.1f %.1f %.1f]", (unsigned long long)r.go, r.nlisteners, l.c_str(), sw.c_str(), rt.c_str(), r.has_pos ? "" : "(none)", pf[0], pf[1], pf[2], pf[3], pf[4], pf[5], pf[6], pf[7], pf[8]);
    }
    s_akm_n = 0; s_akm_until = 0;
    g_ak_win_until.store(0, std::memory_order_relaxed); g_ak_mute_n.store(0, std::memory_order_relaxed);
    {   // the sim's reload emitter we kept alive goes now
        const uint64_t d = g_ak_deferred_unreg.exchange(0, std::memory_order_relaxed);
        if (d != 0 && s_ak_unregister_orig != nullptr) { t_ak_our_post = true; s_ak_unregister_orig(d); t_ak_our_post = false; if (g_cfg.ak_log) API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC6: reload emitter 0x%llX unregistered at the window's end", (unsigned long long)d); }
        g_ak_sim_go.store(0, std::memory_order_relaxed);
    }
    if (g_cfg.ak_rtpc_global) ak_set_rtpc(nullptr, g_cfg.ak_rtpc_restore, "window closed");
}
// Mode 3: the game's post is cut every tick of the window, on OUR firearm's emitter and its
// owner's (the pawn) only -- from the headset, 2026-09-05: only our firearm's animation sound is muted --
// except an event the plugin itself posted in the last 1500 ms.
struct AkPosted { TrackedObject ev; long long at; };
AkPosted s_ak_posted[8]; int s_ak_posted_i = 0;
void ak_note_posted(API::UObject* ev) { s_ak_posted[s_ak_posted_i].ev.set(ev); s_ak_posted[s_ak_posted_i].at = now_ticks(); s_ak_posted_i = (s_ak_posted_i + 1) % 8; }
bool ak_posted_recently(API::UObject* ev) { const long long t = now_ticks(); for (auto& p : s_ak_posted) if (p.ev.get() == ev && t - p.at < ms_to_ticks(1500)) return true; return false; }
void ak_mute_stop_tick() {
    if (g_cfg.reload_ak_mute != 3 || s_akm_n == 0) return;
    auto* wpn = fp_weapon_actor();
    if (wpn == nullptr) return;
    API::UObject* owner = nullptr;
    { alignas(16) uint8_t p[64] = {0}; wpn->call_function(L"GetOwner", p); owner = *reinterpret_cast<API::UObject**>(p); }
    for (int i = 0; i < s_akm_n; ++i) {
        auto* o = s_akm[i].ev.get();
        if (o == nullptr || ak_posted_recently(o)) continue;
        ak_event_stop(o, wpn);
        if (owner != nullptr) ak_event_stop(o, owner);
    }
}
// A muted event's real id, for the plugin's own post: restored around the call.
uint32_t ak_muted_id_of(API::UObject* ev) { for (int i = 0; i < s_akm_n; ++i) if (s_akm[i].ev.get() == ev) return s_akm[i].id; return 0; }
void ak_bus_volume(float vol) {
    auto* arr = API::get()->get_uobject_array();
    auto* wpn = fp_weapon_actor();
    if (arr == nullptr || wpn == nullptr) return;
    API::UObject* wpn_owner = nullptr;
    { alignas(16) uint8_t p[64] = {0}; wpn->call_function(L"GetOwner", p); wpn_owner = *reinterpret_cast<API::UObject**>(p); }
    int n = 0;
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"AkComponent") continue;
        API::UObject* owner = nullptr;
        { alignas(16) uint8_t p[64] = {0}; o->call_function(L"GetOwner", p); owner = *reinterpret_cast<API::UObject**>(p); }
        if (owner == nullptr || (owner != wpn && (wpn_owner == nullptr || owner != wpn_owner))) continue;
        alignas(16) uint8_t p[64] = {0}; *reinterpret_cast<float*>(p) = vol;
        o->call_function(L"SetOutputBusVolume", p);
        ++n;
    }
    if (g_cfg.reload_log || g_cfg.reload_wwise_dump) API::get()->log_info("[Halo-CampE-UEVR] WWISE mute: bus volume %.1f on %d AkComponent(s) of the weapon/owner", vol, n);
}
// AKVTDUMP: the engine interface's vtable, once (see Config.hpp ak_vt_dump).
void ak_vt_dump() {
    static bool s_done = false;
    if (!g_cfg.ak_vt_dump || s_done) return;
    s_done = true;
    const uint8_t* exe = reinterpret_cast<const uint8_t*>(GetModuleHandleA(nullptr));
    if (exe == nullptr) return;
    const uint8_t* const* gp = reinterpret_cast<const uint8_t* const*>(exe + (uintptr_t)(uint32_t)g_cfg.ak_vt_global);
    if (IsBadReadPtr(gp, sizeof(void*))) { API::get()->log_info("[Halo-CampE-UEVR] AKVT: global exe+0x%X unreadable", (unsigned)g_cfg.ak_vt_global); return; }
    const uint8_t* obj = *gp;
    if (obj == nullptr || IsBadReadPtr(obj, sizeof(void*))) { API::get()->log_info("[Halo-CampE-UEVR] AKVT: object null at exe+0x%X", (unsigned)g_cfg.ak_vt_global); return; }
    const uint8_t* const* vt = *reinterpret_cast<const uint8_t* const* const*>(obj);
    if (vt == nullptr || IsBadReadPtr(vt, sizeof(void*))) { API::get()->log_info("[Halo-CampE-UEVR] AKVT: vtable unreadable"); return; }
    API::get()->log_info("[Halo-CampE-UEVR] AKVT: exe %p object %p vtable rva 0x%llX", (const void*)exe, (const void*)obj, (unsigned long long)(reinterpret_cast<const uint8_t*>(vt) - exe));
    std::string line;
    for (int i = 0; i < g_cfg.ak_vt_count; ++i) {
        if (IsBadReadPtr(vt + i, sizeof(void*))) break;
        const uint8_t* f = vt[i];
        if (f == nullptr) break;
        char buf[48]; snprintf(buf, sizeof(buf), " %d:0x%llX", i, (unsigned long long)(f - exe));
        line += buf;
        if (line.size() > 900) { API::get()->log_info("[Halo-CampE-UEVR] AKVT slots%s", line.c_str()); line.clear(); }
    }
    if (!line.empty()) API::get()->log_info("[Halo-CampE-UEVR] AKVT slots%s", line.c_str());
}
void ak_mute_begin() {
    ak_vt_dump();
    ak_id_mute_begin();
    ak_dump_after_press();
}
void ak_mute_tick() {
    if (g_cfg.ak_mimic != 0 || g_cfg.reload_ak_mute == 4 || g_cfg.ak_log) { ak_hook_install(); if (g_cfg.ak_mimic != 0) ak_recipe_hooks_install(); }
    ak_dump_after_press_tick();
    ak_ours_tick();
    if (s_akm_until == 0) return;
    if (now_ticks() < s_akm_until) { ak_mute_stop_tick(); return; }
    ak_id_mute_end();
}
// reloadstepsound: "Weapon:drop=Ev,seat=Ev,rack=Ev;*:drop=Ev" -- the event for this weapon and step.
std::string ak_step_event_in(const char* table, const char* step) {
    std::string tbl = trim_cfg(table);
    if (tbl.empty()) return "";
    std::string stem; { const std::wstring w = ak_weapon_stem(); stem.assign(w.begin(), w.end()); }
    std::string fallback;
    size_t pos = 0;
    while (pos < tbl.size()) {
        size_t semi = tbl.find(';', pos); if (semi == std::string::npos) semi = tbl.size();
        std::string ent = tbl.substr(pos, semi - pos);
        const size_t colon = ent.find(':');
        if (colon != std::string::npos) {
            std::string w = ent.substr(0, colon), rest = ent.substr(colon + 1);
            for (auto& ch : w) ch = (char)tolower((unsigned char)ch);
            std::string found;
            size_t q = 0;
            while (q < rest.size()) {
                size_t comma = rest.find(',', q); if (comma == std::string::npos) comma = rest.size();
                std::string kv = rest.substr(q, comma - q);
                const size_t eq = kv.find('=');
                if (eq != std::string::npos && _stricmp(kv.substr(0, eq).c_str(), step) == 0) found = kv.substr(eq + 1);
                q = comma + 1;
            }
            while (!found.empty() && (unsigned char)found.back() <= ' ') found.pop_back();
            if (stem.rfind(w, 0) == 0 && !found.empty()) return found;
            if (w == "*" && !found.empty()) fallback = found;
        }
        pos = semi + 1;
    }
    return fallback;
}
std::string ak_step_event(const char* step) {
    const std::string o = ak_step_event_in(g_cfg.reload_step_override, step);
    if (!o.empty()) return (o == "none") ? "" : o;
    return ak_step_event_in(g_cfg.reload_step_sound, step);
}
API::UObject* ak_find_event(const std::string& name_in) {
    std::string name = name_in;
    if (name.rfind("Play_", 0) != 0 && name.rfind("play_", 0) != 0) name = "Play_006_chm_ge_weaanim_player_" + name;
    for (auto& ch : name) ch = (char)tolower((unsigned char)ch);
    if (g_cfg.reload_step_variant == 1) { const size_t at = name.find("weaanim_player_"); if (at != std::string::npos) name.replace(at, 15, "weaanim_nonplayer_"); }
    // The table: every step name the session has asked for, hit or miss. The old one-entry cache
    // thrashed between alternating steps and never remembered a miss, so every rack walked the
    // whole object array (perf audit, 2026-09-06). A miss is retried after 20 s.
    struct AkEvCache { std::string name; TrackedObject obj; long long at; bool found; };
    static AkEvCache s_tab[32]; static int s_tab_n = 0;
    const long long nowt = now_ticks();
    AkEvCache* slot = nullptr;
    for (int i = 0; i < s_tab_n; ++i) if (s_tab[i].name == name) { slot = &s_tab[i]; break; }
    if (slot != nullptr) {
        if (slot->found) { if (auto* o = slot->obj.get()) return o; }
        else if (nowt - slot->at < ms_to_ticks(20000)) return nullptr;
    } else {
        slot = &s_tab[s_tab_n < 32 ? s_tab_n++ : 31];
        slot->name = name;
    }
    slot->found = false; slot->at = nowt; slot->obj = TrackedObject{};
    auto* arr = API::get()->get_uobject_array(); if (arr == nullptr) return nullptr;
    std::wstring want(name.begin(), name.end());
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"AkAudioEvent") continue;
        const auto* fnm = o->get_fname(); if (fnm == nullptr) continue;
        std::wstring lo = fnm->to_string(); for (auto& ch : lo) ch = (wchar_t)towlower(ch);
        if (lo == want) { slot->obj.set_at(o, i); slot->found = true; return o; }
    }
    return nullptr;
}
// The RTPC (see Config.hpp ak_rtpc): the AkRtpc asset by name, set on an actor (or globally).
API::UObject* ak_find_rtpc(const std::string& name) {
    static std::string s_name; static TrackedObject s_obj;
    if (name == s_name) if (auto* o = s_obj.get()) return o;
    auto* arr = API::get()->get_uobject_array(); if (arr == nullptr) return nullptr;
    std::wstring want(name.begin(), name.end());
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"AkRtpc") continue;
        const auto* fnm = o->get_fname(); if (fnm == nullptr) continue;
        if (fnm->to_string() == want) { s_name = name; s_obj.set(o); return o; }
    }
    return nullptr;
}
void ak_set_rtpc(API::UObject* actor, float value, const char* why) {
    const std::string nm = trim_cfg(g_cfg.ak_rtpc);
    if (nm.empty()) return;
    auto* rtpc = ak_find_rtpc(nm);
    auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/AkAudio.AkGameplayStatics");
    auto* fn = cls ? cls->find_function(L"SetRTPCValue") : nullptr;
    auto* cdo = cls ? cls->get_class_default_object() : nullptr;
    static bool s_sig = false;
    if (!s_sig) { s_sig = true; ak_dump_function(L"Function /Script/AkAudio.AkGameplayStatics.SetRTPCValue"); }
    if (rtpc == nullptr || fn == nullptr || cdo == nullptr) { static int s_said = 0; if (s_said++ < 3) API::get()->log_info("[Halo-CampE-UEVR] AKRTPC '%s': asset %s, SetRTPCValue %s", nm.c_str(), rtpc ? "found" : "NOT LOADED", fn ? "ok" : "NOT FOUND"); return; }
    alignas(16) uint8_t p[256] = {0};
    if ((size_t)fn->get_properties_size() > sizeof(p)) return;
    auto put = [&](const wchar_t* name, const void* v, size_t n) { auto* pr = fn->find_property(name); if (pr == nullptr) return false; const int32_t off = pr->get_offset(); if (off >= 0 && (size_t)off + n <= sizeof(p)) memcpy(p + off, v, n); return true; };
    const int32_t interp = 0;
    put(L"RTPCValue", &rtpc, sizeof(void*));
    put(L"Value", &value, sizeof(float));
    put(L"InterpolationTimeMs", &interp, sizeof(int32_t));
    put(L"Actor", &actor, sizeof(void*));
    cdo->call_function(L"SetRTPCValue", p);
    if (g_cfg.reload_log || g_cfg.reload_wwise_dump) API::get()->log_info("[Halo-CampE-UEVR] AKRTPC '%s' = %.1f on %ls (%s)", nm.c_str(), value, actor ? class_name_of(actor).c_str() : L"GLOBAL", why);
}
void ak_step_sound(const char* step) {
    std::string ev;
    if (s_sl_empty_at_drop) ev = ak_step_event((std::string(step) + "empty").c_str());
    if (ev.empty()) ev = ak_step_event(step);
    if (ev.empty()) return;
    auto* evo = ak_find_event(ev);
    auto* wpn = fp_weapon_actor();
    auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/AkAudio.AkGameplayStatics");
    auto* fn = cls ? cls->find_function(L"PostEvent") : nullptr;
    auto* cdo = cls ? cls->get_class_default_object() : nullptr;
    if (evo == nullptr && g_cfg.ak_mimic == 7 && s_ak_post_orig != nullptr) {
        // Not loaded yet (the game loads an event's object on its first play): the id is the
        // FNV-1 of the name, and the adopted emitter takes it as it would any other.
        const uint64_t go = g_ak_adopted.load(std::memory_order_relaxed);
        const uint32_t evid = ak_fnv(ev);
        if (go != 0) {
            const uint32_t pl = s_ak_post_orig(evid, go, 0u, nullptr, nullptr, 0u, nullptr, 0u);
            if (g_cfg.reload_log || g_cfg.ak_log) API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC7 step %s: '%s' has no loaded object, posted by hash %u on 0x%llX -> playing %u", step, ev.c_str(), evid, (unsigned long long)go, pl);
            return;
        }
    }
    if (evo == nullptr || wpn == nullptr || fn == nullptr || cdo == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] WWISE step %s: event '%s' %s, weapon %s, PostEvent %s", step, ev.c_str(), evo ? "found" : "NOT FOUND", wpn ? "ok" : "none", fn ? "ok" : "NOT FOUND");
        return;
    }
    alignas(16) uint8_t p[256] = {0};
    const size_t need = (size_t)fn->get_properties_size();
    if (need > sizeof(p)) { API::get()->log_info("[Halo-CampE-UEVR] WWISE step %s: PostEvent params 0x%X too big", step, (unsigned)need); return; }
    bool ok = true;
    auto put = [&](const wchar_t* name, const void* v, size_t n) {
        auto* pr = fn->find_property(name);
        if (pr == nullptr) { ok = false; API::get()->log_info("[Halo-CampE-UEVR] WWISE step: PostEvent has no '%ls'", name); return; }
        const int32_t off = pr->get_offset();
        if (off < 0 || (size_t)off + n > sizeof(p)) { ok = false; return; }
        memcpy(p + off, v, n);
    };
    // The actor the post rides: the weapon, or its owner (the pawn) in mode 2.
    API::UObject* on = wpn;
    if (g_cfg.reload_step_via == 2) {
        alignas(16) uint8_t q[64] = {0}; wpn->call_function(L"GetOwner", q);
        if (auto* o = *reinterpret_cast<API::UObject**>(q)) on = o;
    }
    put(L"AkEvent", &evo, sizeof(void*));
    put(L"Actor", &on, sizeof(void*));
    if (!ok) return;
    if (!g_cfg.ak_rtpc_global && g_cfg.reload_step_via != 3 && g_cfg.reload_step_via != 4) ak_set_rtpc(on, g_cfg.ak_rtpc_value, step);
    // A muted event posts with its real id for this one call.
    const uint32_t muted = (g_cfg.reload_ak_mute == 1) ? ak_muted_id_of(evo) : 0;
    uint32_t* idp = muted ? ak_event_id_ptr(evo) : nullptr;
    if (idp != nullptr) *idp = muted;
    uint32_t playing = 0;
    ak_note_posted(evo);
    t_ak_our_post = true;
    Vec3 gun{}; bool have_gun = false;
    bool posted = false;
    if (g_cfg.ak_mimic == 1 && s_ak_template_ok && s_ak_post_orig != nullptr && s_ak_register != nullptr) {
        // OUR OWN EMITTER, the sim's recipe: register, position, listeners, switches, RTPCs, post.
        const AkRecipe& r = s_ak_template;
        const uint64_t go = s_ak_next_go++;
        const uint32_t evid = ak_event_short_id(evo);
        int rr = s_ak_register(go, "halo_vr_reload");
        int rp = -1, rl = -1;
        if (r.has_pos && s_ak_setposition_orig) rp = s_ak_setposition_orig(go, r.pos, 3);
        if (r.has_listeners && s_ak_setlisteners_orig) rl = s_ak_setlisteners_orig(go, r.listeners, r.nlisteners);
        if (s_ak_setswitch_orig) for (int i = 0; i < r.nsw; ++i) s_ak_setswitch_orig(r.sw_group[i], r.sw_state[i], go);
        if (s_ak_setrtpc_orig) for (int i = 0; i < r.nrtpc; ++i) s_ak_setrtpc_orig(r.rtpc[i], r.rtpc_val[i], go, 0, 4, false);
        playing = s_ak_post_orig(evid, go, 0u, nullptr, nullptr, 0u, nullptr, 0u);
        s_ak_ours[s_ak_ours_i] = AkOurs{go, now_ticks()}; s_ak_ours_i = (s_ak_ours_i + 1) % 8;
        posted = true;
        API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC step %s: emitter 0x%llX register=%d pos=%d listeners=%d(%u) switches=%d rtpcs=%d -> post %u playing %u", step, (unsigned long long)go, rr, rp, rl, r.nlisteners, r.nsw, r.nrtpc, evid, playing);
    } else if (g_cfg.ak_mimic == 7 && s_ak_post_orig != nullptr) {
        const uint32_t evid = ak_event_short_id(evo);
        const uint64_t go = g_ak_adopted.load(std::memory_order_relaxed);
        if (go != 0) {
            playing = s_ak_post_orig(evid, go, 0u, nullptr, nullptr, 0u, nullptr, 0u);
            posted = true;
            if (g_cfg.reload_log || g_cfg.ak_log) API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC7 step %s: event %u on the adopted emitter 0x%llX -> playing %u", step, evid, (unsigned long long)go, playing);
        } else {
            g_ak_pending_id.store(evid, std::memory_order_relaxed); g_ak_pending_at.store(now_ticks(), std::memory_order_relaxed);
            posted = true;
            API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC7 step %s: nothing adopted yet, event %u parked for the next sim emitter", step, evid);
        }
    } else if (g_cfg.ak_mimic == 6 && s_ak_post_orig != nullptr) {
        const uint32_t evid = ak_event_short_id(evo);
        const uint64_t go = g_ak_sim_go.load(std::memory_order_relaxed);
        if (go != 0 && ak_window_open()) {
            playing = s_ak_post_orig(evid, go, 0u, nullptr, nullptr, 0u, nullptr, 0u);
            posted = true;
            API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC6 step %s: event %u on the sim's reload emitter 0x%llX -> playing %u", step, evid, (unsigned long long)go, playing);
        } else {
            g_ak_pending_id.store(evid, std::memory_order_relaxed); g_ak_pending_at.store(now_ticks(), std::memory_order_relaxed);
            posted = true;
            API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC6 step %s: no reload emitter yet, event %u parked for the sim's next", step, evid);
        }
    } else if (g_cfg.ak_mimic == 5) {
        const uint32_t evid = ak_event_short_id(evo);
        g_ak_pending_id.store(evid, std::memory_order_relaxed); g_ak_pending_at.store(now_ticks(), std::memory_order_relaxed);
        posted = true;
        API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC5 step %s: event %u parked for the sim's next emitter", step, evid);
    } else if ((g_cfg.ak_mimic == 2 || g_cfg.ak_mimic == 3) && s_ak_template_ok) {
        // OUR COMPONENT'S EMITTER (learned at the previous post), given the sim's listeners (2) or
        // its switches and RTPCs (3) before the normal post below.
        const uint64_t ours = g_ak_our_go.load(std::memory_order_relaxed);
        const AkRecipe& r = s_ak_template;
        if (ours != 0) {
            if (g_cfg.ak_mimic == 2 && r.has_listeners && s_ak_setlisteners_orig) { const int rl = s_ak_setlisteners_orig(ours, r.listeners, r.nlisteners); API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC step %s: listeners(%u) on our emitter 0x%llX -> %d", step, r.nlisteners, (unsigned long long)ours, rl); }
            if (g_cfg.ak_mimic == 3) { if (s_ak_setswitch_orig) for (int i = 0; i < r.nsw; ++i) s_ak_setswitch_orig(r.sw_group[i], r.sw_state[i], ours); if (s_ak_setrtpc_orig) for (int i = 0; i < r.nrtpc; ++i) s_ak_setrtpc_orig(r.rtpc[i], r.rtpc_val[i], ours, 0, 4, false); API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC step %s: %d switch(es), %d rtpc(s) on our emitter 0x%llX", step, r.nsw, r.nrtpc, (unsigned long long)ours); }
        } else API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC step %s: our emitter not seen yet (this post teaches it)", step);
    }
    if (!posted && g_cfg.reload_step_via == 4 && s_ak_post_orig != nullptr) {
        // THE SIM'S EMITTER (see Config.hpp): only once the sim has posted in this window.
        const long long at = g_ak_sim_go_at.load(std::memory_order_relaxed);
        const uint64_t go = g_ak_sim_go.load(std::memory_order_relaxed);
        if (go != 0 && at != 0 && now_ticks() - at < ms_to_ticks(g_cfg.reload_mute_ms)) {
            const uint32_t id = ak_event_short_id(evo);
            if (id != 0) { playing = s_ak_post_orig(id, go, 1u, nullptr, nullptr, 0u, nullptr, 0u); posted = true;
                API::get()->log_info("[Halo-CampE-UEVR] WWISE step %s: raw engine post of %u on the sim's emitter 0x%llX -> playing id %u", step, id, (unsigned long long)go, playing); }
        }
    }
    if (!posted && g_cfg.reload_step_via == 5) {
        // THE LISTENER: the player controller's camera manager actor.
        if (auto* pc = API::get()->get_player_controller(0)) {
            if (auto** cm = pc->get_property_data<API::UObject*>(L"PlayerCameraManager")) {
                if (!IsBadReadPtr(cm, sizeof(void*)) && *cm != nullptr) {
                    put(L"Actor", cm, sizeof(void*));
                    cdo->call_function(L"PostEvent", p);
                    if (auto* pr = fn->find_property(L"ReturnValue")) playing = *reinterpret_cast<uint32_t*>(p + pr->get_offset());
                    posted = true; on = *cm;
                }
            }
        }
        if (!posted) API::get()->log_info("[Halo-CampE-UEVR] WWISE step %s: no camera manager, falling back", step);
    }
    if (!posted && (g_cfg.reload_step_via == 3 || g_cfg.reload_step_via == 4 || g_cfg.reload_step_via == 5)) {
        // AT THE GUN: AkGameplayStatics.PostEventAtLocation(AkEvent, Location, Orientation, WorldContextObject).
        auto* src = reload_weapon_default_comp();
        have_gun = (src != nullptr) && call_ret_vec3(src, L"K2_GetComponentLocation", &gun);
        auto* lf = cls->find_function(L"PostEventAtLocation");
        static bool s_sig = false;
        if (!s_sig) { s_sig = true; ak_dump_function(L"Function /Script/AkAudio.AkGameplayStatics.PostEventAtLocation"); }
        alignas(16) uint8_t q[256] = {0};
        if (have_gun && lf != nullptr && (size_t)lf->get_properties_size() <= sizeof(q)) {
            bool lok = true;
            auto lput = [&](const wchar_t* name, const void* v, size_t n) { auto* pr = lf->find_property(name); if (pr == nullptr) { lok = false; API::get()->log_info("[Halo-CampE-UEVR] WWISE step: PostEventAtLocation has no '%ls'", name); return; } const int32_t off = pr->get_offset(); if (off >= 0 && (size_t)off + n <= sizeof(q)) memcpy(q + off, v, n); };
            const double loc[3] = {(double)gun.x, (double)gun.y, (double)gun.z};
            const double rot[3] = {0.0, 0.0, 0.0};
            lput(L"AkEvent", &evo, sizeof(void*));
            lput(L"Location", loc, sizeof(loc));
            lput(L"orientation", rot, sizeof(rot));
            lput(L"WorldContextObject", &wpn, sizeof(void*));
            if (lok) {
                cdo->call_function(L"PostEventAtLocation", q);
                if (auto* pr = lf->find_property(L"ReturnValue")) playing = *reinterpret_cast<uint32_t*>(q + pr->get_offset());
            }
        } else if (!have_gun) {
            API::get()->log_info("[Halo-CampE-UEVR] WWISE step %s: no gun location, falling back to the pawn", step);
            cdo->call_function(L"PostEvent", p);
            if (auto* pr = fn->find_property(L"ReturnValue")) playing = *reinterpret_cast<uint32_t*>(p + pr->get_offset());
        }
    } else if (!posted && g_cfg.reload_step_via == 1) {
        // The event's own PostOnActor(Actor, PostEventCallback, CallbackMask, bStopWhenAttachedObjectDestroyed).
        auto* pf = evo->get_class() ? evo->get_class()->find_function(L"PostOnActor") : nullptr;
        alignas(16) uint8_t q[256] = {0};
        if (pf != nullptr && (size_t)pf->get_properties_size() <= sizeof(q)) {
            if (auto* pr = pf->find_property(L"Actor")) memcpy(q + pr->get_offset(), &wpn, sizeof(void*));
            evo->call_function(L"PostOnActor", q);
            if (auto* pr = pf->find_property(L"ReturnValue")) playing = *reinterpret_cast<uint32_t*>(q + pr->get_offset());
        }
    } else if (!posted) {
        cdo->call_function(L"PostEvent", p);
        if (auto* pr = fn->find_property(L"ReturnValue")) playing = *reinterpret_cast<uint32_t*>(p + pr->get_offset());
    }
    t_ak_our_post = false;
    if (idp != nullptr) *idp = 0;
    if (g_cfg.reload_log || g_cfg.reload_wwise_dump) {
        float maxdur = -1.0f, atten = -1.0f;
        if (auto* d = evo->get_property_data<float>(L"MaximumDuration")) if (!IsBadReadPtr(d, 4)) maxdur = *d;
        if (auto* a = evo->get_property_data<float>(L"MaxAttenuationRadius")) if (!IsBadReadPtr(a, 4)) atten = *a;
        // Where the post landed: the actor's distance from the camera, the fact that decides audibility.
        Vec3 loc{}; float dist = -1.0f;
        if (have_gun) loc = gun;
        if (have_gun || call_ret_vec3(on, L"K2_GetActorLocation", &loc)) {
            const float dx = loc.x - g_cam_x.load(std::memory_order_relaxed), dy = loc.y - g_cam_y.load(std::memory_order_relaxed), dz = loc.z - g_cam_z.load(std::memory_order_relaxed);
            dist = std::sqrt(dx * dx + dy * dy + dz * dz) / 100.0f;
        }
        API::get()->log_info("[Halo-CampE-UEVR] WWISE step %s: posted '%s' via %d on %ls -> playing id %u (maxdur %.2fs, atten %.0f, %s %.2f m from camera)", step, ev.c_str(), g_cfg.reload_step_via, have_gun ? L"the gun's location" : class_name_of(on).c_str(), playing, maxdur, atten, have_gun ? "gun" : "actor", dist);
    }
}

void audio_dump_begin() {
    if (!g_cfg.reload_audio_dump) return;
    s_ad_at = now_ticks();
    s_ad_phase = 1;
    weapon_components([&](API::UObject* c) {
        if (class_name_of(c).find(L"Audio") != std::wstring::npos) { audio_dump_fields(c); return false; }
        return true;
    });
}
void audio_dump_tick() {
    if (s_ad_phase == 0) return;
    const long long since = now_ticks() - s_ad_at;
    const int want_ms = (s_ad_phase == 1) ? 120 : 400;
    if (since < ms_to_ticks(want_ms)) return;
    auto* arr = API::get()->get_uobject_array();
    auto* wpn = fp_weapon_actor();
    int found = 0, playing = 0;
    if (arr != nullptr) {
        const int32_t nn = arr->get_object_count();
        for (int32_t i = 0; i < nn && playing < 24; ++i) {
            auto* o = static_cast<API::UObject*>(arr->get_object(i));
            if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
            const std::wstring cl = class_name_of(o);
            if (cl.find(L"AudioComponent") == std::wstring::npos) continue;
            ++found;
            bool is_playing = false;
            { alignas(16) uint8_t p[64] = {0}; o->call_function(L"IsPlaying", p); is_playing = p[0] != 0; }
            if (!is_playing) continue;
            ++playing;
            API::UObject* owner = nullptr;
            { alignas(16) uint8_t p[64] = {0}; o->call_function(L"GetOwner", p); owner = *reinterpret_cast<API::UObject**>(p); }
            std::wstring snd = L"-";
            if (auto** ps = o->get_property_data<API::UObject*>(L"Sound"))
                if (!IsBadReadPtr(ps, sizeof(void*)) && *ps != nullptr && !IsBadReadPtr(*ps, sizeof(void*))) snd = (*ps)->get_full_name();
            const auto* fn = o->get_fname();
            API::get()->log_info("[Halo-CampE-UEVR] AUDIODUMP +%dms PLAYING %ls '%ls' owner=%ls%s sound=%ls",
                                 want_ms, cl.c_str(), fn ? fn->to_string().c_str() : L"?",
                                 owner ? class_name_of(owner).c_str() : L"(none)",
                                 (owner != nullptr && owner == wpn) ? " [FP WEAPON]" : "",
                                 snd.c_str());
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] AUDIODUMP +%dms: %d audio components, %d playing", want_ms, found, playing);
    s_ad_phase = (s_ad_phase == 1) ? 2 : 0;
}

// ---- ANIMVARSET (dev): write one named variable on the weapon's anim instance every tick while
// the key is set -- "animvarset=MagnumOverlay_Bool,1". The palette sweep (2026-09-03) showed the
// weapon's parts are UE bones the AnimBP drives, so its variables are the only handle on the
// slide; this is how a candidate is tried in the headset without a build per guess.
void anim_var_set_tick() {
    if (g_cfg.anim_var_set[0] == 0) return;
    static std::string s_last;
    const std::string spec = g_cfg.anim_var_set;
    const size_t comma = spec.find(',');
    if (comma == std::string::npos) return;
    const std::string name(spec.substr(0, comma));
    const double value = atof(spec.c_str() + comma + 1);
    auto* animbp = reload_weapon_anim_instance();
    if (animbp == nullptr) return;
    auto* cls = animbp->get_class();
    if (cls == nullptr) return;
    const std::wstring wname(name.begin(), name.end());
    auto* prop = cls->find_property(wname.c_str());
    if (prop == nullptr) {
        if (spec != s_last) { s_last = spec; API::get()->log_info("[Halo-CampE-UEVR] ANIMVARSET: no property '%s' on %ls", name.c_str(), class_name_of(animbp).c_str()); }
        return;
    }
    const std::wstring pcls = prop->get_class() ? prop->get_class()->get_name() : L"?";
    uint8_t* p = reinterpret_cast<uint8_t*>(animbp) + prop->get_offset();
    if (IsBadWritePtr(p, 8)) return;
    if      (pcls == L"BoolProperty")   static_cast<API::FBoolProperty*>(prop)->set_value_in_object(animbp, value != 0.0);
    else if (pcls == L"ByteProperty" || pcls == L"EnumProperty") *p = (uint8_t)value;
    else if (pcls == L"IntProperty")    *reinterpret_cast<int32_t*>(p) = (int32_t)value;
    else if (pcls == L"FloatProperty")  *reinterpret_cast<float*>(p)   = (float)value;
    else if (pcls == L"DoubleProperty") *reinterpret_cast<double*>(p)  = value;
    if (spec != s_last) {
        s_last = spec;
        API::get()->log_info("[Halo-CampE-UEVR] ANIMVARSET: %s (%ls) = %.3f on %ls, every tick", name.c_str(), pcls.c_str(), value, class_name_of(animbp).c_str());
    }
}

// ---- AMMOSEQ / AMMOSCRUB (dev, 2026-09-03). The weapon AnimBP layers an "ammunition" pose onto
// the gun from the AnimSequence in its FirstPersonPrimaryAmmunition slot, sampled at the frame in
// PrimaryAmmunition_ExplicitFrame (measured: 59 on a loaded pistol, 0 = slide locked back, and
// writing 0 every tick DOES render). That is a per-frame scrub of a sequence the ABP already
// applies to the weapon mesh -- so point the slot at a sequence whose frames carry slide TRAVEL
// (the reloads, the fire) and sweep the frame. ammoseq swaps the slot every tick and restores
// the game's own sequence when cleared; ammoscrub sweeps the frame 0..N-1 at that many frames
// per second, N from the sequence's own frame count (logged once, with its length fields).
TrackedObject s_aq_seq, s_aq_orig, s_aq_inst;
std::string   s_aq_last;
int32_t       s_aq_off = -1;
int           s_aq_frames = 0;
void ammo_seq_tick() {
    auto* animbp = reload_weapon_anim_instance();
    std::string spec = g_cfg.ammo_seq;
    // String values arrive with the file's line end still attached (2026-09-03: the name carried
    // a stray CR and matched nothing, and the miss retried a full object walk EVERY tick).
    while (!spec.empty() && (spec.back() == '\r' || spec.back() == '\n' || spec.back() == ' ' || spec.back() == '\t')) spec.pop_back();
    if (spec == "0" || _stricmp(spec.c_str(), "off") == 0) spec.clear();
    if (spec.empty()) {
        if (s_aq_off >= 0) {
            if (auto* inst = s_aq_inst.get()) {
                auto** pp = reinterpret_cast<API::UObject**>(reinterpret_cast<uint8_t*>(inst) + s_aq_off);
                if (!IsBadWritePtr(pp, sizeof(void*))) *pp = s_aq_orig.get();
            }
            API::get()->log_info("[Halo-CampE-UEVR] AMMOSEQ: slot restored to the game's sequence");
            s_aq_off = -1; s_aq_last.clear(); s_aq_frames = 0;
            s_aq_seq = TrackedObject{}; s_aq_orig = TrackedObject{}; s_aq_inst = TrackedObject{};
        }
    } else if (animbp != nullptr && (spec != s_aq_last || s_aq_inst.get() != animbp)) {
        s_aq_last = spec;
        s_aq_off = -1;
        s_aq_inst.set(animbp);   // a miss below is final for this spec on this instance: no per-tick retry
        // The slot: the ObjectProperty on the instance's class chain whose name ends in
        // FirstPersonPrimaryAmmunition (the dump prints it as Animations/FirstPersonPrimaryAmmunition).
        int32_t off = -1;
        for (API::UStruct* st = animbp->get_class(); st != nullptr && off < 0; st = st->get_super_struct()) {
            if (IsBadReadPtr(st, sizeof(void*))) break;
            for (API::FField* f = st->get_child_properties(); f != nullptr; f = f->get_next()) {
                if (IsBadReadPtr(f, sizeof(void*))) break;
                auto* fc = f->get_class();
                if (fc == nullptr || fc->get_name() != L"ObjectProperty") continue;
                const auto* fn = f->get_fname();
                if (fn == nullptr) continue;
                const std::wstring nm = fn->to_string();
                static const std::wstring want = L"FirstPersonPrimaryAmmunition";
                if (nm.size() >= want.size() && nm.compare(nm.size() - want.size(), want.size(), want) == 0) {
                    off = static_cast<API::FProperty*>(f)->get_offset(); break;
                }
            }
        }
        if (off < 0) { API::get()->log_info("[Halo-CampE-UEVR] AMMOSEQ: no FirstPersonPrimaryAmmunition slot on %ls", class_name_of(animbp).c_str()); return; }
        auto* seq = find_anim_sequence(spec);
        if (seq == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] AMMOSEQ: no AnimSequence matching '%s'", spec.c_str()); return; }
        auto** pp = reinterpret_cast<API::UObject**>(reinterpret_cast<uint8_t*>(animbp) + off);
        if (IsBadWritePtr(pp, sizeof(void*))) return;
        s_aq_off = off; s_aq_inst.set(animbp); s_aq_orig.set(*pp); s_aq_seq.set(seq);
        // The sequence's own length fields, and the frame count the sweep uses.
        float len = 0.0f; int frames = 0, keys = 0;
        for (API::UStruct* st = seq->get_class(); st != nullptr; st = st->get_super_struct()) {
            if (IsBadReadPtr(st, sizeof(void*))) break;
            for (API::FField* f = st->get_child_properties(); f != nullptr; f = f->get_next()) {
                if (IsBadReadPtr(f, sizeof(void*))) break;
                auto* fc = f->get_class();
                const auto* fn = f->get_fname();
                if (fc == nullptr || fn == nullptr) continue;
                const std::wstring cls = fc->get_name();
                const std::wstring nm  = fn->to_string();
                const int32_t o = static_cast<API::FProperty*>(f)->get_offset();
                const uint8_t* q = reinterpret_cast<const uint8_t*>(seq) + o;
                if (IsBadReadPtr(q, 8)) continue;
                double v = 0.0; bool num = true;
                if      (cls == L"FloatProperty")  v = *reinterpret_cast<const float*>(q);
                else if (cls == L"DoubleProperty") v = *reinterpret_cast<const double*>(q);
                else if (cls == L"IntProperty")    v = *reinterpret_cast<const int32_t*>(q);
                else num = false;
                if (!num) continue;
                if (nm.find(L"Length") != std::wstring::npos || nm.find(L"Frame") != std::wstring::npos ||
                    nm.find(L"Key") != std::wstring::npos || nm.find(L"Rate") != std::wstring::npos) {
                    API::get()->log_info("[Halo-CampE-UEVR] AMMOSEQ   %ls (%ls@0x%X) = %.3f", nm.c_str(), cls.c_str(), (unsigned)o, v);
                }
                if (nm == L"SequenceLength") len = (float)v;
                if (nm == L"NumberOfSampledFrames" || nm == L"NumFrames") frames = (int)v;
                if (nm == L"NumberOfSampledKeys" || nm == L"NumberOfKeys") keys = (int)v;
            }
        }
        s_aq_frames = frames > 1 ? frames : (keys > 1 ? keys : (len > 0.0f ? (int)(len * 30.0f + 0.5f) : 60));
        API::get()->log_info("[Halo-CampE-UEVR] AMMOSEQ: slot @0x%X %ls -> %ls, sweep uses %d frames (len %.3f s)",
                             (unsigned)off, s_aq_orig.get() ? s_aq_orig.get()->get_full_name().c_str() : L"null",
                             seq->get_full_name().c_str(), s_aq_frames, len);
    }
    if (!spec.empty() && s_aq_off >= 0 && animbp != nullptr && s_aq_inst.get() == animbp) {
        auto** pp = reinterpret_cast<API::UObject**>(reinterpret_cast<uint8_t*>(animbp) + s_aq_off);
        if (!IsBadWritePtr(pp, sizeof(void*)) && s_aq_seq.get() != nullptr) *pp = s_aq_seq.get();
    }
    if (g_cfg.ammo_scrub > 0.0f && animbp != nullptr) {
        auto* p = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame");
        if (p != nullptr && !IsBadWritePtr(p, sizeof(int32_t))) {
            static long long s_t0 = 0;
            if (s_t0 == 0) s_t0 = now_ticks();
            const double sec = (double)(now_ticks() - s_t0) / (double)ms_to_ticks(1000);
            const int n = s_aq_frames > 1 ? s_aq_frames : 60;
            const int frame = (int)(sec * (double)g_cfg.ammo_scrub) % n;
            *p = frame;
            static long long s_said = 0;
            if (now_ticks() - s_said > ms_to_ticks(500)) {
                s_said = now_ticks();
                API::get()->log_info("[Halo-CampE-UEVR] AMMOSCRUB: frame %d of %d", frame, n);
            }
        }
    }
}

// ---- THE SLIDE RACK (slidevr). The pistol's slide is node `slidenode` of the weapon object's
// own node block (BlamPalette publishes its world position and forward every sim tick, and
// applies g_slide_pull to it). Here, on the game thread: the off hand's grip closing within
// slideradius of the slide latches it; the pull is the hand's travel since the grab projected
// onto -forward (the barrel, muzzle-ward being +forward), clamped to slidetravel; releasing the
// grip springs it home. Distances are compared in UE centimetres: the hand goes room->world
// through the holster transform, the slide comes Blam->UE by the 304.8 unit with Y negated.
// The slide PART's rendered centre in UE world cm, published by the parts rebuild each tick
// (valid only while a slide part exists). The grab zone lives there (slide_zone=1).
std::atomic<bool>  g_sl_part_valid{false};
// PER-WEAPON ZONE PUSHBACK (from the headset, 2026-09-04): "@back=0.03" on a slidebones entry, metres
// further back along the barrel from the part's centre, added to the global slidezoneback. The
// magnum's serrations sit behind Slide_M's centre; nothing else needed the push.
std::atomic<float> g_sl_zone_back{0.0f};
std::atomic<float> g_sl_zone_up{0.0f};
std::atomic<float> g_sl_zone_right{0.0f};
std::atomic<bool>  g_sl_zone_reload_only{false};   // @reloadonly: the rack works only in the reload state (the shotgun's pump is its foregrip)
std::atomic<bool>  g_sl_zone_every_shot{false};    // @everyshot: every shot locks the gun until the rack (a pump between shots, from the headset 2026-09-06)
std::atomic<bool>  g_sl_zone_pump{false};          // @pump: back all the way then FORWARD all the way is the cycle; no grip release, no re-grab
std::atomic<bool>  g_sl_zone_pull_down{false};     // @pulldown: the pull is measured DOWNWARD, not rearward (the launcher's clamp is pulled down)
std::atomic<float> g_sl_insert{-1.0f};             // @insert=m: this weapon's own mag-in start below the seat (-1 = the global reloadinsert)
float reload_insert_for_weapon() { const float v = g_sl_insert.load(std::memory_order_relaxed); return v >= 0.0f ? v : g_cfg.reload_insert; }
std::atomic<float> g_sl_part_x{0.0f}, g_sl_part_y{0.0f}, g_sl_part_z{0.0f};
bool  s_sl_held = false;
bool  s_sl_racked = false;
bool  s_sl_grip_prev = false;
// THE MANUAL LOOP (slidelockreload): after a magazine seats, the slide is held locked back
// (the AnimBP's two-state ammunition pose, frame 0) and the trigger is dead until the player
// racks it -- pulled to the end and released. Then the pose goes back to what the game set
// (captured the moment the reload's ammo landed, so it is the weapon's own loaded frame, not a
// constant) and the gun fires. Mag in -> rack -> fire. Local presentation plus a local trigger
// block; the game's reload and ammo are untouched, so co-op is untouched.
bool    s_sl_th = false;          // the rack is driven by the two-hand SUPPORT hand: grabbed while holding, completed on the forward return
bool    s_sl_lock_pending = false;
int32_t s_sl_seen_frame = 0;   // last non-zero ammo frame seen on this weapon while idle
bool    s_sl_locked_back = false; // an EMPTY gun's slide stays back from the drop until the rack (the phantom flag no longer covers it)
// A LOADED CHAMBER (from the headset, 2026-09-05: "phantom round doesn't fire"): the sim refuses the
// trigger while its own reload runs, so a press at the drop would eat the chambered shot. On a
// gun with a round in it the press waits for that shot (150 ms after it, so the sim fires
// first) or for the seat, whichever comes first. An empty gun still presses at the drop.
bool      s_sl_press_pending = false;
long long s_sl_press_due_at = 0;
bool      s_sl_pressed_early = false;   // reload_press_at 1: the press went out at the drop
long long s_sl_press_at = 0;            // when the plugin last pressed the game's reload
int32_t s_sl_lock_frame = 0;
// The loaded (forward) ammo frame: the one captured during the lock, else the last one seen idle,
// else 1 -- the pose is two-state and anything but 0 renders forward.
int32_t slide_forward_frame() { if (s_sl_lock_frame != 0) return s_sl_lock_frame; if (s_sl_seen_frame != 0) return s_sl_seen_frame; return 1; }
bool    s_sl_rack_done = false;
bool    s_sl_empty_at_drop = false;   // the pistol was empty (slide locked back) when the mag left
bool    s_sl_reload_due = false;      // SLIDECHAMBER: a seated mag waiting for the rack to fire the reload
bool    s_true_empty = false;         // SLIDEPHANTOM: the gun is empty while the sim reads one round
// ONE CHAMBERED ROUND. With the magazine out a pistol fires exactly once (the round already in
// the chamber), then nothing until a fresh magazine is seated. Counted on trigger press edges
// while the reload is in flight; the press itself is seen through the ForceTube's fire stamp.
int     s_sl_chamber_left = 0;
bool    s_sl_trig_prev = false;
// The weapon AnimBP's ammunition frame: 0 = empty / slide locked back, else loaded. -1 = none.
int32_t slide_ammo_frame() {
    auto* animbp = reload_weapon_anim_instance();
    if (animbp == nullptr) return -1;
    auto* p = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame");
    if (p == nullptr || IsBadReadPtr(p, sizeof(int32_t))) return -1;
    return *p;
}
Vec3  s_sl_grab_ue{};
long long s_sl_release_at = 0;
float s_sl_release_pull = 0.0f;
inline Vec3 blam_to_ue_cm(float bx, float by, float bz) { return Vec3{bx * 304.8f, -by * 304.8f, bz * 304.8f}; }
inline Vec3 blam_dir_to_ue(float bx, float by, float bz) { return Vec3{bx, -by, bz}; }
std::string trim_cfg(const char* v);   // defined with the montage tools below
// ---- THE GESTURE FRAME (reloadframe, Config.hpp). The world-space tests compare weapon
// component positions read on the GAME tick against a hand mapped through the RENDERED camera;
// while sprinting those time bases are up to a tick apart and the measured distance oscillates
// by the camera's per-tick travel. These helpers give the tests one time base.
namespace {
Vec3 s_gc_cam{}; bool s_gc_have = false; unsigned long long s_gc_at = ~0ull;
Vec3 s_gc_prev{}; bool s_gc_prev_have = false; float s_gc_travel = 0.0f;
}
// The GAME camera, fetched at most once per ~tick (ms-keyed; engine ticks are ~31 ms apart) on
// the game thread -- the same time base as every K2_GetComponentLocation/Bounds the tests read.
bool gesture_game_cam(Vec3* out) {
    const unsigned long long nowms = GetTickCount64();
    if (nowms != s_gc_at) {
        s_gc_at = nowms;
        s_gc_have = false;
        if (auto* pc = API::get()->get_player_controller(0)) {
            if (auto** cm = pc->get_property_data<API::UObject*>(L"PlayerCameraManager")) {
                if (!IsBadReadPtr(cm, sizeof(void*)) && *cm != nullptr) {
                    Vec3 v{};
                    if (call_ret_vec3(*cm, L"GetCameraLocation", &v)) { s_gc_cam = v; s_gc_have = true; }
                }
            }
        }
        if (s_gc_have) {
            if (s_gc_prev_have) {
                const float dx = s_gc_cam.x - s_gc_prev.x, dy = s_gc_cam.y - s_gc_prev.y, dz = s_gc_cam.z - s_gc_prev.z;
                float t = std::sqrt(dx * dx + dy * dy + dz * dz) * 0.01f;   // cm -> m this tick
                if (t > 0.30f) t = 0.30f;   // a teleport or level load is not running
                s_gc_travel = t;
            }
            s_gc_prev = s_gc_cam; s_gc_prev_have = true;
        } else { s_gc_prev_have = false; s_gc_travel = 0.0f; }
    }
    if (s_gc_have && out != nullptr) *out = s_gc_cam;
    return s_gc_have;
}
// reloadframe=2: the pad added to the seat/rack gates, metres -- the camera's per-tick travel,
// which is the worst case of the phase error the legacy transform carries. 0 in every other mode.
float reload_gate_pad_m() {
    if (g_cfg.reload_frame != 2) return 0.0f;
    gesture_game_cam(nullptr);
    return s_gc_travel;
}
// The hand (room) into world for the seat tests: the tick's own camera when reloadframe=1 and it
// resolves, the legacy rendered-frame transform otherwise.
Vec3 reload_hand_world(const Vec3& hand_room, const Vec3& head_room) {
    Vec3 cam{};
    if (g_cfg.reload_frame == 1 && gesture_game_cam(&cam)) return holster_room_to_world_at(hand_room, head_room, cam);
    return holster_room_to_world(hand_room, head_room);
}
// The inverse, same frame choice -- the exact mirror of reload_hand_world, so a value pushed
// through both comes back bit-stable.
Vec3 reload_world_room(const Vec3& world, const Vec3& head_room) {
    Vec3 cam{};
    if (g_cfg.reload_frame == 1 && gesture_game_cam(&cam)) return holster_world_to_room_at(world, head_room, cam);
    return holster_world_to_room(world, head_room);
}
// THE WELL RIDES THE HAND (zonehandrel, Config.hpp). The magazine component's position is
// re-expressed in the aim hand's frame, low-passed, and rebuilt from the live hand pose, so the
// seat point tracks the RENDERED gun instead of the game's own sprint animation. The alpha
// (0.2/tick at ~32 Hz, ~150 ms) kills the bob but follows a real weapon swap instantly through
// the snap guard.
Vec3 reload_well_stabilize(void* comp_key, const Vec3& well_world, const Vec3& head_room) {
    if (g_cfg.zone_hand_rel == 0) return well_world;
    const auto ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                          : API::VR::get_right_controller_index();
    Vec3 ap{}; Quat aq{};
    if (ridx < 0 || !get_pose(ridx, &ap, &aq, /*use_aim=*/true)) return well_world;
    const Quat af = apply_aim_fix(aq);
    const Vec3 f = quat_forward(af);
    const Vec3 u = quat_rotate(af, Vec3{0.0f, 1.0f, 0.0f});
    const Vec3 r = quat_rotate(af, Vec3{1.0f, 0.0f, 0.0f});
    const Vec3 wr = reload_world_room(well_world, head_room);
    const Vec3 rel{wr.x - ap.x, wr.y - ap.y, wr.z - ap.z};
    const Vec3 loc{rel.x * r.x + rel.y * r.y + rel.z * r.z,
                   rel.x * u.x + rel.y * u.y + rel.z * u.z,
                   rel.x * f.x + rel.y * f.y + rel.z * f.z};
    static void* s_key = nullptr; static Vec3 s_loc{}; static bool s_have = false;
    const float dx = loc.x - s_loc.x, dy = loc.y - s_loc.y, dz = loc.z - s_loc.z;
    if (comp_key != s_key || !s_have || dx * dx + dy * dy + dz * dz > 0.25f) { s_key = comp_key; s_loc = loc; s_have = true; }
    else { const float a = 0.2f; s_loc.x += dx * a; s_loc.y += dy * a; s_loc.z += dz * a; }
    const Vec3 room{ap.x + r.x * s_loc.x + u.x * s_loc.y + f.x * s_loc.z,
                    ap.y + r.y * s_loc.x + u.y * s_loc.y + f.y * s_loc.z,
                    ap.z + r.z * s_loc.x + u.z * s_loc.y + f.z * s_loc.z};
    return reload_hand_world(room, head_room);
}
// The held weapon's mesh stem, lowercased (SK_FuelRodCannon_Default -> fuelrodcannon), cached per
// weapon key and actor. The class name and the mesh name disagree on one weapon: BP_FP_FlakCannon
// renders the fuel rod cannon, so "FuelRod" in a list never matched its key.
std::string weapon_stem_lc_for(const std::string& key) {
    static std::string s_key; static API::UObject* s_actor = nullptr; static std::string s_stem; static long long s_at = 0;
    auto* actor = fp_weapon_actor();
    const long long nowt = now_ticks();
    if (key != s_key || actor != s_actor || (s_stem.empty() && nowt - s_at > ms_to_ticks(500))) {
        s_key = key; s_actor = actor; s_at = nowt;
        s_stem = narrow(ak_weapon_stem());
    }
    return s_stem;
}
// A weapon key (or its mesh stem) against a comma-separated list of substrings (case-insensitive, trimmed).
bool weapon_in_list(const char* list) {
    const std::string key = weapon_key();
    if (key.empty()) return false;
    std::string lk = key; for (auto& ch : lk) ch = (char)tolower((unsigned char)ch);
    const std::string stem = weapon_stem_lc_for(key);
    std::string tbl = trim_cfg(list);
    size_t pos = 0;
    while (pos <= tbl.size()) {
        size_t comma = tbl.find(',', pos); if (comma == std::string::npos) comma = tbl.size();
        std::string ent = tbl.substr(pos, comma - pos);
        while (!ent.empty() && (unsigned char)ent.back() <= ' ') ent.pop_back();
        while (!ent.empty() && (unsigned char)ent.front() <= ' ') ent.erase(ent.begin());
        for (auto& ch : ent) ch = (char)tolower((unsigned char)ch);
        if (!ent.empty() && (lk.find(ent) != std::string::npos || (!stem.empty() && stem.find(ent) != std::string::npos))) return true;
        if (comma >= tbl.size()) break;
        pos = comma + 1;
    }
    return false;
}
bool slide_weapon_ok() { return weapon_in_list(g_cfg.slide_weapons); }
// The rack part the native mode found for the held weapon (set by the parts code below). A
// weapon whose rack part is missing must never be locked back or phantomed: nothing could
// finish its reload.
bool g_slide_rack_found = false;
bool slide_rack_available() { return g_slide_rack_found; }
// The pistol rules (lock-back, phantom round, rack-fired reload) apply only to these.
bool slide_chamber_ok() { return weapon_in_list(g_cfg.slide_chamber_weapons); }
void slide_haptic(float dur, float amp) {
    const bool off_right = g_cfg.aim_left_hand;
    API::VR::trigger_haptic_vibration(0.0f, dur, 0.0f, amp,
                                      off_right ? API::VR::get_right_joystick_source()
                                                : API::VR::get_left_joystick_source());
}
// THE GAME-THREAD WRITE. The sim rebuilds the slide node every tick (readback measured 2026-09-03),
// and the mesh sync reads it on the game thread during component ticks -- AFTER this pre-engine
// tick. So the pull (and the dev poke) are applied HERE, where the write is guaranteed to sit
// between the rebuild and the read. The sim-side write stays as a second bite at the same tick.
void slide_node_write_tick() {
    if (blam_capture_hook_active()) return;   // the capture pre-hook owns the write (renders)
    const uintptr_t addr = g_slide_node_addr.load(std::memory_order_relaxed);
    if (addr == 0 || IsBadWritePtr((void*)addr, 52)) return;
    struct NodeM { float scale; float fx, fy, fz; float lx, ly, lz; float ux, uy, uz; float px, py, pz; };
    auto* n = reinterpret_cast<NodeM*>(addr);
    float along = 0.0f;
    const float pull = g_slide_pull.load(std::memory_order_relaxed);
    if (pull > 0.0f && std::isfinite(pull)) along -= pull;
    if (g_cfg.wpn_node_poke >= 0) along += g_cfg.wpn_node_poke_amt;
    if (along == 0.0f) return;
    n->px += n->fx * along; n->py += n->fy * along; n->pz += n->fz * along;
    if (g_cfg.slide_log) {
        static long long s_said = 0;
        if (now_ticks() - s_said > ms_to_ticks(1000)) {
            s_said = now_ticks();
            API::get()->log_info("[Halo-CampE-UEVR] SLIDE game-thread write: along=%.4f pos=(%.3f %.3f %.3f)", along, n->px, n->py, n->pz);
        }
    }
}

const char* state_name(ReloadState s);   // defined below; the refusal line names the reload state
void slide_update(const Vec3& head) {
    auto release = [&](const char* why) {
        if (s_sl_held) {
            s_sl_release_pull = g_slide_pull.load(std::memory_order_relaxed);
            s_sl_release_at = now_ticks();
            if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDE released (%s) at pull %.4f", why, s_sl_release_pull);
            if (s_sl_racked) { slide_haptic(0.06f, 0.9f); s_sl_rack_done = true; ak_step_sound("rack"); }
        }
        s_sl_held = false; s_sl_racked = false; s_sl_th = false;
    };
    // The chambered round's shot: a trigger press edge while the reload is in flight spends it.
    {
        const long long since = now_ticks() - g_ft_fire_at.load(std::memory_order_relaxed);
        const bool trig = since >= 0 && since < ms_to_ticks(40);
        if (trig && !s_sl_trig_prev && s_reload != ReloadState::Idle && s_sl_chamber_left > 0) {
            --s_sl_chamber_left;
            if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDE chambered round fired, trigger now dead until the seat");
            if (s_sl_press_pending) { s_sl_press_pending = false; s_sl_press_due_at = now_ticks() + ms_to_ticks(150); }
            if (s_sl_chamber_left <= 0) {
                // The gun is empty NOW: the slide locks back and stays there until the rack after
                // the seat (from the headset, 2026-09-06: the chambered shot did not lock the slide).
                s_sl_empty_at_drop = true; s_sl_locked_back = true;
                if (g_cfg.slide_log || g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDE chamber empty after the shot: locked back, the seat will want a rack");
            }
        }
        s_sl_trig_prev = trig;
    }
    // The lock-back after a reload: hold the ammunition pose at frame 0 until a rack completes.
    // The game's own loaded frame is captured the first time it reads non-zero after the seat
    // (the reload's ammo landing), so the restore is the weapon's value, not a guess.
    // THE LOADED POSE (2026-09-05): PrimaryAmmunition_ExplicitFrame is two-state -- 0 = slide
    // locked back, any other value = forward (measured 1/2/8/28/56/59; it is an animation frame,
    // NOT the round count). With the press at the drop the game's own loaded frame may never be
    // seen (the state hold keeps its reload from landing one), so the last non-zero frame seen on
    // this weapon while idle is remembered, and 1 is the last resort: anything but 0 is forward.
    {
        static std::string s_seen_key; static int32_t s_seen_frame = 0;
        const std::string wk = weapon_key();
        if (wk != s_seen_key) { s_seen_key = wk; s_seen_frame = 0; }
        if (s_reload == ReloadState::Idle && !s_sl_lock_pending && !s_sl_pressed_early) {
            if (auto* animbp = reload_weapon_anim_instance())
                if (auto* p = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame"))
                    if (!IsBadReadPtr(p, sizeof(int32_t)) && *p != 0) s_seen_frame = *p;
        }
        s_sl_seen_frame = s_seen_frame;
    }
    {
        static bool s_ep_held = false;
        if (s_sl_pressed_early && s_reload != ReloadState::Idle && !s_sl_lock_pending) {
            if (auto* animbp = reload_weapon_anim_instance()) {
                if (auto* p = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame")) {
                    // An empty gun shows the slide back (0); a gun with a round chambered keeps the
                    // pose it had before the drop (the frame is the slide, not the counter).
                    if (!IsBadWritePtr(p, sizeof(int32_t))) { if (*p != 0 && s_sl_lock_frame == 0) s_sl_lock_frame = *p; *p = s_sl_empty_at_drop ? 0 : slide_forward_frame(); s_ep_held = true; }
                }
            }
        } else if (s_ep_held) {
            s_ep_held = false;
            if (!s_sl_lock_pending) {   // a chambered-round reload: the seat alone ends it, the pose goes forward now
                if (auto* animbp = reload_weapon_anim_instance())
                    if (auto* p = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame"))
                        if (!IsBadWritePtr(p, sizeof(int32_t))) *p = slide_forward_frame();
            }
        }
    }
    // Only the rack ends this lock, and with slidevr off nothing can rack (the gate below releases
    // the hand): release it as racked, so the pose goes forward and a seated magazine's reload goes
    // out instead of s_sl_reload_due waiting forever behind a frozen slide.
    if (!g_cfg.slide_vr && s_sl_lock_pending) {
        s_sl_rack_done = true;
        if (reload_weapon_anim_instance() == nullptr) { s_sl_lock_pending = false; s_sl_rack_done = false; s_sl_locked_back = false; }
    }
    if (s_sl_lock_pending) {
        if (auto* animbp = reload_weapon_anim_instance()) {
            if (auto* p = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame")) {
                if (!IsBadWritePtr(p, sizeof(int32_t))) {
                    if (s_sl_rack_done) {
                        *p = slide_forward_frame();
                        s_sl_lock_pending = false; s_sl_rack_done = false; s_sl_locked_back = false;
                        if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDE rack complete, pose restored to frame %d, fire unblocked", s_sl_lock_frame);
                    } else {
                        if (*p != 0 && s_sl_lock_frame == 0) s_sl_lock_frame = *p;
                        *p = 0;
                    }
                }
            }
        }
    } else {
        s_sl_rack_done = false;
    }
    // The spring home after a release: a short ease from the released pull to zero.
    if (!s_sl_held && s_sl_release_at != 0) {
        const float ms = (float)std::chrono::duration_cast<std::chrono::milliseconds>(
            clock_t_::duration(now_ticks() - s_sl_release_at)).count();
        const float t = std::fmin(1.0f, ms / 90.0f);
        g_slide_pull.store(s_sl_release_pull * (1.0f - t), std::memory_order_relaxed);
        if (t >= 1.0f) { s_sl_release_at = 0; g_slide_pull.store(0.0f, std::memory_order_relaxed); }
    }
    if (!g_cfg.slide_vr || !g_slide_node_valid.load(std::memory_order_relaxed) || !slide_weapon_ok()) {
        release("unavailable"); return;
    }
    // THE ZONE IS AIM-HAND-RELATIVE, NOT THE NODE'S WORLD POSITION. The published node sits on
    // the third-person gun in the biped's hand (measured: never nearer than ~40 cm to the hand
    // actually on the rendered slide); the mesh sync copies the node's MOTION into the
    // first-person mesh, not its place. So the slide is where the rendered gun is: slideoff
    // metres from the aim hand in the aim-fixed frame (right, up, forward), the same recipe as
    // the reload well's fallback. The pull is measured along the aim direction in room space.
    const auto idx = g_cfg.aim_left_hand ? API::VR::get_right_controller_index()
                                         : API::VR::get_left_controller_index();
    const auto ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                          : API::VR::get_right_controller_index();
    Vec3 hp{}; Quat hq{};
    Vec3 ap{}; Quat aq{};
    if (idx < 0 || ridx < 0 || !get_pose(idx, &hp, &hq, /*use_aim=*/false) ||
        !get_pose(ridx, &ap, &aq, /*use_aim=*/true)) { release("no hand"); return; }
    const Quat af = apply_aim_fix(aq);
    const Vec3 f = quat_forward(af);
    const Vec3 u = quat_rotate(af, Vec3{0.0f, 1.0f, 0.0f});
    const Vec3 r = quat_rotate(af, Vec3{1.0f, 0.0f, 0.0f});
    Vec3 zone{ap.x + r.x * g_cfg.slide_off[0] + u.x * g_cfg.slide_off[1] + f.x * g_cfg.slide_off[2],
              ap.y + r.y * g_cfg.slide_off[0] + u.y * g_cfg.slide_off[1] + f.y * g_cfg.slide_off[2],
              ap.z + r.z * g_cfg.slide_off[0] + u.z * g_cfg.slide_off[1] + f.z * g_cfg.slide_off[2]};
    if (g_cfg.slide_zone == 1 && g_sl_part_valid.load(std::memory_order_relaxed)) {
        // The part's own rendered centre, brought into room space, then a little further back
        // along the aim direction to where the serrations are.
        const Vec3 pw{g_sl_part_x.load(std::memory_order_relaxed), g_sl_part_y.load(std::memory_order_relaxed), g_sl_part_z.load(std::memory_order_relaxed)};
        Vec3 gcam{};
        Vec3 pr = (g_cfg.reload_frame == 1 && gesture_game_cam(&gcam))
                            ? holster_world_to_room_at(pw, head, gcam)
                            : holster_world_to_room(pw, head);
        // HAND-ANCHORED (zonehandrel, Config.hpp): the part offset lives in the aim hand's
        // frame, low-passed; the zone rebuilds from the live hand, so it rides the gun you SEE
        // and not the game's sprint animation. Snap guard for weapon swaps.
        if (g_cfg.zone_hand_rel != 0) {
            const Vec3 rel{pr.x - ap.x, pr.y - ap.y, pr.z - ap.z};
            const Vec3 loc{rel.x * r.x + rel.y * r.y + rel.z * r.z,
                           rel.x * u.x + rel.y * u.y + rel.z * u.z,
                           rel.x * f.x + rel.y * f.y + rel.z * f.z};
            static Vec3 s_zl{}; static bool s_zh = false;
            const float dx = loc.x - s_zl.x, dy = loc.y - s_zl.y, dz = loc.z - s_zl.z;
            if (!s_zh || dx * dx + dy * dy + dz * dz > 0.25f) { s_zl = loc; s_zh = true; }
            else { const float a = 0.2f; s_zl.x += dx * a; s_zl.y += dy * a; s_zl.z += dz * a; }
            pr = Vec3{ap.x + r.x * s_zl.x + u.x * s_zl.y + f.x * s_zl.z,
                      ap.y + r.y * s_zl.x + u.y * s_zl.y + f.y * s_zl.z,
                      ap.z + r.z * s_zl.x + u.z * s_zl.y + f.z * s_zl.z};
        }
        const float back = g_cfg.slide_zone_back + g_sl_zone_back.load(std::memory_order_relaxed);
        const float up = g_sl_zone_up.load(std::memory_order_relaxed), right = g_sl_zone_right.load(std::memory_order_relaxed);
        zone = Vec3{pr.x - f.x * back + u.x * up + r.x * right, pr.y - f.y * back + u.y * up + r.y * right, pr.z - f.z * back + u.z * up + r.z * right};
    }
    // The zone dot (slidemarker): where the grab is, so slideoff can be set by eye.
    {
        static TrackedObject s_dot;
        static long long s_dot_try = 0;
        auto* d = s_dot.get();
        if (g_cfg.slide_marker) {
            if (d == nullptr && now_ticks() - s_dot_try > ms_to_ticks(2000)) {
                s_dot_try = now_ticks();
                if (auto* owner = API::get()->get_local_pawn(0)) {
                    static const wchar_t* kDot[] = { L"StaticMesh /Engine/BasicShapes/Sphere.Sphere" };
                    { const double ds = (double)g_cfg.slide_marker_size; d = marker_spawn_list(owner, kDot, 1, ds, ds, ds); }
                    if (d != nullptr) { marker_tint(d, g_cfg.slide_marker_color); s_dot.set(d); }
                    API::get()->log_info("[Halo-CampE-UEVR] SLIDE zone dot %s", d ? "spawned" : "FAILED to spawn (no sphere mesh?)");
                }
            }
            if (d != nullptr) { holster_marker_place(d, holster_room_to_world(zone, head)); holster_marker_show(d, true); marker_render_anchor(d, zone); }
        } else if (d != nullptr) {
            holster_marker_show(d, false);
            marker_render_drop(d);
        }
    }
    const bool grip = holster_grip_held(g_cfg.aim_left_hand);
    const bool pressed = grip && !s_sl_grip_prev;
    s_sl_grip_prev = grip;
    const float dx = hp.x - zone.x, dy = hp.y - zone.y, dz = hp.z - zone.z;
    const float dist_m = std::sqrt(dx * dx + dy * dy + dz * dz);
    // The rack is LIVE when the weapon has a part and, for a @reloadonly weapon, only in the
    // reload state (a shell in, the mag out, or the lock waiting for the rack).
    const bool rack_live = (g_slide_rack_found || g_sl_part_valid.load(std::memory_order_relaxed))
                        && (!g_sl_zone_reload_only.load(std::memory_order_relaxed) || s_sl_lock_pending || s_reload != ReloadState::Idle || s_true_empty || s_sl_locked_back);
    g_slide_zone_hot.store(rack_live && dist_m <= g_cfg.slide_radius + reload_gate_pad_m(), std::memory_order_relaxed);
    if (g_cfg.slide_log) {
        static uint32_t s_n = 0;
        if ((s_n++ % 30u) == 0u)
            API::get()->log_info("[Halo-CampE-UEVR] SLIDE hand-to-zone=%.0fcm held=%d pull=%.4f", dist_m * 100.0f, (int)s_sl_held, g_slide_pull.load());
    }
    // THE PUMP WITH THE HOLD (from the headset, 2026-09-06: two-handing and pumping work together): while
    // the two-hand hold is latched and the rack is live, the support hand entering the zone
    // takes the rack without a new press; pulling back racks it, and the FORWARD return completes
    // it (there is no grip release to wait for). A hand still in the zone grabs again for the
    // next cycle.
    if (!s_sl_held && grip && rack_live && (two_hand_latched() || g_sl_zone_pump.load(std::memory_order_relaxed)) && dist_m <= g_cfg.slide_radius * 1.5f && s_reload == ReloadState::Idle && !holster_offhand_busy()) {
        s_sl_held = true; s_sl_th = true; s_sl_racked = false; s_sl_grab_ue = hp; s_sl_release_at = 0;
        slide_haptic(0.04f, 0.5f);
        if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDE grabbed by the support hand (two-hand hold) at %.0fcm", dist_m * 100.0f);
    }
    if (!s_sl_held) {
        if (pressed && dist_m <= g_cfg.slide_radius && rack_live) {
            const bool th   = two_hand_latched();
            const bool busy = holster_offhand_busy();
            if (s_reload == ReloadState::Idle && !th && !busy) {
                s_sl_held = true; s_sl_racked = false; s_sl_grab_ue = hp; s_sl_release_at = 0;
                slide_haptic(0.04f, 0.5f);
                if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDE grabbed at %.0fcm", dist_m * 100.0f);
            } else if (g_cfg.slide_log) {
                // A grip inside the zone that did not take: name the gate, do not guess it.
                API::get()->log_info("[Halo-CampE-UEVR] SLIDE grab REFUSED at %.0fcm: reload=%s twohand=%d offhand-busy=%d",
                                     dist_m * 100.0f, state_name(s_reload), (int)th, (int)busy);
            }
        }
        return;
    }
    if (!grip) { release("grip opened"); return; }
    // Pull: hand travel since the grab along -aim forward (rearward), metres -> Blam units.
    const float tx = hp.x - s_sl_grab_ue.x, ty = hp.y - s_sl_grab_ue.y, tz = hp.z - s_sl_grab_ue.z;
    float pull_cm = g_sl_zone_pull_down.load(std::memory_order_relaxed)
                  ? -(tx * u.x + ty * u.y + tz * u.z) * 100.0f * g_cfg.slide_sign    // @pulldown: downward travel
                  : -(tx * f.x + ty * f.y + tz * f.z) * 100.0f * g_cfg.slide_sign;
    if (pull_cm < 0.0f) pull_cm = 0.0f;
    const float travel_cm = g_cfg.slide_travel * 304.8f;
    if (pull_cm > travel_cm) pull_cm = travel_cm;
    g_slide_pull.store(pull_cm / 304.8f, std::memory_order_relaxed);
    // A locked-back slide only needs the slide RELEASE: a quarter of the travel and let go.
    const bool at_end = pull_cm >= travel_cm * (s_sl_lock_pending ? 0.25f : 0.92f);
    if (at_end && !s_sl_racked) {
        s_sl_racked = true;
        slide_haptic(0.08f, 1.0f);
        ak_step_sound("rackback");
        if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDE racked (%.1fcm)", pull_cm);
    } else if (!at_end && s_sl_racked && pull_cm < travel_cm * (s_sl_th ? 0.15f : 0.5f)) {
        if (s_sl_th) { release("pump returned"); return; }   // back all the way, then FORWARD all the way: the cycle
        s_sl_racked = false;
    }
}

// ---- SLIDEMONTAGE. The first-person slide lives in the weapon's Animation Blueprint, so the
// rack poses it there: PlaySlotAnimationAsDynamicMontage on the weapon's anim instance with a
// real weapon sequence at play rate ~0, then Montage_SetPosition every tick to the time the hand
// asks for. Parameters are placed by NAME through the UFunction's own property offsets (logged
// once), never by an assumed layout. A montage only poses bones through a Slot node the ABP
// actually has; the ABP's slot-named node properties are listed once so the slot name is a
// fact, not a guess.
std::string trim_cfg(const char* v) {
    std::string s = v;
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' ' || s.back() == '\t')) s.pop_back();
    return s;
}
TrackedObject s_sm_seq, s_sm_mont, s_sm_inst;
std::string   s_sm_seq_last;
float         s_sm_len = 0.0f;
bool          s_sm_active = false;
bool          s_sm_slots_listed = false;
bool sm_put(API::UFunction* fn, uint8_t* p, size_t cap, const wchar_t* name, const void* v, size_t n, bool* ok) {
    auto* pr = fn->find_property(name);
    if (pr == nullptr) { *ok = false; API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE: no parameter '%ls'", name); return false; }
    const int32_t off = pr->get_offset();
    if (off < 0 || (size_t)off + n > cap) { *ok = false; return false; }
    memcpy(p + off, v, n);
    return true;
}
void sm_list_slots(API::UObject* animbp) {
    if (s_sm_slots_listed) return;
    s_sm_slots_listed = true;
    int n = 0;
    for (API::UStruct* st = animbp->get_class(); st != nullptr && n < 40; st = st->get_super_struct()) {
        if (IsBadReadPtr(st, sizeof(void*))) break;
        for (API::FField* f = st->get_child_properties(); f != nullptr && n < 40; f = f->get_next()) {
            if (IsBadReadPtr(f, sizeof(void*))) break;
            const auto* fn = f->get_fname();
            auto* fc = f->get_class();
            if (fn == nullptr || fc == nullptr) continue;
            const std::wstring nm = fn->to_string();
            if (nm.find(L"Slot") == std::wstring::npos) continue;
            const int32_t off = static_cast<API::FProperty*>(f)->get_offset();
            // FAnimNode_Slot begins with an FPoseLink; the SlotName FName follows it. Two
            // candidate offsets are printed and the one that reads as a name is the answer.
            std::wstring a = L"?", b = L"?";
            const uint8_t* base = reinterpret_cast<const uint8_t*>(animbp) + off;
            if (!IsBadReadPtr(base, 0x28)) {
                a = reinterpret_cast<const API::FName*>(base + 0x10)->to_string();
                b = reinterpret_cast<const API::FName*>(base + 0x18)->to_string();
            }
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE slot-node %ls (%ls@0x%X): name@+0x10='%ls' name@+0x18='%ls'",
                                 nm.c_str(), fc->get_name().c_str(), (unsigned)off, a.c_str(), b.c_str());
            ++n;
        }
    }
    if (n == 0) API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE: no property with 'Slot' in its name on %ls", class_name_of(animbp).c_str());
}
API::UObject* sm_play(API::UObject* animbp, API::UObject* seq, float t) {
    auto* fn = animbp->get_class()->find_function(L"PlaySlotAnimationAsDynamicMontage");
    if (fn == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE: PlaySlotAnimationAsDynamicMontage not found on %ls", class_name_of(animbp).c_str()); return nullptr; }
    alignas(16) uint8_t p[0x100] = {0};
    bool ok = true;
    const std::string slot = trim_cfg(g_cfg.slide_slot);
    const std::wstring wslot(slot.begin(), slot.end());
    API::FName slotname = make_fname(wslot.c_str());
    const float bin = 0.0f, bout = 0.05f, rate = 0.001f, trig = -1.0f;
    const int32_t loops = 1;
    sm_put(fn, p, sizeof(p), L"Asset", &seq, sizeof(void*), &ok);
    sm_put(fn, p, sizeof(p), L"SlotNodeName", &slotname, sizeof(int32_t) * 2, &ok);
    sm_put(fn, p, sizeof(p), L"BlendInTime", &bin, sizeof(float), &ok);
    sm_put(fn, p, sizeof(p), L"BlendOutTime", &bout, sizeof(float), &ok);
    sm_put(fn, p, sizeof(p), L"InPlayRate", &rate, sizeof(float), &ok);
    sm_put(fn, p, sizeof(p), L"LoopCount", &loops, sizeof(int32_t), &ok);
    sm_put(fn, p, sizeof(p), L"BlendOutTriggerTime", &trig, sizeof(float), &ok);
    sm_put(fn, p, sizeof(p), L"InTimeToStartMontageAt", &t, sizeof(float), &ok);
    auto* ret = fn->find_property(L"ReturnValue");
    if (!ok || ret == nullptr) return nullptr;
    static bool s_said = false;
    if (!s_said) {
        s_said = true;
        for (API::FField* f = fn->get_child_properties(); f != nullptr; f = f->get_next()) {
            const auto* nm = f->get_fname();
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE param %ls @0x%X", nm ? nm->to_string().c_str() : L"?",
                                 (unsigned)static_cast<API::FProperty*>(f)->get_offset());
        }
    }
    fn->call(animbp, p);
    return *reinterpret_cast<API::UObject**>(p + ret->get_offset());
}
void sm_set_position(API::UObject* animbp, API::UObject* mont, float t) {
    auto* fn = animbp->get_class()->find_function(L"Montage_SetPosition");
    if (fn == nullptr) return;
    alignas(16) uint8_t p[0x40] = {0};
    bool ok = true;
    sm_put(fn, p, sizeof(p), L"Montage", &mont, sizeof(void*), &ok);
    sm_put(fn, p, sizeof(p), L"NewPosition", &t, sizeof(float), &ok);
    if (ok) fn->call(animbp, p);
}
void sm_stop(API::UObject* animbp, API::UObject* mont) {
    auto* fn = animbp->get_class()->find_function(L"Montage_Stop");
    if (fn == nullptr) return;
    alignas(16) uint8_t p[0x40] = {0};
    bool ok = true;
    const float bout = 0.05f;
    sm_put(fn, p, sizeof(p), L"InBlendOutTime", &bout, sizeof(float), &ok);
    sm_put(fn, p, sizeof(p), L"Montage", &mont, sizeof(void*), &ok);
    if (ok) fn->call(animbp, p);
}
void slide_montage_tick() {
    auto* animbp = reload_weapon_anim_instance();
    const bool usable = g_cfg.slide_montage && g_cfg.slide_vr && animbp != nullptr && slide_weapon_ok();
    if (!usable) {
        if (s_sm_active) {
            if (auto* inst = s_sm_inst.get()) if (auto* m = s_sm_mont.get()) sm_stop(inst, m);
            s_sm_active = false; s_sm_mont = TrackedObject{};
            if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE: stopped (unavailable)");
        }
        return;
    }
    sm_list_slots(animbp);
    const std::string spec = trim_cfg(g_cfg.slide_seq);
    if (spec.empty()) return;
    if (spec != s_sm_seq_last || s_sm_inst.get() != animbp) {
        s_sm_seq_last = spec; s_sm_inst.set(animbp);
        s_sm_active = false; s_sm_mont = TrackedObject{}; s_sm_seq = TrackedObject{}; s_sm_len = 0.0f;
        auto* seq = find_anim_sequence(spec);
        if (seq == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE: no AnimSequence matching '%s'", spec.c_str()); return; }
        s_sm_seq.set(seq);
        if (auto* pl = seq->get_property_data<float>(L"SequenceLength")) if (!IsBadReadPtr(pl, sizeof(float))) s_sm_len = *pl;
        API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE: sequence %ls, length %.3f s", seq->get_full_name().c_str(), s_sm_len);
    }
    auto* seq = s_sm_seq.get();
    if (seq == nullptr) return;
    const float pull = g_slide_pull.load(std::memory_order_relaxed);
    const float frac = (g_cfg.slide_travel > 0.0f) ? std::fmin(1.0f, std::fmax(0.0f, pull / g_cfg.slide_travel)) : 0.0f;
    const bool sweep = g_cfg.slide_seq_sweep > 0.0f && s_sm_len > 0.0f;
    const bool want = sweep || s_sl_held || pull > 0.0005f;
    float t = g_cfg.slide_seq_fwd + (g_cfg.slide_seq_back - g_cfg.slide_seq_fwd) * frac;
    if (sweep) {
        static long long s_t0 = 0;
        if (s_t0 == 0) s_t0 = now_ticks();
        const double sec = (double)(now_ticks() - s_t0) / (double)ms_to_ticks(1000);
        t = (float)std::fmod(sec / (double)g_cfg.slide_seq_sweep, 1.0) * s_sm_len;
    }
    if (want) {
        if (!s_sm_active || s_sm_mont.get() == nullptr) {
            auto* m = sm_play(animbp, seq, t);
            s_sm_active = (m != nullptr);
            if (m != nullptr) s_sm_mont.set(m);
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE: play in slot '%s' at t=%.3f -> montage=%s",
                                 trim_cfg(g_cfg.slide_slot).c_str(), t, m ? "ok" : "NULL");
        } else {
            sm_set_position(animbp, s_sm_mont.get(), t);
        }
        if (sweep && g_cfg.slide_log) {
            static long long s_said = 0;
            if (now_ticks() - s_said > ms_to_ticks(250)) { s_said = now_ticks(); API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE sweep t=%.3f of %.3f", t, s_sm_len); }
        }
    } else if (s_sm_active) {
        if (auto* m = s_sm_mont.get()) sm_stop(animbp, m);
        s_sm_active = false; s_sm_mont = TrackedObject{};
        if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE: stopped (slide home)");
    }
}

API::UObject* native_mag_mesh_impl() {
    static std::string s_key; static TrackedObject s_mesh; static bool s_none = false; static long long s_at = 0;
    const std::string key = weapon_key();
    if (key != s_key || (s_mesh.get() == nullptr && !s_none) || (s_none && now_ticks() - s_at > ms_to_ticks(2000))) {
        s_key = key; s_mesh = TrackedObject{}; s_none = false; s_at = now_ticks();
        auto* src = reload_weapon_default_comp();
        if (src != nullptr) {
            weapon_components([&](API::UObject* c) {
                auto** pp = c->get_property_data<API::UObject*>(L"AttachParent");
                if (pp == nullptr || IsBadReadPtr(pp, sizeof(void*)) || *pp != src) return true;
                auto** pm = c->get_property_data<API::UObject*>(L"StaticMesh");
                if (pm == nullptr || IsBadReadPtr(pm, sizeof(void*)) || *pm == nullptr || (*pm)->get_fname() == nullptr) return true;
                std::wstring lo = (*pm)->get_fname()->to_string(); for (auto& ch : lo) ch = (wchar_t)towlower(ch);
                if (lo.find(L"magazine") == std::wstring::npos && lo.find(L"megazine") == std::wstring::npos) return true;
                s_mesh.set(*pm);
                if (g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD native magazine mesh for %s: %ls", key.c_str(), (*pm)->get_full_name().c_str());
                return false;
            });
        }
        if (s_mesh.get() == nullptr) s_none = true;
    }
    return s_mesh.get();
}

// ---- ANIMSTATE (under slidelog): every change of the weapon AnimBP's FirstPersonState, so the
// fire state's enum value is read off a shot rather than guessed.
void anim_state_probe_tick() {
    if (!g_cfg.slide_log) return;
    auto* animbp = reload_weapon_anim_instance();
    if (animbp == nullptr) return;
    auto* p = animbp->get_property_data<uint8_t>(L"FirstPersonState");
    if (p == nullptr || IsBadReadPtr(p, 1)) return;
    static int s_last = -1, s_lines = 0;
    if ((int)*p != s_last && s_lines < 80) {
        ++s_lines;
        int tog = -1;
        if (auto* t = animbp->get_property_data<uint8_t>(L"AnimToggle")) if (!IsBadReadPtr(t, 1)) tog = (int)*t;
        API::get()->log_info("[Halo-CampE-UEVR] ANIMSTATE FirstPersonState %d -> %d (AnimToggle=%d)", s_last, (int)*p, tog);
        s_last = (int)*p;
    }
}

// ---- SLIDEFIRE. The first-person slide is posed only by the weapon's AnimBP, which has no Slot
// node, so the rack borrows the ABP's own FIRE state: FirstPersonState is held at the fire value
// and the mesh's GlobalAnimRateScale is steered every tick so the fire animation's time follows
// the hand (0 = slide home, slide_fire_back = fully back), then runs to the end on release. Time
// is estimated by integrating the rate we set against the tick's dt -- the ABP exposes no clock.
float s_gest_dt = 1.0f / 60.0f;
bool   s_sf_active = false, s_sf_releasing = false;
double s_sf_t = 0.0;
float  s_sf_len = 0.8f;
TrackedObject s_sf_comp, s_sf_inst;
uint8_t       s_sf_prev_state = 0;
void slide_fire_restore() {
    if (auto* c = s_sf_comp.get()) {
        if (auto* r = c->get_property_data<float>(L"GlobalAnimRateScale")) if (!IsBadWritePtr(r, sizeof(float))) *r = 1.0f;
    }
    // Hand the state back NOW: leaving "fire" set until the game's next 300 ms re-assertion
    // lets the fire animation restart at full rate in between (the second slam, 2026-09-03).
    if (s_sf_active) {
        if (auto* animbp = s_sf_inst.get()) {
            if (auto* st = animbp->get_property_data<uint8_t>(L"FirstPersonState")) if (!IsBadWritePtr(st, 1)) *st = s_sf_prev_state;
        }
    }
    s_sf_active = false; s_sf_releasing = false; s_sf_t = 0.0;
}
void slide_fire_tick() {
    auto* animbp = reload_weapon_anim_instance();
    auto* comp = reload_weapon_default_comp();
    const bool usable = g_cfg.slide_fire && !g_cfg.slide_copy && g_cfg.slide_vr && animbp != nullptr && comp != nullptr && slide_weapon_ok()
                        && s_reload == ReloadState::Idle;
    if (!usable) { if (s_sf_active) { slide_fire_restore(); if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDEFIRE: stopped (unavailable)"); } return; }
    auto* state = animbp->get_property_data<uint8_t>(L"FirstPersonState");
    auto* rate  = comp->get_property_data<float>(L"GlobalAnimRateScale");
    if (state == nullptr || rate == nullptr || IsBadWritePtr(state, 1) || IsBadWritePtr(rate, sizeof(float))) return;
    if (s_sf_comp.get() != comp) {
        s_sf_comp.set(comp);
        // The fire sequence's length, from the ABP's own FirstPersonFire slot.
        s_sf_len = 0.8f;
        for (API::UStruct* st = animbp->get_class(); st != nullptr; st = st->get_super_struct()) {
            if (IsBadReadPtr(st, sizeof(void*))) break;
            bool found = false;
            for (API::FField* f = st->get_child_properties(); f != nullptr; f = f->get_next()) {
                if (IsBadReadPtr(f, sizeof(void*))) break;
                auto* fc = f->get_class(); const auto* fn = f->get_fname();
                if (fc == nullptr || fn == nullptr || fc->get_name() != L"ObjectProperty") continue;
                const std::wstring nm = fn->to_string();
                static const std::wstring want = L"FirstPersonFire";
                if (nm.size() < want.size() || nm.compare(nm.size() - want.size(), want.size(), want) != 0) continue;
                auto** pp = reinterpret_cast<API::UObject**>(reinterpret_cast<uint8_t*>(animbp) + static_cast<API::FProperty*>(f)->get_offset());
                if (IsBadReadPtr(pp, sizeof(void*)) || *pp == nullptr) continue;
                if (auto* pl = (*pp)->get_property_data<float>(L"SequenceLength")) if (!IsBadReadPtr(pl, sizeof(float)) && *pl > 0.05f) s_sf_len = *pl;
                found = true; break;
            }
            if (found) break;
        }
        if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDEFIRE: fire sequence length %.3f s, state value %d", s_sf_len, g_cfg.slide_fire_state);
    }
    const float pull = g_slide_pull.load(std::memory_order_relaxed);
    const float frac = (g_cfg.slide_travel > 0.0f) ? std::fmin(1.0f, std::fmax(0.0f, pull / g_cfg.slide_travel)) : 0.0f;
    const bool want = s_sl_held || pull > 0.0005f;
    const float dt = (s_gest_dt > 0.001f && s_gest_dt < 0.2f) ? s_gest_dt : (1.0f / 60.0f);
    if (!s_sf_active) {
        if (!want) return;
        s_sf_active = true; s_sf_releasing = false; s_sf_t = 0.0;
        s_sf_inst.set(animbp);
        s_sf_prev_state = *state;
        *state = (uint8_t)g_cfg.slide_fire_state;
        const bool entry = g_cfg.slide_fire_entry > 0.0f;
        *rate = entry ? 1.0f : 0.0f;   // the entry window (if any): let the transition blend in at normal rate
        if (entry) s_sf_t += (double)dt;
        if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDEFIRE: enter state %d", g_cfg.slide_fire_state);
        return;
    }
    *state = (uint8_t)g_cfg.slide_fire_state;   // the game re-asserts its own each tick; hold ours
    if (!want && !s_sf_releasing) { s_sf_releasing = true; if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDEFIRE: released at t=%.3f, running to the end", s_sf_t); }
    float r = 0.0f;
    if (s_sf_releasing) {
        r = 1.0f;
    } else if (s_sf_t < (double)g_cfg.slide_fire_entry) {
        r = 1.0f;   // still inside the entry window
    } else {
        const double target = std::fmax((double)g_cfg.slide_fire_entry, (double)frac * (double)g_cfg.slide_fire_back);
        if (s_sf_t < target) r = (float)std::fmin(3.0, (target - s_sf_t) / (double)dt);
    }
    if (g_cfg.slide_log) {
        static long long s_said = 0;
        if (now_ticks() - s_said > ms_to_ticks(250)) { s_said = now_ticks(); API::get()->log_info("[Halo-CampE-UEVR] SLIDEFIRE t=%.3f rate=%.2f frac=%.2f state=%d", s_sf_t, r, frac, (int)*state); }
    }
    *rate = r;
    s_sf_t += (double)r * (double)dt;
    // Home again: hand the state back here, before the recoil and before the game's next idle
    // re-assertion can restart the fire at full rate.
    const double stop_at = (g_cfg.slide_fire_fwd > 0.0f) ? std::fmin((double)g_cfg.slide_fire_fwd, (double)s_sf_len) : (double)s_sf_len;
    if (s_sf_releasing && s_sf_t >= stop_at) {
        slide_fire_restore();
        if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDEFIRE: home at t=%.3f, state handed back", s_sf_t);
    }
}

// ---- SLIDECHAMBER: the reload a seated mag is waiting for fires once the rack has completed and
// the fire-state slide has snapped home (so the game's reload animation, which the state hold
// then suppresses, never overlaps the rack's own motion).
uint16_t* rounds_field() {
    // The object pointer is resolved on the sim thread from the published datum, so right after a
    // swap it can still be the previous weapon's: no read or write lands on that gun.
    const int32_t held = g_wpn_obj_index.load(std::memory_order_relaxed);
    if (held != -1 && g_wpn_obj_ptr_datum.load(std::memory_order_relaxed) != held) return nullptr;
    const uintptr_t obj = g_wpn_obj_ptr.load(std::memory_order_relaxed);
    const int off = g_cfg.rounds_off;
    if (obj == 0 || off <= 0 || off > 0x7FE) return nullptr;
    auto* r = reinterpret_cast<uint16_t*>(obj + (uintptr_t)off);
    if (IsBadWritePtr(r, sizeof(uint16_t))) return nullptr;
    return r;
}
// Is the gun in hand empty right now? The rounds counter when it reads (the phantom's one round
// counts as empty), else the AnimBP's two-state ammunition frame. The frame alone reads 0 on a
// freshly spawned weapon until its first reload (logged: 14 rounds, frame 0), which dropped a loaded
// rifle as empty: trigger dead with the mag out, a rack forced, the game's reload pressed at the drop.
bool weapon_empty_now() {
    if (s_true_empty) return true;
    if (auto* r = rounds_field()) return *r == 0;
    return slide_ammo_frame() == 0;
}
// ---- SLIDEPHANTOM: see Config.hpp. Runs every tick on the game thread against the resolved
// weapon object (ordinary heap memory once resolved).
// The session kind: standalone (single player) or networked (coop, host or client).
bool net_is_coop() {
    static long long s_at = 0; static bool s_coop = false; static bool s_said = false;
    const long long nowt = now_ticks();
    if (nowt - s_at < ms_to_ticks(2000)) return s_coop;
    s_at = nowt;
    auto* pawn = API::get()->get_local_pawn(0);
    auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.KismetSystemLibrary");
    auto* cdo = cls ? cls->get_class_default_object() : nullptr;
    if (pawn == nullptr || cdo == nullptr) return s_coop;
    alignas(16) uint8_t p[64] = {0};
    *reinterpret_cast<void**>(p) = pawn;
    cdo->call_function(L"IsStandalone", p);
    const bool standalone = p[8] != 0;
    const bool coop = !standalone;
    bool server = false;
    { alignas(16) uint8_t q[64] = {0}; *reinterpret_cast<void**>(q) = pawn; cdo->call_function(L"IsServer", q); server = q[8] != 0; }
    if (coop != s_coop || !s_said) { s_said = true; API::get()->log_info("[Halo-CampE-UEVR] NET: session is %s, this machine is the %s%s", coop ? "NETWORKED (coop)" : "standalone", server ? "HOST" : "CLIENT", (coop && g_cfg.coop_auto) ? (g_cfg.coop_hide ? " -- hidden reload: the host reloads underneath, frozen and locked until our gesture" : " -- phantom round and auto-reload undo stand down") : ""); }
    s_coop = coop;
    return s_coop;
}
// The hidden reload applies in coop (coophide) and, with hidesolo, in solo too.
bool reload_hidden_mode() {
    if (!g_cfg.coop_hide) return false;
    if (g_cfg.hide_solo) return true;
    return g_cfg.coop_auto && net_is_coop();
}
void ad_write_zero(bool render_path);   // the anim-variable readout probe (defined with the display block below)
// The phantom's per-weapon flags live outside the tick so the per-weapon reload state carries them.
// The displays read 0 from the dry shot until the mag is SEATED (not the drop, where the press
// clears the lock): s_hide_display outlives s_true_empty through the gesture.
bool    s_hide_display = false;
int     s_coop_lock_rounds = 1;
bool    s_ph_rebase = false;      // the weapon in hand changed identity: re-seed the counter caches
int32_t s_rs_cur_datum = -1;      // the held weapon's datum read this tick by the reload state tracker
void slide_phantom_tick() {
    static std::string s_key;
    static int s_prev = -1;
    static uint16_t s_snap[0x400]; static bool s_have_snap = false;   // last tick's first 0x800 bytes, for the reserve search
    static int s_reserve = -1;
    if (g_cfg.slide_phantom == 0 && !g_cfg.slide_undo_reload) {
        // The hidden reload's display hide is released below this return; switched off mid-lock it
        // stayed set and the ammo displays read 0 on a loaded gun.
        s_true_empty = false; s_hide_display = false; g_wristhud_hide_cradle.store(false, std::memory_order_relaxed);
        s_prev = -1; s_have_snap = false; return;
    }
    const bool coop = g_cfg.coop_auto && net_is_coop();
    const bool hidden = reload_hidden_mode();
    if (!hidden || (s_reload == ReloadState::Idle && !s_true_empty && !s_sl_pressed_early)) s_hide_display = false;
    g_wristhud_hide_cradle.store(hidden && (s_true_empty || s_hide_display), std::memory_order_relaxed);
    if (hidden && (s_true_empty || s_hide_display)) ad_write_zero(false);
    if (coop && !g_cfg.coop_hide && g_cfg.coop_stop_at <= 0) { s_true_empty = false; s_prev = -1; s_have_snap = false; return; }
    const std::string key = weapon_key();
    static long long s_key_at = 0, s_shot_at = 0;
    // Tracked only while the tracker runs (it lives in reload_update, behind reloadvr): with it
    // stopped nothing saves or loads the flags on a swap, and its datum goes stale.
    const bool tracked = g_cfg.reload_state_id != 0 && g_cfg.reload_vr;
    if (key != s_key || (tracked && s_ph_rebase)) {
        // Tracked: the flags already belong to the weapon now in hand (the tracker saved the old
        // weapon's and loaded this one's earlier in this tick); only the counter caches re-seed.
        // Dropping the flag here is what handed a gun back its full magazine for free.
        if (s_true_empty && g_cfg.slide_log) API::get()->log_info(tracked ? "[Halo-CampE-UEVR] PHANTOM: weapon changed; the flag stays with its weapon's record"
                                                                          : "[Halo-CampE-UEVR] PHANTOM: weapon changed while truly empty; flag dropped");
        s_key = key; s_key_at = now_ticks(); s_prev = -1; s_have_snap = false; s_reserve = (g_cfg.reserve_off >= 0) ? g_cfg.reserve_off : -1;
        if (!tracked) { s_true_empty = false; s_hide_display = false; }
        s_ph_rebase = false;
    }
    auto* r = rounds_field();
    if (r == nullptr) { s_prev = -1; s_have_snap = false; return; }
    // Tracked: right after a swap the object pointer can still be the previous weapon's (it is
    // resolved on the sim thread from a published datum); reading or writing that counter would
    // judge -- or clear -- this weapon's flags on the wrong gun.
    if (tracked && s_rs_cur_datum != -1 && g_wpn_obj_ptr_datum.load(std::memory_order_relaxed) != s_rs_cur_datum) { s_prev = -1; s_have_snap = false; return; }
    const int cur = (int)*r;
    const uintptr_t obj = g_wpn_obj_ptr.load(std::memory_order_relaxed);
    const bool obj_ok = obj != 0 && !IsBadReadPtr((const void*)obj, 0x800);
    // A REFILL THE PLAYER DID NOT ASK FOR: rounds jumped up while no reload of ours was in flight.
    // The reserve counter is the u16 that dropped by the same amount in the same tick.
    if (s_prev >= 0 && cur > s_prev + 1 && obj_ok && s_have_snap) {
        const int refill = cur - s_prev;
        const uint16_t* now16 = reinterpret_cast<const uint16_t*>(obj);
        int found = -1, nfound = 0;
        for (int i = 0; i < 0x400; ++i) {
            if (i * 2 == g_cfg.rounds_off) continue;
            if ((int)s_snap[i] - (int)now16[i] == refill && s_snap[i] < 4000) { if (found < 0) found = i * 2; ++nfound; }
        }
        if (found >= 0 && s_reserve < 0) { s_reserve = found; API::get()->log_info("[Halo-CampE-UEVR] PHANTOM: reserve counter found at +0x%03X (dropped by %d with the refill; %d candidate(s))", (unsigned)found, refill, nfound); }
        const bool ours = s_sl_reload_due || s_sl_pressed_early
                       || (g_reload_hold_until.load(std::memory_order_relaxed) > now_ticks() - ms_to_ticks(2000))
                       || (s_sl_press_at != 0 && now_ticks() - s_sl_press_at < ms_to_ticks(g_cfg.reload_mute_ms + 1500));
        if (!coop && !hidden && g_cfg.slide_undo_reload && s_true_empty && !ours) {
            *r = 0;
            if (s_reserve >= 0 && s_reserve < 0x7FE) {
                auto* rs = reinterpret_cast<uint16_t*>(obj + (uintptr_t)s_reserve);
                if (!IsBadWritePtr(rs, 2)) { const int v = (int)*rs + refill; *rs = (uint16_t)(v > 65535 ? 65535 : v); }
            }
            reload_state_hold_begin();   // the game's reload animation stays off screen
            API::get()->log_info("[Halo-CampE-UEVR] PHANTOM: undid a game auto-reload of %d rounds (reserve %s); gun stays empty", refill, s_reserve >= 0 ? "restored" : "UNKNOWN, not restored");
            s_prev = 0;
            if (obj_ok) { memcpy(s_snap, reinterpret_cast<const void*>(obj), sizeof(s_snap)); s_have_snap = true; }
            return;
        }
    }
    // NEVER an auto-reload (from the headset, 2026-09-06): the lock has no timeout. With no reserve the
    // last round simply stays locked, which costs one round and never a zero. The searched reserve
    // offset is not trusted here (in coop it picked a field that reads 0 with a full mag); only a
    // cfg-given reserve offset may veto the lock.
    bool reserve_empty = false;
    if (coop && obj_ok && g_cfg.reserve_off >= 0 && s_reserve >= 0 && s_reserve < 0x7FE) { const auto* rs = reinterpret_cast<const uint16_t*>(obj + (uintptr_t)s_reserve); if (!IsBadReadPtr(rs, 2) && *rs == 0) reserve_empty = true; }
    // FULL AUTO (2026-09-06 21:43, one shot got out 84 ms after the lock): between the tick that
    // sees the count and the sim seeing the trigger let go there is room for one more shot at the
    // AR's rate, so while shots are landing at that pace the lock goes on one round early. The
    // worst case then ends on one, never zero. The key must be 500 ms old: the counter reads junk
    // for a few ticks around a weapon swap.
    const long long nowt_c = now_ticks();
    const bool firing_fast = nowt_c - s_shot_at < ms_to_ticks(250);
    const int stop_at = g_cfg.coop_stop_at + (firing_fast ? 1 : 0);
    // A REAL last shot only: exactly one round going to zero on a weapon held two seconds. A
    // weapon's counter reads 0 while it initialises and on a swap to a plasma weapon (60 -> 0 in
    // the log), and the first build fired the dry trigger in the mission's first second and froze
    // the pose under a live gun (the climbing pitch, 2026-09-07).
    if (g_cfg.reload_vr && hidden && !s_true_empty && s_prev == 1 && cur == 0 && nowt_c - s_key_at > ms_to_ticks(2000)
        && s_reload == ReloadState::Idle && !s_sl_reload_due && !s_sl_pressed_early && !weapon_in_list(g_cfg.reload_skip_weapons)) {
        // The last round went out and the game is reloading: freeze the pose, mute it, show empty,
        // dead trigger. Nothing clears this but our own reload (reload_press_now) or a swap.
        s_true_empty = true; s_hide_display = true; s_coop_lock_rounds = 0x7FFF;
        ak_mute_begin();
        reload_state_hold_begin();
        reload_anim_rate_begin();
        if (g_cfg.coop_mask_ms > 0) blam_palette_hold_pose(g_cfg.coop_mask_ms);   // the hands too, until our gesture
        if (g_cfg.slide_log || g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] DRY (%s): last round fired, the game reloads underneath; pose frozen, muted, locked until our reload", coop ? "coop" : "solo");
        s_prev = cur;
        if (obj_ok) { memcpy(s_snap, reinterpret_cast<const void*>(obj), sizeof(s_snap)); s_have_snap = true; }
        return;
    }
    if (g_cfg.reload_vr && coop && !g_cfg.coop_hide && !s_true_empty && cur >= 1 && cur <= stop_at && !reserve_empty && nowt_c - s_key_at > ms_to_ticks(500)
        && s_reload == ReloadState::Idle && !s_sl_reload_due && !s_sl_pressed_early
        && !weapon_in_list(g_cfg.reload_skip_weapons)) {   // every weapon with a magazine, rack or not (the SMG has no rack part)
        // The dry stop: no write, the last round stays in the counter and the trigger is dead.
        s_true_empty = true; s_coop_lock_rounds = cur;
        if (g_cfg.slide_log || g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] COOP dry stop: %d round(s) left, trigger locked and slide back until our reload", cur);
        s_prev = cur;
        if (obj_ok) { memcpy(s_snap, reinterpret_cast<const void*>(obj), sizeof(s_snap)); s_have_snap = true; }
        return;
    }
    if (g_cfg.reload_vr && !coop && !hidden && !s_true_empty && s_prev == 1 && cur == 0 && slide_weapon_ok() && slide_chamber_ok() && slide_rack_available() && g_cfg.slide_phantom > 0) {
        *r = 1;
        s_true_empty = true;
        if (g_cfg.slide_log || g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] PHANTOM: magazine empty; one phantom round held so the game does not auto-reload (trigger dead, slide back)");
        s_prev = 1;
        if (obj_ok) { memcpy(s_snap, reinterpret_cast<const void*>(obj), sizeof(s_snap)); s_have_snap = true; }
        return;
    }
    if (s_true_empty) {
        if (cur > ((coop || hidden) ? s_coop_lock_rounds : 1) || (coop && reserve_empty)) {
            s_true_empty = false;
            if (g_cfg.slide_log || g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] PHANTOM: rounds now %d, the gun reloaded; phantom cleared", cur);
        } else {
            if (!coop && !hidden && g_cfg.slide_phantom == 2 && cur == 0) *r = 1;   // re-asserted: the sim never sees 0
            if (auto* animbp = reload_weapon_anim_instance()) {
                if (auto* p = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame")) if (!IsBadWritePtr(p, sizeof(int32_t))) *p = 0;
            }
        }
    }
    // A PUMP BETWEEN SHOTS (@everyshot): a shot fired outside our reload locks the gun until the
    // rack completes; the rack is live while the lock waits, hold or no hold.
    if (g_sl_zone_every_shot.load(std::memory_order_relaxed) && cur < s_prev && s_prev > 0 && !s_sl_lock_pending && s_reload == ReloadState::Idle) {
        s_sl_lock_pending = true; s_sl_lock_frame = 0; s_sl_rack_done = false;
        if (g_cfg.slide_log || g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDE shot fired on a pump weapon: locked until the rack");
    }
    {
        const int fin = (int)*r;
        if (s_prev >= 0 && fin > s_prev + 1) g_last_refill_at = now_ticks();
        if (s_prev >= 0 && fin < s_prev && s_prev - fin <= 3) s_shot_at = now_ticks();
        if (g_cfg.reload_log && s_prev >= 0 && fin != s_prev) {
            int reserve = -1;
            if (obj_ok && s_reserve >= 0 && s_reserve < 0x7FE) { const auto* rs = reinterpret_cast<const uint16_t*>(obj + (uintptr_t)s_reserve); if (!IsBadReadPtr(rs, 2)) reserve = (int)*rs; }
            API::get()->log_info("[Halo-CampE-UEVR] AMMO rounds %d -> %d (reserve %d)%s", s_prev, fin, reserve, fin > s_prev ? "  <-- REFILL" : "");
        }
        s_prev = fin;
    }
    if (obj_ok) { memcpy(s_snap, reinterpret_cast<const void*>(obj), sizeof(s_snap)); s_have_snap = true; }
}

// The game's reload press with everything that rides it: the honest 0 first so the refill
// accounts from empty, the hold, the animation clamp, the sound mute.
void reload_press_now(const char* why) {
    const long long nowt = now_ticks();
    if (s_true_empty) {
        s_true_empty = false;   // stop the every-tick re-assert BEFORE the 0 goes back, or the refill counts from 1
        if (!(g_cfg.coop_auto && net_is_coop()) && !reload_hidden_mode()) { if (auto* r = rounds_field()) { if (*r == 1) *r = 0; } }   // coop / hidden: the counter is never written
        if (g_cfg.slide_log || g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] PHANTOM: rounds set back to 0 for the reload (%s)", why);
    }
    g_reload_hold_until.store(nowt + ms_to_ticks(net_is_coop() ? g_cfg.reload_press_ms_coop : g_cfg.reload_press_ms), std::memory_order_relaxed);
    s_sl_press_at = nowt;
    if (reload_hidden_mode()) blam_palette_hold_pose(g_cfg.reload_mask_ms);   // the hidden reload's hand hold ends here (0 releases)
    else if (g_cfg.reload_mask_ms > 0) blam_palette_hold_pose(g_cfg.reload_mask_ms);
    reload_anim_rate_begin();
    reload_state_hold_begin();
    ak_mute_begin();
    if (g_cfg.reload_log || g_cfg.slide_log) { const auto* r = rounds_field(); API::get()->log_info("[Halo-CampE-UEVR] RELOAD pressed (%s): %d ms, rounds %d", why, net_is_coop() ? g_cfg.reload_press_ms_coop : g_cfg.reload_press_ms, r ? (int)*r : -1); }
}
bool reload_gestures_busy() { return s_reload != ReloadState::Idle || s_sl_lock_pending || s_sl_reload_due; }
void slide_chamber_tick() {
    if (s_sl_press_due_at != 0 && now_ticks() >= s_sl_press_due_at) {
        s_sl_press_due_at = 0;
        if (s_reload != ReloadState::Idle) { s_sl_pressed_early = true; reload_press_now("chambered round fired"); }
    }
    if (!s_sl_reload_due) return;
    if (s_reload != ReloadState::Idle || !slide_weapon_ok() || !slide_chamber_ok()) { s_sl_reload_due = false; return; }
    if (s_sl_lock_pending || s_sf_active) return;   // not racked yet, or the slide is still moving
    if (s_sl_pressed_early) { s_sl_pressed_early = false; s_sl_reload_due = false; if (g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD rack complete, the sim reloaded at the drop"); return; }
    reload_press_now("rack");
    s_sl_reload_due = false;
}

// The engine's destroy is K2_DestroyComponent(Object) in reflection; a call to "DestroyComponent"
// finds no function and silently leaves the component alive (2026-09-04: every re-spawn stacked
// another gun).
void ue_destroy_component(API::UObject* c) {
    if (c == nullptr || IsBadReadPtr(c, sizeof(void*))) return;
    alignas(16) uint8_t q[64] = {0}; *reinterpret_cast<void**>(q) = c;
    c->call_function(L"K2_DestroyComponent", q);
}
// ---- SLIDECOPY. See Config.hpp. Every engine call goes through the UFunction's own parameter
// offsets (sm_put), never an assumed layout; FTransform is the UE5 double layout (rotation quat
// at 0, translation at 0x20, scale at 0x40, 0x60 bytes, 16-aligned).
struct alignas(16) XformD { double qx, qy, qz, qw; double tx, ty, tz; double pad0; double sx, sy, sz; double pad1; };
TrackedObject s_sc_copy, s_sc_src, s_sc_actor, s_sc_follower, s_sc_seq;
float         s_sc_len = 0.0f;
std::string   s_sc_key;
API::FName    s_sc_bone;
bool          s_sc_bone_ok = false;
bool          s_sc_hidden = false;
bool          s_sc_failed = false;
long long     s_sc_retry_at = 0;
uint32_t      s_sc_ticks = 0;
API::UFunction* sc_fn(API::UObject* o, const wchar_t* name) {
    if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) return nullptr;
    auto* c = o->get_class();
    return c ? c->find_function(name) : nullptr;
}
void sc_set_visibility(API::UObject* c, bool vis) {
    if (c == nullptr) return;
    alignas(16) uint8_t p[64] = {0}; p[0] = vis ? 1 : 0; p[1] = 0;
    c->call_function(L"SetVisibility", p);
}
void sc_set_scale(API::UObject* c, double sc) {
    if (c == nullptr) return;
    alignas(16) uint8_t p[64] = {0}; auto* d = reinterpret_cast<double*>(p); d[0] = sc; d[1] = sc; d[2] = sc;
    c->call_function(L"SetRelativeScale3D", p);
}
void sc_hide_real(API::UObject* src, bool hide) {
    if (src == nullptr) return;
    if (g_cfg.slide_copy_hide == 1) sc_set_visibility(src, !hide);
    else if (g_cfg.slide_copy_hide == 2) sc_set_scale(src, hide ? 0.001 : 1.0);
}
// The real component's render flags, read by name and mirrored onto the copy through the
// engine's own setters (a bare property write does not recreate the render proxy). Logged so
// the flag that makes a first-person primitive draw is a fact, not a guess.
void sc_mirror_flags(API::UObject* src, API::UObject* copy) {
    static const wchar_t* kBools[] = { L"bOnlyOwnerSee", L"bOwnerNoSee", L"bRenderInMainPass", L"bRenderInDepthPass",
                                       L"bVisibleInReflectionCaptures", L"bVisibleInRealTimeSkyCaptures", L"bVisibleInRayTracing",
                                       L"bReceivesDecals", L"bRenderCustomDepth", L"CastShadow", L"bVisible", L"bHiddenInGame",
                                       L"bTreatAsBackgroundForOcclusion", L"bUseAsOccluder", L"bCastInsetShadow", L"bSelfShadowOnly" };
    std::wstring line;
    for (const wchar_t* nm : kBools) {
        auto* pr = src->get_class()->find_property(nm);
        if (pr == nullptr) continue;
        auto* pcls = pr->get_class();
        if (pcls == nullptr || pcls->get_name() != L"BoolProperty") continue;
        const bool v = static_cast<API::FBoolProperty*>(pr)->get_value_from_object(src);
        line += std::wstring(nm) + L"=" + (v ? L"1 " : L"0 ");
    }
    // The 5.5 first-person primitive type, and any other byte/enum on the primitive with a name
    // that says first person.
    int fp_type = -1;
    for (API::UStruct* st = src->get_class(); st != nullptr; st = st->get_super_struct()) {
        if (IsBadReadPtr(st, sizeof(void*))) break;
        for (API::FField* f = st->get_child_properties(); f != nullptr; f = f->get_next()) {
            if (IsBadReadPtr(f, sizeof(void*))) break;
            auto* fc = f->get_class(); const auto* fn = f->get_fname();
            if (fc == nullptr || fn == nullptr) continue;
            const std::wstring cls = fc->get_name(), nm = fn->to_string();
            if (nm.find(L"FirstPerson") == std::wstring::npos) continue;
            const int32_t off = static_cast<API::FProperty*>(f)->get_offset();
            const uint8_t* q = reinterpret_cast<const uint8_t*>(src) + off;
            if (IsBadReadPtr(q, 4)) continue;
            int v = -1;
            if (cls == L"ByteProperty" || cls == L"EnumProperty" || cls == L"BoolProperty") v = (int)*q;
            else if (cls == L"IntProperty") v = *reinterpret_cast<const int32_t*>(q);
            else if (cls == L"FloatProperty") v = (int)(*reinterpret_cast<const float*>(q) * 1000.0f);
            line += nm + L"(" + cls + L")=" + std::to_wstring(v) + L" ";
            if (nm == L"FirstPersonPrimitiveType") fp_type = v;
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: real mesh flags: %ls", line.c_str());
    // Nanite: the asset's setting and any Nanite-named flag on the copy, by name.
    {
        std::wstring nl;
        for (const wchar_t* nm : { L"SkinnedAsset", L"SkeletalMesh" }) {
            auto** pm = src->get_property_data<API::UObject*>(nm);
            if (pm == nullptr || IsBadReadPtr(pm, sizeof(void*)) || *pm == nullptr) continue;
            auto* asset = *pm;
            if (auto* pr = asset->get_class()->find_property(L"NaniteSettings")) {
                const uint8_t* q = reinterpret_cast<const uint8_t*>(asset) + pr->get_offset();
                if (!IsBadReadPtr(q, 1)) nl += L"asset NaniteSettings.bEnabled=" + std::to_wstring((int)(q[0] & 1)) + L" ";
            }
            break;
        }
        for (API::UStruct* st = copy->get_class(); st != nullptr; st = st->get_super_struct()) {
            if (IsBadReadPtr(st, sizeof(void*))) break;
            for (API::FField* f = st->get_child_properties(); f != nullptr; f = f->get_next()) {
                if (IsBadReadPtr(f, sizeof(void*))) break;
                auto* fc = f->get_class(); const auto* fn = f->get_fname();
                if (fc == nullptr || fn == nullptr) continue;
                const std::wstring nm = fn->to_string();
                if (nm.find(L"Nanite") == std::wstring::npos) continue;
                int v = -1;
                if (fc->get_name() == L"BoolProperty") v = static_cast<API::FBoolProperty*>(f)->get_value_from_object(copy) ? 1 : 0;
                nl += L"copy." + nm + L"(" + fc->get_name() + L")=" + std::to_wstring(v) + L" ";
            }
        }
        API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: nanite: %ls", nl.empty() ? L"(nothing named Nanite)" : nl.c_str());
    }
    fp_type = g_cfg.slide_copy_fp;   // the copy's type is a cfg choice, not a mirror (2026-09-04)
    if (fp_type >= 0) {
        if (auto* fn = sc_fn(copy, L"SetFirstPersonPrimitiveType")) {
            alignas(16) uint8_t p[64] = {0}; bool ok = true; const uint8_t t = (uint8_t)fp_type;
            for (API::FField* f = fn->get_child_properties(); f != nullptr; f = f->get_next()) {
                const auto* fnm = f->get_fname();
                if (fnm == nullptr) continue;
                const std::wstring nm = fnm->to_string();
                if (nm != L"ReturnValue") { sm_put(fn, p, sizeof(p), nm.c_str(), &t, 1, &ok); break; }
            }
            if (ok) { fn->call(copy, p); API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: copy FirstPersonPrimitiveType set to %d", fp_type); }
        } else {
            API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: no SetFirstPersonPrimitiveType on the copy");
        }
    }
    auto mirror_bool = [&](const wchar_t* prop, const wchar_t* setter) {
        auto* pr = src->get_class()->find_property(prop);
        if (pr == nullptr || pr->get_class() == nullptr || pr->get_class()->get_name() != L"BoolProperty") return;
        const bool v = static_cast<API::FBoolProperty*>(pr)->get_value_from_object(src);
        alignas(16) uint8_t p[64] = {0}; p[0] = v ? 1 : 0;
        copy->call_function(setter, p);
    };
    mirror_bool(L"bOnlyOwnerSee", L"SetOnlyOwnerSee");
    mirror_bool(L"bOwnerNoSee", L"SetOwnerNoSee");
    // NOT mirrored: the real mesh has bRenderInMainPass=0 (measured 2026-09-04) and is drawn by
    // something else; a copy with the same flag draws nowhere. The copy stays in the main pass.
    { alignas(16) uint8_t p[64] = {0}; p[0] = 1; copy->call_function(L"SetRenderInMainPass", p); }
    mirror_bool(L"bRenderInDepthPass", L"SetRenderInDepthPass");
    mirror_bool(L"CastShadow", L"SetCastShadow");
    // A visibility toggle recreates the render proxy with the mirrored flags.
    sc_set_visibility(copy, false);
    sc_set_visibility(copy, true);
}
void sc_teardown(const char* why) {
    if (auto* f = s_sc_follower.get()) { alignas(16) uint8_t p[64] = {0}; ue_destroy_component(f); }
    s_sc_follower = TrackedObject{};
    if (auto* c = s_sc_copy.get()) { alignas(16) uint8_t p[64] = {0}; ue_destroy_component(c); }
    if (s_sc_hidden) { if (auto* src = s_sc_src.get()) { sc_set_visibility(src, true); sc_set_scale(src, 1.0); } }
    if (s_sc_copy.get() != nullptr || s_sc_hidden) {
        if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: torn down (%s)", why);
    }
    s_sc_copy = TrackedObject{}; s_sc_src = TrackedObject{}; s_sc_actor = TrackedObject{};
    s_sc_hidden = false; s_sc_bone_ok = false; s_sc_key.clear(); s_sc_ticks = 0;
}
// The real mesh's attach parent, socket and relative transform, read from its own properties.
bool sc_read_attach(API::UObject* src, API::UObject** parent, API::FName* socket, double loc[3], double rot[3], double scl[3]) {
    auto** pp = src->get_property_data<API::UObject*>(L"AttachParent");
    if (pp == nullptr || IsBadReadPtr(pp, sizeof(void*))) return false;
    *parent = *pp;
    auto* ps = src->get_property_data<API::FName>(L"AttachSocketName");
    if (ps == nullptr || IsBadReadPtr(ps, sizeof(int32_t) * 2)) return false;
    memcpy(socket, ps, sizeof(int32_t) * 2);
    auto* pl = src->get_property_data<double>(L"RelativeLocation");
    auto* pr = src->get_property_data<double>(L"RelativeRotation");
    auto* pc = src->get_property_data<double>(L"RelativeScale3D");
    if (pl == nullptr || pr == nullptr || pc == nullptr) return false;
    if (IsBadReadPtr(pl, 24) || IsBadReadPtr(pr, 24) || IsBadReadPtr(pc, 24)) return false;
    for (int i = 0; i < 3; ++i) { loc[i] = pl[i]; rot[i] = pr[i]; scl[i] = pc[i]; }
    return true;
}
void sc_dump_asset_users(API::UObject* asset);   // defined below
bool sc_spawn(API::UObject* actor, API::UObject* src) {
    const bool seq_mode = (g_cfg.slide_copy_mode == 1);
    auto* cls = API::get()->find_uobject<API::UClass>((seq_mode || g_cfg.slide_copy_class == 1) ? L"Class /Script/Engine.SkeletalMeshComponent"
                                                                                                : L"Class /Script/Engine.PoseableMeshComponent");
    if (cls == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: copy component class not found"); return false; }
    API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: spawning a %ls", (seq_mode || g_cfg.slide_copy_class == 1) ? L"SkeletalMeshComponent" : L"PoseableMeshComponent");
    // The mesh asset, whichever name this engine version gives it.
    API::UObject* mesh = nullptr;
    for (const wchar_t* nm : { L"SkinnedAsset", L"SkeletalMesh", L"SkeletalMeshAsset" }) {
        auto** pm = src->get_property_data<API::UObject*>(nm);
        if (pm != nullptr && !IsBadReadPtr(pm, sizeof(void*)) && *pm != nullptr) { mesh = *pm; break; }
    }
    if (mesh == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: the real mesh has no skeletal mesh asset property I can read"); return false; }
    API::UObject* parent = nullptr; API::FName socket{}; double loc[3] = {0}, rot[3] = {0}, scl[3] = {1, 1, 1};
    if (!sc_read_attach(src, &parent, &socket, loc, rot, scl)) { API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: cannot read the real mesh's attachment"); return false; }
    API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: real mesh %ls, parent %ls '%ls' socket '%ls', rel loc (%.2f %.2f %.2f) rot (%.2f %.2f %.2f) scale (%.2f %.2f %.2f)",
                         mesh->get_full_name().c_str(),
                         parent ? class_name_of(parent).c_str() : L"null",
                         (parent && parent->get_fname()) ? parent->get_fname()->to_string().c_str() : L"?",
                         socket.to_string().c_str(), loc[0], loc[1], loc[2], rot[0], rot[1], rot[2], scl[0], scl[1], scl[2]);
    auto* copy = API::get()->add_component_by_class(actor, cls, false);
    if (copy == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: add_component_by_class failed"); return false; }
    // Off the Nanite path BEFORE the mesh is set, so the render object is created classic.
    // Only the poseable: a SkeletalMeshComponent forced off Nanite has no classic data to draw on
    // a Nanite-only asset (2026-09-04: that is why the single-node copy vanished).
    if (g_cfg.slide_copy_no_nanite && !seq_mode && g_cfg.slide_copy_class == 0) {
        auto* pr = copy->get_class()->find_property(L"bForceDisableNanite");
        if (pr != nullptr && pr->get_class() != nullptr && pr->get_class()->get_name() == L"BoolProperty") {
            static_cast<API::FBoolProperty*>(pr)->set_value_in_object(copy, true);
            API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: bForceDisableNanite=1 on the copy");
        } else {
            API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: no bForceDisableNanite on the copy's class");
        }
    }
    // The mesh: 5.1+ names the setter SetSkinnedAssetAndUpdate; older engines SetSkeletalMesh.
    bool mesh_set = false;
    for (const wchar_t* fname : { L"SetSkinnedAssetAndUpdate", L"SetSkeletalMesh" }) {
        auto* fn = sc_fn(copy, fname);
        if (fn == nullptr) continue;
        alignas(16) uint8_t p[64] = {0}; bool ok = true; const bool reinit = true;
        sm_put(fn, p, sizeof(p), L"NewMesh", &mesh, sizeof(void*), &ok);
        sm_put(fn, p, sizeof(p), L"bReinitPose", &reinit, 1, &ok);
        if (!ok) continue;
        fn->call(copy, p); mesh_set = true;
        API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: mesh set via %ls", fname);
        break;
    }
    if (!mesh_set) { API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: no mesh setter found on the copy"); alignas(16) uint8_t p[64] = {0}; ue_destroy_component(copy); return false; }
    { alignas(16) uint8_t p[64] = {0}; p[0] = 0; copy->call_function(L"SetCollisionEnabled", p); }
    if (parent != nullptr) {
        auto* fn = sc_fn(copy, L"K2_AttachToComponent");
        alignas(16) uint8_t p[128] = {0}; bool ok = true; const uint8_t rule = 0; const bool weld = false;
        if (fn != nullptr) {
            sm_put(fn, p, sizeof(p), L"Parent", &parent, sizeof(void*), &ok);
            sm_put(fn, p, sizeof(p), L"SocketName", &socket, sizeof(int32_t) * 2, &ok);
            sm_put(fn, p, sizeof(p), L"LocationRule", &rule, 1, &ok);
            sm_put(fn, p, sizeof(p), L"RotationRule", &rule, 1, &ok);
            sm_put(fn, p, sizeof(p), L"ScaleRule", &rule, 1, &ok);
            sm_put(fn, p, sizeof(p), L"bWeldSimulatedBodies", &weld, 1, &ok);
            if (ok) { fn->call(copy, p); auto* r = fn->find_property(L"ReturnValue"); API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: attached -> %d", r ? (int)p[r->get_offset()] : -1); }
        }
    }
    {
        auto* fn = sc_fn(copy, L"K2_SetRelativeLocationAndRotation");
        if (fn != nullptr) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const bool no = false, yes = true;
            sm_put(fn, p, sizeof(p), L"NewLocation", loc, 24, &ok);
            sm_put(fn, p, sizeof(p), L"NewRotation", rot, 24, &ok);
            sm_put(fn, p, sizeof(p), L"bSweep", &no, 1, &ok);
            sm_put(fn, p, sizeof(p), L"bTeleport", &yes, 1, &ok);
            if (ok) fn->call(copy, p);
        }
        alignas(16) uint8_t q[64] = {0}; memcpy(q, scl, 24); copy->call_function(L"SetRelativeScale3D", q);
    }
    // The bones, and the one to move.
    s_sc_bone_ok = false;
    {
        int32_t nb = 0;
        if (auto* fn = sc_fn(copy, L"GetNumBones")) { alignas(16) uint8_t p[64] = {0}; fn->call(copy, p); auto* r = fn->find_property(L"ReturnValue"); if (r) nb = *reinterpret_cast<int32_t*>(p + r->get_offset()); }
        std::string want = trim_cfg(g_cfg.slide_copy_bone);
        for (auto& ch : want) ch = (char)tolower((unsigned char)ch);
        std::wstring names;
        auto* gb = sc_fn(copy, L"GetBoneName");
        for (int32_t i = 0; gb != nullptr && i < nb && i < 64; ++i) {
            alignas(16) uint8_t p[64] = {0}; bool ok = true;
            sm_put(gb, p, sizeof(p), L"BoneIndex", &i, sizeof(int32_t), &ok);
            auto* r = gb->find_property(L"ReturnValue");
            if (!ok || r == nullptr) break;
            gb->call(copy, p);
            API::FName bn; memcpy(&bn, p + r->get_offset(), sizeof(int32_t) * 2);
            std::wstring wn = bn.to_string();
            names += (i ? L", " : L"") + wn;
            std::string an(wn.begin(), wn.end());
            for (auto& ch : an) ch = (char)tolower((unsigned char)ch);
            if (!s_sc_bone_ok && !want.empty() && an.find(want) != std::string::npos) { s_sc_bone = bn; s_sc_bone_ok = true; }
        }
        API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: %d bones: %ls", nb, names.c_str());
        if (auto* gp = sc_fn(copy, L"GetParentBone")) {
            std::wstring tree;
            for (int32_t i = 0; gb != nullptr && i < nb && i < 64; ++i) {
                alignas(16) uint8_t p[64] = {0}; bool ok = true;
                sm_put(gb, p, sizeof(p), L"BoneIndex", &i, sizeof(int32_t), &ok);
                auto* r = gb->find_property(L"ReturnValue");
                if (!ok || r == nullptr) break;
                gb->call(copy, p);
                API::FName bn; memcpy(&bn, p + r->get_offset(), sizeof(int32_t) * 2);
                alignas(16) uint8_t q[64] = {0}; bool ok2 = true;
                sm_put(gp, q, sizeof(q), L"BoneName", &bn, sizeof(int32_t) * 2, &ok2);
                auto* r2 = gp->find_property(L"ReturnValue");
                if (!ok2 || r2 == nullptr) break;
                gp->call(copy, q);
                API::FName pn; memcpy(&pn, q + r2->get_offset(), sizeof(int32_t) * 2);
                tree += bn.to_string() + L"<-" + pn.to_string() + L"  ";
            }
            API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: hierarchy (bone<-parent): %ls", tree.c_str());
        }
        API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: slide bone '%s' -> %s", want.c_str(), s_sc_bone_ok ? "found" : "NOT FOUND (copy shows, nothing moves)");
    }
    sc_mirror_flags(src, copy);
    sc_dump_asset_users(mesh);
    s_sc_copy.set(copy); s_sc_src.set(src); s_sc_actor.set(actor);
    if (seq_mode) {
        // Single-node animation on the copy: the sequence's time is ours to set.
        s_sc_seq = TrackedObject{}; s_sc_len = 0.0f;
        auto* seq = find_anim_sequence(trim_cfg(g_cfg.slide_copy_seq));
        if (seq == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: no AnimSequence matching '%s'", trim_cfg(g_cfg.slide_copy_seq).c_str()); return true; }
        { alignas(16) uint8_t p[64] = {0}; p[0] = 1;   // EAnimationMode::AnimationSingleNode
          copy->call_function(L"SetAnimationMode", p); }
        { alignas(16) uint8_t p[64] = {0}; *reinterpret_cast<API::UObject**>(p) = seq;
          copy->call_function(L"SetAnimation", p); }
        if (g_cfg.slide_copy_play == 0) { alignas(16) uint8_t p[64] = {0}; p[0] = 0; copy->call_function(L"SetPlayRate", p); }
        else if (g_cfg.slide_copy_play == 1) { alignas(16) uint8_t p[64] = {0}; p[0] = 1; copy->call_function(L"Play", p); }
        else { alignas(16) uint8_t p[64] = {0}; *reinterpret_cast<float*>(p) = 0.001f; copy->call_function(L"SetPlayRate", p); }
        // Tick the pose whether or not the renderer has drawn the component yet: a Nanite-skinned
        // mesh with no evaluated pose may never get bone data, and so never draw (2026-09-04).
        { alignas(16) uint8_t p[64] = {0}; p[0] = 0;   // AlwaysTickPoseAndRefreshBones
          copy->call_function(L"SetVisibilityBasedAnimTickOption", p); }
        if (auto* pl = seq->get_property_data<float>(L"SequenceLength")) if (!IsBadReadPtr(pl, sizeof(float))) s_sc_len = *pl;
        s_sc_seq.set(seq);
        { alignas(16) uint8_t p[64] = {0}; *reinterpret_cast<float*>(p) = s_sc_len; p[4] = 0; copy->call_function(L"SetPosition", p); }
        API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: single-node %ls, length %.3f s, parked at the end (gun at rest)", seq->get_full_name().c_str(), s_sc_len);
        return true;
    }
    if (g_cfg.slide_copy_follower && g_cfg.slide_copy_class == 0) {
        // The leader must refresh its bone transforms every tick even though it never renders.
        { alignas(16) uint8_t p[64] = {0}; p[0] = 0;   // AlwaysTickPoseAndRefreshBones
          copy->call_function(L"SetVisibilityBasedAnimTickOption", p); }
        sc_set_visibility(copy, false);
        auto* fcls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.SkeletalMeshComponent");
        auto* fol = fcls ? API::get()->add_component_by_class(actor, fcls, false) : nullptr;
        if (fol == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: follower spawn failed"); return true; }
        for (const wchar_t* fname : { L"SetSkinnedAssetAndUpdate", L"SetSkeletalMesh" }) {
            auto* fn = sc_fn(fol, fname);
            if (fn == nullptr) continue;
            alignas(16) uint8_t p[64] = {0}; bool ok = true; const bool reinit = true;
            sm_put(fn, p, sizeof(p), L"NewMesh", &mesh, sizeof(void*), &ok);
            sm_put(fn, p, sizeof(p), L"bReinitPose", &reinit, 1, &ok);
            if (ok) { fn->call(fol, p); break; }
        }
        { alignas(16) uint8_t p[64] = {0}; p[0] = 0; fol->call_function(L"SetCollisionEnabled", p); }
        if (parent != nullptr) {
            if (auto* fn = sc_fn(fol, L"K2_AttachToComponent")) {
                alignas(16) uint8_t p[128] = {0}; bool ok = true; const uint8_t rule = 0; const bool weld = false;
                sm_put(fn, p, sizeof(p), L"Parent", &parent, sizeof(void*), &ok);
                sm_put(fn, p, sizeof(p), L"SocketName", &socket, sizeof(int32_t) * 2, &ok);
                sm_put(fn, p, sizeof(p), L"LocationRule", &rule, 1, &ok);
                sm_put(fn, p, sizeof(p), L"RotationRule", &rule, 1, &ok);
                sm_put(fn, p, sizeof(p), L"ScaleRule", &rule, 1, &ok);
                sm_put(fn, p, sizeof(p), L"bWeldSimulatedBodies", &weld, 1, &ok);
                if (ok) fn->call(fol, p);
            }
        }
        if (auto* fn = sc_fn(fol, L"K2_SetRelativeLocationAndRotation")) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const bool no = false, yes = true;
            sm_put(fn, p, sizeof(p), L"NewLocation", loc, 24, &ok);
            sm_put(fn, p, sizeof(p), L"NewRotation", rot, 24, &ok);
            sm_put(fn, p, sizeof(p), L"bSweep", &no, 1, &ok);
            sm_put(fn, p, sizeof(p), L"bTeleport", &yes, 1, &ok);
            if (ok) fn->call(fol, p);
        }
        { alignas(16) uint8_t q[64] = {0}; memcpy(q, scl, 24); fol->call_function(L"SetRelativeScale3D", q); }
        // The follower takes its bone transforms from the leader. 5.1+ names it leader; older, master.
        bool led = false;
        for (const wchar_t* fname : { L"SetLeaderPoseComponent", L"SetMasterPoseComponent" }) {
            auto* fn = sc_fn(fol, fname);
            if (fn == nullptr) continue;
            alignas(16) uint8_t p[64] = {0}; bool ok = true; const bool force = true, tick = false;
            // First parameter = the leader, whatever its name in this engine version.
            for (API::FField* f = fn->get_child_properties(); f != nullptr; f = f->get_next()) {
                const auto* fnm = f->get_fname();
                if (fnm == nullptr) continue;
                const std::wstring nm = fnm->to_string();
                if (nm == L"ReturnValue") continue;
                if (f->get_class() && f->get_class()->get_name() == L"ObjectProperty") {
                    API::UObject* leader = (g_cfg.slide_copy_leader == 1) ? src : copy;
                    sm_put(fn, p, sizeof(p), nm.c_str(), &leader, sizeof(void*), &ok); break;
                }
            }
            sm_put(fn, p, sizeof(p), L"bForceUpdate", &force, 1, &ok);
            auto* tp = fn->find_property(L"bFollowerShouldTickPose"); if (tp == nullptr) tp = fn->find_property(L"bSlaveShouldTickPose");
            if (tp != nullptr) memcpy(p + tp->get_offset(), &tick, 1);
            if (ok) { fn->call(fol, p); led = true; API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: follower spawned, leader = %s, set via %ls", g_cfg.slide_copy_leader == 1 ? "the REAL mesh (draw test)" : "the poseable", fname); }
            break;
        }
        if (!led) API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: follower spawned but no leader-pose setter found");
        sc_set_visibility(fol, false); sc_set_visibility(fol, true);
        s_sc_follower.set(fol);
    }
    return true;
}
void sc_dump_asset_users(API::UObject* asset) {
    if (asset == nullptr) return;
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;
    const int32_t nn = arr->get_object_count();
    int n = 0;
    for (int32_t i = 0; i < nn && n < 24; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        const std::wstring cls = class_name_of(o);
        if (cls.find(L"MeshComponent") == std::wstring::npos) continue;
        API::UObject* mesh = nullptr;
        for (const wchar_t* nm : { L"SkinnedAsset", L"SkeletalMesh", L"StaticMesh" }) {
            auto** pm = o->get_property_data<API::UObject*>(nm);
            if (pm != nullptr && !IsBadReadPtr(pm, sizeof(void*)) && *pm != nullptr) { mesh = *pm; break; }
        }
        if (mesh != asset) continue;
        ++n;
        auto rb = [&](const wchar_t* nm) -> int {
            auto* pr = o->get_class()->find_property(nm);
            if (pr == nullptr || pr->get_class() == nullptr || pr->get_class()->get_name() != L"BoolProperty") return -1;
            return static_cast<API::FBoolProperty*>(pr)->get_value_from_object(o) ? 1 : 0;
        };
        int fpt = -1;
        if (auto* pt = o->get_property_data<uint8_t>(L"FirstPersonPrimitiveType")) if (!IsBadReadPtr(pt, 1)) fpt = (int)*pt;
        API::UObject* par = nullptr;
        if (auto** pp = o->get_property_data<API::UObject*>(L"AttachParent")) if (!IsBadReadPtr(pp, sizeof(void*))) par = *pp;
        double scl[3] = {0, 0, 0};
        if (auto* ps = o->get_property_data<double>(L"RelativeScale3D")) if (!IsBadReadPtr(ps, 24)) { scl[0] = ps[0]; scl[1] = ps[1]; scl[2] = ps[2]; }
        API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY asset user #%d: %ls '%ls' in %ls | mainpass=%d depth=%d visible=%d hidden=%d ownerNoSee=%d onlyOwner=%d fpType=%d | parent %ls '%ls' | scale (%.3f %.3f %.3f)",
                             n, cls.c_str(), o->get_fname() ? o->get_fname()->to_string().c_str() : L"?",
                             o->get_full_name().c_str(),
                             rb(L"bRenderInMainPass"), rb(L"bRenderInDepthPass"), rb(L"bVisible"), rb(L"bHiddenInGame"),
                             rb(L"bOwnerNoSee"), rb(L"bOnlyOwnerSee"), fpt,
                             par ? class_name_of(par).c_str() : L"null",
                             (par && par->get_fname()) ? par->get_fname()->to_string().c_str() : L"?",
                             scl[0], scl[1], scl[2]);
    }
    API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: %d component(s) use this asset", n);
}
// Double-precision transform helpers for the alignment (UE5 FTransform is doubles).
struct QD { double x, y, z, w; };
QD qd_mul(const QD& a, const QD& b) {
    return QD{ a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
               a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
               a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
               a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z };
}
QD qd_conj(const QD& q) { return QD{-q.x, -q.y, -q.z, q.w}; }
void qd_rot(const QD& q, const double v[3], double out[3]) {
    const double cx = q.y*v[2] - q.z*v[1], cy = q.z*v[0] - q.x*v[2], cz = q.x*v[1] - q.y*v[0];
    const double dx = q.y*cz - q.z*cy,     dy = q.z*cx - q.x*cz,     dz = q.x*cy - q.y*cx;
    out[0] = v[0] + 2.0*(q.w*cx + dx); out[1] = v[1] + 2.0*(q.w*cy + dy); out[2] = v[2] + 2.0*(q.w*cz + dz);
}
bool sc_socket_world(API::UObject* comp, const API::FName& name, XformD* out) {
    auto* fn = sc_fn(comp, L"GetSocketTransform");
    if (fn == nullptr) return false;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const uint8_t space = 0;   // RTS_World
    sm_put(fn, p, sizeof(p), L"InSocketName", &name, sizeof(int32_t) * 2, &ok);
    sm_put(fn, p, sizeof(p), L"TransformSpace", &space, 1, &ok);
    auto* r = fn->find_property(L"ReturnValue");
    if (!ok || r == nullptr) return false;
    fn->call(comp, p);
    memcpy(out, p + r->get_offset(), sizeof(XformD));
    return true;
}
bool sc_component_world(API::UObject* comp, XformD* out) {
    auto* fn = sc_fn(comp, L"K2_GetComponentToWorld");
    if (fn == nullptr) return false;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    auto* r = fn->find_property(L"ReturnValue");
    if (r == nullptr) return false;
    fn->call(comp, p);
    memcpy(out, p + r->get_offset(), sizeof(XformD));
    return true;
}
// Move the copy's component so its root bone lands on the real mesh's root bone:
// C' = R_real * inverse(R_copy) * C  (rotation and translation; scale left alone).
void sc_align_roots(API::UObject* src, API::UObject* copy) {
    static API::FName s_root; static std::string s_root_name; static bool s_have = false;
    const std::string want = trim_cfg(g_cfg.slide_copy_root);
    if (!s_have || want != s_root_name) { std::wstring w(want.begin(), want.end()); s_root = make_fname(w.c_str()); s_root_name = want; s_have = true; }
    XformD rr{}, rc{}, c{};
    if (!sc_socket_world(src, s_root, &rr) || !sc_socket_world(copy, s_root, &rc) || !sc_component_world(copy, &c)) return;
    const QD qr{rr.qx, rr.qy, rr.qz, rr.qw}, qc{rc.qx, rc.qy, rc.qz, rc.qw}, qC{c.qx, c.qy, c.qz, c.qw};
    // D = R_real * inverse(R_copy): rotation qd = qr * conj(qc); translation td = tr - qd * tc
    const QD qd = qd_mul(qr, qd_conj(qc));
    const double tc[3] = {rc.tx, rc.ty, rc.tz}; double tcr[3]; qd_rot(qd, tc, tcr);
    const double td[3] = {rr.tx - tcr[0], rr.ty - tcr[1], rr.tz - tcr[2]};
    // C' = D * C: rotation qd*qC, translation td + qd * tC
    const QD qn = qd_mul(qd, qC);
    const double tC[3] = {c.tx, c.ty, c.tz}; double tCr[3]; qd_rot(qd, tC, tCr);
    XformD n = c;
    n.qx = qn.x; n.qy = qn.y; n.qz = qn.z; n.qw = qn.w;
    n.tx = td[0] + tCr[0]; n.ty = td[1] + tCr[1]; n.tz = td[2] + tCr[2];
    if (auto* fn = sc_fn(copy, L"K2_SetWorldTransform")) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const bool no = false, yes = true;
        sm_put(fn, p, sizeof(p), L"NewTransform", &n, sizeof(XformD), &ok);
        sm_put(fn, p, sizeof(p), L"bSweep", &no, 1, &ok);
        sm_put(fn, p, sizeof(p), L"bTeleport", &yes, 1, &ok);
        if (ok) fn->call(copy, p);
    }
    if (g_cfg.slide_log && (s_sc_ticks % 90u) == 1u)
        API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY align: real root (%.1f %.1f %.1f) copy root (%.1f %.1f %.1f) copy comp (%.1f %.1f %.1f) -> comp' (%.1f %.1f %.1f)",
                             rr.tx, rr.ty, rr.tz, rc.tx, rc.ty, rc.tz, c.tx, c.ty, c.tz, n.tx, n.ty, n.tz);
}
void slide_copy_tick() {
    auto* src = reload_weapon_default_comp();
    auto* actor = fp_weapon_actor();
    const bool usable = g_cfg.slide_copy && g_cfg.slide_vr && src != nullptr && actor != nullptr && slide_weapon_ok();
    const std::string key = weapon_key();
    if (!usable || (s_sc_copy.get() != nullptr && (key != s_sc_key || s_sc_src.get() != src))) {
        sc_teardown(usable ? "weapon changed" : "unavailable");
        if (!usable) return;
    }
    if (s_sc_copy.get() == nullptr) {
        if (now_ticks() - s_sc_retry_at < ms_to_ticks(1500)) return;
        s_sc_retry_at = now_ticks();
        if (!sc_spawn(actor, src)) return;
        s_sc_key = key;
    }
    auto* copy = s_sc_copy.get();
    if (copy == nullptr) return;
    ++s_sc_ticks;
    if (g_cfg.slide_copy_mode == 1) {
        if (s_sc_seq.get() != nullptr && s_sc_len > 0.0f) {
            const float pull = g_slide_pull.load(std::memory_order_relaxed);
            float frac = (g_cfg.slide_travel > 0.0f) ? std::fmin(1.0f, std::fmax(0.0f, pull / g_cfg.slide_travel)) : 0.0f;
            if (s_true_empty || s_sl_lock_pending || s_sl_locked_back) frac = 1.0f;   // locked back
            float t = s_sc_len - frac * g_cfg.slide_copy_rel;
            if (g_cfg.slide_copy_sweep) {
                static long long s_t0 = 0;
                if (s_t0 == 0) s_t0 = now_ticks();
                const double sec = (double)(now_ticks() - s_t0) / (double)ms_to_ticks(1000);
                t = s_sc_len - 0.6f + (float)std::fmod(sec * 0.1, 0.6);   // 0.1 s of animation per second
                if (g_cfg.slide_log) { static long long s_said = 0; if (now_ticks() - s_said > ms_to_ticks(500)) { s_said = now_ticks(); API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY sweep t=%.3f (end-%.3f)", t, s_sc_len - t); } }
            }
            if (t < 0.0f) t = 0.0f;
            if (g_cfg.slide_copy_play != 1) {
                alignas(16) uint8_t p[64] = {0}; *reinterpret_cast<float*>(p) = t; p[4] = 0;
                copy->call_function(L"SetPosition", p);
            }
        }
        if (g_cfg.slide_copy_align) sc_align_roots(src, copy);
        if (g_cfg.slide_log && (s_sc_ticks % 90u) == 2u) {
            int rr = -1;
            if (auto* pr = copy->get_class()->find_property(L"bRecentlyRendered")) if (pr->get_class() && pr->get_class()->get_name() == L"BoolProperty") rr = static_cast<API::FBoolProperty*>(pr)->get_value_from_object(copy) ? 1 : 0;
            API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: copy bRecentlyRendered=%d", rr);
            // Where the animated bones land relative to the root: a degenerate or far pose is a
            // number here, not a theory.
            XformD r0{}, rb{}, rs{}, rr0{}, rrb{};
            const API::FName nroot = make_fname(L"Root_M"), nbody = make_fname(L"Body_M"), nslide = make_fname(L"Slide_M");
            if (sc_socket_world(copy, nroot, &r0) && sc_socket_world(copy, nbody, &rb) && sc_socket_world(copy, nslide, &rs)
                && sc_socket_world(src, nroot, &rr0) && sc_socket_world(src, nbody, &rrb)) {
                API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY bones: copy body-root (%.1f %.1f %.1f) slide-root (%.1f %.1f %.1f) scale %.2f | real body-root (%.1f %.1f %.1f) scale %.2f",
                                     rb.tx - r0.tx, rb.ty - r0.ty, rb.tz - r0.tz, rs.tx - r0.tx, rs.ty - r0.ty, rs.tz - r0.tz, rb.sx,
                                     rrb.tx - rr0.tx, rrb.ty - rr0.ty, rrb.tz - rr0.tz, rrb.sx);
            }
        }
        sc_hide_real(src, true);
        s_sc_hidden = true;
        if (s_sc_ticks == 1 && g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: live (single-node); real mesh hidden");
        return;
    }
    // The real mesh's pose, then our slide, then the real mesh out of sight.
    if (auto* fn = sc_fn(copy, L"CopyPoseFromSkeletalComponent")) {
        alignas(16) uint8_t p[64] = {0}; bool ok = true;
        sm_put(fn, p, sizeof(p), L"InComponentToCopy", &src, sizeof(void*), &ok);
        if (ok) fn->call(copy, p);
    }
    const float pull_cm = g_slide_pull.load(std::memory_order_relaxed) * 304.8f * g_cfg.slide_copy_sign;
    const float off_cm = pull_cm + g_cfg.slide_copy_test;
    if (s_sc_bone_ok && std::fabs(off_cm) > 0.0005f) {
        auto* gt = sc_fn(copy, L"GetBoneTransformByName");
        auto* st = sc_fn(copy, L"SetBoneTransformByName");
        if (gt != nullptr && st != nullptr) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const uint8_t space = 1;   // ComponentSpace
            sm_put(gt, p, sizeof(p), L"BoneName", &s_sc_bone, sizeof(int32_t) * 2, &ok);
            sm_put(gt, p, sizeof(p), L"BoneSpace", &space, 1, &ok);
            auto* r = gt->find_property(L"ReturnValue");
            if (ok && r != nullptr) {
                gt->call(copy, p);
                XformD t; memcpy(&t, p + r->get_offset(), sizeof(XformD));
                // The bone-local axis in component space: rotate the unit axis by the bone's rotation.
                double ax[3] = {0, 0, 0}; ax[g_cfg.slide_copy_axis] = 1.0;
                const double qx = t.qx, qy = t.qy, qz = t.qz, qw = t.qw;
                // v' = v + 2*w*(q x v) + 2*(q x (q x v))
                double cx = qy * ax[2] - qz * ax[1], cy = qz * ax[0] - qx * ax[2], cz = qx * ax[1] - qy * ax[0];
                double dx = qy * cz - qz * cy,       dy = qz * cx - qx * cz,       dz = qx * cy - qy * cx;
                const double vx = ax[0] + 2.0 * (qw * cx + dx), vy = ax[1] + 2.0 * (qw * cy + dy), vz = ax[2] + 2.0 * (qw * cz + dz);
                t.tx -= vx * off_cm; t.ty -= vy * off_cm; t.tz -= vz * off_cm;
                alignas(16) uint8_t q[RIG_PARAM_BUF] = {0}; bool ok2 = true;
                sm_put(st, q, sizeof(q), L"BoneName", &s_sc_bone, sizeof(int32_t) * 2, &ok2);
                sm_put(st, q, sizeof(q), L"InTransform", &t, sizeof(XformD), &ok2);
                sm_put(st, q, sizeof(q), L"BoneSpace", &space, 1, &ok2);
                if (ok2) st->call(copy, q);
                if (g_cfg.slide_log && (s_sc_ticks % 60u) == 0u)
                    API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: bone at (%.2f %.2f %.2f) axis%d=(%.2f %.2f %.2f) offset %.2f cm",
                                         t.tx, t.ty, t.tz, g_cfg.slide_copy_axis, vx, vy, vz, off_cm);
            }
        }
    }
    sc_hide_real(src, true);
    s_sc_hidden = true;
    if (s_sc_ticks == 1 && g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: live; real mesh hidden");
}

// ---- SLIDEPART v2. See Config.hpp. The gun as the game's own static parts, each riding its bone.
struct SlidePart { TrackedObject comp; API::FName bone; std::wstring bone_name; XformD bind; bool have_bind; bool is_slide; bool is_mag; bool bone_pivot; bool claimed; API::UObject* mesh; bool dropped; };
TrackedObject s_sp_dropped, s_sp_dropped_proxy; long long s_sp_dropped_at = 0;
// The real mesh's own child components, moved onto our body part while the rebuild lives.
struct SpKid { TrackedObject comp; API::FName socket; double loc[3]; double rot[3]; double scl[3]; };
SpKid s_sp_kids[16]; int s_sp_nkids = 0;
void sp_attach(API::UObject* c, API::UObject* parent, const API::FName& socket, uint8_t rule) {
    auto* fn = sc_fn(c, L"K2_AttachToComponent"); if (fn == nullptr) return;
    alignas(16) uint8_t p[128] = {0}; bool ok = true; const bool weld = false;
    sm_put(fn, p, sizeof(p), L"Parent", &parent, sizeof(void*), &ok);
    sm_put(fn, p, sizeof(p), L"SocketName", &socket, sizeof(int32_t) * 2, &ok);
    sm_put(fn, p, sizeof(p), L"LocationRule", &rule, 1, &ok);
    sm_put(fn, p, sizeof(p), L"RotationRule", &rule, 1, &ok);
    sm_put(fn, p, sizeof(p), L"ScaleRule", &rule, 1, &ok);
    sm_put(fn, p, sizeof(p), L"bWeldSimulatedBodies", &weld, 1, &ok);
    if (ok) fn->call(c, p);
}
void sp_kids_restore(API::UObject* src) {
    for (int i = 0; i < s_sp_nkids; ++i) {
        auto* c = s_sp_kids[i].comp.get();
        if (c == nullptr || src == nullptr) continue;
        sp_attach(c, src, s_sp_kids[i].socket, 0);   // KeepRelative, then the relative transform back
        if (auto* fn = sc_fn(c, L"K2_SetRelativeLocationAndRotation")) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const bool no = false, yes = true;
            sm_put(fn, p, sizeof(p), L"NewLocation", s_sp_kids[i].loc, 24, &ok);
            sm_put(fn, p, sizeof(p), L"NewRotation", s_sp_kids[i].rot, 24, &ok);
            sm_put(fn, p, sizeof(p), L"bSweep", &no, 1, &ok);
            sm_put(fn, p, sizeof(p), L"bTeleport", &yes, 1, &ok);
            if (ok) fn->call(c, p);
        }
        { alignas(16) uint8_t q[64] = {0}; memcpy(q, s_sp_kids[i].scl, 24); c->call_function(L"SetRelativeScale3D", q); }
        { alignas(16) uint8_t q[64] = {0}; c->call_function(L"SetAbsolute", q); }   // all three false again
        s_sp_kids[i] = SpKid{};
    }
    s_sp_nkids = 0;
}
// Every component attached to the real mesh: logged, and moved per slide_part_kids.
void sp_kids_take(API::UObject* src, API::UObject* body_part) {
    s_sp_nkids = 0;
    weapon_components([&](API::UObject* c) {
        if (s_sp_nkids >= 16) return false;
        if (c == src) return true;
        auto** pp = c->get_property_data<API::UObject*>(L"AttachParent");
        if (pp == nullptr || IsBadReadPtr(pp, sizeof(void*)) || *pp != src) return true;
        SpKid& k = s_sp_kids[s_sp_nkids];
        API::UObject* par = nullptr;
        if (!sc_read_attach(c, &par, &k.socket, k.loc, k.rot, k.scl)) return true;
        const auto* fn = c->get_fname();
        std::wstring mesh = L"-";
        if (auto** pm = c->get_property_data<API::UObject*>(L"StaticMesh")) if (!IsBadReadPtr(pm, sizeof(void*)) && *pm != nullptr && (*pm)->get_fname()) mesh = (*pm)->get_fname()->to_string();
        API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART kid: %ls '%ls' mesh %ls at socket '%ls' rel (%.1f %.1f %.1f) -> %s",
                             class_name_of(c).c_str(), fn ? fn->to_string().c_str() : L"?", mesh.c_str(), k.socket.to_string().c_str(),
                             k.loc[0], k.loc[1], k.loc[2],
                             g_cfg.slide_part_kids == 1 ? "onto our body part" : (g_cfg.slide_part_kids == 2 ? "absolute scale" : "left"));
        if (g_cfg.slide_part_kids == 1 && body_part != nullptr) {
            k.comp.set(c);
            sp_attach(c, body_part, make_fname(L"None"), 1);   // KeepWorld
            ++s_sp_nkids;
        } else if (g_cfg.slide_part_kids == 2) {
            k.comp.set(c);
            { alignas(16) uint8_t q[64] = {0}; q[2] = 1; c->call_function(L"SetAbsolute", q); }   // scale absolute
            { alignas(16) uint8_t q[64] = {0}; auto* d = reinterpret_cast<double*>(q); d[0] = k.scl[0]; d[1] = k.scl[1]; d[2] = k.scl[2]; c->call_function(L"SetRelativeScale3D", q); }
            ++s_sp_nkids;
        }
        return true;
    });
}
SlidePart     s_sp_parts[12];
int           s_sp_count = 0;
TrackedObject s_sp_src;
std::string   s_sp_key;
bool          s_sp_hidden = false;
long long     s_sp_retry_at = 0;
uint32_t      s_sp_ticks = 0;
bool sp_socket_component(API::UObject* comp, const API::FName& name, XformD* out) {
    auto* fn = sc_fn(comp, L"GetSocketTransform");
    if (fn == nullptr) return false;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const uint8_t space = 2;   // RTS_Component
    sm_put(fn, p, sizeof(p), L"InSocketName", &name, sizeof(int32_t) * 2, &ok);
    sm_put(fn, p, sizeof(p), L"TransformSpace", &space, 1, &ok);
    auto* r = fn->find_property(L"ReturnValue");
    if (!ok || r == nullptr) return false;
    fn->call(comp, p);
    memcpy(out, p + r->get_offset(), sizeof(XformD));
    return true;
}
// THE PART'S GEOMETRIC CENTRE IN THE WORLD (2026-09-06): the pump's socket and component sit at
// the gun's root (the mesh pivot is at the receiver), so only the geometry's centre puts the zone
// on the pump. Three sources, first non-zero wins: K2_GetComponentBounds' Origin; the component's
// GetLocalBounds (Min, Max) through K2_GetComponentToWorld; the static mesh asset's
// GetBoundingBox through the same transform.
bool part_world_centre(API::UObject* c, Vec3* out, const char** how) {
    if (c == nullptr || out == nullptr) return false;
    auto nonzero = [](const double* v) { return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]) && (std::fabs(v[0]) + std::fabs(v[1]) + std::fabs(v[2]) > 1e-3); };
    {   // 1. K2_GetComponentBounds(Origin, BoxExtent, SphereRadius): Origin at 0
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        c->call_function(L"K2_GetComponentBounds", p);
        const double* o = reinterpret_cast<const double*>(p);
        if (nonzero(o)) { *out = Vec3{(float)o[0], (float)o[1], (float)o[2]}; *how = "bounds"; return true; }
    }
    // The component's world transform: FTransform of doubles, quat @0, translation @0x20, scale @0x40.
    alignas(16) uint8_t tp[RIG_PARAM_BUF] = {0};
    c->call_function(L"K2_GetComponentToWorld", tp);
    const double* q = reinterpret_cast<const double*>(tp);
    const double* tr = reinterpret_cast<const double*>(tp + 0x20);
    const double* sc = reinterpret_cast<const double*>(tp + 0x40);
    const bool have_xf = std::isfinite(q[3]) && std::fabs(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] - 1.0) < 0.05;
    auto to_world = [&](const double local[3], Vec3* w) {
        const double sl[3] = {local[0] * sc[0], local[1] * sc[1], local[2] * sc[2]};
        double r[3]; qd_rot(QD{q[0], q[1], q[2], q[3]}, sl, r);
        *w = Vec3{(float)(r[0] + tr[0]), (float)(r[1] + tr[1]), (float)(r[2] + tr[2])};
    };
    if (have_xf) {
        {   // 2. GetLocalBounds(Min @0, Max @0x18)
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            c->call_function(L"GetLocalBounds", p);
            const double* mn = reinterpret_cast<const double*>(p); const double* mx = reinterpret_cast<const double*>(p + 0x18);
            if (nonzero(mn) || nonzero(mx)) { const double ctr[3] = {(mn[0] + mx[0]) * 0.5, (mn[1] + mx[1]) * 0.5, (mn[2] + mx[2]) * 0.5}; to_world(ctr, out); *how = "localbounds"; return true; }
        }
        {   // 3. the asset: StaticMesh->GetBoundingBox() = FBox {Min @0, Max @0x18, IsValid}
            auto** pm = c->get_property_data<API::UObject*>(L"StaticMesh");
            if (pm != nullptr && !IsBadReadPtr(pm, sizeof(void*)) && *pm != nullptr && !IsBadReadPtr(*pm, sizeof(void*))) {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                (*pm)->call_function(L"GetBoundingBox", p);
                const double* mn = reinterpret_cast<const double*>(p); const double* mx = reinterpret_cast<const double*>(p + 0x18);
                if (nonzero(mn) || nonzero(mx)) { const double ctr[3] = {(mn[0] + mx[0]) * 0.5, (mn[1] + mx[1]) * 0.5, (mn[2] + mx[2]) * 0.5}; to_world(ctr, out); *how = "assetbox"; return true; }
            }
        }
    }
    *how = "none";
    return false;
}
API::FName sp_slide_bone_name() {
    static API::FName s_name; static std::string s_last; static bool s_have = false;
    const std::string want = trim_cfg(g_cfg.slide_part_bone);
    if (!s_have || want != s_last) { std::wstring w(want.begin(), want.end()); s_name = make_fname(w.c_str()); s_last = want; s_have = true; }
    return s_name;
}
// The arms hide's recipe: keep the pose evaluating, then hide the bone (PBO_None). Re-issued
// every tick here, and the engine is asked whether it holds the bone hidden.
void sp_hide_bone(API::UObject* src, bool hide) {
    if (src == nullptr) return;
    const API::FName bone = sp_slide_bone_name();
    if (hide) { alignas(16) uint8_t p[64] = {0}; p[0] = 0; src->call_function(L"SetVisibilityBasedAnimTickOption", p); }
    auto* fn = sc_fn(src, hide ? L"HideBoneByName" : L"UnHideBoneByName");
    if (fn == nullptr) return;
    alignas(16) uint8_t p[64] = {0}; bool ok = true; const uint8_t pbo = (uint8_t)g_cfg.slide_hide_pbo;
    sm_put(fn, p, sizeof(p), L"BoneName", &bone, sizeof(int32_t) * 2, &ok);
    if (hide) { auto* pr = fn->find_property(L"PhysBodyOption"); if (pr) memcpy(p + pr->get_offset(), &pbo, 1); }
    if (ok) fn->call(src, p);
}
int sp_is_bone_hidden(API::UObject* src) {
    auto* fn = sc_fn(src, L"IsBoneHiddenByName");
    if (fn == nullptr) return -1;
    const API::FName bone = sp_slide_bone_name();
    alignas(16) uint8_t p[64] = {0}; bool ok = true;
    sm_put(fn, p, sizeof(p), L"BoneName", &bone, sizeof(int32_t) * 2, &ok);
    auto* r = fn->find_property(L"ReturnValue");
    if (!ok || r == nullptr) return -1;
    fn->call(src, p);
    return p[r->get_offset()] ? 1 : 0;
}
// The real mesh's material slots, by name, once: which one is the slide?
void sp_log_materials(API::UObject* src) {
    int32_t n = 0;
    if (auto* fn = sc_fn(src, L"GetNumMaterials")) { alignas(16) uint8_t p[64] = {0}; fn->call(src, p); auto* r = fn->find_property(L"ReturnValue"); if (r) n = *reinterpret_cast<int32_t*>(p + r->get_offset()); }
    std::wstring line;
    auto* gm = sc_fn(src, L"GetMaterial");
    for (int32_t i = 0; gm != nullptr && i < n && i < 32; ++i) {
        alignas(16) uint8_t p[64] = {0}; bool ok = true;
        sm_put(gm, p, sizeof(p), L"ElementIndex", &i, sizeof(int32_t), &ok);
        auto* r = gm->find_property(L"ReturnValue");
        if (!ok || r == nullptr) break;
        gm->call(src, p);
        auto* m = *reinterpret_cast<API::UObject**>(p + r->get_offset());
        line += std::to_wstring(i) + L":" + ((m && !IsBadReadPtr(m, sizeof(void*)) && m->get_fname()) ? m->get_fname()->to_string() : L"null") + L"  ";
    }
    API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: real mesh has %d material slot(s): %ls", n, line.c_str());
}
// ShowMaterialSection(int32 MaterialID, int32 SectionIndex, bool bShow, int32 LODIndex)
void sp_show_section(API::UObject* src, int32_t material_id, bool show) {
    auto* fn = sc_fn(src, L"ShowMaterialSection");
    if (fn == nullptr) { static bool s_said = false; if (!s_said) { s_said = true; API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: no ShowMaterialSection"); } return; }
    for (int32_t lod = 0; lod < 4; ++lod) {
        alignas(16) uint8_t p[64] = {0}; bool ok = true; const int32_t sec = 0; const bool b = show;
        sm_put(fn, p, sizeof(p), L"MaterialID", &material_id, sizeof(int32_t), &ok);
        sm_put(fn, p, sizeof(p), L"SectionIndex", &sec, sizeof(int32_t), &ok);
        sm_put(fn, p, sizeof(p), L"bShow", &b, 1, &ok);
        sm_put(fn, p, sizeof(p), L"LODIndex", &lod, sizeof(int32_t), &ok);
        if (ok) fn->call(src, p);
    }
}
int s_sp_section_hidden = -1;
int s_sp_mode = -1;
// The real component's bone transform arrays, found by SHAPE: a TArray header {data, num, max}
// with num == bone count whose elements are UE5 double FTransforms (unit quaternion, sane scale).
// The component-space arrays carry the slide at its component position (5.5, 0, 13 on the
// magnum); the bone-space array carries it relative to its parent. Logged, never assumed.
struct BoneArray { uintptr_t data; int32_t off; bool component_space; };
BoneArray s_sp_arrays[6]; int s_sp_narrays = 0; int s_sp_slide_index = -1;
void sp_find_bone_arrays(API::UObject* src, int32_t nbones, int32_t slide_index, const XformD& slide_component) {
    s_sp_narrays = 0;
    const uint8_t* base = reinterpret_cast<const uint8_t*>(src);
    constexpr int32_t SPAN = 0x1800;
    if (IsBadReadPtr(base, SPAN)) return;
    for (int32_t off = 0; off + 16 <= SPAN && s_sp_narrays < 6; off += 8) {
        const uintptr_t data = *reinterpret_cast<const uintptr_t*>(base + off);
        const int32_t num = *reinterpret_cast<const int32_t*>(base + off + 8);
        const int32_t mx  = *reinterpret_cast<const int32_t*>(base + off + 12);
        if (num != nbones || mx < num || mx > num + 64 || data == 0 || (data & 0xF) != 0) continue;
        if (IsBadReadPtr((const void*)data, sizeof(XformD) * (size_t)num)) continue;
        bool ok = true;
        for (int32_t i = 0; i < num && ok; ++i) {
            const XformD* t = reinterpret_cast<const XformD*>(data + sizeof(XformD) * (size_t)i);
            const double qn = t->qx*t->qx + t->qy*t->qy + t->qz*t->qz + t->qw*t->qw;
            if (!(qn > 0.9 && qn < 1.1)) ok = false;
            if (!(std::fabs(t->sx) < 100.0 && std::fabs(t->sy) < 100.0 && std::fabs(t->sz) < 100.0)) ok = false;
            if (!(std::fabs(t->tx) < 100000.0 && std::fabs(t->ty) < 100000.0 && std::fabs(t->tz) < 100000.0)) ok = false;
        }
        if (!ok) continue;
        const XformD* sl = reinterpret_cast<const XformD*>(data + sizeof(XformD) * (size_t)slide_index);
        const bool comp = std::fabs(sl->tx - slide_component.tx) < 0.05 && std::fabs(sl->ty - slide_component.ty) < 0.05 && std::fabs(sl->tz - slide_component.tz) < 0.05;
        s_sp_arrays[s_sp_narrays++] = BoneArray{data, off, comp};
        API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART far: transform array at +0x%X (%d bones): slide entry t=(%.2f %.2f %.2f) s=(%.2f %.2f %.2f) -> %s",
                             (unsigned)off, num, sl->tx, sl->ty, sl->tz, sl->sx, sl->sy, sl->sz, comp ? "COMPONENT space" : "bone space (parent-relative)");
    }
    if (s_sp_narrays == 0) API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART far: no transform array of %d bones found in the component", nbones);
}
void sp_park_far_tick(API::UObject* src) {
    if (s_sp_slide_index < 0) return;
    int wrote = 0;
    for (int i = 0; i < s_sp_narrays; ++i) {
        const bool want_comp = (g_cfg.slide_far_array == 1);
        if (s_sp_arrays[i].component_space != want_comp) continue;
        auto* t = reinterpret_cast<XformD*>(s_sp_arrays[i].data + sizeof(XformD) * (size_t)s_sp_slide_index);
        if (IsBadWritePtr(t, sizeof(XformD))) continue;
        t->tz = (double)g_cfg.slide_far_cm;
        ++wrote;
    }
    if (g_cfg.slide_log && (s_sp_ticks % 90u) == 7u) {
        XformD now{};
        const bool have = sp_socket_component(src, sp_slide_bone_name(), &now);
        API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART far: wrote %d array(s); the slide bone now reads component z=%.1f (%s)",
                             wrote, have ? now.tz : 0.0, (have && now.tz < -100.0) ? "PARKED" : "still in place");
    }
}
// ---- approach C: morph targets on the real mesh
void sp_log_morphs(API::UObject* src) {
    API::UObject* mesh = nullptr;
    for (const wchar_t* nm : { L"SkinnedAsset", L"SkeletalMesh" }) {
        auto** pm = src->get_property_data<API::UObject*>(nm);
        if (pm != nullptr && !IsBadReadPtr(pm, sizeof(void*)) && *pm != nullptr) { mesh = *pm; break; }
    }
    if (mesh == nullptr) return;
    auto* pr = mesh->get_class()->find_property(L"MorphTargets");
    if (pr == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART morph: no MorphTargets property on %ls", class_name_of(mesh).c_str()); return; }
    const uint8_t* arr = reinterpret_cast<const uint8_t*>(mesh) + pr->get_offset();
    if (IsBadReadPtr(arr, 16)) return;
    const uintptr_t data = *reinterpret_cast<const uintptr_t*>(arr);
    const int32_t num = *reinterpret_cast<const int32_t*>(arr + 8);
    std::wstring names;
    for (int32_t i = 0; i < num && i < 32 && data != 0; ++i) {
        auto* o = *reinterpret_cast<API::UObject* const*>(data + sizeof(void*) * (size_t)i);
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*)) || o->get_fname() == nullptr) continue;
        names += o->get_fname()->to_string() + L"  ";
    }
    API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART morph: %d morph target(s): %ls", num, names.empty() ? L"(none)" : names.c_str());
}
void sp_morph_tick(API::UObject* src) {
    const std::string spec = trim_cfg(g_cfg.slide_morph);
    const size_t comma = spec.find(',');
    if (spec.empty() || comma == std::string::npos) return;
    const std::string name = spec.substr(0, comma);
    const float w = (float)atof(spec.c_str() + comma + 1);
    auto* fn = sc_fn(src, L"SetMorphTarget");
    if (fn == nullptr) return;
    std::wstring wn(name.begin(), name.end());
    API::FName mn = make_fname(wn.c_str());
    alignas(16) uint8_t p[64] = {0}; bool ok = true; const bool rm = false;
    sm_put(fn, p, sizeof(p), L"MorphTargetName", &mn, sizeof(int32_t) * 2, &ok);
    sm_put(fn, p, sizeof(p), L"Value", &w, sizeof(float), &ok);
    auto* pz = fn->find_property(L"bRemoveZeroWeight"); if (pz) memcpy(p + pz->get_offset(), &rm, 1);
    if (ok) fn->call(src, p);
}
TrackedObject s_sp_mat_orig; int s_sp_mat_slot = -1;
API::UObject* sp_get_material(API::UObject* src, int32_t slot) {
    auto* gm = sc_fn(src, L"GetMaterial"); if (gm == nullptr) return nullptr;
    alignas(16) uint8_t p[64] = {0}; bool ok = true;
    sm_put(gm, p, sizeof(p), L"ElementIndex", &slot, sizeof(int32_t), &ok);
    auto* r = gm->find_property(L"ReturnValue"); if (!ok || r == nullptr) return nullptr;
    gm->call(src, p);
    return *reinterpret_cast<API::UObject**>(p + r->get_offset());
}
void sp_set_material(API::UObject* src, int32_t slot, API::UObject* mat) {
    auto* fn = sc_fn(src, L"SetMaterial"); if (fn == nullptr) return;
    alignas(16) uint8_t p[64] = {0}; bool ok = true;
    sm_put(fn, p, sizeof(p), L"ElementIndex", &slot, sizeof(int32_t), &ok);
    sm_put(fn, p, sizeof(p), L"Material", &mat, sizeof(void*), &ok);
    if (ok) fn->call(src, p);
}
// A loaded material whose name carries the substring; candidates that sound invisible are logged once.
API::UObject* sp_find_material(const std::string& sub) {
    std::wstring want(sub.begin(), sub.end()); for (auto& ch : want) ch = (wchar_t)towlower(ch);
    auto* arr = API::get()->get_uobject_array(); if (arr == nullptr) return nullptr;
    const int32_t nn = arr->get_object_count();
    API::UObject* hit = nullptr;
    static bool s_listed = false; int listed = 0; std::wstring cands;
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        const std::wstring cls = class_name_of(o);
        if (cls != L"Material" && cls != L"MaterialInstanceConstant" && cls != L"MaterialInstanceDynamic") continue;
        const auto* fn = o->get_fname(); if (fn == nullptr) continue;
        std::wstring nm = fn->to_string(); std::wstring lo = nm; for (auto& ch : lo) ch = (wchar_t)towlower(ch);
        if (hit == nullptr && !want.empty() && lo.find(want) != std::wstring::npos) hit = o;
        if (!s_listed && listed < 24 && (lo.find(L"invis") != std::wstring::npos || lo.find(L"transp") != std::wstring::npos || lo.find(L"hidden") != std::wstring::npos ||
                                        lo.find(L"occlu") != std::wstring::npos || lo.find(L"empty") != std::wstring::npos || lo.find(L"clear") != std::wstring::npos || lo.find(L"nodraw") != std::wstring::npos)) {
            cands += nm + L"  "; ++listed;
        }
    }
    if (!s_listed) { s_listed = true; API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: invisible-sounding materials loaded: %ls", cands.empty() ? L"(none)" : cands.c_str()); }
    return hit;
}
void sp_teardown(const char* why) {
    if (s_sp_mat_slot >= 0) { if (auto* src = s_sp_src.get()) sp_set_material(src, s_sp_mat_slot, s_sp_mat_orig.get()); s_sp_mat_slot = -1; s_sp_mat_orig = TrackedObject{}; }
    if (s_sp_section_hidden >= 0) { if (auto* src = s_sp_src.get()) sp_show_section(src, s_sp_section_hidden, true); s_sp_section_hidden = -1; }
    for (int i = 0; i < s_sp_count; ++i) {
        if (auto* c = s_sp_parts[i].comp.get()) { alignas(16) uint8_t p[64] = {0}; ue_destroy_component(c); }
        s_sp_parts[i] = SlidePart{};
    }
    if (s_sp_hidden) { if (auto* src = s_sp_src.get()) { sc_set_scale(src, 1.0); sc_set_visibility(src, true); sp_hide_bone(src, false); sp_kids_restore(src); } }
    if (s_sp_count > 0 || s_sp_hidden) { if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: torn down (%s)", why); }
    g_sl_part_valid.store(false, std::memory_order_relaxed);
    if (auto* d = s_sp_dropped.get()) ue_destroy_component(d);
    if (auto* px = s_sp_dropped_proxy.get()) ue_destroy_component(px);
    s_sp_dropped = TrackedObject{}; s_sp_dropped_proxy = TrackedObject{};
    s_sp_count = 0; s_sp_src = TrackedObject{}; s_sp_hidden = false; s_sp_key.clear(); s_sp_ticks = 0;
    g_slide_rack_found = false;   // the native mode re-sets it when its part is found
}
API::UObject* sp_find_static_mesh(const std::wstring& shortname_in) {
    std::wstring shortname = shortname_in;
    for (auto& ch : shortname) ch = (wchar_t)towlower(ch);
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr || shortname.empty()) return nullptr;
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"StaticMesh") continue;
        const auto* fn = o->get_fname(); if (fn == nullptr) continue;
        std::wstring nm = fn->to_string(); for (auto& ch : nm) ch = (wchar_t)towlower(ch);
        if (nm == shortname) return o;
    }
    return nullptr;
}
API::UObject* sp_spawn_part(API::UObject* actor, API::UObject* mesh, API::UObject* parent, const API::FName& socket,
                            const double loc[3], const double rot[3], const double scl[3]) {
    auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.StaticMeshComponent");
    auto* part = cls ? API::get()->add_component_by_class(actor, cls, false) : nullptr;
    if (part == nullptr) return nullptr;
    { alignas(16) uint8_t p[64] = {0}; *reinterpret_cast<void**>(p) = mesh; part->call_function(L"SetStaticMesh", p); }
    { alignas(16) uint8_t p[64] = {0}; p[0] = 0; part->call_function(L"SetCollisionEnabled", p); }
    { alignas(16) uint8_t p[64] = {0}; p[0] = g_cfg.slide_part_shadow ? 1 : 0; part->call_function(L"SetCastShadow", p); }
    if (parent != nullptr) {
        if (auto* fn = sc_fn(part, L"K2_AttachToComponent")) {
            alignas(16) uint8_t p[128] = {0}; bool ok = true; const uint8_t rule = 0; const bool weld = false;
            sm_put(fn, p, sizeof(p), L"Parent", &parent, sizeof(void*), &ok);
            sm_put(fn, p, sizeof(p), L"SocketName", &socket, sizeof(int32_t) * 2, &ok);
            sm_put(fn, p, sizeof(p), L"LocationRule", &rule, 1, &ok);
            sm_put(fn, p, sizeof(p), L"RotationRule", &rule, 1, &ok);
            sm_put(fn, p, sizeof(p), L"ScaleRule", &rule, 1, &ok);
            sm_put(fn, p, sizeof(p), L"bWeldSimulatedBodies", &weld, 1, &ok);
            if (ok) fn->call(part, p);
        }
    }
    if (auto* fn = sc_fn(part, L"K2_SetRelativeLocationAndRotation")) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const bool no = false, yes = true;
        sm_put(fn, p, sizeof(p), L"NewLocation", loc, 24, &ok);
        sm_put(fn, p, sizeof(p), L"NewRotation", rot, 24, &ok);
        sm_put(fn, p, sizeof(p), L"bSweep", &no, 1, &ok);
        sm_put(fn, p, sizeof(p), L"bTeleport", &yes, 1, &ok);
        if (ok) fn->call(part, p);
    }
    { alignas(16) uint8_t q[64] = {0}; memcpy(q, scl, 24); part->call_function(L"SetRelativeScale3D", q); }
    return part;
}
// The rackable bone for this weapon stem, from the slide_bones table; slide_part_bone otherwise.
std::wstring sp_rack_bone_in(const char* table, const std::wstring& stem) {
    std::string tbl = trim_cfg(table);
    std::string lstem(stem.begin(), stem.end()); for (auto& ch : lstem) ch = (char)tolower((unsigned char)ch);
    size_t pos = 0;
    while (pos < tbl.size()) {
        size_t comma = tbl.find(',', pos); if (comma == std::string::npos) comma = tbl.size();
        std::string ent = tbl.substr(pos, comma - pos);
        const size_t colon = ent.find(':');
        if (colon != std::string::npos) {
            std::string w = ent.substr(0, colon), b = ent.substr(colon + 1);
            for (auto& ch : w) ch = (char)tolower((unsigned char)ch);
            while (!b.empty() && (unsigned char)b.back() <= ' ') b.pop_back();
            if (lstem.rfind(w, 0) == 0) return std::wstring(b.begin(), b.end());   // the table name is a PREFIX of the stem (SK_ShotgunCommon_Default -> shotguncommon)
        }
        pos = comma + 1;
    }
    return L"";
}
std::wstring sp_rack_bone_for(const std::wstring& stem) {
    std::wstring r = sp_rack_bone_in(g_cfg.slide_bones_override, stem);   // the cfg's per-weapon overrides first
    if (r.empty()) r = sp_rack_bone_in(g_cfg.slide_bones, stem);
    if (r.empty()) { const std::string d = trim_cfg(g_cfg.slide_part_bone); r.assign(d.begin(), d.end()); }
    return r;
}
// Every loaded StaticMesh named SM_<stem>_..., lowercased, for the per-bone match.
void sp_collect_weapon_meshes(const std::wstring& stem, std::vector<std::pair<std::wstring, API::UObject*>>& out) {
    std::wstring prefix = L"sm_" + stem + L"_"; for (auto& ch : prefix) ch = (wchar_t)towlower(ch);
    auto* arr = API::get()->get_uobject_array(); if (arr == nullptr) return;
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"StaticMesh") continue;
        const auto* fn = o->get_fname(); if (fn == nullptr) continue;
        std::wstring nm = fn->to_string(); for (auto& ch : nm) ch = (wchar_t)towlower(ch);
        if (nm.rfind(prefix, 0) != 0) continue;
        if (nm.find(L"shadow") != std::wstring::npos) continue;
        if (!g_cfg.slide_part_ui && (nm.find(L"_ui_") != std::wstring::npos || nm.rfind(prefix + L"ui_", 0) == 0)) continue;
        out.emplace_back(nm, o);
    }
}
bool sp_spawn(API::UObject* actor, API::UObject* src) {
    // The weapon stem from the asset name: SK_Magnum_Default -> Magnum.
    API::UObject* mesh = nullptr;
    for (const wchar_t* nm : { L"SkinnedAsset", L"SkeletalMesh" }) {
        auto** pm = src->get_property_data<API::UObject*>(nm);
        if (pm != nullptr && !IsBadReadPtr(pm, sizeof(void*)) && *pm != nullptr) { mesh = *pm; break; }
    }
    if (mesh == nullptr || mesh->get_fname() == nullptr) return false;
    std::wstring asset = mesh->get_fname()->to_string();
    std::wstring stem = asset;
    if (stem.rfind(L"SK_", 0) == 0) stem = stem.substr(3);
    stem = weapon_stem_from_mesh(stem);
    API::UObject* parent = nullptr; API::FName socket{}; double loc[3] = {0}, rot[3] = {0}, scl[3] = {1, 1, 1};
    if (!sc_read_attach(src, &parent, &socket, loc, rot, scl)) return false;
    // The bind pose: a reference-pose skeletal copy, read once, destroyed.
    API::UObject* ref = nullptr;
    if (auto* scls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.SkeletalMeshComponent")) {
        ref = API::get()->add_component_by_class(actor, scls, false);
        if (ref != nullptr) {
            for (const wchar_t* fname : { L"SetSkinnedAssetAndUpdate", L"SetSkeletalMesh" }) {
                auto* fn = sc_fn(ref, fname); if (fn == nullptr) continue;
                alignas(16) uint8_t p[64] = {0}; bool ok = true; const bool reinit = true;
                sm_put(fn, p, sizeof(p), L"NewMesh", &mesh, sizeof(void*), &ok);
                sm_put(fn, p, sizeof(p), L"bReinitPose", &reinit, 1, &ok);
                if (ok) { fn->call(ref, p); break; }
            }
            sc_set_visibility(ref, false);
        }
    }
    const std::wstring want_slide = sp_rack_bone_for(stem);
    std::vector<std::pair<std::wstring, API::UObject*>> meshes;
    sp_collect_weapon_meshes(stem, meshes);
    std::vector<bool> claimed(meshes.size(), false);
    API::UObject* real_mat = g_cfg.slide_part_mat ? sp_get_material(src, 0) : nullptr;
    API::FName body_bone{}; XformD body_bind{}; bool have_body = false;
    int32_t nb = 0;
    if (auto* fn = sc_fn(src, L"GetNumBones")) { alignas(16) uint8_t p[64] = {0}; fn->call(src, p); auto* r = fn->find_property(L"ReturnValue"); if (r) nb = *reinterpret_cast<int32_t*>(p + r->get_offset()); }
    auto* gb = sc_fn(src, L"GetBoneName");
    std::wstring report;
    s_sp_count = 0;
    for (int32_t i = 0; gb != nullptr && i < nb && s_sp_count < 12; ++i) {
        alignas(16) uint8_t p[64] = {0}; bool ok = true;
        sm_put(gb, p, sizeof(p), L"BoneIndex", &i, sizeof(int32_t), &ok);
        auto* r = gb->find_property(L"ReturnValue");
        if (!ok || r == nullptr) break;
        gb->call(src, p);
        API::FName bn; memcpy(&bn, p + r->get_offset(), sizeof(int32_t) * 2);
        const std::wstring bname = bn.to_string();
        if (bname == L"World") continue;
        if (g_cfg.slide_part_hide != 2 && bname != want_slide) continue;   // every mode but the rebuild: the slide alone
        if (bname == want_slide) s_sp_slide_index = i;
        // Every shipped part that belongs to this bone. Names seen in the pak: SM_Magnum_Slide_Default
        // (suffix dropped), SM_AssaultRifle_GunBody_M_Default (suffix kept), SM_BattleRifle_Root_M_Default_Scope
        // (several parts per bone), SM_Shotgun_Glass_HandleJnt_M_Default (a prefixed extra).
        std::wstring bstem = bname;
        if (bstem.size() > 2 && (bstem.compare(bstem.size() - 2, 2, L"_M") == 0 || bstem.compare(bstem.size() - 2, 2, L"_L") == 0 || bstem.compare(bstem.size() - 2, 2, L"_R") == 0)) bstem = bstem.substr(0, bstem.size() - 2);
        std::wstring lb = bname, ls = bstem, lstem = stem;
        for (auto& ch : lb) ch = (wchar_t)towlower(ch);
        for (auto& ch : ls) ch = (wchar_t)towlower(ch);
        for (auto& ch : lstem) ch = (wchar_t)towlower(ch);
        int found_here = 0;
        for (auto& mp : meshes) {
            if (s_sp_count >= 12) break;
            const std::wstring& nm = mp.first;   // sm_<stem>_...
            const std::wstring rest = nm.substr(3 + lstem.size() + 1);
            bool match = false;
            for (const std::wstring& b : { lb, ls }) {
                if (rest == b + L"_default") match = true;
                else if (rest.rfind(b + L"_default_", 0) == 0) match = true;
                else if (rest == L"glass_" + b + L"_default" || rest == L"screen_" + b + L"_default") match = true;
            }
            if (!match) continue;
            // A stem match must not also be a longer bone's name (Root_M vs Root_M_Default_Body is
            // fine; Wing1Jnt vs Wing1Jnt_L is not): the bone's own full name wins where both exist.
            auto* part = sp_spawn_part(actor, mp.second, parent, socket, loc, rot, scl);
            if (part == nullptr) { report += bname + L":spawn-failed  "; continue; }
            claimed[&mp - &meshes[0]] = true;
            if (real_mat != nullptr) sp_set_material(part, 0, real_mat);
            SlidePart& sp = s_sp_parts[s_sp_count++];
            sp.comp.set(part); sp.bone = bn; sp.bone_name = bname; sp.mesh = mp.second; sp.dropped = false;
            if (g_cfg.slide_part_bind == 1) sp.have_bind = (ref != nullptr) && sp_socket_component(ref, bn, &sp.bind);
            else                            sp.have_bind = sp_socket_component(src, bn, &sp.bind);   // the live pose, now
            sp.is_slide = (bname == want_slide);
            sp.is_mag = (lb.find(L"magazine") != std::wstring::npos || lb.find(L"megazine") != std::wstring::npos);
            // Where the mesh's geometry sits relative to its pivot: UStaticMesh::GetBoundingBox.
            double cx = 0, cy = 0, cz = 0, ext = 0; bool have_box = false;
            if (auto* fn = sc_fn(mp.second, L"GetBoundingBox")) {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                auto* r = fn->find_property(L"ReturnValue");
                if (r != nullptr) {
                    fn->call(mp.second, p);
                    const double* b = reinterpret_cast<const double*>(p + r->get_offset());   // FBox: Min xyz, Max xyz
                    cx = 0.5 * (b[0] + b[3]); cy = 0.5 * (b[1] + b[4]); cz = 0.5 * (b[2] + b[5]);
                    ext = std::sqrt((b[3]-b[0])*(b[3]-b[0]) + (b[4]-b[1])*(b[4]-b[1]) + (b[5]-b[2])*(b[5]-b[2]));
                    have_box = true;
                }
            }
            const double bone_r = std::sqrt(sp.bind.tx*sp.bind.tx + sp.bind.ty*sp.bind.ty + sp.bind.tz*sp.bind.tz);
            const double centre_r = std::sqrt(cx*cx + cy*cy + cz*cz);
            // Authored around its bone: geometry centred near the pivot while the bone itself sits
            // well away from the mesh origin. Authored around the origin otherwise.
            const bool auto_bone = have_box && bone_r > 2.0 && centre_r < 0.5 * bone_r;
            sp.bone_pivot = (g_cfg.slide_part_pivot < 0) ? auto_bone : (g_cfg.slide_part_pivot == 1);
            wchar_t info[160];
            swprintf_s(info, L"%ls->%ls[box c=(%.1f %.1f %.1f) d=%.0f; bone r=%.1f; %ls]  ", bname.c_str(), mp.first.c_str(), cx, cy, cz, ext, bone_r,
                       sp.bone_pivot ? L"at bone" : L"by delta");
            report += info;
            if (lb.find(L"body") != std::wstring::npos && !have_body) { body_bone = bn; body_bind = sp.bind; have_body = sp.have_bind; }
            ++found_here;
        }
        if (found_here == 0) report += bname + L":none  ";
    }
    // Orphans: shipped parts no bone claimed (the magnum's Shroud is the gun's top). They ride
    // the body bone by delta, which is what a fixed part of the frame does.
    if (g_cfg.slide_part_orphans && have_body) {
        for (size_t i = 0; i < meshes.size() && s_sp_count < 12; ++i) {
            if (claimed[i]) continue;
            auto* part = sp_spawn_part(actor, meshes[i].second, parent, socket, loc, rot, scl);
            if (part == nullptr) continue;
            if (real_mat != nullptr) sp_set_material(part, 0, real_mat);
            SlidePart& sp = s_sp_parts[s_sp_count++];
            sp.comp.set(part); sp.bone = body_bone; sp.bone_name = L"(orphan on body)"; sp.bind = body_bind; sp.have_bind = true; sp.mesh = meshes[i].second; sp.dropped = false;
            sp.is_slide = false; sp.is_mag = false; sp.bone_pivot = false; sp.claimed = false;
            report += L"orphan " + meshes[i].first + L"->body  ";
        }
    }
    if (ref != nullptr) { alignas(16) uint8_t p[64] = {0}; ue_destroy_component(ref); }
    API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: %ls -> stem '%ls', rack bone '%ls', %d part(s) of %d loaded, rest from %s, material %s: %ls", asset.c_str(), stem.c_str(), want_slide.c_str(), s_sp_count, (int)meshes.size(),
                         g_cfg.slide_part_bind == 1 ? "the bind pose" : "the live pose", real_mat ? "the real gun's" : "each part's own", report.c_str());
    if (s_sp_count == 0) return false;
    s_sp_src.set(src);
    for (int i = 0; i < s_sp_count; ++i) if (s_sp_parts[i].is_slide) g_slide_rack_found = true;
    if (g_cfg.slide_part_hide == 2) {
        API::UObject* body_part = nullptr;
        for (int i = 0; i < s_sp_count; ++i) { std::wstring lb = s_sp_parts[i].bone_name; for (auto& ch : lb) ch = (wchar_t)towlower(ch); if (lb.find(L"body") != std::wstring::npos) { body_part = s_sp_parts[i].comp.get(); break; } }
        if (body_part == nullptr && s_sp_count > 0) body_part = s_sp_parts[0].comp.get();
        sp_kids_take(src, body_part);
        sc_set_scale(src, 0.001); s_sp_hidden = true;
    }
    else if (g_cfg.slide_part_hide == 1) { sp_hide_bone(src, true); s_sp_hidden = true; }
    else if (g_cfg.slide_part_hide == 5) {
        XformD slc{}; sp_socket_component(src, sp_slide_bone_name(), &slc);
        sp_hide_bone(src, true); s_sp_hidden = true;
        sp_find_bone_arrays(src, nb, s_sp_slide_index, slc);
    }
    else if (g_cfg.slide_part_hide == 6) { sp_log_morphs(src); }
    sp_log_materials(src);
    if (g_cfg.slide_part_hide == 4) {
        const int slot = (g_cfg.slide_hide_mat_slot >= 0) ? g_cfg.slide_hide_mat_slot : (g_cfg.slide_hide_section >= 0 ? g_cfg.slide_hide_section : 0);
        auto* mat = sp_find_material(trim_cfg(g_cfg.slide_hide_mat));
        if (mat == nullptr) API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: no material matching '%s' loaded; slot %d left as is", trim_cfg(g_cfg.slide_hide_mat).c_str(), slot);
        else {
            s_sp_mat_orig.set(sp_get_material(src, slot)); s_sp_mat_slot = slot;
            sp_set_material(src, slot, mat);
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: slot %d material -> %ls", slot, mat->get_full_name().c_str());
        }
    }
    // Is the arms rig's asset Nanite too? The arms bone-hide works; if the arms are not Nanite,
    // that is the difference, not the call.
    if (auto* rig = rig_tracked_component()) {
        for (const wchar_t* nm : { L"SkinnedAsset", L"SkeletalMesh" }) {
            auto** pm = rig->get_property_data<API::UObject*>(nm);
            if (pm == nullptr || IsBadReadPtr(pm, sizeof(void*)) || *pm == nullptr) continue;
            int nan = -1;
            if (auto* pr = (*pm)->get_class()->find_property(L"NaniteSettings")) { const uint8_t* q = reinterpret_cast<const uint8_t*>(*pm) + pr->get_offset(); if (!IsBadReadPtr(q, 1)) nan = q[0] & 1; }
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: arms asset %ls NaniteSettings.bEnabled=%d", (*pm)->get_fname() ? (*pm)->get_fname()->to_string().c_str() : L"?", nan);
            break;
        }
    }
    return true;
}
// A physics box proxy for a dropped mesh component that has no collision of its own: an
// invisible BoxComponent from the mesh's world bounds simulates, the mesh rides it.
API::UObject* sp_drop_proxy_for(API::UObject* m) {
    double origin[3] = {0, 0, 0}, extent[3] = {2, 2, 4};
    if (auto* fn = sc_fn(m, L"K2_GetComponentBounds")) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        auto* po = fn->find_property(L"Origin"); auto* pe = fn->find_property(L"BoxExtent");
        if (po && pe) { fn->call(m, p); memcpy(origin, p + po->get_offset(), 24); memcpy(extent, p + pe->get_offset(), 24); }
    }
    auto* bcls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.BoxComponent");
    auto* actor = fp_weapon_actor();
    auto* box = (bcls && actor) ? API::get()->add_component_by_class(actor, bcls, false) : nullptr;
    if (box == nullptr) return nullptr;
    { alignas(16) uint8_t p[64] = {0}; p[0] = 1; p[1] = 1; p[2] = 1; p[3] = 0; box->call_function(L"K2_DetachFromComponent", p); }
    { alignas(16) uint8_t p[64] = {0}; auto* d = reinterpret_cast<double*>(p); d[0] = std::fmax(1.0, extent[0]); d[1] = std::fmax(1.0, extent[1]); d[2] = std::fmax(1.0, extent[2]); p[24] = 1; box->call_function(L"SetBoxExtent", p); }
    XformD cw{};
    if (sc_component_world(m, &cw)) {
        XformD bw = cw; bw.tx = origin[0]; bw.ty = origin[1]; bw.tz = origin[2]; bw.sx = bw.sy = bw.sz = 1.0;
        if (auto* fn = sc_fn(box, L"K2_SetWorldTransform")) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const bool no = false, yes = true;
            sm_put(fn, p, sizeof(p), L"NewTransform", &bw, sizeof(XformD), &ok);
            sm_put(fn, p, sizeof(p), L"bSweep", &no, 1, &ok);
            sm_put(fn, p, sizeof(p), L"bTeleport", &yes, 1, &ok);
            if (ok) fn->call(box, p);
        }
    }
    { alignas(16) uint8_t p[64] = {0}; p[0] = 1; box->call_function(L"SetHiddenInGame", p); }
    sp_attach(m, box, make_fname(L"None"), 1);   // KeepWorld
    { alignas(16) uint8_t p[64] = {0}; p[0] = 0; m->call_function(L"SetCollisionEnabled", p); }
    { alignas(16) uint8_t p[64] = {0}; API::FName nm = make_fname(L"PhysicsActor"); memcpy(p, &nm, sizeof(int32_t) * 2); p[8] = 1; box->call_function(L"SetCollisionProfileName", p); }
    { alignas(16) uint8_t p[64] = {0}; p[0] = 3; box->call_function(L"SetCollisionEnabled", p); }
    { alignas(16) uint8_t p[64] = {0}; p[0] = 1; box->call_function(L"SetSimulatePhysics", p); }
    return box;
}
// ---- NATIVE PARTS (mode 7). The game's own rack component, found among the real mesh's
// children by the mesh it renders; its socket keeps every game animation, our offset rides on it.
struct NpPart { TrackedObject comp; double orig[3]; double orig_rot[3]; API::FName bone; };
NpPart s_np_parts[4]; int s_np_count = 0; bool s_np_rot = false;
TrackedObject s_np_slide, s_np_src; API::FName s_np_bone; std::wstring s_np_bone_w; std::wstring s_np_stem; double s_np_orig[3] = {0, 0, 0}; bool s_np_have = false;
bool s_np_quiet = false;   // slide_part_tick: the listing line is off after the second failed try
bool np_spawn(API::UObject* src) {
    s_np_have = false; s_np_slide = TrackedObject{};
    API::UObject* mesh = nullptr;
    for (const wchar_t* nm : { L"SkinnedAsset", L"SkeletalMesh" }) {
        auto** pm = src->get_property_data<API::UObject*>(nm);
        if (pm != nullptr && !IsBadReadPtr(pm, sizeof(void*)) && *pm != nullptr) { mesh = *pm; break; }
    }
    if (mesh == nullptr || mesh->get_fname() == nullptr) return false;
    std::wstring stem = mesh->get_fname()->to_string();
    if (stem.rfind(L"SK_", 0) == 0) stem = stem.substr(3);
    stem = weapon_stem_from_mesh(stem);
    // The spec: "Bone" | "BoneA+BoneB" | "Bone@rot" | "Bone@back=0.03". Several parts move
    // together; @rot hinges; @back pushes this weapon's grab zone back along the barrel (metres).
    std::wstring spec = sp_rack_bone_for(stem);
    s_np_rot = false;
    g_sl_zone_back.store(0.0f, std::memory_order_relaxed); g_sl_zone_up.store(0.0f, std::memory_order_relaxed); g_sl_zone_right.store(0.0f, std::memory_order_relaxed); g_sl_zone_reload_only.store(false, std::memory_order_relaxed); g_sl_zone_every_shot.store(false, std::memory_order_relaxed); g_sl_zone_pump.store(false, std::memory_order_relaxed); g_sl_zone_pull_down.store(false, std::memory_order_relaxed); g_sl_insert.store(-1.0f, std::memory_order_relaxed);
    {
        size_t at = spec.find(L'@');
        std::wstring opts = (at != std::wstring::npos) ? spec.substr(at + 1) : L"";
        if (at != std::wstring::npos) spec = spec.substr(0, at);
        size_t pos = 0;
        while (pos < opts.size()) {
            size_t nx = opts.find(L'@', pos); if (nx == std::wstring::npos) nx = opts.size();
            std::wstring o = opts.substr(pos, nx - pos);
            if (o == L"rot") s_np_rot = true;
            else if (o == L"reloadonly") g_sl_zone_reload_only.store(true, std::memory_order_relaxed);
            else if (o == L"everyshot") g_sl_zone_every_shot.store(true, std::memory_order_relaxed);
            else if (o == L"pump") g_sl_zone_pump.store(true, std::memory_order_relaxed);
            else if (o == L"pulldown") g_sl_zone_pull_down.store(true, std::memory_order_relaxed);
            else if (o.rfind(L"insert=", 0) == 0) g_sl_insert.store((float)_wtof(o.c_str() + 7), std::memory_order_relaxed);
            else if (o.rfind(L"back=", 0) == 0) g_sl_zone_back.store((float)_wtof(o.c_str() + 5), std::memory_order_relaxed);
            else if (o.rfind(L"up=", 0) == 0) g_sl_zone_up.store((float)_wtof(o.c_str() + 3), std::memory_order_relaxed);
            else if (o.rfind(L"right=", 0) == 0) g_sl_zone_right.store((float)_wtof(o.c_str() + 6), std::memory_order_relaxed);
            pos = nx + 1;
        }
    }
    std::vector<std::wstring> bones;
    { size_t pos = 0; while (pos <= spec.size()) { size_t plus = spec.find(L'+', pos); if (plus == std::wstring::npos) plus = spec.size(); if (plus > pos) bones.push_back(spec.substr(pos, plus - pos)); if (plus >= spec.size()) break; pos = plus + 1; } }
    const std::wstring bone = bones.empty() ? spec : bones[0];
    std::wstring listing;
    for (int i = 0; i < 4; ++i) s_np_parts[i] = NpPart{};
    s_np_count = 0;
    API::UObject* hit = nullptr; std::wstring hitmesh;
    weapon_components([&](API::UObject* c) {
        auto** pp = c->get_property_data<API::UObject*>(L"AttachParent");
        if (pp == nullptr || IsBadReadPtr(pp, sizeof(void*)) || *pp != src) return true;
        auto** pm = c->get_property_data<API::UObject*>(L"StaticMesh");
        if (pm == nullptr || IsBadReadPtr(pm, sizeof(void*)) || *pm == nullptr || (*pm)->get_fname() == nullptr) return true;
        std::wstring mn = (*pm)->get_fname()->to_string(); std::wstring lo = mn; for (auto& ch : lo) ch = (wchar_t)towlower(ch);
        listing += mn + L"  ";
        for (const std::wstring& b : bones) {
            std::wstring bs = b;
            if (bs.size() > 2 && (bs.compare(bs.size() - 2, 2, L"_M") == 0 || bs.compare(bs.size() - 2, 2, L"_L") == 0 || bs.compare(bs.size() - 2, 2, L"_R") == 0)) bs = bs.substr(0, bs.size() - 2);
            std::wstring w1 = L"_" + bs + L"_", w2 = L"_" + b + L"_";
            for (auto& ch : w1) ch = (wchar_t)towlower(ch);
            for (auto& ch : w2) ch = (wchar_t)towlower(ch);
            if ((lo.find(w1) != std::wstring::npos || lo.find(w2) != std::wstring::npos) && s_np_count < 4) {
                NpPart& np = s_np_parts[s_np_count++];
                np.comp.set(c); np.bone = make_fname(b.c_str());
                if (auto* pl = c->get_property_data<double>(L"RelativeLocation")) if (!IsBadReadPtr(pl, 24)) { np.orig[0] = pl[0]; np.orig[1] = pl[1]; np.orig[2] = pl[2]; }
                if (auto* pr = c->get_property_data<double>(L"RelativeRotation")) if (!IsBadReadPtr(pr, 24)) { np.orig_rot[0] = pr[0]; np.orig_rot[1] = pr[1]; np.orig_rot[2] = pr[2]; }
                if (hit == nullptr) { hit = c; hitmesh = mn; }
                break;
            }
        }
        return true;
    });
    if (!s_np_quiet || hit != nullptr) API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART native: %ls parts on the real mesh: %ls-> rack '%ls' = %ls (%d part(s)%s)", stem.c_str(), listing.c_str(), spec.c_str(), hit ? hitmesh.c_str() : L"NOT FOUND", s_np_count, s_np_rot ? ", hinge" : "");
    if (hit == nullptr) return false;
    s_np_orig[0] = s_np_parts[0].orig[0]; s_np_orig[1] = s_np_parts[0].orig[1]; s_np_orig[2] = s_np_parts[0].orig[2];
    s_np_slide.set(hit); s_np_src.set(src); s_np_bone = make_fname(bone.c_str()); s_np_bone_w = bone; s_np_stem = stem; s_np_have = true;
    g_slide_rack_found = true;
    return true;
}
void np_set_rel(API::UObject* c, const double loc[3], const double rot[3]) {
    if (auto* fn = sc_fn(c, L"K2_SetRelativeLocationAndRotation")) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const bool no = false, yes = true;
        sm_put(fn, p, sizeof(p), L"NewLocation", loc, 24, &ok);
        sm_put(fn, p, sizeof(p), L"NewRotation", rot, 24, &ok);
        sm_put(fn, p, sizeof(p), L"bSweep", &no, 1, &ok);
        sm_put(fn, p, sizeof(p), L"bTeleport", &yes, 1, &ok);
        if (ok) fn->call(c, p);
    }
}
void np_teardown() {
    g_sl_zone_back.store(0.0f, std::memory_order_relaxed); g_sl_zone_up.store(0.0f, std::memory_order_relaxed); g_sl_zone_right.store(0.0f, std::memory_order_relaxed); g_sl_zone_reload_only.store(false, std::memory_order_relaxed); g_sl_zone_every_shot.store(false, std::memory_order_relaxed); g_sl_zone_pump.store(false, std::memory_order_relaxed); g_sl_zone_pull_down.store(false, std::memory_order_relaxed); g_sl_insert.store(-1.0f, std::memory_order_relaxed); g_slide_zone_hot.store(false, std::memory_order_relaxed);
    for (int i = 0; i < s_np_count; ++i) { if (auto* c = s_np_parts[i].comp.get()) np_set_rel(c, s_np_parts[i].orig, s_np_parts[i].orig_rot); s_np_parts[i] = NpPart{}; }
    s_np_count = 0;
    s_np_slide = TrackedObject{}; s_np_src = TrackedObject{}; s_np_have = false;
    g_slide_rack_found = false;
    g_sl_part_valid.store(false, std::memory_order_relaxed);
}
void np_tick() {
    auto* c = s_np_slide.get();
    if (c == nullptr) return;
    const float pull_cm = g_slide_pull.load(std::memory_order_relaxed) * 304.8f;
    const double along = -(double)(pull_cm * g_cfg.slide_part_sign) - (double)g_cfg.slide_part_test;
    double gun_axis[3] = {0, 0, 0}; gun_axis[g_cfg.slide_part_axis] = along;   // in the gun mesh's frame
    for (int i = 0; i < s_np_count; ++i) {
        auto* pc = s_np_parts[i].comp.get();
        if (pc == nullptr) continue;
        double loc[3] = {s_np_parts[i].orig[0], s_np_parts[i].orig[1], s_np_parts[i].orig[2]};
        double rot[3] = {s_np_parts[i].orig_rot[0], s_np_parts[i].orig_rot[1], s_np_parts[i].orig_rot[2]};
        if (s_np_rot) {
            // A HINGE (the rocket launcher's clamp): on the reload press it swings OPEN and the
            // lock holds it there after the seat; the rack closes it (pull fraction 0 -> 1 = open
            // -> closed). Outside a reload the pull just swings it by degrees per cm.
            const float frac = (g_cfg.slide_travel > 0.0f) ? std::fmin(1.0f, std::fmax(0.0f, g_slide_pull.load(std::memory_order_relaxed) / g_cfg.slide_travel)) : 0.0f;
            const bool reloading = (s_reload != ReloadState::Idle) || s_sl_lock_pending;
            rot[g_cfg.slide_part_rot_axis] += reloading ? (double)g_cfg.slide_part_open_deg * (1.0 - (double)frac)
                                                        : (double)(pull_cm + g_cfg.slide_part_test) * (double)g_cfg.slide_part_rot_deg;
        } else {
            double ax[3] = {gun_axis[0], gun_axis[1], gun_axis[2]};
            if (g_cfg.slide_part_frame == 1) {
                // The gun-frame axis into THIS part's own socket frame (the spiker's pin socket is
                // not the hammer's: converted through the hammer's it went down, not back).
                XformD st{};
                if (auto* src = s_np_src.get()) {
                    if (sp_socket_component(src, s_np_parts[i].bone, &st)) {
                        const QD q{st.qx, st.qy, st.qz, st.qw};
                        double l[3]; qd_rot(qd_conj(q), ax, l);
                        ax[0] = l[0]; ax[1] = l[1]; ax[2] = l[2];
                    }
                }
            }
            loc[0] += ax[0]; loc[1] += ax[1]; loc[2] += ax[2];
        }
        np_set_rel(pc, loc, rot);
    }
    // The grab zone: three reference points (Config.hpp slide_zone), all three logged.
    {
        // By-name calls (the class find_function path came back empty on the shotgun and the
        // launcher, 2026-09-06): Origin is K2_GetComponentBounds' first out-param, at offset 0.
        Vec3 bounds{}, sock{}, comp{}; bool hb = false, hs = false, hc = false;
        const char* how = "";
        hb = part_world_centre(c, &bounds, &how);
        if (auto* src = s_np_src.get()) hs = call_socket_location(src, s_np_bone_w.c_str(), &sock);
        hc = call_ret_vec3(c, L"K2_GetComponentLocation", &comp);
        Vec3 use{}; bool have = false;
        if (g_cfg.slide_zone == 2 && hs) { use = sock; have = true; }
        else if (g_cfg.slide_zone == 3 && hc) { use = comp; have = true; }
        else if (hb) { use = bounds; have = true; }
        if (have) { g_sl_part_x.store(use.x, std::memory_order_relaxed); g_sl_part_y.store(use.y, std::memory_order_relaxed); g_sl_part_z.store(use.z, std::memory_order_relaxed); }
        g_sl_part_valid.store(have, std::memory_order_relaxed);
        if (g_cfg.slide_log) {
            static uint32_t s_n = 0;
            if ((s_n++ % 30u) == 0u) {
                Vec3 gun{}; const bool hg = (s_np_src.get() != nullptr) && call_ret_vec3(s_np_src.get(), L"K2_GetComponentLocation", &gun);
                API::get()->log_info("[Halo-CampE-UEVR] SLIDEZONE %ls/%ls refs (cm, world): bounds[%s] %s(%.0f %.0f %.0f) socket %s(%.0f %.0f %.0f) comp %s(%.0f %.0f %.0f) gun %s(%.0f %.0f %.0f) using mode %d",
                                     s_np_stem.c_str(), s_np_bone_w.c_str(), how, hb ? "" : "-", bounds.x, bounds.y, bounds.z, hs ? "" : "-", sock.x, sock.y, sock.z, hc ? "" : "-", comp.x, comp.y, comp.z, hg ? "" : "-", gun.x, gun.y, gun.z, g_cfg.slide_zone);
            }
        }
    }
}
// THE MAGAZINE PART DROPS. Called on the reload press: the gun's own mag part detaches (keep
// world), takes a physics profile and falls. Returns true when it handled the drop, so the
// copy drop (of the game's hidden magazine mesh) stands down. A fresh part comes on the seat.
bool slide_parts_mag_drop() {
    if (!g_cfg.slide_part_magdrop || s_sp_count == 0) return false;
    for (int i = 0; i < s_sp_count; ++i) {
        SlidePart& sp = s_sp_parts[i];
        auto* m = sp.comp.get();
        if (!sp.is_mag || m == nullptr) continue;
        if (auto* old = s_sp_dropped.get()) ue_destroy_component(old);
        if (auto* old = s_sp_dropped_proxy.get()) ue_destroy_component(old);
        s_sp_dropped = TrackedObject{}; s_sp_dropped_proxy = TrackedObject{};
        // The asset's collision shapes, as a fact: UStaticMesh::BodySetup -> AggGeom counts.
        int nsph = -1, nbox = -1, ncap = -1, ncvx = -1;
        if (sp.mesh != nullptr) {
            if (auto** pbs = sp.mesh->get_property_data<API::UObject*>(L"BodySetup")) {
                if (!IsBadReadPtr(pbs, sizeof(void*)) && *pbs != nullptr) {
                    auto* bs = *pbs;
                    if (auto* pag = bs->get_class()->find_property(L"AggGeom")) {
                        const uint8_t* ag = reinterpret_cast<const uint8_t*>(bs) + pag->get_offset();
                        // FKAggregateGeom: TArray SphereElems, BoxElems, SphylElems, ConvexElems, ... (num at +8 of each 16-byte header)
                        if (!IsBadReadPtr(ag, 64)) { nsph = *reinterpret_cast<const int32_t*>(ag + 8); nbox = *reinterpret_cast<const int32_t*>(ag + 24); ncap = *reinterpret_cast<const int32_t*>(ag + 40); ncvx = *reinterpret_cast<const int32_t*>(ag + 56); }
                    }
                }
            }
        }
        const int mode = g_cfg.slide_part_dropmode;
        { alignas(16) uint8_t p[64] = {0}; p[0] = 1; p[1] = 1; p[2] = 1; p[3] = 0; m->call_function(L"K2_DetachFromComponent", p); }
        API::UObject* sim = m;   // the component that simulates
        if (mode == 1) {
            // A box proxy from the part's world bounds; the part rides it.
            XformD cw{}; double origin[3] = {0, 0, 0}, extent[3] = {2, 2, 4};
            if (auto* fn = sc_fn(m, L"K2_GetComponentBounds")) {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                auto* po = fn->find_property(L"Origin"); auto* pe = fn->find_property(L"BoxExtent");
                if (po && pe) { fn->call(m, p); memcpy(origin, p + po->get_offset(), 24); memcpy(extent, p + pe->get_offset(), 24); }
            }
            auto* bcls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.BoxComponent");
            auto* actor = fp_weapon_actor();
            auto* box = (bcls && actor) ? API::get()->add_component_by_class(actor, bcls, false) : nullptr;
            if (box != nullptr) {
                { alignas(16) uint8_t p[64] = {0}; p[0] = 1; p[1] = 1; p[2] = 1; p[3] = 0; box->call_function(L"K2_DetachFromComponent", p); }
                { alignas(16) uint8_t p[64] = {0}; auto* d = reinterpret_cast<double*>(p); d[0] = std::fmax(1.0, extent[0]); d[1] = std::fmax(1.0, extent[1]); d[2] = std::fmax(1.0, extent[2]); p[24] = 1; box->call_function(L"SetBoxExtent", p); }
                if (sc_component_world(m, &cw)) {
                    XformD bw = cw; bw.tx = origin[0]; bw.ty = origin[1]; bw.tz = origin[2]; bw.sx = bw.sy = bw.sz = 1.0;
                    if (auto* fn = sc_fn(box, L"K2_SetWorldTransform")) {
                        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const bool no = false, yes = true;
                        sm_put(fn, p, sizeof(p), L"NewTransform", &bw, sizeof(XformD), &ok);
                        sm_put(fn, p, sizeof(p), L"bSweep", &no, 1, &ok);
                        sm_put(fn, p, sizeof(p), L"bTeleport", &yes, 1, &ok);
                        if (ok) fn->call(box, p);
                    }
                }
                { alignas(16) uint8_t p[64] = {0}; box->call_function(L"SetHiddenInGame", p); p[0] = 1; box->call_function(L"SetHiddenInGame", p); }
                // The part rides the box, keeping its world placement.
                if (auto* fn = sc_fn(m, L"K2_AttachToComponent")) {
                    alignas(16) uint8_t p[128] = {0}; bool ok = true; const uint8_t keep = 1; const bool weld = false; API::FName none = make_fname(L"None");
                    sm_put(fn, p, sizeof(p), L"Parent", &box, sizeof(void*), &ok);
                    sm_put(fn, p, sizeof(p), L"SocketName", &none, sizeof(int32_t) * 2, &ok);
                    sm_put(fn, p, sizeof(p), L"LocationRule", &keep, 1, &ok);
                    sm_put(fn, p, sizeof(p), L"RotationRule", &keep, 1, &ok);
                    sm_put(fn, p, sizeof(p), L"ScaleRule", &keep, 1, &ok);
                    sm_put(fn, p, sizeof(p), L"bWeldSimulatedBodies", &weld, 1, &ok);
                    if (ok) fn->call(m, p);
                }
                { alignas(16) uint8_t p[64] = {0}; p[0] = 0; m->call_function(L"SetCollisionEnabled", p); }
                sim = box; s_sp_dropped_proxy.set(box);
            }
        }
        { alignas(16) uint8_t p[64] = {0}; API::FName nm = make_fname(L"PhysicsActor"); memcpy(p, &nm, sizeof(int32_t) * 2); p[8] = 1; sim->call_function(L"SetCollisionProfileName", p); }
        if (mode == 2) {
            { alignas(16) uint8_t p[64] = {0}; p[0] = 5; sim->call_function(L"SetCollisionObjectType", p); }   // ECC_PhysicsBody
            { alignas(16) uint8_t p[64] = {0}; p[0] = 2; sim->call_function(L"SetCollisionResponseToAllChannels", p); }   // ECR_Block
        }
        { alignas(16) uint8_t p[64] = {0}; p[0] = 3; sim->call_function(L"SetCollisionEnabled", p); }
        { alignas(16) uint8_t p[64] = {0}; p[0] = 1; sim->call_function(L"SetSimulatePhysics", p); }
        int simulating = -1;
        if (auto* fn = sc_fn(sim, L"IsSimulatingPhysics")) {
            alignas(16) uint8_t p[64] = {0}; auto* r = fn->find_property(L"ReturnValue");
            if (r) { fn->call(sim, p); simulating = p[r->get_offset()] ? 1 : 0; }
        }
        s_sp_dropped.set(m); s_sp_dropped_at = now_ticks();
        sp.comp = TrackedObject{}; sp.dropped = true;
        if (g_cfg.reload_log || g_cfg.slide_log)
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: magazine part dropped (mode %d, %s simulating=%d; asset collision: %d spheres %d boxes %d capsules %d convex)",
                                 mode, sim == m ? "the part" : "a box proxy", simulating, nsph, nbox, ncap, ncvx);
        return true;
    }
    return false;
}
void slide_part_tick() {
    auto* src = reload_weapon_default_comp();
    auto* actor = fp_weapon_actor();
    const bool usable = g_cfg.slide_part && g_cfg.slide_vr && src != nullptr && actor != nullptr && slide_weapon_ok();
    const std::string key = weapon_key();
    static int s_bind_mode = -1; static int s_shadow_mode = -1; static int s_pivot_mode = -2; static int s_mat_mode = -1; static int s_orphan_mode = -1; static int s_kids_mode = -1; static int s_ui_mode = -1;
    const bool mode_changed = (g_cfg.slide_part_hide != s_sp_mode) || (g_cfg.slide_part_bind != s_bind_mode) || ((int)g_cfg.slide_part_shadow != s_shadow_mode) || (g_cfg.slide_part_pivot != s_pivot_mode)
                           || ((int)g_cfg.slide_part_mat != s_mat_mode) || ((int)g_cfg.slide_part_orphans != s_orphan_mode)
                           || (g_cfg.slide_part_kids != s_kids_mode) || ((int)g_cfg.slide_part_ui != s_ui_mode);
    static std::string s_np_key; static API::UObject* s_np_src = nullptr; static long long s_np_retry = 0;
    if (g_cfg.slide_part_hide == 7 || s_np_have) {
        if (!usable || g_cfg.slide_part_hide != 7 || (s_np_have && (key != s_np_key || s_np_src != src))) {
            if (s_np_have) np_teardown();
            if (s_sp_count > 0) sp_teardown("native mode");
            if (!usable || g_cfg.slide_part_hide != 7) { if (!usable) return; }
        }
        if (g_cfg.slide_part_hide == 7) {
            if (!s_np_have) {
                static std::string s_np_fail_key; static int s_np_fails = 0;
                // A fresh weapon actor attaches its parts some frames after the swap (logged: only a
                // light cone at the swap tick, the rack found on the 1.5 s retry). The seat's lock-back
                // and the phantom both require the part to be found, so a seat inside that window
                // skipped the rack of a gun that came back empty. Retry fast first.
                if (key != s_np_fail_key) { s_np_fail_key = key; s_np_fails = 0; s_np_retry = 0; }
                if (now_ticks() - s_np_retry < ms_to_ticks(s_np_fails < 8 ? 150 : (s_np_fails < 11 ? 1500 : 15000))) return;
                s_np_retry = now_ticks();
                s_np_quiet = s_np_fails >= 2;
                if (!np_spawn(src)) { ++s_np_fails; return; }
                s_np_fails = 0;
                s_np_key = key; s_np_src = src; s_sp_mode = 7;
            }
            np_tick();
            return;
        }
    }
    if (!usable || (s_sp_count > 0 && (key != s_sp_key || s_sp_src.get() != src || mode_changed))) {
        sp_teardown(!usable ? "unavailable" : (mode_changed ? "mode changed" : "weapon changed"));
        if (!usable) return;
    }
    if (s_sp_count == 0) {
        if (now_ticks() - s_sp_retry_at < ms_to_ticks(1500)) return;
        s_sp_retry_at = now_ticks();
        if (!sp_spawn(actor, src)) return;
        s_sp_key = key; s_sp_mode = g_cfg.slide_part_hide; s_bind_mode = g_cfg.slide_part_bind; s_shadow_mode = (int)g_cfg.slide_part_shadow; s_pivot_mode = g_cfg.slide_part_pivot;
        s_mat_mode = (int)g_cfg.slide_part_mat; s_orphan_mode = (int)g_cfg.slide_part_orphans; s_kids_mode = g_cfg.slide_part_kids; s_ui_mode = (int)g_cfg.slide_part_ui;
    }
    ++s_sp_ticks;
    // The dropped magazine part is removed after mag_drop_ms; a dropped slot re-spawns on the seat.
    if (s_sp_dropped.get() != nullptr && now_ticks() - s_sp_dropped_at > ms_to_ticks(g_cfg.mag_drop_ms)) {
        ue_destroy_component(s_sp_dropped.get()); s_sp_dropped = TrackedObject{};
        if (auto* px = s_sp_dropped_proxy.get()) ue_destroy_component(px);
        s_sp_dropped_proxy = TrackedObject{};
    }
    if (s_reload == ReloadState::Idle) {
        for (int i = 0; i < s_sp_count; ++i) {
            SlidePart& sp = s_sp_parts[i];
            if (!sp.dropped || sp.mesh == nullptr) continue;
            API::UObject* parent = nullptr; API::FName socket{}; double loc[3] = {0}, rot[3] = {0}, scl[3] = {1, 1, 1};
            if (!sc_read_attach(src, &parent, &socket, loc, rot, scl)) break;
            auto* part = sp_spawn_part(actor, sp.mesh, parent, socket, loc, rot, scl);
            if (part == nullptr) break;
            if (g_cfg.slide_part_mat) { if (auto* rm = sp_get_material(src, 0)) sp_set_material(part, 0, rm); }
            sp.comp.set(part); sp.dropped = false;
            if (g_cfg.reload_log || g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: magazine part re-spawned on the seat");
        }
    }
    if (s_sp_hidden) {
        if (g_cfg.slide_part_hide == 2) sc_set_scale(src, 0.001);
        else if (g_cfg.slide_part_hide == 1) {
            sp_hide_bone(src, true);
            if (g_cfg.slide_log && (s_sp_ticks % 90u) == 5u)
                API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: engine says %s hidden = %d", trim_cfg(g_cfg.slide_part_bone).c_str(), sp_is_bone_hidden(src));
        }
    }
    if (g_cfg.slide_part_hide == 5) { sp_hide_bone(src, true); sp_park_far_tick(src); }
    if (g_cfg.slide_part_hide == 6) sp_morph_tick(src);
    // Section hide (mode 3): the live-stepped material index; the previous one is shown again.
    if (g_cfg.slide_part_hide == 3) {
        const int want = g_cfg.slide_hide_section;
        if (want != s_sp_section_hidden) {
            if (s_sp_section_hidden >= 0) sp_show_section(src, s_sp_section_hidden, true);
            if (want >= 0) sp_show_section(src, want, false);
            s_sp_section_hidden = want;
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: material section %d hidden (previous shown)", want);
        }
    }
    const float pull_cm = g_slide_pull.load(std::memory_order_relaxed) * 304.8f;
    const double along = -(double)(pull_cm * g_cfg.slide_part_sign) - (double)g_cfg.slide_part_test;
    const bool mag_out = (s_reload != ReloadState::Idle);
    for (int i = 0; i < s_sp_count; ++i) {
        SlidePart& sp = s_sp_parts[i];
        auto* part = sp.comp.get();
        if (part == nullptr) continue;
        XformD now{};
        if (!sp_socket_component(src, sp.bone, &now)) continue;
        // D = now x inverse(bind), in the mesh's frame: rotation qn*conj(qb), translation tn - D*tb.
        XformD d = now;
        if (sp.bone_pivot) {
            // Authored around its bone: the part IS the bone. now already is the bone in the mesh frame.
        } else if (sp.have_bind) {
            const QD qn{now.qx, now.qy, now.qz, now.qw}, qb{sp.bind.qx, sp.bind.qy, sp.bind.qz, sp.bind.qw};
            const QD qd = qd_mul(qn, qd_conj(qb));
            const double tb[3] = {sp.bind.tx, sp.bind.ty, sp.bind.tz}; double tbr[3]; qd_rot(qd, tb, tbr);
            d.qx = qd.x; d.qy = qd.y; d.qz = qd.z; d.qw = qd.w;
            d.tx = now.tx - tbr[0]; d.ty = now.ty - tbr[1]; d.tz = now.tz - tbr[2];
        } else {
            d.qx = 0; d.qy = 0; d.qz = 0; d.qw = 1; d.tx = 0; d.ty = 0; d.tz = 0;
        }
        d.sx = 1.0; d.sy = 1.0; d.sz = 1.0;
        if (sp.is_slide) {
            double ax[3] = {0, 0, 0}; ax[g_cfg.slide_part_axis] = 1.0;
            d.tx += ax[0] * along; d.ty += ax[1] * along; d.tz += ax[2] * along;
        }
        if (auto* fn = sc_fn(part, L"K2_SetRelativeTransform")) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const bool no = false, yes = true;
            sm_put(fn, p, sizeof(p), L"NewTransform", &d, sizeof(XformD), &ok);
            sm_put(fn, p, sizeof(p), L"bSweep", &no, 1, &ok);
            sm_put(fn, p, sizeof(p), L"bTeleport", &yes, 1, &ok);
            if (ok) fn->call(part, p);
        }
        if (sp.is_mag) sc_set_visibility(part, !(mag_out && g_cfg.slide_part_maghide));
        if (sp.is_slide) {
            bool published = false;
            if (auto* fn = sc_fn(part, L"K2_GetComponentBounds")) {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                auto* po = fn->find_property(L"Origin");
                if (po != nullptr) {
                    fn->call(part, p);
                    const double* o = reinterpret_cast<const double*>(p + po->get_offset());
                    g_sl_part_x.store((float)o[0], std::memory_order_relaxed);
                    g_sl_part_y.store((float)o[1], std::memory_order_relaxed);
                    g_sl_part_z.store((float)o[2], std::memory_order_relaxed);
                    published = true;
                }
            }
            g_sl_part_valid.store(published, std::memory_order_relaxed);
        }
        if (g_cfg.slide_log && sp.is_slide && (s_sp_ticks % 90u) == 1u)
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: %ls now (%.2f %.2f %.2f) bind (%.2f %.2f %.2f) -> rel (%.2f %.2f %.2f) along %.2f",
                                 sp.bone_name.c_str(), now.tx, now.ty, now.tz, sp.bind.tx, sp.bind.ty, sp.bind.tz, d.tx, d.ty, d.tz, along);
    }
}

// ---- WPNAMMODUMP (dev): which field of the weapon object is ROUNDS LOADED? Every tick the
// first 0x400 bytes of the resolved weapon object are compared to the previous tick as 16-bit
// words; a word that dropped by exactly one is logged with its offset. Firing a few rounds
// names the counter (and possibly a mirror or two); a game reload shows it jump back up.
// The rack's live-round eject is one write to that field.
void wpn_ammo_dump_tick() {
    if (!g_cfg.wpn_ammo_dump) return;
    // SMALL INTEGERS, AT THE SHOT. The first pass flagged 16246 -> 16245: the high halves of
    // floats ticking, not a round count. A magazine counter is a small number that drops by one
    // within a few ticks of the trigger, so: values under 2000, and only while the player's
    // fire stamp is fresh. Span widened past the node block (+0x344 on the pistol).
    constexpr int SPAN = 0x800;
    static uint16_t  s_prev[SPAN / 2];
    static uintptr_t s_obj = 0;
    static bool      s_have = false;
    static int       s_lines = 0;
    const uintptr_t obj = g_wpn_obj_ptr.load(std::memory_order_relaxed);
    if (obj == 0 || IsBadReadPtr((const void*)obj, SPAN)) { s_have = false; return; }
    if (obj != s_obj) { s_obj = obj; s_have = false; }
    const uint16_t* cur = reinterpret_cast<const uint16_t*>(obj);
    const long long since_fire = now_ticks() - g_ft_fire_at.load(std::memory_order_relaxed);
    const bool fresh = since_fire >= 0 && since_fire < ms_to_ticks(150);
    if (s_have) {
        for (int i = 0; i < SPAN / 2 && s_lines < 120; ++i) {
            if (fresh && s_prev[i] < 2000 && cur[i] + 1 == s_prev[i]) {
                ++s_lines;
                API::get()->log_info("[Halo-CampE-UEVR] WPNAMMO: +0x%03X u16 %u -> %u (at the shot)", (unsigned)(i * 2), s_prev[i], cur[i]);
            }
            // A RELOAD refills: the loaded counter jumps UP by several; the total does not move.
            if (s_prev[i] < 2000 && cur[i] < 2000 && cur[i] >= s_prev[i] + 2 && cur[i] - s_prev[i] <= 200) {
                ++s_lines;
                API::get()->log_info("[Halo-CampE-UEVR] WPNAMMO: +0x%03X u16 %u -> %u (jump up)", (unsigned)(i * 2), s_prev[i], cur[i]);
            }
        }
    }
    // NAMED CANDIDATES, ANY CHANGE, ANY TIME. +0x2BE went 12 -> 11 at a shot (the CE pistol
    // holds 12) and the +1 of a topped-up reload never clears the jump filter above; the two
    // counters named before it (+0x236, +0x29E) count DOWN every tick, not per shot.
    if (s_have) {
        static const int kWatch[] = {0x2BC, 0x2BE, 0x2C0, 0x2C8, 0x2CA, 0x2CC};
        static int s_wl = 0;
        for (int off : kWatch) {
            const int i = off / 2;
            if (cur[i] != s_prev[i] && s_wl < 300) {
                ++s_wl;
                API::get()->log_info("[Halo-CampE-UEVR] WPNAMMO watch +0x%03X: %u -> %u", (unsigned)off, s_prev[i], cur[i]);
            }
        }
    }
    memcpy(s_prev, cur, sizeof(s_prev));
    s_have = true;
}

// ---- THE DROPPED MAGAZINE (magdrop). On the reload press the gun's own magazine is hidden;
// a copy of its mesh is spawned in its place, detached from the weapon, given collision and
// physics, and falls. Removed after magdrop_ms. Pure presentation: the game's reload is
// untouched. Sequence matters: place, detach (keep world), collision profile, collision on,
// simulate -- a component that simulates while attached fights its parent every frame.
TrackedObject s_magdrop, s_magdrop_proxy;
long long     s_magdrop_at = 0;
API::UObject* sp_drop_proxy_for(API::UObject* m);   // defined with the parts code below
void mag_drop_spawn() {
    if (!g_cfg.mag_drop) return;
    auto* mc = s_mag_hidden.get();
    if (mc == nullptr) return;
    auto** pm = mc->get_property_data<API::UObject*>(L"StaticMesh");
    if (pm == nullptr || IsBadReadPtr(pm, sizeof(void*)) || *pm == nullptr) return;
    Vec3 loc{}, rot{}, scl{1.0f, 1.0f, 1.0f};
    if (!call_ret_vec3(mc, L"K2_GetComponentLocation", &loc)) return;
    call_ret_vec3(mc, L"K2_GetComponentRotation", &rot);
    call_ret_vec3(mc, L"K2_GetComponentScale", &scl);
    if (s_mag_hid_mode == 3) {   // hidden by scale: the drop takes the magazine's own size
        scl.x = (float)(scl.x / 0.001 * s_mag_hid_scale[0]);
        scl.y = (float)(scl.y / 0.001 * s_mag_hid_scale[1]);
        scl.z = (float)(scl.z / 0.001 * s_mag_hid_scale[2]);
    }
    auto* owner = API::get()->get_local_pawn(0);
    if (owner == nullptr) return;
    // A previous drop still falling is destroyed first; one at a time keeps this cheap.
    if (auto* old = s_magdrop.get()) { alignas(16) uint8_t p[64] = {0}; ue_destroy_component(old); }
    s_magdrop = TrackedObject{};
    auto* m = holster_marker_spawn_mesh(owner, *pm, (double)(scl.x > 0.01f ? scl.x : 1.0f));
    if (m == nullptr) return;
    holster_marker_place_rot(m, loc, rot.x, rot.y, rot.z);
    holster_marker_show(m, true);
    { alignas(16) uint8_t p[64] = {0}; p[0] = 1; p[1] = 1; p[2] = 1; p[3] = 0;   // KeepWorld x3, no modify
      m->call_function(L"K2_DetachFromComponent", p); }
    { alignas(16) uint8_t p[64] = {0}; API::FName nm = make_fname(L"PhysicsActor");
      memcpy(p, &nm, sizeof(int32_t) * 2); p[8] = 1;   // bUpdateOverlaps
      m->call_function(L"SetCollisionProfileName", p); }
    { alignas(16) uint8_t p[64] = {0}; p[0] = 3;   // ECollisionEnabled::QueryAndPhysics
      m->call_function(L"SetCollisionEnabled", p); }
    if (g_cfg.slide_part_dropmode == 1) {
        { alignas(16) uint8_t p[64] = {0}; m->call_function(L"SetAbsolute", p); }   // follow the parent again
        if (auto* box = sp_drop_proxy_for(m)) s_magdrop_proxy.set(box);
    } else {
        alignas(16) uint8_t p[64] = {0}; p[0] = 1;
        m->call_function(L"SetSimulatePhysics", p);
    }
    s_magdrop.set(m);
    s_magdrop_at = now_ticks();
    if (g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD mag dropped at (%.0f %.0f %.0f), scale %.2f", loc.x, loc.y, loc.z, scl.x);
}
void mag_drop_tick() {
    if (s_magdrop_at == 0) return;
    if (now_ticks() - s_magdrop_at < ms_to_ticks(g_cfg.mag_drop_ms)) return;
    if (auto* m = s_magdrop.get()) ue_destroy_component(m);
    if (auto* b = s_magdrop_proxy.get()) ue_destroy_component(b);
    s_magdrop = TrackedObject{}; s_magdrop_proxy = TrackedObject{};
    s_magdrop_at = 0;
}

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

// ---- THE ON-WEAPON AMMO DISPLAY (the hidden reload, 2026-09-07). The AR's counter is a
// material (MI_AssaultRifle_Display_*, a digit atlas T_..._AmmoCounter) on a display mesh; the
// count is a scalar parameter. The component is found once per weapon actor (material 0's name
// contains "Display"), its scalar parameters are logged once, and every one whose name looks
// like a count is written to 0 each frame while the hidden reload is pending.
namespace {
struct WdField { int32_t off; int kind; };   // kind: 0 float, 1 double, 2 int32, 3 byte
struct WpnDisplay { API::UObject* actor = nullptr; TrackedObject comp; std::vector<std::wstring> params; bool searched = false;
                    std::vector<WdField> fields; std::vector<WdField> hits; std::vector<int> cpd_hits; std::vector<std::wstring> param_hits; bool matched = false; bool dumped = false; };
WpnDisplay s_wd;
API::UObject* wd_material_at(API::UObject* comp, int32_t slot);
int32_t wd_material_count(API::UObject* comp);
void wd_scalar_params(API::UObject* mat, std::vector<std::wstring>& out, std::wstring& values);
// The display component's own Blueprint variables (its BPC_ classes) and its custom primitive
// data: listed once with values, and whichever equals the live round count is the one written.
void wd_list_fields(API::UObject* c) {
    s_wd.fields.clear();
    std::wstring line; int n = 0;
    for (API::UStruct* st = c->get_class(); st != nullptr && n < 120; st = st->get_super_struct()) {
        if (IsBadReadPtr(st, sizeof(void*))) break;
        const std::wstring sn = st->get_fname() ? st->get_fname()->to_string() : L"?";
        if (sn.rfind(L"BPC_", 0) != 0) break;   // engine classes: stop
        for (API::FField* f = st->get_child_properties(); f != nullptr && n < 120; f = f->get_next()) {
            if (IsBadReadPtr(f, sizeof(void*))) break;
            auto* fc = f->get_class(); const std::wstring tn = fc ? fc->get_name() : L"?";
            const auto* fn = f->get_fname(); const std::wstring nm = fn ? fn->to_string() : L"?";
            const int32_t off = static_cast<API::FProperty*>(f)->get_offset();
            const uint8_t* base = reinterpret_cast<const uint8_t*>(c);
            wchar_t b[128]; int kind = -1; double v = 0.0;
            if (tn == L"FloatProperty")  { kind = 0; v = *reinterpret_cast<const float*>(base + off); }
            else if (tn == L"DoubleProperty") { kind = 1; v = *reinterpret_cast<const double*>(base + off); }
            else if (tn == L"IntProperty")    { kind = 2; v = *reinterpret_cast<const int32_t*>(base + off); }
            else if (tn == L"ByteProperty")   { kind = 3; v = *reinterpret_cast<const uint8_t*>(base + off); }
            if (kind >= 0) { s_wd.fields.push_back({off, kind}); swprintf_s(b, L" %ls=%.2f", nm.c_str(), v); }
            else swprintf_s(b, L" %ls:%ls", nm.c_str(), tn.c_str());
            line += b; ++n;
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] WPNDISPLAY fields:%ls", line.c_str());
    // Custom primitive data: FCustomPrimitiveData { TArray<float> Data } at the property.
    if (auto* pcpd = c->get_class()->find_property(L"CustomPrimitiveData")) {
        struct TArr { const float* data; int32_t num; int32_t max; };
        const auto* a = reinterpret_cast<const TArr*>(reinterpret_cast<const uint8_t*>(c) + pcpd->get_offset());
        std::wstring cl;
        if (a->data != nullptr && a->num > 0 && a->num < 64 && !IsBadReadPtr(a->data, (size_t)a->num * 4)) for (int32_t i = 0; i < a->num; ++i) { wchar_t b[32]; swprintf_s(b, L" [%d]=%.2f", i, a->data[i]); cl += b; }
        API::get()->log_info("[Halo-CampE-UEVR] WPNDISPLAY custom primitive data (%d):%ls", a->data ? a->num : 0, cl.c_str());
    } else API::get()->log_info("[Halo-CampE-UEVR] WPNDISPLAY: no CustomPrimitiveData property");
}
double wd_field_read(API::UObject* c, const WdField& f) {
    const uint8_t* base = reinterpret_cast<const uint8_t*>(c) + f.off;
    switch (f.kind) { case 0: return *reinterpret_cast<const float*>(base); case 1: return *reinterpret_cast<const double*>(base); case 2: return *reinterpret_cast<const int32_t*>(base); default: return *reinterpret_cast<const uint8_t*>(base); }
}
void wd_field_write0(API::UObject* c, const WdField& f) {
    uint8_t* base = reinterpret_cast<uint8_t*>(c) + f.off;
    switch (f.kind) { case 0: *reinterpret_cast<float*>(base) = 0.0f; break; case 1: *reinterpret_cast<double*>(base) = 0.0; break; case 2: *reinterpret_cast<int32_t*>(base) = 0; break; default: *reinterpret_cast<uint8_t*>(base) = 0; break; }
}
// Which field or primitive float carries the count: the ones equal to the live rounds (>= 2).
void wd_match(API::UObject* c) {
    if (s_wd.matched) return;
    const auto* r = rounds_field(); if (r == nullptr) return;
    const int rounds = (int)*r; if (rounds < 2) return;
    std::wstring line;
    for (const auto& f : s_wd.fields) { const double v = wd_field_read(c, f); if (std::fabs(v - rounds) < 0.01) { s_wd.hits.push_back(f); wchar_t b[48]; swprintf_s(b, L" field@0x%X", (unsigned)f.off); line += b; } }
    if (auto* pcpd = c->get_class()->find_property(L"CustomPrimitiveData")) {
        struct TArr { const float* data; int32_t num; int32_t max; };
        const auto* a = reinterpret_cast<const TArr*>(reinterpret_cast<const uint8_t*>(c) + pcpd->get_offset());
        if (a->data != nullptr && a->num > 0 && a->num < 64 && !IsBadReadPtr(a->data, (size_t)a->num * 4)) for (int32_t i = 0; i < a->num; ++i) if (std::fabs(a->data[i] - rounds) < 0.01) { s_wd.cpd_hits.push_back(i); wchar_t b[32]; swprintf_s(b, L" cpd[%d]", i); line += b; }
    }
    // Scalar parameters whose VALUE is the count, on any of the component's materials.
    const int32_t n = wd_material_count(c);
    for (int32_t slot = 0; slot < n; ++slot) {
        auto* m = wd_material_at(c, slot);
        if (m == nullptr || IsBadReadPtr(m, sizeof(void*))) continue;
        std::vector<std::wstring> names; std::wstring values;
        wd_scalar_params(m, names, values);
        // values is " name=v name=v ..."; re-read each value by name from the string.
        {   // texture parameters (a dynamic display texture would show here)
            std::wstring tl;
            for (int hop = 0; hop < 4 && m != nullptr; ++hop) {
                auto* mc = m->get_class(); if (mc == nullptr) break;
                if (auto* prop = mc->find_property(L"TextureParameterValues")) {
                    auto* fc = prop->get_class();
                    if (fc != nullptr && fc->get_name() == L"ArrayProperty") {
                        auto* inner = static_cast<API::FArrayProperty*>(prop)->get_inner();
                        if (inner && inner->get_class() && inner->get_class()->get_name() == L"StructProperty") {
                            auto* st = static_cast<API::FStructProperty*>(inner)->get_struct();
                            const int32_t stride = st ? st->get_properties_size() : 0;
                            auto* pinfo = st ? st->find_property(L"ParameterInfo") : nullptr; auto* pval = st ? st->find_property(L"ParameterValue") : nullptr;
                            int32_t name_off = -1;
                            if (pinfo && pinfo->get_class() && pinfo->get_class()->get_name() == L"StructProperty") { auto* ist = static_cast<API::FStructProperty*>(pinfo)->get_struct(); auto* pn = ist ? ist->find_property(L"Name") : nullptr; if (pn) name_off = pinfo->get_offset() + pn->get_offset(); }
                            struct TArr { uint8_t* data; int32_t num; int32_t max; };
                            const auto* a = reinterpret_cast<const TArr*>(reinterpret_cast<uint8_t*>(m) + prop->get_offset());
                            if (stride > 0 && name_off >= 0 && pval && a->data && a->num > 0 && a->num < 64 && !IsBadReadPtr(a->data, (size_t)a->num * stride)) {
                                for (int32_t i = 0; i < a->num; ++i) {
                                    const uint8_t* e = a->data + (size_t)i * stride;
                                    auto* tex = *reinterpret_cast<API::UObject* const*>(e + pval->get_offset());
                                    tl += L" " + reinterpret_cast<const API::FName*>(e + name_off)->to_string() + L"=" + ((tex && !IsBadReadPtr(tex, sizeof(void*)) && tex->get_fname()) ? tex->get_fname()->to_string() : L"null");
                                }
                            }
                        }
                    }
                }
                auto* pp = mc->find_property(L"Parent"); if (!pp) break;
                auto** parent = reinterpret_cast<API::UObject**>(reinterpret_cast<uint8_t*>(m) + pp->get_offset()); if (IsBadReadPtr(parent, sizeof(void*))) break;
                m = *parent;
            }
            API::get()->log_info("[Halo-CampE-UEVR] WPNDISPLAY slot %d textures:%ls", slot, tl.empty() ? L" (none)" : tl.c_str());
        }
        for (const auto& nm : names) {
            const std::wstring key = L" " + nm + L"=";
            const size_t at = values.find(key); if (at == std::wstring::npos) continue;
            const double v = _wtof(values.c_str() + at + key.size());
            if (std::fabs(v - rounds) < 0.01) { s_wd.param_hits.push_back(nm); line += L" param:" + nm; }
        }
    }
    s_wd.matched = true;
    API::get()->log_info("[Halo-CampE-UEVR] WPNDISPLAY count match at rounds %d:%ls", rounds, line.empty() ? L" NONE" : line.c_str());
}
std::wstring wlower(std::wstring w) { for (auto& ch : w) ch = (wchar_t)towlower(ch); return w; }
API::UObject* wd_material_at(API::UObject* comp, int32_t slot) {
    auto* cls = comp->get_class(); if (cls == nullptr || cls->find_function(L"GetMaterial") == nullptr) return nullptr;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    *reinterpret_cast<int32_t*>(p) = slot;
    comp->call_function(L"GetMaterial", p);
    return *reinterpret_cast<API::UObject**>(p + 8);
}
int32_t wd_material_count(API::UObject* comp) {
    auto* cls = comp->get_class(); if (cls == nullptr || cls->find_function(L"GetNumMaterials") == nullptr) return 0;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    comp->call_function(L"GetNumMaterials", p);
    const int32_t n = *reinterpret_cast<int32_t*>(p);
    return (n < 0 || n > 16) ? 0 : n;
}
API::UObject* wd_material0(API::UObject* comp) { return wd_material_at(comp, 0); }
// Scalar parameter names of a material instance, walking Parent until one has entries.
void wd_scalar_params(API::UObject* mat, std::vector<std::wstring>& out, std::wstring& values) {
    for (int hop = 0; hop < 6 && mat != nullptr && !IsBadReadPtr(mat, sizeof(void*)); ++hop) {
        auto* cls = mat->get_class(); if (cls == nullptr) return;
        auto* prop = cls->find_property(L"ScalarParameterValues");
        if (prop != nullptr) {
            auto* fc = prop->get_class();
            if (fc != nullptr && fc->get_name() == L"ArrayProperty") {
                auto* inner = static_cast<API::FArrayProperty*>(prop)->get_inner();
                auto* ifc = inner ? inner->get_class() : nullptr;
                if (ifc != nullptr && ifc->get_name() == L"StructProperty") {
                    auto* st = static_cast<API::FStructProperty*>(inner)->get_struct();
                    const int32_t stride = st ? st->get_properties_size() : 0;
                    auto* pinfo = st ? st->find_property(L"ParameterInfo") : nullptr;
                    auto* pval  = st ? st->find_property(L"ParameterValue") : nullptr;
                    int32_t name_off = -1;
                    if (pinfo != nullptr && pinfo->get_class() != nullptr && pinfo->get_class()->get_name() == L"StructProperty") {
                        auto* ist = static_cast<API::FStructProperty*>(pinfo)->get_struct();
                        auto* pn = ist ? ist->find_property(L"Name") : nullptr;
                        if (pn != nullptr) name_off = pinfo->get_offset() + pn->get_offset();
                    }
                    struct TArr { uint8_t* data; int32_t num; int32_t max; };
                    const auto* arr = reinterpret_cast<const TArr*>(reinterpret_cast<uint8_t*>(mat) + prop->get_offset());
                    if (stride > 0 && name_off >= 0 && arr->data != nullptr && arr->num > 0 && arr->num < 256 && !IsBadReadPtr(arr->data, (size_t)arr->num * stride)) {
                        for (int32_t i = 0; i < arr->num; ++i) {
                            const uint8_t* e = arr->data + (size_t)i * stride;
                            const std::wstring nm = reinterpret_cast<const API::FName*>(e + name_off)->to_string();
                            const float v = pval ? *reinterpret_cast<const float*>(e + pval->get_offset()) : 0.0f;
                            out.push_back(nm);
                            wchar_t b[96]; swprintf_s(b, L" %ls=%.3f", nm.c_str(), v); values += b;
                        }
                        return;
                    }
                }
            }
        }
        auto* pp = cls->find_property(L"Parent");
        if (pp == nullptr) return;
        auto** parent = reinterpret_cast<API::UObject**>(reinterpret_cast<uint8_t*>(mat) + pp->get_offset());
        if (IsBadReadPtr(parent, sizeof(void*))) return;
        mat = *parent;
    }
}
void wd_find() {
    auto* actor = fp_weapon_actor();
    if (actor == s_wd.actor && (s_wd.searched)) return;
    s_wd = WpnDisplay{}; s_wd.actor = actor; s_wd.searched = true;
    if (actor == nullptr) return;
    // Every slot of every component. A dynamic instance (class MaterialInstanceDynamic, named
    // like MaterialInstanceDynamic_N) is where a game-set count lives; a name with "display"
    // marks the readout's static instance. The first component carrying either is the one.
    weapon_components([&](API::UObject* c) {
        const int32_t n = wd_material_count(c);
        for (int32_t slot = 0; slot < n; ++slot) {
            auto* m = wd_material_at(c, slot);
            if (m == nullptr || IsBadReadPtr(m, sizeof(void*)) || m->get_fname() == nullptr) continue;
            const std::wstring mcls = class_name_of(m);
            const std::wstring mn = wlower(m->get_fname()->to_string());
            const bool dyn = mcls.find(L"Dynamic") != std::wstring::npos;
            if (!dyn && mn.find(L"display") == std::wstring::npos) continue;
            std::vector<std::wstring> params; std::wstring values;
            wd_scalar_params(m, params, values);
            API::get()->log_info("[Halo-CampE-UEVR] WPNDISPLAY %ls (%ls) slot %d on %ls (%ls): %d scalar param(s)%ls", m->get_fname()->to_string().c_str(), mcls.c_str(), slot, c->get_fname() ? c->get_fname()->to_string().c_str() : L"?", class_name_of(c).c_str(), (int)params.size(), values.c_str());
            if (s_wd.comp.get() == nullptr || (dyn && s_wd.params.empty())) { s_wd.comp.set(c); s_wd.params = params; }
        }
        return true;
    });
    if (auto* c = s_wd.comp.get()) wd_list_fields(c);
    if (s_wd.comp.get() == nullptr) API::get()->log_info("[Halo-CampE-UEVR] WPNDISPLAY: no display material on this weapon");
}
bool wd_param_is_count(const std::wstring& nm) {
    const std::wstring lo = wlower(nm);
    return lo.find(L"ammo") != std::wstring::npos || lo.find(L"count") != std::wstring::npos || lo.find(L"round") != std::wstring::npos
        || lo.find(L"clip") != std::wstring::npos || lo.find(L"bullet") != std::wstring::npos || lo.find(L"mag") != std::wstring::npos;
}
void wd_write_zero() {
    wd_find();
    auto* c = s_wd.comp.get(); if (c == nullptr) return;
    wd_match(c);
    for (const auto& f : s_wd.hits) wd_field_write0(c, f);
    if (!s_wd.cpd_hits.empty()) {
        if (auto* fn = c->get_class()->find_function(L"SetCustomPrimitiveDataFloat")) {
            auto* pi = fn->find_property(L"DataIndex"); auto* pv = fn->find_property(L"Value");
            if (pi != nullptr && pv != nullptr) for (int idx : s_wd.cpd_hits) {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                *reinterpret_cast<int32_t*>(p + pi->get_offset()) = idx;
                *reinterpret_cast<float*>(p + pv->get_offset()) = 0.0f;
                c->call_function(L"SetCustomPrimitiveDataFloat", p);
            }
        }
    }
    auto* cls = c->get_class(); auto* fn = cls ? cls->find_function(L"SetScalarParameterValueOnMaterials") : nullptr; if (fn == nullptr) return;
    auto* pn = fn->find_property(L"ParameterName"); auto* pv = fn->find_property(L"ParameterValue"); if (pn == nullptr || pv == nullptr) return;
    for (const auto& nm : s_wd.params) {
        const bool hit = std::find(s_wd.param_hits.begin(), s_wd.param_hits.end(), nm) != s_wd.param_hits.end();
        if (!hit && !wd_param_is_count(nm)) continue;
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        const API::FName fnm = make_fname(nm.c_str());
        memcpy(p + pn->get_offset(), &fnm, sizeof(API::FName));
        *reinterpret_cast<float*>(p + pv->get_offset()) = 0.0f;
        c->call_function(L"SetScalarParameterValueOnMaterials", p);
    }
}
} // namespace
// ---- THE READOUT THROUGH THE ANIMATION (2026-09-07): no dynamic material, no variables, no
// primitive data on the display component, so the count must reach the digits through the
// weapon's anim instance. Every scalar anim variable equal to the live count is a candidate;
// each is written 0 on the game tick and on the render path, and once a second the readback
// says whether the game re-set it (which tells where in the frame the copy happens).
namespace {
std::vector<AnimVar> s_ad_hits; bool s_ad_matched = false; API::UObject* s_ad_ai = nullptr; long long s_ad_said = 0;
void mpc_probe(API::UObject* ctx, int rounds);
void ad_reset_if_new(API::UObject* animbp) { if (animbp != s_ad_ai) { s_ad_ai = animbp; s_ad_hits.clear(); s_ad_matched = false; } }
void ad_match(API::UObject* animbp) {
    if (s_ad_matched) return;
    const auto* r = rounds_field(); if (r == nullptr) return;
    const int rounds = (int)*r; if (rounds < 2) return;
    std::vector<AnimVar> vars; av_collect(animbp, vars);
    std::wstring line, named;
    for (const auto& v : vars) {
        const std::wstring lo = wlower(v.name);
        const bool keyword = lo.find(L"ammo") != std::wstring::npos || lo.find(L"round") != std::wstring::npos || lo.find(L"count") != std::wstring::npos || lo.find(L"clip") != std::wstring::npos || lo.find(L"digit") != std::wstring::npos || lo.find(L"display") != std::wstring::npos;
        if (keyword) { wchar_t b[128]; swprintf_s(b, L" %ls=%.2f", v.name.c_str(), v.v); named += b; }
        if (std::fabs(v.v - rounds) < 0.01) { s_ad_hits.push_back(v); wchar_t b[128]; swprintf_s(b, L" %ls(%ls@0x%X)", v.name.c_str(), v.cls.c_str(), (unsigned)v.off); line += b; }
    }
    s_ad_matched = true;
    std::wstring all; for (const auto& v : vars) { wchar_t b[128]; swprintf_s(b, L" %ls=%.2f", v.name.c_str(), v.v); all += b; }
    API::get()->log_info("[Halo-CampE-UEVR] ANIMREADOUT %zu vars on %ls:%ls", vars.size(), class_name_of(animbp).c_str(), all.c_str());
    API::get()->log_info("[Halo-CampE-UEVR] ANIMREADOUT named:%ls", named.empty() ? L" (none)" : named.c_str());
    API::get()->log_info("[Halo-CampE-UEVR] ANIMREADOUT equal to rounds %d:%ls", rounds, line.empty() ? L" NONE" : line.c_str());
    mpc_probe(fp_weapon_actor(), rounds);
}
void ad_write(API::UObject* animbp, const AnimVar& v, double val) {
    uint8_t* p = reinterpret_cast<uint8_t*>(animbp) + v.off;
    switch (v.kind) { case 1: case 2: *p = (uint8_t)val; break; case 3: *reinterpret_cast<int32_t*>(p) = (int32_t)val; break; case 4: *reinterpret_cast<float*>(p) = (float)val; break; case 5: *reinterpret_cast<double*>(p) = val; break; }
}
// ---- MATERIAL PARAMETER COLLECTIONS (2026-09-07): the last channel the count could reach the
// digits through. Every collection's scalar parameters are read live through
// KismetMaterialLibrary.GetScalarParameterValue, listed once, and any equal to the count named.
void mpc_probe(API::UObject* ctx, int rounds) {
    static bool s_done = false; if (s_done || ctx == nullptr) return; s_done = true;
    auto* kml = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.KismetMaterialLibrary");
    auto* fn = kml ? kml->find_function(L"GetScalarParameterValue") : nullptr;
    auto* cdo = kml ? kml->get_class_default_object() : nullptr;
    if (fn == nullptr || cdo == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] MPC: KismetMaterialLibrary.GetScalarParameterValue not found"); return; }
    auto* pctx = fn->find_property(L"WorldContextObject"); auto* pcol = fn->find_property(L"Collection"); auto* pnm = fn->find_property(L"ParameterName"); auto* pret = fn->find_property(L"ReturnValue");
    if (!pctx || !pcol || !pnm || !pret) { API::get()->log_info("[Halo-CampE-UEVR] MPC: parameter layout not found"); return; }
    auto* arr = API::get()->get_uobject_array(); if (arr == nullptr) return;
    const int32_t nn = arr->get_object_count(); int ncol = 0;
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"MaterialParameterCollection") continue;
        ++ncol;
        std::vector<std::wstring> names;
        if (auto* prop = o->get_class()->find_property(L"ScalarParameters")) {
            auto* fc = prop->get_class();
            if (fc && fc->get_name() == L"ArrayProperty") {
                auto* inner = static_cast<API::FArrayProperty*>(prop)->get_inner();
                if (inner && inner->get_class() && inner->get_class()->get_name() == L"StructProperty") {
                    auto* st = static_cast<API::FStructProperty*>(inner)->get_struct();
                    const int32_t stride = st ? st->get_properties_size() : 0;
                    auto* pn = st ? st->find_property(L"ParameterName") : nullptr;
                    struct TArr { uint8_t* data; int32_t num; int32_t max; };
                    const auto* a = reinterpret_cast<const TArr*>(reinterpret_cast<uint8_t*>(o) + prop->get_offset());
                    if (stride > 0 && pn && a->data && a->num > 0 && a->num < 256 && !IsBadReadPtr(a->data, (size_t)a->num * stride))
                        for (int32_t k = 0; k < a->num; ++k) names.push_back(reinterpret_cast<const API::FName*>(a->data + (size_t)k * stride + pn->get_offset())->to_string());
                }
            }
        }
        std::wstring line, hits;
        for (const auto& nm : names) {
            alignas(16) uint8_t q[128] = {0};
            *reinterpret_cast<API::UObject**>(q + pctx->get_offset()) = ctx;
            *reinterpret_cast<API::UObject**>(q + pcol->get_offset()) = o;
            const API::FName fnm = make_fname(nm.c_str()); memcpy(q + pnm->get_offset(), &fnm, sizeof(API::FName));
            cdo->call_function(L"GetScalarParameterValue", q);
            const float v = *reinterpret_cast<const float*>(q + pret->get_offset());
            wchar_t b[128]; swprintf_s(b, L" %ls=%.2f", nm.c_str(), v); line += b;
            if (std::fabs(v - rounds) < 0.01) hits += L" " + nm;
        }
        API::get()->log_info("[Halo-CampE-UEVR] MPC %ls (%d scalar):%ls%ls%ls", o->get_fname() ? o->get_fname()->to_string().c_str() : L"?", (int)names.size(), line.c_str(), hits.empty() ? L"" : L"  <-- EQUAL TO ROUNDS:", hits.c_str());
    }
    API::get()->log_info("[Halo-CampE-UEVR] MPC: %d collection(s) listed at rounds %d", ncol, rounds);
}
void ad_write_zero(bool render_path) {
    auto* animbp = reload_weapon_anim_instance(); if (animbp == nullptr) return;
    ad_reset_if_new(animbp);
    ad_match(animbp);
    const long long nowt = now_ticks();
    const bool say = nowt - s_ad_said > ms_to_ticks(1000);
    std::wstring line;
    // The explicit ammo frame, on both paths: the tick write alone left the readout at full, so
    // the game re-sets it after our tick; the render-path write lands after the game's.
    if (auto* pf = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame")) {
        if (!IsBadWritePtr(pf, sizeof(int32_t))) { if (say) { wchar_t b[64]; swprintf_s(b, L" ExplicitFrame=%d", *pf); line += b; } *pf = 0; }
    }
    if (s_ad_hits.empty()) { if (say && !line.empty()) { s_ad_said = nowt; API::get()->log_info("[Halo-CampE-UEVR] ANIMREADOUT %s: before the write%ls (0 written)", render_path ? "render" : "tick", line.c_str()); } return; }
    for (const auto& v : s_ad_hits) {
        const double before = av_read(animbp, v.off, v.kind);
        if (say) { wchar_t b[96]; swprintf_s(b, L" %ls=%.1f", v.name.c_str(), before); line += b; }
        ad_write(animbp, v, 0.0);
    }
    if (say) { s_ad_said = nowt; API::get()->log_info("[Halo-CampE-UEVR] ANIMREADOUT %s: before the write%ls (0 written)", render_path ? "render" : "tick", line.c_str()); }
}
} // namespace
void gesture_render_tick() {
    if (!g_wristhud_hide_cradle.load(std::memory_order_relaxed)) return;
    wd_write_zero();
    ad_write_zero(true);
}
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

// ---- RELOAD STATE PER WEAPON (reloadstate, Config.hpp) -----------------------------------------
// The old memory had four holes, and each one hands a weapon back "mag in, racked":
//  1. it saved only when the weapon changed WHILE a reload was in flight, so a seated magazine
//     waiting for the rack, a locked-back slide or the phantom round followed the player;
//  2. gesture_reset (stick mode: vehicles, cutscenes, death; calibration) forced Idle and cleared
//     the flags without saving anything;
//  3. slide_phantom_tick dropped the phantom flag on the key change, on the very tick the
//     restore had put it back;
//  4. the magazine hide landed once on one component, and a swap spawns new components.
// Here the state is kept per weapon identity, saved on every exit path (and mirrored every tick
// with reload_state_save 1), and loaded whenever that identity is in hand again.
struct RsRec {
    std::string id;            // "i:0x<datum>" (instance) or "t:<type>" (type name); empty = free slot
    std::string type;
    int32_t     datum = -1;
    uint32_t    epoch = 0;
    long long   touched = 0;
    ReloadState state = ReloadState::Idle;
    float       grab_y = 0.0f;
    int         chamber_left = 0;
    int32_t     lock_frame = 0;
    int         coop_lock_rounds = 1;
    bool        empty_at_drop = false, locked_back = false, lock_pending = false, reload_due = false;
    bool        pressed_early = false, press_pending = false, true_empty = false, hide_display = false;
};
constexpr int RS_MAX = 16;
RsRec         s_rs_recs[RS_MAX];
std::string   s_rs_live_id;              // identity the live state belongs to; empty = not synced
std::string   s_rs_live_type;            // type of the weapon the live state belongs to (also while its id is pending)
int32_t       s_rs_live_datum = -1;
std::string   s_rs_seen_type;
API::UObject* s_rs_seen_actor = nullptr; // identity only, never dereferenced
long long     s_rs_since = 0;            // when the type or actor in hand last changed (the datum wait)
API::UObject* s_rs_pc = nullptr;         // identity only
uint32_t      s_rs_epoch = 1;
std::string   s_rs_recent[2];            // the last two identities held
std::string   s_rs_carry[2];             // reloadstate 3: those two, frozen at the last level change
long long     s_rs_carry_until = 0;
bool          s_rs_death_cleared = false;
std::string   s_rs_hands[2];             // the two types carried, the one in hand first
bool          s_rs_after_reset = false;  // the next identity adoption follows a gesture reset
bool          s_rs_live_no_manual = false;   // the weapon in hand has no manual reload (reloadskipweapons)
int32_t       s_rs_loaded_datum = -1;    // per type: the object that wrote the record just loaded
bool          s_rs_loaded_after_reset = false;
bool          s_rs_loaded_old_epoch = false;

int32_t rs_read_datum(API::UObject* wpn) {
    if (wpn == nullptr) return -1;
    auto** pc = wpn->get_property_data<API::UObject*>(L"BlamObjectSynchronization");
    if (pc == nullptr || IsBadReadPtr(pc, sizeof(void*)) || *pc == nullptr || IsBadReadPtr(*pc, sizeof(void*))) return -1;
    auto* p = (*pc)->get_property_data<int32_t>(L"BlamObjectIndex");
    if (p == nullptr || IsBadReadPtr(p, sizeof(int32_t))) return -1;
    return *p;
}
void rs_capture(RsRec& r) {
    r.state = s_reload; r.grab_y = s_grab_y;
    r.chamber_left = s_sl_chamber_left; r.lock_frame = s_sl_lock_frame; r.coop_lock_rounds = s_coop_lock_rounds;
    r.empty_at_drop = s_sl_empty_at_drop; r.locked_back = s_sl_locked_back; r.lock_pending = s_sl_lock_pending;
    r.reload_due = s_sl_reload_due; r.pressed_early = s_sl_pressed_early;
    // A press scheduled 150 ms after the chambered shot belongs to THIS gun: carried as pending,
    // it goes out at the seat instead of into whichever weapon is in hand when the timer runs out.
    r.press_pending = s_sl_press_pending || s_sl_press_due_at != 0;
    r.true_empty = s_true_empty; r.hide_display = s_hide_display;
}
bool rs_clean(const RsRec& r) {
    return r.state == ReloadState::Idle && !r.locked_back && !r.lock_pending && !r.reload_due && !r.pressed_early
        && !r.press_pending && !r.true_empty && !r.hide_display;
}
bool rs_same(const RsRec& a, const RsRec& b) {
    return a.state == b.state && a.grab_y == b.grab_y && a.chamber_left == b.chamber_left && a.lock_frame == b.lock_frame
        && a.coop_lock_rounds == b.coop_lock_rounds && a.empty_at_drop == b.empty_at_drop && a.locked_back == b.locked_back
        && a.lock_pending == b.lock_pending && a.reload_due == b.reload_due && a.pressed_early == b.pressed_early
        && a.press_pending == b.press_pending && a.true_empty == b.true_empty && a.hide_display == b.hide_display;
}
void rs_log(const char* verb, const char* why, const RsRec& r) {
    if (!g_cfg.reload_state_log) return;
    API::get()->log_info("[Halo-CampE-UEVR] RSTATE %s (%s) id=%s type=%s datum=0x%08X epoch=%u | state=%s chamber=%d "
                         "empty_at_drop=%d locked_back=%d lock_pending=%d reload_due=%d pressed_early=%d press_pending=%d "
                         "true_empty=%d hide_display=%d lock_frame=%d coop_lock=%d grab_y=%.3f",
                         verb, why, r.id.c_str(), r.type.c_str(), (unsigned)r.datum, (unsigned)r.epoch, state_name(r.state),
                         r.chamber_left, (int)r.empty_at_drop, (int)r.locked_back, (int)r.lock_pending, (int)r.reload_due,
                         (int)r.pressed_early, (int)r.press_pending, (int)r.true_empty, (int)r.hide_display,
                         (int)r.lock_frame, r.coop_lock_rounds, r.grab_y);
}
RsRec* rs_find(const std::string& id) {
    if (id.empty()) return nullptr;
    for (auto& r : s_rs_recs) if (!r.id.empty() && r.id == id) return &r;
    return nullptr;
}
RsRec* rs_slot() {
    RsRec* oldest = &s_rs_recs[0];
    for (auto& r : s_rs_recs) {
        if (r.id.empty()) return &r;
        if (r.touched < oldest->touched) oldest = &r;
    }
    if (g_cfg.reload_state_log) API::get()->log_info("[Halo-CampE-UEVR] RSTATE table full: forgetting the oldest record %s", oldest->id.c_str());
    return oldest;
}
void rs_forget_all(const char* why) {
    for (auto& rec : s_rs_recs) rec = RsRec{};
    s_rs_hands[0].clear(); s_rs_hands[1].clear();
    s_rs_loaded_datum = -1;
    if (g_cfg.reload_state_log) API::get()->log_info("[Halo-CampE-UEVR] RSTATE forget every record (%s)", why);
}
void rs_forget_type(const std::string& type, const char* why) {
    for (auto& rec : s_rs_recs) if (!rec.id.empty() && rec.type == type) { rs_log("forget", why, rec); rec = RsRec{}; }
}
// Halo carries two weapons, and a pickup swaps out the one in the hand.
void rs_hands_note(const std::string& type) {
    if (type.empty() || type == s_rs_hands[0]) return;
    if (type == s_rs_hands[1]) { std::swap(s_rs_hands[0], s_rs_hands[1]); return; }
    if (g_cfg.reload_state_id == 1 && g_cfg.reload_state_drop == 1 && !s_rs_hands[0].empty() && !s_rs_hands[1].empty()) {
        const std::string dropped = s_rs_hands[0];
        if (g_cfg.reload_state_log)
            API::get()->log_info("[Halo-CampE-UEVR] RSTATE carried [%s] [%s], %s came in: %s was swapped for it", s_rs_hands[0].c_str(), s_rs_hands[1].c_str(), type.c_str(), dropped.c_str());
        rs_forget_type(dropped, "swapped for a weapon on the ground");
        s_rs_hands[0] = type;
        return;
    }
    s_rs_hands[1] = s_rs_hands[0]; s_rs_hands[0] = type;
}
// Per type: the record for this type was written by a different Blam object than the one in hand.
// True when the record was forgotten.
bool rs_other_object(const std::string& type, int32_t saved, int32_t held, bool after_reset, bool old_epoch, const char* when) {
    const bool death = after_reset && g_cfg.reload_state_death == 2;
    const bool drop  = !after_reset && !old_epoch && g_cfg.reload_state_drop == 2;
    if (g_cfg.reload_state_log)
        API::get()->log_info("[Halo-CampE-UEVR] RSTATE %s: the %s record was saved by object 0x%08X, the one in hand is 0x%08X (after reset %d, older level %d) -> %s",
                             when, type.c_str(), (unsigned)saved, (unsigned)held, (int)after_reset, (int)old_epoch,
                             death ? "respawn, forget every record" : (drop ? "another weapon of this type, forget its record" : "record KEPT"));
    if (death) { rs_forget_all("respawn: the weapon in hand is a new object"); return true; }
    if (drop)  { rs_forget_type(type, "another weapon of this type, the record's own was left behind"); return true; }
    return false;
}
// The live state into its weapon's record. A clean state (nothing in progress) removes the record.
void rs_save(const char* why, bool quiet_if_same) {
    if (s_rs_live_id.empty()) return;
    RsRec now; now.id = s_rs_live_id; now.type = s_rs_live_type; now.datum = s_rs_live_datum; now.epoch = s_rs_epoch;
    rs_capture(now);
    RsRec* r = rs_find(s_rs_live_id);
    if (s_rs_live_no_manual) {   // no manual reload on this weapon: it never holds a record
        if (r != nullptr) { rs_log("forget", "weapon has no manual reload", *r); *r = RsRec{}; }
        return;
    }
    if (now.datum == -1 && r != nullptr) now.datum = r->datum;   // an unreadable tick never erases the saved object
    if (rs_clean(now)) {
        if (r != nullptr) { rs_log("clear", why, now); *r = RsRec{}; }
        else if (!quiet_if_same) rs_log("save", why, now);
        return;
    }
    const bool same = (r != nullptr) && rs_same(*r, now);
    if (r == nullptr) r = rs_slot();
    now.touched = now_ticks();
    *r = now;
    if (!(same && quiet_if_same)) rs_log("save", why, now);
}
// The live state back to "nothing in progress" for the weapon now in hand.
void rs_live_fresh(const char* why) {
    if (s_reload != ReloadState::Idle) set_state(ReloadState::Idle, why);
    s_sl_chamber_left = 0; s_sl_empty_at_drop = false; s_sl_locked_back = false; s_sl_lock_pending = false;
    s_sl_reload_due = false; s_sl_pressed_early = false; s_sl_press_pending = false; s_sl_press_due_at = 0;
    s_sl_lock_frame = 0; s_sl_rack_done = false;
    s_true_empty = false; s_hide_display = false; s_coop_lock_rounds = 1;
    // A hand on the slide does not travel with the gun: the weapon left the hand.
    s_sl_held = false; s_sl_racked = false; s_sl_th = false; s_sl_release_at = 0;
    g_slide_pull.store(0.0f, std::memory_order_relaxed);
    s_reload_weapon.clear();
    s_ph_rebase = true;
}
void rs_live_load(const RsRec& r, const char* why) {
    s_sl_chamber_left = r.chamber_left; s_sl_empty_at_drop = r.empty_at_drop; s_sl_locked_back = r.locked_back;
    s_sl_lock_pending = r.lock_pending; s_sl_reload_due = r.reload_due; s_sl_pressed_early = r.pressed_early;
    s_sl_press_pending = r.press_pending; s_sl_press_due_at = 0; s_sl_lock_frame = r.lock_frame; s_sl_rack_done = false;
    s_true_empty = r.true_empty; s_hide_display = r.hide_display; s_coop_lock_rounds = r.coop_lock_rounds;
    s_grab_y = r.grab_y;
    if (r.state != ReloadState::Idle) {
        s_reload_weapon = s_rs_live_type;
        s_restoring_mag_out = true;   // no second magazine falls and no press goes out on a restore
        set_state(r.state, why);
        s_restoring_mag_out = false;
    }
    rs_log("restore", why, r);
}
void reload_state_track() {
    const long long nowt = now_ticks();
    // A replaced PlayerController is a map load (the aim reference keys off the same change).
    if (auto* pc = API::get()->get_player_controller(0)) {
        if (pc != s_rs_pc) {
            if (s_rs_pc != nullptr) {
                ++s_rs_epoch;
                if (g_cfg.reload_state_level == 0) rs_forget_all("level change, reloadstatelevel 0");
                s_rs_carry[0] = s_rs_recent[0]; s_rs_carry[1] = s_rs_recent[1];
                s_rs_carry_until = nowt + ms_to_ticks(120000);
                if (g_cfg.reload_state_log)
                    API::get()->log_info("[Halo-CampE-UEVR] RSTATE level change: epoch %u, carry candidates [%s] [%s]",
                                         (unsigned)s_rs_epoch, s_rs_carry[0].c_str(), s_rs_carry[1].c_str());
            }
            s_rs_pc = pc;
        }
    }
    auto* actor = fp_weapon_actor();
    const std::string type = weapon_key();
    const int32_t datum = rs_read_datum(actor);
    s_rs_cur_datum = datum;
    if (actor != s_rs_seen_actor || type != s_rs_seen_type) {
        if (g_cfg.reload_state_log && actor != nullptr)
            API::get()->log_info("[Halo-CampE-UEVR] RSTATE in hand: type=%s stem=%s datum=0x%08X (%s actor) mag=%d rack=%d chamber=%d always=%d nomanual=%d",
                                 type.c_str(), weapon_stem_lc_for(type).c_str(), (unsigned)datum, type == s_rs_seen_type ? "a NEW" : "another",
                                 (int)(native_mag_mesh_impl() != nullptr), (int)weapon_in_list(g_cfg.slide_weapons), (int)weapon_in_list(g_cfg.slide_chamber_weapons),
                                 (int)weapon_in_list(g_cfg.slide_always_weapons), (int)weapon_in_list(g_cfg.reload_skip_weapons));
        s_rs_seen_actor = actor; s_rs_seen_type = type; s_rs_since = nowt;
    }
    if (type.empty()) return;   // nothing resolved this tick: the live state stays with its weapon
    // A different TYPE is a different weapon now, even while its datum is unreadable: the previous
    // weapon's state must not ride onto it for a single tick.
    if (!s_rs_live_type.empty() && type != s_rs_live_type) {
        rs_save("weapon left the hand", false);
        rs_live_fresh("weapon changed, its state is kept with it");
        s_rs_live_id.clear(); s_rs_live_datum = -1;
    }
    rs_hands_note(type);
    s_rs_live_type = type;
    // An instance in hand whose datum blinks unreadable (same type) is held, not re-keyed: a
    // type-name stand-in here would look like another weapon and release the magazine for a tick.
    if (g_cfg.reload_state_id != 1 && datum == -1 && s_rs_live_id.rfind("i:", 0) == 0) return;
    std::string id;
    if (g_cfg.reload_state_id == 1) id = "t:" + type;
    else if (datum != -1) { char b[24]; sprintf_s(b, "i:0x%08X", (unsigned)datum); id = b; }
    else if (nowt - s_rs_since >= ms_to_ticks(g_cfg.reload_state_wait_ms)) id = "t:" + type;
    if (!id.empty() && id == s_rs_live_id) {
        // Per type: the object behind a loaded record is proved once the datum reads (a first tick
        // can be unreadable). Nothing is ever forgotten for an unreadable datum.
        if (s_rs_live_datum == -1 && datum != -1) {
            s_rs_live_datum = datum;
            if (g_cfg.reload_state_id == 1 && s_rs_loaded_datum != -1 && datum != s_rs_loaded_datum) {
                const int32_t saved = s_rs_loaded_datum;
                s_rs_loaded_datum = -1;
                if (rs_other_object(type, saved, datum, s_rs_loaded_after_reset, s_rs_loaded_old_epoch, "late datum"))
                    rs_live_fresh("the loaded record belonged to another object");
            }
        }
        return;
    }
    if (id.empty()) return;
    // The type-name stand-in gives way to the instance once its datum reads (same weapon, late).
    const bool late_datum = !s_rs_live_id.empty() && s_rs_live_id.rfind("t:", 0) == 0 && id.rfind("i:", 0) == 0;
    if (late_datum) {
        if (RsRec* old = rs_find(s_rs_live_id)) *old = RsRec{};
    } else if (!s_rs_live_id.empty()) {
        // Same type, another instance: a second rifle, a pickup.
        rs_save("weapon left the hand", false);
        rs_live_fresh("another weapon of the same type, its state is kept with it");
    }
    RsRec live; rs_capture(live);
    const bool live_dirty = !rs_clean(live);
    s_rs_live_id = id; s_rs_live_datum = datum;
    s_rs_live_no_manual = weapon_in_list(g_cfg.reload_skip_weapons);
    const bool after_reset = s_rs_after_reset; s_rs_after_reset = false;
    s_rs_loaded_datum = -1;
    if (s_rs_recent[0] != id) { s_rs_recent[1] = s_rs_recent[0]; s_rs_recent[0] = id; }
    for (auto& c : s_rs_carry) if (c == id) c.clear();   // matched directly: its datum survived the load
    RsRec* r = rs_find(id);
    if (r == nullptr && g_cfg.reload_state_id == 3 && nowt < s_rs_carry_until) {
        for (auto& c : s_rs_carry) {
            if (c.empty()) continue;
            RsRec* cr = rs_find(c);
            if (cr == nullptr || cr->type != type) continue;
            if (g_cfg.reload_state_log)
                API::get()->log_info("[Halo-CampE-UEVR] RSTATE level carry: %s adopts the record of %s (type %s)", id.c_str(), c.c_str(), type.c_str());
            cr->id = id; cr->datum = datum; r = cr; c.clear();
            break;
        }
    }
    if (r != nullptr && s_rs_live_no_manual) {
        // A weapon with no manual reload never takes a record, whatever wrote one.
        rs_log("forget", "weapon has no manual reload, a record never applies", *r);
        *r = RsRec{}; r = nullptr;
    }
    if (r != nullptr && g_cfg.reload_state_id == 1 && r->datum != -1 && datum != -1 && r->datum != datum) {
        if (rs_other_object(type, r->datum, datum, after_reset, r->epoch != s_rs_epoch, "in hand")) r = nullptr;
    }
    if (r != nullptr) { s_rs_loaded_datum = r->datum; s_rs_loaded_after_reset = after_reset; s_rs_loaded_old_epoch = (r->epoch != s_rs_epoch); }
    if (live_dirty && !s_rs_live_no_manual) {
        // The player acted while this identity was pending: that is newer than any record.
        rs_save("identity resolved, the live state is kept", false);
        return;
    }
    if (r != nullptr) { const RsRec copy = *r; rs_live_load(copy, "weapon in hand again"); }
    else if (g_cfg.reload_state_log)
        API::get()->log_info("[Halo-CampE-UEVR] RSTATE fresh (no record) id=%s type=%s datum=0x%08X epoch=%u",
                             id.c_str(), type.c_str(), (unsigned)datum, (unsigned)s_rs_epoch);
}
// gesture_reset forces Idle (stick mode, calibration, a gate): the state goes to its weapon first.
void reload_state_on_reset() {
    if (g_cfg.reload_state_id == 0) return;
    const bool dead = g_dbg_persp.load(std::memory_order_relaxed) == 2;
    if (!s_rs_live_id.empty() || !s_rs_live_type.empty()) {
        rs_save(dead ? "gesture reset, death camera" : "gesture reset (stick mode, calibration or gate)", false);
        rs_live_fresh("gesture reset, state kept with its weapon");
        s_rs_live_id.clear(); s_rs_live_type.clear(); s_rs_live_datum = -1;
        s_rs_after_reset = true;
    }
    if (dead && g_cfg.reload_state_death == 1 && !s_rs_death_cleared) {
        // A death drops the guns; the respawn loadout starts with nothing in progress.
        rs_forget_all("death camera");
        s_rs_death_cleared = true;
    }
    if (!dead) s_rs_death_cleared = false;
}
void reload_state_mirror() {
    if (g_cfg.reload_state_id == 0 || g_cfg.reload_state_save != 1) return;
    rs_save("mirror", true);
}
// MANUAL RELOAD SWITCHED OFF. Every lock the reload keeps (the phantom round, the dry stop, the hidden
// reload, the lock-back waiting for the rack) is ended only by the gesture, and with reloadvr off the
// gesture never runs: the gun stayed dead, and with the per-weapon records the flag no longer even
// dropped on a swap. The game's own reload runs unmanaged while it is off, so no record or legacy
// memory may survive to hand a gun back "mag out" when the switch comes on again.
void reload_release_all(const char* why) {
    const bool coop = g_cfg.coop_auto && net_is_coop();
    if (s_true_empty && !coop && !reload_hidden_mode()) {
        if (auto* r = rounds_field()) { if (*r == 1) *r = 0; }   // the phantom round back to the honest count
    }
    if (s_sl_lock_pending) {
        if (auto* animbp = reload_weapon_anim_instance())
            if (auto* p = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame"))
                if (!IsBadWritePtr(p, sizeof(int32_t))) *p = slide_forward_frame();
    }
    if (s_reload != ReloadState::Idle) set_state(ReloadState::Idle, why);
    s_sl_chamber_left = 0; s_sl_empty_at_drop = false; s_sl_locked_back = false; s_sl_lock_pending = false;
    s_sl_reload_due = false; s_sl_pressed_early = false; s_sl_press_pending = false; s_sl_press_due_at = 0;
    s_sl_lock_frame = 0; s_sl_rack_done = false;
    s_true_empty = false; s_hide_display = false; s_coop_lock_rounds = 1;
    g_wristhud_hide_cradle.store(false, std::memory_order_relaxed);
    for (auto& m : s_wpn_mem) m = WpnMem{};
    rs_forget_all(why);
    s_rs_live_id.clear(); s_rs_live_type.clear(); s_rs_live_datum = -1; s_rs_cur_datum = -1;
    s_rs_seen_actor = nullptr; s_rs_seen_type.clear(); s_rs_after_reset = false;
    s_reload_weapon.clear();
    s_ph_rebase = true;
    if (g_cfg.reload_log || g_cfg.reload_state_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD released every lock and record (%s)", why);
}

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

    // ---- A WEAPON CHANGE CANCELS THE RELOAD IN FLIGHT. The gesture is a state machine and
    // nothing told it the weapon changed: pull the mag on gun A, swap to gun B, and the fire
    // suppression follows the PLAYER, not the gun -- the player could not shoot the fresh weapon
    // until he performed a gesture that belonged to the previous one. The magazine you pulled was
    // gun A's; gun B arrives in whatever state the game has it. So the reload records which
    // weapon's mag came out, and the moment a DIFFERENT non-empty weapon is in hand the state
    // returns to Idle. Non-empty on both sides on purpose: a transient empty key (weapon lowered
    // for a frame, swap animation) must not cancel a legitimate reload of the same gun.
    // reloadstate 1-3 replaces both legacy blocks below (per-weapon records, Config.hpp).
    if (g_cfg.reload_state_id != 0) reload_state_track();
    if (g_cfg.reload_state_id == 0 && s_reload != ReloadState::Idle) {
        const std::string wk = weapon_key();
        if (!wk.empty() && !s_reload_weapon.empty() && wk != s_reload_weapon) {
            // Remember the gun that left with its mag out, then cancel for the gun in hand.
            WpnMem* m = wpn_mem_slot(s_reload_weapon);
            m->key = s_reload_weapon; m->mag_out = true; m->chamber_left = s_sl_chamber_left;
            m->empty = s_sl_empty_at_drop; m->lock_pending = s_sl_lock_pending; m->true_empty = s_true_empty;
            if (g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD remembered %s: mag out, %d chambered", s_reload_weapon.c_str(), s_sl_chamber_left);
            s_sl_lock_pending = false; s_sl_reload_due = false; s_sl_pressed_early = false; s_sl_locked_back = false; s_sl_press_pending = false; s_sl_press_due_at = 0;
            set_state(ReloadState::Idle, "weapon changed, reload cancelled");
        }
    }
    // ...and a gun that comes back with its mag still out picks up where it left off.
    if (g_cfg.reload_state_id == 0) {
        static std::string s_last_key;
        const std::string wk = weapon_key();
        if (!wk.empty() && wk != s_last_key) {
            s_last_key = wk;
            if (s_reload == ReloadState::Idle) {
                if (WpnMem* m = wpn_mem_find(wk)) {
                    if (m->mag_out) {
                        s_reload_weapon = wk;
                        s_sl_chamber_left = m->chamber_left; s_sl_empty_at_drop = m->empty;
                        s_sl_lock_pending = m->lock_pending; s_true_empty = m->true_empty;
                        s_restoring_mag_out = true;
                        set_state(ReloadState::MagOut, "weapon returned with its mag out");
                        s_restoring_mag_out = false;
                    }
                    wpn_mem_clear(wk);
                }
            }
        }
    }

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
    // No head pose, no belt or well to measure against: the fetch hand counts as absent.
    bool have_left = (head_p != nullptr) && get_pose(lidx, &hand_l, &lrot, /*use_aim=*/false);
    // DEAD-POSE GATE. get_pose passes a sleeping/glitched controller as (0,0,0) -- the room origin,
    // which sits at the head on this rig. From there the well is within 30 cm and "lifted" reads
    // +60 cm, so ONE bad frame seats the magazine: logged 2026-08-16 12:32 as a seat 164 ms after
    // a grab that was 67 cm from the well. Same rule the palette publisher applies to the aim hand.
    if (have_left) {
        const Vec3& head = *head_p;
        const float rx = hand_l.x - head.x, ry = hand_l.y - head.y, rz = hand_l.z - head.z;
        const float reach2 = rx*rx + ry*ry + rz*rz;
        const bool zero = std::fabs(hand_l.x) < 1e-6f && std::fabs(hand_l.y) < 1e-6f && std::fabs(hand_l.z) < 1e-6f;
        if (zero || !std::isfinite(reach2) || reach2 > 1.5f * 1.5f) {
            have_left = false;
            if (g_cfg.reload_log) {
                static uint32_t s_dead = 0;
                if ((s_dead++ % 30u) == 0u)
                    API::get()->log_info("[Halo-CampE-UEVR] RELOAD: ignoring dead/implausible left pose (zero=%d reach=%.2fm)", (int)zero, std::sqrt(reach2));
            }
        }
    }

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
        if (!have_left || hand_r_p == nullptr) break;
        const Vec3& hand_r = *hand_r_p;
        const Vec3& head   = *head_p;
        // THE WELL. First choice: the weapon's OWN magazine component -- the one mag_hide_apply
        // just hid is the rendered magazine sitting in its well, so its live world location IS the
        // per-weapon insert point, on every weapon, with nothing to calibrate. The seat test then
        // runs in world space (the hand goes room->world through the same transform that places
        // every holster marker). Fallback when no component resolved: the old fixed point
        // reload_well_fwd along the aim direction from the aim hand. And the LIFT GATE either
        // way: the mag must have risen since it was grabbed. Without both, the log
        // (2026-08-16 12:07) shows the reload firing 214-224 ms after the belt grab, at the hip.
        float join = 1e9f;
        Vec3  well_world{}; bool have_well_world = false;
        if (auto* mc = s_mag_hidden.get()) {
            if (call_ret_vec3(mc, L"K2_GetComponentLocation", &well_world)) {
                have_well_world = true;
                well_world = reload_well_stabilize(mc, well_world, head);
                const Vec3 hlw = reload_hand_world(hand_l, head);
                const float wdx = hlw.x - well_world.x, wdy = hlw.y - well_world.y,
                            wdz = hlw.z - well_world.z;
                join = std::sqrt(wdx * wdx + wdy * wdy + wdz * wdz) / 100.0f;  // cm -> m
            }
        }
        if (!have_well_world) {
            Vec3 well = hand_r;
            const auto ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                                  : API::VR::get_right_controller_index();
            Vec3 apos{}; Quat aq{};
            if (ridx >= 0 && get_pose(ridx, &apos, &aq, /*use_aim=*/true)) {
                const Vec3 f = quat_forward(apply_aim_fix(aq));
                well = Vec3{hand_r.x + f.x * g_cfg.reload_well_fwd,
                            hand_r.y + f.y * g_cfg.reload_well_fwd,
                            hand_r.z + f.z * g_cfg.reload_well_fwd};
            }
            const float ddx = hand_l.x - well.x, ddy = hand_l.y - well.y, ddz = hand_l.z - well.z;
            join = std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
            well_world = holster_room_to_world(well, head);
        }
        reload_well_marker_update(true, well_world);
        const bool lifted = (hand_l.y - s_grab_y) >= g_cfg.reload_lift;
        if (g_cfg.reload_log) {
            static uint32_t s_rl = 0;
            if ((s_rl++ % 15u) == 0u)
                API::get()->log_info("[Halo-CampE-UEVR] RELOAD held: mag-to-well=%.0fcm lifted=%.0fcm (need <=%.0f, >=%.0f)",
                                     join * 100.0f, (hand_l.y - s_grab_y) * 100.0f,
                                     g_cfg.reload_join_dist * 100.0f, g_cfg.reload_lift * 100.0f);
        }
        // THE SLIDE, in place of the old four-tick debounce. Inside the capture radius (and
        // lifted) the magazine leaves the hand and travels into the well over reload_slide_ms;
        // the reload fires when it lands. Pulling the hand well clear mid-slide aborts it back to
        // the hand. The duration IS the debounce: a one-frame tracking glitch cannot complete it.
        {
            const long long nowt = now_ticks();
            // Target transform: the weapon's magazine component when we have it (its rotation
            // is the seated mag's, so the slide ends exactly on the real one); else the fallback
            // well point with no orientation, which slides position only.
            g_reload_slide_x.store(well_world.x, std::memory_order_relaxed);
            g_reload_slide_y.store(well_world.y, std::memory_order_relaxed);
            g_reload_slide_z.store(well_world.z, std::memory_order_relaxed);
            bool rot_ok = false;
            if (have_well_world) {
                if (auto* mc = s_mag_hidden.get()) {
                    Vec3 wrot{};
                    if (call_ret_vec3(mc, L"K2_GetComponentRotation", &wrot)) {
                        g_reload_slide_pitch.store(wrot.x, std::memory_order_relaxed);
                        g_reload_slide_yaw.store(wrot.y,   std::memory_order_relaxed);
                        g_reload_slide_roll.store(wrot.z,  std::memory_order_relaxed);
                        rot_ok = true;
                    }
                }
            }
            g_reload_slide_rot_valid.store(rot_ok, std::memory_order_relaxed);

            // THE WELL'S MOUTH (insert mode). The seat point is the seated mag's own centre,
            // inside the grip, so a radius around it only fires when the mag is nearly home.
            // The lock instead happens anywhere on the axis from reload_insert below the seat up
            // to the seat, within reload_join_dist of the axis SIDEWAYS; the abort is sideways too.
            bool  axis_ok = false; float lateral_m = join, d_axis_cm = 0.0f;
            if (g_cfg.reload_insert_mode == 1 && have_well_world && rot_ok) {
                const Quat qs = rotator_to_quat(g_reload_slide_pitch.load(std::memory_order_relaxed),
                                                g_reload_slide_yaw.load(std::memory_order_relaxed),
                                                g_reload_slide_roll.load(std::memory_order_relaxed));
                Vec3 up = quat_rotate(qs, Vec3{0.0f, 0.0f, 1.0f});
                up = Vec3{up.x * g_cfg.reload_insert_sign, up.y * g_cfg.reload_insert_sign, up.z * g_cfg.reload_insert_sign};
                const Vec3 hw = reload_hand_world(hand_l, head);
                const Vec3 r{hw.x - well_world.x, hw.y - well_world.y, hw.z - well_world.z};
                d_axis_cm = r.x * up.x + r.y * up.y + r.z * up.z;
                const Vec3 lat{r.x - up.x * d_axis_cm, r.y - up.y * d_axis_cm, r.z - up.z * d_axis_cm};
                lateral_m = std::sqrt(lat.x * lat.x + lat.y * lat.y + lat.z * lat.z) / 100.0f;
                axis_ok = true;
            }
            const float jd = g_cfg.reload_join_dist + reload_gate_pad_m();
            const bool in_mouth = axis_ok
                ? (lateral_m <= jd && d_axis_cm >= -(reload_insert_for_weapon() + jd) * 100.0f && d_axis_cm <= jd * 100.0f)
                : (join <= jd);
            const bool pulled_clear = axis_ok ? (lateral_m > jd * 2.5f || d_axis_cm < -(reload_insert_for_weapon() + jd * 2.5f) * 100.0f)
                                             : (join > jd * 2.5f);

            if (s_slide_start == 0) {
                if (in_mouth && lifted) {
                    s_slide_start = nowt;
                    g_reload_slide_t.store(0.0f, std::memory_order_relaxed);
                    ak_step_sound("seat");
                    if (g_cfg.reload_log)
                        API::get()->log_info("[Halo-CampE-UEVR] RELOAD slide begins at %.0fcm", join * 100.0f);
                }
            } else if (pulled_clear) {
                if (g_cfg.reload_log)
                    API::get()->log_info("[Halo-CampE-UEVR] RELOAD slide aborted, hand pulled clear (%.0fcm)", join * 100.0f);
                reload_slide_reset();
            } else {
                const float ms = (float)std::chrono::duration_cast<std::chrono::milliseconds>(
                    clock_t_::duration(nowt - s_slide_start)).count();
                bool done = false;
                if (g_cfg.reload_insert_mode == 1 && have_well_world && rot_ok) {
                    // HAND-DRIVEN. The mag snaps onto the well's axis (the short blend), then sits
                    // on that axis at the HAND's height below the seat; it is home when the hand
                    // has pushed it within reload_insert_done of the seat.
                    const float snap = std::fmin(1.0f, ms / (float)std::fmax(1, g_cfg.reload_slide_ms));
                    g_reload_slide_t.store(snap, std::memory_order_relaxed);
                    const Quat qs = rotator_to_quat(g_reload_slide_pitch.load(std::memory_order_relaxed),
                                                    g_reload_slide_yaw.load(std::memory_order_relaxed),
                                                    g_reload_slide_roll.load(std::memory_order_relaxed));
                    Vec3 up = quat_rotate(qs, Vec3{0.0f, 0.0f, 1.0f});
                    up = Vec3{up.x * g_cfg.reload_insert_sign, up.y * g_cfg.reload_insert_sign, up.z * g_cfg.reload_insert_sign};
                    const Vec3 hw = reload_hand_world(hand_l, head);
                    const float d = (hw.x - well_world.x) * up.x + (hw.y - well_world.y) * up.y + (hw.z - well_world.z) * up.z;   // cm along the axis, negative = below the seat
                    const float travel = reload_insert_for_weapon() * 100.0f;
                    float along = d; if (along > 0.0f) along = 0.0f; if (along < -travel) along = -travel;
                    g_reload_slide_x.store(well_world.x + up.x * along, std::memory_order_relaxed);
                    g_reload_slide_y.store(well_world.y + up.y * along, std::memory_order_relaxed);
                    g_reload_slide_z.store(well_world.z + up.z * along, std::memory_order_relaxed);
                    done = (d >= -g_cfg.reload_insert_done * 100.0f);
                    if (g_cfg.reload_log) {
                        static uint32_t s_il = 0;
                        if ((s_il++ % 15u) == 0u) API::get()->log_info("[Halo-CampE-UEVR] RELOAD insert: %.1f cm below the seat (home within %.1f)", -d, g_cfg.reload_insert_done * 100.0f);
                    }
                } else {
                    const float t = std::fmin(1.0f, ms / (float)std::fmax(1, g_cfg.reload_slide_ms));
                    g_reload_slide_t.store(t, std::memory_order_relaxed);
                    done = (t >= 1.0f);
                }
                if (done) {
                    reload_slide_reset();
                    // Locked back until racked ONLY if the gun was empty when the mag left --
                    // a chambered round needs no rack, that is how a pistol works.
                    const bool rack_first = g_cfg.slide_vr && g_cfg.slide_lock_reload && slide_weapon_ok() && slide_chamber_ok() && slide_rack_available()
                                            && (s_sl_empty_at_drop || weapon_in_list(g_cfg.slide_always_weapons));
                    if (rack_first) {
                        s_sl_lock_pending = true; s_sl_lock_frame = 0; s_sl_rack_done = false;
                    } else {
                        s_sl_locked_back = false;   // no rack will clear it: a stale flag kept the record alive and the rack zone live
                    }
                    if (rack_first && g_cfg.slide_chamber) {
                        // SLIDECHAMBER: the mag is in, the chamber is not. The rack ends the reload.
                        if (s_sl_press_pending || s_sl_press_due_at != 0) { s_sl_press_pending = false; s_sl_press_due_at = 0; s_sl_pressed_early = true; reload_press_now("seat (waited)"); }
                        s_sl_reload_due = true;
                        if (g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD seated on an empty gun: waiting for the rack");
                    } else if (s_sl_pressed_early) {
                        s_sl_pressed_early = false;
                        if (g_cfg.reload_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD seated, the sim reloaded at the drop");
                    } else {
                        s_sl_press_pending = false; s_sl_press_due_at = 0;
                        reload_press_now("seat");
                    }
                    if (g_cfg.anim_vars) anim_vars_begin();
                    audio_dump_begin();
                    ak_dump("seat");
                    ak_step_sound("seated");
                    if (g_cfg.anim_dump) {
                        s_anim_dump_until = nowt + ms_to_ticks(2500);
                        API::get()->log_info("[Halo-CampE-UEVR] ANIMDUMP begin (seat)");
                    }
                    set_state(ReloadState::Idle, "magazine seated, reload fired");
                }
            }
        }
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

// ---- OFF-HAND MELEE (meleeleft). A punch does not care which hand throws it. Same three tests
// and thresholds as the aim hand, own state, SHARED cooldown so the two detectors cannot
// double-fire one press. What differs is what the off hand does all day -- fetch magazines, pull
// grenades, brace the weapon -- each a fast, extending reach, so each gets an explicit
// stand-down here rather than a threshold tweak.
void offhand_melee_update(float dt) {
    static Vec3      s2_prev_rel{};
    static float     s2_prev_reach = 0.0f;
    static bool      s2_have = false;
    static Vec3      s2_vel{};
    static float     s2_ext = 0.0f;
    static long long s2_last = 0;
    static bool      s2_in_swing = false;
    static float     s2_pk_spd = 0.0f, s2_pk_ext = 0.0f, s2_pk_reach = 0.0f;
    static Vec3      s2_rel0{};          // hand-rel-head where this swing began
    static float     s2_pk_disp = 0.0f;  // furthest it has travelled from there

    if (!g_cfg.melee_left) { s2_have = false; return; }

    // The gates above this call stop the whole tick (stick mode, calibration), so a gap in our
    // own run cadence means one of them was engaged -- reseed instead of differentiating across it.
    const long long nowt = now_ticks();
    if (s2_last != 0 && nowt - s2_last > ms_to_ticks(250)) s2_have = false;
    s2_last = nowt;

    const auto idx = g_cfg.aim_left_hand ? API::VR::get_right_controller_index()
                                         : API::VR::get_left_controller_index();
    Vec3 pos{}; Quat rot{};
    Vec3 hpos{}; Quat hrot{};
    if (!get_pose(idx, &pos, &rot, /*use_aim=*/false) ||
        !get_pose(API::VR::get_hmd_index(), &hpos, &hrot, /*use_aim=*/false)) {
        s2_have = false;
        return;
    }

    const Vec3  rel{pos.x - hpos.x, pos.y - hpos.y, pos.z - hpos.z};
    const float reach = std::sqrt(rel.x * rel.x + rel.y * rel.y + rel.z * rel.z);
    if (reach > g_cfg.melee_max_reach) {
        s2_have = false;
        s2_vel = Vec3{0.0f, 0.0f, 0.0f};
        s2_ext = 0.0f;
        return;
    }
    if (!s2_have) {
        s2_prev_rel = rel;
        s2_prev_reach = reach;
        s2_have = true;
        return;
    }

    const Vec3 raw{(rel.x - s2_prev_rel.x) / dt,
                   (rel.y - s2_prev_rel.y) / dt,
                   (rel.z - s2_prev_rel.z) / dt};
    const float ext_raw = (reach - s2_prev_reach) / dt;
    s2_prev_rel = rel;
    s2_prev_reach = reach;

    const float a = ema_alpha(g_cfg.melee_tau_ms, dt);
    s2_vel.x += (raw.x - s2_vel.x) * a;
    s2_vel.y += (raw.y - s2_vel.y) * a;
    s2_vel.z += (raw.z - s2_vel.z) * a;
    s2_ext   += (ext_raw - s2_ext) * a;

    const float speed = std::sqrt(s2_vel.x * s2_vel.x + s2_vel.y * s2_vel.y + s2_vel.z * s2_vel.z);
    if (speed > g_cfg.melee_max_speed) {
        s2_have = false;
        s2_vel = Vec3{0.0f, 0.0f, 0.0f};
        s2_ext = 0.0f;
        return;
    }

    // Swing segmentation, same shape as the main hand's: peaks over one continuous motion,
    // reported when the hand settles, so a swing that never fires still leaves its numbers.
    float disp = 0.0f;
    if (speed > REST_SPEED_MPS) {
        if (!s2_in_swing) s2_rel0 = rel;
        s2_in_swing = true;
        const float ddx = rel.x - s2_rel0.x, ddy = rel.y - s2_rel0.y, ddz = rel.z - s2_rel0.z;
        disp = std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
        if (speed  > s2_pk_spd)   s2_pk_spd   = speed;
        if (s2_ext > s2_pk_ext)   s2_pk_ext   = s2_ext;
        if (reach  > s2_pk_reach) s2_pk_reach = reach;
        if (disp   > s2_pk_disp)  s2_pk_disp  = disp;
    } else if (s2_in_swing) {
        s2_in_swing = false;
        if (g_cfg.melee_log) {
            const bool would = (s2_pk_spd   >= g_cfg.melee_speed) &&
                               (s2_pk_ext   >= g_cfg.melee_ext ||
                                (g_cfg.melee_disp > 0.0f && s2_pk_disp >= g_cfg.melee_disp)) &&
                               (s2_pk_reach >= g_cfg.melee_reach);
            API::get()->log_info(
                "[Halo-CampE-UEVR] MELEE swing (OFF HAND)  speed=%.2f  ext=%.2f  reach=%.2f  disp=%.2f   "
                "(need spd>=%.2f ext>=%.2f reach>=%.2f) -- %s",
                s2_pk_spd, s2_pk_ext, s2_pk_reach, s2_pk_disp,
                g_cfg.melee_speed, g_cfg.melee_ext, g_cfg.melee_reach,
                would ? "FIRED" : "no");
        }
        s2_pk_spd = 0.0f; s2_pk_ext = 0.0f; s2_pk_reach = 0.0f; s2_pk_disp = 0.0f;
    }

    if (nowt < s_cooldown_until)   return;
    if (speed < g_cfg.melee_speed) return;
    // Extension OR travel. A vertical chop arcs around the shoulder: the hand-to-head distance
    // barely grows (measured 2026-08-31: ext 1.43 and 1.80 against the 2.20 gate) while the hand
    // itself travels over a metre (disp 1.13, 0.77). Ambient jitter and the gunstock's kick both
    // stay under 0.1 m, so travel separates a chop from noise as cleanly as extension separates
    // a punch from it.
    const bool travelled = g_cfg.melee_disp > 0.0f && disp >= g_cfg.melee_disp;
    if (s2_ext < g_cfg.melee_ext && !travelled) return;
    if (reach < g_cfg.melee_reach) return;

    // GUNSTOCK KICK vs PUNCH. While the trigger is down (+ a tail) with the ForceTube on, the
    // stock's kick jolts the off hand into threshold-clearing VELOCITY (2.31 and 2.46 against the
    // 2.20 gate, measured) without the hand actually going anywhere. A real punch TRAVELS. So the
    // shot window demands displacement since the swing began -- not a harder swing, which is how
    // controllers meet door frames.
    if (g_cfg.force_tube && g_cfg.melee_shot_ms > 0 &&
        nowt - g_ft_fire_at.load(std::memory_order_relaxed) < ms_to_ticks(g_cfg.melee_shot_ms) &&
        disp < g_cfg.melee_shot_dist) return;

    // ---- THE OFF HAND'S DAY JOBS, each a hard stand-down. A swing that cleared every numeric
    // gate and dies here is invisible without a name, so each one says so in the log.
    const char* job = nullptr;
    if      (s_reload != ReloadState::Idle)                   job = "reload in progress";
    else if (two_hand_latched())    job = "two-hand brace";
    else if (holster_offhand_busy())                          job = "grenade in pouch/hand";
    else if (holster_offhand_melee_veto())                    job = "holster veto";
    if (job != nullptr) {
        if (g_cfg.melee_log) {
            API::get()->log_info("[Halo-CampE-UEVR] MELEE (OFF HAND) stood down by %s: "
                                 "speed=%.2f ext=%.2f reach=%.2f",
                                 job, speed, s2_ext, reach);
        }
        if (holster_offhand_melee_veto()) s_cooldown_until = nowt + ms_to_ticks(150);
        return;
    }

    g_melee_hold_until.store(nowt + ms_to_ticks(g_cfg.melee_hold_ms), std::memory_order_relaxed);
    s_cooldown_until = nowt + ms_to_ticks(g_cfg.melee_cooldown_ms);

    // Aim hold along the punch, mode 1 only -- mode 0 wants the swing-start aim, which this
    // detector does not track; the shipped mode is 1.
    if (g_cfg.melee_aim_mode == 1 && g_cfg.melee_aim_hold_ms > 0) {
        const float hy = wrap180(std::atan2(s2_vel.x, -s2_vel.z) * RAD2DEG
                                 + g_cfg.aim_turn * g_turn_offset.load(std::memory_order_relaxed));
        const float hp = std::asin(std::fmax(-1.0f, std::fmin(1.0f, s2_vel.y / speed))) * RAD2DEG;
        g_melee_aim_ctrl_yaw.store(hy, std::memory_order_relaxed);
        g_melee_aim_ctrl_pitch.store(hp, std::memory_order_relaxed);
        g_melee_aim_hold_until.store(nowt + ms_to_ticks(g_cfg.melee_aim_hold_ms),
                                     std::memory_order_relaxed);
    }
    if (g_cfg.melee_log) {
        API::get()->log_info("[Halo-CampE-UEVR] MELEE FIRED (OFF HAND): speed=%.2f ext=%.2f reach=%.2f",
                             speed, s2_ext, reach);
    }
}

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
    // The rack, the pump, the chamber and the rest of the reload's own ticks ride with it, not with
    // melee: switching melee off must not switch the slide off. They keep their pose gate.
    if (poses_ok && fork_reload) {
        slide_update(hpos);
        slide_montage_tick();
        anim_state_probe_tick();
        slide_fire_tick();
        slide_phantom_tick();
        slide_chamber_tick();
        slide_copy_tick();
        slide_part_tick();
        slide_node_write_tick();
    }
    // Per-weapon reload state: re-hide a magazine that is out on whichever actor renders the weapon
    // now, and mirror the live state into its record. Before the melee-off return, so switching melee
    // off never stops the reload state from saving.
    if (fork_reload) {
        mag_hide_enforce();
        reload_state_mirror();
    }

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
