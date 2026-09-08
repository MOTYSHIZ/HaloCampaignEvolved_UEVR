#include "Holster.hpp"

#include "Config.hpp"
#include "Gesture.hpp"            // reload_state(): the visible magazine renders that machine
#include "Math.hpp"
#include "MotionAimControl.hpp"
#include "Rig.hpp"
#include "BlamDrive.hpp"
#include "MotionAimControl.hpp"   // the throw aim hold
#include "UeObject.hpp"
#include "Markers.hpp"
#include "WeaponCalib.hpp"        // weapon_key(): which weapon's magazine to render

#include <chrono>
#include <cmath>
#include <cwctype>
#include <string>
#include <vector>

using uevr::API;

namespace halo {

std::atomic<long long> g_holster_swap_until{0};
std::atomic<long long> g_holster_throw_until{0};
std::atomic<long long> g_holster_gswitch_until{0};

namespace {

using clock_t_ = std::chrono::steady_clock;
inline long long now_ticks() { return clock_t_::now().time_since_epoch().count(); }
inline long long ms_to_ticks(int ms) {
    return std::chrono::duration_cast<clock_t_::duration>(std::chrono::milliseconds(ms)).count();
}

// Where A (the game's CURRENT weapon) and B (its backup) live; None = in the hand.
HolsterSlot s_slot_a = HolsterSlot::None;
HolsterSlot s_slot_b = HolsterSlot::RightShoulder;
bool  s_unarmed = false;                              // hand empty: hide + swallow fire
int   s_unhide_ticks = 0;                             // re-assert SetActorHiddenInGame(false) briefly after a draw
HolsterSlot s_in_zone = HolsterSlot::None;           // hand's current zone (for haptics / log)
bool  s_grip_prev = false;
bool  s_grenade_armed = false;
Vec3  s_prev_hand{}; bool s_have_prev = false;
Vec3  s_hand_vel{};                                  // head-relative, m/s, ~60 ms EMA
// ---- THE THROW IS JUDGED ON THE PEAK OF THE SWING, NOT ON THE FRAME YOU LET GO.
//
// Measured 2026-08-24, 10 throws and 1 deliberate put-back. The release-instant speed the gate
// used to read was a median of 0.72 of the swing's own peak, and the ratio got WORSE the harder
// the throw: peak 2.90 -> 0.99 of it survived, peak 4.85 -> 0.29. So the hardest throws were the
// closest to failing, which is why "throw it again, harder" made it worse and why it could not be
// reproduced on demand: it turns on the phase between your grip release and the peak, which is
// not something a person can aim at.
//
// Four of the ten cleared 1.20 m/s by less than 1.35x. The peak clears it by 1.7x at worst.
// The separation against a real put-back is not close: every throw peaked at 2.04 or above, the
// put-back peaked at 0.16. Thirteen times the gap, so no threshold in this range can confuse them.
//
// Stale frames were 0 of 32, 0 of 54, 0 of 16 ... zero in every sample. The EMA is fed properly
// and sampling aliasing is NOT the fault here. Measured, not assumed, and now ruled out.
float s_peak_fwd = 0.0f, s_peak_spd = 0.0f;          // windowed peak of the SAME EMA the gate reads
long long s_peak_at = 0;                             // when that peak was set (decay window)
long long s_arm_at = 0;                              // grip press that armed the grenade
int   s_stale_frames = 0, s_total_frames = 0;        // identical controller pose = a frame the
Vec3  s_prev_raw{};                                  // difference cannot see (sampling aliasing)
bool  s_have_raw = false;
long long s_last_swap = 0;                           // debounce: one swap per grip press, min gap
long long s_last_action = 0;                         // any holster action (melee veto window)
bool  s_near_zone = false;                           // hand within radius+margin of any zone
float s_nearest_dist = 1e9f;                         // ...and how far the nearest one actually is
// Grenade type: READ from the unit object (BlamDrive publishes unit+0x380/+0x382/+0x383), with a
// belief fallback only while that read is not live. The game auto-switches when a type runs out,
// which is exactly what a belief alone would lose.
int   s_gtype = 0;
int   s_gswitch_pending = 0;   // ticks left waiting for the game to flip after our press

// ---- GRENADE VISUALS: two pouch spheres + one on the hand while armed. TrackedObject because a
// level load recycles the components; a failed spawn retries every ~2 s rather than latching dead.
TrackedObject s_pouch_l, s_pouch_r, s_hand_g;
uint32_t s_mk_tick = 0;
// THE MARKER MESHES ARE THE GAME'S OWN GRENADE MODELS, resolved from the loaded-object list.
// The engine sphere proved unreliable on foot -- find_uobject cannot load, and this level had no
// BasicShapes in memory, so three meshless "spheres" rendered as nothing (2026-08-24, in-headset
// and in the spawn log, which said so at the time). Grenade meshes have the opposite property:
// they are loaded exactly where grenades exist, which is exactly where the visuals matter. The
// walk names ~290k objects, so it is PACED and stops for good once both meshes are found.
TrackedObject s_mesh_frag, s_mesh_plasma;
bool s_mesh_logged = false;

// ---- THE VISIBLE MAGAZINE (reloadmag, doctrine in Config.hpp). One marker component: parked on
// the belt point while the reload is in MAG_OUT, riding the fetch hand in MAG_HELD, hidden
// otherwise. The mesh hunt shares the grenade walk below; whether this game HAS a standalone
// magazine StaticMesh was never verified (the mag may be geometry inside the weapon mesh), so the
// walk LOGS every magazine-ish name it meets -- that log is the asset survey, and the frag mesh
// stands in until it says otherwise.
TrackedObject s_mag_marker, s_mesh_mag;
int  s_mag_pick = 0;                 // 0 none, 3 "magazine", 2 "clip", 1 "ammo" -- higher wins
bool s_mag_search_done = false;      // the survey ran to completion once; do not rewalk forever
bool s_mag_zone_prev = false;        // fetch hand inside the mag zone (haptic edge)
std::atomic<bool> s_mag_hand_in{false};   // ...published for Gesture's grab test (prior tick)

// ---- PER-WEAPON MAGS. The 2026-08-27 survey settled the old blocking unknown: this game SHIPS a
// standalone magazine StaticMesh per weapon (SM_Magnum_Magazine_Default, SM_BattleRifle_Magazine_
// M_Default, ...) plus SM_ammo_pickup_* for the ones that load shells instead. So the survey now
// KEEPS every hit, and the marker's mesh is re-picked whenever the held weapon changes: weapon_key
// ("FP_Magnum") minus its FP_ prefix, lowercased, matched into the candidate names -- the same
// substring convention the wpnoff table uses. "magazine" hits outrank "ammo pickup" hits so the
// magnum gets its mag, not its pickup box; no match falls back to the global pick, then the frag.
struct MagCand { std::wstring lname; TrackedObject obj; int rank = 0; };
std::vector<MagCand> s_mag_cands;
std::string s_mag_mesh_key = "\x01";   // weapon key the marker's mesh matches; sentinel = never set

API::UObject* mag_mesh_for_weapon(const std::string& wkey, int* out_rank) {
    std::wstring tok;
    {
        std::string k = wkey;
        if (k.rfind("FP_", 0) == 0) k.erase(0, 3);
        for (char ch : k) tok.push_back((wchar_t)towlower((unsigned char)ch));
    }
    API::UObject* best = nullptr; int best_rank = 0;
    if (!tok.empty()) {
        for (auto& cnd : s_mag_cands) {
            auto* p = cnd.obj.get();
            if (p == nullptr || cnd.rank <= best_rank) continue;
            if (cnd.lname.find(tok) != std::wstring::npos) { best = p; best_rank = cnd.rank; }
        }
    }
    if (out_rank != nullptr) *out_rank = best_rank;
    if (best == nullptr) best = s_mesh_mag.get();
    if (best == nullptr) best = s_mesh_frag.get();
    return best;
}

void resolve_grenade_meshes() {
    const bool want_mag = g_cfg.reload_mag != 0 && !s_mag_search_done;
    if (s_mesh_frag.get() != nullptr && s_mesh_plasma.get() != nullptr && !want_mag) return;
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;
    auto contains_ci = [](const std::wstring& hay, const wchar_t* needle) {
        std::wstring h; h.reserve(hay.size());
        for (wchar_t ch : hay) h.push_back((wchar_t)towlower(ch));
        return h.find(needle) != std::wstring::npos;
    };
    const int32_t nn = arr->get_object_count();
    int mag_lines = 0;
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"StaticMesh") continue;
        const std::wstring full = o->get_full_name();
        if (contains_ci(full, L"grenade")) {
            if (contains_ci(full, L"plasma")) {
                if (s_mesh_plasma.get() == nullptr) s_mesh_plasma.set_at(o, i);
            } else if (s_mesh_frag.get() == nullptr) {
                s_mesh_frag.set_at(o, i);
            }
        }
        // The magazine hunt. "magazine" outranks "clip" outranks "ammo" -- the broader the term,
        // the likelier it names a crate or a pickup rather than the mag itself. Every hit is
        // LOGGED (capped): this survey is the ground truth the per-weapon-mag step was blocked on,
        // and it must print whether or not the pick is any good.
        if (want_mag) {
            std::wstring low; low.reserve(full.size());
            for (wchar_t ch : full) low.push_back((wchar_t)towlower(ch));
            int rank = 0;
            if      (low.find(L"magazine") != std::wstring::npos) rank = 3;
            else if (low.find(L"clip")     != std::wstring::npos) rank = 2;
            else if (low.find(L"ammo")     != std::wstring::npos) rank = 1;
            if (rank > 0) {
                if (mag_lines < 40) {
                    ++mag_lines;
                    API::get()->log_info("[Halo-CampE-UEVR] MAG survey: %ls", full.c_str());
                }
                if (rank > s_mag_pick) { s_mag_pick = rank; s_mesh_mag.set_at(o, i); }
                // Keep the hit for the per-weapon pick. 32 covers the 17 the survey logged with
                // room to spare; past that the extras were crates and duplicates anyway.
                if (s_mag_cands.size() < 32) {
                    MagCand cnd; cnd.lname = std::move(low); cnd.rank = rank; cnd.obj.set_at(o, i);
                    s_mag_cands.push_back(std::move(cnd));
                }
            }
        }
        if (s_mesh_frag.get() != nullptr && s_mesh_plasma.get() != nullptr && !want_mag) break;
    }
    if (want_mag) {
        // The walk covered the whole array: the survey is complete for this level, whatever it
        // found. A retry every 2 s forever would rebuild ~290k names for nothing.
        s_mag_search_done = true;
        auto* mm = s_mesh_mag.get();
        API::get()->log_info("[Halo-CampE-UEVR] MAG survey done: %d candidates logged, using %ls",
                             mag_lines,
                             mm != nullptr ? mm->get_full_name().c_str()
                                           : L"(none -- frag grenade stands in)");
    }
    // One side found and not the other: the found one stands in for both rather than leaving a
    // pouch invisible. Logged so the substitution is visible, not silent.
    if (!s_mesh_logged && (s_mesh_frag.get() != nullptr || s_mesh_plasma.get() != nullptr)) {
        s_mesh_logged = true;
        auto* f = s_mesh_frag.get(); auto* pl = s_mesh_plasma.get();
        API::get()->log_info("[Halo-CampE-UEVR] HOLSTER marker meshes: frag=%ls plasma=%ls",
                             f != nullptr ? f->get_full_name().c_str() : L"(none, using plasma)",
                             pl != nullptr ? pl->get_full_name().c_str() : L"(none, using frag)");
    }
}
// World-frame (VR axes) hand velocity, head motion subtracted -- the head-frame EMA above serves
// the throw GATE, this one serves the throw DIRECTION, which must live in the same frame the aim
// extraction reads (atan2(x,-z)/asin(y), the derive_ctrl_angles convention).
Vec3 s_hand_vel_w{}; Vec3 s_prev_rel_w{};
Vec3 s_peak_velw{};                                  // world velocity AT the fwd peak, the throw's own moment

// ---- THE GRENADE HAND'S OWN TRACK (holster_gren_hand=1: the off hand). Identical machinery to
// the aim hand's above -- position, head-frame and world-frame velocity EMAs, the 250 ms peak
// hold -- because the throw gate and direction must be measured on the hand actually throwing.
// With holster_gren_hand=0 the grenade hand IS the aim hand and this track simply mirrors it.
namespace {
Vec3  s_gprev_hand{}; bool s_ghave_prev = false;
Vec3  s_ghand_vel{};
Vec3  s_ghand_vel_w{}; Vec3 s_gprev_rel_w{};
Vec3  s_gpeak_velw{};
float s_gpeak_fwd = 0.0f, s_gpeak_spd = 0.0f;
long long s_gpeak_at = 0;
Vec3  s_gprev_raw{}; bool s_ghave_raw = false;
int   s_gstale_frames = 0, s_gtotal_frames = 0;
bool  s_ggrip_prev = false;
HolsterSlot s_gin_zone = HolsterSlot::None;          // pouch the OFF hand is in (haptic edge)
HolsterSlot s_pin_zone = HolsterSlot::None;          // pouch the AIM hand is in (haptic edge)
bool  s_carry_off = false;                           // which hand holds the armed grenade
}

// Body yaw state for the torso leash. Radians, VR frame, same convention as the head yaw below.
float s_body_yaw = 0.0f;
bool  s_body_init = false;

void markers_hide_all() {
    holster_marker_show(s_pouch_l.get(), false);
    holster_marker_show(s_pouch_r.get(), false);
    holster_marker_show(s_hand_g.get(), false);
    holster_marker_show(s_mag_marker.get(), false);
    s_mag_hand_in.store(false, std::memory_order_relaxed);
    s_mag_zone_prev = false;
}

const char* slot_name(HolsterSlot s) {
    switch (s) {
        case HolsterSlot::RightShoulder: return "right shoulder";
        case HolsterSlot::LeftShoulder:  return "left shoulder";
        case HolsterSlot::RightHip:      return "right hip";
        case HolsterSlot::LeftChest:     return "left chest (frag)";
        case HolsterSlot::RightChest:    return "right chest (plasma)";
        default: return "none";
    }
}

// The AIM hand is the right unless the player aims left-handed; the OFF hand is the other one.
// Which of them may work the pouches is holster_gren_hand: 0 = aim only, 1 = off only,
// 2 = both, with the carrying hand deciding the armed behaviour.
bool aim_is_right() { return !g_cfg.aim_left_hand; }
bool off_is_right() { return g_cfg.aim_left_hand; }
// holster_grenades gates BOTH, so the pouches can be switched off without losing the
// over-the-shoulder weapon swap. Checked here rather than at the call sites because every path into
// the pouches -- grab, arm, throw, and the pouch markers -- already asks one of these two, so a gate
// here cannot be walked around by a path someone adds later.
bool aim_can_grab() { return g_cfg.holster_grenades && g_cfg.holster_gren_hand != 1; }
bool off_can_grab() { return g_cfg.holster_grenades && g_cfg.holster_gren_hand != 0; }

void haptic_on(bool right, float dur, float amp) {
    if (!g_cfg.holster_haptic) return;
    // ABI order: (seconds_from_now, duration, frequency, amplitude, source) -- see TwoHand.cpp.
    API::VR::trigger_haptic_vibration(0.0f, dur, 0.0f, amp,
                                      right ? API::VR::get_right_joystick_source()
                                            : API::VR::get_left_joystick_source());
}
void haptic(float dur, float amp)  { haptic_on(aim_is_right(), dur, amp); }
void ghaptic(float dur, float amp) { haptic_on(off_is_right(), dur, amp); }

void set_weapon_hidden(bool hidden) {
    auto* w = fp_weapon_actor();
    if (w == nullptr) return;
    alignas(16) uint8_t p[64] = {0};
    p[0] = hidden ? 1 : 0;             // SetActorHiddenInGame(bool bNewHidden)
    w->call_function(L"SetActorHiddenInGame", p);
}

bool grip_held_side(bool right) {
    static UEVR_ActionHandle grip = nullptr;
    if (grip == nullptr) grip = API::VR::get_action_handle("/actions/default/in/Grip");
    if (grip == nullptr) return false;
    return API::VR::is_action_active(grip, right ? API::VR::get_right_joystick_source()
                                                 : API::VR::get_left_joystick_source());
}

} // namespace (reopened below) -- the grip predicate is exported

bool holster_grip_held(bool right) { return grip_held_side(right); }

namespace {

// Zone centre in the HEAD frame (x right, y up, z back), rotated by the head's yaw only.
Vec3 zone_offset(HolsterSlot s) {
    const float* o = nullptr;
    switch (s) {
        case HolsterSlot::RightShoulder: o = g_cfg.holster_rs; break;
        case HolsterSlot::LeftShoulder:  o = g_cfg.holster_ls; break;
        case HolsterSlot::RightHip:      o = g_cfg.holster_rh; break;
        case HolsterSlot::LeftChest:     o = g_cfg.holster_lc; break;
        case HolsterSlot::RightChest:    o = g_cfg.holster_rc; break;
        default: return Vec3{0.0f, 0.0f, 0.0f};
    }
    return Vec3{o[0], o[1], o[2]};
}

} // namespace

bool holster_swap_press_active()  { const auto u = g_holster_swap_until.load(std::memory_order_relaxed);  return u != 0 && now_ticks() < u; }
bool holster_throw_press_active() { const auto u = g_holster_throw_until.load(std::memory_order_relaxed); return u != 0 && now_ticks() < u; }
HolsterSlot holster_stowed_slot() { return (s_slot_a != HolsterSlot::None) ? s_slot_a : s_slot_b; }
bool holster_grenade_armed() { return s_grenade_armed; }
float holster_nearest_dist() { return s_nearest_dist; }
bool  holster_veto_by_proximity() { return s_near_zone; }
// A held grenade suppresses fire ONLY when it shares the aim hand (holster_gren_hand=0): there
// the weapon model is hidden, and an invisible gun that still shoots is worse than either state
// alone. In the off hand the gun is visible, held, and stays live -- that is the point.
bool holster_fire_suppressed() {
    return s_unarmed || (s_grenade_armed && !s_carry_off);
}
// The off hand is doing grenade work: a grenade is carried there, or its grip is closed inside
// a pouch. The two-hand latch yields to this -- the same squeeze must not brace the weapon AND
// pull a grenade.
bool holster_offhand_busy() {
    return (s_grenade_armed && s_carry_off)
        || (s_gin_zone != HolsterSlot::None && s_ggrip_prev);
}
// The fetch hand is on the visible magazine (previous holster tick -- one tick of staleness is
// ~3 cm at reaching speed, inside the grab radius). Gesture's MAG_OUT grab consumes this when
// reloadmag is on and the holster frame is running; anywhere else it is false.
bool holster_mag_hand_in() {
    return s_mag_hand_in.load(std::memory_order_relaxed);
}
bool holster_gswitch_press_active() { const auto u = g_holster_gswitch_until.load(std::memory_order_relaxed); return u != 0 && now_ticks() < u; }
bool holster_melee_veto() {
    if (!g_cfg.holster_enabled) return false;
    // A grenade CARRIED in the aim hand vetoes melee (the throw swing IS a fast aim-hand
    // motion). In the off hand it does not: the melee detector watches the aim hand, and a
    // left-hand throw cannot false-trigger a right-hand punch -- you can still pistol-whip
    // while palming one.
    if (s_near_zone || (s_grenade_armed && !s_carry_off)) return true;
    return (now_ticks() - s_last_action) < ms_to_ticks(g_cfg.holster_melee_veto_ms);
}

void holster_reset() {
    g_holster_swap_until.store(0, std::memory_order_relaxed);
    g_holster_throw_until.store(0, std::memory_order_relaxed);
    // An involuntary transition (vehicle, cutscene, menu, death) hands the weapon back: the game
    // itself may have swapped or re-armed us and the bookkeeping cannot be trusted past it.
    if (s_unarmed || s_grenade_armed) {
        set_weapon_hidden(false);
        if (g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER re-armed (reset)");
    }
    s_unarmed = false; s_unhide_ticks = 0;
    s_slot_a = HolsterSlot::None; s_slot_b = HolsterSlot::RightShoulder;
    if (s_grenade_armed && g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER grenade dropped (reset)");
    s_grenade_armed = false;
    s_grip_prev = false;
    s_in_zone = HolsterSlot::None;
    s_near_zone = false;
    s_have_prev = false;
    s_hand_vel = Vec3{0.0f, 0.0f, 0.0f};
    s_hand_vel_w = Vec3{0.0f, 0.0f, 0.0f};
    s_ggrip_prev = false;
    s_gin_zone = HolsterSlot::None;
    s_pin_zone = HolsterSlot::None;
    s_carry_off = false;
    s_ghave_prev = false;
    s_ghand_vel = Vec3{0.0f, 0.0f, 0.0f};
    s_ghand_vel_w = Vec3{0.0f, 0.0f, 0.0f};
    // The torso re-seeds from the head on the next tick: after a vehicle, a cutscene or a death
    // the old body yaw is a fact about a different situation.
    s_body_init = false;
    markers_hide_all();
}

void holster_update(float dt) {
    if (!g_cfg.enabled || !g_cfg.holster_enabled ||
        g_aim_calibrating.load(std::memory_order_relaxed) ||
        g_stick_mode_active.load(std::memory_order_relaxed)) {
        holster_reset();
        return;
    }
    if (!(dt > 0.0f) || dt > 0.25f) { s_have_prev = false; return; }

    Vec3 hpos{}; Quat hrot{};
    if (!get_pose(API::VR::get_hmd_index(), &hpos, &hrot, /*use_aim=*/false)) { s_have_prev = false; return; }
    Vec3 pos{}; Quat rot{};
    const auto aim_idx = aim_is_right() ? API::VR::get_right_controller_index()
                                        : API::VR::get_left_controller_index();
    if (!get_pose(aim_idx, &pos, &rot, /*use_aim=*/false)) { s_have_prev = false; return; }
    // A dead controller reports the origin; do not let it sit "in" a zone.
    if (std::fabs(pos.x) < 1e-6f && std::fabs(pos.y) < 1e-6f && std::fabs(pos.z) < 1e-6f) { s_have_prev = false; return; }
    // The OFF hand, tracked alongside. A dead off controller degrades softly: its pouches simply
    // become unreachable, everything on the aim hand keeps working.
    Vec3 gpos = pos; Quat grot = rot;
    bool ghand_ok = false;
    {
        const auto gidx = off_is_right() ? API::VR::get_right_controller_index()
                                         : API::VR::get_left_controller_index();
        ghand_ok = (gidx >= 0 && get_pose(gidx, &gpos, &grot, /*use_aim=*/false))
                && !(std::fabs(gpos.x) < 1e-6f && std::fabs(gpos.y) < 1e-6f && std::fabs(gpos.z) < 1e-6f);
        if (!ghand_ok) { s_ghave_prev = false; gpos = pos; grot = rot; }
    }

    // Head yaw from the head's forward vector (VR: forward = -z).
    const Vec3 fwd = quat_forward(hrot);
    const float head_yaw = std::atan2(-fwd.x, -fwd.z);

    // ---- THE TORSO LEASH (doctrine in Config.hpp). The zones used to rotate one-to-one with the
    // head, which put the pouches inside the body the moment the head turned. Now they hang on a
    // lagged BODY yaw: free look inside the dead zone, dragged beyond it, slow recentre toward the
    // head so a sustained turn brings the gear around while a glance moves nothing.
    auto wrap_rad = [](float a) {
        while (a >  3.14159265f) a -= 6.28318531f;
        while (a < -3.14159265f) a += 6.28318531f;
        return a;
    };
    if (!s_body_init) { s_body_yaw = head_yaw; s_body_init = true; }
    const float dead = g_cfg.holster_yaw_dead * 0.01745329f;
    float dyaw = wrap_rad(head_yaw - s_body_yaw);
    if (dyaw >  dead) { s_body_yaw = wrap_rad(head_yaw - dead); dyaw = dead; }
    if (dyaw < -dead) { s_body_yaw = wrap_rad(head_yaw + dead); dyaw = -dead; }
    if (g_cfg.holster_yaw_rate > 0.0f && dt > 0.0f) {
        const float step = g_cfg.holster_yaw_rate * 0.01745329f * dt;
        if (dyaw >  step)      s_body_yaw = wrap_rad(s_body_yaw + step);
        else if (dyaw < -step) s_body_yaw = wrap_rad(s_body_yaw - step);
        else                   s_body_yaw = head_yaw;
    }
    const float yaw = s_body_yaw;
    const float c = std::cos(yaw), s = std::sin(yaw);

    // ---- NECK PIVOT (doctrine in Config.hpp). The anchor is the neck, not the eyes: neck =
    // head + R_head * (0, -down, back), then lifted back to head height in the BODY frame so a
    // level head reproduces the old zone positions EXACTLY (the two offsets cancel by
    // construction) and only genuine head-about-neck rotation changes anything.
    Vec3 anchor = hpos;
    {
        const float nd = g_cfg.holster_neck_down, nb = g_cfg.holster_neck_back;
        if (nd != 0.0f || nb != 0.0f) {
            const Vec3 nw = quat_rotate(hrot, Vec3{0.0f, -nd, nb});
            anchor.x = hpos.x + nw.x + (-nb * s);
            anchor.y = hpos.y + nw.y + nd;
            anchor.z = hpos.z + nw.z + (-nb * c);
        }
    }

    // Hand in the body frame: translate from the ANCHOR, then rotate by -yaw so x=right, z=back.
    const float rx = pos.x - anchor.x, ry = pos.y - anchor.y, rz = pos.z - anchor.z;
    // Rotation about Y by -yaw: world (x, z) -> head frame.
    const float hx =  rx * c + rz * (-s);
    const float hz =  rx * s + rz * c;
    const Vec3 hand{hx, ry, hz};
    // The grenade hand, same frame.
    const float grx = gpos.x - anchor.x, gry = gpos.y - anchor.y, grz = gpos.z - anchor.z;
    const Vec3 ghand{grx * c + grz * (-s), gry, grx * s + grz * c};

    // Velocity (head frame), ~60 ms EMA, for the throw.
    if (s_have_prev) {
        const Vec3 raw{(hand.x - s_prev_hand.x) / dt, (hand.y - s_prev_hand.y) / dt, (hand.z - s_prev_hand.z) / dt};
        const float a = (dt / 0.06f > 1.0f) ? 1.0f : dt / 0.06f;
        s_hand_vel.x += (raw.x - s_hand_vel.x) * a; s_hand_vel.y += (raw.y - s_hand_vel.y) * a; s_hand_vel.z += (raw.z - s_hand_vel.z) * a;
        // World-frame twin, same subtraction, same alpha -- for the throw's DIRECTION. The
        // head-frame EMA cannot serve here: it accumulates across a rotating frame, so a value
        // built in it has no single frame to be converted back from.
        const Vec3 rel_w{pos.x - hpos.x, pos.y - hpos.y, pos.z - hpos.z};
        const Vec3 raww{(rel_w.x - s_prev_rel_w.x) / dt, (rel_w.y - s_prev_rel_w.y) / dt, (rel_w.z - s_prev_rel_w.z) / dt};
        s_hand_vel_w.x += (raww.x - s_hand_vel_w.x) * a; s_hand_vel_w.y += (raww.y - s_hand_vel_w.y) * a; s_hand_vel_w.z += (raww.z - s_hand_vel_w.z) * a;
        s_prev_rel_w = rel_w;

        // Peak-hold over a 250 ms window. The peaks landed 25 to 83 ms before release across all
        // ten measured throws, so 250 ms covers the worst of them three times over, while still
        // being short enough that the reach INTO the pouch cannot leak into the next throw.
        const float f_now = -s_hand_vel.z;
        const float s_now = std::sqrt(s_hand_vel.x * s_hand_vel.x + s_hand_vel.y * s_hand_vel.y + s_hand_vel.z * s_hand_vel.z);
        const long long tnow = now_ticks();
        if (f_now > s_peak_fwd || (tnow - s_peak_at) > ms_to_ticks(250)) {
            s_peak_fwd = f_now; s_peak_spd = s_now; s_peak_at = tnow;
            // The direction rides the peak: the throw is judged at the swing's best moment, so
            // the direction must be sampled at that same moment, not at the release instant.
            s_peak_velw = s_hand_vel_w;
        }
        else if (s_now > s_peak_spd) s_peak_spd = s_now;

        // A pose the tracker has not refreshed differences to exactly zero and drags the EMA down.
        // Count them rather than assume the rate: a probe cannot see faster than it samples.
        ++s_total_frames;
        if (s_have_raw && std::fabs(pos.x - s_prev_raw.x) < 1e-7f && std::fabs(pos.y - s_prev_raw.y) < 1e-7f
            && std::fabs(pos.z - s_prev_raw.z) < 1e-7f) ++s_stale_frames;
    }
    s_prev_raw = pos; s_have_raw = true;
    s_prev_hand = hand; s_have_prev = true;

    // The grenade hand's velocity and peak, same maths, same windows, same doctrine as above.
    if (ghand_ok) {
        if (s_ghave_prev) {
            const Vec3 raw{(ghand.x - s_gprev_hand.x) / dt, (ghand.y - s_gprev_hand.y) / dt, (ghand.z - s_gprev_hand.z) / dt};
            const float a = (dt / 0.06f > 1.0f) ? 1.0f : dt / 0.06f;
            s_ghand_vel.x += (raw.x - s_ghand_vel.x) * a; s_ghand_vel.y += (raw.y - s_ghand_vel.y) * a; s_ghand_vel.z += (raw.z - s_ghand_vel.z) * a;
            const Vec3 rel_w{gpos.x - hpos.x, gpos.y - hpos.y, gpos.z - hpos.z};
            const Vec3 raww{(rel_w.x - s_gprev_rel_w.x) / dt, (rel_w.y - s_gprev_rel_w.y) / dt, (rel_w.z - s_gprev_rel_w.z) / dt};
            s_ghand_vel_w.x += (raww.x - s_ghand_vel_w.x) * a; s_ghand_vel_w.y += (raww.y - s_ghand_vel_w.y) * a; s_ghand_vel_w.z += (raww.z - s_ghand_vel_w.z) * a;
            s_gprev_rel_w = rel_w;
            const float f_now = -s_ghand_vel.z;
            const float s_now = std::sqrt(s_ghand_vel.x * s_ghand_vel.x + s_ghand_vel.y * s_ghand_vel.y + s_ghand_vel.z * s_ghand_vel.z);
            const long long tnow = now_ticks();
            if (f_now > s_gpeak_fwd || (tnow - s_gpeak_at) > ms_to_ticks(250)) {
                s_gpeak_fwd = f_now; s_gpeak_spd = s_now; s_gpeak_at = tnow;
                s_gpeak_velw = s_ghand_vel_w;
            }
            else if (s_now > s_gpeak_spd) s_gpeak_spd = s_now;
            ++s_gtotal_frames;
            if (s_ghave_raw && std::fabs(gpos.x - s_gprev_raw.x) < 1e-7f && std::fabs(gpos.y - s_gprev_raw.y) < 1e-7f
                && std::fabs(gpos.z - s_gprev_raw.z) < 1e-7f) ++s_gstale_frames;
        }
        s_gprev_raw = gpos; s_ghave_raw = true;
        s_gprev_hand = ghand; s_ghave_prev = true;
    }

    // Which zone, if any (nearest within radius). The AIM hand works the weapon slots and -- if
    // allowed -- the pouches; the OFF hand works only the pouches.
    HolsterSlot zone = HolsterSlot::None; float best = 1e9f; float nearest = 1e9f;
    const HolsterSlot slots[3] = {HolsterSlot::RightShoulder, HolsterSlot::LeftShoulder, HolsterSlot::RightHip};
    for (HolsterSlot sl : slots) {
        const Vec3 o = zone_offset(sl);
        const float d = std::sqrt((hand.x - o.x) * (hand.x - o.x) + (hand.y - o.y) * (hand.y - o.y) + (hand.z - o.z) * (hand.z - o.z));
        if (d < nearest) nearest = d;
        if (d < g_cfg.holster_radius && d < best) { best = d; zone = sl; }
    }
    // Each hand against the pouches. Only the AIM hand's pouch proximity joins the melee veto's
    // nearest -- the melee swing is an aim-hand event, and a left hand resting by a pouch must
    // not stand the right hand's punches down.
    HolsterSlot zone_p = HolsterSlot::None; float pbest = 1e9f;   // AIM hand in a pouch
    HolsterSlot zone_g = HolsterSlot::None; float gbest = 1e9f;   // OFF hand in a pouch
    const HolsterSlot pouches[2] = {HolsterSlot::LeftChest, HolsterSlot::RightChest};
    for (HolsterSlot sl : pouches) {
        const Vec3 o = zone_offset(sl);
        if (aim_can_grab()) {
            const float d = std::sqrt((hand.x - o.x) * (hand.x - o.x) + (hand.y - o.y) * (hand.y - o.y) + (hand.z - o.z) * (hand.z - o.z));
            if (d < nearest) nearest = d;
            if (d < g_cfg.holster_gradius && d < pbest) { pbest = d; zone_p = sl; }
        }
        if (off_can_grab() && ghand_ok) {
            const float d = std::sqrt((ghand.x - o.x) * (ghand.x - o.x) + (ghand.y - o.y) * (ghand.y - o.y) + (ghand.z - o.z) * (ghand.z - o.z));
            if (d < g_cfg.holster_gradius && d < gbest) { gbest = d; zone_g = sl; }
        }
    }
    s_nearest_dist = nearest;
    s_near_zone = (nearest < g_cfg.holster_radius + g_cfg.holster_melee_margin);

    // ---- GRENADE VISUALS. Spawned lazily (paced), placed every tick, scale re-applied every
    // tick -- the wheel-disc lesson: anything a live cfg value controls must be re-applied on the
    // cadence the value can change. Placement: zone offsets are HEAD-frame; head->room is the
    // inverse of the rotation above (rx = hx*c + hz*s, rz = -hx*s + hz*c), then the palette's
    // room->world -- the same transform that puts the rendered gun on the hand on foot.
    const bool want_gren_marks = g_cfg.holster_markers != 0;
    const bool want_mag_mark   = g_cfg.reload_mag != 0;
    if ((want_gren_marks || want_mag_mark)
        && ((want_gren_marks && (s_pouch_l.get() == nullptr || s_pouch_r.get() == nullptr || s_hand_g.get() == nullptr))
            || (want_mag_mark && s_mag_marker.get() == nullptr))
        && (++s_mk_tick % 120u) == 1u) {
        resolve_grenade_meshes();
        auto* mf = s_mesh_frag.get(); auto* mp = s_mesh_plasma.get();
        if (mf == nullptr) mf = mp;          // stand-ins, never a meshless component
        if (mp == nullptr) mp = mf;
        if (auto* owner = API::get()->get_local_pawn(0)) {
            if (want_gren_marks && mf != nullptr) {
                const double ms = (double)g_cfg.holster_marker_scale * 12.5;
                if (s_pouch_l.get() == nullptr) { if (auto* m = holster_marker_spawn_mesh(owner, mf, ms)) s_pouch_l.set(m); }
                if (s_pouch_r.get() == nullptr) { if (auto* m = holster_marker_spawn_mesh(owner, mp, ms)) s_pouch_r.set(m); }
                if (s_hand_g.get()  == nullptr) { if (auto* m = holster_marker_spawn_mesh(owner, mf, ms)) s_hand_g.set(m); }
            }
            if (want_mag_mark && s_mag_marker.get() == nullptr) {
                auto* mm = s_mesh_mag.get();
                if (mm == nullptr) mm = mf;   // survey found nothing: the frag stands in, visibly
                if (mm != nullptr) {
                    if (auto* m = holster_marker_spawn_mesh(owner, mm, (double)g_cfg.reload_mag_scale)) {
                        s_mag_marker.set(m);
                        holster_marker_show(m, false);
                        s_mag_mesh_key = "\x01";   // fresh component: force the per-weapon pick
                    }
                }
            }
        }
    }
    if (g_cfg.holster_markers != 0) {
        const float capprch = 0.45f;   // pouches fade in as a grabbing-capable hand approaches (mode 1)
        auto place_pouch = [&](TrackedObject& t, HolsterSlot sl) {
            auto* m = t.get();
            if (m == nullptr) return;
            const Vec3 o = zone_offset(sl);
            float dist = 1e9f;
            if (aim_can_grab()) {
                const float ddx = hand.x - o.x, ddy = hand.y - o.y, ddz = hand.z - o.z;
                dist = std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
            }
            if (off_can_grab() && ghand_ok) {
                const float ddx = ghand.x - o.x, ddy = ghand.y - o.y, ddz = ghand.z - o.z;
                const float d2 = std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
                if (d2 < dist) dist = d2;
            }
            const bool show = (g_cfg.holster_markers == 2) || dist < capprch;
            holster_marker_show(m, show);
            if (!show) return;
            const Vec3 room{anchor.x + o.x * c + o.z * s, anchor.y + o.y, anchor.z + (-o.x * s + o.z * c)};
            holster_marker_place(m, holster_room_to_world(room, hpos));
            // 12.5x: holstermarkerscale keeps its meaning (0.08 = "natural size") across the move
            // from the 100 cm engine sphere to real assets authored at grenade size.
            holster_marker_scale(m, (double)g_cfg.holster_marker_scale * 12.5);
        };
        place_pouch(s_pouch_l, HolsterSlot::LeftChest);
        place_pouch(s_pouch_r, HolsterSlot::RightChest);
        // The grenade in the hand: the grab's receipt. Armed -> a sphere rides the controller;
        // released (thrown or put back) -> gone. Slightly larger than the pouch dots so the state
        // change reads at a glance.
        if (auto* m = s_hand_g.get()) {
            holster_marker_show(m, s_grenade_armed);
            if (s_grenade_armed) {
                holster_marker_place(m, holster_room_to_world(s_carry_off ? gpos : pos, hpos));
                holster_marker_scale(m, (double)g_cfg.holster_marker_scale * 12.5);
                // The hand shows the TYPE being held: frag mesh for frag, plasma for plasma.
                auto* want = (s_gtype != 0) ? s_mesh_plasma.get() : s_mesh_frag.get();
                if (want != nullptr) holster_marker_set_mesh(m, want);
            }
        }
    } else markers_hide_all();

    // ---- THE VISIBLE MAGAZINE. Runs after markers_hide_all so a disabled holster-marker set
    // cannot blank it: the mag belongs to the RELOAD, not to holstermarkers. Body frame, torso
    // leash, neck pivot -- the same maths as the pouches, so the mag hangs off your hip exactly
    // as the grenades do.
    if (g_cfg.reload_mag != 0) {
        const ReloadState rs = reload_state();
        const Vec3 mo{g_cfg.reload_mag_off[0], g_cfg.reload_mag_off[1], g_cfg.reload_mag_off[2]};
        // The fetch hand's distance to the belt point, published for Gesture's grab test. Gated on
        // MAG_OUT so a hand idling at the hip between reloads publishes nothing.
        bool in = false;
        if (rs == ReloadState::MagOut && ghand_ok) {
            const float ddx = ghand.x - mo.x, ddy = ghand.y - mo.y, ddz = ghand.z - mo.z;
            const float d = std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
            in = d <= g_cfg.reload_mag_radius;
            if (g_cfg.reload_log) {
                static uint32_t s_ml = 0;
                if ((s_ml++ % 15u) == 0u)
                    API::get()->log_info("[Halo-CampE-UEVR] RELOAD mag at belt: hand-to-mag=%.0fcm (need <=%.0f)",
                                         d * 100.0f, g_cfg.reload_mag_radius * 100.0f);
            }
        }
        s_mag_hand_in.store(in, std::memory_order_relaxed);
        // Entering the mag's zone buzzes the fetch hand -- same cue the pouches give, and the only
        // feedback that survives the mag mesh failing to resolve.
        if (in != s_mag_zone_prev) {
            if (in) ghaptic(0.08f, 0.5f);
            s_mag_zone_prev = in;
        }
        if (auto* m = s_mag_marker.get()) {
            // THE MAG MATCHES THE GUN. Re-picked only when the key changes (SetStaticMesh per tick
            // is neither needed nor kind), and a transiently empty key changes nothing -- the mesh
            // in hand outlives a weapon-lowered frame, exactly as the reload state itself does.
            if (rs != ReloadState::Idle) {
                const std::string wk = weapon_key();
                if (!wk.empty() && wk != s_mag_mesh_key) {
                    int rank = 0;
                    auto* mesh = mag_mesh_for_weapon(wk, &rank);
                    // A pick that is not a real MAGAZINE gets ONE re-survey before it is accepted.
                    // The survey runs once per session, so its candidate handles die at every
                    // level transition -- the weapon's own SM_*_Magazine drops out while some
                    // SM_ammo_pickup_* survives, and the marker becomes "the ammo box itself"
                    // (field report). Once per weapon key, so shell loaders with no magazine
                    // asset settle on their pickup after a single rebuild instead of thrashing
                    // a ~290k-object walk every tick.
                    static std::string s_resurveyed_for;
                    if (rank < 3 && s_resurveyed_for != wk) {
                        s_resurveyed_for = wk;
                        s_mag_cands.clear();
                        s_mag_pick = 0;
                        s_mag_search_done = false;
                        if (g_cfg.reload_log)
                            API::get()->log_info("[Halo-CampE-UEVR] RELOAD mag pick for %s ranked %d"
                                                 " -- re-surveying", wk.c_str(), rank);
                        // s_mag_mesh_key stays unset: re-pick next tick, after the walk.
                    } else {
                        if (mesh != nullptr) {
                            holster_marker_set_mesh(m, mesh);
                            if (g_cfg.reload_log)
                                API::get()->log_info("[Halo-CampE-UEVR] RELOAD mag mesh for %s -> %ls",
                                                     wk.c_str(), mesh->get_full_name().c_str());
                        }
                        s_mag_mesh_key = wk;
                    }
                }
            }
            // ORIENTATION comes from direction vectors pushed through room_to_world (differences,
            // so the camera translation cancels) -- the same proven position mapping, no second
            // frame-math implementation to get an axis wrong in.
            auto world_dir = [&](const Vec3& at, const Vec3& d) {
                const Vec3 w0 = holster_room_to_world(at, hpos);
                const Vec3 w1 = holster_room_to_world(Vec3{at.x + d.x, at.y + d.y, at.z + d.z}, hpos);
                Vec3 out{w1.x - w0.x, w1.y - w0.y, w1.z - w0.z};
                const float l = std::sqrt(out.x * out.x + out.y * out.y + out.z * out.z);
                if (l > 1e-4f) { out.x /= l; out.y /= l; out.z /= l; }
                return out;
            };
            if (rs == ReloadState::MagOut) {
                const Vec3 room{anchor.x + mo.x * c + mo.z * s, anchor.y + mo.y, anchor.z + (-mo.x * s + mo.z * c)};
                // The belt mag holds the BODY's yaw, so it sits on the hip the same way no matter
                // where in the world you face, instead of pointing at map north.
                const Vec3 bf = world_dir(room, Vec3{-s, 0.0f, -c});
                holster_marker_show(m, true);
                holster_marker_place_rot(m, holster_room_to_world(room, hpos),
                                         0.0f, std::atan2(bf.y, bf.x) * RAD2DEG, 0.0f);
                holster_marker_scale(m, (double)g_cfg.reload_mag_scale);
            } else if (rs == ReloadState::MagHeld && ghand_ok) {
                // IN THE HAND the mag wears the controller's full orientation: yaw and pitch from
                // the hand's forward, roll recovered from where the hand's up landed relative to
                // the roll-free frame. If a wrist twist ever rolls the mag the WRONG way, the fix
                // is one sign on the atan2 below -- say so rather than re-deriving.
                const Vec3 F = world_dir(gpos, quat_forward(grot));
                const Vec3 U = world_dir(gpos, quat_rotate(grot, Vec3{0.0f, 1.0f, 0.0f}));
                const float yawr   = std::atan2(F.y, F.x);
                const float pitchr = std::asin(std::fmax(-1.0f, std::fmin(1.0f, F.z)));
                const float cy = std::cos(yawr), sy = std::sin(yawr);
                const float cp = std::cos(pitchr), sp = std::sin(pitchr);
                const Vec3 right0{-sy, cy, 0.0f};
                const Vec3 up0{-sp * cy, -sp * sy, cp};
                const float rollr = std::atan2(U.x * right0.x + U.y * right0.y + U.z * right0.z,
                                               U.x * up0.x + U.y * up0.y + U.z * up0.z);
                holster_marker_show(m, true);
                holster_marker_place_rot(m, holster_room_to_world(gpos, hpos),
                                         pitchr * RAD2DEG, yawr * RAD2DEG, rollr * RAD2DEG);
                holster_marker_scale(m, (double)g_cfg.reload_mag_scale);
            } else {
                holster_marker_show(m, false);
            }
        }
    }
    if (zone != s_in_zone) {
        // Entering a zone gets a tick of haptics -- the slot you can act on; leaving gets nothing.
        if (zone != HolsterSlot::None) {
            const bool full = (zone == s_slot_a) || (zone == s_slot_b);
            haptic(full ? 0.08f : 0.04f, full ? 0.5f : 0.25f);
            if (g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER enter %s (%s) hand=(%.2f %.2f %.2f)", slot_name(zone), full ? "full" : "empty", hand.x, hand.y, hand.z);
        }
        s_in_zone = zone;
    }
    if (zone_g != s_gin_zone) {
        if (zone_g != HolsterSlot::None) {
            ghaptic(0.08f, 0.5f);
            if (g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER enter %s (off hand)=(%.2f %.2f %.2f)", slot_name(zone_g), ghand.x, ghand.y, ghand.z);
        }
        s_gin_zone = zone_g;
    }
    if (zone_p != s_pin_zone) {
        if (zone_p != HolsterSlot::None) {
            haptic(0.08f, 0.5f);
            if (g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER enter %s (aim hand)=(%.2f %.2f %.2f)", slot_name(zone_p), hand.x, hand.y, hand.z);
        }
        s_pin_zone = zone_p;
    }

    const bool agrip = grip_held_side(aim_is_right());
    const bool pressed   = agrip && !s_grip_prev;
    const bool areleased = !agrip && s_grip_prev;
    s_grip_prev = agrip;
    // The off hand's grip, edges tracked separately so the dispatch below cannot double-fire.
    const bool ggrip = ghand_ok && off_can_grab() && grip_held_side(off_is_right());
    const bool gpressed  = ggrip && !s_ggrip_prev;
    const bool greleased = !ggrip && s_ggrip_prev;
    s_ggrip_prev = ggrip;

    // A GRIP THAT CAUGHT NOTHING is the only evidence of a failed grab, and it was never
    // recorded. Log how far the hand actually was from each pouch: a near miss (d just over the
    // radius) and a wild miss (hand nowhere near) call for opposite fixes -- a bigger sphere
    // versus a moved one -- and the log line separates them without guessing.
    if (g_cfg.holster_log && ((gpressed && zone_g == HolsterSlot::None)
                           || (pressed && aim_can_grab() && zone_p == HolsterSlot::None && zone == HolsterSlot::None))) {
        const Vec3& mh = (gpressed && zone_g == HolsterSlot::None) ? ghand : hand;
        const Vec3 olc = zone_offset(HolsterSlot::LeftChest), orc = zone_offset(HolsterSlot::RightChest);
        const float dlc = std::sqrt((mh.x-olc.x)*(mh.x-olc.x) + (mh.y-olc.y)*(mh.y-olc.y) + (mh.z-olc.z)*(mh.z-olc.z));
        const float drc = std::sqrt((mh.x-orc.x)*(mh.x-orc.x) + (mh.y-orc.y)*(mh.y-orc.y) + (mh.z-orc.z)*(mh.z-orc.z));
        API::get()->log_info("[Halo-CampE-UEVR] HOLSTER MISS: hand=(%.3f %.3f %.3f) frag d=%.3f plasma d=%.3f (gradius %.3f) nearest-any=%.3f",
                             mh.x, mh.y, mh.z, dlc, drc, g_cfg.holster_gradius, nearest);
    }

    // ---- GRENADE GRAB: either capable hand's grip closing inside a pouch. The hand that grabs
    // CARRIES -- and the armed behaviour follows the carrier (see the arm below).
    auto try_grab = [&](HolsterSlot pz, bool carry_off) {
        if (s_grenade_armed) return;
        s_last_action = now_ticks();
        const int want = (pz == HolsterSlot::LeftChest) ? 0 : 1;   // frag left, plasma right
        const bool live = g_unit_gvalid.load(std::memory_order_relaxed);
        const int have  = live ? g_unit_gtype.load(std::memory_order_relaxed) : s_gtype;
        const int count = live ? (want ? g_unit_gplasma.load(std::memory_order_relaxed) : g_unit_gfrag.load(std::memory_order_relaxed)) : 1;
        if (count <= 0) {
            // Nothing in that pouch: a weak buzz and no arm.
            haptic_on(carry_off ? off_is_right() : aim_is_right(), 0.05f, 0.3f);
            if (g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER %s pouch empty (frag %d plasma %d)", want ? "plasma" : "frag", g_unit_gfrag.load(), g_unit_gplasma.load());
            return;
        }
        if (want != have) {
            if (g_cfg.holster_gswitch_mask != 0) {
                g_holster_gswitch_until.store(now_ticks() + ms_to_ticks(g_cfg.holster_press_ms), std::memory_order_relaxed);
                s_gtype = want; s_gswitch_pending = 60;
                if (g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER grenade type -> %s (switch press; unit had %s)", want ? "plasma" : "frag", have ? "plasma" : "frag");
            } else if (g_cfg.holster_log) {
                API::get()->log_info("[Halo-CampE-UEVR] HOLSTER grenade type %s wanted but holstergswitchmask=0 -- not switching", want ? "plasma" : "frag");
            }
        } else {
            s_gtype = have;
        }
        s_grenade_armed = true;
        s_carry_off = carry_off;
        s_arm_at = now_ticks();
        // With the grenade in the SAME hand as the gun, the weapon gives way: holding both in
        // one hand reads wrong the moment the grenade is visible on the controller. In the OFF
        // hand, the gun stays exactly where it is -- gun right, grenade left is the natural
        // two-handed carry.
        if (!carry_off) set_weapon_hidden(true);
        // Clear the CARRIER's peak at the arm, so only motion made while holding the grenade can
        // throw it. The reach in to the chest is itself a fast move; without this it would sit
        // in the window and could throw a grenade you only meant to pick up.
        if (carry_off) {
            s_gpeak_fwd = 0.0f; s_gpeak_spd = 0.0f; s_gpeak_at = now_ticks();
            s_gstale_frames = 0; s_gtotal_frames = 0;
        } else {
            s_peak_fwd = 0.0f; s_peak_spd = 0.0f; s_peak_at = now_ticks();
            s_stale_frames = 0; s_total_frames = 0;
        }
        haptic_on(carry_off ? off_is_right() : aim_is_right(), 0.10f, 0.8f);
        if (g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER grenade armed (%s, %s hand; frag %d plasma %d)",
                                                    want ? "plasma" : "frag", carry_off ? "off" : "aim",
                                                    g_unit_gfrag.load(), g_unit_gplasma.load());
    };
    if (gpressed && zone_g != HolsterSlot::None) try_grab(zone_g, /*carry_off=*/true);
    if (pressed && zone_p != HolsterSlot::None)  try_grab(zone_p, /*carry_off=*/false);

    // ---- WEAPON SLOTS, on the aim hand's grip. A press inside a pouch belongs to the grenade
    // branch above and must not also swap weapons.
    if (pressed && zone != HolsterSlot::None && zone_p == HolsterSlot::None) {
        s_last_action = now_ticks();
        {
            const long long now = now_ticks();
            if (now - s_last_swap > ms_to_ticks(400)) {
                s_last_swap = now;
                const bool a_here = (zone == s_slot_a), b_here = (zone == s_slot_b);
                const bool armed = !s_unarmed;
                if (armed && !a_here && !b_here) {
                    // STOW the held weapon (whichever is in hand) into this empty slot.
                    if (s_slot_a == HolsterSlot::None) s_slot_a = zone; else s_slot_b = zone;
                    s_unarmed = true;
                    set_weapon_hidden(true);
                    haptic(0.12f, 0.9f);
                    if (g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER STOW at %s -> unarmed (A@%s B@%s)", slot_name(zone), slot_name(s_slot_a), slot_name(s_slot_b));
                } else if (!armed && (a_here || b_here)) {
                    // DRAW. The game's backup needs a swap press first; afterwards it IS the
                    // current weapon, so relabel (A is always the game's current).
                    if (b_here) {
                        g_holster_swap_until.store(now + ms_to_ticks(g_cfg.holster_press_ms), std::memory_order_relaxed);
                        const HolsterSlot t = s_slot_a; s_slot_a = s_slot_b; s_slot_b = t;   // relabel
                    }
                    s_slot_a = HolsterSlot::None;
                    s_unarmed = false; s_unhide_ticks = 30;
                    set_weapon_hidden(false);
                    haptic(0.12f, 0.9f);
                    if (g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER DRAW from %s%s -> armed (A@hand B@%s)", slot_name(zone), b_here ? " (swap)" : "", slot_name(s_slot_b));
                } else if (armed && b_here) {
                    // EXCHANGE at the other gun's slot: held gun takes the slot, the other comes out.
                    g_holster_swap_until.store(now + ms_to_ticks(g_cfg.holster_press_ms), std::memory_order_relaxed);
                    s_slot_b = HolsterSlot::None; s_slot_a = zone;
                    { const HolsterSlot t = s_slot_a; s_slot_a = s_slot_b; s_slot_b = t; }     // relabel
                    s_unhide_ticks = 30;
                    haptic(0.12f, 0.9f);
                    if (g_cfg.holster_log) API::get()->log_info("[Halo-CampE-UEVR] HOLSTER EXCHANGE at %s -> swap (A@hand B@%s)", slot_name(zone), slot_name(s_slot_b));
                } else if (g_cfg.holster_log) {
                    API::get()->log_info("[Halo-CampE-UEVR] HOLSTER grip at %s: nothing to do (%s, A@%s B@%s)", slot_name(zone), armed ? "armed" : "unarmed", slot_name(s_slot_a), slot_name(s_slot_b));
                }
            }
        }
    }

    // Keep the hidden state asserted: a swap or a pickup spawns a new weapon actor that arrives
    // visible, and SetActorHiddenInGame is per-actor. Cheap (one UFunction call per tick).
    if (s_unarmed || (s_grenade_armed && !s_carry_off)) set_weapon_hidden(true);
    else if (s_unhide_ticks > 0) { --s_unhide_ticks; set_weapon_hidden(false); }

    // Release comes from the CARRIER's grip, and every measured quantity below is the carrier's.
    const bool crel = s_carry_off ? greleased : areleased;
    if (crel && s_grenade_armed) {
        const bool coff = s_carry_off;
        const Vec3& cvel  = coff ? s_ghand_vel   : s_hand_vel;
        const Vec3& cpeak = coff ? s_gpeak_velw  : s_peak_velw;
        const float cpeak_fwd = coff ? s_gpeak_fwd : s_peak_fwd;
        const float cpeak_spd = coff ? s_gpeak_spd : s_peak_spd;
        const long long cpeak_at = coff ? s_gpeak_at : s_peak_at;
        const int cstale = coff ? s_gstale_frames : s_stale_frames;
        const int ctotal = coff ? s_gtotal_frames : s_total_frames;
        const HolsterSlot czone = coff ? zone_g : zone_p;
        s_grenade_armed = false;
        s_last_action = now_ticks();
        if (!coff && !s_unarmed) { set_weapon_hidden(false); s_unhide_ticks = 30; }
        const float fwd_speed = -cvel.z;
        const float speed = std::sqrt(cvel.x * cvel.x + cvel.y * cvel.y + cvel.z * cvel.z);
        // PUT-BACK IS POSITIONAL, FULL STOP. The rule, stated from the headset, and it is the right one: "a grenade
        // put back is only, and solely, when I literally put it back in the pouch." Release inside
        // a pouch zone = put back; release anywhere else = a throw, however soft. This retires the
        // speed gate entirely (holsterthrowspeed no longer classifies anything): a speed threshold
        // was a proxy for intent, and it misread every gentle lob as a put-back. Where the hand IS
        // at release is not a proxy -- it is the intent. The zone test is the same 13 cm sphere a
        // grab uses, so putting back happens exactly where taking out does.
        const bool in_pouch = (czone != HolsterSlot::None);
        const bool threw = !in_pouch;
        const bool cright = coff ? off_is_right() : aim_is_right();
        if (!threw) haptic_on(cright, 0.06f, 0.4f);   // the pouch accepted it back
        if (threw) {
            g_holster_throw_until.store(now_ticks() + ms_to_ticks(g_cfg.holster_press_ms), std::memory_order_relaxed);
            haptic_on(cright, 0.10f, 1.0f);
            // AIM THE THROW ALONG THE SWING -- the melee fix, applied to the grenade. The game
            // lobs along the aim, and the aim during a throw is the flailing hand. The hold is fed
            // the world velocity SAMPLED AT THE PEAK (the same instant the gate read) through the
            // same extraction derive_ctrl_angles uses, so grenades fly where the arm sent them.
            if (g_cfg.holster_aim_hold_ms > 0) {
                const float vlen = std::sqrt(cpeak.x * cpeak.x + cpeak.y * cpeak.y + cpeak.z * cpeak.z);
                if (vlen > 0.2f) {
                    const float hy = wrap180(std::atan2(cpeak.x, -cpeak.z) * RAD2DEG
                                             + g_cfg.aim_turn * g_turn_offset.load(std::memory_order_relaxed));
                    const float hpp = std::asin(std::fmax(-1.0f, std::fmin(1.0f, cpeak.y / vlen))) * RAD2DEG;
                    g_melee_aim_ctrl_yaw.store(hy, std::memory_order_relaxed);
                    g_melee_aim_ctrl_pitch.store(hpp, std::memory_order_relaxed);
                    g_melee_aim_hold_until.store(now_ticks() + ms_to_ticks(g_cfg.holster_aim_hold_ms),
                                                 std::memory_order_relaxed);
                    if (g_cfg.holster_log)
                        API::get()->log_info("[Halo-CampE-UEVR] HOLSTER THROW AIM: dir=(y%+.1f p%+.1f) |v|=%.2f held %d ms",
                                             hy, hpp, vlen, g_cfg.holster_aim_hold_ms);
                }
            }
        }
        if (g_cfg.holster_log) {
            // EVERY release logs the same four numbers, thrown or not, so the two populations are
            // directly comparable. peak-fwd is the same EMA at its best moment in the last 300 ms;
            // if the throws that failed have a healthy peak and a weak instant, the gate is
            // sampling too late. If BOTH are weak, the EMA is eating the throw. If stale frames
            // are a large share, the difference itself is under-sampled.
            const float hold_ms = (float)std::chrono::duration_cast<std::chrono::milliseconds>(
                clock_t_::duration(now_ticks() - s_arm_at)).count();
            const long long peak_age = now_ticks() - cpeak_at;
            const float peak_ms = (float)std::chrono::duration_cast<std::chrono::milliseconds>(
                clock_t_::duration(peak_age)).count();
            // Both numbers stay in the log so the next session can confirm the peak really is
            // the one that decided it, rather than taking this fit on trust.
            API::get()->log_info("[Halo-CampE-UEVR] HOLSTER %s (%s hand): release fwd %.2f |v| %.2f | PEAK fwd %.2f |v| %.2f (%.0f ms ago) | gate %.2f | held %.0f ms | stale %d/%d",
                                 threw ? "THROW" : "put back (in pouch)", coff ? "off" : "aim",
                                 fwd_speed, speed, cpeak_fwd, cpeak_spd, peak_ms,
                                 g_cfg.holster_throw_speed, hold_ms, cstale, ctotal);
        }
    }
}

} // namespace halo
