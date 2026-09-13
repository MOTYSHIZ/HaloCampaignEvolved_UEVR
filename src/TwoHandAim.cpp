#include "TwoHandAim.hpp"

#include "Config.hpp"
#include "DevTools.hpp"
#include "MotionAimControl.hpp"          // get_pose(), g_aim_law_ridx
#include "Gesture.hpp"                   // g_pad_buttons, for the bindtwohand mask route
#include "WeaponOffset.hpp"
#include "WeaponCalib.hpp"   // calib_hold_active(): stand down while a gesture owns the weapon              // weapon_offset_current_class()
#include "palettearm/TwoHand.hpp"        // the hold itself: pure, unit-tested, shared with the arms

#include "uevr/API.hpp"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cctype>
#include <cstring>

using uevr::API;

namespace halo {
namespace {

namespace pa = ::halo::palettearm;

pa::TwoHandHold   s_hold;
pa::TwoHandTuning s_tuning;   // units_to_metres defaults to 1: OpenXR hands you metres

// ---- THE GUN HELD AT TWO POINTS (twohandgun) ---------------------------------------------------
//
// 1 = measure the hold from the WEAPON (default), 0 = from the controller's aim RAY, with the
// handle offset subtracted from the support hand (every build before 2026-09-13).
//
// The swing turns ONE vector onto the line between the hands. In gun mode that vector is the
// ONE-HANDED gun's own grip-to-handle line: from the aim grip, down the barrel to where the support
// hand is, then out to the handle (wpngrip y/z). So a rigid gun held at two points is solved as a
// rigid gun held at two points, and three things fall out of that together:
//
//   1. A HOLD AT REST MOVES NOTHING. With both hands where the gun already has them, the two
//      vectors coincide and the swing is identity. The ray mode turned the controller's pointing
//      ray onto the hands instead, so grabbing rotated the gun by however far its barrel sits from
//      that ray -- the grip trim plus any per-weapon rotation -- even at the calibrated hold.
//
//   2. NO FEEDBACK. Both vectors come from THIS tick's raw poses and the UNSWUNG gun (the rig block
//      publishes its axes with the swing it applied taken back out). The ray mode rotated the
//      handle offset with the already-swung gun, so every swing moved the next tick's hand line.
//      With the sentinel beam's 27 cm handle and the hands close together that loop gains above 1:
//      it presented as the weapon SPINNING, and with an agreement band closed it flipped between the
//      one- and two-handed pose on alternate ticks -- "two ghost images of the weapon" (2026-09-13).
//
//   3. THE GATES MEAN SOMETHING. Agreement is now "how far is the support hand from where this gun's
//      handle would be, seen from the aim grip": ~1 on the gun, falling as a hand crosses over, at
//      any separation. So the gate can close to a band, and the minimum baseline shrinks to a guard
//      against the hands actually coinciding. Fading by DISTANCE starved grips that are close by
//      design -- the rocket launcher's sit 13 cm apart, which the 10-20 cm band held to about a
//      fifth of its authority.
//
// ITS OWN GATE KEYS (twohandgunagree*/twohandgunminbase), deliberately not the ray mode's: the two
// agreements measure different things, so a value tuned for one silently applied to the other --
// e.g. a player file that opened the ray gate with twohandagreemin=-1 -- would switch the gun gate
// off without saying so.
int   s_gun_mode       = 1;
float s_gun_agree_min  = 0.35f;   // cos 69.5 deg: no authority past this
float s_gun_agree_full = 0.70f;   // cos 45.6 deg: full authority within this
// m: full from 10 cm apart, none at 5 -- the hands coinciding, not the grips being close. In the ray
// mode the same 5 cm spun the weapon whenever the hands closed (reported: anything under 0.14 spun);
// that was the feedback loop above, which this mode does not have.
float s_gun_min_base   = 0.05f;

// The support hand's distance down the barrel is used AS MEASURED (you may grip anywhere along a
// handle, exactly as along a barrel) but never below this, so a hand sliding back past the aim grip
// cannot turn the grip-to-handle vector backwards. Past it the two vectors disagree and the gate
// fades the hold out instead. Metres.
constexpr float kGunMinAlong = 0.02f;

// ---- THE PUBLISHED SWING -----------------------------------------------------------------------
//
// Written on the game thread, read from the game thread, the XInput hook and the sim hook. A
// seqlock rather than a mutex: readers are on the frame path and must never block, and a torn
// quaternion would be a frame of wobble rather than a crash -- but five floats under an even/odd
// counter costs nothing and removes the argument entirely.
struct Swing {
    Quat  r{0.0f, 0.0f, 0.0f, 1.0f};
    bool  valid = false;
    bool  latched = false;
    float blend = 0.0f;
};

std::atomic<uint32_t> s_seq{0};
Swing                 s_swing{};

void publish(const Swing& s) {
    s_seq.fetch_add(1, std::memory_order_release);       // odd: write in progress
    s_swing = s;
    s_seq.fetch_add(1, std::memory_order_release);       // even again: readable
}

// Returns false if the snapshot was mid-write; callers treat that as "no contribution this call"
// rather than retrying, because one frame of one-handed aim is invisible and a spin on the sim
// thread is not.
bool read_swing(Swing* out) {
    const uint32_t before = s_seq.load(std::memory_order_acquire);
    if (before & 1u) return false;
    *out = s_swing;
    return s_seq.load(std::memory_order_acquire) == before;
}

char s_status[320] = "twohand: off";

// Sticky evidence for the status line. "Never true since load" and "false right now" are
// completely different diagnoses for a feature that does nothing, and only the sticky
// version can tell you the OpenXR grip action is simply not bound on this profile.
bool s_grip_ever = false;
bool s_zone_ever = false;

// The reach, for the grab guide. GAME THREAD ONLY -- written at the end of two_hand_update() and
// read by the rig block later in the same tick, so unlike the aim swing above it needs no seqlock:
// derive_ctrl_angles() (the ~2600 calls/sec sim-hook path) never touches it.
TwoHandReach s_reach{};
TwoHandZoneMeas s_zone_meas{};

// ---- PER-WEAPON DENY ---------------------------------------------------------------------------
//
// Halo has one-handed weapons -- the pistols, the Needler, the sword -- and putting a second hand
// on a barrel they do not have is not a feature. There is nothing to grab, so the "support hand"
// is just a hand somewhere in space, and the aim would swing toward it for a reason the player
// cannot see. That reads as the aim being broken, not as a mode they entered by accident.
//
// Matched as case-insensitive SUBSTRINGS of the weapon class name, the same rule wpnoff uses, for
// the same reason: the decorated names are long and a config nobody can type correctly is a config
// nobody uses.
bool weapon_denied(const char* cls) {
    if (cls == nullptr || cls[0] == 0) return false;      // nothing held -> nothing to deny
    const char* list = g_cfg.two_hand_deny;
    if (list[0] == 0) return false;

    // Walk the comma-separated list in place. No allocation: this runs every tick.
    const char* p = list;
    while (*p != 0) {
        while (*p == ',' || *p == ' ') ++p;
        const char* start = p;
        while (*p != 0 && *p != ',') ++p;
        size_t len = (size_t)(p - start);
        while (len > 0 && (unsigned char)start[len - 1] <= ' ') --len;   // trim trailing space/CR
        if (len == 0) continue;

        // Case-insensitive substring search of `start[0..len)` inside `cls`, ENDING ON A WORD
        // BOUNDARY.
        //
        // A bare substring test denied the wrong weapon, in the field: the shipped list carries
        // `Needler` for the one-handed Covenant pistol, and the NEEDLE RIFLE -- a two-handed
        // precision rifle that should absolutely get the hold -- is class
        // `FP_NeedleRifleWeaponActor_C`. Lowercased that is `fp_needlerifleweaponactor_c`, and
        // `needler` is the first seven letters of `needlerifle`. So the rifle silently lost
        // two-handed aim to an entry written for a different gun, with nothing logged.
        //
        // THE BOUNDARY IS ON THE RIGHT ONLY, and it is a camelCase boundary: the character after
        // the match must not be a lowercase letter. That is what separates `Needler` inside
        // `NeedlerWeaponActor` (next char `W`) and `Needler_WeaponActor` (next char `_`) -- both
        // real naming shapes in this game, and we have never held the Needler to learn which --
        // from `Needler` inside `NeedleRifle` (next char `i`, so it is the middle of a longer
        // word). End-of-string counts as a boundary.
        //
        // Left unanchored deliberately, so `BattleRifle` still matches `BP_FP_BattleRifle_...`
        // without anyone typing the prefix, and `PlasmaRifle`/`Sword`/`Magnum` keep working under
        // both conventions. The one thing this rule makes impossible is denying by a truncated
        // word (`Needle` no longer catches `Needler`) -- which is the ambiguity that caused this.
        for (const char* c = cls; *c != 0; ++c) {
            size_t i = 0;
            while (i < len && c[i] != 0 &&
                   std::tolower((unsigned char)c[i]) == std::tolower((unsigned char)start[i])) {
                ++i;
            }
            if (i == len) {
                const char nxt = c[len];   // safe: c[0..len) were all non-zero to get here
                if (!(nxt >= 'a' && nxt <= 'z')) return true;
            }
        }
    }
    return false;
}

// ---- HELPERS -----------------------------------------------------------------------------------

pa::Vec3 to_pa(const Vec3& v) { return {v.x, v.y, v.z}; }

// forward/left/up in raw VR space from a controller quaternion. Matches quat_forward()'s
// convention (Math.hpp) so the basis handed to the hold is the same direction derive_ctrl_angles()
// will bend.
pa::Mat3 vr_basis(const Quat& q) {
    const Vec3 f = quat_forward(q);
    const Vec3 u = quat_rotate(q, Vec3{0.0f, 1.0f, 0.0f});
    const Vec3 l = quat_rotate(q, Vec3{-1.0f, 0.0f, 0.0f});
    return pa::Mat3{to_pa(f), to_pa(l), to_pa(u)};
}

// An orthonormal basis whose FORWARD is `fwd` and whose UP is the aim controller's up with the
// forward component removed. effective_basis() derives the blended LEFT from this up, so roll still
// comes from the aim hand exactly as it does with the ray basis -- the support hand sets where the
// gun points, never how it is twisted. False, with `out` untouched, when `fwd` lies along the
// controller's up and no up can be defined; the caller keeps the ray for that tick.
bool forward_basis(const Vec3& fwd, const pa::Mat3& ray, pa::Mat3* out) {
    const pa::Vec3 f = pa::normalized(to_pa(fwd));
    if (!(pa::length_squared(f) > 0.8f)) return false;
    pa::Vec3 u = ray.up - f * pa::dot(ray.up, f);
    if (!(pa::length_squared(u) > 1.0e-4f)) return false;
    u = pa::normalized(u);
    const pa::Vec3 l = pa::cross(u, f);
    *out = pa::Mat3{f, l, pa::cross(f, l)};
    return true;
}

// Shortest-arc rotation taking `from` onto `to`. Identity (and false) for every degenerate case:
// failing to one-handed aim is a working game.
bool swing_between(const pa::Vec3& from, const pa::Vec3& to, Quat* out) {
    const pa::Vec3 a = pa::normalized(from);
    const pa::Vec3 b = pa::normalized(to);
    if (pa::length_squared(a) < 0.8f || pa::length_squared(b) < 0.8f) return false;

    const float d = pa::dot(a, b);
    if (!std::isfinite(d)) return false;
    if (d > 0.999999f) return false;             // already there; nothing to publish
    if (d < -0.999f)    return false;            // antipodal -- unreachable past the 0.35 agreement
                                                 // gate, rejected anyway rather than guessing an axis
    const pa::Vec3 axis = pa::cross(a, b);
    Quat q{axis.x, axis.y, axis.z, 1.0f + d};
    const float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(n) || n < 1.0e-6f) return false;
    *out = Quat{q.x / n, q.y / n, q.z / n, q.w / n};
    return std::isfinite(out->x) && std::isfinite(out->y) &&
           std::isfinite(out->z) && std::isfinite(out->w);
}

} // namespace

// ---- CONFIG ------------------------------------------------------------------------------------

bool two_hand_parse_key(const char* key, double v) {
    // The pa2h* spellings are the names these shipped under while the hold lived in the palette
    // folder. Kept as aliases rather than dropped: they are catalogued in halo_vr_dev.cfg and
    // somebody may have uncommented one.
    if      (_stricmp(key, "twohandmin")    == 0 || _stricmp(key, "pa2hmin")       == 0)
        s_tuning.zone_min_along_m = (float)v;
    else if (_stricmp(key, "twohandmax")    == 0 || _stricmp(key, "pa2hmax")       == 0)
        s_tuning.zone_max_along_m = (float)v;
    else if (_stricmp(key, "twohandradius") == 0 || _stricmp(key, "pa2hradius")    == 0)
        s_tuning.zone_radius_m = (float)v;
    else if (_stricmp(key, "twohandagreemin")  == 0 || _stricmp(key, "pa2hagreemin")  == 0)
        s_tuning.minimum_agreement = (float)v;
    else if (_stricmp(key, "twohandagreefull") == 0 || _stricmp(key, "pa2hagreefull") == 0)
        s_tuning.full_agreement = (float)v;
    // 1 = hold the gun at two points, measured from the weapon; 0 = the ray mode. See s_gun_mode.
    // The three below tune gun mode only, and the three ray keys tune the ray mode only.
    else if (_stricmp(key, "twohandgun") == 0)
        s_gun_mode = (v != 0.0) ? 1 : 0;
    else if (_stricmp(key, "twohandgunagreemin") == 0)
        s_gun_agree_min = (float)v;
    else if (_stricmp(key, "twohandgunagreefull") == 0)
        s_gun_agree_full = (float)v;
    else if (_stricmp(key, "twohandgunminbase") == 0)
        s_gun_min_base = (float)v;
    else if (_stricmp(key, "twohandblend")  == 0 || _stricmp(key, "pa2hblend")     == 0)
        s_tuning.blend_seconds = (float)v;
    else if (_stricmp(key, "twohandonemin") == 0)
        g_cfg.two_hand_onehand_min_m = (float)v;
    // Metres. 0 disables the band entirely and restores the old behaviour, which is worth keeping
    // reachable: it is the A/B that says whether a report of "the weapon spins" is this or not.
    else if (_stricmp(key, "twohandminbase") == 0)
        s_tuning.min_baseline_m = (float)v;
    else return false;
    return true;
}

// Back to the compiled defaults, ahead of every re-parse. Without this a DELETED key kept its last
// value until restart -- nothing else writes these, and load_config() only resets g_cfg -- so
// "remove the line to undo the experiment" silently did nothing, which is the one instruction every
// dev-cfg experiment ends with.
void two_hand_tuning_reset() {
    s_tuning         = pa::TwoHandTuning{};
    s_gun_mode       = 1;
    s_gun_agree_min  = 0.35f;
    s_gun_agree_full = 0.70f;
    s_gun_min_base   = 0.05f;
}

// ---- READERS -----------------------------------------------------------------------------------

float two_hand_blend_weight() {
    Swing s{};
    if (!read_swing(&s) || !s.valid) return 0.0f;
    return s.blend < 0.0f ? 0.0f : (s.blend > 1.0f ? 1.0f : s.blend);
}

bool two_hand_latched() {
    Swing s{};
    return read_swing(&s) && s.latched;
}

const TwoHandReach& two_hand_reach() { return s_reach; }

void two_hand_set_zone_measurement(const TwoHandZoneMeas& m) { s_zone_meas = m; }
const TwoHandZoneMeas& two_hand_zone_measurement() { return s_zone_meas; }

// ---- GRIP-OFFSET CALIBRATION ------------------------------------------------------------------
// Contract, and why the frozen weapon is already the right frame, are on the declarations.
// NO LATCH OF ITS OWN ANY MORE. The arming, the freeze and the save gestures all belong to
// g_menu_calib_mode (5 = this capture), so there is one arming authority instead of one per
// calibration -- which is what lets the LEFT/RIGHT trigger standard and the trigger SWALLOWING
// apply here without either being reimplemented.
//
// The freeze itself still comes from the rig's existing calibration hold: mode 5 holds it true,
// the rig snapshots on the rising edge, and the weapon is held in world space exactly as it is for
// a pose calibration. Nothing about the freeze is specific to this capture.
bool grip_offset_armed() {
    return g_menu_calib_mode.load(std::memory_order_relaxed) == 5;
}

bool grip_offset_capture() {
    // Called ONLY from the calibration falling edge, and only when the mode that started the
    // freeze was 5 -- so there is no arm flag of our own left to test. Every exit still returns
    // true: the caller reads it as "this release is spoken for" and raises no finish edge, which
    // is what keeps the weapon solve and the global write from running behind this gesture.

    // THE MEASUREMENT IS LAST TICK'S, AND THAT IS THE CORRECT ONE.
    //
    // This runs from the input poll, which is upstream of the rig block that publishes the zone
    // measurement, so s_zone_meas here is the one taken on the previous tick -- the last tick the
    // hold was still active and the weapon still frozen. Taking THIS tick's would measure against
    // a weapon that has already snapped back to following the hand.
    if (!s_zone_meas.valid) {
        API::get()->log_info("[Halo-CampE-UEVR] WPNGRIP: nothing captured -- no two-hand zone "
                             "measurement this tick. Both controllers must be tracking.");
        return true;
    }

    const std::string key = weapon_key();
    if (key.empty()) {
        API::get()->log_info("[Halo-CampE-UEVR] WPNGRIP: nothing captured -- no weapon in hand. "
                             "The offset is a property of a weapon, so there is nothing to store "
                             "it against.");
        return true;
    }

    const float off_y = s_zone_meas.hand_gun.y;
    const float off_z = s_zone_meas.hand_gun.z;
    const float at_x  = s_zone_meas.hand_gun.x;
    wpngrip_set(key, off_y, off_z, at_x);

    // SAY WHAT IT WILL DO, not just what it stored. The aim half of this feature is invisible
    // until you fire, and the number that predicts it -- atan(lateral/along) -- is exactly the
    // skew the capture just removed. Reporting it turns "did that work?" into a reading.
    const float lateral = std::sqrt(off_y * off_y + off_z * off_z);
    const float skew_deg = (std::fabs(at_x) > 1.0e-3f)
                         ? std::atan2(lateral, std::fabs(at_x)) * 57.2957795f : 0.0f;
    API::get()->log_info(
        "[Halo-CampE-UEVR] WPNGRIP: CAPTURED for '%s' -- handle sits %.1f cm off the barrel axis "
        "(y=%.1f z=%.1f), gripped %.1f cm along it. That reach was skewing the two-handed aim by "
        "%.1f deg; the grab zone now follows the handle%s. Delete the wpngrip line in "
        "halo_vr_weapons.cfg to undo.",
        key.c_str(), lateral, off_y, off_z, at_x, skew_deg,
        g_cfg.grip_fix_aim ? " and that skew is corrected" : " (gripfixaim=0, so aim is unchanged)");
    return true;
}

bool grip_offset_clear_current() {
    const std::string key = weapon_key();
    if (key.empty()) return false;
    if (!wpngrip_clear(key)) return false;
    API::get()->log_info("[Halo-CampE-UEVR] WPNGRIP: cleared for '%s'; it is held like a rifle "
                         "again.", key.c_str());
    return true;
}

const char* two_hand_status() { return s_status; }

bool two_hand_bend_orientation(Quat* q) {
    if (q == nullptr || !g_cfg.two_hand_rig) return false;
    Swing s{};
    if (!read_swing(&s) || !s.valid) return false;
    // Left-multiply: the swing is a WORLD rotation in VR space, so it composes onto the
    // controller's orientation from the left. Right-multiplying would rotate about the
    // controller's own axes, which is a different and wrong thing.
    *q = quat_mul(s.r, *q);
    return true;
}

bool two_hand_bend_forward(Vec3* fwd) {
    if (fwd == nullptr || !g_cfg.two_hand_aim) return false;
    Swing s{};
    if (!read_swing(&s) || !s.valid) return false;
    *fwd = quat_rotate(s.r, *fwd);
    return true;
}

bool two_hand_unbend_rig_forward(Vec3* fwd) {
    // Gated EXACTLY as two_hand_bend_orientation() is, so it removes precisely what the rig applied
    // this tick: with twohandrig off the rig bent nothing, and this must take nothing out.
    if (fwd == nullptr || !g_cfg.two_hand_rig) return false;
    Swing s{};
    if (!read_swing(&s) || !s.valid) return false;
    *fwd = quat_rotate(quat_conj(s.r), *fwd);
    return true;
}

void two_hand_reset(const char* why) {
    s_hold.reset();
    publish(Swing{});
    if (why != nullptr && g_cfg.two_hand) {
        API::get()->log_info("[Halo-CampE-UEVR] TWOHAND: reset (%s)", why);
    }
    std::snprintf(s_status, sizeof(s_status), "twohand: reset");
}

// ---- THE TICK ----------------------------------------------------------------------------------

void two_hand_update(float delta_seconds, bool gameplay_active, uint32_t tick) {
    // The kill switch outranks the feature, exactly as it does for the arm drivers. An aim
    // modifier must stop when the plugin stands down.
    if (!g_cfg.enabled || !g_cfg.two_hand) {
        if (s_hold.state().latched || s_hold.state().blend > 0.0f) two_hand_reset("disabled");
        else                                                       publish(Swing{});
        std::snprintf(s_status, sizeof(s_status), "twohand: off");
        return;
    }

    // ---- STAND DOWN WHILE A CALIBRATION IS HELD -------------------------------------------------
    //
    // A calibration gesture freezes the weapon so the player can line their controller up with it.
    // Two-hand aim is a SECOND driver of that same weapon's rotation, and nothing stopped it running
    // through the hold -- so the reference the player was aligning to was being moved by their off
    // hand while they aligned to it, and the solve measured against a moving target. On release the
    // two-hand solve re-derived the rotation immediately, which is the reported "the offset snaps the
    // shotgun rotation away".
    //
    // Reported 2026-09-06 on the shotgun, which is NOT in twohanddeny -- unlike the Magnum and the
    // plasma pistol, where the same gesture behaved. That is suggestive rather than conclusive (the
    // stored table had been rewritten by then), but the rule holds regardless of what the shotgun
    // turns out to be doing: YOU CANNOT CALIBRATE AGAINST A REFERENCE A SECOND DRIVER IS MOVING.
    // This is the same shape as the scope pane, where the per-weapon arm has to hold the pane open
    // precisely so two-handed aiming stops closing it mid-calibration.
    //
    // reset() rather than an early publish of the current swing: leaving a latched hold in place
    // would have it resume mid-gesture the moment the tick resumes, which is the artefact again with
    // extra steps. The player re-grips after calibrating, which re-latches normally.
    if (calib_hold_active()) {
        if (s_hold.state().latched || s_hold.state().blend > 0.0f) two_hand_reset("calibration held");
        else                                                       publish(Swing{});
        std::snprintf(s_status, sizeof(s_status), "twohand: standing down (calibration held)");
        return;
    }

    // ONE-HANDED WEAPONS NEVER TWO-HAND. Checked before the poses are read, so a denied weapon
    // costs nothing at all. The class name is published by weapon_offset_update(), which already
    // resolved it this frame -- see WeaponOffset.hpp for why it is not re-derived here.
    const char* cls = weapon_offset_current_class();
    // DENIED MEANS "NO AIM AUTHORITY", NOT "NO GRIP". Corrected 2026-09-04.
    //
    // It used to return here, before the poses were even read, so a Magnum had no hold state at
    // all. That was right while the hold's only job was bending aim. It is wrong now: the grip is
    // a MODE the rest of the mod reads. The off-hand trigger throws a grenade when you are not
    // gripping and toggles zoom when you are, so a weapon you cannot grip is a weapon whose zoom
    // is unreachable -- and the one-handers are exactly the weapons that have a zoom.
    //
    // So a denied weapon runs the hold completely -- zone, latch, blend, haptics, and the grab
    // guide with it -- and only the SWING is withheld. `latched` is published so two_hand_latched()
    // reads true; `valid` stays false so two_hand_bend_orientation()/_forward() contribute nothing
    // and the shot goes exactly where a one-handed shot went before.
    //
    // Player IK wants the same split later: attach the off hand to a pistol without bending aim.
    const bool deny_aim = weapon_denied(cls);
    // SAY WHICH WEAPON WAS DENIED, AND SAY IT ON CHANGE. A withheld swing is indistinguishable
    // from a hold that never latched -- both present as "two-handed aim does nothing on this gun"
    // -- and the deny list is the one cause with no other symptom. The Needle Rifle collision
    // above went unnoticed precisely because nothing named the weapon it had refused.
    {
        static char s_said_cls[128] = {0};
        static bool s_said_deny = false;
        const char* name = (cls != nullptr) ? cls : "";
        if (deny_aim != s_said_deny || std::strncmp(name, s_said_cls, sizeof(s_said_cls) - 1) != 0) {
            s_said_deny = deny_aim;
            std::strncpy(s_said_cls, name, sizeof(s_said_cls) - 1);
            s_said_cls[sizeof(s_said_cls) - 1] = 0;
            if (deny_aim) {
                API::get()->log_info(
                    "[Halo-CampE-UEVR] twohand: aim swing WITHHELD on %s -- it matches an entry in "
                    "twohanddeny=\"%s\". The hold still runs (zone, latch, haptics, grab guide); "
                    "only the aim bend is suppressed. If this weapon is two-handed, remove or "
                    "narrow that entry.",
                    name[0] != 0 ? name : "(nothing held)", g_cfg.two_hand_deny);
            }
        }
    }

    // Device indices from the runtime, NOT the literals 0/1. In UEVR index 0 is the HMD -- the
    // whole rest of the plugin resolves controllers this way (MotionAimControl.cpp:458,
    // Plugin.cpp:5482), and reading a raw 0 here made the "support hand" the headset. The
    // hand-to-hand line was (head - aim controller), never in the barrel zone, so the hold never
    // latched and the feature did nothing at all. The literals were inherited from PaletteArm.cpp,
    // which has never run, so nothing had exercised them.
    const int32_t aim_idx     = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                                    : API::VR::get_right_controller_index();
    const int32_t support_idx = g_cfg.aim_left_hand ? API::VR::get_right_controller_index()
                                                    : API::VR::get_left_controller_index();
    if (aim_idx < 0) {
        publish(Swing{});
        std::snprintf(s_status, sizeof(s_status), "twohand: no aim controller index");
        return;
    }

    // Grip pose for POSITION, aim pose for DIRECTION. Not interchangeable: Plugin.cpp records that
    // get_aim_pose() returns teleport-scale translations, so its rotation is usable and its
    // position is not.
    Vec3 aim_grip_pos{};  Quat aim_grip_rot{};
    Vec3 support_pos{};   Quat support_rot{};
    Quat canonical_rot{};

    if (!get_pose(aim_idx, &aim_grip_pos, &aim_grip_rot, /*use_aim=*/false)) {
        // No aim hand, no hold. Publish invalid rather than leaving a stale swing standing --
        // failure 3 in the design: never publish a stale R on a failed tick.
        publish(Swing{});
        std::snprintf(s_status, sizeof(s_status), "twohand: no aim pose");
        return;
    }
    {
        Vec3 p{}; Quat q{};
        // Take the canonical direction from whichever source derive_ctrl_angles() will use, so the
        // basis the agreement gate is measured against is the one that actually gets bent.
        const bool want_grip_dir = (g_cfg.aim_src == 1);
        if (get_pose(aim_idx, &p, &q, /*use_aim=*/!want_grip_dir)) canonical_rot = q;
        else                                                       canonical_rot = aim_grip_rot;
    }

    const bool support_tracked =
        get_pose(support_idx, &support_pos, &support_rot, /*use_aim=*/false);

    pa::TwoHandInput in{};
    in.aim_grip_position     = to_pa(aim_grip_pos);
    in.aim_basis             = vr_basis(canonical_rot);
    in.support_grip_position = to_pa(support_pos);
    in.support_tracked       = support_tracked;
    in.gameplay_active       = gameplay_active;
    in.delta_seconds         = delta_seconds;

    // The support-hand grip. Retried until the runtime has built its action set -- caching a
    // permanent null here would silently kill the feature and it would read as "does nothing".
    static UEVR_ActionHandle s_grip = nullptr;
    if (s_grip == nullptr) s_grip = API::VR::get_action_handle("/actions/default/in/Grip");
    const auto grip_src = g_cfg.aim_left_hand ? API::VR::get_right_joystick_source()
                                              : API::VR::get_left_joystick_source();
    if (g_cfg.bind_two_hand != 0) {
        // The XInput route: the same mechanism the reload gesture uses, which is proven on this
        // profile. Sees a button, not a side -- the zone is what makes that safe.
        const unsigned short btn = g_pad_buttons.load(std::memory_order_relaxed);
        in.support_grip_held = (btn & (unsigned short)g_cfg.bind_two_hand) != 0;
    } else {
        in.support_grip_held = s_grip != nullptr && API::VR::is_action_active(s_grip, grip_src);
    }
    if (in.support_grip_held) s_grip_ever = true;

    // THE ZONE IS MEASURED IN THE GUN'S FRAME, by the rig block earlier in this same tick. Along =
    // down the barrel, lateral = off it. Handing the hold these two scalars is what moves the grab
    // cylinder off the controller's ray and onto the rifle -- see TwoHandInput::zone_measured.
    if (s_zone_meas.valid) {
        // GAME CENTIMETRES -> PHYSICAL METRES, and the divisor is rig_scale, NOT 100.
        //
        // This shipped as `* 0.01f` and broke two-handed aiming outright. vr_to_rig() scales by
        // rig_scale, which is documented as "cm of rig movement per metre of hand movement" and
        // MUST INCLUDE UEVR's world scale -- 131.2 on the shipped profile, not 100. So a plain
        // 0.01 inflated every distance by 31%: the 9 cm grab radius behaved like 6.9 cm and the
        // hold simply stopped latching. The zone bounds are authored in PHYSICAL metres (see
        // TwoHandTuning), which is the frame a person can reason about, so that is what has to
        // arrive here.
        const float cm_per_m = (g_cfg.rig_scale > 1.0f) ? g_cfg.rig_scale : 100.0f;
        in.zone_measured  = true;
        in.zone_along_m   = s_zone_meas.hand_gun.x / cm_per_m;

        // ---- THE HANDLE OFFSET MOVES THE CYLINDER'S AXIS, NOT ITS ENDS ------------------------
        //
        // Subtracting it before the magnitude is what puts the grab cylinder along the HANDLE for
        // a weapon whose front grip is off the barrel -- a rocket launcher, a sentinel beam --
        // instead of along a barrel line the player's hand never occupies. Zero for every weapon
        // without an entry, so this is arithmetic on a 0 rather than a branch in the hot path.
        //
        // ONLY y/z. `along` is untouched because it is a permitted RANGE, not a point: you may
        // grip anywhere down the handle's length, exactly as you may down a barrel.
        const float gy = s_zone_meas.grip_off_valid ? s_zone_meas.grip_off_gun.y : 0.0f;
        const float gz = s_zone_meas.grip_off_valid ? s_zone_meas.grip_off_gun.z : 0.0f;
        const float dy = s_zone_meas.hand_gun.y - gy;
        const float dz = s_zone_meas.hand_gun.z - gz;
        in.zone_lateral_m = std::sqrt(dy * dy + dz * dz) / cm_per_m;
    }

    // ---- WHAT THE SWING TURNS, AND ONTO WHAT ---------------------------------------------------
    //
    // GUN MODE (twohandgun=1, the default; see s_gun_mode): FROM is the one-handed gun's own
    // grip-to-handle vector, TO is the raw line between the hands, and nothing is subtracted from
    // the support hand. effective_basis() measures agreement against, and swings from, the aim
    // basis's forward -- so handing it FROM as that forward is the entire change on the hold's side.
    // The hold itself (TwoHand.cpp, unit-tested) is untouched.
    //
    // Both vectors are metres in RAW VR space. The gun's axes arrive UNSWUNG from the rig block, so
    // FROM cannot depend on the swing it is about to produce.
    bool gun_mode = false;
    {
        // The support hand's distance down the barrel, kept for a tick where it is not tracked: the
        // hold then steers from its remembered line, and FROM has to stay the vector it was measured
        // against or the agreement jumps. Rebuilt against the LIVE gun axes, never stored in world
        // space, so it still turns with the aim hand.
        static float s_last_along = 0.30f;
        if (s_gun_mode != 0 && s_zone_meas.gun1_valid) {
            const Vec3& gx = s_zone_meas.gun1_x;
            const Vec3& gy = s_zone_meas.gun1_y;
            const Vec3& gz = s_zone_meas.gun1_z;
            float along = s_last_along;
            if (support_tracked) {
                const Vec3 d{support_pos.x - aim_grip_pos.x, support_pos.y - aim_grip_pos.y,
                             support_pos.z - aim_grip_pos.z};
                along = d.x * gx.x + d.y * gx.y + d.z * gx.z;
                if (!std::isfinite(along) || along < kGunMinAlong) along = kGunMinAlong;
                s_last_along = along;
            }
            // Out to the handle: the wpngrip y/z, game centimetres -> metres through rig_scale (the
            // same divisor the zone uses, and for the same reason). Zero with no entry, which makes
            // FROM the barrel itself; gripfixaim=0 leaves the handle out of aim, as it always has.
            const float cm_per_m = (g_cfg.rig_scale > 1.0f) ? g_cfg.rig_scale : 100.0f;
            const bool  handle   = s_zone_meas.grip_off_valid && g_cfg.grip_fix_aim;
            const float oy = handle ? s_zone_meas.grip_off_gun.y / cm_per_m : 0.0f;
            const float oz = handle ? s_zone_meas.grip_off_gun.z / cm_per_m : 0.0f;
            const Vec3 from{gx.x * along + gy.x * oy + gz.x * oz,
                            gx.y * along + gy.y * oy + gz.y * oz,
                            gx.z * along + gy.z * oy + gz.z * oz};
            const pa::Mat3 ray_basis = in.aim_basis;
            gun_mode = forward_basis(from, ray_basis, &in.aim_basis);
        }
    }

    // ---- RAY MODE ONLY: THE HANDLE OFFSET, SUBTRACTED FROM THE SUPPORT HAND --------------------
    //
    // effective_basis() takes the gun's forward to be normalized(support - aim). For an off-axis
    // handle that line is NOT the barrel, and `along` does not cancel the error -- it scales it:
    // atan(lateral / along), so a 10 cm handle held 40 cm out points the weapon 14 degrees off.
    // Nothing downstream damps it either, because the agreement band ships fully open.
    //
    // Subtracting the handle's REST offset reconstructs where the hand would sit on an equivalent
    // rifle. At rest the weapon points down its own barrel; move the support hand and it still
    // steers by exactly the angle a rifle would give, because only the BASELINE moved onto the
    // handle. That is why this is a subtraction and not a clamp toward the axis.
    //
    // SUPERSEDED BY GUN MODE, and kept whole for the A/B. It has two faults the gun mode was built to
    // remove: it rotates the offset with the ALREADY-SWUNG gun (grip_off_vr), which feeds each swing
    // back into the next tick's hand line; and it steers an off-axis handle by atan(d/along), as if
    // the handle sat on the barrel, where a rigid gun turns by d over the full grip-to-handle reach.
    //
    // Its own switch (gripfixaim) because this one is on the aim path and the zone half is not.
    if (!gun_mode && s_zone_meas.grip_off_valid && g_cfg.grip_fix_aim) {
        const pa::Vec3 fix = to_pa(s_zone_meas.grip_off_vr);
        in.support_grip_position.x -= fix.x;
        in.support_grip_position.y -= fix.y;
        in.support_grip_position.z -= fix.z;
    }

    // ---- ONE-HANDED WEAPONS GRIP AT THE HAND, NOT ALONG A BARREL ------------------------------
    //
    // The zone's near edge is 8 cm FORWARD of the firing grip, which is right for a rifle: you
    // reach out to a foregrip. It is wrong for a Magnum, and wrong in the way that matters -- you
    // cup the pistol at or just under your firing hand, which is `along` around zero or slightly
    // NEGATIVE, so the most natural hold was the one position that could never register.
    //
    // So for a deny-aim weapon the near edge moves back behind the grip. The far edge stays put:
    // a hand a long way down an imaginary barrel that is not there should still not latch.
    //
    // A LOCAL COPY, not a mutation of s_tuning. Writing the relaxed value into the shared tuning
    // would leak the pistol's zone onto the next rifle you picked up, and it would do so silently
    // because nothing re-reads the config on a weapon swap.
    pa::TwoHandTuning tuning = s_tuning;
    if (deny_aim) tuning.zone_min_along_m = g_cfg.two_hand_onehand_min_m;
    // Gun mode's gates are its own; see the note on s_gun_mode for why they are not shared.
    if (gun_mode) {
        tuning.minimum_agreement = s_gun_agree_min;
        tuning.full_agreement    = s_gun_agree_full;
        tuning.min_baseline_m    = s_gun_min_base;
    }

    // SAY WHICH HOLD IS RUNNING, ON CHANGE -- of the CONFIGURATION, not of this tick's fallback, so a
    // tick with no rig to measure cannot make it chatter.
    {
        static int   s_said_mode = -1;
        static float s_said_min = 0.0f, s_said_full = 0.0f, s_said_base = 0.0f;
        const float m = s_gun_mode ? s_gun_agree_min  : s_tuning.minimum_agreement;
        const float f = s_gun_mode ? s_gun_agree_full : s_tuning.full_agreement;
        const float b = s_gun_mode ? s_gun_min_base   : s_tuning.min_baseline_m;
        if (s_said_mode != s_gun_mode || s_said_min != m || s_said_full != f || s_said_base != b) {
            s_said_mode = s_gun_mode; s_said_min = m; s_said_full = f; s_said_base = b;
            const auto deg = [](float c) {
                return std::acos(c < -1.0f ? -1.0f : (c > 1.0f ? 1.0f : c)) * 57.2957795f;
            };
            API::get()->log_info(
                "[Halo-CampE-UEVR] TWOHAND: %s. Authority: full within %.0f deg, none past %.0f deg; "
                "fades out as the hands close from %.0f cm to %.0f cm.",
                s_gun_mode ? "holding the GUN at two points (twohandgun=1) -- a hold where the gun "
                             "already sits moves nothing"
                           : "RAY mode (twohandgun=0) -- the controller's aim ray turns onto the "
                             "hands, as before 2026-09-13",
                deg(f), deg(m), b * 200.0f, b * 100.0f);
        }
    }

    const pa::TwoHandState st = s_hold.update(in, tuning);

    // ---- PUBLISH THE REACH, for the grab guide -----------------------------------------------
    //
    // Everything the guide needs is already computed here and nowhere else: the two grip
    // positions, the aim ray they are measured against, and the zone bounds. Re-deriving any of
    // it in the drawing code would be a second copy of the zone geometry, and the first time
    // someone retuned twohandalong the guide would start pointing at a spot that no longer
    // latches -- an affordance that lies is worse than none.
    //
    // VR SPACE, IN METRES, exactly as the hold sees it. The conversion to game space belongs to
    // Plugin.cpp's vr_to_rig(), which is the single blessed copy of that transform.
    {
        TwoHandReach r{};
        r.valid   = st.measured;
        r.in_zone = st.in_zone;
        r.latched = st.latched;
        r.along_m   = st.along_m;
        r.lateral_m = st.lateral_m;
        if (st.measured) {
            // THE NEAREST GRABBABLE POINT DOWN THE BARREL: the hand's own `along`, clamped into
            // the zone. Inside the zone that is the foot of the perpendicular, so the guide is a
            // short stub from hand to barrel that shrinks to nothing as the hand closes on the
            // axis. It is a point that WOULD latch, which is the whole claim the guide makes.
            //
            // Only the SCALAR is published now. The endpoints used to be vectors derived here in
            // VR space and then converted twice by the caller; since the zone is measured in the
            // gun's frame, the target is simply (along, 0, 0) in that frame and the caller can
            // build it without a conversion at all. One number cannot disagree with itself.
            r.clamped_along_m = (st.along_m < tuning.zone_min_along_m) ? tuning.zone_min_along_m
                              : (st.along_m > tuning.zone_max_along_m) ? tuning.zone_max_along_m
                                                                         : st.along_m;
        }
        s_reach = r;
    }

    // ---- ENTERING THE ZONE BUZZES, separately from latching ---------------------------------
    //
    // Field report: "I had a hard time finding it after a level load." The beam only appears once
    // you are already in range, so it cannot help you get there -- and after a load your hands are
    // wherever you left them, with no arms drawn to tell you. A short tick on crossing INTO the
    // zone gives the range a boundary you can feel, so finding the barrel stops being a hunt.
    //
    // Distinctly lighter and shorter than the latch buzz below (0.04s/0.25 against 0.12s/0.7):
    // these two events happen seconds apart and must not feel like the same thing, or "am I in
    // range" and "am I holding it" become indistinguishable through the controller.
    //
    // Edge-triggered on the way IN only. Leaving the zone is not worth a buzz -- during a hold the
    // hand drifts across the boundary constantly (the zone gates acquisition, never retention),
    // and buzzing on every crossing would be a rattle.
    {
        static bool s_zone_prev = false;
        if (st.in_zone && !s_zone_prev && !st.latched) {
            API::VR::trigger_haptic_vibration(0.0f, 0.04f, 0.0f, 0.25f,
                                              g_cfg.aim_left_hand ? API::VR::get_right_joystick_source()
                                                                  : API::VR::get_left_joystick_source());
        }
        s_zone_prev = st.in_zone;
    }

    if (st.latch_changed) {
        // ABI order: (seconds_from_now, duration, frequency, amplitude). The C++ wrapper's
        // parameter NAMES disagree with that order -- these are ordered for the ABI, not the names.
        API::VR::trigger_haptic_vibration(0.0f, st.latched ? 0.12f : 0.06f, 0.0f,
                                          st.latched ? 0.7f : 0.35f,
                                          g_cfg.aim_left_hand ? API::VR::get_right_joystick_source()
                                                              : API::VR::get_left_joystick_source());

        // The blend moves the aim for a reason the stick did not cause, so the gain estimator
        // would read the ramp as plant response and mis-measure. The suppression already exists
        // for reference recaptures; this is the same argument.
        g_gain_hold = true;
        g_gain_hold_until = tick + 90;

        HALO_VR_DEV_ONLY(
            if (g_cfg.two_hand_log) {
                API::get()->log_info("[Halo-CampE-UEVR] TWOHAND: %s (zone=%d blend=%.2f)",
                                     st.latched ? "GRABBED" : "released",
                                     st.in_zone ? 1 : 0, st.blend);
            });
    }

    // ---- THE BASELINE SAMPLER. For "the weapon yaws when my hands cross".
    //
    // PERIODIC WHILE LATCHED, and periodic is the point: the fault is a number MOVING, and the
    // only samples an edge-triggered line can produce are the ones at its own threshold -- which
    // is how a GRABGUIDE report was misread as pinned endpoints on 2026-09-08. Reporting the
    // separation, the authority it earned and the geometry it came from makes the difference
    // between "the band is doing its job" and "the band is the cause" readable rather than argued.
    //
    // WHAT TO LOOK FOR. baseline oscillating across min_baseline_m with w swinging 0..1 means the
    // aim is being handed back and forth between one- and two-handed, and the fade band is the
    // jitter rather than the cure -- in which case the offset needs to fade with distance from the
    // HANDLE instead, because a fixed offset is extrapolation once the hand leaves it.
    // A steady w with the weapon still yawing means the fault is elsewhere and this is exonerated.
    //
    // Sampled AFTER the swing is published (below), so it can report the swing and the agreement
    // that produced it: the "two ghost images" report was the swing alternating between two values
    // on successive ticks, which only a line carrying the swing itself can show.

    // THE ONE CALL. Nothing else in the plugin may call effective_basis(); see TwoHandAim.hpp.
    const pa::Mat3 canonical = in.aim_basis;
    const pa::Mat3 blended   = s_hold.effective_basis(canonical, in.aim_grip_position,
                                                      in.support_grip_position,
                                                      support_tracked, tuning);

    Swing out{};
    out.latched = st.latched;
    out.blend   = st.blend;
    // deny_aim withholds the SWING ONLY. The latch above is still published, because on a
    // one-handed weapon the grip is what makes the off-hand trigger a zoom toggle instead of a
    // grenade. valid=false means every bend site leaves its argument alone, so the shot is
    // bit-identical to one-handed. See the deny note further up.
    out.valid   = !deny_aim && swing_between(canonical.forward, blended.forward, &out.r);
    if (!out.valid) out.r = Quat{0.0f, 0.0f, 0.0f, 1.0f};
    publish(out);

#if HALO_VR_DEV
    // THE BASELINE SAMPLER itself -- see the note above the one call.
    if (g_cfg.two_hand_log && st.latched) {
        static uint32_t s_last = 0;
        if (tick - s_last >= 16u) {
            s_last = tick;
            const float agree = pa::dot(pa::normalized(in.support_grip_position - in.aim_grip_position),
                                        pa::normalized(canonical.forward));
            const float w_abs = std::fabs(out.r.w) > 1.0f ? 1.0f : std::fabs(out.r.w);
            const float swing_deg = out.valid ? 2.0f * std::acos(w_abs) * 57.2957795f : 0.0f;
            API::get()->log_info(
                "[Halo-CampE-UEVR] TWOHAND BASE: %s sep=%.3fm w=%.2f blend=%.2f agree=%.2f "
                "swing=%.1fdeg | along=%.3f lat=%.3f | gripoff=%d fixaim=%d",
                gun_mode ? "gun" : "ray", st.baseline_m, st.baseline_w, st.blend, agree, swing_deg,
                st.along_m, st.lateral_m,
                s_zone_meas.grip_off_valid ? 1 : 0, g_cfg.grip_fix_aim ? 1 : 0);
        }
    }
#endif

    if (st.in_zone) s_zone_ever = true;

    // ONE LINE THAT ANSWERS "why is it doing nothing". Each field rules out one hypothesis:
    //   griphandle=0  -> the OpenXR grip action is not bound on this profile. Nothing else matters;
    //                    every other read in this plugin uses the XInput mask instead.
    //   gripever=0    -> the handle resolved but the button has NEVER read true since load.
    //   along/lat     -> where the support hand actually is relative to the barrel, in metres,
    //                    against the zone it is being tested against. If these look sane and
    //                    zoneever=0, the zone bounds are wrong, not the tracking.
    //   support=0     -> the off-hand pose is not arriving at all.
    std::snprintf(s_status, sizeof(s_status),
                  "twohand: %s blend=%.2f | griphandle=%d gripever=%d griphold=%d "
                  "support=%d along=%.3fm lat=%.3fm (zone %.2f..%.2f r%.2f) zone=%d zoneever=%d "
                  "swing=%d gameplay=%d bind=0x%04X denyaim=%d wpn=%s",
                  st.latched ? "HELD" : (st.in_zone ? "in-zone" : "idle"), st.blend,
                  s_grip != nullptr ? 1 : 0, s_grip_ever ? 1 : 0, in.support_grip_held ? 1 : 0,
                  support_tracked ? 1 : 0,
                  st.measured ? st.along_m : -999.0f, st.measured ? st.lateral_m : -999.0f,
                  tuning.zone_min_along_m, tuning.zone_max_along_m, tuning.zone_radius_m,
                  st.in_zone ? 1 : 0, s_zone_ever ? 1 : 0, out.valid ? 1 : 0,
                  gameplay_active ? 1 : 0, (unsigned)g_cfg.bind_two_hand, deny_aim ? 1 : 0,
                  (cls != nullptr && cls[0] != 0) ? cls : "<none>");
}

} // namespace halo
