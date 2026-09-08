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
    else if (_stricmp(key, "twohandblend")  == 0 || _stricmp(key, "pa2hblend")     == 0)
        s_tuning.blend_seconds = (float)v;
    else if (_stricmp(key, "twohandonemin") == 0)
        g_cfg.two_hand_onehand_min_m = (float)v;
    else return false;
    return true;
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
        in.zone_lateral_m = std::sqrt(s_zone_meas.hand_gun.y * s_zone_meas.hand_gun.y +
                                      s_zone_meas.hand_gun.z * s_zone_meas.hand_gun.z) / cm_per_m;
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
