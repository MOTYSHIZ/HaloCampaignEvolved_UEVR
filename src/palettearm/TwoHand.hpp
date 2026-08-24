// The two-handed hold: support hand on the barrel, aim along the line between the hands.
//
// PURE, and stateful in ONE explicit object. No globals, no clock, no input API -- the caller
// feeds it poses and a button, it returns a latch and an aim basis. That is what lets
// Scripts\Verify-PaletteArm.ps1 drive a whole grab-and-release sequence deterministically with no
// game attached, which is the only way the hysteresis below can be tested at all.
//
// Ported from elliotttate's HaloCampaignEvolved-UEVR (main.cpp @ 62ee34f) with permission; the
// mechanism is credited there to Halo-MCC-VR's headset-tuned barrel grab.

#pragma once

#include "PaletteMath.hpp"

namespace halo::palettearm {

struct TwoHandTuning {
    // The grab zone is a thin cylinder along the aim hand's ray, measured from the aim grip: this
    // far forward, within this radius. Roughly "where the barrel is".
    float zone_min_along_m = 0.08f;
    float zone_max_along_m = 0.80f;
    float zone_radius_m    = 0.09f;

    // How much the hand-to-hand line has to agree with the one-handed aim ray before it gets any
    // authority, and where it gets full authority. Halo-MCC-VR hard-gates at 0.35; the band is
    // elliotttate's change and it matters -- a hard cutoff snaps the weapon about 70 degrees when
    // a LATCHED support hand crosses the boundary, because the hold persists across a threshold
    // the aim does not.
    float minimum_agreement = 0.35f;
    float full_agreement    = 0.50f;

    // Seconds for the influence to fade fully in or out.
    float blend_seconds = 0.15f;
};

// Everything the hold needs to know about this frame. All poses in ONE consistent frame -- the
// caller composes them; this does not know what space it is handed.
struct TwoHandInput {
    Vec3 aim_grip_position{};       // the weapon hand
    Mat3 aim_basis{};               // its orientation; forward is the aim ray
    Vec3 support_grip_position{};   // the other hand
    bool support_tracked = false;   // false when the support controller drops tracking
    bool support_grip_held = false; // the grip button on the support hand
    bool gameplay_active = false;   // false in menus, cutscenes, pause -- forces a release
    float delta_seconds = 0.0f;
};

// What the hold decided.
struct TwoHandState {
    bool  latched = false;      // the hold is engaged
    bool  in_zone = false;      // the support hand is on the barrel (whether or not it is latched)
    float blend   = 0.0f;       // 0..1, eased; the actual authority the hand-to-hand line has
    bool  latch_changed = false; // this frame -- the caller's cue to fire a haptic buzz
};

// The hold, as an object so its state is visible and testable rather than ambient.
class TwoHandHold {
public:
    // Advance one frame. Returns the new state.
    //
    // THE ZONE ONLY GATES ACQUISITION, NEVER RETENTION. Once the grip button latches the hold, it
    // persists until the button releases, even if the hand wanders outside the cylinder. That is
    // deliberate: a hold that drops when your support hand drifts 10 cm is worse than no hold,
    // and the player has an unambiguous way to end it.
    TwoHandState update(const TwoHandInput& input, const TwoHandTuning& tuning);

    // The aim basis to actually use: the one-handed basis eased onto the hand-to-hand line.
    //
    // FEED EVERY AIM CONSUMER FROM THIS ONE CALL. The rendered barrel, the muzzle ray and the
    // projectile spawn must all use the same basis or the gun points somewhere the shots do not
    // go -- and a second copy of this blend will drift from the first. See README.md.
    //
    // `support_tracked` false eases the tail out along the LAST tracked line rather than snapping
    // back to one-handed aim, so a momentary tracking drop mid-hold is not a flick of the weapon.
    Mat3 effective_basis(const Mat3& one_hand_basis,
                         const Vec3& aim_grip_position,
                         const Vec3& support_grip_position,
                         bool support_tracked,
                         const TwoHandTuning& tuning) const;

    // Drop everything -- latch, blend and the remembered line. Call when the driver is switched
    // off or on any transition that invalidates the poses.
    void reset();

    const TwoHandState& state() const { return m_state; }

private:
    TwoHandState m_state{};
    // Last usable hand-to-hand direction, for the ease-out when support tracking drops.
    Vec3 m_last_forward{};
    bool m_has_last_forward = false;
};

} // namespace halo::palettearm
