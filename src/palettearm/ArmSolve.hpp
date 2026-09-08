// Shoulder anchoring, two-bone arm IK, and wrist placement, applied to the Blam node palette.
//
// PURE. No UEVR, no Config, no Windows, no globals -- palette in, palette mutated, bool out. That
// is what lets Scripts\Verify-PaletteArm.ps1 test these without a game, so keep it that way.
//
// ALL DISTANCES ARE BLAM UNITS in the signatures here, because that is what the palette holds.
// Constants authored in metres are converted at the point of use via kMetresPerBlamUnit.
//
// Ported from elliotttate's HaloCampaignEvolved-UEVR (main.cpp @ 62ee34f) with permission; the
// mechanism is credited there to RoboquestVR's arm rig.

#pragma once

#include "NodeMap.hpp"
#include "PaletteMath.hpp"

#include <cstdint>

namespace halo::palettearm {

// ---- TUNING ------------------------------------------------------------------------------------
// Starting values, not calibration. They describe where a shoulder sits relative to a head, which
// is a person-shaped question, so expect to move them in a headset.

struct ArmTuning {
    // Shoulder offsets from the head, in a YAW-ONLY head frame. Yaw-only is the whole trick: head
    // pitch and roll never swing the arm root, which is the main reason these arms do not cross
    // the player's face when they look down.
    float shoulder_back_m    = 0.16f;
    float shoulder_down_m    = 0.22f;
    float shoulder_lateral_m = 0.17f;

    // When the tracked hand is past the authored arm's reach, slide the arm ROOT toward the target
    // by up to this much before clamping the remainder. Roboquest gets the same effect from a
    // FABRIK chain rooted at the clavicle; this is the cheap version of a shoulder roll.
    float clavicle_assist_m = 0.12f;

    // The wrist bone belongs BEHIND the controller's grip point. The OpenXR grip pose sits at the
    // palm centroid, so placing the wrist there puts the knuckles inside the gun.
    float grip_to_wrist_back_m = 0.08f;
    float grip_to_wrist_down_m = 0.02f;

    // How much of the elbow pole comes from a FIXED BODY DIRECTION rather than from the animated
    // stock pose. 1 = purely body, 0 = purely the authored bend.
    //
    // WHY THIS EXISTS. Taking the pole from the current elbow preserves the authored bend, but that
    // elbow is the GAME animated pose expressed in a CAMERA-LOCAL palette -- so the pole azimuth
    // rotates whenever the camera does, and our camera is driven by the aim hand. Measured with the
    // support hand pinned and the aim hand rotating: the elbow SWIVELS about the shoulder-to-wrist
    // axis by up to 32.8 deg in a single frame, while its distance off that axis (perp) and the
    // joint angle (bend) stay put. Hand steady, shoulder steady, elbow rolling between them -- the
    // reported "the bones between the hand and the shoulders are in constant jitter".
    //
    // 0.75 follows Pancreations MCC VR, which weights its pole 75 percent body / 25 percent animated
    // and applies NO smoothing anywhere in its arm chain -- i.e. it makes the pole stable at the
    // source instead of filtering the symptom. A filter here would add lag to the hand for a problem
    // that is not noise.
    float pole_body_fraction = 0.75f;
};

// ---- PRIMITIVES --------------------------------------------------------------------------------

// Rotate `nodes` rigidly about `pivot`, renormalising each basis as it goes.
void apply_rigid_delta(BlamMatrix4x3* palette, const std::uint8_t* nodes, std::size_t count,
                       const Mat3& rotation, const Vec3& pivot);

// Translate `nodes` by `offset`. No rotation, no renormalisation.
void apply_rigid_offset(BlamMatrix4x3* palette, const std::uint8_t* nodes, std::size_t count,
                        const Vec3& offset);

// Apply an ABSOLUTE rigid transform: each position becomes `translation + basis * position`, and
// each axis is rotated by `basis`.
//
// DISTINCT FROM apply_rigid_delta(), which rotates ABOUT A PIVOT -- that is what a joint does, and
// it is the right tool when the thing you know is the centre of rotation. This is the whole-body
// form, for carrying a subtree from its authored pose onto a destination given directly, with no
// pivot involved. The weapon uses it: the controller says where the gun goes, not what it rotates
// around.
void apply_rigid_transform(BlamMatrix4x3* palette, const std::uint8_t* nodes, std::size_t count,
                           const Mat3& basis, const Vec3& translation);

// A yaw-only frame built from the palette root, for hanging shoulders off. Takes the root's up
// axis as vertical and flattens its forward against that.
Mat3 torso_basis_from_root(const Mat3& root_basis);

// ---- THE SOLVERS -------------------------------------------------------------------------------

// Slide the whole arm subtree so its shoulder joint sits where a shoulder should, relative to the
// head. Translation only -- the IK below does the rotating.
//
// `head_position` is the palette root's position (node 0), which is the camera-derived frame.
bool anchor_shoulder_to_torso(BlamMatrix4x3* palette, const ArmNodes& arm, const Mat3& torso_basis,
                              const Vec3& head_position, bool left_side, const ArmTuning& tuning);

// Place the wrist and everything below it exactly, rigidly, ignoring the arm above. This is the
// "floating hands" mode: no IK, the forearm simply is not drawn where it should be.
bool place_wrist_subtree(BlamMatrix4x3* palette, const ArmNodes& arm,
                         const Vec3& wrist_position, const Mat3& wrist_basis);

// Analytic two-bone IK: rotate the shoulder then the elbow so the wrist lands on
// `requested_wrist_position`, then set the wrist basis exactly.
//
// `fallback_pole` is the elbow-direction hint used only when the current elbow is degenerate
// (collinear with the shoulder-to-wrist line); the root's up axis is the usual choice.
//
// Returns false WITHOUT having half-applied anything only for input that fails up front. Once it
// starts rotating it runs to completion -- callers should treat a false return as "the arm is in
// an unknown state, do not also drive it another way this frame".
bool solve_two_bone_arm(BlamMatrix4x3* palette, const ArmNodes& arm,
                        const Vec3& requested_wrist_position, const Mat3& desired_wrist_basis,
                        const Vec3& fallback_pole, const ArmTuning& tuning);

// The full arm: try the IK, and if it cannot solve, fall back to placing the hand alone so the
// player still has a tracked hand rather than nothing.
// ---- HANDS-ONLY -------------------------------------------------------------------------------
//
// Collapse every node OUTSIDE the hands to invisible, leaving the solve itself completely intact.
//
// WHY A POST-SOLVE FILTER AND NOT A SECOND CODE PATH. The full IK still runs, so aim, the two-hand
// hold and the weapon carry cannot diverge between hands-only and full-arm modes -- there is only
// ever one pose, and this hides part of it. Taken from pancreations MCC VR, whose `floating_hands`
// does exactly this and ships ON by default. The alternative in this file, place_wrist_subtree()
// ("no IK"), is the wrong shape for the same reason: skipping the solve makes hands-only a second
// behaviour to keep in sync forever.
//
// THE WEAPON MUST BE IN THE KEEP SET or the gun vanishes with the arm.
//
// Node 0 is NEVER touched: it is the camera-control root, and scaling it would scale the view.
//
// Fails VISIBLE. Any bad input returns false having changed nothing, so the failure mode is "the
// arms are still there", never an invisible weapon or a half-collapsed rig.
bool collapse_to_hands(BlamMatrix4x3* palette, std::size_t node_count,
                       const ArmNodes& left, const ArmNodes& right,
                       const std::uint8_t* keep_extra, std::size_t keep_extra_count);

bool solve_arm_for_tracked_wrist(BlamMatrix4x3* palette, const ArmNodes& arm,
                                 const Vec3& wrist_position, const Mat3& wrist_basis,
                                 const Vec3& fallback_pole, const ArmTuning& tuning);

// Where the wrist bone goes given a controller grip pose. Applies the grip-to-wrist back/down
// offset in the grip's own frame.
Vec3 wrist_from_grip(const Vec3& grip_position, const Mat3& grip_basis, const ArmTuning& tuning);

// ---- FINGERS -----------------------------------------------------------------------------------

// How CLOSED one hand is. 1 = the authored closed grip the game ships, 0 = fully open.
//
// The sense is "curl", not "openness", because that is the direction the inputs arrive in: a
// pulled trigger and a squeezed grip are both 1. Getting this backwards produces hands that open
// when you squeeze, which looks like a tracking fault rather than a sign error -- so the field
// names are the button names on purpose.
struct HandCurl {
    float index = 1.0f;   // trigger finger
    float grip  = 1.0f;   // middle, ring, pinky
    float thumb = 1.0f;
};

// Open a hand from the authored closed grip toward an open pose derived from the rig's own
// proportions -- extend each digit away from the wrist through its first knuckle, then rotate each
// joint and its descendants partway onto that line.
//
// Derived rather than authored ON PURPOSE: it works for either hand and every shipped weapon with
// no hard-coded Euler axes, which is what makes it survive a weapon we have never tested.
bool apply_hand_openness(BlamMatrix4x3* palette, const ArmNodes& arm, const HandCurl& curl);

} // namespace halo::palettearm
