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

    // WHICH WAY the body-anchored pole points. OFF: torso up (the port's original, which bends the
    // elbow UPWARD -- a raised chicken-wing). ON: pancreations MCC VR's direction, OUT and DOWN in
    // the torso frame -- left*outSign - pole_down*up, outSign +1 for the left arm and -1 for the
    // right -- which is where a held-rifle elbow actually hangs (game.cpp:4618-4628).
    // DEFAULT ON (2026-09-16): with the pole pointing up, the raw probe dump put the rendered elbow
    // 18 cm ABOVE the shoulder-to-wrist midpoint on a pistol held at chest height -- the raised
    // chicken-wing. Out-and-down is where a held-weapon elbow hangs, and it is what MCC VR ships.
    bool  pole_out_down = true;
    float pole_down     = 0.6f;

    // OVER-REACH: stretch instead of clamping. 1.0 = off (clamp at the reach sphere, hand snapped
    // the rest of the way so the forearm end stops short of the hand). pancreations use 1.8:
    // both bone lengths scale by k = min(dist/reach, stretch_max) and the elbow subtree is moved
    // onto the stretched elbow, so skinning stretches the mesh with the bones instead of the hand
    // visibly detaching from the forearm. Applied AFTER the clavicle assist.
    // BOTH bones really lengthen (2026-09-17): every helper node between two joints slides out by
    // its station along the bone, and the hand is carried to the stretched forearm's end.
    float stretch_max = 1.0f;
    // How much of the extension the target asks for the BONES take; the rest still opens at the
    // wrist when the hand is snapped onto the controller. 1 = all of it, up to stretch_max.
    float stretch_share = 1.0f;
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

// ---- HAND SHAPES FROM THE GAME'S OWN ANIMATIONS ------------------------------------------------
//
// Three key poses recorded off the stock first-person palette: the STRETCHED open hand at the
// release of the grenade throw, the RELAXED hand of the weapon-draw animation, and the FIST of the
// Magnum's off-hand punch. Stored as parent-relative joint
// rotations (joint 0 relative to the wrist). This rig mirrors its hands by BEHAVIOUR -- every bone
// offset of the left hand is the exact negative of the right's, measured -- so one set of local
// rotations curls either hand, and the bone OFFSETS are read off the palette in hand rather than
// stored, which keeps every finger bone its authored length.
//
//   curl      -1 = the stretched open hand .. 0 = the relaxed hand .. 1 = the fist
//   authored  1 = leave the fingers exactly as the game posed them .. 0 = replace them entirely
//
// Rotations only: the knuckles stay where the palm puts them. Call AFTER the wrist is placed.
bool apply_hand_shape(BlamMatrix4x3* palette, const ArmNodes& arm, float curl, float authored);

// ---- EMPTY-HAND GESTURES (2026-09-18, by request). The same three key poses, blended PER FINGER
// instead of per hand -- no new authored poses are needed, because every finger is its own chain
// and a point is just "index from the open hand, the rest from the fist".
//
//   curl[]      index, middle, ring, pinky, thumb -- each on apply_hand_shape's axis (-1 open ..
//               0 relaxed .. 1 fist)
//   thumb_over  how far PAST the fist the thumb goes (0..1 of `over_gain`): the punch the fist was
//               taken from holds its thumb beside the fingers, and a clenched fist wraps it over
//               them. Extrapolated along the thumb's own open->fist arc; only used at thumb curl 1.
//   thumb_ext   how far PAST the open hand the thumb's outer joints go (0..1+): the recorded open
//               hand leaves the thumb tip bent, which reads as a limp thumbs-up. Extrapolated along
//               the thumb's own relaxed->open arc, outer joints only, and only while the thumb is
//               opening (curl < 0).
struct HandGesture {
    float curl[5]{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float thumb_over{0.0f};
    float thumb_ext{0.0f};
};
// What a controller's three inputs mean on an empty hand. `rest` is the relaxed hand's curl.
//   grip + trigger + thumb sensor   a fist, thumb clenched over
//   grip + trigger                  thumbs up
//   grip + thumb sensor             pointing, thumb down
//   grip                            pointing, thumb out
//   trigger alone                   the relaxed hand with the index pulled in
//   nothing                         the relaxed hand
//   point_curl  the index's curl when it POINTS (-1 = the recorded open hand, which lifts it too far
//               to read as a point; -0.5 sits between that and the relaxed hand).
//   thumb_ext   the thumb's extension past the open hand when it is OUT (thumbs up, point).
HandGesture gesture_for_inputs(bool grip, bool trigger, bool thumb_touch, float rest,
                               float point_curl = -1.0f, float thumb_ext = 0.0f);
// Ease `current` toward `target`, finger by finger (tau seconds, first-order).
void ease_gesture(HandGesture& current, const HandGesture& target, float dt, float tau);
// `over_gain` = how far past the fist a full thumb_over goes, as a fraction of the open->fist arc.
// `thumb_out` = how far a CURLED thumb's base joint is turned back toward the open hand (0..1), so a
//               thumb laid across a fist sits outside the index instead of through it (headset,
//               2026-09-18: "the thumb down position also needs to be rotated out a little so it
//               doesn't clip with the index finger when clenching a fist"). Base joint only.
bool apply_hand_gesture(BlamMatrix4x3* palette, const ArmNodes& arm, const HandGesture& gesture,
                        float authored, float over_gain = 0.35f, float thumb_out = 0.0f);

// A rotation part of the way from `a` to `b`, along the SHORT ARC.
//
// blend_basis() is a normalised lerp, and says itself that it is only meaningful for nearby
// orientations: with the two forwards opposed it has no forward left to normalise at the half-way
// mark and returns something that is not a rotation. That was harmless while it only ever ramped a
// hand onto a forestock it was already reaching for; a FREE hand handed to a reload animation can
// start from any orientation at all. Never returns a non-rotation: degenerate inputs snap to the
// nearer end.
Mat3 slerp_basis(const Mat3& a, const Mat3& b, float weight);

// ---- FOREARM TWIST --------------------------------------------------------------------------
//
// The solve turns the HAND to the controller and leaves the forearm exactly as the elbow carries
// it, so every degree of controller roll is taken at the wrist joint: "the hand can twist
// unnaturally in its socket". This rig has twist bones for exactly that -- two per forearm, on the
// bone's axis a third and two thirds of the way down -- and the game's own animations drive them
// at 0.31 and 0.72 of the hand's twist (see twist_share in the .cpp for the measurement). This
// spreads the roll the SOLVE ADDED over them by the same rule.
//
// THE BASELINE IS THE AUTHORED POSE: capture_forearm_stock() before the arm is touched, and a hand
// that ends up in its authored relation to the forearm adds nothing, whatever that relation is.
// `gain` scales the game's own distribution: 0 = the forearm as the elbow carries it (the behaviour
// to date), 1 = what the rig's animations would do for this much roll, 2 = the cap.
// `plate_gain` is the ARMOUR's share: the off-axis nodes hanging off the elbow (gauntlet plates),
// which the game never rolls. 1 = a plate turns with the forearm at its own station along the bone,
// 0 = rigid with the elbow as the game has it.
// `bone_gain` is the ELBOW NODE's own share -- whatever is skinned to the forearm bone proper. 1 = it
// rolls as much as the near twist bone, 0 = not at all (the game's way, and ours until 2026-09-17).
//
// `thumb_up_hint` is any direction that reads as "up for a thumb" in the palette's frame (torso up,
// leaning back); it only decides WHERE the roll's 180-degree seam sits, never how much is applied.
struct ForearmStock {
    Mat3 elbow_basis{};
    Mat3 wrist_basis{};
    Vec3 axis_local{};       // elbow -> wrist, unit, in the elbow's own frame
    bool valid{false};
};
struct ForearmTwistResult {
    float hand_deg{};        // the roll the solve added at the wrist, about the forearm
    float neutral_deg{};     // the authored hand's roll short of thumb-up
    float follow_deg{};      // what the forearm is asked to follow (before gain and per-bone share)
    int   nodes{};           // twist bones turned
    int   plates{};          // armour nodes carried round with them
    float bone_deg{};        // how far the elbow node itself was rolled
};
ForearmStock capture_forearm_stock(const BlamMatrix4x3* palette, const ArmNodes& arm);
bool distribute_forearm_twist(BlamMatrix4x3* palette, const ArmNodes& arm, const ForearmStock& stock,
                              const Vec3& thumb_up_hint, float gain, float plate_gain, float bone_gain,
                              ForearmTwistResult* result = nullptr);

// ---- RECOIL PASS-THROUGH ----------------------------------------------------------------------
//
// A carry that lands the weapon's authored marker ON a target cancels every translation the game
// animates into that marker -- including the straight-back kick that is ALL the recoil some weapons
// have (the Assault Rifle: 2-4 cm back per shot, under a degree of rotation). This watches the
// STOCK marker, learns where it rests, and hands back the part of its displacement that is a kick,
// in the stock frame, for the carry to leave in.
//
//   gain   0 = cancel everything (the behaviour to date) .. 1 = the authored kick
//   max_m  the most that is ever let through; displacement beyond it fades out by twice this
//   learn  false on a frame that must not teach the rest pose (a mirror bank, a calibration hold)
struct RecoilPass {
    bool  have_ref{false};
    Vec3  ref_pos{};
    Mat3  ref_basis{};
    bool  have_prev{false};
    Vec3  prev_pos{};
    Mat3  prev_basis{};
    int   stable{0};
    int   latches{0};            // times a rest pose was first learned (diagnostics)
    bool  remembered{false};     // the rest pose was adopted, not learned: see adopt()
    float last_back_m{0.0f};     // what the last update let through, metres (diagnostics)
    float last_moved_m{0.0f};    // how far the marker is from its rest pose, metres...
    float last_turned_deg{0.0f}; // ...and how far it has turned (both 0 until a rest pose is known)

    void reset();
    // Start from a rest pose this model taught EARLIER (the caller keeps one per weapon model):
    // known at once, so everything measured from rest works from the first frame, but held loosely
    // -- the first learn re-anchors it in a third of a second whatever the distance, since an
    // idle sway can leave two learns of the same weapon a few centimetres apart.
    void adopt(const Vec3& pos, const Mat3& basis);
    Vec3 update(const Vec3& marker_pos, const Mat3& marker_basis, float gain, float max_m, bool learn);
    // Known, holding still, and where it was learned: the only state rest RELATIONS are learned in.
    bool at_rest() const;
};

// ---- REST RELATIONS, THE ACTION WATCH, THE MELEE GATE ----------------------------------------
//
// Where one stock node sits in the STOCK weapon marker's frame while the gun is at rest: learned
// the same way RecoilPass learns the marker's own rest pose (still for a third of a second; a
// relation far from the known one has to hold for 1.5 s), and only while RecoilPass::at_rest().
// `dev_*` say how far the node is from that relation as of the last learn().
struct RestRelation {
    bool  have{false};
    Vec3  pos{};
    Mat3  basis{};
    bool  have_prev{false};
    Vec3  prev_pos{};
    Mat3  prev_basis{};
    int   stable{0};
    bool  remembered{false};     // adopted from an earlier session with this model: see adopt()
    float dev_m{0.0f};
    float dev_deg{0.0f};

    void reset();
    // Start from a relation learned earlier for this model (as RecoilPass::adopt): known at once,
    // re-anchored by the first learn in a third of a second whatever the distance.
    void adopt(const Vec3& p, const Mat3& b);
    void learn(bool gun_at_rest, const Vec3& p, const Mat3& b);
};

// While the support hand grips the gun it rides the rigid transform that carries the gun, so it
// performs whatever the game animates -- the magazine swap, the pump. A FREE support hand follows
// its controller, and then a reload swaps a magazine with nobody holding it. This says WHEN the
// game is playing such an action, from the stock palette alone (there is no reload event to
// subscribe to): the authored off hand moving RELATIVE TO THE GUN from its rest relation. The
// result is an eased 0..1 weight for the caller to hand the wrist to the animation by, exactly as
// it does for a two-hand hold. The gun leaving its own rest pose is NOT an action here -- that is
// the EquipGate's business.
//
// `*_age_s` = seconds since that mask was last seen going to the game (negative = never): they say
// which action a hand-over is, for the RETURN CUT (see the .cpp) -- a melee lets go on its first
// sustained approach after the peak, a reload (and anything not asked for) only once the gun is
// nearly home too, a throw `grenade_trim_s` before its authored end. The weight itself survives a
// reset, so nothing pops.
constexpr float kThrowSeconds = 1.35f;      // the authored throw (1.37 / 1.40 measured on Magnum / rifle)

struct ActionWatch {
    enum class Kind : std::uint8_t { Other, Melee, Reload, Grenade };
    RestRelation hand;           // the support wrist in the marker's frame
    float weight{0.0f};
    float last_target{0.0f};     // diagnostics, as of the last update
    bool  engaged{false};        // a hand-over is in progress (weight above zero)
    Kind  kind{Kind::Other};     // ...and what asked for it
    float since_onset_s{0.0f};
    float peak_m{0.0f};          // the furthest the off hand has been from its hold this time
    int   descending{0};         // consecutive frames it has been coming home
    bool  home_cut{false};       // let go for the return; stays so until the authored hand is home

    void  reset();
    // `hand_*` = the STOCK support wrist expressed in the STOCK marker's frame. `gate` scales every
    // threshold (1 = as measured). Live frames only.
    float update(const RecoilPass& gun, const Vec3& hand_pos, const Mat3& hand_basis, float gate, float dt,
                 float melee_age_s = -1.0f, float reload_age_s = -1.0f, float grenade_age_s = -1.0f,
                 float grenade_trim_s = 0.0f);
};

// IS AN EQUIP ANIMATION PLAYING? The put-away (the gun leaving rest within a second of a swap being
// asked for) and the draw (from the weapon model changing until the new weapon rests, 3 s at most).
// `other_age_s` = seconds since ANY OTHER action was asked for (the reload, melee or throw mask;
// negative = never): a press that arrives after the gate opened ends it, because the game does not
// take one during a swap -- a reload straight off the draw is a reload, and the draw is over.
struct EquipGate {
    bool  active{false};
    bool  draw{false};
    float since_s{0.0f};
    float weight{0.0f};          // eased 0..1

    void  reset();
    float update(float swap_age_s, bool weapon_changed, const RecoilPass& gun, float dt,
                 float other_age_s = -1.0f);
};

// A HAND'S REST SHAPE: every wrist-subtree node's relation to the wrist. Captured off the stock
// palette while the gun is at rest, and blended back in when an animation is being held off a
// hand that is otherwise placed rigidly from the live pose -- otherwise "the fingers still animate".
struct RestNode { Vec3 pos{}; Mat3 basis{}; };
bool capture_hand_rest(const BlamMatrix4x3* palette, const ArmNodes& arm, RestNode* out, std::size_t out_count);
bool blend_hand_to_rest(BlamMatrix4x3* palette, const ArmNodes& arm, const RestNode* rest, std::size_t rest_count,
                        float weight);

// IS A MELEE PLAYING? Which animation an action is cannot be read off the pose -- a butt stroke and
// a reload overlap in every magnitude -- but a melee is always ASKED for: the swing gesture presses
// the melee button, and so does a thumb. So this is keyed off the press (`press_age_s`: seconds
// since the melee mask was last seen going to the game, negative = never), which also leads the
// animation by a few frames, and stays up until the pose is back at rest (4 s at most) -- or until
// another action is asked for (`other_age_s`: the reload or throw mask, negative = never), since a
// reload pressed after the swing is a reload, and the melee's business is over.
struct MeleeGate {
    bool  active{false};
    float since_s{0.0f};
    float weight{0.0f};          // eased 0..1

    void  reset();
    float update(float press_age_s, const RecoilPass& gun, float hand_dev_m, float hand_dev_deg, float dt,
                 float other_age_s = -1.0f);
};

// IS A SPRINT PLAYING? The sprint animation holds the gun well away from rest for the whole sprint,
// which is also what a put-away or a melee looks like -- so it counts only with the stick pushed
// (`move_age_s`: seconds since the movement stick was last past half travel) and a sprint asked for
// within the last three seconds (`button_age_s`: the sprint mask, a hold or a toggle). Ends when the
// stick or the pose lets go, or the rest pose is lost. The caller must stop teaching rest poses
// while this is active, or the sprint pose becomes "rest" after a second and a half.
// `other_age_s` = seconds since a reload, melee or throw was asked for (negative = never): within
// two seconds of one it is not a sprint and a running one ends -- a RELOAD also takes the gun
// "well away from rest" while the stick is pushed, and with a sprint asked for moments earlier
// that read as a sprint and held the free hand off the reload (headset, 2026-09-17).
struct SprintWatch {
    bool  active{false};
    float quiet_s{0.0f};
    float weight{0.0f};          // eased 0..1

    void  reset();
    float update(float button_age_s, float move_age_s, const RecoilPass& gun, float dt,
                 float other_age_s = -1.0f);
};

} // namespace halo::palettearm
