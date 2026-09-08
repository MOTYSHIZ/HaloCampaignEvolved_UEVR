#include "PaletteArm.hpp"

#include "ArmSolve.hpp"
#include "NodeDiscovery.hpp"
#include "NodeMap.hpp"
#include "PaletteHook.hpp"
#include "PaletteMath.hpp"
#include "TwoHand.hpp"

#include "../addrcascade/AddressCascade.hpp"
#include "../ArmDriver.hpp"
#include "../Config.hpp"
#include "../DevTools.hpp"        // HALO_VR_DEV_ONLY: the dump must not exist in a player build
#include "../MotionAimControl.hpp"   // get_pose()
#include "../WeaponCalib.hpp"        // wpnfix: the per-weapon rigid delta and its capture latch
#include "../WeaponOffset.hpp"       // weapon_offset_current_class(): the watchdog's "possible"
#include "../TwoHandAim.hpp"
#include "../WeaponDrive.hpp"      // meshdrv: is the legacy container drive still on?         // the ONE two-hand owner -- this folder no longer keeps one

#include "uevr/API.hpp"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <string>

using uevr::API;

namespace halo {
namespace {

namespace pa = ::halo::palettearm;

// ---- FRAME CONVERSION --------------------------------------------------------------------------
//
// ⚠️ THE LEAST VERIFIED THING IN THIS FOLDER. OpenXR is Y-up / -Z-forward, metres. Blam is +Z-up,
// +X-forward, +Y-LEFT, and its unit is 3.048 m. The mapping below is elliotttate's, carried over
// verbatim because a hand-derived frame conversion is exactly the mistake this project has already
// paid for once (see the memory note on consulting reference implementations before deriving VR
// math). If the arms come out mirrored or rotated 90 degrees in a headset, START HERE -- do not
// start by adjusting the shoulder offsets, which is what a wrong frame looks like from the outside.

pa::Vec3 xr_to_blam(const pa::Vec3& v) { return {-v.z, -v.x, v.y}; }

// forward/left/up in OPENXR axes. XR forward is -Z and Blam "left" is XR -X, so this is where
// the axis roles are assigned; xr_to_blam below only relabels components.
pa::Mat3 xr_rotation_to_xr_basis(const pa::Quat& q) {
    return {pa::rotate(q, {0.0f, 0.0f, -1.0f}),
            pa::rotate(q, {-1.0f, 0.0f, 0.0f}),
            pa::rotate(q, {0.0f, 1.0f, 0.0f})};
}

// Relabel a whole basis from OpenXR axes into Blam axes.
//
// This is a proper rotation (the signed permutation (x,y,z) -> (-z,-x,y) has determinant +1),
// so it preserves cross products -- which is what makes it legitimate to run the two-hand blend
// in XR space and relabel the RESULT, rather than relabelling the inputs first. Blending then
// relabelling and relabelling then blending give the same answer.
pa::Mat3 blam_basis_from_xr_basis(const pa::Mat3& m) {
    return {xr_to_blam(m.forward), xr_to_blam(m.left), xr_to_blam(m.up)};
}

pa::Mat3 xr_rotation_to_blam_basis(const pa::Quat& q) {
    return blam_basis_from_xr_basis(xr_rotation_to_xr_basis(q));
}

// Our Math.hpp Vec3/Quat -> this folder's. Two frames, two types, one conversion point; see the
// note at the top of PaletteMath.hpp for why they are not the same type.
pa::Vec3 from_math(const ::halo::Vec3& v) { return {v.x, v.y, v.z}; }
pa::Quat from_math(const ::halo::Quat& q) { return {q.x, q.y, q.z, q.w}; }

// ---- STATE -------------------------------------------------------------------------------------

// NO TwoHandHold HERE. The hold moved to src\TwoHandAim.cpp when it was lifted out of this
// folder -- it is an aim feature and had no business working only under armdriver=2. This
// module now CONSUMES the swing that owner publishes, the same one the aim path and the
// rendered weapon use. If you are about to add a second TwoHandHold here, you want that one.
pa::ArmTuning    s_arm_tuning;

// WHICH INDEX IS WHICH BONE, resolved rather than remembered. Rung 1 derives the map from the
// palette the hook just handed us; rung 2 is elliotttate's measured table; rung 3 is arms stay
// stock. See NodeDiscovery.hpp and docs\PALETTEARM-NODEMAP.md.
//
// The resolver lives here, on the impure side, because it holds per-session state. It is touched
// only from inside the detour, on the game thread.
// Arm's reach for the weapon destination, in PALETTE units (1 unit = 3.048 m). 0.6 ~= 1.8 m, which
// no held weapon node legitimately exceeds; a looser bound only catches NaN-scale garbage and lets
// a tracking dropout through.
constexpr float kWeaponReachMax = 0.6f;

// ---- THE PER-WEAPON RIGID DELTA (wpnfix), AND ITS CAPTURE ---------------------------------------
//
// WHAT IS MISSING WITHOUT IT. The carry below puts the weapon's AUTHORED marker node onto the aim
// controller, so the gun sits where the artist said it meets the hand -- which is right for the
// weapon they authored that marker on, and only approximately right for the next one. The only
// adjustment that existed was pa_wpn_yaw/pitch/roll, ONE trim for every weapon. A pistol and a
// rocket launcher do not share a grip, so that global trim can be correct for at most one of them.
//
// THE FRAME, and it is the question worth being precise about. The delta is rigid in the AIM
// CONTROLLER's own frame, applied immediately after the global trim:
//
//     desired = root o gap o [ LOCK o wctrl o TRIM o Wrot ]        (orientation)
//     hand    = root o gap o [ (grip - hmd) + (wctrl o TRIM) * Wpos ]      (position)
//
// so `Wrot` right-multiplies for the same reason the global trim does -- a fixed wrist angle at any
// orientation -- and `Wpos` is carried by the trimmed basis, which is what makes the pair a rigid
// attachment rather than two independent knobs. Everything inside the square brackets is STAGE
// frame (the recenter-composed room frame); the lift and the root map that into palette space and
// are not part of the delta.
//
// LOCK is the barrel lock (Config.hpp pa_barrel_lock, DEFAULT OFF), and its position in that line
// is the whole of how the two features relate: it is a LEFT-multiply applied AFTER both trims, so
// it can only remove what they left over. `Wrot` therefore keeps the meaning it has always had --
// what puts the barrel on the aim ray -- and no stored halo_vr_weapons.cfg value changes when the
// lock is switched on. It is deliberately UPSTREAM of the s_wcarry publish below so the capture
// solves against the pose actually on screen; see the block at the apply site for what happens
// otherwise (blindcowboy24 measured it: 7.6 -> 23.8 degrees in a single hold).
//
// blindcowboy24's PR #1 stores the same per-weapon idea in the same shape, but in UE axes and with
// his lock applied BEFORE the trim, which makes his `wr` a purely visual trim off the ray where
// ours is not. The two files' numbers are still not interchangeable; see Config.hpp's WeaponFix
// and the wpnfixver stamp.
struct WeaponCarry {
    pa::Mat3 basis{};    // wctrl * global trim * per-weapon rotation, STAGE frame
    pa::Vec3 offset{};   // (aim grip - hmd) + the per-weapon translation, Blam units, STAGE frame
};

// Published by the DRIVE, read by the game tick's capture edges. ALWAYS THE LIVE VALUE, published
// before the freeze substitution below -- a latch that fed on the frozen output would compose the
// capture onto itself and every hold would solve to zero.
//
// Plain values behind an atomic flag, the same shape as TrackingSnapshot above and for the same
// reason: a torn read is one frame of wobble, and every consumer re-checks valid_basis anyway.
WeaponCarry       s_wcarry{};
std::atomic<bool> s_wcarry_valid{false};

// Published by the GAME TICK, read by the drive.
pa::Mat3          s_wfix_rot{};                 // identity while s_wfix_have is false
pa::Vec3          s_wfix_pos{};                 // BLAM UNITS here; the file stores metres
std::atomic<bool> s_wfix_have{false};
WeaponCarry       s_wfreeze{};                  // the stage-frame pose latched at key-down
std::atomic<bool> s_wfreeze_active{false};

// ---- THE SUPPORT HAND'S OWN RIGID FIX, and its capture. -----------------------------------------
//
// SAME MECHANISM AS THE WEAPON FIX ABOVE, deliberately down to the variable names: freeze a
// stage-frame pose, let the player bring their real hand onto it, and difference the two in the
// live pose's own frame. Everything the weapon-fix note argues for -- why the freeze is in the
// STAGE frame and not camera space, why the live carry must be published before the substitution,
// why the solve is exact -- applies here word for word, because it is the same two lines of algebra
// over the same kind of pair. Read that note; this one only records what is DIFFERENT.
//
// WHAT IS DIFFERENT: there is no marker node and no authored attachment in the chain. The support
// hand is free-tracked -- its wrist target is built straight from its controller -- so the pair
// being frozen IS the controller pose, and the trim it solves for is the fixed offset between the
// player's real hand and the grip pose their runtime reports for it. That is a property of a person
// and their controller, which is exactly why nothing is shipped for it (Config.hpp, hand_fix_q).
//
// SUPPORT HAND ONLY. The aim hand is not a candidate: the arm IKs to that controller precisely
// BECAUSE the weapon is calibrated to it (see the note above the HandPlan table), so a second rigid
// trim there would move the hand off the gun it was fitted to and fight the pose calibration.
pa::Mat3          s_hfix_rot{};                 // identity while s_hfix_have is false
pa::Vec3          s_hfix_pos{};                 // BLAM UNITS here; the file stores metres
std::atomic<bool> s_hfix_have{false};
WeaponCarry       s_hcarry{};                   // the LIVE support-hand stage pose, published
std::atomic<bool> s_hcarry_valid{false};
WeaponCarry       s_hfreeze{};                  // latched when the gesture arms
std::atomic<bool> s_hfreeze_active{false};

std::atomic<bool>  s_wpn_driven{false};
std::atomic<float> s_dbg_wpn_reach{0.0f};
// HOW FAR OFF THE AIM RAY THE RENDERED BARREL WAS BEFORE THE LOCK TOUCHED IT, degrees, and the
// strength the fade actually applied. This is the ONLY number that can falsify the barrel lock's
// one assumption (Config.hpp pa_barrel_lock: that the game's stock pose points the gun down the
// camera forward). If the assumption holds, `off` sits near zero at rest with the trims calibrated,
// grows on a flick, and returns; a large CONSTANT `off` means the assumption is wrong on this build
// and the lock is aligning the wrong axis. Published even when the lock is OFF, so the residual can
// be read before anyone enables it. Dev-only: it answers a question rather than playing the game.
std::atomic<float> s_dbg_barrel_off{0.0f};
std::atomic<float> s_dbg_barrel_str{0.0f};
std::atomic<float> s_dbg_sh_x{0.0f}, s_dbg_sh_y{0.0f}, s_dbg_sh_z{0.0f};
std::atomic<float> s_dbg_yaw_hands{0.0f}, s_dbg_yaw_head{0.0f};
std::atomic<float> s_dbg_wpn_x{0.0f}, s_dbg_wpn_y{0.0f}, s_dbg_wpn_z{0.0f};
// SUPPORT arm, so a right-hand-to-left-arm coupling is visible instead of inferred.
std::atomic<float> s_dbg_sup_x{0.0f}, s_dbg_sup_y{0.0f}, s_dbg_sup_z{0.0f};
std::atomic<float> s_dbg_supsh_x{0.0f}, s_dbg_supsh_y{0.0f}, s_dbg_supsh_z{0.0f};
// SUPPORT WRIST IN THE TORSO FRAME, relative to its own shoulder. The aim-independence test.
// sup= is camera-local, so it MUST swing when the camera yaws even for a perfectly body-fixed
// hand -- de-rotating it by the camera yaw is circular, because that yaw is the suspect quantity.
// torso_basis is built from the head and the hand POSITIONS with the lock gap removed, so it does
// not turn with the aim. If suptf is constant while the aim hand pitches, the support arm has no
// aim dependency; if it moves, there is a leak and the number says how big.
std::atomic<float> s_dbg_suptf_x{0.0f}, s_dbg_suptf_y{0.0f}, s_dbg_suptf_z{0.0f};
// The GAME own authored support hand, torso frame, captured BEFORE we touch the arm.
// If the game pitches its first-person rig by the aim pitch, this rotates while the players view
// does not -- the palette is camera-local and VR decouples camera pitch. That is exactly "the left
// arm moves when I pitch but not when I yaw or roll": yaw turns rig and camera together, and there
// is no aim roll. arm_gap corrects YAW ONLY, so nothing downstream removes a pitch.
std::atomic<float> s_dbg_stockL_x{0.0f}, s_dbg_stockL_y{0.0f}, s_dbg_stockL_z{0.0f};
// THE FINAL LEFT HAND IN THE WEAPON OWN FRAME, plus the two-hand blend that explains it.
// Constant => the hand is glued to the gun (what two-handing should look like). Varying with the
// aim hand => the gun pitches away from a hand that stayed behind, which is what the player sees.
std::atomic<float> s_dbg_handgun_x{0.0f}, s_dbg_handgun_y{0.0f}, s_dbg_handgun_z{0.0f};
std::atomic<float> s_dbg_blend{0.0f};
// WHAT ACTUALLY GETS DRAWN. Every earlier number was the TARGET, captured before IK; suptf is
// provably flat under a 107-deg pitch sweep, so if the arm still moves the gap is between the ask
// and the result. gotL/elbL are the FINAL wrist and elbow, torso frame, relative to the shoulder --
// directly comparable to suptf. missL is the support arm reach shortfall, which was only ever
// recorded for the aim arm.
std::atomic<float> s_dbg_gotL_x{0.0f}, s_dbg_gotL_y{0.0f}, s_dbg_gotL_z{0.0f};
std::atomic<float> s_dbg_elbL_x{0.0f}, s_dbg_elbL_y{0.0f}, s_dbg_elbL_z{0.0f};
std::atomic<float> s_dbg_missL{0.0f};

#if HALO_VR_DEV
// ---- PER-FRAME JITTER. Every other diagnostic here samples once per perf window (~1.5 s), which
// cannot see a 40-90 Hz vibration at all -- it samples one arbitrary frame out of a hundred. These
// accumulate frame-to-frame deltas at HOOK rate and report mean/max per window, which is the only
// way to say WHERE the jitter enters: if the elbow moves far more than the wrist, it is the IK pole
// (the pole hint comes from the game ANIMATED stock pose); if the raw controller already moves that
// much, it is tracking noise and not ours; if the gap angle moves, it is our own compensation.
struct JitterAcc {
    pa::Vec3 prev{};
    bool     have = false;
    float    sum = 0.0f, peak = 0.0f;
    unsigned n = 0;
    void feed(const pa::Vec3& v) {
        if (have) { const float d = pa::length(v - prev); sum += d; if (d > peak) peak = d; ++n; }
        prev = v; have = true;
    }
    float mean_cm() const { return n ? (sum / (float)n) * pa::kMetresPerBlamUnit * 100.0f : 0.0f; }
    float peak_cm() const { return peak * pa::kMetresPerBlamUnit * 100.0f; }
    void  reset() { sum = 0.0f; peak = 0.0f; n = 0; }
};
struct ScalarJitter {
    float prev = 0.0f; bool have = false; float sum = 0.0f, peak = 0.0f; unsigned n = 0;
    void feed(float v) {
        if (have) { const float d = std::fabs(v - prev); sum += d; if (d > peak) peak = d; ++n; }
        prev = v; have = true;
    }
    float mean() const { return n ? sum / (float)n : 0.0f; }
    void  reset() { sum = 0.0f; peak = 0.0f; n = 0; }
};
JitterAcc    s_j_tgt, s_j_wrist, s_j_elbow, s_j_ctrl;
// FRAME-INVARIANT ARM SHAPE. Everything above is a PALETTE position, i.e. CAMERA-LOCAL, and the
// camera moves -- it pitches with the aim and shakes with the frame pacing. Those numbers therefore
// conflate camera motion with bone motion in BOTH directions: a still bone can read noisy and a
// jittering bone can read clean. (Same trap that hid the aim-pitch bug earlier today.)
//
// These three describe the arm SHAPE and are unchanged by any rigid transform of the whole arm, so
// the camera cancels exactly:
//   bend   - the elbow joint angle, degrees
//   perp   - how far the elbow sits off the shoulder-to-wrist line, cm
//   swivel - the elbow azimuth about that line, measured against the aim-free torso up axis, degrees
// If these are steady the arm is internally rigid and any wobble is whole-arm. If they jitter, the
// IK solution itself is unstable and that IS the reported jitter.
ScalarJitter s_j_bend, s_j_perp, s_j_swivel;
// THE HAND ITSELF, in the aim-free torso frame. The shape measures above describe the ARM; with a
// hands-only mode on the table the hand is the entire feedback channel, and its ORIENTATION was
// never measured at all. desired_wrist is built from the controller rotation lifted by arm_gap, so
// an imperfect lift shows up here as the hand rotating with the aim -- invisible to a position-only
// diagnostic, and invisible again to anything expressed in camera space.
struct BasisJitter {
    pa::Mat3 prev{};
    bool     have = false;
    float    sum = 0.0f, peak = 0.0f;
    unsigned n = 0;
    void feed(const pa::Mat3& m) {
        if (have) {
            // Angle of the rotation carrying the previous orientation onto this one.
            const pa::Mat3 rel = pa::multiply(m, pa::transpose(prev));
            const float tr = rel.forward.x + rel.left.y + rel.up.z;
            float c = (tr - 1.0f) * 0.5f;
            c = (c < -1.0f) ? -1.0f : ((c > 1.0f) ? 1.0f : c);
            const float d = std::acos(c) * 57.29577951f;
            sum += d; if (d > peak) peak = d; ++n;
        }
        prev = m; have = true;
    }
    float mean() const { return n ? sum / (float)n : 0.0f; }
    void  reset() { sum = 0.0f; peak = 0.0f; n = 0; }
};
// BONE LENGTHS and SKELETON IDENTITY. bend swings 75 deg in one frame with the controller static,
// and bend follows from the law of cosines over the bone lengths and the target distance -- all read
// from the CURRENT palette every call. If the hook fires for more than one skeleton and we drive
// whichever arrives with a single arm map, consecutive frames describe DIFFERENT rigs and the solve
// legitimately lands somewhere else each time. Bone lengths are pure scalars: invariant to every
// frame question that has misled this investigation, so if they move, the input changed.
ScalarJitter s_j_upper, s_j_lower;
std::int32_t s_tag_last = 0;
bool         s_tag_have = false;
unsigned     s_tag_switches = 0;
std::int32_t s_tag_a = 0, s_tag_b = 0;
// FROZEN-POSE DETECTOR. A hand resting on a knee and a snapshot that has stopped updating BOTH read
// as ~0 cm of motion -- magnitude cannot tell them apart, and mistaking one for the other is how the
// arms-stop-tracking report was nearly mis-diagnosed. Real tracking always jitters in the low bits,
// so BIT-IDENTICAL consecutive poses are the discriminator: a resting hand gives runs of 1-2, a
// frozen source gives runs of hundreds.
float    s_frz_x = 0.0f, s_frz_y = 0.0f, s_frz_z = 0.0f;
bool     s_frz_have = false;
unsigned s_frz_run = 0, s_frz_worst = 0;
JitterAcc    s_j_handpos;
BasisJitter  s_j_handrot;
ScalarJitter s_j_gap, s_j_view;
char s_status_jitter[512] = "palettearm jitter: (none)";
// BAIL-OUT CENSUS for the SUPPORT arm. Every one of these leaves the arm at the GAME stock pose
// for that frame, so an intermittent bail flickers the arm between our pose and the animation --
// and because our solve seeds its pole from the CURRENT palette, the frame after a bail starts from
// a different pose and can settle somewhere else entirely. The jitter accumulators only feed on
// SUCCESS, so this is invisible to them: it shows up as a large delta between two good frames.
// 0 disabled  1 no support tracking  2 bad controller basis  3 convention settling
// 4 bad desired wrist  5 solve failed  6 shoulder anchor failed
unsigned s_bail[7] = {0, 0, 0, 0, 0, 0, 0};
unsigned s_posed = 0;
// Proof the hands-only collapse RAN, not merely that it is configured. A key that parses and a
// filter that fires are different facts, and this project has shipped the gap before.
unsigned s_hands_applied = 0;
#endif
std::atomic<bool>  s_dbg_wpn_ok{false};

pa::NodeMapResolver      s_node_map;
addrcascade::TierReporter s_map_reporter;

// ---- THE RESOLVED MAP IS CACHED PER SKELETON --------------------------------------------------
// A node map is a property of the SKELETON, not of the pose. Re-resolving it every frame meant
// re-validating it against a palette WE HAD ALREADY POSED -- the validator saw the two wrists flung
// an implausible distance apart, rejected its own map, and the arms fell back to stock; next frame
// the palette was stock again, so it re-acquired, posed, and rejected again.
//
// Measured live 2026-08-25: "node map" flipping between `hardcoded` and `none` while drive stage
// stayed DRIVING. In a headset that is arms that will not hold still.
//
// model_tag is the identity: a different skeleton is the only thing that can invalidate a map.
// WristConvention already takes this precaution one level down, sampling the stock wrist "BEFORE
// this arm is touched" -- this is the same rule applied to the map itself.
const pa::NodeMap* s_map_cached    = nullptr;
std::int32_t       s_map_key_tag   = -1;
std::uint32_t      s_map_key_nodes = 0;

// Written on the game thread each tick, read inside the detour (a different call, same thread in
// practice, but not guaranteed). Plain values, published as one struct under a seqlock-free
// double-buffer would be overkill here -- atomics on the few scalars that matter is enough,
// because a torn pose for one frame is a wobble, not a crash.
struct TrackingSnapshot {
    pa::Vec3 hmd_position{};
    pa::Quat hmd_rotation{};
    pa::Vec3 aim_grip_position{};
    pa::Quat aim_grip_rotation{};
    pa::Quat stage_rotation{};        // UEVR recenter offset: stage space -> the palette root
    pa::Vec3 aim_rotation_source{};   // unused placeholder to keep the struct one cache line
    pa::Quat aim_aim_rotation{};
    pa::Vec3 support_grip_position{};
    pa::Quat support_grip_rotation{};
    bool     valid = false;
    bool     support_valid = false;
};

TrackingSnapshot s_tracking{};
std::atomic_bool s_tracking_ready{false};

char s_status[256] = "palettearm: off";
// 512, not 256: this line has grown field by field and snprintf TRUNCATES SILENTLY -- a sweep
// once read "sh=" with the numbers cut off and reported nothing wrong.
char s_status_geom[1024] = "palettearm geom: (none)";

// ---- THE LATCHED WRIST CONVENTION (per hand) ---------------------------------------------------
//
// A controller orientation is not a wrist orientation. Each wrist BONE has its own authored rest
// convention, and the two hands are mirrored -- so mapping controller axes onto both wrists with
// one formula puts the left hand into the right hand's frame. That is not theoretical: it is what
// "the left arm responds to my left controller but sits in the right controller's space" looks
// like from inside a headset.
//
// The convention is recoverable from the rig itself: the STOCK wrist basis expressed relative to
// the palette root IS the mapping. Latch it, then build every target as
// root * controller * stock_relative.
//
// LATCH ONLY WHEN THE STOCK POSE HOLDS STILL. The frames right after a weapon change are the
// raise/draw animation; latching mid-swing bakes that transient into the mapping and leaves the
// hand rotated oddly for the whole weapon. elliotttate records exactly this, and the 8-frame
// settle below is his.
struct WristConvention {
    pa::Mat3 latched{};
    pa::Mat3 candidate{};
    int      stable_frames = 0;
    bool     have = false;

    // Returns true once the convention is usable. Until then the caller leaves that arm stock.
    bool observe(const pa::Mat3& stock_relative) {
        if (have) return true;
        if (!pa::valid_basis(stock_relative)) { stable_frames = 0; return false; }
        // Minimum per-axis alignment; 1.0 means identical. 0.9993 is roughly two degrees.
        //
        // Written out rather than std::min: windows.h defines min as a MACRO, so `std::min(`
        // expands to `std::(` and the compiler reports an illegal token after the scope operator,
        // which points nowhere near the actual cause.
        const float af = pa::dot(candidate.forward, stock_relative.forward);
        const float al = pa::dot(candidate.left,    stock_relative.left);
        const float au = pa::dot(candidate.up,      stock_relative.up);
        float align = af;
        if (al < align) align = al;
        if (au < align) align = au;
        if (align > 0.9993f) {
            if (++stable_frames >= 8) { latched = candidate; have = true; }
        } else {
            candidate = stock_relative;
            stable_frames = 0;
        }
        return have;
    }
    void reset() { have = false; stable_frames = 0; candidate = pa::Mat3{}; }
};

WristConvention s_conv_right;
WristConvention s_conv_left;
std::int32_t    s_conv_tag = -1;      // the weapon the conventions were latched against

// WHERE THE DRIVE GOT TO on its most recent call, and how many times it completed.
//
// The node map resolving is NOT the same as the arms moving: six checks sit between them, each
// with its own silent `return false`, and from outside all six look identical. That ambiguity is
// the same shape of blindness that made twohand look unimplemented when it was merely never
// latching -- and it has already cost this feature several in-headset sessions.
//
// Written from the detour (the game's own thread), read on the tick. A pointer to a string
// literal, so it can never be torn; a stale read is one frame old at worst.
std::atomic<const char*> s_drive_stage{"never called"};
std::atomic<uint64_t>    s_drive_ok{0};

// LAST DRIVE'S GEOMETRY, for diagnosing a pose that is wrong rather than absent.
//
// "The hands go funky" has many causes that look alike from inside a headset -- a frame conversion
// with a flipped axis, a wrist target computed in the wrong space, an IK that cannot reach, a
// root that is not where the head is. These numbers separate them: if the TARGET is implausible
// the fault is upstream in the frame maths; if the target is sane but the ACHIEVED wrist is far
// from it, the fault is the solver or the arm's reach.
//
// Written on the game's thread inside the drive, read on the tick. Floats, so a torn read is one
// stale component -- acceptable for a number a human reads every twelve seconds.
std::atomic<float> s_dbg_root_x{0.0f}, s_dbg_root_y{0.0f}, s_dbg_root_z{0.0f};
std::atomic<float> s_dbg_tgt_x{0.0f},  s_dbg_tgt_y{0.0f},  s_dbg_tgt_z{0.0f};
std::atomic<float> s_dbg_got_x{0.0f},  s_dbg_got_y{0.0f},  s_dbg_got_z{0.0f};
std::atomic<float> s_dbg_miss_cm{0.0f};
std::atomic<float> s_dbg_reach_cm{0.0f};
// Where the STOCK aim wrist sat before we moved it, and the root's forward axis. Together these
// say whether our computed target is in the right SPACE at all.
std::atomic<float> s_dbg_stock_x{0.0f}, s_dbg_stock_y{0.0f}, s_dbg_stock_z{0.0f};
std::atomic<float> s_dbg_fwd_x{0.0f},   s_dbg_fwd_y{0.0f},   s_dbg_fwd_z{0.0f};

// The give-up latch. See palettearm_unavailable() in the header for why this is a static and
// not a config field.
bool s_unavailable = false;

// ---- THE DRIVE, run inside the detour ----------------------------------------------------------

// ---- ONE-SHOT PALETTE DUMP (dev builds only) ---------------------------------------------------
//
// Both node-map rungs can fail for two reasons that are indistinguishable from outside -- the
// indices are wrong for this build, or the pointer is not the first-person palette at all -- and
// every session so far has had to guess between them. This prints the actual numbers once.
//
// How to read it:
//   * positions clustered within a metre of node 0, bases valid, two mirrored hand-shaped groups
//       -> a real skeleton. The pointer is right and the problem is the MAP.
//   * wild magnitudes, invalid bases, no structure
//       -> the POINTER is wrong; no amount of discovery helps. Suspect PaletteHook.cpp's eight
//          unguarded offsets, measured on a binary whose function address already moved 16 bytes.
//
// d0 is centimetres from node 0, which is what makes a hand visible by eye: fingertips land
// 15-25 cm out, a shoulder 20-40, and the two hands mirror one another.
void dump_palette(const pa::PaletteAccess& access) {
#if HALO_VR_DEV
    API::get()->log_info("[Halo-CampE-UEVR] PADUMP: %u nodes, player=%d slot=%d "
                         "(d0 = cm from node 0; basis 1 = orthonormal)",
                         access.node_count, access.local_player, access.weapon_slot);
    const pa::Vec3 root = access.palette[0].position;
    for (std::uint32_t i = 0; i < access.node_count; ++i) {
        const pa::BlamMatrix4x3& m = access.palette[i];
        const float d0 = pa::length(m.position - root) * pa::kMetresPerBlamUnit * 100.0f;
        API::get()->log_info("[Halo-CampE-UEVR] PADUMP %02u pos=(%8.4f,%8.4f,%8.4f) d0=%7.2fcm "
                             "scale=%.3f basis=%d reasonable=%d",
                             i, m.position.x, m.position.y, m.position.z, d0, m.scale,
                             pa::valid_basis(pa::orthonormal_basis(m)) ? 1 : 0,
                             pa::reasonable_palette_node(m) ? 1 : 0);
    }
    // THE VERDICT FROM THIS SAME FRAME. Without it the dump is 76 numbers from one frame paired
    // with a verdict logged on some other frame -- and discovery has already been observed giving
    // three different answers ("no hand-shaped cluster", "no shoulder/elbow above a wrist", "more
    // than two hands") across four samples of what should be one stable skeleton. Correlating
    // across frames is exactly how that instability got mistaken for a settled diagnosis.
    API::get()->log_info("[Halo-CampE-UEVR] PADUMP: verdict THIS frame: discovery=%s tier=%s "
                         "detail=%s | unattributed=%u budget_exhausted=%u attempts=%llu",
                         pa::discovery_name(s_node_map.last_discovery()),
                         pa::nodemap_tier_name(s_node_map.tier()),
                         s_node_map.detail(),
                         (unsigned)s_node_map.unattributed(),
                         (unsigned)s_node_map.budget_exhausted(),
                         (unsigned long long)s_node_map.resolve_attempts());
    API::get()->log_info("[Halo-CampE-UEVR] PADUMP: end -- set padump=0 then 1 to repeat");
#else
    (void)access;
#endif
}

bool drive_palette(const pa::PaletteAccess& access) {
    if (!s_tracking_ready.load(std::memory_order_acquire)) {
        s_drive_stage = "no tracking snapshot"; return false;
    }
    const TrackingSnapshot tracking = s_tracking;
    if (!tracking.valid) { s_drive_stage = "tracking snapshot invalid"; return false; }

    // THE DUMP FIRES BEFORE THE GUARD, deliberately: the case we most need numbers for is exactly
    // the one where the map is about to be refused, and running it after would print only on the
    // frames that already work.
    HALO_VR_DEV_ONLY(
        {
            static bool s_dump_prev = false;
            const bool want = g_cfg.pa_dump;
            if (want && !s_dump_prev) dump_palette(access);
            s_dump_prev = want;
        });

    // The guard, now with a cascade in front of it. Rung 1 derives the map from THIS palette; both
    // rungs are then judged by the same nodemap_validate(), which does not ask "did the numbers
    // change" but "does the thing at this index behave like a wrist". A null return leaves the
    // stock pose alone, which is the correct fail-closed answer.
    // Resolve ONLY when we have no map, or the skeleton changed. A null result deliberately does
    // NOT update the key: with no map we pose nothing, so the palette stays stock and retrying next
    // frame is both safe and the only way back.
    if (s_map_cached == nullptr || access.model_tag != s_map_key_tag
        || access.node_count != s_map_key_nodes) {
        const pa::NodeMap* resolved = s_node_map.resolve(access.palette, access.node_count);
        if (resolved != nullptr) {
            s_map_cached    = resolved;
            s_map_key_tag   = access.model_tag;
            s_map_key_nodes = access.node_count;
        } else {
            s_map_cached = nullptr;
        }
    }

    const pa::NodeMap* map = s_map_cached;
    if (map == nullptr) {
        // Report only when the news changes -- this runs on the frame path.
        if (s_map_reporter.changed(0, "none")) {
            const pa::NodeMapFailure& site = pa::nodemap_last_failure_site();
            API::get()->log_info(
                "[Halo-CampE-UEVR] PALETTEARM: no usable node map -- arms stay stock. "
                "discovery=%s, hardcoded table rejected (%s)%s%s%s. %u nodes, %llu attempts, "
                "%u candidates lost to the search budget.",
                pa::discovery_name(s_node_map.last_discovery()), pa::nodemap_last_failure(),
                site.has_site ? site.right_arm ? " on the right " : " on the left " : "",
                site.has_site ? pa::nodemap_chain_name(site.chain) : "",
                site.has_site ? " chain" : "",
                access.node_count, (unsigned long long)s_node_map.resolve_attempts(),
                (unsigned)s_node_map.budget_exhausted());
        }
        return false;
    }
    if (s_map_reporter.changed(reinterpret_cast<uintptr_t>(map),
                               pa::nodemap_tier_name(s_node_map.tier()))) {
        API::get()->log_info(
            "[Halo-CampE-UEVR] PALETTEARM: node map = %s (%u nodes; right wrist %u, left wrist %u; "
            "%u nodes left stock as unattributable).",
            pa::nodemap_tier_name(s_node_map.tier()), access.node_count,
            map->right.wrist, map->left.wrist, (unsigned)s_node_map.unattributed());
    }

    // ---- SMOKE TEST. Shove everything but the root straight up and see whether the screen cares.
    //
    // Deliberately crude, and deliberately BEFORE any solving: it isolates one question -- does
    // writing this memory change what is drawn? The drive already reports success on every call
    // ("ok=9992" against "hook 9992 calls") while the arms sit still, so that is the only question
    // left worth asking. If half a metre of lift is invisible, the renderer is not reading what we
    // write, and no amount of IK tuning will ever help.
    // #if, not HALO_VR_DEV_ONLY(): the braced initialiser below contains commas, and a
    // function-like macro splits its argument on those. Same trap, same one-line fix.
#if HALO_VR_DEV
    if (g_cfg.pa_test_lift != 0.0f) {
        const pa::Vec3 lift{0.0f, 0.0f, g_cfg.pa_test_lift / pa::kMetresPerBlamUnit};
        for (std::uint32_t i = 1; i < access.node_count; ++i) {
            access.palette[i].position = access.palette[i].position + lift;
        }
        s_drive_stage = "SMOKE TEST LIFT APPLIED";
        s_drive_ok.fetch_add(1, std::memory_order_relaxed);
        return true;   // nothing else runs: the lift is the whole experiment
    }
#endif

    const pa::Mat3 root_basis  = pa::orthonormal_basis(access.palette[map->root]);
    if (!pa::valid_basis(root_basis)) { s_drive_stage = "root basis invalid"; return false; }
    const pa::Vec3 root_position = access.palette[map->root].position;
    // ---- COMPOSITION. Stage-anchored, not HMD-relative.
    //
    // The palette root is the stick-driven game camera and contains no HMD content; UEVR renders
    // the view as game-camera * (recenter-offset * hmd), so the recenter offset is the exact map
    // from stage space onto that root.
    const pa::Quat composition = pa::normalized(tracking.stage_rotation);

    // ---- THE TORSO FRAME HANGS OFF THE HEAD, NOT THE CAMERA.
    //
    // This used to derive from root_basis alone -- and root IS the game camera, whose yaw this
    // plugin drives from the aim controller. So the shoulders rotated with AIM: turn the
    // controller and the whole rig swung with it, which is not what a body does and is exactly
    // what "I should not be able to rotate my controllers and have the whole rig offset" means.
    //
    // A torso follows the HEAD. The head in palette space is root * (composed hmd), and
    // torso_basis_from_root() then keeps yaw only -- so head pitch and roll never swing the
    // shoulders either.
    const pa::Mat3 head_basis = xr_rotation_to_blam_basis(
        pa::normalized(composition * tracking.hmd_rotation));
    pa::Mat3 torso_source = root_basis;
    if (pa::valid_basis(head_basis)) {
        if (g_cfg.pa_torso_frame == 1) {
            torso_source = pa::multiply(root_basis, head_basis);            // add head yaw
        } else if (g_cfg.pa_torso_frame == 2) {
            torso_source = pa::multiply(root_basis, pa::transpose(head_basis));  // remove head yaw
        }
    }

    // ---- 3/4: THE BODY FRAME. This is the one that reconciles us with the original design.
    //
    // elliotttate's rig (main.cpp: torso_basis_from_root, ported verbatim as ours) hangs the
    // shoulders off palette[0] with NO head term -- our mode 0 -- and it works for him. The reason
    // is one line in his setup:
    //
    //     API::VR::set_aim_method(API::VR::AimMethod::GAME);      // his main.cpp:4295
    //
    // Under GAME aim the camera yaw is the player's stick-driven BODY facing, so palette[0] really
    // is a torso frame. We run aimdirect and OVERWRITE that yaw from the aim controller, so our
    // palette[0] means AIM. Feeding an aim frame to a function that requires a body frame is
    // exactly why the shoulders follow the controller -- and why mode 1 then double-counts head yaw
    // the view has already applied. Same maths, different meaning of the input.
    //
    // The delta is not re-derived: the view lock measures it every frame. g_dbg_view_in is the
    // camera yaw the aim drove; g_dbg_view_out is the locked view we render, i.e. the body. Undoing
    // that rotation on the root converts aim frame -> body frame.
    //
    // TWO SIGNS, because handedness between the UE rotator and the Blam basis is a coin-flip that
    // costs a headset round-trip to settle: 3 applies the delta, 4 applies its negation. Whichever
    // holds the shoulders still while the controller turns is the correct one, and the loser gets
    // deleted along with this note.
    // ---- 6: TORSO FACES THE HANDS, BLENDED TOWARD THE HEAD. -----------------------------------
    //
    //     yaw = yaw(hands) + shortest_arc(yaw(head) - yaw(hands)) * headshouldersyawinfluence
    //
    // hands = the midpoint of both controllers relative to the head; one controller if only one is
    // tracked; head-only if neither is. influence 1 = pure head, 0 = pure hands.
    //
    // THE AIM MUST STILL BE SUBTRACTED. An earlier version of this comment claimed both terms were
    // camera-relative and would counter-rotate for free. THAT WAS WRONG, and the sweep that was
    // meant to prove it actually disproved it:
    //
    //   head fixed, hands fixed, aim swept 0/45/-45/90 deg  ->  sh IDENTICAL at every angle
    //
    // which is the SYMPTOM, not the proof. `composition` is UEVR's recenter offset: it maps stage
    // space onto the root and is CONSTANT -- it does not track the camera. So head_basis and the
    // hand direction are stage-relative and fixed, and a shoulder placed from them sits still in
    // CAMERA space, i.e. it rides the aim. For the shoulder to hold still in the WORLD while the
    // camera yaws, its camera-local position has to MOVE.
    //
    // So mode 6 carries the same yaw(view - camera) term modes 3/4/5 use, composed with the blend.
    //
    // Shortest-arc, not a raw lerp: blending 179 and -179 the naive way sweeps the torso the long
    // way round through zero.
    if (g_cfg.pa_torso_frame == 6) {
        float yaw_head = 0.0f;
        bool  have_head = false;
        if (pa::valid_basis(head_basis)) {
            yaw_head = std::atan2(head_basis.forward.y, head_basis.forward.x);
            have_head = true;
        }

        // Hand midpoint, in the same camera-relative frame the wrist targets use.
        pa::Vec3 mid{0.0f, 0.0f, 0.0f};
        int hands = 0;
        if (tracking.support_valid) {
            mid = mid + tracking.support_grip_position;
            ++hands;
        }
        mid = mid + tracking.aim_grip_position;   // the aim hand is always present here
        ++hands;
        mid = mid / (float)hands;

        const pa::Vec3 to_hands_xr = pa::rotate(composition, mid - tracking.hmd_position);
        const pa::Vec3 to_hands    = xr_to_blam(to_hands_xr);
        const float    flat        = std::sqrt(to_hands.x * to_hands.x + to_hands.y * to_hands.y);

        float yaw = yaw_head;
        // Below ~5 cm of horizontal separation the direction is noise, not intent -- hands held
        // against the chest would otherwise spin the torso. Fall back to the head there.
        if (flat > 0.015f) {                     // palette units: ~4.6 cm
            const float yaw_hands = std::atan2(to_hands.y, to_hands.x);
            if (have_head) {
                float d = yaw_head - yaw_hands;
                while (d >  3.14159265f) d -= 6.28318531f;
                while (d < -3.14159265f) d += 6.28318531f;
                yaw = yaw_hands + d * g_cfg.pa_head_shoulders_yaw_influence;
            } else {
                yaw = yaw_hands;
            }
        }

        // SAME SIGN AS THE TWO CONFIRMED LIFTS, which is (camera - view), i.e. NEGATED.
        //
        // This read +delta ("same sign as mode 3") and that was wrong. The weapon lift (pawpnlift)
        // and the arm lift (paarmlift) both use -delta and were both confirmed correct in a
        // headset. All three do the IDENTICAL conversion -- take a stage-relative quantity and
        // express it in CAMERA space -- so they cannot legitimately differ in sign. The head and
        // hand yaws feeding this blend are stage-relative exactly like the hand pose is.
        //
        // Mode 3's sign was judged in-headset while BOTH the weapon and the arms were still
        // double-counting the aim, so that judgement was made through two known-broken terms and
        // is not evidence for anything.
        const float gap = -::halo::g_view_lock_delta.load() * 0.01745329252f;
        const float cg = std::cos(gap), sg = std::sin(gap);
        const pa::Mat3 gapm{ pa::Vec3{ cg, sg, 0.0f}, pa::Vec3{-sg, cg, 0.0f},
                             pa::Vec3{0.0f, 0.0f, 1.0f} };
        const float cy = std::cos(yaw), sy = std::sin(yaw);
        const pa::Mat3 blendm{ pa::Vec3{ cy, sy, 0.0f}, pa::Vec3{-sy, cy, 0.0f},
                               pa::Vec3{0.0f, 0.0f, 1.0f} };
        torso_source = pa::multiply(root_basis, pa::multiply(gapm, blendm));
        s_dbg_yaw_hands = yaw * 57.2957795f;
        s_dbg_yaw_head  = yaw_head * 57.2957795f;
    }

    // ---- 5: THE HEAD FRAME WITH THE AIM REMOVED.
    //
    // Mode 3 gives the BODY frame: it subtracts the aim but carries no head term, so the shoulders
    // ignore your head entirely. Mode 1 carries the head but never subtracts the aim, so aim leaks
    // straight through -- that is the "extra yaw" that was reported. This is both at once:
    //
    //     torso = root o yaw(view - camera) o head       (aim removed, THEN head followed)
    //
    // torso_basis_from_root() still keeps YAW ONLY afterwards, so head pitch and roll never swing
    // the shoulders -- elliotttate's original intent ("follows the head's position and yaw but
    // never its pitch or roll"), and the reason looking down does not fold the arms into the view.
    if (g_cfg.pa_torso_frame == 5 && pa::valid_basis(head_basis)) {
        const float gap = ::halo::g_view_lock_delta.load() * 0.01745329252f;
        const float cg = std::cos(gap), sg = std::sin(gap);
        const pa::Mat3 gapm{ pa::Vec3{ cg, sg, 0.0f}, pa::Vec3{-sg, cg, 0.0f},
                             pa::Vec3{0.0f, 0.0f, 1.0f} };
        torso_source = pa::multiply(root_basis, pa::multiply(gapm, head_basis));
    }

    if (g_cfg.pa_torso_frame == 3 || g_cfg.pa_torso_frame == 4) {
        float d = ::halo::g_view_lock_delta.load();
        if (g_cfg.pa_torso_frame == 4) d = -d;
        const float a = d * 0.01745329252f;
        const float c = std::cos(a), sn = std::sin(a);
        // Yaw about the palette's up axis (+Z), in the forward/left/up triple this folder uses.
        const pa::Mat3 yaw_only{ pa::Vec3{c, sn, 0.0f},
                                 pa::Vec3{-sn, c, 0.0f},
                                 pa::Vec3{0.0f, 0.0f, 1.0f} };
        torso_source = pa::multiply(root_basis, yaw_only);
    }
    const pa::Mat3 torso_basis = pa::torso_basis_from_root(torso_source);
    if (!pa::valid_basis(torso_basis)) {
        s_drive_stage = "torso basis invalid";
        return false;
    }

    // Which physical hand aims. Asked once, so left-handed play needs no second code path.
    const bool aim_is_right = !g_cfg.aim_left_hand;

    const pa::ArmNodes& aim_arm     = map->arm(aim_is_right);
    const pa::ArmNodes& support_arm = map->arm(!aim_is_right);

    // ---- BOTH HANDS ARE IK'd TO THEIR CONTROLLERS. THE WEAPON IS NOT TOUCHED.
    //
    // The weapon keeps its own calibrated driver (Rig.cpp's controller attachment, carrying
    // grip_deg/grip_yaw/grip_roll, off_x/y/z and the per-weapon wpnoff deltas). Posing it from
    // here instead -- which this briefly did -- discards that calibration and moves the gun off
    // where it was tuned to sit. Since the weapon is calibrated TO the controller, a hand placed
    // at the controller is a hand on the weapon, and the calibration stays the single source of
    // truth for where the gun lives.
    //
    // Each hand uses ITS OWN latched wrist convention. The two wrist bones are mirrored, so one
    // controller-to-bone formula for both puts the left hand in the right hand's frame.
    struct HandPlan {
        const pa::ArmNodes* arm;
        pa::Quat            grip_rotation;
        pa::Vec3            grip_position;
        WristConvention*    conv;
        bool                left_side;   // for the shoulder anchor
        bool                is_aim;
    };

    ::halo::Quat aim_q{tracking.aim_aim_rotation.x, tracking.aim_aim_rotation.y,
                       tracking.aim_aim_rotation.z, tracking.aim_aim_rotation.w};
    ::halo::two_hand_bend_orientation(&aim_q);

    HandPlan plans[2] = {
        { &aim_arm,     pa::Quat{aim_q.x, aim_q.y, aim_q.z, aim_q.w},
          tracking.aim_grip_position,
          aim_is_right ? &s_conv_right : &s_conv_left, !aim_is_right, true },
        { &support_arm, tracking.support_grip_rotation,
          tracking.support_grip_position,
          aim_is_right ? &s_conv_left : &s_conv_right, aim_is_right, false },
    };

    // The weapon's rigid carry, hoisted so the SUPPORT hand can ride it during a two-hand hold.
    // Same transform, same frame -- the hand lands on the gun because it is carried by the very
    // delta that put the gun there, not by a second guess at where the gun ended up.
    pa::Mat3 wpn_delta_basis{};
    pa::Vec3 wpn_delta_pos{};
    bool     wpn_delta_valid = false;

    // ---- THE WEAPON BRANCH, carried onto the aim controller. ROUTE (c).
    //
    // ONE rigid transform for nodes {7, 8, 22}, taking the branch from its AUTHORED pose onto where
    // the hand is. Rigid on purpose: everything inside the branch -- recoil, reload, the whole stock
    // animation -- survives untouched, because we move the branch, not its contents.
    //
    // The marker (node 8, the authored "primaryweapon" point) is what lands on the controller, so
    // the gun sits where the ARTIST said it meets the hand. That is the "already authored position"
    // half of the design, and it is why this path needs no grip calibration of its own.
    //
    // Done BEFORE the arms below, so the aim hand IKs to a weapon that has already moved.
    if (g_cfg.pa_weapon && map->weapon_count > 0 && map->weapon_marker != pa::kNoNode) {
        const pa::Mat3 wctrl = xr_rotation_to_blam_basis(
            pa::normalized(composition * pa::Quat{aim_q.x, aim_q.y, aim_q.z, aim_q.w}));
        const pa::Mat3 stock_w = pa::orthonormal_basis(access.palette[map->weapon_marker]);
        if (pa::valid_basis(wctrl) && pa::valid_basis(stock_w)) {
            // GRIP TRIM, in the controller's own frame, so it reads as "rotate the gun in my hand"
            // rather than "rotate it about the camera". Applied on the RIGHT of the controller basis
            // for exactly that reason -- on the left it would swing the gun around the view instead.
            pa::Mat3 wgrip = wctrl;
            if (g_cfg.pa_wpn_yaw != 0.0f || g_cfg.pa_wpn_pitch != 0.0f || g_cfg.pa_wpn_roll != 0.0f) {
                const float cy = std::cos(g_cfg.pa_wpn_yaw   * 0.01745329252f);
                const float sy = std::sin(g_cfg.pa_wpn_yaw   * 0.01745329252f);
                const float cp = std::cos(g_cfg.pa_wpn_pitch * 0.01745329252f);
                const float sp = std::sin(g_cfg.pa_wpn_pitch * 0.01745329252f);
                const float cr = std::cos(g_cfg.pa_wpn_roll  * 0.01745329252f);
                const float sr = std::sin(g_cfg.pa_wpn_roll  * 0.01745329252f);
                const pa::Mat3 yawm  { pa::Vec3{ cy, sy, 0.0f}, pa::Vec3{-sy, cy, 0.0f},
                                       pa::Vec3{0.0f, 0.0f, 1.0f} };
                const pa::Mat3 pitchm{ pa::Vec3{ cp, 0.0f, sp}, pa::Vec3{0.0f, 1.0f, 0.0f},
                                       pa::Vec3{-sp, 0.0f, cp} };
                const pa::Mat3 rollm { pa::Vec3{1.0f, 0.0f, 0.0f}, pa::Vec3{0.0f, cr, sr},
                                       pa::Vec3{0.0f, -sr, cr} };
                wgrip = pa::multiply(wctrl, pa::multiply(yawm, pa::multiply(pitchm, rollm)));
            }

            // ---- THE PER-WEAPON RIGID DELTA (wpnfix), composed onto that global trim.
            //
            // Same right-multiply, one factor later, and its translation is carried by the trimmed
            // basis -- see the WeaponCarry note at the top of this file for the full chain. The
            // entry itself is resolved on the GAME thread (weapon_fix_tick) and published: finding
            // it needs the held weapon's class name, and reflection has no business running inside
            // this detour.
            //
            // BOTH HALVES OR NEITHER. If the composed basis is not a rotation the delta is dropped
            // whole rather than applying its translation against an untrimmed orientation, which
            // would move the gun somewhere neither value describes.
            pa::Mat3 wgrip_w = wgrip;
            pa::Vec3 wfix_shift{0.0f, 0.0f, 0.0f};
            if (s_wfix_have.load(std::memory_order_acquire)) {
                const pa::Mat3 composed = pa::multiply(wgrip, s_wfix_rot);
                if (pa::valid_basis(composed)) {
                    wgrip_w    = composed;
                    wfix_shift = pa::transform_vector(wgrip, s_wfix_pos);
                }
            }

            // ---- THE BARREL LOCK, LAST OF THE ORIENTATION STAGES AND BEFORE THE PUBLISH.
            //
            // Constrain the carry so the gun points where it shoots. Full rationale and the
            // coarse/fine split against wpnfix: Config.hpp pa_barrel_lock. Three facts decide
            // everything about this block:
            //
            // 1. THE ORDER. AFTER the global trim and wpnfix, not before. Both were solved as "what
            //    puts the barrel on the ray", so locking first and trimming after would apply each
            //    correction twice and every captured wpnfix in the wild would become wrong.
            //    blindcowboy24 locks first because his gripfix is a separate global capture; ours
            //    is the same knob, so ours goes last. Nothing already stored changes meaning.
            //
            // 2. IT MUST BE UPSTREAM OF s_wcarry. He measured the alternative and it is nasty
            //    (BlamPalette.cpp:1555): if the capture latches a pose the lock has not touched
            //    while the player aligns to the one on screen, the solve measures the LOCK'S OWN
            //    correction and writes it into the per-weapon file -- 7.6 -> 23.8 degrees in one
            //    hold on his build, and then the lock silently quits at its cap. Publishing the
            //    LOCKED basis makes both sides of the solve on-ray poses, so a capture can only
            //    ever move the roll and the position -- the two things the player can actually see.
            //
            // 3. ORIENTATION ONLY. wfix_shift above is deliberately left alone: the lock is a few
            //    degrees of dynamic correction, and feeding it into the position would make the gun
            //    twitch in place every time the aim drive's filter lagged. His does the same -- his
            //    hand_cm never sees the lock.
            //
            // COST: two 3x3 transforms, a dot and an acos, on the frames where a weapon is carried.
            //
            // MEASURED IN DEV BUILDS EVEN WHEN THE LOCK IS OFF, which is the point of the `|| DEV`:
            // the residual it WOULD correct is the number that says whether this build's stock pose
            // really points down the camera forward, and it has to be readable before anyone flips
            // the switch. Nothing is written to the pose unless the switch is on.
            if (g_cfg.pa_barrel_lock || HALO_VR_DEV) {
                // THE BARREL, in the marker bone's own frame, read off the pose the game itself
                // just produced. In the stock pose the gun points down the camera forward (that is
                // what makes hipfire work), so the barrel's bone-frame direction is the camera
                // forward brought into that frame -- and stock_w and root_basis are both palette
                // quantities from THIS frame, so there is no staleness and no frame to convert.
                const pa::Vec3 beta =
                    pa::transform_vector(pa::transpose(stock_w), root_basis.forward);
                // THE AIM RAY, in the STAGE frame that wgrip_w lives in.
                //
                //   desired_basis = root_basis * gapm * wgrip_w      (see the lift below)
                //   aim (palette) = root_basis.forward
                //   => aim (stage) = (root_basis * gapm)^T * root_basis.forward
                //                  = gapm^T * X = (cos gap, -sin gap, 0)
                //
                // gapm is the lock-gap yaw the lift applies; its transpose's forward column is that
                // pair, which is why this needs two lines rather than a matrix. IT MUST TRACK THE
                // LIFT: same pa_wpn_lift gate, same sign, same g_view_lock_delta. Dropping the gap
                // would lock to the VIEW's forward instead of the CAMERA's, and those differ by the
                // lock gap -- measured at 98-125 degrees on this title, not a rounding error.
                const float bl_gap = (g_cfg.pa_wpn_lift != 0)
                    ? -::halo::g_view_lock_delta.load() * 0.01745329252f
                    : 0.0f;
                const pa::Vec3 aim_stage{std::cos(bl_gap), -std::sin(bl_gap), 0.0f};
                const pa::Vec3 barrel_stage = pa::transform_vector(wgrip_w, beta);
                const pa::Mat3 locked = pa::multiply(
                    pa::barrel_lock_correction(barrel_stage, aim_stage,
                                               g_cfg.pa_barrel_full, g_cfg.pa_barrel_release),
                    wgrip_w);
                // GATED ON THE CONFIG, NOT ON THE ENCLOSING `if`. A dev build enters this block
                // unconditionally to publish the residual below; letting the write follow it would
                // make the diagnostic change what is drawn, which is the exact failure the whole
                // HALO_VR_DEV split exists to prevent.
                //
                // Same both-halves-or-neither rule as the wpnfix compose above: a correction that
                // did not come out a rotation leaves the gun exactly where it already was.
                if (g_cfg.pa_barrel_lock && pa::valid_basis(locked)) wgrip_w = locked;

                // #if, not HALO_VR_DEV_ONLY(): a function-like macro splits its argument on any
                // comma not inside PARENTHESES, and braces do not protect one. Same trap, and the
                // same one-line fix, as the smoke-test lift further up this file.
#if HALO_VR_DEV
                {
                    const pa::Vec3 bn = pa::normalized(barrel_stage);
                    float al = pa::dot(bn, aim_stage);
                    al = al < -1.0f ? -1.0f : (al > 1.0f ? 1.0f : al);
                    const float off = (pa::length_squared(bn) < 0.8f)
                        ? -1.0f : std::acos(al) * 57.2957795131f;
                    s_dbg_barrel_off.store(off, std::memory_order_relaxed);
                    // The fade's own answer, re-derived from `off` rather than plumbed out of the
                    // pure function -- keeping barrel_lock_correction() single-valued is what lets
                    // Verify-PaletteArm.ps1 test it without a diagnostics parameter.
                    const float f0 = g_cfg.pa_barrel_full;
                    const float f1 = g_cfg.pa_barrel_release;
                    float str = 0.0f;
                    if (off >= 0.0f && f0 > 0.0f) {
                        str = (f1 > f0) ? (1.0f - pa::smoothstep(f0, f1, off))
                                        : ((off <= f0) ? 1.0f : 0.0f);
                    }
                    s_dbg_barrel_str.store(g_cfg.pa_barrel_lock ? str : 0.0f,
                                           std::memory_order_relaxed);
                }
#endif
            }

            // The stage-frame hand, BEFORE the lock-gap lift -- the frame the delta and the capture
            // both live in. Hoisted above the lift so the freeze below can substitute for it.
            const pa::Vec3 dxr = pa::rotate(composition,
                                            tracking.aim_grip_position - tracking.hmd_position);
            pa::Vec3 stage_pos = xr_to_blam(dxr) / pa::kMetresPerBlamUnit + wfix_shift;

            // PUBLISH THE LIVE CARRY, before the freeze can replace it. This is the pair the
            // capture edge differences against; publishing the frozen values instead would make
            // every hold solve to identity, which looks exactly like a gesture that does nothing.
            s_wcarry.basis  = wgrip_w;
            s_wcarry.offset = stage_pos;
            s_wcarry_valid.store(true, std::memory_order_release);

            // ---- THE CAPTURE FREEZE, in the STAGE frame.
            //
            // Hold the weapon on the stage-frame pose it had at key-down and let the player bring
            // their controller onto it. Stage frame is the whole trick: our aim drive moves the
            // GAME CAMERA from this same controller, so a camera-space latch would ride every aim
            // motion the alignment itself causes -- blindcowboy24 measured exactly that ("my camera
            // was changing the position/rotation of it slightly") and had to move his freeze into
            // world space to escape it. Ours never enters camera space: the lift that follows is
            // the same one the live carry uses, so a stationary stage-frame pose renders as
            // stationary in the world for precisely the reason the live carry does not swim.
            //
            // And the SOLVE is immune either way -- both sides of it are stage-frame quantities, so
            // it is exact even if the lift's sign were wrong. A wrong lift would make the frozen gun
            // visibly drift while you align to it, which is a thing you can see, not a silent error.
            if (s_wfreeze_active.load(std::memory_order_acquire)) {
                if (pa::valid_basis(s_wfreeze.basis)) {
                    wgrip_w   = s_wfreeze.basis;
                    stage_pos = s_wfreeze.offset;
                }
            }

            // LIFT THE ROOM-FRAME HAND INTO THE CAMERA FRAME BY THE LOCK GAP ONLY.
            // See Config.hpp pa_wpn_lift: lifting by the full camera yaw double-counts the aim,
            // because our own aim drive wrote that camera yaw from this same controller.
            pa::Mat3 lifted = wgrip_w;
            if (g_cfg.pa_wpn_lift != 0) {
                const float gap = -::halo::g_view_lock_delta.load() * 0.01745329252f;  // camera-view
                const float cg = std::cos(gap), sg = std::sin(gap);
                const pa::Mat3 gapm{ pa::Vec3{ cg, sg, 0.0f}, pa::Vec3{-sg, cg, 0.0f},
                                     pa::Vec3{0.0f, 0.0f, 1.0f} };
                lifted = pa::multiply(gapm, wgrip_w);
            }
            const pa::Mat3 desired_basis = pa::multiply(root_basis, lifted);
            // THE POSITION HALF OF THE LIFT. This was missing while the ORIENTATION half was
            // applied, so the gun pointed correctly but sat in the wrong PLACE by an amount
            // proportional to the lock gap.
            //
            // Reported as: "snap turn gives no drift, but physically yawing with the controller in
            // hand makes the weapon drift away from the controller and the hands." That asymmetry
            // is the tell. A snap turn moves g_turn_offset, which shifts view AND camera together
            // and leaves the gap unchanged. Physically yawing rotates the CONTROLLER too, which
            // moves the aim, which moves the camera -- so the gap changes, and an error
            // proportional to the gap only shows up in that case.
            //
            // The arms already got both halves (see arm_gap below); the weapon only got one.
            pa::Vec3 dbl = stage_pos;
            if (g_cfg.pa_wpn_lift != 0) {
                const float gp = -::halo::g_view_lock_delta.load() * 0.01745329252f;
                const float cp = std::cos(gp), sp = std::sin(gp);
                const pa::Mat3 gpm{ pa::Vec3{ cp, sp, 0.0f}, pa::Vec3{-sp, cp, 0.0f},
                                    pa::Vec3{0.0f, 0.0f, 1.0f} };
                dbl = pa::transform_vector(gpm, dbl);
            }
            const pa::Vec3 desired_pos =
                root_position + pa::transform_vector(root_basis, dbl);

            const pa::Mat3 delta_basis = pa::multiply(desired_basis, pa::transpose(stock_w));
            const pa::Vec3 carried =
                pa::transform_vector(delta_basis, access.palette[map->weapon_marker].position);
            const pa::Vec3 delta_pos{desired_pos.x - carried.x,
                                     desired_pos.y - carried.y,
                                     desired_pos.z - carried.z};

            // REFUSE ANYTHING OUTSIDE ARM'S REACH before writing. blindcowboy24 records a crash
            // from a sleeping controller reading (0,0,0), which turns the offset into the negated
            // head position and produces a 2.4 m "hand" -- finite, plausible-looking, and fatal.
            // One bad frame is enough, so this is checked on the DESTINATION, not after the fact.
            const float reach = pa::length(desired_pos - root_position);
            if (pa::valid_basis(delta_basis) && std::isfinite(reach) && reach < kWeaponReachMax) {
                pa::apply_rigid_transform(access.palette, map->weapon_nodes, map->weapon_count,
                                          delta_basis, delta_pos);
                s_dbg_wpn_x = desired_pos.x; s_dbg_wpn_y = desired_pos.y;
                s_dbg_wpn_z = desired_pos.z; s_dbg_wpn_ok = true;
                wpn_delta_basis = delta_basis;
                wpn_delta_pos   = delta_pos;
                wpn_delta_valid = true;
                s_wpn_driven = true;
                s_dbg_wpn_reach = reach * pa::kMetresPerBlamUnit * 100.0f;
            } else {
                s_wpn_driven = false;
            }
        }
    }

    bool any_posed = false;
    for (int i = 0; i < 2; ++i) {
        const HandPlan& plan = plans[i];
        const int arm_bit = plan.is_aim ? 1 : 2;
        if ((g_cfg.pa_arms & arm_bit) == 0) { HALO_VR_DEV_ONLY(if (!plan.is_aim) ++s_bail[0];); continue; }
        if (!plan.is_aim && !tracking.support_valid) { HALO_VR_DEV_ONLY(++s_bail[1];); continue; }

        pa::Mat3 controller =
            xr_rotation_to_blam_basis(pa::normalized(composition * plan.grip_rotation));
        if (!pa::valid_basis(controller)) { HALO_VR_DEV_ONLY(if (!plan.is_aim) ++s_bail[2];); continue; }

        // THE STAGE-FRAME HAND OFFSET, HOISTED ABOVE THE LIFT.
        //
        // Identical arithmetic to the delta_xr/delta_blam pair this used to compute further down --
        // it is only moved, so that the support-hand fix and its freeze below have a pose PAIR
        // (basis + offset) to act on in the same frame, before either gap rotates it. The lift is
        // re-applied at the original site and nothing about the aim hand changes.
        pa::Vec3 stage_off =
            xr_to_blam(pa::rotate(composition, plan.grip_position - tracking.hmd_position)) /
            pa::kMetresPerBlamUnit;

        // ---- THE SUPPORT-HAND RIGID FIX, and its capture freeze. SUPPORT HAND ONLY.
        //
        // Composed exactly as the weapon fix is (see the statics at the top of this file): the
        // rotation RIGHT-multiplies, so it reads as "rotate the hand on my controller" rather than
        // "swing it around the view", and the translation is carried by the UNTRIMMED basis, which
        // is what makes the two one rigid attachment instead of two knobs that interact.
        //
        // BOTH HALVES OR NEITHER, same rule as the weapon: a composed basis that is not a rotation
        // drops the whole delta rather than shifting the hand against an untrimmed orientation, and
        // lands it somewhere neither value describes.
        if (!plan.is_aim) {
            if (s_hfix_have.load(std::memory_order_acquire)) {
                const pa::Mat3 composed = pa::multiply(controller, s_hfix_rot);
                if (pa::valid_basis(composed)) {
                    stage_off  = stage_off + pa::transform_vector(controller, s_hfix_pos);
                    controller = composed;
                }
            }
            // PUBLISH THE LIVE PAIR BEFORE THE FREEZE CAN REPLACE IT. Publishing the frozen values
            // instead would make every hold solve to identity -- indistinguishable from a gesture
            // that does nothing, which is the failure the weapon fix's note records paying for.
            s_hcarry.basis  = controller;
            s_hcarry.offset = stage_off;
            s_hcarry_valid.store(true, std::memory_order_release);

            if (s_hfreeze_active.load(std::memory_order_acquire) &&
                pa::valid_basis(s_hfreeze.basis)) {
                controller = s_hfreeze.basis;
                stage_off  = s_hfreeze.offset;
            }
        }

        // LIFT BY THE LOCK GAP ONLY -- see Config.hpp pa_arm_lift. Lifting by the full camera yaw
        // double-counts the aim, because our aim drive wrote that camera yaw from this controller.
        pa::Mat3 arm_gap{ pa::Vec3{1.0f, 0.0f, 0.0f}, pa::Vec3{0.0f, 1.0f, 0.0f},
                          pa::Vec3{0.0f, 0.0f, 1.0f} };
        // Kept UNGAPPED for pa_target_frame=1: torso_basis already carries the gap, so composing it
        // with the gapped controller would apply the same rotation twice.
        const pa::Mat3 controller_raw = controller;
        if (g_cfg.pa_arm_lift != 0) {
            const float g = -::halo::g_view_lock_delta.load() * 0.01745329252f;   // camera - view
            const float cg = std::cos(g), sg = std::sin(g);
            arm_gap = pa::Mat3{ pa::Vec3{ cg, sg, 0.0f}, pa::Vec3{-sg, cg, 0.0f},
                                pa::Vec3{0.0f, 0.0f, 1.0f} };
            controller = pa::multiply(arm_gap, controller);
        }

        // THE PITCH HALF -- see Config.hpp pa_arm_pitch. Composed into arm_gap so every consumer
        // below (orientation, position, and the rest-pose lift) gets it without a second code path.
        if (g_cfg.pa_arm_pitch != 0) {
            // CAMERA PITCH ONLY -- deliberately NOT minus the head pitch.
            //
            // What this needs is the rotation from ROOM space into the CAMERA-LOCAL palette, and
            // that is the camera's orientation alone. The head is not the palette's frame, so its
            // pitch has no business in the transform. Subtracting it made the hands move whenever
            // the player pitched their head with the aim perfectly still -- reported in-headset as
            // "if I pitch my head up it offsets them".
            //
            // The measurement that signed this off could not have caught it: that sweep was run with
            // the HMD LEVEL, where hmd_pitch is 0 and the two formulas are arithmetically identical.
            // A control that pins the suspect variable at zero does not test it.
            float gp = ::halo::g_view_pitch.load() * 0.01745329252f;
            if (g_cfg.pa_arm_pitch == 2) gp = -gp;
            const float cp = std::cos(gp), sp = std::sin(gp);
            const pa::Mat3 pitch_gap{ pa::Vec3{ cp, 0.0f, sp}, pa::Vec3{0.0f, 1.0f, 0.0f},
                                      pa::Vec3{-sp, 0.0f, cp} };
            arm_gap    = pa::multiply(pitch_gap, arm_gap);
            controller = pa::multiply(pitch_gap, controller);
        }

        // The convention is the STOCK wrist expressed relative to the root -- see WristConvention.
        // Captured BEFORE this arm is touched, so it describes the authored pose and not our own
        // previous frame's output.
        // THE GROUND TRUTH. The stock wrist is the game's own hand, authored ONTO the weapon --
        // so where it sits is where our target OUGHT to land. Recording it next to the computed
        // target turns "the hand is in the wrong place" into a measured error vector.
        const pa::Vec3 stock_wrist_pos = access.palette[plan.arm->wrist].position;
        if (!plan.is_aim && !access.is_capture_bank) {
            const pa::Vec3 sh0 = access.palette[plan.arm->shoulder].position;
            const pa::Vec3 srel{stock_wrist_pos.x - sh0.x, stock_wrist_pos.y - sh0.y,
                               stock_wrist_pos.z - sh0.z};
            const pa::Vec3 st = pa::transform_vector(pa::transpose(torso_basis), srel);
            s_dbg_stockL_x = st.x; s_dbg_stockL_y = st.y; s_dbg_stockL_z = st.z;
        }
        if (plan.is_aim && !access.is_capture_bank) {
            s_dbg_stock_x = stock_wrist_pos.x;
            s_dbg_stock_y = stock_wrist_pos.y;
            s_dbg_stock_z = stock_wrist_pos.z;
        }

        const pa::Mat3 stock_rel =
            pa::multiply(pa::transpose(root_basis),
                         pa::orthonormal_basis(access.palette[plan.arm->wrist]));
        if (!plan.conv->observe(stock_rel)) { HALO_VR_DEV_ONLY(if (!plan.is_aim) ++s_bail[3];); continue; }

        // ORIENTATION IN THE SAME FRAME AS THE POSITION -- see Config.hpp pa_target_frame.
        //
        // Mode 1 originally routed only the POSITION through the torso frame, which pinned the hand
        // to 0.002 cm but left its ORIENTATION swinging 49.9 deg per frame, because this still
        // composed the LIVE camera basis with the GAPPED controller. Hand in the right place,
        // pointing somewhere new every frame. Position and facing have to share a frame or the
        // second one reintroduces exactly what the first one removed.
        // MODE 2 ONLY, and separate from mode 1 on purpose. The convention in plan.conv->latched is
        // learned against root_basis (see stock_rel just above), so composing it with a different
        // frame risks a CONSTANT MIS-FACING -- a hand in the right place pointing the wrong way.
        // That is invisible to every headless measure here (it is not jitter, it is an offset), so
        // it needs a human's eyes before it can become the default.
        pa::Mat3 desired_wrist =
            (g_cfg.pa_target_frame >= 2)
                ? pa::multiply(pa::multiply(torso_basis, controller_raw), plan.conv->latched)
                : pa::multiply(pa::multiply(root_basis, controller), plan.conv->latched);
        if (!pa::valid_basis(desired_wrist)) { HALO_VR_DEV_ONLY(if (!plan.is_aim) ++s_bail[4];); continue; }

        // The POSITION half of the same lift. Rotating only the orientation would leave the hand
        // still orbiting the camera with the aim while merely pointing correctly -- and it is the
        // hand's PLACE, not its facing, that drags the whole arm.
        //
        // stage_off is the hoisted hmd-relative offset from the top of this loop -- unchanged for
        // the aim hand, and carrying the support-hand fix (or its freeze) for the other.
        const pa::Vec3 delta_blam = pa::transform_vector(arm_gap, stage_off);

        // The grip-to-wrist offset belongs in ROOT-COMPOSED controller space, matching the space
        // the position is already in. In bare controller space it leaves a constant displacement.
        const pa::Vec3 wrist_local{
            -s_arm_tuning.grip_to_wrist_back_m / pa::kMetresPerBlamUnit, 0.0f,
            -s_arm_tuning.grip_to_wrist_down_m / pa::kMetresPerBlamUnit};
        // TARGET FRAME -- see Config.hpp pa_target_frame.
        //
        // Mode 0 places the hand as root_basis * (arm_gap * offset): the LIVE camera basis times the
        // LIVE lock gap. That gap is not small on this title -- the code below records it measured at
        // 98-125 degrees, and per-frame values near 180 occur -- so the target is being swung by up
        // to half a turn about the head. Measured with the support controller PINNED: hand excursions
        // to 101 cm, correlating with the gap at 1.000, and the forearm stretching 10.8 cm because
        // the wrist is placed exactly even when the arm cannot reach that far.
        //
        // Mode 1 places it in the TORSO frame instead. That frame is already gap-corrected, levelled
        // and stable, so the gap enters the arm exactly ONCE and the target and the rest pose finally
        // share a frame -- which also removes the two-pivot mismatch documented at the rest lift.
        //
        // This follows pancreations MCC VR, whose wrist target touches NO live camera orientation at
        // all: theirs is built from piecewise-constant references (base camera position, head-yaw and
        // game-yaw refs) precisely so a live camera cannot throw the hand.
        pa::Vec3 wrist_target =
            (g_cfg.pa_target_frame >= 1)
                ? (root_position + pa::transform_vector(torso_basis, stage_off) +
                   pa::transform_vector(pa::multiply(torso_basis, controller_raw), wrist_local))
                : (root_position + pa::transform_vector(root_basis, delta_blam) +
                   pa::transform_vector(pa::multiply(root_basis, controller), wrist_local));

        if (!pa::anchor_shoulder_to_torso(access.palette, *plan.arm, torso_basis, root_position,
                                          plan.left_side, s_arm_tuning)) {
            HALO_VR_DEV_ONLY(if (!plan.is_aim) ++s_bail[6];);
            continue;
        }

        // LIFT THE ARM'S REST ORIENTATION TOO, not just its anchor and its target.
        //
        // anchor_shoulder_to_torso() is TRANSLATION ONLY -- by design, so the authored pose inside
        // the arm survives for the IK. But that authored pose is expressed in CAMERA space, and the
        // camera carries our aim. So after fixing the shoulder (body frame) and the wrist target
        // (lifted), the arm BETWEEN them still entered the solve with a camera-glued rest
        // orientation -- and solve_two_bone_arm uses the current elbow direction as its pole hint.
        // A pole hint in the wrong frame tilts the elbow plane by the lock gap, which grows as the
        // aim diverges from the view: reported in-headset as "the elbows are curving inwards oddly".
        //
        // Rotated about the SHOULDER, so the anchor just established does not move.
        //
        // AND THAT PIVOT IS THE PROBLEM -- see Config.hpp pa_arm_rest_lift. The wrist TARGET is
        // lifted by this same arm_gap but about the HEAD (its delta is grip - hmd_position). One
        // rotation, two pivots ~0.09 units apart, so the target and the rest pose disagree by
        // roughly 0.09 * gap -- about 22 cm at the 47 degree gaps measured here. The IK can only
        // bridge that by folding the arm, which is the bend swing: per-window bend peak correlates
        // with the lockdelta peak at 0.941, against 0.461 for the pitch term.
        //
        // This lift exists to put the elbow POLE in the right frame, and the pole is now 75 percent
        // body-anchored (ArmTuning::pole_body_fraction), so it may no longer be earning the
        // mismatch it causes. pa_arm_rest_lift gates ONLY this, leaving the target lift alone --
        // pa_arm_lift=0 would disable both and bring back the arms-follow-the-aim problem.
        if (g_cfg.pa_arm_lift != 0 && g_cfg.pa_arm_rest_lift != 0) {
            pa::apply_rigid_delta(access.palette, plan.arm->shoulder_subtree,
                                  plan.arm->shoulder_count, arm_gap,
                                  access.palette[plan.arm->shoulder].position);
        }
        if (!plan.is_aim && !access.is_capture_bank) {
            const pa::Vec3 ssh = access.palette[plan.arm->shoulder].position;
            s_dbg_supsh_x = ssh.x; s_dbg_supsh_y = ssh.y; s_dbg_supsh_z = ssh.z;
            s_dbg_sup_x = wrist_target.x; s_dbg_sup_y = wrist_target.y;
            s_dbg_sup_z = wrist_target.z;
            // Same target, expressed in the BODY frame relative to this arm own shoulder.
            // transpose() of an orthonormal basis is its inverse, so this is the world-to-torso map.
            const pa::Vec3 rel{wrist_target.x - ssh.x, wrist_target.y - ssh.y,
                              wrist_target.z - ssh.z};
            const pa::Vec3 tf = pa::transform_vector(pa::transpose(torso_basis), rel);
            s_dbg_suptf_x = tf.x; s_dbg_suptf_y = tf.y; s_dbg_suptf_z = tf.z;
        }
        if (plan.is_aim && !access.is_capture_bank) {
            // SHOULDER GROUND TRUTH. The player's complaint is "the shoulders yaw with my aim",
            // and no amount of reasoning about torso_basis settles it -- only sweeping the aim
            // with the head held still and watching this number does. Recorded AFTER anchoring,
            // which is the last thing that sets where the shoulder sits.
            const pa::Vec3 sh = access.palette[plan.arm->shoulder].position;
            s_dbg_sh_x = sh.x; s_dbg_sh_y = sh.y; s_dbg_sh_z = sh.z;
        }
        // ---- THE GRAB: put the SUPPORT hand on the gun while the hold is engaged.
        //
        // Without this the support hand only ever IKs to its own free-tracked controller, so a
        // two-hand hold bends the aim and rotates the weapon but the off hand never touches the
        // forestock -- the hold works and does not LOOK like it.
        //
        // The authored support wrist is still the stock pose here: it is not one of the weapon
        // nodes {7,8,22}, so the carry above never touched it. Running it through the SAME delta
        // puts the hand exactly where the artist posed it on this weapon, which is elliotttate's
        // approach (main.cpp:2599-2624) and why it survives reload and fire animations.
        //
        // FRAMES: every term is palette space (camera-local Blam axes). NO lock-gap lift belongs
        // here -- the gap converts STAGE-relative quantities into camera space, and both the stock
        // wrist and the weapon delta are born camera-local. Adding one would be the double-count
        // in reverse.
        //
        // Weighted by the hold's own ramp, so the hand travels in step with the aim bend.
        //
        // NOT WHILE THE HAND IS FROZEN FOR CALIBRATION. The grab is the one thing downstream of the
        // freeze that can still move this hand, and it would move it exactly where the gesture is
        // trying to hold it still -- the player would align against a hand being dragged onto the
        // gun by their OTHER hand's aim, and the capture would record that drag as their offset.
        // Refusing here (rather than refusing the whole capture when the hold is engaged) keeps the
        // gesture usable two-handed: the hold stays latched, the hand simply stops riding it for
        // the duration.
        // GATED OFF BY DEFAULT -- see Config.hpp pa_grab_weapon.
        //
        // pancreations MCC VR never attach the support hand to the weapon: it is solved onto its own
        // controller, and the only coupling between the hands runs one-way through the aim basis.
        // That is the cleaner dependency, and it matches the report that grabbing behaves as a
        // separate concern from the arm itself.
        if (g_cfg.pa_grab_weapon != 0 && !plan.is_aim && wpn_delta_valid &&
            !s_hfreeze_active.load(std::memory_order_acquire)) {
            const float w = ::halo::two_hand_blend_weight();
            if (w > 0.0f) {
                const pa::Vec3 stock_pos   = access.palette[plan.arm->wrist].position;
                const pa::Mat3 stock_basis = pa::orthonormal_basis(access.palette[plan.arm->wrist]);
                const pa::Vec3 on_gun_pos =
                    wpn_delta_pos + pa::transform_vector(wpn_delta_basis, stock_pos);
                const pa::Mat3 on_gun_basis = pa::multiply(wpn_delta_basis, stock_basis);
                if (pa::valid_basis(on_gun_basis)) {
                    wrist_target  = wrist_target + (on_gun_pos - wrist_target) * w;
                    desired_wrist = pa::blend_basis(desired_wrist, on_gun_basis, w);
                }
            }
        }

        if (!pa::solve_arm_for_tracked_wrist(access.palette, *plan.arm, wrist_target,
                                             desired_wrist, torso_basis.up, s_arm_tuning)) {
            HALO_VR_DEV_ONLY(if (!plan.is_aim) ++s_bail[5];);
            continue;
        }
        any_posed = true;
        HALO_VR_DEV_ONLY(if (!plan.is_aim) ++s_posed;);

        if (!plan.is_aim && !access.is_capture_bank && map->weapon_marker != pa::kNoNode) {
            const auto& wm = access.palette[map->weapon_marker];
            const pa::Mat3 wb = pa::orthonormal_basis(wm);
            if (pa::valid_basis(wb)) {
                const pa::Vec3 gotw = access.palette[plan.arm->wrist].position;
                const pa::Vec3 wrel{gotw.x - wm.position.x, gotw.y - wm.position.y,
                                   gotw.z - wm.position.z};
                const pa::Vec3 hg = pa::transform_vector(pa::transpose(wb), wrel);
                s_dbg_handgun_x = hg.x; s_dbg_handgun_y = hg.y; s_dbg_handgun_z = hg.z;
            }
            s_dbg_blend = ::halo::two_hand_blend_weight();
            {
                const pa::Mat3 tinv = pa::transpose(torso_basis);
                const pa::Vec3 shf = access.palette[plan.arm->shoulder].position;
                const pa::Vec3 wf  = access.palette[plan.arm->wrist].position;
                const pa::Vec3 ef  = access.palette[plan.arm->elbow].position;
                const pa::Vec3 gw = pa::transform_vector(tinv,
                    pa::Vec3{wf.x - shf.x, wf.y - shf.y, wf.z - shf.z});
                const pa::Vec3 ge = pa::transform_vector(tinv,
                    pa::Vec3{ef.x - shf.x, ef.y - shf.y, ef.z - shf.z});
                s_dbg_gotL_x = gw.x; s_dbg_gotL_y = gw.y; s_dbg_gotL_z = gw.z;
                s_dbg_elbL_x = ge.x; s_dbg_elbL_y = ge.y; s_dbg_elbL_z = ge.z;
                s_dbg_missL = pa::length(wf - wrist_target) * pa::kMetresPerBlamUnit * 100.0f;
#if HALO_VR_DEV
                s_j_tgt.feed(wrist_target);
                s_j_wrist.feed(wf);
                s_j_elbow.feed(ef);
                s_j_ctrl.feed(plan.grip_position / pa::kMetresPerBlamUnit);
                s_j_gap.feed(::halo::g_view_lock_delta.load());
                s_j_view.feed(::halo::g_view_pitch.load());
                {
                    const pa::Vec3 S = shf, E = ef, W = wf;
                    const pa::Vec3 sw = W - S;
                    const float    swl = pa::length(sw);
                    if (swl > 1.0e-5f) {
                        const pa::Vec3 axis = sw / swl;
                        const pa::Vec3 se   = E - S;
                        const pa::Vec3 perp = se - axis * pa::dot(se, axis);
                        s_j_perp.feed(pa::length(perp) * pa::kMetresPerBlamUnit * 100.0f);

                        const pa::Vec3 a = S - E, b = W - E;
                        const float la = pa::length(a), lb = pa::length(b);
                        if (la > 1.0e-5f && lb > 1.0e-5f) {
                            float c = pa::dot(a, b) / (la * lb);
                            c = (c < -1.0f) ? -1.0f : ((c > 1.0f) ? 1.0f : c);
                            s_j_bend.feed(std::acos(c) * 57.29577951f);
                        }
                        // Azimuth of the elbow about the arm axis, referenced to the torso up axis
                        // (aim-free), so this is a shape measure and not a camera measure.
                        const pa::Vec3 ref = torso_basis.up - axis * pa::dot(torso_basis.up, axis);
                        if (pa::length_squared(ref) > 1.0e-8f && pa::length_squared(perp) > 1.0e-8f) {
                            const pa::Vec3 r = pa::normalized(ref);
                            const pa::Vec3 s2 = pa::normalized(perp);
                            const pa::Vec3 t2 = pa::cross(axis, r);
                            s_j_swivel.feed(std::atan2(pa::dot(s2, t2), pa::dot(s2, r)) * 57.29577951f);
                        }
                    }
                }
                {
                    // The drawn hand, position AND orientation, in the aim-free torso frame.
                    const pa::Mat3 tinv2 = pa::transpose(torso_basis);
                    {
                        const pa::Vec3& gp = plan.grip_position;
                        if (s_frz_have && gp.x == s_frz_x && gp.y == s_frz_y && gp.z == s_frz_z) {
                            if (++s_frz_run > s_frz_worst) s_frz_worst = s_frz_run;
                        } else { s_frz_run = 0; }
                        s_frz_x = gp.x; s_frz_y = gp.y; s_frz_z = gp.z; s_frz_have = true;
                    }
                    s_j_upper.feed(pa::length(ef - shf) * pa::kMetresPerBlamUnit * 100.0f);
                    s_j_lower.feed(pa::length(wf - ef) * pa::kMetresPerBlamUnit * 100.0f);
                    s_j_handpos.feed(pa::transform_vector(tinv2,
                        pa::Vec3{wf.x - shf.x, wf.y - shf.y, wf.z - shf.z}));
                    const pa::Mat3 wb2 = pa::orthonormal_basis(access.palette[plan.arm->wrist]);
                    if (pa::valid_basis(wb2)) s_j_handrot.feed(pa::multiply(tinv2, wb2));
                }
#endif
            }
        }

        if (plan.is_aim && !access.is_capture_bank) {
            const pa::Vec3 got = access.palette[plan.arm->wrist].position;
            s_dbg_fwd_x = root_basis.forward.x;
            s_dbg_fwd_y = root_basis.forward.y;
            s_dbg_fwd_z = root_basis.forward.z;
            s_dbg_root_x = root_position.x;
            s_dbg_root_y = root_position.y;
            s_dbg_root_z = root_position.z;
            s_dbg_tgt_x  = wrist_target.x;
            s_dbg_tgt_y  = wrist_target.y;
            s_dbg_tgt_z  = wrist_target.z;
            s_dbg_got_x  = got.x;
            s_dbg_got_y  = got.y;
            s_dbg_got_z  = got.z;
            s_dbg_miss_cm  = pa::length(got - wrist_target) * pa::kMetresPerBlamUnit * 100.0f;
            s_dbg_reach_cm = pa::length(wrist_target -
                                        access.palette[plan.arm->shoulder].position) *
                             pa::kMetresPerBlamUnit * 100.0f;
        }
    }

    if (!any_posed) {
        // Both conventions still settling, or both arms refused. Stock pose, and say so.
        s_drive_stage = "no arm posed (conventions settling?)";
        return false;
    }

    // ---- fingers LAST, deliberately. Wrist placement above is a rigid transform and nothing in
    // the aim path reads finger nodes, so opening a hand cannot perturb where the gun points.
    // Curl inputs are not wired yet -- the hands hold the authored grip until they are.
    const pa::HandCurl closed{};
    pa::apply_hand_openness(access.palette, aim_arm, closed);
    if (tracking.support_valid) pa::apply_hand_openness(access.palette, support_arm, closed);

    // ---- HANDS-ONLY, last of all. Everything above has already run, so this only decides what is
    // VISIBLE -- see Config.hpp pa_hands_only. Fails visible: a false return leaves the arms shown.
    HALO_VR_DEV_ONLY(
        if (!s_tag_have) { s_tag_have = true; s_tag_last = access.model_tag; s_tag_a = access.model_tag; }
        else if (access.model_tag != s_tag_last) { ++s_tag_switches; s_tag_b = access.model_tag;
                                                  s_tag_last = access.model_tag; });
    if (g_cfg.pa_hands_only != 0) {
        const bool hid = pa::collapse_to_hands(access.palette, access.node_count,
                                               map->left, map->right,
                                               map->weapon_nodes, map->weapon_count);
        HALO_VR_DEV_ONLY(if (hid) ++s_hands_applied;);
    }

    s_drive_stage = "DRIVING";
    s_drive_ok.fetch_add(1, std::memory_order_relaxed);
    return true;
}

// ---- POSE CAPTURE, on the tick -----------------------------------------------------------------

void capture_tracking() {
    TrackingSnapshot snap{};

    const auto hmd = API::VR::get_hmd_index();
    ::halo::Vec3 p{}; ::halo::Quat q{};
    if (hmd < 0 || !get_pose(hmd, &p, &q, /*use_aim=*/false)) {
        s_tracking_ready.store(false, std::memory_order_release);
        return;
    }
    snap.hmd_position = from_math(p);
    snap.hmd_rotation = from_math(q);

    // STAGE-ANCHORED COMPOSITION. The palette root is the stick-driven game camera with no HMD
    // content in it, and UEVR renders the view as game-camera * (recenter-offset * hmd). The
    // recenter offset is therefore the exact map from stage space onto that root. Composing with
    // the inverse HMD instead subtracts a head rotation the root never contained, which
    // counter-rotates the weapon and hands against every head turn.
    {
        const auto ro = API::VR::get_rotation_offset();
        snap.stage_rotation = pa::Quat{ro.x, ro.y, ro.z, ro.w};
    }

    // Grip pose for POSITION, aim pose for DIRECTION. Plugin.cpp documents why they are not
    // interchangeable here: get_aim_pose() produces teleport-scale translation readings, so its
    // rotation is usable and its position is not.
    // Runtime device indices, not the literals 0/1 -- index 0 is the HMD in UEVR. Same fix as
    // TwoHandAim.cpp; the literals were the folder's original and had never run.
    const int32_t aim_idx     = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                                    : API::VR::get_right_controller_index();
    const int32_t support_idx = g_cfg.aim_left_hand ? API::VR::get_right_controller_index()
                                                    : API::VR::get_left_controller_index();
    if (aim_idx < 0) { s_tracking_ready.store(false, std::memory_order_release); return; }

    if (!get_pose(aim_idx, &p, &q, /*use_aim=*/false)) {
        s_tracking_ready.store(false, std::memory_order_release);
        return;
    }
    snap.aim_grip_position = from_math(p);
    snap.aim_grip_rotation = from_math(q);
    if (get_pose(aim_idx, &p, &q, /*use_aim=*/true)) snap.aim_aim_rotation = from_math(q);
    else                                             snap.aim_aim_rotation = snap.aim_grip_rotation;

    if (get_pose(support_idx, &p, &q, /*use_aim=*/false)) {
        snap.support_grip_position = from_math(p);
        snap.support_grip_rotation = from_math(q);
        snap.support_valid = true;
    }

    snap.valid = true;
    s_tracking = snap;
    s_tracking_ready.store(true, std::memory_order_release);
}

// ---- THE PER-WEAPON FIX: resolve it, and run the freeze-and-align capture. GAME THREAD ----------
//
// NOT BEHIND HALO_VR_DEV, deliberately. This is a player-facing calibration, the same family as the
// End and Page Down gestures, and it logs once per gesture rather than per frame -- it answers "put
// this weapon where I am holding it", not "what is the plugin doing". The per-tick cost is a pointer
// read and a substring compare; nothing here sweeps anything.
//
// THE GESTURE. Hold the per-weapon key (wpncalibkey, INSERT by default): the weapon stops where it
// is. Move your controller so it sits where the frozen grip is, and release. The difference between
// where your hand WAS and where it IS now is the rigid delta this weapon needed, and it is composed
// onto whatever that weapon already had -- so repeat captures REFINE rather than reset, exactly as
// the existing calibrations do.
//
// WHY THE SOLVE IS EXACT. Both the frozen and the live pose are the stage-frame pair the drive
// publishes, so the answer is
//
//     d_rot = live_basis^T * frozen_basis          (a rotation in the live pose's own frame)
//     d_pos = live_basis^T * (frozen_off - live_off)
//
// with no camera, no root and no lift anywhere in it. Nothing the aim did during the hold can reach
// it. Composed onto the stored entry by blindcowboy24's rule -- W' = W * d_rot, T' = T + W * d_pos
// -- which is the pairing that makes "apply" and "solve" inverses of each other rather than two
// nearly-agreeing formulas.
//
// ONE FRAME STALE, AND THAT IS FINE. The drive runs from the game's own builder later in the frame,
// so the carry read at an edge is the previous frame's -- about 11 ms, a millimetre or two of hand
// travel. The capture already has a 20-frame minimum for a much larger reason (a stab at the key is
// not a deliberate alignment), so a frame of latency is well inside the gesture's own noise.
void weapon_fix_tick() {
    // ---- WHICH ENTRY APPLIES. weapon_offset_current_class() is published each tick by machinery
    // that already paid for the reflection (WeaponOffset.hpp), so this costs a pointer read.
    //
    // CONSEQUENCE WORTH KNOWING: that publisher returns early when wpnoffsets=0, so the per-weapon
    // fix goes quiet in that configuration too. wpnoffsets ships ON and is a no-op with an empty
    // table, so this is a documented coupling rather than a hazard -- but it is why turning that key
    // off appears to disable a feature that has nothing to do with it.
    const char* cls = ::halo::weapon_offset_current_class();
    const ::halo::WeaponFix* fix = g_cfg.pa_weapon ? ::halo::weapon_fix_for(cls) : nullptr;
    if (fix != nullptr) {
        const pa::Mat3 m = pa::rotation_basis(
            pa::normalized(pa::Quat{fix->q[0], fix->q[1], fix->q[2], fix->q[3]}));
        // A stored quaternion that is not a rotation is dropped rather than applied. normalized()
        // already turns a zero or non-finite quaternion into identity, so what this catches is a
        // hand-edited line -- and identity here means "this weapon has no fix", which is exactly
        // the right answer for one.
        if (pa::valid_basis(m)) {
            s_wfix_rot = m;
            s_wfix_pos = pa::Vec3{fix->t[0], fix->t[1], fix->t[2]} / pa::kMetresPerBlamUnit;
            s_wfix_have.store(true, std::memory_order_release);
        } else {
            s_wfix_have.store(false, std::memory_order_release);
        }
    } else {
        s_wfix_have.store(false, std::memory_order_release);
    }

    // The rig path's release latch, drained every tick rather than only at our own falling edge.
    // ORDER-INDEPENDENT ON PURPOSE: Plugin.cpp polls the key in a different callback, so whether
    // its falling edge lands before or after this function within a tick is not something either
    // side should have to know. Draining it here means the latch can never survive to be claimed
    // by an unrelated End solve later, which is the failure the latch exists to prevent.
    //
    // GATED ON OWNERSHIP, and the gate is not decoration. This function runs whenever the palette
    // ARM driver is selected, which is NOT the same condition as the palette owning the WEAPON --
    // armdriver=2 with pawpn=0 is arms-here, gun-on-the-rig. Draining unconditionally there would
    // eat the latch belonging to the rig's own capture, and INSERT would silently stop working for
    // the exact configuration that still needs it. Same predicate WeaponCalib.cpp stands down on.
    if (palettearm_weapon_owns()) (void)::halo::wpn_calib_take_pending();

    static bool     s_held = false;
    static bool     s_armed = false;
    static uint32_t s_frames = 0;
    static float    s_gap0 = 0.0f;

    // FOCUS IS PART OF THE HOLD, and leaving it out latches the freeze forever.
    //
    // Plugin.cpp polls wpn_calib_key only while the game window has focus, so alt-tabbing away
    // mid-hold stops the poll with the key state stuck DOWN -- our hold would never end, and the
    // weapon would hang in mid-air with no way back short of a restart. Reading focus here makes
    // that a falling edge, and the edge handler below treats an unfocused one as an ABORT rather
    // than a release: the hand is not where the player left it, so there is nothing honest to solve.
    // palettearm_weapon_owns(), not g_cfg.pa_weapon: it is the SAME predicate WeaponCalib.cpp stands
    // down on, so exactly one of the two capture paths can ever claim a press. Two nearly-identical
    // conditions in two files is how a press ends up writing both destinations, or neither.
    const bool focused = ::halo::game_window_focused();
    const bool held = palettearm_weapon_owns() && focused && ::halo::wpn_calib_held();
    const bool was  = s_held;
    s_held = held;

    if (held && !was) {
        // RISING EDGE. Refuse to freeze on a carry we have never seen produced: latching an
        // uninitialised basis would pin the weapon to garbage and then solve against it.
        if (!s_wcarry_valid.load(std::memory_order_acquire) || !pa::valid_basis(s_wcarry.basis)) {
            s_armed = false;
            API::get()->log_info("[Halo-CampE-UEVR] WPNFIX: nothing to freeze -- the palette weapon "
                                 "carry has not run yet (no weapon in hand, or the drive is not "
                                 "posing it). Capture ignored.");
        } else {
            s_wfreeze = s_wcarry;
            s_wfreeze_active.store(true, std::memory_order_release);
            s_armed  = true;
            s_frames = 0;
            s_gap0   = ::halo::g_view_lock_delta.load();
            API::get()->log_info("[Halo-CampE-UEVR] WPNFIX: weapon FROZEN -- put your controller "
                                 "where the grip is, then release.");
        }
    }
    if (held) { if (s_frames < 0xFFFFFFFFu) ++s_frames; return; }
    if (!was) return;

    // ---- FALLING EDGE. Release the freeze FIRST, whatever happens below: a solve that refuses
    // must still give the weapon back to the hand, or the gun stays pinned in mid-air forever.
    s_wfreeze_active.store(false, std::memory_order_release);
    if (!s_armed) return;
    s_armed = false;
    if (!focused) {
        API::get()->log_info("[Halo-CampE-UEVR] WPNFIX: the game window lost focus during the hold "
                             "-- capture abandoned and the weapon released. Nothing captured.");
        return;
    }

    // 20 frames ~ a fifth of a second. A stab at the key is not a deliberate alignment, and a
    // capture taken from one is a weapon nudged somewhere nobody chose.
    constexpr uint32_t kMinFreezeFrames = 20;
    if (s_frames < kMinFreezeFrames) {
        API::get()->log_info("[Halo-CampE-UEVR] WPNFIX: released after %u frames -- too short to be "
                             "a deliberate match. Nothing captured.", s_frames);
        return;
    }
    if (!s_wcarry_valid.load(std::memory_order_acquire) || !pa::valid_basis(s_wcarry.basis)) {
        API::get()->log_info("[Halo-CampE-UEVR] WPNFIX: the weapon carry stopped during the hold "
                             "(weapon swapped, or tracking dropped). Nothing captured.");
        return;
    }

    const pa::Mat3 live_inv = pa::transpose(s_wcarry.basis);
    const pa::Mat3 d_rot    = pa::multiply(live_inv, s_wfreeze.basis);
    const pa::Vec3 d_pos_m  =
        pa::transform_vector(live_inv, s_wfreeze.offset - s_wcarry.offset) * pa::kMetresPerBlamUnit;

    const float dm = pa::length(d_pos_m);
    // Rotation angle from the trace: tr = 1 + 2cos(theta).
    const float tr = d_rot.forward.x + d_rot.left.y + d_rot.up.z;
    float cosang = (tr - 1.0f) * 0.5f;
    cosang = cosang < -1.0f ? -1.0f : (cosang > 1.0f ? 1.0f : cosang);
    const float ang = std::acos(cosang) * 57.2957795f;

    // THE PLAYER'S FIRST CAPTURE FOR A WEAPON IS ALLOWED A LARGE ROTATION, later ones are not.
    //
    // blindcowboy24 measured why and it is worth keeping: the first thing a capture absorbs is any
    // fixed convention mismatch between where the gun renders and where the maths thinks the hand
    // is, and on his build every clean, deliberate capture solved to ~100 degrees of yaw with the
    // controller placed ON the rendered gun. A gate tight enough to call that a dropout refuses the
    // one measurement that would name it. Once the player HAS a capture the bound tightens, so a
    // genuine tracking dropout can no longer corrupt a calibration they worked for.
    //
    // "FIRST" MEANS THEY HAVE NO CAPTURE OF THEIR OWN -- not that the table is empty. A SHIPPED
    // baseline entry does not remove that convention mismatch for this player: it was solved on
    // somebody else's controllers and grip. Testing the merged table instead would hand a tight
    // 60-degree bound to the very first capture of any weapon we ship a value for, and reject it.
    const ::halo::WeaponFix* prior = ::halo::weapon_fix_for(cls);
    const bool  first   = (prior == nullptr) || !prior->captured;
    const float ang_max = first ? 175.0f : 60.0f;
    if (!std::isfinite(dm) || dm > 1.0f || !std::isfinite(ang) || ang > ang_max) {
        API::get()->log_info("[Halo-CampE-UEVR] WPNFIX: REJECTED (moved %.2f m, rotated %.0f deg, "
                             "limit %.0f) -- tracking dropped, or the weapon changed mid-hold. "
                             "Nothing captured.", dm, ang, ang_max);
        return;
    }

    const std::string key = ::halo::weapon_key();
    if (key.empty()) {
        API::get()->log_info("[Halo-CampE-UEVR] WPNFIX: no weapon in hand -- nothing captured.");
        return;
    }

    // COMPOSE ONTO THE EFFECTIVE DELTA, so repeat captures refine rather than reset.
    //
    // `prior` is whatever weapon_fix_for() resolved -- which may be the SHIPPED baseline rather
    // than one of the player's own, and that is the correct base to compose onto: the player was
    // aligning against the gun AS RENDERED, and the shipped delta was already in that render.
    // Composing onto identity instead would silently discard the shipped fit at the moment they
    // tried to refine it, and the weapon would jump by exactly that fit on release.
    //
    // The result is then stored as a CAPTURED entry, which outranks the shipped one it was built
    // from -- start from ours, end up with theirs, and deleting their file returns to ours.
    pa::Mat3 old_rot{};
    pa::Vec3 old_pos{};
    if (prior != nullptr) {
        const pa::Mat3 m = pa::rotation_basis(
            pa::normalized(pa::Quat{prior->q[0], prior->q[1], prior->q[2], prior->q[3]}));
        if (pa::valid_basis(m)) old_rot = m;
        old_pos = pa::Vec3{prior->t[0], prior->t[1], prior->t[2]};
    }
    const pa::Mat3 new_rot = pa::multiply(old_rot, d_rot);
    const pa::Vec3 new_pos = old_pos + pa::transform_vector(old_rot, d_pos_m);
    if (!pa::valid_basis(new_rot) || !pa::finite(new_pos)) {
        API::get()->log_info("[Halo-CampE-UEVR] WPNFIX: the composed result is not a rotation -- "
                             "'%s' left unchanged.", key.c_str());
        return;
    }

    const pa::Quat qn = pa::rotation_from_basis(new_rot);
    const float q[4] = {qn.x, qn.y, qn.z, qn.w};
    const float t[3] = {new_pos.x, new_pos.y, new_pos.z};
    ::halo::wpnfix_set(key, q, t);

    // gapdrift IS THE DIAGNOSTIC, not decoration. The frozen weapon holds still in the world only
    // to the extent pa_wpn_lift is right, and the lift only does any work when the lock gap MOVES.
    // A capture taken across a large gap change, that then lands wrong, points straight at the lift
    // rather than at the gesture -- and that is otherwise very hard to tell apart from a bad
    // alignment. Reported every time so the number exists before anyone needs it.
    API::get()->log_info("[Halo-CampE-UEVR] WPNFIX '%s': this match moved %.1f cm, rotated %.0f deg "
                         "-> total pos=(%.1f %.1f %.1f)cm  gapdrift=%.1f deg  [halo_vr_weapons.cfg]",
                         key.c_str(), dm * 100.0f, ang,
                         new_pos.x * 100.0f, new_pos.y * 100.0f, new_pos.z * 100.0f,
                         ::halo::g_view_lock_delta.load() - s_gap0);
}

// ---- THE SUPPORT-HAND FIX: resolve it, and run the freeze-and-align capture. GAME THREAD --------
//
// THE GESTURE, and it is the arm-rig calibration's gesture with a different subject. Arm it from the
// in-game menu ("Calibrate support hand"): the support hand stops where it is. Put your real hand
// where the frozen one is, then RIGHT trigger to save and finish, or LEFT trigger to save and stay
// armed for another pass -- the same two controls, with the same meanings, as the pose-match and
// aim-ray gestures. NO HOTKEY, by request: this one is menu-armed only.
//
// WHAT IT SOLVES FOR. The fixed offset between where your runtime reports your controller's grip
// pose and where your hand actually is inside it. That is why there is nothing shipped to calibrate
// against and nothing per-weapon about it -- it is a property of a person holding a controller, and
// one capture is good for every weapon and every session.
//
// THE SOLVE IS THE WEAPON FIX'S, unchanged:
//
//     d_rot = live_basis^T * frozen_basis
//     d_pos = live_basis^T * (frozen_off - live_off)
//
// Both sides are the stage-frame pair the drive publishes, so no camera, root or lift appears in it
// and nothing the aim hand did during the hold can reach it. Composed onto the stored entry the
// same way -- W' = W * d_rot, T' = T + W * d_pos -- which is the pairing that makes apply and solve
// exact inverses rather than two formulas that nearly agree. Repeat captures therefore REFINE.
void hand_fix_tick() {
    // ---- RESOLVE THE STORED VALUE into the form the drive consumes (Blam units, a basis).
    // A stored quaternion that is not a rotation is DROPPED rather than applied; normalized()
    // already turns a zero or non-finite quaternion into identity, so what this catches is a
    // hand-edited line -- and identity means "no trim", which is the right answer for one.
    bool have = false;
    if (g_cfg.hand_fix_valid) {
        const pa::Mat3 m = pa::rotation_basis(pa::normalized(
            pa::Quat{g_cfg.hand_fix_q[0], g_cfg.hand_fix_q[1],
                     g_cfg.hand_fix_q[2], g_cfg.hand_fix_q[3]}));
        if (pa::valid_basis(m)) {
            s_hfix_rot = m;
            s_hfix_pos = pa::Vec3{g_cfg.hand_fix_t[0], g_cfg.hand_fix_t[1], g_cfg.hand_fix_t[2]} /
                         pa::kMetresPerBlamUnit;
            have = true;
        }
    }
    s_hfix_have.store(have, std::memory_order_release);

    static bool     s_h_held   = false;
    static bool     s_h_armed  = false;
    static uint32_t s_h_frames = 0;
    static float    s_h_gap0   = 0.0f;

    // THE HOLD, GATED ON THE MODE. g_hand_calib_held carries only the left-trigger half; the mode is
    // the authoritative one, cleared by the right trigger and by the menu's Cancel. Reading both is
    // what stops a publisher that stops running from leaving the hand pinned in mid-air -- see the
    // declaration in Config.hpp.
    //
    // NO FOCUS TERM, unlike the weapon fix. That one exists because Plugin.cpp polls its KEYBOARD
    // key only while the game is foregrounded, so alt-tabbing mid-hold strands the key state down.
    // This gesture has no key: its input is the pad, through the game's own XInput, which does not
    // arrive at all when the game is not the foreground window. There is nothing to strand.
    const bool held = (::halo::g_menu_calib_mode.load(std::memory_order_relaxed) == 3) &&
                      ::halo::g_hand_calib_held.load(std::memory_order_relaxed);
    const bool was  = s_h_held;
    s_h_held = held;

    if (held && !was) {
        // RISING EDGE. Refuse to freeze a pose the drive has never produced -- latching an
        // uninitialised basis would pin the hand to garbage and then solve against it.
        if ((g_cfg.pa_arms & 2) == 0) {
            s_h_armed = false;
            API::get()->log_info("[Halo-CampE-UEVR] HANDFIX: the support arm is not being driven "
                                 "(paarms has no support bit) -- there is no hand to freeze. "
                                 "Capture ignored.");
        } else if (!s_hcarry_valid.load(std::memory_order_acquire) ||
                   !pa::valid_basis(s_hcarry.basis)) {
            s_h_armed = false;
            API::get()->log_info("[Halo-CampE-UEVR] HANDFIX: nothing to freeze -- the support hand "
                                 "has not been posed yet (no controller tracking, or the drive has "
                                 "not run). Capture ignored.");
        } else if (!s_tracking_ready.load(std::memory_order_acquire) ||
                   !s_tracking.support_valid) {
            // s_hcarry_valid is set by the drive and never CLEARED by it -- the same shape as
            // s_wcarry_valid -- so it survives a tracking dropout and would happily hand us a pose
            // from before the controller went to sleep. Freezing that pins the hand somewhere the
            // player never put it, and the alignment they then make is against a fiction. The live
            // snapshot is the only thing that can tell the difference, and it is a field read.
            s_h_armed = false;
            API::get()->log_info("[Halo-CampE-UEVR] HANDFIX: the support controller is not tracking "
                                 "right now -- the last pose we have is stale. Capture ignored.");
        } else {
            s_hfreeze = s_hcarry;
            s_hfreeze_active.store(true, std::memory_order_release);
            s_h_armed  = true;
            s_h_frames = 0;
            s_h_gap0   = ::halo::g_view_lock_delta.load();
            API::get()->log_info("[Halo-CampE-UEVR] HANDFIX: support hand FROZEN -- put your real "
                                 "hand where it is, then RIGHT trigger = save & finish, LEFT "
                                 "trigger = save & go again.");
        }
    }
    if (held) { if (s_h_frames < 0xFFFFFFFFu) ++s_h_frames; return; }
    if (!was) return;

    // ---- FALLING EDGE. Release the freeze FIRST, whatever happens below: a solve that refuses
    // must still give the hand back, or it stays pinned in mid-air forever.
    s_hfreeze_active.store(false, std::memory_order_release);
    if (!s_h_armed) return;
    s_h_armed = false;

    // 20 frames ~ a fifth of a second, the weapon fix's number and for its reason: a stab at the
    // trigger is not a deliberate alignment, and a capture taken from one moves the hand somewhere
    // nobody chose.
    constexpr uint32_t kMinFreezeFrames = 20;
    if (s_h_frames < kMinFreezeFrames) {
        API::get()->log_info("[Halo-CampE-UEVR] HANDFIX: released after %u frames -- too short to "
                             "be a deliberate match. Nothing captured.", s_h_frames);
        return;
    }
    if (!s_hcarry_valid.load(std::memory_order_acquire) || !pa::valid_basis(s_hcarry.basis)) {
        API::get()->log_info("[Halo-CampE-UEVR] HANDFIX: the support hand stopped being posed "
                             "during the hold (tracking dropped). Nothing captured.");
        return;
    }

    const pa::Mat3 live_inv = pa::transpose(s_hcarry.basis);
    const pa::Mat3 d_rot    = pa::multiply(live_inv, s_hfreeze.basis);
    const pa::Vec3 d_pos_m  =
        pa::transform_vector(live_inv, s_hfreeze.offset - s_hcarry.offset) * pa::kMetresPerBlamUnit;

    const float dm = pa::length(d_pos_m);
    const float tr = d_rot.forward.x + d_rot.left.y + d_rot.up.z;   // tr = 1 + 2cos(theta)
    float cosang = (tr - 1.0f) * 0.5f;
    cosang = cosang < -1.0f ? -1.0f : (cosang > 1.0f ? 1.0f : cosang);
    const float ang = std::acos(cosang) * 57.2957795f;

    // SANITY BOUNDS ON ONE GESTURE'S WORTH OF MOVEMENT, not on the accumulated total -- the file
    // value is deliberately unclamped for exactly that reason (Config.cpp, parse_hand_fix).
    //
    // TIGHTER THAN THE WEAPON'S FIRST CAPTURE, and the asymmetry is real rather than an oversight.
    // The weapon's first capture has a fixed convention mismatch to absorb and legitimately solves
    // to ~100 degrees; this one has none -- the support hand is already placed straight from the
    // controller, so the whole quantity being measured is a human hand's offset inside a human
    // grip. Anything past 45 degrees or 30 cm of that is a tracking dropout, not an alignment.
    if (!std::isfinite(dm) || dm > 0.30f || !std::isfinite(ang) || ang > 45.0f) {
        API::get()->log_info("[Halo-CampE-UEVR] HANDFIX: REJECTED (moved %.2f m, rotated %.0f deg; "
                             "limits 0.30 m / 45 deg) -- tracking dropped, or your hand was not "
                             "where the frozen one was. Nothing captured.", dm, ang);
        return;
    }

    // COMPOSE ONTO THE STORED DELTA, so repeat captures refine rather than reset. The player was
    // aligning against the hand AS RENDERED, and that render already had the stored trim in it --
    // composing onto identity would discard it at the moment they tried to improve it, and the hand
    // would jump by exactly that trim on release.
    pa::Mat3 old_rot{};
    pa::Vec3 old_pos{};
    if (g_cfg.hand_fix_valid) {
        const pa::Mat3 m = pa::rotation_basis(pa::normalized(
            pa::Quat{g_cfg.hand_fix_q[0], g_cfg.hand_fix_q[1],
                     g_cfg.hand_fix_q[2], g_cfg.hand_fix_q[3]}));
        if (pa::valid_basis(m)) old_rot = m;
        old_pos = pa::Vec3{g_cfg.hand_fix_t[0], g_cfg.hand_fix_t[1], g_cfg.hand_fix_t[2]};
    }
    const pa::Mat3 new_rot = pa::multiply(old_rot, d_rot);
    const pa::Vec3 new_pos = old_pos + pa::transform_vector(old_rot, d_pos_m);
    if (!pa::valid_basis(new_rot) || !pa::finite(new_pos)) {
        API::get()->log_info("[Halo-CampE-UEVR] HANDFIX: the composed result is not a rotation -- "
                             "the stored support-hand fix is left unchanged.");
        return;
    }

    const pa::Quat qn = pa::rotation_from_basis(new_rot);
    g_cfg.hand_fix_q[0] = qn.x; g_cfg.hand_fix_q[1] = qn.y;
    g_cfg.hand_fix_q[2] = qn.z; g_cfg.hand_fix_q[3] = qn.w;
    g_cfg.hand_fix_t[0] = new_pos.x;
    g_cfg.hand_fix_t[1] = new_pos.y;
    g_cfg.hand_fix_t[2] = new_pos.z;
    g_cfg.hand_fix_valid = true;
    ::halo::write_calib_file();

    // gapdrift, for the reason the weapon fix records it: the frozen pose only holds still in the
    // room to the extent the lock-gap lift is right, and the lift only does any work when the gap
    // MOVES -- which here means the player's AIM hand moved while they were aligning their support
    // hand. A capture taken across a large gap change that then lands wrong points at the lift
    // rather than at the gesture, and the two are otherwise very hard to tell apart. (Yaw only:
    // paarmpitch's half has no equivalent number yet.)
    API::get()->log_info("[Halo-CampE-UEVR] HANDFIX: this match moved %.1f cm, rotated %.0f deg "
                         "-> total pos=(%.1f %.1f %.1f)cm  gapdrift=%.1f deg  [halo_vr_calib.cfg]",
                         dm * 100.0f, ang,
                         new_pos.x * 100.0f, new_pos.y * 100.0f, new_pos.z * 100.0f,
                         ::halo::g_view_lock_delta.load() - s_h_gap0);
}

} // namespace

bool palettearm_parse_key(const char* key, double v) {
    if      (_stricmp(key, "pashoulderback")  == 0) s_arm_tuning.shoulder_back_m      = (float)v;
    else if (_stricmp(key, "pashoulderdown")  == 0) s_arm_tuning.shoulder_down_m      = (float)v;
    else if (_stricmp(key, "pashoulderlat")   == 0) s_arm_tuning.shoulder_lateral_m   = (float)v;
    else if (_stricmp(key, "paclavicle")      == 0) s_arm_tuning.clavicle_assist_m    = (float)v;
    else if (_stricmp(key, "pawristback")     == 0) s_arm_tuning.grip_to_wrist_back_m = (float)v;
    else if (_stricmp(key, "pawristdown")     == 0) s_arm_tuning.grip_to_wrist_down_m = (float)v;
    else return false;
    return true;
}

const char* palettearm_status() { return s_status; }
const char* palettearm_status_geom() { return s_status_geom; }

// Formats AND resets, so each emitted line is one clean window rather than a running total.
const char* palettearm_status_jitter() {
#if HALO_VR_DEV
    std::snprintf(s_status_jitter, sizeof(s_status_jitter),
        "palettearm jitter n=%u (mean/max): ctrl %.2f/%.2f  tgt %.2f/%.2f  wrist %.2f/%.2f  "
        "elbow %.2f/%.2f cm | viewpitch %.3f/%.3f  lockdelta %.3f/%.3f deg",
        s_j_wrist.n,
        s_j_ctrl.mean_cm(),  s_j_ctrl.peak_cm(),
        s_j_tgt.mean_cm(),   s_j_tgt.peak_cm(),
        s_j_wrist.mean_cm(), s_j_wrist.peak_cm(),
        s_j_elbow.mean_cm(), s_j_elbow.peak_cm(),
        s_j_view.mean(), s_j_view.peak, s_j_gap.mean(), s_j_gap.peak);
    const std::size_t used = std::strlen(s_status_jitter);
    std::snprintf(s_status_jitter + used, sizeof(s_status_jitter) - used,
        " | posed %u bail[track %u conv %u solve %u anchor %u] hands %d/%u"
        " | SHAPE bend %.2f/%.2f deg  perp %.3f/%.3f cm  swivel %.2f/%.2f deg"
        " | HAND pos %.3f/%.3f cm  rot %.3f/%.3f deg"
        " | BONE up %.3f/%.3f lo %.3f/%.3f cm | tag sw=%u a=%d b=%d | frozen run=%u",
        s_posed, s_bail[1], s_bail[3], s_bail[5], s_bail[6], g_cfg.pa_hands_only, s_hands_applied,
        s_j_bend.mean(), s_j_bend.peak, s_j_perp.mean(), s_j_perp.peak,
        s_j_swivel.mean(), s_j_swivel.peak,
        s_j_handpos.mean_cm(), s_j_handpos.peak_cm(), s_j_handrot.mean(), s_j_handrot.peak,
        s_j_upper.mean(), s_j_upper.peak, s_j_lower.mean(), s_j_lower.peak,
        s_tag_switches, s_tag_a, s_tag_b, s_frz_worst);
    s_frz_worst = 0;
    s_j_bend.reset(); s_j_perp.reset(); s_j_swivel.reset(); s_j_handpos.reset(); s_j_handrot.reset(); s_j_upper.reset(); s_j_lower.reset(); s_tag_switches = 0;
    s_posed = 0;
    s_hands_applied = 0;
    for (unsigned i = 0; i < 7; ++i) s_bail[i] = 0;
    s_j_ctrl.reset(); s_j_tgt.reset(); s_j_wrist.reset(); s_j_elbow.reset();
    s_j_view.reset(); s_j_gap.reset();
    return s_status_jitter;
#else
    return "";
#endif
}

bool palettearm_weapon_owns() {
    return g_cfg.pa_weapon && g_cfg.arm_driver == 2 && !palettearm_unavailable();
}

bool palettearm_unavailable() { return s_unavailable; }

// 0 / 1 / 2 -- see the declaration in PaletteArm.hpp for what each one means and why there are two
// positive states rather than one.
//
// This exists because the answer is usually NO for reasons a player cannot see, and an armed
// gesture that silently does nothing is the failure this codebase keeps paying for ("installed is
// not running"). The shipped arm driver is the UE route, so on a default profile hand_fix_tick() is
// never even called -- and the menu would sit there showing ARMED at a plugin that will never look.
//
// s_hcarry_valid is what separates 1 from 2: it is only ever set by a drive pass that produced a
// valid support-hand pose, so it answers "is there a hand to freeze" rather than "is the feature
// configured". Cleared on release, so it cannot outlive the driver that set it.
int palettearm_support_hand_ready() {
    if (g_cfg.arm_driver != 2 || (g_cfg.pa_arms & 2) == 0 || s_unavailable) return 0;
    // BOTH terms, and the tracking one is not redundant: s_hcarry_valid is never cleared by the
    // drive, so on its own it would keep reporting "ready" straight through a controller dropout.
    // The same pair gates the rising edge in hand_fix_tick(), so the menu and the gesture agree.
    const bool tracked = s_tracking_ready.load(std::memory_order_acquire) &&
                         s_tracking.support_valid;
    return (tracked && s_hcarry_valid.load(std::memory_order_acquire)) ? 2 : 1;
}

void palettearm_retry() {
    if (!s_unavailable) return;
    s_unavailable = false;
    API::get()->log_info("[Halo-CampE-UEVR] PALETTEARM: re-armed by an armdriver change -- "
                         "will try to resolve the first-person weapon builder again");
}

void palettearm_release() {
    pa::palettehook_uninstall();
    s_tracking_ready.store(false, std::memory_order_release);
    // DROP THE FREEZE. Walking away with it latched would hand the weapon back to the other driver
    // pinned to a stage pose from a session state that no longer exists, and nothing downstream
    // would ever clear it: weapon_fix_tick() only runs while we are the owner.
    s_wfreeze_active.store(false, std::memory_order_release);
    s_wcarry_valid.store(false, std::memory_order_release);
    s_wfix_have.store(false, std::memory_order_release);
    // Same for the support-hand freeze, and for the same reason: hand_fix_tick() only runs while we
    // own the arms, so a freeze left latched here has no one to clear it.
    s_hfreeze_active.store(false, std::memory_order_release);
    s_hcarry_valid.store(false, std::memory_order_release);
    s_hfix_have.store(false, std::memory_order_release);
    // The map belonged to a skeleton in a process state we are walking away from. Re-derive on the
    // way back in rather than trusting a cache across a release.
    s_node_map.reset();
    s_map_reporter  = addrcascade::TierReporter{};
    s_map_cached    = nullptr;
    s_map_key_tag   = -1;
    s_map_key_nodes = 0;
    std::snprintf(s_status, sizeof(s_status), "palettearm: off");
}

void palettearm_update(float delta_seconds) {
    if (!arm_driver_owns(ArmDriverMode::Palette)) return;

    if (!pa::palettehook_installed()) {
        // WaitingForModule is retried every tick and costs a GetModuleHandleA. Failed is
        // terminal on purpose: the scan is deterministic over a loaded image, so a second
        // attempt cannot succeed where the first did not, and retrying would only spam.
        const pa::HookInstall outcome = pa::palettehook_install(&drive_palette);
        if (outcome == pa::HookInstall::Failed) {
            s_unavailable = true;
            API::get()->log_info("[Halo-CampE-UEVR] PALETTEARM: giving up for this session -- "
                                 "falling back to the UE arm driver. Set armdriver to something "
                                 "else and back to 2 to retry.");
        }
        if (outcome != pa::HookInstall::Installed) {
            std::snprintf(s_status, sizeof(s_status), "palettearm: not installed (%s)",
                          pa::palettehook_resolution());
            return;
        }
    }

    capture_tracking();

    // Per-weapon rigid delta + its capture gesture. AFTER capture_tracking() and BEFORE the drive
    // runs, so a freeze latched this tick is in place for the very next build rather than a frame
    // late -- the visible cost of a frame there is the gun twitching once as the key goes down.
    weapon_fix_tick();

    // The support hand's rigid delta + its capture gesture, in the same slot and for the same
    // reason: the freeze must be standing before the drive next builds a pose.
    hand_fix_tick();

    // The two-handed hold is NOT ticked here any more: src\TwoHandAim.cpp owns it and ticks
    // it in every armdriver mode. drive_palette() reads the swing it publishes.

    // "Installed" is not "running". The watchdog is the only thing that catches an address that
    // took the hook and is never called.
    // WHAT "POSSIBLE" MEANS HERE, and why the first answer was wrong.
    //
    // This used to pass s_tracking_ready -- "are controller poses readable". That is TRUE on a
    // loading screen, so the watchdog counted load time as time the hook could have been called,
    // and on 2026-08-24 it condemned a freshly installed hook 7.2 seconds later with "0 calls
    // seen", before the level had finished loading and before any first-person weapon existed.
    // That is verbatim the failure addrcascade's README documents ("installed at the main menu,
    // alarm fired 11 seconds before gameplay started") -- quoted in this folder's own header, and
    // made anyway.
    //
    // The hook builds a first-person WEAPON. So the honest question is whether a weapon is in
    // hand: no weapon, no call possible, no evidence either way. weapon_offset_current_class() is
    // published each tick by machinery that already resolved it, so this costs a pointer read.
    const char* held = weapon_offset_current_class();
    const bool weapon_in_hand = (held != nullptr && held[0] != 0);
    if (pa::palettehook_watchdog_tick(/*gameplay_active=*/weapon_in_hand &&
                                      s_tracking_ready.load(std::memory_order_acquire))) {
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEARM: the hook installed but has NEVER been "
                             "called. The resolved address (%s) is not the first-person weapon "
                             "builder on this build. Falling back to the UE arm driver.",
                             pa::palettehook_resolution());
        s_unavailable = true;
    }

    // Why the capture mirror declined, per reason. Named counters rather than "banks stopped", so a
    // freeze reports which gate closed instead of costing another round of guessing.
#if HALO_VR_DEV
    std::uint64_t cap_no_tls = 0, cap_no_ctx = 0, cap_gate = 0, cap_miss = 0;
    pa::palettehook_capture_census(&cap_no_tls, &cap_no_ctx, &cap_gate, &cap_miss);
#endif

    std::snprintf(s_status, sizeof(s_status),
                  "palettearm: %s | hook %llu calls (cap=%llu nocap=%llu) | banks %llu | node map %s " "| drive stage=%s ok=%llu",
                  pa::palettehook_resolution(),
                  (unsigned long long)pa::palettehook_call_count(),
                  (unsigned long long)pa::palettehook_capture_calls(),
                  (unsigned long long)pa::palettehook_nocapture_calls(),
                  (unsigned long long)pa::palettehook_bank_writes(),
                  pa::nodemap_tier_name(s_node_map.tier()),
                  s_drive_stage.load(std::memory_order_relaxed),
                  (unsigned long long)s_drive_ok.load(std::memory_order_relaxed));

    // Census appended only in a DEV build. The counters themselves are compiled out in release, so
    // printing them there would be four guaranteed zeros and a misleading line -- and this project
    // requires that anything answering a question rather than playing the game not exist at all in a
    // player build.
    HALO_VR_DEV_ONLY(
        {
            const std::size_t u = std::strlen(s_status);
            std::snprintf(s_status + u, sizeof(s_status) - u,
                          " | capdecline tls=%llu ctx=%llu gate=%llu tagmiss=%llu",
                          (unsigned long long)cap_no_tls, (unsigned long long)cap_no_ctx,
                          (unsigned long long)cap_gate,   (unsigned long long)cap_miss);
        });

    // A second line rather than a longer one: the geometry is what a human reads when the pose is
    // wrong, and burying it at the end of an already-long status line makes it easy to miss.
    HALO_VR_DEV_ONLY(
        std::snprintf(s_status_geom, sizeof(s_status_geom),
                      "palettearm geom: rootFwd=(%.2f,%.2f,%.2f) STOCKwrist=(%.3f,%.3f,%.3f) "
                      "wristTarget=(%.3f,%.3f,%.3f) got=(%.3f,%.3f,%.3f) "
                      "err=%.1fcm miss=%.1fcm reach=%.1fcm arms=%d torso=%d "
                      "vdelta=%.1fdeg meshdrv=%d wpn=%s reach=%.1fcm sh=(%.3f,%.3f,%.3f) "
                      "yaw=%.1f/head%.1f gunhand=%.1fcm "
                      "sup=(%.3f,%.3f,%.3f) supsh=(%.3f,%.3f,%.3f) suptf=(%.3f,%.3f,%.3f) stockL=(%.3f,%.3f,%.3f) aimpitch=%.1f handgun=(%.3f,%.3f,%.3f) blend=%.2f gotL=(%.3f,%.3f,%.3f) elbL=(%.3f,%.3f,%.3f) missL=%.1fcm viewpitch=%.1f",
                      s_dbg_fwd_x.load(), s_dbg_fwd_y.load(), s_dbg_fwd_z.load(),
                      s_dbg_stock_x.load(), s_dbg_stock_y.load(), s_dbg_stock_z.load(),
                      s_dbg_tgt_x.load(),  s_dbg_tgt_y.load(),  s_dbg_tgt_z.load(),
                      s_dbg_got_x.load(),  s_dbg_got_y.load(),  s_dbg_got_z.load(),
                      std::sqrt((s_dbg_tgt_x.load() - s_dbg_stock_x.load()) *
                                (s_dbg_tgt_x.load() - s_dbg_stock_x.load()) +
                                (s_dbg_tgt_y.load() - s_dbg_stock_y.load()) *
                                (s_dbg_tgt_y.load() - s_dbg_stock_y.load()) +
                                (s_dbg_tgt_z.load() - s_dbg_stock_z.load()) *
                                (s_dbg_tgt_z.load() - s_dbg_stock_z.load())) *
                          pa::kMetresPerBlamUnit * 100.0f,
                      s_dbg_miss_cm.load(), s_dbg_reach_cm.load(),
                      g_cfg.pa_arms, g_cfg.pa_torso_frame,
                      ::halo::g_view_lock_delta.load(),
                      (int)(!::halo::weapon_drive_owns() && !palettearm_weapon_owns()),
                      g_cfg.pa_weapon ? (s_wpn_driven.load() ? "driven" : "REFUSED") : "off",
                      s_dbg_wpn_reach.load(),
                      s_dbg_sh_x.load(), s_dbg_sh_y.load(), s_dbg_sh_z.load(),
                      s_dbg_yaw_hands.load(), s_dbg_yaw_head.load(),
                      s_dbg_wpn_ok.load()
                          ? std::sqrt((s_dbg_wpn_x.load() - s_dbg_tgt_x.load()) *
                                      (s_dbg_wpn_x.load() - s_dbg_tgt_x.load()) +
                                      (s_dbg_wpn_y.load() - s_dbg_tgt_y.load()) *
                                      (s_dbg_wpn_y.load() - s_dbg_tgt_y.load()) +
                                      (s_dbg_wpn_z.load() - s_dbg_tgt_z.load()) *
                                      (s_dbg_wpn_z.load() - s_dbg_tgt_z.load())) *
                                pa::kMetresPerBlamUnit * 100.0f
                          : -1.0f,
                      s_dbg_sup_x.load(), s_dbg_sup_y.load(), s_dbg_sup_z.load(),
                      s_dbg_supsh_x.load(), s_dbg_supsh_y.load(), s_dbg_supsh_z.load(),
                        s_dbg_suptf_x.load(), s_dbg_suptf_y.load(), s_dbg_suptf_z.load(),
                        s_dbg_stockL_x.load(), s_dbg_stockL_y.load(), s_dbg_stockL_z.load(),
                        ::halo::g_desired_pitch.load(),
                        s_dbg_handgun_x.load(), s_dbg_handgun_y.load(), s_dbg_handgun_z.load(),
                        s_dbg_blend.load(),
                        s_dbg_gotL_x.load(), s_dbg_gotL_y.load(), s_dbg_gotL_z.load(),
                        s_dbg_elbL_x.load(), s_dbg_elbL_y.load(), s_dbg_elbL_z.load(),
                        s_dbg_missL.load(),
                        ::halo::g_view_pitch.load()));

    // THE BARREL LOCK, APPENDED RATHER THAN WOVEN IN. A separate snprintf onto the same buffer
    // keeps this out of the format string above, which several people edit at once -- and it is
    // the pair of numbers that decides whether the lock's assumption holds on this build, so it
    // wants to be findable rather than buried mid-line. See Config.hpp pa_barrel_lock.
    //   barreloff = degrees the rendered barrel sits off the aim ray BEFORE any correction
    //               (-1 = no barrel direction this frame). Near zero at rest = assumption good.
    //   barrelstr = how much of the correction the fade actually applied, 0..1 (0 while off).
    HALO_VR_DEV_ONLY(do {
        const std::size_t used = std::strlen(s_status_geom);
        if (used + 1 < sizeof(s_status_geom)) {
            std::snprintf(s_status_geom + used, sizeof(s_status_geom) - used,
                          " | barrel=%s off=%.1fdeg str=%.2f",
                          g_cfg.pa_barrel_lock ? "on" : "off",
                          s_dbg_barrel_off.load(), s_dbg_barrel_str.load());
        }
    } while (false));
}

} // namespace halo
