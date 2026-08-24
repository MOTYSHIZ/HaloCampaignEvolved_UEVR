#include "PaletteArm.hpp"

#include "ArmSolve.hpp"
#include "NodeMap.hpp"
#include "PaletteHook.hpp"
#include "PaletteMath.hpp"
#include "TwoHand.hpp"

#include "../ArmDriver.hpp"
#include "../Config.hpp"
#include "../MotionAimControl.hpp"   // get_pose()

#include "uevr/API.hpp"

#include <atomic>
#include <cstdio>
#include <cstring>

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

pa::TwoHandHold  s_two_hand;
pa::ArmTuning    s_arm_tuning;
pa::TwoHandTuning s_two_hand_tuning;

// Written on the game thread each tick, read inside the detour (a different call, same thread in
// practice, but not guaranteed). Plain values, published as one struct under a seqlock-free
// double-buffer would be overkill here -- atomics on the few scalars that matter is enough,
// because a torn pose for one frame is a wobble, not a crash.
struct TrackingSnapshot {
    pa::Vec3 hmd_position{};
    pa::Quat hmd_rotation{};
    pa::Vec3 aim_grip_position{};
    pa::Quat aim_grip_rotation{};
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

// The give-up latch. See palettearm_unavailable() in the header for why this is a static and
// not a config field.
bool s_unavailable = false;

// ---- THE DRIVE, run inside the detour ----------------------------------------------------------

bool drive_palette(const pa::PaletteAccess& access) {
    if (!s_tracking_ready.load(std::memory_order_acquire)) return false;
    const TrackingSnapshot tracking = s_tracking;
    if (!tracking.valid) return false;

    // The guard. Not "is this memory writable" -- "does the thing at index 19 still behave like a
    // wrist". Failing here leaves the stock pose alone, which is the correct fail-closed answer.
    if (!pa::nodemap_validate(access.palette, access.node_count)) {
        static bool s_said = false;
        if (!s_said) {
            s_said = true;
            API::get()->log_info("[Halo-CampE-UEVR] PALETTEARM: node map rejected (%s) -- arms stay "
                                 "stock. The first-person skeleton is not the one this was "
                                 "measured on.", pa::nodemap_last_failure());
        }
        return false;
    }

    const pa::Mat3 root_basis  = pa::orthonormal_basis(access.palette[pa::kRootNode]);
    if (!pa::valid_basis(root_basis)) return false;
    const pa::Vec3 root_position = access.palette[pa::kRootNode].position;
    const pa::Mat3 torso_basis = pa::torso_basis_from_root(root_basis);
    if (!pa::valid_basis(torso_basis)) return false;

    // Which physical hand aims. Asking it once here is what keeps left-handed play working without
    // a second code path -- the same reasoning Hands.cpp uses.
    const bool aim_is_right = !g_cfg.aim_left_hand;

    const pa::Vec3 aim_grip_blam =
        root_position + pa::transform_vector(root_basis,
            xr_to_blam(tracking.aim_grip_position - tracking.hmd_position) / pa::kMetresPerBlamUnit);
    const pa::Vec3 support_grip_blam =
        root_position + pa::transform_vector(root_basis,
            xr_to_blam(tracking.support_grip_position - tracking.hmd_position) / pa::kMetresPerBlamUnit);

    // ONE basis feeds everything. See palettearm\README.md: a second copy of this blend drifts,
    // and the symptom is shots that miss where the barrel is pointing.
    //
    // Run in OPENXR SPACE, in raw metres -- the same frame and unit palettearm_update() feeds
    // TwoHandHold::update(). The two calls share remembered state (the ease-out direction), so
    // they must share a frame; see the warning on TwoHandInput. Only the RESULT is relabelled
    // into Blam axes, which is exact because the relabel is a proper rotation.
    const pa::Mat3 blended_xr = s_two_hand.effective_basis(
        xr_rotation_to_xr_basis(tracking.aim_aim_rotation),
        tracking.aim_grip_position, tracking.support_grip_position,
        tracking.support_valid, s_two_hand_tuning);
    const pa::Mat3 aim_basis = blam_basis_from_xr_basis(blended_xr);
    if (!pa::valid_basis(aim_basis)) return false;

    const pa::ArmNodes& aim_arm     = pa::arm_nodes(aim_is_right);
    const pa::ArmNodes& support_arm = pa::arm_nodes(!aim_is_right);

    // ---- the aim arm
    if (!pa::anchor_shoulder_to_torso(access.palette, aim_arm, torso_basis, root_position,
                                      !aim_is_right, s_arm_tuning)) {
        return false;
    }
    const pa::Vec3 aim_wrist = pa::wrist_from_grip(aim_grip_blam, aim_basis, s_arm_tuning);
    if (!pa::solve_arm_for_tracked_wrist(access.palette, aim_arm, aim_wrist, aim_basis,
                                         root_basis.up, s_arm_tuning)) {
        return false;
    }

    // ---- the support arm, only when it is actually tracked
    if (tracking.support_valid) {
        const pa::Mat3 support_basis = xr_rotation_to_blam_basis(tracking.support_grip_rotation);
        if (pa::valid_basis(support_basis)) {
            pa::anchor_shoulder_to_torso(access.palette, support_arm, torso_basis, root_position,
                                         aim_is_right, s_arm_tuning);
            const pa::Vec3 support_wrist =
                pa::wrist_from_grip(support_grip_blam, support_basis, s_arm_tuning);
            pa::solve_arm_for_tracked_wrist(access.palette, support_arm, support_wrist,
                                            support_basis, root_basis.up, s_arm_tuning);
        }
    }

    // ---- fingers LAST, deliberately. Wrist placement above is a rigid transform and nothing in
    // the aim path reads finger nodes, so opening a hand cannot perturb where the gun points.
    // Curl inputs are not wired yet -- the hands hold the authored grip until they are.
    const pa::HandCurl closed{};
    pa::apply_hand_openness(access.palette, aim_arm, closed);
    if (tracking.support_valid) pa::apply_hand_openness(access.palette, support_arm, closed);

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

    // Grip pose for POSITION, aim pose for DIRECTION. Plugin.cpp documents why they are not
    // interchangeable here: get_aim_pose() produces teleport-scale translation readings, so its
    // rotation is usable and its position is not.
    const int aim_idx     = g_cfg.aim_left_hand ? 0 : 1;
    const int support_idx = g_cfg.aim_left_hand ? 1 : 0;

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

} // namespace

bool palettearm_parse_key(const char* key, double v) {
    if      (_stricmp(key, "pashoulderback")  == 0) s_arm_tuning.shoulder_back_m      = (float)v;
    else if (_stricmp(key, "pashoulderdown")  == 0) s_arm_tuning.shoulder_down_m      = (float)v;
    else if (_stricmp(key, "pashoulderlat")   == 0) s_arm_tuning.shoulder_lateral_m   = (float)v;
    else if (_stricmp(key, "paclavicle")      == 0) s_arm_tuning.clavicle_assist_m    = (float)v;
    else if (_stricmp(key, "pawristback")     == 0) s_arm_tuning.grip_to_wrist_back_m = (float)v;
    else if (_stricmp(key, "pawristdown")     == 0) s_arm_tuning.grip_to_wrist_down_m = (float)v;
    else if (_stricmp(key, "pa2hmin")         == 0) s_two_hand_tuning.zone_min_along_m = (float)v;
    else if (_stricmp(key, "pa2hmax")         == 0) s_two_hand_tuning.zone_max_along_m = (float)v;
    else if (_stricmp(key, "pa2hradius")      == 0) s_two_hand_tuning.zone_radius_m    = (float)v;
    else if (_stricmp(key, "pa2hagreemin")    == 0) s_two_hand_tuning.minimum_agreement = (float)v;
    else if (_stricmp(key, "pa2hagreefull")   == 0) s_two_hand_tuning.full_agreement    = (float)v;
    else if (_stricmp(key, "pa2hblend")       == 0) s_two_hand_tuning.blend_seconds     = (float)v;
    else return false;
    return true;
}

const char* palettearm_status() { return s_status; }

bool palettearm_unavailable() { return s_unavailable; }

void palettearm_retry() {
    if (!s_unavailable) return;
    s_unavailable = false;
    API::get()->log_info("[Halo-CampE-UEVR] PALETTEARM: re-armed by an armdriver change -- "
                         "will try to resolve the first-person weapon builder again");
}

void palettearm_release() {
    pa::palettehook_uninstall();
    s_two_hand.reset();
    s_tracking_ready.store(false, std::memory_order_release);
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

    // Two-handed hold. The zone gates ACQUISITION only -- once latched it holds until the grip
    // button releases. Fed from the same snapshot the drive uses so the latch and the render agree.
    pa::TwoHandInput input{};
    if (s_tracking_ready.load(std::memory_order_acquire)) {
        const TrackingSnapshot t = s_tracking;
        input.aim_grip_position     = t.aim_grip_position;
        // XR-space basis to match the RAW XR positions on either side of it. Feeding a Blam
        // basis here alongside OpenXR positions is the bug this pairing exists to prevent.
        input.aim_basis             = xr_rotation_to_xr_basis(t.aim_aim_rotation);
        input.support_grip_position = t.support_grip_position;
        input.support_tracked       = t.support_valid;
        input.gameplay_active       = true;
        input.delta_seconds         = delta_seconds;

        static UEVR_ActionHandle s_grip = nullptr;
        if (s_grip == nullptr) s_grip = API::VR::get_action_handle("/actions/default/in/Grip");
        input.support_grip_held =
            s_grip != nullptr &&
            API::VR::is_action_active(s_grip, g_cfg.aim_left_hand
                                                  ? API::VR::get_right_joystick_source()
                                                  : API::VR::get_left_joystick_source());
    }
    const pa::TwoHandState two_hand = s_two_hand.update(input, s_two_hand_tuning);

    if (two_hand.latch_changed) {
        // ABI order: (seconds_from_now, duration, frequency, amplitude). The C++ wrapper's
        // parameter NAMES disagree with that order -- these are ordered for the ABI.
        API::VR::trigger_haptic_vibration(0.0f, two_hand.latched ? 0.12f : 0.06f, 0.0f,
                                          two_hand.latched ? 0.7f : 0.35f,
                                          g_cfg.aim_left_hand ? API::VR::get_right_joystick_source()
                                                              : API::VR::get_left_joystick_source());
    }

    // "Installed" is not "running". The watchdog is the only thing that catches an address that
    // took the hook and is never called.
    if (pa::palettehook_watchdog_tick(/*gameplay_active=*/s_tracking_ready.load(
            std::memory_order_acquire))) {
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEARM: the hook installed but has NEVER been "
                             "called. The resolved address (%s) is not the first-person weapon "
                             "builder on this build. Falling back to the UE arm driver.",
                             pa::palettehook_resolution());
        s_unavailable = true;
    }

    std::snprintf(s_status, sizeof(s_status), "palettearm: %s, %llu calls, 2h=%s blend=%.2f",
                  pa::palettehook_resolution(),
                  (unsigned long long)pa::palettehook_call_count(),
                  two_hand.latched ? "held" : (two_hand.in_zone ? "in-zone" : "off"),
                  two_hand.blend);
}

} // namespace halo
