#include "features/palettewpn/PaletteFrame.hpp"

#include "Arms.hpp"
#include "BlamPalette.hpp"
#include "Config.hpp"
#include "Markers.hpp"
#include "Math.hpp"
#include "MotionAimControl.hpp"
#include "PaletteTwoHand.hpp"
#include "Reticule.hpp"
#include "Rig.hpp"
#include "TwoHandAim.hpp"
#include "UeObject.hpp"
#include "WeaponCalib.hpp"
#include "WeaponOffset.hpp"
#include "core/fixes/TickStage.hpp"   // g_tick_stage
#include "core/host/ArmsState.hpp"   // arms_hide_update
#include "core/host/PluginState.hpp"
#include "features/palettewpn/PaletteArmDriver.hpp"
#include "features/palettewpn/PaletteReadbacks.hpp"
#include "features/palettewpn/PoseLatch.hpp"
#include "uevr/API.hpp"

#include <Windows.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

using namespace uevr;

namespace halo {

// ---- CROSS-MODULE PUBLISHES (namespace halo: BlamPalette/WeaponCalib extern these).
// The rendered-view pitch, published beside the yaw for the palette's frame math.
std::atomic<float> g_dbg_view_in_pitch{0.0f};

// The yaw the stereo callback actually OUTPUT this frame -- the rendered base the palette's lock
// correction divides against. Stored on every path through the callback (passthrough when the
// lock is off or unprimed, the assigned value when it holds), so the palette maths reads what was
// really rendered instead of rebuilding locked+turn from parts. In namespace halo for
// BlamPalette.cpp; written from the render thread, read on the game tick -- one frame of
// staleness during a snap turn, invisible next to the turn itself.

// ---- THE MESH-VS-CAMERA CONSTANTS, for the palette's world-space pullback.
//
// The sweep proved the palette's camera error is a TRANSLATION: the FP mesh hangs off the
// rotating camera on a lever, so aim swings sweep the whole bone frame through the world --
// metre-scale, unfixable by any rotation of the bones. The fix is to aim the bones at a WORLD
// target (the same one rigmode's proven math produces) and pull it back through the mesh's
// actual world transform. That transform decomposes as
//     comp_rot = cam * M                where M  = conj(cam) * comp_rot     (rotation constant)
//     comp_pos = parent_pos + cam * v0  where v0 = conj(cam) * (comp - parent)  (lever constant)
// with cam = the aim rotator -- the ONE volatile term, which the sim-side hook can read FRESH at
// build time. M and v0 are constants of the rig (mesh-authoring yaw, camera-to-mesh lever),
// measured here on the game thread where reflected reads are possible, published for the hook.
// If they are NOT constant the model is wrong, so their drift is measured and printed too -- a
// drifting "constant" is a theory failing loudly, which is the only acceptable way left.
std::atomic<float> g_meshM_x{0.0f}, g_meshM_y{0.0f}, g_meshM_z{0.0f}, g_meshM_w{1.0f};
std::atomic<float> g_meshV0_x{0.0f}, g_meshV0_y{0.0f}, g_meshV0_z{0.0f};
// The GAME THREAD's camera, published each tick from the mesh-constant block below -- the same
// read, same thread, same moment as the transform the FP mesh is built from. The palette build
// consumes THIS (palettecam=6) instead of reading ControlRotation from the sim thread at a
// different moment: the right-hand-only flicks were the gap between those two moments.
std::atomic<float> g_tick_cam_p{0.0f}, g_tick_cam_y{0.0f};
std::atomic<long long> g_tick_cam_ms{0};
// The mesh's OWN rotation at the same game-thread instant (palettecam=8's source): the one
// epoch-consistent sample of the transform the tick will render, no render-thread K2 read.
std::atomic<float> g_tick_mrot_x{0.0f}, g_tick_mrot_y{0.0f}, g_tick_mrot_z{0.0f}, g_tick_mrot_w{1.0f};
// MESHCONST instrument (Config.hpp mesh_const): how far the camera moved BETWEEN the two
// ControlRotation reads that bracket the reflected mesh read, in degrees. This is the
// contamination in M, measured directly, per tick. g_meshM_age_ms says how old the M currently
// in use is, so a held-but-clean M cannot be mistaken for a fresh one.
// The tick publishes THREE things that only mean anything together, cam_tick, M and the mesh
// rotation Q_tick, and until now they were three unsynchronised relaxed stores. A reader that
// straddles a tick gets cam from tick N with M from tick N-1, and since M = conj(cam) x Q the
// product then misses by exactly one tick of camera motion, which is the same magnitude as the
// judder. Odd seq means a write is in progress. Same doctrine as g_p_seq.
std::atomic<unsigned> g_tick_seq{0};
std::atomic<float> g_meshM_brk_deg{0.0f};
std::atomic<long long> g_meshM_acc_ms{0};
std::atomic<unsigned> g_meshM_acc{0}, g_meshM_rej{0};
// The engine tick counter, bumped at tick start -- palbuildgate=4 keys "first build this tick"
// on it, and it costs one relaxed add whether or not anything reads it.
std::atomic<unsigned> g_tick_id{0};

// ---- STOMPLOG (Config.hpp stomp_log). Points: 0 = engine-tick start, 1 = stereo pre eye 0,
// 2 = stereo pre eye 1, 3 = stereo post eye 0. Single ring; concurrent writers use an atomic
// index; a torn row is one bad sample in a hunt instrument.
namespace stomplog {
struct Row { double t_ms; int point; float yaw; float e0; float e1; float e2; };
// CIRCULAR since 2026-09-11: the ring holds the LAST ~100 s instead of the first, so the key
// can stay ON all session and a flush (key to 0, or teardown) dumps the freshest window --
// "all of the logging is on" as a standing state, not a 60-second appointment.
constexpr int kCap = 60000;
Row g_rows[kCap];
std::atomic<int> g_n{0};
std::atomic<bool> g_was_on{false};
int g_seq = 0;
double now_ms() {
    return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
} // namespace stomplog
void stomp_flush() {
    if (stomplog::g_was_on.load(std::memory_order_relaxed)) {
        {
            const int n = stomplog::g_n.exchange(0, std::memory_order_relaxed);
            stomplog::g_was_on.store(false, std::memory_order_relaxed);
            if (n > 0 && g_cfg_path[0] != '\0') {
                char path[MAX_PATH]; strncpy_s(path, sizeof(path), g_cfg_path, _TRUNCATE);
                char* slash = strrchr(path, '\\');
                if (slash != nullptr) {
                    char leaf[64]; sprintf_s(leaf, "halo_vr_stomp_%03d.csv", stomplog::g_seq++);
                    slash[1] = '\0'; strncat_s(path, sizeof(path), leaf, _TRUNCATE);
                    FILE* f = nullptr;
                    if (fopen_s(&f, path, "wb") == 0 && f != nullptr) {
                        fprintf(f, "t_ms,point,yaw,e0,e1,e2\r\n");
                        const int m = n < stomplog::kCap ? n : stomplog::kCap;
                        const int start = (n > stomplog::kCap) ? (n % stomplog::kCap) : 0;
                        for (int i = 0; i < m; ++i) {
                            const stomplog::Row& rw = stomplog::g_rows[(start + i) % stomplog::kCap];
                            fprintf(f, "%.3f,%d,%.4f,%.4f,%.4f,%.4f\r\n", rw.t_ms, rw.point, rw.yaw, rw.e0, rw.e1, rw.e2);
                        }
                        fclose(f);
                        API::get()->log_info("[Halo-CampE-UEVR] STOMPLOG: %d samples -> %hs", m, leaf);
                    }
                }
            }
        }
    }
}
// The periodic variant: the mode-6 test was lost when the rolling ring overwrote it during
// typing time -- evidence must land on disk BEFORE it can age out. Ring snapshot on the game
// thread (~1.5 MB copy, sub-millisecond), CSV written by a detached thread. Torn rows from
// writers racing the copy are a few bad samples in a hunt instrument.
static void stomp_flush_async() {
    const int n = stomplog::g_n.exchange(0, std::memory_order_relaxed);
    if (n <= 0 || g_cfg_path[0] == '\0') return;
    const int m = n < stomplog::kCap ? n : stomplog::kCap;
    const int start = (n > stomplog::kCap) ? (n % stomplog::kCap) : 0;
    auto* buf = new stomplog::Row[m];
    for (int i = 0; i < m; ++i) buf[i] = stomplog::g_rows[(start + i) % stomplog::kCap];
    char path[MAX_PATH]; strncpy_s(path, sizeof(path), g_cfg_path, _TRUNCATE);
    char* slash = strrchr(path, '\\');
    if (slash == nullptr) { delete[] buf; return; }
    char leaf[64]; sprintf_s(leaf, "halo_vr_stomp_%03d.csv", stomplog::g_seq++);
    slash[1] = '\0'; strncat_s(path, sizeof(path), leaf, _TRUNCATE);
    std::string spath(path);
    std::thread([buf, m, spath]() {
        FILE* f = nullptr;
        if (fopen_s(&f, spath.c_str(), "wb") == 0 && f != nullptr) {
            fprintf(f, "t_ms,point,yaw,e0,e1,e2\r\n");
            for (int i = 0; i < m; ++i)
                fprintf(f, "%.3f,%d,%.4f,%.4f,%.4f,%.4f\r\n", buf[i].t_ms, buf[i].point, buf[i].yaw, buf[i].e0, buf[i].e1, buf[i].e2);
            fclose(f);
            API::get()->log_info("[Halo-CampE-UEVR] STOMPLOG auto-flush: %d samples -> %hs", m, spath.c_str());
        } else { }
        delete[] buf;
    }).detach();
}
static void stomp_sample(int point) {
    if (g_cfg.stomp_log == 0) { stomp_flush(); return; }
    if (point == 0 && stomplog::g_n.load(std::memory_order_relaxed) >= (stomplog::kCap * 4) / 5)
        stomp_flush_async();
    stomplog::g_was_on.store(true, std::memory_order_relaxed);
    auto* comp = rig_tracked_component();
    if (comp == nullptr) return;
    Vec3 crot{};
    if (!call_ret_vec3(comp, L"K2_GetComponentRotation", &crot)) return;
    const Quat M{halo::g_meshM_x.load(std::memory_order_relaxed), halo::g_meshM_y.load(std::memory_order_relaxed),
                       halo::g_meshM_z.load(std::memory_order_relaxed), halo::g_meshM_w.load(std::memory_order_relaxed)};
    const Quat cam = quat_mul(rotator_to_quat(crot.x, crot.y, crot.z), quat_conj(M));
    float p = 0.0f, y = 0.0f, r = 0.0f;
    quat_to_rotator(cam.x, cam.y, cam.z, cam.w, &p, &y, &r);
    // Beside the mesh camera: the tick id and ControlRotation AT THIS SAME INSTANT, so the CSV
    // shows when the game updates each of the three within the tick, not just the mesh.
    double scp = 0.0, scy = 0.0;
    if (!read_control_rotation_hook(&scp, &scy)) { scp = 0.0; scy = 0.0; }
    const int i = stomplog::g_n.fetch_add(1, std::memory_order_relaxed);
    stomplog::g_rows[i % stomplog::kCap] = stomplog::Row{stomplog::now_ms(), point, y,
        (float)g_tick_id.load(std::memory_order_relaxed), (float)scy, (float)scp};
}
// The SIM-THREAD entry into the same ring (hooked_pose stamps its builds here). No UObject
// reads -- the caller supplies the values -- so it is safe from any thread; the atomic index
// makes ring order a true arrival order across threads. Points 4..7 = build calls,
// 4 + weapon_slot*2 + (capture ? 0 : 1). Flushing stays with stomp_sample on the tick.
void stomp_mark(int point, float yaw, float e0, float e1, float e2) {
    if (g_cfg.stomp_log == 0) return;
    const int i = stomplog::g_n.fetch_add(1, std::memory_order_relaxed);
    stomplog::g_rows[i % stomplog::kCap] = stomplog::Row{stomplog::now_ms(), point, yaw, e0, e1, e2};
}
std::atomic<bool>  g_mesh_const_valid{false};
// Where the hook last WROTE node 8 (palette units), published so the game thread's TRACE-NODE8
// can put it beside the socket read back from the posed skeleton, in the same frame.
std::atomic<float> g_dbg_node8_x{0.0f}, g_dbg_node8_y{0.0f}, g_dbg_node8_z{0.0f};
// The palette's WORLD pose as last resolved (view-lifted, grip fix included, before the camera
// divide), published so TRACE-BARREL can solve the barrel->aim correction in the pose's own frame.
std::atomic<float> g_dbg_pose_w_x{0.0f}, g_dbg_pose_w_y{0.0f}, g_dbg_pose_w_z{0.0f}, g_dbg_pose_w_w{1.0f};
// The barrel axis IN THE PALETTE POSE'S FRAME: conj(pose_w) * (socket -Y in world), measured on the
// game thread from the posed skeleton and averaged over still rows. A constant of the mesh, not
// of the player. Consumed by the pullback's barrel lock (see Config::palette_barrel_lock).
std::atomic<float> g_barrel_axis_x{0.0f}, g_barrel_axis_y{0.0f}, g_barrel_axis_z{0.0f};
std::atomic<bool>  g_barrel_axis_valid{false};
// The camera the projection-scale fix was last applied to (see the FPSCALE block in the
// tick). Reset when the rig re-resolves so a new camera object is fixed again.
API::UObject* g_fpscale_camera = nullptr;

// Called by the arm driver arbiter when the aim owner changes (the author's rig and shotpoint aim
// versus the palette weapon, armdriver mode 3). Dropping the reference makes the next tick
// re-capture it against where the game is aiming now, so the view does not jump by the old owner's
// offset. The rig neutral is dropped with it. No calibration file is touched.
void aim_reanchor_request(const char* why) {
    std::atomic<bool>& g_have_ref = *host::g_plugin_state.have_ref;
    g_have_ref = false;
    g_rig_neutral_valid = false;
    API::get()->log_info("[Halo-CampE-UEVR] AIM: reference dropped (%s) -- re-captures next tick",
                         why != nullptr ? why : "?");
}

// ================================================================ THE PLUGIN HOOKS

// Separate address, separate hook, so no ownership handshake with the aim write is needed.
void palette_wpn_game_tick_after_blam_drive() { blam_palette_hook_tick(); }

void palette_wpn_game_tick_before_vehicle() {
    // ---- PER-WEAPON DELTAS: every tick. After the config reload above (a reload restores the
    // calibrated base and would wipe an applied adjustment), before anything below reads
    // grip/off. Re-captures its base only when g_cfg_load_gen changes, so per-tick is safe.
    weapon_offset_update();

    // ---- PALETTE WEAPON MODE (armdriver 3, experimental). The fork's per-tick placement work, at
    // the fork's position in the tick: the FP arm hide (arms_update, which normally calls it, only
    // runs under UeRig), then the rendered-hand poses the sim-thread palette hook composes from.
    // Its two-hand hold runs after gesture_update in the tick body, as the fork ordered it (the
    // latch must see the gestures settled for this tick).
    if (palette_weapon_mode()) {
        g_tick_stage = "arms_hide";
        arms_hide_update();
        g_tick_stage = "palette_publish";
        blam_palette_publish_poses();
    }

    // The stock FP arms give way to the palette-driven weapon (doctrine in Arms.cpp).

    // ---- THE RENDERED-HAND POSES, EVERY FRAME, published for the palette hook (sim thread).
    // A pose published on the config cadence left the weapon transforming against a hand
    // position up to half a second stale -- the gun swung toward where you USED to point.
    // The two-hand hold: latch, blend ramp, haptics. After the pose publish on purpose -- both
    // read the same grip state, and this one must see it settled for this tick.
}

void palette_wpn_game_tick_after_vehicle(uint32_t tick) {
    std::atomic<float>& g_locked_view_yaw = *host::g_plugin_state.locked_view_yaw;

    // ---- THE PARENT FRAME, MEASURED RATHER THAN MODELLED -- published for the palette weapon.
    //
    // The FP mesh renders every palette node under its attach parent's ACTUAL rotation, and the
    // original 0.2's rig block (below) is explicit about why that value must be READ from the
    // component rather than inferred: ControlRotation, the locked view base, and the aim quat
    // were each tried as reconstructions of it during the palette effort, and each failed with a
    // different geometry of "the gun follows the camera". One reflected call per tick, the same
    // price the rig has always paid. Fail closed: valid=false publishes no divide, and the
    // palette publisher refuses to emit a pose in a frame it cannot name.
    // ---- DECOUPLED PITCH STAYS ON. This is the one UEVR invariant where we must NOT copy 0.5.
    //
    // 0.5 pins decoupled pitch OFF, and it is safe for them because their controller never touches
    // the camera (AimMethod::GAME). Ours drives the game camera FROM the controller by design --
    // so with decoupled pitch off, the aim pitch went straight into the rendered view: "my
    // controller is controlling the pitch", and it made the player sick within a minute. That was
    // this block, one revision ago, pinning it off on a watchdog. Reversed: decoupled pitch is
    // pinned ON, which is what the profile shipped with and what a hand-driven camera requires.
    // The palette frame math already carries the camera pitch explicitly (comp_rot = aim rotator,
    // pitch included), so it does not depend on the view being coupled.
    if (g_cfg.pin_uevr_frame && (tick % 64u) == 0u) {
        if (!API::VR::is_decoupled_pitch_enabled()) {
            API::VR::set_decoupled_pitch_enabled(true);
            API::get()->log_info("[Halo-CampE-UEVR] UEVRPIN: decoupled pitch was OFF -- restored ON "
                                 "(hand-driven aim must never pitch the rendered view)");
        }
    }

    // ---- THE FIRST-PERSON PROJECTION SCALE. What the reference disables before anything else.
    //
    // Halo's UE 5.5 camera enables the engine's first-person primitive scale with
    // FirstPersonScale = 0.15: the FP rig -- our palette's mesh -- is drawn shrunk toward the
    // camera and re-projected. The 0.5 mod ships a script (halo_first_person_projection_fix)
    // whose whole job is to set that to 1.0 and turn the flag off; their pose-matrix validation
    // (palette node within 4e-8 m of the 3.048-based prediction) is taken with it disabled. We
    // never disabled it: every palette placement this codebase has ever rendered went through a
    // 0.15 depth-dependent squash, which is a lever proportional to camera distance -- and the
    // socket readback that measured "456 cm per unit" measured the squash, not the palette.
    //
    // Same policy as their script: apply once per new camera object, and re-apply only if game
    // logic explicitly turns the scale back on. Not rewritten every tick. FOV override handled
    // the same way (their REMOVE_FIRST_PERSON_FOV = true). g_rig_parent IS the CameraComponent
    // -- the object their script finds by class -- so no second lookup.
    g_tick_stage = "fp_scale";
    if (g_cfg.fp_scale_fix && g_rig_parent != nullptr) {
        auto* cam = g_rig_parent;
        auto* enabled = cam->get_property_data<bool>(L"bEnableFirstPersonScale");
        auto* scale   = cam->get_property_data<float>(L"FirstPersonScale");
        if (enabled != nullptr && scale != nullptr) {
            const bool new_camera = (g_fpscale_camera != cam);
            if (new_camera || *enabled) {
                const float old_scale = *scale;
                const bool  old_enabled = *enabled;
                *scale = 1.0f;
                *enabled = false;
                API::get()->log_info("[Halo-CampE-UEVR] FPSCALE camera=%p FirstPersonScale %.3f "
                                     "enabled=%d -> 1.000/false%s",
                                     (void*)cam, old_scale, (int)old_enabled,
                                     new_camera ? " (new camera)" : " (game re-enabled it)");
            }
            auto* fov_enabled = cam->get_property_data<bool>(L"bEnableFirstPersonFieldOfView");
            auto* fov_fp      = cam->get_property_data<float>(L"FirstPersonFieldOfView");
            auto* fov_world   = cam->get_property_data<float>(L"FieldOfView");
            if (fov_enabled != nullptr && fov_fp != nullptr && fov_world != nullptr &&
                (new_camera || *fov_enabled)) {
                *fov_fp = *fov_world;
                *fov_enabled = false;
            }
            g_fpscale_camera = cam;
        } else {
            static bool s_said = false;
            if (!s_said) {
                s_said = true;
                API::get()->log_info("[Halo-CampE-UEVR] FPSCALE: CameraComponent has no "
                                     "FirstPersonScale property on this build -- fix inert");
            }
        }
    }

    // ---- THE WHOLE PICTURE, ONE LINE, ONE FRAME, EVERY SECOND.
    //
    // From the headset: "I don't understand why this is so hard. Add extensive logging -- camera,
    // controller, gun position and rotations for each, and the aim ray." He is right: every
    // diagnostic so far printed one term and reasoned about the others. This prints all of them
    // TOGETHER, in the same UE world frame, from the same tick, so a disagreement between any
    // two is visible by inspection instead of by derivation. Rows: HEAD (HMD, room-relative,
    // metres, UE axes) / HAND (grip pose, room-relative, metres, UE axes) / CAM (ControlRotation:
    // the Blam camera the palette embeds in) / VIEW (what the stereo callback actually rendered)
    // / GUN (the FP mesh's PrimaryWeapon socket, read back from the posed skeleton -- where the
    // gun REALLY is, in world cm, and its rotation) / AIM (the aim ray's origin and direction).
    // The barrel-axis MEASUREMENT (feeds the barrel lock) lives in here and must run whether or
    // not the lines are logged: palettewpnlog gates the log_info calls only.
    // EVERY TICK WHILE A FREEZE KEY IS HELD, every 45 otherwise. DIAGNOSTIC CADENCE ONLY.
    // A hold is 1-2 s; at 45 ticks that is two or three samples, which is not enough to
    // characterise motion the player can see -- and a reading taken at that rate was once
    // reported as "the gun is holding still" when it was moving several cm. Held, this gives
    // ~130 samples/second of the rendered socket. Costs nothing when the key is up.
    // JUDDERLOG: every tick, capped, and it turns the trace lines on by itself.
    static int s_jl_lines = 0;
    const bool jl = g_cfg.judder_log > 0 && s_jl_lines < g_cfg.judder_log;
    const uint32_t trace_every = (halo::blam_palette_freeze_active() || jl) ? 1u : 45u;
    const bool pwl = g_cfg.palette_weapon_log || jl;
    if (jl) ++s_jl_lines;
    // The barrel axis is only measured while the palette weapon owns (mode 3). Otherwise it must stop
    // CLAIMING validity, or aimbore=3 and the barrel lock would use an axis from a pose no longer drawn.
    if (!palette_weapon_mode()) halo::g_barrel_axis_valid.store(false, std::memory_order_relaxed);
    if (palette_weapon_mode() && (tick % trace_every) == 0u) {
        // Head and hand, room-relative, UE axes.
        Vec3 hpos{}, gpos{}; Quat hq{}, gq{};
        const auto hidx = API::VR::get_hmd_index();
        const auto cidx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                              : API::VR::get_right_controller_index();
        const bool have_hmd  = hidx >= 0 && get_pose(hidx, &hpos, &hq, false);
        const bool have_hand = cidx >= 0 && get_pose(cidx, &gpos, &gq, false);
        float hp = 0, hy = 0, hr = 0, gp = 0, gy = 0, gr = 0;
        if (have_hmd)  quat_to_rotator(-hq.z, hq.x, hq.y, -hq.w, &hp, &hy, &hr);
        if (have_hand) quat_to_rotator(-gq.z, gq.x, gq.y, -gq.w, &gp, &gy, &gr);
        const Vec3 head_ue{-hpos.z, hpos.x, hpos.y};
        const Vec3 hand_ue{-gpos.z, gpos.x, gpos.y};

        // Camera (Blam) and rendered view.
        double cp = 0.0, cy = 0.0;
        const bool have_cam = read_control_rotation(&cp, &cy, nullptr);
        const float view_yaw = halo::g_view_base_yaw.load();
        const float view_pit = halo::g_dbg_view_in_pitch.load();

        // Gun: where the FP mesh's weapon socket really is, world cm, and the mesh's rotation.
        g_tick_stage = "parent_frame";
        Vec3 gun_pos{}; Vec3 gun_rot{}; bool have_gun = false;
        if (auto* rigc = rig_tracked_component()) {
            have_gun = rig_socket_world(rigc, L"PrimaryWeapon", &gun_pos)
                    && call_ret_vec3(rigc, L"K2_GetComponentRotation", &gun_rot);
        }
        // Parent (camera component) world position, so gun can be read RELATIVE to the eye.
        Vec3 par_pos{}; bool have_par = (g_rig_parent != nullptr)
            && call_ret_vec3(g_rig_parent, L"K2_GetComponentLocation", &par_pos);

        // Aim ray.
        const Vec3 aim_o{g_ret_origin.x, g_ret_origin.y, g_ret_origin.z};

        // THE JUDGE. hand-head is a room-frame vector; the room is pinned to the VIEW's yaw by the
        // lock, so rotating it by view yaw puts it in the WORLD -- the same frame gun_rel_eye is
        // read in. The two columns are then directly comparable, and their difference (err) is
        // the placement error in cm, measured, per second. No synthetic input can supply this;
        // only the real hand against the real gun socket does.
        // Same anchor the palette uses -- the STANDING ORIGIN, not the head -- or the judge would
        // report head motion as placement error while the palette (correctly) ignores it.
        const auto so_j = API::VR::get_standing_origin();
        const Vec3 so_ue{-so_j.z, so_j.x, so_j.y};
        const Vec3 hh{hand_ue.x - so_ue.x, hand_ue.y - so_ue.y, hand_ue.z - so_ue.z};
        const Vec3 hh_world = quat_rotate(rotator_to_quat(0.0f, view_yaw, 0.0f),
                                          Vec3{hh.x * 100.0f, hh.y * 100.0f, hh.z * 100.0f});
        const Vec3 gre{gun_pos.x - par_pos.x, gun_pos.y - par_pos.y, gun_pos.z - par_pos.z};
        const Vec3 err{gre.x - hh_world.x, gre.y - hh_world.y, gre.z - hh_world.z};

        if (pwl) API::get()->log_info(
            "[Halo-CampE-UEVR] TRACE head=(%.2f %.2f %.2f)m rot(p%.0f y%.0f r%.0f) | "
            "hand=(%.2f %.2f %.2f)m rot(p%.0f y%.0f r%.0f) hand-head=(%.2f %.2f %.2f)m | "
            "cam(p%.0f y%.0f) view(p%.0f y%.0f) | gun_socket_world=(%.0f %.0f %.0f)cm "
            "gun_rel_eye=(%.0f %.0f %.0f)cm mesh_rot(p%.0f y%.0f r%.0f) | "
            "aim_origin=(%.0f %.0f %.0f)cm | ok:hmd%d hand%d cam%d gun%d par%d",
            head_ue.x, head_ue.y, head_ue.z, hp, hy, hr,
            hand_ue.x, hand_ue.y, hand_ue.z, gp, gy, gr,
            hh.x, hh.y, hh.z,
            (float)cp, (float)cy, view_pit, view_yaw,
            gun_pos.x, gun_pos.y, gun_pos.z,
            gre.x, gre.y, gre.z,
            gun_rot.x, gun_rot.y, gun_rot.z,
            aim_o.x, aim_o.y, aim_o.z,
            (int)have_hmd, (int)have_hand, (int)have_cam, (int)have_gun, (int)have_par);
        // ERR expressed in the CAMERA's frame, so a term that rides the camera reads as a
        // constant column here instead of a swirl in world. And the two pitch numbers that decide
        // whether the residual is head-pitch, cam-pitch, or their difference, printed with it.
        const Vec3 err_cam = quat_rotate(quat_conj(rotator_to_quat((float)cp, (float)cy, 0.0f)), err);

        // THE SOCKET IN THE MESH'S OWN FRAME, next to what we WROTE to node 8. Four hand-frame
        // hypotheses have now failed against the JUDGE (cam yaw, view yaw, +-lock gap, hand-hmd),
        // and ERR_in_cam is a CONSTANT ~(0, +28, -15) cm at every pitch, gap and reach. A constant
        // camera-frame offset between the bone we place and the socket that renders is not a hand
        // frame error at all: it is an authored offset between the palette node and the mesh
        // socket. Measure it: pull the socket back into mesh-local (conj(mesh_rot) * (socket -
        // mesh_origin)) and print it beside node 8's written position, same frame, same units.
        // If they differ by a constant, that constant is the missing vector -- and it is measured.
        Vec3 sock_local{}; float w8x = 0, w8y = 0, w8z = 0;
        {
            const Quat mrq = rotator_to_quat(gun_rot.x, gun_rot.y, gun_rot.z);
            const Vec3 d{gun_pos.x - par_pos.x, gun_pos.y - par_pos.y, gun_pos.z - par_pos.z};
            const Vec3 sl = quat_rotate(quat_conj(mrq), d);
            // UE cm -> palette units, one Y flip, 304.8 -- the same map the write uses.
            sock_local = Vec3{sl.x / 304.8f, -sl.y / 304.8f, sl.z / 304.8f};
            w8x = halo::g_dbg_node8_x.load(); w8y = halo::g_dbg_node8_y.load(); w8z = halo::g_dbg_node8_z.load();
        }
        if (pwl) API::get()->log_info(
            "[Halo-CampE-UEVR] TRACE-NODE8 written=(%.3f %.3f %.3f)u  socket_in_mesh=(%.3f %.3f %.3f)u  "
            "diff=(%.3f %.3f %.3f)u = (%.0f %.0f %.0f)cm  [constant diff = authored socket offset]",
            w8x, w8y, w8z, sock_local.x, sock_local.y, sock_local.z,
            sock_local.x - w8x, sock_local.y - w8y, sock_local.z - w8z,
            (sock_local.x - w8x) * 304.8f, (sock_local.y - w8y) * 304.8f, (sock_local.z - w8z) * 304.8f);
        const std::string wkey_tr = weapon_key();
        if (pwl) API::get()->log_info(
            "[Halo-CampE-UEVR] TRACE-JUDGE hand_world=(%.1f %.1f %.1f)cm gun_rel_eye=(%.1f %.1f %.1f)cm "
            "ERR=(%.1f %.1f %.1f)cm |%.1f|cm  ERR_in_cam=(%.1f %.1f %.1f)cm  lock_gap=%.1f  "
            "cam_p=%.1f cam_y=%.1f head_p=%.1f head_y=%.1f hand_p=%.1f hand_y=%.1f dpitch=%.1f  reach=%.0fcm  th=%d  wpn=%s",
            hh_world.x, hh_world.y, hh_world.z, gre.x, gre.y, gre.z,
            err.x, err.y, err.z, std::sqrt(err.x*err.x + err.y*err.y + err.z*err.z),
            err_cam.x, err_cam.y, err_cam.z,
            wrap180((float)cy - view_yaw), (float)cp, (float)cy, hp, hy, gp, gy, wrap180((float)cp - hp),
            std::sqrt(hh_world.x*hh_world.x + hh_world.y*hh_world.y + hh_world.z*hh_world.z),
            (int)halo::two_hand_latched(),
            wkey_tr.empty() ? "-" : wkey_tr.c_str());

        // ---- THE BARREL. Position is proven; this is the gun's POINTING DIRECTION as rendered,
        // against the aim ray, in degrees. Read the socket's world rotation from the posed
        // skeleton, take its three axes, and print each one's angle to the aim ray -- the barrel
        // is whichever axis (or its negative) sits at a small constant angle. No assumption about
        // which authored axis is the barrel; the log picks it. Also the CONTROLLER's pointing
        // direction (through the rigid aim fix, lifted into the world by the view frame), so
        // barrel-vs-hand and aim-vs-hand are both numbers.
        {
            Vec3 srot{};
            const bool have_srot = (rig_tracked_component() != nullptr)
                && rig_socket_world_rot(rig_tracked_component(), L"PrimaryWeapon", &srot);
            const float ap = (float)cp * DEG2RAD, ay = (float)cy * DEG2RAD;
            const Vec3 aim_dir{std::cos(ap) * std::cos(ay), std::cos(ap) * std::sin(ay), std::sin(ap)};
            float hy_c = 0.0f, hp_c = 0.0f;
            const bool have_hand_dir = halo::derive_ctrl_angles(&hy_c, &hp_c,
                g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                    : API::VR::get_right_controller_index());
            const float rigid_frame = g_cfg.view_lock ? g_locked_view_yaw.load() : 0.0f;
            const float hpr = hp_c * DEG2RAD, hyr = (hy_c + rigid_frame) * DEG2RAD;
            const Vec3 hand_dir{std::cos(hpr) * std::cos(hyr), std::cos(hpr) * std::sin(hyr), std::sin(hpr)};
            auto angdeg = [](const Vec3& a, const Vec3& b) {
                const float d = clampf(a.x*b.x + a.y*b.y + a.z*b.z, -1.0f, 1.0f);
                return std::acos(d) * RAD2DEG;
            };
            if (have_srot) {
                const Quat sq = rotator_to_quat(srot.x, srot.y, srot.z);
                const Vec3 sx = quat_rotate(sq, Vec3{1, 0, 0});
                const Vec3 sy = quat_rotate(sq, Vec3{0, 1, 0});
                const Vec3 sz = quat_rotate(sq, Vec3{0, 0, 1});
                // THE CORRECTION, SOLVED: the barrel is the socket's -Y (measured 19:27). Bring the
                // barrel and the aim ray into the palette pose's own frame and take the shortest
                // arc between them: that quaternion, right-multiplied onto the grip fix rotation,
                // puts the barrel ON the aim ray. Printed at full precision so it can be averaged
                // over rows and written to the file without anyone placing a hand on anything.
                {
                    const Quat P{halo::g_dbg_pose_w_x.load(), halo::g_dbg_pose_w_y.load(),
                                 halo::g_dbg_pose_w_z.load(), halo::g_dbg_pose_w_w.load()};
                    const Vec3 barrel_w{-sy.x, -sy.y, -sy.z};
                    const Vec3 b_l = quat_rotate(quat_conj(P), barrel_w);
                    const Vec3 a_l = quat_rotate(quat_conj(P), aim_dir);
                    // MEASURE THE BARREL AXIS IN THE POSE FRAME (a constant of the mesh) and
                    // publish it for the pullback's barrel lock. Still rows only -- during motion the
                    // socket readback and the published pose are a frame apart and b_l smears.
                    // EMA so one bad row cannot steer it; valid after a handful of samples.
                    if (have_hand_dir && angdeg(aim_dir, hand_dir) < 1.0f) {
                        static Vec3 s_bl{}; static int s_bl_n = 0;
                        // Once settled, a sample far from the mean is a weapon swap or a stale socket
                        // read (20:06:54: one 12-degree excursion during Magnum->AR), not the axis
                        // moving. Skip it rather than steer the lock through it.
                        const float sbn = std::sqrt(s_bl.x*s_bl.x + s_bl.y*s_bl.y + s_bl.z*s_bl.z);
                        const bool outlier = (s_bl_n >= 5 && sbn > 1.0e-3f)
                            && angdeg(Vec3{s_bl.x/sbn, s_bl.y/sbn, s_bl.z/sbn}, b_l) > 10.0f;
                        if (s_bl_n == 0) s_bl = b_l;
                        else if (!outlier) { const float a = 0.2f; s_bl = Vec3{s_bl.x + a*(b_l.x - s_bl.x), s_bl.y + a*(b_l.y - s_bl.y), s_bl.z + a*(b_l.z - s_bl.z)}; }
                        if (!outlier) ++s_bl_n;
                        const float bn = std::sqrt(s_bl.x*s_bl.x + s_bl.y*s_bl.y + s_bl.z*s_bl.z);
                        if (bn > 1.0e-3f && s_bl_n >= 5) {
                            halo::g_barrel_axis_x = s_bl.x / bn; halo::g_barrel_axis_y = s_bl.y / bn; halo::g_barrel_axis_z = s_bl.z / bn;
                            halo::g_barrel_axis_valid = true;
                        }
                        if ((s_bl_n % 20) == 5) {
                            if (g_cfg.palette_weapon_log) API::get()->log_info("[Halo-CampE-UEVR] BARRELAXIS in pose frame = (%.4f %.4f %.4f) n=%d  (lock %s)",
                                                 s_bl.x / bn, s_bl.y / bn, s_bl.z / bn, s_bl_n,
                                                 g_cfg.palette_barrel_lock ? "ON" : "off");
                        }
                    }
                    const Vec3 ax{b_l.y * a_l.z - b_l.z * a_l.y, b_l.z * a_l.x - b_l.x * a_l.z, b_l.x * a_l.y - b_l.y * a_l.x};
                    const float dt = b_l.x * a_l.x + b_l.y * a_l.y + b_l.z * a_l.z;
                    Quat dq{ax.x, ax.y, ax.z, 1.0f + dt};
                    const float dn = std::sqrt(dq.x*dq.x + dq.y*dq.y + dq.z*dq.z + dq.w*dq.w);
                    if (dn > 1.0e-6f) { dq.x /= dn; dq.y /= dn; dq.z /= dn; dq.w /= dn; }
                    if (g_cfg.palette_weapon_log) API::get()->log_info(
                        "[Halo-CampE-UEVR] TRACE-BARRELFIX delta_in_pose=(%.6f %.6f %.6f %.6f) = %.2f deg  "
                        "pose_w=(%.4f %.4f %.4f %.4f)  wpn=%s  [right-multiply onto gripfix rot]",
                        dq.x, dq.y, dq.z, dq.w, 2.0f * std::acos(clampf(std::fabs(dq.w), 0.0f, 1.0f)) * RAD2DEG,
                        P.x, P.y, P.z, P.w, wkey_tr.empty() ? "-" : wkey_tr.c_str());
                }
                if (g_cfg.palette_weapon_log) API::get()->log_info(
                    "[Halo-CampE-UEVR] TRACE-BARREL socket_rot(p%.1f y%.1f r%.1f) | aim(p%.1f y%.1f) "
                    "hand(p%.1f y%.1f)%s aim-hand=%.1fdeg | angle to AIM: +X %.1f +Y %.1f +Z %.1f "
                    "| angle to HAND: +X %.1f +Y %.1f +Z %.1f  wpn=%s  [barrel = the axis near 0 or 180]",
                    srot.x, srot.y, srot.z, (float)cp, (float)cy, hp_c, hy_c + rigid_frame,
                    have_hand_dir ? "" : "(no hand)", angdeg(aim_dir, hand_dir),
                    angdeg(sx, aim_dir), angdeg(sy, aim_dir), angdeg(sz, aim_dir),
                    angdeg(sx, hand_dir), angdeg(sy, hand_dir), angdeg(sz, hand_dir),
                    wkey_tr.empty() ? "-" : wkey_tr.c_str());
            } else {
                if (g_cfg.palette_weapon_log) API::get()->log_info("[Halo-CampE-UEVR] TRACE-BARREL socket rotation unavailable | aim(p%.1f y%.1f) hand(p%.1f y%.1f) aim-hand=%.1fdeg",
                    (float)cp, (float)cy, hp_c, hy_c + rigid_frame, angdeg(aim_dir, hand_dir));
            }
        }
    }

    // ---- MEASURE THE MESH CONSTANTS (see their declaration next to g_view_base_yaw).
    //
    // M and v0 relate the FP mesh's world transform to the aim rotator. They are measured, not
    // modelled, and re-measured every tick: if they hold still the model stands and the hook can
    // reconstruct the mesh transform FRESH from the aim rotator alone; if they drift with aim,
    // the drift line below says so and the model is dead. Reflected reads, so game thread only.
    g_tick_stage = "palette_weapon_frame";
    if (palette_weapon_mode()) {
        bool ok = false;
        auto* comp = rig_tracked_component();
        double cy = 0.0, cp = 0.0;
        if (comp != nullptr && g_rig_parent != nullptr &&
            read_control_rotation(&cp, &cy, nullptr)) {
            Vec3 crot{}, cpos{}, ppos{};
            if (call_ret_vec3(comp, L"K2_GetComponentRotation", &crot) &&
                call_ret_vec3(comp, L"K2_GetComponentLocation", &cpos) &&
                call_ret_vec3(g_rig_parent, L"K2_GetComponentLocation", &ppos)) {
                // ---- CLOSE THE BRACKET (Config.hpp mesh_const). The mesh rotation above was read
                // by a reflected call issued after the ControlRotation read, with two more
                // reflected calls between them, so `cp/cy` and `crot` describe different instants.
                // The mesh rides the camera, so that gap lands in M as the camera's angular
                // velocity times the gap -- exactly the judder, in the one factor every bone we
                // write shares. Read the camera a second time to MEASURE the gap instead of
                // assuming it away.
                double cp1 = cp, cy1 = cy;
                const bool brk_ok = read_control_rotation(&cp1, &cy1, nullptr);
                const float brk_dy = brk_ok ? wrap180((float)(cy1 - cy)) : 0.0f;
                const float brk_dp = brk_ok ? (float)(cp1 - cp) : 0.0f;
                const float brk_deg = std::sqrt(brk_dy * brk_dy + brk_dp * brk_dp);
                halo::g_meshM_brk_deg.store(brk_deg, std::memory_order_relaxed);

                static bool     s_have_M = false;
                static unsigned s_clean  = 0;
                const int  mcm   = g_cfg.mesh_const;
                const bool gated = (mcm == 1 || mcm == 3);
                const bool dirty = gated && brk_ok && brk_deg > g_cfg.mesh_const_gate;
                const bool frozen = (mcm == 3 && s_have_M && s_clean >= 8u);
                const bool accept = !dirty && !frozen;

                // Mode 2 pairs the mesh read with the MIDPOINT of the brackets, the best unbiased
                // estimate of where the camera was when the mesh rotation was actually sampled.
                const float use_p = (mcm == 2 && brk_ok) ? (float)(cp + 0.5 * brk_dp) : (float)cp;
                const float use_y = (mcm == 2 && brk_ok) ? (float)(cy + 0.5 * brk_dy) : (float)cy;
                const Quat cam  = rotator_to_quat(use_p, use_y, 0.0f);
                const Quat camI = quat_conj(cam);
                const Quat M    = quat_mul(camI, rotator_to_quat(crot.x, crot.y, crot.z));
                const Vec3 v0   = quat_rotate(camI, Vec3{cpos.x - ppos.x,
                                                         cpos.y - ppos.y,
                                                         cpos.z - ppos.z});
                halo::g_tick_seq.fetch_add(1u, std::memory_order_acq_rel);   // -> odd, writing
                if (accept) {
                    halo::g_meshM_x = M.x; halo::g_meshM_y = M.y;
                    halo::g_meshM_z = M.z; halo::g_meshM_w = M.w;
                    halo::g_meshV0_x = v0.x; halo::g_meshV0_y = v0.y; halo::g_meshV0_z = v0.z;
                    halo::g_meshM_acc_ms.store((long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count(), std::memory_order_relaxed);
                    halo::g_meshM_acc.fetch_add(1, std::memory_order_relaxed);
                    s_have_M = true;
                    if (!dirty) ++s_clean;
                } else {
                    halo::g_meshM_rej.fetch_add(1, std::memory_order_relaxed);
                }
                // The CURRENT tick's camera and mesh rotation keep publishing every tick either
                // way. They describe this tick and several freshness gates downstream read
                // g_tick_cam_ms; starving them would break modes 8 and 9 rather than fix M.
                halo::g_tick_cam_p.store((float)cp, std::memory_order_relaxed);
                halo::g_tick_cam_y.store((float)cy, std::memory_order_relaxed);
                {
                    const Quat mq = rotator_to_quat(crot.x, crot.y, crot.z);
                    halo::g_tick_mrot_x.store(mq.x, std::memory_order_relaxed);
                    halo::g_tick_mrot_y.store(mq.y, std::memory_order_relaxed);
                    halo::g_tick_mrot_z.store(mq.z, std::memory_order_relaxed);
                    halo::g_tick_mrot_w.store(mq.w, std::memory_order_relaxed);
                }
                halo::g_tick_cam_ms.store((long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count(), std::memory_order_release);
                halo::g_tick_seq.fetch_add(1u, std::memory_order_acq_rel);   // -> even, readable
                // A rejected sample is not a failure: M is a constant and the last clean one still
                // stands. Only "never seeded at all" is invalid.
                ok = s_have_M || (mcm == 0);
                if (mcm == 0) {
                    halo::g_meshM_x = M.x; halo::g_meshM_y = M.y;
                    halo::g_meshM_z = M.z; halo::g_meshM_w = M.w;
                    halo::g_meshV0_x = v0.x; halo::g_meshV0_y = v0.y; halo::g_meshV0_z = v0.z;
                }

                // CONSTANCY IS THE CLAIM -- print the evidence. Worst deviation from the first
                // accepted sample, in degrees and cm, over each window.
                if (g_cfg.palette_weapon_log) {
                    static Quat  s_M0{};  static Vec3 s_v00{};
                    static bool  s_seeded = false;
                    static float s_worst_deg = 0.0f, s_worst_cm = 0.0f;
                    static uint32_t s_last_rep = 0;
                    if (!s_seeded) { s_M0 = M; s_v00 = v0; s_seeded = true; }
                    const float dot = clampf(std::fabs(M.x*s_M0.x + M.y*s_M0.y +
                                                       M.z*s_M0.z + M.w*s_M0.w), 0.0f, 1.0f);
                    const float ddeg = 2.0f * std::acos(dot) * RAD2DEG;
                    const Vec3  dv{v0.x - s_v00.x, v0.y - s_v00.y, v0.z - s_v00.z};
                    const float dcm = std::sqrt(dv.x*dv.x + dv.y*dv.y + dv.z*dv.z);
                    if (ddeg > s_worst_deg) s_worst_deg = ddeg;
                    if (dcm  > s_worst_cm)  s_worst_cm  = dcm;
                    if (tick - s_last_rep >= 320) {
                        s_last_rep = tick;
                        // THE VALUES. A "constant" that is constant and WRONG is invisible to a
                        // drift meter, and M has never once been printed as a value. If M is not
                        // identity, the pullback's conj(cam*M) and the lift's cam do NOT cancel,
                        // and the residual rotates with the camera -- which is what the JUDGE
                        // shows: the gun swinging +-60 cm while the hand moves 4.
                        float mp = 0.0f, my = 0.0f, mr = 0.0f;
                        quat_to_rotator(M.x, M.y, M.z, M.w, &mp, &my, &mr);
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] MESHCONST M=(p%.1f y%.1f r%.1f) v0=(%.1f %.1f %.1f)cm "
                            "comp_rot=(p%.1f y%.1f r%.1f) cam=(p%.1f y%.1f) | drift M %.1f deg v0 %.1f cm"
                            " | mode %d bracket %.3f deg acc %u rej %u",
                            mp, my, mr, v0.x, v0.y, v0.z, crot.x, crot.y, crot.z, cp, cy,
                            s_worst_deg, s_worst_cm, g_cfg.mesh_const, brk_deg,
                            halo::g_meshM_acc.load(std::memory_order_relaxed),
                            halo::g_meshM_rej.load(std::memory_order_relaxed));
                        s_worst_deg = s_worst_cm = 0.0f;
                    }
                }
            }
        }
        halo::g_mesh_const_valid.store(ok, std::memory_order_release);
    }

    // ---- THE AXIS-PROBE SAMPLER (see palette_probe in Config.hpp and the probe in
    // BlamPalette.cpp). Accumulates the rendered weapon's world position per probe phase and, at
    // each cycle boundary, prints the three measured columns: bone axis -> world direction, in
    // cm per palette unit. The player stands still; the probe does the moving.
    if (palette_weapon_mode() && g_cfg.palette_probe) {
        static Vec3     s_sum[6] = {};
        static uint32_t s_cnt[6] = {};
        static int      s_prev_phase = -1;

        const int phase = blam_palette_probe_phase();
        auto* rigc = rig_tracked_component();
        Vec3 sock{};
        const bool sock_ok = (phase >= 0 && rigc != nullptr &&
                              rig_socket_world(rigc, L"PrimaryWeapon", &sock));
        // HEARTBEAT, OUTSIDE every gate -- the two silent sessions happened because the failing
        // gate was also the gate on the evidence. phase -1 = the hook never ran the probe (hook
        // dead, or the held weapon builds under a different slot than the phase clock counts);
        // sock_ok 0 = the readback is failing; counts frozen = the settle/steady gates eat all.
        if (g_cfg.palette_weapon_log) {
            static uint32_t s_hb = 0;
            static Vec3     s_hb_sum[6] = {};   // referenced below; zeroed alias for clarity
            (void)s_hb_sum;
            if ((++s_hb % 160u) == 0u) {
                API::get()->log_info(
                    "[Halo-CampE-UEVR] PALETTEPROBE phase=%d rigc=%d sock_ok=%d",
                    phase, (int)(rigc != nullptr), (int)sock_ok);
            }
        }
        if (sock_ok) {
            // THE RENDER FLICKERS between stock and displaced during odd phases -- the heartbeat
            // proved it (baseline bins 170+, displaced bins 0-3: a steadiness filter rejected the
            // displaced state as motion, because it IS motion, at frame rate). So classify
            // instead of filter: during a displaced phase, the just-completed baseline's mean is
            // the anchor, and only samples further than 25 cm from it -- the displaced mode of
            // the bimodal flicker -- are accumulated. Baselines accumulate everything after the
            // settle. The player's own drift is handled by the anchor being LOCAL to each
            // baseline/displaced pair rather than global to the cycle.
            static uint32_t s_phase_age = 0;
            static Vec3     s_anchor{};
            static bool     s_anchor_ok = false;
            if (phase != s_prev_phase) {
                if ((phase & 1) == 1 && s_prev_phase == phase - 1 &&
                    s_cnt[s_prev_phase] >= 8) {
                    s_anchor = Vec3{s_sum[s_prev_phase].x / s_cnt[s_prev_phase],
                                    s_sum[s_prev_phase].y / s_cnt[s_prev_phase],
                                    s_sum[s_prev_phase].z / s_cnt[s_prev_phase]};
                    s_anchor_ok = true;
                } else if ((phase & 1) == 1) {
                    s_anchor_ok = false;
                }
                s_phase_age = 0;
            } else {
                ++s_phase_age;
            }
            if (s_phase_age > 20) {
                if ((phase & 1) == 0) {
                    s_sum[phase].x += sock.x; s_sum[phase].y += sock.y; s_sum[phase].z += sock.z;
                    s_cnt[phase]++;
                } else if (s_anchor_ok) {
                    const Vec3 d{sock.x - s_anchor.x, sock.y - s_anchor.y, sock.z - s_anchor.z};
                    if ((d.x*d.x + d.y*d.y + d.z*d.z) > 625.0f) {   // >25 cm from baseline
                        s_sum[phase].x += sock.x; s_sum[phase].y += sock.y;
                        s_sum[phase].z += sock.z;
                        s_cnt[phase]++;
                    }
                }
            }
            // HEARTBEAT: where do samples die? Two silent sessions in a row were spent inferring
            // that from the outside; this prints it. phase -1 = the hook is not running the probe
            // at all (wrong slot? hook dead?); counts stuck at 0 with a live phase = the settle or
            // steady gate is eating everything.
            if (g_cfg.palette_weapon_log) {
                static uint32_t s_hb = 0;
                if ((++s_hb % 160u) == 0u) {
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] PALETTEPROBE phase=%d age=%u counts=%u/%u/%u/%u/%u/%u",
                        phase, s_phase_age,
                        s_cnt[0], s_cnt[1], s_cnt[2], s_cnt[3], s_cnt[4], s_cnt[5]);
                }
            }

            // Cycle boundary: phase wrapped from 5 back to 0 with data in every bin.
            if (phase == 0 && s_prev_phase == 5) {
                bool full = true;
                for (int i = 0; i < 6; ++i) full = full && (s_cnt[i] >= 8);
                if (full) {
                    const float amt = blam_palette_probe_amount();
                    Vec3 m[6];
                    for (int i = 0; i < 6; ++i) {
                        m[i] = Vec3{s_sum[i].x / s_cnt[i], s_sum[i].y / s_cnt[i],
                                    s_sum[i].z / s_cnt[i]};
                    }
                    for (int a = 0; a < 3; ++a) {
                        const Vec3 col{(m[2*a+1].x - m[2*a].x) / amt,
                                       (m[2*a+1].y - m[2*a].y) / amt,
                                       (m[2*a+1].z - m[2*a].z) / amt};
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] PALETTEAXIS bone %c -> world (%+7.1f %+7.1f %+7.1f) cm/unit",
                            'X' + a, col.x, col.y, col.z);
                    }
                    // The camera at capture, so the CONSTANT part of the map can be separated
                    // from wherever the player happened to be facing: bridge = conj(cam*M) * B.
                    double pcp = 0.0, pcy = 0.0;
                    read_control_rotation(&pcp, &pcy, nullptr);
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] PALETTEAXIS cam=(p%.1f y%.1f) M=(%.4f %.4f %.4f %.4f)",
                        pcp, pcy,
                        halo::g_meshM_x.load(), halo::g_meshM_y.load(),
                        halo::g_meshM_z.load(), halo::g_meshM_w.load());
                }
                for (int i = 0; i < 6; ++i) { s_sum[i] = Vec3{}; s_cnt[i] = 0; }
            }
            s_prev_phase = phase;
        }
    }
}

void palette_wpn_game_tick_after_gestures(float delta) {
    // PALETTE WEAPON MODE: the fork's two-hand hold, after the gestures (its latch must
    // see the reload/rack state they settled) and before the holsters, as the fork ran it.
    if (palette_weapon_mode()) halo::palette_two_hand_update(delta);
}

void palette_wpn_engine_tick_start() {
    // The game thread's id and the tick phase, for the sim hook's phase instrument (see
    // BlamPalette.hpp): the palette hook tells 'inside the engine tick' from 'from the sim
    // thread' by these two.
    g_game_tid.store((uint32_t)GetCurrentThreadId(), std::memory_order_relaxed);
    halo::g_tick_id.fetch_add(1, std::memory_order_relaxed);
    stomp_sample(0);
    {   // Point 24: the Blam control record at tick start, before this tick's writes.
        float ry = 0.0f, rp = 0.0f;
        if (g_cfg.stomp_log != 0 && halo::blam_ctl_read_ue_deg(&ry, &rp))
            halo::stomp_mark(24, ry, rp, (float)halo::g_tick_id.load(std::memory_order_relaxed),
                             (float)halo::g_latch_gen.load(std::memory_order_relaxed));
    }
    // POSELATCH sites 1/2 (palette weapon mode): the frame's one hand sample, taken before
    // anything this frame reads it and before update() publishes the palette poses.
    halo::pose_latch_refresh(1);
}

// The engine tick's phase, for the sim hook's phase instrument (see BlamPalette.hpp).
void palette_wpn_engine_tick_end() { g_engine_phase.store(1, std::memory_order_relaxed); }
void palette_wpn_post_engine_tick() { g_engine_phase.store(0, std::memory_order_relaxed); }

bool palette_wpn_fp_weapon_live() {
    // The palette build is POSITIVE proof of a rendered first-person weapon -- available
    // even with the rig driver off (rig=0, the palette-weapon configuration), where the
    // rig/route signals read absent and would otherwise park the aim stack in stick mode
    // while standing armed on foot.
    return palette_weapon_mode() && blam_palette_fp_live();
}

void palette_wpn_rig_parent_dropped() {
    g_fpscale_camera = nullptr;   // new parent => the projection-scale fix re-applies
}

bool palette_wpn_rig_driver_stood_down() {
    // Everything below is the RIG DRIVER, and only rigmode wants it. With the palette weapon
    // owning placement, running both would be two systems fighting over one skeleton.
    // palette_weapon_mode() stands the rig driver down even with rig=1: armdriver mode 3 owns placement.
    return palette_weapon_mode();
}

void palette_wpn_stereo_pre_eye_instruments(int index) {
    const auto& ps = host::g_plugin_state;
    const std::atomic<float>& g_dbg_view_out = *ps.dbg_view_out;
    const std::atomic<bool>&  g_stick_mode = *ps.stick_mode;
    // The RENDERED base yaw, published for the palette's frame math (see the declaration).
    halo::g_view_base_yaw.store(g_dbg_view_out.load(), std::memory_order_relaxed);

    // READBACK: the last point before the draw. Does the palette still hold our bytes?
    if (index == 0) halo::blam_palette_readback_probe();
    // ---- SOCKROT (2026-09-12). THE DRAWN WEAPON'S ROTATION, never once measured.
    //
    // Everything upstream is proven clean. READBACK says the palette holds our exact bytes at
    // draw time (0.000 cm, 0.02 deg of float noise). RELSTOCK says the stock node relations
    // are dead constants (per-frame change 0.0000). The publisher is transparent (gain 1.01 in
    // every band). The camera is not the carrier -- palettelocal removed it entirely and the
    // judder survived. So the palette contains precisely what we intend.
    //
    // And yet point 12, the only drawn-result recorder in this project, logs the socket's
    // POSITION only. The complaint is rotational, and a socket sitting on the wrist barely
    // moves in position while its ROTATION swings the muzzle 60 cm away. So the one quantity
    // that corresponds to the symptom has never been recorded. rig_socket_world_rot has
    // existed the whole time and was never called from here.
    if (index == 0 && g_cfg.palette_weapon_log && palette_weapon_mode()) {
        auto* srcomp = rig_tracked_component();
        Vec3 srot{};
        if (srcomp != nullptr && rig_socket_world_rot(srcomp, L"PrimaryWeapon", &srot)) {
            const Quat sq = rotator_to_quat(srot.x, srot.y, srot.z);
            halo::blam_palette_sockrot_probe(sq.x, sq.y, sq.z, sq.w);
            // SAMEINST: component rotation and aim read right beside the socket, same callback,
            // same thread, so nothing published by another thread enters the comparison.
            Vec3 crot2{};
            if (call_ret_vec3(srcomp, L"K2_GetComponentRotation", &crot2)) {
                const Quat cq2 = rotator_to_quat(crot2.x, crot2.y, crot2.z);
                double ap = 0.0, ay = 0.0;
                const bool aok = read_control_rotation_hook(&ap, &ay);
                halo::blam_palette_sameinst_probe(sq.x, sq.y, sq.z, sq.w, cq2.x, cq2.y, cq2.z, cq2.w,
                                                  (float)ap, (float)ay, aok);
            }
        }
    }
    stomp_sample(index == 0 ? 1 : 2);
    if (index == 0) halo::blam_palette_stamp_bank();
    // Point 12: the RENDERED gun, the chain's final output. PrimaryWeapon socket of the
    // posed skeleton, world cm, once per frame. Judder that survives every upstream zero
    // must appear here as a back-and-forth world path -- and if this path is smooth while
    // the headset still shows judder, the defect is beyond the skeleton (view/reprojection).
    if (index == 0 && g_cfg.stomp_log != 0) {
        auto* s12 = rig_tracked_component();
        Vec3 s12p{};
        if (s12 != nullptr && rig_socket_world(s12, L"PrimaryWeapon", &s12p))
            halo::stomp_mark(12, s12p.x, s12p.y, s12p.z,
                             halo::g_view_base_yaw.load(std::memory_order_relaxed));
    }
    // ---- RAYGUN (2026-09-12). From the headset: "check that the aim ray moves / rotates the exact same
    // way as the weapon positioning." Every render frame, same callback, same instant:
    //   point 20  yaw = aim ray yaw (ControlRotation), e0 = aim ray pitch,
    //             e1 = hand intent yaw (desired_aim_now, what both writers are given), e2 = intent pitch
    //   point 21  yaw = drawn barrel yaw (socket -Y), e0 = barrel pitch,
    //             e1 = angle barrel to aim ray (deg), e2 = angle barrel to intent (deg)
    // An overshoot-and-snap-back shows as the barrel (or the aim) running past the intent for
    // one frame and returning, which the per-frame series makes a number instead of a feeling.
    if (index == 0 && g_cfg.stomp_log != 0) {
        double rcp = 0.0, rcy = 0.0;
        float riy = 0.0f, rip = 0.0f;
        const bool r_aim = read_control_rotation_hook(&rcp, &rcy);
        const bool r_int = halo::desired_aim_now(&riy, &rip);
        if (r_aim) halo::stomp_mark(20, (float)rcy, (float)rcp, r_int ? riy : -999.0f, r_int ? rip : -999.0f);
        {   // Point 27: at render, the generation this thread was served and the Blam record.
            float bry = -999.0f, brp = -999.0f;
            halo::blam_ctl_read_ue_deg(&bry, &brp);
            halo::stomp_mark(27, bry, brp, (float)halo::pose_latch_last_gen(),
                             (float)halo::g_tick_id.load(std::memory_order_relaxed));
        }
        auto* rgc = rig_tracked_component();
        Vec3 rgrot{};
        if (rgc != nullptr && rig_socket_world_rot(rgc, L"PrimaryWeapon", &rgrot)) {
            const Quat rq = rotator_to_quat(rgrot.x, rgrot.y, rgrot.z);
            const Vec3 ry = quat_rotate(rq, Vec3{0, 1, 0});
            const Vec3 bd{-ry.x, -ry.y, -ry.z};
            const float by = std::atan2(bd.y, bd.x) * RAD2DEG;
            const float bp = std::asin(clampf(bd.z, -1.0f, 1.0f)) * RAD2DEG;
            auto dir_of = [](float pdeg, float ydeg) {
                const float pr = pdeg * DEG2RAD, yr = ydeg * DEG2RAD;
                return Vec3{std::cos(pr) * std::cos(yr), std::cos(pr) * std::sin(yr), std::sin(pr)};
            };
            auto ang = [](const Vec3& a, const Vec3& b) {
                return std::acos(clampf(a.x * b.x + a.y * b.y + a.z * b.z, -1.0f, 1.0f)) * RAD2DEG;
            };
            const float to_aim = r_aim ? ang(bd, dir_of((float)rcp, (float)rcy)) : -999.0f;
            const float to_int = r_int ? ang(bd, dir_of(rip, riy)) : -999.0f;
            halo::stomp_mark(21, by, bp, to_aim, to_int);
        }
    }
    // ---- FPPIN (Config.hpp fp_pin): the FP mesh is re-anchored on the VIEW frame before
    // the palette refresh reads it back. Pitch is ControlRotation's (the lock owns yaw
    // only); yaw is the one this frame renders. Fails closed on any missing ingredient.
    if (index == 0 && g_cfg.fp_pin != 0 && g_cfg.pal_render == 1 && palette_weapon_mode()
        && !g_stick_mode.load() && halo::g_mesh_const_valid.load(std::memory_order_acquire)) {
        auto* pinc = rig_tracked_component();
        auto* pinp = g_rig_parent;
        double pcp = 0.0, pcy = 0.0;
        Vec3 ppos{};
        if (pinc != nullptr && pinp != nullptr && !IsBadReadPtr(pinp, sizeof(void*))
            && read_control_rotation_hook(&pcp, &pcy)
            && call_ret_vec3(pinp, L"K2_GetComponentLocation", &ppos)) {
            const float vy = halo::g_view_base_yaw.load(std::memory_order_relaxed);
            const Quat pr = rotator_to_quat((float)pcp, vy, 0.0f);
            const Quat pM{halo::g_meshM_x.load(std::memory_order_relaxed), halo::g_meshM_y.load(std::memory_order_relaxed),
                          halo::g_meshM_z.load(std::memory_order_relaxed), halo::g_meshM_w.load(std::memory_order_relaxed)};
            const Quat mrot = quat_mul(pr, pM);
            float mp = 0.0f, my = 0.0f, mr = 0.0f;
            quat_to_rotator(mrot.x, mrot.y, mrot.z, mrot.w, &mp, &my, &mr);
            const Vec3 v0{halo::g_meshV0_x.load(std::memory_order_relaxed), halo::g_meshV0_y.load(std::memory_order_relaxed), halo::g_meshV0_z.load(std::memory_order_relaxed)};
            const Vec3 off = quat_rotate(pr, v0);
            if (std::isfinite(mp) && std::isfinite(my) && std::isfinite(off.x))
                halo::holster_marker_place_rot(pinc, Vec3{ppos.x + off.x, ppos.y + off.y, ppos.z + off.z}, mp, my, mr);
        }
    }
}

void palette_wpn_render_refresh() {
    halo::blam_palette_republish_frame();
    halo::blam_palette_render_refresh();
    halo::blam_palette_wpnerr_frame();
}

void palette_wpn_stereo_pre_eye_meters(int index) {
    const std::atomic<float>& g_dbg_view_out = *host::g_plugin_state.dbg_view_out;
    // ---- FPMESH METER (Config.hpp fpmesh_log): does the FP mesh's own transform step at
    // sim rate under the 90 Hz view? Read-only; the embedded camera is recovered through
    // the measured M constant, exactly the relation the mesh-constant block validates.
    if (index == 0 && g_cfg.fpmesh_log != 0) {
        static float s_fm_prev = 0.0f, s_fm_prev_ctl = 0.0f; static bool s_fm_have = false;
        static int s_fm_fr = 0, s_fm_moved = 0; static float s_fm_sum = 0.0f, s_fm_max = 0.0f, s_fm_csum = 0.0f;
        static ULONGLONG s_fm_said = 0;
        auto* fmc = rig_tracked_component();
        Vec3 fcrot{};
        double fcp = 0.0, fcy = 0.0;
        if (fmc != nullptr && call_ret_vec3(fmc, L"K2_GetComponentRotation", &fcrot) &&
            read_control_rotation_hook(&fcp, &fcy)) {
            const Quat fM{halo::g_meshM_x.load(std::memory_order_relaxed), halo::g_meshM_y.load(std::memory_order_relaxed),
                                halo::g_meshM_z.load(std::memory_order_relaxed), halo::g_meshM_w.load(std::memory_order_relaxed)};
            const Quat fcam = quat_mul(rotator_to_quat(fcrot.x, fcrot.y, fcrot.z), quat_conj(fM));
            float fep = 0.0f, fey = 0.0f, fer = 0.0f;
            quat_to_rotator(fcam.x, fcam.y, fcam.z, fcam.w, &fep, &fey, &fer);
            if (s_fm_have) {
                const float dmesh = std::fabs(wrap180(fey - s_fm_prev));
                const float dctl  = std::fabs(wrap180((float)fcy - s_fm_prev_ctl));
                ++s_fm_fr;
                if (dmesh > 0.01f) ++s_fm_moved;
                s_fm_sum += dmesh; if (dmesh > s_fm_max) s_fm_max = dmesh;
                s_fm_csum += dctl;
            }
            s_fm_prev = fey; s_fm_prev_ctl = (float)fcy; s_fm_have = true;
            const ULONGLONG fnow = GetTickCount64();
            if (s_fm_said == 0) s_fm_said = fnow;
            if (fnow - s_fm_said >= 1000 && s_fm_fr > 0) {
                API::get()->log_info("[Halo-CampE-UEVR] FPMESH frames=%d moved=%d step mean %.3f max %.3f deg | ctl-per-frame mean %.3f | mesh-vs-ctl gap %.2f | mesh-vs-view gap %.2f",
                                     s_fm_fr, s_fm_moved, s_fm_sum / s_fm_fr, s_fm_max, s_fm_csum / s_fm_fr,
                                     wrap180((float)fcy - fey), wrap180(g_dbg_view_out.load() - fey));
                s_fm_fr = s_fm_moved = 0; s_fm_sum = s_fm_max = s_fm_csum = 0.0f; s_fm_said = fnow;
            }
        }
    }
}

void palette_wpn_stereo_post_eye_sample(int index) {
    if (index == 0) stomp_sample(3);
}

void palette_wpn_stereo_post_eye_late(int index) {
    const auto& ps = host::g_plugin_state;
    const std::atomic<bool>&  g_have_eye_pos = *ps.have_eye_pos;
    const std::atomic<float>& g_eye_pos_x = *ps.eye_pos_x;
    const std::atomic<float>& g_eye_pos_y = *ps.eye_pos_y;
    const std::atomic<float>& g_eye_pos_z = *ps.eye_pos_z;
    const std::atomic<float>& g_view_pos_x = *ps.view_pos_x;
    const std::atomic<float>& g_view_pos_y = *ps.view_pos_y;
    const std::atomic<float>& g_view_pos_z = *ps.view_pos_z;
    const std::atomic<float>& g_render_view_yaw = *ps.render_view_yaw;
    // RETPROBE 44: eye position this frame and the view position x published beside it.
    //          45: view position y/z and the finished view yaw, same frame.
    if (index == 0 && g_cfg.stomp_log != 0 && g_have_eye_pos.load()) {
        halo::stomp_mark(44, g_eye_pos_x.load(), g_eye_pos_y.load(), g_eye_pos_z.load(), g_view_pos_x.load());
        halo::stomp_mark(45, g_view_pos_y.load(), g_view_pos_z.load(), g_render_view_yaw.load(), 0.0f);
    }
}

void palette_wpn_teardown() {
    stomp_flush();   // a capture must survive the game closing mid-window
}

void palette_wpn_aim_law_sampling() {
    // POSELATCH site 3: latch at the instant the aim is sampled.
    halo::pose_latch_refresh(3);
}

void palette_wpn_aim_law_sampled(double ay, double ap) {
    // PALETTESYNC (Config.hpp palette_sync): hand and aim sampled together, here, where the
    // aim law samples them, and handed to the placement as one pair.
    if (g_cfg.palette_sync && palette_weapon_mode()) {
        const int32_t sidx = g_aim_law_ridx.load();
        Vec3 sap{}, sgp{}; Quat saq{}, sgq{};
        if (sidx >= 0 && get_pose(sidx, &sap, &saq, /*use_aim=*/true) &&
            get_pose(sidx, &sgp, &sgq, /*use_aim=*/false)) {
            halo::blam_palette_sync_capture(saq.x, saq.y, saq.z, saq.w, sap.x, sap.y, sap.z,
                                            sgq.x, sgq.y, sgq.z, sgq.w, sgp.x, sgp.y, sgp.z,
                                            (float)ap, (float)ay);
        }
    }
}

} // namespace halo
