#include "WeaponDrive.hpp"

#include "Config.hpp"
#include "Rig.hpp"
#include "DevTools.hpp"

#include <atomic>
#include <cmath>
#include <cstdio>

using namespace uevr;

namespace halo {

namespace {

// ---- WHY THERE IS NO LATCH ANY MORE -------------------------------------------------------------
// MEASURED 2026-08-25, in this order, and the order matters because each step refuted the last:
//
//  1. A 50-sample probe showed S constant to 0.1 cm, so the first design LATCHED it once.
//  2. That latched (0,0,0) -- on the frame a weapon actor first appears its transform has not yet
//     moved onto the socket, and the first available reading is not the socket. The formula still
//     ran, the write still landed (readback matched R exactly on every frame), and the gun was
//     simply a socket-length wrong. Nothing failed loudly except the self-check.
//  3. Gating the latch on plausibility + stability then refused to latch AT ALL -- because S is not
//     actually constant. Live it hovers near (24,12.7,-26.3) but excurses ~11 cm.
//
// So the 50-sample probe was taken during a STATIC moment and a stationary reading was generalised
// into a structural property. Rig.hpp had it right all along: weapon_world = mesh_world +
// socket_offset(ANIMATED POSE). The socket rides the animation, and legacy lets the gun ride with
// it -- so reproducing legacy exactly REQUIRES the live value, not a frozen one.
//
// ---- THE FEEDBACK PROBLEM, AND ITS EXACT SOLUTION ------------------------------------------------
// Plugin.cpp measures S as the weapon expressed in the mesh's frame, and notes it "feeds back
// nowhere: both terms move together". True only while the weapon root is unwritten. Once we write R
// the measurement becomes S o R, so feeding it back would compound every frame.
//
// But we KNOW R -- we wrote it. So the true socket is recoverable exactly:
//
//     S_measured = S_true o R_prev        =>     S_true = S_measured o R_prev^-1
//
// This is a correction, not an integrator: we divide out the very transform that produced the
// reading, so it is algebraically exact when R_prev is the R in force at sample time, and degrades
// only by how much S moved in one frame otherwise. It cannot compound. On a fresh weapon actor
// R_prev is identity (nothing has written it), which is why the actor pointer resets it.
constexpr float kSocketMinCm = 2.0f;
constexpr float kSocketMaxCm = 300.0f;

// ---- WHY THERE ARE THREE ENGAGE GUARDS, NOT ONE -------------------------------------------------
// MEASURED 2026-08-25, in a live session, and it cost the player their arms:
//
//   ENGAGED -- arm mesh released to neutral      22:14:02
//   DISENGAGED -- legacy mesh drive resumes      22:14:03      ... 5+ cycles in ~2 minutes
//
// while the log simultaneously read "ON-FOOT (no weapon) ENGAGED" and "wpn: sep=|1193.6|cm". The
// player was UNARMED, and fp_weapon_root() was handing back a STALE POOLED ACTOR twelve metres away
// -- exactly the hazard Rig.hpp names ("actors on this title are pooled -- re-resolve rather than
// caching a pointer"). As that phantom drifted, its distance wandered in and out of the plausibility
// window, so the drive engaged and disengaged repeatedly, and the engage edge zeroes the arm mesh
// EVERY time. A guard meant to stop the arms freezing became the thing that stomped them.
//
// The lesson is that a non-null pointer is not a fact about the world. Each guard below closes a
// hole the others do not:
//
//   1. ARMED      -- the plugin already knows (g_on_foot_unarmed drives the arm hiding). Passed in
//                    rather than read as a global, so this module keeps no opinion about the pawn.
//   2. NEAR       -- a HELD weapon is within arm's reach of the mesh. Twelve metres is not a socket,
//                    and no amount of socket maths makes it one.
//   3. HYSTERESIS -- engaging or disengaging must survive a run of ticks. Flapping is destructive
//                    here in a way a steady wrong answer is not, because the edge does work.
constexpr float kWeaponNearMaxCm = 200.0f;
constexpr int   kEngageTicks     = 30;
constexpr int   kDisengageTicks  = 30;

// ---- SELF-CHECK ---------------------------------------------------------------------------------
// Compares where the gun ACTUALLY is against C o S -- the exact transform legacy produces. The error
// it reports IS the recalibration error in cm, so 0 means a player switching this on notices
// nothing. Anything large means hand the gun back to the path that has always worked.
constexpr float kSelfCheckTripCm  = 20.0f;
// ANGULAR check, added 2026-08-25 after the position check passed at err=0.0cm on every frame while
// the player was looking at a gun pointed the wrong way. The socket carries a 90 degree yaw
// (sock rot=(p-0.0 y90.1 r0.3)), so orientation is not a detail here -- it is most of the answer,
// and a guard that cannot see the reported symptom is not a guard.
constexpr float kSelfCheckTripDeg = 12.0f;

// ---- THE INDEPENDENT CHECK, and why the other two are not enough --------------------------------
// MEASURED 2026-08-26, and this is the failure that matters most: the drive ran away to |S| = 68 cm
// against a true socket of ~37, R swung to (68.9,54.9) and (-92.1,-14.2), the player saw "two
// flashing weapons orbiting a central point" -- and the self-check reported err=0.0cm/0.0deg the
// whole time.
//
// Because it was a TAUTOLOGY. expect = C o S and actual = M o S o R = C o S by construction, so S
// sits on BOTH sides and cancels. A wrong S makes both sides wrong together and the error stays
// zero. That check validates the ALGEBRA, never the TRUTH -- exactly what the comment on the stash
// warned about ("comparing against our own input would pass by construction and prove nothing"),
// which I wrote and then designed around anyway.
//
// g_dbg_Lact_* is the answer and it already existed. Rig.hpp: "WHERE THE GUN ACTUALLY ENDED UP, in
// controller-local cm ... it consumes none of the values we wrote, so it cannot come out right by
// construction the way a check against our own output would." It is built from two live engine reads
// plus the controller pose, and compared against off_x/y/z -- the calibration. No S anywhere.
//
// So THIS is what trips. The S-based numbers stay as diagnostics, because they still localise a
// fault once one is known to exist; they simply cannot be trusted to notice one.
constexpr float kPlaceTripCm = 15.0f;
constexpr int   kSelfCheckTripRun = 30;

// ---- CACHED TARGETS, resolved on the GAME thread ------------------------------------------------
// fp_weapon_root() walks reflection; the render callback is not the place for that. Mirrors how
// g_rig_component is already published for the same callback.
std::atomic<void*> s_weapon_root{nullptr};
std::atomic<void*> s_weapon_actor{nullptr};

// ---- S, recovered live on the GAME thread and published for the render thread -------------------
std::atomic<bool>  s_sock_valid{false};
std::atomic<float> s_sock_px{0.0f}, s_sock_py{0.0f}, s_sock_pz{0.0f};
std::atomic<float> s_sock_qx{0.0f}, s_sock_qy{0.0f}, s_sock_qz{0.0f}, s_sock_qw{1.0f};

// ---- R, published by the RENDER thread so the game thread can divide it back out ----------------
std::atomic<bool>  s_lastr_valid{false};
std::atomic<float> s_lastr_px{0.0f}, s_lastr_py{0.0f}, s_lastr_pz{0.0f};
std::atomic<float> s_lastr_qx{0.0f}, s_lastr_qy{0.0f}, s_lastr_qz{0.0f}, s_lastr_qw{1.0f};

void* s_seen_actor = nullptr;   // game thread only

// Hysteresis. GAME THREAD ONLY except s_engaged, which the render thread reads through owns().
std::atomic<bool> s_engaged{false};
int               s_ok_run  = 0;
int               s_bad_run_gate = 0;

// ---- FAIL-CLOSED LATCH --------------------------------------------------------------------------
std::atomic<bool> s_tripped{false};
std::atomic<int>  s_bad_run{0};
std::atomic<int>  s_trip_count{0};

// ---- SELF-CHECK STATE ---------------------------------------------------------------------------
// The write we make this frame is only observable NEXT frame, so the expected pose is stashed and
// compared against a fresh engine read on the following call. Comparing against our own input in the
// same frame would pass by construction and prove nothing.
std::atomic<bool>  s_expect_valid{false};
std::atomic<float> s_expect_x{0.0f}, s_expect_y{0.0f}, s_expect_z{0.0f};
std::atomic<float> s_err_cm{-1.0f};
std::atomic<float> s_err_deg{-1.0f};
std::atomic<float> s_place_cm{-1.0f};
std::atomic<bool>  s_expect_rot_valid{false};
std::atomic<float> s_exp_qx{0.0f}, s_exp_qy{0.0f}, s_exp_qz{0.0f}, s_exp_qw{1.0f};

char s_status[256] = "weapon drive: off";

bool finite3(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool finite_q(const Quat& q) {
    return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

} // namespace

bool weapon_drive_enabled() {
    return g_cfg.wpn_drive;
}

bool weapon_drive_owns() {
    // Every condition is a reason the LEGACY path must keep the gun: mode off, no usable socket this
    // frame, no target resolved, or the self-check has tripped. Defaulting to legacy on all of them
    // is what makes this safe to leave switched on.
    // s_sock_valid is DELIBERATELY NOT HERE. It flips per tick, and owns() edges do destructive
    // work (the engage edge releases the arm mesh to neutral) -- so keying owns() off a per-tick
    // signal made it thrash even though s_engaged was debounced. Measured 2026-08-25: the player
    // reported "it thrashed before settling" while the log showed ENGAGED=1/DISENGAGED=1, because
    // the flapping was happening through THIS term, not through the one I had guarded.
    //
    // s_engaged already requires kEngageTicks of consecutive good ticks and kDisengageTicks of bad
    // ones, so a momentary invalid socket just means apply() reuses the last good S for a frame.
    return g_cfg.wpn_drive
        && s_engaged.load()
        && !s_tripped.load()
        && s_weapon_root.load() != nullptr;
}

// GIVE THE GUN BACK. Zero the weapon root's relative transform so legacy composes on a clean
// child, exactly as it did before we ever touched it.
//
// MEASURED 2026-08-25, and it was the player who found it: "the gun corrected when I switched
// weapons, fixed when I switched back too". A weapon swap spawns a NEW actor whose relative
// transform is identity -- which is the only thing that was clearing our leftover R. On disengage
// or trip we simply stopped writing, so the LAST R we wrote stayed on the gun forever and legacy
// composed on top of it. The gun stayed wrong while the log cheerfully read "TRIPPED -> legacy
// mesh drive", so the fallback was never as clean as it claimed.
//
// The engage edge already neutralises the arm mesh on the way IN. This is the missing other half:
// borrow a transform, hand it back.
void weapon_drive_release() {
    auto* root = reinterpret_cast<API::UObject*>(s_weapon_root.load());
    if (root == nullptr) return;
    rig_set_rotation(root, 0.0, 0.0, 0.0);
    rig_set_location(root, 0.0, 0.0, 0.0);
    s_lastr_valid = false;   // the root is identity again, so nothing is left to divide out
}

void weapon_drive_reset(const char* why) {
    weapon_drive_release();
    s_engaged      = false;
    s_ok_run       = 0;
    s_bad_run_gate = 0;
    s_sock_valid   = false;
    s_lastr_valid  = false;
    s_seen_actor   = nullptr;
    s_expect_valid = false;
    s_bad_run      = 0;
    s_tripped      = false;
    s_err_cm       = -1.0f;
    s_weapon_root  = nullptr;
    s_weapon_actor = nullptr;
    if (why != nullptr && g_cfg.wpn_drive) {
        API::get()->log_info("[Halo-CampE-UEVR] weapon drive: reset (%s)", why);
    }
}

// GAME THREAD. Resolve the targets and recover the true socket for this tick.
void weapon_drive_tick(bool player_unarmed) {
    if (!g_cfg.wpn_drive) {
        if (s_sock_valid.load() || s_weapon_root.load() != nullptr) weapon_drive_reset(nullptr);
        return;
    }

    auto* actor = fp_weapon_actor();
    auto* root  = fp_weapon_root();
    s_weapon_actor.store(actor);
    s_weapon_root.store(root);

    // GUARD 1: ARMED. A resolved pointer is not a held weapon -- unarmed, this hands back a stale
    // pooled actor. Treat it exactly as "no weapon", which is what it is.
    if (player_unarmed) { actor = nullptr; root = nullptr; }

    // GUARD 2: NEAR. The held weapon sits within arm's reach of the mesh. This uses the separation
    // Plugin.cpp already measures, so it costs nothing and cannot disagree with the socket maths.
    if (actor != nullptr) {
        const float sx = g_dbg_wpn_dx.load(), sy = g_dbg_wpn_dy.load(), sz = g_dbg_wpn_dz.load();
        const float sep = std::sqrt(sx * sx + sy * sy + sz * sz);
        if (!g_dbg_wpn_ok.load() || !std::isfinite(sep) || sep > kWeaponNearMaxCm) {
            actor = nullptr; root = nullptr;
        }
    }

    if (actor == nullptr || root == nullptr) {
        s_sock_valid   = false;
        s_lastr_valid  = false;
        s_seen_actor   = nullptr;
        s_expect_valid = false;
        s_ok_run       = 0;
        if (s_engaged.load() && ++s_bad_run_gate >= kDisengageTicks) {
            s_engaged      = false;
            s_bad_run_gate = 0;
        }
        return;
    }

    // A NEW ACTOR HAS AN UNWRITTEN ROOT. Rig.hpp: the gun's offset resets on every weapon swap
    // because each weapon is a new actor -- so R_prev is identity and the raw measurement is the
    // socket. Forgetting to reset this would divide out an R that was never applied to this weapon.
    if (actor != s_seen_actor) {
        // A SWAP IS A FRESH START, NOT A CONTINUATION.
        //
        // MEASURED 2026-08-25: the drive was running at 0.1 cm / 0.4 deg, the player switched
        // weapons, and it instantly read 180.0 deg and tripped for good. The socket measurement
        // still describes the OLD weapon for a tick after the actor changes -- the same lag that
        // produced the original (0,0,0) latch -- so S is momentarily garbage. Driving through that
        // window writes a flipped gun, and because the trip is sticky, one bad frame disabled the
        // mode permanently.
        //
        // So a swap drops the engagement and makes the drive re-earn its kEngageTicks against the
        // NEW weapon's socket, and clears the trip: a fresh actor deserves a fresh chance. A genuine
        // fault simply trips again, and the trip COUNT still records that it happened.
        s_seen_actor       = actor;
        s_engaged          = false;
        s_ok_run           = 0;
        s_bad_run_gate     = 0;
        s_lastr_valid      = false;
        s_expect_valid     = false;
        s_expect_rot_valid = false;
        s_bad_run          = 0;
        s_tripped          = false;
        API::get()->log_info("[Halo-CampE-UEVR] weapon drive: new weapon actor -- "
                             "re-earning engagement against the new socket");
    }

    if (!g_dbg_sock_ok.load()) { s_sock_valid = false; s_ok_run = 0; return; }

    const Vec3 p_meas{g_dbg_sock_x.load(), g_dbg_sock_y.load(), g_dbg_sock_z.load()};
    const Quat q_meas = rotator_to_quat(g_dbg_sock_p.load(), g_dbg_sock_yw.load(),
                                        g_dbg_sock_r.load());
    if (!finite3(p_meas) || !finite_q(q_meas)) { s_sock_valid = false; return; }

    // ---- S_true = S_measured o R_prev^-1
    //
    // R_prev IS READ BACK FROM THE COMPONENT, not taken from what we published.
    //
    // MEASURED 2026-08-26: publishing R from the render thread and dividing it out on the game
    // thread is a RACE, and it showed up as R oscillating between two states every frame --
    // roll alternating about -28 deg and +4 deg -- with |S| wandering 34.8 -> 41.3 -> 67.1 when the
    // true socket is ~37, and the angular error landing on an exact 180.0 deg at its worst. The
    // division is only exact when R_prev is the R IN FORCE AT SAMPLE TIME; the render thread writes
    // twice per frame, so the published value belongs to a different frame than the measurement.
    //
    // The component's own RelativeLocation/RelativeRotation is that value by construction: it is
    // what the engine composed to produce the very measurement we are dividing. Reading it here --
    // same thread, same tick as the socket sample in Plugin.cpp immediately above -- makes the two
    // terms consistent by definition rather than by timing luck. My own header said R_prev must be
    // "the R in force at sample time" and I then sourced it from somewhere that could not guarantee
    // that.
    Vec3 p_true = p_meas;
    Quat q_true = q_meas;
    const double* rl_prev = root->get_property_data<double>(L"RelativeLocation");
    const double* rr_prev = root->get_property_data<double>(L"RelativeRotation");
    if (rl_prev != nullptr && rr_prev != nullptr) {
        const Quat q_r = rotator_to_quat((float)rr_prev[0], (float)rr_prev[1], (float)rr_prev[2]);
        const Vec3 p_r{(float)rl_prev[0], (float)rl_prev[1], (float)rl_prev[2]};

        const Quat q_r_inv = quat_conj(q_r);
        const Vec3 p_r_inv = quat_rotate(q_r_inv, Vec3{-p_r.x, -p_r.y, -p_r.z});

        q_true = quat_mul(q_meas, q_r_inv);
        const Vec3 rot_pri = quat_rotate(q_meas, p_r_inv);
        p_true = Vec3{p_meas.x + rot_pri.x, p_meas.y + rot_pri.y, p_meas.z + rot_pri.z};
    }

    // Plausibility only -- NOT stability. S legitimately moves with the animation, so demanding it
    // hold still is what refused to engage at all; this catches the settling zero and nothing else.
    const float mag = std::sqrt(p_true.x * p_true.x + p_true.y * p_true.y + p_true.z * p_true.z);
    if (!finite3(p_true) || !finite_q(q_true) || mag < kSocketMinCm || mag > kSocketMaxCm) {
        // NOTE: the published S is left ALONE -- apply() reuses the last good one until the
        // debounce actually disengages. Clearing it here is what used to yank the drive mid-frame.
        s_sock_valid = false;
        s_ok_run     = 0;
        if (s_engaged.load() && ++s_bad_run_gate >= kDisengageTicks) {
            s_engaged      = false;
            s_bad_run_gate = 0;
        }
        return;
    }

    s_sock_px = p_true.x; s_sock_py = p_true.y; s_sock_pz = p_true.z;
    s_sock_qx = q_true.x; s_sock_qy = q_true.y; s_sock_qz = q_true.z; s_sock_qw = q_true.w;
    s_sock_valid = true;

    // GUARD 3: HYSTERESIS. Engaging does destructive work (the mesh is released to neutral), so it
    // must survive a run of good ticks rather than a single lucky one.
    s_bad_run_gate = 0;
    if (!s_engaged.load() && ++s_ok_run >= kEngageTicks) {
        s_engaged = true;
        s_ok_run  = 0;
    }
}

// RENDER THREAD. Math and writes only -- no reflection walks.
void weapon_drive_apply(const Quat& c_rot, const Vec3& c_loc) {
    if (!weapon_drive_owns()) return;

    auto* root  = reinterpret_cast<API::UObject*>(s_weapon_root.load());
    auto* actor = reinterpret_cast<API::UObject*>(s_weapon_actor.load());
    auto* rig   = reinterpret_cast<API::UObject*>(g_rig_component.load());
    if (root == nullptr || rig == nullptr) return;
    if (!finite_q(c_rot) || !finite3(c_loc)) return;

    // ---- M: where the (now undriven) mesh actually is, live.
    Vec3 m_pos{}, m_rot{};
    if (!call_ret_vec3(rig, L"K2_GetComponentLocation", &m_pos)) return;
    if (!call_ret_vec3(rig, L"K2_GetComponentRotation", &m_rot)) return;
    if (!finite3(m_pos) || !finite3(m_rot)) return;
    const Quat q_m = rotator_to_quat(m_rot.x, m_rot.y, m_rot.z);

    // ---- SELF-CHECK, IN THE MESH'S OWN FRAME.
    //
    // MEASURED 2026-08-25: the first version stashed the expected WORLD position at frame N and
    // compared it against a live read at frame N+1. The reported error then came out EXACTLY equal
    // to |dM| between those frames -- 10.8/16.5/6.3 cm against 10.8/16.5/6.3 cm of rig motion, to
    // 0.1 cm. It was charging one frame of the player's own movement as placement error, because
    // the weapon is attached and travels with the mesh in between.
    //
    // Expressing both sides in the mesh's frame cancels that rigid motion, so what is left is the
    // thing we actually want to know: how far the gun is from where LEGACY would have put it. A
    // metric that moves when the player walks is not measuring placement.
    if (s_expect_valid.load() && actor != nullptr) {
        Vec3 got{};
        if (call_ret_vec3(actor, L"K2_GetActorLocation", &got) && finite3(got)) {
            const Quat q_m_inv_chk = quat_conj(q_m);
            const Vec3 got_local = quat_rotate(q_m_inv_chk, Vec3{got.x - m_pos.x,
                                                                 got.y - m_pos.y,
                                                                 got.z - m_pos.z});
            const float dx = got_local.x - s_expect_x.load();
            const float dy = got_local.y - s_expect_y.load();
            const float dz = got_local.z - s_expect_z.load();
            const float err = std::sqrt(dx * dx + dy * dy + dz * dz);
            s_err_cm = err;

            // ANGULAR half. The socket carries a 90 degree yaw here, so orientation is most of the
            // answer -- and the position check reads 0.0 cm on a gun pointed at the ground. Compared
            // in the mesh's frame like the position half, so aiming does not register as error.
            float ang = 0.0f;
            Vec3 got_rot{};
            if (s_expect_rot_valid.load()
                && call_ret_vec3(actor, L"K2_GetActorRotation", &got_rot) && finite3(got_rot)) {
                const Quat q_got_local = quat_mul(q_m_inv_chk,
                                                  rotator_to_quat(got_rot.x, got_rot.y, got_rot.z));
                const Quat q_exp_local{s_exp_qx.load(), s_exp_qy.load(),
                                       s_exp_qz.load(), s_exp_qw.load()};
                float d = q_got_local.x * q_exp_local.x + q_got_local.y * q_exp_local.y
                        + q_got_local.z * q_exp_local.z + q_got_local.w * q_exp_local.w;
                d = std::fabs(d);
                if (d > 1.0f) d = 1.0f;
                ang = 2.0f * std::acos(d) * 57.2957795f;
                s_err_deg = ang;
            }

            // THE TRIP CRITERION IS THE INDEPENDENT ONE. err/ang cannot see a drifting S.
            float place_err = -1.0f;
            if (g_dbg_Lact_ok.load()) {
                const float px = g_dbg_Lact_x.load() - g_cfg.off_x;
                const float py = g_dbg_Lact_y.load() - g_cfg.off_y;
                const float pz = g_dbg_Lact_z.load() - g_cfg.off_z;
                place_err = std::sqrt(px * px + py * py + pz * pz);
                s_place_cm = place_err;
            }

            if (place_err > kPlaceTripCm || err > kSelfCheckTripCm || ang > kSelfCheckTripDeg) {
                const int run = s_bad_run.fetch_add(1) + 1;
                if (run >= kSelfCheckTripRun && !s_tripped.exchange(true)) {
                    s_trip_count.fetch_add(1);
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] weapon drive: SELF-CHECK TRIPPED -- placement %.1f cm off "
                        "the calibration (independent), self %.1f cm / %.1f deg, for %d frames. "
                        "Handing the weapon back to the mesh drive. Calibration is UNCHANGED; set "
                        "wpndrive=0 to silence.",
                        place_err, err, ang, run);
                }
            } else {
                s_bad_run = 0;
            }
        }
    }

    // ---- S: recovered live on the game thread, with our own R divided back out.
    const Vec3 s_pos{s_sock_px.load(), s_sock_py.load(), s_sock_pz.load()};
    const Quat q_s{s_sock_qx.load(), s_sock_qy.load(), s_sock_qz.load(), s_sock_qw.load()};

    // ---- R = S^-1 o M^-1 o C o S
    const Quat q_m_inv = quat_conj(q_m);
    const Quat q_a     = quat_mul(q_m_inv, c_rot);
    const Vec3 p_a     = quat_rotate(q_m_inv, Vec3{c_loc.x - m_pos.x,
                                                   c_loc.y - m_pos.y,
                                                   c_loc.z - m_pos.z});

    const Quat q_b = quat_mul(q_a, q_s);
    const Vec3 rs  = quat_rotate(q_a, s_pos);
    const Vec3 p_b{p_a.x + rs.x, p_a.y + rs.y, p_a.z + rs.z};

    const Quat q_s_inv = quat_conj(q_s);
    const Quat q_r     = quat_mul(q_s_inv, q_b);
    const Vec3 p_r     = quat_rotate(q_s_inv, Vec3{p_b.x - s_pos.x,
                                                   p_b.y - s_pos.y,
                                                   p_b.z - s_pos.z});

    float rp = 0.0f, ry = 0.0f, rr = 0.0f;
    quat_to_rotator(q_r.x, q_r.y, q_r.z, q_r.w, &rp, &ry, &rr);
    if (!finite3(p_r) || !std::isfinite(rp) || !std::isfinite(ry) || !std::isfinite(rr)) return;

    // W = C o S, the legacy result. Computed before the write so the diagnostic can report what we
    // aimed at alongside what the component actually holds.
    const Vec3 cs = quat_rotate(c_rot, s_pos);

    rig_set_rotation(root, (double)rp, (double)ry, (double)rr);
    rig_set_location(root, (double)p_r.x, (double)p_r.y, (double)p_r.z);

    // PUBLISH R so the next tick can divide it out of its measurement. Must happen on every write,
    // including ones the self-check later rejects -- the measurement reflects what we WROTE, not
    // what we wish we had written.
    s_lastr_px = p_r.x; s_lastr_py = p_r.y; s_lastr_pz = p_r.z;
    s_lastr_qx = q_r.x; s_lastr_qy = q_r.y; s_lastr_qz = q_r.z; s_lastr_qw = q_r.w;
    s_lastr_valid = true;

#if HALO_VR_DEV
    {
        static int s_frames = 0;
        const int n = s_frames++;
        if (n < 12 || (n % 120) == 0) {
            auto* rl = root->get_property_data<double>(L"RelativeLocation");
            API::get()->log_info(
                "[Halo-CampE-UEVR] WPNDRIVE[%d] M=(%.1f,%.1f,%.1f) S=(%.1f,%.1f,%.1f) |S|=%.1f "
                "C=(%.1f,%.1f,%.1f) R=(%.1f,%.1f,%.1f) readback=(%.1f,%.1f,%.1f) "
                "expect=(%.1f,%.1f,%.1f) err=%.1fcm/%.1fdeg",
                n, m_pos.x, m_pos.y, m_pos.z, s_pos.x, s_pos.y, s_pos.z,
                std::sqrt(s_pos.x * s_pos.x + s_pos.y * s_pos.y + s_pos.z * s_pos.z),
                c_loc.x, c_loc.y, c_loc.z, p_r.x, p_r.y, p_r.z,
                rl != nullptr ? (float)rl[0] : -9999.0f,
                rl != nullptr ? (float)rl[1] : -9999.0f,
                rl != nullptr ? (float)rl[2] : -9999.0f,
                c_loc.x + cs.x, c_loc.y + cs.y, c_loc.z + cs.z,
                s_err_cm.load(), s_err_deg.load());
        }
    }
#endif

    // Stash W = C o S expressed in the MESH's frame, matching how the check above reads it back.
    const Vec3 expect_local = quat_rotate(q_m_inv, Vec3{c_loc.x + cs.x - m_pos.x,
                                                        c_loc.y + cs.y - m_pos.y,
                                                        c_loc.z + cs.z - m_pos.z});
    s_expect_x = expect_local.x;
    s_expect_y = expect_local.y;
    s_expect_z = expect_local.z;
    s_expect_valid = true;

    // W = C o S rotation, in the mesh's frame, matching how the check reads it back.
    const Quat q_exp_local = quat_mul(q_m_inv, quat_mul(c_rot, q_s));
    s_exp_qx = q_exp_local.x; s_exp_qy = q_exp_local.y;
    s_exp_qz = q_exp_local.z; s_exp_qw = q_exp_local.w;
    s_expect_rot_valid = true;
}

const char* weapon_drive_status() {
    if (!g_cfg.wpn_drive) {
        std::snprintf(s_status, sizeof(s_status), "weapon drive: off (wpndrive=0)");
    } else if (s_tripped.load()) {
        std::snprintf(s_status, sizeof(s_status),
                      "weapon drive: TRIPPED -> legacy mesh drive "
                      "(err %.1f cm / %.1f deg, %d trips)",
                      s_err_cm.load(), s_err_deg.load(), s_trip_count.load());
    } else if (!s_sock_valid.load()) {
        std::snprintf(s_status, sizeof(s_status),
                      "weapon drive: no usable socket this tick (weapon=%s)",
                      s_weapon_root.load() != nullptr ? "yes" : "none");
    } else {
        const float sx = s_sock_px.load(), sy = s_sock_py.load(), sz = s_sock_pz.load();
        std::snprintf(s_status, sizeof(s_status),
                      "weapon drive: DRIVING S=(%.1f,%.1f,%.1f) |S|=%.1fcm place=%.1fcm selfcheck=%.2fcm/%.1fdeg trips=%d",
                      sx, sy, sz, std::sqrt(sx * sx + sy * sy + sz * sz),
                      s_place_cm.load(), s_err_cm.load(), s_err_deg.load(),
                      s_trip_count.load());
    }
    return s_status;
}

} // namespace halo
