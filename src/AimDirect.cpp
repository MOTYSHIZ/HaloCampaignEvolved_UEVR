// Direct aim assignment. See AimDirect.hpp for what this replaces and why it is opt-in.

#include "AimDirect.hpp"
#include "Config.hpp"
#include "MotionAimControl.hpp"
#include "addrcascade/AddressCascade.hpp"
#include "uevr/API.hpp"

#include <Windows.h>
#include <TlHelp32.h>
#include <atomic>
#include <cmath>
#include <cstdint>

using namespace uevr;

namespace halo {
namespace {

enum class Stage { Idle, WatchControlRotation, WatchL1, WatchL2, WatchQuatSrc, Ready, Failed };

// Our own module's address range, so the handler can ignore OUR writes to the watched address.
// Stage 3 watches a location this plugin also writes; without this the first hit caught would
// reliably be aim_direct_set itself (it was writer #0 in the live capture) and the game's writer
// would never win the race.
uintptr_t g_self_lo = 0, g_self_hi = 0;

void init_self_range() {
    if (g_self_lo != 0) return;
    // Was a hand-rolled VirtualQuery + DOS/NT walk; the header validation it lacked (it dereferenced
    // e_lfanew before checking anything was readable) now comes free from the shared helper.
    HMODULE self = nullptr;
    if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS
                            | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                            (LPCSTR)&init_self_range, &self) || self == nullptr) return;
    addrcascade::ModuleRange r{};
    if (!addrcascade::module_range((void*)self, &r)) return;
    g_self_lo = r.base;
    g_self_hi = r.end;
}

Stage                  g_stage = Stage::Idle;
std::atomic<uintptr_t> g_target{0};       // the authoritative rotator (L2)
std::atomic<uintptr_t> g_caught_src{0};   // source pointer (rbx) from the most recent trap
std::atomic<uintptr_t> g_caught_rax{0};   // rax at the trap -- stage 3's (rejected) candidate
std::atomic<uintptr_t> g_caught_rcx{0};   // rcx at the trap -- stage 4's quaternion SOURCE
std::atomic<uintptr_t> g_quat_src{0};     // [obj+0x1D0]: what the sync function copies FROM
std::atomic<bool>      g_watching{false};
PVOID                  g_veh = nullptr;
uintptr_t              g_watch_addr = 0;
void*                  g_known_pc = nullptr;
int                    g_stage_ticks = 0;
int                    g_attempts = 0;
double                 g_last_yaw = 0.0, g_last_pitch = 0.0;
bool                   g_have_last = false;
double                 g_stage_motion = 0.0;   // total aim movement seen during the current stage
uintptr_t              g_l2 = 0;               // stage-2 result, kept as the fallback target

// PATCH-SURVIVAL DIAGNOSTICS for the writer hunt (2026-08-18).
//
// That day's game patch moved the orientation getter 0x10 bytes -- BlamDrive's signature caught it
// and carried on -- but it also changed this chain's codegen: every candidate the watchpoint caught
// was a STACK address, so the rotator we followed was a temporary holding garbage by the time the
// game thread read it. Field log: "neither rax 0x384337DFC8 nor L2 0x384337E420 matches the live
// aim", four times, then give up -- and the user got the stick loop.
//
// A persistent aim mirror CANNOT live on a stack: the frame is reused on the next call. So a stack
// candidate is never the thing we want. These record what the handler actually saw, so one live
// session says whether a non-stack writer exists to catch at all rather than us guessing.
std::atomic<uint32_t>  g_cand_stack{0};    // candidates rejected for being on the writing stack
std::atomic<uint32_t>  g_cand_heap{0};     // candidates that were NOT on a stack
std::atomic<uintptr_t> g_cand_last_stack{0};
std::atomic<uintptr_t> g_cand_last_heap{0};

// THE AUTHORITATIVE AIM IS A QUATERNION, and it lives 0x20 BELOW L2 in the same struct.
//
// exe+0x36297B0 is not a getter -- it reads four doubles from [rcx+0x00/08/10/18] and multiply-adds
// them into a rotator, i.e. it converts a quaternion to Euler. The caller then stamps that result
// into L2. So every Euler in the chain (L2, L1, ControlRotation) is derived, which is why writing
// any of them is undone whenever the game recomputes -- the periodic revert, and the snap on firing.
//
// Verified on a live struct: the four doubles at the base had norm 1.0008, and 2*acos(w) came to
// 47.6 deg about a mostly-Y axis against L2's pitch of -47.4 deg.
std::atomic<uintptr_t> g_quat{0};

// UE's FRotator::Quaternion(). Roll is taken as zero: the aim ray has none to give, and the game
// owns weapon cant.
void quat_from_pitch_yaw(double pitch_deg, double yaw_deg, double out[4]) {
    constexpr double kDegToHalfRad = 3.14159265358979323846 / 360.0;   // deg -> rad, then halved
    const double p = pitch_deg * kDegToHalfRad;
    const double y = yaw_deg   * kDegToHalfRad;
    const double sp = std::sin(p), cp = std::cos(p);
    const double sy = std::sin(y), cy = std::cos(y);
    out[0] =  sp * sy;   // x
    out[1] = -sp * cy;   // y
    out[2] =  cp * sy;   // z
    out[3] =  cp * cy;   // w
}
bool                   g_waiting_logged = false;

// ---- FAST RE-ARM ------------------------------------------------------------------------------
// Locating the rotator costs 5-10 s of stick-loop jitter after every level load, and the cost is
// not computation: each stage has to WAIT for the player to move, because the writer only runs when
// the aim changes. Meanwhile the aim falls back to the rate actuator, which is the jitter.
//
// The addresses are heap and are reallocated, but the target's offset FROM THE PLAYERCONTROLLER may
// well be fixed. So remember that offset and, on the next arm, try it directly.
//
// This is a HINT, never a shortcut past validation. A wrong target means writing into unrelated
// memory every frame, so the candidate is accepted only if it reads as a rotator AND still matches
// the real aim after the aim has MOVED -- which is the same discrimination the watch performs,
// in two ticks instead of three stages. Any failure falls through to the watch and costs nothing
// but a few milliseconds.
//
// Deliberately NOT cleared by aim_direct_invalidate(): surviving the invalidate is the whole point.
// It is a per-session memory, not persisted -- the offset is only assumed stable within one run.
bool      g_hint_valid      = false;
ptrdiff_t g_hint_target_off = 0;
int       g_hint_stage      = 0;      // 0 = not yet matched, 1 = matched once, awaiting aim motion
double    g_hint_ref_yaw    = 0.0;
bool      g_hint_miss_logged = false;

constexpr int    MAX_ATTEMPTS  = 5;    // a locate that keeps failing must give up, not retry forever
constexpr int    STAGE_TICKS   = 12;   // ~24 s per stage: long enough to span a player standing still
constexpr double MOTION_DEG    = 0.05; // aim change that counts as "the writer had a reason to run"

// Dr7 for slot 0: L0 enable, RW0=01 (write), LEN0=10 (8 bytes).
uint64_t dr7_write8() { return 1ull | (0b01ull << 16) | (0b10ull << 18); }

// Debug registers are PER-THREAD, and the thread that matters is the one calling this -- the game
// thread stamps the aim. Arming everything except the caller (the obvious way to avoid suspending
// yourself) misses the only writer that exists.
void arm_all(uintptr_t addr) {
    const DWORD pid = GetCurrentProcessId();
    const DWORD self = GetCurrentThreadId();

    auto set_on = [addr](HANDLE th, bool is_self) {
        if (!is_self && SuspendThread(th) == (DWORD)-1) return;
        CONTEXT c{};
        c.ContextFlags = CONTEXT_DEBUG_REGISTERS;
        if (GetThreadContext(th, &c)) {
            c.Dr0 = addr;
            c.Dr7 = addr ? dr7_write8() : 0;
            c.Dr6 = 0;
            c.ContextFlags = CONTEXT_DEBUG_REGISTERS;
            SetThreadContext(th, &c);
        }
        if (!is_self) ResumeThread(th);
    };

    set_on(GetCurrentThread(), true);

    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    if (snap == INVALID_HANDLE_VALUE) return;
    THREADENTRY32 te{}; te.dwSize = sizeof(te);
    if (Thread32First(snap, &te)) {
        do {
            if (te.th32OwnerProcessID != pid || te.th32ThreadID == self) continue;
            HANDLE th = OpenThread(THREAD_GET_CONTEXT | THREAD_SET_CONTEXT | THREAD_SUSPEND_RESUME,
                                   FALSE, te.th32ThreadID);
            if (th == nullptr) continue;
            set_on(th, false);
            CloseHandle(th);
        } while (Thread32Next(snap, &te));
    }
    CloseHandle(snap);
}

LONG CALLBACK veh(EXCEPTION_POINTERS* ep) {
    if (ep == nullptr || ep->ExceptionRecord == nullptr) return EXCEPTION_CONTINUE_SEARCH;
    if (ep->ExceptionRecord->ExceptionCode != EXCEPTION_SINGLE_STEP) return EXCEPTION_CONTINUE_SEARCH;
    if (!g_watching.load(std::memory_order_relaxed)) return EXCEPTION_CONTINUE_SEARCH;
    if ((ep->ContextRecord->Dr6 & 0x1ull) == 0) return EXCEPTION_CONTINUE_SEARCH;   // not our slot
    ep->ContextRecord->Dr6 = 0;

    // Both writers in the chain copy the rotator FROM a pointer held in rbx, so a hit is only
    // interesting when rbx is a canonical, aligned address. The watched bytes live inside a live
    // object, so allocator fill and memcpy hit them too -- unfiltered, that noise wins the race and
    // the real writer is never recorded. Register test only: dereferencing inside a vectored
    // handler risks a nested fault.
    // IGNORE OUR OWN WRITES. Stage 3 watches an address aim_direct_set also writes, so without
    // this the plugin would catch itself and report its own store as the game's.
    const uintptr_t rip = (uintptr_t)ep->ContextRecord->Rip;
    if (g_self_lo != 0 && rip >= g_self_lo && rip < g_self_hi) return EXCEPTION_CONTINUE_EXECUTION;

    const uintptr_t rbx = (uintptr_t)ep->ContextRecord->Rbx;
    if (rbx >= 0x10000ull && rbx < 0x7FFFFFFFFFFFull && (rbx & 7ull) == 0) {
        // REJECT THE WRITING THREAD'S OWN STACK. gs:[0x08] and gs:[0x10] are this thread's TEB
        // StackBase and StackLimit, so this costs two register reads and NO dereference -- which is
        // what makes it safe here, where a nested fault would be fatal.
        //
        // A rotator living in a stack frame is a temporary by definition, since the frame is reused
        // on the next call. Following one is how the 2026-08-18 build burned all five attempts on
        // addresses that read as garbage by the time the game thread checked them.
        const uintptr_t stack_hi = __readgsqword(0x08);
        const uintptr_t stack_lo = __readgsqword(0x10);
        if (stack_lo != 0 && stack_hi > stack_lo && rbx >= stack_lo && rbx < stack_hi) {
            g_cand_stack.fetch_add(1, std::memory_order_relaxed);
            g_cand_last_stack.store(rbx, std::memory_order_relaxed);
            return EXCEPTION_CONTINUE_EXECUTION;
        }
        g_cand_heap.fetch_add(1, std::memory_order_relaxed);
        g_cand_last_heap.store(rbx, std::memory_order_relaxed);

        uintptr_t expected = 0;
        if (g_caught_src.compare_exchange_strong(expected, rbx)) {
            g_caught_rcx.store((uintptr_t)ep->ContextRecord->Rcx);
            // Captured with the same hit, not a later one: the game's stamp of L2 reads its source
            // through rax (`vmovups xmm0,[rax]` immediately before `vmovups [rbx+0x20],xmm0`), so
            // rax and rbx only belong together when they come from one trap.
            g_caught_rax.store((uintptr_t)ep->ContextRecord->Rax);
        }
    }
    return EXCEPTION_CONTINUE_EXECUTION;
}

void begin_watch(uintptr_t addr) {
    init_self_range();
    g_caught_src = 0;
    g_caught_rax = 0;
    g_caught_rcx = 0;
    g_stage_motion = 0.0;
    g_watch_addr = addr;
    if (g_veh == nullptr) g_veh = AddVectoredExceptionHandler(1, veh);
    if (g_veh == nullptr) { g_stage = Stage::Failed; return; }
    g_watching = true;
    arm_all(addr);
    g_stage_ticks = 0;
}

void end_watch() {
    arm_all(0);
    g_watching = false;
    if (g_veh != nullptr) { RemoveVectoredExceptionHandler(g_veh); g_veh = nullptr; }
}

// A rotator we are willing to believe: finite, and within the range ControlRotation can hold.
bool plausible_rotator(uintptr_t p) {
    if (p == 0 || IsBadReadPtr((const void*)p, sizeof(double) * 3)) return false;
    const double* r = (const double*)p;
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(r[i]) || std::fabs(r[i]) > 100000.0) return false;
    }
    return std::fabs(r[0]) <= 91.0;   // pitch is clamped by the game; yaw and roll are not
}

// The candidate must not merely LOOK like a rotator -- it must BE the aim. Checking shape only is
// how a stack temporary got accepted: `exe+0x36297B0` returns an FRotator by value, so under the
// x64 ABI rax comes back pointing at the caller's stack scratch, which read (0.000, 0.000) and
// passed every structural test while being neither authoritative nor safe to write.
bool rotator_matches_aim(uintptr_t p, double pitch, double yaw) {
#if HALO_VR_DEV
    // FAULT INJECTION (blamfault 0x100): reject every candidate, so the locate can never succeed.
    // Drives the stage machine round its retry loop until MAX_ATTEMPTS and proves the give-up path
    // -- which otherwise only runs on a build where the search genuinely cannot work.
    if (g_cfg.blam_fault & 0x100) return false;
#endif
    if (!plausible_rotator(p)) return false;
    const double* r = (const double*)p;
    double dy = r[1] - yaw;
    while (dy > 180.0)  dy -= 360.0;
    while (dy < -180.0) dy += 360.0;
    return std::fabs(r[0] - pitch) < 5.0 && std::fabs(dy) < 5.0;
}

// Evidence intake for the stage machine, funnelled through one place so a fault can starve it.
//
// FAULT INJECTION (blamfault 0x400): report that nothing was ever caught. Every stage then has to
// run its ~24 s clock out and take the TIMEOUT branch.
//
// This is a genuinely different path from 0x100, which is why both exist. 0x100 lets a candidate be
// found and then rejects it, so the stage exits through "bad candidate, burn an attempt" -- fast,
// and it never touches the timeout code. Only starvation exercises the branch that handles "the
// writer never ran at all", which is what a build with a moved watchpoint actually looks like.
uintptr_t caught_src() {
#if HALO_VR_DEV
    if (g_cfg.blam_fault & 0x400) return 0;
#endif
    return g_caught_src.load();
}

uintptr_t caught_rcx() {
#if HALO_VR_DEV
    if (g_cfg.blam_fault & 0x400) return 0;
#endif
    return g_caught_rcx.load();
}

uintptr_t caught_rax() {
#if HALO_VR_DEV
    if (g_cfg.blam_fault & 0x400) return 0;
#endif
    return g_caught_rax.load();
}

// The timeout branches were silent, which made them unobservable and therefore untestable -- the
// same failure as the deliberately-silent re-resolve in BlamDrive. Dev-only: a stage timing out is
// rare and always worth seeing, so this is not gated on the fault bit.
void note_stage_timeout(const char* stage) {
#if HALO_VR_DEV
    API::get()->log_info("[Halo-CampE-UEVR] AIMDIRECT: stage %s TIMED OUT after ~%d s with no write "
                         "caught (aim moved %.3f deg in the window) -- restarting the search",
                         stage, STAGE_TICKS * 2, g_stage_motion);
#else
    (void)stage;
#endif
}

} // namespace

bool aim_direct_ready() { return g_target.load() != 0; }

void aim_direct_invalidate() {
    if (g_watching.load()) end_watch();
    g_target = 0;
    g_quat = 0;
    g_l2 = 0;
    g_stage = Stage::Idle;
    g_attempts = 0;
    g_have_last = false;
    g_waiting_logged = false;
}

bool aim_direct_set(double pitch_deg, double yaw_deg) {
    const uintptr_t t = g_target.load();
    if (t == 0) return false;
    // Revalidate every write. The chain is heap memory and a level load frees it; a stale pointer
    // would otherwise be a write into whatever now occupies that address.
    if (IsBadWritePtr((void*)t, sizeof(double) * 2)) { aim_direct_invalidate(); return false; }
    double* r = (double*)t;
    r[0] = pitch_deg;
    r[1] = yaw_deg;
    // roll (r[2]) deliberately untouched -- the game owns it.

    // Then the SOURCE. Writing the Euler alone is what made firing snap: the game recomputes it
    // from the quaternion, so a resync discards our value. Written after, so that within this frame
    // both representations agree; the quaternion is what survives to the next recompute.
    double nq[4];
    bool have_quat = false;

    const uintptr_t q = g_cfg.aim_quat ? g_quat.load() : 0;
    if (q != 0 && !IsBadWritePtr((void*)q, sizeof(double) * 4)) {
        quat_from_pitch_yaw(pitch_deg, yaw_deg, nq);
        have_quat = true;
        double* dst = (double*)q;
        dst[0] = nq[0]; dst[1] = nq[1]; dst[2] = nq[2]; dst[3] = nq[3];
    }

    // The SOURCE the cache is refreshed from. Writing here is the only place where a resync can
    // land on OUR value instead of reverting to the game's -- which is what the firing snap is.
    const uintptr_t qs = g_cfg.aim_quat_src ? g_quat_src.load() : 0;
    if (qs != 0 && !IsBadWritePtr((void*)qs, sizeof(double) * 4)) {
        if (!have_quat) quat_from_pitch_yaw(pitch_deg, yaw_deg, nq);
        double* dst = (double*)qs;
        dst[0] = nq[0]; dst[1] = nq[1]; dst[2] = nq[2]; dst[3] = nq[3];
    }
    return true;
}

void aim_direct_tick() {
    if (!g_cfg.aim_direct) {
        if (g_stage != Stage::Idle || g_target.load() != 0) aim_direct_invalidate();
        return;
    }

    void* pc = nullptr;
    double p = 0.0, y = 0.0;
    const bool have_pc = read_control_rotation(&p, &y, &pc) && pc != nullptr;

    // A new PlayerController means the whole chain was reallocated.
    if (have_pc && pc != g_known_pc) {
        aim_direct_invalidate();
        g_known_pc = pc;
    }
    if (!have_pc) { aim_direct_invalidate(); g_known_pc = nullptr; g_have_last = false; return; }
    if (g_stage == Stage::Ready || g_stage == Stage::Failed) return;

    // THE WRITER IS CONDITIONAL: it only runs when the aim actually changes. Arming into a
    // dead-still game traps nothing and looks exactly like "the write does not happen" -- which is
    // precisely how the first version of this failed, three attempts in a row, while the identical
    // watchpoint in AimWatch caught the writer on the first frame of stick input.
    const double moved = g_have_last ? (std::fabs(y - g_last_yaw) + std::fabs(p - g_last_pitch)) : 0.0;
    g_last_yaw = y; g_last_pitch = p; g_have_last = true;
    g_stage_motion += moved;

    switch (g_stage) {
    case Stage::Idle: {
        // FAST RE-ARM. Tried before the motion gate below, so it can confirm while the player is
        // still -- the slow path cannot even start until they move.
        if (g_hint_valid) {
            uintptr_t cand = (uintptr_t)pc + g_hint_target_off;
#if HALO_VR_DEV
            // FAULT INJECTION (blamfault 0x080): a hint that no longer points at the rotator. This
            // is the case the validation exists for -- the offset held within one run and then did
            // not -- and it must fall through to the watch rather than write to a stale address.
            if (g_cfg.blam_fault & 0x080) cand += 0x2000;
#endif
            if (plausible_rotator(cand) && rotator_matches_aim(cand, p, y)) {
                if (g_hint_stage == 0) {
                    g_hint_stage   = 1;
                    g_hint_ref_yaw = y;
                } else if (std::fabs(y - g_hint_ref_yaw) >= MOTION_DEG) {
                    // Matched the aim BEFORE and AFTER it moved: it is tracking, not coincidentally
                    // holding the same numbers. A fixed unrelated rotator cannot pass this.
                    g_target = cand;
                    g_stage  = Stage::Ready;
                    g_hint_stage = 0;
                    API::get()->log_info("[Halo-CampE-UEVR] AIMDIRECT: re-armed from cached offset "
                                         "(pc+0x%llX = 0x%llX) - skipped the watch",
                                         (unsigned long long)g_hint_target_off,
                                         (unsigned long long)cand);
                    return;
                }
            } else {
                // The offset is not stable across this transition. Say so once: whether this
                // invariant holds is exactly what a future session needs to know, and a silent
                // fallback would make the fast path look like it never existed.
                if (!g_hint_miss_logged) {
                    g_hint_miss_logged = true;
                    API::get()->log_info("[Halo-CampE-UEVR] AIMDIRECT: cached offset pc+0x%llX did not "
                                         "validate - the target is NOT at a fixed offset from the "
                                         "PlayerController; using the watch",
                                         (unsigned long long)g_hint_target_off);
                }
                g_hint_valid = false;
                g_hint_stage = 0;
            }
        }

        if (g_attempts >= MAX_ATTEMPTS) {
            API::get()->log_info("[Halo-CampE-UEVR] AIMDIRECT: gave up locating the rotator after %d attempts"
                                 " with aim motion present - falling back to the stick loop", MAX_ATTEMPTS);
            // WHAT THE HANDLER ACTUALLY SAW. Without this the give-up says only that the hunt
            // failed, not whether there was anything findable -- which is the difference between
            // "our filter is too strict" and "this build has no persistent mirror to find", and
            // those need opposite fixes. Cheap: printed once, on a path that has already given up.
            API::get()->log_info("[Halo-CampE-UEVR] AIMDIRECT: candidates seen -- %u on the writing "
                                 "thread's STACK (last 0x%llX, rejected: a stack frame is reused, so "
                                 "it can never hold a persistent mirror), %u NOT on a stack (last "
                                 "0x%llX). If that second count is 0, this build never writes the "
                                 "rotator from a durable object on the watched path.",
                                 g_cand_stack.load(), (unsigned long long)g_cand_last_stack.load(),
                                 g_cand_heap.load(),  (unsigned long long)g_cand_last_heap.load());
            g_stage = Stage::Failed;
            return;
        }
        // Wait for the player to move their view rather than nudging the aim ourselves. In VR an
        // uncommanded jerk is a real disturbance, and any head or stick motion serves just as well.
        if (moved < MOTION_DEG) {
            if (!g_waiting_logged) {
                g_waiting_logged = true;
                API::get()->log_info("[Halo-CampE-UEVR] AIMDIRECT: waiting for aim motion to locate the"
                                     " rotator (the writer only runs when aim changes)");
            }
            return;
        }
        // Watch the YAW specifically. Watching the whole rotator would also trap the pitch store and
        // make it ambiguous which instruction we caught.
        begin_watch((uintptr_t)pc + g_control_rotation_offset.load(std::memory_order_relaxed)
                    + sizeof(double));
        g_stage = Stage::WatchControlRotation;
        break;
    }
    case Stage::WatchControlRotation: {
        const uintptr_t src = caught_src();
        if (src != 0) {
            end_watch();
            // src is L1 -- a mirror, not the target. Its own writer holds the real one.
            begin_watch(src + sizeof(double));
            g_stage = Stage::WatchL1;
        } else if (++g_stage_ticks > STAGE_TICKS) {
            end_watch();
            note_stage_timeout("WatchControlRotation");
            // Only a window that DID see movement is evidence of anything. A quiet window means the
            // player stood still, so it must not burn an attempt.
            if (g_stage_motion >= MOTION_DEG) ++g_attempts;
            g_stage = Stage::Idle;
        }
        break;
    }
    case Stage::WatchL1: {
        const uintptr_t src = caught_src();
        if (src != 0) {
            end_watch();
            // The level-2 writer copies from [rbx+0x20], not [rbx]: `mov eax,0x20;
            // vmovups xmm0,[rbx+rax]`. Using rbx directly would land 0x20 short and write into
            // whatever precedes the rotator.
            const uintptr_t target = src + 0x20;
            if (plausible_rotator(target)) {
                // L2 IS NOT AUTHORITATIVE EITHER. The game stamps it every frame from a rotator
                // returned by a call:
                //     call    exe+0x36297B0
                //     vmovups xmm0, [rax]
                //     vmovups [rbx+0x20], xmm0      <- L2
                // so writing L2 is overwritten on the game's own schedule. That is the periodic
                // revert, and it is why firing snapped the aim back: the shot resyncs from the real
                // store. One more stage watches L2 and takes rax, which points at that store.
                g_l2 = target;
                API::get()->log_info("[Halo-CampE-UEVR] AIMDIRECT: L2 at 0x%llX - watching it to find the"
                                     " store the game stamps it from", (unsigned long long)target);
                begin_watch(target + sizeof(double));
                g_stage = Stage::WatchL2;
            } else {
                API::get()->log_info("[Halo-CampE-UEVR] AIMDIRECT: candidate 0x%llX did not read as a"
                                     " rotator - retrying", (unsigned long long)target);
                ++g_attempts;   // a bad candidate IS a failed attempt, or this retries forever
                g_stage = Stage::Idle;
            }
        } else if (++g_stage_ticks > STAGE_TICKS) {
            end_watch();
            note_stage_timeout("WatchL1");
            if (g_stage_motion >= MOTION_DEG) ++g_attempts;
            g_stage = Stage::Idle;
        }
        break;
    }
    case Stage::WatchQuatSrc: {
        const uintptr_t rcx = caught_rcx();
        if (rcx != 0) {
            end_watch();
            const uintptr_t cand = rcx + 0x10;
            bool ok = false;
            if (!IsBadReadPtr((const void*)cand, sizeof(double) * 4)) {
                const double* q = (const double*)cand;
                double n = 0.0; bool finite = true;
                for (int i = 0; i < 4; ++i) {
                    if (!std::isfinite(q[i])) { finite = false; break; }
                    n += q[i] * q[i];
                }
                ok = finite && std::fabs(n - 1.0) < 0.01;
            }
            if (ok) {
                g_quat_src = cand;
                API::get()->log_info("[Halo-CampE-UEVR] AIMDIRECT: quaternion SOURCE at 0x%llX "
                                     "([obj+0x1D0]) - set aimquatsrc=1 to write it and test whether the "
                                     "resync stops fighting", (unsigned long long)cand);
            } else {
                API::get()->log_info("[Halo-CampE-UEVR] AIMDIRECT: rcx+0x10 (0x%llX) is not a unit "
                                     "quaternion - source not taken (rcx is volatile across the call "
                                     "in that function, so it does not always survive)",
                                     (unsigned long long)cand);
            }
            g_stage = Stage::Ready;
        } else if (++g_stage_ticks > STAGE_TICKS) {
            end_watch();
            API::get()->log_info("[Halo-CampE-UEVR] AIMDIRECT: no sync write caught - running on L2 only "
                                 "(aim tracks, but firing still resyncs)");
            g_stage = Stage::Ready;
        }
        break;
    }
    case Stage::WatchL2: {
        // rax, not rbx: the game reads its source through rax and stores through rbx+0x20.
        const uintptr_t src = caught_rax();
        if (src != 0) {
            end_watch();
            if (rotator_matches_aim(src, p, y)) {
                g_target = src;
                g_stage = Stage::Ready;
                // Remember the offset so the next level load can skip all of this. See FAST RE-ARM.
                if (g_known_pc != nullptr) {
                    g_hint_target_off  = (ptrdiff_t)(src - (uintptr_t)g_known_pc);
                    g_hint_valid       = true;
                    g_hint_stage       = 0;
                    g_hint_miss_logged = false;
                }
                const double* r = (const double*)src;
                // The OFFSET is logged alongside the address because it is the only half that means
                // anything outside this process: the absolute address is heap and dies with the
                // session, whereas pc+off is comparable between runs and between machines. Whether
                // it is stable across RUNS is currently unknown -- the fast re-arm above assumes it
                // only within one run -- and printing it is what makes that answerable from logs
                // instead of assumed. Same reasoning as logging an RVA rather than a VA.
                API::get()->log_info("[Halo-CampE-UEVR] AIMDIRECT ready: AUTHORITATIVE rotator at 0x%llX "
                                     "(pc+0x%llX) (pitch %.3f yaw %.3f) - the store the game reads, so "
                                     "the write should survive firing and the periodic resync",
                                     (unsigned long long)src,
                                     (unsigned long long)(g_known_pc != nullptr
                                         ? (uintptr_t)(src - (uintptr_t)g_known_pc) : 0),
                                     r[0], r[1]);
            } else if (g_l2 != 0 && rotator_matches_aim(g_l2, p, y)) {
                // FALL BACK TO L2, and say exactly what that costs. rax was a stack temporary --
                // the getter returns FRotator by value, so it hands back caller scratch, not the
                // store. L2 drives aim correctly frame to frame; it is only overwritten when the
                // game resyncs, which is the snap seen on firing. Better than no direct aim, and
                // strictly better than writing into somebody else's stack.
                g_target = g_l2;
                g_stage = Stage::Ready;

                // LEARN THE HINT HERE TOO. Measured 2026-08-15: this fallback is the path this
                // build actually takes -- rax is a stack temporary here every time -- and it was
                // the ONLY Ready branch that did not record g_hint_target_off. So the FAST RE-ARM
                // above could never engage: the optimisation existed but was unreachable on the
                // live path, and every PlayerController change paid the full multi-stage watch
                // again (~40 s, and it cannot even start until the player moves).
                //
                // Safe because the hint is a HINT: on re-arm it is accepted only after matching the
                // real aim both before and after the aim has moved, so a wrong or unstable offset
                // is rejected in two ticks and falls through to the watch exactly as it does today.
                if (g_known_pc != nullptr) {
                    g_hint_target_off  = (ptrdiff_t)(g_l2 - (uintptr_t)g_known_pc);
                    g_hint_valid       = true;
                    g_hint_stage       = 0;
                    g_hint_miss_logged = false;
                }

                // The quaternion the Euler is derived FROM sits 0x20 below L2. Accepted only if it
                // reads as a unit quaternion, so a layout change fails closed to Euler-only rather
                // than scribbling four doubles over whatever moved in.
                const uintptr_t qcand = g_l2 - 0x20;
                bool quat_ok = false;
                if (!IsBadReadPtr((const void*)qcand, sizeof(double) * 4)) {
                    const double* q = (const double*)qcand;
                    double n = 0.0;
                    bool finite = true;
                    for (int i = 0; i < 4; ++i) {
                        if (!std::isfinite(q[i])) { finite = false; break; }
                        n += q[i] * q[i];
                    }
                    quat_ok = finite && std::fabs(n - 1.0) < 0.01;
                    if (quat_ok) g_quat = qcand;
                }
                // pc+0x... is logged for the same reason as on the authoritative branch: the heap
                // address dies with the session, the OFFSET is the half that can be compared
                // between runs -- which is how "is this offset stable across runs?" becomes a
                // question answerable from two logs instead of an assumption.
                API::get()->log_info("[Halo-CampE-UEVR] AIMDIRECT: L2 at 0x%llX (pc+0x%llX) driving, "
                                     "quaternion cache %s at 0x%llX (rax 0x%llX was a stack temporary "
                                     "- by-value FRotator return, not the store)",
                                     (unsigned long long)g_l2,
                                     (unsigned long long)(g_known_pc != nullptr
                                         ? (uintptr_t)(g_l2 - (uintptr_t)g_known_pc) : 0),
                                     quat_ok ? "OK" : "NOT FOUND",
                                     (unsigned long long)qcand, (unsigned long long)src);

                // STAGE 4: the cache is refreshed from [obj+0x1D0] by a change-guarded sync
                // (exe+0x5B5D7A0: vmovupd ymm0,[rcx+0x1D0] ... vcmppd/test/je ... vmovups [rbx],ymm2).
                // Watching the CACHE catches that sync, and its rcx -- already advanced by 0x1C0 --
                // puts the source at rcx+0x10. Aim is already driving off L2 by now, so a failure
                // here costs nothing but the snap we started with.
                if (quat_ok) {
                    begin_watch(qcand + sizeof(double));
                    g_stage = Stage::WatchQuatSrc;
                    break;
                }
            } else {
                API::get()->log_info("[Halo-CampE-UEVR] AIMDIRECT: neither rax 0x%llX nor L2 0x%llX matches"
                                     " the live aim - retrying", (unsigned long long)src,
                                     (unsigned long long)g_l2);
                ++g_attempts;
                g_stage = Stage::Idle;
            }
        } else if (++g_stage_ticks > STAGE_TICKS) {
            end_watch();
            note_stage_timeout("WatchL2");
            if (g_stage_motion >= MOTION_DEG) ++g_attempts;
            g_stage = Stage::Idle;
        }
        break;
    }
    default: break;
    }
}

} // namespace halo
