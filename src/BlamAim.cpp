#include "BlamAim.hpp"

#if HALO_VR_DEV

#include "BlamDrive.hpp"
#include "Config.hpp"
#include "MotionAimControl.hpp"
#include "uevr/API.hpp"

#include <Windows.h>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <intrin.h>

using namespace uevr;

namespace halo {
namespace {

// ---------------------------------------------------------------- the target
//
// HaloSimulation_tag_release.dll + 0x5A6AD0 -- the ORIENTATION GETTER.
//
//   void get_orientation(uint16_t handle /*rcx*/, Vec3* outFwd /*rdx*/, Vec3* outUp /*r8*/)
//
// This is the convergence point for the shot direction: 45 of the 55 projectile-creation call sites
// share one spawn-params constructor, and the direction handed to it comes from here. Internally it
// resolves the shooter's Blam object and copies object+0x50 and object+0x5C.
//
// WHY A HOOK AND NOT A POINTER WALK
//   The obvious approach -- replicate the TLS chain and read the object directly -- was tried and
//   failed: the sim context at TLS block +0x20 read as zero across every thread, 1.19 billion
//   samples, polled off-thread while the game ran. The chain is only valid INSIDE the sim's own
//   call stack. A hook runs exactly there, so the same chain that never resolved from the tick
//   should resolve here. That is the point of hooking rather than a nicety.
constexpr uintptr_t RVA_GET_ORIENTATION = 0x5A6AD0;

// The projectile spawn itself. Hooking this is how the experiment is VERIFIED without eyes on the
// screen: it receives the finished spawn-params struct, so logging the direction fields shows
// exactly what the game is about to fire along -- and whether an override upstream reached it.
//
//   int create_projectile(SpawnParams* params /*rcx*/)
//   params+0x1C, +0x28, +0x34 are the three vec3s the creation function marshals into the object.
constexpr uintptr_t RVA_CREATE_PROJECTILE = 0x5A0FB0;
constexpr uintptr_t P_VEC1 = 0x1C;
// Local copy so this dead-lane file pulls in nothing from Plugin.cpp. Yaw wraps, so a raw
// difference of 359 deg is really 1 deg of error -- without this the metric would scream at the
// exact moment the aim crosses the +/-180 seam.
inline float wrap180_local(float d) {
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// Where the player is ACTUALLY pointing, in game-space degrees, derived from the controller pose
// and the calibration reference -- deliberately NOT from g_desired_* or ControlRotation.
//
// Both of those are unusable here. g_desired_* is published inside aim_control_law(), which only
// runs on the STICK path, so under direct drive it holds a stale value forever. ControlRotation is
// an output mirror that direct drive itself pins. Sourcing the redirect from either one means that
// in the mode this exists to fix, we would be writing a frozen direction and calling it success.
//
// This mirrors aim_control_law()'s own arithmetic exactly (see MotionAimControl.cpp): the hand's
// rotation since calibration, added to the aim that was captured at calibration.
// MOVED to MotionAimControl.cpp as desired_aim_now(), unchanged, and forwarded to from here.
// Not a tidy-up: the movement frame needs this exact value and cannot call into this file, which
// is entirely #if HALO_VR_DEV. Copying it there would have made three copies of one definition,
// and a copy that drifts is precisely how the movement frame ends up compensating for an aim the
// sim is not being given.
bool controller_desired_aim(float* out_yaw, float* out_pitch) {
    return desired_aim_now(out_yaw, out_pitch);
}

constexpr uintptr_t P_VEC2 = 0x28;
constexpr uintptr_t P_VEC3 = 0x34;

// Chain offsets, straight from the disassembly of that function.
constexpr uintptr_t RVA_TLS_INDEX    = 0xD72730;
constexpr uintptr_t OFF_CTX_IN_TLS   = 0x20;
constexpr uintptr_t OFF_OBJ_TABLE    = 0x50;
constexpr uintptr_t OBJ_ENTRY_SIZE   = 24;
constexpr uintptr_t OFF_OBJ_IN_ENTRY = 0x10;
constexpr uintptr_t OFF_ORIENT_A     = 0x50;
constexpr uintptr_t OFF_ORIENT_B     = 0x5C;

struct Vec3f { float x, y, z; };

// Returning uintptr_t rather than void deliberately: the real function's return value is not
// modelled, and declaring void would let the compiler clobber rax on the way back out.
using GetOrientFn = uintptr_t (*)(uintptr_t handle, Vec3f* outA, Vec3f* outB);

using CreateProjFn = uintptr_t (*)(uintptr_t params);

GetOrientFn      g_original = nullptr;
int              g_hook_id  = -1;
CreateProjFn     g_orig_create = nullptr;
int              g_create_hook_id = -1;
std::atomic<uint64_t> g_spawns{0};

// OVERRIDE. blamaim>=2 rewrites the orientation this getter returns for the handles that track the
// aim. Proving causality first with a fixed direction, rather than wiring the real aim straight in:
// if a hard-coded 90-degree offset does not move the spawn direction, nothing subtler will either.
uintptr_t        g_sim_base = 0;
uint32_t         g_tls_index = 0;

// The sim context is THREAD-LOCAL to the Blam simulation thread: __readgsqword(0x58) only resolves
// it on that thread. Anything running on the UEVR frame path reads ctx=0 and silently finds
// nothing. Cache it from inside the sim-thread hook so off-thread work (the globals scan) can use
// it. Latched, not re-derived, because the whole point is that the deriving code cannot run here.
std::atomic<uintptr_t> g_ctx_cached{0};
std::atomic<uintptr_t> g_table_cached{0};
int              g_prev_flag = 0;

// The hook runs on the sim thread at spawn rate. Everything in it is rate-limited or wait-free:
// a chatty hook on a hot path is how you turn a diagnostic into a stutter.
std::atomic<uint64_t> g_calls{0};
std::atomic<uint64_t> g_last_log_ms{0};

// Captured for the tick to consume. Written by the hook, read elsewhere -- plain atomics, no lock,
// because a torn read here costs a wrong log line and nothing else.
std::atomic<uintptr_t> g_last_obj{0};
std::atomic<uint32_t>  g_last_handle{0};
std::atomic<float>     g_last_ax{0.0f}, g_last_ay{0.0f}, g_last_az{0.0f};
std::atomic<float>     g_last_bx{0.0f}, g_last_by{0.0f}, g_last_bz{0.0f};
std::atomic<bool>      g_chain_ok{false};

// PER-HANDLE CORRELATION.
//
// The getter is called ~2300 times a second across many objects, so "the last call" is whichever
// object happened to be last -- meaningless. The player's object is the one whose orientation
// TRACKS THE AIM, so score every call against the live aim and keep the best per handle. The tick
// publishes the aim vector here; the hook only does a dot product and one compare, which is all a
// hot path can afford.
constexpr uint32_t HANDLE_SLOTS = 4096;
std::atomic<float> g_aim_fx{1.0f}, g_aim_fy{0.0f}, g_aim_fz{0.0f};
std::atomic<float> g_best_dot[HANDLE_SLOTS];
std::atomic<uint32_t> g_hits[HANDLE_SLOTS];
// The A vector last seen for a handle, so a good correlation can be inspected rather than trusted.
std::atomic<float> g_hx[HANDLE_SLOTS], g_hy[HANDLE_SLOTS], g_hz[HANDLE_SLOTS];
std::atomic<bool>  g_stats_armed{false};

bool read_ptr(uintptr_t p, uintptr_t* out) {
    if (p == 0 || IsBadReadPtr((const void*)p, sizeof(uintptr_t))) return false;
    const uintptr_t v = *(const uintptr_t*)p;
    // NO ALIGNMENT REQUIREMENT. This used to reject anything with (v & 7), which cost this project
    // the object table for days: the live chain is
    //   ctx=0x233E122528C  table=0x233E12252FC
    // -- both 4-aligned, both perfectly valid, both silently thrown away. Blam's structures are
    // 4-byte packed, so 8-alignment was never a property these pointers had. The rejection was
    // indistinguishable from a null read, which is how it got recorded as "[block+0x20] is ALWAYS
    // ZERO" and closed the lane.
    if (v != 0 && (v < 0x10000ull || v >= 0x7FFFFFFFFFFFull)) return false;
    *out = v;
    return true;
}

// Resolve the shooter's object the same way the target function does. Only valid on the sim thread,
// inside its scope -- which is precisely where this runs.
// One-shot trace of the pointer chain, logging each hop RAW.
//
// This exists because "[block+0x20] is always zero" was concluded from read_ptr() returning false --
// and read_ptr conflates three different outcomes: unreadable, genuinely zero, and "read fine but
// failed my alignment/range filter". A valid pointer rejected by the filter looks exactly like a
// zero to the caller. The getter itself walks this chain thousands of times a second on this very
// thread, so the walk demonstrably works here; the fault is far more likely to be ours.
void trace_chain_once(uintptr_t handle) {
    static std::atomic<bool> done{false};
    bool expected = false;
    if (!done.compare_exchange_strong(expected, true, std::memory_order_relaxed)) return;

    const uintptr_t tls_array = (uintptr_t)__readgsqword(0x58);
    uintptr_t block = 0, ctx = 0, table = 0, obj = 0;
    const char* stage = "tls_array";

    if (tls_array != 0 && !IsBadReadPtr((const void*)(tls_array + (uintptr_t)g_tls_index * 8), 8)) {
        block = *(const uintptr_t*)(tls_array + (uintptr_t)g_tls_index * 8);
        stage = "block";
        if (block != 0 && !IsBadReadPtr((const void*)(block + OFF_CTX_IN_TLS), 8)) {
            ctx = *(const uintptr_t*)(block + OFF_CTX_IN_TLS);
            stage = "ctx";
            if (ctx != 0 && !IsBadReadPtr((const void*)(ctx + OFF_OBJ_TABLE), 8)) {
                table = *(const uintptr_t*)(ctx + OFF_OBJ_TABLE);
                stage = "table";
                const uintptr_t e = table + (handle & 0xFFFFull) * OBJ_ENTRY_SIZE + OFF_OBJ_IN_ENTRY;
                if (table != 0 && !IsBadReadPtr((const void*)e, 8)) {
                    obj = *(const uintptr_t*)e;
                    stage = "obj";
                }
            }
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] BLAMCHAIN raw: tlsIdx=%u tlsArray=0x%llX block=0x%llX "
                         "ctx=0x%llX table=0x%llX obj=0x%llX handle=0x%04X reached=%s",
                         g_tls_index, (unsigned long long)tls_array, (unsigned long long)block,
                         (unsigned long long)ctx, (unsigned long long)table,
                         (unsigned long long)obj, (unsigned)(handle & 0xFFFF), stage);
}

uintptr_t resolve_object(uintptr_t handle) {
    const uintptr_t tls_array = (uintptr_t)__readgsqword(0x58);
    if (tls_array == 0) return 0;
    uintptr_t block = 0, ctx = 0, table = 0, obj = 0;
    if (!read_ptr(tls_array + (uintptr_t)g_tls_index * 8, &block) || block == 0) return 0;
    if (!read_ptr(block + OFF_CTX_IN_TLS, &ctx) || ctx == 0) return 0;
    if (!read_ptr(ctx + OFF_OBJ_TABLE, &table) || table == 0) return 0;
    const uintptr_t idx = handle & 0xFFFFull;
    if (!read_ptr(table + idx * OBJ_ENTRY_SIZE + OFF_OBJ_IN_ENTRY, &obj)) return 0;
    return obj;
}

// Dump one object's float fields, so two aim poses can be diffed to find the field the stick
// actually moves. THAT field is the prize: it is the sim's own aim, so writing it would give zero
// lag AND replicate -- unlike the spawn redirect, which is client-local.
//
// Must be sampled in STICK mode: under direct drive the sim aim is frozen, so nothing would change
// between poses and the diff would be empty.
uint32_t readable_floats(uintptr_t obj, uint32_t maxBytes);   // defined with the scanners below

void dump_object_once(uintptr_t handle) {
    const uint32_t want = (uint32_t)g_cfg.blam_obj;
    if (want == 0 || (uint32_t)(handle & 0xFFFF) != want) return;

    static std::atomic<uint32_t> last_tick{0};
    const uint32_t now = GetTickCount();
    const uint32_t prev = last_tick.load(std::memory_order_relaxed);
    if (now - prev < 1500) return;                       // ~0.7 Hz; this walks 128 floats
    if (!last_tick.compare_exchange_strong(const_cast<uint32_t&>(prev), now,
                                           std::memory_order_relaxed)) return;

    // Range is configurable because 0x200 was too short: the unit's aim fields sit at 0x1D4-0x21C,
    // but the animation/node state the firing solution actually consumes is further out, and a
    // diff can only find what it dumps. Bytes, clamped to something a hot path can still afford.
    int bytes = g_cfg.blam_obj_len > 0 ? g_cfg.blam_obj_len : 0x200;
    if (bytes < 0x80)  bytes = 0x80;
    if (bytes > 0x800) bytes = 0x800;
    const int chunks = bytes / 0x80;

    const uintptr_t obj = resolve_object(handle);
    if (obj == 0) return;
    // Bound to the committed region rather than trusting IsBadReadPtr, which rejected the whole
    // 0x800 dump outright on a smaller object -- producing zero output that looked like "the handle
    // is wrong" and cost a test cycle. Short is fine; nothing is better than nothing.
    const uint32_t avail = readable_floats(obj, (uint32_t)bytes);
    if (avail < 32) return;
    const int chunks_ok = (int)(avail / 32);

    const float* f = (const float*)obj;
    // 32 floats per line; one line of everything would be truncated by the logger.
    for (int chunk = 0; chunk < chunks_ok; ++chunk) {
        char buf[900];
        int n = 0;
        for (int i = chunk * 32; i < (chunk + 1) * 32 && n < (int)sizeof(buf) - 16; ++i) {
            n += snprintf(buf + n, sizeof(buf) - n, "%.3f ", f[i]);
        }
        API::get()->log_info("[Halo-CampE-UEVR] BLAMOBJ h=0x%04X +0x%03X: %s",
                             (unsigned)want, (unsigned)(chunk * 32 * 4), buf);
    }
}

// THE MULTIPLAYER CANDIDATE: write the sim's OWN aim vector.
//
// obj+0x1D4 tracks the aim exactly (Y-negated, as everything Blam-side is), and it is the value the
// STICK actuator ultimately moves -- which is why stick mode replicates and the spawn redirect does
// not. If writing it steers the shot, we get zero lag AND a value the host derives from, instead of
// a client-local patch applied after the fact.
//
// Runs in the getter hook, on the sim thread, at ~2300 Hz -- fast enough to win against the game's
// own per-tick writer. Independent of the spawn redirect: both can be toggled separately.
// THREE COPIES, not one. The game's own setter writes the same vector to all of these:
//
//   mov  ecx,20h / mov rax,[rbx+rcx] / mov rcx,[rax+50h] / mov r8,[rcx+rdx*8+10h]   <- same chain
//   vmovsd [r8+1D4h],xmm0 ; mov [r8+1DCh],eax      <- copy 1  (x,y then z)
//   vmovsd [r8+1F8h],xmm0 ; mov [r8+200h],eax      <- copy 2
//   vmovsd [r8+21Ch],xmm0                          <- copy 3
//
// Writing only 0x1D4 moved the aim (something reads it) but left the shot 30-45 deg off, because
// the fire path reads a copy we had not updated. Keep them consistent or the unit's aim is
// internally contradictory.
// THE AIM SETTER -- the authoritative place to intervene.
//
//   dll+0x1842C0  push rbx/rbp/rdi/r14 ... mov rdi,rdx ... movzx r14d,cx
//     arg1 (rcx) = object handle,  arg2 (rdx) = pointer to the source vector (3 floats)
//
// Rewriting arg2 before the original runs means the GAME stores our value -- into all three copies,
// in its own order, at its own time. That beats racing it from the getter hook, which only
// half-worked: the aim tracked at one pose and stuck at the next, and the shot never followed.

// ---------------------------------------------------------------- THE FIRE FUNCTION (Option 4)
//
//   dll+0x5CF460   fire(uint32 a1 /*rcx*/, uint16 idx /*dx*/, void* table /*r8*/, uint8 a4 /*r9b*/)
//
// This is where the shot direction is actually decided, and it is NOT any unit aim vector -- which
// is why writing +0x1D4/+0x1F8/+0x21C/+0x1E0 moved the camera and left the shot frozen at ~93 deg.
// The function looks the shooter up in a small table passed as arg3 and takes a TARGET POINT from
// the matching record:
//
//   mov  r8d,[rdi+78h]            ; entry count
//   imul rcx,rax,1Ch              ; stride 28
//   lea  rax,[rdi+8]              ; array base
//   cmp  dword ptr [rdi+rcx+8],r9d ; record+0x00 == shooter id
//   lea  r13,[rax+rcx]            ; -> record
//   ...
//   cmp  byte ptr [r13+5],0 / jne SKIP     ; record+0x05 gates the target path
//   vmovss [r13+8] / [r13+0Ch] / [r13+10h] ; record+0x08 = TARGET POINT
//   -> direction = normalize(target - origin)
//
// Array is 4 records (4 * 0x1C = 0x70, ending exactly at the count field at +0x78).
constexpr uintptr_t FIRE_TBL_ARRAY    = 0x08;   // first record
constexpr uintptr_t FIRE_TBL_COUNT    = 0x78;
constexpr uintptr_t FIRE_REC_STRIDE   = 0x1C;
constexpr uintptr_t FIRE_REC_FLAG     = 0x05;   // non-zero SKIPS the target path
constexpr uintptr_t FIRE_REC_TARGET   = 0x08;

// The origin is computed inside the function and is not available at entry -- but it does not need
// to be. Writing the target a very long way down the desired direction makes the subtraction
// negligible: with |origin| ~ 100 world units, a 1e6 target leaves under 0.01 deg of error.
constexpr float FIRE_TARGET_RANGE = 1.0e6f;

// ---------------------------------------------------------------- THE DIRECTION PRODUCER (Option 5)
//
//   dll+0x2A0DB0 -- called on the NO-TABLE path, which is the one the player's shots take
//   (BLAMFIRE logged table=0x0, and the fire function skips the target lookup via test rdi,rdi).
//
//     lea  rax,[rsp+308h]     ; &params+0x28, the direction slot
//     mov  [rsp+20h],rax      ; 5th argument
//     call 00000001802A0DB0   ; writes the shot direction into it
//
// arg5 lands at [rsp+0x20] from the caller, so the hook must declare all eight arguments to reach
// it. The trailing stack args are floats; they are declared as raw 8-byte integers purely so they
// pass through bit-identical -- we never interpret them.
//
// Everything after this call is spread persistence: the first shot of a burst copies the result to
// [rsp+1EC/1E8/1B4] and later shots restore from there, which is what keeps a burst's cone stable.

// Set by the fire hook for the duration of the player's own fire call. The direction producer is
// not player-specific, so this is what keeps the override off everyone else's shots -- the same job
// blamcaller could never do, done at a frame where the shooter identity is actually known.
thread_local bool t_player_fire = false;

// ALL FIVE COPIES. BLAMSCAN swept every object at every offset for a unit vector equal to the aim,
// twice, ~70 deg apart. Exactly these survived on object 0x0001 (the player):
//
//   +0x1D4  +0x1F8  +0x204  +0x21C  +0x228  = (0.5444,0.4671,-0.6967)  vs aim (0.5447,0.4671,-0.6965)
//
// The game's own setter writes only 0x1D4/0x1F8/0x21C. We wrote those three, the camera followed and
// the shot did not -- so +0x204 and +0x228, which nothing was updating, are the likely reason. Write
// every copy or the unit's aim is internally inconsistent and the fire path reads a stale one.
constexpr uintptr_t OFF_UNIT_AIM_ALL[] = { 0x1D4, 0x1F8, 0x204, 0x21C, 0x228 };

// The OTHER object BLAMSCAN flagged: 0x0005's node-transform array also tracks the aim. These are
// the animated bone matrices the weapon's marker hangs off, so if the fire path reads a transform
// rather than a stored aim, this is where it reads it.
//
// blamwrite=7 writes BOTH sets -- i.e. every field in the entire object table that equals the aim.
// If the shot still ignores that, no stored aim state drives it and memory writes cannot fix it.
constexpr uintptr_t OFF_NODE_AIM_ALL[] = { 0x348, 0x37C, 0x3B0, 0x418, 0x480, 0x4B4 };

constexpr uintptr_t OFF_UNIT_AIM   = 0x1D4;
constexpr uintptr_t OFF_UNIT_AIM_2 = 0x1F8;
constexpr uintptr_t OFF_UNIT_AIM_3 = 0x21C;
constexpr uintptr_t OFF_UNIT_FACING = 0x1E0;   // horizontal body facing (z=0), NOT written by the setter
// (write_sim_aim, hooked_set_aim, hooked_fire and hooked_dir were here -- the blamwrite
//  family. All four wrote a place that does not steer the shot: obj+0x1D4 is a mirror the sim
//  recomputes each tick, the setter at 0x1842C0 installs but is never invoked during normal
//  aiming, and the fire/direction rewrites were measured against ~360 rounds with no effect.
//  The answer was the angular control record instead -- see BlamDrive.cpp, which ships.
//  Removed rather than kept as evidence: they are WRITE paths into a live simulation, and a
//  refuted write is a hazard, not a record. The negative results are in docs\BLAM_AIM_FINDINGS.md.


// ---- THE PLAYER CONTROL TABLE: the angular control state --------------------------------------
//
// MOVED TO BlamDrive.cpp, which is NOT dev-gated. That file holds what this investigation actually
// found -- the control record at tls_block+0xB8, stride 0x198, yaw +0x94 / pitch +0x98, and the one
// hook needed to write it. It ships; this file does not.
//
// Nothing is duplicated here on purpose. A second copy of that resolve or that write is exactly how
// the shipping path and the dev path drift apart, and this file already carries several refuted
// hypotheses that a reader could mistake for the live one.
//
// The dev hook below still calls halo::drive_control_angles(), so a dev build behaves identically
// to a release build on the aim path and only adds instrumentation around it.

// blamwrite=2: intercept the setter and substitute our aim as its INPUT.
// FIND THE PLAYER'S OBJECT BY POSITION -- `blamfind=1`.
//
// Identifying it by orientation correlation was unreliable: it picked 0x0005, whose node transforms
// moved 0.02 for a 66 deg aim change (idle drift, not aim). And handles are PER-SESSION, so a value
// noted in an earlier launch is worse than useless -- two "zero changed fields" diffs were run
// against stale handles that were never the player.
//
// The muzzle origin v1 logged on every spawn IS the player's position in Blam coordinates, so scan
// the object table for whichever object holds that position. That yields the handle AND the
// position field offset, and re-derives itself every session instead of being written down.
//
// One-shot and bounded (512 objects x 256 offsets ~ 131k float reads, well under a frame) because
// this runs on the sim thread and a stall here is nausea, not a blemish.
std::atomic<float> g_find_x{0}, g_find_y{0}, g_find_z{0};
std::atomic<bool>  g_find_armed{false};

void find_object_by_position() {
    if (!g_find_armed.exchange(false, std::memory_order_relaxed)) return;

    const float tx = g_find_x.load(), ty = g_find_y.load(), tz = g_find_z.load();
    const uintptr_t tls_array = (uintptr_t)__readgsqword(0x58);
    uintptr_t block = 0, ctx = 0, table = 0;
    if (!read_ptr(tls_array + (uintptr_t)g_tls_index * 8, &block) ||
        !read_ptr(block + OFF_CTX_IN_TLS, &ctx) ||
        !read_ptr(ctx + OFF_OBJ_TABLE, &table)) {
        API::get()->log_info("[Halo-CampE-UEVR] BLAMFIND: chain unavailable");
        return;
    }

    API::get()->log_info("[Halo-CampE-UEVR] BLAMFIND: hunting objects at (%.3f,%.3f,%.3f)", tx, ty, tz);
    int found = 0;
    for (uint32_t idx = 0; idx < 512 && found < 12; ++idx) {
        uintptr_t obj = 0;
        if (!read_ptr(table + idx * OBJ_ENTRY_SIZE + OFF_OBJ_IN_ENTRY, &obj) || obj == 0) continue;
        if (IsBadReadPtr((const void*)obj, 0x400)) continue;
        const float* f = (const float*)obj;
        for (int o = 0; o < (0x400 / 4) - 2; ++o) {
            if (std::fabs(f[o] - tx) < 0.5f &&
                std::fabs(f[o+1] - ty) < 0.5f &&
                std::fabs(f[o+2] - tz) < 0.5f) {
                API::get()->log_info("[Halo-CampE-UEVR] BLAMFIND: handle=0x%04X obj=0x%llX pos@+0x%03X "
                                     "= (%.3f,%.3f,%.3f)", idx, (unsigned long long)obj,
                                     (unsigned)(o * 4), f[o], f[o+1], f[o+2]);
                ++found;
                break;   // one hit per object is enough to name it
            }
        }
    }
    if (found == 0) API::get()->log_info("[Halo-CampE-UEVR] BLAMFIND: no object matched that position");
}

// FIND THE AIM ANYWHERE IN THE OBJECT TABLE -- `blamscan`.
//
// Guessing which object is the player has now failed three ways: orientation correlation picked
// objects whose fields never move, position matching returns five co-located candidates, and any
// handle noted in a previous session is stale. So stop guessing: sweep EVERY object at EVERY
// offset for a unit vector that equals the current aim, do it at two different aims, and keep only
// the offsets that match both. A field that tracks the aim across two poses is the aim; scenery and
// idle animation cannot fake that twice.
//
//   blamscan=1  -> collect candidates at the current aim
//   (change the aim)
//   blamscan=2  -> re-test the candidates and report the survivors
constexpr uint32_t SCAN_MAX      = 4096;
constexpr uint32_t SCAN_OBJECTS  = 512;
constexpr uint32_t SCAN_BYTES    = 0x600;

// How many floats can actually be read from `obj`, capped at maxBytes.
//
// IsBadReadPtr does NOT reliably catch a read that starts in a valid page and runs off the end of
// the region -- which is exactly what a fixed 0x600 sweep does on a small object. That silently
// faulted mid-scan and killed the scan with no output. VirtualQuery gives the real committed
// extent, so the loop can stop at the boundary instead of walking off it.
uint32_t readable_floats(uintptr_t obj, uint32_t maxBytes) {
    MEMORY_BASIC_INFORMATION mbi{};
    if (VirtualQuery((const void*)obj, &mbi, sizeof(mbi)) == 0) return 0;
    if (mbi.State != MEM_COMMIT) return 0;
    const DWORD readable = PAGE_READONLY | PAGE_READWRITE | PAGE_WRITECOPY |
                           PAGE_EXECUTE_READ | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY;
    if ((mbi.Protect & readable) == 0) return 0;
    if (mbi.Protect & (PAGE_GUARD | PAGE_NOACCESS)) return 0;
    const uintptr_t end = (uintptr_t)mbi.BaseAddress + mbi.RegionSize;
    uintptr_t avail = (end > obj) ? (end - obj) : 0;
    if (avail > maxBytes) avail = maxBytes;
    return (uint32_t)(avail / 4);
}

struct ScanHit { uint16_t obj; uint16_t off; };
ScanHit  g_scan[SCAN_MAX];
uint32_t g_scan_n = 0;
int      g_scan_phase = 0;

bool aim_triplet_matches(const float* f, float ax, float ay, float az) {
    const float m = std::sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
    if (m < 0.97f || m > 1.03f) return false;             // must be a unit vector
    const float d = (f[0]*ax + f[1]*ay + f[2]*az) / m;    // Blam negates Y; caller pre-negates
    return d > 0.995f;
}

void scan_for_aim() {
    const int want = g_cfg.blam_scan;
    if (want == 0 || want == g_scan_phase) return;
    g_scan_phase = want;

    const float ax =  g_aim_fx.load();
    const float ay = -g_aim_fy.load();   // to Blam convention
    const float az =  g_aim_fz.load();

    uintptr_t tls = (uintptr_t)__readgsqword(0x58), block = 0, ctx = 0, table = 0;
    if (!read_ptr(tls + (uintptr_t)g_tls_index * 8, &block) ||
        !read_ptr(block + OFF_CTX_IN_TLS, &ctx) ||
        !read_ptr(ctx + OFF_OBJ_TABLE, &table)) { return; }

    if (want == 1) {
        g_scan_n = 0;
        for (uint32_t i = 0; i < SCAN_OBJECTS && g_scan_n < SCAN_MAX; ++i) {
            uintptr_t obj = 0;
            if (!read_ptr(table + i * OBJ_ENTRY_SIZE + OFF_OBJ_IN_ENTRY, &obj) || obj == 0) continue;
            const uint32_t nf = readable_floats(obj, SCAN_BYTES);
            if (nf < 3) continue;
            const float* f = (const float*)obj;
            for (uint32_t o = 0; o + 2 < nf && g_scan_n < SCAN_MAX; ++o) {
                if (aim_triplet_matches(f + o, ax, ay, az)) {
                    g_scan[g_scan_n].obj = (uint16_t)i;
                    g_scan[g_scan_n].off = (uint16_t)(o * 4);
                    ++g_scan_n;
                }
            }
        }
        API::get()->log_info("[Halo-CampE-UEVR] BLAMSCAN pass1: %u candidates at aim (%.4f,%.4f,%.4f)",
                             g_scan_n, ax, ay, az);
        return;
    }

    // pass 2: the aim has moved, so only fields that FOLLOWED it are real
    uint32_t kept = 0;
    for (uint32_t k = 0; k < g_scan_n; ++k) {
        uintptr_t obj = 0;
        if (!read_ptr(table + (uintptr_t)g_scan[k].obj * OBJ_ENTRY_SIZE + OFF_OBJ_IN_ENTRY, &obj) || obj == 0) continue;
        if (IsBadReadPtr((const void*)(obj + g_scan[k].off), 12)) continue;
        const float* f = (const float*)(obj + g_scan[k].off);
        if (!aim_triplet_matches(f, ax, ay, az)) continue;
        if (kept < 24) {
            API::get()->log_info("[Halo-CampE-UEVR] BLAMSCAN HIT obj=0x%04X +0x%03X = (%.4f,%.4f,%.4f)",
                                 g_scan[k].obj, g_scan[k].off, f[0], f[1], f[2]);
        }
        ++kept;
    }
    API::get()->log_info("[Halo-CampE-UEVR] BLAMSCAN pass2: %u of %u survived at aim (%.4f,%.4f,%.4f)",
                         kept, g_scan_n, ax, ay, az);
}

// SCALAR AIM SCAN -- `blamscan2`. Closes the one gap the vector scan cannot see.
//
// blamscan proved no stored unit VECTOR drives the shot. But an aim held as Euler scalars -- a yaw
// and a pitch float -- would be invisible to it. This sweeps single floats instead, against every
// plausible encoding, and keeps only offsets that track the aim across two poses.
//
// Encodings tried, because the convention is not known in advance: radians and degrees, Blam's
// Y-negated yaw and UE's, and both pitch signs.
constexpr int SCALAR_ENCODINGS = 8;

const char* scalar_name(int i) {
    switch (i) {
        case 0: return "yaw_rad(blam)"; case 1: return "yaw_rad(ue)";
        case 2: return "yaw_deg(blam)"; case 3: return "yaw_deg(ue)";
        case 4: return "pitch_rad";     case 5: return "pitch_rad_neg";
        case 6: return "pitch_deg";     default: return "pitch_deg_neg";
    }
}

void scalar_candidates(float* out) {
    const float fx = g_aim_fx.load(), fy = g_aim_fy.load(), fz = g_aim_fz.load();
    const float yb = std::atan2(-fy, fx), yu = std::atan2(fy, fx);
    const float p  = std::asin(fz < -1.0f ? -1.0f : (fz > 1.0f ? 1.0f : fz));
    out[0] = yb;  out[1] = yu;  out[2] = yb * 57.2957795f; out[3] = yu * 57.2957795f;
    out[4] = p;   out[5] = -p;  out[6] = p * 57.2957795f;  out[7] = -p * 57.2957795f;
}

// Yaw wraps, so 179 deg and -181 deg are the same angle; comparing raw would miss it.
bool scalar_close(float a, float b, bool isDeg, bool isYaw) {
    // REJECT JUNK FIRST. Uninitialised memory is full of values like 1e30, and the old
    // `while (d > half) d -= full;` normalisation would then loop ~1e27 times -- an effective hang.
    // That is what made this scanner vanish after its entry log with no crash and no result.
    if (!std::isfinite(a) || std::fabs(a) > 1.0e6f) return false;
    float d = a - b;
    if (isYaw) {
        const float full = isDeg ? 360.0f : 6.28318531f;
        d -= std::floor((d + full * 0.5f) / full) * full;   // modulo, not a loop
    }
    return std::fabs(d) < (isDeg ? 0.75f : 0.013f);
}

struct ScalarHit { uint16_t obj; uint16_t off; uint8_t enc; };
ScalarHit g_sc[SCAN_MAX];
uint32_t  g_sc_n = 0;
int       g_sc_phase = 0;

void scan_for_aim_scalars() {
    const int want = g_cfg.blam_scan2;
    if (want == 0 || want == g_sc_phase) return;
    g_sc_phase = want;

    float cand[SCALAR_ENCODINGS];
    scalar_candidates(cand);

    // Log on ENTRY. Both scanners used to return silently when the chain failed, which is
    // indistinguishable from "never ran" and cost a whole test cycle to notice.
    API::get()->log_info("[Halo-CampE-UEVR] BLAMSCAN2 enter: phase=%d yawB=%.4f pitch=%.4f", want,
                         cand[0], cand[4]);

    uintptr_t tls = (uintptr_t)__readgsqword(0x58), block = 0, ctx = 0, table = 0;
    if (!read_ptr(tls + (uintptr_t)g_tls_index * 8, &block) || block == 0 ||
        !read_ptr(block + OFF_CTX_IN_TLS, &ctx) || ctx == 0 ||
        !read_ptr(ctx + OFF_OBJ_TABLE, &table) || table == 0) {
        API::get()->log_info("[Halo-CampE-UEVR] BLAMSCAN2: chain failed (block=0x%llX ctx=0x%llX table=0x%llX)",
                             (unsigned long long)block, (unsigned long long)ctx, (unsigned long long)table);
        return;
    }

    if (want == 1) {
        g_sc_n = 0;
        for (uint32_t i = 0; i < SCAN_OBJECTS && g_sc_n < SCAN_MAX; ++i) {
            uintptr_t obj = 0;
            if (!read_ptr(table + i * OBJ_ENTRY_SIZE + OFF_OBJ_IN_ENTRY, &obj) || obj == 0) continue;
            const uint32_t nf = readable_floats(obj, SCAN_BYTES);
            if (nf == 0) continue;
            const float* f = (const float*)obj;
            for (uint32_t o = 0; o < nf && g_sc_n < SCAN_MAX; ++o) {
                for (int e = 0; e < SCALAR_ENCODINGS; ++e) {
                    const bool isDeg = (e == 2 || e == 3 || e == 6 || e == 7);
                    const bool isYaw = (e < 4);
                    // Skip near-zero targets: everything in memory is 0, so they match by accident.
                    if (std::fabs(cand[e]) < (isDeg ? 6.0f : 0.10f)) continue;
                    if (scalar_close(f[o], cand[e], isDeg, isYaw)) {
                        g_sc[g_sc_n].obj = (uint16_t)i;
                        g_sc[g_sc_n].off = (uint16_t)(o * 4);
                        g_sc[g_sc_n].enc = (uint8_t)e;
                        ++g_sc_n;
                        break;
                    }
                }
            }
        }
        API::get()->log_info("[Halo-CampE-UEVR] BLAMSCAN2 pass1: %u scalar candidates", g_sc_n);
        return;
    }

    uint32_t kept = 0;
    for (uint32_t k = 0; k < g_sc_n; ++k) {
        uintptr_t obj = 0;
        if (!read_ptr(table + (uintptr_t)g_sc[k].obj * OBJ_ENTRY_SIZE + OFF_OBJ_IN_ENTRY, &obj) || obj == 0) continue;
        if (IsBadReadPtr((const void*)(obj + g_sc[k].off), 4)) continue;
        const int e = g_sc[k].enc;
        const bool isDeg = (e == 2 || e == 3 || e == 6 || e == 7);
        const bool isYaw = (e < 4);
        if (std::fabs(cand[e]) < (isDeg ? 6.0f : 0.10f)) continue;
        const float v = *(const float*)(obj + g_sc[k].off);
        if (!scalar_close(v, cand[e], isDeg, isYaw)) continue;
        if (kept < 32) {
            API::get()->log_info("[Halo-CampE-UEVR] BLAMSCAN2 HIT obj=0x%04X +0x%03X = %.5f  (%s, want %.5f)",
                                 g_sc[k].obj, g_sc[k].off, v, scalar_name(e), cand[e]);
        }
        ++kept;
    }
    API::get()->log_info("[Halo-CampE-UEVR] BLAMSCAN2 pass2: %u of %u survived", kept, g_sc_n);
}

#if HALO_VR_DEV
// ---------------------------------------------------------------------------------------------
// BLAMSCAN3 -- hunt the aim in the sim module's GLOBALS, not the object table.
//
// Every previous scan walked the object table and found nothing usable: 11 vector fields that
// mirror the aim but which the shot ignores, and zero Euler-scalar matches. That is the expected
// result if the authoritative aim is not object state at all. In Blam the player's aim lives in a
// per-player control global (classically s_player_control.desired_angles, a real_euler_angles2d --
// yaw and pitch as two adjacent radian floats), which is a static global, NOT an object. So it was
// never in the search space, and no amount of extra objects or poses could have found it.
//
// The PAIR is the discriminator. A lone float near the yaw value matches all over memory; two
// ADJACENT floats matching yaw and pitch simultaneously, at two different poses, does not.
// ---------------------------------------------------------------------------------------------
constexpr uint32_t GSCAN_MAX = 4096;

struct GHit { uintptr_t addr; uint8_t enc; };
GHit     g_gh[GSCAN_MAX];
uint32_t g_gh_n = 0;
int      g_gh_phase = 0;

// Pose captured at pass 1, so pass 2 can auto-fire once the aim has moved far enough from it.
bool  g_gh_armed = false;
float g_gh_ax = 0.0f, g_gh_ay = 0.0f, g_gh_az = 0.0f;

// Settle tracker. Pass 2 fires on angular separation, which is reached WHILE the view is still
// turning -- and in motion the plugin's aim sample and the sim's in-memory mirrors are skewed by
// several degrees, far more than the match tolerance. Sampling mid-turn therefore rejects every
// real candidate. This is not a nicety: it is why an object-memory sweep that DID include the 11
// known aim mirrors still reported "0 of 2596 survived".
float g_prev_ax = 0.0f, g_prev_ay = 0.0f, g_prev_az = 0.0f;
int   g_settled = 0;

// The single (yaw,pitch) Euler pair among the survivors -- the desired_angles candidate. Heap, so
// the absolute address changes every session; it is re-derived by each scan, never hard-coded.
std::atomic<uintptr_t> g_angles_addr{0};
int g_angles_enc = 0;

const char* gscan_name(int e) {
    switch (e) {
        case 0:  return "(yaw,pitch)";
        case 1:  return "(pitch,yaw)";
        case 2:  return "vec3";
        case 3:  return "vec3negY";
        default: return "?";
    }
}

// True aim, as the two scalars and the forward vector, from ControlRotation.
void gscan_targets(float* yaw, float* pitch, float* fx, float* fy, float* fz) {
    *fx = g_aim_fx.load(); *fy = g_aim_fy.load(); *fz = g_aim_fz.load();
    *yaw   = std::atan2(-(*fy), *fx);                                     // Blam negates Y
    const float c = *fz < -1.0f ? -1.0f : (*fz > 1.0f ? 1.0f : *fz);
    *pitch = std::asin(c);
}

bool gscan_match(const float* f, int enc, float yaw, float pitch,
                 float fx, float fy, float fz) {
    switch (enc) {
        case 0: return scalar_close(f[0], yaw, false, true) && scalar_close(f[1], pitch, false, false);
        case 1: return scalar_close(f[0], pitch, false, false) && scalar_close(f[1], yaw, false, true);
        case 2: return std::fabs(f[0]-fx) < 0.02f && std::fabs(f[1]-fy) < 0.02f && std::fabs(f[2]-fz) < 0.02f;
        case 3: return std::fabs(f[0]-fx) < 0.02f && std::fabs(f[1]+fy) < 0.02f && std::fabs(f[2]-fz) < 0.02f;
        default: return false;
    }
}

// One region's worth of the sweep, SEH-guarded: a region that is MEM_COMMIT at VirtualQuery time
// can be freed by another thread while we walk it. Kept in its own function because MSVC refuses
// __try in a function that needs object unwinding.
static void gscan_region(const float* f, uint32_t n, float yaw, float pitch,
                         float fx, float fy, float fz, bool* capped) {
    __try {
        for (uint32_t i = 0; i + 2 < n; ++i) {
            for (int e = 0; e < 4; ++e) {
                if (!gscan_match(f + i, e, yaw, pitch, fx, fy, fz)) continue;
                if (g_gh_n < GSCAN_MAX) {
                    g_gh[g_gh_n].addr = (uintptr_t)(f + i);
                    g_gh[g_gh_n].enc  = (uint8_t)e;
                    ++g_gh_n;
                } else {
                    *capped = true;
                }
                break;
            }
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        // region went away mid-sweep; skip it rather than take the process down
    }
}

// Collect pass. Walks EVERY committed writable region the process owns -- heap as well as module
// .data. The module-only sweep returned 0 survivors across two genuinely separated poses, which
// excludes static globals but not the aim: Blam's live state here is reached TLS -> sim context ->
// heap (that is how the object table is found), so the control block is almost certainly a heap
// allocation. Third search space; the first two are excluded by measurement, not by assumption.
// Targeted sweep: only memory reachable in ONE pointer hop from the sim context.
//
// The first attempt walked the entire user address space and HUNG the game -- UE5 has many GB of
// private commit, and `responding=False` inside ~30s. Unbounded was never viable. This is also the
// better search on its own merits: the object table is found at ctx+0x50, so Blam's live per-player
// state is almost certainly in that same allocation or a sibling one hop away.
void gscan_collect(float yaw, float pitch, float fx, float fy, float fz) {
    g_gh_n = 0;
    bool capped = false;
    uint64_t floats_seen = 0;
    uint32_t regions = 0;

    // Use the LATCHED chain: this runs on the UEVR frame thread, where the sim's thread-local
    // context does not resolve. Deriving it here returns ctx=0 and scans nothing.
    const uintptr_t ctx   = g_ctx_cached.load(std::memory_order_relaxed);
    const uintptr_t table = g_table_cached.load(std::memory_order_relaxed);
    if (ctx == 0 || table == 0) {
        API::get()->log_info("[Halo-CampE-UEVR] BLAMSCAN3: sim chain not latched yet "
                             "(ctx=0x%llX table=0x%llX) -- the orientation getter must run at least "
                             "once first; it is silent outside a live mission",
                             (unsigned long long)ctx, (unsigned long long)table);
        return;
    }

    constexpr int MAX_SEEDS = 1024;
    static uintptr_t seeds[MAX_SEEDS];
    int ns = 0;
    seeds[ns++] = ctx;
    seeds[ns++] = table;
    for (uint32_t off = 0; off < 0x800 && ns < MAX_SEEDS; off += 8) {
        uintptr_t v = 0;
        if (read_ptr(ctx + off, &v) && v != 0) seeds[ns++] = v;
    }

    // POSITIVE CONTROL -- seed from the OBJECTS as well, not just the table that points at them.
    // We already know 11 vector fields inside objects track the aim, so a sweep that cannot
    // rediscover those is not evidence of absence, it is evidence the sweep missed. Without this,
    // "0 of N survived" means nothing: the first heap run reported exactly that while never having
    // looked at object memory at all.
    for (uint32_t i = 0; i < SCAN_OBJECTS && ns < MAX_SEEDS; ++i) {
        uintptr_t obj = 0;
        if (read_ptr(table + (uintptr_t)i * OBJ_ENTRY_SIZE + OFF_OBJ_IN_ENTRY, &obj) && obj != 0) {
            seeds[ns++] = obj;
        }
    }

    constexpr uintptr_t MAX_REGION = 64ull * 1024 * 1024;   // larger = asset pool, not game state
    static uintptr_t done[MAX_SEEDS];   // static: 1024 seeds is too much for the stack
    int nd = 0;
    for (int i = 0; i < ns; ++i) {
        MEMORY_BASIC_INFORMATION mbi{};
        if (VirtualQuery((const void*)seeds[i], &mbi, sizeof(mbi)) == 0) continue;
        if (mbi.State != MEM_COMMIT) continue;
        const DWORD writable = PAGE_READWRITE | PAGE_WRITECOPY
                             | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY;
        if ((mbi.Protect & writable) == 0) continue;
        if (mbi.Protect & (PAGE_GUARD | PAGE_NOACCESS)) continue;

        const uintptr_t base = (uintptr_t)mbi.BaseAddress;
        bool seen = false;
        for (int k = 0; k < nd; ++k) if (done[k] == base) { seen = true; break; }
        if (seen) continue;
        if (nd < MAX_SEEDS) done[nd++] = base;
        if (mbi.RegionSize > MAX_REGION) continue;

        const uint32_t n = (uint32_t)(mbi.RegionSize / 4);
        gscan_region((const float*)base, n, yaw, pitch, fx, fy, fz, &capped);
        floats_seen += n;
        ++regions;
    }

    API::get()->log_info("[Halo-CampE-UEVR] BLAMSCAN3 pass1: %u candidates over %llu floats "
                         "in %u regions (%d seeds)%s", g_gh_n, (unsigned long long)floats_seen,
                         regions, ns, capped ? "  *** CAP HIT - RESULTS TRUNCATED ***" : "");
}

// Confirm pass. COMPACTS g_gh in place to the survivors, so repeated poses INTERSECT instead of
// re-collecting from scratch. One pose leaves ~2500 aim mirrors standing; intersecting across
// several distinct poses is what actually narrows it. Scalar (yaw,pitch) pairs are logged in full
// because they are the shape we are hunting -- vec3 mirrors are known-uninteresting and capped.
uint32_t gscan_confirm(float yaw, float pitch, float fx, float fy, float fz) {
    uint32_t kept = 0;
    uint32_t vec_logged = 0;
    for (uint32_t k = 0; k < g_gh_n; ++k) {
        if (IsBadReadPtr((const void*)g_gh[k].addr, 12)) continue;
        const float* f = (const float*)g_gh[k].addr;
        if (!gscan_match(f, g_gh[k].enc, yaw, pitch, fx, fy, fz)) continue;

        const uintptr_t a = g_gh[k].addr;
        const bool inMod = (a >= g_sim_base && a < g_sim_base + 0x3000000ull);
        const bool scalar = (g_gh[k].enc < 2);
        if (scalar || vec_logged < 6) {
            API::get()->log_info("[Halo-CampE-UEVR] BLAMSCAN3 HIT 0x%llX (%s%s) = %.5f %.5f %.5f",
                                 (unsigned long long)a, gscan_name(g_gh[k].enc),
                                 inMod ? ",module" : ",heap", f[0], f[1], f[2]);
            if (!scalar) ++vec_logged;
        }
        g_gh[kept++] = g_gh[k];      // compact in place
    }
    g_gh_n = kept;
    return kept;
}

// blamangles=1: force the candidate Euler pair to the controller aim EVERY frame.
//
// An async poke lost the race last time -- the field is rewritten each tick, so a one-shot write is
// gone before anything reads it. Writing every frame from the plugin's own tick is the only way to
// find out whether this is the authoritative input or one more mirror. If the view follows, it is
// upstream of the animation and therefore the replicated quantity we actually want for MP.
// Pick the surviving scalar pair. MUST be called AFTER intersection, not at collect time: a fresh
// collect turns up several scalar pairs and the first one found is usually a coincidence -- one
// session latched a (pitch,yaw) pair that the very next pose eliminated. Only the pair that
// survives a large pose change is the candidate.
void latch_angles_candidate() {
    g_angles_addr.store(0, std::memory_order_relaxed);
    uint32_t scalars = 0;
    for (uint32_t k = 0; k < g_gh_n; ++k) {
        if (g_gh[k].enc >= 2) continue;
        ++scalars;
        if (g_angles_addr.load(std::memory_order_relaxed) == 0) {
            g_angles_addr.store(g_gh[k].addr, std::memory_order_relaxed);
            g_angles_enc = g_gh[k].enc;
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] BLAMSCAN3 angles candidate: 0x%llX (%s) "
                         "[%u scalar pair(s) surviving]",
                         (unsigned long long)g_angles_addr.load(std::memory_order_relaxed),
                         gscan_name(g_angles_enc), scalars);
}


void scan_globals() {
    if (g_sim_base == 0) return;
    const int want = g_cfg.blam_scan3;

    // Track whether the aim has stopped moving (see g_settled). Both passes must sample at rest.
    {
        const float ax = g_aim_fx.load(), ay = g_aim_fy.load(), az = g_aim_fz.load();
        float dp = ax * g_prev_ax + ay * g_prev_ay + az * g_prev_az;
        dp = dp > 1.0f ? 1.0f : (dp < -1.0f ? -1.0f : dp);
        if (std::acos(dp) < 0.004f) { if (g_settled < 1000) ++g_settled; }
        else                        { g_settled = 0; }
        g_prev_ax = ax; g_prev_ay = ay; g_prev_az = az;
    }
    const bool settled = (g_settled >= 4);

    // Auto-fire pass 2 the moment the aim has moved far enough, with the separation test done HERE
    // rather than by an external script. That external timing coupling is exactly why pass 2 ran at
    // the same pose last time, which let a coincidence survive and get written up as a candidate.
    if (g_gh_armed) {
        const float ax = g_aim_fx.load(), ay = g_aim_fy.load(), az = g_aim_fz.load();
        float d = ax * g_gh_ax + ay * g_gh_ay + az * g_gh_az;
        d = d > 1.0f ? 1.0f : (d < -1.0f ? -1.0f : d);
        const float sep = std::acos(d);
        if (sep <= 0.60f) return;
        if (!settled) return;   // wait for the turn to STOP; mid-turn skew rejects real candidates

        float yaw = 0, pitch = 0, fx = 0, fy = 0, fz = 0;
        gscan_targets(&yaw, &pitch, &fx, &fy, &fz);
        if (std::fabs(pitch) < 0.10f || std::fabs(yaw) < 0.10f) return;   // keep waiting

        API::get()->log_info("[Halo-CampE-UEVR] BLAMSCAN3 pass2 auto-fire: separation=%.1fdeg "
                             "yaw=%.5f pitch=%.5f aim=(%.4f,%.4f,%.4f)",
                             sep * 57.2957795f, yaw, pitch, fx, fy, fz);
        const uint32_t before = g_gh_n;
        const uint32_t kept = gscan_confirm(yaw, pitch, fx, fy, fz);
        API::get()->log_info("[Halo-CampE-UEVR] BLAMSCAN3 intersect: %u of %u survived "
                             "(pose separation %.1fdeg)", kept, before, sep * 57.2957795f);
        if (kept == 0) {
            API::get()->log_info("[Halo-CampE-UEVR] BLAMSCAN3: exhausted -- nothing tracks the aim "
                                 "across these poses");
            g_gh_armed = false; g_gh_phase = 0; g_cfg.blam_scan3 = 0;
            return;
        }
        // Re-arm at THIS pose and keep intersecting on the next move. Never re-collect: the whole
        // point is to whittle one candidate set down, not to start over each time.
        latch_angles_candidate();
        g_gh_ax = fx; g_gh_ay = fy; g_gh_az = fz;
        g_gh_armed = true;
        return;
    }

    if (want == 0 || want == g_gh_phase) return;
    if (!settled) return;   // arm only from a stationary pose, for the same reason as pass 2
    g_gh_phase = want;

    float yaw = 0, pitch = 0, fx = 0, fy = 0, fz = 0;
    gscan_targets(&yaw, &pitch, &fx, &fy, &fz);
    API::get()->log_info("[Halo-CampE-UEVR] BLAMSCAN3 enter: phase=%d yaw=%.5f pitch=%.5f "
                         "aim=(%.4f,%.4f,%.4f)", want, yaw, pitch, fx, fy, fz);

    if (std::fabs(yaw) < 0.10f || std::fabs(pitch) < 0.10f) {
        API::get()->log_info("[Halo-CampE-UEVR] BLAMSCAN3: aim too close to zero to discriminate "
                             "-- look away from centre and retry");
        g_gh_phase = 0;     // let it be retried rather than eating the trigger
        return;
    }

    gscan_collect(yaw, pitch, fx, fy, fz);
    g_gh_ax = fx; g_gh_ay = fy; g_gh_az = fz;
    latch_angles_candidate();
    g_gh_armed = true;
    API::get()->log_info("[Halo-CampE-UEVR] BLAMSCAN3 armed: pass2 auto-fires once the aim moves "
                         ">34deg from here");
}
#endif  // HALO_VR_DEV

uintptr_t hooked_get_orientation(uintptr_t handle, Vec3f* outA, Vec3f* outB) {
    const uintptr_t ret = g_original ? g_original(handle, outA, outB) : 0;

    // Drive the angular control state from here: this hook actually runs on the sim thread
    // (~2600 calls/sec), whereas the setter hook at 0x1842C0 installs but is never invoked
    // during normal player aiming.
    drive_control_angles();
    g_calls.fetch_add(1, std::memory_order_relaxed);

    // Latch the thread-local sim chain while we are ON the sim thread (see g_ctx_cached).
    {
        const uintptr_t tls_array = (uintptr_t)__readgsqword(0x58);
        uintptr_t blk = 0, cx = 0, tbl = 0;
        if (read_ptr(tls_array + (uintptr_t)g_tls_index * 8, &blk) && blk != 0 &&
            read_ptr(blk + OFF_CTX_IN_TLS, &cx) && cx != 0 &&
            read_ptr(cx + OFF_OBJ_TABLE, &tbl) && tbl != 0) {
            g_ctx_cached.store(cx, std::memory_order_relaxed);
            g_table_cached.store(tbl, std::memory_order_relaxed);
        }
    }

    // READ-ONLY FROM HERE DOWN. Every one of these looks at the sim and reports; none of them
    // writes it. The only write on this path is drive_control_angles() above, which is the shipping
    // aim and lives in BlamDrive.cpp. Keep it that way -- a scan that also writes is a scan whose
    // results you cannot trust.
    trace_chain_once(handle);
    find_object_by_position();
    scan_for_aim();
    scan_for_aim_scalars();
    dump_object_once(handle);
    const uintptr_t obj = resolve_object(handle);
    if (obj != 0) {
        g_chain_ok.store(true, std::memory_order_relaxed);
        g_last_obj.store(obj, std::memory_order_relaxed);
        g_last_handle.store((uint32_t)(handle & 0xFFFF), std::memory_order_relaxed);
    }
    if (outA != nullptr) {
        g_last_ax.store(outA->x, std::memory_order_relaxed);
        g_last_ay.store(outA->y, std::memory_order_relaxed);
        g_last_az.store(outA->z, std::memory_order_relaxed);
    }
    if (outB != nullptr) {
        g_last_bx.store(outB->x, std::memory_order_relaxed);
        g_last_by.store(outB->y, std::memory_order_relaxed);
        g_last_bz.store(outB->z, std::memory_order_relaxed);
    }

    // Score this call against the live aim and keep the best per handle.
    if (outA != nullptr && g_stats_armed.load(std::memory_order_relaxed)) {
        const uint32_t h = (uint32_t)(handle & 0xFFFF);
        if (h < HANDLE_SLOTS) {
            const float d = outA->x * g_aim_fx.load(std::memory_order_relaxed)
                          + outA->y * g_aim_fy.load(std::memory_order_relaxed)
                          + outA->z * g_aim_fz.load(std::memory_order_relaxed);
            const float ad = d < 0.0f ? -d : d;
            if (ad > g_best_dot[h].load(std::memory_order_relaxed)) {
                g_best_dot[h].store(ad, std::memory_order_relaxed);
                g_hx[h].store(outA->x, std::memory_order_relaxed);
                g_hy[h].store(outA->y, std::memory_order_relaxed);
                g_hz[h].store(outA->z, std::memory_order_relaxed);
            }
            g_hits[h].fetch_add(1, std::memory_order_relaxed);
        }
    }
    return ret;
}

uintptr_t hooked_create_projectile(uintptr_t params) {
    g_spawns.fetch_add(1, std::memory_order_relaxed);

    // WHICH of the 55 call sites is this? The statically-traced caller uses the orientation getter,
    // but overriding that getter did not move the spawn direction -- so this weapon takes a
    // different route. The return address says exactly which one, instead of guessing among 55.
    const uintptr_t ra = (uintptr_t)_ReturnAddress();
    const uintptr_t rva = (g_sim_base && ra > g_sim_base) ? (ra - g_sim_base) : 0;
    // Read the direction fields BEFORE the call: this is what the game is about to fire along.
    float v1[3]{}, v2[3]{}, v3[3]{};
    if (params != 0 && !IsBadReadPtr((const void*)params, 0x60)) {
        memcpy(v1, (const void*)(params + P_VEC1), sizeof(v1));
        memcpy(v2, (const void*)(params + P_VEC2), sizeof(v2));
        memcpy(v3, (const void*)(params + P_VEC3), sizeof(v3));
    }
    // REDIRECT (blamaim >= 3): rewrite the spawn direction to the aim we actually want.
    //
    // The game computes this field as normalize(target - origin), where target is a world point it
    // traced to. That is why writing ControlRotation never moved the shot: the point is produced
    // upstream and the direction is derived from it, so overriding any orientation field downstream
    // of that trace changes nothing. Writing the finished direction here is the one place it is
    // guaranteed to be the value the projectile is built from.
    //
    // Y IS NEGATED. Measured, not assumed: with the plugin off, a spawn read
    //   v2  = (-0.9372, -0.2222, 0.2689)
    //   aim = (-0.9372, +0.2222, 0.2689)
    // -- identical but for the sign of Y, so Blam's handedness differs from UE's on that axis.
    // Source from the CONTROLLER, not ControlRotation. Under direct drive the aim chain is pinned,
    // so g_aim_* is frozen -- redirecting to it would fire every round at the stale direction while
    // the log happily reported zero error. Computed unconditionally because the ERR metric below
    // needs the same reference whether or not the redirect is armed.
    float want_yaw = 0.0f, want_pitch = 0.0f;
    const bool have_want = controller_desired_aim(&want_yaw, &want_pitch);

    // OWNER HUNT (blamdump=1): dump the head of the params struct as dwords so a player shot can be
    // diffed against an AI shot. Filtering by CALL SITE cannot work -- dll+0x5D124E carried 2266 of
    // 2869 spawns, i.e. it is the shared "something fired" path, not the player's. The only thing
    // that can separate them is an owner/shooter field inside these params.
    // Fire with no AI shooting to capture pure-player rows, then in combat for mixed rows.
    if (g_cfg.blam_dump != 0 && params != 0 && !IsBadReadPtr((const void*)params, 0x80)) {
        // The cap re-arms whenever blamdump CHANGES, so a fresh capture is one config edit away
        // rather than a relaunch. Bump it to 2, 3, ... to take another 40 rows.
        static std::atomic<uint32_t> dumped{0};
        static std::atomic<int>      armed_as{0};
        const int want = g_cfg.blam_dump;
        if (armed_as.exchange(want, std::memory_order_relaxed) != want) {
            dumped.store(0, std::memory_order_relaxed);
        }
        if (dumped.fetch_add(1, std::memory_order_relaxed) < 40) {
            const uint32_t* d = (const uint32_t*)params;
            char buf[512];
            int n = 0;
            for (int i = 0; i < 32 && n < (int)sizeof(buf) - 12; ++i) {
                n += snprintf(buf + n, sizeof(buf) - n, "%08X ", d[i]);
            }
            API::get()->log_info("[Halo-CampE-UEVR] BLAMPARAMS caller=dll+0x%llX | %s",
                                 (unsigned long long)rva, buf);
        }
    }

    // (The `blamaim >= 3` SPAWN REDIRECT was here: it rewrote the direction at params+0x28 from the
    // controller. Removed with the rest of the write paths. The field visibly changed before the
    // constructor read it -- the write landed -- and ~360 rounds fired at a 90 degree offset went
    // exactly where they always went, so the direction is taken from somewhere else by then. The
    // caller filter that guarded it (blamcaller/blamcaller2, so the player's aim was not imposed on
    // AI and teammates' shots) went with it.
    //
    // What remains here is a READOUT, which is the useful half: v2 is what the game is about to fire
    // along, straight from the spawn params, and it is the only ground truth for that.)

    // Arm the position hunt from a real shot: v1 is the muzzle origin, i.e. where the player is.
    if (g_cfg.blam_find != 0 && !g_find_armed.load(std::memory_order_relaxed)) {
        g_find_x.store(v1[0]); g_find_y.store(v1[1]); g_find_z.store(v1[2]);
        g_find_armed.store(true, std::memory_order_relaxed);
    }

    // (A `was` snapshot and a re-read of params+0x28 lived here. Both existed to prove the redirect
    // had landed -- pre-write value vs post-write value. With no write on this path they were the
    // same number printed twice.)

    // PER-SHOT AIM ERROR -- the point of this hook now that the aim hunt is settled.
    //
    // The shot direction tracks the aim to 0.000 deg, so THIS is the honest answer to "is the aim
    // correct while firing": the angle between where the player is pointing (g_desired_*, set from
    // the controller) and where the round actually left. Derived from v2 rather than from
    // ControlRotation on purpose -- ControlRotation is an output mirror, so measuring against it
    // would compare the actuator to itself and always read zero.
    const float shot_x =  v2[0];
    const float shot_y = -v2[1];        // Blam negates Y; back to UE frame to compare with desired
    const float shot_z =  v2[2];
    const float shot_yaw   = std::atan2(shot_y, shot_x) * 57.2957795f;
    const float shot_pitch = std::asin(std::fmax(-1.0f, std::fmin(1.0f, shot_z))) * 57.2957795f;

    // Against the CONTROLLER-derived aim, not g_desired_* -- that one is only maintained by the
    // stick control law, so in direct drive it is stale and every error reading is fiction.
    // -999 marks "no controller pose", so a dead reference can never be mistaken for a good score.
    const float dy  = have_want ? wrap180_local(want_yaw   - shot_yaw)   : -999.0f;
    const float dp  = have_want ? wrap180_local(want_pitch - shot_pitch) : -999.0f;
    const float err = have_want ? std::sqrt(dy * dy + dp * dp)           : -999.0f;
    API::get()->log_info("[Halo-CampE-UEVR] BLAMSPAWN #%llu v1=(%.4f,%.4f,%.4f) v2=(%.4f,%.4f,%.4f) "
                         "v3=(%.4f,%.4f,%.4f) aim=(%.4f,%.4f,%.4f) "
                         "shotYaw=%.2f shotPitch=%.2f dYaw=%.2f dPitch=%.2f ERR=%.2f "
                         "yawt=%.2f caller=dll+0x%llX",
                         (unsigned long long)g_spawns.load(std::memory_order_relaxed),
                         v1[0], v1[1], v1[2], v2[0], v2[1], v2[2],
                         v3[0], v3[1], v3[2],
                         g_aim_fx.load(), g_aim_fy.load(), g_aim_fz.load(),
                         shot_yaw, shot_pitch, dy, dp, err,
                         g_cfg.blam_yaw_off, (unsigned long long)rva);
    return g_orig_create ? g_orig_create(params) : 0;
}

} // namespace

void blam_aim_tick() {
    const int want = g_cfg.blam_aim;

    // ---- teardown
    if (want == 0) {
        if (g_hook_id >= 0) {
            API::get()->param()->functions->unregister_inline_hook(g_hook_id);
            g_hook_id = -1;
            g_original = nullptr;
            if (g_create_hook_id >= 0) {
                API::get()->param()->functions->unregister_inline_hook(g_create_hook_id);
                g_create_hook_id = -1;
                g_orig_create = nullptr;
            }
            // dll+0x5A6AD0 is handed back implicitly: ownership is `blamaim != 0`, which is already
            // 0 to have reached this branch, so blam_drive_tick() installs its own minimal hook on
            // the next frame and the aim write survives the diagnostics being switched off.
            API::get()->log_info("[Halo-CampE-UEVR] BLAMHOOK: removed");
        }
        g_prev_flag = 0;
        return;
    }

    // Drive the globals scan from the FRAME path, not from the orientation-getter hook. Anything
    // hung off a hook that turns out not to fire runs never, silently -- which is indistinguishable
    // from "scanned and found nothing", the exact failure mode that has already cost this
    // investigation days. Keep discovery work on a path you can prove executes.
    scan_globals();

    // ---- install once
    if (g_hook_id < 0) {
        // dll+0x5A6AD0 IS ALREADY OURS: ownership is `blamaim != 0`, and blam_drive_tick() ran
        // earlier in this same update() and removed its own hook on that basis. Do NOT reintroduce
        // a runtime claim here -- the first version did, and it raced. The claim only took effect on
        // the drive tick AFTER this one, so both hooks went onto the function in the same frame and
        // the later uninstall tore out the survivor's trampoline: "installed but NEVER CALLED", and
        // the aim stopped reaching the sim with nothing in the log saying why.
        HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
        if (sim == nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] BLAMHOOK: HaloSimulation_tag_release.dll not loaded");
            // Zeroing blamaim IS the handback: ownership is derived from it, so blam_drive_tick()
            // takes the address on the next frame and the aim write is never left dead.
            g_cfg.blam_aim = 0;
            return;
        }
        g_sim_base = (uintptr_t)sim;
        g_tls_index = *(const uint32_t*)(g_sim_base + RVA_TLS_INDEX);

        void* target = (void*)(g_sim_base + RVA_GET_ORIENTATION);
        const int id = API::get()->param()->functions->register_inline_hook(
            target, (void*)&hooked_get_orientation, (void**)&g_original);
        if (id < 0 || g_original == nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] BLAMHOOK: register_inline_hook FAILED (id=%d) on 0x%llX",
                                 id, (unsigned long long)target);
            // Same as above: zeroing blamaim hands the address to blam_drive_tick().
            g_cfg.blam_aim = 0;
            return;
        }
        g_hook_id = id;
        API::get()->log_info("[Halo-CampE-UEVR] BLAMHOOK: installed on 0x%llX (dll+0x%llX) id=%d tlsIndex=%u",
                             (unsigned long long)target, (unsigned long long)RVA_GET_ORIENTATION,
                             id, g_tls_index);

        void* ctarget = (void*)(g_sim_base + RVA_CREATE_PROJECTILE);
        const int cid = API::get()->param()->functions->register_inline_hook(
            ctarget, (void*)&hooked_create_projectile, (void**)&g_orig_create);
        if (cid < 0 || g_orig_create == nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] BLAMSPAWN: hook FAILED (id=%d) - spawn direction "
                                 "cannot be verified, only the getter is instrumented", cid);
        } else {
            g_create_hook_id = cid;
            API::get()->log_info("[Halo-CampE-UEVR] BLAMSPAWN: installed on 0x%llX (dll+0x%llX) id=%d",
                                 (unsigned long long)ctarget,
                                 (unsigned long long)RVA_CREATE_PROJECTILE, cid);
        }

        g_prev_flag = want;
        return;
    }

    // ---- periodic report from the TICK, comparing what the hook saw against the live aim.
    // Done here rather than in the hook so the correlation costs the sim thread nothing.
    static uint32_t s_tick = 0;
    if ((++s_tick % 4) != 0) return;

    const uint64_t calls = g_calls.load(std::memory_order_relaxed);
    if (calls == 0) {
        API::get()->log_info("[Halo-CampE-UEVR] BLAMHOOK: installed but NEVER CALLED - this getter is not "
                             "on the live fire path, or nothing has fired yet");
        return;
    }

    double aim_pitch = 0.0, aim_yaw = 0.0;
    if (!read_control_rotation(&aim_pitch, &aim_yaw, nullptr)) return;
    constexpr double D2R = 3.14159265358979323846 / 180.0;
    const double cp = std::cos(aim_pitch * D2R), sp = std::sin(aim_pitch * D2R);
    const double cy = std::cos(aim_yaw * D2R),   sy = std::sin(aim_yaw * D2R);
    const float fx = (float)(cp * cy), fy = (float)(cp * sy), fz = (float)sp;

    // Publish the aim for the hook to correlate against, and arm scoring on the first pass.
    g_aim_fx.store(fx, std::memory_order_relaxed);
    g_aim_fy.store(fy, std::memory_order_relaxed);
    g_aim_fz.store(fz, std::memory_order_relaxed);
    if (!g_stats_armed.exchange(true)) {
        API::get()->log_info("[Halo-CampE-UEVR] BLAMHOOK: correlating per handle - MOVE THE AIM so the "
                             "player's object can be told apart from static scenery");
        return;
    }

    // Report the handles whose orientation best tracks the aim. A handle that is genuinely the
    // shooter should sit at |dot| ~ 1.0 across a wide aim sweep; scenery will not.
    uint32_t top[5]{}; float topd[5]{};
    int n = 0;
    for (uint32_t h = 0; h < HANDLE_SLOTS; ++h) {
        if (g_hits[h].load(std::memory_order_relaxed) == 0) continue;
        const float d = g_best_dot[h].load(std::memory_order_relaxed);
        int slot = -1;
        for (int k = 0; k < n; ++k) if (d > topd[k]) { slot = k; break; }
        if (slot < 0 && n < 5) slot = n++;
        else if (slot < 0) continue;
        for (int k = (n < 5 ? n - 1 : 4); k > slot; --k) { top[k] = top[k-1]; topd[k] = topd[k-1]; }
        if (n < 5 && slot + 1 > n) n = slot + 1;
        top[slot] = h; topd[slot] = d;
    }

    uint32_t distinct = 0;
    for (uint32_t h = 0; h < HANDLE_SLOTS; ++h) if (g_hits[h].load(std::memory_order_relaxed)) ++distinct;

    API::get()->log_info("[Halo-CampE-UEVR] BLAMHOOK: calls=%llu chain=%s | aimfwd=(%.4f,%.4f,%.4f) | "
                         "%u distinct handles", (unsigned long long)calls,
                         g_chain_ok.load() ? "OK" : "FAILED", fx, fy, fz, distinct);
    for (int k = 0; k < n; ++k) {
        API::get()->log_info("[Halo-CampE-UEVR]   handle 0x%04X bestdot=%.4f hits=%u A=(%.4f,%.4f,%.4f)",
                             top[k], topd[k], g_hits[top[k]].load(),
                             g_hx[top[k]].load(), g_hy[top[k]].load(), g_hz[top[k]].load());
    }
}

} // namespace halo

#endif // HALO_VR_DEV





