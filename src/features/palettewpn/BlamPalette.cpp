#include "features/palettewpn/BlamPalette.hpp"
#include "core/config/CfgRead.hpp"

#include "BlamDrive.hpp"     // resolve_object_by_datum: the weapon object through the sim's table
#include "core/UnitState.hpp"
#include "core/reload/ReloadEngine.hpp"   // the reload engine's pose hold request
#include "core/WeaponObject.hpp"   // the weapon object service and the slide node it publishes
#include "Config.hpp"
#include "features/palettewpn/PaletteTwoHand.hpp"
#include "ArmDriver.hpp"            // palette_weapon_mode(): this file acts only while armdriver mode 3 owns
#include "features/palettewpn/PaletteArmDriver.hpp"   // palette_weapon_mode()
#include "palettearm/PaletteHook.hpp" // palettehook_installed(): never hook the builder while the palettearm route holds it       // two_hand_delta: the hold rotates the WEAPON pose, not only the shot
#include "Math.hpp"
#include "MotionAimControl.hpp"
#include "features/palettewpn/PoseLatch.hpp"   // the latched intents, the frame audit
#include "Markers.hpp"        // g_cam_x/y/z: the rendered camera, for the WPNERR speed column
#include "Rig.hpp"   // g_turnq_* : the player's accumulated snap/smooth turn
#include "WeaponCalib.hpp"   // weapon_key
#include "features/palettewpn/PaletteReadbacks.hpp"   // rig_socket_world
#include "features/palettewpn/PaletteCalib.hpp"   // pal_apply_aim_fix, pal_wpnfix_find
#include "uevr/API.hpp"

#include <Windows.h>
#include <psapi.h>
#include <tlhelp32.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <cstdio>
#include <vector>
#include <cstring>
#include <intrin.h>

using namespace uevr;

namespace halo {

// The mesh-vs-camera constants, measured per tick by Plugin.cpp (see their declaration there):
// comp_rot = cam * M, comp_pos = parent_pos + cam-rotated v0 (cm). With these, the hook can
// reconstruct the FP mesh's world transform FRESH from the one volatile term -- the aim rotator,
// which the sim thread can read at build time. Declared here rather than in a header: one consumer.
extern std::atomic<float> g_meshM_x, g_meshM_y, g_meshM_z, g_meshM_w;
extern std::atomic<float> g_meshV0_x, g_meshV0_y, g_meshV0_z;
extern std::atomic<float> g_tick_cam_p, g_tick_cam_y;
extern std::atomic<long long> g_tick_cam_ms;
extern std::atomic<float> g_tick_mrot_x, g_tick_mrot_y, g_tick_mrot_z, g_tick_mrot_w;
extern std::atomic<unsigned> g_tick_seq;        // Plugin.cpp: seqlock over cam_tick + M + Q_tick
// MODE 12 (2026-09-12). The mesh's world rotation read LIVE at the render callback and published
// here, seqlocked, so the build writer and the refresh writer divide by the IDENTICAL value.
// Measured: our written pose correlates only 0.5194 with the hand at its best lag (and that lag
// is exactly 0, so nothing is lead or lag), meaning about half of what we write is not the hand.
// The game's aim carries 3-10x the hand's high-frequency content above 3 Hz, that noise enters
// through the camera counter-term, and it only cancels if the divisor IS the mesh. Q_render and
// Q_tick agree on just 16.7% of frames, so no reconstruction has ever been the mesh. Every mode
// so far divided by a reconstruction: mode 8 round-trips through pitch/yaw and drops roll, mode 9
// read live when refreshing but used the tick value when building so its two writers disagreed,
// mode 11 used the tick value in both, self-consistent but not what the renderer draws with.
std::atomic<uint32_t> g_qr_seq{0};
std::atomic<float> g_qr_x{0.0f}, g_qr_y{0.0f}, g_qr_z{0.0f}, g_qr_w{1.0f};
std::atomic<long long> g_qr_ms{0};
extern std::atomic<float> g_meshM_brk_deg;      // Plugin.cpp: camera motion between the M brackets
extern std::atomic<long long> g_meshM_acc_ms;   // Plugin.cpp: when the live M was accepted
void blam_palette_final_hook_tick();   // defined below the builder hook machinery
void blam_palette_sniff_tick();        // ditto
extern std::atomic<unsigned> g_tick_id;
void stomp_mark(int point, float yaw, float e0, float e1, float e2);   // Plugin.cpp, the STOMPLOG ring
extern std::atomic<bool>  g_mesh_const_valid;
// The yaw the stereo callback actually OUTPUT this frame -- the rendered VIEW. Published by
// Plugin.cpp on every path through that callback.
extern std::atomic<float> g_view_base_yaw;
extern std::atomic<float> g_bob_x, g_bob_y, g_bob_z;   // camera bob, world cm (Plugin.cpp)
// Node 8 as last written, published for the game thread's TRACE-NODE8 diagnostic.
extern std::atomic<float> g_dbg_node8_x, g_dbg_node8_y, g_dbg_node8_z;
extern std::atomic<float> g_dbg_pose_w_x, g_dbg_pose_w_y, g_dbg_pose_w_z, g_dbg_pose_w_w;
extern std::atomic<float> g_barrel_axis_x, g_barrel_axis_y, g_barrel_axis_z;
extern std::atomic<bool>  g_barrel_axis_valid;
// For the hooked_pose stamp: the camera resolve_world_pullback last embedded, and how many
// capture banks the last apply found (-1 = the apply never got that far or was gated off).
std::atomic<float> g_dbg_build_cam_p{0.0f}, g_dbg_build_cam_y{0.0f};
std::atomic<int>   g_dbg_build_nbanks{-1};
// The camera the render refresh last embedded -- recovered from the mesh transform, ground
// truth for the frame on screen. palettecam=7 feeds the sim build from HERE so both bank
// writers carry one camera.
std::atomic<float> g_rcam_p{0.0f}, g_rcam_y{0.0f};
std::atomic<long long> g_rcam_ms{0};

namespace {

// The node transform, as Blam stores it. 0x34 bytes: a scale, then three basis vectors and a
// position. Derived from the shape the scan validates against, not from a header we do not have.
struct NodeMatrix {
    float x, y, z;
};
struct PaletteNode {
    float      scale;      // +0x00
    NodeMatrix forward;    // +0x04
    NodeMatrix left;       // +0x10
    NodeMatrix up;         // +0x1C
    NodeMatrix position;   // +0x28
};
// The engine's bank-blend weight and the two bank indices it names, read off the capture ctx.
std::atomic<float> g_blend_t{-999.0f};
std::atomic<int>   g_blend_a{-1}, g_blend_b{-1};
// The live slot palette (slot + OFF_FINAL_PALETTE). sim+0x46A2E0 draws from THIS instead of the
// blended banks whenever the bank blend fails one of its seven checks (0x46A52E:
// lea r14,[rsi+0x1094]). BANKDIFF proved the two banks are identical, so if the drawn pose ever
// departs from them, this buffer is one of the two remaining places it can come from.
std::atomic<uintptr_t> g_live_pal_addr{0};
// PALETTESYNC pair (written by blam_palette_sync_capture from the aim callback).
struct SyncPair {
    std::atomic<uint32_t> seq{0};
    std::atomic<long long> ms{0};
    std::atomic<float> aqx{0}, aqy{0}, aqz{0}, aqw{1}, apx{0}, apy{0}, apz{0};
    std::atomic<float> gqx{0}, gqy{0}, gqz{0}, gqw{1}, gpx{0}, gpy{0}, gpz{0};
    std::atomic<float> cp{0}, cy{0};
};
SyncPair g_sync;
// The aim that belongs to the hand the publisher last published (copied from the pair at publish).
std::atomic<float> g_p_sync_cp{0.0f}, g_p_sync_cy{0.0f};
std::atomic<bool>  g_p_sync_ok{false};
// MODE 13's input (published by obscam_capture on the sim thread): the eased observer camera's
// DYNAMIC gap from the aim. The slow mean is removed so a constant convention offset between the
// two descriptions cannot misaim; only the smoothing lag is carried.
std::atomic<float> g_obs_gap_y{0.0f}, g_obs_gap_p{0.0f};
std::atomic<long long> g_obs_ms{0};
// STILL 52. The 2026-08-17 update did NOT change this struct, and a detour through believing it had
// is worth recording so it is not repeated.
//
// Loose stride probes over the heap reported 60 (runs of 3-5) and once 60-with-run-76, so the struct
// was widened to 60 on that evidence. It was wrong: those probes had landed on OTHER skeletons in
// memory, and widening the struct then broke every read of the real palette -- the capture bank went
// from readable to "run=1", because with a 60-byte stride node 1 is never where it should be.
//
// The authoritative measurement is against the FP capture bank, whose node count the game itself
// publishes, searching START and STRIDE together instead of assuming either:
//   CAPTUREGRID: bank 0 best start=+0x0 stride=52 run=76 (count says 76)
// Both banks, start +0x0 (i.e. exactly OFF_CAPTURE_PALETTE), stride 52, run == the published count.
// A shape probe with no ground truth can find any orthonormal array; only the count field says which
// one is the first-person rig. Measure against something that knows the answer.
static_assert(sizeof(PaletteNode) == 0x34, "palette node must be 52 bytes");

// Sim-thread hook timing, for the game-thread PERF report (microseconds).
std::atomic<uint64_t> g_hook_us_sum{0};
std::atomic<uint32_t> g_hook_us_max{0};
std::atomic<uint32_t> g_hook_n{0};

std::atomic<uintptr_t> g_palette{0};
std::atomic<int32_t>   g_nodes{0};

// Every candidate the scan found, in the order it found them, so `palettepoke` can name one by the
// index that was logged. Kept small deliberately: if a session ever produces more than this many
// plausible first-person palettes, the shape test has stopped discriminating and that is the bug to
// fix, not the array size.
constexpr int kMaxCandidates = 12;
struct Candidate { uintptr_t at; int32_t nodes; };
Candidate            g_cand[kMaxCandidates]{};
std::atomic<int>     g_cand_count{0};

// _tls_index for the sim module, read from the same place BlamDrive reads it. Duplicated rather
// than shared because BlamDrive keeps it file-static, and reaching into it would couple the two
// files for one integer; if a third consumer ever appears, promote it then.
constexpr uintptr_t RVA_TLS_INDEX = 0xD72730;

bool readable(const void* p, size_t n) { return p != nullptr && !IsBadReadPtr(p, n); }

inline float len2(const NodeMatrix& v) { return v.x * v.x + v.y * v.y + v.z * v.z; }
inline float dot(const NodeMatrix& a, const NodeMatrix& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Does this look like a posed node rather than arbitrary bytes?
//
// The three basis vectors carry the discrimination. Random floats are overwhelmingly unlikely to be
// simultaneously unit length AND mutually perpendicular, so requiring an orthonormal basis is what
// makes a run of these a signature rather than a coincidence. Scale and position are only sanity
// bounds -- they reject obvious garbage cheaply, before the expensive test runs.
bool looks_like_node(const PaletteNode& n) {
    if (!std::isfinite(n.scale) || std::fabs(n.scale) > 16.0f || std::fabs(n.scale) < 1.0e-3f) {
        return false;
    }
    for (const NodeMatrix* v : {&n.forward, &n.left, &n.up, &n.position}) {
        if (!std::isfinite(v->x) || !std::isfinite(v->y) || !std::isfinite(v->z)) return false;
    }
    // First-person nodes live within arm's reach of the view root, in Blam world units.
    if (len2(n.position) > 10000.0f) return false;

    // Unit length, generously -- a scaled node is still a valid node.
    for (const NodeMatrix* v : {&n.forward, &n.left, &n.up}) {
        const float l2 = len2(*v);
        if (l2 < 0.64f || l2 > 1.56f) return false;   // ~0.8 .. ~1.25
    }
    // Mutually perpendicular. This is the test that actually finds the palette.
    constexpr float kPerp = 0.10f;
    if (std::fabs(dot(n.forward, n.left)) > kPerp) return false;
    if (std::fabs(dot(n.forward, n.up))   > kPerp) return false;
    if (std::fabs(dot(n.left,    n.up))   > kPerp) return false;
    return true;
}

// How many consecutive valid nodes start here. Capped: a run longer than any plausible skeleton
// means the test is matching something it should not, and reporting that is more useful than
// walking off into unmapped memory to find out.
// How many consecutive valid nodes start here IF the array stride is `stride` bytes.
//
// run_length() below hard-codes sizeof(PaletteNode) == 52, which was the stride on the pre-2026-08-17
// build. After the update the scan finds isolated node-shaped frames (7, 15 of them) with
// longest_run == 1 every time: each node is real, the next one is simply not 52 bytes along. That is
// a stride change, not a shape change, so the stride has to be measured rather than assumed.
int32_t run_length_stride(uintptr_t at, int32_t cap, uintptr_t stride) {
    int32_t n = 0;
    for (; n < cap; ++n) {
        const uintptr_t p = at + (uintptr_t)n * stride;
        if (!readable((const void*)p, sizeof(PaletteNode))) break;
        if (!looks_like_node(*reinterpret_cast<const PaletteNode*>(p))) break;
    }
    return n;
}

// Try every plausible array stride at a confirmed node and report which one produces a skeleton.
// 52 is the old value; a struct that gained a field, or gained padding to 16-byte alignment, lands
// on 56/64/80/96. Logged rather than adopted -- naming the number is the job here.
void probe_stride(uintptr_t at) {
    uintptr_t best_stride = 0; int32_t best_run = 0;
    for (uintptr_t s = 52; s <= 256; s += 4) {
        const int32_t r = run_length_stride(at, 512, s);
        if (r > best_run) { best_run = r; best_stride = s; }
    }
    API::get()->log_info(
        "[Halo-CampE-UEVR] STRIDE PROBE at 0x%llX: best stride=%llu bytes gives run=%d nodes "
        "(old build used 52). A run near 76 names the new PaletteNode size.",
        (unsigned long long)at, (unsigned long long)best_stride, best_run);
}

int32_t run_length(uintptr_t at, int32_t cap) {
    int32_t n = 0;
    while (n < cap) {
        const uintptr_t p = at + (uintptr_t)n * sizeof(PaletteNode);
        if (!readable((const void*)p, sizeof(PaletteNode))) break;
        if (!looks_like_node(*reinterpret_cast<const PaletteNode*>(p))) break;
        ++n;
    }
    return n;
}

// A run is CORROBORATED when the node count is written just before it.
//
// The palette does not float in memory on its own -- the structure that owns it records how many
// nodes it has, and that count sits immediately ahead of the array. Finding the run length spelled
// out in the preceding dwords turns "these bytes look like matrices" into "this is the array, and
// the game agrees about its size". Reported rather than required: a run that is real but not
// preceded by its count is still worth seeing.
int corroborating_counts(uintptr_t palette, int32_t run) {
    int hits = 0;
    for (int back = 1; back <= 4; ++back) {
        const uintptr_t p = palette - (uintptr_t)back * 4;
        if (!readable((const void*)p, 4)) continue;
        if (*reinterpret_cast<const int32_t*>(p) == run) ++hits;
    }
    return hits;
}

} // namespace

uintptr_t blam_palette_address()  { return g_palette.load(std::memory_order_relaxed); }
int32_t   blam_palette_node_count(){ return g_nodes.load(std::memory_order_relaxed); }
void blam_palette_hook_perf(double* out_mean_us, double* out_max_us, uint32_t* out_n) {
    const uint32_t n = g_hook_n.exchange(0, std::memory_order_relaxed);
    const uint64_t sum = g_hook_us_sum.exchange(0, std::memory_order_relaxed);
    const uint32_t mx = g_hook_us_max.exchange(0, std::memory_order_relaxed);
    *out_n = n; *out_max_us = (double)mx; *out_mean_us = n ? (double)sum / (double)n : 0.0;
}

// ---- THE SIM THREAD'S TLS ARRAY, READ FROM ANY THREAD.
//
// The scan below used to be callable only from inside a Blam hook, for one reason: it read
// __readgsqword(0x58), which is the CURRENT thread's TLS array. That single line is what forced the
// scan and the write-watch to ride hooked_get_orientation() -- and when the 2026-08-17 game update
// moved that function, the tools needed to FIND the new address could themselves no longer run.
// A discovery tool that depends on the thing it discovers is not a tool.
//
// A thread's TLS array lives at TEB+0x58, and any thread's TEB base is obtainable through
// NtQueryInformationThread(ThreadBasicInformation). So: walk the process's threads, read each TEB,
// and take the one whose slot [_tls_index] is non-null -- that is the sim thread, by definition,
// because that slot is where the sim module put its per-thread state. Entirely read-only: no hook,
// no patch, no suspend. Falls back to the gs read when we happen to already be on the right thread.
// EVERY thread holding a sim TLS block, not the first one found.
//
// The original code was on the sim thread BY CONSTRUCTION -- it ran inside a Blam hook -- so "the
// current thread's block" was necessarily the right one. Resolving cross-thread loses that: any
// thread that has ever touched the sim module owns a non-null slot, and only ONE of them is the
// thread that builds the first-person palette. Taking the first hit is a guess. So collect them all
// and let the caller try each; `which` selects, and the return value reports how many exist.
int sim_tls_arrays(uintptr_t* out, int max_out) {
    int n = 0;
    const HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
    if (sim == nullptr || out == nullptr || max_out <= 0) return 0;
    const uint32_t tls_index = *(const uint32_t*)((uintptr_t)sim + RVA_TLS_INDEX);

    // Current thread first, when it qualifies -- cheapest, and correct whenever we happen to be on it.
    const uintptr_t here = (uintptr_t)__readgsqword(0x58);
    if (here != 0 && readable((const void*)(here + (uintptr_t)tls_index * 8), 8) &&
        *reinterpret_cast<const uintptr_t*>(here + (uintptr_t)tls_index * 8) != 0) {
        out[n++] = here;
    }

    typedef LONG (NTAPI *PFN_NTQIT)(HANDLE, ULONG, PVOID, ULONG, PULONG);
    static PFN_NTQIT ntqit = (PFN_NTQIT)GetProcAddress(GetModuleHandleA("ntdll.dll"),
                                                       "NtQueryInformationThread");
    if (ntqit == nullptr) return 0;

    struct TBI { LONG ExitStatus; PVOID TebBaseAddress; PVOID UniqueProcess; PVOID UniqueThread;
                 ULONG_PTR AffinityMask; LONG Priority; LONG BasePriority; };

    const DWORD pid = GetCurrentProcessId();
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    if (snap == INVALID_HANDLE_VALUE) return n;
    THREADENTRY32 te{}; te.dwSize = sizeof(te);
    if (Thread32First(snap, &te)) {
        do {
            if (te.th32OwnerProcessID != pid) continue;
            HANDLE th = OpenThread(THREAD_QUERY_INFORMATION, FALSE, te.th32ThreadID);
            if (th == nullptr) continue;
            TBI tbi{}; ULONG got = 0;
            if (ntqit(th, 0 /*ThreadBasicInformation*/, &tbi, sizeof(tbi), &got) >= 0 &&
                tbi.TebBaseAddress != nullptr) {
                const uintptr_t tls = *(const uintptr_t*)((uintptr_t)tbi.TebBaseAddress + 0x58);
                if (tls != 0 && readable((const void*)(tls + (uintptr_t)tls_index * 8), 8)) {
                    const uintptr_t blk = *reinterpret_cast<const uintptr_t*>(tls + (uintptr_t)tls_index * 8);
                    bool dup = false;
                    for (int i = 0; i < n; ++i) if (out[i] == tls) { dup = true; break; }
                    if (blk != 0 && !dup && n < max_out) out[n++] = tls;
                }
            }
            CloseHandle(th);
        } while (n < max_out && Thread32Next(snap, &te));
    }
    CloseHandle(snap);
    return n;
}

// ---- FIND THE PALETTE BY SHAPE, ANYWHERE IN THE HEAP. No TLS block, no indirection assumption.
//
// The TLS sweep follows pointers out of the sim's thread block and searches 0x60000 bytes behind
// each. That encodes an assumption -- "the palette hangs one level off the block" -- which held on
// the old build and does not on the 2026-08-17 one: 16 candidate threads, 7-8 followable bases
// each, and ZERO node-shaped hits anywhere. looks_like_node() is a pure shape test (orthonormal
// basis, sane scale, position within reach) and the sweep steps 4 bytes, so a layout change would
// still have produced hits somewhere. Zero means we were looking in the wrong memory.
//
// A bone palette is a distinctive object: 24+ consecutive 52-byte orthonormal frames. That shape is
// findable without knowing how the game reaches it -- which is the property BlamPalette.hpp claims
// for this whole approach ("a shape survives a patch"). The TLS route was the one place that quietly
// depended on structure instead.
//
// CHUNKED. The heap is large and this runs twice a second, so each call walks a bounded number of
// bytes from a persistent cursor and returns; the cursor wraps when it runs out of address space.
// Cost per call stays flat, coverage completes over a few seconds.
bool heap_scan_step(uintptr_t* out_at, int32_t* out_run, int32_t min_run, int32_t max_run) {
    static uintptr_t cursor = 0;
    constexpr size_t kBytesPerCall = 24u * 1024u * 1024u;   // bounded work per tick
    size_t budget = kBytesPerCall;

    SYSTEM_INFO si{}; GetSystemInfo(&si);
    const uintptr_t lo = (uintptr_t)si.lpMinimumApplicationAddress;
    const uintptr_t hi = (uintptr_t)si.lpMaximumApplicationAddress;
    if (cursor < lo) cursor = lo;

    while (cursor < hi && budget > 0) {
        MEMORY_BASIC_INFORMATION mbi{};
        if (VirtualQuery((LPCVOID)cursor, &mbi, sizeof(mbi)) != sizeof(mbi)) { cursor += 0x1000; continue; }
        const uintptr_t base = (uintptr_t)mbi.BaseAddress;
        const size_t    size = mbi.RegionSize;
        const DWORD     prot = mbi.Protect;
        const bool usable = (mbi.State == MEM_COMMIT) && (mbi.Type == MEM_PRIVATE) &&
                            ((prot & (PAGE_READWRITE | PAGE_EXECUTE_READWRITE)) != 0) &&
                            ((prot & (PAGE_GUARD | PAGE_NOACCESS)) == 0);
        if (!usable || size == 0) { cursor = base + (size ? size : 0x1000); continue; }

        const size_t take = (size < budget) ? size : budget;
        for (uintptr_t off = 0; off + sizeof(PaletteNode) < take; off += 4) {
            const uintptr_t at = base + off;
            if (!readable((const void*)at, sizeof(PaletteNode))) { off += 0xFFC; continue; }
            if (!looks_like_node(*reinterpret_cast<const PaletteNode*>(at))) continue;
            const int32_t run = run_length(at, max_run);
            if (run >= min_run) { *out_at = at; *out_run = run; cursor = at + (uintptr_t)run * sizeof(PaletteNode); return true; }
            off += (uintptr_t)(run > 0 ? run : 1) * sizeof(PaletteNode);
        }
        budget = (budget > take) ? (budget - take) : 0;
        cursor = base + size;
    }
    if (cursor >= hi) cursor = lo;    // wrap and keep looking
    return false;
}

void blam_palette_scan() {
    // Re-arm on a 0 -> non-zero transition, so the scan can be repeated with a different weapon in
    // hand. A weapon's nodes are only in the palette while it is held, so one scan per session
    // cannot answer "which nodes belong to which weapon".
    // RETRY UNTIL IT FINDS SOMETHING, then stop. This used to latch `done` on the FIRST attempt,
    // which was safe while the scan rode a Blam hook -- that only ran once the sim was live and a
    // weapon was in hand. Driven from the game tick it now fires seconds after launch, in the
    // frontend, where there is no player and no weapon to find, and then never looked again:
    // "PALETTE: nothing matched" at 8 s uptime, 2026-08-17. A weapon's nodes only exist in the
    // palette while it is held, so the scan has to keep asking until the answer can be yes.
    // NO EXTRA RATE LIMIT. The caller (blam_palette_hook_tick) already lives inside the tick's
    // config-reload block, so it runs once every 64 ticks -- about twice a second -- and the sweep
    // measured 1 ms (14:33:25.924 -> .925). A second gate of 120 on top of that meant roughly ONE
    // attempt per minute, which is why 50 seconds of gameplay produced a single "nothing matched".
    static bool s_done = false;
    // Best candidate seen across sweeps, and how many sweeps have run. The FP rig only exists while
    // a weapon is held and posed, so a lower-scoring skeleton found early must not be adopted
    // permanently -- keep looking, and settle for the best only after the heap has been covered.
    static uintptr_t s_best_at = 0; static int32_t s_best_run = 0; static int s_best_score = -1;
    static uint32_t s_sweeps = 0;
    if (g_cfg.palette_scan == 0) {
        s_done = false; s_best_at = 0; s_best_run = 0; s_best_score = -1; s_sweeps = 0; return;
    }
    if (s_done) return;
    g_cand_count.store(0, std::memory_order_release);

    // Rotate through every thread that owns a sim TLS block, one per attempt. The scan retries about
    // twice a second, so all candidates get tried within a few seconds -- and because only one of
    // them is the thread that builds the first-person palette, trying just the first is a guess.
    uintptr_t cands[16] = {};
    const int ncand = sim_tls_arrays(cands, 16);
    if (ncand == 0) {
        API::get()->log_info("[Halo-CampE-UEVR] PALETTE: no thread holds the sim TLS block "
                             "(module not loaded yet, or _tls_index RVA is wrong)");
        return;
    }
    static uint32_t s_which = 0;
    const uintptr_t tls_array = cands[s_which++ % (uint32_t)ncand];
    const HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
    if (sim == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] PALETTE: sim module not loaded");
        return;
    }
    const uint32_t tls_index = *(const uint32_t*)((uintptr_t)sim + RVA_TLS_INDEX);

    uintptr_t block = 0;
    if (!readable((const void*)(tls_array + (uintptr_t)tls_index * 8), 8)) {
        API::get()->log_info("[Halo-CampE-UEVR] PALETTE: TLS slot unreadable");
        return;
    }
    block = *reinterpret_cast<const uintptr_t*>(tls_array + (uintptr_t)tls_index * 8);
    if (block == 0) {
        API::get()->log_info("[Halo-CampE-UEVR] PALETTE: TLS block null");
        return;
    }

    API::get()->log_info("[Halo-CampE-UEVR] PALETTE: scanning from TLS block 0x%llX",
                         (unsigned long long)block);

    // WHERE TO LOOK. Every pointer the TLS block holds, then a bounded sweep of whatever each one
    // points at. The palette is not in the block itself -- it hangs off a per-player structure that
    // the block references -- so following one level of indirection is the minimum that can find
    // it, and one level is enough because the block is the sim's own root for per-thread state.
    constexpr uintptr_t kBlockSpan  = 0x800;      // pointer slots to try in the TLS block
    constexpr uintptr_t kRegionSpan = 0x60000;    // bytes to sweep behind each pointer
    constexpr int32_t   kMinRun     = 24;         // shorter runs are noise, not a skeleton
    constexpr int32_t   kMaxRun     = 512;

    int reported = 0;
    int32_t best_run = 0;
    uintptr_t best_at = 0;
    int best_corr = -1;

    // NEAR-MISS COUNTERS. "nothing matched" is not a diagnosis -- it cannot tell a wrong TLS block
    // from an unchanged one whose node LAYOUT the 2026-08-17 update altered. These separate them:
    //   bases == 0            -> the block holds no followable pointers: wrong thread
    //   looks == 0            -> memory is there but nothing has a node's shape: looks_like_node()
    //                            needs re-deriving against the new struct
    //   looks > 0, runs short -> the shape is right and kMinRun is wrong, or the stride changed
    int n_bases = 0, n_looks = 0; int32_t longest = 0;

    for (uintptr_t slot = 0; slot < kBlockSpan; slot += 8) {
        if (!readable((const void*)(block + slot), 8)) continue;
        const uintptr_t base = *reinterpret_cast<const uintptr_t*>(block + slot);
        if (base < 0x10000 || (base & 0x3) != 0) continue;      // null, small, or unaligned
        if (!readable((const void*)base, 0x100)) continue;
        ++n_bases;

        for (uintptr_t off = 0; off + sizeof(PaletteNode) < kRegionSpan; off += 4) {
            const uintptr_t at = base + off;
            if (!readable((const void*)at, sizeof(PaletteNode))) { off += 0xFFC; continue; }
            if (!looks_like_node(*reinterpret_cast<const PaletteNode*>(at))) continue;
            ++n_looks;

            const int32_t run = run_length(at, kMaxRun);
            if (run > longest) longest = run;
            // A confirmed node whose neighbours are not 52 bytes away: measure the real stride
            // instead of discarding it. Bounded to a handful per scan -- the answer is one number
            // and it does not need 400 votes.
            // STRIDE SELF-CHECK. sizeof(PaletteNode) is now 60, measured. If a future update moves
            // it again the symptom is identical to 2026-08-17 -- nodes match, runs come back 1 --
            // so probe at a confirmed node and say the number out loud rather than failing silently.
            if (run < kMinRun) {
                static uint32_t s_probed = 0;
                if (s_probed < 4u) { ++s_probed; probe_stride(at); }
            }
            if (run < kMinRun) continue;

            const int corr = corroborating_counts(at, run);
            if (reported < kMaxCandidates) {
                g_cand[reported] = Candidate{at, run};
                ++reported;
                g_cand_count.store(reported, std::memory_order_release);
                API::get()->log_info(
                    "[Halo-CampE-UEVR] PALETTE hit #%d: tls+0x%03llX -> base 0x%llX +0x%05llX "
                    "run=%d nodes  countBefore=%d  (palettepoke=%d)",
                    reported, (unsigned long long)slot, (unsigned long long)base,
                    (unsigned long long)off, run, corr, reported);
            }
            // Prefer a run whose length is written just before it; fall back to the longest.
            if (corr > best_corr || (corr == best_corr && run > best_run)) {
                best_corr = corr; best_run = run; best_at = at;
            }
            // Skip past this run rather than re-finding it at every node boundary.
            off += (uintptr_t)run * sizeof(PaletteNode);
        }
    }

    if (best_at != 0) {
        s_done = true;                      // only NOW is the question answered; see the gate above
        g_palette.store(best_at, std::memory_order_relaxed);
        g_nodes.store(best_run, std::memory_order_relaxed);
        API::get()->log_info(
            "[Halo-CampE-UEVR] PALETTE: best candidate 0x%llX, %d nodes, countBefore=%d",
            (unsigned long long)best_at, best_run, best_corr);
        // The first node of a first-person palette is the view root. Printing a few lets the run be
        // sanity-checked by eye against how the arms are actually posed.
        for (int i = 0; i < 3 && i < best_run; ++i) {
            const auto& n = *reinterpret_cast<const PaletteNode*>(
                best_at + (uintptr_t)i * sizeof(PaletteNode));
            API::get()->log_info(
                "[Halo-CampE-UEVR] PALETTE   node %02d scale=%.3f pos=(%.3f %.3f %.3f) "
                "fwd=(%.2f %.2f %.2f)",
                i, n.scale, n.position.x, n.position.y, n.position.z,
                n.forward.x, n.forward.y, n.forward.z);
        }
    } else {
        // ---- FALLBACK: the shape scan. COLLECT EVERY SKELETON, THEN PICK THE FIRST-PERSON ONE.
        //
        // The first version latched on the first run >= 24 and stopped. On 2026-08-17 that was a
        // 78-node skeleton with node 0 at world (-30.4, 23.8, 0) facing -Y -- a biped standing out
        // in the level, not the FP rig, whose writer would be a generic node-matrix builder rather
        // than FP_BUILD. The scan then never looked further.
        //
        // The FP palette is identifiable: it is CAMERA-LOCAL, so node 0 sits at the origin with an
        // identity basis (PALETTEROOT logged fwd=(1,0,0) up=(0,0,1) pos~0 on the old build) and the
        // count is ~76 with a weapon held. Score every candidate on exactly that and take the best.
        {
            uintptr_t hat = 0; int32_t hrun = 0;
            // BOUNDED. Each heap_scan_step() call takes a fresh 24 MB budget, so an unbounded loop
            // scanned the address space inside one tick -- measured WORST FRAME=649.8ms and a
            // TICK max of 1076ms on 2026-08-17. Eight candidates per tick is plenty when the scan
            // runs twice a second, and it keeps the frame cost flat.
            for (int guard = 0; guard < 8 && heap_scan_step(&hat, &hrun, kMinRun, kMaxRun); ++guard) {
                const PaletteNode& n0 = *reinterpret_cast<const PaletteNode*>(hat);
                const float dist2 = len2(n0.position);
                const float fx = n0.forward.x, fy = n0.forward.y, fz = n0.forward.z;
                // identity-basis-at-origin score: forward ~ (1,0,0) and position ~ 0
                const bool at_origin = dist2 < 4.0f;
                const bool ident_fwd = (fx > 0.9f) && (std::fabs(fy) < 0.2f) && (std::fabs(fz) < 0.2f);
                const int score = (at_origin ? 2 : 0) + (ident_fwd ? 2 : 0)
                                + ((hrun >= 60 && hrun <= 90) ? 1 : 0);
                API::get()->log_info(
                    "[Halo-CampE-UEVR] PALETTE cand at 0x%llX run=%-3d score=%d | node00 scale=%.3f "
                    "pos=(%.2f %.2f %.2f) fwd=(%.2f %.2f %.2f)%s",
                    (unsigned long long)hat, hrun, score, n0.scale,
                    n0.position.x, n0.position.y, n0.position.z, fx, fy, fz,
                    (score >= 4) ? "   <== looks like the FP rig" : "");
                if (score > s_best_score) {
                    s_best_score = score; s_best_at = hat; s_best_run = hrun;
                }
                if (score >= 4) break;      // camera-local root: stop, this is the one
            }
            if (s_best_at != 0 && (s_best_score >= 4 || s_sweeps++ > 40u)) {
                s_done = true;
                g_palette.store(s_best_at, std::memory_order_relaxed);
                g_nodes.store(s_best_run, std::memory_order_relaxed);
                g_cand[0] = Candidate{s_best_at, s_best_run};
                g_cand_count.store(1, std::memory_order_release);
                API::get()->log_info(
                    "[Halo-CampE-UEVR] PALETTE: SELECTED 0x%llX run=%d score=%d -- palettewatch will "
                    "now name its writer", (unsigned long long)s_best_at, s_best_run, s_best_score);
                return;
            }
        }
        // Rate-limited: this now runs ~2x/second, and an unchanging failure line 400 times is noise.
        static uint32_t s_miss = 0;
        if ((s_miss++ % 20u) == 0u) {
            API::get()->log_info(
                "[Halo-CampE-UEVR] PALETTE: nothing matched | tls=0x%llX (cand %u of %d) "
                "followable_bases=%d node_shaped_hits=%d longest_run=%d (need >=%d) "
                "[bases=0 -> wrong thread; hits=0 -> node layout changed; hits>0 -> run/stride changed]",
                (unsigned long long)block, (unsigned)((s_which - 1) % (uint32_t)ncand), ncand,
                n_bases, n_looks, longest, kMinRun);
        }
    }
}

void blam_palette_poke() {
    const int which = g_cfg.palette_poke;
    if (which <= 0) return;
    const int count = g_cand_count.load(std::memory_order_acquire);
    if (which > count) return;

    const Candidate c = g_cand[which - 1];
    if (c.at == 0 || c.nodes <= 0) return;

    int32_t node = g_cfg.palette_poke_node;
    if (node < 0 || node >= c.nodes) node = 0;

    const uintptr_t p = c.at + (uintptr_t)node * sizeof(PaletteNode);
    if (IsBadWritePtr((void*)p, sizeof(PaletteNode))) return;
    auto* n = reinterpret_cast<PaletteNode*>(p);

    // Displace, do not assign. Adding to whatever the game just wrote keeps the node's authored
    // orientation and its relationship to its parent intact, so a rig that moves is a rig that
    // MOVED rather than one that collapsed -- which is the difference between "the write landed"
    // and "the write destroyed the pose", and those look nothing alike on screen.
    //
    // Every call, at the getter's ~2600/sec. The game rebuilds this palette each frame, so a write
    // only shows if it lands between the rebuild and whatever consumes it. Writing constantly is
    // what makes a NEGATIVE result mean something: if nothing moves after thousands of writes per
    // second, the candidate is not the rendered rig, rather than the write merely having been
    // unlucky with timing.
    // ---- READ BACK BEFORE WRITING AGAIN.
    //
    // A poke that produces nothing on screen has two completely different causes, and they need
    // opposite fixes:
    //
    //   the value we wrote is GONE      -> the game rebuilds this palette between our writes, so we
    //                                      are writing at the wrong point in the frame. Fix: hook
    //                                      where the rig is BUILT, not the orientation getter.
    //   the value we wrote is STILL THERE -> our write persists and simply is not rendered, so this
    //                                      array is not the rig on screen. Fix: keep hunting.
    //
    // Reading the value at entry, before adding to it again, separates them. If the game owns the
    // array the reading tracks the game's own animation and never carries our delta. If we own it,
    // nothing resets it and the value runs away by `amt` on every one of ~2600 calls a second --
    // unmistakable within a frame or two.
    static uintptr_t s_watch = 0;
    static float     s_expect = 0.0f;
    static bool      s_have = false;
    static uint32_t  s_n = 0;

    const float before = n->position.y;
    if (s_watch != p) { s_watch = p; s_have = false; s_n = 0; }   // target changed: start over

    if (s_have && (s_n % 600u) == 0u) {
        const float drift = before - s_expect;
        API::get()->log_info(
            "[Halo-CampE-UEVR] PALETTE readback: expected %.3f, found %.3f, drift %.3f  -- %s",
            s_expect, before, drift,
            (std::fabs(drift) < 0.001f) ? "OUR WRITE PERSISTS (wrong array, not wrong timing)"
                                        : "GAME OVERWRITES US (wrong hook point, not wrong array)");
    }
    ++s_n;

    n->position.y = before + g_cfg.palette_poke_amt;
    s_expect = n->position.y;
    s_have = true;

    static uint32_t s_said = 0;
    if (s_said < 3) {
        ++s_said;
        API::get()->log_info(
            "[Halo-CampE-UEVR] PALETTE poke: candidate #%d at 0x%llX node %d += %.2f on Y "
            "(was %.3f)",
            which, (unsigned long long)c.at, node, g_cfg.palette_poke_amt, before);
    }
}

// ---------------------------------------------------------------- who writes the palette
namespace {

// Dr7 slot 0: L0 enable, RW0=01 (write), LEN0=11 (4 bytes -- one float, which is what a node
// component is). AimDirect watches 8 bytes because it watches a double; the length field must match
// the datum or the trap either misses or fires on neighbours.
uint64_t dr7_write4() { return 1ull | (0b01ull << 16) | (0b11ull << 18); }
// RW=11: trap on READ as well as write. The slide watch wants the mesh sync's read named too.
uint64_t dr7_rw4()    { return 1ull | (0b11ull << 16) | (0b11ull << 18); }

std::atomic<bool> g_watching{false};
uintptr_t         g_watch_addr = 0;
PVOID             g_veh = nullptr;
uintptr_t         g_self_lo = 0, g_self_hi = 0;

// Distinct writers seen, with hit counts. Several things touch a live object -- the pose build,
// memcpy, allocator fill -- so the useful output is a RANKED LIST, not the first hit. The one that
// runs every frame at the rig's cadence is the one worth hooking.
constexpr int kMaxWriters = 24;
// Per RIP, the FIRST trap's return address ([rsp]) and argument registers. For a trap inside
// memcpy (VCRUNTIME) the return address names the CALLER -- the function doing the copy -- and
// rax/rcx/rdx/r8 are dst/dst/src/size as memcpy received them (rax = original dst is preserved
// through the copy; rcx/rdx may have advanced). That is how a copied node block is located.
struct WriterHit {
    std::atomic<uintptr_t> rip; std::atomic<uint32_t> n;
    std::atomic<uintptr_t> ret, rax, rcx, rdx, r8, r9;
};
WriterHit g_writers[kMaxWriters];

void init_self_range() {
    if (g_self_lo != 0) return;
    HMODULE self = nullptr;
    if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                            GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                            (LPCSTR)&init_self_range, &self) || self == nullptr) {
        return;
    }
    MODULEINFO mi{};
    if (GetModuleInformation(GetCurrentProcess(), self, &mi, sizeof(mi))) {
        g_self_lo = (uintptr_t)mi.lpBaseOfDll;
        g_self_hi = g_self_lo + mi.SizeOfImage;
    }
}

// Debug registers are PER-THREAD, so every thread has to be armed. The palette is built on the sim
// thread, but which OS thread that is at any moment is not something worth assuming -- arming all
// of them is what AimDirect does and it is the reason its watch ever caught anything.
void arm_all(uintptr_t addr, bool rw = false) {
    const DWORD pid  = GetCurrentProcessId();
    const DWORD self = GetCurrentThreadId();

    auto set_on = [addr, rw](HANDLE th, bool is_self) {
        if (!is_self && SuspendThread(th) == (DWORD)-1) return;
        CONTEXT c{};
        c.ContextFlags = CONTEXT_DEBUG_REGISTERS;
        if (GetThreadContext(th, &c)) {
            c.Dr0 = addr;
            c.Dr7 = addr ? (rw ? dr7_rw4() : dr7_write4()) : 0;
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

LONG CALLBACK palette_veh(EXCEPTION_POINTERS* ep) {
    if (ep == nullptr || ep->ExceptionRecord == nullptr) return EXCEPTION_CONTINUE_SEARCH;
    if (ep->ExceptionRecord->ExceptionCode != EXCEPTION_SINGLE_STEP) return EXCEPTION_CONTINUE_SEARCH;
    if (!g_watching.load(std::memory_order_relaxed)) return EXCEPTION_CONTINUE_SEARCH;
    if ((ep->ContextRecord->Dr6 & 0x1ull) == 0) return EXCEPTION_CONTINUE_SEARCH;
    ep->ContextRecord->Dr6 = 0;

    // IGNORE OUR OWN STORES. blam_palette_poke() writes this exact address thousands of times a
    // second; without this filter the ranked list is our own poke and nothing else.
    const uintptr_t rip = (uintptr_t)ep->ContextRecord->Rip;
    if (g_self_lo != 0 && rip >= g_self_lo && rip < g_self_hi) return EXCEPTION_CONTINUE_EXECUTION;

    // Minimum work, nothing that can fault. No dereferencing, no module lookups, no logging: a
    // nested fault inside a vectored handler is not a debuggable event.
    for (int i = 0; i < kMaxWriters; ++i) {
        uintptr_t cur = g_writers[i].rip.load(std::memory_order_relaxed);
        if (cur == rip) { g_writers[i].n.fetch_add(1, std::memory_order_relaxed); break; }
        if (cur == 0) {
            uintptr_t expected = 0;
            if (g_writers[i].rip.compare_exchange_strong(expected, rip)) {
                const CONTEXT* c = ep->ContextRecord;
                uintptr_t ret = 0;
                if (c->Rsp != 0 && (c->Rsp & 7ull) == 0) ret = *reinterpret_cast<const uintptr_t*>(c->Rsp);
                g_writers[i].ret.store(ret, std::memory_order_relaxed);
                g_writers[i].rax.store((uintptr_t)c->Rax, std::memory_order_relaxed);
                g_writers[i].rcx.store((uintptr_t)c->Rcx, std::memory_order_relaxed);
                g_writers[i].rdx.store((uintptr_t)c->Rdx, std::memory_order_relaxed);
                g_writers[i].r8.store((uintptr_t)c->R8, std::memory_order_relaxed);
                g_writers[i].r9.store((uintptr_t)c->R9, std::memory_order_relaxed);
                g_writers[i].n.fetch_add(1, std::memory_order_relaxed);
                break;
            }
            if (g_writers[i].rip.load(std::memory_order_relaxed) == rip) {
                g_writers[i].n.fetch_add(1, std::memory_order_relaxed);
                break;
            }
        }
    }
    return EXCEPTION_CONTINUE_EXECUTION;
}

// The start of the function containing a trapped RIP, and how much to believe it.
//
// A trap lands mid-function, on the store. A hook goes on the PROLOGUE, so the captured address is
// not directly usable and the gap has to be closed reliably -- a hook installed at a wrong entry
// corrupts the instruction stream of a function running thousands of times a second.
//
// AUTHORITATIVE FIRST. Every x64 PE carries a .pdata exception directory with a RUNTIME_FUNCTION
// per non-leaf function, and RtlLookupFunctionEntry resolves an address to its containing entry.
// That is the binary's own record of where its functions begin -- the same data the OS unwinder
// trusts to walk stacks -- so it is not a guess and does not care about compiler padding style.
//
// The int3 walk is kept only as a fallback for the case .pdata cannot answer: LEAF functions are
// permitted to have no unwind entry. It is reported as such, because it IS a heuristic -- a
// jump-table target may have no padding before it, and 0xCC occurs inside data embedded in code.
uintptr_t function_start_from(uintptr_t rip, bool* out_authoritative) {
    if (out_authoritative) *out_authoritative = false;

    DWORD64 image_base = 0;
    if (auto* rf = RtlLookupFunctionEntry((DWORD64)rip, &image_base, nullptr)) {
        if (image_base != 0) {
            if (out_authoritative) *out_authoritative = true;
            return (uintptr_t)(image_base + rf->BeginAddress);
        }
    }

    constexpr uintptr_t kMaxBack = 0x2000;
    int run = 0;
    for (uintptr_t back = 1; back < kMaxBack; ++back) {
        const uintptr_t p = rip - back;
        if (IsBadReadPtr((const void*)p, 1)) return 0;
        if (*reinterpret_cast<const uint8_t*>(p) == 0xCC) {
            if (++run >= 2) return p + run;   // first byte past the padding run
        } else {
            run = 0;
        }
    }
    return 0;
}

// Name a captured RIP: which module holds it, and at what RVA. Resolved HERE rather than in the
// handler -- this is the value we actually need, because an RVA is what a hook is installed at.
void report_writers() {
    API::get()->log_info("[Halo-CampE-UEVR] PALETTEWATCH: writers seen at 0x%llX",
                         (unsigned long long)g_watch_addr);
    bool any = false;
    for (int i = 0; i < kMaxWriters; ++i) {
        const uintptr_t rip = g_writers[i].rip.load(std::memory_order_relaxed);
        const uint32_t  n   = g_writers[i].n.load(std::memory_order_relaxed);
        if (rip == 0) continue;
        any = true;
        char name[MAX_PATH] = "?";
        uintptr_t base = 0;
        HMODULE mod = nullptr;
        if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                               GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                               (LPCSTR)rip, &mod) && mod != nullptr) {
            base = (uintptr_t)mod;
            char full[MAX_PATH] = {0};
            if (GetModuleFileNameA(mod, full, MAX_PATH)) {
                const char* slash = strrchr(full, '\\');
                strncpy_s(name, sizeof(name), slash ? slash + 1 : full, _TRUNCATE);
            }
        }
        bool authoritative = false;
        const uintptr_t fn = function_start_from(rip, &authoritative);
        API::get()->log_info(
            "[Halo-CampE-UEVR] PALETTEWATCH   rip=0x%llX  %s+0x%llX  hits=%u   "
            "fn=+0x%llX (%s)  size=0x%llX",
            (unsigned long long)rip, name,
            (unsigned long long)(base ? rip - base : 0), n,
            (unsigned long long)((fn && base) ? fn - base : 0),
            authoritative ? ".pdata" : "int3 guess",
            (unsigned long long)((fn && rip >= fn) ? rip - fn : 0));
        // The first trap's return address (the caller, named by module+RVA) and registers.
        {
            const uintptr_t ret = g_writers[i].ret.load(std::memory_order_relaxed);
            char rname[MAX_PATH] = "?";
            uintptr_t rbase = 0;
            HMODULE rmod = nullptr;
            if (ret != 0 && GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                                              GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                                              (LPCSTR)ret, &rmod) && rmod != nullptr) {
                rbase = (uintptr_t)rmod;
                char full[MAX_PATH] = {0};
                if (GetModuleFileNameA(rmod, full, MAX_PATH)) {
                    const char* slash = strrchr(full, '\\');
                    strncpy_s(rname, sizeof(rname), slash ? slash + 1 : full, _TRUNCATE);
                }
            }
            bool rauth = false;
            const uintptr_t rfn = (ret != 0) ? function_start_from(ret, &rauth) : 0;
            API::get()->log_info(
                "[Halo-CampE-UEVR] PALETTEWATCH      ret=0x%llX  %s+0x%llX  caller fn=+0x%llX (%s)   "
                "rax=0x%llX rcx=0x%llX rdx=0x%llX r8=0x%llX r9=0x%llX   (watch 0x%llX)",
                (unsigned long long)ret, rname, (unsigned long long)(rbase ? ret - rbase : 0),
                (unsigned long long)((rfn && rbase) ? rfn - rbase : 0), rauth ? ".pdata" : "int3 guess",
                (unsigned long long)g_writers[i].rax.load(std::memory_order_relaxed),
                (unsigned long long)g_writers[i].rcx.load(std::memory_order_relaxed),
                (unsigned long long)g_writers[i].rdx.load(std::memory_order_relaxed),
                (unsigned long long)g_writers[i].r8.load(std::memory_order_relaxed),
                (unsigned long long)g_writers[i].r9.load(std::memory_order_relaxed),
                (unsigned long long)g_watch_addr);
        }
    }
    if (!any) {
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEWATCH   nothing trapped -- the breakpoint "
                             "did not arm, or this node is not written by the CPU we watched");
    }
}

} // namespace

void blam_palette_watch() {
    if (g_cfg.palette_watch == 0) {
        if (g_watching.load(std::memory_order_relaxed)) {
            arm_all(0);
            g_watching.store(false, std::memory_order_relaxed);
            if (g_veh != nullptr) { RemoveVectoredExceptionHandler(g_veh); g_veh = nullptr; }
            report_writers();
            for (int i = 0; i < kMaxWriters; ++i) {
                g_writers[i].rip.store(0, std::memory_order_relaxed);
                g_writers[i].n.store(0, std::memory_order_relaxed);
            }
        }
        return;
    }
    if (g_watching.load(std::memory_order_relaxed)) {
        // REPORT ON WALL CLOCK AND ON CHANGE, not on a call count.
        //
        // This was `% 8000` calls, which was right when the watch rode the sim hook at frame rate --
        // a few seconds. It is now driven from blam_palette_hook_tick, once every 64 ticks, so 8000
        // calls is over an HOUR: the 2026-08-17 session armed the watch successfully at 14:58:29 and
        // could not have printed a writer before the headset came off, whatever it trapped.
        // Report the moment the first writer appears, then whenever the set changes, then every 10 s.
        static uint32_t s_seen = 0;
        static ULONGLONG s_last = 0;
        uint32_t total = 0;
        for (int i = 0; i < kMaxWriters; ++i) total += g_writers[i].n.load(std::memory_order_relaxed);
        const ULONGLONG now = GetTickCount64();
        if (total != s_seen || (total > 0 && now - s_last >= 10000ull)) {
            s_seen = total; s_last = now;
            report_writers();
        }
        return;
    }

    const int which = g_cfg.palette_watch;
    const int count = g_cand_count.load(std::memory_order_acquire);
    if (which > count || which <= 0) return;
    const Candidate c = g_cand[which - 1];
    if (c.at == 0) return;

    int32_t node = g_cfg.palette_poke_node;
    if (node < 0 || node >= c.nodes) node = 0;

    // Watch ONE component of one node. Four bytes is the whole datum, and a narrow watch is what
    // keeps the trap on the pose write rather than on every neighbour a wide store happens to span.
    g_watch_addr = c.at + (uintptr_t)node * sizeof(PaletteNode) + offsetof(PaletteNode, position)
                 + offsetof(NodeMatrix, y);
    if ((g_watch_addr & 3ull) != 0) {
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEWATCH: 0x%llX is not 4-aligned, refusing",
                             (unsigned long long)g_watch_addr);
        return;
    }

    init_self_range();
    for (int i = 0; i < kMaxWriters; ++i) {
        g_writers[i].rip.store(0, std::memory_order_relaxed);
        g_writers[i].n.store(0, std::memory_order_relaxed);
    }
    if (g_veh == nullptr) g_veh = AddVectoredExceptionHandler(1, palette_veh);
    if (g_veh == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEWATCH: AddVectoredExceptionHandler failed");
        return;
    }
    g_watching.store(true, std::memory_order_relaxed);
    arm_all(g_watch_addr);
    API::get()->log_info(
        "[Halo-CampE-UEVR] PALETTEWATCH: armed on candidate #%d node %d, 0x%llX (position.y)",
        which, node, (unsigned long long)g_watch_addr);
}

// SLIDE ACCESSORS (paletteslidewatch). The copy scan (2026-09-03) found no second copy of the weapon's
// node block in the capture region or the object, so whatever renders the slide reads THIS node;
// yet a value written there on the sim thread after the pose and again on the game thread before
// the engine tick never shows. A READ+WRITE hardware watch on the node's position.y traps every
// CPU access: the sim's rebuild (writer) and the mesh sync's read (reader), each as module+RVA,
// which is exactly where a hook lands a write that survives. Same trap list, handler and report
// as palettewatch (our own module's accesses are filtered in the handler). Game thread.
// `release` = palettewpn switched off: disarm whatever is armed, whatever the keys say.
void blam_slide_watch(bool release) {
    static uintptr_t s_armed = 0;
    auto disarm = [&](bool report) {
        arm_all(0);
        g_watching.store(false, std::memory_order_relaxed);
        if (report) report_writers();
        for (int i = 0; i < kMaxWriters; ++i) {
            g_writers[i].rip.store(0, std::memory_order_relaxed);
            g_writers[i].n.store(0, std::memory_order_relaxed);
        }
        s_armed = 0;
    };
    if (release || !g_cfg.slide_watch || g_cfg.palette_watch != 0) {
        if (s_armed != 0) {
            disarm(true);
            if (g_veh != nullptr && g_cfg.palette_watch == 0) { RemoveVectoredExceptionHandler(g_veh); g_veh = nullptr; }
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEWATCH: disarmed");
        }
        return;
    }
    const uintptr_t node = g_slide_node_addr.load(std::memory_order_relaxed);
    if (node == 0) return;
    const uintptr_t want = node + offsetof(PaletteNode, position) + offsetof(NodeMatrix, y);
    if (want != s_armed) {
        if (s_armed != 0) disarm(true);
        init_self_range();
        if (g_veh == nullptr) g_veh = AddVectoredExceptionHandler(1, palette_veh);
        if (g_veh == nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEWATCH: AddVectoredExceptionHandler failed");
            return;
        }
        g_watch_addr = want;
        g_watching.store(true, std::memory_order_relaxed);
        arm_all(want, true);
        s_armed = want;
        API::get()->log_info("[Halo-CampE-UEVR] SLIDEWATCH: armed READ+WRITE on slide node position.y at 0x%llX (node 0x%llX)",
                             (unsigned long long)want, (unsigned long long)node);
        return;
    }
    static uint32_t  s_seen = 0;
    static ULONGLONG s_last = 0;
    uint32_t total = 0;
    for (int i = 0; i < kMaxWriters; ++i) total += g_writers[i].n.load(std::memory_order_relaxed);
    const ULONGLONG now = GetTickCount64();
    if (total != s_seen || (total > 0 && now - s_last >= 10000ull)) {
        s_seen = total; s_last = now;
        report_writers();
    }
}

// ---------------------------------------------------------------- owning the pose
namespace {

// THE FIRST-PERSON BUILD ROUTINE.
//
// Not either function the watch caught. Those two (dll+0x257680 and dll+0x257ED0) are the low-level
// matrix writers this one drives, which is why they trapped with identical hit counts and why
// writing after either of them corrupted state the other still needed. This is the routine that
// owns the whole build, so after it returns the pose for that weapon slot is complete.
//
// THREE arguments, void return -- and knowing that matters as much as the address. The earlier
// detour guessed four, which on x64 means it forwarded a garbage fourth register and built a call
// frame of the wrong shape; that is enough on its own to explain a crash the instant the hook did
// any real work.
//
// The arguments are also the answer to "which palette". local_player and weapon_slot index straight
// to the right one, so the four scanned candidates stop being a guess to resolve.
// 2026-08-17 game update: .text shifted by exactly +0x10 and .data did not move, so this went
// 0x46EC10 -> 0x46EC20. Verified statically against the shipped DLL, not inferred:
//   * 0x46EC20 is a .pdata function start (0x46EC20-0x46FCB2, size 0x1092); 0x46EC10 is not
//   * it xrefs SHARED_CAPTURE_PTR (0x1831220) exactly twice, at 0x46FA07 and 0x46FBC2
//   * its prologue spills ecx, edx and r8b -- (int32 local_player, int32 weapon_slot, bool)
// The old address held `44 89 42 38 C3 CC CC...` -- mov [rdx+38],r8d / ret / padding, i.e. the TAIL
// of the previous function. Installing a jump over a ret is why it crashed within 5 ms.
constexpr uintptr_t RVA_FP_BUILD = 0x46EC20;

// The first bytes of that function, checked before the hook goes in. An offset is only true for one
// build; a prologue is evidence. If the next update moves this again the log will say "prologue
// mismatch" and the hook will stay off, instead of patching the middle of whatever moved in.
constexpr uint8_t FP_BUILD_PROLOGUE[] = {
    0x48,0x8B,0xC4, 0x44,0x88,0x40,0x18, 0x89,0x50,0x10, 0x89,0x48,0x08, 0x53,0x56,0x57
};

using FpBuildFn = void (*)(int32_t, int32_t, bool);

FpBuildFn g_pose_original = nullptr;
int       g_pose_hook_id  = -1;
int       g_pose_hooked_which = 0;

// The per-player, per-slot structure the palette lives in, reached from the sim's thread-local
// block. Offsets are properties of this build of the game, validated below rather than trusted:
// the node counts sit immediately before the array and must both read 76, which is the same
// corroboration the shape scan used, applied here as a precondition instead of as evidence.
constexpr uintptr_t OFF_TLS_PLAYER_BANK = 0x4F8;
constexpr uintptr_t PLAYER_STRIDE       = 0x52D8;
constexpr uintptr_t WEAPON_SLOT_STRIDE  = 0x2908;
constexpr uintptr_t OFF_SLOT_FLAGS      = 0x38;
constexpr uintptr_t OFF_SLOT_OBJECT     = 0x44;
constexpr uintptr_t OFF_SLOT_ANIMATION  = 0x58;
constexpr uintptr_t OFF_SLOT_MODEL_TAG  = 0x194;
constexpr uintptr_t OFF_SOURCE_COUNT    = 0x108C;
constexpr uintptr_t OFF_FINAL_COUNT     = 0x1090;
constexpr uintptr_t OFF_FINAL_PALETTE   = 0x1094;
constexpr int32_t   FP_NODE_COUNT       = 76;

// The live palette for one player's weapon slot, or nullptr. Every gate here is a reason a slot can
// legitimately hold nothing -- no weapon, mid-swap, a slot that was never populated -- so a null
// return is ordinary and must not be treated as a failure worth logging every frame.
// THE RENDER PALETTE IS A DIFFERENT ARRAY.
//
// Writing the live palette changes nothing on screen, because the renderer does not read it. The
// build routine also CAPTURES the pose into a separate structure, and that copy is what gets drawn
// -- which is why a write that provably lands, at provably the right moment, was still invisible.
//
// TWO BANKS, both written. The renderer blends the previous bank against the current one for
// sub-tick smoothness. That is right for stock animation and wrong for anything we drive: blending
// a fresh pose against a stale one drags the result backwards. Writing both endpoints makes the
// blend collapse to our value while stock nodes keep their normal interpolation.
constexpr uintptr_t RVA_SHARED_CAPTURE_PTR = 0x1831220;
constexpr uintptr_t OFF_TLS_CAPTURE_CTX    = 0x5B8;
constexpr uintptr_t CAPTURE_CTX_STRIDE     = 0x30600;
constexpr uintptr_t CAPTURE_PLAYER_STRIDE  = 0x30D4;
constexpr uintptr_t CAPTURE_SLOT_STRIDE    = 0x1868;
constexpr uintptr_t OFF_CAPTURE_TAG        = 0x24010;
constexpr uintptr_t OFF_CAPTURE_COUNT      = 0x24014;
constexpr uintptr_t OFF_CAPTURE_PALETTE    = 0x24018;

// What the live slot holds, so the capture banks can be matched against it. A capture record is
// only ours if its model tag and node count agree with the slot the builder just filled -- without
// that check we would write a bank belonging to a different weapon or a stale build.
struct LiveSlot {
    PaletteNode* palette = nullptr;
    int32_t      model_tag = 0;
    int32_t      count = 0;
};

PaletteNode* live_palette_for(int32_t local_player, int32_t weapon_slot, LiveSlot* out = nullptr);

PaletteNode* live_palette_for(int32_t local_player, int32_t weapon_slot, LiveSlot* out) {
    const uintptr_t tls_array = (uintptr_t)__readgsqword(0x58);
    if (tls_array == 0) return nullptr;
    const HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
    if (sim == nullptr) return nullptr;
    const uint32_t tls_index = *(const uint32_t*)((uintptr_t)sim + RVA_TLS_INDEX);

    if (!readable((const void*)(tls_array + (uintptr_t)tls_index * 8), 8)) return nullptr;
    const uintptr_t block = *reinterpret_cast<const uintptr_t*>(tls_array + (uintptr_t)tls_index * 8);
    if (block == 0) return nullptr;

    if (!readable((const void*)(block + OFF_TLS_PLAYER_BANK), 8)) return nullptr;
    const uintptr_t bank = *reinterpret_cast<const uintptr_t*>(block + OFF_TLS_PLAYER_BANK);
    if (bank == 0) return nullptr;

    const uintptr_t slot = bank + (uintptr_t)local_player * PLAYER_STRIDE
                                + (uintptr_t)weapon_slot  * WEAPON_SLOT_STRIDE;
    if (!readable((const void*)slot, OFF_FINAL_PALETTE + sizeof(PaletteNode))) return nullptr;

    const uint32_t flags     = *reinterpret_cast<const uint32_t*>(slot + OFF_SLOT_FLAGS);
    const int32_t  object    = *reinterpret_cast<const int32_t*>(slot + OFF_SLOT_OBJECT);
    const int32_t  animation = *reinterpret_cast<const int32_t*>(slot + OFF_SLOT_ANIMATION);
    const int32_t  model_tag = *reinterpret_cast<const int32_t*>(slot + OFF_SLOT_MODEL_TAG);
    const int32_t  src_count = *reinterpret_cast<const int32_t*>(slot + OFF_SOURCE_COUNT);
    const int32_t  fin_count = *reinterpret_cast<const int32_t*>(slot + OFF_FINAL_COUNT);

    if ((flags & 0x0C) != 0x0C) return nullptr;          // slot not live
    if (object == -1 || animation == -1 || model_tag == -1) return nullptr;
    if (src_count != FP_NODE_COUNT || fin_count != FP_NODE_COUNT) return nullptr;

    auto* pal = reinterpret_cast<PaletteNode*>(slot + OFF_FINAL_PALETTE);
    if (out != nullptr) { out->palette = pal; out->model_tag = model_tag; out->count = fin_count; }
    return pal;
}

// STEADY-STATE BANK TELEMETRY. The renderer blends the previous interpolation bank against the
// current one; a build whose fresh pose lands in fewer than BOTH banks renders dragged back toward
// stale data -- the reference's own history calls the symptom "damped, delayed aim", and a partial
// landing RATE reads as the gun moving a fixed fraction of the hand. The first-5 log lines proved
// banks CAN land (0,0,1,2,2 at weapon spawn) and said nothing about whether they KEEP landing.
// These counters answer that for a whole session, split by the reason a bank was missed.
uint32_t s_bank_builds  = 0;   // builds with the weapon branch applied
uint32_t s_bank_hist[3] = {0, 0, 0};   // how many banks each build reached
uint32_t s_bank_gate    = 0;   // capture context absent or closed this tick
uint32_t s_bank_tagmiss = 0;   // a bank record described a different weapon
uint32_t s_bank_unread  = 0;   // a bank record failed the readability guard

// Displace the same node in both render banks. Returns how many banks were actually written, which
// is the number worth logging: zero means the capture path was skipped and the write is still
// invisible no matter how correct the live palette write was.
// idx[i] receives the bank number (0/1) of out[i]; *cur receives the capture context's own bank
// index byte (the one the gate below already reads), which names the bank being built THIS tick.
int collect_capture_banks(int32_t local_player, int32_t weapon_slot,
                          const LiveSlot& live, PaletteNode* out[2],
                          int idx[2] = nullptr, int* cur = nullptr) {
    const uintptr_t tls_array = (uintptr_t)__readgsqword(0x58);
    if (tls_array == 0) return 0;
    const HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
    if (sim == nullptr) return 0;
    const uint32_t tls_index = *(const uint32_t*)((uintptr_t)sim + RVA_TLS_INDEX);
    if (!readable((const void*)(tls_array + (uintptr_t)tls_index * 8), 8)) return 0;
    const uintptr_t block = *reinterpret_cast<const uintptr_t*>(tls_array + (uintptr_t)tls_index * 8);
    if (block == 0) return 0;

    if (!readable((const void*)(block + OFF_TLS_CAPTURE_CTX), 8)) { ++s_bank_gate; return 0; }
    auto* ctx = *reinterpret_cast<uint8_t**>(block + OFF_TLS_CAPTURE_CTX);
    if (ctx == nullptr || !readable(ctx, 4)) { ++s_bank_gate; return 0; }
    if (ctx[2] == 0 || ctx[0] >= 2) { ++s_bank_gate; return 0; }   // capture not active this tick
    // ---- THE ENGINE'S OWN BLEND WEIGHT (2026-09-12, from verified disassembly of the bank blend
    // at sim+0x23DAC0). That routine loads its interpolation factor with
    //     vbroadcastss xmm6, dword [rax+4]
    // off this exact ctx pointer, and feeds it to the per-node NLERP at sim+0x23BF40 as
    //     out = qA + t*(qB' - qA)
    // with ctx[0] and ctx[1] naming the two source banks. So the weight is a real float four
    // bytes past a pointer this function already holds, and it has never been read.
    //
    // BLENDT inferred t from geometry and came back centred on 1.00 with 23-39% of frames
    // OVERSHOOTING to as much as 4.0, which no interpolation between two poses can produce. This
    // reads the engine's actual value instead of inferring it, and logs the two bank indices
    // beside it, because a blend is only a no-op if BOTH named banks hold our pose. If ctx[1]
    // ever names a bank collect_capture_banks did not hand us, we are being interpolated against
    // a buffer we never wrote.
    if (readable(ctx, 8)) {
        const float t_eng = *reinterpret_cast<const float*>(ctx + 4);
        g_blend_t.store(std::isfinite(t_eng) ? t_eng : -999.0f, std::memory_order_relaxed);
        g_blend_a.store((int)ctx[0], std::memory_order_relaxed);
        g_blend_b.store((int)ctx[1], std::memory_order_relaxed);
    }
    if (cur != nullptr) *cur = (int)ctx[0];

    if (!readable((const void*)((uintptr_t)sim + RVA_SHARED_CAPTURE_PTR), 8)) { ++s_bank_gate; return 0; }
    auto* shared = *reinterpret_cast<uint8_t**>((uintptr_t)sim + RVA_SHARED_CAPTURE_PTR);
    if (shared == nullptr) { ++s_bank_gate; return 0; }

    int found = 0;
    for (uint8_t bank = 0; bank < 2; ++bank) {
        uint8_t* rec = shared + (size_t)bank * CAPTURE_CTX_STRIDE
                              + (size_t)local_player * CAPTURE_PLAYER_STRIDE
                              + (size_t)weapon_slot * CAPTURE_SLOT_STRIDE;
        if (!readable(rec, OFF_CAPTURE_PALETTE + sizeof(PaletteNode) * (size_t)FP_NODE_COUNT)) { ++s_bank_unread; continue; }
        const int32_t tag   = *reinterpret_cast<const int32_t*>(rec + OFF_CAPTURE_TAG);
        const int32_t count = *reinterpret_cast<const int32_t*>(rec + OFF_CAPTURE_COUNT);
        // Only ours if it describes the same weapon the builder just filled.
        if (tag != live.model_tag || count != live.count) { ++s_bank_tagmiss; continue; }
        auto* pal = reinterpret_cast<PaletteNode*>(rec + OFF_CAPTURE_PALETTE);
        if (IsBadWritePtr(pal, sizeof(PaletteNode) * (size_t)FP_NODE_COUNT)) { ++s_bank_unread; continue; }
        if (idx != nullptr) idx[found] = (int)bank;
        out[found++] = pal;
    }
    return found;
}

// The test displacement, applied AFTER the pass we own has finished. If this is the last pass
// before the renderer reads the palette, it survives and the rig visibly moves. If an later pass
// still runs, it is overwritten and nothing happens -- which is the answer to "which one is last",
// obtained by trying rather than by inferring.
// ---- Blam basis maths -------------------------------------------------------------------------
// A basis stored the way the palette stores one: three column vectors. transform_vector is
// therefore forward*x + left*y + up*z, and everything else follows from that.
struct Mat3 { NodeMatrix forward, left, up; };

inline NodeMatrix mul(const NodeMatrix& v, float s) { return {v.x * s, v.y * s, v.z * s}; }
inline NodeMatrix add(const NodeMatrix& a, const NodeMatrix& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline NodeMatrix sub(const NodeMatrix& a, const NodeMatrix& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline NodeMatrix xform(const Mat3& m, const NodeMatrix& v) {
    return add(add(mul(m.forward, v.x), mul(m.left, v.y)), mul(m.up, v.z));
}
inline Mat3 mat_mul(const Mat3& a, const Mat3& b) {
    return {xform(a, b.forward), xform(a, b.left), xform(a, b.up)};
}
inline Mat3 mat_transpose(const Mat3& m) {
    return {{m.forward.x, m.left.x, m.up.x},
            {m.forward.y, m.left.y, m.up.y},
            {m.forward.z, m.left.z, m.up.z}};
}
inline bool norm_node(NodeMatrix* v) {
    const float l = std::sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
    if (!(l > 1.0e-6f) || !std::isfinite(l)) return false;
    v->x /= l; v->y /= l; v->z /= l;
    return true;
}
inline Mat3 basis_of(const PaletteNode& n) {
    Mat3 m{n.forward, n.left, n.up};
    norm_node(&m.forward); norm_node(&m.left); norm_node(&m.up);
    return m;
}
inline bool basis_ok(const Mat3& m) {
    for (const NodeMatrix* v : {&m.forward, &m.left, &m.up}) {
        const float l2 = v->x * v->x + v->y * v->y + v->z * v->z;
        if (!std::isfinite(l2) || l2 < 0.64f || l2 > 1.56f) return false;
    }
    return true;
}

// A basis WE CONSTRUCT and hand to the palette must also be RIGHT-HANDED. New 2026-08-15: for a
// full day basis_ok accepted a REFLECTION (det -1) because it checks lengths only, and the mirror
// rendered as the weapon orbiting the player on a wrist spin. Applied to our own output, not to
// the stock bases read from the palette (0.5's working pipeline measures det = +1 for those, so
// the game is right-handed, but the stock read is the game's business, not ours to veto).
inline bool basis_is_rotation(const Mat3& m) {
    if (!basis_ok(m)) return false;
    const float det = m.forward.x * (m.left.y * m.up.z - m.left.z * m.up.y)
                    - m.forward.y * (m.left.x * m.up.z - m.left.z * m.up.x)
                    + m.forward.z * (m.left.x * m.up.y - m.left.y * m.up.x);
    return std::isfinite(det) && det > 0.5f;
}

// OpenXR is X right / Y up / -Z forward; Blam is X forward / Y left / Z up. One conversion, used
// for both positions and basis axes, so a sign error shows everywhere at once rather than hiding in
// one of them.
inline NodeMatrix xr_to_blam(const Vec3& v) { return {-v.z, -v.x, v.y}; }

inline Mat3 xr_rot_to_blam_basis(const Quat& q) {
    return {xr_to_blam(quat_rotate(q, Vec3{ 0.0f, 0.0f, -1.0f})),
            xr_to_blam(quat_rotate(q, Vec3{-1.0f, 0.0f,  0.0f})),
            xr_to_blam(quat_rotate(q, Vec3{ 0.0f, 1.0f,  0.0f}))};
}

// UE axes (X forward, Y RIGHT, Z up) -> Blam axes (X forward, Y LEFT, Z up). One sign. The
// publisher divides by the MEASURED parent in UE space -- the rig path's proven frame -- so the
// hook receives UE-axis values and converts here.
inline NodeMatrix ue_to_blam(const Vec3& v) { return {v.x, -v.y, v.z}; }

// Renormalise. Every rotation composed into a persisted calibration MUST be unit: a non-unit
// quaternion in quat_rotate is a scale-and-skew rather than a rotation, and a chain of composes
// (capture -> file -> reload -> capture) amplifies any drift. Observed 2026-08-16: gripfix reached
// |q| = 1.028 over four captures, which logs as "rotated 0 deg" while the gun visibly moves.
inline Quat quat_unit(Quat q) {
    const float n = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (!(n > 1.0e-6f) || !std::isfinite(n)) return Quat{0.0f, 0.0f, 0.0f, 1.0f};
    return Quat{q.x / n, q.y / n, q.z / n, q.w / n};
}

// The twist of q about unit axis a (swing-twist decomposition): the part of the rotation that is
// purely about that axis, with the swing discarded. Used to keep the GLOBAL grip capture to pitch
// only -- the pose's +Y in UE convention -- so yaw and roll always come from the controller.
inline Quat quat_twist(const Quat& q, const Vec3& a) {
    const float d = q.x * a.x + q.y * a.y + q.z * a.z;
    Quat t{a.x * d, a.y * d, a.z * d, q.w};
    const float n = std::sqrt(t.x*t.x + t.y*t.y + t.z*t.z + t.w*t.w);
    if (!(n > 1.0e-6f) || !std::isfinite(n)) return Quat{0.0f, 0.0f, 0.0f, 1.0f};
    return Quat{t.x / n, t.y / n, t.z / n, t.w / n};
}

inline Mat3 ue_rot_to_blam_basis(const Quat& q) {
    return {ue_to_blam(quat_rotate(q, Vec3{1.0f,  0.0f, 0.0f})),
            ue_to_blam(quat_rotate(q, Vec3{0.0f, -1.0f, 0.0f})),
            ue_to_blam(quat_rotate(q, Vec3{0.0f,  0.0f, 1.0f}))};
}

constexpr int kFirstPersonNodeCount = 76;
constexpr float kMetersPerBlamUnit = 3.048f;

struct NodeSpan { const uint8_t* p; size_t n; };
template <size_t N> constexpr NodeSpan span_of(const uint8_t (&a)[N]) { return {a, N}; }

inline float dotn(const NodeMatrix& a, const NodeMatrix& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
inline NodeMatrix crossn(const NodeMatrix& a, const NodeMatrix& b) {
    return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}
inline float lenn(const NodeMatrix& a) { return std::sqrt(dotn(a, a)); }
inline bool finiten(const NodeMatrix& a) { return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z); }
inline Mat3 mat_identity() { return {{1,0,0},{0,1,0},{0,0,1}}; }

// Shortest-arc rotation taking direction a onto direction b (Rodrigues). Identity when parallel;
// 180 about any perpendicular when anti-parallel.
inline Mat3 rotation_between(NodeMatrix a, NodeMatrix b) {
    if (!norm_node(&a) || !norm_node(&b)) return mat_identity();
    NodeMatrix v = crossn(a, b);
    const float s = lenn(v), c = dotn(a, b);
    if (s < 1.0e-6f) {
        if (c > 0.0f) return mat_identity();
        NodeMatrix axis = crossn(a, {0, 0, 1});
        if (!norm_node(&axis)) { axis = crossn(a, {0, 1, 0}); norm_node(&axis); }
        // 180 about axis: R = 2 a a^T - I
        return {{2*axis.x*axis.x - 1, 2*axis.x*axis.y,     2*axis.x*axis.z},
                {2*axis.y*axis.x,     2*axis.y*axis.y - 1, 2*axis.y*axis.z},
                {2*axis.z*axis.x,     2*axis.z*axis.y,     2*axis.z*axis.z - 1}};
    }
    v = mul(v, 1.0f / s);
    const float k = 1.0f - c;
    // R = I + sin*[v]x + (1-cos)*[v]x^2 ; columns are R applied to e1,e2,e3
    const float x = v.x, y = v.y, z = v.z;
    return {{c + k*x*x,     s*z + k*x*y,  -s*y + k*x*z},
            {-s*z + k*x*y,  c + k*y*y,     s*x + k*y*z},
            {s*y + k*x*z,  -s*x + k*y*z,   c + k*z*z}};
}

inline void apply_rigid_delta(PaletteNode* palette, NodeSpan nodes, const Mat3& rot, const NodeMatrix& pivot) {
    for (size_t i = 0; i < nodes.n; ++i) {
        PaletteNode& m = palette[nodes.p[i]];
        NodeMatrix f = xform(rot, m.forward), l = xform(rot, m.left), u = xform(rot, m.up);
        norm_node(&f); norm_node(&l); norm_node(&u);
        m.forward = f; m.left = l; m.up = u;
        m.position = add(pivot, xform(rot, sub(m.position, pivot)));
    }
}





inline float basis_min_alignment(const Mat3& a, const Mat3& b) {
    return (std::min)(dotn(a.forward, b.forward), (std::min)(dotn(a.left, b.left), dotn(a.up, b.up)));
}

// ---- poses, sampled on the game thread ---------------------------------------------------------
// The hook runs on the SIM thread. Rather than call the VR runtime from there, the game tick
// samples once and publishes; the hook reads plain floats. Same reason Hands.cpp reads a published
// turn quaternion instead of rebuilding it.
std::atomic<float> g_p_grip_x{0}, g_p_grip_y{0}, g_p_grip_z{0};
std::atomic<float> g_p_aim_x{0}, g_p_aim_y{0}, g_p_aim_z{0}, g_p_aim_w{1};
// The held weapon's ROTATIONAL delta (wpnfix rot), published separately: it is a VISUAL trim of
// the model relative to the aim ray and must be applied AFTER the barrel lock, in the pullback --
// applied before it, the lock cancels every pitch/yaw component (2026-08-15 20:07-20:10: nine AR
// captures accumulated 13 deg of pitch with no visible effect). Translation stays upstream.
std::atomic<float> g_p_wrot_x{0}, g_p_wrot_y{0}, g_p_wrot_z{0}, g_p_wrot_w{1};
// The HEAD's pitch in game degrees, published with the pose. Decoupled pitch is ON on this build
// (correctly -- a hand-driven camera must never pitch the view), so the rendered view pitches
// with the HEAD while the aim camera pitches with the HAND. The pullback cancels the AIM camera;
// the palette renders under the VIEW. Their pitch difference is what leaked -- bone Z = +0.116
// at cam pitch -45 with the head level -- and it is cancelled by re-applying (aim - head) pitch.
std::atomic<float> g_p_head_pitch{0};
// WPNERR raws: the aim hand exactly as sampled by the publisher, BEFORE any calibration --
// gripfix/wpnfix/two-hand all cancel out of a raw-vs-raw compare, so the instrument reads pure
// chain latency. Stamps are steady-clock ms.
std::atomic<float> g_p_raw_gx{0}, g_p_raw_gy{0}, g_p_raw_gz{0};
// PRE-EVERYTHING (2026-09-12). The g_p_raw_* above are a misnomer: they are stored AFTER the
// one-euro filter runs, so no instrument has ever seen the unprocessed controller pose. These
// are captured the instant get_pose returns, before the dead-tracking gate, the recenter, the
// swizzle, the grip fix, the aim fix, the two-hand delta and the filter. The paired test showed
// the placement math adds no jitter (median excess 0.0 to -105 deg/s^2, 43-54% of frames
// positive), so the jitter is already in the pose by publication; these columns split that
// between "the runtime handed us a jittery pose" and "our publisher made it jittery".
std::atomic<float> g_p_tru_gx{0}, g_p_tru_gy{0}, g_p_tru_gz{0};
std::atomic<float> g_p_tru_ax{0}, g_p_tru_ay{0}, g_p_tru_az{0}, g_p_tru_aw{1};
std::atomic<float> g_p_raw_ax{0}, g_p_raw_ay{0}, g_p_raw_az{0}, g_p_raw_aw{1};
std::atomic<long long> g_p_pub_ms{0};   // publisher sent (game tick)
std::atomic<long long> g_p_con_ms{0};   // hook consumed (sim build)
// SEQLOCK for the published weapon pose (grip xyz + aim quat). The writer bumps it odd before
// the seven stores and even after; a reader that sees it change mid-copy retries. Loose atomics
// let a sim-thread build consume a pose MIXED from two frames -- near-unit, wrong direction,
// one wrong rendered frame, only while the two frames differ, i.e. only while the wrist moves.
std::atomic<uint32_t> g_p_seq{0};
// The composed rigid grip offset (global gripfix + the held weapon's delta), stashed by the tick
// publisher so the per-frame republish can replay it without re-deriving weapon keys on the
// render thread. Translation metres in the RAW pose's frame; rotation is the GLOBAL part only,
// exactly as the publisher applies them.
std::atomic<float> g_p_offt_x{0}, g_p_offt_y{0}, g_p_offt_z{0};
std::atomic<float> g_p_rotg_x{0}, g_p_rotg_y{0}, g_p_rotg_z{0}, g_p_rotg_w{1};
std::atomic<bool>  g_p_valid{false};
// ---- THE RIGID GRIP OFFSET AND THE FREEZE. Game thread only (the publisher). See the pose-match
// block in blam_palette_publish_poses(). rot right-multiplies the pose; pos_m is metres in the
// CONTROLLER's frame, rotated by the corrected pose at apply time -- the rigid attachment.
Quat g_grip_fix_rot{0.0f, 0.0f, 0.0f, 1.0f};
Vec3 g_grip_fix_pos_m{0.0f, 0.0f, 0.0f};
bool g_grip_fix_valid = false;
// The calibration key, published by the game thread for the sim-side freeze in the pullback.
std::atomic<bool> g_freeze_key_held{false};

// ---- POSE HOLD: mask the game's own reload animation after the gesture reload. The player has
// just physically seated the magazine; watching the arms then act out a second reload is the
// wrong film. The hook snapshots the RAW 76-node pose (before our displacement) on the first sim
// frame after the seat, and keeps writing that snapshot over the live palette and both banks
// until the deadline -- the weapon branch displacement then runs on top as always, so the gun
// still rides the hand while the animation underneath is silenced. Game thread arms it
// (blam_palette_hold_pose); the sim thread does the rest with plain statics.
std::atomic<long long> g_pose_hold_until{0};
// A solved rigid delta handed BACK from the pullback (sim thread) to the publisher (game thread),
// which composes it onto the grip offset and persists it. One slot; the publisher clears it.
Quat g_pend_rot{0.0f, 0.0f, 0.0f, 1.0f};
Vec3 g_pend_pos_m{0.0f, 0.0f, 0.0f};
std::atomic<bool> g_pend_valid{false};

// The bone-frame result of the world-space pullback -- what the branch actually uses. Written by
// resolve_world_pullback() at the top of a build, read by apply_weapon_branch for the live
// palette and both banks, all inside one sim-thread call, so plain values are safe.
// PER-THREAD (2026-09-12). These were plain globals, with a comment claiming everything ran
// "inside one sim-thread call, so plain values are safe". That stopped being true when the
// per-frame refresh started calling resolve_world_pullback on the RENDER thread: both threads
// then wrote these six values and both read them back in apply_weapon_branch, with
// no lock and no atomics. A sim build whose read straddles a render write consumes a pose built
// from a DIFFERENT camera for exactly one build, and only while the two contexts disagree --
// which is only while the aim wrist is turning. thread_local removes the interleave by
// construction: each thread resolves into its own copy and reads its own copy back.
thread_local Vec3 g_eff_hand{0.0f, 0.0f, 0.0f};
thread_local Quat g_eff_pose{0.0f, 0.0f, 0.0f, 1.0f};
// The SHARED shadow, written by whichever thread resolved last. Not consumed by placement --
// it exists so the branch can measure how far the old shared-global behaviour would have been
// from this thread's own answer. race_deg / race_cm in the branch log are that difference, so
// the fix and the evidence for it land in one build.
Vec3 g_shr_hand{0.0f, 0.0f, 0.0f};
Quat g_shr_pose{0.0f, 0.0f, 0.0f, 1.0f};

// Bumps whenever the held weapon changes.
std::atomic<uint32_t> g_p_wpn_gen{0};
// The published hand pose BEFORE the room->world lift and the mesh divide. PALETTELOCAL needs the
// hand on its own, with no global frame attached to it.
thread_local Quat g_eff_raw_pose{0.0f, 0.0f, 0.0f, 1.0f};
// ---- THE FRAME LATCH (Config.hpp palette_latch). One hand for every write in a frame. Shared
// across threads deliberately: the build writer and the refresh writer touch the same banks, so
// a per-thread latch would leave exactly the disagreement this exists to remove. Seqlocked, odd
// while writing, same doctrine as g_p_seq.
struct HandLatch {
    std::atomic<uint32_t> seq{0};
    std::atomic<long long> ms{0};
    std::atomic<float> ax{0.0f}, ay{0.0f}, az{0.0f}, aw{1.0f};
    std::atomic<float> gx{0.0f}, gy{0.0f}, gz{0.0f};
};
HandLatch g_hl;
// ---- THE DIVISOR LATCH (Config.hpp comp_latch). Shared across threads deliberately: the whole
// point is that the sim-thread build writer and the render-thread refresh use the SAME sample.
// Seqlocked, odd while writing, same doctrine as g_p_seq.
struct CompLatch {
    std::atomic<uint32_t> seq{0};
    std::atomic<long long> ms{0};
    std::atomic<float> x{0.0f}, y{0.0f}, z{0.0f}, w{1.0f};
};
CompLatch g_cl;
// ---- READBACK (2026-09-12). From the headset: "are you logging your math? my bet is nope." Correct.
// Everything logged so far verified our math against ITSELF. res_deg compares
// delta_basis x weapon_basis against desired_basis, which is algebraically forced to zero, so it
// was a tautology. Nothing ever checked whether WHAT ENDED UP IN THE PALETTE IS WHAT WE WROTE.
// His standing claim is that the game's own pose fights ours. That is testable: publish the
// address of live node 8 and a byte copy of exactly what we last wrote there, then read the real
// memory back at the LAST possible moment before the draw and diff them. Point 13 already
// compares at the NEXT hook entry, a whole frame later, which cannot separate "the game re-posed
// for the next frame" (expected) from "the game overwrote us inside this frame" (the defect).
std::atomic<uintptr_t> g_n8_addr{0};
std::atomic<uint32_t>  g_n8_seq{0};
// SOCKGATE (2026-09-12). The corrected SOCKLINK shows the socket sitting EXACTLY on our written
// node 8 (angle bottoms out at 0.0000, typical 0.16-2.6 deg) and then departing by up to 30.53
// deg for isolated frames. READBACK proves the palette memory still holds our bytes at that same
// moment (0.000 cm), so the socket must reflect a skeleton evaluated at a different point than
// the palette we read: on most frames our write lands before the evaluation, on a few it does
// not and the socket shows the pose from before our write.
// palettebuildgate=6 exists to land the write before the game's build and currently reports 318 of
// 320 calls landing, so about one in a hundred misses. These two make the correlation testable:
// the age of our last write, and whether the hook that produced it got the gate-6 landing.
std::atomic<long long> g_n8_ms{0};
// BLENDT (2026-09-12). The stock node-8 basis, published beside what we wrote, so the drawn
// socket can be located ON THE ARC between them.
// WHY: the like-for-like check (BANDMATCH) confirmed the drawn weapon really does step ~1.5x the
// hand -- render/tick hand bandwidth came back 0.99-1.02, so the sampling confound is dead and
// the amplification is real. Everything we control is eliminated: the divisor in every variant
// including none, the lift, the barrel lock, the two writers disagreeing (clatch fired,
// reuse=637), the bone (node 8 owns the socket, relation bottoms at 0.0000), and the palette
// bytes (0.000 cm at draw time). What remains is the RE note that the bank blend at
// sim+0x23DAC0 reads TWO sources and that the render-side arena pair has never been written by
// this mod. A blend of our bank against an unwritten one is a lerp between our pose and
// something else, with a per-frame weight, which gives a ~1.5x step by construction.
// THE PREDICTION: the socket lies on the arc from stock to ours at a fraction t. t == 1 on every
// frame means no blending and this idea dies. t below 1, varying, IS the blend weight, and its
// variation is the judder.
std::atomic<float> g_n8_s_fx{0}, g_n8_s_fy{0}, g_n8_s_fz{0};
std::atomic<float> g_n8_s_lx{0}, g_n8_s_ly{0}, g_n8_s_lz{0};
std::atomic<float> g_n8_s_ux{0}, g_n8_s_uy{0}, g_n8_s_uz{0};
// BANKCHECK (2026-09-12). MY ERROR, found by SOCKGATE. READBACK was gated on ref == nullptr, so
// it verified the LIVE palette -- and the weapon is not drawn from the live palette, it is drawn
// from the CAPTURE BANKS. So "our bytes are intact at draw time, 0.000 cm" was proved about a
// buffer the renderer never reads, and SOCKLINK then compared the socket against that same wrong
// buffer. Meanwhile SOCKGATE showed the socket departing by up to 118.67 deg on frames where the
// build-time landing DID happen, and a write age reaching 81 ms, far outside the 6 ms latch.
// These publish node 8 exactly as it went into each bank, with the address and a timestamp, so
// the socket can be compared against the buffer it is actually drawn from.
struct BankShot {
    std::atomic<uint32_t> seq{0};
    std::atomic<uintptr_t> addr{0};
    std::atomic<long long> ms{0};
    std::atomic<float> fx{0}, fy{0}, fz{0}, lx{0}, ly{0}, lz{0}, ux{0}, uy{0}, uz{0};
};
BankShot g_bank8[2];
std::atomic<unsigned> g_bank8_n{0};
std::atomic<unsigned>  g_n8_gate6{0};
std::atomic<float> g_n8_w_fx{0}, g_n8_w_fy{0}, g_n8_w_fz{0};
std::atomic<float> g_n8_w_lx{0}, g_n8_w_ly{0}, g_n8_w_lz{0};
std::atomic<float> g_n8_w_ux{0}, g_n8_w_uy{0}, g_n8_w_uz{0};
std::atomic<float> g_n8_w_px{0}, g_n8_w_py{0}, g_n8_w_pz{0};

// Move the weapon branch so the gun sits where the hand holds it.
//
// ONE RIGID DELTA over nodes 7, 8 and 22, not a per-node solve. The branch is authored as a unit
// and animates as one -- recoil, reload, the trigger finger's own motion all live inside it -- so
// rotating the whole branch by a single transform moves the weapon while leaving every animation
// within it untouched. Solving nodes independently would flatten exactly the motion worth keeping.
// ---- THE POSE MATCH LIVES IN THE PUBLISHER NOW (blam_palette_publish_poses), not here.
//
// It used to be a palette-space basis+vector solved and applied in this function, with the freeze
// implemented as `return false` while the key was held. Both halves were wrong on this build: the
// palette-space residual produced a pivot that did not rotate with the wrist, and returning false
// dropped the weapon to the game's authored pose -- which is controller-driven here, so the
// "frozen" weapon kept moving with the very hand trying to align to it. The gesture is now 0.5's:
// the publisher latches the world target on key-down and republishes it while held (the weapon
// genuinely stops in the world), and on release solves a rigid controller-frame PoseOffset that is
// applied UPSTREAM to the pose. This function never learns a calibration exists.

// ---- BRANCHLOG: the audit of my own instrument (from the headset, 2026-09-12: "assume everything you
// wrote is wrong ... go back and double check"). The term dump stopped at g_eff_hand/g_eff_pose
// and called them final -- they are NOT. The branch expresses both through the GAME'S LIVE
// ANIMATED nodes: root_basis = basis_of(palette[0]) and weapon_basis = basis_of(palette[8]),
// read fresh every build, and basis_of only normalises each column -- it never orthogonalises.
// If the animation leaves those bases skewed or scaled, mat_transpose(weapon_basis) is not the
// inverse, the cancellation that should make node 8 land exactly on the target is imperfect, and
// the residual moves with the animation. The weapon's sway animation is driven by camera TURN
// RATE, which only the aim wrist produces. So this measures, per build: both reference bases'
// column lengths and orthogonality, their scale fields, and the actual residual between where
// node 8 is commanded and where the maths puts it.
namespace branchlog {
double now_ms() {
    return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct Row {
    double t_ms;
    unsigned long long buf;       // WHICH buffer this call posed: the branch runs ~5x per build
    float root_len[3], root_dot[3], root_scale, root_pos[3];
    float w8_len[3], w8_dot[3], w8_scale, w8_pos[3];
    float res_deg;                // desired_basis vs delta_basis*weapon_basis
    float res_cm;                 // desired_pos vs delta_pos + delta_basis*p8
    // THE SWAY PAIR. The delta cancels exactly for node 8 and for nothing else: every other
    // node is written as delta_pos + delta_basis * its own stock value, so anything the game's
    // animation puts BETWEEN node 8 (the weapon) and node 19 (the right wrist, placed from the
    // same delta) is carried into the arm and the gun with the lever from node 8
    // amplifying it. the player's report is that the default pose has no sway at all, so the
    // expectation is both columns dead flat -- which closes the hypothesis with a number instead
    // of an assumption. Angle between the two bases, and the distance between them.
    float sway_deg, sway_cm;
    float ours_cm;                // |palette[8] - our last written node 8|
    float n8_n20_cm;              // |palette[8] - palette[20]|: rigid if constant
    float n8_n40_cm;              // |palette[8] - palette[40]|
    float desired_pos[3], delta_pos[3];
};
constexpr int kCap = 40000;
Row g_rows[kCap];
std::atomic<int> g_n{0};
int g_seq = 0;
void put(const Row& r) {
    const int i = g_n.fetch_add(1, std::memory_order_relaxed);
    g_rows[i % kCap] = r;
}
void flush() {
    const int n = g_n.exchange(0, std::memory_order_relaxed);
    if (n <= 0 || g_cfg_path[0] == '\0') return;
    char path[MAX_PATH]; strncpy_s(path, sizeof(path), g_cfg_path, _TRUNCATE);
    char* slash = strrchr(path, '\\');
    if (slash == nullptr) return;
    char leaf[64]; sprintf_s(leaf, "halo_vr_branch_%03d.csv", g_seq++);
    slash[1] = '\0'; strncat_s(path, sizeof(path), leaf, _TRUNCATE);
    FILE* f = nullptr;
    if (fopen_s(&f, path, "wb") != 0 || f == nullptr) return;
    fprintf(f, "t_ms,buf,rlen_f,rlen_l,rlen_u,rdot_fl,rdot_fu,rdot_lu,rscale,rpos_x,rpos_y,rpos_z,"
               "wlen_f,wlen_l,wlen_u,wdot_fl,wdot_fu,wdot_lu,wscale,wpos_x,wpos_y,wpos_z,"
               "res_deg,res_cm,sway_deg,sway_cm,ours_cm,race_cm,race_deg,dp_x,dp_y,dp_z,dlt_x,dlt_y,dlt_z\r\n");
    const int m = n < kCap ? n : kCap;
    const int start = (n > kCap) ? (n % kCap) : 0;
    for (int i = 0; i < m; ++i) {
        const Row& r = g_rows[(start + i) % kCap];
        fprintf(f, "%.4f,%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.5f,%.5f,%.5f,"
                   "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.5f,%.5f,%.5f,"
                   "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\r\n",
                r.t_ms, r.buf, r.root_len[0], r.root_len[1], r.root_len[2],
                r.root_dot[0], r.root_dot[1], r.root_dot[2], r.root_scale,
                r.root_pos[0], r.root_pos[1], r.root_pos[2],
                r.w8_len[0], r.w8_len[1], r.w8_len[2],
                r.w8_dot[0], r.w8_dot[1], r.w8_dot[2], r.w8_scale,
                r.w8_pos[0], r.w8_pos[1], r.w8_pos[2],
                r.res_deg, r.res_cm, r.sway_deg, r.sway_cm, r.ours_cm, r.n8_n20_cm, r.n8_n40_cm,
                r.desired_pos[0], r.desired_pos[1], r.desired_pos[2],
                r.delta_pos[0], r.delta_pos[1], r.delta_pos[2]);
    }
    fclose(f);
    API::get()->log_info("[Halo-CampE-UEVR] BRANCHLOG: %d rows -> %hs", m, leaf);
}
} // namespace branchlog
// THE STOCK REFERENCE (measured 2026-09-12, not theorised). This function used to read its
// reference -- root_basis, weapon_basis and node 8's position -- out of the very buffer it then
// overwrote, and transform each node from that buffer's current contents. The branchlog capture
// showed what that means in practice: the render banks hold OUR OWN PREVIOUS OUTPUT on 49.0%,
// 48.7% and 48.1% of builds and the game's stock pose on the other ~50% -- a clean every-other-
// build alternation -- so the reference flipped by ~40 cm per build (39.7 / 40.3 / 40.4 cm
// measured per buffer) while the pose we actually wanted moved 0.05 cm. The one buffer that
// holds stock 100% of the time, the live palette, was simultaneously the only smooth one at
// 0.0030 cm. And transforming an already-transformed buffer compounds: the distance between
// node 8 and node 20, which is rigid in a solid weapon, varied by 4.1 cm, and node 8 to node 40
// by 5.6 cm. That is the judder, in centimetres, on the geometry itself.
//
// So the write is now a PURE FUNCTION of (stock, desired): the reference and every source node
// come from `ref`, the result goes into `palette`. Running it twice on the same buffer gives the
// same answer, and what the buffer previously held cannot influence the outcome.
bool apply_weapon_branch(PaletteNode* palette, Mat3* out_delta_basis = nullptr, NodeMatrix* out_delta_pos = nullptr,
                         const PaletteNode* ref = nullptr) {
                             CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    if (!g_p_valid.load(std::memory_order_acquire)) return false;

    const PaletteNode* R = (ref != nullptr) ? ref : palette;
    const Mat3 root_basis   = basis_of(R[0]);
    const Mat3 weapon_basis = basis_of(R[8]);
    if (!basis_ok(root_basis) || !basis_ok(weapon_basis)) return false;
    // ---- NODE0 (2026-09-12). The reference .5 build states palette node 0 is the stick-driven game
    // camera (its main.cpp:420-426); ours treats it as identity from a log that prints once every 512
    // builds. If node 0 moves with the aim, desired_basis = root_basis x visual applies the camera a
    // SECOND time on top of the divide. Measured here on every build: node 0's angle from identity and
    // its per-build change.
    if (g_cfg.palette_weapon_log) {
        const float tr0 = root_basis.forward.x + root_basis.left.y + root_basis.up.z;
        float c0 = (tr0 - 1.0f) * 0.5f; if (c0 > 1.0f) c0 = 1.0f; if (c0 < -1.0f) c0 = -1.0f;
        const float a0 = std::acos(c0) * RAD2DEG;
        static bool s_n0h = false; static Mat3 s_n0p{};
        static uint32_t s_n0n = 0; static double s_n0a = 0.0, s_n0s = 0.0; static float s_n0mx = 0.0f, s_n0smx = 0.0f;
        if (s_n0h) {
            const float trs = dot(root_basis.forward, s_n0p.forward) + dot(root_basis.left, s_n0p.left)
                            + dot(root_basis.up, s_n0p.up);
            float cs = (trs - 1.0f) * 0.5f; if (cs > 1.0f) cs = 1.0f; if (cs < -1.0f) cs = -1.0f;
            const float st = std::acos(cs) * RAD2DEG;
            ++s_n0n; s_n0a += a0; s_n0s += st;
            if (a0 > s_n0mx) s_n0mx = a0;
            if (st > s_n0smx) s_n0smx = st;
            if ((s_n0n % 480u) == 0u) {
                API::get()->log_info(
                    "[Halo-CampE-UEVR] NODE0 palette root basis vs identity mean %.4f worst %.4f deg | per-build "
                    "change mean %.4f worst %.4f deg  [~0 = identity, the divide is the only camera term; moving "
                    "= the camera is applied twice]",
                    s_n0a / s_n0n, s_n0mx, s_n0s / s_n0n, s_n0smx);
                s_n0n = 0; s_n0a = s_n0s = 0.0; s_n0mx = s_n0smx = 0.0f;
            }
        }
        s_n0p = root_basis; s_n0h = true;
    }
    // BLENDT's stock reference, captured HERE and not in the commit loop. In the commit loop
    // R == palette whenever ref is nullptr, and node 8 has already been overwritten with our own
    // pose by then, so it published OUR basis as "stock", the arc length came out zero and the
    // probe gated itself out entirely (BLENDT printed 0 times while its neighbours printed 8).
    // Same class of error as the bank call that used the already-written live palette as its
    // stock reference: reading a reference after writing through it.
    if (ref == nullptr) {
        g_n8_s_fx.store(R[8].forward.x, std::memory_order_relaxed);
        g_n8_s_fy.store(R[8].forward.y, std::memory_order_relaxed);
        g_n8_s_fz.store(R[8].forward.z, std::memory_order_relaxed);
        g_n8_s_lx.store(R[8].left.x, std::memory_order_relaxed);
        g_n8_s_ly.store(R[8].left.y, std::memory_order_relaxed);
        g_n8_s_lz.store(R[8].left.z, std::memory_order_relaxed);
        g_n8_s_ux.store(R[8].up.x, std::memory_order_relaxed);
        g_n8_s_uy.store(R[8].up.y, std::memory_order_relaxed);
        g_n8_s_uz.store(R[8].up.z, std::memory_order_relaxed);
    }

    // NODE 0, PERIODICALLY. Every composition here rides root_basis on the assumption that the
    // palette root is identity (it always read that way when sampled). If the root actually
    // TILTS with the aim under the view lock, we compose the camera in AND divide it out --
    // double-counting that no correction gain can be right for. One line answers it.
    if (g_cfg.palette_weapon_log) {
        static uint32_t s_r = 0;
        if ((s_r++ % 512u) == 0u) {
            API::get()->log_info(
                "[Halo-CampE-UEVR] PALETTEROOT fwd=(%.3f %.3f %.3f) up=(%.3f %.3f %.3f) "
                "pos=(%.3f %.3f %.3f)",
                root_basis.forward.x, root_basis.forward.y, root_basis.forward.z,
                root_basis.up.x, root_basis.up.y, root_basis.up.z,
                palette[0].position.x, palette[0].position.y, palette[0].position.z);
        }
    }

    // The desired pose already pulled back into the mesh's local frame by
    // resolve_world_pullback(), in UE convention. ONE conversion, applied ONCE: the bridge's Y-flip
    // (UE right-handed Y-right -> Blam Y-left) lands on the render side of the basis --
    // node = F * R, so the mesh renders R_cam * F * F * R = R_cam * R, the target. Applying the
    // flip on both sides (a similarity) negates pitch and crosses vertical into roll: "aim down,
    // it aims up", observed verbatim on 2026-08-14. There used to be a four-way knob here for
    // where the flip lands; the working build ran this one, and the knob was a confession of
    // uncertainty dressed as configurability. Deleted -- like 0.5, this converts exactly once.
    // A ROTATION, NOT A REFLECTION. The orientation self-test measured det(visual) = -1.000 on
    // every sample (2026-08-15 15:02): the basis handed to the palette was a mirror image, and
    // had been all day. Converting a rotation between frames is a similarity, C R C^-1 -- the
    // axis flip must land on BOTH sides. One-sided ({f(rx), f(ry), f(rz)}) has det -1, and
    // basis_ok never caught it because it checks column lengths only. A reflection fed where a
    // rotation belongs is exactly "spin the controller like a top and the weapon orbits me":
    // the mirrored basis swings the gun around the player instead of turning it in place. With
    // C = diag(1,-1,1) the two-sided form is {f(rx), -f(ry), f(rz)} -- the middle column negated.
    // A reviewer derived this by hand and I dismissed it on the strength of a build that "worked";
    // that build was a reflection in a different axis that looked half-right from one direction.
    // The self-test below prints the determinant, so this is checked in the log, not argued.
    const Quat& q = g_eff_pose;
    const Vec3 rx = quat_rotate(q, Vec3{1.0f, 0.0f, 0.0f});
    const Vec3 ry = quat_rotate(q, Vec3{0.0f, 1.0f, 0.0f});
    const Vec3 rz = quat_rotate(q, Vec3{0.0f, 0.0f, 1.0f});
    const Mat3 controller{ue_to_blam(rx),
                          ue_to_blam(Vec3{-ry.x, -ry.y, -ry.z}),
                          ue_to_blam(rz)};
    if (!basis_is_rotation(controller)) return false;

    // THE ATTACHMENT IS AUTHORED WITH +Y DOWN THE BARREL, while the controller basis points along
    // its forward. Feeding the controller basis in directly aims the rendered gun along the
    // controller's LEFT axis -- the model points sideways while the shots still go where you aim.
    // Remapping only the visual basis fixes the render without touching the fire path.
    // {-left, forward, up} is a signed permutation with det = +1 (swap two columns: -1; negate
    // one: -1; product +1), so a rotation in stays a rotation out. Verified by the self-test's
    // det print rather than by this comment.
    const Mat3 visual{mul(controller.left, -1.0f), controller.forward, controller.up};
    if (!basis_is_rotation(visual)) return false;

    Mat3 desired_basis = mat_mul(root_basis, visual);

    // ---- PALETTELOCAL (Config.hpp palette_local). Doctrine and the measurements that motivate
    // it are at the Config key. The orientation becomes the STOCK bone rotated by the wrist's
    // delta from a captured rest pose, so no global frame enters and there is nothing for the
    // mesh's own animation to leak through.
    // Statics at function scope so BOTH branches can reach them: the off branch is what re-arms
    // the capture, and a static declared inside the on branch could never be cleared.
    static bool s_pl_have_rest = false;
    static Quat s_pl_rest{0.0f, 0.0f, 0.0f, 1.0f};
    if (g_cfg.palette_local == 0) s_pl_have_rest = false;   // flipping 0 -> 1 re-captures
    if (g_cfg.palette_local != 0) {
        const Quat& hq = g_eff_raw_pose;
        const float hn = std::sqrt(hq.x*hq.x + hq.y*hq.y + hq.z*hq.z + hq.w*hq.w);
        if (hn > 1.0e-6f && std::isfinite(hn)) {
            if (!s_pl_have_rest) {
                s_pl_rest = Quat{hq.x/hn, hq.y/hn, hq.z/hn, hq.w/hn};
                s_pl_have_rest = true;
                API::get()->log_info("[Halo-CampE-UEVR] PALETTELOCAL: rest pose captured, "
                                     "orientation is now a local delta on the stock bone");
            }
            // the wrist's delta from rest, in the CONTROLLER's own frame
            Quat d = quat_mul(quat_conj(s_pl_rest), Quat{hq.x/hn, hq.y/hn, hq.z/hn, hq.w/hn});
            const float dn = std::sqrt(d.x*d.x + d.y*d.y + d.z*d.z + d.w*d.w);
            if (dn > 1.0e-6f && std::isfinite(dn)) {
                d = Quat{d.x/dn, d.y/dn, d.z/dn, d.w/dn};
                // Same two-sided similarity the absolute path uses, so a rotation stays a
                // rotation: the UE->Blam axis flip lands on BOTH sides (middle column negated).
                const Vec3 dx = quat_rotate(d, Vec3{1.0f, 0.0f, 0.0f});
                const Vec3 dy = quat_rotate(d, Vec3{0.0f, 1.0f, 0.0f});
                const Vec3 dz = quat_rotate(d, Vec3{0.0f, 0.0f, 1.0f});
                const Mat3 dloc{ue_to_blam(dx),
                                ue_to_blam(Vec3{-dy.x, -dy.y, -dy.z}),
                                ue_to_blam(dz)};
                if (basis_is_rotation(dloc)) desired_basis = mat_mul(weapon_basis, dloc);
            }
        }
    }

    // ---- ORIENTATION SELF-TEST. Measurement only; changes nothing.
    //
    // The position self-test proved yaw exact and caught pitch, but it pushes only a POSITION
    // through the pipeline. the in-headset report -- "spin the controller like a top and the weapon
    // orbits me instead of turning in place" -- is an ORIENTATION symptom, and it has two
    // candidate causes this line separates: (a) the basis handed to the palette is a REFLECTION
    // (det -1), which basis_ok cannot see because it checks lengths only -- a reviewer's claim,
    // and a reflected basis fed as a rotation produces exactly "kind of rotated, and swings";
    // (b) a proper rotation (det +1) applied in the wrong frame. det(visual) settles (a).
    // Then a synthetic wrist spin: rotate the published pose 90 degrees about the CONTROLLER's
    // own forward and see where the barrel goes; a rotation about the hand keeps the barrel
    // direction almost fixed and rolls the gun -- an orbit swings it. Printed, not acted on.
    if (g_cfg.palette_weapon_log) {
        static uint32_t s_o = 0;
        if ((s_o++ % 240u) == 0u) {
            const NodeMatrix& f = visual.forward; const NodeMatrix& l = visual.left; const NodeMatrix& u = visual.up;
            const float det = f.x * (l.y * u.z - l.z * u.y)
                            - f.y * (l.x * u.z - l.z * u.x)
                            + f.z * (l.x * u.y - l.y * u.x);
            // THE CONVENTION TEST. Every capture with the controller placed ON the rendered gun
            // solved to ~100 degrees of rotation -- a fixed convention error, not a calibration.
            // The stock node's authored basis IS the answer key: it is how the game orients a held
            // weapon for a controller pointing straight down the view. So push a SYNTHETIC
            // controller -- identity pose, pointing dead ahead, level -- through the exact
            // conversion + remap this function applies, and print all three columns beside the
            // stock node's. Where they disagree is the convention error, axis by axis, in one
            // line, no headset. (For the live pose the columns legitimately differ; the synthetic
            // one removes the pose so only the convention remains.)
            const Mat3 synth_ctrl{ue_to_blam(Vec3{1.0f, 0.0f, 0.0f}),
                                  ue_to_blam(Vec3{0.0f, -1.0f, 0.0f}),
                                  ue_to_blam(Vec3{0.0f, 0.0f, 1.0f})};
            const Mat3 synth_vis{mul(synth_ctrl.left, -1.0f), synth_ctrl.forward, synth_ctrl.up};
            const Mat3 synth_out = mat_mul(root_basis, synth_vis);
            const Mat3 stock = basis_of(R[8]);
            API::get()->log_info(
                "[Halo-CampE-UEVR] ORIENTSELFTEST det=%+.3f | SYNTH level ctrl -> "
                "fwd=(%.2f %.2f %.2f) left=(%.2f %.2f %.2f) up=(%.2f %.2f %.2f) | STOCK node8 "
                "fwd=(%.2f %.2f %.2f) left=(%.2f %.2f %.2f) up=(%.2f %.2f %.2f) "
                "[columns should MATCH; a mismatch names the convention error]",
                det,
                synth_out.forward.x, synth_out.forward.y, synth_out.forward.z,
                synth_out.left.x, synth_out.left.y, synth_out.left.z,
                synth_out.up.x, synth_out.up.y, synth_out.up.z,
                stock.forward.x, stock.forward.y, stock.forward.z,
                stock.left.x, stock.left.y, stock.left.z,
                stock.up.x, stock.up.y, stock.up.z);
        }
    }

    // The bone-space grip position, fully resolved by resolve_world_pullback(): world target
    // pulled back through the fresh mesh transform, axes flipped, already in palette units.
    // Scale and units were applied there -- nothing is multiplied here, deliberately, so there
    // is exactly one place unit errors can live.
    const NodeMatrix grip_blam{g_eff_hand.x, g_eff_hand.y, g_eff_hand.z};

    // MESH-ORIGIN RELATIVE, NOT ROOT-NODE RELATIVE. The rendered socket sits at
    // parent_origin + R_cam * node8 (TRACE-NODE8 diff == 0 on every weapon), so palette positions
    // are absolute in mesh space; the root node's own position is just where the root bone was
    // authored. Adding it moved every weapon whose root is not at the origin by exactly the root:
    // 2026-08-17 17:05, Plasma Pistol root = (-0.010 -0.003 0.007) u = (-3.0 -0.9 +2.1) cm, and
    // the socket sat (-3.1 +0.9 +2.2) cm off the rigid hand model in the camera frame (sd 0.8 cm,
    // 173 rows, every reach and roll) while the AR and SMG (root 0) sat at 0.1 cm. 0.5 anchors on
    // palette[0] and never saw it because their weapons' roots are at the origin.
    NodeMatrix desired_pos = xform(root_basis, grip_blam);

    // ---- GRIP OFFSET, in the WEAPON's own frame.
    //
    // The hand position is where the controller is; the weapon's authored origin is not at its
    // grip, so placing node 8 at the hand hangs the gun forward of it. The correction is "back
    // along the barrel", which only means anything in the weapon's frame -- applied in world space
    // it would be right at one wrist angle and wrong at every other.
    //
    // Its OWN keys rather than the rig's off_x/y/z: those are consumed only by the UObjectHook
    // attach path, which was measured inert on this title, and by calibration bookkeeping. Nothing
    // places the rigmode weapon with them, so there is no tuned value here to inherit -- and the
    // frames differ anyway. Centimetres in, Blam units out, so the numbers read like the rest of
    // the config.
    {
        constexpr float kCmPerBlamUnit = 304.8f;
        const NodeMatrix off_cm{g_cfg.palette_weapon_off_x,
                                g_cfg.palette_weapon_off_y,
                                g_cfg.palette_weapon_off_z};
        if (off_cm.x != 0.0f || off_cm.y != 0.0f || off_cm.z != 0.0f) {
            const Mat3 weapon_frame = mat_mul(root_basis, visual);
            desired_pos = add(desired_pos,
                              xform(weapon_frame, mul(off_cm, 1.0f / kCmPerBlamUnit)));
        }
    }

    // NO PALETTE-SPACE CALIBRATION IS APPLIED HERE ANY MORE. The grip offset is now a rigid
    // controller-frame PoseOffset applied to the pose UPSTREAM in the publisher (0.5's design),
    // so the hand this function receives is already the corrected hand and the palette arithmetic
    // never learns a calibration exists. The palette-frame g_fix (basis + pos + lever, and the
    // palettewpnfixframe knob choosing which frame the residual rotated in) is retired: it was
    // the source of the non-rotating pivot, and its two frames were two wrong answers to a
    // question the upstream offset does not ask. Legacy palettewpnfix values in the calibration
    // file are ignored -- one Page Up capture replaces them.

    // ---- WHERE DOES THE GAME PUT IT, AND WHERE ARE WE PUTTING IT?
    //
    // The one comparison that settles scale. The stock node 8 position is the game's own answer for
    // where a held weapon belongs relative to the view root; ours is what the hand offset produces.
    // If ours is systematically NEARER the root, the conversion is wrong and no weapon-frame offset
    // can fix it -- a calibration would absorb the difference into a lever arm that is only correct
    // at the orientation it was solved at, which is the failure being chased.
    if (g_cfg.palette_weapon_log) {
        static uint32_t s_n = 0;
        if ((s_n++ % 120u) == 0u) {
            const NodeMatrix& s = palette[8].position;
            const float stock_len = std::sqrt(s.x*s.x + s.y*s.y + s.z*s.z);
            const float ours_len  = std::sqrt(desired_pos.x*desired_pos.x +
                                              desired_pos.y*desired_pos.y +
                                              desired_pos.z*desired_pos.z);
            API::get()->log_info(
                "[Halo-CampE-UEVR] PALETTEWPN stock8=(%.4f %.4f %.4f) |%.4f|  "
                "ours=(%.4f %.4f %.4f) |%.4f|  ratio=%.3f  hand=(%.4f %.4f %.4f)",
                s.x, s.y, s.z, stock_len,
                desired_pos.x, desired_pos.y, desired_pos.z, ours_len,
                (stock_len > 1.0e-6f) ? (ours_len / stock_len) : 0.0f,
                grip_blam.x, grip_blam.y, grip_blam.z);
        }
    }

    // ---- ORTHOGONALITY, THE THING NEVER MEASURED (2026-09-12, from the headset: "we are fighting a
    // socket that doesnt rotate ... we need to fix the math somewhere").
    //
    // The line below uses mat_transpose(weapon_basis) AS ITS INVERSE. That identity holds only
    // for an ORTHONORMAL matrix. basis_of() normalises each column's LENGTH and never
    // orthogonalises; basis_ok() checks LENGTHS and never checks orthogonality. So if the stock
    // node-8 basis is skewed, delta_basis is wrong by exactly that skew -- and weapon_basis is
    // read from the STOCK ANIMATED pose, which the game rebuilds every frame, so the error
    // changes every frame. That is a per-frame error in the arithmetic, and it vanishes when
    // we do not write.
    //
    // The earlier orientation trace read only the Gram DIAGONAL, which basis_of pins to 1 by
    // construction, so it was a tautology. These are the OFF-DIAGONALS. Zero means the bases are
    // orthonormal, the transpose is a legitimate inverse, and this whole idea is dead for free.
    if (g_cfg.palette_weapon_log) {
        static uint32_t s_g = 0;
        static float s_mx_w = 0.0f, s_mx_r = 0.0f, s_mx_dw = 0.0f;
        static float s_pw[3] = {0.0f, 0.0f, 0.0f};
        static bool  s_ph = false;
        auto dotc = [](const NodeMatrix& a, const NodeMatrix& b) { return a.x*b.x + a.y*b.y + a.z*b.z; };
        auto detm = [](const Mat3& m) {
            return m.forward.x * (m.left.y * m.up.z - m.left.z * m.up.y)
                 - m.forward.y * (m.left.x * m.up.z - m.left.z * m.up.x)
                 + m.forward.z * (m.left.x * m.up.y - m.left.y * m.up.x);
        };
        const float wfl = dotc(weapon_basis.forward, weapon_basis.left);
        const float wfu = dotc(weapon_basis.forward, weapon_basis.up);
        const float wlu = dotc(weapon_basis.left,    weapon_basis.up);
        const float rfl = dotc(root_basis.forward, root_basis.left);
        const float rfu = dotc(root_basis.forward, root_basis.up);
        const float rlu = dotc(root_basis.left,    root_basis.up);
        const float aw = std::fabs(wfl) + std::fabs(wfu) + std::fabs(wlu);
        const float ar = std::fabs(rfl) + std::fabs(rfu) + std::fabs(rlu);
        if (aw > s_mx_w) s_mx_w = aw;
        if (ar > s_mx_r) s_mx_r = ar;
        // the per-frame CHANGE of the skew is what would judder, so measure that too
        if (s_ph) {
            const float dw = std::fabs(wfl - s_pw[0]) + std::fabs(wfu - s_pw[1]) + std::fabs(wlu - s_pw[2]);
            if (dw > s_mx_dw) s_mx_dw = dw;
        }
        s_pw[0] = wfl; s_pw[1] = wfu; s_pw[2] = wlu; s_ph = true;
        if ((s_g++ % 120u) == 0u) {
            // ANSWERED 2026-09-12: off-diag 0.00000 on every sample, det +1.0000, raw column
            // lengths 1.0000. The stock bases ARE orthonormal, so mat_transpose IS a legitimate
            // inverse here and the transpose-as-inverse suspicion is DEAD. Kept because a future
            // weapon or a game patch could break the assumption silently, and it is one line.
            // raw column lengths BEFORE basis_of normalised them: non-unit means the bone
            // carries scale, which a transpose-as-inverse also gets wrong.
            const NodeMatrix& f = R[8].forward; const NodeMatrix& l = R[8].left; const NodeMatrix& u = R[8].up;
            API::get()->log_info(
                "[Halo-CampE-UEVR] GRAM node8 off-diag fl=%+.5f fu=%+.5f lu=%+.5f (sum %.5f, worst %.5f, "
                "worst per-frame change %.5f) det=%+.4f | root off-diag fl=%+.5f fu=%+.5f lu=%+.5f "
                "(worst %.5f) det=%+.4f | node8 RAW col lengths %.4f %.4f %.4f",
                wfl, wfu, wlu, aw, s_mx_w, s_mx_dw, detm(weapon_basis),
                rfl, rfu, rlu, s_mx_r, detm(root_basis),
                std::sqrt(f.x*f.x + f.y*f.y + f.z*f.z),
                std::sqrt(l.x*l.x + l.y*l.y + l.z*l.z),
                std::sqrt(u.x*u.x + u.y*u.y + u.z*u.z));
            s_mx_w = s_mx_r = s_mx_dw = 0.0f;
        }
    }

    // ---- RELATIVE STOCK POSE (2026-09-12). From the headset: "the aim ray is what determines where the
    // characters wepaon and arms are posed", and "left hand writes are pervect, it only has to do
    // with the right hand".
    //
    // Node 8 is exact by construction: written_8 = delta x stock_8 = desired, because the delta is
    // built as desired x transpose(stock_8). Every OTHER node we touch -- 7, 22, and the arms --
    // lands on desired x transpose(stock_8) x stock_k, which CARRIES the game's own relative pose
    // between node k and node 8. The game rebuilds that relation every frame from the aim ray,
    // and only the right hand moves the aim ray. So if that relation moves, those nodes move for
    // reasons that have nothing to do with our math, the weapon's drawn socket rides one of them,
    // and the left hand is immune because it never touches the aim ray.
    if (g_cfg.palette_weapon_log) {
        static uint32_t s_rs = 0;
        static float s_p7 = -1.0f, s_p22 = -1.0f;
        static float s_mx7 = 0.0f, s_mx22 = 0.0f, s_mxd7 = 0.0f, s_mxd22 = 0.0f;
        auto rel_deg = [&](int k) {
            const Mat3 bk = basis_of(R[k]);
            // angle of transpose(stock_8) x stock_k, via the trace of the relative rotation
            const Mat3 wt = mat_transpose(weapon_basis);
            const Mat3 rel = mat_mul(wt, bk);
            float tr3 = rel.forward.x + rel.left.y + rel.up.z;
            float ca = (tr3 - 1.0f) * 0.5f;
            if (ca > 1.0f) ca = 1.0f; if (ca < -1.0f) ca = -1.0f;
            return std::acos(ca) * RAD2DEG;
        };
        const float r7 = rel_deg(7), r22 = rel_deg(22);
        if (r7 > s_mx7) s_mx7 = r7;
        if (r22 > s_mx22) s_mx22 = r22;
        if (s_p7 >= 0.0f) {
            const float d7 = std::fabs(r7 - s_p7), d22 = std::fabs(r22 - s_p22);
            if (d7 > s_mxd7) s_mxd7 = d7;
            if (d22 > s_mxd22) s_mxd22 = d22;
        }
        s_p7 = r7; s_p22 = r22;
        if ((s_rs++ % 120u) == 0u) {
            API::get()->log_info(
                "[Halo-CampE-UEVR] RELSTOCK transpose(stock8)xstock7 = %.4f deg (worst %.4f, worst "
                "per-frame change %.4f) | xstock22 = %.4f deg (worst %.4f, worst per-frame change "
                "%.4f)  [node 8 is exact by construction; these relations ride into nodes 7, 22 and "
                "the arms, and the GAME rebuilds them from the aim ray every frame]",
                r7, s_mx7, s_mxd7, r22, s_mx22, s_mxd22);
            s_mx7 = s_mx22 = s_mxd7 = s_mxd22 = 0.0f;
        }
    }

    // ---- INTRA-FRAME POSE SPREAD (2026-09-12). The GRAM line revealed that this function runs
    // ~267 times a second while the game renders at 43 Hz, so about SIX writes per rendered
    // frame: the precompose stages two banks, the generic path writes the live palette plus two
    // banks, and the render refresh writes again. resolve_world_pullback() ran ~2.75 times per
    // hook call by its own counter, and EACH resolve re-reads the published hand pose fresh.
    //
    // If the publisher lands a new pose between two of those writes, the live palette and the
    // capture banks end up holding DIFFERENT hands within one frame. The engine blends the banks
    // (per-node slerp), so the drawn weapon would sit somewhere between two hand poses, with a
    // weight that changes frame to frame. That is a discrepancy between the drawn rotation and
    // the aim path, rebuilt every frame, which is exactly the shape of the complaint.
    //
    // The earlier two-writer fix (palettecam=7) synchronised the CAMERA across writers. It never
    // synchronised the HAND. This measures whether it needed to.
    if (g_cfg.palette_weapon_log) {
        static Quat s_fp{0.0f, 0.0f, 0.0f, 1.0f};
        static long long s_fms = 0;
        static uint32_t s_in = 0, s_diff = 0, s_rep = 0;
        static float s_mx = 0.0f;
        static double s_sum = 0.0;
        const long long nw = (long long)std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (s_fms != 0 && (nw - s_fms) < 6000) {          // same rendered frame, 6 ms window
            float d = s_fp.x * g_eff_pose.x + s_fp.y * g_eff_pose.y
                    + s_fp.z * g_eff_pose.z + s_fp.w * g_eff_pose.w;
            d = std::fabs(d); if (d > 1.0f) d = 1.0f;
            const float deg = 2.0f * std::acos(d) * RAD2DEG;
            ++s_in;
            if (deg > 0.0005f) { ++s_diff; s_sum += (double)deg; if (deg > s_mx) s_mx = deg; }
        } else {
            s_fp = g_eff_pose;                            // first write of a new frame
        }
        s_fms = nw;
        if ((s_rep++ % 240u) == 0u) {
            API::get()->log_info(
                "[Halo-CampE-UEVR] INTRAFRAME same-frame writes=%u differing=%u (%.1f%%) "
                "mean %.4f deg worst %.4f deg  [all writes in one frame MUST use one hand; "
                "a spread here means the banks hold different hands and the blend draws between them]",
                s_in, s_diff, s_in ? (100.0 * (double)s_diff / (double)s_in) : 0.0,
                s_diff ? (s_sum / (double)s_diff) : 0.0, s_mx);
            s_in = s_diff = 0; s_mx = 0.0f; s_sum = 0.0;
        }
    }

    const Mat3 delta_basis = mat_mul(desired_basis, mat_transpose(weapon_basis));
    if (!basis_ok(delta_basis)) return false;
    const NodeMatrix delta_pos = sub(desired_pos, xform(delta_basis, R[8].position));
    if (g_cfg.term_log != 0) {
        branchlog::Row br{};
        br.t_ms = branchlog::now_ms();
        br.buf = (unsigned long long)(uintptr_t)palette;
        auto len = [](const NodeMatrix& v) { return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z); };
        auto dot = [](const NodeMatrix& a, const NodeMatrix& b) { return a.x*b.x + a.y*b.y + a.z*b.z; };
        const PaletteNode& r0 = R[0];
        const PaletteNode& r8 = R[8];
        br.root_len[0] = len(r0.forward); br.root_len[1] = len(r0.left); br.root_len[2] = len(r0.up);
        br.root_dot[0] = dot(r0.forward, r0.left); br.root_dot[1] = dot(r0.forward, r0.up); br.root_dot[2] = dot(r0.left, r0.up);
        br.root_scale = r0.scale;
        br.root_pos[0] = r0.position.x; br.root_pos[1] = r0.position.y; br.root_pos[2] = r0.position.z;
        br.w8_len[0] = len(r8.forward); br.w8_len[1] = len(r8.left); br.w8_len[2] = len(r8.up);
        br.w8_dot[0] = dot(r8.forward, r8.left); br.w8_dot[1] = dot(r8.forward, r8.up); br.w8_dot[2] = dot(r8.left, r8.up);
        br.w8_scale = r8.scale;
        br.w8_pos[0] = r8.position.x; br.w8_pos[1] = r8.position.y; br.w8_pos[2] = r8.position.z;
        // THE RESIDUAL: what the maths actually lands on node 8 versus what it was told to.
        const Mat3 landed = mat_mul(delta_basis, weapon_basis);
        const float tr3 = dot(landed.forward, desired_basis.forward)
                        + dot(landed.left, desired_basis.left)
                        + dot(landed.up, desired_basis.up);
        float ca = (tr3 - 1.0f) * 0.5f;
        if (ca > 1.0f) ca = 1.0f; if (ca < -1.0f) ca = -1.0f;
        br.res_deg = std::acos(ca) * 57.29578f;
        const NodeMatrix lp = add(delta_pos, xform(delta_basis, r8.position));
        const NodeMatrix pe = sub(lp, desired_pos);
        br.res_cm = std::sqrt(pe.x*pe.x + pe.y*pe.y + pe.z*pe.z) * 304.8f;
        // WHICH POSE IS THE REFERENCE IN? Distance from our own last write tells us: ~0 means
        // the palette still holds OUR bytes, ~39 cm means the game re-posed it to stock. And the
        // node-to-node distances say whether the weapon is rigid across that flip: constant
        // means every node moved together (harmless), jumping means node 8 alternates ALONE,
        // which would drag every other node through the delta.
        {
            const float ox = r8.position.x - g_dbg_node8_x.load(std::memory_order_relaxed);
            const float oy = r8.position.y - g_dbg_node8_y.load(std::memory_order_relaxed);
            const float oz = r8.position.z - g_dbg_node8_z.load(std::memory_order_relaxed);
            br.ours_cm = std::sqrt(ox*ox + oy*oy + oz*oz) * 304.8f;
            auto nd = [&](int k) {
                const NodeMatrix& q = palette[k].position;
                const float dx = r8.position.x - q.x, dy = r8.position.y - q.y, dz = r8.position.z - q.z;
                return std::sqrt(dx*dx + dy*dy + dz*dz) * 304.8f;
            };
            {   // the sway pair
                const Mat3 b8 = basis_of(R[8]);
                const Mat3 b19 = basis_of(R[19]);
                const float tr3 = (b8.forward.x*b19.forward.x + b8.forward.y*b19.forward.y + b8.forward.z*b19.forward.z)
                                + (b8.left.x*b19.left.x + b8.left.y*b19.left.y + b8.left.z*b19.left.z)
                                + (b8.up.x*b19.up.x + b8.up.y*b19.up.y + b8.up.z*b19.up.z);
                float ca = (tr3 - 1.0f) * 0.5f;
                if (ca > 1.0f) ca = 1.0f;
                if (ca < -1.0f) ca = -1.0f;
                br.sway_deg = std::acos(ca) * 57.29578f;
                const float sx = R[8].position.x - R[19].position.x;
                const float sy = R[8].position.y - R[19].position.y;
                const float sz = R[8].position.z - R[19].position.z;
                br.sway_cm = std::sqrt(sx*sx + sy*sy + sz*sz) * 304.8f;
            }
            // THE RACE, measured: how far the shared shadow (what the old code would have
            // consumed) is from THIS thread's own resolve. Zero everywhere means the two
            // contexts never disagreed and the race was harmless; non-zero values that grow
            // while the wrist turns mean the old shared globals were feeding the branch a pose
            // from the other thread's camera. Nodes 20 and 40 are gone: the audit established
            // the branch never transforms them, so their distance to node 8 had to move as the
            // hand moved and measured nothing.
            {
                const float hx = (g_eff_hand.x - g_shr_hand.x);
                const float hy = (g_eff_hand.y - g_shr_hand.y);
                const float hz = (g_eff_hand.z - g_shr_hand.z);
                br.n8_n20_cm = std::sqrt(hx*hx + hy*hy + hz*hz) * 304.8f;
                float qd = g_eff_pose.x*g_shr_pose.x + g_eff_pose.y*g_shr_pose.y
                         + g_eff_pose.z*g_shr_pose.z + g_eff_pose.w*g_shr_pose.w;
                if (qd < 0.0f) qd = -qd; if (qd > 1.0f) qd = 1.0f;
                br.n8_n40_cm = 2.0f * std::acos(qd) * 57.29578f;
            }
        }
        br.desired_pos[0] = desired_pos.x; br.desired_pos[1] = desired_pos.y; br.desired_pos[2] = desired_pos.z;
        br.delta_pos[0] = delta_pos.x; br.delta_pos[1] = delta_pos.y; br.delta_pos[2] = delta_pos.z;
        branchlog::put(br);
    }
    if (out_delta_basis != nullptr) *out_delta_basis = delta_basis;
    if (out_delta_pos   != nullptr) *out_delta_pos   = delta_pos;

    // ALL OR NOTHING (found by audit, 2026-09-12). This loop used to validate and commit node by
    // node over {7, 8, 22}, and every rejection path below -- a degenerate basis, a non-finite
    // position, the arm's-reach clamp -- returns false. A rejection at node 8 or 22 therefore
    // returned having ALREADY committed node 7, leaving the rig split: node 7 on the hand and
    // nodes 8 and 22 at stock. One frame of that is a visible tear of the whole weapon, and
    // nothing in the logs distinguished it from a clean build. So: stage all three, validate all
    // three, and commit only if every one passed.
    struct Staged { NodeMatrix f, l, u, p; };
    Staged staged[3];
    int staged_n = 0;
    for (const int node : {7, 8, 22}) {
        const PaletteNode& s = R[node];          // STOCK source: never the buffer being written
        NodeMatrix f = xform(delta_basis, s.forward);
        NodeMatrix l = xform(delta_basis, s.left);
        NodeMatrix u = xform(delta_basis, s.up);
        if (!norm_node(&f) || !norm_node(&l) || !norm_node(&u)) return false;
        const NodeMatrix p = add(delta_pos, xform(delta_basis, s.position));
        // REFUSE ANYTHING ABSURD, for the same reason the poke does: a first-person node far
        // outside arm's reach is what the sim faulted on, and one bad frame is enough. The bound
        // is ARM'S REACH -- 0.6 palette units is ~1.8 m from the view root, which no held weapon
        // node ever legitimately exceeds -- not the old 8.0 (24 m), which only caught NaN-scale
        // garbage. The 2026-08-15 crash was a 2.4 m "hand" from a tracking dropout (controller
        // reads 0,0,0 asleep; the offset becomes the negated head position) sailing under 8.0,
        // being frozen by the calibration key, and faulting HaloCampaignEvolved.exe itself.
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
        if (std::fabs(p.x) > 0.6f || std::fabs(p.y) > 0.6f || std::fabs(p.z) > 0.6f) return false;
        staged[staged_n++] = Staged{f, l, u, p};
    }
    // Every node validated: commit them together.
    {
        int k = 0;
        for (const int node : {7, 8, 22}) {
            PaletteNode& m = palette[node];
            m.forward = staged[k].f; m.left = staged[k].l; m.up = staged[k].u;
            m.position = staged[k].p;
            if (node == 8) {
                g_dbg_node8_x.store(staged[k].p.x, std::memory_order_relaxed);
                g_dbg_node8_y.store(staged[k].p.y, std::memory_order_relaxed);
                g_dbg_node8_z.store(staged[k].p.z, std::memory_order_relaxed);
                // READBACK: the address we just wrote, and exactly what we put there. Only when
                // this call is writing the LIVE palette (ref == nullptr means the caller handed
                // us the buffer that is also its own reference, i.e. the live one), because the
                // banks are copies the game has no reason to touch.
                if (ref != nullptr) {
                    // A bank (or precompose) buffer. Record it under its own address so the probe
                    // can match the socket against whichever buffer the renderer actually reads.
                    const unsigned slot = g_bank8_n.fetch_add(1u, std::memory_order_relaxed) & 1u;
                    BankShot& bs = g_bank8[slot];
                    bs.seq.fetch_add(1u, std::memory_order_acq_rel);
                    bs.addr.store((uintptr_t)&m, std::memory_order_relaxed);
                    bs.ms.store((long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count(),
                        std::memory_order_relaxed);
                    bs.fx.store(staged[k].f.x, std::memory_order_relaxed);
                    bs.fy.store(staged[k].f.y, std::memory_order_relaxed);
                    bs.fz.store(staged[k].f.z, std::memory_order_relaxed);
                    bs.lx.store(staged[k].l.x, std::memory_order_relaxed);
                    bs.ly.store(staged[k].l.y, std::memory_order_relaxed);
                    bs.lz.store(staged[k].l.z, std::memory_order_relaxed);
                    bs.ux.store(staged[k].u.x, std::memory_order_relaxed);
                    bs.uy.store(staged[k].u.y, std::memory_order_relaxed);
                    bs.uz.store(staged[k].u.z, std::memory_order_relaxed);
                    bs.seq.fetch_add(1u, std::memory_order_acq_rel);
                }
                if (ref == nullptr) {
                    g_n8_seq.fetch_add(1u, std::memory_order_acq_rel);
                    g_n8_addr.store((uintptr_t)&m, std::memory_order_relaxed);
                    g_n8_ms.store((long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count(),
                        std::memory_order_relaxed);
                    g_n8_w_fx.store(staged[k].f.x, std::memory_order_relaxed);
                    g_n8_w_fy.store(staged[k].f.y, std::memory_order_relaxed);
                    g_n8_w_fz.store(staged[k].f.z, std::memory_order_relaxed);
                    g_n8_w_lx.store(staged[k].l.x, std::memory_order_relaxed);
                    g_n8_w_ly.store(staged[k].l.y, std::memory_order_relaxed);
                    g_n8_w_lz.store(staged[k].l.z, std::memory_order_relaxed);
                    g_n8_w_ux.store(staged[k].u.x, std::memory_order_relaxed);
                    g_n8_w_uy.store(staged[k].u.y, std::memory_order_relaxed);
                    g_n8_w_uz.store(staged[k].u.z, std::memory_order_relaxed);
                    g_n8_w_px.store(staged[k].p.x, std::memory_order_relaxed);
                    g_n8_w_py.store(staged[k].p.y, std::memory_order_relaxed);
                    g_n8_w_pz.store(staged[k].p.z, std::memory_order_relaxed);
                    g_n8_seq.fetch_add(1u, std::memory_order_acq_rel);
                }
            }
            ++k;
        }
    }
    return true;
}


// THE WORLD-SPACE PULLBACK. Resolve the published world target into THIS build's bone frame,
// with the mesh's world transform rebuilt FRESH from the aim rotator read right now.
//
//   comp_rot = cam * M                             cam: read here, sim-side, zero staleness
//   mesh-origin-from-parent = cam-rotated v0       M, v0: measured constants (Plugin.cpp)
//   bone_pos = conj(comp_rot) * (hand_cm - cam*v0) / 304.8, UE axes -> Blam axes
//   bone_rot = conj(comp_rot) * q_gun_world
//
// The published half contains no camera term at all, so the only staleness left in the whole
// chain is the HAND's own tick latency -- the same latency every UEVR mod's game-thread sampling
// has, and the part that was never the disease. The camera term, which was, is exact by
// construction. Returns false when inputs are missing: the stock pose renders, wrong-but-stable.
// ---- TERMLOG (doctrine at Config::term_log). One row per build, every stage of the chain.
namespace termlog {
struct Row {
    double t_ms; int ctx;
    float ctl_p, ctl_y;           // ControlRotation as read (post torn-read gate)
    float cam_p, cam_y;           // the camera actually used, after the mode swap
    float mq[4]; int mq_ok;       // the mesh quaternion when a mode reads one
    float M[4];                   // the tick-measured mesh constant
    float comp[4];                // the composition the bones are divided by
    float view_yaw;               // the rendered view yaw (the room->world lift)
    float bob[3];
    float hand_room[3];           // published grip, cm, room frame
    float hand_cm[3];             // after the lift and the bob subtraction
    float aim[4];                 // published aim quaternion
    float pose_lift[4];           // after the room->world lift
    float pose_barrel[4];         // after the barrel lock
    float pose_trim[4];           // after the roll trim and the weapon rotation
    float rel_cm[3];              // world hand pulled into the mesh frame
    float eff_hand[3];            // FINAL: palette units, Blam axes
    float eff_pose[4];            // FINAL: mesh-frame pose
    float tru_g[3];               // controller grip as the runtime returned it
    float tru_a[4];               // controller aim as the runtime returned it
    // THE CANCELLATION TEST (2026-09-12). We write bones as conj(cam*M) (x) world_thing and the
    // renderer draws them at the FP mesh's own world rotation Q. The cancellation is exact only
    // when Q == cam*M; the residual conj(Q) (x) (cam*M) rotates the whole drawn rig, both hands,
    // and only while the camera moves. mq_* is zero outside modes 8 and 9, so Q was never once
    // logged in the mode we actually play. mrot_* is Q, published every tick, unconditionally.
    float mrot[4];                // Q_tick, the mesh world rotation the game reported this tick
    float qr[4]; int qr_ok;       // Q_RENDER, the mesh rotation read live AT the render callback
    float brk;                    // deg the camera moved BETWEEN the two reads that measure M
    float mage;                   // ms since the live M was accepted (0 = measured this frame)
};
constexpr int kCap = 40000;
Row g_rows[kCap];
std::atomic<int> g_n{0};
bool g_was_on = false;
int g_seq = 0;
double now_ms() {
    return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
void put(const Row& r) {
    const int i = g_n.fetch_add(1, std::memory_order_relaxed);
    g_rows[i % kCap] = r;
}
void flush() {
    const int n = g_n.exchange(0, std::memory_order_relaxed);
    if (n <= 0 || g_cfg_path[0] == '\0') return;
    char path[MAX_PATH]; strncpy_s(path, sizeof(path), g_cfg_path, _TRUNCATE);
    char* slash = strrchr(path, '\\');
    if (slash == nullptr) return;
    char leaf[64]; sprintf_s(leaf, "halo_vr_term_%03d.csv", g_seq++);
    slash[1] = '\0'; strncat_s(path, sizeof(path), leaf, _TRUNCATE);
    FILE* f = nullptr;
    if (fopen_s(&f, path, "wb") != 0 || f == nullptr) return;
    fprintf(f, "t_ms,ctx,ctl_p,ctl_y,cam_p,cam_y,mq_x,mq_y,mq_z,mq_w,mq_ok,"
               "M_x,M_y,M_z,M_w,comp_x,comp_y,comp_z,comp_w,view_yaw,"
               "bob_x,bob_y,bob_z,hr_x,hr_y,hr_z,hc_x,hc_y,hc_z,"
               "aim_x,aim_y,aim_z,aim_w,pl_x,pl_y,pl_z,pl_w,"
               "pb_x,pb_y,pb_z,pb_w,pt_x,pt_y,pt_z,pt_w,"
               "rel_x,rel_y,rel_z,eh_x,eh_y,eh_z,ep_x,ep_y,ep_z,ep_w,tg_x,tg_y,tg_z,ta_x,ta_y,ta_z,ta_w,mr_x,mr_y,mr_z,mr_w,brk,mage,qr_x,qr_y,qr_z,qr_w,qr_ok\r\n");
    const int m = n < kCap ? n : kCap;
    const int start = (n > kCap) ? (n % kCap) : 0;
    for (int i = 0; i < m; ++i) {
        const Row& r = g_rows[(start + i) % kCap];
        fprintf(f, "%.4f,%d,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%d,"
                   "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,"
                   "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                   "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                   "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                   "%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%d\r\n",
                r.t_ms, r.ctx, r.ctl_p, r.ctl_y, r.cam_p, r.cam_y,
                r.mq[0], r.mq[1], r.mq[2], r.mq[3], r.mq_ok,
                r.M[0], r.M[1], r.M[2], r.M[3],
                r.comp[0], r.comp[1], r.comp[2], r.comp[3], r.view_yaw,
                r.bob[0], r.bob[1], r.bob[2],
                r.hand_room[0], r.hand_room[1], r.hand_room[2],
                r.hand_cm[0], r.hand_cm[1], r.hand_cm[2],
                r.aim[0], r.aim[1], r.aim[2], r.aim[3],
                r.pose_lift[0], r.pose_lift[1], r.pose_lift[2], r.pose_lift[3],
                r.pose_barrel[0], r.pose_barrel[1], r.pose_barrel[2], r.pose_barrel[3],
                r.pose_trim[0], r.pose_trim[1], r.pose_trim[2], r.pose_trim[3],
                r.rel_cm[0], r.rel_cm[1], r.rel_cm[2],
                r.eff_hand[0], r.eff_hand[1], r.eff_hand[2],
                r.eff_pose[0], r.eff_pose[1], r.eff_pose[2], r.eff_pose[3],
                r.tru_g[0], r.tru_g[1], r.tru_g[2],
                r.tru_a[0], r.tru_a[1], r.tru_a[2], r.tru_a[3],
                r.mrot[0], r.mrot[1], r.mrot[2], r.mrot[3], r.brk, r.mage,
                r.qr[0], r.qr[1], r.qr[2], r.qr[3], r.qr_ok);
    }
    fclose(f);
    API::get()->log_info("[Halo-CampE-UEVR] TERMLOG: %d rows -> %hs", m, leaf);
}
} // namespace termlog
void blam_palette_term_tick() {
    if (g_cfg.term_log != 0) { termlog::g_was_on = true; return; }
    if (termlog::g_was_on) {
        termlog::g_was_on = false;
        termlog::flush();
        branchlog::flush();
    }
}
// WHY-NOT LEDGER (2026-09-12). PALSLOT's first run said two things that no audit had caught.
//   gate6=0 on EVERY line: the gate-6 build-time landing, the mechanism credited with curing
//     the stock-landing window, has never executed once. g_pre6.valid is false, and eight
//     different early returns in palette_precompose could be the reason.
//   generic went 320,320,320,320,320,203,0,11 per 320 calls: the write stops happening
//     ENTIRELY for stretches, which is the weapon-swap alternation between stock and our pose,
//     and a low duty cycle of the same thing would be the judder.
// Both land on "which early return fired", and guessing between eight of them is exactly the
// habit that has cost this hunt a day. So every bail gets a counter and prints itself.
namespace whynot {
std::atomic<unsigned> pre_nopal{0}, pre_tag{0}, pre_seq{0}, pre_banks{0}, pre_stale{0},
                      pre_pull{0}, pre_branch{0}, pre_ok{0};
std::atomic<unsigned> rwp_nopose{0}, rwp_nomesh{0}, rwp_noctl{0}, rwp_norm{0}, rwp_ok{0};
std::atomic<unsigned> aap_pull{0}, aap_nopal{0};
// REVCLAMP: frames seen, frames whose backward component was impossible, and the total degrees
// removed (x100, integer, so the meter needs no float atomics).
std::atomic<unsigned> rev_seen{0}, rev_hit{0}, rev_deg{0};
// TREMOR: frames filtered and the total degrees the notch removed (x1000, integer).
std::atomic<unsigned> trem_seen{0}, trem_deg{0};
// MODE 12: did the divisor come from the live render-thread mesh read, or did we fall back?
std::atomic<unsigned> m12_live{0}, m12_fallback{0};
// PALETTELATCH: writes that reused the frame's latched hand, versus writes that opened a new one.
std::atomic<unsigned> latch_hit{0}, latch_miss{0};
// COMPLATCH: writes that reused the frame's latched DIVISOR, versus writes that opened a new one.
std::atomic<unsigned> clatch_hit{0}, clatch_miss{0};
void report() {
    API::get()->log_info(
        "[Halo-CampE-UEVR] WHYNOT precompose[nopal=%u tag=%u seq=%u banks=%u stale=%u pull=%u branch=%u OK=%u] "
        "pullback[nopose=%u nomesh=%u noctl=%u norm=%u OK=%u] after_pose[pullfail=%u nopal=%u] "
        "revclamp[seen=%u hit=%u removed=%.2f deg] tremor[seen=%u removed=%.3f deg] mode12[live=%u fallback=%u] latch[reuse=%u fresh=%u] clatch[reuse=%u fresh=%u]",
        pre_nopal.exchange(0), pre_tag.exchange(0), pre_seq.exchange(0), pre_banks.exchange(0),
        pre_stale.exchange(0), pre_pull.exchange(0), pre_branch.exchange(0), pre_ok.exchange(0),
        rwp_nopose.exchange(0), rwp_nomesh.exchange(0), rwp_noctl.exchange(0), rwp_norm.exchange(0),
        rwp_ok.exchange(0), aap_pull.exchange(0), aap_nopal.exchange(0),
        rev_seen.exchange(0), rev_hit.exchange(0), (double)rev_deg.exchange(0) * 0.01,
        trem_seen.exchange(0), (double)trem_deg.exchange(0) * 0.001,
        m12_live.exchange(0), m12_fallback.exchange(0), latch_hit.exchange(0), latch_miss.exchange(0),
        clatch_hit.exchange(0), clatch_miss.exchange(0));
}
} // namespace whynot

bool resolve_world_pullback(bool render_ctx = false) {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    termlog::Row tr{};
    const bool tlog = g_cfg.term_log != 0;
    tr.ctx = render_ctx ? 1 : 0;
    if (!g_p_valid.load(std::memory_order_acquire)) { whynot::rwp_nopose.fetch_add(1, std::memory_order_relaxed); return false; }
    if (!g_mesh_const_valid.load(std::memory_order_acquire)) { whynot::rwp_nomesh.fetch_add(1, std::memory_order_relaxed); return false; }
    g_p_con_ms.store((long long)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count(), std::memory_order_relaxed);

    double cam_pitch = 0.0, cam_yaw = 0.0;
    if (!read_control_rotation(&cam_pitch, &cam_yaw, nullptr)) { whynot::rwp_noctl.fetch_add(1, std::memory_order_relaxed); return false; }
    // ---- MODE 13 (2026-09-12). DIVIDE BY THE CAMERA THE GAME ACTUALLY USES.
    // Static RE: sim+0x46A2E0 composes every first-person bone with a root built from the
    // per-player observer camera (rec = *(TLS+0x4E8) + p*0x410), and that camera is EASED toward
    // the control angles by sim+0x235840 rather than assigned. OBSCAM measured the lag: the
    // camera-minus-aim gap moves against every aim step, yaw corr -0.27..-0.86 with slope up to
    // -0.752, pitch corr -0.12..-0.55, in all five windows. SAMEINST measured the drawn weapon
    // picking up ~0.7 of each aim step. Every mode until now divided by the aim (ControlRotation
    // or a mesh read that follows it). This one divides by aim + the live eased-camera gap, so the
    // cancellation matches what the game composes on top of our bone.
    if (g_cfg.palette_cam == 13) {
        const long long o_now = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const long long o_ms = g_obs_ms.load(std::memory_order_relaxed);
        if (o_ms != 0 && o_now - o_ms < 100) {
            cam_yaw   += (double)g_obs_gap_y.load(std::memory_order_relaxed);
            cam_pitch += (double)g_obs_gap_p.load(std::memory_order_relaxed);
        }
    }
    // ---- THE TORN-READ GATE (filmed 2026-09-11). With every downstream stage disabled the
    // 60 fps film still showed the gun jumping 10-28 px for single frames while the hand stepped
    // under 2 -- and with THIS hook's write disabled the gun went smooth. The hook reads
    // ControlRotation from the sim thread while the game thread writes it; a read that lands
    // mid-write yields a garbage camera for exactly one build, only while the value is changing,
    // which is only while the wrist rotates. So the read is VERIFIED: read again, and if the two
    // disagree beyond rounding, a third read arbitrates (two reads agreeing microseconds apart
    // cannot both straddle a write). A verified read that still leaps implausibly against the
    // last accepted value (>60 deg in one build, beyond any snap turn) is discarded for the
    // last good camera, one build of hold beating one frame of garbage.
    {
        double p2 = 0.0, y2 = 0.0;
        if (read_control_rotation(&p2, &y2, nullptr)) {
            double dy = cam_yaw - y2; while (dy > 180.0) dy -= 360.0; while (dy < -180.0) dy += 360.0;
            if (std::fabs(dy) > 0.05 || std::fabs(cam_pitch - p2) > 0.05) {
                double p3 = 0.0, y3 = 0.0;
                if (read_control_rotation(&p3, &y3, nullptr)) {
                    double d23 = y2 - y3; while (d23 > 180.0) d23 -= 360.0; while (d23 < -180.0) d23 += 360.0;
                    if (std::fabs(d23) <= 0.05 && std::fabs(p2 - p3) <= 0.05) { cam_yaw = y2; cam_pitch = p2; }
                    else { cam_yaw = y3; cam_pitch = p3; }
                }
            }
        }
        static double s_lc_y = 0.0, s_lc_p = 0.0; static bool s_lc_have = false;
        double dl = cam_yaw - s_lc_y; while (dl > 180.0) dl -= 360.0; while (dl < -180.0) dl += 360.0;
        if (s_lc_have && (std::fabs(dl) > 60.0 || std::fabs(cam_pitch - s_lc_p) > 60.0)) {
            cam_yaw = s_lc_y; cam_pitch = s_lc_p;   // one build of hold; the next build re-accepts
            s_lc_have = false;                       // and the gate cannot latch a bad hold twice
        } else { s_lc_y = cam_yaw; s_lc_p = cam_pitch; s_lc_have = true; }
    }
    tr.ctl_p = (float)cam_pitch; tr.ctl_y = (float)cam_yaw;
    // RENDER CONTEXT (palrender): the embed camera is recovered from the MESH TRANSFORM ITSELF,
    // cam = comp_rot * conj(M), read this same instant. The FPMESH meter (2026-09-11) showed the
    // mesh holding a camera up to ~3.5 deg OLDER than ControlRotation at the render callback's
    // moment -- banks counter-rotated for a camera the mesh has not adopted yet are exactly the
    // residual wrist judder. Sampling the mesh removes the mismatch by construction; a missing
    // component falls back to ControlRotation, which is the old behaviour.
    // palettecam=9 (doctrine at the Config key): the mesh's world rotation, carried WHOLE.
    Quat mesh_q{0.0f, 0.0f, 0.0f, 1.0f};
    bool mesh_q_ok = false;
    if (render_ctx && g_cfg.palette_cam == 9) {
        auto* mc9 = rig_tracked_component();
        Vec3 mr9{};
        if (mc9 != nullptr && call_ret_vec3(mc9, L"K2_GetComponentRotation", &mr9)) {
            mesh_q = rotator_to_quat(mr9.x, mr9.y, mr9.z);
            mesh_q_ok = std::isfinite(mesh_q.x) && std::isfinite(mesh_q.w);
        }
    } else if (!render_ctx && g_cfg.palette_cam == 9) {
        const long long tk9 = g_tick_cam_ms.load(std::memory_order_acquire);
        const long long now9 = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (tk9 != 0 && now9 - tk9 < 100) {
            mesh_q = Quat{g_tick_mrot_x.load(std::memory_order_relaxed), g_tick_mrot_y.load(std::memory_order_relaxed),
                          g_tick_mrot_z.load(std::memory_order_relaxed), g_tick_mrot_w.load(std::memory_order_relaxed)};
            const float n9 = std::sqrt(mesh_q.x*mesh_q.x + mesh_q.y*mesh_q.y + mesh_q.z*mesh_q.z + mesh_q.w*mesh_q.w);
            mesh_q_ok = n9 > 0.9f && n9 < 1.1f;
        }
    }
    // ---- MODE 11 (2026-09-12). Divide by Q_tick ITSELF, seqlocked, in BOTH contexts.
    // Measured this session: brk = 0.0000 on all 7314 frames and mage = 3 ms, so M is neither
    // latency-contaminated nor stale, yet E = conj(Q_tick) x comp_rot still runs median 1.42 deg
    // and p95 6.00 deg per frame at 80-200 deg/s of hand rate. So the inputs to cam*M are clean
    // and the PRODUCT is still not the mesh. Two ways that happens, and this mode closes both:
    //   * a torn read of the (cam_tick, M) pair, which were three unsynchronised relaxed stores
    //     until the seqlock added alongside this. cam from tick N times M from tick N-1 misses by
    //     one tick of camera motion, which is exactly the observed size;
    //   * mode 9, the only other mode that used the mesh rotation, read the TICK value when
    //     building and did a LIVE render-thread component read when refreshing, so its two
    //     writers divided by two different quaternions. Mode 11 gives both writers the identical
    //     value, so they cannot disagree.
    // Not the same as mode 6. Mode 6 reaches Q_tick as cam_tick x M, a product of two separately
    // read atomics, through a pitch/yaw rotator. Mode 11 reads the one quaternion and uses it.
    if (g_cfg.palette_cam == 11) {
        for (int s_try = 0; s_try < 4; ++s_try) {
            const unsigned s0 = g_tick_seq.load(std::memory_order_acquire);
            const long long tk11 = g_tick_cam_ms.load(std::memory_order_relaxed);
            const Quat q11{g_tick_mrot_x.load(std::memory_order_relaxed), g_tick_mrot_y.load(std::memory_order_relaxed),
                           g_tick_mrot_z.load(std::memory_order_relaxed), g_tick_mrot_w.load(std::memory_order_relaxed)};
            if ((s0 & 1u) || g_tick_seq.load(std::memory_order_acquire) != s0) continue;
            const long long now11 = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            const float n11 = std::sqrt(q11.x*q11.x + q11.y*q11.y + q11.z*q11.z + q11.w*q11.w);
            if (tk11 != 0 && now11 - tk11 < 100 && n11 > 0.9f && n11 < 1.1f) {
                mesh_q = Quat{q11.x / n11, q11.y / n11, q11.z / n11, q11.w / n11};
                mesh_q_ok = true;
            }
            break;
        }
    }
    // ---- Q_RENDER, A PURE INSTRUMENT (termlog only, never steers placement).
    // THE ONE UNKNOWN LEFT. We divide by a camera sampled at render time, but a component's world
    // transform is game-thread state, so the renderer may well be drawing at the TICK's rotation.
    // If Q_render == Q_tick then E above is the real bodily rotation of the drawn rig and the
    // divisor must be Q_tick. If Q_render instead tracks the render-time camera then E is harmless
    // and the defect is not here at all. That has never been measured, so it gets measured.
    // Also re-checks the 2026-09-11 claim that live mesh reads "step up to 43 deg/frame", which
    // cannot be literally true of the mesh itself or the stock gun riding it would tear apart.
    if (render_ctx && (tlog || g_cfg.palette_cam == 12)) {
        auto* mcr = rig_tracked_component();
        Vec3 mrr{};
        if (mcr != nullptr && call_ret_vec3(mcr, L"K2_GetComponentRotation", &mrr)) {
            const Quat qr = rotator_to_quat(mrr.x, mrr.y, mrr.z);
            if (std::isfinite(qr.x) && std::isfinite(qr.w)) {
                tr.qr[0] = qr.x; tr.qr[1] = qr.y; tr.qr[2] = qr.z; tr.qr[3] = qr.w;
                tr.qr_ok = 1;
                // Publish for MODE 12 so the build writer gets the same quaternion the refresh
                // just read, rather than reconstructing its own. Seqlocked: odd = writing.
                const float qn12 = std::sqrt(qr.x*qr.x + qr.y*qr.y + qr.z*qr.z + qr.w*qr.w);
                if (qn12 > 0.9f && qn12 < 1.1f) {
                    g_qr_seq.fetch_add(1u, std::memory_order_acq_rel);
                    g_qr_x.store(qr.x / qn12, std::memory_order_relaxed);
                    g_qr_y.store(qr.y / qn12, std::memory_order_relaxed);
                    g_qr_z.store(qr.z / qn12, std::memory_order_relaxed);
                    g_qr_w.store(qr.w / qn12, std::memory_order_relaxed);
                    g_qr_ms.store((long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count(), std::memory_order_relaxed);
                    g_qr_seq.fetch_add(1u, std::memory_order_acq_rel);
                }
            }
        }
    }
    // ORDER FIX (2026-09-12, found by audit): this block used to sit BEFORE the Q_RENDER read,
    // so in the render refresh it consumed the PREVIOUS callback's published mesh rotation while
    // the same call was about to read a fresh one -- one frame of mesh turn left on every refresh
    // write. It now runs after the fresh read has been published.
    // ---- MODE 12. Both writers take the ONE live mesh rotation the render thread published.
    // No euler round trip, so roll survives. No per-context difference, so the two writers
    // cannot disagree. Falls back to cam*M only if nothing has been published yet or it went
    // stale, and the fallback is COUNTED so "mode 12 was on" is never assumed.
    if (g_cfg.palette_cam == 12) {
        bool got = false;
        for (int s_try = 0; s_try < 4; ++s_try) {
            const uint32_t s0 = g_qr_seq.load(std::memory_order_acquire);
            const Quat q12{g_qr_x.load(std::memory_order_relaxed), g_qr_y.load(std::memory_order_relaxed),
                           g_qr_z.load(std::memory_order_relaxed), g_qr_w.load(std::memory_order_relaxed)};
            const long long qms = g_qr_ms.load(std::memory_order_relaxed);
            if ((s0 & 1u) || g_qr_seq.load(std::memory_order_acquire) != s0) continue;
            const long long now12 = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            const float n12 = std::sqrt(q12.x*q12.x + q12.y*q12.y + q12.z*q12.z + q12.w*q12.w);
            if (qms != 0 && now12 - qms < 100 && n12 > 0.9f && n12 < 1.1f) {
                mesh_q = Quat{q12.x / n12, q12.y / n12, q12.z / n12, q12.w / n12};
                mesh_q_ok = true; got = true;
            }
            break;
        }
        if (got) whynot::m12_live.fetch_add(1, std::memory_order_relaxed);
        else     whynot::m12_fallback.fetch_add(1, std::memory_order_relaxed);
    }
    if (render_ctx && g_cfg.palette_cam == 8) {
        // Mode 8 (doctrine at the Config key): the tick's published mesh rotation, never a live
        // render-thread component read.
        const long long tk8 = g_tick_cam_ms.load(std::memory_order_acquire);
        const long long now8 = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (tk8 != 0 && now8 - tk8 < 100) {
            const Quat mq{g_tick_mrot_x.load(std::memory_order_relaxed), g_tick_mrot_y.load(std::memory_order_relaxed),
                          g_tick_mrot_z.load(std::memory_order_relaxed), g_tick_mrot_w.load(std::memory_order_relaxed)};
            const Quat M8{g_meshM_x.load(std::memory_order_relaxed), g_meshM_y.load(std::memory_order_relaxed),
                          g_meshM_z.load(std::memory_order_relaxed), g_meshM_w.load(std::memory_order_relaxed)};
            const Quat cam8 = quat_mul(mq, quat_conj(M8));
            float p8 = 0.0f, y8 = 0.0f, r8 = 0.0f;
            quat_to_rotator(cam8.x, cam8.y, cam8.z, cam8.w, &p8, &y8, &r8);
            if (std::isfinite(p8) && std::isfinite(y8)) { cam_pitch = (double)p8; cam_yaw = (double)y8; }
        }
    } else if (render_ctx && g_cfg.palette_cam != 10 && g_cfg.palette_cam != 13 && g_cfg.palette_cam != 14
               && g_cfg.palette_cam != 15 && !g_cfg.palette_sync) {
        // Mode 10 must never touch the mesh read, in either context (that read IS the noise).
        auto* mc = rig_tracked_component();
        Vec3 mrot{};
        if (mc != nullptr && call_ret_vec3(mc, L"K2_GetComponentRotation", &mrot)) {
            const Quat M{g_meshM_x.load(std::memory_order_relaxed), g_meshM_y.load(std::memory_order_relaxed),
                         g_meshM_z.load(std::memory_order_relaxed), g_meshM_w.load(std::memory_order_relaxed)};
            const Quat cam = quat_mul(rotator_to_quat(mrot.x, mrot.y, mrot.z), quat_conj(M));
            float mp = 0.0f, my = 0.0f, mr = 0.0f;
            quat_to_rotator(cam.x, cam.y, cam.z, cam.w, &mp, &my, &mr);
            if (std::isfinite(mp) && std::isfinite(my)) {
                // palrender=3 (doctrine at Config::pal_build_gate): embed the PREVIOUS frame's
                // recovered camera, matching the renderer's one-tick-old transform snapshot.
                if (g_cfg.pal_render == 3) {
                    static float s_dp = 0.0f, s_dy = 0.0f; static bool s_dh = false;
                    if (s_dh) { cam_pitch = (double)s_dp; cam_yaw = (double)s_dy; }
                    else      { cam_pitch = (double)mp;   cam_yaw   = (double)my; }
                    s_dp = mp; s_dy = my; s_dh = true;
                } else {
                    cam_pitch = (double)mp; cam_yaw = (double)my;
                }
            }
        }
    }
    // PALETTECAM: embed against the camera the frame will RENDER under, not the one ControlRotation
    // holds at build time (one tick behind it, fitted 2026-09-03: the gun shifts by 0.86 of a
    // tick of camera yaw per tick). The commanded aim is what the sim is given for this tick.
    {
        const double ctl_p = cam_pitch, ctl_y = cam_yaw;
        bool swapped = false;
        if (g_cfg.palette_cam == 6) {
            // THE GAME THREAD'S CAMERA (2026-09-11, the right-hand-only flicks). Published each
            // tick by the mesh-constant block: the same read, thread and moment as the transform
            // the mesh is built from, so the bones counter exactly the camera the mesh carries.
            // Fresh within 60 ms or the raw read stands (menus, weapon down).
            const long long tk = g_tick_cam_ms.load(std::memory_order_acquire);
            const long long nowk = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            if (tk != 0 && nowk - tk < 60) {
                cam_pitch = (double)g_tick_cam_p.load(std::memory_order_relaxed);
                cam_yaw   = (double)g_tick_cam_y.load(std::memory_order_relaxed);
                swapped = true;
            }
        } else if (!render_ctx && g_cfg.palette_cam == 8) {
            const long long tk8 = g_tick_cam_ms.load(std::memory_order_acquire);
            const long long now8 = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            if (tk8 != 0 && now8 - tk8 < 100) {
                const Quat mq{g_tick_mrot_x.load(std::memory_order_relaxed), g_tick_mrot_y.load(std::memory_order_relaxed),
                              g_tick_mrot_z.load(std::memory_order_relaxed), g_tick_mrot_w.load(std::memory_order_relaxed)};
                const Quat M8{g_meshM_x.load(std::memory_order_relaxed), g_meshM_y.load(std::memory_order_relaxed),
                              g_meshM_z.load(std::memory_order_relaxed), g_meshM_w.load(std::memory_order_relaxed)};
                const Quat cam8 = quat_mul(mq, quat_conj(M8));
                float p8 = 0.0f, y8 = 0.0f, r8 = 0.0f;
                quat_to_rotator(cam8.x, cam8.y, cam8.z, cam8.w, &p8, &y8, &r8);
                if (std::isfinite(p8) && std::isfinite(y8)) { cam_pitch = (double)p8; cam_yaw = (double)y8; swapped = true; }
            }
        } else if (!render_ctx && g_cfg.palette_cam == 7) {
            // THE REFRESH'S CAMERA (doctrine at the Config key): one camera for both bank
            // writers. Fresh within 60 ms or the raw read stands.
            const long long rms = g_rcam_ms.load(std::memory_order_acquire);
            const long long rnow = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            // A WHOLE SECOND of grace, not 60 ms: a stale mesh camera tracks the headset and is
            // nearly right; the raw read is wrong by the full wrist deflection. Falling back
            // fast was the leap (fitted, the 47-52 deg error tail).
            if (rms != 0 && rnow - rms < 1000) {
                cam_pitch = (double)g_rcam_p.load(std::memory_order_relaxed);
                cam_yaw   = (double)g_rcam_y.load(std::memory_order_relaxed);
                swapped = true;
            }
        } else if (!render_ctx && g_cfg.palette_cam == 1 && g_aim_law_armed.load(std::memory_order_relaxed)) {
            const float dy = g_desired_yaw.load(std::memory_order_relaxed);
            const float dp = g_desired_pitch.load(std::memory_order_relaxed);
            if (std::isfinite(dy) && std::isfinite(dp)) { cam_yaw = dy; cam_pitch = dp; swapped = true; }
        } else if (!render_ctx && g_cfg.palette_cam == 2) {
            float dy = 0.0f, dp = 0.0f;
            if (desired_aim_now(&dy, &dp) && std::isfinite(dy) && std::isfinite(dp)) { cam_yaw = dy; cam_pitch = dp; swapped = true; }
        } else if (!render_ctx && g_cfg.palette_cam == 4) {
            // THE RENDERED YAW (fitted 2026-09-11): the view lock ASSIGNS the yaw every frame,
            // so the yaw the frame renders under is OUR OWN number, published as
            // g_view_base_yaw. The lock gap -- ControlRotation minus that yaw -- measured a
            // -20..+6 degree wander (sd 6.3) in the wrist-rotation trace, and the gap moves at
            // wrist speed because the aim steers the game camera while the lock holds the view.
            // A gun embedded against ControlRotation swings by exactly that gap. Embedding
            // against the rendered yaw removes the gap by construction; pitch stays
            // ControlRotation's, the lock does not touch pitch.
            cam_yaw = (double)g_view_base_yaw.load(std::memory_order_relaxed);
            swapped = true;
        } else if (!render_ctx && g_cfg.palette_cam == 5) {
            // SMOOTHED-RATE LEAD (fitted 2026-09-11). Mode 3 extrapolated by the raw last step
            // and amplified noise; mode 1 injected the aim loop's tracking error. This one leads
            // ControlRotation along its own EMA-smoothed angular rate (~80 ms, so steady wrist
            // rotation gives a clean constant lead and jitter is attenuated instead of doubled)
            // by palettecamlead of one build interval -- the fitted miss was 0.86 of a tick.
            static double s5_py = 0.0, s5_pp = 0.0; static long long s5_ms = 0; static bool s5_have = false;
            static float s5_ry = 0.0f, s5_rp = 0.0f;
            const long long s5_now = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            if (s5_have) {
                double dy = ctl_y - s5_py; while (dy > 180.0) dy -= 360.0; while (dy < -180.0) dy += 360.0;
                const double dp = ctl_p - s5_pp;
                const float dts = (float)(s5_now - s5_ms) * 0.001f;
                if (dts > 1e-4f && dts < 0.1f) {
                    const float ry = (float)(dy / dts), rp = (float)(dp / dts);
                    const float a = dts / (dts + 0.080f);
                    if (std::fabs(ry) < 720.0f) s5_ry += a * (ry - s5_ry);
                    if (std::fabs(rp) < 720.0f) s5_rp += a * (rp - s5_rp);
                    cam_yaw   = ctl_y + (double)(s5_ry * dts * g_cfg.palette_cam_lead);
                    cam_pitch = ctl_p + (double)(s5_rp * dts * g_cfg.palette_cam_lead);
                    swapped = true;
                }
            }
            s5_py = ctl_y; s5_pp = ctl_p; s5_ms = s5_now; s5_have = true;
        } else if (!render_ctx && g_cfg.palette_cam == 3) {
            static double s_py = 0.0, s_pp = 0.0; static bool s_have = false;
            if (s_have) {
                double ddy = ctl_y - s_py; while (ddy > 180.0) ddy -= 360.0; while (ddy < -180.0) ddy += 360.0;
                cam_yaw = ctl_y + ddy; cam_pitch = ctl_p + (ctl_p - s_pp); swapped = true;
            }
            s_py = ctl_y; s_pp = ctl_p; s_have = true;
        }
        if (!render_ctx && g_cfg.judder_log > 0) {
            static int s_cl = 0;
            if (s_cl < 3000) {
                ++s_cl;
                double dy = cam_yaw - ctl_y; while (dy > 180.0) dy -= 360.0; while (dy < -180.0) dy += 360.0;
                API::get()->log_info("[Halo-CampE-UEVR] CAMLAG ctl=(p%.2f y%.2f) used=(p%.2f y%.2f) d=(%.2f %.2f) mode=%d swapped=%d",
                                     ctl_p, ctl_y, cam_pitch, cam_yaw, cam_pitch - ctl_p, dy, g_cfg.palette_cam, (int)swapped);
            }
        }
    }
    // ---- CAMLEAD (Config.hpp cam_lead_all). Doctrine and the two independent measurements of
    // the 0.86-tick miss are at the Config key. Leads whatever camera the mode chose along its
    // own EMA-smoothed rate, so mode 7's two-writer fix and mode 5's lag fix COMPOSE instead of
    // excluding each other. Separate state per context: the build and the refresh run at
    // different cadences, and one shared EMA would make each see the other's interval.
    if (g_cfg.cam_lead_all != 0 && g_cfg.palette_cam != 5) {
        struct LeadState { double py = 0.0, pp = 0.0; long long ms = 0; bool have = false; float ry = 0.0f, rp = 0.0f; };
        static LeadState s_lead[2];
        LeadState& L = s_lead[render_ctx ? 1 : 0];
        const long long lnow = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (L.have) {
            double dy = cam_yaw - L.py; while (dy > 180.0) dy -= 360.0; while (dy < -180.0) dy += 360.0;
            const double dp = cam_pitch - L.pp;
            const float dts = (float)(lnow - L.ms) * 0.001f;
            if (dts > 1.0e-4f && dts < 0.1f) {
                const float ry = (float)(dy / dts), rp = (float)(dp / dts);
                const float a = dts / (dts + 0.080f);
                if (std::fabs(ry) < 720.0f) L.ry += a * (ry - L.ry);
                if (std::fabs(rp) < 720.0f) L.rp += a * (rp - L.rp);
                const double ly = (double)(L.ry * dts * g_cfg.palette_cam_lead);
                const double lp = (double)(L.rp * dts * g_cfg.palette_cam_lead);
                if (std::isfinite(ly) && std::isfinite(lp)) { cam_yaw += ly; cam_pitch += lp; }
            }
        }
        L.py = cam_yaw; L.pp = cam_pitch; L.ms = lnow; L.have = true;
    }

    // Published per context: the build's camera for the hooked_pose stamp, the refresh's for
    // palettecam=7 and the point-8 stamp. One resolve serves both callers, so an unconditional
    // store here would let a render-thread resolve contaminate the build stamp mid-read.
    if (render_ctx) {
        g_rcam_p.store((float)cam_pitch, std::memory_order_relaxed);
        g_rcam_y.store((float)cam_yaw, std::memory_order_relaxed);
        g_rcam_ms.store((long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count(), std::memory_order_release);
    } else {
        g_dbg_build_cam_p.store((float)cam_pitch, std::memory_order_relaxed);
        g_dbg_build_cam_y.store((float)cam_yaw, std::memory_order_relaxed);
    }
    // PALETTECAMSMOOTH (doctrine at the Config key): one shared EMA state so build and render
    // contexts smooth the SAME trajectory; wrap-aware in yaw. Applied after every source swap.
    if (g_cfg.palette_cam_smooth_ms > 1.0f) {
        static double s_sm_p = 0.0, s_sm_y = 0.0; static long long s_sm_ms = 0; static bool s_sm_have = false;
        const long long sm_now = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (!s_sm_have || sm_now - s_sm_ms > 500) { s_sm_p = cam_pitch; s_sm_y = cam_yaw; s_sm_have = true; }
        else {
            const float dtm = (float)(sm_now - s_sm_ms);
            const float a = dtm / (dtm + g_cfg.palette_cam_smooth_ms);
            double dy = cam_yaw - s_sm_y; while (dy > 180.0) dy -= 360.0; while (dy < -180.0) dy += 360.0;
            s_sm_y += a * dy; while (s_sm_y > 180.0) s_sm_y -= 360.0; while (s_sm_y < -180.0) s_sm_y += 360.0;
            s_sm_p += a * (cam_pitch - s_sm_p);
        }
        s_sm_ms = sm_now;
        cam_pitch = s_sm_p; cam_yaw = s_sm_y;
    }
    // PALETTESYNC: the aim this placement divides by is the one sampled WITH the hand being placed.
    if (g_cfg.palette_sync && g_p_sync_ok.load(std::memory_order_relaxed)) {
        cam_pitch = (double)g_p_sync_cp.load(std::memory_order_relaxed);
        cam_yaw   = (double)g_p_sync_cy.load(std::memory_order_relaxed);
    }
    // DRAWAIM (palettecam 14/15): divide by the aim the drawn frame will compose with, stored at
    // the latch refresh (doctrine at pose_latch_refresh). 14 = previous snapshot's intent (the fit),
    // 15 = current snapshot's intent (the lag-direction check). Same number in both contexts.
    if (g_cfg.palette_cam == 14 && g_intent_prev_ok.load(std::memory_order_relaxed)) {
        cam_pitch = (double)g_intent_prev_p.load(std::memory_order_relaxed);
        cam_yaw   = (double)g_intent_prev_y.load(std::memory_order_relaxed);
    } else if (g_cfg.palette_cam == 15 && g_intent_cur_ok.load(std::memory_order_relaxed)) {
        cam_pitch = (double)g_intent_cur_p.load(std::memory_order_relaxed);
        cam_yaw   = (double)g_intent_cur_y.load(std::memory_order_relaxed);
    }
    // Point 28: the divisor this placement uses, its snapshot generation, and which writer.
    // The generation is only meaningful when the divisor came from a stored intent: 14 with a valid
    // previous intent, 15 with a valid current one. Every other mode stamps 0 (not a snapshot value).
    if (g_cfg.stomp_log != 0) {
        uint32_t dg = 0;
        if (g_cfg.palette_cam == 14 && g_intent_prev_ok.load(std::memory_order_relaxed))
            dg = g_intent_prev_gen.load(std::memory_order_relaxed);
        else if (g_cfg.palette_cam == 15 && g_intent_cur_ok.load(std::memory_order_relaxed))
            dg = g_intent_cur_gen.load(std::memory_order_relaxed);
        stomp_mark(28, (float)cam_yaw, (float)cam_pitch, (float)dg, render_ctx ? 1.0f : 0.0f);
    }
    const Quat cam  = rotator_to_quat((float)cam_pitch, (float)cam_yaw, 0.0f);

    const Quat M{g_meshM_x.load(std::memory_order_relaxed),
                 g_meshM_y.load(std::memory_order_relaxed),
                 g_meshM_z.load(std::memory_order_relaxed),
                 g_meshM_w.load(std::memory_order_relaxed)};
    const Vec3 v0{g_meshV0_x.load(std::memory_order_relaxed),
                  g_meshV0_y.load(std::memory_order_relaxed),
                  g_meshV0_z.load(std::memory_order_relaxed)};

    // FIX (reviewer, .5-parity): the palette bones are SKELETON-LOCAL and the game already renders
    // them under the live camera every frame (TRACE: mesh_rot == cam, always). Dividing the hand by
    // the live camera HERE applies the camera a SECOND time -- that is the head/aim "swim". .5 writes
    // these same bones with NO camera term at all; the only frame map it uses is the recenter offset,
    // which the publisher already applied as q_ro. So neutralise the camera composition to identity:
    // comp_rot/comp_inv become no-ops, and the placement reduces to exactly .5's
    // (recenter-composed hand -> openxr_to_blam -> /3.048 -> palette[0]-relative). cam/M/v0 are left
    // computed (used only by the log line below) but no longer steer placement.
    // 2026-08-15 18:42 session, fitted (25 rows, zero free parameters): the rendered socket sits at
    // R_cam(pitch,yaw) * published_hand to 4.5 cm RMS. The bones are camera-local and the game
    // applies the FULL camera. Identity here therefore rendered the room hand rotated by the
    // camera -- off from the real hand by the lock gap in yaw and the whole aim pitch (23 cm RMS,
    // 63 cm max, growing with |gap| and |pitch| row by row). Divide by the camera the mesh
    // renders under, and lift the room hand by the frame the room actually sits in (below).
    // MODE 9 divides by the mesh's own rotation -- no euler round trip, no dropped roll, no
    // cam*M product of two different moments (doctrine at Config::palette_cam).
    // MODE 10 (2026-09-12, the frozen-pose verdict): with the pose frozen the gun is rock solid
    // under HEAD motion and jitters under WRIST motion -- the wrist is the only thing that still
    // turns the game camera, so the composition only fails while the mesh is TURNING. Our mesh
    // reads step up to 43 deg/frame while ControlRotation steps under half a degree and the
    // stock gun riding that same mesh is smooth: the read is the noise, not the mesh. And the
    // MESHCONST log proves the mesh rotation EQUALS ControlRotation at rest (M = identity,
    // comp_rot == cam to the decimal). So mode 10 composes from ControlRotation alone -- no mesh
    // read, no M, no euler recovery -- and the apparent 68 deg "M drift" is treated as the read
    // artifact it appears to be.
    const Quat comp_rot = (g_cfg.palette_cam == 10 || g_cfg.palette_cam == 14 || g_cfg.palette_cam == 15) ? cam
                        : (((g_cfg.palette_cam == 9 || g_cfg.palette_cam == 11 || g_cfg.palette_cam == 12) && mesh_q_ok && !g_cfg.palette_sync)
                               ? mesh_q : quat_mul(cam, M));
    // ---- ONE DIVISOR PER FRAME (Config.hpp comp_latch). Reasoning and numbers at the Config
    // key. Whichever context resolves first in a frame publishes the divisor; every later write
    // in that frame reuses it, so the build-time landing and the render-time refresh cannot
    // place the weapon with two different mesh samples.
    Quat comp_used = comp_rot;
    if (g_cfg.comp_latch != 0) {
        const long long cl_now = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        bool got = false;
        for (int i = 0; i < 4; ++i) {
            const uint32_t s0 = g_cl.seq.load(std::memory_order_acquire);
            const long long lms = g_cl.ms.load(std::memory_order_relaxed);
            const Quat q{g_cl.x.load(std::memory_order_relaxed), g_cl.y.load(std::memory_order_relaxed),
                         g_cl.z.load(std::memory_order_relaxed), g_cl.w.load(std::memory_order_relaxed)};
            if ((s0 & 1u) || g_cl.seq.load(std::memory_order_acquire) != s0) continue;
            const float n = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
            if (lms != 0 && (float)(cl_now - lms) <= g_cfg.comp_latch_ms && n > 0.9f && n < 1.1f) {
                comp_used = Quat{q.x/n, q.y/n, q.z/n, q.w/n};
                got = true;
                whynot::clatch_hit.fetch_add(1, std::memory_order_relaxed);
            }
            break;
        }
        if (!got) {                 // first resolve of this frame: publish for the rest of it
            g_cl.seq.fetch_add(1u, std::memory_order_acq_rel);
            g_cl.x.store(comp_rot.x, std::memory_order_relaxed);
            g_cl.y.store(comp_rot.y, std::memory_order_relaxed);
            g_cl.z.store(comp_rot.z, std::memory_order_relaxed);
            g_cl.w.store(comp_rot.w, std::memory_order_relaxed);
            g_cl.ms.store(cl_now, std::memory_order_relaxed);
            g_cl.seq.fetch_add(1u, std::memory_order_acq_rel);
            whynot::clatch_miss.fetch_add(1, std::memory_order_relaxed);
        }
    }
    Quat comp_inv = quat_conj(comp_used);
    // COMPGAIN (Config.hpp comp_gain). The fit and the numbers behind it are at the Config key.
    // Raises the divisor to a real power by slerping from identity along the shortest arc, so
    // +1 is unchanged, -1 is the flip the E = mesh^2 fit predicts, and 0 removes the divide.
    if (g_cfg.comp_gain != 1.0f) {
        Quat q = comp_inv;
        if (q.w < 0.0f) { q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w; }   // shortest arc
        float cw = q.w; if (cw > 1.0f) cw = 1.0f; if (cw < -1.0f) cw = -1.0f;
        const float half = std::acos(cw);                  // half the rotation angle
        const float s = std::sqrt(1.0f - cw * cw);
        if (s > 1.0e-6f && std::isfinite(half)) {
            const float nh = half * g_cfg.comp_gain;       // scaled angle
            const float sc = std::sin(nh) / s;
            comp_inv = Quat{q.x * sc, q.y * sc, q.z * sc, std::cos(nh)};
            const float n = std::sqrt(comp_inv.x*comp_inv.x + comp_inv.y*comp_inv.y
                                    + comp_inv.z*comp_inv.z + comp_inv.w*comp_inv.w);
            if (n > 1.0e-6f && std::isfinite(n)) {
                comp_inv = Quat{comp_inv.x/n, comp_inv.y/n, comp_inv.z/n, comp_inv.w/n};
            } else {
                comp_inv = quat_conj(comp_used);
            }
        }
    }
    // ---- PALSTEP (2026-09-12, fitted before written). RAYGUN, per render frame, paletteposelatch=1:
    //   aim(t) = intent(t-2) exactly (coef 1.000, R2 1.000)
    //   barrel(t) - aim(t) = k * (aim(t-1) - aim(t-2)),  k 0.94 yaw / 0.91 pitch, corr 0.92 / 0.94
    // The drawn gun runs one previous aim step ahead of the aim ray: the overshoot, and the return
    // when the wrist slows. Cancelling that term drops barrel-off-aim sd while moving from
    // 1.30/1.43 deg to 0.47/0.47 deg. The fit fixes the size, not the sign at the divisor, so the
    // gain is signed and live: comp_inv gains conj(D), D = R(aim + g*step) * conj(R(aim)).
    //   palstepsrc 0 = ControlRotation step seen by THIS context, 1 = intent step (g_desired_*)
    //   palstepctx 0 = both writers, 1 = render refresh only, 2 = build only
    if (g_cfg.palette_step != 0.0f) {
        const bool ctx_on = g_cfg.palette_step_ctx == 0 || (g_cfg.palette_step_ctx == 1 && render_ctx)
                         || (g_cfg.palette_step_ctx == 2 && !render_ctx);
        struct StepState { float y = 0.0f, p = 0.0f; long long ms = 0; bool have = false; };
        static StepState s_st[4];
        const int si = (render_ctx ? 1 : 0) + (g_cfg.palette_step_src == 1 ? 2 : 0);
        StepState& S = s_st[si];
        const float src_y = (g_cfg.palette_step_src == 1) ? g_desired_yaw.load(std::memory_order_relaxed) : tr.ctl_y;
        const float src_p = (g_cfg.palette_step_src == 1) ? g_desired_pitch.load(std::memory_order_relaxed) : tr.ctl_p;
        const long long st_now = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        static std::atomic<uint32_t> s_ps_n{0}; static std::atomic<float> s_ps_sum{0.0f};
        static long long s_ps_log = 0;
        if (S.have && st_now - S.ms < 60 && std::isfinite(src_y) && std::isfinite(src_p)) {
            float dy = src_y - S.y; while (dy > 180.0f) dy -= 360.0f; while (dy < -180.0f) dy += 360.0f;
            const float dp = src_p - S.p;
            if (ctx_on && std::fabs(dy) < 30.0f && std::fabs(dp) < 30.0f) {
                const float g = g_cfg.palette_step;
                const Quat r0 = rotator_to_quat(tr.ctl_p, tr.ctl_y, 0.0f);
                const Quat r1 = rotator_to_quat(tr.ctl_p + g * dp, tr.ctl_y + g * dy, 0.0f);
                const Quat D = quat_mul(r1, quat_conj(r0));
                Quat ci = quat_mul(comp_inv, quat_conj(D));
                const float n = std::sqrt(ci.x*ci.x + ci.y*ci.y + ci.z*ci.z + ci.w*ci.w);
                if (n > 0.9f && n < 1.1f && std::isfinite(n)) {
                    comp_inv = Quat{ci.x/n, ci.y/n, ci.z/n, ci.w/n};
                    s_ps_n.fetch_add(1, std::memory_order_relaxed);
                    s_ps_sum.store(s_ps_sum.load(std::memory_order_relaxed) + std::fabs(dy) + std::fabs(dp), std::memory_order_relaxed);
                }
            }
        }
        S.y = src_y; S.p = src_p; S.ms = st_now; S.have = true;
        if (s_ps_log == 0) s_ps_log = st_now;
        if (st_now - s_ps_log >= 10000) {
            const uint32_t n = s_ps_n.exchange(0);
            const float sum = s_ps_sum.exchange(0.0f);
            API::get()->log_info("[Halo-CampE-UEVR] PALSTEP gain %+.3f src %d ctx %d: %u corrections in 10 s, mean |step| %.3f deg",
                                 g_cfg.palette_step, g_cfg.palette_step_src, g_cfg.palette_step_ctx, n, n ? sum / n : 0.0f);
            s_ps_log = st_now;
        }
    }
    tr.cam_p = (float)cam_pitch; tr.cam_y = (float)cam_yaw;
    tr.mq[0] = mesh_q.x; tr.mq[1] = mesh_q.y; tr.mq[2] = mesh_q.z; tr.mq[3] = mesh_q.w;
    tr.mq_ok = mesh_q_ok ? 1 : 0;
    tr.M[0] = M.x; tr.M[1] = M.y; tr.M[2] = M.z; tr.M[3] = M.w;
    tr.comp[0] = comp_rot.x; tr.comp[1] = comp_rot.y; tr.comp[2] = comp_rot.z; tr.comp[3] = comp_rot.w;
    if (g_cfg.palette_cam == 9 && g_cfg.judder_log > 0) {
        static int s_m9 = 0;
        if (s_m9 < 400) {
            ++s_m9;
            const Quat rc = quat_mul(cam, M);
            float d = rc.x*comp_rot.x + rc.y*comp_rot.y + rc.z*comp_rot.z + rc.w*comp_rot.w;
            if (d < 0.0f) d = -d; if (d > 1.0f) d = 1.0f;
            API::get()->log_info("[Halo-CampE-UEVR] MODE9 mesh-vs-reconstruction = %.3f deg (render=%d ok=%d)",
                                 2.0f * std::acos(d) * 57.29578f, (int)render_ctx, (int)mesh_q_ok);
        }
    }

    // Desired offset from the parent origin, metres -> cm. palettewpnscale here is the
    // metre-to-game scale factor divided by 100 (UEVR world scale, the rig's own lesson:
    // rigscale MUST include it or under-translation masquerades as a pivot error).
    const float cm_per_m = 100.0f * ((g_cfg.palette_weapon_scale > 0.01f)
                                     ? g_cfg.palette_weapon_scale : 1.0f);
    // THE FRAME BUG (2026-08-15, found by logging, not theory): the published hand is a HEAD-
    // RELATIVE, ROOM-YAW vector -- with the recenter at identity, "forward" is the room's forward,
    // not the camera's. The lever cam*v0 is a WORLD vector. Subtracting a world lever from a room
    // hand mixes frames by exactly the camera yaw: near zero yaw it is invisible (13:51 worked),
    // and after a turn -- 110-degree lock gap live in the log -- a hand 54 cm forward pulled back
    // to a weapon behind the face. Measured: room=(0.538, -0.07, -0.39) with the hand held
    // straight out; bone came out at x=-0.09. The head-relative hand must be rotated INTO the
    // world by the camera before anything world-frame is subtracted from it.
    //
    // YAW: the camera's yaw, and the position self-test proves it exact -- (0.164, 0, 0) at
    // camera yaws of 57, 63, 95 and 150.
    //
    // PITCH: NOT the camera's pitch, and not zero either -- the DIFFERENCE between the aim
    // camera's pitch and the head's. Decoupled pitch is ON (correctly: a hand-driven camera must
    // never pitch the rendered view), so the view the palette renders under pitches with the
    // HEAD while the camera the pullback divides by pitches with the HAND. A room hand is a real
    // offset from the head, so it must be lifted by (aim pitch - head pitch) to be expressed in
    // the aim camera's frame that conj(cam*M) then removes. Measured before this term existed:
    // bone Z = +0.116 at cam pitch -45 with the head level -- the whole aim pitch leaking through.
    // A full-cam lift (aim pitch alone) would have been the wrong fix; it cancels the aim's pitch
    // and leaves the head's, which is exactly the residual the player feels when looking around.
    // ---- ONE HAND PER FRAME (Config.hpp palette_latch). Measurements and reasoning are at the
    // Config key. Grip and aim are latched TOGETHER, so no write can pair one frame's position
    // with another frame's orientation either.
    Vec3 hand_room{};
    Quat latched_aim{0.0f, 0.0f, 0.0f, 1.0f};
    bool have_latch = false;
    const long long hl_now = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    if (g_cfg.palette_latch != 0) {
        for (int t_try = 0; t_try < 4; ++t_try) {
            const uint32_t s0 = g_hl.seq.load(std::memory_order_acquire);
            const long long lms = g_hl.ms.load(std::memory_order_relaxed);
            const Quat la{g_hl.ax.load(std::memory_order_relaxed), g_hl.ay.load(std::memory_order_relaxed),
                          g_hl.az.load(std::memory_order_relaxed), g_hl.aw.load(std::memory_order_relaxed)};
            const Vec3 lg{g_hl.gx.load(std::memory_order_relaxed), g_hl.gy.load(std::memory_order_relaxed),
                          g_hl.gz.load(std::memory_order_relaxed)};
            if ((s0 & 1u) || g_hl.seq.load(std::memory_order_acquire) != s0) continue;
            if (lms != 0 && (float)(hl_now - lms) <= g_cfg.palette_latch_ms) {
                latched_aim = la; hand_room = lg; have_latch = true;
                whynot::latch_hit.fetch_add(1, std::memory_order_relaxed);
            }
            break;
        }
    }
    if (!have_latch) {
        {   // seqlocked copy: never a pose mixed from two frames (doctrine at g_p_seq)
            for (int sl_try = 0; sl_try < 4; ++sl_try) {
                const uint32_t s0 = g_p_seq.load(std::memory_order_acquire);
                hand_room = Vec3{g_p_grip_x.load(std::memory_order_relaxed) * cm_per_m,
                                 g_p_grip_y.load(std::memory_order_relaxed) * cm_per_m,
                                 g_p_grip_z.load(std::memory_order_relaxed) * cm_per_m};
                latched_aim = Quat{g_p_aim_x.load(std::memory_order_relaxed), g_p_aim_y.load(std::memory_order_relaxed),
                                   g_p_aim_z.load(std::memory_order_relaxed), g_p_aim_w.load(std::memory_order_relaxed)};
                if (!(s0 & 1u) && g_p_seq.load(std::memory_order_acquire) == s0) break;
            }
        }
        if (g_cfg.palette_latch != 0) {     // open the window for the rest of this frame's writes
            g_hl.seq.fetch_add(1u, std::memory_order_acq_rel);
            g_hl.ax.store(latched_aim.x, std::memory_order_relaxed);
            g_hl.ay.store(latched_aim.y, std::memory_order_relaxed);
            g_hl.az.store(latched_aim.z, std::memory_order_relaxed);
            g_hl.aw.store(latched_aim.w, std::memory_order_relaxed);
            g_hl.gx.store(hand_room.x, std::memory_order_relaxed);
            g_hl.gy.store(hand_room.y, std::memory_order_relaxed);
            g_hl.gz.store(hand_room.z, std::memory_order_relaxed);
            g_hl.ms.store(hl_now, std::memory_order_relaxed);
            g_hl.seq.fetch_add(1u, std::memory_order_acq_rel);
        }
        whynot::latch_miss.fetch_add(1, std::memory_order_relaxed);
    }
    // THE LOCK GAP, MEASURED (TRACE, 2026-08-15 15:43): head/view yaw -100, Blam camera yaw
    // +160, mesh_rot == cam. The FP mesh renders in the CAMERA's frame; the player looks along
    // the VIEW's frame; under the view lock those differ by the pinned offset -- 100 degrees in
    // that log, and 98/110/125 in every capture the calibration ever solved "with the controller
    // on the gun". Every one of those was this gap, measured through the weapon. The synthetic
    // orientation test PASSED (all three columns match the stock node), so there is NO
    // convention error in the conversion; the residual was never in the basis. It is here:
    //
    // The published hand is head-relative in the ROOM frame, which the view lock pins to the
    // VIEW's yaw. Lifting it by the full camera yaw put a view-frame hand into the camera frame
    // as if the two agreed -- and they differ by exactly the lock gap. The correct lift is
    // (camera - view): the room hand already sits at view yaw, so only the difference between
    // the two frames is missing. Pitch takes the same treatment (view pitch = head pitch under
    // decoupled pitch, which is what the earlier pitch fix already used).
    // NO HEAD TERM. NO VIEW TERM. The camera's yaw, and nothing else.
    //
    // Two corrections were stacked here on 2026-08-15 and both were wrong, and a reviewer's
    // controlled sweep proved it with the arms as ground truth: head swept -3 to +71 degrees with
    // the hand still, and the weapon placement rotated by exactly the head pitch --
    // 0.164*cos(head), 0.164*sin(head) -- while the ARMS, which have no head term, held still.
    // The "(aim - head) pitch" fix did not cancel a head dependence; it CREATED one. And the
    // ~100-125 degrees every calibration solved was never a convention error (the synthetic
    // orientation test proved the basis exact at level): the player calibrates looking DOWN at
    // the hand, and the head term had swung the gun by that much. The "(camera - view)" yaw
    // lift, added on top, put the synthetic forward hand at bone x = -0.162 -- behind the face.
    //
    // The state the yaw self-test proved exact -- (0.164, 0, 0) at camera yaws 57, 63, 95, 150
    // -- had the camera yaw ONLY. Back to it, and nothing added. The room hand is anchored to the
    // standing origin, which does not move with the head, so there is no head motion to remove;
    // and the camera lift is the frame the mesh renders in, full stop.
    tr.hand_room[0] = hand_room.x; tr.hand_room[1] = hand_room.y; tr.hand_room[2] = hand_room.z;
    tr.view_yaw = g_view_base_yaw.load(std::memory_order_relaxed);
    (void)g_p_head_pitch;
    // FIX (reviewer, .5-parity): NO camera-yaw lift. The publisher already put the hand in the
    // recenter frame (q_ro) anchored to the standing origin -- that IS .5's grip_delta. Lifting by
    // the live camera yaw re-introduces the very frame the game applies at render. Identity here;
    // the hand passes straight through to the axis-swizzle/scale below.
    // THE LIFT IS THE VIEW'S BASE YAW -- 0.2's T (q_turn), level, no pitch. Under the lock the
    // room frame is pinned to g_view_base_yaw (Plugin.cpp: the yaw handed to UEVR every frame;
    // with the lock off it IS the camera yaw, so this and the divide cancel to 0.5's bone = hand).
    // Never previously run in this combination: cam-yaw lift (off by the gap), (cam-view) on top
    // of it (2*cam - view), identity (off by cam). Round 9 of the fit: this makes the rendered
    // socket land on Rot(view_yaw)*hand by construction.
    // LIFTYAW (Config.hpp lift_yaw). The lock gap, and the measurements behind it, are at the
    // Config key. Mode 0 lifts by the LOCKED view yaw and leaves the gap standing; modes 1 and 2
    // lift by the same yaw the divide uses, so the gap cancels instead of riding the weapon.
    float lift_y = g_view_base_yaw.load(std::memory_order_relaxed);
    if (g_cfg.lift_yaw == 1) {
        lift_y = (float)cam_yaw;
    } else if (g_cfg.lift_yaw == 2) {
        // The yaw of comp_rot itself. Correct even where the divisor is the mesh rotation rather
        // than cam * M, which is the case in mode 12.
        float cp2 = 0.0f, cy2 = 0.0f, cr2 = 0.0f;
        quat_to_rotator(comp_rot.x, comp_rot.y, comp_rot.z, comp_rot.w, &cp2, &cy2, &cr2);
        if (std::isfinite(cy2)) lift_y = cy2;
    }
    const Quat cam_yaw_only = rotator_to_quat(0.0f, lift_y, 0.0f);
    Vec3 hand_cm = quat_rotate(cam_yaw_only, hand_room);                // room hand -> world
    // CAMERA BOB CANCEL: the mesh rides the bobbing camera; the view is rendered from the camera
    // minus the bob (Plugin.cpp stereo callback). Take the same vector out of the hand so the gun
    // stays where the hand is in the stabilised view instead of bobbing against it.
    hand_cm.x -= g_bob_x.load(std::memory_order_relaxed);
    hand_cm.y -= g_bob_y.load(std::memory_order_relaxed);
    hand_cm.z -= g_bob_z.load(std::memory_order_relaxed);
    tr.bob[0] = g_bob_x.load(std::memory_order_relaxed);
    tr.bob[1] = g_bob_y.load(std::memory_order_relaxed);
    tr.bob[2] = g_bob_z.load(std::memory_order_relaxed);
    tr.hand_cm[0] = hand_cm.x; tr.hand_cm[1] = hand_cm.y; tr.hand_cm[2] = hand_cm.z;

    // Same lift for the ORIENTATION, or position and orientation diverge by the camera yaw --
    // "it moves but doesn't rotate right". The published pose is room-frame; bring it into the
    // world by the same rotation. Identical frames for both halves, always.
    // Latched above alongside the grip, so position and orientation can never come from two
    // different frames and no two writes in one frame can disagree.
    Quat aim_q = latched_aim;
    g_eff_raw_pose = aim_q;          // for PALETTELOCAL, the hand with no frame attached
    Quat pose_world = quat_mul(cam_yaw_only, aim_q);
    tr.aim[0] = aim_q.x; tr.aim[1] = aim_q.y; tr.aim[2] = aim_q.z; tr.aim[3] = aim_q.w;
    tr.tru_g[0] = g_p_tru_gx.load(std::memory_order_relaxed);
    tr.tru_g[1] = g_p_tru_gy.load(std::memory_order_relaxed);
    tr.tru_g[2] = g_p_tru_gz.load(std::memory_order_relaxed);
    tr.tru_a[0] = g_p_tru_ax.load(std::memory_order_relaxed);
    tr.tru_a[1] = g_p_tru_ay.load(std::memory_order_relaxed);
    tr.tru_a[2] = g_p_tru_az.load(std::memory_order_relaxed);
    tr.tru_a[3] = g_p_tru_aw.load(std::memory_order_relaxed);
    tr.mrot[0] = g_tick_mrot_x.load(std::memory_order_relaxed);
    tr.mrot[1] = g_tick_mrot_y.load(std::memory_order_relaxed);
    tr.mrot[2] = g_tick_mrot_z.load(std::memory_order_relaxed);
    tr.mrot[3] = g_tick_mrot_w.load(std::memory_order_relaxed);
    tr.brk  = g_meshM_brk_deg.load(std::memory_order_relaxed);
    tr.mage = (float)((long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now().time_since_epoch()).count()
              - g_meshM_acc_ms.load(std::memory_order_relaxed));
    tr.pose_lift[0] = pose_world.x; tr.pose_lift[1] = pose_world.y;
    tr.pose_lift[2] = pose_world.z; tr.pose_lift[3] = pose_world.w;

    const bool held = g_freeze_key_held.load(std::memory_order_acquire);
    static uint32_t s_wf_linger = 0;    // frames to keep publishing F after release
    // The lock and the weapon trim run BEFORE the freeze block, so what the freeze latches is the
    // pose that actually renders (lock + trim), and the Home/PageUp solve is measured on it.
    // ---- ONE SHARED AIM DIRECTION (Config::palette_barrel_lock). The reticle is the camera
    // forward; the barrel is the pose's measured barrel axis. Rotate the pose by the shortest arc
    // from one to the other, in WORLD, this build -- both read this build, so nothing is a frame
    // apart, and a shortest arc adds no twist, so the controller's roll about the barrel survives.
    // Not while frozen/lingering: a frozen gun must hold still in the world, not follow the aim.
    // NOT GATED ON `held`, and that gate was a self-inflicted runaway. MEASURED 2026-08-17:
    //
    // The lock is a LEFT-multiply that bends the gun onto the ray, and it was skipped while a
    // freeze key was down -- so the freeze latched the UNLOCKED pose while the player saw, and the
    // release solve compared against, the LOCKED one. The capture therefore measured the lock's own
    // correction and wrote it into gripfix. From a hold where the hand never moved:
    //   key-down snap  16.6 deg  (socket roll -11.6 -> +5.0, offAim 0.3 -> 16.5)
    //   solve reported 16 deg
    //   gripfix        7.6 -> 23.8 deg      (7.6 + 16.2, exact)
    // and the lock's size is itself explained: aimfix puts 8.0 deg of pitch on the RAY that the
    // palette pose does not carry, plus gripfix's own 7.6 = 15.6 deg. Next hold would need 31.8,
    // past the 25-degree cap below, at which point the lock silently quits -- which is exactly what
    // the second hold showed (offAim pinned at 32.8 the whole time, solve collapsing to 1 deg).
    //
    // Ungating costs nothing: the freeze block a few lines down OVERWRITES pose_world with the
    // latched pose while held, so the lock's output is computed and discarded for the duration. The
    // frozen gun still does not chase the aim -- and now the latch is the pose that is ON SCREEN,
    // which is the one the player is aligning a controller to. That is what the comment above this
    // block always claimed; the gate is what stopped it being true.
    if (g_cfg.palette_barrel_lock && g_barrel_axis_valid.load(std::memory_order_relaxed)) {
        const Vec3 b_l{g_barrel_axis_x.load(std::memory_order_relaxed),
                       g_barrel_axis_y.load(std::memory_order_relaxed),
                       g_barrel_axis_z.load(std::memory_order_relaxed)};
        const Vec3 b_w = quat_rotate(pose_world, b_l);
        const Vec3 a_w = quat_rotate(cam, Vec3{1.0f, 0.0f, 0.0f});
        const Vec3 ax{b_w.y * a_w.z - b_w.z * a_w.y, b_w.z * a_w.x - b_w.x * a_w.z, b_w.x * a_w.y - b_w.y * a_w.x};
        const float dt = b_w.x * a_w.x + b_w.y * a_w.y + b_w.z * a_w.z;
        Quat dq{ax.x, ax.y, ax.z, 1.0f + dt};
        const float dn = std::sqrt(dq.x*dq.x + dq.y*dq.y + dq.z*dq.z + dq.w*dq.w);
        // Only a SMALL correction is legitimate here (the two paths agree to a few degrees). A
        // large one means the measured axis is wrong or the aim is mid-flick a frame behind the
        // hand; either way, leave the pose alone rather than swing the gun onto a bad target.
        if (dn > 1.0e-6f && dt > 0.906f) {   // < 25 degrees
            dq.x /= dn; dq.y /= dn; dq.z /= dn; dq.w /= dn;
            pose_world = quat_mul(dq, pose_world);
        }
    }
    tr.pose_barrel[0] = pose_world.x; tr.pose_barrel[1] = pose_world.y;
    tr.pose_barrel[2] = pose_world.z; tr.pose_barrel[3] = pose_world.w;
    // ---- THE WEAPON'S VISUAL TRIM, after the lock: rotate the model relative to the ray by the
    // held weapon's delta, in the pose's own frame (a right-multiply, so it rides the wrist).
    // The shot still goes along the reticle; only how the model sits on that line changes --
    // "the AR needs to pitch up" is exactly this. Not while frozen/lingering: the latched world
    // pose already contains it. The Home solve measures its delta against THIS pose, so it
    // composes onto the weapon rotation by the same right-multiply rule.
    // Ungated for the same reason as the lock above: the latch must be the rendered pose. Gated,
    // a non-zero weapon trim was stripped at key-down and restored at release -- the "gun rolls one
    // way when I hold, the other way when I let go" report, whose size is exactly this trim.
    {
        // GLOBAL ROLL TRIM (paletterolltrim, degrees, + = clockwise seen from behind the gun): a
        // fixed roll about the barrel -- the pose's +X -- for every weapon. The global grip capture
        // keeps pitch only, so this is the one place a deliberate global roll lives; the per-weapon
        // trim below can add its own on top. Live-tunable from the config.
        if (g_cfg.palette_roll_trim != 0.0f) {
            const float h = g_cfg.palette_roll_trim * 0.5f * DEG2RAD;
            pose_world = quat_unit(quat_mul(pose_world, Quat{std::sin(h), 0.0f, 0.0f, std::cos(h)}));
        }
        const Quat wr{g_p_wrot_x.load(std::memory_order_relaxed), g_p_wrot_y.load(std::memory_order_relaxed),
                      g_p_wrot_z.load(std::memory_order_relaxed), g_p_wrot_w.load(std::memory_order_relaxed)};
        pose_world = quat_mul(pose_world, wr);
    }
    tr.pose_trim[0] = pose_world.x; tr.pose_trim[1] = pose_world.y;
    tr.pose_trim[2] = pose_world.z; tr.pose_trim[3] = pose_world.w;

    // ---- THE FREEZE, IN WORLD TERMS, HERE -- not in the publisher.
    //
    // The publisher latched a HEAD-RELATIVE ROOM hand, and this function then lifted it by the
    // LIVE camera every frame. So the "frozen" weapon was frozen relative to the aim camera --
    // and on this build the aim camera follows the hand. In-headset, holding the key: "my camera was
    // changing the position/rotation of it slightly." Exactly: it rode the camera. 0.5's latch is
    // a world freeze for them only because their camera never moves with the hand.
    //
    // A world freeze on a hand-driven camera has to latch AFTER the lift -- the world hand and
    // the world pose -- and hold THOSE while the live camera is divided out of a fixed world
    // point. The weapon then stays where it is in the level no matter what the aim does, which
    // is what "frozen" has to mean for the player to align a controller to it. The solve on
    // release is the same rigid offset as before, taken from world quantities. The publisher's
    // job is now only to say whether the key is held.
    // FREEZE FIX (reviewer): normal placement is camera-free now (.5-parity), which killed the swim
    // but also made the freeze only SKELETON-fixed -- the frozen gun rode the aim, so you could not
    // align to it. Bring the live camera back HERE ONLY: latch the hand/pose in WORLD terms on
    // key-down, then re-derive the camera-relative hand every frame so the world point holds still.
    // ce10246 made hand_cm/pose_world ALREADY world (view-lifted) and comp_rot the live camera,
    // which the divide below applies. Latching comp_rot_live*hand_cm here then applied the camera
    // twice: the "frozen" gun rotated against the aim while the player tried to put a hand on it
    // (18:52 log: three captures wandering 4-9 cm each, ERR 6 -> 11 cm). The world quantities are
    // hand_cm and pose_world themselves; latch and restore them directly.
    const Quat comp_rot_live{0.0f, 0.0f, 0.0f, 1.0f};
    const Quat comp_inv_live = quat_conj(comp_rot_live);
    static bool  s_wf_valid = false;
    static Vec3  s_wf_hand_cm{};   // WORLD position, held fixed while frozen
    static Quat  s_wf_pose{};
    static uint32_t s_wf_frames = 0;
    if (held) {
        if (!s_wf_valid) {
            s_wf_hand_cm = quat_rotate(comp_rot_live, hand_cm);
            s_wf_pose = quat_mul(comp_rot_live, pose_world);
            s_wf_valid = true; s_wf_frames = 0;
            API::get()->log_info("[Halo-CampE-UEVR] PALETTECAL: weapon FROZEN in the WORLD -- move "
                                 "your controller onto it, then release");
        }
        ++s_wf_frames;
        hand_cm = quat_rotate(comp_inv_live, s_wf_hand_cm);
        pose_world = quat_mul(comp_inv_live, s_wf_pose);
    } else if (s_wf_valid) {
        s_wf_valid = false;
        if (s_wf_frames < 20u) {
            API::get()->log_info("[Halo-CampE-UEVR] PALETTECAL: released after %u frames -- too short "
                                 "to be a deliberate match; nothing captured.", s_wf_frames);
        } else {
            // Solve in the CONTROLLER's frame from world quantities: P = live world hand/pose,
            // F = frozen world hand/pose. Same convention as the publisher's apply
            // (translation by the raw pose, then the rotation), same compose. Handed back to the
            // publisher through the pending slot; it composes and persists on its next tick.
            const Vec3 live_world_hand = quat_rotate(comp_rot_live, hand_cm);
            const Quat live_world_pose = quat_mul(comp_rot_live, pose_world);
            const Quat d_rot = quat_mul(quat_conj(live_world_pose), s_wf_pose);
            const Vec3 d_pos_m = quat_rotate(quat_conj(live_world_pose),
                                             Vec3{(s_wf_hand_cm.x - live_world_hand.x) / cm_per_m,
                                                  (s_wf_hand_cm.y - live_world_hand.y) / cm_per_m,
                                                  (s_wf_hand_cm.z - live_world_hand.z) / cm_per_m});
            g_pend_rot = d_rot; g_pend_pos_m = d_pos_m;
            g_pend_valid.store(true, std::memory_order_release);
        }
        // KEEP PUBLISHING F FOR A FEW BUILDS after ANY release -- accepted, rejected, or too
        // short. The solve travels sim -> game thread -> next publish before it is applied; in
        // that gap the live hand takes over and the weapon snaps from F to wherever the
        // uncorrected hand is. Reported as "once I let go it flips and moves far left". Linger on
        // F across the handoff so an accepted match lands without a jump and a rejected one
        // simply eases back instead of snapping.
        s_wf_linger = 6;
    }
    if (s_wf_linger > 0) {
        --s_wf_linger;
        hand_cm = quat_rotate(comp_inv_live, s_wf_hand_cm);
        pose_world = quat_mul(comp_inv_live, s_wf_pose);
    }

    // FIX (reviewer, .5-parity): NO camera lever. .5 places relative to palette[0].position with no
    // parent-origin lever; v0 measured ~0 in every log anyway. With comp_inv == identity and
    // lever == 0, rel_cm is just the recenter-frame hand -- exactly .5's grip_delta before the
    // axis swizzle + /304.8 below.
    const Vec3 lever{0.0f, 0.0f, 0.0f};
    (void)cam; (void)v0;
    const Vec3 rel_cm = quat_rotate(comp_inv, Vec3{hand_cm.x - lever.x,
                                                   hand_cm.y - lever.y,
                                                   hand_cm.z - lever.z});
    // UE cm -> palette units: one Y sign for the axes, and 304.8 cm per unit (10 ft per Blam
    // unit) -- the reference's value, whose pose-matrix validation puts the palette node within
    // 4e-8 m of the 3.048-based prediction. The axis probe once measured ~456 here and a previous
    // revision of this comment argued for it; that reading was the UE weapon socket seen through
    // the engine's first-person primitive scale AND under a second, stale world-transform writer,
    // both since removed. It was never the palette's truth. Configurable only so a clean readback
    // can be re-taken; do not put 456 back on the strength of the old measurement.
    const float u = (g_cfg.palette_units_cm > 50.0f) ? g_cfg.palette_units_cm : 304.8f;
    g_eff_hand = Vec3{rel_cm.x / u, -rel_cm.y / u, rel_cm.z / u};
    g_shr_hand = g_eff_hand;
    tr.rel_cm[0] = rel_cm.x; tr.rel_cm[1] = rel_cm.y; tr.rel_cm[2] = rel_cm.z;
    tr.eff_hand[0] = g_eff_hand.x; tr.eff_hand[1] = g_eff_hand.y; tr.eff_hand[2] = g_eff_hand.z;

    // ---- FREEZETRACE. DIAGNOSTIC ONLY -- writes nothing, changes nothing.
    //
    // While a freeze key is down the WORLD hand is a latched constant, so the ONLY way the rendered
    // gun can still move is the camera: the bone written here is conj(cam_build * M) * frozen_hand,
    // and the game re-applies the camera at RENDER time. If those two cameras differ the
    // cancellation is incomplete and the frozen gun swims by the difference.
    //
    // `cam` here is the BUILD-time camera. The game-thread TRACE line (which runs every tick while
    // held, see Plugin.cpp) reports mesh_rot -- the RENDER-time camera -- and the socket that came
    // out. Line the two streams up and the leak is either visible or ruled out, with no theory.
    // Also prints the latched hand so "the freeze itself drifted" can be checked rather than argued.
    if (held && g_cfg.palette_weapon_log) {
        static uint32_t s_fz = 0;
        if ((s_fz++ % 8u) == 0u) {
            API::get()->log_info(
                "[Halo-CampE-UEVR] FREEZETRACE build=%u cam_build=(p%.2f y%.2f) "
                "frozen_world_hand=(%.2f %.2f %.2f)cm bone=(%.4f %.4f %.4f)u",
                s_wf_frames, (float)cam_pitch, (float)cam_yaw,
                s_wf_hand_cm.x, s_wf_hand_cm.y, s_wf_hand_cm.z,
                g_eff_hand.x, g_eff_hand.y, g_eff_hand.z);
        }
    }

    g_dbg_pose_w_x.store(pose_world.x, std::memory_order_relaxed);
    g_dbg_pose_w_y.store(pose_world.y, std::memory_order_relaxed);
    g_dbg_pose_w_z.store(pose_world.z, std::memory_order_relaxed);
    g_dbg_pose_w_w.store(pose_world.w, std::memory_order_relaxed);

    // Divide the (possibly frozen) world pose by the mesh.
    Quat pose = quat_mul(comp_inv, pose_world);
    const float n = std::sqrt(pose.x*pose.x + pose.y*pose.y + pose.z*pose.z + pose.w*pose.w);
    if (!(n > 1.0e-6f) || !std::isfinite(n)) { whynot::rwp_norm.fetch_add(1, std::memory_order_relaxed); return false; }
    whynot::rwp_ok.fetch_add(1, std::memory_order_relaxed);
    pose.x /= n; pose.y /= n; pose.z /= n; pose.w /= n;
    g_eff_pose = pose;
    g_shr_pose = pose;
    if (tlog) {
        tr.eff_pose[0] = pose.x; tr.eff_pose[1] = pose.y; tr.eff_pose[2] = pose.z; tr.eff_pose[3] = pose.w;
        tr.t_ms = termlog::now_ms();
        termlog::put(tr);
    }

    if (g_cfg.palette_weapon_log) {
        static uint32_t s_s = 0;
        if ((s_s++ % 240u) == 0u) {
            // SELF-TEST, no headset motion required (suggested from the headset, and the right call):
            // push a SYNTHETIC hand -- 50 cm dead ahead of the head, room frame -- through this
            // exact pullback with the LIVE camera and constants, and print where it lands. A
            // correct pullback puts it at bone x = +50/304.8 = +0.164, y ~ 0, z ~ 0, at EVERY
            // camera yaw. If the printed value moves with cam_yaw, the frame is still wrong, and
            // the log says so without anyone waving an arm.
            // WHAT THIS TEST CAN AND CANNOT PROVE. A synthetic hand defined at room yaw zero was
            // "exact at every camera yaw" under the full-camera lift -- and that told us NOTHING
            // about the lock gap, because the synthetic hand never carried the view yaw a real
            // published hand does. Any synthetic input is defined IN a frame, so it cannot judge
            // which frame the lift should assume; only a real hand against the real gun can. This
            // test therefore certifies SCALE and AXIS MAP only (a 50 cm hand -> 0.164 units along
            // the right axis, pitch cancelled). THE LIFT IS JUDGED BY THE TRACE LINE: gun_rel_eye
            // against hand-head, both measured, in the same frame, every second.
            const Vec3 synth_room{50.0f, 0.0f, 0.0f};
            const Vec3 synth_world = quat_rotate(cam_yaw_only, synth_room);
            const Vec3 synth_rel = quat_rotate(comp_inv, Vec3{synth_world.x - lever.x,
                                                              synth_world.y - lever.y,
                                                              synth_world.z - lever.z});
            const Vec3 synth_bone{synth_rel.x / u, -synth_rel.y / u, synth_rel.z / u};
            API::get()->log_info(
                "[Halo-CampE-UEVR] PALETTEPULL bone=(%.3f %.3f %.3f) |%.3f|u cam=(p%.1f y%.1f) "
                "| SELFTEST 50cm-fwd hand -> bone=(%.3f %.3f %.3f) [scale/axis check only: "
                "want ~+0.164 0 0] lever=(%.0f %.0f %.0f)cm",
                g_eff_hand.x, g_eff_hand.y, g_eff_hand.z,
                std::sqrt(g_eff_hand.x*g_eff_hand.x + g_eff_hand.y*g_eff_hand.y +
                          g_eff_hand.z*g_eff_hand.z),
                (float)cam_pitch, (float)cam_yaw,
                synth_bone.x, synth_bone.y, synth_bone.z, lever.x, lever.y, lever.z);
        }
    }
    return true;
}

// ---- THE AXIS PROBE. The last assumption, measured.
//
// Everything else in the pullback is now measured (M, v0, the camera) and their drift reads
// zero -- yet the camera still leaks into the render, and an exact cancellation only leaks
// through a term that was assumed. One assumption remains: that palette bone values map to the
// mesh via one Y-flip at 304.8 cm/unit. This measures the real map: displace node 8 along one
// bone axis at a time, and let the game thread read back which WORLD direction the rendered
// weapon actually moved. Three columns, read not derived -- the same treatment every other
// term eventually needed.
//
// Phase cycle (published for the game-thread sampler): 0 baseline, 1 +X, 2 baseline, 3 +Y,
// 4 baseline, 5 +Z. The displacement is absolute on top of the freshly built stock pose, so
// nothing accumulates.
std::atomic<int> g_probe_phase{-1};
constexpr float kProbeAmt = 0.15f;   // palette units; ~0.3 was clearly visible in the first poke

// `+=` ON PURPOSE, and here is why it is not the accumulation bug this codebase was burned by. The
// stock value is ANIMATED -- it changes every build -- so "latch a baseline and write baseline +
// amount" would freeze the probe against a stale pose and drift from stock. The displacement has
// to be relative to THIS build's fresh stock value, and the only correct way to say that is +=.
// It is bounded because this runs exactly once per build, after the game has rebuilt the node:
// the same invariant apply_after_pose stands on. It is NOT bounded if that invariant breaks --
// so probe mode carries a hard clamp below rather than trusting it.
void probe_apply(PaletteNode* palette, int phase) {
    if ((phase & 1) == 0) return;                 // baselines: leave stock untouched
    const int axis = phase / 2;                   // 0=x 1=y 2=z
    float* p = &palette[8].position.x;
    const float want = p[axis] + kProbeAmt;
    if (!std::isfinite(want) || std::fabs(want) > 0.6f) return;   // arm's reach; refuse to compound
    p[axis] = want;
}

// ---- PALRENDER SNAPSHOT (doctrine at the Config key). Single writer (the sim hook), single
// reader (the render refresh); the seq is odd while the writer is inside.
namespace {
struct RrSnap {
    std::atomic<uint32_t> seq{0};
    PaletteNode stock[2][FP_NODE_COUNT];
    PaletteNode* bank[2] = {nullptr, nullptr};
    int nbanks = 0; int32_t count = 0; uint32_t gen = 0;
    std::atomic<long long> at_ms{0};
    std::atomic<float> cam_y{0.0f}, cam_p{0.0f};   // the camera THIS build embedded against
};
RrSnap g_rr;
long long rr_now_ms() {
    return (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
// ---- PALBUILDGATE=6 PRECOMPOSE (doctrine at the Config key). Runs on the sim thread BEFORE
// g_pose_original, so everything slow -- the pullback resolve, the branch, the arms -- happens
// while the banks still hold OUR last frame. All state is sim-thread-local to the hook call.
struct Pre6 { PaletteNode nodes[2][FP_NODE_COUNT]; int n = 0; int32_t cnt = 0; int32_t tag = 0; bool valid = false; };
Pre6 g_pre6;
// gate 7: true while the current hook call skipped g_pose_original (sim thread only).
bool g_pre7_no_original = false;
// PALETTEFINAL's golden copy: the exact bank bytes of our last landing, seqlocked (writer =
// sim thread at each landing, reader = the render-thread slerp hook).
struct Golden {
    std::atomic<uint32_t> seq{0};
    PaletteNode nodes[FP_NODE_COUNT];
    int32_t count = 0;
    int32_t tag = 0;
    std::atomic<long long> at_ms{0};
};
Golden g_golden;
// PALSNIFF's watched address (live palette node 8), published by the sim hook each build,
// consumed by the sniffer thread defined far below.
std::atomic<uintptr_t> g_sniff_lp{0};
void golden_store(const PaletteNode* src, int32_t cnt, int32_t tag) {
    if (src == nullptr || cnt <= 0 || cnt > FP_NODE_COUNT || tag == 0) return;
    g_golden.seq.fetch_add(1, std::memory_order_acq_rel);
    memcpy(g_golden.nodes, src, sizeof(PaletteNode) * (size_t)cnt);
    g_golden.count = cnt;
    g_golden.tag = tag;
    g_golden.at_ms.store((long long)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count(), std::memory_order_relaxed);
    g_golden.seq.fetch_add(1, std::memory_order_acq_rel);
}
} // namespace
// PALSLOT (2026-09-12). From the headset, unprompted: "when i swap to a different gun, the hand
// alternates between the default pose and our pose, when i swap back to the other gun i loaded
// in with, the pose stays where we placed it". Alternating between stock and ours is not a
// second bug, it is THE judder with the amplitude turned up, and it explains the 90-degree-roll
// amplification for nothing: the further the hand is from the stock pose, the larger the gap
// between the two poses it is flipping between, so the same alternation draws a bigger jump.
//
// The suspect is structural, not mathematical. palette_precompose() and the gate-6 build-time
// landing are BOTH hard-gated on `weapon_slot == 0` (the two call sites below, and the landing
// at the head of apply_after_pose). Every other slot the game poses gets only the late generic
// write, which lands in the window the renderer has already consumed -- the exact window gate 6
// exists to beat. If the drawn weapon is not slot 0, it alternates.
//
// That is a THEORY about which slots the hook actually fires for, and it has never been
// measured. If the hook only ever passes slot 0, the theory is dead and no refactor is owed.
// So: count calls per (player, slot), remember each slot's model tag and node count, and count
// how many of those calls actually received a gate-6 landing. Cheap, sim-thread only, no
// reflected calls, gated behind palettewpnlog.

namespace palslot {
struct Slot { unsigned calls = 0, gate6 = 0, generic = 0; int32_t tag = 0, count = 0; };
Slot g_s[2][4];
std::atomic<unsigned> g_reported{0};
void note_call(int32_t p, int32_t w) {
    if (p < 0 || p > 1 || w < 0 || w > 3) return;
    ++g_s[p][w].calls;
}
void note_gate6(int32_t p, int32_t w) { if (p >= 0 && p <= 1 && w >= 0 && w <= 3) ++g_s[p][w].gate6; }
void note_generic(int32_t p, int32_t w, int32_t tag, int32_t cnt) {
    if (p < 0 || p > 1 || w < 0 || w > 3) return;
    ++g_s[p][w].generic; g_s[p][w].tag = tag; g_s[p][w].count = cnt;
}
void report() {
    char line[512]; int n = 0;
    for (int p = 0; p < 2; ++p) {
        for (int w = 0; w < 4; ++w) {
            const Slot& s = g_s[p][w];
            if (s.calls == 0) continue;
            n += snprintf(line + n, sizeof(line) - (size_t)n,
                          "[p%d w%d calls=%u gate6=%u generic=%u tag=%08X cnt=%d] ",
                          p, w, s.calls, s.gate6, s.generic, (unsigned)s.tag, s.count);
            if (n < 0 || n >= (int)sizeof(line) - 80) { n = (int)sizeof(line) - 80; break; }
        }
    }
    if (n <= 0) return;
    if (g_cfg.palette_weapon_log != 0) {
        API::get()->log_info("[Halo-CampE-UEVR] PALSLOT %hs", line);
        whynot::report();
    }
    for (int p = 0; p < 2; ++p) for (int w = 0; w < 4; ++w) {
        g_s[p][w].calls = g_s[p][w].gate6 = g_s[p][w].generic = 0;
    }
}
} // namespace palslot

static void palette_precompose() {
    g_pre6.valid = false;
    if (!palette_weapon_mode()) return;
    {   // a weapon swap invalidates the snapshot: the original must rebuild it (gate 7 relies
        // on this to let the game's builder run exactly when fresh structures are needed).
        LiveSlot lv{};
        if (live_palette_for(0, 0, &lv) == nullptr) { whynot::pre_nopal.fetch_add(1, std::memory_order_relaxed); return; }
        // WEAPON-SWAP DEADLOCK (2026-09-12, found by the whynot ledger: pre_tag = 320 of 320
        // calls with pre_ok = 0, permanently, from the first weapon swap onward).
        // The intent here is "a weapon swap invalidates the snapshot, let the game's builder
        // rebuild it once". The bug is that the tag was never re-latched on the way out, and the
        // ONLY line that re-latches it lives inside the gate-6 landing block, which needs
        // g_pre6.valid, which needs this function to succeed, which this stale tag forbids. So
        // one weapon swap closed gate 6 for the rest of the session and nothing could reopen it.
        // That is why swapping back to the weapon loaded in with looked fine: its tag is the one
        // still latched. Re-latch here, stay invalid for THIS call so the builder runs and
        // refreshes g_rr, and the next call proceeds normally.
        if (g_pre6.tag != 0 && lv.model_tag != g_pre6.tag) {
            whynot::pre_tag.fetch_add(1, std::memory_order_relaxed);
            g_pre6.tag = lv.model_tag;
            return;
        }
    }
    const uint32_t s0 = g_rr.seq.load(std::memory_order_acquire);
    if (s0 & 1u) { whynot::pre_seq.fetch_add(1, std::memory_order_relaxed); return; }
    const int nb = g_rr.nbanks; const int32_t cnt = g_rr.count;
    if (nb <= 0 || cnt <= 0 || cnt > FP_NODE_COUNT) { whynot::pre_banks.fetch_add(1, std::memory_order_relaxed); return; }
    if (rr_now_ms() - g_rr.at_ms.load(std::memory_order_relaxed) > 60) { whynot::pre_stale.fetch_add(1, std::memory_order_relaxed); return; }
    if (!resolve_world_pullback()) { whynot::pre_pull.fetch_add(1, std::memory_order_relaxed); return; }
    for (int i = 0; i < nb && i < 2; ++i) {
        memcpy(g_pre6.nodes[i], g_rr.stock[i], sizeof(PaletteNode) * (size_t)cnt);
        Mat3 dB{}; NodeMatrix dP{};
        if (!apply_weapon_branch(g_pre6.nodes[i], &dB, &dP)) { whynot::pre_branch.fetch_add(1, std::memory_order_relaxed); return; }
    }
    g_pre6.n = nb; g_pre6.cnt = cnt; g_pre6.valid = true;
    whynot::pre_ok.fetch_add(1, std::memory_order_relaxed);
}
void apply_after_pose(int32_t local_player, int32_t weapon_slot) {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    weapon_object_probe_from_builder();
    if (g_cfg.palette_hook_test <= 0 && !palette_weapon_mode()) return;

    // Probe mode replaces the weapon branch entirely: stock pose plus one clean axis nudge.
    if (g_cfg.palette_probe) {
        LiveSlot live{};
        PaletteNode* palette = live_palette_for(local_player, weapon_slot, &live);
        if (palette == nullptr) return;
        PaletteNode* banks[2] = {nullptr, nullptr};
        const int nbanks = collect_capture_banks(local_player, weapon_slot, live, banks);
        // Clock the phases on SLOT-0 builds only: the hook fires once per weapon slot, so a raw
        // call counter runs the phase clock at slot-count speed -- v2's 20-tick settle window
        // then outlived the entire shortened phase and every sample was discarded as settling
        // (the silent-probe session). Longer phases too: ~4 s each gives the sampler ~100 steady
        // ticks after settle instead of a handful.
        static uint32_t s_n = 0;
        if (weapon_slot == 0) ++s_n;
        const int phase = (int)((s_n / 240u) % 6u);
        g_probe_phase.store(phase, std::memory_order_relaxed);
        probe_apply(palette, phase);
        for (int i = 0; i < nbanks; ++i) probe_apply(banks[i], phase);
        return;
    }

    // palettebuildgate=6: land the PRECOMPOSED banks before anything slow. The exposure window is
    // the game's build return to these memcpys; stock is snapshotted out on the way (the refresh
    // and the next precompose both feed on it).
    bool pre6_done = false;
    if ((g_cfg.pal_build_gate == 6 || g_cfg.pal_build_gate == 7) && palette_weapon_mode() && g_pre6.valid && weapon_slot == 0) {
        LiveSlot plive{};
        PaletteNode* ppal = live_palette_for(local_player, weapon_slot, &plive);
        if (ppal != nullptr && plive.count == g_pre6.cnt) {
            PaletteNode* pbanks[2] = {nullptr, nullptr};
            int pidx[2] = {-1, -1}; int pcur = -1;
            const int pnb = collect_capture_banks(local_player, weapon_slot, plive, pbanks, pidx, &pcur);
            if (pnb > 0) {
                g_rr.seq.fetch_add(1, std::memory_order_acq_rel);   // odd: writing
                g_rr.nbanks = 0;
                for (int i = 0; i < pnb && i < 2; ++i) {
                    if (pbanks[i] == nullptr) continue;
                    // gate 7 with the original skipped: the bank holds OUR last landing, not
                    // stock -- re-snapshotting it would compound the branch on itself. The
                    // stock bytes captured on the last original run stay authoritative.
                    // From the LIVE palette: the bank may hold our own previous landing (measured
                    // ~50% of builds), and snapshotting that as "stock" compounds it every frame.
                    if (!g_pre7_no_original) memcpy(g_rr.stock[g_rr.nbanks], ppal, sizeof(PaletteNode) * (size_t)plive.count);
                    g_rr.bank[g_rr.nbanks] = pbanks[i];
                    ++g_rr.nbanks;
                    const int j = (i < g_pre6.n) ? i : 0;
                    memcpy(pbanks[i], g_pre6.nodes[j], sizeof(PaletteNode) * (size_t)plive.count);
                }
                g_rr.count = plive.count;
                g_pre6.tag = plive.model_tag;
                g_rr.gen = g_p_wpn_gen.load(std::memory_order_relaxed);
                {   // the camera this build embedded (the refresh's delta baseline)
                    double bp = 0.0, by = 0.0;
                    if (read_control_rotation(&bp, &by, nullptr)) { g_rr.cam_y.store((float)by, std::memory_order_relaxed); g_rr.cam_p.store((float)bp, std::memory_order_relaxed); }
                }
                g_rr.at_ms.store(rr_now_ms(), std::memory_order_relaxed);
                g_rr.seq.fetch_add(1, std::memory_order_acq_rel);   // even: stable
                golden_store(g_pre6.nodes[0], plive.count, plive.model_tag);
                pre6_done = true;
                if (g_cfg.palette_weapon_log) palslot::note_gate6(local_player, weapon_slot);
                g_n8_gate6.store(1u, std::memory_order_relaxed);
            }
        }
    }
    if (palette_weapon_mode() && !resolve_world_pullback()) { whynot::aap_pull.fetch_add(1, std::memory_order_relaxed); return; }
    LiveSlot live{};
    PaletteNode* palette = live_palette_for(local_player, weapon_slot, &live);
    if (palette == nullptr) whynot::aap_nopal.fetch_add(1, std::memory_order_relaxed);
    if (g_cfg.palette_weapon_log && palette != nullptr)
        palslot::note_generic(local_player, weapon_slot, live.model_tag, live.count);
    if (palette == nullptr) return;

    PaletteNode* banks[2] = {nullptr, nullptr};
    int bank_idx[2] = {-1, -1};
    int cur_bank = -1;
    const int nbanks = collect_capture_banks(local_player, weapon_slot, live, banks, bank_idx, &cur_bank);
    g_dbg_build_nbanks.store(nbanks, std::memory_order_relaxed);

    // PALRENDER: the banks as the game just built them, before this build writes anything.
    if (!pre6_done && g_cfg.pal_render != 0 && palette_weapon_mode() && weapon_slot == 0) {
        g_rr.seq.fetch_add(1, std::memory_order_acq_rel);   // odd: writing
        g_rr.nbanks = 0;
        if (live.count > 0 && live.count <= FP_NODE_COUNT) {
            for (int i = 0; i < nbanks && i < 2; ++i) {
                if (banks[i] == nullptr) continue;
                memcpy(g_rr.stock[g_rr.nbanks], palette, sizeof(PaletteNode) * (size_t)live.count);
                g_rr.bank[g_rr.nbanks] = banks[i];
                ++g_rr.nbanks;
            }
        }
        g_rr.count = live.count;
        g_rr.gen = g_p_wpn_gen.load(std::memory_order_relaxed);
        {   // the camera this build reads -- the refresh's delta against it IS the judder
            double bp = 0.0, by = 0.0;
            if (read_control_rotation(&bp, &by, nullptr)) { g_rr.cam_y.store((float)by, std::memory_order_relaxed); g_rr.cam_p.store((float)bp, std::memory_order_relaxed); }
        }
        g_rr.at_ms.store(rr_now_ms(), std::memory_order_relaxed);
        g_rr.seq.fetch_add(1, std::memory_order_acq_rel);   // even: stable
    }

    // ---- WRITE-TARGET EVIDENCE, on change only. The respawn defect writes correct data that the
    // renderer never shows: written==hand in the TRACE while socket_in_mesh sits at the stock
    // constant. Every candidate mechanism differs in WHICH ADDRESSES the resolves hand back after
    // the respawn, so print them -- palette, both banks, and the hook's own arguments. A weapon
    // swap and a respawn each print once; steady play prints nothing.
    if (g_cfg.palette_weapon_log) {
        static uintptr_t s_lp_pal = 0, s_lp_b0 = 0, s_lp_b1 = 0;
        static int32_t   s_lp_args = -1;
        static uint32_t  s_lp_lines = 0;   // two builders ALTERNATING would flood; 60 changes prove it
        const int32_t args_now = (local_player << 8) | weapon_slot;
        if (s_lp_lines < 60 &&
            ((uintptr_t)palette != s_lp_pal || (uintptr_t)banks[0] != s_lp_b0 ||
             (uintptr_t)banks[1] != s_lp_b1 || args_now != s_lp_args)) {
            ++s_lp_lines;
            s_lp_pal = (uintptr_t)palette; s_lp_b0 = (uintptr_t)banks[0];
            s_lp_b1 = (uintptr_t)banks[1]; s_lp_args = args_now;
            API::get()->log_info(
                "[Halo-CampE-UEVR] PALETTEADDR lp=%d slot=%d palette=0x%llX banks=0x%llX,0x%llX "
                "tag=0x%08X count=%d",
                local_player, weapon_slot, (unsigned long long)(uintptr_t)palette,
                (unsigned long long)(uintptr_t)banks[0], (unsigned long long)(uintptr_t)banks[1],
                (unsigned)live.model_tag, live.count);
        }
    }

    // ---- POSE HOLD (see g_pose_hold_until): silence the animation under the displacement.
    {
        // PER SLOT. The hook fires once per weapon slot with a different model tag each, so one
        // shared snapshot keyed on tag re-snapshotted on every call and never wrote a frame
        // (2026-09-02: "the animation is still playing out").
        struct PoseHold { PaletteNode hold[FP_NODE_COUNT]; bool valid; int32_t tag; uint32_t writes; };
        static PoseHold s_ph[4] = {};
        PoseHold& ph = s_ph[(weapon_slot < 0) ? 0 : ((weapon_slot > 3) ? 3 : weapon_slot)];
        const long long until = (std::max)(g_pose_hold_until.load(std::memory_order_acquire),
                                           g_reload_pose_hold_until.load(std::memory_order_acquire));
        const long long nowt  = std::chrono::steady_clock::now().time_since_epoch().count();
        if (until != 0 && nowt < until && live.count == FP_NODE_COUNT) {
            if (!ph.valid || ph.tag != live.model_tag) {
                memcpy(ph.hold, palette, sizeof(ph.hold));
                ph.valid = true;
                ph.tag   = live.model_tag;
                ph.writes = 0;
            } else {
                memcpy(palette, ph.hold, sizeof(ph.hold));
                for (int i = 0; i < nbanks; ++i) memcpy(banks[i], ph.hold, sizeof(ph.hold));
                ++ph.writes;
            }
        } else if (ph.valid) {
            ph.valid = false;
            API::get()->log_info("[Halo-CampE-UEVR] POSEHOLD slot=%d tag=0x%08X: wrote %u frames (banks this frame=%d)",
                                 weapon_slot, (unsigned)ph.tag, ph.writes, nbanks);
        }
    }

    // ---- THE WEAPON BRANCH.
    //
    // Applied to the live palette AND to every render bank. The live copy is what the sim reasons
    // about; the banks are what get drawn. Transforming one without the other is the split that
    // makes a gun render in one place and shoot from another.
    if (palette_weapon_mode()) {
        // ---- THE STOCK REFERENCE (2026-09-12). From the headset, on palettebuildgate=0: "the gun stays on my
        // hand, the arm though goes between default and ours".
        //
        // The delta is a property of (stock, desired) and nothing else. It must be measured
        // against the pose THE GAME BUILT, once, and then applied to every buffer. The bank call
        // below used to pass the LIVE PALETTE as its reference -- but the live palette has already
        // been written by the call above it, so basis_of(ref[8]) was our own desired basis and the
        // delta came back as EXACTLY identity and EXACTLY zero, and those then overwrote dB and dP.
        // The weapon survived it because writing the same pose twice is idempotent, but anything
        // else placed from that delta alternated between our pose and the default one.
        //
        // 76 nodes x 52 bytes is under 4 KB, on the sim thread's stack, so no shared buffer and no
        // second thread can touch it.
        PaletteNode stock_ref[FP_NODE_COUNT];
        const bool have_ref = (live.count > 0 && live.count <= FP_NODE_COUNT);
        if (have_ref) memcpy(stock_ref, palette, sizeof(PaletteNode) * (size_t)live.count);
        const PaletteNode* SREF = have_ref ? stock_ref : nullptr;

        Mat3 dB{}; NodeMatrix dP{};
        bool ok = apply_weapon_branch(palette, &dB, &dP, SREF);
        g_live_pal_addr.store((uintptr_t)palette, std::memory_order_relaxed);
        bool arms_ok = ok;   // the weapon branch alone
        // PALETTELERP: two real endpoints for the renderer's blend. The fresh pose goes into the
        // bank the capture context names current; the other bank gets the full palette this hook
        // wrote one build ago for this slot (snapshotted AFTER the branch and the arms, so it is
        // exactly what was drawn last tick). Falls back to fresh-in-both when the snapshot is
        // missing or describes another weapon. Whether the context's bank index really alternates
        // build to build is COUNTED (flips vs same) and logged, not assumed.
        struct LerpSnap { int32_t tag; int32_t count; bool valid; PaletteNode nodes[FP_NODE_COUNT]; };
        static LerpSnap s_lerp[4][2] = {};
        static int      s_lerp_prev_cur[4][2] = {{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}};
        static uint32_t s_lerp_flip = 0, s_lerp_same = 0, s_lerp_fallback = 0, s_lerp_builds = 0;
        LerpSnap& snap = s_lerp[local_player][weapon_slot];
        const bool lerp = g_cfg.palette_lerp && nbanks == 2 && (cur_bank == 0 || cur_bank == 1) &&
                          live.count > 0 && live.count <= FP_NODE_COUNT;
        int fresh_i = -1;
        // palettebuildgate=5 (doctrine at the Config key): the banks belong to the per-frame
        // refresh alone; the live palette and the stock snapshot above still happen, so the
        // refresh keeps its inputs and the sim keeps its pose.
        const bool bg_banks = !pre6_done && !(g_cfg.pal_build_gate == 5 && g_cfg.pal_render == 1);
        for (int i = 0; bg_banks && i < nbanks; ++i) {
            const bool is_cur = (bank_idx[i] == cur_bank);
            if (lerp && !is_cur && snap.valid && snap.tag == live.model_tag && snap.count == live.count) {
                memcpy(banks[i], snap.nodes, sizeof(PaletteNode) * (size_t)live.count);
                continue;
            }
            // PALETTEBANK (lerp off): fresh pose into the chosen bank(s) only; the other is left
            // exactly as the game built it, so the effect of each bank can be seen on its own.
            if (!lerp) {
                const int m = g_cfg.palette_bank;
                const bool write_this = (m < 0) || (m == 0 && bank_idx[i] == 0) || (m == 1 && bank_idx[i] == 1) ||
                                        (m == 2 && is_cur) || (m == 3 && !is_cur);
                if (!write_this) continue;
            }
            // Against the STOCK snapshot, never against the live palette we just wrote. And into
            // LOCAL out-params, so a bank call can never clobber the live palette's delta.
            Mat3 bdB{}; NodeMatrix bdP{};
            const bool bok = apply_weapon_branch(banks[i], &bdB, &bdP, SREF);
            ok = bok && ok;
            if (is_cur) fresh_i = i;
            else if (lerp) ++s_lerp_fallback;
        }
        if (!pre6_done && ok && nbanks > 0 && banks[0] != nullptr && weapon_slot == 0)
            golden_store(banks[0], live.count, live.model_tag);
        if (lerp && fresh_i >= 0 && ok) {
            memcpy(snap.nodes, banks[fresh_i], sizeof(PaletteNode) * (size_t)live.count);
            snap.tag = live.model_tag; snap.count = live.count; snap.valid = true;
        } else {
            snap.valid = false;
        }
        if (lerp) {
            ++s_lerp_builds;
            int& pc = s_lerp_prev_cur[local_player][weapon_slot];
            if (pc >= 0) { if (pc != cur_bank) ++s_lerp_flip; else ++s_lerp_same; }
            pc = cur_bank;
            if (g_cfg.palette_weapon_log && (s_lerp_builds % 512u) == 0u) {
                API::get()->log_info("[Halo-CampE-UEVR] PALETTELERP builds=%u cur-bank flips=%u same=%u fresh-both-fallback=%u (cur now %d)",
                                     s_lerp_builds, s_lerp_flip, s_lerp_same, s_lerp_fallback, cur_bank);
            }
        }
        {
            static uint32_t s_asaid = 0;
            if (s_asaid < 3) { ++s_asaid; API::get()->log_info("[Halo-CampE-UEVR] PALETTEWPN: banks applied=%d count=%d", (int)arms_ok, live.count); }
        }
        static uint32_t s_wsaid = 0;
        if (s_wsaid < 5) {
            ++s_wsaid;
            API::get()->log_info("[Halo-CampE-UEVR] PALETTEWPN: branch applied ok=%d banks=%d",
                                 (int)ok, nbanks);
        }
        // The steady-state landing rate, the number the first-5 lines above cannot give. If b2 is
        // not ~100% of builds, the renderer is blending in stale data and no frame maths can look
        // right -- fix the landing rate first, then judge the maths.
        ++s_bank_builds;
        s_bank_hist[(nbanks < 0) ? 0 : ((nbanks > 2) ? 2 : nbanks)]++;
        if (g_cfg.palette_weapon_log && (s_bank_builds % 512u) == 0u) {
            API::get()->log_info(
                "[Halo-CampE-UEVR] PALETTEBANKS builds=%u b0=%u b1=%u b2=%u "
                "gate_closed=%u tag_miss=%u unreadable=%u",
                s_bank_builds, s_bank_hist[0], s_bank_hist[1], s_bank_hist[2],
                s_bank_gate, s_bank_tagmiss, s_bank_unread);
        }
        if (g_cfg.palette_hook_test <= 0) return;
    }

    int32_t node = g_cfg.palette_poke_node;
    if (node < 0 || node >= FP_NODE_COUNT) node = 0;
    // A RANGE of consecutive nodes (palettepokecount), so a part can be found by halving instead
    // of one node per run: poke half the weapon's children, ask whether the slide moved, halve.
    int32_t count = g_cfg.palette_poke_count;
    if (count < 1) count = 1;
    if (node + count > FP_NODE_COUNT) count = FP_NODE_COUNT - node;

    // ABSOLUTE, NOT ACCUMULATED. This crashed the game once by adding on every pass.
    //
    // From the sim-thread hook the game reset the value between our writes, so `+=` stayed bounded
    // and looked safe. Writing here it is the opposite: we run LAST, so our value is what the next
    // frame builds from, and the delta compounds -- 10, 20, 30 units a frame, thousands of metres
    // within a second, until the sim computed a bounds index from it and faulted with RCX negative.
    //
    // So: latch what the game produced the first time we saw it, and write that baseline plus the
    // offset every time. Bounded by construction, no matter how many frames run. One baseline per
    // node, keyed on the palette address, so a range re-latches when the palette moves.
    static uintptr_t s_base_pal = 0;
    static float     s_base_y[FP_NODE_COUNT] = {};
    if (s_base_pal != (uintptr_t)palette) {
        s_base_pal = (uintptr_t)palette;
        for (int32_t k = 0; k < FP_NODE_COUNT; ++k) s_base_y[k] = palette[k].position.y;
    }

    int written = 0;
    for (int32_t k = node; k < node + count; ++k) {
        const float want = s_base_y[k] + g_cfg.palette_poke_amt;
        // REFUSE ANYTHING ABSURD. A first-person node lives within arm's reach of the view root,
        // so a coordinate far outside that is not a displacement the sim was built to survive --
        // and it is exactly what the crash was made of.
        if (!std::isfinite(want) || std::fabs(want) > 8.0f) continue;
        palette[k].position.y = want;
        // AND THE RENDER BANKS, which is the write that actually shows.
        for (int i = 0; i < nbanks; ++i) banks[i][k].position.y = want;
        ++written;
    }

    static uint32_t s_said = 0;
    static int32_t  s_said_node = -1, s_said_count = -1;
    if (s_said < 5 || node != s_said_node || count != s_said_count) {
        ++s_said; s_said_node = node; s_said_count = count;
        API::get()->log_info(
            "[Halo-CampE-UEVR] PALETTEHOOK: player=%d slot=%d nodes=%d..%d (%d written) amt=%.2f  capture banks written=%d",
            local_player, weapon_slot, node, node + count - 1, written, g_cfg.palette_poke_amt, nbanks);
    }
}

// Last time the game built the LOCAL player's first-person palette, steady-clock ms. The build
// routine only runs while a first-person weapon is actually being rendered, which makes it a
// POSITIVE weapon-presence signal -- stronger than any rig or route resolve, and available even
// with the rig driver off (rig=0), which is the shipping configuration once the palette owns
// weapon placement. Consumed by the stick-mode detector.
std::atomic<int64_t> g_fp_built_ms{0};


// ---- OBSCAM (2026-09-12). THE GAME'S OWN CAMERA, read directly.
//
// Static RE of the first-person poser (sim+0x46A2E0) found that the root node it composes every
// bone with is built from a per-player OBSERVER CAMERA record, rec = *(TLS+0x4E8) + p*0x410:
// forward at rec+0x17C, up at rec+0x188, position at rec+0x154. That record is rebuilt every tick
// by sim+0x233370 from the player's control angles (floats at *(TLS+0xB8) + p*0xB8 + 0x6F4/+0x6F8),
// but NOT by assignment: it is pulled toward them through an interpolation (sim+0x235840, called
// at 0x233442). A camera that EASES toward the aim leaves a fixed fraction of every aim step
// behind each tick -- and SAMEINST measured the drawn weapon picking up ~0.7 of each frame's aim
// step as a jump. Our divisor comes from ControlRotation / the UE mesh read, which is the aim
// itself, not this eased camera. The two agree only when the aim is still.
//
// This reads the record on the SIM thread (TLS is per-thread, so the render-thread probes cannot
// see it) right after the game's own builder has run this tick, beside ControlRotation read at
// the same moment. In a shared yaw/pitch description it measures the GAP between the eased camera
// and the aim, and whether that gap tracks the aim step (first-order lag) and with what slope.
// A gap that follows the aim step closely IS the smoothing, and the fix is to divide by this
// camera instead of by the aim.
static void obscam_capture() {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    if (!g_cfg.palette_weapon_log && g_cfg.palette_cam != 13) return;
    const HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
    if (sim == nullptr) return;
    const uintptr_t tls_array = (uintptr_t)__readgsqword(0x58);
    const uint32_t tls_index = *(const uint32_t*)((uintptr_t)sim + RVA_TLS_INDEX);
    if (!readable((const void*)(tls_array + (uintptr_t)tls_index * 8), 8)) return;
    const uintptr_t block = *reinterpret_cast<const uintptr_t*>(tls_array + (uintptr_t)tls_index * 8);
    if (block == 0 || !readable((const void*)(block + 0x4E8), 8)) return;
    const uintptr_t recbase = *reinterpret_cast<const uintptr_t*>(block + 0x4E8);
    if (recbase == 0 || !readable((const void*)(recbase + 0x154), 0x40)) return;
    const float* f = reinterpret_cast<const float*>(recbase + 0x17C);
    const float fx = f[0], fy = f[1], fz = f[2];
    const float fl = std::sqrt(fx*fx + fy*fy + fz*fz);
    if (!(fl > 0.5f) || !std::isfinite(fl)) return;
    float ca0 = 0.0f, ca1 = 0.0f; bool ca_ok = false;
    if (readable((const void*)(block + 0xB8), 8)) {
        const uintptr_t cab = *reinterpret_cast<const uintptr_t*>(block + 0xB8);
        if (cab != 0 && readable((const void*)(cab + 0x6F4), 8)) {
            ca0 = *reinterpret_cast<const float*>(cab + 0x6F4);
            ca1 = *reinterpret_cast<const float*>(cab + 0x6F8);
            ca_ok = std::isfinite(ca0) && std::isfinite(ca1);
        }
    }
    double cp = 0.0, cy = 0.0;
    if (!read_control_rotation(&cp, &cy, nullptr)) return;

    // eased camera as yaw/pitch (Blam: z up). Both yaw sign conventions are tracked, because the
    // UE<->Blam Y flip can negate yaw; the convention whose gap has the smaller spread is the real one.
    const float oyaw = std::atan2(fy / fl, fx / fl) * RAD2DEG;
    const float opit = std::asin(std::fmax(-1.0f, std::fmin(1.0f, fz / fl))) * RAD2DEG;
    auto wrp = [](double d) { while (d > 180.0) d -= 360.0; while (d < -180.0) d += 360.0; return d; };

    struct Acc { double x=0,y=0,xx=0,yy=0,xy=0; uint32_t n=0;
                 void add(double a,double b){++n;x+=a;y+=b;xx+=a*a;yy+=b*b;xy+=a*b;}
                 double corr() const { if(n<10)return 0; const double N=n; const double vx=xx-x*x/N,vy=yy-y*y/N,cv=xy-x*y/N;
                                       return (vx>1e-12&&vy>1e-12)?cv/std::sqrt(vx*vy):0; }
                 double slope() const { if(n<10)return 0; const double N=n; const double vx=xx-x*x/N,cv=xy-x*y/N; return vx>1e-12?cv/vx:0; }
                 double sdy() const { if(n<2)return 0; const double N=n; const double v=yy-y*y/N; return v>0?std::sqrt(v/N):0; } };
    static bool s_h = false;
    static double s_pcy = 0.0, s_pcp = 0.0, s_pgyA = 0.0, s_pgyB = 0.0, s_pgp = 0.0;
    static Acc gyA_step, gyB_step, gp_step, gapchg_yA, gapchg_yB, gapchg_p;
    static uint32_t s_n = 0;
    static double s_sum_ca0 = 0, s_sum_ca1 = 0, s_sum_cp = 0, s_sum_cy = 0;
    const double gapYA = wrp((double)oyaw - cy);          // yaw, same sign
    const double gapYB = wrp(-(double)oyaw - cy);         // yaw, flipped sign
    const double gapP  = (double)opit - cp;
    // Publish for MODE 13. The flipped yaw convention is the measured one: OBSCAM picked it in every
    // window, and the raw control angle at +0x6F4 is -yaw in radians to three figures. A 2-second
    // EMA of each gap is subtracted so only the lag (the part that follows the aim step) is applied.
    {
        static bool s_eh = false; static double s_my = 0.0, s_mp = 0.0;
        if (!s_eh) { s_my = gapYB; s_mp = gapP; s_eh = true; }
        const double a = 1.0 / 90.0;                 // ~2 s at the ~45 Hz tick
        s_my += a * wrp(gapYB - s_my);
        s_mp += a * (gapP - s_mp);
        g_obs_gap_y.store((float)wrp(gapYB - s_my), std::memory_order_relaxed);
        g_obs_gap_p.store((float)(gapP - s_mp), std::memory_order_relaxed);
        g_obs_ms.store((long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count(), std::memory_order_relaxed);
    }
    if (!g_cfg.palette_weapon_log) return;
    if (s_h) {
        const double dy = wrp(cy - s_pcy), dp = cp - s_pcp;
        // first-order lag: the gap tracks the aim STEP. slope ~ -(fraction of the step left behind)
        gyA_step.add(dy, gapYA); gyB_step.add(dy, gapYB); gp_step.add(dp, gapP);
        // and the gap's own change per tick, against the step (the jump the gun would inherit)
        gapchg_yA.add(dy, wrp(gapYA - s_pgyA)); gapchg_yB.add(dy, wrp(gapYB - s_pgyB)); gapchg_p.add(dp, gapP - s_pgp);
        // ---- CAMBONE (2026-09-12). THE ONE TERM THAT SURVIVES INTO THE RENDERED BONES.
        // Mode 13 cancelled the eased camera and the gun did not change (published correction only
        // +-0.04..0.14 deg against aim steps of 1.2-2.1), because static RE shows that camera never
        // reaches the renderer: the consumer sim+0x1A2E0 converts every composed node to
        // parent-local, local = inverse(parent_world) x world, which cancels the camera root A
        // exactly for child nodes. What survives, on the ROOT node, is inverse(X), where X is a
        // camera BONE: the bank blend of the camera node (sim+0x23DE70) or, when that fails,
        // slot+0x28F8 -- written by 0x46EC20 (our hooked function) at 0x46F9A5, which also sets
        // flag 0x10 on slot+0x38 that arms the correction. So whatever X does per tick lands on
        // the drawn rig, between the UE component and the bone we wrote, which is exactly where
        // SAMEINST's residual lives. This reads X, the flag and the gate every sim tick and asks
        // the only question that matters: does X rotate with the aim step?
        {
            static bool s_xh = false;
            static float s_x0[3] = {0,0,0}, s_x1[3] = {0,0,0}, s_x2[3] = {0,0,0};
            static Acc x_step;
            static uint32_t s_xn = 0, s_flag = 0, s_gate = 0, s_rd = 0;
            static double s_xs = 0.0; static float s_xmx = 0.0f;
            if (readable((const void*)(block + 0x4F8), 8)) {
                const uintptr_t slotbase = *reinterpret_cast<const uintptr_t*>(block + 0x4F8);
                if (slotbase != 0 && readable((const void*)(slotbase + 0x28F4), 0x3C) && readable((const void*)(slotbase + 0x38), 1)) {
                    ++s_rd;
                    const uint8_t fl = *reinterpret_cast<const uint8_t*>(slotbase + 0x38);
                    const int32_t gi = *reinterpret_cast<const int32_t*>(slotbase + 0x28F4);
                    if (fl & 0x10) ++s_flag;
                    if (gi != -1) ++s_gate;
                    const float* xn = reinterpret_cast<const float*>(slotbase + 0x28F8);
                    const float r0[3] = {xn[1], xn[2], xn[3]};
                    const float r1[3] = {xn[4], xn[5], xn[6]};
                    const float r2[3] = {xn[7], xn[8], xn[9]};
                    auto nr = [](const float* v) { return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); };
                    if (nr(r0) > 0.5f && nr(r1) > 0.5f && nr(r2) > 0.5f && std::isfinite(r0[0])) {
                        if (s_xh) {
                            const float tr3 = (r0[0]*s_x0[0] + r0[1]*s_x0[1] + r0[2]*s_x0[2]) / (nr(r0)*nr(s_x0))
                                            + (r1[0]*s_x1[0] + r1[1]*s_x1[1] + r1[2]*s_x1[2]) / (nr(r1)*nr(s_x1))
                                            + (r2[0]*s_x2[0] + r2[1]*s_x2[1] + r2[2]*s_x2[2]) / (nr(r2)*nr(s_x2));
                            float cc = (tr3 - 1.0f) * 0.5f;
                            if (cc > 1.0f) cc = 1.0f; if (cc < -1.0f) cc = -1.0f;
                            const float xs = std::acos(cc) * RAD2DEG;
                            const double astep = std::sqrt(dy*dy + dp*dp);
                            x_step.add(astep, xs);
                            ++s_xn; s_xs += xs; if (xs > s_xmx) s_xmx = xs;
                            if ((s_xn % 120u) == 0u) {
                                API::get()->log_info(
                                    "[Halo-CampE-UEVR] CAMBONE X (slot+0x28F8) per-tick rotation mean %.4f worst %.4f deg | "
                                    "corr(X step, AIM step) %+.3f slope %+.3f | flag 0x10 set %.1f%%  gate(+0x28F4!=-1) %.1f%% "
                                    "of %u reads  [X rotating with the aim AND flag+gate on = inverse(X) is what the "
                                    "renderer leaves on our bone; X still = the carrier is elsewhere]",
                                    s_xs / s_xn, s_xmx, x_step.corr(), x_step.slope(),
                                    s_rd ? 100.0 * s_flag / s_rd : 0.0, s_rd ? 100.0 * s_gate / s_rd : 0.0, s_rd);
                                x_step = Acc(); s_xn = 0; s_xs = 0.0; s_xmx = 0.0f; s_flag = s_gate = s_rd = 0;
                            }
                        }
                        for (int i = 0; i < 3; ++i) { s_x0[i] = r0[i]; s_x1[i] = r1[i]; s_x2[i] = r2[i]; }
                        s_xh = true;
                    }
                }
            }
        }
        ++s_n;
        s_sum_cp += cp; s_sum_cy += cy;
        if (ca_ok) { s_sum_ca0 += ca0; s_sum_ca1 += ca1; }
        if ((s_n % 120u) == 0u) {
            const bool useA = gyA_step.sdy() <= gyB_step.sdy();
            const Acc& gy = useA ? gyA_step : gyB_step;
            const Acc& gc = useA ? gapchg_yA : gapchg_yB;
            API::get()->log_info(
                "[Halo-CampE-UEVR] OBSCAM eased-camera minus aim (sim tick, same instant): YAW[%hs sign] gap sd %.3f deg, "
                "corr(gap, aim step) %+.3f slope %+.3f, corr(gap change, aim step) %+.3f slope %+.3f | PITCH gap sd %.3f, "
                "corr(gap, aim step) %+.3f slope %+.3f, corr(gap change, aim step) %+.3f slope %+.3f | means: "
                "ctrlrot p %.2f y %.2f, raw angles +0x6F4 %.4f +0x6F8 %.4f, published lag y %+.3f p %+.3f  [a gap that tracks the aim step with a "
                "steady slope IS the camera's smoothing lag, and is the term to divide by]",
                useA ? "same" : "flipped", gy.sdy(), gy.corr(), gy.slope(), gc.corr(), gc.slope(),
                gp_step.sdy(), gp_step.corr(), gp_step.slope(), gapchg_p.corr(), gapchg_p.slope(),
                s_sum_cp / s_n, s_sum_cy / s_n, s_sum_ca0 / s_n, s_sum_ca1 / s_n,
                g_obs_gap_y.load(std::memory_order_relaxed), g_obs_gap_p.load(std::memory_order_relaxed));
            gyA_step = Acc(); gyB_step = Acc(); gp_step = Acc();
            gapchg_yA = Acc(); gapchg_yB = Acc(); gapchg_p = Acc();
            s_n = 0; s_sum_ca0 = s_sum_ca1 = s_sum_cp = s_sum_cy = 0.0;
        }
    }
    s_pcy = cy; s_pcp = cp; s_pgyA = gapYA; s_pgyB = gapYB; s_pgp = gapP; s_h = true;
}

void hooked_pose(int32_t local_player, int32_t weapon_slot, bool capture_render_palette) {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    if (local_player == 0 && weapon_slot == 0) g_n8_gate6.store(0u, std::memory_order_relaxed);
    if (g_cfg.palette_weapon_log) {
        palslot::note_call(local_player, weapon_slot);
        if ((palslot::g_reported.fetch_add(1, std::memory_order_relaxed) % 320u) == 319u) palslot::report();
    }
    // Point 13, AT HOOK ENTRY on the sim thread (live_palette_for is TLS-bound to this thread;
    // the render-side probe silently returned null): whose bytes the live palette holds RIGHT
    // NOW -- everything that accumulated since our last write, the state any capture copy in
    // between would have sampled.
    if (g_cfg.stomp_log != 0 && local_player == 0 && weapon_slot == 0) {
        LiveSlot lv13{};
        PaletteNode* lp13 = live_palette_for(0, 0, &lv13);
        if (lp13 != nullptr && lv13.count > 8 && !IsBadReadPtr(lp13, sizeof(PaletteNode) * 9)) {
            const float wx = g_dbg_node8_x.load(std::memory_order_relaxed);
            const float wy = g_dbg_node8_y.load(std::memory_order_relaxed);
            const float wz = g_dbg_node8_z.load(std::memory_order_relaxed);
            const float dx = lp13[8].position.x - wx, dy = lp13[8].position.y - wy, dz = lp13[8].position.z - wz;
            stomp_mark(13, lp13[8].position.y, std::sqrt(dx * dx + dy * dy + dz * dz) * 304.8f,
                       0.0f, lp13[8].position.x);
            g_sniff_lp.store((uintptr_t)&lp13[8].position.x, std::memory_order_release);
        }
    }
    // palettebuildgate=6/7: everything slow runs BEFORE the game's build (doctrine at the Config key).
    if ((g_cfg.pal_build_gate == 6 || g_cfg.pal_build_gate == 7) && local_player == 0 && weapon_slot == 0) palette_precompose();
    // gate 7: ONE WRITER. With a valid precompose the game's builder is not called at all for
    // the local FP slot -- stock never enters the banks (doctrine at the Config key).
    g_pre7_no_original = (g_cfg.pal_build_gate == 7 && local_player == 0 && weapon_slot == 0 && g_pre6.valid);
    if (g_pose_original != nullptr && !g_pre7_no_original) {
        g_pose_original(local_player, weapon_slot, capture_render_palette);
        if (local_player == 0 && weapon_slot == 0) obscam_capture();
    }
    if (local_player == 0) {
        g_fp_built_ms.store(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count(), std::memory_order_relaxed);
    }
    // AFTER the original, always. The whole point is to write once the game has finished posing;
    // running first would put us back exactly where the sim-thread poke already is.
    //
    // Bounds first. These are the game's own arguments, but they index a stride-multiplied address,
    // so a value outside the expected range would be read as a structure that is not there.
    if (local_player < 0 || local_player > 3 || weapon_slot < 0 || weapon_slot > 1) return;
    // TIMED. The game-thread PerfScope sites never saw this path (sim thread); a stall here would
    // be a feeling, not a number. QPC around the whole body, published as sum/max/n for the
    // game-thread PERF report to print beside the others.
    // ---- PALBUILDGATE (Config.hpp pal_build_gate): drop a class of build calls, live-flippable.
    bool bg_skip = false;
    {
        const int bg = g_cfg.pal_build_gate;
        if      (bg == 1 && !capture_render_palette) bg_skip = true;
        else if (bg == 2 && weapon_slot != 0)        bg_skip = true;
        else if (bg == 3 && (!capture_render_palette || weapon_slot != 0)) bg_skip = true;
        else if (bg == 4) {
            static unsigned s_bg_last[2] = {0xFFFFFFFFu, 0xFFFFFFFFu};
            const unsigned t = g_tick_id.load(std::memory_order_relaxed);
            if (s_bg_last[weapon_slot] == t) bg_skip = true;
            else s_bg_last[weapon_slot] = t;
        }
    }
    LARGE_INTEGER t0{}, t1{};
    QueryPerformanceCounter(&t0);
    g_dbg_build_nbanks.store(-1, std::memory_order_relaxed);
    if (!bg_skip) apply_after_pose(local_player, weapon_slot);
    QueryPerformanceCounter(&t1);
    // ---- THE WRITE-ORDER STAMP (stomplog points 4..7): every build call lands in the STOMPLOG
    // ring beside the tick-start / stereo-pre mesh samples, so one capture answers, per tick:
    // how many builds ran, in what order against the mesh's transform update, and embedding
    // WHICH camera against the camera the mesh renders under. yaw = the embed camera's yaw,
    // e0 = capture banks found (-1 = gated off or early-out, so pollution shows as a non-capture
    // row with e0 > 0), e1 = the embed camera's pitch.
    if (g_cfg.stomp_log != 0 && local_player == 0) {
        stomp_mark(4 + weapon_slot * 2 + (capture_render_palette ? 0 : 1),
                   g_dbg_build_cam_y.load(std::memory_order_relaxed),
                   bg_skip ? -2.0f : (float)g_dbg_build_nbanks.load(std::memory_order_relaxed),
                   g_dbg_build_cam_p.load(std::memory_order_relaxed),
                   (float)((long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now().time_since_epoch()).count()
                       - g_p_pub_ms.load(std::memory_order_relaxed)));
    }
    // THREAD + PHASE (slidelog/paletteslidewatch): which OS thread this hook runs on, and whether it
    // fires inside the engine tick. If the sim steps on the game thread inside the tick, the
    // node's rebuild, this hook and the mesh sync's read are ORDERED, and "both writes land,
    // nothing renders" is an ordering fact, not a copy.
    if (g_cfg.slide_log || g_cfg.slide_watch) {
        static uint32_t  s_calls = 0, s_in_tick = 0, s_on_game = 0;
        static long long s_said = 0;
        const uint32_t tid = (uint32_t)GetCurrentThreadId();
        ++s_calls;
        if (g_engine_phase.load(std::memory_order_relaxed) == 1) ++s_in_tick;
        if (tid == g_game_tid.load(std::memory_order_relaxed)) ++s_on_game;
        const long long nowh = std::chrono::steady_clock::now().time_since_epoch().count();
        if (nowh - s_said > std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::seconds(5)).count()) {
            s_said = nowh;
            API::get()->log_info("[Halo-CampE-UEVR] SLIDE THREADS: hook tid=%u game tid=%u; on the game thread %u of %u calls; inside the engine tick %u of %u",
                                 tid, g_game_tid.load(std::memory_order_relaxed), s_on_game, s_calls, s_in_tick, s_calls);
        }
    }
    static double s_freq = 0.0;
    if (s_freq == 0.0) { LARGE_INTEGER f{}; QueryPerformanceFrequency(&f); s_freq = (double)f.QuadPart; }
    const double us = (double)(t1.QuadPart - t0.QuadPart) * 1.0e6 / s_freq;
    const uint32_t us_u = (uint32_t)((us < 0.0) ? 0.0 : ((us > 4.0e9) ? 4.0e9 : us));
    g_hook_us_sum.fetch_add(us_u, std::memory_order_relaxed);
    g_hook_n.fetch_add(1, std::memory_order_relaxed);
    uint32_t prev = g_hook_us_max.load(std::memory_order_relaxed);
    while (us_u > prev && !g_hook_us_max.compare_exchange_weak(prev, us_u, std::memory_order_relaxed)) {}
}

} // namespace

std::atomic<uint32_t>  g_game_tid{0};
std::atomic<int>       g_engine_phase{0};

bool blam_palette_fp_live() {
    const int64_t last = g_fp_built_ms.load(std::memory_order_relaxed);
    if (last == 0) return false;
    const int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    // 300 ms: the build runs every sim frame while a weapon renders, so anything recent means
    // "first person, weapon out, right now". Death, seats and cutscenes stop the build within a
    // frame and this goes false 300 ms later -- faster than the enter debounce it informs.
    return (now - last) < 300;
}

bool blam_palette_freeze_active() {
    return g_freeze_key_held.load(std::memory_order_relaxed);
}

void blam_palette_hold_pose(int ms) {
    if (ms <= 0) { g_pose_hold_until.store(0, std::memory_order_release); return; }
    const long long nowt = std::chrono::steady_clock::now().time_since_epoch().count();
    const long long span = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                               std::chrono::milliseconds(ms)).count();
    g_pose_hold_until.store(nowt + span, std::memory_order_release);
}

void blam_palette_publish_poses() {
    // The calibration key, polled here because this already runs on the game thread every frame and
    // the hook must not be reading input.
    // Page Up = GLOBAL freeze-and-align (palgripfix). Home (palettewpncalibkey) = the SAME gesture, but the
    // solve lands in the held weapon's palwpnfix delta instead. Which one started the hold is latched
    // on the rising edge, because by the time the solve arrives the key is up.
    const bool held_global = g_cfg.palette_calib_key != 0 &&
                             (GetAsyncKeyState(g_cfg.palette_calib_key) & 0x8000) != 0;
    const bool held_wpn = g_cfg.pal_wpn_calib_key != 0 &&
                          (GetAsyncKeyState(g_cfg.pal_wpn_calib_key) & 0x8000) != 0;
    const bool held = held_global || held_wpn;
    // PUBLISH THE KEY STATE HERE, BEFORE ANY EARLY RETURN. Read by blam_palette_freeze_active(),
    // which the TRACE cadence consults. Every `return` below -- palette_weapon off, no controller
    // index, the dead-tracking gate -- would otherwise leave this latched TRUE if a controller
    // slept mid-hold, and the diagnostic would then log at full rate forever.
    g_freeze_key_held.store(held, std::memory_order_release);
    static bool s_pend_is_wpn = false;
    static bool s_was_held = false;
    if (held && !s_was_held) s_pend_is_wpn = held_wpn && !held_global;
    s_was_held = held;
    if (!palette_weapon_mode()) { g_p_valid.store(false, std::memory_order_release); return; }
    // POSEFREEZE (doctrine at the Config key): the published pose is left exactly as it is.
    if (g_cfg.pose_freeze != 0 && g_p_valid.load(std::memory_order_acquire)) return;

    // MIRROR the file, both directions. This used to adopt an offset from disk and never let it
    // go: once g_grip_fix_valid was true in memory, deleting gripfix= from the file changed
    // nothing until a restart -- so every "clear" today was clean on disk and dirty in RAM, and
    // the next capture composed onto the ghost. A cleared file must clear the offset; a written
    // file must be adopted verbatim (so a hand-edited or restored file takes effect too).
    if (g_cfg.pal_grip_fix_valid) {
        // Pitch only (see the capture compose below): whatever yaw/roll an older file carries is
        // dropped at adoption, so a hand-edited or legacy gripfix cannot put yaw or roll into the gun.
        g_grip_fix_rot   = quat_twist(quat_unit(Quat{g_cfg.pal_grip_fix[0], g_cfg.pal_grip_fix[1], g_cfg.pal_grip_fix[2], g_cfg.pal_grip_fix[3]}),
                                      Vec3{0.0f, 1.0f, 0.0f});
        g_grip_fix_pos_m = Vec3{g_cfg.pal_grip_fix[4], g_cfg.pal_grip_fix[5], g_cfg.pal_grip_fix[6]};
        g_grip_fix_valid = true;
    } else if (g_grip_fix_valid) {
        g_grip_fix_valid = false;
        g_grip_fix_rot   = Quat{0.0f, 0.0f, 0.0f, 1.0f};
        g_grip_fix_pos_m = Vec3{0.0f, 0.0f, 0.0f};
        API::get()->log_info("[Halo-CampE-UEVR] PALETTECAL: grip offset cleared (removed from the calibration file)");
    }

    const auto hidx = API::VR::get_hmd_index();
    const auto cidx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                          : API::VR::get_right_controller_index();
    if (hidx < 0 || cidx < 0) { g_p_valid.store(false, std::memory_order_release); return; }

    Vec3 hpos{}, gpos{}, apos{}; Quat hq{}, gq{}, aq{};
    if (!get_pose(hidx, &hpos, &hq, /*use_aim=*/false) ||
        !get_pose(cidx, &gpos, &gq, /*use_aim=*/false) ||
        !get_pose(cidx, &apos, &aq, /*use_aim=*/true)) {
        g_p_valid.store(false, std::memory_order_release);
        return;
    }
    // Point 29: the snapshot generation the weapon hand was published from.
    if (g_cfg.stomp_log != 0) stomp_mark(29, (float)pose_latch_last_gen(), (float)g_tick_id.load(std::memory_order_relaxed), 0.0f, 0.0f);
    // PALETTESYNC: replace the controller read with the pair the aim law used, and remember the
    // aim that came with it, so the placement divides by exactly that aim.
    g_p_sync_ok.store(false, std::memory_order_relaxed);
    if (g_cfg.palette_sync) {
        const long long sn = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        for (int i = 0; i < 4; ++i) {
            const uint32_t s0 = g_sync.seq.load(std::memory_order_acquire);
            const long long sms = g_sync.ms.load(std::memory_order_relaxed);
            const Quat saq{g_sync.aqx.load(std::memory_order_relaxed), g_sync.aqy.load(std::memory_order_relaxed),
                           g_sync.aqz.load(std::memory_order_relaxed), g_sync.aqw.load(std::memory_order_relaxed)};
            const Vec3 sap{g_sync.apx.load(std::memory_order_relaxed), g_sync.apy.load(std::memory_order_relaxed),
                           g_sync.apz.load(std::memory_order_relaxed)};
            const Quat sgq{g_sync.gqx.load(std::memory_order_relaxed), g_sync.gqy.load(std::memory_order_relaxed),
                           g_sync.gqz.load(std::memory_order_relaxed), g_sync.gqw.load(std::memory_order_relaxed)};
            const Vec3 sgp{g_sync.gpx.load(std::memory_order_relaxed), g_sync.gpy.load(std::memory_order_relaxed),
                           g_sync.gpz.load(std::memory_order_relaxed)};
            const float scp = g_sync.cp.load(std::memory_order_relaxed);
            const float scy = g_sync.cy.load(std::memory_order_relaxed);
            if ((s0 & 1u) || g_sync.seq.load(std::memory_order_acquire) != s0) continue;
            if (sms != 0 && sn - sms < 60) {
                aq = saq; apos = sap; gq = sgq; gpos = sgp;
                g_p_sync_cp.store(scp, std::memory_order_relaxed);
                g_p_sync_cy.store(scy, std::memory_order_relaxed);
                g_p_sync_ok.store(true, std::memory_order_relaxed);
            }
            break;
        }
    }
    g_p_tru_gx.store(gpos.x, std::memory_order_relaxed);
    g_p_tru_gy.store(gpos.y, std::memory_order_relaxed);
    g_p_tru_gz.store(gpos.z, std::memory_order_relaxed);
    g_p_tru_ax.store(aq.x, std::memory_order_relaxed);
    g_p_tru_ay.store(aq.y, std::memory_order_relaxed);
    g_p_tru_az.store(aq.z, std::memory_order_relaxed);
    g_p_tru_aw.store(aq.w, std::memory_order_relaxed);
    // DEAD-TRACKING GATE. A sleeping controller reads exactly (0,0,0) and passes get_pose, whose
    // validity check rejects only the identity ROTATION. The "hand offset" then becomes the
    // negated head position -- 2.4 m on 2026-08-15 -- and that pose went into the palette,
    // got latched by the calibration freeze, and faulted the game itself. No human hand is 1.5 m
    // from their own head. Fail closed here, before anything downstream can believe it.
    {
        const float dx = gpos.x - hpos.x, dy = gpos.y - hpos.y, dz = gpos.z - hpos.z;
        const float reach2 = dx*dx + dy*dy + dz*dz;
        const bool grip_zero = std::fabs(gpos.x) < 1e-6f && std::fabs(gpos.y) < 1e-6f &&
                               std::fabs(gpos.z) < 1e-6f;
        if (grip_zero || !std::isfinite(reach2) || reach2 > 1.5f * 1.5f) {
            static uint32_t s_dead = 0;
            if ((s_dead++ % 300u) == 0u) {
                API::get()->log_info("[Halo-CampE-UEVR] PALETTEPOSE: refusing a dead/implausible hand "
                                     "(grip_zero=%d reach=%.2f m) -- controller asleep?",
                                     (int)grip_zero, std::sqrt(reach2));
            }
            g_p_valid.store(false, std::memory_order_release);
            return;
        }
    }

    // ---- TREMOR (Config.hpp tremor). An 8 Hz notch on the ORIENTATION, because the measured
    // judder is an 8 Hz peak in the angular step and volitional aiming is the 2 Hz shelf below
    // it. Doctrine and the spectrum are at the Config key. Filters the angular STEP and
    // re-integrates, which is the same as filtering the orientation because both operations are
    // linear, and it keeps the quaternion unit by construction.
    if (g_cfg.tremor != 0) {
        static bool  s_th = false;
        static Quat  s_tprev{0.0f, 0.0f, 0.0f, 1.0f};
        static long long s_tms = 0;
        // biquad state per axis, plus the low-pass state for modes 2 and 3
        static float s_x1[3] = {}, s_x2[3] = {}, s_y1[3] = {}, s_y2[3] = {};
        static float s_lp[3] = {};
        static float s_fs = 0.0f, s_b0 = 1.0f, s_b1 = 0.0f, s_b2 = 0.0f, s_a1 = 0.0f, s_a2 = 0.0f;
        const long long tnow = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const float tdt = (s_tms > 0) ? (float)(tnow - s_tms) * 0.001f : 0.021f;
        s_tms = tnow;
        if (!s_th || !(tdt > 0.0005f && tdt < 0.2f)) {
            s_tprev = aq; s_th = true;
            for (int k = 0; k < 3; ++k) { s_x1[k] = s_x2[k] = s_y1[k] = s_y2[k] = s_lp[k] = 0.0f; }
        } else {
            const float fs = 1.0f / tdt;
            if (std::fabs(fs - s_fs) > 1.0f) {      // recompute only when the rate really moves
                s_fs = fs;
                const float f0 = (g_cfg.tremor_hz < fs * 0.45f) ? g_cfg.tremor_hz : fs * 0.45f;
                const float w0 = 6.2831853f * f0 / fs;
                const float cw = std::cos(w0);
                const float al = std::sin(w0) / (2.0f * g_cfg.tremor_q);
                const float a0 = 1.0f + al;         // RBJ notch
                s_b0 = 1.0f / a0; s_b1 = (-2.0f * cw) / a0; s_b2 = 1.0f / a0;
                s_a1 = (-2.0f * cw) / a0; s_a2 = (1.0f - al) / a0;
            }
            Quat cur = aq;
            if (cur.x * s_tprev.x + cur.y * s_tprev.y + cur.z * s_tprev.z + cur.w * s_tprev.w < 0.0f) {
                cur.x = -cur.x; cur.y = -cur.y; cur.z = -cur.z; cur.w = -cur.w;
            }
            // angular step, degrees, as a vector
            const Quat dq = quat_mul(quat_conj(s_tprev), cur);
            float dw = dq.w; if (dw > 1.0f) dw = 1.0f; if (dw < -1.0f) dw = -1.0f;
            const float sq = 1.0f - dw * dw;        // windows.h defines max(), so no std::max
            const float sn = std::sqrt(sq > 0.0f ? sq : 0.0f);
            float wv[3] = {0.0f, 0.0f, 0.0f};
            if (sn > 1.0e-9f) {
                const float sgn = (dw >= 0.0f) ? 1.0f : -1.0f;
                const float kk = (2.0f * std::acos(std::fabs(dw)) * RAD2DEG) / sn * sgn;
                wv[0] = dq.x * kk; wv[1] = dq.y * kk; wv[2] = dq.z * kk;
            }
            float out[3];
            for (int k = 0; k < 3; ++k) {
                float v = wv[k];
                if (g_cfg.tremor == 1 || g_cfg.tremor == 3) {
                    const float y = s_b0 * v + s_b1 * s_x1[k] + s_b2 * s_x2[k]
                                  - s_a1 * s_y1[k] - s_a2 * s_y2[k];
                    s_x2[k] = s_x1[k]; s_x1[k] = v;
                    s_y2[k] = s_y1[k]; s_y1[k] = y;
                    v = y;
                }
                if (g_cfg.tremor == 2 || g_cfg.tremor == 3) {
                    const float rc = 6.2831853f * g_cfg.tremor_hz * tdt;
                    const float a  = rc / (rc + 1.0f);
                    s_lp[k] += a * (v - s_lp[k]);
                    v = s_lp[k];
                }
                out[k] = std::isfinite(v) ? v : wv[k];
            }
            whynot::trem_seen.fetch_add(1, std::memory_order_relaxed);
            const float rem = std::sqrt((wv[0]-out[0])*(wv[0]-out[0]) + (wv[1]-out[1])*(wv[1]-out[1])
                                      + (wv[2]-out[2])*(wv[2]-out[2]));
            whynot::trem_deg.fetch_add((unsigned)(rem * 1000.0f + 0.5f), std::memory_order_relaxed);
            // re-integrate
            const float on = std::sqrt(out[0]*out[0] + out[1]*out[1] + out[2]*out[2]);
            Quat step{0.0f, 0.0f, 0.0f, 1.0f};
            if (on > 1.0e-9f) {
                const float h = 0.5f * on * DEG2RAD, sh = std::sin(h);
                step = Quat{out[0]/on*sh, out[1]/on*sh, out[2]/on*sh, std::cos(h)};
            }
            const Quat nq = quat_mul(s_tprev, step);
            const float nn = std::sqrt(nq.x*nq.x + nq.y*nq.y + nq.z*nq.z + nq.w*nq.w);
            if (nn > 1.0e-6f && std::isfinite(nn)) {
                aq = Quat{nq.x/nn, nq.y/nn, nq.z/nn, nq.w/nn};
                s_tprev = aq;
            } else {
                s_tprev = cur;
            }
        }
    }

    // ---- REVCLAMP (Config.hpp rev_clamp). Doctrine, numbers and the one-to-one guarantee are
    // at the Config key. Runs BEFORE the one-euro filter so every consumer sees one stream, and
    // before the raws are stored so the recorders show what actually went downstream.
    if (g_cfg.rev_clamp != 0) {
        auto aa_of = [](const Quat& a, const Quat& b) {          // rotation a->b as deg-vector
            const Quat d = quat_mul(quat_conj(a), b);
            float w = d.w; if (w > 1.0f) w = 1.0f; if (w < -1.0f) w = -1.0f;
            const float ss = 1.0f - w * w;   // windows.h defines max(), so no std::max here
            const float s = std::sqrt(ss > 0.0f ? ss : 0.0f);
            if (s < 1.0e-9f) return Vec3{0.0f, 0.0f, 0.0f};
            const float sgn = (w >= 0.0f) ? 1.0f : -1.0f;
            const float k = (2.0f * std::acos(std::fabs(w)) * RAD2DEG) / s * sgn;
            return Vec3{d.x * k, d.y * k, d.z * k};
        };
        auto q_of = [](const Vec3& v) {                          // deg-vector back to a quaternion
            const float n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
            if (n < 1.0e-9f) return Quat{0.0f, 0.0f, 0.0f, 1.0f};
            const float h = 0.5f * n * DEG2RAD, sh = std::sin(h);
            return Quat{v.x / n * sh, v.y / n * sh, v.z / n * sh, std::cos(h)};
        };
        static bool  s_have = false;
        static Quat  s_prev{0.0f, 0.0f, 0.0f, 1.0f};
        static Vec3  s_pstep{0.0f, 0.0f, 0.0f};
        static Vec3  s_hist[3] = {};
        static int   s_nh = 0;
        static long long s_ms = 0;
        const long long now = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const float dt = (s_ms > 0) ? (float)(now - s_ms) * 0.001f : 0.02f;
        s_ms = now;
        if (!s_have || !(dt > 0.0005f && dt < 0.2f)) {
            s_prev = aq; s_have = true; s_nh = 0;
            s_pstep = Vec3{0.0f, 0.0f, 0.0f};
        } else {
            // shortest arc, or a sign flip reads as a 360 degree reversal
            Quat cur = aq;
            if (cur.x * s_prev.x + cur.y * s_prev.y + cur.z * s_prev.z + cur.w * s_prev.w < 0.0f) {
                cur.x = -cur.x; cur.y = -cur.y; cur.z = -cur.z; cur.w = -cur.w;
            }
            Vec3 w = aa_of(s_prev, cur);
            if (g_cfg.rev_clamp == 2 || g_cfg.rev_clamp == 3) {
                s_hist[2] = s_hist[1]; s_hist[1] = s_hist[0]; s_hist[0] = w;
                if (s_nh < 3) ++s_nh;
                if (s_nh == 3) {
                    auto med3 = [](float a, float b, float c) {
                        return (a > b) ? ((b > c) ? b : ((a > c) ? c : a))
                                       : ((a > c) ? a : ((b > c) ? c : b));
                    };
                    w = Vec3{med3(s_hist[0].x, s_hist[1].x, s_hist[2].x),
                             med3(s_hist[0].y, s_hist[1].y, s_hist[2].y),
                             med3(s_hist[0].z, s_hist[1].z, s_hist[2].z)};
                }
            }
            if (g_cfg.rev_clamp == 1 || g_cfg.rev_clamp == 3) {
                const float pn = std::sqrt(s_pstep.x * s_pstep.x + s_pstep.y * s_pstep.y + s_pstep.z * s_pstep.z);
                if (pn > 1.0e-6f) {
                    const Vec3 uh{s_pstep.x / pn, s_pstep.y / pn, s_pstep.z / pn};
                    const float fwd = w.x * uh.x + w.y * uh.y + w.z * uh.z;
                    if (fwd < 0.0f) {
                        const float cap = g_cfg.rev_clamp_dps * dt;
                        const float back = -fwd;
                        if (back > cap) {
                            // remove ONLY the impossible part of the backward component.
                            // Perpendicular motion (real curvature) is untouched by construction.
                            const float excess = back - cap;
                            w.x += uh.x * excess; w.y += uh.y * excess; w.z += uh.z * excess;
                            whynot::rev_hit.fetch_add(1, std::memory_order_relaxed);
                            whynot::rev_deg.fetch_add((unsigned)(excess * 100.0f + 0.5f), std::memory_order_relaxed);
                        }
                    }
                }
            }
            const Quat fixed = quat_mul(s_prev, q_of(w));
            const float fn = std::sqrt(fixed.x * fixed.x + fixed.y * fixed.y + fixed.z * fixed.z + fixed.w * fixed.w);
            if (fn > 1.0e-6f && std::isfinite(fn)) {
                aq = Quat{fixed.x / fn, fixed.y / fn, fixed.z / fn, fixed.w / fn};
                s_pstep = w;
                s_prev = aq;
            } else {
                s_prev = cur;
            }
            whynot::rev_seen.fetch_add(1, std::memory_order_relaxed);
        }
    }

    // ---- THE ONE-EURO POSE FILTER (Config.hpp pose_filter). Before the raws are stored so
    // every consumer -- palette, arms, WPNERR -- sees one consistent, filtered hand.
    if (g_cfg.pose_filter != 0) {
        struct Euro { float x = 0.0f, dx = 0.0f; bool have = false; };
        static Euro s_pe[3];        // grip position
        static Euro s_qe[4];        // aim pose quat
        static Euro s_ge[4];        // grip pose quat
        static long long s_pf_ms = 0;
        const long long pf_now = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        float pf_dt = (s_pf_ms > 0) ? (float)(pf_now - s_pf_ms) * 0.001f : 0.011f;
        s_pf_ms = pf_now;
        if (pf_dt > 0.0005f && pf_dt < 0.25f) {
            auto ealpha = [](float fc, float dt) { const float r = 6.2831853f * fc * dt; return r / (r + 1.0f); };
            auto euro = [&](Euro& e, float v, float beta) {
                if (!e.have) { e.x = v; e.dx = 0.0f; e.have = true; return v; }
                const float dv = (v - e.x) / pf_dt;
                e.dx += ealpha(g_cfg.pose_filter_dcut, pf_dt) * (dv - e.dx);
                const float fc = g_cfg.pose_filter_min + beta * std::fabs(e.dx);
                e.x += ealpha(fc, pf_dt) * (v - e.x);
                return e.x;
            };
            gpos.x = euro(s_pe[0], gpos.x, g_cfg.pose_filter_beta);
            gpos.y = euro(s_pe[1], gpos.y, g_cfg.pose_filter_beta);
            gpos.z = euro(s_pe[2], gpos.z, g_cfg.pose_filter_beta);
            // Quats: sign-continuity first, filter components, renormalize. Valid for the small
            // per-sample steps this filter exists to smooth; large real motion opens the cutoff
            // and passes through barely touched.
            auto qfilt = [&](Euro* e, Quat& q) {
                if (e[3].have) {
                    const float d = q.x * e[0].x + q.y * e[1].x + q.z * e[2].x + q.w * e[3].x;
                    if (d < 0.0f) { q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w; }
                }
                q.x = euro(e[0], q.x, g_cfg.pose_filter_rbeta);
                q.y = euro(e[1], q.y, g_cfg.pose_filter_rbeta);
                q.z = euro(e[2], q.z, g_cfg.pose_filter_rbeta);
                q.w = euro(e[3], q.w, g_cfg.pose_filter_rbeta);
                const float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
                if (n > 1e-6f) { q.x /= n; q.y /= n; q.z /= n; q.w /= n; }
            };
            qfilt(s_qe, aq);
            qfilt(s_ge, gq);
        }
    }
    g_p_raw_gx.store(gpos.x, std::memory_order_relaxed);
    g_p_raw_gy.store(gpos.y, std::memory_order_relaxed);
    g_p_raw_gz.store(gpos.z, std::memory_order_relaxed);
    g_p_raw_ax.store(aq.x, std::memory_order_relaxed);
    g_p_raw_ay.store(aq.y, std::memory_order_relaxed);
    g_p_raw_az.store(aq.z, std::memory_order_relaxed);
    g_p_raw_aw.store(aq.w, std::memory_order_relaxed);
    g_p_pub_ms.store((long long)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count(), std::memory_order_relaxed);
    // Point 9: the publish. yaw = the published aim yaw (room frame), e0 = grip step cm since
    // the last publish, e1 = aim step deg, e2 = the publish interval ms -- tremor, tracking
    // noise and cadence measured where the signal enters the pipeline.
    if (g_cfg.stomp_log != 0) {
        static Vec3 s_sp_pos{}; static Quat s_sp_q{0.0f, 0.0f, 0.0f, 1.0f};
        static long long s_sp_ms = 0; static bool s_sp_have = false;
        const long long sp_now = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        float sp_p = 0.0f, sp_y = 0.0f, sp_r = 0.0f;
        quat_to_rotator(aq.x, aq.y, aq.z, aq.w, &sp_p, &sp_y, &sp_r);
        float step_cm = 0.0f, step_deg = 0.0f, sp_dt = 0.0f;
        if (s_sp_have) {
            const float dx = gpos.x - s_sp_pos.x, dy = gpos.y - s_sp_pos.y, dz = gpos.z - s_sp_pos.z;
            step_cm = std::sqrt(dx * dx + dy * dy + dz * dz) * 100.0f;
            float dq = std::fabs(aq.x * s_sp_q.x + aq.y * s_sp_q.y + aq.z * s_sp_q.z + aq.w * s_sp_q.w);
            if (dq > 1.0f) dq = 1.0f;
            step_deg = 2.0f * std::acos(dq) * 57.2957795f;
            sp_dt = (float)(sp_now - s_sp_ms);
        }
        s_sp_pos = gpos; s_sp_q = aq; s_sp_ms = sp_now; s_sp_have = true;
        stomp_mark(9, sp_y, step_cm, step_deg, sp_dt);
    }

    // ROOM SPACE -> the frame the palette root lives in. Same rotation offset the rig composes
    // against; omitting it is masked by calibration at one yaw and drifts as the body turns.
    Quat q_ro{0.0f, 0.0f, 0.0f, 1.0f};
    if (g_cfg.rig_view_yaw != 0.0f) {
        const auto ro = API::VR::get_rotation_offset();
        q_ro = Quat{ro.x, ro.y, ro.z, ro.w};
        if (g_cfg.rig_view_yaw < 0.0f) q_ro = quat_conj(q_ro);
    }

    // ---- PUBLISH THE WORLD TARGET. The sweep's verdict, built in.
    //
    // The measured failure was a TRANSLATION: the FP mesh rides the rotating camera on a lever,
    // so the bone frame itself sweeps through the world when the aim moves -- metre-scale, and
    // unfixable by rotating bones (hand 1.5 cm, gun 150 cm, on every rotation variant). So stop
    // correcting in bone space. Publish the pose rigmode's PROVEN math produces -- the desired
    // WORLD orientation and the desired offset from the camera-parent's origin, no camera term
    // anywhere -- and let the hook pull it back through the mesh's world transform, rebuilt
    // FRESH at build time from the aim rotator plus the measured constants. Nothing here can go
    // stale, because nothing here contains the camera.
    //
    //   orientation: ue(q_ro * aim_pose)               (aim pose for the remap)
    //   position   : swizzle(q_ro * (grip - hmd)), METRES, UE axes
    //
    // NO q_turn HERE -- and it was here until 2026-08-15, which is why the weapon went behind the
    // player's face after any turn. The rig path composed q_turn (snap/smooth turn + the locked
    // view frame) because it wrote a WORLD transform under a view-locked parent: the parent's
    // frame was the pinned view, so the hand had to be expressed in it. The palette does not
    // embed in the view -- it embeds in the CAMERA, and the pullback in the hook already divides
    // by that camera, which contains the aim yaw. Applying q_turn as well counted the lock gap
    // twice: with viewOut=19 against viewIn=138 (a 119-degree lock gap, live in the log), a hand
    // 30 cm in front published as 30 cm behind. It worked at 13:51 only because the gap was small
    // then. So the target is composed in the camera's own frame: q_ro alone -- the recenter --
    // which is what 0.5 does and what the lock-off runs proved exact.
    // ANCHOR: THE STANDING ORIGIN, NOT THE HEAD. In-headset: "the weapon moves when I move my head;
    // the rig stays still, the aim ray stays still, like it should, but the weapon doesn't." The
    // hand was measured from the HMD (0.5's grip - hmd), so moving the head moved the reference
    // point and the hand offset changed with the head still -- and the gun followed. 0.5 gets away
    // with it because their view IS the camera, so head motion cancels in the embed. Ours does
    // not: the head moves the VIEW, the camera does not follow, and the palette embeds in the
    // camera -- head motion leaks straight into gun position. The rig path never had this: it
    // anchors to the standing origin, a fixed room point, which is why the arms hold still. Same
    // anchor here (it was the original palette design on 8/14, removed chasing 0.5-parity -- the
    // wrong call for a hand-driven camera). Head motion now moves the view freely; the gun stays
    // where the hand is.
    const auto so = API::VR::get_standing_origin();
    const Vec3 anchor{so.x, so.y, so.z};
    const Vec3 d_vr = quat_rotate(q_ro, Vec3{gpos.x - anchor.x, gpos.y - anchor.y, gpos.z - anchor.z});
    Vec3 d_xr{-d_vr.z, d_vr.x, d_vr.y};                                     // UE axes, metres

    // THE AIM FIX BELONGS TO THE HAND, NOT ONLY TO THE RAY.
    //
    // derive_ctrl_angles applies apply_aim_fix before taking the forward vector; publishing the RAW
    // aim pose here left the rendered gun on the uncorrected pose while the shot went down the
    // corrected one. Measured 2026-08-16 with the lock off, 74 still frames: total 8.42 deg (sd
    // 1.68), decomposing as pitch +6.71 / yaw -0.10, and corr(d_yaw, sin(wrist roll)) = 0.908 --
    // the signature of a rotation fixed in the CONTROLLER's frame, which is precisely what aimfix
    // is. Identical on AR and Shotgun. Applying it here took barrel-off-ray from 8.4 to 0.3-1.3 deg.
    //
    // It is also what made the barrel lock have 8 degrees of standing work to do, which the freeze
    // gate above then laundered into gripfix. One source of truth for both halves.
    // THE TWO-HAND HOLD ROTATES THE WEAPON, NOT ONLY THE SHOT. derive_ctrl_angles blends the aim
    // toward the right-to-left hand line while a hold is latched (two_hand_blend); the palette pose
    // never received that rotation, so the gun stayed on the rear hand's own axis while the shot
    // and reticle followed the support hand. Measured 2026-08-17 15:39 (108 clean rows): barrel vs
    // ray 0.13 +/- 0.73 deg outside holds, 6.9 / 31 / 46 deg inside TWOHAND grabbed windows -- and
    // it is why moving the support arm did not move the gun. Same delta the rig path applies to
    // its poses (Plugin.cpp, two_hand_delta): ONE shortest-arc rotation, in VR space, before the
    // recenter, so roll survives and the ray and the gun agree by construction. two_hand_delta
    // measures against the aim-fixed forward, so it composes on the aim-fixed pose here.
    Quat aq_fixed = pal_apply_aim_fix(aq);
    {
        Quat thd{0.0f, 0.0f, 0.0f, 1.0f};
        if (palette_two_hand_delta(&thd)) aq_fixed = quat_unit(quat_mul(thd, aq_fixed));
    }
    const Quat aim_ro = quat_mul(q_ro, aq_fixed);
    Quat pose_xr{-aim_ro.z, aim_ro.x, aim_ro.y, -aim_ro.w};                 // UE convention

    // ---- THE RIGID GRIP OFFSET, applied UPSTREAM to the pose -- 0.5's PoseOffset, not a
    // palette-space constant. Rotation is a right-multiply (a fixed wrist angle at any
    // orientation); translation is expressed in the CONTROLLER's frame and rotated by the live
    // pose, so it rides the wrist -- the rigid attachment. Solved by the pose match below.
    // Everything downstream -- palette, banks, calibration freeze -- sees the corrected hand and
    // never has to know a calibration exists. This is the change that removes the non-rotating
    // pivot the palette-space fix produced.
    // ORDER MATTERS, and it was wrong once: the translation is rotated by the RAW pose, then the
    // rotation is applied -- 0.5's apply_offset exactly. Rotating the translation by the already-
    // corrected pose instead makes apply and solve disagree by precisely the wrist rotation
    // between latching the freeze and releasing the key, which for a "move your hand onto the
    // frozen gun" gesture is never zero: the calibration lands near but not on, wronger the more
    // you rotated to align, and repeat captures fight instead of refining. Caught in review before
    // it cost a headset session.
    // GLOBAL, then the held weapon's delta, composed into ONE rigid offset first (rot = g*w,
    // pos = g.pos + R(g)*w.pos) and applied once by the same rule -- so a weapon with no entry
    // is exactly the global, and the freeze solve below sees a single corrected pose either way.
    const std::string wkey = weapon_key();
    const WeaponFix* wfix = pal_wpnfix_find(wkey);
    {
        static std::string s_last_wkey;
        if (wkey != s_last_wkey) { s_last_wkey = wkey; g_p_wpn_gen.fetch_add(1, std::memory_order_relaxed); }
    }
    // WPNPROBE (stomplog), point 47 once per publish: a stable id for the held weapon and the
    // rotation its trim applies (pitch, yaw, roll, degrees; zero with no entry), so a barrel-to-aim
    // offset can be split per weapon against the calibration that should explain it.
    if (g_cfg.stomp_log != 0) {
        uint32_t wh = 2166136261u;
        for (unsigned char ch : wkey) { wh ^= ch; wh *= 16777619u; }
        const uint32_t wid = wh & 0xFFFFFFu;
        float fpd = 0.0f, fyd = 0.0f, frd = 0.0f;
        if (wfix != nullptr) quat_to_rotator(wfix->q[0], wfix->q[1], wfix->q[2], wfix->q[3], &fpd, &fyd, &frd);
        stomp_mark(47, (float)wid, fpd, fyd, frd);
        static std::string s_probe_wkey = "\x01";
        if (wkey != s_probe_wkey) {
            s_probe_wkey = wkey;
            API::get()->log_info("[Halo-CampE-UEVR] WPNPROBE weapon %s = id %u | trim %s (p%.3f y%.3f r%.3f deg)",
                                 wkey.empty() ? "-" : wkey.c_str(), wid, wfix ? "yes" : "none", fpd, fyd, frd);
        }
    }
    {
        const Quat rot_g = g_grip_fix_valid ? g_grip_fix_rot : Quat{0.0f, 0.0f, 0.0f, 1.0f};
        Vec3 pos_t = g_grip_fix_valid ? g_grip_fix_pos_m : Vec3{0.0f, 0.0f, 0.0f};
        Quat wr{0.0f, 0.0f, 0.0f, 1.0f};
        if (wfix != nullptr) {
            wr = Quat{wfix->q[0], wfix->q[1], wfix->q[2], wfix->q[3]};
            const Vec3 wp = quat_rotate(rot_g, Vec3{wfix->t[0], wfix->t[1], wfix->t[2]});
            pos_t = Vec3{pos_t.x + wp.x, pos_t.y + wp.y, pos_t.z + wp.z};
        }
        if (g_grip_fix_valid || wfix != nullptr) {
            const Vec3 t = quat_rotate(pose_xr, pos_t);   // raw pose
            d_xr = Vec3{d_xr.x + t.x, d_xr.y + t.y, d_xr.z + t.z};
            pose_xr = quat_mul(pose_xr, rot_g);            // GLOBAL rotation only, here
        }
        g_p_offt_x.store(pos_t.x, std::memory_order_relaxed);
        g_p_offt_y.store(pos_t.y, std::memory_order_relaxed);
        g_p_offt_z.store(pos_t.z, std::memory_order_relaxed);
        g_p_rotg_x.store(rot_g.x, std::memory_order_relaxed);
        g_p_rotg_y.store(rot_g.y, std::memory_order_relaxed);
        g_p_rotg_z.store(rot_g.z, std::memory_order_relaxed);
        g_p_rotg_w.store(rot_g.w, std::memory_order_relaxed);
        // The weapon's rotation goes to the pullback, applied after the barrel lock.
        g_p_wrot_x.store(wr.x, std::memory_order_relaxed); g_p_wrot_y.store(wr.y, std::memory_order_relaxed);
        g_p_wrot_z.store(wr.z, std::memory_order_relaxed); g_p_wrot_w.store(wr.w, std::memory_order_relaxed);
    }

    // ---- THE FREEZE LIVES IN THE PULLBACK NOW (resolve_world_pullback), in WORLD terms.
    //
    // It used to be here: latch the head-relative room hand and republish it while held. That is
    // 0.5's design and it is a world freeze for THEM, because their camera never moves with the
    // hand. Ours does -- so a room-frame latch, lifted by the live camera every build, froze the
    // weapon relative to the AIM CAMERA and it rode along with every aim motion. In-headset, key
    // held: "my camera was changing the position/rotation of it slightly." The pullback now
    // latches the WORLD hand and pose after the camera lift and holds those; this function only
    // reports the key and adopts the solve the pullback hands back.
    g_freeze_key_held.store(held, std::memory_order_release);
    if (g_pend_valid.exchange(false, std::memory_order_acq_rel)) {
        const Quat d_rot = g_pend_rot;
        const Vec3 d_pos_m = g_pend_pos_m;
        const float dm = std::sqrt(d_pos_m.x*d_pos_m.x + d_pos_m.y*d_pos_m.y + d_pos_m.z*d_pos_m.z);
        const float ang_deg = 2.0f * std::acos(clampf(std::fabs(d_rot.w), 0.0f, 1.0f)) * RAD2DEG;
        // The 60-degree bound is for REFINEMENTS onto an existing offset. The FIRST capture is
        // allowed a large rotation, because the first thing it absorbs is any constant convention
        // mismatch between where the gun renders and where the maths thinks the hand is -- and on
        // 2026-08-15 every clean, deliberate world-freeze capture solved to ~100 degrees of yaw
        // (98, 110) with the controller placed ON the rendered gun. That is not a dropout; a
        // dropout does not repeat to within 12 degrees. It is a fixed axis convention that only a
        // capture can measure, and the gate was refusing the one measurement that would name it.
        // Position stays bounded at 1 m either way; the 60-degree bound resumes once an offset
        // exists, so a later dropout still cannot corrupt a good calibration.
        const float ang_bound = g_grip_fix_valid ? 60.0f : 175.0f;
        const bool sane = std::isfinite(dm) && dm <= 1.0f && std::isfinite(ang_deg) && ang_deg <= ang_bound;
        if (!sane) {
            API::get()->log_info("[Halo-CampE-UEVR] PALETTECAL: REJECTED a match (moved %.2f m, "
                                 "rotated %.0f deg) -- tracking dropped, or the weapon was mid-jump "
                                 "when the hold caught it. Calibration unchanged.", dm, ang_deg);
        } else if (s_pend_is_wpn) {
            // PER-WEAPON: the solve is a delta in the fully corrected pose's frame (global*weapon),
            // so it composes onto the WEAPON layer by the same rule: w.rot' = w.rot * d_rot,
            // w.pos' = w.pos + R(w.rot) * d_pos. The global is untouched.
            s_pend_is_wpn = false;   // the per-weapon hold's latch, taken by this solve
            if (wkey.empty()) {
                API::get()->log_info("[Halo-CampE-UEVR] PALETTECAL(WPN): no weapon in hand -- nothing written");
            } else {
                Quat wr = wfix ? Quat{wfix->q[0], wfix->q[1], wfix->q[2], wfix->q[3]} : Quat{0.0f, 0.0f, 0.0f, 1.0f};
                Vec3 wp = wfix ? Vec3{wfix->t[0], wfix->t[1], wfix->t[2]} : Vec3{0.0f, 0.0f, 0.0f};
                const Vec3 dp = quat_rotate(wr, d_pos_m);
                wr = quat_mul(wr, d_rot);
                wp = Vec3{wp.x + dp.x, wp.y + dp.y, wp.z + dp.z};
                const float q[4] = {wr.x, wr.y, wr.z, wr.w};
                const float t[3] = {wp.x, wp.y, wp.z};
                pal_wpnfix_set(wkey, q, t);
                float gp = 0.0f, gy = 0.0f, gr = 0.0f;
                quat_to_rotator(wr.x, wr.y, wr.z, wr.w, &gp, &gy, &gr);
                API::get()->log_info(
                    "[Halo-CampE-UEVR] PALETTECAL(WPN) '%s': this match moved %.1f cm, rotated %.0f deg; "
                    "weapon delta rot=(p%.1f y%.1f r%.1f) pos=(%.1f %.1f %.1f)cm -> halo_vr_palette_calib.cfg",
                    wkey.c_str(), dm * 100.0f, ang_deg, gp, gy, gr, wp.x * 100.0f, wp.y * 100.0f, wp.z * 100.0f);
            }
        } else {
            // COMPOSE onto the existing offset (apply = raw.pos + R(raw)*pos, raw.rot*rot):
            //     rot' = rot * d_rot ,  pos' = pos + R(rot) * d_pos.   0.5's compose_offsets.
            const Quat old_rot = g_grip_fix_valid ? g_grip_fix_rot : Quat{0.0f, 0.0f, 0.0f, 1.0f};
            const Vec3 old_pos = g_grip_fix_valid ? g_grip_fix_pos_m : Vec3{0.0f, 0.0f, 0.0f};
            // CONJUGATE THE DELTA BY THE WEAPON TRIM. The global is the FIRST rotation in the chain
            // and the weapon trim is the LAST:
            //     rendered = B * G * W          (B = view-lifted raw pose, G = global, W = weapon)
            // The solve returns d in the RENDERED frame, so d composes onto W exactly (the
            // per-weapon branch above needs no correction). Composing it onto G instead requires
            //     B*G'*W = B*G*W*d   =>   G' = G * (W * d * W^-1)
            // Without the conjugation a Page Up capture lands rotated by the weapon trim, which is
            // why the global never converged while a per-weapon fix was non-zero. Same for the
            // translation: d_pos is measured in the rendered frame and applied one factor earlier.
            const Quat wtrim = wfix ? quat_unit(Quat{wfix->q[0], wfix->q[1], wfix->q[2], wfix->q[3]})
                                    : Quat{0.0f, 0.0f, 0.0f, 1.0f};
            const Quat d_rot_g = quat_mul(quat_mul(wtrim, d_rot), quat_conj(wtrim));
            const Vec3 d_pos_g = quat_rotate(wtrim, d_pos_m);
            const Vec3 d_pos_raw = quat_rotate(old_rot, d_pos_g);
            // THE GLOBAL ROTATION IS PITCH, AND ONLY PITCH. In-headset, 2026-08-16: "the gun should
            // be the same roll and yaw as the controller and aim ray, the pitch however should be
            // able to be calibrated" -- Page Up sets position and pitch, nothing else; Home (the
            // per-weapon delta above) stays a full trim. A freeze-and-align hold cannot report yaw
            // and roll honestly anyway: the 15:39 log's gripfix carried yaw 0.7 and roll -0.8 deg of
            // hand alignment noise, and every one of those degrees is a barrel off the ray. Keep the
            // twist about the pose's +Y (UE pitch axis) of the COMPOSED rotation, so repeat captures
            // refine the pitch and never accumulate the other two.
            g_grip_fix_rot   = quat_twist(quat_unit(quat_mul(old_rot, d_rot_g)), Vec3{0.0f, 1.0f, 0.0f});
            g_grip_fix_pos_m = Vec3{old_pos.x + d_pos_raw.x, old_pos.y + d_pos_raw.y, old_pos.z + d_pos_raw.z};
            g_grip_fix_valid = true;
            float gp = 0.0f, gy = 0.0f, gr = 0.0f;
            quat_to_rotator(g_grip_fix_rot.x, g_grip_fix_rot.y, g_grip_fix_rot.z, g_grip_fix_rot.w,
                            &gp, &gy, &gr);
            API::get()->log_info(
                "[Halo-CampE-UEVR] PALETTECAL: solved rigid grip offset -- this match moved %.1f cm, "
                "rotated %.0f deg; total rot=(p%.1f y%.1f r%.1f) pos=(%.1f %.1f %.1f)cm",
                dm * 100.0f, ang_deg, gp, gy, gr, g_grip_fix_pos_m.x * 100.0f,
                g_grip_fix_pos_m.y * 100.0f, g_grip_fix_pos_m.z * 100.0f);
            g_cfg.pal_grip_fix[0] = g_grip_fix_rot.x; g_cfg.pal_grip_fix[1] = g_grip_fix_rot.y;
            g_cfg.pal_grip_fix[2] = g_grip_fix_rot.z; g_cfg.pal_grip_fix[3] = g_grip_fix_rot.w;
            g_cfg.pal_grip_fix[4] = g_grip_fix_pos_m.x; g_cfg.pal_grip_fix[5] = g_grip_fix_pos_m.y;
            g_cfg.pal_grip_fix[6] = g_grip_fix_pos_m.z;
            g_cfg.pal_grip_fix_valid = true;
            g_cfg.pal_grip_fix_captured = true;
            pal_calib_write_file();
        }
    }

    const float pn = std::sqrt(pose_xr.x * pose_xr.x + pose_xr.y * pose_xr.y +
                               pose_xr.z * pose_xr.z + pose_xr.w * pose_xr.w);
    if (!(pn > 1.0e-6f) || !std::isfinite(pn)) {
        g_p_valid.store(false, std::memory_order_release);
        return;
    }
    pose_xr.x /= pn; pose_xr.y /= pn; pose_xr.z /= pn; pose_xr.w /= pn;

    if (g_cfg.palette_weapon_log) {
        static uint32_t s_n = 0;
        if ((s_n++ % 120u) == 0u) {
            // The recenter yaw, and the hand BEFORE and AFTER it. Never logged until 2026-08-15,
            // when a hand held in front published 50 cm to the right with a 110-degree lock gap
            // live: q_ro on this build is not a small constant -- the view lock re-anchors the
            // world yaw -- and it may be the term rotating the hand out of the camera frame.
            // room = raw controller-minus-head, UE axes; stage = after q_ro. Whichever one points
            // FORWARD when the player's hand is forward is the correct input.
            // FULL q_ro, ALL THREE ANGLES AND THE RAW QUAT. The previous revision printed yaw
            // only, read "-0.0", and concluded the recenter was identity -- while the same line
            // showed room=(0.092 0.550 -0.406) turning into stage=(-0.331 0.681 -0.457): a 42 cm
            // change under a "zero" rotation. Zero YAW is not identity. Print everything.
            // Same anchor as the published value (standing origin), or this line compares two
            // different vectors and reads as a phantom 40 cm rotation -- which it did, once.
            const Vec3 room{-(gpos.z - anchor.z), gpos.x - anchor.x, gpos.y - anchor.y};
            float rp = 0.0f, ry = 0.0f, rr = 0.0f;
            const Quat q_ro_ue{-q_ro.z, q_ro.x, q_ro.y, -q_ro.w};
            quat_to_rotator(q_ro_ue.x, q_ro_ue.y, q_ro_ue.z, q_ro_ue.w, &rp, &ry, &rr);
            API::get()->log_info(
                "[Halo-CampE-UEVR] PALETTEPOSE room=(%.3f %.3f %.3f) q_ro=(p%.1f y%.1f r%.1f) "
                "raw=(%.3f %.3f %.3f %.3f) -> stage=(%.3f %.3f %.3f)",
                room.x, room.y, room.z, rp, ry, rr, q_ro.x, q_ro.y, q_ro.z, q_ro.w,
                d_xr.x, d_xr.y, d_xr.z);
        }
    }

    // Head pitch, game degrees, from the HMD pose (recenter applied, VR->UE, then the same rotator
    // extraction the aim lane uses). Yaw is not needed: the view lock re-owns yaw, and the
    // position self-test already proves yaw exact.
    {
        const Quat h_ro = quat_mul(q_ro, hq);
        float hp = 0.0f, hy = 0.0f, hr = 0.0f;
        quat_to_rotator(-h_ro.z, h_ro.x, h_ro.y, -h_ro.w, &hp, &hy, &hr);
        g_p_head_pitch.store(hp, std::memory_order_relaxed);
    }

    // SINGLE WRITER (2026-09-11 evening, the republish's third failure explained): with
    // palettepubframe on, the tick publisher and the frame republish each wrote the pose through
    // their own filter state, two slightly different streams, and the consumer flip-flopped
    // between them every frame -- a rest jerk by construction, present in every republish test.
    // When the republish owns the pose, the tick publisher writes it only ONCE to seed a fresh
    // weapon (valid still false); afterwards it keeps publishing everything else -- stashes,
    // head pitch, validity -- and leaves the pose fields to the render path alone.
    if (g_cfg.pal_pub_frame == 0 || !g_p_valid.load(std::memory_order_acquire)) {
        g_p_seq.fetch_add(1, std::memory_order_acq_rel);
        g_p_grip_x.store(d_xr.x, std::memory_order_relaxed);
        g_p_grip_y.store(d_xr.y, std::memory_order_relaxed);
        g_p_grip_z.store(d_xr.z, std::memory_order_relaxed);
        g_p_aim_x.store(pose_xr.x, std::memory_order_relaxed);
        g_p_aim_y.store(pose_xr.y, std::memory_order_relaxed);
        g_p_aim_z.store(pose_xr.z, std::memory_order_relaxed);
        g_p_aim_w.store(pose_xr.w, std::memory_order_relaxed);
        g_p_seq.fetch_add(1, std::memory_order_acq_rel);
    }
    g_p_valid.store(true, std::memory_order_release);
}




// ---- WPNERR: the weapon-vs-controller error meter (doctrine at the Config key). RENDER THREAD,
// once per frame. Raw-vs-raw in room space, so a nonzero readout is chain latency and nothing
// else; cam_v and hand_v ride along so sway can be fitted against movement offline. Ring flushed
// as CSV beside the cfg when the key drops to 0 or the ring fills.
namespace {
struct WpnErrSample { float t, dt, err_cm, err_deg, pub_age, con_age, cam_v, hand_v; };
constexpr int kWeCap = 20000;   // ~3.7 minutes at 90 Hz
WpnErrSample s_we[kWeCap]; int s_we_n = 0; int s_we_seq = 0;
double s_we_t0 = 0.0; double s_we_prev_t = 0.0;
Vec3 s_we_prev_hand{}; bool s_we_prev_have = false;
float s_we_prev_cam[3] = {0, 0, 0}; bool s_we_cam_have = false;
// 1 Hz summary accumulators, split still/moving at 25 cm/s of camera speed.
double s_we_sum_t = 0.0; int s_we_sn[2] = {0, 0};
double s_we_se[2] = {0, 0}, s_we_sr[2] = {0, 0}; float s_we_me[2] = {0, 0}, s_we_mr[2] = {0, 0};
double s_we_sage_p = 0.0, s_we_sage_c = 0.0;
bool s_we_was_on = false;
double we_now_s() {
    return std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
void we_flush() {
    if (s_we_n <= 0) return;
    char path[MAX_PATH] = {0};
    if (g_cfg_path[0] == '\0') { s_we_n = 0; return; }
    strncpy_s(path, sizeof(path), g_cfg_path, _TRUNCATE);
    char* slash = strrchr(path, '\\');
    if (slash == nullptr) { s_we_n = 0; return; }
    char leaf[64]; sprintf_s(leaf, "halo_vr_wpnerr_%03d.csv", s_we_seq++);
    slash[1] = '\0'; strncat_s(path, sizeof(path), leaf, _TRUNCATE);
    FILE* f = nullptr;
    if (fopen_s(&f, path, "wb") != 0 || f == nullptr) { s_we_n = 0; return; }
    fprintf(f, "# halo_vr wpnerr, raw controller now vs raw pose the palette drew from, room frame\r\n");
    fprintf(f, "t,dt,err_cm,err_deg,pub_age_ms,con_age_ms,cam_v_cms,hand_v_cms\r\n");
    for (int i = 0; i < s_we_n; ++i) {
        const WpnErrSample& w = s_we[i];
        fprintf(f, "%.4f,%.4f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f\r\n",
                w.t, w.dt, w.err_cm, w.err_deg, w.pub_age, w.con_age, w.cam_v, w.hand_v);
    }
    fclose(f);
    API::get()->log_info("[Halo-CampE-UEVR] WPNERR: %d samples -> %hs", s_we_n, leaf);
    s_we_n = 0;
}
} // namespace
// ---- PALRENDER (doctrine at the Config key). RENDER THREAD, once per frame. Replays the
// build from the stock snapshot with the camera read fresh, so the gun's counter-rotation is
// exact for THIS frame instead of the sawtooth left by the last build. Every guard fails
// closed: a stale, mismatched or torn snapshot leaves the banks exactly as the build wrote
// them, which is the old behaviour.
namespace {
// The 1 Hz PALRENDER summary: how big the sawtooth is, and whether the refresh actually ran.
struct RrStats { int frames=0, applied=0, sk_stale=0, sk_gen=0, sk_torn=0, sk_pull=0; double sum=0, mx=0; std::vector<float> ds; long long said=0; };
RrStats g_rrs;
float rr_wrap180(float a) { while (a > 180.f) a -= 360.f; while (a < -180.f) a += 360.f; return a; }
void rr_summary() {
    const long long nowms = rr_now_ms();
    if (g_rrs.said == 0) { g_rrs.said = nowms; return; }
    if (nowms - g_rrs.said < 1000) return;
    if (g_rrs.frames > 0 && !g_rrs.ds.empty()) {
        std::sort(g_rrs.ds.begin(), g_rrs.ds.end());
        const float p95 = g_rrs.ds[(size_t)((double)g_rrs.ds.size() * 0.95)];
        if (g_cfg.palette_weapon_log != 0) API::get()->log_info("[Halo-CampE-UEVR] PALRENDER frames=%d applied=%d skip(stale=%d gen=%d torn=%d pull=%d) cam-delta mean %.3f p95 %.3f max %.3f deg%s",
                             g_rrs.frames, g_rrs.applied, g_rrs.sk_stale, g_rrs.sk_gen, g_rrs.sk_torn, g_rrs.sk_pull,
                             g_rrs.sum / g_rrs.frames, p95, g_rrs.mx,
                             g_cfg.pal_render == 2 ? " (measure only)" : "");
    }
    g_rrs = RrStats{}; g_rrs.said = nowms;
}
// The bank replay lives HERE, inside the anonymous namespace, because Mat3/NodeMatrix/apply_*
// are anonymous-namespace types and the public entry point below cannot name them.
bool rr_apply_banks(PaletteNode* const* bank, int nb, int32_t cnt) {
    // COMPOSE IN SCRATCH, COMMIT IN ONE COPY (fitted 2026-09-11): the first version restored
    // stock INTO the live bank and rewrote it in place, which left a window every frame where
    // the bank held the STOCK pose -- and the judge measured the result, 30-53 cm gun spikes
    // with the hand still, the error reversing direction on 62% of moving ticks. The renderer
    // was catching the half-written bank. Scratch composition narrows the racy window to one
    // memcpy of ~3.5 KB.
    static PaletteNode s_scratch[FP_NODE_COUNT];
    bool any = false;
    for (int i = 0; i < nb; ++i) {
        PaletteNode* b = bank[i];
        if (b == nullptr || IsBadWritePtr(b, sizeof(PaletteNode) * (size_t)cnt)) continue;
        memcpy(s_scratch, g_rr.stock[i], sizeof(PaletteNode) * (size_t)cnt);
        Mat3 dB{}; NodeMatrix dP{};
        if (apply_weapon_branch(s_scratch, &dB, &dP)) {
            memcpy(b, s_scratch, sizeof(PaletteNode) * (size_t)cnt);
            any = true;
        }
    }
    return any;
}
} // namespace
void blam_palette_render_refresh() {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    if (g_cfg.pal_render == 0 || !palette_weapon_mode()) return;
    rr_summary();
    // THE CAMERA FIRST, unconditionally (fitted 2026-09-11): a skipped refresh used to leave
    // g_rcam stale, and mode 7's fallback -- raw ControlRotation -- is off the locked-view
    // camera by exactly the wrist deflection. The gun leapt by the wrist angle on the builds
    // after every skip: 15% foreign frames, spikes at the skip rate, right hand only. The
    // resolve needs no banks and no snapshot, so it runs before every skip check.
    const bool rr_cam_ok = resolve_world_pullback(/*render_ctx=*/true);
    const uint32_t s0 = g_rr.seq.load(std::memory_order_acquire);
    if (s0 & 1u) { ++g_rrs.sk_torn; return; }
    const int nb = g_rr.nbanks; const int32_t cnt = g_rr.count;
    if (nb <= 0 || cnt <= 0 || cnt > FP_NODE_COUNT) return;
    if (rr_now_ms() - g_rr.at_ms.load(std::memory_order_relaxed) > 40) { ++g_rrs.sk_stale; return; }
    if (g_rr.gen != g_p_wpn_gen.load(std::memory_order_relaxed)) { ++g_rrs.sk_gen; return; }
    // THE NUMBER: how far the camera has moved since the build that placed the gun -- the exact
    // error the refresh removes, and with mode 2 the untouched baseline in the same units.
    float rr_d = -1.0f;
    if (g_cfg.palette_weapon_log != 0 || g_cfg.stomp_log != 0) {
        double rp = 0.0, ry = 0.0;
        if (read_control_rotation(&rp, &ry, nullptr)) {
            const float dy = rr_wrap180((float)ry - g_rr.cam_y.load(std::memory_order_relaxed));
            const float dp = (float)rp - g_rr.cam_p.load(std::memory_order_relaxed);
            const float d = std::sqrt(dy * dy + dp * dp);
            rr_d = d;
            ++g_rrs.frames; g_rrs.sum += d; if (d > g_rrs.mx) g_rrs.mx = d;
            if (g_rrs.ds.size() < 4096) g_rrs.ds.push_back(d);
        }
    }
    PaletteNode* bank[2] = {g_rr.bank[0], g_rr.bank[1]};
    if (g_rr.seq.load(std::memory_order_acquire) != s0) { ++g_rrs.sk_torn; return; }   // the hook got in
    // The render-context resolve runs in measure-only mode too: it is itself a measurement AND
    // it publishes g_rcam, palettecam=7's source. The first palrender=2 A/B returned above this
    // line, mode 7 silently fell back to the raw read, and the test was judged on the wrong
    // embedding (2026-09-11).
    if (g_cfg.pal_render == 2) {
        if (g_cfg.stomp_log != 0)
            stomp_mark(8, g_rcam_y.load(std::memory_order_relaxed), -1.0f,
                       g_rcam_p.load(std::memory_order_relaxed), rr_d);
        return;   // measure only: the banks stay as the build wrote them
    }
    if (!rr_cam_ok) { ++g_rrs.sk_pull; return; }
    const bool rr_ok = rr_apply_banks(bank, nb, cnt);
    if (rr_ok) ++g_rrs.applied;
    // Point 8 beside the build stamps: one capture now shows both writers' cameras and their
    // arrival order against the same clock.
    if (g_cfg.stomp_log != 0)
        stomp_mark(8, g_rcam_y.load(std::memory_order_relaxed), rr_ok ? 1.0f : 0.0f,
                   g_rcam_p.load(std::memory_order_relaxed), rr_d);
}

// Called from the stereo callback, the last point before the draw. Reads the REAL palette memory
// through the published address and compares it to what we wrote there. Any difference is the
// game having changed our bone after we set it.
void blam_palette_readback_probe() {
    if (!g_cfg.palette_weapon_log || !palette_weapon_mode()) return;
    uintptr_t a = 0; float wf[3], wu[3], wp[3];
    for (int i = 0; i < 4; ++i) {
        const uint32_t s0 = g_n8_seq.load(std::memory_order_acquire);
        a = g_n8_addr.load(std::memory_order_relaxed);
        wf[0] = g_n8_w_fx.load(std::memory_order_relaxed);
        wf[1] = g_n8_w_fy.load(std::memory_order_relaxed);
        wf[2] = g_n8_w_fz.load(std::memory_order_relaxed);
        wu[0] = g_n8_w_ux.load(std::memory_order_relaxed);
        wu[1] = g_n8_w_uy.load(std::memory_order_relaxed);
        wu[2] = g_n8_w_uz.load(std::memory_order_relaxed);
        wp[0] = g_n8_w_px.load(std::memory_order_relaxed);
        wp[1] = g_n8_w_py.load(std::memory_order_relaxed);
        wp[2] = g_n8_w_pz.load(std::memory_order_relaxed);
        if (!(s0 & 1u) && g_n8_seq.load(std::memory_order_acquire) == s0) break;
        a = 0;
    }
    if (a == 0 || IsBadReadPtr((void*)a, sizeof(PaletteNode))) return;
    const PaletteNode& live = *(const PaletteNode*)a;
    if (!std::isfinite(live.position.x) || !std::isfinite(live.forward.x)) return;
    const float dpx = live.position.x - wp[0], dpy = live.position.y - wp[1], dpz = live.position.z - wp[2];
    const float dcm = std::sqrt(dpx*dpx + dpy*dpy + dpz*dpz) * 304.8f;
    float dotf = live.forward.x*wf[0] + live.forward.y*wf[1] + live.forward.z*wf[2];
    float dotu = live.up.x*wu[0] + live.up.y*wu[1] + live.up.z*wu[2];
    if (dotf > 1.0f) dotf = 1.0f; if (dotf < -1.0f) dotf = -1.0f;
    if (dotu > 1.0f) dotu = 1.0f; if (dotu < -1.0f) dotu = -1.0f;
    const float degf = std::acos(dotf) * RAD2DEG;
    const float degu = std::acos(dotu) * RAD2DEG;
    static uint32_t s_n = 0, s_diff = 0;
    static float s_mx_cm = 0.0f, s_mx_deg = 0.0f;
    static double s_sum_cm = 0.0, s_sum_deg = 0.0;
    ++s_n;
    const float worst_deg = (degf > degu) ? degf : degu;
    if (dcm > 0.001f || worst_deg > 0.01f) {
        ++s_diff;
        s_sum_cm += (double)dcm; s_sum_deg += (double)worst_deg;
        if (dcm > s_mx_cm) s_mx_cm = dcm;
        if (worst_deg > s_mx_deg) s_mx_deg = worst_deg;
    }
    if ((s_n % 120u) == 0u) {
        API::get()->log_info(
            "[Halo-CampE-UEVR] READBACK node8 checked=%u CHANGED=%u (%.1f%%) mean %.3f cm / %.3f deg "
            "worst %.3f cm / %.3f deg  [what the palette HOLDS at draw time vs what we WROTE; "
            "non-zero = the game altered our bone after we set it]",
            s_n, s_diff, 100.0 * (double)s_diff / (double)s_n,
            s_diff ? (s_sum_cm / (double)s_diff) : 0.0,
            s_diff ? (s_sum_deg / (double)s_diff) : 0.0,
            s_mx_cm, s_mx_deg);
        s_n = s_diff = 0; s_mx_cm = s_mx_deg = 0.0f; s_sum_cm = s_sum_deg = 0.0;
    }
}

// SOCKROT (declared in BlamPalette.hpp). Compares the DRAWN weapon socket's rotation, handed in
// from the stereo callback, against the published hand that asked for it. Lives here because
// g_p_aim_* is in an anonymous namespace. "collapses" counts frames where the angular step fell
// to under half the previous frame's, which is what a single-frame excursion looks like: a jump
// followed by a stall. A rigid chain gives the socket the SAME collapse rate as the hand.
void blam_palette_sockrot_probe(float sx, float sy, float sz, float sw) {
    const Quat sq{sx, sy, sz, sw};
    const Quat hq{g_p_aim_x.load(std::memory_order_relaxed), g_p_aim_y.load(std::memory_order_relaxed),
                  g_p_aim_z.load(std::memory_order_relaxed), g_p_aim_w.load(std::memory_order_relaxed)};
    static bool  s_have = false;
    static Quat  s_ps{}, s_ph{};
    static float s_pss = 0.0f, s_phs = 0.0f;
    static uint32_t s_n = 0, s_cs = 0, s_ch = 0;
    static double s_sum_s = 0.0, s_sum_h = 0.0;
    static float s_mx_s = 0.0f, s_mx_h = 0.0f;
    auto step = [](const Quat& a, const Quat& b) {
        float d = a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
        if (d < 0.0f) d = -d; if (d > 1.0f) d = 1.0f;
        return 2.0f * std::acos(d) * RAD2DEG;
    };
    // ---- SOCKCAM (2026-09-12). VALIDATE THE ANCHOR, or kill it.
    //
    // SOCKROT's 1.48x is the one measurement everything now rests on, and it is the only one
    // that touches no mesh signal. But it compares the socket's WORLD rotation against the
    // hand's ROOM pose, and those differ by the room-to-world lift. That lift is constant (the
    // locked yaw measured one value, -90.0000, on all 9471 rows) so it cannot inflate a step
    // magnitude -- UNLESS the world frame itself turns under the player, which snap turning and
    // the aim loop both do while the locked yaw stays put. If so, the weapon's world rotation
    // changes with no hand motion, the ratio inflates for free, and the whole "judder is
    // measured" claim collapses into me measuring the player turning.
    //
    // The split settles it. Bucket the same (socket step, hand step) pairs by how much the
    // CAMERA moved that frame:
    //   ratio ~1.00 with the camera still and >1 when it moves  => the socket is picking up
    //     camera motion that failed to cancel. That IS the defect, localised, and proportional.
    //   ratio >1 in EVERY bucket, including camera-still => the socket amplifies the hand
    //     itself, and no camera term can explain it.
    //   ratio ~1.00 in every bucket once the camera is held still => SOCKROT was measuring
    //     turning all along and I retract it.
    {
        // BUCKET ON THE DIVISOR, NOT ON ControlRotation (corrected 2026-09-12). The first
        // version used g_rcam, which is ControlRotation -- but under palettecam=12 the divisor is
        // the live MESH read, a different signal. "camera still" therefore did not mean "divisor
        // constant", and the conclusion drawn from it was weaker than reported. Use the mesh
        // rotation mode 12 publishes, and fall back to ControlRotation only if it is unavailable.
        Quat cq{rotator_to_quat(g_rcam_p.load(std::memory_order_relaxed),
                                g_rcam_y.load(std::memory_order_relaxed), 0.0f)};
        for (int i = 0; i < 4; ++i) {
            const uint32_t s0 = g_qr_seq.load(std::memory_order_acquire);
            const Quat q{g_qr_x.load(std::memory_order_relaxed), g_qr_y.load(std::memory_order_relaxed),
                         g_qr_z.load(std::memory_order_relaxed), g_qr_w.load(std::memory_order_relaxed)};
            if ((s0 & 1u) || g_qr_seq.load(std::memory_order_acquire) != s0) continue;
            const float n = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
            if (n > 0.9f && n < 1.1f) cq = Quat{q.x/n, q.y/n, q.z/n, q.w/n};
            break;
        }
        static bool  s_ch = false;
        static Quat  s_pcq{};
        static double s_ssum[3] = {0.0, 0.0, 0.0}, s_hsum[3] = {0.0, 0.0, 0.0};
        static uint32_t s_cn[3] = {0, 0, 0}, s_creport = 0;
        static float s_sworst[3] = {0.0f, 0.0f, 0.0f};
        if (s_ch && s_have) {
            float d = s_pcq.x*cq.x + s_pcq.y*cq.y + s_pcq.z*cq.z + s_pcq.w*cq.w;
            if (d < 0.0f) d = -d; if (d > 1.0f) d = 1.0f;
            const float cs = 2.0f * std::acos(d) * RAD2DEG;
            const float ss2 = step(s_ps, sq), hs2 = step(s_ph, hq);
            const int b = (cs < 0.10f) ? 0 : ((cs < 1.00f) ? 1 : 2);
            ++s_cn[b]; s_ssum[b] += (double)ss2; s_hsum[b] += (double)hs2;
            if (ss2 > s_sworst[b]) s_sworst[b] = ss2;
            if ((++s_creport % 240u) == 0u) {
                API::get()->log_info(
                    "[Halo-CampE-UEVR] SOCKCAM ratio drawn/hand by DIVISOR (mesh) motion: "
                    "cam-still(<0.1deg) n=%u ratio %.2fx (sock %.4f hand %.4f worst %.3f) | "
                    "cam-slow(0.1-1) n=%u ratio %.2fx (sock %.4f hand %.4f worst %.3f) | "
                    "cam-fast(>1) n=%u ratio %.2fx (sock %.4f hand %.4f worst %.3f)  "
                    "[~1.00 when the camera is STILL and >1 when it moves = camera motion "
                    "failing to cancel; >1 everywhere = the hand itself is amplified]",
                    s_cn[0], (s_hsum[0] > 1e-9) ? (s_ssum[0]/s_hsum[0]) : 0.0,
                    s_cn[0] ? s_ssum[0]/s_cn[0] : 0.0, s_cn[0] ? s_hsum[0]/s_cn[0] : 0.0, s_sworst[0],
                    s_cn[1], (s_hsum[1] > 1e-9) ? (s_ssum[1]/s_hsum[1]) : 0.0,
                    s_cn[1] ? s_ssum[1]/s_cn[1] : 0.0, s_cn[1] ? s_hsum[1]/s_cn[1] : 0.0, s_sworst[1],
                    s_cn[2], (s_hsum[2] > 1e-9) ? (s_ssum[2]/s_hsum[2]) : 0.0,
                    s_cn[2] ? s_ssum[2]/s_cn[2] : 0.0, s_cn[2] ? s_hsum[2]/s_cn[2] : 0.0, s_sworst[2]);
                for (int i = 0; i < 3; ++i) { s_cn[i] = 0; s_ssum[i] = s_hsum[i] = 0.0; s_sworst[i] = 0.0f; }
            }
        }
        s_pcq = cq; s_ch = true;
    }
    // ---- SLIPHIST (2026-09-12). AMPLIFIED, or SLIPPING? A distribution, not an average.
    //
    // SOCKCAM killed the camera as the carrier: the drawn/hand ratio is above 1 in EVERY bucket
    // and is HIGHEST when the camera is still (2.72x, 1.12x, 1.70x at cam-still), with worsts of
    // 58.2, 17.1 and 36.1 deg. With the camera still both comp_inv and the lift are effectively
    // constant, and a constant rotation preserves step magnitude EXACTLY, so the socket step
    // should equal the hand step. It does not.
    //
    // Two very different mechanisms give a mean ratio above 1, and an average cannot tell them
    // apart:
    //   AMPLIFICATION: every frame is scaled, so the per-frame ratio clusters in ONE lump near
    //     1.5 and there are few near-zero frames.
    //   SLIPPING: the drawn pose occasionally lags a frame and then catches up, so the ratio is
    //     BIMODAL, piling up near 0 (the frame that was skipped) and near 2 (the catch-up). The
    //     mean sits near 1 while the worsts explode, and E/mesh lands on 2 on the catch-up
    //     frames -- which would finally explain why E/mesh was invariant to every knob.
    //
    // So histogram the per-frame ratio. Bimodal at 0 and 2 means a timing slip and the fix is a
    // latency fix. One lump near 1.5 means genuine amplification and timing is innocent.
    // Only frames where the hand actually moved are counted, since a near-zero denominator
    // manufactures huge ratios out of nothing.
    if (s_have) {
        const float ss3 = step(s_ps, sq), hs3 = step(s_ph, hq);
        // GATE RAISED (2026-09-12). At 0.05 deg the distribution came back flat across the
        // whole range with a max of 99.5, which is division noise rather than a shape: frames
        // where the hand barely moved dominated it. 0.50 deg per frame at ~45 Hz is about
        // 22 deg/s of wrist rotation, comfortably real motion, and a socket step of a few
        // degrees against it can no longer produce a meaningless ratio.
        if (hs3 > 0.50f) {
            const float r = ss3 / hs3;
            static uint32_t s_hb[8] = {0,0,0,0,0,0,0,0};   // <0.1 .1-.5 .5-.9 .9-1.1 1.1-1.5 1.5-2 2-3 >3
            static uint32_t s_hn = 0;
            static float s_rmax = 0.0f;
            int b = 7;
            if      (r < 0.10f) b = 0;
            else if (r < 0.50f) b = 1;
            else if (r < 0.90f) b = 2;
            else if (r < 1.10f) b = 3;
            else if (r < 1.50f) b = 4;
            else if (r < 2.00f) b = 5;
            else if (r < 3.00f) b = 6;
            ++s_hb[b]; ++s_hn;
            if (r > s_rmax) s_rmax = r;
            if ((s_hn % 240u) == 0u) {
                const double n = (double)s_hn;
                API::get()->log_info(
                    "[Halo-CampE-UEVR] SLIPHIST per-frame drawn/hand ratio, n=%u, hand-moving frames only: "
                    "<0.1 %.1f%% | 0.1-0.5 %.1f%% | 0.5-0.9 %.1f%% | 0.9-1.1 %.1f%% | 1.1-1.5 %.1f%% | "
                    "1.5-2 %.1f%% | 2-3 %.1f%% | >3 %.1f%%  max %.1f  [BIMODAL at <0.1 and 2-3 = a "
                    "timing SLIP, fix latency. ONE lump near 1.1-1.5 = genuine amplification, timing "
                    "is innocent]",
                    s_hn,
                    100.0*s_hb[0]/n, 100.0*s_hb[1]/n, 100.0*s_hb[2]/n, 100.0*s_hb[3]/n,
                    100.0*s_hb[4]/n, 100.0*s_hb[5]/n, 100.0*s_hb[6]/n, 100.0*s_hb[7]/n, s_rmax);
                for (int i = 0; i < 8; ++i) s_hb[i] = 0;
                s_hn = 0; s_rmax = 0.0f;
            }
        }
    }
    // ---- BANDMATCH (2026-09-12). COMPARE LIKE WITH LIKE, and settle whether 1.5x is real.
    //
    // E/mesh ~ 2.0 and SOCKROT ~ 1.5x have now survived a stale divisor, a fresh-only divisor, a
    // LATCHED divisor (clatch reuse=637 fresh=247, so it demonstrably fired), an inverted one, a
    // doubled one, NO divisor at all, two different divisor signals, both lift yaws, the barrel
    // lock off, and one hand per frame. A number that survives every possible change to the
    // system is a property of the MEASUREMENT, not of the system.
    //
    // The reason: SOCKROT's numerator is the socket, which palrender re-writes every RENDERED
    // frame with a live camera. Its denominator is g_p_aim_*, written only by the TICK publisher.
    // Mean absolute step is a total-variation statistic and total variation GROWS WITH BANDWIDTH,
    // so a render-rate signal over a tick-rate signal exceeds 1.0 with zero defect present. That
    // is invariant to every knob, which is precisely what was observed.
    //
    // So read the controller HERE, at the render callback, at the same rate the socket is
    // sampled, and compare against that instead. If the ratio collapses to ~1.00 the 1.5x was my
    // sampling artifact and the drawn weapon is faithful. If it stays at 1.5x the amplification
    // is real and survives a like-for-like comparison.
    {
        const auto bm_idx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                                : API::VR::get_right_controller_index();
        Vec3 bm_p{}; Quat bm_q{};
        if (bm_idx >= 0 && get_pose(bm_idx, &bm_p, &bm_q, /*use_aim=*/true)) {
            static bool  s_bm_have = false;
            static Quat  s_bm_prev{}, s_bm_psock{};
            static uint32_t s_bm_n = 0;
            static double s_bm_sock = 0.0, s_bm_hand = 0.0, s_bm_tick = 0.0;
            static float s_bm_mxs = 0.0f, s_bm_mxh = 0.0f;
            if (s_bm_have) {
                const float hs_fresh = step(s_bm_prev, bm_q);
                const float ss_same  = step(s_bm_psock, sq);
                const float hs_tick  = step(s_ph, hq);      // the old, bandwidth-mismatched one
                ++s_bm_n;
                s_bm_sock += (double)ss_same; s_bm_hand += (double)hs_fresh; s_bm_tick += (double)hs_tick;
                if (ss_same > s_bm_mxs) s_bm_mxs = ss_same;
                if (hs_fresh > s_bm_mxh) s_bm_mxh = hs_fresh;
                if ((s_bm_n % 120u) == 0u) {
                    const double n = (double)s_bm_n;
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] BANDMATCH n=%u | socket %.4f worst %.4f | hand read AT THE "
                        "RENDER CALLBACK %.4f worst %.4f -> ratio %.2fx  <== LIKE FOR LIKE | hand from "
                        "the TICK publisher %.4f -> ratio %.2fx (the old, bandwidth-mismatched number) "
                        "| render/tick hand bandwidth %.2fx  [if the like-for-like ratio is ~1.00 the "
                        "1.5x was a sampling artifact and the drawn weapon is faithful]",
                        s_bm_n, s_bm_sock/n, s_bm_mxs, s_bm_hand/n, s_bm_mxh,
                        (s_bm_hand > 1e-9) ? (s_bm_sock/s_bm_hand) : 0.0,
                        s_bm_tick/n, (s_bm_tick > 1e-9) ? (s_bm_sock/s_bm_tick) : 0.0,
                        (s_bm_tick > 1e-9) ? (s_bm_hand/s_bm_tick) : 0.0);
                    s_bm_n = 0; s_bm_sock = s_bm_hand = s_bm_tick = 0.0;
                    s_bm_mxs = s_bm_mxh = 0.0f;
                }
            }
            s_bm_prev = bm_q; s_bm_psock = sq; s_bm_have = true;
        }
    }
    if (s_have) {
        const float ss = step(s_ps, sq), hs = step(s_ph, hq);
        ++s_n;
        s_sum_s += (double)ss; s_sum_h += (double)hs;
        if (ss > s_mx_s) s_mx_s = ss;
        if (hs > s_mx_h) s_mx_h = hs;
        if (s_pss > 1.0e-4f && ss < s_pss * 0.5f) ++s_cs;
        if (s_phs > 1.0e-4f && hs < s_phs * 0.5f) ++s_ch;
        s_pss = ss; s_phs = hs;
        if ((s_n % 120u) == 0u) {
            API::get()->log_info(
                "[Halo-CampE-UEVR] SOCKROT drawn socket step mean %.4f worst %.4f deg collapses=%u "
                "(%.1f%%) | published hand step mean %.4f worst %.4f deg collapses=%u (%.1f%%) | "
                "ratio drawn/hand %.2fx  [the DRAWN weapon's rotation against the hand that asked "
                "for it; the symptom is rotational and this was never recorded]",
                s_sum_s / (double)s_n, s_mx_s, s_cs, 100.0 * (double)s_cs / (double)s_n,
                s_sum_h / (double)s_n, s_mx_h, s_ch, 100.0 * (double)s_ch / (double)s_n,
                (s_sum_h > 1.0e-9) ? (s_sum_s / s_sum_h) : 0.0);
            s_n = s_cs = s_ch = 0; s_sum_s = s_sum_h = 0.0; s_mx_s = s_mx_h = 0.0f;
        }
    }
    // ---- SOCKLINK. Does OUR node 8 alone determine the socket's world rotation?
    //
    // RETRACTION (2026-09-12). The first version of this compared basis_of(node8).forward, a
    // BLAM-convention column, against quat_rotate(socket, {1,0,0}), a UE axis, with no frame map
    // between them. Those frames differ by a similarity, so the mismatch does not cancel -- it
    // grows and shrinks with the pose. It reported a 31.9 to 121.9 degree "spread" and a 30 deg
    // per-frame change, and I read that out as proof that our node does not own the socket. It
    // was proof of nothing. Same class of error as the det = -1 reflection bug in August: a
    // rotation compared across two conventions without converting between them.
    // Corrected below by building the socket's basis in BLAM convention with the same two-sided
    // conversion apply_weapon_branch uses, then measuring the true relative rotation by trace.
    //
    // SOCKROT measured the drawn socket rotating 1.5x the hand with single-frame spikes to 47.33
    // deg while the hand's worst in the same window was 15.77. Meanwhile READBACK proved the
    // palette holds our exact bytes at draw time (0.000 cm) and RELSTOCK proved the stock node
    // relations are dead constants (per-frame change 0.0000). So the amplification enters between
    // the palette and the socket.
    //
    // If the palette is MODEL-space, node 8's basis alone fixes the socket's world rotation and
    // the relation between them is a CONSTANT socket offset. If the palette is LOCAL, the socket
    // rides a parent chain the game animates and that we never write, and the relation MOVES --
    // which is exactly the player's "the aim ray is what determines where the characters wepaon and
    // arms are posed", and it would multiply our hand by the game's animation.
    //
    // The relation is transpose(our written node 8) x (socket world). Constant means our write
    // fully owns the socket. Moving means it does not, and its motion is the leak.
    {
        const float fx = g_n8_w_fx.load(std::memory_order_relaxed);
        const float fy = g_n8_w_fy.load(std::memory_order_relaxed);
        const float fz = g_n8_w_fz.load(std::memory_order_relaxed);
        const float ux = g_n8_w_ux.load(std::memory_order_relaxed);
        const float uy = g_n8_w_uy.load(std::memory_order_relaxed);
        const float uz = g_n8_w_uz.load(std::memory_order_relaxed);
        const float fl = std::sqrt(fx*fx + fy*fy + fz*fz);
        if (fl > 0.5f) {
            // The socket, in MESH space and in BLAM convention. Both conversions matter: the
            // socket is a WORLD rotation while the palette is mesh-local, and it is a UE
            // rotation while the palette is Blam. Skipping either one makes the comparison
            // pose-dependent, which is what the retracted version did.
            Quat mqq{0.0f, 0.0f, 0.0f, 1.0f};
            bool mqq_ok = false;
            for (int i = 0; i < 4; ++i) {
                const uint32_t s0 = g_qr_seq.load(std::memory_order_acquire);
                const Quat q{g_qr_x.load(std::memory_order_relaxed), g_qr_y.load(std::memory_order_relaxed),
                             g_qr_z.load(std::memory_order_relaxed), g_qr_w.load(std::memory_order_relaxed)};
                if ((s0 & 1u) || g_qr_seq.load(std::memory_order_acquire) != s0) continue;
                const float n = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
                if (n > 0.9f && n < 1.1f) { mqq = Quat{q.x/n, q.y/n, q.z/n, q.w/n}; mqq_ok = true; }
                break;
            }
            // STALE-SAMPLE BUG, FIXED (2026-09-12, found by an adversarial audit of my own
            // instruments). This early return sits BEFORE the previous-sample update at the end
            // of the function, while the SOCKROT and SLIPHIST accumulators run BEFORE it. So on
            // any frame where the seqlocked mesh read failed, a step was still counted but the
            // previous sample was never advanced, and the next good frame reported a MULTI-FRAME
            // step as a single one. Every maximum this probe has printed -- 47.33, 58.16, 77.85,
            // 161.46 deg -- was therefore possibly an accumulation rather than a spike, and the
            // headline of the whole hunt rested on it. Advance the samples before leaving.
            if (!mqq_ok) { s_ps = sq; s_ph = hq; s_have = true; return; }
            const Quat sm = quat_mul(quat_conj(mqq), sq);
            const Vec3 rx = quat_rotate(sm, Vec3{1.0f, 0.0f, 0.0f});
            const Vec3 ry = quat_rotate(sm, Vec3{0.0f, 1.0f, 0.0f});
            const Vec3 rz = quat_rotate(sm, Vec3{0.0f, 0.0f, 1.0f});
            const auto skf = ue_to_blam(rx);
            const auto skl = ue_to_blam(Vec3{-ry.x, -ry.y, -ry.z});
            const auto sku = ue_to_blam(rz);
            const float lx = g_n8_w_lx.load(std::memory_order_relaxed);
            const float ly = g_n8_w_ly.load(std::memory_order_relaxed);
            const float lz = g_n8_w_lz.load(std::memory_order_relaxed);
            // TRUE relative rotation between what we wrote and the socket. Both are orthonormal
            // bases, so trace(transpose(ours) x sock) is just the three column dot products, and
            // no matrix type is needed -- which matters because Mat3 is ambiguous out here.
            // RIGID means this angle is CONSTANT, whatever its value.
            const float tr3 = (fx*skf.x + fy*skf.y + fz*skf.z) / fl
                            + (lx*skl.x + ly*skl.y + lz*skl.z)
                            + (ux*sku.x + uy*sku.y + uz*sku.z);
            float ca = (tr3 - 1.0f) * 0.5f;
            if (ca > 1.0f) ca = 1.0f; if (ca < -1.0f) ca = -1.0f;
            const float af = std::acos(ca) * RAD2DEG;
            const float au = af;
            static bool s_link_have = false;   // winsock2 macro-collides on the short name
            static float s_pf = 0.0f, s_pu = 0.0f;
            static uint32_t s_ln = 0;
            static float s_mn_f = 1e9f, s_mx_f = -1e9f, s_mx_df = 0.0f;
            if (s_link_have) {
                const float ddf = std::fabs(af - s_pf), ddu = std::fabs(au - s_pu);
                const float w = (ddf > ddu) ? ddf : ddu;
                if (w > s_mx_df) s_mx_df = w;
            }
            s_pf = af; s_pu = au; s_link_have = true;
            if (af < s_mn_f) s_mn_f = af;
            if (af > s_mx_f) s_mx_f = af;
            // ---- ENDERR (2026-09-12). THE TOTAL END-TO-END ERROR, with nothing assumed.
    //
    // The chain is supposed to satisfy one equation. The renderer draws the weapon at
    // mesh_actual x node8, and we write node8 = conj(mesh_estimate) x hand_world, so
    //     socket_world = mesh_actual x conj(mesh_estimate) x hand_world
    // If our estimate were right that collapses to hand_world and SOCKROT would read 1.00x.
    // It reads 1.48x with 47 deg single-frame spikes, so the cancellation is failing by about
    // half the hand's motion, and the entire failure is one term:
    //     E = socket_world (x) conj(hand_world) = mesh_actual x conj(mesh_estimate)
    //
    // E needs no frame conversion (two world rotations of the same kind, the reason SOCKROT
    // survived when three probes today did not), no assumption about which buffer is drawn, and
    // no theory of mechanism. A CONSTANT E means the chain is perfect and the judder is outside
    // it. A MOVING E is the judder itself, in world degrees.
    //
    // And because E is logged beside the candidates, its motion can be REGRESSED rather than
    // guessed: against the camera we divide by, against the live mesh read, against the hand.
    // Whichever it tracks names what our estimate is missing, and the coefficient is the fix.
    // Fitting a correction to a measured error is the rule I have been breaking all day by
    // deriving mechanisms and hoping.
    {
        // hand in WORLD: the published aim lifted by the same room->world yaw the placement uses
        const Quat lift = rotator_to_quat(0.0f, g_view_base_yaw.load(std::memory_order_relaxed), 0.0f);
        const Quat hand_world = quat_mul(lift, hq);
        const Quat Eq = quat_mul(sq, quat_conj(hand_world));
        float ew = Eq.w; if (ew < 0.0f) ew = -ew; if (ew > 1.0f) ew = 1.0f;
        const float edeg = 2.0f * std::acos(ew) * RAD2DEG;
        // the two candidates E could be made of, as angles, same instant
        const Quat camq = rotator_to_quat(g_rcam_p.load(std::memory_order_relaxed),
                                          g_rcam_y.load(std::memory_order_relaxed), 0.0f);
        static bool  s_eh = false;
        static Quat  s_pE{}, s_pc{}, s_pm{}, s_phw{};
        static uint32_t s_en = 0;
        static double s_sE = 0.0, s_sc = 0.0, s_sm = 0.0, s_sh = 0.0;
        static float s_mE = 0.0f;
        static double s_absE = 0.0; static float s_absmn = 1.0e9f, s_absmx = -1.0e9f;
        auto stp = [](const Quat& a, const Quat& b) {
            float d = a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
            if (d < 0.0f) d = -d; if (d > 1.0f) d = 1.0f;
            return 2.0f * std::acos(d) * RAD2DEG;
        };
        if (s_eh) {
            const float dE = stp(s_pE, Eq);
            const float dc = stp(s_pc, camq);
            const float dm = stp(s_pm, mqq);
            const float dh = stp(s_phw, hand_world);
            ++s_en;
            s_sE += (double)dE; s_sc += (double)dc; s_sm += (double)dm; s_sh += (double)dh;
            if (dE > s_mE) s_mE = dE;
            s_absE += (double)edeg;
            if (edeg < s_absmn) s_absmn = edeg;
            if (edeg > s_absmx) s_absmx = edeg;
            if ((s_en % 120u) == 0u) {
                API::get()->log_info(
                    "[Halo-CampE-UEVR] ENDERR |E| mean %.3f range %.3f..%.3f deg (a CONSTANT |E| is "
                    "a harmless static offset) | E per-frame MOVEMENT mean %.4f worst %.4f deg "
                    "<== THIS IS THE JUDDER | same-window per-frame movement of: camera %.4f, live "
                    "mesh %.4f, hand-in-world %.4f deg | E as a fraction of hand %.3f, of camera "
                    "%.3f, of mesh %.3f  [the fraction nearest 1.00 names what our estimate misses]",
                    s_absE / s_en, s_absmn, s_absmx, s_sE / s_en, s_mE,
                    s_sc / s_en, s_sm / s_en, s_sh / s_en,
                    (s_sh > 1e-9) ? (s_sE / s_sh) : 0.0,
                    (s_sc > 1e-9) ? (s_sE / s_sc) : 0.0,
                    (s_sm > 1e-9) ? (s_sE / s_sm) : 0.0);
                s_en = 0; s_sE = s_sc = s_sm = s_sh = 0.0; s_mE = 0.0f;
                s_absE = 0.0; s_absmn = 1.0e9f; s_absmx = -1.0e9f;
            }
        }
        // ---- ERRAXIS. E's rotation AXIS, not its angle.
        //
        // Both compgain sweeps made things worse (ratio 1.48 -> 1.68/2.53 at -1 and -> 2.39 at
        // +2, E/mesh 1.96 -> 2.84 and -> 2.89), so the divisor's MAGNITUDE is already right and
        // no scalar coefficient fixes this. What is left is its AXIS.
        //
        // And |E| parks on exactly 90 degrees: 90.47, 90.30, 90.72, 90.86 across four windows.
        // A standing error that sits on 90 is not a calibration value, it is an AXIS SWAP.
        // RELSTOCK independently reported transpose(stock_8) x stock_7 = 90.0000 dead constant.
        // If the divisor turns about the wrong axis then the cancellation ADDS a component of
        // the mesh's motion instead of subtracting it, which is exactly E/mesh ~ 2, is invisible
        // at rest because there is no motion to mis-cancel, and cannot be helped by scaling.
        //
        // So decompose E into an axis. A FIXED axis aligned with a coordinate direction names
        // the permutation to correct. A WANDERING axis means it is not a convention error.
        // Reported in the camera frame too, because that is the frame the divisor is built in.
        {
            Quat e = Eq;
            if (e.w < 0.0f) { e.x = -e.x; e.y = -e.y; e.z = -e.z; e.w = -e.w; }
            const float s = std::sqrt(e.x*e.x + e.y*e.y + e.z*e.z);
            if (s > 1.0e-6f) {
                const float ax = e.x / s, ay = e.y / s, az = e.z / s;
                // the same axis expressed in the camera's frame, where comp_rot is built
                const Quat ci = quat_conj(camq);
                const Vec3 acam = quat_rotate(ci, Vec3{ax, ay, az});
                static uint32_t s_an = 0;
                static double s_ax = 0.0, s_ay = 0.0, s_az = 0.0;
                static double s_cx = 0.0, s_cy = 0.0, s_cz = 0.0;
                static double s_dev = 0.0;
                static float s_pax = 0.0f, s_pay = 0.0f, s_paz = 0.0f;
                static bool s_ap = false;
                ++s_an;
                s_ax += ax; s_ay += ay; s_az += az;
                s_cx += acam.x; s_cy += acam.y; s_cz += acam.z;
                if (s_ap) {
                    // how much the axis itself moves frame to frame, in degrees
                    float d = s_pax*ax + s_pay*ay + s_paz*az;
                    if (d > 1.0f) d = 1.0f; if (d < -1.0f) d = -1.0f;
                    s_dev += (double)(std::acos(std::fabs(d)) * RAD2DEG);
                }
                s_pax = ax; s_pay = ay; s_paz = az; s_ap = true;
                if ((s_an % 120u) == 0u) {
                    const double n = (double)s_an;
                    const double mx = s_ax/n, my = s_ay/n, mz = s_az/n;
                    const double ml = std::sqrt(mx*mx + my*my + mz*mz);
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] ERRAXIS E axis mean (%.3f %.3f %.3f) |mean| %.3f "
                        "(1.000 = perfectly FIXED axis, 0 = wandering) | in CAMERA frame "
                        "(%.3f %.3f %.3f) | axis wander %.2f deg/frame  [a fixed axis on a "
                        "coordinate direction names the permutation to fix; |E| already parks "
                        "on exactly 90 deg]",
                        mx, my, mz, ml, s_cx/n, s_cy/n, s_cz/n, s_dev/n);
                    s_an = 0; s_ax = s_ay = s_az = 0.0; s_cx = s_cy = s_cz = 0.0; s_dev = 0.0;
                }
            }
        }
        s_pE = Eq; s_pc = camq; s_pm = mqq; s_phw = hand_world; s_eh = true;
    }

    // ---- BANKCHECK. The socket against every buffer we wrote, not just the live one.
            // The renderer draws from a bank, so the buffer that MATCHES the socket names where
            // the drawn pose actually comes from, and the buffer that does NOT match by a lot is
            // the one holding a pose from another frame.
            {
                auto ang_to = [&](float bfx, float bfy, float bfz, float blx, float bly, float blz,
                                  float bux, float buy, float buz) {
                    const float n = std::sqrt(bfx*bfx + bfy*bfy + bfz*bfz);
                    if (!(n > 0.5f)) return -1.0f;
                    const float tr = (bfx*skf.x + bfy*skf.y + bfz*skf.z) / n
                                   + (blx*skl.x + bly*skl.y + blz*skl.z)
                                   + (bux*sku.x + buy*sku.y + buz*sku.z);
                    float c = (tr - 1.0f) * 0.5f;
                    if (c > 1.0f) c = 1.0f; if (c < -1.0f) c = -1.0f;
                    return std::acos(c) * RAD2DEG;
                };
                float ab[2] = {-1.0f, -1.0f};
                long long agb[2] = {-1, -1};
                const long long nowb = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                for (int s = 0; s < 2; ++s) {
                    for (int i = 0; i < 4; ++i) {
                        const uint32_t s0 = g_bank8[s].seq.load(std::memory_order_acquire);
                        const float bfx = g_bank8[s].fx.load(std::memory_order_relaxed);
                        const float bfy = g_bank8[s].fy.load(std::memory_order_relaxed);
                        const float bfz = g_bank8[s].fz.load(std::memory_order_relaxed);
                        const float blx = g_bank8[s].lx.load(std::memory_order_relaxed);
                        const float bly = g_bank8[s].ly.load(std::memory_order_relaxed);
                        const float blz = g_bank8[s].lz.load(std::memory_order_relaxed);
                        const float bux = g_bank8[s].ux.load(std::memory_order_relaxed);
                        const float buy = g_bank8[s].uy.load(std::memory_order_relaxed);
                        const float buz = g_bank8[s].uz.load(std::memory_order_relaxed);
                        const long long bms = g_bank8[s].ms.load(std::memory_order_relaxed);
                        if ((s0 & 1u) || g_bank8[s].seq.load(std::memory_order_acquire) != s0) continue;
                        ab[s] = ang_to(bfx, bfy, bfz, blx, bly, blz, bux, buy, buz);
                        agb[s] = (bms != 0) ? (nowb - bms) : -1;
                        break;
                    }
                }
                static uint32_t s_bn = 0, s_win_live = 0, s_win_b0 = 0, s_win_b1 = 0;
                static double s_sl = 0.0, s_s0 = 0.0, s_s1 = 0.0;
                static float s_ml = 0.0f, s_m0 = 0.0f, s_m1 = 0.0f;
                ++s_bn;
                s_sl += (double)af; if (af > s_ml) s_ml = af;
                if (ab[0] >= 0.0f) { s_s0 += (double)ab[0]; if (ab[0] > s_m0) s_m0 = ab[0]; }
                if (ab[1] >= 0.0f) { s_s1 += (double)ab[1]; if (ab[1] > s_m1) s_m1 = ab[1]; }
                float bestv = af; int bestw = 0;
                if (ab[0] >= 0.0f && ab[0] < bestv) { bestv = ab[0]; bestw = 1; }
                if (ab[1] >= 0.0f && ab[1] < bestv) { bestv = ab[1]; bestw = 2; }
                if (bestw == 0) ++s_win_live; else if (bestw == 1) ++s_win_b0; else ++s_win_b1;
                if ((s_bn % 120u) == 0u) {
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] BANKCHECK socket vs LIVE mean %.4f worst %.4f | vs bankA "
                        "mean %.4f worst %.4f (age %lld ms) | vs bankB mean %.4f worst %.4f (age %lld ms) "
                        "| closest was LIVE %u bankA %u bankB %u of %u  [the buffer that MATCHES is "
                        "what the renderer draws from]",
                        s_sl / s_bn, s_ml, s_s0 / s_bn, s_m0, agb[0], s_s1 / s_bn, s_m1, agb[1],
                        s_win_live, s_win_b0, s_win_b1, s_bn);
                    s_bn = s_win_live = s_win_b0 = s_win_b1 = 0;
                    s_sl = s_s0 = s_s1 = 0.0; s_ml = s_m0 = s_m1 = 0.0f;
                }
            }

            // ---- BANKDIFF (2026-09-12). DO THE TWO BANKS THE ENGINE BLENDS ACTUALLY DIFFER?
            //
            // Reading the engine's own weight off ctx+0x04 showed a real, varying interpolation:
            // t = 0.049, 0.059, 0.580, 0.343, with the two named banks swapping roles every frame
            // (A=0 B=1, then A=1 B=0) and both of them collected, i.e. both written by us. The
            // disassembly of sim+0x23BF40 makes the consequence exact: out = nlerp(bankB, bankA, t).
            // If both banks hold the SAME node 8 at draw time, t does nothing. If they differ, the
            // drawn weapon is a moving mix of two poses, which is a jump-and-return by construction.
            // So measure, per rendered frame, the angle between the two banks' node 8 as they sit in
            // memory right before the draw, and where the drawn socket lands between them.
            {
                const uint32_t brs = g_rr.seq.load(std::memory_order_acquire);
                const PaletteNode* bk0 = g_rr.bank[0];
                const PaletteNode* bk1 = g_rr.bank[1];
                if (!(brs & 1u) && g_rr.nbanks >= 2 && bk0 != nullptr && bk1 != nullptr &&
                    !IsBadReadPtr((void*)bk0, sizeof(PaletteNode) * 9) &&
                    !IsBadReadPtr((void*)bk1, sizeof(PaletteNode) * 9)) {
                    const PaletteNode n0 = bk0[8];
                    const PaletteNode n1 = bk1[8];
                    auto nrm = [](const NodeMatrix& v) { return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z); };
                    auto trace_deg = [](const NodeMatrix& af, const NodeMatrix& al, const NodeMatrix& au,
                                        float bfx, float bfy, float bfz, float blx, float bly, float blz,
                                        float bux, float buy, float buz) {
                        const float tr = (af.x*bfx + af.y*bfy + af.z*bfz) + (al.x*blx + al.y*bly + al.z*blz)
                                       + (au.x*bux + au.y*buy + au.z*buz);
                        float c = (tr - 1.0f) * 0.5f;
                        if (c > 1.0f) c = 1.0f; if (c < -1.0f) c = -1.0f;
                        return std::acos(c) * RAD2DEG;
                    };
                    const float l0 = nrm(n0.forward), l1 = nrm(n1.forward);
                    if (l0 > 0.5f && l1 > 0.5f) {
                        const float a01 = trace_deg(n0.forward, n0.left, n0.up,
                                                    n1.forward.x, n1.forward.y, n1.forward.z,
                                                    n1.left.x, n1.left.y, n1.left.z,
                                                    n1.up.x, n1.up.y, n1.up.z);
                        const float as0 = trace_deg(n0.forward, n0.left, n0.up,
                                                    skf.x, skf.y, skf.z, skl.x, skl.y, skl.z, sku.x, sku.y, sku.z);
                        const float as1 = trace_deg(n1.forward, n1.left, n1.up,
                                                    skf.x, skf.y, skf.z, skl.x, skl.y, skl.z, sku.x, sku.y, sku.z);
                        const float dp0 = std::sqrt((n0.position.x-n1.position.x)*(n0.position.x-n1.position.x)
                                                  + (n0.position.y-n1.position.y)*(n0.position.y-n1.position.y)
                                                  + (n0.position.z-n1.position.z)*(n0.position.z-n1.position.z)) * 304.8f;
                        // ---- LIVEVSBANK + RESIDUAL (2026-09-12). BANKDIFF: the banks are identical
                        // (0.0000 deg, 0.0000 cm, 0% of frames differ), so the blend is a no-op. But
                        // the drawn socket still sits 0.15-2.09 deg (mean) off BOTH banks, so the
                        // departure enters AFTER the blend. Two places it can: the poser's fallback
                        // to the live palette, and its compose with a root node. This checks the
                        // first directly and measures whether the post-blend residual is steady
                        // (a static offset, harmless) or jumps frame to frame (the judder).
                        {
                            const uintptr_t lpa = g_live_pal_addr.load(std::memory_order_relaxed);
                            float lva = -1.0f, lvcm = -1.0f;
                            if (lpa != 0 && !IsBadReadPtr((void*)lpa, sizeof(PaletteNode) * 9)) {
                                const PaletteNode ln = ((const PaletteNode*)lpa)[8];
                                if (nrm(ln.forward) > 0.5f) {
                                    lva = trace_deg(n0.forward, n0.left, n0.up,
                                                    ln.forward.x, ln.forward.y, ln.forward.z,
                                                    ln.left.x, ln.left.y, ln.left.z, ln.up.x, ln.up.y, ln.up.z);
                                    const float dx = ln.position.x - n0.position.x;
                                    const float dy = ln.position.y - n0.position.y;
                                    const float dz = ln.position.z - n0.position.z;
                                    lvcm = std::sqrt(dx*dx + dy*dy + dz*dz) * 304.8f;
                                }
                            }
                            const float resid = (as0 < as1) ? as0 : as1;
                            static bool s_rh = false; static float s_rp = 0.0f;
                            static uint32_t s_ln2 = 0, s_lvdiff = 0, s_rbig = 0;
                            static double s_lva = 0.0, s_lvcm = 0.0, s_rj = 0.0;
                            static float s_lvamx = 0.0f, s_lvcmmx = 0.0f, s_rjmx = 0.0f;
                            ++s_ln2;
                            if (lva >= 0.0f) {
                                s_lva += lva; s_lvcm += lvcm;
                                if (lva > s_lvamx) s_lvamx = lva;
                                if (lvcm > s_lvcmmx) s_lvcmmx = lvcm;
                                if (lva > 0.5f || lvcm > 0.5f) ++s_lvdiff;
                            }
                            if (s_rh) {
                                const float j = std::fabs(resid - s_rp);
                                s_rj += j; if (j > s_rjmx) s_rjmx = j;
                                if (j > 1.0f) ++s_rbig;
                            }
                            s_rp = resid; s_rh = true;
                            if ((s_ln2 % 120u) == 0u) {
                                const double n = (double)s_ln2;
                                API::get()->log_info(
                                    "[Halo-CampE-UEVR] LIVEVSBANK live palette vs bank node8: angle mean %.4f worst %.4f deg, "
                                    "pos mean %.4f worst %.4f cm, frames differing %.1f%% | socket-minus-bank RESIDUAL "
                                    "per-frame jump mean %.4f worst %.4f deg, jumps over 1 deg %.1f%%  [live==bank means "
                                    "the fallback cannot cause a pop; a residual that JUMPS is the judder entering "
                                    "after the blend, which leaves the poser's root compose]",
                                    s_lva/n, s_lvamx, s_lvcm/n, s_lvcmmx, 100.0*s_lvdiff/n,
                                    s_rj/n, s_rjmx, 100.0*s_rbig/n);
                                s_ln2 = s_lvdiff = s_rbig = 0; s_lva = s_lvcm = s_rj = 0.0;
                                s_lvamx = s_lvcmmx = s_rjmx = 0.0f;
                            }
                        }
                        const float te = g_blend_t.load(std::memory_order_relaxed);
                        static uint32_t s_dn = 0, s_ddiff = 0;
                        static double s_da = 0.0, s_dcm = 0.0, s_dmatch = 0.0;
                        static float s_dmx = 0.0f, s_dcmmx = 0.0f;
                        // Pearson between the engine weight and where the socket sits from bank0 to bank1
                        static double s_sx = 0, s_sy = 0, s_sxx = 0, s_syy = 0, s_sxy = 0; static uint32_t s_cn = 0;
                        ++s_dn;
                        s_da += (double)a01; if (a01 > s_dmx) s_dmx = a01;
                        s_dcm += (double)dp0; if (dp0 > s_dcmmx) s_dcmmx = dp0;
                        s_dmatch += (double)((as0 < as1) ? as0 : as1);
                        if (a01 > 0.5f || dp0 > 0.5f) ++s_ddiff;
                        if (a01 > 0.5f && te >= 0.0f && te <= 1.0f) {
                            const double fr = (double)as0 / (double)a01;
                            ++s_cn; s_sx += te; s_sy += fr; s_sxx += (double)te*te; s_syy += fr*fr; s_sxy += te*fr;
                        }
                        if ((s_dn % 120u) == 0u) {
                            const double n = (double)s_dn;
                            double corr = 0.0;
                            if (s_cn > 10) {
                                const double c = (double)s_cn;
                                const double vx = s_sxx - s_sx*s_sx/c, vy = s_syy - s_sy*s_sy/c, cv = s_sxy - s_sx*s_sy/c;
                                if (vx > 1e-12 && vy > 1e-12) corr = cv / std::sqrt(vx*vy);
                            }
                            API::get()->log_info(
                                "[Halo-CampE-UEVR] BANKDIFF bank0 vs bank1 node8: angle mean %.4f worst %.4f deg, "
                                "position mean %.4f worst %.4f cm, frames that DIFFER %.1f%% | socket to nearer bank "
                                "mean %.4f deg | corr(engine t, socket fraction bank0->bank1) %+.3f over %u frames  "
                                "[banks IDENTICAL = the blend is a no-op and t is harmless; banks DIFFER and the "
                                "socket tracks t = the engine's interpolation between our two writes IS the judder]",
                                s_da/n, s_dmx, s_dcm/n, s_dcmmx, 100.0*s_ddiff/n, s_dmatch/n, corr, s_cn);
                            s_dn = s_ddiff = 0; s_da = s_dcm = s_dmatch = 0.0; s_dmx = s_dcmmx = 0.0f;
                            s_sx = s_sy = s_sxx = s_syy = s_sxy = 0.0; s_cn = 0;
                        }
                    }
                }
            }

            // ---- BLENDT. WHERE ON THE ARC FROM STOCK TO OURS DOES THE DRAWN SOCKET SIT?
            // Doctrine and the eliminations that lead here are at the g_n8_s_* declaration.
            // Angles are taken by trace, in mesh space and Blam convention, so all three bases
            // live in one frame. t = angle(stock -> socket) / angle(stock -> ours).
            //   t ~ 1.00 every frame  => the socket IS our pose, no blending, idea dead.
            //   t < 1 and VARYING     => the drawn pose is a lerp between ours and something
            //                            else, and the variation of t is the judder.
            {
                const float sfx = g_n8_s_fx.load(std::memory_order_relaxed);
                const float sfy = g_n8_s_fy.load(std::memory_order_relaxed);
                const float sfz = g_n8_s_fz.load(std::memory_order_relaxed);
                const float slx = g_n8_s_lx.load(std::memory_order_relaxed);
                const float sly = g_n8_s_ly.load(std::memory_order_relaxed);
                const float slz = g_n8_s_lz.load(std::memory_order_relaxed);
                const float sux = g_n8_s_ux.load(std::memory_order_relaxed);
                const float suy = g_n8_s_uy.load(std::memory_order_relaxed);
                const float suz = g_n8_s_uz.load(std::memory_order_relaxed);
                const float snf = std::sqrt(sfx*sfx + sfy*sfy + sfz*sfz);
                if (snf > 0.5f) {
                    auto tr_ang = [](float ax, float ay, float az, float bx, float by, float bz,
                                     float cx, float cy, float cz,
                                     float dx, float dy, float dz, float ex, float ey, float ez,
                                     float fx2, float fy2, float fz2) {
                        const float tr = (ax*dx + ay*dy + az*dz)
                                       + (bx*ex + by*ey + bz*ez)
                                       + (cx*fx2 + cy*fy2 + cz*fz2);
                        float c = (tr - 1.0f) * 0.5f;
                        if (c > 1.0f) c = 1.0f; if (c < -1.0f) c = -1.0f;
                        return std::acos(c) * RAD2DEG;
                    };
                    // stock -> socket, and stock -> ours
                    const float a_sock = tr_ang(sfx/snf, sfy/snf, sfz/snf, slx, sly, slz, sux, suy, suz,
                                                skf.x, skf.y, skf.z, skl.x, skl.y, skl.z, sku.x, sku.y, sku.z);
                    const float a_ours = tr_ang(sfx/snf, sfy/snf, sfz/snf, slx, sly, slz, sux, suy, suz,
                                                fx/fl, fy/fl, fz/fl, lx, ly, lz, ux, uy, uz);
                    if (a_ours > 1.0f && std::isfinite(a_sock) && std::isfinite(a_ours)) {
                        const float tt = a_sock / a_ours;
                        static uint32_t s_tn = 0, s_tb[6] = {0,0,0,0,0,0};
                        static double s_tsum = 0.0;
                        static float s_tmn = 1.0e9f, s_tmx = -1.0e9f;
                        int b = 5;
                        if      (tt < 0.20f) b = 0;
                        else if (tt < 0.50f) b = 1;
                        else if (tt < 0.80f) b = 2;
                        else if (tt < 0.95f) b = 3;
                        else if (tt < 1.05f) b = 4;
                        ++s_tb[b]; ++s_tn; s_tsum += (double)tt;
                        if (tt < s_tmn) s_tmn = tt;
                        if (tt > s_tmx) s_tmx = tt;
                        if ((s_tn % 120u) == 0u) {
                            const double n = (double)s_tn;
                            API::get()->log_info(
                                "[Halo-CampE-UEVR] BLENDT engine t=%.4f banks A=%d B=%d (we collected %d) | "
                                "geometric t: mean %.3f "
                                "range %.3f..%.3f | t<0.2 %.1f%% | 0.2-0.5 %.1f%% | 0.5-0.8 %.1f%% | "
                                "0.8-0.95 %.1f%% | 0.95-1.05 %.1f%% | >1.05 %.1f%% | stock->ours %.2f deg  "
                                "[t==1 always = the socket IS our pose and blending is dead; t<1 varying = "
                                "the drawn pose is a LERP and t's variation is the judder]",
                                g_blend_t.load(std::memory_order_relaxed),
                                g_blend_a.load(std::memory_order_relaxed),
                                g_blend_b.load(std::memory_order_relaxed),
                                g_rr.nbanks,
                                s_tsum/n, s_tmn, s_tmx,
                                100.0*s_tb[0]/n, 100.0*s_tb[1]/n, 100.0*s_tb[2]/n,
                                100.0*s_tb[3]/n, 100.0*s_tb[4]/n, 100.0*s_tb[5]/n, a_ours);
                            s_tn = 0; for (int i = 0; i < 6; ++i) s_tb[i] = 0;
                            s_tsum = 0.0; s_tmn = 1.0e9f; s_tmx = -1.0e9f;
                        }
                    }
                }
            }

            // ---- SOCKGATE. Do the socket's departures land on the frames that MISSED the
            // gate-6 build-time window? If they do, the fix is to close the remaining misses and
            // no math needs touching. If the departures are spread evenly across both, the
            // landing window is innocent and the socket diverges for another reason.
            {
                const unsigned g6 = g_n8_gate6.load(std::memory_order_relaxed);
                const long long wms = g_n8_ms.load(std::memory_order_relaxed);
                const long long nowg = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                const long long age = (wms != 0) ? (nowg - wms) : -1;
                static uint32_t s_ng[2] = {0, 0}, s_bad[2] = {0, 0};
                static double  s_sg[2] = {0.0, 0.0};
                static float   s_mg[2] = {0.0f, 0.0f};
                static uint32_t s_age_n = 0; static double s_age_sum = 0.0; static long long s_age_mx = 0;
                const int b = (g6 != 0) ? 1 : 0;
                ++s_ng[b]; s_sg[b] += (double)af;
                if (af > s_mg[b]) s_mg[b] = af;
                if (af > 2.0f) ++s_bad[b];
                if (age >= 0) { ++s_age_n; s_age_sum += (double)age; if (age > s_age_mx) s_age_mx = age; }
                static uint32_t s_gn = 0;
                if ((++s_gn % 120u) == 0u) {
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] SOCKGATE gate6-LANDED n=%u mean %.4f worst %.4f deg over-2deg %u (%.1f%%) "
                        "| gate6-MISSED n=%u mean %.4f worst %.4f deg over-2deg %u (%.1f%%) "
                        "| our write age mean %.1f worst %lld ms  [if the departures sit only in the "
                        "MISSED column, close the landing window and the math is fine]",
                        s_ng[1], s_ng[1] ? s_sg[1] / s_ng[1] : 0.0, s_mg[1], s_bad[1],
                        s_ng[1] ? 100.0 * s_bad[1] / s_ng[1] : 0.0,
                        s_ng[0], s_ng[0] ? s_sg[0] / s_ng[0] : 0.0, s_mg[0], s_bad[0],
                        s_ng[0] ? 100.0 * s_bad[0] / s_ng[0] : 0.0,
                        s_age_n ? s_age_sum / s_age_n : 0.0, s_age_mx);
                    s_ng[0] = s_ng[1] = s_bad[0] = s_bad[1] = 0;
                    s_sg[0] = s_sg[1] = 0.0; s_mg[0] = s_mg[1] = 0.0f;
                    s_age_n = 0; s_age_sum = 0.0; s_age_mx = 0;
                }
            }
            if ((++s_ln % 120u) == 0u) {
                API::get()->log_info(
                    "[Halo-CampE-UEVR] SOCKLINK our-node8 vs socket: fwd angle %.4f deg "
                    "(range %.4f to %.4f, spread %.4f), worst per-frame change %.4f deg  "
                    "[CONSTANT = our write owns the socket; MOVING = the socket rides a parent "
                    "the game animates and we never write, and that motion is the leak]",
                    af, s_mn_f, s_mx_f, s_mx_f - s_mn_f, s_mx_df);
                s_mn_f = 1e9f; s_mx_f = -1e9f; s_mx_df = 0.0f;
            }
        }
    }
    // ---- SOCKOWNER (2026-09-12). WHICH BONE ACTUALLY OWNS THE WEAPON SOCKET?
    //
    // SOCKLINK settled that ours does not: the angle between our written node 8 and the socket's
    // world rotation ranges over 31.9 to 121.9 degrees and changes by up to 30.0 degrees in one
    // frame. So we compute the correct pose and write it to a bone that does not determine where
    // the weapon is drawn, which is exactly the player's "the aim ray is what determines where the
    // characters wepaon and arms are posed".
    //
    // Rather than guess the bone name through the reflection API, survey every node. Bring the
    // socket's world rotation into mesh space with the live mesh rotation mode 12 already
    // publishes, then for each palette node measure the angle to it and track that angle's SPREAD
    // over a window. The node that owns the socket is rigidly related to it, so its spread
    // collapses to nearly zero while every other node's wanders. The five smallest win.
    //
    // The palette is read from the render thread through the address published at the write, since
    // live_palette_for is bound to the sim thread's TLS and returns null here.
    if (g_cfg.palette_weapon_log) {
        static uint32_t s_ow_n = 0;
        static float s_mn[FP_NODE_COUNT], s_mx[FP_NODE_COUNT];
        static bool  s_ow_init = false;
        if (!s_ow_init) {
            for (int i = 0; i < FP_NODE_COUNT; ++i) { s_mn[i] = 1.0e9f; s_mx[i] = -1.0e9f; }
            s_ow_init = true;
        }
        // the mesh rotation the render thread published for mode 12, so world -> mesh is exact
        Quat mq{0.0f, 0.0f, 0.0f, 1.0f};
        bool mq_ok = false;
        for (int i = 0; i < 4; ++i) {
            const uint32_t s0 = g_qr_seq.load(std::memory_order_acquire);
            const Quat q{g_qr_x.load(std::memory_order_relaxed), g_qr_y.load(std::memory_order_relaxed),
                         g_qr_z.load(std::memory_order_relaxed), g_qr_w.load(std::memory_order_relaxed)};
            if ((s0 & 1u) || g_qr_seq.load(std::memory_order_acquire) != s0) continue;
            const float n = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
            if (n > 0.9f && n < 1.1f) { mq = Quat{q.x/n, q.y/n, q.z/n, q.w/n}; mq_ok = true; }
            break;
        }
        const uintptr_t a8 = g_n8_addr.load(std::memory_order_relaxed);
        if (mq_ok && a8 != 0) {
            const PaletteNode* base = (const PaletteNode*)(a8 - 8 * sizeof(PaletteNode));
            if (!IsBadReadPtr((void*)base, sizeof(PaletteNode) * FP_NODE_COUNT)) {
                // The socket in MESH space AND in BLAM convention. The first survey compared a
                // Blam column against a UE axis and ranked noise: node 0 and node 4 came back
                // with identical spreads to three decimals, which no two real bones can do.
                const Quat sm2 = quat_mul(quat_conj(mq), sq);
                const Vec3 ax = quat_rotate(sm2, Vec3{1.0f, 0.0f, 0.0f});
                const Vec3 ay = quat_rotate(sm2, Vec3{0.0f, 1.0f, 0.0f});
                const Vec3 az = quat_rotate(sm2, Vec3{0.0f, 0.0f, 1.0f});
                const auto sbf = ue_to_blam(ax);
                const auto sbl = ue_to_blam(Vec3{-ay.x, -ay.y, -ay.z});
                const auto sbu = ue_to_blam(az);
                for (int k = 0; k < FP_NODE_COUNT; ++k) {
                    const NodeMatrix& nf = base[k].forward;
                    const NodeMatrix& nl = base[k].left;
                    const NodeMatrix& nu = base[k].up;
                    const float lf = std::sqrt(nf.x*nf.x + nf.y*nf.y + nf.z*nf.z);
                    const float ll = std::sqrt(nl.x*nl.x + nl.y*nl.y + nl.z*nl.z);
                    const float lu = std::sqrt(nu.x*nu.x + nu.y*nu.y + nu.z*nu.z);
                    // skip identity, unused and degenerate nodes. The confounded first survey
                    // ranked exactly these: node 0 and node 4 came back with identical spreads to
                    // three decimals, which no two real bones can do.
                    if (!(lf > 0.5f && ll > 0.5f && lu > 0.5f)) continue;
                    if (!std::isfinite(lf) || !std::isfinite(ll) || !std::isfinite(lu)) continue;
                    const float tr3 = (nf.x*sbf.x + nf.y*sbf.y + nf.z*sbf.z) / lf
                                    + (nl.x*sbl.x + nl.y*sbl.y + nl.z*sbl.z) / ll
                                    + (nu.x*sbu.x + nu.y*sbu.y + nu.z*sbu.z) / lu;
                    float ca2 = (tr3 - 1.0f) * 0.5f;
                    if (ca2 > 1.0f) ca2 = 1.0f; if (ca2 < -1.0f) ca2 = -1.0f;
                    const float ang = std::acos(ca2) * RAD2DEG;
                    if (!std::isfinite(ang)) continue;
                    if (ang < s_mn[k]) s_mn[k] = ang;
                    if (ang > s_mx[k]) s_mx[k] = ang;
                }
                if ((++s_ow_n % 240u) == 0u) {
                    int best[5] = {-1, -1, -1, -1, -1};
                    float bs[5] = {1.0e9f, 1.0e9f, 1.0e9f, 1.0e9f, 1.0e9f};
                    for (int k = 0; k < FP_NODE_COUNT; ++k) {
                        if (s_mx[k] < s_mn[k]) continue;
                        const float sp = s_mx[k] - s_mn[k];
                        for (int j = 0; j < 5; ++j) {
                            if (sp < bs[j]) {
                                for (int m = 4; m > j; --m) { bs[m] = bs[m-1]; best[m] = best[m-1]; }
                                bs[j] = sp; best[j] = k;
                                break;
                            }
                        }
                    }
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] SOCKOWNER most rigid vs the weapon socket, over %u frames: "
                        "node %d spread %.3f | node %d %.3f | node %d %.3f | node %d %.3f | node %d %.3f "
                        "| node 8 (the one we write) spread %.3f deg  [the owner's spread collapses to "
                        "~0; ours does not, so we are writing the wrong bone]",
                        240u, best[0], bs[0], best[1], bs[1], best[2], bs[2], best[3], bs[3],
                        best[4], bs[4], (s_mx[8] >= s_mn[8]) ? (s_mx[8] - s_mn[8]) : -1.0f);
                    for (int k = 0; k < FP_NODE_COUNT; ++k) { s_mn[k] = 1.0e9f; s_mx[k] = -1.0e9f; }
                }
            }
        }
    }
    s_ps = sq; s_ph = hq; s_have = true;
}

// ---- SAMEINST (2026-09-12). The post-blend residual, measured with no timing leak, and split
// between AIM and HAND.
//
// LIVEVSBANK found the residual between the drawn socket and the (identical) banks JUMPS: per
// frame mean 0.52-1.64 deg, worst 9.69, over 1 deg on 10-53% of frames. BANKDIFF showed the blend
// is a no-op and the fallback buffer matches the banks on 97.5-100% of frames, so the judder
// enters AFTER the banks, where sim+0x46A2E0 composes every node with a root node handed in by its
// caller. the player's standing claim is that the aim ray decides the pose, which is exactly what a
// per-tick aim-derived root would do.
//
// One weakness had to go first: that residual brought the socket into mesh space with a mesh
// rotation published by the render refresh, possibly sampled at a different moment than the
// socket. Here the component rotation is read back to back with the socket in the same callback.
// Then the residual's per-frame jump is correlated against the AIM step (ControlRotation, read at
// the same moment) and against the HAND step (controller read at the same moment). Whichever
// correlates names what drives the error that enters after our write.
void blam_palette_sameinst_probe(float sx, float sy, float sz, float sw,
                                 float cx, float cy, float cz, float cw,
                                 float aim_pitch, float aim_yaw, bool aim_ok) {
    if (!g_cfg.palette_weapon_log) return;
    const uint32_t brs = g_rr.seq.load(std::memory_order_acquire);
    const PaletteNode* bk0 = g_rr.bank[0];
    if ((brs & 1u) || g_rr.nbanks < 1 || bk0 == nullptr || IsBadReadPtr((void*)bk0, sizeof(PaletteNode) * 9)) return;
    const PaletteNode n8 = bk0[8];
    const float nl = std::sqrt(n8.forward.x*n8.forward.x + n8.forward.y*n8.forward.y + n8.forward.z*n8.forward.z);
    if (!(nl > 0.5f)) return;
    const Quat sq{sx, sy, sz, sw};
    const Quat cq{cx, cy, cz, cw};
    const Quat sm = quat_mul(quat_conj(cq), sq);                       // socket in mesh space, same instant
    const Vec3 rx = quat_rotate(sm, Vec3{1.0f, 0.0f, 0.0f});
    const Vec3 ry = quat_rotate(sm, Vec3{0.0f, 1.0f, 0.0f});
    const Vec3 rz = quat_rotate(sm, Vec3{0.0f, 0.0f, 1.0f});
    const auto bf = ue_to_blam(rx);
    const auto bl = ue_to_blam(Vec3{-ry.x, -ry.y, -ry.z});
    const auto bu = ue_to_blam(rz);
    const float tr = (n8.forward.x*bf.x + n8.forward.y*bf.y + n8.forward.z*bf.z) / nl
                   + (n8.left.x*bl.x + n8.left.y*bl.y + n8.left.z*bl.z)
                   + (n8.up.x*bu.x + n8.up.y*bu.y + n8.up.z*bu.z);
    float c = (tr - 1.0f) * 0.5f;
    if (c > 1.0f) c = 1.0f; if (c < -1.0f) c = -1.0f;
    const float resid = std::acos(c) * RAD2DEG;

    // ---- ROOTFIT (2026-09-12). From the headset, firmly: the aim decides the pose. Taken as given; this
    // does not try to prove it again, it extracts the SHAPE so the correction can be written.
    //
    // Downstream of our write, sim+0x46A2E0 composes every node as root x src. So with the socket
    // brought into mesh space at the same instant, rel = socket_mesh x transpose(bank_node) IS the
    // root rotation the game applied on top of our bone this frame (if the node map sends 8 to 8).
    // Decompose rel into pitch-like (about Blam lateral Y), yaw-like (about Blam up Z) and
    // roll-like (about Blam forward X) components and correlate each with the game's aim pitch and
    // aim yaw read at the same moment. Whichever component follows the aim, with what slope, is
    // the term to cancel -- computed from ControlRotation in our own write, before the poser runs,
    // so there is no one-frame lag in the correction.
    {
        auto nrm3 = [](float x, float y, float z) { return std::sqrt(x*x + y*y + z*z); };
        const float lf = nrm3(n8.forward.x, n8.forward.y, n8.forward.z);
        const float ll = nrm3(n8.left.x, n8.left.y, n8.left.z);
        const float lu = nrm3(n8.up.x, n8.up.y, n8.up.z);
        if (lf > 0.5f && ll > 0.5f && lu > 0.5f) {
            const float Nf[3] = {n8.forward.x/lf, n8.forward.y/lf, n8.forward.z/lf};
            const float Nl[3] = {n8.left.x/ll, n8.left.y/ll, n8.left.z/ll};
            const float Nu[3] = {n8.up.x/lu, n8.up.y/lu, n8.up.z/lu};
            const float Sf[3] = {bf.x, bf.y, bf.z};
            const float Sl[3] = {bl.x, bl.y, bl.z};
            const float Su[3] = {bu.x, bu.y, bu.z};
            float M[3][3];
            for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
                M[i][j] = Sf[i]*Nf[j] + Sl[i]*Nl[j] + Su[i]*Nu[j];
            auto clamp1 = [](float v) { return v > 1.0f ? 1.0f : (v < -1.0f ? -1.0f : v); };
            const float rollish  = std::asin(clamp1(0.5f * (M[2][1] - M[1][2]))) * RAD2DEG;   // about X
            const float pitchish = std::asin(clamp1(0.5f * (M[0][2] - M[2][0]))) * RAD2DEG;   // about Y
            const float yawish   = std::asin(clamp1(0.5f * (M[1][0] - M[0][1]))) * RAD2DEG;   // about Z
            if (aim_ok) {
                struct Acc { double x=0,y=0,xx=0,yy=0,xy=0; uint32_t n=0;
                             void add(double a, double b){ ++n; x+=a; y+=b; xx+=a*a; yy+=b*b; xy+=a*b; }
                             double corr() const { if (n<10) return 0; const double N=n;
                                 const double vx=xx-x*x/N, vy=yy-y*y/N, cv=xy-x*y/N;
                                 return (vx>1e-12&&vy>1e-12)? cv/std::sqrt(vx*vy):0; }
                             double slope() const { if (n<10) return 0; const double N=n;
                                 const double vx=xx-x*x/N, cv=xy-x*y/N; return vx>1e-12? cv/vx:0; }
                             double meany() const { return n? y/n:0; } };
                static Acc p_ap, p_ay, y_ap, y_ay, r_ap, r_ay;
                static uint32_t s_rf = 0;
                // ---- LAGFIT (2026-09-12). ROOTFIT: the root components do NOT follow the aim ANGLE
                // (signed means 0.00-0.3 deg, corr with aim ~0) while the residual magnitude is
                // 1.3-2.1 deg, so it sign-flips. SAMEINST: its per-frame jump tracks the AIM STEP
                // (corr +0.49..+0.53, vs hand +0.27..+0.36) at ~0.7 of the step. That is a TIMING
                // signature: the game applies an aim from one moment, we cancel an aim from another,
                // and about one frame of aim motion lands on the gun, reversing with the motion.
                // This fits it directly. (a) Which published camera is our write stale against:
                // correlate the signed residual with (aim now - build camera) and with
                // (aim now - refresh camera); the one with corr near 1 is the sample to replace, and
                // its slope is the fraction of that gap that reaches the gun. (b) The lag in frames:
                // correlate the residual against the signed aim step at offsets -2..+2.
                {
                    auto wrp = [](float d) { while (d > 180.0f) d -= 360.0f; while (d < -180.0f) d += 360.0f; return d; };
                    const float dPb = aim_pitch - g_rr.cam_p.load(std::memory_order_relaxed);
                    const float dYb = wrp(aim_yaw - g_rr.cam_y.load(std::memory_order_relaxed));
                    const float dPr = aim_pitch - g_rcam_p.load(std::memory_order_relaxed);
                    const float dYr = wrp(aim_yaw - g_rcam_y.load(std::memory_order_relaxed));
                    static Acc lp_b, lp_r, ly_b, ly_r;
                    lp_b.add(dPb, pitchish); lp_r.add(dPr, pitchish);
                    ly_b.add(dYb, yawish);   ly_r.add(dYr, yawish);
                    // lag scan: history of signed aim steps and residual components
                    static float h_dp[5] = {0,0,0,0,0}, h_dy[5] = {0,0,0,0,0};
                    static float h_rp[5] = {0,0,0,0,0}, h_ry[5] = {0,0,0,0,0};
                    static float s_lp = 0.0f, s_ly = 0.0f; static bool s_lh2 = false; static int s_hn = 0;
                    if (s_lh2) {
                        const float sdp = aim_pitch - s_lp, sdy = wrp(aim_yaw - s_ly);
                        for (int k = 4; k > 0; --k) { h_dp[k]=h_dp[k-1]; h_dy[k]=h_dy[k-1]; h_rp[k]=h_rp[k-1]; h_ry[k]=h_ry[k-1]; }
                        h_dp[0] = sdp; h_dy[0] = sdy; h_rp[0] = pitchish; h_ry[0] = yawish;
                        if (s_hn < 5) ++s_hn;
                        if (s_hn >= 5) {
                            // residual at the MIDDLE (index 2) against the step at index 0..4:
                            // index 0 = step 2 frames LATER than the residual, 4 = 2 frames EARLIER.
                            static Acc lagP[5], lagY[5];
                            for (int k = 0; k < 5; ++k) { lagP[k].add(h_dp[k], h_rp[2]); lagY[k].add(h_dy[k], h_ry[2]); }
                            static uint32_t s_lgn = 0;
                            if ((++s_lgn % 240u) == 0u) {
                                API::get()->log_info(
                                    "[Halo-CampE-UEVR] LAGFIT residual vs (aim now - BUILD cam): pitch corr %+.3f slope %+.3f, "
                                    "yaw corr %+.3f slope %+.3f | vs (aim now - REFRESH cam): pitch corr %+.3f slope %+.3f, "
                                    "yaw corr %+.3f slope %+.3f | lag scan corr(residual, aim step) at step offset "
                                    "+2,+1,0,-1,-2 frames: PITCH %+.2f %+.2f %+.2f %+.2f %+.2f  YAW %+.2f %+.2f %+.2f %+.2f %+.2f  "
                                    "[the camera with corr near 1 is the stale sample; its slope is how much of the gap "
                                    "reaches the gun; the peak offset is the lag in frames]",
                                    lp_b.corr(), lp_b.slope(), ly_b.corr(), ly_b.slope(),
                                    lp_r.corr(), lp_r.slope(), ly_r.corr(), ly_r.slope(),
                                    lagP[0].corr(), lagP[1].corr(), lagP[2].corr(), lagP[3].corr(), lagP[4].corr(),
                                    lagY[0].corr(), lagY[1].corr(), lagY[2].corr(), lagY[3].corr(), lagY[4].corr());
                                lp_b = Acc(); lp_r = Acc(); ly_b = Acc(); ly_r = Acc();
                                for (int k = 0; k < 5; ++k) { lagP[k] = Acc(); lagY[k] = Acc(); }
                            }
                        }
                    }
                    s_lp = aim_pitch; s_ly = aim_yaw; s_lh2 = true;
                }
                p_ap.add(aim_pitch, pitchish); p_ay.add(aim_yaw, pitchish);
                y_ap.add(aim_pitch, yawish);   y_ay.add(aim_yaw, yawish);
                r_ap.add(aim_pitch, rollish);  r_ay.add(aim_yaw, rollish);
                if ((++s_rf % 240u) == 0u) {
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] ROOTFIT root-on-our-bone components vs AIM (same instant, 240 frames): "
                        "PITCHish mean %.3f deg corr(aimP) %+.3f slope %+.3f corr(aimY) %+.3f | "
                        "YAWish mean %.3f corr(aimP) %+.3f corr(aimY) %+.3f slope %+.3f | "
                        "ROLLish mean %.3f corr(aimP) %+.3f corr(aimY) %+.3f  [the component with |corr| near 1 "
                        "and its slope is exactly what to cancel from ControlRotation inside our own write]",
                        p_ap.meany(), p_ap.corr(), p_ap.slope(), p_ay.corr(),
                        y_ap.meany(), y_ap.corr(), y_ay.corr(), y_ay.slope(),
                        r_ap.meany(), r_ap.corr(), r_ay.corr());
                    p_ap = Acc(); p_ay = Acc(); y_ap = Acc(); y_ay = Acc(); r_ap = Acc(); r_ay = Acc();
                }
            }
        }
    }

    // hand, same moment
    const auto hidx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                          : API::VR::get_right_controller_index();
    Vec3 hp{}; Quat hq{};
    const bool hok = (hidx >= 0) && get_pose(hidx, &hp, &hq, /*use_aim=*/true);

    static bool s_have = false;
    static float s_pres = 0.0f, s_pap = 0.0f, s_pay = 0.0f;
    static Quat s_phq{};
    static uint32_t s_n = 0;
    static double s_res = 0.0, s_jump = 0.0, s_big = 0.0;
    static float s_jmx = 0.0f;
    // Pearson accumulators: jump vs aim step, jump vs hand step
    static double a_x = 0, a_y = 0, a_xx = 0, a_yy = 0, a_xy = 0; static uint32_t a_n = 0;
    static double h_x = 0, h_y = 0, h_xx = 0, h_yy = 0, h_xy = 0; static uint32_t h_n = 0;
    if (s_have) {
        const float jump = std::fabs(resid - s_pres);
        ++s_n; s_res += resid; s_jump += jump; if (jump > s_jmx) s_jmx = jump; if (jump > 1.0f) s_big += 1.0;
        if (aim_ok) {
            float dy = aim_yaw - s_pay; while (dy > 180.0f) dy -= 360.0f; while (dy < -180.0f) dy += 360.0f;
            const float dp = aim_pitch - s_pap;
            const double as = std::sqrt((double)dy*dy + (double)dp*dp);
            ++a_n; a_x += as; a_y += jump; a_xx += as*as; a_yy += (double)jump*jump; a_xy += as*jump;
        }
        if (hok) {
            float d = s_phq.x*hq.x + s_phq.y*hq.y + s_phq.z*hq.z + s_phq.w*hq.w;
            if (d < 0.0f) d = -d; if (d > 1.0f) d = 1.0f;
            const double hs = 2.0 * std::acos(d) * RAD2DEG;
            ++h_n; h_x += hs; h_y += jump; h_xx += hs*hs; h_yy += (double)jump*jump; h_xy += hs*jump;
        }
        if ((s_n % 120u) == 0u) {
            auto corr = [](double n, double x, double y, double xx, double yy, double xy) {
                if (n < 10) return 0.0;
                const double vx = xx - x*x/n, vy = yy - y*y/n, cv = xy - x*y/n;
                return (vx > 1e-12 && vy > 1e-12) ? cv / std::sqrt(vx*vy) : 0.0;
            };
            const double n = (double)s_n;
            API::get()->log_info(
                "[Halo-CampE-UEVR] SAMEINST residual socket-vs-bank (same-instant mesh) mean %.4f deg | per-frame "
                "jump mean %.4f worst %.4f, over 1 deg %.1f%% | corr(jump, AIM step) %+.3f n=%u mean aim step %.4f | "
                "corr(jump, HAND step) %+.3f n=%u mean hand step %.4f  [jumps near 0 = the residual was my timing "
                "leak; jumps that track AIM = the aim-driven root compose puts the judder on the drawn weapon]",
                s_res/n, s_jump/n, s_jmx, 100.0*s_big/n,
                corr((double)a_n, a_x, a_y, a_xx, a_yy, a_xy), a_n, a_n ? a_x/a_n : 0.0,
                corr((double)h_n, h_x, h_y, h_xx, h_yy, h_xy), h_n, h_n ? h_x/h_n : 0.0);
            s_n = 0; s_res = s_jump = s_big = 0.0; s_jmx = 0.0f;
            a_x = a_y = a_xx = a_yy = a_xy = 0.0; a_n = 0;
            h_x = h_y = h_xx = h_yy = h_xy = 0.0; h_n = 0;
        }
    }
    // ---- TICKPHASE (2026-09-12). Is the Unreal side interpolating the skeleton between sim ticks?
    // CAMBONE: the renderer's inverse(X) correction uses a camera bone that is bit-for-bit constant
    // (0.0000 deg/tick), so it cannot judder. The residual between the UE component frame and our
    // (correct, identical-in-both-banks) bone still jumps by ~0.7 of each render-frame aim step.
    // The sim ticks at ~45 Hz and the renderer runs at ~72 Hz. If the UE side lerps the skeleton
    // between the last two sim-tick poses by how far this render frame is into the tick, the drawn
    // bone trails the aim by (1 - phase) of a tick's motion while anything read fresh does not --
    // and that residual would change size with the phase. So bucket the residual's jump, normalised
    // by the aim step, by tick phase. A strong phase dependence is interpolation; a flat line is not.
    {
        static long long s_last_tick = 0, s_prev_tick_seen = 0;
        static double s_period = 22.0;                                // ms, refined below
        static float s_tp_prev_res = 0.0f, s_tp_pap = 0.0f, s_tp_pay = 0.0f;
        static bool s_tp_h = false;
        static double s_bj[3] = {0,0,0}, s_ba[3] = {0,0,0};
        static uint32_t s_bn[3] = {0,0,0}, s_tpn = 0;
        const long long tick = g_tick_cam_ms.load(std::memory_order_relaxed);
        const long long nowms = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (tick != 0 && tick != s_last_tick) {
            if (s_last_tick != 0) {
                const double iv = (double)(tick - s_last_tick);
                if (iv > 5.0 && iv < 80.0) s_period += 0.05 * (iv - s_period);
            }
            s_last_tick = tick;
        }
        if (s_tp_h && aim_ok && tick != 0) {
            double phase = (double)(nowms - tick) / s_period;
            if (phase < 0.0) phase = 0.0;
            if (phase > 0.999) phase = 0.999;
            float dyy = aim_yaw - s_tp_pay; while (dyy > 180.0f) dyy -= 360.0f; while (dyy < -180.0f) dyy += 360.0f;
            const float dpp = aim_pitch - s_tp_pap;
            const double astep = std::sqrt((double)dyy*dyy + (double)dpp*dpp);
            const double jump = std::fabs((double)resid - s_tp_prev_res);
            if (astep > 0.3) {
                const int b = (phase < 0.3333) ? 0 : ((phase < 0.6667) ? 1 : 2);
                ++s_bn[b]; s_bj[b] += jump; s_ba[b] += astep;
            }
            if ((++s_tpn % 240u) == 0u) {
                auto rat = [&](int b) { return s_ba[b] > 1e-9 ? s_bj[b] / s_ba[b] : 0.0; };
                API::get()->log_info(
                    "[Halo-CampE-UEVR] TICKPHASE residual jump / aim step by render-frame phase inside the sim tick "
                    "(tick period %.1f ms): early 0-0.33 n=%u ratio %.3f | mid 0.33-0.67 n=%u ratio %.3f | "
                    "late 0.67-1 n=%u ratio %.3f  [a ratio that changes strongly with phase = the UE side is "
                    "interpolating the skeleton between sim ticks; flat = it is not]",
                    s_period, s_bn[0], rat(0), s_bn[1], rat(1), s_bn[2], rat(2));
                for (int i = 0; i < 3; ++i) { s_bj[i] = s_ba[i] = 0.0; s_bn[i] = 0; }
            }
        }
        s_tp_prev_res = resid; s_tp_pap = aim_pitch; s_tp_pay = aim_yaw; s_tp_h = true;
        (void)s_prev_tick_seen;
    }
    s_pres = resid; s_pap = aim_pitch; s_pay = aim_yaw; s_phq = hq; s_have = true;
}

void blam_palette_sync_capture(float aqx, float aqy, float aqz, float aqw,
                               float apx, float apy, float apz,
                               float gqx, float gqy, float gqz, float gqw,
                               float gpx, float gpy, float gpz,
                               float ctl_pitch, float ctl_yaw) {
    g_sync.seq.fetch_add(1u, std::memory_order_acq_rel);
    g_sync.aqx.store(aqx, std::memory_order_relaxed); g_sync.aqy.store(aqy, std::memory_order_relaxed);
    g_sync.aqz.store(aqz, std::memory_order_relaxed); g_sync.aqw.store(aqw, std::memory_order_relaxed);
    g_sync.apx.store(apx, std::memory_order_relaxed); g_sync.apy.store(apy, std::memory_order_relaxed);
    g_sync.apz.store(apz, std::memory_order_relaxed);
    g_sync.gqx.store(gqx, std::memory_order_relaxed); g_sync.gqy.store(gqy, std::memory_order_relaxed);
    g_sync.gqz.store(gqz, std::memory_order_relaxed); g_sync.gqw.store(gqw, std::memory_order_relaxed);
    g_sync.gpx.store(gpx, std::memory_order_relaxed); g_sync.gpy.store(gpy, std::memory_order_relaxed);
    g_sync.gpz.store(gpz, std::memory_order_relaxed);
    g_sync.cp.store(ctl_pitch, std::memory_order_relaxed); g_sync.cy.store(ctl_yaw, std::memory_order_relaxed);
    g_sync.ms.store((long long)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count(), std::memory_order_relaxed);
    g_sync.seq.fetch_add(1u, std::memory_order_acq_rel);
}

void blam_palette_stamp_bank() {
    if (g_cfg.stomp_log == 0 || g_cfg.pal_render == 0 || !palette_weapon_mode()) return;
    // Point 13: the LIVE palette's node 8 against our last write, same probe as the banks.
    // The gate-7 failure proved the capture pass re-fills the banks from somewhere each tick
    // with the builder suppressed -- if that source is the live palette, ITS state at the copy
    // instant is what renders, and this row shows whose bytes it holds at frame start.
    {
        LiveSlot lv{};
        PaletteNode* lp = live_palette_for(0, 0, &lv);
        if (lp != nullptr && lv.count > 8 && !IsBadReadPtr(lp, sizeof(PaletteNode) * 9)) {
            const float wx = g_dbg_node8_x.load(std::memory_order_relaxed);
            const float wy = g_dbg_node8_y.load(std::memory_order_relaxed);
            const float wz = g_dbg_node8_z.load(std::memory_order_relaxed);
            const float dx = lp[8].position.x - wx, dy = lp[8].position.y - wy, dz = lp[8].position.z - wz;
            stomp_mark(13, lp[8].position.y, std::sqrt(dx * dx + dy * dy + dz * dz) * 304.8f,
                       0.0f, lp[8].position.x);
        }
    }
    const uint32_t s0 = g_rr.seq.load(std::memory_order_acquire);
    if (s0 & 1u) return;
    const int nb = g_rr.nbanks; const int32_t cnt = g_rr.count;
    if (nb <= 0 || cnt <= 8) return;
    const float wx = g_dbg_node8_x.load(std::memory_order_relaxed);
    const float wy = g_dbg_node8_y.load(std::memory_order_relaxed);
    const float wz = g_dbg_node8_z.load(std::memory_order_relaxed);
    // Points 14/15: the RENDER-SIDE arenas. Static RE (2026-09-11 night) found the renderer's
    // bank blend selecting between TWO more shared arenas at sim+0x1831228/+0x1831230 -- the
    // pair the screen consumes, filled by a copy stage from the sim arena we write. Whose bytes
    // they hold at frame start is the delivery verdict. Plain global reads, thread-safe.
    {
        const HMODULE sim14 = GetModuleHandleA("HaloSimulation_tag_release.dll");
        if (sim14 != nullptr) {
            for (int a = 0; a < 2; ++a) {
                const uintptr_t prva = 0x1831228 + (uintptr_t)a * 8;
                if (!readable((const void*)((uintptr_t)sim14 + prva), 8)) continue;
                auto* sh = *reinterpret_cast<uint8_t**>((uintptr_t)sim14 + prva);
                if (sh == nullptr) continue;
                auto* pal = reinterpret_cast<PaletteNode*>(sh + OFF_CAPTURE_PALETTE);
                if (!readable(pal, sizeof(PaletteNode) * 9)) continue;
                const float dx = pal[8].position.x - wx, dy = pal[8].position.y - wy, dz = pal[8].position.z - wz;
                stomp_mark(14 + a, pal[8].position.y, std::sqrt(dx * dx + dy * dy + dz * dz) * 304.8f,
                           0.0f, pal[8].position.x);
            }
        }
    }
    for (int i = 0; i < nb && i < 2; ++i) {
        PaletteNode* b = g_rr.bank[i];
        if (b == nullptr || IsBadReadPtr(b, sizeof(PaletteNode) * 9)) continue;
        const float dx = b[8].position.x - wx, dy = b[8].position.y - wy, dz = b[8].position.z - wz;
        // yaw column carries the raw bank node-8 y, e0 the distance from our last write in cm,
        // e1 which bank, e2 the raw bank node-8 x.
        stomp_mark(10 + i, b[8].position.y, std::sqrt(dx * dx + dy * dy + dz * dz) * 304.8f,
                   (float)i, b[8].position.x);
    }
}
// ---- THE PER-FRAME REPUBLISH (doctrine at the Config key). RENDER THREAD, once per frame,
// BEFORE the WPNERR sampler. The publish tail's math exactly: q_ro recenter, standing-origin
// anchor, VR->UE swizzle, the stashed rigid grip offset by the publisher's own rule, normalize,
// store. Refreshes only -- if the tick publisher has not validated a pose, this does nothing.
void blam_palette_republish_frame() {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    if (g_cfg.pal_pub_frame == 0) return;
    if (!palette_weapon_mode()) return;
    if (!g_p_valid.load(std::memory_order_acquire)) return;
    const auto hidx = API::VR::get_hmd_index();
    const auto cidx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                          : API::VR::get_right_controller_index();
    if (hidx < 0 || cidx < 0) return;
    Vec3 hpos{}, gpos{}, apos{}; Quat hq{}, gq{}, aq{};
    if (!get_pose(hidx, &hpos, &hq, /*use_aim=*/false) ||
        !get_pose(cidx, &gpos, &gq, /*use_aim=*/false) ||
        !get_pose(cidx, &apos, &aq, /*use_aim=*/true)) return;
    // The dead-tracking reach gate, replicated from the tick publisher.
    {
        const float dx = gpos.x - hpos.x, dy = gpos.y - hpos.y, dz = gpos.z - hpos.z;
        const float reach2 = dx * dx + dy * dy + dz * dz;
        const bool grip_zero = std::fabs(gpos.x) < 1e-6f && std::fabs(gpos.y) < 1e-6f &&
                               std::fabs(gpos.z) < 1e-6f;
        if (grip_zero || !std::isfinite(reach2) || reach2 > 1.5f * 1.5f) return;
    }
    // THE FILTER AT FRAME CADENCE (2026-09-11 evening). The republish alone showed the gun the
    // raw per-frame tracking stream and it JERKED AT REST -- the controller stream itself is
    // noisy frame to frame and the grip lever is a microscope for it. The tick path hid some of
    // it by accident of cadence. So the one-euro filter runs HERE, on the frame-fresh samples,
    // the cadence it was designed for (and the only one that can help): rest tremor dies, real
    // motion opens the cutoff within a sample. Own state, frame cadence; the tick publisher's
    // instance keeps its own.
    if (g_cfg.pose_filter != 0) {
        struct Euro { float x = 0.0f, dx = 0.0f; bool have = false; };
        static Euro s_fp[3]; static Euro s_fq[4]; static long long s_fms = 0;
        const long long fnow = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        float fdt = (s_fms > 0) ? (float)(fnow - s_fms) * 0.001f : 0.014f;
        s_fms = fnow;
        if (fdt > 0.0005f && fdt < 0.25f) {
            auto ealpha = [](float fc, float dt) { const float r = 6.2831853f * fc * dt; return r / (r + 1.0f); };
            auto euro = [&](Euro& e, float v, float beta) {
                if (!e.have) { e.x = v; e.dx = 0.0f; e.have = true; return v; }
                const float dv = (v - e.x) / fdt;
                e.dx += ealpha(g_cfg.pose_filter_dcut, fdt) * (dv - e.dx);
                const float fc = g_cfg.pose_filter_min + beta * std::fabs(e.dx);
                e.x += ealpha(fc, fdt) * (v - e.x);
                return e.x;
            };
            gpos.x = euro(s_fp[0], gpos.x, g_cfg.pose_filter_beta);
            gpos.y = euro(s_fp[1], gpos.y, g_cfg.pose_filter_beta);
            gpos.z = euro(s_fp[2], gpos.z, g_cfg.pose_filter_beta);
            if (s_fq[3].have) {
                const float d = aq.x * s_fq[0].x + aq.y * s_fq[1].x + aq.z * s_fq[2].x + aq.w * s_fq[3].x;
                if (d < 0.0f) { aq.x = -aq.x; aq.y = -aq.y; aq.z = -aq.z; aq.w = -aq.w; }
            }
            aq.x = euro(s_fq[0], aq.x, g_cfg.pose_filter_rbeta);
            aq.y = euro(s_fq[1], aq.y, g_cfg.pose_filter_rbeta);
            aq.z = euro(s_fq[2], aq.z, g_cfg.pose_filter_rbeta);
            aq.w = euro(s_fq[3], aq.w, g_cfg.pose_filter_rbeta);
            const float n = std::sqrt(aq.x * aq.x + aq.y * aq.y + aq.z * aq.z + aq.w * aq.w);
            if (n > 1e-6f) { aq.x /= n; aq.y /= n; aq.z /= n; aq.w /= n; }
        }
    }
    Quat q_ro{0.0f, 0.0f, 0.0f, 1.0f};
    if (g_cfg.rig_view_yaw != 0.0f) {
        const auto ro = API::VR::get_rotation_offset();
        q_ro = Quat{ro.x, ro.y, ro.z, ro.w};
        if (g_cfg.rig_view_yaw < 0.0f) q_ro = quat_conj(q_ro);
    }
    const auto so = API::VR::get_standing_origin();
    const Vec3 anchor{so.x, so.y, so.z};
    const Vec3 d_vr = quat_rotate(q_ro, Vec3{gpos.x - anchor.x, gpos.y - anchor.y, gpos.z - anchor.z});
    Vec3 d_xr{-d_vr.z, d_vr.x, d_vr.y};
    const Quat aim_ro = quat_mul(q_ro, aq);
    Quat pose_xr{-aim_ro.z, aim_ro.x, aim_ro.y, -aim_ro.w};
    // The stashed rigid grip offset, applied by the publisher's exact rule: translation rotated
    // by the RAW pose, then the global rotation.
    {
        const Vec3 pos_t{g_p_offt_x.load(std::memory_order_relaxed), g_p_offt_y.load(std::memory_order_relaxed), g_p_offt_z.load(std::memory_order_relaxed)};
        const Quat rot_g{g_p_rotg_x.load(std::memory_order_relaxed), g_p_rotg_y.load(std::memory_order_relaxed), g_p_rotg_z.load(std::memory_order_relaxed), g_p_rotg_w.load(std::memory_order_relaxed)};
        const Vec3 t = quat_rotate(pose_xr, pos_t);
        d_xr = Vec3{d_xr.x + t.x, d_xr.y + t.y, d_xr.z + t.z};
        pose_xr = quat_mul(pose_xr, rot_g);
    }
    const float pn = std::sqrt(pose_xr.x * pose_xr.x + pose_xr.y * pose_xr.y +
                               pose_xr.z * pose_xr.z + pose_xr.w * pose_xr.w);
    if (pn < 1e-6f) return;
    pose_xr.x /= pn; pose_xr.y /= pn; pose_xr.z /= pn; pose_xr.w /= pn;
    // Head pitch for the arms, same extraction as the tick publisher.
    {
        const Quat h_ro = quat_mul(q_ro, hq);
        float hp = 0.0f, hy = 0.0f, hr = 0.0f;
        quat_to_rotator(-h_ro.z, h_ro.x, h_ro.y, -h_ro.w, &hp, &hy, &hr);
        g_p_head_pitch.store(hp, std::memory_order_relaxed);
    }
    g_p_raw_gx.store(gpos.x, std::memory_order_relaxed);
    g_p_raw_gy.store(gpos.y, std::memory_order_relaxed);
    g_p_raw_gz.store(gpos.z, std::memory_order_relaxed);
    g_p_raw_ax.store(aq.x, std::memory_order_relaxed);
    g_p_raw_ay.store(aq.y, std::memory_order_relaxed);
    g_p_raw_az.store(aq.z, std::memory_order_relaxed);
    g_p_raw_aw.store(aq.w, std::memory_order_relaxed);
    g_p_pub_ms.store((long long)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count(), std::memory_order_relaxed);
    g_p_seq.fetch_add(1, std::memory_order_acq_rel);
    g_p_grip_x.store(d_xr.x, std::memory_order_relaxed);
    g_p_grip_y.store(d_xr.y, std::memory_order_relaxed);
    g_p_grip_z.store(d_xr.z, std::memory_order_relaxed);
    g_p_aim_x.store(pose_xr.x, std::memory_order_relaxed);
    g_p_aim_y.store(pose_xr.y, std::memory_order_relaxed);
    g_p_aim_z.store(pose_xr.z, std::memory_order_relaxed);
    g_p_aim_w.store(pose_xr.w, std::memory_order_relaxed);
    g_p_seq.fetch_add(1, std::memory_order_acq_rel);
}
void blam_palette_wpnerr_frame() {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    const bool on = g_cfg.wpn_err_log != 0;
    if (!on) {
        if (s_we_was_on) { we_flush(); s_we_was_on = false; }
        return;
    }
    if (!s_we_was_on) { s_we_was_on = true; s_we_t0 = we_now_s(); s_we_prev_t = 0.0; s_we_prev_have = false; s_we_cam_have = false; s_we_sum_t = 0.0; }
    if (!g_p_valid.load(std::memory_order_acquire)) return;
    const auto cidx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                          : API::VR::get_right_controller_index();
    Vec3 gpos{}; Quat gq{}; Vec3 apos{}; Quat aq{};
    if (cidx < 0 || !get_pose(cidx, &gpos, &gq, /*use_aim=*/false) || !get_pose(cidx, &apos, &aq, /*use_aim=*/true)) return;
    const double t = we_now_s() - s_we_t0;
    const float dt = (s_we_prev_t > 0.0) ? (float)(t - s_we_prev_t) : 0.0f;
    s_we_prev_t = t;
    const long long now_ms = (long long)((we_now_s()) * 1000.0);
    // The error, raw vs raw.
    const Vec3 pg{g_p_raw_gx.load(std::memory_order_relaxed), g_p_raw_gy.load(std::memory_order_relaxed), g_p_raw_gz.load(std::memory_order_relaxed)};
    const Quat pa{g_p_raw_ax.load(std::memory_order_relaxed), g_p_raw_ay.load(std::memory_order_relaxed), g_p_raw_az.load(std::memory_order_relaxed), g_p_raw_aw.load(std::memory_order_relaxed)};
    const float ex = gpos.x - pg.x, ey = gpos.y - pg.y, ez = gpos.z - pg.z;
    const float err_cm = std::sqrt(ex * ex + ey * ey + ez * ez) * 100.0f;
    float d = aq.x * pa.x + aq.y * pa.y + aq.z * pa.z + aq.w * pa.w;
    if (d < 0.0f) d = -d; if (d > 1.0f) d = 1.0f;
    const float err_deg = 2.0f * std::acos(d) * 57.29578f;
    const float pub_age = (float)(now_ms - g_p_pub_ms.load(std::memory_order_relaxed));
    const float con_age = (float)(now_ms - g_p_con_ms.load(std::memory_order_relaxed));
    // Speeds: the camera in world cm/s (the run), the hand in room cm/s (the swing).
    float cam_v = 0.0f;
    {
        const float cx = g_cam_x.load(std::memory_order_relaxed), cy = g_cam_y.load(std::memory_order_relaxed), cz = g_cam_z.load(std::memory_order_relaxed);
        if (s_we_cam_have && dt > 1e-4f) {
            const float dx = cx - s_we_prev_cam[0], dy = cy - s_we_prev_cam[1], dz = cz - s_we_prev_cam[2];
            cam_v = std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
        }
        s_we_prev_cam[0] = cx; s_we_prev_cam[1] = cy; s_we_prev_cam[2] = cz; s_we_cam_have = true;
    }
    float hand_v = 0.0f;
    if (s_we_prev_have && dt > 1e-4f) {
        const float dx = gpos.x - s_we_prev_hand.x, dy = gpos.y - s_we_prev_hand.y, dz = gpos.z - s_we_prev_hand.z;
        hand_v = std::sqrt(dx * dx + dy * dy + dz * dz) * 100.0f / dt;
    }
    s_we_prev_hand = gpos; s_we_prev_have = true;
    if (s_we_n < kWeCap) s_we[s_we_n++] = WpnErrSample{(float)t, dt, err_cm, err_deg, pub_age, con_age, cam_v, hand_v};
    if (s_we_n >= kWeCap) we_flush();
    // The 1 Hz summary, still vs moving at 25 cm/s of camera speed.
    const int b = (cam_v > 25.0f) ? 1 : 0;
    s_we_sn[b]++; s_we_se[b] += err_cm; s_we_sr[b] += err_deg;
    if (err_cm > s_we_me[b]) s_we_me[b] = err_cm;
    if (err_deg > s_we_mr[b]) s_we_mr[b] = err_deg;
    s_we_sage_p += pub_age; s_we_sage_c += con_age;
    if (s_we_sum_t == 0.0) s_we_sum_t = t;
    if (t - s_we_sum_t >= 1.0) {
        const int n0 = s_we_sn[0], n1 = s_we_sn[1], nt = n0 + n1;
        if (nt > 0)
            API::get()->log_info("[Halo-CampE-UEVR] WPNERR still n=%d %0.2fcm/%0.2fdeg max %0.2f/%0.2f | moving n=%d %0.2fcm/%0.2fdeg max %0.2f/%0.2f | pubage %0.1fms conage %0.1fms",
                                 n0, n0 ? s_we_se[0] / n0 : 0.0, n0 ? s_we_sr[0] / n0 : 0.0, s_we_me[0], s_we_mr[0],
                                 n1, n1 ? s_we_se[1] / n1 : 0.0, n1 ? s_we_sr[1] / n1 : 0.0, s_we_me[1], s_we_mr[1],
                                 s_we_sage_p / nt, s_we_sage_c / nt);
        s_we_sn[0] = s_we_sn[1] = 0; s_we_se[0] = s_we_se[1] = 0.0; s_we_sr[0] = s_we_sr[1] = 0.0;
        s_we_me[0] = s_we_me[1] = 0.0f; s_we_mr[0] = s_we_mr[1] = 0.0f;
        s_we_sage_p = s_we_sage_c = 0.0;
        s_we_sum_t = t;
    }
}

int   blam_palette_probe_phase()  { return g_probe_phase.load(std::memory_order_relaxed); }
float blam_palette_probe_amount() { return kProbeAmt; }

void blam_palette_hook_tick() {
    // ---- IS THE OTHER .data CONSTANT STILL GOOD? One read, once, costs a line.
    //
    // RVA_TLS_INDEX (.data) survived the update untouched -- verified exactly against the PE TLS
    // directory. RVA_SHARED_CAPTURE_PTR lives in the same section, so it may well have survived too,
    // and if it does it points straight at the render capture banks: the FP palette, without finding
    // it by shape at all. Then only FP_BUILD is left to name, and a write-watch on a bank node names
    // it. If the pointer is junk this prints one line and we have lost nothing.
    {
        // RETRY UNTIL THE BANKS ARE POPULATED, then latch. One-shot was wrong for the same reason
        // the scan's one-shot was: it fired 7 s after launch, in the frontend, and read
        // "palette 0x0 count=0" from banks the game had not filled yet. The capture banks only hold
        // a palette while a first-person weapon is being posed.
        static bool s_checked = false;
        static uint32_t s_cap_gate = 0;
        if (!s_checked && (s_cap_gate++ % 8u) == 0u) {
            const HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
            if (sim != nullptr) {
                const uintptr_t slot = (uintptr_t)sim + RVA_SHARED_CAPTURE_PTR;
                if (readable((const void*)slot, 8)) {
                    const uintptr_t p = *reinterpret_cast<const uintptr_t*>(slot);
                    const bool ok = (p >= 0x10000) && readable((const void*)p, 0x100);
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] CAPTUREPTR: dll+0x%llX -> 0x%llX  readable=%d "
                        "(same .data section as the TLS index, which survived)",
                        (unsigned long long)RVA_SHARED_CAPTURE_PTR, (unsigned long long)p, (int)ok);
                    if (ok) {
                        for (int bank = 0; bank < 2; ++bank) {
                            const uintptr_t ctx = p + (uintptr_t)bank * CAPTURE_CTX_STRIDE;
                            if (!readable((const void*)(ctx + OFF_CAPTURE_PALETTE), 8)) continue;
                            const int32_t cnt = readable((const void*)(ctx + OFF_CAPTURE_COUNT), 4)
                                ? *reinterpret_cast<const int32_t*>(ctx + OFF_CAPTURE_COUNT) : -1;

                            // TWO READINGS OF THE SAME FIELD, because the 2026-08-17 log settled it:
                            //   bank 0 -> palette 0x3F8000003F800000 count=76
                            // 0x3F800000 is 1.0f, twice -- that is not an address, it is the first
                            // two floats of a node (scale, forward.x). The count is correct at its
                            // old offset and the ARRAY IS INLINE right behind it, not behind a
                            // pointer. Try the dereference too, in case some bank does hold one.
                            const uintptr_t deref = *reinterpret_cast<const uintptr_t*>(ctx + OFF_CAPTURE_PALETTE);
                            const uintptr_t inln  = ctx + OFF_CAPTURE_PALETTE;
                            const int32_t run_deref = (deref >= 0x10000 && readable((const void*)deref, sizeof(PaletteNode)))
                                ? run_length(deref, 512) : -1;
                            const int32_t run_inln = readable((const void*)inln, sizeof(PaletteNode))
                                ? run_length(inln, 512) : -1;
                            const bool inline_wins = (run_inln >= 24) && (run_inln >= run_deref);
                            const uintptr_t pal = inline_wins ? inln : deref;
                            const int32_t   run = inline_wins ? run_inln : run_deref;
                            API::get()->log_info(
                                "[Halo-CampE-UEVR] CAPTUREPTR:   bank %d count=%d | inline 0x%llX run=%d "
                                "| deref 0x%llX run=%d  %s",
                                bank, cnt, (unsigned long long)inln, run_inln,
                                (unsigned long long)deref, run_deref,
                                (run >= 24) ? (inline_wins ? "<== VALID, FP palette is INLINE"
                                                           : "<== VALID via pointer")
                                            : "(neither is a skeleton)");
                            // COUNT SAYS 76, RUN SAYS 1: node 0 is valid and node 1 is not where we
                            // think. Two unknowns -- where the array starts relative to the count
                            // field, and its stride -- so search both rather than guess again.
                            // Bounded and one-shot per bank; the answer is two numbers.
                            if (cnt >= 24 && run < 24) {
                                static int s_grid = 0;
                                if (s_grid < 2) {
                                    ++s_grid;
                                    uintptr_t bs = 0, bt = 0; int32_t br = 0;
                                    for (uintptr_t st = 0; st <= 128; st += 4) {
                                        const uintptr_t a0 = ctx + OFF_CAPTURE_PALETTE + st;
                                        if (!readable((const void*)a0, sizeof(PaletteNode))) continue;
                                        if (!looks_like_node(*reinterpret_cast<const PaletteNode*>(a0))) continue;
                                        for (uintptr_t sd = 48; sd <= 160; sd += 4) {
                                            const int32_t r = run_length_stride(a0, 512, sd);
                                            if (r > br) { br = r; bs = st; bt = sd; }
                                        }
                                    }
                                    API::get()->log_info(
                                        "[Halo-CampE-UEVR] CAPTUREGRID: bank %d best start=+0x%llX stride=%llu "
                                        "run=%d (count says %d). Array begins at ctx+0x%llX.",
                                        bank, (unsigned long long)bs, (unsigned long long)bt, br, cnt,
                                        (unsigned long long)(OFF_CAPTURE_PALETTE + bs));
                                }
                            }
                            // Latch only on a real answer, so an empty frontend read does not end it.
                            if (run >= 24) {
                                s_checked = true;
                                g_palette.store(pal, std::memory_order_relaxed);
                                g_nodes.store(run, std::memory_order_relaxed);
                                g_cand[0] = Candidate{pal, run};
                                g_cand_count.store(1, std::memory_order_release);
                            }
                        }
                    }
                } else {
                    API::get()->log_info("[Halo-CampE-UEVR] CAPTUREPTR: dll+0x%llX unreadable",
                                         (unsigned long long)RVA_SHARED_CAPTURE_PTR);
                }
            }
        }
    }

    // ---- DISCOVERY RUNS HERE NOW, ON THE GAME THREAD, BEFORE ANY HOOK GATE.
    //
    // These two used to be called only from inside hooked_get_orientation() -- i.e. only when a hook
    // at a hardcoded RVA was installed and working. The 2026-08-17 update moved that function, and
    // the tools for finding its new address were therefore unreachable. Now that the scan resolves
    // the sim TLS block cross-thread (sim_tls_array) and arm_all() already arms every thread, both
    // work from here with no hook at all, which is what a discovery tool has to be able to do.
    blam_palette_scan();
    blam_palette_watch();
    blam_slide_watch(false);
    blam_palette_final_hook_tick();
    blam_palette_sniff_tick();
    blam_palette_term_tick();

    // (The legacy palette-space calibration used to be adopted from disk here. It is retired; the
    // rigid grip offset is adopted in the publisher. palettewpnfix in an old file is ignored.)
    // The builder pose hook follows the arm driver arbiter (armdriver mode 3), not this poll: see
    // blam_palette_pose_hook_sync below, which the arbiter also calls every tick.
    blam_palette_pose_hook_sync();
}

// ---------------------------------------------------------------- the consumption hook
// (PALETTEFINAL, doctrine at the Config key). The renderer's per-node slerp: register args
// only (rcx = node A, rdx = node B, r8 = &t, r9 = destination node), verified by disassembly;
// the destination gate below touches ONLY the local player's FP palette in the render arenas.
namespace {
constexpr uintptr_t RVA_NODE_SLERP = 0x23BF40;
// palettefinal=2 (2026-09-12, after the record-ownership freeze proved the post-blend record
// is UPLOAD-GATED -- fresh moving bytes written 40/s never reached the screen): the blend at
// 0x23DAC0 is the game's own final writer, so its INPUTS become ours instead. Pre-original,
// golden lands in both sim banks; the blend then interpolates our data with its own code, its
// dirty tracking sees its own writes, the upload runs normally, and the phase-locked 17 us
// stock window dies because at the read moment the sources are always ours.
constexpr uintptr_t RVA_BANK_BLEND = 0x23DAC0;
constexpr uint8_t BANK_BLEND_PROLOGUE[12] = {0x48, 0x89, 0x5C, 0x24, 0x10, 0x48, 0x89, 0x6C, 0x24, 0x18, 0x4C, 0x89};
using BankBlendFn = uintptr_t (*)(uintptr_t, uintptr_t, uintptr_t, uintptr_t,
                                  uintptr_t, uintptr_t, uintptr_t, uintptr_t);
BankBlendFn g_blend_original = nullptr;
int g_blend_hook_id = -1;
uintptr_t hooked_bank_blend(uintptr_t a1, uintptr_t a2, uintptr_t a3, uintptr_t a4,
                            uintptr_t a5, uintptr_t a6, uintptr_t a7, uintptr_t a8) {
                                CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    if (g_cfg.palette_final == 2 && palette_weapon_mode()) {
        const long long ga = g_golden.at_ms.load(std::memory_order_relaxed);
        const long long gn = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (ga != 0 && gn - ga < 100) {
            const uint32_t s0 = g_rr.seq.load(std::memory_order_acquire);
            if (!(s0 & 1u)) {
                const int nb = g_rr.nbanks; const int32_t cnt = g_rr.count;
                const uint32_t gs = g_golden.seq.load(std::memory_order_acquire);
                if (!(gs & 1u) && nb > 0 && cnt > 0 && cnt <= FP_NODE_COUNT && g_golden.count == cnt) {
                    for (int i = 0; i < nb && i < 2; ++i) {
                        PaletteNode* b = g_rr.bank[i];
                        if (b != nullptr && !IsBadWritePtr(b, sizeof(PaletteNode) * (size_t)cnt))
                            memcpy(b, g_golden.nodes, sizeof(PaletteNode) * (size_t)cnt);
                    }
                }
            }
        }
    }
    return g_blend_original(a1, a2, a3, a4, a5, a6, a7, a8);
}
constexpr uint8_t NODE_SLERP_PROLOGUE[12] = {0x4C, 0x8B, 0xDC, 0x48, 0x81, 0xEC, 0xD8, 0x00, 0x00, 0x00, 0xC5, 0xFA};
constexpr uintptr_t RVA_RENDER_ARENAS = 0x1831228;   // two pointers, +0 and +8
using NodeSlerpFn = void (*)(void*, void*, void*, void*);
NodeSlerpFn g_slerp_original = nullptr;
int g_slerp_hook_id = -1;
void hooked_node_slerp(void* a, void* b, void* t, void* dst) {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    g_slerp_original(a, b, t, dst);
    // MINIMAL unless gameplay is live: the level-load crashes ride this function's load-time
    // call storms, so outside live gameplay the hook is a tail-call and nothing more.
    if (g_cfg.palette_final != 1 || !palette_weapon_mode()) return;
    {
        const long long fp = g_fp_built_ms.load(std::memory_order_relaxed);
        const long long nowms = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (fp == 0 || nowms - fp > 300) return;
    }
    static HMODULE s_sim = nullptr;
    if (s_sim == nullptr) s_sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
    if (s_sim == nullptr) return;
    // Phase A -- LOCATE by content: the record whose freshly blended nodes match the golden
    // copy is the local FP palette. Phase B -- OWN: once a base has accumulated enough hits,
    // every node the blend writes inside it is replaced from golden at the same index.
    static std::atomic<uint32_t> s_calls{0}, s_owned{0};
    static long long s_said = 0;
    static std::atomic<intptr_t> s_lock_base{-1};
    static std::atomic<int> s_lock_arena{-1};
    struct Cand { ptrdiff_t base; uint32_t hits; };
    static Cand s_cand[8] = {}; static int s_ncand = 0;
    s_calls.fetch_add(1, std::memory_order_relaxed);
    for (int arena = 0; arena < 2; ++arena) {
        auto* sh = *reinterpret_cast<uint8_t* const*>((uintptr_t)s_sim + RVA_RENDER_ARENAS + (uintptr_t)arena * 8);
        if (sh == nullptr) continue;
        const ptrdiff_t rel = (uint8_t*)dst - sh;
        if (rel < 0 || rel >= 0x200000) continue;
        const uint32_t s0 = g_golden.seq.load(std::memory_order_acquire);
        if (s0 & 1u) break;
        const int gc = g_golden.count;
        if (gc <= 8) break;
        const intptr_t locked = s_lock_base.load(std::memory_order_relaxed);
        if (locked >= 0 && s_lock_arena.load(std::memory_order_relaxed) == arena) {
            // NEVER own with a stale copy (the install-freeze of 22:01): a stalled golden feed
            // must degrade to the game's own output, not to a frozen rig.
            {
                const long long ga = g_golden.at_ms.load(std::memory_order_relaxed);
                const long long gn = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                if (ga == 0 || gn - ga > 100) break;
            }
            // FULL-RECORD REFRESH (the player's freeze observation, 22:1x): the blend only
            // rewrites nodes it considers CHANGED (~2/frame, the owned counter showed it), so
            // a per-node reply leaves the rest of the record at whatever it held -- the
            // install-time copy, frozen forever. Any write near our record now triggers a
            // rate-limited refresh of the WHOLE record from fresh golden.
            {
                static std::atomic<long long> s_last_full{0};
                const long long gn2 = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                long long lf = s_last_full.load(std::memory_order_relaxed);
                if (gn2 - lf >= 4 && s_last_full.compare_exchange_strong(lf, gn2)) {
                    static PaletteNode s_full[FP_NODE_COUNT];
                    memcpy(s_full, g_golden.nodes, sizeof(PaletteNode) * (size_t)gc);
                    if (g_golden.seq.load(std::memory_order_acquire) == s0) {
                        memcpy(sh + (size_t)locked, s_full, sizeof(PaletteNode) * (size_t)gc);
                        s_owned.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            }
            break;
        }
        const auto* out = reinterpret_cast<const PaletteNode*>(dst);
        int best = -1; float bd = 0.0004f;
        for (int k = 0; k < gc; ++k) {
            const float dx = out->position.x - g_golden.nodes[k].position.x;
            const float dy = out->position.y - g_golden.nodes[k].position.y;
            const float dz = out->position.z - g_golden.nodes[k].position.z;
            const float d = dx * dx + dy * dy + dz * dz;
            if (d < bd) { bd = d; best = k; }
        }
        if (best < 0) break;
        const ptrdiff_t base = rel - (ptrdiff_t)best * (ptrdiff_t)sizeof(PaletteNode);
        bool have = false;
        for (int c = 0; c < s_ncand; ++c) if (s_cand[c].base == base) {
            if (++s_cand[c].hits >= 1000 && s_lock_base.load(std::memory_order_relaxed) < 0) {
                s_lock_base.store(base, std::memory_order_relaxed);
                s_lock_arena.store(arena, std::memory_order_relaxed);
                API::get()->log_info("[Halo-CampE-UEVR] PALETTEFINAL: LOCKED arena %d base %lld",
                                     arena, (long long)base);
            }
            have = true; break;
        }
        if (!have && s_ncand < 8) { s_cand[s_ncand].base = base; s_cand[s_ncand].hits = 1; ++s_ncand; }
        break;
    }
    const long long noww = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    if (noww - s_said > 2000) {
        s_said = noww;
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEFINAL calls=%u owned=%u lock=%lld golden-age=%lldms g8=(%.4f %.4f %.4f) cand0=%lld(%u)",
            (unsigned)s_calls.load(), (unsigned)s_owned.load(),
            (long long)s_lock_base.load(),
            (long long)(noww - g_golden.at_ms.load(std::memory_order_relaxed)),
            g_golden.nodes[8].position.x, g_golden.nodes[8].position.y, g_golden.nodes[8].position.z,
            (long long)s_cand[0].base, s_cand[0].hits);
    }
}
} // namespace
// ---------------------------------------------------------------- the palette sniffer
// (PALSNIFF, doctrine at the Config key). All state file-scope; the thread is created on
// first sight of the key and parks itself when the key drops.
namespace {
std::atomic<int> g_sniff_run{0};
std::atomic<bool> g_sniff_started{false};
std::atomic<uint32_t> g_sniff_gen{0};   // a thread exits once its generation is retired (release)
struct SniffEv { double t_ms; int to_stock; float dist; };
constexpr int kSniffCap = 8192;
SniffEv g_sniff_ev[kSniffCap];
std::atomic<int> g_sniff_n{0};
std::atomic<uint32_t> g_sniff_samples{0};
int g_sniff_seq = 0;
bool sniff_read_node(const float** out, float* x, float* y, float* z) {
    const uintptr_t a = g_sniff_lp.load(std::memory_order_acquire);
    if (a == 0) return false;
    __try {
        const float* f = reinterpret_cast<const float*>(a);
        *x = f[0]; *y = f[1]; *z = f[2];
        *out = f;
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        g_sniff_lp.store(0, std::memory_order_release);
        return false;
    }
}
DWORD WINAPI sniff_thread(LPVOID arg) {
    const uint32_t gen = (uint32_t)(uintptr_t)arg;
    int last_state = -1;   // 0 ours, 1 stock
    while (true) {
        if (g_sniff_gen.load(std::memory_order_relaxed) != gen) return 0;
        if (g_sniff_run.load(std::memory_order_relaxed) == 0) { Sleep(50); last_state = -1; continue; }
        const float wx = g_dbg_node8_x.load(std::memory_order_relaxed);
        const float wy = g_dbg_node8_y.load(std::memory_order_relaxed);
        const float wz = g_dbg_node8_z.load(std::memory_order_relaxed);
        const float* f = nullptr; float x = 0, y = 0, z = 0;
        if (!sniff_read_node(&f, &x, &y, &z)) { Sleep(5); continue; }
        g_sniff_samples.fetch_add(1, std::memory_order_relaxed);
        const float dx = x - wx, dy = y - wy, dz = z - wz;
        const float d = std::sqrt(dx * dx + dy * dy + dz * dz) * 304.8f;
        const int state = (d > 20.0f) ? 1 : 0;
        if (last_state >= 0 && state != last_state) {
            const int i = g_sniff_n.fetch_add(1, std::memory_order_relaxed);
            if (i < kSniffCap) {
                g_sniff_ev[i] = SniffEv{
                    std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                        std::chrono::steady_clock::now().time_since_epoch()).count(),
                    state, d};
            }
        }
        last_state = state;
        YieldProcessor();
    }
}
void sniff_flush() {
    const int n = g_sniff_n.exchange(0, std::memory_order_relaxed);
    if (n <= 0 || g_cfg_path[0] == '\0') return;
    char path[MAX_PATH]; strncpy_s(path, sizeof(path), g_cfg_path, _TRUNCATE);
    char* slash = strrchr(path, '\\');
    if (slash == nullptr) return;
    char leaf[64]; sprintf_s(leaf, "halo_vr_sniff_%03d.csv", g_sniff_seq++);
    slash[1] = '\0'; strncat_s(path, sizeof(path), leaf, _TRUNCATE);
    FILE* fp = nullptr;
    if (fopen_s(&fp, path, "wb") == 0 && fp != nullptr) {
        fprintf(fp, "t_ms,to_stock,dist_cm\r\n");
        const int m = n < kSniffCap ? n : kSniffCap;
        for (int i = 0; i < m; ++i)
            fprintf(fp, "%.4f,%d,%.2f\r\n", g_sniff_ev[i].t_ms, g_sniff_ev[i].to_stock, g_sniff_ev[i].dist);
        fclose(fp);
        API::get()->log_info("[Halo-CampE-UEVR] PALSNIFF: %d transitions -> %hs (samples %u)",
                             m, leaf, (unsigned)g_sniff_samples.exchange(0));
    }
}
} // namespace
void blam_palette_sniff_tick() {
    const bool want = g_cfg.pal_sniff != 0;
    const bool was = g_sniff_run.load(std::memory_order_relaxed) != 0;
    if (want && !g_sniff_started.load(std::memory_order_relaxed)) {
        g_sniff_started.store(true, std::memory_order_relaxed);
        HANDLE h = CreateThread(nullptr, 0, &sniff_thread,
                                (LPVOID)(uintptr_t)g_sniff_gen.load(std::memory_order_relaxed), 0, nullptr);
        if (h != nullptr) CloseHandle(h);
    }
    if (want && !was) g_sniff_run.store(1, std::memory_order_relaxed);
    if (!want && was) { g_sniff_run.store(0, std::memory_order_relaxed); sniff_flush(); }
}
// ---------------------------------------------------------------- the FP animation kill
// (FPANIMKILL, doctrine at the Config key). sim+0x25E0C0: a true pdata function head, homing
// stores prove (int32 ecx, int32 edx, bool r8b, void* r9, int32 stack5); forwarded as eight
// integer args so any deeper stack slots ride along verbatim. Suppression returns 0 without
// running -- in solo play the FP palettes are the local player's alone.
namespace {
// RETARGETED (2026-09-11 21:2x): 0x25E0C0 suppressed cleanly but the stock intrusion rate
// did not move (28.6% with the kill on) and reload anims still rendered -- wrong writer.
// 0x46A2E0 is the next candidate: a true head (four homed register args) whose body loops
// the weapon slots (0x2908 stride), gates on the slot's flags and reads its animation field
// before taking the palette at +0x1094 -- the per-slot poser.
constexpr uintptr_t RVA_FP_ANIM = 0x46A2E0;
constexpr uint8_t FP_ANIM_PROLOGUE[12] = {0x4C, 0x89, 0x4C, 0x24, 0x20, 0x44, 0x89, 0x44, 0x24, 0x18, 0x89, 0x54};
using FpAnimFn = uintptr_t (*)(uintptr_t, uintptr_t, uintptr_t, uintptr_t,
                               uintptr_t, uintptr_t, uintptr_t, uintptr_t);
FpAnimFn g_fpanim_original = nullptr;
int g_fpanim_hook_id = -1;
uintptr_t hooked_fp_anim(uintptr_t a1, uintptr_t a2, uintptr_t a3, uintptr_t a4,
                         uintptr_t a5, uintptr_t a6, uintptr_t a7, uintptr_t a8) {
                             CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    // SUPPRESSION IS DEAD (T-pose AND frozen locomotion: this function is part of the unit's
    // whole update, not just the FP pose). Mode 1 is SURGICAL instead: the poser runs in
    // full -- movement, state, everything -- and the moment it returns, our golden bytes land
    // back on the live palette, at the second writer's own doorstep. The stock window at this
    // site closes from a tick to microseconds. Mode 2 keeps the broken suppression for
    // experiments only.
    if (g_cfg.fp_anim_kill == 2 && palette_weapon_mode() &&
        g_p_valid.load(std::memory_order_acquire)) {
        const long long fp = g_fp_built_ms.load(std::memory_order_relaxed);
        const long long nowk = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (fp != 0 && nowk - fp < 150) return 0;
    }
    // KEEP THE RIG ALIVE (the 24-second golden stalls, 22:13): with our bytes in the
    // consumed record, the game's change detector sees a static FP rig and stops posing it,
    // which stops the builder, which starves our own feed. The poser gates each slot on
    // flags & 3 == 3, so slot 0 is marked dirty before every pass -- the game can never
    // conclude the rig is static.
    if (g_cfg.fp_anim_kill == 1 && palette_weapon_mode()) {
        LiveSlot dv{};
        PaletteNode* dp = live_palette_for(0, 0, &dv);
        if (dp != nullptr) {
            auto* slot_base = reinterpret_cast<uint8_t*>(dp) - OFF_FINAL_PALETTE;
            if (!IsBadWritePtr(slot_base + OFF_SLOT_FLAGS, 4))
                *reinterpret_cast<uint32_t*>(slot_base + OFF_SLOT_FLAGS) |= 3u;
        }
    }
    const uintptr_t r = g_fpanim_original(a1, a2, a3, a4, a5, a6, a7, a8);
    if (g_cfg.fp_anim_kill == 1 && palette_weapon_mode() &&
        g_p_valid.load(std::memory_order_acquire)) {
        static std::atomic<uint32_t> s_calls{0}, s_landed{0}, s_null{0}, s_mismatch{0};
        static std::atomic<long long> s_said{0};
        s_calls.fetch_add(1, std::memory_order_relaxed);
        LiveSlot lv{};
        PaletteNode* lp = live_palette_for(0, 0, &lv);
        if (lp == nullptr) s_null.fetch_add(1, std::memory_order_relaxed);
        if (lp != nullptr && lv.count > 0 && lv.count <= FP_NODE_COUNT &&
            !IsBadWritePtr(lp, sizeof(PaletteNode) * (size_t)lv.count)) {
            for (int sl_try = 0; sl_try < 4; ++sl_try) {
                const uint32_t s0 = g_golden.seq.load(std::memory_order_acquire);
                if (s0 & 1u) continue;
                if (g_golden.count != lv.count || g_golden.tag != lv.model_tag) { s_mismatch.fetch_add(1, std::memory_order_relaxed); break; }
                memcpy(lp, g_golden.nodes, sizeof(PaletteNode) * (size_t)lv.count);
                if (g_golden.seq.load(std::memory_order_acquire) == s0) { s_landed.fetch_add(1, std::memory_order_relaxed); break; }
            }
        }
        const long long noww = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        long long prev = s_said.load(std::memory_order_relaxed);
        if (noww - prev > 2000 && s_said.compare_exchange_strong(prev, noww)) {
            API::get()->log_info("[Halo-CampE-UEVR] FPANIM-SURGICAL tid=%u calls=%u landed=%u null-lp=%u tag-mismatch=%u (builder tid=%u)",
                (unsigned)GetCurrentThreadId(), (unsigned)s_calls.load(), (unsigned)s_landed.load(),
                (unsigned)s_null.load(), (unsigned)s_mismatch.load(),
                (unsigned)g_game_tid.load(std::memory_order_relaxed));
        }
    }
    return r;
}
} // namespace
void blam_palette_final_hook_tick() {
    if (g_cfg.fp_anim_kill != 0 && palette_weapon_mode() && g_fpanim_hook_id < 0) {
        HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
        if (sim != nullptr) {
            void* target = (void*)((uintptr_t)sim + RVA_FP_ANIM);
            bool ok = readable(target, sizeof(FP_ANIM_PROLOGUE));
            if (ok) {
                const uint8_t* got = (const uint8_t*)target;
                for (size_t i = 0; i < sizeof(FP_ANIM_PROLOGUE); ++i)
                    if (got[i] != FP_ANIM_PROLOGUE[i]) { ok = false; break; }
            }
            if (!ok) {
                API::get()->log_info("[Halo-CampE-UEVR] FPANIMKILL: prologue mismatch at dll+0x%llX -- not hooking",
                                     (unsigned long long)RVA_FP_ANIM);
                g_cfg.fp_anim_kill = 0;
            } else {
                const int id = API::get()->param()->functions->register_inline_hook(
                    target, (void*)&hooked_fp_anim, (void**)&g_fpanim_original);
                if (id < 0 || g_fpanim_original == nullptr) {
                    API::get()->log_info("[Halo-CampE-UEVR] FPANIMKILL: hook FAILED (id=%d)", id);
                    g_cfg.fp_anim_kill = 0;
                } else {
                    g_fpanim_hook_id = id;
                    API::get()->log_info("[Halo-CampE-UEVR] FPANIMKILL: installed on dll+0x%llX id=%d",
                                         (unsigned long long)RVA_FP_ANIM, id);
                }
            }
        }
    }
    const bool want = g_cfg.palette_final != 0 && palette_weapon_mode();
    if (!want || g_slerp_hook_id >= 0) return;
    // DEFERRED INSTALL (the load-crash lesson): the slerp hook goes in only after gameplay
    // has been continuously live -- builds fresh across ~200 consecutive ticks -- so it does
    // not exist while the first level streams. Outside live gameplay the body tail-calls.
    {
        // Wall-time, not call-count: this function's cadence is not the tick's. Install after
        // builds have been continuously fresh for 8 seconds.
        static long long s_fresh_since = 0;
        const long long fp = g_fp_built_ms.load(std::memory_order_relaxed);
        const long long nowms = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (fp == 0 || nowms - fp > 500) { s_fresh_since = 0; return; }
        if (s_fresh_since == 0) { s_fresh_since = nowms; return; }
        if (nowms - s_fresh_since < 8000) return;
    }
    HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
    if (sim == nullptr) return;
    void* target = (void*)((uintptr_t)sim + RVA_NODE_SLERP);
    if (!readable(target, sizeof(NODE_SLERP_PROLOGUE))) return;
    const uint8_t* got = (const uint8_t*)target;
    for (size_t i = 0; i < sizeof(NODE_SLERP_PROLOGUE); ++i) {
        if (got[i] != NODE_SLERP_PROLOGUE[i]) {
            API::get()->log_info("[Halo-CampE-UEVR] PALETTEFINAL: PROLOGUE MISMATCH at dll+0x%llX byte %zu -- not hooking",
                                 (unsigned long long)RVA_NODE_SLERP, i);
            g_cfg.palette_final = 0;
            return;
        }
    }
    const int id = API::get()->param()->functions->register_inline_hook(
        target, (void*)&hooked_node_slerp, (void**)&g_slerp_original);
    if (id < 0 || g_slerp_original == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEFINAL: register_inline_hook FAILED (id=%d)", id);
        g_cfg.palette_final = 0;
        return;
    }
    g_slerp_hook_id = id;
    API::get()->log_info("[Halo-CampE-UEVR] PALETTEFINAL: installed mid-gameplay on dll+0x%llX id=%d",
                         (unsigned long long)RVA_NODE_SLERP, id);
    {   // the blend hook rides the same deferred moment; body gates on palettefinal=2
        void* btarget = (void*)((uintptr_t)sim + RVA_BANK_BLEND);
        bool bok = readable(btarget, sizeof(BANK_BLEND_PROLOGUE));
        if (bok) {
            const uint8_t* bgot = (const uint8_t*)btarget;
            for (size_t i = 0; i < sizeof(BANK_BLEND_PROLOGUE); ++i)
                if (bgot[i] != BANK_BLEND_PROLOGUE[i]) { bok = false; break; }
        }
        if (bok) {
            const int bid = API::get()->param()->functions->register_inline_hook(
                btarget, (void*)&hooked_bank_blend, (void**)&g_blend_original);
            if (bid >= 0 && g_blend_original != nullptr) {
                g_blend_hook_id = bid;
                API::get()->log_info("[Halo-CampE-UEVR] PALETTEFINAL: blend hook installed on dll+0x%llX id=%d",
                                     (unsigned long long)RVA_BANK_BLEND, bid);
            } else {
                API::get()->log_info("[Halo-CampE-UEVR] PALETTEFINAL: blend hook FAILED (id=%d)", bid);
            }
        } else {
            API::get()->log_info("[Halo-CampE-UEVR] PALETTEFINAL: blend prologue mismatch -- input landing unavailable");
        }
    }
}

// AIMBORE accessor (see BlamPalette.hpp). The last published values persist across a dropped
// publish, so the aim does not jump by the trim when a single tick fails to resolve the hands.
bool palette_trim_rotations(float grip_q[4], float weapon_q[4]) {
    if (!palette_weapon_mode()) return false;
    grip_q[0] = g_p_rotg_x.load(std::memory_order_relaxed); grip_q[1] = g_p_rotg_y.load(std::memory_order_relaxed);
    grip_q[2] = g_p_rotg_z.load(std::memory_order_relaxed); grip_q[3] = g_p_rotg_w.load(std::memory_order_relaxed);
    weapon_q[0] = g_p_wrot_x.load(std::memory_order_relaxed); weapon_q[1] = g_p_wrot_y.load(std::memory_order_relaxed);
    weapon_q[2] = g_p_wrot_z.load(std::memory_order_relaxed); weapon_q[3] = g_p_wrot_w.load(std::memory_order_relaxed);
    return true;
}


// ================================================================================================
// OWNERSHIP OF THE BUILDER HOOK (armdriver mode 3). The fork's pose hook and the palettearm route's
// hook detour the same function, so exactly one may be installed, and only by the mode that owns.
// ================================================================================================
namespace {
bool s_pose_unavailable = false;   // latched refusal: the arbiter falls back to UeRig
}

bool blam_palette_unavailable() { return s_pose_unavailable; }
void blam_palette_retry() { s_pose_unavailable = false; }

int blam_palette_pose_hook_sync() {
    // palettehook picks the pass; under mode 3 an unset key still means pass 1, because the
    // placement cannot run at all without the hook. Any other mode wants none.
    const int want = palette_weapon_mode() ? (g_cfg.palette_hook > 0 ? g_cfg.palette_hook : 1) : 0;

    if (want != g_pose_hooked_which && g_pose_hook_id >= 0) {
        API::get()->param()->functions->unregister_inline_hook(g_pose_hook_id);
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEHOOK: removed id=%d (%s)", g_pose_hook_id,
                             want == 0 ? "armdriver mode 3 no longer owns" : "pass changed");
        g_pose_hook_id = -1;
        g_pose_original = nullptr;
        g_pose_hooked_which = 0;
    }
    if (want == 0) return -1;
    if (g_pose_hook_id >= 0) return g_pose_hook_id;
    if (s_pose_unavailable) return -1;

    // NEVER TWO DETOURS ON ONE FUNCTION. The arbiter releases the other route before this mode takes
    // over, so this only trips if something is out of order; refuse and say so rather than stack.
    if (halo::palettearm::palettehook_installed()) {
        static bool s_said = false;
        if (!s_said) {
            s_said = true;
            API::get()->log_info("[Halo-CampE-UEVR] PALETTEHOOK: refusing -- the palettearm route still "
                                 "holds the builder hook");
        }
        return -1;
    }

    HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
    if (sim == nullptr) return -1;   // still loading; the sim DLL arrives late

    const uintptr_t rva = RVA_FP_BUILD;
    void* target = (void*)((uintptr_t)sim + rva);

    // VERIFY BEFORE PATCHING. Fail closed, say which byte disagreed, and let the arbiter fall back.
    if (!readable(target, sizeof(FP_BUILD_PROLOGUE))) {
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEHOOK: dll+0x%llX unreadable -- not hooking; "
                             "armdriver falls back to UeRig", (unsigned long long)rva);
        s_pose_unavailable = true;
        return -1;
    }
    {
        const uint8_t* got = (const uint8_t*)target;
        for (size_t i = 0; i < sizeof(FP_BUILD_PROLOGUE); ++i) {
            if (got[i] != FP_BUILD_PROLOGUE[i]) {
                API::get()->log_info(
                    "[Halo-CampE-UEVR] PALETTEHOOK: PROLOGUE MISMATCH at dll+0x%llX byte %zu "
                    "(expected 0x%02X, found 0x%02X) -- the game build moved this function. "
                    "NOT hooking; armdriver falls back to UeRig.",
                    (unsigned long long)rva, i, FP_BUILD_PROLOGUE[i], got[i]);
                s_pose_unavailable = true;
                return -1;
            }
        }
    }

    const int id = API::get()->param()->functions->register_inline_hook(
        target, (void*)&hooked_pose, (void**)&g_pose_original);
    if (id < 0 || g_pose_original == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEHOOK: register_inline_hook FAILED (id=%d) "
                             "on dll+0x%llX -- armdriver falls back to UeRig", id, (unsigned long long)rva);
        g_pose_original = nullptr;
        s_pose_unavailable = true;
        return -1;
    }
    g_pose_hook_id = id;
    g_pose_hooked_which = want;
    API::get()->log_info("[Halo-CampE-UEVR] PALETTEHOOK: installed on dll+0x%llX (pass %d) id=%d",
                         (unsigned long long)rva, want, id);
    return id;
}

void blam_palette_release(const char* why, char* out, size_t cap) {
    char buf[192] = "";
    int n = 0;
    auto drop = [&](int& hook_id, const char* name) {
        if (hook_id < 0) return;
        API::get()->param()->functions->unregister_inline_hook(hook_id);
        if (n < (int)sizeof(buf)) n += std::snprintf(buf + n, sizeof(buf) - (size_t)n, "%s%s id=%d",
                                                     n ? ", " : "", name, hook_id);
        hook_id = -1;
    };
    drop(g_pose_hook_id, "pose");
    g_pose_original = nullptr;
    g_pose_hooked_which = 0;
    drop(g_slerp_hook_id, "final slerp");
    g_slerp_original = nullptr;
    drop(g_blend_hook_id, "bank blend");
    g_blend_original = nullptr;
    drop(g_fpanim_hook_id, "fp anim");
    g_fpanim_original = nullptr;
    // Nothing this stack published may outlive it: the pose, the hold, the freeze.
    g_p_valid.store(false, std::memory_order_release);
    blam_palette_hold_pose(0);
    g_freeze_key_held.store(false, std::memory_order_relaxed);
    if (n > 0) {
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEWPN: released (%s): %s", why ? why : "?", buf);
    }
    if (out != nullptr && cap > 0) std::snprintf(out, cap, "palettewpn hooks %s", n ? buf : "none");
}

// PALETTEWPN SWITCHED OFF. The dev discovery instruments install things that outlive the ticks that
// manage them (those ticks only run while the feature is on): hardware watchpoints on every thread and
// the exception handler behind them (palettewatch, paletteslidewatch), inline hooks on the renderer and the FP
// poser (palettefinal, fpanimkill), a sampler thread (palsniff) and an unwritten row buffer (termlog).
// All of it goes here. palettescan installs nothing (a sweep per call) and needs no release. Game thread.
void blam_palette_instruments_release() {
    blam_slide_watch(true);
    if (g_watching.load(std::memory_order_relaxed)) {
        arm_all(0);
        g_watching.store(false, std::memory_order_relaxed);
        report_writers();
        for (int i = 0; i < kMaxWriters; ++i) {
            g_writers[i].rip.store(0, std::memory_order_relaxed);
            g_writers[i].n.store(0, std::memory_order_relaxed);
        }
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEWATCH: disarmed (palettewpn switched off)");
    }
    if (g_veh != nullptr) { RemoveVectoredExceptionHandler(g_veh); g_veh = nullptr; }
    blam_palette_release("palettewpn switched off", nullptr, 0);
    if (g_sniff_run.exchange(0, std::memory_order_relaxed) != 0) sniff_flush();
    if (g_sniff_started.exchange(false, std::memory_order_relaxed)) g_sniff_gen.fetch_add(1, std::memory_order_relaxed);
    if (termlog::g_was_on) {
        termlog::g_was_on = false;
        termlog::flush();
        branchlog::flush();
    }
}

} // namespace halo
