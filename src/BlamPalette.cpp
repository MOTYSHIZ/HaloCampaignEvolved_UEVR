#include "BlamPalette.hpp"

#include "Config.hpp"
#include "TwoHand.hpp"       // two_hand_delta: the hold rotates the WEAPON pose, not only the shot
#include "Math.hpp"
#include "MotionAimControl.hpp"
#include "Rig.hpp"   // g_turnq_* : the player's accumulated snap/smooth turn
#include "WeaponCalib.hpp"   // weapon_key / wpnfix: the per-weapon rigid delta
#include "uevr/API.hpp"

#include <Windows.h>
#include <psapi.h>
#include <tlhelp32.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
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

std::atomic<bool> g_watching{false};
uintptr_t         g_watch_addr = 0;
PVOID             g_veh = nullptr;
uintptr_t         g_self_lo = 0, g_self_hi = 0;

// Distinct writers seen, with hit counts. Several things touch a live object -- the pose build,
// memcpy, allocator fill -- so the useful output is a RANKED LIST, not the first hit. The one that
// runs every frame at the rig's cadence is the one worth hooking.
constexpr int kMaxWriters = 8;
struct WriterHit { std::atomic<uintptr_t> rip; std::atomic<uint32_t> n; };
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
void arm_all(uintptr_t addr) {
    const DWORD pid  = GetCurrentProcessId();
    const DWORD self = GetCurrentThreadId();

    auto set_on = [addr](HANDLE th, bool is_self) {
        if (!is_self && SuspendThread(th) == (DWORD)-1) return;
        CONTEXT c{};
        c.ContextFlags = CONTEXT_DEBUG_REGISTERS;
        if (GetThreadContext(th, &c)) {
            c.Dr0 = addr;
            c.Dr7 = addr ? dr7_write4() : 0;
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
int collect_capture_banks(int32_t local_player, int32_t weapon_slot,
                          const LiveSlot& live, PaletteNode* out[2]) {
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
std::atomic<bool>  g_p_valid{false};
// ---- THE RIGID GRIP OFFSET AND THE FREEZE. Game thread only (the publisher). See the pose-match
// block in blam_palette_publish_poses(). rot right-multiplies the pose; pos_m is metres in the
// CONTROLLER's frame, rotated by the corrected pose at apply time -- the rigid attachment.
Quat g_grip_fix_rot{0.0f, 0.0f, 0.0f, 1.0f};
Vec3 g_grip_fix_pos_m{0.0f, 0.0f, 0.0f};
bool g_grip_fix_valid = false;
// The calibration key, published by the game thread for the sim-side freeze in the pullback.
std::atomic<bool> g_freeze_key_held{false};
// A solved rigid delta handed BACK from the pullback (sim thread) to the publisher (game thread),
// which composes it onto the grip offset and persists it. One slot; the publisher clears it.
Quat g_pend_rot{0.0f, 0.0f, 0.0f, 1.0f};
Vec3 g_pend_pos_m{0.0f, 0.0f, 0.0f};
std::atomic<bool> g_pend_valid{false};

// The bone-frame result of the world-space pullback -- what the branch actually uses. Written by
// resolve_world_pullback() at the top of a build, read by apply_weapon_branch for the live
// palette and both banks, all inside one sim-thread call, so plain values are safe.
Vec3 g_eff_hand{0.0f, 0.0f, 0.0f};
Quat g_eff_pose{0.0f, 0.0f, 0.0f, 1.0f};

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

bool apply_weapon_branch(PaletteNode* palette, Mat3* out_delta_basis = nullptr, NodeMatrix* out_delta_pos = nullptr) {
    if (!g_p_valid.load(std::memory_order_acquire)) return false;

    const Mat3 root_basis   = basis_of(palette[0]);
    const Mat3 weapon_basis = basis_of(palette[8]);
    if (!basis_ok(root_basis) || !basis_ok(weapon_basis)) return false;

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
            const Mat3 stock = basis_of(palette[8]);
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

    const Mat3 delta_basis = mat_mul(desired_basis, mat_transpose(weapon_basis));
    if (!basis_ok(delta_basis)) return false;
    const NodeMatrix delta_pos = sub(desired_pos, xform(delta_basis, palette[8].position));
    if (out_delta_basis != nullptr) *out_delta_basis = delta_basis;
    if (out_delta_pos   != nullptr) *out_delta_pos   = delta_pos;

    for (const int node : {7, 8, 22}) {
        PaletteNode& m = palette[node];
        NodeMatrix f = xform(delta_basis, m.forward);
        NodeMatrix l = xform(delta_basis, m.left);
        NodeMatrix u = xform(delta_basis, m.up);
        if (!norm_node(&f) || !norm_node(&l) || !norm_node(&u)) return false;
        const NodeMatrix p = add(delta_pos, xform(delta_basis, m.position));
        // REFUSE ANYTHING ABSURD, for the same reason the poke does: a first-person node far
        // outside arm's reach is what the sim faulted on, and one bad frame is enough. The bound
        // is ARM'S REACH -- 0.6 palette units is ~1.8 m from the view root, which no held weapon
        // node ever legitimately exceeds -- not the old 8.0 (24 m), which only caught NaN-scale
        // garbage. The 2026-08-15 crash was a 2.4 m "hand" from a tracking dropout (controller
        // reads 0,0,0 asleep; the offset becomes the negated head position) sailing under 8.0,
        // being frozen by the calibration key, and faulting HaloCampaignEvolved.exe itself.
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
        if (std::fabs(p.x) > 0.6f || std::fabs(p.y) > 0.6f || std::fabs(p.z) > 0.6f) return false;
        m.forward = f; m.left = l; m.up = u; m.position = p;
        if (node == 8) {
            g_dbg_node8_x.store(p.x, std::memory_order_relaxed);
            g_dbg_node8_y.store(p.y, std::memory_order_relaxed);
            g_dbg_node8_z.store(p.z, std::memory_order_relaxed);
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
bool resolve_world_pullback() {
    if (!g_p_valid.load(std::memory_order_acquire)) return false;
    if (!g_mesh_const_valid.load(std::memory_order_acquire)) return false;

    double cam_pitch = 0.0, cam_yaw = 0.0;
    if (!read_control_rotation(&cam_pitch, &cam_yaw, nullptr)) return false;
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
    const Quat comp_rot = quat_mul(cam, M);
    const Quat comp_inv = quat_conj(comp_rot);

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
    const Vec3 hand_room{g_p_grip_x.load(std::memory_order_relaxed) * cm_per_m,
                         g_p_grip_y.load(std::memory_order_relaxed) * cm_per_m,
                         g_p_grip_z.load(std::memory_order_relaxed) * cm_per_m};
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
    const Quat cam_yaw_only = rotator_to_quat(0.0f, g_view_base_yaw.load(std::memory_order_relaxed), 0.0f);
    Vec3 hand_cm = quat_rotate(cam_yaw_only, hand_room);                // room hand -> world
    // CAMERA BOB CANCEL: the mesh rides the bobbing camera; the view is rendered from the camera
    // minus the bob (Plugin.cpp stereo callback). Take the same vector out of the hand so the gun
    // stays where the hand is in the stabilised view instead of bobbing against it.
    hand_cm.x -= g_bob_x.load(std::memory_order_relaxed);
    hand_cm.y -= g_bob_y.load(std::memory_order_relaxed);
    hand_cm.z -= g_bob_z.load(std::memory_order_relaxed);

    // Same lift for the ORIENTATION, or position and orientation diverge by the camera yaw --
    // "it moves but doesn't rotate right". The published pose is room-frame; bring it into the
    // world by the same rotation. Identical frames for both halves, always.
    Quat pose_world = quat_mul(cam_yaw_only, Quat{g_p_aim_x.load(std::memory_order_relaxed),
                                                   g_p_aim_y.load(std::memory_order_relaxed),
                                                   g_p_aim_z.load(std::memory_order_relaxed),
                                                   g_p_aim_w.load(std::memory_order_relaxed)});

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
    if (!(n > 1.0e-6f) || !std::isfinite(n)) return false;
    pose.x /= n; pose.y /= n; pose.z /= n; pose.w /= n;
    g_eff_pose = pose;

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

void apply_after_pose(int32_t local_player, int32_t weapon_slot) {
    if (g_cfg.palette_hook_test <= 0 && !g_cfg.palette_weapon) return;

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

    if (g_cfg.palette_weapon && !resolve_world_pullback()) return;
    LiveSlot live{};
    PaletteNode* palette = live_palette_for(local_player, weapon_slot, &live);
    if (palette == nullptr) return;

    PaletteNode* banks[2] = {nullptr, nullptr};
    const int nbanks = collect_capture_banks(local_player, weapon_slot, live, banks);

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

    // ---- THE WEAPON BRANCH.
    //
    // Applied to the live palette AND to every render bank. The live copy is what the sim reasons
    // about; the banks are what get drawn. Transforming one without the other is the split that
    // makes a gun render in one place and shoot from another.
    if (g_cfg.palette_weapon) {
        Mat3 dB{}; NodeMatrix dP{};
        bool ok = apply_weapon_branch(palette, &dB, &dP);
        for (int i = 0; i < nbanks; ++i) {
            ok = apply_weapon_branch(banks[i], &dB, &dP) && ok;
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
    auto* n = &palette[node];
    const uintptr_t p = (uintptr_t)n;

    // ABSOLUTE, NOT ACCUMULATED. This crashed the game once by adding on every pass.
    //
    // From the sim-thread hook the game reset the value between our writes, so `+=` stayed bounded
    // and looked safe. Writing here it is the opposite: we run LAST, so our value is what the next
    // frame builds from, and the delta compounds -- 10, 20, 30 units a frame, thousands of metres
    // within a second, until the sim computed a bounds index from it and faulted with RCX negative.
    //
    // So: latch what the game produced the first time we saw it, and write that baseline plus the
    // offset every time. Bounded by construction, no matter how many frames run.
    static uintptr_t s_base_at = 0;
    static float     s_base_y  = 0.0f;
    if (s_base_at != p) { s_base_at = p; s_base_y = n->position.y; }

    const float want = s_base_y + g_cfg.palette_poke_amt;

    // REFUSE ANYTHING ABSURD. A first-person node lives within arm's reach of the view root, so a
    // coordinate far outside that is not a displacement the sim was built to survive -- and it is
    // exactly what the crash was made of. Cheap, and the difference between a test and an outage.
    if (!std::isfinite(want) || std::fabs(want) > 8.0f) return;
    n->position.y = want;

    // AND THE RENDER BANKS, which is the write that actually shows. The live palette above is what
    // the sim reasons about; the capture is what gets drawn.
    for (int i = 0; i < nbanks; ++i) banks[i][node].position.y = want;

    static uint32_t s_said = 0;
    if (s_said < 5) {
        ++s_said;
        API::get()->log_info(
            "[Halo-CampE-UEVR] PALETTEHOOK: player=%d slot=%d node=%d y=%.3f  capture banks written=%d",
            local_player, weapon_slot, node, want, nbanks);
    }
}

// Last time the game built the LOCAL player's first-person palette, steady-clock ms. The build
// routine only runs while a first-person weapon is actually being rendered, which makes it a
// POSITIVE weapon-presence signal -- stronger than any rig or route resolve, and available even
// with the rig driver off (rig=0), which is the shipping configuration once the palette owns
// weapon placement. Consumed by the stick-mode detector.
std::atomic<int64_t> g_fp_built_ms{0};

void hooked_pose(int32_t local_player, int32_t weapon_slot, bool capture_render_palette) {
    if (g_pose_original != nullptr) {
        g_pose_original(local_player, weapon_slot, capture_render_palette);
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
    LARGE_INTEGER t0{}, t1{};
    QueryPerformanceCounter(&t0);
    apply_after_pose(local_player, weapon_slot);
    QueryPerformanceCounter(&t1);
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

void blam_palette_publish_poses() {
    // The calibration key, polled here because this already runs on the game thread every frame and
    // the hook must not be reading input.
    // Page Up = GLOBAL freeze-and-align (gripfix). Home (wpn_calib_key) = the SAME gesture, but the
    // solve lands in the held weapon's wpnfix delta instead. Which one started the hold is latched
    // on the rising edge, because by the time the solve arrives the key is up.
    const bool held_global = g_cfg.palette_calib_key != 0 &&
                             (GetAsyncKeyState(g_cfg.palette_calib_key) & 0x8000) != 0;
    const bool held_wpn = wpn_calib_held();
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
    if (!g_cfg.palette_weapon) { g_p_valid.store(false, std::memory_order_release); return; }

    // MIRROR the file, both directions. This used to adopt an offset from disk and never let it
    // go: once g_grip_fix_valid was true in memory, deleting gripfix= from the file changed
    // nothing until a restart -- so every "clear" today was clean on disk and dirty in RAM, and
    // the next capture composed onto the ghost. A cleared file must clear the offset; a written
    // file must be adopted verbatim (so a hand-edited or restored file takes effect too).
    if (g_cfg.grip_fix_valid) {
        // Pitch only (see the capture compose below): whatever yaw/roll an older file carries is
        // dropped at adoption, so a hand-edited or legacy gripfix cannot put yaw or roll into the gun.
        g_grip_fix_rot   = quat_twist(quat_unit(Quat{g_cfg.grip_fix[0], g_cfg.grip_fix[1], g_cfg.grip_fix[2], g_cfg.grip_fix[3]}),
                                      Vec3{0.0f, 1.0f, 0.0f});
        g_grip_fix_pos_m = Vec3{g_cfg.grip_fix[4], g_cfg.grip_fix[5], g_cfg.grip_fix[6]};
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
    Quat aq_fixed = apply_aim_fix(aq);
    {
        Quat thd{0.0f, 0.0f, 0.0f, 1.0f};
        if (two_hand_delta(&thd)) aq_fixed = quat_unit(quat_mul(thd, aq_fixed));
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
    const WeaponFix* wfix = wpnfix_find(wkey);
    {
        static std::string s_last_wkey;
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
            wpn_calib_take_pending();
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
                wpnfix_set(wkey, q, t);
                float gp = 0.0f, gy = 0.0f, gr = 0.0f;
                quat_to_rotator(wr.x, wr.y, wr.z, wr.w, &gp, &gy, &gr);
                API::get()->log_info(
                    "[Halo-CampE-UEVR] PALETTECAL(WPN) '%s': this match moved %.1f cm, rotated %.0f deg; "
                    "weapon delta rot=(p%.1f y%.1f r%.1f) pos=(%.1f %.1f %.1f)cm -> halo_vr_weapons.cfg",
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
            g_cfg.grip_fix[0] = g_grip_fix_rot.x; g_cfg.grip_fix[1] = g_grip_fix_rot.y;
            g_cfg.grip_fix[2] = g_grip_fix_rot.z; g_cfg.grip_fix[3] = g_grip_fix_rot.w;
            g_cfg.grip_fix[4] = g_grip_fix_pos_m.x; g_cfg.grip_fix[5] = g_grip_fix_pos_m.y;
            g_cfg.grip_fix[6] = g_grip_fix_pos_m.z;
            g_cfg.grip_fix_valid = true;
            write_calib_file();
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

    g_p_grip_x.store(d_xr.x, std::memory_order_relaxed);
    g_p_grip_y.store(d_xr.y, std::memory_order_relaxed);
    g_p_grip_z.store(d_xr.z, std::memory_order_relaxed);
    g_p_aim_x.store(pose_xr.x, std::memory_order_relaxed);
    g_p_aim_y.store(pose_xr.y, std::memory_order_relaxed);
    g_p_aim_z.store(pose_xr.z, std::memory_order_relaxed);
    g_p_aim_w.store(pose_xr.w, std::memory_order_release);
    g_p_valid.store(true, std::memory_order_release);
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

    // (The legacy palette-space calibration used to be adopted from disk here. It is retired; the
    // rigid grip offset is adopted in the publisher. palettewpnfix in an old file is ignored.)
    const int want = g_cfg.palette_hook;

    if (want != g_pose_hooked_which && g_pose_hook_id >= 0) {
        API::get()->param()->functions->unregister_inline_hook(g_pose_hook_id);
        g_pose_hook_id = -1;
        g_pose_original = nullptr;
        g_pose_hooked_which = 0;
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEHOOK: removed");
    }
    if (want == 0 || g_pose_hook_id >= 0) return;

    HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
    if (sim == nullptr) return;   // still loading; the sim DLL arrives late

    const uintptr_t rva = RVA_FP_BUILD;
    void* target = (void*)((uintptr_t)sim + rva);

    // VERIFY BEFORE PATCHING. Fail closed and say which byte disagreed.
    if (!readable(target, sizeof(FP_BUILD_PROLOGUE))) {
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEHOOK: dll+0x%llX unreadable -- not hooking",
                             (unsigned long long)rva);
        g_cfg.palette_hook = 0;
        return;
    }
    {
        const uint8_t* got = (const uint8_t*)target;
        for (size_t i = 0; i < sizeof(FP_BUILD_PROLOGUE); ++i) {
            if (got[i] != FP_BUILD_PROLOGUE[i]) {
                API::get()->log_info(
                    "[Halo-CampE-UEVR] PALETTEHOOK: PROLOGUE MISMATCH at dll+0x%llX byte %zu "
                    "(expected 0x%02X, found 0x%02X) -- the game build moved this function. "
                    "NOT hooking. Re-find it before re-enabling; hooking the wrong address crashes.",
                    (unsigned long long)rva, i, FP_BUILD_PROLOGUE[i], got[i]);
                g_cfg.palette_hook = 0;
                return;
            }
        }
    }

    const int id = API::get()->param()->functions->register_inline_hook(
        target, (void*)&hooked_pose, (void**)&g_pose_original);
    if (id < 0 || g_pose_original == nullptr) {
        // Fail LOUD and stay off, the same rule BlamDrive follows. A half-installed hook on a
        // frame-rate function is worse than no hook at all.
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEHOOK: register_inline_hook FAILED (id=%d) "
                             "on dll+0x%llX", id, (unsigned long long)rva);
        g_cfg.palette_hook = 0;
        return;
    }
    g_pose_hook_id = id;
    g_pose_hooked_which = want;
    API::get()->log_info("[Halo-CampE-UEVR] PALETTEHOOK: installed on dll+0x%llX (pass %d) id=%d",
                         (unsigned long long)rva, want, id);
}

} // namespace halo
