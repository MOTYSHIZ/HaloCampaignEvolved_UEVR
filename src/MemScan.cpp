// Heap value scanner. See MemScan.hpp for why this exists and why it runs off the game thread.

#include "MemScan.hpp"

#if HALO_VR_DEV

#include "AimDirect.hpp"   // aim_direct_target/quat_src/writer_rip -- the AIMDIG dig's inputs
#include "BlamDrive.hpp"   // g_sim_tls_block, the TLS-graph walk's root
#include "Config.hpp"
#include "uevr/API.hpp"

#include <Windows.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>
#include <unordered_set>
#include <vector>

using namespace uevr;

namespace halo {
namespace {

std::atomic<bool> g_scanning{false};
bool g_prev_flag = false;   // tick-thread only

// Parse "0.625,0.625,0.35" into floats. Anything unparseable ends the list rather than being
// silently skipped -- a typo'd signature that quietly scans for fewer values would produce a flood
// of false hits and look like a successful scan.
std::vector<float> parse_values(const char* s) {
    std::vector<float> out;
    if (s == nullptr) return out;
    const char* p = s;
    while (*p && out.size() < 16) {
        char* end = nullptr;
        const float v = std::strtof(p, &end);
        if (end == p) break;
        out.push_back(v);
        p = end;
        while (*p == ',' || *p == ' ') ++p;
    }
    return out;
}

// Tolerant compare: tag floats are authored values, but they can arrive scaled or converted (a
// percentage becoming a fraction, degrees becoming radians), so an exact bit match would miss the
// value we are actually looking for. Relative tolerance keeps small and large values comparable.
bool near_eq(float a, float b) {
    const float d = std::fabs(a - b);
    const float m = std::fabs(b) > 1e-6f ? std::fabs(b) : 1.0f;
    return (d / m) < 0.0005f;
}

void scan_worker(std::vector<float> want) {
    if (want.empty()) { g_scanning = false; return; }

    SYSTEM_INFO si{};
    GetSystemInfo(&si);
    auto* addr = (const uint8_t*)si.lpMinimumApplicationAddress;
    auto* const maxAddr = (const uint8_t*)si.lpMaximumApplicationAddress;

    int hits = 0, regions = 0;
    size_t bytes = 0;
    std::vector<uint8_t> buf;

    API::get()->log_info("[Halo-CampE-UEVR] MEMSCAN start: %zu values, first=%.6f",
                         want.size(), want[0]);

    // HIT CAP. This used to be a bare `hits < 64`, which quietly turned every busy search into a
    // partial one: the walk stopped at the 64th coincidental match and never reached the rest of
    // the address space. Runs "completing" after 124 MB and after 13.3 GB looked like wild variance
    // in the process; they were just where the 64th hit landed. Worse, a capped run is
    // indistinguishable from an exhaustive one in the log, so "not found" was being read off
    // searches that had covered a fraction of memory.
    //
    // Now: configurable, far higher by default, and the summary SAYS when it truncated.
    const int hit_cap = (g_cfg.mem_scan_max > 0) ? g_cfg.mem_scan_max : 512;
    while (addr < maxAddr && hits < hit_cap) {
        MEMORY_BASIC_INFORMATION mbi{};
        if (VirtualQuery(addr, &mbi, sizeof(mbi)) == 0) break;
        auto* next = (const uint8_t*)mbi.BaseAddress + mbi.RegionSize;

        // Committed, readable, not guarded, not image code. Tag data is private or mapped heap;
        // skipping PAGE_GUARD matters because touching a guard page raises an exception rather
        // than reading, and skipping images keeps this off the exe pattern-scanner's territory.
        const bool readable = (mbi.State == MEM_COMMIT)
                           && !(mbi.Protect & PAGE_GUARD)
                           && !(mbi.Protect & PAGE_NOACCESS)
                           && (mbi.Type == MEM_PRIVATE || mbi.Type == MEM_MAPPED);

        const size_t span = sizeof(float) * want.size();
        if (readable && mbi.RegionSize >= span) {
            // CHUNKED READ.
            //
            // This used to resize the buffer to the WHOLE region and read it in one call. A single
            // multi-gigabyte region therefore asked for a multi-gigabyte allocation, and the process
            // simply vanished -- no exception, no minidump, nothing in the log. Three sessions were
            // lost to that during long scans before the cause was obvious.
            //
            // Fixed-size chunks instead, overlapping by (signature - 1) floats so a match straddling
            // a chunk boundary is still found rather than silently dropped.
            constexpr size_t CHUNK   = 4u << 20;          // 4 MB
            const     size_t overlap = span - sizeof(float);
            const     size_t bufsz   = (mbi.RegionSize < CHUNK) ? (size_t)mbi.RegionSize : CHUNK;
            if (buf.size() < bufsz) buf.resize(bufsz);

            bool counted = false;
            for (size_t roff = 0; roff + span <= mbi.RegionSize && hits < hit_cap; ) {
                const size_t want_read = ((size_t)(mbi.RegionSize - roff) < bufsz)
                                       ? (size_t)(mbi.RegionSize - roff) : bufsz;
                SIZE_T got = 0;
                if (!ReadProcessMemory(GetCurrentProcess(), (const uint8_t*)mbi.BaseAddress + roff,
                                       buf.data(), want_read, &got) || got == 0) {
                    break;   // region went away mid-walk; skip the rest of it
                }
                if (!counted) { ++regions; counted = true; }
                bytes += got;

                const size_t lim = got / sizeof(float);
                const auto* f = reinterpret_cast<const float*>(buf.data());
                for (size_t i = 0; i + want.size() <= lim; ++i) {
                    bool all = true;
                    for (size_t k = 0; k < want.size(); ++k) {
                        if (!near_eq(f[i + k], want[k])) { all = false; break; }
                    }
                    if (!all) continue;

                    const auto hitAddr = (uintptr_t)mbi.BaseAddress + roff + i * sizeof(float);
                    // Log surrounding floats too: the signature identifies the struct, but the
                    // FIELD we want sits at some offset from it, and seeing the neighbourhood is
                    // what makes that offset identifiable instead of guesswork.
                    char ctx[256]; int off = 0;
                    const size_t lo = (i >= 6) ? i - 6 : 0;
                    for (size_t k = lo; k < lo + 14 && k < lim && off < (int)sizeof(ctx) - 16; ++k) {
                        off += sprintf_s(ctx + off, sizeof(ctx) - off, "%.4g ", f[k]);
                    }
                    API::get()->log_info("[Halo-CampE-UEVR] MEMSCAN hit @ 0x%llX  ctx[-6..+8]: %s",
                                         (unsigned long long)hitAddr, ctx);

                    // FORENSICS, first 8 hits only (a saturated scan must not flood the log).
                    // The float context above identifies the RECORD; these identify the
                    // CONTAINER -- which is the actual quarry once a record is known to
                    // relocate: the allocation base says which arena it lives in and the
                    // hit's offset within it constrains the container header's position,
                    // while the raw qwords expose handles and pointers that float prints
                    // mangle (the navpoint hunt's records decode as 0x64-tagged Blam datum
                    // handles, invisible in the %.4g view).
                    if (hits < 8) {
                        API::get()->log_info("[Halo-CampE-UEVR] MEMSCAN forensic: alloc=0x%llX "
                                             "region=0x%llX size=0x%llX type=0x%lX prot=0x%lX "
                                             "hit=alloc+0x%llX",
                                             (unsigned long long)(uintptr_t)mbi.AllocationBase,
                                             (unsigned long long)(uintptr_t)mbi.BaseAddress,
                                             (unsigned long long)mbi.RegionSize,
                                             mbi.Type, mbi.Protect,
                                             (unsigned long long)(hitAddr - (uintptr_t)mbi.AllocationBase));
                        const size_t hit_b  = i * sizeof(float);
                        const size_t win_lo = ((hit_b >= 0x40) ? hit_b - 0x40 : 0) & ~7ull;
                        const size_t win_hi = ((hit_b + 0x80) < got) ? (hit_b + 0x80) : got;
                        char qline[240]; int qoff = 0; int emitted = 0;
                        int64_t line_rel = 0;
                        for (size_t b = win_lo; b + 8 <= win_hi; b += 8) {
                            if (emitted == 0) line_rel = (int64_t)b - (int64_t)hit_b;
                            qoff += sprintf_s(qline + qoff, sizeof(qline) - qoff, "%016llX ",
                                              (unsigned long long)*reinterpret_cast<const uint64_t*>(buf.data() + b));
                            if (++emitted == 8) {
                                API::get()->log_info("[Halo-CampE-UEVR] MEMSCAN q[hit%+lld]: %s",
                                                     (long long)line_rel, qline);
                                qoff = 0; emitted = 0; qline[0] = 0;
                            }
                        }
                        if (emitted > 0) {
                            API::get()->log_info("[Halo-CampE-UEVR] MEMSCAN q[hit%+lld]: %s",
                                                 (long long)line_rel, qline);
                        }
                    }
                    // Was a hard-coded 64 here too, independently of the loop cap: past 64 hits it
                    // bailed out of every region after its FIRST match, so a "1537 hit" result was
                    // really 64 real hits plus one-per-region sampling. Same constant, same lie.
                    if (++hits >= hit_cap) break;
                }

                if (got < want_read) break;              // short read: nothing further in this region
                if (want_read <= overlap) break;         // no forward progress possible
                roff += want_read - overlap;
            }
        }

        if (next <= addr) break;   // guard against a zero/stepping-backwards region
        addr = next;
    }

    // Say it plainly when the walk stopped early -- a truncated search that reads like a complete
    // one is worse than no search, because "not found" gets believed.
    if (hits >= hit_cap) {
        API::get()->log_info("[Halo-CampE-UEVR] MEMSCAN **TRUNCATED** at the %d-hit cap - the rest of the "
                             "address space was NOT searched. Raise memscanmax, or use a longer/rarer "
                             "value signature so there are fewer coincidental matches.", hit_cap);
    }
    API::get()->log_info("[Halo-CampE-UEVR] MEMSCAN done: %d hits over %d regions (%.1f MB)",
                         hits, regions, (double)bytes / (1024.0 * 1024.0));
    g_scanning = false;
}

} // namespace

// ---------------------------------------------------------------- DIFFERENTIAL (UNKNOWN-VALUE) SCAN
//
// WHY THIS EXISTS
//   Value scanning cannot find normalised controller input. The stored form is unknown -- raw,
//   post-deadzone, post-curve, radial or axial -- so you are guessing a transform, and every guess
//   lands in 0..1 where animation and mesh data saturate the space. Searching for 0.634438 returned
//   2000 hits inside 4 GB. Raising the cap only returns more noise.
//
//   This searches by BEHAVIOUR instead of by value: hold the stick still, snapshot; move it,
//   snapshot; keep only what changed. Nothing needs to be known about the representation.
//
// WHY TWO LEVELS
//   A flat snapshot is not affordable: ~5e9 float slots across a 20 GB address space. So stage 1
//   stores only a hash per 64 KB block (~5 MB), and per-float candidates are materialised ONLY
//   inside blocks that actually moved. That is what keeps this inside a game process rather than
//   needing an external debugger.
//
// USE (memdiff, watched like memscan):
//   1  snapshot block hashes            -- hold the stick STILL
//   2  rehash; changed blocks -> candidate floats   -- after MOVING the stick
//   3  re-read candidates, keep those that changed again  -- repeat, moving each time
//   0  clear
//   Each pass logs the survivor count; once it is small the addresses themselves are logged.
constexpr size_t DIFF_BLOCK = 64u << 10;    // coarse granularity for stage 1
// 8M candidates ~ 96 MB. Was 2M, which truncated on the very first real run: 201 changed blocks of
// 64 KB is ~3.4M floats, so a third of the changed memory was never examined and "no persistent
// field exists" would have been a conclusion drawn from 62% of the evidence. Sized so a normal
// changed-set fits whole; it still truncates loudly rather than silently if one does not.
constexpr size_t DIFF_MAX_CANDS = 1u << 23;

struct DiffBlock { uintptr_t base; uint32_t len; uint64_t hash; };
struct DiffCand  { uintptr_t addr; float val; };

static std::vector<DiffBlock> g_blocks;
static std::vector<DiffCand>  g_cands;
static std::atomic<bool>      g_diffing{false};
static int                    g_prev_diff = 0;

static uint64_t fnv1a(const uint8_t* p, size_t n) {
    uint64_t h = 1469598103934665603ull;
    for (size_t i = 0; i < n; ++i) { h ^= p[i]; h *= 1099511628211ull; }
    return h;
}

// Read a block, tolerating a region that vanished mid-walk. Returns bytes actually read.
static size_t read_block(uintptr_t base, uint8_t* dst, size_t n) {
    SIZE_T got = 0;
    if (!ReadProcessMemory(GetCurrentProcess(), (const void*)base, dst, n, &got)) return 0;
    return (size_t)got;
}

// Visit every readable region, same filter the value scanner uses.
template <typename F>
static void walk_regions(F&& fn) {
    const uint8_t* addr = nullptr;
    const uint8_t* maxAddr = (const uint8_t*)((uintptr_t)1 << 47);
    while (addr < maxAddr) {
        MEMORY_BASIC_INFORMATION mbi{};
        if (VirtualQuery(addr, &mbi, sizeof(mbi)) == 0) break;
        const auto* next = (const uint8_t*)mbi.BaseAddress + mbi.RegionSize;
        const bool readable = (mbi.State == MEM_COMMIT)
                           && !(mbi.Protect & PAGE_GUARD)
                           && !(mbi.Protect & PAGE_NOACCESS)
                           && (mbi.Type == MEM_PRIVATE || mbi.Type == MEM_MAPPED);
        if (readable) fn((uintptr_t)mbi.BaseAddress, (size_t)mbi.RegionSize);
        if (next <= addr) break;
        addr = next;
    }
}

static void diff_snapshot_worker() {
    g_blocks.clear();
    std::vector<uint8_t> buf(DIFF_BLOCK);
    size_t bytes = 0;
    walk_regions([&](uintptr_t base, size_t size) {
        for (size_t off = 0; off < size; off += DIFF_BLOCK) {
            const size_t n = (size - off < DIFF_BLOCK) ? (size - off) : DIFF_BLOCK;
            const size_t got = read_block(base + off, buf.data(), n);
            if (got == 0) continue;
            bytes += got;
            g_blocks.push_back(DiffBlock{ base + off, (uint32_t)got, fnv1a(buf.data(), got) });
        }
    });
    API::get()->log_info("[Halo-CampE-UEVR] MEMDIFF snapshot: %zu blocks (%.1f MB). Now MOVE the input and "
                         "set memdiff=2.", g_blocks.size(), bytes / (1024.0 * 1024.0));
    g_diffing = false;
}

static void diff_blocks_worker() {
    if (g_blocks.empty()) {
        API::get()->log_info("[Halo-CampE-UEVR] MEMDIFF: no snapshot - set memdiff=1 first");
        g_diffing = false; return;
    }
    g_cands.clear();
    std::vector<uint8_t> buf(DIFF_BLOCK);
    size_t changed = 0;
    bool truncated = false;
    for (const auto& b : g_blocks) {
        const size_t got = read_block(b.base, buf.data(), b.len);
        if (got != b.len) continue;                     // shrank or vanished: not comparable
        if (fnv1a(buf.data(), got) == b.hash) continue; // unchanged
        ++changed;
        const size_t lim = got / sizeof(float);
        const auto* f = reinterpret_cast<const float*>(buf.data());
        for (size_t i = 0; i < lim; ++i) {
            // Only plausible finite values are worth tracking; this is the one place a filter is
            // safe, because input and angles are small and finite whatever the representation.
            if (!std::isfinite(f[i]) || std::fabs(f[i]) > 1000.0f) continue;
            if (g_cands.size() >= DIFF_MAX_CANDS) { truncated = true; break; }
            g_cands.push_back(DiffCand{ b.base + i * sizeof(float), f[i] });
        }
        if (truncated) break;
    }
    if (truncated) {
        API::get()->log_info("[Halo-CampE-UEVR] MEMDIFF **TRUNCATED** at %zu candidates - narrowing is "
                             "incomplete. Move a SMALLER part of the state between snapshots.", g_cands.size());
    }
    API::get()->log_info("[Halo-CampE-UEVR] MEMDIFF pass 2: %zu/%zu blocks changed -> %zu candidate floats. "
                         "Move the input again and set memdiff=3 (repeat to narrow).",
                         changed, g_blocks.size(), g_cands.size());
    g_diffing = false;
}

// want_changed=true  (memdiff 3): keep what MOVED  -- run after moving the input. Drops constants.
// want_changed=false (memdiff 4): keep what HELD   -- run while the input is STILL. Drops churn.
//
// Neither filter alone converges. "Changed" keeps every timer, animation channel and physics value
// in the process, because those move every pass whatever the input does. "Held" keeps the entire
// static heap. Alternating them is what narrows: a real input field is the only thing that moves
// when you move the stick AND sits still when you do not.
static void diff_narrow_worker(bool want_changed) {
    if (g_cands.empty()) {
        API::get()->log_info("[Halo-CampE-UEVR] MEMDIFF: no candidates - run memdiff=1 then 2 first");
        g_diffing = false; return;
    }
    const size_t before = g_cands.size();
    std::vector<DiffCand> keep;
    keep.reserve(g_cands.size() / 4 + 16);
    for (const auto& c : g_cands) {
        float now = 0.0f;
        if (read_block(c.addr, (uint8_t*)&now, sizeof(now)) != sizeof(now)) continue;
        if (!std::isfinite(now)) continue;
        const bool moved = (now != c.val);
        if (moved == want_changed) keep.push_back(DiffCand{ c.addr, now });
    }
    g_cands.swap(keep);
    API::get()->log_info("[Halo-CampE-UEVR] MEMDIFF narrow (%s): %zu -> %zu survivors",
                         want_changed ? "moved" : "held", before, g_cands.size());
    if (g_cands.size() <= 40) {
        for (const auto& c : g_cands) {
            API::get()->log_info("[Halo-CampE-UEVR]   MEMDIFF cand 0x%llX = %.6f",
                                 (unsigned long long)c.addr, c.val);
        }
    } else {
        API::get()->log_info("[Halo-CampE-UEVR]   (still too many to list - move the input and set "
                             "memdiff=3 again)");
    }
    g_diffing = false;
}

// ---- TLS-GRAPH WALK (see MemScan.hpp) ----------------------------------------------------------

namespace {

std::atomic<bool> g_tls_walking{false};

bool tls_ptr_plausible(uint64_t v) {
    return v > 0x10000 && v < 0x7FFFFFFFFFFFull && (v & 7) == 0;
}

// EVERY read in this walker goes through ReadProcessMemory into a local buffer -- NEVER a direct
// dereference. v1 dereferenced with IsBadReadPtr guards and the session CRASHED ~10 s after the
// walk: IsBadReadPtr races frees and, worse, CONSUMES stack guard pages, which detonates the
// victim thread's next stack growth long after the walker finished. RPM cannot do either.
bool rpm(uintptr_t src, void* dst, size_t bytes, size_t* got) {
    SIZE_T g = 0;
    const BOOL ok = ReadProcessMemory(GetCurrentProcess(), (const void*)src, dst, bytes, &g);
    *got = (size_t)g;
    return ok && g > 0;
}

// Scan up to `bytes` at `base` (RPM-copied) for an adjacent float pair near (x,y). Returns the
// first offset or -1. Absolute tolerance in world units: 0.2 wu ~ 60 cm, generous enough for a
// player standing "at" the objective, tight enough to reject coincidences at map scale.
int64_t tls_scan_region(uintptr_t base, size_t bytes, float x, float y) {
    static thread_local std::vector<uint8_t> buf;
    if (buf.size() < bytes) buf.resize(bytes);
    size_t got = 0;
    if (!rpm(base, buf.data(), bytes, &got) || got < sizeof(float) * 2) return -1;
    const float* f = reinterpret_cast<const float*>(buf.data());
    const size_t n = got / sizeof(float);
    for (size_t i = 0; i + 1 < n; ++i) {
        if (std::fabs(f[i] - x) <= 0.2f && std::fabs(f[i + 1] - y) <= 0.2f) {
            return (int64_t)(i * sizeof(float));
        }
    }
    return -1;
}

void tls_walk_worker(uintptr_t block, float x, float y) {
    // STRUCTURED walk, not blind: the aim hunt already mapped this graph. block+0x20 -> sim
    // context; context+0x50 -> object table, entries at table + idx*24 with the object pointer
    // at +0x10 (the exact chain the projectile-direction getter uses, BlamAim.cpp). v1's blind
    // 2-level walk finished clean with 0 hits, so the pair is NOT in shallow ad-hoc structures
    // -- but a Blam OBJECT standing at the objective (nav flag, target vehicle) would carry it
    // in its record, and its TABLE INDEX + HANDLE are the stable reference the markers need.
    API::get()->log_info("[Halo-CampE-UEVR] TLSNAV v2: block 0x%llX, hunting (%.3f, %.3f) wu "
                         "in the OBJECT TABLE", (unsigned long long)block, x, y);
    size_t got = 0;
    uintptr_t ctx = 0, table = 0;
    if (!rpm(block + 0x20, &ctx, 8, &got) || ctx == 0) {
        API::get()->log_info("[Halo-CampE-UEVR] TLSNAV v2: no sim context at block+0x20");
        g_tls_walking = false;
        return;
    }
    if (!rpm(ctx + 0x50, &table, 8, &got) || table == 0) {
        API::get()->log_info("[Halo-CampE-UEVR] TLSNAV v2: no object table at ctx+0x50");
        g_tls_walking = false;
        return;
    }
    API::get()->log_info("[Halo-CampE-UEVR] TLSNAV v2: ctx=0x%llX table=0x%llX -- walking 2048 slots",
                         (unsigned long long)ctx, (unsigned long long)table);

    // REPRESENTATION SWEEP. v2 (wu-float32 only, first 0x200) found 0 hits across 89 objects --
    // but the PLAYER is one of those objects and we stand at (x,y), so a zero means object
    // records simply do not hold position as a wu-float32 pair. So scan each object's first
    // 0x400 for the pair in FOUR representations at once; the player is guaranteed to match in
    // whichever the records actually use, revealing the format AND the offset. Cm values are
    // reconstructed from the wu inputs (x,y were cm/304.8 upstream).
    const float  wu_x = x,          wu_y = y;
    const float  cm_x = x * 304.8f, cm_y = y * 304.8f;
    const double cmd_x = (double)cm_x, cmd_y = (double)cm_y;
    const double wud_x = (double)x,    wud_y = (double)y;
    static thread_local std::vector<uint8_t> rec;
    if (rec.size() < 0x400) rec.resize(0x400);

    int live = 0, hits = 0;
    for (int idx = 0; idx < 2048 && hits < 12; ++idx) {
        uintptr_t obj = 0;
        if (!rpm(table + (uintptr_t)idx * 24 + 0x10, &obj, 8, &got)) break;
        if (!tls_ptr_plausible(obj)) continue;
        ++live;
        size_t rgot = 0;
        if (!rpm(obj, rec.data(), 0x400, &rgot) || rgot < 32) continue;
        const size_t nf = rgot / 4, nd = rgot / 8;
        const float*  ff = reinterpret_cast<const float*>(rec.data());
        const double* dd = reinterpret_cast<const double*>(rec.data());
        // TOLERANCE ~5 m (wu 1.64 = 5 m / 3.048, cm 500). The player can only get ~2 m from this
        // objective (field-reported); 5 m gives that a safety margin without approaching object
        // spacing (hundreds of wu), so false matches stay unlikely. The player object still
        // matches its OWN coords well inside this, so format detection is unaffected -- the
        // window only matters for surfacing a DISTINCT objective-object standing a few m away.
        int64_t off = -1; const char* rep = nullptr;
        for (size_t i = 0; i + 1 < nf && off < 0; ++i) {
            if (std::fabs(ff[i] - wu_x) <= 1.64f && std::fabs(ff[i+1] - wu_y) <= 1.64f) {
                off = (int64_t)(i*4); rep = "wu-f32";
            } else if (std::fabs(ff[i] - cm_x) <= 500.0f && std::fabs(ff[i+1] - cm_y) <= 500.0f) {
                off = (int64_t)(i*4); rep = "cm-f32";
            }
        }
        for (size_t i = 0; i + 1 < nd && off < 0; ++i) {
            if (std::fabs(dd[i] - wud_x) <= 1.64 && std::fabs(dd[i+1] - wud_y) <= 1.64) {
                off = (int64_t)(i*8); rep = "wu-f64";
            } else if (std::fabs(dd[i] - cmd_x) <= 500.0 && std::fabs(dd[i+1] - cmd_y) <= 500.0) {
                off = (int64_t)(i*8); rep = "cm-f64";
            }
        }
        if (off >= 0) {
            uint64_t head[2] = {}; size_t g2 = 0; rpm(obj, head, sizeof(head), &g2);
            API::get()->log_info("[Halo-CampE-UEVR] TLSNAV v3 HIT idx=%d @0x%llX +0x%llX %s | "
                                 "head %016llX %016llX", idx, (unsigned long long)obj,
                                 (unsigned long long)off, rep,
                                 (unsigned long long)head[0], (unsigned long long)head[1]);
            ++hits;
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] TLSNAV v3 done: %d hit(s) across %d live objects "
                         "(4 representations)", hits, live);
    g_tls_walking = false;
}

}  // namespace

// ---- MANAGER-ROOTED GRAPH WALK (see MemScan.hpp) -----------------------------------------------

namespace {

std::atomic<bool> g_graph_walking{false};

struct GraphNode {
    uintptr_t addr;
    int       depth;
    int       parent;      // index into the visit list, -1 for the root
    int32_t   from_off;    // offset within the parent that pointed here
};

void graph_walk_worker(uintptr_t root, float x, float y) {
    // The whole point of this walker vs the TLS one: the ROOT IS STABLE (a UE object reached
    // through the widget tree), so a hit's offset chain is a RESOLUTION PATH, not a one-session
    // address. Depth 4 with a hard node budget -- enough for manager -> interface -> provider ->
    // list -> record, which is the shape the live inspection showed.
    // ⚠️ THE BUDGET IS AN ENQUEUE LIMIT, NOT A PROCESSING LIMIT. v1 put `nodes.size() <
    // MAX_NODES` in the LOOP CONDITION, so the walk TERMINATED the instant the queue filled --
    // it scanned a few dozen blocks in 6 ms and reported "0 hits over 4000 nodes", which reads
    // exactly like a clean negative and is not one. Processing now runs to the end of the queue;
    // only enqueueing stops at the cap.
    constexpr int    MAX_DEPTH = 4;
    constexpr size_t MAX_NODES = 40000;
    constexpr size_t BLOCK     = 0x600;   // bytes scanned/enumerated per node
    constexpr double TIME_BUDGET_S = 20.0;

    API::get()->log_info("[Halo-CampE-UEVR] NAVGRAPH: walking from manager 0x%llX for (%.3f, %.3f) wu",
                         (unsigned long long)root, x, y);

    std::vector<GraphNode> nodes;
    std::unordered_set<uintptr_t> seen;   // O(1) dedup: a linear scan is O(n^2) at this scale
    nodes.reserve(MAX_NODES);
    seen.reserve(MAX_NODES);
    nodes.push_back({root, 0, -1, 0});
    seen.insert(root);

    std::vector<uint8_t> buf(BLOCK);
    int hits = 0;
    size_t scanned = 0;
    const auto t0 = std::chrono::steady_clock::now();

    for (size_t n = 0; n < nodes.size() && hits < 6; ++n) {
        if ((n & 0xFF) == 0) {
            const double el = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            if (el > TIME_BUDGET_S) {
                API::get()->log_info("[Halo-CampE-UEVR] NAVGRAPH: time budget reached at node %zu", n);
                break;
            }
        }
        ++scanned;
        const GraphNode cur = nodes[n];
        size_t got = 0;
        if (!rpm(cur.addr, buf.data(), BLOCK, &got) || got < 16) continue;

        // 1) Does THIS block hold the pair? Both float widths, 5 m tolerance.
        {
            const float* ff = reinterpret_cast<const float*>(buf.data());
            const double* dd = reinterpret_cast<const double*>(buf.data());
            int64_t off = -1; const char* rep = nullptr;
            for (size_t i = 0; i + 1 < got / 4 && off < 0; ++i) {
                if (std::fabs(ff[i] - x) <= 1.64f && std::fabs(ff[i+1] - y) <= 1.64f) {
                    off = (int64_t)(i * 4); rep = "wu-f32";
                } else if (std::fabs(ff[i] - x * 304.8f) <= 500.0f
                        && std::fabs(ff[i+1] - y * 304.8f) <= 500.0f) {
                    off = (int64_t)(i * 4); rep = "cm-f32";
                }
            }
            for (size_t i = 0; i + 1 < got / 8 && off < 0; ++i) {
                if (std::fabs(dd[i] - (double)x) <= 1.64 && std::fabs(dd[i+1] - (double)y) <= 1.64) {
                    off = (int64_t)(i * 8); rep = "wu-f64";
                } else if (std::fabs(dd[i] - (double)x * 304.8) <= 500.0
                        && std::fabs(dd[i+1] - (double)y * 304.8) <= 500.0) {
                    off = (int64_t)(i * 8); rep = "cm-f64";
                }
            }
            if (off >= 0) {
                // Reconstruct the chain: root +o1 -> +o2 -> ... -> +off
                char chain[256]; int co = 0;
                int stack[MAX_DEPTH + 1]; int sn = 0;
                for (int p = (int)n; p > 0 && sn <= MAX_DEPTH; p = nodes[p].parent) stack[sn++] = p;
                co += sprintf_s(chain + co, sizeof(chain) - co, "manager");
                for (int s = sn - 1; s >= 0; --s) {
                    co += sprintf_s(chain + co, sizeof(chain) - co, " +0x%X ->",
                                    (unsigned)nodes[stack[s]].from_off);
                }
                co += sprintf_s(chain + co, sizeof(chain) - co, " +0x%llX",
                                (unsigned long long)off);
                API::get()->log_info("[Halo-CampE-UEVR] NAVGRAPH HIT (%s) depth=%d @0x%llX: %s",
                                     rep, cur.depth, (unsigned long long)cur.addr, chain);
                ++hits;
            }
        }

        // 2) Enqueue this block's pointers for the next level.
        if (cur.depth >= MAX_DEPTH) continue;
        for (size_t o = 0; o + 8 <= got && nodes.size() < MAX_NODES; o += 8) {
            const uint64_t p = *reinterpret_cast<const uint64_t*>(buf.data() + o);
            if (!tls_ptr_plausible(p)) continue;
            if (!seen.insert((uintptr_t)p).second) continue;   // already queued
            nodes.push_back({(uintptr_t)p, cur.depth + 1, (int)n, (int32_t)o});
        }
    }

    // Report SCANNED vs QUEUED separately: they diverge whenever the cap binds, and conflating
    // them is what made the v1 bug look like a result.
    API::get()->log_info("[Halo-CampE-UEVR] NAVGRAPH done: %d hit(s) -- scanned %zu of %zu queued "
                         "nodes (depth<=%d, cap %s)",
                         hits, scanned, nodes.size(), MAX_DEPTH,
                         nodes.size() >= MAX_NODES ? "BOUND (graph larger than budget)" : "not reached");
    g_graph_walking = false;
}

// ---- AIMDIG: the AimDirect derivation-chain dig (see MemScan.hpp) ------------------------------
std::atomic<bool> g_aimdig_walking{false};
bool              g_prev_aimdig = false;

// Whole-ALLOCATION span, not one region: VirtualQuery forward while AllocationBase matches. Blam
// reserves large blocks and commits pieces, so RegionSize alone under-reports by design.
bool aimdig_alloc_span(uintptr_t a, uintptr_t* lo, uintptr_t* hi, DWORD* type) {
    MEMORY_BASIC_INFORMATION mbi{};
    if (a == 0 || VirtualQuery((void*)a, &mbi, sizeof(mbi)) == 0) return false;
    const uintptr_t base = (uintptr_t)mbi.AllocationBase;
    if (base == 0) return false;
    uintptr_t cur = base, end = base;
    for (int guard = 0; guard < 65536; ++guard) {
        MEMORY_BASIC_INFORMATION m2{};
        if (VirtualQuery((void*)cur, &m2, sizeof(m2)) == 0) break;
        if ((uintptr_t)m2.AllocationBase != base) break;
        end = (uintptr_t)m2.BaseAddress + m2.RegionSize;
        cur = end;
    }
    *lo = base; *hi = end; *type = mbi.Type;
    return true;
}

void aimdig_worker(uintptr_t l2, uintptr_t qsrc, uintptr_t tls_block, uintptr_t wrip,
                   uintptr_t pc_root) {
    const uintptr_t exe = (uintptr_t)GetModuleHandleA(nullptr);
    API::get()->log_info("[Halo-CampE-UEVR] AIMDIG: L2=0x%llX quatsrc=0x%llX simTLS=0x%llX "
                         "writer=dll+0x%llX",
                         (unsigned long long)l2, (unsigned long long)qsrc,
                         (unsigned long long)tls_block,
                         (unsigned long long)(wrip >= exe ? wrip - exe : 0));

    // 1) Allocation census. obj is the quat-sync's base register (source = obj+0x10).
    struct Item { const char* name; uintptr_t addr; uintptr_t lo, hi; bool ok; };
    Item items[3] = { {"L2",     l2,                          0, 0, false},
                      {"obj",    qsrc != 0 ? qsrc - 0x10 : 0, 0, 0, false},
                      {"simTLS", tls_block,                   0, 0, false} };
    for (auto& it : items) {
        DWORD type = 0;
        it.ok = aimdig_alloc_span(it.addr, &it.lo, &it.hi, &type);
        if (it.ok) {
            API::get()->log_info("[Halo-CampE-UEVR] AIMDIG: %s=0x%llX alloc=[0x%llX..0x%llX) "
                                 "size=0x%llX type=0x%X offset-in-alloc=0x%llX",
                                 it.name, (unsigned long long)it.addr,
                                 (unsigned long long)it.lo, (unsigned long long)it.hi,
                                 (unsigned long long)(it.hi - it.lo), (unsigned)type,
                                 (unsigned long long)(it.addr - it.lo));
        } else {
            API::get()->log_info("[Halo-CampE-UEVR] AIMDIG: %s=0x%llX - no allocation (null or "
                                 "unreadable)", it.name, (unsigned long long)it.addr);
        }
    }
    if (items[0].ok) {
        const bool obj_same = items[1].ok && items[1].lo == items[0].lo;
        const bool tls_same = items[2].ok && items[2].lo == items[0].lo;
        API::get()->log_info("[Halo-CampE-UEVR] AIMDIG: sameAlloc(L2,obj)=%d sameAlloc(L2,simTLS)=%d "
                             "L2-simTLS=%+lld L2-obj=%+lld  <-- offsets stable across launches ONLY "
                             "if the pair shares an allocation or the allocator is deterministic",
                             obj_same ? 1 : 0, tls_same ? 1 : 0,
                             items[2].ok ? (long long)(l2 - items[2].addr) : 0ll,
                             items[1].ok ? (long long)(l2 - items[1].addr) : 0ll);
    }

    // 2) STATIC ROOTS: exe data sections holding a pointer into L2's or obj's allocation. A hit
    // here is the jackpot -- module + RVA -> deref -> offset is a per-launch derivation chain
    // with no hunt, the BlamDrive shape.
    if (exe != 0 && items[0].ok) {
        auto* dos = (const IMAGE_DOS_HEADER*)exe;
        auto* nt  = (const IMAGE_NT_HEADERS64*)(exe + dos->e_lfanew);
        auto* sec = IMAGE_FIRST_SECTION(nt);
        int found = 0;
        std::vector<uint8_t> chunk(0x40000);
        for (WORD s = 0; s < nt->FileHeader.NumberOfSections && found < 16; ++s) {
            const auto& sc = sec[s];
            if (!(sc.Characteristics & IMAGE_SCN_MEM_READ)) continue;
            if (sc.Characteristics & IMAGE_SCN_MEM_EXECUTE) continue;
            const uintptr_t beg = exe + sc.VirtualAddress;
            const uintptr_t sz  = sc.Misc.VirtualSize;
            for (uintptr_t o = 0; o + 8 <= sz && found < 16; o += chunk.size()) {
                size_t got = 0;
                const size_t want = (sz - o < chunk.size()) ? (size_t)(sz - o) : chunk.size();
                if (!rpm(beg + o, chunk.data(), want, &got) || got < 8) continue;
                for (size_t i = 0; i + 8 <= got && found < 16; i += 8) {
                    const uint64_t v = *reinterpret_cast<const uint64_t*>(chunk.data() + i);
                    for (int t = 0; t < 2; ++t) {
                        if (!items[t].ok || v < items[t].lo || v >= items[t].hi) continue;
                        ++found;
                        API::get()->log_info("[Halo-CampE-UEVR] AIMDIG: STATIC ROOT dll+0x%llX -> "
                                             "0x%llX (into %s's alloc; delta to %s = %+lld)",
                                             (unsigned long long)(beg + o + i - exe),
                                             (unsigned long long)v, items[t].name, items[t].name,
                                             (long long)(v - items[t].addr));
                        break;
                    }
                }
            }
        }
        API::get()->log_info("[Halo-CampE-UEVR] AIMDIG: static scan done - %d pointer(s) from exe "
                             "data sections into the L2/obj allocations%s", found,
                             found == 0 ? " (no static root: the chain must route through TLS or "
                                          "another heap structure)" : "");
    }

    // 3) POINTER-GRAPH WALKS: a chain from a re-derivable root that lands on (or just above) L2.
    // The window reaches below L2 because a chain normally points at the CONTAINING record, not
    // at the rotator field itself.
    // packed4: step 4 bytes and accept 4-ALIGNED pointers when enqueueing. Blam's structures are
    // 4-byte packed, and rejecting `(v & 7)` is precisely the filter that hid the object table from
    // this project for days (see the note on read_ptr in BlamAim.cpp). UE-side roots stay on the
    // 8-aligned fast path; only the Blam-targeted walks pay the 2x cost.
    auto walk = [&](const char* rootname, uintptr_t root, uintptr_t win_lo, uintptr_t win_hi,
                    uintptr_t target_addr, const char* target, int max_depth, bool packed4 = false) {
        if (root == 0 || win_lo == 0) return;
        const int        MAX_DEPTH = max_depth;
        constexpr size_t MAX_NODES = 60000;
        constexpr size_t BLOCK     = 0x800;
        constexpr double TIME_BUDGET_S = 15.0;
        std::vector<GraphNode> nodes;
        std::unordered_set<uintptr_t> seen;
        nodes.reserve(MAX_NODES); seen.reserve(MAX_NODES);
        nodes.push_back({root, 0, -1, 0});
        seen.insert(root);
        std::vector<uint8_t> buf(BLOCK);
        int hits = 0; size_t scanned = 0;
        const auto t0 = std::chrono::steady_clock::now();
        for (size_t n = 0; n < nodes.size() && hits < 8; ++n) {
            if ((n & 0x3F) == 0) {
                const double el = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t0).count();
                if (el > TIME_BUDGET_S) {
                    API::get()->log_info("[Halo-CampE-UEVR] AIMDIG: %s->%s walk time budget at node %zu",
                                         rootname, target, n);
                    break;
                }
            }
            ++scanned;
            const GraphNode cur = nodes[n];
            size_t got = 0;
            if (!rpm(cur.addr, buf.data(), BLOCK, &got) || got < 8) continue;
            const size_t step = packed4 ? 4u : 8u;
            for (size_t o = 0; o + 8 <= got; o += step) {
                uint64_t p = 0;
                memcpy(&p, buf.data() + o, 8);   // unaligned-safe: o is only 4-aligned in packed4
                if ((uintptr_t)p >= win_lo && (uintptr_t)p < win_hi) {
                    ++hits;
                    char chain[256]; int co = 0;
                    int stack[9]; int sn = 0;   // fixed: MAX_DEPTH is a runtime bound now
                    for (int q = (int)n; q > 0 && sn < (int)(sizeof(stack) / sizeof(stack[0]));
                         q = nodes[q].parent) stack[sn++] = q;
                    co += sprintf_s(chain + co, sizeof(chain) - co, "%s", rootname);
                    for (int si = sn - 1; si >= 0; --si) {
                        co += sprintf_s(chain + co, sizeof(chain) - co, " +0x%X ->",
                                        (unsigned)nodes[stack[si]].from_off);
                    }
                    co += sprintf_s(chain + co, sizeof(chain) - co, " +0x%llX",
                                    (unsigned long long)o);
                    API::get()->log_info("[Halo-CampE-UEVR] AIMDIG HIT(%s): %s = 0x%llX "
                                         "(delta to %s %+lld)", target, chain,
                                         (unsigned long long)p, target,
                                         (long long)((uintptr_t)p - target_addr));
                }
                const bool enqueueable = packed4
                    ? (p > 0x10000 && p < 0x7FFFFFFFFFFFull && (p & 3) == 0)
                    : tls_ptr_plausible(p);
                if (cur.depth < MAX_DEPTH && enqueueable
                    && nodes.size() < MAX_NODES && seen.insert((uintptr_t)p).second) {
                    nodes.push_back({(uintptr_t)p, cur.depth + 1, (int)n, (int32_t)o});
                }
            }
        }
        API::get()->log_info("[Halo-CampE-UEVR] AIMDIG: %s->%s walk done - %d hit(s), scanned %zu of "
                             "%zu queued (depth<=%d)", rootname, target, hits, scanned, nodes.size(),
                             MAX_DEPTH);
    };
    const uintptr_t objbase_pre = (qsrc != 0) ? qsrc - 0x10 - 0x1C0 : 0;
    // THE ROOT THAT MATTERS: the PlayerController is a UObject we can fetch through reflection in
    // microseconds, every launch, with no scan and no hook. If objbase hangs off it within a few
    // hops, rung 1 is a pure pointer-deref chain. Searched for objbase FIRST (the object is the
    // real prize -- L2 follows from it by the two hops already proven) and for L2's window second.
    if (pc_root != 0 && objbase_pre != 0) {
        walk("PC", pc_root, objbase_pre - 0x40, objbase_pre + 0x40, objbase_pre, "objbase", 4);
        walk("PC", pc_root, l2 - 0x8000, l2 + 0x1000, l2, "L2win", 4);
    } else {
        API::get()->log_info("[Halo-CampE-UEVR] AIMDIG: no PlayerController recorded - skipping the "
                             "PC-rooted walks (the cheapest possible root)");
    }
    walk("simTLS", tls_block, l2 - 0x8000, l2 + 0x1000, l2, "L2win", 3);
    walk("obj", items[1].addr, l2 - 0x8000, l2 + 0x1000, l2, "L2win", 3);

    // THE BLAMDRIVE QUESTION. BlamDrive already resolves this record dynamically, but only from
    // the SIM THREAD (gs:[0x58]) -- which is the entire reason the getter hook exists, and why the
    // hookless tier 2 has to enumerate threads and read TEBs. If the record is reachable from the
    // PlayerController by pointer derefs, resolution stops depending on the sim thread altogether.
    // The record is 4-BYTE PACKED (Blam structs are), so the window is tight and the walk must not
    // assume 8-alignment of the value -- it is a pointer we are matching, not a field.
    const uintptr_t rec = blam_control_record();
    if (rec != 0) {
        API::get()->log_info("[Halo-CampE-UEVR] AIMDIG: blam control record = 0x%llX (L2-rec = %+lld) "
                             "- probing whether the UE side can reach it",
                             (unsigned long long)rec, (long long)(l2 - rec));
        if (pc_root != 0) {
            walk("PC", pc_root, rec - 0x400, rec + 0x400, rec, "blamREC", 4, /*packed4=*/true);
        }
        // The named UE<->Blam bridge: /Script/BlamSynchronization.BlamUnitComponent. Reflected, so
        // a hit here is resolvable in microseconds with no scan at all.
        if (auto* pawn = API::get()->get_local_pawn(0)) {
            walk("pawn", (uintptr_t)pawn, rec - 0x400, rec + 0x400, rec, "blamREC", 4, true);
        }
    } else {
        API::get()->log_info("[Halo-CampE-UEVR] AIMDIG: no blam control record resolved "
                             "(blamangles must be on and have resolved) - skipping the BlamDrive probe");
    }

    // 4) VTABLE CENSUS -- the candidate RESOLVER, measured.
    //
    // The dig's finding: the quat-sync's rcx is objbase+0x1C0, and [objbase+0x1C0]+0x20 IS L2.
    // So the rotator is two fixed struct hops from an object whose CLASS is identified by its
    // vtable pointer -- and a vtable address is module-relative, i.e. build-stable and
    // launch-stable, exactly the anchor the heap address could never be. (RTTI would be nicer
    // still but UE ships /GR-, so the vtable RVA is the identity.)
    //
    // This measures whether a resolver built on it would work: how many live instances carry
    // that vtable, and how many of those reach the SAME L2 the watch found. One instance whose
    // chain lands on L2 = the hunt can be demoted to a fallback.
    const uintptr_t objbase = (qsrc != 0) ? qsrc - 0x10 - 0x1C0 : 0;
    if (objbase != 0 && !IsBadReadPtr((const void*)objbase, 8)) {
        const uintptr_t vt = *(const uintptr_t*)objbase;
        API::get()->log_info("[Halo-CampE-UEVR] AIMDIG: objbase=0x%llX vtable=0x%llX (dll+0x%llX) "
                             "chain objbase+0x1C0 -> +0x20 = 0x%llX (L2 %s)",
                             (unsigned long long)objbase, (unsigned long long)vt,
                             (unsigned long long)(vt >= exe ? vt - exe : 0),
                             (unsigned long long)(*(const uintptr_t*)(objbase + 0x1C0) + 0x20),
                             (*(const uintptr_t*)(objbase + 0x1C0) + 0x20) == l2 ? "MATCH" : "differs");

        int instances = 0, chain_ok = 0, chain_hits_l2 = 0;
        uintptr_t first_hit = 0;
        MEMORY_BASIC_INFORMATION mbi{};
        uintptr_t a = 0x10000;
        std::vector<uint8_t> buf(0x10000);
        const auto t0 = std::chrono::steady_clock::now();
        while (a < 0x7FFFFFFF0000ull && VirtualQuery((void*)a, &mbi, sizeof(mbi)) != 0) {
            const uintptr_t next = (uintptr_t)mbi.BaseAddress + mbi.RegionSize;
            const bool usable = (mbi.State == MEM_COMMIT) && (mbi.Type == MEM_PRIVATE)
                && !(mbi.Protect & (PAGE_NOACCESS | PAGE_GUARD))
                && (mbi.Protect & (PAGE_READWRITE | PAGE_READONLY | PAGE_WRITECOPY
                                   | PAGE_EXECUTE_READ | PAGE_EXECUTE_READWRITE));
            if (usable) {
                for (uintptr_t o = 0; o < mbi.RegionSize; o += buf.size()) {
                    const size_t want = (size_t)((mbi.RegionSize - o < buf.size())
                                                 ? (mbi.RegionSize - o) : buf.size());
                    size_t got = 0;
                    if (!rpm((uintptr_t)mbi.BaseAddress + o, buf.data(), want, &got) || got < 8) continue;
                    for (size_t i = 0; i + 8 <= got; i += 8) {
                        if (*reinterpret_cast<const uint64_t*>(buf.data() + i) != (uint64_t)vt) continue;
                        const uintptr_t cand = (uintptr_t)mbi.BaseAddress + o + i;
                        ++instances;
                        if (first_hit == 0) first_hit = cand;
                        uintptr_t inner = 0; size_t g2 = 0;
                        if (rpm(cand + 0x1C0, &inner, 8, &g2) && g2 == 8 && inner != 0) {
                            double rot[2] = {0, 0};
                            if (rpm(inner + 0x20, rot, sizeof(rot), &g2) && g2 == sizeof(rot)
                                && std::isfinite(rot[0]) && std::isfinite(rot[1])
                                && std::fabs(rot[0]) <= 360.0 && std::fabs(rot[1]) <= 360.0) {
                                ++chain_ok;
                                if (inner + 0x20 == l2) ++chain_hits_l2;
                            }
                        }
                    }
                }
            }
            if (next <= a) break;
            a = next;
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > 20.0) {
                API::get()->log_info("[Halo-CampE-UEVR] AIMDIG: vtable census time budget reached");
                break;
            }
        }
        const double ms = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count() * 1000.0;
        API::get()->log_info("[Halo-CampE-UEVR] AIMDIG: vtable census - %d instance(s) of dll+0x%llX, "
                             "%d with a plausible rotator via +0x1C0 -> +0x20, %d landing on THE L2 "
                             "(first 0x%llX) in %.0f ms  <-- 1 = the hunt can become a fallback",
                             instances, (unsigned long long)(vt >= exe ? vt - exe : 0),
                             chain_ok, chain_hits_l2, (unsigned long long)first_hit, ms);
    }

    API::get()->log_info("[Halo-CampE-UEVR] AIMDIG: report complete - compare these lines across "
                         "two launches; whatever stays constant is the anchor");
    g_aimdig_walking = false;
}

}  // namespace

void nav_graph_scan(uintptr_t manager_root, float wu_x, float wu_y) {
    if (manager_root == 0) {
        API::get()->log_info("[Halo-CampE-UEVR] NAVGRAPH: no navpoints manager resolved "
                             "(navworld must have found the widget tree first)");
        return;
    }
    if (g_graph_walking.exchange(true)) return;
    std::thread(graph_walk_worker, manager_root, wu_x, wu_y).detach();
}

void aim_chain_scan_tick() {
    const bool want = g_cfg.aim_dig;
    if (want == g_prev_aimdig) return;
    g_prev_aimdig = want;
    if (!want) return;
    const uintptr_t l2 = aim_direct_target();
    if (l2 == 0) {
        API::get()->log_info("[Halo-CampE-UEVR] AIMDIG: aimdirect is not resolved yet - aim with "
                             "motion until it reports Ready, then set aimdig=1 again");
        return;
    }
    if (g_aimdig_walking.exchange(true)) return;
    std::thread(aimdig_worker, l2, aim_direct_quat_src(),
                g_sim_tls_block.load(std::memory_order_relaxed),
                aim_direct_writer_rip(), aim_direct_known_pc()).detach();
}

void nav_tls_scan(float wu_x, float wu_y) {
    const uintptr_t block = g_sim_tls_block.load(std::memory_order_relaxed);
    if (block == 0) {
        API::get()->log_info("[Halo-CampE-UEVR] TLSNAV: sim TLS block not published yet "
                             "(blamangles must have resolved at least once)");
        return;
    }
    if (g_tls_walking.exchange(true)) return;
    std::thread(tls_walk_worker, block, wu_x, wu_y).detach();
}

void mem_scan_tick() {
    const bool want = g_cfg.mem_scan;
    if (want == g_prev_flag) return;
    g_prev_flag = want;
    if (!want) return;
    if (g_scanning.exchange(true)) return;   // one at a time

    auto vals = parse_values(g_cfg.mem_scan_vals);
    if (vals.empty()) {
        API::get()->log_info("[Halo-CampE-UEVR] MEMSCAN: memscanvals is empty or unparseable - nothing to search for");
        g_scanning = false;
        return;
    }
    std::thread(scan_worker, std::move(vals)).detach();
}

void mem_diff_tick() {
    const int stage = g_cfg.mem_diff;
    if (stage == g_prev_diff) return;
    g_prev_diff = stage;
    if (stage == 0) {
        g_blocks.clear(); g_blocks.shrink_to_fit();
        g_cands.clear();  g_cands.shrink_to_fit();
        API::get()->log_info("[Halo-CampE-UEVR] MEMDIFF cleared");
        return;
    }
    if (g_diffing.exchange(true)) return;   // one at a time, same as the value scanner
    switch (stage) {
    case 1: std::thread(diff_snapshot_worker).detach();      break;
    case 2: std::thread(diff_blocks_worker).detach();        break;
    case 4: std::thread(diff_narrow_worker, false).detach(); break;  // keep what HELD (input still)
    default: std::thread(diff_narrow_worker, true).detach(); break;  // keep what MOVED (input moved)
    }
}

} // namespace halo

#endif // HALO_VR_DEV
