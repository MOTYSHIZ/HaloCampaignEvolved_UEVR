// Heap value scanner. See MemScan.hpp for why this exists and why it runs off the game thread.

#include "MemScan.hpp"

#if HALO_VR_DEV

#include "Config.hpp"
#include "uevr/API.hpp"

#include <Windows.h>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>
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
