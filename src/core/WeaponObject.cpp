#include "core/WeaponObject.hpp"
#include "core/config/CfgRead.hpp"

#include "Config.hpp"
#include "core/Services.hpp"
#include "core/UnitState.hpp"   // resolve_object_by_datum: the weapon object through the sim's table
#include "uevr/API.hpp"

#include <Windows.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>

using namespace uevr;

namespace halo {

namespace {

// The node transform, as Blam stores it (the layout BlamPalette.cpp validates against the capture
// bank count): a scale, then three basis vectors and a position, 0x34 bytes.
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
static_assert(sizeof(PaletteNode) == 0x34, "palette node must be 52 bytes");

bool readable(const void* p, size_t n) { return p != nullptr && !IsBadReadPtr(p, n); }

// The shared render capture pointer and the capture context stride (BlamPalette.cpp documents both);
// the copy scan below searches the capture region for copies of the weapon's nodes.
constexpr uintptr_t RVA_SHARED_CAPTURE_PTR = 0x1831220;
constexpr uintptr_t CAPTURE_CTX_STRIDE     = 0x30600;

// THE RENDER CAPTURE OF OBJECT NODES (2026-09-03). Found by a read+write hardware watch on the
// pistol's slide node: the only reader outside the sim's own rebuild was VCRUNTIME memcpy, called
// from dll+0x5AEA8E inside this function. Read statically from the shipped DLL (capstone):
//   * .pdata entry 0x5ADBC0..0x5AEB15; one argument, the object DATUM in ecx (ebx = datum, bx =
//     its index into the object table, stride 24 at [table+0x10]); rdx/r8/r9 are only ever loaded
//     from memory before use and no stack argument slot is read -- a (uint32_t) signature is exact;
//   * it reads the sim TLS block through the same index constant as RVA_TLS_INDEX (0xD72730);
//   * it recurses into child objects (call to itself at +0x5AE693);
//   * at +0x5AEA4E..+0x5AEA89 it takes the capture context's bank byte, multiplies by
//     CAPTURE_CTX_STRIDE, reads the bank record's node-pool pointer at +0x24008, adds a per-object
//     offset, and memcpy's node_count x 52 bytes from the object's node block. THAT copy is what
//     BlamMeshSynchronization renders, which is why writes to the live node never showed.
// A PRE-hook applying the slide pull to the live node, for the weapon's datum only, just before
// the original runs, is what makes the pull render. rax is returned through untouched.
constexpr uintptr_t RVA_OBJ_CAPTURE = 0x5ADBC0;
constexpr uint8_t OBJ_CAPTURE_PROLOGUE[] = {
    0x40, 0x55,                   // push rbp
    0x53, 0x56, 0x57,             // push rbx / rsi / rdi
    0x41, 0x54, 0x41, 0x55,       // push r12 / r13
    0x41, 0x56, 0x41, 0x57,       // push r14 / r15
    0x48, 0x8D, 0x6C, 0x24, 0xF8, // lea rbp, [rsp-8]
    0x48, 0x81, 0xEC, 0x08, 0x01, 0x00, 0x00,   // sub rsp, 0x108
};
using ObjCaptureFn = uint64_t (*)(uint32_t);
ObjCaptureFn g_capture_original = nullptr;
int          g_capture_hook_id  = -1;
std::atomic<uint32_t> g_capture_calls{0}, g_capture_matched{0}, g_capture_written{0};

// When the builder pose hook last ran the probe (GetTickCount64 ms, 0 = never).
std::atomic<uint64_t> g_builder_probe_ms{0};

} // namespace

std::atomic<int32_t> g_wpn_obj_index{-1};   // halo scope: Gesture.cpp publishes it
std::atomic<int32_t> g_wpn_obj_ptr_datum{-1};
std::atomic<bool>  g_slide_node_valid{false};
std::atomic<float> g_slide_wx{0}, g_slide_wy{0}, g_slide_wz{0};
std::atomic<float> g_slide_fx{0}, g_slide_fy{0}, g_slide_fz{0};
std::atomic<float> g_slide_pull{0.0f};
std::atomic<uintptr_t> g_wpn_obj_ptr{0};
std::atomic<uintptr_t> g_slide_node_addr{0};

namespace {

// ---- WEAPON OBJECT NODES (dev: wpnnodedump, wpnnodepoke). SIM THREAD.
//
// The weapon actor mirrors a Blam object (BlamObjectSynchronizationComponent::BlamObjectIndex),
// and BlamMeshSynchronization carries that object's node matrices into the UE mesh -- which is
// where the slide's continuous pose comes from under the game's own animations. Resolve the
// object through the sim's table (same resolver the grenade work used) and find its node block
// by SHAPE: the longest run of 52-byte records with a sane scale, an orthonormal basis and a
// position within reach. Logged once per object. wpnnodepoke then displaces one node of that
// run along its Y (the barrel, by the authoring convention) so the slide can be named by eye,
// the same way the arms rig was walked.
void weapon_object_nodes_probe() {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    // The palette-node slide (superseded by the native parts, slideparthide 7) is the only thing
    // this per-frame memory walk served; with native parts it only froze the game every second
    // with its readback (2026-09-06). It runs for its own debug keys, or for a non-native slide.
    // The weapon OBJECT is resolved whatever the slide does: rounds_field reads it for the phantom,
    // the dry stop and the hidden reload, and returning here left the last weapon's pointer
    // published for as long as slidevr stayed off.
    const bool rack = service_active(SVC_RACK_AVAILABLE);   // a feature publishes the rack as available
    const bool slide_work = g_cfg.wpn_node_dump || g_cfg.wpn_node_poke >= 0 || rack;
    if (!slide_work) g_slide_node_valid.store(false, std::memory_order_relaxed);
    // Native slide mode still needs the weapon OBJECT (rounds_field reads it) and the legacy
    // valid flag the slide tick gates on; only the node scan and the per-second readback are
    // skipped (the gate that skipped everything killed the racks, 2026-09-06).
    const bool node_work = g_cfg.wpn_node_dump || g_cfg.wpn_node_poke >= 0 || (rack && g_cfg.slide_part_hide != 7);
    // A Blam datum is salt:index in one dword and reads NEGATIVE as int32 (measured 0xE27C000D
    // for the held pistol); only the all-ones NONE value means "no object".
    const int32_t idx = g_wpn_obj_index.load(std::memory_order_relaxed);
    if (idx == -1) { g_wpn_obj_ptr.store(0, std::memory_order_relaxed); g_wpn_obj_ptr_datum.store(-1, std::memory_order_relaxed); return; }
    const uintptr_t obj = resolve_object_by_datum((uint32_t)idx);
    static uintptr_t s_obj = 0;
    static int       s_off = -1, s_count = 0;
    if (obj == 0 || IsBadReadPtr((const void*)obj, 0x100)) { g_wpn_obj_ptr.store(0, std::memory_order_relaxed); g_wpn_obj_ptr_datum.store(-1, std::memory_order_relaxed); return; }
    g_wpn_obj_ptr.store(obj, std::memory_order_relaxed);
    g_wpn_obj_ptr_datum.store(idx, std::memory_order_relaxed);
    if (!node_work) { if (slide_work) g_slide_node_valid.store(true, std::memory_order_relaxed); return; }
    if (obj != s_obj) {
        s_obj = obj; s_off = -1; s_count = 0;
        constexpr int SPAN = 0x3000;
        if (IsBadReadPtr((const void*)obj, SPAN)) return;
        // STRICT shape: unit-length forward/left/up, mutually orthogonal, sane scale. The first
        // pass accepted a run whose "forward" was the player's world position (length ~131),
        // i.e. the record frame was 36 bytes off -- a loose test finds smooth data anywhere.
        auto unit = [](const NodeMatrix& v) {
            const float l = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
            return l > 0.95f && l < 1.05f;
        };
        auto ortho = [](const NodeMatrix& a, const NodeMatrix& b) {
            return std::fabs(a.x * b.x + a.y * b.y + a.z * b.z) < 0.1f;
        };
        // EVERY run of 3+ nodes, not the longest: the window runs past the small weapon object
        // into the next object in the table, and "longest" picked the biped's 64-node skeleton
        // at +0x2068 (positions at the player's own location). The weapon's block is the FIRST
        // short run; the target is the earliest run of 3..24 nodes, all runs logged.
        int best_off = -1, best_run = 0, runs_logged = 0;
        for (int off = 0; off + (int)sizeof(PaletteNode) <= SPAN; off += 4) {
            int run = 0;
            for (int k = 0; off + (int)sizeof(PaletteNode) * (k + 1) <= SPAN && k < 80; ++k) {
                auto* n = reinterpret_cast<const PaletteNode*>(obj + off + sizeof(PaletteNode) * k);
                if (!(n->scale > 0.2f && n->scale < 5.0f)) break;
                if (!unit(n->forward) || !unit(n->left) || !unit(n->up)) break;
                if (!ortho(n->forward, n->left) || !ortho(n->forward, n->up) || !ortho(n->left, n->up)) break;
                if (std::fabs(n->position.x) > 5000.0f || std::fabs(n->position.y) > 5000.0f ||
                    std::fabs(n->position.z) > 5000.0f) break;
                ++run;
            }
            if (run >= 3) {
                if (runs_logged < 8) {
                    ++runs_logged;
                    API::get()->log_info("[Halo-CampE-UEVR] WPNNODES: run at +0x%X, %d nodes", (unsigned)off, run);
                }
                if (best_off < 0 && run <= 24) { best_off = off; best_run = run; }
                off += (int)sizeof(PaletteNode) * run - 4;   // skip past this run
            }
        }
        s_off = best_off; s_count = best_run;
        API::get()->log_info("[Halo-CampE-UEVR] WPNNODES: object 0x%llX (index 0x%X): target run at +0x%X, %d nodes",
                             (unsigned long long)obj, (unsigned)idx, (unsigned)(best_off < 0 ? 0 : best_off), best_run);
        for (int k = 0; k < s_count && k < 24; ++k) {
            auto* n = reinterpret_cast<const PaletteNode*>(obj + s_off + sizeof(PaletteNode) * k);
            API::get()->log_info("[Halo-CampE-UEVR] WPNNODES   node %d: pos=(%.3f %.3f %.3f) fwd=(%.2f %.2f %.2f) scale=%.2f",
                                 k, n->position.x, n->position.y, n->position.z,
                                 n->forward.x, n->forward.y, n->forward.z, n->scale);
        }
    }
    // READBACK: did our last write survive to this tick, or did the game rebuild the node over
    // it? A write the game overwrites BEFORE the mesh sync reads it is invisible no matter how
    // correct it is -- "the poke worked before, now nothing" needs this to say which case.
    static NodeMatrix s_rb_last{};
    static bool       s_rb_have = false;
    const int rb_k = (g_cfg.wpn_node_poke >= 0 && g_cfg.wpn_node_poke < s_count) ? g_cfg.wpn_node_poke
                   : ((g_cfg.slide_node >= 0 && g_cfg.slide_node < s_count) ? g_cfg.slide_node : 0);
    PaletteNode* rb = (s_off >= 0 && s_count > 0)
                    ? reinterpret_cast<PaletteNode*>(obj + s_off + sizeof(PaletteNode) * rb_k) : nullptr;
    if (rb != nullptr && s_rb_have) {
        static long long s_rb_said = 0;
        const long long nowr = std::chrono::steady_clock::now().time_since_epoch().count();
        if (nowr - s_rb_said > std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::milliseconds(1000)).count()) {
            s_rb_said = nowr;
            const float dx = rb->position.x - s_rb_last.x, dy = rb->position.y - s_rb_last.y, dz = rb->position.z - s_rb_last.z;
            const float d = std::sqrt(dx * dx + dy * dy + dz * dz);
            API::get()->log_info("[Halo-CampE-UEVR] WPNNODES readback: node %d now (%.3f %.3f %.3f), we left (%.3f %.3f %.3f), moved %.4f -> %s",
                                 rb_k, rb->position.x, rb->position.y, rb->position.z,
                                 s_rb_last.x, s_rb_last.y, s_rb_last.z, d,
                                 d < 0.0005f ? "our value SURVIVED the tick (game did not rebuild it)" : "game REBUILT it before this call");
        }
    }
    // The value this call leaves behind is recorded at every exit below.
    auto rb_record = [&]() { if (rb != nullptr) { s_rb_last = rb->position; s_rb_have = true; } };

    // ---- COPY SCAN (wpnnodecopyscan): both our writes land and the slide still does not render,
    // so the mesh sync reads a COPY of these nodes -- the arms rig renders from capture banks,
    // not the live palette, and the weapon's parts must have the same arrangement. Once a
    // second, take the live block's node 0 and node 1 positions and search the sim's shared
    // capture region and the deeper object for a matching pair 52 bytes apart. Every hit is
    // logged; with the poke armed, the copies are poked too, so the one that renders names
    // itself on screen.
    if (g_cfg.wpn_node_copy_scan && s_off >= 0 && s_count >= 2) {
        static long long s_cs_at = 0;
        static uintptr_t s_hits[8]; static int s_nhits = 0;
        const long long nowc = std::chrono::steady_clock::now().time_since_epoch().count();
        if (nowc - s_cs_at > std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::milliseconds(1000)).count()) {
            s_cs_at = nowc; s_nhits = 0;
            auto* n0 = reinterpret_cast<const PaletteNode*>(obj + s_off);
            auto* n1 = reinterpret_cast<const PaletteNode*>(obj + s_off + sizeof(PaletteNode));
            auto near3 = [](const NodeMatrix& a, const NodeMatrix& b) {
                return std::fabs(a.x - b.x) < 1e-3f && std::fabs(a.y - b.y) < 1e-3f && std::fabs(a.z - b.z) < 1e-3f;
            };
            auto scan = [&](uintptr_t base, size_t span, const char* what) {
                for (size_t off = 0; off + sizeof(PaletteNode) * 2 <= span && s_nhits < 8; off += 4) {
                    if ((off & 0xFFF) == 0 && IsBadReadPtr((const void*)(base + off), 0x1000 + sizeof(PaletteNode) * 2)) { off += 0x1000 - 4; continue; }
                    auto* c0 = reinterpret_cast<const PaletteNode*>(base + off);
                    if (c0 == n0) continue;
                    if (!near3(c0->position, n0->position)) continue;
                    auto* c1 = reinterpret_cast<const PaletteNode*>(base + off + sizeof(PaletteNode));
                    if (!near3(c1->position, n1->position)) continue;
                    s_hits[s_nhits++] = base + off;
                    API::get()->log_info("[Halo-CampE-UEVR] WPNNODES copy: %s +0x%llX (abs 0x%llX)", what, (unsigned long long)off, (unsigned long long)(base + off));
                }
            };
            scan(obj + 0x3000, 0x8000, "object");
            const HMODULE simm = GetModuleHandleA("HaloSimulation_tag_release.dll");
            if (simm != nullptr && !IsBadReadPtr((const void*)((uintptr_t)simm + RVA_SHARED_CAPTURE_PTR), 8)) {
                auto* shared = *reinterpret_cast<uint8_t**>((uintptr_t)simm + RVA_SHARED_CAPTURE_PTR);
                if (shared != nullptr) scan((uintptr_t)shared, (size_t)CAPTURE_CTX_STRIDE * 2, "capture");
            }
            if (s_nhits == 0) API::get()->log_info("[Halo-CampE-UEVR] WPNNODES copy: none found this second");
        }
        // Poke the copies too, every call, so the rendering one shows itself.
        if (g_cfg.wpn_node_poke >= 0 && g_cfg.wpn_node_poke < s_count) {
            for (int h = 0; h < s_nhits; ++h) {
                auto* c = reinterpret_cast<PaletteNode*>(s_hits[h] + sizeof(PaletteNode) * g_cfg.wpn_node_poke);
                if (IsBadWritePtr(c, sizeof(PaletteNode))) continue;
                const float a = g_cfg.wpn_node_poke_amt;
                c->position.x += c->forward.x * a; c->position.y += c->forward.y * a; c->position.z += c->forward.z * a;
            }
        }
    }

    // ---- THE SLIDE: publish its node, then apply the hand's pull along -forward. Re-applied
    // every call against the game's fresh value (the sim rebuilds node matrices from the
    // animation each tick), so it cannot compound. Published BEFORE the pull is applied, so the
    // zone the hand reaches for is the slide's rest position, not wherever it was dragged to.
    if (s_off >= 0 && rack && g_cfg.slide_node >= 0 && g_cfg.slide_node < s_count) {
        auto* n = reinterpret_cast<PaletteNode*>(obj + s_off + sizeof(PaletteNode) * g_cfg.slide_node);
        g_slide_wx.store(n->position.x, std::memory_order_relaxed);
        g_slide_wy.store(n->position.y, std::memory_order_relaxed);
        g_slide_wz.store(n->position.z, std::memory_order_relaxed);
        g_slide_fx.store(n->forward.x, std::memory_order_relaxed);
        g_slide_fy.store(n->forward.y, std::memory_order_relaxed);
        g_slide_fz.store(n->forward.z, std::memory_order_relaxed);
        g_slide_node_valid.store(true, std::memory_order_relaxed);
        g_slide_node_addr.store((uintptr_t)n, std::memory_order_relaxed);
        const float pull = g_slide_pull.load(std::memory_order_relaxed);
        if (pull > 0.0f && std::isfinite(pull) && g_capture_hook_id < 0) {   // fallback only
            n->position.x -= n->forward.x * pull;
            n->position.y -= n->forward.y * pull;
            n->position.z -= n->forward.z * pull;
            // Proof the write happens, and where: "the slide does not move" with a published
            // pull needs this line to say whether the sim side ever ran it.
            static long long s_said_at = 0;
            const long long nowt = std::chrono::steady_clock::now().time_since_epoch().count();
            if (nowt - s_said_at > std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::milliseconds(500)).count()) {
                s_said_at = nowt;
                API::get()->log_info("[Halo-CampE-UEVR] SLIDE sim: node %d at +0x%X pull=%.4f fwd=(%.2f %.2f %.2f) pos=(%.3f %.3f %.3f)",
                                     g_cfg.slide_node, (unsigned)(s_off + (int)sizeof(PaletteNode) * g_cfg.slide_node), pull,
                                     n->forward.x, n->forward.y, n->forward.z, n->position.x, n->position.y, n->position.z);
            }
        }
    } else {
        g_slide_node_valid.store(false, std::memory_order_relaxed);
        g_slide_node_addr.store(0, std::memory_order_relaxed);
    }

    if (s_off < 0 || g_cfg.wpn_node_poke < 0 || g_cfg.wpn_node_poke >= s_count) { rb_record(); return; }
    // Poke ALONG THE NODE'S OWN FORWARD, in world: these matrices are world-space (position is
    // the player's location plus the part's offset). Same non-compounding property as the pull.
    auto* n = reinterpret_cast<PaletteNode*>(obj + s_off + sizeof(PaletteNode) * g_cfg.wpn_node_poke);
    const float a = g_cfg.wpn_node_poke_amt;
    n->position.x += n->forward.x * a;
    n->position.y += n->forward.y * a;
    n->position.z += n->forward.z * a;
    static long long s_pk_said = 0;
    const long long nowp = std::chrono::steady_clock::now().time_since_epoch().count();
    if (nowp - s_pk_said > std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::milliseconds(1000)).count()) {
        s_pk_said = nowp;
        API::get()->log_info("[Halo-CampE-UEVR] WPNNODES poke: node %d of %d at +0x%X amt=%.3f obj=0x%llX",
                             g_cfg.wpn_node_poke, s_count, (unsigned)(s_off + (int)sizeof(PaletteNode) * g_cfg.wpn_node_poke),
                             a, (unsigned long long)obj);
    }
    rb_record();
}

} // namespace

void weapon_object_probe_from_builder() {
    g_builder_probe_ms.store(GetTickCount64(), std::memory_order_relaxed);
    if (!service_active(SVC_WEAPON_OBJECT)) return;
    weapon_object_nodes_probe();
}

// Called from BlamDrive's orientation hook while the service is active. Rate-limited: that hook runs
// thousands of times a second. While the builder hook runs the probe (the palette weapon owns the
// pose), this one stands down.
void weapon_object_offhook_tick() {
    static uint32_t s_wo = 0;
    if ((++s_wo & 0x3Fu) != 0u) return;
    const uint64_t last = g_builder_probe_ms.load(std::memory_order_relaxed);
    if (last != 0 && GetTickCount64() - last < 250ull) return;
    weapon_object_nodes_probe();
}

void weapon_object_rack_reset() {
    g_slide_node_valid.store(false, std::memory_order_relaxed);
    g_slide_node_addr.store(0, std::memory_order_relaxed);
    g_slide_pull.store(0.0f, std::memory_order_relaxed);
}

void weapon_object_reset() {
    weapon_object_rack_reset();
    g_wpn_obj_index.store(-1, std::memory_order_relaxed);
    g_wpn_obj_ptr.store(0, std::memory_order_relaxed);
    g_wpn_obj_ptr_datum.store(-1, std::memory_order_relaxed);
}

// ---------------------------------------------------------------- the object-node capture hook
namespace {
uint64_t hooked_capture(uint32_t datum) {
    g_capture_calls.fetch_add(1, std::memory_order_relaxed);
    const int32_t want = g_wpn_obj_index.load(std::memory_order_relaxed);
    if (want != -1 && (datum & 0xFFFFu) == ((uint32_t)want & 0xFFFFu)) {
        g_capture_matched.fetch_add(1, std::memory_order_relaxed);
        const uintptr_t addr = g_slide_node_addr.load(std::memory_order_relaxed);
        const float     pull = g_slide_pull.load(std::memory_order_relaxed);
        if (addr != 0 && pull > 0.0f && std::isfinite(pull) && !IsBadWritePtr((void*)addr, sizeof(PaletteNode))) {
            auto* n = reinterpret_cast<PaletteNode*>(addr);
            // ONCE PER REBUILD. The sim rewrites the node every tick; a second capture of the same
            // object inside one tick (a parent's recursion) finds our value already there and
            // leaves it, so the pull never compounds.
            static NodeMatrix s_last{};
            if (!(n->position.x == s_last.x && n->position.y == s_last.y && n->position.z == s_last.z)) {
                n->position.x -= n->forward.x * pull;
                n->position.y -= n->forward.y * pull;
                n->position.z -= n->forward.z * pull;
                s_last = n->position;
                g_capture_written.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }
    return (g_capture_original != nullptr) ? g_capture_original(datum) : 0ull;
}
} // namespace

bool blam_capture_hook_active() { return g_capture_hook_id >= 0; }

void blam_capture_hook_tick() {
    const bool want = service_active(SVC_RACK_AVAILABLE) && g_cfg.slide_hook;
    if (!want) {
        if (g_capture_hook_id >= 0) {
            API::get()->param()->functions->unregister_inline_hook(g_capture_hook_id);
            g_capture_hook_id = -1;
            g_capture_original = nullptr;
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEHOOK: removed");
        }
        return;
    }
    if (g_capture_hook_id >= 0) {
        if (g_cfg.slide_log) {
            static ULONGLONG s_said = 0;
            const ULONGLONG now = GetTickCount64();
            if (now - s_said >= 5000ull) {
                s_said = now;
                API::get()->log_info("[Halo-CampE-UEVR] SLIDEHOOK: calls=%u matched=%u wrote=%u",
                                     g_capture_calls.load(std::memory_order_relaxed),
                                     g_capture_matched.load(std::memory_order_relaxed),
                                     g_capture_written.load(std::memory_order_relaxed));
            }
        }
        return;
    }
    static bool s_refused = false;
    if (s_refused) return;
    HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
    if (sim == nullptr) return;
    void* target = (void*)((uintptr_t)sim + RVA_OBJ_CAPTURE);
    if (!readable(target, sizeof(OBJ_CAPTURE_PROLOGUE))) {
        API::get()->log_info("[Halo-CampE-UEVR] SLIDEHOOK: dll+0x%llX unreadable -- not hooking", (unsigned long long)RVA_OBJ_CAPTURE);
        s_refused = true;
        return;
    }
    const uint8_t* got = (const uint8_t*)target;
    for (size_t i = 0; i < sizeof(OBJ_CAPTURE_PROLOGUE); ++i) {
        if (got[i] != OBJ_CAPTURE_PROLOGUE[i]) {
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEHOOK: PROLOGUE MISMATCH at dll+0x%llX byte %zu (expected 0x%02X, found 0x%02X) -- the game build moved this function. NOT hooking.",
                                 (unsigned long long)RVA_OBJ_CAPTURE, i, OBJ_CAPTURE_PROLOGUE[i], got[i]);
            s_refused = true;
            return;
        }
    }
    const int id = API::get()->param()->functions->register_inline_hook(target, (void*)&hooked_capture, (void**)&g_capture_original);
    if (id < 0 || g_capture_original == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] SLIDEHOOK: register_inline_hook FAILED (id=%d) on dll+0x%llX", id, (unsigned long long)RVA_OBJ_CAPTURE);
        s_refused = true;
        return;
    }
    g_capture_hook_id = id;
    API::get()->log_info("[Halo-CampE-UEVR] SLIDEHOOK: installed on dll+0x%llX id=%d", (unsigned long long)RVA_OBJ_CAPTURE, id);
}

} // namespace halo
