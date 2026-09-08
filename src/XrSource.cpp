// XrSource -- the widget render target, resolved down to an ID3D12Resource. Doctrine is in
// XrSource.hpp; read that first. This file is the walk, the filters and the validation.
//
// GAME THREAD ONLY. Everything here reads UE objects and calls UEVR's SDK, neither of which may
// happen on the submit thread. The only thing that crosses is one already-validated pointer, handed
// over through xrlayer_set_source().

#include "XrSource.hpp"

#include "Config.hpp"
#include "DevTools.hpp"
#include "Reticule.hpp"
#include "UeObject.hpp"    // class_name_of, for validating a component before dereferencing it
#include "XrLayer.hpp"
#include "addrcascade/AddressCascade.hpp"

#include "uevr/API.hpp"

#include <Windows.h>
#include <d3d12.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>

using uevr::API;

namespace halo {
namespace {

constexpr const char* TAG = "[Halo-CampE-UEVR] XRSRC:";

void logf(const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    _vsnprintf_s(buf, sizeof(buf), _TRUNCATE, fmt, ap);
    va_end(ap);
    API::get()->log_info("%s %s", TAG, buf);
}

// ============================================================================================
// PER-TICK RESOLVE SPLIT -- DEV ONLY
// ============================================================================================
//
// "the 16 ms/slot is get_native_resource" was a HYPOTHESIS. This measures it instead of asserting
// it: the per-slot resolve cost is split, per tick and summed over every serviced slot, into the
// reflected GetRenderTarget call, UEVR's get_native_resource() virtual call, the resource's GetDesc,
// and the bounded pointer walk that is everything else in resolve_latched. It also counts how many
// get_native_resource calls the cache SAVED. Reset at the top of xrsource_tick, printed in the
// throttled state line. QPC-based, the same currency Plugin.cpp's PerfScope uses.
#if HALO_VR_DEV
struct ResolveSplit {
    double   native_ms = 0.0;   // call_native_guarded = UEVR get_native_resource (the virtual call)
    double   desc_ms   = 0.0;   // desc_guarded        = ID3D12Resource::GetDesc + GetDevice
    double   walk_ms   = 0.0;   // resolve_latched minus the two guarded calls (the pointer walk)
    double   getrt_ms  = 0.0;   // component_render_target = reflected GetRenderTarget
    uint32_t native_n  = 0, desc_n = 0, getrt_n = 0;
    uint32_t cache_hit = 0, cache_miss = 0;   // native calls the cache saved vs. still had to make
};
ResolveSplit g_split;

double qpc_ms() {
    static const double f = [] {
        LARGE_INTEGER q{};
        return QueryPerformanceFrequency(&q) && q.QuadPart != 0 ? 1000.0 / (double)q.QuadPart : 0.0;
    }();
    return f;
}

// Adds its lifetime to one accumulator. Holds only POD, so it never collides with the __try in the
// guarded calls -- and it is used only AROUND those calls (in validate_native), never inside them.
struct SplitTimer {
    double*       acc;
    uint32_t*     cnt;   // may be null (walk time has no natural per-call count)
    LARGE_INTEGER t0{};
    SplitTimer(double* a, uint32_t* c) : acc(a), cnt(c) { QueryPerformanceCounter(&t0); }
    ~SplitTimer() {
        LARGE_INTEGER t1{};
        QueryPerformanceCounter(&t1);
        *acc += (double)(t1.QuadPart - t0.QuadPart) * qpc_ms();
        if (cnt != nullptr) ++*cnt;
    }
    SplitTimer(const SplitTimer&) = delete;
    SplitTimer& operator=(const SplitTimer&) = delete;
};
#endif   // HALO_VR_DEV

// ============================================================================================
// Memory classification
// ============================================================================================
//
// The whole safety argument rests on these three, so they are deliberately conservative and they
// answer questions a null check cannot.
//
// A VTABLE lives in a mapped image (MEM_IMAGE): it is part of a DLL or EXE that was loaded, not
// something allocated at runtime. A HEAP OBJECT lives in private committed memory (MEM_PRIVATE).
// Requiring both, in the right places, rejects the overwhelming majority of coincidental
// pointer-shaped values -- an interior pointer into an array of floats fails the vtable test, and a
// pointer to a string literal fails the private-memory test.

bool mem_is_image(const void* p) {
    if (p == nullptr) return false;
    MEMORY_BASIC_INFORMATION mbi{};
    if (VirtualQuery(p, &mbi, sizeof(mbi)) == 0) return false;
    if (mbi.State != MEM_COMMIT) return false;
    if (mbi.Type != MEM_IMAGE) return false;
    constexpr DWORD READABLE = PAGE_READONLY | PAGE_READWRITE | PAGE_WRITECOPY |
                               PAGE_EXECUTE_READ | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY;
    if ((mbi.Protect & READABLE) == 0) return false;
    return (mbi.Protect & PAGE_GUARD) == 0;
}

bool mem_is_private(const void* p, size_t need) {
    if (p == nullptr || need == 0) return false;
    MEMORY_BASIC_INFORMATION mbi{};
    if (VirtualQuery(p, &mbi, sizeof(mbi)) == 0) return false;
    if (mbi.State != MEM_COMMIT) return false;
    if (mbi.Type != MEM_PRIVATE) return false;
    return addrcascade::readable_bytes(p, need) >= need;
}

// A pointer that behaves like a polymorphic C++ object allocated on the heap: 8-byte aligned, in
// private committed memory with at least `need` readable bytes, and whose first qword points at a
// vtable in a mapped image with at least 16 readable slots.
//
// The 16-slot requirement is not decoration. UEVR's get_native_resource() walks vtable indices 2..15
// looking for the one that returns a D3D resource; handing it an object whose "vtable" is a mapped
// image address with only a couple of readable qwords after it is precisely how that walk turns
// into a fault.
// `need` is the MINIMUM that must be readable, not the window a caller intends to scan. Those are
// different numbers and conflating them silently loses real answers: an engine object can sit near
// the end of its allocation region, so demanding a whole 0x200-byte window be readable would reject
// the correct FTextureResource on some launches and not others. Callers scan up to
// scan_limit(p, window) instead.
bool looks_like_object(const void* p, size_t need) {
    if (p == nullptr) return false;
    if ((reinterpret_cast<uintptr_t>(p) & 7u) != 0u) return false;
    if (!mem_is_private(p, need)) return false;
    const void* vt = *reinterpret_cast<void* const*>(p);
    if (!mem_is_image(vt)) return false;
    if (addrcascade::readable_bytes(vt, sizeof(void*) * 16) < sizeof(void*) * 16) return false;
    // Every entry we might reach must itself be code.
    for (int i = 0; i < 16; ++i) {
        const void* fn = reinterpret_cast<void* const*>(vt)[i];
        if (fn == nullptr) break;              // short vtable is fine; a BAD entry is not
        if (!mem_is_image(fn)) return false;
    }
    return true;
}

// How far into an object it is actually safe to read, capped at the window we care about.
size_t scan_limit(const void* p, size_t window) {
    return addrcascade::readable_bytes(p, window);
}

// ============================================================================================
// The guarded calls
// ============================================================================================
//
// Both of these are SEH-guarded and hold only POD locals, which is what lets __try live in them at
// all. A guard is NOT a licence to call on a bad pointer -- an access violation caught here can
// still have run arbitrary code first. It is the last net under the filters above, not a substitute
// for them.

void* call_native_guarded(void* rhi) {
    auto* p = API::get()->param();
    if (p == nullptr || p->sdk == nullptr || p->sdk->frhitexture2d == nullptr ||
        p->sdk->frhitexture2d->get_native_resource == nullptr) {
        return nullptr;
    }
    __try {
        return p->sdk->frhitexture2d->get_native_resource((UEVR_FRHITexture2DHandle)rhi);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return nullptr;
    }
}

bool desc_guarded(void* native, D3D12_RESOURCE_DESC* out, void** out_device) {
    if (native == nullptr || out == nullptr) return false;
    __try {
        ID3D12Resource* r = (ID3D12Resource*)native;
        *out = r->GetDesc();
        if (out_device != nullptr) {
            ID3D12Device* dev = nullptr;
            *out_device = SUCCEEDED(r->GetDevice(IID_PPV_ARGS(&dev))) ? (void*)dev : nullptr;
            if (dev != nullptr) dev->Release();   // GetDevice AddRefs; we only want its identity
        }
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

// ============================================================================================
// The latched chain
// ============================================================================================
//
// ADDR-HYGIENE: guarded -- these three offsets are MEASURED AT RUNTIME by the dev probe, never
// written down as constants. They are re-validated on EVERY resolve (the FRHITexture must still
// read back the widget's exact draw size in both axes, and the resulting ID3D12Resource's own
// GetDesc must agree with it on the same ID3D12Device UEVR reports). A patch that moves any of
// them makes the validation fail, which drops the source and leaves the generated ring drawing --
// it cannot make us write through a stale address. There is deliberately no compiled-in fallback
// value: on a shipping build with no measurement, this feature is simply unavailable, which is the
// fail-closed answer.
struct Chain {
    int32_t off_res = -1;   // UTextureRenderTarget2D -> FTextureResource*
    int32_t off_rhi = -1;   // FTextureResource       -> FRHITexture*
    int32_t off_ext = -1;   // FRHITexture            -> int32 SizeX (SizeY at +4)
    bool valid() const { return off_res >= 0 && off_rhi >= 0 && off_ext >= 0; }
};
Chain g_chain;

// What we resolved, and what it belongs to -- PER SLOT. The RT object identity matters: a mission
// transition re-hosts the crosshair on a NEW UWidgetComponent with a NEW render target, and
// continuing to hand the compositor the old ID3D12Resource would be presenting freed memory every
// frame. A navpoint marker does the same thing far more often, because the pool re-hosts a slot
// whenever the navpoint occupying it changes type.
//
// ONE Chain (above) SERVES ALL OF THEM. That is the load-bearing claim of the multi-component
// design: off_res/off_rhi/off_ext are offsets of members of UTexture and FTexture, so they are
// properties of the CLASS, not of any instance. It is very probably true and it is still an
// assumption, so a dev build measures a second component and compares -- see g_crosscheck.
struct Target {
    void*    comp         = nullptr;   // UWidgetComponent -- IS dereferenced, see comp_index
    // ITS SLOT IN THE GLOBAL UOBJECT ARRAY, resolved once when the component is adopted.
    //
    // MEASURED 2026-09-04, and this is the fault the whole crash hunt was chasing:
    //   TICK FAULT: 0xC0000005 at UEVRBackend.dll+0x6283EC reading 0x1E31C0908,
    //               last lane entered 'xrsource_tick'
    // We handed UEVR a UWidgetComponent the level teardown had already freed, and UEVR faulted
    // walking it inside call_function. The address is a live-looking heap pointer, not null, which
    // is exactly why the first guard here (IsBadReadPtr + a reflected class-name check) did not
    // help: a freed UObject whose memory has been REUSED passes both. Its own comment said so.
    //
    // Only ARRAY LIVENESS can tell a recycled address from a live object, which is what
    // TrackedObject::get() does everywhere else in this codebase. Resolving the index costs one
    // walk of the object array, so it is done ONLY when the component changes (a handful of times
    // per level); verifying it afterwards is a single indexed compare per slot per tick.
    int32_t  comp_index   = -1;
    bool     is_rt        = false;     // comp IS the render target (pane); skip the widget hop
    int      want         = 0;         // its SetDrawSize edge -- the ValueAgreement number
    void*    native       = nullptr;
    void*    native_rt    = nullptr;   // the UObject the resource was resolved from
    void*    native_rhi   = nullptr;
    int      dim          = 0;
    uint32_t fmt          = 0;
    // WHICH POINTER THE COMPOSITOR LAYER IS ACTUALLY HOLDING for this slot, not "did we ever hand
    // one over". It was a bool once and the bool was wrong twice: it latched even when
    // xrlayer_set_source() REFUSED, so a session whose swapchain came up wrong never offered again;
    // and recording only "yes" meant a re-resolve to a DIFFERENT resource was never offered either.
    void*    fed          = nullptr;
    uint32_t next_resolve = 0;
    uint32_t next_offer   = 0;
};
Target g_t[XRLAYER_SLOTS];

// What Plugin.cpp asked for, between ticks. Consumed at the top of xrsource_tick.
// `is_rt` says the object in `comp` IS the UTextureRenderTarget2D, not a UWidgetComponent that owns
// one -- so the resolve enters one rung lower and skips the reflected GetRenderTarget call. Nothing
// below that rung is widget-shaped: the offsets walk UTextureRenderTarget2D -> FTextureResource ->
// FRHITexture regardless of who owns the target, which is exactly what g_probe_rt has always proved
// by walking a render target this module creates itself.
struct SlotReq { void* comp = nullptr; int want = 0; bool is_rt = false; };
SlotReq g_req[XRLAYER_SLOTS];

// THE LAST COMPONENT POINTER REFUSED FOR EACH SLOT.
//
// The refusal below sets t.comp = nullptr but CANNOT clear g_req -- the request belongs to the
// publisher, and g_req persists between ticks by design. So a publisher that keeps offering the
// same freed pointer (which is exactly what a level teardown produces: Plugin.cpp stops resolving
// a new one, and the last value sits there) makes `g_req[s].comp != t.comp` true on EVERY tick,
// and each one re-runs the ~296k object-array walk and logs again. A per-tick full-array walk is
// the precise cost this project has already been bitten by (the find_uobject miss, and the
// comps=9 report at 87-197 ms per tick), so the refusal must be remembered, not just acted on.
//
// Remembering it costs one pointer compare and changes no behaviour: a genuinely NEW component
// differs from the rejected value and resolves exactly as before.
void* g_slot_rejected[XRLAYER_SLOTS] = {};

// 0 = not attempted, 1 = a second component's chain AGREED, -1 = it disagreed.
int g_crosscheck = 0;

char g_status[192] = "idle";

void set_status(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    _vsnprintf_s(g_status, sizeof(g_status), _TRUNCATE, fmt, ap);
    va_end(ap);
}

// A component's render target, through reflection. Safe: GetRenderTarget is a real UFunction and
// Reticule.cpp already calls it on the reticule's component.
//
// TAKES THE COMPONENT rather than reaching for the reticule's. Every hop below this one is a member
// of UTexture/FTexture and therefore the same for any render target, which is what makes ONE
// measured chain serve nine components -- see xrsource_chain_crosscheck_state() for the proof
// rather than the assertion.
// Is this pointer still the object we adopted, in the slot we adopted it from?
//
// THE ONLY CHECK THAT CATCHES A RECYCLED ADDRESS. A freed UObject stays mapped and its memory is
// handed to the next allocation, so IsBadReadPtr passes and the class name can even still read
// "WidgetComponent". The array slot is the identity: if it no longer holds our pointer, the object
// we adopted is gone whatever now lives at that address.
bool comp_still_live(void* comp, int32_t index) {
    if (comp == nullptr || index < 0) return false;
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return true;                       // cannot tell -- fail open, as before
    if (index >= arr->get_object_count()) return false;
    return arr->get_object(index) == comp;
}

API::UObject* component_render_target(API::UObject* wc) {
    if (wc == nullptr) return nullptr;

    // ---- VALIDATE BEFORE DEREFERENCING. THIS POINTER IS HELD ACROSS TICKS. ------------------
    //
    // Target::comp is documented as "identity only, never dereferenced here" -- and that stopped
    // being true the moment this function started taking the component as an argument. It IS
    // dereferenced, by call_function, one frame or one LEVEL LOAD after whoever published it.
    //
    // MEASURED 2026-09-04, and it predates that session: on a level transition
    // (mission -> menu -> new mission) UEVR logs "Exception occurred in on_pre_engine_tick" every
    // tick and never recovers -- 13,879 of them in one session. The line 2 ms before the first one
    // is ours: "XRSRC: slot 0 component changed (...->0000000000000000)". A component going null is
    // the transition; the slot that went NULL is handled by service_slot's own null check, so the
    // fault is a SIBLING slot still holding a non-null pointer to a widget the level change
    // destroyed. From the player's seat this reads as "the controls broke after the loading
    // screen", because the tick body dies while BLAMCTL/DIRECT keep running on the XInput hook.
    //
    // This is precisely the standing rule in CLAUDE.md that actors here are pooled and recycled and
    // must never be held across frames. The same guard navw_entry_widget already uses for map
    // entries: a readability check, then a reflected CLASS check, and a null return that the caller
    // already treats as "no render target" and resets the slot for.
    //
    // HONEST LIMIT: a freed UObject whose memory has been REUSED by another component will pass
    // both checks. This does not make the pointer provably valid -- it removes the unmapped and
    // garbage cases, which is what is actually faulting here, and it fails to a path the caller
    // already handles rather than to an access violation.
    if (IsBadReadPtr(wc, 0x30)) return nullptr;
    if (class_name_of(wc).find(L"Component") == std::wstring::npos) return nullptr;

    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    wc->call_function(L"GetRenderTarget", p);
    return *reinterpret_cast<API::UObject**>(p);
}

#if HALO_VR_DEV
// A render target WE make, purely to have something to measure against.
//
// THE OFFSETS ARE STRUCTURAL, THE OBJECT IS NOT. Every UTextureRenderTarget2D on this build reaches
// its FTextureResource and its FRHITexture through the same members, so a target we created
// ourselves teaches us the same three offsets the widget's would -- and it is available when the
// widget's is not, which turned out to matter: on the headless OpenVR lane the aim stack idles
// ("aim pose invalid"), reticule_widget_ensure is never called, and the widget reticule therefore
// never exists. A probe that can only run when the whole aim stack is healthy is a probe that
// cannot be run on the lane it was supposed to be safe on.
//
// It is measured at aim_widget_draw so the size filter keys on the same distinctive number either
// way, and it never becomes the presented source -- only the widget's own target does.
TrackedObject g_probe_rt;

API::UObject* probe_render_target(int dim) {
    if (auto* p = g_probe_rt.get_checked(L"TextureRenderTarget2D")) return p;
    // Transparent black: nothing samples it, only its plumbing is of interest.
    auto* rt = make_color_rt(0.0f, 0.0f, 0.0f, 0.0f, dim);
    if (rt == nullptr) return nullptr;
    g_probe_rt.set(rt);
    logf("created a %dx%d probe render target %p to measure the chain against (the widget's own "
         "target is not up).", dim, dim, (void*)rt);
    return g_probe_rt.get_checked(L"TextureRenderTarget2D");
}
#endif

// Does the memory around a size match read like a real FRHITextureDesc?
//
// MEASURED, NOT ASSUMED (2026-08-23, mode-1 walk against a 324x324 render target). The winning
// candidate put the desc at FRHITexture+0x20 -- the standard UE 5.5/5.6 shipping offset -- giving
// extent at +0x44/+0x48 and mips/samples/dimension/format at +0x50..+0x53, reading
// 1 / 1 / 0 (Texture2D) / 2 (PF_B8G8R8A8). That is exactly what a UWidgetComponent's render target
// should be, and it is why this gate is written the way it is.
//
// (UEVR's own SDK carries a comment saying this title's FD3D12Texture matches none of the known
// desc offsets. That is about the SCENE target it bootstraps from, and it is not true of a render
// target reached this way. Worth writing down: the two statements look contradictory and are not.)
//
// The gate matters because it decides how many VIRTUAL CALLS mode 2 makes. On the measured run it
// takes eight extent matches down to one distinct pointer: the two junk candidates carried mips=0
// and format=0, neither of which any real texture can have.
bool desc_plausible(const uint8_t* q) {
    const uint8_t mips = *(q + 0x0C), samples = *(q + 0x0D);
    const uint8_t dim = *(q + 0x0E), fmt = *(q + 0x0F);
    if (mips == 0 || mips > 32) return false;
    if (!(samples == 1 || samples == 2 || samples == 4 || samples == 8 || samples == 16)) return false;
    if (dim > 8) return false;
    if (fmt == 0 || fmt > 128) return false;
    return true;
}

// Turn a candidate FRHITexture into a validated ID3D12Resource, or nullptr with a reason logged.
//
// THIS is the ValueAgreement rung. The dimensions are known independently (they are what we asked
// UE for with SetDrawSize), so a resource that reports them is agreeing with a number we did not
// read out of the same place we read the pointer.
void* validate_native(void* rhi, int want, bool loud, int* out_dim, uint32_t* out_fmt) {
    void* native;
    {
#if HALO_VR_DEV
        SplitTimer _t(&g_split.native_ms, &g_split.native_n);   // the virtual call, timed on its own
#endif
        native = call_native_guarded(rhi);
    }
    if (native == nullptr) {
        if (loud) logf("  rhi %p -> get_native_resource returned null", rhi);
        return nullptr;
    }

    D3D12_RESOURCE_DESC d{};
    void* dev = nullptr;
    bool desc_ok;
    {
#if HALO_VR_DEV
        SplitTimer _t(&g_split.desc_ms, &g_split.desc_n);       // GetDesc + GetDevice, timed on its own
#endif
        desc_ok = desc_guarded(native, &d, &dev);
    }
    if (!desc_ok) {
        if (loud) logf("  rhi %p -> native %p but GetDesc faulted -- REJECTED", rhi, native);
        return nullptr;
    }

    auto* p = API::get()->param();
    void* uevr_dev = (p != nullptr && p->renderer != nullptr) ? p->renderer->device : nullptr;

    const bool dim_ok = (d.Dimension == D3D12_RESOURCE_DIMENSION_TEXTURE2D);
    const bool size_ok = ((int)d.Width == want && (int)d.Height == want);
    const bool dev_ok = (uevr_dev == nullptr || dev == nullptr || dev == uevr_dev);

    if (loud) {
        logf("  rhi %p -> native %p desc dim=%d %llux%u fmt=%u mips=%u samples=%u dev=%p (UEVR %p)",
             rhi, native, (int)d.Dimension, (unsigned long long)d.Width, (unsigned)d.Height,
             (unsigned)d.Format, (unsigned)d.MipLevels, (unsigned)d.SampleDesc.Count, dev, uevr_dev);
    }

    if (!dim_ok || !size_ok || !dev_ok) {
        if (loud) {
            logf("  REJECTED: %s%s%s", dim_ok ? "" : "not a 2D texture; ",
                 size_ok ? "" : "dimensions disagree with aimwidgetdraw; ",
                 dev_ok ? "" : "different ID3D12Device than UEVR reports; ");
        }
        return nullptr;
    }

    if (out_dim != nullptr) *out_dim = want;
    if (out_fmt != nullptr) *out_fmt = (uint32_t)d.Format;
    return native;
}

// Walk the latched chain and re-validate it end to end. No searching, no assumptions: every hop is
// re-checked, so this is safe in a shipping build once a dev build has measured the offsets.
//
// THE PER-TICK NATIVE CACHE lives here (xrlayersrccache, default on). `cached_native`/`cached_rhi`/
// `cached_fmt` are this slot's result from a previous tick. The cheap pointer walk below runs EVERY
// tick regardless; the EXPENSIVE last hop (get_native_resource + GetDesc, ~one frame of render-thread
// sync per slot) is skipped when the walk still lands on the SAME live FRHITexture the cache was
// derived from and that FRHITexture still reads back want x want with a plausible desc. It is re-run
// only on a cache miss: no cache yet, or the FRHITexture pointer changed (a re-host).
//
// WHY THIS KEEPS THE 2026-08-23 CRASH-FIX INVARIANT (read XrSource.hpp's CACHE section, and the
// g_owned commentary in XrLayer.cpp, before touching it):
//   * The cheap walk still re-classifies rt -> res -> rhi as heap objects with mapped-image vtables
//     and re-reads the FRHITexture's own extent EVERY tick. A freed/re-hosted chain fails it and the
//     slot is dropped, exactly as before.
//   * service_slot() still compares the render-target UObject identity (rt != t.native_rt) and
//     xrsource_tick() still compares the component identity every tick; either change drops the slot
//     and clears t.native/t.native_rhi, so the cache cannot outlive a re-host.
//   * An FRHITexture owns a fixed ID3D12Resource for its lifetime; UE allocates a NEW FRHITexture
//     (new pointer) when it recreates the GPU resource. So caching keyed on the FRHITexture pointer
//     is equivalent to re-deriving via get_native_resource -- minus the render-thread sync.
//   * The dangerous barrier (xrlayer_capture_record) is on the GAME THREAD; during a level load the
//     game thread is parked in LoadMap and this function is not called at all, so no capture is
//     issued against a resource whose GPU backing is being torn down. The cache does not widen that.
void* resolve_latched(API::UObject* rt, int want, void* cached_native, void* cached_rhi,
                      uint32_t cached_fmt, int* out_dim, uint32_t* out_fmt, void** out_rhi) {
    if (!g_chain.valid() || rt == nullptr) return nullptr;

    // WHERE THE PER-TICK COST ACTUALLY IS (measured 2026-08-24, live, dev split instrument): NOT
    // get_native_resource/GetDesc -- those are ~0.00 ms -- but this walk, ~14.5 ms per slot and
    // GROWING with the session. The cause is looks_like_object(): it classifies each hop with
    // VirtualQuery (via mem_is_private/mem_is_image and a 16-slot vtable sweep = ~32 VirtualQuery
    // across res+rhi), and VirtualQuery walks the kernel VAD tree, which on this game (~294k UObjects,
    // a huge, growing address space) costs hundreds of microseconds EACH. ~42 VirtualQuery/slot x
    // ~0.35 ms = the measured 14.5 ms, rising as the VAD tree grows. Six markers = 6x = the ~100 ms
    // xrsource_tick the headset reported.
    //
    // THE VTABLE CLASSIFICATION EXISTS ONLY TO MAKE get_native_resource'S VIRTUAL CALL SAFE. On a
    // CACHE HIT we do NOT call get_native_resource (we return the cached ID3D12Resource), so that
    // classification is unnecessary on the hit path -- and it is the whole cost. So the hit path
    // reaches rhi with cheap readable_bytes guards only (1 VirtualQuery each, ~4 total) and re-uses
    // the cached native once the chain still leads to the SAME FRHITexture reading back want x want.
    //
    // SAFETY (the 2026-08-23 invariant is preserved -- read XrSource.hpp's CACHE section):
    //   * A cache hit CANNOT happen after a re-host: service_slot drops the slot (clears t.native)
    //     the moment the render-target UObject identity changes, and xrsource_tick the moment the
    //     component changes -- both every tick, both cheap. During a level load the game thread is
    //     parked in LoadMap and this function is not called at all.
    //   * Every dereference on the hit path is still guarded by readable_bytes (no fault on freed
    //     memory), and rhi == cached_rhi + extent == want + desc_plausible still prove it is the same
    //     texture. What is dropped is only the vtable sweep, which guards a virtual call we do not make.
    //   * xrlayersrccache=0 forces the FULL classified walk every tick (the pre-fix behaviour): a safe
    //     fallback if the lightweight path is ever doubted, and the A/B control for the measurement.
    // A CACHE MISS (first resolve, re-host, FRHITexture pointer changed) takes the FULL path below,
    // exactly as before: classify res+rhi, then get_native_resource + GetDesc.
    const bool cache_on = (cached_native != nullptr && g_cfg.xr_layer_src_cache);
    bool fast_hit = false;
    void* rhi = nullptr;
    {
#if HALO_VR_DEV
        SplitTimer _t(&g_split.walk_ms, nullptr);   // the pointer walk, timed on its own
#endif
        auto* base = reinterpret_cast<const uint8_t*>(rt);
        if (addrcascade::readable_bytes(base, (size_t)g_chain.off_res + 8) < (size_t)g_chain.off_res + 8) {
            return nullptr;
        }
        void* res = *reinterpret_cast<void* const*>(base + g_chain.off_res);

        // ---- LIGHTWEIGHT HIT PATH: reach rhi with guarded reads, no vtable classification ---------
        if (cache_on) {
            // res only needs to be readable far enough to read the FRHITexture pointer out of it.
            if (res != nullptr &&
                addrcascade::readable_bytes(res, (size_t)g_chain.off_rhi + 8) >= (size_t)g_chain.off_rhi + 8) {
                void* rhi2 = *reinterpret_cast<void* const*>(reinterpret_cast<uint8_t*>(res) + g_chain.off_rhi);
                if (rhi2 == cached_rhi &&
                    addrcascade::readable_bytes(rhi2, (size_t)g_chain.off_ext + 16) >= (size_t)g_chain.off_ext + 16) {
                    auto* rb = reinterpret_cast<const uint8_t*>(rhi2);
                    const int32_t x = *reinterpret_cast<const int32_t*>(rb + g_chain.off_ext);
                    const int32_t y = *reinterpret_cast<const int32_t*>(rb + g_chain.off_ext + 4);
                    if (x == want && y == want && desc_plausible(rb + g_chain.off_ext)) {
                        rhi = rhi2;
                        fast_hit = true;
                    }
                }
            }
        }

        // ---- FULL PATH: classify each hop (safe for the virtual call), then the extent+desc gates --
        if (!fast_hit) {
            if (!looks_like_object(res, (size_t)g_chain.off_rhi + 8)) return nullptr;
            rhi = *reinterpret_cast<void* const*>(reinterpret_cast<uint8_t*>(res) + g_chain.off_rhi);
            if (!looks_like_object(rhi, (size_t)g_chain.off_ext + 8)) return nullptr;

            // THE DIMENSION AND DESCRIPTOR GATES, ON EVERY (missed) RESOLVE -- not once at discovery.
            // If a patch moves any hop, what we land on will not read back the widget's exact draw
            // size in both axes with a sane mip/sample/dimension/format -- and we hand nothing to the
            // compositor. Deliberately NOT "does the value look finite": it is agreement with a number
            // we chose ourselves when we called SetDrawSize.
            auto* rhi_b = reinterpret_cast<const uint8_t*>(rhi);
            if (scan_limit(rhi_b, (size_t)g_chain.off_ext + 16) < (size_t)g_chain.off_ext + 16) return nullptr;
            const int32_t x = *reinterpret_cast<const int32_t*>(rhi_b + g_chain.off_ext);
            const int32_t y = *reinterpret_cast<const int32_t*>(rhi_b + g_chain.off_ext + 4);
            if (x != want || y != want) return nullptr;
            if (!desc_plausible(rhi_b + g_chain.off_ext)) return nullptr;
        }
    }

    if (out_rhi != nullptr) *out_rhi = rhi;

    if (fast_hit) {
        // CACHE HIT -- the chain still leads to the same FRHITexture, still the right size. Re-use the
        // cached ID3D12Resource; get_native_resource()/GetDesc() would return exactly what we hold.
#if HALO_VR_DEV
        ++g_split.cache_hit;
#endif
        if (out_dim != nullptr) *out_dim = want;
        if (out_fmt != nullptr) *out_fmt = cached_fmt;
        return cached_native;
    }

    // CACHE MISS -- derive and validate the native fresh (get_native_resource + GetDesc).
#if HALO_VR_DEV
    ++g_split.cache_miss;
#endif
    return validate_native(rhi, want, /*loud=*/false, out_dim, out_fmt);
}

// ============================================================================================
// The discovery walk -- DEV ONLY
// ============================================================================================
//
// Bounded on all three axes and it dereferences nothing it has not classified first. Mode 1 logs
// and stops; mode 2 additionally attempts the virtual call on what it found. That split is the
// point: the walk itself cannot crash, and turning the crash-capable half on is a separate,
// deliberate act by an operator who has already read the candidate list.

#if HALO_VR_DEV

// How far into each object to look. Generous enough to cover an FTextureRenderTargetResource (which
// carries two base vtables and a fair amount of state before TextureRHI) without being a sweep.
constexpr int32_t RT_WINDOW  = 0x400;   // UTextureRenderTarget2D
constexpr int32_t RES_WINDOW = 0x200;   // FTextureResource
constexpr int32_t EXT_WINDOW = 0x140;   // FRHITexture

// Attempts are capped and de-duplicated by FRHITexture pointer.
//
// THE SAME TEXTURE IS REACHED SEVERAL WAYS ON PURPOSE. UTexture keeps both PrivateResource and
// PrivateResourceRenderThread, and they normally hold the SAME FTextureResource; an FRHITexture is
// likewise referenced from more than one member. Without de-duplication the walk would call
// get_native_resource() on the same pointer a dozen times and then declare its own duplicates
// "ambiguous" -- an ambiguity check that fires on corroboration is worse than none.
constexpr int MAX_ATTEMPTS = 12;


// `out` is where a successful walk LATCHES its chain. Parameterised rather than writing g_chain
// directly so the cross-check can run a second walk into a scratch Chain and compare, instead of
// overwriting the one nine slots are already resolving through.
void probe(API::UObject* rt, int want, int mode, Chain* out) {
    logf("PROBE mode %d: rt=%p looking for a %dx%d texture.", mode, (void*)rt, want, want);
    logf("  NOTE: %d is the value of aimwidgetdraw. If it is a round number you will get "
         "coincidences -- set aimwidgetdraw to something unusual (300, 324) and re-probe.", want);

    auto* rt_base = reinterpret_cast<const uint8_t*>(rt);
    const size_t rt_lim = scan_limit(rt_base, RT_WINDOW);
    if (rt_lim < 0x40) {
        logf("  render target object is only %zu bytes readable -- nothing to walk", rt_lim);
        return;
    }

    int candidates = 0;
    int accepted = 0;
    int attempts = 0;

    void* tried[MAX_ATTEMPTS] = {};
    int   tried_n = 0;

    void* first_native = nullptr;

    // 0x28 skips the UObject header (vtable, flags, index, outer, name, class), none of which can
    // be an FTextureResource pointer.
    for (int32_t o1 = 0x28; (size_t)o1 + 8 <= rt_lim; o1 += 8) {
        void* res = *reinterpret_cast<void* const*>(rt_base + o1);
        if (!looks_like_object(res, 0x40)) continue;
        const size_t res_lim = scan_limit(res, RES_WINDOW);

        for (int32_t o2 = 0x08; (size_t)o2 + 8 <= res_lim; o2 += 8) {
            void* rhi = *reinterpret_cast<void* const*>(reinterpret_cast<uint8_t*>(res) + o2);
            if (rhi == res || rhi == (void*)rt) continue;
            if (!looks_like_object(rhi, 0x40)) continue;
            const size_t ext_lim = scan_limit(rhi, EXT_WINDOW);

            for (int32_t oe = 0x08; (size_t)oe + 16 <= ext_lim; oe += 4) {
                auto* q = reinterpret_cast<const uint8_t*>(rhi) + oe;
                const int32_t x = *reinterpret_cast<const int32_t*>(q);
                const int32_t y = *reinterpret_cast<const int32_t*>(q + 4);
                if (x != want || y != want) continue;

                // Report the bytes that WOULD be FRHITextureDesc's mips/samples/dimension/format if
                // this game laid the desc out the standard way. UEVR's SDK records that it does not
                // (StereoStuff.cpp gates Halo/Meteorite out of the desc-validated path), so these
                // are evidence to read, never a gate to pass.
                const uint8_t mips = *(q + 0x0C), samples = *(q + 0x0D);
                const uint8_t dim = *(q + 0x0E), fmt = *(q + 0x0F);
                const bool plausible = desc_plausible(q);

                ++candidates;
                logf("CANDIDATE #%d%s  rt+0x%X -> res %p  res+0x%X -> rhi %p  rhi+0x%X = %dx%d "
                     "(mips=%u samples=%u dim=%u fmt=%u)",
                     candidates, plausible ? "" : " [desc implausible]",
                     (unsigned)o1, res, (unsigned)o2, rhi, (unsigned)oe, x, y,
                     (unsigned)mips, (unsigned)samples, (unsigned)dim, (unsigned)fmt);

                if (mode < 2) continue;   // mode 1 LOGS AND STOPS. No virtual call.

                // EVERY extent match is LOGGED; only a plausible one is CALLED. Filtering the log
                // as well would hide the evidence that says what the layout actually is -- which is
                // the only thing that made this gate writable in the first place.
                if (!plausible) continue;

                bool seen = false;
                for (int i = 0; i < tried_n; ++i) if (tried[i] == rhi) { seen = true; break; }
                if (seen) continue;
                if (attempts >= MAX_ATTEMPTS) {
                    logf("  attempt cap (%d) reached -- stopping rather than calling further.",
                         MAX_ATTEMPTS);
                    goto done;
                }
                tried[tried_n++] = rhi;
                ++attempts;

                int dim_out = 0;
                uint32_t fmt_out = 0;
                void* native = validate_native(rhi, want, /*loud=*/true, &dim_out, &fmt_out);
                if (native == nullptr) continue;

                ++accepted;
                logf("ACCEPTED: chain rt+0x%X res+0x%X rhi+0x%X -> ID3D12Resource %p, %dx%d, "
                     "DXGI format %u. Dimensions agree with aimwidgetdraw on UEVR's own device.",
                     (unsigned)o1, (unsigned)o2, (unsigned)oe, native, dim_out, dim_out, fmt_out);

                if (first_native == nullptr) {
                    first_native = native;
                    out->off_res = o1;
                    out->off_rhi = o2;
                    out->off_ext = oe;
                } else if (native == first_native) {
                    // NOT ambiguity -- CORROBORATION. Two independent walks reaching the same
                    // ID3D12Resource is the strongest evidence this probe can produce, and it is
                    // exactly what UTexture's paired PrivateResource / PrivateResourceRenderThread
                    // members should cause. Keep the first chain.
                    logf("  (same resource reached by a second path -- corroboration, chain kept)");
                } else {
                    // Two walks, two DIFFERENT resources. At least one is not the widget's render
                    // target and nothing here can say which. Refuse both, the same way
                    // scan_signature refuses an ambiguous match.
                    logf("AMBIGUOUS: a second chain validated a DIFFERENT resource (%p vs %p). "
                         "Refusing to latch either -- re-probe with a more distinctive "
                         "aimwidgetdraw.", native, first_native);
                    *out = Chain{};
                    return;
                }
            }
        }
    }

done:
    logf("PROBE done: %d candidate%s, %d accepted, %d call%s attempted.%s",
         candidates, candidates == 1 ? "" : "s", accepted, attempts, attempts == 1 ? "" : "s",
         (mode < 2 && candidates > 0)
             ? " Set xrlayersrcprobe 2 to attempt the virtual call on these -- read the list first."
             : "");
}

#endif   // HALO_VR_DEV

// ============================================================================================
// Driving it
// ============================================================================================

uint32_t g_next_probe = 0;

void reset_slot(int s) {
    Target& t = g_t[s];
    if (t.fed != nullptr) { xrlayer_set_slot_source(s, nullptr); t.fed = nullptr; }
    t.native = nullptr;
    t.native_rt = nullptr;
    t.native_rhi = nullptr;
    t.dim = 0;
    t.fmt = 0;
}

// Re-validate one slot's chain end to end and, if it is still the pointer the layer holds, RECORD
// the copy immediately -- see the adjacency argument in XrLayer.hpp's capture section.
//
// `batch` is 0 = not opened yet, 1 = open, 2 = begin() refused this tick. Passing it through means
// the batch is opened lazily, right before the first record, rather than speculatively.
void service_slot(int s, uint32_t tick, bool feed, int* batch) {
    Target& t = g_t[s];
    if (t.comp == nullptr) {
        if (t.native != nullptr || t.fed != nullptr) reset_slot(s);
        return;
    }
    if (t.want < 16 || t.want > 4096) {
        if (s == 0) set_status("draw size %d is not a usable texture edge", t.want);
        return;
    }

    // LIVENESS FIRST, BEFORE EITHER RUNG DEREFERENCES ANYTHING. Both the widget hop and the pane
    // rung walk this pointer; a dead one faults inside UEVR, not here, which is why the crash never
    // named us. Dropping the slot is the path service_slot already takes for "no render target".
    if (!comp_still_live(t.comp, t.comp_index)) {
        static uint32_t said = 0;
        if (said < 8) {
            ++said;
            logf("slot %d: the component we adopted is no longer in its UObject array slot -- it was "
                 "freed (a level teardown does this). Dropping the source rather than handing a dead "
                 "object to UEVR.", s);
        }
        if (t.native != nullptr || t.fed != nullptr) reset_slot(s);
        t.comp = nullptr; t.comp_index = -1;
        return;
    }

    API::UObject* rt;
    if (t.is_rt) {
        // THE PANE RUNG: the caller handed us the render target itself, so there is no
        // GetRenderTarget to call and no widget to be up or down. Everything below this point is
        // identical -- the same offset walk, the same ValueAgreement on `want`, the same
        // re-validation every tick, the same refusal to capture from a resource whose identity
        // changed. The ONLY thing being skipped is one reflected UFunction call.
        // SAME CROSS-TICK HAZARD AS THE WIDGET RUNG, and this one had no check whatsoever: the
        // pointer is cast straight to a render target and then walked for offsets. A level
        // transition frees the pane's target exactly as it frees a widget, so validate it here for
        // the same reason and by the same means. Returning null lands on the `rt == nullptr` path
        // below, which already resets the slot and re-resolves.
        auto* pane = reinterpret_cast<API::UObject*>(t.comp);
        if (IsBadReadPtr(pane, 0x30) ||
            class_name_of(pane).find(L"RenderTarget") == std::wstring::npos) {
            pane = nullptr;
        }
        rt = pane;
    } else {
#if HALO_VR_DEV
        SplitTimer _t(&g_split.getrt_ms, &g_split.getrt_n);   // reflected GetRenderTarget, timed
#endif
        rt = component_render_target(reinterpret_cast<API::UObject*>(t.comp));
    }
    if (rt == nullptr) {
        if (t.native != nullptr || t.fed != nullptr) reset_slot(s);
        if (s == 0) set_status("widget reticule not up -- no render target to present");
        return;
    }
    if ((void*)rt != t.native_rt && t.native != nullptr) {
        // THE WEAPON-PICKUP CASE, and the reason the reticule flash was reproducible on demand.
        // Picking a weapon off the floor makes the HUD rebuild its crosshair and hand the component
        // a NEW render target; a navpoint slot does the same whenever the marker occupying it
        // changes type. The old ID3D12Resource then belongs to nothing and must not be CAPTURED
        // from -- which is not an argument for throwing away the frame already in the atlas.
        logf("slot %d re-allocated its render target (%p -> %p) -- dropping the source and "
             "re-resolving. Expect a new ID3D12Resource within a few ticks.",
             s, t.native_rt, (void*)rt);
        reset_slot(s);
    }

    if (!g_chain.valid()) return;
    if ((int32_t)(tick - t.next_resolve) < 0) return;
    // EVERY TICK ONCE SOMETHING IS BEING PRESENTED. This used to be a flat 16-tick (~0.5 s)
    // throttle, and that throttle is the window the 2026-08-23 crashes lived in. While nothing is
    // resolved yet there is nothing to go stale, so that case keeps the cheap throttle.
    t.next_resolve = tick + ((t.native != nullptr) ? 1 : 16);

    int dim = 0;
    uint32_t fmt = 0;
    void* rhi = nullptr;
    // Pass this slot's previous result as the cache. resolve_latched re-runs the cheap walk every
    // tick and only re-makes the expensive get_native_resource()+GetDesc call on a miss (no cache,
    // or the FRHITexture pointer changed). t.native/t.native_rhi are cleared by reset_slot on any
    // re-host or drop, so the cache can never outlive the chain it describes.
    void* native = resolve_latched(rt, t.want, t.native, t.native_rhi, t.fmt, &dim, &fmt, &rhi);
    if (native == nullptr) {
        if (t.native != nullptr) {
            logf("slot %d re-validation FAILED -- dropping the source and falling back. Nothing is "
                 "written through a stale address.", s);
            reset_slot(s);
        }
        return;
    }

    const bool changed = (native != t.native);
    t.native = native;
    t.native_rt = (void*)rt;
    t.native_rhi = rhi;
    t.dim = dim;
    t.fmt = fmt;
    if (changed) {
        logf("slot %d resolved: rt %p -> rhi %p -> ID3D12Resource %p (%dx%d, DXGI %u)",
             s, (void*)rt, rhi, native, dim, dim, fmt);
    }
    if (s == 0) set_status("resolved %p %dx%d fmt %u", native, dim, dim, fmt);

    // CAPTURE IT NOW, ON THIS THREAD, WITH NOTHING IN BETWEEN.
    //
    // This is deliberately the very next statement after the re-validation that walked the whole
    // chain and re-read the resource's own GetDesc. Its predecessor stamped a heartbeat instead and
    // let the SUBMIT thread do the copy "while the stamp is fresh" -- a check-then-use across a
    // thread boundary that crashed the process a third time on 2026-08-23. Do not reintroduce a gap
    // here, and in particular do not hoist the whole loop's validation above the whole loop's
    // recording: the batch exists to share ONE submission, not to defer the barriers.
    if (feed && t.fed == native) {
        if (*batch == 0) *batch = xrlayer_capture_begin() ? 1 : 2;
        if (*batch == 1) xrlayer_capture_record(s);
    }
}

}   // namespace

void xrsource_set_slot_component(int slot, void* widget_component, int want_dim) {
    if (slot < 0 || slot >= XRLAYER_SLOTS) return;
    g_req[slot].comp  = widget_component;
    g_req[slot].want  = want_dim;
    g_req[slot].is_rt = false;
}

void xrsource_set_slot_render_target(int slot, void* render_target, int want_dim) {
    if (slot < 0 || slot >= XRLAYER_SLOTS) return;
    g_req[slot].comp  = render_target;
    g_req[slot].want  = want_dim;
    g_req[slot].is_rt = true;
}

int xrsource_chain_crosscheck_state() { return g_crosscheck; }

void xrsource_reset() {
    for (int s = 0; s < XRLAYER_SLOTS; ++s) reset_slot(s);
    set_status("reset");
}

void xrsource_tick(uint32_t tick) {
    const bool feed = g_cfg.xr_layer_src;

#if HALO_VR_DEV
    g_split = ResolveSplit{};   // this tick's resolve split; summed over the slots serviced below
#endif

    // ASKING FOR THE FEED IS ASKING FOR THE RESOLUTION.
    //
    // xrlayersrcprobe used to gate the walk independently, so xrlayersrc=1 with the probe at its
    // default 0 produced `probe=0 feed=1 chain=none` forever: the feed was requested, nothing ever
    // resolved a source, and the layer silently kept drawing the generated ring. That is a setting
    // that appears to do nothing -- the exact failure this repo has already been bitten by twice
    // (the aimwidgettint clamp that reduced values with no log, and the live tunable whose change
    // never reached the pixels). Reported from a headset on 2026-08-23 as "I still am not seeing
    // the game art come through".
    //
    // There is no way to honour xrlayersrc=1 WITHOUT the resolution -- the whole feature is that
    // pointer -- so gating it behind a second, diagnostic-sounding key protected nobody. It only
    // made the request fail quietly. The probe key survives as a RESEARCH override (level 1 =
    // walk and log without the virtual call), which is a real thing to want on its own.
    const int probe_mode = (feed && g_cfg.xr_layer_src_probe < 2) ? 2 : g_cfg.xr_layer_src_probe;
    if (probe_mode != g_cfg.xr_layer_src_probe) {
        static bool said = false;
        if (!said) {
            said = true;
            logf("xrlayersrc=1 implies the source walk; running at probe level 2 (xrlayersrcprobe "
                 "was %d). Set xrlayersrcprobe explicitly only to research the walk itself.",
                 g_cfg.xr_layer_src_probe);
        }
    }

    if (probe_mode == 0 && !feed) {
        for (int s = 0; s < XRLAYER_SLOTS; ++s) {
            if (g_t[s].native != nullptr || g_t[s].fed != nullptr) reset_slot(s);
        }
        return;
    }

    // ---- adopt this tick's component set ------------------------------------------------------
    //
    // Slot 0 is the reticule's own component, found here as it always was. Slots 1..8 are whatever
    // Plugin.cpp pushed in through xrsource_set_slot_component() since the last tick; a slot it did
    // not name this tick has comp = nullptr and is dropped by service_slot().
    g_req[XRLAYER_SLOT_RETICULE].comp = (void*)g_ret_widget_comp.ptr;
    g_req[XRLAYER_SLOT_RETICULE].want = (int)g_cfg.aim_widget_draw;

    // xrlayernav=0 IS A SUFFICIENT OFF SWITCH FOR THE MARKER SLOTS, ON ITS OWN.
    //
    // Plugin.cpp is supposed to retire a slot it stops feeding (xrsource_set_slot_component(nullptr)),
    // but the marker request lives in g_req, which PERSISTS between ticks -- so toggling xrlayernav
    // off, or any nav path that just stops calling, leaves the last-fed component sitting in g_req
    // forever. That kept nine components resolving and re-validating every tick with the feature off
    // (reported from a headset: comps=9 while xrlayernav=0, slots=0x000, xrsource_tick 87-197 ms).
    // Gate the nav slots here, at the one point every path funnels through, so the collapse to
    // reticule-only cannot depend on every caller being perfect. The reticule (slot 0) is unaffected;
    // it is not a navpoint and has its own key (xrlayersrc). The adoption loop below sees comp go to
    // nullptr and drops the slot, logging the drop once, exactly as a component change does.
    // BOUNDED BY XRLAYER_NAV_COUNT, NOT XRLAYER_SLOTS. This read `s < XRLAYER_SLOTS` while the
    // markers were the only slots above 0; adding the pane at slot 9 turned it into "xrlayernav=0
    // also silently unfeeds the scope pane", which is a different feature with a different key.
    if (!g_cfg.xr_layer_nav) {
        for (int s = XRLAYER_SLOT_NAV_BASE; s < XRLAYER_SLOT_NAV_BASE + XRLAYER_NAV_COUNT; ++s) {
            g_req[s].comp = nullptr;
        }
    }

    for (int s = 0; s < XRLAYER_SLOTS; ++s) {
        Target& t = g_t[s];
        // is_rt is part of the IDENTITY, not a mode flag applied to a surviving resolve: the same
        // address reused as a different KIND of object would otherwise keep the previous rung's
        // cached native pointer. Cheap insurance against a case that should never happen.
        // ALREADY REFUSED THIS EXACT POINTER -- do not walk the object array for it again. See
        // g_slot_rejected. Without this the teardown case re-walks ~296k objects per tick, per
        // stuck slot, forever, and re-logs the refusal with it.
        if (g_req[s].comp != nullptr && g_req[s].comp == g_slot_rejected[s] && t.comp == nullptr) {
            t.want = g_req[s].want;
            continue;
        }
        if (g_req[s].comp != t.comp || g_req[s].is_rt != t.is_rt) {
            // A new widget component means a new render target and a new D3D12 resource. Dropping
            // the old one here is what stops us capturing from freed memory across a mission
            // transition -- and the symptom if we did not would be art that looks right until the
            // level changes.
            //
            // SAY SO. This drop and the one in service_slot used to be silent, and between them
            // they are the whole of the "the placeholder ring flashes for a split second" report
            // from 2026-08-23. A drop is a handful of times per level, so this cannot spam.
            if (t.native != nullptr) {
                logf("slot %d component changed (%p -> %p) -- dropping the source; the layer holds "
                     "the last captured frame while this re-resolves (xrlayerhold).",
                     s, t.comp, g_req[s].comp);
            }
            t.comp  = g_req[s].comp;
            // Resolve the array slot for the NEW component. One walk, on change only -- never on
            // the steady path, because a full object-array walk per tick is the exact cost this
            // project has already been bitten by (see the find_uobject miss).
            t.comp_index = -1;
            if (t.comp != nullptr) {
                if (auto* arr = API::get()->get_uobject_array()) {
                    const int32_t n = arr->get_object_count();
                    for (int32_t i = 0; i < n; ++i) {
                        if (arr->get_object(i) == t.comp) { t.comp_index = i; break; }
                    }
                }
                if (t.comp_index < 0) {
                    logf("slot %d: component %p is not in the UObject array -- refusing to hold it. "
                         "Nothing will be captured for this slot until a live one is published.",
                         s, t.comp);
                    g_slot_rejected[s] = t.comp;   // remember it: never re-walk this same pointer
                    t.comp = nullptr;
                } else {
                    g_slot_rejected[s] = nullptr;  // resolved -- a later refusal starts fresh
                }
            }
            t.is_rt = g_req[s].is_rt;
            reset_slot(s);
        }
        t.want = g_req[s].want;
    }

    // ---- re-validate and capture, ONE batch for all nine slots --------------------------------
    //
    // 0 = batch not opened, 1 = open, 2 = xrlayer_capture_begin() refused this tick (its ring is
    // busy, which is a skip and never a block). Opened lazily inside service_slot so that the
    // first record still immediately follows its own validation.
    int batch = 0;
    for (int s = 0; s < XRLAYER_SLOTS; ++s) service_slot(s, tick, feed, &batch);
    if (batch == 1) xrlayer_capture_submit();

    // ---- feed the compositor layer, on change only, per slot ----------------------------------
    //
    // "On change" means the POINTER changing, and a REFUSAL IS NOT A CHANGE. The layer can
    // legitimately say no (a slot with no atlas cell, a size that does not match its cell), and
    // that answer can stop being true later in the same session -- so a refusal keeps the offer
    // pending and re-tries at a slow, quiet cadence rather than latching the feature off.
    if (feed) {
        for (int s = 0; s < XRLAYER_SLOTS; ++s) {
            Target& t = g_t[s];
            if (t.native != nullptr && t.native != t.fed) {
                if ((int32_t)(tick - t.next_offer) >= 0) {
                    t.next_offer = tick + 32;   // ~1 s; a refused offer must not spin the log
                    if (xrlayer_set_slot_source(s, t.native)) {
                        t.fed = t.native;
                        logf("handed %p to compositor slot %d.", t.native, s);
                    }
                }
            } else if (t.native == nullptr && t.fed != nullptr) {
                xrlayer_set_slot_source(s, nullptr);
                t.fed = nullptr;
            }
        }
    } else {
        for (int s = 0; s < XRLAYER_SLOTS; ++s) {
            if (g_t[s].fed != nullptr) { xrlayer_set_slot_source(s, nullptr); g_t[s].fed = nullptr; }
        }
    }

#if HALO_VR_DEV
    // ---- the walk: only when we have nothing, and never faster than once a second ----
    if (probe_mode > 0 && !g_chain.valid() && (int32_t)(tick - g_next_probe) >= 0) {
        g_next_probe = tick + 32;

        const int want = g_t[XRLAYER_SLOT_RETICULE].want;
        // The widget's own target when it exists, otherwise one we make. The offsets are the same
        // either way; see probe_render_target().
        API::UObject* subject =
            component_render_target(g_ret_widget_comp.get_checked(L"WidgetComponent"));
        if (subject == nullptr && want >= 16 && want <= 4096) subject = probe_render_target(want);

        if (subject == nullptr) {
            set_status("no render target to probe (neither the widget's nor a made one)");
        } else {
            probe(subject, want, probe_mode, &g_chain);
            if (g_chain.valid()) {
                for (auto& t : g_t) t.next_resolve = tick;   // resolve through it next tick
                logf("LATCHED chain rt+0x%X / res+0x%X / rhi+0x%X. Re-validated on every resolve.",
                     (unsigned)g_chain.off_res, (unsigned)g_chain.off_rhi, (unsigned)g_chain.off_ext);
            }
        }
    }

    // ---- PROVE THE OFFSETS ARE CLASS-LEVEL RATHER THAN ASSUMING IT -----------------------------
    //
    // Nine components resolving through ONE measured chain is only sound if off_res/off_rhi/off_ext
    // are members of UTexture/FTexture and therefore identical for every UTextureRenderTarget2D.
    // That is very probably true. It is also exactly the kind of "obviously fine" step this repo's
    // address rule exists to stop being taken on faith -- so once a SECOND component has a render
    // target, walk it into a scratch chain and compare.
    //
    // OPT-IN, THROTTLED, and NEVER on the hot path -- because probe() is the expensive walk.
    //
    // This costs a full triple-nested discovery probe (RT_WINDOW x RES_WINDOW x EXT_WINDOW, each
    // iteration classifying memory with VirtualQuery), tens to ~200 ms of it. It USED to run on
    // every tick until it reached a verdict: the "no accepted chain, retry" branch below never
    // latches g_crosscheck, so while a nav slot's render target had no chain yet it re-ran the whole
    // probe on the next tick, and the next. That is the 87-197 ms xrsource_tick the headset saw with
    // markers up. HALO_VR_DEV did not protect anyone here because we PLAYTEST on dev builds, so the
    // harness now needs its own explicit key (xrlayersrcxcheck, default 0) AND a throttle: even when
    // a developer turns it on it cannot run faster than ~once a second, and it still runs at most
    // until it reaches a verdict (g_crosscheck != 0), after which this block is skipped for good.
    // On the steady play path g_crosscheck stays 0 and the state line reads xcheck=0 -- correct,
    // because the proof is a research question, not part of resolving the feed.
    static uint32_t s_xcheck_next = 0;
    if (g_cfg.xr_layer_src_xcheck && g_crosscheck == 0 && g_chain.valid() &&
        (int32_t)(tick - s_xcheck_next) >= 0) {
        s_xcheck_next = tick + 32;
        for (int s = XRLAYER_SLOT_NAV_BASE; s < XRLAYER_SLOTS; ++s) {
            if (g_t[s].comp == nullptr || g_t[s].want < 16) continue;
            API::UObject* rt2 = component_render_target(reinterpret_cast<API::UObject*>(g_t[s].comp));
            if (rt2 == nullptr || (void*)rt2 == g_t[XRLAYER_SLOT_RETICULE].native_rt) continue;

            Chain scratch;
            probe(rt2, g_t[s].want, 2, &scratch);
            if (!scratch.valid()) {
                // Not a disagreement -- the walk simply found nothing to accept on this component
                // (its target may not have an RHI texture yet). Leave the flag at 0 and try the
                // next slot on a later tick.
                logf("CROSS-CHECK: slot %d's render target produced no accepted chain this time -- "
                     "not a disagreement, will retry.", s);
                break;
            }
            if (scratch.off_res == g_chain.off_res && scratch.off_rhi == g_chain.off_rhi &&
                scratch.off_ext == g_chain.off_ext) {
                g_crosscheck = 1;
                logf("CROSS-CHECK PASSED: slot %d's render target measures the SAME chain "
                     "rt+0x%X / res+0x%X / rhi+0x%X as the reticule's. The offsets are class-level "
                     "(UTexture/FTexture members), which is what lets one measurement serve every "
                     "slot. This is now measured rather than assumed.",
                     s, (unsigned)scratch.off_res, (unsigned)scratch.off_rhi,
                     (unsigned)scratch.off_ext);
            } else {
                g_crosscheck = -1;
                logf("CROSS-CHECK FAILED: slot %d measures rt+0x%X / res+0x%X / rhi+0x%X but the "
                     "reticule measured rt+0x%X / res+0x%X / rhi+0x%X. The offsets are NOT "
                     "class-level on this build, so one chain cannot serve every slot. NOT adopting "
                     "the second chain -- every resolve still re-validates dimensions and desc, so "
                     "a wrong slot resolves to nothing rather than to the wrong texture, but this "
                     "needs looking at before the marker lane is trusted.",
                     s, (unsigned)scratch.off_res, (unsigned)scratch.off_rhi,
                     (unsigned)scratch.off_ext, (unsigned)g_chain.off_res,
                     (unsigned)g_chain.off_rhi, (unsigned)g_chain.off_ext);
            }
            break;
        }
    }
#else
    if (probe_mode > 0) {
        static bool said = false;
        if (!said) {
            said = true;
            logf("xrlayersrcprobe is a DEV-BUILD key: the discovery walk is not compiled into a "
                 "shipping build. Without measured offsets there is nothing to resolve.");
        }
    }
#endif

    // ---- SAY SOMETHING, EVERY ~4 s, WHENEVER A KEY IS ON ----
    //
    // The first probe run produced NOT ONE LINE of log: the widget reticule was never built on that
    // lane, the early return only set a status string nobody printed, and from the outside that was
    // indistinguishable from the module not being compiled in, the key not being parsed, or the
    // build not being deployed. A diagnostic that can be completely silent is a diagnostic that
    // costs a whole session to interpret.
    if ((tick % 128) == 0) {
        int n_comp = 0, n_res = 0, n_fed = 0;
        for (const auto& t : g_t) {
            if (t.comp != nullptr) ++n_comp;
            if (t.native != nullptr) ++n_res;
            if (t.fed != nullptr) ++n_fed;
        }
        logf("state: probe=%d feed=%d chain=%s xcheck=%d comps=%d resolved=%d fed=%d | %s",
             probe_mode, (int)feed, g_chain.valid() ? "latched" : "none", g_crosscheck,
             n_comp, n_res, n_fed, g_status);
#if HALO_VR_DEV
        // THE SPLIT, for this one tick, so "the per-slot cost is X" is a measurement. cache=on/off
        // (xrlayersrccache) decides whether native/desc are paid every tick or only on a miss.
        logf("  resolve split (this tick): getrt=%.2fms/%u  native(get_native_resource)=%.2fms/%u  "
             "desc(GetDesc)=%.2fms/%u  walk=%.2fms | cache=%s hit=%u miss=%u | "
             "per-native=%.2fms per-getrt=%.3fms",
             g_split.getrt_ms, g_split.getrt_n, g_split.native_ms, g_split.native_n,
             g_split.desc_ms, g_split.desc_n, g_split.walk_ms,
             g_cfg.xr_layer_src_cache ? "on" : "OFF", g_split.cache_hit, g_split.cache_miss,
             g_split.native_n ? g_split.native_ms / g_split.native_n : 0.0,
             g_split.getrt_n ? g_split.getrt_ms / g_split.getrt_n : 0.0);
#endif
    }
}

void* xrsource_resource() { return g_t[XRLAYER_SLOT_RETICULE].native; }

int xrsource_dim() { return g_t[XRLAYER_SLOT_RETICULE].dim; }

const char* xrsource_status() { return g_status; }

}   // namespace halo
