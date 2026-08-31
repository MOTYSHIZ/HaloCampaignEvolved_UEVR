#include "PaletteHook.hpp"

#include "NodeMap.hpp"
#include "../addrcascade/AddressCascade.hpp"

#include "uevr/API.hpp"

#include <windows.h>
#include <intrin.h>

#include <atomic>
#include <cstdio>

using uevr::API;

namespace halo::palettearm {
namespace {

// ---- THE ADDRESS CHAIN -------------------------------------------------------------------------
// Every constant below is a measurement of ONE build of HaloSimulation_tag_release.dll, taken by
// elliotttate. They are grouped here rather than scattered so the blast radius of a game patch is
// one screen of code.

// The builder's prologue. 31 bytes, all exact: mov rax,rsp / the register spills / push rbx rsi rdi
// r12-r15 / sub rsp,3B0h. A long, specific prologue is what makes a unique scan plausible.
constexpr unsigned char kBuildSig[] = {
    0x48, 0x8B, 0xC4, 0x44, 0x88, 0x40, 0x18, 0x89, 0x50, 0x10,
    0x89, 0x48, 0x08, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55,
    0x41, 0x56, 0x41, 0x57, 0x48, 0x81, 0xEC, 0xB0, 0x03, 0x00,
    0x00};
constexpr char kBuildMask[] = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
static_assert(sizeof(kBuildMask) - 1 == sizeof(kBuildSig), "signature and mask must agree");

// ADDR-HYGIENE: resolved -- fallback only. palettehook_install() scans kBuildSig first and requires
// a UNIQUE match; this RVA is used only if the scan finds nothing, and even then only after the
// signature bytes are confirmed present AT this address. If both fail the hook stays off.
constexpr uintptr_t FP_WEAPON_BUILD_RVA = 0x46EC10;

// ADDR-HYGIENE: resolved -- fallback only. addrcascade::tls_index() reads the PE TLS directory,
// which is structural and survives patches; this recorded slot is compared against it and used
// only if the directory read fails. A mismatch is logged.
constexpr uintptr_t TLS_INDEX_RVA = 0xD72730;

// ---- THE WEAPON-SLOT OFFSET CHAIN --------------------------------------------------------------
//
// Five levels deep into a struct nobody here has a symbol for: TLS block -> player bank -> player
// -> weapon slot -> palette. Measured on one build, with no independent resolver available for any
// of it. This is the most fragile thing in the folder and it is where a game patch will land.
//
// They are not trusted blind. derive_palette() reads the flags/object/animation/tag/count fields
// FIRST and requires all six to be self-consistent (flags 0x0C set, no -1 indices, both node
// counts exactly 76) before the palette pointer is even formed; nodemap_validate() then has to
// agree the result behaves like a pair of hands; and the whole dereference sits inside an
// __except. That is a VALUE check rather than a writability check, which is the distinction
// src\addrcascade\README.md insists on -- but it is still a measurement, and it will rot silently
// on a repack. RE-DERIVE these, never nudge them.
//
// Each constant carries its own marker because Check-AddressHygiene.ps1 looks within 6 lines of
// the declaration; one block comment above the group does not cover the tail of it.

// ADDR-HYGIENE: UNGUARDED -- see the chain note above; value-checked in derive_palette(), not resolved.
constexpr size_t TLS_PLAYER_BANK_OFF   = 0x4F8;
// ADDR-HYGIENE: UNGUARDED -- stride, same chain. NOTE: *_STRIDE names do not match the audit's
// pattern (it keys on RVA/OFF/ADDR/BASE), so these two would have gone unlisted. Worth widening
// the pattern in Check-AddressHygiene.ps1 -- a stride is exactly the kind of constant it exists for.
constexpr size_t PLAYER_STRIDE         = 0x52D8;
// ADDR-HYGIENE: UNGUARDED -- stride, same chain, same audit-pattern gap as PLAYER_STRIDE.
constexpr size_t WEAPON_SLOT_STRIDE    = 0x2908;
// ADDR-HYGIENE: UNGUARDED -- see the chain note above; one of the six fields that must agree.
constexpr size_t SLOT_FLAGS_OFF        = 0x38;
// ADDR-HYGIENE: UNGUARDED -- see the chain note above; one of the six fields that must agree.
constexpr size_t SLOT_OBJECT_INDEX_OFF = 0x44;
// ADDR-HYGIENE: UNGUARDED -- see the chain note above; one of the six fields that must agree.
constexpr size_t SLOT_ANIM_INDEX_OFF   = 0x58;
// ADDR-HYGIENE: UNGUARDED -- see the chain note above; one of the six fields that must agree.
constexpr size_t SLOT_MODEL_TAG_OFF    = 0x194;
// ADDR-HYGIENE: UNGUARDED -- see the chain note above; must read exactly 76 or the drive stands down.
constexpr size_t SOURCE_NODE_COUNT_OFF = 0x108C;
// ADDR-HYGIENE: UNGUARDED -- see the chain note above; must read exactly 76 or the drive stands down.
constexpr size_t FINAL_NODE_COUNT_OFF  = 0x1090;
// ADDR-HYGIENE: UNGUARDED -- see the chain note above. THIS is the one we write through; it is
// guarded only by everything above it agreeing first, plus nodemap_validate().
constexpr size_t FINAL_PALETTE_OFF     = 0x1094;

// ---- THE SHARED CAPTURE BUFFER -- THE ONE THE RENDERER ACTUALLY READS ---------------------------
//
// Writing the live weapon slot above is NOT enough, and this is measured rather than argued: on
// 2026-08-25 the drive succeeded on 2,931 consecutive calls with the entire node set displaced half
// a metre upward, and the screen did not change. In the same run the builder's
// `capture_render_palette` flag was FALSE on all 2,931 calls.
//
// elliotttate's original says why: "The render reader can select a captured interpolation palette
// instead of the live slot." So the stock builder mirrors each pose into a shared capture record,
// and the renderer may read THAT. A pose written only to the slot is real, correct, and invisible.
// This block is the piece of his port that was dropped, and it is why armdriver=2 did nothing for
// days while every diagnostic reported success.
//
// TWO BANKS, BOTH WRITTEN. Halo blends the previous bank against the current one for sub-tick
// smoothness. For stock animation that is wanted; for controller-driven nodes it is unwanted
// motion smoothing -- it lags the rendered weapon up to a tick behind the hand and, during a
// strafe, drags the visible gun behind the pose projectiles spawn from. Writing the same fresh
// pose into both endpoints collapses the blend for our nodes while stock nodes interpolate as
// usual.
//
// ADDR-HYGIENE: UNGUARDED -- the capture chain, measured on the same foreign build as the slot
// chain above. NOT written blind: each record must match BOTH the model tag and the node count of
// the palette the builder just produced before anything is written through it, and the context
// itself is range-checked (context[0] < 2, context[2] != 0). That is a value check on two
// independent fields, the same standard derive_palette() is held to -- but it is a measurement and
// it will rot on a repack. Re-derive, never nudge.
constexpr uintptr_t SHARED_CAPTURE_PTR_RVA  = 0x1831220;
// ADDR-HYGIENE: UNGUARDED -- see the capture-chain note above.
constexpr size_t TLS_CAPTURE_CONTEXT_OFF    = 0x5B8;
// ADDR-HYGIENE: UNGUARDED -- see the capture-chain note above.
constexpr size_t CAPTURE_CONTEXT_STRIDE     = 0x30600;
// ADDR-HYGIENE: UNGUARDED -- see the capture-chain note above.
constexpr size_t CAPTURE_PLAYER_STRIDE      = 0x30D4;
// ADDR-HYGIENE: UNGUARDED -- see the capture-chain note above.
constexpr size_t CAPTURE_SLOT_STRIDE        = 0x1868;
// ADDR-HYGIENE: UNGUARDED -- see the capture-chain note above; matched against the live model tag.
constexpr size_t CAPTURE_TAG_OFF            = 0x24010;
// ADDR-HYGIENE: UNGUARDED -- see the capture-chain note above; matched against the live node count.
constexpr size_t CAPTURE_COUNT_OFF          = 0x24014;
// ADDR-HYGIENE: UNGUARDED -- see the capture-chain note above. THIS is the one the renderer reads.
constexpr size_t CAPTURE_PALETTE_OFF        = 0x24018;

// ---- STATE -------------------------------------------------------------------------------------

using BuildFn = void (*)(std::int32_t local_player, std::int32_t weapon_slot, bool capture);

BuildFn          s_original{};
PaletteDriveFn   s_drive{};
int              s_hook_id = -1;
uint8_t*         s_sim_base = nullptr;
uint32_t         s_tls_index = 0;
bool             s_tls_index_ok = false;
std::atomic<uint64_t> s_calls{0};

// CAPTURE-FLAG CENSUS. The builder takes a `capture_render_palette` flag that this detour passes
// through and otherwise ignores. If the game calls it twice per frame -- once to build and once to
// capture for rendering -- then which call we write on decides whether anything is ever seen. The
// counts say whether that split exists at all; guessing about it has already cost sessions.
std::atomic<uint64_t> s_calls_capture{0};
std::atomic<uint64_t> s_calls_nocapture{0};

// How many capture-bank records have been written. Zero while the hook is running means the mirror
// is finding no matching record, which is a completely different problem from the drive failing.
std::atomic<uint64_t> s_bank_writes{0};
char             s_resolution[192] = "not resolved";

// 600 ticks is ~10 s at 60 Hz on the engine tick this is driven from. Stated here because getting
// the cadence wrong fails silently in the direction that looks fine: the alarm just never fires.
addrcascade::HookWatchdog s_watchdog{600};

// ---- REACHING THE PALETTE ----------------------------------------------------------------------

bool derive_palette(std::int32_t local_player, std::int32_t weapon_slot, PaletteAccess* out) {
    if (s_sim_base == nullptr || !s_tls_index_ok || out == nullptr) return false;
    if (local_player < 0 || local_player > 3 || weapon_slot < 0 || weapon_slot > 1) return false;

    // The game thread's TLS block. __readgsqword(0x58) is the x64 TEB's ThreadLocalStoragePointer;
    // that part is ABI, not a measurement.
    auto** const tls_slots = reinterpret_cast<void**>(__readgsqword(0x58));
    if (tls_slots == nullptr) return false;
    auto* const tls = static_cast<uint8_t*>(tls_slots[s_tls_index]);
    if (tls == nullptr) return false;

    auto* const player_bank = *reinterpret_cast<uint8_t**>(tls + TLS_PLAYER_BANK_OFF);
    if (player_bank == nullptr) return false;

    uint8_t* const slot = player_bank +
                          static_cast<size_t>(local_player) * PLAYER_STRIDE +
                          static_cast<size_t>(weapon_slot)  * WEAPON_SLOT_STRIDE;

    // Prove the slot is a live first-person weapon BEFORE forming a pointer into it. Five
    // independent fields have to agree; unrelated memory clearing all five is far less likely than
    // clearing any one.
    const uint32_t flags        = *reinterpret_cast<const uint32_t*>(slot + SLOT_FLAGS_OFF);
    const int32_t  object_index = *reinterpret_cast<const int32_t*>(slot + SLOT_OBJECT_INDEX_OFF);
    const int32_t  anim_index   = *reinterpret_cast<const int32_t*>(slot + SLOT_ANIM_INDEX_OFF);
    const int32_t  model_tag    = *reinterpret_cast<const int32_t*>(slot + SLOT_MODEL_TAG_OFF);
    const int32_t  source_count = *reinterpret_cast<const int32_t*>(slot + SOURCE_NODE_COUNT_OFF);
    const int32_t  final_count  = *reinterpret_cast<const int32_t*>(slot + FINAL_NODE_COUNT_OFF);

    if ((flags & 0x0C) != 0x0C || object_index == -1 || anim_index == -1 || model_tag == -1 ||
        source_count != static_cast<int32_t>(kFirstPersonNodeCount) ||
        final_count  != static_cast<int32_t>(kFirstPersonNodeCount)) {
        return false;
    }

    out->palette      = reinterpret_cast<BlamMatrix4x3*>(slot + FINAL_PALETTE_OFF);
    out->node_count   = static_cast<uint32_t>(final_count);
    out->local_player = local_player;
    out->weapon_slot  = weapon_slot;
    out->model_tag    = model_tag;
    out->is_capture_bank = false;
    out->bank_index   = 0;
    return true;
}

// Drive the capture records the renderer may read instead of the live slot. Returns how many
// banks were written.
//
// Each record is only touched when its OWN tag and node count match the palette the builder just
// produced -- a record for a different weapon, or a stale one, is left alone.
int drive_capture_banks(const PaletteAccess& live, PaletteDriveFn drive) {
    if (s_sim_base == nullptr || !s_tls_index_ok || drive == nullptr) return 0;

    auto** const tls_slots = reinterpret_cast<void**>(__readgsqword(0x58));
    if (tls_slots == nullptr) return 0;
    auto* const tls = static_cast<uint8_t*>(tls_slots[s_tls_index]);
    if (tls == nullptr) return 0;

    auto* const context = *reinterpret_cast<uint8_t**>(tls + TLS_CAPTURE_CONTEXT_OFF);
    auto* const shared  = *reinterpret_cast<uint8_t**>(s_sim_base + SHARED_CAPTURE_PTR_RVA);
    if (context == nullptr || shared == nullptr) return 0;
    if (context[2] == 0 || context[0] >= 2) return 0;   // capture not active this frame

    int written = 0;
    for (uint8_t bank = 0; bank < 2; ++bank) {
        uint8_t* const record = shared +
            static_cast<size_t>(bank) * CAPTURE_CONTEXT_STRIDE +
            static_cast<size_t>(live.local_player) * CAPTURE_PLAYER_STRIDE +
            static_cast<size_t>(live.weapon_slot) * CAPTURE_SLOT_STRIDE;

        const int32_t tag   = *reinterpret_cast<const int32_t*>(record + CAPTURE_TAG_OFF);
        const int32_t count = *reinterpret_cast<const int32_t*>(record + CAPTURE_COUNT_OFF);
        if (tag != live.model_tag || count != static_cast<int32_t>(live.node_count)) continue;

        PaletteAccess bank_access = live;
        bank_access.palette         = reinterpret_cast<BlamMatrix4x3*>(record + CAPTURE_PALETTE_OFF);
        bank_access.is_capture_bank = true;
        bank_access.bank_index      = bank;
        if (drive(bank_access)) ++written;
    }
    return written;
}

// ---- THE DETOUR --------------------------------------------------------------------------------

void hooked_build(std::int32_t local_player, std::int32_t weapon_slot, bool capture) {
    // Let the game build its pose first; we are a post-pass over the result, not a replacement.
    if (s_original != nullptr) s_original(local_player, weapon_slot, capture);

    s_calls.fetch_add(1, std::memory_order_relaxed);
    (capture ? s_calls_capture : s_calls_nocapture).fetch_add(1, std::memory_order_relaxed);

    PaletteDriveFn drive = s_drive;
    if (drive == nullptr) return;

    PaletteAccess access{};
    if (!derive_palette(local_player, weapon_slot, &access)) return;

    // Structured exception handling, not a try/catch: this dereferences memory whose layout is a
    // measurement. If an offset has rotted, a crash here is a crash in the player's game -- and
    // the frame path is not where a crash should be discovered.
    __try {
        // The live slot first, then the capture banks the renderer may read instead. Writing only
        // the slot is invisible -- measured, see the capture-chain note above.
        (void)drive(access);
        s_bank_writes.fetch_add(
            static_cast<uint64_t>(drive_capture_banks(access, drive)),
            std::memory_order_relaxed);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        s_drive = nullptr;   // one fault disables the drive for the session; the hook stays.
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEARM: faulted inside the palette drive -- "
                             "disabled for this session. The offset chain has probably moved.");
    }
}

} // namespace

const char*   palettehook_resolution() { return s_resolution; }
bool          palettehook_installed()  { return s_hook_id >= 0; }
std::uint64_t palettehook_call_count() { return s_calls.load(std::memory_order_relaxed); }
std::uint64_t palettehook_capture_calls()   { return s_calls_capture.load(std::memory_order_relaxed); }
std::uint64_t palettehook_nocapture_calls() { return s_calls_nocapture.load(std::memory_order_relaxed); }
std::uint64_t palettehook_bank_writes()      { return s_bank_writes.load(std::memory_order_relaxed); }

bool palettehook_watchdog_tick(bool gameplay_active) {
    // event_happened is a LATCH -- "has this hook EVER been called" -- not "did it fire since the
    // previous tick". AddressCascade.hpp states that on the parameter itself, and the first
    // version of this function passed the per-tick delta anyway.
    //
    // The cost was not theoretical. On 2026-08-23 this exact code logged, two lines apart:
    //     "the hook installed but has NEVER been called"
    //     "hook removed -- 65527 calls seen"
    // condemning a hook that was running 65,527 times. The drive was returning early every frame
    // (the node map was rejecting this build skeleton), so the COUNT stopped changing while the
    // hook itself kept being called -- and a delta reads that as silence. The latch cannot make
    // that mistake: called once, ever, is the whole question.
    //
    // event_possible is gameplay_active, NOT "installed" -- see the header.
    return s_watchdog.tick(palettehook_installed() && gameplay_active,
                           palettehook_call_count() > 0);
}

HookInstall palettehook_install(PaletteDriveFn drive) {
    if (palettehook_installed()) return HookInstall::Installed;
    if (drive == nullptr) return HookInstall::Failed;

    HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
    if (sim == nullptr) {
        // Not an error. The simulation DLL is not loaded at the main menu or mid-load, and the
        // caller is expected to keep asking -- condemning the feature here would kill it on
        // every launch before gameplay ever started.
        std::snprintf(s_resolution, sizeof(s_resolution),
                      "waiting: HaloSimulation_tag_release.dll is not loaded yet");
        return HookInstall::WaitingForModule;
    }
    s_sim_base = reinterpret_cast<uint8_t*>(sim);

    // TLS index: prefer the PE directory (structural), fall back to the recorded slot.
    uint32_t  directory_index = 0;
    uintptr_t directory_rva   = 0;
    if (addrcascade::tls_index(sim, &directory_index, &directory_rva)) {
        s_tls_index    = directory_index;
        s_tls_index_ok = true;
    } else {
        s_tls_index    = *reinterpret_cast<const DWORD*>(s_sim_base + TLS_INDEX_RVA);
        s_tls_index_ok = true;
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEARM: PE TLS directory unreadable -- using "
                             "the recorded slot at dll+0x%llX (index %u)",
                             (unsigned long long)TLS_INDEX_RVA, s_tls_index);
    }

    // Rung 1: scan. A unique match is the only acceptable result -- two matches means the
    // signature no longer identifies one function and picking the first is a coin toss.
    const addrcascade::Signature sig{kBuildSig, kBuildMask, sizeof(kBuildSig)};
    uintptr_t target = 0;
    const addrcascade::ScanResult scan = addrcascade::scan_signature(sim, sig);
    if (scan.unique() && !addrcascade::fault(1)) {
        target = scan.address;
        std::snprintf(s_resolution, sizeof(s_resolution),
                      "signature, unique match at dll+0x%llX",
                      (unsigned long long)(target - reinterpret_cast<uintptr_t>(sim)));
    } else {
        // Rung 2: the recorded RVA -- but only if the prologue really is there. Without that check
        // this is just "hook whatever lives at 0x46EC10", which succeeds on any readable address.
        const uintptr_t candidate = reinterpret_cast<uintptr_t>(sim) + FP_WEAPON_BUILD_RVA;
        const bool bytes_match =
            addrcascade::readable_bytes(reinterpret_cast<const void*>(candidate),
                                        sizeof(kBuildSig)) >= sizeof(kBuildSig) &&
            std::memcmp(reinterpret_cast<const void*>(candidate), kBuildSig,
                        sizeof(kBuildSig)) == 0 &&
            !addrcascade::fault(2);
        if (bytes_match) {
            target = candidate;
            std::snprintf(s_resolution, sizeof(s_resolution),
                          "recorded RVA dll+0x%llX, prologue verified (scan found %llu matches)",
                          (unsigned long long)FP_WEAPON_BUILD_RVA,
                          (unsigned long long)scan.matches);
        } else {
            std::snprintf(s_resolution, sizeof(s_resolution),
                          "UNRESOLVED -- scan found %llu matches and dll+0x%llX does not hold "
                          "the prologue. This build has moved the builder.",
                          (unsigned long long)scan.matches,
                          (unsigned long long)FP_WEAPON_BUILD_RVA);
            API::get()->log_info("[Halo-CampE-UEVR] PALETTEARM: %s Arms stay stock.", s_resolution);
            return HookInstall::Failed;
        }
    }

    s_drive = drive;
    const int id = API::get()->param()->functions->register_inline_hook(
        reinterpret_cast<void*>(target), reinterpret_cast<void*>(&hooked_build),
        reinterpret_cast<void**>(&s_original));
    if (id < 0 || s_original == nullptr) {
        s_drive = nullptr;
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEARM: register_inline_hook FAILED (id=%d) on "
                             "0x%llX -- arms stay stock", id, (unsigned long long)target);
        return HookInstall::Failed;
    }

    s_hook_id = id;
    s_watchdog.reset();
    API::get()->log_info("[Halo-CampE-UEVR] PALETTEARM: hook installed on 0x%llX via %s (id=%d). "
                         "INSTALLED IS NOT RUNNING -- watch for the first-call line.",
                         (unsigned long long)target, s_resolution, id);
    return HookInstall::Installed;
}

void palettehook_uninstall() {
    s_drive = nullptr;
    if (s_hook_id >= 0) {
        API::get()->param()->functions->unregister_inline_hook(s_hook_id);
        s_hook_id = -1;
        s_original = nullptr;
        API::get()->log_info("[Halo-CampE-UEVR] PALETTEARM: hook removed (%llu calls seen)",
                             (unsigned long long)palettehook_call_count());
    }
}

} // namespace halo::palettearm
