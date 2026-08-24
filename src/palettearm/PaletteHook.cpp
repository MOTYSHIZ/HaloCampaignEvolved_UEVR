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

// ---- STATE -------------------------------------------------------------------------------------

using BuildFn = void (*)(std::int32_t local_player, std::int32_t weapon_slot, bool capture);

BuildFn          s_original{};
PaletteDriveFn   s_drive{};
int              s_hook_id = -1;
uint8_t*         s_sim_base = nullptr;
uint32_t         s_tls_index = 0;
bool             s_tls_index_ok = false;
std::atomic<uint64_t> s_calls{0};
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
    return true;
}

// ---- THE DETOUR --------------------------------------------------------------------------------

void hooked_build(std::int32_t local_player, std::int32_t weapon_slot, bool capture) {
    // Let the game build its pose first; we are a post-pass over the result, not a replacement.
    if (s_original != nullptr) s_original(local_player, weapon_slot, capture);

    s_calls.fetch_add(1, std::memory_order_relaxed);

    PaletteDriveFn drive = s_drive;
    if (drive == nullptr) return;

    PaletteAccess access{};
    if (!derive_palette(local_player, weapon_slot, &access)) return;

    // Structured exception handling, not a try/catch: this dereferences memory whose layout is a
    // measurement. If an offset has rotted, a crash here is a crash in the player's game -- and
    // the frame path is not where a crash should be discovered.
    __try {
        (void)drive(access);
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

bool palettehook_watchdog_tick(bool gameplay_active) {
    static uint64_t s_seen = 0;
    const uint64_t now = palettehook_call_count();
    const bool happened = now != s_seen;
    s_seen = now;
    // event_possible is gameplay_active, NOT "installed". See the header.
    return s_watchdog.tick(palettehook_installed() && gameplay_active, happened);
}

bool palettehook_install(PaletteDriveFn drive) {
    if (palettehook_installed()) return true;
    if (drive == nullptr) return false;

    HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
    if (sim == nullptr) {
        std::snprintf(s_resolution, sizeof(s_resolution),
                      "HaloSimulation_tag_release.dll is not loaded");
        return false;
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
            return false;
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
        return false;
    }

    s_hook_id = id;
    s_watchdog.reset();
    API::get()->log_info("[Halo-CampE-UEVR] PALETTEARM: hook installed on 0x%llX via %s (id=%d). "
                         "INSTALLED IS NOT RUNNING -- watch for the first-call line.",
                         (unsigned long long)target, s_resolution, id);
    return true;
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
