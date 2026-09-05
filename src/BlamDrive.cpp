#include "BlamDrive.hpp"

#include "Config.hpp"
#include "Math.hpp"
#include "MotionAimControl.hpp"
#include "AimConverge.hpp"
#include "AimDirect.hpp"
#include "addrcascade/AddressCascade.hpp"
#include "uevr/API.hpp"

#include <Windows.h>
#include <tlhelp32.h>   // thread enumeration for the off-thread TLS resolution below
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <intrin.h>

using namespace uevr;

namespace halo {
namespace {

// The orientation getter, dll+0x5A6AD0. Hooked purely to get onto the sim thread -- its return
// value is passed through untouched. See BlamDrive.hpp for why this function and not the setter.
// ADDR-HYGIENE: resolved -- find_getter_by_signature() locates this function by SHAPE at startup and
// refuses an ambiguous match; this constant is only the fallback when the scan finds nothing, and
// the watchdog below catches the case where the fallback is also wrong (installed but never called).
constexpr uintptr_t RVA_GET_ORIENTATION = 0x5A6AD0;

// Where _tls_index sat on the build these offsets were derived from. NO LONGER USED TO READ IT --
// the PE TLS directory is the canonical source and is exact on any build (see
// tls_index_from_headers). This is kept purely as an EXPECTATION: if the header points somewhere
// else, we are on a different build of the DLL, and therefore RVA_GET_ORIENTATION -- which has no
// canonical source and cannot be checked this way -- is not trustworthy either.
//
// That inference is the whole value of keeping it. Reading the TLS index properly fixed one silent
// failure and would have removed the only signal that the OTHER constant is stale; cross-checking
// the two keeps that signal while making the read exact.
// ADDR-HYGIENE: resolved -- the live value comes from the PE TLS directory via
// addrcascade::tls_index(); this is only the EXPECTATION it is compared against, so a build that
// differs says so out loud instead of reading through a stale address.
constexpr uintptr_t EXPECTED_TLS_INDEX_RVA = 0xD72730;

// Ceiling for a believable _tls_index. Windows tops out around 1088 TLS slots per process and a
// module's own index is single digits in practice, so this is deliberately far above anything
// real: it is here to catch UNRELATED DATA, not to police the exact value.
constexpr uint32_t MAX_PLAUSIBLE_TLS_INDEX = 4096;

// Fault-injection bits (blamfault). See the block comment on Config::blam_fault for what each one
// proves and why proving it matters. Named here so the effect sites read as intent rather than as
// magic numbers. Dev builds only -- every use is #if HALO_VR_DEV.
// ADDR-HYGIENE: structural -- these are fault-injection BITS, not addresses. The lint matches them
// only because one of the names contains "RVA"; nothing here points at the target binary.
constexpr int FAULT_HOOK_DEAD    = 0x001;
constexpr int FAULT_SIG_NONE     = 0x002;
constexpr int FAULT_SIG_AMBIG    = 0x004;
constexpr int FAULT_TLS_RVA_DIFF = 0x008;
constexpr int FAULT_GETTER_MOVED = 0x010;
constexpr int FAULT_TLS_UNREAD   = 0x020;
constexpr int FAULT_LOG_RERESOLVE = 0x040;
// 0x080 / 0x100 belong to AimDirect.cpp; 0x400 too. Kept in one numbering space so a single
// blamfault value can describe a fault anywhere in the aim stack.
constexpr int FAULT_LAYOUT_STUCK  = 0x200;
constexpr int FAULT_LAYOUT_SHIFTED = 0x800;

// âš ï¸ THE UNIT HERE IS blam_drive_tick() CALLS, AND IT IS NOT A FRAME.
//
// blam_drive_tick() is called from inside update()'s CONFIG-RELOAD block -- `if (tick -
// g_cfg_check_tick >= 64)` -- so it runs roughly once every TWO SECONDS, not once per tick and
// certainly not once per frame. Sizing these against "~32 Hz" (as an earlier revision of this
// comment did) inflates every one of them by ~64x: the watchdog then needs five MINUTES of live
// gameplay to fire, which is indistinguishable from never.
//
// Caught by the blamforcetier=2 fault-injection run on 2026-08-14, which is exactly the sort of
// thing that only shows up when you force the broken path to execute.
constexpr uint32_t TICK_SECONDS = 2;   // approximate period of one blam_drive_tick() call

// Calls of LIVE gameplay before declaring the hook address wrong (~10 s). Generous against the
// measurement: with gameplay already live, install -> first call lands in the SAME SECOND. The
// multi-second gaps seen elsewhere were the hook installing before the sim was ticking, which is
// what the stick-mode gate excludes.
constexpr uint32_t WATCHDOG_LIVE_TICKS = 5;

// How often to drop the cached record so the next sim-thread call re-resolves it (~20 s).
constexpr uint32_t RERESOLVE_TICKS = 10;

// Cooldown between TEB-walk attempts when it fails (~4 s). The walk is cheap once and ruinous in a
// loop; a build where it never resolves must not turn into a repeating hitch.
constexpr uint32_t TEB_SCAN_RETRY_TICKS = 2;

// ---------------------------------------------------------------- THE GETTER, BY SIGNATURE
// TIER 3: find the orientation getter by what it IS rather than by where it was.
//
//   40 53                        push rbx
//   48 81 EC ?? ?? ?? ??         sub  rsp, <frame>          <- wildcard: frame size moves
//   44 8B 0D ?? ?? ?? ??         mov  r9d, [rip+<disp>]     <- wildcard: that is _tls_index
//   4C 8B DA                     mov  r11, rdx
//   65 48 8B 04 25 58 00 00 00   mov  rax, gs:[58h]
//
// The two wildcards are exactly the operands that legitimately change between builds; the register
// allocation and instruction order -- the function's shape -- are what we match on.
//
// MEASURED on the 2026-07-29 build before shipping this: the bare `gs:[58]` TLS idiom occurs
// 6,888 times in .text, so anything shorter than this is useless as an anchor. This 28-byte
// pattern matches EXACTLY ONCE, at 0x5A6AD0 -- the getter itself.
//
// Uniqueness is therefore a REQUIREMENT, not a nicety: two matches means we cannot tell which is
// the getter, and a confidently-wrong hook is worse than none. Multiple matches fall back to the
// recorded RVA and say so.
constexpr unsigned char GETTER_SIG[] = {
    0x40, 0x53,
    0x48, 0x81, 0xEC, 0x00, 0x00, 0x00, 0x00,
    0x44, 0x8B, 0x0D, 0x00, 0x00, 0x00, 0x00,
    0x4C, 0x8B, 0xDA,
    0x65, 0x48, 0x8B, 0x04, 0x25, 0x58, 0x00, 0x00, 0x00,
};
constexpr char GETTER_MASK[] = "xx" "xxx????" "xxx????" "xxx" "xxxxxxxxx";
static_assert(sizeof(GETTER_SIG) == sizeof(GETTER_MASK) - 1, "signature and mask must pair up");

// The player control table: tls_block + 0xB8, record = table + index * 0x198, yaw at +0x94 and
// pitch at +0x98. Aim reconstructs as (cos p * cos y, -cos p * sin y, sin p) -- the Blam Y
// negation, which is why the yaw sign knob below exists.
// ADDR-HYGIENE: guarded -- resolve_control_record() only accepts a table whose entries pass a
// structural scan, and the layout guard then proves the yaw/pitch fields below actually hold this
// build's aim before a single write is armed. A wrong table cannot survive both.
constexpr uintptr_t OFF_CTL_TABLE  = 0xB8;
constexpr uintptr_t CTL_REC_STRIDE = 0x198;
// ADDR-HYGIENE: guarded -- layout_gate() holds the write off until this field's VALUE agrees with
// the game's own aim (see LAYOUT_TOLERANCE_DEG). Value, not motion: the likeliest patch failure is
// landing on the NEIGHBOURING field, which moves just as much and is completely wrong.
constexpr uintptr_t OFF_CTL_YAW    = 0x94;
// Pitch sits immediately after yaw. The write path reaches it as fp[1] rather than through this
// name; it exists so the layout guard and its log can say WHICH field it validated.
// ADDR-HYGIENE: guarded -- second independent witness in layout_gate(). An inserted struct member
// shifts yaw AND pitch, so requiring both to agree makes a coincidental pass vanishingly unlikely.
constexpr uintptr_t OFF_CTL_PITCH  = 0x98;

// The wrap the game itself applies after its own store: yaw is kept in [0,2pi). Named locally
// because it is a property of THIS record's encoding, not general maths. DEG2RAD comes from
// Math.hpp -- a second copy here would be one more constant to keep in step for no gain.
constexpr float CTL_YAW_WRAP = 6.28318531f;

struct Vec3f { float x, y, z; };

// Returning uintptr_t rather than void deliberately: the real function's return value is not
// modelled, and declaring void would let the compiler clobber rax on the way back out.
using GetOrientFn = uintptr_t (*)(uintptr_t handle, Vec3f* outA, Vec3f* outB);

GetOrientFn g_original = nullptr;
int         g_hook_id  = -1;
uintptr_t   g_sim_base = 0;
uint32_t    g_tls_index = 0;

// The resolved control record. Re-resolved when it goes bad AND on a slow timer -- see the
// re-resolve note in blam_drive_tick(). Steady state is a null check on a hot path.
std::atomic<uintptr_t> g_ctl_rec{0};

// Set by the hook the first time it is actually CALLED. The install log cannot prove the address
// was right; only this can. Read by the watchdog in blam_drive_tick().
std::atomic<bool> g_hook_ran{false};

// (The "announce only on change" state that used to live here is now addrcascade::TierReporter,
// held as a local static at the announce site.)

// What the install path actually resolved, kept so the self-test can report it rather than re-deriving
// it (and disagreeing with the code that matters through some subtle difference).
uintptr_t   g_getter_addr    = 0;
const char* g_getter_via     = "?";
unsigned    g_getter_matches = 0;
uintptr_t   g_tls_rva_used   = 0;

#if HALO_VR_DEV
// Index within the control table that resolve_control_record() settled on, and the table base --
// kept only so the report below can say WHICH record we own, which is the question in co-op.
int       g_ctl_index = -1;
uintptr_t g_ctl_table = 0;

// The last values WE wrote, so the next pass can tell "the record still holds our value" from
// "something else has written it since".
float g_last_written_y = 0.0f, g_last_written_p = 0.0f;
bool  g_have_written   = false;

// BLAMCTL REPORT -- the three quantities that actually settle a reticle-vs-shot disagreement,
// sampled at ONE instant on the sim thread:
//
//   prev     what the record held on entry, i.e. after whatever last touched it
//   ours     what we wrote on the previous pass
//   wrote    what we are writing now
//   ctlrot   ControlRotation right now, which is what the reticle is drawn from
//
// Read it as two independent checks:
//
//   drift = prev - ours.  Non-zero means the record is NOT exclusively ours -- either the game
//     rewrites it between our passes, or we resolved onto another player's record. Zero means we
//     own it and our value survives.
//
//   dYaw/dPitch = (record expressed back in UE degrees) - ControlRotation.  This is THE number:
//     the reticle follows ControlRotation and the shot follows the record, so a constant non-zero
//     here IS the offset, measured rather than estimated. Zero here with a visible offset on
//     screen means the divergence is downstream of both and the reticle geometry is the suspect.
//
// Rate-limited hard: this runs ~2600 times a second on the sim thread.
void blam_ctl_report(uintptr_t rec, float prev_y, float prev_p, float wrote_y, float wrote_p,
                     float want_yaw_deg, float want_pitch_deg) {
    const int every = g_cfg.blam_ctl_log;
    if (every <= 0) return;
    static std::atomic<uint32_t> n{0};
    if ((n.fetch_add(1, std::memory_order_relaxed) % (uint32_t)every) != 0) return;

    constexpr float RAD2DEG_L = 57.2957795f;
    // The record back in UE convention: undo the wrap and the Blam yaw negation, so it is directly
    // comparable with ControlRotation instead of needing mental arithmetic at read time.
    float rec_yaw_ue = -(prev_y * RAD2DEG_L);
    while (rec_yaw_ue >  180.0f) rec_yaw_ue -= 360.0f;
    while (rec_yaw_ue < -180.0f) rec_yaw_ue += 360.0f;
    const float rec_pitch_ue = prev_p * RAD2DEG_L;

    double cr_pitch = 0.0, cr_yaw = 0.0;
    const bool have_cr = read_control_rotation_hook(&cr_pitch, &cr_yaw);

    float d_yaw = have_cr ? (rec_yaw_ue - (float)cr_yaw) : 0.0f;
    while (d_yaw >  180.0f) d_yaw -= 360.0f;
    while (d_yaw < -180.0f) d_yaw += 360.0f;
    const float d_pitch = have_cr ? (rec_pitch_ue - (float)cr_pitch) : 0.0f;

    const float drift_y = g_have_written ? (prev_y - g_last_written_y) : 0.0f;
    const float drift_p = g_have_written ? (prev_p - g_last_written_p) : 0.0f;

    API::get()->log_info(
        "[Halo-CampE-UEVR] BLAMCTL idx=%d rec=0x%llX | want=(y%.2f,p%.2f) wrote=(y%.4f,p%.4f)rad "
        "| recNow=(y%.2f,p%.2f)deg-UE ctlrot=(y%.2f,p%.2f) D=(y%.2f,p%.2f) "
        "| drift=(%.4f,%.4f)rad own=%d",
        g_ctl_index, (unsigned long long)rec,
        want_yaw_deg, want_pitch_deg, wrote_y, wrote_p,
        rec_yaw_ue, rec_pitch_ue,
        have_cr ? (float)cr_yaw : 0.0f, have_cr ? (float)cr_pitch : 0.0f,
        d_yaw, d_pitch,
        drift_y, drift_p, (int)have_cr);
}

// Every populated slot in the control table, logged ONCE per resolve. In single player only two
// records are ever populated; in co-op the table holds every player, and "which index is me" is
// the question that decides whether the lowest-index heuristic is safe. Resolve-time only, so it
// costs nothing on the hot path.
void blam_ctl_dump_table(uintptr_t table, int chosen) {
    API::get()->log_info("[Halo-CampE-UEVR] BLAMCTL TABLE at 0x%llX (chose idx %d):",
                         (unsigned long long)table, chosen);
    for (int i = 0; i < 16; ++i) {
        const uintptr_t rec = table + (uintptr_t)i * CTL_REC_STRIDE + OFF_CTL_YAW;
        if (IsBadReadPtr((const void*)rec, 8)) continue;
        const float y = ((const float*)rec)[0], p = ((const float*)rec)[1];
        if (!std::isfinite(y) || !std::isfinite(p)) continue;
        if (y == 0.0f && p == 0.0f) continue;
        API::get()->log_info("[Halo-CampE-UEVR]   idx %2d: yaw=%.4f pitch=%.4f rad  (%.1f, %.1f deg)%s",
                             i, y, p, y * 57.2957795f, p * 57.2957795f, (i == chosen) ? "  <== ours" : "");
    }
}
#endif  // HALO_VR_DEV

bool read_ptr(uintptr_t p, uintptr_t* out) {
    if (p == 0 || IsBadReadPtr((const void*)p, sizeof(uintptr_t))) return false;
    *out = *(const uintptr_t*)p;
    return true;
}

// ------------------------------------------------------------------ MODULE IDENTITY
// _tls_index STRAIGHT OUT OF THE PE HEADERS, not from a recorded RVA.
//
// Every PE that uses thread-local storage carries an IMAGE_TLS_DIRECTORY, and its AddressOfIndex
// field points at that module's `_tls_index`. The loader relocates it, so at runtime it is simply
// a usable address. That makes this EXACT on every build of every variant of this DLL, with no
// scanning, no signature and nothing to re-derive after a patch.
//
// The value this replaces was a hand-recorded copy of the same thing: on the build these offsets
// came from, AddressOfIndex resolves to dll+0xD72730 -- byte for byte the constant that used to
// live here. Reading it from the header is not a new technique, it is the canonical source for a
// number we were transcribing.
bool tls_index_from_headers(uintptr_t base, uint32_t* out_index, uintptr_t* out_rva) {
    // The PE walk moved to addrcascade, where the identical code in log_module_identity and the
    // signature scan had also been copied. out_rva is logged, so two reports stay comparable.
    return addrcascade::tls_index((void*)base, out_index, out_rva);
}

// Scan the module's executable sections for GETTER_SIG. Returns the address only when the match is
// UNIQUE; *out_matches reports what was actually found so an ambiguous build says so out loud.
// One-shot at install: ~8 MB with a first-byte skip, tens of milliseconds, and never on a hot path.
//
// The scan itself now lives in addrcascade, which is where the identical PE-walk-and-match code was
// extracted to. This function is the Blam-specific part that is left: our signature, our fault bits,
// our "ambiguous is a failure" policy.
uintptr_t find_getter_by_signature(uintptr_t base, unsigned* out_matches) {
    const addrcascade::Signature sig{GETTER_SIG, GETTER_MASK, sizeof(GETTER_SIG)};
    const addrcascade::ScanResult hit = addrcascade::scan_signature((void*)base, sig);
    *out_matches = (unsigned)hit.matches;

#if HALO_VR_DEV
    // FAULT INJECTION: pretend this build's getter does not match the signature, or matches it more
    // than once. Both must end in the same place -- fall back to the recorded RVA and say so --
    // because a confidently-wrong hook is worse than no hook.
    if (addrcascade::fault(FAULT_SIG_NONE))  { *out_matches = 0; return 0; }
    if (addrcascade::fault(FAULT_SIG_AMBIG)) { *out_matches = 2; return 0; }
#endif
    // Ambiguous is a failure, not a coin toss -- see the note on the signature.
    return hit.unique() ? hit.address : 0;
}

// WHICH BUILD IS THIS? Logged once, because every field report so far has cost a round trip to
// establish it. SizeOfImage alone separates builds (the report that produced this work differed
// from ours by 0x9000); the CodeView GUID+age is the canonical identity the linker stamped, so a
// known-good list can be kept against it rather than against sizes that could collide.
void log_module_identity(uintptr_t base) {
    // Header parsing and the CodeView/RSDS decode moved to addrcascade; what stays here is our
    // wording. Keep the fields and their order stable -- field reports get compared against it.
    addrcascade::ModuleIdentity id{};
    if (!addrcascade::module_identity((void*)base, &id)) return;
    API::get()->log_info("[Halo-CampE-UEVR] BLAMDRIVE: sim module SizeOfImage=0x%X stamp=0x%08X "
                         "pdb=%s age=%u",
                         id.size_of_image, id.timestamp, id.pdb_guid, id.pdb_age);
}

// SIM THREAD ONLY. gs:[0x58] is the TEB's TLS pointer, so off the sim thread this resolves to zero
// or garbage.
//
// The record index is found STRUCTURALLY -- the lowest-index entry holding a plausible
// (yaw in [0,2pi), pitch in [-pi/2,pi/2]) pair. Live, only records 0 and 4 are populated and the
// rest are zero, so this lands on the player without depending on any other plugin state.
//
// An earlier version matched the record against the plugin's own aim sample instead. That failed
// SILENTLY: the sample defaults to (1,0,0) and is not guaranteed fresh on this path, so every
// candidate was rejected, nothing was written, and NOTHING WAS LOGGED. Hence the reason string and
// the unconditional failure log at the call site -- a resolver that can fail must say so.
// The table walk, factored out so BOTH ways of reaching a TLS block share one validated filter.
// A second, subtly-different copy of this is exactly how two resolution paths drift apart.
uintptr_t record_from_block(uintptr_t block, const char** why, int* out_index) {
    *why = "ok";
    *out_index = -1;
    uintptr_t table = 0;
    if (block == 0) { *why = "TLS block null"; return 0; }
    // Published for the navpoint hunt's TLS-graph walker (see BlamDrive.hpp). Both resolution
    // tiers funnel through here, so this is the one site that always knows the live block.
    g_sim_tls_block.store(block, std::memory_order_relaxed);
    if (!read_ptr(block + OFF_CTL_TABLE, &table) || table == 0) {
        *why = "control table null"; return 0;
    }

    for (int i = 0; i < 16; ++i) {
        const uintptr_t rec = table + (uintptr_t)i * CTL_REC_STRIDE + OFF_CTL_YAW;
        if (IsBadReadPtr((const void*)rec, 8)) continue;
        const float yaw = ((const float*)rec)[0], pitch = ((const float*)rec)[1];
        if (!std::isfinite(yaw) || !std::isfinite(pitch)) continue;
        if (yaw == 0.0f && pitch == 0.0f) continue;         // unpopulated slot
        if (yaw < -0.01f || yaw > 6.2932f) continue;        // not the [0,2pi) wrap
        if (pitch < -1.5808f || pitch > 1.5808f) continue;  // not a pitch
        // WHICH record we took, in RELEASE builds too. The lowest-index rule assumes the local
        // player is the first populated slot -- true in every solo session measured so far, but an
        // assumption all the same, and one that co-op or a differently-ordered table could break.
        // A report that states the index is the only way to find that out from the field.
        *out_index = i;
#if HALO_VR_DEV
        g_ctl_index = i;
        g_ctl_table = table;
        g_have_written = false;      // a new record: no prior write of ours to compare against
        blam_ctl_dump_table(table, i);
#endif
        return rec;
    }
    *why = "no populated record in the table";
    return 0;
}

// TIER 1 -- the sim thread's own TLS array. Free, but only correct when we are ON that thread.
uintptr_t resolve_control_record(const char** why, int* out_index) {
    *why = "ok";
    *out_index = -1;
    const uintptr_t tls_array = (uintptr_t)__readgsqword(0x58);
    if (tls_array == 0) { *why = "no TLS array (wrong thread?)"; return 0; }
    uintptr_t block = 0;
    if (!read_ptr(tls_array + (uintptr_t)g_tls_index * 8, &block) || block == 0) {
        *why = "TLS block null"; return 0;
    }
    return record_from_block(block, why, out_index);
}

// TIER 2 -- ANY thread's TLS array, read from outside it. No hook, no code address, no signature.
//
// `__readgsqword(0x58)` is CURRENT-THREAD-ONLY by definition: it reads this thread's TEB, so no
// number of samples taken from the frame path can ever reveal the sim thread's table. That fact
// was originally read as "this state is only reachable from the sim thread", which is what made an
// inline hook on a hardcoded code address load-bearing for RESOLUTION rather than merely for write
// timing -- and that address is the one thing left that rots on a game patch.
//
// A thread's TEB is reachable from outside via NtQueryInformationThread(ThreadBasicInformation),
// and TEB+0x58 is its TLS array. So: walk our own threads, index each array by the (canonical,
// header-derived) tls_index, and accept the first block whose control table passes the SAME
// plausibility filter tier 1 uses. Build-independent by construction.
//
// GAME THREAD ONLY, and rate-limited by the caller: this opens a toolhelp snapshot and a handle
// per thread. Cheap once (~a few hundred microseconds), ruinous at the 2600 Hz the hook runs at.
struct TEB_BASIC_INFO {
    LONG      ExitStatus;
    PVOID     TebBaseAddress;
    PVOID     UniqueProcess;
    PVOID     UniqueThread;
    ULONG_PTR AffinityMask;
    LONG      Priority;
    LONG      BasePriority;
};
using NtQueryInformationThread_t = LONG(NTAPI*)(HANDLE, ULONG, PVOID, ULONG, PULONG);

uintptr_t resolve_control_record_any_thread(const char** why, int* out_index, uint32_t* out_tid) {
    *why = "ok";
    *out_index = -1;
    if (out_tid != nullptr) *out_tid = 0;

    static NtQueryInformationThread_t s_nt = nullptr;
    if (s_nt == nullptr) {
        if (HMODULE nt = GetModuleHandleA("ntdll.dll")) {
            s_nt = (NtQueryInformationThread_t)GetProcAddress(nt, "NtQueryInformationThread");
        }
        if (s_nt == nullptr) { *why = "NtQueryInformationThread unavailable"; return 0; }
    }

    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    if (snap == INVALID_HANDLE_VALUE) { *why = "thread snapshot failed"; return 0; }

    const DWORD pid = GetCurrentProcessId();
    THREADENTRY32 te{};
    te.dwSize = sizeof(te);
    uintptr_t found = 0;

    if (Thread32First(snap, &te)) {
        do {
            if (te.th32OwnerProcessID != pid) continue;
            HANDLE th = OpenThread(THREAD_QUERY_INFORMATION, FALSE, te.th32ThreadID);
            if (th == nullptr) continue;

            TEB_BASIC_INFO tbi{};
            if (s_nt(th, 0 /*ThreadBasicInformation*/, &tbi, sizeof(tbi), nullptr) == 0
                && tbi.TebBaseAddress != nullptr) {
                uintptr_t tls_array = 0;
                if (read_ptr((uintptr_t)tbi.TebBaseAddress + 0x58, &tls_array) && tls_array != 0) {
                    uintptr_t block = 0;
                    if (read_ptr(tls_array + (uintptr_t)g_tls_index * 8, &block) && block != 0) {
                        const char* w2 = "?";
                        int idx = -1;
                        const uintptr_t rec = record_from_block(block, &w2, &idx);
                        if (rec != 0) {
                            found = rec;
                            *out_index = idx;
                            if (out_tid != nullptr) *out_tid = te.th32ThreadID;
                        }
                    }
                }
            }
            CloseHandle(th);
        } while (found == 0 && Thread32Next(snap, &te));
    }

    CloseHandle(snap);
    if (found == 0) *why = "no thread's TLS block held a plausible control table";
    return found;
}

// ONE-SHOT SELF-TEST -- runs EVERY resolver and reports what each one found, independently of which
// one the cascade is actually using.
//
// The point is that a cascade hides its own failures by design: a lower tier covering for a broken
// upper one looks exactly like everything working. Every tier here was verified on the ONE build it
// was written against, and the paths that only execute when something is broken had -- before this
// -- never executed at all. That is the same category of code as the `aim write live` line that
// lied for a whole session.
//
// The last line is the valuable one: two INDEPENDENT resolvers (the sim thread's own gs:[0x58] and
// an outside-in TEB walk) cross-checked against each other. Agreement is far stronger evidence than
// either passing its own sanity gate, because the two share no code below record_from_block().
//
// DEV BUILDS ONLY. This answers a question rather than playing the game, which is exactly the line
// DevTools.hpp draws -- and it walks every thread in the process with a toolhelp snapshot to do it,
// on the game thread. A one-shot OS call of unbounded duration in a VR frame path is a hitch, and a
// hitch here is nausea rather than a blemish.
//
// Users are covered by the FAILURE reports instead, which are event-driven and cost nothing on a
// healthy install: BUILD DIFFERS, OFFSET MISMATCH, getter MOVED, "signature did NOT resolve", the
// install-vs-running watchdog, and the telling absence of `hook RUNNING`. Those diagnose a broken
// build; this proves a healthy one, which is our question, not theirs.
//
// Body-wrapped with an empty stub so the call site needs no #if.
void run_selftest_once() {
#if HALO_VR_DEV
    static bool s_done = false;
    if (s_done) return;
    s_done = true;

    // Exercise the shared harness's two anchor scanners against OUR OWN module. They have no
    // consumer in the cascade yet, and shipping an unexercised scanner inside a library about not
    // shipping unexercised code would be self-defeating.
    {
        HMODULE self = nullptr;
        GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS
                           | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           (LPCSTR)&run_selftest_once, &self);
        if (self != nullptr) addrcascade::self_test((void*)self);
    }

    const bool tls_ok = (g_tls_rva_used == EXPECTED_TLS_INDEX_RVA);
    API::get()->log_info("[Halo-CampE-UEVR] BLAMSELFTEST tls-index : %u @ dll+0x%llX "
                         "(recorded dll+0x%llX) %s",
                         g_tls_index, (unsigned long long)g_tls_rva_used,
                         (unsigned long long)EXPECTED_TLS_INDEX_RVA,
                         tls_ok ? "AGREE" : "DIFFER -- this is not the build these offsets came from");

    const bool sig_ok = (g_getter_addr == g_sim_base + RVA_GET_ORIENTATION);
    API::get()->log_info("[Halo-CampE-UEVR] BLAMSELFTEST getter    : dll+0x%llX via %s, %u match(es) "
                         "(recorded dll+0x%llX) %s",
                         (unsigned long long)(g_getter_addr - g_sim_base), g_getter_via,
                         g_getter_matches, (unsigned long long)RVA_GET_ORIENTATION,
                         sig_ok ? "AGREE" : "DIFFER -- the function moved on this build");

    API::get()->log_info("[Halo-CampE-UEVR] BLAMSELFTEST hook      : installed=%d everCalled=%d",
                         (int)(g_hook_id >= 0), (int)g_hook_ran.load(std::memory_order_relaxed));

    // Tier 1's answer is whatever the live path has already cached; tier 2 is resolved fresh here.
    const uintptr_t live = g_ctl_rec.load(std::memory_order_relaxed);
    const char* why = "?";
    int idx = -1;
    uint32_t tid = 0;
    const uintptr_t teb = resolve_control_record_any_thread(&why, &idx, &tid);

    if (live != 0 && teb != 0) {
        API::get()->log_info("[Halo-CampE-UEVR] BLAMSELFTEST record    : live=0x%llX | TEB-scan=0x%llX "
                             "idx=%d tid=%u  %s",
                             (unsigned long long)live, (unsigned long long)teb, idx, tid,
                             (live == teb) ? "AGREE"
                                           : "DIFFER -- two resolvers disagree, do not trust either");
    } else {
        API::get()->log_info("[Halo-CampE-UEVR] BLAMSELFTEST record    : live=0x%llX | TEB-scan=%s "
                             "(%s)",
                             (unsigned long long)live,
                             teb ? "resolved" : "FAILED", why);
    }
#endif  // HALO_VR_DEV
}

// Minimal pass-through hook: call the original, then take the one action this file exists for.
// Deliberately nothing else on this path -- it runs ~2600 times a second on the sim thread, and a
// chatty hook on a hot path is how a diagnostic becomes a stutter.
uintptr_t hooked_get_orientation(uintptr_t handle, Vec3f* outA, Vec3f* outB) {
    // "INSTALLED" IS NOT "RUNNING", said once, from the only place that can prove it.
    //
    // register_inline_hook succeeds on whatever address it is handed, so the install log is not
    // evidence that this function is the orientation getter on this build -- a field report showed
    // `aim write live` for a whole session on a hook that was never once called. This line is the
    // difference, and its ABSENCE from a log is now the diagnosis.
    //
    // Affordable on a path that runs ~2600 times a second because the steady state is a relaxed
    // atomic load -- a plain mov on x86, no lock and no fence. The read-modify-write happens once.
#if HALO_VR_DEV
    // FAULT INJECTION (blamforcetier=2): behave exactly as a hook on the WRONG address does --
    // pass through, never announce, never write. The watchdog then rules the hook dead and tier 2
    // takes over, with tier 1 silent so the result is unambiguously tier 2's. This is the only way
    // to exercise the fallback on a machine where nothing is actually broken.
    if (g_cfg.blam_fault & FAULT_HOOK_DEAD) {
        return g_original ? g_original(handle, outA, outB) : 0;
    }
#endif

    if (!g_hook_ran.load(std::memory_order_relaxed)
        && !g_hook_ran.exchange(true, std::memory_order_relaxed)) {
        API::get()->log_info("[Halo-CampE-UEVR] BLAMDRIVE: hook RUNNING -- first sim-thread call "
                             "through the orientation getter. The address is correct for this build.");
    }

    const uintptr_t ret = g_original ? g_original(handle, outA, outB) : 0;
    drive_control_angles();
    return ret;
}

}  // namespace

// The sim's TLS block, for the navpoint hunt's off-thread walker (see BlamDrive.hpp).
std::atomic<uintptr_t> g_sim_tls_block{0};

// The resolved control record, READ-ONLY, for the AIMDIG probe (dev builds). AimDirect's rotator
// turned out to be reachable from the PlayerController by pointer derefs, which retired its
// watchpoint hunt; the open question is whether THIS record is reachable the same way. If it is,
// resolution stops needing the sim thread -- which is the only reason the getter hook and the
// TEB-walking tier 2 exist. Read-only on purpose: the resolvers stay the sole writers.
uintptr_t blam_control_record() { return g_ctl_rec.load(std::memory_order_relaxed); }

// Exported so BlamAim.cpp can use the same definition rather than keep its own RVA. See the note in
// BlamDrive.hpp for why that matters more than it looks.
bool sim_tls_index(uintptr_t module_base, uint32_t* out_index, uintptr_t* out_rva) {
    return tls_index_from_headers(module_base, out_index, out_rva);
}

// Shared body for both entry points. `off_thread` picks the RESOLVER only -- the gates, the sign
// handling and the write itself are identical, deliberately, so the fallback path cannot drift into
// behaving differently from the one it is standing in for.
// ===================================================================== LAYOUT GUARD
//
// The cascade above protects the CODE address -- the getter RVA, the TLS index. It says nothing
// about OFF_CTL_YAW/OFF_CTL_PITCH, the struct offsets we then write through. A patch that moves
// that field by four bytes defeats every check we have: the signature still finds the function, the
// record still resolves, IsBadWritePtr still passes, and reading back our own write still returns
// our own value. We would write into the neighbouring field, forever, silently.
//
// Two things that DON'T work, both tried on paper first:
//   - Read-back. It only proves the address is writable. We wrote the value; of course we read it.
//   - Comparing the record against ControlRotation while we drive. aimdirect assigns the local view
//     from the same intent, so the view follows our command whether or not this write is landing.
//     The pair masks exactly the failure we are hunting.
//
// What does work is the idiom AimDirect already proves: a field holding a quantity must MOVE WHEN
// THAT QUANTITY MOVES, sampled while WE ARE NOT DRIVING IT. So the write is held off until the
// record's yaw has been shown to track the game's own aim.
//
// Sampled only while aimdirect is NOT armed, because once it is, ControlRotation follows our intent
// and would legitimately diverge from a record we are deliberately not writing -- a false mismatch,
// and the cost of a false mismatch here is the player's aim.
//
// DEADLOCK AVOIDED DELIBERATELY: both subsystems need aim motion to arm, so aimdirect can win the
// race and close our sampling window for good. If that happens we arm ANYWAY and say the layout is
// unverified. Never let "could not test it" degrade into "refuse to work" -- that turns a
// diagnostic into an outage, which is the trade this whole lane exists to avoid.
enum class Layout { Unproven, Proven, Wrong, Unverified };
Layout g_layout = Layout::Unproven;

// ONE definition of the look-around threshold. It appears in the config below AND in the message
// that explains why the guard did not arm; two copies means the log eventually lies about the
// number, and that log line is the whole support story for this feature.
constexpr double LAYOUT_REF_DEG = 25.0;

// A mismatch has to REPEAT before it disables the write. The asymmetry is the point: a missed
// detection costs one wrong-address write, while a false positive costs the player all sim aim (and
// all co-op aim replication) for the session. The window is only guarded by aimdirect and the
// calibration hold, so one unlucky window -- some other path moving the view without moving the
// record -- must not be enough to condemn the build.
constexpr int LAYOUT_MISMATCH_STRIKES = 3;
int g_layout_strikes = 0;

// Sampling opportunities counted so far toward LAYOUT_SETTLE_SAMPLES. At namespace scope rather
// than a local static because blam_drive_tick() has to be able to RESET it: stick mode (cutscenes,
// vehicles, death) returns above layout_gate entirely, so the gate cannot notice it came and went.
// A level that STARTS in a cutscene is fine either way -- the counter simply never starts. The case
// this exists for is a cutscene that interrupts a measurement already in progress, where the
// counter would otherwise still be satisfied and we would sample the view teleport on the way out.
uint32_t g_layout_settle = 0;

// TOLERANCE, centred on MEASURED data from both sides rather than on a tidy-looking number.
//
//   healthy, settled          0.01 deg   (the two agree almost exactly when the aim is steady)
//   healthy, worst SAMPLE     4.2  deg   (fast automated stick sweep -- the two are read a tick
//                                         apart, so a hard swing shows up as a one-sample spike)
//   wrong field (+4 bytes)   82-94 deg   (measured with blamfault 0x800)
//
// The first pick of 5 deg sat at 84% of the healthy PEAK, which is far too close to a threshold
// whose false positive costs the player all sim aim. 15 deg is ~3.5x above the worst healthy sample
// and still ~5.5x below the failure it must catch -- the gap here is enormous, so there is no reason
// to run tight. Note the real defence against a transient is the CONSECUTIVE-sample rule in
// ValueAgreement, not the width of this number: a wrong field disagrees on every single sample.
constexpr double LAYOUT_TOLERANCE_DEG = 15.0;

// DURATION IS THE REAL DISCRIMINATOR, not magnitude. Measured 2026-08-15 in live co-op, the worst
// healthy SPIKE was 56.6 deg yaw / 31.8 pitch, against 82-94 for a genuinely wrong field -- only
// ~1.4x apart, which is nowhere near enough to decide on. But a wrong field disagrees on EVERY
// sample forever, while a healthy excursion is a transient of a swing or a teleporting view.
//
// So the verdict needs this many CONSECUTIVE out-of-tolerance samples. At the 1-in-32 throttle on a
// ~2600/s getter that is ~80 samples/s, so this is roughly seven seconds of CONTINUOUS
// disagreement. A broken build reaches it immediately and never recovers; no transient sustains it.
//
// MEASURED, and this is why the axis changed. Live play 2026-08-15:
//   worst SPIKE      63.9 deg healthy  vs  82-94 wrong  -> 1.3x, overlapping, useless to decide on
//   longest RUN      25 samples healthy vs unbounded    -> the actual discriminator
// Set at 600 rather than the 25 observed because the healthy spike GREW with playtime (56.6 -> 63.9
// across sessions), so run length plausibly grows too, and the trade is lopsided: raising this costs
// only detection latency (7 s instead of 3 to catch a genuinely wrong build, which nobody notices),
// while a false positive costs a player their aim mid-match. The CONFIRMED line reports the run
// actually seen, so this can be tightened later on evidence rather than nerve.
constexpr uint32_t LAYOUT_STRIKE_SAMPLES = 600;

// Sampling opportunities to discard before measuring, at ~80/s -- about two seconds of settled
// gameplay. See g_layout_settle and layout_gate(): the previous behaviour measured the SPAWN
// transient, which is the noisiest instant in the session and the one every threshold was
// accidentally being tuned against.
constexpr uint32_t LAYOUT_SETTLE_SAMPLES = 160;

// Yaw is the primary: it wraps, and it is what the player swings. Pitch is a second, independent
// witness on the SAME struct -- an inserted member shifts both, so requiring both to agree makes a
// coincidental pass vanishingly unlikely.
addrcascade::ValueAgreement g_layout_yaw{[] {
    addrcascade::ValueAgreement::Config c;
    c.tolerance                 = LAYOUT_TOLERANCE_DEG;
    c.reference_motion_required = LAYOUT_REF_DEG;   // the window must not be degenerate
    c.max_samples               = 1200;             // with the 1-in-32 throttle, ~15 s of sampling
    c.strikes_to_fail           = LAYOUT_STRIKE_SAMPLES;
    c.wrap                      = 360.0;            // or a 359->1 step reads as 358 of motion
    return c;
}()};

addrcascade::ValueAgreement g_layout_pitch{[] {
    addrcascade::ValueAgreement::Config c;
    c.tolerance                 = LAYOUT_TOLERANCE_DEG;
    c.reference_motion_required = 0.0;    // a player can turn without ever looking up or down
    c.max_samples               = 1200;
    c.strikes_to_fail           = LAYOUT_STRIKE_SAMPLES;
    c.wrap                      = 0.0;    // pitch is clamped to +-90, it does not wrap
    return c;
}()};

void layout_reset() {
    g_layout = Layout::Unproven;
    g_layout_yaw.reset();
    g_layout_pitch.reset();
    g_layout_strikes = 0;
}

bool layout_gate(uintptr_t rec, bool off_thread) {
    // SIM THREAD ONLY. The state below is deliberately not atomic, and the 1-in-32 throttle is sized
    // for the getter's ~2600 calls/sec -- neither survives being shared with the game thread. In
    // production the two callers are mutually exclusive (tier 2 only runs once the hook is ruled
    // dead, and then tier 1 never fires), but clearing a fault mid-session makes both live, so make
    // the exclusion explicit rather than depending on it. Tier 2 is already the degraded path.
    if (off_thread) return true;
    if (g_cfg.blam_layout == 0)        return true;    // escape hatch, and the A/B for this guard
    if (g_layout == Layout::Proven)    return true;
    if (g_layout == Layout::Unverified) return true;
    if (g_layout == Layout::Wrong)     return false;

    // Once aimdirect owns the view the comparison is no longer meaningful -- see above.
    if (aim_direct_ready()) {
        g_layout = Layout::Unverified;
        API::get()->log_info(
            "[Halo-CampE-UEVR] BLAMLAYOUT: could not verify the record layout before aimdirect "
            "armed (needs ~%.0f deg of look-around while neither driver is writing). Writing "
            "anyway -- the offsets are UNVERIFIED this session, not known bad.",
            LAYOUT_REF_DEG);
        return true;
    }
    // DEADLINE: "could not verify" must NEVER become "refuse to work".
    //
    // The rule was already written above for the aimdirect race, and then not applied to anything
    // else -- so every OTHER way of failing to validate held the write off forever. That is not a
    // safety check, it is an outage: the player loses sim aim and co-op replication because a
    // diagnostic could not run. It is also not hypothetical; a stale PlayerController pointer did
    // exactly this for ten minutes on 2026-08-15, and only a log line I happened to add found it.
    //
    // The asymmetry decides it. Failing to VERIFY costs an unverified write -- which is precisely
    // the behaviour that shipped in 0.2 and works. Failing to WRITE costs the feature. So after
    // ~90 s of gameplay without a verdict, arm anyway and say so. A genuinely wrong layout still
    // gets caught: Mismatch needs only ~3 s of sustained disagreement, far inside this deadline.
    {
        constexpr uint32_t GIVE_UP_CALLS = 2600u * 90u;   // getter runs ~2600/s
        static uint32_t s_trying = 0;
        if (++s_trying > GIVE_UP_CALLS) {
            g_layout = Layout::Unverified;
            API::get()->log_info(
                "[Halo-CampE-UEVR] BLAMLAYOUT: gave up trying to verify the record layout after "
                "~90 s of gameplay (yaw saw %.1f of %.0f deg of movement it needed). Writing anyway "
                "-- UNVERIFIED is not known-bad, and refusing to write would cost more than the "
                "check is worth. The lines above say what was blocking it.",
                g_layout_yaw.reference_motion(), LAYOUT_REF_DEG);
            return true;
        }
    }

    // NEVER SIT SILENT. Everything below can decline to sample, and while it declines the sim write
    // is HELD OFF -- so a silent decline is a feature that has switched itself off and told nobody.
    // That is the exact failure this whole lane exists to stop, and it happened here: the guard sat
    // mute for ten minutes with the write disabled and the log offering no reason. Say why, once
    // every ~30 s, naming the specific blocker.
    // SETTLE FIRST. Measured 2026-08-15: the guard confirmed 0.77 s after gameplay began, and 25 of
    // the ~62 samples in that window were out of tolerance -- because the window lands exactly on
    // SPAWN, where the view teleports and the two mirrors are furthest apart. That is the worst
    // possible moment to measure, and every threshold tuned against it was really being tuned
    // against a startup transient rather than against the thing being detected.
    //
    // So discard the first stretch of otherwise-good samples and measure settled play instead. This
    // counts SAMPLING OPPORTUNITIES, not wall time, so a menu or a cutscene simply pauses it rather
    // than burning it; it resets whenever something blocks sampling, so a transient that interrupts
    // us mid-measurement is not sampled on the way out either.

    auto stalled = [](const char* why) -> bool {
        g_layout_settle = 0;   // whatever blocked us will teleport the view again on its way out
        static uint32_t s_n = 0;
        static const char* s_last = nullptr;
        // Report on the first tick, whenever the REASON changes, and then rarely.
        if (why != s_last || (s_n % 2400) == 0) {
            s_last = why;
            API::get()->log_info("[Halo-CampE-UEVR] BLAMLAYOUT: not validating yet -- %s. The sim "
                                 "write stays HELD OFF until this clears (blamlayout=0 disables the "
                                 "check entirely).", why);
        }
        ++s_n;
        return false;
    };

    if (g_aim_calibrating.load(std::memory_order_relaxed)) return stalled("aim calibration is holding every driver silent");

    // The sim thread runs this ~2600 times a second. Sampling every call would reach the sample cap
    // in a fraction of a second, across no player motion at all, and decide nothing.
    static uint32_t s_throttle = 0;
    if ((++s_throttle & 31u) != 0) return false;

    double cr_pitch = 0.0, cr_yaw = 0.0;
    if (!read_control_rotation_hook(&cr_pitch, &cr_yaw)) {
        return stalled("ControlRotation is unreadable from here, so there is nothing to compare the "
                       "record against");
    }

    const float* src = (const float*)rec;   // [0] = yaw, [1] = pitch, in Blam radians
#if HALO_VR_DEV
    // FAULT INJECTION (blamfault 0x800): read ONE FIELD LATE, which is what a patch that inserts a
    // member into this struct actually does to us. This is the realistic failure and the one the
    // old motion-only check could not see: the neighbouring field moves just as much as the one we
    // wanted, so "does it move?" says yes while the value is completely wrong.
    if (addrcascade::fault(FAULT_LAYOUT_SHIFTED)) src = src + 1;
#endif

    // Same transform blam_ctl_report uses, so the two read consistently: undo the Blam yaw negation
    // and express the record in UE degrees.
    constexpr float RAD2DEG_L = 57.2957795f;
    float rec_yaw_ue = -(src[0] * RAD2DEG_L);
    while (rec_yaw_ue >  180.0f) rec_yaw_ue -= 360.0f;
    while (rec_yaw_ue < -180.0f) rec_yaw_ue += 360.0f;
    const float rec_pitch_ue = src[1] * RAD2DEG_L;

#if HALO_VR_DEV
    // FAULT INJECTION (blamfault 0x200): freeze the candidate -- the offset landing on something
    // DEAD. Kept alongside 0x800 because the two failures look nothing alike from in here.
    if (addrcascade::fault(FAULT_LAYOUT_STUCK)) rec_yaw_ue = 0.0f;
#endif

    // Discard the settling stretch -- see g_layout_settle. The write stays held off meanwhile,
    // which is the same state it was already in, so this costs nothing but a couple of seconds.
    if (++g_layout_settle <= LAYOUT_SETTLE_SAMPLES) return false;

    // Both witnesses are sampled every time, so neither can be starved by the other short-circuiting.
    const auto vy = g_layout_yaw.sample(rec_yaw_ue, cr_yaw);
    const auto vp = g_layout_pitch.sample(rec_pitch_ue, cr_pitch);

    // Combine: either witness disagreeing is a mismatch (an inserted member moves BOTH fields, so a
    // real shift shows up in whichever one the player happened to exercise). Confirmation needs both
    // -- pitch has no motion requirement, so on its own it could pass on a window that proved little.
    addrcascade::ValueAgreement::Verdict verdict;
    if (vy == addrcascade::ValueAgreement::Verdict::Mismatch ||
        vp == addrcascade::ValueAgreement::Verdict::Mismatch) {
        verdict = addrcascade::ValueAgreement::Verdict::Mismatch;
    } else if (vy == addrcascade::ValueAgreement::Verdict::Match &&
               vp == addrcascade::ValueAgreement::Verdict::Match) {
        verdict = addrcascade::ValueAgreement::Verdict::Match;
    } else if (vy == addrcascade::ValueAgreement::Verdict::Inconclusive) {
        verdict = addrcascade::ValueAgreement::Verdict::Inconclusive;
    } else {
        verdict = addrcascade::ValueAgreement::Verdict::Pending;
    }

    switch (verdict) {
    case addrcascade::ValueAgreement::Verdict::Match:
        g_layout = Layout::Proven;
        API::get()->log_info(
            "[Halo-CampE-UEVR] BLAMLAYOUT: record layout CONFIRMED -- yaw at +0x%llX AGREED with the "
            "game's own aim over %.1f deg of movement (worst spike yaw %.1f / pitch %.1f deg, "
            "longest disagreement RUN yaw %u / pitch %u of %u allowed). Arming the sim write.",
            (unsigned long long)OFF_CTL_YAW, g_layout_yaw.reference_motion(),
            g_layout_yaw.worst_error(), g_layout_pitch.worst_error(),
            g_layout_yaw.worst_run(), g_layout_pitch.worst_run(), LAYOUT_STRIKE_SAMPLES);
        return true;

    case addrcascade::ValueAgreement::Verdict::Mismatch:
        // Strike, not a conviction -- see LAYOUT_MISMATCH_STRIKES. The write stays held off while we
        // re-measure, so retrying costs nothing that refusing would not have cost anyway.
        if (++g_layout_strikes < LAYOUT_MISMATCH_STRIKES) {
            API::get()->log_info(
                "[Halo-CampE-UEVR] BLAMLAYOUT: the field at +0x%llX disagreed with the aim by up to "
                "%.1f deg this window (pitch by %.1f) -- strike %d of %d, re-measuring before "
                "condemning the layout.",
                (unsigned long long)OFF_CTL_YAW, g_layout_yaw.worst_error(),
                g_layout_pitch.worst_error(), g_layout_strikes, LAYOUT_MISMATCH_STRIKES);
            g_layout_yaw.reset();
            g_layout_pitch.reset();
            return false;
        }
        g_layout = Layout::Wrong;
        API::get()->log_info(
            "[Halo-CampE-UEVR] BLAMLAYOUT: WRONG OFFSETS -- the field at +0x%llX disagreed with the "
            "game's own aim by up to %.1f deg (pitch at +0x%llX by %.1f), across %.1f deg of "
            "movement. Those offsets are not this build's aim, so the sim write is DISABLED rather "
            "than corrupting whatever now lives there. Almost certainly a patched or different-store "
            "binary -- aim falls back to the stick loop. Set blamlayout=0 to override this check.",
            (unsigned long long)OFF_CTL_YAW, g_layout_yaw.worst_error(),
            (unsigned long long)OFF_CTL_PITCH, g_layout_pitch.worst_error(),
            g_layout_yaw.reference_motion());
        return false;

    case addrcascade::ValueAgreement::Verdict::Inconclusive:
        // The player stood still. That is not evidence of anything -- start another window rather
        // than condemning a layout that simply had nothing to prove itself against.
        g_layout_yaw.reset();
        g_layout_pitch.reset();
        return false;

    case addrcascade::ValueAgreement::Verdict::Pending:
    default: {
        // Same rule as above: the write is held off while this is Pending, so say so periodically
        // with the numbers that explain WHAT it is waiting for. "Needs more look-around" is
        // actionable; silence is not.
        static uint32_t s_pending = 0;
        if ((s_pending++ % 1200) == 0) {
            API::get()->log_info(
                "[Halo-CampE-UEVR] BLAMLAYOUT: still measuring -- %.1f of %.0f deg of aim movement "
                "seen, worst disagreement so far yaw %.2f / pitch %.2f deg (tolerance %.1f). Sim "
                "write held off until this resolves.",
                g_layout_yaw.reference_motion(), LAYOUT_REF_DEG,
                g_layout_yaw.worst_error(), g_layout_pitch.worst_error(), LAYOUT_TOLERANCE_DEG);
        }
        return false;
    }
    }
}

static void drive_angles_impl(bool off_thread) {
    if (g_cfg.blam_angles == 0) return;

    // HOLD OFF DURING AIM CALIBRATION (Page Down). That gesture works by silencing every aim
    // driver so the reticle stands still while the player points the controller at it. This write
    // is one of those drivers, so it observes the same hold -- otherwise the aim keeps tracking the
    // hand, the reticle never freezes, and there is nothing to calibrate against.
    if (g_aim_calibrating.load(std::memory_order_relaxed)) return;

    // HOLD OFF IN STICK MODE (vehicle seats, cutscenes, death). Halo binds the vehicle chase camera
    // to the aim, so a motion-driven aim swings the whole camera -- which is the entire reason
    // stick mode exists. The control law is disarmed at the stick-mode gate in Plugin.cpp, but this
    // write is driven from the sim's orientation getter and is NOT on that code path, so without
    // this it kept steering the seat camera from the hand while the player's stick did nothing.
    if (g_stick_mode_active.load(std::memory_order_relaxed)) return;

    uintptr_t rec = g_ctl_rec.load(std::memory_order_relaxed);
    if (rec == 0 || IsBadWritePtr((void*)rec, 8)) {
        const char* why = "?";
        int index = -1;
        uint32_t tid = 0;
        if (off_thread) {
            // RATE-LIMIT THE TEB WALK. It opens a toolhelp snapshot and a handle per thread. Once
            // the record resolves this never runs again -- but on a build where it CANNOT resolve
            // it would otherwise run every single frame, which is a hitch loop, and a hitch in VR
            // is nausea rather than a blemish. Retry every ~2 s instead.
            static uint32_t s_cooldown = 0;
            if (s_cooldown != 0) { --s_cooldown; return; }
            s_cooldown = TEB_SCAN_RETRY_TICKS;
            rec = resolve_control_record_any_thread(&why, &index, &tid);
        } else {
            rec = resolve_control_record(&why, &index);
        }
        if (rec != 0) {
            g_ctl_rec.store(rec, std::memory_order_relaxed);
            // Announce only when the ANSWER CHANGED. The slow re-resolve in blam_drive_tick()
            // would otherwise reprint this every ~19 s, and a line that always appears carries no
            // information -- which is the failure this whole lane was built to stop repeating.
            //
            // WHICH TIER WON is part of the line on purpose: a cascade that does not say how it
            // resolved will happily run a whole population on its fallback with nobody the wiser.
            // addrcascade::TierReporter holds the "announce on CHANGE" rule, keyed on the pair
            // (address, tier) -- so a silent switch from tier 1 to tier 2 at the SAME address still
            // reports, which a plain address comparison would have swallowed.
            static addrcascade::TierReporter s_reporter;
            const char* tier = off_thread ? "TEB scan" : "sim thread gs:[0x58]";
            if (s_reporter.changed(rec, tier)) {
                char via[64];
                if (off_thread) _snprintf_s(via, sizeof(via), _TRUNCATE, "TEB scan, tid %u", tid);
                else            _snprintf_s(via, sizeof(via), _TRUNCATE, "sim thread gs:[0x58]");
                API::get()->log_info("[Halo-CampE-UEVR] BLAMCTL: control record resolved at 0x%llX "
                                     "(index %d, yaw +0x94, pitch +0x98) via %s",
                                     (unsigned long long)rec, index, via);
            }
        } else {
            // NEVER fail silently here -- see the note on resolve_control_record().
            static std::atomic<uint32_t> flog{0};
            const uint32_t fn = flog.fetch_add(1, std::memory_order_relaxed);
            if (fn < 3 || (fn % 8000) == 0) {
                API::get()->log_info("[Halo-CampE-UEVR] BLAMCTL: resolve FAILED (%s) -- blamangles "
                                     "is on but nothing is being written", why);
            }
            return;
        }
    }
    if (IsBadWritePtr((void*)rec, 8)) return;

    float* fp = (float*)rec;
#if HALO_VR_DEV
    // Sampled BEFORE our write: this is what the record holds after whatever last touched it, which
    // is the only way to tell our value surviving from something else owning the slot.
    const float prev_y = fp[0], prev_p = fp[1];
#endif
    if (g_cfg.blam_angles == 3) {
        // Absolute probe: pin a distinctive angle. If this is the authoritative input the view
        // snaps there and stays, regardless of the stick. PINS THE VIEW -- not a play setting.
        //
        // DELIBERATELY ABOVE THE LAYOUT GATE. The probe's whole job is to answer "is this record
        // authoritative?", which is the same question the gate answers by a slower route -- and it
        // is used with no controllers attached, so it would never generate the aim motion the gate
        // waits for and would simply appear broken.
        fp[0] = 1.50f; fp[1] = 0.30f;
        return;
    }

    // LAYOUT GUARD -- after the probe, before any real aim write. See layout_gate().
    if (!layout_gate(rec, off_thread)) return;

    float yaw = 0.0f, pitch = 0.0f;
    if (!desired_aim_now(&yaw, &pitch)) return;

    // 6DoF CONVERGENCE. desired_aim_now() returns the INTENT -- where the player is pointing --
    // which is a direction and therefore only lands on the target when the shot leaves from the
    // player's eye. It does not: it leaves from Blam's own origin. Bending the intent onto the
    // traced range is what makes the two agree. Declines to act while the head is leashed, so this
    // is a no-op in the shipped configuration. See AimConverge.hpp.
    aim_converge_apply(&yaw, &pitch);

    // YAW SIGN. desired_aim_now() returns UE-convention degrees, but this record stores BLAM yaw,
    // which is its negation -- the game derives the aim as (cos p * cos y, -cos p * sin y, sin p).
    //
    // Confirmed live in co-op, and the failure was diagnostic in itself: with no flip the LOCAL
    // view was correct (direct drive owns that independently) while the HOST saw yaw mirrored --
    // and pitch was right on both sides, which is exactly what a Y-only negation predicts, since
    // z = sin(pitch) is identical in both conventions. That asymmetry is also what proved this
    // record is the outbound replication source.
    const float asign = (g_cfg.blam_angles_ysign >= 0) ? 1.0f : -1.0f;
    float ry = asign * (yaw + g_cfg.blam_yaw_off) * DEG2RAD;
    const float rp = (pitch + g_cfg.blam_pitch_off) * DEG2RAD;
    // The game stores yaw wrapped into [0,2pi) and re-wraps after its own store, so match that or
    // the value reads as out of range.
    ry -= std::floor(ry / CTL_YAW_WRAP) * CTL_YAW_WRAP;
    fp[0] = ry; fp[1] = rp;

#if HALO_VR_DEV
    blam_ctl_report(rec, prev_y, prev_p, ry, rp, yaw, pitch);
    g_last_written_y = ry; g_last_written_p = rp; g_have_written = true;
#endif
}

// TIER 1 entry: called from the hook, on the sim thread, ~2600 times a second.
void drive_control_angles() { drive_angles_impl(/*off_thread=*/false); }

// TIER 2 entry: called from the GAME THREAD, and only once the watchdog has established that the
// hook is never going to fire on this build. Costs a toolhelp snapshot on the frames where the
// record is not yet resolved, which is why it must never be reachable from the hook.
void blam_drive_offthread_write() { drive_angles_impl(/*off_thread=*/true); }

void blam_drive_tick() {
    const bool want = (g_cfg.blam_angles != 0);

    // Hand the shared harness its log sink (once) and the live fault mask (every poll, so a dev can
    // flip a bit in the cfg and see the effect on the next ~2 s reload without a restart).
    {
        static bool s_logger_set = false;
        if (!s_logger_set) {
            s_logger_set = true;
            addrcascade::set_logger([](const char* m) { API::get()->log_info("%s", m); });
        }
        addrcascade::set_fault_mask(g_cfg.blam_fault);
    }

    // STICK MODE RESTARTS THE SETTLE, but only while the layout is still undecided. Cutscenes,
    // vehicles and death return above layout_gate entirely, so the gate itself cannot tell that one
    // came and went -- it would resume sampling straight into the view teleport on the way out,
    // which is the same transient the settle exists to skip. Costs nothing once Proven, because the
    // gate returns before any of this.
    if (g_layout != Layout::Proven && g_layout != Layout::Unverified
        && g_stick_mode_active.load(std::memory_order_relaxed)) {
        g_layout_settle = 0;
    }

    // NOTE the layout guard is deliberately NOT reset by the periodic re-resolve below. The record
    // ADDRESS changes on a level load; the struct LAYOUT is a property of the binary and cannot.
    // Re-validating on every re-resolve would hold the write off for a fresh look-around every ~20 s
    // -- a recurring aim dropout, caused entirely by the safety check.
#if HALO_VR_DEV
    // Which leaves the guard untestable without a reboot, so: re-arm it whenever the fault mask
    // CHANGES. Dev-only, and edge-triggered so it costs nothing on an unchanging config.
    {
        static int s_prev_fault = 0;
        if (g_cfg.blam_fault != s_prev_fault) {
            s_prev_fault = g_cfg.blam_fault;
            layout_reset();
        }
    }
#endif

    // THE CANONICAL PAIRING IS blamangles=1 WITH aimdirect=1 -- see Config.hpp on either key. Split
    // them and the closed loop is still steering the aim toward a setpoint that this file is also
    // assigning exactly; the two fight, and it reads as heavy jitter rather than as a setting being
    // wrong. That is a miserable thing to diagnose from feel, so say it. Edge-triggered, because
    // config is re-read every ~2 s and a per-poll line would be its own problem.
    {
        static int prev = -1;
        const int state = (want ? 2 : 0) | (g_cfg.aim_direct ? 1 : 0);
        if (state != prev) {
            prev = state;
            if (state == 2) {
                API::get()->log_info("[Halo-CampE-UEVR] BLAMDRIVE: blamangles=1 with aimdirect=0 -- "
                                     "expect jitter. These two are one setting in two halves; set "
                                     "aimdirect=1 unless you are deliberately A/Bing them.");
            } else if (state == 1) {
                API::get()->log_info("[Halo-CampE-UEVR] BLAMDRIVE: aimdirect=1 with blamangles=0 -- "
                                     "local aim only. Other players will not see where you point.");
            }
        }
    }

#if HALO_VR_DEV
    // Ownership of dll+0x5A6AD0, read from config rather than from a runtime claim -- see the
    // block in BlamDrive.hpp for the race that made the handshake version unusable. Both this and
    // blam_aim_tick() evaluate the same value before either touches the address, so there is no
    // window in which both are installed.
    const bool yield = (g_cfg.blam_aim != 0);
#else
    constexpr bool yield = false;
#endif

    if (!want || yield) {
        if (g_hook_id >= 0) {
            API::get()->param()->functions->unregister_inline_hook(g_hook_id);
            g_hook_id  = -1;
            g_original = nullptr;
            g_ctl_rec.store(0, std::memory_order_relaxed);
            API::get()->log_info("[Halo-CampE-UEVR] BLAMDRIVE: hook removed (%s)",
                                 want ? "dev diagnostic hook owns the address" : "blamangles=0");
        }
        return;
    }

    if (g_hook_id >= 0) {
        static uint32_t s_frames = 0;
        ++s_frames;

        // WATCHDOG -- "installed" is not "running", now enforced rather than merely logged.
        //
        // register_inline_hook succeeds on any readable address, so a stale RVA installs cleanly
        // onto a function the game never calls and the log says `aim write live` all session. That
        // is precisely the field failure this lane was opened for. Counting only CALLS where the
        // motion stack is armed keeps it honest: before stick mode releases the sim may not be
        // ticking at all, and a warning there would be a false alarm every single launch.
        //
        // The counter unit is blam_drive_tick() calls (~2 s each), NOT frames -- see TICK_SECONDS.
        //
        // COUNT ONLY ACTUAL GAMEPLAY, or this condemns a perfectly good address. Injection happens
        // at the MAIN MENU by design, and the sim calls the orientation getter neither there nor
        // during a level load. Measured on a healthy build: hook installed 10:46:21.6, gameplay
        // began 10:46:42.9, first hook call 10:46:44.3 -- so counting from INSTALLATION fired the
        // alarm at 10:46:31.4, eleven seconds before the game it was judging even existed, and it
        // did so on every launch.
        //
        // That is not merely a wrong log line. s_warned is load-bearing below: it switches on the
        // tier-2 game-thread writer, so a false positive silently puts a SECOND writer on a healthy
        // install -- breaking the "a HEALTHY install can never reach it" guarantee stated there,
        // and in a codebase whose hard rule is that two aim drivers must never run at once.
        //
        // g_frontend_active DEFAULTS TO TRUE, so the count cannot start before gameplay is
        // established; resetting it (rather than merely pausing) stops a level change from
        // accumulating a stale partial count across the gap. Healthy margin: the hook proves itself
        // ~1.3 s into gameplay against a ~10 s budget, and stick mode covers the load/transition
        // window where the frontend has gone but the sim has not taken over yet.
        //
        // FRONTEND, NOT "menu". The first version of this gate used g_menu_active, which also counts
        // an open in-game menu WIDGET -- and measured 2026-08-15, the sim keeps calling this very
        // hook while such a widget is open (observed running with inmenu=1 for three minutes). That
        // gate stopped the clock during time in which the watched call was entirely possible, so on
        // a genuinely broken build the alarm would have been slow or silent. Over-correcting a false
        // positive into a watchdog that cannot fire just moves the failure, it does not fix it.
        // The counting rule itself lives in addrcascade::HookWatchdog, whose tick() REQUIRES the
        // "was this even possible?" argument -- the lesson above, made structural so the next
        // watchdog in this codebase cannot quietly omit it.
        static addrcascade::HookWatchdog s_watchdog{WATCHDOG_LIVE_TICKS};
        const bool sim_should_be_calling =
            !g_frontend_active.load(std::memory_order_relaxed)
            && !g_stick_mode_active.load(std::memory_order_relaxed);
        if (s_watchdog.tick(sim_should_be_calling, g_hook_ran.load(std::memory_order_relaxed))) {
            API::get()->log_info(
                "[Halo-CampE-UEVR] BLAMDRIVE: hook installed at dll+0x%llX but NEVER CALLED "
                "after ~%u s of live gameplay -- that address is not the orientation "
                "getter on this build. The sim's aim will not follow your hand and will not "
                "replicate to other players. Almost certainly a different game build (patch, "
                "or a different store's binary).",
                (unsigned long long)RVA_GET_ORIENTATION, s_watchdog.counted() * TICK_SECONDS);
        }
        const bool s_warned = s_watchdog.dead();

        // SELF-TEST, once there is something meaningful to say: either tier 1 has produced a record
        // (so both resolvers can be cross-checked) or the watchdog has ruled the hook dead (so the
        // report explains why there is nothing to cross-check against).
        if (g_ctl_rec.load(std::memory_order_relaxed) != 0 || s_warned) run_selftest_once();

        // TIER 2 TAKES OVER once the watchdog has ruled the hook dead.
        //
        // This is the whole point of the cascade: on a build where the getter RVA is stale the hook
        // never fires, so nothing ever resolves and nothing is ever written -- the field failure.
        // Here the record is resolved by walking threads instead (no code address involved) and the
        // write is issued from the game thread. Gated on s_warned so a HEALTHY install can never
        // reach it: no snapshot cost, no second writer, no behaviour change whatsoever.
        if (s_warned) blam_drive_offthread_write();

        // SLOW RE-RESOLVE of the control record.
        //
        // The old comment claimed re-resolution happens "when it goes bad (level load frees the
        // table)" -- but freed heap stays mapped and writable, so IsBadWritePtr passes and a stale
        // pointer keeps accepting writes forever, with no symptom except that nothing moves. The
        // rig driver already learned this and re-resolves on a timer ALWAYS: "a pointer to a
        // recycled component never becomes null ... can pin the driver to residue" (Plugin.cpp).
        // Cheap: the resolve is a 16-entry structural scan, and the announce is change-gated.
        if ((s_frames % RERESOLVE_TICKS) == 0) {
            g_ctl_rec.store(0, std::memory_order_relaxed);
#if HALO_VR_DEV
            // The re-resolve is deliberately SILENT (the announce is change-gated), which also
            // makes it unverifiable. This bit exists so it can be observed at least once.
            if (g_cfg.blam_fault & FAULT_LOG_RERESOLVE) {
                API::get()->log_info("[Halo-CampE-UEVR] BLAMDRIVE: periodic re-resolve fired "
                                     "(call %u) -- record dropped, next sim-thread call re-finds it",
                                     s_frames);
            }
#endif
        }
        return;
    }

    // OFFSETS ALREADY REJECTED FOR THIS BUILD -- latched on purpose.
    //
    // Clearing g_cfg.blam_angles is not enough by itself: load_config() re-reads the file every
    // ~2 s and restores whatever it says, so the next tick would walk straight back in here, fail
    // identically, and log again for the rest of the session. The latch is what makes this one
    // readable verdict instead of a scrolling wall, and the offsets cannot change mid-session
    // anyway -- the module is already loaded.
    static bool s_offsets_rejected = false;
#if HALO_VR_DEV
    // A fault-mask CHANGE clears the latch, so the hard-fail path can be re-tested without a
    // restart. The latch exists to stop a retry loop, not to outlast a deliberate experiment.
    {
        static int s_prev_fault = 0;
        if (s_prev_fault != g_cfg.blam_fault) {
            s_prev_fault       = g_cfg.blam_fault;
            s_offsets_rejected = false;
        }
    }
#endif
    if (s_offsets_rejected) return;

    HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
    if (sim == nullptr) {
        // Not an error while the game is still loading -- the sim DLL arrives late. Rate-limited so
        // a genuinely missing module is still visible without filling the log during startup.
        static std::atomic<uint32_t> mlog{0};
        const uint32_t mn = mlog.fetch_add(1, std::memory_order_relaxed);
        if (mn == 0 || (mn % 600) == 0) {
            API::get()->log_info("[Halo-CampE-UEVR] BLAMDRIVE: HaloSimulation_tag_release.dll not "
                                 "loaded yet -- aim write idle");
        }
        return;
    }

    g_sim_base = (uintptr_t)sim;

    // SAY WHICH BUILD THIS IS, THEN RESOLVE WHAT CAN BE RESOLVED EXACTLY.
    //
    // Every constant in this file was derived from ONE build of this DLL. A game patch moves them;
    // a different store's binary is not the same file at all. None of that is detectable
    // downstream -- register_inline_hook takes an ADDRESS, not a symbol, so it succeeds on
    // whatever now lives at RVA_GET_ORIENTATION and reports success.
    //
    // FIELD-OBSERVED, and the reason any of this exists: a report came in with tlsIndex=3192455160
    // -- unrelated data read through a stale RVA -- on an image 0x9000 larger than ours. The hook
    // "installed" onto a function that is never called, the log said `aim write live` for the
    // entire session, and nothing was ever written. The player's aim stopped replicating and their
    // movement stopped following their turning, while every line in the log said the driver was
    // healthy. A write that silently does not happen must never again be indistinguishable from
    // one that does.
    //
    // So: _tls_index now comes from the PE TLS directory, which is canonical and exact on any
    // build. Where it landed then judges the constant that has no canonical source.
    log_module_identity(g_sim_base);

    uint32_t  tls_index = 0;
    uintptr_t tls_rva   = 0;
    bool tls_ok = tls_index_from_headers(g_sim_base, &tls_index, &tls_rva);
#if HALO_VR_DEV
    // FAULT INJECTION: a build whose PE has no usable TLS directory (hard stop), or one where the
    // index simply lives somewhere else (warn, keep going -- the read is still exact).
    if (g_cfg.blam_fault & FAULT_TLS_UNREAD)   { tls_ok = false; tls_index = 0; }
    if (g_cfg.blam_fault & FAULT_TLS_RVA_DIFF) { tls_rva ^= 0x1000; }
#endif
    if (!tls_ok || tls_index >= MAX_PLAUSIBLE_TLS_INDEX) {
        // Hard stop: nothing downstream can resolve without a usable index, and guessing one is
        // how a write ends up at an arbitrary address.
        API::get()->log_info(
            "[Halo-CampE-UEVR] BLAMDRIVE: could not read _tls_index from the PE TLS directory "
            "(got %u) -- the sim aim write is DISABLED rather than aimed at a guessed address.",
            tls_index);
        API::get()->log_info(
            "[Halo-CampE-UEVR] BLAMDRIVE: expect aim NOT to replicate to other players, and "
            "movement direction NOT to follow snap turn. Workaround: set movelive=1 in "
            "halo_vr.cfg -- it measures the aim the game actually has instead of the one we asked "
            "for, so movement keeps following the view.");
        g_cfg.blam_angles  = 0;
        s_offsets_rejected = true;
        return;
    }
    g_tls_index    = tls_index;
    g_tls_rva_used = tls_rva;   // kept for the self-test, so it reports what was actually used

    // BUILD DRIFT. The header just told us where _tls_index really lives; if that is not where it
    // lived on the build these constants came from, this is a different binary and the getter RVA
    // below has no such canonical source to be checked against.
    //
    // WARN, DO NOT DISABLE. The hook is a pass-through -- landing it on the wrong function wastes
    // the call, it does not corrupt anything -- the TLS index is now exact, and the record
    // resolver validates what it finds. A patch that moves one constant may well leave the other
    // alone, so refusing to try would break users we could still have served. What we must not do
    // is stay quiet about it, which is exactly what happened before.
    if (tls_rva != EXPECTED_TLS_INDEX_RVA) {
        API::get()->log_info(
            "[Halo-CampE-UEVR] BLAMDRIVE: BUILD DIFFERS -- _tls_index is at dll+0x%llX, expected "
            "dll+0x%llX. This is not the build these Blam offsets came from (game patch, or a "
            "different store's binary), so the orientation-getter address may be wrong too. "
            "Continuing anyway; the `hook RUNNING` line below says whether it actually was.",
            (unsigned long long)tls_rva, (unsigned long long)EXPECTED_TLS_INDEX_RVA);
    }

    // TIER 3 -- locate the getter by SIGNATURE; the recorded RVA is demoted to an expectation.
    //
    // Scan primary, constant as assertion: on a build that matches, the two agree and this is a
    // free self-test. On a patched or different-store binary the scan adapts where the constant
    // cannot, and a disagreement is the loudest possible statement that the build moved.
    unsigned    sig_matches = 0;
    const uintptr_t scanned = find_getter_by_signature(g_sim_base, &sig_matches);
    uintptr_t expected = g_sim_base + RVA_GET_ORIENTATION;
#if HALO_VR_DEV
    // FAULT INJECTION: move the EXPECTATION, not the scan result. That makes the "getter MOVED"
    // report fire while the genuinely-correct address is still what gets hooked -- so this proves
    // the reporting without ever pointing a hook at the wrong code.
    if (g_cfg.blam_fault & FAULT_GETTER_MOVED) expected += 0x1000;
#endif

    uintptr_t   getter = 0;
    const char* via    = nullptr;
    if (scanned != 0) {
        getter = scanned;
        via    = "signature";
        if (scanned != expected) {
            API::get()->log_info(
                "[Halo-CampE-UEVR] BLAMDRIVE: getter MOVED -- signature found it at dll+0x%llX, "
                "the recorded RVA says dll+0x%llX. Trusting the signature: it describes the "
                "function, the constant only describes where it used to be.",
                (unsigned long long)(scanned - g_sim_base),
                (unsigned long long)(expected - g_sim_base));   // what we actually compared against
        }
    } else {
        getter = expected;
        via    = "recorded RVA";
        API::get()->log_info(
            "[Halo-CampE-UEVR] BLAMDRIVE: getter signature did NOT resolve (%u matches, need "
            "exactly 1) -- falling back to the recorded dll+0x%llX. If the hook then never runs, "
            "this build has moved the function and the signature needs re-deriving.",
            sig_matches, (unsigned long long)RVA_GET_ORIENTATION);
    }

    g_getter_addr    = getter;        // recorded for the self-test rather than re-derived there
    g_getter_via     = via;
    g_getter_matches = sig_matches;

    void* target = (void*)getter;
    const int id = API::get()->param()->functions->register_inline_hook(
        target, (void*)&hooked_get_orientation, (void**)&g_original);
    if (id < 0 || g_original == nullptr) {
        // Fail LOUD and stay off. A half-installed hook that silently does nothing is the failure
        // mode this whole lane kept hitting; better to say the aim write is unavailable.
        API::get()->log_info("[Halo-CampE-UEVR] BLAMDRIVE: register_inline_hook FAILED (id=%d) on "
                             "0x%llX -- motion aim will not reach the sim",
                             id, (unsigned long long)target);
        // LATCH LOCALLY, do not clear g_cfg.blam_angles. load_config() now resets the whole struct
        // every ~2 s, so a config field can no longer carry a runtime decision -- the file value
        // would come straight back and this would become a retry-and-log loop for the session.
        s_offsets_rejected = true;
        return;
    }

    g_hook_id = id;
    // The RVA printed here is the RESOLVED one, not the recorded constant -- otherwise two reports
    // from different builds would show the same number while pointing at different code.
    API::get()->log_info("[Halo-CampE-UEVR] BLAMDRIVE: installed on 0x%llX (dll+0x%llX, via %s) "
                         "id=%d tlsIndex=%u -- aim write live",
                         (unsigned long long)target, (unsigned long long)(getter - g_sim_base),
                         via, id, g_tls_index);
}


// ---- GRENADE STATE: DECLARED, DELIBERATELY NOT POPULATED IN THIS TREE.
//
// Holster.cpp links against these to draw the chest pouches. In blindcowboy24 PR-1 they are
// filled from raw offsets into the Blam unit object (u8[0x380/0x382/0x383]) -- hardcoded struct
// offsets carrying no ADDR-HYGIENE marker and no addrcascade guard. That plumbing was NOT taken
// with this extraction, which is the WEAPON SWITCHING only.
//
// g_unit_gvalid therefore stays false for ever, and that is the fail-closed answer: Holster.cpp
// reads it as "counts unknown", so the pouches stay empty rather than inventing a grenade.
// Populate these from a GUARDED offset and the pouches light up with no other change.
//
// At namespace-halo scope on purpose -- inside the anonymous namespace above they would be
// file-local and Holster.obj would not link.
std::atomic<int>  g_unit_gtype{0}, g_unit_gfrag{0}, g_unit_gplasma{0};
std::atomic<bool> g_unit_gvalid{false};
}  // namespace halo
