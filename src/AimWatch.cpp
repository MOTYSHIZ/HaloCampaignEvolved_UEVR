// Hardware write-watchpoint. See AimWatch.hpp for why this exists and why it is bounded.

#include "AimWatch.hpp"

#if HALO_VR_DEV

#include "Config.hpp"
#include "MotionAimControl.hpp"
#include "UeObject.hpp"
#include "uevr/API.hpp"

#include <Windows.h>
#include <TlHelp32.h>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

using namespace uevr;

namespace halo {
namespace {

constexpr int MAX_HITS = 32;  // room for noise; the interesting writer is filtered out of it below

PVOID              g_veh      = nullptr;
std::atomic<bool>  g_armed{false};
std::atomic<int>   g_hit_count{0};
uintptr_t          g_watch_addr = 0;

// Distinct faulting instruction addresses seen so far. Tiny fixed array: the handler runs inside an
// exception and must not allocate.
std::atomic<uintptr_t> g_hits[MAX_HITS]{};

// The register state at the trap, captured alongside the instruction address.
//
// Knowing WHERE the write happens is only half the answer. Disassembly of this game's writer shows
// it is a straight copy -- `vmovups xmm0,[rbx]` then `vmovups [rdi+0x350],xmm0` -- so ControlRotation
// is not computed here at all, it is duplicated FROM [rbx]. That source pointer is the thing worth
// having, and it exists only in the trap context. Recording just RIP threw it away.
std::atomic<uintptr_t> g_hit_rbx[MAX_HITS]{};
std::atomic<uintptr_t> g_hit_rdi[MAX_HITS]{};
std::atomic<uintptr_t> g_hit_rsi[MAX_HITS]{};
std::atomic<uintptr_t> g_hit_rcx[MAX_HITS]{};

bool g_prev_flag = false;   // tick thread only
int  g_settle_ticks = 0;    // let a few more writers accumulate before reporting

// Dr7 layout for slot 0: L0 (bit 0) enables it locally; RW0 (bits 16-17) = 01 for "break on write";
// LEN0 (bits 18-19) encodes the width -- 00=1 byte, 01=2, 11=4, 10=8.
uint64_t dr7_for(int lenBytes) {
    uint64_t dr7 = 1ull;                 // L0
    dr7 |= (0b01ull << 16);              // RW0 = write-only
    uint64_t lenBits = 0b00;
    if (lenBytes == 2) lenBits = 0b01;
    else if (lenBytes == 4) lenBits = 0b11;
    else if (lenBytes == 8) lenBits = 0b10;
    dr7 |= (lenBits << 18);              // LEN0
    return dr7;
}

// Debug registers are per-thread, so EVERY thread has to be armed. The thread that stamps the aim
// is not necessarily the one calling this, and arming only our own thread would produce a confident
// "nothing writes here".
int g_armed_threads = 0;   // reported, so "no hits" can be told apart from "nothing was armed"

// THE CALLING THREAD MUST BE ARMED TOO -- it is the one that matters.
//
// The first version skipped it, reasoning that SuspendThread on yourself deadlocks. True, but this
// runs from the plugin TICK, which is the GAME THREAD, and the game thread is precisely what stamps
// ControlRotation. Excluding it armed every thread except the writer, and produced a confident
// "0 distinct writers" that looked like evidence no such write exists. Even a deliberate test write
// through MCP went untrapped, because that is marshalled to the game thread as well.
//
// Self is armed WITHOUT suspending: setting your own debug registers via Get/SetThreadContext is
// well-defined for the DR block specifically (the general-purpose registers of a running thread are
// not, but we neither read nor write those).
void arm_one(HANDLE th, uintptr_t addr, bool is_self) {
    if (!is_self && SuspendThread(th) == (DWORD)-1) return;
    CONTEXT c{};
    c.ContextFlags = CONTEXT_DEBUG_REGISTERS;
    if (GetThreadContext(th, &c)) {
        c.Dr0 = addr;
        c.Dr7 = addr ? dr7_for(8) : 0;   // addr 0 = disarm
        c.Dr6 = 0;
        c.ContextFlags = CONTEXT_DEBUG_REGISTERS;
        if (SetThreadContext(th, &c) && addr) ++g_armed_threads;
    }
    if (!is_self) ResumeThread(th);
}

void for_each_thread(uintptr_t addr) {
    const DWORD pid = GetCurrentProcessId();
    const DWORD self = GetCurrentThreadId();

    // Self first, so an early failure enumerating the others still leaves the writer armed.
    arm_one(GetCurrentThread(), addr, true);

    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    if (snap == INVALID_HANDLE_VALUE) return;
    THREADENTRY32 te{};
    te.dwSize = sizeof(te);
    if (Thread32First(snap, &te)) {
        do {
            if (te.th32OwnerProcessID != pid) continue;
            if (te.th32ThreadID == self) continue;   // already done, and suspending self deadlocks
            HANDLE th = OpenThread(THREAD_GET_CONTEXT | THREAD_SET_CONTEXT | THREAD_SUSPEND_RESUME,
                                   FALSE, te.th32ThreadID);
            if (th == nullptr) continue;
            arm_one(th, addr, false);
            CloseHandle(th);
        } while (Thread32Next(snap, &te));
    }
    CloseHandle(snap);
}

LONG CALLBACK veh(EXCEPTION_POINTERS* ep) {
    if (ep == nullptr || ep->ExceptionRecord == nullptr) return EXCEPTION_CONTINUE_SEARCH;
    if (ep->ExceptionRecord->ExceptionCode != EXCEPTION_SINGLE_STEP) return EXCEPTION_CONTINUE_SEARCH;
    if (!g_armed.load(std::memory_order_relaxed)) return EXCEPTION_CONTINUE_SEARCH;

    // Bit 0 of Dr6 = our slot fired. Anything else is somebody else's single-step and must be
    // passed on untouched.
    if ((ep->ContextRecord->Dr6 & 0x1ull) == 0) return EXCEPTION_CONTINUE_SEARCH;
    ep->ContextRecord->Dr6 = 0;

    const uintptr_t rip = (uintptr_t)ep->ContextRecord->Rip;

    // ONLY RECORD WRITES WHOSE SOURCE POINTER IS PLAUSIBLE.
    //
    // The watched address sits in a live UObject, so ordinary memory traffic -- allocator fill,
    // memcpy during construction, unrelated struct copies -- also lands on it. Unfiltered, that
    // noise filled every slot within seconds of arming and the real per-frame writer never got
    // one, which reads exactly like "the write does not happen".
    //
    // The writer we care about copies the rotator FROM a pointer (vmovups xmm0,[rbx]), so rbx must
    // be a canonical, aligned user-mode address. Junk values (0x1, 0xF, 0x2F) are rejected for free.
    // Deliberately a REGISTER test only -- dereferencing inside a vectored handler risks a nested
    // fault, so the actual read happens later, on the tick.
    const uintptr_t rbx = (uintptr_t)ep->ContextRecord->Rbx;
    const bool plausible_src = (rbx >= 0x10000ull) && (rbx < 0x7FFFFFFFFFFFull) && ((rbx & 7ull) == 0);
    if (!plausible_src) return EXCEPTION_CONTINUE_EXECUTION;

    for (int i = 0; i < MAX_HITS; ++i) {
        uintptr_t cur = g_hits[i].load(std::memory_order_relaxed);
        if (cur == rip) return EXCEPTION_CONTINUE_EXECUTION;    // already recorded
        if (cur == 0) {
            uintptr_t expected = 0;
            if (g_hits[i].compare_exchange_strong(expected, rip)) {
                g_hit_rbx[i].store((uintptr_t)ep->ContextRecord->Rbx, std::memory_order_relaxed);
                g_hit_rdi[i].store((uintptr_t)ep->ContextRecord->Rdi, std::memory_order_relaxed);
                g_hit_rsi[i].store((uintptr_t)ep->ContextRecord->Rsi, std::memory_order_relaxed);
                g_hit_rcx[i].store((uintptr_t)ep->ContextRecord->Rcx, std::memory_order_relaxed);
                g_hit_count.fetch_add(1, std::memory_order_relaxed);
            }
            return EXCEPTION_CONTINUE_EXECUTION;
        }
    }
    return EXCEPTION_CONTINUE_EXECUTION;
}

void report_and_disarm() {
    // Module-relative addresses, because absolute ones move with ASLR and are useless in a note.
    HMODULE mods[256]; DWORD needed = 0;
    char exeName[MAX_PATH] = {0};
    GetModuleFileNameA(nullptr, exeName, MAX_PATH);
    const uintptr_t exeBase = (uintptr_t)GetModuleHandleA(nullptr);

    for (int i = 0; i < MAX_HITS; ++i) {
        const uintptr_t rip = g_hits[i].load();
        if (rip == 0) continue;
        MEMORY_BASIC_INFORMATION mbi{};
        uintptr_t base = 0;
        char modName[MAX_PATH] = {0};
        if (VirtualQuery((LPCVOID)rip, &mbi, sizeof(mbi)) != 0) {
            base = (uintptr_t)mbi.AllocationBase;
            GetModuleFileNameA((HMODULE)mbi.AllocationBase, modName, MAX_PATH);
        }
        const char* leaf = strrchr(modName, '\\');
        API::get()->log_info("[Halo-CampE-UEVR] AIMWATCH writer #%d: RIP=0x%llX  module=%s+0x%llX",
                             i, (unsigned long long)rip,
                             leaf ? leaf + 1 : "?",
                             (unsigned long long)(base ? rip - base : 0));
        API::get()->log_info("[Halo-CampE-UEVR]   regs: rbx=0x%llX rdi=0x%llX rsi=0x%llX rcx=0x%llX",
                             (unsigned long long)g_hit_rbx[i].load(),
                             (unsigned long long)g_hit_rdi[i].load(),
                             (unsigned long long)g_hit_rsi[i].load(),
                             (unsigned long long)g_hit_rcx[i].load());
        // The source operand, read back. If this matches the aim we can see through
        // ControlRotation, [rbx] IS the authoritative store rather than another mirror.
        {
            const uintptr_t src = g_hit_rbx[i].load();
            if (src != 0 && !IsBadReadPtr((const void*)src, sizeof(double) * 3)) {
                const double* r = (const double*)src;
                API::get()->log_info("[Halo-CampE-UEVR]   [rbx] = pitch %.4f  yaw %.4f  roll %.4f",
                                     r[0], r[1], r[2]);
            }
        }
    }
    (void)mods; (void)needed; (void)exeBase; (void)exeName;

    for_each_thread(0);
    g_armed = false;
    if (g_veh != nullptr) { RemoveVectoredExceptionHandler(g_veh); g_veh = nullptr; }
    API::get()->log_info("[Halo-CampE-UEVR] AIMWATCH disarmed: %d distinct writers, %d threads had been armed",
                         g_hit_count.load(), g_armed_threads);
}

} // namespace

void aim_watch_tick() {
    // Report and tear down once enough distinct writers have been seen. Done from the TICK, never
    // from the exception handler -- logging inside a VEH on the game thread is how a diagnostic
    // turns into a deadlock.
    if (g_armed.load() && g_hit_count.load() >= 1 && ++g_settle_ticks > 6) {
        report_and_disarm();
        return;
    }

    const bool want = g_cfg.aim_watch;

    // NEVER ARM OUTSIDE GAMEPLAY.
    //
    // The flag lives in a config file, so it survives a restart -- and the first version of this
    // armed on the 0->1 edge wherever that happened to land, which after any crash-and-relaunch was
    // DURING LEVEL LOAD. Suspending every thread and writing debug registers while the game streams
    // a level killed it on three consecutive launches, and looked convincingly like the UEVR build
    // being wrong rather than the diagnostic being at fault.
    //
    // Holding the edge until gameplay costs nothing: the aim is stamped every frame once we are
    // there, so there is no early write to miss.
    // A STABLE CONTROL ROTATION, which is what this actually needs.
    //
    // Not a reflection call: class_name_of() on the PlayerController is exactly what this file must
    // never do, because UEVR's reflection ACCESS-VIOLATES on this game's Blam objects (it is why
    // aim is read by raw memory at a validated offset at all).
    //
    // Nor g_aim_law_armed, which was the first attempt: that additionally requires live controller
    // POSES, so with the controllers idle -- headset put down, player away -- it never becomes true
    // and the watchpoint silently never arms. The aim is still stamped every frame regardless, so
    // requiring the aim loop to be driving was gating on something irrelevant to the measurement.
    //
    // Consecutive successful reads mean a PlayerController exists and is stable, which is false
    // during a level load (the window that crashed the game) and true in gameplay whether or not
    // anyone is holding a controller.
    static int stable = 0;
    {
        double p = 0.0, y = 0.0; void* pc = nullptr;
        if (read_control_rotation(&p, &y, &pc) && pc != nullptr) {
            if (stable < 1000) ++stable;
        } else {
            stable = 0;
        }
    }
    if (want && stable < 15) return;   // deliberately does NOT latch g_prev_flag -- retry later

    if (want == g_prev_flag) return;
    g_prev_flag = want;

    if (!want) {
        if (g_armed.load()) report_and_disarm();
        return;
    }
    if (g_armed.exchange(true)) return;

    uintptr_t addr = (uintptr_t)g_cfg.aim_watch_addr;
    if (addr == 0) {
        void* pc = nullptr;
        double p = 0.0, y = 0.0;
        if (!read_control_rotation(&p, &y, &pc) || pc == nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] AIMWATCH: no player controller yet");
            g_armed = false;
            return;
        }
        // Yaw specifically: rot[1]. Watching the whole rotator would also trap the pitch write and
        // muddy which instruction we are looking at.
        addr = (uintptr_t)pc + CONTROL_ROTATION_OFFSET + sizeof(double);
    }
    g_watch_addr = addr;
    for (int i = 0; i < MAX_HITS; ++i) g_hits[i] = 0;
    g_hit_count = 0;
    g_settle_ticks = 0;

    g_veh = AddVectoredExceptionHandler(1, veh);
    if (g_veh == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] AIMWATCH: could not install exception handler");
        g_armed = false;
        return;
    }
    g_armed_threads = 0;
    for_each_thread(addr);
    API::get()->log_info("[Halo-CampE-UEVR] AIMWATCH armed on 0x%llX (ControlRotation.Yaw) across %d threads",
                         (unsigned long long)addr, g_armed_threads);
}

void aim_watch_shutdown() {
    if (g_armed.load()) {
        for_each_thread(0);
        g_armed = false;
    }
    if (g_veh != nullptr) { RemoveVectoredExceptionHandler(g_veh); g_veh = nullptr; }
}

} // namespace halo

#endif // HALO_VR_DEV
