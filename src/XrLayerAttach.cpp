// XrLayerAttach -- see XrLayerAttach.hpp for why UEVR's static linking makes this necessary.

#include "XrLayerAttach.hpp"

#include "addrcascade/AddressCascade.hpp"
#include "uevr/API.hpp"

#include <Windows.h>

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>

using uevr::API;

namespace halo {
namespace {

void alog(const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    _vsnprintf_s(buf, sizeof(buf), _TRUNCATE, fmt, ap);
    va_end(ap);
    API::get()->log_info("[Halo-CampE-UEVR] XRATTACH: %s", buf);
}

// ---- dbghelp, loaded dynamically ------------------------------------------------------------
//
// LoadLibrary rather than linking dbghelp.lib, so neither build script has to change and the
// contributor/CI build -- which compiles none of this -- keeps its two-library link line.

struct SymbolInfoPacked {
    ULONG   SizeOfStruct;
    ULONG   TypeIndex;
    ULONG64 Reserved[2];
    ULONG   Index;
    ULONG   Size;
    ULONG64 ModBase;
    ULONG   Flags;
    ULONG64 Value;
    ULONG64 Address;
    ULONG   Register;
    ULONG   Scope;
    ULONG   Tag;
    ULONG   NameLen;
    ULONG   MaxNameLen;
    CHAR    Name[1];
};

using PFN_SymSetOptions    = DWORD (WINAPI*)(DWORD);
using PFN_SymInitializeW   = BOOL  (WINAPI*)(HANDLE, PCWSTR, BOOL);
using PFN_SymLoadModuleExW = DWORD64 (WINAPI*)(HANDLE, HANDLE, PCWSTR, PCWSTR, DWORD64, DWORD, void*, DWORD);
using PFN_SymFromName      = BOOL  (WINAPI*)(HANDLE, PCSTR, SymbolInfoPacked*);

constexpr DWORD SYMOPT_UNDNAME_       = 0x00000002;
constexpr DWORD SYMOPT_DEFERRED_LOADS_= 0x00000004;
constexpr DWORD SYMOPT_FAIL_CRITICAL_ = 0x00000200;   // no message boxes, ever

HMODULE            g_dbghelp = nullptr;
PFN_SymSetOptions    p_SymSetOptions    = nullptr;
PFN_SymInitializeW   p_SymInitializeW   = nullptr;
PFN_SymLoadModuleExW p_SymLoadModuleExW = nullptr;
PFN_SymFromName      p_SymFromName      = nullptr;

std::atomic<bool> g_ready{false};
std::atomic<bool> g_started{false};
std::atomic<bool> g_pdb_ok{false};
std::mutex        g_sym_lock;          // dbghelp is not thread-safe
char              g_status[256]        = "not started";

HMODULE  g_backend      = nullptr;
DWORD64  g_backend_base = 0;

// Locate UEVRBackend.dll in this process and remember its path, so the PDB is looked for beside it
// rather than on some ambient symbol path.
bool find_backend(wchar_t* out_path, size_t cap) {
    g_backend = GetModuleHandleW(L"UEVRBackend.dll");
    if (g_backend == nullptr) {
        alog("UEVRBackend.dll not found in this process -- cannot resolve UEVR's OpenXR entry points");
        return false;
    }
    if (GetModuleFileNameW(g_backend, out_path, (DWORD)cap) == 0) return false;
    g_backend_base = (DWORD64)(uintptr_t)g_backend;
    return true;
}

bool load_dbghelp() {
    if (g_dbghelp != nullptr) return true;
    g_dbghelp = LoadLibraryW(L"dbghelp.dll");
    if (g_dbghelp == nullptr) { alog("dbghelp.dll failed to load"); return false; }

    p_SymSetOptions    = (PFN_SymSetOptions)   GetProcAddress(g_dbghelp, "SymSetOptions");
    p_SymInitializeW   = (PFN_SymInitializeW)  GetProcAddress(g_dbghelp, "SymInitializeW");
    p_SymLoadModuleExW = (PFN_SymLoadModuleExW)GetProcAddress(g_dbghelp, "SymLoadModuleExW");
    p_SymFromName      = (PFN_SymFromName)     GetProcAddress(g_dbghelp, "SymFromName");

    if (!p_SymSetOptions || !p_SymInitializeW || !p_SymLoadModuleExW || !p_SymFromName) {
        alog("dbghelp.dll is missing an expected export");
        return false;
    }
    return true;
}

// Load UEVRBackend.pdb. Runs on the worker thread; takes seconds.
void load_symbols() {
    wchar_t backend_path[MAX_PATH]{};
    if (!find_backend(backend_path, MAX_PATH)) {
        strcpy_s(g_status, "UEVRBackend.dll not present");
        return;
    }
    if (!load_dbghelp()) {
        strcpy_s(g_status, "dbghelp unavailable");
        return;
    }

    // Search path = the folder UEVRBackend.dll came from, which is where its PDB ships.
    wchar_t search[MAX_PATH]{};
    wcscpy_s(search, backend_path);
    if (wchar_t* slash = wcsrchr(search, L'\\')) *slash = 0;

    p_SymSetOptions(SYMOPT_UNDNAME_ | SYMOPT_DEFERRED_LOADS_ | SYMOPT_FAIL_CRITICAL_);

    // SymInitialize FAILING IS NOT FATAL. UEVR installs its own crash handler and may already have
    // initialised dbghelp for this process, in which case a second call returns
    // ERROR_INVALID_PARAMETER and the right move is to carry on and use the existing session --
    // not to give up, and emphatically not to SymCleanup, which would tear down theirs.
    const HANDLE proc = GetCurrentProcess();
    if (!p_SymInitializeW(proc, search, FALSE)) {
        const DWORD e = GetLastError();
        alog("SymInitializeW returned false (err %lu) -- continuing on the assumption dbghelp is "
             "already initialised for this process (UEVR's crash handler does this)", e);
    }

    const DWORD64 base = p_SymLoadModuleExW(proc, nullptr, backend_path, nullptr,
                                            g_backend_base, 0, nullptr, 0);
    if (base == 0 && GetLastError() != ERROR_SUCCESS) {
        alog("SymLoadModuleExW failed (err %lu) -- is UEVRBackend.pdb beside the DLL?", GetLastError());
        strcpy_s(g_status, "PDB not loaded");
        return;
    }

    g_pdb_ok.store(true, std::memory_order_release);
    sprintf_s(g_status, "PDB loaded for UEVRBackend.dll @ %p", (void*)(uintptr_t)g_backend_base);
    alog("%s", g_status);
}

void* resolve_via_pdb(const char* symbol) {
    if (!g_pdb_ok.load(std::memory_order_acquire)) return nullptr;

    std::lock_guard<std::mutex> lk(g_sym_lock);

    alignas(8) uint8_t storage[sizeof(SymbolInfoPacked) + 512]{};
    auto* si = reinterpret_cast<SymbolInfoPacked*>(storage);
    si->SizeOfStruct = sizeof(SymbolInfoPacked);
    si->MaxNameLen   = 512;

    if (!p_SymFromName(GetCurrentProcess(), symbol, si)) return nullptr;
    if (si->Address == 0) return nullptr;

    // Sanity: the symbol must live INSIDE UEVRBackend.dll. dbghelp will happily answer from another
    // module if one exports the same name -- and openxr_loader.dll, which exports every one of
    // these, is loaded in this very process. Without this check the resolver could hand back the
    // exact wrong address the whole module exists to avoid, and it would look like a success.
    // addrcascade's guarded PE walk rather than psapi: it validates the headers before touching
    // them, it is already unit-tested standalone, and it keeps this file's link line at zero extra
    // libraries. Its header says it plainly -- hand-copying the six-line walk is how one copy ends
    // up without its guard.
    addrcascade::ModuleRange mr{};
    if (addrcascade::module_range((void*)(uintptr_t)g_backend_base, &mr)) {
        const uintptr_t lo = mr.base;
        const uintptr_t hi = mr.end;
        const uintptr_t a  = (uintptr_t)si->Address;
        if (a < lo || a >= hi) {
            alog("REJECTED %s at %p -- outside UEVRBackend.dll [%p,%p). That is almost certainly "
                 "openxr_loader.dll's export, which is the address that does not work here.",
                 symbol, (void*)a, (void*)lo, (void*)hi);
            return nullptr;
        }
    }
    return (void*)(uintptr_t)si->Address;
}

void* resolve_via_loader_export(const char* symbol) {
    HMODULE m = GetModuleHandleW(L"openxr_loader.dll");
    if (m == nullptr) return nullptr;
    return (void*)GetProcAddress(m, symbol);
}

}   // namespace

void xrattach_begin_async() {
    bool expected = false;
    if (!g_started.compare_exchange_strong(expected, true)) return;

    std::thread([] {
        load_symbols();
        g_ready.store(true, std::memory_order_release);
    }).detach();
}

bool xrattach_ready() { return g_ready.load(std::memory_order_acquire); }

const char* xrattach_status() { return g_status; }

void* xrattach_resolve(const char* symbol, XrAttachTier* out_tier) {
    if (out_tier != nullptr) *out_tier = XrAttachTier::None;
    if (symbol == nullptr) return nullptr;

#if HALO_VR_DEV
    if (void* p = resolve_via_pdb(symbol)) {
        if (out_tier != nullptr) *out_tier = XrAttachTier::BackendPdb;
        return p;
    }
#endif

    // The fallback that is correct for a dynamically-linking host and wrong for UEVR. It is tried
    // rather than skipped because "wrong for UEVR" is a fact about UEVR, not about this function --
    // but the caller is told which tier answered, so nothing downstream can mistake the two.
    if (void* p = resolve_via_loader_export(symbol)) {
        if (out_tier != nullptr) *out_tier = XrAttachTier::LoaderExport;
        return p;
    }
    return nullptr;
}

}   // namespace halo
