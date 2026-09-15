#include "core/config/CfgRead.hpp"

#include "core/Services.hpp"
#include "uevr/API.hpp"

#include <Windows.h>

#include <atomic>

namespace halo {

namespace {
// All sequentially consistent (the default order): the handshake below depends on it.
std::atomic<int32_t> s_live{0};           // readers holding g_cfg itself
std::atomic<int32_t> s_snap_readers{0};   // readers holding the copy
std::atomic<bool>    s_redirect{false};   // a reload is rebuilding g_cfg: other threads take the copy
std::atomic<DWORD>   s_writer_tid{0};     // the reloading thread keeps reading g_cfg
bool                 s_began = false;     // game thread only
Config               s_copy{};            // g_cfg as it stood just before the reset

// Wait for a counter to reach zero, bounded. A reader that never finishes must not hang the game
// thread: past the bound the reload goes ahead and says so.
bool drain(const std::atomic<int32_t>& count, ULONGLONG bound_ms, ULONGLONG* waited_ms) {
    const ULONGLONG t0 = GetTickCount64();
    while (count.load() != 0) {
        if (GetTickCount64() - t0 >= bound_ms) { *waited_ms = GetTickCount64() - t0; return false; }
        SwitchToThread();
    }
    *waited_ms = GetTickCount64() - t0;
    return true;
}
}  // namespace

CfgRead::CfgRead() {
    for (;;) {
        s_live.fetch_add(1);
        if (!s_redirect.load() || GetCurrentThreadId() == s_writer_tid.load()) {
            m_cfg = &g_cfg;
            m_snap = false;
            return;
        }
        s_live.fetch_sub(1);
        s_snap_readers.fetch_add(1);
        if (s_redirect.load()) {
            m_cfg = &s_copy;
            m_snap = true;
            return;
        }
        // The reload finished between the two looks: read g_cfg after all.
        s_snap_readers.fetch_sub(1);
    }
}

CfgRead::~CfgRead() {
    if (m_snap) s_snap_readers.fetch_sub(1);
    else        s_live.fetch_sub(1);
}

void cfg_reload_begin() {
    if (!service_active(SVC_STABILITY)) return;
    ULONGLONG waited = 0;
    // The copy is rewritten only while nobody holds it (the previous reload's readers are long gone).
    const bool copy_free = drain(s_snap_readers, 100, &waited);
    if (copy_free) s_copy = g_cfg;
    s_writer_tid.store(GetCurrentThreadId());
    s_redirect.store(copy_free);
    s_began = true;
    if (!copy_free) {
        uevr::API::get()->log_info("[Halo-CampE-UEVR] CFGREAD: a reader held the cfg copy for %llu ms -- this reload "
                                   "is not bracketed", (unsigned long long)waited);
        return;
    }
    // From here every new reader on another thread takes the copy; wait for the ones already on g_cfg.
    // The reloading thread's own readers are not counted out: it holds none at this point.
    if (!drain(s_live, 100, &waited)) {
        uevr::API::get()->log_info("[Halo-CampE-UEVR] CFGREAD: a reader stayed on g_cfg for %llu ms -- the reload "
                                   "went ahead", (unsigned long long)waited);
    }
}

void cfg_reload_end() {
    if (!s_began) return;
    s_began = false;
    s_redirect.store(false);
}

} // namespace halo
