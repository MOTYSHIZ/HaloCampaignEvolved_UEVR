#pragma once

// THE CFG AS A HOOK THREAD MAY READ IT.
//
// load_config() (game thread) rebuilds g_cfg in place: it resets the whole struct to defaults and then
// re-parses every file, a few milliseconds during which g_cfg holds defaults and half-parsed values.
// The sim thread (Blam hooks), the XInput hook, the render callbacks and the XR submit thread read g_cfg
// concurrently, so a reload could hand them a frame of defaults: masters off, offsets zero.
//
// With stabilityfixes on, the reload is bracketed (cfg_reload_begin / cfg_reload_end). Just before the
// reset, a copy of g_cfg is taken; while the reload runs, a reader on any other thread gets that copy.
// Outside a reload, and on the reloading thread itself, readers get g_cfg as before, so values the game
// thread changes between reloads are seen exactly as before.
//
// The handshake that makes the switch exact: a reader counts itself on g_cfg BEFORE it looks at the
// switch, and the reload turns the switch on BEFORE it waits for that count to drain. Either the reload
// sees the reader and waits for it, or the reader sees the switch and takes the copy; no reader can be
// on g_cfg while it is reset. The copy is written only while no reader holds it.
//
// USE: CFG_HOOK_READ; as the first statement of a function that runs off the game thread. It declares a
// local const reference named g_cfg that shadows the global for the rest of the function, so the body
// reads unchanged.

#include "Config.hpp"

namespace halo {

class CfgRead {
public:
    CfgRead();
    ~CfgRead();
    CfgRead(const CfgRead&) = delete;
    CfgRead& operator=(const CfgRead&) = delete;
    const Config& get() const { return *m_cfg; }

private:
    const Config* m_cfg = nullptr;
    bool          m_snap = false;
};

#define CFG_HOOK_READ \
    const ::halo::CfgRead cfg_hook_read_guard_; \
    const ::halo::Config& g_cfg = cfg_hook_read_guard_.get()

// load_config, game thread, immediately before g_cfg is reset. Acts only with stabilityfixes on (read
// from g_cfg before the reset).
void cfg_reload_begin();

// The registry's features_apply, the last write of a reload: hook threads read g_cfg again.
void cfg_reload_end();

} // namespace halo
