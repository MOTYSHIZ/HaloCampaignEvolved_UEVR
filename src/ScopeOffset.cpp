#include "ScopeOffset.hpp"

#include "Config.hpp"
#include "Rig.hpp"
#include "UeObject.hpp"
#include "WeaponCalib.hpp"

#include <atomic>
#include <cstring>
#include <string>

using uevr::API;

namespace halo {
namespace {

// The global scope fit exactly as the FILE supplies it, before any per-weapon delta.
struct Base {
    float zoom = 0.0f;
    float dist = 0.0f, right = 0.0f, up = 0.0f;
    float rot_p = 0.0f, rot_y = 0.0f, rot_r = 0.0f;
    bool  valid = false;
};
Base s_base;

// The load generation the base was taken from. NOT a tick counter -- see the long note in
// scope_offset_update() for why that distinction is the whole correctness argument here.
uint32_t s_last_gen = 0xFFFFFFFFu;
std::wstring s_last_weapon;

// Armed by the Script UI (calib:wpnscope) or scopewpnarm. Consumed by the capture.
std::atomic<bool> s_armed{false};

void publish_base() {
    g_cfg.scope_base_zoom  = s_base.zoom;
    g_cfg.scope_base_dist  = s_base.dist;
    g_cfg.scope_base_right = s_base.right;
    g_cfg.scope_base_up    = s_base.up;
    g_cfg.scope_base_rot_p = s_base.rot_p;
    g_cfg.scope_base_rot_y = s_base.rot_y;
    g_cfg.scope_base_rot_r = s_base.rot_r;
}

void restore_base() {
    g_cfg.scope_zoom  = s_base.zoom;
    g_cfg.scope_dist  = s_base.dist;
    g_cfg.scope_right = s_base.right;
    g_cfg.scope_up    = s_base.up;
    g_cfg.scope_rot_p = s_base.rot_p;
    g_cfg.scope_rot_y = s_base.rot_y;
    g_cfg.scope_rot_r = s_base.rot_r;
}

// Substring, not exact: class names on this title are long and decorated, and a config that has to
// reproduce them character-perfect is one nobody will successfully write. "FP_Magnum" matches
// BP_FP_Magnum_WeaponActor_C, and is the same key the wpnoff table already uses.
const ScopeAdjust* find_entry(const std::wstring& wpn) {
    if (wpn.empty()) return nullptr;
    for (int i = 0; i < g_cfg.scope_count; ++i) {
        const auto& e = g_cfg.wpn_scope[i];
        if (e.match[0] == 0) continue;
        const std::string m{e.match};
        const std::wstring wm(m.begin(), m.end());
        if (wpn.find(wm) != std::wstring::npos) return &e;
    }
    return nullptr;
}

} // namespace

void scope_offset_arm(bool on)  { s_armed.store(on, std::memory_order_relaxed); }
bool scope_offset_armed()       { return s_armed.load(std::memory_order_relaxed); }

void scope_offset_update() {
    if (!g_cfg.scope_offsets) {
        // Hand the global fit back on the way out, once, so turning this off mid-session does not
        // strand the last weapon's trim in g_cfg for the rest of the run.
        if (s_base.valid) {
            restore_base();
            s_base.valid = false;
            s_last_weapon.clear();
        }
        return;
    }

    // RE-CAPTURE THE BASE ONLY WHEN THE CONFIG ACTUALLY RELOADED FROM DISK.
    //
    // This is the correctness argument for the whole file, and it is copied deliberately from
    // WeaponOffset.cpp, which learned it the hard way. A successful load_config() has ALREADY put
    // the clean global scope fit back into g_cfg -- it runs before this function every ~2 s -- so
    // reading g_cfg here IS reading the file, and no undo of our own delta is needed or wanted.
    //
    // The tempting defensive version subtracts the applied delta before re-capturing, to cover the
    // early return load_config() takes when the main file fails to parse. That undo also fires on
    // the NORMAL path, where there is nothing to undo, so the base walks by -delta on every reload:
    // fixed direction, growing magnitude, the pane marching away from the player. The correct answer
    // to a failed parse is to not capture at all, which is exactly what watching the load
    // GENERATION gives -- g_cfg_load_gen is bumped past every early return, unlike a tick counter.
    if (g_cfg_load_gen != s_last_gen || !s_base.valid) {
        s_last_gen  = g_cfg_load_gen;
        s_base.zoom  = g_cfg.scope_zoom;
        s_base.dist  = g_cfg.scope_dist;
        s_base.right = g_cfg.scope_right;
        s_base.up    = g_cfg.scope_up;
        s_base.rot_p = g_cfg.scope_rot_p;
        s_base.rot_y = g_cfg.scope_rot_y;
        s_base.rot_r = g_cfg.scope_rot_r;
        s_base.valid = true;
        publish_base();
    }

    std::wstring wpn;
    if (auto* actor = fp_weapon_actor()) wpn = class_name_of(actor);
    const ScopeAdjust* hit = find_entry(wpn);

    // BASE FIRST, ALWAYS, BY ASSIGNMENT. A weapon with no entry must land exactly on the global
    // fit, and assigning rather than skipping the add is what stops a swap from a trimmed weapon
    // to an untrimmed one leaving the previous trim in place. It also means a tick that ran on the
    // parse-failure path cannot accumulate.
    restore_base();

    if (hit != nullptr) {
        // Zoom is a MULTIPLIER on the base, not an addend: magnification is a ratio (the pane lens
        // is scope_base_fov / zoom), so "+4" means something different at 2x than at 16x while
        // "x1.5" does not. Stored as a delta around 0 so an all-zero line is still a no-op, which
        // is what makes a hand-written entry with missing trailing fields behave.
        if (hit->d_zoom != 0.0f) {
            g_cfg.scope_zoom = clampf(s_base.zoom * (1.0f + hit->d_zoom), 1.0f, 64.0f);
        }
        g_cfg.scope_dist  += hit->d_dist;
        g_cfg.scope_right += hit->d_right;
        g_cfg.scope_up    += hit->d_up;
        g_cfg.scope_rot_p += hit->d_rot_p;
        g_cfg.scope_rot_y += hit->d_rot_y;
        g_cfg.scope_rot_r += hit->d_rot_r;
    }

    if (wpn != s_last_weapon) {
        s_last_weapon = wpn;
        if (g_cfg.scope_wpn_log) {
            if (hit != nullptr) {
                API::get()->log_info("[Halo-CampE-UEVR] SCOPEOFF %ls -> '%s' zoom x%.3f "
                                     "d=(%.2f,%.2f,%.2f)cm rot=(%.2f,%.2f,%.2f)",
                                     wpn.empty() ? L"<none>" : wpn.c_str(), hit->match,
                                     1.0f + hit->d_zoom, hit->d_dist, hit->d_right, hit->d_up,
                                     hit->d_rot_p, hit->d_rot_y, hit->d_rot_r);
            } else {
                API::get()->log_info("[Halo-CampE-UEVR] SCOPEOFF %ls -> no entry, using the global fit",
                                     wpn.empty() ? L"<none>" : wpn.c_str());
            }
        }
    }
}

bool scope_offset_capture() {
    if (!s_armed.exchange(false, std::memory_order_relaxed)) return false;

    if (!g_cfg.scope_offsets || !s_base.valid) {
        API::get()->log_info("[Halo-CampE-UEVR] SCOPECAL: per-weapon trims are off (scopeoffsets=0) "
                             "or no base captured yet -- nothing stored, the global fit is unchanged.");
        return true;   // still claimed: do NOT fall through to writing the global fit
    }

    const std::string key = weapon_key();
    if (key.empty()) {
        API::get()->log_info("[Halo-CampE-UEVR] SCOPECAL: no weapon in hand, capture ignored "
                             "(the global scope fit was NOT written either)");
        return true;
    }

    // THE DELTA AGAINST THE BASE, not against whatever is in g_cfg right now. By here g_cfg holds
    // the freshly captured absolute values, and scope_offset_update() has been adding this weapon's
    // existing trim on top of the base all along -- so measuring from g_cfg would fold the old trim
    // into the new one and double it on every capture.
    const float dz = (s_base.zoom > 0.0001f) ? (g_cfg.scope_zoom / s_base.zoom) - 1.0f : 0.0f;
    const float dd = g_cfg.scope_dist  - s_base.dist;
    const float dr = g_cfg.scope_right - s_base.right;
    const float du = g_cfg.scope_up    - s_base.up;
    const float dp = g_cfg.scope_rot_p - s_base.rot_p;
    const float dy = g_cfg.scope_rot_y - s_base.rot_y;
    const float dl = g_cfg.scope_rot_r - s_base.rot_r;

    // Replace an existing entry rather than appending a second one -- otherwise the first match
    // wins for ever and re-calibrating that weapon appears to do nothing.
    int slot = -1;
    for (int i = 0; i < g_cfg.scope_count; ++i) {
        if (_stricmp(g_cfg.wpn_scope[i].match, key.c_str()) == 0) { slot = i; break; }
    }
    if (slot < 0) {
        if (g_cfg.scope_count >= kMaxScopeAdjust) {
            API::get()->log_info("[Halo-CampE-UEVR] SCOPECAL: table full (%d), cannot add '%s'",
                                 kMaxScopeAdjust, key.c_str());
            return true;
        }
        slot = g_cfg.scope_count++;
    }

    // OUTLIER WARNING, on the same reasoning as the weapon-offset capture: a capture cannot be
    // validated automatically -- only the player can see whether the pane looks right -- but a
    // delta far outside the plausible range is almost always a bad capture, and one written
    // silently shows up weeks later as a mystery. Say so while they still remember what they did.
    {
        const char* why = nullptr;
        if (std::fabs(dd) > 60.0f || std::fabs(dr) > 60.0f || std::fabs(du) > 60.0f) why = "placement over 60cm";
        else if (std::fabs(dz) > 3.0f)                                               why = "zoom over 4x the global";
        if (why != nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] SCOPECAL WARNING '%s': %s -- zoom x%.3f "
                                 "d=(%.2f,%.2f,%.2f)cm. Almost certainly a bad capture; re-do it, "
                                 "or delete the wpnscope line for this weapon.",
                                 key.c_str(), why, 1.0f + dz, dd, dr, du);
        }
    }

    auto& e = g_cfg.wpn_scope[slot];
    strncpy_s(e.match, sizeof(e.match), key.c_str(), _TRUNCATE);
    e.d_zoom = dz;
    e.d_dist = dd; e.d_right = dr; e.d_up = du;
    e.d_rot_p = dp; e.d_rot_y = dy; e.d_rot_r = dl;

    wpn_calib_write_file();
    API::get()->log_info("[Halo-CampE-UEVR] SCOPECAL '%s': zoom x%.3f d=(%.2f,%.2f,%.2f)cm "
                         "rot=(%.2f,%.2f,%.2f)  [slot %d of %d]",
                         key.c_str(), 1.0f + dz, dd, dr, du, dp, dy, dl, slot, g_cfg.scope_count);
    return true;
}

} // namespace halo
