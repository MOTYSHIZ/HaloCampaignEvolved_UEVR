#include "WeaponCalib.hpp"

#include "Config.hpp"
#include "Rig.hpp"
#include "UeObject.hpp"
// palettearm_weapon_owns(): which driver a capture belongs to. Header only -- this file does not
// reach into the palette folder's internals, it asks the one ownership question ArmDriver arbitrates.
#include "palettearm/PaletteArm.hpp"

#include <windows.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

using uevr::API;

namespace halo {

char g_wpn_calib_path[MAX_PATH] = {0};

namespace {

std::atomic<bool> s_held{false};

// LATCHED ON RELEASE, consumed by the capture.
//
// The first version asked "is the key held?" from inside the finish handler -- which runs BECAUSE
// the key was just released, so the answer was always no. The capture never claimed anything and
// every press fell through to write_calib_file(), silently overwriting the global calibration with
// a fit made while holding one specific weapon. Exactly the failure this feature exists to avoid.
//
// The author's own gesture has the same shape and solves it the same way: g_calib_start and
// g_calib_finish are edges latched by the poll, not states sampled later.
std::atomic<bool> s_pending{false};

} // namespace

// A short, stable key for the weapon in hand.
//
// EXTERNAL LINKAGE, not file-local: Holster.cpp renders the magazine for the weapon actually in
// hand and needs this key too, so it is declared in WeaponCalib.hpp rather than kept private.
//
// The full class is BP_<name>_WeaponActor_C. Storing the whole decorated string would work, but
// the substring matcher in WeaponOffset.cpp wants something a human can also type by hand into
// halo_vr_user.cfg, and "AssaultRifle" is that. Strip the known prefix and suffix; if the shape is
// unexpected, keep the whole name rather than guess a truncation.
std::string weapon_key() {
    auto* actor = fp_weapon_actor();
    if (actor == nullptr) return {};
    std::string n = narrow(class_name_of(actor));
    if (n.empty()) return {};

    const std::string suffix = "_WeaponActor_C";
    if (n.size() > suffix.size() && n.compare(n.size() - suffix.size(), suffix.size(), suffix) == 0) {
        n.erase(n.size() - suffix.size());
    }
    if (n.rfind("BP_", 0) == 0) n.erase(0, 3);
    return n;
}

const WeaponFix* wpnfix_find(const std::string& key) {
    if (key.empty()) return nullptr;
    // LAST match wins, for the same reason weapon_fix_for() in Config.hpp takes the last one: the
    // table holds the shipped baseline first and the captured entries after it, and a capture is
    // meant to outrank the baseline it was taken against.
    const WeaponFix* hit = nullptr;
    for (int i = 0; i < g_cfg.wpnfix_count; ++i) {
        const auto& w = g_cfg.wpnfix[i];
        if (w.match[0] == 0) continue;
        if (key.find(w.match) != std::string::npos) hit = &w;
    }
    return hit;
}

void wpnfix_set(const std::string& key, const float q[4], const float t[3]) {
    if (key.empty()) return;

    // SEARCH THE CAPTURED ENTRIES ONLY, and this is the load-bearing half of the two-tier design.
    //
    // The table also holds the shipped baseline from halo_vr.cfg. Overwriting one of those would
    // (a) be undone within ~2 s, because the reload reparses the shipped file, and (b) worse, be
    // COPIED into the player's halo_vr_weapons.cfg by the rewrite below -- a stale duplicate of a
    // value they never chose, which then wins forever and silently defeats any future update to
    // the shipped baseline. Appending a captured entry instead leaves the baseline intact and
    // simply outranks it: the resolver takes the LAST match (Config.hpp, weapon_fix_for).
    int slot = -1;
    for (int i = 0; i < g_cfg.wpnfix_count; ++i) {
        if (!g_cfg.wpnfix[i].captured) continue;
        if (g_cfg.wpnfix[i].match[0] != 0 && key.find(g_cfg.wpnfix[i].match) != std::string::npos) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        if (g_cfg.wpnfix_count >= kMaxWeaponAdjust) {
            API::get()->log_info("[Halo-CampE-UEVR] WPNFIX: table full (%d), cannot add '%s'",
                                 kMaxWeaponAdjust, key.c_str());
            return;
        }
        slot = g_cfg.wpnfix_count++;
        strncpy_s(g_cfg.wpnfix[slot].match, sizeof(g_cfg.wpnfix[slot].match), key.c_str(), _TRUNCATE);
    }
    g_cfg.wpnfix[slot].captured = true;
    for (int i = 0; i < 4; ++i) g_cfg.wpnfix[slot].q[i] = q[i];
    for (int i = 0; i < 3; ++i) g_cfg.wpnfix[slot].t[i] = t[i];
    wpn_calib_write_file();
}

// Store a captured SUPPORT-HAND GRIP OFFSET for `key` and rewrite halo_vr_weapons.cfg.
//
// Same two-tier rule as wpnfix_set above, and for the same reason: search only the CAPTURED
// entries, so a shipped baseline is outranked rather than overwritten-and-copied into the player's
// file, where the copy would outlive the value it was copied from. weapon_grip_for() takes the LAST
// match, so appending is what makes a capture win.
void wpngrip_set(const std::string& key, float off_y, float off_z, float at_x) {
    if (key.empty()) return;

    int slot = -1;
    for (int i = 0; i < g_cfg.grip_count; ++i) {
        if (!g_cfg.wpn_grip[i].captured) continue;
        if (g_cfg.wpn_grip[i].match[0] != 0 &&
            key.find(g_cfg.wpn_grip[i].match) != std::string::npos) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        if (g_cfg.grip_count >= kMaxWeaponGrip) {
            API::get()->log_info("[Halo-CampE-UEVR] WPNGRIP: table full (%d), cannot add '%s'",
                                 kMaxWeaponGrip, key.c_str());
            return;
        }
        slot = g_cfg.grip_count++;
        strncpy_s(g_cfg.wpn_grip[slot].match, sizeof(g_cfg.wpn_grip[slot].match),
                  key.c_str(), _TRUNCATE);
    }
    g_cfg.wpn_grip[slot].captured = true;
    g_cfg.wpn_grip[slot].off_y    = off_y;
    g_cfg.wpn_grip[slot].off_z    = off_z;
    g_cfg.wpn_grip[slot].at_x     = at_x;
    wpn_calib_write_file();
}

bool wpngrip_clear(const std::string& key) {
    if (key.empty()) return false;
    for (int i = 0; i < g_cfg.grip_count; ++i) {
        if (!g_cfg.wpn_grip[i].captured) continue;
        if (g_cfg.wpn_grip[i].match[0] == 0 ||
            key.find(g_cfg.wpn_grip[i].match) == std::string::npos) continue;
        for (int j = i; j + 1 < g_cfg.grip_count; ++j) g_cfg.wpn_grip[j] = g_cfg.wpn_grip[j + 1];
        g_cfg.wpn_grip[--g_cfg.grip_count] = WeaponGrip{};
        wpn_calib_write_file();
        return true;
    }
    return false;
}

bool wpn_calib_take_pending() { return s_pending.exchange(false, std::memory_order_relaxed); }

void wpn_calib_set_pending() { s_pending.store(true, std::memory_order_relaxed); }

// Drop the equipped weapon's per-weapon POSE delta, so it falls back to the global fit.
//
// Deliberately NOT filtered on a `captured` flag the way wpngrip_clear is: WeaponAdjust has no
// such flag, because wpnoff entries have always come from one source. Removing a hand-written
// wpnoff from halo_vr.cfg is therefore not possible here and should not be -- this rewrites the
// machine-owned file only, and a shipped entry reappears on the next reload, which is the honest
// outcome rather than a silent half-clear.
bool wpnoff_clear_current() {
    const std::string key = weapon_key();
    if (key.empty()) {
        API::get()->log_info("[Halo-CampE-UEVR] WPNCAL: nothing cleared -- no weapon in hand.");
        return false;
    }
    for (int i = 0; i < g_cfg.wpn_count; ++i) {
        if (g_cfg.wpn[i].match[0] == 0 ||
            key.find(g_cfg.wpn[i].match) == std::string::npos) continue;
        for (int j = i; j + 1 < g_cfg.wpn_count; ++j) g_cfg.wpn[j] = g_cfg.wpn[j + 1];
        g_cfg.wpn[--g_cfg.wpn_count] = WeaponAdjust{};
        wpn_calib_write_file();
        API::get()->log_info("[Halo-CampE-UEVR] WPNCAL: cleared the per-weapon pose for '%s'; it "
                             "uses the shipped pose for this weapon if there is one, otherwise the "
                             "global fit.", key.c_str());
        return true;
    }
    API::get()->log_info("[Halo-CampE-UEVR] WPNCAL: '%s' had no per-weapon pose to clear.",
                         key.c_str());
    return false;
}

bool wpn_calib_held() { return s_held.load(std::memory_order_relaxed); }

// THE COMBINED CALIBRATION HOLD, published for other translation units.
//
// Plugin.cpp's g_calib_held lives in an ANONYMOUS namespace, so it has internal linkage and nothing
// outside that file can read it -- and reaching for it from another TU is a link error, not a
// compile one, which is a slow way to find out. This mirrors it here, where the rest of the
// calibration state already lives and the namespace is real.
//
// It covers ALL the hold sources (END, the menu mode, and the per-weapon key), because a second
// driver of the weapon has to stand down for every one of them, not just the per-weapon gesture.
std::atomic<bool> s_calib_hold_any{false};
void calib_hold_publish(bool on) { s_calib_hold_any.store(on, std::memory_order_relaxed); }
bool calib_hold_active()         { return s_calib_hold_any.load(std::memory_order_relaxed); }

void wpn_calib_poll() {
    const bool down = (g_cfg.wpn_calib_key != 0) &&
                      ((GetAsyncKeyState(g_cfg.wpn_calib_key) & 0x8000) != 0);
    s_held.store(down, std::memory_order_relaxed);
    // NO LONGER LATCHES THE CLAIM HERE, and removing it was required rather than tidy.
    //
    // The destination is now decided by the ARMED MODE at the moment the freeze began (mode 4 =
    // this weapon), latched in Plugin.cpp's calibration edge. If this still set the claim on its
    // own falling edge, then releasing Insert during a mode 1 (GLOBAL) calibration would silently
    // redirect that capture into the held weapon's delta -- the global fit would appear not to
    // save, and the reason would be a key the player was only using as a hold.
    //
    // The key itself still works: it feeds the hold through wpn_calib_held(), and Plugin.cpp only
    // consults that while a mode is armed.
}

// Rewrite halo_vr_weapons.cfg in full.
//
// Shared by the HOME capture and by END, which clears the held weapon delta and must persist that
// -- otherwise the cleared entry comes straight back on the next ~2 s config reload.
//
// Whole-file rewrite is safe precisely because this file is machine-owned, which is the same
// reasoning Config.cpp gives for keeping halo_vr_calib.cfg separate from the commented one.
void wpn_calib_write_file() {
    FILE* f = nullptr;
    if (g_wpn_calib_path[0] != 0 && fopen_s(&f, g_wpn_calib_path, "wb") == 0 && f != nullptr) {
        fprintf(f, "# halo_vr - PER-WEAPON CALIBRATION. Written by the per-weapon capture key.\r\n"
                   "# Machine-owned: this file is rewritten in full on every capture, so do not\r\n"
                   "# hand-edit it. Hand-written wpnoff lines belong in halo_vr.cfg, which keeps\r\n"
                   "# its comments; both are loaded into the same table.\r\n"
                   "#\r\n"
                   "# wpnoff=<match>,<dx>,<dy>,<dz>,<dgrip>,<dyaw>,<droll>  -- DELTAS on the\r\n"
                   "# calibration in halo_vr_calib.cfg, not absolute values.\r\n"
                   "# Delete a line to send that weapon back to the plain calibration.\r\n\r\n");
        for (int i = 0; i < g_cfg.wpn_count; ++i) {
            const auto& e = g_cfg.wpn[i];
            // THE PLAYER'S ENTRIES ONLY, for the reason the wpnfix block below gives. This loop used
            // to write every entry, shipped baselines included, which is how the v0.4 shotgun test
            // line reached players' own files -- see parse_weapon_offset().
            if (e.match[0] == 0 || !e.captured) continue;
            fprintf(f, "wpnoff=%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                    e.match, e.d_x, e.d_y, e.d_z, e.d_grip, e.d_grip_yaw, e.d_grip_roll);
        }
        // ---- THE PALETTE PER-WEAPON DELTAS, CAPTURED ONES ONLY.
        //
        // The in-memory table also holds the SHIPPED baseline parsed out of halo_vr.cfg. Writing
        // those here would copy somebody else's calibration into the player's own file, where the
        // copy outlives the value it was copied from -- so a future release that improves the
        // shipped fit would be silently overridden by a stale duplicate the player never chose.
        // Only what a capture produced belongs in a file a capture owns.
        int captured = 0;
        for (int i = 0; i < g_cfg.wpnfix_count; ++i) {
            if (g_cfg.wpnfix[i].captured && g_cfg.wpnfix[i].match[0] != 0) ++captured;
        }
        if (captured > 0) {
            // THE STAMP GOES FIRST, and it MUST: the parser scopes wpnfixver to the file it appears
            // in and resets it per file, so a line above the stamp is a line with no stamp and is
            // dropped. That is deliberate -- it is what stops a foreign file inheriting ours.
            fprintf(f, "\r\n"
                       "# PALETTE per-weapon rigid delta -- the palette weapon carry (armdriver=2,\r\n"
                       "# pawpn=1), NOT the rig path's wpnoff above. Captured by the same key.\r\n"
                       "#\r\n"
                       "# These OVERRIDE the shipped per-weapon calibration in halo_vr.cfg, weapon by\r\n"
                       "# weapon. A weapon with no line here keeps the shipped fit; DELETING THIS FILE\r\n"
                       "# puts every weapon back on the shipped fit. Updates never overwrite it.\r\n"
                       "#\r\n"
                       "# wpnfix=<match>,qx,qy,qz,qw,tx,ty,tz\r\n"
                       "#   quaternion then METRES, both in the aim controller's own frame after the\r\n"
                       "#   global pawpnyaw/pitch/roll trim, in BLAM axes (X forward, Y LEFT, Z up).\r\n"
                       "#   The rotation right-multiplies the trimmed pose; the translation is\r\n"
                       "#   carried by it, so both ride the wrist.\r\n"
                       "#\r\n"
                       "# wpnfixver stamps that convention, and must stay ABOVE the lines it covers.\r\n"
                       "# Entries under any other stamp are ignored, so a file from another fork --\r\n"
                       "# blindcowboy24's PR #1 writes wpnfix lines in UE axes -- is refused rather\r\n"
                       "# than applied in the wrong frame.\r\n"
                       "wpnfixver=%d\r\n", kWeaponFixSchema);
            for (int i = 0; i < g_cfg.wpnfix_count; ++i) {
                const auto& e = g_cfg.wpnfix[i];
                if (!e.captured || e.match[0] == 0) continue;
                fprintf(f, "wpnfix=%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
                        e.match, e.q[0], e.q[1], e.q[2], e.q[3], e.t[0], e.t[1], e.t[2]);
            }
        }

        // PER-WEAPON SCOPE TRIMS live in the same machine-owned file, for the same reason: they
        // are written by a capture gesture and rewritten in full, so they must not sit in
        // halo_vr.cfg where they would destroy its comments. See ScopeOffset.hpp.
        fprintf(f, "\r\n# wpnscope=<match>,<dzoom>,<ddist>,<dright>,<dup>,<dp>,<dy>,<dr>  -- DELTAS on\r\n"
                   "# the GLOBAL scope fit. dzoom is a plain multiplier (1.5 = 1.5x); 0 = unset.\r\n"
                   "# Delete a line to send that weapon back to the global scope fit.\r\n\r\n");
        for (int i = 0; i < g_cfg.scope_count; ++i) {
            const auto& s = g_cfg.wpn_scope[i];
            if (s.match[0] == 0) continue;
            fprintf(f, "wpnscope=%s,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                    s.match, s.d_zoom, s.d_dist, s.d_right, s.d_up,
                    s.d_rot_p, s.d_rot_y, s.d_rot_r);
        }

        // ---- SUPPORT-HAND GRIP OFFSETS. CAPTURED ENTRIES ONLY.
        //
        // Unlike the wpnscope block above this filters on `captured`, because this table is the
        // two-tier kind: if a shipped baseline is ever added to halo_vr.cfg, writing every entry
        // here would copy it into the player's file and it would then outrank future updates to
        // itself. Filtering costs nothing today and removes the trap before it can be set.
        fprintf(f, "\r\n# wpngrip=<match>,<offy>,<offz>[,<atx>]  -- where this weapon's FRONT HANDLE\r\n"
                   "# sits relative to its barrel, in the gun's own frame, centimetres. Lets a\r\n"
                   "# rocket launcher or sentinel beam be gripped where it actually has a handle.\r\n"
                   "# atx is the reach it was captured at: recorded for diagnosis, never applied.\r\n"
                   "# Delete a line to hold that weapon like a rifle again.\r\n\r\n");
        for (int i = 0; i < g_cfg.grip_count; ++i) {
            const auto& gp = g_cfg.wpn_grip[i];
            if (gp.match[0] == 0 || !gp.captured) continue;
            fprintf(f, "wpngrip=%s,%.2f,%.2f,%.2f\r\n", gp.match, gp.off_y, gp.off_z, gp.at_x);
        }
        fclose(f);
    }
}

bool wpn_calib_capture() {
    // ---- THE PALETTE DRIVER OWNS THIS PRESS.
    //
    // CLAIM IT BUT DO NOT CONSUME THE LATCH. Claiming (returning true) is what stops the caller
    // falling through to write_calib_file() -- under the palette driver the rig's fitted grip and
    // mount are not in the weapon's chain at all, so a press here must not rewrite the one global
    // calibration every unlisted weapon on the RIG path still depends on. Leaving the latch alone
    // is what lets palettearm's own release edge take it (wpn_calib_take_pending) whichever of the
    // two runs first this tick; it consumes the latch unconditionally, so it cannot linger and fire
    // against an unrelated END solve later -- which is the exact failure the latch exists for.
    //
    // Writing a wpnoff delta here as well would not be harmless: it is invisible while the palette
    // drives, and then moves the weapon the moment the player switches back to the rig driver.
    if (palettearm_weapon_owns()) return true;

    if (!s_pending.exchange(false, std::memory_order_relaxed)) return false;

    const std::string key = weapon_key();
    if (key.empty()) {
        API::get()->log_info("[Halo-CampE-UEVR] WPNCAL: no weapon in hand, capture ignored "
                             "(the global calibration was NOT written either)");
        return true;   // still claimed: do not let it fall through to the global write
    }

    // The DELTA against the base the config supplied. g_cfg currently holds the freshly solved
    // absolute values, and WeaponOffset has already been applying this weapon's existing delta on
    // top of that base -- so the correct new delta is measured from the base, not from whatever is
    // in g_cfg right now.
    const float dx = g_cfg.off_x - g_cfg.wpn_base_off_x;
    const float dy = g_cfg.off_y - g_cfg.wpn_base_off_y;
    const float dz = g_cfg.off_z - g_cfg.wpn_base_off_z;
    const float dg = g_cfg.grip_deg  - g_cfg.wpn_base_grip;
    const float dgy = g_cfg.grip_yaw  - g_cfg.wpn_base_grip_yaw;
    const float dgr = g_cfg.grip_roll - g_cfg.wpn_base_grip_roll;

    // Replace an existing entry for this weapon rather than appending a second one -- otherwise
    // the first match wins forever and re-calibrating appears to do nothing.
    int slot = -1;
    for (int i = 0; i < g_cfg.wpn_count; ++i) {
        if (_stricmp(g_cfg.wpn[i].match, key.c_str()) == 0) { slot = i; break; }
    }
    if (slot < 0) {
        if (g_cfg.wpn_count >= kMaxWeaponAdjust) {
            API::get()->log_info("[Halo-CampE-UEVR] WPNCAL: table full (%d), cannot add '%s'",
                                 kMaxWeaponAdjust, key.c_str());
            return true;
        }
        slot = g_cfg.wpn_count++;
    }
    // ROLL IS NOT MEASURED BY THIS GESTURE -- see Config::wpn_roll. Recording it stores whatever
    // the wrist happened to be doing, which then rotates the weapon on any session that starts
    // with a different gun in hand.
    const float dgr_store = g_cfg.wpn_roll ? dgr : 0.0f;

    // OUTLIER WARNING. A capture cannot be validated automatically -- only the player can see
    // whether the gun looks right -- but a delta far outside the plausible range is almost always
    // a bad capture, and one silently written is one that shows up weeks later as a mystery. Say
    // so at the moment it happens, when the player still remembers what they just did.
    {
        const char* why = nullptr;
        if (std::fabs(dx) > 15.0f || std::fabs(dy) > 15.0f || std::fabs(dz) > 15.0f) why = "translation over 15cm";
        else if (std::fabs(dg) > 25.0f || std::fabs(dgy) > 25.0f)                    why = "pitch/yaw over 25deg";
        // 12, not 5: several degrees of per-weapon roll is normal on this title, and a warning
        // that fires on correct captures is one that gets ignored when it matters.
        else if (g_cfg.wpn_roll && std::fabs(dgr) > 12.0f)                           why = "roll over 12deg";
        if (why != nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] WPNCAL WARNING '%s': %s -- d=(%.2f,%.2f,%.2f)cm "
                                 "grip=(%.2f,%.2f,%.2f)deg. This is almost certainly a bad capture; "
                                 "re-do it, or delete the line from halo_vr_weapons.cfg.",
                                 key.c_str(), why, dx, dy, dz, dg, dgy, dgr);
        }
    }

    auto& w = g_cfg.wpn[slot];
    strncpy_s(w.match, sizeof(w.match), key.c_str(), _TRUNCATE);
    w.d_x = dx; w.d_y = dy; w.d_z = dz;
    w.d_grip = dg; w.d_grip_yaw = dgy; w.d_grip_roll = dgr_store;
    w.captured = true;   // the player's now, even if it replaced a shipped baseline

    wpn_calib_write_file();
    API::get()->log_info("[Halo-CampE-UEVR] WPNCAL '%s': d=(%.2f,%.2f,%.2f)cm "
                         "grip=(%.2f,%.2f,%.2f)deg  [slot %d of %d]",
                         key.c_str(), dx, dy, dz, dg, dgy, dgr, slot, g_cfg.wpn_count);
    return true;
}

void wpn_calib_load() {
    if (g_wpn_calib_path[0] == 0) return;
    parse_config_file(g_wpn_calib_path);
}

} // namespace halo
