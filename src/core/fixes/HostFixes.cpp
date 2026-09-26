#include "core/fixes/HostFixes.hpp"
#include "TwoHandAim.hpp"   // two_hand_latched / two_hand_reset

#include "Config.hpp"
#include "MotionAimControl.hpp"   // g_turn_offset
#include "Reticule.hpp"           // reticule_widget_set_scene_hidden, reticule_mode3_reassert
#include "Rig.hpp"                // g_rig_component, g_rig_parent, g_rig_resolve_tick, g_rig_neutral_valid, g_dbg_persp, rig_component_alive
#include "UeObject.hpp"
#include "XrLayer.hpp"
#include "XrLayerBridge.hpp"
#include "XrSource.hpp"
#include "core/MarkerFaces.hpp"
#include "core/Services.hpp"
#include "core/fixes/TickStage.hpp"
#include "core/host/PluginState.hpp"
#include "features/hooks/PluginHooks.hpp"   // features_rig_lost

#include <Windows.h>

#include <atomic>
#include <cmath>
#include <cstdio>

using namespace uevr;

namespace halo {

namespace {

bool stab_on()  { return service_active(SVC_STABILITY); }
// The unconditional half: fixes to our own code, never behind a master key (kAlwaysOnServices).
bool fix_on()   { return service_active(SVC_HOST_FIXES); }
bool guard_on() { return service_active(SVC_RIG_GUARD); }

volatile bool g_tick_fault_pending = false;
TrackedObject g_rig_parent_track;   // the rig's attach parent by array slot (checked every tick)

}  // namespace

// ================================================================ SVC_RIG_GUARD

namespace { bool s_guard_was_on = false; }

void stability_tick_begin() {
    // THE GUARD COMING ON LIVE. The parent track is normally set when the rig's parent resolves, which
    // happens once per new rig; a guard switched on (stabilityfixes or roomscale) after that found the
    // track empty, read the live parent as gone, and dropped a healthy rig. Adopt the current parent on
    // the on edge instead. set() compares pointers against the object array and never dereferences, so
    // a parent that really is stale fails to adopt and the guard still drops it.
    const bool on = guard_on();
    if (on && !s_guard_was_on) g_rig_parent_track.set(g_rig_parent);
    s_guard_was_on = on;
    if (!on) { g_tick_fault_pending = false; return; }
    // AFTER A FAULT: whatever pointer the game rejected, the rig and its parent are the two the
    // tick trusts blindly; drop them and resolve again rather than fault on the same one next tick.
    if (g_tick_fault_pending) {
        g_tick_fault_pending = false;
        g_rig_component = nullptr;
        g_rig_parent = nullptr;
        g_rig_parent_track = TrackedObject{};
        g_rig_resolve_tick = 0;
    }
}

void stability_tick_faulted() {
    // ...and hand the recovery to the next tick: update() drops the rig and its parent and
    // re-resolves them, so a pointer the game rejected is not dereferenced again next tick.
    if (guard_on()) g_tick_fault_pending = true;
}

void stability_rig_parent_resolved() {
    if (guard_on()) g_rig_parent_track.set(g_rig_parent);
}

void stability_stale_rig_guard() {
    if (!guard_on()) return;
    // ---- STALE RIG GUARD. g_rig_parent is a RAW pointer taken off the tracked rig component; on
    // a level transition that keeps the same PlayerController (mission -> next mission, checkpoint
    // reload) the pawn is torn down but nothing nulls it, and the first per-tick caller to
    // dereference it -- the roomscale eye read -- threw inside on_pre_engine_tick EVERY tick
    // (measured: 10k exceptions, arms and roomscale dead until a toggle let the rig re-resolve).
    // Ask the tracker instead of trusting the pointer, and drop both pointers the moment the
    // component's array slot is gone, so every user below sees null and the resolve runs promptly.
    // THE PARENT DIES ON ITS OWN (2026-09-09): a mission spawn replaced the camera component
    // while the FP mesh lived on, the parent pointer dangled, and every tick faulted in the game
    // on it for three minutes -- the resolver that would have refreshed it sits after the fault.
    // The parent is tracked by array slot and checked here every tick, like the rig itself.
    if (g_rig_parent != nullptr && (!rig_component_alive() || g_rig_parent_track.get() != g_rig_parent)) {
        API::get()->log_info("[Halo-CampE-UEVR] rig: tracked %s gone (level transition?) -- dropping rig + parent, re-resolving", rig_component_alive() ? "PARENT" : "component");
        g_rig_component = nullptr;
        g_rig_parent = nullptr;
        g_rig_resolve_tick = 0;
        features_rig_lost();
    }
}

// ================================================================ SVC_STABILITY

void stability_tick_stage(const char* stage) {
    if (fix_on()) g_tick_stage = stage;
}

const char* stability_fault_stage_suffix() {
    // Exception filter context: a static buffer, no allocation.
    static char s_buf[96];
    if (!fix_on()) return "";
    const char* st = g_tick_stage;
    std::snprintf(s_buf, sizeof(s_buf), " stage '%s'", st != nullptr ? st : "");
    return s_buf;
}

namespace {
#define NAVW_MARK(s) g_navw_mark.store((s), std::memory_order_relaxed)
// ---- THE NAV LANE IS QUARANTINED. 2026-09-02, measured: a null dereference inside engine code,
// reached from this lane on the ticks after a marker re-host, took the whole tick down with it --
// update() never reached the aim law, which therefore never re-armed, so the sim-thread write
// kept pushing a STALE setpoint and the field symptom was "I can't turn" (11,084 exceptions in
// one session, err=50 deg on the control record while snap turns logged fine). A lane that draws
// waypoints must not be able to stop the player turning. SEH in a function with nothing to
// unwind; on a fault the marker pool is dropped (slots re-create on the next foot segment) and
// the lane sleeps ~10 s before trying again, so a persistent fault costs one line every 10 s
// instead of every frame.
uint32_t g_navw_sleep_until = 0;
void navw_drop_all() {
    // Plugin.cpp's nav lane state, through the bridge: the same objects under the same names.
    TrackedObject* g_navw_pool = host::g_plugin_state.navw_pool;
    bool* g_navw_mid_ok = host::g_plugin_state.navw_mid_ok;
    void** g_navw_slot_class = host::g_plugin_state.navw_slot_class;
    auto& g_navw_placed_n = *host::g_plugin_state.navw_placed_n;
    for (int i = 0; i < 8; ++i) {
        g_navw_pool[i] = TrackedObject{};
        g_navw_mid_ok[i] = false;
        g_navw_slot_class[i] = nullptr;
    }
    g_navw_placed_n = 0;
}
void nav_world_tick_guarded(bool engaged, uint32_t tick) {
    auto& g_navw_mark = *host::g_plugin_state.navw_mark;
    std::atomic<uint32_t>* g_lane_faults = host::g_plugin_state.lane_faults;
    const int PERF_NAVWORLD = host::g_plugin_state.perf_navworld;
    const auto nav_world_tick = host::g_plugin_state.nav_world_tick;
    if (tick < g_navw_sleep_until) return;
    __try {
        nav_world_tick(engaged, tick);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        static uint32_t s_n = 0;
        const char* mark = g_navw_mark.load(std::memory_order_relaxed);
        // Attributed to the lane the way report_tick_fault would have been, so the perf window's
        // fault column and lane_cooling() see it even though it never escaped the guard.
        g_lane_faults[PERF_NAVWORLD].fetch_add(1, std::memory_order_relaxed);
        if (++s_n <= 10)
            API::get()->log_info("[Halo-CampE-UEVR] NAVWORLD: exception #%u inside the nav lane (step '%s') -- "
                                 "marker pool dropped, lane sleeping 10 s, rest of the tick continues",
                                 s_n, (mark != nullptr) ? mark : "-");
        NAVW_MARK(nullptr);
        navw_drop_all();
        g_navw_sleep_until = tick + 320;
    }
}
}  // namespace

bool stability_nav_world_guarded(bool engaged, uint32_t tick) {
    if (!fix_on()) return false;
    nav_world_tick_guarded(engaged, tick);
    return true;
}

bool stability_ui_manager_miss_throttled() {
    if (!fix_on()) return false;
    // A miss means a full object-array sweep; retried every 128th call, not three times a second.
    static uint32_t miss_calls = 0;
    return (miss_calls++ & 127) != 0;
}

bool stability_reticle_rescan_follow(bool hud_hide, int reticle_count) {
    // Moves/hides the flat reticle. HIDING needs the sweep only until the widget is found (and
    // again if it dies): the hide re-applies every tick on the handle it already holds. Following
    // still needs it throughout, as before.
    if (!fix_on()) return true;
    return (!hud_hide || reticle_count == 0);
}

namespace { bool s_hide_dead = false; }
void stability_reticle_hide_begin() { s_hide_dead = false; }
void stability_reticle_hide_dead()  { s_hide_dead = true; }
bool stability_reticle_hide_end() {
    // HUD rebuilt the widget: the gated sweep finds the new one.
    return fix_on() && s_hide_dead;
}

// THE COMPOSITOR RETICULE. The author's update() runs xrlayer_tick, the world-reticule latch,
// reticule_mode3_reassert and xrsource_tick every tick, above the early-outs. This service used to
// run its own copy of all four later in the same tick, so each ran twice and the two latches
// disagreed (below). It now adds its two changes to the author's block instead.
//
// THE LATCH. The author's latch never clears, so switching xrlayer off after the layer was once
// live left the world reticule hidden with no compositor reticule to replace it. Cleared while the
// layer is off; called right after the author's liveness test, so a layer still live on the first
// off tick cannot set it again.
bool stability_xrlayer_latch_released() {
    return fix_on() && !g_cfg.xr_layer;
}

// THE SOURCE WALK. With xrlayersrc=1 (the default) the author's xrsource_tick walks the hosted
// crosshair's widget chain every tick even while the layer is off. Skipped while the layer is off,
// and reset once on the way off so no resolved source outlives the layer.
// THE MOVEMENT PROBE. The author's update() samples the camera and logs a PROBE line on every
// step while the stick is held: an engine call per sample, found spamming a play session
// (2026-09-02). Gated on the dev key moveprobe, as the fork gated it.
bool stability_move_probe_allowed() {
    return !fix_on() || g_cfg.move_probe;
}

// EXTRA STEAL BITS (stealextra). Pad bits stripped alongside the author's holster steal, for a
// code the button log names (2026-09-04: left X still threw a grenade). Steal only, never injected.
unsigned short stability_steal_extra_mask() {
    return stab_on() ? (unsigned short)g_cfg.steal_extra_mask : (unsigned short)0;
}

// A DEAD STEAL THAT EATS A LIVE CONTROL. The author strips holstergswitchmask alongside the throw
// mask, but the only thing that ever INJECTS that mask is the grenade type switch inside the pouch
// grab (Holster.cpp try_grab), and every path into the pouches goes through aim_can_grab() or
// off_can_grab(), both of which require holstergren. With holstergren off -- its own default --
// nothing can inject it, so the steal is pure loss. It is worse than loss on the owner's profile:
// his holstergswitchmask is 0x0008, d-pad RIGHT, and he runs mapdpadshift=1, so the steal eats the
// shifted d-pad right the shift exists to produce.
//
// Returned as bits to KEEP, so his mask expression is untouched and the steal simply does not claim
// a button nothing of his or ours can press. Nothing else is moved: his block stays where he put
// it, and with this feature off his steal is exactly his again.
unsigned short stability_steal_dead_mask() {
    if (!stab_on()) return 0;
    const unsigned short gsw = (unsigned short)g_cfg.holster_gswitch_mask;
    if (gsw == 0) return 0;
    if (g_cfg.holster_enabled && g_cfg.holster_grenades) return 0;   // the grab can run: the injector is live
    return gsw;
}

namespace { bool s_src_was_on = false; }
bool stability_xrsource_wanted() {
    if (!fix_on() || g_cfg.xr_layer) {
        s_src_was_on = true;
        return true;
    }
    if (s_src_was_on) {
        s_src_was_on = false;
        xrsource_reset();
    }
    return false;
}

namespace { bool s_entered_dead = false; }

void stability_stick_mode_want(bool want) {
    if (!fix_on()) { s_entered_dead = false; return; }
            // WHY stick mode engaged decides what the EXIT does. A vehicle ride's net rotation
            // must FOLD into the turn offset (you exit facing where the ride faced). A DEATH is
            // different: the respawn camera jump is not a rotation the player performed, and
            // folding it rotates the whole room frame -- measured on a respawn as turn -90.0 ->
            // -76.2 with a 13.8 deg visible snap, and everything afterwards consistently
            // "facing left". RAW byte 2 is the death/third-person camera; remember it while stuck.
            //
            // The RAW byte, not fp_presentation_state(): that function NORMALIZES to
            // {-1 unknown, 0 not-FP, 1 FP} and can never return 2, so comparing its result
            // against the enum value left this branch unreachable -- measured 2026-08-26: a
            // death logged raw persp=2 at stick enter, no death mark, and the exit folded the
            // 76 deg respawn jump into the turn offset (the exact failure this branch exists
            // to stop). Only the raw byte still carries the death value.
            if (want && g_dbg_persp.load(std::memory_order_relaxed) == 2 && !s_entered_dead) {
                s_entered_dead = true;
                API::get()->log_info("[Halo-CampE-UEVR] STICK: death camera noted (raw persp=2) -- exit will re-anchor");
            }
}

bool stability_stick_exit_after_death() {
    if (!fix_on() || !s_entered_dead) return false;
    // Plugin.cpp's own state, through the bridge: the same objects under the same names.
    auto& g_lock_ever   = *host::g_plugin_state.lock_ever;
    auto& g_lock_primed = *host::g_plugin_state.lock_primed;
    auto& g_have_ref    = *host::g_plugin_state.have_ref;
                    // EXIT AFTER A DEATH: full first-prime. The base adopts the respawn camera
                    // outright and the accumulated turn is cleared -- physical forward becomes
                    // the respawn's forward, which is what a fresh spawn means. The fold below
                    // stays for rides, where the net rotation is genuinely the player's.
                    s_entered_dead = false;
                    g_turn_offset = 0.0f;
                    g_lock_ever = false;
                    g_lock_primed = false;
                    g_have_ref = false;
                    g_rig_neutral_valid = false;
                    API::get()->log_info("[Halo-CampE-UEVR] STICK EXIT after death -- full re-anchor (turn cleared)");
    return true;
}

void stability_turn_gate_note(bool fp_control_now) {
    if (!stab_on()) return;
    const auto& g_raw_stick_x = *host::g_plugin_state.raw_stick_x;
    const auto& g_stick_mode  = *host::g_plugin_state.stick_mode;
    const bool  g_fp_control_now = fp_control_now;
    //
    // WHY-NOT INSTRUMENT (stabilityturnlog): "sometimes turning works and sometimes it doesnt" cannot be
    // diagnosed from transition logs alone -- the flick that went nowhere is the evidence, and
    // only this spot knows why. Logs ONE line per deadzone crossing while any gate blocks, naming
    // every gate's state, and one line per snap that lands, so the two interleave in time.
    if (g_cfg.turn_log) {
        static bool s_tl_past = false;
        const float sxl = g_raw_stick_x.load();
        const bool  past = std::fabs(sxl) > g_cfg.turn_dz;
        const bool  blocked = g_cfg.turn_mode == 0 || g_stick_mode.load() || !g_fp_control_now;
        if (past && !s_tl_past && blocked) {
            API::get()->log_info("[Halo-CampE-UEVR] TURN blocked: sx=%.2f mode=%d stickmode=%d "
                                 "fpcontrol=%d persp=%d",
                                 sxl, g_cfg.turn_mode, (int)g_stick_mode.load(),
                                 (int)g_fp_control_now, g_dbg_persp.load());
        }
        s_tl_past = past;
    }
}

void stability_turn_snap_note(float step) {
    if (stab_on() && g_cfg.turn_log)
        API::get()->log_info("[Halo-CampE-UEVR] TURN snap %+.0f -> turn=%.1f", step, g_turn_offset.load());
}

void stability_teardown_early() {
    if (!fix_on()) return;
        // 0. THE OPENXR LAYER BEFORE ANYTHING ELSE. It owns an XR swapchain and D3D12 resources
        //    parented to the session UEVR is about to destroy, and its end-frame path runs on the
        //    submit thread. Left up, the game hangs on exit inside UEVR's own teardown.
        //    Idempotent, and safe when it was never brought up.
        API::get()->log_info("[Halo-CampE-UEVR] TEARDOWN stage: xrlayer_shutdown");
        xrlayer_shutdown();
        API::get()->log_info("[Halo-CampE-UEVR] TEARDOWN stage: xrlayer_shutdown RETURNED");
}

void stability_teardown_restore() {
    // And the API layer's projection rewrite, in case the exit lands mid-cutscene: an atomic
    // store in the layer, harmless when the layer is absent or already gone.
    if (fix_on()) halo::xrbridge_set_projection_mono(0);
}

void stability_holster_marker_tint(uevr::API::UObject* marker) {
    if (stab_on()) marker_tint(marker, g_cfg.holster_marker_color);
}

bool stability_throw_too_slow(float peak_speed) {
    // MIN THROW SPEED (stabilitygrenminthrow, 0 = off): a release that never swung is a put-back wherever
    // the hand is. Judged on the PEAK: every measured throw peaked at 2.04 or above and the
    // deliberate put-back at 0.16.
    return stab_on() && g_cfg.gren_min_throw > 0.0f && peak_speed < g_cfg.gren_min_throw;
}

const char* stability_putback_text(const char* his_text, bool in_pouch) {
    return in_pouch ? his_text : "put back (below stabilitygrenminthrow)";
}

// GESTURE RESET, the two-handed hold (moved from gesture_reset):
// The two-handed hold rides along. It is latched on a button and blended into aim, so a
// transition the player did not choose (kill switch, stick mode, a tracking stall) must drop it
// too -- otherwise the aim stays blended toward a support hand nothing is tracking any more.
void stability_gesture_reset_two_hand() {
    if (two_hand_latched()) two_hand_reset("gesture reset");   // his reset logs; only drop a hold that exists
}

// MENU COMMAND FILE: an attributes query first. The command file is almost never there, and a failed open every
// poll was the last un-gated file operation on the tick path (perf audit, 2026-09-06).
bool stability_menu_command_file_absent(const char* path) {
    return GetFileAttributesA(path) == INVALID_FILE_ATTRIBUTES;
}

// ---- OFF-PATH PARITY, PRINTED. Every hook in this file is a gate on the AUTHOR'S code, so with
// stabilityfixes off each one must reproduce HIS expression exactly -- not "allow", and not the
// play build's fork gate, which is a different thing and the easy mistake to make when checking
// this. Several of his expressions are UNGATED, so the honest off-value for those hooks really is
// "allow", and the only way to tell the two apart is to write his expression down beside it.
//
// Logged once, when the feature resolves off, against upstream/development at the line given.
// Checked 2026-09-18, hook by hook, and every row below was read out of his file rather than
// assumed. The two that read oddest are real: the movement probe and the crosshair source walk
// are both ungated in his release, and the fork gates on moveprobe and xrlayer arrived with the
// play build, so falling through to "runs" IS his behaviour and gating them would not be.
void stability_log_off_parity() {
    struct Row { const char* hook; const char* off_value; const char* his_expression; };
    static const Row kRows[] = {
        { "fault_stage_suffix",      "\"\" (no suffix)",   "no stage suffix exists in his release" },
        { "nav_world_guarded",       "false",             "his own nav_world_tick runs unguarded" },
        { "ui_manager_miss_throttled", "false",           "his miss sweep is not throttled" },
        { "reticle_rescan_follow",   "true",              "|| g_cfg.hud_follow, ungated (Plugin.cpp:2357)" },
        { "reticle_hide_end",        "false",             "no re-arm after a HUD rebuild in his release" },
        { "xrlayer_latch_released",  "false",             "his latch is never cleared" },
        { "move_probe_allowed",      "true",              "if (g_rig_parent && mag > 0.5f), UNGATED (Plugin.cpp:7995)" },
        { "steal_extra_mask",        "0",                 "his steal mask has no extra bits" },
        { "steal_dead_mask",         "0",                 "his steal claims holstergswitchmask unconditionally" },
        { "xrsource_wanted",         "true",              "xrsource_tick(tick), UNGATED (Plugin.cpp:6068)" },
        { "stick_exit_after_death",  "false",             "his exit does not re-anchor after a death" },
        { "turn_gate_note",          "no-op",             "he logs no swallowed flick" },
        { "turn_snap_note",          "no-op",             "he logs no snap" },
        { "teardown_early",          "no-op",             "his teardown order is unchanged" },
        { "teardown_restore",        "no-op",             "he has no projection rewrite to restore" },
        { "holster_marker_tint",     "no tint",           "his pouch markers are untinted" },
        { "throw_too_slow",          "false",             "any release is a throw in his release" },
        // Not in this file, listed because they are the same shape and were checked with it.
        { "hmd_pose_plausible",      "true",              "get_pose alone, no plausibility test" },
        { "widget_log",              "true",              "if ((t++ % 32) == 0), UNGATED (Reticule.cpp:1639)" },
        { "widget_alpha_hide_applies", "true",            "scene_hidden && hide_ws == 1 (Reticule.cpp:1691)" },
        { "leash_lateral / vertical", "true (skips his)",  "his block only runs under g_cfg.hmd_leash at all" },
    };
    API::get()->log_info("[Halo-CampE-UEVR] STABPARITY: stabilityfixes is OFF; each hook's value here against the author's own expression");
    for (const Row& r : kRows)
        API::get()->log_info("[Halo-CampE-UEVR] STABPARITY   %-28s off=%-18s his: %s", r.hook, r.off_value, r.his_expression);
}

} // namespace halo
