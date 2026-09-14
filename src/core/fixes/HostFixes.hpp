#pragma once

// STABILITY FIXES: robustness fixes to the author's code paths that belong to no feature. Each runs
// while its service is active (core/Services.hpp): SVC_STABILITY is declared by the stabilityfixes
// registry row alone; SVC_RIG_GUARD also by the features that rely on the rig guard to work (see the
// row's documentation in features/stabilityfixes/StabilityFixes.hpp). Inactive, every entry point
// is a no-op that leaves the author's code exactly as he wrote it.

#include <cstdint>

#include "Math.hpp"
#include "uevr/API.hpp"

namespace halo {

// ---- SVC_RIG_GUARD: fault recovery and the stale rig guard (update(), game thread)
void stability_tick_begin();            // update()'s first statement: drop the rig after a fault
void stability_tick_faulted();          // the engine tick's exception filter: arm that drop
void stability_rig_parent_resolved();   // the rig resolve, right after g_rig_parent is taken
void stability_stale_rig_guard();       // right before the leash block's feature slot

// ---- SVC_STABILITY
void stability_tick_stage(const char* stage);   // a tick stage marker for the fault report
const char* stability_fault_stage_suffix();      // " stage 'x'" for the fault line, or ""
bool stability_nav_world_guarded(bool engaged, uint32_t tick);   // true = ran the lane under SEH
bool stability_ui_manager_miss_throttled();      // find_ui_manager: skip this miss's sweep
bool stability_reticle_rescan_follow(bool hud_hide, int reticle_count);   // hud_follow's sweep need
void stability_reticle_hide_begin();             // hud_reticle_follow's hide pass
void stability_reticle_hide_dead();
bool stability_reticle_hide_end();               // true = a hosted crosshair died: rescan
void stability_xrlayer_early(uint32_t tick);     // the compositor reticule's tick above the early-outs
void stability_stick_mode_want(bool want);       // note a stick-mode entry on the death camera
bool stability_stick_exit_after_death();         // true = the exit re-anchored after a death
void stability_turn_gate_note(bool fp_control_now);   // turnlog: a flick a gate swallowed
void stability_turn_snap_note(float step);             // turnlog: a snap that landed
void stability_teardown_early();                 // teardown: the OpenXR layer first
void stability_teardown_restore();               // teardown: the API layer's projection rewrite
void stability_holster_marker_tint(uevr::API::UObject* marker);   // holstermarkercolor on a pouch marker
bool stability_throw_too_slow(float peak_speed);  // grenminthrow
const char* stability_putback_text(const char* his_text, bool in_pouch);

} // namespace halo
