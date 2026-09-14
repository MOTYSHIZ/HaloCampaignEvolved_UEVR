#pragma once

// THE WEAPON OBJECT SERVICE (SVC_WEAPON_OBJECT). Core machinery, running while any feature that
// declares the service is enabled (reloadvr, slidevr, palettewpn).
//
// The held weapon's Blam object is resolved on the sim thread from the index the game thread
// publishes, and its node block is found by shape. Two sim-thread call sites run the resolve: the
// palette weapon's builder pose hook (every call) and the orientation getter hook in BlamDrive
// (every 64th call, standing down while the builder hook has probed in the last 250 ms).
//
// The slide node publish and the object-node capture pre-hook run while a feature publishes the
// rack as available (SVC_RACK_AVAILABLE).

#include <atomic>
#include <cstdint>

namespace halo {

// The held weapon's Blam OBJECT index, published by the game thread from the weapon actor's
// BlamObjectSynchronizationComponent (BlamObjectIndex). -1 = unknown. Consumed on the sim
// thread by the weapon-object node probe: the object's own node matrices are where the
// slide, magazine and trigger really live (the 76-node palette is the arms rig).
extern std::atomic<int32_t> g_wpn_obj_index;
// The datum g_wpn_obj_ptr was resolved from (-1 = none). Right after a swap the pointer can
// still be the previous weapon's object; a reader that must write the right weapon compares.
extern std::atomic<int32_t> g_wpn_obj_ptr_datum;

// THE SLIDE. The weapon object's node block is found by shape on the sim thread (its first
// short run of node matrices; node `slide_node` of it is the pistol's slide, named by eye
// 2026-09-03). The sim publishes that node's world position and forward every tick, and
// applies g_slide_pull (Blam units, along -forward) to it after the game's own pose so the
// slide follows the hand and still renders through BlamMeshSynchronization.
extern std::atomic<bool>  g_slide_node_valid;
extern std::atomic<float> g_slide_wx, g_slide_wy, g_slide_wz;     // Blam world
extern std::atomic<float> g_slide_fx, g_slide_fy, g_slide_fz;     // Blam world, unit
extern std::atomic<float> g_slide_pull;                           // Blam units, >= 0
// The resolved weapon object's address (0 = none), for game-thread READS of its fields --
// ordinary heap memory once resolved; only the resolve itself needs the sim thread.
extern std::atomic<uintptr_t> g_wpn_obj_ptr;
// The slide node's ADDRESS (a PaletteNode in the weapon object), for the GAME-THREAD write.
// Measured 2026-09-03: the sim rebuilds the node every tick, so a sim-side write only renders
// when the mesh sync happens to read after it. The game thread's pre-engine tick runs before
// the sync component ticks, so a write there lands between the rebuild and the read.
extern std::atomic<uintptr_t> g_slide_node_addr;

// THE OBJECT-NODE CAPTURE HOOK (slidehook). Installed/removed from the game thread; true while
// the pre-hook owns the slide write (the other two writers then stand down).
void blam_capture_hook_tick();
bool blam_capture_hook_active();

// SIM THREAD. The builder pose hook's probe (apply_after_pose), and the orientation getter hook's.
void weapon_object_probe_from_builder();
void weapon_object_offhook_tick();

// GAME THREAD, on the service's off edges (features_config_loaded): nothing published outlives the
// service. rack_reset clears the slide node and the pull, reset clears the object as well.
void weapon_object_rack_reset();
void weapon_object_reset();

} // namespace halo
