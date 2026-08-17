// Heap value scanner -- finds Blam-side data that UE reflection cannot see.
//
// WHY THIS EXISTS
//   This title is a hybrid: UE renders, a preserved Blam sim owns gameplay. Gameplay state is not
//   in the UE object graph, so reflection cannot reach it -- which is already why ControlRotation
//   is read as raw memory rather than through a property. The controller sensitivity and look
//   deadzone are the same kind of value: the UE settings object holds a copy that stores, persists
//   and does absolutely nothing (verified -- writing it changes no turn rate), because the real
//   value lives on the Blam side and the UE object is a UI mirror that only pushes on menu apply.
//
//   Those Blam values are loaded from tag data in the pak, so they sit in HEAP memory. UEVR's
//   pattern scanner searches the executable image and therefore cannot see them at all.
//
//   docs\Community-HaloCE-VR-Research.md documents the shipping values for `globals-globals ->
//   player control[0]`, which gives a signature to search for rather than a blind hunt:
//       magnetism friction / adhesion  0.625, 0.625
//       controller dead zones          axial 0.125, radial 0.125
//       look stick pegged scale        x=3, y=2   (pegged time 1.15)
//   Finding a run of those floats locates the whole struct at once.
//
// WHY IT RUNS ON ITS OWN THREAD
//   A full address-space walk takes far longer than a frame. Doing it on the game thread would
//   freeze the game outright, which in VR is not a stutter but a fault. The scan is read-only and
//   its results are only logged, so a detached worker is safe -- and it is the only shape that does
//   not cost the player anything even while running.
//
// DEV-ONLY, per DevTools.hpp: this answers a question rather than playing the game. Nothing here is
// compiled into a release build.

#pragma once

#include "DevTools.hpp"

#include <cstdint>

namespace halo {

#if HALO_VR_DEV

// Watches the `memscan` config key. 0 -> 1 kicks off one background scan for the float sequence in
// `memscanvals`, logging every hit with its address and surrounding values. Called from the tick;
// starting a scan is cheap because the tick only spawns the worker.
void mem_scan_tick();

// Differential (unknown-value) scan, driven by memdiff: 1 = snapshot, 2 = find changed blocks and
// materialise candidates, 3+ = narrow again, 0 = clear. Finds state by BEHAVIOUR rather than value,
// which is the only way to locate controller input -- its stored representation is unknown and any
// guess lands in 0..1 where mesh and animation data saturate the search.
void mem_diff_tick();

// TLS-GRAPH WALK (the navpoint hunt's anchor search). Walks the pointer graph reachable from the
// sim's TLS block (published by BlamDrive) two levels deep, scanning each pointed-to region for an
// adjacent float pair near (wu_x, wu_y) -- the objective's position in Blam world units. A hit's
// CHAIN (block+0xA -> +0xB -> offset) is a stable structural path, which is the whole point: heap
// scans found the value in relocating buffers; this finds it via a root that survives relocation.
// Off-thread, read-only, bounded. One shot per call.
void nav_tls_scan(float wu_x, float wu_y);

// MANAGER-ROOTED GRAPH WALK (the DataInterfaces path). Given the navpoints manager pointer --
// a STABLE root, reached from the widget tree the same way navworld resolves it -- walks the
// pointer graph breadth-first to a bounded depth, RPM-copying every read, scanning each visited
// block for the (x,y) world-unit pair. On a hit it logs the FULL OFFSET CHAIN from the manager
// (manager +0xA -> +0xB -> ... -> +0xN), which is exactly the stable per-tick resolution path
// the markers need -- no reflection, no relocation, no aim. root=0 no-ops.
void nav_graph_scan(uintptr_t manager_root, float wu_x, float wu_y);

#else

inline void mem_scan_tick() {}
inline void mem_diff_tick() {}
inline void nav_tls_scan(float, float) {}
inline void nav_graph_scan(uintptr_t, float, float) {}

#endif

} // namespace halo
