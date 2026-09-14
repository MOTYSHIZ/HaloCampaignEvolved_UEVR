#pragma once

// UNIT STATE FROM THE BLAM UNIT OBJECT, published on the sim thread. The fork's reader behind the
// author's declared-not-populated grenade atomics (BlamDrive.hpp): mounted, position, facing, the
// mounted vehicle's facing and position, and the seat publish that keeps them live in stick mode.
// Read by the holsters' fork additions, the seat camera and in-vehicle view, the wrist radar,
// roomscale, the height calibration, the head block and ForceTube's player filter.

#include <atomic>
#include <cstdint>

namespace halo {

// Unit state read from the player's unit object on the sim thread (see publish_unit_state):
// grenade type (0 frag / 1 plasma), pouch counts, whether the read is live, and whether the unit
// has a parent object (vehicle seat / turret). Consumed by the holsters.
extern std::atomic<bool> g_unit_mounted;
// The unit's WORLD POSITION (+0x20, Blam world units) and FACING (+0x50, unit vector, Blam
// frame), published beside the grenade state for the seat camera and the in-vehicle view.
// Measured: over 972 samples the large-motion delta ratios against the camera are +308/-306/+332,
// i.e. the 304.8 cm world unit with Blam's Y negation, constant residual = the eye height.
extern std::atomic<float> g_unit_px, g_unit_py, g_unit_pz;
extern std::atomic<bool>  g_unit_pvalid;
extern std::atomic<float> g_unit_fx, g_unit_fy;
// The MOUNTED VEHICLE's facing (+0x1D4 pair -- swept 3226 deg as a unit vector in the spin test
// while every +0x50 field stayed constant) and its own position (+0x20, same layout as the
// biped's). Resolved once per mount from the biped's parent datum and cached; retried ~1 s while
// mounted-unresolved, because a mount-edge failure used to latch a backwards camera all ride.
extern std::atomic<float> g_veh_fx, g_veh_fy;
extern std::atomic<bool>  g_veh_fvalid;
extern std::atomic<float> g_vehpx, g_vehpy, g_vehpz;

// Resolve ANY object datum through the sim's object table. SIM THREAD ONLY (walks gs:[0x58]).
uintptr_t resolve_object_by_datum(uint32_t datum);

// SEAT PUBLISH EVIDENCE (vehlog). seq advances on every successful rider read from any path (sim
// publish or direct read), so the camera can tell a live rider from a frozen one. calls = sim
// publishes, norec = stick-mode calls that had no control record to publish from, reresolve =
// stick-mode record re-resolves, direct = vehseatdirect reads.
extern std::atomic<uint32_t> g_seat_pub_seq, g_seat_pub_calls, g_seat_norec, g_seat_reresolve,
                             g_seat_direct_reads;
// The rider object and its vehicle object as the last sim publish saw them (vehseatdirect).
extern std::atomic<uintptr_t> g_seat_obj, g_seat_vobj;

// vehseatdirect: refresh the seat atomics from the cached object pointers. Any thread; acts only
// while stick mode holds the sim publish's normal path off. No-op when the key is 0.
void seat_direct_refresh();

// The two entry points BlamDrive.cpp's hook calls reach (features/hooks/BlamDriveHooks.hpp).
// drive_angles_impl's stick-mode hold: the seat publish that must not stop while the aim write holds off.
void unit_state_stick_mode_publish(bool off_thread);
// drive_angles_impl with the record resolved and writable (rec at the yaw field): the unit publish.
void unit_state_record_ready(uintptr_t rec, bool off_thread);

} // namespace halo
