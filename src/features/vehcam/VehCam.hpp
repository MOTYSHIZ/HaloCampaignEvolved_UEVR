// ============================================================================================
// HANDS ON THE WHEEL -- physical vehicle control.
//
// WHAT MADE THIS CHEAP. The Warthog does not have a steering input of its own: while mounted, the
// player's control record carries the SAME movement pair the biped walks with -- (+0xAC forward,
// +0xB0 right), measured 2026-08-20 by correlating 2086 dumped record samples against the
// delivered stick: r(lx) = -0.988 at +0xB0 and r(ly) = +0.979 at +0xAC. So driving is not new
// archaeology, it is a second writer on a proven address.
//
// THE GESTURE. Grip inside the wheel zone (bolted to the hull's frame, tuned live like the
// holsters) and the wheel is held: the hand's ANGLE around the wheel centre becomes steering,
// zeroed at the instant of the grab so wherever you grabbed is centre. Two hands grip -> each
// hand accumulates its own rotation and the wheel takes the mean, which is how a real wheel is
// read. Release and the stick is yours again immediately -- the write simply stops, it does not
// fight the pad.
//
// No sim-thread write consumes the steer in this tree: the wheel latches, logs and publishes, and
// the vehicle does not steer from it yet.
// ============================================================================================
//
// FEATURE vehcam (Experimental), with the steering wheel (vehiclewheel, Experimental) and the in-vehicle
// view (vehview) it carries. The first-person seat camera bolted to the vehicle's drawn hull, the view
// yaw anchored to the vehicle, the driver body hide, the hull resolve, the wheel gesture and the
// travel heading. Hook slots: parse_key (vehiclewheel, veh*), game_tick_vehicle (body hide, hull
// resolve, wheel, heading), stereo_pre_eye_seat (the seat camera), stereo_view_override (the
// in-vehicle view), stereo_post_eye_rendered (the seat write-survival check). Table: kVehCamHooks.
// The unit and seat publish it reads is core (core/UnitState.hpp).

#pragma once

#include <atomic>
#include <cstdint>

#include "Math.hpp"
#include "features/FeatureHooks.hpp"
#include "uevr/API.hpp"

namespace halo {

extern const FeatureHooks kVehCamHooks;

// Published by the wheel below (game thread).
// g_veh_active gates the write: false = the pad owns the record, exactly as before.
extern std::atomic<bool>  g_veh_active;
extern std::atomic<float> g_veh_steer;    // -1..1, already signed for the record
extern std::atomic<float> g_veh_thr;      // -1..1 forward, or NaN-free 0 when not driven
extern std::atomic<bool>  g_veh_thr_on;   // whether the throttle field is ours this tick
// Count of steering writes that actually landed -- "gesture works, nothing moves" is a
// different bug from "gesture never fires", and this is what tells them apart.
extern std::atomic<uint32_t> g_veh_writes;
// The vehicle's heading in UE degrees, derived from its own travel (update_heading, below).
extern std::atomic<float> g_veh_heading;
extern std::atomic<bool>  g_veh_heading_valid;
// Smoothed ground speed in wu/s, published every update -- the render-side anchor gates its
// gameyaw correction on "actually driving", and the raw heading alone cannot say that.
extern std::atomic<float> g_veh_speed;
// Seat velocity in wu/s, for projecting the camera forward between sim ticks (update_heading, below).
extern std::atomic<float> g_unit_vx, g_unit_vy;

// Game thread, once per tick.
void vehicle_update(float dt);
// Drop the hold (level transition, dismount, menu).
void vehicle_reset();

// The component the mounted vehicle is DRAWN from (the Warthog's ".hull"), resolved on the game
// thread and published as pointer+slot for the render-side rigid seat camera to
// re-validate through TrackedObject. 0 / -1 while unmounted or unresolved.
extern std::atomic<uintptr_t> g_hog_body_ptr;
extern std::atomic<int32_t>   g_hog_body_idx;

// Game thread, once per tick: hide every mesh part of the player's own biped while mounted
// (vehhidebody), reconciled every tick, restored on dismount. Resolves the biped by walking the
// object array, every 2 s and at most ten tries per mount.
void driver_hide_update();

// Game thread, once per tick: driver_hide_update() plus the hog hull resolve (mount edge,
// retried ~2 s while unresolved, cleared on dismount). The vehicle tick slot calls it.
void vehicle_body_update();

// vehseatdirect: refresh the seat atomics from the cached object pointers. Any thread; acts only
// while stick mode holds the sim publish's normal path off. No-op when the key is 0.
void seat_direct_refresh();

} // namespace halo
