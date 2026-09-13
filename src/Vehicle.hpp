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
// The sim-thread write lives in BlamDrive.cpp, because the record only resolves through
// gs:[0x58] on the sim thread.
// ============================================================================================
#pragma once

#include <atomic>

#include "Math.hpp"
#include "uevr/API.hpp"

namespace halo {

// Published by Vehicle.cpp (game thread), consumed on the sim thread.
// g_veh_active gates the write: false = the pad owns the record, exactly as before.
extern std::atomic<bool>  g_veh_active;
extern std::atomic<float> g_veh_steer;    // -1..1, already signed for the record
extern std::atomic<float> g_veh_thr;      // -1..1 forward, or NaN-free 0 when not driven
extern std::atomic<bool>  g_veh_thr_on;   // whether the throttle field is ours this tick
// Count of steering writes that actually landed -- "gesture works, nothing moves" is a
// different bug from "gesture never fires", and this is what tells them apart.
extern std::atomic<uint32_t> g_veh_writes;
// The vehicle's heading in UE degrees, derived from its own travel (see Vehicle.cpp).
extern std::atomic<float> g_veh_heading;
extern std::atomic<bool>  g_veh_heading_valid;
// Smoothed ground speed in wu/s, published every update -- the render-side anchor gates its
// gameyaw correction on "actually driving", and the raw heading alone cannot say that.
extern std::atomic<float> g_veh_speed;
// Seat velocity in wu/s, for projecting the camera forward between sim ticks (see Vehicle.cpp).
extern std::atomic<float> g_unit_vx, g_unit_vy;

// Game thread, once per tick.
void vehicle_update(float dt);
// Drop the hold (level transition, dismount, menu).
void vehicle_reset();

} // namespace halo
