#pragma once

// THE VEHICLE CAMERA FILE (halo_vr_vehcams.json): per vehicle, an ordered list of cameras, each with
// its own placement and motion settings. Left X / left Y step to the next / previous camera in a vehicle.
//
//   { "version": 1,
//     "vehicles": {
//       "Banshee": { "match": ["BansheeVehicleActor"], "defaultCamera": 0,
//                    "cameras": [ { "name": "Onboard", "origin": "seat", "offset": [0, 0, 0],
//                                   "offsetRides": "vehicle", "viewFollows": [] }, ... ] },
//       ...
//       "default": { "cameras": [ ... ] } } }
//
// A vehicle is picked by the FIRST entry whose "match" strings (case-insensitive substrings; the
// entry's own key when absent) occur in the vehicle actor's name; "default" catches the rest. A camera
// field the file leaves out takes its built-in default. Keys starting with '_' are notes and ignored.
//
// Pure: no engine, no file I/O. VehCam.cpp reads the file and hands the text in.

#include <cstdint>
#include <string>
#include <vector>

namespace halo::vehcampresets {

enum class CamType : uint8_t { Chase = 0, FirstPerson = 1 };
// Where the camera's offset is measured FROM: the vehicle's own origin, or the seat you sit in.
enum class Origin : uint8_t { Vehicle = 0, Seat = 1 };
// What carries the offset round: the vehicle's full attitude (yaw, pitch, roll), its yaw only (a
// level offset), or your VIEW's yaw (an orbiting chase cam: turning your view swings it round).
enum class Rides : uint8_t { Vehicle = 0, VehicleYaw = 1, View = 2 };

struct Camera {
    std::string name;
    CamType type = CamType::Chase;
    Origin  origin = Origin::Vehicle;
    float   offset[3] = {0.0f, 0.0f, 0.0f};   // cm: forward, right, up
    Rides   rides = Rides::Vehicle;
    bool    follow_yaw = false, follow_pitch = false, follow_roll = false;   // the VIEW turns with the vehicle
    bool    collide = true;
    float   collide_margin = 30.0f;           // cm short of whatever blocks the offset
    int     hide_body = -1;                   // -1 = auto (hide your body when the origin is your seat)
    bool hides_body() const { return hide_body < 0 ? origin == Origin::Seat : hide_body != 0; }
};

struct Vehicle {
    std::string name;                 // the entry's key
    std::vector<std::string> match;   // lower-case substrings of the vehicle actor's name
    bool  is_default = false;         // the "default" entry: never matched by name
    int   default_camera = 0;
    int   motion_aim = -1;            // -1 = the global vehaim key; 0 / 1 = off / on for this vehicle
    std::vector<Camera> cameras;
};

struct Table {
    std::vector<Vehicle> vehicles;
};

constexpr int kMaxVehicles = 32;
constexpr int kMaxCameras  = 16;

// The built-in table: what a missing file gives, and what a first run writes out.
Table default_table();

struct ParseResult {
    bool ok{false};
    std::string error;                  // syntax / type error with "line L col C"; `out` untouched
    std::vector<std::string> ignored;   // keys that matched nothing ("vehicles.Banshee.cameras[1].ofset")
    int vehicles{0}, cameras{0};
};

// Parse `text` into `out`, REPLACING it. On any error `out` is left exactly as it was.
ParseResult table_from_json(const char* text, std::size_t len, Table& out);

// The table as the file format, with a short guide at the top. from_json(to_json(t)) == t.
std::string table_to_json(const Table& t);

// Index of the entry for this vehicle actor name (case-insensitive substring match, file order), else
// the "default" entry, else -1.
int match_vehicle(const Table& t, const std::string& actor_name);

const char* type_name(CamType t);
const char* origin_name(Origin o);
const char* rides_name(Rides r);

} // namespace halo::vehcampresets
