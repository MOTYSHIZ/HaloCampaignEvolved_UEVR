#pragma once

// THE VEHICLE CAMERA FILE (halo_vr_vehcams.json): per vehicle, an ordered list of cameras, each with
// its own placement and motion settings. Left Y / left X step to the next / previous camera in a vehicle.
//
//   { "version": 1,
//     "vehicles": {
//       "Banshee": { "match": ["BansheeVehicleActor"], "defaultCamera": 0,
//                    "cameras": [ { "name": "Cockpit", "origin": "seat", "offset": [0, 0, 0],
//                                   "locationTracking": ["yaw", "pitch", "roll"],
//                                   "rotationTracking": ["yaw", "pitch", "roll"] }, ... ] },
//       ...
//       "default": { "cameras": [ ... ] } } }
//
// A vehicle is picked by the entry with the LONGEST "match" string (case-insensitive substrings; the
// entry's own key when absent) found in the name of the vehicle actor your seat belongs to, the earlier
// entry on a tie -- so a seat with an actor of its own gets its own entry wherever it sits in the file. An
// entry may also name its "seat" (driver / gunner / passenger), which tells apart seats that share one
// actor, such as the Warthog's driver and passenger; "default" catches the rest. A camera
// field the file leaves out takes its built-in default. Keys starting with '_' are notes and ignored.
// The first release of the file said "viewFollows" / "offsetRides"; both are still read.
//
// Pure: no engine, no file I/O. VehCam.cpp reads the file and hands the text in.

#include <cstdint>
#include <string>
#include <vector>

namespace halo::vehcampresets {

enum class CamType : uint8_t { Chase = 0, FirstPerson = 1 };
// Where a camera's offset is measured FROM: the vehicle's own origin, the seat you sit in, or -- Head,
// "playerhead" in the file -- the Chief's head (his body's head bone). A Head camera's tracking follows
// the CHIEF rather than the vehicle's mesh: he turns with a turret whose mesh does not (the Shade), where
// yaw tethering to the mesh does nothing.
enum class Origin : uint8_t { Vehicle = 0, Seat = 1, Head = 2 };

struct Camera {
    std::string name;                         // optional; empty = unnamed (shown by its number)
    CamType type = CamType::Chase;
    Origin  origin = Origin::Vehicle;
    float   offset[3] = {0.0f, 0.0f, 0.0f};   // cm: forward, right, up
    // LOCATION TRACKING: which of the vehicle's rotations carry the offset round. All three = rigid to
    // the vehicle; yaw only = a level offset that turns with it; none = a fixed world direction.
    // loc_view instead makes it ride YOUR VIEW -- an orbiting camera that swings round as you turn.
    bool    loc_yaw = true, loc_pitch = true, loc_roll = true;
    bool    loc_view = false;
    // ROTATION TRACKING: which of the vehicle's rotations turn your VIEW. Independent of each other:
    // pitch and roll without yaw tilt your view with the vehicle's deck while you keep your own heading.
    bool    rot_yaw = false, rot_pitch = false, rot_roll = false;
    bool    collide = true;
    float   collide_margin = 30.0f;           // cm short of whatever blocks the offset
    int     hide_body = -1;                   // -1 = auto: hidden unless the origin is the vehicle's, and
                                              //      always for a firstperson camera (its view is inside it)
    bool hides_body() const {
        if (hide_body >= 0) return hide_body != 0;
        return type == CamType::FirstPerson || origin != Origin::Vehicle;
    }
};

// THE SEAT YOU ARE IN, as the GAME reports it (BlamUnitComponent.GetSeatStates: bIsDriver / bIsGunner on
// the seat whose occupant is you). A seat can be both (a turret's seat may drive the turret AND fire it),
// so an entry's "seat" list is tested against the seat's FLAGS: "driver" = bIsDriver, "gunner" =
// bIsGunner, "passenger" = neither. Unknown = the game did not say: seat detection unavailable on this
// build, or not read yet this ride.
enum class SeatRole : int8_t { Unknown = -1, Driver = 0, Gunner = 1, Passenger = 2 };
constexpr uint8_t kSeatDriver    = 1u << static_cast<int>(SeatRole::Driver);
constexpr uint8_t kSeatGunner    = 1u << static_cast<int>(SeatRole::Gunner);
constexpr uint8_t kSeatPassenger = 1u << static_cast<int>(SeatRole::Passenger);
// The bits for a seat from its flags; 0 = the game has not said.
constexpr uint8_t seat_bits(bool driver, bool gunner) {
    return static_cast<uint8_t>((driver ? kSeatDriver : 0) | (gunner ? kSeatGunner : 0)
                              | ((!driver && !gunner) ? kSeatPassenger : 0));
}

struct Vehicle {
    std::string name;                 // the entry's key
    std::vector<std::string> match;   // lower-case substrings of the vehicle actor's name
    bool  is_default = false;         // the "default" entry: never matched by name
    int   default_camera = 0;
    int   motion_aim = -1;            // -1 = the global vehaim key; 0 / 1 = off / on for this vehicle
    bool  aim_marker = true;          // "aimMarker": a ring where the VEHICLE aims, beside the crosshair
    // "seat": the seats this entry is for, as bits (1 << SeatRole); 0 = any seat. A passenger rides the
    // same vehicle actor as its driver, so this is what tells their entries apart. Ignored on "default".
    uint8_t seats = 0;
    // "chassis": lower-case substrings of the MESH the cameras use as the vehicle's frame, when its actor
    // has more than one ("hull", "sk_wraithmortar"). Left out: a mesh named hull or body, else the nearest.
    std::vector<std::string> chassis;
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

// Index of the entry for this vehicle actor name and seat (case-insensitive substring match; an entry with
// a "seat" list is considered only when the game has named the seat and one of its flags is listed; the
// longest match wins, then an entry with a seat list over one without, then file order), else the
// "default" entry, else -1. `seat` = seat_bits(); 0 = unknown. An entry with no "seat" list is therefore
// the driver's -- and the one any other seat of that vehicle falls back to when it has none of its own.
int match_vehicle(const Table& t, const std::string& actor_name, uint8_t seat = 0);
inline int match_vehicle(const Table& t, const std::string& actor_name, SeatRole role) {
    return match_vehicle(t, actor_name, role == SeatRole::Unknown ? uint8_t{0} : static_cast<uint8_t>(1u << static_cast<int>(role)));
}

const char* type_name(CamType t);
const char* origin_name(Origin o);
const char* seat_role_name(SeatRole s);   // "driver" / "gunner" / "passenger" / "unknown"
// For readouts and logs: "Driver", "Gunner", "Driver, Gunner", "Passenger"; "" = unknown.
std::string seat_text(uint8_t seat);

// For readouts and logs: "Yaw, Pitch, Roll" / "Pitch, Roll" / "None"; location adds "Your view (orbit)".
std::string rotation_tracking_text(const Camera& c);
std::string location_tracking_text(const Camera& c);

} // namespace halo::vehcampresets
