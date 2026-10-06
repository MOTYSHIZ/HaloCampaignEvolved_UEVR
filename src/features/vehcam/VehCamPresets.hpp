#pragma once

// THE VEHICLE CAMERA FILE (halo_vr_vehcams.json): per vehicle, an ordered list of cameras, each with
// its own placement and motion settings, and its TETHERING MODES. Left Y steps to the next camera in a
// vehicle; left X steps through the current camera's modes.
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
#include <utility>
#include <vector>

namespace halo::vehcampresets {

enum class CamType : uint8_t { Chase = 0, FirstPerson = 1 };
// Where a camera's offset is measured FROM: the vehicle's own origin, the seat you sit in, or -- Head,
// "playerhead" in the file -- the Chief's head (his body's head bone). A Head camera's tracking follows
// the CHIEF rather than the vehicle's mesh: he turns with a turret whose mesh does not (the Shade), where
// yaw tethering to the mesh does nothing. Socket: ANY OTHER NAME in the file is a bone or socket on the
// vehicle -- its own mesh first, then the parts near your seat (a turret that is an actor of its own, like
// the Scorpion's cannon); "Part/Name" picks the part ("ScorpionCannon/AimYaw"). Its tracking follows that
// bone, so a camera can ride a gun that turns on its own. Not found: the vehicle's origin stands in.
enum class Origin : uint8_t { Vehicle = 0, Seat = 1, Head = 2, Socket = 3 };

// THE CAMERA'S LEASH ("leashMin" / "leashMax"): a box YOUR HEAD stays inside, in the SAME SPACE AS "offset"
// -- [forward, right, up], cm FROM THE ORIGIN on the offset's directions, so a limit is a place and the two
// are tuned together (an offset 360 up with a max of 360 up is a camera already at its ceiling). For seats
// tuned to tight tolerances: past a limit the view stops following your head that way, so leaning cannot put
// your eyes through the canopy or into the gun. Any value may take any sign; the camera's own point is kept
// inside the box when it is applied (vehcammath::leash_relative), so a leash only ever stops you, never moves
// you. kUnleashed on an axis = no limit that way (null in the file, or left out).
constexpr float kUnleashed = 1.0e9f;
// Any axis of this side limited.
inline bool leash_set(const float v[3]) {
    for (int k = 0; k < 3; ++k) if (v[k] > -kUnleashed && v[k] < kUnleashed) return true;
    return false;
}

// ONE TETHERING MODE of a camera. Left Y steps cameras; left X steps the current camera's modes. A mode
// sets what carries the camera's position round (locationTracking), what turns your view
// (rotationTracking), where it sits (offset -- a tethered view often wants a slightly different spot),
// what that is measured from (origin) and the box your head stays inside (leashMin / leashMax), and
// may turn the vehicle aim ring on or off; anything it leaves out is the camera's own. So one camera can be "Cockpit" held still AND tethered, switched with one button
// while filming -- and a turret's tethered mode can ride the Chief ("playerhead", who turns with the gun)
// while its untethered mode stays on the seat.
struct Tether {
    std::string name;
    float offset[3] = {0.0f, 0.0f, 0.0f};     // cm: forward, right, up (the camera's, unless the mode says)
    float leash_min[3] = {-kUnleashed, -kUnleashed, -kUnleashed};   // "leashMin" (the camera's, unless the mode says)
    float leash_max[3] = { kUnleashed,  kUnleashed,  kUnleashed};   // "leashMax"
    bool loc_yaw = true, loc_pitch = true, loc_roll = true, loc_view = false;
    bool rot_yaw = false, rot_pitch = false, rot_roll = false;
    int  aim_marker = -1;                     // -1 = the camera's, else the vehicle entry's; 0 / 1 = off / on
    int  origin = -1;                         // -1 = the camera's; else an Origin
    std::string origin_socket;                // origin Socket: the bone / socket name ("Part/Name" allowed)
};

struct Camera {
    std::string name;                         // optional; empty = unnamed (shown by its number)
    CamType type = CamType::Chase;
    Origin  origin = Origin::Vehicle;
    std::string origin_socket;                // origin Socket: the bone / socket name ("Part/Name" allowed)
    float   offset[3] = {0.0f, 0.0f, 0.0f};   // cm: forward, right, up
    float   leash_min[3] = {-kUnleashed, -kUnleashed, -kUnleashed};   // "leashMin": see kUnleashed
    float   leash_max[3] = { kUnleashed,  kUnleashed,  kUnleashed};   // "leashMax"
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
    int     hide_head = -1;                   // "hideHead": -1 = the vehicle entry's; 0 / 1
    int     aim_marker = -1;                  // "aimMarker": -1 = the vehicle entry's; 0 / 1
    // "hideMeshes": this camera's own list (Vehicle::hide_meshes says what it names), used only when
    // hide_meshes_set -- left out or null = the seat's list, [] = nothing hidden in this camera.
    std::vector<std::string> hide_meshes;
    bool    hide_meshes_set = false;
    // "tethering": the modes left X steps through. Empty = one mode, the camera's own tracking above.
    std::vector<Tether> tethering;
    bool hides_body() const { return hides_body(origin); }
    // ...from origin `o` -- a tethering mode may measure from somewhere else than its camera.
    bool hides_body(Origin o) const {
        if (hide_body >= 0) return hide_body != 0;
        // At your seat or your head the view is inside you; a vehicle part is not.
        return type == CamType::FirstPerson || o == Origin::Seat || o == Origin::Head;
    }
    // Mode `m`'s origin: its own, else this camera's -- and, for a Socket origin, the name.
    Origin origin_of(const Tether& m) const { return m.origin >= 0 ? static_cast<Origin>(m.origin) : origin; }
    const std::string& socket_of(const Tether& m) const { return m.origin >= 0 ? m.origin_socket : origin_socket; }
    int mode_count() const { return tethering.empty() ? 1 : static_cast<int>(tethering.size()); }
    // Mode i (wrapped into range): the camera's own tracking when it lists no tethering.
    Tether mode(int i) const {
        if (tethering.empty()) {
            Tether t;
            t.offset[0] = offset[0]; t.offset[1] = offset[1]; t.offset[2] = offset[2];
            for (int k = 0; k < 3; ++k) { t.leash_min[k] = leash_min[k]; t.leash_max[k] = leash_max[k]; }
            t.loc_yaw = loc_yaw; t.loc_pitch = loc_pitch; t.loc_roll = loc_roll; t.loc_view = loc_view;
            t.rot_yaw = rot_yaw; t.rot_pitch = rot_pitch; t.rot_roll = rot_roll;
            return t;
        }
        const int n = static_cast<int>(tethering.size());
        return tethering[static_cast<size_t>(((i % n) + n) % n)];
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
    // "enabled": false -- NONE OF THIS in the seats this entry matches: the game's own camera and controls,
    // exactly as with no entry at all (the user, 2026-09-27, for the Pelican ride at a level's start, which
    // was fine before any of it). The entry still MATCHES, so it beats "default"; the selection then stands
    // down, and with it the eye, the hand aim, the buttons and the readout. Its cameras may be left out, and
    // any it lists are kept, so switching it back on brings them back.
    bool  enabled = true;
    int   default_camera = 0;
    int   default_mode = 0;           // "defaultMode": the default camera's tethering mode to start in
    int   motion_aim = -1;            // -1 = the global vehaim key; 0 / 1 = off / on for this vehicle
    bool  aim_marker = true;          // "aimMarker": a ring where the VEHICLE aims, beside the crosshair
                                      // (a camera or a tethering mode may say otherwise)
    bool  hide_head = false;          // "hideHead": hide the player's HEAD in this seat -- true first person
                                      // from a camera at the head (a camera may say otherwise). Off by default.
    // "leashMin" / "leashMax" for every camera of this seat: the seat's own box, which a camera and then a
    // tethering mode may override (null at either = no leash there). The parser resolves the cascade, so
    // each Camera and Tether already holds its effective values.
    float leash_min[3] = {-kUnleashed, -kUnleashed, -kUnleashed};
    float leash_max[3] = { kUnleashed,  kUnleashed,  kUnleashed};
    // "seat": the seats this entry is for, as bits (1 << SeatRole); 0 = any seat. A passenger rides the
    // same vehicle actor as its driver, so this is what tells their entries apart. Ignored on "default".
    uint8_t seats = 0;
    // "chassis": lower-case substrings of the MESH the cameras use as the vehicle's frame, when its actor
    // has more than one ("hull", "sk_wraithmortar"). Left out: a mesh named hull or body, else the nearest.
    std::vector<std::string> chassis;
    // "hideMeshes": PARTS OF THE VEHICLE hidden while you sit in this seat -- lower-case substrings of a mesh
    // component's name or of the mesh asset it draws. For a camera inside the vehicle, which sees out
    // through the hull's back faces: the internals a damaged vehicle reveals (the Banshee) come up inside
    // that view and block it. A camera may give its own list. Left out = none. VehMeshes.cpp applies it.
    std::vector<std::string> hide_meshes;
    std::vector<Camera> cameras;
};

struct Table {
    std::vector<Vehicle> vehicles;
};

constexpr int kMaxVehicles   = 32;
constexpr int kMaxCameras    = 16;
constexpr int kMaxHideMeshes = 32;   // names in one "hideMeshes" list: every part of the vehicle is tested against each

// The built-in table: what a missing file gives, and what a first run writes out -- a CHECKPOINT
// CANONIZATION of a player's tuned file (VehCamDefaults.inc, written by Scripts\VehCams-Tool.ps1 canonize).
Table default_table();
// The programmatic starter set default_table() falls back to if the canonized table ever failed to read
// (a guard, never the expected path: the standalone test proves the canonized one reads).
Table starter_table();

struct ParseResult {
    bool ok{false};
    std::string error;                  // syntax / type error with "line L col C"; `out` untouched
    std::vector<std::string> ignored;   // keys that matched nothing ("vehicles.Banshee.cameras[1].ofset")
    int vehicles{0}, cameras{0};
};

// Parse `text` into `out`, REPLACING it. On any error `out` is left exactly as it was.
ParseResult table_from_json(const char* text, std::size_t len, Table& out);

// The table as the file format, with a short guide at the top (`guide` = false leaves it out -- the
// built-in table's own source, VehCamDefaults.inc). from_json(to_json(t)) == t.
std::string table_to_json(const Table& t, bool guide = true);

// THE ONE-TIME MOVE TO TETHERING MODES, by the naming players already used. In each vehicle, a camera
// named "Tethered ..." joins the camera before it that is not ("Cockpit" / "Tethered Cockpit",
// "HangGlider" / "Tethered Yaw HangGlider" / "Tethered HangGlider") when both share type, origin,
// collision and body/head hiding: they become ONE camera, named for the first, with a mode each. A mode is
// named for what its camera's name adds ("Tethered Yaw"), "Tethered" when that is all it adds, and the
// first "Untethered" when it turns no rotation. Offsets may differ between modes. defaultCamera /
// defaultMode follow the camera they pointed at. Returns how many cameras were folded into another.
// `moved` (optional): per vehicle, each old camera's new (camera, mode) -- for anything else that named
// the old cameras, such as the per-seat memory file.
int merge_tethered(Table& t, std::vector<std::vector<std::pair<int, int>>>* moved = nullptr);

// The effective settings of camera `ci`, mode `mi` of vehicle `v`: the aim ring and the head hide, each
// the mode's, else the camera's, else the vehicle entry's.
bool effective_aim_marker(const Vehicle& v, int ci, int mi);
bool effective_hide_head(const Vehicle& v, int ci);
// ...and the parts of the vehicle hidden in camera `ci`: the camera's own "hideMeshes" when it gives one
// (possibly empty), else the vehicle entry's.
const std::vector<std::string>& effective_hide_meshes(const Vehicle& v, int ci);

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
// As the file writes it: the socket's own name for a Socket origin, else origin_name.
std::string origin_text(Origin o, const std::string& socket);
const char* seat_role_name(SeatRole s);   // "driver" / "gunner" / "passenger" / "unknown"
// For readouts and logs: "Driver", "Gunner", "Driver, Gunner", "Passenger"; "" = unknown.
std::string seat_text(uint8_t seat);

// For readouts and logs: "Yaw, Pitch, Roll" / "Pitch, Roll" / "None"; location adds "Your view (orbit)".
std::string rotation_tracking_text(const Camera& c);
std::string location_tracking_text(const Camera& c);
std::string rotation_tracking_text(const Tether& t);
std::string location_tracking_text(const Tether& t);
// For readouts and logs: the limits as places from the origin, like the offset -- "up at most 360 cm",
// "forward -30 to 15, up 40 to 65 cm"; "None".
std::string leash_text(const float leash_min[3], const float leash_max[3]);

// Left Y onto camera `to` from a view in mode `from`: the mode that keeps what you chose -- one of the same
// name ("Tethered" stays tethered), else one turning your view on the same axes (an untethered choice stays
// untethered under any name), else the first. A comfort choice is not undone by changing where you sit.
int carry_mode(const Camera& to, const Tether& from);

} // namespace halo::vehcampresets
