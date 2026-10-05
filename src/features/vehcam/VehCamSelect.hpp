#pragma once

// WHICH CAMERA, FOR WHICH VEHICLE. The runtime half of halo_vr_vehcams.json (VehCamPresets.hpp holds
// the file format): reading and live-reloading the file, identifying the vehicle you are in, stepping
// through its cameras on left Y and the current camera's tethering modes on left X, and publishing the
// selected camera, in its mode, for the render thread.
//
// THREADS. The table, the file and the selection live on the GAME thread. What the eye callback,
// the view override and the sim-thread aim write need is a plain copy (VehActiveCam), double-buffered
// and published whole, so no reader ever sees half a camera. The input hook only posts a step or a
// controls toggle.

#include <cstdint>
#include <string>
#include <vector>

namespace halo {

// The camera IN ITS TETHERING MODE: the tracking, offset and aim ring are the mode's (VehCamPresets'
// Camera::mode), everything else the camera's.
struct VehActiveCam {
    bool    valid = false;                    // a vehicle is identified and one of its cameras selected
    uint8_t type = 0;                         // vehcampresets::CamType
    uint8_t origin = 0;                       // vehcampresets::Origin
    char    socket[64] = {};                  // origin Socket: the bone / socket name ("Part/Name"), else empty
    bool    loc_yaw = true, loc_pitch = true, loc_roll = true, loc_view = false;   // locationTracking
    bool    rot_yaw = false, rot_pitch = false, rot_roll = false;                  // rotationTracking
    bool    collide = true;
    bool    hide_body = false;
    bool    hide_head = false;                // "hideHead": the Chief's head hidden (true first person)
    float   offset[3] = {-450.0f, 0.0f, 180.0f};
    // "leashMin" / "leashMax" (the mode's, else the camera's): the box your head stays inside, cm from the
    // origin on the offset's axes, like `offset`; +-1e9 (vehcampresets::kUnleashed) = no limit that way.
    // `leashed` = any limit at all, so the eye skips the whole clamp for the usual unleashed camera.
    float   leash_min[3] = {-1.0e9f, -1.0e9f, -1.0e9f};
    float   leash_max[3] = { 1.0e9f,  1.0e9f,  1.0e9f};
    bool    leashed = false;
    float   collide_margin = 30.0f;
    int     motion_aim = -1;                  // -1 = the vehaim key, else this vehicle's (file or left stick click)
    bool    aim_marker = true;                // "aimMarker": the ring where the vehicle aims (the eye poses it)
    int     index = 0, count = 0;             // the camera, of how many
    int     mode = 0, mode_count = 1;         // its tethering mode, of how many
    // Bumped by a CHANGE of camera the player made or got -- getting in, left Y (camera), left X
    // (tethering mode), the left stick click (controls), a seat switch -- never by a file reload. The eye
    // acts on a change: recenter_gen lines what aims the vehicle up with where it aims (vehcamrecenter);
    // place_gen puts your head back on the camera's point (vehcamrecenterpos), wherever you had leaned
    // or walked to in the room.
    uint32_t recenter_gen = 0;
    uint32_t place_gen = 0;
};

// Any thread: the selected camera as last published.
VehActiveCam veh_active_cam();

// Any thread, one atomic load: a camera from the file is selected for the vehicle you are in (chase or
// first-person) -- i.e. our vehicle controls are live. For the input hook, which runs far too often to copy
// the whole VehActiveCam per call.
bool veh_cam_selected();

// Any thread, one atomic load: a camera is selected AND the game names your seat a passenger's (neither the
// driver's nor a gunner's) AND vehpassswap is on -- the seat where the left trigger switches your weapon.
// False until the game has named the seat. For the input hook.
bool veh_seat_passenger();

// GAME thread, once at startup: remember the path, write the built-in cameras there if the player
// has no file (never overwrites one), and load it.
void vehcam_presets_init(const char* path);
// GAME thread, on the config poll (~2 s): re-read the file when its write time changes.
void vehcam_presets_poll();

// GAME thread, every tick. `in_vehicle` = our camera system is running in a vehicle; `chassis` = the
// resolved vehicle mesh (0 = not yet); `vehicle_name` = the full object name the camera file is matched
// against -- the vehicle actor your SEAT belongs to when the game has said, else the chassis mesh's;
// `seat` = vehcampresets::seat_bits as the game reports them (0 = unknown). Identifies the vehicle when
// the chassis or the seat changes, applies left Y / left X steps, keeps the published camera current,
// remembers your camera, its mode and your controls per seat across sessions, and owns UEVR's
// decoupled-pitch switch while a camera that tilts with the vehicle is up.
void vehcam_select_tick(bool in_vehicle, uintptr_t chassis, const std::wstring& vehicle_name, int seat);

// GAME thread: the "chassis" names of the entry this vehicle and seat would get (empty = none given).
std::vector<std::string> vehcam_chassis_hint(const std::wstring& vehicle_name, int seat);

// GAME thread: the parts of the vehicle the selected camera hides ("hideMeshes": the camera's own list,
// else its seat's) -- empty while no camera is selected. `rev` (optional) changes whenever the list does,
// so a caller can act on a change without comparing lists every tick.
const std::vector<std::string>& vehcam_hide_meshes(uint32_t* rev = nullptr);

// Any thread (the input hook, left Y): +1 = next camera, -1 = previous.
void veh_cam_step(int dir);
// Any thread (the input hook, left X): +1 = the current camera's next tethering mode, -1 = previous.
void veh_cam_mode_step(int dir);
// Any thread (the input hook, left X or Y held a second): reset the view in the camera and mode you are in
// -- lined up with the vehicle's aim, your head back on the camera's point.
void veh_cam_view_reset();

// Any thread (the input hook, left stick click): flip THIS vehicle between motion controls (the
// controller aims) and stick controls (the right stick aims, as on a gamepad). Applied on the game
// tick, said on the text panel, and remembered per vehicle entry for the session -- like the camera.
void veh_ctrl_toggle();

} // namespace halo
