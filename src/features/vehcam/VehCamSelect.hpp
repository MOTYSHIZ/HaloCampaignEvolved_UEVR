#pragma once

// WHICH CAMERA, FOR WHICH VEHICLE. The runtime half of halo_vr_vehcams.json (VehCamPresets.hpp holds
// the file format): reading and live-reloading the file, identifying the vehicle you are in, stepping
// through its cameras on left X / left Y, and publishing the selected camera for the render thread.
//
// THREADS. The table, the file and the selection live on the GAME thread. What the eye callback,
// the view override and the sim-thread aim write need is a plain copy (VehActiveCam), double-buffered
// and published whole, so no reader ever sees half a camera. The input hook only posts a step.

#include <cstdint>
#include <string>

namespace halo {

struct VehActiveCam {
    bool    valid = false;                    // a vehicle is identified and one of its cameras selected
    uint8_t type = 0;                         // vehcampresets::CamType
    uint8_t origin = 0;                       // vehcampresets::Origin
    uint8_t rides = 0;                        // vehcampresets::Rides
    bool    follow_yaw = false, follow_pitch = false, follow_roll = false;
    bool    collide = true;
    bool    hide_body = false;
    float   offset[3] = {-450.0f, 0.0f, 180.0f};
    float   collide_margin = 30.0f;
    int     motion_aim = -1;                  // -1 = the vehaim key, else this vehicle's override
    int     index = 0, count = 0;             // for the log line
};

// Any thread: the selected camera as last published.
VehActiveCam veh_active_cam();

// GAME thread, once at startup: remember the path, write the built-in cameras there if the player
// has no file (never overwrites one), and load it.
void vehcam_presets_init(const char* path);
// GAME thread, on the config poll (~2 s): re-read the file when its write time changes.
void vehcam_presets_poll();

// GAME thread, every tick. `in_vehicle` = our camera system is running in a vehicle; `chassis` = the
// resolved vehicle mesh (0 = not yet); `chassis_name` its full object name. Identifies the vehicle when
// the chassis changes, applies left X / left Y steps, keeps the published camera current, and owns
// UEVR's decoupled-pitch switch while a camera that tilts with the vehicle is up.
void vehcam_select_tick(bool in_vehicle, uintptr_t chassis, const std::wstring& chassis_name);

// Any thread (the input hook): +1 = next camera, -1 = previous.
void veh_cam_step(int dir);

} // namespace halo
