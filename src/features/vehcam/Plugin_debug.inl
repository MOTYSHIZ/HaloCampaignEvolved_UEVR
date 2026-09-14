// vehcam (fork feature, Experimental): the seat camera evidence the VEHSEAT log line reads (render thread).
// Textual fragment, included by Plugin.cpp in its anonymous namespace. Moved verbatim; not compiled on its own.
// SEAT CAMERA EVIDENCE (vehlog). Written by the vehcam block on the frame-computing eye, read by
// the once-a-second VEHSEAT line in the same callback. Render thread only.
enum VehCamPath { VCP_NONE = 0, VCP_RIGID, VCP_HOLD, VCP_CHASE, VCP_SYNTH };
struct VehCamDbg {
    uint32_t frames[5] = {0, 0, 0, 0, 0};   // per path since the last VEHSEAT line; [0] = gate off
    int      last_path = -1;
    bool     have_hull = false;
    double   hull[3] = {0.0, 0.0, 0.0};
    double   off[3] = {0.0, 0.0, 0.0};
    double   hspeed = 0.0;                   // drawn hull speed, cm/s
    float    stale_ms = 0.0f;
    bool     learning = false;
    uint32_t learn_frames = 0, snaps = 0, snaps_refused = 0;
    bool     hull_dead = false;              // vehcamhullcheck ruled the hull component not the drawn vehicle
    // WRITE SURVIVAL: the seat position the pre callback wrote on eye 0, and what the post callback
    // (after UEVR's HMD transform) actually rendered. A gap larger than any head offset means
    // something replaced the camera after we wrote it.
    double   fc[3] = {0.0, 0.0, 0.0};
    bool     wrote = false;
    double   post_max = 0.0;
    uint32_t post_frames = 0, post_bad = 0;
};
VehCamDbg g_vcd;
