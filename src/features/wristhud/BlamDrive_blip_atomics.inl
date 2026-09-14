// wristhud (fork feature, Experimental): the radar blips published by the sim scan.
// Textual fragment, included by BlamDrive.cpp at namespace halo scope, beside the grenade-track handoff. Moved verbatim; not compiled on its own.
std::atomic<int>   g_blip_count{0};
std::atomic<float> g_blip_dx[MAX_BLIPS], g_blip_dy[MAX_BLIPS];
std::atomic<int>   g_blip_team[MAX_BLIPS];
std::atomic<bool>  g_blip_moving[MAX_BLIPS];
std::atomic<uint32_t> g_blip_id[MAX_BLIPS];
std::atomic<uint32_t> g_blip_raw[MAX_BLIPS];
std::atomic<uint32_t> g_blip_type[MAX_BLIPS];
std::atomic<float> g_blip_wx[MAX_BLIPS], g_blip_wy[MAX_BLIPS], g_blip_wz[MAX_BLIPS];
std::atomic<float> g_blip_speed[MAX_BLIPS];
std::atomic<int>   g_blip_dtm[MAX_BLIPS];
std::atomic<int>   g_blip_run[MAX_BLIPS];
