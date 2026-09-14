// roomscale (fork feature, Experimental): the left-stick injection state shared by the game tick and the XInput hook.
// Textual fragment, included by Plugin.cpp in its anonymous namespace. Moved verbatim; not compiled on its own.
// ROOMSCALE (Config::roomscale): the left-stick vector the game tick wants injected, already in
// the game's movement frame (aim yaw), and whether a command is standing this tick. Consumed in
// the XInput hook when the player's own stick is idle.
std::atomic<float> g_rs_lx{0.0f}, g_rs_ly{0.0f};
std::atomic<bool>  g_rs_active{false};
std::atomic<uint32_t> g_rs_injected{0};
std::atomic<float> g_rs_user_stick{0.0f};
