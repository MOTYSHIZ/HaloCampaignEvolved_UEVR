// roomscale (fork feature, Experimental): the game tick's throttle command, read on the sim thread.
// Textual fragment, included by BlamDrive.cpp at namespace halo scope. Moved verbatim; not compiled on its own.
// Roomscale's throttle command and gates, published by the game tick in Plugin.cpp. Consumed
// here on the sim thread (mode 3: the unit object's own throttle vectors).
extern std::atomic<float>    g_rs_thr_fwd, g_rs_thr_right;
extern std::atomic<bool>     g_rs_thr_active;
extern std::atomic<uint32_t> g_rs_thr_written;
extern std::atomic<float>    g_pad_user_mag;
extern std::atomic<float>    g_dbg_face[6];
