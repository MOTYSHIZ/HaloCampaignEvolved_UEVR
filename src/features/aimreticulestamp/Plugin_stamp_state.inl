// aimreticulestamp (fork feature, Experimental): the latest on-foot trace depth and the render placement gate.
// Textual fragment, included by Plugin.cpp in its anonymous namespace, before onfoot_reticule_tick(). Moved verbatim; not compiled on its own.
// RETSTAMP: the latest on-foot trace depth (cm) and when it was taken, for the render publish.
static std::atomic<float>     g_ret_last_d{0.0f};
static std::atomic<long long> g_ret_last_d_ms{0};
// RETSTAMP render placement (aimreticulestamp 1/2) draws the STAMPED hand intent, and that stamp is
// only taken while the pose latch runs in palette weapon mode: poselatch 0 never stores it, and
// poselatch 3 stores it from the XInput-rate law (aimrate=1) only. Without it the render publish never
// fires and the compositor reticle goes dark with no message, so the tick publish takes over instead.
static bool reticule_stamp_render_active() {
    if (!palette_weapon_mode() || g_cfg.aim_reticule_stamp == 0) return false;
    if (g_cfg.pose_latch == 0) return false;
    if (g_cfg.pose_latch == 3 && !g_cfg.aim_rate_render) return false;
    return true;
}
