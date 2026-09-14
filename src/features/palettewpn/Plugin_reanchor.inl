// palettewpn (fork feature, Experimental): aim_reanchor_request(), called by the arm driver arbiter when the aim owner changes.
// Textual fragment, included by Plugin.cpp at file scope, after the plugin class. Moved verbatim; not compiled on its own.
namespace halo {
// Called by the arm driver arbiter when the aim owner changes (the author's rig and shotpoint aim
// versus the palette weapon, armdriver mode 3). Dropping the reference makes the next tick
// re-capture it against where the game is aiming now, so the view does not jump by the old owner's
// offset. The rig neutral is dropped with it. No calibration file is touched.
void aim_reanchor_request(const char* why) {
    g_have_ref = false;
    g_rig_neutral_valid = false;
    API::get()->log_info("[Halo-CampE-UEVR] AIM: reference dropped (%s) -- re-captures next tick",
                         why != nullptr ? why : "?");
}
} // namespace halo
