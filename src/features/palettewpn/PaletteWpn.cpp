#include "features/palettewpn/PaletteWpn.hpp"
#include "core/config/CfgRead.hpp"

#include "features/palettewpn/PaletteArmDriver.hpp"   // palette_weapon_mode(), the arbiter and arm hide slots
#include "Config.hpp"
#include "features/palettewpn/PaletteTwoHand.hpp"   // palette_two_hand_reset()
#include "features/palettewpn/PaletteReadbacks.hpp"
#include "features/palettewpn/PalettePoseProvider.hpp"
#include "features/palettewpn/PoseLatch.hpp"
#include "features/palettewpn/PaletteFrame.hpp"
#include "features/palettewpn/PaletteKeys.hpp"
#include "features/palettewpn/PaletteCalib.hpp"   // the aim reference slots
#include "features/palettewpn/BlamPalette.hpp"   // blam_palette_instruments_release
#include <cstdio>
#include "core/Services.hpp"

namespace halo {

namespace {
// The calibration file's palette line (write_calib_file, between the rig fit and the aim offset). Written
// whether or not the feature is enabled, so a measured fix is never dropped from the file. aimfix is not
// written here: the author's write_calib_file writes it, once.
void palette_wpn_calib_file_write(FILE* f) {
    if (g_cfg.grip_fix_valid) {
        fprintf(f,
            "# Rigid grip offset for the PALETTE weapon (Page Up freeze-and-align). Quaternion\r\n"
            "# x,y,z,w then translation x,y,z in METRES, controller frame. Applied upstream to the\r\n"
            "# controller pose; repeated captures compose. A measurement -- do not hand-edit.\r\n"
            "gripfix=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
            g_cfg.grip_fix[0], g_cfg.grip_fix[1], g_cfg.grip_fix[2], g_cfg.grip_fix[3],
            g_cfg.grip_fix[4], g_cfg.grip_fix[5], g_cfg.grip_fix[6]);
    }
}
bool palette_wpn_enabled() { CFG_HOOK_READ; return g_cfg.palette_weapon || g_cfg.arm_driver == 3; }
// The palette weapon's own hold (armdriver mode 3); a no-op when idle.
void palette_wpn_gesture_reset() { palette_two_hand_reset(); }
// The FP weapon-actor route and the rig component, while mode 3 owns.
bool palette_wpn_rig_resolve_wanted() { return palette_weapon_mode(); }
// Switched off: the arm hide it applied, then every dev instrument it left installed.
void palette_wpn_released() {
    palette_wpn_arm_hide_released();
    blam_palette_instruments_release();
}
}  // namespace

constinit const FeatureHooks kPaletteWpnHooks{
    .key      = "palettewpn",
    .parse_key = &palette_wpn_parse_key,
    .menu_command = &palette_calib_menu_command,
    .gesture_reset      = &palette_wpn_gesture_reset,
    .rig_resolve_wanted = &palette_wpn_rig_resolve_wanted,
    .sim_record_written = &palette_wpn_sim_record_written,
    .arm_driver_name             = &palette_wpn_arm_driver_name,
    .arm_driver_mode_wanted      = &palette_wpn_arm_driver_mode_wanted,
    .arm_driver_mode_unavailable = &palette_wpn_arm_driver_mode_unavailable,
    .arm_driver_key_changed      = &palette_wpn_arm_driver_key_changed,
    .arm_driver_steady           = &palette_wpn_arm_driver_steady,
    .arm_driver_active           = &palette_wpn_arm_driver_active,
    .arm_driver_release_all      = &palette_wpn_arm_driver_release_all,
    .arm_hide_component          = &palette_wpn_arm_hide_component,
    .arm_hide_held_off           = &palette_wpn_arm_hide_held_off,
    .arm_hide_needs_rig          = &palette_wpn_arm_hide_needs_rig,
    .palette_pose                = &kPalettePoseProvider,
    .aim_reference_offset        = &palette_calib_aim_reference_offset,
    .aim_calibrated              = &palette_calib_aim_calibrated,
    .aim_direct_writing          = &aim_writer_compare_direct,
    .aim_direct_write_skipped    = &palette_wpn_aim_direct_write_skipped,
    .aim_direct_written          = &palette_wpn_aim_direct_written,
    .pose_latched                = &pose_latch_lookup,
    .game_tick_after_blam_drive = &palette_wpn_game_tick_after_blam_drive,
    .game_tick_before_vehicle   = &palette_wpn_game_tick_before_vehicle,
    .game_tick_after_vehicle    = &palette_wpn_game_tick_after_vehicle,
    .game_tick_after_gestures   = &palette_wpn_game_tick_after_gestures,
    .engine_tick_start          = &palette_wpn_engine_tick_start,
    .engine_tick_end            = &palette_wpn_engine_tick_end,
    .post_engine_tick           = &palette_wpn_post_engine_tick,
    .fp_weapon_live             = &palette_wpn_fp_weapon_live,
    .rig_parent_dropped         = &palette_wpn_rig_parent_dropped,
    .rig_driver_stood_down      = &palette_wpn_rig_driver_stood_down,
    .stereo_pre_eye_instruments = &palette_wpn_stereo_pre_eye_instruments,
    .render_refresh             = &palette_wpn_render_refresh,
    .stereo_pre_eye_meters      = &palette_wpn_stereo_pre_eye_meters,
    .stereo_post_eye_sample     = &palette_wpn_stereo_post_eye_sample,
    .stereo_post_eye_late       = &palette_wpn_stereo_post_eye_late,
    .calib_file_write           = &palette_wpn_calib_file_write,
    .teardown                   = &palette_wpn_teardown,
    .aim_law_sampling           = &palette_wpn_aim_law_sampling,
    .aim_law_sampled            = &palette_wpn_aim_law_sampled,
    .game_tick_after_rig_driver = &palette_wpn_game_tick_after_rig_driver,
    .enabled  = &palette_wpn_enabled,
    .services = SVC_MARKER_ANCHOR | SVC_CAMERA_BOB | SVC_WEAPON_OBJECT,
    .released = &palette_wpn_released,
};

} // namespace halo
