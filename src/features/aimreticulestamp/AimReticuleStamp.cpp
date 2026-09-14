#include "features/aimreticulestamp/AimReticuleStamp.hpp"

#include "Config.hpp"
#include "Math.hpp"   // clampf

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include "Math.hpp"
#include "Reticule.hpp"   // g_ret_scale_mul
#include "XrLayer.hpp"    // the compositor reticule
#include "core/PalettePose.hpp"
#include "core/ReticuleDepth.hpp"
#include "core/host/PluginState.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include "core/Services.hpp"

namespace halo {

namespace {
bool aim_reticule_stamp_parse_key(const char* key, const char* val, double v) {
    (void)val; (void)v;
    if (_stricmp(key, "aimreticulestamp") == 0) { g_cfg.aim_reticule_stamp = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    return false;
}

bool aim_reticule_stamp_enabled() { return g_cfg.aim_reticule_stamp != 0; }

// The stamped placement (modes 1/2) owns the compositor reticle's publish at render, and only while the
// palette weapon owns the aim and its latch stamps the intent. Without it the render publish never fires
// and the compositor reticle goes dark with no message, so the tick publish takes over instead.
int reticule_render_publish_mode() {
    if (!palette_pose_owns_aim() || !palette_pose_stamp_available()) return 0;
    return g_cfg.aim_reticule_stamp;
}

// The stereo post-callback, after the eye publish: the stamped reticle placed at render rate.
void stereo_post_eye_publish(int index) {
    const auto& ps = host::g_plugin_state;
    const std::atomic<bool>&  g_stick_mode = *ps.stick_mode;
    const std::atomic<bool>&  g_have_view_pos = *ps.have_view_pos;
    const std::atomic<float>& g_view_pos_x = *ps.view_pos_x;
    const std::atomic<float>& g_view_pos_y = *ps.view_pos_y;
    const std::atomic<float>& g_view_pos_z = *ps.view_pos_z;
    const auto layer_anchor = ps.layer_anchor;
    // RETSTAMP render publish (modes 1/2). Same frame as the eye note that follows: this frame's
    // view position, the stamped intent, the latest trace depth. No smoothing.
    if (index == 0 && reticule_render_publish_mode() != 0 && g_cfg.aim_reticule && g_cfg.xr_layer
        && !g_stick_mode.load() && g_have_view_pos.load()) {
        const long long rs_now = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const long long rs_dms = reticule_depth_ms();
        const bool two_back = (g_cfg.aim_reticule_stamp == 1);
        float ay = 0.0f, ap = 0.0f;
        const bool rs_ok = halo::palette_pose_stamped_intent(two_back, &ay, &ap);
        if (rs_ok && rs_dms != 0 && rs_now - rs_dms < 200) {
            const float rd = reticule_depth();
            const float cpr = std::cos(ap * DEG2RAD);
            const Vec3 rf{cpr * std::cos(ay * DEG2RAD), cpr * std::sin(ay * DEG2RAD), std::sin(ap * DEG2RAD)};
            const Vec3 rtarget{g_view_pos_x.load() + rf.x * rd,
                               g_view_pos_y.load() + rf.y * rd,
                               g_view_pos_z.load() + rf.z * rd};
            halo::xrlayer_note_publish_gate(0);
            halo::xrlayer_notice_reticule(layer_anchor(halo::XRLAYER_SLOT_RETICULE, rtarget),
                                          g_ret_scale_mul.load());
            halo::palette_pose_mark(46, ay, ap, rd, (float)g_cfg.aim_reticule_stamp);
        }
    }
}
}  // namespace

constinit const FeatureHooks kAimReticuleStampHooks{
    .key      = "aimreticulestamp",
    .parse_key = &aim_reticule_stamp_parse_key,
    .reticule_render_publish_mode = &reticule_render_publish_mode,
    .stereo_post_eye_publish = &stereo_post_eye_publish,
    .enabled  = &aim_reticule_stamp_enabled,
    .services = SVC_POSE_INTENTS,
};

} // namespace halo
