// <Windows.h>'s min/max macros would eat every std::min/std::max below. Before ANY include, since a
// project header may pull <Windows.h> in first.
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "XrText.hpp"

#include "Config.hpp"
#include "Markers.hpp"          // g_view_base_*: the room's rotation the panel is placed in
#include "XrLayer.hpp"
#include "XrTextRaster.hpp"
#include "core/host/PluginState.hpp"

#include <Windows.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace halo {

namespace {

// ---- the panel showing now (game thread) ------------------------------------------------------------
bool            s_showing = false;
uint64_t        s_end_ms = 0;
XrTextAnchor    s_anchor = XrTextAnchor::View;
Vec3            s_pos{};        // View: the offset from the mono view position, fixed at show; World: the point
float           s_w_cm = 0.0f, s_h_cm = 0.0f;
std::vector<uint8_t> s_px;      // reused between notices

} // namespace

XrTextPlacement xrtext_default_placement() {
    XrTextPlacement p;
    p.anchor = XrTextAnchor::View;
    p.dist_cm = g_cfg.xr_text_dist_cm;
    p.right_cm = g_cfg.xr_text_right_cm;
    p.up_cm = g_cfg.xr_text_up_cm;
    p.width_cm = g_cfg.xr_text_width_cm;
    return p;
}

XrTextTiming xrtext_default_timing() {
    XrTextTiming t;
    t.fade_in_ms = g_cfg.xr_text_fade_in_ms;
    t.hold_ms = g_cfg.xr_text_hold_ms;
    t.fade_out_ms = g_cfg.xr_text_fade_out_ms;
    return t;
}

bool xrtext_show(const std::string& markup) {
    return xrtext_show(markup, xrtext_default_placement(), xrtext_default_timing());
}

bool xrtext_show(const std::string& markup, const XrTextPlacement& where, const XrTextTiming& when) {
    if (!g_cfg.xr_text || !g_cfg.xr_layer) return false;
    int w = 0, h = 0;
    xrlayer_text_cell(&w, &h);
    if (w <= 0 || h <= 0) return false;

    // WHERE: placed once, now. In front of where you look, then held IN YOUR ROOM -- in the view base's
    // own axes, rebuilt every frame against the base as it is then (xrlayer_set_quad_view_relative).
    // So it stays put when the room moves with a vehicle AND when the room turns with one (a camera
    // that tracks the vehicle's yaw, pitch or roll), and it does not chase your head. "Level" and "up"
    // here are the ROOM's: on a tilted cockpit camera the panel sits relative to the cockpit's floor.
    // Or at a world point, for a notice about a place or a thing.
    Vec3 pos{};
    if (where.anchor == XrTextAnchor::View) {
        Vec3 fwd{}, up{}, mono{};
        if (!xrlayer_view_basis(&fwd, &up) || !xrlayer_mono_view_pos(&mono)) return false;
        const auto& ps = host::g_plugin_state;
        Vec3 head = mono;
        if (ps.have_eye_pos->load(std::memory_order_relaxed))
            head = Vec3{ps.eye_pos_x->load(std::memory_order_relaxed), ps.eye_pos_y->load(std::memory_order_relaxed),
                        ps.eye_pos_z->load(std::memory_order_relaxed)};
        // The view base's axes (UE FRotationMatrix rows): the room's forward / right / up in the world.
        const double D2R = 0.01745329252;
        const double bp = (double)g_view_base_pitch.load(std::memory_order_relaxed) * D2R;
        const double by = (double)g_view_base_yaw.load(std::memory_order_relaxed) * D2R;
        const double br = (double)g_view_base_roll.load(std::memory_order_relaxed) * D2R;
        const double cp = std::cos(bp), sp = std::sin(bp), cy = std::cos(by), sy = std::sin(by);
        const double cr = std::cos(br), sr = std::sin(br);
        const double X[3] = { cp * cy, cp * sy, sp };
        const double Y[3] = { sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp };
        const double Z[3] = { -(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp };
        auto to_room = [&](double wx, double wy, double wz, double out[3]) {
            out[0] = wx * X[0] + wy * X[1] + wz * X[2];
            out[1] = wx * Y[0] + wy * Y[1] + wz * Y[2];
            out[2] = wx * Z[0] + wy * Z[1] + wz * Z[2];
        };
        double f[3], h[3];
        to_room(fwd.x, fwd.y, fwd.z, f);                                  // where you look, in the room
        to_room(head.x - mono.x, head.y - mono.y, head.z - mono.z, h);    // your head, in the room
        double fx = f[0], fy = f[1];
        const double fl = std::sqrt(fx * fx + fy * fy);
        if (fl < 1e-3) { fx = 1.0; fy = 0.0; } else { fx /= fl; fy /= fl; }   // level IN THE ROOM
        const double rx = -fy, ry = fx;                                    // UE: the right of that
        pos = Vec3{(float)(h[0] + fx * where.dist_cm + rx * where.right_cm),
                   (float)(h[1] + fy * where.dist_cm + ry * where.right_cm),
                   (float)(h[2] + where.up_cm)};
    } else {
        pos = where.world;
    }

    // WHAT: rasterised now, handed to the layer, which uploads it on its own thread.
    xrtextraster::Style st;
    st.scale = g_cfg.xr_text_scale;
    st.bg = g_cfg.xr_text_bg;
    if (!xrtextraster::rasterise(markup, w, h, xrlayer_text_is_bgra(), st, s_px)) return false;
    const uint64_t now = GetTickCount64();
    const uint32_t fin = (uint32_t)std::max(0, when.fade_in_ms);
    const uint32_t hld = (uint32_t)std::max(0, when.hold_ms);
    const uint32_t fot = (uint32_t)std::max(0, when.fade_out_ms);
    if (!xrlayer_text_set(s_px.data(), w, h, now, fin, hld, fot)) return false;

    s_anchor = where.anchor;
    s_pos = pos;
    s_w_cm = std::max(1.0f, where.width_cm);
    s_h_cm = s_w_cm * (float)h / (float)w;
    s_end_ms = now + fin + hld + fot;
    s_showing = true;
    xrlayer_clear_quad_orientation(XRLAYER_SLOT_TEXT);   // no orientation = it faces you
    xrtext_tick();                                         // posed this tick, not next
    return true;
}

void xrtext_tick() {
    if (!s_showing) return;
    if (!g_cfg.xr_text || GetTickCount64() >= s_end_ms) {
        xrlayer_retire_quad(XRLAYER_SLOT_TEXT);
        s_showing = false;
        return;
    }
    // Re-published every tick: the layer drops a quad whose pose has gone stale, which is how it tells
    // a live notice from a forgotten one. A View panel is ROOM-relative (its offset is in the view
    // base's axes); a World panel is neither.
    const bool view = (s_anchor == XrTextAnchor::View);
    xrlayer_set_quad_head_relative(XRLAYER_SLOT_TEXT, false);
    xrlayer_set_quad_view_relative(XRLAYER_SLOT_TEXT, view);
    xrlayer_notice_quad(XRLAYER_SLOT_TEXT, s_pos, s_w_cm, 0.0f, /*priority=*/1, s_h_cm);
}

} // namespace halo
