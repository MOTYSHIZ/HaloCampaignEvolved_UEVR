// <Windows.h>'s min/max macros would eat every std::min/std::max below. Before ANY include, since a
// project header may pull <Windows.h> in first.
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "XrText.hpp"

#include "Config.hpp"
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

    // WHERE: placed once, now. In front of where you look -- level, so looking down does not put it on
    // the floor -- then held as an offset from the camera, so it rides with a moving vehicle but does
    // not chase your head. Or at a world point, for a notice about a place or a thing.
    Vec3 pos{};
    if (where.anchor == XrTextAnchor::View) {
        Vec3 fwd{}, up{}, mono{};
        if (!xrlayer_view_basis(&fwd, &up) || !xrlayer_mono_view_pos(&mono)) return false;
        const auto& ps = host::g_plugin_state;
        Vec3 head = mono;
        if (ps.have_eye_pos->load(std::memory_order_relaxed))
            head = Vec3{ps.eye_pos_x->load(std::memory_order_relaxed), ps.eye_pos_y->load(std::memory_order_relaxed),
                        ps.eye_pos_z->load(std::memory_order_relaxed)};
        float fx = fwd.x, fy = fwd.y;
        const float fl = std::sqrt(fx * fx + fy * fy);
        if (fl < 1e-3f) { fx = 1.0f; fy = 0.0f; } else { fx /= fl; fy /= fl; }
        const float rx = -fy, ry = fx;   // UE: the right of a level forward
        pos = Vec3{head.x + fx * where.dist_cm + rx * where.right_cm - mono.x,
                   head.y + fy * where.dist_cm + ry * where.right_cm - mono.y,
                   head.z + where.up_cm - mono.z};
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
    // a live notice from a forgotten one.
    xrlayer_set_quad_head_relative(XRLAYER_SLOT_TEXT, s_anchor == XrTextAnchor::View);
    xrlayer_notice_quad(XRLAYER_SLOT_TEXT, s_pos, s_w_cm, 0.0f, /*priority=*/1, s_h_cm);
}

} // namespace halo
