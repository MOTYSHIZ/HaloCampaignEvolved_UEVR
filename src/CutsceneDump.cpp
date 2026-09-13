// CutsceneDump -- see the header. Dev builds only.

#include "CutsceneDump.hpp"
#include "DevTools.hpp"

#if HALO_VR_DEV

#include "uevr/API.hpp"
#include "Config.hpp"

#include <Windows.h>
#include <d3d12.h>
#include <cstdint>
#include <cstdio>
#include <cstring>

using uevr::API;

namespace halo {
namespace {

bool g_registered = false;

// The render callback. Runs on the render thread with a live command list.
//
// Mechanics, and the two things deliberately accepted for a dev-only one-shot:
//   * The copy is recorded on UEVR's own command list, so it executes in UEVR's frame order.
//     There is no fence we can wait on from here, so the buffer is read TWO callbacks later, by
//     which point that list has long since executed and presented.
//   * The scene target is assumed to be in RENDER_TARGET here (UEVR's own eye copy restores it
//     to that state), and it is transitioned to COPY_SOURCE and back around the copy. If the
//     assumption is wrong the debug layer objects and the pixels are undefined -- recoverable
//     and visible, as opposed to skipping the barrier and hoping.
// Written as a 32bpp top-down BMP straight from the BGRA8 rows; no image library.
void on_post_render_dx12(void* cmd_list_v, void* rt_resource_v, void* /*rtv_v*/) {
    static int      s_state = 0;          // 0 idle, 1 copy recorded, 2 written (until re-armed)
    static int      s_seen  = 0;          // edge trigger on the cfg value
    static uint32_t s_wait  = 0;
    static ID3D12Resource* s_rb = nullptr;
    static D3D12_PLACED_SUBRESOURCE_FOOTPRINT s_fp{};
    static UINT64   s_rb_size = 0;

    const int want = g_cfg.cutscene_dump;
    if (want == 0) { s_seen = 0; if (s_state == 2) s_state = 0; }   // re-arm on 0

    if (s_state == 1) {
        if (++s_wait < 2) return;
        char path[MAX_PATH] = {0};
        if (g_data_dir[0] != 0) {
            sprintf_s(path, MAX_PATH, "%s\\eyedump-%lu.bmp", g_data_dir, (unsigned long)GetTickCount());
        }
        void* p = nullptr;
        const D3D12_RANGE rr{ 0, (SIZE_T)s_rb_size };
        FILE* fo = nullptr;
        if (path[0] != 0 && s_rb != nullptr && SUCCEEDED(s_rb->Map(0, &rr, &p)) && p != nullptr) {
            const uint32_t w = s_fp.Footprint.Width, h = s_fp.Footprint.Height;
            const uint32_t pitch = s_fp.Footprint.RowPitch;
            const uint32_t img = w * h * 4u;
            if (fopen_s(&fo, path, "wb") == 0 && fo != nullptr) {
                // BITMAPFILEHEADER (14) + BITMAPINFOHEADER (40), 32bpp, negative height = top-down.
                uint8_t fh[14] = { 'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0 };
                const uint32_t fsz = 54u + img; memcpy(fh + 2, &fsz, 4);
                uint8_t ih[40] = {0};
                const uint32_t hsz = 40u; memcpy(ih + 0, &hsz, 4);
                const int32_t  bw = (int32_t)w, bh = -(int32_t)h; memcpy(ih + 4, &bw, 4); memcpy(ih + 8, &bh, 4);
                const uint16_t planes = 1, bpp = 32; memcpy(ih + 12, &planes, 2); memcpy(ih + 14, &bpp, 2);
                memcpy(ih + 20, &img, 4);
                fwrite(fh, 1, 14, fo); fwrite(ih, 1, 40, fo);
                const uint8_t* rowp = reinterpret_cast<const uint8_t*>(p);
                for (uint32_t y = 0; y < h; ++y) fwrite(rowp + (size_t)y * pitch, 1, (size_t)w * 4u, fo);
                fclose(fo);
                API::get()->log_info("[Halo-CampE-UEVR] EYEDUMP: wrote %s (%ux%u BGRA, side-by-side: "
                                     "left eye = x 0..%u, right eye = x %u..%u). Set cutscenedump=0 to re-arm.",
                                     path, w, h, w / 2u - 1u, w / 2u, w - 1u);
            } else {
                API::get()->log_info("[Halo-CampE-UEVR] EYEDUMP: could not open %s for writing", path);
            }
            const D3D12_RANGE wr{ 0, 0 };
            s_rb->Unmap(0, &wr);
        } else {
            API::get()->log_info("[Halo-CampE-UEVR] EYEDUMP: Map failed or the data folder is unknown");
        }
        if (s_rb != nullptr) { s_rb->Release(); s_rb = nullptr; }
        s_state = 2;
        return;
    }

    if (s_state != 0 || want == 0 || s_seen != 0) return;   // idle, or armed and already fired
    s_seen = want;

    auto* cmd = (ID3D12GraphicsCommandList*)cmd_list_v;
    if (cmd == nullptr || rt_resource_v == nullptr) return;
    auto* scene = API::StereoHook::get_scene_render_target();
    auto* src = (scene != nullptr) ? (ID3D12Resource*)scene->get_native_resource() : nullptr;
    if (src == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] EYEDUMP: scene render target unavailable on the render thread");
        s_state = 2;
        return;
    }

    ID3D12Device* dev = nullptr;
    if (FAILED(src->GetDevice(__uuidof(ID3D12Device), (void**)&dev)) || dev == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] EYEDUMP: no device behind the scene target");
        s_state = 2;
        return;
    }
    const D3D12_RESOURCE_DESC sdesc = src->GetDesc();
    UINT rows = 0; UINT64 row_bytes = 0, total = 0;
    dev->GetCopyableFootprints(&sdesc, 0, 1, 0, &s_fp, &rows, &row_bytes, &total);
    D3D12_HEAP_PROPERTIES hp{}; hp.Type = D3D12_HEAP_TYPE_READBACK;
    D3D12_RESOURCE_DESC bd{};
    bd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER; bd.Width = total; bd.Height = 1;
    bd.DepthOrArraySize = 1; bd.MipLevels = 1; bd.Format = DXGI_FORMAT_UNKNOWN;
    bd.SampleDesc.Count = 1; bd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    if (s_rb != nullptr) { s_rb->Release(); s_rb = nullptr; }
    if (SUCCEEDED(dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &bd,
            D3D12_RESOURCE_STATE_COPY_DEST, nullptr, __uuidof(ID3D12Resource), (void**)&s_rb))
        && s_rb != nullptr) {
        s_rb_size = total;
        D3D12_RESOURCE_BARRIER b{};
        b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        b.Transition.pResource = src; b.Transition.Subresource = 0;
        b.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
        b.Transition.StateAfter  = D3D12_RESOURCE_STATE_COPY_SOURCE;
        cmd->ResourceBarrier(1, &b);
        D3D12_TEXTURE_COPY_LOCATION dl{}; dl.pResource = s_rb;
        dl.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT; dl.PlacedFootprint = s_fp;
        D3D12_TEXTURE_COPY_LOCATION sl{}; sl.pResource = src;
        sl.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX; sl.SubresourceIndex = 0;
        cmd->CopyTextureRegion(&dl, 0, 0, 0, &sl, nullptr);
        b.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_SOURCE;
        b.Transition.StateAfter  = D3D12_RESOURCE_STATE_RENDER_TARGET;
        cmd->ResourceBarrier(1, &b);
        s_state = 1; s_wait = 0;
        API::get()->log_info("[Halo-CampE-UEVR] EYEDUMP: copy recorded -- scene target "
                             "%ux%u fmt=%d rowpitch=%u; writing the BMP two frames from now",
                             (unsigned)s_fp.Footprint.Width, (unsigned)s_fp.Footprint.Height,
                             (int)sdesc.Format, (unsigned)s_fp.Footprint.RowPitch);
    } else {
        API::get()->log_info("[Halo-CampE-UEVR] EYEDUMP: readback buffer creation FAILED "
                             "(%llu bytes)", (unsigned long long)total);
        s_state = 2;
    }
    dev->Release();
}

}   // namespace

void cutscene_dump_register() {
    if (g_registered) return;
    auto* param = API::get()->param();
    if (param == nullptr || param->callbacks == nullptr
        || param->callbacks->on_post_render_vr_framework_dx12 == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] EYEDUMP: this UEVR build exposes no "
                             "on_post_render_vr_framework_dx12 callback -- cutscenedump unavailable");
        g_registered = true;   // nothing to retry; say it once
        return;
    }
    g_registered = true;
    param->callbacks->on_post_render_vr_framework_dx12(&on_post_render_dx12);
    API::get()->log_info("[Halo-CampE-UEVR] EYEDUMP: render callback registered at init "
                         "(dev build; inert until cutscenedump=1)");
}

}   // namespace halo

#else   // player builds: nothing registered, nothing compiled in

namespace halo {
void cutscene_dump_register() {}
}   // namespace halo

#endif
