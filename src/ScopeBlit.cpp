#include "ScopeBlit.hpp"
#include "Config.hpp"
#include "DevTools.hpp"

#include "uevr/API.hpp"

#include <Windows.h>
#include <d3d12.h>
#include <atomic>
#include <cstring>

using namespace uevr;

namespace halo {
namespace {

// ---------------------------------------------------------------------------------------------
// NO NEW LINK DEPENDENCIES. Both build scripts link user32.lib and nothing else, and one of them
// is the public contributor build -- adding libs there is a change to a shipped file for a feature
// that is still unproven. D3DCompile and D3D12SerializeRootSignature are therefore resolved at
// runtime, which also means a machine missing either DLL degrades to "feature off" instead of
// "plugin fails to load".
using PFN_D3DCOMPILE = HRESULT(WINAPI*)(LPCVOID, SIZE_T, LPCSTR, const void*, void*, LPCSTR,
                                        LPCSTR, UINT, UINT, ID3DBlob**, ID3DBlob**);
using PFN_SERIALIZE_RS = HRESULT(WINAPI*)(const D3D12_ROOT_SIGNATURE_DESC*,
                                          D3D_ROOT_SIGNATURE_VERSION, ID3DBlob**, ID3DBlob**);

PFN_D3DCOMPILE   g_D3DCompile = nullptr;
PFN_SERIALIZE_RS g_SerializeRS = nullptr;

// ---------------------------------------------------------------------------------------------
// State. All of it is created once, on the render thread, the first time the callback runs with
// the feature enabled.
bool g_registered = false;   // callback installed (game thread)
bool g_init_done  = false;   // one-shot init attempted (render thread)
bool g_dead       = false;   // a failure was logged; never try again

ID3D12Device*              g_device   = nullptr;
ID3D12RootSignature*       g_root_sig = nullptr;
ID3D12PipelineState*       g_pso      = nullptr;
ID3D12DescriptorHeap*      g_srv_heap = nullptr;
ID3D12Resource*            g_srv_for  = nullptr;   // which resource the SRV currently describes

// Reported once so a null result is never confused with a callback that is not running.
bool g_logged_facts = false;

void die(const char* why) {
    g_dead = true;
    API::get()->log_info("[Halo-CampE-UEVR] SCOPEBLIT DISABLED: %s", why);
}

const char* kShaderSrc = R"(
cbuffer Crop : register(b0) { float4 uvRect; }   // xy = uv min, zw = uv size
struct VSOut { float4 pos : SV_POSITION; float2 uv : TEXCOORD0; };

VSOut VSMain(uint id : SV_VertexID) {
    float2 t = float2((id << 1) & 2, id & 2);     // (0,0) (2,0) (0,2) -- fullscreen triangle
    VSOut o;
    o.pos = float4(t * float2(2.0, -2.0) + float2(-1.0, 1.0), 0.0, 1.0);
    o.uv  = uvRect.xy + t * uvRect.zw;
    return o;
}

Texture2D    gTex : register(t0);
SamplerState gSmp : register(s0);

float4 PSMain(VSOut i) : SV_TARGET {
    return float4(gTex.Sample(gSmp, i.uv).rgb, 1.0);
}
)";

bool load_entrypoints() {
    if (g_D3DCompile != nullptr && g_SerializeRS != nullptr) return true;
    HMODULE dc = LoadLibraryA("d3dcompiler_47.dll");
    HMODULE d12 = GetModuleHandleA("d3d12.dll");
    if (d12 == nullptr) d12 = LoadLibraryA("d3d12.dll");
    if (dc == nullptr || d12 == nullptr) return false;
    g_D3DCompile  = (PFN_D3DCOMPILE)GetProcAddress(dc, "D3DCompile");
    g_SerializeRS = (PFN_SERIALIZE_RS)GetProcAddress(d12, "D3D12SerializeRootSignature");
    return g_D3DCompile != nullptr && g_SerializeRS != nullptr;
}

bool create_pipeline(ID3D12Device* dev) {
    // Root signature: 4 float root constants (the crop rect) + one SRV table + a static sampler.
    D3D12_DESCRIPTOR_RANGE range{};
    range.RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    range.NumDescriptors = 1;
    range.BaseShaderRegister = 0;
    range.OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

    D3D12_ROOT_PARAMETER params[2]{};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
    params[0].Constants.Num32BitValues = 4;
    params[0].Constants.ShaderRegister = 0;
    params[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_VERTEX;
    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    params[1].DescriptorTable.NumDescriptorRanges = 1;
    params[1].DescriptorTable.pDescriptorRanges = &range;
    params[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;

    D3D12_STATIC_SAMPLER_DESC samp{};
    samp.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    samp.AddressU = samp.AddressV = samp.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    samp.MaxLOD = D3D12_FLOAT32_MAX;
    samp.ShaderRegister = 0;
    samp.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;

    D3D12_ROOT_SIGNATURE_DESC rs{};
    rs.NumParameters = 2;
    rs.pParameters = params;
    rs.NumStaticSamplers = 1;
    rs.pStaticSamplers = &samp;
    rs.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

    ID3DBlob* rs_blob = nullptr;
    ID3DBlob* err = nullptr;
    if (FAILED(g_SerializeRS(&rs, D3D_ROOT_SIGNATURE_VERSION_1, &rs_blob, &err)) ||
        rs_blob == nullptr) {
        if (err != nullptr) err->Release();
        return false;
    }
    HRESULT hr = dev->CreateRootSignature(0, rs_blob->GetBufferPointer(),
                                          rs_blob->GetBufferSize(),
                                          __uuidof(ID3D12RootSignature), (void**)&g_root_sig);
    rs_blob->Release();
    if (err != nullptr) err->Release();
    if (FAILED(hr) || g_root_sig == nullptr) return false;

    ID3DBlob* vs = nullptr; ID3DBlob* ps = nullptr; ID3DBlob* cerr = nullptr;
    const SIZE_T len = strlen(kShaderSrc);
    if (FAILED(g_D3DCompile(kShaderSrc, len, "scopeblit", nullptr, nullptr,
                            "VSMain", "vs_5_0", 0, 0, &vs, &cerr)) || vs == nullptr) {
        if (cerr != nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] SCOPEBLIT vs compile: %s",
                                 (const char*)cerr->GetBufferPointer());
            cerr->Release();
        }
        return false;
    }
    if (cerr != nullptr) { cerr->Release(); cerr = nullptr; }
    if (FAILED(g_D3DCompile(kShaderSrc, len, "scopeblit", nullptr, nullptr,
                            "PSMain", "ps_5_0", 0, 0, &ps, &cerr)) || ps == nullptr) {
        if (cerr != nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] SCOPEBLIT ps compile: %s",
                                 (const char*)cerr->GetBufferPointer());
            cerr->Release();
        }
        vs->Release();
        return false;
    }
    if (cerr != nullptr) cerr->Release();

    D3D12_GRAPHICS_PIPELINE_STATE_DESC pso{};
    pso.pRootSignature = g_root_sig;
    pso.VS = { vs->GetBufferPointer(), vs->GetBufferSize() };
    pso.PS = { ps->GetBufferPointer(), ps->GetBufferSize() };
    pso.BlendState.RenderTarget[0].SrcBlend = D3D12_BLEND_ONE;
    pso.BlendState.RenderTarget[0].DestBlend = D3D12_BLEND_ZERO;
    pso.BlendState.RenderTarget[0].BlendOp = D3D12_BLEND_OP_ADD;
    pso.BlendState.RenderTarget[0].SrcBlendAlpha = D3D12_BLEND_ONE;
    pso.BlendState.RenderTarget[0].DestBlendAlpha = D3D12_BLEND_ZERO;
    pso.BlendState.RenderTarget[0].BlendOpAlpha = D3D12_BLEND_OP_ADD;
    pso.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
    pso.SampleMask = UINT_MAX;
    pso.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
    pso.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
    pso.RasterizerState.DepthClipEnable = TRUE;
    pso.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    pso.NumRenderTargets = 1;
    pso.RTVFormats[0] = DXGI_FORMAT_B8G8R8A8_UNORM;   // matches the measured frame format
    pso.SampleDesc.Count = 1;

    hr = dev->CreateGraphicsPipelineState(&pso, __uuidof(ID3D12PipelineState), (void**)&g_pso);
    vs->Release();
    ps->Release();
    if (FAILED(hr) || g_pso == nullptr) return false;

    D3D12_DESCRIPTOR_HEAP_DESC hd{};
    hd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    hd.NumDescriptors = 1;
    hd.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    if (FAILED(dev->CreateDescriptorHeap(&hd, __uuidof(ID3D12DescriptorHeap),
                                         (void**)&g_srv_heap)) || g_srv_heap == nullptr) {
        return false;
    }
    return true;
}

// The render callback. Runs on the render thread with a live command list.
// Set on the game thread, read on the render thread. See the header.
std::atomic<bool> g_cine_on{false};

// Is the cutscene lane asking to draw THIS frame? Kept as one expression so the entry gate and
// the draw both ask the identical question -- the same discipline the cutscene detector itself
// had to learn (two conditions that disagree in any overlapping state are an oscillator).
inline bool cutscene_wants_blit() {
    return g_cfg.cutscene_blit && g_cine_on.load(std::memory_order_relaxed);
}

void on_post_render_dx12(void* cmd_list_v, void* rt_resource_v, void* rtv_v) {
    const bool cine = cutscene_wants_blit();
    if (g_dead || (!g_cfg.scope_blit && !cine)) return;
    auto* cmd = (ID3D12GraphicsCommandList*)cmd_list_v;
    auto* dst = (ID3D12Resource*)rt_resource_v;
    auto* rtv = (D3D12_CPU_DESCRIPTOR_HANDLE*)rtv_v;
    if (cmd == nullptr || dst == nullptr || rtv == nullptr) return;

    auto* scene = API::StereoHook::get_scene_render_target();
    auto* src = (scene != nullptr) ? (ID3D12Resource*)scene->get_native_resource() : nullptr;
    if (src == nullptr) { die("scene render target unavailable on the render thread"); return; }

    const D3D12_RESOURCE_DESC dd = dst->GetDesc();
    const D3D12_RESOURCE_DESC sd = src->GetDesc();

    // FACTS FIRST, ONCE. Whether the destination is the same surface we are sampling decides
    // whether this approach is possible at all -- a resource cannot be an SRV and a render target
    // in the same draw -- and no amount of shader work fixes it if it is.
    if (!g_logged_facts) {
        g_logged_facts = true;
        API::get()->log_info("[Halo-CampE-UEVR] SCOPEBLIT facts: dst=%p %ux%u fmt=%d | "
                             "src=%p %ux%u fmt=%d | SAME RESOURCE=%s",
                             (void*)dst, (unsigned)dd.Width, (unsigned)dd.Height, (int)dd.Format,
                             (void*)src, (unsigned)sd.Width, (unsigned)sd.Height, (int)sd.Format,
                             (dst == src) ? "YES -- cannot sample and write the same surface"
                                          : "no");
    }
    if (dst == src) { die("destination and scene render target are the SAME resource"); return; }

    if (!g_init_done) {
        g_init_done = true;
        if (!load_entrypoints()) { die("D3DCompile / D3D12SerializeRootSignature unavailable"); return; }
        if (FAILED(dst->GetDevice(__uuidof(ID3D12Device), (void**)&g_device)) || g_device == nullptr) {
            die("could not get the D3D12 device from the frame resource"); return;
        }
        if (!create_pipeline(g_device)) { die("pipeline/root signature/heap creation failed"); return; }
        API::get()->log_info("[Halo-CampE-UEVR] SCOPEBLIT: pipeline ready (device=%p)", (void*)g_device);
    }
    if (g_root_sig == nullptr || g_pso == nullptr || g_srv_heap == nullptr) return;

    // (Re)describe the source. TYPELESS resources need a typed view; the measured format is
    // B8G8R8A8_TYPELESS, so it is viewed as UNORM.
    if (g_srv_for != src) {
        g_srv_for = src;
        D3D12_SHADER_RESOURCE_VIEW_DESC srv{};
        srv.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
        srv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
        srv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        srv.Texture2D.MipLevels = 1;
        g_device->CreateShaderResourceView(src, &srv, g_srv_heap->GetCPUDescriptorHandleForHeapStart());
    }

    // NO RESOURCE BARRIER ON THE SOURCE, DELIBERATELY, IN THIS FIRST CUT. We do not know the state
    // UEVR leaves the scene target in, and a transition declaring the wrong before-state is a way
    // to remove the device -- i.e. to hard-crash a headset session. Sampling a resource that is in
    // the wrong state reads undefined data instead, which is recoverable and diagnosable. If the
    // quad comes out garbage rather than absent, THAT is the thing to fix here.

    const float mag = (g_cfg.scope_blit_mag < 1.05f) ? 1.05f : g_cfg.scope_blit_mag;
    // Crop half-size in UV, per eye. The scope magnifies (a small centred crop); the cutscene lane
    // takes the eye WHOLE (half = 0.5 => the full half-width, full height), because it is
    // reproducing the frame rather than zooming into it.
    const float half = cine ? 0.5f : (0.5f / mag);
    const bool  side_by_side = (sd.Width >= sd.Height * 2u);
    const int   eyes = side_by_side ? 2 : 1;

    cmd->SetGraphicsRootSignature(g_root_sig);
    cmd->SetPipelineState(g_pso);
    ID3D12DescriptorHeap* heaps[] = { g_srv_heap };
    cmd->SetDescriptorHeaps(1, heaps);
    cmd->SetGraphicsRootDescriptorTable(1, g_srv_heap->GetGPUDescriptorHandleForHeapStart());
    cmd->OMSetRenderTargets(1, rtv, FALSE, nullptr);
    cmd->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

    for (int e = 0; e < eyes; ++e) {
        // Destination rect. Two modes share this loop:
        //   scope    -- a square of scopeblitsize (fraction of frame height) centred at
        //               (scopeblitx, scopeblity) within this eye's half.
        //   cutscene -- the WHOLE eye (or cutsceneblitfill of it, centred). Filling the eye is
        //               the point: the doubled native composite is not hidden behind this, it is
        //               overwritten by it, which is what makes the lane work without
        //               VR_2DScreenMode.
        const float eye_w = (float)dd.Width / (float)eyes;
        float w, h, cx, cy;
        if (cine) {
            const float fill = (g_cfg.cutscene_blit_fill <= 0.0f) ? 1.0f : g_cfg.cutscene_blit_fill;
            w  = eye_w * fill;
            h  = (float)dd.Height * fill;
            cx = eye_w * ((float)e + 0.5f);
            cy = (float)dd.Height * 0.5f;
        } else {
            const float side = (float)dd.Height * g_cfg.scope_blit_size;
            w = side; h = side;
            cx = eye_w * (float)e + eye_w * g_cfg.scope_blit_x;
            cy = (float)dd.Height * g_cfg.scope_blit_y;
        }

        D3D12_VIEWPORT vp{};
        vp.TopLeftX = cx - w * 0.5f;
        vp.TopLeftY = cy - h * 0.5f;
        vp.Width = w;
        vp.Height = h;
        vp.MaxDepth = 1.0f;
        D3D12_RECT sc{};
        sc.left = (LONG)vp.TopLeftX; sc.top = (LONG)vp.TopLeftY;
        sc.right = (LONG)(vp.TopLeftX + w); sc.bottom = (LONG)(vp.TopLeftY + h);
        cmd->RSSetViewports(1, &vp);
        cmd->RSSetScissorRects(1, &sc);

        // Source crop, in UV of the WHOLE source texture. With a side-by-side source each eye
        // occupies half the width, so the crop is centred inside that half.
        const float u_span = 1.0f / (float)eyes;
        const float u_mid  = u_span * ((float)e + 0.5f);
        const float uv[4] = { u_mid - half * u_span, 0.5f - half, 2.0f * half * u_span, 2.0f * half };
        cmd->SetGraphicsRoot32BitConstants(0, 4, uv, 0);
        cmd->DrawInstanced(3, 1, 0, 0);
    }
}

} // namespace

void cutscene_blit_set_active(bool on) {
    g_cine_on.store(on, std::memory_order_relaxed);
}

void scope_blit_tick() {
    // Register when EITHER lane is enabled. Gating registration on scope_blit alone would leave
    // cutsceneblit=1 silently inert on a build where the scope blit is off -- a setting that
    // appears to do nothing, which is the failure this project keeps re-learning.
    if (g_registered || g_dead || (!g_cfg.scope_blit && !g_cfg.cutscene_blit)) return;

    // SAY THAT WE ARE TRYING, ONCE. Without this line the two ways of failing are indistinguishable
    // from outside: "this function was never called" and "it was called and could not register"
    // both produce a log with no SCOPEBLIT in it at all. Measured 2026-09-08 -- cutsceneblit=1 was
    // set, the game hung, and the first question (did my code even run?) could not be answered from
    // the log, only from the ABSENCE of lines, which is a weak reading of a strong claim.
    //
    // ARMING is therefore the positive marker: see it and the call site is live and the gate open;
    // do not see it and nothing below this point ever executed. That distinction is the whole
    // reason the line exists -- see feedback_prove_the_tick_not_the_init.
    {
        static bool s_said_arming = false;
        if (!s_said_arming) {
            s_said_arming = true;
            API::get()->log_info("[Halo-CampE-UEVR] SCOPEBLIT: ARMING (scopeblit=%d cutsceneblit=%d) "
                                 "-- registering the render callback. A 'registered' line should "
                                 "follow immediately; if it does not, read the reason logged next.",
                                 (int)g_cfg.scope_blit, (int)g_cfg.cutscene_blit);
        }
    }

    // The renderer callbacks live on UEVR_PluginCallbacks (param->callbacks), NOT on
    // param->functions -- functions is the log/hook/version surface.
    auto* param = API::get()->param();
    if (param == nullptr || param->callbacks == nullptr) {
        // NOT die() -- this is legitimately transient. We are called every tick, and the plugin
        // param can be unavailable early; the next tick may well succeed. But it must not be
        // SILENT, which is what it was: an enabled feature that never registers and never explains
        // itself is indistinguishable from a key that does nothing.
        //
        // Rate-limited rather than once-only, so a PERMANENT null (the case that actually strands
        // the feature) keeps saying so instead of scrolling away after one line at start-up.
        static uint32_t s_quiet = 0;
        if ((s_quiet++ % 512u) == 0) {
            API::get()->log_info("[Halo-CampE-UEVR] SCOPEBLIT: cannot register yet -- %s is null. "
                                 "Retrying every tick; the feature is INERT until this clears.",
                                 (param == nullptr) ? "the plugin param" : "param->callbacks");
        }
        return;
    }
    if (param->callbacks->on_post_render_vr_framework_dx12 == nullptr) {
        die("UEVR build exposes no on_post_render_vr_framework_dx12 callback");
        return;
    }
    g_registered = true;
    param->callbacks->on_post_render_vr_framework_dx12(&on_post_render_dx12);
    API::get()->log_info("[Halo-CampE-UEVR] SCOPEBLIT: render callback registered "
                         "(mag=%.2f size=%.2f at %.2f,%.2f)", g_cfg.scope_blit_mag,
                         g_cfg.scope_blit_size, g_cfg.scope_blit_x, g_cfg.scope_blit_y);
}

} // namespace halo
