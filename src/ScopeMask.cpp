// Feathered oval alpha for the scope pane's atlas cell. Doctrine, and why this is not fill_upload,
// are in ScopeMask.hpp -- read that first.

#include "ScopeMask.hpp"
#include "Config.hpp"

#include <d3d12.h>
#include <windows.h>
#include <cstdio>

#include "uevr/API.hpp"

namespace halo {
namespace {

// ---- state, all latching ------------------------------------------------------------------------
//
// Every failure below sets s_dead and never retries. A cosmetic mask that reattempts a failing
// device call every frame is worse than one that is simply off: it turns a logged one-time problem
// into a per-frame stall nobody can see the cause of.
bool                   s_dead     = false;
bool                   s_ready    = false;
bool                   s_applied  = false;   // did the LAST apply actually write alpha
ID3D12RootSignature*   s_rootsig  = nullptr;
ID3D12PipelineState*   s_pso      = nullptr;
ID3D12DescriptorHeap*  s_heap     = nullptr;
ID3D12Resource*        s_heap_for = nullptr;  // which atlas the UAV in s_heap describes
bool                   s_atlas_uav = false;   // was the LIVE atlas created UAV-capable
bool                   s_said_late = false;

void logf_(const char* fmt, ...) {
    char buf[640];
    va_list ap; va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    uevr::API::get()->log_info("[Halo-CampE-UEVR] scopemask: %s", buf);
}

void die(const char* why) {
    if (!s_dead) { s_dead = true; logf_("DISABLED -- %s. The pane keeps drawing without the mask.", why); }
}

// THE OVAL IS ANALYTIC. No mask texture, no upload, no asset: the cell's own pixel coordinate is
// mapped to -1..1, divided by per-axis radii, and smoothstepped. rx > ry is WIDER THAN TALL, which
// is the shape a scope wants and the thing that would otherwise need an authored image per size.
//
// Alpha is REPLACED, not multiplied. The capture's own alpha is not meaningful for an opaque scene
// (it reads ~0, which is exactly what made the pane invisible when it was blended by it), so there
// is nothing in it worth preserving -- and multiplying by ~0 would mask everything away again.
const char* kCS =
    "RWTexture2D<float4> Atlas : register(u0);\n"
    "cbuffer C : register(b0) { int4 rect; float4 shape; };\n"
    "[numthreads(8,8,1)]\n"
    "void main(uint3 id : SV_DispatchThreadID) {\n"
    "    if (id.x >= (uint)rect.z || id.y >= (uint)rect.z) return;\n"
    "    float2 uv = ((float2(id.xy) + 0.5f) / (float)rect.z) * 2.0f - 1.0f;\n"
    "    float2 q  = float2(uv.x / max(shape.x, 1e-3f), uv.y / max(shape.y, 1e-3f));\n"
    "    float  d  = length(q);\n"
    "    float  a  = 1.0f - smoothstep(1.0f - max(shape.z, 1e-3f), 1.0f, d);\n"
    "    uint2  c  = uint2((uint)rect.x + id.x, (uint)rect.y + id.y);\n"
    "    float4 px = Atlas[c];\n"
    "    px.a = a;\n"
    "    Atlas[c] = px;\n"
    "}\n";

// D3DCompile BY GetProcAddress, deliberately. Both build scripts link only user32.lib, and a
// cosmetic mask must not be the reason either of them grows a dependency -- the same call ScopeBlit
// makes, for the same reason. d3dcompiler_47.dll ships with Windows and is already resident in any
// D3D12 process, so this is a lookup rather than a load in practice.
typedef HRESULT (WINAPI *PFN_D3DCompile)(LPCVOID, SIZE_T, LPCSTR, const void*, void*, LPCSTR,
                                          LPCSTR, UINT, UINT, ID3DBlob**, ID3DBlob**);
typedef HRESULT (WINAPI *PFN_SerializeRS)(const D3D12_ROOT_SIGNATURE_DESC*,
                                          D3D_ROOT_SIGNATURE_VERSION, ID3DBlob**, ID3DBlob**);

bool build(ID3D12Device* device) {
    if (s_ready) return true;
    if (s_dead || device == nullptr) return false;

    HMODULE dc = ::GetModuleHandleW(L"d3dcompiler_47.dll");
    if (dc == nullptr) dc = ::LoadLibraryW(L"d3dcompiler_47.dll");
    auto compile = (dc != nullptr) ? (PFN_D3DCompile)::GetProcAddress(dc, "D3DCompile") : nullptr;
    HMODULE d12 = ::GetModuleHandleW(L"d3d12.dll");
    auto serialize = (d12 != nullptr)
        ? (PFN_SerializeRS)::GetProcAddress(d12, "D3D12SerializeRootSignature") : nullptr;
    if (compile == nullptr || serialize == nullptr) {
        die("D3DCompile or D3D12SerializeRootSignature could not be resolved");
        return false;
    }

    ID3DBlob* cs = nullptr; ID3DBlob* err = nullptr;
    if (FAILED(compile(kCS, std::strlen(kCS), "scopemask", nullptr, nullptr, "main", "cs_5_0",
                       0, 0, &cs, &err)) || cs == nullptr) {
        char msg[256] = {0};
        if (err != nullptr && err->GetBufferPointer() != nullptr) {
            std::snprintf(msg, sizeof(msg), "%.200s", (const char*)err->GetBufferPointer());
        }
        if (err != nullptr) err->Release();
        die(msg[0] != '\0' ? msg : "the mask compute shader would not compile");
        return false;
    }
    if (err != nullptr) err->Release();

    // One UAV in a table, plus 8 root constants (int4 rect + float4 shape).
    D3D12_DESCRIPTOR_RANGE range{};
    range.RangeType                         = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;
    range.NumDescriptors                    = 1;
    range.BaseShaderRegister                = 0;
    range.OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

    D3D12_ROOT_PARAMETER params[2]{};
    params[0].ParameterType                       = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    params[0].DescriptorTable.NumDescriptorRanges = 1;
    params[0].DescriptorTable.pDescriptorRanges   = &range;
    params[0].ShaderVisibility                    = D3D12_SHADER_VISIBILITY_ALL;
    params[1].ParameterType            = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
    params[1].Constants.Num32BitValues = 8;
    params[1].Constants.ShaderRegister = 0;
    params[1].ShaderVisibility         = D3D12_SHADER_VISIBILITY_ALL;

    D3D12_ROOT_SIGNATURE_DESC rsd{};
    rsd.NumParameters = 2;
    rsd.pParameters   = params;

    ID3DBlob* rsb = nullptr; ID3DBlob* rserr = nullptr;
    if (FAILED(serialize(&rsd, D3D_ROOT_SIGNATURE_VERSION_1, &rsb, &rserr)) || rsb == nullptr) {
        if (rserr != nullptr) rserr->Release();
        cs->Release();
        die("the root signature would not serialise");
        return false;
    }
    if (rserr != nullptr) rserr->Release();

    if (FAILED(device->CreateRootSignature(0, rsb->GetBufferPointer(), rsb->GetBufferSize(),
                                           IID_PPV_ARGS(&s_rootsig)))) {
        rsb->Release(); cs->Release();
        die("CreateRootSignature failed");
        return false;
    }
    rsb->Release();

    D3D12_COMPUTE_PIPELINE_STATE_DESC pd{};
    pd.pRootSignature   = s_rootsig;
    pd.CS.pShaderBytecode = cs->GetBufferPointer();
    pd.CS.BytecodeLength  = cs->GetBufferSize();
    const HRESULT hr = device->CreateComputePipelineState(&pd, IID_PPV_ARGS(&s_pso));
    cs->Release();
    if (FAILED(hr)) { die("CreateComputePipelineState failed"); return false; }

    D3D12_DESCRIPTOR_HEAP_DESC hd{};
    hd.Type           = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    hd.NumDescriptors = 1;
    hd.Flags          = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    if (FAILED(device->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&s_heap)))) {
        die("CreateDescriptorHeap failed");
        return false;
    }

    s_ready = true;
    logf_("ready -- analytic feathered oval, compute, no asset and no upload");
    return true;
}

}   // namespace

bool scopemask_wanted() { return g_cfg.scope_mask != 0; }

void scopemask_note_atlas(bool created_with_uav) {
    s_atlas_uav = created_with_uav;
    s_heap_for  = nullptr;    // any previous UAV described an atlas that no longer exists
    s_said_late = false;
}
bool scopemask_applied() { return s_applied; }

bool scopemask_apply(ID3D12Device* device, ID3D12GraphicsCommandList* list, ID3D12Resource* atlas,
                     int cell_x, int cell_y, int cell_dim) {
    s_applied = false;
    if (!scopemask_wanted() || s_dead) return false;
    if (device == nullptr || list == nullptr || atlas == nullptr || cell_dim <= 0) return false;
    // THE GUARD WHOSE ABSENCE CRASHED THE GAME. The UAV flag is decided when the atlas is created,
    // so enabling scopemask on a LIVE session leaves a perfectly valid-looking request against a
    // texture that cannot accept a UAV -- and CreateUnorderedAccessView on it is invalid usage, not
    // a failed call: the device is removed and the process goes with it. Refuse, say so, and let
    // the next bring-up create an atlas that can take it.
    if (!s_atlas_uav) {
        if (!s_said_late) {
            s_said_late = true;
            logf_("the live atlas was created WITHOUT unordered-access, so the mask cannot run on "
                  "it. scopemask is read when the atlas is BUILT, not when the key changes -- "
                  "toggle xrlayer 0 -> 1, or restart, and it applies at the next bring-up. Doing "
                  "this any other way removes the D3D12 device.");
        }
        return false;
    }
    if (!build(device)) return false;

    // The UAV describes ONE atlas. If the atlas is ever recreated the old descriptor points at a
    // dead resource, so it is rebuilt on identity change rather than assumed stable.
    if (s_heap_for != atlas) {
        // FORMAT FROM THE RESOURCE, never assumed. The atlas is B8G8R8A8_UNORM when the
        // swapchain is BGRA and R8G8B8A8_UNORM when it is not, and a UAV whose format disagrees
        // with its resource is invalid usage with the same consequence as no flag at all. Asking
        // the resource costs one call and cannot be wrong on a machine nobody tested.
        const D3D12_RESOURCE_DESC adesc = atlas->GetDesc();
        D3D12_UNORDERED_ACCESS_VIEW_DESC ud{};
        ud.Format        = adesc.Format;
        ud.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
        device->CreateUnorderedAccessView(atlas, nullptr, &ud,
                                          s_heap->GetCPUDescriptorHandleForHeapStart());
        s_heap_for = atlas;
    }

    // COPY_DEST -> UNORDERED_ACCESS and back. The caller owns the surrounding state and gets it
    // returned unchanged, which is what lets this be a single call dropped into an existing
    // sequence rather than a restructuring of it.
    D3D12_RESOURCE_BARRIER to_uav{};
    to_uav.Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    to_uav.Transition.pResource   = atlas;
    to_uav.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    to_uav.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
    to_uav.Transition.StateAfter  = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    list->ResourceBarrier(1, &to_uav);

    ID3D12DescriptorHeap* heaps[] = {s_heap};
    list->SetDescriptorHeaps(1, heaps);
    list->SetComputeRootSignature(s_rootsig);
    list->SetPipelineState(s_pso);
    list->SetComputeRootDescriptorTable(0, s_heap->GetGPUDescriptorHandleForHeapStart());

    struct { int32_t rect[4]; float shape[4]; } c{};
    c.rect[0] = cell_x; c.rect[1] = cell_y; c.rect[2] = cell_dim; c.rect[3] = 0;
    // WIDER THAN TALL: the X radius is 1.0 (the cell's full half-width) and the Y radius is divided
    // by the aspect, so scopemaskaspect > 1 squashes the oval vertically without cropping the
    // horizontal extent. Clamped so a nonsense config cannot produce a degenerate or inverted oval.
    float aspect = g_cfg.scope_mask_aspect;
    if (!(aspect > 0.2f) || aspect > 5.0f) aspect = 1.35f;
    float feather = g_cfg.scope_mask_feather;
    if (!(feather > 0.0f) || feather > 1.0f) feather = 0.15f;
    c.shape[0] = 1.0f;
    c.shape[1] = 1.0f / aspect;
    c.shape[2] = feather;
    c.shape[3] = 0.0f;
    list->SetComputeRoot32BitConstants(1, 8, &c, 0);

    const UINT groups = (UINT)((cell_dim + 7) / 8);
    list->Dispatch(groups, groups, 1);

    D3D12_RESOURCE_BARRIER back = to_uav;
    back.Transition.StateBefore = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    back.Transition.StateAfter  = D3D12_RESOURCE_STATE_COPY_DEST;
    list->ResourceBarrier(1, &back);

    // Say it ONCE, with the numbers, so "the oval is the wrong shape" is answerable from the log
    // instead of from a screenshot.
    static bool said = false;
    if (!said) {
        said = true;
        logf_("applied to cell %d,%d %dx%d -- aspect %.2f (wider than tall), feather %.2f. "
              "Slot 9's blend flag follows scopemask_applied(), so alpha blending is only "
              "re-enabled on frames this actually ran.",
              cell_x, cell_y, cell_dim, cell_dim, aspect, feather);
    }
    s_applied = true;
    return true;
}

void scopemask_shutdown() {
    if (s_heap    != nullptr) { s_heap->Release();    s_heap    = nullptr; }
    if (s_pso     != nullptr) { s_pso->Release();     s_pso     = nullptr; }
    if (s_rootsig != nullptr) { s_rootsig->Release(); s_rootsig = nullptr; }
    s_heap_for = nullptr;
    s_ready    = false;
    s_applied  = false;
}

}   // namespace halo
