// The scope pane's cleared-frame guard. The race, the design and the safety rules are in
// ScopeGuard.hpp -- read that first.

#include "ScopeGuard.hpp"
#include "Config.hpp"
#include "DevTools.hpp"

#include <d3d12.h>
#include <windows.h>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "uevr/API.hpp"

namespace halo {
namespace {

constexpr uint32_t kMaxHold  = 3;     // consecutive cleared frames refused before one is let through
constexpr UINT     kBufBytes = 256;   // predicate (8 B) + held count (4 B) + refused total (4 B), padded
constexpr int      kSlots    = kScopeGuardSlots;   // XrLayer static_asserts GT_RING matches

// ---- state, all latching (the ScopeMask rule: a failing device call is never retried per frame) ----
bool                   s_dead     = false;
bool                   s_ready    = false;
ID3D12RootSignature*   s_rootsig  = nullptr;
ID3D12PipelineState*   s_pso      = nullptr;
ID3D12DescriptorHeap*  s_heap     = nullptr;   // kSlots x { SRV of the target, UAV of the buffer }
ID3D12Resource*        s_pred     = nullptr;   // the predicate buffer -- ours, never the engine's
// Tracked in recording order. A BUFFER DECAYS TO COMMON when its ExecuteCommandLists completes, so
// every guarded sequence ends by returning it to COMMON explicitly: the tracker then agrees with the
// real state at the start of the next list (the guard runs at most once per capture list).
D3D12_RESOURCE_STATES  s_pred_state = D3D12_RESOURCE_STATE_COMMON;
UINT                   s_inc      = 0;
#if HALO_VR_DEV
// DEV ONLY: how many cleared frames the guard has refused, read back every 64 guarded copies. This
// is the log line that turns "the flicker stopped" into "the flicker WAS this".
ID3D12Resource*        s_readback = nullptr;
const uint32_t*        s_rb_map   = nullptr;
uint32_t               s_rb_calls = 0;
uint32_t               s_rb_said  = 0;
uint32_t               s_rb_base  = 0;     // first reading: the buffer starts with undefined content
bool                   s_rb_have  = false;
#endif

void logf_(const char* fmt, ...) {
    char buf[640];
    va_list ap; va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    uevr::API::get()->log_info("[Halo-CampE-UEVR] scopeguard: %s", buf);
}

void die(const char* why) {
    if (!s_dead) {
        s_dead = true;
        logf_("DISABLED -- %s. The pane copy runs unguarded, exactly as before.", why);
    }
}

// Nine points on a 3x3 grid at the sixths of the target. ALL must be exactly black for the frame to
// count as the engine's clear: the clear colour is (0,0,0) and an 8-bit UNORM zero reads back as
// exactly 0.0, whereas a lit, auto-exposed frame is essentially never pure zero at nine spread-out
// points. `held` counts consecutive refusals so a genuinely black view is still let through.
const char* kCS =
    "Texture2D<float4> Src : register(t0);\n"
    "RWByteAddressBuffer Pred : register(u0);\n"
    "cbuffer C : register(b0) { uint dim; uint maxHold; uint pad0; uint pad1; };\n"
    "[numthreads(1,1,1)]\n"
    "void main() {\n"
    "    bool cleared = true;\n"
    "    [unroll] for (uint i = 0; i < 3; ++i) {\n"
    "        [unroll] for (uint j = 0; j < 3; ++j) {\n"
    "            uint2 p = uint2(((2 * i + 1) * dim) / 6, ((2 * j + 1) * dim) / 6);\n"
    "            float3 c = Src.Load(int3(p, 0)).rgb;\n"
    "            if (any(c != 0.0f)) cleared = false;\n"
    "        }\n"
    "    }\n"
    "    uint held = Pred.Load(8);\n"
    "    uint skip = 0;\n"
    "    if (cleared && held < maxHold) { skip = 1; held = held + 1; } else { held = 0; }\n"
    "    Pred.Store2(0, uint2(skip, 0));\n"
    "    Pred.Store(8, held);\n"
    "    Pred.Store(12, Pred.Load(12) + skip);\n"
    "}\n";

typedef HRESULT (WINAPI *PFN_D3DCompile)(LPCVOID, SIZE_T, LPCSTR, const void*, void*, LPCSTR,
                                          LPCSTR, UINT, UINT, ID3DBlob**, ID3DBlob**);
typedef HRESULT (WINAPI *PFN_SerializeRS)(const D3D12_ROOT_SIGNATURE_DESC*,
                                          D3D_ROOT_SIGNATURE_VERSION, ID3DBlob**, ID3DBlob**);

bool build(ID3D12Device* device) {
    if (s_ready) return true;
    if (s_dead || device == nullptr) return false;

    // D3DCompile by GetProcAddress, as ScopeMask and ScopeBlit do: no new link dependency.
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
    if (FAILED(compile(kCS, std::strlen(kCS), "scopeguard", nullptr, nullptr, "main", "cs_5_0",
                       0, 0, &cs, &err)) || cs == nullptr) {
        char msg[256] = {0};
        if (err != nullptr && err->GetBufferPointer() != nullptr) {
            std::snprintf(msg, sizeof(msg), "%.200s", (const char*)err->GetBufferPointer());
        }
        if (err != nullptr) err->Release();
        die(msg[0] != '\0' ? msg : "the probe compute shader would not compile");
        return false;
    }
    if (err != nullptr) err->Release();

    // One table: the target's SRV at t0 (offset 0) and the buffer's UAV at u0 (offset 1); plus four
    // root constants (dim, maxHold, pad, pad).
    D3D12_DESCRIPTOR_RANGE ranges[2]{};
    ranges[0].RangeType                         = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    ranges[0].NumDescriptors                    = 1;
    ranges[0].BaseShaderRegister                = 0;
    ranges[0].OffsetInDescriptorsFromTableStart = 0;
    ranges[1].RangeType                         = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;
    ranges[1].NumDescriptors                    = 1;
    ranges[1].BaseShaderRegister                = 0;
    ranges[1].OffsetInDescriptorsFromTableStart = 1;

    D3D12_ROOT_PARAMETER params[2]{};
    params[0].ParameterType                       = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    params[0].DescriptorTable.NumDescriptorRanges = 2;
    params[0].DescriptorTable.pDescriptorRanges   = ranges;
    params[0].ShaderVisibility                    = D3D12_SHADER_VISIBILITY_ALL;
    params[1].ParameterType            = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
    params[1].Constants.Num32BitValues = 4;
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
    pd.pRootSignature     = s_rootsig;
    pd.CS.pShaderBytecode = cs->GetBufferPointer();
    pd.CS.BytecodeLength  = cs->GetBufferSize();
    const HRESULT hr = device->CreateComputePipelineState(&pd, IID_PPV_ARGS(&s_pso));
    cs->Release();
    if (FAILED(hr)) { die("CreateComputePipelineState failed"); return false; }

    D3D12_DESCRIPTOR_HEAP_DESC hd{};
    hd.Type           = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    hd.NumDescriptors = 2 * kSlots;
    hd.Flags          = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    if (FAILED(device->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&s_heap)))) {
        die("CreateDescriptorHeap failed");
        return false;
    }
    s_inc = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

    // The predicate buffer. Buffers are effectively created in COMMON whatever is asked for, so the
    // state tracker starts there and the first use transitions it explicitly.
    D3D12_HEAP_PROPERTIES hp{};
    hp.Type = D3D12_HEAP_TYPE_DEFAULT;
    D3D12_RESOURCE_DESC bd{};
    bd.Dimension        = D3D12_RESOURCE_DIMENSION_BUFFER;
    bd.Width            = kBufBytes;
    bd.Height           = 1;
    bd.DepthOrArraySize = 1;
    bd.MipLevels        = 1;
    bd.Format           = DXGI_FORMAT_UNKNOWN;
    bd.SampleDesc.Count = 1;
    bd.Layout           = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    bd.Flags            = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
    if (FAILED(device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &bd,
                                               D3D12_RESOURCE_STATE_COMMON, nullptr,
                                               IID_PPV_ARGS(&s_pred)))) {
        die("the predicate buffer could not be created");
        return false;
    }
    s_pred_state = D3D12_RESOURCE_STATE_COMMON;

#if HALO_VR_DEV
    // A persistently mapped readback copy of the counters. A readback buffer lives in COPY_DEST.
    D3D12_HEAP_PROPERTIES rp{};
    rp.Type = D3D12_HEAP_TYPE_READBACK;
    D3D12_RESOURCE_DESC rd = bd;
    rd.Flags = D3D12_RESOURCE_FLAG_NONE;
    if (SUCCEEDED(device->CreateCommittedResource(&rp, D3D12_HEAP_FLAG_NONE, &rd,
                                                  D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                                  IID_PPV_ARGS(&s_readback)))) {
        void* p = nullptr;
        D3D12_RANGE r{0, 16};
        if (SUCCEEDED(s_readback->Map(0, &r, &p))) s_rb_map = static_cast<const uint32_t*>(p);
    }
#endif

    s_ready = true;
    logf_("ready -- the scope pane's copy now refuses a frame that is still the capture's black "
          "clear (nine-point probe, predicated copy, at most %u frames held)", kMaxHold);
    return true;
}

// The SRV format for the target. A TYPELESS resource (the engine's target is DXGI 90 here) is viewed
// as its UNORM member; a FULLY-TYPED one can only be viewed in its OWN format -- reinterpreting it
// (say UNORM_SRGB as UNORM) is invalid without the relaxed-casting feature, and invalid view usage
// can remove the device. An sRGB view decodes, but a zero still reads as exactly zero, so the probe
// works the same. Anything outside the two 8-bit families is refused rather than guessed.
DXGI_FORMAT view_format(DXGI_FORMAT f) {
    switch (f) {
        case DXGI_FORMAT_B8G8R8A8_TYPELESS:   return DXGI_FORMAT_B8G8R8A8_UNORM;
        case DXGI_FORMAT_R8G8B8A8_TYPELESS:   return DXGI_FORMAT_R8G8B8A8_UNORM;
        case DXGI_FORMAT_B8G8R8A8_UNORM:
        case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
        case DXGI_FORMAT_R8G8B8A8_UNORM:
        case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB: return f;
        default:                              return DXGI_FORMAT_UNKNOWN;
    }
}

void transition(ID3D12GraphicsCommandList* list, D3D12_RESOURCE_STATES to) {
    if (s_pred_state == to) return;
    D3D12_RESOURCE_BARRIER b{};
    b.Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    b.Transition.pResource   = s_pred;
    b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    b.Transition.StateBefore = s_pred_state;
    b.Transition.StateAfter  = to;
    list->ResourceBarrier(1, &b);
    s_pred_state = to;
}

}  // namespace

bool scopeguard_begin(ID3D12Device* device, ID3D12GraphicsCommandList* list, ID3D12Resource* src,
                      int dim, int ring_slot) {
    if (g_cfg.scope_clear_guard == 0 || s_dead) return false;
    if (device == nullptr || list == nullptr || src == nullptr || dim <= 0) return false;
    if (ring_slot < 0 || ring_slot >= kSlots) return false;

    const DXGI_FORMAT vf = view_format(src->GetDesc().Format);
    if (vf == DXGI_FORMAT_UNKNOWN) {
        static bool said = false;
        if (!said) {
            said = true;
            logf_("the pane's source is DXGI format %u, not an 8-bit RGBA/BGRA family -- not "
                  "guarding it (the copy runs as before)", (unsigned)src->GetDesc().Format);
        }
        return false;
    }
    if (!build(device)) return false;

    // THIS SLOT'S DESCRIPTOR PAIR, rewritten every call. Safe because xrlayer_capture_begin() only
    // opens a ring slot once that slot's previous list has completed on the GPU, so nothing in
    // flight can still be reading these two descriptors.
    D3D12_CPU_DESCRIPTOR_HANDLE cpu = s_heap->GetCPUDescriptorHandleForHeapStart();
    cpu.ptr += (SIZE_T)ring_slot * 2 * s_inc;
    D3D12_SHADER_RESOURCE_VIEW_DESC sv{};
    sv.Format                  = vf;
    sv.ViewDimension           = D3D12_SRV_DIMENSION_TEXTURE2D;
    sv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    sv.Texture2D.MipLevels     = 1;
    device->CreateShaderResourceView(src, &sv, cpu);

    D3D12_CPU_DESCRIPTOR_HANDLE cpu_u = cpu;
    cpu_u.ptr += s_inc;
    D3D12_UNORDERED_ACCESS_VIEW_DESC uv{};
    uv.Format              = DXGI_FORMAT_R32_TYPELESS;
    uv.ViewDimension       = D3D12_UAV_DIMENSION_BUFFER;
    uv.Buffer.FirstElement = 0;
    uv.Buffer.NumElements  = kBufBytes / 4;
    uv.Buffer.Flags        = D3D12_BUFFER_UAV_FLAG_RAW;
    device->CreateUnorderedAccessView(s_pred, nullptr, &uv, cpu_u);

    // The probe. The target is read in the ENGINE_SRC_COLOR state the caller's copy already assumes
    // (it includes NON_PIXEL_SHADER_RESOURCE), BEFORE the caller moves it to COPY_SOURCE.
    transition(list, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    ID3D12DescriptorHeap* heaps[] = {s_heap};
    list->SetDescriptorHeaps(1, heaps);
    list->SetComputeRootSignature(s_rootsig);
    list->SetPipelineState(s_pso);
    D3D12_GPU_DESCRIPTOR_HANDLE gpu = s_heap->GetGPUDescriptorHandleForHeapStart();
    gpu.ptr += (UINT64)ring_slot * 2 * s_inc;
    list->SetComputeRootDescriptorTable(0, gpu);
    const uint32_t consts[4] = {(uint32_t)dim, kMaxHold, 0u, 0u};
    list->SetComputeRoot32BitConstants(1, 4, consts, 0);
    list->Dispatch(1, 1, 1);

    // PREDICATION ON: NOT_EQUAL_ZERO skips the predicated work that follows (the copy) whenever the
    // probe wrote a non-zero predicate. Barriers are not predicated, so the caller's transitions
    // around its copy still run and its state tracking stays true either way.
    transition(list, D3D12_RESOURCE_STATE_PREDICATION);
    list->SetPredication(s_pred, 0, D3D12_PREDICATION_OP_NOT_EQUAL_ZERO);

    static bool said_armed = false;
    if (!said_armed) {
        said_armed = true;
        logf_("armed on the pane copy (%dx%d, DXGI %u viewed as %u)", dim, dim,
              (unsigned)src->GetDesc().Format, (unsigned)vf);
    }
    return true;
}

void scopeguard_end(ID3D12GraphicsCommandList* list) {
    if (list == nullptr) return;
    list->SetPredication(nullptr, 0, D3D12_PREDICATION_OP_EQUAL_ZERO);

#if HALO_VR_DEV
    // Every 64 guarded copies: report the counters from the PREVIOUS readback (long complete by now:
    // it was recorded 64 captures ago), then queue a fresh one. Recorded after predication is off,
    // so the readback copy itself always runs.
    if (s_readback != nullptr && s_rb_map != nullptr && (++s_rb_calls % 64) == 0) {
        const uint32_t total = s_rb_map[3];
        if (!s_rb_have) {
            s_rb_have = true;               // the first read may carry the buffer's initial content
            s_rb_said = total;
            s_rb_base = total;
        } else if (total != s_rb_said) {
            logf_("refused %u more cleared frame(s) (%u since the first reading) -- each would have "
                  "been a one-frame black flash in the scope", total - s_rb_said, total - s_rb_base);
            s_rb_said = total;
        }
        transition(list, D3D12_RESOURCE_STATE_COPY_SOURCE);
        list->CopyBufferRegion(s_readback, 0, s_pred, 0, 16);
    }
#endif

    // Back to COMMON, which is where decay leaves it once this list completes (see s_pred_state).
    transition(list, D3D12_RESOURCE_STATE_COMMON);
}

}  // namespace halo
