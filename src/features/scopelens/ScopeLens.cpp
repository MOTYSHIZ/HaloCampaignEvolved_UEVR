// PHYSICAL SCOPE LENS (scopelens, Experimental): the scene-capture lens and its key family. The
// include list is the scope's own (Scope.cpp's), so every call resolves against the same
// declarations the scope code uses.

#include "ScopeLens.hpp"
#include "core/config/CfgRead.hpp"
#include "core/fixes/TickStage.hpp"

#include "Scope.hpp"
#include "ScopeOffset.hpp"
#include "Config.hpp"
#include "core/Services.hpp"
#include "DevTools.hpp"
#include "Rig.hpp"
#include "Reticule.hpp"
#include "ScopeBlit.hpp"
#include "ScopeLayer.hpp"
#include <d3d12.h>
// The PHYSICAL scope (second section at the bottom of this file, scopewpn=... entries) reads the
// held weapon, the holster gate and the stick-mode flag through these.
#include "Holster.hpp"           // holster_fire_suppressed
#include "MotionAimControl.hpp"  // read_control_rotation, g_stick_mode_active
#include "WeaponCalib.hpp"       // weapon_key

#include <cmath>
#include <cstring>
#include <chrono>
#include <string>

using namespace uevr;

namespace halo {

// Defined in Reticule.cpp with external linkage (shared engine helpers), declared at the top of
// Scope.cpp, which this code shared.
API::UObject* load_asset_by_path(const char* path);

// ============================================================================================
// PHYSICAL SCOPE -- the SECOND scope path, live only for weapons with a scopewpn=... entry
// (Config::scopes). Everything above is the floating compositor/XR-layer pane and stays the path
// of record; nothing below touches its state. This path builds its own SceneCaptureComponent2D
// (source 9, half tone curve, EV bias / tint), a round Cylinder lens in the weapon's scope housing,
// roll correction from the weapon root, and an etched reticle quad -- see the Scope.hpp tail.
// Statics that share a name with the pane path's carry a phys_ infix.
// ============================================================================================


API::UObject* find_or_load_material(const std::string& object_path);   // Reticule.cpp
API::UObject* import_texture_file(const char* path);                   // Reticule.cpp

namespace {

// What we built, all owned by the pawn's actor (add_component_by_class) and re-created when the
// owner changes. Raw pointers: validated through the rig tracker's owner each tick.
API::UObject* s_owner   = nullptr;
API::UObject* s_phys_rt      = nullptr;
API::UObject* s_phys_capture = nullptr;
API::UObject* s_lens    = nullptr;
API::UObject* s_mid     = nullptr;
API::UObject* s_ret     = nullptr;   // etched-reticle quad (scope_reticle.png beside the cfg)
API::UObject* s_ret_mid = nullptr;
API::UObject* s_attached_to = nullptr;   // weapon root the lens is currently attached to
std::string   s_key_logged;              // last key logged as "no scope entry"
bool          s_active = false;
bool          s_phys_failed = false;
bool          s_round_mesh = false;
int           s_cfg_idx = -1;

const ScopeCfg* find_cfg(const std::string& key) {
    for (int i = 0; i < g_cfg.scope_cfg_count; ++i) {
        if (key == g_cfg.scopes[i].key) return &g_cfg.scopes[i];
    }
    return nullptr;
}

void set_hidden(API::UObject* comp, bool hidden) {
    if (comp == nullptr) return;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    p[0] = hidden ? 1 : 0; p[1] = 1;   // propagate to children
    comp->call_function(L"SetHiddenInGame", p);
}

// POST-PROCESS OVERRIDES on the capture: the tonemapped capture paths run their OWN auto-exposure,
// which on a fresh capture settles near black (measured 2026-08-19: LDR and tone-curve sources both
// dark, raw scene colour blown out). Force manual exposure with an EV bias we control, so the
// picture keeps bloom/reflections/tonemapping and a stable, tunable brightness.
void apply_capture_pp() {
    if (s_phys_capture == nullptr) return;
    static API::UStruct* pps = nullptr;
    if (pps == nullptr) pps = API::get()->find_uobject<API::UStruct>(L"ScriptStruct /Script/Engine.PostProcessSettings");
    auto* base = s_phys_capture->get_property_data<uint8_t>(L"PostProcessSettings");
    if (pps == nullptr || base == nullptr) { static bool said = false; if (!said) { said = true; API::get()->log_info("[Halo-CampE-UEVR] SCOPE: PostProcessSettings not reachable (struct %p data %p)", (void*)pps, (void*)base); } return; }
    static bool s_logged = false;
    const bool write_ev  = (g_cfg.scope_pp_override != 0);
    const bool write_lum = (g_cfg.scope_lumen > 0);   // > 0, not != 0: the pane path's scopelumen uses -1 for "leave alone"
    if (!write_ev && !write_lum && g_cfg.scope_tone_curve < 0.0f && s_logged) return;   // log-only pass once, then nothing
    // Every setter takes its own write flag so the exposure block and the Lumen block are
    // independently switchable (scopepp / scopelumen); offsets/masks/before-values log once.
    auto set_bit = [&](bool write, const wchar_t* name, bool on) {
        auto* bp = reinterpret_cast<API::FBoolProperty*>(pps->find_property(name));
        if (bp == nullptr) { if (!s_logged) API::get()->log_info("[Halo-CampE-UEVR] SCOPE-PP: %ls NOT FOUND", name); return; }
        uint8_t* b = base + bp->get_offset() + bp->get_byte_offset();
        const uint8_t m = (uint8_t)bp->get_byte_mask();
        if (!s_logged) API::get()->log_info("[Halo-CampE-UEVR] SCOPE-PP: %ls off=%d byteoff=%u mask=0x%02X fieldsize=%u before=0x%02X", name, bp->get_offset(), bp->get_byte_offset(), m, bp->get_field_size(), *b);
        if (write) { if (on) *b |= m; else *b &= (uint8_t)~m; }
    };
    auto set_u8 = [&](bool write, const wchar_t* name, uint8_t v) { if (auto* pr = pps->find_property(name)) { if (!s_logged) API::get()->log_info("[Halo-CampE-UEVR] SCOPE-PP: %ls off=%d (u8) before=%u", name, pr->get_offset(), base[pr->get_offset()]); if (write) base[pr->get_offset()] = v; } else if (!s_logged) API::get()->log_info("[Halo-CampE-UEVR] SCOPE-PP: %ls NOT FOUND", name); };
    auto set_f  = [&](bool write, const wchar_t* name, float v)   { if (auto* pr = pps->find_property(name)) { if (!s_logged) API::get()->log_info("[Halo-CampE-UEVR] SCOPE-PP: %ls off=%d (f) before=%.3f", name, pr->get_offset(), *reinterpret_cast<float*>(base + pr->get_offset())); if (write) *reinterpret_cast<float*>(base + pr->get_offset()) = v; } else if (!s_logged) API::get()->log_info("[Halo-CampE-UEVR] SCOPE-PP: %ls NOT FOUND", name); };
    // ---- exposure (scopepp)
    set_bit(write_ev, L"bOverride_AutoExposureMethod", true);  set_u8(write_ev, L"AutoExposureMethod", 2);          // AEM_Manual
    set_bit(write_ev, L"bOverride_AutoExposureBias", true);    set_f(write_ev, L"AutoExposureBias", g_cfg.scope_ev);
    set_bit(write_ev, L"bOverride_AutoExposureApplyPhysicalCameraExposure", true);
    set_bit(write_ev, L"AutoExposureApplyPhysicalCameraExposure", false);   // EV bias alone, not ISO/aperture defaults
    // ---- Lumen (scopelumen): the GI / reflection METHOD is a per-view post-process setting. The
    // capture's view resolves it from project defaults + volumes like any camera, but the capture
    // measured 2026-08-19 renders no bounce light, so force it to Lumen (1) here.
    // EDynamicGlobalIlluminationMethod: 0 None, 1 Lumen, 2 ScreenSpace, 3 RayTraced, 4 Plugin.
    // EReflectionMethod:                0 None, 1 Lumen, 2 ScreenSpace, 3 RayTraced.
    set_bit(write_lum, L"bOverride_DynamicGlobalIlluminationMethod", true); set_u8(write_lum, L"DynamicGlobalIlluminationMethod", 1);
    set_bit(write_lum, L"bOverride_ReflectionMethod", true);                set_u8(write_lum, L"ReflectionMethod", 1);
    // ---- tone curve (scopetonecurve, live; <0 = leave alone). The decisive knob for the lens:
    // sources 2/9 composite translucency (the shield wall) but apply the film tone curve, and the
    // main view then tone-curves the lens surface AGAIN -> flat grey (measured 2026-08-19). With
    // ToneCurveAmount=0 the capture still runs the FULL post chain (translucency, bloom, manual
    // exposure) but outputs LINEAR colour -- the one thing the main view's tonemapper wants.
    const bool write_tc = (g_cfg.scope_tone_curve >= 0.0f);
    set_bit(write_tc, L"bOverride_ToneCurveAmount", true);
    set_f(write_tc, L"ToneCurveAmount", g_cfg.scope_tone_curve < 0.0f ? 0.0f : g_cfg.scope_tone_curve);
    if (write_ev || write_lum || write_tc) { if (auto* w = s_phys_capture->get_property_data<float>(L"PostProcessBlendWeight")) *w = 1.0f; }
    s_logged = true;
}

// SHOW FLAGS on the capture (scopeshowflags). USceneCaptureComponent copies its ShowFlagSettings
// array into the real FEngineShowFlags in UpdateShowFlags(), which runs on registration -- so the
// array has to be filled BEFORE the component is finished (deferred add + FinishAddComponent).
// Memory comes from the engine's FMalloc: the component frees it when the pawn dies.
bool write_show_flags(API::UObject* cap) {
    auto* cls = cap->get_class();
    auto* arr = cls ? reinterpret_cast<API::FArrayProperty*>(cls->find_property(L"ShowFlagSettings")) : nullptr;
    if (arr == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SCOPE-SF: ShowFlagSettings NOT FOUND"); return false; }
    auto* inner = reinterpret_cast<API::FStructProperty*>(arr->get_inner());
    auto* st = inner ? reinterpret_cast<API::UStruct*>(inner->get_struct()) : nullptr;
    if (st == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SCOPE-SF: element struct NOT FOUND"); return false; }
    auto* pname = st->find_property(L"ShowFlagName");
    auto* pen   = reinterpret_cast<API::FBoolProperty*>(st->find_property(L"Enabled"));
    const int esz = st->get_properties_size();
    if (pname == nullptr || pen == nullptr || esz <= 0) { API::get()->log_info("[Halo-CampE-UEVR] SCOPE-SF: element layout NOT FOUND (name %p enabled %p size %d)", (void*)pname, (void*)pen, esz); return false; }
    struct TArrayHdr { void* data; int32_t num; int32_t max; };
    auto* hdr = reinterpret_cast<TArrayHdr*>(reinterpret_cast<uint8_t*>(cap) + arr->get_offset());
    API::get()->log_info("[Halo-CampE-UEVR] SCOPE-SF: array off=%d elem=%d name@%d enabled@%d/0x%02X existing num=%d", arr->get_offset(), esz, pname->get_offset(), pen->get_offset(), (unsigned)pen->get_byte_mask(), hdr->num);
    if (hdr->data != nullptr || hdr->num != 0) { API::get()->log_info("[Halo-CampE-UEVR] SCOPE-SF: array not empty, leaving it"); return false; }
    // Flag names come from cfg (scopesfflags=comma,separated,list) so experiments are a cfg edit
    // plus a level reload, not a deploy. Unknown names are ignored by the engine's by-name lookup,
    // so listing generously is safe.
    std::wstring names[32];
    int n = 0;
    for (const char* p = g_cfg.scope_sf_names; *p != 0 && n < 32; ) {
        while (*p == ',' || *p == ' ') ++p;
        const char* e = p;
        while (*e != 0 && *e != ',' && *e != ' ') ++e;
        if (e > p) names[n++].assign(p, e);
        p = e;
    }
    if (n == 0) { API::get()->log_info("[Halo-CampE-UEVR] SCOPE-SF: scopesfflags empty, writing none"); return false; }
    auto* fm = API::FMalloc::get();
    if (fm == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SCOPE-SF: no FMalloc"); return false; }
    auto* data = reinterpret_cast<uint8_t*>(fm->malloc((uint32_t)(esz * n), 16));
    if (data == nullptr) return false;
    memset(data, 0, (size_t)esz * n);
    for (int i = 0; i < n; ++i) {
        uint8_t* e = data + (size_t)i * esz;
        const size_t len = names[i].size();
        auto* buf = reinterpret_cast<wchar_t*>(fm->malloc((uint32_t)((len + 1) * sizeof(wchar_t)), 2));
        if (buf == nullptr) return false;
        memcpy(buf, names[i].c_str(), (len + 1) * sizeof(wchar_t));
        auto* fs = reinterpret_cast<TArrayHdr*>(e + pname->get_offset());
        fs->data = buf; fs->num = (int32_t)(len + 1); fs->max = (int32_t)(len + 1);
        e[pen->get_offset() + pen->get_byte_offset()] |= (uint8_t)pen->get_byte_mask();
    }
    hdr->data = data; hdr->num = n; hdr->max = n;
    API::get()->log_info("[Halo-CampE-UEVR] SCOPE-SF: wrote %d show flags", n);
    return true;
}

// Finish a deferred add: AActor::FinishAddComponent(Component, bManualAttachment, RelativeTransform).
// Param offsets are read from the UFunction, not assumed.
bool finish_add_component(API::UObject* owner, API::UObject* comp) {
    auto* cls = owner->get_class();
    auto* fn = cls ? cls->find_function(L"FinishAddComponent") : nullptr;
    if (fn == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SCOPE-SF: FinishAddComponent NOT FOUND"); return false; }
    auto* pc = fn->find_property(L"Component");
    auto* pm = reinterpret_cast<API::FBoolProperty*>(fn->find_property(L"bManualAttachment"));
    auto* pt = fn->find_property(L"RelativeTransform");
    if (pc == nullptr || pm == nullptr || pt == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SCOPE-SF: FinishAddComponent params NOT FOUND"); return false; }
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    *reinterpret_cast<API::UObject**>(p + pc->get_offset()) = comp;
    p[pm->get_offset() + pm->get_byte_offset()] &= (uint8_t)~pm->get_byte_mask();   // auto-attach, as the non-deferred path does
    auto* t = reinterpret_cast<double*>(p + pt->get_offset());   // FTransform: quat(4) translation(3+pad) scale(3)
    t[3] = 1.0; t[8] = 1.0; t[9] = 1.0; t[10] = 1.0;
    API::get()->log_info("[Halo-CampE-UEVR] SCOPE-SF: FinishAddComponent comp@%d manual@%d xf@%d", pc->get_offset(), pm->get_offset(), pt->get_offset());
    owner->call_function(L"FinishAddComponent", p);
    return true;
}

// RT PROBE (scopeprobe): read the centre pixel of the render target back from the GPU once a second
// and log it, so brightness is a NUMBER in the log rather than an impression in the headset.
void probe_rt() {
    if (!g_cfg.scope_probe || s_phys_rt == nullptr) return;
    static auto last = std::chrono::steady_clock::now() - std::chrono::seconds(5);
    const auto now = std::chrono::steady_clock::now();
    if (now - last < std::chrono::seconds(1)) return;
    last = now;
    auto* pc = API::get()->get_player_controller(0);
    auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.KismetRenderingLibrary");
    auto* krl = cls ? cls->get_class_default_object() : nullptr;
    auto* fn = cls ? cls->find_function(L"ReadRenderTargetRawPixel") : nullptr;
    if (pc == nullptr || krl == nullptr || fn == nullptr) { static bool said = false; if (!said) { said = true; API::get()->log_info("[Halo-CampE-UEVR] SCOPE-RT: ReadRenderTargetRawPixel unavailable"); } return; }
    auto* pw = fn->find_property(L"WorldContextObject");
    auto* pr = fn->find_property(L"TextureRenderTarget");
    auto* px = fn->find_property(L"X");
    auto* py = fn->find_property(L"Y");
    auto* pn = reinterpret_cast<API::FBoolProperty*>(fn->find_property(L"bNormalize"));
    auto* prv = fn->find_property(L"ReturnValue");
    if (pw == nullptr || pr == nullptr || px == nullptr || py == nullptr || prv == nullptr) { static bool said = false; if (!said) { said = true; API::get()->log_info("[Halo-CampE-UEVR] SCOPE-RT: param layout NOT FOUND"); } return; }
    const int c = g_cfg.scope_res / 2;
    const int pts[3][2] = { {c, c}, {c / 2, c / 2}, {c + c / 2, c + c / 2} };
    float out[3][4] = {};
    for (int i = 0; i < 3; ++i) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<API::UObject**>(p + pw->get_offset()) = pc;
        *reinterpret_cast<API::UObject**>(p + pr->get_offset()) = s_phys_rt;
        *reinterpret_cast<int32_t*>(p + px->get_offset()) = pts[i][0];
        *reinterpret_cast<int32_t*>(p + py->get_offset()) = pts[i][1];
        if (pn != nullptr) p[pn->get_offset() + pn->get_byte_offset()] &= (uint8_t)~pn->get_byte_mask();   // raw, not normalised
        krl->call_function(L"ReadRenderTargetRawPixel", p);
        memcpy(out[i], p + prv->get_offset(), sizeof(float) * 4);
    }
    API::get()->log_info("[Halo-CampE-UEVR] SCOPE-RT: centre %.4f %.4f %.4f | q1 %.4f %.4f %.4f | q3 %.4f %.4f %.4f (src %d fmt %d tint %.2f pp %d lumen %d)",
        out[0][0], out[0][1], out[0][2], out[1][0], out[1][1], out[1][2], out[2][0], out[2][1], out[2][2],
        g_cfg.scope_capture_source, g_cfg.scope_rt_format, g_cfg.scope_tint, g_cfg.scope_pp_override, g_cfg.scope_lumen);
}

bool s_cap_on = true;   // the gate/AB decision; the rate cap only fires while this is true

void set_capture_enabled(bool on) {
    s_cap_on = on;
    if (s_phys_capture == nullptr) return;
    // With a rate cap the engine flag stays FALSE and captures are manual CaptureScene calls.
    s_phys_capture->set_bool_property(L"bCaptureEveryFrame", on && g_cfg.scope_hz <= 0.0f);
}

API::UObject* make_rt(int size) {
    auto* pc = API::get()->get_player_controller(0);
    auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.KismetRenderingLibrary");
    auto* krl = cls ? cls->get_class_default_object() : nullptr;
    if (pc == nullptr || krl == nullptr) return nullptr;
    // CreateRenderTarget2D(WorldContextObject, Width, Height, Format, ClearColor, bAutoGenerateMipMaps, bSupportUAVs)
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    *reinterpret_cast<void**>(p) = pc;
    *reinterpret_cast<int32_t*>(p + 8)  = size;
    *reinterpret_cast<int32_t*>(p + 12) = size;
    p[16] = (uint8_t)g_cfg.scope_rt_format;   // ETextureRenderTargetFormat: 2 = RGBA8, 6 = RGBA16f (no clamp -> manual exposure via scopetint)
    auto* cc = reinterpret_cast<float*>(p + 20); cc[0] = 0; cc[1] = 0; cc[2] = 0; cc[3] = 1;
    krl->call_function(L"CreateRenderTarget2D", p);
    return *reinterpret_cast<API::UObject**>(p + 40);
}

bool build(API::UObject* owner) {
    s_owner = owner;
    s_phys_rt = make_rt(g_cfg.scope_res);
    if (s_phys_rt == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SCOPE: render target FAILED"); return false; }

    // ---- the capture camera
    auto* cap_cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.SceneCaptureComponent2D");
    if (cap_cls == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SCOPE: SceneCaptureComponent2D class NOT FOUND"); return false; }
    // scopeshowflags: deferred add so ShowFlagSettings is in place when the component registers.
    const bool deferred = (g_cfg.scope_showflags != 0);
    s_phys_capture = API::get()->add_component_by_class(owner, cap_cls, deferred);
    if (s_phys_capture == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SCOPE: capture add_component FAILED (deferred %d)", deferred ? 1 : 0); return false; }
    if (deferred) {
        write_show_flags(s_phys_capture);
        if (!finish_add_component(owner, s_phys_capture)) { API::get()->log_info("[Halo-CampE-UEVR] SCOPE: deferred finish FAILED"); return false; }
    }
    if (auto* tt = s_phys_capture->get_property_data<API::UObject*>(L"TextureTarget")) *tt = s_phys_rt;
    else API::get()->log_info("[Halo-CampE-UEVR] SCOPE: TextureTarget property not found");
    if (auto* src = s_phys_capture->get_property_data<uint8_t>(L"CaptureSource")) *src = (uint8_t)g_cfg.scope_capture_source;
    s_phys_capture->set_bool_property(L"bCaptureEveryFrame", false);
    s_phys_capture->set_bool_property(L"bCaptureOnMovement", false);
    s_phys_capture->set_bool_property(L"bAlwaysPersistRenderingState", true);
    apply_capture_pp();

    // ---- the lens
    auto* smc_cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.StaticMeshComponent");
    if (smc_cls == nullptr) return false;
    s_lens = API::get()->add_component_by_class(owner, smc_cls, false);
    if (s_lens == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SCOPE: lens add_component FAILED"); return false; }
    // Round lens: the engine Cylinder squashed flat along its axis -- its cap is a disc with a
    // planar UV map, so the render target shows as a circular crop. Plane (square) as fallback.
    const bool round = (g_cfg.scope_round != 0);
    auto* mesh = API::get()->find_uobject<API::UObject>(round ? L"StaticMesh /Engine/BasicShapes/Cylinder.Cylinder"
                                                              : L"StaticMesh /Engine/BasicShapes/Plane.Plane");
    if (round && mesh == nullptr) mesh = load_asset_by_path("/Engine/BasicShapes/Cylinder.Cylinder");   // BasicShapes are loadable, just not resident (proven 2026-08-19)
    if (round && mesh == nullptr) API::get()->log_info("[Halo-CampE-UEVR] SCOPE: Cylinder mesh not found, square Plane fallback");
    if (mesh == nullptr) mesh = API::get()->find_uobject<API::UObject>(L"StaticMesh /Engine/BasicShapes/Plane.Plane");
    if (mesh == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SCOPE: no lens mesh (Cylinder/Plane) found"); return false; }
    s_round_mesh = round && (mesh != nullptr);
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; *reinterpret_cast<void**>(p) = mesh; s_lens->call_function(L"SetStaticMesh", p); }
    // No collision, no shadow.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 0; s_lens->call_function(L"SetCollisionEnabled", p); }
    s_lens->set_bool_property(L"CastShadow", false);

    // Material: the VR pass-through (exposure-correct) if the LogicMod pak is present, else the
    // stock opaque pass-through. SlateUI = our render target, tint white.
    // EXPOSURE: the stock Widget3DPassThrough is multiplied by the scene's pre-exposure (the "dark
    // crosshair" mechanism); the VREditor WidgetVRPassThrough variants divide it back out. Try the
    // exposure-correct ones first (LogicMod pak), then stock, and say which one we got.
    // LOADER SELF-TEST (once): run an asset that is CERTAIN to be loadable (the stock widget
    // material, already in memory) through load_asset_by_path. Null here = the loader itself is
    // broken (marshalling); non-null with the pak assets still nulling = the loose IoStore
    // container's packages are not registered in the package store (pak route needs UE4SS).
    {
        static bool tested = false;
        if (!tested) {
            tested = true;
            auto* t = load_asset_by_path("/Engine/EngineMaterials/Widget3DPassThrough_Opaque.Widget3DPassThrough_Opaque");
            API::get()->log_info("[Halo-CampE-UEVR] SCOPE-LOADTEST: known-good asset -> %p", (void*)t);
            // Round-lens candidates: which flat-ish primitives does THIS build actually cook, and
            // is the procedural-mesh module present? (Cylinder already proven missing.)
            const char* shapes[] = { "/Engine/BasicShapes/Cylinder.Cylinder", "/Engine/BasicShapes/Sphere.Sphere",
                                     "/Engine/BasicShapes/Cone.Cone", "/Engine/BasicShapes/Cube.Cube",
                                     "/Engine/BasicShapes/Plane.Plane" };
            for (const char* s : shapes) {
                const std::string a(s);
                auto* m = API::get()->find_uobject<API::UObject>((L"StaticMesh " + std::wstring(a.begin(), a.end())).c_str());
                if (m == nullptr) m = load_asset_by_path(s);
                API::get()->log_info("[Halo-CampE-UEVR] SCOPE-MESH: %s -> %p", s, (void*)m);
            }
            auto* pmc = API::get()->find_uobject<API::UClass>(L"Class /Script/ProceduralMeshComponent.ProceduralMeshComponent");
            API::get()->log_info("[Halo-CampE-UEVR] SCOPE-MESH: ProceduralMeshComponent class -> %p", (void*)pmc);
        }
    }
    API::UObject* base = nullptr; const char* used = "?";
    struct Cand { const char* path; bool load; const char* name; } cands[] = {
        {"/Engine/VREditor/UI/WidgetVRPassThrough.WidgetVRPassThrough", true, "VREditor base (exposure-correct)"},
        {"/Engine/VREditor/UI/WidgetVRPassThrough_Translucent_OneSided.WidgetVRPassThrough_Translucent_OneSided", true, "VREditor Translucent (exposure-correct)"},
        {"MaterialInstanceConstant /Engine/EngineMaterials/Widget3DPassThrough_Opaque.Widget3DPassThrough_Opaque", false, "stock Opaque"},
        {"MaterialInstanceConstant /Engine/EngineMaterials/Widget3DPassThrough_Translucent.Widget3DPassThrough_Translucent", false, "stock Translucent"},
    };
    for (const auto& cd : cands) {
        if (cd.load) base = find_or_load_material(cd.path);
        else { const std::string a(cd.path); base = API::get()->find_uobject<API::UObject>(std::wstring(a.begin(), a.end()).c_str()); }
        if (base != nullptr) { used = cd.name; break; }
    }
    if (base == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SCOPE: no pass-through material"); return false; }
    API::get()->log_info("[Halo-CampE-UEVR] SCOPE: lens material = %s", used);
    {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<int32_t*>(p) = 0; *reinterpret_cast<void**>(p + 8) = base;
        s_lens->call_function(L"CreateDynamicMaterialInstance", p);
        s_mid = *reinterpret_cast<API::UObject**>(p + 24);
    }
    if (s_mid == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SCOPE: MID FAILED"); return false; }
    {
        alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
        API::FName nm = make_fname(L"SlateUI"); memcpy(q, &nm, sizeof(int32_t) * 2);
        *reinterpret_cast<void**>(q + 8) = s_phys_rt;
        s_mid->call_function(L"SetTextureParameterValue", q);
    }
    {
        alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
        API::FName nm = make_fname(L"TintColorAndOpacity"); memcpy(q, &nm, sizeof(int32_t) * 2);
        auto* c = reinterpret_cast<float*>(q + 8); c[0] = 1; c[1] = 1; c[2] = 1; c[3] = 1;
        s_mid->call_function(L"SetVectorParameterValue", q);
    }
    {
        alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
        API::FName nm = make_fname(L"OpacityFromTexture"); memcpy(q, &nm, sizeof(int32_t) * 2);
        *reinterpret_cast<float*>(q + 8) = 0.0f;
        s_mid->call_function(L"SetScalarParameterValue", q);
    }
    set_hidden(s_lens, true);

    // ---- etched reticle: a second quad a hair in front of the lens, crosshair PNG from the
    // profile dir (alpha = shape). Attached to the gun, so it ROLLS with the gun -- which is what
    // a real etched reticle does; only the world image behind it stays level (scoperollfix).
    if (g_cfg.scope_reticle != 0) {
        char png[MAX_PATH] = {0};
        strncpy_s(png, g_cfg_path, sizeof(png) - 1);
        if (char* sl = strrchr(png, '\\')) sl[1] = 0; else png[0] = 0;
        strncat_s(png, "scope_reticle.png", _TRUNCATE);
        auto* tex = import_texture_file(png);
        auto* tbase = API::get()->find_uobject<API::UObject>(L"MaterialInstanceConstant /Engine/EngineMaterials/Widget3DPassThrough_Translucent.Widget3DPassThrough_Translucent");
        if (tex != nullptr && tbase != nullptr) {
            s_ret = API::get()->add_component_by_class(owner, smc_cls, false);
            if (s_ret != nullptr) {
                { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; *reinterpret_cast<void**>(p) = mesh; s_ret->call_function(L"SetStaticMesh", p); }
                { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 0; s_ret->call_function(L"SetCollisionEnabled", p); }
                s_ret->set_bool_property(L"CastShadow", false);
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                *reinterpret_cast<int32_t*>(p) = 0; *reinterpret_cast<void**>(p + 8) = tbase;
                s_ret->call_function(L"CreateDynamicMaterialInstance", p);
                s_ret_mid = *reinterpret_cast<API::UObject**>(p + 24);
                if (s_ret_mid != nullptr) {
                    { alignas(16) uint8_t q[RIG_PARAM_BUF] = {0}; API::FName nm = make_fname(L"SlateUI"); memcpy(q, &nm, sizeof(int32_t) * 2); *reinterpret_cast<void**>(q + 8) = tex; s_ret_mid->call_function(L"SetTextureParameterValue", q); }
                    { alignas(16) uint8_t q[RIG_PARAM_BUF] = {0}; API::FName nm = make_fname(L"OpacityFromTexture"); memcpy(q, &nm, sizeof(int32_t) * 2); *reinterpret_cast<float*>(q + 8) = 1.0f; s_ret_mid->call_function(L"SetScalarParameterValue", q); }
                }
                set_hidden(s_ret, true);
            }
        } else {
            API::get()->log_info("[Halo-CampE-UEVR] SCOPE: reticle skipped (tex %p mat %p, '%s')", (void*)tex, (void*)tbase, png);
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] SCOPE: built (rt %dx%d fmt %d, capture source %d)", g_cfg.scope_res, g_cfg.scope_res, g_cfg.scope_rt_format, g_cfg.scope_capture_source);
    return true;
}

void attach_lens(API::UObject* wroot, const ScopeCfg& sc) {
    if (s_lens == nullptr || wroot == nullptr) return;
    if (s_attached_to != wroot) {
        // K2_AttachToComponent(Parent, SocketName, LocationRule, RotationRule, ScaleRule, bWeld): KeepRelative = 0.
        alignas(16) uint8_t pa[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<void**>(pa) = wroot;
        API::FName none = make_fname(L"None"); memcpy(pa + 8, &none, sizeof(int32_t) * 2);
        pa[16] = 0; pa[17] = 0; pa[18] = 0; pa[19] = 0;
        s_lens->call_function(L"K2_AttachToComponent", pa);
        if (s_ret != nullptr) s_ret->call_function(L"K2_AttachToComponent", pa);
        s_attached_to = wroot;
    }
    // Relative transform from config (cm / deg), re-applied every tick so live edits land.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; auto* v = reinterpret_cast<double*>(p); v[0] = sc.pos[0]; v[1] = sc.pos[1]; v[2] = sc.pos[2]; s_lens->call_function(L"K2_SetRelativeLocation", p); }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; auto* r = reinterpret_cast<double*>(p); r[0] = sc.rot[0]; r[1] = sc.rot[1]; r[2] = sc.rot[2]; s_lens->call_function(L"K2_SetRelativeRotation", p); }
    // Cylinder: 100 cm diameter, 100 cm tall along Z -> squash Z to a sliver so it is a disc.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; auto* v = reinterpret_cast<double*>(p); v[0] = sc.size; v[1] = sc.size; v[2] = s_round_mesh ? 0.0005 : sc.size; s_lens->call_function(L"SetRelativeScale3D", p); }   // no K2_ variant exists for scale

    // Etched reticle rides the same transform, a hair towards the shooter so it draws over the lens.
    if (s_ret != nullptr) {
        const float rs = sc.size * g_cfg.scope_reticle_scale;
        { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; auto* v = reinterpret_cast<double*>(p); v[0] = sc.pos[0] - 0.25; v[1] = sc.pos[1]; v[2] = sc.pos[2]; s_ret->call_function(L"K2_SetRelativeLocation", p); }
        { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; auto* r = reinterpret_cast<double*>(p); r[0] = sc.rot[0]; r[1] = sc.rot[1]; r[2] = sc.rot[2]; s_ret->call_function(L"K2_SetRelativeRotation", p); }
        { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; auto* v = reinterpret_cast<double*>(p); v[0] = rs; v[1] = rs; v[2] = s_round_mesh ? 0.0005 : rs; s_ret->call_function(L"SetRelativeScale3D", p); }
    }
}

} // namespace

void scope_reset() {
    // Components die with their owner; just forget them.
    s_owner = nullptr; s_phys_rt = nullptr; s_phys_capture = nullptr; s_lens = nullptr; s_mid = nullptr;
    s_ret = nullptr; s_ret_mid = nullptr;
    s_attached_to = nullptr; s_active = false; s_cfg_idx = -1;
}

// ONE-SHOT CVAR DISCOVERY (Config::scope_cvar_dump): list every console object whose name contains
// Lumen / SceneCapture / Capture, so the knobs that decide whether a scene capture renders Lumen

namespace {

void cvar_dump_once() {
    static bool done = false;
    if (done || !g_cfg.scope_cvar_dump) return;
    done = true;
    auto* cm = API::get()->get_console_manager();
    if (cm == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SCOPE-CVAR: no console manager"); return; }
    auto& objs = cm->get_console_objects();
    int n = 0;
    for (int32_t i = 0; i < objs.count; ++i) {
        const wchar_t* k = objs.data[i].key;
        if (k == nullptr) continue;
        std::wstring w(k);
        std::wstring lw = w; for (auto& ch : lw) ch = (wchar_t)towlower(ch);
        const bool sc  = lw.find(L"scenecapture") != std::wstring::npos;
        const bool lum = lw.find(L"lumen") != std::wstring::npos &&
                         (lw.find(L"allow") != std::wstring::npos || lw.find(L"supported") != std::wstring::npos ||
                          lw.find(L"capture") != std::wstring::npos || lw.find(L"screenprobegather") != std::wstring::npos ||
                          lw.find(L"enable") != std::wstring::npos || lw == L"r.lumen.diffuseindirect" || lw == L"r.lumen.reflections");
        const bool misc = (lw.find(L"dynamicglobalillumination") != std::wstring::npos || lw.find(L"reflectionmethod") != std::wstring::npos);
        if (sc || lum || misc) {
            API::get()->log_info("[Halo-CampE-UEVR] SCOPE-CVAR: %ls", k);
            ++n;
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] SCOPE-CVAR: %d matches of %d console objects", n, (int)objs.count);
}

// Live cvar: scopeseptrans (see Config.hpp). Applied on change; unknown name -> one-shot dump of
// every console object containing "translucen" so the real knob is read from THIS build.
void apply_sep_trans() {
    static int applied = -2;
    if (g_cfg.scope_sep_trans == applied || g_cfg.scope_sep_trans < 0) return;
    auto* cm = API::get()->get_console_manager();
    if (cm == nullptr) return;
    auto* var = cm->find_variable(L"r.SeparateTranslucency");
    if (var != nullptr) {
        var->set(g_cfg.scope_sep_trans);
        applied = g_cfg.scope_sep_trans;
        API::get()->log_info("[Halo-CampE-UEVR] SCOPE: r.SeparateTranslucency -> %d (now %d)", g_cfg.scope_sep_trans, var->get_int());
        return;
    }
    static bool dumped = false;
    applied = g_cfg.scope_sep_trans;   // do not retry every tick
    if (dumped) return;
    dumped = true;
    API::get()->log_info("[Halo-CampE-UEVR] SCOPE: r.SeparateTranslucency NOT FOUND, translucency cvars:");
    auto& objs = cm->get_console_objects();
    for (int32_t i = 0; i < objs.count; ++i) {
        const wchar_t* k = objs.data[i].key;
        if (k == nullptr) continue;
        std::wstring lw(k); for (auto& ch : lw) ch = (wchar_t)towlower(ch);
        if (lw.find(L"translucen") != std::wstring::npos) API::get()->log_info("[Halo-CampE-UEVR] SCOPE-CVAR: %ls", k);
    }
}

// A/B GPU cost harness (scopeabtest): while a scope is held, the capture is toggled 4 s on /
// 4 s off and the engine tick dt of each phase is accumulated (first 0.5 s after each switch
// discarded as settle time). The ON-minus-OFF dt delta IS the capture's render cost -- the only
// measurement a plugin can make of GPU work it does not own. Caveat: under VR reprojection dt
// quantises to frame multiples, so a cost smaller than the headroom can read as ~0.
void scope_ab_tick(float dt) {
    static int    phase = -1;     // -1 idle, 0 = capture ON, 1 = capture OFF
    static float  t = 0.0f;
    static double sum[2] = {0, 0}; static int n[2] = {0, 0};
    if (!g_cfg.scope_ab_test || !s_active) {
        if (phase == 1) set_capture_enabled(true);   // never leave the capture off after a test
        phase = -1; t = 0; sum[0] = sum[1] = 0; n[0] = n[1] = 0;
        return;
    }
    if (phase < 0) { phase = 0; t = 0; set_capture_enabled(true); }
    t += dt;
    if (t > 0.5f) { sum[phase] += dt; ++n[phase]; }
    if (t >= 4.0f) {
        if (phase == 1 && n[0] > 0 && n[1] > 0) {
            const double on  = sum[0] / n[0] * 1000.0;
            const double off = sum[1] / n[1] * 1000.0;
            API::get()->log_info("[Halo-CampE-UEVR] SCOPE-AB: capture ON %.2fms (%d f)  OFF %.2fms (%d f)  delta %+.2fms",
                                 on, n[0], off, n[1], on - off);
            sum[0] = sum[1] = 0; n[0] = n[1] = 0;
        }
        phase ^= 1; t = 0;
        set_capture_enabled(phase == 0);
    }
}

} // namespace

void scope_update(float dt) {
    if (!g_cfg.enabled || !g_cfg.scope_enabled || !g_cfg.scope_lens || s_phys_failed) {
        // Off must mean off even mid-aim: a lens switched off while showing kept its last image
        // and its capture running, because this return used to skip the hide below.
        if (s_active) { set_hidden(s_lens, true); set_hidden(s_ret, true); set_capture_enabled(false); s_active = false; }
        return;
    }
    if (g_cfg.scope_cfg_count <= 0) return;   // no scopewpn= entries: this path builds nothing, the pane path above is the scope
    cvar_dump_once();
    apply_sep_trans();

    auto* rig = rig_tracked_component();
    auto* owner = (rig != nullptr) ? rig->get_outer() : nullptr;
    if (owner == nullptr) { if (s_owner != nullptr) scope_reset(); return; }
    if (owner != s_owner) {
        scope_reset();
        if (!build(owner)) { s_phys_failed = true; return; }
    }

    const std::string key = weapon_key();
    const ScopeCfg* sc = key.empty() ? nullptr : find_cfg(key);
    const bool want = (sc != nullptr) && !holster_fire_suppressed()
                   && !g_stick_mode_active.load(std::memory_order_relaxed);
    if (!want) {
        if (!key.empty() && sc == nullptr && key != s_key_logged) {
            s_key_logged = key;
            API::get()->log_info("[Halo-CampE-UEVR] SCOPE: holding %s (no scope entry)", key.c_str());
        }
        if (s_active) { set_hidden(s_lens, true); set_hidden(s_ret, true); set_capture_enabled(false); s_active = false; }
        scope_ab_tick(dt);   // resets the harness so a later activation starts a fresh test
        return;
    }

    auto* wroot = fp_weapon_root();
    if (wroot == nullptr) return;
    attach_lens(wroot, *sc);
    if (!s_active) {
        set_hidden(s_lens, false);
        set_hidden(s_ret, false);
        set_capture_enabled(true);
        s_active = true;
        API::get()->log_info("[Halo-CampE-UEVR] SCOPE: active on %s (fov %.1f)", key.c_str(), sc->fov);
    }
    if (auto* f = s_phys_capture->get_property_data<float>(L"FOVAngle")) *f = sc->fov;
    apply_capture_pp();   // live: scopeev / scopelumen
    probe_rt();           // scopeprobe
    if (auto* src = s_phys_capture->get_property_data<uint8_t>(L"CaptureSource")) {
        if (*src != (uint8_t)g_cfg.scope_capture_source) {
            *src = (uint8_t)g_cfg.scope_capture_source;
            API::get()->log_info("[Halo-CampE-UEVR] SCOPE: capture source -> %d", g_cfg.scope_capture_source);
        }
    }
    // Live tint multiplier (brightness) -- >1 lifts a pre-exposure-darkened picture.
    if (s_mid != nullptr) {
        alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
        API::FName nm = make_fname(L"TintColorAndOpacity"); memcpy(q, &nm, sizeof(int32_t) * 2);
        auto* col = reinterpret_cast<float*>(q + 8); col[0] = g_cfg.scope_tint; col[1] = g_cfg.scope_tint; col[2] = g_cfg.scope_tint; col[3] = 1.0f;
        s_mid->call_function(L"SetVectorParameterValue", q);
    }
    if (s_ret_mid != nullptr) {
        alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
        API::FName nm = make_fname(L"TintColorAndOpacity"); memcpy(q, &nm, sizeof(int32_t) * 2);
        auto* col = reinterpret_cast<float*>(q + 8); col[0] = g_cfg.scope_reticle_tint[0]; col[1] = g_cfg.scope_reticle_tint[1]; col[2] = g_cfg.scope_reticle_tint[2]; col[3] = 1.0f;
        s_ret_mid->call_function(L"SetVectorParameterValue", q);
    }

    scope_ab_tick(dt);

    // The capture sits at the lens and looks along the AIM: the ray the bullets follow.
    Vec3 lpos{};
    if (call_ret_vec3(s_lens, L"K2_GetComponentLocation", &lpos)) {
        // EYE-PROXIMITY GATE (scopeeyedist): the second scene render runs only while the lens is
        // at the eye. Gated off, the lens keeps its last frame and the placement writes are
        // skipped too. The A/B harness owns the capture toggle while it is running.
        if (g_cfg.scope_eye_dist > 0.0f && g_cfg.scope_ab_test == 0) {
            static bool s_near = true;
            Vec3 cam{};
            bool have = false;
            if (auto* pc = API::get()->get_player_controller(0)) {
                if (auto* pcm_p = reinterpret_cast<API::UObject*>(pc)->get_property_data<API::UObject*>(L"PlayerCameraManager")) {
                    if (*pcm_p != nullptr) have = call_ret_vec3(*pcm_p, L"GetCameraLocation", &cam);
                }
            }
            if (have) {
                const float dx = cam.x - lpos.x, dy = cam.y - lpos.y, dz = cam.z - lpos.z;
                const float d2 = dx * dx + dy * dy + dz * dz;
                const float r_on = g_cfg.scope_eye_dist, r_off = g_cfg.scope_eye_dist + 8.0f;
                if (s_near) { if (d2 > r_off * r_off) s_near = false; }
                else        { if (d2 < r_on * r_on)   s_near = true; }
            }
            set_capture_enabled(s_near);
            if (!s_near) return;
        }
        double cp = 0.0, cy = 0.0;
        if (read_control_rotation(&cp, &cy, nullptr)) {
            // Push the capture forward along the aim ray past the weapon's own geometry
            // (scopecamfwd, cm) -- at scope FOVs the parallax from this offset is negligible.
            const double pr = cp * 3.14159265358979323846 / 180.0;
            const double yr = cy * 3.14159265358979323846 / 180.0;
            const double f  = (double)g_cfg.scope_cam_fwd;
            lpos.x += (float)(cos(pr) * cos(yr) * f);
            lpos.y += (float)(cos(pr) * sin(yr) * f);
            lpos.z += (float)(sin(pr) * f);
            { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; auto* v = reinterpret_cast<double*>(p); v[0] = lpos.x; v[1] = lpos.y; v[2] = lpos.z; s_phys_capture->call_function(L"K2_SetWorldLocation", p); }
            // Roll: real optics are rotationally symmetric -- rolling the gun must not roll the
            // image. The lens disc rolls with the gun, so the capture takes the weapon root's
            // world roll (about the barrel = the disc's spin) to cancel it (scoperollfix).
            double roll = 0.0;
            if (g_cfg.scope_roll_fix != 0 && wroot != nullptr) {
                Vec3 wrot{};
                if (call_ret_vec3(wroot, L"K2_GetComponentRotation", &wrot)) roll = (double)g_cfg.scope_roll_fix * wrot.z;   // FRotator: x=Pitch y=Yaw z=Roll
            }
            { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; auto* r = reinterpret_cast<double*>(p); r[0] = cp; r[1] = cy; r[2] = roll; s_phys_capture->call_function(L"K2_SetWorldRotation", p); }

            // RATE CAP (scopehz): manual CaptureScene at the display rate instead of the engine's
            // uncapped tick rate. Runs AFTER placement so the captured frame uses this tick's pose.
            if (g_cfg.scope_hz > 0.0f && s_cap_on) {
                static float acc = 1.0f;   // first gated-on tick captures immediately
                acc += dt;
                const float period = 1.0f / g_cfg.scope_hz;
                if (acc >= period) {
                    acc = (acc > 2.0f * period) ? 0.0f : acc - period;   // no debt after a stall
                    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                    s_phys_capture->call_function(L"CaptureScene", p);
                }
            }
        }
    }
}

// ---- PHYSICAL PER-WEAPON SCOPES (Scope.hpp: scopewpn= entries and their capture knobs). A
// second hoisted family beside parse_scope_key above, early-return for the same C1061 reason.
// `scope`, `scoperes` and `scopelumen` are parsed by parse_scope_key, which owns those names.
bool parse_physscope_key(const char* key, const char* val, double v) {
    if (_stricmp(key, "scopelens") == 0) { g_cfg.scope_lens = (v != 0.0); return true; }
    if (_stricmp(key, "scopesource")    == 0) { g_cfg.scope_capture_source = (int)v; return true; }
    if (_stricmp(key, "scopetint")      == 0) { g_cfg.scope_tint = clampf((float)v, 0.01f, 20.0f); return true; }
    if (_stricmp(key, "scopertfmt")     == 0) { g_cfg.scope_rt_format = (int)v; return true; }
    if (_stricmp(key, "scopeev")        == 0) { g_cfg.scope_ev = clampf((float)v, -10.0f, 10.0f); return true; }
    if (_stricmp(key, "scopepp")        == 0) { g_cfg.scope_pp_override = (int)v; return true; }
    if (_stricmp(key, "scopecvardump")  == 0) { g_cfg.scope_cvar_dump = (v != 0.0); return true; }
    if (_stricmp(key, "scoperound")     == 0) { g_cfg.scope_round = (int)v; return true; }
    if (_stricmp(key, "scopeshowflags") == 0) { g_cfg.scope_showflags = (int)v; return true; }
    if (_stricmp(key, "scopeprobe")     == 0) { g_cfg.scope_probe = (int)v; return true; }
    if (_stricmp(key, "scopetonecurve") == 0) { g_cfg.scope_tone_curve = clampf((float)v, -1.0f, 1.0f); return true; }
    if (_stricmp(key, "scopeseptrans")  == 0) { g_cfg.scope_sep_trans = (int)v; return true; }
    if (_stricmp(key, "scopesfflags")   == 0) { strncpy_s(g_cfg.scope_sf_names, val, sizeof(g_cfg.scope_sf_names) - 1); return true; }
    if (_stricmp(key, "scopecamfwd")    == 0) { g_cfg.scope_cam_fwd = clampf((float)v, 0.0f, 500.0f); return true; }
    if (_stricmp(key, "scoperollfix")   == 0) { g_cfg.scope_roll_fix = (int)v; return true; }
    if (_stricmp(key, "scopereticle")   == 0) { g_cfg.scope_reticle = (int)v; return true; }
    if (_stricmp(key, "scopereticlescale") == 0) { g_cfg.scope_reticle_scale = clampf((float)v, 0.05f, 2.0f); return true; }
    if (_stricmp(key, "scopeabtest")    == 0) { g_cfg.scope_ab_test = (int)v; return true; }
    if (_stricmp(key, "scopeeyedist")   == 0) { g_cfg.scope_eye_dist = clampf((float)v, 0.0f, 300.0f); return true; }
    if (_stricmp(key, "scopehz")        == 0) { g_cfg.scope_hz = clampf((float)v, 0.0f, 240.0f); return true; }
    if (_stricmp(key, "scopereticletint")  == 0) {
            float r = 1, g = 1, b = 1;
            if (sscanf_s(val, "%f,%f,%f", &r, &g, &b) == 3) { g_cfg.scope_reticle_tint[0] = r; g_cfg.scope_reticle_tint[1] = g; g_cfg.scope_reticle_tint[2] = b; } return true; }
    if (_stricmp(key, "scopewpn")       == 0) {
        // scopewpn=KEY,fov,x,y,z,pitch,yaw,roll,size  -- replaces an existing entry for KEY
        ScopeCfg sc; char k[64] = {0};
        const int n = sscanf_s(val, "%63[^,],%f,%f,%f,%f,%f,%f,%f,%f", k, (unsigned)sizeof(k),
                               &sc.fov, &sc.pos[0], &sc.pos[1], &sc.pos[2], &sc.rot[0], &sc.rot[1], &sc.rot[2], &sc.size);
        if (n >= 2) {
            strncpy_s(sc.key, k, sizeof(sc.key) - 1);
            int slot = -1;
            for (int i = 0; i < g_cfg.scope_cfg_count; ++i) if (_stricmp(g_cfg.scopes[i].key, sc.key) == 0) slot = i;
            if (slot < 0 && g_cfg.scope_cfg_count < 8) slot = g_cfg.scope_cfg_count++;
            if (slot >= 0) g_cfg.scopes[slot] = sc;
        }
        return true;
    }
    return false;
}

namespace {

// THE SCOPE (the physical-lens system): a second camera down the aim ray rendered onto a lens
// mounted on the weapon. It resolves its own weapon, attaches its own components, and switches the
// capture off when no scoped weapon is held; the tick just drives it. It runs above the tick's
// early-outs so the lens hides on parked ticks (menus, seats).
void scopelens_game_tick_after_offsets(float dt) {
    g_tick_stage = "scope";
    scope_update(dt);
    g_tick_stage = "after scope";
}

void scopelens_rig_lost() {
    scope_reset();
}

// The physical lens owns the scope; the pane never toggles.
bool scopelens_trigger_stood_down(bool& s_down) {
    if (g_cfg.scope_lens) { s_down = false; g_scope_active = false; return true; }
    return false;
}

bool scopelens_pane_stands_down() {
    return g_cfg.scope_lens;
}

} // namespace

namespace {
bool scope_lens_enabled() { CFG_HOOK_READ; return g_cfg.scope_lens; }
}  // namespace

constinit const FeatureHooks kScopeLensHooks{
    .key                      = "scopelens",
    .parse_key                = &parse_physscope_key,
    .game_tick_after_offsets  = &scopelens_game_tick_after_offsets,
    .rig_lost                 = &scopelens_rig_lost,
    .scope_trigger_stood_down = &scopelens_trigger_stood_down,
    .scope_pane_stands_down   = &scopelens_pane_stands_down,
    .enabled                    = &scope_lens_enabled,
    .services                   = 0,
};

} // namespace halo
