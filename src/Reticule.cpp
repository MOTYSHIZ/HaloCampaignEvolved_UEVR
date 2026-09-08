// The aim reticule: showing the player where the shot actually goes.
//
// TWO INDEPENDENT RETICULES live here, because neither alone covers every case:
//
//   * MESH RETICULE   -- our own StaticMeshComponent, created once and moved every tick. Exists
//     because the obvious route cannot work: UE strips DrawDebug* from shipping builds, so those
//     calls return cleanly and draw nothing at all.
//   * WIDGET RETICULE -- hosts one of the game's OWN reticle widgets on a WidgetComponent, so the
//     per-weapon art, the hit marker and the reload/scope states come along for free instead of
//     being re-implemented.
//
// Both are driven from update() in Plugin.cpp: *_ensure() creates on demand, *_move() repositions.
// Creation failures LATCH (g_ret_mesh_failed / g_ret_widget_failed) so a broken configuration costs
// one attempt rather than one attempt per tick.
//
// The asset helpers at the top are here rather than in a general utility module because the reticule
// is their only caller: UEVR's find_uobject only searches what is ALREADY loaded, so a mod-supplied
// material shipped in an added pak is invisible to it until something forces a load.

// API.hpp, NOT Plugin.hpp -- Plugin.hpp defines the plugin entry points and may only be included
// by Plugin.cpp. See UeObject.hpp.
#include "uevr/API.hpp"
#include "Reticule.hpp"
#include "Scope.hpp"   // g_scope_active -- the zoom-fit gate
#include "Config.hpp"
#include "Math.hpp"
#include "UeObject.hpp"
#include "DevTools.hpp"   // HALO_VR_DEV -- the vsco state line below is diagnostics only
#include "XrLayer.hpp"    // xrlayer_live() -- the hide's actual driver, reported alongside the bit

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

using namespace uevr;

namespace halo {

// Storage for the resolved HUD reticle widgets. Defined here rather than in Plugin.cpp because
// Plugin.cpp's body sits in an anonymous namespace, which would make the definition internal to
// that translation unit. reticle_rescan() there populates it via the extern declarations.
ReticleTarget g_reticles[8];
int      g_reticle_count = 0;
uint32_t g_reticle_scan_tick = 0;
// Load an asset by object path, pulling it off disk if it is not already in memory.
//
// UEVR's find_uobject only searches what is ALREADY loaded, and assets shipped in an added pak are
// not loaded until something references them -- so a mod-supplied material is invisible to it. This
// drives UKismetSystemLibrary::LoadAsset_Blocking instead, which resolves and loads synchronously.
//
// The argument is a TSoftObjectPtr, marshalled by hand (UE 5.5 layout, taken from the engine
// headers rather than assumed):
//   TPersistentObjectPtr = { FWeakObjectPtr WeakPtr(8); FSoftObjectPath ObjectID }
//   FSoftObjectPath      = { FTopLevelAssetPath{FName PackageName(8); FName AssetName(8)};
//                            FString SubPathString(16) }
// giving 40 bytes, with the returned UObject* immediately after.
//
// `path` is a full object path ("/Game/HaloVR/M_HaloVRReticle.M_HaloVRReticle") or a package path
// ("/Game/HaloVR/M_HaloVRReticle"), which is completed by repeating the trailing name.
API::UObject* load_asset_by_path(const char* path) {
    if (path == nullptr || path[0] != '/') return nullptr;

    std::string pkg{path};
    std::string asset;
    const size_t dot = pkg.rfind('.');
    const size_t slash = pkg.rfind('/');
    if (dot != std::string::npos && dot > slash) {
        asset = pkg.substr(dot + 1);
        pkg   = pkg.substr(0, dot);
    } else {
        asset = (slash == std::string::npos) ? pkg : pkg.substr(slash + 1);
    }
    if (pkg.empty() || asset.empty()) return nullptr;

    auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.KismetSystemLibrary");
    auto* ksl = (cls != nullptr) ? cls->get_class_default_object() : nullptr;
    if (ksl == nullptr) return nullptr;

    const std::wstring wpkg(pkg.begin(), pkg.end());
    const std::wstring wasset(asset.begin(), asset.end());
    API::FName fpkg   = make_fname(wpkg.c_str());
    API::FName fasset = make_fname(wasset.c_str());
    if (fpkg.comparison_index == 0 || fasset.comparison_index == 0) return nullptr;

    alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
    memcpy(q + 8,  &fpkg,   sizeof(int32_t) * 2);   // AssetPath.PackageName
    memcpy(q + 16, &fasset, sizeof(int32_t) * 2);   // AssetPath.AssetName
    // SubPathString stays an empty FString (null data, 0/0) -- already zeroed.
    ksl->call_function(L"LoadAsset_Blocking", q);
    auto* obj = *reinterpret_cast<API::UObject**>(q + 40);

    API::get()->log_info("[Halo-CampE-UEVR] LoadAsset_Blocking('%s.%s') -> %p",
                         pkg.c_str(), asset.c_str(), (void*)obj);
    return obj;
}

// Find a material by path, loading it from disk if necessary. Accepts either a bare object path or
// one prefixed with a class name, and tries the classes a material can actually be.
// The optional exposure-compensated material widget_quad_begin prefers. ONE definition, shared with
// reticule_prime_material_cache(), because the negative-result cache in find_or_load_material keys on
// the EXACT path string -- a second literal that drifted by one character would prime a cache entry
// the real request never hits, and the stall would come back with the warmup looking like it ran.
constexpr char kVREditorPassThroughPath[] =
    "/Engine/VREditor/UI/WidgetVRPassThrough_Translucent_OneSided."
    "WidgetVRPassThrough_Translucent_OneSided";

API::UObject* find_or_load_material(const std::string& object_path) {
    // ---- THE ABSENT CACHE IS CHECKED FIRST -- ABOVE THE find_uobject PROBES, NOT BELOW THEM.
    //
    // CORRECTED 2026-08-25 BY MEASUREMENT. The previous version checked this cache only in front of
    // the blocking load and let the three find_uobject probes run every time, on the stated
    // reasoning that "they are a cheap hash lookup". THAT REASONING IS WRONG, and it is why the
    // ~500 ms per-quad hitch survived the frontend warm-up that was supposed to end it.
    //
    // UEVR's sdk::find_uobject is a CACHE ON HIT AND A FULL LINEAR SCAN ON MISS
    // (UESDK/src/sdk/UObjectArray.cpp): a miss walks every entry of FUObjectArray and builds
    // object->get_full_name() -- a fresh wstring, outer chain walked -- for each one, to compare
    // against the requested name. With ~294k live objects mid-mission that is ~150-180 ms PER MISS,
    // and this function issues THREE of them (MaterialInstanceConstant/Material/
    // MaterialInstanceDynamic) for a path that is absent on every stock install. 3 x ~170 ms is the
    // 449-549 ms that the 2026-08-25 10:25-10:27 log charged to "unattributed" at reticule creation
    // and to navworld_tick at each new marker slot -- six hitches, one per widget quad created,
    // with NO LoadAsset_Blocking line anywhere near them.
    //
    // The blocking load was only ever the second half of the cost, and it is now the cheap half:
    // the same log shows the frontend prime completing its LoadAsset_Blocking in 87 ms, because at
    // the frontend the object array is a fraction of its mid-mission size. So the whole discovery
    // -- probes AND load -- is now paid once, by reticule_prime_material_cache() at the frontend,
    // where both halves are cheap and nobody is playing.
    //
    // THE TRADE, STATED: a path proven absent stays absent for the session, so a pak mounted
    // mid-session would not be picked up until a restart. This title mounts its paks at startup and
    // the prime self-gates on engine content being queryable, so the window that trade closes does
    // not exist here -- and the alternative is a ~500 ms game-thread stall per widget quad, which
    // in VR is nausea, not a blemish. Game-thread only (both callers run inside update()), so the
    // statics need no synchronisation.
    constexpr int kMaxAbsent = 8;
    static std::string s_absent[kMaxAbsent];
    static int         s_absent_n = 0;
    for (int i = 0; i < s_absent_n; ++i)
        if (s_absent[i] == object_path) return nullptr;       // already proven absent this session

    // PHASE TIMING, PERMANENT AND SELF-ANNOUNCING (the "announce deliberate cost" rule). Three
    // steady_clock reads on a path that runs a handful of times per session, and a line that a
    // healthy build never prints. This cost has now hidden THREE times on this project -- as an
    // unscoped object-array sweep, as an unattributed blocking load, and as these probes -- each
    // time because nothing said out loud how long it took. It says so now.
    const auto t0 = std::chrono::steady_clock::now();

    const std::wstring w(object_path.begin(), object_path.end());
    static const wchar_t* kPrefixes[] = { L"MaterialInstanceConstant ", L"Material ",
                                          L"MaterialInstanceDynamic " };
    API::UObject* found = nullptr;
    for (const wchar_t* pre : kPrefixes) {
        if (auto* m = API::get()->find_uobject<API::UObject>((std::wstring(pre) + w).c_str())) {
            found = m;
            break;
        }
    }
    const auto t1 = std::chrono::steady_clock::now();

    // Only a path NOTHING already resident answers reaches the synchronous package load.
    if (found == nullptr) found = load_asset_by_path(object_path.c_str());
    const auto t2 = std::chrono::steady_clock::now();

    const double probe_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    const double load_ms  = std::chrono::duration<double, std::milli>(t2 - t1).count();
    if (probe_ms + load_ms > 50.0) {
        API::get()->log_info("[Halo-CampE-UEVR] PERF: material lookup '%s' STALLED the game thread "
                             "%.1f ms (find_uobject probes %.1f ms, blocking load %.1f ms) -> %s. "
                             "Each probe MISS is a full object-array walk; this should only ever "
                             "happen once per path, at the frontend prime.",
                             object_path.c_str(), probe_ms + load_ms, probe_ms, load_ms,
                             found != nullptr ? "found" : "ABSENT (cached, will not be retried)");
    }

    if (found == nullptr && s_absent_n < kMaxAbsent) s_absent[s_absent_n++] = object_path;
    return found;
}

// Pay the one-time material discovery OFF the gameplay path. See Reticule.hpp.
void reticule_prime_material_cache() {
    static bool s_done = false;
    if (s_done) return;

    // GATE ON ENGINE CONTENT BEING QUERYABLE, so that a load which FAILS means the asset is
    // genuinely absent and not merely that the object system was not ready yet. The stock Widget3D
    // pass-through is always cooked; until find_uobject can see it, LoadAsset_Blocking is not safe to
    // trust. Without this gate an early false-"absent" would poison the negative cache for a player
    // who actually has the optional HaloCEReticleColor pak -- turning a mitigation into a regression.
    //
    // THROTTLED, because this probe is NOT free while it is failing (2026-08-25). A find_uobject
    // MISS is a full walk of the object array building get_full_name() per entry -- the same
    // property that made the material probes below a ~500 ms stall. Unthrottled, a gate that
    // returns "not yet" was paying that walk on EVERY tick until engine content came up. Once every
    // 30 ticks (~1 s) bounds it, and priming a second later at the frontend costs nobody anything.
    {
        static int s_gate_countdown = 0;
        if (s_gate_countdown > 0) { --s_gate_countdown; return; }
        if (API::get()->find_uobject<API::UObject>(
                L"MaterialInstanceConstant /Engine/EngineMaterials/Widget3DPassThrough_Translucent."
                L"Widget3DPassThrough_Translucent") == nullptr) {
            s_gate_countdown = 30;   // engine content not up yet -- ask again in ~1 s, not next tick
            return;
        }
    }

    s_done = true;

    // PERF DEBT, ANNOUNCED (per the eng-vault "announce-deliberate-cost" rule): this is a deliberate
    // one-time game-thread stall, placed HERE on purpose. widget_quad_begin needs the optional
    // VREditor material at creation, and discovering its ABSENCE on a stock install costs BOTH
    // halves of find_or_load_material -- three find_uobject probe MISSES (each a full object-array
    // walk) and then a synchronous LoadAsset_Blocking pak scan. Paying that at reticule/marker
    // creation is what the field felt as a level-start stall and, measured on 2026-08-25, as a
    // ~500 ms hitch at every new navpoint marker slot. Doing it once at the frontend -- engine up,
    // no mission running, object array still small, nobody playing through it -- moves the cost off
    // the gameplay path AND makes it far smaller (87 ms measured for the load half at the frontend
    // versus ~500 ms mid-mission). The absent cache then makes every later widget_quad_begin free.
    //
    // This is a MITIGATION, not the end state. The real fix is an ASYNC load so even this one never
    // blocks the game thread (peer note 2026-08-24); until that exists, this is the cheap win. Logged
    // loudly so it is never a mystery in a profile and never mistaken for "already fixed".
    API::get()->log_info("[Halo-CampE-UEVR] PERF: priming widget material cache off the gameplay "
                         "path -- a one-time blocking discovery follows IF the optional VREditor "
                         "material is absent (stock install): three full object-array probe walks "
                         "plus a synchronous pak scan. This is deliberate and belongs here, not at "
                         "level start. TODO: make this async so it never blocks at all.");
    auto* m = find_or_load_material(kVREditorPassThroughPath);
    API::get()->log_info("[Halo-CampE-UEVR] PERF: widget material cache primed -- %s. "
                         "Widget-quad creation will not block from here.",
                         m != nullptr ? "present (optional pak mounted)"
                                      : "absent, stock fallback cached -- no per-creation stall");
}

// Load an image file from disk as a UTexture2D via UKismetRenderingLibrary::ImportFileAsTexture2D.
// This is how the reticule gets custom art without shipping a pak: a PNG beside the profile, with
// the shape in its alpha and WHITE in its RGB so the material tint decides the colour.
API::UObject* import_texture_file(const char* path) {
    auto* pc = API::get()->get_player_controller(0);
    if (pc == nullptr || path == nullptr || path[0] == 0) return nullptr;

    auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.KismetRenderingLibrary");
    auto* krl = (cls != nullptr) ? cls->get_class_default_object() : nullptr;
    if (krl == nullptr) return nullptr;

    const std::string a{path};
    const std::wstring w(a.begin(), a.end());

    // ImportFileAsTexture2D(WorldContextObject, FString Filename) -> UTexture2D*.
    // FString is {TCHAR* Data; int32 Num; int32 Max}; the callee only reads it.
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    *reinterpret_cast<void**>(p) = pc;
    const int32_t len = (int32_t)w.size() + 1;
    *reinterpret_cast<const wchar_t**>(p + 8) = w.c_str();
    *reinterpret_cast<int32_t*>(p + 16) = len;
    *reinterpret_cast<int32_t*>(p + 20) = len;
    krl->call_function(L"ImportFileAsTexture2D", p);
    auto* tex = *reinterpret_cast<API::UObject**>(p + 24);

    API::get()->log_info("[Halo-CampE-UEVR] ImportFileAsTexture2D('%s') -> %p", path, (void*)tex);
    return tex;
}

// Build a render target filled with a chosen colour and hand it back for use as SlateUI.
// Returns nullptr on failure, and logs which call failed -- every step here is a marshalled
// UFunction, and a clean return proves nothing.
API::UObject* make_color_rt(float r, float g, float b, float a, int size) {
    auto* pc = API::get()->get_player_controller(0);
    if (pc == nullptr) return nullptr;

    static API::UObject* krl = nullptr;
    static bool tried = false;
    if (!tried) {
        tried = true;
        if (auto* c = API::get()->find_uobject<API::UClass>(
                L"Class /Script/Engine.KismetRenderingLibrary")) {
            krl = c->get_class_default_object();
        }
        API::get()->log_info("[Halo-CampE-UEVR] KismetRenderingLibrary: %s", krl ? "found" : "NOT FOUND");
    }
    if (krl == nullptr) return nullptr;

    // CreateRenderTarget2D(WorldContextObject, int32 Width, int32 Height, ETextureRenderTargetFormat,
    //                      FLinearColor ClearColor, bool bAutoGenerateMipMaps, bool bSupportUAVs)
    API::UObject* rt = nullptr;
    {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<void**>(p) = pc;
        *reinterpret_cast<int32_t*>(p + 8)  = size;
        *reinterpret_cast<int32_t*>(p + 12) = size;
        p[16] = 2;   // RTF_RGBA8
        auto* cc = reinterpret_cast<float*>(p + 20);
        cc[0] = r; cc[1] = g; cc[2] = b; cc[3] = a;
        krl->call_function(L"CreateRenderTarget2D", p);
        rt = *reinterpret_cast<API::UObject**>(p + 40);
    }
    if (rt == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] CreateRenderTarget2D returned null");
        return nullptr;
    }

    // Clear explicitly: the ClearColor argument above sets the asset's clear colour, but the
    // surface is not guaranteed to have been filled with it yet.
    {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<void**>(p)     = pc;
        *reinterpret_cast<void**>(p + 8) = rt;
        auto* cc = reinterpret_cast<float*>(p + 16);
        cc[0] = r; cc[1] = g; cc[2] = b; cc[3] = a;
        krl->call_function(L"ClearRenderTarget2D", p);
    }

    API::get()->log_info("[Halo-CampE-UEVR] colour render target created @%p (%dx%d, rgba %.2f %.2f %.2f %.2f)",
                         (void*)rt, size, size, r, g, b, a);
    return rt;
}

// ---------------------------------------------------------------- WORLD-SPACE MESH RETICULE
//
// Our own StaticMeshComponent, created once and moved every tick. This exists because the debug-
// draw route cannot work: UE strips DrawDebug* from shipping builds, so those calls return
// cleanly and draw nothing.
TrackedObject g_ret_mesh;
// Ray origin, published so the mesh reticule can face the viewer the same way the widget does.
Vec3 g_ret_origin{0.0f, 0.0f, 0.0f};
std::atomic<bool> g_have_ret_origin{false};
std::atomic<float> g_ret_scale_mul{1.0f};
TrackedObject g_ret_mesh_mid;
// Parent path the current MID was built from; a config change triggers a rebind.
std::string   g_applied_mesh_parent;
// The bound reticule texture, kept so its mips can be pinned resident (see below).
TrackedObject g_ret_tex;

// Set the reticule's colour. Cheap enough to call every tick, which is what makes a hit flash
// possible later without any extra machinery.
void reticule_mesh_color(float r, float g, float b, float a) {
    auto* mid = g_ret_mesh_mid.get_checked(L"MaterialInstanceDynamic");
    if (mid == nullptr) return;

    // Several candidate names, because the parameter BasicShapeMaterial exposes is not something to
    // guess one build at a time. Writing a name the material does not have is harmless -- the MID
    // just stores an override nothing reads -- so trying them all costs one call each and converges
    // in a single run instead of one name per test cycle.
    static const wchar_t* kNames[] = { L"Color", L"BaseColor", L"Tint", L"Albedo", L"DiffuseColor" };
    for (const wchar_t* name : kNames) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        API::FName param = make_fname(name);
        memcpy(p, &param, sizeof(int32_t) * 2);
        auto* c = reinterpret_cast<float*>(p + 8);
        c[0] = r; c[1] = g; c[2] = b; c[3] = a;
        mid->call_function(L"SetVectorParameterValue", p);
    }

    // PROBE THE PARENT MATERIAL, NOT THE MID.
    //
    // A MID stores an override for ANY name you write, and hands it straight back -- so querying the
    // MID cannot tell a real parameter from a typo. The parent Material returns a non-zero DEFAULT
    // only for parameters it actually has, which does discriminate.
    //
    // The function is K2_GetVectorParameterValue -- "GetVectorParameterValue" does not exist, and
    // call_function on a missing name does nothing, so the caller reads back its own zeroed
    // buffer: convincing-looking readings that measure absolutely nothing.
    static bool probed = false;
    if (!probed) {
        probed = true;
        auto* base = API::get()->find_uobject<API::UObject>(
            L"Material /Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial");
        if (base != nullptr) {
            for (const wchar_t* name : kNames) {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                API::FName param = make_fname(name);
                memcpy(p, &param, sizeof(int32_t) * 2);
                base->call_function(L"K2_GetVectorParameterValue", p);
                const auto* out = reinterpret_cast<const float*>(p + 8);
                const bool exists = (out[0] != 0.0f || out[1] != 0.0f || out[2] != 0.0f || out[3] != 0.0f);
                API::get()->log_info("[Halo-CampE-UEVR] BasicShapeMaterial param '%s' default=(%.2f,%.2f,%.2f,%.2f) %s",
                                     narrow(name).c_str(), out[0], out[1], out[2], out[3],
                                     exists ? "<-- EXISTS" : "");
            }
        }
    }
}
bool g_ret_mesh_failed = false;     // latch: never retry creation every tick on a failure

// Bind (or rebind) the reticule material. Separate from component creation and retried from the
// per-tick path because game materials load on demand: a parent like the FX debug arrow may simply
// not be in memory yet when the pawn spawns, and a one-shot bind at creation then leaves the mesh
// on its default material forever.
void reticule_mesh_bind_material(API::UObject* comp) {
    if (comp == nullptr) return;
    if (g_cfg.aim_tex && !g_cfg.aim_mesh_override) {
        API::get()->log_info("[Halo-CampE-UEVR] reticule mesh: keeping the asset's own material (aimmeshmat=0)");
    }
    else if (g_cfg.aim_tex) {
        // Textured quad: a Widget3DPassThrough variant + a real Texture2D in its "SlateUI" parameter.
        std::wstring parent;
        API::UObject* base = nullptr;
        if (g_cfg.aim_mesh_parent[0] == '/') {
            // A full object path selects any material, including one supplied by an added pak --
            // which is loaded on demand here, since nothing in the game references it.
            base = find_or_load_material(std::string(g_cfg.aim_mesh_parent));
        }
        else if (_stricmp(g_cfg.aim_mesh_parent, "translucent") == 0) {
            parent = L"MaterialInstanceConstant /Engine/EngineMaterials/Widget3DPassThrough_Translucent."
                     L"Widget3DPassThrough_Translucent";
        }
        else if (_stricmp(g_cfg.aim_mesh_parent, "opaque") == 0) {
            parent = L"MaterialInstanceConstant /Engine/EngineMaterials/Widget3DPassThrough_Opaque."
                     L"Widget3DPassThrough_Opaque";
        }
        else {
            parent = L"MaterialInstanceConstant /Engine/EngineMaterials/Widget3DPassThrough_Masked."
                     L"Widget3DPassThrough_Masked";
        }
        if (base == nullptr && !parent.empty()) {
            base = API::get()->find_uobject<API::UObject>(parent.c_str());
        }
        API::UObject* mid = nullptr;
        if (base != nullptr) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            *reinterpret_cast<int32_t*>(p) = 0;
            *reinterpret_cast<void**>(p + 8) = base;
            comp->call_function(L"CreateDynamicMaterialInstance", p);
            mid = *reinterpret_cast<API::UObject**>(p + 24);
        }

        API::UObject* tex = nullptr;
        if (mid != nullptr && g_cfg.aim_rt) {
            tex = make_color_rt(g_cfg.aim_mesh_cr, g_cfg.aim_mesh_cg, g_cfg.aim_mesh_cb, 1.0f,
                                g_cfg.aim_rt_size);
            if (tex != nullptr) {
                alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
                API::FName param = make_fname(L"SlateUI");
                memcpy(q, &param, sizeof(int32_t) * 2);
                *reinterpret_cast<void**>(q + 8) = tex;
                mid->call_function(L"SetTextureParameterValue", q);
            }
        }
        else if (mid != nullptr && g_cfg.aim_tex_path[0] != 0) {
            const std::string a{g_cfg.aim_tex_path};
            const std::wstring w(a.begin(), a.end());
            tex = API::get()->find_uobject<API::UObject>(w.c_str());
            if (tex != nullptr) {
                alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
                API::FName param = make_fname(L"SlateUI");
                memcpy(q, &param, sizeof(int32_t) * 2);
                *reinterpret_cast<void**>(q + 8) = tex;
                mid->call_function(L"SetTextureParameterValue", q);
            }

            // THE TINT. Widget3DPassThrough computes colour as SlateUI.rgb * TintColorAndOpacity,
            // and the asset's default tint is (0,0,0,1) -- BLACK. So any MID built from it
            // renders black until the tint is set, whatever texture it has. UWidgetComponent
            // sets texture, tint and opacity together (UpdateMaterialInstanceParameters);
            // setting only the texture produces a black quad.
            {
                alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
                API::FName param = make_fname(L"TintColorAndOpacity");
                memcpy(q, &param, sizeof(int32_t) * 2);
                auto* c = reinterpret_cast<float*>(q + 8);
                c[0] = g_cfg.aim_mesh_cr; c[1] = g_cfg.aim_mesh_cg; c[2] = g_cfg.aim_mesh_cb; c[3] = 1.0f;
                mid->call_function(L"SetVectorParameterValue", q);
            }
            {
                alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
                API::FName param = make_fname(L"OpacityFromTexture");
                memcpy(q, &param, sizeof(int32_t) * 2);
                *reinterpret_cast<float*>(q + 8) = 1.0f;
                mid->call_function(L"SetScalarParameterValue", q);
            }
        }
        // Log whether API::FName actually resolves here: a None name means every FName-taking
        // UFunction (socket lookups, texture params, this tint) receives a nameless argument and
        // ignores it, so the answer matters well beyond the reticule.
        {
            API::FName probe = make_fname(L"TintColorAndOpacity");
            API::get()->log_info("[Halo-CampE-UEVR] FNAME PROBE: comparison_index=%d number=%d to_string='%s'",
                                 probe.comparison_index, probe.number,
                                 narrow(probe.to_string()).c_str());
        }

        g_ret_mesh_mid.set(mid);
        API::get()->log_info("[Halo-CampE-UEVR] textured reticule: mat=%p mid=%p tex=%p ('%s')",
                             (void*)base, (void*)mid, (void*)tex, g_cfg.aim_tex_path);
    }
    else {
        auto* base = API::get()->find_uobject<API::UObject>(
            L"Material /Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial");
        if (base != nullptr) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            *reinterpret_cast<int32_t*>(p) = 0;              // ElementIndex
            *reinterpret_cast<void**>(p + 8) = base;         // SourceMaterial
            comp->call_function(L"CreateDynamicMaterialInstance", p);
            g_ret_mesh_mid.set(*reinterpret_cast<API::UObject**>(p + 24));
        }
        API::get()->log_info("[Halo-CampE-UEVR] reticule mesh material: base=%p mid=%p",
                             (void*)base, (void*)g_ret_mesh_mid.ptr);
    }
}

void reticule_mesh_ensure(API::UObject* rig) {
    if (!g_ret_mesh.empty() || g_ret_mesh_failed || !g_cfg.aim_mesh) return;

    // A component has to belong to an ACTOR. The rig component's outer is that actor.
    auto* owner = (rig != nullptr) ? rig->get_outer() : nullptr;
    if (owner == nullptr) return;   // not fatal yet -- try again next tick

    auto* smc_cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.StaticMeshComponent");
    if (smc_cls == nullptr) {
        g_ret_mesh_failed = true;
        API::get()->log_info("[Halo-CampE-UEVR] reticule mesh: StaticMeshComponent class NOT FOUND");
        return;
    }

    auto* comp = API::get()->add_component_by_class(owner, smc_cls, false);
    if (comp == nullptr) {
        g_ret_mesh_failed = true;
        API::get()->log_info("[Halo-CampE-UEVR] reticule mesh: add_component_by_class FAILED on %s",
                             narrow(class_name_of(owner)).c_str());
        return;
    }

    // Engine primitives, so nothing is taken from the level. Sphere is 100 cm radius, hence the
    // small default scale.
    // Mesh selection. An explicit path wins; otherwise try a TORUS (see-through centre, which is
    // the point of a ring reticule), then fall back to the shapes that certainly exist. Each
    // candidate is logged so a missing torus is visible rather than silently becoming a sphere.
    API::UObject* mesh = nullptr;
    const wchar_t* mesh_used = L"(none)";
    if (g_cfg.aim_mesh_path[0] != 0) {
        const std::string a{g_cfg.aim_mesh_path};
        const std::wstring w(a.begin(), a.end());
        mesh = API::get()->find_uobject<API::UObject>(w.c_str());
        mesh_used = L"(configured)";
    }
    if (mesh == nullptr) {
        static const wchar_t* kMeshes[] = {
            // The intended shipped ring, FIRST. It used to be aim_mesh_path's default in
            // Config.hpp, but a char-array string default on the global g_cfg does not survive
            // MSVC's constant-initialization (see the note there), so it belongs in this
            // candidate list -- which also degrades correctly if a game patch moves the asset.
            L"StaticMesh /Game/FX/Meshes/Primitives/Torus/SM_Torus_ThinDense_01.SM_Torus_ThinDense_01",
            L"StaticMesh /Engine/BasicShapes/Torus.Torus",
            L"StaticMesh /Engine/EngineMeshes/Torus.Torus",
            L"StaticMesh /Engine/EditorMeshes/Torus.Torus",
        };
        for (const wchar_t* m : kMeshes) {
            mesh = API::get()->find_uobject<API::UObject>(m);
            if (mesh != nullptr) { mesh_used = m; break; }
        }
    }
    if (mesh == nullptr) {
        // No torus in this build: a flat quad if we are texturing, otherwise a sphere.
        mesh_used = g_cfg.aim_tex ? L"StaticMesh /Engine/BasicShapes/Plane.Plane"
                                  : L"StaticMesh /Engine/BasicShapes/Sphere.Sphere";
        mesh = API::get()->find_uobject<API::UObject>(mesh_used);
    }
    API::get()->log_info("[Halo-CampE-UEVR] reticule mesh asset: %s -> %s",
                         narrow(mesh_used).c_str(), mesh != nullptr ? "ok" : "MISSING");
    if (mesh == nullptr) {
        mesh = API::get()->find_uobject<API::UObject>(L"StaticMesh /Engine/EngineMeshes/Sphere.Sphere");
    }
    if (mesh != nullptr) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<void**>(p) = mesh;
        comp->call_function(L"SetStaticMesh", p);
    } else {
        API::get()->log_info("[Halo-CampE-UEVR] reticule mesh: no engine sphere found (component will be invisible)");
    }

    // COLOUR WE CONTROL.
    //
    // Widget3DPassThrough (the BASE material behind the widget instances) is not cooked for use
    // on a mesh in this build and renders flat black. BasicShapeMaterial is the default material
    // of the /Engine/BasicShapes meshes, so it is packaged wherever those meshes are, and it
    // exposes a "Color" parameter.
    //
    // This exists because the game's own reticle widget writes ZERO colour into an offscreen render
    // target -- shape, alpha and animation all survive, colour does not. (Established by
    // elimination: an invalid material on this same quad renders WorldGridMaterial in colour, so
    // the quad-to-screen path is fine and the target's RGB really is black.)
    reticule_mesh_bind_material(comp);

    // No collision: a solid sphere sitting on the aim ray would block shots and bump the player.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 0; comp->call_function(L"SetCollisionEnabled", p); }
    // Absolute transform, so the owner actor's own motion does not drag the reticule around.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1; p[1] = 1; p[2] = 1;
      comp->call_function(L"SetAbsolute", p); }
    // No shadow: it is a HUD element, and a floating quad throwing a shadow onto the level reads as
    // a bug immediately. Also keeps it out of the depth/shadow passes entirely.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; comp->call_function(L"SetCastShadow", p); }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1; comp->call_function(L"SetVisibility", p); }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 0; comp->call_function(L"SetHiddenInGame", p); }

    const float s = g_cfg.aim_mesh_scale;
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      auto* d = reinterpret_cast<double*>(p); d[0] = s; d[1] = s; d[2] = s;
      comp->call_function(L"SetWorldScale3D", p); }

    g_ret_mesh.set(comp);
    API::get()->log_info("[Halo-CampE-UEVR] reticule mesh CREATED on %s @%p (mesh=%s)",
                         narrow(class_name_of(owner)).c_str(), (void*)comp,
                         mesh != nullptr ? "ok" : "MISSING");
}

// ---- THE SHARED WORLD-SPACE WIDGET QUAD (see Reticule.hpp for why this is the only copy).
//
// BlendMode has no setter, so the property is written directly. Offset 1428 was verified against
// the live object (OpacityFromTexture reads 1.0f at 1424 immediately before it). ONE definition:
// a second copy of this constant in another file is exactly how a patch breaks one call site and
// leaves the other silently wrong.
constexpr size_t WIDGET_BLENDMODE_OFFSET = 1428;

API::UObject* widget_quad_begin(API::UObject* owner, int blend_mode, bool* out_exposure_compensated) {
    if (out_exposure_compensated != nullptr) *out_exposure_compensated = false;
    if (owner == nullptr) return nullptr;

    auto* wc_cls = API::get()->find_uobject<API::UClass>(L"Class /Script/UMG.WidgetComponent");
    if (wc_cls == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] widget quad: UMG.WidgetComponent class NOT FOUND");
        return nullptr;
    }
    auto* comp = API::get()->add_component_by_class(owner, wc_cls, /*deferred=*/true);
    if (comp == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] widget quad: deferred add_component_by_class FAILED");
        return nullptr;
    }

    *(reinterpret_cast<uint8_t*>(comp) + WIDGET_BLENDMODE_OFFSET) =
        (uint8_t)clampf((float)blend_mode, 0.0f, 2.0f);

    // The MIC itself, not a MID of the parent Material. Preferred: the VR-editor pass-through
    // variant, which multiplies SlateUI by EyeAdaptationInverse -- the stock Widget3D pass is
    // unlit but still multiplied by the scene's PRE-EXPOSURE, so authored colours tonemap to
    // near-black in bright scenes. The optional HaloCEReticleColor LogicMod pak supplies it;
    // without that pak this resolves null and the stock MIC keeps prior behaviour.
    API::UObject* mic = find_or_load_material(kVREditorPassThroughPath);
    const bool compensated = (mic != nullptr);
    if (mic == nullptr) {
        mic = API::get()->find_uobject<API::UObject>(
            L"MaterialInstanceConstant /Engine/EngineMaterials/Widget3DPassThrough_Translucent."
            L"Widget3DPassThrough_Translucent");
    }
    if (mic != nullptr) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<int32_t*>(p) = 0;
        *reinterpret_cast<void**>(p + 8) = mic;
        comp->call_function(L"SetMaterial", p);
    }
    if (out_exposure_compensated != nullptr) *out_exposure_compensated = compensated;

    // Space FIRST: setting the widget before the space can build the render target for the wrong
    // mode. 0 = World, 1 = Screen.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 0; comp->call_function(L"SetWidgetSpace", p); }
    // Two-sided: if the facing maths is ever off by 180 the quad is still visible rather than
    // invisible, which is a much easier failure to diagnose.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1; comp->call_function(L"SetTwoSided", p); }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 0; comp->call_function(L"SetCollisionEnabled", p); }
    // Absolute transform, so the owner actor's own motion does not drag the quad around.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1; p[1] = 1; p[2] = 1;
      comp->call_function(L"SetAbsolute", p); }
    // A dynamically added component inherits tick settings from the CDO, and the redraw that
    // fills the render target happens on tick.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1;
      comp->call_function(L"SetComponentTickEnabled", p); }
    // Widget components only redraw when they think they are visible; a quad off to the side of
    // the view is precisely the case that gets culled.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1;
      comp->call_function(L"SetTickWhenOffscreen", p); }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; comp->call_function(L"SetCastShadow", p); }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1; comp->call_function(L"SetVisibility", p); }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 0; comp->call_function(L"SetHiddenInGame", p); }
    return comp;
}

void widget_quad_finish(API::UObject* owner, API::UObject* comp, float bounds_scale) {
    if (owner == nullptr || comp == nullptr) return;
    // FINISH LAST. Registration happens here, so everything set before -- blend mode, material,
    // widget, draw size -- is already in place when the component builds its render target and
    // scene proxy. FTransform is identity at UE5 double precision.
    {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<void**>(p) = comp;          // Component
        p[8] = 0;                                     // bManualAttachment
        auto* t = reinterpret_cast<double*>(p + 16);
        t[0] = 0.0; t[1] = 0.0; t[2] = 0.0; t[3] = 1.0;   // Rotation (x,y,z,w)
        t[4] = 0.0; t[5] = 0.0; t[6] = 0.0;               // Translation
        t[8] = 1.0; t[9] = 1.0; t[10] = 1.0;              // Scale3D
        owner->call_function(L"FinishAddComponent", p);
    }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      *reinterpret_cast<float*>(p) = bounds_scale;
      comp->call_function(L"SetBoundsScale", p); }
    // Re-assert ABSOLUTE after registration -- the pre-finish SetAbsolute does not survive the
    // attach (field-proven: a marker rode a departing dropship, inheriting its parent's motion
    // between placements).
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1; p[1] = 1; p[2] = 1;
      comp->call_function(L"SetAbsolute", p); }
}

// ---- WIDGET RETICULE
// Hosts the game's own crosshair widget on a world-space UWidgetComponent, so the per-weapon art
// and the hit marker (both children of that widget) come with it.
TrackedObject g_ret_widget_comp;
bool g_ret_widget_failed = false;

// True when the widget quad ended up on a material that cancels pre-exposure (the VREditor
// EyeAdaptationInverse pass). That material preserves the authored colours at unit tint; the stock
// pass needs an emissive gain instead. Set when the material is chosen, consumed by the tint.
bool g_ret_widget_exposure_compensated = false;

// Defined further down; declared here because the scene-hidden latch below drives the hide THROUGH
// it (alpha), rather than writing the component directly.
void apply_widget_tint(uevr::API::UObject* comp, bool force);

// ---- SCENE-HIDDEN LATCH (xrlayerhidews) ------------------------------------------------------
//
// HIDDEN-IN-GAME, NOT INVISIBLE, AND THE DIFFERENCE IS THE ENTIRE POINT.
//
// The compositor reticule (XrLayer/XrSource) presents THIS component's render target. So the
// widget must keep ticking and keep drawing -- that is where the art and the firing/reload
// animation come from. SetVisibility(false) would stop the component updating and freeze the
// layer on whatever frame it last drew, which would look like it worked right up until you fired.
// SetHiddenInGame(false->true) drops only the SCENE PROXY: the widget renders to its target
// exactly as before, it simply is not composited into the world.
//
// Never set from the config directly -- see the call site, which requires the layer to be PROVEN
// live first. Hiding the only crosshair a player has, because a feature silently failed, is the
// one outcome this whole module is built to avoid.
std::atomic<bool> g_ws_scene_hidden{false};
// How many times the reticule's widget component has been (re)bound. A weapon swap, a death or
// an area transition destroys the pawn the component is outered to, so this counts the events
// that hand mode 3 a FRESH, unflagged widget. Reported on every hide/restore transition so the
// two can be correlated in the log instead of in someone's head.
std::atomic<uint32_t> g_ret_rehosts{0};

int read_bitfield_bool(uevr::API::UObject* obj, const wchar_t* name) {
    if (obj == nullptr) return -1;
    auto* cls = obj->get_class();
    if (cls == nullptr) return -1;
    auto* prop = cls->find_property(name);
    if (prop == nullptr) return -1;
    auto* bp   = static_cast<uevr::API::FBoolProperty*>(prop);
    auto* byte = reinterpret_cast<uint8_t*>(obj) + bp->get_offset();
    const uint8_t mask = bp->get_field_mask() ? bp->get_field_mask() : 0xFFu;
    return ((*byte & mask) != 0) ? 1 : 0;
}

// Write one bitfield bool by its FBoolProperty mask. Returns 1/0 as read back, -1 if absent.
// Read-back rather than assumed, because a packed bool written through the wrong mask silently
// lands on a neighbour and reports success.
int set_bitfield_bool(uevr::API::UObject* obj, const wchar_t* name, bool on) {
    if (obj == nullptr) return -1;
    auto* cls = obj->get_class();
    if (cls == nullptr) return -1;
    auto* prop = cls->find_property(name);
    if (prop == nullptr) return -1;
    auto* bp   = static_cast<uevr::API::FBoolProperty*>(prop);
    auto* byte = reinterpret_cast<uint8_t*>(obj) + bp->get_offset();
    const uint8_t mask = bp->get_field_mask() ? bp->get_field_mask() : 0xFFu;
    *byte = on ? (uint8_t)(*byte | mask) : (uint8_t)(*byte & ~mask);
    return ((*byte & mask) != 0) ? 1 : 0;
}

// MODE 3 NEEDS BOTH FLAGS, and bVisibleInSceneCaptureOnly alone is NOT enough.
//
// WHAT MODE 3 ACTUALLY DOES, corrected 2026-08-30 after three in-headset rounds. The earlier text
// here claimed the flag hid the widget until a capture ran; that was inferred from the shape of the
// reports, and the final round disproved it: with vsco set and read back true, the scope INACTIVE,
// and the layer live, the widget was still plainly visible in the main view.
//
// So the honest description is: bVisibleInSceneCaptureOnly does NOT hide a UWidgetComponent from the
// main view at all. What it does do is get the widget drawn into a SceneCapture, which is the half
// we actually need -- it is what puts a reticule in the scope pane.
//
// Mode 3 is therefore "show it in the capture, and accept that it also shows in the main view".
// The double is the PRICE of the pane reticule, not a bug to be fixed by adding more flags -- see
// reticule_set_capture_only for what happened when one was added.
//
// The alpha is restored on un-hide, so switching modes at runtime cannot strand the widget invisible.
// MODE 4's other half. bOwnerNoSee is evaluated against the VIEW's ViewActor: in the main view that
// is the player's view target -- our pawn, which outers this widget -- so the component is skipped;
// a SceneCaptureComponent2D does not set a ViewActor, so the capture still draws it. That is exactly
// the split mode 3 was supposed to provide and measurably does not.
//
// SetOwnerNoSee is on the probed function list for this build (see Arms.cpp), unlike the socket
// rotation calls -- but call_function on an absent UFUNCTION fails SILENTLY here, so the caller
// reads the property back rather than trusting the call. Returns the read-back bit, or -1 if the
// property could not be read at all.
int reticule_set_owner_no_see(uevr::API::UObject* wid, bool on) {
    if (wid == nullptr) return -1;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    p[0] = on ? 1 : 0;
    wid->call_function(L"SetOwnerNoSee", p);
    return read_bitfield_bool(wid, L"bOwnerNoSee");
}

// TELL THE RENDERER THE BIT CHANGED. THIS IS WHY THE DOUBLE RETICULE SURVIVED A CORRECT WRITE.
//
// set_bitfield_bool() writes the UPROPERTY byte directly. That is the right way to READ or POKE a
// value, but bVisibleInSceneCaptureOnly is consumed by the SCENE PROXY, which is built once and
// then cached: a raw memory write marks no render state dirty, so the proxy keeps drawing with
// whatever the flag was when it was created. UPrimitiveComponent's own setters exist precisely to
// pair the write with MarkRenderStateDirty(), and we were doing only half of that.
//
// The result is a bug that reads as impossible from the log: bit=1, want=1, widget live, layer
// live, no rehost -- and the world reticule still drawn in the main view. Measured in a headset
// 2026-09-06 with the RETDEV line: `bit=1 want=1 off=609 mask=0x80 live=1 rehosts=1`, doubling on
// screen the whole time. It also explains every confusing thing about this flag's history:
//   * why it works when applied EARLY -- the proxy is built after the write, so it reads the new
//     value and no refresh is needed;
//   * why DYING AND TAKING A CHECKPOINT RELOAD cured it -- that rebuilds the proxy;
//   * why the 08-30 and 09-06 "vsco does not hide from the main view" measurements were wrong.
//     They were not wrong about the flag. They were reading a stale proxy, and a read-back of the
//     byte agreed with them every time, because the byte WAS set.
//
// MarkRenderStateDirty() is not a UFUNCTION, so it cannot be called through reflection. Toggling
// visibility is: USceneComponent::SetVisibility() marks the render state dirty on change, so
// off-then-on inside one tick destroys and recreates the proxy, which then reads the new flag.
// Both calls land before the frame renders, so there is no visible flicker.
//
// bPropagateToChildren = false, for the same reason Arms.cpp passes false: never move state onto
// anything parented to this component.
//
// CHANGE-GATED BY ITS CALLERS, NOT BY ITSELF. Every reticule_set_capture_only() call site already
// fires only on a transition (the re-assert's `have == want` early-out, and the mode 3/4 handlers),
// so this costs two engine calls a handful of times per level -- never per tick. If a per-tick
// caller is ever added, gate it there; two SetVisibility calls every frame would be a real cost.
void vsco_force_render_refresh(uevr::API::UObject* wid) {
    if (wid == nullptr) return;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    p[0] = 0; p[1] = 0;   // SetVisibility(false, bPropagateToChildren=false)
    wid->call_function(L"SetVisibility", p);
    p[0] = 1; p[1] = 0;   // ...and straight back on, same tick
    wid->call_function(L"SetVisibility", p);
}

void reticule_set_capture_only(uevr::API::UObject* wid, bool on, int* out_vsco, int* out_mainp) {
    const int v = set_bitfield_bool(wid, L"bVisibleInSceneCaptureOnly", on);
    // The write above only changes MEMORY. Without this the scene proxy never learns, and the
    // world reticule keeps drawing in the main view with the bit reading correct. See above.
    if (v >= 0) vsco_force_render_refresh(wid);

    // bRenderInMainPass IS DELIBERATELY NOT WRITTEN. Writing it KILLS THE PANE RETICULE: the
    // SceneCapture honours it, so the one place we want the widget is the one place it disappears
    // from. That half was measured 2026-08-30 and still stands -- do not re-add it.
    //
    // ** CORRECTED 2026-09-06: THE OTHER HALF OF THAT MEASUREMENT WAS WRONG. **
    //
    // This block used to also claim "vsco alone -> widget still visible in the MAIN view (the flag
    // does not govern a UWidgetComponent's main-view draw at all)", and concluded mode 3 was a
    // capture-only HINT whose leak into the main view was an accepted cost. That is false, and it
    // cost two sessions: an agent read it, believed the behaviour was unreachable, and went off to
    // build the compositor scope pane as an alternative route to something mode 3 already did.
    //
    // CONFIRMED TWICE IN A HEADSET, independently, with vsco set and bRenderInMainPass NOT written:
    //   unscoped, main view ............ reticule NOT visible
    //   scoped, main view around scope . NOT visible
    //   scoped, through the scope ...... VISIBLE
    // (log preserved: _Builds\_logs\log.ANOMALY-REPRODUCED-20260906-154556.txt, plus a second
    // confirmation from the scope-pane lane after it restored the re-assert host it had removed.)
    //
    // WHY THE ORIGINAL MEASUREMENT WAS WRONG, because this is the trap and it recurred the same day:
    // A NEGATIVE RESULT ABOUT A FLAG IS ONLY AS GOOD AS THE MACHINERY KEEPING THE FLAG APPLIED.
    // vsco is a single bool written once; anything that rebuilds or re-hosts the widget clears it,
    // and it stays clear until something re-applies it. Both false measurements -- 08-30's and a
    // repeat on 09-06 -- were taken while the re-assert was not running (see the note below: it
    // needs BOTH hosts, and 09-06's repeat had the late one removed). The flag was read back as
    // `true` in both cases, which is exactly what makes this convincing and wrong: the write landed,
    // and something later in the same frame undid it.
    //
    // Corollary worth carrying: the HIDDEN/restored transition log is NOT a proxy for what is on
    // screen. The session that reproduced the working behaviour logged TWO restores, and sessions
    // with hundreds of tick faults logged none.
    if (out_vsco)  *out_vsco  = v;
    if (out_mainp) *out_mainp = -1;   // not written -- never report a value we did not set
}

// MODE 3'S PER-TICK RE-ASSERT. Modes 1 and 2 get this for free -- alpha is re-applied by
// apply_widget_tint and scale by reticule_widget_move, both every tick -- which is why a missed or
// clobbered write repairs itself within a frame. A bool written once does not, so it needs its own
// host, and without one a single missed write is permanent (that is exactly what the first-load
// report was).
//
// Cheap by construction: one property lookup and a masked byte compare, and it WRITES ONLY WHEN THE
// BIT DISAGREES, so the steady state costs a read. Safe to call unconditionally every tick.
// THE vsco BIT'S LAYOUT, cached from a healthy reflection resolve. -1 = not yet known.
int32_t s_vsco_offset = -1;
uint8_t s_vsco_mask   = 0;
// The raw re-assert writes the bit with NO reflection (that is its whole purpose -- it runs while
// reflection is paused after a fault), so it cannot call vsco_force_render_refresh() itself: that
// needs two UFUNCTION calls. It raises this instead, and the reflection-side re-assert spends it on
// the next healthy tick. Without this hand-off a repair made during a fault window would set the
// byte and leave the scene proxy stale -- the exact bug this refresh exists to close, reintroduced
// on the one path that most needs it.
std::atomic<bool> s_vsco_refresh_pending{false};

// RE-APPLY THE HIDE WITHOUT TOUCHING REFLECTION.
//
// WHY THIS EXISTS: after a tick fault the plugin pauses ALL reflection for ~1 s, and a tick that
// faults aborts outright -- both skip every re-assert host inside update(). A widget rebuild in
// either window leaves bVisibleInSceneCaptureOnly clear with nothing to restore it, and the
// world-space reticule reappears in the main view. That is the doubling the scope-pane lane
// correlated with the fault storm.
//
// WHY IT IS SAFE TO RUN WHEN REFLECTION IS NOT: it makes no reflection call. It validates the
// widget through the OBJECT ARRAY (uobject_live -- an index lookup, it never dereferences the
// object) and then does one masked byte read and, only on disagreement, one masked byte write at
// a class-level offset resolved earlier while reflection was healthy.
//
// THE RISK, STATED PLAINLY because it was raised as an objection and accepted deliberately by
// the user: this writes into a UObject during a window in which something ELSE is dereferencing
// null through UEVR's reflection. uobject_live() proves the pointer still occupies its array
// slot, which is the strongest cheap evidence available that the object is alive -- but it is
// not proof the layout is what we cached. If a crash ever lands INSIDE this function, this
// comment is the first place to look, and reverting to the reflection path is the fallback.
void reticule_mode3_reassert_raw() {
    if (g_cfg.xr_layer_hide_ws != 3 && g_cfg.xr_layer_hide_ws != 4) return;
#if HALO_VR_DEV
    // RETDEV state line, ~2 s. THE DOUBLE-RETICULE REPORT NEEDS THIS AND NOTHING ELSE PRINTS IT.
    //
    // The re-assert below writes ONLY when the bit disagrees, so a session in which the bit reads
    // CORRECT and the player still sees the world reticule in the main view produces no log line
    // at all. That is exactly what the 2026-09-06 first-load report looked like from this side:
    // zero re-assert fires, the HIDDEN transition logged once and never flapped, and the player
    // saw the doubling for a whole life anyway. That silence has two very different causes and
    // this line is what separates them:
    //   bit=1 while the doubling is ON SCREEN -> the write LANDED but the scene proxy never picked
    //     it up. A raw masked byte write marks no render state dirty, so the proxy keeps rendering
    //     with the old flag until something else rebuilds it -- which is why dying and taking a
    //     checkpoint reload cleared it. The fix would then be a proxy refresh, not another write.
    //   bit=0 (or off=-1 / wid=null) -> the write never happened, and this says which gate stopped
    //     it: the layout was never resolved, or the widget handle is empty.
    // Costs one masked read every 64 ticks and is absent from a player build.
    {
        static uint32_t s_t = 0;
        if ((s_t++ % 64) == 0) {
            auto* w = g_ret_widget_comp.ptr;
            int bit = -1;
            if (s_vsco_offset >= 0 && w != nullptr) {
                static int32_t s_dbg_idx = -1;
                if (uobject_live(w, &s_dbg_idx))
                    bit = ((*(reinterpret_cast<uint8_t*>(w) + s_vsco_offset)) & s_vsco_mask) != 0;
            }
            uevr::API::get()->log_info(
                "[Halo-CampE-UEVR] RETDEV vsco: bit=%d want=%d off=%d mask=0x%02X wid=%p live=%d "
                "rehosts=%u hidews=%d -- bit==want while the world reticule is still visible means "
                "the scene proxy is stale, not the write.",
                bit, (int)g_ws_scene_hidden.load(std::memory_order_relaxed), (int)s_vsco_offset,
                (unsigned)s_vsco_mask, (void*)w, (int)xrlayer_live(),
                g_ret_rehosts.load(std::memory_order_relaxed), (int)g_cfg.xr_layer_hide_ws);
        }
    }
#endif
    if (s_vsco_offset < 0) return;                 // never resolved; nothing safe to write
    auto* wid = g_ret_widget_comp.ptr;
    if (wid == nullptr) return;
    static int32_t s_idx = -1;
    if (!uobject_live(wid, &s_idx)) return;        // array lookup only -- no dereference

    auto* byte = reinterpret_cast<uint8_t*>(wid) + s_vsco_offset;
    const bool want = g_ws_scene_hidden.load(std::memory_order_relaxed);
    const bool have = (*byte & s_vsco_mask) != 0;
    if (have == want) return;                      // steady state costs one read
    *byte = want ? (uint8_t)(*byte | s_vsco_mask)
                 : (uint8_t)(*byte & (uint8_t)~s_vsco_mask);
    // The byte is right; the SCENE PROXY still is not. Hand the refresh to the reflection side --
    // see s_vsco_refresh_pending. Doing it here would mean UFUNCTION calls during the very window
    // in which reflection is unsafe.
    s_vsco_refresh_pending.store(true, std::memory_order_relaxed);
    static uint32_t s_said = 0;
    if (s_said < 5) {
        ++s_said;
        uevr::API::get()->log_info(
            "[Halo-CampE-UEVR] reticule: vsco re-applied WITHOUT reflection (want=%d) -- the "
            "widget was rebuilt while reflection was paused or a tick had faulted. First %u "
            "only; this is the repair that keeps the world reticule out of the main view.",
            (int)want, 5u);
    }
}

void reticule_mode3_reassert() {
    if (g_cfg.xr_layer_hide_ws != 3 && g_cfg.xr_layer_hide_ws != 4) return;

    // MODE 4's SECOND FLAG NEEDS THE SAME REPAIR AS THE FIRST. bOwnerNoSee is written once per
    // transition, so a widget the game re-hosts comes back with it clear -- the identical hole that
    // made mode 3 flap. Cheap: a masked read, and a write only when the bit disagrees.
    if (g_cfg.xr_layer_hide_ws == 4) {
        if (auto* w = g_ret_widget_comp.get_checked(L"WidgetComponent")) {
            const bool want_ons = g_ws_scene_hidden.load(std::memory_order_relaxed);
            const int  have_ons = read_bitfield_bool(w, L"bOwnerNoSee");
            if (have_ons >= 0 && (have_ons != 0) != want_ons) {
                reticule_set_owner_no_see(w, want_ons);
            }
        }
    }
    auto* wid = g_ret_widget_comp.get_checked(L"WidgetComponent");
    if (wid == nullptr) return;
    // SPEND A REFRESH THE RAW PATH DEFERRED. This must sit ABOVE the `have == want` early-out
    // below: the raw re-assert has already made the byte agree, so that check returns and every
    // line after it is skipped -- which is precisely how a repair made during a fault window would
    // otherwise leave the proxy stale forever.
    if (s_vsco_refresh_pending.exchange(false, std::memory_order_relaxed)) {
        vsco_force_render_refresh(wid);
    }
    auto* cls = wid->get_class();
    if (cls == nullptr) return;
    auto* prop = cls->find_property(L"bVisibleInSceneCaptureOnly");
    if (prop == nullptr) return;
    auto* bp   = static_cast<uevr::API::FBoolProperty*>(prop);
    // CACHE THE LAYOUT WHILE REFLECTION IS HEALTHY. It is class-level, so it is the same for
    // every widget of this class and for the life of the process. reticule_mode3_reassert_raw()
    // below uses it to re-apply the bit with NO reflection at all, which is what lets the repair
    // run while reflection is paused after a fault. Written here rather than in a separate
    // resolve step so it can only ever hold values the normal path has already validated.
    s_vsco_offset = (int32_t)bp->get_offset();
    s_vsco_mask   = bp->get_field_mask() ? bp->get_field_mask() : 0xFFu;
    auto* byte = reinterpret_cast<uint8_t*>(wid) + bp->get_offset();
    const uint8_t mask = s_vsco_mask;
    const bool want = g_ws_scene_hidden.load(std::memory_order_relaxed);
    const bool have = (*byte & mask) != 0;
    // Only vsco is ours now -- bRenderInMainPass is left alone (see reticule_set_capture_only),
    // so requiring it to agree would rewrite the pair every tick and re-break the pane.
    if (have == want) return;
    reticule_set_capture_only(wid, want, nullptr, nullptr);
}

void reticule_widget_set_scene_hidden(bool hidden) {
    if (g_ws_scene_hidden.load(std::memory_order_relaxed) == hidden) return;   // on CHANGE only

    auto* wid = g_ret_widget_comp.get_checked(L"WidgetComponent");
    if (wid == nullptr) return;   // nothing bound yet -- do NOT latch; retry on the next change

    // THE STORE HAPPENS AFTER THE WIDGET CHECK, and that ordering is the whole fix for a bug
    // reported 2026-08-30: "when I first load in, I can see the reticule in main view".
    //
    // It used to store FIRST. At startup the layer goes live BEFORE the reticule widget is bound,
    // so this ran with hidden=true, recorded "hidden", then bailed at the null widget having written
    // nothing. Every later call then saw no CHANGE and returned immediately -- the state said hidden
    // while the widget had never been touched, and it stayed visible until something forced a
    // second transition (dying and respawning, which re-hosts the widget and toggles live off/on --
    // exactly what the report describes as fixing it).
    //
    // Modes 1 and 2 survived this because their value is re-asserted every tick by another host
    // (apply_widget_tint for alpha, reticule_widget_move for scale), so the missed write was
    // repaired within a frame. Mode 3 writes a bool ONCE and had no such host, which is what made a
    // latent ordering bug into a visible one. Mode 3 now re-asserts too -- see
    // reticule_mode3_reassert() -- but the ordering is fixed here as well, because a state flag that
    // records an action that did not happen is wrong for every mode, cured or not.
    g_ws_scene_hidden.store(hidden, std::memory_order_relaxed);

    // ---- HIDE BY ALPHA, NOT BY VISIBILITY -----------------------------------------------
    //
    // SetHiddenInGame(true) FREEZES the compositor layer, measured in a headset twice. The widget
    // stops redrawing its render target once the engine stops rendering the component, so the layer
    // -- which presents that target -- holds whatever frame it last drew.
    //
    // TickWhenOffscreen is NOT the gate, and believing it was cost a build: this component has had
    // it set since creation (see SetTickWhenOffscreen in widget_quad_begin), so the "fix" that set
    // it here was a no-op and the freeze was unchanged. Whatever UWidgetComponent actually keys its
    // redraw on, being hidden defeats it even with that flag on. Do not re-litigate this by
    // reasoning about engine internals we cannot read -- it has now been tested.
    //
    // So do not hide it at all. Leave the component fully visible and RENDERED, and multiply its
    // colour to zero. Every visibility- or render-time-based gate stays satisfied because as far as
    // the engine is concerned nothing changed; the quad simply contributes no pixels.
    //
    // The cost is honest and small: one draw call and a translucency pass for a 256x256 quad that
    // outputs nothing. That is the price of the widget continuing to animate, which is the entire
    // reason the layer has real art.
    //
    // The alpha is applied by apply_widget_tint(), which already re-asserts itself whenever the
    // component rebuilds its material behind our back. Piggy-backing on that is what makes the hide
    // survive a SetDrawSize; a one-shot write here would silently revert exactly as the tint did.
    // Mode 1 drives alpha through apply_widget_tint; mode 2 drives scale in reticule_widget_move.
    // Both are re-asserted every tick by their host, which is what makes either survive the
    // component rebuilding its material or transform behind our back.
    // ---- MODE 3: HIDE FROM THE MAIN VIEW ONLY, STAY VISIBLE TO SCENE CAPTURES ----------------
    //
    // WHY THIS MODE EXISTS. Modes 1 and 2 hide the in-scene reticule from EVERYTHING that renders
    // the world -- including the weapon scope's SceneCaptureComponent2D. So with the compositor
    // reticule on, the zoom pane lost its crosshair, and it could not simply be given the layer's
    // one: a composition layer is submitted at xrEndFrame, AFTER the engine has finished the frame,
    // so a capture that runs inside the engine can never see it. The two layers do not compose.
    //
    // bVisibleInSceneCaptureOnly is UPrimitiveComponent's own answer to exactly this: the component
    // is skipped in the main pass and drawn in scene captures. The main view therefore keeps the
    // compositor reticule alone (no doubling, and the emissive-bloom win is preserved), while the
    // scope pane gets the REAL world-space reticule at the REAL traced hit point -- which is the
    // part a reticule drawn at the pane's centre could not honestly promise, because centre only
    // equals impact if the capture is perfectly aim-aligned.
    //
    // IT IS A PACKED BITFIELD. Writing the byte would clobber every neighbouring flag in the same
    // word (bHiddenInSceneCapture and bRenderInMainPass live there too), so the FBoolProperty's own
    // field mask does the work -- the same way Scope.cpp sets the bOverride_ flags.
    //
    // THE RISK THIS MODE IS ON TRIAL FOR, stated so the next reader does not have to rediscover it:
    // SetHiddenInGame froze the widget's render target, which is why modes 1 and 2 exist at all. If
    // being skipped in the main pass defeats whatever UWidgetComponent keys its redraw on, the
    // layer's source goes stale and this mode is strictly worse than mode 1. That is a MEASUREMENT,
    // not an argument: watch the layer's held= age and whether the art still swaps on weapon change.
    // MODE 4: mode 3's capture-only hint PLUS bOwnerNoSee, which is the flag that actually removes
    // the widget from the main view. Kept as a separate mode rather than folded into 3 so the two can
    // be compared in a headset without a rebuild, and so a build where SetOwnerNoSee turns out to be
    // absent degrades to exactly mode 3's behaviour rather than to nothing.
    if (g_cfg.xr_layer_hide_ws == 4) {
        int vsco = -1, mainp = -1;
        reticule_set_capture_only(wid, hidden, &vsco, &mainp);
        const int ons = reticule_set_owner_no_see(wid, hidden);
        uevr::API::get()->log_info(
            "[Halo-CampE-UEVR] reticule: in-scene widget %s the main pass (mode 4: "
            "bVisibleInSceneCaptureOnly=%s bOwnerNoSee=%s, read back). bOwnerNoSee is the half that "
            "hides it from the MAIN VIEW; the scene capture has no ViewActor so the scope pane still "
            "sees it. %s",
            hidden ? "HIDDEN from" : "restored to",
            vsco < 0 ? "ABSENT" : (vsco ? "true" : "false"),
            ons  < 0 ? "ABSENT[SetOwnerNoSee did not resolve -- this is mode 3 behaviour]"
                     : (ons ? "true" : "false"),
            (ons < 0) ? "Falling back to mode 3's effect." : "");
    } else if (g_cfg.xr_layer_hide_ws == 3) {
        int vsco = -1, mainp = -1;
        reticule_set_capture_only(wid, hidden, &vsco, &mainp);
        // Mode 3 does NOT touch alpha or scale, so restore whatever those were: switching modes at
        // runtime must not leave the widget both flagged AND alpha-zeroed.
        apply_widget_tint(wid, /*force=*/true);
        // WHY THIS TRANSITION HAPPENED, not just that it did.
        //
        // g_ws_scene_hidden follows (hide_ws != 0 && xrlayer_live()), so a transition BACK to
        // visible means the LAYER dropped out of live -- there is no other input. Printing live
        // and the re-host count separates the two candidate causes of "the world-space reticule
        // is showing in my main view": a layer that keeps dropping, versus a widget re-host that
        // outran the re-assert. One failing session logged 17 of these against a working
        // session's 5, and nothing recorded which kind they were.
        //
        // bRenderInMainPass reads ABSENT ON PURPOSE -- reticule_set_capture_only deliberately
        // does not write it (writing it made the SCOPE pane lose its reticule). The old text
        // here claimed "BOTH are required", which is wrong and cost a session: it reads as a
        // failure to apply half the fix when it is the fix working as designed.
        uevr::API::get()->log_info(
            "[Halo-CampE-UEVR] reticule: in-scene widget %s (mode 3: "
            "bVisibleInSceneCaptureOnly=%s bRenderInMainPass=%s[not written by design], "
            "read back) | rehosts=%u -- in mode 3 `restored` IS xrlayer_live() going false, "
            "since hidden = (hide_ws != 0 && live) and hide_ws is 3 here. The SCOPE CAPTURE "
            "still sees the widget either way.",
            hidden ? "HIDDEN from the main pass" : "restored to the main pass",
            vsco  < 0 ? "ABSENT" : (vsco  ? "true" : "false"),
            mainp < 0 ? "ABSENT" : (mainp ? "true" : "false"),
            g_ret_rehosts.load(std::memory_order_relaxed));
        return;
    }

    apply_widget_tint(wid, /*force=*/true);
    uevr::API::get()->log_info("[Halo-CampE-UEVR] reticule: in-scene widget %s (alpha %s; component "
                               "stays rendered so it keeps redrawing its target for the layer)",
                               hidden ? "HIDDEN" : "restored",
                               hidden ? (g_cfg.xr_layer_hide_ws == 2 ? "mode 2: sub-pixel scale"
                                                                     : "mode 1: alpha -> 0")
                                      : "restored");
}

// Mirrors the early-out in reticule_widget_ensure() below EXACTLY -- if that gate changes, change
// this with it, or the scan feeding it will stop while it is still waiting for a widget.
bool reticle_widget_needs_pick() {
    return g_cfg.aim_widget && g_ret_widget_comp.empty() && !g_ret_widget_failed;
}

// Which widget class the reticule hosts. Normally the first-person reticle; aimwidgetclass
// overrides it for the colour diagnostic. This has to be consulted by BOTH the object-array scan
// that collects candidates and the picker that chooses among them -- if only the picker consults
// it, the override silently does nothing and falls back to the reticle.
std::wstring wanted_widget_class() {
    if (g_cfg.aim_widget_class[0] != 0) {
        const std::string a{g_cfg.aim_widget_class};
        return std::wstring(a.begin(), a.end());
    }
    return L"WBP_FirstPersonReticle";
}

// A CONSTRUCTED widget, as opposed to the class ARCHETYPE that lives inside the loaded blueprint
// package under <Class>.WidgetTree.<Name>. Live instances are outered somewhere under
// /Engine/Transient; the archetype never is.
//
// Both consumers of the reticle sweep need this test, for opposite reasons: pick_live_reticle()
// must never BIND the archetype (it renders nothing), and reticle_collapse_strays() must never
// TOUCH it (see there -- that one is destructive and it has already cost a session).
bool is_live_widget_instance(API::UObject* o) {
    for (API::UObject* p = o; p != nullptr; p = p->get_outer()) {
        const auto* fn = p->get_fname();
        if (fn != nullptr && fn->to_string().find(L"Transient") != std::wstring::npos) return true;
    }
    return false;
}

// ---- STRAY NATIVE RETICLES
//
// Hosting hides the game's crosshair by REMOVING IT FROM ITS PARENT, which is not the same as
// hiding the crosshair CLASS. Rebuild the HUD -- finishing a mission and loading the next one does
// exactly that -- and the game builds a fresh crosshair on the flat HUD while we are still hosting
// the old one. Both are then on screen: ours in the world, theirs pasted over the view.
//
// So the flat one is collapsed EXPLICITLY, which also makes the behaviour survive whatever the HUD
// does next rather than depending on our removal having been the only copy.
uint32_t g_ret_stray_until = 0;

void reticle_arm_stray_check() { g_ret_stray_until = 0xFFFFFFFFu; }   // resolved on the next scan

bool reticle_stray_check_due(uint32_t tick) {
    if (g_ret_stray_until == 0) return false;
    // First call after arming: open a short window measured from NOW, so the scan's own 120-tick
    // throttle gets a chance to fire inside it.
    if (g_ret_stray_until == 0xFFFFFFFFu) g_ret_stray_until = tick + 400;   // ~12 s
    if (tick >= g_ret_stray_until) { g_ret_stray_until = 0; return false; }
    return true;
}

// The widget we are currently hosting, and the panel we took it from.
//
// Tracked because BOTH can die independently of our component. The hosted widget dies whenever the
// HUD is rebuilt, which does not require the pawn (and therefore our component) to be destroyed --
// a checkpoint reload does exactly that. A component still bound to the dead widget renders a
// frozen quad forever while the game's new crosshair sits back on the flat HUD.
TrackedObject g_ret_hosted_widget;
TrackedObject g_ret_widget_parent;

// Collapse every scanned reticle widget that is not the one we host. ESlateVisibility::Collapsed
// is 1. Called from the end of the scan, so it costs nothing of its own -- it reuses the list the
// sweep just built rather than looking again.
void reticle_collapse_strays() {
    if (!g_cfg.aim_hide_native) return;
    auto* mine = g_ret_hosted_widget.get();

    // FAIL CLOSED WHEN OUR OWN WIDGET CANNOT BE IDENTIFIED.
    //
    // The widget we host IS one of the game's crosshairs -- taken off the HUD, not a copy -- so the
    // sweep finds it like any other, and this identity check is the only thing keeping it visible.
    // If the tracked handle cannot resolve (recycled slot, mid-rebuild) then `mine` is null, every
    // match looks like a stray, and we would collapse our own world-space reticule.
    //
    // So: while the widget reticule is in use, no identifiable hosted widget means no collapsing at
    // all. With aim_widget off there is nothing of ours among them and every match is genuinely
    // the game's.
    if (g_cfg.aim_widget && mine == nullptr) return;

    for (int i = 0; i < g_reticle_count; ++i) {
        auto* w = g_reticles[i].obj.get();
        if (w == nullptr || w == mine) continue;

        // LIVE INSTANCES ONLY -- NEVER THE CLASS ARCHETYPE.
        //
        // The sweep collects both, and the archetype is not a stray crosshair: it is the template
        // every future HUD is duplicated from. Removing it from its parent does not hide anything
        // that is on screen; it deletes the crosshair from WBP_HUD_Main_C for the rest of the
        // process, so the next HUD the game builds has no reticle at all -- no flat one for the
        // player and nothing for us to host. Because the archetype belongs to the loaded package
        // it also never dies on its own, so this loop found it and "removed" it once per scan.
        //
        // Measured 2026-08-09: four removals at 15:50:59-15:51:13 against the same address, the
        // archetype gone from the sweep by 15:52:55, and the level load at 15:57:40 produced a HUD
        // with every other child present and no FirstPersonReticle. Diagnosed as eye adaptation
        // and as a hosting failure before the log made the sequence plain.
        if (!is_live_widget_instance(w)) continue;

        // REMOVE IT, do not merely hide it.
        //
        // The first version set Visibility=Collapsed and lost: the HUD re-asserts visibility on its
        // own schedule, so the same two widgets were re-collapsed every scan and were visible again
        // in between. Removing from the parent is the lever that actually works -- it is what
        // hosting does to the widget we take, and that one has never come back.
        //
        // Visibility is still set first, so a widget whose removal fails for any reason is at least
        // hidden for the moment rather than left fully visible.
        {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            p[0] = 1;   // ESlateVisibility::Collapsed
            w->call_function(L"SetVisibility", p);
        }
        {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            w->call_function(L"RemoveFromParent", p);
        }
        API::get()->log_info("[Halo-CampE-UEVR] reticule: removed a stray native crosshair @%p "
                             "(the HUD rebuilt one behind us)", (void*)w);
    }
}

// True once we have taken a widget off the HUD, so reticule_widget_release() knows there is
// something to give back. Latches false again when it does.
bool g_ret_widget_hosted_once = false;

// Set while a re-bind is waiting on a fresh sweep, so the sweep is forced ONCE per episode rather
// than every tick. File scope rather than a function-local static so the teardown path can clear it.
bool g_ret_rebind_forced = false;

// Drop every collected candidate and let the next tick re-sweep.
//
// The sweep in reticle_rescan() is throttled to 120 ticks and is a FULL object-array walk, so this
// must be called once per event, never per tick -- forcing it every tick turns a ~4 s sweep cadence
// into a per-frame sweep, which is the pattern that has collapsed framerate here before.
void force_reticle_rescan() {
    g_reticle_count = 0;
    // Unsigned wrap is intentional and correct: rescan's gate is `tick - g_reticle_scan_tick < 120`,
    // and subtracting the interval makes that difference >= 120 for any value of tick.
    g_reticle_scan_tick -= 120;
}

// Prefer the CONSTRUCTED instance over the class template. The template lives under
// <Class>.WidgetTree.FirstPersonReticle and is not what renders; the live one is under
// /Engine/Transient. Handing the template to a WidgetComponent hosts a widget nothing drives, so no
// hit markers would ever fire.
//
// THE TEMPLATE IS NEVER AN ACCEPTABLE ANSWER. This used to return it as a fallback when no transient
// instance was found, which produced the reticule's worst failure mode. The two objects have very
// different lifetimes: the transient instance is destroyed on every level and checkpoint load, while
// the template belongs to the loaded HUD blueprint package and SURVIVES. So immediately after a
// transition the stale candidate list holds a dead transient entry and a still-valid template -- and
// the fallback bound the template every time, giving an unrecognisable reticule that nothing drives
// while the real crosshair stayed flat on the HUD. Returning null instead simply means "not yet",
// which the caller already handles by retrying next tick.
//
// The found_tick test is the other half of that fix: only candidates from the MOST RECENT sweep are
// eligible. reticule_widget_ensure() runs every tick but the sweep runs at most every 120, so
// without this a stale list is what gets picked from for up to ~4 seconds after every load.
API::UObject* pick_live_reticle() {
    for (int i = 0; i < g_reticle_count; ++i) {
        if (g_reticles[i].found_tick != g_reticle_scan_tick) continue;   // stale sweep -- ignore
        auto* o = g_reticles[i].obj.get();
        if (o == nullptr) continue;
        if (class_name_of(o).find(wanted_widget_class()) == std::wstring::npos) continue;

        if (is_live_widget_instance(o)) return o;
    }
    return nullptr;
}

// Move a widget onto our component: take it off the HUD, clear any HUD-follow offset, hand it over.
// Shared by the initial bind and the re-bind that follows a HUD rebuild, so the two cannot drift.
//
// NOTE this is the HOSTING path, which takes the game's widget off the HUD -- and that is what
// zeroes its colour (measured 2026-08-09: hosted renders a solid BLACK crosshair with correct alpha,
// the same widget un-hosted renders bright cyan). The crosshair is drawn by a MaterialInstanceDynamic
// the HUD drives every frame; off the HUD nothing drives it. A "mirror" path that left the game's
// widget on the HUD and drew a second instance of the same class was tried on 2026-08-09 and did
// not work; the colour is fixed instead by aimwidgetgain/aimwidgettint in the profile.
void host_widget(API::UObject* comp, API::UObject* w) {
    // Remember where it came from, so releasing the feature can put it back (see
    // reticule_widget_release). Captured BEFORE RemoveFromParent, which is what clears it.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      w->call_function(L"GetParent", p);
      g_ret_widget_parent.set(*reinterpret_cast<API::UObject**>(p)); }

    // DETACH FIRST. The widget is still a child of the HUD's WidgetTree, and a widget that already
    // has a parent will not render through a WidgetComponent -- the component hosts the right
    // widget and nothing appears. uevrlib's equivalent has a removeFromViewport step for the same
    // reason.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      w->call_function(L"RemoveFromParent", p); }

    // RESET THE RENDER TRANSLATION. The HUD-follow path may have been driving this same widget to
    // offsets of several hundred pixels to chase the aim across the flat HUD. Inside a draw-size
    // render target that lands far outside the canvas, so the widget renders correctly and is
    // simply not on its own surface. Hosting it makes that offset not merely redundant but harmful.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      auto* d = reinterpret_cast<double*>(p); d[0] = 0.0; d[1] = 0.0;
      w->call_function(L"SetRenderTranslation", p); }

    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      *reinterpret_cast<void**>(p) = w;
      comp->call_function(L"SetWidget", p); }

    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      comp->call_function(L"RequestRedraw", p); }

    g_ret_hosted_widget.set(w);
    g_ret_widget_hosted_once = true;

    // A HUD rebuild can produce the new crosshair AFTER we have already picked, so one more sweep
    // shortly from now catches the straggler. Deliberately a ONE-SHOT window and not a standing
    // poll: that sweep costs 100-125 ms on the game thread, and running it periodically is the
    // periodic microstutter this codebase already had to remove once.
    reticle_arm_stray_check();
}

void reticule_widget_ensure(API::UObject* rig) {
    if (!g_ret_widget_comp.empty() || g_ret_widget_failed || !g_cfg.aim_widget) return;

    auto* w = pick_live_reticle();
    if (w == nullptr) return;   // HUD not built yet -- retry next tick, not a failure

    // CONTROL TEST. A UUserWidget owns exactly ONE underlying SWidget. Ours is a nested child of the
    // HUD's widget tree, not a viewport-level widget like uevrlib's usual subjects -- so if the HUD
    // still holds that SWidget, our component may never get a displayable one, and every property
    // on the component can be perfect while nothing draws.
    //
    // Creating a FRESH instance of the same class separates the two cases outright: if a fresh one
    // renders and the borrowed one does not, the problem is re-parenting, not the component. It
    // costs the hit marker (nothing drives a widget the HUD does not own), so it is a diagnostic,
    // not the destination.
    if (g_cfg.aim_widget_fresh) {
        auto* wbl = API::get()->find_uobject<API::UClass>(L"Class /Script/UMG.WidgetBlueprintLibrary");
        auto* cls = (w != nullptr) ? w->get_class() : nullptr;
        auto* pc  = API::get()->get_player_controller(0);
        if (wbl != nullptr && cls != nullptr) {
            if (auto* cdo = wbl->get_class_default_object()) {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                *reinterpret_cast<void**>(p +  0) = pc;    // WorldContextObject
                *reinterpret_cast<void**>(p +  8) = cls;   // WidgetType
                *reinterpret_cast<void**>(p + 16) = pc;    // OwningPlayer
                cdo->call_function(L"Create", p);
                if (auto* fresh = *reinterpret_cast<API::UObject**>(p + 24)) {
                    API::get()->log_info("[Halo-CampE-UEVR] widget reticule: using FRESH instance @%p", (void*)fresh);
                    w = fresh;
                } else {
                    API::get()->log_info("[Halo-CampE-UEVR] widget reticule: fresh Create() returned null, using live widget");
                }
            }
        }
    }

    auto* owner = (rig != nullptr) ? rig->get_outer() : nullptr;
    if (owner == nullptr) return;

    auto* wc_cls = API::get()->find_uobject<API::UClass>(L"Class /Script/UMG.WidgetComponent");
    if (wc_cls == nullptr) {
        g_ret_widget_failed = true;
        API::get()->log_info("[Halo-CampE-UEVR] widget reticule: UMG.WidgetComponent class NOT FOUND");
        return;
    }

    // ---- DEFERRED CONSTRUCTION, configure, THEN finish -- via widget_quad_begin(), the ONE
    // copy of the recipe (its banner above documents the ordering, the credit to Pande's
    // OblivionVR, the measured BlendMode offset and the exposure-compensated material chain).
    // The navpoint markers build the identical object; keeping a private copy here is what put
    // that measured offset in two files.
    auto* comp = widget_quad_begin(owner, g_cfg.aim_widget_blend, &g_ret_widget_exposure_compensated);
    if (comp == nullptr) {
        g_ret_widget_failed = true;
        return;
    }
    API::get()->log_info("[Halo-CampE-UEVR] widget reticule material: %s (gain %s)",
                         g_ret_widget_exposure_compensated ? "VREditor pass-through"
                                                           : "stock Widget3DPassThrough",
                         g_ret_widget_exposure_compensated ? "forced 1.0" : "aimwidgetgain");

    // FVector2D is double under UE5 LWC.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      auto* d = reinterpret_cast<double*>(p);
      d[0] = g_cfg.aim_widget_draw; d[1] = g_cfg.aim_widget_draw;
      comp->call_function(L"SetDrawSize", p); }

    // (two-sided / no-collision / absolute / tick / offscreen-tick / no-shadow / visible are all
    // set by widget_quad_begin -- the shared recipe. Only the reticule-specific parts remain.)
    const float s = g_cfg.aim_widget_scale;
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      auto* d = reinterpret_cast<double*>(p); d[0] = s; d[1] = s; d[2] = s;
      comp->call_function(L"SetWorldScale3D", p); }

    host_widget(comp, w);

    // ---- TRANSLUCENT MATERIAL, BOUND TO THE COMPONENT'S OWN RENDER TARGET.
    // Replaces the Masked material the component built for itself. CreateDynamicMaterialInstance
    // also assigns the result to the component, so this is the assignment as well as the creation.
    //
    // DEFERRED, NOT DONE HERE. The render target is allocated LAZILY on the component's first
    // tick (null immediately after creation, even though the material instance already exists),
    // so an inline swap silently skips and leaves the Masked material in place.
    // reticule_widget_finish() below retries until the target exists.
    if (false) {
        auto* src = API::get()->find_uobject<API::UObject>(
            L"Material /Engine/EngineMaterials/Widget3DPassThrough.Widget3DPassThrough");
        API::UObject* rt = nullptr;
        { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
          comp->call_function(L"GetRenderTarget", p);
          rt = *reinterpret_cast<API::UObject**>(p); }

        if (src != nullptr && rt != nullptr) {
            API::UObject* mid = nullptr;
            { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
              *reinterpret_cast<int32_t*>(p) = 0;              // ElementIndex
              *reinterpret_cast<void**>(p + 8) = src;          // SourceMaterial
              // p+16 is the optional FName, left as NAME_None
              comp->call_function(L"CreateDynamicMaterialInstance", p);
              mid = *reinterpret_cast<API::UObject**>(p + 24); }

            if (mid != nullptr) {
                // The widget material samples the render target through this parameter.
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                API::FName param = make_fname(L"SlateUI");
                memcpy(p, &param, sizeof(int32_t) * 2);
                *reinterpret_cast<void**>(p + 8) = rt;
                mid->call_function(L"SetTextureParameterValue", p);
            }
            API::get()->log_info("[Halo-CampE-UEVR] widget reticule: translucent MID %s (rt=%p)",
                                 mid != nullptr ? "created" : "FAILED", (void*)rt);
        } else {
            API::get()->log_info("[Halo-CampE-UEVR] widget reticule: material swap skipped (src=%p rt=%p)",
                                 (void*)src, (void*)rt);
        }
    }

    // Diagnostic background -- see aim_widget_bg.
    if (g_cfg.aim_widget_bg) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        auto* c = reinterpret_cast<float*>(p);
        c[0] = 1.0f; c[1] = 0.0f; c[2] = 0.0f; c[3] = 1.0f;   // opaque red
        comp->call_function(L"SetBackgroundColor", p);
    }

    // FINISH LAST -- registration, plus the anti-cull bounds and the absolute re-assert, all in
    // widget_quad_finish() (the shared recipe). Everything above is in place before the render
    // target and scene proxy are built, which is the whole point of the deferred construction.
    widget_quad_finish(owner, comp, /*bounds_scale=*/10.0f);

    // Scale AFTER finishing. FinishAddComponent takes a RelativeTransform and applies it, so the
    // identity transform passed above overwrites any scale set before it.
    {
        const float sc = g_cfg.aim_widget_scale;
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        auto* d = reinterpret_cast<double*>(p); d[0] = sc; d[1] = sc; d[2] = sc;
        comp->call_function(L"SetWorldScale3D", p);
    }

    g_ret_widget_comp.set(comp);
    g_ret_rehosts.fetch_add(1, std::memory_order_relaxed);

    // READ BACK WHAT ACTUALLY TOOK, from the component's own memory at offsets taken from the live
    // type schema. CurrentDrawSize is the one that matters most: DrawSize is the REQUEST,
    // CurrentDrawSize is what the scene proxy builds its quad from, and a zero there is a
    // degenerate quad -- invisible no matter how correct the material, transform and render
    // target are.
    {
        auto* base = reinterpret_cast<uint8_t*>(comp);
        const int32_t cur_x = *reinterpret_cast<int32_t*>(base + 1344);
        const int32_t cur_y = *reinterpret_cast<int32_t*>(base + 1348);
        auto* rt_p   = *reinterpret_cast<void**>(base + 1488);
        auto* mat_p  = *reinterpret_cast<void**>(base + 1496);
        auto* wid_p  = *reinterpret_cast<void**>(base + 1584);
        const uint8_t space = *(base + 1304);
        const uint8_t blend = *(base + 1428);

        // FULL name, not the class name. The class name is IDENTICAL for the live instance and the
        // non-rendering class template, so it could never distinguish the two -- which is why the
        // template-binding bug was invisible in the log for as long as it was. The full name says
        // which one outright: "/Engine/Transient..." is the live one.
        API::get()->log_info(
            "[Halo-CampE-UEVR] widget reticule CREATED @%p hosting %s | space=%u blend=%u "
            "curDraw=(%d,%d) rt=%p mat=%p widget=%p",
            (void*)comp, narrow(w->get_full_name()).c_str(), (unsigned)space, (unsigned)blend,
            cur_x, cur_y, rt_p, mat_p, wid_p);
    }
}

// Everything that can only be done AFTER the component has ticked at least once and allocated its
// render target. Retried every tick until it succeeds, then latched.
bool g_ret_widget_finished = false;

// EMISSIVE GAIN -- why the hosted crosshair needs one.
//
// Bind the component's live render target into its material's SlateUI parameter, and report whether
// the material it is actually rendering with cancels pre-exposure by itself.
//
// WHY THIS IS NOT AUTOMATIC. UWidgetComponent binds SlateUI in UpdateMaterialInstanceParameters --
// but only onto the material instance IT built, at the moments IT expects. We replace the material
// after construction, on a component added dynamically to a pawn the game never expected to carry
// one, and SetDrawSize can reallocate both the target and the instance underneath us. The binding
// is therefore free to end up pointing at a stale target or at nothing at all.
//
// An unbound SlateUI is NOT a cosmetic problem. Sampling it faults inside the translucency pass:
// measured 2026-08-02, EXCEPTION_ACCESS_VIOLATION reading a small offset in
// ParallelDraw -> RenderTranslucency, within 6-13 s of the material being applied, 4/4 runs across
// three different materials. Every one of those runs had the material swapped and SlateUI never
// bound. That is why the exposure-compensated material "could not be shipped" -- it was never the
// material's fault.
//
// Writes only on mismatch, so the steady state costs two reads and no engine writes.
//
// CREDIT: elliotttate's HaloCampaignEvolved-UEVR diagnosed this and repairs it the same way --
// including the parent-chain walk below, which is authoritative where a name check on the material
// we ASKED for is not: the component may be rendering something else entirely.
bool bind_widget_slate_ui(API::UObject* comp) {
    if (comp == nullptr) return false;

    API::UObject* rt = nullptr;
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      comp->call_function(L"GetRenderTarget", p);
      rt = *reinterpret_cast<API::UObject**>(p); }
    if (rt == nullptr) return false;          // allocated lazily on the first tick

    API::UObject* mi = nullptr;
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      comp->call_function(L"GetMaterialInstance", p);
      mi = *reinterpret_cast<API::UObject**>(p); }
    if (mi == nullptr) return false;

    // Walk up the instance chain to whatever base material is really in play.
    bool compensated = false;
    {
        API::UObject* m = mi;
        for (int depth = 0; m != nullptr && depth < 8; ++depth) {
            if (m->get_full_name().find(L"WidgetVRPassThrough") != std::wstring::npos) {
                compensated = true;
                break;
            }
            auto* parent = m->get_property_data<API::UObject*>(L"Parent");
            m = (parent == nullptr) ? nullptr : *parent;
        }
    }
    if (compensated != g_ret_widget_exposure_compensated) {
        g_ret_widget_exposure_compensated = compensated;
        API::get()->log_info("[Halo-CampE-UEVR] widget reticule material chain: %s",
                             compensated ? "exposure-compensated (EyeAdaptationInverse) -- gain forced 1.0"
                                         : "stock pass -- compensating with aimwidgetgain");
    }

    // Already pointing at this target? Then there is nothing to do.
    API::UObject* current = nullptr;
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      API::FName param = make_fname(L"SlateUI");
      memcpy(p, &param, sizeof(int32_t) * 2);
      mi->call_function(L"K2_GetTextureParameterValue", p);
      current = *reinterpret_cast<API::UObject**>(p + 8); }
    // One-shot proof that this ran at all. Without it a silent early-out and a correct binding are
    // indistinguishable in the log, and "the fix did nothing" and "the fix had nothing to do" are
    // very different answers.
    // Read TintColorAndOpacity back off the MID, not off the component.
    //
    // These are NOT the same value. We set the COMPONENT property and verify that, but the shader
    // samples the MID's parameter, and they only agree if UpdateMaterialInstanceParameters actually
    // propagated. The render target provably contains Halo's cyan (measured 2026-08-09:
    // R80 G191 B210 A246), so SlateUI.rgb is not the zero in SlateUI.rgb * TintColorAndOpacity --
    // which leaves the tint on the MID as the candidate. A zero here multiplies correct colour to
    // black and is immune to both gain and exposure, which is every symptom we have.
    {
        static uint32_t t = 0;
        if ((t++ % 32) == 0) {
            float mid_tint[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
            auto* getv = mi->get_class()->find_function(L"K2_GetVectorParameterValue");
            if (getv != nullptr) {
                alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
                API::FName pn = make_fname(L"TintColorAndOpacity");
                memcpy(q, &pn, sizeof(int32_t) * 2);
                mi->process_event(getv, q);
                memcpy(mid_tint, q + 8, sizeof(mid_tint));
            }
            auto* comp_tint = comp->get_property_data<float>(L"TintColorAndOpacity");
            API::get()->log_info("[Halo-CampE-UEVR] TINT COMPARE  component=(%.2f %.2f %.2f %.2f)  "
                                 "MID=(%.2f %.2f %.2f %.2f)  mid=%p rt=%p SlateUI=%s",
                                 comp_tint ? comp_tint[0] : -1.0f, comp_tint ? comp_tint[1] : -1.0f,
                                 comp_tint ? comp_tint[2] : -1.0f, comp_tint ? comp_tint[3] : -1.0f,
                                 mid_tint[0], mid_tint[1], mid_tint[2], mid_tint[3],
                                 (void*)mi, (void*)rt, current == rt ? "bound" : "STALE");
        }
    }
    if (current == rt) return true;

    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      API::FName param = make_fname(L"SlateUI");
      memcpy(p, &param, sizeof(int32_t) * 2);
      *reinterpret_cast<void**>(p + 8) = rt;
      mi->call_function(L"SetTextureParameterValue", p); }

    API::get()->log_info("[Halo-CampE-UEVR] widget reticule: SlateUI re-bound (mid=%p rt=%p, was %p)",
                         (void*)mi, (void*)rt, (void*)current);
    return true;
}

// `TintColorAndOpacity` is a straight multiplier on the widget's colour. The stock Widget3D pass is
// unlit but its output is still multiplied by the scene's PRE-EXPOSURE before the filmic tonemapper,
// so in a bright scene Halo's authored cyan lands near black. Multiplying the tint back up cancels
// that. The render target itself always held the correct colours (measured: R83 G197 B216) -- only
// the display path was crushing them.
//
// On the exposure-compensated material the gain must be 1.0 or the colours blow out instead.
//
// Re-applied whenever the value changes, so the gain can be dialled live in-headset: edit
// `aimwidgetgain` and the next tick picks it up. The component can also rebuild its material
// (SetDrawSize replaces the MID), which would silently drop the tint -- `force` re-asserts it.
//
// CREDIT: the pre-exposure diagnosis and the fallback-gain approach are elliotttate's.
void apply_widget_tint(API::UObject* comp, bool force) {
    if (comp == nullptr) return;

    const float gain = g_ret_widget_exposure_compensated ? 1.0f
                                                         : g_cfg.aim_widget_gain;
    const float rgb   = gain * g_cfg.aim_widget_tint;
    // HIDE BY ALPHA, NOT BY VISIBILITY -- see reticule_widget_set_scene_hidden.
    const bool hide_alpha = g_ws_scene_hidden.load(std::memory_order_relaxed) &&
                            g_cfg.xr_layer_hide_ws == 1;
    const float alpha = hide_alpha ? 0.0f : g_cfg.aim_widget_alpha;

    // VERIFY AGAINST THE COMPONENT, never against a cache of what we last wrote.
    //
    // The component rebuilds its material and render target behind our back -- SetDrawSize and the
    // component's own internal material update both do it -- and the tint reverts to its default of
    // 1.0 when that happens. A "did the config value change?" cache cannot see that: it still
    // believes the gain is applied, so the crosshair silently drops to unity gain and goes dark
    // until something else forces a rewrite. That presents as the tint working, then intermittently
    // not, with no config change involved.
    //
    // So read the live value back and re-assert on mismatch. This is what makes the gain durable
    // rather than a one-shot that happens to survive. (elliotttate's plugin does the same, for the
    // same reason.)
    auto* live = comp->get_property_data<float>(L"TintColorAndOpacity");
    const bool matches = live != nullptr &&
                         std::fabs(live[0] - rgb)   < 0.001f &&
                         std::fabs(live[1] - rgb)   < 0.001f &&
                         std::fabs(live[2] - rgb)   < 0.001f &&
                         std::fabs(live[3] - alpha) < 0.001f;
    if (!force && matches) return;

    // Log only on a real change, so a per-tick re-assert cannot flood the log.
    const bool reverted = !force && live != nullptr && !matches;

    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    auto* c = reinterpret_cast<float*>(p);
    c[0] = rgb; c[1] = rgb; c[2] = rgb; c[3] = alpha;
    comp->call_function(L"SetTintColorAndOpacity", p);

    static float last_logged = -1.0f;
    if (force || rgb != last_logged || reverted) {
        last_logged = rgb;
        API::get()->log_info("[Halo-CampE-UEVR] widget tint -> %.2f (gain %.2f x tint %.2f, alpha %.2f)%s%s",
                             rgb, gain, g_cfg.aim_widget_tint, alpha,
                             g_ret_widget_exposure_compensated ? " [exposure-compensated material]" : "",
                             reverted ? "  <-- REVERTED by the component, re-asserted" : "");
    }
}

void reticule_widget_finish() {
    if (g_ret_widget_finished) return;
    auto* comp = g_ret_widget_comp.get_checked(L"WidgetComponent");
    if (comp == nullptr) return;

    API::UObject* rt = nullptr;
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      comp->call_function(L"GetRenderTarget", p);
      rt = *reinterpret_cast<API::UObject**>(p); }
    if (rt == nullptr) return;      // not yet -- try again next tick

    // Re-apply the draw size now that the target exists: after the pre-registration call
    // CurrentDrawSize can still sit at the engine default (500x500), the request never having
    // reached the proxy.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      auto* d = reinterpret_cast<double*>(p);
      d[0] = g_cfg.aim_widget_draw; d[1] = g_cfg.aim_widget_draw;
      comp->call_function(L"SetDrawSize", p); }

    API::UObject* mid = nullptr;
    if (g_cfg.aim_widget_mat) {
        // The TRANSLUCENT INSTANCE, not the base material.
        //
        // /Engine/EngineMaterials/Widget3DPassThrough is the parent Material; the usable assets are
        // MaterialInstanceConstants beside it (_Translucent, _Masked, _Opaque, and _OneSided
        // variants). Building a MID from the bare parent produces a material the widget vertex
        // factory cannot use, so the renderer substitutes WorldGridMaterial -- a grey grid quad.
        // The two-sided variant matches our SetTwoSided(true).
        auto* src = API::get()->find_uobject<API::UObject>(
            L"MaterialInstanceConstant /Engine/EngineMaterials/Widget3DPassThrough_Translucent."
            L"Widget3DPassThrough_Translucent");
        if (src == nullptr) {
            src = API::get()->find_uobject<API::UObject>(
                L"MaterialInstanceConstant /Engine/EngineMaterials/Widget3DPassThrough_Masked."
                L"Widget3DPassThrough_Masked");
        }
        if (src != nullptr) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            *reinterpret_cast<int32_t*>(p) = 0;
            *reinterpret_cast<void**>(p + 8) = src;
            comp->call_function(L"CreateDynamicMaterialInstance", p);
            mid = *reinterpret_cast<API::UObject**>(p + 24);

            if (mid != nullptr) {
                alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
                API::FName param = make_fname(L"SlateUI");
                memcpy(q, &param, sizeof(int32_t) * 2);
                *reinterpret_cast<void**>(q + 8) = rt;
                mid->call_function(L"SetTextureParameterValue", q);
            }
        }
    }

    if (g_cfg.aim_widget_bg) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        auto* c = reinterpret_cast<float*>(p);
        c[0] = 1.0f; c[1] = 0.0f; c[2] = 0.0f; c[3] = 1.0f;   // opaque red diagnostic
        comp->call_function(L"SetBackgroundColor", p);
    }

    // Bind the render target into whatever material is on the component NOW -- which may be the MID
    // the block above just created, or the exposure-compensated MIC set at creation. Must run before
    // the tint: it is what decides whether the gain is 1.0 or aimwidgetgain.
    bind_widget_slate_ui(comp);

    // Tint is applied here rather than at creation for the same reason as everything else in this
    // function: the component is only fully built after it has ticked once.
    apply_widget_tint(comp, /*force=*/true);

    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; comp->call_function(L"RequestRedraw", p); }

    auto* base = reinterpret_cast<uint8_t*>(comp);
    API::get()->log_info("[Halo-CampE-UEVR] widget reticule FINISHED: rt=%p mid=%s curDraw=(%d,%d) mat=%p",
                         (void*)rt, mid != nullptr ? "translucent" : "FAILED",
                         *reinterpret_cast<int32_t*>(base + 1344),
                         *reinterpret_cast<int32_t*>(base + 1348),
                         *reinterpret_cast<void**>(base + 1496));
    g_ret_widget_finished = true;
}

// Position at the aim point and rotate to face the viewer. `origin` is the ray start, which is
// close enough to the eye for the facing to read correctly and avoids needing the HMD pose here.
void reticule_widget_move(const Vec3& target, const Vec3& origin) {
    // MODE 3/4's RE-ASSERT RUNS IN BOTH PLACES, DELIBERATELY.
    //
    // It is ALSO called unconditionally from update(), because this function is not per-tick: both of
    // its call sites sit behind a successful aim pick, so a tick whose pick fails skipped the repair
    // entirely -- and a failed pick and a widget re-host share their causes (a scope transition, a
    // pawn rebuild). That hole is real and the unconditional host closes it.
    //
    // But removing it from HERE was a mistake, caught by the session that owns the compositor lane
    // (2026-09-06). This call site runs LATE in the tick, after the reticule's own update and after
    // anything that rebuilds the widget's material or transform behind our back; the update() one
    // runs early. A flag re-applied only early can be clobbered later in the same frame by exactly
    // the rebuild it exists to survive. Their working mode-3 reproduction depended on the late
    // re-application, so dropping it risked regressing a configuration that was measured good.
    //
    // Calling it twice costs a masked bit compare on the steady path and writes only on disagreement,
    // so the duplicate is close to free and strictly safer than choosing one host over the other.
    reticule_mode3_reassert();

    // Validated through the object array, never by dereferencing the cached pointer: the component
    // is outered to the pawn, which is destroyed on death and area transitions.
    auto* comp = g_ret_widget_comp.get_checked(L"WidgetComponent");
    if (comp == nullptr) {
        if (!g_ret_widget_comp.empty() || g_ret_widget_failed || g_ret_widget_finished) {
            g_ret_widget_comp.reset();
            g_ret_widget_failed = false;     // allow a clean re-create against the new pawn
            g_ret_widget_finished = false;
            g_ret_hosted_widget.reset();
            g_ret_widget_parent.reset();
            g_ret_rebind_forced = false;

            // The candidate list is from the OLD level and must not be picked from. Its transient
            // entry is already dead, but the class-template entry survives a transition, so leaving
            // it in place is what let the re-create bind the template. Clearing forces the next
            // sweep to supply a fresh list before anything can be chosen.
            force_reticle_rescan();
        }
        return;
    }
    reticule_widget_finish();

    // RE-BIND IF THE HUD WAS REBUILT UNDER US.
    //
    // The component outlives the widget whenever the HUD is rebuilt without the pawn being
    // destroyed -- a checkpoint reload is exactly that. The component then holds a dead widget and
    // renders a frozen quad forever, while the game's new crosshair sits back on the flat HUD.
    // Cheap to detect: TrackedObject::get() is an index compare, no class-name lookup.
    if (!g_ret_hosted_widget.empty() && g_ret_hosted_widget.get() == nullptr) {
        // Force the sweep ONCE per episode, not per tick: it is a full object-array walk, and
        // re-forcing it every tick would run it every frame. pick_live_reticle() returns null until
        // that sweep lands (it only accepts candidates from the latest one), so this simply retries.
        if (!g_ret_rebind_forced) {
            g_ret_rebind_forced = true;
            force_reticle_rescan();
            API::get()->log_info("[Halo-CampE-UEVR] widget reticule: hosted widget died "
                                 "(HUD rebuilt) -- re-binding");
        }
        if (auto* fresh = pick_live_reticle()) {
            host_widget(comp, fresh);
            g_ret_widget_finished = false;   // new widget, new render target: re-run the finish pass
            g_ret_rebind_forced = false;
            API::get()->log_info("[Halo-CampE-UEVR] widget reticule: re-bound to %s",
                                 narrow(fresh->get_full_name()).c_str());
        }
    }

    // Live gain, verified against the component each tick (see apply_widget_tint): the write only
    // happens when the value has actually drifted, so the steady path is a property read.
    apply_widget_tint(comp, /*force=*/false);

    // KEEP THE RENDER TARGET LIVE.
    //
    // We detach the authored reticle from the HUD's widget tree, which also takes it off the path
    // that normally marks it dirty. A UWidgetComponent only re-renders its target when something
    // requests a redraw, so the quad can end up showing a FROZEN frame: the crosshair still looks
    // plausible (it is a static shape most of the time) while anything transient -- the hit marker,
    // the fire/heat animation -- appears once and then never updates again.
    //
    // Requesting a redraw on a cadence rather than every tick: enough to animate, while keeping the
    // engine call off the hot path (a per-tick UFunction call is the pattern that has cost us
    // framerate before). ~5 Hz at the 32 Hz tick.
    {
        static uint32_t redraw_tick = 0;
        if ((redraw_tick++ % 6) == 0) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            comp->call_function(L"RequestRedraw", p);

            // Same cadence, same reason: SetDrawSize and the component's own material updates can
            // replace the render target or the material instance at any point, and a material left
            // sampling the old one crashes the translucency pass rather than merely looking wrong.
            // Reads two properties and writes only when they have actually diverged.
            bind_widget_slate_ui(comp);
        }
    }

    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      auto* d = reinterpret_cast<double*>(p);
      d[0] = target.x; d[1] = target.y; d[2] = target.z;
      *reinterpret_cast<int32_t*>(p + 40) = 2;   // ETeleportType::ResetPhysics
      comp->call_function(L"K2_SetWorldLocation", p); }

    const float dx = origin.x - target.x, dy = origin.y - target.y, dz = origin.z - target.z;
    const float horiz = std::sqrt(dx * dx + dy * dy);
    float yaw   = std::atan2(dy, dx) / DEG2RAD;
    const float pitch = std::atan2(dz, horiz) / DEG2RAD;
    if (g_cfg.aim_widget_flip) yaw = wrap180(yaw + 180.0f);

    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      auto* d = reinterpret_cast<double*>(p);
      d[0] = pitch; d[1] = yaw; d[2] = 0.0;
      *reinterpret_cast<int32_t*>(p + 32) = 2;
      comp->call_function(L"K2_SetWorldRotation", p); }

    // Scale re-applied every tick so aimwidgetscale is live-tunable: it is otherwise only set at
    // construction, and FinishAddComponent overwrites it.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      // Distance-compensated, same as the mesh ring above.
      // MODE 2 (scale-hide): shrink the quad to sub-pixel instead of touching visibility.
      //
      // The component stays visible, in the frustum and RENDERED -- so whatever gate stops an
      // unrendered UWidgetComponent redrawing its target never trips -- but it covers no pixels.
      // A different mechanism from mode 1 (alpha) on purpose: if the redraw gate turns out to be
      // material-related, alpha fails and this still works, and vice versa.
      //
      // It MUST stay in the frustum. Moving it behind the camera would cull it, which means not
      // rendered, which is the freeze all over again. Shrinking in place cannot do that -- though
      // if the renderer culls it by screen size, raise xrlayerhidescale until it survives.
      float sc = g_cfg.aim_widget_scale * g_ret_scale_mul.load();
      if (g_ws_scene_hidden.load(std::memory_order_relaxed) && g_cfg.xr_layer_hide_ws == 2) {
          sc *= g_cfg.xr_layer_hide_scale;
      }

      // ---- ZOOM FIT: shrink the world reticule so it looks right INSIDE THE SCOPE PANE ---------
      //
      // Reported 2026-08-30: "the reticle is a bit large in the zoom pane". It is, and by a factor
      // we can compute rather than guess. The reticule is a world object sized to look constant in
      // the MAIN view; the pane shows a capture rendered at scope_base_fov / scope_zoom (70/16 =
      // ~4.4 deg), then displays that image on a quad subtending roughly 2*atan(w/2 / d). Anything
      // in the capture is therefore magnified by (pane angular width / capture FOV) relative to
      // being looked at directly -- about 3x at the shipped numbers, which is exactly "a bit large".
      //
      // THIS IS ONLY SAFE IN MODE 3, and that is why it is gated on it. In modes 0/1/2 the same
      // widget is what the player sees in the main view (or is the thing being hidden), so shrinking
      // it would shrink the main-view reticule. In mode 3 the widget is bVisibleInSceneCaptureOnly
      // -- the main view is showing the compositor quad instead -- so its size affects the pane and
      // NOTHING else. The fix is free precisely because of the mode it rides on.
      //
      // Auto by default, with xrlayerzoomfit as a trim on top, because the derivation assumes the
      // pane distance is the mount offset and that the capture fills the quad. Both are true today
      // and neither is guaranteed, so the computed factor is LOGGED and the trim exists to correct
      // it without a rebuild.
      if (g_cfg.xr_layer_hide_ws == 3 && g_scope_active.load(std::memory_order_relaxed)
          && g_ws_scene_hidden.load(std::memory_order_relaxed)) {
          const float cap_fov = (g_cfg.scope_zoom > 1.0f) ? (g_cfg.scope_base_fov / g_cfg.scope_zoom)
                                                          : g_cfg.scope_base_fov;
          const float d_cm = std::sqrt(g_cfg.scope_layer_fwd   * g_cfg.scope_layer_fwd +
                                       g_cfg.scope_layer_right * g_cfg.scope_layer_right +
                                       g_cfg.scope_layer_up    * g_cfg.scope_layer_up);
          if (cap_fov > 0.01f && d_cm > 1.0f && g_cfg.scope_layer_width > 0.01f) {
              const float pane_deg = 2.0f * std::atan((g_cfg.scope_layer_width * 0.5f) / d_cm)
                                          * (180.0f / 3.14159265f);
              const float fit = cap_fov / pane_deg;             // <1 shrinks, which is the expected way
              sc *= fit * g_cfg.xr_layer_zoom_fit;
              static float s_said = 0.0f;
              if (std::fabs(fit - s_said) > 0.01f) {
                  s_said = fit;
                  uevr::API::get()->log_info(
                      "[Halo-CampE-UEVR] reticule ZOOM FIT: capture %.2f deg on a %.1f deg pane -> "
                      "world reticule x%.3f (trim xrlayerzoomfit=%.2f). Mode 3 only: the widget is "
                      "capture-only, so this changes the PANE and not the main view.",
                      cap_fov, pane_deg, fit, g_cfg.xr_layer_zoom_fit);
              }
          }
      }
      auto* d = reinterpret_cast<double*>(p); d[0] = sc; d[1] = sc; d[2] = sc;
      comp->call_function(L"SetWorldScale3D", p); }
}

// GIVE THE CROSSHAIR BACK when the feature is switched off.
//
// Hosting the widget calls RemoveFromParent on it, which takes the game's crosshair OUT of the HUD.
// Without this, setting aimwidget=0 live left the player with no crosshair at all -- the widget was
// off the HUD and our quad had stopped being positioned. A live tunable has to be reversible.
//
// Deliberately NOT wired into the teardown path (pawn/level destruction). There the widget is
// usually being destroyed anyway, and re-parenting during a level transition means engine calls at
// the single most fragile moment in this title's lifecycle -- the one that has already produced
// crashes here. This runs only on an explicit, user-initiated toggle, where nothing else is in
// flight. Everything is re-validated through the object array first; any dead handle skips.
void reticule_widget_release() {
    // Self-latching: this is called every tick from the disabled branch, and doing nothing when
    // nothing was ever hosted keeps that path to a single bool test. Set in host_widget().
    if (!g_ret_widget_hosted_once) return;
    g_ret_widget_hosted_once = false;

    auto* comp = g_ret_widget_comp.get_checked(L"WidgetComponent");
    auto* w    = g_ret_hosted_widget.get();
    auto* par  = g_ret_widget_parent.get();

    if (comp != nullptr) {
        { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};   // SetWidget(nullptr)
          comp->call_function(L"SetWidget", p); }
        { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
          comp->call_function(L"SetVisibility", p); }   // false
        { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1;
          comp->call_function(L"SetHiddenInGame", p); }
    }
    if (w != nullptr && par != nullptr) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<void**>(p) = w;
        par->call_function(L"AddChild", p);
        API::get()->log_info("[Halo-CampE-UEVR] widget reticule RELEASED -- crosshair returned to %s",
                             narrow(class_name_of(par)).c_str());
    }

    g_ret_hosted_widget.reset();
    g_ret_widget_parent.reset();
}

bool reticule_force_visible(Vec3* out_pos, int* out_have) {
    auto* mesh = g_ret_mesh.get_checked(L"StaticMeshComponent");
    auto* wid  = g_ret_widget_comp.get_checked(L"WidgetComponent");
    if (out_have != nullptr) *out_have = (mesh != nullptr ? 1 : 0) | (wid != nullptr ? 2 : 0);

    uevr::API::UObject* comps[2] = { mesh, wid };
    for (auto* c : comps) {
        if (c == nullptr) continue;
        // Both setters take (bool, bool bPropagateToChildren); the zeroed buffer supplies the
        // second argument as false, which is what we want -- neither component has children.
        { alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
          q[0] = 1; c->call_function(L"SetVisibility", q); }
        // NOTE: the scene-hidden latch is NOT applied here any more -- hiding is done with alpha
        // (see reticule_widget_set_scene_hidden), precisely so this function can keep forcing both
        // components visible without fighting it.
        //
        // Without this the latch would lose a fight it never knew it was in: this function runs
        // every tick on the seated path and unconditionally un-hid both components, so hiding the
        // in-scene crosshair would appear to work for one frame and then flicker back forever.
        // VISIBILITY stays TRUE either way -- see reticule_widget_set_scene_hidden for why that
        // distinction is the whole trick.
        { alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
          q[0] = 0; c->call_function(L"SetHiddenInGame", q); }
    }

    if (mesh == nullptr || out_pos == nullptr) return false;
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    mesh->call_function(L"K2_GetComponentLocation", params);
    auto* v = reinterpret_cast<double*>(params);
    *out_pos = Vec3{(float)v[0], (float)v[1], (float)v[2]};
    return true;
}

void reticule_mesh_move(const Vec3& p) {
    // Validated through the object array, never by dereferencing the cached pointer: the component
    // is outered to the pawn, which is destroyed on death and area transitions.
    auto* mesh = g_ret_mesh.get_checked(L"StaticMeshComponent");
    if (mesh == nullptr) {
        if (!g_ret_mesh.empty() || g_ret_mesh_failed) {
            g_ret_mesh.reset();
            g_ret_mesh_failed = false;   // allow one clean re-create against the new pawn
        }
        return;
    }

    // Disabling the mesh must HIDE it, not merely stop moving it -- otherwise it stays rendering,
    // frozen at its last aim point. Tracked so re-enabling shows it again.
    static bool hidden = false;
    if (!g_cfg.aim_mesh) {
        if (!hidden) {
            hidden = true;
            alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
            mesh->call_function(L"SetVisibility", q);   // false
        }
        return;
    }
    if (hidden) {
        hidden = false;
        alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
        q[0] = 1;
        mesh->call_function(L"SetVisibility", q);
    }
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    auto* d = reinterpret_cast<double*>(params);
    d[0] = p.x; d[1] = p.y; d[2] = p.z;
    *reinterpret_cast<int32_t*>(params + 40) = 2;   // ETeleportType::ResetPhysics
    mesh->call_function(L"K2_SetWorldLocation", params);

    // Scale re-applied every tick so aimmeshscale is live-tunable -- it is otherwise only set at
    // creation, and resizing the ring would cost a relaunch per attempt.
    {
        alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
        // Scaled by distance (see g_ret_scale_mul) so the ring keeps the same apparent size
        // wherever it is placed.
        const float sc = g_cfg.aim_mesh_scale * g_ret_scale_mul.load();
        auto* d = reinterpret_cast<double*>(q); d[0] = sc; d[1] = sc; d[2] = sc;
        mesh->call_function(L"SetWorldScale3D", q);
    }

    // Re-applied every tick so the colour keys in the config are live-tunable.
    if (!g_cfg.aim_tex) {
        reticule_mesh_color(g_cfg.aim_mesh_cr, g_cfg.aim_mesh_cg, g_cfg.aim_mesh_cb, 1.0f);
    } else if (g_cfg.aim_mesh_override &&
               (g_ret_mesh_mid.empty() || g_applied_mesh_parent != g_cfg.aim_mesh_parent)) {
        // Rebind when the parent is missing (on-demand assets stream in late) OR the configured
        // parent changed -- which makes material experiments a live config edit instead of a
        // relaunch per candidate.
        static uint32_t bind_backoff = 0;
        if ((bind_backoff++ % 120) == 0) {
            reticule_mesh_bind_material(mesh);
            if (!g_ret_mesh_mid.empty()) g_applied_mesh_parent = g_cfg.aim_mesh_parent;
        }
    } else if (auto* mid = g_ret_mesh_mid.get_checked(L"MaterialInstanceDynamic")) {
        // Generic colour names too, so an arbitrary game material parent (MI_Arrow, the light
        // cones, the hit markers...) picks the tint up under whatever its parameter is called.
        reticule_mesh_color(g_cfg.aim_mesh_cr, g_cfg.aim_mesh_cg, g_cfg.aim_mesh_cb, 1.0f);
        // Live-tunable tint, and the same channel a hit flash would use.
        {
            alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
            API::FName param = make_fname(L"TintColorAndOpacity");
            memcpy(q, &param, sizeof(int32_t) * 2);
            auto* c = reinterpret_cast<float*>(q + 8);
            c[0] = g_cfg.aim_mesh_cr; c[1] = g_cfg.aim_mesh_cg; c[2] = g_cfg.aim_mesh_cb; c[3] = 1.0f;
            mid->call_function(L"SetVectorParameterValue", q);
        }

        // PIN THE TEXTURE'S MIPS. These are streaming UI textures -- the game wraps them in
        // HaloUILazyImage, i.e. they load on demand. The UTexture2D object exists and binds fine,
        // but with no resident mips the sample returns BLACK, which is indistinguishable from a
        // black texture or a bad parameter. Re-asserted periodically because the streamer will
        // otherwise evict a texture nothing else is asking for.
        if (auto* t = g_ret_tex.get_checked(L"Texture")) {
            static uint32_t pin_counter = 0;
            if ((pin_counter++ % 300) == 0) {
                alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
                *reinterpret_cast<float*>(q)     = 30.0f;   // Seconds
                *reinterpret_cast<int32_t*>(q + 4) = -1;    // CinematicTextureGroups
                t->call_function(L"SetForceMipLevelsToBeResident", q);
            }
        }

        // Live texture swap. Sources by precedence: the hosted widget's render target
        // (aimrtwidget), a PNG from disk (aimtexfile), a game texture asset (aimtexpath).
        static std::string applied;
        const bool use_rt   = g_cfg.aim_rt_from_widget;
        const bool use_file = !use_rt && g_cfg.aim_tex_file[0] != 0;
        std::string want;
        if (use_rt) {
            // Keyed per widget-component instance: the render target is recreated with it.
            char tag[32];
            snprintf(tag, sizeof(tag), "rt:%p", (void*)g_ret_widget_comp.ptr);
            want = tag;
        } else if (use_file) {
            want = std::string("file:") + g_cfg.aim_tex_file;
        } else {
            want = g_cfg.aim_tex_path;
        }
        static const void* applied_on_mid = nullptr;
        if ((applied != want || applied_on_mid != (const void*)mid) && !want.empty()) {
            applied_on_mid = (const void*)mid;
            applied = want;   // set even on failure, so a bad path is not retried every tick
            API::UObject* t = nullptr;
            if (use_rt) {
                if (auto* wc = g_ret_widget_comp.get_checked(L"WidgetComponent")) {
                    alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
                    wc->call_function(L"GetRenderTarget", q);
                    t = *reinterpret_cast<API::UObject**>(q);
                }
            } else if (use_file) {
                t = import_texture_file(g_cfg.aim_tex_file);
            } else {
                const std::string a{g_cfg.aim_tex_path};
                const std::wstring w(a.begin(), a.end());
                t = API::get()->find_uobject<API::UObject>(w.c_str());
                if (t == nullptr) {
                    // Strip any leading class name, then load: the texture for a weapon you are not
                    // currently holding will not be resident.
                    const size_t sp = a.find(' ');
                    t = load_asset_by_path((sp == std::string::npos ? a : a.substr(sp + 1)).c_str());
                }
            }
            if (t != nullptr) {
                // An exact parameter name from texhunt/matdump wins; the shotgun list is the
                // fallback when none is configured (writing a name a material lacks is a no-op).
                if (g_cfg.aim_tex_param[0] != 0) {
                    const std::string a{g_cfg.aim_tex_param};
                    const std::wstring w2(a.begin(), a.end());
                    alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
                    API::FName param = make_fname(w2.c_str());
                    memcpy(q, &param, sizeof(int32_t) * 2);
                    *reinterpret_cast<void**>(q + 8) = t;
                    mid->call_function(L"SetTextureParameterValue", q);
                }
                // Shotgunned across candidate parameter names: with an arbitrary game material as
                // parent, its texture parameter's name is unknowable up front, and writing a name a
                // material lacks is a no-op on the MID. SlateUI covers the widget family.
                static const wchar_t* kTexNames[] = {
                    L"SlateUI", L"Texture", L"Tex", L"MainTexture", L"BaseTexture", L"Mask",
                    L"OpacityMask", L"Pattern", L"Noise", L"Gradient", L"Diffuse", L"Albedo",
                    L"EmissiveTexture", L"Lines", L"Sprite"
                };
                for (const wchar_t* nm : kTexNames) {
                    alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
                    API::FName param = make_fname(nm);
                    memcpy(q, &param, sizeof(int32_t) * 2);
                    *reinterpret_cast<void**>(q + 8) = t;
                    mid->call_function(L"SetTextureParameterValue", q);
                }
                g_ret_tex.set(t);
                API::get()->log_info("[Halo-CampE-UEVR] reticule texture -> %s (%p)", want.c_str(), (void*)t);
            } else {
                API::get()->log_info("[Halo-CampE-UEVR] reticule texture NOT FOUND: %s", want.c_str());
                // The widget's render target is allocated lazily, so an rt: miss is expected on the
                // first attempts -- keep retrying. File/asset misses stay latched (a bad path would
                // otherwise be retried every tick forever).
                if (use_rt) applied.clear();
            }
        }
    }

    // A flat quad has to be turned to face the viewer or it is edge-on and invisible. Same look-at
    // the widget uses, plus a configurable offset because /Engine/BasicShapes/Plane's facing axis is
    // not something to guess at -- aimtexrot lets it be corrected without a rebuild.
    // Applied for ANY mesh, not just the textured plane: a torus left at its authored orientation
    // (axis up, hole facing the sky) reads as a flat ring seen edge-on. A ring reticule only works
    // if its axis points at the viewer, and that needs the same look-at the plane uses.
    //
    // aimtexrot{p,y,r} are the offset knobs for whichever mesh is in use: a plane wants its face
    // turned toward you, a torus wants its AXIS turned toward you, so the two differ by 90 degrees.
    if (g_have_ret_origin.load()) {
        const Vec3 o = g_ret_origin;
        const float dx = o.x - p.x, dy = o.y - p.y, dz = o.z - p.z;
        const float horiz = std::sqrt(dx * dx + dy * dy);
        const float yaw   = std::atan2(dy, dx) / DEG2RAD;
        const float pitch = std::atan2(dz, horiz) / DEG2RAD;
        alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
        auto* d = reinterpret_cast<double*>(q);
        d[0] = pitch + g_cfg.aim_tex_rot_p;
        d[1] = yaw   + g_cfg.aim_tex_rot_y;
        d[2] = g_cfg.aim_tex_rot_r;
        *reinterpret_cast<int32_t*>(q + 32) = 2;
        mesh->call_function(L"K2_SetWorldRotation", q);
    }
}

} // namespace halo
