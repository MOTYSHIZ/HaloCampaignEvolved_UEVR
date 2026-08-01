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
#include "Config.hpp"
#include "Math.hpp"
#include "UeObject.hpp"

#include <atomic>
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
API::UObject* find_or_load_material(const std::string& object_path) {
    const std::wstring w(object_path.begin(), object_path.end());
    static const wchar_t* kPrefixes[] = { L"MaterialInstanceConstant ", L"Material ",
                                          L"MaterialInstanceDynamic " };
    for (const wchar_t* pre : kPrefixes) {
        if (auto* m = API::get()->find_uobject<API::UObject>((std::wstring(pre) + w).c_str())) return m;
    }
    return load_asset_by_path(object_path.c_str());
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

// ---- WIDGET RETICULE
// Hosts the game's own crosshair widget on a world-space UWidgetComponent, so the per-weapon art
// and the hit marker (both children of that widget) come with it.
TrackedObject g_ret_widget_comp;
bool g_ret_widget_failed = false;

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

// Prefer the CONSTRUCTED instance over the class template. The template lives under
// <Class>.WidgetTree.FirstPersonReticle and is not what renders; the live one is under
// /Engine/Transient. Handing the template to a WidgetComponent would host a widget nothing drives,
// so no hit markers would ever fire.
API::UObject* pick_live_reticle() {
    API::UObject* fallback = nullptr;
    for (int i = 0; i < g_reticle_count; ++i) {
        auto* o = g_reticles[i].obj.get();
        if (o == nullptr) continue;
        if (class_name_of(o).find(wanted_widget_class()) == std::wstring::npos) continue;

        bool transient = false;
        for (API::UObject* p = o; p != nullptr; p = p->get_outer()) {
            const auto* fn = p->get_fname();
            if (fn != nullptr && fn->to_string().find(L"Transient") != std::wstring::npos) {
                transient = true;
                break;
            }
        }
        if (transient) return o;
        if (fallback == nullptr) fallback = o;
    }
    return fallback;
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

    // ---- DEFERRED CONSTRUCTION, configure, THEN finish.
    //
    // CREDIT: this sequence comes from Pande's OblivionVR (Profile/scripts/VRHud.lua), which puts a
    // game HUD widget into world space and renders it in colour:
    //
    //     comp = actor:AddComponentByClass(WidgetClass, false, zero_transform, /*deferred*/ false)
    //     comp:SetWidget(w); comp:SetDrawSize(...); comp:SetMaterial(0, translucent_MIC)
    //     comp.BlendMode = 2
    //     actor:FinishAddComponent(comp, false, zero_transform)
    //
    // The ordering is the whole point. A registered component builds its render target AND picks
    // its material from BlendMode immediately, so setting those AFTERWARDS loses: the CDO write
    // never propagates, and a post-hoc material swap runs against a null render target.
    // Deferring registration lets all of it be set BEFORE anything is built.
    //
    // It also assigns the translucent MaterialInstanceConstant DIRECTLY rather than creating a MID
    // from the base material -- the base is what rendered WorldGridMaterial here.
    auto* comp = API::get()->add_component_by_class(owner, wc_cls, /*deferred=*/true);
    if (comp == nullptr) {
        g_ret_widget_failed = true;
        API::get()->log_info("[Halo-CampE-UEVR] widget reticule: deferred add_component_by_class FAILED");
        return;
    }

    // BlendMode has no setter, so write the property directly. Offset 1428 was verified against the
    // live object (OpacityFromTexture reads 1.0f at 1424 immediately before it).
    constexpr size_t BLENDMODE_OFFSET = 1428;
    *(reinterpret_cast<uint8_t*>(comp) + BLENDMODE_OFFSET) =
        (uint8_t)clampf((float)g_cfg.aim_widget_blend, 0.0f, 2.0f);

    // The MIC itself, not a MID of the parent Material.
    if (auto* mic = API::get()->find_uobject<API::UObject>(
            L"MaterialInstanceConstant /Engine/EngineMaterials/Widget3DPassThrough_Translucent."
            L"Widget3DPassThrough_Translucent")) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<int32_t*>(p) = 0;
        *reinterpret_cast<void**>(p + 8) = mic;
        comp->call_function(L"SetMaterial", p);
    }

    // Space FIRST: setting the widget before the space can build the render target for the wrong
    // mode. 0 = World, 1 = Screen.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 0; comp->call_function(L"SetWidgetSpace", p); }

    // FVector2D is double under UE5 LWC.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      auto* d = reinterpret_cast<double*>(p);
      d[0] = g_cfg.aim_widget_draw; d[1] = g_cfg.aim_widget_draw;
      comp->call_function(L"SetDrawSize", p); }

    // Two-sided: if our facing maths is ever off by 180 the reticule is still visible rather than
    // invisible, which is a much easier failure to diagnose.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1; comp->call_function(L"SetTwoSided", p); }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 0; comp->call_function(L"SetCollisionEnabled", p); }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1; p[1] = 1; p[2] = 1;
      comp->call_function(L"SetAbsolute", p); }

    const float s = g_cfg.aim_widget_scale;
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      auto* d = reinterpret_cast<double*>(p); d[0] = s; d[1] = s; d[2] = s;
      comp->call_function(L"SetWorldScale3D", p); }

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

    // Explicit: a dynamically added component inherits tick settings from the CDO, and the redraw
    // that fills the render target happens on tick.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1;
      comp->call_function(L"SetComponentTickEnabled", p); }

    // Widget components only redraw when they think they are visible; a reticule that sits off to
    // the side of the view is precisely the case that gets culled.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1;
      comp->call_function(L"SetTickWhenOffscreen", p); }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      comp->call_function(L"RequestRedraw", p); }

    // No shadow: it is a HUD element, and a floating quad throwing a shadow onto the level reads as
    // a bug immediately. Also keeps it out of the depth/shadow passes entirely.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; comp->call_function(L"SetCastShadow", p); }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1; comp->call_function(L"SetVisibility", p); }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 0; comp->call_function(L"SetHiddenInGame", p); }

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

    // FINISH LAST. Registration happens here, so everything above -- blend mode, material, widget,
    // draw size, two-sided -- is already in place when the component builds its render target and
    // scene proxy. FTransform is identity: quat(0,0,0,1), translation 0, scale 1, at UE5 double
    // precision (32-byte quat, then 32 for translation incl. padding, then 32 for scale).
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

    // BoundsScale: uevrlib sets this too -- small widget components get frustum-culled without
    // it as soon as they sit near the edge of view.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      *reinterpret_cast<float*>(p) = 10.0f;
      comp->call_function(L"SetBoundsScale", p); }

    // Scale AFTER finishing. FinishAddComponent takes a RelativeTransform and applies it, so the
    // identity transform passed above overwrites any scale set before it.
    {
        const float sc = g_cfg.aim_widget_scale;
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        auto* d = reinterpret_cast<double*>(p); d[0] = sc; d[1] = sc; d[2] = sc;
        comp->call_function(L"SetWorldScale3D", p);
    }

    g_ret_widget_comp.set(comp);

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

        API::get()->log_info(
            "[Halo-CampE-UEVR] widget reticule CREATED @%p hosting %s | space=%u blend=%u "
            "curDraw=(%d,%d) rt=%p mat=%p widget=%p",
            (void*)comp, narrow(class_name_of(w)).c_str(), (unsigned)space, (unsigned)blend,
            cur_x, cur_y, rt_p, mat_p, wid_p);
    }
}

// Everything that can only be done AFTER the component has ticked at least once and allocated its
// render target. Retried every tick until it succeeds, then latched.
bool g_ret_widget_finished = false;

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

    // Tint is applied here rather than at creation for the same reason as everything else in this
    // function: the component is only fully built after it has ticked once.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      auto* c = reinterpret_cast<float*>(p);
      c[0] = g_cfg.aim_widget_tint; c[1] = g_cfg.aim_widget_tint;
      c[2] = g_cfg.aim_widget_tint; c[3] = g_cfg.aim_widget_alpha;
      comp->call_function(L"SetTintColorAndOpacity", p); }

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
    // Validated through the object array, never by dereferencing the cached pointer: the component
    // is outered to the pawn, which is destroyed on death and area transitions.
    auto* comp = g_ret_widget_comp.get_checked(L"WidgetComponent");
    if (comp == nullptr) {
        if (!g_ret_widget_comp.empty() || g_ret_widget_failed || g_ret_widget_finished) {
            g_ret_widget_comp.reset();
            g_ret_widget_failed = false;     // allow a clean re-create against the new pawn
            g_ret_widget_finished = false;
        }
        return;
    }
    reticule_widget_finish();

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
      const float sc = g_cfg.aim_widget_scale;
      auto* d = reinterpret_cast<double*>(p); d[0] = sc; d[1] = sc; d[2] = sc;
      comp->call_function(L"SetWorldScale3D", p); }
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
        const float sc = g_cfg.aim_mesh_scale;
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
