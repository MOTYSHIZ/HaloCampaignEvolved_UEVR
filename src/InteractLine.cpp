// The grab guide. See InteractLine.hpp for what it is and why it only shows when the grab works.

#include "InteractLine.hpp"

#include "Config.hpp"
#include "Reticule.hpp"     // make_color_rt(), find_or_load_material() -- the proven mesh path
#include "Rig.hpp"          // call_ret_vec3()
#include "UeObject.hpp"     // TrackedObject, RIG_PARAM_BUF

#include <cmath>

using namespace uevr;

namespace halo {

namespace {

// Validated through the object array on every use, never by dereferencing a cached pointer: this
// component is outered to the pawn, which is destroyed on death and on every area transition. The
// same discipline the reticule mesh uses, and the reason a level change does not crash us the way
// it does when a raw component pointer is kept.
TrackedObject g_line;
TrackedObject g_mid;        // the dynamic material instance, for the colour
bool g_failed = false;      // creation refused; do not retry every tick
bool g_visible = false;     // last SetVisibility we sent, so we only send changes
float g_col_sent[4] = {-1.0f, -1.0f, -1.0f, -1.0f};   // last colour pushed, so we only push changes

constexpr float RAD2DEG_F = 57.29577951308232f;

void set_visible(API::UObject* comp, bool on) {
    if (g_visible == on) return;   // SetVisibility every tick is a reflected call for nothing
    g_visible = on;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    p[0] = on ? 1 : 0;
    comp->call_function(L"SetVisibility", p);
}

// Build the component once. Mirrors reticule_mesh_ensure(): engine primitive, absolute transform,
// no collision, no shadow.
bool ensure(API::UObject* rig) {
    if (!g_line.empty()) return true;
    if (g_failed) return false;

    auto* owner = (rig != nullptr) ? rig->get_outer() : nullptr;
    if (owner == nullptr) return false;   // not fatal -- try again next tick against a live pawn

    auto* smc_cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.StaticMeshComponent");
    if (smc_cls == nullptr) {
        g_failed = true;
        API::get()->log_info("[Halo-CampE-UEVR] GRABGUIDE: StaticMeshComponent class NOT FOUND");
        return false;
    }
    auto* comp = API::get()->add_component_by_class(owner, smc_cls, false);
    if (comp == nullptr) {
        g_failed = true;
        API::get()->log_info("[Halo-CampE-UEVR] GRABGUIDE: add_component_by_class FAILED");
        return false;
    }

    // A unit cube stretched into a bar. Cube is 100 cm, so scale is (length/100, t/100, t/100).
    //
    // WHY NOT A FEATHERED BEAM, which is what was actually asked for: feathering needs a texture
    // with an alpha falloff, and there is no way to author one here -- make_color_rt() is a flat
    // fill (CreateRenderTarget2D + ClearRenderTarget2D), and nothing in this plugin draws pixels.
    // The honest options were a Canvas draw (a pile of new reflected calls) or an engine gradient
    // asset that may not be cooked into this build. So: a translucent bar now, and `grabguidemat`
    // takes a full material object path -- point it at a feathered material from an added pak and
    // this becomes the beam with no code change.
    API::UObject* mesh = nullptr;
    static const wchar_t* kMeshes[] = {
        L"StaticMesh /Engine/BasicShapes/Cube.Cube",
        L"StaticMesh /Engine/EngineMeshes/Cube.Cube",
    };
    for (const wchar_t* m : kMeshes) {
        mesh = API::get()->find_uobject<API::UObject>(m);
        if (mesh != nullptr) break;
    }
    if (mesh != nullptr) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<void**>(p) = mesh;
        comp->call_function(L"SetStaticMesh", p);
    } else {
        API::get()->log_info("[Halo-CampE-UEVR] GRABGUIDE: no engine cube found -- guide invisible");
    }

    // BasicShapeMaterial, NOT Widget3DPassThrough.
    //
    // CORRECTED 2026-09-04, after the first build shipped a BLACK beam. Reticule.cpp already
    // records why, and I used the wrong one anyway: "Widget3DPassThrough (the BASE material behind
    // the widget instances) is not cooked for use on a mesh in this build and renders flat black."
    // That trap is written down precisely so nobody walks into it twice.
    //
    // BasicShapeMaterial is the default material of the /Engine/BasicShapes meshes, so it is
    // packaged wherever the cube is, and it exposes a colour parameter that works on a mesh.
    {
        API::UObject* base = nullptr;
        if (g_cfg.grab_guide_mat[0] == '/') {
            base = find_or_load_material(std::string(g_cfg.grab_guide_mat));
        }
        // UNLIT FIRST, then the lit fallback.
        //
        // BasicShapeMaterial is LIT, so the beam takes the scene's lighting and exposure: it dims
        // in shadow and washes out in sun, which is wrong for something whose entire job is to be
        // findable. The same argument the XR compositor layer exists for, one rung down.
        //
        // Candidate list rather than one name, and each is logged, because which unlit materials
        // are cooked into THIS build is not something to guess one release at a time -- the torus
        // lookup in Reticule.cpp is the same pattern for the same reason. If none is present the
        // lit material still works and the beam is merely lighting-dependent, so this degrades
        // rather than failing.
        static const wchar_t* kUnlit[] = {
            L"Material /Engine/EngineMaterials/EmissiveMeshMaterial.EmissiveMeshMaterial",
            L"Material /Engine/EngineDebugMaterials/DebugMeshMaterial.DebugMeshMaterial",
            L"Material /Engine/EngineMaterials/UnlitMaterial.UnlitMaterial",
        };
        const wchar_t* picked = L"(none)";
        for (const wchar_t* m : kUnlit) {
            if (base != nullptr) break;
            base = API::get()->find_uobject<API::UObject>(m);
            if (base != nullptr) picked = m;
        }
        if (base != nullptr && picked[0] != L'(') {
            API::get()->log_info("[Halo-CampE-UEVR] GRABGUIDE: unlit material %s", narrow(picked).c_str());
        }
        if (base == nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] GRABGUIDE: no unlit material found -- falling back "
                                 "to BasicShapeMaterial, which IS lit, so the beam will dim in shadow. "
                                 "Point grabguidemat at an unlit material to fix.");
            base = API::get()->find_uobject<API::UObject>(
                L"Material /Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial");
        }
        if (base != nullptr) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            *reinterpret_cast<int32_t*>(p) = 0;
            *reinterpret_cast<void**>(p + 8) = base;
            comp->call_function(L"CreateDynamicMaterialInstance", p);
            g_mid.set(*reinterpret_cast<API::UObject**>(p + 24));
        }
        API::get()->log_info("[Halo-CampE-UEVR] GRABGUIDE: created (material %s, mid %s)",
                             base != nullptr ? "BasicShapeMaterial" : "MISSING",
                             g_mid.empty() ? "none" : "ok");
    }

    // A bar lying on the barrel must not block shots, bump the player, or cast a shadow.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 0; comp->call_function(L"SetCollisionEnabled", p); }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; comp->call_function(L"SetCastShadow", p); }
    // ABSOLUTE, and NOT attached to the rig.
    //
    // REVERTED 2026-09-05 after the field report "a large transparent cube parented to my firearm".
    // Attaching and writing RELATIVE offsets was an attempt to remove a one-frame lag, and on this
    // stack the relative setters DO NOT APPLY -- Plugin.cpp's rig block states it plainly, and
    // reads its own writes back for exactly this reason: "K2_SetRelativeTransform is a known no-op
    // on this stack, so K2_SetRelativeLocation is never assumed to apply."
    //
    // With every relative write a silent no-op the component simply kept its defaults: scale
    // (1,1,1) on a 100 cm engine cube, hence a metre-wide transparent box riding the gun. Nothing
    // errored. That warning was eleven lines from code I had already read.
    //
    // The lag it was meant to fix is real but small, and this is now only the FALLBACK for when the
    // compositor route is unavailable. A beam one frame behind beats a beam the size of a room.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1; p[1] = 1; p[2] = 1;
      comp->call_function(L"SetAbsolute", p); }
    // Scale at creation too, so a component that somehow never reaches the update path is not a
    // 100 cm cube while it waits.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      auto* d = reinterpret_cast<double*>(p);
      d[0] = 0.001; d[1] = 0.001; d[2] = 0.001;
      comp->call_function(L"SetWorldScale3D", p); }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; comp->call_function(L"SetVisibility", p); }
    g_visible = false;
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 0; comp->call_function(L"SetHiddenInGame", p); }

    g_line.set(comp);
    return !g_line.empty();
}

} // namespace

void interact_line_release() {
    g_line.reset();
    g_mid.reset();          // belongs to the component that just went away
    g_failed = false;
    g_visible = false;
    for (float& c : g_col_sent) c = -1.0f;   // force a re-push against the new material instance
}

bool interact_line_parse_key(const char* key, double v, const char* val) {
    if (_stricmp(key, "grabguide")      == 0) { g_cfg.grab_guide   = (v != 0.0); return true; }
    if (_stricmp(key, "grabguider")     == 0) { g_cfg.grab_guide_r = clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "grabguideg")     == 0) { g_cfg.grab_guide_g = clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "grabguideb")     == 0) { g_cfg.grab_guide_b = clampf((float)v, 0.0f, 1.0f); return true; }
    // Live so the label can be sized, and the beam's basis found, in the headset. A rebuild is
    // ~70 s now, so "try 5, try 8" as a rebuild loop costs minutes to answer what the eye settles
    // in seconds -- and finding a sign flip by rebuilding is the worst version of that.
    if (_stricmp(key, "grabguidelabelcm")  == 0) { g_cfg.grab_guide_label_cm = clampf((float)v, 0.5f, 40.0f); return true; }
    if (_stricmp(key, "grabguidemode")     == 0) { g_cfg.grab_guide_mode       = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "grabguidebeambasis")== 0) { g_cfg.grab_guide_beam_basis = (int)clampf((float)v, 0.0f, 3.0f); return true; }
    // Beam thickness -- only has an effect in grabguidemode=1; the label is square.
    if (_stricmp(key, "grabguidethick") == 0) { g_cfg.grab_guide_thick_cm = clampf((float)v, 0.05f, 10.0f); return true; }
    if (_stricmp(key, "grabguidemin")   == 0) { g_cfg.grab_guide_min_cm   = clampf((float)v, 0.0f, 50.0f); return true; }
    if (_stricmp(key, "grabguidemat")   == 0) {
        if (val == nullptr) return true;
        strncpy_s(g_cfg.grab_guide_mat, sizeof(g_cfg.grab_guide_mat), val, _TRUNCATE);
        size_t n = strlen(g_cfg.grab_guide_mat);
        while (n > 0) {
            const unsigned char c = (unsigned char)g_cfg.grab_guide_mat[n - 1];
            if (c != 13 && c != 10 && c != 32 && c != 9) break;
            g_cfg.grab_guide_mat[--n] = 0;
        }
        return true;
    }
    return false;
}

void interact_line_update(API::UObject* rig, const Vec3& hand_rel, const Vec3& target_rel,
                          bool show, float alpha) {
    if (!g_cfg.grab_guide) {
        auto* c = g_line.get_checked(L"StaticMeshComponent");
        if (c != nullptr) set_visible(c, false);
        return;
    }
    if (rig == nullptr || !ensure(rig)) return;

    auto* comp = g_line.get_checked(L"StaticMeshComponent");
    if (comp == nullptr) {
        // The pawn went away with the component on it. Drop the handle and let the next tick
        // rebuild against the new one, exactly as the reticule mesh does.
        interact_line_release();
        return;
    }
    if (!show) { set_visible(comp, false); return; }

    // The rig's world position anchors both endpoints, which arrive as offsets from it in the
    // gun's frame. One reflected read, and only while the guide is actually visible.
    Vec3 origin{};
    if (!call_ret_vec3(rig, L"K2_GetComponentLocation", &origin)) { set_visible(comp, false); return; }
    const Vec3 a{origin.x + hand_rel.x,   origin.y + hand_rel.y,   origin.z + hand_rel.z};
    const Vec3 b{origin.x + target_rel.x, origin.y + target_rel.y, origin.z + target_rel.z};

    const float dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    const float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    // Below this the beam is shorter than it is thick and reads as a blob sitting on the barrel,
    // which is worse than nothing -- and it is exactly the state you are in when the grab is
    // perfect. Hiding it there is the right answer: you no longer need to be told.
    if (len < g_cfg.grab_guide_min_cm) { set_visible(comp, false); return; }

    set_visible(comp, true);

    { // Midpoint -- the cube is centred on its origin. RELATIVE to the rig we are attached to.
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        auto* d = reinterpret_cast<double*>(p);
        d[0] = (double)(a.x + dx * 0.5f);
        d[1] = (double)(a.y + dy * 0.5f);
        d[2] = (double)(a.z + dz * 0.5f);
        *reinterpret_cast<int32_t*>(p + 40) = 2;   // ETeleportType::ResetPhysics
        comp->call_function(L"K2_SetWorldLocation", p);
    }
    { // Point local +X down the line. Rotator order in this ABI is (pitch, yaw, roll).
        const float fh = std::sqrt(dx * dx + dy * dy);
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        auto* d = reinterpret_cast<double*>(p);
        d[0] = (double)(std::atan2(dz, fh) * RAD2DEG_F);
        d[1] = (double)(std::atan2(dy, dx) * RAD2DEG_F);
        d[2] = 0.0;
        comp->call_function(L"K2_SetWorldRotation", p);
    }
    { // Cube is 100 cm on a side, so a unit of scale is a metre of bar.
        const double t = (double)(g_cfg.grab_guide_thick_cm / 100.0f);
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        auto* d = reinterpret_cast<double*>(p);
        d[0] = (double)(len / 100.0f); d[1] = t; d[2] = t;
        comp->call_function(L"SetWorldScale3D", p);
    }

    // COLOUR, pushed only when it CHANGES -- the live tunables make it movable, but it moves a few
    // times a session, not every frame, and this lane sits inside a VR frame budget.
    //
    // Several candidate parameter names for the same reason reticule_mesh_color() tries several:
    // which one BasicShapeMaterial exposes is not worth one build per guess, and writing a name
    // the material does not have is a harmless stored override.
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    const float want[4] = {g_cfg.grab_guide_r, g_cfg.grab_guide_g, g_cfg.grab_guide_b, alpha};
    const bool changed = std::fabs(want[0] - g_col_sent[0]) > 0.005f
                      || std::fabs(want[1] - g_col_sent[1]) > 0.005f
                      || std::fabs(want[2] - g_col_sent[2]) > 0.005f
                      || std::fabs(want[3] - g_col_sent[3]) > 0.02f;
    if (changed) {
        if (auto* mid = g_mid.get_checked(L"MaterialInstanceDynamic")) {
            for (int i = 0; i < 4; ++i) g_col_sent[i] = want[i];
            // EmissiveColor FIRST, and it was the omission that made the beam look unlit: the
            // material we bind is EmissiveMeshMaterial, whose parameter is emissive, and this list
            // was copied from the reticule's LIT material where the name is Color. The material
            // resolved fine, the colour simply never landed on the channel that glows.
            //
            // Writing a name a material does not have is a harmless stored override (see
            // reticule_mesh_color), so trying all of them costs one call each and converges in a
            // single run rather than one build per guess.
            static const wchar_t* kNames[] = { L"EmissiveColor", L"Emissive", L"Color",
                                               L"BaseColor", L"Tint", L"Albedo", L"DiffuseColor" };
            for (const wchar_t* name : kNames) {
                alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
                API::FName param = make_fname(name);
                memcpy(q, &param, sizeof(int32_t) * 2);
                auto* c = reinterpret_cast<float*>(q + 8);
                c[0] = want[0]; c[1] = want[1]; c[2] = want[2]; c[3] = want[3];
                mid->call_function(L"SetVectorParameterValue", q);
            }
        }
    }
}

} // namespace halo
