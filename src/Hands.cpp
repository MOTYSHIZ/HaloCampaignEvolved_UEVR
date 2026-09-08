#include "Hands.hpp"

#include "Config.hpp"
#include "Gesture.hpp"
#include "Math.hpp"
#include "Rig.hpp"
#include "UeObject.hpp"

#include <cstdint>
#include <cstring>
#include <string>

using uevr::API;

namespace halo {
namespace {

// One spawned, controller-attached mesh. Three of them: left hand, right hand, magazine.
struct HandPiece {
    TrackedObject comp;
    bool          attached = false;
    bool          visible  = true;
    bool          failed   = false;   // creation failed once; do not retry every tick forever
};

HandPiece s_left, s_right, s_mag;

// Pieces are stored by SIDE but used by ROLE, because which physical hand aims is a config
// choice. Resolving the mapping in one place is what keeps left-handed play working without a
// second code path.
HandPiece& s_aim_ref() { return g_cfg.aim_left_hand ? s_left  : s_right; }
HandPiece& s_off_ref() { return g_cfg.aim_left_hand ? s_right : s_left;  }

// SetVisibility(bool bNewVisibility, bool bPropagateToChildren).
void set_visible(API::UObject* c, bool v) {
    if (c == nullptr) return;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    p[0] = v ? 1 : 0;
    p[1] = 0;
    c->call_function(L"SetVisibility", p);
}

// SetRelativeScale3D(FVector NewScale3D). LWC: three doubles.
void set_scale(API::UObject* c, float s) {
    if (c == nullptr) return;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    auto* d = reinterpret_cast<double*>(p);
    d[0] = d[1] = d[2] = (double)s;
    c->call_function(L"SetRelativeScale3D", p);
}

// Engine primitives only. Taking a mesh out of the level would tie us to one map and risk
// borrowing something the game still wants; /Engine/BasicShapes ships with the engine itself.
// Each candidate is logged so a missing asset is visible rather than silently becoming a cube.
API::UObject* pick_mesh(const char* configured, const wchar_t* fallback, std::wstring* used) {
    // Whitespace-only counts as EMPTY. The config file is CRLF, and a bare "handmesh=" line was
    // arriving as a lone carriage return -- non-empty by the naive test, so it was looked up as an
    // asset path and logged as a missing mesh with a blank name. That is what the first live run
    // reported.
    auto blank = [](const char* s) {
        if (s == nullptr) return true;
        for (const char* p = s; *p; ++p) if (*p > ' ') return false;
        return true;
    };
    if (!blank(configured)) {
        const std::string a{configured};
        const std::wstring w(a.begin(), a.end());
        if (auto* m = API::get()->find_uobject<API::UObject>(w.c_str())) { if (used) *used = w; return m; }
        API::get()->log_info("[Halo-CampE-UEVR] HANDS configured mesh not found: %s", configured);
    }
    static const wchar_t* const kRoots[] = {
        L"StaticMesh /Engine/BasicShapes/", L"StaticMesh /Engine/EngineMeshes/",
    };
    for (const wchar_t* root : kRoots) {
        std::wstring path = std::wstring(root) + fallback + L"." + fallback;
        if (auto* m = API::get()->find_uobject<API::UObject>(path.c_str())) { if (used) *used = path; return m; }
        API::get()->log_info("[Halo-CampE-UEVR] HANDS   candidate miss: %ls", path.c_str());
    }
    return nullptr;
}

// Create a StaticMeshComponent on the pawn's actor and give it a mesh.
//
// A component must belong to an ACTOR -- the rig component's outer is that actor, which is the
// same route Reticule.cpp takes to spawn its mesh reticule.
bool ensure_piece(HandPiece& hp, const char* label, const char* cfg_path,
                  const wchar_t* fallback_shape, float scale) {
    if (hp.failed) return false;
    if (hp.comp.get() != nullptr) return true;

    auto* rig = rig_tracked_component();
    if (rig == nullptr) return false;   // no rig yet -- menus, vehicles, post-load. Retry next tick.
    auto* owner = rig->get_outer();
    if (owner == nullptr) return false;

    auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.StaticMeshComponent");
    if (cls == nullptr) { hp.failed = true; return false; }

    auto* comp = API::get()->add_component_by_class(owner, cls, false);
    if (comp == nullptr) {
        hp.failed = true;
        API::get()->log_info("[Halo-CampE-UEVR] HANDS %s: add_component_by_class FAILED", label);
        return false;
    }

    std::wstring used;
    if (auto* mesh = pick_mesh(cfg_path, fallback_shape, &used)) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<void**>(p) = mesh;
        comp->call_function(L"SetStaticMesh", p);
        API::get()->log_info("[Halo-CampE-UEVR] HANDS %s mesh: %ls", label, used.c_str());
    } else {
        API::get()->log_info("[Halo-CampE-UEVR] HANDS %s: no mesh asset found, component invisible",
                             label);
    }

    set_scale(comp, scale);
    hp.comp.set(comp);
    hp.attached = false;
    hp.visible = true;
    API::get()->log_info("[Halo-CampE-UEVR] HANDS %s created (scale %.3f)", label, scale);
    return hp.comp.get() != nullptr;
}

// Attach to a motion controller through UObjectHook -- the same call Rig.cpp's attach_apply uses.
// hand: 0 = left, 1 = right. Offsets are in centimetres, in the controller's own frame.
void attach_piece(HandPiece& hp, int hand, const Vec3& loc_cm, const Quat& rot) {
    auto* c = hp.comp.get();
    if (c == nullptr) return;
    auto* st = API::UObjectHook::get_or_add_motion_controller_state(c);
    if (st == nullptr) return;

    UEVR_Quaternionf q{rot.x, rot.y, rot.z, rot.w};
    UEVR_Vector3f    v{loc_cm.x, loc_cm.y, loc_cm.z};
    st->set_rotation_offset(&q);
    st->set_location_offset(&v);
    st->set_hand((uint32_t)hand);
    // NOT permanent: a permanent attachment outlives our control of it, and these components are
    // ours to clean up. attach_permanent exists for the rig, which the player calibrates once.
    st->set_permanent(false);
    if (!hp.attached) {
        API::get()->log_info("[Halo-CampE-UEVR] HANDS attached to controller %d, offset (%.1f %.1f %.1f) cm",
                             hand, loc_cm.x, loc_cm.y, loc_cm.z);
    }
    hp.attached = true;
}

void detach_piece(HandPiece& hp) {
    if (!hp.attached) return;
    if (auto* c = hp.comp.get()) API::UObjectHook::remove_motion_controller_state(c);
    hp.attached = false;
}

void show_piece(HandPiece& hp, bool v) {
    if (hp.visible == v) return;
    if (auto* c = hp.comp.get()) { set_visible(c, v); hp.visible = v; }
}

} // namespace

void hands_release() {
    for (HandPiece* hp : {&s_left, &s_right, &s_mag}) {
        detach_piece(*hp);
        show_piece(*hp, false);
    }
}

void hands_update() {
    if (!g_cfg.enabled || !g_cfg.hands_vr) {
        hands_release();
        return;
    }

    // Which controller is which. The aim hand holds the weapon, so the OFF hand is the one that
    // fetches magazines -- asking it that way keeps left-handed play working for free.
    const int aim_hand = g_cfg.aim_left_hand ? 0 : 1;
    const int off_hand = g_cfg.aim_left_hand ? 1 : 0;

    // ---- THE OFF HAND. Always shown: it is the one with nothing else to do, and the one the
    // reload gesture is performed with, so it is the hand that most needs to be visible.
    if (ensure_piece(s_off_ref(), "off hand", g_cfg.hand_mesh_path, L"Sphere", g_cfg.hand_scale)) {
        auto& hp = s_off_ref();
        attach_piece(hp, off_hand,
                     Vec3{g_cfg.hand_off_x, g_cfg.hand_off_y, g_cfg.hand_off_z},
                     Quat{0.0f, 0.0f, 0.0f, 1.0f});
        show_piece(hp, true);
    }

    // ---- THE AIM HAND. Optional: the weapon is already there and already tracks this controller,
    // so a mesh on top of it is often just clutter intersecting the gun.
    if (g_cfg.hand_show_aim) {
        if (ensure_piece(s_aim_ref(), "aim hand", g_cfg.hand_mesh_path, L"Sphere", g_cfg.hand_scale)) {
            auto& hp = s_aim_ref();
            attach_piece(hp, aim_hand,
                         Vec3{g_cfg.hand_off_x, g_cfg.hand_off_y, g_cfg.hand_off_z},
                         Quat{0.0f, 0.0f, 0.0f, 1.0f});
            show_piece(hp, true);
        }
    } else {
        detach_piece(s_aim_ref());
        show_piece(s_aim_ref(), false);
    }

    // ---- WHERE ARE THEY, ACTUALLY?
    //
    // Creation and attach both report success and nothing appears. Two very different causes:
    // the pieces track correctly and fail to render, or they render fine and are parked somewhere
    // you never look. Only a world position tells them apart -- and UObjectHook motion-controller
    // attachment was measured INERT on this title (both attachmode 1 and 2 left their target
    // frozen), which is the same call attach_piece makes, so "parked" is the likely answer.
    //
    // A position that does not move while you do is the confirmation.
    static uint32_t s_diag_tick = 0;
    if (g_cfg.hands_diag && (++s_diag_tick % 60) == 0) {
        struct { const char* n; HandPiece* p; } items[] = {
            {"off", &s_off_ref()}, {"aim", &s_aim_ref()}, {"mag", &s_mag} };
        for (auto& it : items) {
            auto* c = it.p->comp.get();
            if (c == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] HANDSPOS %s: <no component>", it.n); continue; }
            Vec3 w{};
            const bool ok = call_ret_vec3(c, L"K2_GetComponentLocation", &w);
            API::get()->log_info("[Halo-CampE-UEVR] HANDSPOS %s: world=(%.1f,%.1f,%.1f) attached=%d visible=%d%s",
                                 it.n, w.x, w.y, w.z, (int)it.p->attached, (int)it.p->visible,
                                 ok ? "" : "  (READ FAILED)");
        }
    }

    // ---- THE MAGAZINE. Visible only while the reload state machine says it is in your hand.
    //
    // Hidden rather than destroyed between reloads: add_component_by_class every reload would
    // churn objects on the game thread for no benefit, and a component we keep is a component we
    // can still find to clean up.
    if (g_cfg.mag_show) {
        if (ensure_piece(s_mag, "magazine", g_cfg.mag_mesh_path, L"Cube", g_cfg.mag_scale)) {
            const bool held = (reload_state() == ReloadState::MagHeld);
            if (held) {
                attach_piece(s_mag, off_hand,
                             Vec3{g_cfg.mag_off_x, g_cfg.mag_off_y, g_cfg.mag_off_z},
                             Quat{0.0f, 0.0f, 0.0f, 1.0f});
            } else {
                detach_piece(s_mag);
            }
            show_piece(s_mag, held);
        }
    }
}

} // namespace halo
