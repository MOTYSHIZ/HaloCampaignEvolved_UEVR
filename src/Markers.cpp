// WORLD-SPACE MARKERS -- the visual layer for physical interactions (grenade pouches, the
// in-hand grenade; later gear). One proven recipe: a StaticMeshComponent on an existing actor,
// absolute transform, no collision, no shadow, scale re-applied by the caller every tick (a live
// cfg value must be re-applied on the cadence it can change). A component is only ever built from
// an ALREADY-RESOLVED mesh: null mesh = no component = the caller retries, so an invisible ghost
// can never be cached.
//
// room_to_world is the same transform that puts the rendered weapon on the hand: relative to the
// HMD, room->UE axis swizzle, rotated by the rendered base yaw, anchored on the rendered camera.
// A marker placed with it lands on the controller by construction.

#include "Markers.hpp"

#include "Config.hpp"
#include "Math.hpp"
#include "Rig.hpp"
#include "UeObject.hpp"
#include "MotionAimControl.hpp"   // get_pose: the render pass needs the frame's own head pose

#include <cmath>

using uevr::API;

namespace halo {

// The rendered camera position (UE cm), published by the stereo callback in Plugin.cpp.
std::atomic<float> g_cam_x{0.0f}, g_cam_y{0.0f}, g_cam_z{0.0f};

// The rendered base yaw, published each frame by the view callback in Plugin.cpp. Defined HERE
// rather than there because Plugin.cpp keeps its globals in an anonymous namespace, and this one
// must have external linkage at namespace-halo scope for Markers/Holster to share it.
std::atomic<float> g_view_base_yaw{0.0f};

namespace {

// Build a marker from an ALREADY-RESOLVED mesh. Split out of the name-list version because the
// holster learned the hard way that /Engine/BasicShapes is not reliably loaded outside vehicles:
// three "spheres" spawned meshless and invisible, were cached, and never retried. A component is
// only worth caching WITH a mesh, so mesh resolution now happens before the component exists and
// a null mesh means no component at all -- the caller retries instead of keeping a ghost.
API::UObject* spawn_marker_mesh(API::UObject* owner, API::UObject* mesh, double sx, double sy, double sz) {
    if (mesh == nullptr || owner == nullptr) return nullptr;
    auto* smc_cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.StaticMeshComponent");
    if (smc_cls == nullptr) return nullptr;
    auto* comp = API::get()->add_component_by_class(owner, smc_cls, false);
    if (comp == nullptr) return nullptr;
    {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<void**>(p) = mesh;
        comp->call_function(L"SetStaticMesh", p);
    }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 0; comp->call_function(L"SetCollisionEnabled", p); }
    // ABSOLUTE WORLD PLACEMENT. Attaching to the hull was tried and SILENTLY FAILED --
    // K2_AttachToComponent reported nothing, the components kept no parent, and the hull-local
    // coordinates then landed as WORLD coordinates at the map origin: disc gone, dots frozen.
    // Absolute placement is the mechanism that demonstrably worked, so the frame fix rides on it
    // rather than on an untested attach. One change at a time.
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; p[0] = 1; p[1] = 1; p[2] = 1; comp->call_function(L"SetAbsolute", p); }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; comp->call_function(L"SetCastShadow", p); }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      auto* d = reinterpret_cast<double*>(p); d[0] = sx; d[1] = sy; d[2] = sz;
      comp->call_function(L"SetWorldScale3D", p); }
    return comp;
}

// The wheel's name-list front end, now a resolve step over the mesh-first builder.
API::UObject* spawn_marker(API::UObject* owner, const wchar_t* const* meshes, int nmesh, double sx, double sy, double sz) {
    API::UObject* mesh = nullptr;
    const wchar_t* mesh_used = L"(NONE -- not spawning an invisible component)";
    for (int i = 0; i < nmesh && mesh == nullptr; ++i) {
        mesh = API::get()->find_uobject<API::UObject>(meshes[i]);
        if (mesh != nullptr) mesh_used = meshes[i];
    }
    API::get()->log_info("[Halo-CampE-UEVR] WHEEL: marker mesh %ls", mesh_used);
    return spawn_marker_mesh(owner, mesh, sx, sy, sz);
}

// Scale applied EVERY TICK, not once at spawn. It was set only in spawn_marker, so changing
// vehwheelrad live moved the invisible grab zone while the disc kept the size it was created at --
// the radius was tuned up four times in the headset against a mesh that could not change, which is the worst
// kind of dial: it answers, but never to the thing you are looking at. Anything a live cfg value
// controls has to be re-applied on the same cadence the value can change.
void marker_scale(API::UObject* comp, double sx, double sy, double sz) {
    if (comp == nullptr) return;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    auto* d = reinterpret_cast<double*>(p);
    d[0] = sx; d[1] = sy; d[2] = sz;
    comp->call_function(L"SetWorldScale3D", p);
}

void marker_place(API::UObject* comp, const Vec3& world, float pitch, float yaw) {
    if (comp == nullptr) return;
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      auto* d = reinterpret_cast<double*>(p);
      d[0] = world.x; d[1] = world.y; d[2] = world.z;
      comp->call_function(L"K2_SetWorldLocation", p); }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      auto* d = reinterpret_cast<double*>(p);
      d[0] = pitch; d[1] = yaw; d[2] = 0.0;
      comp->call_function(L"K2_SetWorldRotation", p); }
}

// Hull-local -> world, and the hull's yaw with it. The inverse of world_to_hull_local, using the
// same basis, so a constant in the hog's frame renders at the right world point without any head
// term -- which is the whole fix, independent of how the component is placed.
bool hull_local_to_world(API::UObject* hull, const Vec3& local, Vec3* out, float* hull_yaw) {
    Vec3 hloc{}, hrot{};
    if (!call_ret_vec3(hull, L"K2_GetComponentLocation", &hloc)) return false;
    if (!call_ret_vec3(hull, L"K2_GetComponentRotation", &hrot)) return false;
    const float D2R = 0.01745329252f;
    const float cp = std::cos(hrot.x * D2R), sp = std::sin(hrot.x * D2R);
    const float cy = std::cos(hrot.y * D2R), sy = std::sin(hrot.y * D2R);
    const float cr = std::cos(hrot.z * D2R), sr = std::sin(hrot.z * D2R);
    const float ax[3] = { cp * cy, cp * sy, sp };
    const float ay[3] = { sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp };
    const float az[3] = { -(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp };
    out->x = hloc.x + ax[0] * local.x + ay[0] * local.y + az[0] * local.z;
    out->y = hloc.y + ax[1] * local.x + ay[1] * local.y + az[1] * local.z;
    out->z = hloc.z + ax[2] * local.x + ay[2] * local.y + az[2] * local.z;
    if (hull_yaw != nullptr) *hull_yaw = hrot.y;
    return true;
}

// World point -> the hull's local frame, using the hull transform read THIS tick. Because the
// camera and the hull are read at the same instant, the difference between them is the frozen
// seat offset and the result is stable regardless of when the tick lands.
bool world_to_hull_local(API::UObject* hull, const Vec3& world, Vec3* out) {
    Vec3 hloc{}, hrot{};
    if (!call_ret_vec3(hull, L"K2_GetComponentLocation", &hloc)) return false;
    if (!call_ret_vec3(hull, L"K2_GetComponentRotation", &hrot)) return false;
    const float D2R = 0.01745329252f;
    const float cp = std::cos(hrot.x * D2R), sp = std::sin(hrot.x * D2R);
    const float cy = std::cos(hrot.y * D2R), sy = std::sin(hrot.y * D2R);
    const float cr = std::cos(hrot.z * D2R), sr = std::sin(hrot.z * D2R);
    const float ax[3] = { cp * cy, cp * sy, sp };
    const float ay[3] = { sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp };
    const float az[3] = { -(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp };
    const float dx = world.x - hloc.x, dy = world.y - hloc.y, dz = world.z - hloc.z;
    out->x = ax[0] * dx + ax[1] * dy + ax[2] * dz;
    out->y = ay[0] * dx + ay[1] * dy + ay[2] * dz;
    out->z = az[0] * dx + az[1] * dy + az[2] * dz;
    return true;
}

void marker_show(API::UObject* comp, bool show) {
    if (comp == nullptr) return;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    p[0] = show ? 0 : 1;
    comp->call_function(L"SetHiddenInGame", p);
}


// Room point (UEVR tracking space, metres) -> UE world cm, via the palette's transform. The
// swizzle (-z, x, y) is the room->UE axis map Plugin.cpp uses for head_ue/hand_ue.
Vec3 room_to_world_at(const Vec3& room, const Vec3& hmd_room, const Vec3& cam) {
    // The anchor: the standing origin (what the head's rendered offset is measured from), or
    // the HMD for the old behaviour. See Config.hpp room_anchor.
    Vec3 anchor = hmd_room;
    if (g_cfg.room_anchor == 1) { const auto so = API::VR::get_standing_origin(); anchor = Vec3{so.x, so.y, so.z}; }
    const Vec3 rel_ue{-(room.z - anchor.z) * 100.0f,
                       (room.x - anchor.x) * 100.0f,
                       (room.y - anchor.y) * 100.0f};
    const Quat yawq = rotator_to_quat(0.0f, g_view_base_yaw.load(std::memory_order_relaxed), 0.0f);
    const Vec3 w = quat_rotate(yawq, rel_ue);
    return Vec3{cam.x + w.x, cam.y + w.y, cam.z + w.z};
}
Vec3 room_to_world(const Vec3& room, const Vec3& hmd_room) {
    return room_to_world_at(room, hmd_room,
                            Vec3{g_cam_x.load(std::memory_order_relaxed),
                                 g_cam_y.load(std::memory_order_relaxed),
                                 g_cam_z.load(std::memory_order_relaxed)});
}


} // namespace

void marker_tint(API::UObject* comp, const char* rgb) {
    if (!g_cfg.marker_tint_on || comp == nullptr || rgb == nullptr || rgb[0] == 0) return;
    float r = 1.0f, g = 1.0f, b = 1.0f;
    if (sscanf_s(rgb, "%f,%f,%f", &r, &g, &b) != 3) return;
    API::UObject* mid = nullptr;
    {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<int32_t*>(p) = 0;          // ElementIndex
        *reinterpret_cast<void**>(p + 8) = nullptr;  // SourceMaterial: the mesh's own
        comp->call_function(L"CreateDynamicMaterialInstance", p);
        mid = *reinterpret_cast<API::UObject**>(p + 24);
    }
    if (mid == nullptr || IsBadReadPtr(mid, sizeof(void*))) { API::get()->log_info("[Halo-CampE-UEVR] MARKER tint: no dynamic material instance for %ls", class_name_of(comp).c_str()); return; }
    static const wchar_t* kNames[] = { L"Color", L"BaseColor", L"Tint", L"Albedo", L"DiffuseColor" };
    for (const wchar_t* name : kNames) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        API::FName param = make_fname(name);
        memcpy(p, &param, sizeof(int32_t) * 2);
        auto* col = reinterpret_cast<float*>(p + 8);
        col[0] = r; col[1] = g; col[2] = b; col[3] = 1.0f;
        mid->call_function(L"SetVectorParameterValue", p);
    }
}
API::UObject* holster_marker_spawn_mesh(API::UObject* owner, API::UObject* mesh, double scale) {
    return spawn_marker_mesh(owner, mesh, scale, scale, scale);
}
void holster_marker_set_mesh(API::UObject* comp, API::UObject* mesh) {
    if (comp == nullptr || mesh == nullptr) return;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    *reinterpret_cast<void**>(p) = mesh;
    comp->call_function(L"SetStaticMesh", p);
}
void holster_marker_place(API::UObject* comp, const Vec3& world) { marker_place(comp, world, 0.0f, 0.0f); }
// Full-rotation placement, for markers that must follow a HAND's orientation (the reload mag).
// marker_place zeroes roll by design -- fine for spheres, wrong for anything with an axis.
void holster_marker_place_rot(API::UObject* comp, const Vec3& world, float pitch, float yaw, float roll) {
    if (comp == nullptr) return;
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      auto* d = reinterpret_cast<double*>(p);
      d[0] = world.x; d[1] = world.y; d[2] = world.z;
      comp->call_function(L"K2_SetWorldLocation", p); }
    { alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
      auto* d = reinterpret_cast<double*>(p);
      d[0] = pitch; d[1] = yaw; d[2] = roll;
      comp->call_function(L"K2_SetWorldRotation", p); }
}
void holster_marker_show(API::UObject* comp, bool show) { marker_show(comp, show); }
void holster_marker_scale(API::UObject* comp, double s) { marker_scale(comp, s, s, s); }
Vec3 holster_room_to_world(const Vec3& room, const Vec3& hmd_room) { return room_to_world(room, hmd_room); }
Vec3 holster_room_to_world_at(const Vec3& room, const Vec3& hmd_room, const Vec3& cam) { return room_to_world_at(room, hmd_room, cam); }
// The inverse: UE world cm -> room metres, same yaw and swizzle undone.
Vec3 holster_world_to_room_at(const Vec3& world, const Vec3& hmd_room, const Vec3& cam) {
    Vec3 anchor = hmd_room;
    if (g_cfg.room_anchor == 1) { const auto so = API::VR::get_standing_origin(); anchor = Vec3{so.x, so.y, so.z}; }
    const Vec3 w{world.x - cam.x, world.y - cam.y, world.z - cam.z};
    const Quat yawq = rotator_to_quat(0.0f, g_view_base_yaw.load(std::memory_order_relaxed), 0.0f);
    const Vec3 rel_ue = quat_rotate(quat_conj(yawq), w);
    // rel_ue = (-(dz), dx, dy) * 100  ->  dx = rel_ue.y/100, dy = rel_ue.z/100, dz = -rel_ue.x/100
    return Vec3{anchor.x + rel_ue.y * 0.01f, anchor.y + rel_ue.z * 0.01f, anchor.z - rel_ue.x * 0.01f};
}
Vec3 holster_world_to_room(const Vec3& world, const Vec3& hmd_room) {
    return holster_world_to_room_at(world, hmd_room,
                                    Vec3{g_cam_x.load(std::memory_order_relaxed),
                                         g_cam_y.load(std::memory_order_relaxed),
                                         g_cam_z.load(std::memory_order_relaxed)});
}

// The vehicle wheel's faces (see the header). Thin by design, same as the holster's.
API::UObject* marker_spawn_list(API::UObject* owner, const wchar_t* const* meshes,
                                int nmesh, double sx, double sy, double sz) {
    return spawn_marker(owner, meshes, nmesh, sx, sy, sz);
}
void marker_place_rot(API::UObject* comp, const Vec3& world, float pitch, float yaw) {
    marker_place(comp, world, pitch, yaw);
}
void marker_scale3(API::UObject* comp, double sx, double sy, double sz) {
    marker_scale(comp, sx, sy, sz);
}
bool markers_hull_local_to_world(API::UObject* hull, const Vec3& local, Vec3* out, float* hull_yaw) {
    return hull_local_to_world(hull, local, out, hull_yaw);
}
bool markers_world_to_hull_local(API::UObject* hull, const Vec3& world, Vec3* out) {
    return world_to_hull_local(hull, world, out);
}


// ---- THE RENDER-ANCHOR REGISTRY (doctrine in the header). Sixteen slots; each holds the
// component (TrackedObject, so a recycled pointer reads back null instead of faulting), its
// room pose, and whether the rotation is meaningful. Writers are the game tick; the reader is
// the render thread; a torn float costs one frame of one marker.
namespace {
struct RegSlot {
    std::atomic<void*> key{nullptr};
    TrackedObject      obj;
    std::atomic<float> x{0.0f}, y{0.0f}, z{0.0f}, pd{0.0f}, yd{0.0f}, rd{0.0f};
    std::atomic<bool>  use_rot{false};
    std::atomic<bool>  active{false};
};
RegSlot s_reg[16];
RegSlot* reg_claim(API::UObject* comp) {
    if (comp == nullptr) return nullptr;
    RegSlot* free_slot = nullptr;
    for (auto& r : s_reg) {
        if (r.key.load(std::memory_order_relaxed) == comp) {
            if (r.obj.get() != comp) r.obj.set(comp);   // same address, new object: re-track
            return &r;
        }
        if (free_slot == nullptr && !r.active.load(std::memory_order_relaxed)) free_slot = &r;
    }
    if (free_slot == nullptr) return nullptr;   // full: the marker stays tick-placed, no harm
    free_slot->obj.set(comp);
    free_slot->key.store(comp, std::memory_order_relaxed);
    return free_slot;
}
void reg_store(RegSlot* r, const Vec3& room, bool use_rot, float pd, float yd, float rd) {
    r->x.store(room.x, std::memory_order_relaxed); r->y.store(room.y, std::memory_order_relaxed); r->z.store(room.z, std::memory_order_relaxed);
    r->pd.store(pd, std::memory_order_relaxed); r->yd.store(yd, std::memory_order_relaxed); r->rd.store(rd, std::memory_order_relaxed);
    r->use_rot.store(use_rot, std::memory_order_relaxed);
    r->active.store(true, std::memory_order_relaxed);
}
} // namespace
void marker_render_anchor(API::UObject* comp, const Vec3& room) {
    if (auto* r = reg_claim(comp)) reg_store(r, room, false, 0.0f, 0.0f, 0.0f);
}
void marker_render_anchor_rot(API::UObject* comp, const Vec3& room, float pitch, float yaw, float roll) {
    if (auto* r = reg_claim(comp)) reg_store(r, room, true, pitch, yaw, roll);
}
void marker_render_drop(API::UObject* comp) {
    if (comp == nullptr) return;
    for (auto& r : s_reg)
        if (r.key.load(std::memory_order_relaxed) == comp) { r.active.store(false, std::memory_order_relaxed); return; }
}
void markers_render_place() {
    if (g_cfg.mag_render == 0 || !(g_cfg.reload_vr || g_cfg.slide_vr)) return;   // fork reload family only
    Vec3 hp{}; Quat hr{};
    if (!get_pose(API::VR::get_hmd_index(), &hp, &hr, /*use_aim=*/false)) return;
    for (auto& r : s_reg) {
        if (!r.active.load(std::memory_order_relaxed)) continue;
        auto* m = r.obj.get();
        if (m == nullptr) { r.active.store(false, std::memory_order_relaxed); r.key.store(nullptr, std::memory_order_relaxed); continue; }
        const Vec3 room{r.x.load(std::memory_order_relaxed), r.y.load(std::memory_order_relaxed), r.z.load(std::memory_order_relaxed)};
        const Vec3 w = room_to_world(room, hp);
        if (r.use_rot.load(std::memory_order_relaxed))
            holster_marker_place_rot(m, w, r.pd.load(std::memory_order_relaxed), r.yd.load(std::memory_order_relaxed), r.rd.load(std::memory_order_relaxed));
        else
            holster_marker_place(m, w);
    }
}

} // namespace halo
