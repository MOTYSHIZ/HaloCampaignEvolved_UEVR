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
Vec3 room_to_world(const Vec3& room, const Vec3& hmd_room) {
    const Vec3 rel_ue{-(room.z - hmd_room.z) * 100.0f,
                       (room.x - hmd_room.x) * 100.0f,
                       (room.y - hmd_room.y) * 100.0f};
    const Quat yawq = rotator_to_quat(0.0f, g_view_base_yaw.load(std::memory_order_relaxed), 0.0f);
    const Vec3 w = quat_rotate(yawq, rel_ue);
    return Vec3{g_cam_x.load(std::memory_order_relaxed) + w.x,
                g_cam_y.load(std::memory_order_relaxed) + w.y,
                g_cam_z.load(std::memory_order_relaxed) + w.z};
}


} // namespace

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

namespace {
Vec3 room_to_world(const Vec3& room, const Vec3& hmd_room);
bool world_to_hull_local(uevr::API::UObject* hull, const Vec3& world, Vec3* out);
void wheel_markers_update(bool active);
}


} // namespace halo
