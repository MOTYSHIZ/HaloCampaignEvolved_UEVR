#include "core/MarkerFaces.hpp"
#include "core/config/CfgRead.hpp"

#include "Config.hpp"
#include "Markers.hpp"            // the author's machinery: g_cam_*, g_view_base_yaw, holster_marker_*
#include "Math.hpp"
#include "MotionAimControl.hpp"   // get_pose: the render pass needs the frame's own head pose
#include "Rig.hpp"                // RIG_PARAM_BUF
#include "UeObject.hpp"
#include "core/Services.hpp"
#include "core/host/MarkersState.hpp"
#include "core/host/PluginState.hpp"   // the view position

#include <Windows.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>

using uevr::API;

namespace halo {

namespace {

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

} // namespace

bool room_to_world_anchored(const Vec3& room, const Vec3& hmd_room, Vec3* out) {
    if (!service_active(SVC_MARKER_ANCHOR) || g_cfg.room_anchor != 1) return false;
    *out = room_to_world_at(room, hmd_room,
                            Vec3{g_cam_x.load(std::memory_order_relaxed),
                                 g_cam_y.load(std::memory_order_relaxed),
                                 g_cam_z.load(std::memory_order_relaxed)});
    return true;
}

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
Vec3 holster_room_to_world_at(const Vec3& room, const Vec3& hmd_room, const Vec3& cam) { return room_to_world_at(room, hmd_room, cam); }

// The mesh sweep's backoff (see the header). No feature gate: it guards the author's own walk.
namespace { uint32_t s_mk_fails = 0; }   // empty surveys in a row
unsigned marker_sweep_period() { return (s_mk_fails < 5) ? 120u : 1200u; }
void marker_sweep_result(const void* mf) { if (mf == nullptr) ++s_mk_fails; else s_mk_fails = 0; }
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

// The vehicle wheel's faces (see the header). Thin by design, same as the holster's; Markers.cpp's own
// helpers are reached through the bridge.
API::UObject* marker_spawn_list(API::UObject* owner, const wchar_t* const* meshes,
                                int nmesh, double sx, double sy, double sz) {
    return host::g_markers_state.spawn_marker(owner, meshes, nmesh, sx, sy, sz);
}
void marker_place_rot(API::UObject* comp, const Vec3& world, float pitch, float yaw) {
    host::g_markers_state.marker_place(comp, world, pitch, yaw);
}
void marker_scale3(API::UObject* comp, double sx, double sy, double sz) {
    host::g_markers_state.marker_scale(comp, sx, sy, sz);
}
bool markers_hull_local_to_world(API::UObject* hull, const Vec3& local, Vec3* out, float* hull_yaw) {
    return host::g_markers_state.hull_local_to_world(hull, local, out, hull_yaw);
}
bool markers_world_to_hull_local(API::UObject* hull, const Vec3& world, Vec3* out) {
    return host::g_markers_state.world_to_hull_local(hull, world, out);
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
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    if (g_cfg.mag_render == 0 || !service_active(SVC_MARKER_ANCHOR)) return;   // a marker-anchoring feature is on
    Vec3 hp{}; Quat hr{};
    if (!get_pose(API::VR::get_hmd_index(), &hp, &hr, /*use_aim=*/false)) return;
    for (auto& r : s_reg) {
        if (!r.active.load(std::memory_order_relaxed)) continue;
        auto* m = r.obj.get();
        if (m == nullptr) { r.active.store(false, std::memory_order_relaxed); r.key.store(nullptr, std::memory_order_relaxed); continue; }
        const Vec3 room{r.x.load(std::memory_order_relaxed), r.y.load(std::memory_order_relaxed), r.z.load(std::memory_order_relaxed)};
        const Vec3 w = holster_room_to_world(room, hp);
        if (r.use_rot.load(std::memory_order_relaxed))
            holster_marker_place_rot(m, w, r.pd.load(std::memory_order_relaxed), r.yd.load(std::memory_order_relaxed), r.rd.load(std::memory_order_relaxed));
        else
            holster_marker_place(m, w);
    }
}

void marker_camera_publish() {
    const auto& ps = host::g_plugin_state;
    g_cam_x.store(ps.view_pos_x->load(std::memory_order_relaxed), std::memory_order_relaxed);
    g_cam_y.store(ps.view_pos_y->load(std::memory_order_relaxed), std::memory_order_relaxed);
    g_cam_z.store(ps.view_pos_z->load(std::memory_order_relaxed), std::memory_order_relaxed);
}

} // namespace halo
