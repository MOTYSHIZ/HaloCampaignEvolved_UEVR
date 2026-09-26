#pragma once

// PLUGIN.CPP'S FILE-LOCAL STATE THAT CORE AND FEATURES READ.
//
// Plugin.cpp keeps its state in an anonymous namespace, which no other translation unit can name.
// HALO_PLUGIN_STATE_BRIDGE, expanded once in Plugin.cpp right after that namespace closes, defines
// g_plugin_state as the addresses of the originals, so every read and write lands on exactly the
// object the code used when it was a textual fragment of Plugin.cpp. A user binds a reference to
// each object it uses under the object's own name, so the code reads the same as it did in place.
//
// Pointers, not reference members: MSVC 19.29 refuses constinit on an aggregate of references, and
// constinit is what guarantees the bridge is initialised statically -- valid from any thread from the
// moment the DLL is loaded, with no initialisation order against any other translation unit.

#include <atomic>
#include <cstdint>

#include "Math.hpp"       // Vec3
#include "UeObject.hpp"   // TrackedObject

namespace halo::host {

struct PluginState {
    std::atomic<bool>*  in_menu;            // g_in_menu
    std::atomic<bool>*  cut2d_engaged;      // g_cut2d_engaged
    std::atomic<float>* last_dt;            // g_last_dt
    std::atomic<bool>*  stick_mode;         // g_stick_mode
    std::atomic<bool>*  dpad_shift_active;  // g_dpad_shift_active
    short (*to_raw)(float v);               // to_raw()
    std::atomic<float>* view_pos_x;         // g_view_pos_x
    std::atomic<float>* view_pos_y;         // g_view_pos_y
    std::atomic<float>* view_pos_z;         // g_view_pos_z
    std::atomic<float>* dbg_view_in;        // g_dbg_view_in
    std::atomic<float>* dbg_view_out;       // g_dbg_view_out
    std::atomic<bool>*  lock_primed;        // g_lock_primed
    // ---- the nav lane (core/fixes/HostFixes: its fault quarantine)
    TrackedObject*      navw_pool;          // g_navw_pool[8]
    bool*               navw_mid_ok;        // g_navw_mid_ok[8]
    void**              navw_slot_class;    // g_navw_slot_class[8]
    std::atomic<int>*   navw_placed_n;      // g_navw_placed_n
    std::atomic<const char*>* navw_mark;    // g_navw_mark
    std::atomic<uint32_t>*    lane_faults;  // g_lane_faults[PERF_COUNT]
    int                 perf_navworld;      // PERF_NAVWORLD
    void (*nav_world_tick)(bool engaged, uint32_t tick);   // nav_world_tick()
    // ---- the aim reference and turning (the stick-mode exit fix, the turn instrument)
    std::atomic<bool>*  lock_ever;          // g_lock_ever
    std::atomic<bool>*  have_ref;           // g_have_ref
    std::atomic<float>* raw_stick_x;        // g_raw_stick_x
    bool*               fp_control_now;     // g_fp_control_now
    // ---- the rendered frame (the palette weapon's instruments)
    std::atomic<float>* locked_view_yaw;    // g_locked_view_yaw
    std::atomic<float>* eye_pos_x;          // g_eye_pos_x
    std::atomic<float>* eye_pos_y;          // g_eye_pos_y
    std::atomic<float>* eye_pos_z;          // g_eye_pos_z
    std::atomic<bool>*  have_eye_pos;       // g_have_eye_pos
    std::atomic<float>* render_view_yaw;    // g_render_view_yaw
    std::atomic<bool>*  have_view_pos;      // g_have_view_pos
    Vec3 (*layer_anchor)(int slot, const Vec3& world);   // layer_anchor()
    // onfoot_reticule_tick(): the on-foot reticule (the author's block, wrapped so the palette weapon can run it
    // with the rig driver off; see the report)
    void (*onfoot_reticule_tick)(uevr::API::UObject* rig, const Vec3& comp_world, double aim_yaw, double aim_pitch, uint32_t tick);
    // ---- the author's per-tick perf table (core/dev/DriverProbe reads it and never writes it). Filled only
    // while his perflog key is on; each entry is this tick's milliseconds in one PerfScope site.
    const double*       perf_now;           // g_perf_now[PERF_COUNT]
    int                 perf_count;         // PERF_COUNT
    int                 perf_tick;          // PERF_TICK, the whole tick
    int                 perf_palarm;        // PERF_PALARM, the palette arm driver's tick half
};

extern const PluginState g_plugin_state;

} // namespace halo::host

#define HALO_PLUGIN_STATE_BRIDGE                                          \
    constinit const ::halo::host::PluginState halo::host::g_plugin_state{ \
        &g_in_menu,                                                        \
        &g_cut2d_engaged,                                                  \
        &g_last_dt,                                                        \
        &g_stick_mode,                                                     \
        &g_dpad_shift_active,                                              \
        &to_raw,                                                           \
        &g_view_pos_x,                                                     \
        &g_view_pos_y,                                                     \
        &g_view_pos_z,                                                     \
        &g_dbg_view_in,                                                    \
        &g_dbg_view_out,                                                   \
        &g_lock_primed,                                                    \
        g_navw_pool,                                                       \
        g_navw_mid_ok,                                                     \
        g_navw_slot_class,                                                 \
        &g_navw_placed_n,                                                  \
        &g_navw_mark,                                                      \
        g_lane_faults,                                                     \
        PERF_NAVWORLD,                                                     \
        &nav_world_tick,                                                   \
        &g_lock_ever,                                                      \
        &g_have_ref,                                                       \
        &g_raw_stick_x,                                                    \
        &g_fp_control_now,                                                 \
        &g_locked_view_yaw,                                                \
        &g_eye_pos_x,                                                      \
        &g_eye_pos_y,                                                      \
        &g_eye_pos_z,                                                      \
        &g_have_eye_pos,                                                   \
        &g_render_view_yaw,                                                \
        &g_have_view_pos,                                                  \
        &layer_anchor,                                                     \
        &onfoot_reticule_tick,                                             \
        g_perf_now,                                                        \
        PERF_COUNT,                                                        \
        PERF_TICK,                                                         \
        PERF_PALARM,                                                       \
    };
