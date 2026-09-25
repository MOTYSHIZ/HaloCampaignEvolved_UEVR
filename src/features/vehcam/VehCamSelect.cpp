#include "features/vehcam/VehCamSelect.hpp"

#include "Config.hpp"
#include "features/vehcam/VehCam.hpp"          // g_veh_tp_active
#include "features/vehcam/VehCamPresets.hpp"
#include "uevr/API.hpp"

#include <Windows.h>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>

using uevr::API;

namespace halo {

namespace vcp = vehcampresets;

namespace {

// ---- what the other threads read ---------------------------------------------------------------
VehActiveCam     s_active[2];
std::atomic<int> s_active_front{0};
std::atomic<int> s_step{0};                  // left X / left Y, posted by the input hook

void publish(const VehActiveCam& a) {
    const int back = s_active_front.load(std::memory_order_relaxed) ^ 1;
    s_active[back] = a;
    s_active_front.store(back, std::memory_order_release);
}

// ---- the table and the selection: GAME THREAD only ---------------------------------------------
vcp::Table         s_table = vcp::default_table();   // the built-in cameras until the file loads
bool               s_table_changed = false;         // the poll replaced it; the tick re-applies
char               s_path[MAX_PATH]{};
unsigned long long s_stamp = ~0ull;                 // last write time seen; ~0 = never looked

uintptr_t          s_chassis = 0;                   // the vehicle mesh the selection was made for
int                s_vehicle = -1;                  // index into s_table.vehicles; -1 = none
int                s_camera = 0;
std::string        s_vehicle_name;                  // the entry's name (a reload may reorder entries)
std::map<std::string, int> s_remembered;            // entry name -> the camera you last used, this session

// ---- UEVR's decoupled pitch --------------------------------------------------------------------
bool     s_dp_forced = false;                       // we turned VR_DecoupledPitch off
bool     s_dp_want = false;                         // last request, so only CHANGES are acted on
char     s_dp_saved[16]{};
char     s_dp_marker[MAX_PATH]{};
uint32_t s_ticks = 0;

unsigned long long file_stamp(const char* path) {
    WIN32_FILE_ATTRIBUTE_DATA fa{};
    if (!GetFileAttributesExA(path, GetFileExInfoStandard, &fa)) return 0;   // 0 = absent
    return (static_cast<unsigned long long>(fa.ftLastWriteTime.dwHighDateTime) << 32) |
           fa.ftLastWriteTime.dwLowDateTime;
}

std::string narrow(const std::wstring& w) {
    std::string s;
    s.reserve(w.size());
    for (const wchar_t c : w) s.push_back((c > 0 && c < 0x80) ? static_cast<char>(c) : '?');
    return s;
}

VehActiveCam make_active(int vi, int ci) {
    const vcp::Vehicle& v = s_table.vehicles[vi];
    const vcp::Camera& c = v.cameras[ci];
    VehActiveCam a;
    a.valid = true;
    a.type = static_cast<uint8_t>(c.type);
    a.origin = static_cast<uint8_t>(c.origin);
    a.rides = static_cast<uint8_t>(c.rides);
    a.follow_pitch = c.follow_pitch;
    a.follow_roll = c.follow_roll;
    // Tilting implies turning: a pitch or roll measured in a heading the view does not share would
    // tip the horizon about the wrong axis.
    a.follow_yaw = c.follow_yaw || c.follow_pitch || c.follow_roll;
    a.collide = c.collide;
    a.collide_margin = c.collide_margin;
    a.hide_body = c.hides_body();
    for (int k = 0; k < 3; ++k) a.offset[k] = c.offset[k];
    a.motion_aim = v.motion_aim;
    a.index = ci;
    a.count = static_cast<int>(v.cameras.size());
    return a;
}

void clear_selection() {
    if (s_vehicle >= 0) API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: out of the vehicle -- camera released");
    s_vehicle = -1;
    s_vehicle_name.clear();
    publish(VehActiveCam{});
    g_veh_tp_active.store(false, std::memory_order_relaxed);
}

void select(int vi, int ci, const char* why) {
    const vcp::Vehicle& v = s_table.vehicles[vi];
    const int n = static_cast<int>(v.cameras.size());
    if (n <= 0) { clear_selection(); return; }
    ci = ((ci % n) + n) % n;
    s_vehicle = vi;
    s_vehicle_name = v.name;
    s_camera = ci;
    s_remembered[v.name] = ci;
    const VehActiveCam a = make_active(vi, ci);
    publish(a);
    // Our camera draws only for a chase camera; a first-person entry hands the view to the seat camera.
    g_veh_tp_active.store(g_cfg.veh_tp && a.type == static_cast<uint8_t>(vcp::CamType::Chase),
                          std::memory_order_relaxed);
    const vcp::Camera& c = v.cameras[ci];
    std::string follows;
    if (a.follow_yaw)   follows += "yaw ";
    if (a.follow_pitch) follows += "pitch ";
    if (a.follow_roll)  follows += "roll ";
    if (follows.empty()) follows = "nothing ";
    API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: %s -- camera %d/%d \"%s\" (%s): %s, origin %s, "
                         "offset (%.0f %.0f %.0f), rides %s, view follows %s, body %s",
                         v.name.c_str(), ci + 1, n, c.name.c_str(), why, vcp::type_name(c.type),
                         vcp::origin_name(c.origin), c.offset[0], c.offset[1], c.offset[2],
                         vcp::rides_name(c.rides), follows.c_str(), a.hide_body ? "hidden" : "shown");
}

// UEVR FLATTENS the view to its yaw right after our callback while VR_DecoupledPitch is on
// (FFakeStereoRenderingHook.cpp: `if (vr->is_decoupled_pitch_enabled()) vqi_norm = flatten(vqi_norm)`),
// so a camera that pitches or rolls with the vehicle needs it OFF. Only while OUR camera owns the whole
// view rotation, though: on foot it is what keeps the game's aim pitch out of the headset. So it is
// forced off only while such a camera is up, only if it was on, and put back on the way out. A marker
// file records the promise so a crash mid-ride is repaired at the next start.
void set_decoupled_pitch(const char* v) {
    if (auto* p = API::get()->param(); p != nullptr && p->vr != nullptr && p->vr->set_mod_value != nullptr)
        p->vr->set_mod_value("VR_DecoupledPitch", v);
}
bool read_decoupled_pitch(char* out, size_t n) {
    out[0] = 0;
    auto* p = API::get()->param();
    if (p == nullptr || p->vr == nullptr || p->vr->get_mod_value == nullptr) return false;
    p->vr->get_mod_value("VR_DecoupledPitch", out, static_cast<int>(n));
    return out[0] != 0;
}

void decoupled_pitch_update(bool want_off) {
    if (want_off != s_dp_want) {
        s_dp_want = want_off;
        if (want_off && !s_dp_forced) {
            char cur[16]{};
            if (read_decoupled_pitch(cur, sizeof(cur)) && std::strcmp(cur, "true") == 0) {
                strcpy_s(s_dp_saved, sizeof(s_dp_saved), cur);
                if (s_dp_marker[0] != 0) {
                    FILE* f = nullptr;
                    if (fopen_s(&f, s_dp_marker, "wb") == 0 && f != nullptr) { fputs(cur, f); fclose(f); }
                }
                set_decoupled_pitch("false");
                s_dp_forced = true;
                API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: VR_DecoupledPitch -> false while this camera "
                                     "tilts with the vehicle (restored when it stops)");
            }
        } else if (!want_off && s_dp_forced) {
            set_decoupled_pitch(s_dp_saved[0] != 0 ? s_dp_saved : "true");
            if (s_dp_marker[0] != 0) DeleteFileA(s_dp_marker);
            s_dp_forced = false;
            API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: VR_DecoupledPitch -> %s (restored)",
                                 s_dp_saved[0] != 0 ? s_dp_saved : "true");
        }
    }
    // Re-asserted every ~10 s while forced: anything that reloads UEVR's config would otherwise put it
    // back on silently, and the only symptom would be a view that stops tilting.
    if (s_dp_forced && (s_ticks % 300u) == 0u) {
        char cur[16]{};
        if (read_decoupled_pitch(cur, sizeof(cur)) && std::strcmp(cur, "false") != 0) set_decoupled_pitch("false");
    }
}

// Once, when UEVR's mod values are ready (the same ~10 s the cutscene startup check waits): a marker
// left behind means a previous session forced decoupled pitch off and never got to put it back.
void decoupled_pitch_startup_check() {
    if (s_dp_marker[0] == 0 || file_stamp(s_dp_marker) == 0) return;
    char saved[16]{};
    FILE* f = nullptr;
    if (fopen_s(&f, s_dp_marker, "rb") == 0 && f != nullptr) {
        const size_t n = fread(saved, 1, sizeof(saved) - 1, f);
        saved[n] = 0;
        fclose(f);
    }
    const char* v = (std::strcmp(saved, "true") == 0 || std::strcmp(saved, "false") == 0) ? saved : "true";
    set_decoupled_pitch(v);
    DeleteFileA(s_dp_marker);
    API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: restored VR_DecoupledPitch=%s -- a previous session turned it "
                         "off for a tilting vehicle camera and ended before putting it back", v);
}

} // namespace

VehActiveCam veh_active_cam() {
    return s_active[s_active_front.load(std::memory_order_acquire)];
}

void veh_cam_step(int dir) {
    s_step.fetch_add(dir > 0 ? 1 : -1, std::memory_order_relaxed);
}

void vehcam_presets_init(const char* path) {
    if (path == nullptr || path[0] == 0) return;
    strcpy_s(s_path, sizeof(s_path), path);
    // The decoupled-pitch marker lives beside the file.
    strcpy_s(s_dp_marker, sizeof(s_dp_marker), path);
    if (char* dot = std::strrchr(s_dp_marker, '.')) *dot = 0;
    strcat_s(s_dp_marker, sizeof(s_dp_marker), ".decoupledpitch.restore");
    // Written ONCE, from the built-in table, when the player has no file. Never touched again: it is
    // theirs (user-owned, never shipped). 'wx' refuses to open a file that already exists.
    FILE* f = nullptr;
    if (fopen_s(&f, s_path, "wx") == 0 && f != nullptr) {
        const std::string text = vcp::table_to_json(vcp::default_table());
        fwrite(text.data(), 1, text.size(), f);
        fclose(f);
        API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: wrote the built-in cameras to %s", s_path);
    }
    vehcam_presets_poll();
}

void vehcam_presets_poll() {
    if (s_path[0] == 0) return;
    const unsigned long long stamp = file_stamp(s_path);
    if (stamp == s_stamp) return;              // one stat call per poll; the file is read on change only
    s_stamp = stamp;
    if (stamp == 0) {
        s_table = vcp::default_table();
        s_table_changed = true;
        API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: no %s -- using the built-in cameras", s_path);
        return;
    }
    std::string text;
    FILE* f = nullptr;
    if (fopen_s(&f, s_path, "rb") != 0 || f == nullptr) {
        s_stamp = ~0ull;                       // an editor may hold it mid-save: try again next poll
        return;
    }
    char buf[4096];
    size_t n = 0;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0 && text.size() < (256u << 10)) text.append(buf, n);
    fclose(f);
    vcp::Table t;
    const vcp::ParseResult r = vcp::table_from_json(text.c_str(), text.size(), t);
    if (!r.ok) {
        // Keep what is running: a half-typed edit must not drop you out of your camera.
        API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: %s NOT applied -- %s. The previous cameras stay in "
                             "use until the file reads cleanly.", s_path, r.error.c_str());
        return;
    }
    s_table = std::move(t);
    s_table_changed = true;
    std::string ign;
    for (const std::string& k : r.ignored) { if (!ign.empty()) ign += ", "; ign += k; }
    API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: loaded %d vehicle(s), %d camera(s) from %s%s%s",
                         r.vehicles, r.cameras, s_path,
                         ign.empty() ? "" : " -- IGNORED (no such setting): ", ign.c_str());
}

void vehcam_select_tick(bool in_vehicle, uintptr_t chassis, const std::wstring& chassis_name) {
    ++s_ticks;
    if (s_ticks == 300) decoupled_pitch_startup_check();

    if (!in_vehicle || chassis == 0) {
        // Out of the vehicle (or not identified yet): nothing selected, nothing forced, and a step
        // pressed meanwhile does not carry into the next ride.
        if (!in_vehicle) { if (s_vehicle >= 0 || s_chassis != 0) clear_selection(); s_chassis = 0; }
        s_step.store(0, std::memory_order_relaxed);
        decoupled_pitch_update(false);
        return;
    }

    if (chassis != s_chassis || s_table_changed) {
        const bool reload = (chassis == s_chassis);
        const std::string prev = s_vehicle_name;
        s_chassis = chassis;
        s_table_changed = false;
        const std::string actor = narrow(chassis_name);
        const int vi = vcp::match_vehicle(s_table, actor);
        if (vi < 0) {
            clear_selection();
            API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: no camera entry matches %s and there is no "
                                 "\"default\" entry -- the game's camera stays", actor.c_str());
        } else {
            const vcp::Vehicle& v = s_table.vehicles[vi];
            int ci = v.default_camera;
            if (reload && prev == v.name) {
                ci = s_camera;                       // an edit keeps you in the camera you are in
            } else if (auto it = s_remembered.find(v.name); it != s_remembered.end()) {
                ci = it->second;                     // back in a vehicle you used: the camera you left it in
            }
            select(vi, ci, reload ? "file reloaded" : "entered");
        }
    }

    const int step = s_step.exchange(0, std::memory_order_relaxed);
    if (step != 0 && s_vehicle >= 0)
        select(s_vehicle, s_camera + step, step > 0 ? "left X: next" : "left Y: previous");

    const VehActiveCam a = veh_active_cam();
    decoupled_pitch_update(a.valid && g_veh_tp_active.load(std::memory_order_relaxed)
                           && (a.follow_pitch || a.follow_roll));
}

} // namespace halo
