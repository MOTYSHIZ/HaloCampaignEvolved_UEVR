#include "features/vehcam/VehCamSelect.hpp"

#include "Config.hpp"
#include "XrText.hpp"                           // the camera readout on the text panel
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
std::atomic<int> s_ctrl_toggle{0};           // left stick click, posted by the input hook

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
// Entry name -> the controls you picked with the left stick click, this session -- kept WITH the default
// it was picked against, so a later edit of that default (the file's motionAim, or vehaim) wins over it:
// the edit is the newer statement of what you want.
struct CtrlChoice { bool motion; bool base; };
std::map<std::string, CtrlChoice> s_ctrl;
uint32_t           s_recenter_gen = 0;              // VehActiveCam::recenter_gen: bumped by select(..., recenter)

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

// What this vehicle starts in: its motionAim in the file, else the vehaim key.
bool base_motion(const vcp::Vehicle& v) { return v.motion_aim >= 0 ? v.motion_aim != 0 : g_cfg.veh_aim; }

// The left-stick choice for this vehicle: -1 = none (or its default has changed under it since, which
// drops it), else 0 = stick / 1 = motion.
int ctrl_choice(const vcp::Vehicle& v) {
    const auto it = s_ctrl.find(v.name);
    if (it == s_ctrl.end()) return -1;
    if (it->second.base != base_motion(v)) { s_ctrl.erase(it); return -1; }
    return it->second.motion ? 1 : 0;
}

// Whether the controller aims under this published camera. Only under one of OUR chase cameras: the
// pointing ray is built from the view our eye publishes, and a first-person entry hands the view to the
// seat camera, which publishes none -- so there it is stick controls whatever was chosen.
bool motion_on(const VehActiveCam& a) {
    if (a.type != static_cast<uint8_t>(vcp::CamType::Chase)) return false;
    return a.motion_aim >= 0 ? a.motion_aim != 0 : g_cfg.veh_aim;
}

VehActiveCam make_active(int vi, int ci) {
    const vcp::Vehicle& v = s_table.vehicles[vi];
    const vcp::Camera& c = v.cameras[ci];
    VehActiveCam a;
    a.valid = true;
    a.type = static_cast<uint8_t>(c.type);
    a.origin = static_cast<uint8_t>(c.origin);
    a.loc_yaw = c.loc_yaw; a.loc_pitch = c.loc_pitch; a.loc_roll = c.loc_roll; a.loc_view = c.loc_view;
    // Independent: pitch and roll without yaw tilt the view with the vehicle's deck while it keeps its
    // own heading (vehcammath::tracked_frame says exactly what that means).
    a.rot_yaw = c.rot_yaw; a.rot_pitch = c.rot_pitch; a.rot_roll = c.rot_roll;
    a.collide = c.collide;
    a.collide_margin = c.collide_margin;
    a.hide_body = c.hides_body();
    for (int k = 0; k < 3; ++k) a.offset[k] = c.offset[k];
    const int choice = ctrl_choice(v);                      // the left stick click beats the file
    a.motion_aim = choice >= 0 ? choice : v.motion_aim;
    a.aim_marker = v.aim_marker;
    a.index = ci;
    a.count = static_cast<int>(v.cameras.size());
    a.recenter_gen = s_recenter_gen;
    return a;
}

void clear_selection() {
    if (s_vehicle >= 0) API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: out of the vehicle -- camera released");
    s_vehicle = -1;
    s_vehicle_name.clear();
    publish(VehActiveCam{});
    g_veh_tp_active.store(false, std::memory_order_relaxed);
}

// recenter: this selection is a camera CHANGE the player made or got (getting in, left X / Y), so the view
// turns onto the vehicle's forward (vehcamrecenter). A file reload keeps your view where it is: editing a
// number should not spin you round.
void select(int vi, int ci, const char* why, bool recenter) {
    const vcp::Vehicle& v = s_table.vehicles[vi];
    const int n = static_cast<int>(v.cameras.size());
    if (n <= 0) { clear_selection(); return; }
    ci = ((ci % n) + n) % n;
    s_vehicle = vi;
    s_vehicle_name = v.name;
    s_camera = ci;
    s_remembered[v.name] = ci;
    if (recenter && g_cfg.veh_cam_recenter) ++s_recenter_gen;
    const VehActiveCam a = make_active(vi, ci);
    publish(a);
    // Our camera draws only for a chase camera; a first-person entry hands the view to the seat camera.
    g_veh_tp_active.store(g_cfg.veh_tp && a.type == static_cast<uint8_t>(vcp::CamType::Chase),
                          std::memory_order_relaxed);
    const vcp::Camera& c = v.cameras[ci];
    const std::string loc = vcp::location_tracking_text(c), rot = vcp::rotation_tracking_text(c);
    API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: %s -- camera %d/%d \"%s\" (%s): %s, origin %s, "
                         "offset (%.0f %.0f %.0f), location tracking %s, rotation tracking %s, body %s, "
                         "controls %s",
                         v.name.c_str(), ci + 1, n, c.name.c_str(), why, vcp::type_name(c.type),
                         vcp::origin_name(c.origin), c.offset[0], c.offset[1], c.offset[2],
                         loc.c_str(), rot.c_str(), (a.hide_body && g_cfg.veh_cam_hide_body != 0) ? "hidden" : "shown",
                         motion_on(a) ? "motion" : "stick");

    // THE READOUT on the text panel (vehcamreadout): which vehicle, which camera of how many, its name
    // when it has one, and what it does. Placed and timed by the xrtext* defaults.
    if (g_cfg.veh_cam_readout) {
        std::string md = "# " + v.name + " \xC2\xB7 Camera " + std::to_string(ci + 1) + " of " + std::to_string(n) + "\n";
        if (!c.name.empty()) md += "## " + c.name + "\n";
        if (c.type == vcp::CamType::FirstPerson) {
            md += "**Type:** First-person seat camera\n";
        } else {
            md += std::string("**Origin:** ") + (c.origin == vcp::Origin::Seat ? "Seat" : "Vehicle") + "\n";
            char off[96];
            std::snprintf(off, sizeof(off), "**Offset:** %.0f, %.0f, %.0f cm\n", c.offset[0], c.offset[1], c.offset[2]);
            md += off;
            md += "**Location Tracking:** " + loc + "\n";
            md += "**Rotation Tracking:** " + rot + "\n";
        }
        md += std::string("**Controls:** ") + (motion_on(a) ? "Motion aim" : "Stick") + "\n";
        xrtext_show(md);
    }
}

// LEFT STICK CLICK: flip this vehicle between motion controls and stick controls, and say which on the
// text panel -- always, since it answers a press (vehcamreadout is for the camera readout). Remembered
// for the entry, like the camera; a choice that lands back on the default simply forgets itself.
void apply_ctrl_toggle() {
    const vcp::Vehicle& v = s_table.vehicles[s_vehicle];
    const VehActiveCam cur = veh_active_cam();
    const std::string which = "## " + v.name + "\n";          // the mode is the title; this is whose it is
    if (cur.type != static_cast<uint8_t>(vcp::CamType::Chase)) {
        API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: %s -- left stick click: this is the first-person seat "
                             "camera, which has no motion aim -- stick controls stay", v.name.c_str());
        xrtext_show("# Stick controls\n" + which +
                    "Motion aim needs one of the chase cameras\n*Step to one with left X / Y*\n");
        return;
    }
    const bool want = !motion_on(cur);
    const bool base = base_motion(v);
    if (want == base) s_ctrl.erase(v.name); else s_ctrl[v.name] = CtrlChoice{want, base};
    if (s_camera >= 0 && s_camera < static_cast<int>(v.cameras.size())) publish(make_active(s_vehicle, s_camera));
    API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: %s -- left stick click: %s controls%s", v.name.c_str(),
                         want ? "MOTION" : "STICK",
                         want == base ? " (this vehicle's default)" : " (for this vehicle, this session)");
    std::string md;
    if (want) {
        md = "# Motion controls\n" + which + "Aim with the controller\n";
        md += (g_cfg.veh_stick_mode == 1) ? "Right stick turns your view\n" : "Right stick: the game's own\n";
    } else {
        md = "# Stick controls\n" + which + "Aim with the right stick\n";
    }
    md += "*Click the left stick to switch*\n";
    xrtext_show(md);
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

void veh_ctrl_toggle() {
    s_ctrl_toggle.fetch_add(1, std::memory_order_relaxed);
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
        // Said in the headset too: otherwise a typo shows only as "my edit did nothing".
        if (g_cfg.veh_tp)
            xrtext_show("# Camera file not applied\n" + r.error + "\n*The cameras you had stay in use until it reads cleanly*\n");
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
        s_ctrl_toggle.store(0, std::memory_order_relaxed);
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
            select(vi, ci, reload ? "file reloaded" : "entered", /*recenter=*/!reload);
        }
    }

    const int step = s_step.exchange(0, std::memory_order_relaxed);
    if (step != 0 && s_vehicle >= 0)
        select(s_vehicle, s_camera + step, step > 0 ? "left Y: next" : "left X: previous", /*recenter=*/true);

    if (s_ctrl_toggle.exchange(0, std::memory_order_relaxed) != 0 && s_vehicle >= 0) apply_ctrl_toggle();

    // A left-stick choice whose default has since changed under it (vehaim edited live -- a file edit
    // already re-selects above) gives way to the edit, and the aim follows at once.
    if (!s_ctrl.empty() && s_vehicle >= 0) {
        const vcp::Vehicle& v = s_table.vehicles[s_vehicle];
        if (s_ctrl.count(v.name) != 0 && ctrl_choice(v) < 0) {
            if (s_camera >= 0 && s_camera < static_cast<int>(v.cameras.size())) publish(make_active(s_vehicle, s_camera));
            API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: %s -- its default controls changed, so your left-stick "
                                 "choice gives way to it (%s)", v.name.c_str(), base_motion(v) ? "motion" : "stick");
        }
    }

    const VehActiveCam a = veh_active_cam();
    decoupled_pitch_update(a.valid && g_veh_tp_active.load(std::memory_order_relaxed)
                           && (a.rot_pitch || a.rot_roll));
}

} // namespace halo
