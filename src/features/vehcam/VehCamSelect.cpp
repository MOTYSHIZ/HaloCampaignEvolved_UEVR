#include "features/vehcam/VehCamSelect.hpp"

#include "Config.hpp"
#include "XrText.hpp"                           // the camera readout on the text panel
#include "features/vehcam/VehCam.hpp"          // g_veh_tp_active
#include "features/vehcam/VehCamPresets.hpp"
#include "uevr/API.hpp"

#include <Windows.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

using uevr::API;

namespace halo {

namespace vcp = vehcampresets;

namespace {

// ---- what the other threads read ---------------------------------------------------------------
VehActiveCam     s_active[2];
std::atomic<int> s_active_front{0};
std::atomic<int> s_step{0};                  // left Y: cameras, posted by the input hook
std::atomic<int> s_mode_step{0};             // left X: the camera's tethering modes, posted by the input hook
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
int                s_seat = 0;                      // ...and the seat (vcp::seat_bits; 0 = the game has not said)
std::string        s_vehicle_actor;                 // ...and the name it was matched against
int                s_vehicle = -1;                  // index into s_table.vehicles; -1 = none
int                s_camera = 0;
int                s_mode = 0;                      // the camera's tethering mode
std::string        s_vehicle_name;                  // the entry's name (a reload may reorder entries)
uint32_t           s_recenter_gen = 0;              // VehActiveCam::recenter_gen: bumped by select(..., recenter)

// ---- YOUR LAST CAMERA, ITS MODE AND YOUR CONTROLS IN EACH SEAT, kept across sessions ----------------
// Keyed by the entry AND the seat the game names (a Scorpion's driver and a rider on it can share one
// entry). The camera and its tethering mode are kept by NAME as well as number, so reordering the file
// does not move you. The controls are kept WITH the default they were chosen against, so a later edit of
// that default (the file's motionAim, or vehaim) wins over the choice: the edit is the newer statement of
// what you want, and a choice that lands back on the default simply forgets itself. Written beside the
// camera file (halo_vr_vehcams_last.txt: user-owned, never shipped), at most every ~2 s, replaced whole.
struct SeatMemory {
    int         camera = -1;          // the camera's index; -1 = none kept
    std::string camera_name;
    int         mode = -1;            // its tethering mode's index; -1 = none kept (a line from before modes)
    std::string mode_name;
    int         ctrl = -1;            // the left stick click's choice: -1 none, 0 stick, 1 motion
    int         ctrl_base = -1;       // ...and the default it was chosen against (0 stick, 1 motion)
};
std::map<std::string, SeatMemory> s_memory;
bool s_memory_dirty = false;
char s_mem_path[MAX_PATH]{};

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

// A driver's seat that does not also fire: there, Halo steers the vehicle toward where it aims.
bool driver_steers() {
    return (s_seat & vcp::kSeatDriver) != 0 && (s_seat & vcp::kSeatGunner) == 0;
}

// The memory's key: the entry, and the seat as the game names it.
std::string mem_key(const vcp::Vehicle& v) {
    const std::string seat = vcp::seat_text(static_cast<uint8_t>(s_seat));
    return v.name + "|" + (seat.empty() ? std::string("any seat") : seat);
}

// The left-stick choice for this seat: -1 = none (or its default has changed under it since, which
// drops it), else 0 = stick / 1 = motion.
int ctrl_choice(const vcp::Vehicle& v) {
    const auto it = s_memory.find(mem_key(v));
    if (it == s_memory.end() || it->second.ctrl < 0) return -1;
    if (it->second.ctrl_base != (base_motion(v) ? 1 : 0)) {
        it->second.ctrl = -1; it->second.ctrl_base = -1;
        s_memory_dirty = true;
        return -1;
    }
    return it->second.ctrl;
}

// The camera this seat was last left in: by name when the entry still has one of that name, else by
// number while it is in range; -1 = none.
int remembered_camera(const vcp::Vehicle& v) {
    const auto it = s_memory.find(mem_key(v));
    if (it == s_memory.end() || it->second.camera < 0) return -1;
    const int n = static_cast<int>(v.cameras.size());
    if (!it->second.camera_name.empty())
        for (int i = 0; i < n; ++i) if (v.cameras[i].name == it->second.camera_name) return i;
    return it->second.camera < n ? it->second.camera : -1;
}

// ...and the tethering mode it was left in, when that camera is camera `ci`: by name, else by number while
// in range; -1 = none (the memory names another camera, or predates modes).
int remembered_mode(const vcp::Vehicle& v, int ci) {
    const auto it = s_memory.find(mem_key(v));
    if (it == s_memory.end() || it->second.mode < 0 || remembered_camera(v) != ci) return -1;
    const vcp::Camera& c = v.cameras[static_cast<size_t>(ci)];
    const int n = c.mode_count();
    if (!it->second.mode_name.empty())
        for (int i = 0; i < n; ++i) if (c.mode(i).name == it->second.mode_name) return i;
    return it->second.mode < n ? it->second.mode : -1;
}

// ---- the memory file: tab-separated, one seat a line --------------------------------------------
void memory_load() {
    if (s_mem_path[0] == 0) return;
    FILE* f = nullptr;
    if (fopen_s(&f, s_mem_path, "rb") != 0 || f == nullptr) return;   // none yet: nothing kept
    char line[512];
    int n = 0;
    while (fgets(line, sizeof(line), f) != nullptr) {
        if (line[0] == '#' || line[0] == '\r' || line[0] == '\n') continue;
        std::string fld[8];
        int k = 0;
        for (const char* p = line; *p != 0 && *p != '\r' && *p != '\n'; ++p) {
            if (*p == '\t') { if (++k >= 8) break; continue; }
            fld[k].push_back(*p);
        }
        // 8 fields; 6 = a line written before tethering modes (no mode columns), read as "no mode kept".
        if ((k != 7 && k != 5) || fld[0].empty() || fld[1].empty()) continue;   // not a line of ours
        const std::string& ctrl = (k == 7) ? fld[6] : fld[4];
        const std::string& base = (k == 7) ? fld[7] : fld[5];
        SeatMemory m;
        const int num = std::atoi(fld[2].c_str());
        m.camera = num > 0 ? num - 1 : -1;
        m.camera_name = fld[3];
        if (k == 7) {
            const int mnum = std::atoi(fld[4].c_str());
            m.mode = mnum > 0 ? mnum - 1 : -1;
            m.mode_name = fld[5];
        }
        m.ctrl = ctrl == "motion" ? 1 : (ctrl == "stick" ? 0 : -1);
        m.ctrl_base = base == "motion" ? 1 : (base == "stick" ? 0 : -1);
        if (m.ctrl < 0 || m.ctrl_base < 0) { m.ctrl = -1; m.ctrl_base = -1; }
        s_memory[fld[0] + "|" + fld[1]] = m;
        ++n;
    }
    fclose(f);
    API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: your last camera, mode and controls kept for %d seat(s) (%s)", n, s_mem_path);
}

void memory_save() {
    if (!s_memory_dirty || s_mem_path[0] == 0) return;
    s_memory_dirty = false;
    char tmp[MAX_PATH + 8];
    strcpy_s(tmp, sizeof(tmp), s_mem_path);
    strcat_s(tmp, sizeof(tmp), ".tmp");
    FILE* f = nullptr;
    if (fopen_s(&f, tmp, "wb") != 0 || f == nullptr) { s_memory_dirty = true; return; }   // try again next poll
    fputs("# Your last camera, tethering mode and controls in each vehicle seat -- written by the mod, read when it starts.\r\n"
          "# Delete this file to start every seat in its defaultCamera / defaultMode and default controls again.\r\n"
          "# entry\tseat\tcamera number\tcamera name\tmode number\tmode name\tcontrols you chose\tthe default you chose them against\r\n", f);
    for (const auto& kv : s_memory) {
        const size_t bar = kv.first.rfind('|');
        if (bar == std::string::npos) continue;
        const SeatMemory& m = kv.second;
        if (m.camera < 0 && m.ctrl < 0) continue;
        std::fprintf(f, "%s\t%s\t%d\t%s\t%d\t%s\t%s\t%s\r\n", kv.first.substr(0, bar).c_str(), kv.first.substr(bar + 1).c_str(),
                     m.camera + 1, m.camera_name.c_str(), m.mode + 1, m.mode_name.c_str(),
                     m.ctrl == 1 ? "motion" : (m.ctrl == 0 ? "stick" : "-"),
                     m.ctrl_base == 1 ? "motion" : (m.ctrl_base == 0 ? "stick" : "-"));
    }
    fclose(f);
    if (!MoveFileExA(tmp, s_mem_path, MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
        DeleteFileA(tmp);
        s_memory_dirty = true;
    }
}

// Whether the controller aims under this published camera. Only under one of OUR chase cameras: the
// pointing ray is built from the view our eye publishes, and a first-person entry hands the view to the
// seat camera, which publishes none -- so there it is stick controls whatever was chosen.
bool motion_on(const VehActiveCam& a) {
    if (a.type != static_cast<uint8_t>(vcp::CamType::Chase)) return false;
    return a.motion_aim >= 0 ? a.motion_aim != 0 : g_cfg.veh_aim;
}

// Camera `ci` of entry `vi`, in its tethering mode `mi` (wrapped into range): the mode's tracking, offset
// and aim ring, the camera's everything else.
VehActiveCam make_active(int vi, int ci, int mi) {
    const vcp::Vehicle& v = s_table.vehicles[vi];
    const vcp::Camera& c = v.cameras[ci];
    const int nm = c.mode_count();
    mi = ((mi % nm) + nm) % nm;
    const vcp::Tether m = c.mode(mi);
    VehActiveCam a;
    a.valid = true;
    a.type = static_cast<uint8_t>(c.type);
    a.origin = static_cast<uint8_t>(c.origin);
    a.loc_yaw = m.loc_yaw; a.loc_pitch = m.loc_pitch; a.loc_roll = m.loc_roll; a.loc_view = m.loc_view;
    // Independent: pitch and roll without yaw tilt the view with the vehicle's deck while it keeps its
    // own heading (vehcammath::tracked_frame says exactly what that means).
    a.rot_yaw = m.rot_yaw; a.rot_pitch = m.rot_pitch; a.rot_roll = m.rot_roll;
    a.collide = c.collide;
    a.collide_margin = c.collide_margin;
    a.hide_body = c.hides_body();
    a.hide_head = vcp::effective_hide_head(v, ci);
    for (int k = 0; k < 3; ++k) a.offset[k] = m.offset[k];
    const int choice = ctrl_choice(v);                      // the left stick click beats the file
    a.motion_aim = choice >= 0 ? choice : v.motion_aim;
    a.aim_marker = vcp::effective_aim_marker(v, ci, mi);    // the mode's, else the camera's, else the entry's
    a.index = ci;
    a.count = static_cast<int>(v.cameras.size());
    a.mode = mi;
    a.mode_count = nm;
    a.recenter_gen = s_recenter_gen;
    return a;
}

// A mode's name for the readout and the log: its own, else its number.
std::string mode_label(const vcp::Tether& m, int mi) {
    return m.name.empty() ? "Mode " + std::to_string(mi + 1) : m.name;
}

void clear_selection() {
    if (s_vehicle >= 0) API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: out of the vehicle -- camera released");
    s_vehicle = -1;
    s_vehicle_name.clear();
    publish(VehActiveCam{});
    g_veh_tp_active.store(false, std::memory_order_relaxed);
}

// recenter: this selection is a camera CHANGE the player made or got (getting in, left Y / X), so the view
// turns until what aims the vehicle points where it aims (vehcamrecenter; the eye works it out). A file
// reload keeps your view where it is: editing a number should not spin you round.
void select(int vi, int ci, int mi, const char* why, bool recenter) {
    const vcp::Vehicle& v = s_table.vehicles[vi];
    const int n = static_cast<int>(v.cameras.size());
    if (n <= 0) { clear_selection(); return; }
    ci = ((ci % n) + n) % n;
    const vcp::Camera& c = v.cameras[ci];
    const int nm = c.mode_count();
    mi = ((mi % nm) + nm) % nm;
    const vcp::Tether mode = c.mode(mi);
    s_vehicle = vi;
    s_vehicle_name = v.name;
    s_camera = ci;
    s_mode = mi;
    {
        SeatMemory& m = s_memory[mem_key(v)];
        if (m.camera != ci || m.camera_name != c.name || m.mode != mi || m.mode_name != mode.name) {
            m.camera = ci;
            m.camera_name = c.name;
            m.mode = mi;
            m.mode_name = mode.name;
            s_memory_dirty = true;
        }
    }
    if (recenter && g_cfg.veh_cam_recenter) ++s_recenter_gen;
    const VehActiveCam a = make_active(vi, ci, mi);
    publish(a);
    // Our camera draws only for a chase camera; a first-person entry hands the view to the seat camera.
    g_veh_tp_active.store(g_cfg.veh_tp && a.type == static_cast<uint8_t>(vcp::CamType::Chase),
                          std::memory_order_relaxed);
    const std::string loc = vcp::location_tracking_text(mode), rot = vcp::rotation_tracking_text(mode);
    const std::string mlabel = mode_label(mode, mi);
    API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: %s -- camera %d/%d \"%s\", mode %d/%d \"%s\" (%s): %s, origin %s, "
                         "offset (%.0f %.0f %.0f), location tracking %s, rotation tracking %s, body %s, head %s, "
                         "aim ring %s, controls %s, seat %s",
                         v.name.c_str(), ci + 1, n, c.name.c_str(), mi + 1, nm, mlabel.c_str(), why, vcp::type_name(c.type),
                         vcp::origin_name(c.origin), a.offset[0], a.offset[1], a.offset[2],
                         loc.c_str(), rot.c_str(), (a.hide_body && g_cfg.veh_cam_hide_body != 0) ? "hidden" : "shown",
                         a.hide_head ? "hidden" : "shown", a.aim_marker ? "on" : "off",
                         motion_on(a) ? "motion" : "stick",
                         s_seat != 0 ? vcp::seat_text(static_cast<uint8_t>(s_seat)).c_str() : "unknown");

    // THE READOUT on the text panel (vehcamreadout): which vehicle, which camera of how many, its name and
    // its tethering mode's, and what it does. ROTATION TRACKING IS YELLOW: in VR the view turning when you
    // did not turn is what carries the most motion-sickness risk, so the axes that do it stand out.
    // Placed and timed by the xrtext* defaults.
    if (g_cfg.veh_cam_readout) {
        std::string md = "# " + v.name + " \xC2\xB7 Camera " + std::to_string(ci + 1) + " of " + std::to_string(n) + "\n";
        const bool first_person = c.type == vcp::CamType::FirstPerson;
        const bool modes = nm > 1 && !first_person;
        if (modes) md += "## " + (c.name.empty() ? mlabel : c.name + " \xC2\xB7 " + mlabel) + "\n";
        else if (!c.name.empty()) md += "## " + c.name + "\n";
        if (first_person) {
            md += "**Type:** First-person seat camera\n";
        } else {
            if (modes) md += "**Tethering:** " + std::to_string(mi + 1) + " of " + std::to_string(nm) + " *(left X)*\n";
            md += std::string("**Origin:** ")
                + (c.origin == vcp::Origin::Seat ? "Seat" : (c.origin == vcp::Origin::Head ? "Player's head" : "Vehicle")) + "\n";
            char off[96];
            std::snprintf(off, sizeof(off), "**Offset:** %.0f, %.0f, %.0f cm\n", a.offset[0], a.offset[1], a.offset[2]);
            md += off;
            md += "**Location Tracking:** " + loc + "\n";
            md += "**Rotation Tracking:** " + ((mode.rot_yaw || mode.rot_pitch || mode.rot_roll) ? "==" + rot + "==" : rot) + "\n";
        }
        md += std::string("**Controls:** ") + (motion_on(a) ? "Motion aim" : "Stick") + "\n";
        // The seat as the GAME reports it -- what an entry's "seat" is matched against.
        if (s_seat != 0) md += "**Seat:** " + vcp::seat_text(static_cast<uint8_t>(s_seat)) + "\n";
        // Halo steers a vehicle toward where it aims, so in the driver's seat pointing IS steering.
        if (motion_on(a) && driver_steers()) md += "*Point to steer and aim*\n";
        xrtext_show(md);
    }
}

// LEFT STICK CLICK: flip this seat between motion controls and stick controls, and say which on the text
// panel -- always, since it answers a press (vehcamreadout is for the camera readout). Kept for the seat
// across sessions, like the camera; a choice that lands back on the default simply forgets itself.
void apply_ctrl_toggle() {
    const vcp::Vehicle& v = s_table.vehicles[s_vehicle];
    const VehActiveCam cur = veh_active_cam();
    const std::string which = "## " + v.name + "\n";          // the mode is the title; this is whose it is
    if (cur.type != static_cast<uint8_t>(vcp::CamType::Chase)) {
        API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: %s -- left stick click: this is the first-person seat "
                             "camera, which has no motion aim -- stick controls stay", v.name.c_str());
        xrtext_show("# Stick controls\n" + which +
                    "Motion aim needs one of the chase cameras\n*Step to one with left Y*\n");
        return;
    }
    const bool want = !motion_on(cur);
    const bool base = base_motion(v);
    {
        SeatMemory& m = s_memory[mem_key(v)];
        if (want == base) { m.ctrl = -1; m.ctrl_base = -1; }                // back on the default: forgets itself
        else              { m.ctrl = want ? 1 : 0; m.ctrl_base = base ? 1 : 0; }
        s_memory_dirty = true;
    }
    if (s_camera >= 0 && s_camera < static_cast<int>(v.cameras.size())) publish(make_active(s_vehicle, s_camera, s_mode));
    API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: %s -- left stick click: %s controls%s", v.name.c_str(),
                         want ? "MOTION" : "STICK",
                         want == base ? " (this vehicle's default)" : " (kept for this seat)");
    std::string md;
    if (want) {
        md = "# Motion controls\n" + which + (driver_steers() ? "Point with the controller to steer and aim\n"
                                                              : "Aim with the controller\n");
        md += (g_cfg.veh_stick_mode == 1) ? "Right stick turns your view\n" : "Right stick: the game's own\n";
    } else {
        md = "# Stick controls\n" + which + "Aim with the right stick\n";
    }
    md += "*Click the left stick to switch*\n";
    xrtext_show(md);
}

// LEFT X: the current camera's next tethering mode (what turns your view, what carries the camera round,
// and where it sits), recentred like a camera change. A camera with one mode says so instead -- the press
// is answered either way, so it shows whatever vehcamreadout says.
void apply_mode_step(int dir) {
    const vcp::Vehicle& v = s_table.vehicles[s_vehicle];
    if (s_camera < 0 || s_camera >= static_cast<int>(v.cameras.size())) return;
    const vcp::Camera& c = v.cameras[s_camera];
    if (c.type == vcp::CamType::FirstPerson || c.mode_count() <= 1) {
        API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: %s -- left X: camera \"%s\" has %s", v.name.c_str(), c.name.c_str(),
                             c.type == vcp::CamType::FirstPerson ? "no tethering modes (the first-person seat camera)"
                                                                 : "one tethering mode");
        xrtext_show("# " + (c.name.empty() ? std::string("This camera") : c.name) + "\n## " + v.name + "\n"
                    + (c.type == vcp::CamType::FirstPerson ? "The first-person seat camera has no tethering modes\n"
                                                           : "This camera has one tethering mode\n")
                    + "*Left Y: next camera*\n");
        return;
    }
    select(s_vehicle, s_camera, s_mode + dir, dir > 0 ? "left X: next mode" : "left X: previous mode", /*recenter=*/true);
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

void veh_cam_mode_step(int dir) {
    s_mode_step.fetch_add(dir > 0 ? 1 : -1, std::memory_order_relaxed);
}

void veh_ctrl_toggle() {
    s_ctrl_toggle.fetch_add(1, std::memory_order_relaxed);
}

void vehcam_presets_init(const char* path) {
    if (path == nullptr || path[0] == 0) return;
    strcpy_s(s_path, sizeof(s_path), path);
    // The decoupled-pitch marker lives beside the file, and so does your per-seat memory.
    strcpy_s(s_dp_marker, sizeof(s_dp_marker), path);
    if (char* dot = std::strrchr(s_dp_marker, '.')) *dot = 0;
    strcpy_s(s_mem_path, sizeof(s_mem_path), s_dp_marker);
    strcat_s(s_dp_marker, sizeof(s_dp_marker), ".decoupledpitch.restore");
    strcat_s(s_mem_path, sizeof(s_mem_path), "_last.txt");
    memory_load();
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
    memory_save();                             // your per-seat memory, when it changed (~2 s at most late)
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

std::vector<std::string> vehcam_chassis_hint(const std::wstring& vehicle_name, int seat) {
    const int vi = vcp::match_vehicle(s_table, narrow(vehicle_name), static_cast<uint8_t>(seat));
    if (vi < 0) return {};
    return s_table.vehicles[vi].chassis;
}

void vehcam_select_tick(bool in_vehicle, uintptr_t chassis, const std::wstring& vehicle_name, int seat) {
    ++s_ticks;
    if (s_ticks == 300) decoupled_pitch_startup_check();

    if (!in_vehicle || chassis == 0) {
        // Out of the vehicle (or not identified yet): nothing selected, nothing forced, and a step
        // pressed meanwhile does not carry into the next ride.
        if (!in_vehicle) {
            if (s_vehicle >= 0 || s_chassis != 0) clear_selection();
            s_chassis = 0; s_seat = 0; s_vehicle_actor.clear();
        }
        s_step.store(0, std::memory_order_relaxed);
        s_mode_step.store(0, std::memory_order_relaxed);
        s_ctrl_toggle.store(0, std::memory_order_relaxed);
        decoupled_pitch_update(false);
        return;
    }

    // Re-matched when the chassis changes, the file changes, or the GAME names the seat (it may do so a
    // little after you get in, and a seat swap names another).
    const std::string actor = narrow(vehicle_name);
    const bool seat_changed = seat != s_seat || actor != s_vehicle_actor;
    if (chassis != s_chassis || s_table_changed || seat_changed) {
        const bool reload = chassis == s_chassis && !seat_changed;   // only the file changed
        const bool seat_only = chassis == s_chassis && !s_table_changed && seat_changed;
        const std::string prev = s_vehicle_name;
        s_chassis = chassis;
        s_seat = seat;
        s_vehicle_actor = actor;
        s_table_changed = false;
        const int vi = vcp::match_vehicle(s_table, actor, static_cast<uint8_t>(seat));
        const std::string seat_said = seat != 0 ? vcp::seat_text(static_cast<uint8_t>(seat)) : std::string("unknown");
        if (vi < 0) {
            clear_selection();
            API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: no camera entry matches %s (seat %s) and there is no "
                                 "\"default\" entry -- the game's camera stays", actor.c_str(), seat_said.c_str());
        } else {
            const vcp::Vehicle& v = s_table.vehicles[vi];
            const int kept = remembered_camera(v);   // this seat's last camera, from any earlier ride or session
            if (seat_only && vi == s_vehicle && (kept < 0 || kept == s_camera)) {
                // The seat was named and changes nothing: same entry, same camera -- keep the view as it is,
                // but take this seat's kept controls.
                API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: %s -- the game names your seat: %s", s_vehicle_name.c_str(),
                                     seat_said.c_str());
                if (s_camera >= 0 && s_camera < static_cast<int>(v.cameras.size()))
                    publish(make_active(s_vehicle, s_camera, s_mode));
            } else {
                // This seat's last camera, in its last mode -- by NAME, so a file edit that reorders the
                // cameras keeps you in the one you are in (select() keeps the memory current).
                int ci = v.default_camera, mi = v.default_mode;
                if (kept >= 0) {
                    ci = kept;
                    const int km = remembered_mode(v, kept);
                    mi = km >= 0 ? km : (kept == v.default_camera ? v.default_mode : 0);
                } else if (reload && prev == v.name) {
                    ci = s_camera; mi = s_mode;
                }
                select(vi, ci, mi, reload ? "file reloaded" : (seat_only ? "seat named" : "entered"), /*recenter=*/!reload);
            }
        }
    }

    // LEFT Y: the next camera, in the mode that keeps your tethering choice (vcp::carry_mode).
    const int step = s_step.exchange(0, std::memory_order_relaxed);
    if (step != 0 && s_vehicle >= 0) {
        const vcp::Vehicle& v = s_table.vehicles[s_vehicle];
        const int n = static_cast<int>(v.cameras.size());
        if (n > 0 && s_camera >= 0 && s_camera < n) {
            const int to = (((s_camera + step) % n) + n) % n;
            const int mi = vcp::carry_mode(v.cameras[to], v.cameras[s_camera].mode(s_mode));
            select(s_vehicle, to, mi, step > 0 ? "left Y: next camera" : "previous camera", /*recenter=*/true);
        }
    }

    // LEFT X: the current camera's next tethering mode.
    const int mstep = s_mode_step.exchange(0, std::memory_order_relaxed);
    if (mstep != 0 && s_vehicle >= 0) apply_mode_step(mstep);

    if (s_ctrl_toggle.exchange(0, std::memory_order_relaxed) != 0 && s_vehicle >= 0) apply_ctrl_toggle();

    // A left-stick choice whose default has since changed under it (vehaim edited live -- a file edit
    // already re-selects above) gives way to the edit, and the aim follows at once. Checked about twice a
    // second: the default only changes on the ~2 s config poll.
    if (s_vehicle >= 0 && (s_ticks % 16u) == 0u) {
        const vcp::Vehicle& v = s_table.vehicles[s_vehicle];
        const auto it = s_memory.find(mem_key(v));
        if (it != s_memory.end() && it->second.ctrl >= 0 && ctrl_choice(v) < 0) {
            if (s_camera >= 0 && s_camera < static_cast<int>(v.cameras.size())) publish(make_active(s_vehicle, s_camera, s_mode));
            API::get()->log_info("[Halo-CampE-UEVR] VEHCAM: %s -- its default controls changed, so your left-stick "
                                 "choice gives way to it (%s)", v.name.c_str(), base_motion(v) ? "motion" : "stick");
        }
    }

    const VehActiveCam a = veh_active_cam();
    decoupled_pitch_update(a.valid && g_veh_tp_active.load(std::memory_order_relaxed)
                           && (a.rot_pitch || a.rot_roll));
}

} // namespace halo
