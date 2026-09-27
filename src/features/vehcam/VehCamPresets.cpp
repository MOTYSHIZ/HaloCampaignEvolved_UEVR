#include "features/vehcam/VehCamPresets.hpp"

#include "core/JsonLite.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <utility>

namespace halo::vehcampresets {

namespace {

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

bool ieq(const std::string& a, const char* b) { return lower(a) == lower(std::string(b)); }

struct Axes { bool yaw, pitch, roll; };
constexpr Axes kAll{true, true, true};
constexpr Axes kYaw{true, false, false};
constexpr Axes kNone{false, false, false};

Camera cam(const char* name, Origin origin, float f, float r, float u, Axes loc, Axes rot) {
    Camera c;
    c.name = name; c.origin = origin;
    c.offset[0] = f; c.offset[1] = r; c.offset[2] = u;
    c.loc_yaw = loc.yaw; c.loc_pitch = loc.pitch; c.loc_roll = loc.roll;
    c.rot_yaw = rot.yaw; c.rot_pitch = rot.pitch; c.rot_roll = rot.roll;
    return c;
}

// The starter set every vehicle gets until the player tunes it, named the way players have named theirs:
// a view and its "Tethered" twin, which turns with the vehicle. Cockpit sits at your seat; Third Person
// behind the vehicle. The names say WHERE; the readout says what each tracks. Every vehicle STARTS in the
// plain Cockpit, where the world holds still: the generally comfortable first view. (A ground vehicle
// starting in a level tethered cockpit was tried the same day and rejected in-headset by the user as the
// less comfortable initial setting.) No "firstperson" entry: that hands the view to the older seat
// camera, which stays available by adding one to the file.
std::vector<Camera> starter_cameras() {
    // Each camera in two tethering modes (left X): held still, then turning with the vehicle.
    auto modes = [](Camera c, Axes tethered) {
        Tether still, teth;
        still.name = "Untethered";
        still.loc_yaw = c.loc_yaw; still.loc_pitch = c.loc_pitch; still.loc_roll = c.loc_roll;
        teth = still;
        teth.name = "Tethered";
        teth.rot_yaw = tethered.yaw; teth.rot_pitch = tethered.pitch; teth.rot_roll = tethered.roll;
        c.tethering = {still, teth};
        return c;
    };
    return {
        modes(cam("Cockpit",      Origin::Seat,    0.0f,    0.0f, 0.0f,   kAll, kNone), kAll),
        modes(cam("Third Person", Origin::Vehicle, -450.0f, 0.0f, 180.0f, kAll, kNone), kYaw),
    };
}

Vehicle vehicle(const char* name, std::vector<std::string> match) {
    Vehicle v;
    v.name = name;
    v.match = std::move(match);
    v.cameras = starter_cameras();
    return v;
}

// ---- reading ------------------------------------------------------------------------------------
using jsonlite::Value;

struct Apply {
    const char* text;
    ParseResult& r;

    bool type_error(const Value& v, const std::string& where, const char* want) {
        int line = 0, col = 0;
        jsonlite::line_col(text, v.at, line, col);
        char buf[320];
        std::snprintf(buf, sizeof(buf), "%s must be %s (line %d col %d)", where.c_str(), want, line, col);
        r.error = buf;
        return false;
    }
    // Offsets and margins, in cm. Bounded well inside float range: the reader refuses only a non-finite
    // DOUBLE, and 1e39 narrowed to +inf here, then to a NaN view position.
    bool num(const Value& v, const std::string& where, float& dst) {
        if (v.is_null()) return true;
        if (!v.is_num() || !(std::fabs(v.n) <= 1.0e6)) return type_error(v, where, "a number from -1000000 to 1000000");
        dst = static_cast<float>(v.n);
        return true;
    }
    bool boolean(const Value& v, const std::string& where, bool& dst) {
        if (v.is_null()) return true;
        if (!v.is_bool()) return type_error(v, where, "true or false");
        dst = v.b;
        return true;
    }
    // A list of any of "yaw", "pitch", "roll" ([] = none).
    bool axes(const Value& x, const std::string& w, bool& yaw, bool& pitch, bool& roll) {
        if (!x.is_arr()) return type_error(x, w, "a list of any of \"yaw\", \"pitch\", \"roll\" ([] = none)");
        yaw = pitch = roll = false;
        for (std::size_t i = 0; i < x.items.size(); ++i) {
            const Value& e = x.items[i];
            const std::string we = w + "[" + std::to_string(i) + "]";
            if (!e.is_str()) return type_error(e, we, "\"yaw\", \"pitch\" or \"roll\"");
            if (ieq(e.s, "yaw")) yaw = true;
            else if (ieq(e.s, "pitch")) pitch = true;
            else if (ieq(e.s, "roll")) roll = true;
            else return type_error(e, we, "\"yaw\", \"pitch\" or \"roll\"");
        }
        return true;
    }
    // locationTracking: a list of axes, or "view".
    bool loc_tracking(const Value& x, const std::string& w, bool& view, bool& yaw, bool& pitch, bool& roll) {
        if (x.is_str()) {
            if (!ieq(x.s, "view")) return type_error(x, w, "a list of any of \"yaw\", \"pitch\", \"roll\", or \"view\"");
            view = true;
            yaw = pitch = roll = false;
            return true;
        }
        view = false;
        return axes(x, w, yaw, pitch, roll);
    }
    // "leashMin" / "leashMax": [forward, right, up] in cm, each a number or null (no limit that way); null
    // for the whole key = no limits, which is how a mode drops its camera's. A min is 0 or less and a max
    // 0 or more: the camera's point stays inside the box, so a leash only ever stops you, never moves you.
    bool leash(const Value& x, const std::string& w, float dst[3], bool is_min) {
        const float none = is_min ? -kUnleashed : kUnleashed;
        dst[0] = dst[1] = dst[2] = none;
        if (x.is_null()) return true;
        const char* want = is_min ? "[forward, right, up] in cm, each 0 or less, or null (no limit that way)"
                                  : "[forward, right, up] in cm, each 0 or more, or null (no limit that way)";
        if (!x.is_arr() || x.items.size() > 3) return type_error(x, w, want);
        for (std::size_t i = 0; i < x.items.size(); ++i) {
            const Value& e = x.items[i];
            const std::string we = w + "[" + std::to_string(i) + "]";
            if (e.is_null()) continue;
            float f = none;
            if (!num(e, we, f)) return false;
            if (is_min ? f > 0.0f : f < 0.0f) return type_error(e, we, want);
            dst[i] = f;
        }
        return true;
    }
    // true / false, or null = inherit (-1).
    bool tri(const Value& x, const std::string& w, int& dst, const char* what) {
        if (x.is_null()) { dst = -1; return true; }
        if (!x.is_bool()) return type_error(x, w, what);
        dst = x.b ? 1 : 0;
        return true;
    }
    // One tethering mode, with what it named -- the rest is the camera's, filled in once the whole camera
    // is read (the file may list "tethering" before the camera's own tracking).
    struct RawTether { Tether t; bool has_loc = false, has_rot = false, has_off = false, has_lmin = false, has_lmax = false; };
    bool tether(const Value& e, const std::string& we, RawTether& rt) {
        if (!e.is_obj()) return type_error(e, we, "an object { \"name\": ..., \"rotationTracking\": [...] }");
        for (std::size_t k = 0; k < e.keys.size(); ++k) {
            const std::string& key = e.keys[k];
            const Value& x = e.items[k];
            const std::string w = we + "." + key;
            if (!key.empty() && key[0] == '_') continue;
            bool ok = true;
            if (key == "name") {
                if (x.is_null()) { rt.t.name.clear(); continue; }
                if (!x.is_str()) return type_error(x, w, "text");
                rt.t.name = x.s;
            } else if (key == "locationTracking") {
                rt.has_loc = true;
                ok = loc_tracking(x, w, rt.t.loc_view, rt.t.loc_yaw, rt.t.loc_pitch, rt.t.loc_roll);
            } else if (key == "rotationTracking") {
                rt.has_rot = true;
                ok = axes(x, w, rt.t.rot_yaw, rt.t.rot_pitch, rt.t.rot_roll);
            } else if (key == "offset") {
                if (!x.is_arr() || x.items.size() > 3) return type_error(x, w, "[forward, right, up] in cm");
                rt.has_off = true;
                for (std::size_t i = 0; i < x.items.size() && ok; ++i)
                    ok = num(x.items[i], w + "[" + std::to_string(i) + "]", rt.t.offset[i]);
            } else if (key == "leashMin") {
                rt.has_lmin = true;
                ok = leash(x, w, rt.t.leash_min, /*is_min=*/true);
            } else if (key == "leashMax") {
                rt.has_lmax = true;
                ok = leash(x, w, rt.t.leash_max, /*is_min=*/false);
            } else if (key == "aimMarker") {
                ok = tri(x, w, rt.t.aim_marker, "true, false or null (null = the camera's)");
            } else if (key == "origin") {
                Origin o = Origin::Vehicle;
                if (x.is_null()) { rt.t.origin = -1; rt.t.origin_socket.clear(); continue; }
                if (!origin_value(x, w, o, rt.t.origin_socket,
                                  "\"vehicle\", \"seat\", \"playerhead\", a bone or socket name, or null (null = the camera's)"))
                    return false;
                rt.t.origin = static_cast<int>(o);
            } else {
                r.ignored.push_back(w);
            }
            if (!ok) return false;
        }
        return true;
    }
    // "vehicle" | "seat" | "playerhead" ("head" too); any other name is a bone or socket on the vehicle.
    bool origin_value(const Value& x, const std::string& w, Origin& o, std::string& sock, const char* want) {
        if (!x.is_str() || x.s.empty()) return type_error(x, w, want);
        sock.clear();
        if (ieq(x.s, "vehicle")) o = Origin::Vehicle;
        else if (ieq(x.s, "seat")) o = Origin::Seat;
        else if (ieq(x.s, "playerhead") || ieq(x.s, "head")) o = Origin::Head;
        else { o = Origin::Socket; sock = x.s; }
        return true;
    }
    bool camera(const Value& v, const std::string& where, Camera& c) {
        if (!v.is_obj()) return type_error(v, where, "an object { \"name\": ..., \"offset\": [...] }");
        std::vector<RawTether> raw;
        for (std::size_t k = 0; k < v.keys.size(); ++k) {
            const std::string& key = v.keys[k];
            const Value& x = v.items[k];
            const std::string w = where + "." + key;
            if (!key.empty() && key[0] == '_') continue;
            bool ok = true;
            if (key == "name") {
                if (x.is_null()) { c.name.clear(); continue; }
                if (!x.is_str()) return type_error(x, w, "text");
                c.name = x.s;
            } else if (key == "type") {
                if (!x.is_str()) return type_error(x, w, "\"chase\" or \"firstperson\"");
                if (ieq(x.s, "chase")) c.type = CamType::Chase;
                else if (ieq(x.s, "firstperson")) c.type = CamType::FirstPerson;
                else return type_error(x, w, "\"chase\" or \"firstperson\"");
            } else if (key == "origin") {
                if (!origin_value(x, w, c.origin, c.origin_socket,
                                  "\"vehicle\", \"seat\", \"playerhead\", or a bone or socket name on the vehicle"))
                    return false;
            } else if (key == "offset") {
                if (!x.is_arr() || x.items.size() > 3) return type_error(x, w, "[forward, right, up] in cm");
                for (std::size_t i = 0; i < x.items.size() && ok; ++i)
                    ok = num(x.items[i], w + "[" + std::to_string(i) + "]", c.offset[i]);
            } else if (key == "leashMin") {
                ok = leash(x, w, c.leash_min, /*is_min=*/true);
            } else if (key == "leashMax") {
                ok = leash(x, w, c.leash_max, /*is_min=*/false);
            } else if (key == "locationTracking") {
                ok = loc_tracking(x, w, c.loc_view, c.loc_yaw, c.loc_pitch, c.loc_roll);
            } else if (key == "rotationTracking" || key == "viewFollows") {   // viewFollows: the first file
                ok = axes(x, w, c.rot_yaw, c.rot_pitch, c.rot_roll);
            } else if (key == "offsetRides") {                                 // the first file's form
                if (!x.is_str()) return type_error(x, w, "\"vehicle\", \"vehicleYaw\" or \"view\"");
                c.loc_view = false;
                if (ieq(x.s, "vehicle")) { c.loc_yaw = c.loc_pitch = c.loc_roll = true; }
                else if (ieq(x.s, "vehicleYaw")) { c.loc_yaw = true; c.loc_pitch = c.loc_roll = false; }
                else if (ieq(x.s, "view")) { c.loc_view = true; c.loc_yaw = c.loc_pitch = c.loc_roll = false; }
                else return type_error(x, w, "\"vehicle\", \"vehicleYaw\" or \"view\"");
            } else if (key == "collide") {
                ok = boolean(x, w, c.collide);
            } else if (key == "collideMargin") {
                ok = num(x, w, c.collide_margin);
            } else if (key == "hideBody") {
                if (x.is_null()) { c.hide_body = -1; continue; }
                if (!x.is_bool()) return type_error(x, w, "true, false or null (null = hide it for seat cameras)");
                c.hide_body = x.b ? 1 : 0;
            } else if (key == "hideHead") {
                ok = tri(x, w, c.hide_head, "true, false or null (null = the seat's hideHead)");
            } else if (key == "aimMarker") {
                ok = tri(x, w, c.aim_marker, "true, false or null (null = the seat's aimMarker)");
            } else if (key == "tethering") {
                if (!x.is_arr() || x.items.empty()) return type_error(x, w, "a list of at least one tethering mode");
                if (x.items.size() > 8) return type_error(x, w, "at most 8 tethering modes");
                for (std::size_t i = 0; i < x.items.size(); ++i) {
                    RawTether rt;
                    if (!tether(x.items[i], w + "[" + std::to_string(i) + "]", rt)) return false;
                    raw.push_back(rt);
                }
            } else {
                r.ignored.push_back(w);
            }
            if (!ok) return false;
        }
        // Each mode takes the camera's own tracking for what it left out.
        c.tethering.clear();
        for (RawTether& rt : raw) {
            if (!rt.has_loc) { rt.t.loc_view = c.loc_view; rt.t.loc_yaw = c.loc_yaw; rt.t.loc_pitch = c.loc_pitch; rt.t.loc_roll = c.loc_roll; }
            if (!rt.has_rot) { rt.t.rot_yaw = c.rot_yaw; rt.t.rot_pitch = c.rot_pitch; rt.t.rot_roll = c.rot_roll; }
            if (!rt.has_off) { rt.t.offset[0] = c.offset[0]; rt.t.offset[1] = c.offset[1]; rt.t.offset[2] = c.offset[2]; }
            for (int k = 0; k < 3; ++k) {
                if (!rt.has_lmin) rt.t.leash_min[k] = c.leash_min[k];
                if (!rt.has_lmax) rt.t.leash_max[k] = c.leash_max[k];
            }
            c.tethering.push_back(rt.t);
        }
        return true;
    }
    bool vehicle_entry(const std::string& key, const Value& v, const std::string& where, Vehicle& out) {
        if (!v.is_obj()) return type_error(v, where, "an object { \"cameras\": [...] }");
        out = Vehicle{};
        out.name = key;
        out.is_default = ieq(key, "default");
        bool have_match = false, have_cams = false;
        // The seat's leash FIRST, wherever the file puts it: every camera starts from it (a camera, and then a
        // tethering mode, may override it), and "cameras" may well come before it in the file.
        for (std::size_t k = 0; k < v.keys.size(); ++k) {
            const std::string& fk = v.keys[k];
            if (fk == "leashMin" && !leash(v.items[k], where + "." + fk, out.leash_min, /*is_min=*/true)) return false;
            if (fk == "leashMax" && !leash(v.items[k], where + "." + fk, out.leash_max, /*is_min=*/false)) return false;
        }
        for (std::size_t k = 0; k < v.keys.size(); ++k) {
            const std::string& fk = v.keys[k];
            const Value& x = v.items[k];
            const std::string w = where + "." + fk;
            if (!fk.empty() && fk[0] == '_') continue;
            if (fk == "match") {
                have_match = true;
                if (x.is_str()) { out.match.push_back(lower(x.s)); continue; }
                if (!x.is_arr()) return type_error(x, w, "text or a list of text");
                for (std::size_t i = 0; i < x.items.size(); ++i) {
                    if (!x.items[i].is_str()) return type_error(x.items[i], w + "[" + std::to_string(i) + "]", "text");
                    if (!x.items[i].s.empty()) out.match.push_back(lower(x.items[i].s));
                }
            } else if (fk == "chassis") {
                out.chassis.clear();
                if (x.is_null()) continue;
                if (x.is_str()) { if (!x.s.empty()) out.chassis.push_back(lower(x.s)); continue; }
                if (!x.is_arr()) return type_error(x, w, "text or a list of text (part of the mesh's name)");
                for (std::size_t i = 0; i < x.items.size(); ++i) {
                    if (!x.items[i].is_str()) return type_error(x.items[i], w + "[" + std::to_string(i) + "]", "text");
                    if (!x.items[i].s.empty()) out.chassis.push_back(lower(x.items[i].s));
                }
            } else if (fk == "defaultCamera") {
                // Bounded before the cast: 3000000000 used to become INT_MIN and index far outside the
                // camera list -- a crash, and at startup UEVR then unloads the whole plugin every launch.
                if (!x.is_num() || x.n < 0.0 || x.n > 1000.0 || x.n != std::floor(x.n))
                    return type_error(x, w, "a camera index from 0");
                out.default_camera = static_cast<int>(x.n);
            } else if (fk == "defaultMode") {
                if (!x.is_num() || x.n < 0.0 || x.n > 1000.0 || x.n != std::floor(x.n))
                    return type_error(x, w, "a tethering mode index from 0");
                out.default_mode = static_cast<int>(x.n);
            } else if (fk == "hideHead") {
                if (!x.is_bool()) return type_error(x, w, "true or false");
                out.hide_head = x.b;
            } else if (fk == "motionAim") {
                if (x.is_null()) { out.motion_aim = -1; continue; }
                if (!x.is_bool()) return type_error(x, w, "true, false or null (null = the vehaim setting)");
                out.motion_aim = x.b ? 1 : 0;
            } else if (fk == "aimMarker") {
                if (!x.is_bool()) return type_error(x, w, "true or false");
                out.aim_marker = x.b;
            } else if (fk == "leashMin" || fk == "leashMax") {
                continue;   // read above, before the cameras
            } else if (fk == "seat") {
                // "passenger", or a list of them; [] or null = any seat.
                out.seats = 0;
                if (x.is_null()) continue;
                const char* want = "\"driver\", \"gunner\" or \"passenger\" (or a list of them)";
                auto role_bit = [](const std::string& s) -> uint8_t {
                    if (ieq(s, "driver"))    return 1u << static_cast<int>(SeatRole::Driver);
                    if (ieq(s, "gunner"))    return 1u << static_cast<int>(SeatRole::Gunner);
                    if (ieq(s, "passenger")) return 1u << static_cast<int>(SeatRole::Passenger);
                    return 0;
                };
                if (x.is_str()) {
                    const uint8_t b = role_bit(x.s);
                    if (b == 0) return type_error(x, w, want);
                    out.seats = b;
                    continue;
                }
                if (!x.is_arr()) return type_error(x, w, want);
                for (std::size_t i = 0; i < x.items.size(); ++i) {
                    const Value& e = x.items[i];
                    const uint8_t b = e.is_str() ? role_bit(e.s) : 0;
                    if (b == 0) return type_error(e, w + "[" + std::to_string(i) + "]", "\"driver\", \"gunner\" or \"passenger\"");
                    out.seats |= b;
                }
            } else if (fk == "cameras") {
                have_cams = true;
                if (!x.is_arr() || x.items.empty()) return type_error(x, w, "a list of at least one camera");
                if (static_cast<int>(x.items.size()) > kMaxCameras) return type_error(x, w, "at most 16 cameras");
                for (std::size_t i = 0; i < x.items.size(); ++i) {
                    Camera c;
                    for (int j = 0; j < 3; ++j) { c.leash_min[j] = out.leash_min[j]; c.leash_max[j] = out.leash_max[j]; }
                    if (!camera(x.items[i], w + "[" + std::to_string(i) + "]", c)) return false;
                    out.cameras.push_back(c);
                }
            } else {
                r.ignored.push_back(w);
            }
        }
        if (!have_cams) return type_error(v, where, "an object with a \"cameras\" list");
        if (!have_match && !out.is_default) out.match.push_back(lower(key));
        if (out.default_camera < 0 || out.default_camera >= static_cast<int>(out.cameras.size())) out.default_camera = 0;
        if (out.default_mode < 0 || out.default_mode >= out.cameras[static_cast<size_t>(out.default_camera)].mode_count())
            out.default_mode = 0;
        return true;
    }
};

// ---- writing ------------------------------------------------------------------------------------
std::string fmt_num(float v) {
    char buf[32];
    if (std::fabs(v - std::round(v)) < 1e-4f) std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(std::lround(v)));
    else std::snprintf(buf, sizeof(buf), "%g", static_cast<double>(v));
    return buf;
}

std::string quoted(const std::string& s) {
    std::string o = "\"";
    for (const char c : s) {
        if (c == '"' || c == '\\') { o += '\\'; o += c; }
        else if (static_cast<unsigned char>(c) < 0x20) o += ' ';
        else o += c;
    }
    return o + "\"";
}

std::string axes_json(bool yaw, bool pitch, bool roll) {
    std::string s = "[";
    if (yaw)   s += "\"yaw\"";
    if (pitch) s += std::string(s.size() > 1 ? ", " : "") + "\"pitch\"";
    if (roll)  s += std::string(s.size() > 1 ? ", " : "") + "\"roll\"";
    return s + "]";
}

// "leashMin" / "leashMax" as the file writes them: [forward, right, up], null on an axis with no limit;
// "null" when no axis has one.
std::string leash_json(const float v[3]) {
    if (!leash_set(v)) return "null";
    std::string s = "[";
    for (int k = 0; k < 3; ++k)
        s += std::string(k ? ", " : "") + ((v[k] > -kUnleashed && v[k] < kUnleashed) ? fmt_num(v[k]) : std::string("null"));
    return s + "]";
}

bool same3(const float a[3], const float b[3]) { return a[0] == b[0] && a[1] == b[1] && a[2] == b[2]; }

std::string axes_text(bool yaw, bool pitch, bool roll) {
    std::string s;
    if (yaw)   s += "Yaw";
    if (pitch) s += std::string(s.empty() ? "" : ", ") + "Pitch";
    if (roll)  s += std::string(s.empty() ? "" : ", ") + "Roll";
    return s.empty() ? "None" : s;
}

} // namespace

const char* type_name(CamType t)  { return t == CamType::FirstPerson ? "firstperson" : "chase"; }
const char* origin_name(Origin o) {
    return o == Origin::Seat ? "seat" : (o == Origin::Head ? "playerhead" : (o == Origin::Socket ? "socket" : "vehicle"));
}
std::string origin_text(Origin o, const std::string& socket) {
    return o == Origin::Socket ? socket : std::string(origin_name(o));
}
const char* seat_role_name(SeatRole s) {
    switch (s) {
        case SeatRole::Driver:    return "driver";
        case SeatRole::Gunner:    return "gunner";
        case SeatRole::Passenger: return "passenger";
        default:                  return "unknown";
    }
}
std::string seat_text(uint8_t seat) {
    std::string s;
    if (seat & kSeatDriver)    s += "Driver";
    if (seat & kSeatGunner)    s += std::string(s.empty() ? "" : ", ") + "Gunner";
    if (seat & kSeatPassenger) s += std::string(s.empty() ? "" : ", ") + "Passenger";
    return s;
}

std::string rotation_tracking_text(const Camera& c) { return axes_text(c.rot_yaw, c.rot_pitch, c.rot_roll); }
std::string location_tracking_text(const Camera& c) {
    return c.loc_view ? std::string("Your view (orbit)") : axes_text(c.loc_yaw, c.loc_pitch, c.loc_roll);
}
std::string rotation_tracking_text(const Tether& t) { return axes_text(t.rot_yaw, t.rot_pitch, t.rot_roll); }
std::string location_tracking_text(const Tether& t) {
    return t.loc_view ? std::string("Your view (orbit)") : axes_text(t.loc_yaw, t.loc_pitch, t.loc_roll);
}
std::string leash_text(const float leash_min[3], const float leash_max[3]) {
    static const char* const kPos[3] = {"forward", "right", "up"};
    static const char* const kNeg[3] = {"back", "left", "down"};
    std::string s;
    for (int k = 0; k < 3; ++k) {
        if (leash_max[k] < kUnleashed)  s += std::string(s.empty() ? "" : ", ") + kPos[k] + " " + fmt_num(leash_max[k]);
        if (leash_min[k] > -kUnleashed) s += std::string(s.empty() ? "" : ", ") + kNeg[k] + " " + fmt_num(-leash_min[k]);
    }
    return s.empty() ? std::string("None") : s + " cm";
}

int carry_mode(const Camera& to, const Tether& from) {
    const int n = to.mode_count();
    if (n <= 1) return 0;
    for (int i = 0; i < n; ++i)
        if (!from.name.empty() && lower(to.tethering[static_cast<size_t>(i)].name) == lower(from.name)) return i;
    for (int i = 0; i < n; ++i) {
        const Tether& m = to.tethering[static_cast<size_t>(i)];
        if (m.rot_yaw == from.rot_yaw && m.rot_pitch == from.rot_pitch && m.rot_roll == from.rot_roll) return i;
    }
    return 0;
}

// THE BUILT-IN CAMERAS: a CHECKPOINT CANONIZATION of the player's own tuned file (VehCamDefaults.inc,
// written by Scripts\VehCams-Tool.ps1 canonize). What a missing camera file gets, and what a first run
// writes out.
Table default_table() {
    static const char kCanonical[] =
#include "features/vehcam/VehCamDefaults.inc"
        ;
    Table t;
    if (sizeof(kCanonical) > 1 && table_from_json(kCanonical, sizeof(kCanonical) - 1, t).ok && !t.vehicles.empty())
        return t;
    return starter_table();
}

Table starter_table() {
    // Vehicle actor names from the game's own assets (BP_<name>VehicleActor). The turrets are
    // separate vehicles (the Warthog's chaingun, the Wraith's and the Scorpion's anti-infantry guns), so
    // a gunner gets their own entry -- the most specific match wins (match_vehicle), and
    // "WarthogVehicleActor" is chosen over "Warthog" so it does not match the chaingun at all. The
    // Warthog's passenger rides the Warthog actor itself, so that entry is told apart by its seat.
    Table t;
    t.vehicles.push_back(vehicle("Banshee",           {"bansheevehicleactor"}));
    t.vehicles.push_back(vehicle("Ghost",             {"ghostvehicleactor"}));
    t.vehicles.push_back(vehicle("Warthog gunner",    {"warthogchaingunvehicleactor"}));
    t.vehicles.push_back(vehicle("Warthog",           {"warthogvehicleactor"}));
    Vehicle wp = vehicle("Warthog passenger", {"warthogvehicleactor"});
    wp.seats = kSeatPassenger;
    t.vehicles.push_back(wp);
    t.vehicles.push_back(vehicle("Scorpion gunner",   {"scorpionantiinfantry"}));
    t.vehicles.push_back(vehicle("Scorpion",          {"scorpion"}));
    t.vehicles.push_back(vehicle("Wraith turret",     {"wraithantiinfantry"}));
    t.vehicles.push_back(vehicle("Wraith",            {"wraith"}));
    t.vehicles.push_back(vehicle("Shade",             {"shade"}));
    Vehicle d = vehicle("default", {});
    d.is_default = true;
    t.vehicles.push_back(d);
    return t;
}

ParseResult table_from_json(const char* text, std::size_t len, Table& out) {
    ParseResult r;
    jsonlite::Value root;
    std::string err;
    if (!jsonlite::parse(text, len, root, err)) { r.error = err; return r; }
    Apply a{text, r};
    if (!root.is_obj()) { a.type_error(root, "the file", "an object { \"vehicles\": { ... } }"); return r; }
    Table t;
    bool have_vehicles = false;
    for (std::size_t k = 0; k < root.keys.size(); ++k) {
        const std::string& key = root.keys[k];
        const jsonlite::Value& x = root.items[k];
        if (!key.empty() && key[0] == '_') continue;
        if (key == "version") continue;   // 1; read for the record, nothing depends on it yet
        if (key != "vehicles") { r.ignored.push_back(key); continue; }
        have_vehicles = true;
        if (!x.is_obj()) { a.type_error(x, "vehicles", "an object { \"Banshee\": { ... }, ... }"); return r; }
        if (static_cast<int>(x.keys.size()) > kMaxVehicles) { a.type_error(x, "vehicles", "at most 32 entries"); return r; }
        for (std::size_t i = 0; i < x.keys.size(); ++i) {
            if (!x.keys[i].empty() && x.keys[i][0] == '_') continue;
            Vehicle v;
            if (!a.vehicle_entry(x.keys[i], x.items[i], "vehicles." + x.keys[i], v)) return r;
            r.cameras += static_cast<int>(v.cameras.size());
            t.vehicles.push_back(std::move(v));
        }
    }
    if (!have_vehicles) { a.type_error(root, "the file", "an object with a \"vehicles\" section"); return r; }
    r.vehicles = static_cast<int>(t.vehicles.size());
    r.ok = true;
    out = std::move(t);
    return r;
}

std::string table_to_json(const Table& t, bool guide) {
    std::string s;
    s += "{\n";
    if (guide) {
    s += "  \"_readme\": [\n";
    s += "    \"VEHICLE CAMERAS. In a vehicle: LEFT Y = next camera, LEFT X = that camera's next tethering mode, and\",\n";
    s += "    \"  holding either for a second resets the view (lined up with the vehicle, your head back on the camera's\",\n";
    s += "    \"  point). LEFT GRIP = the game's switch seat. LEFT STICK CLICK = motion aim <-> stick aim for this seat.\",\n";
    s += "    \"  RIGHT STICK = turn your view, while the controller aims.\",\n";
    s += "    \"Each vehicle has its own list, used in order. The entry whose 'match' text appears in the vehicle's\",\n";
    s += "    \"name is used -- the longest such text when several do, so a turret with its own entry beats its\",\n";
    s += "    \"vehicle's (case does not matter); 'default' covers any vehicle not listed.\",\n";
    s += "    \"Camera fields -- all optional; anything left out takes its default:\",\n";
    s += "    \"  name              shown when you switch to it (left out = just its number)\",\n";
    s += "    \"  type              chase (this mod's camera) | firstperson (the older seat camera)\",\n";
    s += "    \"  origin            seat | vehicle | playerhead : where 'offset' is measured from (your seat, the\",\n";
    s += "    \"                    vehicle's centre, or the Chief's head -- a playerhead camera's tracking follows\",\n";
    s += "    \"                    the Chief, so it turns with a turret even where the turret's mesh does not);\",\n";
    s += "    \"                    or any other name: a bone or socket on the vehicle, and the camera rides it --\",\n";
    s += "    \"                    e.g. \\\"MainTurret_M\\\", the Scorpion cannon's turret. \\\"Part/Name\\\" picks the part\",\n";
    s += "    \"                    (\\\"ScorpionCannon/AimYaw\\\"); not found = the vehicle's centre (the log says why)\",\n";
    s += "    \"  offset            [forward, right, up] in cm from the origin (negative forward = behind);\",\n";
    s += "    \"                    locationTracking decides whether those directions turn with the vehicle\",\n";
    s += "    \"  locationTracking  which vehicle rotations carry the camera's position round: any of yaw, pitch,\",\n";
    s += "    \"                    roll ([] = a fixed world direction), or \\\"view\\\" to orbit with YOUR view\",\n";
    s += "    \"  rotationTracking  which vehicle rotations turn your VIEW: any of yaw, pitch, roll ([] = the world\",\n";
    s += "    \"                    holds still). Pitch and roll without yaw tilt your view with the vehicle's deck\",\n";
    s += "    \"                    while you keep your own heading.\",\n";
    s += "    \"  leashMin, leashMax  the LEASH: a box your HEAD stays inside, around the camera's point -- so a lean\",\n";
    s += "    \"                    in a tight seat cannot take you through the canopy or into the gun. Both are\",\n";
    s += "    \"                    [forward, right, up] in cm, on the offset's directions. leashMin = how far you may\",\n";
    s += "    \"                    move back, left and down (each 0 or negative); leashMax = how far forward, right\",\n";
    s += "    \"                    and up (each 0 or positive). null on an axis, or a shorter list, = no limit that\",\n";
    s += "    \"                    way. Past a limit the view stops following your head that way (the world moves\",\n";
    s += "    \"                    with you); coming back is free. Measured from where your head was put at the last\",\n";
    s += "    \"                    camera change or reset.\",\n";
    s += "    \"                    e.g. \\\"leashMin\\\": [-10, -15, -20], \\\"leashMax\\\": [15, 15, 5]\",\n";
    s += "    \"                         = at most 10 back, 15 left, 20 down, 15 forward, 15 right and 5 up.\",\n";
    s += "    \"                    Set it for a whole seat (in the vehicle entry), for a camera, or for a tethering\",\n";
    s += "    \"                    mode: each overrides the one above it, and null there = no leash at that level.\",\n";
    s += "    \"  tethering         the camera's MODES, which left X steps through: a list of { name, origin, offset,\",\n";
    s += "    \"                    leashMin, leashMax, locationTracking, rotationTracking, aimMarker }, each taking\",\n";
    s += "    \"                    the camera's own for what it leaves out -- e.g. the same cockpit held still and\",\n";
    s += "    \"                    tethered to the vehicle, or a turret held still at the seat and tethered to the\",\n";
    s += "    \"                    Chief (origin playerhead)\",\n";
    s += "    \"  collide           pull the camera in when a wall is in the way; collideMargin = cm to stop short\",\n";
    s += "    \"  hideBody          true | false: hide your character's body -- only while the vehcamhidebody setting\",\n";
    s += "    \"                    is on, which it is not by default (a firstperson camera hides it regardless).\",\n";
    s += "    \"                    Left out = hidden when the camera sits at your seat or head, shown otherwise\",\n";
    s += "    \"  hideHead, aimMarker  true | false for this camera, over the seat's own\",\n";
    s += "    \"Per vehicle: defaultCamera / defaultMode = where you start (from 0); motionAim = true | false overrides\",\n";
    s += "    \"  vehaim; hideHead = true hides your character's head in this seat, for true first person from a camera\",\n";
    s += "    \"  at your head (left out = false);\",\n";
    s += "    \"  aimMarker = true | false: a ring where the VEHICLE is aiming, beside the crosshair (left out = true);\",\n";
    s += "    \"  leashMin / leashMax = the leash for every camera of this seat (see leashMin above);\",\n";
    s += "    \"  seat = \\\"driver\\\" | \\\"gunner\\\" | \\\"passenger\\\" (or a list): this entry is only for that seat, as the\",\n";
    s += "    \"  game reports it (the readout's Seat line) -- how the Warthog's passenger gets cameras of its own. Left\",\n";
    s += "    \"  out: the driver's entry, and the one any other seat of the vehicle uses when it has none of its own;\",\n";
    s += "    \"  chassis = part of the name of the vehicle mesh the cameras follow, when it has several (left out =\",\n";
    s += "    \"  one named hull or body, else the nearest). Your last camera, mode and controls in each seat are kept.\",\n";
    s += "    \"Keys starting with _ are notes and are ignored: add your own anywhere (\\\"_why\\\": \\\"...\\\").\",\n";
    s += "    \"Saved changes apply within a couple of seconds. This file is yours: updates never overwrite it,\",\n";
    s += "    \"and deleting it brings the built-in cameras back.\"\n";
    s += "  ],\n";
    }
    s += "  \"version\": 1,\n";
    s += "  \"vehicles\": {\n";
    for (std::size_t vi = 0; vi < t.vehicles.size(); ++vi) {
        const Vehicle& v = t.vehicles[vi];
        s += "    " + quoted(v.name) + ": {\n";
        if (!v.is_default && !v.match.empty()) {
            s += "      \"match\": [";
            for (std::size_t m = 0; m < v.match.size(); ++m) s += (m ? ", " : "") + quoted(v.match[m]);
            s += "],\n";
        }
        s += "      \"defaultCamera\": " + std::to_string(v.default_camera) + ",\n";
        if (v.default_mode != 0) s += "      \"defaultMode\": " + std::to_string(v.default_mode) + ",\n";
        if (v.motion_aim >= 0) s += std::string("      \"motionAim\": ") + (v.motion_aim ? "true" : "false") + ",\n";
        s += std::string("      \"aimMarker\": ") + (v.aim_marker ? "true" : "false") + ",\n";
        if (v.hide_head) s += "      \"hideHead\": true,\n";
        if (leash_set(v.leash_min)) s += "      \"leashMin\": " + leash_json(v.leash_min) + ",\n";
        if (leash_set(v.leash_max)) s += "      \"leashMax\": " + leash_json(v.leash_max) + ",\n";
        if (v.seats != 0) {
            std::string roles;
            for (const SeatRole r : {SeatRole::Driver, SeatRole::Gunner, SeatRole::Passenger})
                if ((v.seats & (1u << static_cast<int>(r))) != 0)
                    roles += std::string(roles.empty() ? "" : ", ") + "\"" + seat_role_name(r) + "\"";
            s += "      \"seat\": [" + roles + "],\n";
        }
        if (!v.chassis.empty()) {
            s += "      \"chassis\": [";
            for (std::size_t m = 0; m < v.chassis.size(); ++m) s += (m ? ", " : "") + quoted(v.chassis[m]);
            s += "],\n";
        }
        s += "      \"cameras\": [\n";
        for (std::size_t ci = 0; ci < v.cameras.size(); ++ci) {
            const Camera& c = v.cameras[ci];
            s += "        { ";
            if (!c.name.empty()) s += "\"name\": " + quoted(c.name) + ", ";
            s += std::string("\"type\": \"") + type_name(c.type) + "\""
               + ", \"origin\": " + quoted(origin_text(c.origin, c.origin_socket))
               + ", \"offset\": [" + fmt_num(c.offset[0]) + ", " + fmt_num(c.offset[1]) + ", " + fmt_num(c.offset[2]) + "]"
               // Only where the camera differs from its seat's (null = it drops the seat's leash).
               + (!same3(c.leash_min, v.leash_min) ? ", \"leashMin\": " + leash_json(c.leash_min) : std::string())
               + (!same3(c.leash_max, v.leash_max) ? ", \"leashMax\": " + leash_json(c.leash_max) : std::string())
               + ", \"locationTracking\": " + (c.loc_view ? std::string("\"view\"") : axes_json(c.loc_yaw, c.loc_pitch, c.loc_roll))
               + ", \"rotationTracking\": " + axes_json(c.rot_yaw, c.rot_pitch, c.rot_roll)
               + ", \"collide\": " + (c.collide ? "true" : "false")
               + ", \"collideMargin\": " + fmt_num(c.collide_margin)
               + (c.hide_body < 0 ? std::string() : std::string(", \"hideBody\": ") + (c.hide_body ? "true" : "false"))
               + (c.hide_head < 0 ? std::string() : std::string(", \"hideHead\": ") + (c.hide_head ? "true" : "false"))
               + (c.aim_marker < 0 ? std::string() : std::string(", \"aimMarker\": ") + (c.aim_marker ? "true" : "false"));
            // Each mode on its own line under the camera, naming only what differs from the camera's own
            // tracking -- the reader fills the rest back in, so the file round-trips.
            if (!c.tethering.empty()) {
                s += ",\n          \"tethering\": [\n";
                for (std::size_t mi = 0; mi < c.tethering.size(); ++mi) {
                    const Tether& m = c.tethering[mi];
                    std::string f;
                    auto add = [&f](const std::string& kv) { f += (f.empty() ? "" : ", ") + kv; };
                    if (!m.name.empty()) add("\"name\": " + quoted(m.name));
                    if (m.origin >= 0 && (m.origin != static_cast<int>(c.origin)
                                          || (m.origin == static_cast<int>(Origin::Socket) && m.origin_socket != c.origin_socket)))
                        add("\"origin\": " + quoted(origin_text(static_cast<Origin>(m.origin), m.origin_socket)));
                    if (m.offset[0] != c.offset[0] || m.offset[1] != c.offset[1] || m.offset[2] != c.offset[2])
                        add("\"offset\": [" + fmt_num(m.offset[0]) + ", " + fmt_num(m.offset[1]) + ", " + fmt_num(m.offset[2]) + "]");
                    if (!same3(m.leash_min, c.leash_min)) add("\"leashMin\": " + leash_json(m.leash_min));
                    if (!same3(m.leash_max, c.leash_max)) add("\"leashMax\": " + leash_json(m.leash_max));
                    if (m.loc_view != c.loc_view || m.loc_yaw != c.loc_yaw || m.loc_pitch != c.loc_pitch || m.loc_roll != c.loc_roll)
                        add("\"locationTracking\": " + (m.loc_view ? std::string("\"view\"") : axes_json(m.loc_yaw, m.loc_pitch, m.loc_roll)));
                    if (m.rot_yaw != c.rot_yaw || m.rot_pitch != c.rot_pitch || m.rot_roll != c.rot_roll)
                        add("\"rotationTracking\": " + axes_json(m.rot_yaw, m.rot_pitch, m.rot_roll));
                    if (m.aim_marker >= 0) add(std::string("\"aimMarker\": ") + (m.aim_marker ? "true" : "false"));
                    s += "            { " + f + (f.empty() ? "}" : " }") + (mi + 1 < c.tethering.size() ? ",\n" : "\n");
                }
                s += "          ] }";
            } else {
                s += " }";
            }
            s += (ci + 1 < v.cameras.size() ? ",\n" : "\n");
        }
        s += "      ]\n";
        s += std::string("    }") + (vi + 1 < t.vehicles.size() ? ",\n" : "\n");
    }
    s += "  }\n";
    s += "}\n";
    return s;
}

// THE MOST SPECIFIC MATCH WINS: the longest "match" text found in the name, then an entry naming your seat
// over one that does not, then the earlier entry. So a seat with an actor of its own gets its own entry
// wherever it sits in the file -- the Wraith's turret ("wraithantiinfantry") beats the Wraith ("wraith"),
// which also appears in the turret's name -- and a seat sharing its vehicle's actor (the Warthog's
// passenger) gets its own by naming the seat. An entry with a "seat" list is never picked while the seat
// is unknown: guessing would hand a driver the passenger's cameras.
int match_vehicle(const Table& t, const std::string& actor_name, uint8_t seat) {
    const std::string n = lower(actor_name);
    int def = -1, best = -1;
    std::size_t best_len = 0;
    bool best_seated = false;
    for (std::size_t i = 0; i < t.vehicles.size(); ++i) {
        const Vehicle& v = t.vehicles[i];
        if (v.is_default) { if (def < 0) def = static_cast<int>(i); continue; }
        const bool seated = v.seats != 0;
        if (seated && (v.seats & seat) == 0) continue;   // seat 0 (unknown) never passes a seat list
        for (const std::string& m : v.match) {
            if (m.empty() || n.find(m) == std::string::npos) continue;
            if (m.size() > best_len || (m.size() == best_len && seated && !best_seated)) {
                best = static_cast<int>(i); best_len = m.size(); best_seated = seated;
            }
        }
    }
    return best >= 0 ? best : def;
}

namespace {
Tether own_tether(const Camera& c) {
    Tether t;
    t.offset[0] = c.offset[0]; t.offset[1] = c.offset[1]; t.offset[2] = c.offset[2];
    for (int k = 0; k < 3; ++k) { t.leash_min[k] = c.leash_min[k]; t.leash_max[k] = c.leash_max[k]; }
    t.loc_view = c.loc_view; t.loc_yaw = c.loc_yaw; t.loc_pitch = c.loc_pitch; t.loc_roll = c.loc_roll;
    t.rot_yaw = c.rot_yaw; t.rot_pitch = c.rot_pitch; t.rot_roll = c.rot_roll;
    t.aim_marker = c.aim_marker;
    return t;
}
bool tracks_rotation(const Tether& t) { return t.rot_yaw || t.rot_pitch || t.rot_roll; }
bool starts_tethered(const std::string& n) { return lower(n).rfind("tethered", 0) == 0; }
// One view in another mode: the same kind of camera from the same place, hidden the same way.
bool same_view(const Camera& a, const Camera& b) {
    return a.type == b.type && a.origin == b.origin && a.origin_socket == b.origin_socket
        && a.collide == b.collide && a.collide_margin == b.collide_margin
        && a.hide_body == b.hide_body && a.hide_head == b.hide_head && a.tethering.empty() && b.tethering.empty();
}
std::string trim(std::string s) {
    while (!s.empty() && s.front() == ' ') s.erase(s.begin());
    while (!s.empty() && s.back() == ' ') s.pop_back();
    return s;
}
// What a camera's name adds to its group's: "Tethered Yaw HangGlider" in "HangGlider" = "Tethered Yaw".
std::string mode_name(const std::string& name, const std::string& base, const Tether& t) {
    if (lower(name) == lower(base)) return tracks_rotation(t) ? "Tethered" : "Untethered";
    const std::string ln = lower(name), lb = lower(base);
    const size_t at = ln.find(lb);
    if (at != std::string::npos) {
        const std::string rest = trim(name.substr(0, at) + name.substr(at + base.size()));
        if (!rest.empty()) return rest;
    }
    if (starts_tethered(name)) return "Tethered";   // "Tethered OTS" under "Over the Shoulder"
    return name;
}
} // namespace

int merge_tethered(Table& t, std::vector<std::vector<std::pair<int, int>>>* moved) {
    int folded = 0;
    if (moved != nullptr) moved->clear();
    for (Vehicle& v : t.vehicles) {
        std::vector<Camera> out;
        std::vector<std::pair<int, int>> where(v.cameras.size(), std::make_pair(0, 0));   // old -> (camera, mode)
        std::size_t i = 0;
        while (i < v.cameras.size()) {
            // A group: this camera, and every "Tethered ..." one straight after it that is the same view.
            std::size_t j = i + 1;
            if (!starts_tethered(v.cameras[i].name))
                while (j < v.cameras.size() && starts_tethered(v.cameras[j].name) && same_view(v.cameras[i], v.cameras[j])) ++j;
            if (j - i < 2) {
                where[i] = {static_cast<int>(out.size()), 0};
                out.push_back(v.cameras[i]);
                ++i;
                continue;
            }
            const int at = static_cast<int>(out.size());
            Camera c = v.cameras[i];                     // the group's own name, place and tracking
            c.aim_marker = -1;                           // each mode carries its own
            c.tethering.clear();
            // The one holding the world still first when it is not already; otherwise the file's order.
            std::vector<std::size_t> order;
            for (std::size_t k = i; k < j; ++k) order.push_back(k);
            for (std::size_t k = 0; k < order.size(); ++k)
                if (!tracks_rotation(own_tether(v.cameras[order[k]]))) {
                    const auto at_k = order.begin() + static_cast<std::ptrdiff_t>(k);
                    std::rotate(order.begin(), at_k, at_k + 1);
                    break;
                }
            for (std::size_t k = 0; k < order.size(); ++k) {
                const Camera& src = v.cameras[order[k]];
                Tether m = own_tether(src);
                m.name = mode_name(src.name, v.cameras[i].name, m);
                c.tethering.push_back(m);
                where[order[k]] = {at, static_cast<int>(k)};
            }
            // The camera's own fields are the first mode's, so the file names only what the others change.
            const Tether& m0 = c.tethering.front();
            for (int k = 0; k < 3; ++k) c.offset[k] = m0.offset[k];
            c.loc_view = m0.loc_view; c.loc_yaw = m0.loc_yaw; c.loc_pitch = m0.loc_pitch; c.loc_roll = m0.loc_roll;
            c.rot_yaw = m0.rot_yaw; c.rot_pitch = m0.rot_pitch; c.rot_roll = m0.rot_roll;
            out.push_back(c);
            folded += static_cast<int>(j - i) - 1;
            i = j;
        }
        if (v.default_camera >= 0 && v.default_camera < static_cast<int>(where.size())) {
            v.default_mode = where[static_cast<size_t>(v.default_camera)].second;
            v.default_camera = where[static_cast<size_t>(v.default_camera)].first;
        }
        v.cameras = std::move(out);
        if (moved != nullptr) moved->push_back(std::move(where));
    }
    return folded;
}

bool effective_aim_marker(const Vehicle& v, int ci, int mi) {
    if (ci < 0 || ci >= static_cast<int>(v.cameras.size())) return v.aim_marker;
    const Camera& c = v.cameras[static_cast<size_t>(ci)];
    if (!c.tethering.empty()) {
        const Tether m = c.mode(mi);
        if (m.aim_marker >= 0) return m.aim_marker != 0;
    }
    if (c.aim_marker >= 0) return c.aim_marker != 0;
    return v.aim_marker;
}

bool effective_hide_head(const Vehicle& v, int ci) {
    if (ci >= 0 && ci < static_cast<int>(v.cameras.size()) && v.cameras[static_cast<size_t>(ci)].hide_head >= 0)
        return v.cameras[static_cast<size_t>(ci)].hide_head != 0;
    return v.hide_head;
}

} // namespace halo::vehcampresets
