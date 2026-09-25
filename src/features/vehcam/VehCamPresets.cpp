#include "features/vehcam/VehCamPresets.hpp"

#include "core/JsonLite.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace halo::vehcampresets {

namespace {

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

bool ieq(const std::string& a, const char* b) { return lower(a) == lower(std::string(b)); }

Camera cam(const char* name, CamType type, Origin origin, float f, float r, float u, Rides rides,
           bool fy, bool fp, bool fr) {
    Camera c;
    c.name = name; c.type = type; c.origin = origin;
    c.offset[0] = f; c.offset[1] = r; c.offset[2] = u;
    c.rides = rides; c.follow_yaw = fy; c.follow_pitch = fp; c.follow_roll = fr;
    return c;
}

// The starter set every vehicle gets until the player tunes it. Onboard first: it is the closest to
// the view the owned camera was tuned with (at the vehicle, the world holding still), and the one a
// motion-sensitive player should land in. No "firstperson" entry: that hands the view to the older
// seat camera, which stays available by adding one to the file.
std::vector<Camera> starter_cameras() {
    return {
        cam("Onboard",                 CamType::Chase, Origin::Seat,    0.0f,    0.0f, 0.0f,   Rides::Vehicle, false, false, false),
        cam("Cockpit (tilts with it)", CamType::Chase, Origin::Seat,    0.0f,    0.0f, 0.0f,   Rides::Vehicle, true,  true,  true),
        cam("Chase",                   CamType::Chase, Origin::Vehicle, -450.0f, 0.0f, 180.0f, Rides::Vehicle, false, false, false),
        cam("Chase (turns with it)",   CamType::Chase, Origin::Vehicle, -450.0f, 0.0f, 180.0f, Rides::Vehicle, true,  false, false),
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
    bool num(const Value& v, const std::string& where, float& dst) {
        if (v.is_null()) return true;
        if (!v.is_num()) return type_error(v, where, "a number");
        dst = static_cast<float>(v.n);
        return true;
    }
    bool boolean(const Value& v, const std::string& where, bool& dst) {
        if (v.is_null()) return true;
        if (!v.is_bool()) return type_error(v, where, "true or false");
        dst = v.b;
        return true;
    }
    bool camera(const Value& v, const std::string& where, Camera& c) {
        if (!v.is_obj()) return type_error(v, where, "an object { \"name\": ..., \"offset\": [...] }");
        for (std::size_t k = 0; k < v.keys.size(); ++k) {
            const std::string& key = v.keys[k];
            const Value& x = v.items[k];
            const std::string w = where + "." + key;
            if (!key.empty() && key[0] == '_') continue;
            bool ok = true;
            if (key == "name") {
                if (!x.is_str()) return type_error(x, w, "text");
                c.name = x.s;
            } else if (key == "type") {
                if (!x.is_str()) return type_error(x, w, "\"chase\" or \"firstperson\"");
                if (ieq(x.s, "chase")) c.type = CamType::Chase;
                else if (ieq(x.s, "firstperson")) c.type = CamType::FirstPerson;
                else return type_error(x, w, "\"chase\" or \"firstperson\"");
            } else if (key == "origin") {
                if (!x.is_str()) return type_error(x, w, "\"vehicle\" or \"seat\"");
                if (ieq(x.s, "vehicle")) c.origin = Origin::Vehicle;
                else if (ieq(x.s, "seat")) c.origin = Origin::Seat;
                else return type_error(x, w, "\"vehicle\" or \"seat\"");
            } else if (key == "offset") {
                if (!x.is_arr() || x.items.size() > 3) return type_error(x, w, "[forward, right, up] in cm");
                for (std::size_t i = 0; i < x.items.size() && ok; ++i)
                    ok = num(x.items[i], w + "[" + std::to_string(i) + "]", c.offset[i]);
            } else if (key == "offsetRides") {
                if (!x.is_str()) return type_error(x, w, "\"vehicle\", \"vehicleYaw\" or \"view\"");
                if (ieq(x.s, "vehicle")) c.rides = Rides::Vehicle;
                else if (ieq(x.s, "vehicleYaw")) c.rides = Rides::VehicleYaw;
                else if (ieq(x.s, "view")) c.rides = Rides::View;
                else return type_error(x, w, "\"vehicle\", \"vehicleYaw\" or \"view\"");
            } else if (key == "viewFollows") {
                if (!x.is_arr()) return type_error(x, w, "a list of any of \"yaw\", \"pitch\", \"roll\" ([] = none)");
                c.follow_yaw = c.follow_pitch = c.follow_roll = false;
                for (std::size_t i = 0; i < x.items.size(); ++i) {
                    const Value& e = x.items[i];
                    const std::string we = w + "[" + std::to_string(i) + "]";
                    if (!e.is_str()) return type_error(e, we, "\"yaw\", \"pitch\" or \"roll\"");
                    if (ieq(e.s, "yaw")) c.follow_yaw = true;
                    else if (ieq(e.s, "pitch")) c.follow_pitch = true;
                    else if (ieq(e.s, "roll")) c.follow_roll = true;
                    else return type_error(e, we, "\"yaw\", \"pitch\" or \"roll\"");
                }
            } else if (key == "collide") {
                ok = boolean(x, w, c.collide);
            } else if (key == "collideMargin") {
                ok = num(x, w, c.collide_margin);
            } else if (key == "hideBody") {
                if (x.is_null()) { c.hide_body = -1; continue; }
                if (!x.is_bool()) return type_error(x, w, "true, false or null (null = hide it for seat cameras)");
                c.hide_body = x.b ? 1 : 0;
            } else {
                r.ignored.push_back(w);
            }
            if (!ok) return false;
        }
        return true;
    }
    bool vehicle_entry(const std::string& key, const Value& v, const std::string& where, Vehicle& out) {
        if (!v.is_obj()) return type_error(v, where, "an object { \"cameras\": [...] }");
        out = Vehicle{};
        out.name = key;
        out.is_default = ieq(key, "default");
        bool have_match = false, have_cams = false;
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
            } else if (fk == "defaultCamera") {
                if (!x.is_num() || x.n < 0.0 || x.n != std::floor(x.n)) return type_error(x, w, "a camera index from 0");
                out.default_camera = static_cast<int>(x.n);
            } else if (fk == "motionAim") {
                if (x.is_null()) { out.motion_aim = -1; continue; }
                if (!x.is_bool()) return type_error(x, w, "true, false or null (null = the vehaim setting)");
                out.motion_aim = x.b ? 1 : 0;
            } else if (fk == "cameras") {
                have_cams = true;
                if (!x.is_arr() || x.items.empty()) return type_error(x, w, "a list of at least one camera");
                if (static_cast<int>(x.items.size()) > kMaxCameras) return type_error(x, w, "at most 16 cameras");
                for (std::size_t i = 0; i < x.items.size(); ++i) {
                    Camera c;
                    c.name = "Camera " + std::to_string(i + 1);
                    if (!camera(x.items[i], w + "[" + std::to_string(i) + "]", c)) return false;
                    out.cameras.push_back(c);
                }
            } else {
                r.ignored.push_back(w);
            }
        }
        if (!have_cams) return type_error(v, where, "an object with a \"cameras\" list");
        if (!have_match && !out.is_default) out.match.push_back(lower(key));
        if (out.default_camera >= static_cast<int>(out.cameras.size())) out.default_camera = 0;
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

} // namespace

const char* type_name(CamType t)  { return t == CamType::FirstPerson ? "firstperson" : "chase"; }
const char* origin_name(Origin o) { return o == Origin::Seat ? "seat" : "vehicle"; }
const char* rides_name(Rides r) {
    switch (r) {
        case Rides::VehicleYaw: return "vehicleYaw";
        case Rides::View:       return "view";
        default:                return "vehicle";
    }
}

Table default_table() {
    // Vehicle actor names from the game's own assets (BP_<name>VehicleActor). The turrets are
    // separate vehicles (the Warthog's chaingun, the Scorpion's guns), so a gunner gets their own entry
    // -- and "WarthogVehicleActor" is chosen over "Warthog" precisely so it does not match the chaingun.
    Table t;
    t.vehicles.push_back(vehicle("Banshee",        {"bansheevehicleactor"}));
    t.vehicles.push_back(vehicle("Ghost",          {"ghostvehicleactor"}));
    t.vehicles.push_back(vehicle("Warthog gunner", {"warthogchaingunvehicleactor"}));
    t.vehicles.push_back(vehicle("Warthog",        {"warthogvehicleactor"}));
    t.vehicles.push_back(vehicle("Scorpion",       {"scorpion"}));
    t.vehicles.push_back(vehicle("Wraith",         {"wraith"}));
    t.vehicles.push_back(vehicle("Shade",          {"shade"}));
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

std::string table_to_json(const Table& t) {
    std::string s;
    s += "{\n";
    s += "  \"_readme\": [\n";
    s += "    \"VEHICLE CAMERAS. In a vehicle: LEFT X = next camera, LEFT Y = previous camera.\",\n";
    s += "    \"Each vehicle has its own list, used in order. The first entry whose 'match' text appears in the\",\n";
    s += "    \"vehicle's name is used (case does not matter); 'default' covers any vehicle not listed.\",\n";
    s += "    \"Camera fields -- all optional; anything left out takes its default:\",\n";
    s += "    \"  name           shown in the log when you switch\",\n";
    s += "    \"  type           chase (this mod's camera) | firstperson (the seat camera)\",\n";
    s += "    \"  origin         vehicle | seat : where 'offset' is measured from\",\n";
    s += "    \"  offset         [forward, right, up] in cm (negative forward = behind)\",\n";
    s += "    \"  offsetRides    vehicle (turns, pitches and rolls with it) | vehicleYaw (turns with it, stays level)\",\n";
    s += "    \"                 | view (swings round the vehicle as you turn your view)\",\n";
    s += "    \"  viewFollows    which vehicle motions turn your VIEW: [] = none (the world holds still), or any of\",\n";
    s += "    \"                 yaw, pitch, roll. Pitch or roll also follows yaw.\",\n";
    s += "    \"  collide        pull the camera in when a wall is in the way; collideMargin = cm to stop short\",\n";
    s += "    \"  hideBody       true | false: hide your character's body (left out = hidden for seat cameras)\",\n";
    s += "    \"Per vehicle: defaultCamera = the index (from 0) you start in; motionAim = true | false overrides vehaim.\",\n";
    s += "    \"Saved changes apply within a couple of seconds. This file is yours: updates never overwrite it,\",\n";
    s += "    \"and deleting it brings the built-in cameras back.\"\n";
    s += "  ],\n";
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
        if (v.motion_aim >= 0) s += std::string("      \"motionAim\": ") + (v.motion_aim ? "true" : "false") + ",\n";
        s += "      \"cameras\": [\n";
        for (std::size_t ci = 0; ci < v.cameras.size(); ++ci) {
            const Camera& c = v.cameras[ci];
            std::string follows;
            if (c.follow_yaw)   follows += std::string(follows.empty() ? "" : ", ") + "\"yaw\"";
            if (c.follow_pitch) follows += std::string(follows.empty() ? "" : ", ") + "\"pitch\"";
            if (c.follow_roll)  follows += std::string(follows.empty() ? "" : ", ") + "\"roll\"";
            s += "        { \"name\": " + quoted(c.name)
               + ", \"type\": \"" + type_name(c.type) + "\""
               + ", \"origin\": \"" + origin_name(c.origin) + "\""
               + ", \"offset\": [" + fmt_num(c.offset[0]) + ", " + fmt_num(c.offset[1]) + ", " + fmt_num(c.offset[2]) + "]"
               + ", \"offsetRides\": \"" + rides_name(c.rides) + "\""
               + ", \"viewFollows\": [" + follows + "]"
               + ", \"collide\": " + (c.collide ? "true" : "false")
               + ", \"collideMargin\": " + fmt_num(c.collide_margin)
               + (c.hide_body < 0 ? std::string() : std::string(", \"hideBody\": ") + (c.hide_body ? "true" : "false"))
               + " }"
               + (ci + 1 < v.cameras.size() ? ",\n" : "\n");
        }
        s += "      ]\n";
        s += std::string("    }") + (vi + 1 < t.vehicles.size() ? ",\n" : "\n");
    }
    s += "  }\n";
    s += "}\n";
    return s;
}

int match_vehicle(const Table& t, const std::string& actor_name) {
    const std::string n = lower(actor_name);
    int def = -1;
    for (std::size_t i = 0; i < t.vehicles.size(); ++i) {
        const Vehicle& v = t.vehicles[i];
        if (v.is_default) { if (def < 0) def = static_cast<int>(i); continue; }
        for (const std::string& m : v.match)
            if (!m.empty() && n.find(m) != std::string::npos) return static_cast<int>(i);
    }
    return def;
}

} // namespace halo::vehcampresets
