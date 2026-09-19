#include "HandPoseJson.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace halo::palettearm {

void default_hand_poses(HandPose out[kHandPoseCount]) {
    // Canonized 2026-09-18 from the user's own headset tuning of the first gesture build: the
    // pointing index (curl 0, segments 0/-0.85/-3, -6 deg on the knuckle) in point and pointdown;
    // the thumb-down set (20/0/20 deg on the thumb's base) in fist, pointdown and ok; the thumb-up
    // set (-20/0/-20 deg, ext -2) in thumbsup and point; thumb over 0.35 on the clenched fist,
    // thumb out 0.3 everywhere. OK: see below.
    for (int p = 0; p < kHandPoseCount; ++p) { out[p] = HandPose{}; out[p].thumb_out = 0.3f; }
    const auto fingers = [](HandPose& h, float i, float m, float r, float k, float t) {
        h.finger[0].curl = i; h.finger[1].curl = m; h.finger[2].curl = r; h.finger[3].curl = k; h.finger[4].curl = t;
    };
    const auto point_index = [](HandPose& h) {
        h.finger[0].curl = 0.0f; h.finger[0].seg[1] = -0.85f; h.finger[0].seg[2] = -3.0f; h.finger[0].rot[0][0] = -6.0f;
    };
    const auto thumb_down = [](HandPose& h) { h.finger[4].rot[0][0] = 20.0f;  h.finger[4].rot[0][2] = 20.0f; };
    const auto thumb_up   = [](HandPose& h) { h.finger[4].rot[0][0] = -20.0f; h.finger[4].rot[0][2] = -20.0f; h.thumb_ext = -2.0f; };
    HandPose& fist = out[static_cast<int>(HandPoseId::Fist)];
    HandPose& up   = out[static_cast<int>(HandPoseId::ThumbsUp)];
    HandPose& pt   = out[static_cast<int>(HandPoseId::Point)];
    HandPose& ptd  = out[static_cast<int>(HandPoseId::PointDown)];
    HandPose& ok   = out[static_cast<int>(HandPoseId::Ok)];
    fingers(out[static_cast<int>(HandPoseId::RestIndex)], 1, 0, 0, 0, 0);
    fingers(fist, 1, 1, 1, 1, 1);  thumb_down(fist); fist.thumb_over = 0.35f;
    fingers(up,   1, 1, 1, 1, -1); thumb_up(up);
    fingers(pt,   0, 1, 1, 1, -1); point_index(pt);  thumb_up(pt);
    fingers(ptd,  0, 1, 1, 1, 1);  point_index(ptd); thumb_down(ptd);
    // OK, CANONIZED 2026-09-18 from the user's own tuning in halo_vr_handposes.json: the index
    // curled with its joints eased back open (a ring with the thumb, not a fist), the other three
    // fanned -- each segment turned a little further out about its own z, the pinky most.
    fingers(ok,   1, 0, 0, 0, 1);  ok.thumb_over = 0.35f;
    const auto segs = [](FingerPose& f, float a, float b, float c) { f.seg[0] = a; f.seg[1] = b; f.seg[2] = c; };
    segs(ok.finger[0], -0.45f, -0.5f, -0.8f);
    segs(ok.finger[1],  0.25f, -0.5f, -0.9f);  ok.finger[1].rot[2][2] = -20.0f;
    segs(ok.finger[2], -0.25f,  0.0f,  0.0f);  ok.finger[2].rot[1][2] = -20.0f; ok.finger[2].rot[2][2] = -20.0f;
    segs(ok.finger[3], -0.7f,   0.0f,  0.0f);  ok.finger[3].rot[0][2] = -45.0f; ok.finger[3].rot[1][2] = -20.0f;
                                                ok.finger[3].rot[2][2] = -30.0f;
    segs(ok.finger[4],  0.5f,   0.0f,  0.0f);  ok.finger[4].rot[0][0] = 20.0f;  ok.finger[4].rot[0][2] = 5.0f;
                                                ok.finger[4].rot[2][2] = -30.0f;
}

namespace {

const char* const kFingerNames[kHandFingers] = {"index", "middle", "ring", "pinky", "thumb"};

const char* pose_inputs(int p) {
    switch (static_cast<HandPoseId>(p)) {
        case HandPoseId::Rest:      return "no grip, no trigger (thumb ignored)";
        case HandPoseId::RestIndex: return "no grip + trigger";
        case HandPoseId::Fist:      return "grip + trigger + thumb";
        case HandPoseId::ThumbsUp:  return "grip + trigger";
        case HandPoseId::Point:     return "grip";
        case HandPoseId::PointDown: return "grip + thumb";
        case HandPoseId::Ok:        return "no grip + trigger + thumb";
        default:                    return "";
    }
}

// ---- a small JSON reader: enough of RFC 8259 for a hand-edited file, strict about syntax --------
struct JVal {
    enum class T { Null, Bool, Num, Str, Arr, Obj } t{T::Null};
    double n{0.0};
    bool b{false};
    std::string s;
    std::vector<JVal> items;        // array elements, or object values
    std::vector<std::string> keys;  // object keys, parallel to items
    std::size_t at{0};              // byte offset, for error positions
};

struct Reader {
    const char* p;
    std::size_t len;
    std::size_t i{0};
    std::string err;
    std::size_t err_at{0};

    bool fail(const char* msg) { if (err.empty()) { err = msg; err_at = i; } return false; }
    void ws() {
        while (i < len && (p[i] == ' ' || p[i] == '\t' || p[i] == '\r' || p[i] == '\n')) ++i;
    }
    bool lit(const char* w) {
        const std::size_t n = std::strlen(w);
        if (len - i < n || std::strncmp(p + i, w, n) != 0) return false;
        i += n;
        return true;
    }
    bool str(std::string& out) {
        if (i >= len || p[i] != '"') return fail("expected a string");
        ++i;
        while (i < len && p[i] != '"') {
            char c = p[i++];
            if (static_cast<unsigned char>(c) < 0x20) return fail("control character inside a string");
            if (c == '\\') {
                if (i >= len) break;
                const char e = p[i++];
                switch (e) {
                    case '"': case '\\': case '/': c = e; break;
                    case 'n': c = '\n'; break;
                    case 't': c = '\t'; break;
                    case 'r': c = '\r'; break;
                    case 'b': c = '\b'; break;
                    case 'f': c = '\f'; break;
                    case 'u':   // names and notes only: keep the escape's low byte, enough for ASCII
                        if (len - i < 4) return fail("bad \\u escape");
                        c = static_cast<char>(std::strtol(std::string(p + i, 4).c_str(), nullptr, 16) & 0x7F);
                        i += 4;
                        break;
                    default: return fail("bad escape in a string");
                }
            }
            out.push_back(c);
        }
        if (i >= len) return fail("unterminated string");
        ++i;
        return true;
    }
    bool value(JVal& v, int depth) {
        if (depth > 16) return fail("nested too deeply");
        ws();
        v.at = i;
        if (i >= len) return fail("unexpected end of file");
        const char c = p[i];
        if (c == '{') {
            v.t = JVal::T::Obj; ++i; ws();
            if (i < len && p[i] == '}') { ++i; return true; }
            for (;;) {
                ws();
                std::string k;
                if (!str(k)) return false;
                ws();
                if (i >= len || p[i] != ':') return fail("expected ':' after a key");
                ++i;
                v.keys.push_back(k);
                v.items.emplace_back();
                if (!value(v.items.back(), depth + 1)) return false;
                ws();
                if (i < len && p[i] == ',') { ++i; continue; }
                if (i < len && p[i] == '}') { ++i; return true; }
                return fail("expected ',' or '}' (a missing comma, or a trailing one?)");
            }
        }
        if (c == '[') {
            v.t = JVal::T::Arr; ++i; ws();
            if (i < len && p[i] == ']') { ++i; return true; }
            for (;;) {
                v.items.emplace_back();
                if (!value(v.items.back(), depth + 1)) return false;
                ws();
                if (i < len && p[i] == ',') { ++i; continue; }
                if (i < len && p[i] == ']') { ++i; return true; }
                return fail("expected ',' or ']'");
            }
        }
        if (c == '"') { v.t = JVal::T::Str; return str(v.s); }
        if (lit("true"))  { v.t = JVal::T::Bool; v.b = true;  return true; }
        if (lit("false")) { v.t = JVal::T::Bool; v.b = false; return true; }
        if (lit("null"))  { v.t = JVal::T::Null; return true; }
        if (c == '-' || (c >= '0' && c <= '9')) {
            // strtod would read past the token on its own; bound it to the JSON number characters.
            std::size_t j = i;
            while (j < len && (std::strchr("+-.eE0123456789", p[j]) != nullptr)) ++j;
            const std::string tok(p + i, j - i);
            char* end = nullptr;
            const double d = std::strtod(tok.c_str(), &end);
            if (end == nullptr || *end != 0) return fail("bad number");
            if (!std::isfinite(d)) return fail("number out of range");
            v.t = JVal::T::Num; v.n = d; i = j;
            return true;
        }
        return fail("expected a value (a number, [ ], { }, \"text\", true, false or null)");
    }
};

void line_col(const char* text, std::size_t at, int& line, int& col) {
    line = 1; col = 1;
    for (std::size_t k = 0; k < at && text[k] != 0; ++k) {
        if (text[k] == '\n') { ++line; col = 1; } else { ++col; }
    }
}

// ---- applying the tree --------------------------------------------------------------------------
struct Apply {
    const char* text;
    HandPoseJsonResult& r;
    bool type_error(const JVal& v, const std::string& where, const char* want) {
        int line = 0, col = 0;
        line_col(text, v.at, line, col);
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%s must be %s (line %d col %d)", where.c_str(), want, line, col);
        r.error = buf;
        return false;
    }
    bool num(const JVal& v, const std::string& where, float& dst) {
        if (v.t == JVal::T::Null) return true;                  // null = keep
        if (v.t != JVal::T::Num) return type_error(v, where, "a number (or null)");
        dst = static_cast<float>(v.n);
        ++r.values;
        return true;
    }
    bool nums(const JVal& v, const std::string& where, float* dst, int n) {
        if (v.t == JVal::T::Null) return true;
        if (v.t != JVal::T::Arr) return type_error(v, where, "a list of numbers");
        if (static_cast<int>(v.items.size()) > n) return type_error(v, where, n == 3 ? "at most 3 numbers" : "shorter");
        for (std::size_t k = 0; k < v.items.size(); ++k)
            if (!num(v.items[k], where + "[" + std::to_string(k) + "]", dst[k])) return false;
        return true;
    }
    bool finger(const JVal& v, const std::string& where, HandPose& pose, int f) {
        if (v.t == JVal::T::Null) return true;
        if (v.t != JVal::T::Obj) return type_error(v, where, "an object { \"curl\": ... }");
        FingerPose& fp = pose.finger[f];
        for (std::size_t k = 0; k < v.keys.size(); ++k) {
            const std::string& key = v.keys[k];
            const JVal& x = v.items[k];
            const std::string w = where + "." + key;
            if (!key.empty() && key[0] == '_') continue;
            bool ok = true;
            if (key == "curl") ok = num(x, w, fp.curl);
            else if (key == "seg") ok = nums(x, w, fp.seg, kHandSegments);
            else if (key == "rot") {
                if (x.t == JVal::T::Null) continue;
                if (x.t != JVal::T::Arr || static_cast<int>(x.items.size()) > kHandSegments)
                    return type_error(x, w, "a list of up to 3 [x, y, z] lists");
                for (std::size_t s = 0; s < x.items.size() && ok; ++s)
                    ok = nums(x.items[s], w + "[" + std::to_string(s) + "]", fp.rot[s], 3);
            }
            else if (f == 4 && key == "over") ok = num(x, w, pose.thumb_over);
            else if (f == 4 && key == "ext")  ok = num(x, w, pose.thumb_ext);
            else if (f == 4 && key == "out")  ok = num(x, w, pose.thumb_out);
            else r.ignored.push_back(w);
            if (!ok) return false;
        }
        return true;
    }
    bool pose(const JVal& v, const std::string& where, HandPose& h) {
        if (v.t == JVal::T::Null) return true;
        if (v.t != JVal::T::Obj) return type_error(v, where, "an object of fingers");
        for (std::size_t k = 0; k < v.keys.size(); ++k) {
            const std::string& key = v.keys[k];
            if (!key.empty() && key[0] == '_') continue;
            int f = -1;
            for (int n = 0; n < kHandFingers; ++n) if (key == kFingerNames[n]) f = n;
            if (f < 0) { r.ignored.push_back(where + "." + key); continue; }
            if (!finger(v.items[k], where + "." + key, h, f)) return false;
        }
        return true;
    }
};

// Shortest text that reads back as the same float.
std::string fmt(float v) {
    char buf[32];
    for (int prec = 6; prec <= 9; ++prec) {
        std::snprintf(buf, sizeof(buf), "%.*g", prec, static_cast<double>(v));
        if (static_cast<float>(std::strtod(buf, nullptr)) == v) break;
    }
    if (std::strcmp(buf, "-0") == 0) return "0";
    return buf;
}

} // namespace

HandPoseJsonResult hand_poses_from_json(const char* text, std::size_t len, HandPose table[kHandPoseCount]) {
    HandPoseJsonResult r;
    if (text == nullptr || table == nullptr) { r.error = "no text"; return r; }
    // A UTF-8 byte-order mark (Notepad adds one) is not JSON; step over it.
    std::size_t skip = (len >= 3 && static_cast<unsigned char>(text[0]) == 0xEF &&
                        static_cast<unsigned char>(text[1]) == 0xBB && static_cast<unsigned char>(text[2]) == 0xBF) ? 3 : 0;
    Reader rd{text + skip, len - skip};
    JVal root;
    bool ok = rd.value(root, 0);
    if (ok) { rd.ws(); if (rd.i < rd.len) ok = rd.fail("unexpected text after the closing '}'"); }
    if (!ok) {
        int line = 0, col = 0;
        line_col(text + skip, rd.err_at, line, col);
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%s (line %d col %d)", rd.err.c_str(), line, col);
        r.error = buf;
        return r;
    }
    HandPose work[kHandPoseCount];
    for (int p = 0; p < kHandPoseCount; ++p) work[p] = table[p];
    Apply ap{text + skip, r};
    if (root.t != JVal::T::Obj) { ap.type_error(root, "the file", "an object { ... }"); return r; }
    for (std::size_t k = 0; k < root.keys.size(); ++k) {
        const std::string& key = root.keys[k];
        const JVal& v = root.items[k];
        if ((!key.empty() && key[0] == '_') || key == "version") continue;
        if (key != "poses") { r.ignored.push_back(key); continue; }
        if (v.t != JVal::T::Obj) { ap.type_error(v, "poses", "an object of poses"); return r; }
        for (std::size_t q = 0; q < v.keys.size(); ++q) {
            const std::string& pn = v.keys[q];
            if (!pn.empty() && pn[0] == '_') continue;
            int p = -1;
            for (int n = 0; n < kHandPoseCount; ++n) if (pn == hand_pose_name(static_cast<HandPoseId>(n))) p = n;
            if (p < 0) { r.ignored.push_back("poses." + pn); continue; }
            if (!ap.pose(v.items[q], pn, work[p])) return r;
        }
    }
    for (int p = 0; p < kHandPoseCount; ++p) table[p] = work[p];
    r.ok = true;
    return r;
}

std::string hand_poses_to_json(const HandPose table[kHandPoseCount]) {
    std::string o;
    o += "{\n";
    o += "  \"_about\": [\n";
    o += "    \"Hand poses for the palette arms (armdriver=2). Save to apply: read again within ~2 s.\",\n";
    o += "    \"Each pose is picked by the controller inputs in its _inputs line and eased into.\",\n";
    o += "    \"curl: -1 = the recorded open hand, 0 = relaxed, 1 = the recorded fist; past +-1 extrapolates.\",\n";
    o += "    \"seg: curl ADDED to one segment [knuckle, second, last], along its own arc.\",\n";
    o += "    \"rot: degrees about each segment's own [x, y, z], one list per segment.\",\n";
    o += "    \"thumb only -- over: a curled thumb carried past the fist; ext: an open thumb's outer joints\",\n";
    o += "    \"past the open hand (negative = short of it); out: a curled thumb's base turned back out.\",\n";
    o += "    \"Every number is unbounded. null or a missing value = the built-in value. Keys starting\",\n";
    o += "    \"with _ are notes. pahandrest (cfg) still adds to the relaxed fingers of rest, index and ok.\",\n";
    o += "    \"Delete this file to go back to the built-in poses; it is written again on the next launch.\"\n";
    o += "  ],\n";
    o += "  \"version\": 1,\n";
    o += "  \"poses\": {\n";
    for (int p = 0; p < kHandPoseCount; ++p) {
        const HandPose& h = table[p];
        o += "    \"";
        o += hand_pose_name(static_cast<HandPoseId>(p));
        o += "\": {\n      \"_inputs\": \"";
        o += pose_inputs(p);
        o += "\",\n";
        for (int f = 0; f < kHandFingers; ++f) {
            const FingerPose& fp = h.finger[f];
            char pad[8];
            std::snprintf(pad, sizeof(pad), "%*s", 7 - static_cast<int>(std::strlen(kFingerNames[f])), "");
            o += "      \"";
            o += kFingerNames[f];
            o += "\": ";
            o += pad;
            o += "{ \"curl\": " + fmt(fp.curl);
            o += ", \"seg\": [" + fmt(fp.seg[0]) + ", " + fmt(fp.seg[1]) + ", " + fmt(fp.seg[2]) + "]";
            o += ", \"rot\": [";
            for (int s = 0; s < kHandSegments; ++s) {
                o += "[" + fmt(fp.rot[s][0]) + ", " + fmt(fp.rot[s][1]) + ", " + fmt(fp.rot[s][2]) + "]";
                if (s + 1 < kHandSegments) o += ", ";
            }
            o += "]";
            if (f == 4) o += ", \"over\": " + fmt(h.thumb_over) + ", \"ext\": " + fmt(h.thumb_ext) + ", \"out\": " + fmt(h.thumb_out);
            o += (f + 1 < kHandFingers) ? " },\n" : " }\n";
        }
        o += (p + 1 < kHandPoseCount) ? "    },\n" : "    }\n";
    }
    o += "  }\n}\n";
    return o;
}

} // namespace halo::palettearm
