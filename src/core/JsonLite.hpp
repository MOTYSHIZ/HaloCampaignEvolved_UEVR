#pragma once

// A SMALL JSON READER for the hand-edited files a player keeps in the profile: enough of RFC 8259,
// strict about syntax, header-only and pure (no engine, no project headers). Object keys keep their
// file order, and every value remembers its byte offset so an error can name "line L col C".
//
// palettearm/HandPoseJson.cpp holds an older private copy of the same reader: that folder must build
// with no include paths at all (Scripts\Verify-PaletteArm.ps1 compiles it out of tree), so it cannot
// include this one without that script changing too. New files use this header.

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace halo::jsonlite {

struct Value {
    enum class T { Null, Bool, Num, Str, Arr, Obj } t{T::Null};
    double n{0.0};
    bool b{false};
    std::string s;
    std::vector<Value> items;        // array elements, or object values
    std::vector<std::string> keys;   // object keys, parallel to items
    std::size_t at{0};               // byte offset, for error positions

    bool is_null() const { return t == T::Null; }
    bool is_obj() const  { return t == T::Obj; }
    bool is_arr() const  { return t == T::Arr; }
    bool is_num() const  { return t == T::Num; }
    bool is_str() const  { return t == T::Str; }
    bool is_bool() const { return t == T::Bool; }
    // The member called `key`, or null. Linear: these files are small.
    const Value* get(const char* key) const {
        if (t != T::Obj) return nullptr;
        for (std::size_t i = 0; i < keys.size(); ++i)
            if (keys[i] == key) return &items[i];
        return nullptr;
    }
};

inline void line_col(const char* text, std::size_t at, int& line, int& col) {
    line = 1; col = 1;
    for (std::size_t k = 0; k < at && text[k] != 0; ++k) {
        if (text[k] == '\n') { ++line; col = 1; } else { ++col; }
    }
}

namespace detail {
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
    bool value(Value& v, int depth) {
        if (depth > 16) return fail("nested too deeply");
        ws();
        v.at = i;
        if (i >= len) return fail("unexpected end of file");
        const char c = p[i];
        if (c == '{') {
            v.t = Value::T::Obj; ++i; ws();
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
            v.t = Value::T::Arr; ++i; ws();
            if (i < len && p[i] == ']') { ++i; return true; }
            for (;;) {
                v.items.emplace_back();
                if (!value(v.items.back(), depth + 1)) return false;
                ws();
                if (i < len && p[i] == ',') { ++i; continue; }
                if (i < len && p[i] == ']') { ++i; return true; }
                return fail("expected ',' or ']' (a missing comma, or a trailing one?)");
            }
        }
        if (c == '"') { v.t = Value::T::Str; return str(v.s); }
        if (lit("true"))  { v.t = Value::T::Bool; v.b = true;  return true; }
        if (lit("false")) { v.t = Value::T::Bool; v.b = false; return true; }
        if (lit("null"))  { v.t = Value::T::Null; return true; }
        if (c == '-' || (c >= '0' && c <= '9')) {
            // strtod would read past the token on its own; bound it to the JSON number characters.
            std::size_t j = i;
            while (j < len && (std::strchr("+-.eE0123456789", p[j]) != nullptr)) ++j;
            const std::string tok(p + i, j - i);
            char* end = nullptr;
            const double d = std::strtod(tok.c_str(), &end);
            if (end == nullptr || *end != 0) return fail("bad number");
            if (!std::isfinite(d)) return fail("number out of range");
            v.t = Value::T::Num; v.n = d; i = j;
            return true;
        }
        return fail("expected a value (a number, [ ], { }, \"text\", true, false or null)");
    }
};
} // namespace detail

// Parse the whole of `text`. On failure `err` reads "line L col C: what", and `out` is unspecified.
inline bool parse(const char* text, std::size_t len, Value& out, std::string& err) {
    detail::Reader r{text, len};
    out = Value{};
    bool ok = r.value(out, 0);
    if (ok) {
        r.ws();
        if (r.i < len) ok = r.fail("unexpected text after the end of the document");
    }
    if (!ok) {
        int line = 0, col = 0;
        line_col(text, r.err_at, line, col);
        char buf[256];
        std::snprintf(buf, sizeof(buf), "line %d col %d: %s", line, col, r.err.c_str());
        err = buf;
    }
    return ok;
}

} // namespace halo::jsonlite
