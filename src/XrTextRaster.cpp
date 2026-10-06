// <Windows.h>'s min/max macros would eat every std::min/std::max below.
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "XrTextRaster.hpp"

#include <Windows.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace halo::xrtextraster {

namespace {

// ---- markup ---------------------------------------------------------------------------------------
enum class Kind { Title, Heading, Bullet, Body, Gap };
struct Run  { std::wstring text; bool bold = false; bool italic = false; bool caution = false; };
struct Line { Kind kind = Kind::Body; std::vector<Run> runs; };

std::wstring widen(const std::string& s) {
    if (s.empty()) return std::wstring();
    const int n = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, s.data(), (int)s.size(), nullptr, 0);
    if (n <= 0) {   // not UTF-8: take it byte for byte, which is right for ASCII and Latin-1
        std::wstring w;
        w.reserve(s.size());
        for (const unsigned char c : s) w.push_back((wchar_t)c);
        return w;
    }
    std::wstring w((size_t)n, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.data(), (int)s.size(), &w[0], n);
    return w;
}

// **bold**, *italic* and ==caution==, toggled as they are met. An unmatched marker simply styles the rest
// of the line -- the forgiving reading of a hand-typed line, and never an error. A lone '=' is text.
std::vector<Run> parse_inline(const std::wstring& s) {
    std::vector<Run> runs;
    Run cur;
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == L'*') {
            const bool dbl = (i + 1 < s.size() && s[i + 1] == L'*');
            if (!cur.text.empty()) { runs.push_back(cur); cur.text.clear(); }
            if (dbl) { cur.bold = !cur.bold; ++i; } else { cur.italic = !cur.italic; }
            continue;
        }
        if (s[i] == L'=' && i + 1 < s.size() && s[i + 1] == L'=') {
            if (!cur.text.empty()) { runs.push_back(cur); cur.text.clear(); }
            cur.caution = !cur.caution;
            ++i;
            continue;
        }
        cur.text.push_back(s[i]);
    }
    if (!cur.text.empty()) runs.push_back(cur);
    return runs;
}

std::vector<Line> parse_markup(const std::wstring& text) {
    std::vector<Line> out;
    size_t start = 0;
    for (;;) {
        size_t end = text.find(L'\n', start);
        if (end == std::wstring::npos) end = text.size();
        std::wstring l = text.substr(start, end - start);
        while (!l.empty() && (l.back() == L'\r' || l.back() == L' ' || l.back() == L'\t')) l.pop_back();
        size_t lead = 0;
        while (lead < l.size() && (l[lead] == L' ' || l[lead] == L'\t')) ++lead;
        const std::wstring t = l.substr(lead);
        Line ln;
        if (t.empty())                                       { ln.kind = Kind::Gap; }
        else if (t.rfind(L"## ", 0) == 0)                    { ln.kind = Kind::Heading; ln.runs = parse_inline(t.substr(3)); }
        else if (t.rfind(L"# ", 0) == 0)                     { ln.kind = Kind::Title;   ln.runs = parse_inline(t.substr(2)); }
        else if (t.rfind(L"- ", 0) == 0 || t.rfind(L"* ", 0) == 0)
                                                             { ln.kind = Kind::Bullet;  ln.runs = parse_inline(t.substr(2)); }
        else                                                 { ln.kind = Kind::Body;    ln.runs = parse_inline(t); }
        out.push_back(std::move(ln));
        if (end >= text.size()) break;
        start = end + 1;
    }
    // Leading and trailing gaps would only pad the block.
    while (!out.empty() && out.back().kind == Kind::Gap) out.pop_back();
    while (!out.empty() && out.front().kind == Kind::Gap) out.erase(out.begin());
    return out;
}

// ---- layout ---------------------------------------------------------------------------------------
struct TStyle { float size = 0.0f; int weight = FW_NORMAL; bool italic = false; };
struct Frag   { int x = 0; std::wstring text; TStyle st; bool caution = false; };
struct VLine  { int top = 0; int height = 0; int ascent = 0; int text_h = 0; int width = 0; bool accent = false;
                std::vector<Frag> frags; bool bullet = false; TStyle bullet_st; };

// One small font cache per render pass: a handful of (size, weight, italic) faces.
struct Fonts {
    struct E { int px; int weight; bool italic; HFONT f; };
    std::vector<E> list;
    HFONT get(const TStyle& s) {
        const int px = std::max(6, (int)std::lround(s.size));
        for (const E& e : list) if (e.px == px && e.weight == s.weight && e.italic == s.italic) return e.f;
        HFONT f = CreateFontW(-px, 0, 0, 0, s.weight, s.italic ? TRUE : FALSE, FALSE, FALSE, DEFAULT_CHARSET,
                              OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY,
                              DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
        list.push_back(E{px, s.weight, s.italic, f});
        return f;
    }
    ~Fonts() { for (const E& e : list) if (e.f != nullptr) DeleteObject(e.f); }
};

TStyle line_style(Kind k, float base) {
    switch (k) {
        case Kind::Title:   return TStyle{base * 1.40f, FW_BOLD, false};
        case Kind::Heading: return TStyle{base * 1.12f, FW_SEMIBOLD, false};
        default:            return TStyle{base, FW_NORMAL, false};
    }
}
TStyle run_style(const TStyle& ls, const Run& r) {
    TStyle s = ls;
    if (r.bold) s.weight = std::max(s.weight, (int)FW_BOLD);
    s.italic = r.italic;
    return s;
}

// Lay the lines out at `base` px, wrapping at `max_w`. Returns the block's height; `fits` is false
// when a single word is wider than the panel (only a smaller size cures that).
int layout(HDC dc, Fonts& fonts, const std::vector<Line>& lines, float base, int max_w,
           std::vector<VLine>& out, int* block_w, bool* fits) {
    out.clear();
    *fits = true;
    int y = 0, bw = 0;
    for (const Line& ln : lines) {
        if (ln.kind == Kind::Gap) { y += (int)std::lround(base * 0.5f); continue; }
        const TStyle ls = line_style(ln.kind, base);
        const int indent = (ln.kind == Kind::Bullet) ? (int)std::lround(base * 1.1f) : 0;
        TEXTMETRICW tm{};
        SelectObject(dc, fonts.get(ls));
        GetTextMetricsW(dc, &tm);
        const int lh = (int)std::lround((float)tm.tmHeight * 1.18f);
        VLine cur;
        auto start_line = [&]() {
            cur = VLine{};
            cur.top = y; cur.height = lh; cur.ascent = tm.tmAscent; cur.text_h = tm.tmHeight;
            cur.accent = (ln.kind == Kind::Title);
        };
        start_line();
        if (ln.kind == Kind::Bullet) { cur.bullet = true; cur.bullet_st = ls; }
        int x = indent;
        auto flush = [&]() {
            bw = std::max(bw, cur.width);
            out.push_back(cur);
            y += lh;
            start_line();
            x = indent;
        };
        for (const Run& r : ln.runs) {
            const TStyle st = run_style(ls, r);
            SelectObject(dc, fonts.get(st));
            // Words WITH their trailing spaces, so a wrapped line keeps its spacing and a run boundary
            // inside a word ("**bold**text") does not insert one.
            size_t i = 0;
            while (i < r.text.size()) {
                size_t j = i;
                while (j < r.text.size() && r.text[j] != L' ') ++j;
                while (j < r.text.size() && r.text[j] == L' ') ++j;
                const std::wstring word = r.text.substr(i, j - i);
                SIZE ext{};
                GetTextExtentPoint32W(dc, word.c_str(), (int)word.size(), &ext);
                // Measured WITHOUT its trailing spaces for the fit test: a line may end in a space.
                std::wstring bare = word;
                while (!bare.empty() && bare.back() == L' ') bare.pop_back();
                SIZE bext{};
                GetTextExtentPoint32W(dc, bare.c_str(), (int)bare.size(), &bext);
                if (x + bext.cx > max_w && x > indent) flush();
                if (indent + bext.cx > max_w) *fits = false;
                cur.frags.push_back(Frag{x, word, st, r.caution});
                cur.width = std::max(cur.width, x + (int)bext.cx);
                x += ext.cx;
                i = j;
            }
        }
        flush();
    }
    *block_w = bw;
    return y;
}

// Signed distance from (px, py) to a rounded rectangle; <= 0 inside.
float rounded_rect_sd(float px, float py, float x0, float y0, float x1, float y1, float r) {
    const float cx = (x0 + x1) * 0.5f, cy = (y0 + y1) * 0.5f;
    const float hx = (x1 - x0) * 0.5f - r, hy = (y1 - y0) * 0.5f - r;
    const float qx = std::fabs(px - cx) - hx, qy = std::fabs(py - cy) - hy;
    const float ox = std::max(qx, 0.0f), oy = std::max(qy, 0.0f);
    return std::sqrt(ox * ox + oy * oy) + std::min(std::max(qx, qy), 0.0f) - r;
}

} // namespace

// GDI renders WHITE ON BLACK and the luminance is read back as coverage, exactly as XrLayer's "Grip"
// label does, and for the same reasons: GDI has no alpha, and ClearType's per-channel coverage would
// fringe on an HMD's optics -- so ANTIALIASED_QUALITY.
bool rasterise(const std::string& markup, int w, int h, bool bgra, const Style& style, std::vector<uint8_t>& out) {
    if (w <= 0 || h <= 0) return false;
    const std::vector<Line> lines = parse_markup(widen(markup));
    HDC dc = CreateCompatibleDC(nullptr);
    if (dc == nullptr) return false;
    BITMAPINFO bi{};
    bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bi.bmiHeader.biWidth = w;
    bi.bmiHeader.biHeight = -h;   // top-down, like the atlas
    bi.bmiHeader.biPlanes = 1;
    bi.bmiHeader.biBitCount = 32;
    bi.bmiHeader.biCompression = BI_RGB;
    void* bits = nullptr;
    HBITMAP bmp = CreateDIBSection(dc, &bi, DIB_RGB_COLORS, &bits, nullptr, 0);
    if (bmp == nullptr || bits == nullptr) { DeleteDC(dc); return false; }
    ZeroMemory(bits, (size_t)w * (size_t)h * 4);
    const HGDIOBJ old_bmp = SelectObject(dc, bmp);
    const HGDIOBJ old_font = GetCurrentObject(dc, OBJ_FONT);

    std::vector<VLine> vl;
    int bw = 0, bh = 0;
    const int pad = std::max(8, h / 16);
    {
        Fonts fonts;
        const int max_w = w - 4 * pad;
        const int max_h = h - 4 * pad;
        float base = (float)h * 0.105f * std::max(0.2f, style.scale);
        bool fits = true;
        // MEASURE, THEN FIT: shrink until the block fits the panel (text size is very nearly linear in
        // font height, so a few passes settle it). Never grows past the requested size.
        for (int pass = 0; pass < 8; ++pass) {
            bh = layout(dc, fonts, lines, base, max_w, vl, &bw, &fits);
            if (bh <= max_h && fits) break;
            const float k = (bh > max_h) ? std::max(0.5f, (float)max_h / (float)std::max(bh, 1)) : 0.85f;
            base *= std::min(k, 0.97f);
            if (base < 7.0f) { base = 7.0f; bh = layout(dc, fonts, lines, base, max_w, vl, &bw, &fits); break; }
        }

        // The block, centred; its lines left-aligned within it. ONE CHANNEL PER COLOUR: ordinary text is
        // drawn in green, ==caution== text in red, and each channel read back is that colour's coverage
        // (greyscale antialiasing, so a channel is an exact mask) -- two colours from one GDI pass.
        const int x0 = (w - bw) / 2, y0 = (h - bh) / 2;
        SetBkMode(dc, TRANSPARENT);
        SetTextAlign(dc, TA_BASELINE | TA_LEFT);
        for (const VLine& v : vl) {
            const int base_y = y0 + v.top + (v.height - v.text_h) / 2 + v.ascent;
            if (v.bullet) {
                SelectObject(dc, fonts.get(v.bullet_st));
                SetTextColor(dc, RGB(0, 255, 0));
                const int bx = x0 + (int)std::lround(v.bullet_st.size * 0.25f);
                TextOutW(dc, bx, base_y, L"\x2022", 1);
            }
            for (const Frag& f : v.frags) {
                SelectObject(dc, fonts.get(f.st));
                SetTextColor(dc, f.caution ? RGB(255, 0, 0) : RGB(0, 255, 0));
                TextOutW(dc, x0 + f.x, base_y, f.text.c_str(), (int)f.text.size());
            }
        }
        GdiFlush();
        SelectObject(dc, old_font);   // before the cache deletes the faces it made

        // Composite: text over an optional dark rounded backing, PREMULTIPLIED.
        std::vector<unsigned char> row_accent((size_t)h, 0);
        for (const VLine& v : vl)
            if (v.accent)
                for (int yy = std::max(0, y0 + v.top); yy < std::min(h, y0 + v.top + v.height); ++yy)
                    row_accent[(size_t)yy] = 1;
        const float bg_a = std::clamp(style.bg, 0.0f, 1.0f);
        const float bx0 = (float)(x0 - pad), by0 = (float)(y0 - pad);
        const float bx1 = (float)(x0 + bw + pad), by1 = (float)(y0 + bh + pad);
        const float rad = std::min((float)pad * 1.5f, (by1 - by0) * 0.5f);
        const float bgc[3] = {0.035f, 0.045f, 0.065f};
        const float body[3] = {0.94f, 0.95f, 0.97f};
        const float accent[3] = {0.55f, 0.85f, 1.00f};
        const float caution[3] = {1.00f, 0.86f, 0.22f};   // yellow
        out.assign((size_t)w * (size_t)h * 4, 0);
        const uint8_t* src = (const uint8_t*)bits;
        for (int y = 0; y < h; ++y) {
            const float* tc = row_accent[(size_t)y] ? accent : body;
            for (int x = 0; x < w; ++x) {
                const uint8_t* s = src + ((size_t)y * (size_t)w + (size_t)x) * 4;   // B, G, R, x
                const float cn = (float)s[1] / 255.0f;   // ordinary text (green)
                const float cc = (float)s[2] / 255.0f;   // caution text (red)
                const float cov = std::max(cn, cc);
                // The text's colour: the two masks' colours weighted by their coverage, so an edge where
                // two neighbouring glyphs meet stays premultiplied (never brighter than its coverage).
                float t[3] = {0.0f, 0.0f, 0.0f};
                if (cn + cc > 0.0f)
                    for (int k = 0; k < 3; ++k) t[k] = (tc[k] * cn + caution[k] * cc) / (cn + cc);
                float ba = 0.0f;
                if (bg_a > 0.0f && !vl.empty()) {
                    const float d = rounded_rect_sd((float)x + 0.5f, (float)y + 0.5f, bx0, by0, bx1, by1, rad);
                    ba = bg_a * std::clamp(0.5f - d, 0.0f, 1.0f);
                }
                const float a = cov + ba * (1.0f - cov);
                const float r = t[0] * cov + bgc[0] * ba * (1.0f - cov);
                const float g = t[1] * cov + bgc[1] * ba * (1.0f - cov);
                const float b = t[2] * cov + bgc[2] * ba * (1.0f - cov);
                uint8_t* p = out.data() + ((size_t)y * (size_t)w + (size_t)x) * 4;
                const uint8_t R = (uint8_t)std::lround(std::clamp(r, 0.0f, 1.0f) * 255.0f);
                const uint8_t G = (uint8_t)std::lround(std::clamp(g, 0.0f, 1.0f) * 255.0f);
                const uint8_t B = (uint8_t)std::lround(std::clamp(b, 0.0f, 1.0f) * 255.0f);
                p[0] = bgra ? B : R;
                p[1] = G;
                p[2] = bgra ? R : B;
                p[3] = (uint8_t)std::lround(std::clamp(a, 0.0f, 1.0f) * 255.0f);
            }
        }
    }
    SelectObject(dc, old_bmp);
    DeleteObject(bmp);
    DeleteDC(dc);
    return true;
}

} // namespace halo::xrtextraster
