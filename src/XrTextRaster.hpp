#pragma once

// THE TEXT PANEL'S RASTERISER: markup in, a premultiplied RGBA panel out. GDI only -- no engine, no
// project headers -- so it can be run out of tree and its output LOOKED AT (the text panel's layout is
// otherwise only visible in a headset). XrText.cpp owns when and where a panel shows.
//
// MARKUP, a small subset of Markdown, one line per line:
//   # Title        large, bold, accent colour
//   ## Heading     medium, semibold
//   - item         a bullet (also "* item")
//   (blank line)   a gap
//   anything else  body text
// and inline, anywhere: **bold** and *italic*. Long lines wrap at word boundaries, and the whole block
// shrinks to fit when it would not. UTF-8 in.

#include <cstdint>
#include <string>
#include <vector>

namespace halo::xrtextraster {

struct Style {
    float scale = 1.0f;   // text size relative to the panel (xrtextscale); shrinks further to fit
    float bg = 0.55f;     // the dark rounded backing's opacity (xrtextbg); 0 = text only
};

// Rasterise `markup` into a w x h panel, top row first, PREMULTIPLIED, in R,G,B,A order -- or B,G,R,A
// when `bgra`. False only when GDI could not give us a bitmap.
bool rasterise(const std::string& markup, int w, int h, bool bgra, const Style& style,
               std::vector<uint8_t>& out);

} // namespace halo::xrtextraster
