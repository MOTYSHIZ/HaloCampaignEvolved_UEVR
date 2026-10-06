#pragma once

// THE TEXT PANEL -- a short notice on the compositor layer (XrLayer slot 11) that fades in, holds and
// fades out. Crisp, never occluded, never lit or bloomed: it is composited after the game's image.
// Any feature can show one; the vehicle camera readout is the first.
//
// MARKUP, a small subset of Markdown, one line per line:
//   # Title                   large, bold, accent colour
//   ## Heading                medium, semibold
//   - item                    a bullet (indented)
//   (blank line)              a gap
//   anything else             body text
// and inline, anywhere: **bold** and *italic*. Long lines wrap at word boundaries, and the whole block
// shrinks to fit when it would not. Plain ASCII/Latin-1 text; no links, colours or images.
//
// ONE PANEL AT A TIME: a new notice replaces the one showing. It is placed ONCE, when it appears --
// in front of where you are looking (riding with your camera, not turning with your head), or at a
// world point -- and faces you.
//
// GAME THREAD for everything here.

#include "Math.hpp"   // Vec3

#include <string>

namespace halo {

enum class XrTextAnchor : unsigned char {
    View  = 0,   // in front of where you look when it appears, then rides with the camera
    World = 1,   // at `world` (UE world cm), for a notice about a place or a thing
};

struct XrTextPlacement {
    XrTextAnchor anchor = XrTextAnchor::View;
    float dist_cm  = 130.0f;   // View: how far in front of your eyes
    float right_cm = 0.0f;     // View: right (+) / left (-) of where you look
    float up_cm    = -20.0f;   // View: above (+) / below (-) your eye line
    float width_cm = 48.0f;    // the panel's width; its height follows the panel's shape
    Vec3  world{};             // World: the panel's centre
};

struct XrTextTiming {
    int fade_in_ms  = 150;
    int hold_ms     = 1500;
    int fade_out_ms = 1500;
};

// The xrtext* tunables' current values -- start from these and change what your notice needs.
XrTextPlacement xrtext_default_placement();
XrTextTiming    xrtext_default_timing();

// Show `markup` (see above), replacing whatever is showing. False when there is no panel to show it on:
// xrtext=0, the layer is off or not up yet, or the atlas had no room for the panel's row.
bool xrtext_show(const std::string& markup, const XrTextPlacement& where, const XrTextTiming& when);
bool xrtext_show(const std::string& markup);   // the defaults for both

// Every game tick: keeps the showing panel posed and retires it when its fade is done.
void xrtext_tick();

} // namespace halo
