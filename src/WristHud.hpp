// The wrist HUD: the game's own HUD widgets, re-hosted onto world-space quads that ride the
// off-hand forearm -- shields and ammo read off your wrist like a Pip-Boy instead of floating on
// a flat screen layer.
//
// THE TECHNIQUE IS ALREADY PROVEN IN THIS CODEBASE, three times over: the widget reticule hosts
// WBP_FirstPersonReticle on a WidgetComponent (the game keeps driving its art and hit markers),
// the navpoint markers host the game's own waypoint widgets in world space, and huddump mapped
// MeteoriteHudVisibility's per-element switches. This module is those parts pointed at the status
// HUD with the wrist as the anchor.
//
// CENSUS-FIRST, because hosting is destructive: taking a widget means RemoveFromParent on the
// game's HUD, and doing that to a guessed class name damages the flat HUD for the session. With
// wristhudclasses empty this module only runs the census -- one log line per distinct live
// widget class -- and hosts nothing. The session's log names the shield/ammo/tracker classes,
// the cfg gets them, and the same build starts hosting.

#pragma once

#include <atomic>

namespace halo {

// Left-trigger glance gate: written by the XInput hook (poll rate, after the calibration menu's
// own trigger eat so that keeps precedence), read by the tick. With wristhudtrigger=1 the panels
// exist only while the left trigger is held -- and the hook swallows the trigger from the game so
// the glance does not also fire whatever LT natively does.
extern std::atomic<bool> g_wristhud_lt;
// The hidden reload (coop): the weapon cradle would show the host's refill before the mag is in,
// so the cradle panel is hidden while this is set (Gesture.cpp, slide_phantom_tick).
extern std::atomic<bool> g_wristhud_hide_cradle;

// Game tick. Census + hosting + per-tick wrist placement. Does nothing while wristhud=0.
void wristhud_tick();

// Placement, called from the stereo view callback so the panels are positioned against the frame
// being drawn rather than the last game tick. Tick-rate placement swam against the world whenever
// the player moved; the weapon rig re-applies on the render path for the same reason.
void wristhud_place();

} // namespace halo
