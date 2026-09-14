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
//
// FEATURE wristhud (Experimental). Hook slots: parse_key (the wrist HUD, radar and tracker keys),
// game_tick_late (the census and hosting tick), render_frame (placement), xinput_after_calib_trigger
// (the left-trigger glance gate), sim_unit_state_radar (the blip survey and scan, sim thread) and
// widget_tint_mul (the per-panel gain of the shared widget tint). Table: kWristHudHooks.

#pragma once

#include "features/FeatureHooks.hpp"

#include <atomic>
#include <cstdint>

namespace halo {

extern const FeatureHooks kWristHudHooks;

// The wrist HUD, radar and tracker keys.
bool wristhud_parse_key(const char* key, const char* val, double v);

// Left-trigger glance gate: written by the XInput hook (poll rate, after the calibration menu's
// own trigger eat so that keeps precedence), read by the tick. With wristhudtrigger=1 the panels
// exist only while the left trigger is held -- and the hook swallows the trigger from the game so
// the glance does not also fire whatever LT natively does.
extern std::atomic<bool> g_wristhud_lt;

// Game tick. Census + hosting + per-tick wrist placement. Does nothing while wristhud=0.
void wristhud_tick();

// Placement, called from the stereo view callback so the panels are positioned against the frame
// being drawn rather than the last game tick. Tick-rate placement swam against the world whenever
// the player moved; the weapon rig re-applies on the render path for the same reason.
void wristhud_place();

// ---- WRIST RADAR blips (blipdump survey, 2026-08-28): unit+0x177 is the TEAM byte -- 0x0E
// human (player + marines, armed or corpse), 0x0D covenant (the one carrier photographed held
// PLASMA grenades). Published by the sim (slow cached table scan + per-publish position reads):
// relative Blam-unit offsets from the player, team, and a moving flag. Consumed by the placement.
constexpr int MAX_BLIPS = 12;
extern std::atomic<int>   g_blip_count;
extern std::atomic<float> g_blip_dx[MAX_BLIPS], g_blip_dy[MAX_BLIPS];
extern std::atomic<int>   g_blip_team[MAX_BLIPS];     // 0 = human, 1 = covenant
extern std::atomic<bool>  g_blip_moving[MAX_BLIPS];
// Identity (low dword of the object pointer): publish order compacts as contacts drop in and
// out of range, so an INDEX is not a contact -- the renderer's per-blip smoothing must key on
// this or it smears one dot's motion onto another's.
extern std::atomic<uint32_t> g_blip_id[MAX_BLIPS];
// The RAW +0x177 byte, published alongside the two-way classification. The original survey saw
// three humans and one Covenant, which is far too thin to call the byte "team" -- different
// enemies paint different colours in the field, so at least one more value exists. Logged so the
// real value set can be read off instead of assumed.
extern std::atomic<uint32_t> g_blip_raw[MAX_BLIPS];
// SPECIES ID: the object's leading dword, a tag/definition id. THIS is the real species key --
// surveyed 2026-08-29, it groups instances exactly (six of one type, two of another) and it
// separates a MARINE from an ELITE, which +0x177 cannot (both read 0x0E there). Blip colour is
// keyed on this. Assumed stable across runs; if colours ever shuffle between sessions, re-survey
// -- a per-run pointer or handle would look just like this in a single capture.
extern std::atomic<uint32_t> g_blip_type[MAX_BLIPS];
// The movement test's own numbers per contact: measured speed (blam units/sec), the window it
// was measured over (ms), and the consecutive-window run. Published because contacts that were
// visibly walking metres reported mv=0, and the test has to be read rather than reasoned about.
// ABSOLUTE Blam position per contact, for the species-naming match: a contact is identified by
// finding the Unreal actor standing at the same place and reading its CLASS NAME, which -- unlike
// the tag id -- is the same on every level.
extern std::atomic<float> g_blip_wx[MAX_BLIPS], g_blip_wy[MAX_BLIPS], g_blip_wz[MAX_BLIPS];
extern std::atomic<float> g_blip_speed[MAX_BLIPS];
extern std::atomic<int>   g_blip_dtm[MAX_BLIPS];
extern std::atomic<int>   g_blip_run[MAX_BLIPS];

} // namespace halo
