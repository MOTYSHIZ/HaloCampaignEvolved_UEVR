// The game's own controller settings, read live and changed TRANSIENTLY.
//
// WHAT THIS IS FOR
//   Motion aim and stick mode want different things from the same global settings:
//
//     LOOK INVERSION - motion aim must never be inverted. Your hand points where it points; there
//       is no sense in which "up is down" when the gun IS the input. But stick mode is a gamepad,
//       and plenty of people invert deliberately for vehicles. So invert has to apply in stick mode
//       and not in motion aim.
//
//     SENSITIVITY / DEAD ZONE - the aim loop wants sensitivity high (mid-stick deflections then buy
//       more turn rate, which is where the loop operates) and the dead zone small. A human driving
//       a Warthog camera wants their own values.
//
// TWO DIFFERENT MECHANISMS, ON PURPOSE
//   Inversion is cancelled IN OUR OWN OUTPUT -- we simply negate the stick value we synthesise --
//   and the game setting is never touched. That is strictly better than toggling the setting per
//   mode: nothing is written, so no crash, hard exit or mistimed transition can leave a player's
//   comfort preference flipped, and stick mode inherits their choice for free by doing nothing.
//
//   Sensitivity and dead zone CANNOT be done that way. They are properties of the plant -- how much
//   the game turns for a given stick value -- and no amount of arithmetic on our side manufactures
//   authority the game will not deliver. Those must go through the game's own setter + apply.
//
// WHY THE SETTER PATH IS SAFE (measured, not assumed)
//   Writing the UE property directly does nothing: it stores and persists but never reaches the aim
//   path. The working route is Set<X>() followed by ApplyHaloUserSettings(), which is what the
//   options menu does.
//
//   Crucially, Apply does NOT write to disk -- SaveHaloUserSettings() is a separate function we
//   never call. Verified by setting a value the player has never chosen, letting two 60-second
//   autosaves pass, hard-killing the process, and confirming their own value reloaded. So a change
//   made here lives exactly as long as the process, and a crash restores the player's settings by
//   doing nothing at all.

#pragma once

#include <cstdint>

namespace halo {

// Refreshed from the game at config-poll rate. Sign multipliers, ready to fold into the aim law's
// output: 1.0 normally, -1.0 when the game is set to invert that axis. Motion aim multiplies by
// these to CANCEL the inversion the game is about to apply.
extern float g_invert_cancel_x;
extern float g_invert_cancel_y;

// Re-read the game's controller settings and, if the mode has changed, push our values or restore
// the player's. Call from the tick; cheap (a resolve-once object lookup, then field reads).
//   in_stick_mode: true while the player's own stick is driving (vehicles, cutscenes).
void game_settings_tick(bool in_stick_mode);

// Put the player's own values back and forget our resolved object. Called at shutdown so a normal
// unload leaves the session as it was found, without relying on process exit to do it.
void game_settings_restore();

} // namespace halo
