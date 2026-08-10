// CUTSCENE HINT OVERLAY -- a SteamVR overlay showing a shipped PNG while the cutscene flatten
// is engaged (doctrine in Config.hpp, cut_hint block).
//
// WHY AN OPENVR OVERLAY, IN AN OPENXR SESSION. On SteamVR's OpenXR runtime, UEVR's 2D screen
// mode submits the picture as eye-visibility quad layers that SteamVR fails to composite in
// normal viewing -- the headset shows black exactly when the player most needs to be told what
// to do. A SteamVR overlay is composited by SteamVR itself, above every application layer, so
// it displays even in that state -- and it never appears in the game's desktop mirror, which is
// where the hint sends the player. SteamVR accepts overlay connections from any process,
// including one already running an OpenXR session.
//
// FAIL-OPEN BY CONSTRUCTION: openvr_api.dll is loaded dynamically at first use (never linked),
// and any failure -- no SteamVR installed, no runtime running, init refused, PNG missing --
// latches the feature off for the session with one log line. On runtimes where the 2D screen
// simply works (non-SteamVR), init fails fast and nothing is shown, which is the right outcome.
//
// Game thread only. show/hide are idempotent and cheap when the state is unchanged.

#pragma once

namespace halo {

// Show the hint (lazy-initialises the overlay on first call).
void cutscene_hint_show();

// Hide the hint. Safe to call in any state.
void cutscene_hint_hide();

}
