// VIEW-CONSUMER FIXES -- game systems that read the game's OWN camera, which the aim driver
// points at the controller while the view lock pins what the player actually sees.
//
// THE ONE MECHANISM (docs\CAMERA_CONSUMERS_FINDINGS.md)
//   Blam derives its camera from the aim, and the aim follows the hand. The stereo hook cancels
//   that for the EYES only. Every game-side consumer of the camera still sees the hand:
//     * the audio listener        -> positional sound pans against the hand, not the head
//     * Blam relevancy / culling  -> distant actors vanish when the hand aims away
//     * screen-space projection   -> waypoint/objective markers drift with the hand
//   Each fix here is the SAME correction -- the aim-vs-rendered-view delta the plugin already
//   publishes -- applied to one more consumer. The navpoint half lives in Plugin.cpp (it shares
//   the reticle-widget scan); this file owns the audio listener and the dev exec harness.
//
// DISENGAGE IS HALF THE FEATURE. In stick mode (vehicle seats, cutscenes, death) the view lock
// stands down and the game camera IS the rendered view -- the consumers are all correct again,
// and a correction left engaged would introduce exactly the error it exists to remove. Every
// entry point here therefore takes or checks an `engaged` gate and RELEASES (one-shot, logged)
// when it drops, rather than assuming someone else cleans up.

#pragma once

#include "DevTools.hpp"

#include <cstdint>

namespace halo {

// Drive the UE audio listener to the COMPOSED RENDERED VIEW (position + yaw/pitch, roll left
// alone), via APlayerController::SetAudioListenerOverride resolved by reflection. Call once per
// tick from update(), ABOVE its early-outs, with `engaged` false whenever the view lock is not
// actively cancelling the aim (stick mode, frontend, no composed view, feature off, kill switch)
// -- the function clears the override on that edge and goes quiet.
//
// HONESTY NOTE: this game's audio is WWISE (module list + pak paths), and the engine-side
// listener override is only proven to steer UE's AudioMixer. Whether Meteorite's Wwise
// integration honours the PlayerController listener is THE open question this implementation
// answers cheaply: if the UFunction is missing, or panning does not follow the head with
// audiofix=1, the fallback lane is driving the Wwise listener AkComponent directly (see the
// audiodump survey below, which exists to find it).
void audio_fix_tick(bool engaged,
                    float view_x, float view_y, float view_z,
                    float view_yaw_deg, float view_pitch_deg);

// LANE 2 (audiocomp): drive the game's OWN listener component to the rendered view. The 2026-08-12
// survey + ear test settled the lane question: this game routes positional audio through a custom
// `HaloAudioListenerComponent` riding `BP_BlamCameraManager_C` (the aim camera), and the engine
// override above, while it resolves and applies, does not move what Wwise hears -- panning kept
// tracking the controller with it engaged. So the fix is writing the WORLD transform of that
// component every tick. On release the captured relative transform is restored (identity fallback),
// so stick mode / cutscenes hand the listener back to the camera exactly as authored.
void audio_comp_tick(bool engaged, uint32_t tick,
                     float view_x, float view_y, float view_z,
                     float view_yaw_deg, float view_pitch_deg);

// THE CULLING FIX (cullfix / culldist) -- shipping code, NOT dev-gated. Settled live 2026-08-12:
// distant actors (enemy bodies) vanish when the aim camera faces away because Blam relevancy
// culls by ITS view -- the controller. `Blam.Synchronization.Relevancy.OutOfViewCullDistance
// <culldist>` alone fixes it, player-confirmed, with no side effects. Its sibling
// `CullByBlamVisibility 0` is explicitly NOT used: the visibility term WAKES actors as well as
// culling them, and zeroing it froze ambient animation (birds) and the FP arms. Re-applied on
// value change, on PlayerController change (level loads reset cvars; recycled pointers make
// that detection imperfect), and on a slow idempotent timer as insurance. There is no un-exec:
// turning the key off stops re-applying, a restart restores stock. Call at config-poll cadence.
void cull_fix_tick();

#if HALO_VR_DEV

// NAVDUMP (navdump=<n>, edge-triggered): the waypoint lever-1 recon. Sweeps the object array
// for Navpoint / Waypoint / HudDataAsset objects, then dumps each unique class's property
// names+offsets a few supers deep -- looking for the camera-rotation input that feeds the
// CHUD projection (the HudDataAssetReticle.OnCameraRotationChanged pattern). Output is the
// design input for feeding the navpoint layer the RENDERED rotation at its source.
void nav_dump_tick();

// DEV EXEC HARNESS (devexec1..devexec4): each key holds one console command, executed ON CHANGE
// at config-poll rate and logged loudly. Exists for the culling hunt -- the Blam relevancy cvars
// (Blam.Synchronization.Relevancy.*) are strings in the binary whose reads return garbage on
// this build, so WRITES are the experiment, and a live-editable exec line is the cheapest rig
// that can run it. There is no un-exec: clearing a key only stops it being re-applied.
void dev_exec_tick();

// One-shot object-array survey (audiodump=<n>, edge-triggered on the value changing): logs every
// object whose class name mentions Ak / Audio / Listener / Wwise, to locate the live Wwise
// listener component for the fallback lane above. Full-array sweep -- dev builds only, on demand.
void audio_dump_tick();

#else

inline void nav_dump_tick() {}
inline void dev_exec_tick() {}
inline void audio_dump_tick() {}

#endif

} // namespace halo
