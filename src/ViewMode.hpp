// ViewMode -- WHICH EYES UEVR IS ACTUALLY RENDERING THIS FRAME, and therefore what the per-eye
// stereo callbacks mean.
//
// ============================================================================================
// THE PROBLEM
// ============================================================================================
// on_pre/on_post_calculate_stereo_view_offset hand this plugin a VIEW INDEX and a VIEW POSITION,
// and two consumers (XrLayer's compositor quads, AimConverge's cyclopean eye) keep a slot per
// index and average the two. That is exactly right for Native Stereo and silently wrong for every
// other VR_RenderingMethod, because the index UEVR passes is the ENGINE's raw view index, not the
// eye (FFakeStereoRenderingHook.cpp: the plugin dispatch passes `view_index`; the eye is
// `true_index`, which the hook derives and never exports):
//
//   method                    | callbacks per frame | index seen | position handed to us
//   --------------------------+---------------------+------------+------------------------------
//   0 Native Stereo (+/- fix) | 2                   | 0, 1       | left eye, right eye
//   1 Synchronized Sequential | 1                   | 0          | ALTERNATES left / right by frame
//   2 Alternating (AFR)       | 1                   | 0          | ALTERNATES left / right by frame
//   3 Mono (monofix backend)  | 1                   | 0          | the CENTRE eye (midpoint), every frame
//
// So under AFR a per-index slot averages nothing and the "head" hops +/- half an IPD every frame
// (a 45 Hz shimmer on every quad). Under Mono the single view IS the head, which the slot logic
// gets right only until the method is flipped live: the other slot then keeps the last eye it saw
// -- wherever the player was standing when they flipped -- and the midpoint of that and the live
// view is nowhere near the head. Measured from the user's log 2026-09-15: the method was flipped
// live nine times in one session, and "the xr layer elements do not track well" was the report.
//
// ============================================================================================
// WHAT THIS MODULE DOES
// ============================================================================================
// Classifies the topology FROM THE CALLBACKS THEMSELVES, on the render thread, with no engine
// call and no logging:
//   * a slot that has not reported within the last few callbacks is STALE -> one view per frame;
//   * one view per frame whose position jumps by an IPD with alternating sign is ALTERNATING;
//   * one view per frame that moves like a head is MONO.
// The declared VR_RenderingMethod is read separately on the game thread's 2 s poll and kept
// alongside, for the log line and for the one decision that must not rest on a heuristic alone
// (flattening the compositor quads, XrLayer.cpp -- that needs the topology AND the declaration).
//
// The detector rather than the declaration decides how the consumers average, because the
// declaration can be true and inert: an older backend ignores VR_RenderingMethod=3 and renders
// stereo; the PureDark AFW backend reads 3 as Alternate Frame Warping. What the callbacks DO is
// the only fact that matters to a consumer of the callbacks.
//
// Render thread: plain working state, one atomic to publish. Game thread: atomics only.

#pragma once

namespace halo {

enum class ViewMode : int {
    Unknown     = 0,   // too few samples yet -- treat the current view as the head
    Stereo      = 1,   // two views per frame, one per index: average the two slots
    Alternating = 2,   // one view per frame, alternating eyes: average this sample with the last
    Mono        = 3,   // one view per frame, the centre eye: this sample IS the head
};

// RENDER THREAD, from on_post_calculate_stereo_view_offset, BEFORE aim_converge_note_post and
// xrlayer_note_eye -- both ask viewmode_current() inside the same callback and must see this
// sample counted. Position in game units (UE cm), the post-hook (rendered) view position.
void viewmode_note_post(int view_index, float x, float y, float z);

// Any thread. The topology as of the last callback.
ViewMode viewmode_current();

// GAME THREAD. The VR_RenderingMethod UEVR reports on the 2 s poll; -1 = unreadable.
void viewmode_set_declared(int method);
int  viewmode_declared();

// True when the single rendered view is the CENTRE eye by construction: the topology reads Mono
// AND UEVR declares the Mono method (3). Both are required -- see the header comment.
bool viewmode_is_mono();

// Number of post-callbacks counted so far (a liveness counter for the state line).
unsigned viewmode_samples();

const char* viewmode_name(ViewMode m);
const char* viewmode_method_name(int declared_method);

} // namespace halo
