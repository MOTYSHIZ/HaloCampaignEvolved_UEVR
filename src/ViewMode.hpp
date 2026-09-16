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
// (Extreme Compatibility Mode forces the AFR plumbing whatever the method says.)
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
//   * one view per frame whose position SWINGS by two IPDs with alternating sign is ALTERNATING;
//   * one view per frame that moves like a head is MONO.
// The swing test is on the SECOND difference of the position, not the first: under AFR the
// per-frame delta is (camera motion) +/- IPD, and a sign test on that only works while the camera
// moves slower than one IPD per frame -- 8 cm, i.e. any vehicle and nearly a sprint at 72 Hz
// (review finding, 2026-09-15). The difference of two consecutive deltas cancels the common
// motion and leaves +/- 2 IPD plus acceleration, which is millimetres per frame^2 even in a
// Warthog, so the verdict no longer depends on how fast the player is going.
// The declared VR_RenderingMethod is read separately on the game thread's 2 s poll and kept
// alongside, for the log line and for the one decision that must not rest on a heuristic alone
// (flattening the compositor quads, XrLayer.cpp). That decision needs THREE things to agree:
//   1. the topology reads Mono (one view per frame that does not alternate);
//   2. UEVR declares method 3;
//   3. BOTH EYES REPORT THE SAME PROJECTION MATRIX (get_ue_projection_matrix per eye, also read on
//      the poll). This is the one that measures what flattening actually depends on: the monofix
//      mono path gives every eye the union-FOV projection, so left and right come back identical,
//      while every stereo-pair mode -- Native, AFR, Synchronized, AFW -- hands back mirror images
//      of each other because a headset's per-eye FOV is asymmetric. A backend whose method 3
//      rendered ONE FIXED eye would still report that eye's projection, distinct from the other's,
//      and fail this test. Residual false positive: a symmetric-FOV headset (or the SimVR rig,
//      whose eyes are symmetric) on such a backend -- not a real configuration.
// The same fact also OVERRIDES the alternation vote: a single view whose two projections are
// identical is classified Mono without consulting the ring, because an alternating pair cannot
// have identical frustums. Needed on this title, where the view position swings by several cm
// per frame^2 with alternating sign while the player walks with the scope open -- enough to win
// the vote for a second at a time and flicker the flattening (2026-09-15 headset report).
//
// The detector rather than the declaration decides how the consumers average, because the
// declaration can be true and inert: an older backend ignores VR_RenderingMethod=3 and renders
// stereo; the PureDark AFW backend reads 3 as Alternate Frame Warping, which renders the eyes by
// turns and therefore reads Alternating here. What the callbacks DO is the only fact that matters
// to a consumer of the callbacks; the projection test above covers the one case they cannot tell.
//
// HEADLESS CAVEAT: under the SimVR/OpenVR harness the runtime reports no HMD, UEVR's eye offsets
// are zero, and AFR is indistinguishable from Mono by construction -- an AFR arm reading "mono"
// there is the rig, not the detector.
//
// Render thread: plain working state, one atomic to publish. Game thread: atomics only.

#pragma once

namespace halo {

enum class ViewMode : int {
    Unknown     = 0,   // one view per frame, verdict pending -- consumers average this sample with
                       // the previous one (harmless under Mono, correct under AFR), never one eye
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

// GAME THREAD, on the same poll: whether get_ue_projection_matrix(LEFT) == (RIGHT). Fail-closed:
// false until the poll has looked, and false for an all-zero matrix (the runtime not ready yet,
// which would otherwise compare equal).
void viewmode_set_shared_projection(bool shared);
bool viewmode_shared_projection();

// True when the single rendered view is the CENTRE eye by construction: the topology reads Mono,
// UEVR declares the Mono method (3), AND both eyes share one projection. All three are required --
// see the header comment.
bool viewmode_is_mono();

// Number of post-callbacks counted so far. A consumer that averages "this sample with the
// previous one" keys on it: two samples are consecutive only if this count moved by exactly one
// between them, which is what rules out a previous sample from before the consumer was armed.
unsigned viewmode_samples();

// GAME THREAD, for the VIEWMODE line: the 8-bit alternation vote ring as of the last sample
// (bit set = that same-index sample swung by >4 cm AND reversed direction) and the largest
// second difference seen since the previous call, in game cm (reading resets it). Together they
// show what the single view's position is doing frame to frame, which is how the scoped-walking
// swing on this title was seen.
void viewmode_diag(unsigned* ring, float* swing_max_cm);

const char* viewmode_name(ViewMode m);
const char* viewmode_method_name(int declared_method);

} // namespace halo
