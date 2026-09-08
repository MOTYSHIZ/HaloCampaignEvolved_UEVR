// XrLayer -- a bright reticule drawn by the OpenXR COMPOSITOR instead of by the scene.
//
// ============================================================================================
// WHY THIS EXISTS
// ============================================================================================
// The hosted-widget reticule (Reticule.cpp) renders inside UE's scene, so its colour is
// multiplied by the scene's pre-exposure and then run through the tonemapper. Halo's authored
// cyan therefore sinks toward black in a bright exterior and blows out in shade. The shipped
// mitigation is a CONSTANT multiplier -- aim_widget_tint 1024 x aim_widget_gain 5 -- and
// Config.hpp is candid that this is
//
//     "the in-headset compromise between bright-beach readability and blowing out in shade"
//
// which is the signature of a constant fighting a VARYING term. No single value wins.
//
// The in-engine fix already exists in widget_quad_begin(): the VR-editor pass-through material
// multiplies by EyeAdaptationInverse and cancels the exposure outright. But it lives at
// /Engine/VREditor/..., an EDITOR-ONLY asset that is not cooked into the shipping game, so on a
// stock install it resolves null and every player falls back to the constant. Shipping it means
// shipping a LogicMod pak through the retoc/UnrealReZen toolchain.
//
// An OpenXR composition layer is submitted to the runtime AFTER the entire post chain. Scene
// exposure, eye adaptation and tonemapping never touch it. It is immune by construction rather
// than compensating, and it needs no game assets at all.
//
// ============================================================================================
// WHAT THIS IS NOT
// ============================================================================================
// This does NOT replace the in-scene reticule and must never be built to. The in-scene widget is
// the game's own live HUD crosshair re-parented onto a world-space UWidgetComponent, which is
// where the firing/reload/hit-marker ANIMATION comes from -- we never authored any of that art
// and cannot reproduce it. Reticule.cpp stays exactly as it is.
//
// STAGE 1 (this file, now): the compositor plumbing, proven end to end, drawing a reticule we
//   generate ourselves. Bright, exposure-proof, always visible -- but STATIC. No firing bloom,
//   no reload state, no hit marker.
// STAGE 2 (XrSource.cpp): swap the texture source from our generated bitmap to the widget's own
//   render target, so UE keeps drawing the animations and this layer merely PRESENTS them. That
//   needs UTextureRenderTarget2D -> FTextureResource -> FRHITexture -> ID3D12Resource, two
//   unreflected engine hops whose last step makes a virtual call on an inferred pointer. It
//   therefore lives in its own module with its own probe ladder and its own validation rather than
//   being smuggled in here -- read XrSource.hpp. The seam is xrlayer_set_source() below.
//
// Because stage 1 has no animation, it is DEFAULT OFF and is a dev/research key, not a player
// one. Turning it on is an experiment, not a setting.
//
// ============================================================================================
// HOW IT ATTACHES -- no UEVR fork, no forked backend
// ============================================================================================
// UEVR is the OpenXR *application*; we are a plugin inside it. UEVR builds its own layer list and
// calls xrEndFrame, and the plugin API offers no way to add a layer to it (the
// UEVR_OPENXR_SWAPCHAIN_* constants in API.h have no accessor -- they are dead constants). So we
// intercept the call.
//
// xrEndFrame is a NAMED EXPORT of the openxr_loader.dll that ships beside UEVRBackend.dll, so the
// address is GetProcAddress-resolved, not a recorded RVA -- see the ADDR-HYGIENE marker in
// XrLayer.cpp. We copy UEVR's layer array, append ours LAST (topmost), and forward.
//
// ============================================================================================
// !! THAT DOES NOT WORK, AND THE REASON MATTERS (measured 2026-08-23) !!
// ============================================================================================
// UEVR STATICALLY LINKS the OpenXR loader into UEVRBackend.dll. `dumpbin /imports UEVRBackend.dll`
// shows NO openxr_loader.dll import whatsoever. So UEVR's xrEndFrame call never touches the
// exported symbol we hook, and the hook -- which installs cleanly and reports success -- is called
// exactly zero times. In the field this presented as "I turned it on and nothing is different".
//
// The trap is that openxr_loader.dll IS loaded in the process (Virtual Desktop's streamer pulls it
// in), so GetModuleHandleW finds it, GetProcAddress resolves a real xrEndFrame, and safetyhook
// patches it successfully. Every check passes. The address is simply not on any path UEVR executes.
// This is "installed is not running" in its purest form: nothing was wrong with the address, the
// resolution, or the hook -- only with the belief about who calls it.
//
// WHAT THIS FILE ACTUALLY DOES NOW (and it works -- verified in-headset and in-sim 2026-08-23):
// hook UEVR's OWN statically-linked xrEndFrame, resolved BY SYMBOL out of UEVRBackend.pdb. See
// XrLayerAttach.hpp. That is a third option the paragraphs above did not consider, and it is the
// shipping one -- do not read the section above and conclude the feature is unbuilt.
//
// The catch is the PDB: only a UEVR checkout has UEVRBackend.pdb, so this attachment can never
// SUCCEED for a player. It is compiled into release builds all the same -- it is not behind
// HALO_VR_DEV -- and is still asked as a fallback when the API layer is not registered, where it
// fails closed and logs. See XrLayerAttach.hpp for why it is kept rather than gated out.
//
// FOR SHIPPING, the route is a real OpenXR API layer: the statically-linked loader still walks the
// implicit-layer registry and loads what it finds there, which is proven on this exact stack --
// C:\Program Files\Virtual Desktop Streamer\openxr-oculus-compatibility.json is registered implicit
// (DWORD 0 = enabled) and its DLL is loaded inside the Halo process alongside UEVR. A layer needs no
// hooking and no symbols at all: the loader hands it xrEndFrame through the normal chain.
//
// Everything below the attachment point -- the swapchain, the blit, the pose math, the colour
// override, the source resolution in XrSource.cpp, the fail-open contract -- is attachment-agnostic
// and carries over unchanged. Only how we get called has to change.
//
// Everything here fails OPEN. Any failure -- no OpenXR, no loader export, hook refused, swapchain
// refused, session lost -- latches this module off and leaves the in-scene reticule untouched and
// unaware. There is no state in which turning this on can leave the player with no crosshair.

#pragma once

#include "Math.hpp"

namespace halo {

// ---- publication from the reticule drive site (GAME THREAD) ----------------------------------
//
// Called from update() right where reticule_widget_move() is called, with the same target. All UE
// world space, centimetres and degrees; `apparent_scale` is g_ret_scale_mul, the same 1/distance
// compensation the in-scene reticule uses, so the two agree on size.
//
// Cheap and safe to call every tick whether or not the layer is enabled -- it writes one
// seqlock-guarded snapshot and returns. It does NOT touch OpenXR; the compositor side runs on the
// submit thread and only ever reads this snapshot.
//
// The camera pose is passed in rather than read from Plugin.cpp's globals on purpose: this module
// stays decoupled from Plugin.cpp's internals, the same way scope_notice_ray() takes its ray.
// TICK RATE decides WHICH world point and HOW BIG. That needs the aim state and the trace, neither
// of which belongs on the render thread.
void xrlayer_notice_reticule(const Vec3& world_pos, float apparent_scale);

// ---- SLOTS: the same machinery, for more than one quad ----------------------------------------
//
// Slot 0 IS the reticule and xrlayer_notice_reticule() above is its call path, unchanged. Slots
// 1..8 are the world navpoint markers. Everything a slot needs -- a cell in the atlas, a pose, a
// source, a capture -- is indexed by this number and nothing else.
//
// WHY THE MARKERS WANT THIS AT ALL, in the user's own words: a composition layer is submitted
// AFTER the whole post chain, so exposure and the tonemapper never touch it (the markers currently
// carry the same aim_widget_gain x aim_widget_tint constant the reticule did, and bloom the scene
// for the same reason), AND a compositor quad is never occluded -- which is the entire job of the
// navpoint line trace at Plugin.cpp's lane 2. The reticule's trace is NOT redundant in the same way
// and must stay: it puts the reticle on the surface the shot will hit, which is real information.
constexpr int XRLAYER_SLOT_RETICULE = 0;
constexpr int XRLAYER_SLOT_NAV_BASE = 1;    // slots 1..8
// THE MARKER COUNT IS ITS OWN CONSTANT, not XRLAYER_SLOTS - 1. It was the latter until a tenth slot
// was added, at which point "slots minus the reticule" silently became NINE markers: the atlas
// builder would have laid out a ninth cell and the drop list a ninth quad, both indexing a slot the
// navpoint lane never feeds. Nothing would have failed to compile.
constexpr int XRLAYER_NAV_COUNT     = 8;

// SLOT 9 -- THE PANE. A large, slowly-refreshed, CAMERA-shaped source rather than a HUD glyph: the
// weapon scope's SceneCaptureComponent2D render target (Scope.cpp owns the capture; this module only
// presents it). It exists as its own slot rather than as a tenth marker because it differs in all
// three ways a slot can differ -- cell size, refresh cadence, and orientation.
//
// WHY IT IS IN THE SHARED ATLAS AND NOT ITS OWN SWAPCHAIN. Its own swapchain was the first plan and
// was wrong: it would have duplicated the format/usage negotiation ladder, the three-deep allocator
// ring, the fence, and the teardown -- every one of which has crash history in this module (see the
// borrowed-resource section) -- to gain nothing the imageRect already gives. One swapchain with
// disjoint imageRects is what XrSwapchainSubImage is FOR, and it is the argument this file already
// makes for the nine. The pane just needs a bigger rectangle.
//
// CONSTRAINT THE CALLER MUST HONOUR: cells are SQUARE. A scope pane is conventionally round or
// square and the caller sizes its own render target, so this costs nothing today -- but a non-square
// source is REFUSED rather than stretched, on the same "refuse rather than copy" rule as every other
// slot, because a silently letterboxed scope is worse than one that says why it is absent.
constexpr int XRLAYER_SLOT_PANE     = 9;

// THE GRAB GUIDE -- the beam from the support hand to the point on the barrel it would grab.
//
// It is here for the reasons the reticule and the markers are: in the scene it was occluded by the
// weapon it lies along, it took the game's lighting and exposure so it dimmed exactly where it was
// needed most, and an emissive material bright enough to read would bloom. In the compositor none
// of those exist, because it never enters the scene.
//
// THE CHEAPEST SLOT IN THIS FILE, and deliberately so. It uses NONE of the source machinery: no
// XrSource target, no widget to host, no render target to capture, no rehost or coherence tracking,
// no per-tick re-resolve. It is a FLAT COLOUR, filled once, so its cell is 8 px rather than the
// reticule's 256 or a marker's 128. What it needs from this module is the two things only this
// module can do: a world pose the compositor honours, and pixels that are never occluded.
//
// It is also the first NON-SQUARE quad. A beam is long and thin, and XrCompositionLayerQuad::size
// is an XrExtent2Df -- width and height, independent -- so the squareness was only ever our own
// single-scalar API. See xrlayer_notice_quad's `world_cm_h`.
constexpr int XRLAYER_SLOT_GUIDE    = 10;

constexpr int XRLAYER_SLOTS         = 11;

// GAME THREAD, tick rate. Same contract as xrlayer_notice_reticule: one snapshot write, no OpenXR,
// cheap enough to call unconditionally.
//
// `world_cm` IS PASSED IN RATHER THAN DERIVED, and that is deliberate. compute_pose() used to build
// the quad's world size from aim_widget_draw x aim_widget_scale -- the RETICULE's numbers -- which
// silently coupled this module to one caller's config. A navpoint has its own size formula
// (nav_world_scale x the per-kind multiplier x the drawn distance), so the size has to arrive from
// the caller or the two will disagree the moment either is retuned.
//
// `hold_cm` IS THE DEPTH DECISION, and it is a deliberate one.
//   0        draw the quad AT world_pos. The reticule uses this: its target already is the surface
//            the shot will hit, at the distance it is actually at.
//   > 0      draw it along the eye -> world_pos ray at exactly this distance, re-derived from the
//            CURRENT eye every rendered frame. The navpoint markers use this, and they have to:
//            the in-scene lane already re-places them in the stereo callback because "at a 4 m
//            draw distance the eye-to-objective direction changes materially between 32 Hz ticks",
//            and a world-fixed quad would additionally GROW as the player walked towards it,
//            because world_cm is fixed.
//
// The trace used to supply this number as a side effect of avoiding occlusion. A compositor quad
// cannot be occluded, but it still needs a depth, because depth is what sets VERGENCE -- the eyes
// physically converge on it. So the caller states it (objective's true distance, clamped to a
// comfort band) instead of inheriting whatever a raycast happened to hit.
//
// `priority` orders the drop list when the runtime has fewer free layers than we have quads.
// 0 = never dropped (the reticule). Higher = dropped sooner. Within one priority the SMALLEST
// APPARENT quad goes first, which is "furthest/smallest first" expressed as one number.
// `world_cm_h` IS THE SECOND EXTENT, and 0 means "square" so every existing caller is unchanged.
//
// XrCompositionLayerQuad::size has always been an XrExtent2Df -- width and height, independent.
// This API carried one scalar and wrote it into both, so every quad in this file has been square
// by our choice rather than the runtime's. The grab guide is a BEAM: long along the barrel, thin
// across it, and stretching a square would mean either a fat beam or a short one.
//
// The cell stays SQUARE regardless -- this is the quad's world extent, not its source rectangle.
// A flat colour has no aspect to distort, so nothing is letterboxed; a caller feeding real ART
// through a non-square extent WOULD stretch it, and should not.
void xrlayer_notice_quad(int slot, const Vec3& world_pos, float world_cm, float hold_cm,
                         int priority, float world_cm_h = 0.0f);

// GAME THREAD. Stop submitting this slot. Cheap, idempotent, and does NOT drop the slot's captured
// art -- a marker that comes back within a tick or two should not have to re-resolve its texture.
void xrlayer_retire_quad(int slot);

// GAME THREAD. Additionally DISCARD this slot's captured art, so xrlayer_slot_ready() is false
// until genuinely new pixels land. The counterpart to retire_quad for a slot whose MEANING changes
// between uses, rather than one that merely blinks.
//
// retire_quad deliberately keeps the art, and for a navpoint that is right: the marker that comes
// back is the same marker, and re-resolving its texture would cost more than one stale frame. The
// scope is the opposite case. Its cell holds a picture of a DIFFERENT weapon's sight at a different
// zoom, and xrlayerhold is 1500 ms -- so re-opening the scope inside a second and a half found the
// slot already "ready" and drew the previous session's frozen image while the new capture caught up.
// That was the reported "I see the previous image frozen in the pane for a split second".
//
// Cheap and idempotent: it zeroes the freshness beat, and the next real capture sets it again. A
// beat of 0 is already the module's "has never captured" value, so nothing downstream needs to
// learn a new state -- this just returns the slot to it.
void xrlayer_invalidate_capture(int slot);

// GAME THREAD. This slot's hosted widget CLASS just changed (the sparse navpoint map reordered, so a
// stable pool component now wears a different navpoint's art). Marks the slot incoherent until its
// cell is re-captured on a later tick, so the compositor never presents the previous widget's art at
// the new marker's pose -- the one-frame "wrong marker flash". Cheap and idempotent.
void xrlayer_notice_rehost(int slot);

// GAME THREAD. Give this slot its OWN orientation instead of the head's.
//
// Every quad is head-ORIENTED by default (`pose.orientation = head_q`) while being world-POSITIONED.
// That is correct for a billboarded crosshair or waypoint, which should face the player from
// wherever it sits, and it is wrong for anything gun-mounted: a scope pane has to rotate with the
// weapon or it is not a scope, it is a floating picture.
//
// TWO WORLD-SPACE DIRECTION VECTORS, NOT A ROTATOR OR A QUATERNION, and that is the whole design
// decision of this entry point rather than an inconvenience to work around.
//
// This started as `(yaw, pitch, roll)` fed through rotator_to_quat(). That is WRONG and would have
// shipped a subtly rotated pane: rotator_to_quat() produces a quaternion in UE's LEFT-handed frame,
// while XrPosef::orientation is OpenXR's RIGHT-handed one, and the basis map this module already
// uses (ue_offset_to_xr: x = y, y = z, z = -x) has DETERMINANT -1. A handedness flip does not carry
// a rotation across by copying x/y/z/w, and getting it wrong produces an error that is near zero
// when you look straight down the sights -- which is the only time anyone looks at a scope -- and
// grows off-axis. It would have passed every deliberate test and failed in play.
//
// So the override is expressed as directions, which go through ue_offset_to_xr(), the ONE mapping
// in this file that is proven in a headset (it is what carries every quad's position, and its
// missing-roll bug was found and fixed in headset on 2026-08-23). The quaternion is then built here
// from the converted basis. No new frame math crosses the UE/XR boundary.
//
// `fwd_world` is where the quad's face should point (the weapon's aim direction for a scope);
// `up_world` is its roll reference and need not be perpendicular -- it is orthogonalised. Both are
// UE world-space directions; neither needs to be normalised. A degenerate pair (either vector zero,
// or the two parallel) is REFUSED and leaves the slot head-oriented rather than emitting a garbage
// basis.
//
// SELF-CHECK WORTH KNOWING: passing the VIEW's own forward and up must reproduce head-locked
// behaviour exactly, because the converted basis is then identity. That is the cheapest way to
// verify a caller's vectors are in the frame it thinks they are.
//
// NOT STICKY ACROSS RETIREMENT: xrlayer_retire_quad() clears the override, so a slot reused for
// something else cannot inherit a stale weapon rotation. Call it each tick you set the pose, the way
// the pose itself is published.
void xrlayer_set_quad_orientation(int slot, const Vec3& fwd_world, const Vec3& up_world);

// GAME THREAD. Back to head-oriented for this slot. Idempotent; safe on a slot that never had one.
void xrlayer_clear_quad_orientation(int slot);

// Treat this slot's next target as an offset from the eye, not a world point. For quads glued to
// something that moves WITH the player (the aim reticule, the grab guide) -- it is what stops them
// trailing a tick of travel behind during locomotion.
void xrlayer_set_quad_head_relative(int slot, bool on);

// RENDER THREAD. The rig's world rotation for the frame being drawn.
//
// Plugin.cpp already recomposes the rig against a live parent every frame -- its own instrument
// reports "corrected up to 161 deg of parent motion since tick". Anything placed from a GAME-TICK
// sample of that rig is therefore stale by however far the weapon turned in the interval, which is
// why a quad following the in-world pane tracks correctly when still and lags when you rotate.
// Publishing the render-rate rotation lets a slot's orientation be corrected by the same delta the
// arms already get.
void xrlayer_note_rig(const Vec3& pos, const Vec3& fwd, const Vec3& right, const Vec3& up);

// GAME THREAD. Publish this slot's POSITION as an offset expressed in the RIG's own basis, to be
// recomposed against the render-rate rig each frame.
//
// MEASURED 2026-09-07: with scopelayer=3 -- no orientation published at all -- the quad still
// hopped while the controller was ROTATED. That isolates the fault to POSITION, and explains it:
// the pane swings through an arc about the shoulder at high angular rate, and the quad's position
// was a ~32 Hz sample of it displayed at 90+ Hz.
//
// Head-relative anchoring cannot fix this. It cancels translation the target and the EYE share,
// which is why it fixed locomotion -- but a weapon rotating about the shoulder moves relative to a
// stationary head, so there is nothing for it to cancel. The rig is the frame the pane actually
// rides, so the offset has to be expressed there.
//
// Supersedes head-relative for a weapon-mounted slot: the rig frame already carries the body's
// translation, so this cancels both.
void xrlayer_set_quad_rig_relative(int slot, bool on, const Vec3& rig_pos, const Vec3& rig_fwd,
                                   const Vec3& rig_right, const Vec3& rig_up);

// GAME THREAD. As xrlayer_set_quad_orientation, but ALSO records the rig rotation the vectors were
// measured against. At submit the slot's orientation is rotated by (render_rig * inverse(tick_rig)),
// so a weapon-mounted quad stays with the weapon between ticks instead of trailing it.
//
// Pass the rig rotation from the SAME tick that produced fwd/up -- the correction is a difference
// between two measurements of one thing, and mixing ticks reintroduces exactly the error it removes.
void xrlayer_set_quad_orientation_tracked(int slot, const Vec3& fwd_world, const Vec3& up_world,
                                          const Vec3& rig_fwd, const Vec3& rig_right,
                                          const Vec3& rig_up);

// GAME THREAD. Where the scope reticule sits WITHIN the pane, as normalised pane coordinates in
// -1..1 (0,0 = centre, +u right, +v up). The reticule quad was pinned to dead pane centre, which is
// correct only while the capture camera's axis and the traced impact point agree; the caller
// projects the real impact point into the capture's own frustum and publishes the result here.
// valid=false restores centre, which is the old behaviour exactly.
void xrlayer_set_scope_reticle_offset(float u, float v, bool valid);

// GAME THREAD. The CAPTURE CAMERA'S world right/up. Mode 2 of the scope reticule needs them to
// project into the frame the image is actually rendered in.
//
// Mode 2 used to build its own basis from world up (right = aim x worldUp). That is a THIRD frame:
// the image's axes are the CAMERA'S, and the blit maps them onto the quad's local axes one to one,
// so a world-levelled projection is wrong by the camera's whole roll -- scope_cam_roll, the
// per-shape uv_roll, AND the live roll lock. The roll lock is what made it more than a constant: it
// moves continuously to hold the image upright as the weapon cants, so the reticule's slide
// direction was wrong by an angle that changed as the gun rolled. That is why scoperetflipx/y could
// never fix it -- a sign flip corrects 180 degrees, not a moving angle.
void xrlayer_note_scope_cam_axes(const Vec3& right, const Vec3& up, bool valid);

// GAME THREAD, from the reticule publish site. Say WHY this tick did not publish a target, so the
// "layer DARK" line can name the upstream gate instead of just observing that nothing arrived.
//
// This exists because the honest version of that log line was "the game thread is not calling
// xrlayer_notice_reticule(), and I cannot see why" -- which sends the reader into a 300-line block
// in Plugin.cpp with a list of candidate gates and no way to tell them apart from a log. One byte
// from the caller turns that into an answer.
//
//   0  published normally
//   1  the feature is off (aimreticule=0)
//   2  no rig component, or K2_GetComponentLocation on it failed
//   3  no rig PARENT, or K2_GetComponentLocation on it failed  <- this is `have_origin`
//   4  no trace target this tick
//
// Cheap (one relaxed store) and unconditional: a gate that only reports itself when a debug key is
// set is a gate nobody can debug from a player's log, which is the whole failure this addresses.
void xrlayer_note_publish_gate(int gate);

// GAME THREAD. The RAW aim ray the weapon scope's capture is mounted on -- the same one handed to
// scope_notice_ray(), from the same site, so the two cannot drift apart.
//
// WHY THIS IS NEEDED AT ALL, and it is not obvious: the scope's capture sits ON this ray and looks
// down it (Scope.cpp: cam_pos = origin + dir * scope_cam_dist), so a point on the ray lands dead
// centre of the capture BY CONSTRUCTION. Centre would therefore be exact -- except the scope is
// given the RAW aim angles while the RETICULE is drawn at a SMOOTHED target (Plugin.cpp says so
// where it builds this ray: "reticule_ray_angles layers display smoothing tuned for a floating
// dot"). The two diverge, and that divergence is real information about where the shot goes rather
// than where the optics point. Drawing at centre discards it.
//
// Cheap: three atomic stores, no maths. The projection happens on the submit thread from this plus
// the reticule target it already holds.
void xrlayer_note_scope_ray(const Vec3& origin, const Vec3& target);

// The COMPOSED view basis, as UE world directions in the same frame as xrlayer_notice_quad's
// world_pos. Returns false until a stereo frame has actually been composed (pre-injection, frontend)
// -- and that false is the point of the function, not a formality.
//
// WHY THIS LIVES HERE RATHER THAN BEING REBUILT BY THE CALLER. Plugin.cpp publishes the view yaw and
// pitch as globals but deliberately does NOT publish roll, with a comment saying a third global
// would invite consumers to read a value they were not written against. It is right. A basis rebuilt
// from yaw and pitch alone is roll-less, and a roll-less basis is EXACTLY the live bug this module
// already found and fixed in a headset (see ROLL MUST BE PASSED on xrlayer_note_eye below): it
// agrees with the true basis until the player tilts their head, then swings about the view axis.
//
// This module receives all three from xrlayer_note_eye and keeps them together, so the roll decision
// stays where it was already got right once.
//
// THE INTENDED USE IS A SELF-CHECK, and it is worth stating what makes it one: feed the result
// straight back into xrlayer_set_quad_orientation() and the converted basis must come out IDENTITY,
// so the quad is byte-identical to head-locked. Any other outcome means the caller's frame
// assumptions are wrong -- which is what you want to discover on a bench, not in a headset. The
// false return is load-bearing for that too: reading raw angles at the frontend yields 0/0/0, which
// is a plausible-looking identity basis and would launder "no view yet" as "verified".
bool xrlayer_view_basis(Vec3* fwd_world, Vec3* up_world);

// The MONO view position in UE world cm -- the same value head-relative slots are re-anchored
// against. false until a view has been composed. For callers outside Plugin.cpp that need to turn
// a world point into a head-relative offset (see xrlayer_set_quad_head_relative); Plugin.cpp's own
// layer_anchor() does this with file-local globals nothing else can reach.
bool xrlayer_mono_view_pos(Vec3* out_world);

// GAME THREAD, ONCE, BEFORE THE ATLAS IS BUILT (i.e. from config load or first tick, not mid-session).
// Ask for the pane cell to be `cell_px` square. 0 = no pane cell at all, which is the default: a
// build nobody has asked for a pane on pays no atlas for it.
//
// Returns false and changes nothing if the request cannot fit under the atlas cap alongside the
// reticule and the markers -- the caller should treat that as "the pane is off" and say so, rather
// than retrying smaller behind the player's back. The reticule and markers are never sacrificed to
// fit a pane: they are the proven half.
bool xrlayer_pane_configure(int cell_px);

// True when this slot has real art captured into the atlas and that capture is still within the
// hold window -- i.e. the compositor is drawing THIS slot's game art right now.
//
// The navpoint lane gates its in-scene marker on this, for exactly the reason the reticule gates on
// xrlayer_live(): if the compositor is not actually showing the marker, hiding the in-scene one
// leaves the player with no waypoint at all.
bool xrlayer_slot_ready(int slot);

// The atlas cell edge this slot was sized at, or 0 if the slot has no cell (the atlas is built once
// at bring-up). XrSource needs it: a source is only accepted if its own dimensions equal ITS CELL.
int xrlayer_cell_dim(int slot);

// RENDER RATE decides WHERE that point lands in stage space. Called from the post-stereo callback,
// once per eye, with that eye's rendered position and the finished view rotation.
//
// THE SPLIT IS THE WHOLE POINT, and this project has paid for the lesson twice already: the
// navpoint markers re-place at render rate because "at a 4 m draw distance the eye-to-objective
// direction changes materially between 32 Hz ticks", and the movement-orientation term was moved
// live for the same reason ("stale by up to ~30 ms ... drifting while you turn and settling only
// once you stop"). A reticule sits closer than a navpoint, so it is worse here, not better.
//
// Computing the stage pose on the tick meant pairing a head pose sampled NOW against eye/view data
// from the last rendered frame -- two samples of the same physical thing taken at different times,
// which is an angular error proportional to head speed. That is the residual drift left after the
// first fix, and no amount of smoothing hides it because it is not noise, it is lag.
// ROLL MUST BE PASSED. The pose maths rotates the reticule offset by the head orientation from
// get_pose(), which carries the player's real roll; reconstructing the camera basis without roll
// makes the two disagree about it and swings the quad around the view axis. Was a live bug.
// mono_view_pos is the PRE-HOOK game camera -- the same value for BOTH eyes. It is what
// head-relative slots are re-anchored against; eye_pos must NOT be used for that (see below).
void xrlayer_note_eye(int eye_index, const Vec3& eye_pos, const Vec3& mono_view_pos,
                      float view_yaw, float view_pitch, float view_roll);

// Per-tick housekeeping on the GAME THREAD: config changes, lazy init, the liveness watchdog, and
// the state logging. Never blocks. Does nothing but one bool test when the feature is off.
void xrlayer_tick();

// True only while the layer is PROVEN to be reaching the compositor -- i.e. our hook ran and
// appended a layer within the watchdog window, not merely that installation returned success.
//
// "Installed" is not "running": register_inline_hook succeeds on any readable address and reports
// success. A caller that wants to hide the in-scene reticule in favour of this one must gate on
// THIS, never on whether setup succeeded. Nothing gates on it today (stage 1 deliberately draws
// alongside the in-scene reticule); it exists so stage 2 has an honest signal to use.
bool xrlayer_live();

// ATTACHED, not DRAWING. True once the hook is installed and the swapchain exists (state Armed),
// regardless of whether a quad was submitted in the last window.
//
// THE DISTINCTION IS LOAD-BEARING AND THE TWO ARE NOT INTERCHANGEABLE:
//
//   xrlayer_live()     "is the compositor showing something right now" -- correct for deciding
//                      whether to HIDE an in-scene fallback, because hiding on a layer that is not
//                      drawing leaves the player looking at nothing.
//   xrlayer_attached() "does this lane belong to the compositor at all" -- correct for OWNERSHIP,
//                      because live is both an input and an output of the ownership loop.
//
// Measured 2026-08-28: navw_layer_owns() used live(). g_live counts submitted quads across ALL
// slots, so a drawing navpoint held live=1 while the reticule slot was already stale; when the
// marker stopped, live went 0, the nav lane retired its quads -- and could then never revive the
// layer, because only the reticule slot can, and it was gated off upstream. A self-latch built out
// of a signal that measures its own output.
bool xrlayer_attached();

// STAGE 2 SEAM. Hand the layer a D3D12 texture to present instead of the generated bitmap. Passing
// nullptr reverts to the generated one. XrSource.cpp is the caller; it resolves the widget
// reticule's own render target, and everything it can validate from the UE side it validates
// before calling here.
//
// THIS FUNCTION IS THE SECOND GATE, and it asks the questions only this side can answer: is the
// resource the same size as the swapchain, and is its DXGI format in the same typeless family? A
// CopyResource that fails either is an INVALID CALL, not a wrong-looking picture. It refuses rather
// than copying, and refusing costs nothing -- the generated ring keeps drawing.
//
// GAME THREAD. The resource must live on the same ID3D12Device UEVR reports in UEVR_RendererData
// and must be in ENGINE_SRC_COLOR when the game thread borrows it (see the barrier pair in
// xrlayer_capture_source, which names that as the assumption it is).
//
// ACCEPTING IS NOT PRESENTING. The pointer is recorded for the capture path only; it is never
// published to the submit thread. See xrlayer_capture_source() for why that distinction is the
// whole safety argument.
//
// RETURNS whether the source was taken. A refusal is NOT terminal and must not be treated as one:
// the caller should keep re-offering, because the reason (a swapchain sized before the widget
// existed) can stop being true later in the same session. Latching "I handed it over" on a call
// that refused is what made a size mismatch permanent for a whole session and unclearable by any
// tunable -- reported from a headset on 2026-08-23 as "no game art, whatever I set".
bool xrlayer_set_source(void* d3d12_resource);

// The same, per slot. xrlayer_set_source() is exactly xrlayer_set_slot_source(0, ...).
//
// The size question it asks is now "does this resource match ITS CELL", not "does it match the
// swapchain": one swapchain image holds every slot's art as a texture ATLAS, so the reticule's
// 256x256 and a marker's 128x128 are both legal and both land in different rectangles of it.
bool xrlayer_set_slot_source(int slot, void* d3d12_resource);

// GAME THREAD, once per tick, immediately after the source has been re-validated end to end.
//
// Copies the accepted engine resource into a texture THIS MODULE OWNS, and presents that. This is
// the only place in the plugin that touches an engine-owned D3D12 resource.
//
// WHY IT IS HERE AND NOT ON THE SUBMIT THREAD. Three crashes on 2026-08-23, all the same
// instruction: an access violation in the NVIDIA UMD, reached from our own ResourceBarrier on the
// widget's render target, on an object that was still a live COM object (valid vtable, refcount 2,
// right device, GetDesc succeeds) but had lost its GPU backing. Nothing the submit thread can ask
// distinguishes that from a healthy resource, and AddRef does not keep the backing alive. Two
// generations of staleness detector -- a tick counter, then a 250 ms wall-clock heartbeat -- both
// failed, the second because it is a CHECK-THEN-USE whose gap includes a fence wait of up to a
// full second inside blit_into.
//
// Moving the read here does not make it provably safe; only UE's own render-command ordering could,
// and a UEVR plugin cannot enqueue into that. What it does is remove the state all three crashes
// were in: a game thread parked inside LoadMap issues no captures, so there is no code running in
// the window that used to keep barriering a dead resource.
//
// Never blocks: if the previous capture is still in flight it skips, and the overlay shows the
// previous widget frame. Returns false only when the layer is not in a state to capture.
bool xrlayer_capture_source();

// ---- THE BATCHED CAPTURE, for more than one slot ----------------------------------------------
//
// Nine slots cannot each submit their own command list: an allocator may not be reset while its
// commands are still executing, so nine sequential Reset/record/Close/Execute rounds on a two-deep
// ring would either block the game thread or corrupt. So the copies are RECORDED into one list and
// SUBMITTED once.
//
// THE CHECK-THEN-USE GAP IS NOT WIDENED BY THIS, and that is the property to preserve if you touch
// it. The dangerous operation is `ResourceBarrier` on an engine-owned resource -- that is the exact
// instruction all three 2026-08-23 crashes faulted in, because the driver dereferences the
// resource's allocation record there. Recording still happens IMMEDIATELY after that slot's
// re-validation, with nothing in between; only ExecuteCommandLists moves, and it moves by the
// microseconds it takes to validate the remaining slots on the same thread with no blocking call
// anywhere in the loop. Do not restructure this into "validate all, then record all".
//
// Usage, and it is the caller's job to keep it in this order:
//     if (xrlayer_capture_begin()) {
//         for each slot: { re-validate the chain; xrlayer_capture_record(slot); }
//         xrlayer_capture_submit();
//     }
// begin() returns false when the previous capture is still in flight (skip, never block) or when
// the layer is not armed. submit() is safe to call after zero records -- it just does nothing.
bool xrlayer_capture_begin();
bool xrlayer_capture_record(int slot);
void xrlayer_capture_submit();

// Tear down the swapchain, remove the hook and forget everything. Safe to call when nothing was
// ever set up.
//
// CALLED FROM plugin_teardown() (Plugin.cpp, step 0) AND on the xrlayer 1 -> 0 config edge.
//
// THIS COMMENT USED TO SAY THE OPPOSITE, AND IT COST SOMEBODY A HANG. It asserted that "this plugin
// has no unload path (uevr::Plugin exposes no destructor override and UEVR does not unload
// plugins), so the process exiting is what cleans up otherwise", and then told the reader not to
// add a teardown call without checking one exists. The check was mine to do and I did not do it:
// plugin_teardown() exists, and because nothing called this from it we left the xrEndFrame detour
// installed inside UEVRBackend plus an XR swapchain and D3D12 resources parented to a session UEVR
// was about to destroy. The game hung on exit, every archived hang log showing state=2 (Armed) at
// the moment of exit, and the log always stopping inside UEVR's own teardown.
//
// The lesson is not "add teardown calls". It is that a confident negative in a comment is read as
// research already done, so it stops the next person looking -- which is exactly the outcome a
// comment is supposed to prevent. Assert what you have checked, not what you assume.
//
// Safe to call when nothing was ever set up, and safe to call twice.
void xrlayer_shutdown();

}   // namespace halo
