// ScopeLayer -- present the scope's render target as a GUN-MOUNTED OpenXR quad composition layer
// instead of (or alongside) the in-world pane Scope.cpp builds.
//
// ============================================================================================
// WHY
// ============================================================================================
// The in-world pane is a StaticMeshComponent inside UE's scene, so everything that happens to the
// scene happens to it: pre-exposure, the tonemapper, TAA/TSR ghosting on a surface that is being
// re-textured every capture, and depth interaction with whatever the gun is in front of. A
// composition layer is submitted to the OpenXR runtime AFTER the whole post chain, at the
// swapchain's own resolution, so none of that touches it -- the same argument XrLayer.hpp makes
// for the reticule, applied to the one surface in this mod whose whole job is to be read.
//
// The reference design is Halo-MCC-VR's, recorded in docs\GameRecon\MCC-VR-Study.md:232. Verbatim
// from that file: an OpenXR quad composition layer (XR_TYPE_COMPOSITION_LAYER_QUAD, both eyes) at
// controller-local metric offsets -- width 0.159 m, right -0.058 m, up 0.216 m, forward 0.050 m --
// orientation = the aim quaternion. Those are the DEFAULTS below: field-proven numbers from a
// shipping mod rather than guesses. Read the caveats on each one in Config.hpp before treating
// them as fitted for THIS game -- they are a starting point, not a calibration.
//
// ============================================================================================
// THE POSE COMES FROM THE AIM RAY, NOT FROM THE GUN MODEL
// ============================================================================================
// Scope.hpp's design premise is that the scope shows WHERE THE SHOT GOES, not where the weapon
// mesh points -- the capture camera is placed on the aim ray for exactly that reason. This module
// is fed the SAME ray (scope_notice_ray's origin/target, unsmoothed) and builds the quad's frame
// from it, so the quad and the image inside it cannot disagree about the shot line.
//
// MCC's offsets are controller-local and their orientation is the aim quaternion; ours are both in
// the aim frame. That is a deliberate difference and it is the one this project's premise requires.
// The gun's ROLL still rides along (scopelayerroll), taken from the lens's own up vector and
// perpendicularised against the aim axis -- the construction MCC uses for its scope camera ("up =
// controller up projected perpendicular, so roll rides the gun") and the same one already written
// into Scope.cpp's roll lock.
//
// ============================================================================================
// THE SEAM -- one adapter block, and the five entry points it depends on
// ============================================================================================
// Every call into the compositor lane is in ONE block in ScopeLayer.cpp, marked as the adapter.
// Nothing else in this module -- and nothing anywhere else in the plugin -- names those symbols
// for the scope. The five it uses, all landed:
//
//   xrlayer_pane_configure(cell_px)        slot 9's atlas cell. BEFORE BRING-UP; false if late.
//   xrsource_set_slot_render_target(...)   UTextureRenderTarget2D -> validated ID3D12Resource, one
//                                          rung below the widget entry point (we hold the target
//                                          itself, so there is no GetRenderTarget hop to make).
//   xrlayer_notice_quad(...)               position + size, UE world cm, exactly like the markers.
//   xrlayer_set_quad_orientation(...)      two UE-world DIRECTIONS. Separate from the position on
//                                          purpose, so "stop driving it" degrades to head-locked
//                                          rather than to a stale weapon angle.
//   xrlayer_clear_quad_orientation(...)    back to head-oriented, explicitly.
//
// Each is bound with a static_assert on its exact signature, so a change on their side is a build
// error that NAMES THE SYMBOL, and each logs once when it is first actually reached. Both halves
// are deliberate: a compile-time check cannot tell you a call site is never executed, and a runtime
// proof cannot tell you a symbol quietly stopped existing. An earlier draft of this file used
// variadic fallbacks to tolerate an absent API, and that is exactly how a typo'd, never-existed
// symbol compiled into a permanent silent no-op. Do not reintroduce that.
//
// ON THE FACING CONVENTION, because it looks like a contradiction and is not. The quad's VISIBLE
// FACE NORMAL points back at the shooter -- that follows from compute_pose(), whose facing axis is
// +Z and whose +Z points at the viewer. The layer's parameter is named `fwd_world` and wants the
// DOWNRANGE direction, normalize(target - origin), unflipped. Same surface, read from opposite
// ends: the face looks THROUGH the glass, downrange. Negating it would put the pane's back to the
// player and make it invisible from the front.

#pragma once

// API.hpp, NOT Plugin.hpp -- see UeObject.hpp.
#include "uevr/API.hpp"
#include "Math.hpp"

#include <cstdint>

namespace halo {

// Everything the quad needs, gathered by the caller that already has it in hand.
//
// PUSHED IN RATHER THAN REACHED FOR, the same rule XrSource.hpp states for the navpoint pool:
// this module has no business resolving the scope's components, and exporting them would make
// their lifetime rules everybody's problem. Scope.cpp validates them every tick anyway.
struct ScopeLayerFeed {
    Vec3 ray_origin{0.0f, 0.0f, 0.0f};   // UE world cm -- the RAW aim ray, same one the capture uses
    Vec3 ray_target{0.0f, 0.0f, 0.0f};

    // The lens's own world up vector (pane K2 GetUpVector). Supplies the gun's ROLL, and only the
    // roll: it is perpendicularised against the aim axis before use. valid=false falls back to a
    // world-derived up, which is level but does not cant with the weapon.
    //
    // The layer orthogonalises and normalises up_world itself and asks for neither, but this module
    // needs a real orthonormal basis anyway to PLACE the quad, so it is built once and used twice.
    // Its one refusal case is an up parallel to the aim -- no roll reference -- which is guarded
    // and logged here rather than left to degrade into a head-oriented quad nobody asked for.
    Vec3 lens_up{0.0f, 0.0f, 0.0f};
    bool lens_up_valid = false;

    // The UTextureRenderTarget2D the scope capture writes, and the square edge it was actually
    // CREATED at (s_rt_size_applied, not the config key -- a live scoperes edit rebuilds the target
    // and the two disagree for a tick). XrSource validates the whole resolved chain against this
    // number, so a guess here is a wrong ValueAgreement, not a cosmetic slip.
    uevr::API::UObject* render_target = nullptr;
    int                 rt_dim        = 0;

    // THE IN-WORLD PANE'S OWN PLACEMENT, so the quad can be derived from it instead of from a
    // parallel set of numbers. The pane is where the whole calibration lands -- scopedist/right/up,
    // the per-weapon wpnscope trims, the socket handshake, everything -- and duplicating that into
    // scopelayerfwd/right/up guarantees the two drift apart the moment anyone calibrates.
    // Reading the placed component's world transform inherits all of it for free and cannot go
    // stale, because it IS the answer rather than a copy of the inputs to it.
    // valid=false falls back to the scopelayer* offsets, so a build where the location UFUNCTION
    // is not callable degrades to the old behaviour rather than putting the quad at the origin.
    Vec3  pane_world{0.0f, 0.0f, 0.0f};
    float pane_width_cm = 0.0f;
    bool  pane_valid    = false;

    // WHERE THE SHOT ACTUALLY LANDS, as normalised pane coordinates in -1..1 (0,0 = pane centre,
    // +u right, +v up). The scope reticule quad was pinned to dead pane centre, which is only
    // correct while the capture camera's axis and the traced impact point agree -- true for
    // scopecamtrack=0 at rest, and false the moment the camera rides the weapon through recoil,
    // sway, or any mode where it is not re-anchored to the live ray every tick.
    //
    // Computed by PROJECTING the traced hit point into the capture camera's OWN frustum (its real
    // world basis, read back from the component, and the FOV actually written to it) rather than
    // assumed from the aim ray -- the whole point is to catch the cases where those two differ.
    // valid=false leaves the reticule at centre, i.e. exactly the old behaviour.
    float ret_u      = 0.0f;
    float ret_v      = 0.0f;
    bool  ret_valid  = false;
    // THE CAPTURE CAMERA'S OWN AXES. The pane image is rendered in this frame, so it is the frame
    // the reticule offset is expressed in -- and it is NOT the quad's frame, because the camera
    // carries scope_cam_roll + uv_roll + the live roll lock. Carried so the layer can measure the
    // angle between the two rather than assume it.
    Vec3  cam_fwd{1.0f, 0.0f, 0.0f};
    Vec3  cam_right{0.0f, 1.0f, 0.0f};
    Vec3  cam_up{0.0f, 0.0f, 1.0f};
    bool  cam_axes_valid = false;

    // The pane's OWN facing axis. With scopelayerfollowpane the quad should BE the pane
    // geometrically -- position and orientation both -- rather than sitting at the pane's position
    // while facing where the ray points. Those are different frames, and the gap between them
    // changes as the weapon rotates, which is what produced a non-constant offset that no placement
    // constant could explain.
    //
    // ROLL IS THE ONE DELIBERATE EXCEPTION: a real optic does not spin its image when the rifle
    // cants, so roll keeps coming from scopelayerroll rather than from the pane.
    // ALL THREE of the pane's axes, because WHICH ONE FACES DOWNRANGE IS NOT KNOWABLE FROM HERE.
    // Measured 2026-09-07: the pane's GetForwardVector reads -0.17 against the aim -- near
    // perpendicular, so it is not the facing axis at all. That is what a Plane mesh does: its
    // normal is local +Z, i.e. its UP vector, which is exactly why the lens roll already reads
    // GetUpVector rather than GetForwardVector.
    //
    // Rather than swap one assumption for another, all three are published and the facing axis is
    // CHOSEN by which is most aligned with the aim. That is a measurement, it costs two extra
    // UFUNCTION calls per tick, and it cannot be wrong on a shape nobody tested.
    Vec3 pane_fwd{0.0f, 0.0f, 0.0f};
    Vec3 pane_right{0.0f, 0.0f, 0.0f};
    Vec3 pane_up_axis{0.0f, 0.0f, 0.0f};
    bool pane_fwd_valid = false;

    // THE RIG'S WORLD ROTATION ON THIS TICK, so the layer can correct the orientation for however
    // far the rig turns before the frame is actually drawn.
    //
    // The in-world pane is parented to the weapon socket and its transform is composed at RENDER
    // rate -- Plugin.cpp recomposes the rig every frame and reports correcting "up to 161 deg of
    // parent motion since tick". Everything above is a ~32 Hz sample of that, so a quad built from
    // it agrees with the pane while still and trails it while you rotate. Publishing the rig
    // rotation these vectors were measured against lets the submit thread apply the same delta.
    // The rig's own AXES on this tick -- vectors, never Euler angles. An Euler round-trip
    // redistributes between its three terms as pitch changes and degenerates near vertical, which
    // showed up as the pane going jittery when the controller was rolled.
    Vec3 rig_pos{0.0f, 0.0f, 0.0f};
    Vec3 rig_fwd{1.0f, 0.0f, 0.0f};
    Vec3 rig_right{0.0f, 1.0f, 0.0f};
    Vec3 rig_up{0.0f, 0.0f, 1.0f};
    bool rig_rot_valid = false;
};

// GAME THREAD, from scope_apply() -- i.e. only on a tick that produced a real aim ray, with the
// scope open and its components validated. Computes the quad pose and offers it to the compositor.
// Costs one bool test while scopelayer is off.
void scopelayer_notice(const ScopeLayerFeed& feed, uint32_t tick);

// GAME THREAD, every tick, from scope_frame_end() -- ABOVE its early-outs, like xrlayer_tick().
// Owns retirement (no notice within the grace window = the scope is not up), the config edges and
// the state logging. Costs one bool test while scopelayer is off.
// Ask for the pane's atlas cell BEFORE the layer brings up. Must be called from update()
// ahead of xrlayer_tick(); see the definition for the 92 ms race it exists to lose.
void scopelayer_configure_cell_early();

void scopelayer_tick(uint32_t tick);

// True ONLY while the compositor is proven to be presenting this slot's art -- our source was
// accepted AND XrLayer reports the slot ready. Never true merely because we asked.
//
// This is what scopelayerhidepane gates on. The rule is XrLayer.hpp's, learned the expensive way on
// the reticule: gate on LIVE, never on "setup returned success", or a refused layer leaves the
// player looking at nothing at all.
bool scopelayer_presenting();

} // namespace halo
