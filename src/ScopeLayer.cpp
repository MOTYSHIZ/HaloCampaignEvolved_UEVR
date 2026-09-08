// Gun-mounted scope on an OpenXR quad composition layer. Design and doctrine in ScopeLayer.hpp.
//
// Nothing in this file touches the engine except one property readback on the render target (the
// squareness proof). It takes a ray, a render target and a lens up-vector from Scope.cpp, turns
// them into a quad pose, and offers the pair to the compositor modules through ONE adapter.

#include "ScopeLayer.hpp"
#include "Config.hpp"
#include "DevTools.hpp"

// The other lane's modules. THESE TWO INCLUDES, AND THE ADAPTER BLOCK BELOW, ARE THE ONLY PLACE
// this feature knows their names. Read-only from here: XrLayer.cpp / XrSource.cpp are owned
// elsewhere and are actively being edited.
#include "XrLayer.hpp"
#include "XrSource.hpp"

#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <type_traits>

// THE FINISHED VIEW, for the head-basis self-check arm, COMES FROM THE LAYER MODULE -- and the two
// ways it could have come from here instead are both wrong. Recorded because both look reasonable:
//
//   * `extern std::atomic<float> g_render_view_yaw` and friends DO NOT LINK. Plugin.cpp:156 opens an
//     ANONYMOUS NAMESPACE and they are inside it, so they have internal linkage and there is no
//     external symbol to bind to. No amount of qualification or header reordering changes that, and
//     exposing them would mean editing Plugin.cpp, which three sessions share tonight.
//   * Rebuilding the basis from yaw and pitch WOULD HAVE REPRODUCED A FIXED BUG. Those two atomics
//     carry no ROLL -- Plugin.cpp keeps the view roll as a local, deliberately -- and XrLayer.hpp
//     records, above xrlayer_note_eye(), that reconstructing the camera basis without roll makes the
//     two frames disagree and swings the quad about the view axis: "was a live bug". A validator
//     that is only wrong when the player tilts their head is worse than no validator, because it
//     launders a bad frame as verified.
//
// So the arm asks the module that already receives all three angles. See layer_view_basis().

using namespace uevr;

namespace halo {
namespace {

// ---- tiny vector helpers (UE world space, centimetres) ---------------------------------------
//
// UE's basis is X forward, Y right, Z up. The two identities used below -- cross(up, fwd) == right
// and cross(fwd, right) == up -- hold under the ordinary numeric cross product in that basis, and
// they are the only frame math this file does. The handedness question that genuinely bites (UE is
// left-handed, OpenXR is not, and the module's basis map has determinant -1) never arises here,
// because the layer's orientation entry point takes DIRECTION VECTORS and does that conversion
// itself through the one mapping it has already proven in a headset. See XrLayer.hpp on
// xrlayer_set_quad_orientation -- it is emphatic about why, and it is worth reading before
// "simplifying" any of this into a quaternion.
Vec3 vcross(const Vec3& a, const Vec3& b) {
    return Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
float vdot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
float vlen(const Vec3& v) { return std::sqrt(vdot(v, v)); }
bool  vnorm(Vec3* v) {
    const float l = vlen(*v);
    if (!(l > 1e-4f)) return false;
    v->x /= l; v->y /= l; v->z /= l;
    return true;
}

// The quad, fully described, in UE world space and centimetres -- the same space and unit
// xrlayer_notice_quad's world_pos already uses. NOT metres, NOT OpenXR axes: both halves of the
// pose cross through one mapping inside the layer module, and converting here would apply it twice.
struct ScopeQuad {
    Vec3  centre{0.0f, 0.0f, 0.0f};   // cm
    bool  follow_pane = false;        // placed FROM the in-world pane, not our offsets
    float width_cm = 0.0f;            // cells are SQUARE, so this is the height too
    // DOWNRANGE, unflipped: normalize(target - origin). The quad's visible face NORMAL does point
    // back at the shooter -- compute_pose()'s facing axis is +Z and its +Z faces the viewer -- but
    // the layer's parameter is named for the direction the face looks THROUGH. Same surface, read
    // from opposite ends. Negating it would put the pane's back to the player.
    // The zero vector is this module's own marker for the control arm, never a computed value.
    Vec3  fwd{0.0f, 0.0f, 0.0f};
    Vec3  up{0.0f, 0.0f, 0.0f};       // unit here, though the layer orthogonalises it anyway
    bool  rolled     = false;         // `up` carries the weapon's cant
    bool  head_basis = false;         // the self-check arm built this from the VIEW, not the aim
    // Where the reticule sits WITHIN the pane, normalised -1..1. Carried on the quad rather
    // than read from the feed at the call site so the pose and the offset are always the
    // same tick's answer -- they describe one geometry and must not be assembled from two.
    float ret_u     = 0.0f;
    float ret_v     = 0.0f;
    bool  ret_valid = false;
    // The rig rotation `fwd`/`up` were measured against, carried alongside them so the pair can
    // never be assembled from two different ticks -- the correction is a difference between two
    // measurements of one thing, and mixing instants reintroduces the error it removes.
    Vec3  rig_pos{0.0f, 0.0f, 0.0f};
    Vec3  rig_fwd{1.0f, 0.0f, 0.0f};
    Vec3  rig_right{0.0f, 1.0f, 0.0f};
    Vec3  rig_up{0.0f, 0.0f, 1.0f};
    bool  rig_rot_valid = false;
};

struct LayerResult {
    bool        offered    = false;   // source + pose + orientation all published this tick
    bool        presenting = false;   // the layer says it is drawing this slot RIGHT NOW
    int         cell_dim   = 0;       // the atlas cell edge this slot owns, 0 = none
    const char* refused    = nullptr; // exactly why, when either of the above is false
};

// THE DROP ORDER IS THE LAYER MODULE'S, AND WE DO NOT FIGHT IT: reticule (0, never dropped), then
// navpoint markers (1 objectives, 2 everything else -- Plugin.cpp), then the pane. The pane is the
// newest and least proven of the three and is the first thing shed when the runtime is short of
// layers. The atlas builder applies the same order for space; see build_atlas_layout().
constexpr int kPanePriority = 3;

// The slot default in Config.hpp is a literal 9 because Config.hpp must not include another
// module's header. This is what stops the two drifting apart in silence.
static_assert(XRLAYER_SLOT_PANE == 9,
              "Config.hpp's scope_layer_slot default (9) and XRLAYER_SLOT_PANE have diverged -- "
              "update the default and its comment together.");

} // namespace

#if HALO_VR_DEV
// ==============================================================================================
// THE ONE ENTRY POINT THAT HAS NOT LANDED -- and the ONLY tolerated binding in this file
// ==============================================================================================
// `bool xrlayer_view_basis(Vec3* fwd_world, Vec3* up_world)` is requested from the layer lane and
// may or may not appear. It feeds the head-basis SELF-CHECK arm and nothing else, so the shipping
// feature does not wait on it -- but the arm has to compile either way.
//
// A SIBLING namespace whose variadic overload is the worst possible match, so the real declaration
// wins the moment it exists, with no edit here. THIS IS THE ONLY PLACE IN THIS FILE WHERE THAT
// PATTERN IS ALLOWED, and it carries the tax that makes it safe: reaching the stub LOGS, once,
// naming the symbol. An earlier draft used this pattern for everything, and that is precisely how
// `xlayer_notice_quad_oriented` -- a function that never existed in any version of their header --
// compiled into a permanent silent no-op indistinguishable from "their module is broken". A
// tolerant seam without an alarm is a defect generator. Everything else here binds by address with
// a static_assert on its exact signature, so a typo cannot compile at all.
namespace scopelayer_pending {
struct absent_t { };
inline absent_t xrlayer_view_basis(...) { return {}; }
}   // namespace scopelayer_pending
#endif

// ==============================================================================================
// THE ADAPTER -- the ONLY place another lane's symbols are named
// ==============================================================================================
//
// Six landed entry points, all owned by the compositor lane, plus one that has not landed:
//
//   xrlayer_pane_configure(cell_px)          ask for slot 9's atlas cell. BEFORE BRING-UP.
//   xrsource_set_slot_render_target(...)     UTextureRenderTarget2D -> validated ID3D12Resource.
//   xrlayer_notice_quad(...)                 position + size, UE world cm, exactly as the markers.
//   xrlayer_set_quad_orientation(...)        two UE-world DIRECTIONS -- not a rotator, not a quat.
//   xrlayer_clear_quad_orientation(...)      back to head-oriented.
//   xrlayer_retire_quad(...)                 stop submitting; also clears the orientation override.
//   xrlayer_view_basis(...)      [PENDING]   dev self-check only; see the block above this one.
//
// POSITION AND ORIENTATION ARE SEPARATE CALLS, and that separation is load-bearing rather than
// incidental: a caller that stops driving the orientation falls back to HEAD-LOCKED, not to a stale
// weapon angle. "Stop" is therefore a safe state, which is why nothing has to be nulled on scope
// close beyond the retire path.
//
// THE ONE THING TO UNDERSTAND BEFORE EDITING THIS BLOCK is why the orientation is two vectors.
// XrLayer.hpp spells it out: rotator_to_quat() builds a quaternion in UE's LEFT-handed frame,
// XrPosef::orientation is OpenXR's right-handed one, and the basis map between them
// (x = y, y = z, z = -x) has determinant -1. Carrying a rotation across a handedness flip by
// copying components produces an error that is near ZERO looking straight down the sights -- the
// only time anyone looks through a scope -- and grows off-axis. It would pass every deliberate
// test and fail in play. Directions go through ue_offset_to_xr(), the one mapping in that module
// proven in a headset, and the quaternion is built on the far side. Do not "simplify" this.
//
// EVERYTHING IN UE WORLD SPACE AND CENTIMETRES. No metres, no OpenXR axes, no world-scale division
// on this side -- the layer applies its own cm-per-metre once, to both halves of the pose.
namespace scopelayer_adapter {

// ---- BINDING PROOF, BOTH HALVES -----------------------------------------------------------
//
// AT COMPILE TIME: every symbol is bound by taking its address and asserting its exact type. A
// signature change on their side is then a build error that NAMES THE SYMBOL, and an invented or
// typo'd name cannot compile at all.
//
// THIS REPLACED A VARIADIC-FALLBACK SCHEME, and the reason is worth keeping. That scheme let this
// file compile against an API that did not exist yet, by falling through to `f(...)` stubs. It also
// let `xlayer_notice_quad_oriented` -- a function that has never existed in any version of their
// header -- compile into a permanent silent no-op that looked exactly like "their module is
// broken". A tolerant seam is a defect generator when the thing on the far side is a moving target:
// every typo becomes a runtime mystery instead of a build error. Do not reintroduce it.
constexpr auto kFnPaneConfigure  = &xrlayer_pane_configure;
constexpr auto kFnSetRenderTarget = &xrsource_set_slot_render_target;
constexpr auto kFnNoticeQuad     = &xrlayer_notice_quad;
constexpr auto kFnSetOrientation = &xrlayer_set_quad_orientation;
constexpr auto kFnClearOrient    = &xrlayer_clear_quad_orientation;
constexpr auto kFnRetireQuad     = &xrlayer_retire_quad;

static_assert(std::is_same_v<decltype(kFnPaneConfigure), bool (*const)(int)>,
              "xrlayer_pane_configure(int) changed shape -- update the scope adapter");
static_assert(std::is_same_v<decltype(kFnSetRenderTarget), void (*const)(int, void*, int)>,
              "xrsource_set_slot_render_target(int, void*, int) changed shape -- update the scope adapter");
// Gained a SIXTH parameter 2026-09-04: `float world_cm_h`, the quad's second world extent, added
// for the grab guide's beam. It is DEFAULTED to 0 = square in the header, so this file's call site
// is unchanged and the scope pane stays square as it always was -- but the pointer type changed, so
// the bind below had to be updated with it. That is the assertion doing its job: a defaulted
// parameter is invisible at every call site and would otherwise have been an entirely silent ABI
// change on a seam whose whole purpose is that changes here cannot be silent.
static_assert(std::is_same_v<decltype(kFnNoticeQuad),
                             void (*const)(int, const Vec3&, float, float, int, float)>,
              "xrlayer_notice_quad(int, const Vec3&, float, float, int, float) changed shape -- update the scope adapter");
static_assert(std::is_same_v<decltype(kFnSetOrientation),
                             void (*const)(int, const Vec3&, const Vec3&)>,
              "xrlayer_set_quad_orientation(int, const Vec3&, const Vec3&) changed shape -- update the scope adapter");
static_assert(std::is_same_v<decltype(kFnClearOrient), void (*const)(int)>,
              "xrlayer_clear_quad_orientation(int) changed shape -- update the scope adapter");
static_assert(std::is_same_v<decltype(kFnRetireQuad), void (*const)(int)>,
              "xrlayer_retire_quad(int) changed shape -- update the scope adapter");

// Kept because a compile-time bind proves the symbol EXISTS and proves nothing about whether the
// call site is ever executed. Logged when the feature arms, which covers the "never reached" case
// that a runtime proof by definition cannot report.
constexpr bool kHaveSourceApi = true;
constexpr bool kHavePoseApi   = true;

// AT RUNTIME: say so the first time each entry point is genuinely reached. PROVE THE TICK, NOT THE
// INIT -- this project has already shipped a periodic function with zero callers that read as
// working for days. One line per symbol per session, then free forever.
enum Bind { BIND_CELL, BIND_SRC, BIND_QUAD, BIND_ORIENT, BIND_CLEAR, BIND_RETIRE, BIND_N };
bool g_bind_seen[BIND_N] = {false, false, false, false, false, false};
const char* const kBindName[BIND_N] = {
    "xrlayer_pane_configure", "xrsource_set_slot_render_target", "xrlayer_notice_quad",
    "xrlayer_set_quad_orientation", "xrlayer_clear_quad_orientation", "xrlayer_retire_quad"};

void bind_proof(Bind b) {
    if (g_bind_seen[b]) return;
    g_bind_seen[b] = true;
    API::get()->log_info("[Halo-CampE-UEVR] scopelayer: reached %s() for the first time -- the "
                         "call site is live, not merely compiled", kBindName[b]);
}

#if HALO_VR_DEV
// ---- THE UNLANDED ONE. Merge the real declaration (if it exists) and the stub into one overload
// set; the return TYPE says which ran. Never silent: the stub announces itself, once, by name.
using namespace ::halo;
using namespace ::halo::scopelayer_pending;

bool basis_took(scopelayer_pending::absent_t) {
    static bool s_said = false;
    if (!s_said) {
        s_said = true;
        API::get()->log_info(
            "[Halo-CampE-UEVR] scopelayer: STUBBED CALL REACHED -- xrlayer_view_basis(Vec3*, Vec3*) "
            "is not declared in XrLayer.hpp, so this call did NOTHING. The scopelayer=2 self-check "
            "arm cannot run and refuses; scopelayer=1 (the real feature) is unaffected. If that "
            "entry point is declined, DROP the arm rather than rebuilding the view basis from "
            "another source -- a validator that is subtly wrong launders a bad frame as verified.");
    }
    return false;
}
bool basis_took(bool v) { return v; }

constexpr bool kHaveViewBasisApi = !std::is_same_v<
    decltype(xrlayer_view_basis((Vec3*)nullptr, (Vec3*)nullptr)), scopelayer_pending::absent_t>;

// The finished VIEW's forward and up, in UE world space, from the module that already receives all
// THREE angles including roll. Refuses -- never returns a zero basis -- when no stereo frame has
// been composed yet (pre-injection, the frontend), because a zero basis is not obviously wrong and
// would poison the self-check in silence.
bool layer_view_basis(Vec3* fwd, Vec3* up) {
    *fwd = Vec3{0.0f, 0.0f, 0.0f};
    *up  = Vec3{0.0f, 0.0f, 0.0f};
    if (!basis_took(xrlayer_view_basis(fwd, up))) return false;
    const float fl = std::sqrt(fwd->x * fwd->x + fwd->y * fwd->y + fwd->z * fwd->z);
    const float ul = std::sqrt(up->x * up->x + up->y * up->y + up->z * up->z);
    return fl > 1e-4f && ul > 1e-4f;
}
#endif

// Ask for the pane's atlas cell. Returns whether the request is going to take effect: false means
// either that it cannot fit alongside the reticule and the markers (which are never sacrificed for
// it) or that the layer is already up and the atlas cannot be resized under a live submit thread.
bool layer_configure_cell(int cell_px) {
    bind_proof(BIND_CELL);
    return xrlayer_pane_configure(cell_px);
}

int layer_cell_dim(int slot) {
    if (slot < 0 || slot >= XRLAYER_SLOTS) return 0;
    return xrlayer_cell_dim(slot);
}

// THE one call on the hot path. Offers the render target, the pose and the orientation; reports
// precisely how far it got.
//
// FAILS CLOSED AND SILENT TOWARD THE GAME: every refusal leaves the in-world pane untouched and
// unaware, exactly like XrLayer's own contract. A refusal is never terminal -- the caller keeps
// re-offering, because the reason (a swapchain sized before the scope existed) can stop being true
// later in the same session.
LayerResult layer_present(int slot, void* render_target, int rt_dim, const ScopeQuad& q,
                          char* detail, size_t detail_sz) {
    LayerResult r;
    if (detail != nullptr && detail_sz > 0) detail[0] = '\0';

    if (!g_cfg.xr_layer) {
        r.refused = "xrlayer=0 -- the compositor layer module itself is switched off";
        return r;
    }
    if (slot < 0 || slot >= XRLAYER_SLOTS) {
        r.refused = "scopelayerslot is outside the layer's slot range";
        if (detail != nullptr) {
            std::snprintf(detail, detail_sz, "slot %d, XRLAYER_SLOTS=%d, pane slot is %d",
                          slot, XRLAYER_SLOTS, XRLAYER_SLOT_PANE);
        }
        return r;
    }
    if (slot == XRLAYER_SLOT_RETICULE) {
        r.refused = "scopelayerslot 0 is the RETICULE -- refusing to overwrite the crosshair";
        return r;
    }

    // A source is only accepted when its dimensions equal ITS CELL, so check it here: the reason
    // then reads as two numbers instead of as a silent refusal three modules away.
    r.cell_dim = xrlayer_cell_dim(slot);
    if (r.cell_dim == 0) {
        r.refused = "this slot has no atlas cell";
        if (detail != nullptr) {
            std::snprintf(detail, detail_sz,
                          "either the layer has not brought up yet, or xrlayer_pane_configure(%d) "
                          "was refused or came too late -- see the 'pane cell' line above",
                          rt_dim);
        }
        return r;
    }
    if (r.cell_dim != rt_dim) {
        r.refused = "the scope render target does not match this slot's atlas cell";
        if (detail != nullptr) {
            std::snprintf(detail, detail_sz,
                          "cell %dx%d vs render target %dx%d -- set scoperes=%d and re-bring-up "
                          "the layer (xrlayer 0 -> 1), since the atlas cannot be resized live",
                          r.cell_dim, r.cell_dim, rt_dim, rt_dim, r.cell_dim);
        }
        return r;
    }

    // Records only -- xrsource_tick() does the resolve, and it runs EARLIER in the update than our
    // caller does, so this lands one tick later. That is the same latency the navpoint pool has and
    // it is why the offer is repeated every tick rather than latched.
    bind_proof(BIND_SRC);
    xrsource_set_slot_render_target(slot, render_target, rt_dim);

    // Position and size. hold_cm = 0: a gun-mounted quad is AT its real distance by definition,
    // which is also the vergence the eyes should converge at.
    //
    // HEAD-RELATIVE, because this quad is GUN-MOUNTED and therefore moves WITH the player.
    //
    // compute_pose does d_world = target - eye, with the target chosen on the ~32 Hz game tick and
    // the eye belonging to the frame being drawn. For a world-fixed navpoint that is exactly right.
    // For anything glued to the player it is exactly wrong: while you locomote, gun and eye
    // translate together, so a fresh eye is differenced against a stale gun and the quad sits one
    // tick of travel BEHIND. At a walking pace that is several centimetres, and it reads as the
    // pane lagging and jittering when you move -- reported in a headset 2026-09-07, and the same
    // symptom the reticule and grab guide already carry this fix for.
    //
    // Publishing (target - eye) and letting the layer add a FRESH eye back at render rate makes both
    // sides of the subtraction come from the same instant, so the translation cancels and what
    // remains is sub-frame rather than sub-tick.
    //
    // The flag and the vector are set TOGETHER and never apart: the flag says how to INTERPRET the
    // vector, so a stale one reinterprets a world position as an offset and throws the quad a whole
    // world-origin away. That failure reads as "the layer vanished", not as a bad number, and
    // nothing in the log would say why -- which is precisely the shape of bug this session spent
    // hours on. Falling back to the world point when no view has been composed keeps the worst case
    // at the OLD behaviour rather than a quad at the origin.
    // RIG-RELATIVE beats head-relative for a weapon-mounted quad, and the difference is measurable:
    // head-relative cancels translation the target shares with the EYE, which fixed locomotion but
    // does nothing when the weapon swings about a stationary head. Measured with scopelayer=3 (no
    // orientation published at all) the quad still hopped while the controller rotated, which
    // isolates the fault to POSITION and to the rig frame rather than the head's.
    //
    // The rig frame already carries the body's translation, so this cancels both and head-relative
    // is switched off for the slot rather than stacked with it -- two anchors would each try to
    // remove the same motion.
    const bool rig_rel = q.rig_rot_valid && q.follow_pane;
    Vec3 anchor = q.centre;
    if (!rig_rel) {
        Vec3 eye{};
        const bool head_rel = xrlayer_mono_view_pos(&eye);
        if (head_rel) {
            anchor = Vec3{q.centre.x - eye.x, q.centre.y - eye.y, q.centre.z - eye.z};
        }
        xrlayer_set_quad_head_relative(slot, head_rel);
    } else {
        xrlayer_set_quad_head_relative(slot, false);
    }

    bind_proof(BIND_QUAD);
    xrlayer_notice_quad(slot, anchor, q.width_cm, 0.0f, kPanePriority);
    // AFTER notice_quad: it decomposes the target the layer has just been given, so the world
    // position must already be stored. Ordering them the other way silently decomposes the PREVIOUS
    // tick's target against this tick's rig.
    xrlayer_set_quad_rig_relative(slot, rig_rel, q.rig_pos, q.rig_fwd, q.rig_right, q.rig_up);

    // The reticule's position WITHIN the pane, normalised. Published every tick the pose is, so the
    // two can never describe different frames -- and cleared to centre when the projection did not
    // resolve, rather than left holding the last good value while the scope points somewhere else.
    xrlayer_set_scope_reticle_offset(q.ret_u, q.ret_v, q.ret_valid);

    // Orientation, published SEPARATELY from the position and every tick the position is. The split
    // is theirs and it is a good one: a caller that simply stops driving orientation falls back to
    // head-locked rather than holding a stale weapon angle, so "stop" is a safe state.
    //
    // `fwd_world` IS THE DOWNRANGE DIRECTION, unflipped. The quad's visible face normal does point
    // back at the shooter -- that follows from compute_pose(), whose facing axis is +Z and whose +Z
    // points at the viewer -- but the parameter is named for the direction the face looks THROUGH,
    // which is downrange. Same surface, opposite ends. Negating it here would turn the pane's back
    // to the player and make it invisible from the front.
    //
    // The zero vector is this module's own "control arm" marker, never a value we compute.
    if (q.fwd.x != 0.0f || q.fwd.y != 0.0f || q.fwd.z != 0.0f) {
        bind_proof(BIND_ORIENT);
        // TRACKED when we know which rig rotation these vectors belong to, so the layer can correct
        // for the rig turning between this tick and the frame that draws it. Plain otherwise --
        // the untracked call is exactly the old behaviour, so a build with no rig reading degrades
        // to trailing rather than to nothing.
        if (q.rig_rot_valid) {
            xrlayer_set_quad_orientation_tracked(slot, q.fwd, q.up,
                                                 q.rig_fwd, q.rig_right, q.rig_up);
        } else {
            xrlayer_set_quad_orientation(slot, q.fwd, q.up);
        }
    } else {
        bind_proof(BIND_CLEAR);
        xrlayer_clear_quad_orientation(slot);
    }
    r.offered = true;

    // PRESENTING is not "we asked". xrlayer_slot_ready() is true only when the slot has real art
    // captured into the atlas and that capture is inside the hold window.
    r.presenting = xrlayer_slot_ready(slot);
    if (!r.presenting) {
        r.refused = "offered and accepted, but the layer does not report the slot ready yet -- no "
                    "capture in the hold window. Normal for a tick or two after opening; "
                    "persistent means the source chain has not resolved (see the XRSRC lines)";
    }
    return r;
}

void layer_clear_orientation(int slot) {
    if (slot < 0 || slot >= XRLAYER_SLOTS) return;
    bind_proof(BIND_CLEAR);
    xrlayer_clear_quad_orientation(slot);
}

// Stop presenting. xrlayer_retire_quad() also clears the orientation override, so a slot reused for
// something else cannot inherit a stale weapon rotation.
void layer_retire(int slot) {
    if (slot < 0 || slot >= XRLAYER_SLOTS) return;
    bind_proof(BIND_RETIRE);
    xrlayer_retire_quad(slot);
    // AND DISCARD THE ART. retire_quad keeps it on purpose -- right for a navpoint, which comes
    // back as the same marker -- but this slot's cell holds a picture of the weapon we just put
    // away, at its zoom, of a scene we may no longer be looking at. With xrlayerhold at 1500 ms
    // that stale cell still reads READY, so re-scoping inside a second and a half drew the previous
    // session's frozen frame until a new capture landed. Retirement is the moment we know this
    // slot's meaning changes; the orientation override is already dropped here for exactly that
    // reason, and the art deserves the same treatment.
    xrlayer_invalidate_capture(slot);
    bind_proof(BIND_SRC);
    // Drop the source too. Our render target is rebuilt on a scoperes change, on a capture-source
    // format change and on every level load, so a slot left pointing at it across a teardown is a
    // dangling pointer the compositor could try to copy from.
    xrsource_set_slot_render_target(slot, nullptr, 0);
}

} // namespace scopelayer_adapter

// ==============================================================================================
// State and logging
// ==============================================================================================
namespace {

bool     s_published   = false;   // a quad is out and not yet retired
bool     s_presenting  = false;   // last adapter verdict
uint32_t s_last_notice = 0;
int      s_slot_out    = -1;      // which slot the live quad went to (retirement needs it)

// Config edges. Applied ON CHANGE, never per tick -- the rule the whole live-tuning scheme rests on.
int   s_cfg_mode   = -1;
int   s_cfg_slot   = -1;
int   s_cfg_roll   = -1;
int   s_cfg_cell   = -1;          // the cell px we last asked xrlayer_pane_configure for
float s_cfg_fwd    = 1e9f, s_cfg_right = 1e9f, s_cfg_up = 1e9f, s_cfg_width = 1e9f;

// The squareness proof, cached per render-target identity.
void* s_rt_checked = nullptr;
bool  s_rt_square  = false;
int   s_rt_x = 0, s_rt_y = 0;

void logf(const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    API::get()->log_info("[Halo-CampE-UEVR] scopelayer: %s", buf);
}

// EVERY EARLY RETURN SAYS WHY, and says it at a rate a log can carry. Rate-limited by CHANGE rather
// than by a counter: a new reason prints immediately (which is what makes a bisect possible), an
// unchanged one reprints about every 8 s so a persistent stall never disappears.
char     s_reason[352] = {0};
bool     s_reason_seen = false;
uint32_t s_reason_tick = 0;

void say(uint32_t tick, const char* fmt, ...) {
    char buf[352];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    constexpr uint32_t kRepeatTicks = 256;   // ~8 s at ~32 Hz
    if (s_reason_seen && std::strcmp(buf, s_reason) == 0 &&
        (uint32_t)(tick - s_reason_tick) < kRepeatTicks) {
        return;
    }
    std::memcpy(s_reason, buf, sizeof(s_reason));
    s_reason_seen = true;
    s_reason_tick = tick;
    API::get()->log_info("[Halo-CampE-UEVR] scopelayer: not presenting -- %s", buf);
}

void forget_reason() { s_reason_seen = false; s_reason[0] = '\0'; }

// UE centimetres per real-world metre, i.e. 100 x VR_WorldScale. Read the same way XrLayer reads it
// (the raw C accessor, because the header's float path runs std::stof on an empty buffer when the
// key is absent). Called only when something is being LOGGED -- never on the per-tick path.
float cm_per_m() {
    float ws = 1.0f;
    char buf[64]{};
    if (auto* p = API::get()->param(); p != nullptr && p->vr != nullptr &&
                                       p->vr->get_mod_value != nullptr) {
        p->vr->get_mod_value("VR_WorldScale", buf, sizeof(buf));
        if (buf[0] != 0) {
            const float v = (float)std::atof(buf);
            if (v > 0.01f && v < 100.0f) ws = v;
        }
    }
    return 100.0f * ws;
}

// THE READBACK FOR THE GEOMETRY KEYS. Every offset here is a UE centimetre (the house unit, and the
// unit the layer's own world_pos takes), but the number that decides whether it looks right in a
// headset is the PHYSICAL metre it becomes after the world scale -- and MCC's field-proven figures
// are physical metres. Printing both, next to theirs, is what stops "I set the width to their
// number and it came out small" from being a mystery.
void log_geometry(const char* why) {
    const float k = cm_per_m();
    logf("%s -- slot %d, width %.1f cm = %.3f m physical (MCC 0.159 m), "
         "fwd %.1f cm = %.3f m (MCC 0.050), right %.1f cm = %.3f m (MCC -0.058), "
         "up %.1f cm = %.3f m (MCC 0.216); roll %s; world scale %.3f (%.1f cm/m)",
         why, g_cfg.scope_layer_slot,
         g_cfg.scope_layer_width, g_cfg.scope_layer_width / k,
         g_cfg.scope_layer_fwd,   g_cfg.scope_layer_fwd   / k,
         g_cfg.scope_layer_right, g_cfg.scope_layer_right / k,
         g_cfg.scope_layer_up,    g_cfg.scope_layer_up    / k,
         g_cfg.scope_layer_roll != 0 ? "rides the gun" : "world-levelled",
         k / 100.0f, k);
}

// PROVE THE RENDER TARGET IS SQUARE -- do not take the creation path's word for it.
//
// Atlas cells are square and a non-square source is REFUSED rather than letterboxed, so this is a
// hard precondition, not a nicety. make_scope_rt() passes scoperes for both dimensions, so it is
// square by construction today -- but "by construction" is the kind of claim that quietly stops
// being true (a live scopesrc change rebuilds the target, and a future non-square capture is an
// obvious thing for someone to try), and the failure would present as a scope that is simply
// absent. Read SizeX/SizeY off the object and compare. Cached per render-target identity, so this
// costs two reflected property reads per rebuild, not per tick.
bool rt_is_square(uevr::API::UObject* rt, int want_dim, char* detail, size_t detail_sz) {
    if (rt != s_rt_checked) {
        s_rt_checked = rt;
        s_rt_square  = false;
        s_rt_x = s_rt_y = 0;
        const auto* sx = rt->get_property_data<int32_t>(L"SizeX");
        const auto* sy = rt->get_property_data<int32_t>(L"SizeY");
        if (sx == nullptr || sy == nullptr) {
            // Unreadable is NOT the same as known-bad. Fall back to the caller's number, which is
            // the edge make_scope_rt() was actually called with, and say that is what happened.
            s_rt_x = s_rt_y = want_dim;
            s_rt_square = (want_dim > 0);
            logf("render target @%p: SizeX/SizeY not reflected on this build -- trusting the "
                 "creation edge %d instead. If the scope pane is absent with no other reason "
                 "logged, this is the assumption to doubt.", (void*)rt, want_dim);
        } else {
            s_rt_x = *sx;
            s_rt_y = *sy;
            s_rt_square = (s_rt_x == s_rt_y) && (s_rt_x > 0);
            logf("render target @%p readback: %dx%d (%s), creation edge %d",
                 (void*)rt, s_rt_x, s_rt_y, s_rt_square ? "square" : "NOT SQUARE", want_dim);
        }
    }
    if (!s_rt_square && detail != nullptr) {
        std::snprintf(detail, detail_sz,
                      "%dx%d -- atlas cells are square and a non-square source is refused, not "
                      "letterboxed", s_rt_x, s_rt_y);
    }
    if (s_rt_square && s_rt_x != want_dim && detail != nullptr) {
        std::snprintf(detail, detail_sz,
                      "readback %dx%d disagrees with the creation edge %d -- the ValueAgreement "
                      "XrSource checks the whole resolved chain against would be wrong",
                      s_rt_x, s_rt_y, want_dim);
        return false;
    }
    return s_rt_square;
}

#if HALO_VR_DEV
// The head-basis arm's source. ASKED FOR, NOT REBUILT -- see the note at the top of this file for
// the two locally-available reconstructions that are both wrong, one because it does not link and
// one because it silently omits the view roll and would reproduce a bug XrLayer already fixed.
bool view_basis(Vec3* fwd, Vec3* up) {
    return scopelayer_adapter::layer_view_basis(fwd, up);
}
#endif

// Build the quad's frame from the aim ray. Returns false with a reason rather than a pose it does
// not believe in.
bool build_quad(const ScopeLayerFeed& f, int mode, ScopeQuad* out, const char** why) {
    Vec3 fwd{f.ray_target.x - f.ray_origin.x,
             f.ray_target.y - f.ray_origin.y,
             f.ray_target.z - f.ray_origin.z};
    if (!vnorm(&fwd)) {
        *why = "the aim ray is degenerate (trace start == end)";
        return false;
    }

    // ---- UP, and with it the weapon's ROLL ----
    //
    // MCC's construction, and the same one Scope.cpp's roll lock already uses: take the lens's own
    // up, remove the component lying along the aim axis, and what remains is the gun's cant
    // measured about the shot line. No Euler decomposition, so no gimbal and no flip near vertical
    // -- both of which were live bugs in this codebase's earlier attempts at the same quantity.
    //
    // The layer orthogonalises up_world itself and does not need it normalised, but we need a
    // proper orthonormal basis anyway to place the quad, so it is done once here and used twice.
    Vec3 up{0.0f, 0.0f, 0.0f};
    bool rolled = false;
    if (g_cfg.scope_layer_roll != 0 && f.lens_up_valid) {
        const float along = vdot(f.lens_up, fwd);
        Vec3 u{f.lens_up.x - fwd.x * along,
               f.lens_up.y - fwd.y * along,
               f.lens_up.z - fwd.z * along};
        // Degenerate only when the lens's own up axis lies along the aim ray. Fall through to the
        // world-levelled frame rather than snapping through the singularity.
        if (vnorm(&u)) { up = u; rolled = true; }
    }

    Vec3 right{0.0f, 0.0f, 0.0f};
    if (rolled) {
        right = vcross(up, fwd);        // UE: up x fwd == right
        (void)vnorm(&right);            // already unit (up _|_ fwd); normalise against drift
    } else {
        right = vcross(Vec3{0.0f, 0.0f, 1.0f}, fwd);
        if (!vnorm(&right)) right = Vec3{0.0f, 1.0f, 0.0f};   // aiming straight up or down
        up = vcross(fwd, right);        // UE: fwd x right == up
        (void)vnorm(&up);
    }

    // FOLLOW THE PANE, or place from our own offsets.
    //
    // The pane is where the calibration lands. Deriving the quad from its PLACED world transform
    // means a recalibration moves both together by construction -- there is no second set of
    // numbers to keep in step, and no way for them to drift apart silently. The scopelayer*
    // offsets remain as the manual override for fitting work, and as the fallback when the pane's
    // location cannot be read.
    const bool follow = (g_cfg.scope_layer_follow_pane != 0) && f.pane_valid;
    if (follow) {
        out->centre   = f.pane_world;
        out->width_cm = f.pane_width_cm;
    } else {
        out->centre = Vec3{
            f.ray_origin.x + fwd.x * g_cfg.scope_layer_fwd + right.x * g_cfg.scope_layer_right +
                up.x * g_cfg.scope_layer_up,
            f.ray_origin.y + fwd.y * g_cfg.scope_layer_fwd + right.y * g_cfg.scope_layer_right +
                up.y * g_cfg.scope_layer_up,
            f.ray_origin.z + fwd.z * g_cfg.scope_layer_fwd + right.z * g_cfg.scope_layer_right +
                up.z * g_cfg.scope_layer_up};
        out->width_cm = g_cfg.scope_layer_width;
    }
    // ---- THE DEFAULT ORIENTATION, WRITTEN BEFORE ANYTHING THAT REFINES IT ----------------------
    //
    // These four USED TO SIT AT THE BOTTOM of this function, below the pane-orientation block and
    // below the roll trim, and they overwrote both. Everything downstream computed a pane-derived
    // basis into out->fwd/out->up and then had it thrown away two lines later, so the quad was
    // ALWAYS the plain aim frame no matter what the pane said.
    //
    // The symptom was a knob that did nothing: scopelayerrolltrim had no effect at any value, and
    // scopelayerroll=2 appeared to "work" only because modes 1 and 2 both take the lens-cant branch
    // for the LOCAL up above -- so what looked like the pane's roll being inherited was really just
    // mode 1. Four consecutive orientation fixes in one session produced no visible change for this
    // reason, and each one was re-theorised as a frame or UV problem.
    //
    // Order is the invariant here: a default is written FIRST and refined after. Writing it last
    // makes every refinement above it dead code that still compiles, still runs, and still logs.
    out->fwd        = fwd;      // the layer's fwd_world IS the aim direction; it owns the sign
    out->up         = up;
    out->rolled     = rolled;
    out->head_basis = false;

    // ORIENTATION FROM THE PANE TOO, when following it. Position from the pane and facing from
    // the ray are two different frames, and their disagreement grows and shrinks as the weapon
    // rotates -- a non-constant offset no placement constant could produce, which is exactly how it
    // was reported. If the quad is following the pane it should BE the pane.
    //
    // WHICH PANE AXIS IS DOWNRANGE IS RESOLVED, NOT ASSUMED. The pane's visible face points back at
    // the shooter, so its forward vector may be either sense. The downrange axis is whichever one
    // agrees with the aim direction, so the sign comes from a dot product rather than a constant
    // somebody has to get right in a headset. If the axis is near-perpendicular to the aim then it
    // is not the facing axis at all -- fall back to the ray and say so, rather than silently
    // turning the pane sideways.
    if (follow && f.pane_fwd_valid) {
        // PICK THE FACING AXIS BY MEASUREMENT. A Plane's normal is its local +Z (its UP vector);
        // a Cylinder's cap points elsewhere again. Testing all three and taking the best-aligned
        // one means this works on any pane shape without a per-shape constant to maintain -- and
        // the alignment value itself is the evidence for whether the answer is trustworthy.
        struct Cand { Vec3 v; const char* name; };
        const Cand cands[3] = {{f.pane_fwd, "forward"}, {f.pane_right, "right"},
                               {f.pane_up_axis, "up"}};
        Vec3 pf{}; float best = 0.0f; const char* best_name = "none";
        for (const Cand& c : cands) {
            Vec3 v = c.v;
            if (!vnorm(&v)) continue;
            const float a = vdot(v, fwd);
            if (std::fabs(a) > std::fabs(best)) { best = a; pf = v; best_name = c.name; }
        }
        {
            const float agree = best;
            // Log the CHOICE once, with the number, so "which axis is the pane facing along" is
            // answerable from the log rather than from a screenshot.
            // REPORT ALL THREE, not just the winner.
            //
            // The winner alone cannot distinguish "this is the normal" from "this was the least bad
            // of three wrong answers". forward won at 0.85 -- about 32 degrees off the aim -- which
            // is either a genuinely tilted pane or a sign the normal is not among these axes at all,
            // and those need opposite fixes. Printing the full set makes that readable instead of
            // inferable, which is the difference between the next step being a measurement and
            // being a third guess.
            //
            // Also prints the quad's own facing so the two can be compared directly: if the pane's
            // chosen axis and the quad's fwd agree but the IMAGE still disagrees, the fault is in
            // the roll or the UV, not in the facing.
            static bool said_axis = false;
            if (!said_axis) {
                said_axis = true;
                Vec3 af = f.pane_fwd, ar = f.pane_right, au = f.pane_up_axis;
                const bool nf = vnorm(&af), nr = vnorm(&ar), nu = vnorm(&au);
                logf("pane axis alignments vs aim -- forward %.3f%s, right %.3f%s, up %.3f%s. "
                     "CHOSE %s (%.3f). A true surface normal reads near +/-1.000; if the best is "
                     "well under that, the pane's normal is NOT one of these three and inheriting "
                     "any of them will leave the quad tilted.",
                     nf ? vdot(af, fwd) : 0.0f, nf ? "" : " (degenerate)",
                     nr ? vdot(ar, fwd) : 0.0f, nr ? "" : " (degenerate)",
                     nu ? vdot(au, fwd) : 0.0f, nu ? "" : " (degenerate)",
                     best_name, agree);
            }
            if (std::fabs(agree) >= 0.5f) {
                if (agree < 0.0f) { pf.x = -pf.x; pf.y = -pf.y; pf.z = -pf.z; }
                out->fwd = pf;
                // ROLL MODE 2: take the pane's OWN up as well, so the quad inherits the pane's
                // FULL orientation rather than its facing only.
                //
                // MEASURED 2026-09-07: forward reads 0.968 against the aim (right 0.083, up
                // -0.239), so the facing axis is unambiguous and correctly inherited -- and the
                // image still disagreed with the pane. That leaves ROLL, and it is ours: modes 0
                // and 1 build `up` from the aim frame, while the CAPTURE bakes its roll
                // compensation (scopecamroll + the per-shape UV roll + the roll lock) sized for the
                // PANE's frame. Showing that content on a differently-rolled quad rotates the image
                // by exactly the difference.
                //
                // Mode 2 makes the quad match the pane, so the baked compensation lands as intended.
                // Upright-in-the-world is then a property of what the CAPTURE bakes, not of the
                // quad -- which is the correct place for it, because that is the term that also
                // decides what the in-world pane shows.
                if (g_cfg.scope_layer_roll == 2) {
                    Vec3 pu = f.pane_up_axis;
                    if (vnorm(&pu)) {
                        // Re-orthogonalise against the facing rather than trusting the pair: the
                        // layer refuses a degenerate basis, and a refusal here would silently fall
                        // back to head-oriented, which looks like the feature having no effect.
                        Vec3 r2 = vcross(pu, pf);
                        if (vnorm(&r2)) {
                            Vec3 u2 = vcross(pf, r2);
                            if (vnorm(&u2)) out->up = u2;
                        }
                    }
                }
                // ROLL STAYS OURS. up is left exactly as the scopelayerroll path built it --
                // world-upright at 0, weapon-canted at 1 -- because a real optic does not spin its
                // image when the rifle cants. Re-orthogonalise against the new facing so the basis
                // stays square; the layer orthogonalises again, but a degenerate pair sent from
                // here would be refused rather than corrected.
                Vec3 r2 = vcross(out->up, pf);
                if (vnorm(&r2)) {
                    Vec3 u2 = vcross(pf, r2);
                    if (vnorm(&u2)) out->up = u2;
                }
            } else {
                static bool said = false;
                if (!said) {
                    said = true;
                    logf("NO pane axis faces the aim -- best was %s at %.2f, and anything under "
                         "0.50 is perpendicular rather than facing. Orientation stays on the aim "
                         "ray. If the quad looks rotated against the pane, this is the line to "
                         "doubt: it means the pane's basis is not what this assumes.",
                         best_name, agree);
                }
            }
        }
    }
    out->follow_pane = follow;
    // ROLL TRIM: rotate the quad about its OWN facing axis, in degrees.
    //
    // The quad and the in-world pane show the SAME render target, but the mesh displays it through
    // its UVs and the quad does not -- and for the round lens those UVs are rotated 90 degrees
    // (which is the entire reason scope_cam_roll is -90 against a uv_roll of +90; the pair cancels
    // for the MESH). A quad has no UV stage to compensate, so a UV-space difference cannot be fixed
    // by inheriting geometry, which is why taking the pane's own up axis made the roll worse rather
    // than better.
    //
    // It is a TRIM rather than a computed constant for the same reason scope_cam_roll is one: it was
    // fitted in a headset, because the number depends on the mesh's UV layout and no amount of
    // reasoning about basis vectors recovers it. Rodrigues about the facing axis -- fwd is already
    // unit and perpendicular to up, so the general form reduces to a plane rotation.
    if (g_cfg.scope_layer_roll_trim != 0.0f) {
        const float a = g_cfg.scope_layer_roll_trim * 3.14159265f / 180.0f;
        const float ca = std::cos(a), sa = std::sin(a);
        Vec3 r = vcross(out->up, out->fwd);      // UE: up x fwd == right
        if (vnorm(&r)) {
            Vec3 u2{out->up.x * ca + r.x * sa,
                    out->up.y * ca + r.y * sa,
                    out->up.z * ca + r.z * sa};
            if (vnorm(&u2)) out->up = u2;
        }
    }

    out->rig_pos       = f.rig_pos;
    out->rig_fwd       = f.rig_fwd;
    out->rig_right     = f.rig_right;
    out->rig_up        = f.rig_up;
    out->rig_rot_valid = f.rig_rot_valid;

    out->ret_u     = f.ret_u;
    out->ret_v     = f.ret_v;
    out->ret_valid = f.ret_valid;

    // WHAT THE QUAD ACTUALLY ENDED UP WITH, next to what the pane wanted. One shot.
    //
    // The bug above was invisible precisely because the pane-axis line already logged a confident,
    // CORRECT-looking measurement ("forward 0.968, CHOSE forward") -- of a value that was then
    // discarded. A log that reports an intermediate proves the intermediate, not the output. This
    // one prints the FINAL basis, so "did the pane's orientation reach the quad" is readable
    // instead of inferable.
    {
        static bool said_final = false;
        // Mode 1 only: the dev self-check arms below deliberately replace the basis after this
        // point, so printing it for them would report a value that is about to be discarded --
        // which is the exact failure this log exists to catch.
        if (!said_final && follow && mode == 1) {
            said_final = true;
            Vec3 pf = f.pane_fwd, pu = f.pane_up_axis;
            const bool nf = vnorm(&pf), nu = vnorm(&pu);
            logf("quad FINAL basis: fwd (%.3f %.3f %.3f) up (%.3f %.3f %.3f); pane fwd "
                 "(%.3f %.3f %.3f) up (%.3f %.3f %.3f); fwd.fwd %.3f up.up %.3f "
                 "(roll=%d trim=%.1f). Near 1.000 on both dots means the quad IS the pane.",
                 out->fwd.x, out->fwd.y, out->fwd.z, out->up.x, out->up.y, out->up.z,
                 pf.x, pf.y, pf.z, pu.x, pu.y, pu.z,
                 nf ? vdot(out->fwd, pf) : 0.0f, nu ? vdot(out->up, pu) : 0.0f,
                 g_cfg.scope_layer_roll, g_cfg.scope_layer_roll_trim);
        }
    }

#if HALO_VR_DEV
    // ---- THE SELF-CHECK ARMS. Position is left exactly where the gun-mounted arm put it; only the
    // ORIENTATION changes, which is the thing under test.
    //
    //   2  feed the layer the VIEW's own forward and up. The converted basis is then IDENTITY, so
    //      the quad must render exactly head-oriented.
    //   3  the CONTROL: publish no orientation at all, which IS head-oriented.
    //
    // 2 and 3 must be indistinguishable. If they are not, our vectors are not in the frame we think
    // they are -- and that failure is total and obvious rather than the near-zero-on-axis error a
    // handedness mistake would produce. That is the whole reason the arm exists.
    if (mode == 2) {
        Vec3 vf{}, vu{};
        if (!view_basis(&vf, &vu)) {
            *why = scopelayer_adapter::kHaveViewBasisApi
                ? "scopelayer=2 self-check: no composed view yet -- xrlayer_view_basis() refused, "
                  "which is correct before the first stereo frame (pre-injection, or the frontend)"
                : "scopelayer=2 self-check: xrlayer_view_basis(Vec3*, Vec3*) is NOT DECLARED in "
                  "XrLayer.hpp yet, so the arm cannot run. Use scopelayer=1; the real feature does "
                  "not depend on it";
            return false;
        }
        out->fwd = vf;
        out->up  = vu;
        out->head_basis = true;
    } else if (mode == 3) {
        out->fwd = Vec3{0.0f, 0.0f, 0.0f};   // the adapter reads a zero fwd as "clear the override"
        out->up  = Vec3{0.0f, 0.0f, 0.0f};
        out->head_basis = true;
    }
#else
    (void)mode;
#endif

    // ---- THE ONE REFUSAL CASE THE LAYER HAS, CHECKED HERE SO IT IS NOT A SILENT DEGRADE --------
    //
    // xrlayer_set_quad_orientation() refuses an up_world parallel to fwd_world (no roll reference)
    // and leaves the slot HEAD-ORIENTED rather than emitting a sheared basis. That refusal is
    // correct and it is also invisible: the pane would quietly stop being gun-mounted with nothing
    // in our log to say why. Both vectors are perpendicular by construction above, so reaching this
    // means something numerically odd happened -- say so rather than shipping it downstream.
    //
    // The control arm's deliberate zero vector is not a degeneracy; skip the test for it.
    if (out->fwd.x != 0.0f || out->fwd.y != 0.0f || out->fwd.z != 0.0f) {
        const float fl = vlen(out->fwd), ul = vlen(out->up);
        if (!(fl > 1e-4f) || !(ul > 1e-4f) ||
            std::fabs(vdot(out->fwd, out->up)) > 0.999f * fl * ul) {
            *why = "fwd and up are parallel or degenerate, so there is no roll reference -- the "
                   "layer would refuse the orientation and leave the quad head-oriented. Not "
                   "publishing a pose this tick";
            return false;
        }
    }
    return true;
}

void retire_now(const char* why) {
    if (!s_published) return;
    scopelayer_adapter::layer_retire(s_slot_out);
    s_published  = false;
    s_presenting = false;
    logf("quad retired from slot %d (%s)", s_slot_out, why);
    s_slot_out = -1;
}

// scopelayer as the code sees it: 0 off, 1 gun-mounted, 2/3 the dev self-check pair. A release
// build has no self-check compiled in, so 2 and 3 fall back to the real feature and say so once --
// the "parse and do nothing" convention halo_vr_dev.cfg documents for every [dev build] key.
int effective_mode() {
    const int m = g_cfg.scope_layer;
#if HALO_VR_DEV
    return m;
#else
    if (m <= 1) return m;
    static bool s_said = false;
    if (!s_said) {
        s_said = true;
        logf("scopelayer=%d is a [dev build] self-check arm and is not compiled into a release "
             "build -- running the ordinary gun-mounted arm instead.", m);
    }
    return 1;
#endif
}

} // namespace

// ==============================================================================================
// Public entry points
// ==============================================================================================

void scopelayer_notice(const ScopeLayerFeed& feed, uint32_t tick) {
    if (!g_cfg.scope_layer) return;
    const int mode = effective_mode();
    if (mode == 0) return;
    s_last_notice = tick;

    if (feed.render_target == nullptr || feed.rt_dim <= 0) {
        say(tick, "the scope has no render target yet (rt=%p dim=%d) -- the capture side has to "
                  "exist before there is anything to present",
            (void*)feed.render_target, feed.rt_dim);
        return;
    }

    char detail[256] = {0};
    if (!rt_is_square(feed.render_target, feed.rt_dim, detail, sizeof(detail))) {
        say(tick, "the scope render target is unusable as a compositor source -- %s", detail);
        return;
    }
    detail[0] = '\0';

    ScopeQuad q;
    const char* why = nullptr;
    if (!build_quad(feed, mode, &q, &why)) {
        // A published quad must not keep a stale weapon rotation while we cannot compute a new one.
        // Head-oriented is wrong for a scope but it is not a lie about where the gun is pointing.
        if (s_published) scopelayer_adapter::layer_clear_orientation(s_slot_out);
        say(tick, "%s", why);
        return;
    }

    // The capture's own axes, for the layer's mode-2 projection. Published every tick the quad is,
    // and BEFORE layer_present, so the basis the offset is built against is never a tick behind the
    // offset itself -- the same ordering rule the rig-relative position needed.
    xrlayer_note_scope_cam_axes(feed.cam_right, feed.cam_up, feed.cam_axes_valid);

    const LayerResult r = scopelayer_adapter::layer_present(
        g_cfg.scope_layer_slot, feed.render_target, feed.rt_dim, q, detail, sizeof(detail));

    if (r.offered) {
        s_published = true;
        s_slot_out  = g_cfg.scope_layer_slot;
    }

    if (r.presenting != s_presenting) {
        s_presenting = r.presenting;
        if (s_presenting) {
            forget_reason();
            // PROVE THE TICK, NOT THE INIT. This line is the only honest evidence that the
            // compositor is drawing the scope; "the call was made" is not "it took effect".
            logf("PRESENTING on slot %d -- cell %dx%d, rt %dx%d, width %.1f cm, orientation %s. "
                 "This is the compositor drawing the scope, not merely an accepted offer.",
                 g_cfg.scope_layer_slot, r.cell_dim, r.cell_dim, feed.rt_dim, feed.rt_dim,
                 q.width_cm,
                 q.head_basis ? (mode == 2 ? "SELF-CHECK (view basis -- must look head-locked)"
                                           : "CONTROL (none -- head-oriented)")
                              : (q.rolled ? "aim ray, rolling with the gun"
                                          : "aim ray, world-levelled"));
        } else {
            logf("STOPPED presenting on slot %d", g_cfg.scope_layer_slot);
        }
    }

    if (!r.presenting) {
        say(tick, "%s%s%s", r.refused != nullptr ? r.refused : "no reason recorded (a bug here)",
            detail[0] != '\0' ? " -- " : "", detail);
    }
}

// ASK FOR THE ATLAS CELL EARLY -- called from update() BEFORE xrlayer_tick(), which is the tick
// that brings the layer up and decides the atlas.
//
// THE RACE THIS EXISTS TO LOSE. The request below used to live only in scopelayer_tick(), which
// runs from scope_frame_end() near the END of update(); xrlayer_tick() runs near the TOP. At the
// main menu update() returns before ever reaching scope_frame_end, so the layer armed with
// g_pane_req still 0 and built an atlas with no pane cell -- permanently, because the atlas is
// deliberately never resized under a live submit thread. By the time gameplay started and the
// scope finally asked, it was 92 ms too late (measured 2026-09-06) and the pane had no cell for
// the whole session. The symptom was "the scope pane and its reticule are simply not there",
// with a log line claiming the request had been accepted.
//
// The size is pure config (scoperes), so nothing here needs a weapon, a pawn or a render target --
// which is exactly why it can run this early and why it should.
void scopelayer_configure_cell_early() {
    if (!g_cfg.scope_layer) return;
    if (s_cfg_cell == g_cfg.scope_rt_size) return;    // already asked for this size
    const int want = g_cfg.scope_rt_size;
    s_cfg_cell = want;
    const bool ok = scopelayer_adapter::layer_configure_cell(want);
    logf("pane cell requested %dpx (from scoperes, EARLY -- before the layer can bring up) -> %s",
         want, ok ? "accepted" : "NOT IN EFFECT NOW (atlas already built; next bring-up)");
}

void scopelayer_tick(uint32_t tick) {
    // ---- the master edge, first: turning it off must retire, not merely stop publishing ---------
    const int mode = g_cfg.scope_layer;
    if (mode != s_cfg_mode) {
        const int was = s_cfg_mode;
        s_cfg_mode = mode;
        if (mode != 0) {
            forget_reason();
            static const char* const kArm[] = {
                "off", "GUN-MOUNTED (the feature)",
                "SELF-CHECK: view basis -- the quad must render exactly head-locked",
                "CONTROL: no orientation override -- head-oriented, the thing arm 2 must match"};
            logf("ON, arm %d = %s. Experimental and default-off; the in-world pane is unchanged "
                 "and still does the work unless scopelayerhidepane=1.",
                 mode, kArm[(mode >= 0 && mode <= 3) ? mode : 1]);
#if HALO_VR_DEV
            if (mode == 2) {
                logf("SELF-CHECK ARMED. It feeds the layer the FINISHED VIEW's own forward and up, "
                     "so the converted basis is identity and the result must be indistinguishable "
                     "from scopelayer=3. Roll included -- the basis is ASKED FOR via "
                     "xrlayer_view_basis(), not rebuilt here, so head tilt is part of the test "
                     "rather than an excuse for a mismatch. xrlayer_view_basis: %s",
                     scopelayer_adapter::kHaveViewBasisApi
                         ? "PRESENT"
                         : "NOT DECLARED YET -- this arm will refuse and say so at the call");
            }
#endif
            log_geometry("geometry");
        } else {
            retire_now("scopelayer=0");
            if (was > 0) {
                // Give the atlas back. It cannot shrink under a live submit thread, so this is a
                // request for the NEXT bring-up -- which is the right answer: an unconfigured build
                // must have a byte-identical atlas.
                const bool ok = scopelayer_adapter::layer_configure_cell(0);
                logf("OFF -- back to the in-world pane alone. Pane cell released: %s",
                     ok ? "yes" : "not now (the layer is up; it takes effect on the next bring-up)");
                s_cfg_cell = 0;
            }
        }
        s_cfg_slot = g_cfg.scope_layer_slot;   // adopt without logging a spurious slot move
    }
    if (!g_cfg.scope_layer) return;

    // ---- THE ATLAS CELL. Must be asked for BEFORE the layer brings up; a late request is only
    // recorded for next time and returns false, which is exactly the thing to say out loud.
    //
    // The request is scoperes -- the config, not the live render target's edge, because the target
    // is created lazily when the scope first opens and the atlas is decided long before that.
    if (s_cfg_cell != g_cfg.scope_rt_size) {
        const int want = g_cfg.scope_rt_size;
        s_cfg_cell = want;
        const bool ok = scopelayer_adapter::layer_configure_cell(want);
        const int got = scopelayer_adapter::layer_cell_dim(g_cfg.scope_layer_slot);
        logf("pane cell requested %dpx (from scoperes) -> %s; slot %d currently has a %dpx cell. "
             "%s", want, ok ? "accepted for this bring-up" : "NOT IN EFFECT NOW",
             g_cfg.scope_layer_slot, got,
             ok ? "" : "The atlas cannot be resized under a live submit thread, so this is "
                       "recorded for the next bring-up -- toggle xrlayer 0 -> 1, or it applies on "
                       "the next session.");
    }

    // ---- the geometry edges, applied by being READ next tick; logged with a readback ------------
    if (s_cfg_fwd   != g_cfg.scope_layer_fwd   || s_cfg_right != g_cfg.scope_layer_right ||
        s_cfg_up    != g_cfg.scope_layer_up    || s_cfg_width != g_cfg.scope_layer_width ||
        s_cfg_roll  != (g_cfg.scope_layer_roll ? 1 : 0)) {
        s_cfg_fwd   = g_cfg.scope_layer_fwd;
        s_cfg_right = g_cfg.scope_layer_right;
        s_cfg_up    = g_cfg.scope_layer_up;
        s_cfg_width = g_cfg.scope_layer_width;
        s_cfg_roll  = g_cfg.scope_layer_roll ? 1 : 0;
        log_geometry("geometry changed");
    }
    if (s_cfg_slot != g_cfg.scope_layer_slot) {
        // A slot move must vacate the old one, or the previous slot keeps a stale pose and a
        // pointer to our render target.
        if (s_published) retire_now("slot changed");
        logf("slot %d -> %d%s", s_cfg_slot, g_cfg.scope_layer_slot,
             g_cfg.scope_layer_slot != XRLAYER_SLOT_PANE
                 ? "  <-- NOT the pane slot. xrlayer_pane_configure() only sizes the pane cell, so "
                   "any other slot has a marker-sized cell and the source will be refused."
                 : "");
        s_cfg_slot = g_cfg.scope_layer_slot;
        forget_reason();
    }

    // ---- retirement -----------------------------------------------------------------------------
    //
    // The same grace the pane uses, and for the same reason: the ray source is itself gated, so it
    // does not fire on literally every tick, and a 1-tick window made the pane flicker. Anything
    // that stops the scope -- menus, seats, death, a weapon switch, the trigger -- stops the
    // notices, so this one test covers all of them by construction.
    // MEASURED IN A LIVE SESSION 2026-08-26: 4 was far too tight and the feature could never
    // have worked. The adapter thrashed offer -> retire -> offer every 0.85-3 s, and because
    // retirement NULLS the source ("XRSRC: slot 9 component changed -> 0000000000000000"), no
    // capture ever landed inside the hold window and the quad never presented once.
    //
    // The old comment -- "~125 ms at ~32 Hz, matching scope_frame_end" -- was an assumption
    // about the tick domain, not a measurement. scopelayer_tick() is called from
    // scope_frame_end() which runs PER FRAME, while scopelayer_notice() comes from
    // scope_apply(); those are not the same cadence, so the tick count and the millisecond
    // figure never described the same thing.
    //
    // It cannot simply be made enormous: this is the ONLY path that retires the quad when the
    // scope closes (the other two call sites are config edges), so an over-long grace leaves a
    // stale quad hanging after the scope shuts. ~1 s is the compromise.
    constexpr uint32_t kGraceTicks = 48;
    const uint32_t gap = (uint32_t)(tick - s_last_notice);

    // INSTRUMENT RATHER THAN GUESS TWICE. Report the largest gap actually observed while
    // published, once. That measurement is what the grace should be tuned against -- without
    // it the next tuning pass is another guess dressed as a constant.
    static uint32_t s_max_gap  = 0;
    static bool     s_gap_said = false;
    if (s_published && gap > s_max_gap) {
        s_max_gap = gap;
        if (!s_gap_said && gap > 4) {
            s_gap_said = true;
            logf("notice gap reached %u ticks while published -- the old grace of 4 would have "
                 "retired the quad here, nulling the source. Grace is now %u. If this number "
                 "ever climbs near the grace, the thrash is back and THIS is the value to "
                 "raise.", gap, kGraceTicks);
        }
    }

    if (s_published && gap > kGraceTicks) {
        retire_now("the scope stopped feeding rays (closed, in a menu, in a seat, or dead)");
        forget_reason();
    }
}

bool scopelayer_presenting() {
    return g_cfg.scope_layer != 0 && s_presenting;
}

} // namespace halo
