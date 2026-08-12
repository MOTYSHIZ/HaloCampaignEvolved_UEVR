#include "Rig.hpp"
#include "Config.hpp"
#include "Reticule.hpp"   // reticle_arm_stray_check: a weapon change rebuilds the HUD crosshair

#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <unordered_map>

using namespace uevr;

namespace halo {
// ================================================================ FP WEAPON RIG
//
// THE TECHNIQUE: the animation graph re-poses the arm SKELETON every frame, but a scene
// component's RELATIVE TRANSFORM sits ABOVE the pose and the anim node never touches it. So we
// write the rig component's relative transform each tick and let the animation keep running
// underneath. Nothing is attached, hooked, or re-parented.
//
// PARAM MARSHALLING -- deliberately conservative. K2_SetRelativeRotation's signature is
//   (FRotator NewRelativeRotation, bool bSweep, FHitResult& SweepHitResult, bool bTeleport)
// and FHitResult is large and version-sensitive. Rather than hand-model it, we pass a generously
// oversized ZEROED buffer with the rotator/vector at offset 0. Every other field's desired value
// IS zero/false, so a zero-filled tail is correct by construction and cannot be mis-sized in a way
// that changes behaviour. FRotator/FVector are DOUBLE here (LWC) -- 24 bytes, not 12.

// Defined further down (needs the object API); declared here because socket lookups above use it.

std::atomic<uint32_t> g_rig_resolve_tick{0};
std::atomic<bool>  g_rig_neutral_valid{false};
std::atomic<float> g_rig_neutral_x{0.0f}, g_rig_neutral_y{0.0f}, g_rig_neutral_z{0.0f};
std::atomic<int>   g_rig_loc_works{-1};   // -1 unknown, 0 no-op detected, 1 confirmed applying

// How far the controller POSITION has actually travelled from its neutral, in metres. This is the
// diagnostic that separates the two possible causes of "no translation on the arms":
//   travel ~ 0      -> the runtime is handing us ORIENTATION-ONLY poses; translation is not
//                      reachable from here at all and no amount of rig work will fix it.
//   travel > 0      -> position IS live and the fault is in our translation path.
// Without this the two are indistinguishable, and they need completely different fixes.
std::atomic<float> g_ctrl_travel_max{0.0f};
std::atomic<float> g_dbg_rig_x{0.0f}, g_dbg_rig_y{0.0f}, g_dbg_rig_z{0.0f}, g_dbg_rig_roll{0.0f};
std::atomic<bool>  g_rig_wrote_once{false};
std::atomic<float> g_rig_survive_drift{0.0f};   // how far our write had moved by the NEXT tick
// Raw controller position, logged to expose pose jumps: 1.8 m of "travel" is a teleport, not a hand.
std::atomic<float> g_dbg_pos_x{0.0f}, g_dbg_pos_y{0.0f}, g_dbg_pos_z{0.0f};

// ---------------------------------------------------------------- rig resolution
// The target is BPC_FP_SkeletalMesh_C and nothing else. The WEAPON is a separate actor attached
// to this component at socket `PrimaryWeapon`,
//
//     BPC_FP_SkeletalMesh_C                       <- we drive this
//       +-- BPC_FP_StaticMesh_C x6                   armour: Elbow/Shoulder/Wrist L+R
//       +-- BP_FP_Magnum_WeaponActor_C  sock=PrimaryWeapon   <- the gun rides along
//
// so moving the rig moves the gun, and hiding the arms leaves the weapon visible.
// (Recorded here because it is easy to mistake the six FP static meshes for weapon parts and
// "helpfully" drive them -- they are shoulder pads.)
//
// !!! RESOLUTION IS BY ATTACHMENT, NOT BY CLASS-SCAN ORDER: actors on this title are pooled and
// recycled, so a class match can be residue from a previous life -- and writes to such a corpse
// land, persist a full frame, read back verbatim, and still draw nothing. Finding the rig
// THROUGH a live weapon actor makes binding to a corpse structurally impossible.
std::atomic<void*> g_rig_component{nullptr};

// The rig's attach parent. Its live world rotation is what a RELATIVE transform is measured
// against, so it is read rather than inferred -- inferring it from ControlRotation was only ever
// an approximation, and any error in it shows up as the weapon leaving the pivot under rotation.
API::UObject* g_rig_parent = nullptr;


// Follow an object-typed property one hop. Returns nullptr rather than faulting on anything odd --
// UEVR's reflection is known to fault on Blam-backed objects on this title, so every hop is guarded.
API::UObject* follow_object(API::UObject* obj, const wchar_t* prop) {
    if (obj == nullptr) return nullptr;
    auto** slot = obj->get_property_data<API::UObject*>(prop);
    if (slot == nullptr) return nullptr;
    if (IsBadReadPtr(slot, sizeof(void*))) return nullptr;
    auto* v = *slot;
    if (v == nullptr || IsBadReadPtr(v, sizeof(void*))) return nullptr;
    return v;
}


// Find the live FP rig by walking BACK from a first-person weapon actor:
//     weapon actor -> RootComponent -> AttachParent == BPC_FP_SkeletalMesh_C
// The weapon actor the rig was last reached through. Caching the WEAPON (not the rig) is what
// makes skipping the sweep safe: the rig is still re-derived by walking attachment from a live
// weapon actor on every call, so the "never adopt by class match" invariant above is untouched.
// A TrackedObject, not a raw pointer, so a recycled array slot is detected rather than followed.
static TrackedObject g_fp_weapon;

// weapon actor -> RootComponent -> AttachParent, accepted only if the parent really is the rig.
// nullptr means "this weapon actor no longer leads to a rig" -- the caller's cue to sweep.
static API::UObject* rig_through_weapon(API::UObject* wpn) {
    auto* root = follow_object(wpn, L"RootComponent");
    auto* par  = follow_object(root, L"AttachParent");
    if (par == nullptr) return nullptr;
    if (class_name_of(par).find(L"BPC_FP_SkeletalMesh_C") == std::wstring::npos) return nullptr;
    return par;
}

// Tracked mirror of the resolved rig component, for rig_component_alive() below. Refreshed only
// when the resolved pointer CHANGES: TrackedObject::set() is an O(n) object-array scan, fine on a
// rare acquisition and exactly the periodic cost resolve_rig's fast path exists to avoid.
static TrackedObject g_rig_track;

static void note_resolved_rig(API::UObject* rig) {
    if (rig != nullptr && g_rig_track.ptr != rig) g_rig_track.set(rig);
}

// Stick mode's core signal (see Rig.hpp). Read-only walk; a live g_fp_weapon is left untouched.
bool fp_weapon_route_alive() {
    auto* w = g_fp_weapon.get();
    if (w == nullptr) return false;
    const std::wstring cn = class_name_of(w);
    if (cn.find(L"_FP_")       == std::wstring::npos ||
        cn.find(L"WeaponActor") == std::wstring::npos) return false;
    return rig_through_weapon(w) != nullptr;
}

// Diagnostic for the stick-mode transition logs: does the rig COMPONENT still occupy its object-
// array slot? A weapon swap kills the route but not this; what a vehicle seat does to it is recon
// R1, answered by this appearing in one log line.
bool rig_component_alive() {
    return g_rig_track.get_checked(L"BPC_FP_SkeletalMesh_C") != nullptr;
}

// The first-person weapon ACTOR the rig was last reached through. Exposed for the reticule trace:
// the gun is its own actor attached to the rig, so it is not covered by ignoring the pawn, and a
// trace from the eye hits it whenever an animation swings it across the camera -- a reload puts the
// reticule in your face. Tracked, so a recycled array slot reads as null rather than as a corpse.
API::UObject* fp_weapon_actor() {
    // get_CHECKED, not get(). get() only proves the array slot still holds this pointer, and actors
    // on this title are POOLED: a slot can be reused in place, so the same pointer can be a
    // different object with a different life. The class check is what makes a swapped or recycled
    // weapon read as "gone" instead of as a live one, and handing a corpse to the trace's ignore
    // list is not a thing worth finding out about from a crash dump.
    //
    // Every FP weapon class on this title is <something>_WeaponActor_C, so the substring covers the
    // whole family rather than naming one gun.
    return g_fp_weapon.get_checked(L"WeaponActor");
}

API::UObject* rig_tracked_component() {
    return g_rig_track.get_checked(L"BPC_FP_SkeletalMesh_C");
}

API::UObject* resolve_rig() {
    // ---- FAST PATH. The sweep below is a full object-array walk with a class-name string built
    // per object, and it ran unconditionally every ~2 s even with a perfectly good rig already
    // resolved -- one of the periodic-microstutter suspects. Re-deriving through the cached weapon
    // actor is O(1) and reaches the SAME rig by the SAME attachment walk.
    //
    // Safe against a weapon swap, which creates a new actor: if the old one is gone, recycled, or
    // no longer attached to the rig, every check below fails and we fall through to the sweep.
    // Safe against a stale-but-attached one too -- the rig is the PAWN's component and does not
    // change when the gun does, so the derived answer is still correct. Pawn changes (level load,
    // respawn) clear g_rig_component and re-resolve from scratch on the next tick.
    if (g_cfg.rig_fast) {
        if (auto* w = g_fp_weapon.get()) {
            const std::wstring cn = class_name_of(w);
            if (cn.find(L"_FP_")       != std::wstring::npos &&
                cn.find(L"WeaponActor") != std::wstring::npos) {
                if (auto* rig = rig_through_weapon(w)) { note_resolved_rig(rig); return rig; }
            }
        }
        g_fp_weapon.reset();   // handle is no good; the sweep re-establishes it
    }

    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return nullptr;

    // The fast path above is O(1) and covers steady play. This is the fallback, and during a LEVEL
    // LOAD it is what runs: there is no weapon actor to derive from yet, so every attempt falls
    // through here. Measured at 65-78 ms a sweep, 12-14 sweeps per 600-tick window -- 406 ms and
    // 443 ms of game-thread stall in two consecutive windows. After memoising: 21.4 ms.
    std::unordered_map<const void*, std::wstring> name_of_class;

    const int32_t n = arr->get_object_count();
    for (int32_t i = 0; i < n; ++i) {
        auto* obj = arr->get_object(i);
        if (obj == nullptr) continue;

        // Candidate = a first-person weapon actor, e.g. BP_FP_Magnum_WeaponActor_C.
        // MEMOISED ON THE CLASS. This walks the entire UObject array building a class-name string
        // per object, and objects outnumber classes by orders of magnitude -- the same few names
        // were rebuilt tens of thousands of times per sweep.
        auto* ocls = obj->get_class();
        if (ocls == nullptr) continue;
        auto memo = name_of_class.find(ocls);
        if (memo == name_of_class.end()) memo = name_of_class.emplace(ocls, class_name_of(obj)).first;
        const std::wstring& cn = memo->second;
        if (cn.find(L"_FP_") == std::wstring::npos) continue;
        if (cn.find(L"WeaponActor") == std::wstring::npos) continue;

        auto* cls = ocls;
        if (cls != nullptr && obj == cls->get_class_default_object()) continue;

        if (auto* par = rig_through_weapon(obj)) {
            // A DIFFERENT weapon actor means the HUD has been rebuilt around it -- a swap, a
            // respawn, a pickup. That rebuild makes the game a FRESH flat crosshair while we are
            // still hosting the old widget, so both end up on screen. Nothing else re-checks for
            // that: the hosting window is armed once when a widget is taken and has long expired
            // by the time a weapon changes.
            //
            // Arming here rather than polling keeps the object-array sweep event-driven. It costs
            // one extra sweep per weapon change, not a standing 100-125 ms poll.
            if (g_fp_weapon.get() != obj) reticle_arm_stray_check();

            // Remember the ROUTE, with its array slot, so the next call can skip this sweep.
            g_fp_weapon.set_at(obj, i);
            note_resolved_rig(par);
            return par;
        }
    }
    return nullptr;
}

// ---------------------------------------------------------------- FP SHIELD SHELL
// The first-person shield/overshield visual is NOT a material on the arms and NOT a mesh of its
// own invention: it is BPC_FP_TranslucentSkeletalMesh_C, a component of the pawn that carries the
// translucent energy skin (the pawn's FindTranslucentMeshes gathers these by tag, and
// ShieldMaterialIndex + the base<->masked material maps are what light them up).
//
// It is a SIBLING of the arms rig, not a child, and it runs its OWN instance of the same
// first-person anim blueprint (ABP_SpartansFP_NEW_C). So it is posed identically to the arms while
// being transformed independently -- which is exactly why writing the arms' relative transform
// leaves it behind, and why the shield appears to hang in space once the arms follow the hand.
// Giving it the IDENTICAL transform re-marries the two.
//
// !!! MATCH THE EXACT CLASS NAME. NEVER A "BPC_FP_" PREFIX. The pawn also carries
// BPC_FP_ShadowSkeletalMesh_C, which despite the matching prefix is a FULL-BODY shadow-casting
// proxy standing on the ground -- its anim blueprint is ABP_Spartans_Common_C, the THIRD-PERSON
// graph, and that is the tell. Driving that one from the hand pose lifts the player's entire body
// shadow off the floor and spins it. The anim BP is the discriminator:
//     ABP_SpartansFP_NEW_C   -> arms-shaped, co-located with the arms, safe to drive
//     ABP_Spartans_Common_C  -> full body, belongs at the feet, must be left alone
static constexpr const wchar_t* kShellClass = L"BPC_FP_TranslucentSkeletalMesh_C";

static TrackedObject g_shell_track;

// A UE TArray header -- same shape the stick-mode dismount watcher reads for AttachChildren.
struct FRawArrayRO { void* data; int32_t num; int32_t max; };

// Scan one TArray<UObject*> property for the shell. Fails closed on anything unreadable rather than
// faulting: every hop here can be mid-teardown on this title, and a bogus component is far worse
// than no component.
static API::UObject* find_shell_in_array(API::UObject* owner, const wchar_t* prop) {
    if (owner == nullptr) return nullptr;
    auto* arr = owner->get_property_data<FRawArrayRO>(prop);
    if (arr == nullptr || IsBadReadPtr(arr, sizeof(FRawArrayRO))) return nullptr;
    if (arr->data == nullptr || arr->num <= 0 || arr->num > 4096) return nullptr;

    auto** elems = reinterpret_cast<API::UObject**>(arr->data);
    if (IsBadReadPtr(elems, sizeof(void*) * (size_t)arr->num)) return nullptr;

    for (int32_t i = 0; i < arr->num; ++i) {
        auto* c = elems[i];
        if (c == nullptr || IsBadReadPtr(c, sizeof(void*))) continue;
        if (class_name_of(c) == kShellClass) return c;   // EXACT, see the prefix warning above
    }
    return nullptr;
}

API::UObject* resolve_shield_shell(API::UObject* rig_parent) {
    // get_local_pawn is the cheap per-frame handle -- no object-array walk. A full sweep here is
    // exactly the pattern that has already collapsed framerate in a live session once.
    auto* pawn = API::get()->get_local_pawn(0);

    // BlueprintCreatedComponents lists the construction-script components flat, independent of how
    // they are attached, so it is preferred over walking attachment topology we have not measured.
    if (auto* s = find_shell_in_array(pawn, L"BlueprintCreatedComponents")) return s;
    if (auto* s = find_shell_in_array(pawn, L"InstanceComponents"))         return s;
    // Fallback: as a sibling of the arms, the shell hangs off the arms' own attach parent.
    if (auto* s = find_shell_in_array(rig_parent, L"AttachChildren"))       return s;
    return nullptr;
}

// Published for the render-rate re-apply. Kept in lockstep with g_shell_track below so the render
// path can never outlive the validated handle: every place that invalidates one clears the other.
std::atomic<void*> g_shell_component{nullptr};

API::UObject* shield_shell() {
    auto* s = g_shell_track.get_checked(kShellClass);
    // A recycled slot must stop the RENDER path too, not just this one -- otherwise the stereo
    // callback keeps writing into whatever now occupies the address.
    if (s == nullptr) g_shell_component.store(nullptr);
    return s;
}

void note_resolved_shell(API::UObject* shell) {
    if (shell != nullptr && g_shell_track.ptr != shell) {
        g_shell_track.set(shell);
        // set() refuses a pointer it cannot find in the object array, so publish what it ACTUALLY
        // adopted rather than the argument -- otherwise a rejected pointer would still reach the
        // render thread.
        g_shell_component.store(g_shell_track.ptr);
    }
}

void forget_shield_shell() {
    g_shell_track.reset();
    g_shell_component.store(nullptr);
}

// Write the rig's relative rotation. Returns false if the call could not be made at all.
bool rig_set_rotation(API::UObject* rig, double pitch, double yaw, double roll) {
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    auto* rot = reinterpret_cast<double*>(params);
    rot[0] = pitch; rot[1] = yaw; rot[2] = roll;
    rig->call_function(L"K2_SetRelativeRotation", params);
    return true;
}

// K2_SetRelativeScale3D takes ONLY an FVector -- no sweep/hit/teleport tail, unlike its
// location/rotation siblings. The oversized zeroed buffer is harmless either way.
bool rig_set_scale(API::UObject* rig, double s) {
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    auto* v = reinterpret_cast<double*>(params);
    v[0] = s; v[1] = s; v[2] = s;
    rig->call_function(L"K2_SetRelativeScale3D", params);
    return true;
}

bool rig_set_location(API::UObject* rig, double x, double y, double z) {
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    auto* v = reinterpret_cast<double*>(params);
    v[0] = x; v[1] = y; v[2] = z;
    rig->call_function(L"K2_SetRelativeLocation", params);
    return true;
}

// WORLD-space writes. Same call convention as their relative siblings, different target frame.
//
// These exist because the RELATIVE rotation write does not survive on this game's first-person
// mesh: the RelativeRotation FIELD accepts the value and reads back exactly, yet the component's
// world rotation, read through K2_GetComponentRotation, does not move at all -- measured across a
// 75 degree hand sweep it stayed inside 1.4 degrees. Location behaves the opposite way and does
// follow. That asymmetry is precisely the reported symptom, the arms translating while barely
// rotating, and no amount of getting the parent composition right can fix a write the engine
// recomputes afterwards. Writing the world transform removes the parent from the question.
bool rig_set_world_rotation(API::UObject* rig, double pitch, double yaw, double roll) {
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    auto* rot = reinterpret_cast<double*>(params);
    rot[0] = pitch; rot[1] = yaw; rot[2] = roll;
    rig->call_function(L"K2_SetWorldRotation", params);
    return true;
}

bool rig_set_world_location(API::UObject* rig, double x, double y, double z) {
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    auto* v = reinterpret_cast<double*>(params);
    v[0] = x; v[1] = y; v[2] = z;
    rig->call_function(L"K2_SetWorldLocation", params);
    return true;
}

// ---------------------------------------------------------------- UOBJECTHOOK ATTACHMENT
// Hands the arms component to UEVR's own motion-controller attachment instead of driving its
// relative transform ourselves. UEVR then applies it per-eye on the render path, exactly as it does
// for every other mod -- which is the consistency argument for switching.
//
// THE MAP-LOAD RACE IS INHERITED. UObjectHook applies attachments from
// on_pre_calculate_stereo_view_offset (UObjectHook.cpp:1901) with no revalidation before the
// UFunction call, while a level transition destroys actors on the game thread.
//
// So we DETACH on PlayerController change (our existing level-transition signal) and re-attach once
// the new rig resolves. The race only exists for attachments that live across the transition, so
// not being attached at that moment removes it by construction rather than by guard.
bool g_attached = false;

// Desired WORLD rotation of the rig, published by the tick and consumed by the stereo callback so
// the relative rotation can be recomputed against the live parent every render frame.
std::atomic<float> g_rigw_x{0.0f}, g_rigw_y{0.0f}, g_rigw_z{0.0f}, g_rigw_w{1.0f};
std::atomic<bool>  g_rigw_valid{false};
std::atomic<float> g_rigw_off_x{0.0f}, g_rigw_off_y{0.0f}, g_rigw_off_z{0.0f};
std::atomic<bool>  g_rigw_off_valid{false};
std::atomic<float> g_rigw_parent_yaw{0.0f};

void attach_apply(API::UObject* rig, const Quat& rot_off, const Vec3& loc_off_cm) {
    if (rig == nullptr) return;
    auto* st = API::UObjectHook::get_or_add_motion_controller_state(rig);
    if (st == nullptr) return;

    UEVR_Quaternionf q{rot_off.x, rot_off.y, rot_off.z, rot_off.w};
    UEVR_Vector3f    v{loc_off_cm.x, loc_off_cm.y, loc_off_cm.z};
    st->set_rotation_offset(&q);
    st->set_location_offset(&v);
    st->set_hand(1);   // MotionControllerStateBase::Hand::RIGHT
    st->set_permanent(g_cfg.attach_permanent);
    g_attached = true;
}

void attach_release(API::UObject* rig, const char* why) {
    if (!g_attached) return;
    if (rig != nullptr) API::UObjectHook::remove_motion_controller_state(rig);
    g_attached = false;
    API::get()->log_info("[Halo-CampE-UEVR] UObjectHook attachment released (%s)", why);
}

// ---------------------------------------------------------------- THE PIVOT
// K2_SetRelativeRotation spins a component about ITS OWN ORIGIN, and this component's origin is the
// arm root -- not the hand. So however well the weapon is positioned, rolling the wrist swings it
// in an arc about the shoulder end. Calibration cannot fix this: it can move where the mesh SITS,
// but the point it turns about is a property of the transform, not of the offset.
//
// The fix is to rotate about a chosen local point G ("the grip"):
//     origin = target - R * G
// so the mesh point at local coordinate G stays put while everything else swings around it.
//
// G is READ FROM THE GAME rather than dialled in: the weapon actor is attached to this component at
// socket `PrimaryWeapon`, so that socket IS the grip point, and
//     G = inverse(R_component) * (socket_world - component_world)
// gives it in component-local space. One fewer thing to tune, and it is correct per weapon.

// Call a no-argument UFunction whose return value is an FVector/FRotator (3 doubles at offset 0).
bool call_ret_vec3(API::UObject* obj, const wchar_t* fn, Vec3* out) {
    if (obj == nullptr || out == nullptr) return false;
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    obj->call_function(fn, params);
    auto* d = reinterpret_cast<double*>(params);
    if (!std::isfinite(d[0]) || !std::isfinite(d[1]) || !std::isfinite(d[2])) return false;
    *out = Vec3{(float)d[0], (float)d[1], (float)d[2]};
    return true;
}

// GetSocketLocation(FName) -> FVector. FName is {int32,int32} = 8 bytes, so the FVector return
// lands at offset 8 and is 8-byte aligned there.
bool call_socket_location(API::UObject* comp, const wchar_t* socket, Vec3* out) {
    if (comp == nullptr || out == nullptr) return false;
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    // make_fname, NOT API::FName: the latter resolves to None in this plugin (see make_fname), and
    // GetSocketLocation(None) quietly returns the COMPONENT'S OWN location -- every socket lookup
    // would silently succeed with the wrong answer.
    API::FName name = make_fname(socket);
    memcpy(params, &name, sizeof(int32_t) * 2);
    comp->call_function(L"GetSocketLocation", params);
    auto* d = reinterpret_cast<double*>(params + 8);
    if (!std::isfinite(d[0]) || !std::isfinite(d[1]) || !std::isfinite(d[2])) return false;
    *out = Vec3{(float)d[0], (float)d[1], (float)d[2]};
    return true;
}

// ---------------------------------------------------------------- debug sphere
// Draws a marker at a WORLD position via UKismetSystemLibrary::DrawDebugSphere, so the pivot can be
// SEEN without borrowing the arms or the weapon -- both of which are the reference the pivot is
// being judged against, and so cannot also be the instrument.
//
// Param frame (x64, LWC doubles), laid out by hand because there is no reflection helper for this:
//   0  UObject* WorldContextObject
//   8  FVector  Center      (3 x double, 8-aligned)
//   32 float    Radius
//   36 int32    Segments
//   40 FLinearColor LineColor (4 x float)
//   56 float    Duration
//   60 float    Thickness
//
// MAY BE A NO-OP. UE compiles DrawDebug* out of shipping builds, and this is a shipping build.
// If nothing appears, that is the likely reason rather than a bad frame layout -- fall back to
// driving a real component. Cheap enough to be worth trying first.
// duration/segments/thickness are parameters rather than constants because the reticule needs very
// different values from the pivot diagnostic. In particular DURATION MATTERS: this is called from
// on_pre_engine_tick at ~32 Hz while the headset renders at 90+, so a duration of 0 ("this frame
// only") would leave the sphere absent on two render frames out of three and strobe.
void draw_debug_sphere(API::UObject* world_ctx, const Vec3& c, float radius,
                       float r, float g, float b,
                       float duration, int32_t segments, float thickness) {   // defaults in Rig.hpp
    if (world_ctx == nullptr) return;

    static API::UObject* kis = nullptr;
    static bool tried = false;
    if (!tried) {
        tried = true;
        auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.KismetSystemLibrary");
        if (cls != nullptr) kis = cls->get_class_default_object();
        API::get()->log_info("[Halo-CampE-UEVR] KismetSystemLibrary CDO %s", kis ? "found" : "NOT FOUND");
    }
    if (kis == nullptr) return;

    alignas(16) uint8_t params[128] = {0};
    *reinterpret_cast<void**>(params + 0) = world_ctx;
    auto* ctr = reinterpret_cast<double*>(params + 8);
    ctr[0] = c.x; ctr[1] = c.y; ctr[2] = c.z;
    *reinterpret_cast<float*>(params + 32)   = radius;
    *reinterpret_cast<int32_t*>(params + 36) = segments;
    auto* col = reinterpret_cast<float*>(params + 40);
    col[0] = r; col[1] = g; col[2] = b; col[3] = 1.0f;
    *reinterpret_cast<float*>(params + 56) = duration;
    *reinterpret_cast<float*>(params + 60) = thickness;
    kis->call_function(L"DrawDebugSphere", params);
}

// ---------------------------------------------------------------- the cube marker
// DrawDebugSphere draws NOTHING here (UE strips debug rendering from shipping builds), so the
// marker has to be a real, already-rendered object -- and it must not be the arms or the weapon,
// because those are the reference the pivot is being judged against.
//
// A level StaticMeshActor is borrowed instead: shrunk, made Movable, and parked on the pivot every
// tick. Its original location is remembered and restored when the marker is switched off.
// Two of these: one marks the pivot, one is the VR reticule on the aim ray. They must not
// borrow the same actor, hence the exclude parameter on resolution.

BorrowedMarker g_pivot_marker;
BorrowedMarker g_aim_marker;

bool actor_set_location(API::UObject* actor, const Vec3& p) {
    if (actor == nullptr) return false;
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    auto* v = reinterpret_cast<double*>(params);
    v[0] = p.x; v[1] = p.y; v[2] = p.z;
    actor->call_function(L"K2_SetActorLocation", params);
    return true;
}

// Level geometry defaults to Static mobility, and a Static actor silently ignores SetActorLocation.
// Without this the cube would simply never move and look like a broken marker.
void actor_set_movable(API::UObject* actor) {
    if (actor == nullptr) return;
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    params[0] = 2;   // EComponentMobility::Movable
    actor->call_function(L"SetMobility", params);
}

// AActor::GetActorBounds(bool bOnlyCollidingComponents, FVector& Origin, FVector& BoxExtent,
//                        bool bIncludeFromChildActors)
// Origin and BoxExtent are OUT params written back into the param frame:
//   0 bool | 8 FVector Origin | 32 FVector BoxExtent | 56 bool
// Needed for two reasons: the borrowed prop is whatever size the level author made it, and its
// pivot is wherever the artist put it -- neither of which is any use for a marker until measured.
bool actor_get_bounds(API::UObject* actor, Vec3* origin, Vec3* extent) {
    if (actor == nullptr) return false;
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    params[0] = 0;   // bOnlyCollidingComponents = false
    actor->call_function(L"GetActorBounds", params);

    auto* o = reinterpret_cast<double*>(params + 8);
    auto* e = reinterpret_cast<double*>(params + 32);
    if (!std::isfinite(o[0]) || !std::isfinite(e[0])) return false;
    *origin = Vec3{(float)o[0], (float)o[1], (float)o[2]};
    *extent = Vec3{(float)e[0], (float)e[1], (float)e[2]};
    return true;
}

void actor_set_bool(API::UObject* actor, const wchar_t* fn, bool value) {
    if (actor == nullptr) return;
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    params[0] = value ? 1 : 0;
    actor->call_function(fn, params);
}

// Pick the NEAREST PLACED StaticMeshActor to `ref`, not merely the first in the object array:
// a first match can be an unplaced template sitting at world (0,0,0) with no streamed mesh,
// which renders nothing at all. Two filters fix that: reject the origin, and prefer the closest
// actor to the player, which is necessarily in a streamed-in, visible part of the level.
bool resolve_marker(BorrowedMarker& m, float scale, const Vec3& ref,
                    const char* label, API::UObject* exclude) {
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return false;

    API::UObject* best = nullptr;
    Vec3  best_home{};
    float best_d2 = 3.4e38f;
    int   considered = 0;

    const int32_t n = arr->get_object_count();
    for (int32_t i = 0; i < n; ++i) {
        auto* obj = arr->get_object(i);
        if (obj == nullptr) continue;
        if (class_name_of(obj) != L"StaticMeshActor") continue;

        auto* cls = obj->get_class();
        if (cls != nullptr && obj == cls->get_class_default_object()) continue;
        if (obj == exclude) continue;   // already borrowed by the other marker

        Vec3 home{};
        if (!call_ret_vec3(obj, L"K2_GetActorLocation", &home)) continue;

        // Unplaced template / not streamed in.
        if (std::fabs(home.x) < 1.0f && std::fabs(home.y) < 1.0f && std::fabs(home.z) < 1.0f) continue;
        if (follow_object(obj, L"RootComponent") == nullptr) continue;

        ++considered;
        const float dx = home.x - ref.x, dy = home.y - ref.y, dz = home.z - ref.z;
        const float d2 = dx * dx + dy * dy + dz * dz;
        if (d2 < best_d2) { best_d2 = d2; best = obj; best_home = home; }
    }

    if (best == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] %s: no PLACED StaticMeshActor found (%d considered)", label, considered);
        return false;
    }

    m.actor = best;
    m.home  = best_home;
    m.root  = follow_object(best, L"RootComponent");

    actor_set_movable(best);
    actor_set_bool(best, L"SetActorHiddenInGame", false);
    // A shrunken prop teleported onto the player would otherwise shove them around.
    actor_set_bool(best, L"SetActorEnableCollision", false);

    // ---- AUTO-SIZE. `scale` here is a TARGET SIZE IN CM, not a scale factor: a fixed factor is
    // useless because the borrowed prop might be a pebble or a hangar door. Measure it at scale 1,
    // then scale so its largest dimension is the size asked for.
    float applied = 1.0f;
    Vec3 o1{}, e1{};
    if (actor_get_bounds(best, &o1, &e1)) {
        const float half = std::fmax(std::fmax(std::fabs(e1.x), std::fabs(e1.y)), std::fabs(e1.z));
        if (half > 0.01f) applied = clampf((scale * 0.5f) / half, 0.0005f, 1.0f);
    }

    // Scale the ACTOR, not the root component: a component on a level actor does not necessarily
    // accept a relative-scale write, which leaves the prop full size.
    {
        alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
        auto* v = reinterpret_cast<double*>(params);
        v[0] = v[1] = v[2] = (double)applied;
        best->call_function(L"SetActorScale3D", params);
    }

    // ---- AUTO-CENTRE, computed rather than re-measured. Scaling happens about the actor origin, so
    // the origin-to-centre offset scales with it. Re-reading bounds immediately returns stale
    // values (the transform has not propagated yet), which silently produces a marker sitting
    // metres from where it claims to be.
    m.center_off = Vec3{(o1.x - best_home.x) * applied,
                        (o1.y - best_home.y) * applied,
                        (o1.z - best_home.z) * applied};

    const std::string nm = (best->get_fname() != nullptr) ? narrow(best->get_fname()->to_string()) : "?";
    API::get()->log_info("[Halo-CampE-UEVR] %s = %s  %.1f m away  (%d candidates)",
                         label, nm.c_str(), std::sqrt(best_d2) / 100.0f, considered);
    API::get()->log_info("[Halo-CampE-UEVR]   half-extent %.0f cm -> scale %.4f for a %.0f cm marker; centre offset (%.1f, %.1f, %.1f)",
                         std::fmax(std::fmax(std::fabs(e1.x), std::fabs(e1.y)), std::fabs(e1.z)),
                         applied, scale, m.center_off.x, m.center_off.y, m.center_off.z);
    return true;
}

// A borrowed actor belongs to the LEVEL, so a load or stream-out destroys it while we still hold
// the pointer -- the same pooled-and-recycled hazard that governs everything else on this title.
// Calling a UFunction on that corpse is an access violation, so liveness is re-checked before every
// use rather than assumed.
bool marker_alive(BorrowedMarker& m) {
    if (m.actor == nullptr) return false;
    if (IsBadReadPtr(m.actor, sizeof(void*))) { m.actor = nullptr; m.root = nullptr; m.active = false; return false; }
    if (class_name_of(m.actor) != L"StaticMeshActor") {
        // Recycled into something else: forget it rather than write into whatever now lives here.
        m.actor = nullptr; m.root = nullptr; m.active = false;
        API::get()->log_info("[Halo-CampE-UEVR] borrowed marker went stale -- dropped, will re-acquire");
        return false;
    }
    return true;
}

// Park a marker so its VISUAL CENTRE lands on `p`.
void park_marker(BorrowedMarker& m, const Vec3& p) {
    if (!marker_alive(m)) return;
    actor_set_location(m.actor, Vec3{p.x - m.center_off.x, p.y - m.center_off.y, p.z - m.center_off.z});
}

void release_marker(BorrowedMarker& m, const char* label) {
    if (marker_alive(m)) {
        if (m.root != nullptr) rig_set_scale(m.root, 1.0);
        actor_set_location(m.actor, m.home);
        API::get()->log_info("[Halo-CampE-UEVR] %s released (returned home)", label);
    }
    m.actor = nullptr;
    m.root  = nullptr;
    m.active = false;
}

// Returns false and leaves *out untouched if anything looks wrong -- a bogus pivot is far more
// destructive than no pivot, so this fails closed. UE returns the COMPONENT location when a socket
// name does not resolve, which shows up here as a ~0 length delta and is treated as "not found".
bool derive_pivot(API::UObject* rig, const wchar_t* socket, Vec3* out) {
    Vec3 comp{}, rot{}, sock{};
    if (!call_ret_vec3(rig, L"K2_GetComponentLocation", &comp)) return false;
    if (!call_ret_vec3(rig, L"K2_GetComponentRotation", &rot)) return false;   // pitch, yaw, roll
    if (!call_socket_location(rig, socket, &sock)) return false;

    const Vec3 d{sock.x - comp.x, sock.y - comp.y, sock.z - comp.z};
    const float len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    if (!std::isfinite(len) || len < 0.5f || len > 300.0f) return false;

    const Quat q = rotator_to_quat(rot.x, rot.y, rot.z);
    *out = quat_rotate(quat_conj(q), d);
    return true;
}

// Log every plausible pivot socket so the choice is made from measurements rather than a guess.
// These names are the sockets present on this rig.
void log_pivot_candidates(API::UObject* rig) {
    static const wchar_t* const kCandidates[] = {
        L"PrimaryWeapon", L"Wrist_R", L"Wrist_L", L"ElbowPart2_R", L"ShoulderArmor_R"
    };
    for (const wchar_t* s : kCandidates) {
        Vec3 p{};
        if (derive_pivot(rig, s, &p)) {
            API::get()->log_info("[Halo-CampE-UEVR]   candidate pivot %-16ls = (%7.1f, %7.1f, %7.1f) cm",
                                 s, p.x, p.y, p.z);
        } else {
            API::get()->log_info("[Halo-CampE-UEVR]   candidate pivot %-16ls = <unresolved>", s);
        }
    }
}


} // namespace halo
