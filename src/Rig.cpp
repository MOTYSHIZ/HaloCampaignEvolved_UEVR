#include "Rig.hpp"
#include "Config.hpp"
#include "ArmDriver.hpp"  // the palette driver owns the weapon too; do not attach under it
#include "Reticule.hpp"   // reticle_arm_stray_check: a weapon change rebuilds the HUD crosshair
#include "MotionAimControl.hpp"  // get_pose(), g_turn_offset -- the frozen bore capture works in the aim frame
#include "TwoHandAim.hpp"        // two_hand_bend_orientation -- the aim frame must swing with the rig

#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <unordered_map>
#include <unordered_set>

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
std::atomic<float> g_dbg_wpn_dx{0.0f}, g_dbg_wpn_dy{0.0f}, g_dbg_wpn_dz{0.0f};
std::atomic<bool>  g_dbg_wpn_ok{false};
std::atomic<int>   g_dbg_parent_changes{0};
std::atomic<float> g_dbg_sock_x{0.0f}, g_dbg_sock_y{0.0f}, g_dbg_sock_z{0.0f};
std::atomic<float> g_dbg_sock_p{0.0f}, g_dbg_sock_yw{0.0f}, g_dbg_sock_r{0.0f};
std::atomic<bool>  g_dbg_sock_ok{false};
std::atomic<float> g_dbg_Lact_x{0.0f}, g_dbg_Lact_y{0.0f}, g_dbg_Lact_z{0.0f};
std::atomic<bool>  g_dbg_Lact_ok{false};
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


// ---------------------------------------------------------------- COMPONENT LOOKUP, OWNER-SCOPED
// A UE TArray header. Read raw: the elements are UObject* and nothing here ever writes.
struct FRawArrayRO { void* data; int32_t num; int32_t max; };

// Scan one TArray<UObject*> property of `owner` for a component whose class name matches EXACTLY.
// Fails closed on anything unreadable rather than faulting -- every hop here can be mid-teardown on
// this title, and a bogus component is far worse than no component.
//
// !!! EXACT, NEVER A PREFIX. The pawn carries BPC_FP_SkeletalMesh_C,
// BPC_FP_TranslucentSkeletalMesh_C and BPC_FP_ShadowSkeletalMesh_C -- three meshes with three
// different jobs, and the shadow one is a FULL-BODY proxy standing on the floor. A prefix match
// picks the wrong one and lifts the player's shadow off the ground (see the shell block below).
static API::UObject* find_component_in_array(API::UObject* owner, const wchar_t* prop,
                                             const wchar_t* cls_name) {
    if (owner == nullptr) return nullptr;
    auto* arr = owner->get_property_data<FRawArrayRO>(prop);
    if (arr == nullptr || IsBadReadPtr(arr, sizeof(FRawArrayRO))) return nullptr;
    if (arr->data == nullptr || arr->num <= 0 || arr->num > 4096) return nullptr;

    auto** elems = reinterpret_cast<API::UObject**>(arr->data);
    if (IsBadReadPtr(elems, sizeof(void*) * (size_t)arr->num)) return nullptr;

    for (int32_t i = 0; i < arr->num; ++i) {
        auto* c = elems[i];
        if (c == nullptr || IsBadReadPtr(c, sizeof(void*))) continue;
        if (class_name_of(c) == cls_name) return c;
    }
    return nullptr;
}

// A component of the LIVE LOCAL PAWN, by exact class name.
//
// BlueprintCreatedComponents lists the construction-script components flat, independent of how they
// are attached, so it is preferred over walking attachment topology we have not measured.
// get_local_pawn is the cheap per-frame handle -- no object-array walk, which is the whole point.
static API::UObject* component_on_pawn(const wchar_t* cls_name) {
    auto* pawn = API::get()->get_local_pawn(0);
    if (auto* c = find_component_in_array(pawn, L"BlueprintCreatedComponents", cls_name)) return c;
    if (auto* c = find_component_in_array(pawn, L"InstanceComponents",         cls_name)) return c;
    return nullptr;
}

// The arms rig's class. One spelling, because three separate string literals of the same name is
// exactly how one of them ends up subtly different.
static constexpr const wchar_t* kRigClass = L"BPC_FP_SkeletalMesh_C";

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
    if (class_name_of(par).find(kRigClass) == std::wstring::npos) return nullptr;
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
    return g_rig_track.get_checked(kRigClass) != nullptr;
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

API::UObject* fp_weapon_root() {
    // Same walk resolve_rig() uses in reverse (weapon -> RootComponent -> AttachParent == rig),
    // stopping one step earlier. get_checked, not get: actors on this title are pooled.
    return follow_object(fp_weapon_actor(), L"RootComponent");
}

API::UObject* rig_tracked_component() {
    return g_rig_track.get_checked(kRigClass);
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

    // The fast path above is O(1) and handles steady play. This is the fallback, and during a
    // LEVEL LOAD it is what runs: there is no weapon actor to derive from yet, so every attempt
    // falls through here. Measured at 65-78 ms a sweep, 12-14 sweeps per 600-tick window --
    // 406 ms and 443 ms of game-thread stall in two consecutive windows, the largest single
    // contributor to load-time hitching.
    std::unordered_map<const void*, std::wstring> name_of_class;

    const int32_t n = arr->get_object_count();
    for (int32_t i = 0; i < n; ++i) {
        auto* obj = arr->get_object(i);
        if (obj == nullptr) continue;

        // Candidate = a first-person weapon actor, e.g. BP_FP_Magnum_WeaponActor_C.
        // MEMOISED ON THE CLASS. This walks the entire UObject array building a class-name string
        // per object, and objects outnumber classes by orders of magnitude -- the same few names
        // were being rebuilt tens of thousands of times per sweep.
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

    // ---- PAWN-DOWN FALLBACK: there is no first-person weapon to walk back from.
    //
    // Being unarmed is a REAL, PLAYABLE STATE on this game, not just a loading artifact -- the
    // campaign opens with no weapon in your hands at all. The shipped content carries eighteen
    // BP_FP_*_WeaponActor classes and not one unarmed variant, so the sweep above is not missing
    // an actor: there genuinely is none, and it will find nothing however often it runs.
    //
    // The rig is a component of the PAWN, so ask the pawn directly. This does NOT violate the
    // never-adopt-by-class-match banner at the top of this section: the search is scoped to the
    // LIVE LOCAL PAWN's own component list, so a pooled corpse from a previous life is not
    // reachable at all -- the same property that makes the weapon walk trustworthy, obtained a
    // different way.
    //
    // Deliberately AFTER the sweep, never before it. The sweep is what establishes g_fp_weapon,
    // and that handle is what the fast path, the pivot derivation and the reticle re-arm all run
    // on. Short-circuiting to the pawn would leave the weapon handle permanently unset, so picking
    // a gun up would never be noticed and the unarmed state would never end.
    if (auto* rig = component_on_pawn(kRigClass)) {
        note_resolved_rig(rig);
        return rig;
    }
    return nullptr;
}

// ---------------------------------------------------------------- FIRST-PERSON PRESENTATION
// "Is the game presenting the player in first person right now?" -- which is the question stick
// mode actually wants answered, asked directly instead of inferred from whether a gun exists.
//
// WHY THE WEAPON IS THE WRONG PROXY. Stick mode stands the whole motion stack down when the FP
// weapon route dies, because that is what a vehicle seat, a cutscene, death and the post-load
// window all look like from outside. Standing on your feet with nothing in your hands looks
// identical to it -- so the campaign's opening minutes played as flat gamepad, motion aim and
// snap turn dead, which is where new players got stuck.
//
// This is the same bug class the menu detector already had and already fixed: `no_rig` was a cheap
// stand-in for "a load or transition" that also meant "no first-person weapon", i.e. a vehicle. The
// fix there was to stop proxying and read the real state. Same fix here.
//
// THE VALUE IS LEARNED, NOT ASSUMED. CurrentBlamCameraPerspective is an EBlamCameraPerspective
// byte on BlamPawn. Its declared order is {FirstPerson, ThirdPerson, None}, but the one live sample
// taken on foot with a weapon in hand read 2 -- which under that order would be `None`. So the
// declared order cannot be trusted and a hardcoded comparison would be a guess.
//
// It does not need to be trusted. Whenever the FP weapon route IS alive the player is
// unambiguously on foot in first person, so whatever the byte reads at that moment IS this build's
// first-person value, by observation. The seed below is only what to believe before the first
// weapon of the session has been seen; the first armed tick corrects it, and a build that orders
// the enum differently corrects itself with no code change.
//
// Returns 1 = first person, 0 = not first person, -1 = could not tell. Callers must treat -1 as
// "assume nothing" and fall back to the weapon-route behaviour: a build where this property is
// missing or renamed then behaves exactly as it did before this existed.
// HOW THE BYTE IS REACHED, and why it is not simply a reflection lookup. The FA session that found
// this field recorded that reflection AGAINST THE LIVE PAWN INSTANCE fails on this title while a
// raw read at the offset succeeds -- and its recommendation was explicit: consume it the way
// ControlRotation is consumed, raw at a validated offset behind IsBadReadPtr, failing closed.
//
// So this does what resolve_control_rotation_offset does. Reflection is TRIED, because when it
// answers it is right by construction and survives a patch that moves the field; its answer is
// range-checked before adoption, since a resolved-but-wrong offset is worse than a compiled one
// (it looks authoritative); and the measured offset is the fallback when reflection declines.
// Which one is in force is logged once, so "the fix silently did nothing" is never a diagnosis
// anyone has to reach for.
// ADDR-HYGIENE: resolved -- resolve_perspective_offset() prefers UE reflection, range-checks the
// answer, and falls back to this measured value only when reflection declines (logged either way).
constexpr uintptr_t PERSPECTIVE_OFFSET_EXPECTED = 0x3C1;   // BlamPawn+961, measured 2026-08-02
static std::atomic<size_t> g_persp_offset{PERSPECTIVE_OFFSET_EXPECTED};

static void resolve_perspective_offset(API::UObject* pawn) {
    static bool s_done = false;
    if (s_done || pawn == nullptr) return;
    s_done = true;

    auto* p = pawn->get_property_data<uint8_t>(L"CurrentBlamCameraPerspective");
    if (p == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] PERSP: reflection could not find "
                             "CurrentBlamCameraPerspective -- using the measured offset +0x%llX",
                             (unsigned long long)PERSPECTIVE_OFFSET_EXPECTED);
        return;
    }
    const uintptr_t off = (uintptr_t)p - (uintptr_t)pawn;
    // A one-byte enum has no alignment to check, so the plausibility gate is position alone: past
    // the UObject header, inside a pawn a few KB long.
    if (off < 0x40 || off > 0x4000) {
        API::get()->log_info("[Halo-CampE-UEVR] PERSP: reflection returned an implausible offset "
                             "+0x%llX -- keeping the measured +0x%llX",
                             (unsigned long long)off,
                             (unsigned long long)PERSPECTIVE_OFFSET_EXPECTED);
        return;
    }
    g_persp_offset.store((size_t)off, std::memory_order_relaxed);
    API::get()->log_info("[Halo-CampE-UEVR] PERSP: CurrentBlamCameraPerspective at +0x%llX "
                         "(reflection; the measured expectation is +0x%llX)",
                         (unsigned long long)off,
                         (unsigned long long)PERSPECTIVE_OFFSET_EXPECTED);
}

// EBlamCameraPerspective declares three members, so a live value above this is not the enum -- it
// is a wrong offset being read as one. Rejecting it costs the on-foot exception (fail closed, the
// old behaviour) and buys immunity to the failure that would otherwise be invisible: garbage that
// happens to be stable gets LEARNED while armed, and then every vehicle reads as first person.
constexpr uint8_t PERSPECTIVE_VALUE_MAX = 3;

// THE SEED. MEASURED, and it took two wrong guesses to stop guessing.
//
// It only matters before the session's first weapon -- which IS the campaign opening this exists
// to fix, so it is not a detail. The history is worth keeping because it is the whole lesson:
//   * 2 -- VEHICLE_CAMERA_FINDINGS.md §0, read 2026-08-02 on an older build. Flagged in that same
//          paragraph as not matching the declared enum order. Shipped as the seed; the on-foot
//          exception never fired.
//   * 0 -- inferred from the declared order {FirstPerson, ThirdPerson, None} plus a live read that
//          turned out to be sampled during a CUTSCENE. Shipped; still never fired.
//   * 1 -- MEASURED. Logged by the value tracer across a full opening (log 2026-08-15 17:23-17:25)
//          and independently confirmed by the learn path the moment a weapon went live:
//          `first-person value learned to 1`.
//
// The tracer settled in one session what two rounds of reasoning-from-a-constant could not. When a
// value can be observed, observe it; the declared order of an enum this game did not have to
// respect is not evidence.
//
// What the byte actually does on this build, from the same log: 1 = first person (armed OR
// unarmed), 0 = cinematic/none, 18 = a transient during level load (rejected by the range gate
// below, which is why the load window still behaves as it always did).
//
// Being wrong here is still not silent: the first armed tick re-learns and says so in the log.
static uint8_t g_persp_fp_value = 1;
static bool    g_persp_learned  = false;
std::atomic<int> g_dbg_persp{-1};         // last raw byte, for the transition log

int fp_presentation_state(bool route_alive) {
    auto* pawn = API::get()->get_local_pawn(0);
    if (pawn == nullptr) return -1;

    resolve_perspective_offset(pawn);

    const auto* p = reinterpret_cast<const uint8_t*>(pawn)
                  + g_persp_offset.load(std::memory_order_relaxed);
    if (IsBadReadPtr(p, 1)) return -1;

    const uint8_t v = *p;
    const int prev = g_dbg_persp.exchange((int)v, std::memory_order_relaxed);

    // THE CALIBRATION THE FINDINGS DOC ASKED FOR, taken continuously instead of by appointment.
    // Edge-triggered, so it costs nothing while the value holds, and one ordinary playthrough
    // (unarmed opening -> pick up a gun -> board something) records every value this byte takes
    // and what the player was doing at the time.
    //
    // It also exposes the failure this whole approach is exposed to: reflection cannot find the
    // property on this build, so the offset is unverified and the byte could be an unrelated one
    // that happens to read a stable value. A session that shows NO change line across a weapon
    // pickup and a vehicle ride has proved exactly that, and the answer is stickonfoot=0 plus the
    // GetSeatStates route -- not another guess at the constant.
    // CAPPED. If the offset is wrong the byte can churn every tick, and an uncapped edge log would
    // then be 32 lines a second in a shipping build -- the exact chattiness the dev-tooling rule
    // exists to stop. The cap is also the diagnosis: hitting it AT ALL means this is not a
    // three-state enum and the whole approach is unsound on this build.
    static int s_persp_lines = 0;
    constexpr int PERSP_LINE_CAP = 48;
    if (prev != (int)v && s_persp_lines <= PERSP_LINE_CAP) {
        if (++s_persp_lines > PERSP_LINE_CAP) {
            API::get()->log_info("[Halo-CampE-UEVR] PERSP: value changed more than %d times -- "
                                 "this byte is not a stable perspective enum on this build. "
                                 "Silencing; treat the on-foot exception as unreliable here "
                                 "(stickonfoot=0).", PERSP_LINE_CAP);
        } else {
            API::get()->log_info("[Halo-CampE-UEVR] PERSP value %d -> %u (route=%d, believed "
                                 "first-person value %u%s)",
                                 prev, (unsigned)v, (int)route_alive, (unsigned)g_persp_fp_value,
                                 g_persp_learned ? "" : ", SEEDED not learned");
        }
    }

    if (v > PERSPECTIVE_VALUE_MAX) return -1;

    if (route_alive) {
        // A live first-person weapon IS the ground truth. Re-learn on any disagreement rather than
        // only once: if the value legitimately differs per level or per build, the correction
        // should follow it instead of latching the first thing ever seen.
        if (!g_persp_learned || g_persp_fp_value != v) {
            API::get()->log_info("[Halo-CampE-UEVR] perspective: first-person value %s to %u "
                                 "(learned from a live FP weapon)",
                                 g_persp_learned ? "RE-LEARNED" : "learned", (unsigned)v);
            g_persp_fp_value = v;
            g_persp_learned  = true;
        }
        return 1;
    }
    return (v == g_persp_fp_value) ? 1 : 0;
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

API::UObject* resolve_shield_shell(API::UObject* rig_parent) {
    // component_on_pawn covers BlueprintCreatedComponents then InstanceComponents off the live
    // local pawn -- the cheap per-frame handle, no object-array walk. A full sweep here is exactly
    // the pattern that has already collapsed framerate in a live session once.
    if (auto* s = component_on_pawn(kShellClass)) return s;
    // Fallback: as a sibling of the arms, the shell hangs off the arms' own attach parent.
    if (auto* s = find_component_in_array(rig_parent, L"AttachChildren", kShellClass)) return s;
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

// ---------------------------------------------------------------- VISIBILITY
// SetVisibility(bool bNewVisibility, bool bPropagateToChildren).
//
// PROPAGATION IS NOT OPTIONAL HERE. UE does not push visibility to children by default, and the
// arms rig has six BPC_FP_StaticMesh_C children -- the shoulder, elbow and wrist armour. Hiding the
// rig alone leaves those six floating in front of the camera in the exact shape of the arms that
// are no longer there, which is a worse artefact than the thing being hidden.
//
// It does NOT reach the shield shell: that is a SIBLING of the rig, not a child (see the shell
// block above), so it takes its own call.
bool rig_set_visible(API::UObject* comp, bool visible) {
    if (comp == nullptr) return false;
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    params[0] = visible ? 1 : 0;
    params[1] = 1;                     // bPropagateToChildren
    comp->call_function(L"SetVisibility", params);
    return true;
}

// KEEP A HIDDEN SKELETAL MESH ANIMATING -- WRITTEN AS A PROPERTY, NOT CALLED AS A FUNCTION.
//
// SetVisibilityBasedAnimTickOption DOES NOT EXIST as a UFUNCTION on this build. The bone dump
// probes it by name and reports "-- absent" (measured in-session 2026-08-23), and
// call_function() on a name that does not resolve fails SILENTLY. So the previous version of
// this helper reported success while doing nothing whatsoever: every hide path believed it had
// kept the pose alive, the mesh kept UE default OnlyTickPoseWhenRendered, the PrimaryWeapon
// socket froze the instant the arms were hidden, and the weapon lost its recoil. armkeeppose
// had never worked on this title in any build, including the fork it arrived in.
//
// This is the failure mode the project already names elsewhere -- a call that "fails QUIETLY
// with a plausible answer". The lesson it cost is the reason for the logging below.
//
// The underlying UPROPERTY is still reachable by reflection. It is a
// TEnumAsByte<EVisibilityBasedAnimTickOption>, i.e. one byte, where 0 =
// AlwaysTickPoseAndRefreshBones. Same idiom as read_byte_prop() in Plugin.cpp.
//
// RETURNS FALSE and says so once when the property cannot be resolved, and confirms success
// once when it can. An unobservable no-op is exactly what hid this for the life of the feature,
// so this call is never allowed to be silent in either direction again.
bool rig_set_always_tick_pose(API::UObject* comp) {
    if (comp == nullptr) return false;
    auto* cls  = comp->get_class();
    auto* prop = (cls != nullptr) ? cls->find_property(L"VisibilityBasedAnimTickOption") : nullptr;
    if (prop == nullptr) {
        // Static meshes legitimately have no such property, so this is only interesting for a
        // SKELETAL mesh -- which is the only kind the callers pass. One line, once.
        static bool s_warned = false;
        if (!s_warned) {
            s_warned = true;
            API::get()->log_info("[Halo-CampE-UEVR] ARMHIDE: VisibilityBasedAnimTickOption did not "
                                 "resolve -- hidden arms will FREEZE their pose, so the weapon "
                                 "loses recoil and its socket stops moving. armkeeppose cannot "
                                 "work on this build; use a hide that leaves the mesh visible.");
        }
        return false;
    }
    *(reinterpret_cast<uint8_t*>(comp) + prop->get_offset()) = 0;   // AlwaysTickPoseAndRefreshBones
    static bool s_ok_logged = false;
    if (!s_ok_logged) {
        s_ok_logged = true;
        API::get()->log_info("[Halo-CampE-UEVR] ARMHIDE: VisibilityBasedAnimTickOption resolved at "
                             "offset 0x%X and set to AlwaysTickPoseAndRefreshBones -- hidden arms "
                             "keep animating, so the weapon keeps its recoil.",
                             (unsigned)prop->get_offset());
    }
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

// WHAT is currently attached. g_attached alone was enough while the arms mesh was the only
// possible target; attaching the WEAPON instead makes the target change on every swap and every
// respawn, and releasing the wrong object leaves a live attachment nobody owns.
API::UObject* g_attach_obj = nullptr;

void attach_apply(API::UObject* rig, const Quat& rot_off, const Vec3& loc_off_cm) {
    // NOT GATED ON THE ARM DRIVER, and that is deliberate -- it was, briefly, and it was wrong.
    //
    // The two-drivers rule is about two things moving the SAME object. This attachment moves
    // the WEAPON; the palette route poses the ARM NODES. Different objects, no conflict.
    //
    // More importantly, this path carries the CALIBRATION -- grip_deg/grip_yaw/grip_roll, the
    // off_x/y/z placement, and the per-weapon wpnoff deltas the player tuned by hand. Posing
    // the weapon from the palette instead replaced all of that with a raw controller basis,
    // which discards the calibration and is why the gun stopped sitting where it was tuned to
    // sit. The weapon keeps its calibrated driver; the hand is IK'd TO the weapon.
    if (rig == nullptr) return;
    // Target changed under us -- drop the old one first, or it stays pinned to the controller
    // forever with no reference left to release it by.
    if (g_attached && g_attach_obj != nullptr && g_attach_obj != rig) {
        API::UObjectHook::remove_motion_controller_state(g_attach_obj);
    }
    auto* st = API::UObjectHook::get_or_add_motion_controller_state(rig);
    if (st == nullptr) return;

    UEVR_Quaternionf q{rot_off.x, rot_off.y, rot_off.z, rot_off.w};
    UEVR_Vector3f    v{loc_off_cm.x, loc_off_cm.y, loc_off_cm.z};
    st->set_rotation_offset(&q);
    st->set_location_offset(&v);
    st->set_hand(1);   // MotionControllerStateBase::Hand::RIGHT
    st->set_permanent(g_cfg.attach_permanent);
    g_attach_obj = rig;
    g_attached = true;
}

void attach_release(API::UObject* rig, const char* why) {
    if (!g_attached) return;
    // The RECORDED target wins over the argument: callers pass the rig because that used to be the
    // only thing attachable, and in weapon mode that is not what is attached.
    auto* obj = (g_attach_obj != nullptr) ? g_attach_obj : rig;
    if (obj != nullptr) API::UObjectHook::remove_motion_controller_state(obj);
    g_attach_obj = nullptr;
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

// ---------------------------------------------------------------- SHOT POINT (muzzle marker)
// The equipped weapon's authored muzzle marker in WORLD space. Halo weapons carry an
// "fx_muzzleflash" marker on their own skeletal model (verified per-weapon 2026-07-27; the FX
// system spawns muzzle flashes at it by name via BPFL_BlamEffectUtilities.RandomMuzzleFlashSocketNames).
// We read it the fastest, most honest rung of the cascade -- UE reflection GetSocketLocation with a
// REAL FName -- exactly where the MCP inspector could not (its FName args marshal to None). The
// weapon is a separate actor attached at PrimaryWeapon; the marker lives on its skeletal mesh
// COMPONENT, whose class we do NOT hardcode (a wrong class name is the one-build constant that
// rots) -- every mesh-like component is probed and the first that owns the marker wins.
constexpr const wchar_t* kMuzzleMarker = L"fx_muzzleflash";   // primary / display default

// Muzzle socket names tried at RUNTIME (per-tick), in order; the first a mesh owns wins. UNSC
// weapons use fx_muzzleflash; others name it differently -- the flak/fuel-rod cannon
// (BP_FP_FlakCannon_WeaponActor_C) has NO fx_muzzleflash socket -- so this is a LIST, not one name.
// GetSocketLocation resolves BONE names too, so a muzzle bone is found the same way. Keep this list
// short (it is probed per tick for a weapon with no match); promote a name here once the dev scan
// below identifies it. Extend when a new weapon's muzzle is discovered.
static constexpr const wchar_t* kMuzzleMarkers[] = {
    L"fx_muzzleflash", L"fx_muzzleflash_01", L"fx_muzzle", L"fx_fire",
    L"Muzzle", L"MuzzleFlash", L"muzzle", L"b_muzzle",
    // Meteorite FP-weapon skeleton BONE (dumped from BP_FP_FlakCannon_WeaponActor_C, 2026-09-13):
    // the flak/fuel-rod cannon has no fx_muzzleflash SOCKET but does have this barrel bone.
    // GetSocketLocation resolves bones, so this identifies the skeletal weapon mesh; the bore is
    // then that component's forward, same as any socket-identified weapon (its FWD axis reads level
    // while UP reads ~vertical, confirming FWD is the barrel). Tried last so a real fx_muzzleflash
    // socket always wins on weapons that have one.
    L"Barrel_M",
};

// GetSocketLocation returns the component's OWN origin for a socket/bone it does NOT have (the None
// trap call_socket_location documents), so a name EXISTS iff its lookup differs from a bogus name's
// -- otherwise both merely returned the origin. Test several names against ONE origin read, so N
// names cost N+1 calls, not 2N; returns the first that resolves off the origin (world pos in *out),
// else nullptr. Resolves BONE names too, so a muzzle is found whether it is a socket or a bone.
static const wchar_t* first_socket_on_component(API::UObject* comp, const wchar_t* const* names,
                                                size_t count, Vec3* out) {
    Vec3 origin{};
    if (!call_socket_location(comp, L"__halo_vr_nomatch__", &origin)) return nullptr;
    for (size_t i = 0; i < count; ++i) {
        Vec3 at{};
        if (!call_socket_location(comp, names[i], &at)) continue;
        const float dx = at.x - origin.x, dy = at.y - origin.y, dz = at.z - origin.z;
        if ((dx * dx + dy * dy + dz * dz) >= 1e-6f) { if (out) *out = at; return names[i]; }
    }
    return nullptr;
}

// Probe the weapon actor's own component arrays for whichever mesh carries a known muzzle marker.
// Tries kMuzzleMarkers in order; the first mesh+name that resolves wins. *out_name (if given) gets
// the matched name, for logging and so the caller knows which convention this weapon uses.
static API::UObject* weapon_marker_component(API::UObject* wpn, Vec3* out,
                                             const wchar_t** out_name = nullptr) {
    if (wpn == nullptr) return nullptr;
    constexpr size_t kN = sizeof(kMuzzleMarkers) / sizeof(kMuzzleMarkers[0]);
    for (const wchar_t* arrp : { L"BlueprintCreatedComponents", L"InstanceComponents" }) {
        auto* arr = wpn->get_property_data<FRawArrayRO>(arrp);
        if (arr == nullptr || IsBadReadPtr(arr, sizeof(FRawArrayRO))) continue;
        if (arr->data == nullptr || arr->num <= 0 || arr->num > 4096) continue;
        auto** elems = reinterpret_cast<API::UObject**>(arr->data);
        if (IsBadReadPtr(elems, sizeof(void*) * (size_t)arr->num)) continue;
        for (int32_t i = 0; i < arr->num; ++i) {
            auto* c = elems[i];
            if (c == nullptr || IsBadReadPtr(c, sizeof(void*))) continue;
            if (class_name_of(c).find(L"Mesh") == std::wstring::npos) continue;  // only meshes have sockets/bones
            const wchar_t* m = first_socket_on_component(c, kMuzzleMarkers, kN, out);
            if (m != nullptr) { if (out_name) *out_name = m; return c; }
        }
    }
    return nullptr;
}

bool shotpoint_world(Vec3* out_pos, Vec3* out_fwd) {
    auto* wpn = fp_weapon_actor();
    if (wpn == nullptr) return false;
    Vec3 p{};
    auto* comp = weapon_marker_component(wpn, &p);
    if (comp == nullptr) return false;
    if (out_pos) *out_pos = p;
    if (out_fwd) {
        Vec3 f{};
        // GetForwardVector: the component's world +X. This build has no socket-rotation UFUNCTION,
        // so the mesh component forward stands in for the muzzle's authored forward. Zeroed on a
        // failed read so the caller can tell "no direction" from a real one.
        *out_fwd = call_ret_vec3(comp, L"GetForwardVector", &f) ? f : Vec3{0.0f, 0.0f, 0.0f};
    }
    return true;
}

// Published at TICK rate by shotpoint_tick(), read at aim rate by derive_ctrl_angles -- the
// two-clocks rule: the mesh-forward read is reflection (GetSocketLocation x2 + GetForwardVector)
// and belongs on the ~32 Hz tick; the aim law runs far faster and must not do reflection.
static std::atomic<float> g_sp_fx{0.0f}, g_sp_fy{0.0f}, g_sp_fz{0.0f};
static std::atomic<bool>  g_sp_fwd_valid{false};

// FROZEN per-weapon bore, in the AIM CONTROLLER's own VR-local frame (snap-turn removed), captured
// while the weapon rigidly tracks the hand. shotpoint_tick publishes it AS-IS to g_bl_*; the aim
// hook rotates it by the LIVE controller pose and re-adds the snap turn -> FROZEN (immune to
// reload/recoil -- no mesh read at aim time) and ROLL-INVARIANT. An End recalibration is handled by
// RE-CAPTURE (the auto gate re-stores when the value drifts), not by any placement transform.
// Populated in DEV (auto-capture at a steady hold); consumed in any build.
static std::unordered_map<std::wstring, Vec3> g_bore_cache;
static std::atomic<float> g_bl_x{0.0f}, g_bl_y{0.0f}, g_bl_z{0.0f};
static std::atomic<bool>  g_bl_valid{false};

// THE DEFAULT for uncaptured weapons is the ASSAULT RIFLE's bore: a weapon with no capture of its
// own aims with the AR's forward instead of the animation-following live-mesh bootstrap.
//
// BAKED FACTORY DEFAULT (kDefBore*): the AR's measured bore in the schema-v3 un-driven-weapon frame,
// so uncaptured weapons are animation-immune from a ZERO-capture / release install -- no file and no
// held-AR required. Measured 2026-09-13 (7-weapon sweep; every marker sat within ~5 deg of
// weapon-forward, so the AR's forward is a good fallback for any weapon). It is compiled rather than
// shipped in halo_vr.cfg precisely so it works when NO calib data is present at all. Because F now
// divides out the live grip trim, this value is grip-trim-INDEPENDENT (the capture that produced it
// read ~straight-forward under a non-default End -- proof the frame cancels the grip), so it is a
// true factory constant. A SCHEMA BUMP INVALIDATES IT: re-measure and update alongside kShotFixSchema.
//
// g_def_* / g_def_valid are the RUNTIME default: set true only when the user captures the AR or a
// calib shotfixdefault is loaded, so they -- not the baked constant -- are what gets persisted
// (shotpoint_emit_calib keys off g_def_valid). shotpoint_tick uses the runtime default when valid,
// else falls back to kDefBore*, so the baked value never pollutes a user's calib file.
static constexpr float kDefBoreX = 0.99999f, kDefBoreY = 0.00250f, kDefBoreZ = 0.00401f;
static std::atomic<float> g_def_x{0.0f}, g_def_y{0.0f}, g_def_z{0.0f};
static std::atomic<bool>  g_def_valid{false};

void shotpoint_tick() {
    Vec3 p{}, f{};
    if (shotpoint_world(&p, &f) && (f.x * f.x + f.y * f.y + f.z * f.z) > 0.5f) {
        g_sp_fx.store(f.x); g_sp_fy.store(f.y); g_sp_fz.store(f.z);
        g_sp_fwd_valid.store(true);
    } else {
        g_sp_fwd_valid.store(false);   // no weapon / no marker -> caller keeps its own direction
    }

    // Publish the stored bore for the held weapon: own capture -> user AR default -> BAKED AR
    // default. The aim hook reconstructs it against the live rig composition. Captures are MANUAL
    // (Page Down); an uncaptured weapon always has the baked fallback, so it is never left on the
    // animation-following live-mesh bootstrap even from a zero-capture install.
    bool have = false;
    if (auto* wpn = fp_weapon_actor()) {
        auto it = g_bore_cache.find(class_name_of(wpn));
        if (it != g_bore_cache.end()) {
            g_bl_x.store(it->second.x); g_bl_y.store(it->second.y); g_bl_z.store(it->second.z);
        } else if (g_def_valid.load()) {
            g_bl_x.store(g_def_x.load()); g_bl_y.store(g_def_y.load()); g_bl_z.store(g_def_z.load());
        } else {
            g_bl_x.store(kDefBoreX); g_bl_y.store(kDefBoreY); g_bl_z.store(kDefBoreZ);
        }
        have = true;
    }
    g_bl_valid.store(have);
}

bool shotpoint_dir(Vec3* out_fwd) {
    if (!g_sp_fwd_valid.load()) return false;
    if (out_fwd) *out_fwd = Vec3{g_sp_fx.load(), g_sp_fy.load(), g_sp_fz.load()};
    return true;
}

bool shotpoint_bore_local(Vec3* out) {
    if (!g_bl_valid.load()) return false;
    if (out) *out = Vec3{g_bl_x.load(), g_bl_y.load(), g_bl_z.load()};
    return true;
}

// ---- PERSISTENCE (calib file). Class names are ASCII (BP_FP_...), so narrow<->wide is a byte cast.
void shotpoint_set_intrinsic(const char* cls, float x, float y, float z) {
    if (cls == nullptr || cls[0] == 0) return;
    std::wstring w;
    for (const char* p = cls; *p; ++p) w.push_back((wchar_t)(unsigned char)*p);
    g_bore_cache[w] = Vec3{x, y, z};
}

void shotpoint_set_default(float x, float y, float z) {
    g_def_x.store(x); g_def_y.store(y); g_def_z.store(z);
    g_def_valid.store(true);
}

bool shotpoint_schema_ok(int ver) { return ver == kShotFixSchema; }

void shotpoint_emit_calib(std::FILE* f) {
    if (f == nullptr) return;
    if (g_bore_cache.empty() && !g_def_valid.load()) return;
    std::fprintf(f,
        "# Shot-point per-weapon bore, in the aim controller's frame. shotfixver stamps the frame\r\n"
        "# convention and must stay ABOVE the lines it covers. A measurement -- do not hand-edit.\r\n"
        "# Delete these lines (or recalibrate End and hold steady) to recapture.\r\n"
        "shotfixver=%d\r\n", kShotFixSchema);
    for (const auto& kv : g_bore_cache) {
        std::string n;
        for (wchar_t c : kv.first) n.push_back((char)c);
        std::fprintf(f, "shotfix=%s,%.5f,%.5f,%.5f\r\n", n.c_str(),
                     kv.second.x, kv.second.y, kv.second.z);
    }
    if (g_def_valid.load()) {
        std::fprintf(f, "shotfixdefault=%.5f,%.5f,%.5f\r\n",
                     g_def_x.load(), g_def_y.load(), g_def_z.load());
    }
}

void shotpoint_dev_readout(unsigned tick) {
#if HALO_VR_DEV
    if (g_cfg.shot_aim_log <= 0) return;
    if ((tick % (unsigned)g_cfg.shot_aim_log) != 0) return;
    auto* wpn = fp_weapon_actor();
    if (wpn == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] SHOTPOINT: no weapon equipped -> controller aim path (unchanged)");
        return;
    }
    const std::wstring wc = class_name_of(wpn);
    Vec3 p{};
    const wchar_t* matched = nullptr;
    auto* comp = weapon_marker_component(wpn, &p, &matched);
    if (comp != nullptr) {
        // ALL THREE component axes, each as UE-convention game angles (yaw about +Z, +X forward).
        // This game uses NON-STANDARD axis conventions -- the scope pane's aim axis turned out to be
        // GetUpVector, not GetForwardVector (ScopeLayer.hpp) -- so which axis is the BORE cannot be
        // assumed. Log all three; in-headset, aim at a distant reference and whichever axis's
        // yaw/pitch matches where you are pointing IS the bore. (SimVR cannot answer this: its idle
        // pose leaves the gun lowered, so the axes do not point where the player would aim.)
        auto ue = [](const Vec3& v, float* y, float* pt) {
            *y  = std::atan2(v.y, v.x) * RAD2DEG;
            *pt = std::asin(clampf(v.z, -1.0f, 1.0f)) * RAD2DEG;
        };
        Vec3 vf{}, vu{}, vr{};
        const bool hf = call_ret_vec3(comp, L"GetForwardVector", &vf);
        const bool hu = call_ret_vec3(comp, L"GetUpVector",      &vu);
        const bool hr = call_ret_vec3(comp, L"GetRightVector",   &vr);
        float fy=0,fp=0,uy=0,up=0,ry=0,rp=0;
        if (hf) ue(vf,&fy,&fp);  if (hu) ue(vu,&uy,&up);  if (hr) ue(vr,&ry,&rp);
        API::get()->log_info(
            "[Halo-CampE-UEVR] SHOTPOINT: '%ls' marker '%ls' on %ls | pos (%.1f,%.1f,%.1f) | "
            "FWD y%.1f p%.1f | UP y%.1f p%.1f | RIGHT y%.1f p%.1f | have f%d u%d r%d",
            wc.c_str(), matched ? matched : kMuzzleMarker, class_name_of(comp).c_str(), p.x, p.y, p.z,
            fy, fp, uy, up, ry, rp, (int)hf, (int)hu, (int)hr);
    } else {
        // DISCOVERY (dev-only): no known muzzle name matched. Dump the weapon's attach vocabulary
        // ONCE per class so the real muzzle is IDENTIFIED, not guessed:
        //   (1) each mesh comp's FWD/UP/RIGHT axes as game angles -- aim at a distant reference and
        //       whichever axis matches where you point IS the bore (the scope pane's was UP, not
        //       FWD). A matching comp axis enables a socket-free capture straight off that comp.
        //   (2) the skeleton's bone names via GetNumBones/GetBoneName (proven on this build, see
        //       Arms.cpp), in case the muzzle is a named bone we can add to kMuzzleMarkers.
        // Once-per-class (a one-shot burst the first time a weapon is held) + behind this dev +
        // throttled logger, so nothing here runs in a player build or per frame.
        API::get()->log_info("[Halo-CampE-UEVR] SHOTPOINT: weapon '%ls' has NO known muzzle marker -> "
                             "baked AR default in use", wc.c_str());
        static std::unordered_set<std::wstring> s_dumped;
        if (s_dumped.insert(wc).second) {
            auto ang = [](const Vec3& v, float* y, float* pt) {
                *y  = std::atan2(v.y, v.x) * RAD2DEG;
                *pt = std::asin(clampf(v.z, -1.0f, 1.0f)) * RAD2DEG;
            };
            for (const wchar_t* arrp : { L"BlueprintCreatedComponents", L"InstanceComponents" }) {
                auto* arr = wpn->get_property_data<FRawArrayRO>(arrp);
                if (arr == nullptr || IsBadReadPtr(arr, sizeof(FRawArrayRO))) continue;
                if (arr->data == nullptr || arr->num <= 0 || arr->num > 4096) continue;
                auto** elems = reinterpret_cast<API::UObject**>(arr->data);
                if (IsBadReadPtr(elems, sizeof(void*) * (size_t)arr->num)) continue;
                for (int32_t i = 0; i < arr->num; ++i) {
                    auto* c = elems[i];
                    if (c == nullptr || IsBadReadPtr(c, sizeof(void*))) continue;
                    const std::wstring cc = class_name_of(c);
                    if (cc.find(L"Mesh") == std::wstring::npos) continue;
                    Vec3 vf{}, vu{}, vr{};
                    const bool hf = call_ret_vec3(c, L"GetForwardVector", &vf);
                    const bool hu = call_ret_vec3(c, L"GetUpVector",      &vu);
                    const bool hr = call_ret_vec3(c, L"GetRightVector",   &vr);
                    float fy=0,fp=0,uy=0,up=0,ry=0,rp=0;
                    if (hf) ang(vf,&fy,&fp);  if (hu) ang(vu,&uy,&up);  if (hr) ang(vr,&ry,&rp);
                    API::get()->log_info("[Halo-CampE-UEVR] SHOTPOINT-DUMP: '%ls' comp '%ls' | "
                                         "FWD y%.1f p%.1f | UP y%.1f p%.1f | RIGHT y%.1f p%.1f | have f%d u%d r%d",
                                         wc.c_str(), cc.c_str(), fy,fp,uy,up,ry,rp,(int)hf,(int)hu,(int)hr);
                    // Bones live only on skeletal meshes; gate on the class so GetNumBones is never
                    // issued at a component that has no such function.
                    if (cc.find(L"Skeletal") == std::wstring::npos) continue;
                    alignas(16) uint8_t pn[RIG_PARAM_BUF] = {0};
                    c->call_function(L"GetNumBones", pn);
                    const int32_t nb = *reinterpret_cast<int32_t*>(pn);
                    if (nb <= 0 || nb > 512) continue;
                    API::get()->log_info("[Halo-CampE-UEVR] SHOTPOINT-DUMP:   '%ls' has %d bones:", cc.c_str(), nb);
                    for (int32_t b = 0; b < nb; ++b) {
                        alignas(16) uint8_t pb[RIG_PARAM_BUF] = {0};
                        *reinterpret_cast<int32_t*>(pb) = b;   // GetBoneName(int32 in@0) -> FName@4
                        c->call_function(L"GetBoneName", pb);
                        const std::wstring bn = reinterpret_cast<API::FName*>(pb + 4)->to_string();
                        if (!bn.empty())
                            API::get()->log_info("[Halo-CampE-UEVR] SHOTPOINT-DUMP:     bone[%d] '%ls'", b, bn.c_str());
                    }
                }
            }
        }
    }
#else
    (void)tick;
#endif
}

// ---------------------------------------------------------------- SHOT-POINT ASSET MEASUREMENT
// Measure fx_muzzleflash ONCE PER WEAPON CLASS as a constant in the weapon's OWN ROOT frame:
// the muzzle POSITION (cm) and the bore DIRECTION, both expressed relative to the weapon actor's
// root component rather than the world. That makes them lane-independent -- a property of the
// asset, not of how any lane places the weapon -- so the same numbers drive the rig lane and the
// palette lane. Combined with the weapon's placement (socket attach in the rig lane, the palette
// grip in the palette lane) they reconstruct the world muzzle and bore, which is what the seam's
// eventual producer needs: bore-relative-to-root x root-relative-to-controller = the aim_fix.
//
// It is exact only at REST: idle sway and recoil animate the mesh relative to the root, so a
// sample taken mid-animation is off. The stability gate is the whole point -- accumulate across
// samples, track the max deviation from the running mean, and only trust the constant once it has
// held still (low deviation over enough samples). A moving reading is visibly unstable and says so.
//
// Dev/recon only (#if HALO_VR_DEV, gated on shotaimlog): reflection every tick while measuring.
#if HALO_VR_DEV
struct AssetMeasure {
    Vec3  muzzle_mean{0, 0, 0};   // root-local, cm
    Vec3  bore_mean{0, 0, 0};     // root-local, ~unit (mean of unit samples)
    int   n = 0;
    float bore_dev_max = 0.0f;    // deg, max angle of a sample off the running mean
    float muzzle_dev_max = 0.0f;  // cm,  max distance of a sample off the running mean
    bool  logged_stable = false;
};
static std::unordered_map<std::wstring, AssetMeasure> g_asset_cache;
#endif  // AssetMeasure/g_asset_cache are dev-only; the helpers + capture below are ALWAYS-compiled
        // because the manual Page Down override (shotpoint_capture_held) needs them in release too.

// ---- THE RIG COMPOSITION, factored so the aim frame is the SAME one the barrel is drawn in.
//
// The rendered weapon's game-space orientation (mode 3, before the parent divides out and re-applies)
// is  q_ctrl * q_grip_dir , where q_ctrl comes from the CONTROLLER pose converted VR->UE with the -w
// HANDEDNESS term and q_grip_dir is the live End grip trim. shotpoint_gun_quat() reproduces exactly
// that rotation, MINUS the snap turn (callers add turn as a scalar yaw -- equivalent to the rig's
// q_turn pre-rotation, since a yaw about UE +Z just adds to atan2(y,x) and leaves asin(z) alone).
//
// Because the grip trim is applied LIVE here (never frozen), a bore captured against this frame
// follows an End grip change with no re-capture -- that is the grip-independence. And because the
// pose conversion matches the rig, aim points where the barrel VISUALLY points.
//
// ⚠️ THIS MIRRORS the rig's mode-3 pose->orientation block in Plugin.cpp (the one that builds
// g_pitch/g_yaw/g_roll from cq_2h/gqo, then q_ctrl*q_grip_dir). It is a deliberate second copy, the
// same discipline as the derive_ctrl_angles / inline-aim pair: if you change that rig block, change
// this. (rotator_to_quat is the exact inverse of quat_to_rotator per Math.hpp, so building the
// quaternion straight from the -w-swizzled components equals the rig's rotator round-trip, without
// the Euler gimbal degeneracy near vertical.)
//
// Uses the CONFIGURED AIM HAND (left or right), never a hardcoded controller.
static Quat shotpoint_gun_quat(const Quat& cq, const Quat& gq, bool have_grip,
                               const Quat& q_ro, bool two_hand) {
    Quat bcq = cq, bgq = gq;
    if (two_hand) {                       // the SAME swing the rig applies to the rendered weapon
        two_hand_bend_orientation(&bcq);
        two_hand_bend_orientation(&bgq);
    }
    // Grip pose * rotation_offset when the grip pose is available (as the rig does), else the aim
    // pose. rotation_offset is composed in VR space, before the conversion -- matching the rig.
    const Quat pose = have_grip ? quat_mul(q_ro, bgq) : bcq;
    const Quat ue{ -pose.z, pose.x, pose.y, -pose.w };   // VR -> UE, WITH handedness (the -w term)
    const Quat q_grip_dir = rotator_to_quat(g_cfg.rig_dir_grip_deg,
                                            g_cfg.rig_dir_grip_yaw,
                                            g_cfg.rig_dir_grip_roll);
    return quat_mul(ue, q_grip_dir);
}
// Samples the grip pose + rotation offset for `ridx` and composes the gun quat. `cq` is the aim pose
// the caller already sampled. The one place the rig_view_yaw sign convention on the offset lives.
static Quat shotpoint_gun_quat_live(int32_t ridx, const Quat& cq, bool two_hand) {
    Vec3 gpos{}; Quat gq{};
    const bool have_grip = get_pose(ridx, &gpos, &gq, /*use_aim=*/false);
    const auto ro = API::VR::get_rotation_offset();
    Quat q_ro{ro.x, ro.y, ro.z, ro.w};
    if (g_cfg.rig_view_yaw < 0.0f) q_ro = quat_conj(q_ro);   // mirror the rig block (Plugin.cpp)
    return shotpoint_gun_quat(cq, gq, have_grip, q_ro, two_hand);
}

// Reconstruct the frozen constant F to game angles: F is the bore in the un-driven weapon frame, so
// rotating it by the live gun quat gives the world bore, and atan2(y,x)/asin(z) (+turn) are its
// game angles. This IS the aim consumers' shared setpoint -- see the header.
bool shotpoint_aim_angles(int32_t ridx, const Quat& cq, bool two_hand,
                          float* out_yaw, float* out_pitch) {
    Vec3 F{};
    if (!shotpoint_bore_local(&F)) return false;   // no frozen constant -> caller cascades
    const Quat q_gun = shotpoint_gun_quat_live(ridx, cq, two_hand);   // game space, no turn
    const Vec3 bore  = quat_rotate(q_gun, F);
    const float turn = g_cfg.aim_turn * g_turn_offset.load();
    if (out_yaw)   *out_yaw   = wrap180(std::atan2(bore.y, bore.x) * RAD2DEG + turn);
    if (out_pitch) *out_pitch = std::asin(clampf(bore.z, -1.0f, 1.0f)) * RAD2DEG;
    return true;
}

// Capture the frozen controller-frame bore for the held weapon (force-overwrite). Self-checks by
// reconstructing at this pose; rejects a bad transform. Uses the configured aim hand (left or right).
static bool capture_bore_local(API::UObject* wpn) {
    if (wpn == nullptr) return false;
    Vec3 mpos{};
    auto* comp = weapon_marker_component(wpn, &mpos);
    if (comp == nullptr) return false;
    Vec3 bore_ue{};
    if (!call_ret_vec3(comp, L"GetForwardVector", &bore_ue)) return false;
    const float blen = std::sqrt(bore_ue.x*bore_ue.x + bore_ue.y*bore_ue.y + bore_ue.z*bore_ue.z);
    if (blen < 1e-4f) return false;
    bore_ue.x /= blen; bore_ue.y /= blen; bore_ue.z /= blen;

    const int32_t ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                             : API::VR::get_right_controller_index();
    if (ridx < 0) return false;
    Vec3 cpos{}; Quat cq{};
    if (!get_pose(ridx, &cpos, &cq, /*use_aim=*/true)) return false;

    // Freeze the bore in the weapon's UN-DRIVEN frame: divide out the rig's live composition (grip
    // pose + grip trim), and strip the snap turn, so what remains is grip-, turn- and
    // animation-free. two_hand=false -- Page Down is a deliberate single-handed calibration gesture
    // (a live swing would cancel between q_gun and the swung mesh bore anyway, but keeping it off
    // makes the stored constant unambiguous).
    const Quat q_gun = shotpoint_gun_quat_live(ridx, cq, /*two_hand=*/false);   // game space, no turn
    const float turn = g_cfg.aim_turn * g_turn_offset.load();
    const float tr = turn * DEG2RAD, ct = std::cos(tr), st = std::sin(tr);
    const Vec3 bore_nt{ bore_ue.x*ct + bore_ue.y*st, -bore_ue.x*st + bore_ue.y*ct, bore_ue.z };  // strip snap turn (UE +Z)
    const Vec3 F = quat_rotate(quat_conj(q_gun), bore_nt);

    // Self-check: reconstruct THIS freshly-computed F at THIS pose; must reproduce the live world
    // bore's game angles. A near-tautology by construction (rec == bore_nt), so its real job is to
    // reject a degenerate/NaN pose (e.g. an empty tracking pose) before it is frozen. NB: cannot use
    // shotpoint_aim_angles here -- that reads the PUBLISHED atomic (last tick's F), not this one.
    const Vec3 rec = quat_rotate(q_gun, F);
    const float rec_yaw = wrap180(std::atan2(rec.y, rec.x) * RAD2DEG + turn);
    const float rec_pit = std::asin(clampf(rec.z, -1.0f, 1.0f)) * RAD2DEG;
    const float bore_yaw = std::atan2(bore_ue.y, bore_ue.x) * RAD2DEG;
    const float bore_pit = std::asin(clampf(bore_ue.z, -1.0f, 1.0f)) * RAD2DEG;
    const float dyaw = std::fabs(wrap180(rec_yaw - bore_yaw));
    const float dpit = std::fabs(rec_pit - bore_pit);
    if (!std::isfinite(dyaw) || !std::isfinite(dpit) || dyaw > 2.0f || dpit > 2.0f) {
        API::get()->log_info("[Halo-CampE-UEVR] SHOTFIX: REJECT '%ls' -- self-check off yaw=%.2f pit=%.2f",
                             class_name_of(wpn).c_str(), dyaw, dpit);
        return false;
    }
    const std::wstring cn = class_name_of(wpn);
    g_bore_cache[cn] = F;
    API::get()->log_info("[Halo-CampE-UEVR] SHOTFIX: captured '%ls' F=(%.3f,%.3f,%.3f) at bore "
                         "yaw=%.1f pit=%.1f -- grip-independent frozen aim armed",
                         cn.c_str(), F.x, F.y, F.z, bore_yaw, bore_pit);
    // The AR is the reference weapon: its bore also becomes the DEFAULT for uncaptured weapons.
    if (cn.find(L"AssaultRifle") != std::wstring::npos) {
        g_def_x.store(F.x); g_def_y.store(F.y); g_def_z.store(F.z);
        g_def_valid.store(true);
        API::get()->log_info("[Halo-CampE-UEVR] SHOTFIX: AR captured -> DEFAULT for uncaptured weapons");
    }
    write_calib_file();   // persist (rare -- once per weapon's first stable hold, never per tick)
    return true;
}

// Public manual override (Page Down): force-capture the held weapon now. Any build.
bool shotpoint_capture_held() {
    auto* wpn = fp_weapon_actor();
    return wpn != nullptr && capture_bore_local(wpn);
}

#if HALO_VR_DEV
void shotpoint_asset_dev(unsigned tick) {
    if (g_cfg.shot_aim_log <= 0) return;

    auto* wpn = fp_weapon_actor();
    if (wpn == nullptr) return;
    Vec3 mpos{};
    auto* comp = weapon_marker_component(wpn, &mpos);
    if (comp == nullptr) return;
    Vec3 bore_ue{};
    if (!call_ret_vec3(comp, L"GetForwardVector", &bore_ue)) return;
    const float bl0 = std::sqrt(bore_ue.x*bore_ue.x + bore_ue.y*bore_ue.y + bore_ue.z*bore_ue.z);
    if (bl0 < 1e-4f) return;
    bore_ue.x /= bl0; bore_ue.y /= bl0; bore_ue.z /= bl0;

    const int32_t ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                             : API::VR::get_right_controller_index();
    if (ridx < 0) return;
    Vec3 cpos{}; Quat cq{};
    if (!get_pose(ridx, &cpos, &cq, /*use_aim=*/true)) return;

    // THE BORE IN THE WEAPON'S UN-DRIVEN FRAME (the frozen constant, sampled live). Constant while
    // the weapon rigidly tracks the hand, moving during draw/recoil/reload -- so ITS variance is the
    // right "is the player holding steady?" signal. Same math as capture_bore_local, so this window
    // measures exactly what a Page Down would freeze. (The first cut measured bore-vs-ROOT, which is
    // rigidly constant -- dev=0.00 forever -- so "stable" was always true and the capture fired
    // mid-draw at whatever pose, e.g. the Magnum at 80 deg up. That is the pistol-reticle-gone bug.)
    const Quat  q_gun = shotpoint_gun_quat_live(ridx, cq, /*two_hand=*/true);   // game space, no turn
    const float turn  = g_cfg.aim_turn * g_turn_offset.load();
    const float tr = turn * DEG2RAD, ct = std::cos(tr), st = std::sin(tr);
    const Vec3  bore_nt{ bore_ue.x*ct + bore_ue.y*st, -bore_ue.x*st + bore_ue.y*ct, bore_ue.z };
    const Vec3  bl = quat_rotate(quat_conj(q_gun), bore_nt);

    constexpr float kBoreTolDeg   = 2.0f;   // window: a sample beyond this restarts the hold
    constexpr int   kStableSamples = 30;    // ~0.9 s of CONSECUTIVE quiet at ~32 Hz

    auto ang = [](const Vec3& a, const Vec3& b) {
        const float la = std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z), lb = std::sqrt(b.x*b.x+b.y*b.y+b.z*b.z);
        if (la < 1e-4f || lb < 1e-4f) return 0.0f;
        return std::acos(clampf((a.x*b.x+a.y*b.y+a.z*b.z)/(la*lb), -1.0f, 1.0f)) * RAD2DEG;
    };

    AssetMeasure& m = g_asset_cache[class_name_of(wpn)];

    // Reset-on-motion window on the un-driven-frame bore: a sample beyond tolerance ends the hold
    // and restarts, so draw/recoil/reload/sway just keep restarting the counter and only a genuine
    // steady hold (weapon rigidly tracking the hand) reaches STABLE.
    bool broke = false;
    if (m.n > 0) {
        const float bdev = ang(bl, m.bore_mean);
        if (bdev > kBoreTolDeg) broke = true;
        else if (bdev > m.bore_dev_max) m.bore_dev_max = bdev;
    }
    if (m.n == 0 || broke) {
        m.bore_mean = bl; m.n = 1; m.bore_dev_max = 0.0f; m.logged_stable = false;
    } else {
        ++m.n;
        m.bore_mean.x += (bl.x - m.bore_mean.x) / m.n;
        m.bore_mean.y += (bl.y - m.bore_mean.y) / m.n;
        m.bore_mean.z += (bl.z - m.bore_mean.z) / m.n;
    }
    const bool stable = (m.n >= kStableSamples);

    // AUTO-CAPTURE SCRAPPED (user, 2026-09-12): the reset-on-motion heuristic never captured
    // reliably after a cfg change (mostly aimed too high) and was an overcomplication. Capture is
    // now MANUAL ONLY (Page Down / shotpoint_capture_held). This window + the SHOTASSET line below
    // stay purely as a dev diagnostic -- nothing here writes the cache.

    // Log + self-verify at the throttle: recompose the window-mean bore with the CURRENT pose and
    // compare to the LIVE world bore. ~0 at a steady hold confirms the frozen model tracks the hand;
    // it also shows the recapture converging back to ~0 after an End change.
    if ((tick % (unsigned)g_cfg.shot_aim_log) == 0 || (stable && !m.logged_stable)) {
        if (stable) m.logged_stable = true;
        const Vec3  rec = quat_rotate(q_gun, m.bore_mean);   // window-mean F, current pose
        const float ry  = wrap180(std::atan2(rec.y, rec.x) * RAD2DEG + turn);
        const float rp  = std::asin(clampf(rec.z, -1.0f, 1.0f)) * RAD2DEG;
        const float live_yaw = std::atan2(bore_ue.y, bore_ue.x) * RAD2DEG;
        const float live_pit = std::asin(clampf(bore_ue.z, -1.0f, 1.0f)) * RAD2DEG;
        API::get()->log_info(
            "[Halo-CampE-UEVR] SHOTASSET: '%ls' n=%d %s | F=(%.3f,%.3f,%.3f) dev=%.2fdeg | "
            "recompose-vs-live dyaw=%.2f dpit=%.2f",
            class_name_of(wpn).c_str(), m.n, stable ? "STABLE" : "settling",
            m.bore_mean.x, m.bore_mean.y, m.bore_mean.z, m.bore_dev_max,
            wrap180(ry - live_yaw), rp - live_pit);
    }
}
#endif

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
