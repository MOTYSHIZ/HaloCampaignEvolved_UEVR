#include "Arms.hpp"

#include "BlamDrive.hpp"     // ADDITION: unit position/mounted -- the driver hide and hog resolve anchor on them
#include "Config.hpp"
#include "DevTools.hpp"
#include "Markers.hpp"       // ADDITION: g_cam_* -- the rendered eye, for the driver-body enumeration log
#include "Rig.hpp"
#include "UeObject.hpp"
#include "BlamPalette.hpp"
#include "ArmDriver.hpp"     // palette_weapon_mode(): the FP-build hold-off applies to mode 3 only

#include <windows.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

using uevr::API;

namespace halo {
namespace {

// A skeleton with more bones than this is not this game's first-person arms, it is a bad read.
// Bounded so a garbage GetNumBones cannot spin the game thread.
constexpr int32_t kMaxBones = 512;

// UE TArray header -- same shape Rig.cpp reads for AttachChildren / BlueprintCreatedComponents.
struct FRawArrayRO { void* data; int32_t num; int32_t max; };

// Bone functions the hybrid plan needs. Probed rather than assumed -- see the header.
const wchar_t* const kProbe[] = {
    L"GetNumBones",              // enumeration
    L"GetBoneName",
    L"GetParentBone",
    L"HideBoneByName",           // the hybrid plan's core call
    L"UnHideBoneByName",
    L"IsBoneHidden",
    L"GetBoneLocation",          // needed if melee is to read the real hand bone
    L"GetSocketLocation",
    L"GetAllSocketNames",
    L"SetBoneTransformByName",   // only route B needs this; probed to know if B is even open
    L"K2_AttachToComponent",     // needed to hang our own left hand off something
    // Whole-component hiding -- the blunt fallback if per-bone hiding is inert. The weapon is a
    // SEPARATE ACTOR socket-attached to this mesh, so hiding the mesh should leave the gun.
    L"SetVisibility",
    L"SetHiddenInGame",
    L"SetRenderInMainPass",
    L"SetOwnerNoSee",
    // Spawning and placing our own hands.
    L"K2_SetWorldLocation",
    L"K2_SetWorldRotation",
    L"SetRelativeScale3D",
    // Does hiding stop the pose evaluating? UE skeletal meshes default to
    // OnlyTickPoseWhenRendered, and Rig.cpp positions the weapon from this mesh's PrimaryWeapon
    // SOCKET -- a frozen pose means a frozen socket and a gun in the wrong place.
    L"SetVisibilityBasedAnimTickOption",
    L"SetForcedLOD",
};

// GetNumBones() -> int32. No parameters, so the return lands at offset 0.
bool call_get_num_bones(API::UObject* comp, int32_t* out) {
    if (comp == nullptr || out == nullptr) return false;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    comp->call_function(L"GetNumBones", p);
    const int32_t n = *reinterpret_cast<int32_t*>(p);
    if (n < 0 || n > kMaxBones) return false;
    *out = n;
    return true;
}

// GetBoneName(int32 BoneIndex) -> FName. int32 in at 0; FName is {int32,int32} and 4-aligned, so
// the return sits immediately after at offset 4.
bool call_get_bone_name(API::UObject* comp, int32_t index, std::wstring* out) {
    if (comp == nullptr || out == nullptr) return false;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    *reinterpret_cast<int32_t*>(p) = index;
    comp->call_function(L"GetBoneName", p);
    auto* nm = reinterpret_cast<API::FName*>(p + 4);
    *out = nm->to_string();
    return !out->empty();
}

// GetParentBone(FName BoneName) -> FName. FName in at 0 (8 bytes), FName out at 8.
bool call_get_parent_bone(API::UObject* comp, const wchar_t* bone, std::wstring* out) {
    if (comp == nullptr || bone == nullptr || out == nullptr) return false;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    // make_fname, NOT API::FName -- the latter resolves to None on this build, and a None lookup
    // fails QUIETLY with a plausible answer (the trap call_socket_location documents).
    API::FName in = make_fname(bone);
    memcpy(p, &in, sizeof(int32_t) * 2);
    comp->call_function(L"GetParentBone", p);
    auto* nm = reinterpret_cast<API::FName*>(p + 8);
    *out = nm->to_string();
    return !out->empty();
}

// Log every class name in one of the pawn's component arrays. This is how we find out what ELSE
// is posed by the first-person anim blueprint: the shield shell runs its own instance of it, so a
// bone hidden on the arms but not on the shell would leave a floating shield arm.
void dump_component_array(API::UObject* owner, const wchar_t* prop) {
    // DEV-ONLY BODY -- bulk reflection dump; see arms_dump_skeleton below.
    (void)owner; (void)prop;
#if HALO_VR_DEV
    if (owner == nullptr) return;
    auto* arr = owner->get_property_data<FRawArrayRO>(prop);
    if (arr == nullptr || IsBadReadPtr(arr, sizeof(FRawArrayRO))) return;
    if (arr->data == nullptr || arr->num <= 0 || arr->num > 4096) return;

    auto** elems = reinterpret_cast<API::UObject**>(arr->data);
    if (IsBadReadPtr(elems, sizeof(void*) * (size_t)arr->num)) return;

    API::get()->log_info("[Halo-CampE-UEVR] BONEDUMP   %ls: %d entries", prop, arr->num);
    for (int32_t i = 0; i < arr->num; ++i) {
        auto* c = elems[i];
        if (c == nullptr || IsBadReadPtr(c, sizeof(void*))) continue;
        const std::wstring cn = class_name_of(c);
        if (cn.empty()) continue;
        // Flag the skeletal meshes: those are the ones an AnimBP poses, and therefore the ones a
        // bone hide would have to be applied to consistently.
        const bool skel = cn.find(L"SkeletalMesh") != std::wstring::npos;
        API::get()->log_info("[Halo-CampE-UEVR] BONEDUMP     [%2d] %ls%s",
                             i, cn.c_str(), skel ? "   <-- SKELETAL" : "");
    }
#endif
}

// ---------------------------------------------------------------- LEFT ARM HIDE
//
// The three first-person meshes that carry the anim blueprint's pose, per the bone dump. The
// arms mesh alone is not enough: the shield shell runs its own instance of the same anim
// blueprint, and the shadow mesh casts a silhouette, so hiding one leaves the others' left arms.
const wchar_t* const kFpMeshClasses[] = {
    L"BPC_FP_SkeletalMesh_C",
    L"BPC_FP_TranslucentSkeletalMesh_C",
    L"BPC_FP_ShadowSkeletalMesh_C",
};
constexpr int kFpMeshCount = (int)(sizeof(kFpMeshClasses) / sizeof(kFpMeshClasses[0]));

// Every first-person MESH component on the pawn, not just the skeletal ones.
//
// Hiding the three skeletal meshes left the armour floating: the bone dump listed six
// BPC_FP_StaticMesh_C components as well, which are the shoulder plates, elbow guards and
// gauntlets parented to arm bones. They are separate components, so a skeletal-mesh hide does not
// touch them.
//
// The test is the BPC_FP_ prefix plus "Mesh". The prefix is what keeps this away from the plugin's
// OWN components -- the reticule's plain StaticMeshComponent and WidgetComponent also live on this
// pawn, and hiding those would erase the aim reticule along with the arms.
bool is_fp_mesh_class(const std::wstring& cn) {
    return cn.rfind(L"BPC_FP_", 0) == 0 && cn.find(L"Mesh") != std::wstring::npos;
}

// Room for every FP mesh with headroom. The dump found nine; 32 is generous and bounded.
constexpr int kMaxHidden = 32;

// What we have hidden, and on what. IsBoneHidden is ABSENT on this build, so the engine cannot be
// asked -- this is the only record that exists.
TrackedObject s_hidden_on[kMaxHidden];
int           s_hidden_count = 0;
char          s_hidden_bone[64] = "";
// The mode ACTUALLY used, so release undoes what was done rather than what the config now says.
int           s_hidden_mode = -1;
bool          s_any_hidden = false;

// Find a component of the pawn by EXACT class name. Exact, not prefix: BPC_FP_SkeletalMesh_C and
// BPC_FP_ShadowSkeletalMesh_C share a prefix, and Rig.cpp already warns that matching loosely
// here picks up the wrong mesh.
API::UObject* find_pawn_component(const wchar_t* cls) {
    auto* pawn = API::get()->get_local_pawn(0);
    if (pawn == nullptr) return nullptr;
    for (const wchar_t* prop : {L"BlueprintCreatedComponents", L"InstanceComponents"}) {
        auto* arr = pawn->get_property_data<FRawArrayRO>(prop);
        if (arr == nullptr || IsBadReadPtr(arr, sizeof(FRawArrayRO))) continue;
        if (arr->data == nullptr || arr->num <= 0 || arr->num > 4096) continue;
        auto** elems = reinterpret_cast<API::UObject**>(arr->data);
        if (IsBadReadPtr(elems, sizeof(void*) * (size_t)arr->num)) continue;
        for (int32_t i = 0; i < arr->num; ++i) {
            auto* c = elems[i];
            if (c == nullptr || IsBadReadPtr(c, sizeof(void*))) continue;
            if (class_name_of(c) == cls) return c;
        }
    }
    return nullptr;
}

// HideBoneByName(FName BoneName, EPhysBodyOp PhysBodyOption).
// FName in at 0 (8 bytes); the enum is a uint8 at 8. PBO_None = 0 -- hide the bone WITHOUT
// terminating its physics body, which is the conservative choice: we want it invisible, not
// structurally removed from a skeleton the game is still simulating.
void call_hide_bone(API::UObject* comp, const wchar_t* bone) {
    if (comp == nullptr || bone == nullptr) return;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    API::FName n = make_fname(bone);
    memcpy(p, &n, sizeof(int32_t) * 2);
    p[8] = 0;   // PBO_None
    comp->call_function(L"HideBoneByName", p);
}

// UnHideBoneByName(FName BoneName). FName in at 0.
void call_unhide_bone(API::UObject* comp, const wchar_t* bone) {
    if (comp == nullptr || bone == nullptr) return;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    API::FName n = make_fname(bone);
    memcpy(p, &n, sizeof(int32_t) * 2);
    comp->call_function(L"UnHideBoneByName", p);
}

// SetVisibility(bool bNewVisibility, bool bPropagateToChildren).
//
// The blunt fallback. bPropagateToChildren is FALSE deliberately: the weapon actor is attached to
// this mesh at socket PrimaryWeapon, and propagating would take the gun with the arms. We want the
// arms gone and the weapon left exactly where it is.
void call_set_visibility(API::UObject* comp, bool visible) {
    if (comp == nullptr) return;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    p[0] = visible ? 1 : 0;
    p[1] = 0;   // bPropagateToChildren = false -- keep the weapon
    comp->call_function(L"SetVisibility", p);
}

// The pose-keeping call used to be duplicated here. It now lives in Rig.cpp as
// rig_set_always_tick_pose() -- see Rig.hpp. The duplicate is why the two hide paths diverged:
// this one kept hidden arms animating and the default one (Plugin.cpp's hide_arms / showarms=0)
// did not, so the weapon lost its recoil for everyone who never enabled armhide.

// SetHiddenInGame(bool bNewHidden, bool bPropagateToChildren). Same propagation reasoning.
void call_set_hidden(API::UObject* comp, bool hidden) {
    if (comp == nullptr) return;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    p[0] = hidden ? 1 : 0;
    p[1] = 0;
    comp->call_function(L"SetHiddenInGame", p);
}

// Sweep every first-person mesh component and apply one call to each.
//
// NO TRACKING LIST. There was one, and it was the bug: components are recreated without the pawn
// changing -- a weapon swap spawns new attachment meshes -- so the list accumulated a generation
// per swap. One session logged 36 hides across a single pawn replacement and filled the 32-entry
// cap, after which nothing new was hidden at all. That is what put the meshes back on a checkpoint
// load.
//
// The list only ever existed so release knew what to undo. But release can sweep exactly the same
// set and call the opposite function, so the list bought nothing and cost a whole class of
// staleness bugs: dead handles, cap exhaustion, and a pawn-identity check to paper over both.
//
// Returns how many components it touched.
int sweep_fp_meshes(bool hide) {
    auto* pawn = API::get()->get_local_pawn(0);
    if (pawn == nullptr) return 0;

    // COMMA-SEPARATED bone list (fork addition). A single name behaves exactly as before; the palette
    // weapon presentation passes "Shoulder_L,Shoulder_R" so both arms hide by bone while the weapon
    // branch stays drawn. Everything at or below space is trimmed: a CRLF file leaves '' on the last
    // token, and FName Add-mode would CREATE that bogus name and hide nothing.
    std::wstring bones[8];
    int nbones = 0;
    {
        const char* s2 = g_cfg.arm_hide_bone;
        while (*s2 != 0 && nbones < 8) {
            const char* e = s2;
            while (*e != 0 && *e != ',') ++e;
            std::string one(s2, e);
            while (!one.empty() && (unsigned char)one.back() <= ' ') one.pop_back();
            while (!one.empty() && (unsigned char)one.front() <= ' ') one.erase(one.begin());
            if (!one.empty()) bones[nbones++] = std::wstring(one.begin(), one.end());
            s2 = (*e == ',') ? e + 1 : e;
        }
    }
    // Mode 3 (weapon-only, fork addition, only when armhidemode=3): the FP pawn is MODULAR -- armour
    // pieces are separate skeletal-mesh components of the same classes, and an arm bone name is a
    // silent no-op on them. So whole-hide every swept component EXCEPT the one the rig tracks (it
    // carries the weapon bones), and bone-hide the arm list on that one.
    const int mode = hide ? g_cfg.arm_hide_mode : s_hidden_mode;
    API::UObject* keep = (mode == 3) ? rig_tracked_component() : nullptr;
    int n = 0;

    for (const wchar_t* prop : {L"BlueprintCreatedComponents", L"InstanceComponents"}) {
        auto* arr = pawn->get_property_data<FRawArrayRO>(prop);
        if (arr == nullptr || IsBadReadPtr(arr, sizeof(FRawArrayRO))) continue;
        if (arr->data == nullptr || arr->num <= 0 || arr->num > 4096) continue;
        auto** elems = reinterpret_cast<API::UObject**>(arr->data);
        if (IsBadReadPtr(elems, sizeof(void*) * (size_t)arr->num)) continue;

        for (int32_t i = 0; i < arr->num; ++i) {
            auto* comp = elems[i];
            if (comp == nullptr || IsBadReadPtr(comp, sizeof(void*))) continue;
            const std::wstring cn = class_name_of(comp);
            if (!is_fp_mesh_class(cn)) continue;
            if (!g_cfg.arm_hide_all && cn != L"BPC_FP_SkeletalMesh_C") continue;

            // Before hiding, make sure the pose will still evaluate. Order matters only in that
            // it must be set at all; doing it first means the mesh is never briefly hidden with
            // the default tick option in force.
            if (hide && g_cfg.arm_keep_pose && cn.find(L"SkeletalMesh") != std::wstring::npos) {
                rig_set_always_tick_pose(comp);
            }
            switch (mode) {
                case 1:  call_set_visibility(comp, !hide);          break;
                case 2:  call_set_hidden(comp, hide);               break;
                case 3:  if (comp == keep) {
                             for (int bi = 0; bi < nbones; ++bi) {
                                 if (hide) call_hide_bone(comp, bones[bi].c_str());
                                 else      call_unhide_bone(comp, bones[bi].c_str());
                             }
                             if (!hide) call_set_hidden(comp, false);
                         } else {
                             call_set_hidden(comp, hide);
                         }
                         break;
                default: for (int bi = 0; bi < nbones; ++bi) {
                             if (hide) call_hide_bone(comp, bones[bi].c_str());
                             else      call_unhide_bone(comp, bones[bi].c_str());
                         }
                         break;
            }
            ++n;
        }
    }
    return n;
}

} // namespace

void arms_release_hide() {
    if (!s_any_hidden) return;
    const int n = sweep_fp_meshes(/*hide=*/false);
    API::get()->log_info("[Halo-CampE-UEVR] ARMHIDE released mode %d on %d component(s)",
                         s_hidden_mode, n);
    s_any_hidden = false;
    s_hidden_bone[0] = 0;
    s_hidden_mode = -1;
}

void arms_dump_skeleton(API::UObject* rig) {
    // DEV-ONLY BODY. A bulk reflection dump of every first-person bone and component -- exactly
    // what DevTools.hpp says must not exist in a player build, and a config flag is not a
    // sufficient guard for it. PR #7 shipped this behind `bonedump` alone.
    (void)rig;
#if HALO_VR_DEV
    if (rig == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] BONEDUMP: no rig component");
        return;
    }

    API::get()->log_info("[Halo-CampE-UEVR] ===== BONEDUMP on %s =====",
                         narrow(class_name_of(rig)).c_str());

    // ---- WHAT ELSE WEARS THIS POSE.
    // The hybrid plan hides the left arm. If the translucent shield shell and the shadow mesh run
    // the same anim blueprint -- Rig.cpp says the shell does -- then hiding the bone on only one
    // of them leaves the other's left arm visible. Enumerate them now rather than discover it in
    // the headset.
    if (auto* pawn = API::get()->get_local_pawn(0)) {
        dump_component_array(pawn, L"BlueprintCreatedComponents");
        dump_component_array(pawn, L"InstanceComponents");
    } else {
        API::get()->log_info("[Halo-CampE-UEVR] BONEDUMP   no local pawn, skipping component list");
    }

    // ---- WHICH BONE FUNCTIONS DOES THIS BUILD ACTUALLY EXPOSE.
    // find_function is a lookup on the class, not a call, so this cannot perturb the game.
    if (auto* cls = rig->get_class()) {
        for (const wchar_t* fn : kProbe) {
            const bool have = (cls->find_function(fn) != nullptr);
            API::get()->log_info("[Halo-CampE-UEVR] BONEDUMP   fn %-26ls %s",
                                 fn, have ? "PRESENT" : "-- absent");
        }
    } else {
        API::get()->log_info("[Halo-CampE-UEVR] BONEDUMP   class unavailable, cannot probe functions");
    }

    // ---- THE SKELETON.
    int32_t n = 0;
    if (!call_get_num_bones(rig, &n)) {
        API::get()->log_info("[Halo-CampE-UEVR] BONEDUMP   GetNumBones failed or returned nonsense "
                             "-- enumeration unavailable, the hybrid plan needs another route");
        return;
    }
    API::get()->log_info("[Halo-CampE-UEVR] BONEDUMP   %d bones", n);

    int32_t n_left = 0, n_right = 0;
    for (int32_t i = 0; i < n; ++i) {
        std::wstring bone;
        if (!call_get_bone_name(rig, i, &bone)) continue;

        std::wstring parent;
        call_get_parent_bone(rig, bone.c_str(), &parent);

        // Side tally: this skeleton uses a <Part>_R / <Part>_L convention (Wrist_R, Wrist_L,
        // ElbowPart2_R, ShoulderArmor_R are the sockets log_pivot_candidates probes). Counting
        // both sides tells us at a glance whether the left chain is as complete as the right --
        // if it is not, hiding it is a very different job.
        const bool is_l = bone.size() > 2 && bone.compare(bone.size() - 2, 2, L"_L") == 0;
        const bool is_r = bone.size() > 2 && bone.compare(bone.size() - 2, 2, L"_R") == 0;
        if (is_l) ++n_left;
        if (is_r) ++n_right;

        API::get()->log_info("[Halo-CampE-UEVR] BONEDUMP   [%3d] %-32ls parent=%ls%s",
                             i, bone.c_str(),
                             parent.empty() ? L"<none>" : parent.c_str(),
                             is_l ? "   <-- LEFT" : (is_r ? "   <-- right" : ""));
    }

    API::get()->log_info("[Halo-CampE-UEVR] BONEDUMP   sided bones: %d left, %d right", n_left, n_right);
    API::get()->log_info("[Halo-CampE-UEVR] ===== BONEDUMP end =====");
#endif
}

// Forward declaration: the hide runs every tick, the dump only when armed.
void arms_hide_update();

void arms_update() {
    // ARM on the rising edge, FIRE when the rig actually exists.
    //
    // The first version dumped on the config edge itself, which is unusable in practice: the rig
    // does not exist in menus, in vehicles, during cutscenes, or for ~5 s after a level load, so
    // setting the key before launching just logged "no rig component" at the main menu and
    // disarmed itself. Arming and waiting means the key can be set any time -- including before
    // the game starts -- and the dump lands the moment you are in gameplay holding a weapon.
    static bool s_prev  = false;
    static bool s_armed = false;
    static uint32_t s_wait_ticks = 0;

    // Hide state is reconciled EVERY tick, before the dump: it must re-apply after a level load
    // rebuilds the components, and must release the instant the kill switch or armhide goes off.
    arms_hide_update();

    const bool want = g_cfg.bone_dump;
    if (want && !s_prev) {
        s_armed = true;
        s_wait_ticks = 0;
        API::get()->log_info("[Halo-CampE-UEVR] BONEDUMP armed -- waiting for the first-person rig "
                             "(get into gameplay with a weapon drawn)");
    }
    s_prev = want;

    if (!s_armed) return;

    if (auto* rig = rig_tracked_component()) {
        s_armed = false;
        arms_dump_skeleton(rig);
        return;
    }

    // Heartbeat while waiting, so an armed-but-silent dump is distinguishable from a broken one.
    // ~32 Hz tick, so 320 ticks is roughly every 10 s.
    if (++s_wait_ticks % 320 == 0) {
        API::get()->log_info("[Halo-CampE-UEVR] BONEDUMP still armed, no rig yet "
                             "(menus, vehicles and cutscenes have none)");
    }
}

void arms_hide_update() {
    // The kill switch reaches this too. An invisible arm is exactly the state Config.cpp insists a
    // person in a headset must always be able to get out of.
    const bool want = g_cfg.enabled && g_cfg.arm_hide;

    if (!want) { arms_release_hide(); return; }

    // A changed mode or bone must undo the OLD one first, using the mode that was actually
    // applied -- otherwise switching leaves the previous hide in place with nothing that knows how
    // to reverse it.
    if (s_any_hidden && (s_hidden_mode != g_cfg.arm_hide_mode ||
                         strcmp(s_hidden_bone, g_cfg.arm_hide_bone) != 0)) {
        API::get()->log_info("[Halo-CampE-UEVR] ARMHIDE settings changed, releasing first");
        arms_release_hide();
    }

    // ADDITION -- HOLD OFF WHILE THE FP BUILD IS DARK, AND FOR TWO SECONDS AFTER IT RETURNS.
    // Bisected 2026-08-26: with the sweep running, every respawn froze the palette-driven
    // PrimaryWeapon socket at the stock pose ~1 s in -- the hide lands on the freshly rebuilt
    // components mid-initialization and the FP palette sync never binds, so the weapon actor
    // rides the camera-parented mesh at its stock socket ("attached to the rig"). The same death
    // with armhide=0 tracks perfectly, and the pre-death sweep on settled components never hurt
    // anything.
    //
    // A pawn-identity watch was the first cut and it MISSED: this build reuses the pawn object
    // across a death and rebuilds only its COMPONENTS (the respawn logged a new rig component
    // and no pawn change). The one signal that reliably goes dark at every rebuild window is
    // the FP palette build itself -- it stops within a frame on death, seats and cutscenes,
    // which are exactly the moments components get torn down. So the sweep runs only once the
    // build has been back for ~2 s, and a dark spell forgets the hide state (the components it
    // covered are being torn down; releasing would sweep whatever replaced them).
    {
        static uint32_t s_hold = 0;
        // Only while the palette weapon (armdriver mode 3) owns placement: under the author's arm
        // drivers the palette build stamp never runs, and this gate would hold the hide off forever.
        if (palette_weapon_mode() && !blam_palette_fp_live()) {
            if (s_hold == 0 && s_any_hidden) {
                API::get()->log_info("[Halo-CampE-UEVR] ARMHIDE: FP build went dark -- holding "
                                     "the sweep until it is back ~2 s");
            }
            s_hold = 64;   // ~2 s at the ~32 Hz tick, restarted while dark
            s_any_hidden = false;
            return;
        }
        if (s_hold != 0) { --s_hold; return; }
    }

    // Only per-bone hiding needs the rig, and only to target the component the plugin provably
    // drives. Modes 1 and 2 sweep by class and never needed it -- applying the gate to them is
    // what stopped the re-assert after a weapon swap, since Rig.hpp records that a swap kills the
    // route.
    if ((g_cfg.arm_hide_mode == 0 || g_cfg.arm_hide_mode == 3) && rig_tracked_component() == nullptr) return;

    // Roughly 8 Hz. Fast enough that a checkpoint or respawn shows the meshes for ~125 ms rather
    // than the ~500 ms a half-second cadence gave, slow enough that the calls stay off the
    // per-frame budget the 0.2.0 release worked to reclaim.
    static uint32_t s_tick = 0;
    if ((++s_tick % 4) != 0) return;

    const int n = sweep_fp_meshes(/*hide=*/true);
    if (n == 0) return;

    if (!s_any_hidden) {
        s_any_hidden = true;
        s_hidden_mode = g_cfg.arm_hide_mode;
        strncpy_s(s_hidden_bone, sizeof(s_hidden_bone), g_cfg.arm_hide_bone, _TRUNCATE);
        API::get()->log_info("[Halo-CampE-UEVR] ARMHIDE applied mode %d to %d component(s)",
                             s_hidden_mode, n);
    }

    // Heartbeat with the live count, so a changing number of components is visible without a line
    // per component per pass.
    static int s_last_n = -1;
    if (n != s_last_n) {
        s_last_n = n;
        API::get()->log_info("[Halo-CampE-UEVR] ARMHIDE now covering %d FP mesh component(s)", n);
    }
}

// ================================================================================================
// ADDITIONS. Everything above this line is the upstream Arms.cpp as it stands there, apart from
// three marked includes and the marked ARMHIDE hold-off inside arms_hide_update. What follows is
// the vehicle body work: the driver-body hide while mounted, and the Warthog hull resolve that the
// rigid vehicle camera in Vehicle.cpp reads. Declared at the tail of Arms.hpp.
// ================================================================================================

namespace {

// SetRelativeScale3D(FVector NewScale3D). LWC: three DOUBLES, not floats -- Hands.cpp pays for
// that distinction already and getting it wrong writes garbage into the first two components.
void call_set_scale(API::UObject* comp, double sc) {
    if (comp == nullptr) return;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    auto* d = reinterpret_cast<double*>(p);
    d[0] = d[1] = d[2] = sc;
    comp->call_function(L"SetRelativeScale3D", p);
}

// ---------------------------------------------------------------- DRIVER BODY HIDE
//
// WHAT IT IS, measured 2026-08-21 by a ranked sweep around the rendered eye:
//
//   BP_SpartansBipedActor_C_<id>.Body     SkeletalMeshComponent, SK_Spartans_AnimDynamics
//
// That actor is the player. With the camera at the seat your head is inside it, so it clips
// constantly. MATCHED BY OUTER CHAIN, NOT BY CLASS: the class is plain "SkeletalMeshComponent",
// shared with every marine, weapon and NPC in the level. The full name carries the owning actor,
// so "SpartansBipedActor" + ".Body" is the thing that actually identifies it; the instance id
// changes per load, so it is deliberately not part of the match.
bool is_driver_body(const std::wstring& full) {
    return full.find(L"SpartansBipedActor") != std::wstring::npos
        && full.size() >= 5 && full.compare(full.size() - 5, 5, L".Body") == 0;
}

// Pull "BP_SpartansBipedActor_C_<id>" out of a full name. The instance id changes every load,
// so the ACTOR TOKEN has to be read from a component we already matched.
std::wstring driver_actor_token(const std::wstring& full) {
    const size_t k = full.find(L"SpartansBipedActor");
    if (k == std::wstring::npos) return L"";
    const size_t start = full.rfind(L'.', k);
    const size_t end = full.find(L'.', k);
    if (end == std::wstring::npos) return L"";
    const size_t from = (start == std::wstring::npos) ? 0 : start + 1;
    if (end <= from) return L"";
    return full.substr(from, end - from);
}

// EVERY mesh component on the driver actor, not just .Body. These are modular characters (the
// marines nearby are SIX components each), so the Spartan is one too and .Body is one piece.
constexpr int kMaxDriverParts = 24;
TrackedObject s_driver_parts[kMaxDriverParts];
int           s_driver_part_count = 0;
bool          s_driver_hidden = false;
int           s_driver_tries = 0; ULONGLONG s_driver_try_at = 0;

// Resolve the driver actor, then collect its parts. Enumerates every Spartan body with distance
// from BOTH references and SELECTS ON d_blam: the Blam unit position is the player's biped by
// definition, while the eye is only meaningful after the vehicle camera has run -- and at the
// mount edge it has not. (Selecting on the eye once picked a Spartan 847 cm away.)
int resolve_driver_parts() {
    s_driver_part_count = 0;
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return 0;
    const double ex = (double)g_cam_x.load(std::memory_order_relaxed);
    const double ey = (double)g_cam_y.load(std::memory_order_relaxed);
    const double ez = (double)g_cam_z.load(std::memory_order_relaxed);
    const double S = 304.8;
    const double bx =  (double)g_unit_px.load(std::memory_order_relaxed) * S;
    const double by = -(double)g_unit_py.load(std::memory_order_relaxed) * S;
    const double bz =  (double)g_unit_pz.load(std::memory_order_relaxed) * S;
    API::get()->log_info("[Halo-CampE-UEVR] VEHBODY: eye=(%.0f %.0f %.0f) blam=(%.0f %.0f %.0f)",
                         ex, ey, ez, bx, by, bz);
    const int32_t n = arr->get_object_count();
    std::wstring token; double bestd = 1e18;
    int spartans = 0;
    for (int32_t i = 0; i < n; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"SkeletalMeshComponent") continue;
        const std::wstring full = o->get_full_name();
        if (!is_driver_body(full)) continue;
        Vec3 w{};
        if (!call_ret_vec3(o, L"K2_GetComponentLocation", &w)) continue;
        const double de = std::sqrt(((double)w.x - ex) * ((double)w.x - ex)
                                  + ((double)w.y - ey) * ((double)w.y - ey)
                                  + ((double)w.z - ez) * ((double)w.z - ez));
        const double db = std::sqrt(((double)w.x - bx) * ((double)w.x - bx)
                                  + ((double)w.y - by) * ((double)w.y - by)
                                  + ((double)w.z - bz) * ((double)w.z - bz));
        ++spartans;
        API::get()->log_info("[Halo-CampE-UEVR] VEHBODY   spartan d_eye=%8.1f d_blam=%8.1f  %ls",
                             de, db, full.c_str());
        if (db < bestd) { bestd = db; token = driver_actor_token(full); }
    }
    API::get()->log_info("[Halo-CampE-UEVR] VEHBODY: %d spartan bodies in the level", spartans);
    if (token.empty()) return 0;
    for (int32_t i = 0; i < n && s_driver_part_count < kMaxDriverParts; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        const std::wstring cn = class_name_of(o);
        if (cn.find(L"Mesh") == std::wstring::npos) continue;
        if (cn.find(L"Component") == std::wstring::npos) continue;
        const std::wstring full = o->get_full_name();
        if (full.find(token) == std::wstring::npos) continue;
        s_driver_parts[s_driver_part_count++].set_at(o, i);
    }
    API::get()->log_info("[Halo-CampE-UEVR] VEHBODY: chose %ls at d_blam=%.1f -- %d mesh parts",
                         token.c_str(), bestd, s_driver_part_count);
    return s_driver_part_count;
}

} // namespace

// Reconciled EVERY TICK while mounted, not once on the transition: a component whose whole job
// is to drive this mesh from the Blam simulation is exactly the kind of thing that reasserts
// state underneath us, and re-applying every tick beats guessing.
//
// SCALE TO NOTHING, as well as hiding. The readback settled that SetHiddenInGame TAKES on this
// component (hid=1, held, nothing reverting it) and the Spartan still drew -- the mesh is drawn
// by something that does not consult UE visibility. A transform is not a visibility flag: the
// draw demonstrably honours scale, so 0.001 is what actually removes the body (vehhidebody=2).
void driver_hide_update() {
    // The body is hidden because the SEAT CAMERA sits inside it. With vehcam off the view is the
    // stock chase camera, where hiding it just deletes the Spartan from the shot.
    const bool want = g_cfg.enabled && g_cfg.veh_hide_body != 0 && g_cfg.veh_cam != 0
                   && g_unit_mounted.load(std::memory_order_relaxed);

    if (!want) {
        if (s_driver_hidden) {
            for (int i = 0; i < s_driver_part_count; ++i) {
                if (auto* c = s_driver_parts[i].get()) {
                    call_set_hidden(c, false);
                    call_set_visibility(c, true);
                    call_set_scale(c, 1.0);   // unconditional: restore whatever mode did
                }
            }
            API::get()->log_info("[Halo-CampE-UEVR] VEHBODY: driver restored (%d parts)",
                                 s_driver_part_count);
            s_driver_part_count = 0;
            s_driver_hidden = false;
        }
        s_driver_tries = 0;
        return;
    }

    if (!s_driver_hidden) {
        // Two full object-array walks per try: every 2 s, ten tries per mount, then it gives up
        // until the next mount (it retried every tick before, perf audit 2026-09-06).
        if (s_driver_tries >= 10) return;
        const ULONGLONG t = GetTickCount64();
        if (t - s_driver_try_at < 2000) return;
        s_driver_try_at = t; ++s_driver_tries;
        if (resolve_driver_parts() == 0) return;      // not resolvable yet; retry in 2 s
        s_driver_tries = 0;
        s_driver_hidden = true;
        API::get()->log_info("[Halo-CampE-UEVR] VEHBODY: driver hidden (%d parts)",
                             s_driver_part_count);
    }

    for (int i = 0; i < s_driver_part_count; ++i) {
        auto* c = s_driver_parts[i].get();
        if (c == nullptr) continue;
        rig_set_always_tick_pose(c);
        call_set_hidden(c, true);
        call_set_visibility(c, false);
        if (g_cfg.veh_hide_body == 2) call_set_scale(c, 0.001);
    }
    // Mode 2 -> 1 mid-ride: put the scale back once, or the parts stay shrunk under mode 1.
    {
        static int s_last_mode = 0;
        if (s_last_mode == 2 && g_cfg.veh_hide_body != 2) {
            for (int i = 0; i < s_driver_part_count; ++i)
                if (auto* c = s_driver_parts[i].get()) call_set_scale(c, 1.0);
        }
        s_last_mode = g_cfg.veh_hide_body;
    }

    // One readback a second, so the log answers "did it take" without inference.
    if (g_cfg.veh_log) {
        static uint32_t n = 0;
        if ((n++ % 90u) == 0u) {
            for (int i = 0; i < s_driver_part_count; ++i) {
                auto* c = s_driver_parts[i].get();
                if (c == nullptr) continue;
                int vis = -1, hid = -1;
                if (auto* v = c->get_property_data<bool>(L"bVisible")) vis = *v ? 1 : 0;
                if (auto* h = c->get_property_data<bool>(L"bHiddenInGame")) hid = *h ? 1 : 0;
                API::get()->log_info("[Halo-CampE-UEVR] VEHBODY   readback vis=%d hid=%d  %ls",
                                     vis, hid, c->get_full_name().c_str());
            }
        }
    }
}

// ---------------------------------------------------------------- HOG BODY RESOLVE
//
// The rigid vehicle camera needs the component the hog is drawn from, resolved on the GAME
// thread (this walk names 290k objects; the render callback must never pay that) and published
// as a pointer+slot pair the render side re-validates through TrackedObject each frame.
//
// The match is the Spartan pattern transplanted: a SkeletalMeshComponent on a
// "...VehicleActor_C_<id>" instance in the PersistentLevel, nearest the rider's Blam position.
std::atomic<uintptr_t> g_hog_body_ptr{0};
std::atomic<int32_t>   g_hog_body_idx{-1};

namespace {

void resolve_hog_body() {
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;
    const double S = 304.8;
    const double bx =  (double)g_unit_px.load(std::memory_order_relaxed) * S;
    const double by = -(double)g_unit_py.load(std::memory_order_relaxed) * S;
    const double bz =  (double)g_unit_pz.load(std::memory_order_relaxed) * S;
    const int32_t n = arr->get_object_count();
    API::UObject* best = nullptr; int32_t besti = -1; double bestd = 1e18;
    for (int32_t i = 0; i < n; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"SkeletalMeshComponent") continue;
        const std::wstring full = o->get_full_name();
        if (full.find(L"PersistentLevel") == std::wstring::npos) continue;
        if (full.find(L"VehicleActor") == std::wstring::npos) continue;
        // The Warthog's drawn mesh is ".hull" -- lowercase, measured by HOGDUMP, and the reason
        // a ".Body"-only match resolved nothing while the camera silently fell back. The NAME
        // gate cannot be dropped for "nearest skeletal", because the nearest skeletal mesh from
        // the driver's seat is the CHAINGUN (199 cm, its own actor) -- bolting the camera to the
        // turret would aim your head wherever the gunner points. Named hull/body first; anything
        // else only as a logged last resort for vehicles this convention misses.
        auto ends_with_ci = [&full](const wchar_t* suf) {
            const size_t m = wcslen(suf);
            if (full.size() < m) return false;
            for (size_t k = 0; k < m; ++k)
                if (towlower(full[full.size() - m + k]) != towlower(suf[k])) return false;
            return true;
        };
        const bool named = ends_with_ci(L".hull") || ends_with_ci(L".body");
        Vec3 w{};
        if (!call_ret_vec3(o, L"K2_GetComponentLocation", &w)) continue;
        const double d = std::sqrt(((double)w.x - bx) * ((double)w.x - bx)
                                 + ((double)w.y - by) * ((double)w.y - by)
                                 + ((double)w.z - bz) * ((double)w.z - bz));
        // A named hull always beats an unnamed candidate; distance only breaks ties in a class.
        const double score = named ? d : d + 100000.0;
        if (score < bestd) { bestd = score; best = o; besti = i; }
    }
    const bool fell_back = (bestd >= 100000.0 && bestd < 1e17);
    if (fell_back) bestd -= 100000.0;
    if (best != nullptr && bestd < 1000.0) {   // within 10 m, or it is not the thing you sit in
        g_hog_body_ptr.store((uintptr_t)best, std::memory_order_relaxed);
        g_hog_body_idx.store(besti, std::memory_order_relaxed);
        API::get()->log_info("[Halo-CampE-UEVR] HOGBODY: %ls at %.1fcm%s", best->get_full_name().c_str(),
                             bestd, fell_back ? "  (UNNAMED fallback -- check this is the hull)" : "");
    } else {
        g_hog_body_ptr.store(0, std::memory_order_relaxed);
        g_hog_body_idx.store(-1, std::memory_order_relaxed);
        API::get()->log_info("[Halo-CampE-UEVR] HOGBODY: no VehicleActor hull within 10 m (best %.0f) -- "
                             "camera stays on the chase-cam anchor", bestd < 1e17 ? bestd : -1.0);
    }
}

} // namespace

// Game-thread tick for the vehicle body work: the driver hide reconciles every tick, and the
// hog hull resolves at the mount edge and KEEPS RETRYING while it fails. The one-shot version
// cost a whole session of false verdicts: it fired at the exact instant the mounted flag flips
// -- mid entry animation -- found the nearest hull 235 m away, gave up for the rest of the
// mount, and the camera silently rode the fallback while three builds of the rigid path went
// untested. A resolve that can fail transiently must retry; every ~2 s while mounted-and-
// unresolved is invisible in cost. Cleared on dismount.
void vehicle_body_update() {
    driver_hide_update();
    // The hull resolve feeds the seat camera, the seated view and the wheel only (experimental, all off
    // by default): with none of them on, a mount must not sweep for the hog.
    // Exactly two consumers read the hull: the rigid seat camera (vehcam with anchor 2) and the wheel.
    if (!(g_cfg.enabled && ((g_cfg.veh_cam != 0 && g_cfg.veh_cam_anchor == 2) || g_cfg.vehicle_wheel != 0))) {
        g_hog_body_ptr.store(0, std::memory_order_relaxed);
        g_hog_body_idx.store(-1, std::memory_order_relaxed);
        return;
    }
    static bool s_was_mounted = false;
    static uint32_t s_hog_tick = 0;
    const bool m = g_unit_mounted.load(std::memory_order_relaxed);
    if (m && !s_was_mounted) { resolve_hog_body(); s_hog_tick = 0; }
    else if (m && g_hog_body_ptr.load(std::memory_order_relaxed) == 0) {
        if ((++s_hog_tick % 90u) == 0u) resolve_hog_body();
    }
    if (!m && s_was_mounted) { g_hog_body_ptr.store(0, std::memory_order_relaxed);
                               g_hog_body_idx.store(-1, std::memory_order_relaxed); }
    s_was_mounted = m;
}

} // namespace halo
