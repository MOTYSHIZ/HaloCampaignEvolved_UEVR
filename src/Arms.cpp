#include "Arms.hpp"

#include "Config.hpp"
#include "DevTools.hpp"
#include "Rig.hpp"
#include "UeObject.hpp"

#include <windows.h>

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

// KEEP THE POSE ALIVE ON A HIDDEN MESH.
//
// EVisibilityBasedAnimTickOption::AlwaysTickPoseAndRefreshBones = 0. The UE default is
// OnlyTickPoseWhenRendered, so hiding a skeletal mesh can stop its animation evaluating
// altogether -- and Rig.cpp derives the weapon position from THIS mesh's PrimaryWeapon socket.
// A frozen pose therefore freezes the socket and the gun drifts to wherever the pose stopped,
// intermittently, depending on when the hide lands relative to the rig resolving.
//
// Applied to skeletal meshes only: the static armour pieces have no pose to tick.
void call_always_tick_pose(API::UObject* comp) {
    if (comp == nullptr) return;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    p[0] = 0;   // AlwaysTickPoseAndRefreshBones
    comp->call_function(L"SetVisibilityBasedAnimTickOption", p);
}

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

    const std::string b{g_cfg.arm_hide_bone};
    const std::wstring wb(b.begin(), b.end());
    const int mode = hide ? g_cfg.arm_hide_mode : s_hidden_mode;
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
                call_always_tick_pose(comp);
            }
            switch (mode) {
                case 1:  call_set_visibility(comp, !hide);          break;
                case 2:  call_set_hidden(comp, hide);               break;
                default: if (hide) call_hide_bone(comp, wb.c_str());
                         else      call_unhide_bone(comp, wb.c_str()); break;
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

    // Only per-bone hiding needs the rig, and only to target the component the plugin provably
    // drives. Modes 1 and 2 sweep by class and never needed it -- applying the gate to them is
    // what stopped the re-assert after a weapon swap, since Rig.hpp records that a swap kills the
    // route.
    if (g_cfg.arm_hide_mode == 0 && rig_tracked_component() == nullptr) return;

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
} // namespace halo
