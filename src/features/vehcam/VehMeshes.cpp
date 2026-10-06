#include "features/vehcam/VehMeshes.hpp"

#include "Config.hpp"
#include "DevTools.hpp"
#include "Markers.hpp"                        // g_cam_x/y/z: the rendered eye (vehmeshdump's distances)
#include "Rig.hpp"                            // call_ret_vec3 (vehmeshdump)
#include "UeObject.hpp"                       // TrackedObject, uobject_slot_*, class_name_of, RIG_PARAM_BUF
#include "core/host/ArmsState.hpp"            // Arms.cpp's SetVisibility / SetHiddenInGame, never propagated
#include "features/vehcam/VehCam.hpp"         // vehcam_actor_tree
#include "features/vehcam/VehCamSelect.hpp"   // vehcam_hide_meshes

#include <Windows.h>

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using uevr::API;

namespace halo {

namespace {

// ---------------------------------------------------------------- THE FIELDS, from the engine's reflection
//
// USceneComponent's own fields, and the mesh each kind of mesh component draws, resolved ONCE on the class
// that declares them: every subclass has them at the same offsets, so what runs four times a second is
// plain reads. Reading the mesh by KIND matters more than it looks: UEVR caches a property a class HAS, but
// searches the whole class chain again -- naming every property on the way -- for one it lacks, so asking
// a skeletal mesh for "StaticMesh" pays that on every call. Fails CLOSED: without the fields the hide needs,
// nothing is hidden, and the log says so once.
struct Refl {
    int state = -1;                        // -1 not tried; 0 unusable; 1 ready
    API::UClass* scene = nullptr;          // USceneComponent
    API::UClass* actor = nullptr;          // AActor
    API::UClass* static_mesh = nullptr;    // UStaticMeshComponent
    API::UClass* skinned = nullptr;        // USkinnedMeshComponent
    int32_t scale = -1;                    // RelativeScale3D: three doubles (LWC)
    int32_t kids = -1;                     // AttachChildren: TArray<USceneComponent*>
    int32_t parent = -1;                   // AttachParent
    int32_t socket = -1;                   // AttachSocketName
    int32_t vis = -1, hid = -1;            // the bytes holding bVisible / bHiddenInGame...
    uint8_t vis_mask = 0, hid_mask = 0;    // ...and their bits
    int32_t sm_mesh = -1;                  // UStaticMeshComponent.StaticMesh
    int32_t sk_mesh = -1;                  // USkinnedMeshComponent.SkinnedAsset (SkeletalMesh before UE 5.1)
    int32_t tick = -1;                     // USkinnedMeshComponent.VisibilityBasedAnimTickOption (one byte)
};
Refl s_rf;

int32_t prop_off(API::UClass* c, const wchar_t* name) {
    auto* p = (c != nullptr) ? c->find_property(name) : nullptr;
    return (p != nullptr) ? p->get_offset() : -1;
}

// A bool UPROPERTY's byte and bit. bVisible and bHiddenInGame are one-bit bitfields: a plain read of the
// byte would answer for whichever flag shares it.
bool bool_field(API::UClass* c, const wchar_t* name, int32_t* off, uint8_t* mask) {
    auto* p = (c != nullptr) ? c->find_property(name) : nullptr;
    auto* fc = (p != nullptr) ? p->get_class() : nullptr;
    auto* fn = (fc != nullptr) ? fc->get_fname() : nullptr;
    if (fn == nullptr || fn->to_string() != L"BoolProperty") return false;
    auto* bp = reinterpret_cast<API::FBoolProperty*>(p);
    *off = p->get_offset() + static_cast<int32_t>(bp->get_byte_offset());
    *mask = static_cast<uint8_t>(bp->get_byte_mask());
    return *mask != 0;
}

bool refl_ready() {
    if (s_rf.state >= 0) return s_rf.state == 1;
    s_rf.state = 0;
    const auto& api = API::get();
    s_rf.scene       = api->find_uobject<API::UClass>(L"Class /Script/Engine.SceneComponent");
    s_rf.actor       = api->find_uobject<API::UClass>(L"Class /Script/Engine.Actor");
    s_rf.static_mesh = api->find_uobject<API::UClass>(L"Class /Script/Engine.StaticMeshComponent");
    s_rf.skinned     = api->find_uobject<API::UClass>(L"Class /Script/Engine.SkinnedMeshComponent");
    s_rf.scale  = prop_off(s_rf.scene, L"RelativeScale3D");
    s_rf.kids   = prop_off(s_rf.scene, L"AttachChildren");
    s_rf.parent = prop_off(s_rf.scene, L"AttachParent");
    s_rf.socket = prop_off(s_rf.scene, L"AttachSocketName");
    const bool vis = bool_field(s_rf.scene, L"bVisible", &s_rf.vis, &s_rf.vis_mask);
    const bool hid = bool_field(s_rf.scene, L"bHiddenInGame", &s_rf.hid, &s_rf.hid_mask);
    s_rf.sm_mesh = prop_off(s_rf.static_mesh, L"StaticMesh");
    s_rf.sk_mesh = prop_off(s_rf.skinned, L"SkinnedAsset");
    if (s_rf.sk_mesh < 0) s_rf.sk_mesh = prop_off(s_rf.skinned, L"SkeletalMesh");
    s_rf.tick = prop_off(s_rf.skinned, L"VisibilityBasedAnimTickOption");
    // What the hide needs. The mesh fields only let it match a part by the mesh it draws as well as its name.
    if (s_rf.scene != nullptr && s_rf.actor != nullptr && s_rf.scale >= 0 && s_rf.kids >= 0 && vis && hid) s_rf.state = 1;
    API::get()->log_info("[Halo-CampE-UEVR] VEHMESH: %s (SceneComponent %s, scale @%d, children @%d, bVisible %s, "
                         "bHiddenInGame %s; the mesh a part draws: static @%d, skinned @%d)",
                         s_rf.state == 1 ? "a vehicle's parts can be hidden (hideMeshes)"
                                         : "hideMeshes is NOT available on this build -- a field it needs is missing",
                         s_rf.scene != nullptr ? "found" : "NOT found", s_rf.scale, s_rf.kids,
                         vis ? "found" : "NOT found", hid ? "found" : "NOT found", s_rf.sm_mesh, s_rf.sk_mesh);
    return s_rf.state == 1;
}

template <typename T> T* at(API::UObject* o, int32_t off) {
    return reinterpret_cast<T*>(reinterpret_cast<uint8_t*>(o) + off);
}
bool vis_flag(API::UObject* c) { return (*at<uint8_t>(c, s_rf.vis) & s_rf.vis_mask) != 0; }
bool hid_flag(API::UObject* c) { return (*at<uint8_t>(c, s_rf.hid) & s_rf.hid_mask) != 0; }

struct RawArr { API::UObject** data; int32_t num; int32_t max; };
int child_count(API::UObject* c) {
    const RawArr* a = at<RawArr>(c, s_rf.kids);
    return (a->num > 0 && a->num <= a->max && a->max < (1 << 16)) ? a->num : 0;
}

// The mesh asset a part draws, read by its kind; nullptr = not a static or skinned mesh, or none set.
API::UObject* mesh_of(API::UObject* c) {
    int32_t off = -1;
    if (s_rf.sm_mesh >= 0 && c->is_a(s_rf.static_mesh)) off = s_rf.sm_mesh;
    else if (s_rf.sk_mesh >= 0 && c->is_a(s_rf.skinned)) off = s_rf.sk_mesh;
    if (off < 0) return nullptr;
    API::UObject* m = *at<API::UObject*>(c, off);
    return (m != nullptr && uobject_slot_valid(m)) ? m : nullptr;
}

std::wstring name_of(API::UObject* o) {
    auto* f = (o != nullptr) ? o->get_fname() : nullptr;
    return (f != nullptr) ? f->to_string() : std::wstring{};
}

// Lower-case ASCII, as the camera file's names are kept (anything else becomes '?', which no name matches).
std::string lower_ascii(const std::wstring& w) {
    std::string s;
    s.reserve(w.size());
    for (const wchar_t ch : w)
        s.push_back((ch > 0 && ch < 0x80) ? static_cast<char>(std::tolower(static_cast<int>(ch))) : '?');
    return s;
}

// The actor a component belongs to: its outer, or up to two steps further out -- a part made by one of the
// actor's components, as Blam's mesh synchronization makes a Spartan's armour. nullptr = none.
API::UObject* owner_actor(API::UObject* c) {
    API::UObject* o = c->get_outer();
    for (int k = 0; k < 3 && o != nullptr; ++k, o = o->get_outer())
        if (o->is_a(s_rf.actor)) return o;
    return nullptr;
}

// THE VEHICLE: the actor your seat belongs to, and the actor of the mesh the cameras follow when that is
// another (a turret's seat is on the turret's actor, the hull on the vehicle's). `frame` = that mesh.
void vehicle_actors(API::UObject* seat_actor, uintptr_t chassis, int32_t chassis_idx, API::UObject* out[2],
                    API::UObject** frame) {
    out[0] = out[1] = nullptr;
    *frame = nullptr;
    if (seat_actor != nullptr && uobject_slot_valid(seat_actor)) out[0] = seat_actor;
    static TrackedObject s_ch;
    if (chassis == 0) { s_ch.reset(); return; }
    if (reinterpret_cast<uintptr_t>(s_ch.ptr) != chassis || s_ch.index != chassis_idx)
        s_ch.set_at(reinterpret_cast<API::UObject*>(chassis), chassis_idx);
    if (API::UObject* ch = s_ch.get()) {
        *frame = ch;
        API::UObject* own = owner_actor(ch);
        if (own != nullptr && own != out[0]) out[out[0] != nullptr ? 1 : 0] = own;
    }
}

constexpr int kTree = 256;   // components looked at in one vehicle: its own, and whatever rides it

// Every component of the vehicle's actors, once each. Live when returned; nothing is kept.
int vehicle_tree(API::UObject* const actors[2], API::UObject** out) {
    int n = 0;
    for (int a = 0; a < 2; ++a) {
        if (actors[a] == nullptr) continue;
        API::UObject* sub[kTree];
        const int k = vehcam_actor_tree(actors[a], sub, kTree);
        for (int j = 0; j < k && n < kTree; ++j) {
            bool dup = false;
            for (int q = 0; q < n && !dup; ++q) dup = out[q] == sub[j];
            if (!dup) out[n++] = sub[j];
        }
    }
    return n;
}

// SetRelativeScale3D(FVector): three DOUBLES under LWC (VehCam.cpp's call_set_scale says what floats do).
void set_scale(API::UObject* c, const double sc[3]) {
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    auto* d = reinterpret_cast<double*>(p);
    d[0] = sc[0]; d[1] = sc[1]; d[2] = sc[2];
    c->call_function(L"SetRelativeScale3D", p);
}
constexpr double kShrink = 0.001;
const double kShrunk[3] = {kShrink, kShrink, kShrink};
bool is_shrunk(API::UObject* c) {
    const double* s = at<double>(c, s_rf.scale);
    return std::fabs(s[0] - kShrink) < 1e-5 && std::fabs(s[1] - kShrink) < 1e-5 && std::fabs(s[2] - kShrink) < 1e-5;
}

// A UFUNCTION parameter's offset in its parameter block; -1 = none (or a block too big for our buffer).
int32_t fn_param(API::UFunction* fn, const wchar_t* name) {
    if (fn == nullptr || fn->get_properties_size() > static_cast<int32_t>(RIG_PARAM_BUF)) return -1;
    auto* p = fn->find_property(name);
    return (p != nullptr) ? p->get_offset() : -1;
}

// ---------------------------------------------------------------- THE HIDE ("hideMeshes")
constexpr int kMaxParts = 48;   // hidden at once: a vehicle's damage parts are a handful

// Shrunk: a part nothing hangs from. Flagged: a part that carries others (or the cameras' frame), hidden by its
// own flags. A SKINNED one is also held to AlwaysTickPoseAndRefreshBones for as long as it is hidden (`tick`):
// a skeletal mesh that is not drawn refreshes its bones only with that option, and everything hung on it
// follows its bones. Tried on the Banshee's hull in the headset (the author, 2026-09-29), in order: its flags
// alone hid its stray triangle and froze every animated part on it; HideBoneByName on its root froze most of
// them in a flip; ShowMaterialSection on its one section (applied, per the log) left the triangle drawn.
enum class How : uint8_t { Shrunk, Flagged };
struct Part {
    TrackedObject obj;
    How    how = How::Shrunk;
    double scale[3] = {1.0, 1.0, 1.0};   // Shrunk: the scale to put back -- the one it had, or the game's latest
    bool   vis = true, hid = false;      // Flagged: the flags to put back, likewise
    int    tick = -1;                    // Flagged skinned: its VisibilityBasedAnimTickOption to put back; -1 = untouched
    bool   fought = false;               // something put it back while we held it (said once)
    std::wstring label;                  // its name, and the mesh it draws
};

uint8_t* tick_option(API::UObject* c) { return at<uint8_t>(c, s_rf.tick); }
// A component judged this ride, and the mesh it drew then: judged again only when that changes.
struct Seen { API::UObject* o; int32_t i; API::UObject* mesh; };

struct Hide {
    Part parts[kMaxParts];
    int  n = 0;
    std::vector<Seen> seen;
    bool on = false;                     // holding a choice -- the list and the vehicle below
    uint32_t rev = 0;                    // the hideMeshes revision it was made for
    API::UObject* actors[2] = {};        // ...and the vehicle
    ULONGLONG next = 0;                  // the next look, four times a second
    bool full_said = false;
};
Hide s_h;

void put_back(Part& p) {
    API::UObject* c = p.obj.get();
    if (c == nullptr) return;                              // gone with its vehicle: nothing to put back
    if (p.how == How::Shrunk) {
        if (is_shrunk(c)) set_scale(c, p.scale);           // only while it is still ours
        return;
    }
    // The flags we set, back to the game's word -- only where they still hold ours.
    if (!vis_flag(c) && p.vis) host::g_arms_state.call_set_visibility(c, true);
    if (hid_flag(c) && !p.hid) host::g_arms_state.call_set_hidden(c, false);
    // ...then a skinned one's tick option, drawn again first so its pose never stops in between.
    if (p.tick >= 0 && *tick_option(c) == 0) *tick_option(c) = static_cast<uint8_t>(p.tick);
}

void restore_all(const char* why) {
    if (!s_h.on) return;
    int live = 0;
    for (int k = 0; k < s_h.n; ++k) {
        if (s_h.parts[k].obj.get() != nullptr) ++live;
        put_back(s_h.parts[k]);
    }
    if (s_h.n > 0)
        API::get()->log_info("[Halo-CampE-UEVR] VEHMESH: %d vehicle part(s) shown again -- %s", live, why);
    for (int k = 0; k < s_h.n; ++k) s_h.parts[k] = Part{};
    s_h.n = 0;
    s_h.seen.clear();
    s_h.on = false;
    s_h.full_said = false;
    s_h.actors[0] = s_h.actors[1] = nullptr;
}

void hide_part(API::UObject* c, int32_t idx, bool frame, std::wstring label) {
    if (s_h.n >= kMaxParts) {
        if (!s_h.full_said) {
            s_h.full_said = true;
            API::get()->log_info("[Halo-CampE-UEVR] VEHMESH: more than %d parts match hideMeshes -- the rest stay "
                                 "shown; name the parts more narrowly", kMaxParts);
        }
        return;
    }
    // It carries other parts, or it IS the frame every seat and socket sits on: never shrunk, which would
    // shrink everything hung on it.
    const bool carrier = frame || child_count(c) > 0;
    const bool skinned = carrier && s_rf.skinned != nullptr && c->is_a(s_rf.skinned);
    int tick = -1;
    if (skinned) {
        // Its pose must keep refreshing while it is not drawn, or every part hung on it freezes. Where that
        // cannot be set it stays SHOWN (fail closed) -- a stray triangle beats a frozen vehicle.
        tick = s_rf.tick >= 0 ? *tick_option(c) : -1;
        if (tick < 0 || !rig_set_always_tick_pose(c)) {
            API::get()->log_info("[Halo-CampE-UEVR] VEHMESH: %ls NOT hidden -- it is skinned and carries other parts, and "
                                 "its pose cannot be kept updating on this build (VisibilityBasedAnimTickOption not "
                                 "found), so hiding it would freeze them", label.c_str());
            return;
        }
    }
    Part& p = s_h.parts[s_h.n++];
    p = Part{};
    p.obj.set_at(c, idx);
    p.label = std::move(label);
    if (carrier) {
        // Hidden by its own flags, never propagated, so whatever hangs from it stays shown.
        p.how = How::Flagged;
        p.vis = vis_flag(c);
        p.hid = hid_flag(c);
        p.tick = tick;
        host::g_arms_state.call_set_visibility(c, false);
        host::g_arms_state.call_set_hidden(c, true);
    } else {
        const double* s = at<double>(c, s_rf.scale);
        for (int k = 0; k < 3; ++k) p.scale[k] = std::isfinite(s[k]) ? s[k] : 1.0;
        set_scale(c, kShrunk);
    }
    const char* why = frame ? "it is the cameras' frame" : "parts hang from it";
    if (p.how == How::Flagged && p.tick >= 0)
        API::get()->log_info("[Halo-CampE-UEVR] VEHMESH: hidden %ls -- by its visibility, its pose kept updating "
                             "(tick option %d -> AlwaysTickPoseAndRefreshBones) (%s, so it is not shrunk and they stay "
                             "shown and moving)", p.label.c_str(), p.tick, why);
    else if (p.how == How::Flagged)
        API::get()->log_info("[Halo-CampE-UEVR] VEHMESH: hidden %ls -- by its visibility (%s, so it is not shrunk and "
                             "they stay shown)", p.label.c_str(), why);
    else
        API::get()->log_info("[Halo-CampE-UEVR] VEHMESH: hidden %ls -- shrunk", p.label.c_str());
}

// Whether `c` is a part the list names: a MESH of the vehicle's own actors -- never the Chief riding it, or
// anything else merely hanging on it -- whose name, or whose mesh's, contains one of the names.
bool wanted(API::UObject* c, API::UObject* mesh, const std::vector<std::string>& names, std::wstring* label) {
    if (class_name_of(c).find(L"Mesh") == std::wstring::npos) return false;   // never a camera, light or scene node
    API::UObject* own = owner_actor(c);
    if (own == nullptr || (own != s_h.actors[0] && own != s_h.actors[1])) return false;
    const std::wstring cn = name_of(c), mn = name_of(mesh);
    const std::string lc = lower_ascii(cn), lm = lower_ascii(mn);
    for (const std::string& n : names) {
        if (lc.find(n) == std::string::npos && (lm.empty() || lm.find(n) == std::string::npos)) continue;
        *label = cn + (mn.empty() ? std::wstring{} : L" (" + mn + L")");
        return true;
    }
    return false;
}

// Hold what is hidden: a part something has put back since -- its scale or its flags -- is hidden again,
// and what it was put back TO becomes what the restore gives back (a damage state revealing a part while
// we hold it is the game's latest word on it). Parts gone with their objects are dropped.
void hold() {
    int w = 0;
    for (int k = 0; k < s_h.n; ++k) {
        Part& p = s_h.parts[k];
        API::UObject* c = p.obj.get();
        if (c == nullptr) continue;
        bool fought = false;
        if (p.how == How::Shrunk) {
            if (!is_shrunk(c)) {
                const double* s = at<double>(c, s_rf.scale);
                for (int j = 0; j < 3; ++j) if (std::isfinite(s[j])) p.scale[j] = s[j];
                set_scale(c, kShrunk);
                fought = true;
            }
        } else {
            const bool v = vis_flag(c), h = hid_flag(c);
            if (v || !h) {
                if (v) p.vis = true;
                if (!h) p.hid = false;
                host::g_arms_state.call_set_visibility(c, false);
                host::g_arms_state.call_set_hidden(c, true);
                fought = true;
            }
            if (p.tick >= 0 && *tick_option(c) != 0) {   // its pose left to stop again: set back, the new value kept
                p.tick = *tick_option(c);
                rig_set_always_tick_pose(c);
                fought = true;
            }
        }
        if (fought && !p.fought) {
            p.fought = true;
            API::get()->log_info("[Halo-CampE-UEVR] VEHMESH: %ls was put back while hidden -- hidden again (if the game "
                                 "does this every frame, it will flicker)", p.label.c_str());
        }
        if (w != k) s_h.parts[w] = std::move(p);
        ++w;
    }
    for (int k = w; k < s_h.n; ++k) s_h.parts[k] = Part{};
    s_h.n = w;
}

// Look at the vehicle's parts: each judged once, by its name and its mesh's, and again only if it changes
// mesh (a damage state may swap what a part draws). A part the list names is hidden; one that stops being
// named by a mesh change is shown again.
void scan(const std::vector<std::string>& names, API::UObject* frame) {
    API::UObject* tree[kTree];
    const int nt = vehicle_tree(s_h.actors, tree);
    for (int t = 0; t < nt; ++t) {
        API::UObject* c = tree[t];
        const int32_t idx = uobject_slot_index(c);
        if (idx < 0 || !c->is_a(s_rf.scene)) continue;
        API::UObject* mesh = mesh_of(c);
        Seen* seen = nullptr;
        for (Seen& e : s_h.seen) if (e.o == c && e.i == idx) { seen = &e; break; }
        if (seen != nullptr && seen->mesh == mesh) continue;          // judged, and drawing what it drew then
        if (seen != nullptr) seen->mesh = mesh;
        else s_h.seen.push_back({c, idx, mesh});
        int held = -1;
        for (int k = 0; k < s_h.n && held < 0; ++k)
            if (s_h.parts[k].obj.ptr == c && s_h.parts[k].obj.index == idx) held = k;
        std::wstring label;
        const bool want = wanted(c, mesh, names, &label);
        if (want && held < 0) {
            hide_part(c, idx, c == frame, std::move(label));
        } else if (!want && held >= 0) {
            API::get()->log_info("[Halo-CampE-UEVR] VEHMESH: %ls changed mesh and is no longer named -- shown again",
                                 s_h.parts[held].label.c_str());
            put_back(s_h.parts[held]);
            for (int k = held; k + 1 < s_h.n; ++k) s_h.parts[k] = std::move(s_h.parts[k + 1]);
            s_h.parts[--s_h.n] = Part{};
        }
    }
}

#if HALO_VR_DEV
// ---------------------------------------------------------------- vehmeshdump (dev builds)
//
// EVERY PART OF THE VEHICLE YOU SIT IN, for choosing what "hideMeshes" names: each component in its tree and
// each mesh it owns -- class, name, owner, the mesh it draws and its materials, what it hangs from and at
// which socket, whether it is shown, its scale, how far it is from your eye, and a skinned mesh's bones.
// Listed when you get in -- the vehicle as it stands, with any parts the game keeps hidden until they are
// needed -- then WATCHED twice a second while you ride: a part that appears, goes, is shown or hidden,
// changes mesh or changes scale is logged the moment it does (a DAMAGE STATE as it happens), and the file is
// written again with every change so far and the vehicle as it is now. Read-only.
// The file: <profile>\data\halo_vr_vehmeshes_<vehicle class>.txt.

struct Snap {
    API::UObject* o; int32_t i;
    API::UObject* mesh;
    bool vis, hid;
    double scale[3];
    std::wstring name;                   // kept, so a part that goes can still be named
};
struct Dump {
    bool on = false;
    API::UObject* actors[2] = {};
    ULONGLONG t0 = 0, next = 0;
    std::vector<Snap> last;
    std::string head, stock, changes;
    int nchanges = 0;
    std::string path;
};
Dump s_dd;

std::string w2s(const std::wstring& w) { return narrow(w); }

std::string materials_of(API::UObject* c) {
    auto* cls = c->get_class();
    API::UFunction* fn_n = (cls != nullptr) ? cls->find_function(L"GetNumMaterials") : nullptr;
    API::UFunction* fn_m = (cls != nullptr) ? cls->find_function(L"GetMaterial") : nullptr;
    const int32_t n_ret = fn_param(fn_n, L"ReturnValue");
    const int32_t m_idx = fn_param(fn_m, L"ElementIndex"), m_ret = fn_param(fn_m, L"ReturnValue");
    if (n_ret < 0 || m_idx < 0 || m_ret < 0) return {};
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    c->call_function(L"GetNumMaterials", p);
    const int32_t n = *reinterpret_cast<int32_t*>(p + n_ret);
    if (n <= 0 || n > 64) return {};
    std::string out;
    for (int32_t k = 0; k < n; ++k) {
        std::memset(p, 0, sizeof(p));
        *reinterpret_cast<int32_t*>(p + m_idx) = k;
        c->call_function(L"GetMaterial", p);
        API::UObject* m = *reinterpret_cast<API::UObject**>(p + m_ret);
        out += (k != 0 ? ", " : "") + ((m != nullptr && uobject_slot_valid(m)) ? w2s(name_of(m)) : std::string("none"));
    }
    return out;
}

std::string bones_of(API::UObject* c, int32_t* count) {
    *count = 0;
    auto* cls = c->get_class();
    API::UFunction* fn_n = (cls != nullptr) ? cls->find_function(L"GetNumBones") : nullptr;
    API::UFunction* fn_b = (cls != nullptr) ? cls->find_function(L"GetBoneName") : nullptr;
    const int32_t n_ret = fn_param(fn_n, L"ReturnValue");
    const int32_t b_idx = fn_param(fn_b, L"BoneIndex"), b_ret = fn_param(fn_b, L"ReturnValue");
    if (n_ret < 0 || b_idx < 0 || b_ret < 0) return {};
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    c->call_function(L"GetNumBones", p);
    const int32_t n = *reinterpret_cast<int32_t*>(p + n_ret);
    if (n <= 0 || n > 2048) return {};
    *count = n;
    std::string out;
    for (int32_t k = 0; k < n; ++k) {
        std::memset(p, 0, sizeof(p));
        *reinterpret_cast<int32_t*>(p + b_idx) = k;
        c->call_function(L"GetBoneName", p);
        out += (k != 0 ? ", " : "") + w2s(reinterpret_cast<API::FName*>(p + b_ret)->to_string());
    }
    return out;
}

// One part as the file lists it; `brief` = the log's one line (no materials, no bones).
std::string describe(const Snap& s, bool brief) {
    API::UObject* c = s.o;
    char buf[640];
    API::UObject* own = owner_actor(c);
    API::UObject* outer = c->get_outer();
    const bool ours = own != nullptr && (own == s_dd.actors[0] || own == s_dd.actors[1]);
    std::string line = w2s(class_name_of(c)) + " " + w2s(s.name);
    line += "  owner=" + (own != nullptr ? w2s(name_of(own)) : std::string("?")) + (ours ? "" : " (NOT the vehicle)");
    if (outer != nullptr && outer != own) line += " via " + w2s(name_of(outer));
    if (s.mesh != nullptr) {
        std::wstring path = s.mesh->get_full_name();                   // "StaticMesh /Game/.../SM_X.SM_X"
        const size_t sp = path.find(L' ');
        if (sp != std::wstring::npos) path = path.substr(sp + 1);
        line += "  mesh=" + w2s(name_of(s.mesh)) + " [" + w2s(path) + "]";
    }
    std::snprintf(buf, sizeof(buf), "  shown=%d (bVisible=%d bHiddenInGame=%d)  scale=(%g %g %g)",
                  (s.vis && !s.hid) ? 1 : 0, s.vis ? 1 : 0, s.hid ? 1 : 0, s.scale[0], s.scale[1], s.scale[2]);
    line += buf;
    if (s_rf.parent >= 0) {
        API::UObject* par = *at<API::UObject*>(c, s_rf.parent);
        if (par != nullptr && uobject_slot_valid(par)) {
            line += "  hangs from=" + w2s(name_of(par));
            if (s_rf.socket >= 0) {
                const std::wstring sock = at<API::FName>(c, s_rf.socket)->to_string();
                if (!sock.empty() && sock != L"None") line += "@" + w2s(sock);
            }
        }
    }
    std::snprintf(buf, sizeof(buf), "  children=%d", child_count(c));
    line += buf;
    Vec3 w{};
    if (call_ret_vec3(c, L"K2_GetComponentLocation", &w)) {
        const double dx = (double)w.x - g_cam_x.load(std::memory_order_relaxed);
        const double dy = (double)w.y - g_cam_y.load(std::memory_order_relaxed);
        const double dz = (double)w.z - g_cam_z.load(std::memory_order_relaxed);
        std::snprintf(buf, sizeof(buf), "  from your eye=%.0f cm", std::sqrt(dx * dx + dy * dy + dz * dz));
        line += buf;
    }
    if (brief) return line;
    if (class_name_of(c).find(L"Mesh") != std::wstring::npos) {
        const std::string mats = materials_of(c);
        if (!mats.empty()) line += "\r\n      materials: " + mats;
        if (s_rf.skinned != nullptr && c->is_a(s_rf.skinned)) {
            int32_t nb = 0;
            const std::string bones = bones_of(c, &nb);
            std::snprintf(buf, sizeof(buf), "\r\n      bones (%d): ", nb);
            if (nb > 0) line += buf + bones;
        }
    }
    return line;
}

std::vector<Snap> snapshot() {
    std::vector<Snap> out;
    API::UObject* tree[kTree];
    const int nt = vehicle_tree(s_dd.actors, tree);
    out.reserve(static_cast<size_t>(nt));
    for (int t = 0; t < nt; ++t) {
        API::UObject* c = tree[t];
        const int32_t idx = uobject_slot_index(c);
        if (idx < 0 || !c->is_a(s_rf.scene)) continue;
        Snap s{c, idx, mesh_of(c), vis_flag(c), hid_flag(c), {1.0, 1.0, 1.0}, {}};
        const double* sc = at<double>(c, s_rf.scale);
        for (int k = 0; k < 3; ++k) s.scale[k] = sc[k];
        for (const Snap& l : s_dd.last) if (l.o == c && l.i == idx) { s.name = l.name; break; }
        if (s.name.empty()) s.name = name_of(c);
        out.push_back(std::move(s));
    }
    return out;
}

std::string listing(const std::vector<Snap>& snaps, bool log_it) {
    std::string s;
    for (size_t k = 0; k < snaps.size(); ++k) {
        char num[16];
        std::snprintf(num, sizeof(num), "  [%zu] ", k);
        s += num + describe(snaps[k], false) + "\r\n";
        if (log_it)
            API::get()->log_info("[Halo-CampE-UEVR] VEHMESH   [%zu] %s", k, describe(snaps[k], true).c_str());
    }
    return s;
}

double since(ULONGLONG t) { return static_cast<double>(t - s_dd.t0) / 1000.0; }

void write_file(const std::vector<Snap>* now_snaps, ULONGLONG now) {
    FILE* f = nullptr;
    if (s_dd.path.empty() || fopen_s(&f, s_dd.path.c_str(), "wb") != 0 || f == nullptr) return;
    std::string s = s_dd.head;
    s += "\r\n== WHEN YOU GOT IN (+0.0 s) ==\r\n" + s_dd.stock;
    char buf[96];
    std::snprintf(buf, sizeof(buf), "\r\n== CHANGES WHILE RIDING (%d) ==\r\n", s_dd.nchanges);
    s += buf + (s_dd.changes.empty() ? std::string("  none yet\r\n") : s_dd.changes);
    if (now_snaps != nullptr) {
        std::snprintf(buf, sizeof(buf), "\r\n== AS IT IS NOW (+%.1f s) ==\r\n", since(now));
        s += buf + listing(*now_snaps, false);
    }
    fwrite(s.data(), 1, s.size(), f);
    fclose(f);
}

void change(ULONGLONG now, const std::string& what) {
    char t[32];
    std::snprintf(t, sizeof(t), "  +%.1f s  ", since(now));
    s_dd.changes += t + what + "\r\n";
    ++s_dd.nchanges;
    API::get()->log_info("[Halo-CampE-UEVR] VEHMESH: %s", what.c_str());
}

bool same_scale(const double a[3], const double b[3]) {
    return std::fabs(a[0] - b[0]) < 1e-4 && std::fabs(a[1] - b[1]) < 1e-4 && std::fabs(a[2] - b[2]) < 1e-4;
}
bool shrunk3(const double s[3]) {
    return std::fabs(s[0] - kShrink) < 1e-5 && std::fabs(s[1] - kShrink) < 1e-5 && std::fabs(s[2] - kShrink) < 1e-5;
}

void dump_tick(bool riding, API::UObject* seat_actor, uintptr_t chassis, int32_t chassis_idx) {
    if (!riding || !g_cfg.veh_mesh_dump) {
        if (s_dd.on) {
            API::get()->log_info("[Halo-CampE-UEVR] VEHMESH: listing ended -- %d change(s) seen, written to %s",
                                 s_dd.nchanges, s_dd.path.c_str());
            s_dd = Dump{};
        }
        return;
    }
    if (!refl_ready()) return;
    API::UObject* actors[2];
    API::UObject* frame = nullptr;
    vehicle_actors(seat_actor, chassis, chassis_idx, actors, &frame);
    if (actors[0] == nullptr && actors[1] == nullptr) return;   // not known yet
    const ULONGLONG now = GetTickCount64();
    if (!s_dd.on || actors[0] != s_dd.actors[0] || actors[1] != s_dd.actors[1]) {
        s_dd = Dump{};
        s_dd.on = true;
        s_dd.actors[0] = actors[0];
        s_dd.actors[1] = actors[1];
        s_dd.t0 = now;
        s_dd.next = now + 500;
        API::UObject* main = actors[0] != nullptr ? actors[0] : actors[1];
        std::string cls = w2s(class_name_of(main));
        for (char& ch : cls) if (!std::isalnum(static_cast<unsigned char>(ch)) && ch != '_') ch = '_';
        s_dd.path = std::string(g_data_dir) + "\\halo_vr_vehmeshes_" + cls + ".txt";
        s_dd.last = snapshot();
        s_dd.head = "halo_vr vehicle parts (vehmeshdump) -- every component of the vehicle you sat in, and each change\r\n"
                    "seen while you rode it. For choosing \"hideMeshes\" in halo_vr_vehcams.json: a part is named by text\r\n"
                    "found in its own name or its mesh's. 'shown' = bVisible and not bHiddenInGame.\r\n\r\n";
        s_dd.head += "Vehicle: " + w2s(main->get_full_name()) + "\r\n";
        if (actors[0] != nullptr && actors[1] != nullptr) s_dd.head += "  and: " + w2s(actors[1]->get_full_name()) + "\r\n";
        if (frame != nullptr) s_dd.head += "The cameras' frame (chassis): " + w2s(frame->get_full_name()) + "\r\n";
        API::get()->log_info("[Halo-CampE-UEVR] VEHMESH: listing %s -- %d part(s), watching for changes (vehmeshdump); "
                             "file %s", w2s(name_of(main)).c_str(), (int)s_dd.last.size(), s_dd.path.c_str());
        s_dd.stock = listing(s_dd.last, /*log_it=*/true);
        write_file(nullptr, now);
        return;
    }
    if (now < s_dd.next) return;
    s_dd.next = now + 500;
    std::vector<Snap> cur = snapshot();
    const int before = s_dd.nchanges;
    for (const Snap& c : cur) {
        const Snap* l = nullptr;
        for (const Snap& e : s_dd.last) if (e.o == c.o && e.i == c.i) { l = &e; break; }
        if (l == nullptr) { change(now, "APPEARED " + describe(c, true)); continue; }
        if (l->vis != c.vis || l->hid != c.hid) {
            char b[160];
            std::snprintf(b, sizeof(b), " (bVisible %d->%d, bHiddenInGame %d->%d)", l->vis ? 1 : 0, c.vis ? 1 : 0,
                          l->hid ? 1 : 0, c.hid ? 1 : 0);
            const bool was = l->vis && !l->hid, is = c.vis && !c.hid;
            change(now, std::string(is == was ? "FLAGS " : (is ? "SHOWN " : "HIDDEN ")) + w2s(c.name) + b);
        }
        if (l->mesh != c.mesh)
            change(now, "MESH " + w2s(c.name) + ": " + (l->mesh != nullptr && uobject_slot_valid(l->mesh) ? w2s(name_of(l->mesh)) : std::string("none"))
                        + " -> " + (c.mesh != nullptr ? w2s(name_of(c.mesh)) : std::string("none")));
        if (!same_scale(l->scale, c.scale)) {
            char b[200];
            std::snprintf(b, sizeof(b), ": (%g %g %g) -> (%g %g %g)%s", l->scale[0], l->scale[1], l->scale[2], c.scale[0],
                          c.scale[1], c.scale[2], (shrunk3(l->scale) || shrunk3(c.scale)) ? "  (hideMeshes' own shrink)" : "");
            change(now, "SCALE " + w2s(c.name) + b);
        }
    }
    for (const Snap& l : s_dd.last) {
        bool still = false;
        for (const Snap& c : cur) if (c.o == l.o && c.i == l.i) { still = true; break; }
        if (!still) change(now, "GONE " + w2s(l.name));
    }
    s_dd.last = std::move(cur);
    if (s_dd.nchanges != before) write_file(&s_dd.last, now);
}
#else
inline void dump_tick(bool, API::UObject*, uintptr_t, int32_t) {}
#endif

} // namespace

void vehmesh_tick(bool riding, API::UObject* seat_actor, uintptr_t chassis, int32_t chassis_idx) {
    const bool live = riding && g_cfg.enabled;
    dump_tick(live, seat_actor, chassis, chassis_idx);      // vehmeshdump: a no-op in a release build
    uint32_t rev = 0;
    const std::vector<std::string>& names = vehcam_hide_meshes(&rev);
    if (!live || names.empty()) {
        // (riding already folds the kill switch in, so it is asked first)
        restore_all(!g_cfg.enabled ? "the kill switch" : !riding ? "out of the vehicle" : "this camera hides nothing");
        return;
    }
    if (!refl_ready()) return;
    API::UObject* actors[2];
    API::UObject* frame = nullptr;
    vehicle_actors(seat_actor, chassis, chassis_idx, actors, &frame);
    if (actors[0] == nullptr && actors[1] == nullptr) {
        restore_all("the vehicle is not known");
        return;
    }
    if (s_h.on && rev != s_h.rev) restore_all("the camera's list changed");
    else if (s_h.on && (actors[0] != s_h.actors[0] || actors[1] != s_h.actors[1])) restore_all("another vehicle");
    const ULONGLONG now = GetTickCount64();
    if (!s_h.on) {
        s_h.on = true;
        s_h.rev = rev;
        s_h.actors[0] = actors[0];
        s_h.actors[1] = actors[1];
        s_h.next = 0;
        std::string list;
        for (const std::string& n : names) list += (list.empty() ? "" : ", ") + n;
        API::get()->log_info("[Halo-CampE-UEVR] VEHMESH: %ls -- hiding the parts named [%s] (hideMeshes)",
                             name_of(actors[0] != nullptr ? actors[0] : actors[1]).c_str(), list.c_str());
    }
    // Four times a second: hold what is hidden, then look for parts that came since (a damage state).
    if (now < s_h.next) return;
    s_h.next = now + 250;
    hold();
    scan(names, frame);
}

} // namespace halo
