#include "HitTrace.hpp"

#include "Config.hpp"
#include "UeObject.hpp"   // narrow(), class_name_of() -- used by the dev hit readout
#include "uevr/API.hpp"

#include <Windows.h>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

using namespace uevr;

namespace halo {
namespace {

// -1 not tried, 0 failed (sticky), 1 usable.
int g_state = -1;

API::UObject*   g_cdo  = nullptr;
API::UFunction* g_func = nullptr;
int32_t         g_params_size = 0;

// Parameter offsets, all resolved from reflection. -1 means "not found", which fails the resolve.
struct Offsets {
    int32_t world_ctx  = -1;
    int32_t start      = -1;
    int32_t end        = -1;
    int32_t channel    = -1;
    int32_t complex    = -1;
    int32_t ignore     = -1;
    int32_t debug      = -1;
    int32_t out_hit    = -1;
    int32_t self       = -1;
    int32_t ret        = -1;
} g_off;

// Offset of the impact point WITHIN FHitResult.
int32_t g_impact_off = -1;
// Offset of FHitResult::Component (a TWeakObjectPtr), and the component the last trace hit.
int32_t g_hitcomp_off = -1;
uevr::API::UObject* g_last_hit_component = nullptr;

std::wstring field_name(API::FField* f) {
    if (f == nullptr) return L"";
    auto* n = f->get_fname();
    return (n != nullptr) ? n->to_string() : L"";
}

// Walk a struct's own child properties, matching by name. Deliberately does NOT walk the super
// chain: a function's parameters are all declared on the function itself, and FHitResult's fields
// on FHitResult, so a match found further up would be a different field with a colliding name.
int32_t offset_of(API::UStruct* s, const wchar_t* want) {
    if (s == nullptr) return -1;
    for (auto* f = s->get_child_properties(); f != nullptr; f = f->get_next()) {
        if (field_name(f) == want) {
            return reinterpret_cast<API::FProperty*>(f)->get_offset();
        }
    }
    return -1;
}

void log_off(const char* what, int32_t v) {
    API::get()->log_info("[Halo-CampE-UEVR]   HITTRACE %-18s offset=%d%s", what, v,
                         (v < 0) ? "   <== MISSING" : "");
}

bool resolve() {
    auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.KismetSystemLibrary");
    if (cls == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] HITTRACE: KismetSystemLibrary not found -- reticule "
                             "stays at its fixed distance");
        return false;
    }
    g_cdo  = cls->get_class_default_object();
    g_func = cls->find_function(L"LineTraceSingle");
    if (g_cdo == nullptr || g_func == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] HITTRACE: LineTraceSingle not found (cdo=%p fn=%p)",
                             (void*)g_cdo, (void*)g_func);
        return false;
    }

    // The engine's own size for the parameter block. Allocating by hand-summed field sizes is how a
    // reflection call ends up writing past the end of its own frame.
    g_params_size = g_func->get_properties_size();

    g_off.world_ctx = offset_of(g_func, L"WorldContextObject");
    g_off.start     = offset_of(g_func, L"Start");
    g_off.end       = offset_of(g_func, L"End");
    g_off.channel   = offset_of(g_func, L"TraceChannel");
    g_off.complex   = offset_of(g_func, L"bTraceComplex");
    g_off.ignore    = offset_of(g_func, L"ActorsToIgnore");
    g_off.debug     = offset_of(g_func, L"DrawDebugType");
    g_off.out_hit   = offset_of(g_func, L"OutHit");
    g_off.self      = offset_of(g_func, L"bIgnoreSelf");
    g_off.ret       = offset_of(g_func, L"ReturnValue");

    auto* hit = API::get()->find_uobject<API::UScriptStruct>(L"ScriptStruct /Script/Engine.HitResult");
    if (hit != nullptr) {
        // ImpactPoint is the surface point; Location is the swept shape's centre and equals it for
        // a line trace. Prefer ImpactPoint, accept Location, so a renamed field is not fatal.
        g_impact_off = offset_of(hit, L"ImpactPoint");
        if (g_impact_off < 0) g_impact_off = offset_of(hit, L"Location");

        // WHAT WE HIT, not just where. The scope cannot render this game's shield meshes (they
        // are skeletal meshes whose shading comes from a post pass the engine force-disables for
        // scene captures), so a player sighting through the pane can be aimed into a shield they
        // cannot see. Knowing the hit's COMPONENT lets the mod say so.
        // FHitResult::Component is a TWeakObjectPtr: { int32 ObjectIndex; int32 SerialNumber }.
        // Reading the index and resolving it through the global object array is the only route --
        // there is no reflected getter, and dereferencing the weak pointer's target directly is
        // not something this API exposes.
        g_hitcomp_off = offset_of(hit, L"Component");
    }

    API::get()->log_info("[Halo-CampE-UEVR] HITTRACE: resolving LineTraceSingle, params_size=%d",
                         g_params_size);
    log_off("WorldContext", g_off.world_ctx);
    log_off("Start",        g_off.start);
    log_off("End",          g_off.end);
    log_off("TraceChannel", g_off.channel);
    log_off("OutHit",       g_off.out_hit);
    log_off("ReturnValue",  g_off.ret);
    log_off("HitResult.Impact", g_impact_off);
    log_off("HitResult.Component", g_hitcomp_off);

    // FAIL CLOSED. Only the fields actually written or read are required -- the optional ones
    // (bTraceComplex, ActorsToIgnore, DrawDebugType, bIgnoreSelf) are left at the zeroed default,
    // which is the value we want for each, so a missing offset there costs nothing.
    const bool ok = (g_params_size > 0) && (g_off.start >= 0) && (g_off.end >= 0)
                 && (g_off.out_hit >= 0) && (g_off.ret >= 0) && (g_impact_off >= 0)
                 && (g_off.out_hit + g_impact_off + 24 <= g_params_size);
    if (!ok) {
        API::get()->log_info("[Halo-CampE-UEVR] HITTRACE: resolve FAILED -- tracing disabled, the "
                             "reticule keeps its fixed distance. Nothing is being guessed.");
    }
    return ok;
}

#if HALO_VR_DEV
// Dump the project's NAMED collision channels so aimreticuletracechannel can be chosen by name
// rather than by walking indices.
//
// READ IT KNOWING WHAT IT CANNOT TELL YOU. Halo's projectiles do not use UE collision at all: the
// Blam sim carries its own collision BSP (`global_collision_bsp_struct`, `collision_model`, leaf and
// surface traversal) and `HaloSimulation_tag_release.dll` references no UE collision symbols
// whatsoever -- no ECC_, no TraceTypeQuery, no LineTraceSingle. So none of these IS the projectile
// channel. The best any of them can be is the closest APPROXIMATION of the same world, and a
// channel named Weapon or Projectile is a better guess than Visibility, not a guarantee.
//
// Offsets resolved from reflection, same reason as the trace's (see HitTrace.hpp).
void dump_collision_channels() {
    auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.CollisionProfile");
    if (cls == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] CHANNELS: CollisionProfile not found"); return; }
    auto* cdo = cls->get_class_default_object();
    const int32_t arr_off = offset_of(cls, L"DefaultChannelResponses");
    if (cdo == nullptr || arr_off < 0) {
        API::get()->log_info("[Halo-CampE-UEVR] CHANNELS: cdo=%p DefaultChannelResponses=%d",
                             (void*)cdo, arr_off);
        return;
    }

    auto* st = API::get()->find_uobject<API::UScriptStruct>(L"ScriptStruct /Script/Engine.CustomChannelSetup");
    const int32_t n_off = (st != nullptr) ? offset_of(st, L"Name")       : -1;
    const int32_t c_off = (st != nullptr) ? offset_of(st, L"Channel")    : -1;
    const int32_t t_off = (st != nullptr) ? offset_of(st, L"bTraceType") : -1;
    const int32_t elem  = (st != nullptr) ? st->get_struct_size()        : 0;
    if (n_off < 0 || c_off < 0 || elem <= 0) {
        API::get()->log_info("[Halo-CampE-UEVR] CHANNELS: CustomChannelSetup layout unresolved "
                             "(name=%d channel=%d trace=%d size=%d)", n_off, c_off, t_off, elem);
        return;
    }

    struct FRawArray { uint8_t* data; int32_t num; int32_t max; };
    const auto* arr = reinterpret_cast<const FRawArray*>(reinterpret_cast<const uint8_t*>(cdo) + arr_off);
    if (IsBadReadPtr(arr, sizeof(FRawArray)) || arr->data == nullptr || arr->num <= 0 || arr->num > 64) {
        API::get()->log_info("[Halo-CampE-UEVR] CHANNELS: no custom channels (num=%d)",
                             arr ? arr->num : -1);
        return;
    }

    API::get()->log_info("[Halo-CampE-UEVR] CHANNELS: %d custom channels. aimreticuletracechannel is "
                         "an ETraceTypeQuery INDEX over the TRACE ones only: 0=Visibility, 1=Camera, "
                         "then each trace channel below in order.", arr->num);
    int trace_idx = 2;
    for (int i = 0; i < arr->num; ++i) {
        const uint8_t* e = arr->data + (size_t)i * (size_t)elem;
        if (IsBadReadPtr(e, (size_t)elem)) break;
        const auto* nm  = reinterpret_cast<const API::FName*>(e + n_off);
        const int   ch  = (int)*(e + c_off);
        const bool  tr  = (t_off >= 0) && (*(e + t_off) != 0);
        if (tr) {
            API::get()->log_info("[Halo-CampE-UEVR]   ch%-3d %-28ls TRACE  <- aimreticuletracechannel=%d",
                                 ch, nm ? nm->to_string().c_str() : L"?", trace_idx++);
        } else {
            API::get()->log_info("[Halo-CampE-UEVR]   ch%-3d %-28ls object",
                                 ch, nm ? nm->to_string().c_str() : L"?");
        }
    }
}
#endif

}  // namespace

#if HALO_VR_DEV
void hit_trace_dump_channels() { dump_collision_channels(); }
#endif

bool hit_trace_ready() {
    if (g_state < 0) g_state = resolve() ? 1 : 0;
    return g_state == 1;
}

bool hit_trace(const Vec3& start, const Vec3& end,
               API::UObject* const* ignore, int ignore_count, Vec3* out_hit) {
    if (!hit_trace_ready() || out_hit == nullptr) return false;

    auto* world = reinterpret_cast<API::UObject*>(API::get()->get_local_pawn(0));
    if (world == nullptr) return false;

    // Zeroed whole: every field we do NOT set wants its zero value (no complex trace, no actors
    // ignored, no debug draw, channel 0 = Visibility), so a zero fill is correct by construction
    // rather than by our remembering to set each one.
    //
    // ONE BUFFER, REUSED. The zero fill is required every call; the ALLOCATION is not, and this
    // runs on the reticule's per-tick trace, so a fresh vector here was a malloc/free per tick for
    // the life of the session. assign() rewrites the bytes without reallocating after the first
    // call.
    //
    // Deliberately not a fixed stack array like RIG_PARAM_BUF: g_params_size comes from the
    // engine's own UFunction at runtime, and a compile-time cap that turned out to be too small on
    // some future build would silently disable tracing -- which would take the reticule AND the
    // aim convergence with it. Sizing from the engine cannot be wrong.
    //
    // Single-threaded by the same argument as the rest of this file: every hit_trace() call site
    // is inside update(), on the tick thread.
    static std::vector<uint8_t> params;
    params.assign((size_t)g_params_size, 0);
    uint8_t* p = params.data();

    if (g_off.world_ctx >= 0) *reinterpret_cast<API::UObject**>(p + g_off.world_ctx) = world;

    // FVector is DOUBLE under LWC -- 24 bytes, not 12. Getting this wrong writes half a vector and
    // leaves the rest as garbage from the frame.
    auto* s = reinterpret_cast<double*>(p + g_off.start);
    s[0] = start.x; s[1] = start.y; s[2] = start.z;
    auto* e = reinterpret_cast<double*>(p + g_off.end);
    e[0] = end.x; e[1] = end.y; e[2] = end.z;

    // IGNORE THE PLAYER'S OWN GEOMETRY.
    //
    // bIgnoreSelf covers the WorldContextObject's actor -- the pawn -- and nothing else. The gun is
    // its own actor attached to the rig, so it needs listing explicitly or a reload animation
    // swinging it across the camera puts the reticule on the gun, in your face.
    if (g_off.self >= 0) *(p + g_off.self) = 1;

    // ETraceTypeQuery is a uint8 enum. 0 (the zeroed default) is TraceTypeQuery1 = Visibility,
    // which is blocked by things a bullet passes straight through -- invisible blocking volumes,
    // triggers. See aim_reticule_trace_channel for why this is a live knob and not a constant.
    if (g_off.channel >= 0) {
        const int ch = g_cfg.aim_reticule_trace_channel;
        *(p + g_off.channel) = (uint8_t)((ch < 0) ? 0 : (ch > 255 ? 255 : ch));
    }

    // TArray is {T* Data; int32 Num; int32 Max}. Pointed at the CALLER'S buffer, which is safe
    // here and only here: LineTraceSingle is a native static, so ProcessEvent hands this frame
    // straight to the native thunk and never takes ownership of or destroys the parameters. Do not
    // copy this pattern to a BLUEPRINT function, which allocates and destroys its own frame.
    struct FRawArray { void* data; int32_t num; int32_t max; };
    if (g_off.ignore >= 0 && ignore != nullptr && ignore_count > 0) {
        auto* arr = reinterpret_cast<FRawArray*>(p + g_off.ignore);
        arr->data = (void*)ignore;
        arr->num  = ignore_count;
        arr->max  = ignore_count;
    }

    g_cdo->call_function(L"LineTraceSingle", p);

    // Drop our borrowed pointer before the buffer dies, so nothing downstream can mistake this
    // frame for an array it owns.
    if (g_off.ignore >= 0) {
        auto* arr = reinterpret_cast<FRawArray*>(p + g_off.ignore);
        arr->data = nullptr; arr->num = 0; arr->max = 0;
    }

    if (*(p + g_off.ret) == 0) return false;   // no blocking hit

    const auto* ip = reinterpret_cast<const double*>(p + g_off.out_hit + g_impact_off);
    const Vec3 h{(float)ip[0], (float)ip[1], (float)ip[2]};
    if (!std::isfinite(h.x) || !std::isfinite(h.y) || !std::isfinite(h.z)) return false;
    *out_hit = h;

    // WHAT was hit. Published rather than returned so every existing caller is untouched.
    g_last_hit_component = nullptr;
    if (g_hitcomp_off >= 0 &&
        g_off.out_hit + g_hitcomp_off + (int32_t)sizeof(int32_t) * 2 <= g_params_size) {
        // TWeakObjectPtr { int32 ObjectIndex; int32 ObjectSerialNumber }. Resolve the index
        // through the global object array; a stale index simply yields nothing, which is the
        // correct failure here -- never dereference it as a raw pointer.
        const int32_t idx = *reinterpret_cast<const int32_t*>(p + g_off.out_hit + g_hitcomp_off);
        auto* arr = API::get()->get_uobject_array();
        if (arr != nullptr && idx >= 0 && idx < arr->get_object_count()) {
            g_last_hit_component = reinterpret_cast<API::UObject*>(arr->get_object(idx));
        }
    }
    return true;
}

// The component the last successful trace hit, or nullptr. Valid only until the next trace; the
// reticule traces every tick, so treat it as this-tick-only and never store it.
API::UObject* hit_trace_last_component() { return g_last_hit_component; }

#if HALO_VR_DEV
// Find a component of a given class on an actor, WITHOUT sweeping the object array -- the same
// route resolve_shield_shell() uses, for the same reason: a full sweep here is the pattern that
// has already collapsed framerate in a live session. Both arrays are plain TArray<UObject*>.
API::UObject* find_component_on(API::UObject* actor, const wchar_t* class_substr) {
    if (actor == nullptr) return nullptr;
    struct FRawArray { API::UObject** data; int32_t num; int32_t max; };
    static const wchar_t* kArrays[] = { L"BlueprintCreatedComponents", L"InstanceComponents" };
    for (const wchar_t* arr_name : kArrays) {
        auto* arr = actor->get_property_data<FRawArray>(arr_name);
        if (arr == nullptr || arr->data == nullptr || arr->num <= 0 || arr->num > 512) continue;
        for (int32_t i = 0; i < arr->num; ++i) {
            auto* c = arr->data[i];
            if (c == nullptr) continue;
            if (class_name_of(c).find(class_substr) != std::wstring::npos) return c;
        }
    }
    return nullptr;
}

// Read a float from a 0-arg reflected getter. Returns false if the function is absent.
bool call_float_getter(API::UObject* obj, const wchar_t* fn, float* out) {
    if (obj == nullptr || out == nullptr) return false;
    auto* cls = obj->get_class();
    if (cls == nullptr || cls->find_function(fn) == nullptr) return false;
    alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
    obj->call_function(fn, q);
    *out = *reinterpret_cast<float*>(q);
    return true;
}

// ONE-SHOT: prove the shield getters answer against a LIVE enemy, without needing to aim at one.
//
// Aiming at an Elite under SimVR is impractical (no combat, and the ray has to land on the biped),
// but the question that actually matters -- do these reflected getters return real values? -- does
// not need the ray at all. Find any live enemy biped, read its damage component, log what comes
// back. One full array walk, ONCE, dev builds only.
void hit_trace_dev_shield_probe() {
    static bool done = false;
    if (done) return;
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;
    done = true;

    const int32_t n = arr->get_object_count();
    int found = 0;
    for (int32_t i = 0; i < n && found < 3; ++i) {
        auto* o = reinterpret_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr) continue;
        if (class_name_of(o).find(L"BlamObjectDamageComponent") == std::wstring::npos) continue;
        // LIVE INSTANCES ONLY. Filtering on the owner's short name was wrong twice over: a class
        // default object's outer is the PACKAGE ("/Script/BlamSynchronization"), and a Blueprint
        // archetype's is the generated class ("BP_BaseVehicleActor_C") -- neither contains
        // "Default__", so both sailed through and reported a DEFAULT vitality of 1.000 that looked
        // exactly like a healthy live enemy. A real in-world component's full name always sits
        // under the level: /Game/Levels/.../PersistentLevel.<Actor>.<Component>.
        const std::string full = narrow(o->get_full_name());
        if (full.find("PersistentLevel") == std::string::npos) continue;
        if (full.find("Default__") != std::string::npos) continue;

        auto* owner = o->get_outer();
        std::string owner_name = "?";
        if (owner != nullptr) {
            if (const auto* fn = owner->get_fname()) owner_name = narrow(fn->to_string());
        }

        float vit = -1.0f, act = -1.0f, over = -1.0f;
        const bool a = call_float_getter(o, L"GetShieldVitality", &vit);
        const bool b = call_float_getter(o, L"GetActiveShieldVitality", &act);
        const bool c = call_float_getter(o, L"GetOvershieldAmount", &over);
        API::get()->log_info("[Halo-CampE-UEVR] SHIELDPROBE owner='%s' vitality=%s%.3f "
                             "active=%s%.3f overshield=%s%.3f",
                             owner_name.c_str(), a ? "" : "(absent)", vit,
                             b ? "" : "(absent)", act, c ? "" : "(absent)", over);
        ++found;
    }
    if (found == 0) {
        API::get()->log_info("[Halo-CampE-UEVR] SHIELDPROBE: no live BlamObjectDamageComponent "
                             "found -- enemies may not be streamed in here");
    }
}

// Names what the aim ray is on, ON CHANGE ONLY. This is the instrument that decides whether
// shield detection is even possible: if an Elite's shield mesh or a Covenant portable shield
// shows up here when the ray is on one, the mod can warn about a shield it cannot draw. If the
// ray reports only the biped, detection has to come from somewhere else.
void hit_trace_dev_report() {
    auto* comp = g_last_hit_component;
    static void* last = nullptr;
    if (comp == last) return;
    last = comp;
    if (comp == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] HITWHAT: (nothing resolved)");
        return;
    }
    // Component class + its own name, and the owning actor's name -- the actor is what carries
    // the recognisable identity (BP_CovPortableShield*, the Elite bipeds), while the component
    // name is what distinguishes a shield mesh from the body mesh on the same actor.
    std::string comp_name = "?";
    if (const auto* fn = comp->get_fname()) comp_name = narrow(fn->to_string());
    std::string owner = "?";
    if (auto* o = comp->get_outer()) {
        if (const auto* fn = o->get_fname()) owner = narrow(fn->to_string());
    }
    API::get()->log_info("[Halo-CampE-UEVR] HITWHAT: %s  comp='%s'  owner='%s'",
                         narrow(class_name_of(comp)).c_str(), comp_name.c_str(), owner.c_str());

    // SHIELD STATE, asked of the SIMULATION rather than inferred from what rendered.
    //
    // The scope cannot draw this game's shields (their shading comes from a post pass the engine
    // force-disables for scene captures), so the requirement -- see enemy state, never fire blind
    // into an invisible stationary shield -- is met by READING the state instead. Both components
    // below are reflected on this build; this is the readout that proves the calls actually work
    // against a live object, which is the one thing the object-array search could not establish.
    auto* actor = comp->get_outer();
    if (actor == nullptr) return;

    if (auto* dmg = find_component_on(actor, L"BlamObjectDamageComponent")) {
        float shield = -1.0f, active = -1.0f, over = -1.0f;
        const bool a = call_float_getter(dmg, L"GetShieldVitality", &shield);
        const bool b = call_float_getter(dmg, L"GetActiveShieldVitality", &active);
        const bool c = call_float_getter(dmg, L"GetOvershieldAmount", &over);
        API::get()->log_info("[Halo-CampE-UEVR] HITSHIELD: vitality=%s%.3f active=%s%.3f "
                             "overshield=%s%.3f",
                             a ? "" : "(absent)", shield, b ? "" : "(absent)", active,
                             c ? "" : "(absent)", over);
    }
    if (auto* dev = find_component_on(actor, L"BlamDeviceMachineComponent")) {
        alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
        dev->call_function(L"IsShielded", q);
        API::get()->log_info("[Halo-CampE-UEVR] HITSHIELD: device IsShielded=%d  <-- a stationary "
                             "shield reports here", (int)q[0]);
    }
}
#endif

}  // namespace halo
