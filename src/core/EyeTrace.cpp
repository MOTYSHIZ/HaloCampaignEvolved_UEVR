#include "core/EyeTrace.hpp"

#include "Config.hpp"
#include "HitTrace.hpp"                // hit_trace_passable: the aim ray's look-past rule, shared
#include "UeObject.hpp"                // narrow, class_name_of: naming what a trace started inside
#include "ViewMode.hpp"                // how the eyes are rendered decides how they pair
#include "core/WorldScale.hpp"
#include "uevr/API.hpp"

#include <Windows.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

using namespace uevr;

namespace halo {
namespace eyetrace {

// ---- published by the view callbacks, read by the tick (UE world cm)
std::atomic<float> g_body_x{0.0f}, g_body_y{0.0f}, g_body_z{0.0f};
std::atomic<bool>  g_have_body{false};
std::atomic<float> g_head_cx{0.0f}, g_head_cy{0.0f}, g_head_cz{0.0f};
std::atomic<bool>  g_have_head{false};

void hblog(const char* fmt, ...) {
    if (g_cfg.head_block_log <= 0 && g_cfg.height_log <= 0) return;
    char buf[640];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    API::get()->log_info("[Halo-CampE-UEVR] %s", buf);
}

TraceFn g_line, g_sphere;

namespace {

// ---- view-callback side (one thread)
double   g_pre[2][3]{};
bool     g_have_pre[2]{};
double   g_raw_prev[2][3]{};
bool     g_have_raw[2]{};
uint64_t g_raw_seq[2]{};   // the post sample count when each slot was last written
uint64_t g_post_seq = 0;   // post samples so far, either eye
double   g_off_prev[3]{};  // the previous post sample's offset from its own pre (either slot)
uint64_t g_off_prev_seq = 0;
bool     g_off_prev_have = false;

// ---- reflection-resolved Kismet traces. Every offset comes from the UFunction and the HitResult
// script struct, never from a written-down layout (the same discipline as HitTrace.cpp).
std::wstring field_name(API::FField* f) {
    if (f == nullptr) return L"";
    auto* n = f->get_fname();
    return (n != nullptr) ? n->to_string() : L"";
}

API::FProperty* prop_of(API::UStruct* s, const wchar_t* want) {
    if (s == nullptr) return nullptr;
    for (auto* f = s->get_child_properties(); f != nullptr; f = f->get_next()) {
        if (field_name(f) == want) return reinterpret_cast<API::FProperty*>(f);
    }
    return nullptr;
}

int32_t off_of(API::UStruct* s, const wchar_t* want) {
    auto* p = prop_of(s, want);
    return (p != nullptr) ? p->get_offset() : -1;
}

int             g_tstate = -1;   // -1 unresolved, 0 failed, 1 at least one trace usable
API::UObject*   g_cdo = nullptr;
int32_t         g_hit_impact = -1, g_hit_location = -1;
// OPTIONAL HitResult fields, for looking past a start-inside hit (run_trace). Absent = the trace
// behaves exactly as it always did. Time is read as whatever the reflection says it is.
int32_t         g_hit_time = -1, g_hit_comp = -1;
bool            g_hit_time_double = false;
std::vector<uint8_t> g_buf;

// ---- the head clamp as a fraction of the head offset (eye_clamped_standing_origin)
std::atomic<float>     g_clamp_k{0.0f};
std::atomic<bool>      g_clamp_horiz{false};
std::atomic<long long> g_clamp_ms{0};

long long steady_ms() {
    return (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool resolve_fn(API::UClass* cls, const wchar_t* name, TraceFn* t, bool want_radius) {
    t->name = name;
    t->fn = cls->find_function(name);
    if (t->fn == nullptr) { hblog("HEADBLOCK: %ls not found", name); return false; }
    t->size    = t->fn->get_properties_size();
    t->ctx     = off_of(t->fn, L"WorldContextObject");
    t->start   = off_of(t->fn, L"Start");
    t->end     = off_of(t->fn, L"End");
    t->channel = off_of(t->fn, L"TraceChannel");
    t->ignore  = off_of(t->fn, L"ActorsToIgnore");
    t->out_hit = off_of(t->fn, L"OutHit");
    t->self    = off_of(t->fn, L"bIgnoreSelf");
    t->ret     = off_of(t->fn, L"ReturnValue");
    std::wstring rclass;
    if (want_radius) {
        auto* rp = prop_of(t->fn, L"Radius");
        if (rp != nullptr) {
            auto* fc = rp->get_class();
            auto* fnm = (fc != nullptr) ? fc->get_fname() : nullptr;
            rclass = (fnm != nullptr) ? fnm->to_string() : L"";
            if (rclass == L"FloatProperty" || rclass == L"DoubleProperty") {
                t->radius = rp->get_offset();
                t->radius_double = (rclass == L"DoubleProperty");
            }
        }
    }
    t->ok = t->size > 0 && t->start >= 0 && t->end >= 0 && t->out_hit >= 0 && t->ret >= 0 &&
            (!want_radius || t->radius >= 0) && g_hit_impact >= 0 && g_hit_location >= 0 &&
            t->out_hit + (std::max)(g_hit_impact, g_hit_location) + 24 <= t->size;
    hblog("HEADBLOCK: %ls size=%d ctx=%d start=%d end=%d radius=%d(%ls) channel=%d ignore=%d outhit=%d "
          "self=%d ret=%d -> %s", name, t->size, t->ctx, t->start, t->end, t->radius, rclass.c_str(),
          t->channel, t->ignore, t->out_hit, t->self, t->ret, t->ok ? "OK" : "UNUSABLE");
    return t->ok;
}

}  // namespace

bool traces_ready() {
    if (g_tstate >= 0) return g_tstate == 1;
    g_tstate = 0;
    auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.KismetSystemLibrary");
    if (cls == nullptr) { hblog("HEADBLOCK: KismetSystemLibrary not found -- traces unavailable"); return false; }
    g_cdo = cls->get_class_default_object();
    auto* hit = API::get()->find_uobject<API::UScriptStruct>(L"ScriptStruct /Script/Engine.HitResult");
    if (hit != nullptr) {
        g_hit_impact   = off_of(hit, L"ImpactPoint");
        g_hit_location = off_of(hit, L"Location");
        // FHitResult::Component is a TWeakObjectPtr { int32 ObjectIndex; int32 SerialNumber } --
        // resolved through the object array, as HitTrace.cpp does, never dereferenced as a pointer.
        g_hit_comp     = off_of(hit, L"Component");
        if (auto* tp = prop_of(hit, L"Time")) {
            auto* fc = tp->get_class();
            auto* fnm = (fc != nullptr) ? fc->get_fname() : nullptr;
            const std::wstring tclass = (fnm != nullptr) ? fnm->to_string() : L"";
            if (tclass == L"FloatProperty" || tclass == L"DoubleProperty") {
                g_hit_time = tp->get_offset();
                g_hit_time_double = (tclass == L"DoubleProperty");
            }
        }
    }
    hblog("HEADBLOCK: HitResult ImpactPoint=%d Location=%d Time=%d%s Component=%d", g_hit_impact,
          g_hit_location, g_hit_time, g_hit_time_double ? "(double)" : "", g_hit_comp);
    const bool a = resolve_fn(cls, L"LineTraceSingle", &g_line, false);
    const bool b = resolve_fn(cls, L"SphereTraceSingle", &g_sphere, true);
    g_tstate = (g_cdo != nullptr && (a || b)) ? 1 : 0;
    return g_tstate == 1;
}

namespace {

// ONE call of the reflected trace. Fills the hit's location, impact point, Time (1 when the field did
// not resolve, so a missing field never reads as "started inside") and component.
bool trace_once(const TraceFn& t, API::UObject* world, const Vec3& a, const Vec3& b, float radius,
                int channel, API::UObject* const* ignore, int n_ignore, Vec3* out_loc, Vec3* out_impact,
                float* out_time, API::UObject** out_comp) {
    g_buf.assign((size_t)t.size, 0);
    uint8_t* p = g_buf.data();
    if (t.ctx >= 0) *reinterpret_cast<API::UObject**>(p + t.ctx) = world;
    auto* s = reinterpret_cast<double*>(p + t.start);
    s[0] = a.x; s[1] = a.y; s[2] = a.z;
    auto* e = reinterpret_cast<double*>(p + t.end);
    e[0] = b.x; e[1] = b.y; e[2] = b.z;
    if (t.radius >= 0) {
        if (t.radius_double) *reinterpret_cast<double*>(p + t.radius) = (double)radius;
        else                 *reinterpret_cast<float*>(p + t.radius)  = radius;
    }
    if (t.self >= 0) *(p + t.self) = 1;
    if (t.channel >= 0) *(p + t.channel) = (uint8_t)((channel < 0) ? 0 : (channel > 255 ? 255 : channel));
    // TArray {T* Data; int32 Num; int32 Max} pointed at the caller's array: safe for a native
    // static, which never takes ownership of its parameter frame (see HitTrace.cpp).
    struct FRawArray { void* data; int32_t num; int32_t max; };
    if (t.ignore >= 0 && ignore != nullptr && n_ignore > 0) {
        auto* arr = reinterpret_cast<FRawArray*>(p + t.ignore);
        arr->data = (void*)ignore; arr->num = n_ignore; arr->max = n_ignore;
    }

    g_cdo->call_function(t.name, p);

    if (t.ignore >= 0) {
        auto* arr = reinterpret_cast<FRawArray*>(p + t.ignore);
        arr->data = nullptr; arr->num = 0; arr->max = 0;
    }
    if (*(p + t.ret) == 0) return false;

    const auto* lp = reinterpret_cast<const double*>(p + t.out_hit + g_hit_location);
    const auto* ip = reinterpret_cast<const double*>(p + t.out_hit + g_hit_impact);
    const Vec3 l{(float)lp[0], (float)lp[1], (float)lp[2]};
    const Vec3 i{(float)ip[0], (float)ip[1], (float)ip[2]};
    if (!std::isfinite(l.x) || !std::isfinite(l.y) || !std::isfinite(l.z) ||
        !std::isfinite(i.x) || !std::isfinite(i.y) || !std::isfinite(i.z)) return false;
    *out_loc = l;
    *out_impact = i;

    *out_time = 1.0f;
    if (g_hit_time >= 0 && t.out_hit + g_hit_time + (g_hit_time_double ? 8 : 4) <= t.size) {
        *out_time = g_hit_time_double ? (float)*reinterpret_cast<const double*>(p + t.out_hit + g_hit_time)
                                      : *reinterpret_cast<const float*>(p + t.out_hit + g_hit_time);
    }
    *out_comp = nullptr;
    if (g_hit_comp >= 0 && t.out_hit + g_hit_comp + (int32_t)sizeof(int32_t) * 2 <= t.size) {
        // A stale index yields nothing, which is the right failure -- the same rule as HitTrace.cpp.
        const int32_t idx = *reinterpret_cast<const int32_t*>(p + t.out_hit + g_hit_comp);
        auto* arr = API::get()->get_uobject_array();
        if (arr != nullptr && idx >= 0 && idx < arr->get_object_count()) {
            *out_comp = reinterpret_cast<API::UObject*>(arr->get_object(idx));
        }
    }
    return true;
}

// A BODY, not a wall: a skeletal mesh (the player's biped, a marine standing in him). Everything the
// camera can clip into and must stay stopped by -- walls, rocks, the landscape, instanced foliage --
// is a static mesh of some kind. Remembered for the last component AND its class -- an address can be
// reused by a different object -- so a start-inside hit that repeats every tick costs two compares.
bool is_body_component(API::UObject* comp) {
    static const void* s_comp = nullptr;
    static const void* s_cls  = nullptr;
    static bool        s_body = false;
    if (comp == nullptr) return false;
    const void* cls = comp->get_class();
    if (comp != s_comp || cls != s_cls) {
        s_comp = comp;
        s_cls  = cls;
        s_body = class_name_of(comp).find(L"SkeletalMeshComponent") != std::wstring::npos;
    }
    return s_body;
}

// WHAT THE CAMERA IS INSIDE, by name -- the question the look-past exists to answer (is it the
// player's own biped, a marine, something else?). With headblocklog or heightlog only, once per
// actor and then every 30 s while it persists, never more than once in 2 s.
void note_started_inside(const TraceFn& t, API::UObject* actor) {
    if (g_cfg.head_block_log <= 0 && g_cfg.height_log <= 0) return;
    static const void* s_last = nullptr;
    static ULONGLONG   s_at = 0;
    const ULONGLONG now = GetTickCount64();
    if (now - s_at < 2000) return;
    if (actor == s_last && now - s_at < 30000) return;
    s_last = actor;
    s_at = now;
    std::string name = "?";
    if (const auto* fn = actor->get_fname()) name = narrow(fn->to_string());
    hblog("HEADBLOCK: %ls started INSIDE %s '%s' -- looked past it: the camera is inside it, so it is "
          "not a surface ahead", t.name, narrow(class_name_of(actor)).c_str(), name.c_str());
}

} // namespace

bool run_trace(const TraceFn& t, const Vec3& a, const Vec3& b, float radius, int channel,
               API::UObject* const* ignore, int n_ignore, Vec3* out_loc, Vec3* out_impact,
               TraceHit* out_hit) {
    if (out_hit != nullptr) *out_hit = TraceHit{};
    if (!t.ok || g_cdo == nullptr) return false;
    auto* world = reinterpret_cast<API::UObject*>(API::get()->get_local_pawn(0));
    if (world == nullptr) return false;

    // The caller's list, plus each actor looked past, one at a time. Two reasons to look past a hit:
    //   * PASSABLE (hit_trace_passable, HitTrace.hpp): an actor that blocks the channel without being
    //     anything a round or an eye stops at -- the Library's BP_AtmoArray_C cylinders, which blocked
    //     the aim ray there for five minutes (2026-09-28) and block the Camera channel the head block
    //     shares with it. Any trace, start-inside or not: standing INSIDE a cylinder must not pin the
    //     head. The aim ray's own rule and switch (tracepassthrough), not a second list.
    //   * A BODY the trace started inside (see the header).
    // Room for the extra entries is kept whatever the caller passed.
    constexpr int kExtra = 4;
    constexpr int kMax = 16;
    API::UObject* ig[kMax];
    int n = 0;
    for (int i = 0; ignore != nullptr && i < n_ignore && n < kMax - kExtra; ++i) ig[n++] = ignore[i];

    for (int attempt = 0; attempt <= kExtra; ++attempt) {
        Vec3 loc{}, imp{};
        float time = 1.0f;
        API::UObject* comp = nullptr;
        if (!trace_once(t, world, a, b, radius, channel, ig, n, &loc, &imp, &time, &comp)) return false;
        API::UObject* owner = nullptr;
        const bool passable = hit_trace_passable(comp, &owner);
        // LINE TRACES ONLY (t.radius < 0: the line function has no Radius). A line starts inside
        // something only when the camera POINT is inside it. A sphere starts inside anything within
        // its radius -- a real wall the body stands close to -- and that start hit is the correct
        // answer for a sweep: looking past it would let the head into the wall.
        // BODIES ONLY: a static mesh the camera is inside is a wall it clipped into, and there the
        // start hit is right too -- looking past it would put the head through to the far side.
        const bool inside_body = !(time > 0.0f) && t.radius < 0 && is_body_component(comp);
        if (!passable && !inside_body) {
            *out_loc = loc;
            *out_impact = imp;
            if (out_hit != nullptr) out_hit->component = comp;
            return true;
        }
        // Look past it. Out of attempts, it is no hit -- still not a surface ahead.
        API::UObject* actor = passable ? owner : comp->get_outer();
        if (out_hit != nullptr) {
            if (inside_body) out_hit->inside = actor;
            out_hit->retries = attempt + 1;
        }
        if (actor == nullptr || attempt == kExtra) return false;
        if (!passable) note_started_inside(t, actor);   // a passable class logs itself, once
        ig[n++] = actor;
    }
    return false;
}

} // namespace eyetrace

using namespace eyetrace;

bool eye_body_world(Vec3* out) {
    if (!g_have_body.load(std::memory_order_relaxed)) return false;
    *out = Vec3{g_body_x.load(std::memory_order_relaxed), g_body_y.load(std::memory_order_relaxed),
                g_body_z.load(std::memory_order_relaxed)};
    return true;
}

bool eye_head_offset(Vec3* out) {
    if (!g_have_head.load(std::memory_order_relaxed)) return false;
    *out = Vec3{g_head_cx.load(std::memory_order_relaxed), g_head_cy.load(std::memory_order_relaxed),
                g_head_cz.load(std::memory_order_relaxed)};
    return true;
}

Vec3 eye_clamped_standing_origin(const Vec3& so, const Vec3& hmd) {
    const long long age = steady_ms() - g_clamp_ms.load(std::memory_order_relaxed);
    float k = g_clamp_k.load(std::memory_order_relaxed);
    if (!(k > 0.0f) || age < 0 || age > 250) return so;
    if (k > 1.0f) k = 1.0f;
    Vec3 o{so.x + (hmd.x - so.x) * k, so.y + (hmd.y - so.y) * k, so.z + (hmd.z - so.z) * k};
    if (g_clamp_horiz.load(std::memory_order_relaxed)) o.y = so.y;   // room Y is up
    return o;
}

bool kismet_line_trace(const Vec3& a, const Vec3& b, API::UObject* const* ignore, int n_ignore,
                       int channel, Vec3* out_impact) {
    if (!traces_ready() || !g_line.ok) return false;
    Vec3 loc{};
    return run_trace(g_line, a, b, 0.0f, channel, ignore, n_ignore, &loc, out_impact);
}

namespace {

void eye_note_pre(int index, double x, double y, double z) {
    if (index < 0 || index > 1) return;
    g_pre[index][0] = x; g_pre[index][1] = y; g_pre[index][2] = z;
    g_have_pre[index] = true;
    if (index == 0) {
        g_body_x.store((float)x, std::memory_order_relaxed);
        g_body_y.store((float)y, std::memory_order_relaxed);
        g_body_z.store((float)z, std::memory_order_relaxed);
        g_have_body.store(true, std::memory_order_relaxed);
    }
}

bool eye_note_post(int index, double* x, double* y, double* z, const HeadClamp* clamp) {
    if (index < 0 || index > 1 || !g_have_pre[index]) return false;
    const double raw[3] = {*x, *y, *z};
    const int mode = (clamp != nullptr) ? clamp->mode() : 0;
    bool moved = false;

    // ONE HEAD OFFSET PER FRAME, computed on eye 0 and reused on eye 1, so both eyes move by the same
    // vector and the stereo separation is untouched. Published always: auto height logs it as the
    // measured view height.
    if (index == 0 || !g_have_raw[0]) {
        // The head CENTRE's offset from the body: this eye's offset, corrected by half of last
        // frame's eye-to-eye vector (eye 0 sits half a separation to one side of the centre).
        //
        // ONLY FROM A REAL PAIR: the two eyes rendered back to back. A slot that stops being fed keeps
        // its last value forever, so "both slots ever seen" pairs a live eye with a frozen one once
        // the rendering method changes under a running game -- Native Stereo -> Mono stops eye 1 --
        // and half of the gap between them lands in the head offset, growing with every metre walked
        // since the switch. 2026-09-27, Truth and Reconciliation: a level load put ~840 m between the
        // eye frozen at the switch and the live one, the head offset read 419 m, and the head block's
        // trace, hitting the ground 2.6 m along it, pushed the rendered eye 416 m into the sky.
        // AimConverge learned the same lesson (its CYCLOPEAN EYE note). And a separation no head can
        // have is not one: half a real metre is several times any IPD, at any world scale.
        const double cap = 0.5 * (double)uevr_cm_per_metre_cached();
        double c[3];
        const ViewMode vm = viewmode_current();
        if (vm == ViewMode::Alternating || vm == ViewMode::Unknown) {
            // ONE VIEW PER FRAME, THE EYES TAKING TURNS (AFR, Synchronized Sequential; Unknown while
            // the verdict is pending). Every sample arrives as index 0, so there is never an eye-1 slot
            // to pair with, and one eye's offset alone carries its half IPD: the clamp, a fraction of
            // it, then moved each eye by a DIFFERENT vector and pulled the two views together by the
            // clamp fraction -- at a full clamp, no stereo at all. The head centre is this eye's offset
            // averaged with the previous sample's, which was the other eye (AimConverge averages its
            // deltas the same way). Only when that sample came straight before this one, and within the
            // same half-metre cap: anything else falls back to this eye alone, as before. Harmless if
            // the pending verdict turns out to be Mono: two centre-eye samples average to the centre.
            double off[3];
            double m2 = 0.0;
            for (int k = 0; k < 3; ++k) {
                off[k] = raw[k] - g_pre[index][k];
                const double d = off[k] - g_off_prev[k];
                m2 += d * d;
            }
            const bool pair = g_off_prev_have && g_off_prev_seq == g_post_seq && m2 <= cap * cap;
            for (int k = 0; k < 3; ++k) c[k] = pair ? 0.5 * (off[k] + g_off_prev[k]) : off[k];
        } else {
            // Two views per frame (Native Stereo), or Mono's one centre eye (where no eye-1 slot is
            // live and e stays zero).
            double e[3] = {0.0, 0.0, 0.0};
            if (g_have_raw[0] && g_have_raw[1] && g_raw_seq[1] == g_raw_seq[0] + 1) {
                double m2 = 0.0;
                for (int k = 0; k < 3; ++k) {
                    e[k] = g_raw_prev[1][k] - g_raw_prev[0][k];
                    m2 += e[k] * e[k];
                }
                if (!(m2 <= cap * cap)) e[0] = e[1] = e[2] = 0.0;
            }
            const double half = (index == 0) ? 0.5 : -0.5;
            for (int k = 0; k < 3; ++k) c[k] = raw[k] - g_pre[index][k] + half * e[k];
        }
        g_head_cx.store((float)c[0], std::memory_order_relaxed);
        g_head_cy.store((float)c[1], std::memory_order_relaxed);
        g_head_cz.store((float)c[2], std::memory_order_relaxed);
        g_have_head.store(true, std::memory_order_relaxed);

        // The clamp's fraction, published with its time for eye_clamped_standing_origin -- 0 when
        // nothing clamps, so a clamp that switches off stops moving the hands on the next frame.
        bool horiz = false;
        const float k = (clamp != nullptr) ? clamp->shift(mode, c, &horiz) : 0.0f;
        g_clamp_k.store((std::isfinite(k) && k > 0.0f) ? k : 0.0f, std::memory_order_relaxed);
        g_clamp_horiz.store(horiz, std::memory_order_relaxed);
        g_clamp_ms.store(steady_ms(), std::memory_order_relaxed);
    }
    if (clamp != nullptr && clamp->apply(mode, raw, x, y, z)) moved = true;

    for (int k = 0; k < 3; ++k) {
        g_raw_prev[index][k] = raw[k];
        g_off_prev[k] = raw[k] - g_pre[index][k];
    }
    g_have_raw[index] = true;
    g_raw_seq[index] = ++g_post_seq;
    g_off_prev_seq  = g_post_seq;
    g_off_prev_have = true;
    return moved;
}

} // namespace

void eye_note_pre_view(int index, UEVR_Vector3f* position, bool is_double) {
            // The body's eye, at full precision (LWC world coordinates).
            if (is_double) {
                auto* p = reinterpret_cast<UEVR_Vector3d*>(position);
                halo::eye_note_pre(index, p->x, p->y, p->z);
            } else {
                halo::eye_note_pre(index, position->x, position->y, position->z);
            }
}

void eye_note_post_view(int index, UEVR_Vector3f* position, bool is_double, const HeadClamp* clamp) {
        // THE HEAD OFFSET, and the head block's pull-back of the composed eye out of geometry, BEFORE
        // anything after this callback's hook publishes it, so markers and the reticule reason from
        // the eye that is actually rendered.
        if (position != nullptr) {
            if (is_double) {
                auto* p = reinterpret_cast<UEVR_Vector3d*>(position);
                double hx = p->x, hy = p->y, hz = p->z;
                if (halo::eye_note_post(index, &hx, &hy, &hz, clamp)) { p->x = hx; p->y = hy; p->z = hz; }
            } else {
                double hx = position->x, hy = position->y, hz = position->z;
                if (halo::eye_note_post(index, &hx, &hy, &hz, clamp)) {
                    position->x = (float)hx; position->y = (float)hy; position->z = (float)hz;
                }
            }
        }
}

} // namespace halo
