#include "core/EyeTrace.hpp"

#include "Config.hpp"
#include "uevr/API.hpp"

#include <Windows.h>
#include <atomic>
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
double g_pre[2][3]{};
bool   g_have_pre[2]{};
double g_raw_prev[2][3]{};
bool   g_have_raw[2]{};

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
std::vector<uint8_t> g_buf;

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
    }
    hblog("HEADBLOCK: HitResult ImpactPoint=%d Location=%d", g_hit_impact, g_hit_location);
    const bool a = resolve_fn(cls, L"LineTraceSingle", &g_line, false);
    const bool b = resolve_fn(cls, L"SphereTraceSingle", &g_sphere, true);
    g_tstate = (g_cdo != nullptr && (a || b)) ? 1 : 0;
    return g_tstate == 1;
}

bool run_trace(const TraceFn& t, const Vec3& a, const Vec3& b, float radius, int channel,
               API::UObject* const* ignore, int n_ignore, Vec3* out_loc, Vec3* out_impact) {
    if (!t.ok || g_cdo == nullptr) return false;
    auto* world = reinterpret_cast<API::UObject*>(API::get()->get_local_pawn(0));
    if (world == nullptr) return false;

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
    return true;
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
        double e[3] = {0.0, 0.0, 0.0};
        if (g_have_raw[0] && g_have_raw[1]) {
            for (int k = 0; k < 3; ++k) e[k] = g_raw_prev[1][k] - g_raw_prev[0][k];
        }
        const double half = (index == 0) ? 0.5 : -0.5;
        double c[3];
        for (int k = 0; k < 3; ++k) c[k] = raw[k] - g_pre[index][k] + half * e[k];
        g_head_cx.store((float)c[0], std::memory_order_relaxed);
        g_head_cy.store((float)c[1], std::memory_order_relaxed);
        g_head_cz.store((float)c[2], std::memory_order_relaxed);
        g_have_head.store(true, std::memory_order_relaxed);

        if (clamp != nullptr) clamp->shift(mode, c);
    }
    if (clamp != nullptr && clamp->apply(mode, raw, x, y, z)) moved = true;

    for (int k = 0; k < 3; ++k) g_raw_prev[index][k] = raw[k];
    g_have_raw[index] = true;
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
