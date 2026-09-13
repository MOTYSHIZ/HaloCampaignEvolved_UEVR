#include "HeadBlock.hpp"

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
namespace {

// ---- published by the view callbacks, read by the tick (UE world cm)
std::atomic<float> g_body_x{0.0f}, g_body_y{0.0f}, g_body_z{0.0f};
std::atomic<float> g_head_cx{0.0f}, g_head_cy{0.0f}, g_head_cz{0.0f};
std::atomic<bool>  g_have_head{false};
// ---- published by the tick, read by the view callbacks
std::atomic<float> g_allow{-1.0f};    // how far the head may extend from the body, UE cm; < 0 = unlimited
std::atomic<int>   g_eff_mode{0};     // the mode actually running (a trace mode falls back when unresolved)
std::atomic<float> g_pushed{0.0f};    // last applied pull-back, UE cm, for the log

// ---- view-callback side (one thread)
double g_pre[2][3]{};
bool   g_have_pre[2]{};
double g_raw_prev[2][3]{};
bool   g_have_raw[2]{};
double g_shift[3]{};

void hblog(const char* fmt, ...) {
    if (g_cfg.head_block_log <= 0) return;
    char buf[640];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    API::get()->log_info("[Halo-CampE-UEVR] %s", buf);
}

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

struct TraceFn {
    const wchar_t*  name = nullptr;
    API::UFunction* fn = nullptr;
    int32_t size = 0;
    int32_t ctx = -1, start = -1, end = -1, radius = -1, channel = -1, ignore = -1, out_hit = -1,
            self = -1, ret = -1;
    bool radius_double = false;
    bool ok = false;
};

int             g_tstate = -1;   // -1 unresolved, 0 failed, 1 at least one trace usable
API::UObject*   g_cdo = nullptr;
TraceFn         g_line, g_sphere;
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

bool run_trace(const TraceFn& t, const Vec3& a, const Vec3& b, float radius,
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
    if (t.channel >= 0) {
        const int ch = g_cfg.head_block_channel;
        *(p + t.channel) = (uint8_t)((ch < 0) ? 0 : (ch > 255 ? 255 : ch));
    }
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

float dist(const Vec3& a, const Vec3& b) {
    const float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

const char* mode_name(int m) {
    switch (m) {
    case 1: return "line-trace";
    case 2: return "sphere-sweep";
    case 3: return "lean-limit";
    default: return "off";
    }
}

}  // namespace

void headblock_note_pre(int index, double x, double y, double z) {
    if (index < 0 || index > 1) return;
    g_pre[index][0] = x; g_pre[index][1] = y; g_pre[index][2] = z;
    g_have_pre[index] = true;
    if (index == 0) {
        g_body_x.store((float)x, std::memory_order_relaxed);
        g_body_y.store((float)y, std::memory_order_relaxed);
        g_body_z.store((float)z, std::memory_order_relaxed);
    }
}

bool headblock_apply_post(int index, double* x, double* y, double* z) {
    if (index < 0 || index > 1 || !g_have_pre[index]) return false;
    const double raw[3] = {*x, *y, *z};
    const int mode = g_eff_mode.load(std::memory_order_relaxed);
    bool moved = false;

    if (mode != 0) {
        // ONE PULL-BACK PER FRAME, computed on eye 0 and reused on eye 1, so both eyes move by the
        // same vector and the stereo separation is untouched.
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

            double s[3] = {0.0, 0.0, 0.0};
            if (mode == 3) {
                // Horizontal only (UE Z is up): a crouch is not a lean.
                const double lean = g_cfg.head_block_lean;
                const double lh = std::sqrt(c[0] * c[0] + c[1] * c[1]);
                if (lh > lean && lh > 1e-6) {
                    const double k = 1.0 - lean / lh;
                    s[0] = c[0] * k; s[1] = c[1] * k;
                }
            } else {
                const double L = g_allow.load(std::memory_order_relaxed);
                const double len = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
                if (L >= 0.0 && len > L && len > 1e-6) {
                    const double k = 1.0 - L / len;
                    for (int k2 = 0; k2 < 3; ++k2) s[k2] = c[k2] * k;
                }
            }
            for (int k = 0; k < 3; ++k) g_shift[k] = s[k];
            g_pushed.store((float)std::sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]),
                           std::memory_order_relaxed);
        }
        if (g_shift[0] != 0.0 || g_shift[1] != 0.0 || g_shift[2] != 0.0) {
            *x = raw[0] - g_shift[0];
            *y = raw[1] - g_shift[1];
            *z = raw[2] - g_shift[2];
            moved = true;
        }
    } else {
        g_shift[0] = g_shift[1] = g_shift[2] = 0.0;
    }

    for (int k = 0; k < 3; ++k) g_raw_prev[index][k] = raw[k];
    g_have_raw[index] = true;
    return moved;
}

void headblock_tick(bool active, API::UObject* const* ignore, int n_ignore, float dt) {
    static int   s_cfg_mode = 0;
    static int   s_eff = -1;
    static float s_L = -1.0f;
    static bool  s_hit_prev = false;

    const int cfg_mode = (g_cfg.head_block >= 1 && g_cfg.head_block <= 3) ? g_cfg.head_block : 0;
    if (cfg_mode != s_cfg_mode) {
        hblog("HEADBLOCK: headblock %d -> %d (%s)", s_cfg_mode, cfg_mode, mode_name(cfg_mode));
        s_cfg_mode = cfg_mode;
        s_L = -1.0f;
    }

    int eff = active ? cfg_mode : 0;
    // Automatic fallback: sphere -> line -> lean limit, when the reflected trace is not usable.
    if (eff == 1 || eff == 2) {
        if (!traces_ready()) eff = 3;
        else if (eff == 2 && !g_sphere.ok) eff = g_line.ok ? 1 : 3;
        else if (eff == 1 && !g_line.ok) eff = g_sphere.ok ? 2 : 3;
    }
    if (eff != s_eff) {
        if (s_eff != -1 || eff != 0) {
            hblog("HEADBLOCK: running %s (requested %s, %s)", mode_name(eff), mode_name(cfg_mode),
                  active ? "on foot" : "standing down: menu, vehicle or cutscene");
        }
        s_eff = eff;
        s_L = -1.0f;
    }
    g_eff_mode.store(eff, std::memory_order_relaxed);
    if (eff == 0 || eff == 3) {
        g_allow.store(-1.0f, std::memory_order_relaxed);
        if (eff == 3 && g_cfg.head_block_log > 1) {
            static uint32_t s_n3 = 0;
            if ((s_n3++ % (uint32_t)g_cfg.head_block_log) == 0u) {
                hblog("HEADBLOCK lean-limit head=(%.1f %.1f %.1f) limit=%.1f pushed=%.1f cm",
                      g_head_cx.load(), g_head_cy.load(), g_head_cz.load(), g_cfg.head_block_lean,
                      g_pushed.load());
            }
        }
        return;
    }
    if (!g_have_head.load(std::memory_order_relaxed)) return;

    const Vec3 B{g_body_x.load(), g_body_y.load(), g_body_z.load()};
    const Vec3 c{g_head_cx.load(), g_head_cy.load(), g_head_cz.load()};
    const float len = std::sqrt(c.x * c.x + c.y * c.y + c.z * c.z);
    const float r = g_cfg.head_block_radius;

    float L_raw = -1.0f;
    bool hit = false;
    Vec3 loc{}, imp{};
    if (len > 0.5f) {
        if (eff == 2) {
            // The sphere centre where the sweep stopped is the furthest the head centre can go.
            const Vec3 end{B.x + c.x, B.y + c.y, B.z + c.z};
            if (run_trace(g_sphere, B, end, r, ignore, n_ignore, &loc, &imp)) {
                hit = true;
                L_raw = dist(loc, B);
            }
        } else {
            // A ray to the head plus the radius; stop the head a radius short of the surface.
            const float k = (len + r) / len;
            const Vec3 end{B.x + c.x * k, B.y + c.y * k, B.z + c.z * k};
            if (run_trace(g_line, B, end, 0.0f, ignore, n_ignore, &loc, &imp)) {
                hit = true;
                L_raw = (std::max)(0.0f, dist(imp, B) - r);
            }
        }
    }

    // Tighten at once -- geometry must never be seen through. Loosen at headblockrelease cm/s, so
    // a hit that clears (a corner passed, a grazing edge) eases the head out instead of popping it.
    const float rel = g_cfg.head_block_release * ((dt > 0.0f && dt < 0.5f) ? dt : 0.0f);
    if (L_raw >= 0.0f && (s_L < 0.0f || L_raw < s_L)) {
        s_L = L_raw;
    } else if (s_L >= 0.0f) {
        s_L += rel;
        if (L_raw >= 0.0f && s_L > L_raw) s_L = L_raw;
        if (L_raw < 0.0f && s_L > len + r) s_L = -1.0f;
    }
    g_allow.store(s_L, std::memory_order_relaxed);

    if (hit != s_hit_prev) {
        s_hit_prev = hit;
        if (hit) hblog("HEADBLOCK: contact (%s) head |%.1f| cm from body, allowed %.1f cm", mode_name(eff), len, L_raw);
        else     hblog("HEADBLOCK: clear (%s), releasing from %.1f cm", mode_name(eff), s_L);
    }
    if (g_cfg.head_block_log > 1) {
        static uint32_t s_n = 0;
        if ((s_n++ % (uint32_t)g_cfg.head_block_log) == 0u) {
            hblog("HEADBLOCK %s body=(%.0f %.0f %.0f) head=(%.1f %.1f %.1f)|%.1f| hit=%d loc=(%.0f %.0f %.0f) "
                  "Lraw=%.1f L=%.1f pushed=%.1f cm r=%.1f ch=%d ignore=%d",
                  mode_name(eff), B.x, B.y, B.z, c.x, c.y, c.z, len, (int)hit, loc.x, loc.y, loc.z,
                  L_raw, s_L, g_pushed.load(), r, g_cfg.head_block_channel, n_ignore);
        }
    }
}

}  // namespace halo
