// reloadvr (fork feature, Experimental): the hidden reload's on-weapon ammo display and gesture_render_tick().
// Textual fragment, included by Gesture.cpp at namespace halo scope, after the tap/hold tracking. Moved verbatim; not compiled on its own.
// ---- THE ON-WEAPON AMMO DISPLAY (the hidden reload, 2026-09-07). The AR's counter is a
// material (MI_AssaultRifle_Display_*, a digit atlas T_..._AmmoCounter) on a display mesh; the
// count is a scalar parameter. The component is found once per weapon actor (material 0's name
// contains "Display"), its scalar parameters are logged once, and every one whose name looks
// like a count is written to 0 each frame while the hidden reload is pending.
namespace {
struct WdField { int32_t off; int kind; };   // kind: 0 float, 1 double, 2 int32, 3 byte
struct WpnDisplay { API::UObject* actor = nullptr; TrackedObject comp; std::vector<std::wstring> params; bool searched = false;
                    std::vector<WdField> fields; std::vector<WdField> hits; std::vector<int> cpd_hits; std::vector<std::wstring> param_hits; bool matched = false; bool dumped = false; };
WpnDisplay s_wd;
API::UObject* wd_material_at(API::UObject* comp, int32_t slot);
int32_t wd_material_count(API::UObject* comp);
void wd_scalar_params(API::UObject* mat, std::vector<std::wstring>& out, std::wstring& values);
// The display component's own Blueprint variables (its BPC_ classes) and its custom primitive
// data: listed once with values, and whichever equals the live round count is the one written.
void wd_list_fields(API::UObject* c) {
    s_wd.fields.clear();
    std::wstring line; int n = 0;
    for (API::UStruct* st = c->get_class(); st != nullptr && n < 120; st = st->get_super_struct()) {
        if (IsBadReadPtr(st, sizeof(void*))) break;
        const std::wstring sn = st->get_fname() ? st->get_fname()->to_string() : L"?";
        if (sn.rfind(L"BPC_", 0) != 0) break;   // engine classes: stop
        for (API::FField* f = st->get_child_properties(); f != nullptr && n < 120; f = f->get_next()) {
            if (IsBadReadPtr(f, sizeof(void*))) break;
            auto* fc = f->get_class(); const std::wstring tn = fc ? fc->get_name() : L"?";
            const auto* fn = f->get_fname(); const std::wstring nm = fn ? fn->to_string() : L"?";
            const int32_t off = static_cast<API::FProperty*>(f)->get_offset();
            const uint8_t* base = reinterpret_cast<const uint8_t*>(c);
            wchar_t b[128]; int kind = -1; double v = 0.0;
            if (tn == L"FloatProperty")  { kind = 0; v = *reinterpret_cast<const float*>(base + off); }
            else if (tn == L"DoubleProperty") { kind = 1; v = *reinterpret_cast<const double*>(base + off); }
            else if (tn == L"IntProperty")    { kind = 2; v = *reinterpret_cast<const int32_t*>(base + off); }
            else if (tn == L"ByteProperty")   { kind = 3; v = *reinterpret_cast<const uint8_t*>(base + off); }
            if (kind >= 0) { s_wd.fields.push_back({off, kind}); swprintf_s(b, L" %ls=%.2f", nm.c_str(), v); }
            else swprintf_s(b, L" %ls:%ls", nm.c_str(), tn.c_str());
            line += b; ++n;
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] WPNDISPLAY fields:%ls", line.c_str());
    // Custom primitive data: FCustomPrimitiveData { TArray<float> Data } at the property.
    if (auto* pcpd = c->get_class()->find_property(L"CustomPrimitiveData")) {
        struct TArr { const float* data; int32_t num; int32_t max; };
        const auto* a = reinterpret_cast<const TArr*>(reinterpret_cast<const uint8_t*>(c) + pcpd->get_offset());
        std::wstring cl;
        if (a->data != nullptr && a->num > 0 && a->num < 64 && !IsBadReadPtr(a->data, (size_t)a->num * 4)) for (int32_t i = 0; i < a->num; ++i) { wchar_t b[32]; swprintf_s(b, L" [%d]=%.2f", i, a->data[i]); cl += b; }
        API::get()->log_info("[Halo-CampE-UEVR] WPNDISPLAY custom primitive data (%d):%ls", a->data ? a->num : 0, cl.c_str());
    } else API::get()->log_info("[Halo-CampE-UEVR] WPNDISPLAY: no CustomPrimitiveData property");
}
double wd_field_read(API::UObject* c, const WdField& f) {
    const uint8_t* base = reinterpret_cast<const uint8_t*>(c) + f.off;
    switch (f.kind) { case 0: return *reinterpret_cast<const float*>(base); case 1: return *reinterpret_cast<const double*>(base); case 2: return *reinterpret_cast<const int32_t*>(base); default: return *reinterpret_cast<const uint8_t*>(base); }
}
void wd_field_write0(API::UObject* c, const WdField& f) {
    uint8_t* base = reinterpret_cast<uint8_t*>(c) + f.off;
    switch (f.kind) { case 0: *reinterpret_cast<float*>(base) = 0.0f; break; case 1: *reinterpret_cast<double*>(base) = 0.0; break; case 2: *reinterpret_cast<int32_t*>(base) = 0; break; default: *reinterpret_cast<uint8_t*>(base) = 0; break; }
}
// Which field or primitive float carries the count: the ones equal to the live rounds (>= 2).
void wd_match(API::UObject* c) {
    if (s_wd.matched) return;
    const auto* r = rounds_field(); if (r == nullptr) return;
    const int rounds = (int)*r; if (rounds < 2) return;
    std::wstring line;
    for (const auto& f : s_wd.fields) { const double v = wd_field_read(c, f); if (std::fabs(v - rounds) < 0.01) { s_wd.hits.push_back(f); wchar_t b[48]; swprintf_s(b, L" field@0x%X", (unsigned)f.off); line += b; } }
    if (auto* pcpd = c->get_class()->find_property(L"CustomPrimitiveData")) {
        struct TArr { const float* data; int32_t num; int32_t max; };
        const auto* a = reinterpret_cast<const TArr*>(reinterpret_cast<const uint8_t*>(c) + pcpd->get_offset());
        if (a->data != nullptr && a->num > 0 && a->num < 64 && !IsBadReadPtr(a->data, (size_t)a->num * 4)) for (int32_t i = 0; i < a->num; ++i) if (std::fabs(a->data[i] - rounds) < 0.01) { s_wd.cpd_hits.push_back(i); wchar_t b[32]; swprintf_s(b, L" cpd[%d]", i); line += b; }
    }
    // Scalar parameters whose VALUE is the count, on any of the component's materials.
    const int32_t n = wd_material_count(c);
    for (int32_t slot = 0; slot < n; ++slot) {
        auto* m = wd_material_at(c, slot);
        if (m == nullptr || IsBadReadPtr(m, sizeof(void*))) continue;
        std::vector<std::wstring> names; std::wstring values;
        wd_scalar_params(m, names, values);
        // values is " name=v name=v ..."; re-read each value by name from the string.
        {   // texture parameters (a dynamic display texture would show here)
            std::wstring tl;
            for (int hop = 0; hop < 4 && m != nullptr; ++hop) {
                auto* mc = m->get_class(); if (mc == nullptr) break;
                if (auto* prop = mc->find_property(L"TextureParameterValues")) {
                    auto* fc = prop->get_class();
                    if (fc != nullptr && fc->get_name() == L"ArrayProperty") {
                        auto* inner = static_cast<API::FArrayProperty*>(prop)->get_inner();
                        if (inner && inner->get_class() && inner->get_class()->get_name() == L"StructProperty") {
                            auto* st = static_cast<API::FStructProperty*>(inner)->get_struct();
                            const int32_t stride = st ? st->get_properties_size() : 0;
                            auto* pinfo = st ? st->find_property(L"ParameterInfo") : nullptr; auto* pval = st ? st->find_property(L"ParameterValue") : nullptr;
                            int32_t name_off = -1;
                            if (pinfo && pinfo->get_class() && pinfo->get_class()->get_name() == L"StructProperty") { auto* ist = static_cast<API::FStructProperty*>(pinfo)->get_struct(); auto* pn = ist ? ist->find_property(L"Name") : nullptr; if (pn) name_off = pinfo->get_offset() + pn->get_offset(); }
                            struct TArr { uint8_t* data; int32_t num; int32_t max; };
                            const auto* a = reinterpret_cast<const TArr*>(reinterpret_cast<uint8_t*>(m) + prop->get_offset());
                            if (stride > 0 && name_off >= 0 && pval && a->data && a->num > 0 && a->num < 64 && !IsBadReadPtr(a->data, (size_t)a->num * stride)) {
                                for (int32_t i = 0; i < a->num; ++i) {
                                    const uint8_t* e = a->data + (size_t)i * stride;
                                    auto* tex = *reinterpret_cast<API::UObject* const*>(e + pval->get_offset());
                                    tl += L" " + reinterpret_cast<const API::FName*>(e + name_off)->to_string() + L"=" + ((tex && !IsBadReadPtr(tex, sizeof(void*)) && tex->get_fname()) ? tex->get_fname()->to_string() : L"null");
                                }
                            }
                        }
                    }
                }
                auto* pp = mc->find_property(L"Parent"); if (!pp) break;
                auto** parent = reinterpret_cast<API::UObject**>(reinterpret_cast<uint8_t*>(m) + pp->get_offset()); if (IsBadReadPtr(parent, sizeof(void*))) break;
                m = *parent;
            }
            API::get()->log_info("[Halo-CampE-UEVR] WPNDISPLAY slot %d textures:%ls", slot, tl.empty() ? L" (none)" : tl.c_str());
        }
        for (const auto& nm : names) {
            const std::wstring key = L" " + nm + L"=";
            const size_t at = values.find(key); if (at == std::wstring::npos) continue;
            const double v = _wtof(values.c_str() + at + key.size());
            if (std::fabs(v - rounds) < 0.01) { s_wd.param_hits.push_back(nm); line += L" param:" + nm; }
        }
    }
    s_wd.matched = true;
    API::get()->log_info("[Halo-CampE-UEVR] WPNDISPLAY count match at rounds %d:%ls", rounds, line.empty() ? L" NONE" : line.c_str());
}
std::wstring wlower(std::wstring w) { for (auto& ch : w) ch = (wchar_t)towlower(ch); return w; }
API::UObject* wd_material_at(API::UObject* comp, int32_t slot) {
    auto* cls = comp->get_class(); if (cls == nullptr || cls->find_function(L"GetMaterial") == nullptr) return nullptr;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    *reinterpret_cast<int32_t*>(p) = slot;
    comp->call_function(L"GetMaterial", p);
    return *reinterpret_cast<API::UObject**>(p + 8);
}
int32_t wd_material_count(API::UObject* comp) {
    auto* cls = comp->get_class(); if (cls == nullptr || cls->find_function(L"GetNumMaterials") == nullptr) return 0;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    comp->call_function(L"GetNumMaterials", p);
    const int32_t n = *reinterpret_cast<int32_t*>(p);
    return (n < 0 || n > 16) ? 0 : n;
}
API::UObject* wd_material0(API::UObject* comp) { return wd_material_at(comp, 0); }
// Scalar parameter names of a material instance, walking Parent until one has entries.
void wd_scalar_params(API::UObject* mat, std::vector<std::wstring>& out, std::wstring& values) {
    for (int hop = 0; hop < 6 && mat != nullptr && !IsBadReadPtr(mat, sizeof(void*)); ++hop) {
        auto* cls = mat->get_class(); if (cls == nullptr) return;
        auto* prop = cls->find_property(L"ScalarParameterValues");
        if (prop != nullptr) {
            auto* fc = prop->get_class();
            if (fc != nullptr && fc->get_name() == L"ArrayProperty") {
                auto* inner = static_cast<API::FArrayProperty*>(prop)->get_inner();
                auto* ifc = inner ? inner->get_class() : nullptr;
                if (ifc != nullptr && ifc->get_name() == L"StructProperty") {
                    auto* st = static_cast<API::FStructProperty*>(inner)->get_struct();
                    const int32_t stride = st ? st->get_properties_size() : 0;
                    auto* pinfo = st ? st->find_property(L"ParameterInfo") : nullptr;
                    auto* pval  = st ? st->find_property(L"ParameterValue") : nullptr;
                    int32_t name_off = -1;
                    if (pinfo != nullptr && pinfo->get_class() != nullptr && pinfo->get_class()->get_name() == L"StructProperty") {
                        auto* ist = static_cast<API::FStructProperty*>(pinfo)->get_struct();
                        auto* pn = ist ? ist->find_property(L"Name") : nullptr;
                        if (pn != nullptr) name_off = pinfo->get_offset() + pn->get_offset();
                    }
                    struct TArr { uint8_t* data; int32_t num; int32_t max; };
                    const auto* arr = reinterpret_cast<const TArr*>(reinterpret_cast<uint8_t*>(mat) + prop->get_offset());
                    if (stride > 0 && name_off >= 0 && arr->data != nullptr && arr->num > 0 && arr->num < 256 && !IsBadReadPtr(arr->data, (size_t)arr->num * stride)) {
                        for (int32_t i = 0; i < arr->num; ++i) {
                            const uint8_t* e = arr->data + (size_t)i * stride;
                            const std::wstring nm = reinterpret_cast<const API::FName*>(e + name_off)->to_string();
                            const float v = pval ? *reinterpret_cast<const float*>(e + pval->get_offset()) : 0.0f;
                            out.push_back(nm);
                            wchar_t b[96]; swprintf_s(b, L" %ls=%.3f", nm.c_str(), v); values += b;
                        }
                        return;
                    }
                }
            }
        }
        auto* pp = cls->find_property(L"Parent");
        if (pp == nullptr) return;
        auto** parent = reinterpret_cast<API::UObject**>(reinterpret_cast<uint8_t*>(mat) + pp->get_offset());
        if (IsBadReadPtr(parent, sizeof(void*))) return;
        mat = *parent;
    }
}
void wd_find() {
    auto* actor = fp_weapon_actor();
    if (actor == s_wd.actor && (s_wd.searched)) return;
    s_wd = WpnDisplay{}; s_wd.actor = actor; s_wd.searched = true;
    if (actor == nullptr) return;
    // Every slot of every component. A dynamic instance (class MaterialInstanceDynamic, named
    // like MaterialInstanceDynamic_N) is where a game-set count lives; a name with "display"
    // marks the readout's static instance. The first component carrying either is the one.
    weapon_components([&](API::UObject* c) {
        const int32_t n = wd_material_count(c);
        for (int32_t slot = 0; slot < n; ++slot) {
            auto* m = wd_material_at(c, slot);
            if (m == nullptr || IsBadReadPtr(m, sizeof(void*)) || m->get_fname() == nullptr) continue;
            const std::wstring mcls = class_name_of(m);
            const std::wstring mn = wlower(m->get_fname()->to_string());
            const bool dyn = mcls.find(L"Dynamic") != std::wstring::npos;
            if (!dyn && mn.find(L"display") == std::wstring::npos) continue;
            std::vector<std::wstring> params; std::wstring values;
            wd_scalar_params(m, params, values);
            API::get()->log_info("[Halo-CampE-UEVR] WPNDISPLAY %ls (%ls) slot %d on %ls (%ls): %d scalar param(s)%ls", m->get_fname()->to_string().c_str(), mcls.c_str(), slot, c->get_fname() ? c->get_fname()->to_string().c_str() : L"?", class_name_of(c).c_str(), (int)params.size(), values.c_str());
            if (s_wd.comp.get() == nullptr || (dyn && s_wd.params.empty())) { s_wd.comp.set(c); s_wd.params = params; }
        }
        return true;
    });
    if (auto* c = s_wd.comp.get()) wd_list_fields(c);
    if (s_wd.comp.get() == nullptr) API::get()->log_info("[Halo-CampE-UEVR] WPNDISPLAY: no display material on this weapon");
}
bool wd_param_is_count(const std::wstring& nm) {
    const std::wstring lo = wlower(nm);
    return lo.find(L"ammo") != std::wstring::npos || lo.find(L"count") != std::wstring::npos || lo.find(L"round") != std::wstring::npos
        || lo.find(L"clip") != std::wstring::npos || lo.find(L"bullet") != std::wstring::npos || lo.find(L"mag") != std::wstring::npos;
}
void wd_write_zero() {
    wd_find();
    auto* c = s_wd.comp.get(); if (c == nullptr) return;
    wd_match(c);
    for (const auto& f : s_wd.hits) wd_field_write0(c, f);
    if (!s_wd.cpd_hits.empty()) {
        if (auto* fn = c->get_class()->find_function(L"SetCustomPrimitiveDataFloat")) {
            auto* pi = fn->find_property(L"DataIndex"); auto* pv = fn->find_property(L"Value");
            if (pi != nullptr && pv != nullptr) for (int idx : s_wd.cpd_hits) {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                *reinterpret_cast<int32_t*>(p + pi->get_offset()) = idx;
                *reinterpret_cast<float*>(p + pv->get_offset()) = 0.0f;
                c->call_function(L"SetCustomPrimitiveDataFloat", p);
            }
        }
    }
    auto* cls = c->get_class(); auto* fn = cls ? cls->find_function(L"SetScalarParameterValueOnMaterials") : nullptr; if (fn == nullptr) return;
    auto* pn = fn->find_property(L"ParameterName"); auto* pv = fn->find_property(L"ParameterValue"); if (pn == nullptr || pv == nullptr) return;
    for (const auto& nm : s_wd.params) {
        const bool hit = std::find(s_wd.param_hits.begin(), s_wd.param_hits.end(), nm) != s_wd.param_hits.end();
        if (!hit && !wd_param_is_count(nm)) continue;
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        const API::FName fnm = make_fname(nm.c_str());
        memcpy(p + pn->get_offset(), &fnm, sizeof(API::FName));
        *reinterpret_cast<float*>(p + pv->get_offset()) = 0.0f;
        c->call_function(L"SetScalarParameterValueOnMaterials", p);
    }
}
} // namespace
// ---- THE READOUT THROUGH THE ANIMATION (2026-09-07): no dynamic material, no variables, no
// primitive data on the display component, so the count must reach the digits through the
// weapon's anim instance. Every scalar anim variable equal to the live count is a candidate;
// each is written 0 on the game tick and on the render path, and once a second the readback
// says whether the game re-set it (which tells where in the frame the copy happens).
namespace {
std::vector<AnimVar> s_ad_hits; bool s_ad_matched = false; API::UObject* s_ad_ai = nullptr; long long s_ad_said = 0;
void mpc_probe(API::UObject* ctx, int rounds);
void ad_reset_if_new(API::UObject* animbp) { if (animbp != s_ad_ai) { s_ad_ai = animbp; s_ad_hits.clear(); s_ad_matched = false; } }
void ad_match(API::UObject* animbp) {
    if (s_ad_matched) return;
    const auto* r = rounds_field(); if (r == nullptr) return;
    const int rounds = (int)*r; if (rounds < 2) return;
    std::vector<AnimVar> vars; av_collect(animbp, vars);
    std::wstring line, named;
    for (const auto& v : vars) {
        const std::wstring lo = wlower(v.name);
        const bool keyword = lo.find(L"ammo") != std::wstring::npos || lo.find(L"round") != std::wstring::npos || lo.find(L"count") != std::wstring::npos || lo.find(L"clip") != std::wstring::npos || lo.find(L"digit") != std::wstring::npos || lo.find(L"display") != std::wstring::npos;
        if (keyword) { wchar_t b[128]; swprintf_s(b, L" %ls=%.2f", v.name.c_str(), v.v); named += b; }
        if (std::fabs(v.v - rounds) < 0.01) { s_ad_hits.push_back(v); wchar_t b[128]; swprintf_s(b, L" %ls(%ls@0x%X)", v.name.c_str(), v.cls.c_str(), (unsigned)v.off); line += b; }
    }
    s_ad_matched = true;
    std::wstring all; for (const auto& v : vars) { wchar_t b[128]; swprintf_s(b, L" %ls=%.2f", v.name.c_str(), v.v); all += b; }
    API::get()->log_info("[Halo-CampE-UEVR] ANIMREADOUT %zu vars on %ls:%ls", vars.size(), class_name_of(animbp).c_str(), all.c_str());
    API::get()->log_info("[Halo-CampE-UEVR] ANIMREADOUT named:%ls", named.empty() ? L" (none)" : named.c_str());
    API::get()->log_info("[Halo-CampE-UEVR] ANIMREADOUT equal to rounds %d:%ls", rounds, line.empty() ? L" NONE" : line.c_str());
    mpc_probe(fp_weapon_actor(), rounds);
}
void ad_write(API::UObject* animbp, const AnimVar& v, double val) {
    uint8_t* p = reinterpret_cast<uint8_t*>(animbp) + v.off;
    switch (v.kind) { case 1: case 2: *p = (uint8_t)val; break; case 3: *reinterpret_cast<int32_t*>(p) = (int32_t)val; break; case 4: *reinterpret_cast<float*>(p) = (float)val; break; case 5: *reinterpret_cast<double*>(p) = val; break; }
}
// ---- MATERIAL PARAMETER COLLECTIONS (2026-09-07): the last channel the count could reach the
// digits through. Every collection's scalar parameters are read live through
// KismetMaterialLibrary.GetScalarParameterValue, listed once, and any equal to the count named.
void mpc_probe(API::UObject* ctx, int rounds) {
    static bool s_done = false; if (s_done || ctx == nullptr) return; s_done = true;
    auto* kml = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.KismetMaterialLibrary");
    auto* fn = kml ? kml->find_function(L"GetScalarParameterValue") : nullptr;
    auto* cdo = kml ? kml->get_class_default_object() : nullptr;
    if (fn == nullptr || cdo == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] MPC: KismetMaterialLibrary.GetScalarParameterValue not found"); return; }
    auto* pctx = fn->find_property(L"WorldContextObject"); auto* pcol = fn->find_property(L"Collection"); auto* pnm = fn->find_property(L"ParameterName"); auto* pret = fn->find_property(L"ReturnValue");
    if (!pctx || !pcol || !pnm || !pret) { API::get()->log_info("[Halo-CampE-UEVR] MPC: parameter layout not found"); return; }
    auto* arr = API::get()->get_uobject_array(); if (arr == nullptr) return;
    const int32_t nn = arr->get_object_count(); int ncol = 0;
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"MaterialParameterCollection") continue;
        ++ncol;
        std::vector<std::wstring> names;
        if (auto* prop = o->get_class()->find_property(L"ScalarParameters")) {
            auto* fc = prop->get_class();
            if (fc && fc->get_name() == L"ArrayProperty") {
                auto* inner = static_cast<API::FArrayProperty*>(prop)->get_inner();
                if (inner && inner->get_class() && inner->get_class()->get_name() == L"StructProperty") {
                    auto* st = static_cast<API::FStructProperty*>(inner)->get_struct();
                    const int32_t stride = st ? st->get_properties_size() : 0;
                    auto* pn = st ? st->find_property(L"ParameterName") : nullptr;
                    struct TArr { uint8_t* data; int32_t num; int32_t max; };
                    const auto* a = reinterpret_cast<const TArr*>(reinterpret_cast<uint8_t*>(o) + prop->get_offset());
                    if (stride > 0 && pn && a->data && a->num > 0 && a->num < 256 && !IsBadReadPtr(a->data, (size_t)a->num * stride))
                        for (int32_t k = 0; k < a->num; ++k) names.push_back(reinterpret_cast<const API::FName*>(a->data + (size_t)k * stride + pn->get_offset())->to_string());
                }
            }
        }
        std::wstring line, hits;
        for (const auto& nm : names) {
            alignas(16) uint8_t q[128] = {0};
            *reinterpret_cast<API::UObject**>(q + pctx->get_offset()) = ctx;
            *reinterpret_cast<API::UObject**>(q + pcol->get_offset()) = o;
            const API::FName fnm = make_fname(nm.c_str()); memcpy(q + pnm->get_offset(), &fnm, sizeof(API::FName));
            cdo->call_function(L"GetScalarParameterValue", q);
            const float v = *reinterpret_cast<const float*>(q + pret->get_offset());
            wchar_t b[128]; swprintf_s(b, L" %ls=%.2f", nm.c_str(), v); line += b;
            if (std::fabs(v - rounds) < 0.01) hits += L" " + nm;
        }
        API::get()->log_info("[Halo-CampE-UEVR] MPC %ls (%d scalar):%ls%ls%ls", o->get_fname() ? o->get_fname()->to_string().c_str() : L"?", (int)names.size(), line.c_str(), hits.empty() ? L"" : L"  <-- EQUAL TO ROUNDS:", hits.c_str());
    }
    API::get()->log_info("[Halo-CampE-UEVR] MPC: %d collection(s) listed at rounds %d", ncol, rounds);
}
void ad_write_zero(bool render_path) {
    auto* animbp = reload_weapon_anim_instance(); if (animbp == nullptr) return;
    ad_reset_if_new(animbp);
    ad_match(animbp);
    const long long nowt = now_ticks();
    const bool say = nowt - s_ad_said > ms_to_ticks(1000);
    std::wstring line;
    // The explicit ammo frame, on both paths: the tick write alone left the readout at full, so
    // the game re-sets it after our tick; the render-path write lands after the game's.
    if (auto* pf = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame")) {
        if (!IsBadWritePtr(pf, sizeof(int32_t))) { if (say) { wchar_t b[64]; swprintf_s(b, L" ExplicitFrame=%d", *pf); line += b; } *pf = 0; }
    }
    if (s_ad_hits.empty()) { if (say && !line.empty()) { s_ad_said = nowt; API::get()->log_info("[Halo-CampE-UEVR] ANIMREADOUT %s: before the write%ls (0 written)", render_path ? "render" : "tick", line.c_str()); } return; }
    for (const auto& v : s_ad_hits) {
        const double before = av_read(animbp, v.off, v.kind);
        if (say) { wchar_t b[96]; swprintf_s(b, L" %ls=%.1f", v.name.c_str(), before); line += b; }
        ad_write(animbp, v, 0.0);
    }
    if (say) { s_ad_said = nowt; API::get()->log_info("[Halo-CampE-UEVR] ANIMREADOUT %s: before the write%ls (0 written)", render_path ? "render" : "tick", line.c_str()); }
}
} // namespace
void gesture_render_tick() {
    if (!g_wristhud_hide_cradle.load(std::memory_order_relaxed)) return;
    wd_write_zero();
    ad_write_zero(true);
}
